// ═══════════════════════════════════════════════════════════════════════════════
// rtsp_camera_combined.cpp
//
// Smart camera system for Raspberry Pi 4:
//   - Streams live video by PUBLISHING into MediaMTX (a standalone RTSP
//     media server), instead of hosting an RTSP server inside this app.
//     (gst-rtsp-server has a known bug with dynamic pipelines + client
//     seek that causes "multiple player cannot be played" errors in VLC.
//     Publishing into MediaMTX via rtspclientsink avoids that entirely.)
//   - Runs OpenCV face detection (Haar cascade) on every frame.
//   - Only writes video to disk while a face is detected, with a tail
//     period after the face disappears to avoid choppy clips.
//   - Logs every detection/recording event to a file.
//
// REQUIREMENT: MediaMTX must already be running on the Pi before this
// app is started (the app publishes into it, it does not start it).
//
// Build:
//   g++ rtsp_camera_combined.cpp -o app \
//       $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 opencv4) \
//       -lpthread
// ═══════════════════════════════════════════════════════════════════════════════

// ── GStreamer core headers ────────────────────────────────────────────────────
#include <gst/gst.h>                  // Core GStreamer API: pipelines, elements, buffers
#include <gst/app/gstappsink.h>       // appsink: pulls frames OUT of a GStreamer pipeline into C++
#include <gst/app/gstappsrc.h>        // appsrc: pushes frames FROM C++ back INTO a pipeline

// ── OpenCV ─────────────────────────────────────────────────────────────────────
#include <opencv2/opencv.hpp>         // cv::Mat, cv::CascadeClassifier, drawing functions, etc.

// ── C++ standard library ────────────────────────────────────────────────────────
#include <iostream>                   // std::cout / std::cerr for console output
#include <fstream>                    // std::ofstream for writing the log file
#include <sstream>                    // std::ostringstream for building formatted strings
#include <iomanip>                    // std::put_time, std::setw, std::setfill (date/number formatting)
#include <chrono>                     // std::chrono::steady_clock for measuring elapsed time
#include <thread>                     // std::thread to run the OpenCV worker on its own thread
#include <mutex>                      // std::mutex to protect data shared between threads
#include <condition_variable>         // std::condition_variable to wake the worker thread on new frames
#include <queue>                      // std::queue used as the frame buffer between threads
#include <atomic>                     // std::atomic<bool> for the thread-safe "keep running" flag
#include <csignal>                    // std::signal to catch Ctrl-C (SIGINT)
#include <ctime>                      // std::time, std::localtime for timestamps

// ═══════════════════════════════════════════════════════════════════════════════
// CONFIGURATION — change these constants to tune behavior
// ═══════════════════════════════════════════════════════════════════════════════
static const char *LOGO_PATH        = "/home/bobobo/rtsp_camera/logo/penguin-svgrepo.svg";
                                       // path to the logo image overlaid on every frame
static const char *RECORDING_DIR    = "/home/bobobo/rtsp_camera/recordings";
                                       // directory where .mp4 recordings are saved
static const char *LOG_PATH         = "/home/bobobo/rtsp_camera/recordings/detect.log";
                                       // text log file: one line per event (start/stop/detect)
static const char *CASCADE_PATH     =
    "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml";
                                       // Haar cascade XML for frontal (face-on) detection
static const char *PROFILE_PATH     =
    "/usr/share/opencv4/haarcascades/haarcascade_profileface.xml";
                                       // Haar cascade XML for side-profile faces, used to
                                       // cross-check frontal results and reduce false positives
static const char *MEDIAMTX_URL     = "rtsp://127.0.0.1:8554/camera";
                                       // RTSP URL of the locally running MediaMTX server that
                                       // this app PUBLISHES its stream into

static const int   VIDEO_WIDTH      = 1280;   // frame width in pixels
static const int   VIDEO_HEIGHT     = 720;    // frame height in pixels
static const int   VIDEO_FPS        = 30;     // target frames per second
static const int   HW_BITRATE       = 2000000;// hardware encoder bitrate, in BITS per second (not kbps)
static const int   SPLIT_SECONDS    = 20;     // each recorded .mp4 segment is cut after N seconds
static const int   DETECT_EVERY     = 10;     // run face detection every N frames (saves CPU);
                                               // at 30fps this is roughly 3 detections per second
static const int   QUEUE_MAX        = 3;      // max number of frames buffered between
                                               // the camera thread and the OpenCV thread
static const int   RECORD_TAIL_SEC  = 5;      // keep recording for N more seconds after the
                                               // last face disappears, to avoid cutting clips short
static const int   CONFIRM_FRAMES   = 3;      // number of CONSECUTIVE detection passes that must
                                               // find a face before recording actually starts;
                                               // filters out single-frame false positives

// ═══════════════════════════════════════════════════════════════════════════════
// RECORDING STATE
// ═══════════════════════════════════════════════════════════════════════════════

// Two possible states: not currently writing video, or actively writing video.
enum class RecordState { IDLE, RECORDING };

static RecordState  g_rec_state = RecordState::IDLE;
                                       // current recording state, read/written from multiple places
static GstElement  *g_splitmux  = nullptr;
                                       // pointer to the splitmuxsink element (the thing that
                                       // actually writes .mp4 files to disk)
static GstElement  *g_valve     = nullptr;
                                       // pointer to the valve element that gates the recording
                                       // branch on/off without needing to start/stop the pipeline
static std::mutex   g_rec_mutex;      // protects g_rec_state from concurrent access
static std::chrono::steady_clock::time_point g_last_face_time;
                                       // timestamp of the last frame where a face was seen;
                                       // used to compute the RECORD_TAIL_SEC countdown

// ═══════════════════════════════════════════════════════════════════════════════
// SHARED STATE BETWEEN THREADS
// ═══════════════════════════════════════════════════════════════════════════════

// One captured camera frame, carried from the GStreamer thread to the OpenCV thread.
struct Frame {
    cv::Mat      mat;       // the actual pixel data as an OpenCV BGR image
    GstClockTime pts;       // original GStreamer presentation timestamp (preserved so the
                             // downstream pipeline stays in sync)
    GstClockTime duration;  // original frame duration (1/fps, in nanoseconds)
};

static std::queue<Frame>       g_frame_queue; // FIFO buffer of frames waiting to be processed
static std::mutex              g_queue_mutex; // protects g_frame_queue from concurrent access
static std::condition_variable g_queue_cv;    // lets the OpenCV thread sleep until a frame arrives
                                               // instead of busy-waiting / polling
static std::atomic<bool>       g_running{true};
                                               // global "keep running" flag; set to false on Ctrl-C,
                                               // checked by every loop in every thread
static GMainLoop               *g_main_loop = nullptr;
                                               // GLib main loop that drives GStreamer's event handling
static GstElement              *g_appsrc    = nullptr;
                                               // the appsrc element inside the encoding pipeline that
                                               // the OpenCV thread pushes processed frames into

// ═══════════════════════════════════════════════════════════════════════════════
// CTRL-C HANDLER
// ═══════════════════════════════════════════════════════════════════════════════
static void on_sigint(int) {
    g_running = false;          // tell every loop (main, OpenCV thread) to stop
    g_queue_cv.notify_all();    // wake up the OpenCV thread immediately if it's
                                 // currently blocked waiting for a new frame
    if (g_main_loop) g_main_loop_quit(g_main_loop);
                                 // unblock g_main_loop_run() in main() so cleanup can run
}

// ═══════════════════════════════════════════════════════════════════════════════
// LOGGING HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

// Returns the current local time formatted as "YYYY-MM-DD HH:MM:SS".
static std::string now_str() {
    auto t  = std::time(nullptr);             // current time as seconds-since-epoch
    auto tm = *std::localtime(&t);             // convert to local calendar time (respects
                                                // the system timezone, e.g. Asia/Ho_Chi_Minh)
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S"); // format into the desired string layout
    return oss.str();
}

// Writes one timestamped line to both the console and the log file.
static void write_log(const std::string &msg) {
    std::string line = "[" + now_str() + "] " + msg;
    std::cout << line << "\n";                       // print to terminal
    std::ofstream f(LOG_PATH, std::ios::app);         // open log file in append mode
    if (f) f << line << "\n";                         // write the line if the file opened OK
}

// ═══════════════════════════════════════════════════════════════════════════════
// RECORDING CONTROL VIA THE VALVE ELEMENT
// ═══════════════════════════════════════════════════════════════════════════════
//
// Why use a "valve" element instead of pausing/stopping the pipeline?
//   - Stopping/starting GStreamer elements mid-pipeline is slow and can cause
//     glitches or re-negotiation of caps.
//   - A "valve" is a tiny element designed exactly for this: when drop=TRUE
//     it silently discards every buffer that passes through it; when
//     drop=FALSE it lets buffers through unchanged. Toggling it is instant
//     and doesn't disturb the rest of the running pipeline.

// Begin recording: open the valve so encoded frames start reaching splitmuxsink.
static void start_recording() {
    std::lock_guard<std::mutex> lock(g_rec_mutex); // lock to avoid racing with stop_recording()
    if (g_rec_state == RecordState::RECORDING) return; // already recording, nothing to do
    if (!g_valve) return;                              // valve not ready yet, bail out safely

    g_object_set(g_valve, "drop", FALSE, nullptr); // open the valve: frames now flow through
    g_rec_state = RecordState::RECORDING;          // update our tracked state
    write_log("RECORDING START - face detected");
}

// Stop recording: close the valve and cleanly finalize the current .mp4 file.
static void stop_recording() {
    std::lock_guard<std::mutex> lock(g_rec_mutex); // lock to avoid racing with start_recording()
    if (g_rec_state == RecordState::IDLE) return;  // already idle, nothing to do
    if (!g_valve || !g_splitmux) return;           // elements not ready, bail out safely

    g_object_set(g_valve, "drop", TRUE, nullptr);  // close the valve: stop feeding new frames in

    // Send an End-Of-Stream event directly into splitmuxsink's input pad.
    // This tells splitmuxsink to finalize (write the moov atom for) the
    // currently-open .mp4 file so it's a valid, playable video file.
    GstPad *sink_pad = gst_element_get_static_pad(g_splitmux, "sink");
    if (sink_pad) {
        gst_pad_send_event(sink_pad, gst_event_new_eos()); // push the EOS event
        gst_object_unref(sink_pad);                        // release our pad reference
    }

    g_rec_state = RecordState::IDLE;
    write_log("RECORDING STOP  - no face for " +
              std::to_string(RECORD_TAIL_SEC) + "s");
}

// ═══════════════════════════════════════════════════════════════════════════════
// APPSINK CALLBACK — pulls frames OUT of the camera pipeline into C++
// ═══════════════════════════════════════════════════════════════════════════════
//
// This callback runs on a GStreamer streaming thread and must return FAST.
// It only copies the frame into a queue; the actual heavy lifting (face
// detection, drawing) happens later on the separate OpenCV thread.
static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer) {
    // Pull the next available sample (buffer + caps) from appsink.
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR; // nothing available / error pulling it

    GstBuffer *buf = gst_sample_get_buffer(sample); // get the raw pixel buffer
    GstMapInfo map;
    // Map the buffer's memory so we can read its bytes directly.
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_OK; // mapping failed; just skip this frame, don't crash
    }

    {
        std::unique_lock<std::mutex> lock(g_queue_mutex);
        if ((int)g_frame_queue.size() < QUEUE_MAX) {
            // Wrap the raw pixel data in a cv::Mat, then .clone() it.
            // clone() is mandatory here: map.data is only valid while the
            // buffer stays mapped, so we must make our own copy before
            // unmapping it below.
            Frame f;
            f.mat      = cv::Mat(VIDEO_HEIGHT, VIDEO_WIDTH,
                                 CV_8UC3, map.data).clone(); // BGR, 3 bytes/pixel
            f.pts      = GST_BUFFER_PTS(buf);      // preserve original timestamp
            f.duration = GST_BUFFER_DURATION(buf); // preserve original duration
            g_frame_queue.push(std::move(f));      // move (not copy) into the queue
            g_queue_cv.notify_one();               // wake the OpenCV thread if it's sleeping
        }
        // If the queue is already full, we silently DROP this frame instead
        // of blocking. This keeps the camera pipeline running smoothly even
        // if OpenCV is momentarily slower than the camera's frame rate.
    }

    gst_buffer_unmap(buf, &map); // release the memory mapping (required after gst_buffer_map)
    gst_sample_unref(sample);    // release the sample
    return GST_FLOW_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// OPENCV WORKER THREAD — runs face detection and feeds the encoding pipeline
// ═══════════════════════════════════════════════════════════════════════════════
static void opencv_thread(cv::CascadeClassifier *cascade,  // frontal-face cascade
                          cv::CascadeClassifier *profile)  // side-profile cascade
{
    int frame_count = 0;               // total frames processed so far (used for DETECT_EVERY)
    std::vector<cv::Rect> last_faces;  // most recent detection result, reused on frames
                                        // where we skip the (expensive) detection step
    bool was_recording = false;        // this thread's own view of whether we're recording
    int  confirm_count = 0;            // consecutive detection passes that found a face

    while (g_running) {
        Frame f;

        // ── Wait for the next frame from the queue ──────────────────────────
        {
            std::unique_lock<std::mutex> lock(g_queue_mutex);
            // Sleep until either a frame is available OR we're shutting down.
            g_queue_cv.wait(lock, [] {
                return !g_frame_queue.empty() || !g_running;
            });
            if (!g_running && g_frame_queue.empty()) break; // nothing left, exit the thread
            f = std::move(g_frame_queue.front()); // take ownership of the front frame
            g_frame_queue.pop();
        }

        // ── Run face detection, but only every DETECT_EVERY frames ─────────
        // Frames in between reuse last_faces — this is a big CPU saver since
        // Haar cascade detection is the most expensive operation per frame.
        if (frame_count % DETECT_EVERY == 0) {

            // Step 1: convert to grayscale — Haar cascades only work on
            // single-channel images, not color.
            cv::Mat gray, small;
            cv::cvtColor(f.mat, gray, cv::COLOR_BGR2GRAY);

            // Step 2: equalize the histogram to improve contrast, which
            // helps detection in poor or uneven lighting.
            cv::equalizeHist(gray, gray);

            // Step 3: downscale to half resolution before detecting.
            // detectMultiScale runs on 640x360 instead of 1280x720, which
            // is roughly 4x fewer pixels and therefore much faster.
            cv::resize(gray, small,
                       cv::Size(VIDEO_WIDTH/2, VIDEO_HEIGHT/2));

            // Step 4: run the frontal-face Haar cascade on the downscaled image.
            std::vector<cv::Rect> faces_small;
            cascade->detectMultiScale(
                small,
                faces_small,
                1.05,             // scaleFactor: how much the image is shrunk at each
                                   // scan scale; smaller = more thorough but slower
                9,                 // minNeighbors: how many overlapping detections are
                                   // required before something counts as a real face;
                                   // higher = fewer false positives
                0,                 // flags: unused (legacy parameter)
                cv::Size(60, 60),    // minSize (in the downscaled image == 120x120 real)
                cv::Size(350, 350)); // maxSize (== 700x700 real) — filters out giant
                                     // false-positive boxes covering head+shoulders

            // Step 5: merge overlapping detection boxes (a simple form of
            // non-maximum suppression).
            if (faces_small.size() > 1) {
                std::vector<int> w(faces_small.size(), 1); // equal weight per box
                cv::groupRectangles(faces_small, w, 1, 0.2); // eps=0.2: boxes within
                                                              // 20% size/position are merged
            }

            // Step 6: scale detected box coordinates back up to full resolution
            // (multiply by 2, since we detected on a half-size image).
            std::vector<cv::Rect> faces_full;
            for (auto &r : faces_small)
                faces_full.push_back(
                    cv::Rect(r.x*2, r.y*2, r.width*2, r.height*2));

            // Step 7: cross-check with the profile (side-face) cascade to
            // reduce false positives. The frontal cascade alone sometimes
            // mistakes random objects for faces; requiring the profile
            // cascade to also "see something" in that region adds confidence.
            last_faces.clear();
            if (!profile || faces_full.empty()) {
                // No profile cascade loaded, or nothing detected at all —
                // just use the raw frontal results.
                last_faces = faces_full;
            } else {
                for (auto &face : faces_full) {
                    // Crop out just the region of this detected face from
                    // the full-resolution grayscale image.
                    int x1 = std::max(0, face.x);
                    int y1 = std::max(0, face.y);
                    int x2 = std::min(VIDEO_WIDTH,  face.x + face.width);
                    int y2 = std::min(VIDEO_HEIGHT, face.y + face.height);
                    cv::Mat roi = gray(cv::Rect(x1, y1, x2-x1, y2-y1));

                    // Run the profile cascade on just that cropped region.
                    std::vector<cv::Rect> profile_faces;
                    profile->detectMultiScale(roi, profile_faces, 1.1, 3, 0,
                        cv::Size(20,20)); // smaller minSize since the ROI is already small

                    // Only keep this box if the profile cascade ALSO found
                    // something there — i.e. both cascades agree.
                    if (!profile_faces.empty())
                        last_faces.push_back(face);
                }
                // Fallback: if cross-checking eliminated everything, fall
                // back to the raw frontal results. This avoids missing real
                // faces just because someone is looking straight at the
                // camera (where the profile cascade naturally finds nothing).
                if (last_faces.empty() && !faces_full.empty())
                    last_faces = faces_full;
            }

            // ── Decide whether to start/stop recording ──────────────────────
            bool has_face = !last_faces.empty();

            if (has_face) {
                confirm_count++;                       // count this as one more "seen" pass
                g_last_face_time = std::chrono::steady_clock::now(); // reset the tail timer

                // Only START recording after CONFIRM_FRAMES consecutive
                // detection passes found a face — this filters out a single
                // stray frame of false-positive detection.
                if (!was_recording && confirm_count >= CONFIRM_FRAMES) {
                    start_recording();
                    was_recording = true;
                    write_log("Detected " + std::to_string(last_faces.size())
                              + " face(s) (confirmed x"
                              + std::to_string(confirm_count) + ")");
                }
            } else {
                confirm_count = 0; // reset the streak — a face must be seen
                                   // CONFIRM_FRAMES times again from scratch

                if (was_recording) {
                    // Compute how long it's been since we last saw a face.
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::steady_clock::now() - g_last_face_time).count();

                    // Only stop recording once the tail period has fully
                    // elapsed, so a brief gap (person stepping out of frame
                    // and back) doesn't fragment the recording.
                    if (elapsed >= RECORD_TAIL_SEC) {
                        stop_recording();
                        was_recording = false;
                        write_log("No face detected");
                    }
                }
            }
        }
        frame_count++; // advance regardless of whether we ran detection this frame

        // ── Draw bounding boxes on the frame ─────────────────────────────────
        // Reuses last_faces even on frames where detection was skipped, so
        // the boxes stay visible and don't flicker between detection passes.
        for (size_t i = 0; i < last_faces.size(); i++) {
            const cv::Rect &r = last_faces[i];
            // Draw a green rectangle around the detected face, 2px thick.
            cv::rectangle(f.mat, r, cv::Scalar(0, 255, 0), 2);
            // Draw a "Face N" label just above the box.
            cv::putText(f.mat,
                        "Face " + std::to_string(i + 1),
                        cv::Point(r.x, r.y - 8), // 8px above the top of the box
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,                       // font scale
                        cv::Scalar(0, 255, 0),     // green text
                        2);                        // stroke thickness
        }

        // Draw a "[REC]" indicator in the bottom-right corner while recording.
        {
            std::lock_guard<std::mutex> lock(g_rec_mutex);
            if (g_rec_state == RecordState::RECORDING) {
                cv::putText(f.mat, "[REC]",
                            cv::Point(VIDEO_WIDTH - 120, VIDEO_HEIGHT - 20),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.7,
                            cv::Scalar(0, 0, 255), // red text
                            2);
            }
        }

        // ── Push the processed frame back into the GStreamer pipeline ───────
        if (!g_appsrc) continue; // appsrc not ready yet — drop this frame and keep going

        gsize size = VIDEO_WIDTH * VIDEO_HEIGHT * 3; // BGR = 3 bytes per pixel

        // Allocate a fresh GstBuffer to hold this frame's pixel data.
        GstBuffer *out = gst_buffer_new_allocate(nullptr, size, nullptr);

        // Map it for writing, copy the OpenCV Mat's bytes into it, then unmap.
        GstMapInfo m;
        gst_buffer_map(out, &m, GST_MAP_WRITE);
        memcpy(m.data, f.mat.data, size); // copy pixel data from cv::Mat into the GstBuffer
        gst_buffer_unmap(out, &m);

        // Preserve the original timestamp/duration from the camera so the
        // downstream pipeline (encoder, MediaMTX, recording) stays in sync.
        GST_BUFFER_PTS(out)      = f.pts;
        GST_BUFFER_DURATION(out) = f.duration;

        // Push this buffer into appsrc — from here it flows through the
        // encoder, then splits into the MediaMTX-publish branch and the
        // recording branch via tee.
        GstFlowReturn ret;
        g_signal_emit_by_name(g_appsrc, "push-buffer", out, &ret);
        gst_buffer_unref(out); // release our reference (GStreamer now owns the data)
    }

    // Thread is exiting (g_running became false) — make sure any in-progress
    // recording is stopped cleanly so the last .mp4 file isn't left corrupt.
    stop_recording();
}

// ═══════════════════════════════════════════════════════════════════════════════
// RECORDING FILENAME CALLBACK
// ═══════════════════════════════════════════════════════════════════════════════
// splitmuxsink calls this every time it needs to start a new output file
// (i.e. every SPLIT_SECONDS, or right when recording starts).
static gchar *on_format_location(GstElement *, guint id, gpointer) {
    auto t  = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    // Filename format: rec_YYYYMMDD_HHMMSS_NNN.mp4
    //   - the timestamp makes files easy to find by time
    //   - NNN is a zero-padded sequence number within this run
    oss << RECORDING_DIR << "/rec_"
        << std::put_time(&tm, "%Y%m%d_%H%M%S")
        << "_" << std::setw(3) << std::setfill('0') << id
        << ".mp4";
    std::string fn = oss.str();
    write_log("New file: " + fn);
    return g_strdup(fn.c_str()); // GStreamer takes ownership and g_free()s this later
}

// ═══════════════════════════════════════════════════════════════════════════════
// PIPELINE BUS WATCH — reports errors and EOS from the GStreamer pipeline
// ═══════════════════════════════════════════════════════════════════════════════
// Useful for diagnosing problems like "MediaMTX isn't running" (which shows
// up here as a connection-refused error from rtspclientsink).
static gboolean on_bus_message(GstBus *, GstMessage *msg, gpointer) {
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err;
            gchar  *debug;
            gst_message_parse_error(msg, &err, &debug); // extract error details
            std::cerr << "[Pipeline ERROR] " << err->message << "\n";
            if (debug) std::cerr << "  Debug: " << debug << "\n";
            g_error_free(err);
            g_free(debug);
            break;
        }
        case GST_MESSAGE_EOS:
            std::cout << "[Pipeline] EOS received\n"; // end-of-stream reached
            break;
        default:
            break; // ignore all other message types (state changes, tags, etc.)
    }
    return TRUE; // TRUE keeps this watch installed so it keeps receiving future messages
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════════════════════
int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);     // initialize GStreamer (parses any --gst-* CLI args)
    std::signal(SIGINT,  on_sigint); // catch Ctrl-C
    std::signal(SIGTERM, on_sigint); // catch kill/termination signal

    // ── Check that the hardware H.264 encoder is available ──────────────────
    // v4l2h264enc depends on the Pi's kernel video driver and /dev/video*
    // device nodes; it might be missing on a fresh/incomplete install.
    GstElementFactory *v4l2_fac = gst_element_factory_find("v4l2h264enc");
    if (!v4l2_fac) {
        std::cerr << "[HW] v4l2h264enc not found.\n";
        return 1;
    }
    gst_object_unref(v4l2_fac); // release the factory reference once checked
    std::cout << "[HW] v4l2h264enc OK\n";

    // ── Check that rtspclientsink is available ───────────────────────────────
    // This is the element used to PUBLISH our stream into MediaMTX. It comes
    // from gstreamer1.0-plugins-bad and may not be installed by default.
    GstElementFactory *rtspclientsink_fac =
        gst_element_factory_find("rtspclientsink");
    if (!rtspclientsink_fac) {
        std::cerr << "[GST] rtspclientsink not found.\n"
                  << "      Try: sudo apt install gstreamer1.0-plugins-bad\n";
        return 1;
    }
    gst_object_unref(rtspclientsink_fac);
    std::cout << "[GST] rtspclientsink OK\n";

    // ── Load the frontal-face Haar cascade ───────────────────────────────────
    cv::CascadeClassifier face_cascade;
    if (!face_cascade.load(CASCADE_PATH)) {
        std::cerr << "[OpenCV] Cannot load frontal cascade: " << CASCADE_PATH << "\n";
        return 1; // this cascade is required; abort if it's missing
    }
    std::cout << "[OpenCV] Frontal cascade loaded OK\n";

    // ── Load the profile (side-face) Haar cascade ────────────────────────────
    // Used purely to cross-check the frontal cascade's results and reduce
    // false positives. The app still works without it (just less accurate),
    // so failure here is not fatal.
    cv::CascadeClassifier profile_cascade;
    if (!profile_cascade.load(PROFILE_PATH)) {
        std::cerr << "[OpenCV] Profile cascade not found: " << PROFILE_PATH << "\n";
    } else {
        std::cout << "[OpenCV] Profile cascade loaded OK\n";
    }

    // ── Build the camera source pipeline ─────────────────────────────────────
    //
    //   libcamerasrc → video/x-raw → queue → videoconvert
    //     → gdkpixbufoverlay (logo image burned into the video)
    //     → clockoverlay (live timestamp burned into the video)
    //     → videoconvert → force BGR format
    //     → appsink (hands frames over to our on_new_sample callback)
    //
    gchar *src_str = g_strdup_printf(
        "libcamerasrc ! "
        "video/x-raw,width=%d,height=%d,framerate=%d/1 ! "
        "queue ! videoconvert ! "                       // decouple the camera from
                                                          // downstream processing via a queue,
                                                          // then convert to a format overlays accept
        "gdkpixbufoverlay location=%s offset-x=20 offset-y=20 ! "
                                                          // burns LOGO_PATH onto the video at
                                                          // (20,20) pixels from the top-left
        "clockoverlay "
        "  time-format=\"%%Y-%%m-%%d %%H:%%M:%%S\" "     // strftime-style format string
        "  halignment=right valignment=top "             // position: top-right corner
        "  ypad=50 font-desc=\"Sans 18\" shaded-background=true ! "
                                                          // 50px padding from top, readable
                                                          // font, dark background for contrast
        "videoconvert ! video/x-raw,format=BGR ! "        // force BGR so OpenCV can use the
                                                          // pixel data directly without conversion
        "appsink name=face_sink emit-signals=true sync=false "
        "  max-buffers=2 drop=true",                      // keep at most 2 buffered frames,
                                                          // drop the oldest if appsink can't
                                                          // keep up — never blocks the camera

        VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_FPS, LOGO_PATH
    );

    GError *err = nullptr;
    GstElement *src_pipeline = gst_parse_launch(src_str, &err); // parse the string into a real pipeline
    g_free(src_str); // the string itself is no longer needed once parsed
    if (err) {
        std::cerr << "[Src] " << err->message << "\n";
        g_error_free(err);
        return 1;
    }

    // Find the appsink element by name and hook up our callback to it.
    GstElement *sink_el = gst_bin_get_by_name(GST_BIN(src_pipeline), "face_sink");
    GstAppSinkCallbacks cbs{};      // zero-initialize the whole callback struct
    cbs.new_sample = on_new_sample; // called whenever a new frame is available
    gst_app_sink_set_callbacks(GST_APP_SINK(sink_el), &cbs, nullptr, nullptr);
    gst_object_unref(sink_el); // release our reference (the pipeline still holds its own)

    // ── Build the encoding + publish + recording pipeline ───────────────────
    //
    //   appsrc(BGR, from OpenCV thread)
    //     → queue (small, leaky — drops old frames instead of blocking)
    //     → videoconvert → force I420 (the format v4l2h264enc expects)
    //     → v4l2h264enc (hardware H.264 encoder — ONE shared instance)
    //     → h264parse (normalizes the H.264 bitstream, re-inserts SPS/PPS)
    //     → tee (splits the encoded stream into two branches)
    //          ├── PUBLISH branch:  queue → rtspclientsink → MediaMTX
    //          └── RECORD branch:   queue → valve → h264parse → splitmuxsink → .mp4
    //
    // Why only ONE v4l2h264enc instead of one per branch? The Pi 4's
    // VideoCore hardware encoder is only reliably stable running a single
    // instance at a time; running two in parallel (one per branch) was
    // found to overload the driver and disrupt the live stream whenever
    // recording started. Encoding once and tee-ing the already-encoded
    // H.264 bitstream avoids that problem entirely.
    gchar *pipeline_str = g_strdup_printf(
        // appsrc: where the OpenCV thread injects its processed frames.
        //   format=time     → use presentation timestamps (PTS) for sync
        //   is-live=true    → behaves as a live source (no seeking, etc.)
        //   block=false     → push-buffer never blocks waiting for downstream,
        //                     which avoids deadlocks if something is congested
        //   do-timestamp=true → let GStreamer auto-manage timestamps if we
        //                     ever push a buffer without one set; combined
        //                     with our explicit PTS assignment above, this
        //                     keeps segment/clock handling consistent and
        //                     avoids "segment format mismatch" errors seen
        //                     with do-timestamp=false in earlier testing.
        //   max-bytes=10MB  → internal queue limit so appsrc can't grow
        //                     unbounded if downstream stalls
        "appsrc name=opencv_src format=time is-live=true block=false "
        "  do-timestamp=true "
        "  max-bytes=10000000 "
        "  caps=\"video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1\" ! "

        // Small leaky queue: if the encoder briefly can't keep up, drop the
        // OLDEST buffered frame rather than blocking appsrc (and therefore
        // the OpenCV thread) indefinitely.
        "queue max-size-buffers=4 leaky=downstream ! "

        "videoconvert ! "
        "video/x-raw,format=I420 ! "        // v4l2h264enc requires I420 (YUV 4:2:0) input

        // Hardware H.264 encoder, running on the Pi's VideoCore GPU block
        // (uses only a small fraction of CPU compared to software x264enc).
        //   repeat_sequence_header=1 → re-send SPS/PPS with every keyframe,
        //       so a client joining mid-stream can start decoding immediately
        //   video_gop_size=VIDEO_FPS → force one keyframe per second,
        //       guaranteeing clients never wait long for the first decodable frame
        //   video_bitrate=HW_BITRATE → target bitrate in bits per second
        "v4l2h264enc extra-controls=\"controls,"
        "  repeat_sequence_header=1,"
        "  video_gop_size=%d,"
        "  video_bitrate=%d\" ! "

        "video/x-h264,level=(string)4 ! "   // explicitly tag H.264 level 4 for broad
                                             // client compatibility
        "h264parse config-interval=1 ! "    // normalize the bitstream and re-insert
                                             // SPS/PPS into the stream every second

        // Split the single encoded H.264 stream into two independent branches.
        "tee name=t "

        // ── Publish branch: send the live stream to MediaMTX ────────────────
        "t. ! queue max-size-buffers=4 leaky=downstream ! "
        // rtspclientsink acts as an RTSP CLIENT that connects OUT to MediaMTX
        // and publishes ("announces") this stream there, rather than this
        // app waiting for incoming RTSP client connections itself.
        //   protocols=tcp → use TCP for RTP delivery (more firewall/NAT
        //       friendly than UDP, especially over Tailscale/VPN links)
        //   latency=0     → don't add extra buffering delay when publishing
        "rtspclientsink name=mtx_sink location=%s protocols=tcp latency=0 "

        // ── Recording branch: only writes to disk while the valve is open ───
        "t. ! queue name=rec_queue max-size-buffers=4 leaky=downstream ! "
        // valve: drop=true by default, meaning NOTHING is recorded until
        // start_recording() explicitly opens it (sets drop=FALSE).
        "valve name=rec_valve drop=true ! "
        "h264parse ! "                       // re-parse before muxing (good practice,
                                              // ensures splitmuxsink gets clean access units)
        // splitmuxsink writes .mp4 files, automatically starting a new file
        // every max-size-time nanoseconds. max-size-bytes=0 disables the
        // size-based cutoff entirely so only the time-based one applies.
        "splitmuxsink name=recorder max-size-time=%lu max-size-bytes=0",

        VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_FPS,        // appsrc caps
        VIDEO_FPS, HW_BITRATE,                       // v4l2h264enc gop_size, bitrate
        MEDIAMTX_URL,                                // rtspclientsink location
        (guint64)SPLIT_SECONDS * GST_SECOND          // splitmuxsink max-size-time
                                                      // (GST_SECOND = 1,000,000,000 ns;
                                                      // cast to guint64 to avoid 32-bit
                                                      // integer overflow on the multiply)
    );

    GstElement *pipeline = gst_parse_launch(pipeline_str, &err);
    g_free(pipeline_str);
    if (err) {
        std::cerr << "[Pipeline] " << err->message << "\n";
        g_error_free(err);
        return 1;
    }

    // Grab a reference to appsrc so the OpenCV thread can push frames into it.
    g_appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "opencv_src");
    std::cout << (g_appsrc ? "[appsrc] Ready\n" : "[appsrc] NOT FOUND\n");

    // Grab splitmuxsink and hook up the filename callback.
    g_splitmux = gst_bin_get_by_name(GST_BIN(pipeline), "recorder");
    if (g_splitmux) {
        g_signal_connect(g_splitmux, "format-location",
                         G_CALLBACK(on_format_location), nullptr);
        std::cout << "[Recorder] Callback registered\n";
    } else {
        std::cerr << "[Recorder] splitmuxsink not found\n";
    }

    // Grab the valve so start_recording()/stop_recording() can toggle it.
    g_valve = gst_bin_get_by_name(GST_BIN(pipeline), "rec_valve");
    std::cout << (g_valve ? "[Valve] Ready (drop=true, recording paused)\n"
                          : "[Valve] NOT FOUND\n");

    // Watch the pipeline's bus for errors (e.g. MediaMTX not running) and EOS.
    GstBus *bus = gst_element_get_bus(pipeline);
    gst_bus_add_watch(bus, on_bus_message, nullptr);
    gst_object_unref(bus); // the watch holds its own reference internally

    // ── Start the encoding/publish/recording pipeline ────────────────────────
    if (gst_element_set_state(pipeline, GST_STATE_PLAYING)
            == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "[Pipeline] Failed to start. "
                  << "Kiem tra MediaMTX da chay chua: " << MEDIAMTX_URL << "\n";
                  // (Vietnamese: "Check whether MediaMTX is already running")
        return 1;
    }
    std::cout << "[Pipeline] Publishing to MediaMTX started\n";

    // ── Start the camera source pipeline ──────────────────────────────────────
    if (gst_element_set_state(src_pipeline, GST_STATE_PLAYING)
            == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "[Src] Failed to start source pipeline\n";
        return 1;
    }
    std::cout << "[Src] Camera pipeline started\n";

    // ── Launch the OpenCV worker thread ───────────────────────────────────────
    std::thread cv_thread(opencv_thread, &face_cascade, &profile_cascade);

    write_log("=== Camera system started ===");
    std::cout << "\n====================================\n"
              << "Camera -> MediaMTX + Smart Recording + Face Detection\n"
              << "------------------------------------\n"
              << "Publish to : " << MEDIAMTX_URL << "\n"
              << "View on VLC: rtsp://<PI_IP>:8554/camera\n"
              << "             (MediaMTX phai dang chay truoc!)\n"
              << "Resolution : " << VIDEO_WIDTH << "x" << VIDEO_HEIGHT
              << " @ " << VIDEO_FPS << " FPS\n"
              << "Output dir : " << RECORDING_DIR << "\n"
              << "Log file   : " << LOG_PATH << "\n"
              << "Record mode: only when face detected\n"
              << "Confirm    : " << CONFIRM_FRAMES << " consecutive detections\n"
              << "Tail time  : " << RECORD_TAIL_SEC << "s after last face\n"
              << "Press Ctrl-C to stop\n"
              << "====================================\n\n";

    // ── GLib main loop: blocks here, processing GStreamer events ─────────────
    // (bus messages, timers, etc.) until on_sigint() calls g_main_loop_quit().
    g_main_loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(g_main_loop); // blocks until Ctrl-C

    // ── Shutdown sequence ──────────────────────────────────────────────────────
    g_running = false;          // tell the OpenCV thread to stop its loop
    g_queue_cv.notify_all();    // wake it up in case it's currently waiting
    cv_thread.join();           // wait for it to fully finish (incl. stop_recording())

    write_log("=== Camera system stopped ===");

    // Tear down both pipelines: camera source first, then the encoding/publish one.
    gst_element_set_state(src_pipeline, GST_STATE_NULL);
    gst_element_set_state(pipeline, GST_STATE_NULL);

    // Release all GStreamer object references (ref counts drop to zero and
    // the objects are destroyed automatically).
    gst_object_unref(src_pipeline);
    if (g_appsrc)   gst_object_unref(g_appsrc);
    if (g_splitmux) gst_object_unref(g_splitmux);
    if (g_valve)    gst_object_unref(g_valve);
    gst_object_unref(pipeline);
    g_main_loop_unref(g_main_loop);

    std::cout << "[Main] Done.\n";
    return 0;
}