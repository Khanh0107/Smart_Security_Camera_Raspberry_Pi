#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <csignal>
#include <ctime>

static const char *LOGO_PATH        = "/home/bobobo/rtsp_camera/logo/penguin-svgrepo.svg";
static const char *RECORDING_DIR    = "/home/bobobo/rtsp_camera/recordings";
static const char *LOG_PATH         = "/home/bobobo/rtsp_camera/recordings/detect.log";
static const char *CASCADE_PATH     = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml";
static const char *PROFILE_PATH     = "/usr/share/opencv4/haarcascades/haarcascade_profileface.xml";
static const char *MEDIAMTX_URL     = "rtsp://127.0.0.1:8554/camera";

static const int   VIDEO_WIDTH      = 1280;
static const int   VIDEO_HEIGHT     = 720;
static const int   VIDEO_FPS        = 30;
static const int   HW_BITRATE       = 2000000;
static const int   SPLIT_SECONDS    = 20;
static const int   DETECT_EVERY     = 10;
static const int   QUEUE_MAX        = 3;
static const int   RECORD_TAIL_SEC  = 5;
static const int   CONFIRM_FRAMES   = 3;

enum class RecordState { IDLE, RECORDING };

static RecordState  g_rec_state = RecordState::IDLE;
static GstElement  *g_splitmux  = nullptr;
static GstElement  *g_valve     = nullptr;
static std::mutex   g_rec_mutex;
static std::chrono::steady_clock::time_point g_last_face_time;

struct Frame {
    cv::Mat      mat;
    GstClockTime pts;
    GstClockTime duration;
};

static std::queue<Frame>       g_frame_queue;
static std::mutex              g_queue_mutex;
static std::condition_variable g_queue_cv;
static std::atomic<bool>       g_running{true};
static GMainLoop              *g_main_loop = nullptr;
static GstElement             *g_appsrc    = nullptr;

static void on_sigint(int) {
    g_running = false;
    g_queue_cv.notify_all();
    if (g_main_loop) g_main_loop_quit(g_main_loop);
}

static std::string now_str() {
    auto t  = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

static void write_log(const std::string &msg) {
    std::string line = "[" + now_str() + "] " + msg;
    std::cout << line << "\n";
    std::ofstream f(LOG_PATH, std::ios::app);
    if (f) f << line << "\n";
}

static void start_recording() {
    std::lock_guard<std::mutex> lock(g_rec_mutex);
    if (g_rec_state == RecordState::RECORDING) return;
    if (!g_valve) return;
    g_object_set(g_valve, "drop", FALSE, nullptr);
    g_rec_state = RecordState::RECORDING;
    write_log("RECORDING START - face detected");
}

static void stop_recording() {
    std::lock_guard<std::mutex> lock(g_rec_mutex);
    if (g_rec_state == RecordState::IDLE) return;
    if (!g_valve || !g_splitmux) return;
    g_object_set(g_valve, "drop", TRUE, nullptr);
    GstPad *sink_pad = gst_element_get_static_pad(g_splitmux, "sink");
    if (sink_pad) {
        gst_pad_send_event(sink_pad, gst_event_new_eos());
        gst_object_unref(sink_pad);
    }
    g_rec_state = RecordState::IDLE;
    write_log("RECORDING STOP  - no face for " +
              std::to_string(RECORD_TAIL_SEC) + "s");
}

static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer) {
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer *buf = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    {
        std::unique_lock<std::mutex> lock(g_queue_mutex);
        if ((int)g_frame_queue.size() < QUEUE_MAX) {
            Frame f;
            f.mat      = cv::Mat(VIDEO_HEIGHT, VIDEO_WIDTH,
                                 CV_8UC3, map.data).clone();
            f.pts      = GST_BUFFER_PTS(buf);
            f.duration = GST_BUFFER_DURATION(buf);
            g_frame_queue.push(std::move(f));
            g_queue_cv.notify_one();
        }
    }

    gst_buffer_unmap(buf, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

static void opencv_thread(cv::CascadeClassifier *cascade,
                          cv::CascadeClassifier *profile) {
    int frame_count = 0;
    std::vector<cv::Rect> last_faces;
    bool was_recording = false;
    int  confirm_count = 0;

    while (g_running) {
        Frame f;
        {
            std::unique_lock<std::mutex> lock(g_queue_mutex);
            g_queue_cv.wait(lock, [] {
                return !g_frame_queue.empty() || !g_running;
            });
            if (!g_running && g_frame_queue.empty()) break;
            f = std::move(g_frame_queue.front());
            g_frame_queue.pop();
        }

        if (frame_count % DETECT_EVERY == 0) {
            cv::Mat gray, small;
            cv::cvtColor(f.mat, gray, cv::COLOR_BGR2GRAY);
            cv::equalizeHist(gray, gray);
            cv::resize(gray, small,
                       cv::Size(VIDEO_WIDTH/2, VIDEO_HEIGHT/2));

            std::vector<cv::Rect> faces_small;
            cascade->detectMultiScale(small, faces_small,
                1.05, 9, 0,
                cv::Size(60, 60), cv::Size(350, 350));

            if (faces_small.size() > 1) {
                std::vector<int> w(faces_small.size(), 1);
                cv::groupRectangles(faces_small, w, 1, 0.2);
            }

            std::vector<cv::Rect> faces_full;
            for (auto &r : faces_small)
                faces_full.push_back(
                    cv::Rect(r.x*2, r.y*2, r.width*2, r.height*2));

            last_faces.clear();
            if (!profile || faces_full.empty()) {
                last_faces = faces_full;
            } else {
                for (auto &face : faces_full) {
                    int x1 = std::max(0, face.x);
                    int y1 = std::max(0, face.y);
                    int x2 = std::min(VIDEO_WIDTH,  face.x + face.width);
                    int y2 = std::min(VIDEO_HEIGHT, face.y + face.height);
                    cv::Mat roi = gray(cv::Rect(x1, y1, x2-x1, y2-y1));
                    std::vector<cv::Rect> profile_faces;
                    profile->detectMultiScale(roi, profile_faces, 1.1, 3, 0,
                        cv::Size(20,20));
                    if (!profile_faces.empty())
                        last_faces.push_back(face);
                }
                if (last_faces.empty() && !faces_full.empty())
                    last_faces = faces_full;
            }

            bool has_face = !last_faces.empty();

            if (has_face) {
                confirm_count++;
                g_last_face_time = std::chrono::steady_clock::now();
                if (!was_recording && confirm_count >= CONFIRM_FRAMES) {
                    start_recording();
                    was_recording = true;
                    write_log("Detected " + std::to_string(last_faces.size())
                              + " face(s) (confirmed x"
                              + std::to_string(confirm_count) + ")");
                }
            } else {
                confirm_count = 0;
                if (was_recording) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::steady_clock::now() - g_last_face_time).count();
                    if (elapsed >= RECORD_TAIL_SEC) {
                        stop_recording();
                        was_recording = false;
                        write_log("No face detected");
                    }
                }
            }
        }
        frame_count++;

        for (size_t i = 0; i < last_faces.size(); i++) {
            const cv::Rect &r = last_faces[i];
            cv::rectangle(f.mat, r, cv::Scalar(0, 255, 0), 2);
            cv::putText(f.mat, "Face " + std::to_string(i + 1),
                        cv::Point(r.x, r.y - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 0), 2);
        }

        {
            std::lock_guard<std::mutex> lock(g_rec_mutex);
            if (g_rec_state == RecordState::RECORDING) {
                cv::putText(f.mat, "[REC]",
                            cv::Point(VIDEO_WIDTH - 120, VIDEO_HEIGHT - 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            cv::Scalar(0, 0, 255), 2);
            }
        }

        if (!g_appsrc) continue;

        gsize size = VIDEO_WIDTH * VIDEO_HEIGHT * 3;
        GstBuffer *out = gst_buffer_new_allocate(nullptr, size, nullptr);
        GstMapInfo m;
        gst_buffer_map(out, &m, GST_MAP_WRITE);
        memcpy(m.data, f.mat.data, size);
        gst_buffer_unmap(out, &m);

        GST_BUFFER_PTS(out)      = f.pts;
        GST_BUFFER_DURATION(out) = f.duration;

        GstFlowReturn ret;
        g_signal_emit_by_name(g_appsrc, "push-buffer", out, &ret);
        gst_buffer_unref(out);
    }

    stop_recording();
}

static gchar *on_format_location(GstElement *, guint id, gpointer) {
    auto t  = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << RECORDING_DIR << "/rec_"
        << std::put_time(&tm, "%Y%m%d_%H%M%S")
        << "_" << std::setw(3) << std::setfill('0') << id
        << ".mp4";
    std::string fn = oss.str();
    write_log("New file: " + fn);
    return g_strdup(fn.c_str());
}

static gboolean on_bus_message(GstBus *, GstMessage *msg, gpointer) {
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err;
            gchar  *debug;
            gst_message_parse_error(msg, &err, &debug);
            std::cerr << "[Pipeline ERROR] " << err->message << "\n";
            if (debug) std::cerr << "  Debug: " << debug << "\n";
            g_error_free(err);
            g_free(debug);
            break;
        }
        case GST_MESSAGE_EOS:
            std::cout << "[Pipeline] EOS received\n";
            break;
        default:
            break;
    }
    return TRUE;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);
    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    GstElementFactory *v4l2_fac = gst_element_factory_find("v4l2h264enc");
    if (!v4l2_fac) {
        std::cerr << "[HW] v4l2h264enc not found.\n";
        return 1;
    }
    gst_object_unref(v4l2_fac);
    std::cout << "[HW] v4l2h264enc OK\n";

    GstElementFactory *rtspclientsink_fac =
        gst_element_factory_find("rtspclientsink");
    if (!rtspclientsink_fac) {
        std::cerr << "[GST] rtspclientsink not found.\n"
                  << "      Try: sudo apt install gstreamer1.0-plugins-bad\n";
        return 1;
    }
    gst_object_unref(rtspclientsink_fac);
    std::cout << "[GST] rtspclientsink OK\n";

    cv::CascadeClassifier face_cascade;
    if (!face_cascade.load(CASCADE_PATH)) {
        std::cerr << "[OpenCV] Cannot load frontal cascade: " << CASCADE_PATH << "\n";
        return 1;
    }
    std::cout << "[OpenCV] Frontal cascade loaded OK\n";

    cv::CascadeClassifier profile_cascade;
    if (!profile_cascade.load(PROFILE_PATH)) {
        std::cerr << "[OpenCV] Profile cascade not found: " << PROFILE_PATH << "\n";
    } else {
        std::cout << "[OpenCV] Profile cascade loaded OK\n";
    }

    gchar *src_str = g_strdup_printf(
        "libcamerasrc ! "
        "video/x-raw,width=%d,height=%d,framerate=%d/1 ! "
        "queue ! videoconvert ! "
        "gdkpixbufoverlay location=%s offset-x=20 offset-y=20 ! "
        "clockoverlay "
        "  time-format=\"%%Y-%%m-%%d %%H:%%M:%%S\" "
        "  halignment=right valignment=top "
        "  ypad=50 font-desc=\"Sans 18\" shaded-background=true ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=face_sink emit-signals=true sync=false "
        "  max-buffers=2 drop=true",
        VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_FPS, LOGO_PATH
    );

    GError *err = nullptr;
    GstElement *src_pipeline = gst_parse_launch(src_str, &err);
    g_free(src_str);
    if (err) {
        std::cerr << "[Src] " << err->message << "\n";
        g_error_free(err);
        return 1;
    }

    GstElement *sink_el = gst_bin_get_by_name(GST_BIN(src_pipeline), "face_sink");
    GstAppSinkCallbacks cbs{};
    cbs.new_sample = on_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(sink_el), &cbs, nullptr, nullptr);
    gst_object_unref(sink_el);

    gchar *pipeline_str = g_strdup_printf(
        "appsrc name=opencv_src format=time is-live=true block=false "
        "  do-timestamp=true "
        "  max-bytes=10000000 "
        "  caps=\"video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1\" ! "
        "queue max-size-buffers=4 leaky=downstream ! "
        "videoconvert ! "
        "video/x-raw,format=I420 ! "
        "v4l2h264enc extra-controls=\"controls,"
        "  repeat_sequence_header=1,"
        "  video_gop_size=%d,"
        "  video_bitrate=%d\" ! "
        "video/x-h264,level=(string)4 ! "
        "h264parse config-interval=1 ! "
        "tee name=t "
        "t. ! queue max-size-buffers=4 leaky=downstream ! "
        "rtspclientsink name=mtx_sink location=%s protocols=tcp latency=0 "
        "t. ! queue name=rec_queue max-size-buffers=4 leaky=downstream ! "
        "valve name=rec_valve drop=true ! "
        "h264parse ! "
        "splitmuxsink name=recorder max-size-time=%lu max-size-bytes=0",

        VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_FPS,
        VIDEO_FPS, HW_BITRATE,
        MEDIAMTX_URL,
        (guint64)SPLIT_SECONDS * GST_SECOND
    );

    GstElement *pipeline = gst_parse_launch(pipeline_str, &err);
    g_free(pipeline_str);
    if (err) {
        std::cerr << "[Pipeline] " << err->message << "\n";
        g_error_free(err);
        return 1;
    }

    g_appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "opencv_src");
    std::cout << (g_appsrc ? "[appsrc] Ready\n" : "[appsrc] NOT FOUND\n");

    g_splitmux = gst_bin_get_by_name(GST_BIN(pipeline), "recorder");
    if (g_splitmux) {
        g_signal_connect(g_splitmux, "format-location",
                         G_CALLBACK(on_format_location), nullptr);
        std::cout << "[Recorder] Callback registered\n";
    } else {
        std::cerr << "[Recorder] splitmuxsink not found\n";
    }

    g_valve = gst_bin_get_by_name(GST_BIN(pipeline), "rec_valve");
    std::cout << (g_valve ? "[Valve] Ready (drop=true, recording paused)\n"
                          : "[Valve] NOT FOUND\n");

    GstBus *bus = gst_element_get_bus(pipeline);
    gst_bus_add_watch(bus, on_bus_message, nullptr);
    gst_object_unref(bus);

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING)
            == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "[Pipeline] Failed to start. "
                  << "Kiem tra MediaMTX da chay chua: " << MEDIAMTX_URL << "\n";
        return 1;
    }
    std::cout << "[Pipeline] Publishing to MediaMTX started\n";

    if (gst_element_set_state(src_pipeline, GST_STATE_PLAYING)
            == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "[Src] Failed to start source pipeline\n";
        return 1;
    }
    std::cout << "[Src] Camera pipeline started\n";

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

    g_main_loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(g_main_loop);

    g_running = false;
    g_queue_cv.notify_all();
    cv_thread.join();

    write_log("=== Camera system stopped ===");

    gst_element_set_state(src_pipeline, GST_STATE_NULL);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(src_pipeline);
    if (g_appsrc)   gst_object_unref(g_appsrc);
    if (g_splitmux) gst_object_unref(g_splitmux);
    if (g_valve)    gst_object_unref(g_valve);
    gst_object_unref(pipeline);
    g_main_loop_unref(g_main_loop);

    std::cout << "[Main] Done.\n";
    return 0;
}