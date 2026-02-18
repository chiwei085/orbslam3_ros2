/**
 * @file make_e2e_bag.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for make e2e bag.
 */
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "orbslam3_ros2/msg/observations.hpp"

namespace fs = std::filesystem;
namespace Msg = orbslam3_ros2::msg;

struct CliOptions
{
    fs::path tum_dir;
    fs::path output_bag;
    std::string topic{"/orbslam3/observations"};
    std::string frame_id{"camera_link"};
    double max_dt_sec{0.02};
    std::size_t max_frames{0};
    bool overwrite{false};
};

struct TimedPath
{
    double t{0.0};
    fs::path rel_path;
};

void print_usage() {
    std::cerr << "Usage:\n"
              << "  orbslam3_make_e2e_bag"
              << " --tum-dir <path> --output-bag <path> [options]\n\n"
              << "Options:\n"
              << "  --topic <name>           default: /orbslam3/observations\n"
              << "  --frame-id <id>          default: camera_link\n"
              << "  --max-dt-ms <ms>         default: 20\n"
              << "  --max-frames <n>         default: 0 (all)\n"
              << "  --overwrite              remove output bag dir if exists\n";
}

bool parse_cli(int argc, char** argv, CliOptions& opt) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        auto need_value = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << flag << "\n";
                return nullptr;
            }
            ++i;
            return argv[i];
        };

        if (arg == "--tum-dir") {
            const char* value = need_value("--tum-dir");
            if (!value) {
                return false;
            }
            opt.tum_dir = fs::path(value);
        }
        else if (arg == "--output-bag") {
            const char* value = need_value("--output-bag");
            if (!value) {
                return false;
            }
            opt.output_bag = fs::path(value);
        }
        else if (arg == "--topic") {
            const char* value = need_value("--topic");
            if (!value) {
                return false;
            }
            opt.topic = value;
        }
        else if (arg == "--frame-id") {
            const char* value = need_value("--frame-id");
            if (!value) {
                return false;
            }
            opt.frame_id = value;
        }
        else if (arg == "--max-dt-ms") {
            const char* value = need_value("--max-dt-ms");
            if (!value) {
                return false;
            }
            opt.max_dt_sec = std::stod(value) * 1e-3;
        }
        else if (arg == "--max-frames") {
            const char* value = need_value("--max-frames");
            if (!value) {
                return false;
            }
            opt.max_frames = static_cast<std::size_t>(std::stoull(value));
        }
        else if (arg == "--overwrite") {
            opt.overwrite = true;
        }
        else if (arg == "--help" || arg == "-h") {
            print_usage();
            return false;
        }
        else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return false;
        }
    }

    if (opt.tum_dir.empty() || opt.output_bag.empty()) {
        std::cerr << "--tum-dir and --output-bag are required\n";
        return false;
    }
    return true;
}

std::string trim(const std::string& in) {
    const auto begin = in.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = in.find_last_not_of(" \t\r\n");
    return in.substr(begin, end - begin + 1);
}

std::vector<TimedPath> parse_tum_list(const fs::path& file_path) {
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        throw std::runtime_error("Failed to open list file: " +
                                 file_path.string());
    }

    std::vector<TimedPath> out;
    std::string line;
    while (std::getline(ifs, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        double t = 0.0;
        std::string rel;
        iss >> t >> rel;
        if (!iss || rel.empty()) {
            continue;
        }

        if (rel.rfind("./", 0) == 0) {
            rel = rel.substr(2);
        }
        out.push_back(TimedPath{t, fs::path(rel)});
    }

    std::sort(out.begin(), out.end(),
              [](const TimedPath& a, const TimedPath& b) { return a.t < b.t; });
    return out;
}

std::vector<std::pair<TimedPath, TimedPath>> associate_rgb_depth(
    const std::vector<TimedPath>& rgbs, const std::vector<TimedPath>& depths,
    const double max_dt_sec, const std::size_t max_frames) {
    std::vector<std::pair<TimedPath, TimedPath>> pairs;
    if (rgbs.empty() || depths.empty()) {
        return pairs;
    }

    std::size_t j = 0;
    for (const auto& rgb : rgbs) {
        while (j + 1 < depths.size() && std::fabs(depths[j + 1].t - rgb.t) <=
                                            std::fabs(depths[j].t - rgb.t)) {
            ++j;
        }
        if (std::fabs(depths[j].t - rgb.t) <= max_dt_sec) {
            pairs.emplace_back(rgb, depths[j]);
            if (max_frames > 0 && pairs.size() >= max_frames) {
                break;
            }
        }
    }
    return pairs;
}

builtin_interfaces::msg::Time to_stamp(const std::int64_t ns) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return stamp;
}

sensor_msgs::msg::Image make_rgb_msg(const cv::Mat& rgb_bgr,
                                     const std_msgs::msg::Header& header) {
    cv_bridge::CvImage cv(header, sensor_msgs::image_encodings::BGR8, rgb_bgr);
    return *cv.toImageMsg();
}

sensor_msgs::msg::Image make_depth_msg(const cv::Mat& depth_16u,
                                       const std_msgs::msg::Header& header) {
    cv::Mat depth_m(depth_16u.rows, depth_16u.cols, CV_32FC1);
    constexpr float kTumDepthScale = 5000.0f;
    for (int r = 0; r < depth_16u.rows; ++r) {
        const auto* in = depth_16u.ptr<std::uint16_t>(r);
        auto* out = depth_m.ptr<float>(r);
        for (int c = 0; c < depth_16u.cols; ++c) {
            out[c] =
                in[c] > 0 ? static_cast<float>(in[c]) / kTumDepthScale : 0.0f;
        }
    }
    cv_bridge::CvImage cv(header, sensor_msgs::image_encodings::TYPE_32FC1,
                          depth_m);
    return *cv.toImageMsg();
}

int main(int argc, char** argv) {
    CliOptions opt;
    if (!parse_cli(argc, argv, opt)) {
        print_usage();
        return 2;
    }

    try {
        const auto rgb_list = parse_tum_list(opt.tum_dir / "rgb.txt");
        const auto depth_list = parse_tum_list(opt.tum_dir / "depth.txt");
        const auto pairs = associate_rgb_depth(rgb_list, depth_list,
                                               opt.max_dt_sec, opt.max_frames);

        if (pairs.empty()) {
            std::cerr << "No associated rgb/depth pairs found under "
                      << opt.tum_dir << "\n";
            return 1;
        }

        if (fs::exists(opt.output_bag)) {
            if (!opt.overwrite) {
                std::cerr << "Output bag exists: " << opt.output_bag
                          << " (use --overwrite)\n";
                return 1;
            }
            fs::remove_all(opt.output_bag);
        }
        fs::create_directories(opt.output_bag.parent_path());

        rclcpp::init(0, nullptr);
        rosbag2_cpp::Writer writer;
        writer.open(opt.output_bag.string());
        writer.create_topic(
            {opt.topic, "orbslam3_ros2/msg/Observations", "cdr", ""});

        std::int64_t prev_ns = -1;
        std::size_t written = 0;
        for (const auto& [rgb_e, depth_e] : pairs) {
            const auto rgb_path = opt.tum_dir / rgb_e.rel_path;
            const auto depth_path = opt.tum_dir / depth_e.rel_path;
            cv::Mat rgb = cv::imread(rgb_path.string(), cv::IMREAD_COLOR);
            cv::Mat depth =
                cv::imread(depth_path.string(), cv::IMREAD_UNCHANGED);

            if (rgb.empty() || depth.empty()) {
                std::cerr << "Skip unreadable pair:\n  rgb=" << rgb_path
                          << "\n  depth=" << depth_path << "\n";
                continue;
            }
            if (depth.type() != CV_16UC1) {
                std::cerr << "Skip depth type != CV_16UC1: " << depth_path
                          << "\n";
                continue;
            }

            const auto stamp_ns =
                static_cast<std::int64_t>(std::llround(rgb_e.t * 1e9));
            std_msgs::msg::Header header;
            header.stamp = to_stamp(stamp_ns);
            header.frame_id = opt.frame_id;

            Msg::Observations obs;
            obs.header = header;
            obs.mode = Msg::Observations::MODE_RGBD;
            obs.images.reserve(2);
            obs.images.push_back(make_rgb_msg(rgb, header));
            obs.images.push_back(make_depth_msg(depth, header));
            obs.camera_frame = opt.frame_id;
            obs.t_track_ns =
                static_cast<std::uint64_t>(std::max<std::int64_t>(0, stamp_ns));
            obs.dt_ns = prev_ns >= 0 ? (stamp_ns - prev_ns) : 0;

            writer.write(obs, opt.topic, rclcpp::Time(stamp_ns));
            prev_ns = stamp_ns;
            ++written;
        }

        rclcpp::shutdown();
        std::cout << "Wrote " << written << " observations to "
                  << opt.output_bag << "\n"
                  << "Topic: " << opt.topic << "\n"
                  << "Frame: " << opt.frame_id << "\n";
        if (written == 0) {
            return 1;
        }
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Failed: " << e.what() << "\n";
        return 1;
    }
}
