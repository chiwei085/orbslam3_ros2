/**
 * @file native_runner.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for native runner.
 */
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "orbslam3_ros2/orbslam3/system.hpp"

namespace fs = std::filesystem;
namespace Orb = orbslam3_ros2::orbslam3;

struct CliOptions
{
    fs::path tum_dir;
    fs::path vocab_path;
    fs::path settings_path;
    fs::path association_path;
    double depth_scale{5000.0};
    double max_dt_sec{0.02};
    std::size_t max_frames{0};
};

struct TimedPath
{
    double t{0.0};
    fs::path rel_path;
};

struct FramePair
{
    double t{0.0};
    fs::path rgb_rel;
    fs::path depth_rel;
};

void print_usage() {
    std::cerr
        << "Usage:\n"
        << "  orbslam3_native_runner --tum-dir <path> --voc <path> "
           "--settings <path> [options]\n"
        << "Options:\n"
        << "  --associate <path>      TUM association file "
           "(default: <tum-dir>/associate.txt)\n"
        << "  --depth-scale <value>   depth scale divisor for 16UC1 "
           "(default: 5000)\n"
        << "  --max-frames <n>        limit processed frames (default: all)\n"
        << "  --max-dt-ms <ms>        max RGB-depth match gap for "
           "rgb/depth.txt "
           "mode (default: 20)\n";
}

bool parse_cli(int argc, char** argv, CliOptions* out) {
    if (!out) {
        return false;
    }
    CliOptions opt;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        auto require_value = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << flag << "\n";
                return nullptr;
            }
            ++i;
            return argv[i];
        };

        if (arg == "--tum-dir") {
            const auto* v = require_value("--tum-dir");
            if (!v) {
                return false;
            }
            opt.tum_dir = fs::path(v);
            continue;
        }
        if (arg == "--voc") {
            const auto* v = require_value("--voc");
            if (!v) {
                return false;
            }
            opt.vocab_path = fs::path(v);
            continue;
        }
        if (arg == "--settings") {
            const auto* v = require_value("--settings");
            if (!v) {
                return false;
            }
            opt.settings_path = fs::path(v);
            continue;
        }
        if (arg == "--associate") {
            const auto* v = require_value("--associate");
            if (!v) {
                return false;
            }
            opt.association_path = fs::path(v);
            continue;
        }
        if (arg == "--depth-scale") {
            const auto* v = require_value("--depth-scale");
            if (!v) {
                return false;
            }
            opt.depth_scale = std::stod(v);
            continue;
        }
        if (arg == "--max-frames") {
            const auto* v = require_value("--max-frames");
            if (!v) {
                return false;
            }
            opt.max_frames = static_cast<std::size_t>(std::stoull(v));
            continue;
        }
        if (arg == "--max-dt-ms") {
            const auto* v = require_value("--max-dt-ms");
            if (!v) {
                return false;
            }
            opt.max_dt_sec = std::stod(v) * 1e-3;
            continue;
        }
        if (arg == "--help" || arg == "-h") {
            print_usage();
            return false;
        }
        std::cerr << "Unknown option: " << arg << "\n";
        return false;
    }

    if (opt.tum_dir.empty() || opt.vocab_path.empty() ||
        opt.settings_path.empty()) {
        std::cerr << "--tum-dir, --voc, --settings are required.\n";
        return false;
    }
    if (opt.association_path.empty()) {
        opt.association_path = opt.tum_dir / "associate.txt";
    }
    if (opt.depth_scale <= 0.0) {
        std::cerr << "--depth-scale must be > 0\n";
        return false;
    }

    *out = std::move(opt);
    return true;
}

std::string trim(std::string s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

bool parse_association(const fs::path& file, std::vector<FramePair>* out) {
    std::ifstream ifs(file);
    if (!ifs.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(ifs, line)) {
        line = trim(std::move(line));
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream iss(line);
        FramePair p;
        double t_depth = 0.0;
        iss >> p.t >> p.rgb_rel >> t_depth >> p.depth_rel;
        if (iss && !p.rgb_rel.empty() && !p.depth_rel.empty()) {
            out->push_back(std::move(p));
        }
    }
    return !out->empty();
}

std::vector<TimedPath> parse_timed_list(const fs::path& file) {
    std::ifstream ifs(file);
    if (!ifs.is_open()) {
        return {};
    }
    std::vector<TimedPath> out;
    std::string line;
    while (std::getline(ifs, line)) {
        line = trim(std::move(line));
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream iss(line);
        TimedPath e;
        iss >> e.t >> e.rel_path;
        if (iss && !e.rel_path.empty()) {
            out.push_back(std::move(e));
        }
    }
    std::sort(out.begin(), out.end(),
              [](const TimedPath& a, const TimedPath& b) { return a.t < b.t; });
    return out;
}

std::vector<FramePair> pair_from_rgb_depth_lists(const fs::path& tum_dir,
                                                 const double max_dt_sec) {
    const auto rgbs = parse_timed_list(tum_dir / "rgb.txt");
    const auto depths = parse_timed_list(tum_dir / "depth.txt");
    std::vector<FramePair> pairs;
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
            pairs.push_back(FramePair{rgb.t, rgb.rel_path, depths[j].rel_path});
        }
    }
    return pairs;
}

bool convert_depth_to_meters(cv::Mat* depth, const double depth_scale) {
    if (!depth || depth->empty()) {
        return false;
    }
    if (depth->type() == CV_16UC1) {
        cv::Mat converted(depth->rows, depth->cols, CV_32FC1);
        const float inv_scale = static_cast<float>(1.0 / depth_scale);
        for (int r = 0; r < depth->rows; ++r) {
            const auto* src = depth->ptr<std::uint16_t>(r);
            auto* dst = converted.ptr<float>(r);
            for (int c = 0; c < depth->cols; ++c) {
                dst[c] =
                    src[c] > 0 ? static_cast<float>(src[c]) * inv_scale : 0.0f;
            }
        }
        *depth = std::move(converted);
        return true;
    }
    if (depth->type() == CV_32FC1) {
        if (std::fabs(depth_scale - 1.0) > 1e-9) {
            depth->convertTo(*depth, CV_32FC1, 1.0 / depth_scale);
        }
        return true;
    }
    return false;
}

int main(int argc, char** argv) {
    CliOptions opt;
    if (!parse_cli(argc, argv, &opt)) {
        return 2;
    }

    std::vector<FramePair> pairs;
    if (!parse_association(opt.association_path, &pairs)) {
        pairs = pair_from_rgb_depth_lists(opt.tum_dir, opt.max_dt_sec);
    }
    if (pairs.empty()) {
        std::cerr << "No valid frame pairs found.\n";
        return 1;
    }
    if (opt.max_frames > 0 && pairs.size() > opt.max_frames) {
        pairs.resize(opt.max_frames);
    }

    std::cout << "Pairs: " << pairs.size() << "\n";
    std::cout << "Vocabulary: " << opt.vocab_path << "\n";
    std::cout << "Settings: " << opt.settings_path << "\n";
    std::cout << "Depth scale: " << opt.depth_scale << "\n";

    auto system = Orb::make_rgbd_system(opt.vocab_path.string(),
                                        opt.settings_path.string(), false);
    std::size_t processed = 0;
    for (const auto& p : pairs) {
        const auto rgb_path = (opt.tum_dir / p.rgb_rel).string();
        const auto depth_path = (opt.tum_dir / p.depth_rel).string();

        cv::Mat rgb = cv::imread(rgb_path, cv::IMREAD_UNCHANGED);
        cv::Mat depth = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
        if (rgb.empty() || depth.empty()) {
            std::cerr << "Failed to load frame rgb=" << rgb_path
                      << " depth=" << depth_path << "\n";
            return 1;
        }
        if (!convert_depth_to_meters(&depth, opt.depth_scale)) {
            std::cerr << "Unsupported depth type at " << depth_path
                      << " type=" << depth.type() << "\n";
            return 1;
        }

        (void)Orb::track_rgbd(*system, rgb, depth, p.t);
        ++processed;
        if (processed % 100 == 0 || processed == pairs.size()) {
            std::cout << "Processed " << processed << "/" << pairs.size()
                      << "\n";
        }
    }

    Orb::shutdown_system(*system);
    std::cout << "Done. Processed frames: " << processed << "\n";
    return 0;
}
