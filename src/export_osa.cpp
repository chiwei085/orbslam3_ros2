#include <Eigen/Core>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "Atlas.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "System.h"

namespace
{

struct Args
{
    std::string voc_file;
    std::string settings_file;
    std::string atlas_file;
    std::string out_ply{"map.ply"};
    std::string out_traj_xyz{"traj.xyz"};
    bool include_bad_mappoints{false};
    bool allow_empty_traj{false};
    bool no_traj{false};
    bool debug_dump{false};
};

struct PointStats
{
    size_t total{0};
    size_t null_count{0};
    size_t bad_count{0};
    size_t nonfinite_count{0};
    size_t kept_count{0};
};

struct KeyframeStats
{
    size_t total{0};
    size_t null_count{0};
    size_t bad_count{0};
    size_t nonfinite_count{0};
    size_t kept_count{0};
};

struct ExportSnapshot
{
    size_t map_point_count{0};
    bool write_points_with_bad_mappoints{false};
    bool used_bad_mappoint_fallback{false};
    std::vector<std::pair<unsigned long, Eigen::Vector3f>> keyframe_positions;
    std::string point_source;
    std::string keyframe_source;
};

ExportSnapshot capture_snapshot(ORB_SLAM3::Atlas& atlas, const Args& args);

std::string to_lower_ascii(std::string s) {
    for (char& ch : s) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return s;
}

bool has_osa_ext(const std::string& path) {
    if (path.size() < 4) {
        return false;
    }
    return to_lower_ascii(path.substr(path.size() - 4)) == ".osa";
}

std::string ensure_osa_ext(const std::string& path) {
    return has_osa_ext(path) ? path : path + ".osa";
}

std::filesystem::path to_absolute_normalized(const std::string& path_str) {
    std::error_code ec;
    const auto abs =
        std::filesystem::absolute(std::filesystem::path(path_str), ec);
    if (ec) {
        throw std::runtime_error("Failed to resolve absolute path for " +
                                 path_str + ": " + ec.message());
    }
    return abs.lexically_normal();
}

bool env_truthy(const char* name) {
    const char* value = std::getenv(name);
    if (!value) {
        return false;
    }
    const std::string v = to_lower_ascii(value);
    return v == "1" || v == "true" || v == "yes" || v == "on";
}

class ScopedCurrentPath
{
public:
    explicit ScopedCurrentPath(const std::filesystem::path& target) {
        std::error_code ec;
        old_path_ = std::filesystem::current_path(ec);
        if (ec) {
            throw std::runtime_error(
                "Failed to read current working directory: " + ec.message());
        }
        std::filesystem::current_path(target, ec);
        if (ec) {
            throw std::runtime_error("Failed to change working directory to " +
                                     target.string() + ": " + ec.message());
        }
        active_ = true;
    }

    ~ScopedCurrentPath() {
        if (!active_) {
            return;
        }
        std::error_code ec;
        std::filesystem::current_path(old_path_, ec);
    }

    ScopedCurrentPath(const ScopedCurrentPath&) = delete;
    ScopedCurrentPath& operator=(const ScopedCurrentPath&) = delete;

private:
    std::filesystem::path old_path_;
    bool active_{false};
};

bool load_atlas_with_parent_cwd(ORB_SLAM3::System& slam,
                                const std::filesystem::path& atlas_abs_path) {
    const auto parent = atlas_abs_path.parent_path().empty()
                            ? std::filesystem::current_path()
                            : atlas_abs_path.parent_path();
    const auto file_name = atlas_abs_path.filename();
    ScopedCurrentPath cwd_guard(parent);
    return slam.LoadAtlasFromFile(file_name.string());
}

void print_usage(std::ostream& os, const char* argv0) {
    os << "Usage: " << argv0
       << " --voc_file <path> --settings_file <path> --atlas_file <path> "
       << "[--out_ply <path>] [--out_traj_xyz <path>] "
       << "[--include_bad_mappoints] [--allow_empty_traj] [--no_traj] "
       << "[--debug_dump]\n";
}

Args parse_args(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto need_value = [&](const char* flag) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("Missing value for ") +
                                         flag);
            }
            return argv[++i];
        };

        if (k == "--voc_file") {
            args.voc_file = need_value("--voc_file");
        }
        else if (k == "--settings_file") {
            args.settings_file = need_value("--settings_file");
        }
        else if (k == "--atlas_file") {
            args.atlas_file = need_value("--atlas_file");
        }
        else if (k == "--out_ply") {
            args.out_ply = need_value("--out_ply");
        }
        else if (k == "--out_traj_xyz") {
            args.out_traj_xyz = need_value("--out_traj_xyz");
        }
        else if (k == "--no_traj") {
            args.no_traj = true;
        }
        else if (k == "--include_bad_mappoints") {
            args.include_bad_mappoints = true;
        }
        else if (k == "--allow_empty_traj") {
            args.allow_empty_traj = true;
        }
        else if (k == "--debug_dump") {
            args.debug_dump = true;
        }
        else if (k == "-h" || k == "--help") {
            print_usage(std::cout, argv[0]);
            std::exit(0);
        }
        else {
            throw std::runtime_error("Unknown argument: " + k);
        }
    }

    if (args.voc_file.empty() || args.settings_file.empty() ||
        args.atlas_file.empty()) {
        throw std::runtime_error(
            "Required arguments: --voc_file, --settings_file, --atlas_file");
    }
    return args;
}

void ensure_parent_dir(const std::string& path) {
    const auto parent = std::filesystem::path(path).parent_path();
    if (parent.empty()) {
        return;
    }
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (ec) {
        throw std::runtime_error("Failed to create output directory for " +
                                 path + ": " + ec.message());
    }
}

bool is_finite_vec3(const Eigen::Vector3f& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

void print_point_stats(const std::string& label, const PointStats& s) {
    std::cerr << "[export_osa] MapPoints(" << label << ") total=" << s.total
              << " null=" << s.null_count << " bad=" << s.bad_count
              << " nonfinite=" << s.nonfinite_count << " kept=" << s.kept_count
              << "\n";
}

void print_keyframe_stats(const std::string& label, const KeyframeStats& s) {
    std::cerr << "[export_osa] KeyFrames(" << label << ") total=" << s.total
              << " null=" << s.null_count << " bad=" << s.bad_count
              << " nonfinite=" << s.nonfinite_count << " kept=" << s.kept_count
              << "\n";
}

void debug_dump_atlas_public_raw_counts(ORB_SLAM3::System& slam,
                                        std::ostream& os) {
    ORB_SLAM3::Atlas* atlas = slam.GetAtlas();
    os << "[export_osa] system_ptr=" << &slam << " atlas_ptr=" << atlas << "\n";
    if (!atlas) {
        os << "[export_osa] atlas is null\n";
        return;
    }

    ORB_SLAM3::Map* current_map = nullptr;
    try {
        current_map = atlas->GetCurrentMap();
    }
    catch (...) {
        os << "[export_osa] atlas->GetCurrentMap() threw\n";
    }

    size_t all_maps_size = 0;
    atlas->ForEachMap([&](ORB_SLAM3::Map* m) {
        (void)m;
        ++all_maps_size;
    });

    os << "[export_osa] atlas.CountMaps()=" << atlas->CountMaps()
       << " ForEachMap.count=" << all_maps_size
       << " current_map_ptr=" << current_map << "\n";

    size_t idx = 0;
    atlas->ForEachMap([&](ORB_SLAM3::Map* m) {
        if (!m) {
            os << "[export_osa] maps[" << idx++ << "]=null\n";
            return;
        }
        os << "[export_osa] maps[" << idx << "] ptr=" << m
           << " id=" << m->GetId()
           << " raw_mspMapPoints=" << m->MapPointsInMapRaw()
           << " raw_mspKeyFrames=" << m->KeyFramesInMapRaw()
           << " is_bad=" << m->IsBad() << " is_in_use=" << m->IsInUse()
           << (m == current_map ? " <current>" : "") << "\n";
        ++idx;
    });
}

PointStats count_valid_mappoints_streaming(const ORB_SLAM3::Atlas& atlas,
                                           bool include_bad) {
    PointStats stats;
    atlas.ForEachMap([&](ORB_SLAM3::Map* map_ptr) {
        if (!map_ptr || map_ptr->IsBad()) {
            return;
        }
        map_ptr->ForEachMapPoint([&](ORB_SLAM3::MapPoint* mp) {
            ++stats.total;
            if (!mp) {
                ++stats.null_count;
                return;
            }
            const bool is_bad = mp->isBad();
            if (is_bad) {
                ++stats.bad_count;
                if (!include_bad) {
                    return;
                }
            }
            const Eigen::Vector3f pos = mp->GetWorldPos();
            if (!is_finite_vec3(pos)) {
                ++stats.nonfinite_count;
                return;
            }
            ++stats.kept_count;
        });
    });
    return stats;
}

void write_ply_binary_little_endian_xyz(const std::string& path,
                                        const ORB_SLAM3::Atlas& atlas,
                                        bool include_bad, size_t vertex_count) {
    ensure_parent_dir(path);
    std::ofstream os(path, std::ios::binary);
    if (!os.is_open()) {
        throw std::runtime_error("Failed to open PLY output: " + path);
    }

    os << "ply\n";
    os << "format binary_little_endian 1.0\n";
    os << "comment generated by orbslam3_export_osa\n";
    os << "element vertex " << vertex_count << "\n";
    os << "property float x\n";
    os << "property float y\n";
    os << "property float z\n";
    os << "end_header\n";

    size_t written = 0;
    atlas.ForEachMap([&](ORB_SLAM3::Map* map_ptr) {
        if (!map_ptr || map_ptr->IsBad()) {
            return;
        }
        map_ptr->ForEachMapPoint([&](ORB_SLAM3::MapPoint* mp) {
            if (!mp) {
                return;
            }
            if (mp->isBad() && !include_bad) {
                return;
            }
            const Eigen::Vector3f pos = mp->GetWorldPos();
            if (!is_finite_vec3(pos)) {
                return;
            }
            const float xyz[3] = {pos.x(), pos.y(), pos.z()};
            os.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
            ++written;
        });
    });

    if (written != vertex_count) {
        std::ostringstream oss;
        oss << "PLY vertex count mismatch: header=" << vertex_count
            << " written=" << written;
        throw std::runtime_error(oss.str());
    }
    if (!os.good()) {
        throw std::runtime_error("Failed while writing PLY output: " + path);
    }
}

std::pair<std::vector<std::pair<unsigned long, Eigen::Vector3f>>, KeyframeStats>
collect_keyframes_streaming(const ORB_SLAM3::Atlas& atlas) {
    KeyframeStats stats;
    std::vector<std::pair<unsigned long, Eigen::Vector3f>> out;

    atlas.ForEachMap([&](ORB_SLAM3::Map* map_ptr) {
        if (!map_ptr || map_ptr->IsBad()) {
            return;
        }
        map_ptr->ForEachKeyFrame([&](ORB_SLAM3::KeyFrame* kf) {
            ++stats.total;
            if (!kf) {
                ++stats.null_count;
                return;
            }
            if (kf->isBad()) {
                ++stats.bad_count;
                return;
            }
            const Eigen::Vector3f pos = kf->GetPoseInverse().translation();
            if (!is_finite_vec3(pos)) {
                ++stats.nonfinite_count;
                return;
            }
            out.emplace_back(kf->mnId, pos);
        });
    });

    stats.kept_count = out.size();
    return {std::move(out), stats};
}

ExportSnapshot capture_snapshot(ORB_SLAM3::Atlas& atlas, const Args& args) {
    ExportSnapshot snapshot;
    snapshot.point_source = "all_nonbad_maps";
    snapshot.keyframe_source = "all_nonbad_maps";

    if (args.include_bad_mappoints) {
        snapshot.write_points_with_bad_mappoints = true;
        const PointStats point_stats =
            count_valid_mappoints_streaming(atlas, /*include_bad=*/true);
        print_point_stats(snapshot.point_source + "/include_bad", point_stats);
        snapshot.map_point_count = point_stats.kept_count;
    }
    else {
        const PointStats strict_stats =
            count_valid_mappoints_streaming(atlas, /*include_bad=*/false);
        print_point_stats(snapshot.point_source + "/strict", strict_stats);

        if (strict_stats.kept_count > 0) {
            snapshot.map_point_count = strict_stats.kept_count;
            snapshot.write_points_with_bad_mappoints = false;
        }
        else {
            const PointStats fallback_stats =
                count_valid_mappoints_streaming(atlas, /*include_bad=*/true);
            print_point_stats(snapshot.point_source + "/include_bad",
                              fallback_stats);
            snapshot.map_point_count = fallback_stats.kept_count;
            if (fallback_stats.kept_count > 0) {
                std::cerr << "Warning: all MapPoints are marked bad after "
                             "LoadAtlasFromFile; exporting them anyway.\n";
                snapshot.write_points_with_bad_mappoints = true;
                snapshot.used_bad_mappoint_fallback = true;
            }
        }
    }

    auto [kfs, kf_stats] = collect_keyframes_streaming(atlas);
    print_keyframe_stats(snapshot.keyframe_source, kf_stats);
    snapshot.keyframe_positions = std::move(kfs);

    std::sort(snapshot.keyframe_positions.begin(),
              snapshot.keyframe_positions.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    return snapshot;
}

void write_traj_xyz(
    const std::string& path,
    const std::vector<std::pair<unsigned long, Eigen::Vector3f>>& kfs) {
    ensure_parent_dir(path);
    std::ofstream os(path);
    if (!os.is_open()) {
        throw std::runtime_error("Failed to open trajectory output: " + path);
    }

    os << std::setprecision(9);
    for (const auto& [id, p] : kfs) {
        (void)id;
        os << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
    }
    if (!os.good()) {
        throw std::runtime_error("Failed while writing trajectory output: " +
                                 path);
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);

        const auto voc_file_path = to_absolute_normalized(args.voc_file);
        const auto settings_file_path =
            to_absolute_normalized(args.settings_file);
        const auto atlas_file_path =
            to_absolute_normalized(ensure_osa_ext(args.atlas_file));

        if (!std::filesystem::exists(voc_file_path)) {
            std::cerr << "Error: voc_file not found: " << voc_file_path.string()
                      << "\n";
            return 2;
        }
        if (!std::filesystem::exists(settings_file_path)) {
            std::cerr << "Error: settings_file not found: "
                      << settings_file_path.string() << "\n";
            return 2;
        }
        if (!std::filesystem::exists(atlas_file_path)) {
            std::cerr << "Error: atlas_file not found: "
                      << atlas_file_path.string() << "\n";
            return 2;
        }

        auto slam = std::make_unique<ORB_SLAM3::System>(
            voc_file_path.string(), settings_file_path.string(),
            ORB_SLAM3::System::RGBD, false);

        if (!load_atlas_with_parent_cwd(*slam, atlas_file_path)) {
            std::cerr
                << "Error: failed to load atlas via public ORB_SLAM3 API: "
                << atlas_file_path.string() << "\n";
            return 3;
        }
        if (args.debug_dump || env_truthy("ORBSLAM3_EXPORT_OSA_DEBUG_DUMP")) {
            debug_dump_atlas_public_raw_counts(*slam, std::cerr);
        }

        ORB_SLAM3::Atlas* atlas = slam->GetAtlas();
        if (!atlas) {
            std::cerr << "Error: loaded system returned null atlas pointer\n";
            return 4;
        }

        ExportSnapshot snapshot = capture_snapshot(*atlas, args);

        if (snapshot.map_point_count == 0) {
            std::cerr << "Error: atlas contains no valid MapPoints\n";
            return 5;
        }
        if (snapshot.keyframe_positions.empty() && !args.allow_empty_traj &&
            !args.no_traj) {
            std::cerr << "Error: atlas contains no valid KeyFrames\n";
            return 6;
        }

        write_ply_binary_little_endian_xyz(
            args.out_ply, *atlas, snapshot.write_points_with_bad_mappoints,
            snapshot.map_point_count);
        if (!args.no_traj) {
            write_traj_xyz(args.out_traj_xyz, snapshot.keyframe_positions);
        }

        std::cout << "Exported atlas\n"
                  << "  atlas_file: " << atlas_file_path.string() << "\n"
                  << "  point_src:   " << snapshot.point_source << "\n"
                  << "  kf_src:      " << snapshot.keyframe_source << "\n"
                  << "  map_points: " << snapshot.map_point_count << "\n"
                  << "  keyframes:  " << snapshot.keyframe_positions.size()
                  << "\n"
                  << "  out_ply:    " << args.out_ply << "\n";
        if (args.no_traj) {
            std::cout << "  out_traj:   (disabled via --no_traj)\n";
        }
        else if (snapshot.keyframe_positions.empty()) {
            std::cout << "  out_traj:   (skipped: no keyframes, allowed)\n";
        }
        else {
            std::cout << "  out_traj:   " << args.out_traj_xyz << "\n";
        }

        // Avoid implicit autosave side effects.
        slam.reset();
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        print_usage(std::cerr, argc > 0 ? argv[0] : "orbslam3_export_osa");
        return 1;
    }
}
