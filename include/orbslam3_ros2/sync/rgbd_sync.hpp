#pragma once

/**
 * @file rgbd_sync.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 *
 * The synchronizer is bounded and non-blocking. It keeps two FIFO queues
 * (RGB and depth). Each push runs one light pipeline:
 * 1. Timestamp sanity checks.
 * 2. Push to the target queue.
 * 3. Queue overflow trim.
 * 4. Span guard check.
 * 5. At most one match attempt.
 *
 * Matching is deterministic:
 * - Anchor = earlier one between rgb_q_.front() and depth_q_.front().
 * - Candidate = nearest timestamp in the opposite queue.
 * - If nearest delta <= max_dt_ns, produce one pair.
 *
 * If no pair is possible in the current window:
 * - First run a precision stale-front check.
 * - If one side is clearly stale by more than max_dt_ns, drop that side.
 * - Otherwise drop the anchor as fallback to guarantee progress.
 *
 * Output includes t_track_ns (RGB timestamp) and dt_ns = t_rgb - t_depth
 * for sync quality diagnostics.
 */

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <tuple>
#include <utility>

namespace orbslam3_ros2::sync
{
/**
 * @brief RGB-Depth synchronizer
 *
 * - push_rgb(...) / push_depth(...): ingest one sample and return at most one
 *   synchronized pair.
 * - try_match_once(): run one matching step without new input.
 * - set_config(...): apply runtime knobs and re-enforce queue invariants.
 * - reset(...): clear queues and update reset counters.
 * - config(), stats(), rgb_queue_size(), depth_queue_size(): read-only runtime
 *   introspection helpers.
 */
template <typename RgbMsgT, typename DepthMsgT>
class RgbdSync
{
public:
    /**
     * @brief (rgb, depth, t_track_ns, dt_ns)
     */
    using Pair = std::tuple<RgbMsgT, DepthMsgT, std::uint64_t, std::int64_t>;

    /**
     * @brief Queue reset causes.
     */
    enum class ResetReason : std::uint8_t
    {
        kManual,
        kTimeBackwards,
        kHugeJump,
        kSpanGuard,
    };

    /**
     * @brief Runtime knobs for matching and guards.
     */
    struct Config
    {
        std::size_t queue_size = 30;
        std::uint64_t max_dt_ns = 10'000'000;

        bool enable_span_guard = true;
        std::uint64_t max_span_ns = 500'000'000;

        bool enable_huge_jump_reset = true;
        std::uint64_t huge_jump_ns = 2'000'000'000;

        bool reset_on_time_backwards = true;

        /**
         * @brief Clamp invalid values to safe defaults.
         */
        void validate_and_sanitize() noexcept {
            if (max_dt_ns == 0) {
                max_dt_ns = 1;
            }
        }
    };

    /**
     * @brief Runtime counters and timing diagnostics.
     */
    struct Stats
    {
        std::uint64_t paired = 0;

        std::uint64_t drop_rgb = 0;
        std::uint64_t drop_depth = 0;

        std::uint64_t drop_overflow_rgb = 0;
        std::uint64_t drop_overflow_depth = 0;

        std::uint64_t resets = 0;
        std::uint64_t resets_time_backwards = 0;
        std::uint64_t resets_huge_jump = 0;
        std::uint64_t resets_span_guard = 0;

        std::int64_t dt_last_ns = 0;
        std::uint64_t dt_max_abs_ns = 0;
        std::uint64_t dt_sum_abs_ns = 0;

        std::size_t rgb_q_peak = 0;
        std::size_t depth_q_peak = 0;

        /**
         * @brief Mean absolute dt over successful pairs.
         */
        std::uint64_t mean_dt_abs_ns() const noexcept {
            return (paired == 0) ? 0 : (dt_sum_abs_ns / paired);
        }
    };

    explicit RgbdSync(Config cfg = {}) : cfg_(cfg) {
        cfg_.validate_and_sanitize();
    }

    /**
     * @brief Apply new config and enforce immediate queue invariants.
     */
    void set_config(Config cfg) noexcept {
        cfg.validate_and_sanitize();
        cfg_ = cfg;

        trim_overflow_(rgb_q_, stats_.drop_overflow_rgb);
        trim_overflow_(depth_q_, stats_.drop_overflow_depth);
        update_peaks_();
        enforce_span_guard_();
    }

    const Config& config() const noexcept { return cfg_; }
    const Stats& stats() const noexcept { return stats_; }

    std::size_t rgb_queue_size() const noexcept { return rgb_q_.size(); }
    std::size_t depth_queue_size() const noexcept { return depth_q_.size(); }

    /**
     * @brief Clear queues and record reset reason.
     */
    void reset(ResetReason reason = ResetReason::kManual) noexcept {
        rgb_q_.clear();
        depth_q_.clear();
        last_seen_any_ns_.reset();

        ++stats_.resets;
        switch (reason) {
            case ResetReason::kTimeBackwards:
                ++stats_.resets_time_backwards;
                break;
            case ResetReason::kHugeJump:
                ++stats_.resets_huge_jump;
                break;
            case ResetReason::kSpanGuard:
                ++stats_.resets_span_guard;
                break;
            case ResetReason::kManual:
            default:
                break;
        }
    }

    /**
     * @brief Push one RGB sample and attempt one match.
     */
    [[nodiscard]] std::optional<Pair> push_rgb(RgbMsgT msg,
                                               std::uint64_t t_ns) noexcept {
        if (!ingest_time_(t_ns)) {
            return std::nullopt;
        }

        rgb_q_.push_back(ItemRgb{t_ns, std::move(msg)});
        trim_overflow_(rgb_q_, stats_.drop_overflow_rgb);
        update_peaks_();

        if (!enforce_span_guard_()) {
            return std::nullopt;
        }

        return try_match_once_();
    }

    /**
     * @brief Push one depth sample and attempt one match.
     */
    [[nodiscard]] std::optional<Pair> push_depth(DepthMsgT msg,
                                                 std::uint64_t t_ns) noexcept {
        if (!ingest_time_(t_ns)) {
            return std::nullopt;
        }

        depth_q_.push_back(ItemDepth{t_ns, std::move(msg)});
        trim_overflow_(depth_q_, stats_.drop_overflow_depth);
        update_peaks_();

        if (!enforce_span_guard_()) {
            return std::nullopt;
        }

        return try_match_once_();
    }

    /**
     * @brief Attempt one match without pushing data.
     */
    [[nodiscard]] std::optional<Pair> try_match_once() noexcept {
        return try_match_once_();
    }

private:
    enum class Stream : std::uint8_t
    {
        kRgb,
        kDepth,
    };

    enum class DropDecision : std::uint8_t
    {
        kNone,
        kDropRgb,
        kDropDepth,
    };

    struct ItemRgb
    {
        // Timestamp in nanoseconds.
        std::uint64_t t_ns;
        RgbMsgT msg;
    };

    struct ItemDepth
    {
        // Timestamp in nanoseconds.
        std::uint64_t t_ns;
        DepthMsgT msg;
    };

    // Absolute value for int64_t, returned as uint64_t.
    static constexpr std::uint64_t abs_u64_(std::int64_t x) noexcept {
        return (x >= 0) ? static_cast<std::uint64_t>(x)
                        : (static_cast<std::uint64_t>(-(x + 1)) + 1U);
    }

    // Absolute difference for non-negative timestamps.
    static constexpr std::uint64_t abs_diff_u64_(std::uint64_t a,
                                                 std::uint64_t b) noexcept {
        return (a >= b) ? (a - b) : (b - a);
    }

    // Signed (a - b) with int64_t saturation.
    static constexpr std::int64_t signed_diff_i64_(std::uint64_t a,
                                                   std::uint64_t b) noexcept {
        constexpr auto kI64MaxU = static_cast<std::uint64_t>(
            std::numeric_limits<std::int64_t>::max());
        constexpr auto kI64MinAbsU = kI64MaxU + 1U;

        if (a >= b) {
            const auto d = a - b;
            return (d > kI64MaxU) ? std::numeric_limits<std::int64_t>::max()
                                  : static_cast<std::int64_t>(d);
        }

        const auto d = b - a;
        if (d >= kI64MinAbsU) {
            return std::numeric_limits<std::int64_t>::min();
        }
        return -static_cast<std::int64_t>(d);
    }

    // Saturating add for uint64_t.
    static constexpr std::uint64_t sat_add_u64_(std::uint64_t a,
                                                std::uint64_t b) noexcept {
        const auto max_u64 = std::numeric_limits<std::uint64_t>::max();
        return (a > (max_u64 - b)) ? max_u64 : (a + b);
    }

    // Keep queue bounded and count dropped overflow items.
    template <typename Q>
    void trim_overflow_(Q& q, std::uint64_t& overflow_counter) noexcept {
        if (cfg_.queue_size == 0) {
            while (q.size() > 1) {
                q.pop_front();
                ++overflow_counter;
            }
            return;
        }

        while (q.size() > cfg_.queue_size) {
            q.pop_front();
            ++overflow_counter;
        }
    }

    // Track queue peak sizes.
    void update_peaks_() noexcept {
        if (rgb_q_.size() > stats_.rgb_q_peak) {
            stats_.rgb_q_peak = rgb_q_.size();
        }
        if (depth_q_.size() > stats_.depth_q_peak) {
            stats_.depth_q_peak = depth_q_.size();
        }
    }

    // Sanity checks for incoming timestamp stream.
    bool ingest_time_(std::uint64_t t_ns) noexcept {
        if (!last_seen_any_ns_) {
            last_seen_any_ns_ = t_ns;
            return true;
        }

        const auto last = *last_seen_any_ns_;
        if (cfg_.reset_on_time_backwards && t_ns < last) {
            reset(ResetReason::kTimeBackwards);
            last_seen_any_ns_ = t_ns;
            return false;
        }

        if (cfg_.enable_huge_jump_reset &&
            abs_diff_u64_(t_ns, last) > cfg_.huge_jump_ns) {
            reset(ResetReason::kHugeJump);
            last_seen_any_ns_ = t_ns;
            return false;
        }

        last_seen_any_ns_ = t_ns;
        return true;
    }

    // Reset if any queue span grows beyond max_span_ns.
    bool enforce_span_guard_() noexcept {
        if (!cfg_.enable_span_guard) {
            return true;
        }

        const auto span_rgb =
            rgb_q_.empty() ? 0U : (rgb_q_.back().t_ns - rgb_q_.front().t_ns);
        const auto span_depth =
            depth_q_.empty() ? 0U
                             : (depth_q_.back().t_ns - depth_q_.front().t_ns);

        if (span_rgb > cfg_.max_span_ns || span_depth > cfg_.max_span_ns) {
            reset(ResetReason::kSpanGuard);
            return false;
        }
        return true;
    }

    // Find nearest timestamp in one queue to anchor time.
    // Returns (index, best_abs_dt_ns).
    template <typename Q>
    std::tuple<std::size_t, std::uint64_t> best_match_index_(
        const Q& q, std::uint64_t t_anchor) const noexcept {
        std::size_t best_i = 0;
        std::uint64_t best_dt = std::numeric_limits<std::uint64_t>::max();

        for (std::size_t i = 0; i < q.size(); ++i) {
            const auto dt = abs_diff_u64_(q[i].t_ns, t_anchor);
            if (dt < best_dt) {
                best_dt = dt;
                best_i = i;
            }
        }

        return std::tuple<std::size_t, std::uint64_t>{best_i, best_dt};
    }

    // Precision stale test on queue fronts.
    // Returns kNone when neither side is clearly stale.
    DropDecision stale_drop_decision_(std::uint64_t t_rgb0,
                                      std::uint64_t t_depth0) const noexcept {
        if (sat_add_u64_(t_rgb0, cfg_.max_dt_ns) < t_depth0) {
            return DropDecision::kDropRgb;
        }
        if (sat_add_u64_(t_depth0, cfg_.max_dt_ns) < t_rgb0) {
            return DropDecision::kDropDepth;
        }
        return DropDecision::kNone;
    }

    // Drop one front item from selected stream.
    void drop_front_(Stream stream) noexcept {
        if (stream == Stream::kRgb) {
            rgb_q_.pop_front();
            ++stats_.drop_rgb;
            return;
        }

        depth_q_.pop_front();
        ++stats_.drop_depth;
    }

    // Apply stale-drop decision. Returns true when something was dropped.
    bool apply_drop_decision_(DropDecision decision) noexcept {
        if (decision == DropDecision::kDropRgb) {
            drop_front_(Stream::kRgb);
            return true;
        }
        if (decision == DropDecision::kDropDepth) {
            drop_front_(Stream::kDepth);
            return true;
        }
        return false;
    }

    // One match attempt for a chosen anchor stream.
    [[nodiscard]] std::optional<Pair> try_match_for_anchor_(
        Stream anchor, std::uint64_t t_rgb0, std::uint64_t t_depth0,
        std::uint64_t max_dt_u64) noexcept {
        if (anchor == Stream::kRgb) {
            const auto match = best_match_index_(depth_q_, t_rgb0);
            const auto j = std::get<0>(match);
            const auto best_dt = std::get<1>(match);

            if (best_dt <= max_dt_u64) {
                const auto t_rgb_match = t_rgb0;
                const auto t_depth_match = depth_q_[j].t_ns;

                auto rgb_msg = std::move(rgb_q_.front().msg);
                auto depth_msg = std::move(depth_q_[j].msg);

                rgb_q_.pop_front();
                depth_q_.erase(depth_q_.begin() +
                               static_cast<std::ptrdiff_t>(j));

                const auto dt_ns = signed_diff_i64_(t_rgb_match, t_depth_match);
                on_paired_(dt_ns);
                return Pair{std::move(rgb_msg), std::move(depth_msg),
                            t_rgb_match, dt_ns};
            }

            const auto decision = stale_drop_decision_(t_rgb0, t_depth0);
            if (apply_drop_decision_(decision)) {
                return std::nullopt;
            }

            // Fallback: drop anchor to guarantee progress.
            drop_front_(anchor);
            return std::nullopt;
        }

        const auto match = best_match_index_(rgb_q_, t_depth0);
        const auto i = std::get<0>(match);
        const auto best_dt = std::get<1>(match);

        if (best_dt <= max_dt_u64) {
            const auto t_rgb_match = rgb_q_[i].t_ns;
            const auto t_depth_match = t_depth0;

            auto rgb_msg = std::move(rgb_q_[i].msg);
            auto depth_msg = std::move(depth_q_.front().msg);

            depth_q_.pop_front();
            rgb_q_.erase(rgb_q_.begin() + static_cast<std::ptrdiff_t>(i));

            const auto dt_ns = signed_diff_i64_(t_rgb_match, t_depth_match);
            on_paired_(dt_ns);
            return Pair{std::move(rgb_msg), std::move(depth_msg), t_rgb_match,
                        dt_ns};
        }

        const auto decision = stale_drop_decision_(t_rgb0, t_depth0);
        if (apply_drop_decision_(decision)) {
            return std::nullopt;
        }

        // Fallback: drop anchor to guarantee progress.
        drop_front_(anchor);
        return std::nullopt;
    }

    // Update pair counters and dt quality statistics.
    void on_paired_(std::int64_t dt_ns) noexcept {
        ++stats_.paired;
        stats_.dt_last_ns = dt_ns;

        const auto abs_dt_u64 = abs_u64_(dt_ns);
        const auto sum = stats_.dt_sum_abs_ns;
        stats_.dt_sum_abs_ns =
            (abs_dt_u64 > (std::numeric_limits<std::uint64_t>::max() - sum))
                ? std::numeric_limits<std::uint64_t>::max()
                : (sum + abs_dt_u64);

        if (abs_dt_u64 > stats_.dt_max_abs_ns) {
            stats_.dt_max_abs_ns = abs_dt_u64;
        }
    }

    // Internal single-step matcher with bounded retries.
    [[nodiscard]] std::optional<Pair> try_match_once_() noexcept {
        // Bounded loop prevents pathological lockups.
        for (int iter = 0; iter < 64; ++iter) {
            if (rgb_q_.empty() || depth_q_.empty()) {
                return std::nullopt;
            }

            const auto t_rgb0 = rgb_q_.front().t_ns;
            const auto t_depth0 = depth_q_.front().t_ns;
            const auto anchor =
                (t_rgb0 <= t_depth0) ? Stream::kRgb : Stream::kDepth;

            const auto pair =
                try_match_for_anchor_(anchor, t_rgb0, t_depth0, cfg_.max_dt_ns);
            if (pair) {
                return pair;
            }
        }

        return std::nullopt;
    }

private:
    Config cfg_;
    Stats stats_;

    std::deque<ItemRgb> rgb_q_;
    std::deque<ItemDepth> depth_q_;
    std::optional<std::uint64_t> last_seen_any_ns_;
};

}  // namespace orbslam3_ros2::sync
