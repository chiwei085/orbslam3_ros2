import math
import statistics


def percentile(values, p):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(math.ceil(p * len(ordered)) - 1)))
    return float(ordered[index])


def compute_hz(receive_times_s):
    if len(receive_times_s) < 2:
        return 0.0
    duration = receive_times_s[-1] - receive_times_s[0]
    if duration <= 0.0:
        return 0.0
    return float((len(receive_times_s) - 1) / duration)


def compute_jitter_ms(receive_times_s):
    if len(receive_times_s) < 3:
        return 0.0, 0.0
    intervals_ms = [
        (receive_times_s[i + 1] - receive_times_s[i]) * 1000.0
        for i in range(len(receive_times_s) - 1)
    ]
    mean_interval = statistics.mean(intervals_ms)
    jitter_abs = [abs(v - mean_interval) for v in intervals_ms]
    return statistics.pstdev(intervals_ms), percentile(jitter_abs, 0.99)


def compute_latency_ms(receive_times_s, source_times_s):
    latencies = [
        max(0.0, (recv - src) * 1000.0) for recv, src in zip(receive_times_s, source_times_s)
    ]
    if not latencies:
        return 0.0, 0.0
    return statistics.mean(latencies), percentile(latencies, 0.99)


def compute_processed_hz(sample_times_s, processed_counts):
    if len(sample_times_s) < 2 or len(processed_counts) < 2:
        return 0.0
    duration = sample_times_s[-1] - sample_times_s[0]
    if duration <= 0.0:
        return 0.0
    delta = processed_counts[-1] - processed_counts[0]
    if delta <= 0:
        return 0.0
    return float(delta / duration)
