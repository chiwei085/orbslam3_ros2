import pathlib
import sys

sys.path.append(str(pathlib.Path(__file__).resolve().parents[1] / "utils"))

from metrics import (
    compute_hz,
    compute_jitter_ms,
    compute_latency_ms,
    compute_processed_hz,
    percentile,
)


def test_metrics_threshold_example():
    receive = [0.000, 0.033, 0.066, 0.099, 0.132, 0.165]
    source = [t - 0.020 for t in receive]

    hz = compute_hz(receive)
    jitter_std_ms, jitter_p99_ms = compute_jitter_ms(receive)
    _, latency_p99_ms = compute_latency_ms(receive, source)

    assert hz >= 25.0
    assert jitter_p99_ms < 20.0
    assert jitter_std_ms < 20.0
    assert latency_p99_ms < 100.0


def test_percentile_empty_safe():
    assert percentile([], 0.99) == 0.0


def test_processed_hz_from_status_like_samples():
    sample_times = [0.0, 1.0, 2.0, 3.0]
    processed_counts = [10, 25, 40, 55]
    assert compute_processed_hz(sample_times, processed_counts) == 15.0
