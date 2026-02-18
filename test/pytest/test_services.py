import time


def assert_bounded_rtt(samples_ms, p99_limit_ms):
    assert samples_ms, "no samples"
    ordered = sorted(samples_ms)
    p99_index = max(0, min(len(ordered) - 1, int(0.99 * len(ordered)) - 1))
    p99 = ordered[p99_index]
    assert p99 < p99_limit_ms, f"p99 RTT too high: {p99:.2f}ms >= {p99_limit_ms:.2f}ms"


def run_timed_calls(callable_fn, count):
    samples = []
    for _ in range(count):
        t0 = time.monotonic()
        callable_fn()
        samples.append((time.monotonic() - t0) * 1000.0)
    return samples


def test_bounded_rtt_example():
    samples = run_timed_calls(lambda: time.sleep(0.002), 10)
    assert_bounded_rtt(samples, p99_limit_ms=200.0)


def test_bounded_rtt_detects_outlier():
    samples = [1.0] * 9 + [300.0]
    try:
        assert_bounded_rtt(samples, p99_limit_ms=200.0)
        assert False, "expected bounded RTT check to fail"
    except AssertionError:
        pass
