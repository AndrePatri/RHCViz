import time


def perf_sleep(dt: float):
    """Sleep for dt seconds with the highest precision available."""
    perf_sleep_fn = getattr(time, "perf_sleep", None)
    if perf_sleep_fn is not None:
        perf_sleep_fn(dt)
    else:
        time.sleep(dt)
