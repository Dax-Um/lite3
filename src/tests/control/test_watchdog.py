from lite3_control.watchdog import CommandWatchdog


def test_watchdog_expires_before_first_output():
    watchdog = CommandWatchdog(timeout_sec=0.30)

    assert watchdog.expired(now=10.0) is True


def test_watchdog_mark_output_resets_timeout():
    watchdog = CommandWatchdog(timeout_sec=0.30)

    watchdog.mark_output(now=10.0)

    assert watchdog.expired(now=10.30) is False
    assert watchdog.expired(now=10.31) is True
