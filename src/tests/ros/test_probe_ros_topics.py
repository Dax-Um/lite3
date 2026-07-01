import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "probe_ros_topics.py"


def load_script():
    spec = importlib.util.spec_from_file_location("probe_ros_topics", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_topic_stats_reports_missing_before_first_message():
    script = load_script()
    stats = script.TopicStats("/scan")

    row = stats.as_row(now=10.0, min_hz=1.0, stale_sec=0.5)

    assert row == {
        "topic": "/scan",
        "seen": "false",
        "count": "0",
        "hz": "0.0",
        "last_age_sec": "",
        "status": "missing",
    }


def test_topic_stats_reports_ok_when_seen_fresh_and_fast_enough():
    script = load_script()
    stats = script.TopicStats("/scan")
    stats.mark_seen(10.0)
    stats.mark_seen(10.1)
    stats.mark_seen(10.2)

    row = stats.as_row(now=10.25, min_hz=5.0, stale_sec=0.5)

    assert row["seen"] == "true"
    assert row["count"] == "3"
    assert row["hz"] == "10.0"
    assert row["last_age_sec"] == "0.05"
    assert row["status"] == "ok"


def test_topic_stats_reports_stale_when_last_message_is_too_old():
    script = load_script()
    stats = script.TopicStats("/imu/data")
    stats.mark_seen(10.0)

    row = stats.as_row(now=10.6, min_hz=1.0, stale_sec=0.5)

    assert row["status"] == "stale"


def test_topic_stats_reports_slow_when_hz_is_below_minimum():
    script = load_script()
    stats = script.TopicStats("/leg_odom2")
    stats.mark_seen(10.0)
    stats.mark_seen(11.0)

    row = stats.as_row(now=11.1, min_hz=5.0, stale_sec=0.5)

    assert row["status"] == "slow"


def test_exit_code_is_success_only_when_all_rows_ok():
    script = load_script()

    assert script.exit_code_for_rows([{"status": "ok"}, {"status": "ok"}]) == 0
    assert script.exit_code_for_rows([{"status": "ok"}, {"status": "missing"}]) == 1
