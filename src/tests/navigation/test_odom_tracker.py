from lite3_common.types import Pose2D
from lite3_navigation.odom_tracker import OdomTracker, OdomTrackerConfig


def test_start_session_sets_home_pose():
    tracker = OdomTracker()
    home = Pose2D(1.0, 2.0, 0.3)

    tracker.start_session(home, now=10.0)

    assert tracker.active() is True
    assert tracker.home_pose() == home
    assert tracker.current_pose() == home
    assert tracker.path_trace()[0].pose == home


def test_sample_adds_point_after_time_and_distance():
    tracker = OdomTracker(OdomTrackerConfig(sample_period_sec=0.2, min_distance_step_m=0.05))
    tracker.start_session(Pose2D(0.0, 0.0, 0.0), now=10.0)

    tracker.sample(Pose2D(0.06, 0.0, 0.0), now=10.21)

    assert [point.pose for point in tracker.path_trace()] == [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.06, 0.0, 0.0),
    ]


def test_sample_ignores_too_close_points():
    tracker = OdomTracker(OdomTrackerConfig(sample_period_sec=0.2, min_distance_step_m=0.05))
    tracker.start_session(Pose2D(0.0, 0.0, 0.0), now=10.0)

    tracker.sample(Pose2D(0.04, 0.0, 0.0), now=10.21)

    assert tracker.current_pose() == Pose2D(0.04, 0.0, 0.0)
    assert len(tracker.path_trace()) == 1


def test_sample_ignores_inactive_session_for_path_append():
    tracker = OdomTracker()
    tracker.start_session(Pose2D(0.0, 0.0, 0.0), now=10.0)
    tracker.stop_session()

    tracker.sample(Pose2D(1.0, 0.0, 0.0), now=11.0)

    assert tracker.current_pose() == Pose2D(1.0, 0.0, 0.0)
    assert len(tracker.path_trace()) == 1


def test_path_trace_returns_copy():
    tracker = OdomTracker()
    tracker.start_session(Pose2D(0.0, 0.0, 0.0), now=10.0)
    trace = tracker.path_trace()

    trace.clear()

    assert len(tracker.path_trace()) == 1
