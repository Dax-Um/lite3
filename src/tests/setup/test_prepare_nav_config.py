import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace


SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "perception_host_prepare_nav_config.py"


def load_script():
    spec = importlib.util.spec_from_file_location("prepare_nav_config", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def install_fake_parameter_ros(monkeypatch, module, *, invalid_name=None):
    events = {"destroyed_clients": [], "destroyed_node": False, "shutdown": False}

    class ParameterType:
        PARAMETER_INTEGER = 2
        PARAMETER_DOUBLE = 3

    class GetParameters:
        class Request:
            def __init__(self):
                self.names = []

    expected = {}
    for (node_name, subsection, key), raw_value in module.EXPECTED.items():
        name = "{}.{}".format(subsection, key) if subsection else key
        wanted = (
            int(raw_value)
            if key in {"vx_samples", "vy_samples"}
            else float(raw_value)
        )
        expected[("/" + node_name, name)] = wanted

    class FakeFuture:
        def __init__(self, response):
            self._response = response

        def done(self):
            return True

        def result(self):
            return self._response

    class FakeClient:
        def __init__(self, service_name):
            self.target = service_name[: -len("/get_parameters")]

        def wait_for_service(self, *, timeout_sec):
            events.setdefault("service_timeouts", []).append(timeout_sec)
            return True

        def call_async(self, request):
            values = []
            for name in request.names:
                wanted = expected[(self.target, name)]
                actual = float("nan") if name == invalid_name else wanted
                if isinstance(wanted, int):
                    values.append(
                        SimpleNamespace(
                            type=ParameterType.PARAMETER_INTEGER,
                            integer_value=actual,
                            double_value=0.0,
                        )
                    )
                else:
                    values.append(
                        SimpleNamespace(
                            type=ParameterType.PARAMETER_DOUBLE,
                            integer_value=0,
                            double_value=actual,
                        )
                    )
            return FakeFuture(SimpleNamespace(values=values))

    class FakeNode:
        def create_client(self, service_type, service_name):
            assert service_type is GetParameters
            client = FakeClient(service_name)
            events.setdefault("services", []).append(service_name)
            return client

        def destroy_client(self, client):
            events["destroyed_clients"].append(client)

        def destroy_node(self):
            events["destroyed_node"] = True

    rclpy = ModuleType("rclpy")
    rclpy.__path__ = []
    rclpy.init = lambda args=None: events.setdefault("init_args", args)
    rclpy.create_node = lambda name: FakeNode()
    rclpy.spin_once = lambda node, timeout_sec: None

    def shutdown():
        events["shutdown"] = True

    rclpy.shutdown = shutdown

    interfaces = ModuleType("rcl_interfaces")
    interfaces.__path__ = []
    interfaces_msg = ModuleType("rcl_interfaces.msg")
    interfaces_msg.ParameterType = ParameterType
    interfaces_srv = ModuleType("rcl_interfaces.srv")
    interfaces_srv.GetParameters = GetParameters

    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rcl_interfaces", interfaces)
    monkeypatch.setitem(sys.modules, "rcl_interfaces.msg", interfaces_msg)
    monkeypatch.setitem(sys.modules, "rcl_interfaces.srv", interfaces_srv)
    return events


UNSAFE = """planner_server:
  ros__parameters:
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.15
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    progress_checker:
      required_movement_radius: 0.15
      movement_time_allowance: 20.0
    goal_checker:
      xy_goal_tolerance: 0.20
      yaw_goal_tolerance: 0.20
    FollowPath:
      min_vel_x: 1.0
      max_vel_x: 0.3
      min_speed_xy: 1.0
      max_speed_xy: 0.3
      vx_samples: 1
      acc_lim_x: 10.0
      decel_lim_x: -10.0
      min_vel_y: -0.2
      max_vel_y: 0.2
      acc_lim_y: 0.2
      decel_lim_y: -0.2
      vy_samples: 5
      PreferForward.strafe_x: 1.0
      xy_goal_tolerance: 0.15
"""


def test_rewrite_applies_precise_planner_goal_progress_and_lateral_values():
    module = load_script()

    updated, changes = module.rewrite_nav_config(UNSAFE)

    assert len(changes) == len(module.EXPECTED)
    assert "tolerance: 1.0" in updated
    assert "controller_frequency: 5.0" in updated
    assert "required_movement_radius: 0.5" in updated
    assert "movement_time_allowance: 10.0" in updated
    assert "xy_goal_tolerance: 0.3" in updated
    assert "yaw_goal_tolerance: 0.25" in updated
    assert "min_vel_x: 0.0" in updated
    assert "max_vel_x: 1.0" in updated
    assert "min_speed_xy: 0.0" in updated
    assert "max_speed_xy: 1.0" in updated
    assert "vx_samples: 21" in updated
    assert "acc_lim_x: 1.0" in updated
    assert "decel_lim_x: -1.0" in updated
    assert "PreferForward.strafe_x: 0.3" in updated
    assert "max_vel_y: 0.0" in updated
    assert "vy_samples: 1" in updated
    assert "      xy_goal_tolerance: 0.15" in updated


def test_rewrite_is_idempotent():
    module = load_script()
    updated, _ = module.rewrite_nav_config(UNSAFE)

    repeated, changes = module.rewrite_nav_config(updated)

    assert repeated == updated
    assert changes == []


def test_live_parameter_check_accepts_exact_foxy_parameter_response(monkeypatch):
    module = load_script()
    events = install_fake_parameter_ros(monkeypatch, module)

    assert module.check_live_parameters(timeout_sec=0.25) is True
    assert events["services"] == [
        "/planner_server/get_parameters",
        "/controller_server/get_parameters",
    ]
    assert len(events["destroyed_clients"]) == 2
    assert events["destroyed_node"] is True
    assert events["shutdown"] is True


def test_live_parameter_check_rejects_nonfinite_value(monkeypatch):
    module = load_script()
    events = install_fake_parameter_ros(
        monkeypatch, module, invalid_name="FollowPath.max_vel_y"
    )

    assert module.check_live_parameters(timeout_sec=0.25) is False
    assert len(events["destroyed_clients"]) == 2
    assert events["shutdown"] is True
