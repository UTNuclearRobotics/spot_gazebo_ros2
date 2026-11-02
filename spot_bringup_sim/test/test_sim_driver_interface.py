"""The sim driver must offer what the real driver offers, under the same names and types.

Code that drives Spot in simulation (e-stop, power, stand, velocity) is meant to run
unchanged on the robot, so every service, status topic and command topic it uses has to
exist on both drivers with the same type. This reads both sources rather than running
them: the real driver needs a robot.

    python3 -m pytest src/spot_gazebo_ros2/spot_bringup_sim/test/test_sim_driver_interface.py
"""

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
REAL = os.path.join(SRC, "spot_ros", "spot_driver", "spot_driver", "spot_ros.py")
SIM = os.path.join(HERE, "..", "scripts", "sim_spot_driver.py")

# What a client relying on the body's semantics uses. Extend as clients use more.
SERVICES = [
    "~/claim", "~/release", "~/stop", "~/sit", "~/stand", "~/power_on", "~/power_off",
    "~/estop/hard", "~/estop/gentle", "~/estop/release",
]
PUBLISHERS = ["~/status/power_state", "~/status/estop", "~/status/feedback"]
SUBSCRIPTIONS = ["~/cmd_vel", "~/body_pose"]


def declared(path: str, call: str) -> dict:
    """`{name: type}` for every `create_<call>(Type, 'name'` in a source file."""
    text = open(path).read()
    pattern = rf"create_{call}\(\s*(\w+)\s*,\s*['\"]([^'\"]+)['\"]"
    return {name: ty for ty, name in re.findall(pattern, text)}


def check(call: str, names: list):
    real, sim = declared(REAL, call), declared(SIM, call)
    for name in names:
        assert name in real, f"the real driver has no {call} '{name}' -- update this list"
        assert name in sim, f"the sim driver lacks {call} '{name}' ({real[name]})"
        assert sim[name] == real[name], f"{call} '{name}': sim {sim[name]}, real {real[name]}"


def test_services_match_the_real_driver():
    check("service", SERVICES)


def test_status_topics_match_the_real_driver():
    check("publisher", PUBLISHERS)


def test_command_topics_match_the_real_driver():
    check("subscription", SUBSCRIPTIONS)
