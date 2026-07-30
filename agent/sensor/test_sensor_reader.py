#!/usr/bin/env python3
from __future__ import annotations

from types import SimpleNamespace

from sensor_reader import lowstate_to_dict, parse_json_payload


def test_parse_json_payload() -> None:
    assert parse_json_payload('{"reward": 1.25}') == {"reward": 1.25}
    assert parse_json_payload("[1, 2, 3]") == {"value": [1, 2, 3]}
    assert parse_json_payload("not json") == {"raw": "not json"}


def test_lowstate_to_dict() -> None:
    msg = SimpleNamespace(
        tick=7,
        mode_machine=2,
        imu_state=SimpleNamespace(
            quaternion=[0.0, 0.0, 0.0, 1.0],
            accelerometer=[0.1, 0.2, 9.8],
            gyroscope=[0.3, 0.4, 0.5],
            rpy=[0.0, 0.1, 0.2],
        ),
        motor_state=[
            SimpleNamespace(q=1.0, dq=2.0, tau_est=3.0),
            SimpleNamespace(q=4.0, dq=5.0, tau_est=6.0),
        ],
    )

    data = lowstate_to_dict(msg)

    assert data["tick"] == 7
    assert data["mode_machine"] == 2
    assert data["imu"]["quaternion_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    assert data["joints"]["q"] == [1.0, 4.0]
    assert data["joints"]["dq"] == [2.0, 5.0]
    assert data["joints"]["tau_est"] == [3.0, 6.0]


if __name__ == "__main__":
    test_parse_json_payload()
    test_lowstate_to_dict()
