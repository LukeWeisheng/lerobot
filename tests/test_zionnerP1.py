#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from lerobot.cameras.gemini335l import Gemini335LCameraConfig
from lerobot.robots import make_robot_from_config
from lerobot.robots.zionnerP1_follower import (
    ZIONNER_P1_JOINTS,
    ZionnerP1Follower,
    ZionnerP1FollowerConfig,
)
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.teleoperators.zionnerP1_leader import (
    ZionnerP1Leader,
    ZionnerP1LeaderConfig,
)


def _hardware_tests_enabled() -> bool:
    env_value = os.getenv("LEROBOT_RUN_ZIONNERP1_HARDWARE_TESTS", "1")
    return env_value.lower() not in {"0", "false", "no"}


def _make_client_mock(*args, **kwargs):
    client = MagicMock(name="XCoreZionnerP1ClientMock")
    client.is_connected = False
    client.read_joint_positions.return_value = [
        float(index) for index in range(7)
    ]

    def connect():
        client.is_connected = True

    def disconnect():
        client.is_connected = False

    client.connect.side_effect = connect
    client.disconnect.side_effect = disconnect
    client.send_joint_positions.side_effect = (
        lambda joint_positions: joint_positions
    )
    client.local_ip_address = kwargs.get("local_ip_address")
    return client


def _make_camera_mock(config):
    camera = MagicMock(name=f"CameraMock:{getattr(config, 'serial_number_or_name', 'cam')}")
    camera.is_connected = False
    camera.use_depth = getattr(config, "use_depth", False)

    color = np.zeros((config.height, config.width, 3), dtype=np.uint8)
    depth = None
    if camera.use_depth:
        depth = np.full(
            (config.depth_height, config.depth_width),
            1024,
            dtype=np.uint16,
        )

    def connect():
        camera.is_connected = True

    def disconnect():
        camera.is_connected = False

    camera.connect.side_effect = connect
    camera.disconnect.side_effect = disconnect
    camera.async_read.return_value = color
    camera.read_frame_bundle.return_value = {"color": color, "depth": depth}
    return camera


@pytest.fixture
def patched_zionner_clients():
    with (
        patch(
            (
                "lerobot.robots.zionnerP1_follower."
                "zionnerP1_follower.XCoreZionnerP1Client"
            ),
            side_effect=_make_client_mock,
        ),
        patch(
            (
                "lerobot.teleoperators.zionnerP1_leader."
                "zionnerP1_leader.XCoreZionnerP1Client"
            ),
            side_effect=_make_client_mock,
        ),
        patch(
            "lerobot.cameras.gemini335l.Gemini335LCamera",
            side_effect=_make_camera_mock,
        ),
        patch(
            "lerobot.cameras.utils.Gemini335LCamera",
            side_effect=_make_camera_mock,
            create=True,
        ),
    ):
        yield


def test_make_zionner_devices_from_config(patched_zionner_clients):
    follower = make_robot_from_config(ZionnerP1FollowerConfig())
    leader = make_teleoperator_from_config(ZionnerP1LeaderConfig())

    assert isinstance(follower, ZionnerP1Follower)
    assert isinstance(leader, ZionnerP1Leader)
    assert follower.config.type == "zionnerP1_follower"
    assert leader.config.type == "zionnerP1_leader"
    assert follower.config.use_realtime is True
    assert set(follower.config.cameras) == {"arm_camera", "head_camera"}


def test_zionner_features_and_io(patched_zionner_clients):
    follower = ZionnerP1Follower(ZionnerP1FollowerConfig())
    leader = ZionnerP1Leader(ZionnerP1LeaderConfig())

    expected_observation_keys = set(ZIONNER_P1_JOINTS) | {
        "arm_camera",
        "arm_camera_depth",
        "head_camera",
        "head_camera_depth",
    }
    assert set(follower.observation_features) == expected_observation_keys
    assert set(follower.action_features) == set(ZIONNER_P1_JOINTS)
    assert set(leader.action_features) == set(ZIONNER_P1_JOINTS)
    assert leader.feedback_features == {}

    follower.connect()
    leader.connect()

    observation = follower.get_observation()
    action = leader.get_action()
    expected = {
        joint_name: float(index)
        for index, joint_name in enumerate(ZIONNER_P1_JOINTS)
    }

    for joint_name, joint_value in expected.items():
        assert observation[joint_name] == action[joint_name] == joint_value
    assert observation["arm_camera"].shape == (480, 640, 3)
    assert observation["head_camera"].shape == (480, 640, 3)
    assert observation["arm_camera_depth"].shape == (480, 640, 3)
    assert observation["head_camera_depth"].shape == (480, 640, 3)

    sent_action = follower.send_action(action)
    assert sent_action == action
    client_mock = follower.client
    assert isinstance(client_mock, MagicMock)
    client_mock.send_joint_positions.assert_called_once_with(
        [float(index) for index in range(7)]
    )

    leader.disconnect()
    follower.disconnect()


def test_default_camera_config_uses_serial_mapping():
    config = ZionnerP1FollowerConfig(enable_default_cameras=True)

    arm_config = config.cameras["arm_camera"]
    head_config = config.cameras["head_camera"]

    assert isinstance(arm_config, Gemini335LCameraConfig)
    assert isinstance(head_config, Gemini335LCameraConfig)
    assert arm_config.serial_number_or_name == "CP2N1630006X"
    assert head_config.serial_number_or_name == "CP26363000BJ"
    assert arm_config.use_depth is True
    assert head_config.use_depth is True
    assert arm_config.width == 640
    assert arm_config.height == 480
    assert arm_config.depth_width == 640
    assert arm_config.depth_height == 480
    assert head_config.width == 640
    assert head_config.height == 480
    assert head_config.depth_width == 640
    assert head_config.depth_height == 480


def test_depth_pack_roundtrip():
    from lerobot.cameras.gemini335l.camera_gemini335l import Gemini335LCamera

    depth = np.array([[0, 1, 255], [256, 1024, 65535]], dtype=np.uint16)
    packed = Gemini335LCamera.pack_depth_for_storage(depth)
    unpacked = Gemini335LCamera.unpack_depth_from_storage(packed)

    assert packed.shape == (2, 3, 3)
    assert packed.dtype == np.uint8
    assert np.array_equal(unpacked, depth)


def test_zionner_hardware_roundtrip():
    if not _hardware_tests_enabled():
        pytest.skip("Disabled by LEROBOT_RUN_ZIONNERP1_HARDWARE_TESTS=0")

    follower_ip = os.getenv("LEROBOT_ZIONNERP1_FOLLOWER_IP", "169.254.160.182")
    leader_ip = os.getenv("LEROBOT_ZIONNERP1_LEADER_IP", "169.254.160.160")
    follower = ZionnerP1Follower(
        ZionnerP1FollowerConfig(
            ip_address=follower_ip,
            enable_default_cameras=False,
        )
    )
    leader = ZionnerP1Leader(ZionnerP1LeaderConfig(ip_address=leader_ip))

    try:
        follower.connect()
        leader.connect()

        follower_observation = follower.get_observation()
        leader_action = leader.get_action()

        assert set(follower_observation) == set(ZIONNER_P1_JOINTS)
        assert set(leader_action) == set(ZIONNER_P1_JOINTS)
        assert all(
            isinstance(value, float) for value in follower_observation.values()
        )
        assert all(
            isinstance(value, float) for value in leader_action.values()
        )

        sent_action = follower.send_action(follower_observation)
        assert sent_action == follower_observation
    finally:
        if leader.is_connected:
            leader.disconnect()
        if follower.is_connected:
            follower.disconnect()
