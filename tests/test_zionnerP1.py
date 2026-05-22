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

import pytest

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


def test_zionner_features_and_io(patched_zionner_clients):
    follower = ZionnerP1Follower(ZionnerP1FollowerConfig())
    leader = ZionnerP1Leader(ZionnerP1LeaderConfig())

    assert set(follower.observation_features) == set(ZIONNER_P1_JOINTS)
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

    assert observation == action == expected

    sent_action = follower.send_action(action)
    assert sent_action == action
    client_mock = follower.client
    assert isinstance(client_mock, MagicMock)
    client_mock.send_joint_positions.assert_called_once_with(
        [float(index) for index in range(7)]
    )

    leader.disconnect()
    follower.disconnect()


def test_zionner_hardware_roundtrip():
    if not _hardware_tests_enabled():
        pytest.skip("Disabled by LEROBOT_RUN_ZIONNERP1_HARDWARE_TESTS=0")

    follower_ip = os.getenv("LEROBOT_ZIONNERP1_FOLLOWER_IP", "169.254.160.182")
    leader_ip = os.getenv("LEROBOT_ZIONNERP1_LEADER_IP", "169.254.160.160")
    follower = ZionnerP1Follower(
        ZionnerP1FollowerConfig(ip_address=follower_ip)
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
