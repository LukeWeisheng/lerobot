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

import logging
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat
from typing import Any

from lerobot.configs import parser
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.pipeline_features import (
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.processor import make_default_processors
from lerobot.robots.zionnerP1_follower import (
    ZIONNER_P1_JOINTS,
    ZionnerP1Follower,
    ZionnerP1FollowerConfig,
)
from lerobot.robots.vendor.xcore_sdk import XCoreZionnerP1Client
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import is_headless
from lerobot.utils.import_utils import register_third_party_devices
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging, log_say


def init_q_keyboard_listener() -> tuple[Any | None, dict[str, bool]]:
    events = {
        "stop": False,
        "exit_early": False,
    }

    if is_headless():
        logging.warning(
            "Headless environment detected. "
            "Keyboard inputs will not be available. "
            "Use duration_s to stop automatically."
        )
        return None, events

    from pynput import keyboard

    def on_press(key):
        try:
            key_char = getattr(key, "char", None)
            if key_char is not None and key_char.lower() == "q":
                print("q pressed. Stopping...")
                events["stop"] = True
                events["exit_early"] = True
            elif key == keyboard.Key.esc:
                print("Escape key pressed. Stopping...")
                events["stop"] = True
                events["exit_early"] = True
        except Exception as error:
            print(f"Error handling key press: {error}")

    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    return listener, events


@dataclass
class TrajectoryDatasetConfig:
    repo_id: str = "local/zionnerp1_manual_trajectory"
    root: str | Path | None = None
    single_task: str = "manual_drag_teach"
    fps: int = 30
    episode: int = 0
    duration_s: float | None = None
    push_to_hub: bool = False
    private: bool = False
    tags: list[str] | None = None


@dataclass
class ReplayRecordDatasetConfig:
    repo_id: str = "local/zionnerp1_replay_capture"
    root: str | Path | None = None
    single_task: str = "replay_capture"
    fps: int | None = None
    video: bool = True
    push_to_hub: bool = False
    private: bool = False
    tags: list[str] | None = None
    num_image_writer_processes: int = 0
    num_image_writer_threads_per_camera: int = 4
    video_encoding_batch_size: int = 1


@dataclass
class TwoStageRecordConfig:
    mode: str = "teach"
    robot: ZionnerP1FollowerConfig = field(
        default_factory=ZionnerP1FollowerConfig
    )
    trajectory: TrajectoryDatasetConfig = field(
        default_factory=TrajectoryDatasetConfig
    )
    dataset: ReplayRecordDatasetConfig = field(
        default_factory=ReplayRecordDatasetConfig
    )
    teach_power_off: bool = True
    display_data: bool = False
    play_sounds: bool = True


def _joint_observation_from_positions(
    joint_positions: list[float],
) -> dict[str, float]:
    return {
        joint_name: value
        for joint_name, value in zip(
            ZIONNER_P1_JOINTS,
            joint_positions,
            strict=True,
        )
    }


def _build_joint_only_features() -> dict[str, dict[str, Any]]:
    _, robot_action_processor, robot_observation_processor = (
        make_default_processors()
    )
    joint_features = dict.fromkeys(ZIONNER_P1_JOINTS, float)
    return combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=robot_action_processor,
            initial_features=create_initial_features(action=joint_features),
            use_videos=False,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(
                observation=joint_features,
            ),
            use_videos=False,
        ),
    )


def _build_robot_features(
    robot: ZionnerP1Follower,
    *,
    use_videos: bool,
) -> dict[str, dict[str, Any]]:
    _, robot_action_processor, robot_observation_processor = (
        make_default_processors()
    )
    return combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=robot_action_processor,
            initial_features=create_initial_features(
                action=robot.action_features,
            ),
            use_videos=use_videos,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(
                observation=robot.observation_features,
            ),
            use_videos=use_videos,
        ),
    )


def _create_dataset(
    repo_id: str,
    root: str | Path | None,
    fps: int,
    robot_type: str,
    features: dict[str, dict[str, Any]],
    *,
    use_videos: bool,
    image_writer_processes: int = 0,
    image_writer_threads: int = 0,
    batch_encoding_size: int = 1,
) -> LeRobotDataset:
    return LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        root=root,
        robot_type=robot_type,
        features=features,
        use_videos=use_videos,
        image_writer_processes=image_writer_processes,
        image_writer_threads=image_writer_threads,
        batch_encoding_size=batch_encoding_size,
    )


def _set_power_state(client: XCoreZionnerP1Client, enabled: bool) -> None:
    robot = client._require_robot()
    ec: dict[str, Any] = {}
    robot.setPowerState(enabled, ec)
    if ec.get("ec", 0) != 0:
        raise RuntimeError(f"setPowerState({enabled}) failed: {ec}")


def run_teach_mode(cfg: TwoStageRecordConfig) -> LeRobotDataset:
    fps = cfg.trajectory.fps
    features = _build_joint_only_features()
    dataset = _create_dataset(
        repo_id=cfg.trajectory.repo_id,
        root=cfg.trajectory.root,
        fps=fps,
        robot_type="zionnerP1_follower",
        features=features,
        use_videos=False,
    )

    client = XCoreZionnerP1Client(
        ip_address=cfg.robot.ip_address,
        local_ip_address=cfg.robot.local_ip_address,
        use_realtime=False,
    )
    listener = None
    frame_count = 0

    try:
        client.connect()
        if cfg.teach_power_off:
            _set_power_state(client, False)

        listener, events = init_q_keyboard_listener()
        log_say(
            "Teach mode started. Manually drag the arm and press q to stop.",
            cfg.play_sounds,
            blocking=True,
        )

        start_t = time.perf_counter()
        while True:
            loop_start_t = time.perf_counter()
            if events["stop"]:
                break

            if cfg.trajectory.duration_s is not None:
                if time.perf_counter() - start_t >= cfg.trajectory.duration_s:
                    break

            joint_positions = client.read_joint_positions()
            observation = _joint_observation_from_positions(joint_positions)
            observation_frame = build_dataset_frame(
                dataset.features,
                observation,
                prefix=OBS_STR,
            )
            action_frame = build_dataset_frame(
                dataset.features,
                observation,
                prefix=ACTION,
            )
            dataset.add_frame(
                {
                    **observation_frame,
                    **action_frame,
                    "task": cfg.trajectory.single_task,
                }
            )
            frame_count += 1

            dt_s = time.perf_counter() - loop_start_t
            busy_wait(1 / fps - dt_s)

        if frame_count == 0:
            raise RuntimeError("No joint trajectory frames were recorded.")

        dataset.save_episode()
    finally:
        client.disconnect()
        if listener is not None:
            listener.stop()

    log_say("Teach mode finished", cfg.play_sounds, blocking=True)
    if cfg.trajectory.push_to_hub:
        dataset.push_to_hub(
            tags=cfg.trajectory.tags,
            private=cfg.trajectory.private,
        )
    return dataset


def _load_trajectory_actions(
    cfg: TwoStageRecordConfig,
) -> tuple[LeRobotDataset, Any, Any]:
    trajectory_dataset = LeRobotDataset(
        cfg.trajectory.repo_id,
        root=cfg.trajectory.root,
        episodes=[cfg.trajectory.episode],
    )
    episode_frames = trajectory_dataset.hf_dataset.filter(
        lambda item: item["episode_index"] == cfg.trajectory.episode
    )
    if len(episode_frames) == 0:
        raise ValueError(
            "Episode "
            f"{cfg.trajectory.episode} not found in trajectory dataset "
            f"{cfg.trajectory.repo_id}."
        )
    actions = episode_frames.select_columns(ACTION)
    return trajectory_dataset, episode_frames, actions


def run_replay_record_mode(cfg: TwoStageRecordConfig) -> LeRobotDataset:
    trajectory_dataset, episode_frames, actions = _load_trajectory_actions(cfg)
    robot = ZionnerP1Follower(cfg.robot)
    record_fps = cfg.dataset.fps or trajectory_dataset.fps
    features = _build_robot_features(robot, use_videos=cfg.dataset.video)
    dataset = _create_dataset(
        repo_id=cfg.dataset.repo_id,
        root=cfg.dataset.root,
        fps=record_fps,
        robot_type=robot.name,
        features=features,
        use_videos=cfg.dataset.video,
        image_writer_processes=cfg.dataset.num_image_writer_processes,
        image_writer_threads=(
            cfg.dataset.num_image_writer_threads_per_camera
            * len(robot.cameras)
        ),
        batch_encoding_size=cfg.dataset.video_encoding_batch_size,
    )

    listener = None
    frame_count = 0
    try:
        robot.connect()
        listener, events = init_q_keyboard_listener()
        log_say(
            "Replay-record mode started. "
            "Replaying trajectory and capturing robot observations.",
            cfg.play_sounds,
            blocking=True,
        )

        with VideoEncodingManager(dataset):
            for idx in range(len(episode_frames)):
                loop_start_t = time.perf_counter()
                if events["stop"]:
                    break

                observation = robot.get_observation()
                action_array = actions[idx][ACTION]
                action = {
                    name: float(action_array[action_idx])
                    for action_idx, name in enumerate(
                        trajectory_dataset.features[ACTION]["names"]
                    )
                }

                robot.send_action(action)

                observation_frame = build_dataset_frame(
                    dataset.features,
                    observation,
                    prefix=OBS_STR,
                )
                action_frame = build_dataset_frame(
                    dataset.features,
                    action,
                    prefix=ACTION,
                )
                dataset.add_frame(
                    {
                        **observation_frame,
                        **action_frame,
                        "task": cfg.dataset.single_task,
                    }
                )
                frame_count += 1

                dt_s = time.perf_counter() - loop_start_t
                busy_wait(1 / record_fps - dt_s)

        if frame_count == 0:
            raise RuntimeError("No replay-record frames were captured.")

        dataset.save_episode()
    finally:
        if listener is not None:
            listener.stop()
        if robot.is_connected:
            robot.disconnect()

    log_say("Replay-record mode finished", cfg.play_sounds, blocking=True)
    if cfg.dataset.push_to_hub:
        dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)
    return dataset


@parser.wrap()
def two_stage_record(cfg: TwoStageRecordConfig) -> LeRobotDataset:
    init_logging()
    logging.info(pformat(asdict(cfg)))

    if cfg.mode not in {"teach", "replay_record"}:
        raise ValueError(
            "mode must be one of {'teach', 'replay_record'}, got "
            f"{cfg.mode!r}."
        )

    if cfg.mode == "teach":
        return run_teach_mode(cfg)
    if cfg.mode == "replay_record":
        return run_replay_record_mode(cfg)

    raise ValueError(f"Unsupported mode: {cfg.mode}")


def main() -> None:
    register_third_party_devices()
    two_stage_record()


if __name__ == "__main__":
    main()
