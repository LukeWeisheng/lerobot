#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

from __future__ import annotations

from pathlib import Path
import sys
from typing import Any
from unittest.mock import patch

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"

for search_path in (SRC_ROOT, REPO_ROOT):
    search_path_str = str(search_path)
    if search_path_str not in sys.path:
        sys.path.insert(0, search_path_str)

import cv2
import numpy as np
import pytest

from lerobot.cameras.configs import Cv2Rotation  # noqa: E402
from lerobot.utils.errors import (  # noqa: E402
    DeviceAlreadyConnectedError,
    DeviceNotConnectedError,
)

pytest.importorskip("pyorbbecsdk")

from lerobot.cameras.gemini335l import (  # noqa: E402
    Gemini335LCamera,
    Gemini335LCameraConfig,
)


def _gemini_available() -> bool:
    try:
        return len(Gemini335LCamera.find_cameras()) > 0
    except Exception:
        return False


HARDWARE_AVAILABLE = _gemini_available()


class FakeScalarEnum:
    def __init__(self, name: str):
        self.name = name

    def __repr__(self) -> str:
        return self.name

    def __eq__(self, other: object) -> bool:
        return isinstance(other, FakeScalarEnum) and self.name == other.name


class FakeVideoProfile:
    def __init__(self, width: int, height: int, fps: int, fmt: FakeScalarEnum):
        self._width = width
        self._height = height
        self._fps = fps
        self._format = fmt

    def get_width(self):
        return self._width

    def get_height(self):
        return self._height

    def get_fps(self):
        return self._fps

    def get_format(self):
        return self._format

    def as_video_stream_profile(self):
        return self

    def get_intrinsic(self):
        return {"fx": 1.0, "fy": 1.0}

    def get_distortion(self):
        return {"k1": 0.0}

    def get_extrinsic_to(self, _other):
        return {"rotation": [1.0] * 9, "translation": [0.0, 0.0, 0.0]}


class FakeProfileList:
    def __init__(self, profiles: list[FakeVideoProfile]):
        self.profiles = profiles

    def get_count(self):
        return len(self.profiles)

    def get_default_video_stream_profile(self):
        return self.profiles[0]

    def get_stream_profile_by_index(self, index: int):
        return self.profiles[index]

    def get_video_stream_profile(self, width: int, height: int, fmt: FakeScalarEnum, fps: int):
        for profile in self.profiles:
            if (
                profile.get_width() == width
                and profile.get_height() == height
                and profile.get_fps() == fps
                and profile.get_format() == fmt
            ):
                return profile
        raise RuntimeError("Profile not found")


class FakeFrame:
    def __init__(
        self,
        data: np.ndarray,
        fmt: FakeScalarEnum,
        width: int,
        height: int,
        metadata: dict[str, Any] | None = None,
        depth_scale: float = 1.0,
    ):
        self._data = data
        self._format = fmt
        self._width = width
        self._height = height
        self._metadata = metadata or {}
        self._depth_scale = depth_scale

    def get_data(self):
        return self._data

    def get_format(self):
        return self._format

    def get_width(self):
        return self._width

    def get_height(self):
        return self._height

    def get_timestamp(self):
        return 1

    def get_timestamp_us(self):
        return 1000

    def get_global_timestamp_us(self):
        return 1001

    def get_system_timestamp(self):
        return 1002

    def get_system_timestamp_us(self):
        return 1003

    def get_index(self):
        return 7

    def has_metadata(self, metadata_type):
        return metadata_type.name in self._metadata

    def get_metadata_value(self, metadata_type):
        return self._metadata[metadata_type.name]

    def get_depth_scale(self):
        return self._depth_scale


class FakeFrameSet:
    def __init__(self, color_frame: FakeFrame, depth_frame: FakeFrame | None = None):
        self._color_frame = color_frame
        self._depth_frame = depth_frame

    def get_color_frame(self):
        return self._color_frame

    def get_depth_frame(self):
        return self._depth_frame


class FakeEnabledProfileList:
    def __init__(self, profile: FakeVideoProfile):
        self.profile = profile

    def get_stream_profile_by_index(self, _index: int):
        return self.profile


class FakeConfig:
    def __init__(self):
        self.enabled_profiles: list[Any] = []
        self.align_mode = None
        self.frame_aggregate_output_mode = None

    def enable_stream(self, profile):
        self.enabled_profiles.append(profile)

    def get_enabled_stream_profile_list(self):
        return FakeEnabledProfileList(self.enabled_profiles[0])

    def set_align_mode(self, align_mode):
        self.align_mode = align_mode

    def set_frame_aggregate_output_mode(self, mode):
        self.frame_aggregate_output_mode = mode


class FakePipeline:
    def __init__(self, color_profile: FakeVideoProfile, depth_profile: FakeVideoProfile, frameset: FakeFrameSet):
        self.color_profile = color_profile
        self.depth_profile = depth_profile
        self.frameset = frameset
        self.started = False
        self.frame_sync_enabled = False

    def get_stream_profile_list(self, sensor_type):
        if sensor_type.name == "COLOR_SENSOR":
            return FakeProfileList([self.color_profile])
        return FakeProfileList([self.depth_profile])

    def start(self, _config):
        self.started = True

    def stop(self):
        self.started = False

    def wait_for_frames(self, _timeout_ms: int):
        return self.frameset

    def enable_frame_sync(self):
        self.frame_sync_enabled = True


class FakeMetadataType:
    FRAME_NUMBER = FakeScalarEnum("FRAME_NUMBER")
    EXPOSURE = FakeScalarEnum("EXPOSURE")


class FakeOBModule:
    Config = FakeConfig

    class OBFormat:
        MJPG = FakeScalarEnum("MJPG")
        RGB = FakeScalarEnum("RGB")
        BGR = FakeScalarEnum("BGR")
        Y16 = FakeScalarEnum("Y16")
        YUYV = FakeScalarEnum("YUYV")
        YUY2 = FakeScalarEnum("YUY2")
        UYVY = FakeScalarEnum("UYVY")
        NV12 = FakeScalarEnum("NV12")
        NV21 = FakeScalarEnum("NV21")
        I420 = FakeScalarEnum("I420")

    class OBSensorType:
        COLOR_SENSOR = FakeScalarEnum("COLOR_SENSOR")
        DEPTH_SENSOR = FakeScalarEnum("DEPTH_SENSOR")

    class OBAlignMode:
        DISABLE = FakeScalarEnum("DISABLE")
        SW_MODE = FakeScalarEnum("SW_MODE")
        HW_MODE = FakeScalarEnum("HW_MODE")

    class OBFrameAggregateOutputMode:
        DISABLE = FakeScalarEnum("DISABLE")
        ANY_SITUATION = FakeScalarEnum("ANY_SITUATION")
        COLOR_FRAME_REQUIRE = FakeScalarEnum("COLOR_FRAME_REQUIRE")
        FULL_FRAME_REQUIRE = FakeScalarEnum("FULL_FRAME_REQUIRE")

    OBFrameMetadataType = FakeMetadataType


def make_mock_camera(use_depth: bool = True, rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION):
    config = Gemini335LCameraConfig(
        serial_number_or_name="CP2N1630006X",
        width=1280,
        height=720,
        fps=30,
        use_depth=use_depth,
        rotation=rotation,
    )
    color_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    success, mjpg = cv2.imencode(".jpg", color_img)
    assert success
    color_frame = FakeFrame(
        mjpg.flatten(),
        FakeOBModule.OBFormat.MJPG,
        1280,
        720,
        metadata={"FRAME_NUMBER": 42, "EXPOSURE": 100},
    )
    depth_frame = FakeFrame(
        np.zeros((480, 848), dtype=np.uint16),
        FakeOBModule.OBFormat.Y16,
        848,
        480,
    )
    color_profile = FakeVideoProfile(1280, 720, 30, FakeOBModule.OBFormat.MJPG)
    depth_profile = FakeVideoProfile(848, 480, 30, FakeOBModule.OBFormat.Y16)
    frameset = FakeFrameSet(color_frame, depth_frame if use_depth else None)
    pipeline = FakePipeline(color_profile, depth_profile, frameset)
    patches = [
        patch(
            "lerobot.cameras.gemini335l.camera_gemini335l.ob",
            FakeOBModule,
        ),
        patch.object(
            Gemini335LCamera,
            "find_cameras",
            return_value=[
                {"id": "CP2N1630006X", "name": "Orbbec Gemini 335L"}
            ],
        ),
        patch.object(
            Gemini335LCamera,
            "_create_pipeline",
            return_value=pipeline,
        ),
    ]
    with patches[1]:
        camera = Gemini335LCamera(config)
    return camera, pipeline, patches


def test_config_validation():
    with pytest.raises(ValueError):
        Gemini335LCameraConfig(serial_number_or_name="042", width=1280, height=None, fps=30)


def test_identifier_lookup_with_duplicate_name():
    with patch.object(
        Gemini335LCamera,
        "find_cameras",
        return_value=[
            {"id": "1", "name": "Orbbec Gemini 335L"},
            {"id": "2", "name": "Orbbec Gemini 335L"},
        ],
    ):
        with pytest.raises(ValueError):
            Gemini335LCamera(Gemini335LCameraConfig(serial_number_or_name="Orbbec Gemini 335L"))


def test_connect_and_read_mock():
    camera, pipeline, patches = make_mock_camera()
    with patches[0], patches[1], patches[2]:
        camera.connect(warmup=False)
        image = camera.read()
        depth = camera.read_depth()
        info = camera.get_device_info()
        intrinsics = camera.get_intrinsics()
        extrinsics = camera.get_extrinsics()
        bundle = camera.read_frame_bundle()

        assert camera.is_connected
        assert pipeline.started
        assert isinstance(image, np.ndarray)
        assert image.shape == (720, 1280, 3)
        assert isinstance(depth, np.ndarray)
        assert depth.shape == (480, 848)
        assert info["id"] == "CP2N1630006X"
        assert "color" in intrinsics
        assert extrinsics["translation"] == [0.0, 0.0, 0.0]
        assert bundle["color_metadata"]["FRAME_NUMBER"] == 42


def test_connect_already_connected_mock():
    camera, _pipeline, patches = make_mock_camera()
    with patches[0], patches[1], patches[2]:
        camera.connect(warmup=False)
        with pytest.raises(DeviceAlreadyConnectedError):
            camera.connect(warmup=False)


def test_async_read_mock():
    camera, _pipeline, patches = make_mock_camera()
    with patches[0], patches[1], patches[2]:
        camera.connect(warmup=False)
        try:
            image = camera.async_read(timeout_ms=500)
            assert isinstance(image, np.ndarray)
            assert camera.thread is not None
            assert camera.thread.is_alive()
        finally:
            if camera.is_connected:
                camera.disconnect()


def test_disconnect_before_connect_mock():
    with patch.object(
        Gemini335LCamera,
        "find_cameras",
        return_value=[{"id": "CP2N1630006X", "name": "Orbbec Gemini 335L"}],
    ):
        camera = Gemini335LCamera(Gemini335LCameraConfig(serial_number_or_name="CP2N1630006X"))
        with pytest.raises(DeviceNotConnectedError):
            camera.disconnect()


@pytest.mark.parametrize(
    "rotation, expected_shape",
    [
        (Cv2Rotation.NO_ROTATION, (720, 1280, 3)),
        (Cv2Rotation.ROTATE_90, (1280, 720, 3)),
        (Cv2Rotation.ROTATE_180, (720, 1280, 3)),
        (Cv2Rotation.ROTATE_270, (1280, 720, 3)),
    ],
)
def test_rotation_mock(rotation, expected_shape):
    camera, _pipeline, patches = make_mock_camera(rotation=rotation)
    with patches[0], patches[1], patches[2]:
        camera.connect(warmup=False)
        image = camera.read()
        assert image.shape == expected_shape


@pytest.mark.skipif(not HARDWARE_AVAILABLE, reason="Orbbec Gemini 335L hardware is not available.")
def test_find_cameras_hardware():
    cameras = Gemini335LCamera.find_cameras()
    assert cameras
    assert any(camera["backend_type"] == "orbbec_gemini335l" for camera in cameras)


@pytest.mark.skipif(not HARDWARE_AVAILABLE, reason="Orbbec Gemini 335L hardware is not available.")
def test_connect_and_read_hardware():
    identifier = Gemini335LCamera.find_cameras()[0]["id"]
    config = Gemini335LCameraConfig(
        serial_number_or_name=identifier,
        use_depth=True,
        align_mode="sw",
        enable_frame_sync=True,
        color_format="MJPG",
    )
    camera = Gemini335LCamera(config)
    try:
        camera.connect(warmup=False)
        image = camera.read(timeout_ms=2000)
        depth = camera.read_depth(timeout_ms=2000)
        bundle = camera.read_frame_bundle(timeout_ms=2000)
        intrinsics = camera.get_intrinsics()
        extrinsics = camera.get_extrinsics()
        async_image = camera.async_read(timeout_ms=2000)

        assert image.ndim == 3
        assert image.shape[2] == 3
        assert depth.ndim == 2
        assert bundle["color_metadata"]
        assert "color" in intrinsics
        assert extrinsics is not None
        assert async_image.ndim == 3
    finally:
        if camera.is_connected:
            camera.disconnect()


def main() -> int:
    return pytest.main([
        "--import-mode=importlib",
        str(Path(__file__)),
        "-q",
        "-s",
    ])


if __name__ == "__main__":
    raise SystemExit(main())
