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

"""Provides the Gemini335LCamera class for Orbbec Gemini 335L cameras."""

import logging
import time
from threading import Event, Lock, Thread
from typing import Any

import cv2  # type: ignore
import numpy as np  # type: ignore
from numpy.typing import NDArray  # type: ignore

try:
    import pyorbbecsdk as ob  # type: ignore
except Exception as e:
    ob = None
    logging.info(f"Could not import pyorbbecsdk: {e}")

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..camera import Camera
from ..configs import ColorMode
from ..utils import get_cv2_rotation
from .configuration_gemini335l import Gemini335LCameraConfig

logger = logging.getLogger(__name__)


class Gemini335LCamera(Camera):
    """Camera backend for Orbbec Gemini 335L devices via `pyorbbecsdk`."""

    _COLOR_FORMAT_MAP = {
        "MJPG": "MJPG",
        "RGB": "RGB",
        "BGR": "BGR",
        "UYVY": "UYVY",
        "YUYV": "YUYV",
        "YUY2": "YUY2",
        "NV12": "NV12",
        "NV21": "NV21",
        "I420": "I420",
    }
    _DEPTH_FORMAT_MAP = {
        "Y16": "Y16",
        "Z16": "Z16",
        "RW16": "RW16",
    }
    _ALIGN_MODE_MAP = {
        "disable": "DISABLE",
        "sw": "SW_MODE",
        "hw": "HW_MODE",
    }
    _FRAME_AGGREGATE_MODE_MAP = {
        "disable": "DISABLE",
        "any_situation": "ANY_SITUATION",
        "color_frame_require": "COLOR_FRAME_REQUIRE",
        "full_frame_require": "FULL_FRAME_REQUIRE",
    }

    def __init__(self, config: Gemini335LCameraConfig):
        super().__init__(config)

        self.config = config
        self.color_mode = config.color_mode
        self.color_format = config.color_format
        self.use_depth = config.use_depth
        self.depth_width = config.depth_width
        self.depth_height = config.depth_height
        self.depth_fps = config.depth_fps
        self.depth_format = config.depth_format
        self.align_mode = config.align_mode
        self.enable_frame_sync = config.enable_frame_sync
        self.frame_aggregate_output_mode = config.frame_aggregate_output_mode
        self.warmup_s = config.warmup_s

        self.identifier = config.serial_number_or_name
        self.device_info: dict[str, Any] = self._find_device_info(self.identifier)
        self.serial_number = str(self.device_info["id"])
        self.device_name = str(self.device_info["name"])

        self.pipeline: Any = None
        self.pipeline_config: Any = None
        self.color_profile: Any = None
        self.depth_profile: Any = None

        self.thread: Thread | None = None
        self.stop_event: Event | None = None
        self.frame_lock: Lock = Lock()
        self.latest_frame: NDArray[Any] | None = None
        self.latest_depth_frame: NDArray[Any] | None = None
        self.latest_frame_bundle: dict[str, Any] | None = None
        self.new_frame_event: Event = Event()

        self.rotation: int | None = get_cv2_rotation(config.rotation)
        self.depth_scale: float | None = None

        self.capture_width = self.width
        self.capture_height = self.height
        if self.capture_width and self.capture_height and self.rotation in [
            cv2.ROTATE_90_CLOCKWISE,
            cv2.ROTATE_90_COUNTERCLOCKWISE,
        ]:
            self.capture_width, self.capture_height = self.capture_height, self.capture_width

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({self.serial_number})"

    @property
    def is_connected(self) -> bool:
        return self.pipeline is not None and self.pipeline_config is not None

    @staticmethod
    def _require_sdk() -> Any:
        if ob is None:
            raise ImportError(
                "pyorbbecsdk is required for `orbbec_gemini335l`. Activate the `lerobot` conda environment first."
            )
        return ob

    @classmethod
    def find_cameras(cls) -> list[dict[str, Any]]:
        sdk = cls._require_sdk()
        context = sdk.Context()
        devices = context.query_devices()
        found_cameras_info: list[dict[str, Any]] = []

        for index in range(devices.get_count()):
            device = devices.get_device_by_index(index)
            device_info = device.get_device_info()
            camera_info: dict[str, Any] = {
                "name": device_info.get_name(),
                "type": "Orbbec Gemini 335L",
                "backend_type": "orbbec_gemini335l",
                "id": device_info.get_serial_number(),
                "firmware_version": device_info.get_firmware_version(),
                "usb_type_descriptor": getattr(device_info, "get_connection_type", lambda: None)(),
                "uid": getattr(device_info, "get_uid", lambda: None)(),
                "product_id": hex(device_info.get_pid()),
                "vendor_id": hex(device_info.get_vid()),
            }

            pipeline = sdk.Pipeline(device)
            for sensor_type, key in (
                (sdk.OBSensorType.COLOR_SENSOR, "default_color_stream_profile"),
                (sdk.OBSensorType.DEPTH_SENSOR, "default_depth_stream_profile"),
            ):
                try:
                    profile = pipeline.get_stream_profile_list(sensor_type).get_default_video_stream_profile()
                    camera_info[key] = {
                        "width": profile.get_width(),
                        "height": profile.get_height(),
                        "fps": profile.get_fps(),
                        "format": profile.get_format().name,
                    }
                except Exception:
                    continue

            found_cameras_info.append(camera_info)

        return found_cameras_info

    def _find_device_info(self, identifier: str) -> dict[str, Any]:
        camera_infos = self.find_cameras()
        if not camera_infos:
            raise ValueError("No Orbbec Gemini 335L camera found. Please connect a device and retry.")

        matches = [cam for cam in camera_infos if str(cam["id"]) == identifier]
        if matches:
            return matches[0]

        matches = [cam for cam in camera_infos if str(cam["name"]) == identifier]
        if not matches:
            available_ids = [cam["id"] for cam in camera_infos]
            available_names = [cam["name"] for cam in camera_infos]
            raise ValueError(
                f"No Orbbec Gemini 335L camera found with identifier '{identifier}'. Available serial numbers: {available_ids}; available names: {available_names}."
            )

        if len(matches) > 1:
            serial_numbers = [cam["id"] for cam in matches]
            raise ValueError(
                f"Multiple Orbbec Gemini 335L cameras found with name '{identifier}'. Please use a serial number instead. Found SNs: {serial_numbers}"
            )

        return matches[0]

    def _get_sdk_enum(self, enum_class_name: str, attr_name: str) -> Any:
        sdk = self._require_sdk()
        return getattr(getattr(sdk, enum_class_name), attr_name)

    def _get_color_format_enum(self) -> Any:
        if self.color_format is None:
            return None
        return self._get_sdk_enum("OBFormat", self._COLOR_FORMAT_MAP[self.color_format.upper()])

    def _get_depth_format_enum(self) -> Any:
        if self.depth_format is None:
            return None
        return self._get_sdk_enum("OBFormat", self._DEPTH_FORMAT_MAP[self.depth_format.upper()])

    def _select_video_profile(
        self,
        pipeline: Any,
        sensor_type: Any,
        width: int | None,
        height: int | None,
        fps: int | None,
        fmt: Any,
    ) -> Any:
        profile_list = pipeline.get_stream_profile_list(sensor_type)
        if width is None or height is None or fps is None:
            return profile_list.get_default_video_stream_profile()

        if fmt is not None:
            return profile_list.get_video_stream_profile(width, height, fmt, fps)

        last_error: Exception | None = None
        for index in range(profile_list.get_count()):
            profile = profile_list.get_stream_profile_by_index(index).as_video_stream_profile()
            if profile.get_width() == width and profile.get_height() == height and profile.get_fps() == fps:
                return profile
        try:
            return profile_list.get_video_stream_profile(width, height, profile_list.get_default_video_stream_profile().get_format(), fps)
        except Exception as e:
            last_error = e
        raise ConnectionError(
            f"No stream profile found for sensor={sensor_type}, width={width}, height={height}, fps={fps}, format={fmt}."
        ) from last_error

    def _create_pipeline(self) -> Any:
        sdk = self._require_sdk()
        context = sdk.Context()
        devices = context.query_devices()
        for index in range(devices.get_count()):
            device = devices.get_device_by_index(index)
            device_info = device.get_device_info()
            if str(device_info.get_serial_number()) == self.serial_number:
                return sdk.Pipeline(device)
        raise ConnectionError(
            f"Failed to locate Orbbec Gemini 335L device with serial number '{self.serial_number}'."
        )

    def _configure_pipeline(self) -> tuple[Any, Any, Any | None]:
        sdk = self._require_sdk()
        pipeline = self._create_pipeline()
        config = sdk.Config()

        color_profile = self._select_video_profile(
            pipeline,
            sdk.OBSensorType.COLOR_SENSOR,
            self.width,
            self.height,
            self.fps,
            self._get_color_format_enum(),
        )
        config.enable_stream(color_profile)

        depth_profile = None
        if self.use_depth:
            depth_profile = self._select_video_profile(
                pipeline,
                sdk.OBSensorType.DEPTH_SENSOR,
                self.depth_width,
                self.depth_height,
                self.depth_fps,
                self._get_depth_format_enum(),
            )
            config.enable_stream(depth_profile)

            align_mode = self._ALIGN_MODE_MAP[self.align_mode]
            config.set_align_mode(getattr(sdk.OBAlignMode, align_mode))
            aggregate_mode = self._FRAME_AGGREGATE_MODE_MAP[self.frame_aggregate_output_mode]
            config.set_frame_aggregate_output_mode(
                getattr(sdk.OBFrameAggregateOutputMode, aggregate_mode)
            )

        return pipeline, config, depth_profile

    def connect(self, warmup: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        try:
            self.pipeline, self.pipeline_config, self.depth_profile = self._configure_pipeline()
            self.color_profile = self.pipeline_config.get_enabled_stream_profile_list().get_stream_profile_by_index(0)
            self.pipeline.start(self.pipeline_config)
            if self.enable_frame_sync and self.use_depth:
                self.pipeline.enable_frame_sync()
            self._configure_capture_settings()
        except Exception as e:
            self.pipeline = None
            self.pipeline_config = None
            self.color_profile = None
            self.depth_profile = None
            raise ConnectionError(
                f"Failed to open {self}. Run `lerobot-find-cameras orbbec_gemini335l` to inspect available devices and profiles."
            ) from e

        if warmup:
            start_time = time.time()
            while time.time() - start_time < self.warmup_s:
                self.read_frame_bundle(timeout_ms=1500)
                time.sleep(0.05)

        logger.info(f"{self} connected.")

    def _configure_capture_settings(self) -> None:
        if not self.is_connected or self.color_profile is None:
            raise DeviceNotConnectedError(f"Cannot validate settings for {self} as it is not connected.")

        color_profile = self.color_profile.as_video_stream_profile()
        actual_width = int(round(color_profile.get_width()))
        actual_height = int(round(color_profile.get_height()))
        actual_fps = int(round(color_profile.get_fps()))

        if self.fps is None:
            self.fps = actual_fps

        if self.rotation in [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE]:
            self.capture_width, self.capture_height = actual_width, actual_height
            self.width, self.height = actual_height, actual_width
        else:
            self.capture_width, self.capture_height = actual_width, actual_height
            self.width, self.height = actual_width, actual_height

        if self.use_depth and self.depth_profile is not None:
            depth_profile = self.depth_profile.as_video_stream_profile()
            self.depth_width = int(round(depth_profile.get_width()))
            self.depth_height = int(round(depth_profile.get_height()))
            self.depth_fps = int(round(depth_profile.get_fps()))

    def _frame_to_bgr_image(self, frame: Any) -> NDArray[Any]:
        sdk = self._require_sdk()
        width = frame.get_width()
        height = frame.get_height()
        color_format = frame.get_format()
        data = np.asanyarray(frame.get_data())

        if color_format == sdk.OBFormat.RGB:
            image = np.resize(data, (height, width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if color_format == sdk.OBFormat.BGR:
            return np.resize(data, (height, width, 3))
        if color_format in (sdk.OBFormat.YUYV, sdk.OBFormat.YUY2):
            image = np.resize(data, (height, width, 2))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
        if color_format == sdk.OBFormat.UYVY:
            image = np.resize(data, (height, width, 2))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)
        if color_format == sdk.OBFormat.MJPG:
            image = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError(f"{self} failed to decode MJPG color frame.")
            return image
        if color_format == sdk.OBFormat.NV12:
            image = np.frombuffer(data, dtype=np.uint8).reshape((height * 3 // 2, width))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV12)
        if color_format == sdk.OBFormat.NV21:
            image = np.frombuffer(data, dtype=np.uint8).reshape((height * 3 // 2, width))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV21)
        if color_format == sdk.OBFormat.I420:
            image = np.frombuffer(data, dtype=np.uint8).reshape((height * 3 // 2, width))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)

        raise RuntimeError(f"Unsupported color format for {self}: {color_format}.")

    def _extract_frame_metadata(self, frame: Any) -> dict[str, Any]:
        sdk = self._require_sdk()
        metadata: dict[str, Any] = {
            "timestamp": frame.get_timestamp(),
            "timestamp_us": frame.get_timestamp_us(),
            "global_timestamp_us": frame.get_global_timestamp_us(),
            "system_timestamp": frame.get_system_timestamp(),
            "system_timestamp_us": frame.get_system_timestamp_us(),
            "index": frame.get_index(),
            "format": frame.get_format().name,
            "width": frame.get_width(),
            "height": frame.get_height(),
        }
        for attr in dir(sdk.OBFrameMetadataType):
            if attr.startswith("__"):
                continue
            metadata_type = getattr(sdk.OBFrameMetadataType, attr)
            if hasattr(metadata_type, "name") and frame.has_metadata(metadata_type):
                metadata[attr] = frame.get_metadata_value(metadata_type)
        return metadata

    def _postprocess_color_image(
        self, image: NDArray[Any], color_mode: ColorMode | None = None
    ) -> NDArray[Any]:
        if color_mode and color_mode not in (ColorMode.RGB, ColorMode.BGR):
            raise ValueError(
                f"Invalid requested color mode '{color_mode}'. Expected {ColorMode.RGB} or {ColorMode.BGR}."
            )

        h, w, c = image.shape
        if c != 3:
            raise RuntimeError(f"{self} frame channels={c} do not match expected 3 channels.")
        if h != self.capture_height or w != self.capture_width:
            raise RuntimeError(
                f"{self} frame width={w} or height={h} do not match configured width={self.capture_width} or height={self.capture_height}."
            )

        processed_image = image
        requested_color_mode = color_mode or self.color_mode
        if requested_color_mode == ColorMode.RGB:
            processed_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2RGB)

        if self.rotation in [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE, cv2.ROTATE_180]:
            processed_image = cv2.rotate(processed_image, self.rotation)

        return processed_image

    def _postprocess_depth_image(self, image: NDArray[Any]) -> NDArray[Any]:
        h, w = image.shape
        if self.depth_height is not None and self.depth_width is not None:
            if h != self.depth_height or w != self.depth_width:
                raise RuntimeError(
                    f"{self} depth frame width={w} or height={h} do not match configured depth width={self.depth_width} or depth height={self.depth_height}."
                )

        if self.rotation in [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE, cv2.ROTATE_180]:
            image = cv2.rotate(image, self.rotation)
        return image

    def _wait_for_frames(self, timeout_ms: int) -> Any:
        if self.pipeline is None:
            raise RuntimeError(f"{self}: pipeline must be initialized before use.")
        return self.pipeline.wait_for_frames(timeout_ms)

    def read_frame_bundle(self, timeout_ms: int = 200) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start_time = time.perf_counter()
        frames = self._wait_for_frames(timeout_ms)
        if frames is None:
            raise RuntimeError(f"{self} read failed: no frame set returned.")

        color_frame = frames.get_color_frame()
        if color_frame is None:
            raise RuntimeError(f"{self} read failed: missing color frame.")

        color_image_bgr = self._frame_to_bgr_image(color_frame)
        color_image = self._postprocess_color_image(color_image_bgr)

        depth_map = None
        depth_metadata = None
        if self.use_depth:
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                raise RuntimeError(f"{self} read failed: missing depth frame while depth is enabled.")
            depth_map = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(
                depth_frame.get_height(), depth_frame.get_width()
            )
            depth_map = self._postprocess_depth_image(depth_map)
            self.depth_scale = float(depth_frame.get_depth_scale())
            depth_metadata = self._extract_frame_metadata(depth_frame)

        bundle = {
            "color": color_image,
            "depth": depth_map,
            "color_metadata": self._extract_frame_metadata(color_frame),
            "depth_metadata": depth_metadata,
            "read_duration_ms": (time.perf_counter() - start_time) * 1e3,
        }
        return bundle

    def read(self, color_mode: ColorMode | None = None, timeout_ms: int = 200) -> NDArray[Any]:
        bundle = self.read_frame_bundle(timeout_ms=timeout_ms)
        image = bundle["color"]
        if color_mode is not None and color_mode != self.color_mode:
            if color_mode == ColorMode.RGB and self.color_mode == ColorMode.BGR:
                return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            if color_mode == ColorMode.BGR and self.color_mode == ColorMode.RGB:
                return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    def read_depth(self, timeout_ms: int = 200) -> NDArray[Any]:
        if not self.use_depth:
            raise RuntimeError(
                f"Failed to capture depth frame '.read_depth()'. Depth stream is not enabled for {self}."
            )
        bundle = self.read_frame_bundle(timeout_ms=timeout_ms)
        depth_map = bundle["depth"]
        if depth_map is None:
            raise RuntimeError(f"{self} read_depth failed: missing depth frame.")
        return depth_map

    def get_device_info(self) -> dict[str, Any]:
        return dict(self.device_info)

    def get_intrinsics(self) -> dict[str, Any]:
        if self.color_profile is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        color_intrinsic = self.color_profile.as_video_stream_profile().get_intrinsic()
        data = {"color": color_intrinsic}
        if self.use_depth and self.depth_profile is not None:
            data["depth"] = self.depth_profile.as_video_stream_profile().get_intrinsic()
            data["depth_distortion"] = self.depth_profile.as_video_stream_profile().get_distortion()
            data["color_distortion"] = self.color_profile.as_video_stream_profile().get_distortion()
        return data

    def get_extrinsics(self) -> Any:
        if not self.use_depth or self.depth_profile is None or self.color_profile is None:
            raise RuntimeError(f"{self} depth/color profiles are required to compute extrinsics.")
        return self.depth_profile.get_extrinsic_to(self.color_profile)

    def _read_loop(self) -> None:
        if self.stop_event is None:
            raise RuntimeError(f"{self}: stop_event is not initialized before starting read loop.")

        while not self.stop_event.is_set():
            try:
                bundle = self.read_frame_bundle(timeout_ms=500)
                with self.frame_lock:
                    self.latest_frame = bundle["color"]
                    self.latest_depth_frame = bundle["depth"]
                    self.latest_frame_bundle = bundle
                self.new_frame_event.set()
            except DeviceNotConnectedError:
                break
            except Exception as e:
                logger.warning(f"Error reading frame in background thread for {self}: {e}")

    def _start_read_thread(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=0.1)
        if self.stop_event is not None:
            self.stop_event.set()

        self.stop_event = Event()
        self.thread = Thread(target=self._read_loop, args=(), name=f"{self}_read_loop")
        self.thread.daemon = True
        self.thread.start()

    def _stop_read_thread(self) -> None:
        if self.stop_event is not None:
            self.stop_event.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self.thread = None
        self.stop_event = None

    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.thread is None or not self.thread.is_alive():
            self._start_read_thread()

        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            thread_alive = self.thread is not None and self.thread.is_alive()
            raise TimeoutError(
                f"Timed out waiting for frame from camera {self} after {timeout_ms} ms. Read thread alive: {thread_alive}."
            )

        with self.frame_lock:
            frame = self.latest_frame
            self.new_frame_event.clear()

        if frame is None:
            raise RuntimeError(f"Internal error: event set but no frame available for {self}.")
        return frame

    def disconnect(self) -> None:
        if not self.is_connected and self.thread is None:
            raise DeviceNotConnectedError(
                f"Attempted to disconnect {self}, but it appears already disconnected."
            )

        if self.thread is not None:
            self._stop_read_thread()

        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
            self.pipeline_config = None
            self.color_profile = None
            self.depth_profile = None

        logger.info(f"{self} disconnected.")
