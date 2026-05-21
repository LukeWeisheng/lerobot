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

from dataclasses import dataclass

from ..configs import CameraConfig, ColorMode, Cv2Rotation


@CameraConfig.register_subclass("orbbec_gemini335l")
@dataclass
class Gemini335LCameraConfig(CameraConfig):
    """Configuration class for Orbbec Gemini 335L cameras.

    Attributes:
        serial_number_or_name: Camera serial number or unique device name.
        color_mode: Output color mode for `read`.
        color_format: Requested SDK color stream format. Typical values are
            `MJPG`, `RGB`, or `BGR`.
        use_depth: Whether to enable the depth stream.
        depth_width: Requested depth width in pixels.
        depth_height: Requested depth height in pixels.
        depth_fps: Requested depth fps.
        depth_format: Requested SDK depth stream format. Defaults to `Y16`.
        align_mode: Depth-to-color alignment mode: `disable`, `sw`, or `hw`.
        enable_frame_sync: Whether to enable SDK frame synchronization.
        frame_aggregate_output_mode: Aggregation mode: `disable`, `any_situation`,
            `color_frame_require`, or `full_frame_require`.
        rotation: OpenCV rotation applied after color/depth conversion.
        warmup_s: Time to read frames before returning from `connect`.
    """

    serial_number_or_name: str
    color_mode: ColorMode = ColorMode.RGB
    color_format: str | None = "MJPG"
    use_depth: bool = False
    depth_width: int | None = 848
    depth_height: int | None = 480
    depth_fps: int | None = 30
    depth_format: str | None = "Y16"
    align_mode: str = "disable"
    enable_frame_sync: bool = False
    frame_aggregate_output_mode: str = "full_frame_require"
    rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION
    warmup_s: int = 1

    def __post_init__(self) -> None:
        if self.color_mode not in (ColorMode.RGB, ColorMode.BGR):
            raise ValueError(
                f"`color_mode` is expected to be {ColorMode.RGB.value} or {ColorMode.BGR.value}, but {self.color_mode} is provided."
            )

        if self.rotation not in (
            Cv2Rotation.NO_ROTATION,
            Cv2Rotation.ROTATE_90,
            Cv2Rotation.ROTATE_180,
            Cv2Rotation.ROTATE_270,
        ):
            raise ValueError(
                f"`rotation` is expected to be in {(Cv2Rotation.NO_ROTATION, Cv2Rotation.ROTATE_90, Cv2Rotation.ROTATE_180, Cv2Rotation.ROTATE_270)}, but {self.rotation} is provided."
            )

        color_values = (self.fps, self.width, self.height)
        if any(v is not None for v in color_values) and any(v is None for v in color_values):
            raise ValueError(
                "For color stream `fps`, `width` and `height`, either all of them need to be set, or none of them."
            )

        depth_values = (self.depth_fps, self.depth_width, self.depth_height)
        if any(v is not None for v in depth_values) and any(v is None for v in depth_values):
            raise ValueError(
                "For depth stream `depth_fps`, `depth_width` and `depth_height`, either all of them need to be set, or none of them."
            )

        if self.align_mode not in {"disable", "sw", "hw"}:
            raise ValueError("`align_mode` must be one of: 'disable', 'sw', 'hw'.")

        if self.frame_aggregate_output_mode not in {
            "disable",
            "any_situation",
            "color_frame_require",
            "full_frame_require",
        }:
            raise ValueError(
                "`frame_aggregate_output_mode` must be one of: 'disable', 'any_situation', 'color_frame_require', 'full_frame_require'."
            )
