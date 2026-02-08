import datetime
import json
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import PySpin
import yaml

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "light_control"))
import light_control as lc  # noqa: E402

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from camera_control.flir_control import (  # noqa: E402
    chunk_to_dict,
    enable_chunk_data,
    set_raw_pixel_format,
)


class Ansi:
    BLUE = "\033[94m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    CYAN = "\033[96m"
    RED = "\033[91m"
    RESET = "\033[0m"


def load_config(path: str) -> dict:
    with open(path, "r") as handle:
        return yaml.safe_load(handle)


def _set_enum(nodemap, name: str, entry_name: str) -> bool:
    node = PySpin.CEnumerationPtr(nodemap.GetNode(name))
    if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
        return False
    entry = node.GetEntryByName(entry_name)
    if not PySpin.IsAvailable(entry) or not PySpin.IsReadable(entry):
        return False
    try:
        node.SetIntValue(entry.GetValue())
        return True
    except PySpin.SpinnakerException:
        return False


def _set_bool(nodemap, name: str, value: bool) -> bool:
    node = PySpin.CBooleanPtr(nodemap.GetNode(name))
    if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
        return False
    try:
        node.SetValue(bool(value))
        return True
    except PySpin.SpinnakerException:
        return False


def _set_float(nodemap, name: str, value: float) -> Optional[float]:
    node = PySpin.CFloatPtr(nodemap.GetNode(name))
    if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
        return None
    try:
        minimum = node.GetMin()
        maximum = node.GetMax()
        clamped = float(max(minimum, min(maximum, value)))
        node.SetValue(clamped)
        return clamped
    except PySpin.SpinnakerException:
        return None


def _read_exposure_us(config: dict, key_prefix: str, fallback: Optional[float] = None) -> float:
    key_us = f"{key_prefix}_US"
    key_ms = f"{key_prefix}_MS"
    if key_us in config:
        return float(config[key_us])
    if key_ms in config:
        return float(config[key_ms]) * 1000.0
    if fallback is not None:
        return fallback
    raise KeyError(f"{key_us} (or {key_ms}) missing from configuration.")


def build_exposure_schedule(
    center_us: float,
    bracket_steps: int,
) -> List[float]:
    if bracket_steps < 0:
        raise ValueError("Bracket steps must be non-negative.")
    if center_us <= 0:
        raise ValueError("Center exposure must be positive.")
    
    exposures: List[float] = []
    # Generate powers of 2 from -bracket_steps to +bracket_steps
    # e.g. steps=2 -> -2, -1, 0, 1, 2 -> 0.25x, 0.5x, 1x, 2x, 4x
    for i in range(-bracket_steps, bracket_steps + 1):
        factor = 2.0 ** i
        val = center_us * factor
        exposures.append(round(val, 2))
    
    return exposures


def build_light_schedule(step_percent: float) -> List[float]:
    if step_percent <= 0:
        return [0.0, 1.0]
    step = min(max(step_percent, 0.1), 100.0) / 100.0
    values = [0.0]
    current = step
    while current < 1.0 - 1e-6:
        values.append(round(current, 4))
        current += step
    if values[-1] < 1.0:
        values.append(1.0)
    return values


def connect_sync_cameras(config: dict) -> Tuple[PySpin.System, PySpin.CameraList, PySpin.CameraPtr, PySpin.CameraPtr]:
    left_serial = config["CAMERA_SERIAL"]
    right_serial = config["AUTO_CAMERA_SERIAL"]
    if left_serial == right_serial:
        raise ValueError("CAMERA_SERIAL and AUTO_CAMERA_SERIAL must refer to different devices.")

    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    try:
        left = cam_list.GetBySerial(left_serial)
        right = cam_list.GetBySerial(right_serial)
        left.Init()
        right.Init()
    except PySpin.SpinnakerException:
        cam_list.Clear()
        system.ReleaseInstance()
        raise
    return system, cam_list, left, right


def disconnect_cameras(
    system: Optional[PySpin.System],
    cam_list: Optional[PySpin.CameraList],
    pair: Tuple[Optional[PySpin.CameraPtr], Optional[PySpin.CameraPtr]],
) -> None:
    left, right = pair
    for name, cam in (("left", left), ("right", right)):
        if cam is None:
            continue
        try:
            if hasattr(cam, "IsInitialized") and cam.IsInitialized():
                cam.DeInit()
                print(f"{Ansi.GREEN}{name} camera deinitialised.{Ansi.RESET}")
        except PySpin.SpinnakerException as exc:
            print(f"{Ansi.YELLOW}Warning: failed to deinit {name} camera -> {exc}{Ansi.RESET}")
    if cam_list is not None:
        try:
            cam_list.Clear()
        except PySpin.SpinnakerException as exc:
            print(f"{Ansi.YELLOW}Warning: failed to clear camera list -> {exc}{Ansi.RESET}")
    if system is not None:
        try:
            system.ReleaseInstance()
        except PySpin.SpinnakerException as exc:
            print(f"{Ansi.YELLOW}Warning: failed to release system instance -> {exc}{Ansi.RESET}")
    print(f"{Ansi.GREEN}Camera resources released.{Ansi.RESET}")


def setup_light_controller(config: dict) -> Optional[lc.PWMDriver]:
    port = config.get("LIGHT_SERIAL_PORT")
    if not port:
        print(f"{Ansi.YELLOW}LIGHT_SERIAL_PORT not provided; skipping light controller setup.{Ansi.RESET}")
        return None
    controller = lc.PWMDriver(
        port=port,
        baud=config.get("LIGHT_BAUD_RATE", 115200),
        verbose=config.get("LIGHT_VERBOSE", False),
    )
    if hasattr(controller, "stop"):
        controller.stop()
    return controller


def set_light_intensity(light: Optional[lc.PWMDriver], value: float, settle_sec: float) -> None:
    if light is None:
        return
    try:
        light.intensity = float(value)
        if settle_sec > 0:
            time.sleep(settle_sec)
    except AttributeError:
        if hasattr(light, "set_intensity"):
            light.set_intensity(float(value))
            if settle_sec > 0:
                time.sleep(settle_sec)


def configure_master_linear_camera(
    cam: PySpin.CameraPtr,
    *,
    exposure_time_us: float,
    gain_db: float,
    target_fps: float,
    output_line: str,
    enable_chunk: bool,
    enable_3v3: bool,
) -> None:
    nodemap = cam.GetNodeMap()
    _set_enum(nodemap, "AcquisitionMode", "Continuous")
    if target_fps > 0:
        if _set_bool(nodemap, "AcquisitionFrameRateEnable", True):
            _set_float(nodemap, "AcquisitionFrameRate", target_fps)
    else:
        _set_bool(nodemap, "AcquisitionFrameRateEnable", False)
    _set_enum(nodemap, "TriggerMode", "Off")
    _set_enum(nodemap, "TriggerSelector", "FrameStart")
    _set_enum(nodemap, "SensorShutterMode", "Global")
    _set_enum(nodemap, "ExposureMode", "Timed")
    _set_enum(nodemap, "ExposureAuto", "Off")
    _set_float(nodemap, "ExposureTime", exposure_time_us)
    _set_enum(nodemap, "GainAuto", "Off")
    _set_float(nodemap, "Gain", gain_db)
    set_raw_pixel_format(nodemap)

    _set_enum(nodemap, "LineSelector", output_line)
    _set_enum(nodemap, "LineMode", "Output")
    _set_enum(nodemap, "LineFormat", "OptoCoupled")
    _set_enum(nodemap, "LineSource", "ExposureActive")
    _set_bool(nodemap, "LineInverter", False)
    _set_enum(nodemap, "InputFilterSelector", "Deglitch")
    _set_float(nodemap, "LineFilterWidth", 0.0)
    _set_bool(nodemap, "V3_3Enable", enable_3v3)

    if enable_chunk:
        enable_chunk_data(cam)


def update_master_exposure(cam: PySpin.CameraPtr, exposure_time_us: float) -> None:
    nodemap = cam.GetNodeMap()
    _set_float(nodemap, "ExposureTime", exposure_time_us)


def configure_master_color_ae_camera(
    cam: PySpin.CameraPtr,
    *,
    gain_db: float,
    target_fps: float,
    output_line: str,
    enable_chunk: bool,
    enable_3v3: bool,
    pixel_format: str = "BGR8",
) -> None:
    """Configure master (left) camera for Color mode with Auto Exposure."""
    nodemap = cam.GetNodeMap()
    _set_enum(nodemap, "AcquisitionMode", "Continuous")
    if target_fps > 0:
        if _set_bool(nodemap, "AcquisitionFrameRateEnable", True):
            _set_float(nodemap, "AcquisitionFrameRate", target_fps)
    else:
        _set_bool(nodemap, "AcquisitionFrameRateEnable", False)
    _set_enum(nodemap, "TriggerMode", "Off")
    _set_enum(nodemap, "TriggerSelector", "FrameStart")
    _set_enum(nodemap, "SensorShutterMode", "Global")
    _set_enum(nodemap, "ExposureMode", "Timed")
    _set_enum(nodemap, "ExposureAuto", "Continuous")
    _set_enum(nodemap, "GainAuto", "Off")
    _set_float(nodemap, "Gain", gain_db)

    # Set pixel format to color
    pixel_node = PySpin.CEnumerationPtr(nodemap.GetNode("PixelFormat"))
    if PySpin.IsAvailable(pixel_node) and PySpin.IsWritable(pixel_node):
        entry = pixel_node.GetEntryByName(pixel_format)
        if PySpin.IsAvailable(entry) and PySpin.IsReadable(entry):
            pixel_node.SetIntValue(entry.GetValue())

    # Configure trigger output (same as linear master)
    _set_enum(nodemap, "LineSelector", output_line)
    _set_enum(nodemap, "LineMode", "Output")
    _set_enum(nodemap, "LineFormat", "OptoCoupled")
    _set_enum(nodemap, "LineSource", "ExposureActive")
    _set_bool(nodemap, "LineInverter", False)
    _set_enum(nodemap, "InputFilterSelector", "Deglitch")
    _set_float(nodemap, "LineFilterWidth", 0.0)
    _set_bool(nodemap, "V3_3Enable", enable_3v3)

    if enable_chunk:
        enable_chunk_data(cam)


def read_ae_exposure_time(cam: PySpin.CameraPtr, chunk_data: Optional[Dict[str, float]] = None) -> Optional[float]:
    """Read exposure time from chunk data or nodemap after AE."""
    # Try chunk data first
    if chunk_data and "ExposureTime" in chunk_data:
        return float(chunk_data["ExposureTime"])
    
    # Fallback to reading from nodemap
    nodemap = cam.GetNodeMap()
    exposure_node = PySpin.CFloatPtr(nodemap.GetNode("ExposureTime"))
    if PySpin.IsAvailable(exposure_node) and PySpin.IsReadable(exposure_node):
        try:
            return float(exposure_node.GetValue())
        except PySpin.SpinnakerException:
            pass
    return None


def lock_ae_exposure(
    master_cam: PySpin.CameraPtr,
    slave_cam: PySpin.CameraPtr,
    locked_exposure_us: float,
    *,
    master_gain_db: float,
    slave_gain_db: float,
    slave_trigger_line: str,
    slave_trigger_delay_us: float,
    master_chunk: bool,
    slave_chunk: bool,
    master_3v3: bool,
    slave_3v3: bool,
) -> None:
    """Lock the AE exposure and configure both cameras to use it."""
    # Lock master camera: turn off AE, set exposure manually
    master_nodemap = master_cam.GetNodeMap()
    _set_enum(master_nodemap, "ExposureAuto", "Off")
    _set_enum(master_nodemap, "ExposureMode", "Timed")
    _set_float(master_nodemap, "ExposureTime", locked_exposure_us)
    
    # Configure slave camera (linear) to use the locked exposure
    configure_slave_linear_camera(
        slave_cam,
        trigger_line=slave_trigger_line,
        trigger_delay_us=slave_trigger_delay_us,
        exposure_time_us=locked_exposure_us,
        gain_db=slave_gain_db,
        enable_chunk=slave_chunk,
        enable_3v3=slave_3v3,
    )


def configure_slave_linear_camera(
    cam: PySpin.CameraPtr,
    *,
    trigger_line: str,
    trigger_delay_us: float,
    exposure_time_us: float,
    gain_db: float,
    enable_chunk: bool,
    enable_3v3: bool,
) -> None:
    """Configure slave (right) camera for Linear mode with manual exposure."""
    nodemap = cam.GetNodeMap()
    _set_enum(nodemap, "AcquisitionMode", "Continuous")
    _set_bool(nodemap, "AcquisitionFrameRateEnable", False)
    _set_enum(nodemap, "SensorShutterMode", "Global")
    _set_enum(nodemap, "ExposureMode", "Timed")
    _set_enum(nodemap, "ExposureAuto", "Off")
    _set_float(nodemap, "ExposureTime", exposure_time_us)
    _set_enum(nodemap, "GainAuto", "Off")
    _set_float(nodemap, "Gain", gain_db)
    set_raw_pixel_format(nodemap)

    _set_enum(nodemap, "TriggerSelector", "FrameStart")
    _set_enum(nodemap, "TriggerMode", "On")
    _set_enum(nodemap, "TriggerSource", trigger_line)
    _set_enum(nodemap, "TriggerActivation", "RisingEdge")
    _set_enum(nodemap, "TriggerOverlap", "Off")
    _set_float(nodemap, "TriggerDelay", trigger_delay_us)

    _set_enum(nodemap, "LineSelector", trigger_line)
    _set_enum(nodemap, "LineMode", "Input")
    _set_enum(nodemap, "LineFormat", "TriState")
    _set_enum(nodemap, "LineSource", "Off")
    _set_bool(nodemap, "LineInverter", False)
    _set_enum(nodemap, "InputFilterSelector", "Deglitch")
    _set_float(nodemap, "LineFilterWidth", 0.0)
    _set_bool(nodemap, "V3_3Enable", enable_3v3)

    if enable_chunk:
        enable_chunk_data(cam)


def configure_slave_color_camera(
    cam: PySpin.CameraPtr,
    *,
    trigger_line: str,
    trigger_delay_us: float,
    pixel_format: str,
    enable_chunk: bool,
    enable_3v3: bool,
) -> None:
    nodemap = cam.GetNodeMap()
    _set_enum(nodemap, "AcquisitionMode", "Continuous")
    _set_bool(nodemap, "AcquisitionFrameRateEnable", False)
    _set_enum(nodemap, "SensorShutterMode", "Global")
    
    # Auto Exposure / Gain / White Balance
    _set_enum(nodemap, "ExposureMode", "Timed")
    _set_enum(nodemap, "ExposureAuto", "Continuous")
    _set_enum(nodemap, "GainAuto", "Continuous")
    _set_enum(nodemap, "BalanceWhiteAuto", "Continuous")

    _set_enum(nodemap, "TriggerSelector", "FrameStart")
    _set_enum(nodemap, "TriggerMode", "On")
    _set_enum(nodemap, "TriggerSource", trigger_line)
    _set_enum(nodemap, "TriggerActivation", "RisingEdge")
    _set_enum(nodemap, "TriggerOverlap", "Off")
    _set_float(nodemap, "TriggerDelay", trigger_delay_us)

    _set_enum(nodemap, "LineSelector", trigger_line)
    _set_enum(nodemap, "LineMode", "Input")
    _set_enum(nodemap, "LineFormat", "TriState")
    _set_enum(nodemap, "LineSource", "Off")
    _set_bool(nodemap, "LineInverter", False)
    _set_enum(nodemap, "InputFilterSelector", "Deglitch")
    _set_float(nodemap, "LineFilterWidth", 0.0)
    _set_bool(nodemap, "V3_3Enable", enable_3v3)

    pixel_node = PySpin.CEnumerationPtr(nodemap.GetNode("PixelFormat"))
    if PySpin.IsAvailable(pixel_node) and PySpin.IsWritable(pixel_node):
        entry = pixel_node.GetEntryByName(pixel_format)
        if PySpin.IsAvailable(entry) and PySpin.IsReadable(entry):
            pixel_node.SetIntValue(entry.GetValue())

    if enable_chunk:
        enable_chunk_data(cam)


def begin_synced_acquisition(master_cam: PySpin.CameraPtr, slave_cam: PySpin.CameraPtr) -> None:
    slave_cam.BeginAcquisition()
    master_cam.BeginAcquisition()


def end_synced_acquisition(master_cam: PySpin.CameraPtr, slave_cam: PySpin.CameraPtr) -> None:
    for name, cam in (("slave", slave_cam), ("master", master_cam)):
        try:
            if hasattr(cam, "IsStreaming") and cam.IsStreaming():
                cam.EndAcquisition()
        except PySpin.SpinnakerException as exc:
            print(f"{Ansi.YELLOW}Warning: failed to stop {name} acquisition -> {exc}{Ansi.RESET}")


def _fetch_image(cam: PySpin.CameraPtr, timeout_ms: int, expect_chunk: bool) -> Tuple[np.ndarray, Dict[str, float]]:
    image = cam.GetNextImage(timeout_ms)
    if image.IsIncomplete():
        status = image.GetImageStatus()
        image.Release()
        raise RuntimeError(f"Incomplete image received (status {status}).")
    chunk_fields: Dict[str, float] = {}
    if expect_chunk:
        try:
            chunk_fields = chunk_to_dict(image.GetChunkData())
        except PySpin.SpinnakerException:
            chunk_fields = {}
    array = image.GetNDArray()
    image.Release()
    return array, chunk_fields


def capture_synced_frame_pair(
    master_cam: PySpin.CameraPtr,
    slave_cam: PySpin.CameraPtr,
    *,
    timeout_ms: int,
    num_discard: int,
    master_chunk: bool,
    slave_chunk: bool,
) -> Tuple[np.ndarray, Dict[str, float], np.ndarray, Dict[str, float]]:
    begin_synced_acquisition(master_cam, slave_cam)
    try:
        for _ in range(max(num_discard, 0)):
            image = master_cam.GetNextImage(timeout_ms)
            if not image.IsIncomplete():
                image.Release()
            else:
                status = image.GetImageStatus()
                image.Release()
                raise RuntimeError(f"Incomplete master discard frame (status {status}).")
            image = slave_cam.GetNextImage(timeout_ms)
            if not image.IsIncomplete():
                image.Release()
            else:
                status = image.GetImageStatus()
                image.Release()
                raise RuntimeError(f"Incomplete slave discard frame (status {status}).")
        left_frame, left_chunk = _fetch_image(master_cam, timeout_ms, master_chunk)
        right_frame, right_chunk = _fetch_image(slave_cam, timeout_ms, slave_chunk)
        return left_frame, left_chunk, right_frame, right_chunk
    finally:
        end_synced_acquisition(master_cam, slave_cam)


def save_frame_to_disk(
    frame: np.ndarray,
    metadata: dict,
    filename: str,
    directory: str,
    *,
    timestamp: Optional[datetime.datetime] = None,
    suffix: Optional[str] = None,
) -> str:
    ts = timestamp or datetime.datetime.now()
    ts_str = ts.strftime("%Y%m%d_%H%M%S_%f")
    base = f"{filename}_{ts_str}"
    if suffix:
        base = f"{base}_{suffix}"
    os.makedirs(directory, exist_ok=True)
    base_path = os.path.join(directory, base)
    np.save(f"{base_path}.npy", frame)
    with open(f"{base_path}_metadata.json", "w") as handle:
        json.dump(metadata, handle, indent=4)
    print(f"{Ansi.GREEN}Saved{Ansi.RESET} {base_path}.npy")
    return base_path


def compile_linear_metadata(
    config: dict,
    chunk_data: Dict[str, float],
    *,
    exposure_time_us: float,
    gain_db: float,
    light_intensity: float,
    ae_mode: bool = False,
    ae_locked_exposure_us: Optional[float] = None,
) -> dict:
    metadata = {
        "camera": {
            "serial": config["CAMERA_SERIAL"],
            "mode": "scene_linear",
            "set_exposure_time_us": exposure_time_us,
            "set_gain_db": gain_db,
            "ae_mode": ae_mode,
        },
        "light": {
            "set_intensity": light_intensity,
        },
        "chunk_data": chunk_data,
    }
    if ae_locked_exposure_us is not None:
        metadata["camera"]["ae_locked_exposure_us"] = ae_locked_exposure_us
    return metadata


def compile_color_metadata(
    config: dict,
    chunk_data: Dict[str, float],
    *,
    exposure_time_us: float,
    gain_db: float,
    light_intensity: float,
    pixel_format: str,
    ae_mode: bool = False,
    ae_locked_exposure_us: Optional[float] = None,
) -> dict:
    metadata = {
        "camera": {
            "serial": config["AUTO_CAMERA_SERIAL"],
            "mode": "processed",
            "pixel_format": pixel_format,
            "set_exposure_time_us": exposure_time_us,
            "set_gain_db": gain_db,
            "ae_mode": ae_mode,
        },
        "light": {
            "set_intensity": light_intensity,
        },
        "chunk_data": chunk_data,
    }
    if ae_locked_exposure_us is not None:
        metadata["camera"]["ae_locked_exposure_us"] = ae_locked_exposure_us
    return metadata


def _format_scene_name(config: dict, user_scene: str) -> str:
    prefix = (
        config.get("pilot_home")
        or config.get("PILOT_HOME")
        or config.get("SEQUENCE_NAME_PREFIX")
        or ""
    )
    suffix = config.get("SCENE_SUFFIX", "")
    return f"{prefix}{user_scene}{suffix}"


def settle_auto_exposure(cam: PySpin.CameraPtr, settle_time_sec: float = 2.0) -> None:
    """Run camera in free-run mode to let AE/AWB settle."""
    nodemap = cam.GetNodeMap()
    # Disable trigger for free-run
    _set_enum(nodemap, "TriggerMode", "Off")
    _set_enum(nodemap, "AcquisitionMode", "Continuous")
    
    # Enable Auto algorithms
    _set_enum(nodemap, "ExposureAuto", "Continuous")
    _set_enum(nodemap, "GainAuto", "Continuous")
    _set_enum(nodemap, "BalanceWhiteAuto", "Continuous")
    
    print(f"{Ansi.YELLOW}Settling Auto Exposure/White Balance for {settle_time_sec}s...{Ansi.RESET}")
    try:
        cam.BeginAcquisition()
        end_time = time.time() + settle_time_sec
        while time.time() < end_time:
            image = cam.GetNextImage(1000)
            image.Release()
        cam.EndAcquisition()
    except PySpin.SpinnakerException as e:
        print(f"{Ansi.RED}Error during settling: {e}{Ansi.RESET}")


def capture_scene(
    config: dict,
    scene_name: str,
    left_cam: PySpin.CameraPtr,
    right_cam: PySpin.CameraPtr,
    light_controller: Optional[lc.PWMDriver],
) -> None:
    dataset_root = os.path.expanduser(config.get("DATASET_LOCATION", "data"))
    os.makedirs(dataset_root, exist_ok=True)
    full_scene = _format_scene_name(config, scene_name)
    scene_dir = os.path.join(dataset_root, full_scene)
    os.makedirs(scene_dir, exist_ok=True)
    print(f"{Ansi.CYAN}Captures will be saved under{Ansi.RESET} {scene_dir}")

    center_exp = _read_exposure_us(config, "CENTER_EXPOSURE", fallback=10000.0)
    bracket_steps = int(config.get("EXPOSURE_BRACKET_STEPS", 2))
    exposures = build_exposure_schedule(center_exp, bracket_steps)
    light_step = float(config.get("LIGHT_INTENSITY_INCR", 10.0))
    intensities = build_light_schedule(light_step)

    print(
        f"{Ansi.BLUE}Sweep summary:{Ansi.RESET} {len(exposures)} exposures x {len(intensities)} light levels"
        f" -> {len(exposures) * len(intensities)} frame pairs."
    )

    settle_sec = float(config.get("LIGHT_SETTLE_TIME_SEC", 0.5))
    discard_frames = int(config.get("SYNC_NUM_DISCARD_FRAMES", 2))
    timeout_ms = int(config.get("SYNC_CAPTURE_TIMEOUT_MS", 2000))
    target_fps = float(config.get("SYNC_TARGET_FPS", 10.0))
    master_chunk_enabled = bool(config.get("ENABLE_CHUNK", True))
    slave_chunk_enabled = bool(config.get("AUTO_ENABLE_CHUNK", True))

    master_line = config.get("SYNC_MASTER_OUTPUT_LINE", "Line1")
    slave_line = config.get("SYNC_SLAVE_TRIGGER_LINE", "Line3")
    master_3v3 = bool(config.get("SYNC_MASTER_ENABLE_3V3", False))
    slave_3v3 = bool(config.get("SYNC_SLAVE_ENABLE_3V3", False))
    slave_trigger_delay = float(config.get("SYNC_SLAVE_TRIGGER_DELAY_US", 0.0))
    master_gain = float(config.get("GAIN_DB", 0.0))
    slave_gain = float(config.get("SLAVE_GAIN_DB", 0.0))
    slave_exposure = _read_exposure_us(
        config,
        "SLAVE_EXPOSURE",
        fallback=config.get("AUTO_EXPOSURE_TIME_US", 8000.0),
    )
    slave_pixel_format = config.get("SLAVE_PIXEL_FORMAT", "BGR8")

    intensity_dirs = {}
    for intensity in intensities:
        label = f"P{intensity:.2f}"
        path = os.path.join(scene_dir, label)
        os.makedirs(path, exist_ok=True)
        intensity_dirs[intensity] = path

    try:
        configure_master_linear_camera(
            left_cam,
            exposure_time_us=exposures[0],
            gain_db=master_gain,
            target_fps=target_fps,
            output_line=master_line,
            enable_chunk=master_chunk_enabled,
            enable_3v3=master_3v3,
        )
        configure_slave_color_camera(
            right_cam,
            trigger_line=slave_line,
            trigger_delay_us=slave_trigger_delay,
            pixel_format=slave_pixel_format,
            enable_chunk=slave_chunk_enabled,
            enable_3v3=slave_3v3,
        )
        light_controller = setup_light_controller(config)
        set_light_intensity(light_controller, 0.0, settle_sec)

        capture_index = 1
        for intensity in intensities:
            set_light_intensity(light_controller, intensity, settle_sec)
            print(f"{Ansi.BLUE}Light intensity set to P={intensity:.2f}{Ansi.RESET}")
            
            # Settle Auto Exposure on Slave Camera
            settle_auto_exposure(right_cam, settle_time_sec=2.0)
            
            # Re-enable Trigger on Slave Camera
            configure_slave_color_camera(
                right_cam,
                trigger_line=slave_line,
                trigger_delay_us=slave_trigger_delay,
                pixel_format=slave_pixel_format,
                enable_chunk=slave_chunk_enabled,
                enable_3v3=slave_3v3,
            )
            
            # Manual exposure sweep for Master
            for exp_us in exposures:
                print(f"{Ansi.CYAN}Setting master exposure to{Ansi.RESET} {exp_us:.1f} us")
                update_master_exposure(left_cam, exp_us)
                time.sleep(max(0.01, exp_us / 1e6))
                print(
                    f"{Ansi.BLUE}Capture #{capture_index:04d}{Ansi.RESET} | "
                    f"P={intensity:.2f} | E={exp_us:.0f}us"
                )
                (
                    linear_frame,
                    linear_chunk,
                    color_frame,
                    color_chunk,
                ) = capture_synced_frame_pair(
                    left_cam,
                    right_cam,
                    timeout_ms=timeout_ms,
                    num_discard=discard_frames,
                    master_chunk=master_chunk_enabled,
                    slave_chunk=slave_chunk_enabled,
                )
                timestamp = datetime.datetime.now()
                base_filename = f"{full_scene}_P{intensity:.2f}_E{int(exp_us):06d}_{capture_index:04d}"
                
                metadata_linear = compile_linear_metadata(
                    config,
                    linear_chunk,
                    exposure_time_us=exp_us,
                    gain_db=master_gain,
                    light_intensity=float(intensity),
                )
                
                # Get actual exposure from chunk if possible
                actual_color_exp = read_ae_exposure_time(right_cam, color_chunk) or 0.0
                
                metadata_color = compile_color_metadata(
                    config,
                    color_chunk,
                    exposure_time_us=actual_color_exp,
                    gain_db=0.0, # Auto
                    light_intensity=float(intensity),
                    pixel_format=slave_pixel_format,
                    ae_mode=True,
                )
                directory = intensity_dirs[intensity]
                save_frame_to_disk(
                    linear_frame,
                    metadata_linear,
                    filename=base_filename,
                    directory=directory,
                    timestamp=timestamp,
                )
                save_frame_to_disk(
                    color_frame,
                    metadata_color,
                    filename=base_filename,
                    directory=directory,
                    timestamp=timestamp,
                    suffix="color",
                )
                capture_index += 1
            
        print(f"{Ansi.GREEN}Capture sweep complete for scene{Ansi.RESET} {full_scene}")
    except Exception as e:
        print(f"{Ansi.RED}Error during scene capture: {e}{Ansi.RESET}")
        raise


def main() -> None:
    config_path = os.path.join(os.path.dirname(__file__), "..", "stereo-config-bracket.yaml")
    config = load_config(config_path)
    
    print(f"{Ansi.CYAN}=== Stereo Sync CLID Capture ==={Ansi.RESET}")
    print(f"{Ansi.BLUE}Enter a base scene name. Subsequent scenes will be numbered automatically.{Ansi.RESET}\n")
    
    # Get base scene name once
    base_scene = input(f"{Ansi.CYAN}Base scene name (blank or 'q' to quit): {Ansi.RESET}").strip()
    
    if not base_scene or base_scene.lower() in ('q', 'quit'):
        print(f"{Ansi.YELLOW}Exiting capture session.{Ansi.RESET}")
        return
    
    # Initialize cameras and light controller once for the entire session
    system = None
    cam_list = None
    left_cam = None
    right_cam = None
    light_controller = None
    
    try:
        print(f"{Ansi.CYAN}Initializing cameras...{Ansi.RESET}")
        system, cam_list, left_cam, right_cam = connect_sync_cameras(config)
        light_controller = setup_light_controller(config)
        print(f"{Ansi.GREEN}Cameras initialized.{Ansi.RESET}\n")
        
        scene_counter = 0
        
        while True:
            # Generate scene name with counter
            if scene_counter == 0:
                scene_name = base_scene
            else:
                scene_name = f"{base_scene}_{scene_counter}"
            
            print(f"{Ansi.CYAN}Capturing scene: {scene_name}{Ansi.RESET}")
            
            try:
                capture_scene(config, scene_name, left_cam, right_cam, light_controller)
                print(f"{Ansi.GREEN}Scene '{scene_name}' capture complete!{Ansi.RESET}\n")
                scene_counter += 1
            except KeyboardInterrupt:
                print(f"\n{Ansi.YELLOW}Capture interrupted by user.{Ansi.RESET}")
                response = input(f"{Ansi.CYAN}Continue with next scene? (y/n): {Ansi.RESET}").strip().lower()
                if response not in ('y', 'yes'):
                    print(f"{Ansi.YELLOW}Exiting capture session.{Ansi.RESET}")
                    break
                scene_counter += 1
                continue
            except Exception as e:
                print(f"{Ansi.RED}Error capturing scene '{scene_name}': {e}{Ansi.RESET}")
                response = input(f"{Ansi.CYAN}Continue with next scene? (y/n): {Ansi.RESET}").strip().lower()
                if response not in ('y', 'yes'):
                    print(f"{Ansi.YELLOW}Exiting capture session.{Ansi.RESET}")
                    break
                scene_counter += 1
                continue
            
            # Ask if user wants to capture another scene
            response = input(f"{Ansi.CYAN}Next scene? (y/n, default=y): {Ansi.RESET}").strip().lower()
            if response in ('n', 'no', 'q', 'quit'):
                print(f"{Ansi.YELLOW}Exiting capture session.{Ansi.RESET}")
                break
    
    finally:
        # Clean up cameras and light controller only at the end
        print(f"\n{Ansi.CYAN}Cleaning up resources...{Ansi.RESET}")
        if light_controller and hasattr(light_controller, "stop"):
            try:
                light_controller.stop()
            except Exception as e:
                print(f"{Ansi.YELLOW}Warning: Error stopping light controller: {e}{Ansi.RESET}")
        if left_cam is not None or right_cam is not None:
            disconnect_cameras(system, cam_list, (left_cam, right_cam))
        print(f"{Ansi.GREEN}Cleanup complete.{Ansi.RESET}")


if __name__ == "__main__":
    main()
