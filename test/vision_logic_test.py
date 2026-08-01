#!/usr/bin/env python3

from __future__ import annotations

import ast
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VISION = ROOT / "vision" / "yanyanview5.py"
CALIBRATION = ROOT / "vision" / "calibration.json"

class FakeCircle:
    def __init__(self, x: int, y: int, radius: int) -> None:
        self._x = x
        self._y = y
        self._radius = radius

    def x(self) -> int:
        return self._x

    def y(self) -> int:
        return self._y

    def r(self) -> int:
        return self._radius

def load_ring_selection_functions():
    source = VISION.read_text(encoding="utf-8")
    tree = ast.parse(source)
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name
        in {
            "filter_outer_ring_candidates",
            "select_center_ring",
            "select_centered_ring_fallback",
            "select_endpoint_ring",
        }
    ]
    module = ast.Module(body=functions, type_ignores=[])
    namespace = {
        "ring_min_spacing_pixels": 20,
        "ring_max_spacing_ratio": 1.45,
        "ring_max_line_error_ratio": 0.18,
        "ring_max_radius_ratio": 1.60,
        "ring_candidate_min_radius_pixels": 28,
        "ring_candidate_min_largest_radius_ratio": 0.55,
        "image_center_x": 160,
        "image_center_y": 120,
        "ring_fallback_max_center_distance_pixels": 70,
        "ring_fallback_min_distance_margin_pixels": 15,
        "ring_fallback_min_dominant_radius_ratio": 0.75,
        "endpoint_max_center_distance_pixels": 90,
        "endpoint_min_dominant_radius_ratio": 0.75,
    }
    exec(compile(module, str(VISION), "exec"), namespace)
    return (
        namespace["select_center_ring"],
        namespace["select_centered_ring_fallback"],
        namespace["select_endpoint_ring"],
    )

def load_protocol_functions():
    source = VISION.read_text(encoding="utf-8")
    tree = ast.parse(source)
    wanted = {
        "crc8",
        "decode_v2_request_frame",
        "build_v2_response",
        "detection_mode_for_request",
    }
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name in wanted
    ]
    module = ast.Module(body=functions, type_ignores=[])
    namespace = {
        "request_frame_length": 6,
        "frame_header": 0xAA,
        "frame_tail": 0xBB,
        "PROTOCOL_VERSION": 2,
    }
    exec(compile(module, str(VISION), "exec"), namespace)
    return (
        namespace["crc8"],
        namespace["decode_v2_request_frame"],
        namespace["build_v2_response"],
        namespace["detection_mode_for_request"],
    )

def load_circle_stability_functions():
    source = VISION.read_text(encoding="utf-8")
    tree = ast.parse(source)
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name
        in {
            "reset_circle_stability",
            "update_circle_stability",
        }
    ]
    module = ast.Module(body=functions, type_ignores=[])
    namespace = {
        "duration": 500,
        "circle_stable_minimum_samples": 5,
        "circle_stable_max_span": 3,
        "circle_stable_max_radius_span": 3,
        "endpoint_stable_duration": 120,
        "endpoint_stable_minimum_samples": 2,
        "endpoint_stable_max_span": 2,
        "endpoint_stable_max_radius_span": 2,
        "current_color_to_detect": 9,
        "circle_start_time": None,
        "circle_min_x": 0,
        "circle_max_x": 0,
        "circle_min_y": 0,
        "circle_max_y": 0,
        "circle_min_r": 0,
        "circle_max_r": 0,
        "circle_sum_x": 0,
        "circle_sum_y": 0,
        "circle_sum_r": 0,
        "circle_sample_count": 0,
    }
    exec(compile(module, str(VISION), "exec"), namespace)
    return (
        namespace["update_circle_stability"],
        namespace["reset_circle_stability"],
        namespace,
    )

def load_color_stability_functions():
    source = VISION.read_text(encoding="utf-8")
    tree = ast.parse(source)
    wanted = {
        "reset_color_stability",
        "update_color_stability",
        "stable_color_coordinate",
        "latest_color_coordinate",
        "color_stability_profile",
        "color_response_coordinate",
    }
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name in wanted
    ]
    module = ast.Module(body=functions, type_ignores=[])
    namespace = {
        "multi_color_duration": 300,
        "multi_color_max_span": 3,
        "multi_color_minimum_samples": 5,
        "target_color_duration": 0,
        "target_color_max_span": 12,
        "target_color_minimum_samples": 2,
        "color_stable_start_times": [None, None, None, None],
        "color_min_x": [0, 0, 0, 0],
        "color_max_x": [0, 0, 0, 0],
        "color_min_y": [0, 0, 0, 0],
        "color_max_y": [0, 0, 0, 0],
        "color_latest_x": [0, 0, 0, 0],
        "color_latest_y": [0, 0, 0, 0],
        "color_sample_counts": [0, 0, 0, 0],
        "color_sum_x": [0, 0, 0, 0],
        "color_sum_y": [0, 0, 0, 0],
    }
    exec(compile(module, str(VISION), "exec"), namespace)
    return (
        namespace["update_color_stability"],
        namespace["reset_color_stability"],
        namespace["stable_color_coordinate"],
        namespace["latest_color_coordinate"],
        namespace["color_stability_profile"],
        namespace["color_response_coordinate"],
    )

def main() -> None:
    (
        select_center_ring,
        select_centered_ring_fallback,
        select_endpoint_ring,
    ) = load_ring_selection_functions()
    (
        crc8,
        decode_request,
        build_response,
        detection_mode_for_request,
    ) = load_protocol_functions()
    (
        update_circle_stability,
        reset_circle_stability,
        circle_stability_namespace,
    ) = (
        load_circle_stability_functions()
    )
    (
        update_color_stability,
        reset_color_stability,
        stable_color_coordinate,
        latest_color_coordinate,
        color_stability_profile,
        color_response_coordinate,
    ) = load_color_stability_functions()
    assert crc8(b"123456789") == 0xF4

    assert not update_color_stability(0, 100, 100, 0, 0, 12, 2)
    assert update_color_stability(0, 108, 105, 10, 0, 12, 2)
    assert stable_color_coordinate(0) == (104, 102)
    assert latest_color_coordinate(0) == (108, 105)
    assert color_stability_profile(1) == (0, 12, 2)
    assert color_stability_profile(4) == (0, 12, 2)
    assert color_stability_profile(8) == (300, 3, 5)

    assert color_response_coordinate(1, 0) == (108, 105)
    assert color_response_coordinate(8, 0) == (104, 102)
    reset_color_stability(0)
    assert not update_color_stability(0, 100, 100, 0, 0, 12, 2)

    assert not update_color_stability(0, 113, 100, 10, 0, 12, 2)
    assert update_color_stability(0, 120, 103, 20, 0, 12, 2)

    stable_result = None
    for time_ms, radius in zip(
        (0, 100, 200, 300, 400, 500),
        (70, 71, 72, 73, 72, 71),
    ):
        stable_result = update_circle_stability(
            FakeCircle(160, 120, radius),
            time_ms,
        )
    assert stable_result == (160, 120, 72)

    reset_circle_stability()
    circle_stability_namespace[
        "current_color_to_detect"
    ] = 10
    endpoint_stable_result = None
    for time_ms, radius in zip(
        (0, 120),
        (84, 85),
    ):
        endpoint_stable_result = update_circle_stability(
            FakeCircle(160, 120, radius),
            time_ms,
        )
    assert endpoint_stable_result == (160, 120, 84)

    left = FakeCircle(60, 121, 40)
    center = FakeCircle(160, 120, 41)
    right = FakeCircle(261, 119, 40)
    distractor = FakeCircle(154, 40, 12)
    assert (
        select_center_ring([right, distractor, left, center])
        is center
    )

    assert select_center_ring([left, center]) is None
    assert select_centered_ring_fallback([center]) is center
    assert (
        select_centered_ring_fallback(
            [FakeCircle(10, 10, 40)]
        )
        is None
    )
    assert (
        select_centered_ring_fallback(
            [
                FakeCircle(150, 120, 40),
                FakeCircle(171, 120, 40),
            ]
        )
        is None
    )
    near_center = FakeCircle(163, 122, 60)
    digit_two = FakeCircle(160, 120, 12)
    digit_three = FakeCircle(158, 121, 18)
    assert (
        select_centered_ring_fallback(
            [digit_two, near_center, digit_three]
        )
        is near_center
    )
    assert (
        select_centered_ring_fallback(
            [digit_two, digit_three]
        )
        is None
    )
    assert (
        select_center_ring(
            [
                FakeCircle(60, 120, 60),
                FakeCircle(160, 120, 61),
                FakeCircle(260, 120, 60),
                FakeCircle(60, 120, 12),
                FakeCircle(160, 120, 16),
                FakeCircle(260, 120, 18),
            ]
        )
        .r()
        == 61
    )
    assert (
        select_center_ring(
            [
                FakeCircle(40, 120, 40),
                FakeCircle(80, 120, 40),
                FakeCircle(250, 120, 40),
            ]
        )
        is None
    )

    endpoint = FakeCircle(166, 117, 42)
    digit_one = FakeCircle(160, 120, 12)
    other_ring = FakeCircle(245, 122, 41)
    assert (
        select_endpoint_ring(
            [digit_one, other_ring, endpoint, digit_three]
        )
        is endpoint
    )
    assert (
        select_endpoint_ring(
            [endpoint, FakeCircle(161, 120, 30)]
        )
        is endpoint
    )

    assert (
        select_endpoint_ring(
            [endpoint, FakeCircle(310, 230, 90)]
        )
        is endpoint
    )
    assert (
        select_endpoint_ring([FakeCircle(10, 10, 42)])
        is None
    )
    assert select_endpoint_ring([digit_one, digit_three]) is None

    request = bytearray([0xAA, 0x02, 0x37, 0x08, 0x00, 0xBB])
    request[4] = crc8(request[1:4])
    assert decode_request(request) == (0x37, 0x08)
    damaged_request = bytearray(request)
    damaged_request[4] ^= 0x01
    assert decode_request(damaged_request) is None
    known_request = bytearray(
        [0xAA, 0x02, 0x34, 0x09, 0x44, 0xBB]
    )
    assert decode_request(known_request) == (0x34, 0x09)
    endpoint_request = bytearray(
        [0xAA, 0x02, 0x35, 0x0A, 0x00, 0xBB]
    )
    endpoint_request[4] = crc8(endpoint_request[1:4])
    assert decode_request(endpoint_request) == (0x35, 0x0A)
    assert detection_mode_for_request(0x35, 10, -1) == 10
    assert detection_mode_for_request(0x35, 10, 0x35) == 10
    assert detection_mode_for_request(0x34, 9, -1) == 9
    assert detection_mode_for_request(0x34, 9, 0x34) == 9
    for target_mode in (1, 2, 3, 4):
        assert (
            detection_mode_for_request(0x40, target_mode, -1)
            == target_mode
        )
        assert detection_mode_for_request(0x40, target_mode, 0x40) == 0
    assert detection_mode_for_request(0x37, 8, -1) == 8
    assert detection_mode_for_request(0x37, 8, 0x37) == 0
    assert detection_mode_for_request(0x38, 7, -1) == 0

    response = build_response(
        0x37, 8, 0, 3, 160, 120, 2400, 867, 123456
    )
    payload, checksum_text = response.rstrip("\n").split("*")
    assert checksum_text == f"{crc8(payload.encode('ascii')):02X}"
    assert payload.startswith("V2,55,8,0,3,160,120,")
    targeted_response = build_response(
        0x40, 3, 0, 3, 168, 123, 2400, 800, 123500
    )
    targeted_payload, targeted_checksum = (
        targeted_response.rstrip("\n").split("*")
    )
    assert targeted_checksum == (
        f"{crc8(targeted_payload.encode('ascii')):02X}"
    )
    assert targeted_payload.startswith(
        "V2,64,3,0,3,168,123,2400,800,"
    )
    endpoint_response = build_response(
        0x35, 10, 0, 1, 161, 119, 42, 1000, 123956
    )
    endpoint_payload, endpoint_checksum = (
        endpoint_response.rstrip("\n").split("*")
    )
    assert endpoint_checksum == (
        f"{crc8(endpoint_payload.encode('ascii')):02X}"
    )
    assert endpoint_payload.startswith(
        "V2,53,10,0,1,161,119,42,1000,"
    )
    assert (
        select_center_ring(
            [
                FakeCircle(60, 80, 40),
                FakeCircle(160, 120, 40),
                FakeCircle(260, 160, 40),
            ]
        )
        is None
    )
    assert (
        select_center_ring(
            [
                FakeCircle(60, 120, 10),
                FakeCircle(160, 120, 30),
                FakeCircle(260, 120, 10),
            ]
        )
        is None
    )

    calibration = json.loads(
        CALIBRATION.read_text(encoding="utf-8")
    )
    assert calibration["schema_version"] == 1
    assert (
        calibration["vision_version"]
        == "5.3.0-find1,3"
    )
    assert calibration["protocol_version"] == 2
    assert calibration["ring_layout"]["require_three_rings"] is True
    assert (
        calibration["ring_layout"][
            "allow_centered_ring_fallback"
        ]
        is True
    )
    assert (
        calibration["ring_layout"][
            "candidate_minimum_radius_pixels"
        ]
        == 28
    )
    assert calibration["endpoint_scan"]["mode"] == 10
    assert calibration["endpoint_scan"]["target_id"] == 1
    assert (
        calibration["endpoint_scan"][
            "requires_three_ring_layout"
        ]
        is False
    )
    assert (
        calibration["endpoint_scan"]["stable_duration_ms"]
        == 120
    )
    assert (
        calibration["endpoint_scan"]["minimum_stable_samples"]
        == 2
    )
    assert (
        calibration["endpoint_scan"][
            "fast_accept_minimum_confidence"
        ]
        == 1000
    )
    assert (
        calibration["endpoint_scan"]["fast_accept_confirmations"]
        == 1
    )
    assert (
        calibration["endpoint_scan"][
            "maximum_center_span_pixels"
        ]
        == 2
    )

    print(
        "PASS Vision v5 targeted-color, ring-layout, endpoint-mode, "
        "protocol-v2 CRC and calibration schema"
    )
    if not calibration["verified"]:
        print(
            "NOTICE Vision calibration is intentionally marked UNVERIFIED; "
            "real-image commissioning is still required"
        )

if __name__ == "__main__":
    main()
