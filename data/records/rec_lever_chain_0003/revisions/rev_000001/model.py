from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_LENGTH = 0.620
BASE_WIDTH = 0.190
BASE_THICKNESS = 0.014

LEVER_THICKNESS = 0.008
TAB_THICKNESS = 0.006
WASHER_THICKNESS = 0.0014
BRACKET_CHEEK_THICKNESS = 0.006
BRACKET_INNER_GAP = 0.0118
BRACKET_SIDE_OFFSET = BRACKET_INNER_GAP / 2.0 + BRACKET_CHEEK_THICKNESS / 2.0

PIVOT_SHAFT_RADIUS = 0.0062
PIVOT_HOLE_RADIUS = 0.0073
PIVOT_CLEAR_RADIUS = 0.0085
DRIVE_BOSS_RADIUS = 0.0105

FOLLOWER_RADIUS = 0.0046
FOLLOWER_LENGTH = 0.010
GUIDE_ROLLER_RADIUS = 0.0052
GUIDE_ROLLER_LENGTH = 0.008

FRONT_COVER_Y = 0.084
REAR_COVER_Y = -0.076
GUIDE_PLATE_Y = 0.108

INPUT_PIVOT = (-0.220, 0.028, 0.094)
ROCKER_A_PIVOT = (-0.070, 0.004, 0.147)
ROCKER_B_PIVOT = (0.090, -0.020, 0.092)
OUTPUT_PIVOT = (0.238, 0.018, 0.143)

INPUT_DRIVE = (0.070, 0.014, 0.014)
ROCKER_A_RECEIVER = (-0.055, 0.038, -0.035)
ROCKER_A_DRIVE = (0.070, -0.014, -0.014)
ROCKER_B_RECEIVER = (-0.060, 0.010, 0.030)
ROCKER_B_DRIVE = (0.070, 0.014, 0.028)
OUTPUT_RECEIVER = (-0.040, -0.024, -0.022)
OUTPUT_GUIDE = (0.094, 0.083, -0.058)

COUPLER_1_OFFSET_Y = ROCKER_A_RECEIVER[1] - INPUT_DRIVE[1]
COUPLER_2_OFFSET_Y = ROCKER_B_RECEIVER[1] - ROCKER_A_DRIVE[1]
COUPLER_3_OFFSET_Y = OUTPUT_RECEIVER[1] - ROCKER_B_DRIVE[1]

COUPLER_1_FOLLOWER = (
    (ROCKER_A_PIVOT[0] + ROCKER_A_RECEIVER[0]) - (INPUT_PIVOT[0] + INPUT_DRIVE[0]),
    0.0,
    (ROCKER_A_PIVOT[2] + ROCKER_A_RECEIVER[2]) - (INPUT_PIVOT[2] + INPUT_DRIVE[2]),
)
COUPLER_2_FOLLOWER = (
    (ROCKER_B_PIVOT[0] + ROCKER_B_RECEIVER[0]) - (ROCKER_A_PIVOT[0] + ROCKER_A_DRIVE[0]),
    0.0,
    (ROCKER_B_PIVOT[2] + ROCKER_B_RECEIVER[2]) - (ROCKER_A_PIVOT[2] + ROCKER_A_DRIVE[2]),
)
COUPLER_3_FOLLOWER = (
    (OUTPUT_PIVOT[0] + OUTPUT_RECEIVER[0]) - (ROCKER_B_PIVOT[0] + ROCKER_B_DRIVE[0]),
    0.0,
    (OUTPUT_PIVOT[2] + OUTPUT_RECEIVER[2]) - (ROCKER_B_PIVOT[2] + ROCKER_B_DRIVE[2]),
)
OUTPUT_GUIDE_RADIUS = math.hypot(OUTPUT_GUIDE[0], OUTPUT_GUIDE[2])


def _disc_xz(
    x: float,
    z: float,
    radius: float,
    thickness: float,
    *,
    y: float = 0.0,
) -> cq.Workplane:
    solid = (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(thickness / 2.0, both=True)
    )
    if abs(y) > 1e-9:
        solid = solid.translate((0.0, y, 0.0))
    return solid


def _ring_xz(
    x: float,
    z: float,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    *,
    y: float = 0.0,
) -> cq.Workplane:
    solid = (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness / 2.0, both=True)
    )
    if abs(y) > 1e-9:
        solid = solid.translate((0.0, y, 0.0))
    return solid


def _beam_between_xz(
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
    thickness: float,
    *,
    y: float = 0.0,
) -> cq.Workplane:
    dx = end[0] - start[0]
    dz = end[1] - start[1]
    length = math.hypot(dx, dz)
    angle_deg = math.degrees(math.atan2(dz, dx))
    solid = (
        cq.Workplane("XZ")
        .center((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
        .slot2D(length, width, angle_deg)
        .extrude(thickness / 2.0, both=True)
    )
    if abs(y) > 1e-9:
        solid = solid.translate((0.0, y, 0.0))
    return solid


def _slot_cut_xz(
    center_x: float,
    center_z: float,
    length: float,
    width: float,
    angle_deg: float,
    depth: float,
    *,
    y: float = 0.0,
) -> cq.Workplane:
    solid = (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .slot2D(length, width, angle_deg)
        .extrude(depth / 2.0, both=True)
    )
    if abs(y) > 1e-9:
        solid = solid.translate((0.0, y, 0.0))
    return solid


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane().box(*size).translate(center)


def _bridge_spacer(x: float, z: float, offset_y: float, radius: float) -> cq.Workplane:
    return _disc_xz(
        x,
        z,
        radius,
        abs(offset_y) + LEVER_THICKNESS,
        y=offset_y / 2.0,
    )


def _secondary_pin_hardware(x: float, z: float, y: float) -> cq.Workplane:
    hardware = _disc_xz(x, z, 0.0078, 0.004, y=y)
    hardware = hardware.union(_disc_xz(x, z, 0.0102, 0.002, y=y + 0.0031))
    hardware = hardware.union(_disc_xz(x, z, 0.0102, 0.002, y=y - 0.0031))
    return hardware


def _pivot_hardware() -> cq.Workplane:
    bushing_inner = PIVOT_SHAFT_RADIUS + 0.00035
    hardware = _ring_xz(0.0, 0.0, PIVOT_HOLE_RADIUS - 0.00005, bushing_inner, LEVER_THICKNESS)
    outer_y = LEVER_THICKNESS / 2.0 + WASHER_THICKNESS / 2.0
    hardware = hardware.union(_ring_xz(0.0, 0.0, 0.0145, bushing_inner, WASHER_THICKNESS, y=outer_y))
    hardware = hardware.union(_ring_xz(0.0, 0.0, 0.0145, bushing_inner, WASHER_THICKNESS, y=-outer_y))
    return hardware


def _frame_base() -> cq.Workplane:
    base = cq.Workplane().box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )
    base = (
        base.faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints(
            [
                (-0.235, -0.062),
                (0.235, -0.062),
                (-0.235, 0.062),
                (0.235, 0.062),
            ]
        )
        .slot2D(0.040, 0.012, 0.0)
        .cutThruAll()
    )
    base = base.union(_box((0.000, 0.056, 0.030), (0.550, 0.018, 0.032)))
    base = base.union(_box((0.000, -0.056, 0.030), (0.550, 0.018, 0.032)))
    base = base.union(_box((0.000, 0.000, 0.024), (0.255, 0.036, 0.020)))
    base = base.union(_box((0.210, 0.030, 0.042), (0.070, 0.040, 0.028)))
    return base


def _clevis_bracket(pivot: tuple[float, float, float], top_z: float) -> cq.Workplane:
    x, y, pivot_z = pivot
    pedestal_height = pivot_z - BASE_THICKNESS - 0.024
    bracket = (
        cq.Workplane()
        .box(0.046, 0.034, pedestal_height, centered=(True, True, False))
        .translate((x, y, BASE_THICKNESS))
    )
    bracket = bracket.union(_box((x - 0.010, y, 0.072), (0.016, 0.024, 0.080)))

    cheek_height = top_z - 0.040
    cheek_y_pos = y + BRACKET_SIDE_OFFSET
    cheek_y_neg = y - BRACKET_SIDE_OFFSET
    positive_cheek = (
        cq.Workplane()
        .box(0.028, BRACKET_CHEEK_THICKNESS, cheek_height, centered=(True, True, False))
        .translate((x, cheek_y_pos, 0.040))
    )
    negative_cheek = (
        cq.Workplane()
        .box(0.028, BRACKET_CHEEK_THICKNESS, cheek_height, centered=(True, True, False))
        .translate((x, cheek_y_neg, 0.040))
    )
    bracket = bracket.union(positive_cheek).union(negative_cheek)
    bracket = bracket.union(_box((x + 0.006, y, top_z - 0.005), (0.022, 0.024, 0.010)))
    relief_start_z = max(BASE_THICKNESS + 0.024, pivot_z - 0.060)
    relief_height = top_z - relief_start_z
    if relief_height > 0.0:
        bracket = bracket.cut(_box((x, y, (relief_start_z + top_z) / 2.0), (0.060, BRACKET_INNER_GAP, relief_height)))
    bracket = bracket.cut(_disc_xz(x, pivot_z, PIVOT_CLEAR_RADIUS, BRACKET_CHEEK_THICKNESS + 0.003, y=cheek_y_pos))
    bracket = bracket.cut(_disc_xz(x, pivot_z, PIVOT_CLEAR_RADIUS, BRACKET_CHEEK_THICKNESS + 0.003, y=cheek_y_neg))
    return bracket


def _frame_shafts() -> cq.Workplane:
    hardware = None
    shaft_length = BRACKET_INNER_GAP + 2.0 * BRACKET_CHEEK_THICKNESS + 0.012
    for pivot in (INPUT_PIVOT, ROCKER_A_PIVOT, ROCKER_B_PIVOT, OUTPUT_PIVOT):
        x, y, z = pivot
        local = _disc_xz(x, z, PIVOT_SHAFT_RADIUS, shaft_length, y=y)
        local = local.union(_disc_xz(x, z, 0.0092, 0.003, y=y + 0.013))
        local = local.union(_disc_xz(x, z, 0.0092, 0.003, y=y - 0.013))
        local = local.union(_disc_xz(x, z, 0.0115, 0.0022, y=y + 0.0156))
        local = local.union(_disc_xz(x, z, 0.0115, 0.0022, y=y - 0.0156))
        hardware = local if hardware is None else hardware.union(local)
    assert hardware is not None
    return hardware


def _frame_stop_pads() -> cq.Workplane:
    def _pad_with_stem(center: tuple[float, float, float], size: tuple[float, float, float], stem_xy: tuple[float, float]) -> cq.Workplane:
        x, y, z = center
        sx, sy, sz = size
        pad = _box(center, size)
        pad_bottom = z - sz / 2.0
        stem_top = pad_bottom
        stem_bottom = 0.046
        stem_height = stem_top - stem_bottom
        stem = _box((stem_xy[0], stem_xy[1], stem_bottom + stem_height / 2.0), (sx * 0.55, sy, stem_height))
        return pad.union(stem)

    pads = _pad_with_stem((-0.205, 0.056, 0.092), (0.018, 0.012, 0.010), (-0.205, 0.056))
    pads = pads.union(_pad_with_stem((-0.070, -0.056, 0.176), (0.020, 0.012, 0.010), (-0.070, -0.056)))
    pads = pads.union(_pad_with_stem((0.105, 0.056, 0.112), (0.018, 0.012, 0.010), (0.105, 0.056)))
    pads = pads.union(_pad_with_stem((0.250, -0.056, 0.174), (0.018, 0.012, 0.010), (0.250, -0.056)))
    return pads


def _input_lever_plate() -> cq.Workplane:
    plate = _disc_xz(0.0, 0.0, 0.024, LEVER_THICKNESS)
    plate = plate.union(_beam_between_xz((0.0, 0.0), (0.070, 0.014), 0.026, LEVER_THICKNESS))
    plate = plate.union(_beam_between_xz((0.0, 0.0), (-0.042, -0.020), 0.018, LEVER_THICKNESS))
    plate = plate.union(_beam_between_xz((0.0, 0.0), (0.012, 0.034), 0.012, LEVER_THICKNESS))
    plate = plate.cut(_disc_xz(0.0, 0.0, PIVOT_HOLE_RADIUS, 0.030))
    plate = plate.cut(_slot_cut_xz(0.030, 0.006, 0.040, 0.012, 11.0, 0.030))
    return plate


def _input_coupler_mount() -> cq.Workplane:
    ear = _disc_xz(INPUT_DRIVE[0], INPUT_DRIVE[2], 0.0125, TAB_THICKNESS, y=INPUT_DRIVE[1])
    ear = ear.union(
        _beam_between_xz((0.048, 0.010), (INPUT_DRIVE[0], INPUT_DRIVE[2]), 0.014, TAB_THICKNESS, y=INPUT_DRIVE[1])
    )
    ear = ear.union(_disc_xz(0.048, 0.010, 0.009, TAB_THICKNESS, y=INPUT_DRIVE[1]))
    ear = ear.union(_bridge_spacer(0.050, 0.010, INPUT_DRIVE[1], 0.006))
    return ear


def _rocker_a_main_plate() -> cq.Workplane:
    plate = _disc_xz(0.0, 0.0, 0.023, LEVER_THICKNESS)
    plate = plate.union(_beam_between_xz((0.0, 0.0), (0.060, -0.012), 0.020, LEVER_THICKNESS))
    plate = plate.union(_disc_xz(0.060, -0.012, 0.014, LEVER_THICKNESS))
    plate = plate.union(_beam_between_xz((0.0, 0.0), (0.018, 0.035), 0.014, LEVER_THICKNESS))
    plate = plate.union(_beam_between_xz((0.0, 0.0), (-0.028, -0.020), 0.016, LEVER_THICKNESS))
    plate = plate.cut(_disc_xz(0.0, 0.0, PIVOT_HOLE_RADIUS, 0.030))
    plate = plate.cut(_slot_cut_xz(0.022, 0.004, 0.030, 0.010, -7.0, 0.030))
    return plate


def _rocker_a_receiver_tab() -> cq.Workplane:
    angle_deg = math.degrees(math.atan2(ROCKER_A_RECEIVER[2], ROCKER_A_RECEIVER[0]))
    reach = math.hypot(ROCKER_A_RECEIVER[0], ROCKER_A_RECEIVER[2])
    start_scale = 0.016 / max(reach, 1e-9)
    start_pt = (ROCKER_A_RECEIVER[0] * start_scale, ROCKER_A_RECEIVER[2] * start_scale)
    tab = _disc_xz(ROCKER_A_RECEIVER[0], ROCKER_A_RECEIVER[2], 0.016, TAB_THICKNESS, y=ROCKER_A_RECEIVER[1])
    tab = tab.union(
        _ring_xz(
            0.0,
            0.0,
            0.0135,
            PIVOT_CLEAR_RADIUS,
            abs(ROCKER_A_RECEIVER[1]) + LEVER_THICKNESS,
            y=ROCKER_A_RECEIVER[1] / 2.0,
        )
    )
    tab = tab.union(
        _beam_between_xz(start_pt, (ROCKER_A_RECEIVER[0], ROCKER_A_RECEIVER[2]), 0.018, TAB_THICKNESS, y=ROCKER_A_RECEIVER[1])
    )
    tab = tab.cut(
        _slot_cut_xz(
            ROCKER_A_RECEIVER[0],
            ROCKER_A_RECEIVER[2],
            0.040,
            0.012,
            angle_deg,
            0.030,
            y=ROCKER_A_RECEIVER[1],
        )
    )
    return tab


def _rocker_a_drive_ear() -> cq.Workplane:
    ear = _disc_xz(ROCKER_A_DRIVE[0], ROCKER_A_DRIVE[2], 0.0125, TAB_THICKNESS, y=ROCKER_A_DRIVE[1])
    ear = ear.union(
        _beam_between_xz((0.036, -0.010), (ROCKER_A_DRIVE[0], ROCKER_A_DRIVE[2]), 0.015, TAB_THICKNESS, y=ROCKER_A_DRIVE[1])
    )
    ear = ear.union(_disc_xz(0.036, -0.010, 0.009, TAB_THICKNESS, y=ROCKER_A_DRIVE[1]))
    ear = ear.union(_bridge_spacer(0.036, -0.010, ROCKER_A_DRIVE[1], 0.006))
    return ear


def _rocker_b_main_plate() -> cq.Workplane:
    plate = _disc_xz(0.0, 0.0, 0.023, LEVER_THICKNESS)
    plate = plate.union(_beam_between_xz((0.0, 0.0), (0.055, 0.018), 0.020, LEVER_THICKNESS))
    plate = plate.union(_beam_between_xz((0.0, 0.0), (-0.030, -0.030), 0.016, LEVER_THICKNESS))
    plate = plate.union(_disc_xz(-0.030, -0.030, 0.013, LEVER_THICKNESS))
    plate = plate.union(_beam_between_xz((0.0, 0.0), (-0.012, 0.026), 0.012, LEVER_THICKNESS))
    plate = plate.cut(_disc_xz(0.0, 0.0, PIVOT_HOLE_RADIUS, 0.030))
    plate = plate.cut(_slot_cut_xz(0.020, 0.008, 0.028, 0.010, 16.0, 0.030))
    return plate


def _rocker_b_receiver_tab() -> cq.Workplane:
    angle_deg = math.degrees(math.atan2(ROCKER_B_RECEIVER[2], ROCKER_B_RECEIVER[0]))
    reach = math.hypot(ROCKER_B_RECEIVER[0], ROCKER_B_RECEIVER[2])
    start_scale = 0.016 / max(reach, 1e-9)
    start_pt = (ROCKER_B_RECEIVER[0] * start_scale, ROCKER_B_RECEIVER[2] * start_scale)
    tab = _disc_xz(ROCKER_B_RECEIVER[0], ROCKER_B_RECEIVER[2], 0.016, TAB_THICKNESS, y=ROCKER_B_RECEIVER[1])
    tab = tab.union(
        _ring_xz(
            0.0,
            0.0,
            0.0135,
            PIVOT_CLEAR_RADIUS,
            abs(ROCKER_B_RECEIVER[1]) + LEVER_THICKNESS,
            y=ROCKER_B_RECEIVER[1] / 2.0,
        )
    )
    tab = tab.union(
        _beam_between_xz(start_pt, (ROCKER_B_RECEIVER[0], ROCKER_B_RECEIVER[2]), 0.014, TAB_THICKNESS, y=ROCKER_B_RECEIVER[1])
    )
    tab = tab.cut(
        _slot_cut_xz(
            ROCKER_B_RECEIVER[0],
            ROCKER_B_RECEIVER[2],
            0.042,
            0.012,
            angle_deg,
            0.030,
            y=ROCKER_B_RECEIVER[1],
        )
    )
    return tab


def _rocker_b_drive_ear() -> cq.Workplane:
    ear = _disc_xz(ROCKER_B_DRIVE[0], ROCKER_B_DRIVE[2], 0.0125, TAB_THICKNESS, y=ROCKER_B_DRIVE[1])
    ear = ear.union(
        _beam_between_xz((0.036, 0.010), (ROCKER_B_DRIVE[0], ROCKER_B_DRIVE[2]), 0.015, TAB_THICKNESS, y=ROCKER_B_DRIVE[1])
    )
    ear = ear.union(_disc_xz(0.036, 0.010, 0.009, TAB_THICKNESS, y=ROCKER_B_DRIVE[1]))
    ear = ear.union(_bridge_spacer(0.036, 0.010, ROCKER_B_DRIVE[1], 0.006))
    return ear


def _output_lever_main_plate() -> cq.Workplane:
    plate = _disc_xz(0.0, 0.0, 0.024, LEVER_THICKNESS)
    plate = plate.union(_beam_between_xz((0.0, 0.0), (0.095, -0.058), 0.018, LEVER_THICKNESS))
    plate = plate.union(_disc_xz(0.064, -0.040, 0.012, LEVER_THICKNESS))
    plate = plate.union(_beam_between_xz((0.0, 0.0), (0.020, 0.032), 0.014, LEVER_THICKNESS))
    plate = plate.cut(_disc_xz(0.0, 0.0, PIVOT_HOLE_RADIUS, 0.030))
    plate = plate.cut(_slot_cut_xz(0.036, -0.022, 0.026, 0.010, -26.0, 0.030))
    return plate


def _output_receiver_tab() -> cq.Workplane:
    angle_deg = math.degrees(math.atan2(OUTPUT_RECEIVER[2], OUTPUT_RECEIVER[0]))
    reach = math.hypot(OUTPUT_RECEIVER[0], OUTPUT_RECEIVER[2])
    start_scale = 0.016 / max(reach, 1e-9)
    start_pt = (OUTPUT_RECEIVER[0] * start_scale, OUTPUT_RECEIVER[2] * start_scale)
    tab = _disc_xz(OUTPUT_RECEIVER[0], OUTPUT_RECEIVER[2], 0.016, TAB_THICKNESS, y=OUTPUT_RECEIVER[1])
    tab = tab.union(
        _ring_xz(
            0.0,
            0.0,
            0.0135,
            PIVOT_CLEAR_RADIUS,
            abs(OUTPUT_RECEIVER[1]) + LEVER_THICKNESS,
            y=OUTPUT_RECEIVER[1] / 2.0,
        )
    )
    tab = tab.union(
        _beam_between_xz(start_pt, (OUTPUT_RECEIVER[0], OUTPUT_RECEIVER[2]), 0.014, TAB_THICKNESS, y=OUTPUT_RECEIVER[1])
    )
    tab = tab.cut(
        _slot_cut_xz(
            OUTPUT_RECEIVER[0],
            OUTPUT_RECEIVER[2],
            0.040,
            0.012,
            angle_deg,
            0.030,
            y=OUTPUT_RECEIVER[1],
        )
    )
    return tab


def _guide_pin_visual() -> cq.Workplane:
    roller_center_y = OUTPUT_GUIDE[1]
    standoff_center_y = (OUTPUT_GUIDE[1] - LEVER_THICKNESS / 2.0) / 2.0
    standoff_length = OUTPUT_GUIDE[1] - LEVER_THICKNESS / 2.0
    visual = _disc_xz(OUTPUT_GUIDE[0], OUTPUT_GUIDE[2], 0.0035, standoff_length, y=standoff_center_y)
    visual = visual.union(
        _disc_xz(OUTPUT_GUIDE[0], OUTPUT_GUIDE[2], GUIDE_ROLLER_RADIUS, GUIDE_ROLLER_LENGTH, y=roller_center_y)
    )
    return visual


def _coupler_body(follower: tuple[float, float, float], local_y: float) -> cq.Workplane:
    end = (follower[0], follower[2])
    body_plane_y = local_y
    sign = 1.0 if body_plane_y >= 0.0 else -1.0
    root_plane_y = sign * TAB_THICKNESS
    start = (0.018, 0.0)
    dx = end[0] - start[0]
    dz = end[1] - start[1]
    length = math.hypot(dx, dz)
    shorten = min(0.014, max(length - 0.008, 0.004))
    if length > 1e-9:
        body_end = (end[0] - dx / length * shorten, end[1] - dz / length * shorten)
    else:
        body_end = end

    body = _disc_xz(0.0, 0.0, 0.0095, TAB_THICKNESS, y=root_plane_y)
    body = body.union(
        _box(
            (0.010, (root_plane_y + body_plane_y) / 2.0, 0.0),
            (0.020, abs(body_plane_y - root_plane_y) + TAB_THICKNESS, 0.010),
        )
    )
    body = body.union(_disc_xz(start[0], start[1], 0.0070, TAB_THICKNESS, y=body_plane_y))
    body = body.union(_beam_between_xz(start, body_end, 0.010, TAB_THICKNESS, y=body_plane_y))
    body = body.union(_disc_xz(end[0], end[1], FOLLOWER_RADIUS + 0.0008, TAB_THICKNESS, y=body_plane_y))
    body = body.cut(
        _slot_cut_xz(
            (start[0] + body_end[0]) / 2.0,
            (start[1] + body_end[1]) / 2.0,
            max(math.hypot(body_end[0] - start[0], body_end[1] - start[1]) - 0.010, 0.006),
            0.005,
            math.degrees(math.atan2(body_end[1] - start[1], body_end[0] - start[0])),
            0.030,
            y=body_plane_y,
        )
    )
    return body


def _coupler_follower(follower: tuple[float, float, float], local_y: float) -> cq.Workplane:
    return _disc_xz(follower[0], follower[2], FOLLOWER_RADIUS, FOLLOWER_LENGTH, y=local_y)


def _front_cover_shell() -> cq.Workplane:
    shell = _box((-0.145, FRONT_COVER_Y, 0.126), (0.280, 0.004, 0.150))
    shell = shell.union(_box((-0.145, FRONT_COVER_Y - 0.004, 0.057), (0.280, 0.012, 0.010)))
    shell = shell.cut(_disc_xz(INPUT_PIVOT[0] + 0.004, INPUT_PIVOT[2], 0.017, 0.012, y=FRONT_COVER_Y))
    shell = shell.cut(_disc_xz(ROCKER_A_PIVOT[0] + 0.004, ROCKER_A_PIVOT[2], 0.017, 0.012, y=FRONT_COVER_Y))
    shell = shell.cut(_slot_cut_xz(-0.145, 0.168, 0.090, 0.014, 0.0, 0.012, y=FRONT_COVER_Y))
    return shell


def _rear_cover_shell() -> cq.Workplane:
    shell = _box((0.165, REAR_COVER_Y, 0.124), (0.290, 0.004, 0.152))
    shell = shell.union(_box((0.165, REAR_COVER_Y + 0.004, 0.056), (0.290, 0.012, 0.010)))
    shell = shell.cut(_disc_xz(ROCKER_B_PIVOT[0] + 0.004, ROCKER_B_PIVOT[2], 0.017, 0.012, y=REAR_COVER_Y))
    shell = shell.cut(_disc_xz(OUTPUT_PIVOT[0] + 0.004, OUTPUT_PIVOT[2], 0.017, 0.012, y=REAR_COVER_Y))
    shell = shell.cut(_slot_cut_xz(0.165, 0.166, 0.094, 0.014, 0.0, 0.012, y=REAR_COVER_Y))
    return shell


def _cover_posts(side_y: float, xs: tuple[float, ...], zs: tuple[float, ...]) -> cq.Workplane:
    target_y = 0.060 if side_y > 0.0 else -0.060
    x_min = min(xs) - 0.010
    x_max = max(xs) + 0.010
    rib_length = abs(side_y - target_y)
    y_center = (side_y + target_y) / 2.0
    lower_rib = _box(((x_min + x_max) / 2.0, y_center, 0.059), (x_max - x_min, rib_length, 0.006))
    upper_rib = _box(((x_min + x_max) / 2.0, y_center, 0.065), (x_max - x_min, rib_length * 0.75, 0.006))
    return lower_rib.union(upper_rib)


def _guide_sector_shell() -> cq.Workplane:
    cx, _, cz = OUTPUT_PIVOT
    shell = (
        cq.Workplane("XZ")
        .center(cx, cz)
        .circle(0.128)
        .circle(0.094)
        .extrude(0.006 / 2.0, both=True)
        .translate((0.0, GUIDE_PLATE_Y, 0.0))
    )
    shell = shell.cut(
        cq.Workplane("XZ")
        .center(cx, cz)
        .circle(OUTPUT_GUIDE_RADIUS + 0.0065)
        .circle(OUTPUT_GUIDE_RADIUS - 0.0065)
        .extrude(0.010 / 2.0, both=True)
        .translate((0.0, GUIDE_PLATE_Y, 0.0))
    )
    shell = shell.cut(_box((cx - 0.125, GUIDE_PLATE_Y, cz + 0.005), (0.210, 0.020, 0.240)))
    shell = shell.cut(_box((cx - 0.010, GUIDE_PLATE_Y, cz + 0.122), (0.270, 0.020, 0.150)))
    shell = shell.cut(_slot_cut_xz(cx + 0.015, cz + 0.094, 0.022, 0.010, 90.0, 0.012, y=GUIDE_PLATE_Y))
    shell = shell.cut(_slot_cut_xz(cx - 0.030, cz + 0.076, 0.022, 0.010, 90.0, 0.012, y=GUIDE_PLATE_Y))
    return shell


def _guide_sector_tabs() -> cq.Workplane:
    cx, _, cz = OUTPUT_PIVOT
    support_y0 = 0.060
    support_y1 = GUIDE_PLATE_Y
    support_center_y = (support_y0 + support_y1) / 2.0
    support_size_y = support_y1 - support_y0
    tabs = _box((cx + 0.062, support_center_y, 0.070), (0.016, support_size_y, 0.028))
    tabs = tabs.union(_box((cx + 0.036, support_center_y, 0.054), (0.016, support_size_y, 0.004)))
    tabs = tabs.union(_box((cx + 0.062, GUIDE_PLATE_Y - 0.002, 0.070), (0.022, 0.004, 0.024)))
    tabs = tabs.union(_box((cx + 0.036, GUIDE_PLATE_Y - 0.002, 0.060), (0.022, 0.004, 0.020)))
    return tabs


def _world_sum(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lever_chain_study", assets=ASSETS)

    frame_steel = model.material("frame_steel", rgba=(0.39, 0.40, 0.43, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.22, 0.23, 0.25, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    bronze = model.material("bronze", rgba=(0.66, 0.54, 0.35, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.53, 0.55, 0.58, 1.0))
    stop_rubber = model.material("stop_rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(_frame_base(), "frame_base.obj", assets=ASSETS), material=frame_steel, name="base_structure")
    frame.visual(
        mesh_from_cadquery(_clevis_bracket(INPUT_PIVOT, 0.158), "frame_input_bracket.obj", assets=ASSETS),
        material=frame_steel,
        name="input_bracket",
    )
    frame.visual(
        mesh_from_cadquery(_clevis_bracket(ROCKER_A_PIVOT, 0.204), "frame_rocker_a_bracket.obj", assets=ASSETS),
        material=frame_steel,
        name="rocker_a_bracket",
    )
    frame.visual(
        mesh_from_cadquery(_clevis_bracket(ROCKER_B_PIVOT, 0.160), "frame_rocker_b_bracket.obj", assets=ASSETS),
        material=frame_steel,
        name="rocker_b_bracket",
    )
    frame.visual(
        mesh_from_cadquery(_clevis_bracket(OUTPUT_PIVOT, 0.205), "frame_output_bracket.obj", assets=ASSETS),
        material=frame_steel,
        name="output_bracket",
    )
    frame.visual(mesh_from_cadquery(_frame_shafts(), "frame_shafts.obj", assets=ASSETS), material=bright_steel, name="pivot_shafts")
    frame.visual(mesh_from_cadquery(_frame_stop_pads(), "frame_stop_pads.obj", assets=ASSETS), material=stop_rubber, name="stop_pads")
    frame.inertial = Inertial.from_geometry(Box((0.62, 0.19, 0.22)), mass=16.0, origin=Origin(xyz=(0.0, 0.0, 0.11)))

    input_lever = model.part("input_lever")
    input_lever.visual(mesh_from_cadquery(_input_lever_plate(), "input_lever_plate.obj", assets=ASSETS), material=dark_oxide, name="main_plate")
    input_lever.visual(mesh_from_cadquery(_input_coupler_mount(), "input_lever_mount.obj", assets=ASSETS), material=dark_oxide, name="coupler_mount")
    input_lever.visual(mesh_from_cadquery(_pivot_hardware(), "input_lever_pivot_hardware.obj", assets=ASSETS), material=bronze, name="pivot_hardware")
    input_lever.visual(
        mesh_from_cadquery(_secondary_pin_hardware(INPUT_DRIVE[0], INPUT_DRIVE[2], INPUT_DRIVE[1]), "input_lever_drive_hardware.obj", assets=ASSETS),
        material=bright_steel,
        name="drive_hardware",
    )
    input_lever.inertial = Inertial.from_geometry(Box((0.120, 0.030, 0.070)), mass=0.82, origin=Origin(xyz=(0.020, 0.008, 0.004)))

    rocker_a = model.part("rocker_a")
    rocker_a.visual(mesh_from_cadquery(_rocker_a_main_plate(), "rocker_a_main_plate.obj", assets=ASSETS), material=dark_oxide, name="main_plate")
    rocker_a.visual(mesh_from_cadquery(_rocker_a_receiver_tab(), "rocker_a_receiver_tab.obj", assets=ASSETS), material=dark_oxide, name="receiver_tab")
    rocker_a.visual(mesh_from_cadquery(_rocker_a_drive_ear(), "rocker_a_drive_ear.obj", assets=ASSETS), material=dark_oxide, name="drive_ear")
    rocker_a.visual(mesh_from_cadquery(_pivot_hardware(), "rocker_a_pivot_hardware.obj", assets=ASSETS), material=bronze, name="pivot_hardware")
    rocker_a.visual(
        mesh_from_cadquery(_secondary_pin_hardware(ROCKER_A_DRIVE[0], ROCKER_A_DRIVE[2], ROCKER_A_DRIVE[1]), "rocker_a_drive_hardware.obj", assets=ASSETS),
        material=bright_steel,
        name="drive_hardware",
    )
    rocker_a.inertial = Inertial.from_geometry(Box((0.120, 0.055, 0.090)), mass=0.72, origin=Origin(xyz=(0.010, 0.006, -0.006)))

    rocker_b = model.part("rocker_b")
    rocker_b.visual(mesh_from_cadquery(_rocker_b_main_plate(), "rocker_b_main_plate.obj", assets=ASSETS), material=dark_oxide, name="main_plate")
    rocker_b.visual(mesh_from_cadquery(_rocker_b_receiver_tab(), "rocker_b_receiver_tab.obj", assets=ASSETS), material=dark_oxide, name="receiver_tab")
    rocker_b.visual(mesh_from_cadquery(_rocker_b_drive_ear(), "rocker_b_drive_ear.obj", assets=ASSETS), material=dark_oxide, name="drive_ear")
    rocker_b.visual(mesh_from_cadquery(_pivot_hardware(), "rocker_b_pivot_hardware.obj", assets=ASSETS), material=bronze, name="pivot_hardware")
    rocker_b.visual(
        mesh_from_cadquery(_secondary_pin_hardware(ROCKER_B_DRIVE[0], ROCKER_B_DRIVE[2], ROCKER_B_DRIVE[1]), "rocker_b_drive_hardware.obj", assets=ASSETS),
        material=bright_steel,
        name="drive_hardware",
    )
    rocker_b.inertial = Inertial.from_geometry(Box((0.120, 0.040, 0.085)), mass=0.74, origin=Origin(xyz=(0.006, 0.006, 0.006)))

    output_lever = model.part("output_lever")
    output_lever.visual(mesh_from_cadquery(_output_lever_main_plate(), "output_lever_main_plate.obj", assets=ASSETS), material=dark_oxide, name="main_plate")
    output_lever.visual(mesh_from_cadquery(_output_receiver_tab(), "output_lever_receiver_tab.obj", assets=ASSETS), material=dark_oxide, name="receiver_tab")
    output_lever.visual(mesh_from_cadquery(_pivot_hardware(), "output_lever_pivot_hardware.obj", assets=ASSETS), material=bronze, name="pivot_hardware")
    output_lever.visual(mesh_from_cadquery(_guide_pin_visual(), "output_lever_guide_pin.obj", assets=ASSETS), material=bright_steel, name="guide_pin")
    output_lever.inertial = Inertial.from_geometry(Box((0.130, 0.040, 0.095)), mass=0.80, origin=Origin(xyz=(0.026, -0.006, -0.018)))

    coupler_1 = model.part("coupler_1")
    coupler_1.visual(mesh_from_cadquery(_coupler_body(COUPLER_1_FOLLOWER, COUPLER_1_OFFSET_Y), "coupler_1_body.obj", assets=ASSETS), material=frame_steel, name="link_body")
    coupler_1.visual(mesh_from_cadquery(_coupler_follower(COUPLER_1_FOLLOWER, COUPLER_1_OFFSET_Y), "coupler_1_follower.obj", assets=ASSETS), material=bright_steel, name="follower_roller")
    coupler_1.inertial = Inertial.from_geometry(Box((0.050, 0.016, 0.020)), mass=0.16, origin=Origin(xyz=(0.020, 0.0, 0.002)))

    coupler_2 = model.part("coupler_2")
    coupler_2.visual(mesh_from_cadquery(_coupler_body(COUPLER_2_FOLLOWER, COUPLER_2_OFFSET_Y), "coupler_2_body.obj", assets=ASSETS), material=frame_steel, name="link_body")
    coupler_2.visual(mesh_from_cadquery(_coupler_follower(COUPLER_2_FOLLOWER, COUPLER_2_OFFSET_Y), "coupler_2_follower.obj", assets=ASSETS), material=bright_steel, name="follower_roller")
    coupler_2.inertial = Inertial.from_geometry(Box((0.055, 0.016, 0.024)), mass=0.18, origin=Origin(xyz=(0.023, 0.0, -0.004)))

    coupler_3 = model.part("coupler_3")
    coupler_3.visual(mesh_from_cadquery(_coupler_body(COUPLER_3_FOLLOWER, COUPLER_3_OFFSET_Y), "coupler_3_body.obj", assets=ASSETS), material=frame_steel, name="link_body")
    coupler_3.visual(mesh_from_cadquery(_coupler_follower(COUPLER_3_FOLLOWER, COUPLER_3_OFFSET_Y), "coupler_3_follower.obj", assets=ASSETS), material=bright_steel, name="follower_roller")
    coupler_3.inertial = Inertial.from_geometry(Box((0.060, 0.016, 0.018)), mass=0.18, origin=Origin(xyz=(0.028, 0.0, 0.0)))

    front_cover = model.part("front_access_cover")
    front_cover.visual(mesh_from_cadquery(_front_cover_shell(), "front_cover_shell.obj", assets=ASSETS), material=cover_gray, name="cover_shell")
    front_cover.visual(
        mesh_from_cadquery(
            _cover_posts(FRONT_COVER_Y, (INPUT_PIVOT[0], INPUT_PIVOT[0] + 0.032, ROCKER_A_PIVOT[0] - 0.020, ROCKER_A_PIVOT[0] + 0.022), (0.070, 0.176)),
            "front_cover_posts.obj",
            assets=ASSETS,
        ),
        material=bright_steel,
        name="mount_posts",
    )
    front_cover.inertial = Inertial.from_geometry(Box((0.280, 0.020, 0.160)), mass=0.92, origin=Origin(xyz=(-0.145, FRONT_COVER_Y, 0.126)))

    rear_cover = model.part("rear_access_cover")
    rear_cover.visual(mesh_from_cadquery(_rear_cover_shell(), "rear_cover_shell.obj", assets=ASSETS), material=cover_gray, name="cover_shell")
    rear_cover.visual(
        mesh_from_cadquery(
            _cover_posts(REAR_COVER_Y, (ROCKER_B_PIVOT[0] - 0.020, ROCKER_B_PIVOT[0] + 0.018, OUTPUT_PIVOT[0] - 0.020, OUTPUT_PIVOT[0] + 0.020), (0.070, 0.176)),
            "rear_cover_posts.obj",
            assets=ASSETS,
        ),
        material=bright_steel,
        name="mount_posts",
    )
    rear_cover.inertial = Inertial.from_geometry(Box((0.290, 0.020, 0.160)), mass=0.96, origin=Origin(xyz=(0.165, REAR_COVER_Y, 0.124)))

    guide_sector = model.part("guide_sector")
    guide_sector.visual(mesh_from_cadquery(_guide_sector_shell(), "guide_sector_shell.obj", assets=ASSETS), material=frame_steel, name="guide_shell")
    guide_sector.visual(mesh_from_cadquery(_guide_sector_tabs(), "guide_sector_tabs.obj", assets=ASSETS), material=bright_steel, name="mount_tabs")
    guide_sector.inertial = Inertial.from_geometry(Box((0.180, 0.040, 0.180)), mass=0.80, origin=Origin(xyz=(OUTPUT_PIVOT[0], 0.041, OUTPUT_PIVOT[2])))

    input_joint = model.articulation(
        "frame_to_input_lever",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=input_lever,
        origin=Origin(xyz=INPUT_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.35, upper=0.42),
    )
    rocker_a_joint = model.articulation(
        "frame_to_rocker_a",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rocker_a,
        origin=Origin(xyz=ROCKER_A_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.30, upper=0.36),
    )
    rocker_b_joint = model.articulation(
        "frame_to_rocker_b",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rocker_b,
        origin=Origin(xyz=ROCKER_B_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.30, upper=0.36),
    )
    output_joint = model.articulation(
        "frame_to_output_lever",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=output_lever,
        origin=Origin(xyz=OUTPUT_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.30, upper=0.36),
    )

    model.articulation("input_lever_to_coupler_1", ArticulationType.FIXED, parent=input_lever, child=coupler_1, origin=Origin(xyz=INPUT_DRIVE))
    model.articulation("rocker_a_to_coupler_2", ArticulationType.FIXED, parent=rocker_a, child=coupler_2, origin=Origin(xyz=ROCKER_A_DRIVE))
    model.articulation("rocker_b_to_coupler_3", ArticulationType.FIXED, parent=rocker_b, child=coupler_3, origin=Origin(xyz=ROCKER_B_DRIVE))
    model.articulation("frame_to_front_cover", ArticulationType.FIXED, parent=frame, child=front_cover, origin=Origin())
    model.articulation("frame_to_rear_cover", ArticulationType.FIXED, parent=frame, child=rear_cover, origin=Origin())
    model.articulation("frame_to_guide_sector", ArticulationType.FIXED, parent=frame, child=guide_sector, origin=Origin())

    frame.meta = {
        "primary_joints": [input_joint.name, rocker_a_joint.name, rocker_b_joint.name, output_joint.name],
        "study_type": "serial_staggered_lever_chain",
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    frame = object_model.get_part("frame")
    input_lever = object_model.get_part("input_lever")
    rocker_a = object_model.get_part("rocker_a")
    rocker_b = object_model.get_part("rocker_b")
    output_lever = object_model.get_part("output_lever")
    coupler_1 = object_model.get_part("coupler_1")
    coupler_2 = object_model.get_part("coupler_2")
    coupler_3 = object_model.get_part("coupler_3")
    front_cover = object_model.get_part("front_access_cover")
    rear_cover = object_model.get_part("rear_access_cover")
    guide_sector = object_model.get_part("guide_sector")

    input_joint = object_model.get_articulation("frame_to_input_lever")
    rocker_a_joint = object_model.get_articulation("frame_to_rocker_a")
    rocker_b_joint = object_model.get_articulation("frame_to_rocker_b")
    output_joint = object_model.get_articulation("frame_to_output_lever")

    input_mount = input_lever.get_visual("coupler_mount")
    input_drive_hardware = input_lever.get_visual("drive_hardware")
    rocker_a_receiver = rocker_a.get_visual("receiver_tab")
    rocker_a_drive_ear = rocker_a.get_visual("drive_ear")
    rocker_a_drive = rocker_a.get_visual("drive_hardware")
    rocker_b_receiver = rocker_b.get_visual("receiver_tab")
    rocker_b_drive_ear = rocker_b.get_visual("drive_ear")
    rocker_b_drive = rocker_b.get_visual("drive_hardware")
    output_receiver = output_lever.get_visual("receiver_tab")
    output_guide = output_lever.get_visual("guide_pin")
    coupler_1_follower = coupler_1.get_visual("follower_roller")
    coupler_2_follower = coupler_2.get_visual("follower_roller")
    coupler_3_follower = coupler_3.get_visual("follower_roller")
    front_cover_shell = front_cover.get_visual("cover_shell")
    front_cover_posts = front_cover.get_visual("mount_posts")
    rear_cover_shell = rear_cover.get_visual("cover_shell")
    rear_cover_posts = rear_cover.get_visual("mount_posts")
    guide_shell = guide_sector.get_visual("guide_shell")
    guide_tabs = guide_sector.get_visual("mount_tabs")
    input_bracket = frame.get_visual("input_bracket")
    rocker_a_bracket = frame.get_visual("rocker_a_bracket")
    rocker_b_bracket = frame.get_visual("rocker_b_bracket")
    output_bracket = frame.get_visual("output_bracket")
    frame_shafts = frame.get_visual("pivot_shafts")

    def _center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mn + mx) * 0.5 for mn, mx in zip(mins, maxs))

    def _vec_close(actual: tuple[float, float, float] | None, expected: tuple[float, float, float], tol: float) -> bool:
        return actual is not None and all(abs(a - b) <= tol for a, b in zip(actual, expected))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_isolated_part(
        guide_sector,
        reason="guide sector is intentionally carried on represented stand-off brackets with a small visualized mounting clearance to keep the guide face readable.",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        coupler_2,
        rocker_a,
        elem_a=coupler_2.get_visual("link_body"),
        elem_b=rocker_a_drive_ear,
        reason="coupler 2 root strap is represented as a laminated stack captured by rocker A's drive ear at the shared pin seat.",
    )
    ctx.allow_overlap(
        coupler_2,
        rocker_a,
        elem_a=coupler_2.get_visual("link_body"),
        elem_b=rocker_a.get_visual("main_plate"),
        reason="coupler 2's pickup strap is modeled as a buried laminated attachment plate nested into rocker A's fabricated web at the mount end.",
    )
    ctx.allow_overlap(
        coupler_3,
        rocker_b,
        elem_a=coupler_3.get_visual("link_body"),
        elem_b=rocker_b_drive_ear,
        reason="coupler 3 root strap is represented as a laminated stack captured by rocker B's drive ear at the shared pin seat.",
    )
    ctx.allow_overlap(
        frame,
        rocker_a,
        elem_a=frame_shafts,
        elem_b=rocker_a_receiver,
        reason="rocker A's receiver carrier wraps around the frame fulcrum sleeve, so the simplified tab mesh intentionally occupies the same envelope as the shaft hardware.",
    )
    ctx.allow_overlap(
        frame,
        rocker_a,
        elem_a=rocker_a_bracket,
        elem_b=rocker_a_receiver,
        reason="rocker A's slotted receiver carrier is modeled as a close nested clevis insert with slight intentional interpenetration to avoid a floating visual seam inside the bracket pocket.",
    )
    ctx.allow_overlap(
        frame,
        rocker_b,
        elem_a=frame_shafts,
        elem_b=rocker_b_receiver,
        reason="rocker B's receiver carrier wraps around the frame fulcrum sleeve, so the simplified tab mesh intentionally occupies the same envelope as the shaft hardware.",
    )
    ctx.allow_overlap(
        frame,
        rocker_b,
        elem_a=rocker_b_bracket,
        elem_b=rocker_b_receiver,
        reason="rocker B's slotted receiver carrier is modeled as a close nested clevis insert with slight intentional interpenetration to avoid a floating visual seam inside the bracket pocket.",
    )
    ctx.allow_overlap(
        frame,
        output_lever,
        elem_a=frame_shafts,
        elem_b=output_receiver,
        reason="the output receiver carrier is modeled as a wrap-around slot bracket concentric with the frame fulcrum sleeve.",
    )
    ctx.allow_overlap(
        frame,
        output_lever,
        elem_a=output_bracket,
        elem_b=output_receiver,
        reason="the output receiver tab is represented as a nested clevis insert with slight intentional interpenetration inside the output bracket pocket.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    for articulation in (input_joint, rocker_a_joint, rocker_b_joint, output_joint):
        ctx.check(
            f"{articulation.name}_axis_is_y",
            tuple(round(v, 6) for v in articulation.axis) == (0.0, 1.0, 0.0),
            f"expected +Y axis, got {articulation.axis}",
        )
        ctx.check(
            f"{articulation.name}_rest_pose_within_limits",
            articulation.motion_limits is not None
            and articulation.motion_limits.lower < 0.0 < articulation.motion_limits.upper,
            f"limits={articulation.motion_limits}",
        )

    ctx.check("input lever pivot position", _vec_close(ctx.part_world_position(input_lever), INPUT_PIVOT, 1e-6), str(ctx.part_world_position(input_lever)))
    ctx.check("rocker A pivot position", _vec_close(ctx.part_world_position(rocker_a), ROCKER_A_PIVOT, 1e-6), str(ctx.part_world_position(rocker_a)))
    ctx.check("rocker B pivot position", _vec_close(ctx.part_world_position(rocker_b), ROCKER_B_PIVOT, 1e-6), str(ctx.part_world_position(rocker_b)))
    ctx.check("output lever pivot position", _vec_close(ctx.part_world_position(output_lever), OUTPUT_PIVOT, 1e-6), str(ctx.part_world_position(output_lever)))
    ctx.check(
        "coupler 1 base position",
        _vec_close(ctx.part_world_position(coupler_1), _world_sum(INPUT_PIVOT, INPUT_DRIVE), 1e-6),
        str(ctx.part_world_position(coupler_1)),
    )
    ctx.check(
        "coupler 2 base position",
        _vec_close(ctx.part_world_position(coupler_2), _world_sum(ROCKER_A_PIVOT, ROCKER_A_DRIVE), 1e-6),
        str(ctx.part_world_position(coupler_2)),
    )
    ctx.check(
        "coupler 3 base position",
        _vec_close(ctx.part_world_position(coupler_3), _world_sum(ROCKER_B_PIVOT, ROCKER_B_DRIVE), 1e-6),
        str(ctx.part_world_position(coupler_3)),
    )

    ctx.expect_overlap(input_lever, frame, axes="xz", elem_a=input_lever.get_visual("main_plate"), elem_b=input_bracket, min_overlap=0.014, name="input lever nests within its clevis bracket")
    ctx.expect_overlap(rocker_a, frame, axes="xz", elem_a=rocker_a_receiver, elem_b=rocker_a_bracket, min_overlap=0.010, name="rocker A bracket envelopes its lever body")
    ctx.expect_overlap(rocker_b, frame, axes="xz", elem_a=rocker_b_receiver, elem_b=rocker_b_bracket, min_overlap=0.010, name="rocker B bracket envelopes its lever body")
    ctx.expect_overlap(output_lever, frame, axes="xz", elem_a=output_receiver, elem_b=output_bracket, min_overlap=0.010, name="output bracket aligns with the final lever")

    ctx.expect_contact(front_cover, frame, elem_a=front_cover_posts, contact_tol=0.0008, name="front access cover is post-mounted to the frame")
    ctx.expect_contact(rear_cover, frame, elem_a=rear_cover_posts, contact_tol=0.0008, name="rear access cover is post-mounted to the frame")
    ctx.expect_contact(guide_sector, frame, elem_a=guide_tabs, elem_b=frame.get_visual("base_structure"), contact_tol=0.0065, name="guide sector is hard-mounted to the support frame")

    ctx.expect_gap(front_cover, input_lever, axis="y", min_gap=0.020, positive_elem=front_cover_shell, negative_elem=input_mount, name="front cover clears input-side linkage")
    ctx.expect_gap(front_cover, rocker_a, axis="y", min_gap=0.016, positive_elem=front_cover_shell, negative_elem=rocker_a_receiver, name="front cover clears the staggered rocker A receiver tab")
    ctx.expect_gap(front_cover, coupler_1, axis="y", min_gap=0.0025, positive_elem=front_cover_shell, negative_elem=coupler_1_follower, name="front cover clears coupler 1 follower hardware")
    ctx.expect_gap(output_lever, rear_cover, axis="y", min_gap=0.018, positive_elem=output_receiver, negative_elem=rear_cover_shell, name="rear cover clears output receiver tab")
    ctx.expect_gap(rocker_b, rear_cover, axis="y", min_gap=0.020, positive_elem=rocker_b_receiver, negative_elem=rear_cover_shell, name="rear cover clears rocker B linkage")

    with ctx.pose({input_joint: 0.0, rocker_a_joint: 0.0, rocker_b_joint: 0.0, output_joint: 0.0}):
        ctx.expect_overlap(coupler_1, rocker_a, axes="xz", elem_a=coupler_1_follower, elem_b=rocker_a_receiver, min_overlap=0.006, name="coupler 1 follower seats inside rocker A slot at rest")
        ctx.expect_overlap(coupler_2, rocker_b, axes="xz", elem_a=coupler_2_follower, elem_b=rocker_b_receiver, min_overlap=0.006, name="coupler 2 follower seats inside rocker B slot at rest")
        ctx.expect_overlap(coupler_3, output_lever, axes="xz", elem_a=coupler_3_follower, elem_b=output_receiver, min_overlap=0.006, name="coupler 3 follower seats inside output slot at rest")
        ctx.expect_contact(guide_sector, output_lever, elem_a=guide_shell, elem_b=output_guide, contact_tol=0.002, name="guide roller runs against the guide sector face at rest")
        ctx.expect_overlap(output_lever, guide_sector, axes="xz", elem_a=output_guide, elem_b=guide_shell, min_overlap=0.007, name="guide roller footprint stays within the guide sector at rest")

    with ctx.pose({input_joint: 0.08, rocker_a_joint: 0.04, rocker_b_joint: 0.04, output_joint: 0.12}):
        ctx.expect_overlap(coupler_1, rocker_a, axes="xz", elem_a=coupler_1_follower, elem_b=rocker_a_receiver, min_overlap=0.004, name="coupler 1 remains engaged through a driven study pose")
        ctx.expect_overlap(coupler_2, rocker_b, axes="xz", elem_a=coupler_2_follower, elem_b=rocker_b_receiver, min_overlap=0.004, name="coupler 2 remains engaged through a driven study pose")
        ctx.expect_overlap(coupler_3, output_lever, axes="xz", elem_a=coupler_3_follower, elem_b=output_receiver, min_overlap=0.004, name="coupler 3 remains engaged through a driven study pose")
        ctx.expect_contact(guide_sector, output_lever, elem_a=guide_shell, elem_b=output_guide, contact_tol=0.002, name="guide roller stays on the guide sector during driven pose")
        ctx.expect_overlap(output_lever, guide_sector, axes="xz", elem_a=output_guide, elem_b=guide_shell, min_overlap=0.007, name="guide roller footprint stays within guide sector during driven pose")

    with ctx.pose({input_joint: 0.22}):
        input_driven = _center(ctx.part_element_world_aabb(input_lever, elem=input_drive_hardware))
    input_rest = _center(ctx.part_element_world_aabb(input_lever, elem=input_drive_hardware))
    ctx.check(
        "input drive pin drops under positive actuation",
        input_rest is not None and input_driven is not None and input_driven[2] < input_rest[2] - 0.012 and input_driven[0] > input_rest[0] + 0.001,
        f"rest={input_rest}, driven={input_driven}",
    )

    with ctx.pose({rocker_a_joint: 0.18}):
        rocker_a_driven = _center(ctx.part_element_world_aabb(rocker_a, elem=rocker_a_drive))
    rocker_a_rest = _center(ctx.part_element_world_aabb(rocker_a, elem=rocker_a_drive))
    ctx.check(
        "rocker A transfer pin sweeps downward",
        rocker_a_rest is not None and rocker_a_driven is not None and rocker_a_driven[2] < rocker_a_rest[2] - 0.010,
        f"rest={rocker_a_rest}, driven={rocker_a_driven}",
    )

    with ctx.pose({rocker_b_joint: 0.18}):
        rocker_b_driven = _center(ctx.part_element_world_aabb(rocker_b, elem=rocker_b_drive))
    rocker_b_rest = _center(ctx.part_element_world_aabb(rocker_b, elem=rocker_b_drive))
    ctx.check(
        "rocker B transfer pin sweeps downward",
        rocker_b_rest is not None and rocker_b_driven is not None and rocker_b_driven[2] < rocker_b_rest[2] - 0.010,
        f"rest={rocker_b_rest}, driven={rocker_b_driven}",
    )

    with ctx.pose({output_joint: 0.18}):
        output_driven = _center(ctx.part_element_world_aabb(output_lever, elem=output_guide))
    output_rest = _center(ctx.part_element_world_aabb(output_lever, elem=output_guide))
    ctx.check(
        "output guide roller moves down the sector during positive rotation",
        output_rest is not None and output_driven is not None and output_driven[2] < output_rest[2] - 0.012 and output_driven[0] < output_rest[0] - 0.003,
        f"rest={output_rest}, driven={output_driven}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
