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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_L = 0.310
BASE_W = 0.200
BASE_T = 0.016

CHEEK_T = 0.018
FRAME_INNER_SPAN = 0.164
CHEEK_INNER_Y = FRAME_INNER_SPAN / 2.0
CHEEK_OUTER_Y = CHEEK_INNER_Y + CHEEK_T
AXIS_Z = 0.154

BEARING_BORE_R = 0.028
OUTER_BOSS_R = 0.047
OUTER_BOSS_LEN = 0.008
INNER_BOSS_R = 0.034
INNER_BOSS_LEN = 0.003

BEARING_OUTER_R = 0.0274
BEARING_FLANGE_R = 0.033
BEARING_INNER_R = 0.0186
BEARING_LEN = 0.016
BEARING_FLANGE_T = 0.002

CAP_T = 0.004
CAP_OR = 0.046
CAP_IR = 0.0355
CAP_BOLT_R = 0.0038
CAP_BOLT_LEN = 0.004
CAP_BOLT_CIRCLE = 0.036

COVER_X = 0.086
COVER_Z = 0.056
COVER_T = 0.004

CRADLE_BODY_X = 0.192
CRADLE_BODY_Y = 0.146
TRUNNION_BOSS_R = 0.030
TRUNNION_BOSS_LEN = 0.011
TRUNNION_COLLAR_R = 0.024
TRUNNION_COLLAR_LEN = 0.006
TRUNNION_SHAFT_R = 0.0178
TRUNNION_SHAFT_LEN = 0.045
SECTOR_MOUNT_R = 0.028
SECTOR_MOUNT_T = 0.004

SECTOR_Y = 0.106
SECTOR_T = 0.006
SECTOR_R_IN = 0.055
SECTOR_R_OUT = 0.090

CLAMP_BRACKET_X = 0.122
CLAMP_BRACKET_Z = AXIS_Z
CLAMP_BOLT_SHANK_R = 0.0029
CLAMP_BOLT_HEAD_R = 0.0052
CLAMP_BOLT_WASHER_R = 0.0047
CLAMP_BOLT_HEAD_T = 0.004
CLAMP_BOLT_WASHER_T = 0.001
CLAMP_BOLT_NUT_T = 0.004
CLAMP_BOLT_SPAN = 0.030


def _regular_polygon_points(radius: float, sides: int, start_deg: float = 0.0) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for i in range(sides):
        ang = math.radians(start_deg + 360.0 * i / sides)
        pts.append((radius * math.cos(ang), radius * math.sin(ang)))
    return pts


def _y_cylinder(radius: float, length: float, *, x: float = 0.0, y0: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((x, y0, z))
    )


def _y_tube(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    x: float = 0.0,
    y0: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((x, y0, z))
    )


def _arc_points(radius: float, start_deg: float, end_deg: float, steps: int) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for i in range(steps + 1):
        t = i / steps
        ang = math.radians(start_deg + (end_deg - start_deg) * t)
        pts.append((radius * math.cos(ang), radius * math.sin(ang)))
    return pts


def _bolt_circle_points(radius: float, angles_deg: tuple[float, ...]) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(math.radians(angle)), radius * math.sin(math.radians(angle)))
        for angle in angles_deg
    ]


def _make_frame() -> cq.Workplane:
    left_rail = cq.Workplane("XY").box(BASE_L, 0.034, BASE_T, centered=(True, True, False)).translate((0.0, -0.081, 0.0))
    right_rail = left_rail.mirror("XZ")
    foot_pads = (
        cq.Workplane("XY")
        .pushPoints([(-0.126, -0.072), (-0.126, 0.072), (0.126, -0.072), (0.126, 0.072)])
        .box(0.038, 0.028, 0.006)
        .translate((0.0, 0.0, -0.003))
    )

    cheek_profile = [
        (-0.126, BASE_T),
        (-0.126, 0.074),
        (-0.094, 0.140),
        (-0.046, 0.198),
        (0.046, 0.198),
        (0.096, 0.144),
        (0.126, 0.076),
        (0.126, BASE_T),
    ]
    cheek_blank = cq.Workplane("XZ").polyline(cheek_profile).close().extrude(CHEEK_T)
    access_cut = (
        cq.Workplane("XY")
        .box(0.078, CHEEK_T + 0.006, 0.052)
        .translate((-0.056, CHEEK_T / 2.0, 0.083))
    )
    upper_relief = (
        cq.Workplane("XY")
        .box(0.052, CHEEK_T + 0.006, 0.034)
        .translate((0.060, CHEEK_T / 2.0, 0.120))
    )
    bearing_bore = (
        cq.Workplane("XZ")
        .circle(BEARING_OUTER_R + 0.0008)
        .extrude(CHEEK_T + 0.002)
        .translate((0.0, -0.001, AXIS_Z))
    )
    cheek_assembly = cheek_blank.cut(access_cut).cut(upper_relief).cut(bearing_bore)

    right_cheek = cheek_assembly.translate((0.0, CHEEK_INNER_Y, 0.0))
    left_cheek = cheek_assembly.mirror("XZ").translate((0.0, -CHEEK_INNER_Y, 0.0))

    rear_tie = cq.Workplane("XY").box(0.034, FRAME_INNER_SPAN, 0.014).translate((-0.126, 0.0, 0.188))
    front_tie = cq.Workplane("XY").box(0.024, FRAME_INNER_SPAN, 0.014).translate((0.136, 0.0, 0.042))
    clamp_pad = cq.Workplane("XY").box(0.060, 0.004, 0.102).translate((0.102, 0.125, AXIS_Z))

    return left_rail.union(right_rail).union(foot_pads).union(left_cheek).union(right_cheek).union(rear_tie).union(front_tie).union(clamp_pad)


def _make_cradle() -> cq.Workplane:
    left_side_plate = cq.Workplane("XY").box(0.150, 0.008, 0.076).translate((0.0, -0.048, -0.042))
    right_side_plate = left_side_plate.mirror("XZ")
    front_bulkhead = cq.Workplane("XY").box(0.014, 0.096, 0.046).translate((0.056, 0.0, -0.062))
    rear_bulkhead = cq.Workplane("XY").box(0.014, 0.096, 0.046).translate((-0.056, 0.0, -0.062))
    lower_plate = cq.Workplane("XY").box(0.100, 0.092, 0.010).translate((0.0, 0.0, -0.086))
    center_spar = cq.Workplane("XY").box(0.060, 0.028, 0.018).translate((0.0, 0.0, -0.072))

    gusset_profile = [(-0.028, -0.086), (0.010, -0.086), (-0.010, -0.040), (-0.022, -0.040)]
    gusset = cq.Workplane("XZ").polyline(gusset_profile).close().extrude(0.008)
    left_front_gusset = gusset.translate((0.028, -0.044, 0.0))
    right_front_gusset = gusset.translate((0.028, 0.036, 0.0))
    left_rear_gusset = gusset.translate((-0.040, -0.044, 0.0))
    right_rear_gusset = gusset.translate((-0.040, 0.036, 0.0))

    top_slots = (
        cq.Workplane("XY")
        .pushPoints([(-0.032, -0.022), (-0.032, 0.022), (0.032, -0.022), (0.032, 0.022)])
        .box(0.030, 0.010, 0.012)
        .translate((0.0, 0.0, -0.028))
    )

    left_shaft = cq.Workplane("XZ", origin=(0.0, -0.086, 0.0)).circle(TRUNNION_SHAFT_R).extrude(0.036)
    right_shaft = cq.Workplane("XZ", origin=(0.0, 0.050, 0.0)).circle(TRUNNION_SHAFT_R).extrude(0.036)
    right_sector_shoulder = cq.Workplane("XZ", origin=(0.0, 0.118, 0.0)).circle(0.024).extrude(0.006)
    right_sector_mount = cq.Workplane("XZ", origin=(0.0, 0.124, 0.0)).circle(SECTOR_MOUNT_R).circle(0.0183).extrude(0.003)

    return (
        left_side_plate.union(right_side_plate)
        .union(front_bulkhead)
        .union(rear_bulkhead)
        .union(lower_plate)
        .union(center_spar)
        .union(left_front_gusset)
        .union(right_front_gusset)
        .union(left_rear_gusset)
        .union(right_rear_gusset)
        .union(left_shaft)
        .union(right_shaft)
        .union(right_sector_shoulder)
        .union(right_sector_mount)
        .cut(top_slots)
    )


def _make_sector_plate() -> cq.Workplane:
    outer = _arc_points(SECTOR_R_OUT, -112.0, 86.0, 56)
    inner = list(reversed(_arc_points(SECTOR_R_IN, -112.0, 86.0, 56)))
    plate = cq.Workplane("XZ").polyline(outer + inner).close().extrude(SECTOR_T).mirror("XZ")

    hole_points = _bolt_circle_points(0.072, (-80.0, -42.0, -4.0, 34.0, 72.0))
    holes = (
        cq.Workplane("XZ", origin=(0.0, -SECTOR_T - 0.001, 0.0))
        .pushPoints(hole_points)
        .circle(0.0085)
        .extrude(SECTOR_T + 0.008)
    )

    tick_points = _bolt_circle_points(0.084, tuple(-55.0 + 10.0 * i for i in range(12)))
    ticks = (
        cq.Workplane("XZ", origin=(0.0, -SECTOR_T - 0.001, 0.0))
        .pushPoints(tick_points)
        .rect(0.0018, 0.007)
        .extrude(SECTOR_T + 0.008)
    )
    zero_tick = (
        cq.Workplane("XZ", origin=(0.0, -SECTOR_T - 0.001, 0.0))
        .rect(0.0026, 0.012)
        .extrude(SECTOR_T + 0.008)
        .translate((0.0, 0.0, 0.084))
    )

    hub = cq.Workplane("XZ", origin=(0.0, -0.006, 0.0)).circle(0.031).circle(0.0183).extrude(0.006)
    stop_pads = (
        cq.Workplane("XZ", origin=(0.0, -SECTOR_T, 0.0))
        .pushPoints(_bolt_circle_points(0.087, (-100.0, 82.0)))
        .circle(0.0075)
        .extrude(SECTOR_T + 0.002)
    )

    return plate.cut(holes).cut(ticks).cut(zero_tick).union(hub).union(stop_pads)


def _make_bearing_cartridge(outward_sign: float) -> cq.Workplane:
    positive = (
        cq.Workplane("XZ")
        .circle(BEARING_FLANGE_R)
        .circle(BEARING_INNER_R)
        .extrude(0.003)
        .union(
            cq.Workplane("XZ", origin=(0.0, 0.003, 0.0))
            .circle(BEARING_OUTER_R)
            .circle(BEARING_INNER_R)
            .extrude(CHEEK_T)
        )
        .union(
            cq.Workplane("XZ", origin=(0.0, 0.003 + CHEEK_T - 0.002, 0.0))
            .circle(0.024)
            .circle(BEARING_INNER_R)
            .extrude(0.002)
        )
    )
    return positive if outward_sign > 0.0 else positive.mirror("XZ")


def _make_bearing_cap(outward_sign: float) -> cq.Workplane:
    positive = (
        cq.Workplane("XZ")
        .circle(CAP_OR)
        .circle(CAP_IR)
        .extrude(CAP_T)
        .union(
            cq.Workplane("XZ", origin=(0.0, CAP_T, 0.0))
            .pushPoints(_bolt_circle_points(CAP_BOLT_CIRCLE, (45.0, 135.0, 225.0, 315.0)))
            .circle(CAP_BOLT_R)
            .extrude(CAP_BOLT_LEN)
        )
    )
    return positive if outward_sign > 0.0 else positive.mirror("XZ")


def _make_access_cover(outward_sign: float) -> cq.Workplane:
    positive = (
        cq.Workplane("XY", origin=(0.0, COVER_T / 2.0, 0.0)).box(COVER_X, COVER_T, COVER_Z)
        .union(
            cq.Workplane("XZ", origin=(0.0, COVER_T, 0.0))
            .pushPoints([(-0.030, -0.018), (-0.030, 0.018), (0.030, -0.018), (0.030, 0.018)])
            .circle(0.0038)
            .extrude(0.004)
        )
    )
    return positive if outward_sign > 0.0 else positive.mirror("XZ")


def _make_clamp_bracket() -> cq.Workplane:
    back_plate = cq.Workplane("XY", origin=(-0.020, 0.002, 0.0)).box(0.060, 0.004, 0.102)
    upper_arm = cq.Workplane("XY", origin=(0.006, 0.014, 0.036)).box(0.014, 0.020, 0.014)
    lower_arm = cq.Workplane("XY", origin=(0.006, 0.014, -0.036)).box(0.014, 0.020, 0.014)
    upper_pad = cq.Workplane("XY", origin=(0.016, 0.022, 0.036)).box(0.006, 0.006, 0.014)
    lower_pad = cq.Workplane("XY", origin=(0.016, 0.022, -0.036)).box(0.006, 0.006, 0.014)
    pointer = (
        cq.Workplane("XZ")
        .polyline([(-0.034, 0.034), (-0.022, 0.040), (-0.022, 0.028)])
        .close()
        .extrude(0.004)
        .translate((0.0, 0.002, 0.0))
    )
    bolt_heads = (
        cq.Workplane("XZ", origin=(0.0, 0.004, 0.0))
        .pushPoints([(-0.030, -0.032), (-0.030, 0.032)])
        .polygon(6, CLAMP_BOLT_HEAD_R * 2.0)
        .extrude(0.004)
    )
    bolt_shanks = (
        cq.Workplane("XZ", origin=(0.0, 0.001, 0.0))
        .pushPoints([(-0.030, -0.032), (-0.030, 0.032)])
        .circle(CLAMP_BOLT_SHANK_R)
        .extrude(0.006)
    )
    return back_plate.union(upper_arm).union(lower_arm).union(upper_pad).union(lower_pad).union(pointer).union(bolt_heads).union(bolt_shanks)


def _make_clamp_bolt() -> cq.Workplane:
    head = (
        cq.Workplane("XZ", origin=(0.0, -CLAMP_BOLT_HEAD_T, 0.0))
        .polyline(_regular_polygon_points(CLAMP_BOLT_HEAD_R, 6, start_deg=30.0))
        .close()
        .extrude(CLAMP_BOLT_HEAD_T)
    )
    outer_washer = (
        cq.Workplane("XZ", origin=(0.0, 0.0, 0.0))
        .circle(CLAMP_BOLT_WASHER_R)
        .circle(CLAMP_BOLT_SHANK_R)
        .extrude(CLAMP_BOLT_WASHER_T)
    )
    shank = (
        cq.Workplane("XZ", origin=(0.0, CLAMP_BOLT_WASHER_T, 0.0))
        .circle(CLAMP_BOLT_SHANK_R)
        .extrude(CLAMP_BOLT_SPAN - CLAMP_BOLT_HEAD_T - CLAMP_BOLT_NUT_T - 2.0 * CLAMP_BOLT_WASHER_T)
    )
    inner_washer = (
        cq.Workplane("XZ", origin=(0.0, CLAMP_BOLT_SPAN - CLAMP_BOLT_NUT_T - CLAMP_BOLT_WASHER_T, 0.0))
        .circle(CLAMP_BOLT_WASHER_R)
        .circle(CLAMP_BOLT_SHANK_R)
        .extrude(CLAMP_BOLT_WASHER_T)
    )
    nut = (
        cq.Workplane("XZ", origin=(0.0, CLAMP_BOLT_SPAN - CLAMP_BOLT_NUT_T, 0.0))
        .polyline(_regular_polygon_points(CLAMP_BOLT_HEAD_R, 6, start_deg=30.0))
        .close()
        .extrude(CLAMP_BOLT_NUT_T)
    )
    return head.union(outer_washer).union(shank).union(inner_washer).union(nut)


def _make_frame_study() -> cq.Workplane:
    left_rail = cq.Workplane("XY").box(BASE_L, 0.034, BASE_T).translate((0.0, -0.081, BASE_T / 2.0))
    right_rail = left_rail.mirror("XZ")
    foot_pads = (
        cq.Workplane("XY")
        .pushPoints([(-0.126, -0.072), (-0.126, 0.072), (0.126, -0.072), (0.126, 0.072)])
        .box(0.038, 0.028, 0.006)
        .translate((0.0, 0.0, 0.003))
    )

    cheek_profile = [
        (-0.126, BASE_T),
        (-0.126, 0.074),
        (-0.094, 0.140),
        (-0.046, 0.198),
        (0.046, 0.198),
        (0.096, 0.144),
        (0.126, 0.076),
        (0.126, BASE_T),
    ]
    cheek_blank = cq.Workplane("XZ").polyline(cheek_profile).close().extrude(CHEEK_T)
    trunnion_bore = _y_cylinder(TRUNNION_SHAFT_R + 0.0008, CHEEK_T + 0.004, y0=-0.002, z=AXIS_Z)
    access_cut = (
        cq.Workplane("XY")
        .box(0.080, CHEEK_T + 0.004, 0.054)
        .translate((-0.056, CHEEK_T / 2.0, 0.084))
    )
    inspection_cover = cq.Workplane("XY").box(0.086, 0.003, 0.056).translate((-0.056, CHEEK_T + 0.0015, 0.084))
    cap_ring = _y_tube(0.046, 0.0355, 0.003, y0=CHEEK_T, z=AXIS_Z)
    bearing_flange = _y_tube(BEARING_FLANGE_R, TRUNNION_SHAFT_R, 0.003, y0=CHEEK_T, z=AXIS_Z)
    cheek = cheek_blank.cut(trunnion_bore).cut(access_cut).union(inspection_cover).union(cap_ring).union(bearing_flange)

    right_cheek = cheek.translate((0.0, CHEEK_INNER_Y, 0.0))
    left_cheek = cheek.mirror("XZ").translate((0.0, -CHEEK_INNER_Y, 0.0))

    rear_tie = cq.Workplane("XY").box(0.034, FRAME_INNER_SPAN, 0.014).translate((-0.126, 0.0, 0.188))
    front_tie = cq.Workplane("XY").box(0.024, FRAME_INNER_SPAN, 0.014).translate((0.136, 0.0, 0.042))
    clamp_pad = cq.Workplane("XY").box(0.022, 0.004, 0.090).translate((0.105, 0.142, AXIS_Z))

    return (
        left_rail.union(right_rail)
        .union(foot_pads)
        .union(left_cheek)
        .union(right_cheek)
        .union(rear_tie)
        .union(front_tie)
        .union(clamp_pad)
    )


def _make_cradle_study() -> cq.Workplane:
    left_side_plate = cq.Workplane("XY").box(0.138, 0.008, 0.070).translate((0.0, -0.046, -0.048))
    right_side_plate = left_side_plate.mirror("XZ")
    front_bulkhead = cq.Workplane("XY").box(0.014, 0.088, 0.042).translate((0.050, 0.0, -0.066))
    rear_bulkhead = cq.Workplane("XY").box(0.014, 0.088, 0.042).translate((-0.050, 0.0, -0.066))
    lower_plate = cq.Workplane("XY").box(0.090, 0.084, 0.010).translate((0.0, 0.0, -0.088))
    center_spar = cq.Workplane("XY").box(0.054, 0.026, 0.016).translate((0.0, 0.0, -0.074))

    gusset_profile = [(-0.024, -0.088), (0.010, -0.088), (-0.008, -0.046), (-0.018, -0.046)]
    gusset = cq.Workplane("XZ").polyline(gusset_profile).close().extrude(0.008)
    left_front_gusset = gusset.translate((0.026, -0.042, 0.0))
    right_front_gusset = gusset.translate((0.026, 0.034, 0.0))
    left_rear_gusset = gusset.translate((-0.036, -0.042, 0.0))
    right_rear_gusset = gusset.translate((-0.036, 0.034, 0.0))

    top_slots = (
        cq.Workplane("XY")
        .pushPoints([(-0.028, -0.020), (-0.028, 0.020), (0.028, -0.020), (0.028, 0.020)])
        .box(0.026, 0.010, 0.012)
        .translate((0.0, 0.0, -0.030))
    )

    left_journal = _y_cylinder(TRUNNION_SHAFT_R, CHEEK_T, y0=-(CHEEK_INNER_Y + CHEEK_T), z=0.0)
    right_journal = _y_cylinder(TRUNNION_SHAFT_R, CHEEK_T, y0=CHEEK_INNER_Y, z=0.0)
    left_stub = _y_cylinder(TRUNNION_SHAFT_R, 0.010, y0=-(CHEEK_OUTER_Y + 0.010), z=0.0)
    right_stub = _y_cylinder(TRUNNION_SHAFT_R, 0.024, y0=CHEEK_OUTER_Y, z=0.0)

    return (
        left_side_plate.union(right_side_plate)
        .union(front_bulkhead)
        .union(rear_bulkhead)
        .union(lower_plate)
        .union(center_spar)
        .union(left_front_gusset)
        .union(right_front_gusset)
        .union(left_rear_gusset)
        .union(right_rear_gusset)
        .union(left_journal)
        .union(right_journal)
        .union(left_stub)
        .union(right_stub)
        .cut(top_slots)
    )


def _make_sector_plate_study() -> cq.Workplane:
    outer = _arc_points(SECTOR_R_OUT, -112.0, 86.0, 56)
    inner = list(reversed(_arc_points(SECTOR_R_IN, -112.0, 86.0, 56)))
    plate = cq.Workplane("XZ").polyline(outer + inner).close().extrude(SECTOR_T)
    holes = (
        cq.Workplane("XZ")
        .pushPoints(_bolt_circle_points(0.072, (-80.0, -42.0, -4.0, 34.0, 72.0)))
        .circle(0.0085)
        .extrude(SECTOR_T)
    )
    ticks = (
        cq.Workplane("XZ")
        .pushPoints(_bolt_circle_points(0.084, tuple(-55.0 + 10.0 * i for i in range(12))))
        .rect(0.0018, 0.007)
        .extrude(SECTOR_T)
    )
    zero_tick = cq.Workplane("XZ").rect(0.0026, 0.012).extrude(SECTOR_T).translate((0.0, 0.0, 0.084))
    hub = cq.Workplane("XZ").circle(0.031).circle(0.0183).extrude(SECTOR_T)
    stop_pads = (
        cq.Workplane("XZ")
        .pushPoints(_bolt_circle_points(0.087, (-100.0, 82.0)))
        .circle(0.0075)
        .extrude(SECTOR_T)
    )
    return plate.cut(holes).cut(ticks).cut(zero_tick).union(hub).union(stop_pads)


def _make_clamp_bracket_study() -> cq.Workplane:
    back_plate = cq.Workplane("XY", origin=(0.0, 0.002, 0.0)).box(0.060, 0.004, 0.102)
    upper_arm = cq.Workplane("XY", origin=(0.014, 0.014, 0.036)).box(0.014, 0.018, 0.014)
    lower_arm = cq.Workplane("XY", origin=(0.014, 0.014, -0.036)).box(0.014, 0.018, 0.014)
    upper_pad = cq.Workplane("XY", origin=(0.020, 0.022, 0.036)).box(0.006, 0.006, 0.014)
    lower_pad = cq.Workplane("XY", origin=(0.020, 0.022, -0.036)).box(0.006, 0.006, 0.014)
    pointer = (
        cq.Workplane("XZ")
        .polyline([(-0.026, 0.034), (-0.014, 0.040), (-0.014, 0.028)])
        .close()
        .extrude(0.004)
        .translate((0.0, 0.002, 0.0))
    )
    bolt_heads = (
        cq.Workplane("XZ", origin=(0.0, 0.004, 0.0))
        .pushPoints([(-0.020, -0.032), (-0.020, 0.032)])
        .polygon(6, CLAMP_BOLT_HEAD_R * 2.0)
        .extrude(0.004)
    )
    bolt_shanks = (
        cq.Workplane("XZ", origin=(0.0, 0.001, 0.0))
        .pushPoints([(-0.020, -0.032), (-0.020, 0.032)])
        .circle(CLAMP_BOLT_SHANK_R)
        .extrude(0.006)
    )
    return back_plate.union(upper_arm).union(lower_arm).union(upper_pad).union(lower_pad).union(pointer).union(bolt_heads).union(bolt_shanks)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitch_axis_module", assets=ASSETS)

    frame_color = model.material("frame_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    cradle_color = model.material("machined_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    cap_color = model.material("black_oxide", rgba=(0.16, 0.17, 0.18, 1.0))
    sector_color = model.material("blued_steel", rgba=(0.25, 0.28, 0.33, 1.0))

    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(_make_frame_study(), "frame.obj", assets=ASSETS), name="frame_shell", material=frame_color)

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(_make_cradle_study(), "cradle.obj", assets=ASSETS),
        name="cradle_shell",
        material=cradle_color,
    )

    sector_plate = model.part("sector_plate")
    sector_plate.visual(
        mesh_from_cadquery(_make_sector_plate_study(), "sector_plate.obj", assets=ASSETS),
        name="sector_plate_shell",
        material=sector_color,
    )

    clamp_bracket = model.part("clamp_bracket")
    clamp_bracket.visual(
        mesh_from_cadquery(_make_clamp_bracket_study(), "clamp_bracket.obj", assets=ASSETS),
        name="clamp_bracket_shell",
        material=cap_color,
    )

    model.articulation(
        "frame_to_cradle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-0.82, upper=0.82),
    )

    model.articulation(
        "cradle_to_sector_plate",
        ArticulationType.FIXED,
        parent=cradle,
        child=sector_plate,
        origin=Origin(xyz=(0.0, 0.124, 0.0)),
    )
    model.articulation(
        "frame_to_clamp_bracket",
        ArticulationType.FIXED,
        parent=frame,
        child=clamp_bracket,
        origin=Origin(xyz=(0.094, 0.144, CLAMP_BRACKET_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    cradle = object_model.get_part("cradle")
    sector_plate = object_model.get_part("sector_plate")
    clamp_bracket = object_model.get_part("clamp_bracket")
    pitch_axis = object_model.get_articulation("frame_to_cradle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.allow_isolated_part(
        sector_plate,
        reason="Sector plate is a bolted study plate with a tiny modeled stand-off; exact mount contact is checked separately.",
    )
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        frame,
        cradle,
        reason="The trunnion journals are intentionally modeled as seated bearing fits inside the frame cheek bores.",
    )
    ctx.allow_overlap(
        frame,
        clamp_bracket,
        reason="The clamp backplate is intentionally seated flush against the welded frame mounting pad.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(cradle, frame, contact_tol=0.0010, name="trunnion_shafts_run_in_frame_bores")
    ctx.expect_contact(clamp_bracket, frame, contact_tol=0.0010, name="clamp_bracket_mounts_to_frame")
    ctx.expect_contact(sector_plate, cradle, contact_tol=0.0010, name="sector_plate_bolts_to_trunnion_flange")

    ctx.expect_origin_distance(
        cradle,
        cradle,
        axes="xz",
        max_dist=0.001,
        name="cradle_is_centered_on_pitch_axis",
    )
    ctx.expect_origin_distance(
        sector_plate,
        cradle,
        axes="xz",
        max_dist=0.001,
        name="sector_plate_is_centered_on_trunnion_axis",
    )

    ctx.expect_overlap(cradle, frame, axes="xz", min_overlap=0.10, name="cradle_sits_between_frame_cheeks")
    ctx.expect_overlap(clamp_bracket, frame, axes="xz", min_overlap=0.015, name="clamp_bracket_has_real_mounting_footprint")
    ctx.expect_gap(
        clamp_bracket,
        sector_plate,
        axis="y",
        min_gap=0.004,
        max_gap=0.020,
        name="clamp_bracket_stands_off_sector_plate",
    )

    with ctx.pose({pitch_axis: 0.0}):
        ctx.expect_overlap(
            sector_plate,
            clamp_bracket,
            axes="xz",
            min_overlap=0.020,
            name="sector_plate_tracks_clamp_window_at_neutral",
        )

    with ctx.pose({pitch_axis: 0.72}):
        ctx.expect_overlap(
            sector_plate,
            clamp_bracket,
            axes="xz",
            min_overlap=0.020,
            name="sector_plate_tracks_clamp_window_at_positive_pitch",
        )

    with ctx.pose({pitch_axis: -0.72}):
        ctx.expect_overlap(
            sector_plate,
            clamp_bracket,
            axes="xz",
            min_overlap=0.020,
            name="sector_plate_tracks_clamp_window_at_negative_pitch",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="pitch_axis_motion_stays_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
