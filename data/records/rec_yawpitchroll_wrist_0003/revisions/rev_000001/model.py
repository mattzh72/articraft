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

BASE_W = 0.180
BASE_D = 0.140
BASE_Z0 = -0.072
BASE_Z1 = -0.054
PEDESTAL_Z0 = -0.046
PEDESTAL_Z1 = -0.010
YAW_SEAT_Z0 = -0.010
YAW_SEAT_Z1 = 0.000
YAW_RETAINER_INNER_RADIUS = 0.0605

YAW_CHEEK_CENTER_Y = 0.052
YAW_CHEEK_THICKNESS = 0.016
YAW_CHEEK_INNER_Y = YAW_CHEEK_CENTER_Y - YAW_CHEEK_THICKNESS / 2.0
YAW_CHEEK_OUTER_Y = YAW_CHEEK_CENTER_Y + YAW_CHEEK_THICKNESS / 2.0
YAW_COVER_FACE_Y = 0.062
PITCH_AXIS_Z = 0.074

PITCH_JOURNAL_RADIUS = 0.0206
PITCH_SHOULDER_RADIUS = 0.026
PITCH_FRAME_OUTER_X = 0.072
PITCH_FRAME_OUTER_Y = 0.044
PITCH_FRAME_POST_X = 0.012
PITCH_FRAME_POST_OFFSET_X = 0.030
PITCH_BRIDGE_Z0 = 0.020
PITCH_BRIDGE_Z1 = 0.030
PITCH_CLEAR_RADIUS = 0.0245
PITCH_SHOULDER_Y0 = 0.022
PITCH_SHOULDER_Y1 = 0.034
PITCH_JOURNAL_Y0 = 0.034
PITCH_JOURNAL_Y1 = 0.060

ROLL_SHAFT_RADIUS = 0.0125
ROLL_JOURNAL_RADIUS = 0.0160
ROLL_TRANSITION_RADIUS = 0.019
ROLL_HUB_RADIUS = 0.022
ROLL_BEARING_OUTER_RADIUS = 0.030
ROLL_BEARING_FACE_RADIUS = 0.034
ROLL_BEARING_X0 = 0.036
ROLL_BEARING_X1 = 0.042


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    merged = shapes[0]
    for shape in shapes[1:]:
        merged = merged.union(shape)
    return merged


def _box_z(size_x: float, size_y: float, z0: float, z1: float) -> cq.Workplane:
    return cq.Workplane("XY").rect(size_x, size_y).extrude(z1 - z0).translate((0.0, 0.0, z0))


def _centered_cyl_z(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -length / 2.0))


def _cyl_z(radius: float, z0: float, z1: float) -> cq.Workplane:
    return _centered_cyl_z(radius, z1 - z0).translate((0.0, 0.0, (z0 + z1) / 2.0))


def _ring_z(outer_radius: float, inner_radius: float, z0: float, z1: float) -> cq.Workplane:
    return _cyl_z(outer_radius, z0, z1).cut(_cyl_z(inner_radius, z0, z1))


def _cyl_x(radius: float, x0: float, x1: float) -> cq.Workplane:
    return _centered_cyl_z(radius, x1 - x0).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0).translate(
        ((x0 + x1) / 2.0, 0.0, 0.0)
    )


def _ring_x(outer_radius: float, inner_radius: float, x0: float, x1: float) -> cq.Workplane:
    return _cyl_x(outer_radius, x0, x1).cut(_cyl_x(inner_radius, x0, x1))


def _cyl_y(radius: float, y0: float, y1: float) -> cq.Workplane:
    return _centered_cyl_z(radius, y1 - y0).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0).translate(
        (0.0, (y0 + y1) / 2.0, 0.0)
    )


def _ring_y(outer_radius: float, inner_radius: float, y0: float, y1: float) -> cq.Workplane:
    return _cyl_y(outer_radius, y0, y1).cut(_cyl_y(inner_radius, y0, y1))


def _bolt_heads_z(points_xy: list[tuple[float, float]], radius: float, z0: float, z1: float) -> cq.Workplane:
    return _union_all([_cyl_z(radius, z0, z1).translate((x, y, 0.0)) for x, y in points_xy])


def _bolt_heads_x(points_yz: list[tuple[float, float]], radius: float, x0: float, x1: float) -> cq.Workplane:
    return _union_all([_cyl_x(radius, x0, x1).translate((0.0, y, z)) for y, z in points_yz])


def _bolt_heads_y(points_xz: list[tuple[float, float]], radius: float, y0: float, y1: float) -> cq.Workplane:
    return _union_all([_cyl_y(radius, y0, y1).translate((x, 0.0, z)) for x, z in points_xz])


def _make_base_support() -> cq.Workplane:
    body = _union_all(
        [
            _box_z(BASE_W, BASE_D, BASE_Z0, BASE_Z1),
            _box_z(0.104, 0.082, BASE_Z1, -0.046),
            _cyl_z(0.050, PEDESTAL_Z0, PEDESTAL_Z1),
            _ring_z(0.060, 0.028, YAW_SEAT_Z0, YAW_SEAT_Z1),
            _box_z(0.018, 0.060, BASE_Z1, -0.012).translate((0.038, 0.0, 0.0)),
            _box_z(0.018, 0.060, BASE_Z1, -0.012).translate((-0.038, 0.0, 0.0)),
            _box_z(0.060, 0.018, BASE_Z1, -0.012).translate((0.0, 0.038, 0.0)),
            _box_z(0.060, 0.018, BASE_Z1, -0.012).translate((0.0, -0.038, 0.0)),
            _bolt_heads_z(
                [
                    (-0.060, -0.044),
                    (-0.060, 0.044),
                    (0.060, -0.044),
                    (0.060, 0.044),
                ],
                radius=0.010,
                z0=BASE_Z1,
                z1=BASE_Z1 + 0.006,
            ),
        ]
    )

    body = body.cut(_cyl_z(0.024, PEDESTAL_Z0, 0.002))
    for x, y in [(-0.060, -0.044), (-0.060, 0.044), (0.060, -0.044), (0.060, 0.044)]:
        body = body.cut(_cyl_z(0.005, BASE_Z0, BASE_Z1 + 0.008).translate((x, y, 0.0)))
    return body


def _make_yaw_stage() -> cq.Workplane:
    body = _union_all(
        [
            _ring_z(0.059, 0.036, 0.000, 0.014),
            _ring_z(0.064, 0.046, 0.014, 0.028),
            _box_z(0.096, YAW_CHEEK_THICKNESS, 0.020, 0.104).translate((0.0, YAW_CHEEK_CENTER_Y, 0.0)),
            _box_z(0.096, YAW_CHEEK_THICKNESS, 0.020, 0.104).translate((0.0, -YAW_CHEEK_CENTER_Y, 0.0)),
            _cyl_y(0.030, YAW_CHEEK_INNER_Y, YAW_CHEEK_OUTER_Y).translate((0.0, 0.0, PITCH_AXIS_Z)),
            _cyl_y(0.030, -YAW_CHEEK_OUTER_Y, -YAW_CHEEK_INNER_Y).translate((0.0, 0.0, PITCH_AXIS_Z)),
            _cyl_y(0.031, 0.060, 0.062).translate((0.0, 0.0, PITCH_AXIS_Z)),
            _cyl_y(0.031, -0.062, -0.060).translate((0.0, 0.0, PITCH_AXIS_Z)),
        ]
    )
    body = body.cut(_box_z(0.084, 0.088, 0.024, 0.104))
    body = body.cut(_cyl_y(0.021, YAW_CHEEK_INNER_Y - 0.004, YAW_CHEEK_OUTER_Y).translate((0.0, 0.0, PITCH_AXIS_Z)))
    body = body.cut(_cyl_y(0.021, -YAW_CHEEK_OUTER_Y, -YAW_CHEEK_INNER_Y + 0.004).translate((0.0, 0.0, PITCH_AXIS_Z)))
    body = body.cut(_cyl_y(0.022, 0.060, 0.062).translate((0.0, 0.0, PITCH_AXIS_Z)))
    body = body.cut(_cyl_y(0.022, -0.062, -0.060).translate((0.0, 0.0, PITCH_AXIS_Z)))
    return body


def _make_yaw_cover() -> cq.Workplane:
    body = _union_all(
        [
            _cyl_y(0.037, 0.000, 0.004),
            _cyl_y(0.020, 0.004, 0.010),
            _bolt_heads_y(
                [
                    (-0.020, -0.020),
                    (-0.020, 0.020),
                    (0.020, -0.020),
                    (0.020, 0.020),
                ],
                radius=0.0045,
                y0=0.004,
                y1=0.008,
            ),
        ]
    )
    return body.cut(_cyl_y(0.010, 0.000, 0.010))


def _make_yaw_cover_seat() -> cq.Workplane:
    return _ring_y(0.031, 0.010, 0.000, 0.002)


def _make_pitch_stage() -> cq.Workplane:
    body = _union_all(
        [
            _cyl_y(PITCH_JOURNAL_RADIUS, PITCH_JOURNAL_Y0, PITCH_JOURNAL_Y1),
            _cyl_y(PITCH_SHOULDER_RADIUS, PITCH_SHOULDER_Y0, PITCH_SHOULDER_Y1),
            _cyl_y(PITCH_JOURNAL_RADIUS, -PITCH_JOURNAL_Y1, -PITCH_JOURNAL_Y0),
            _cyl_y(PITCH_SHOULDER_RADIUS, -PITCH_SHOULDER_Y1, -PITCH_SHOULDER_Y0),
            _ring_x(0.032, ROLL_JOURNAL_RADIUS, -ROLL_BEARING_X1, -ROLL_BEARING_X0),
            _ring_x(0.032, ROLL_JOURNAL_RADIUS, ROLL_BEARING_X0, ROLL_BEARING_X1),
            _ring_x(ROLL_BEARING_FACE_RADIUS, 0.022, -0.048, -ROLL_BEARING_X1),
            _ring_x(ROLL_BEARING_FACE_RADIUS, 0.022, ROLL_BEARING_X1, 0.048),
            _box_z(PITCH_FRAME_OUTER_X, PITCH_FRAME_OUTER_Y, PITCH_BRIDGE_Z0, PITCH_BRIDGE_Z1),
            _box_z(PITCH_FRAME_OUTER_X, PITCH_FRAME_OUTER_Y, -PITCH_BRIDGE_Z1, -PITCH_BRIDGE_Z0),
            _box_z(PITCH_FRAME_POST_X, PITCH_FRAME_OUTER_Y, -PITCH_BRIDGE_Z0, PITCH_BRIDGE_Z0).translate(
                (PITCH_FRAME_POST_OFFSET_X, 0.0, 0.0)
            ),
            _box_z(PITCH_FRAME_POST_X, PITCH_FRAME_OUTER_Y, -PITCH_BRIDGE_Z0, PITCH_BRIDGE_Z0).translate(
                (-PITCH_FRAME_POST_OFFSET_X, 0.0, 0.0)
            ),
        ]
    )
    return body.cut(_cyl_x(PITCH_CLEAR_RADIUS, -0.050, 0.050))


def _make_roll_stage() -> cq.Workplane:
    body = _union_all(
        [
            _cyl_x(ROLL_SHAFT_RADIUS, -0.072, 0.088),
            _cyl_x(0.018, -0.070, -0.064),
            _cyl_x(ROLL_JOURNAL_RADIUS, -ROLL_BEARING_X1, -ROLL_BEARING_X0),
            _cyl_x(ROLL_TRANSITION_RADIUS, -ROLL_BEARING_X0, -0.028),
            _cyl_x(ROLL_HUB_RADIUS, -0.028, 0.028),
            _cyl_x(ROLL_TRANSITION_RADIUS, 0.028, ROLL_BEARING_X0),
            _cyl_x(ROLL_JOURNAL_RADIUS, ROLL_BEARING_X0, ROLL_BEARING_X1),
            _cyl_x(0.018, ROLL_BEARING_X1, 0.072),
            _cyl_x(0.034, 0.072, 0.096),
        ]
    )
    body = body.cut(_cyl_x(0.011, 0.086, 0.096))
    body = body.cut(cq.Workplane("XY").box(0.048, 0.016, 0.008).translate((0.0, 0.0, 0.022)))
    body = body.cut(cq.Workplane("XY").box(0.048, 0.016, 0.008).translate((0.0, 0.0, -0.022)))
    return body


def _make_wrist_flange() -> cq.Workplane:
    bolt_points = [
        (0.000, 0.043),
        (0.037, 0.0215),
        (0.037, -0.0215),
        (0.000, -0.043),
        (-0.037, -0.0215),
        (-0.037, 0.0215),
    ]
    body = _union_all(
        [
            _ring_x(0.050, 0.011, -0.006, 0.006),
            _cyl_x(0.026, 0.006, 0.014),
            _bolt_heads_x(bolt_points, radius=0.0045, x0=0.002, x1=0.006),
        ]
    )
    for y, z in bolt_points:
        body = body.cut(_cyl_x(0.0035, -0.006, 0.006).translate((0.0, y, z)))
    return body


def _make_roll_collar() -> cq.Workplane:
    body = _union_all(
        [
            _ring_x(0.024, 0.0145, 0.000, 0.006),
            _bolt_heads_x(
                [
                    (0.0, 0.024),
                    (0.0, -0.024),
                ],
                radius=0.005,
                x0=0.002,
                x1=0.006,
            ),
        ]
    )
    body = body.cut(cq.Workplane("XY").box(0.008, 0.054, 0.012).translate((0.004, 0.0, 0.024)))
    return body


def _make_roll_bearing_cap() -> cq.Workplane:
    body = _union_all(
        [
            _ring_x(0.036, 0.0215, 0.000, 0.006),
            _ring_x(0.028, 0.0215, 0.006, 0.012),
            _bolt_heads_x(
                [
                    (0.024, 0.024),
                    (0.024, -0.024),
                    (-0.024, 0.024),
                    (-0.024, -0.024),
                ],
                radius=0.0045,
                x0=0.0015,
                x1=0.0055,
            ),
        ]
    )
    for y, z in [(0.024, 0.024), (0.024, -0.024), (-0.024, 0.024), (-0.024, -0.024)]:
        body = body.cut(_cyl_x(0.0028, 0.000, 0.012).translate((0.0, y, z)))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_pitch_roll_wrist_study", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.14, 0.15, 0.17, 1.0))
    machined_alloy = model.material("machined_alloy", rgba=(0.61, 0.64, 0.67, 1.0))
    hardened_steel = model.material("hardened_steel", rgba=(0.46, 0.48, 0.51, 1.0))

    base_support = model.part("base_support")
    base_support.visual(
        mesh_from_cadquery(_make_base_support(), "base_support.obj", assets=ASSETS),
        material=painted_steel,
        name="base_shell",
    )
    base_support.inertial = Inertial.from_geometry(
        Box((0.180, 0.140, 0.072)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_make_yaw_stage(), "yaw_stage.obj", assets=ASSETS),
        material=dark_oxide,
        name="yaw_shell",
    )
    yaw_stage.visual(
        mesh_from_cadquery(
            _ring_y(0.031, 0.022, 0.060, 0.062).translate((0.0, 0.0, PITCH_AXIS_Z)),
            "yaw_cover_land_pos.obj",
            assets=ASSETS,
        ),
        material=dark_oxide,
        name="yaw_cover_land_pos",
    )
    yaw_stage.visual(
        mesh_from_cadquery(
            _ring_y(0.031, 0.022, -0.062, -0.060).translate((0.0, 0.0, PITCH_AXIS_Z)),
            "yaw_cover_land_neg.obj",
            assets=ASSETS,
        ),
        material=dark_oxide,
        name="yaw_cover_land_neg",
    )
    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.128, 0.136, 0.112)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    yaw_cover_pos = model.part("yaw_cover_pos")
    yaw_cover_pos.visual(
        mesh_from_cadquery(_make_yaw_cover(), "yaw_cover_pos.obj", assets=ASSETS),
        material=machined_alloy,
        name="yaw_cover_pos_shell",
    )
    yaw_cover_pos.visual(
        mesh_from_cadquery(_make_yaw_cover_seat(), "yaw_cover_pos_seat.obj", assets=ASSETS),
        material=machined_alloy,
        name="yaw_cover_pos_seat",
    )
    yaw_cover_pos.inertial = Inertial.from_geometry(
        Box((0.074, 0.014, 0.074)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
    )

    yaw_cover_neg = model.part("yaw_cover_neg")
    yaw_cover_neg.visual(
        mesh_from_cadquery(_make_yaw_cover(), "yaw_cover_neg.obj", assets=ASSETS),
        material=machined_alloy,
        name="yaw_cover_neg_shell",
    )
    yaw_cover_neg.visual(
        mesh_from_cadquery(_make_yaw_cover_seat(), "yaw_cover_neg_seat.obj", assets=ASSETS),
        material=machined_alloy,
        name="yaw_cover_neg_seat",
    )
    yaw_cover_neg.inertial = Inertial.from_geometry(
        Box((0.074, 0.014, 0.074)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
    )

    pitch_stage = model.part("pitch_stage")
    pitch_stage.visual(
        mesh_from_cadquery(_make_pitch_stage(), "pitch_stage.obj", assets=ASSETS),
        material=painted_steel,
        name="pitch_shell",
    )
    pitch_stage.inertial = Inertial.from_geometry(
        Box((0.100, 0.120, 0.068)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    roll_bearing_cap_pos = model.part("roll_bearing_cap_pos")
    roll_bearing_cap_pos.visual(
        mesh_from_cadquery(_make_roll_bearing_cap(), "roll_bearing_cap_pos.obj", assets=ASSETS),
        material=machined_alloy,
        name="roll_bearing_cap_pos_shell",
    )
    roll_bearing_cap_pos.inertial = Inertial.from_geometry(
        Box((0.012, 0.072, 0.072)),
        mass=0.16,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
    )

    roll_bearing_cap_neg = model.part("roll_bearing_cap_neg")
    roll_bearing_cap_neg.visual(
        mesh_from_cadquery(_make_roll_bearing_cap(), "roll_bearing_cap_neg.obj", assets=ASSETS),
        material=machined_alloy,
        name="roll_bearing_cap_neg_shell",
    )
    roll_bearing_cap_neg.inertial = Inertial.from_geometry(
        Box((0.012, 0.072, 0.072)),
        mass=0.16,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
    )

    roll_stage = model.part("roll_stage")
    roll_stage.visual(
        mesh_from_cadquery(_make_roll_stage(), "roll_stage.obj", assets=ASSETS),
        material=hardened_steel,
        name="roll_shell",
    )
    roll_stage.inertial = Inertial.from_geometry(
        Box((0.166, 0.068, 0.068)),
        mass=1.2,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    wrist_flange = model.part("wrist_flange")
    wrist_flange.visual(
        mesh_from_cadquery(_make_wrist_flange(), "wrist_flange.obj", assets=ASSETS),
        material=machined_alloy,
        name="wrist_flange_shell",
    )
    wrist_flange.inertial = Inertial.from_geometry(
        Box((0.020, 0.100, 0.100)),
        mass=0.42,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
    )

    roll_collar = model.part("roll_collar")
    roll_collar.visual(
        mesh_from_cadquery(_make_roll_collar(), "roll_collar.obj", assets=ASSETS),
        material=machined_alloy,
        name="roll_collar_shell",
    )
    roll_collar.inertial = Inertial.from_geometry(
        Box((0.020, 0.084, 0.084)),
        mass=0.22,
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.CONTINUOUS,
        parent=base_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0),
    )
    model.articulation(
        "yaw_to_cover_pos",
        ArticulationType.FIXED,
        parent=yaw_stage,
        child=yaw_cover_pos,
        origin=Origin(xyz=(0.0, YAW_COVER_FACE_Y, PITCH_AXIS_Z)),
    )
    model.articulation(
        "yaw_to_cover_neg",
        ArticulationType.FIXED,
        parent=yaw_stage,
        child=yaw_cover_neg,
        origin=Origin(xyz=(0.0, -YAW_COVER_FACE_Y, PITCH_AXIS_Z), rpy=(0.0, 0.0, math.pi)),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_stage,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.4, lower=-1.0, upper=1.0),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.CONTINUOUS,
        parent=pitch_stage,
        child=roll_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0),
    )
    model.articulation(
        "pitch_to_roll_bearing_cap_pos",
        ArticulationType.FIXED,
        parent=pitch_stage,
        child=roll_bearing_cap_pos,
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
    )
    model.articulation(
        "pitch_to_roll_bearing_cap_neg",
        ArticulationType.FIXED,
        parent=pitch_stage,
        child=roll_bearing_cap_neg,
        origin=Origin(xyz=(-0.048, 0.0, 0.0), rpy=(0.0, math.pi, 0.0)),
    )
    model.articulation(
        "roll_to_flange",
        ArticulationType.FIXED,
        parent=roll_stage,
        child=wrist_flange,
        origin=Origin(xyz=(0.102, 0.0, 0.0)),
    )
    model.articulation(
        "roll_to_collar",
        ArticulationType.FIXED,
        parent=roll_stage,
        child=roll_collar,
        origin=Origin(xyz=(-0.076, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_support = object_model.get_part("base_support")
    yaw_stage = object_model.get_part("yaw_stage")
    yaw_cover_pos = object_model.get_part("yaw_cover_pos")
    yaw_cover_neg = object_model.get_part("yaw_cover_neg")
    pitch_stage = object_model.get_part("pitch_stage")
    roll_bearing_cap_pos = object_model.get_part("roll_bearing_cap_pos")
    roll_bearing_cap_neg = object_model.get_part("roll_bearing_cap_neg")
    roll_stage = object_model.get_part("roll_stage")
    wrist_flange = object_model.get_part("wrist_flange")
    roll_collar = object_model.get_part("roll_collar")

    yaw_cover_land_pos = yaw_stage.get_visual("yaw_cover_land_pos")
    yaw_cover_land_neg = yaw_stage.get_visual("yaw_cover_land_neg")
    yaw_cover_pos_seat = yaw_cover_pos.get_visual("yaw_cover_pos_seat")
    yaw_cover_neg_seat = yaw_cover_neg.get_visual("yaw_cover_neg_seat")

    base_to_yaw = object_model.get_articulation("base_to_yaw")
    yaw_to_cover_pos = object_model.get_articulation("yaw_to_cover_pos")
    yaw_to_cover_neg = object_model.get_articulation("yaw_to_cover_neg")
    yaw_to_pitch = object_model.get_articulation("yaw_to_pitch")
    pitch_to_roll = object_model.get_articulation("pitch_to_roll")
    pitch_to_roll_bearing_cap_pos = object_model.get_articulation("pitch_to_roll_bearing_cap_pos")
    pitch_to_roll_bearing_cap_neg = object_model.get_articulation("pitch_to_roll_bearing_cap_neg")
    roll_to_flange = object_model.get_articulation("roll_to_flange")
    roll_to_collar = object_model.get_articulation("roll_to_collar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "wrist_joint_types",
        (
            base_to_yaw.articulation_type == ArticulationType.CONTINUOUS
            and yaw_to_pitch.articulation_type == ArticulationType.REVOLUTE
            and pitch_to_roll.articulation_type == ArticulationType.CONTINUOUS
            and yaw_to_cover_pos.articulation_type == ArticulationType.FIXED
            and yaw_to_cover_neg.articulation_type == ArticulationType.FIXED
            and pitch_to_roll_bearing_cap_pos.articulation_type == ArticulationType.FIXED
            and pitch_to_roll_bearing_cap_neg.articulation_type == ArticulationType.FIXED
            and roll_to_flange.articulation_type == ArticulationType.FIXED
            and roll_to_collar.articulation_type == ArticulationType.FIXED
        ),
        details=(
            f"types={[base_to_yaw.articulation_type, yaw_to_pitch.articulation_type, pitch_to_roll.articulation_type]}"
        ),
    )
    ctx.check(
        "wrist_joint_axes",
        tuple(base_to_yaw.axis) == (0.0, 0.0, 1.0)
        and tuple(yaw_to_pitch.axis) == (0.0, 1.0, 0.0)
        and tuple(pitch_to_roll.axis) == (1.0, 0.0, 0.0),
        details=f"axes={base_to_yaw.axis},{yaw_to_pitch.axis},{pitch_to_roll.axis}",
    )

    ctx.expect_contact(yaw_stage, base_support, name="yaw_stage_seats_on_base")
    ctx.expect_overlap(yaw_stage, base_support, axes="xy", min_overlap=0.090, name="yaw_stage_footprint_on_base")
    ctx.expect_origin_distance(yaw_stage, base_support, axes="xy", max_dist=0.001, name="yaw_axis_centered_on_base")

    ctx.expect_contact(yaw_cover_pos, yaw_stage, name="positive_cover_seated")
    ctx.expect_gap(
        yaw_cover_pos,
        yaw_stage,
        axis="y",
        positive_elem=yaw_cover_pos_seat,
        negative_elem=yaw_cover_land_pos,
        max_gap=0.0005,
        max_penetration=0.00005,
        name="positive_cover_flush_mount",
    )
    ctx.expect_overlap(yaw_cover_pos, yaw_stage, axes="xz", min_overlap=0.050, name="positive_cover_overlap_patch")

    ctx.expect_contact(yaw_cover_neg, yaw_stage, name="negative_cover_seated")
    ctx.expect_gap(
        yaw_stage,
        yaw_cover_neg,
        axis="y",
        positive_elem=yaw_cover_land_neg,
        negative_elem=yaw_cover_neg_seat,
        max_gap=0.0005,
        max_penetration=0.00005,
        name="negative_cover_flush_mount",
    )
    ctx.expect_overlap(yaw_cover_neg, yaw_stage, axes="xz", min_overlap=0.050, name="negative_cover_overlap_patch")

    ctx.expect_contact(pitch_stage, yaw_stage, contact_tol=0.0005, name="pitch_trunnions_supported")
    ctx.expect_origin_distance(pitch_stage, yaw_stage, axes="xy", max_dist=0.001, name="pitch_axis_centered_in_yaw")
    ctx.expect_origin_gap(
        pitch_stage,
        base_support,
        axis="z",
        min_gap=0.070,
        max_gap=0.078,
        name="pitch_axis_above_base",
    )

    ctx.expect_contact(roll_bearing_cap_pos, pitch_stage, name="positive_roll_bearing_cap_seated")
    ctx.expect_gap(
        roll_bearing_cap_pos,
        pitch_stage,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        name="positive_roll_bearing_cap_flush_mount",
    )
    ctx.expect_overlap(
        roll_bearing_cap_pos,
        pitch_stage,
        axes="yz",
        min_overlap=0.060,
        name="positive_roll_bearing_cap_cover_patch",
    )

    ctx.expect_contact(roll_bearing_cap_neg, pitch_stage, name="negative_roll_bearing_cap_seated")
    ctx.expect_gap(
        pitch_stage,
        roll_bearing_cap_neg,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.00001,
        name="negative_roll_bearing_cap_flush_mount",
    )
    ctx.expect_overlap(
        roll_bearing_cap_neg,
        pitch_stage,
        axes="yz",
        min_overlap=0.060,
        name="negative_roll_bearing_cap_cover_patch",
    )

    ctx.expect_overlap(roll_stage, pitch_stage, axes="yz", min_overlap=0.050, name="roll_stage_supported_by_pitch")
    ctx.expect_overlap(roll_stage, pitch_stage, axes="yz", min_overlap=0.050, name="roll_and_pitch_share_bearing_envelope")
    ctx.expect_origin_distance(roll_stage, pitch_stage, axes="yz", max_dist=0.001, name="roll_axis_intersects_pitch_axis")

    ctx.expect_contact(wrist_flange, roll_stage, name="wrist_flange_seated_on_roll_hub")
    ctx.expect_gap(
        wrist_flange,
        roll_stage,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.00001,
        name="wrist_flange_flush_to_roll_shoulder",
    )
    ctx.expect_overlap(wrist_flange, roll_stage, axes="yz", min_overlap=0.060, name="wrist_flange_concentric_with_roll_hub")

    ctx.expect_contact(roll_collar, roll_stage, name="roll_collar_retains_roll_stage")
    ctx.expect_origin_distance(
        roll_collar,
        roll_stage,
        axes="yz",
        max_dist=0.001,
        name="roll_collar_concentric_on_shaft",
    )

    with ctx.pose({base_to_yaw: 1.1}):
        ctx.expect_contact(yaw_stage, base_support, name="yaw_contact_persists_at_yaw_pose")
        ctx.expect_contact(yaw_cover_pos, yaw_stage, name="positive_cover_remains_seated_at_yaw_pose")
        ctx.expect_contact(yaw_cover_neg, yaw_stage, name="negative_cover_remains_seated_at_yaw_pose")

    with ctx.pose({yaw_to_pitch: 0.80}):
        ctx.expect_contact(pitch_stage, yaw_stage, contact_tol=0.0005, name="pitch_contact_persists_at_positive_pitch")
        ctx.expect_contact(roll_bearing_cap_pos, pitch_stage, name="positive_bearing_cap_contact_persists_at_positive_pitch")
        ctx.expect_contact(roll_bearing_cap_neg, pitch_stage, name="negative_bearing_cap_contact_persists_at_positive_pitch")
        ctx.expect_overlap(roll_stage, pitch_stage, axes="yz", min_overlap=0.050, name="roll_contact_persists_at_positive_pitch")
        ctx.expect_contact(wrist_flange, roll_stage, name="wrist_flange_contact_persists_at_positive_pitch")
        ctx.expect_gap(
            pitch_stage,
            base_support,
            axis="z",
            min_gap=0.008,
            max_gap=0.120,
            name="pitched_stage_clears_base",
        )

    with ctx.pose({yaw_to_pitch: -0.80, pitch_to_roll: 1.30}):
        ctx.expect_contact(pitch_stage, yaw_stage, contact_tol=0.0005, name="pitch_contact_persists_at_negative_pitch")
        ctx.expect_contact(roll_bearing_cap_pos, pitch_stage, name="positive_bearing_cap_contact_persists_at_roll_pose")
        ctx.expect_contact(roll_bearing_cap_neg, pitch_stage, name="negative_bearing_cap_contact_persists_at_roll_pose")
        ctx.expect_overlap(roll_stage, pitch_stage, axes="yz", min_overlap=0.050, name="roll_contact_persists_at_roll_pose")
        ctx.expect_contact(wrist_flange, roll_stage, name="wrist_flange_contact_persists_at_roll_pose")
        ctx.expect_contact(roll_collar, roll_stage, name="collar_contact_persists_at_roll_pose")
        ctx.expect_origin_distance(
            roll_stage,
            pitch_stage,
            axes="yz",
            max_dist=0.001,
            name="roll_axis_remains_concentric_in_combined_pose",
        )

    with ctx.pose({base_to_yaw: -0.65, yaw_to_pitch: 0.55, pitch_to_roll: -1.70}):
        ctx.expect_contact(yaw_stage, base_support, name="yaw_support_persists_in_combined_pose")
        ctx.expect_contact(pitch_stage, yaw_stage, contact_tol=0.0005, name="pitch_support_persists_in_combined_pose")
        ctx.expect_contact(roll_bearing_cap_pos, pitch_stage, name="positive_bearing_cap_support_persists_in_combined_pose")
        ctx.expect_contact(roll_bearing_cap_neg, pitch_stage, name="negative_bearing_cap_support_persists_in_combined_pose")
        ctx.expect_overlap(roll_stage, pitch_stage, axes="yz", min_overlap=0.050, name="roll_support_persists_in_combined_pose")
        ctx.expect_contact(wrist_flange, roll_stage, name="wrist_flange_support_persists_in_combined_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
