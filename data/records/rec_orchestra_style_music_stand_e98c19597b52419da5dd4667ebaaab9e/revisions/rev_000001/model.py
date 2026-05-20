from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.62
BASE_D = 0.40
BASE_T = 0.026
POST_Y = 0.10
POST_SPACING = 0.19
POST_R = 0.018
POST_LEN = 1.18
POST_BASE_Z = 0.061

CARRIAGE_Z0 = 0.50
SLEEVE_LEN = 0.23
SLEEVE_OUTER_R = 0.030
SLEEVE_INNER_R = 0.0205

HINGE_Y = -0.155
HINGE_Z = 0.245
AXLE_R = 0.0078
AXLE_LEN = 0.156

BOSS_CENTER_X = 0.137
BOSS_CENTER_Z = 0.105
BOSS_LEN = 0.044
BOSS_OUTER_R = 0.018
BOSS_INNER_R = 0.0085
GUIDE_PAD_R = 0.0015
GUIDE_PAD_OFFSET = POST_R + GUIDE_PAD_R


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_cylinder_z(radius: float, length: float, center: tuple[float, float, float]):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1], center[2] - 0.5 * length))
    )


def _cq_cylinder_x(radius: float, length: float, center: tuple[float, float, float]):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _cq_tube_z(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
):
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((center[0], center[1], center[2] - 0.5 * length))
    )


def _cq_tube_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
):
    return _cq_cylinder_x(outer_radius, length, center).cut(
        _cq_cylinder_x(inner_radius, length + 0.004, center)
    )


def _build_floor_base():
    base = _cq_box((BASE_W, BASE_D, BASE_T), (0.0, 0.0, 0.5 * BASE_T))
    rear_beam = _cq_box((0.30, 0.12, 0.050), (0.0, POST_Y, BASE_T + 0.025))
    front_weight = _cq_box((0.36, 0.18, 0.010), (0.0, -0.055, BASE_T + 0.005))
    return base.union(rear_beam).union(front_weight)


def _build_carriage_bridge():
    bridge = _cq_box((0.140, 0.050, 0.066), (0.0, -0.002, 0.112))
    neck = _cq_box((0.060, 0.070, 0.060), (0.0, -0.026, 0.168))
    left_arm = _cq_box((0.022, 0.165, 0.070), (-0.052, -0.082, 0.206))
    right_arm = _cq_box((0.022, 0.165, 0.070), (0.052, -0.082, 0.206))
    left_cheek = _cq_box((0.018, 0.024, 0.042), (-0.074, HINGE_Y, HINGE_Z))
    right_cheek = _cq_box((0.018, 0.024, 0.042), (0.074, HINGE_Y, HINGE_Z))
    return bridge.union(neck).union(left_arm).union(right_arm).union(left_cheek).union(right_cheek)


def _build_clamp_boss():
    boss = _cq_cylinder_x(BOSS_OUTER_R, BOSS_LEN, (BOSS_CENTER_X, 0.0, BOSS_CENTER_Z))
    return boss.cut(
        _cq_cylinder_x(BOSS_INNER_R, BOSS_LEN + 0.004, (BOSS_CENTER_X, 0.0, BOSS_CENTER_Z))
    )


def _build_desk():
    rest_tilt_deg = -12.0
    panel = _cq_box((0.74, 0.012, 0.46), (0.0, -0.032, 0.250))
    left_flange = _cq_box((0.016, 0.030, 0.44), (-0.362, -0.022, 0.248))
    right_flange = _cq_box((0.016, 0.030, 0.44), (0.362, -0.022, 0.248))
    shelf = _cq_box((0.64, 0.058, 0.015), (0.0, -0.060, 0.030))
    fence = _cq_box((0.64, 0.008, 0.026), (0.0, -0.086, 0.039))
    stiffener = _cq_box((0.14, 0.026, 0.28), (0.0, -0.022, 0.184))
    left_bracket = _cq_box((0.050, 0.026, 0.100), (-0.170, -0.017, 0.070))
    right_bracket = _cq_box((0.050, 0.026, 0.100), (0.170, -0.017, 0.070))

    desk = (
        panel.union(left_flange)
        .union(right_flange)
        .union(shelf)
        .union(fence)
        .union(stiffener)
        .union(left_bracket)
        .union(right_bracket)
    )
    return desk.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), rest_tilt_deg)


def _build_knob():
    knob = _cq_cylinder_x(0.011, 0.018, (0.011, 0.0, 0.0))
    for angle_deg in (0.0, 72.0, 144.0, 216.0, 288.0):
        angle = math.radians(angle_deg)
        knob = knob.union(
            _cq_cylinder_x(
                0.010,
                0.014,
                (
                    0.008,
                    0.018 * math.cos(angle),
                    0.018 * math.sin(angle),
                ),
            )
        )
    knob = knob.union(_cq_cylinder_x(0.0040, 0.036, (-0.016, 0.0, 0.0)))
    knob = knob.union(_cq_cylinder_x(0.0032, 0.008, (-0.038, 0.0, 0.0)))
    return knob


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conductors_music_stand")

    satin_black = model.material("satin_black", rgba=(0.15, 0.15, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    phenolic = model.material("phenolic", rgba=(0.10, 0.10, 0.11, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_build_floor_base(), "floor_base"),
        material=satin_black,
        name="floor_base",
    )
    stand.visual(
        Cylinder(radius=POST_R, length=POST_LEN),
        origin=Origin(xyz=(-0.5 * POST_SPACING, POST_Y, POST_BASE_Z + 0.5 * POST_LEN)),
        material=graphite,
        name="post_0",
    )
    stand.visual(
        Cylinder(radius=POST_R, length=POST_LEN),
        origin=Origin(xyz=(0.5 * POST_SPACING, POST_Y, POST_BASE_Z + 0.5 * POST_LEN)),
        material=graphite,
        name="post_1",
    )
    stand.visual(
        mesh_from_cadquery(
            _cq_box((0.25, 0.028, 0.022), (0.0, POST_Y + 0.010, POST_BASE_Z + POST_LEN - 0.015)),
            "mast_tie",
        ),
        material=satin_black,
        name="mast_tie",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(
            _cq_tube_z(
                SLEEVE_OUTER_R,
                SLEEVE_INNER_R,
                SLEEVE_LEN,
                (-0.5 * POST_SPACING, 0.0, 0.5 * SLEEVE_LEN),
            ),
            "carriage_sleeve_0",
        ),
        material=satin_black,
        name="carriage_sleeve_0",
    )
    carriage.visual(
        mesh_from_cadquery(
            _cq_tube_z(
                SLEEVE_OUTER_R,
                SLEEVE_INNER_R,
                SLEEVE_LEN,
                (0.5 * POST_SPACING, 0.0, 0.5 * SLEEVE_LEN),
            ),
            "carriage_sleeve_1",
        ),
        material=satin_black,
        name="carriage_sleeve_1",
    )
    carriage.visual(
        Box((0.140, 0.050, 0.066)),
        origin=Origin(xyz=(0.0, -0.002, 0.112)),
        material=satin_black,
        name="carriage_bridge",
    )
    carriage.visual(
        Box((0.060, 0.070, 0.060)),
        origin=Origin(xyz=(0.0, -0.026, 0.168)),
        material=satin_black,
        name="carriage_neck",
    )
    carriage.visual(
        Cylinder(
            radius=0.009,
            length=_distance((-0.034, -0.030, 0.176), (-0.074, HINGE_Y, HINGE_Z)),
        ),
        origin=Origin(
            xyz=_midpoint((-0.034, -0.030, 0.176), (-0.074, HINGE_Y, HINGE_Z)),
            rpy=_rpy_for_cylinder((-0.034, -0.030, 0.176), (-0.074, HINGE_Y, HINGE_Z)),
        ),
        material=satin_black,
        name="arm_0",
    )
    carriage.visual(
        Cylinder(
            radius=0.009,
            length=_distance((0.034, -0.030, 0.176), (0.074, HINGE_Y, HINGE_Z)),
        ),
        origin=Origin(
            xyz=_midpoint((0.034, -0.030, 0.176), (0.074, HINGE_Y, HINGE_Z)),
            rpy=_rpy_for_cylinder((0.034, -0.030, 0.176), (0.074, HINGE_Y, HINGE_Z)),
        ),
        material=satin_black,
        name="arm_1",
    )
    carriage.visual(
        Box((0.018, 0.024, 0.042)),
        origin=Origin(xyz=(-0.074, HINGE_Y, HINGE_Z)),
        material=satin_black,
        name="cheek_0",
    )
    carriage.visual(
        Box((0.018, 0.024, 0.042)),
        origin=Origin(xyz=(0.074, HINGE_Y, HINGE_Z)),
        material=satin_black,
        name="cheek_1",
    )
    carriage.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_axle",
    )
    carriage.visual(
        mesh_from_cadquery(_build_clamp_boss(), "clamp_boss"),
        material=satin_black,
        name="clamp_boss",
    )
    for sleeve_index, sleeve_x in enumerate((-0.5 * POST_SPACING, 0.5 * POST_SPACING)):
        for pad_index, offset_sign in enumerate((-1.0, 1.0)):
            carriage.visual(
                Cylinder(radius=GUIDE_PAD_R, length=SLEEVE_LEN - 0.020),
                origin=Origin(
                    xyz=(
                        sleeve_x + offset_sign * GUIDE_PAD_OFFSET,
                        0.0,
                        0.5 * SLEEVE_LEN,
                    )
                ),
                material=graphite,
                name=f"guide_pad_{sleeve_index}_{pad_index}",
            )

    desk = model.part("desk")
    desk_rest_tilt = math.radians(-12.0)
    desk.visual(
        Box((0.74, 0.012, 0.46)),
        origin=Origin(xyz=(0.0, -0.032, 0.250), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="desk_panel",
    )
    desk.visual(
        Box((0.016, 0.030, 0.44)),
        origin=Origin(xyz=(-0.362, -0.022, 0.248), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="flange_0",
    )
    desk.visual(
        Box((0.016, 0.030, 0.44)),
        origin=Origin(xyz=(0.362, -0.022, 0.248), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="flange_1",
    )
    desk.visual(
        Box((0.64, 0.058, 0.015)),
        origin=Origin(xyz=(0.0, -0.060, 0.030), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="shelf",
    )
    desk.visual(
        Box((0.64, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, -0.086, 0.039), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="fence",
    )
    desk.visual(
        Box((0.14, 0.036, 0.28)),
        origin=Origin(xyz=(0.0, -0.026, 0.184), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=satin_black,
        name="stiffener",
    )
    desk.visual(
        Box((0.040, 0.026, 0.100)),
        origin=Origin(xyz=(-0.055, -0.017, 0.062), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=satin_black,
        name="bracket_0",
    )
    desk.visual(
        Box((0.040, 0.026, 0.100)),
        origin=Origin(xyz=(0.055, -0.017, 0.062), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=satin_black,
        name="bracket_1",
    )
    desk.visual(
        Box((0.118, 0.020, 0.032)),
        origin=Origin(xyz=(0.0, -0.014, 0.028), rpy=(desk_rest_tilt, 0.0, 0.0)),
        material=satin_black,
        name="hinge_bridge",
    )
    desk.visual(
        mesh_from_cadquery(_cq_tube_x(0.013, 0.0076, 0.118, (0.0, 0.0, 0.0)), "hinge_tube"),
        material=satin_black,
        name="hinge_tube",
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        mesh_from_cadquery(_build_knob(), "clamp_knob"),
        material=phenolic,
        name="clamp_knob",
    )
    clamp_knob.visual(
        Box((0.007, 0.004, 0.004)),
        origin=Origin(xyz=(0.013, 0.0, 0.014)),
        material=graphite,
        name="index_mark",
    )

    model.articulation(
        "stand_to_carriage",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=carriage,
        origin=Origin(xyz=(0.0, POST_Y, CARRIAGE_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.22,
            lower=0.0,
            upper=0.40,
        ),
    )
    model.articulation(
        "carriage_to_desk",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=desk,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.4,
            lower=-0.35,
            upper=0.60,
        ),
    )
    model.articulation(
        "carriage_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=clamp_knob,
        origin=Origin(xyz=(BOSS_CENTER_X + 0.5 * BOSS_LEN, 0.0, BOSS_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    carriage = object_model.get_part("carriage")
    desk = object_model.get_part("desk")
    clamp_knob = object_model.get_part("clamp_knob")

    slide = object_model.get_articulation("stand_to_carriage")
    tilt = object_model.get_articulation("carriage_to_desk")
    knob_spin = object_model.get_articulation("carriage_to_clamp_knob")

    ctx.allow_overlap(
        carriage,
        desk,
        elem_a="hinge_axle",
        elem_b="hinge_tube",
        reason="The desk pivots around the carriage axle, which is intentionally nested inside the hinge tube.",
    )
    ctx.allow_overlap(
        carriage,
        clamp_knob,
        elem_a="carriage_sleeve_1",
        elem_b="clamp_knob",
        reason="The clamp screw is represented passing through the collar wall proxy on the right sleeve.",
    )

    ctx.expect_within(
        stand,
        carriage,
        axes="xy",
        inner_elem="post_0",
        outer_elem="carriage_sleeve_0",
        margin=0.0015,
        name="left post stays centered in its sleeve",
    )
    ctx.expect_within(
        stand,
        carriage,
        axes="xy",
        inner_elem="post_1",
        outer_elem="carriage_sleeve_1",
        margin=0.0015,
        name="right post stays centered in its sleeve",
    )
    ctx.expect_overlap(
        stand,
        carriage,
        axes="z",
        elem_a="post_0",
        elem_b="carriage_sleeve_0",
        min_overlap=0.18,
        name="left sleeve retains deep insertion at rest",
    )
    ctx.expect_overlap(
        stand,
        carriage,
        axes="z",
        elem_a="post_1",
        elem_b="carriage_sleeve_1",
        min_overlap=0.18,
        name="right sleeve retains deep insertion at rest",
    )

    slide_limits = slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        rest_pos = ctx.part_world_position(carriage)
        with ctx.pose({slide: slide_limits.upper}):
            ctx.expect_within(
                stand,
                carriage,
                axes="xy",
                inner_elem="post_0",
                outer_elem="carriage_sleeve_0",
                margin=0.0015,
                name="left post stays centered at max height",
            )
            ctx.expect_within(
                stand,
                carriage,
                axes="xy",
                inner_elem="post_1",
                outer_elem="carriage_sleeve_1",
                margin=0.0015,
                name="right post stays centered at max height",
            )
            ctx.expect_overlap(
                stand,
                carriage,
                axes="z",
                elem_a="post_0",
                elem_b="carriage_sleeve_0",
                min_overlap=0.14,
                name="left sleeve remains captured at max height",
            )
            ctx.expect_overlap(
                stand,
                carriage,
                axes="z",
                elem_a="post_1",
                elem_b="carriage_sleeve_1",
                min_overlap=0.14,
                name="right sleeve remains captured at max height",
            )
            high_pos = ctx.part_world_position(carriage)

        ctx.check(
            "carriage rises on the twin-post mast",
            rest_pos is not None and high_pos is not None and high_pos[2] > rest_pos[2] + 0.30,
            details=f"rest={rest_pos}, high={high_pos}",
        )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None and tilt_limits.lower is not None:
        rest_center = _aabb_center(ctx.part_element_world_aabb(desk, elem="desk_panel"))
        with ctx.pose({tilt: tilt_limits.upper}):
            upper_center = _aabb_center(ctx.part_element_world_aabb(desk, elem="desk_panel"))
        with ctx.pose({tilt: tilt_limits.lower}):
            lower_center = _aabb_center(ctx.part_element_world_aabb(desk, elem="desk_panel"))

        ctx.check(
            "desk tilts backward at the upper stop",
            rest_center is not None
            and upper_center is not None
            and upper_center[1] > rest_center[1] + 0.05,
            details=f"rest={rest_center}, upper={upper_center}",
        )
        ctx.check(
            "desk tilts forward at the lower stop",
            rest_center is not None
            and lower_center is not None
            and lower_center[1] < rest_center[1] - 0.04,
            details=f"rest={rest_center}, lower={lower_center}",
        )

    mark_rest = _aabb_center(ctx.part_element_world_aabb(clamp_knob, elem="index_mark"))
    with ctx.pose({knob_spin: math.pi / 2.0}):
        mark_turn = _aabb_center(ctx.part_element_world_aabb(clamp_knob, elem="index_mark"))
    ctx.check(
        "clamp knob spins on its threaded axis",
        mark_rest is not None
        and mark_turn is not None
        and abs(mark_turn[1] - mark_rest[1]) > 0.010
        and abs(mark_turn[2] - mark_rest[2]) > 0.010,
        details=f"rest={mark_rest}, turned={mark_turn}",
    )

    return ctx.report()


object_model = build_object_model()
