from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def _cylinder_origin_between(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[Origin, float]:
    """Return an Origin and length for a cylinder whose local +Z spans start->end."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be distinct")

    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_pan_tilt_tower")

    dark_steel = model.material("dark_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    tower_steel = model.material("tower_steel", rgba=(0.22, 0.24, 0.25, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.015, 0.016, 0.017, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.92, 0.66, 0.10, 1.0))
    lamp_coat = model.material("lamp_coat", rgba=(0.12, 0.14, 0.16, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.44, 0.76, 0.95, 0.55))
    rubber = model.material("rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.78, 0.78, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.070, length=1.050),
        origin=Origin(xyz=(0.0, 0.0, 0.570)),
        material=tower_steel,
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.115, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=tower_steel,
        name="mast_foot",
    )
    mast.visual(
        Cylinder(radius=0.205, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.125)),
        material=dark_steel,
        name="top_flange",
    )
    mast.visual(
        Cylinder(radius=0.185, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 1.2025)),
        material=bearing_black,
        name="lower_bearing",
    )
    mast.visual(
        Cylinder(radius=0.225, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 1.2575)),
        material=dark_steel,
        name="bearing_lip",
    )
    for idx, (sx, sy) in enumerate(
        ((0.30, 0.30), (-0.30, 0.30), (-0.30, -0.30), (0.30, -0.30))
    ):
        mast.visual(
            Box((0.125, 0.125, 0.030)),
            origin=Origin(xyz=(sx, sy, 0.060)),
            material=dark_steel,
            name=f"outrigger_pad_{idx}",
        )
        start = (sx * 0.90, sy * 0.90, 0.080)
        end = (sx * 0.28, sy * 0.28, 1.105)
        brace_origin, brace_length = _cylinder_origin_between(start, end)
        mast.visual(
            Cylinder(radius=0.018, length=brace_length),
            origin=brace_origin,
            material=tower_steel,
            name=f"diagonal_brace_{idx}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.205, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=bearing_black,
        name="turntable_disk",
    )
    yoke.visual(
        Cylinder(radius=0.135, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=dark_steel,
        name="rotary_housing",
    )
    yoke.visual(
        Cylinder(radius=0.245, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=dark_steel,
        name="upper_bearing_ring",
    )

    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.660, 0.340, 0.430),
            span_width=0.430,
            trunnion_diameter=0.104,
            trunnion_center_z=0.285,
            base_thickness=0.090,
            corner_radius=0.018,
            center=False,
        ),
        "yoke_frame",
    )
    yoke.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=safety_yellow,
        name="yoke_frame",
    )
    bearing_collar_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.082, tube=0.018, radial_segments=18, tubular_segments=36),
        "yoke_bearing_collar",
    )
    for idx, x in enumerate((-0.345, 0.345)):
        yoke.visual(
            bearing_collar_mesh,
            origin=Origin(xyz=(x, 0.0, 0.405), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"bearing_collar_{idx}",
        )

    lamp = model.part("lamp")
    lamp_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.125, -0.260),
                (0.145, -0.160),
                (0.168, 0.080),
                (0.194, 0.250),
            ],
            inner_profile=[
                (0.095, -0.235),
                (0.113, -0.130),
                (0.134, 0.080),
                (0.158, 0.222),
            ],
            segments=72,
            start_cap="round",
            end_cap="round",
            lip_samples=8,
        ),
        "lamp_shell",
    )
    lamp.visual(
        lamp_shell_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=lamp_coat,
        name="lamp_shell",
    )
    lamp.visual(
        Cylinder(radius=0.160, length=0.018),
        origin=Origin(xyz=(0.0, 0.256, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass_blue,
        name="front_lens",
    )
    front_bezel_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.176, tube=0.016, radial_segments=18, tubular_segments=48),
        "front_bezel",
    )
    lamp.visual(
        front_bezel_mesh,
        origin=Origin(xyz=(0.0, 0.260, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.110, length=0.045),
        origin=Origin(xyz=(0.0, -0.280, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_cap",
    )
    lamp.visual(
        Cylinder(radius=0.052, length=0.735),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="trunnion_pin",
    )
    for idx, x in enumerate((-0.180, 0.180)):
        lamp.visual(
            Cylinder(radius=0.065, length=0.032),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"side_boss_{idx}",
        )
    for idx, z in enumerate((-0.070, 0.0, 0.070)):
        lamp.visual(
            Box((0.030, 0.105, 0.010)),
            origin=Origin(xyz=(0.0, -0.305, z)),
            material=tower_steel,
            name=f"rear_cooling_fin_{idx}",
        )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.270)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=0.9,
            lower=-0.70,
            upper=0.95,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

    ctx.allow_overlap(
        lamp,
        yoke,
        elem_a="trunnion_pin",
        elem_b="yoke_frame",
        reason=(
            "The tilt shaft is intentionally represented as a captured pin passing "
            "through the yoke bearing bosses."
        ),
    )
    ctx.expect_gap(
        yoke,
        mast,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="bearing_lip",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable rests on the bearing lip",
    )
    ctx.expect_overlap(
        lamp,
        yoke,
        axes="x",
        elem_a="trunnion_pin",
        elem_b="yoke_frame",
        min_overlap=0.55,
        name="trunnion pin spans both thick yoke supports",
    )
    ctx.expect_within(
        lamp,
        yoke,
        axes="x",
        inner_elem="lamp_shell",
        outer_elem="yoke_frame",
        margin=0.0,
        name="lamp body fits between yoke arms",
    )

    rest_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({tilt: 0.55}):
        tilted_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    ctx.check(
        "positive tilt raises lamp face",
        rest_lens is not None
        and tilted_lens is not None
        and (tilted_lens[0][2] + tilted_lens[1][2]) * 0.5
        > (rest_lens[0][2] + rest_lens[1][2]) * 0.5
        + 0.08,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}",
    )

    rest_pan_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({pan: 0.75}):
        panned_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    ctx.check(
        "pan joint sweeps the head horizontally",
        rest_pan_lens is not None
        and panned_lens is not None
        and abs(
            (panned_lens[0][0] + panned_lens[1][0]) * 0.5
            - (rest_pan_lens[0][0] + rest_pan_lens[1][0]) * 0.5
        )
        > 0.12,
        details=f"rest_lens={rest_pan_lens}, panned_lens={panned_lens}",
    )

    return ctx.report()


object_model = build_object_model()
