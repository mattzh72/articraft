from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _wishbone_loop(
    name: str,
    *,
    inner_front_x: float,
    inner_rear_x: float,
    outer_y: float,
    outer_z: float,
    radius: float,
):
    return _mesh(
        name,
        wire_from_points(
            [
                (inner_front_x, 0.0, 0.0),
                (0.0, outer_y, outer_z),
                (inner_rear_x, 0.0, 0.0),
            ],
            radius=radius,
            radial_segments=18,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=0.05 if radius >= 0.017 else 0.04,
            corner_segments=10,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_wishbone_front_corner")

    subframe_paint = model.material("subframe_paint", rgba=(0.20, 0.21, 0.23, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    forged_steel = model.material("forged_steel", rgba=(0.36, 0.38, 0.41, 1.0))
    machined_metal = model.material("machined_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    bushing_black = model.material("bushing_black", rgba=(0.08, 0.08, 0.09, 1.0))

    subframe = model.part("subframe")
    subframe.inertial = Inertial.from_geometry(
        Box((0.42, 0.16, 0.36)),
        mass=28.0,
        origin=Origin(xyz=(0.0, -0.04, 0.12)),
    )
    subframe.visual(
        Box((0.36, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, -0.05, -0.03)),
        material=subframe_paint,
        name="lower_crossmember",
    )
    subframe.visual(
        Box((0.28, 0.12, 0.32)),
        origin=Origin(xyz=(0.0, -0.04, 0.14)),
        material=subframe_paint,
        name="upright_tower",
    )
    subframe.visual(
        Box((0.20, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, -0.01, 0.09)),
        material=subframe_paint,
        name="mid_gusset",
    )
    for name, x_pos, z_pos in (
        ("lower_front_bracket", 0.12, 0.02),
        ("lower_rear_bracket", -0.12, 0.02),
        ("upper_front_bracket", 0.09, 0.28),
        ("upper_rear_bracket", -0.09, 0.28),
    ):
        subframe.visual(
            Box((0.06, 0.04, 0.08)),
            origin=Origin(xyz=(x_pos, 0.0, z_pos)),
            material=subframe_paint,
            name=name,
        )
    subframe.visual(
        Cylinder(radius=0.022, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_steel,
        name="lower_pivot_tube",
    )
    subframe.visual(
        Cylinder(radius=0.021, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.28), rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_steel,
        name="upper_pivot_tube",
    )

    upper_wishbone = model.part("upper_wishbone")
    upper_wishbone.inertial = Inertial.from_geometry(
        Box((0.24, 0.30, 0.06)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.15, 0.0)),
    )
    upper_wishbone.visual(
        _wishbone_loop(
            "upper_wishbone_loop",
            inner_front_x=0.10,
            inner_rear_x=-0.10,
            outer_y=0.24,
            outer_z=-0.01,
            radius=0.015,
        ),
        material=arm_paint,
        name="upper_wishbone_loop",
    )
    for name, x_pos in (("upper_front_bushing", 0.10), ("upper_rear_bushing", -0.10)):
        upper_wishbone.visual(
            Cylinder(radius=0.024, length=0.060),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=bushing_black,
            name=name,
        )
    for name, x_pos in (("upper_outer_ear_front", 0.023), ("upper_outer_ear_rear", -0.023)):
        upper_wishbone.visual(
            Box((0.018, 0.074, 0.040)),
            origin=Origin(xyz=(x_pos, 0.273, -0.01)),
            material=arm_paint,
            name=name,
        )
    upper_wishbone.visual(
        Box((0.060, 0.050, 0.022)),
        origin=Origin(xyz=(0.0, 0.255, -0.01)),
        material=arm_paint,
        name="upper_outer_yoke",
    )

    lower_wishbone = model.part("lower_wishbone")
    lower_wishbone.inertial = Inertial.from_geometry(
        Box((0.28, 0.33, 0.07)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.16, 0.015)),
    )
    lower_wishbone.visual(
        _wishbone_loop(
            "lower_wishbone_loop",
            inner_front_x=0.12,
            inner_rear_x=-0.12,
            outer_y=0.26,
            outer_z=0.03,
            radius=0.018,
        ),
        material=arm_paint,
        name="lower_wishbone_loop",
    )
    for name, x_pos in (("lower_front_bushing", 0.12), ("lower_rear_bushing", -0.12)):
        lower_wishbone.visual(
            Cylinder(radius=0.027, length=0.065),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=bushing_black,
            name=name,
        )
    for name, x_pos in (("lower_outer_ear_front", 0.026), ("lower_outer_ear_rear", -0.026)):
        lower_wishbone.visual(
            Box((0.020, 0.070, 0.050)),
            origin=Origin(xyz=(x_pos, 0.285, 0.03)),
            material=arm_paint,
            name=name,
        )

    upper_knuckle_ear = model.part("upper_knuckle_ear")
    upper_knuckle_ear.inertial = Inertial.from_geometry(
        Box((0.05, 0.04, 0.07)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
    )
    upper_knuckle_ear.visual(
        Box((0.020, 0.030, 0.040)),
        material=forged_steel,
        name="upper_ball_receiver",
    )
    upper_knuckle_ear.visual(
        Box((0.038, 0.028, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=forged_steel,
        name="upper_upright_post",
    )

    knuckle = model.part("knuckle")
    knuckle.inertial = Inertial.from_geometry(
        Box((0.12, 0.14, 0.24)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.04, 0.12)),
    )
    knuckle.visual(
        Box((0.032, 0.034, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=forged_steel,
        name="lower_ball_receiver",
    )
    knuckle.visual(
        Box((0.050, 0.050, 0.164)),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=forged_steel,
        name="upright_body",
    )
    knuckle.visual(
        Cylinder(radius=0.048, length=0.056),
        origin=Origin(xyz=(0.0, 0.053, 0.15), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined_metal,
        name="bearing_boss",
    )
    knuckle.visual(
        Box((0.12, 0.024, 0.020)),
        origin=Origin(xyz=(0.065, 0.018, 0.105)),
        material=forged_steel,
        name="steering_arm",
    )

    hub = model.part("hub")
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.090),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    hub.visual(
        Cylinder(radius=0.085, length=0.060),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined_metal,
        name="hub_flange",
    )
    hub.visual(
        Cylinder(radius=0.035, length=0.090),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=forged_steel,
        name="hub_snout",
    )
    hub.visual(
        Cylinder(radius=0.060, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined_metal,
        name="mounting_face",
    )
    for stud_index in range(5):
        angle = 2.0 * pi * stud_index / 5.0
        hub.visual(
            Cylinder(radius=0.0055, length=0.020),
            origin=Origin(
                xyz=(0.043 * cos(angle), 0.010, 0.043 * sin(angle)),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=machined_metal,
            name=f"stud_{stud_index + 1}",
        )

    model.articulation(
        "upper_inner_pivot",
        ArticulationType.REVOLUTE,
        parent=subframe,
        child=upper_wishbone,
        origin=Origin(xyz=(0.0, 0.045, 0.28)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.6, lower=-0.35, upper=0.25),
    )
    model.articulation(
        "lower_inner_pivot",
        ArticulationType.REVOLUTE,
        parent=subframe,
        child=lower_wishbone,
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.6, lower=-0.25, upper=0.30),
    )
    model.articulation(
        "upper_outer_ball_joint",
        ArticulationType.REVOLUTE,
        parent=upper_wishbone,
        child=upper_knuckle_ear,
        origin=Origin(xyz=(0.0, 0.31, -0.01)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "lower_outer_ball_joint",
        ArticulationType.REVOLUTE,
        parent=lower_wishbone,
        child=knuckle,
        origin=Origin(xyz=(0.0, 0.31, 0.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.0, lower=-0.30, upper=0.30),
    )
    model.articulation(
        "hub_bearing",
        ArticulationType.CONTINUOUS,
        parent=knuckle,
        child=hub,
        origin=Origin(xyz=(0.0, 0.081, 0.15)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    subframe = object_model.get_part("subframe")
    upper_wishbone = object_model.get_part("upper_wishbone")
    lower_wishbone = object_model.get_part("lower_wishbone")
    upper_knuckle_ear = object_model.get_part("upper_knuckle_ear")
    knuckle = object_model.get_part("knuckle")
    hub = object_model.get_part("hub")

    upper_inner_pivot = object_model.get_articulation("upper_inner_pivot")
    lower_inner_pivot = object_model.get_articulation("lower_inner_pivot")
    upper_outer_ball_joint = object_model.get_articulation("upper_outer_ball_joint")
    lower_outer_ball_joint = object_model.get_articulation("lower_outer_ball_joint")
    hub_bearing = object_model.get_articulation("hub_bearing")

    ctx.check(
        "wishbone and hub articulation axes are correct",
        upper_inner_pivot.axis == (1.0, 0.0, 0.0)
        and lower_inner_pivot.axis == (1.0, 0.0, 0.0)
        and upper_outer_ball_joint.axis == (1.0, 0.0, 0.0)
        and lower_outer_ball_joint.axis == (1.0, 0.0, 0.0)
        and hub_bearing.axis == (0.0, 1.0, 0.0),
        details=(
            f"upper_inner={upper_inner_pivot.axis}, lower_inner={lower_inner_pivot.axis}, "
            f"upper_outer={upper_outer_ball_joint.axis}, lower_outer={lower_outer_ball_joint.axis}, "
            f"hub={hub_bearing.axis}"
        ),
    )

    with ctx.pose(
        {
            upper_inner_pivot: 0.0,
            lower_inner_pivot: 0.0,
            upper_outer_ball_joint: 0.0,
            lower_outer_ball_joint: 0.0,
            hub_bearing: 0.0,
        }
    ):
        ctx.expect_gap(
            upper_knuckle_ear,
            knuckle,
            axis="z",
            max_gap=0.001,
            max_penetration=1e-6,
            name="upper knuckle ear seats on the upright top face",
        )
        ctx.expect_overlap(
            upper_knuckle_ear,
            knuckle,
            axes="xy",
            min_overlap=0.020,
            name="upper knuckle ear stays centered over the upright",
        )
        ctx.expect_gap(
            hub,
            knuckle,
            axis="y",
            max_gap=0.001,
            max_penetration=1e-6,
            name="hub seats against the knuckle bearing shoulder",
        )
        ctx.expect_overlap(
            hub,
            knuckle,
            axes="xz",
            min_overlap=0.060,
            name="hub remains coaxial with the knuckle bearing",
        )
        ctx.expect_origin_gap(
            hub,
            subframe,
            axis="y",
            min_gap=0.35,
            name="hub sits clearly outboard of the subframe",
        )

    rest_upper_ear_pos = ctx.part_world_position(upper_knuckle_ear)
    with ctx.pose({upper_inner_pivot: 0.20}):
        lifted_upper_ear_pos = ctx.part_world_position(upper_knuckle_ear)
    ctx.check(
        "upper wishbone pivots upward about the subframe axis",
        rest_upper_ear_pos is not None
        and lifted_upper_ear_pos is not None
        and lifted_upper_ear_pos[2] > rest_upper_ear_pos[2] + 0.04,
        details=f"rest={rest_upper_ear_pos}, lifted={lifted_upper_ear_pos}",
    )

    rest_knuckle_pos = ctx.part_world_position(knuckle)
    with ctx.pose({lower_inner_pivot: 0.20}):
        lifted_knuckle_pos = ctx.part_world_position(knuckle)
    ctx.check(
        "lower wishbone carries the knuckle upward in jounce",
        rest_knuckle_pos is not None
        and lifted_knuckle_pos is not None
        and lifted_knuckle_pos[2] > rest_knuckle_pos[2] + 0.04,
        details=f"rest={rest_knuckle_pos}, lifted={lifted_knuckle_pos}",
    )

    rest_hub_pos = ctx.part_world_position(hub)
    with ctx.pose({hub_bearing: 1.8}):
        spun_hub_pos = ctx.part_world_position(hub)
    ctx.check(
        "hub spin keeps the bearing axis fixed in space",
        rest_hub_pos is not None
        and spun_hub_pos is not None
        and max(abs(a - b) for a, b in zip(rest_hub_pos, spun_hub_pos)) < 1e-9,
        details=f"rest={rest_hub_pos}, spun={spun_hub_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
