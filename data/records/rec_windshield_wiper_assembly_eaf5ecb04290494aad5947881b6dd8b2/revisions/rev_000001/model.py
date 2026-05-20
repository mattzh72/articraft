from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _hollow_roll_sleeve() -> cq.Workplane:
    """A compact tube coaxial with the arm tip, modeled hollow for real clearance."""
    return (
        cq.Workplane("YZ")
        .circle(0.024)
        .circle(0.014)
        .extrude(0.062, both=True)
        .edges()
        .fillet(0.0012)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_windshield_wiper")

    satin_black = _mat("satin_black", (0.015, 0.016, 0.017, 1.0))
    rubber = _mat("rubber_black", (0.002, 0.002, 0.002, 1.0))
    cast_metal = _mat("cast_dark_metal", (0.18, 0.19, 0.20, 1.0))
    brushed = _mat("brushed_steel", (0.55, 0.56, 0.54, 1.0))

    # Root: a dense motor/gearbox pod with an exposed fixed spindle boss.
    housing = model.part("motor_housing")
    housing.visual(
        Box((0.255, 0.125, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=cast_metal,
        name="mount_plate",
    )
    housing.visual(
        Cylinder(radius=0.074, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=satin_black,
        name="gear_case",
    )
    housing.visual(
        Cylinder(radius=0.043, length=0.178),
        origin=Origin(xyz=(0.0, -0.132, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="motor_can",
    )
    housing.visual(
        Box((0.052, 0.090, 0.026)),
        origin=Origin(xyz=(0.0, -0.056, 0.032)),
        material=satin_black,
        name="neck_casting",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=cast_metal,
        name="spindle_boss",
    )
    housing.visual(
        Cylinder(radius=0.0175, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=brushed,
        name="spindle_post",
    )
    for i, x in enumerate((-0.095, 0.095)):
        housing.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(x, 0.041, 0.018)),
            material=brushed,
            name=f"bolt_head_{i}",
        )

    # Child frame is exactly on the spindle axis at the top of the post.
    arm = model.part("sweep_arm")
    arm.visual(
        Cylinder(radius=0.036, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=cast_metal,
        name="hub_cap",
    )
    arm.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=brushed,
        name="retaining_nut",
    )
    arm_profile = [
        (0.024, -0.018),
        (0.335, -0.010),
        (0.555, -0.007),
        (0.585, -0.012),
        (0.585, 0.012),
        (0.555, 0.007),
        (0.335, 0.010),
        (0.024, 0.018),
    ]
    arm.visual(
        mesh_from_geometry(ExtrudeGeometry(arm_profile, 0.012, center=True), "tapered_arm_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=cast_metal,
        name="tapered_arm_plate",
    )
    arm.visual(
        Box((0.425, 0.007, 0.010)),
        origin=Origin(xyz=(0.310, 0.0, 0.024)),
        material=brushed,
        name="center_rib",
    )
    arm.visual(
        Box((0.075, 0.010, 0.010)),
        origin=Origin(xyz=(0.586, 0.0, 0.014)),
        material=cast_metal,
        name="tip_neck",
    )
    arm.visual(
        Cylinder(radius=0.014, length=0.052),
        origin=Origin(xyz=(0.620, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="tip_barrel",
    )

    blade = model.part("blade_carrier")
    blade.visual(
        mesh_from_cadquery(_hollow_roll_sleeve(), "hollow_roll_sleeve", tolerance=0.0007),
        origin=Origin(),
        material=cast_metal,
        name="roll_sleeve",
    )
    blade.visual(
        Box((0.024, 0.078, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=cast_metal,
        name="sleeve_saddle",
    )
    blade.visual(
        Box((0.032, 0.160, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=cast_metal,
        name="blade_bridge",
    )
    blade.visual(
        Box((0.020, 0.645, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.066)),
        material=brushed,
        name="spring_rail",
    )
    blade.visual(
        Box((0.018, 0.635, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.081)),
        material=rubber,
        name="rubber_backing",
    )
    blade.visual(
        Box((0.006, 0.650, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.103)),
        material=rubber,
        name="rubber_edge",
    )
    for i, y in enumerate((-0.327, 0.327)):
        blade.visual(
            Box((0.024, 0.012, 0.034)),
            origin=Origin(xyz=(0.0, y, -0.082)),
            material=rubber,
            name=f"end_cap_{i}",
        )

    sweep_joint = model.articulation(
        "spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-1.20, upper=1.20),
    )
    roll_joint = model.articulation(
        "tip_roll",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(0.620, 0.0, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.35, upper=0.35),
    )
    # Keep explicit references in metadata for easy inspection without inventing
    # secondary mechanisms.
    model.meta["primary_joints"] = (sweep_joint.name, roll_joint.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("motor_housing")
    arm = object_model.get_part("sweep_arm")
    blade = object_model.get_part("blade_carrier")
    sweep_joint = object_model.get_articulation("spindle_sweep")
    roll_joint = object_model.get_articulation("tip_roll")

    ctx.allow_overlap(
        blade,
        arm,
        elem_a="roll_sleeve",
        elem_b="tip_barrel",
        reason=(
            "The roll sleeve is an idealized captured bushing around the arm tip barrel; "
            "the tight nested shaft fit is intentional and local to the roll joint."
        ),
    )

    ctx.expect_contact(
        arm,
        housing,
        elem_a="hub_cap",
        elem_b="spindle_post",
        contact_tol=0.001,
        name="arm hub is seated on the spindle post",
    )
    ctx.expect_overlap(
        arm,
        housing,
        axes="xy",
        elem_a="hub_cap",
        elem_b="spindle_post",
        min_overlap=0.020,
        name="hub surrounds spindle in plan",
    )
    ctx.expect_within(
        arm,
        blade,
        axes="yz",
        inner_elem="tip_barrel",
        outer_elem="roll_sleeve",
        margin=0.001,
        name="tip barrel is tightly nested in roll sleeve envelope",
    )
    ctx.expect_overlap(
        arm,
        blade,
        axes="x",
        elem_a="tip_barrel",
        elem_b="roll_sleeve",
        min_overlap=0.045,
        name="roll sleeve spans the arm tip barrel",
    )

    rest_tip = ctx.part_world_position(blade)
    with ctx.pose({sweep_joint: 0.80}):
        swept_tip = ctx.part_world_position(blade)
    ctx.check(
        "arm sweep moves blade tip around spindle",
        rest_tip is not None
        and swept_tip is not None
        and swept_tip[1] > rest_tip[1] + 0.40
        and swept_tip[0] < rest_tip[0] - 0.15,
        details=f"rest={rest_tip}, swept={swept_tip}",
    )

    rest_edge = ctx.part_element_world_aabb(blade, elem="rubber_edge")
    with ctx.pose({roll_joint: 0.30}):
        rolled_edge = ctx.part_element_world_aabb(blade, elem="rubber_edge")
    rest_z_span = rest_edge[1][2] - rest_edge[0][2] if rest_edge else 0.0
    rolled_z_span = rolled_edge[1][2] - rolled_edge[0][2] if rolled_edge else 0.0
    ctx.check(
        "blade carrier rolls about the arm axis",
        rolled_z_span > rest_z_span + 0.12,
        details=f"rest_z_span={rest_z_span}, rolled_z_span={rolled_z_span}",
    )

    return ctx.report()


object_model = build_object_model()
