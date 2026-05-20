from __future__ import annotations

import cadquery as cq
from math import pi

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


REAR_ARM_LENGTH = 0.62
FRONT_ARM_LENGTH = 0.56


def _add_long_arm(part, length: float, material, accent_material) -> None:
    """Two parallel stamped bars captured by broad pivot bosses."""
    part.visual(
        Cylinder(radius=0.060, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=accent_material,
        name="inner_pivot_boss",
    )
    part.visual(
        Cylinder(radius=0.060, length=0.048),
        origin=Origin(xyz=(length, 0.0, 0.024)),
        material=accent_material,
        name="outer_pivot_boss",
    )
    for y, name in ((-0.036, "bar_0"), (0.036, "bar_1")):
        part.visual(
            Box((length - 0.080, 0.024, 0.032)),
            origin=Origin(xyz=(length * 0.5, y, 0.024)),
            material=material,
            name=name,
        )
    part.visual(
        Box((length - 0.150, 0.090, 0.010)),
        origin=Origin(xyz=(length * 0.5, 0.0, 0.0445)),
        material=material,
        name="cable_cover",
    )


def _vesa_plate_mesh():
    """A thin VESA adapter plate with real through-holes and softened corners."""
    plate = (
        cq.Workplane("XY")
        .box(0.012, 0.150, 0.150)
        .edges("|X")
        .fillet(0.010)
        .faces(">X")
        .workplane()
        .pushPoints(
            [
                (-0.050, -0.050),
                (-0.050, 0.050),
                (0.050, -0.050),
                (0.050, 0.050),
            ]
        )
        .circle(0.0045)
        .cutThruAll()
        .translate((0.060, 0.0, 0.0))
    )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_bench_monitor_arm")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.04, 0.045, 0.05, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.21, 0.22, 1.0))
    rubber = model.material("rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.280, 0.200, 0.035)),
        origin=Origin(xyz=(-0.015, 0.0, 0.0175)),
        material=dark_metal,
        name="clamp_top_plate",
    )
    base.visual(
        Box((0.055, 0.170, 0.170)),
        origin=Origin(xyz=(-0.130, 0.0, -0.050)),
        material=dark_metal,
        name="clamp_spine",
    )
    base.visual(
        Box((0.180, 0.150, 0.035)),
        origin=Origin(xyz=(-0.075, 0.0, -0.1525)),
        material=dark_metal,
        name="lower_jaw",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.115),
        origin=Origin(xyz=(-0.005, 0.0, -0.080)),
        material=dark_metal,
        name="clamp_screw",
    )
    base.visual(
        Cylinder(radius=0.037, length=0.010),
        origin=Origin(xyz=(-0.005, 0.0, -0.0175)),
        material=rubber,
        name="pressure_pad",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=dark_metal,
        name="post_collar",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=satin_black,
        name="vertical_post",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.5175)),
        material=dark_metal,
        name="shoulder_cap",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.052),
        origin=Origin(xyz=(0.055, -0.067, 0.066), rpy=(pi / 2, 0.0, 0.0)),
        material=rubber,
        name="collar_knob",
    )

    rear_arm = model.part("rear_arm")
    _add_long_arm(rear_arm, REAR_ARM_LENGTH, matte_black, dark_metal)

    front_arm = model.part("front_arm")
    _add_long_arm(front_arm, FRONT_ARM_LENGTH, matte_black, dark_metal)

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        Cylinder(radius=0.052, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="swivel_stack",
    )
    head_swivel.visual(
        Box((0.092, 0.045, 0.035)),
        origin=Origin(xyz=(0.046, 0.0, 0.025)),
        material=satin_black,
        name="short_neck",
    )
    head_swivel.visual(
        Box((0.030, 0.135, 0.018)),
        origin=Origin(xyz=(0.083, 0.0, 0.025)),
        material=satin_black,
        name="yoke_bridge",
    )
    head_swivel.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.120, -0.052, 0.025), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_knuckle_0",
    )
    head_swivel.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.120, 0.052, 0.025), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_knuckle_1",
    )

    vesa_plate = model.part("vesa_plate")
    vesa_plate.visual(
        Cylinder(radius=0.018, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_barrel",
    )
    vesa_plate.visual(
        Box((0.055, 0.044, 0.034)),
        origin=Origin(xyz=(0.0275, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_tongue",
    )
    vesa_plate.visual(
        mesh_from_cadquery(_vesa_plate_mesh(), "vesa_mount_plate", tolerance=0.0008),
        material=satin_black,
        name="mount_plate",
    )

    model.articulation(
        "base_to_rear_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rear_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-1.60, upper=1.60),
    )
    model.articulation(
        "rear_arm_to_front_arm",
        ArticulationType.REVOLUTE,
        parent=rear_arm,
        child=front_arm,
        origin=Origin(xyz=(REAR_ARM_LENGTH, 0.0, 0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.5, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "front_arm_to_head_swivel",
        ArticulationType.REVOLUTE,
        parent=front_arm,
        child=head_swivel,
        origin=Origin(xyz=(FRONT_ARM_LENGTH, 0.0, 0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "head_swivel_to_vesa_plate",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=vesa_plate,
        origin=Origin(xyz=(0.120, 0.0, 0.025)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = {
        name: object_model.get_articulation(name)
        for name in (
            "base_to_rear_arm",
            "rear_arm_to_front_arm",
            "front_arm_to_head_swivel",
            "head_swivel_to_vesa_plate",
        )
    }
    ctx.check(
        "four requested revolute joints",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints.values()),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )

    base = object_model.get_part("base")
    rear_arm = object_model.get_part("rear_arm")
    front_arm = object_model.get_part("front_arm")
    head_swivel = object_model.get_part("head_swivel")
    vesa_plate = object_model.get_part("vesa_plate")

    ctx.expect_gap(
        rear_arm,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="inner_pivot_boss",
        negative_elem="shoulder_cap",
        name="rear arm is seated on post cap",
    )
    ctx.expect_gap(
        front_arm,
        rear_arm,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="inner_pivot_boss",
        negative_elem="outer_pivot_boss",
        name="front arm is stacked on rear elbow",
    )
    ctx.expect_gap(
        head_swivel,
        front_arm,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="swivel_stack",
        negative_elem="outer_pivot_boss",
        name="head swivel is stacked on forearm end",
    )
    ctx.expect_overlap(
        vesa_plate,
        head_swivel,
        axes="xz",
        min_overlap=0.020,
        elem_a="tilt_barrel",
        elem_b="tilt_knuckle_0",
        name="tilt barrel is coaxial with yoke knuckles",
    )

    head_pos = ctx.part_world_position(head_swivel)
    ctx.check(
        "long reach exceeds one meter",
        head_pos is not None and head_pos[0] > 1.05,
        details=f"head_swivel_position={head_pos}",
    )

    rest_plate = ctx.part_world_aabb(vesa_plate)
    with ctx.pose({"base_to_rear_arm": 0.80, "rear_arm_to_front_arm": -0.40}):
        folded_head = ctx.part_world_position(head_swivel)
    ctx.check(
        "arm fold joints move the head laterally",
        head_pos is not None
        and folded_head is not None
        and abs(folded_head[1] - head_pos[1]) > 0.20,
        details=f"rest={head_pos}, folded={folded_head}",
    )

    with ctx.pose({"head_swivel_to_vesa_plate": 0.35}):
        tilted_plate = ctx.part_world_aabb(vesa_plate)
    ctx.check(
        "tilt hinge changes plate pitch envelope",
        rest_plate is not None
        and tilted_plate is not None
        and abs((tilted_plate[1][0] - tilted_plate[0][0]) - (rest_plate[1][0] - rest_plate[0][0]))
        > 0.015,
        details=f"rest_aabb={rest_plate}, tilted_aabb={tilted_plate}",
    )

    return ctx.report()


object_model = build_object_model()
