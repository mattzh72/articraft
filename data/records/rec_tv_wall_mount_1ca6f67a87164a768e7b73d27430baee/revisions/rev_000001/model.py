from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_wall_display_arm")

    matte_black = model.material("matte_black", rgba=(0.02, 0.022, 0.024, 1.0))
    graphite = model.material("graphite_powdercoat", rgba=(0.12, 0.125, 0.13, 1.0))
    dark_rubber = model.material("dark_bolt_caps", rgba=(0.005, 0.005, 0.004, 1.0))
    worn_edge = model.material("worn_metal_edges", rgba=(0.46, 0.47, 0.46, 1.0))

    wall = model.part("wall_bracket")
    wall.visual(
        Box((0.035, 0.24, 0.36)),
        origin=Origin(xyz=(-0.095, 0.0, 0.0)),
        material=graphite,
        name="wall_plate",
    )
    # Top and bottom hinge shelves form an exposed clevis around the first yaw pin.
    for bridge_name, clevis_name, z in (
        ("upper_bridge", "upper_clevis", 0.026),
        ("lower_bridge", "lower_clevis", -0.026),
    ):
        wall.visual(
            Box((0.080, 0.086, 0.018)),
            origin=Origin(xyz=(-0.040, 0.0, z)),
            material=graphite,
            name=bridge_name,
        )
        wall.visual(
            Cylinder(radius=0.055, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=graphite,
            name=clevis_name,
        )
    for i, (y, z) in enumerate(((-0.072, 0.115), (0.072, 0.115), (-0.072, -0.115), (0.072, -0.115))):
        wall.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(-0.1155, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_rubber,
            name=f"wall_bolt_{i}",
        )

    primary = model.part("primary_link")
    primary.visual(
        Cylinder(radius=0.050, length=0.026),
        origin=Origin(),
        material=matte_black,
        name="proximal_hub",
    )
    primary.visual(
        Box((0.070, 0.075, 0.036)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=matte_black,
        name="proximal_neck",
    )
    for suffix, y in (("rail_0", -0.034), ("rail_1", 0.034)):
        primary.visual(
            Box((0.300, 0.024, 0.036)),
            origin=Origin(xyz=(0.205, y, 0.0)),
            material=matte_black,
            name=suffix,
        )
    primary.visual(
        Box((0.280, 0.045, 0.014)),
        origin=Origin(xyz=(0.210, 0.0, 0.0)),
        material=graphite,
        name="bridge_spine",
    )
    for arm_name, boss_name, z in (
        ("upper_fork_arm", "upper_fork_boss", 0.025),
        ("lower_fork_arm", "lower_fork_boss", -0.025),
    ):
        primary.visual(
            Box((0.130, 0.082, 0.014)),
            origin=Origin(xyz=(0.360, 0.0, z)),
            material=matte_black,
            name=arm_name,
        )
        primary.visual(
            Cylinder(radius=0.052, length=0.018),
            origin=Origin(xyz=(0.420, 0.0, z)),
            material=matte_black,
            name=boss_name,
        )

    secondary = model.part("secondary_link")
    secondary.visual(
        Cylinder(radius=0.047, length=0.026),
        origin=Origin(),
        material=matte_black,
        name="proximal_hub",
    )
    secondary.visual(
        Box((0.062, 0.068, 0.034)),
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
        material=matte_black,
        name="proximal_neck",
    )
    for suffix, y in (("rail_0", -0.030), ("rail_1", 0.030)):
        secondary.visual(
            Box((0.230, 0.022, 0.034)),
            origin=Origin(xyz=(0.170, y, 0.0)),
            material=matte_black,
            name=suffix,
        )
    secondary.visual(
        Box((0.210, 0.040, 0.012)),
        origin=Origin(xyz=(0.174, 0.0, 0.0)),
        material=graphite,
        name="bridge_spine",
    )
    for arm_name, boss_name, z in (
        ("upper_fork_arm", "upper_fork_boss", 0.024),
        ("lower_fork_arm", "lower_fork_boss", -0.024),
    ):
        secondary.visual(
            Box((0.118, 0.074, 0.014)),
            origin=Origin(xyz=(0.290, 0.0, z)),
            material=matte_black,
            name=arm_name,
        )
        secondary.visual(
            Cylinder(radius=0.048, length=0.018),
            origin=Origin(xyz=(0.340, 0.0, z)),
            material=matte_black,
            name=boss_name,
        )

    swivel = model.part("swivel_knuckle")
    swivel.visual(
        Cylinder(radius=0.043, length=0.026),
        origin=Origin(),
        material=matte_black,
        name="swivel_hub",
    )
    swivel.visual(
        Box((0.085, 0.052, 0.036)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=matte_black,
        name="swivel_neck",
    )
    swivel.visual(
        Box((0.052, 0.150, 0.036)),
        origin=Origin(xyz=(0.068, 0.0, 0.0)),
        material=graphite,
        name="yoke_bridge",
    )
    for suffix, y in (("yoke_0", -0.075), ("yoke_1", 0.075)):
        swivel.visual(
            Box((0.080, 0.018, 0.092)),
            origin=Origin(xyz=(0.120, y, 0.0)),
            material=matte_black,
            name=suffix,
        )
        swivel.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(0.120, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_edge,
            name=f"{suffix}_pin_cap",
        )

    head = model.part("vesa_head")
    head.visual(
        Cylinder(radius=0.024, length=0.132),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="tilt_barrel",
    )
    head.visual(
        Box((0.052, 0.048, 0.036)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=matte_black,
        name="head_neck",
    )
    head.visual(
        Box((0.018, 0.165, 0.165)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=graphite,
        name="vesa_plate",
    )
    head.visual(
        Box((0.006, 0.070, 0.016)),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material=dark_rubber,
        name="horizontal_slot",
    )
    head.visual(
        Box((0.006, 0.016, 0.070)),
        origin=Origin(xyz=(0.071, 0.0, 0.0)),
        material=dark_rubber,
        name="vertical_slot",
    )
    for row, z in enumerate((-0.050, 0.050)):
        for col, y in enumerate((-0.050, 0.050)):
            head.visual(
                Cylinder(radius=0.009, length=0.006),
                origin=Origin(xyz=(0.070, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_rubber,
                name=f"vesa_bolt_{row}_{col}",
            )

    model.articulation(
        "wall_to_primary",
        ArticulationType.REVOLUTE,
        parent=wall,
        child=primary,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.7, lower=-2.15, upper=2.15),
    )
    model.articulation(
        "secondary_to_swivel",
        ArticulationType.REVOLUTE,
        parent=secondary,
        child=swivel,
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall_bracket")
    primary = object_model.get_part("primary_link")
    secondary = object_model.get_part("secondary_link")
    swivel = object_model.get_part("swivel_knuckle")
    head = object_model.get_part("vesa_head")

    wall_joint = object_model.get_articulation("wall_to_primary")
    elbow_joint = object_model.get_articulation("primary_to_secondary")
    swivel_joint = object_model.get_articulation("secondary_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")

    for joint in (wall_joint, elbow_joint, swivel_joint, tilt_joint):
        ctx.check(
            f"{joint.name} is revolute",
            str(joint.articulation_type).lower().endswith("revolute"),
            details=f"{joint.name} has type {joint.articulation_type}",
        )

    ctx.expect_overlap(
        wall,
        primary,
        axes="xy",
        elem_a="upper_clevis",
        elem_b="proximal_hub",
        min_overlap=0.045,
        name="wall clevis captures primary hub in plan",
    )
    ctx.expect_gap(
        wall,
        primary,
        axis="z",
        positive_elem="upper_clevis",
        negative_elem="proximal_hub",
        min_gap=0.001,
        max_gap=0.008,
        name="wall upper clevis clears primary hub",
    )
    ctx.expect_overlap(
        primary,
        secondary,
        axes="xy",
        elem_a="upper_fork_boss",
        elem_b="proximal_hub",
        min_overlap=0.040,
        name="elbow clevis captures secondary hub in plan",
    )
    ctx.expect_gap(
        primary,
        secondary,
        axis="z",
        positive_elem="upper_fork_boss",
        negative_elem="proximal_hub",
        min_gap=0.001,
        max_gap=0.008,
        name="elbow fork clears secondary hub",
    )
    ctx.expect_overlap(
        secondary,
        swivel,
        axes="xy",
        elem_a="upper_fork_boss",
        elem_b="swivel_hub",
        min_overlap=0.038,
        name="outer clevis captures swivel hub in plan",
    )
    ctx.expect_overlap(
        swivel,
        head,
        axes="xz",
        elem_a="yoke_1",
        elem_b="tilt_barrel",
        min_overlap=0.020,
        name="tilt yoke straddles head barrel profile",
    )

    head_aabb_rest = ctx.part_element_world_aabb(head, elem="vesa_plate")
    with ctx.pose({tilt_joint: 0.45}):
        head_aabb_tilted = ctx.part_element_world_aabb(head, elem="vesa_plate")
    if head_aabb_rest is not None and head_aabb_tilted is not None:
        rest_center_z = (head_aabb_rest[0][2] + head_aabb_rest[1][2]) * 0.5
        tilted_center_z = (head_aabb_tilted[0][2] + head_aabb_tilted[1][2]) * 0.5
        ctx.check(
            "tilt joint pitches VESA plate",
            abs(tilted_center_z - rest_center_z) > 0.010,
            details=f"rest_z={rest_center_z:.4f}, tilted_z={tilted_center_z:.4f}",
        )
    else:
        ctx.fail("tilt joint pitches VESA plate", "VESA plate AABB could not be measured")

    return ctx.report()


object_model = build_object_model()
