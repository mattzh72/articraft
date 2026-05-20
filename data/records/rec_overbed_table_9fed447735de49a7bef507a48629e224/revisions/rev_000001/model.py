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
    model = ArticulatedObject(name="hospital_overbed_table")

    painted_steel = model.material("painted_steel", rgba=(0.80, 0.82, 0.84, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    tray_plastic = model.material("tray_plastic", rgba=(0.88, 0.90, 0.88, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.035, 0.035, 0.035, 1.0))
    lock_blue = model.material("lock_blue", rgba=(0.05, 0.18, 0.58, 1.0))
    release_gray = model.material("release_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.08, 0.08, 0.085, 1.0))

    base = model.part("base_frame")

    # Low C-shaped rolling base: two long rails and one rear cross rail leave a
    # clear open front for rolling under a hospital bed.
    base.visual(
        Box((0.075, 0.690, 0.045)),
        origin=Origin(xyz=(-0.325, 0.0, 0.105)),
        material=painted_steel,
        name="side_rail_0",
    )
    base.visual(
        Box((0.075, 0.690, 0.045)),
        origin=Origin(xyz=(0.325, 0.0, 0.105)),
        material=painted_steel,
        name="side_rail_1",
    )
    base.visual(
        Box((0.720, 0.075, 0.045)),
        origin=Origin(xyz=(0.0, 0.305, 0.105)),
        material=painted_steel,
        name="rear_cross_rail",
    )
    base.visual(
        Box((0.160, 0.125, 0.038)),
        origin=Origin(xyz=(0.0, 0.260, 0.142)),
        material=painted_steel,
        name="post_socket",
    )
    base.visual(
        Box((0.105, 0.060, 0.100)),
        origin=Origin(xyz=(0.0, 0.230, 0.175)),
        material=painted_steel,
        name="front_gusset",
    )
    base.visual(
        Box((0.105, 0.060, 0.100)),
        origin=Origin(xyz=(0.0, 0.300, 0.175)),
        material=painted_steel,
        name="rear_gusset",
    )

    # Hollow outer post represented by four separated steel walls with a real
    # clear bore for the sliding inner lift column.
    outer_od = 0.074
    wall = 0.012
    post_height = 0.455
    post_center_z = 0.3225
    post_y = 0.260
    base.visual(
        Box((wall, outer_od, post_height)),
        origin=Origin(xyz=(-(outer_od / 2.0 - wall / 2.0), post_y, post_center_z)),
        material=brushed_steel,
        name="outer_post_side_wall_0",
    )
    base.visual(
        Box((wall, outer_od, post_height)),
        origin=Origin(xyz=((outer_od / 2.0 - wall / 2.0), post_y, post_center_z)),
        material=brushed_steel,
        name="outer_post_side_wall_1",
    )
    base.visual(
        Box((outer_od, wall, post_height)),
        origin=Origin(xyz=(0.0, post_y - (outer_od / 2.0 - wall / 2.0), post_center_z)),
        material=brushed_steel,
        name="outer_post_front_wall",
    )
    base.visual(
        Box((outer_od, wall, post_height)),
        origin=Origin(xyz=(0.0, post_y + (outer_od / 2.0 - wall / 2.0), post_center_z)),
        material=brushed_steel,
        name="outer_post_rear_wall",
    )
    base.visual(
        Box((0.100, 0.012, 0.025)),
        origin=Origin(xyz=(0.0, post_y - 0.041, 0.562)),
        material=brushed_steel,
        name="outer_collar_front",
    )
    base.visual(
        Box((0.100, 0.012, 0.025)),
        origin=Origin(xyz=(0.0, post_y + 0.041, 0.562)),
        material=brushed_steel,
        name="outer_collar_rear",
    )
    base.visual(
        Box((0.012, 0.076, 0.025)),
        origin=Origin(xyz=(-0.041, post_y, 0.562)),
        material=brushed_steel,
        name="outer_collar_side_0",
    )
    base.visual(
        Box((0.012, 0.076, 0.025)),
        origin=Origin(xyz=(0.041, post_y, 0.562)),
        material=brushed_steel,
        name="outer_collar_side_1",
    )

    caster_positions = [
        (-0.325, -0.300),
        (0.325, -0.300),
        (-0.325, 0.300),
        (0.325, 0.300),
    ]
    for index, (x, y) in enumerate(caster_positions):
        outward = -1.0 if x < 0.0 else 1.0
        base.visual(
            Cylinder(radius=0.018, length=0.030),
            origin=Origin(xyz=(x, y, 0.105)),
            material=brushed_steel,
            name=f"caster_stem_{index}",
        )
        base.visual(
            Box((0.058, 0.048, 0.012)),
            origin=Origin(xyz=(x, y, 0.096)),
            material=brushed_steel,
            name=f"caster_plate_{index}",
        )
        base.visual(
            Box((0.006, 0.044, 0.080)),
            origin=Origin(xyz=(x - 0.026, y, 0.050)),
            material=brushed_steel,
            name=f"caster_fork_ear_{index}_0",
        )
        base.visual(
            Box((0.006, 0.044, 0.080)),
            origin=Origin(xyz=(x + 0.026, y, 0.050)),
            material=brushed_steel,
            name=f"caster_fork_ear_{index}_1",
        )
        base.visual(
            Box((0.150, 0.010, 0.012)),
            origin=Origin(xyz=(x + outward * 0.040, y + 0.020, 0.116)),
            material=brushed_steel,
            name=f"lock_pivot_bracket_{index}",
        )

    inner = model.part("inner_column")
    inner.visual(
        Box((0.050, 0.050, 0.750)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=brushed_steel,
        name="inner_bar",
    )
    inner.visual(
        Box((0.066, 0.066, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
        material=brushed_steel,
        name="upper_collar",
    )

    head = model.part("support_head")
    head.visual(
        Box((0.115, 0.100, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=painted_steel,
        name="head_block",
    )
    head.visual(
        Box((0.310, 0.075, 0.028)),
        origin=Origin(xyz=(0.0, -0.012, 0.050)),
        material=painted_steel,
        name="tilt_saddle",
    )
    head.visual(
        Cylinder(radius=0.015, length=0.120),
        origin=Origin(xyz=(0.0, -0.020, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="center_hinge_barrel",
    )
    head.visual(
        Box((0.035, 0.035, 0.030)),
        origin=Origin(xyz=(0.0, -0.020, 0.0675)),
        material=painted_steel,
        name="hinge_barrel_web",
    )
    head.visual(
        Box((0.028, 0.090, 0.065)),
        origin=Origin(xyz=(-0.150, -0.010, 0.055)),
        material=painted_steel,
        name="hinge_cheek_0",
    )
    head.visual(
        Box((0.028, 0.090, 0.065)),
        origin=Origin(xyz=(0.150, -0.010, 0.055)),
        material=painted_steel,
        name="hinge_cheek_1",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.840, 0.480, 0.025)),
        origin=Origin(xyz=(0.0, -0.250, 0.032)),
        material=tray_plastic,
        name="tray_panel",
    )
    tray.visual(
        Box((0.850, 0.032, 0.052)),
        origin=Origin(xyz=(0.0, -0.492, 0.071)),
        material=tray_plastic,
        name="front_lip",
    )
    tray.visual(
        Box((0.850, 0.028, 0.045)),
        origin=Origin(xyz=(0.0, -0.010, 0.068)),
        material=tray_plastic,
        name="rear_lip",
    )
    tray.visual(
        Box((0.032, 0.480, 0.047)),
        origin=Origin(xyz=(-0.425, -0.250, 0.068)),
        material=tray_plastic,
        name="side_lip_0",
    )
    tray.visual(
        Box((0.032, 0.480, 0.047)),
        origin=Origin(xyz=(0.425, -0.250, 0.068)),
        material=tray_plastic,
        name="side_lip_1",
    )
    tray.visual(
        Box((0.600, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, -0.075, 0.005)),
        material=tray_plastic,
        name="rear_under_rib",
    )
    tray.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(-0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="tray_hinge_knuckle_0",
    )
    tray.visual(
        Box((0.058, 0.032, 0.018)),
        origin=Origin(xyz=(-0.085, -0.016, 0.012)),
        material=brushed_steel,
        name="hinge_leaf_0",
    )
    tray.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="tray_hinge_knuckle_1",
    )
    tray.visual(
        Box((0.058, 0.032, 0.018)),
        origin=Origin(xyz=(0.085, -0.016, 0.012)),
        material=brushed_steel,
        name="hinge_leaf_1",
    )
    tray.visual(
        Box((0.028, 0.030, 0.045)),
        origin=Origin(xyz=(-0.155, -0.488, 0.000)),
        material=release_gray,
        name="lever_pivot_lug_0",
    )
    tray.visual(
        Box((0.028, 0.030, 0.045)),
        origin=Origin(xyz=(0.155, -0.488, 0.000)),
        material=release_gray,
        name="lever_pivot_lug_1",
    )

    lever = model.part("release_lever")
    lever.visual(
        Cylinder(radius=0.008, length=0.282),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=release_gray,
        name="lever_pivot",
    )
    lever.visual(
        Box((0.300, 0.032, 0.014)),
        origin=Origin(xyz=(0.0, 0.055, -0.024)),
        material=release_gray,
        name="squeeze_bar",
    )
    lever.visual(
        Box((0.034, 0.070, 0.012)),
        origin=Origin(xyz=(-0.118, 0.030, -0.012)),
        material=release_gray,
        name="lever_arm_0",
    )
    lever.visual(
        Box((0.034, 0.070, 0.012)),
        origin=Origin(xyz=(0.118, 0.030, -0.012)),
        material=release_gray,
        name="lever_arm_1",
    )

    caster_parts = []
    lock_parts = []
    for index in range(4):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=0.041, length=0.028),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_rubber,
            name="rubber_wheel",
        )
        caster.visual(
            Cylinder(radius=0.018, length=0.038),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_black,
            name="wheel_hub",
        )
        caster_parts.append(caster)

        lock = model.part(f"lock_tab_{index}")
        lock.visual(
            Cylinder(radius=0.0065, length=0.030),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=lock_blue,
            name="lock_pivot",
        )
        lock.visual(
            Box((0.052, 0.034, 0.012)),
            origin=Origin(xyz=(0.0, -0.020, 0.007)),
            material=lock_blue,
            name="foot_tab",
        )
        lock.visual(
            Box((0.026, 0.038, 0.010)),
            origin=Origin(xyz=(0.0, -0.006, -0.010)),
            material=lock_blue,
            name="brake_tooth",
        )
        lock_parts.append(lock)

    lift = model.articulation(
        "column_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner,
        origin=Origin(xyz=(0.0, post_y, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.220),
    )
    model.articulation(
        "head_mount",
        ArticulationType.FIXED,
        parent=inner,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
    )
    tilt = model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=head,
        child=tray,
        origin=Origin(xyz=(0.0, -0.020, 0.090)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=-0.18, upper=0.55),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=lever,
        origin=Origin(xyz=(0.0, -0.492, 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=0.55),
    )

    for index, ((x, y), caster, lock) in enumerate(zip(caster_positions, caster_parts, lock_parts)):
        model.articulation(
            f"caster_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x, y, 0.046)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=25.0),
        )
        # Each brake tab has its own small local hinge next to its fork.
        outward = -1.0 if x < 0.0 else 1.0
        model.articulation(
            f"lock_tab_pivot_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=lock,
            origin=Origin(xyz=(x + outward * 0.080, y + 0.020, 0.102)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=0.0, upper=0.85),
        )

    # Keep references alive for targeted tests by name.
    _ = (lift, tilt)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    inner = object_model.get_part("inner_column")
    tray = object_model.get_part("tray")
    lever = object_model.get_part("release_lever")
    lift = object_model.get_articulation("column_lift")
    tilt = object_model.get_articulation("tray_tilt")
    lever_joint = object_model.get_articulation("lever_pivot")

    ctx.expect_overlap(
        inner,
        base,
        axes="z",
        elem_a="inner_bar",
        elem_b="outer_post_front_wall",
        min_overlap=0.18,
        name="inner column retained in outer post at low height",
    )
    with ctx.pose({lift: 0.220}):
        ctx.expect_overlap(
            inner,
            base,
            axes="z",
            elem_a="inner_bar",
            elem_b="outer_post_front_wall",
            min_overlap=0.08,
            name="inner column retained in outer post at full height",
        )

    low_pos = ctx.part_world_position(inner)
    with ctx.pose({lift: 0.220}):
        high_pos = ctx.part_world_position(inner)
    ctx.check(
        "column lift moves tray support upward",
        low_pos is not None and high_pos is not None and high_pos[2] > low_pos[2] + 0.20,
        details=f"low={low_pos}, high={high_pos}",
    )

    flat_front = ctx.part_element_world_aabb(tray, elem="front_lip")
    with ctx.pose({tilt: 0.55}):
        tilted_front = ctx.part_element_world_aabb(tray, elem="front_lip")
    flat_z = flat_front[0][2] if flat_front is not None else None
    tilted_z = tilted_front[0][2] if tilted_front is not None else None
    ctx.check(
        "positive tray tilt lifts the front lip",
        flat_z is not None and tilted_z is not None and tilted_z > flat_z + 0.10,
        details=f"flat_front_min_z={flat_z}, tilted_front_min_z={tilted_z}",
    )

    rest_lever = ctx.part_element_world_aabb(lever, elem="squeeze_bar")
    with ctx.pose({lever_joint: 0.55}):
        squeezed_lever = ctx.part_element_world_aabb(lever, elem="squeeze_bar")
    rest_lever_z = rest_lever[1][2] if rest_lever is not None else None
    squeezed_lever_z = squeezed_lever[1][2] if squeezed_lever is not None else None
    ctx.check(
        "squeeze lever rotates upward toward the tray",
        rest_lever_z is not None
        and squeezed_lever_z is not None
        and squeezed_lever_z > rest_lever_z + 0.015,
        details=f"rest={rest_lever_z}, squeezed={squeezed_lever_z}",
    )

    ctx.check(
        "four casters have continuous wheel spin joints",
        all(object_model.get_articulation(f"caster_spin_{i}") is not None for i in range(4)),
    )
    ctx.check(
        "four caster lock tabs have local pivots",
        all(object_model.get_articulation(f"lock_tab_pivot_{i}") is not None for i in range(4)),
    )

    return ctx.report()


object_model = build_object_model()
