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
    model = ArticulatedObject(name="pneumatic_overbed_table")

    model.material("powder_white", rgba=(0.92, 0.94, 0.93, 1.0))
    model.material("warm_tray", rgba=(0.78, 0.70, 0.56, 1.0))
    model.material("soft_gray", rgba=(0.45, 0.48, 0.50, 1.0))
    model.material("dark_rubber", rgba=(0.02, 0.022, 0.024, 1.0))
    model.material("brushed_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    model.material("release_blue", rgba=(0.08, 0.22, 0.55, 1.0))
    model.material("lock_red", rgba=(0.78, 0.08, 0.04, 1.0))

    base = model.part("base")
    # Low rolling C-frame: two forward legs and a rear cross tube, open at the front.
    base.visual(
        Box((0.84, 0.070, 0.055)),
        origin=Origin(xyz=(0.060, 0.300, 0.100)),
        material="powder_white",
        name="side_rail_0",
    )
    base.visual(
        Box((0.84, 0.070, 0.055)),
        origin=Origin(xyz=(0.060, -0.300, 0.100)),
        material="powder_white",
        name="side_rail_1",
    )
    base.visual(
        Box((0.090, 0.670, 0.055)),
        origin=Origin(xyz=(-0.360, 0.0, 0.100)),
        material="powder_white",
        name="rear_crossbar",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.045),
        origin=Origin(xyz=(-0.360, 0.0, 0.105)),
        material="soft_gray",
        name="column_foot_boss",
    )

    outer_sleeve = model.part("outer_sleeve")
    # A rectangular tube cover, built from separate walls so the sliding mast is visibly nested.
    outer_sleeve.visual(
        Box((0.094, 0.010, 0.460)),
        origin=Origin(xyz=(0.0, 0.037, 0.230)),
        material="powder_white",
        name="front_wall",
    )
    outer_sleeve.visual(
        Box((0.094, 0.010, 0.460)),
        origin=Origin(xyz=(0.0, -0.037, 0.230)),
        material="powder_white",
        name="rear_wall",
    )
    outer_sleeve.visual(
        Box((0.012, 0.074, 0.460)),
        origin=Origin(xyz=(0.047, 0.0, 0.230)),
        material="powder_white",
        name="side_wall_0",
    )
    outer_sleeve.visual(
        Box((0.012, 0.074, 0.460)),
        origin=Origin(xyz=(-0.047, 0.0, 0.230)),
        material="powder_white",
        name="side_wall_1",
    )
    outer_sleeve.visual(
        Box((0.104, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.043, 0.461)),
        material="soft_gray",
        name="top_lip_front",
    )
    outer_sleeve.visual(
        Box((0.104, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.043, 0.461)),
        material="soft_gray",
        name="top_lip_rear",
    )
    outer_sleeve.visual(
        Box((0.016, 0.086, 0.018)),
        origin=Origin(xyz=(0.052, 0.0, 0.461)),
        material="soft_gray",
        name="top_lip_side_0",
    )
    outer_sleeve.visual(
        Box((0.016, 0.086, 0.018)),
        origin=Origin(xyz=(-0.052, 0.0, 0.461)),
        material="soft_gray",
        name="top_lip_side_1",
    )
    outer_sleeve.visual(
        Box((0.100, 0.082, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="soft_gray",
        name="bottom_collar",
    )

    lift_column = model.part("lift_column")
    lift_column.visual(
        Box((0.052, 0.036, 0.600)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material="brushed_steel",
        name="inner_mast",
    )
    lift_column.visual(
        Cylinder(radius=0.010, length=0.555),
        origin=Origin(xyz=(-0.023, -0.016, -0.060)),
        material="soft_gray",
        name="gas_spring_cover",
    )
    lift_column.visual(
        Cylinder(radius=0.006, length=0.390),
        origin=Origin(xyz=(-0.023, -0.016, 0.020)),
        material="brushed_steel",
        name="piston_rod",
    )
    lift_column.visual(
        Box((0.110, 0.110, 0.048)),
        origin=Origin(xyz=(0.000, 0.0, 0.258)),
        material="powder_white",
        name="head_block",
    )
    lift_column.visual(
        Box((0.016, 0.016, 0.055)),
        origin=Origin(xyz=(0.0335, 0.0, -0.180)),
        material="soft_gray",
        name="guide_pad_0",
    )
    lift_column.visual(
        Box((0.016, 0.016, 0.055)),
        origin=Origin(xyz=(-0.0335, 0.0, -0.180)),
        material="soft_gray",
        name="guide_pad_1",
    )
    lift_column.visual(
        Box((0.018, 0.016, 0.055)),
        origin=Origin(xyz=(0.0, 0.0250, -0.180)),
        material="soft_gray",
        name="guide_pad_2",
    )
    lift_column.visual(
        Box((0.018, 0.016, 0.055)),
        origin=Origin(xyz=(0.0, -0.0250, -0.180)),
        material="soft_gray",
        name="guide_pad_3",
    )
    lift_column.visual(
        Box((0.360, 0.070, 0.026)),
        origin=Origin(xyz=(0.160, 0.0, 0.252)),
        material="powder_white",
        name="under_tray_arm",
    )
    lift_column.visual(
        Cylinder(radius=0.010, length=0.500),
        origin=Origin(xyz=(0.060, 0.0, 0.292), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="tilt_pin",
    )
    lift_column.visual(
        Box((0.030, 0.500, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, 0.274)),
        material="powder_white",
        name="pin_support_bar",
    )
    lift_column.visual(
        Box((0.026, 0.018, 0.020)),
        origin=Origin(xyz=(0.060, 0.240, 0.284)),
        material="powder_white",
        name="pin_end_clamp_0",
    )
    lift_column.visual(
        Box((0.026, 0.018, 0.020)),
        origin=Origin(xyz=(0.060, -0.240, 0.284)),
        material="powder_white",
        name="pin_end_clamp_1",
    )

    tray = model.part("tray")
    # Tray frame origin is the tilt-hinge line; the work surface cantilevers forward.
    tray.visual(
        Box((0.780, 0.460, 0.026)),
        origin=Origin(xyz=(0.420, 0.0, 0.022)),
        material="warm_tray",
        name="tray_panel",
    )
    tray.visual(
        Box((0.790, 0.028, 0.040)),
        origin=Origin(xyz=(0.425, 0.244, 0.043)),
        material="soft_gray",
        name="right_rim",
    )
    tray.visual(
        Box((0.790, 0.028, 0.040)),
        origin=Origin(xyz=(0.425, -0.244, 0.043)),
        material="soft_gray",
        name="left_rim",
    )
    tray.visual(
        Box((0.034, 0.460, 0.040)),
        origin=Origin(xyz=(0.822, 0.0, 0.043)),
        material="soft_gray",
        name="front_rim",
    )
    tray.visual(
        Box((0.030, 0.460, 0.040)),
        origin=Origin(xyz=(0.022, 0.0, 0.043)),
        material="soft_gray",
        name="hinge_rim",
    )
    tray.visual(
        Cylinder(radius=0.012, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="tilt_barrel",
    )
    tray.visual(
        Box((0.044, 0.030, 0.026)),
        origin=Origin(xyz=(0.014, 0.150, 0.014)),
        material="brushed_steel",
        name="hinge_strap_0",
    )
    tray.visual(
        Box((0.044, 0.030, 0.026)),
        origin=Origin(xyz=(0.014, -0.150, 0.014)),
        material="brushed_steel",
        name="hinge_strap_1",
    )
    tray.visual(
        Box((0.040, 0.035, 0.030)),
        origin=Origin(xyz=(0.290, 0.232, -0.0055)),
        material="soft_gray",
        name="release_lug_0",
    )
    tray.visual(
        Box((0.040, 0.035, 0.030)),
        origin=Origin(xyz=(0.405, 0.232, -0.0055)),
        material="soft_gray",
        name="release_lug_1",
    )

    release_paddle = model.part("release_paddle")
    release_paddle.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="paddle_pivot",
    )
    release_paddle.visual(
        Box((0.165, 0.036, 0.012)),
        origin=Origin(xyz=(0.030, 0.035, -0.018)),
        material="release_blue",
        name="paddle_plate",
    )
    release_paddle.visual(
        Box((0.112, 0.020, 0.018)),
        origin=Origin(xyz=(0.010, 0.010, -0.006)),
        material="release_blue",
        name="paddle_web",
    )

    model.articulation(
        "base_to_outer_sleeve",
        ArticulationType.FIXED,
        parent=base,
        child=outer_sleeve,
        origin=Origin(xyz=(-0.360, 0.0, 0.1275)),
    )
    model.articulation(
        "sleeve_to_lift_column",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=lift_column,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.240),
    )
    model.articulation(
        "lift_column_to_tray",
        ArticulationType.REVOLUTE,
        parent=lift_column,
        child=tray,
        origin=Origin(xyz=(0.060, 0.0, 0.292)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.8,
            lower=math.radians(-8.0),
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "tray_to_release_paddle",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=release_paddle,
        origin=Origin(xyz=(0.345, 0.248, -0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=math.radians(-28.0), upper=math.radians(18.0)),
    )

    caster_mounts = [
        ("0", (0.455, 0.300, 0.073)),
        ("1", (0.455, -0.300, 0.073)),
        ("2", (-0.360, 0.300, 0.073)),
        ("3", (-0.360, -0.300, 0.073)),
    ]
    for suffix, mount_xyz in caster_mounts:
        fork = model.part(f"caster_fork_{suffix}")
        fork.visual(
            Box((0.072, 0.076, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material="soft_gray",
            name="top_plate",
        )
        fork.visual(
            Box((0.010, 0.070, 0.066)),
            origin=Origin(xyz=(0.026, 0.0, -0.039)),
            material="soft_gray",
            name="fork_cheek_0",
        )
        fork.visual(
            Box((0.010, 0.070, 0.066)),
            origin=Origin(xyz=(-0.026, 0.0, -0.039)),
            material="soft_gray",
            name="fork_cheek_1",
        )
        fork.visual(
            Box((0.072, 0.010, 0.044)),
            origin=Origin(xyz=(0.0, -0.0385, -0.036)),
            material="soft_gray",
            name="fork_bridge",
        )
        fork.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(xyz=(0.037, 0.0, -0.043), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_steel",
            name="axle_cap_0",
        )
        fork.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(xyz=(-0.037, 0.0, -0.043), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_steel",
            name="axle_cap_1",
        )

        wheel = model.part(f"caster_wheel_{suffix}")
        wheel.visual(
            Cylinder(radius=0.032, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dark_rubber",
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.016, length=0.042),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="soft_gray",
            name="wheel_hub",
        )
        wheel.visual(
            Cylinder(radius=0.006, length=0.044),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_steel",
            name="axle_bushing",
        )

        lock_tab = model.part(f"lock_tab_{suffix}")
        lock_tab.visual(
            Cylinder(radius=0.0055, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_steel",
            name="lock_pivot",
        )
        lock_tab.visual(
            Box((0.010, 0.048, 0.007)),
            origin=Origin(xyz=(0.008, 0.026, -0.003)),
            material="lock_red",
            name="lock_pedal",
        )
        lock_tab.visual(
            Box((0.006, 0.022, 0.018)),
            origin=Origin(xyz=(0.008, 0.006, -0.010)),
            material="lock_red",
            name="lock_tooth",
        )

        model.articulation(
            f"base_to_caster_fork_{suffix}",
            ArticulationType.FIXED,
            parent=base,
            child=fork,
            origin=Origin(xyz=mount_xyz),
        )
        model.articulation(
            f"fork_to_caster_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.043)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=25.0),
        )
        model.articulation(
            f"fork_to_lock_tab_{suffix}",
            ArticulationType.REVOLUTE,
            parent=fork,
            child=lock_tab,
            origin=Origin(xyz=(0.037, 0.020, -0.026)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=math.radians(-38.0), upper=math.radians(24.0)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lift_joint = object_model.get_articulation("sleeve_to_lift_column")
    tilt_joint = object_model.get_articulation("lift_column_to_tray")
    paddle_joint = object_model.get_articulation("tray_to_release_paddle")
    outer_sleeve = object_model.get_part("outer_sleeve")
    lift_column = object_model.get_part("lift_column")
    tray = object_model.get_part("tray")
    release_paddle = object_model.get_part("release_paddle")

    ctx.allow_overlap(
        lift_column,
        tray,
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        reason="The tray tilt hinge has a captured steel pin intentionally passing through the barrel.",
    )
    ctx.allow_overlap(
        lift_column,
        tray,
        elem_a="tilt_pin",
        elem_b="hinge_strap_0",
        reason="The hinge leaf strap wraps the same captured steel pin as the tilt barrel.",
    )
    ctx.allow_overlap(
        lift_column,
        tray,
        elem_a="tilt_pin",
        elem_b="hinge_strap_1",
        reason="The hinge leaf strap wraps the same captured steel pin as the tilt barrel.",
    )
    ctx.allow_overlap(
        release_paddle,
        tray,
        elem_a="paddle_pivot",
        elem_b="release_lug_0",
        reason="The release paddle pivot shaft is intentionally captured by the underside hinge lug.",
    )
    ctx.allow_overlap(
        release_paddle,
        tray,
        elem_a="paddle_pivot",
        elem_b="release_lug_1",
        reason="The release paddle pivot shaft is intentionally captured by the underside hinge lug.",
    )

    ctx.check(
        "primary lift is prismatic",
        lift_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={lift_joint.articulation_type}",
    )
    ctx.check(
        "tray uses a tilt hinge",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={tilt_joint.articulation_type}",
    )
    ctx.check(
        "release paddle pivots",
        paddle_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={paddle_joint.articulation_type}",
    )
    for i in range(4):
        wheel_joint = object_model.get_articulation(f"fork_to_caster_wheel_{i}")
        lock_joint = object_model.get_articulation(f"fork_to_lock_tab_{i}")
        ctx.check(
            f"caster wheel {i} spins continuously",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={wheel_joint.articulation_type}",
        )
        ctx.check(
            f"caster lock tab {i} pivots",
            lock_joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={lock_joint.articulation_type}",
        )

    ctx.expect_within(
        lift_column,
        outer_sleeve,
        axes="xy",
        inner_elem="inner_mast",
        margin=0.002,
        name="inner mast remains centered inside sleeve footprint",
    )
    ctx.expect_overlap(
        lift_column,
        outer_sleeve,
        axes="z",
        elem_a="inner_mast",
        min_overlap=0.300,
        name="lowered lift column remains deeply inserted",
    )
    with ctx.pose({lift_joint: 0.240}):
        ctx.expect_overlap(
            lift_column,
            outer_sleeve,
            axes="z",
            elem_a="inner_mast",
            min_overlap=0.090,
            name="raised lift column retains insertion",
        )

    ctx.expect_overlap(
        lift_column,
        tray,
        axes="y",
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        min_overlap=0.400,
        name="tilt pin spans the tray hinge barrel",
    )
    ctx.expect_overlap(
        lift_column,
        tray,
        axes="y",
        elem_a="tilt_pin",
        elem_b="hinge_strap_0",
        min_overlap=0.020,
        name="tilt pin passes through first hinge strap",
    )
    ctx.expect_overlap(
        lift_column,
        tray,
        axes="y",
        elem_a="tilt_pin",
        elem_b="hinge_strap_1",
        min_overlap=0.020,
        name="tilt pin passes through second hinge strap",
    )
    ctx.expect_overlap(
        release_paddle,
        tray,
        axes="x",
        elem_a="paddle_pivot",
        elem_b="release_lug_0",
        min_overlap=0.020,
        name="release pivot passes through first lug",
    )
    ctx.expect_overlap(
        release_paddle,
        tray,
        axes="x",
        elem_a="paddle_pivot",
        elem_b="release_lug_1",
        min_overlap=0.019,
        name="release pivot passes through second lug",
    )

    flat_aabb = ctx.part_world_aabb(tray)
    with ctx.pose({tilt_joint: math.radians(45.0)}):
        tilted_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "tray free edge rises when tilted",
        flat_aabb is not None and tilted_aabb is not None and tilted_aabb[1][2] > flat_aabb[1][2] + 0.18,
        details=f"flat={flat_aabb}, tilted={tilted_aabb}",
    )

    rest_paddle = ctx.part_world_aabb(release_paddle)
    with ctx.pose({paddle_joint: math.radians(-24.0)}):
        pulled_paddle = ctx.part_world_aabb(release_paddle)
    ctx.check(
        "release paddle swings downward",
        rest_paddle is not None and pulled_paddle is not None and pulled_paddle[0][2] < rest_paddle[0][2] - 0.006,
        details=f"rest={rest_paddle}, pulled={pulled_paddle}",
    )

    return ctx.report()


object_model = build_object_model()
