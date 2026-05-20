from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_overbed_table")

    satin = model.material("satin_stainless", rgba=(0.72, 0.74, 0.74, 1.0))
    dark = model.material("dark_hardware", rgba=(0.06, 0.065, 0.07, 1.0))
    tray_plastic = model.material("warm_white_tray", rgba=(0.92, 0.90, 0.84, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    hub = model.material("caster_hub_grey", rgba=(0.62, 0.64, 0.66, 1.0))
    lever_blue = model.material("release_blue", rgba=(0.05, 0.23, 0.62, 1.0))
    brake_red = model.material("brake_red", rgba=(0.72, 0.05, 0.03, 1.0))

    base = model.part("base_frame")
    # C/U shaped rolling base: two long rails remain open at the front for sliding
    # beneath a bed, with a heavier rear cross member and column socket.
    base.visual(
        Box((0.055, 0.68, 0.035)),
        origin=Origin(xyz=(-0.32, -0.07, 0.105)),
        material=satin,
        name="side_rail_0",
    )
    base.visual(
        Box((0.055, 0.68, 0.035)),
        origin=Origin(xyz=(0.32, -0.07, 0.105)),
        material=satin,
        name="side_rail_1",
    )
    base.visual(
        Box((0.70, 0.060, 0.035)),
        origin=Origin(xyz=(0.0, 0.235, 0.105)),
        material=satin,
        name="rear_cross_rail",
    )
    base.visual(
        Box((0.14, 0.12, 0.035)),
        origin=Origin(xyz=(0.0, 0.205, 0.139)),
        material=satin,
        name="column_socket_plate",
    )

    # Supported brake-bar pivots on the two lower rails.
    base.visual(
        Box((0.030, 0.045, 0.040)),
        origin=Origin(xyz=(-0.2775, -0.300, 0.0675)),
        material=dark,
        name="brake_pivot_0",
    )
    base.visual(
        Box((0.030, 0.045, 0.040)),
        origin=Origin(xyz=(0.2775, -0.300, 0.0675)),
        material=dark,
        name="brake_pivot_1",
    )

    # Four caster forks are rigidly welded to the lower rails.  The wheels are
    # separate continuous-joint parts below.
    caster_locations = [
        (-0.32, -0.36),
        (0.32, -0.36),
        (-0.32, 0.22),
        (0.32, 0.22),
    ]
    for index, (x, y) in enumerate(caster_locations):
        base.visual(
            Cylinder(radius=0.012, length=0.060),
            origin=Origin(xyz=(x, y, 0.132)),
            material=satin,
            name=f"caster_stem_{index}",
        )
        base.visual(
            Box((0.075, 0.038, 0.012)),
            origin=Origin(xyz=(x, y, 0.095)),
            material=satin,
            name=f"caster_crown_{index}",
        )
        for side in (-1, 1):
            base.visual(
                Box((0.008, 0.024, 0.088)),
                origin=Origin(xyz=(x + side * 0.027, y, 0.055)),
                material=satin,
                name=f"caster_fork_{index}_{0 if side < 0 else 1}",
            )

    outer = model.part("outer_post")
    # Hollow square outer sleeve, open in the middle for the sliding inner post.
    outer.visual(
        Box((0.012, 0.082, 0.450)),
        origin=Origin(xyz=(-0.035, 0.0, 0.225)),
        material=satin,
        name="post_wall_x0",
    )
    outer.visual(
        Box((0.012, 0.082, 0.450)),
        origin=Origin(xyz=(0.035, 0.0, 0.225)),
        material=satin,
        name="post_wall_x1",
    )
    outer.visual(
        Box((0.082, 0.012, 0.450)),
        origin=Origin(xyz=(0.0, -0.035, 0.225)),
        material=satin,
        name="post_wall_y0",
    )
    outer.visual(
        Box((0.082, 0.012, 0.450)),
        origin=Origin(xyz=(0.0, 0.035, 0.225)),
        material=satin,
        name="post_wall_y1",
    )
    outer.visual(
        Box((0.100, 0.100, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin,
        name="lower_collar",
    )
    outer.visual(
        Box((0.100, 0.100, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        material=satin,
        name="upper_collar",
    )

    inner = model.part("inner_column")
    inner.visual(
        Box((0.044, 0.044, 0.560)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin,
        name="sliding_tube",
    )
    inner.visual(
        Box((0.054, 0.054, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.294)),
        material=satin,
        name="top_cap",
    )

    head = model.part("support_head")
    head.visual(
        Box((0.220, 0.120, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=satin,
        name="head_block",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.250),
        origin=Origin(xyz=(0.0, -0.020, 0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="tilt_hinge_barrel",
    )
    for x in (-0.080, 0.080):
        head.visual(
            Box((0.018, 0.090, 0.045)),
            origin=Origin(xyz=(x, -0.005, 0.028)),
            material=satin,
            name=f"head_gusset_{0 if x < 0 else 1}",
        )

    tray = model.part("tray")
    tray.visual(
        Box((0.820, 0.420, 0.025)),
        origin=Origin(xyz=(0.0, -0.095, 0.028)),
        material=tray_plastic,
        name="tray_panel",
    )
    tray.visual(
        Box((0.820, 0.025, 0.055)),
        origin=Origin(xyz=(0.0, -0.2925, 0.057)),
        material=tray_plastic,
        name="front_lip",
    )
    tray.visual(
        Box((0.820, 0.025, 0.055)),
        origin=Origin(xyz=(0.0, 0.1025, 0.057)),
        material=tray_plastic,
        name="rear_lip",
    )
    for x in (-0.3975, 0.3975):
        tray.visual(
            Box((0.025, 0.420, 0.055)),
            origin=Origin(xyz=(x, -0.095, 0.057)),
            material=tray_plastic,
            name=f"side_lip_{0 if x < 0 else 1}",
        )
    tray.visual(
        Box((0.240, 0.080, 0.014)),
        origin=Origin(xyz=(0.0, -0.020, 0.010)),
        material=dark,
        name="hinge_leaf",
    )
    for x in (-0.200, 0.200):
        tray.visual(
            Box((0.018, 0.026, 0.040)),
            origin=Origin(xyz=(x, -0.300, -0.004)),
            material=dark,
            name=f"lever_hanger_{0 if x < 0 else 1}",
        )

    lever = model.part("squeeze_lever")
    lever.visual(
        Cylinder(radius=0.008, length=0.382),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="lever_hinge_rod",
    )
    lever.visual(
        Box((0.320, 0.050, 0.016)),
        origin=Origin(xyz=(0.0, -0.025, -0.012)),
        material=lever_blue,
        name="squeeze_paddle",
    )

    brake = model.part("brake_bar")
    brake.visual(
        Cylinder(radius=0.014, length=0.525),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brake_red,
        name="cross_bar",
    )
    brake.visual(
        Box((0.280, 0.055, 0.014)),
        origin=Origin(xyz=(0.0, -0.020, 0.014)),
        material=brake_red,
        name="foot_pad",
    )

    # Wheel visuals are authored as four identical articulated caster wheels.
    for index, (x, y) in enumerate(caster_locations):
        wheel = model.part(f"caster_wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.038, length=0.030),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="rubber_tire",
        )
        wheel.visual(
            Cylinder(radius=0.020, length=0.046),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hub,
            name="hub_disc",
        )
        model.articulation(
            f"base_to_caster_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(xyz=(x, y, 0.040)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=12.0),
        )

    model.articulation(
        "base_to_outer",
        ArticulationType.FIXED,
        parent=base,
        child=outer,
        origin=Origin(xyz=(0.0, 0.205, 0.156)),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 0.450)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.220),
    )
    model.articulation(
        "inner_to_head",
        ArticulationType.FIXED,
        parent=inner,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.303)),
    )
    model.articulation(
        "head_to_tray",
        ArticulationType.REVOLUTE,
        parent=head,
        child=tray,
        origin=Origin(xyz=(0.0, -0.020, 0.056)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=-0.25, upper=0.70),
    )
    model.articulation(
        "tray_to_lever",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=lever,
        origin=Origin(xyz=(0.0, -0.315, -0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=0.45),
    )
    model.articulation(
        "base_to_brake",
        ArticulationType.REVOLUTE,
        parent=base,
        child=brake,
        origin=Origin(xyz=(0.0, -0.300, 0.0675)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.35, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    outer = object_model.get_part("outer_post")
    inner = object_model.get_part("inner_column")
    head = object_model.get_part("support_head")
    tray = object_model.get_part("tray")
    brake = object_model.get_part("brake_bar")
    lever = object_model.get_part("squeeze_lever")

    lift = object_model.get_articulation("outer_to_inner")
    tilt = object_model.get_articulation("head_to_tray")
    brake_joint = object_model.get_articulation("base_to_brake")
    lever_joint = object_model.get_articulation("tray_to_lever")

    ctx.allow_overlap(
        inner,
        outer,
        elem_a="sliding_tube",
        elem_b="upper_collar",
        reason="The inner lift tube intentionally passes through the simplified solid upper collar of the outer sleeve.",
    )
    ctx.allow_overlap(
        head,
        tray,
        elem_a="tilt_hinge_barrel",
        elem_b="hinge_leaf",
        reason="The tray hinge leaf is simplified as a plate captured around the hinge barrel at the support head.",
    )

    ctx.check(
        "inner column uses prismatic lift",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"found {lift.articulation_type}",
    )
    ctx.check(
        "tray uses horizontal tilt hinge",
        tilt.articulation_type == ArticulationType.REVOLUTE and abs(tilt.axis[0]) > 0.9,
        details=f"type={tilt.articulation_type}, axis={tilt.axis}",
    )
    ctx.check(
        "brake bar uses horizontal pivots",
        brake_joint.articulation_type == ArticulationType.REVOLUTE and abs(brake_joint.axis[0]) > 0.9,
        details=f"type={brake_joint.articulation_type}, axis={brake_joint.axis}",
    )
    ctx.check(
        "release lever is articulated",
        lever_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"found {lever_joint.articulation_type}",
    )
    for index in range(4):
        caster_joint = object_model.get_articulation(f"base_to_caster_{index}")
        ctx.check(
            f"caster {index} spins continuously",
            caster_joint.articulation_type == ArticulationType.CONTINUOUS and abs(caster_joint.axis[0]) > 0.9,
            details=f"type={caster_joint.articulation_type}, axis={caster_joint.axis}",
        )

    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem="sliding_tube",
        outer_elem="upper_collar",
        margin=0.002,
        name="inner column is centered in outer sleeve",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="sliding_tube",
        elem_b="post_wall_x0",
        min_overlap=0.20,
        name="collapsed lift remains inserted in outer post",
    )
    rest_head = ctx.part_world_position(head)
    with ctx.pose({lift: 0.220}):
        ctx.expect_within(
            inner,
            outer,
            axes="xy",
            inner_elem="sliding_tube",
            outer_elem="upper_collar",
            margin=0.002,
            name="raised column remains centered in sleeve",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a="sliding_tube",
            elem_b="post_wall_x0",
            min_overlap=0.03,
            name="raised lift preserves retained insertion",
        )
        raised_head = ctx.part_world_position(head)
    ctx.check(
        "lift travel raises support head",
        rest_head is not None and raised_head is not None and raised_head[2] > rest_head[2] + 0.20,
        details=f"rest={rest_head}, raised={raised_head}",
    )

    ctx.expect_gap(
        tray,
        head,
        axis="z",
        min_gap=0.0,
        max_gap=0.030,
        positive_elem="tray_panel",
        negative_elem="tilt_hinge_barrel",
        name="tray clears hinge barrel while seated close",
    )
    ctx.expect_gap(
        base,
        brake,
        axis="z",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem="side_rail_0",
        negative_elem="cross_bar",
        name="brake bar stays below the left rail",
    )
    ctx.expect_overlap(
        brake,
        base,
        axes="x",
        elem_a="cross_bar",
        elem_b="brake_pivot_0",
        min_overlap=0.0,
        name="brake bar reaches left pivot",
    )
    ctx.expect_gap(
        tray,
        lever,
        axis="z",
        positive_elem="tray_panel",
        negative_elem="lever_hinge_rod",
        min_gap=0.0,
        max_gap=0.050,
        name="squeeze lever hangs below tray front",
    )
    ctx.expect_overlap(
        tray,
        head,
        axes="x",
        elem_a="hinge_leaf",
        elem_b="tilt_hinge_barrel",
        min_overlap=0.20,
        name="tray hinge leaf captures hinge barrel length",
    )

    return ctx.report()


object_model = build_object_model()
