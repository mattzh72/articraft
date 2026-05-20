from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="desk_post_monitor_arm")

    dark_metal = model.material("dark_powder_coat", rgba=(0.05, 0.055, 0.06, 1.0))
    satin_metal = model.material("satin_hardware", rgba=(0.55, 0.56, 0.54, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    plastic_black = model.material("black_plastic", rgba=(0.02, 0.022, 0.026, 1.0))

    clamp_post = model.part("clamp_post")
    clamp_post.visual(
        Cylinder(radius=0.023, length=0.660),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=satin_metal,
        name="upright_post",
    )
    clamp_post.visual(
        Box((0.180, 0.110, 0.018)),
        origin=Origin(xyz=(-0.035, 0.0, 0.025)),
        material=dark_metal,
        name="desk_top_jaw",
    )
    clamp_post.visual(
        Box((0.026, 0.110, 0.130)),
        origin=Origin(xyz=(-0.120, 0.0, -0.025)),
        material=dark_metal,
        name="clamp_spine",
    )
    clamp_post.visual(
        Box((0.150, 0.100, 0.018)),
        origin=Origin(xyz=(-0.048, 0.0, -0.084)),
        material=dark_metal,
        name="lower_jaw",
    )
    clamp_post.visual(
        Cylinder(radius=0.012, length=0.082),
        origin=Origin(xyz=(0.015, 0.0, -0.126)),
        material=satin_metal,
        name="clamp_screw",
    )
    clamp_post.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.015, 0.0, -0.171)),
        material=black_rubber,
        name="pressure_pad",
    )
    clamp_post.visual(
        Box((0.085, 0.090, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.038)),
        material=black_rubber,
        name="desk_pad",
    )

    primary_link = model.part("primary_link")
    # Square split collar around the post: the open center clears the upright post
    # while the link bars bolt to the outer clamp blocks.
    primary_link.visual(
        Box((0.016, 0.086, 0.070)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=dark_metal,
        name="collar_side_0",
    )
    primary_link.visual(
        Box((0.016, 0.086, 0.070)),
        origin=Origin(xyz=(-0.036, 0.0, 0.0)),
        material=dark_metal,
        name="collar_side_1",
    )
    primary_link.visual(
        Box((0.064, 0.016, 0.070)),
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
        material=dark_metal,
        name="collar_face_0",
    )
    primary_link.visual(
        Box((0.064, 0.016, 0.070)),
        origin=Origin(xyz=(0.0, -0.036, 0.0)),
        material=dark_metal,
        name="collar_face_1",
    )
    primary_link.visual(
        Cylinder(radius=0.030, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="collar_bushing",
    )
    primary_link.visual(
        Cylinder(radius=0.007, length=0.052),
        origin=Origin(xyz=(0.0, -0.064, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="collar_lock_stem",
    )
    primary_link.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.0, -0.097, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=plastic_black,
        name="collar_lock_cap",
    )
    primary_link.visual(
        Box((0.360, 0.018, 0.050)),
        origin=Origin(xyz=(0.220, 0.042, 0.0)),
        material=dark_metal,
        name="primary_rail_0",
    )
    primary_link.visual(
        Box((0.360, 0.018, 0.050)),
        origin=Origin(xyz=(0.220, -0.042, 0.0)),
        material=dark_metal,
        name="primary_rail_1",
    )
    primary_link.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        material=satin_metal,
        name="elbow_pin",
    )
    primary_link.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.420, 0.0, 0.030)),
        material=dark_metal,
        name="elbow_top_cap",
    )
    primary_link.visual(
        Box((0.032, 0.102, 0.010)),
        origin=Origin(xyz=(0.405, 0.0, 0.026)),
        material=dark_metal,
        name="elbow_cap_bridge",
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        Cylinder(radius=0.026, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="elbow_bearing",
    )
    secondary_link.visual(
        Box((0.260, 0.020, 0.040)),
        origin=Origin(xyz=(0.130, 0.030, 0.0)),
        material=dark_metal,
        name="secondary_rail_0",
    )
    secondary_link.visual(
        Box((0.260, 0.020, 0.040)),
        origin=Origin(xyz=(0.130, -0.030, 0.0)),
        material=dark_metal,
        name="secondary_rail_1",
    )
    secondary_link.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        material=satin_metal,
        name="swivel_pin",
    )
    secondary_link.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.265, 0.0, 0.026)),
        material=dark_metal,
        name="swivel_top_cap",
    )
    secondary_link.visual(
        Box((0.032, 0.088, 0.010)),
        origin=Origin(xyz=(0.250, 0.0, 0.022)),
        material=dark_metal,
        name="swivel_cap_bridge",
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        Cylinder(radius=0.024, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="swivel_bearing",
    )
    head_swivel.visual(
        Box((0.040, 0.050, 0.020)),
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_neck",
    )
    head_swivel.visual(
        Box((0.025, 0.012, 0.055)),
        origin=Origin(xyz=(0.060, 0.026, 0.0)),
        material=dark_metal,
        name="tilt_fork_0",
    )
    head_swivel.visual(
        Box((0.025, 0.012, 0.055)),
        origin=Origin(xyz=(0.060, -0.026, 0.0)),
        material=dark_metal,
        name="tilt_fork_1",
    )
    head_swivel.visual(
        Cylinder(radius=0.006, length=0.085),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="tilt_pin",
    )

    vesa_head = model.part("vesa_head")
    vesa_head.visual(
        Cylinder(radius=0.016, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_barrel",
    )
    vesa_head.visual(
        Box((0.046, 0.030, 0.024)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=dark_metal,
        name="head_backbone",
    )
    vesa_head.visual(
        Box((0.016, 0.145, 0.012)),
        origin=Origin(xyz=(0.055, 0.0, 0.052)),
        material=dark_metal,
        name="vesa_top_bar",
    )
    vesa_head.visual(
        Box((0.016, 0.145, 0.012)),
        origin=Origin(xyz=(0.055, 0.0, -0.052)),
        material=dark_metal,
        name="vesa_bottom_bar",
    )
    vesa_head.visual(
        Box((0.016, 0.012, 0.104)),
        origin=Origin(xyz=(0.055, 0.072, 0.0)),
        material=dark_metal,
        name="vesa_side_0",
    )
    vesa_head.visual(
        Box((0.016, 0.012, 0.104)),
        origin=Origin(xyz=(0.055, -0.072, 0.0)),
        material=dark_metal,
        name="vesa_side_1",
    )
    vesa_head.visual(
        Box((0.016, 0.132, 0.010)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=dark_metal,
        name="vesa_cross_bar",
    )
    vesa_head.visual(
        Box((0.016, 0.012, 0.104)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=dark_metal,
        name="vesa_vertical_bar",
    )
    for y in (-0.050, 0.050):
        for z in (-0.050, 0.050):
            suffix = f"{0 if y < 0 else 1}_{0 if z < 0 else 1}"
            vesa_head.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(0.050, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=satin_metal,
                name=f"vesa_boss_{suffix}",
            )
            vesa_head.visual(
                Cylinder(radius=0.004, length=0.011),
                origin=Origin(xyz=(0.044, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=plastic_black,
                name=f"vesa_hole_{suffix}",
            )

    model.articulation(
        "collar_joint",
        ArticulationType.REVOLUTE,
        parent=clamp_post,
        child=primary_link,
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "head_swivel_joint",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=head_swivel,
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.6, upper=1.6),
    )
    model.articulation(
        "head_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=vesa_head,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.8, upper=0.8),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    primary = object_model.get_part("primary_link")
    secondary = object_model.get_part("secondary_link")
    head_swivel = object_model.get_part("head_swivel")
    vesa_head = object_model.get_part("vesa_head")

    collar_joint = object_model.get_articulation("collar_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    head_swivel_joint = object_model.get_articulation("head_swivel_joint")
    head_tilt_joint = object_model.get_articulation("head_tilt_joint")

    ctx.allow_overlap(
        "clamp_post",
        primary,
        elem_a="upright_post",
        elem_b="collar_bushing",
        reason="The collar bushing is a simplified sleeve wrapped around the post at the height-adjustable yaw joint.",
    )
    ctx.expect_within(
        "clamp_post",
        primary,
        axes="xy",
        inner_elem="upright_post",
        outer_elem="collar_bushing",
        margin=0.0,
        name="post passes through collar bushing",
    )
    ctx.expect_overlap(
        "clamp_post",
        primary,
        axes="z",
        elem_a="upright_post",
        elem_b="collar_bushing",
        min_overlap=0.060,
        name="collar grips a vertical span of post",
    )

    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_pin",
        elem_b="elbow_bearing",
        reason="The vertical elbow pin is intentionally captured inside the secondary bearing sleeve.",
    )
    ctx.expect_within(
        primary,
        secondary,
        axes="xy",
        inner_elem="elbow_pin",
        outer_elem="elbow_bearing",
        margin=0.0,
        name="elbow pin is centered in bearing",
    )
    ctx.expect_overlap(
        primary,
        secondary,
        axes="z",
        elem_a="elbow_pin",
        elem_b="elbow_bearing",
        min_overlap=0.030,
        name="elbow pin spans the bearing height",
    )

    ctx.allow_overlap(
        secondary,
        head_swivel,
        elem_a="swivel_pin",
        elem_b="swivel_bearing",
        reason="The head swivel pin is intentionally captured inside the compact yaw bearing.",
    )
    ctx.expect_within(
        secondary,
        head_swivel,
        axes="xy",
        inner_elem="swivel_pin",
        outer_elem="swivel_bearing",
        margin=0.0,
        name="head swivel pin is centered",
    )
    ctx.expect_overlap(
        secondary,
        head_swivel,
        axes="z",
        elem_a="swivel_pin",
        elem_b="swivel_bearing",
        min_overlap=0.028,
        name="head swivel pin spans bearing",
    )

    ctx.allow_overlap(
        head_swivel,
        vesa_head,
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        reason="The tilt pin passes through the VESA head barrel as a captured hinge shaft.",
    )
    ctx.expect_within(
        head_swivel,
        vesa_head,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="tilt_barrel",
        margin=0.0,
        name="tilt pin is coaxial with barrel",
    )
    ctx.expect_overlap(
        head_swivel,
        vesa_head,
        axes="y",
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        min_overlap=0.020,
        name="tilt pin crosses the barrel",
    )

    ctx.expect_origin_gap(
        secondary,
        primary,
        axis="x",
        min_gap=0.390,
        max_gap=0.450,
        name="primary link is the longer span",
    )
    ctx.expect_origin_gap(
        head_swivel,
        secondary,
        axis="x",
        min_gap=0.240,
        max_gap=0.290,
        name="secondary link is shorter than primary",
    )
    ctx.check(
        "collar and elbow rotate about vertical axes",
        tuple(collar_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(elbow_joint.axis) == (0.0, 0.0, 1.0),
        details=f"collar={collar_joint.axis}, elbow={elbow_joint.axis}",
    )
    ctx.check(
        "head has separate swivel and tilt axes",
        tuple(head_swivel_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(head_tilt_joint.axis) == (0.0, 1.0, 0.0),
        details=f"swivel={head_swivel_joint.axis}, tilt={head_tilt_joint.axis}",
    )

    rest_head = ctx.part_world_position(vesa_head)
    with ctx.pose({head_tilt_joint: 0.45, head_swivel_joint: 0.5}):
        moved_head = ctx.part_world_position(vesa_head)
    ctx.check(
        "compact head moves on swivel and tilt joints",
        rest_head is not None
        and moved_head is not None
        and abs(moved_head[1] - rest_head[1]) > 0.020,
        details=f"rest={rest_head}, moved={moved_head}",
    )

    return ctx.report()


object_model = build_object_model()
