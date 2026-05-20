from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_way_radio_handset")

    body_color = model.material("body_color", rgba=(0.16, 0.17, 0.19, 1.0))
    rubber_color = model.material("rubber_color", rgba=(0.08, 0.08, 0.09, 1.0))
    metal_color = model.material("metal_color", rgba=(0.74, 0.76, 0.79, 1.0))
    screen_color = model.material("screen_color", rgba=(0.08, 0.14, 0.18, 1.0))
    key_color = model.material("key_color", rgba=(0.24, 0.26, 0.29, 1.0))
    accent_color = model.material("accent_color", rgba=(0.46, 0.49, 0.53, 1.0))

    body = model.part("body")

    lower_shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.062, 0.038, 0.007),
        0.125,
    )
    body.visual(
        mesh_from_geometry(lower_shell, "radio_body_lower_shell"),
        material=body_color,
        name="lower_shell",
    )

    upper_shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.056, 0.034, 0.006),
        0.018,
    )
    upper_shell.translate(0.0, 0.0, 0.125)
    body.visual(
        mesh_from_geometry(upper_shell, "radio_body_upper_shell"),
        material=rubber_color,
        name="upper_shell",
    )

    body.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(-0.018, 0.0, 0.149), rpy=(0.0, 0.0, 0.0)),
        material=rubber_color,
        name="antenna_collar",
    )
    body.visual(
        Cylinder(radius=0.0075, length=0.013),
        origin=Origin(xyz=(0.018, 0.0, 0.1495), rpy=(0.0, 0.0, 0.0)),
        material=rubber_color,
        name="top_knob",
    )
    body.visual(
        Box((0.030, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.0185, 0.137)),
        material=accent_color,
        name="clip_mount_block",
    )

    body.visual(
        Box((0.030, 0.002, 0.023)),
        origin=Origin(xyz=(0.0, 0.0164, 0.126)),
        material=accent_color,
        name="speaker_panel",
    )
    body.visual(
        Box((0.024, 0.002, 0.018)),
        origin=Origin(xyz=(0.0, 0.0182, 0.096)),
        material=screen_color,
        name="display_window",
    )
    body.visual(
        Box((0.004, 0.016, 0.040)),
        origin=Origin(xyz=(-0.029, 0.0, 0.078)),
        material=accent_color,
        name="ptt_button",
    )

    key_x_positions = (-0.014, 0.0, 0.014)
    key_z_positions = (0.073, 0.059, 0.045, 0.031)
    for row_index, z_pos in enumerate(key_z_positions):
        for col_index, x_pos in enumerate(key_x_positions):
            key_name = f"key_{row_index}_{col_index}"
            body.visual(
                Box((0.009, 0.0024, 0.008)),
                origin=Origin(xyz=(x_pos, 0.0181, z_pos)),
                material=key_color,
                name=key_name,
            )

    antenna_stage1 = model.part("antenna_stage1")
    antenna_stage1.visual(
        Cylinder(radius=0.0055, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=rubber_color,
        name="outer_base",
    )
    antenna_stage1.visual(
        Cylinder(radius=0.0028, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=metal_color,
        name="outer_whip",
    )

    antenna_stage2 = model.part("antenna_stage2")
    antenna_stage2.visual(
        Cylinder(radius=0.0031, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=metal_color,
        name="inner_collar",
    )
    antenna_stage2.visual(
        Cylinder(radius=0.0016, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=metal_color,
        name="inner_whip",
    )
    antenna_stage2.visual(
        Cylinder(radius=0.0022, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
        material=metal_color,
        name="tip_cap",
    )

    belt_clip = model.part("belt_clip")
    belt_clip.visual(
        Cylinder(radius=0.003, length=0.026),
        origin=Origin(xyz=(0.0, -0.003, -0.0028), rpy=(0.0, pi / 2.0, 0.0)),
        material=accent_color,
        name="clip_barrel",
    )
    belt_clip.visual(
        Box((0.028, 0.003, 0.078)),
        origin=Origin(xyz=(0.0, -0.0026, -0.043)),
        material=accent_color,
        name="clip_leaf",
    )
    belt_clip.visual(
        Box((0.020, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.0035, -0.087)),
        material=accent_color,
        name="clip_toe",
    )

    model.articulation(
        "body_to_antenna_stage1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=antenna_stage1,
        origin=Origin(xyz=(-0.018, 0.0, 0.159)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.35,
            lower=0.0,
            upper=0.110,
        ),
    )
    model.articulation(
        "antenna_stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=antenna_stage1,
        child=antenna_stage2,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.35,
            lower=0.0,
            upper=0.120,
        ),
    )
    model.articulation(
        "body_to_belt_clip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=belt_clip,
        origin=Origin(xyz=(0.0, -0.0205, 0.140)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
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

    body = object_model.get_part("body")
    antenna_stage1 = object_model.get_part("antenna_stage1")
    antenna_stage2 = object_model.get_part("antenna_stage2")
    belt_clip = object_model.get_part("belt_clip")

    lower_shell = body.get_visual("lower_shell")
    antenna_collar = body.get_visual("antenna_collar")
    clip_leaf = belt_clip.get_visual("clip_leaf")
    outer_base = antenna_stage1.get_visual("outer_base")

    stage1_slide = object_model.get_articulation("body_to_antenna_stage1")
    stage2_slide = object_model.get_articulation("antenna_stage1_to_stage2")
    clip_hinge = object_model.get_articulation("body_to_belt_clip")

    ctx.expect_gap(
        antenna_stage1,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=outer_base,
        negative_elem=antenna_collar,
        name="outer antenna stage seats on the top collar",
    )
    ctx.expect_gap(
        antenna_stage2,
        antenna_stage1,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="inner antenna stage begins at the top of the outer stage",
    )
    ctx.expect_within(
        antenna_stage2,
        antenna_stage1,
        axes="xy",
        margin=0.004,
        name="inner antenna stage stays centered within the outer stage footprint",
    )
    ctx.expect_gap(
        body,
        belt_clip,
        axis="y",
        min_gap=0.0005,
        max_gap=0.004,
        positive_elem=lower_shell,
        negative_elem=clip_leaf,
        name="belt clip rests just behind the rear housing",
    )
    ctx.expect_overlap(
        belt_clip,
        body,
        axes="xz",
        min_overlap=0.020,
        elem_a=clip_leaf,
        elem_b=lower_shell,
        name="belt clip stays aligned to the rear of the handset",
    )

    stage1_rest = ctx.part_world_position(antenna_stage1)
    stage2_rest = ctx.part_world_position(antenna_stage2)
    clip_rest_aabb = ctx.part_world_aabb(belt_clip)

    stage1_upper = stage1_slide.motion_limits.upper if stage1_slide.motion_limits else None
    stage2_upper = stage2_slide.motion_limits.upper if stage2_slide.motion_limits else None
    clip_upper = clip_hinge.motion_limits.upper if clip_hinge.motion_limits else None

    with ctx.pose(
        {
            stage1_slide: stage1_upper,
            stage2_slide: stage2_upper,
            clip_hinge: clip_upper,
        }
    ):
        ctx.expect_within(
            antenna_stage2,
            antenna_stage1,
            axes="xy",
            margin=0.004,
            name="inner antenna stage stays centered when fully extended",
        )
        stage1_extended = ctx.part_world_position(antenna_stage1)
        stage2_extended = ctx.part_world_position(antenna_stage2)
        clip_open_aabb = ctx.part_world_aabb(belt_clip)

    ctx.check(
        "outer antenna stage extends upward",
        stage1_rest is not None
        and stage1_extended is not None
        and stage1_upper is not None
        and stage1_extended[2] > stage1_rest[2] + 0.080,
        details=f"rest={stage1_rest}, extended={stage1_extended}, travel={stage1_upper}",
    )
    ctx.check(
        "inner antenna stage extends upward beyond the outer stage travel",
        stage2_rest is not None
        and stage2_extended is not None
        and stage2_upper is not None
        and stage2_extended[2] > stage2_rest[2] + 0.180,
        details=f"rest={stage2_rest}, extended={stage2_extended}, travel={stage2_upper}",
    )
    ctx.check(
        "belt clip swings outward from the rear face",
        clip_rest_aabb is not None
        and clip_open_aabb is not None
        and clip_open_aabb[0][1] < clip_rest_aabb[0][1] - 0.015,
        details=f"rest_aabb={clip_rest_aabb}, open_aabb={clip_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
