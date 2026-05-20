from __future__ import annotations

import cadquery as cq
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
    mesh_from_cadquery,
)


BODY_W = 0.066
BODY_D = 0.036
BODY_H = 0.170
TOP_CAP_H = 0.010
ANT_X = -0.021
ANT_Y = 0.000
ANT_BORE_R = 0.0064


def _rounded_body_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )
    # A real radio top cap has a bored receiver well for the first antenna tube.
    return (
        shell.faces(">Z")
        .workplane()
        .center(ANT_X, ANT_Y)
        .circle(ANT_BORE_R)
        .cutBlind(-0.158)
    )


def _top_cap_plate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BODY_W * 0.96, BODY_D * 1.03, TOP_CAP_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .faces(">Z")
        .workplane()
        .center(ANT_X, ANT_Y)
        .circle(ANT_BORE_R)
        .cutBlind(-TOP_CAP_H)
    )


def _tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def _belt_clip_plate() -> cq.Workplane:
    # A tapered spring clip plate, authored in the clip frame: hinge line at z=0,
    # plate extending down the rear of the handset, and thickness in +Y.
    top_w = 0.036
    bottom_w = 0.025
    length = 0.126
    thickness = 0.003
    profile = [
        (-top_w / 2, -0.003),
        (top_w / 2, -0.003),
        (bottom_w / 2, -length),
        (-bottom_w / 2, -length),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(thickness)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_way_radio_handset")

    body_mat = Material("mat_charcoal_body", color=(0.025, 0.030, 0.032, 1.0))
    rubber_mat = Material("mat_black_rubber", color=(0.005, 0.006, 0.006, 1.0))
    dark_mat = Material("mat_dark_plastic", color=(0.010, 0.012, 0.014, 1.0))
    screen_mat = Material("mat_blue_glass", color=(0.020, 0.085, 0.130, 1.0))
    metal_mat = Material("mat_brushed_steel", color=(0.65, 0.68, 0.66, 1.0))
    clip_mat = Material("mat_rear_clip", color=(0.020, 0.022, 0.024, 1.0))
    label_mat = Material("mat_pale_markings", color=(0.72, 0.76, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_body_shell(), "radio_rounded_body", tolerance=0.0007),
        material=body_mat,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_top_cap_plate(), "radio_top_cap", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.0, BODY_H)),
        material=dark_mat,
        name="top_cap",
    )
    body.visual(
        mesh_from_cadquery(_tube(0.010, ANT_BORE_R + 0.0009, 0.020), "antenna_socket"),
        origin=Origin(xyz=(ANT_X, ANT_Y, BODY_H)),
        material=rubber_mat,
        name="antenna_socket",
    )

    # Front face details are shallow, slightly seated trims on the main housing.
    front_y = -BODY_D / 2 - 0.00035
    body.visual(
        Box((0.043, 0.0014, 0.026)),
        origin=Origin(xyz=(0.0, front_y, 0.132)),
        material=screen_mat,
        name="display_window",
    )
    body.visual(
        Box((0.047, 0.0012, 0.032)),
        origin=Origin(xyz=(0.0, front_y, 0.092)),
        material=dark_mat,
        name="speaker_panel",
    )
    for i, x in enumerate((-0.018, -0.009, 0.0, 0.009, 0.018)):
        body.visual(
            Box((0.0032, 0.0017, 0.027)),
            origin=Origin(xyz=(x, front_y - 0.00025, 0.092)),
            material=label_mat,
            name=f"speaker_slot_{i}",
        )
    body.visual(
        Box((0.041, 0.0018, 0.030)),
        origin=Origin(xyz=(0.0, front_y - 0.00025, 0.047)),
        material=rubber_mat,
        name="keypad_membrane",
    )
    for row, z in enumerate((0.055, 0.046, 0.037)):
        for col, x in enumerate((-0.013, 0.0, 0.013)):
            body.visual(
                Cylinder(radius=0.0033, length=0.0018),
                origin=Origin(xyz=(x, front_y - 0.0011, z), rpy=(math.pi / 2, 0.0, 0.0)),
                material=dark_mat,
                name=f"molded_key_{row}_{col}",
            )

    # Rear hinge lugs for the belt clip.
    rear_y = BODY_D / 2
    hinge_y = rear_y + 0.0030
    hinge_z = 0.146
    for side, x in enumerate((-0.022, 0.022)):
        body.visual(
            Box((0.010, 0.006, 0.014)),
            origin=Origin(xyz=(x, rear_y + 0.0022, hinge_z - 0.001)),
            material=clip_mat,
            name=f"clip_lug_block_{side}",
        )
        body.visual(
            Cylinder(radius=0.0042, length=0.010),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2, 0.0)),
            material=clip_mat,
            name=f"clip_hinge_knuckle_{side}",
        )

    # Top rotary volume knob.
    volume_knob = model.part("volume_knob")
    volume_knob.visual(
        Cylinder(radius=0.009, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=rubber_mat,
        name="knob_grip",
    )
    for angle_index, angle in enumerate((0, math.pi / 3, 2 * math.pi / 3)):
        x = 0.006 * math.cos(angle)
        y = 0.006 * math.sin(angle)
        volume_knob.visual(
            Box((0.0020, 0.0030, 0.010)),
            origin=Origin(xyz=(x, y, 0.0067), rpy=(0.0, 0.0, angle)),
            material=body_mat,
            name=f"knob_rib_{angle_index}",
        )
    volume_knob.visual(
        Box((0.0020, 0.010, 0.0012)),
        origin=Origin(xyz=(0.0, -0.0015, 0.0136)),
        material=label_mat,
        name="knob_indicator",
    )
    model.articulation(
        "body_to_volume_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=volume_knob,
        origin=Origin(xyz=(0.019, 0.000, BODY_H + TOP_CAP_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=4.0),
    )

    # Side push-to-talk key: the negative limit is the protruding unpressed pose,
    # and q=0 sits flush against the left side of the shell.
    side_button = model.part("side_button")
    side_button.visual(
        Box((0.006, 0.016, 0.036)),
        origin=Origin(xyz=(-0.003, 0.0, 0.0)),
        material=rubber_mat,
        name="ptt_button",
    )
    model.articulation(
        "body_to_side_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=side_button,
        origin=Origin(xyz=(-BODY_W / 2, 0.0, 0.096)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=-0.002, upper=0.0),
    )

    # First telescoping antenna stage: a hollow lower tube retained in the body
    # receiver well and the black top collar.
    antenna_stage_0 = model.part("antenna_stage_0")
    antenna_stage_0.visual(
        mesh_from_cadquery(_tube(0.0048, 0.0030, 0.230), "antenna_stage_0_tube", tolerance=0.0004),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=metal_mat,
        name="stage_0_tube",
    )
    antenna_stage_0.visual(
        Cylinder(radius=0.0074, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=metal_mat,
        name="stage_0_stop_collar",
    )
    model.articulation(
        "body_to_antenna_stage_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=antenna_stage_0,
        origin=Origin(xyz=(ANT_X, ANT_Y, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.25, lower=0.0, upper=0.130),
    )

    # Second whip stage slides inside the first hollow tube and carries a small
    # rounded end cap so it reads as a real telescoping whip rather than a wire.
    antenna_stage_1 = model.part("antenna_stage_1")
    antenna_stage_1.visual(
        Cylinder(radius=0.0022, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=metal_mat,
        name="inner_whip",
    )
    antenna_stage_1.visual(
        Cylinder(radius=0.0036, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal_mat,
        name="stage_1_stop_collar",
    )
    antenna_stage_1.visual(
        Cylinder(radius=0.0030, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=metal_mat,
        name="whip_tip",
    )
    model.articulation(
        "stage_0_to_antenna_stage_1",
        ArticulationType.PRISMATIC,
        parent=antenna_stage_0,
        child=antenna_stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=0.35, lower=0.0, upper=0.160),
    )

    # Rear belt clip with an exposed hinge barrel on the clip and matching lugs
    # on the body. Positive angle pivots the lower end away from the handset.
    belt_clip = model.part("belt_clip")
    belt_clip.visual(
        mesh_from_cadquery(_belt_clip_plate(), "belt_clip_plate", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.0002, 0.0)),
        material=clip_mat,
        name="clip_plate",
    )
    belt_clip.visual(
        Cylinder(radius=0.0037, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=clip_mat,
        name="clip_hinge_barrel",
    )
    belt_clip.visual(
        Box((0.020, 0.0022, 0.006)),
        origin=Origin(xyz=(0.0, -0.0006, -0.118)),
        material=rubber_mat,
        name="clip_contact_pad",
    )
    model.articulation(
        "body_to_belt_clip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=belt_clip,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.7, velocity=3.0, lower=0.0, upper=0.95),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    body = object_model.get_part("body")
    stage_0 = object_model.get_part("antenna_stage_0")
    stage_1 = object_model.get_part("antenna_stage_1")
    belt_clip = object_model.get_part("belt_clip")
    side_button = object_model.get_part("side_button")

    stage_0_slide = object_model.get_articulation("body_to_antenna_stage_0")
    stage_1_slide = object_model.get_articulation("stage_0_to_antenna_stage_1")
    clip_hinge = object_model.get_articulation("body_to_belt_clip")
    button_slide = object_model.get_articulation("body_to_side_button")

    ctx.expect_within(
        stage_0,
        body,
        axes="xy",
        inner_elem="stage_0_tube",
        outer_elem="antenna_socket",
        margin=0.001,
        name="lower antenna stage is centered in the top cap socket",
    )
    ctx.expect_overlap(
        stage_0,
        body,
        axes="z",
        elem_a="stage_0_tube",
        elem_b="antenna_socket",
        min_overlap=0.018,
        name="lower antenna stage remains inserted in top socket",
    )
    ctx.expect_within(
        stage_1,
        stage_0,
        axes="xy",
        inner_elem="inner_whip",
        outer_elem="stage_0_tube",
        margin=0.001,
        name="upper whip stage runs inside lower antenna tube",
    )

    stage_0_rest = ctx.part_world_position(stage_0)
    stage_1_rest = ctx.part_world_position(stage_1)
    clip_rest_aabb = ctx.part_world_aabb(belt_clip)
    button_rest_aabb = ctx.part_world_aabb(side_button)

    with ctx.pose({stage_0_slide: 0.130, stage_1_slide: 0.160}):
        stage_0_extended = ctx.part_world_position(stage_0)
        stage_1_extended = ctx.part_world_position(stage_1)
        ctx.expect_overlap(
            stage_0,
            body,
            axes="z",
            elem_a="stage_0_tube",
            elem_b="antenna_socket",
            min_overlap=0.018,
            name="extended lower antenna stage still has retained insertion",
        )
        ctx.expect_overlap(
            stage_1,
            stage_0,
            axes="z",
            elem_a="inner_whip",
            elem_b="stage_0_tube",
            min_overlap=0.018,
            name="extended upper whip stage still engages lower tube",
        )

    ctx.check(
        "telescoping antenna extends upward in two stages",
        stage_0_rest is not None
        and stage_0_extended is not None
        and stage_1_rest is not None
        and stage_1_extended is not None
        and stage_0_extended[2] > stage_0_rest[2] + 0.12
        and stage_1_extended[2] > stage_1_rest[2] + 0.28,
        details=f"stage0 rest/extended={stage_0_rest}/{stage_0_extended}, "
        f"stage1 rest/extended={stage_1_rest}/{stage_1_extended}",
    )

    with ctx.pose({clip_hinge: 0.95}):
        clip_open_aabb = ctx.part_world_aabb(belt_clip)
    ctx.check(
        "belt clip pivots away from rear face",
        clip_rest_aabb is not None
        and clip_open_aabb is not None
        and clip_open_aabb[1][1] > clip_rest_aabb[1][1] + 0.045,
        details=f"closed_aabb={clip_rest_aabb}, open_aabb={clip_open_aabb}",
    )

    with ctx.pose({button_slide: -0.002}):
        button_out_aabb = ctx.part_world_aabb(side_button)
    ctx.check(
        "side push-to-talk button has inward prismatic travel",
        button_rest_aabb is not None
        and button_out_aabb is not None
        and button_out_aabb[0][0] < button_rest_aabb[0][0] - 0.0015,
        details=f"flush_aabb={button_rest_aabb}, protruding_aabb={button_out_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
