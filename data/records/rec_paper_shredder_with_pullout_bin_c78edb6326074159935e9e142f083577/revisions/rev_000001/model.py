from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 0.355
BODY_DEPTH = 0.235
LOWER_HEIGHT = 0.305
HEAD_HEIGHT = 0.110
TOTAL_HEIGHT = LOWER_HEIGHT + HEAD_HEIGHT

BIN_WIDTH = 0.314
BIN_DEPTH = 0.192
BIN_HEIGHT = 0.258
BIN_TRAVEL = 0.105


def _build_body_shell() -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, LOWER_HEIGHT)
        .translate((0.0, 0.0, LOWER_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.016)
    )
    head = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, 0.214, HEAD_HEIGHT)
        .translate((0.0, -0.004, LOWER_HEIGHT + HEAD_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.012)
    )
    shell = lower.union(head)

    bin_cavity = (
        cq.Workplane("XY")
        .box(0.326, 0.206, 0.272)
        .translate((0.0, 0.0145, 0.166))
    )
    drum_chamber = (
        cq.Workplane("XY")
        .box(0.276, 0.078, 0.092)
        .translate((0.0, 0.004, 0.340))
    )
    throat = (
        cq.Workplane("XY")
        .box(0.242, 0.030, 0.060)
        .translate((0.0, 0.014, 0.370))
    )
    paper_slot = (
        cq.Workplane("XY")
        .box(0.236, 0.014, 0.020)
        .translate((0.0, 0.018, TOTAL_HEIGHT - 0.004))
    )
    return shell.cut(bin_cavity).cut(drum_chamber).cut(throat).cut(paper_slot)


def _build_control_band() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.272, 0.066, 0.010)
        .edges("|Z")
        .fillet(0.007)
        .faces(">Z")
        .edges()
        .fillet(0.003)
    )


def _build_bin_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BIN_WIDTH, BIN_DEPTH, BIN_HEIGHT)
        .translate((0.0, -BIN_DEPTH * 0.5, BIN_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.014)
        .faces(">Z")
        .shell(-0.004)
    )
    window_cut = (
        cq.Workplane("XY")
        .box(0.184, 0.020, 0.106)
        .translate((0.0, -0.003, 0.118))
    )
    pull_lip = (
        cq.Workplane("XY")
        .box(0.182, 0.014, 0.016)
        .translate((0.0, 0.002, 0.229))
        .edges("|X")
        .fillet(0.0035)
    )
    grip_cut = (
        cq.Workplane("XY")
        .box(0.126, 0.020, 0.020)
        .translate((0.0, 0.006, 0.224))
    )
    return shell.cut(window_cut).union(pull_lip).cut(grip_cut)


def _build_button_cap() -> cq.Workplane:
    return cq.Workplane("XY").box(0.032, 0.018, 0.004).edges("|Z").fillet(0.002)


def _build_selector_dial() -> cq.Workplane:
    skirt = cq.Workplane("XY").circle(0.022).extrude(0.004)
    cap = cq.Workplane("XY").circle(0.018).extrude(0.018).translate((0.0, 0.0, 0.004))
    pointer = (
        cq.Workplane("XY")
        .box(0.003, 0.010, 0.002)
        .translate((0.0, 0.012, 0.019))
    )
    return skirt.union(cap).union(pointer)


def _build_cutter_drum() -> cq.Workplane:
    drum = cq.Workplane("YZ").circle(0.0125).extrude(0.116, both=True)
    for x_pos in (-0.096, -0.072, -0.048, -0.024, 0.0, 0.024, 0.048, 0.072, 0.096):
        collar = (
            cq.Workplane("YZ")
            .circle(0.016)
            .extrude(0.004, both=True)
            .translate((x_pos, 0.0, 0.0))
        )
        drum = drum.union(collar)
    drum = drum.union(
        cq.Workplane("YZ").circle(0.005).extrude(0.011, both=True).translate((0.127, 0.0, 0.0))
    )
    drum = drum.union(
        cq.Workplane("YZ").circle(0.005).extrude(0.011, both=True).translate((-0.127, 0.0, 0.0))
    )
    return drum


def _build_flap() -> cq.Workplane:
    hinge_leaf = (
        cq.Workplane("XY")
        .box(0.228, 0.006, 0.0022)
        .translate((0.0, 0.003, 0.0011))
    )
    flap_panel = cq.Workplane("XY").box(0.230, 0.028, 0.0022).translate((0.0, 0.017, 0.0011))
    return hinge_leaf.union(flap_panel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_cut_shredder")

    body_plastic = model.material("body_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    band_plastic = model.material("band_plastic", rgba=(0.21, 0.22, 0.24, 1.0))
    bin_plastic = model.material("bin_plastic", rgba=(0.20, 0.21, 0.22, 1.0))
    window_tint = model.material("window_tint", rgba=(0.20, 0.28, 0.30, 0.35))
    control_grey = model.material("control_grey", rgba=(0.70, 0.72, 0.74, 1.0))
    control_dark = model.material("control_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    cutter_steel = model.material("cutter_steel", rgba=(0.68, 0.70, 0.73, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "shredder_body"),
        material=body_plastic,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_build_control_band(), "control_band"),
        origin=Origin(xyz=(0.0, 0.012, TOTAL_HEIGHT - 0.0045)),
        material=band_plastic,
        name="control_band",
    )
    body.visual(
        Box((0.290, 0.070, 0.004)),
        origin=Origin(xyz=(0.0, 0.006, 0.002)),
        material=band_plastic,
        name="base_shadow",
    )

    bin_part = model.part("bin")
    bin_part.visual(
        mesh_from_cadquery(_build_bin_shell(), "pullout_bin"),
        material=bin_plastic,
        name="bin_shell",
    )
    bin_part.visual(
        Box((0.186, 0.005, 0.108)),
        origin=Origin(xyz=(0.0, -0.0065, 0.118)),
        material=window_tint,
        name="window",
    )
    bin_part.visual(
        Box((0.006, 0.142, 0.020)),
        origin=Origin(xyz=(-0.160, -0.092, 0.032)),
        material=bin_plastic,
        name="runner_0",
    )
    bin_part.visual(
        Box((0.006, 0.142, 0.020)),
        origin=Origin(xyz=(0.160, -0.092, 0.032)),
        material=bin_plastic,
        name="runner_1",
    )

    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.0, 0.112, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.28,
            lower=0.0,
            upper=BIN_TRAVEL,
        ),
    )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_build_flap(), "slot_flap"),
        material=control_grey,
        name="flap_panel",
    )
    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0105, TOTAL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=1.12,
        ),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_build_selector_dial(), "selector_dial"),
        material=control_dark,
        name="dial_cap",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.092, 0.012, TOTAL_HEIGHT + 0.0005)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    for index, x_pos in enumerate((-0.092, -0.054, -0.016, 0.022)):
        button = model.part(f"button_{index}")
        button.visual(
            mesh_from_cadquery(_build_button_cap(), f"touch_button_{index}"),
            material=control_grey,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.012, TOTAL_HEIGHT + 0.0005)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0025,
            ),
        )

    for drum_name, y_pos in (("drum_0", 0.0175), ("drum_1", -0.0175)):
        drum = model.part(drum_name)
        drum.visual(
            mesh_from_cadquery(_build_cutter_drum(), drum_name),
            material=cutter_steel,
            name="cutter_drum",
        )
        model.articulation(
            f"body_to_{drum_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=drum,
            origin=Origin(xyz=(0.0, y_pos, 0.346)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    flap = object_model.get_part("flap")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    bin_slide = object_model.get_articulation("body_to_bin")
    flap_hinge = object_model.get_articulation("body_to_flap")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    dial_joint = object_model.get_articulation("body_to_dial")
    drum_0_joint = object_model.get_articulation("body_to_drum_0")
    drum_1_joint = object_model.get_articulation("body_to_drum_1")

    ctx.allow_overlap(
        bin_part,
        body,
        elem_a="runner_0",
        elem_b="body_shell",
        reason="The concealed side runner stands in for the bin's guided slide pad inside the shell.",
    )
    ctx.allow_overlap(
        bin_part,
        body,
        elem_a="runner_1",
        elem_b="body_shell",
        reason="The concealed side runner stands in for the bin's guided slide pad inside the shell.",
    )
    ctx.allow_overlap(
        body,
        "drum_0",
        elem_a="body_shell",
        elem_b="cutter_drum",
        reason="The cutter drum is intentionally nested inside the simplified motor-head shell cavity.",
    )
    ctx.allow_overlap(
        body,
        "drum_1",
        elem_a="body_shell",
        elem_b="cutter_drum",
        reason="The cutter drum is intentionally nested inside the simplified motor-head shell cavity.",
    )

    ctx.expect_within(
        bin_part,
        body,
        axes="x",
        inner_elem="bin_shell",
        outer_elem="body_shell",
        margin=0.022,
        name="bin stays centered laterally in the body",
    )
    ctx.expect_overlap(
        bin_part,
        body,
        axes="y",
        elem_a="bin_shell",
        elem_b="body_shell",
        min_overlap=0.18,
        name="closed bin remains deeply inserted in the body",
    )

    bin_rest_pos = ctx.part_world_position(bin_part)
    flap_rest_aabb = ctx.part_world_aabb(flap)
    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)

    bin_extended_pos = None
    flap_open_aabb = None
    button_0_pressed = None
    button_1_still = None
    if (
        bin_slide.motion_limits is not None
        and bin_slide.motion_limits.upper is not None
        and flap_hinge.motion_limits is not None
        and flap_hinge.motion_limits.upper is not None
        and button_0_joint.motion_limits is not None
        and button_0_joint.motion_limits.upper is not None
    ):
        with ctx.pose({bin_slide: bin_slide.motion_limits.upper}):
            ctx.expect_overlap(
                bin_part,
                body,
                axes="y",
                elem_a="bin_shell",
                elem_b="body_shell",
                min_overlap=0.075,
                name="extended bin keeps retained insertion",
            )
            bin_extended_pos = ctx.part_world_position(bin_part)

        with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
            flap_open_aabb = ctx.part_world_aabb(flap)

        with ctx.pose({button_0_joint: button_0_joint.motion_limits.upper}):
            button_0_pressed = ctx.part_world_position(button_0)
            button_1_still = ctx.part_world_position(button_1)

    ctx.check(
        "bin slides forward",
        bin_rest_pos is not None
        and bin_extended_pos is not None
        and bin_extended_pos[1] > bin_rest_pos[1] + 0.09,
        details=f"rest={bin_rest_pos}, extended={bin_extended_pos}",
    )
    ctx.check(
        "slot flap lifts upward",
        flap_rest_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[1][2] > flap_rest_aabb[1][2] + 0.01,
        details=f"rest={flap_rest_aabb}, open={flap_open_aabb}",
    )
    ctx.check(
        "button press is independent",
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_pressed is not None
        and button_1_still is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.0015
        and abs(button_1_still[2] - button_1_rest[2]) < 1e-5,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_during_press={button_1_still}"
        ),
    )
    ctx.check(
        "selector dial uses vertical continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(dial_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}",
    )
    ctx.check(
        "cutter drums use matched horizontal axes",
        drum_0_joint.articulation_type == ArticulationType.CONTINUOUS
        and drum_1_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(drum_0_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(drum_1_joint.axis) == (1.0, 0.0, 0.0),
        details=(
            f"drum_0=(type={drum_0_joint.articulation_type}, axis={drum_0_joint.axis}), "
            f"drum_1=(type={drum_1_joint.articulation_type}, axis={drum_1_joint.axis})"
        ),
    )

    return ctx.report()


object_model = build_object_model()
