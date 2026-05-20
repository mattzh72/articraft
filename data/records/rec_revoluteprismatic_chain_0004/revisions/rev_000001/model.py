from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_LEN = 0.090
BASE_W = 0.050
BASE_TH = 0.012
SUPPORT_LEN = 0.022
SUPPORT_TH = 0.008
SUPPORT_H = 0.055
PIVOT_Z = 0.040
PIN_RADIUS = 0.0052
SUPPORT_Y = -0.006
ARM_Y = 0.010

ARM_HUB_OUTER_R = 0.0105
ARM_HUB_LENGTH = 0.016
ARM_HOLE_R = 0.00535
ARM_BEAM_LEN = 0.076
ARM_BEAM_W = 0.012
ARM_BEAM_H = 0.008
ARM_BEAM_Z = -0.012
GUIDE_START_X = 0.084
GUIDE_LEN = 0.090
GUIDE_SIZE = 0.010
GUIDE_Z = 0.0
SLIDER_JOINT_X = 0.094

SLIDER_LEN = 0.030
SLIDER_W = 0.019
SLIDER_H = 0.016
SLIDER_TOP_PAD_L = 0.020
SLIDER_TOP_PAD_H = 0.006


def _base_bracket_shape() -> cq.Workplane:
    bracket = cq.Workplane("XY").box(
        BASE_LEN,
        BASE_W,
        BASE_TH,
        centered=(True, True, False),
    )
    support = (
        cq.Workplane("XY")
        .box(SUPPORT_LEN, SUPPORT_TH, SUPPORT_H, centered=(True, True, False))
        .translate((0.0, SUPPORT_Y, BASE_TH))
    )
    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.022, BASE_TH),
                (-0.022, BASE_TH + 0.004),
                (0.010, BASE_TH + 0.028),
                (0.010, BASE_TH),
            ]
        )
        .close()
        .extrude(SUPPORT_TH)
        .translate((0.0, SUPPORT_Y - SUPPORT_TH / 2.0, 0.0))
    )
    bracket = bracket.union(support).union(gusset)

    pin_length = (ARM_Y + ARM_HUB_LENGTH / 2.0) - (SUPPORT_Y - SUPPORT_TH / 2.0)
    pin_center_y = (ARM_Y + ARM_HUB_LENGTH / 2.0 + SUPPORT_Y - SUPPORT_TH / 2.0) / 2.0

    pin = (
        cq.Workplane("XZ")
        .center(0.0, PIVOT_Z)
        .circle(PIN_RADIUS)
        .extrude(pin_length, both=True)
        .translate((0.0, pin_center_y, 0.0))
    )
    collar = (
        cq.Workplane("XZ")
        .center(0.0, PIVOT_Z)
        .circle(0.0075)
        .extrude(0.0005, both=True)
        .translate((0.0, 0.0015, 0.0))
    )
    return bracket.union(pin).union(collar)


def _arm_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XZ")
        .circle(ARM_HUB_OUTER_R)
        .extrude(ARM_HUB_LENGTH / 2.0, both=True)
        .cut(
            cq.Workplane("XZ")
            .circle(ARM_HOLE_R)
            .extrude(ARM_HUB_LENGTH * 0.75, both=True)
        )
        .translate((0.0, ARM_Y, 0.0))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.050, ARM_BEAM_W, 0.006)
        .translate((0.034, ARM_Y, -0.006))
    )
    beam = (
        cq.Workplane("XY")
        .box(ARM_BEAM_LEN, ARM_BEAM_W, ARM_BEAM_H)
        .translate((GUIDE_START_X - 0.010 + ARM_BEAM_LEN / 2.0, ARM_Y, ARM_BEAM_Z))
    )
    riser = (
        cq.Workplane("XY")
        .box(0.016, ARM_BEAM_W * 0.90, 0.016)
        .translate((GUIDE_START_X - 0.008, ARM_Y, -0.004))
    )
    rail = (
        cq.Workplane("XY")
        .box(GUIDE_LEN, GUIDE_SIZE, GUIDE_SIZE)
        .translate((GUIDE_START_X + GUIDE_LEN / 2.0, ARM_Y, GUIDE_Z))
    )
    nose = (
        cq.Workplane("YZ")
        .circle(0.006)
        .extrude(0.008)
        .translate((GUIDE_START_X + GUIDE_LEN - 0.004, ARM_Y, GUIDE_Z))
    )
    return hub.union(neck).union(beam).union(riser).union(rail).union(nose)


def _slider_shape() -> cq.Workplane:
    inner_w = GUIDE_SIZE + 0.002
    inner_h = GUIDE_SIZE + 0.002
    wall_t = 0.003
    top_t = 0.004
    bridge = (
        cq.Workplane("XY")
        .box(SLIDER_LEN, inner_w + 2.0 * wall_t, top_t)
        .translate((0.0, ARM_Y, GUIDE_SIZE / 2.0 + top_t / 2.0))
    )
    side_wall_h = inner_h
    side_wall_z = 0.0
    left_wall = (
        cq.Workplane("XY")
        .box(SLIDER_LEN, wall_t, side_wall_h)
        .translate((0.0, ARM_Y - (inner_w / 2.0 + wall_t / 2.0), side_wall_z))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(SLIDER_LEN, wall_t, side_wall_h)
        .translate((0.0, ARM_Y + inner_w / 2.0 + wall_t / 2.0, side_wall_z))
    )
    top_pad = (
        cq.Workplane("XY")
        .box(SLIDER_TOP_PAD_L, SLIDER_W * 0.84, SLIDER_TOP_PAD_H)
        .translate((0.0, ARM_Y, GUIDE_SIZE / 2.0 + top_t + SLIDER_TOP_PAD_H / 2.0))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.010, SLIDER_W * 0.60, 0.010)
        .translate((SLIDER_LEN / 2.0 + 0.005, ARM_Y, inner_h / 2.0 + top_t + 0.005))
    )
    return bridge.union(left_wall).union(right_wall).union(top_pad).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolute_prismatic_chain", assets=ASSETS)

    steel = model.material("steel", rgba=(0.48, 0.50, 0.54, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    slider_finish = model.material("slider_finish", rgba=(0.86, 0.48, 0.16, 1.0))

    base = model.part("base_bracket")
    base.visual(
        mesh_from_cadquery(_base_bracket_shape(), "base_bracket.obj", assets=ASSETS),
        name="base_shell",
        material=dark_steel,
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_W, SUPPORT_H + BASE_TH)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, (SUPPORT_H + BASE_TH) / 2.0)),
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_shape(), "arm.obj", assets=ASSETS),
        name="arm_shell",
        material=steel,
    )
    arm.inertial = Inertial.from_geometry(
        Box((GUIDE_START_X + GUIDE_LEN, ARM_BEAM_W, 0.030)),
        mass=0.85,
        origin=Origin(xyz=((GUIDE_START_X + GUIDE_LEN) / 2.0, 0.0, -0.002)),
    )

    slider = model.part("slider")
    slider.visual(
        mesh_from_cadquery(_slider_shape(), "slider.obj", assets=ASSETS),
        name="slider_shell",
        material=slider_finish,
    )
    slider.inertial = Inertial.from_geometry(
        Box((SLIDER_LEN + 0.010, SLIDER_W, 0.020)),
        mass=0.35,
        origin=Origin(xyz=(0.005, 0.0, 0.008)),
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-math.radians(120.0),
            upper=math.radians(120.0),
        ),
    )
    model.articulation(
        "arm_to_slider",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=slider,
        origin=Origin(xyz=(SLIDER_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=0.080,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_bracket")
    arm = object_model.get_part("arm")
    slider = object_model.get_part("slider")
    swing = object_model.get_articulation("base_to_arm")
    slide = object_model.get_articulation("arm_to_slider")
    base_shell = base.get_visual("base_shell")
    arm_shell = arm.get_visual("arm_shell")
    slider_shell = slider.get_visual("slider_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        base,
        arm,
        elem_a=base_shell,
        elem_b=arm_shell,
        reason="The base pin intentionally passes through the arm hinge bore as a nested revolute joint.",
    )
    ctx.allow_overlap(
        arm,
        slider,
        elem_a=arm_shell,
        elem_b=slider_shell,
        reason="The slider is a guided sleeve that intentionally envelopes the distal rail of the arm.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("base part present", base is not None, "base_bracket missing")
    ctx.check("arm part present", arm is not None, "arm missing")
    ctx.check("slider part present", slider is not None, "slider missing")
    ctx.check("revolute joint present", swing is not None, "base_to_arm missing")
    ctx.check("prismatic joint present", slide is not None, "arm_to_slider missing")

    ctx.expect_overlap(
        arm,
        base,
        axes="yz",
        min_overlap=0.010,
        elem_a=arm_shell,
        elem_b=base_shell,
        name="arm pivot region sits within bracket envelope",
    )
    ctx.expect_overlap(
        slider,
        arm,
        axes="yz",
        min_overlap=0.009,
        elem_a=slider_shell,
        elem_b=arm_shell,
        name="slider is captured around guide cross-section",
    )
    ctx.expect_origin_distance(
        arm,
        base,
        axes="xy",
        max_dist=1e-6,
        name="arm pivot origin is centered on base pin axis",
    )
    ctx.expect_origin_gap(
        arm,
        base,
        axis="z",
        min_gap=PIVOT_Z - 1e-6,
        max_gap=PIVOT_Z + 1e-6,
        name="arm joint origin sits at base pin height",
    )
    ctx.expect_origin_distance(
        slider,
        arm,
        axes="yz",
        max_dist=1e-6,
        name="slider joint is aligned to arm guide axis",
    )

    with ctx.pose({swing: 0.0, slide: 0.0}):
        arm_pos = ctx.part_world_position(arm)
        slider_pos_retracted = ctx.part_world_position(slider)
        ctx.check(
            "retracted slider starts on arm axis",
            arm_pos is not None and slider_pos_retracted is not None and abs(slider_pos_retracted[1] - arm_pos[1]) < 1e-4,
            f"arm_pos={arm_pos}, slider_pos={slider_pos_retracted}",
        )
        ctx.check(
            "retracted slider sits at nominal rail start",
            slider_pos_retracted is not None and abs(slider_pos_retracted[0] - SLIDER_JOINT_X) < 1e-4,
            f"slider x was {None if slider_pos_retracted is None else slider_pos_retracted[0]}",
        )

    with ctx.pose({swing: 0.0, slide: 0.080}):
        slider_pos_extended = ctx.part_world_position(slider)
        ctx.check(
            "prismatic stage extends 80 mm along arm axis at zero swing",
            slider_pos_extended is not None and abs(slider_pos_extended[0] - (SLIDER_JOINT_X + 0.080)) < 1e-4,
            f"slider x was {None if slider_pos_extended is None else slider_pos_extended[0]}",
        )

    with ctx.pose({swing: math.pi / 2.0, slide: 0.080}):
        slider_pos_vertical = ctx.part_world_position(slider)
        ctx.check(
            "revolute joint rotates arm into vertical pose",
            slider_pos_vertical is not None and abs(slider_pos_vertical[2] - (PIVOT_Z - (SLIDER_JOINT_X + 0.080))) < 2e-3,
            f"slider vertical pose position was {slider_pos_vertical}",
        )
        ctx.check(
            "vertical pose keeps slider centered in bracket plane",
            slider_pos_vertical is not None and abs(slider_pos_vertical[1]) < 1e-4,
            f"slider y was {None if slider_pos_vertical is None else slider_pos_vertical[1]}",
        )

    with ctx.pose({swing: -math.radians(110.0), slide: 0.040}):
        slider_pos_combined = ctx.part_world_position(slider)
        ctx.check(
            "combined pose keeps slider on rotated arm axis",
            slider_pos_combined is not None
            and abs(
                math.hypot(slider_pos_combined[0], slider_pos_combined[2] - PIVOT_Z)
                - (SLIDER_JOINT_X + 0.040)
            )
            < 2e-3,
            f"combined pose slider position was {slider_pos_combined}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
