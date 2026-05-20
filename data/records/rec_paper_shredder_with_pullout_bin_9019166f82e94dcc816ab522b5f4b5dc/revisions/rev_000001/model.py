from __future__ import annotations

import math

import cadquery as cq

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


BODY_W = 0.30
BODY_D = 0.22
BODY_H = 0.42
WALL_T = 0.006
FLOOR_T = 0.014
ROOF_T = 0.030

FRONT_OPEN_W = 0.274
FRONT_OPEN_H = 0.268
FRONT_OPEN_D = 0.160
FRONT_OPEN_Z = 0.014

PAPER_SLOT_W = 0.220
PAPER_SLOT_D = 0.008
PAPER_SLOT_Y = 0.018

BIN_W = 0.262
BIN_D = 0.166
BIN_H = 0.220
BIN_WALL = 0.0035
BIN_CLOSED_Y = 0.020
BIN_CLOSED_Z = 0.126

GUIDE_W = 0.085
GUIDE_D = 0.030
GUIDE_H = 0.006
GUIDE_SLOT_W = 0.060
GUIDE_SLOT_D = 0.008
GUIDE_X = -0.078
GUIDE_Y = -0.054

BUTTON_BEZEL_W = 0.040
BUTTON_BEZEL_D = 0.030
BUTTON_BEZEL_H = 0.004
BUTTON_HOLE_W = 0.018
BUTTON_HOLE_D = 0.014
BUTTON_X = 0.082
BUTTON_Y = -0.054

DRUM_LEN = 0.260
DRUM_SHAFT_R = 0.0055
DRUM_CUTTER_R = 0.014
DRUM_CUTTER_T = 0.012
DRUM_Z = 0.350
DRUM_Y_0 = 0.002
DRUM_Y_1 = 0.032


def cq_box(size: tuple[float, float, float], center: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def make_body_shell() -> cq.Workplane:
    outer = cq_box((BODY_W, BODY_D, BODY_H), center=(0.0, 0.0, BODY_H / 2.0))
    cavity_h = BODY_H - FLOOR_T - ROOF_T
    inner = cq_box(
        (BODY_W - 2.0 * WALL_T, BODY_D - 2.0 * WALL_T, cavity_h),
        center=(0.0, 0.0, FLOOR_T + cavity_h / 2.0),
    )
    front_open = cq_box(
        (FRONT_OPEN_W, FRONT_OPEN_D, FRONT_OPEN_H),
        center=(0.0, BODY_D / 2.0 - FRONT_OPEN_D / 2.0 + 0.002, FRONT_OPEN_Z + FRONT_OPEN_H / 2.0),
    )
    paper_slot = cq_box(
        (PAPER_SLOT_W, PAPER_SLOT_D, ROOF_T + 0.010),
        center=(0.0, PAPER_SLOT_Y, BODY_H - ROOF_T / 2.0),
    )
    slider_slot = cq_box(
        (GUIDE_SLOT_W, GUIDE_SLOT_D, ROOF_T + GUIDE_H + 0.006),
        center=(GUIDE_X, GUIDE_Y, BODY_H - ROOF_T / 2.0 + GUIDE_H / 2.0),
    )
    button_hole = cq_box(
        (BUTTON_HOLE_W, BUTTON_HOLE_D, ROOF_T + BUTTON_BEZEL_H + 0.006),
        center=(BUTTON_X, BUTTON_Y, BODY_H - ROOF_T / 2.0 + BUTTON_BEZEL_H / 2.0),
    )
    return outer.cut(inner).cut(front_open).cut(paper_slot).cut(slider_slot).cut(button_hole)


def make_rect_frame(outer_size: tuple[float, float, float], inner_size: tuple[float, float, float]) -> cq.Workplane:
    return cq_box(outer_size).cut(cq_box(inner_size))


def make_bin_shell() -> cq.Workplane:
    outer = cq_box((BIN_W, BIN_D, BIN_H))
    inner = cq_box((BIN_W - 2.0 * BIN_WALL, BIN_D - 2.0 * BIN_WALL, BIN_H - BIN_WALL), center=(0.0, 0.0, BIN_WALL / 2.0))
    return outer.cut(inner)


def add_drum_visuals(part, prefix: str, material) -> None:
    drum_rpy = (0.0, math.pi / 2.0, 0.0)
    part.visual(
        Cylinder(radius=DRUM_SHAFT_R, length=DRUM_LEN),
        origin=Origin(rpy=drum_rpy),
        material=material,
        name=f"{prefix}_shaft",
    )
    for index, x_pos in enumerate((-0.094, -0.072, -0.050, -0.028, -0.006, 0.016, 0.038, 0.060, 0.082)):
        part.visual(
            Cylinder(radius=DRUM_CUTTER_R, length=DRUM_CUTTER_T),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=drum_rpy),
            material=material,
            name=f"{prefix}_disk_{index}",
        )
    for index, x_pos in enumerate((-0.130, 0.130)):
        part.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=drum_rpy),
            material=material,
            name=f"{prefix}_collar_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_shredder")

    body_mat = model.material("body_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    trim_mat = model.material("trim_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    bin_mat = model.material("bin_clear", rgba=(0.72, 0.82, 0.92, 0.32))
    drum_mat = model.material("drum_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    slider_mat = model.material("slider_gray", rgba=(0.74, 0.75, 0.77, 1.0))
    button_mat = model.material("button_blue", rgba=(0.18, 0.38, 0.70, 1.0))
    flap_mat = model.material("flap_smoke", rgba=(0.35, 0.38, 0.42, 0.45))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shell(), "body_shell"),
        material=body_mat,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(
            make_rect_frame(
                (GUIDE_W, GUIDE_D, GUIDE_H),
                (GUIDE_SLOT_W, GUIDE_SLOT_D, GUIDE_H + 0.002),
            ),
            "slider_guide",
        ),
        origin=Origin(xyz=(GUIDE_X, GUIDE_Y, BODY_H + GUIDE_H / 2.0)),
        material=trim_mat,
        name="slider_guide",
    )
    body.visual(
        mesh_from_cadquery(
            make_rect_frame(
                (BUTTON_BEZEL_W, BUTTON_BEZEL_D, BUTTON_BEZEL_H),
                (BUTTON_HOLE_W, BUTTON_HOLE_D, BUTTON_BEZEL_H + 0.002),
            ),
            "button_bezel",
        ),
        origin=Origin(xyz=(BUTTON_X, BUTTON_Y, BODY_H + BUTTON_BEZEL_H / 2.0)),
        material=trim_mat,
        name="button_bezel",
    )
    for index, x_pos in enumerate((-0.1395, 0.1395)):
        body.visual(
            Box((0.009, 0.150, 0.006)),
            origin=Origin(xyz=(x_pos, -0.010, 0.111)),
            material=trim_mat,
            name=f"rail_{index}",
        )
    for drum_index, drum_y in enumerate((DRUM_Y_0, DRUM_Y_1)):
        for side_index, x_pos in enumerate((-0.1395, 0.1395)):
            body.visual(
                Box((0.009, 0.022, 0.020)),
                origin=Origin(xyz=(x_pos, drum_y, DRUM_Z)),
                material=trim_mat,
                name=f"drum_support_{drum_index}_{side_index}",
            )

    bin_part = model.part("bin")
    bin_part.visual(
        mesh_from_cadquery(make_bin_shell(), "pullout_bin"),
        material=bin_mat,
        name="shell",
    )
    bin_part.visual(
        Box((0.082, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, BIN_D / 2.0 + 0.006, 0.020)),
        material=trim_mat,
        name="front_handle",
    )
    for index, x_pos in enumerate((-0.1345, 0.1345)):
        bin_part.visual(
            Box((0.004, 0.136, 0.004)),
            origin=Origin(xyz=(-0.133 if index == 0 else 0.133, -0.004, -0.020)),
            material=trim_mat,
            name=f"runner_{index}",
        )

    mode_slider = model.part("mode_slider")
    mode_slider.visual(
        Box((0.022, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=slider_mat,
        name="thumb",
    )
    mode_slider.visual(
        Box((0.010, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=slider_mat,
        name="runner",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.018, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=button_mat,
        name="cap",
    )
    start_button.visual(
        Box((0.010, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=button_mat,
        name="stem",
    )

    safety_flap = model.part("safety_flap")
    safety_flap.visual(
        Cylinder(radius=0.0025, length=PAPER_SLOT_W + 0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flap_mat,
        name="barrel",
    )
    safety_flap.visual(
        Box((PAPER_SLOT_W + 0.018, 0.028, 0.0018)),
        origin=Origin(xyz=(0.0, 0.014, -0.0025)),
        material=flap_mat,
        name="plate",
    )

    cutter_drum_0 = model.part("cutter_drum_0")
    add_drum_visuals(cutter_drum_0, "front", drum_mat)

    cutter_drum_1 = model.part("cutter_drum_1")
    add_drum_visuals(cutter_drum_1, "rear", drum_mat)

    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.0, BIN_CLOSED_Y, BIN_CLOSED_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.120, effort=45.0, velocity=0.25),
    )
    model.articulation(
        "body_to_mode_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_slider,
        origin=Origin(xyz=(GUIDE_X, GUIDE_Y, BODY_H + GUIDE_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.022, upper=0.022, effort=5.0, velocity=0.10),
    )
    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(BUTTON_X, BUTTON_Y, BODY_H + BUTTON_BEZEL_H)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.004, effort=8.0, velocity=0.08),
    )
    model.articulation(
        "body_to_safety_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=safety_flap,
        origin=Origin(xyz=(0.0, PAPER_SLOT_Y - PAPER_SLOT_D / 2.0 - 0.002, BODY_H + 0.0034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=1.0, velocity=2.0),
    )
    model.articulation(
        "body_to_cutter_drum_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cutter_drum_0,
        origin=Origin(xyz=(0.0, DRUM_Y_0, DRUM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=10.0),
    )
    model.articulation(
        "body_to_cutter_drum_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cutter_drum_1,
        origin=Origin(xyz=(0.0, DRUM_Y_1, DRUM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    mode_slider = object_model.get_part("mode_slider")
    start_button = object_model.get_part("start_button")
    safety_flap = object_model.get_part("safety_flap")
    cutter_drum_0 = object_model.get_part("cutter_drum_0")
    cutter_drum_1 = object_model.get_part("cutter_drum_1")

    bin_joint = object_model.get_articulation("body_to_bin")
    slider_joint = object_model.get_articulation("body_to_mode_slider")
    button_joint = object_model.get_articulation("body_to_start_button")
    flap_joint = object_model.get_articulation("body_to_safety_flap")

    ctx.expect_overlap(
        bin_part,
        body,
        axes="xz",
        min_overlap=0.20,
        name="bin remains registered within the shredder opening",
    )
    ctx.expect_gap(
        mode_slider,
        body,
        axis="z",
        positive_elem="thumb",
        negative_elem="slider_guide",
        min_gap=0.0,
        max_gap=0.006,
        name="mode slider thumb rides on the guide",
    )
    ctx.expect_gap(
        start_button,
        body,
        axis="z",
        positive_elem="cap",
        negative_elem="button_bezel",
        min_gap=0.0,
        max_gap=0.006,
        name="start button cap sits above the bezel",
    )
    ctx.expect_gap(
        safety_flap,
        body,
        axis="z",
        positive_elem="plate",
        negative_elem="shell",
        min_gap=0.0,
        max_gap=0.004,
        name="safety flap rests just above the paper slot",
    )
    ctx.expect_overlap(
        cutter_drum_0,
        cutter_drum_1,
        axes="x",
        min_overlap=0.20,
        name="cutter drums span the slot width together",
    )
    ctx.expect_origin_distance(
        cutter_drum_0,
        cutter_drum_1,
        axes="y",
        min_dist=0.026,
        max_dist=0.034,
        name="cutter drums sit as a close horizontal pair",
    )

    bin_rest = ctx.part_world_position(bin_part)
    with ctx.pose({bin_joint: 0.120}):
        ctx.expect_overlap(
            bin_part,
            body,
            axes="xz",
            min_overlap=0.20,
            name="extended bin stays aligned with the cabinet",
        )
        bin_extended = ctx.part_world_position(bin_part)
    ctx.check(
        "bin slides forward out of the lower body",
        bin_rest is not None and bin_extended is not None and bin_extended[1] > bin_rest[1] + 0.10,
        details=f"rest={bin_rest}, extended={bin_extended}",
    )

    with ctx.pose({slider_joint: -0.022}):
        slider_low = ctx.part_world_position(mode_slider)
    with ctx.pose({slider_joint: 0.022}):
        slider_high = ctx.part_world_position(mode_slider)
    ctx.check(
        "mode slider traverses its top guide",
        slider_low is not None and slider_high is not None and slider_high[0] > slider_low[0] + 0.035,
        details=f"low={slider_low}, high={slider_high}",
    )

    button_rest = ctx.part_world_position(start_button)
    with ctx.pose({button_joint: 0.004}):
        button_pressed = ctx.part_world_position(start_button)
    ctx.check(
        "start button depresses downward",
        button_rest is not None and button_pressed is not None and button_pressed[2] < button_rest[2] - 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    flap_closed = ctx.part_element_world_aabb(safety_flap, elem="plate")
    with ctx.pose({flap_joint: 1.10}):
        flap_open = ctx.part_element_world_aabb(safety_flap, elem="plate")
    ctx.check(
        "safety flap swings upward above the slot",
        flap_closed is not None and flap_open is not None and flap_open[1][2] > flap_closed[1][2] + 0.018,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    return ctx.report()


object_model = build_object_model()
