from __future__ import annotations

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


BODY_W = 0.330
BODY_D = 0.230
LOWER_H = 0.305
HEAD_W = 0.338
HEAD_D = 0.236
HEAD_H = 0.082
HEAD_BOTTOM = LOWER_H - 0.002
HEAD_TOP = HEAD_BOTTOM + HEAD_H

CONTROL_STRIP_W = 0.240
CONTROL_STRIP_D = 0.060
CONTROL_STRIP_H = 0.010
CONTROL_STRIP_Y = -0.018
CONTROL_STRIP_Z = HEAD_TOP + CONTROL_STRIP_H / 2.0 - 0.001
CONTROL_STRIP_TOP = CONTROL_STRIP_Z + CONTROL_STRIP_H / 2.0

SLIDER_X = -0.056
SLIDER_Y = CONTROL_STRIP_Y
SLIDER_TRENCH_DEPTH = 0.006
SLIDER_TRENCH_FLOOR_Z = CONTROL_STRIP_TOP - SLIDER_TRENCH_DEPTH - 0.0005

BUTTON_XS = (0.044, 0.090)
BUTTON_Y = CONTROL_STRIP_Y
BUTTON_TOP_Z = CONTROL_STRIP_TOP

BIN_W = 0.282
BIN_D = 0.186
BIN_H = 0.262
BIN_BOTTOM_Z = 0.013
BIN_CLOSED_Y = BODY_D / 2.0 - BIN_D / 2.0

DRUM_RADIUS = 0.016
DRUM_LENGTH = 0.292
DRUM_Z = HEAD_BOTTOM + 0.031
DRUM_YS = (-0.018, 0.018)


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]):
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _cylinder_z(radius: float, height: float, center: tuple[float, float, float]):
    cx, cy, cz = center
    return cq.Workplane("XY").circle(radius).extrude(height).translate((cx, cy, cz - height / 2.0))


def _make_lower_shell():
    outer = _box_at((BODY_W, BODY_D, LOWER_H), (0.0, 0.0, LOWER_H / 2.0))
    outer = outer.edges("|Z").fillet(0.018)

    cavity = _box_at(
        (0.288, 0.200, 0.302),
        (
            0.0,
            BODY_D / 2.0 - 0.090,
            0.013 + 0.151,
        ),
    )
    return outer.cut(cavity)


def _make_head_shell():
    head = _box_at((HEAD_W, HEAD_D, HEAD_H), (0.0, 0.0, HEAD_BOTTOM + HEAD_H / 2.0))
    head = head.edges("|Z").fillet(0.012)

    underside_cavity = _box_at(
        (0.292, 0.160, 0.056),
        (
            0.0,
            0.012,
            HEAD_BOTTOM + 0.028,
        ),
    )
    paper_slot = _box_at(
        (0.224, 0.007, 0.042),
        (
            0.0,
            0.038,
            HEAD_TOP - 0.018,
        ),
    )
    return head.cut(underside_cavity).cut(paper_slot)


def _make_control_strip():
    strip = _box_at(
        (CONTROL_STRIP_W, CONTROL_STRIP_D, CONTROL_STRIP_H),
        (0.0, CONTROL_STRIP_Y, CONTROL_STRIP_Z),
    )
    strip = strip.edges("|Z").fillet(0.004)

    slider_trench = _box_at(
        (0.078, 0.020, SLIDER_TRENCH_DEPTH + 0.001),
        (
            SLIDER_X,
            SLIDER_Y,
            CONTROL_STRIP_TOP - SLIDER_TRENCH_DEPTH / 2.0,
        ),
    )
    strip = strip.cut(slider_trench)

    for button_x in BUTTON_XS:
        counterbore = _cylinder_z(
            radius=0.0105,
            height=0.0036,
            center=(button_x, BUTTON_Y, CONTROL_STRIP_TOP - 0.0018),
        )
        stem_hole = _cylinder_z(
            radius=0.0060,
            height=CONTROL_STRIP_H + 0.001,
            center=(button_x, BUTTON_Y, CONTROL_STRIP_Z),
        )
        strip = strip.cut(counterbore).cut(stem_hole)

    return strip


def _make_bin():
    wall = 0.0032
    outer = _box_at((BIN_W, BIN_D, BIN_H), (0.0, 0.0, BIN_H / 2.0))
    outer = outer.edges("|Z").fillet(0.010)

    inner = _box_at(
        (BIN_W - 2.0 * wall, BIN_D - 2.0 * wall, BIN_H),
        (0.0, 0.0, wall + BIN_H / 2.0),
    )
    grip = _box_at(
        (0.132, 0.018, 0.030),
        (0.0, BIN_D / 2.0 - 0.005, BIN_H * 0.73),
    )
    return outer.cut(inner).cut(grip)


def _make_drum():
    drum = cq.Workplane("YZ").circle(0.010).extrude(DRUM_LENGTH / 2.0, both=True)
    for x in (
        -0.112,
        -0.098,
        -0.084,
        -0.070,
        -0.056,
        -0.042,
        -0.028,
        -0.014,
        0.0,
        0.014,
        0.028,
        0.042,
        0.056,
        0.070,
        0.084,
        0.098,
        0.112,
    ):
        drum = drum.union(
            cq.Workplane("YZ").workplane(offset=x).circle(DRUM_RADIUS).extrude(0.004, both=True)
        )
    return drum


def _make_button():
    stem = cq.Workplane("XY").circle(0.0060).extrude(0.0072).translate((0.0, 0.0, -0.0072))
    cap = cq.Workplane("XY").circle(0.0095).extrude(0.004)
    return stem.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_office_shredder")

    body_grey = model.material("body_grey", rgba=(0.79, 0.81, 0.83, 1.0))
    head_black = model.material("head_black", rgba=(0.11, 0.12, 0.13, 1.0))
    strip_black = model.material("strip_black", rgba=(0.18, 0.19, 0.20, 1.0))
    control_black = model.material("control_black", rgba=(0.13, 0.14, 0.15, 1.0))
    button_black = model.material("button_black", rgba=(0.15, 0.16, 0.17, 1.0))
    drum_dark = model.material("drum_dark", rgba=(0.21, 0.22, 0.23, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BIN_BOTTOM_Z)),
        origin=Origin(xyz=(0.0, 0.0, BIN_BOTTOM_Z / 2.0)),
        material=body_grey,
        name="base_floor",
    )
    body.visual(
        Box((0.020, BODY_D, LOWER_H - BIN_BOTTOM_Z)),
        origin=Origin(xyz=(-BODY_W / 2.0 + 0.010, 0.0, BIN_BOTTOM_Z + (LOWER_H - BIN_BOTTOM_Z) / 2.0)),
        material=body_grey,
        name="side_wall_0",
    )
    body.visual(
        Box((0.020, BODY_D, LOWER_H - BIN_BOTTOM_Z)),
        origin=Origin(xyz=(BODY_W / 2.0 - 0.010, 0.0, BIN_BOTTOM_Z + (LOWER_H - BIN_BOTTOM_Z) / 2.0)),
        material=body_grey,
        name="side_wall_1",
    )
    body.visual(
        Box((BODY_W - 0.040, 0.038, LOWER_H - BIN_BOTTOM_Z)),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_D / 2.0 + 0.019,
                BIN_BOTTOM_Z + (LOWER_H - BIN_BOTTOM_Z) / 2.0,
            )
        ),
        material=body_grey,
        name="back_wall",
    )
    body.visual(
        Box((0.023, HEAD_D, HEAD_H - 0.018)),
        origin=Origin(xyz=(-HEAD_W / 2.0 + 0.0115, 0.0, HEAD_BOTTOM + (HEAD_H - 0.018) / 2.0)),
        material=head_black,
        name="head_side_0",
    )
    body.visual(
        Box((0.023, HEAD_D, HEAD_H - 0.018)),
        origin=Origin(xyz=(HEAD_W / 2.0 - 0.0115, 0.0, HEAD_BOTTOM + (HEAD_H - 0.018) / 2.0)),
        material=head_black,
        name="head_side_1",
    )
    body.visual(
        Box((HEAD_W - 0.046, 0.038, HEAD_H - 0.018)),
        origin=Origin(
            xyz=(
                0.0,
                -HEAD_D / 2.0 + 0.019,
                HEAD_BOTTOM + (HEAD_H - 0.018) / 2.0,
            )
        ),
        material=head_black,
        name="head_back",
    )
    body.visual(
        Box((HEAD_W - 0.046, 0.028, HEAD_H - 0.018)),
        origin=Origin(
            xyz=(
                0.0,
                HEAD_D / 2.0 - 0.014,
                HEAD_BOTTOM + (HEAD_H - 0.018) / 2.0,
            )
        ),
        material=head_black,
        name="head_front",
    )
    body.visual(
        Box((HEAD_W, 0.148, 0.018)),
        origin=Origin(xyz=(0.0, -0.044, HEAD_TOP - 0.009)),
        material=head_black,
        name="roof_rear",
    )
    body.visual(
        Box((HEAD_W, 0.080, 0.018)),
        origin=Origin(xyz=(0.0, 0.079, HEAD_TOP - 0.009)),
        material=head_black,
        name="roof_front",
    )
    body.visual(
        Box((0.060, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.034, HEAD_TOP - 0.003)),
        material=head_black,
        name="slot_bezel",
    )
    body.visual(
        Box((CONTROL_STRIP_W, CONTROL_STRIP_D, CONTROL_STRIP_H)),
        origin=Origin(xyz=(0.0, CONTROL_STRIP_Y, CONTROL_STRIP_Z)),
        material=strip_black,
        name="control_strip",
    )
    body.visual(
        Box((0.086, 0.004, 0.004)),
        origin=Origin(xyz=(SLIDER_X, SLIDER_Y - 0.008, CONTROL_STRIP_TOP + 0.002)),
        material=control_black,
        name="slider_guide_0",
    )
    body.visual(
        Box((0.086, 0.004, 0.004)),
        origin=Origin(xyz=(SLIDER_X, SLIDER_Y + 0.008, CONTROL_STRIP_TOP + 0.002)),
        material=control_black,
        name="slider_guide_1",
    )

    bin_part = model.part("bin")
    bin_part.visual(
        mesh_from_cadquery(_make_bin(), "shredder_bin"),
        material=body_grey,
        name="bin_shell",
    )
    bin_part.visual(
        Box((0.256, 0.003, 0.132)),
        origin=Origin(xyz=(0.0, BIN_D / 2.0 - 0.0025, BIN_H * 0.50)),
        material=strip_black,
        name="bin_window",
    )
    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.0, BIN_CLOSED_Y, BIN_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=0.112,
        ),
    )

    drum_mesh = mesh_from_cadquery(_make_drum(), "shredder_drum")
    for index, drum_y in enumerate(DRUM_YS):
        drum_part = model.part(f"drum_{index}")
        drum_part.visual(
            drum_mesh,
            material=drum_dark,
            name="drum",
        )
        model.articulation(
            f"body_to_drum_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=drum_part,
            origin=Origin(xyz=(0.0, drum_y, DRUM_Z)),
            axis=((1.0, 0.0, 0.0) if index == 0 else (-1.0, 0.0, 0.0)),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=20.0,
            ),
        )

    slider = model.part("mode_slider")
    slider.visual(
        Box((0.018, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=control_black,
        name="slider_base",
    )
    slider.visual(
        Box((0.006, 0.008, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=control_black,
        name="slider_stem",
    )
    slider.visual(
        Box((0.028, 0.011, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=control_black,
        name="slider_cap",
    )
    model.articulation(
        "body_to_mode_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(SLIDER_X, SLIDER_Y, CONTROL_STRIP_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=-0.016,
            upper=0.016,
        ),
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.0095, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=button_black,
            name="button",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, BUTTON_Y, BUTTON_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0036,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    slider = object_model.get_part("mode_slider")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    drum_0 = object_model.get_part("drum_0")
    drum_1 = object_model.get_part("drum_1")

    bin_joint = object_model.get_articulation("body_to_bin")
    slider_joint = object_model.get_articulation("body_to_mode_slider")
    button_joint_0 = object_model.get_articulation("body_to_button_0")
    button_joint_1 = object_model.get_articulation("body_to_button_1")

    with ctx.pose({bin_joint: 0.0}):
        bin_aabb = ctx.part_element_world_aabb(bin_part, elem="bin_shell")
        body_aabb = ctx.part_world_aabb(body)
        flush_gap = None
        if bin_aabb is not None and body_aabb is not None:
            flush_gap = abs(bin_aabb[1][1] - body_aabb[1][1])
        ctx.check(
            "closed bin front sits nearly flush with the cabinet",
            flush_gap is not None and flush_gap <= 0.004,
            details=f"flush_gap={flush_gap}, bin_aabb={bin_aabb}, body_aabb={body_aabb}",
        )
        ctx.expect_overlap(
            bin_part,
            body,
            axes="x",
            elem_a="bin_shell",
            min_overlap=0.270,
            name="closed bin stays centered across the cabinet width",
        )
        ctx.expect_gap(
            drum_0,
            bin_part,
            axis="z",
            positive_elem="drum",
            negative_elem="bin_shell",
            min_gap=0.035,
            max_gap=0.095,
            name="front drum sits above the open bin mouth",
        )
        ctx.expect_gap(
            drum_1,
            bin_part,
            axis="z",
            positive_elem="drum",
            negative_elem="bin_shell",
            min_gap=0.035,
            max_gap=0.095,
            name="rear drum sits above the open bin mouth",
        )

    bin_limits = bin_joint.motion_limits
    if bin_limits is not None and bin_limits.upper is not None:
        closed_pos = ctx.part_world_position(bin_part)
        with ctx.pose({bin_joint: bin_limits.upper}):
            extended_pos = ctx.part_world_position(bin_part)
            ctx.expect_overlap(
                bin_part,
                body,
                axes="y",
                elem_a="bin_shell",
                min_overlap=0.070,
                name="extended bin keeps retained insertion inside the cabinet",
            )
        ctx.check(
            "bin slides outward from the cabinet",
            closed_pos is not None
            and extended_pos is not None
            and extended_pos[1] > closed_pos[1] + 0.10,
            details=f"closed={closed_pos}, extended={extended_pos}",
        )

    slider_limits = slider_joint.motion_limits
    if slider_limits is not None and slider_limits.lower is not None and slider_limits.upper is not None:
        with ctx.pose({slider_joint: slider_limits.lower}):
            slider_low = ctx.part_world_position(slider)
        with ctx.pose({slider_joint: slider_limits.upper}):
            slider_high = ctx.part_world_position(slider)
        ctx.check(
            "mode slider traverses laterally across the control strip",
            slider_low is not None
            and slider_high is not None
            and slider_high[0] > slider_low[0] + 0.025
            and abs(slider_high[2] - slider_low[2]) < 0.001,
            details=f"low={slider_low}, high={slider_high}",
        )

    for button_part, button_joint, label in (
        (button_0, button_joint_0, "front button"),
        (button_1, button_joint_1, "rear button"),
    ):
        limits = button_joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        rest = ctx.part_world_position(button_part)
        with ctx.pose({button_joint: limits.upper}):
            pressed = ctx.part_world_position(button_part)
        ctx.check(
            f"{label} depresses downward",
            rest is not None and pressed is not None and pressed[2] < rest[2] - 0.0025,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
