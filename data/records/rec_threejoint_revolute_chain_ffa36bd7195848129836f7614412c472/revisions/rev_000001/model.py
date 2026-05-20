from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isfinite

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PIN_RADIUS = 0.005
PIN_CLEARANCE = 0.0007
PIN_HEAD_RADIUS = 0.007
PIN_HEAD_THICK = 0.0025

EAR_LENGTH = 0.018
EAR_THICK = 0.006
EAR_HEIGHT = 0.042
CLEVIS_GAP = 0.014
CLEVIS_SPAN_Y = CLEVIS_GAP + 2.0 * EAR_THICK
EAR_CENTER_Y = CLEVIS_GAP / 2.0 + EAR_THICK / 2.0

TANG_THICK = EAR_LENGTH
TANG_WIDTH = 0.012
TANG_HEIGHT = 0.036

LINK_WIDTH = 0.028
LINK_HEIGHT = 0.036
LINK_WALL = 0.0045
PROXIMAL_BLOCK_LEN = 0.022
DISTAL_BLOCK_LEN = 0.022
JOINT_FACE_CLEARANCE = 0.0

LINK_1_SPAN = 0.160
LINK_2_SPAN = 0.145
OUTPUT_ARM_LEN = 0.070
OUTPUT_PAD_LEN = 0.055
OUTPUT_PAD_WIDTH = 0.050
OUTPUT_PAD_THICK = 0.012
OUTPUT_PAD_HOLE_RADIUS = 0.0032


def _solid_block(length: float, width: float, height: float, x0: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, True))
        .translate((x0, 0.0, 0.0))
    )


def _tube_segment(length: float, width: float, height: float, wall: float, x0: float) -> cq.Workplane:
    outer = cq.Workplane("YZ").rect(width, height).extrude(length)
    inner = cq.Workplane("YZ").rect(width - 2.0 * wall, height - 2.0 * wall).extrude(length)
    return outer.cut(inner).translate((x0, 0.0, 0.0))


def _y_cylinder(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((x, y, z))
    )


def _z_cylinder(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((x, y, z))
    )


def _union_all(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _axis_bosses(x_pos: float) -> cq.Workplane:
    boss_length = 0.006
    positive_boss = _y_cylinder(
        PIN_HEAD_RADIUS,
        boss_length,
        x=x_pos,
        y=CLEVIS_SPAN_Y / 2.0 + boss_length / 2.0 - 0.001,
    )
    negative_boss = _y_cylinder(
        PIN_HEAD_RADIUS,
        boss_length,
        x=x_pos,
        y=-(CLEVIS_SPAN_Y / 2.0 + boss_length / 2.0 - 0.001),
    )
    return _union_all(positive_boss, negative_boss)


def _clevis_ears(x_pos: float) -> cq.Workplane:
    return _union_all(
        _solid_block(EAR_LENGTH, EAR_THICK, EAR_HEIGHT, x_pos - EAR_LENGTH / 2.0).translate((0.0, EAR_CENTER_Y, 0.0)),
        _solid_block(EAR_LENGTH, EAR_THICK, EAR_HEIGHT, x_pos - EAR_LENGTH / 2.0).translate((0.0, -EAR_CENTER_Y, 0.0)),
    )


def _link_shape(span: float) -> cq.Workplane:
    x_after_joint = EAR_LENGTH / 2.0 + JOINT_FACE_CLEARANCE
    x_after_prox = x_after_joint + PROXIMAL_BLOCK_LEN
    x_before_clevis = span - EAR_LENGTH / 2.0
    x_before_distal = x_before_clevis - DISTAL_BLOCK_LEN

    proximal_block = _solid_block(PROXIMAL_BLOCK_LEN, LINK_WIDTH * 0.72, LINK_HEIGHT * 0.78, x_after_joint)
    main_tube = _solid_block(x_before_distal - x_after_prox, LINK_WIDTH, LINK_HEIGHT, x_after_prox)
    distal_block = _solid_block(DISTAL_BLOCK_LEN, LINK_WIDTH, LINK_HEIGHT, x_before_distal)
    clevis = _clevis_ears(span)
    pin = _axis_bosses(span)

    return _union_all(proximal_block, main_tube, distal_block, clevis, pin)


def _root_clevis_shape() -> cq.Workplane:
    clevis = _clevis_ears(0.0)
    pin = _axis_bosses(0.0)
    rear_bridge = _solid_block(0.030, 0.030, EAR_HEIGHT, -0.039)
    main_body = _solid_block(0.052, 0.050, 0.048, -0.089)
    foot = _solid_block(0.078, 0.060, 0.010, -0.095).translate((0.0, 0.0, -0.029))
    foot_hole_a = _z_cylinder(0.0042, 0.014, x=-0.075, y=0.018, z=-0.029)
    foot_hole_b = _z_cylinder(0.0042, 0.014, x=-0.075, y=-0.018, z=-0.029)
    foot_hole_c = _z_cylinder(0.0042, 0.014, x=-0.036, y=0.018, z=-0.029)
    foot_hole_d = _z_cylinder(0.0042, 0.014, x=-0.036, y=-0.018, z=-0.029)
    return _union_all(clevis, pin, rear_bridge, main_body, foot).cut(
        _union_all(foot_hole_a, foot_hole_b, foot_hole_c, foot_hole_d)
    )


def _output_arm_shape() -> cq.Workplane:
    x_after_joint = EAR_LENGTH / 2.0 + JOINT_FACE_CLEARANCE
    neck = _solid_block(OUTPUT_ARM_LEN - x_after_joint, 0.022, 0.026, x_after_joint)
    return neck


def _output_pad_shape() -> cq.Workplane:
    pad_x0 = OUTPUT_ARM_LEN
    pad = _solid_block(OUTPUT_PAD_LEN, OUTPUT_PAD_WIDTH, OUTPUT_PAD_THICK, pad_x0)
    hole_offset = 0.014
    hole_x = pad_x0 + OUTPUT_PAD_LEN * 0.5
    pad_holes = _union_all(
        _z_cylinder(OUTPUT_PAD_HOLE_RADIUS, OUTPUT_PAD_THICK + 0.004, x=hole_x, y=hole_offset),
        _z_cylinder(OUTPUT_PAD_HOLE_RADIUS, OUTPUT_PAD_THICK + 0.004, x=hole_x, y=-hole_offset),
    )
    return pad.cut(pad_holes)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    values = (
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
    )
    if not all(isfinite(v) for v in values):
        return None
    return values


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_three_link_hinge_chain")

    model.material("graphite_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("satin_link", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("pad_polymer", rgba=(0.17, 0.18, 0.20, 1.0))

    root_clevis = model.part("root_clevis")
    root_clevis.visual(
        mesh_from_cadquery(_root_clevis_shape(), "root_clevis"),
        material="graphite_steel",
        name="root_clevis_body",
    )

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(_link_shape(LINK_1_SPAN), "first_link"),
        material="satin_link",
        name="first_link_body",
    )

    second_link = model.part("second_link")
    second_link.visual(
        mesh_from_cadquery(_link_shape(LINK_2_SPAN), "second_link"),
        material="satin_link",
        name="second_link_body",
    )

    output_pad = model.part("output_pad")
    output_pad.visual(
        mesh_from_cadquery(_output_arm_shape(), "output_arm"),
        material="satin_link",
        name="output_arm",
    )
    output_pad.visual(
        mesh_from_cadquery(_output_pad_shape(), "output_pad"),
        material="pad_polymer",
        name="output_pad",
    )

    model.articulation(
        "root_to_first",
        ArticulationType.REVOLUTE,
        parent=root_clevis,
        child=first_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.20, effort=16.0, velocity=2.0),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(LINK_1_SPAN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.15, upper=1.20, effort=12.0, velocity=2.2),
    )
    model.articulation(
        "second_to_output",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=output_pad,
        origin=Origin(xyz=(LINK_2_SPAN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.20, upper=1.25, effort=8.0, velocity=2.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_clevis = object_model.get_part("root_clevis")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    output_pad = object_model.get_part("output_pad")

    root_to_first = object_model.get_articulation("root_to_first")
    first_to_second = object_model.get_articulation("first_to_second")
    second_to_output = object_model.get_articulation("second_to_output")

    ctx.check(
        "all chain parts exist",
        all(part is not None for part in (root_clevis, first_link, second_link, output_pad)),
        details="Expected root clevis, first link, second link, and output pad parts.",
    )
    ctx.check(
        "all revolute joints exist",
        all(joint is not None for joint in (root_to_first, first_to_second, second_to_output)),
        details="Expected three serial revolute joints in the hinge chain.",
    )

    ctx.expect_origin_distance(
        root_clevis,
        first_link,
        axes="xyz",
        max_dist=1e-6,
        name="root clevis and first link share the first hinge axis",
    )
    ctx.expect_contact(
        root_clevis,
        first_link,
        name="root clevis physically supports the first link",
    )
    ctx.expect_origin_gap(
        second_link,
        first_link,
        axis="x",
        min_gap=LINK_1_SPAN - 1e-6,
        max_gap=LINK_1_SPAN + 1e-6,
        name="first and second link hinge spacing is preserved at rest",
    )
    ctx.expect_contact(
        first_link,
        second_link,
        name="first link physically supports the second link",
    )
    ctx.expect_origin_gap(
        output_pad,
        second_link,
        axis="x",
        min_gap=LINK_2_SPAN - 1e-6,
        max_gap=LINK_2_SPAN + 1e-6,
        name="second link and output pad hinge spacing is preserved at rest",
    )
    ctx.expect_contact(
        second_link,
        output_pad,
        name="second link physically supports the output pad",
    )

    rest_second = ctx.part_world_position(second_link)
    with ctx.pose({root_to_first: 0.60}):
        raised_second = ctx.part_world_position(second_link)
    ctx.check(
        "positive root joint raises the downstream chain",
        rest_second is not None
        and raised_second is not None
        and raised_second[2] > rest_second[2] + 0.06,
        details=f"rest={rest_second}, raised={raised_second}",
    )

    rest_output_origin = ctx.part_world_position(output_pad)
    with ctx.pose({first_to_second: 0.70}):
        elbowed_output_origin = ctx.part_world_position(output_pad)
    ctx.check(
        "positive middle joint lifts the output member",
        rest_output_origin is not None
        and elbowed_output_origin is not None
        and elbowed_output_origin[2] > rest_output_origin[2] + 0.05,
        details=f"rest={rest_output_origin}, elbowed={elbowed_output_origin}",
    )

    rest_pad_center = _aabb_center(ctx.part_element_world_aabb(output_pad, elem="output_pad"))
    with ctx.pose({second_to_output: 0.75}):
        raised_pad_center = _aabb_center(ctx.part_element_world_aabb(output_pad, elem="output_pad"))
    ctx.check(
        "positive output joint pitches the pad upward",
        rest_pad_center is not None
        and raised_pad_center is not None
        and raised_pad_center[2] > rest_pad_center[2] + 0.02,
        details=f"rest={rest_pad_center}, raised={raised_pad_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
