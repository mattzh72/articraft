from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


HINGE_AXIS = (0.0, 1.0, 0.0)


def _pin_cap(part, *, x: float, y: float, radius: float, material, name: str) -> None:
    """Add a short round pin head on a hinge cheek."""
    part.visual(
        Cylinder(radius=radius, length=0.006),
        origin=Origin(xyz=(x, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _fork(
    part,
    *,
    x: float,
    gap: float,
    cheek_thickness: float,
    cheek_length: float,
    height: float,
    material,
    pin_material,
    prefix: str,
) -> None:
    """Two boxed hinge cheeks with visible metal pin caps, centered on the Y axis."""
    for index, sign in enumerate((1.0, -1.0)):
        y_center = sign * (gap * 0.5 + cheek_thickness * 0.5)
        outer_y = sign * (gap * 0.5 + cheek_thickness)
        part.visual(
            Box((cheek_length, cheek_thickness, height)),
            origin=Origin(xyz=(x, y_center, 0.0)),
            material=material,
            name=f"{prefix}_cheek_{index}",
        )
        _pin_cap(
            part,
            x=x,
            y=outer_y + sign * 0.0015,
            radius=height * 0.22,
            material=pin_material,
            name=f"{prefix}_pin_{index}",
        )


def _phalanx(
    part,
    *,
    length: float,
    body_width: float,
    body_height: float,
    tongue_width: float,
    tongue_length: float,
    knuckle_height: float,
    body_material,
    knuckle_material,
    pin_material,
    fork_gap: float | None,
    prefix: str,
) -> None:
    """Build a robotic phalanx with a central proximal knuckle and optional distal fork."""
    part.visual(
        Box((tongue_length, tongue_width, knuckle_height)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=knuckle_material,
        name="base_knuckle",
    )
    part.visual(
        Box((0.026, tongue_width * 0.86, body_height * 0.88)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=body_material,
        name="knuckle_neck",
    )

    body_start = 0.022
    if fork_gap is None:
        body_end = length
    else:
        body_end = length - 0.015
    part.visual(
        Box((body_end - body_start, body_width, body_height)),
        origin=Origin(xyz=((body_start + body_end) * 0.5, 0.0, 0.0)),
        material=body_material,
        name="link_body",
    )

    if fork_gap is not None:
        cheek_thickness = max(0.010, (body_width + 0.008 - fork_gap) * 0.5)
        cheek_y = fork_gap * 0.5 + cheek_thickness * 0.5
        outer_y = fork_gap * 0.5 + cheek_thickness
        part.visual(
            Box((0.036, cheek_thickness, knuckle_height)),
            origin=Origin(xyz=(length, cheek_y, 0.0)),
            material=knuckle_material,
            name="distal_cheek_0",
        )
        _pin_cap(
            part,
            x=length,
            y=outer_y + 0.0015,
            radius=knuckle_height * 0.22,
            material=pin_material,
            name="distal_pin_0",
        )
        part.visual(
            Box((0.036, cheek_thickness, knuckle_height)),
            origin=Origin(xyz=(length, -cheek_y, 0.0)),
            material=knuckle_material,
            name="distal_cheek_1",
        )
        _pin_cap(
            part,
            x=length,
            y=-outer_y - 0.0015,
            radius=knuckle_height * 0.22,
            material=pin_material,
            name="distal_pin_1",
        )
        part.visual(
            Cylinder(radius=knuckle_height * 0.16, length=body_width + 0.014),
            origin=Origin(xyz=(length, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pin_material,
            name="distal_pin_shaft",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_articulated_finger")

    body_mat = model.material("matte_graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    knuckle_mat = model.material("black_knuckle_blocks", rgba=(0.035, 0.040, 0.045, 1.0))
    root_mat = model.material("dark_palm_block", rgba=(0.10, 0.12, 0.14, 1.0))
    metal_mat = model.material("brushed_pin_metal", rgba=(0.68, 0.70, 0.70, 1.0))
    rubber_mat = model.material("soft_tip_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    root = model.part("root_block")
    root.visual(
        Box((0.072, 0.076, 0.048)),
        origin=Origin(xyz=(-0.052, 0.0, 0.0)),
        material=root_mat,
        name="palm_block",
    )
    root.visual(
        Box((0.028, 0.066, 0.014)),
        origin=Origin(xyz=(-0.006, 0.0, -0.029)),
        material=root_mat,
        name="palm_foot",
    )
    root.visual(
        Box((0.036, 0.014, 0.044)),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=knuckle_mat,
        name="root_cheek_0",
    )
    _pin_cap(
        root,
        x=0.0,
        y=0.0325,
        radius=0.0097,
        material=metal_mat,
        name="root_pin_0",
    )
    root.visual(
        Box((0.036, 0.014, 0.044)),
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
        material=knuckle_mat,
        name="root_cheek_1",
    )
    _pin_cap(
        root,
        x=0.0,
        y=-0.0325,
        radius=0.0097,
        material=metal_mat,
        name="root_pin_1",
    )
    root.visual(
        Cylinder(radius=0.0065, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="root_pin_shaft",
    )
    for index, y in enumerate((-0.020, 0.020)):
        root.visual(
            Cylinder(radius=0.0055, length=0.0035),
            origin=Origin(xyz=(-0.054, y, 0.025)),
            material=metal_mat,
            name=f"screw_head_{index}",
        )

    proximal = model.part("proximal_phalanx")
    _phalanx(
        proximal,
        length=0.105,
        body_width=0.048,
        body_height=0.027,
        tongue_width=0.030,
        tongue_length=0.030,
        knuckle_height=0.038,
        body_material=body_mat,
        knuckle_material=knuckle_mat,
        pin_material=metal_mat,
        fork_gap=0.031,
        prefix="proximal",
    )

    middle = model.part("middle_phalanx")
    _phalanx(
        middle,
        length=0.075,
        body_width=0.041,
        body_height=0.024,
        tongue_width=0.027,
        tongue_length=0.028,
        knuckle_height=0.034,
        body_material=body_mat,
        knuckle_material=knuckle_mat,
        pin_material=metal_mat,
        fork_gap=0.028,
        prefix="middle",
    )

    distal = model.part("distal_phalanx")
    _phalanx(
        distal,
        length=0.056,
        body_width=0.034,
        body_height=0.021,
        tongue_width=0.024,
        tongue_length=0.026,
        knuckle_height=0.030,
        body_material=body_mat,
        knuckle_material=knuckle_mat,
        pin_material=metal_mat,
        fork_gap=None,
        prefix="distal",
    )
    distal.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.059, 0.0, 0.0)),
        material=rubber_mat,
        name="distal_tip",
    )

    model.articulation(
        "root_to_proximal",
        ArticulationType.REVOLUTE,
        parent=root,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=1.8, velocity=4.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=0.0, upper=1.10),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_block")
    proximal = object_model.get_part("proximal_phalanx")
    middle = object_model.get_part("middle_phalanx")
    distal = object_model.get_part("distal_phalanx")
    root_joint = object_model.get_articulation("root_to_proximal")
    middle_joint = object_model.get_articulation("proximal_to_middle")
    distal_joint = object_model.get_articulation("middle_to_distal")
    joints = (root_joint, middle_joint, distal_joint)

    ctx.check(
        "three revolute finger hinges",
        len(joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
    )
    ctx.check(
        "parallel hinge axes",
        all(tuple(round(v, 6) for v in j.axis) == HINGE_AXIS for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )
    ctx.allow_overlap(
        root,
        proximal,
        elem_a="root_pin_shaft",
        elem_b="base_knuckle",
        reason="The root hinge pin is intentionally captured through the proximal knuckle bore.",
    )
    ctx.allow_overlap(
        proximal,
        middle,
        elem_a="distal_pin_shaft",
        elem_b="base_knuckle",
        reason="The proximal hinge pin is intentionally captured through the middle knuckle bore.",
    )
    ctx.allow_overlap(
        middle,
        distal,
        elem_a="distal_pin_shaft",
        elem_b="base_knuckle",
        reason="The middle hinge pin is intentionally captured through the distal knuckle bore.",
    )

    with ctx.pose({root_joint: 0.0, middle_joint: 0.0, distal_joint: 0.0}):
        ctx.expect_overlap(
            root,
            proximal,
            axes="xyz",
            elem_a="root_pin_shaft",
            elem_b="base_knuckle",
            min_overlap=0.010,
            name="root pin passes through proximal knuckle",
        )
        ctx.expect_overlap(
            proximal,
            root,
            axes="xz",
            elem_a="base_knuckle",
            elem_b="root_cheek_0",
            min_overlap=0.020,
            name="root hinge cheeks surround proximal knuckle",
        )
        ctx.expect_gap(
            root,
            proximal,
            axis="y",
            positive_elem="root_cheek_0",
            negative_elem="base_knuckle",
            min_gap=0.001,
            max_gap=0.004,
            name="root hinge side clearance",
        )
        ctx.expect_overlap(
            proximal,
            middle,
            axes="xyz",
            elem_a="distal_pin_shaft",
            elem_b="base_knuckle",
            min_overlap=0.009,
            name="proximal pin passes through middle knuckle",
        )
        ctx.expect_overlap(
            middle,
            proximal,
            axes="xz",
            elem_a="base_knuckle",
            elem_b="distal_cheek_0",
            min_overlap=0.020,
            name="middle knuckle captured by proximal fork",
        )
        ctx.expect_gap(
            proximal,
            middle,
            axis="y",
            positive_elem="distal_cheek_0",
            negative_elem="base_knuckle",
            min_gap=0.001,
            max_gap=0.004,
            name="proximal hinge side clearance",
        )
        ctx.expect_overlap(
            middle,
            distal,
            axes="xyz",
            elem_a="distal_pin_shaft",
            elem_b="base_knuckle",
            min_overlap=0.008,
            name="middle pin passes through distal knuckle",
        )
        ctx.expect_overlap(
            distal,
            middle,
            axes="xz",
            elem_a="base_knuckle",
            elem_b="distal_cheek_0",
            min_overlap=0.020,
            name="distal knuckle captured by middle fork",
        )
        ctx.expect_gap(
            middle,
            distal,
            axis="y",
            positive_elem="distal_cheek_0",
            negative_elem="base_knuckle",
            min_gap=0.001,
            max_gap=0.004,
            name="middle hinge side clearance",
        )
        rest_pos = ctx.part_world_position(distal)

    with ctx.pose({root_joint: 0.80, middle_joint: 0.95, distal_joint: 0.65}):
        flexed_pos = ctx.part_world_position(distal)

    ctx.check(
        "finger curls toward palm side",
        rest_pos is not None
        and flexed_pos is not None
        and flexed_pos[2] < rest_pos[2] - 0.045
        and flexed_pos[0] < rest_pos[0] - 0.030,
        details=f"rest={rest_pos}, flexed={flexed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
