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


AXIS_Y = Origin(rpy=(pi / 2.0, 0.0, 0.0))
AXIS_X = Origin(rpy=(0.0, pi / 2.0, 0.0))

BOSS_RADIUS = 0.025
CENTER_LUG_WIDTH = 0.052
SIDE_PLATE_THICKNESS = 0.018
SIDE_PLATE_Y = 0.039
BODY_WIDTH = 0.040
BODY_THICKNESS = 0.028


def y_axis_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=AXIS_Y.rpy)


def x_axis_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=AXIS_X.rpy)


def add_pin_cap(part, x: float, y: float, z: float, radius: float, length: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=y_axis_origin(x, y, z),
        material=material,
        name=name,
    )


def add_central_lug(part, material, steel) -> None:
    part.visual(
        Cylinder(radius=BOSS_RADIUS, length=CENTER_LUG_WIDTH),
        origin=y_axis_origin(0.0, 0.0, 0.0),
        material=material,
        name="proximal_boss",
    )


def add_forked_segment(part, length: float, material, steel) -> None:
    """Flat service-gripper link with a central proximal lug and forked distal boss."""
    add_central_lug(part, material, steel)

    body_start = 0.010
    body_end = length - 0.055
    part.visual(
        Box((body_end - body_start, BODY_WIDTH, BODY_THICKNESS)),
        origin=Origin(xyz=((body_start + body_end) / 2.0, 0.0, 0.0)),
        material=material,
        name="flat_web",
    )

    # A short cross bridge ties the central web to both distal side plates but
    # stays behind the child lug so the hinge slot remains visibly open.
    part.visual(
        Box((0.028, 0.092, BODY_THICKNESS)),
        origin=Origin(xyz=(length - 0.065, 0.0, 0.0)),
        material=material,
        name="distal_bridge",
    )
    part.visual(
        Cylinder(radius=0.0075, length=0.112),
        origin=y_axis_origin(length, 0.0, 0.0),
        material=steel,
        name="distal_pin",
    )

    for suffix, sign in (("pos", 1.0), ("neg", -1.0)):
        side_y = sign * SIDE_PLATE_Y
        part.visual(
            Box((0.076, SIDE_PLATE_THICKNESS, 0.032)),
            origin=Origin(xyz=(length - 0.038, side_y, 0.0)),
            material=material,
            name=f"distal_plate_{suffix}",
        )
        part.visual(
            Cylinder(radius=BOSS_RADIUS, length=SIDE_PLATE_THICKNESS),
            origin=y_axis_origin(length, side_y, 0.0),
            material=material,
            name=f"distal_boss_{suffix}",
        )
        cap_y = sign * (SIDE_PLATE_Y + SIDE_PLATE_THICKNESS / 2.0 + 0.0025)
        add_pin_cap(part, length, cap_y, 0.0, 0.011, 0.005, steel, f"distal_cap_{suffix}")


def add_tip_segment(part, length: float, material, steel, rubber) -> None:
    add_central_lug(part, material, steel)

    body_start = 0.010
    body_end = length - 0.030
    part.visual(
        Box((body_end - body_start, BODY_WIDTH, BODY_THICKNESS)),
        origin=Origin(xyz=((body_start + body_end) / 2.0, 0.0, 0.0)),
        material=material,
        name="flat_web",
    )
    part.visual(
        Box((0.040, 0.056, 0.042)),
        origin=Origin(xyz=(length - 0.020, 0.0, 0.0)),
        material=material,
        name="squared_tip",
    )
    part.visual(
        Box((0.007, 0.060, 0.046)),
        origin=Origin(xyz=(length + 0.0035, 0.0, 0.0)),
        material=rubber,
        name="fingertip_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_gripper_finger_chain")

    aluminum = Material("matte_anodized_aluminum", rgba=(0.54, 0.58, 0.62, 1.0))
    dark_aluminum = Material("dark_side_plates", rgba=(0.18, 0.20, 0.22, 1.0))
    steel = Material("brushed_pin_steel", rgba=(0.74, 0.72, 0.66, 1.0))
    rubber = Material("black_rubber_pad", rgba=(0.015, 0.014, 0.013, 1.0))

    root = model.part("root_mount")
    root.visual(
        Box((0.030, 0.130, 0.130)),
        origin=Origin(xyz=(-0.115, 0.0, 0.0)),
        material=dark_aluminum,
        name="mount_plate",
    )
    root.visual(
        Box((0.075, 0.090, 0.090)),
        origin=Origin(xyz=(-0.0675, 0.0, 0.0)),
        material=dark_aluminum,
        name="root_block",
    )
    for suffix, sign in (("pos", 1.0), ("neg", -1.0)):
        cheek_y = sign * 0.040
        root.visual(
            Box((0.070, SIDE_PLATE_THICKNESS, 0.075)),
            origin=Origin(xyz=(0.0, cheek_y, 0.0)),
            material=dark_aluminum,
            name=f"fork_plate_{suffix}",
        )
        root.visual(
            Cylinder(radius=0.027, length=0.015),
            origin=y_axis_origin(0.0, sign * 0.0555, 0.0),
            material=dark_aluminum,
            name=f"root_boss_{suffix}",
        )
    root.visual(
        Cylinder(radius=0.0075, length=0.116),
        origin=y_axis_origin(0.0, 0.0, 0.0),
        material=steel,
        name="root_pin",
    )
    for y in (-0.042, 0.042):
        for z in (-0.042, 0.042):
            root.visual(
                Cylinder(radius=0.0065, length=0.009),
                origin=x_axis_origin(-0.134, y, z),
                material=steel,
                name=f"bolt_{'p' if y > 0 else 'n'}_{'p' if z > 0 else 'n'}",
            )

    segment_0 = model.part("segment_0")
    segment_1 = model.part("segment_1")
    segment_2 = model.part("segment_2")

    length_0 = 0.160
    length_1 = 0.130
    length_2 = 0.105

    add_forked_segment(segment_0, length_0, aluminum, steel)
    add_forked_segment(segment_1, length_1, aluminum, steel)
    add_tip_segment(segment_2, length_2, aluminum, steel, rubber)

    limits = MotionLimits(effort=18.0, velocity=3.5, lower=0.0, upper=1.15)
    model.articulation(
        "mount_joint",
        ArticulationType.REVOLUTE,
        parent=root,
        child=segment_0,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "knuckle_0",
        ArticulationType.REVOLUTE,
        parent=segment_0,
        child=segment_1,
        origin=Origin(xyz=(length_0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "knuckle_1",
        ArticulationType.REVOLUTE,
        parent=segment_1,
        child=segment_2,
        origin=Origin(xyz=(length_1, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mount_joint = object_model.get_articulation("mount_joint")
    knuckle_0 = object_model.get_articulation("knuckle_0")
    knuckle_1 = object_model.get_articulation("knuckle_1")
    root_mount = object_model.get_part("root_mount")
    segment_0 = object_model.get_part("segment_0")
    segment_1 = object_model.get_part("segment_1")
    segment_2 = object_model.get_part("segment_2")

    joints = (mount_joint, knuckle_0, knuckle_1)
    ctx.check(
        "three serial revolute joints",
        len(joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and [j.parent for j in joints] == ["root_mount", "segment_0", "segment_1"]
        and [j.child for j in joints] == ["segment_0", "segment_1", "segment_2"],
    )
    ctx.check(
        "joint axes share bending plane",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
    )

    for parent, child, pin_elem in (
        (root_mount, segment_0, "root_pin"),
        (segment_0, segment_1, "distal_pin"),
        (segment_1, segment_2, "distal_pin"),
    ):
        ctx.allow_overlap(
            parent,
            child,
            elem_a=pin_elem,
            elem_b="proximal_boss",
            reason="A captured steel hinge pin intentionally passes through the child boss bore.",
        )
        ctx.expect_within(
            parent,
            child,
            axes="xz",
            inner_elem=pin_elem,
            outer_elem="proximal_boss",
            name=f"{parent.name} pin centered in {child.name} boss",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            min_overlap=0.045,
            elem_a=pin_elem,
            elem_b="proximal_boss",
            name=f"{parent.name} pin spans {child.name} boss",
        )

    ctx.expect_origin_gap(
        segment_1,
        segment_0,
        axis="x",
        min_gap=0.159,
        max_gap=0.161,
        name="first link length at rest",
    )
    ctx.expect_origin_gap(
        segment_2,
        segment_1,
        axis="x",
        min_gap=0.129,
        max_gap=0.131,
        name="second link length at rest",
    )

    tip_rest = ctx.part_element_world_aabb(segment_2, elem="fingertip_pad")
    with ctx.pose({mount_joint: 0.65, knuckle_0: 0.55, knuckle_1: 0.45}):
        tip_curled = ctx.part_element_world_aabb(segment_2, elem="fingertip_pad")
        seg2_pos = ctx.part_world_position(segment_2)
    ctx.check(
        "positive motion curls fingertip downward",
        tip_rest is not None
        and tip_curled is not None
        and tip_curled[0][2] < tip_rest[0][2] - 0.070
        and seg2_pos is not None
        and abs(seg2_pos[1]) < 0.002,
        details=f"rest_tip={tip_rest}, curled_tip={tip_curled}, segment_2_pos={seg2_pos}",
    )

    return ctx.report()


object_model = build_object_model()
