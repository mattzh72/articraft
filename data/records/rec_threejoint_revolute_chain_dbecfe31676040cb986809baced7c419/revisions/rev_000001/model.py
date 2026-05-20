from __future__ import annotations

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_inspection_arm")

    wall = model.material("painted_wall", rgba=(0.78, 0.78, 0.74, 1.0))
    base_mat = model.material("powder_coated_base", rgba=(0.10, 0.12, 0.14, 1.0))
    link_mat = model.material("anodized_blue_links", rgba=(0.08, 0.22, 0.38, 1.0))
    pin_mat = model.material("brushed_pin_steel", rgba=(0.70, 0.70, 0.66, 1.0))
    rubber = model.material("matte_rubber_pad", rgba=(0.02, 0.02, 0.018, 1.0))

    hinge_rpy = (-math.pi / 2.0, 0.0, 0.0)  # SDK cylinders are local +Z; rotate them onto the hinge +Y axis.
    screw_rpy = (0.0, math.pi / 2.0, 0.0)  # screw heads face out along +X.

    def add_pin_caps(part, *, x: float, prefix: str) -> None:
        for y in (-0.074, 0.074):
            part.visual(
                Cylinder(radius=0.018, length=0.008),
                origin=Origin(xyz=(x, y, 0.0), rpy=hinge_rpy),
                material=pin_mat,
                name=f"{prefix}_cap_{'neg' if y < 0 else 'pos'}",
            )

    def add_reach_link(part, *, length: float, bar_height: float) -> None:
        part.visual(
            Cylinder(radius=0.030, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=hinge_rpy),
            material=link_mat,
            name="proximal_barrel",
        )
        part.visual(
            Box((length - 0.078, 0.036, bar_height)),
            origin=Origin(xyz=(length / 2.0 - 0.021, 0.0, 0.0)),
            material=link_mat,
            name="main_spine",
        )
        part.visual(
            Box((0.030, 0.110, bar_height)),
            origin=Origin(xyz=(length - 0.075, 0.0, 0.0)),
            material=link_mat,
            name="fork_bridge",
        )
        for y in (-0.052, 0.052):
            part.visual(
                Box((0.125, 0.018, 0.064)),
                origin=Origin(xyz=(length - 0.0225, y, 0.0)),
                material=link_mat,
                name=f"fork_ear_{'neg' if y < 0 else 'pos'}",
            )

    base = model.part("base_plate")
    base.visual(
        Box((0.012, 0.36, 0.46)),
        origin=Origin(xyz=(-0.104, 0.0, 0.0)),
        material=wall,
        name="wall_patch",
    )
    base.visual(
        Box((0.028, 0.20, 0.30)),
        origin=Origin(xyz=(-0.084, 0.0, 0.0)),
        material=base_mat,
        name="mounting_plate",
    )
    for y in (-0.070, 0.070):
        for z in (-0.105, 0.105):
            base.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(-0.067, y, z), rpy=screw_rpy),
                material=pin_mat,
                name=f"screw_{'low' if z < 0 else 'high'}_{'neg' if y < 0 else 'pos'}",
            )
    for y in (-0.052, 0.052):
        base.visual(
            Box((0.090, 0.018, 0.070)),
            origin=Origin(xyz=(-0.027, y, 0.0)),
            material=base_mat,
            name=f"base_yoke_{'neg' if y < 0 else 'pos'}",
        )
    base.visual(
        Cylinder(radius=0.012, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=hinge_rpy),
        material=pin_mat,
        name="shoulder_pin",
    )
    add_pin_caps(base, x=0.0, prefix="shoulder_pin")

    shoulder_link = model.part("shoulder_link")
    add_reach_link(shoulder_link, length=0.440, bar_height=0.034)
    shoulder_link.visual(
        Cylinder(radius=0.011, length=0.145),
        origin=Origin(xyz=(0.440, 0.0, 0.0), rpy=hinge_rpy),
        material=pin_mat,
        name="elbow_pin",
    )
    add_pin_caps(shoulder_link, x=0.440, prefix="elbow_pin")

    elbow_link = model.part("elbow_link")
    add_reach_link(elbow_link, length=0.290, bar_height=0.030)
    elbow_link.visual(
        Cylinder(radius=0.011, length=0.145),
        origin=Origin(xyz=(0.290, 0.0, 0.0), rpy=hinge_rpy),
        material=pin_mat,
        name="wrist_pin",
    )
    add_pin_caps(elbow_link, x=0.290, prefix="wrist_pin")

    wrist_link = model.part("wrist_link")
    wrist_length = 0.170
    wrist_link.visual(
        Cylinder(radius=0.027, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=hinge_rpy),
        material=link_mat,
        name="proximal_barrel",
    )
    wrist_link.visual(
        Box((wrist_length - 0.033, 0.032, 0.028)),
        origin=Origin(xyz=(wrist_length / 2.0 + 0.0015, 0.0, 0.0)),
        material=link_mat,
        name="main_spine",
    )
    wrist_link.visual(
        Box((0.040, 0.036, 0.036)),
        origin=Origin(xyz=(wrist_length - 0.005, 0.0, 0.0)),
        material=link_mat,
        name="pad_neck",
    )
    wrist_link.visual(
        Box((0.024, 0.090, 0.090)),
        origin=Origin(xyz=(wrist_length + 0.012, 0.0, 0.0)),
        material=rubber,
        name="end_pad",
    )

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.85, upper=0.95),
    )
    model.articulation(
        "shoulder_to_elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=elbow_link,
        origin=Origin(xyz=(0.440, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "elbow_to_wrist",
        ArticulationType.REVOLUTE,
        parent=elbow_link,
        child=wrist_link,
        origin=Origin(xyz=(0.290, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-1.25, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_plate")
    shoulder = object_model.get_part("shoulder_link")
    elbow = object_model.get_part("elbow_link")
    wrist = object_model.get_part("wrist_link")
    shoulder_joint = object_model.get_articulation("base_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_elbow")
    wrist_joint = object_model.get_articulation("elbow_to_wrist")

    ctx.check(
        "three revolute inspection arm joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "parallel hinge axes",
        all(tuple(j.axis) == (0.0, 1.0, 0.0) for j in (shoulder_joint, elbow_joint, wrist_joint)),
        details=f"axes={[j.axis for j in (shoulder_joint, elbow_joint, wrist_joint)]}",
    )

    ctx.allow_overlap(
        base,
        shoulder,
        elem_a="shoulder_pin",
        elem_b="proximal_barrel",
        reason="The base pin is intentionally captured through the shoulder barrel bore.",
    )
    ctx.allow_overlap(
        shoulder,
        elbow,
        elem_a="elbow_pin",
        elem_b="proximal_barrel",
        reason="The elbow pin is intentionally captured through the elbow barrel bore.",
    )
    ctx.allow_overlap(
        elbow,
        wrist,
        elem_a="wrist_pin",
        elem_b="proximal_barrel",
        reason="The wrist pin is intentionally captured through the wrist barrel bore.",
    )

    ctx.expect_within(
        base,
        shoulder,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="proximal_barrel",
        margin=0.001,
        name="shoulder pin centered in shoulder barrel",
    )
    ctx.expect_overlap(
        base,
        shoulder,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="proximal_barrel",
        min_overlap=0.045,
        name="shoulder pin spans the barrel",
    )
    ctx.expect_within(
        shoulder,
        elbow,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="proximal_barrel",
        margin=0.001,
        name="elbow pin centered in elbow barrel",
    )
    ctx.expect_overlap(
        shoulder,
        elbow,
        axes="y",
        elem_a="elbow_pin",
        elem_b="proximal_barrel",
        min_overlap=0.045,
        name="elbow pin spans the barrel",
    )
    ctx.expect_within(
        elbow,
        wrist,
        axes="xz",
        inner_elem="wrist_pin",
        outer_elem="proximal_barrel",
        margin=0.001,
        name="wrist pin centered in wrist barrel",
    )
    ctx.expect_overlap(
        elbow,
        wrist,
        axes="y",
        elem_a="wrist_pin",
        elem_b="proximal_barrel",
        min_overlap=0.042,
        name="wrist pin spans the barrel",
    )

    ctx.expect_origin_gap(
        elbow,
        shoulder,
        axis="x",
        min_gap=0.420,
        max_gap=0.460,
        name="long shoulder link length",
    )
    ctx.expect_origin_gap(
        wrist,
        elbow,
        axis="x",
        min_gap=0.270,
        max_gap=0.310,
        name="shorter elbow link length",
    )

    with ctx.pose({shoulder_joint: 0.35, elbow_joint: -0.55, wrist_joint: 0.45}):
        base_pos = ctx.part_world_position(base)
        pad_aabb = ctx.part_element_world_aabb(wrist, elem="end_pad")
        ctx.check(
            "posed arm carries pad away from wall",
            base_pos is not None and pad_aabb is not None and pad_aabb[1][0] > base_pos[0] + 0.65,
            details=f"base={base_pos}, pad_aabb={pad_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
