from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_fold_out_support_arm")

    dark_powder = model.material("dark_powder_coat", rgba=(0.04, 0.045, 0.05, 1.0))
    link_finish = model.material("brushed_aluminum", rgba=(0.62, 0.66, 0.68, 1.0))
    hinge_finish = model.material("black_oxide_pins", rgba=(0.015, 0.016, 0.018, 1.0))
    bracket_finish = model.material("satin_steel_bracket", rgba=(0.50, 0.54, 0.56, 1.0))
    screw_finish = model.material("dark_screw_heads", rgba=(0.10, 0.10, 0.11, 1.0))

    link_length = 0.180
    hinge_radius = 0.018
    link_width = 0.030
    link_thickness = 0.026
    barrel_width = 0.110

    # Fixed vertical mounting plate.  Its front face is +X; the arm moves in
    # the X-Z plane about horizontal, mutually parallel Y axes.
    base = model.part("base_plate")
    base.visual(
        Box((0.026, 0.180, 0.230)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=dark_powder,
        name="wall_plate",
    )
    base.visual(
        Box((0.028, 0.120, 0.070)),
        origin=Origin(xyz=(0.0130, 0.0, 0.060)),
        material=dark_powder,
        name="hinge_stand",
    )
    base.visual(
        Box((0.035, 0.135, 0.010)),
        origin=Origin(xyz=(0.009, 0.0, 0.060)),
        material=bracket_finish,
        name="reinforcing_plate",
    )
    for idx, (y, z) in enumerate(
        ((-0.060, 0.042), (0.060, 0.042), (-0.060, 0.188), (0.060, 0.188))
    ):
        base.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(0.0145, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw_finish,
            name=f"screw_head_{idx}",
        )

    def add_boxed_link(part_name: str, *, body_y: float, final: bool = False):
        link = model.part(part_name)
        # A round hinge barrel at the proximal joint with a short boxed arm
        # extending from it.  The box-section proportions make each segment read
        # as a compact support-arm link rather than a flat strap.
        link.visual(
            Cylinder(radius=hinge_radius, length=barrel_width),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_finish,
            name="hinge_barrel",
        )
        link.visual(
            Box((link_length - hinge_radius, link_width, link_thickness)),
            origin=Origin(xyz=((link_length - hinge_radius) / 2.0, body_y, 0.0)),
            material=link_finish,
            name="link_body",
        )
        link.visual(
            Box((link_length - 0.045, 0.004, 0.030)),
            origin=Origin(xyz=(link_length / 2.0, body_y - link_width / 2.0 - 0.002, 0.0)),
            material=hinge_finish,
            name="lower_edge_strip",
        )
        link.visual(
            Box((link_length - 0.045, 0.004, 0.030)),
            origin=Origin(xyz=(link_length / 2.0, body_y + link_width / 2.0 + 0.002, 0.0)),
            material=hinge_finish,
            name="upper_edge_strip",
        )
        if final:
            # A small end platform bracket fixed to the last segment: a neck,
            # a flat shelf, and two raised side ears for whatever load the arm
            # supports.
            link.visual(
                Box((0.060, 0.032, 0.020)),
                origin=Origin(xyz=(link_length + 0.005, body_y, 0.0)),
                material=bracket_finish,
                name="platform_neck",
            )
            link.visual(
                Box((0.095, 0.080, 0.010)),
                origin=Origin(xyz=(link_length + 0.055, body_y, 0.015)),
                material=bracket_finish,
                name="platform_plate",
            )
            link.visual(
                Box((0.065, 0.007, 0.030)),
                origin=Origin(xyz=(link_length + 0.055, body_y - 0.0415, 0.030)),
                material=bracket_finish,
                name="platform_ear_0",
            )
            link.visual(
                Box((0.065, 0.007, 0.030)),
                origin=Origin(xyz=(link_length + 0.055, body_y + 0.0415, 0.030)),
                material=bracket_finish,
                name="platform_ear_1",
            )
        return link

    link_0 = add_boxed_link("link_0", body_y=-0.033)
    link_1 = add_boxed_link("link_1", body_y=0.033)
    link_2 = add_boxed_link("link_2", body_y=-0.033)
    link_3 = add_boxed_link("link_3", body_y=0.033, final=True)

    # Folded rest angles in the X-Z plane: +75, -75, +75, -75 degrees.  Opening
    # poses can cancel these relative rotations to make a long, nearly straight
    # four-segment arm.
    base_pitch = math.radians(-75.0)
    fold_back = math.radians(150.0)
    fold_forward = math.radians(-150.0)
    open_75 = math.radians(75.0)
    open_150 = math.radians(150.0)

    joint_limits = MotionLimits(effort=18.0, velocity=2.0, lower=-open_150, upper=open_150)
    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.045, 0.0, 0.060), rpy=(0.0, base_pitch, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=open_75),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(link_length, 0.0, 0.0), rpy=(0.0, fold_back, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(link_length, 0.0, 0.0), rpy=(0.0, fold_forward, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(link_length, 0.0, 0.0), rpy=(0.0, fold_back, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [
        object_model.get_articulation("base_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_link_3"),
    ]
    ctx.check(
        "four revolute fold-out joints",
        len(joints) == 4 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "joint axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    base = object_model.get_part("base_plate")
    end = object_model.get_part("link_3")
    rest_base = ctx.part_world_aabb(base)
    rest_end = ctx.part_world_aabb(end)
    if rest_base is not None and rest_end is not None:
        rest_reach = rest_end[1][0] - rest_base[0][0]
    else:
        rest_reach = None

    with ctx.pose(
        {
            joints[0]: math.radians(75.0),
            joints[1]: math.radians(-150.0),
            joints[2]: math.radians(150.0),
            joints[3]: math.radians(-150.0),
        }
    ):
        open_base = ctx.part_world_aabb(base)
        open_end = ctx.part_world_aabb(end)
        if open_base is not None and open_end is not None:
            open_reach = open_end[1][0] - open_base[0][0]
        else:
            open_reach = None
        ctx.check(
            "unfolded arm reaches well beyond folded package",
            rest_reach is not None
            and open_reach is not None
            and rest_reach < 0.34
            and open_reach > 0.70,
            details=f"rest_reach={rest_reach}, open_reach={open_reach}",
        )

    return ctx.report()


object_model = build_object_model()
