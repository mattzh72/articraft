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


HINGE_X = 0.055
HINGE_Z = 0.085
FINGER_Y = 0.050


def _add_base_clevis(base, y0: float, steel: Material, dark: Material) -> None:
    """Add the fixed side plates that independently carry one first finger joint."""
    base.visual(
        Box((0.062, 0.070, 0.008)),
        origin=Origin(xyz=(HINGE_X, y0, 0.062)),
        material=steel,
        name=f"mount_pad_{'p' if y0 > 0 else 'n'}",
    )
    for side, y_offset in (("inner", -0.020), ("outer", 0.020)):
        y = y0 + y_offset
        base.visual(
            Box((0.050, 0.010, 0.054)),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z)),
            material=steel,
            name=f"base_plate_{'p' if y0 > 0 else 'n'}_{side}",
        )
        base.visual(
            Cylinder(radius=0.0085, length=0.058),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z)),
            material=dark,
            name=f"base_pin_cap_{'p' if y0 > 0 else 'n'}_{side}",
        )


def _add_finger_link(
    link,
    length: float,
    *,
    body_mat: Material,
    plate_mat: Material,
    rubber_mat: Material,
    distal_fork: bool,
) -> None:
    """A slim planar-link with a center tongue at its proximal hinge.

    The first two links end in a fork: paired side plates leave a clear gap for
    the next link's tongue.  The last link instead carries a service-tool pad.
    """
    link.visual(
        Box((0.040, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=body_mat,
        name="proximal_tongue",
    )
    link.visual(
        Cylinder(radius=0.0115, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=plate_mat,
        name="proximal_bushing",
    )

    if distal_fork:
        beam_end = length - 0.053
    else:
        beam_end = length - 0.020
    beam_start = 0.018
    beam_len = beam_end - beam_start
    link.visual(
        Box((beam_len, 0.018, 0.018)),
        origin=Origin(xyz=(beam_start + beam_len / 2.0, 0.0, 0.0)),
        material=body_mat,
        name="center_web",
    )

    if distal_fork:
        link.visual(
            Box((0.018, 0.048, 0.024)),
            origin=Origin(xyz=(length - 0.048, 0.0, 0.0)),
            material=body_mat,
            name="distal_bridge",
        )
        for side, y in (("side_0", -0.020), ("side_1", 0.020)):
            link.visual(
                Box((0.074, 0.008, 0.032)),
                origin=Origin(xyz=(length - 0.010, y, 0.0)),
                material=plate_mat,
                name=f"distal_plate_{side}",
            )
            link.visual(
                Cylinder(radius=0.0085, length=0.034),
                origin=Origin(xyz=(length, y, 0.0)),
                material=plate_mat,
                name=f"distal_boss_{side}",
            )
    else:
        link.visual(
            Box((0.052, 0.030, 0.022)),
            origin=Origin(xyz=(length - 0.004, 0.0, 0.0)),
            material=body_mat,
            name="tip_nose",
        )
        link.visual(
            Box((0.020, 0.038, 0.026)),
            origin=Origin(xyz=(length + 0.022, 0.0, 0.0)),
            material=rubber_mat,
            name="rubber_pad",
        )
        link.visual(
            Cylinder(radius=0.014, length=0.040),
            origin=Origin(xyz=(length + 0.031, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber_mat,
            name="rounded_tip",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_tool_finger_pair")

    dark = model.material("blackened_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    link_blue = model.material("anodized_blue", rgba=(0.05, 0.18, 0.36, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.013, 1.0))
    bolt = model.material("dark_bolt_heads", rgba=(0.02, 0.022, 0.025, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.170, 0.180, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark,
        name="dense_body",
    )
    base.visual(
        Box((0.138, 0.148, 0.010)),
        origin=Origin(xyz=(0.004, 0.0, 0.063)),
        material=steel,
        name="top_plate",
    )
    for x in (-0.048, 0.050):
        for y in (-0.064, 0.064):
            base.visual(
                Cylinder(radius=0.006, length=0.007),
                origin=Origin(xyz=(x, y, 0.066)),
                material=bolt,
                name=f"top_bolt_{x:.3f}_{y:.3f}",
            )
    _add_base_clevis(base, FINGER_Y, steel, bolt)
    _add_base_clevis(base, -FINGER_Y, steel, bolt)

    link_lengths = (0.150, 0.132, 0.112)
    fingers = []
    for finger_index, y0 in enumerate((FINGER_Y, -FINGER_Y)):
        chain = []
        for link_index, length in enumerate(link_lengths):
            link = model.part(f"finger_{finger_index}_link_{link_index}")
            _add_finger_link(
                link,
                length,
                body_mat=link_blue,
                plate_mat=steel,
                rubber_mat=rubber,
                distal_fork=link_index < 2,
            )
            chain.append(link)
        fingers.append(chain)

        model.articulation(
            f"finger_{finger_index}_joint_0",
            ArticulationType.REVOLUTE,
            parent=base,
            child=chain[0],
            origin=Origin(xyz=(HINGE_X, y0, HINGE_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.70, upper=0.70),
        )
        model.articulation(
            f"finger_{finger_index}_joint_1",
            ArticulationType.REVOLUTE,
            parent=chain[0],
            child=chain[1],
            origin=Origin(xyz=(link_lengths[0], 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.8, lower=-0.95, upper=0.95),
        )
        model.articulation(
            f"finger_{finger_index}_joint_2",
            ArticulationType.REVOLUTE,
            parent=chain[1],
            child=chain[2],
            origin=Origin(xyz=(link_lengths[1], 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-1.05, upper=0.85),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [object_model.get_articulation(f"finger_{i}_joint_{j}") for i in range(2) for j in range(3)]
    ctx.check(
        "two independent three-joint fingers",
        len(joints) == 6 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={joints}",
    )

    base = object_model.get_part("base")
    for i in range(2):
        link_0 = object_model.get_part(f"finger_{i}_link_0")
        link_1 = object_model.get_part(f"finger_{i}_link_1")
        link_2 = object_model.get_part(f"finger_{i}_link_2")
        ctx.expect_overlap(
            link_0,
            base,
            axes="xy",
            min_overlap=0.020,
            name=f"finger {i} has a base-mounted first hinge",
        )
        ctx.expect_overlap(
            link_0,
            link_1,
            axes="xy",
            min_overlap=0.018,
            name=f"finger {i} first and second links share a hinge footprint",
        )
        ctx.expect_overlap(
            link_1,
            link_2,
            axes="xy",
            min_overlap=0.018,
            name=f"finger {i} second and third links share a hinge footprint",
        )

    joint = object_model.get_articulation("finger_0_joint_0")
    distal = object_model.get_part("finger_0_link_1")
    rest_pos = ctx.part_world_position(distal)
    with ctx.pose({joint: 0.45}):
        swept_pos = ctx.part_world_position(distal)
    ctx.check(
        "first finger shoulder joint sweeps the chain",
        rest_pos is not None and swept_pos is not None and swept_pos[1] > rest_pos[1] + 0.020,
        details=f"rest={rest_pos}, swept={swept_pos}",
    )

    return ctx.report()


object_model = build_object_model()
