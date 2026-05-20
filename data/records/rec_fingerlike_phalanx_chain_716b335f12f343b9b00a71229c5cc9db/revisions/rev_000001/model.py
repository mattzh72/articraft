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


LINK_LENGTHS = (0.105, 0.088, 0.070)
OUTER_KNUCKLE_Y = 0.015


bracket_mat = Material("mat_dark_anodized_bracket", color=(0.10, 0.11, 0.12, 1.0))
link_mat = Material("mat_satin_aluminum_links", color=(0.68, 0.72, 0.72, 1.0))
barrel_mat = Material("mat_blued_steel_barrels", color=(0.20, 0.22, 0.24, 1.0))
screw_mat = Material("mat_black_screw_heads", color=(0.02, 0.02, 0.018, 1.0))
tip_mat = Material("mat_flat_rubber_tip", color=(0.025, 0.025, 0.022, 1.0))


def _y_barrel(radius: float, length: float) -> Cylinder:
    """Cylinder descriptor for a hinge barrel; visual origins rotate it onto Y."""

    return Cylinder(radius=radius, length=length)


def _barrel_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_clevis_link(part, length: float, *, has_tip: bool = False) -> None:
    """Add one narrow phalanx with a central proximal knuckle and distal features."""

    # The child frame and proximal hinge axis are at x=0.  A short central
    # neck clears the parent's outer clevis ears before the main link widens.
    part.visual(
        _y_barrel(0.010, 0.020),
        origin=_barrel_origin(0.0),
        material=barrel_mat,
        name="proximal_barrel",
    )
    part.visual(
        Box((0.032, 0.018, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=link_mat,
        name="proximal_neck",
    )

    if has_tip:
        beam_end = length - 0.022
        part.visual(
            Box((beam_end - 0.030, 0.024, 0.010)),
            origin=Origin(xyz=((0.030 + beam_end) / 2.0, 0.0, 0.0)),
            material=link_mat,
            name="slim_web",
        )
        part.visual(
            Box((0.030, 0.030, 0.010)),
            origin=Origin(xyz=(length - 0.015, 0.0, 0.0)),
            material=link_mat,
            name="tip_carrier",
        )
        part.visual(
            Box((0.006, 0.034, 0.014)),
            origin=Origin(xyz=(length + 0.003, 0.0, 0.0)),
            material=tip_mat,
            name="flat_tip",
        )
    else:
        part.visual(
            Box((length - 0.064, 0.028, 0.010)),
            origin=Origin(xyz=((0.034 + length - 0.030) / 2.0, 0.0, 0.0)),
            material=link_mat,
            name="slim_web",
        )
        # Distal fork ears and outer barrel segments leave a clear center slot
        # for the next phalanx's proximal barrel.
        for side, y in (("upper", OUTER_KNUCKLE_Y), ("lower", -OUTER_KNUCKLE_Y)):
            part.visual(
                Box((0.046, 0.010, 0.010)),
                origin=Origin(xyz=(length - 0.020, y, 0.0)),
                material=link_mat,
                name=f"distal_fork_{side}",
            )
            part.visual(
                _y_barrel(0.010, 0.010),
                origin=_barrel_origin(length, y, 0.0),
                material=barrel_mat,
                name=f"distal_barrel_{side}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_service_finger_module")

    root = model.part("root_bracket")
    root.visual(
        Box((0.060, 0.060, 0.026)),
        origin=Origin(xyz=(-0.050, 0.0, -0.012)),
        material=bracket_mat,
        name="mounting_block",
    )
    for side, y in (("upper", OUTER_KNUCKLE_Y), ("lower", -OUTER_KNUCKLE_Y)):
        root.visual(
            Box((0.052, 0.010, 0.034)),
            origin=Origin(xyz=(-0.014, y, -0.006)),
            material=bracket_mat,
            name=f"root_fork_{side}",
        )
        root.visual(
            _y_barrel(0.010, 0.010),
            origin=_barrel_origin(0.0, y, 0.0),
            material=barrel_mat,
            name=f"root_barrel_{side}",
        )
    for side, y in (("upper", 0.018), ("lower", -0.018)):
        root.visual(
            Cylinder(radius=0.0045, length=0.003),
            origin=Origin(xyz=(-0.055, y, 0.002), rpy=(0.0, 0.0, 0.0)),
            material=screw_mat,
            name=f"mount_screw_{side}",
        )

    phalanx_0 = model.part("phalanx_0")
    _add_clevis_link(phalanx_0, LINK_LENGTHS[0])

    phalanx_1 = model.part("phalanx_1")
    _add_clevis_link(phalanx_1, LINK_LENGTHS[1])

    phalanx_2 = model.part("phalanx_2")
    _add_clevis_link(phalanx_2, LINK_LENGTHS[2], has_tip=True)

    joint_limits = MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.25)
    model.articulation(
        "root_to_phalanx_0",
        ArticulationType.REVOLUTE,
        parent=root,
        child=phalanx_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "phalanx_0_to_phalanx_1",
        ArticulationType.REVOLUTE,
        parent=phalanx_0,
        child=phalanx_1,
        origin=Origin(xyz=(LINK_LENGTHS[0], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "phalanx_1_to_phalanx_2",
        ArticulationType.REVOLUTE,
        parent=phalanx_1,
        child=phalanx_2,
        origin=Origin(xyz=(LINK_LENGTHS[1], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("root_to_phalanx_0"),
        object_model.get_articulation("phalanx_0_to_phalanx_1"),
        object_model.get_articulation("phalanx_1_to_phalanx_2"),
    ]
    phalanx_2 = object_model.get_part("phalanx_2")

    ctx.check(
        "three serial revolute joints",
        len(joints) == 3
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in joints),
        details=f"joints={[joint.name for joint in joints]}",
    )
    ctx.check(
        "hinge axes share one bending plane",
        all(tuple(round(v, 6) for v in joint.axis) == (0.0, -1.0, 0.0) for joint in joints),
        details=f"axes={[joint.axis for joint in joints]}",
    )

    rest_tip = ctx.part_element_world_aabb(phalanx_2, elem="flat_tip")
    rest_origin = ctx.part_world_position(phalanx_2)
    with ctx.pose({joint: 0.75 for joint in joints}):
        bent_tip = ctx.part_element_world_aabb(phalanx_2, elem="flat_tip")
        bent_origin = ctx.part_world_position(phalanx_2)

    ctx.check(
        "serial finger bends upward in the xz plane",
        rest_tip is not None
        and bent_tip is not None
        and rest_origin is not None
        and bent_origin is not None
        and bent_tip[0][2] > rest_tip[0][2] + 0.055
        and abs(bent_origin[1] - rest_origin[1]) < 1e-6,
        details=f"rest_tip={rest_tip}, bent_tip={bent_tip}, rest_origin={rest_origin}, bent_origin={bent_origin}",
    )

    return ctx.report()


object_model = build_object_model()
