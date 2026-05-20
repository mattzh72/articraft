from __future__ import annotations

from math import pi

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


AXIS_Y_CYL = Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=AXIS_Y_CYL.rpy)


def _add_segment(part, *, length: float, has_distal_barrel: bool) -> None:
    """Add one rigid robotic-finger phalanx in a hinge-centered local frame."""
    link_blue = "powder_blue"
    hinge_dark = "dark_bushing"

    # Broad, straight link plate.  Its local origin is the proximal hinge axis.
    plate_start = 0.060
    plate_end = length - (0.050 if has_distal_barrel else 0.025)
    part.visual(
        Box((plate_end - plate_start, 0.120, 0.034)),
        origin=Origin(xyz=((plate_start + plate_end) / 2.0, 0.0, 0.0)),
        material=link_blue,
        name="link_plate",
    )
    part.visual(
        Box((0.075, 0.026, 0.045)),
        origin=Origin(xyz=(0.0525, 0.071, 0.0)),
        material=link_blue,
        name="fork_cheek_0",
    )
    part.visual(
        Box((0.075, 0.026, 0.045)),
        origin=Origin(xyz=(0.0525, -0.071, 0.0)),
        material=link_blue,
        name="fork_cheek_1",
    )

    # Outer fork barrels straddle the parent barrel and make the hinge overlap
    # visually generous while remaining physically clear in the Y direction.
    part.visual(
        Cylinder(radius=0.036, length=0.045),
        origin=_y_cylinder_origin(0.0, 0.073, 0.0),
        material=hinge_dark,
        name="proximal_barrel_0",
    )
    part.visual(
        Cylinder(radius=0.036, length=0.045),
        origin=_y_cylinder_origin(0.0, -0.073, 0.0),
        material=hinge_dark,
        name="proximal_barrel_1",
    )

    if has_distal_barrel:
        part.visual(
            Box((0.090, 0.065, 0.040)),
            origin=Origin(xyz=(length - 0.023, 0.0, 0.0)),
            material=link_blue,
            name="distal_neck",
        )
        part.visual(
            Cylinder(radius=0.038, length=0.055),
            origin=_y_cylinder_origin(length, 0.0, 0.0),
            material=hinge_dark,
            name="distal_barrel",
        )
        part.visual(
            Cylinder(radius=0.014, length=0.205),
            origin=_y_cylinder_origin(length, 0.0, 0.0),
            material="brushed_steel",
            name="distal_pin",
        )
    else:
        part.visual(
            Cylinder(radius=0.035, length=0.105),
            origin=_y_cylinder_origin(length - 0.010, 0.0, 0.0),
            material=link_blue,
            name="rounded_tip",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_backed_finger_chain")

    model.material("powder_blue", rgba=(0.22, 0.48, 0.78, 1.0))
    model.material("dark_bushing", rgba=(0.03, 0.035, 0.045, 1.0))
    model.material("spine_graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))

    rear_spine = model.part("rear_spine")
    # Ground plate and ladder-like rear spine: two rails with cross rungs.
    rear_spine.visual(
        Box((0.610, 0.210, 0.020)),
        origin=Origin(xyz=(0.230, 0.0, 0.010)),
        material="spine_graphite",
        name="ground_plate",
    )
    for i, y in enumerate((-0.085, 0.085)):
        rear_spine.visual(
            Box((0.565, 0.022, 0.025)),
            origin=Origin(xyz=(0.220, y, 0.034)),
            material="brushed_steel",
            name=f"ladder_rail_{i}",
        )
    for i, x in enumerate((-0.030, 0.110, 0.240, 0.385, 0.500)):
        rear_spine.visual(
            Box((0.030, 0.190, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.034)),
            material="brushed_steel",
            name=f"ladder_rung_{i}",
        )

    # The grounded upright carries the first hinge barrel.
    rear_spine.visual(
        Box((0.050, 0.050, 0.135)),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material="spine_graphite",
        name="root_upright",
    )
    rear_spine.visual(
        Cylinder(radius=0.038, length=0.055),
        origin=_y_cylinder_origin(0.0, 0.0, 0.180),
        material="dark_bushing",
        name="root_barrel",
    )
    rear_spine.visual(
        Cylinder(radius=0.014, length=0.205),
        origin=_y_cylinder_origin(0.0, 0.0, 0.180),
        material="brushed_steel",
        name="root_pin",
    )

    phalanx_0 = model.part("phalanx_0")
    phalanx_1 = model.part("phalanx_1")
    phalanx_2 = model.part("phalanx_2")

    length_0 = 0.220
    length_1 = 0.180
    length_2 = 0.145
    _add_segment(phalanx_0, length=length_0, has_distal_barrel=True)
    _add_segment(phalanx_1, length=length_1, has_distal_barrel=True)
    _add_segment(phalanx_2, length=length_2, has_distal_barrel=False)

    hinge_limits = MotionLimits(lower=0.0, upper=1.05, effort=7.0, velocity=2.0)
    model.articulation(
        "spine_to_phalanx_0",
        ArticulationType.REVOLUTE,
        parent=rear_spine,
        child=phalanx_0,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "phalanx_0_to_phalanx_1",
        ArticulationType.REVOLUTE,
        parent=phalanx_0,
        child=phalanx_1,
        origin=Origin(xyz=(length_0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "phalanx_1_to_phalanx_2",
        ArticulationType.REVOLUTE,
        parent=phalanx_1,
        child=phalanx_2,
        origin=Origin(xyz=(length_1, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=hinge_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("spine_to_phalanx_0"),
        object_model.get_articulation("phalanx_0_to_phalanx_1"),
        object_model.get_articulation("phalanx_1_to_phalanx_2"),
    ]
    ctx.check(
        "three parallel revolute joints",
        len(joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(tuple(j.axis) == (0.0, -1.0, 0.0) for j in joints),
        details=f"joints={joints}",
    )

    rear_spine = object_model.get_part("rear_spine")
    phalanx_0 = object_model.get_part("phalanx_0")
    phalanx_1 = object_model.get_part("phalanx_1")
    phalanx_2 = object_model.get_part("phalanx_2")

    captured_pin_pairs = (
        (rear_spine, phalanx_0, "root_pin", "proximal_barrel_0", "root upper fork"),
        (rear_spine, phalanx_0, "root_pin", "proximal_barrel_1", "root lower fork"),
        (phalanx_0, phalanx_1, "distal_pin", "proximal_barrel_0", "middle upper fork"),
        (phalanx_0, phalanx_1, "distal_pin", "proximal_barrel_1", "middle lower fork"),
        (phalanx_1, phalanx_2, "distal_pin", "proximal_barrel_0", "tip upper fork"),
        (phalanx_1, phalanx_2, "distal_pin", "proximal_barrel_1", "tip lower fork"),
    )
    for parent, child, pin_elem, barrel_elem, label in captured_pin_pairs:
        ctx.allow_overlap(
            parent,
            child,
            elem_a=pin_elem,
            elem_b=barrel_elem,
            reason="A steel hinge pin is intentionally captured inside the fork bushing.",
        )
        ctx.expect_within(
            parent,
            child,
            axes="xz",
            inner_elem=pin_elem,
            outer_elem=barrel_elem,
            margin=0.002,
            name=f"{label} pin stays centered in bushing",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a=pin_elem,
            elem_b=barrel_elem,
            min_overlap=0.040,
            name=f"{label} pin passes through bushing width",
        )

    with ctx.pose({joints[0]: 0.0, joints[1]: 0.0, joints[2]: 0.0}):
        ctx.expect_overlap(
            rear_spine,
            phalanx_0,
            axes="xz",
            elem_a="root_barrel",
            elem_b="proximal_barrel_0",
            min_overlap=0.045,
            name="root hinge barrels overlap in projection",
        )
        ctx.expect_overlap(
            phalanx_0,
            phalanx_1,
            axes="xz",
            elem_a="distal_barrel",
            elem_b="proximal_barrel_0",
            min_overlap=0.045,
            name="middle hinge barrels overlap in projection",
        )
        ctx.expect_overlap(
            phalanx_1,
            phalanx_2,
            axes="xz",
            elem_a="distal_barrel",
            elem_b="proximal_barrel_0",
            min_overlap=0.045,
            name="tip hinge barrels overlap in projection",
        )
        ctx.expect_gap(
            phalanx_0,
            rear_spine,
            axis="y",
            positive_elem="proximal_barrel_0",
            negative_elem="root_barrel",
            min_gap=0.015,
            max_gap=0.060,
            name="root hinge fork clears center barrel",
        )
        rest_tip_aabb = ctx.part_world_aabb(phalanx_2)

    with ctx.pose({joints[0]: 0.35, joints[1]: 0.55, joints[2]: 0.70}):
        flexed_tip_aabb = ctx.part_world_aabb(phalanx_2)

    ctx.check(
        "finger chain flexes upward from spine",
        rest_tip_aabb is not None
        and flexed_tip_aabb is not None
        and flexed_tip_aabb[1][2] > rest_tip_aabb[1][2] + 0.070,
        details=f"rest_tip_aabb={rest_tip_aabb}, flexed_tip_aabb={flexed_tip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
