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


PIVOT_Z = 0.14
LINK_0_LENGTH = 0.36
LINK_1_LENGTH = 0.32
END_LINK_LENGTH = 0.25


def _y_cylinder() -> Origin:
    """Cylinder origin with its local Z axis turned onto the hinge Y axis."""

    return Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_deep_fork_link(part, *, length: float, material, boss_material) -> None:
    """Add one boxed lever link with a center tongue and forked distal boss."""

    # The main rectangular section is deliberately deep in Z and relatively
    # narrow in Y, like a fabricated industrial box link.
    body_start = 0.050
    body_end = length - 0.080
    body_length = body_end - body_start
    part.visual(
        Box((body_length, 0.052, 0.080)),
        origin=Origin(xyz=(body_start + body_length / 2.0, 0.0, 0.0)),
        material=material,
        name="box_section",
    )
    flange_length = body_length - 0.070
    part.visual(
        Box((flange_length, 0.060, 0.014)),
        origin=Origin(xyz=(body_start + 0.040 + flange_length / 2.0, 0.0, 0.045)),
        material=material,
        name="top_flange",
    )
    part.visual(
        Box((flange_length, 0.060, 0.014)),
        origin=Origin(xyz=(body_start + 0.040 + flange_length / 2.0, 0.0, -0.045)),
        material=material,
        name="bottom_flange",
    )

    # Center tongue boss at the parent joint.
    part.visual(
        Cylinder(radius=0.060, length=0.038),
        origin=_y_cylinder(),
        material=boss_material,
        name="proximal_boss",
    )

    # The distal end is a fork: two outside cheeks and visible boss pads leave a
    # center slot for the next link's tongue.
    shoulder_x = length - 0.095
    part.visual(
        Box((0.034, 0.088, 0.060)),
        origin=Origin(xyz=(shoulder_x, 0.0, 0.0)),
        material=material,
        name="fork_bridge",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.126),
        origin=Origin(xyz=(length, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=boss_material,
        name="distal_pin",
    )
    for rail_name, boss_name, cap_name, y in (
        ("fork_rail_pos", "distal_boss_pos", "pin_cap_pos", 0.041),
        ("fork_rail_neg", "distal_boss_neg", "pin_cap_neg", -0.041),
    ):
        part.visual(
            Box((0.112, 0.022, 0.060)),
            origin=Origin(xyz=(length - 0.025, y, 0.0)),
            material=material,
            name=rail_name,
        )
        part.visual(
            Cylinder(radius=0.060, length=0.022),
            origin=Origin(xyz=(length, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=boss_material,
            name=boss_name,
        )
        cap_y = 0.056 if y > 0.0 else -0.056
        part.visual(
            Cylinder(radius=0.034, length=0.009),
            origin=Origin(xyz=(length, cap_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=boss_material,
            name=cap_name,
        )


def _add_end_link(part, *, length: float, material, boss_material, tab_material) -> None:
    """Add the final lever link with a small terminal tab."""

    body_start = 0.050
    body_end = length - 0.040
    body_length = body_end - body_start
    part.visual(
        Box((body_length, 0.048, 0.074)),
        origin=Origin(xyz=(body_start + body_length / 2.0, 0.0, 0.0)),
        material=material,
        name="box_section",
    )
    part.visual(
        Cylinder(radius=0.060, length=0.038),
        origin=_y_cylinder(),
        material=boss_material,
        name="proximal_boss",
    )
    part.visual(
        Box((0.112, 0.034, 0.044)),
        origin=Origin(xyz=(length + 0.014, 0.0, 0.0)),
        material=tab_material,
        name="end_tab",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.036),
        origin=Origin(xyz=(length + 0.062, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=boss_material,
        name="tab_boss",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.038),
        origin=Origin(xyz=(length + 0.062, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="blackened_pin",
        name="tab_hole_darkener",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_industrial_lever_chain")

    painted_steel = model.material("painted_steel", color=(0.18, 0.22, 0.24, 1.0))
    dark_steel = model.material("dark_steel", color=(0.07, 0.075, 0.07, 1.0))
    worn_edge = model.material("worn_edge", color=(0.48, 0.48, 0.43, 1.0))
    safety_orange = model.material("safety_orange", color=(0.95, 0.34, 0.08, 1.0))
    blackened_pin = model.material("blackened_pin", color=(0.005, 0.005, 0.004, 1.0))

    base = model.part("base_cheek")
    base.visual(
        Box((0.40, 0.21, 0.040)),
        origin=Origin(xyz=(-0.035, 0.0, 0.020)),
        material=painted_steel,
        name="base_plate",
    )
    for cheek_name, boss_name, rib_name, y in (
        ("cheek_pos", "outer_pin_boss_pos", "diagonal_rib_pos", 0.055),
        ("cheek_neg", "outer_pin_boss_neg", "diagonal_rib_neg", -0.055),
    ):
        base.visual(
            Box((0.145, 0.024, 0.182)),
            origin=Origin(xyz=(0.010, y, 0.118)),
            material=painted_steel,
            name=cheek_name,
        )
        base.visual(
            Cylinder(radius=0.043, length=0.012),
            origin=Origin(xyz=(0.0, 0.073 if y > 0.0 else -0.073, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=boss_name,
        )
        # Sloped cheek reinforcement ribs visibly tie each upright back into the
        # base plate. They slightly bury into both pieces so the base reads as
        # one welded bracket.
        base.visual(
            Box((0.170, 0.012, 0.020)),
            origin=Origin(xyz=(-0.010, 0.068 if y > 0.0 else -0.068, 0.080), rpy=(0.0, -0.72, 0.0)),
            material=worn_edge,
            name=rib_name,
        )

    base.visual(
        Cylinder(radius=0.018, length=0.142),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="base_pin",
    )

    for i, (x, y) in enumerate(((-0.175, -0.075), (-0.175, 0.075), (0.110, -0.075), (0.110, 0.075))):
        base.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(x, y, 0.041)),
            material=dark_steel,
            name=f"mount_bolt_{i}",
        )

    link_0 = model.part("link_0")
    _add_deep_fork_link(link_0, length=LINK_0_LENGTH, material=safety_orange, boss_material=dark_steel)

    link_1 = model.part("link_1")
    _add_deep_fork_link(link_1, length=LINK_1_LENGTH, material=safety_orange, boss_material=dark_steel)

    end_link = model.part("end_link")
    _add_end_link(end_link, length=END_LINK_LENGTH, material=safety_orange, boss_material=dark_steel, tab_material=worn_edge)

    limits = MotionLimits(effort=120.0, velocity=1.5, lower=-0.95, upper=1.15)
    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK_0_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "link_1_to_end_link",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=end_link,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_cheek")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    end_link = object_model.get_part("end_link")
    j0 = object_model.get_articulation("base_to_link_0")
    j1 = object_model.get_articulation("link_0_to_link_1")
    j2 = object_model.get_articulation("link_1_to_end_link")

    ctx.check(
        "three serial revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in (j0, j1, j2))
        and (j0.parent, j0.child, j1.parent, j1.child, j2.parent, j2.child)
        == ("base_cheek", "link_0", "link_0", "link_1", "link_1", "end_link"),
        details="The lever chain should be base -> link_0 -> link_1 -> end_link.",
    )
    ctx.check(
        "hinge axes are parallel",
        j0.axis == j1.axis == j2.axis == (0.0, -1.0, 0.0),
        details=f"axes: {j0.axis}, {j1.axis}, {j2.axis}",
    )

    ctx.allow_overlap(
        base,
        link_0,
        elem_a="base_pin",
        elem_b="proximal_boss",
        reason="The first hinge pin is intentionally captured through the link tongue boss.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="proximal_boss",
        reason="The second hinge pin is intentionally captured through the next link boss.",
    )
    ctx.allow_overlap(
        link_1,
        end_link,
        elem_a="distal_pin",
        elem_b="proximal_boss",
        reason="The final hinge pin is intentionally captured through the end-link boss.",
    )

    ctx.expect_overlap(
        link_0,
        base,
        axes="xz",
        elem_a="proximal_boss",
        elem_b="outer_pin_boss_pos",
        min_overlap=0.055,
        name="first boss is coaxial with base cheek pin",
    )
    ctx.expect_overlap(
        base,
        link_0,
        axes="xyz",
        elem_a="base_pin",
        elem_b="proximal_boss",
        min_overlap=0.030,
        name="base pin passes through first boss",
    )
    ctx.expect_gap(
        base,
        link_0,
        axis="y",
        positive_elem="cheek_pos",
        negative_elem="proximal_boss",
        min_gap=0.015,
        name="first tongue clears positive base cheek",
    )
    ctx.expect_gap(
        link_0,
        base,
        axis="y",
        positive_elem="proximal_boss",
        negative_elem="cheek_neg",
        min_gap=0.015,
        name="first tongue clears negative base cheek",
    )

    for parent, child, parent_name in (
        (link_0, link_1, "middle"),
        (link_1, end_link, "final"),
    ):
        ctx.expect_overlap(
            child,
            parent,
            axes="xz",
            elem_a="proximal_boss",
            elem_b="distal_boss_pos",
            min_overlap=0.055,
            name=f"{parent_name} hinge bosses are coaxial",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="xyz",
            elem_a="distal_pin",
            elem_b="proximal_boss",
            min_overlap=0.030,
            name=f"{parent_name} hinge pin passes through boss",
        )
        ctx.expect_gap(
            parent,
            child,
            axis="y",
            positive_elem="distal_boss_pos",
            negative_elem="proximal_boss",
            min_gap=0.008,
            name=f"{parent_name} hinge positive fork clearance",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="y",
            positive_elem="proximal_boss",
            negative_elem="distal_boss_neg",
            min_gap=0.008,
            name=f"{parent_name} hinge negative fork clearance",
        )

    rest_end = ctx.part_world_position(end_link)
    with ctx.pose({j0: 0.40, j1: 0.35, j2: 0.25}):
        raised_end = ctx.part_world_position(end_link)
    ctx.check(
        "joint chain lifts at positive pose",
        rest_end is not None and raised_end is not None and raised_end[2] > rest_end[2] + 0.10,
        details=f"rest={rest_end}, raised={raised_end}",
    )

    return ctx.report()


object_model = build_object_model()
