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


LINK_LEN = 0.280
JOINT_Z = 0.180
HINGE_AXIS = (0.0, -1.0, 0.0)


def _cyl_y(part, radius: float, length: float, xyz, material: Material, name: str) -> None:
    """Add a cylinder whose axis runs along local/world Y."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_side_plates(part, x: float, z: float, clear: Material, dark: Material, prefix: str) -> None:
    """Transparent fork plates and exterior screw washers around one hinge."""
    plate_size = (0.084, 0.008, 0.064)
    _cyl_y(part, 0.010, 0.062, (x, 0.0, z), dark, f"{prefix}_pin")
    part.visual(
        Box(plate_size),
        origin=Origin(xyz=(x, 0.025, z)),
        material=clear,
        name=f"{prefix}_plate_pos",
    )
    part.visual(
        Box(plate_size),
        origin=Origin(xyz=(x, -0.025, z)),
        material=clear,
        name=f"{prefix}_plate_neg",
    )
    _cyl_y(part, 0.017, 0.005, (x, 0.0315, z), dark, f"{prefix}_washer_pos")
    _cyl_y(part, 0.017, 0.005, (x, -0.0315, z), dark, f"{prefix}_washer_neg")


def _add_standard_link(part, metal: Material, clear: Material, dark: Material, prefix: str) -> None:
    """One slim moving link: central lug at the proximal joint and a distal clevis."""
    _cyl_y(part, 0.026, 0.018, (0.0, 0.0, 0.0), metal, "proximal_lug")
    part.visual(
        Box((0.198, 0.020, 0.016)),
        origin=Origin(xyz=(0.119, 0.0, 0.0)),
        material=metal,
        name="main_bar",
    )
    part.visual(
        Box((0.038, 0.060, 0.016)),
        origin=Origin(xyz=(LINK_LEN - 0.048, 0.0, 0.0)),
        material=metal,
        name="fork_bridge",
    )
    _add_side_plates(part, LINK_LEN, 0.0, clear, dark, prefix)


def _add_distal_link(part, metal: Material, dark: Material) -> None:
    """Final link with a small actuating tab instead of another fork."""
    _cyl_y(part, 0.026, 0.018, (0.0, 0.0, 0.0), metal, "proximal_lug")
    part.visual(
        Box((0.330, 0.020, 0.016)),
        origin=Origin(xyz=(0.185, 0.0, 0.0)),
        material=metal,
        name="main_bar",
    )
    part.visual(
        Box((0.075, 0.030, 0.020)),
        origin=Origin(xyz=(LINK_LEN + 0.070, 0.0, 0.0)),
        material=metal,
        name="end_tab",
    )
    _cyl_y(part, 0.010, 0.004, (LINK_LEN + 0.080, 0.017, 0.0), dark, "tab_mark_pos")
    _cyl_y(part, 0.010, 0.004, (LINK_LEN + 0.080, -0.017, 0.0), dark, "tab_mark_neg")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_top_lever_chain")

    black = model.material("blackened_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    clear = model.material("clear_polycarbonate", rgba=(0.62, 0.90, 1.0, 0.36))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.360, 0.200, 0.025)),
        origin=Origin(xyz=(0.075, 0.0, 0.0125)),
        material=black,
        name="foot_plate",
    )
    for i, (x, y) in enumerate(((-0.075, -0.075), (-0.075, 0.075), (0.225, -0.075), (0.225, 0.075))):
        base.visual(
            Box((0.045, 0.035, 0.008)),
            origin=Origin(xyz=(x, y, -0.0035)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )
    base.visual(
        Box((0.082, 0.076, 0.124)),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=black,
        name="pedestal",
    )
    _add_side_plates(base, 0.0, JOINT_Z, clear, black, "joint_0")

    link_0 = model.part("link_0")
    _add_standard_link(link_0, steel, clear, black, "joint_1")

    link_1 = model.part("link_1")
    _add_standard_link(link_1, steel, clear, black, "joint_2")

    distal_link = model.part("distal_link")
    _add_distal_link(distal_link, steel, black)

    limits = MotionLimits(effort=18.0, velocity=2.5, lower=-0.85, upper=1.15)
    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=HINGE_AXIS,
        motion_limits=limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK_LEN, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=limits,
    )
    model.articulation(
        "link_1_to_distal_link",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=distal_link,
        origin=Origin(xyz=(LINK_LEN, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    distal_link = object_model.get_part("distal_link")
    joints = (
        object_model.get_articulation("base_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_distal_link"),
    )

    ctx.check("three revolute joints", len(object_model.articulations) == 3, "lever chain should have exactly three joints")
    ctx.check(
        "hinge axes share one motion plane",
        all(tuple(joint.axis) == HINGE_AXIS for joint in joints),
        details=f"axes={[joint.axis for joint in joints]}",
    )

    ctx.allow_overlap(
        base,
        link_0,
        elem_a="joint_0_pin",
        elem_b="proximal_lug",
        reason="The first hinge pin is intentionally captured inside the link lug bore.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="joint_1_pin",
        elem_b="proximal_lug",
        reason="The second hinge pin is intentionally captured inside the link lug bore.",
    )
    ctx.allow_overlap(
        link_1,
        distal_link,
        elem_a="joint_2_pin",
        elem_b="proximal_lug",
        reason="The third hinge pin is intentionally captured inside the distal lug bore.",
    )

    ctx.expect_overlap(
        link_0,
        base,
        axes="xz",
        elem_a="proximal_lug",
        elem_b="joint_0_plate_pos",
        min_overlap=0.030,
        name="first lug is captured between base side plates",
    )
    ctx.expect_gap(
        base,
        link_0,
        axis="y",
        positive_elem="joint_0_plate_pos",
        negative_elem="proximal_lug",
        min_gap=0.006,
        max_gap=0.018,
        name="first hinge has side clearance",
    )
    ctx.expect_gap(
        link_0,
        base,
        axis="y",
        positive_elem="proximal_lug",
        negative_elem="joint_0_plate_neg",
        min_gap=0.006,
        max_gap=0.018,
        name="first hinge has opposite side clearance",
    )
    ctx.expect_within(
        base,
        link_0,
        axes="xz",
        inner_elem="joint_0_pin",
        outer_elem="proximal_lug",
        margin=0.002,
        name="first hinge pin is centered in the lug bore",
    )
    ctx.expect_overlap(
        base,
        link_0,
        axes="y",
        elem_a="joint_0_pin",
        elem_b="proximal_lug",
        min_overlap=0.016,
        name="first hinge pin passes through the lug",
    )
    ctx.expect_overlap(
        link_1,
        link_0,
        axes="xz",
        elem_a="proximal_lug",
        elem_b="joint_1_plate_pos",
        min_overlap=0.030,
        name="second lug is captured between clear side plates",
    )
    ctx.expect_gap(
        link_0,
        link_1,
        axis="y",
        positive_elem="joint_1_plate_pos",
        negative_elem="proximal_lug",
        min_gap=0.006,
        max_gap=0.018,
        name="second hinge has side clearance",
    )
    ctx.expect_within(
        link_0,
        link_1,
        axes="xz",
        inner_elem="joint_1_pin",
        outer_elem="proximal_lug",
        margin=0.002,
        name="second hinge pin is centered in the lug bore",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="y",
        elem_a="joint_1_pin",
        elem_b="proximal_lug",
        min_overlap=0.016,
        name="second hinge pin passes through the lug",
    )
    ctx.expect_overlap(
        distal_link,
        link_1,
        axes="xz",
        elem_a="proximal_lug",
        elem_b="joint_2_plate_pos",
        min_overlap=0.030,
        name="third lug is captured between clear side plates",
    )
    ctx.expect_within(
        link_1,
        distal_link,
        axes="xz",
        inner_elem="joint_2_pin",
        outer_elem="proximal_lug",
        margin=0.002,
        name="third hinge pin is centered in the lug bore",
    )
    ctx.expect_overlap(
        link_1,
        distal_link,
        axes="y",
        elem_a="joint_2_pin",
        elem_b="proximal_lug",
        min_overlap=0.016,
        name="third hinge pin passes through the lug",
    )

    rest_pos = ctx.part_world_position(distal_link)
    with ctx.pose({"base_to_link_0": 0.45, "link_0_to_link_1": 0.35, "link_1_to_distal_link": 0.25}):
        raised_pos = ctx.part_world_position(distal_link)
    ctx.check(
        "positive joint motion raises the distal chain in the shared plane",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.20 and abs(raised_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
