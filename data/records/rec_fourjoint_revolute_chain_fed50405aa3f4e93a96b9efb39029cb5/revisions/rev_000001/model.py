from __future__ import annotations

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


HINGE_Z = 0.13
LINK_WIDTH = 0.048
LINK_THICKNESS = 0.024
LUG_RADIUS = 0.035
LEAF_RADIUS = 0.048
LEAF_THICKNESS = 0.018
PIN_RADIUS = 0.011
PIN_LENGTH = 0.092
LEAF_Z = 0.030


def _add_revolute_link(part, *, length: float, material, pin_material) -> None:
    """Add one flat planar link with a proximal tongue and distal hinge fork."""
    bar_start = 0.018
    bar_end = length - 0.052
    part.visual(
        Cylinder(radius=LUG_RADIUS, length=LINK_THICKNESS),
        origin=Origin(),
        material=material,
        name="proximal_lug",
    )
    part.visual(
        Box((bar_end - bar_start, LINK_WIDTH, LINK_THICKNESS)),
        origin=Origin(xyz=((bar_start + bar_end) * 0.5, 0.0, 0.0)),
        material=material,
        name="web_bar",
    )
    part.visual(
        Box((0.030, LINK_WIDTH + 0.010, 0.085)),
        origin=Origin(xyz=(length - 0.057, 0.0, 0.0)),
        material=material,
        name="distal_block",
    )
    for name, z in (("lower_leaf", -LEAF_Z), ("upper_leaf", LEAF_Z)):
        part.visual(
            Cylinder(radius=LEAF_RADIUS, length=LEAF_THICKNESS),
            origin=Origin(xyz=(length, 0.0, z)),
            material=material,
            name=name,
        )
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(length, 0.0, 0.0)),
        material=pin_material,
        name="distal_pin",
    )


def _add_end_tab(part, *, material, accent_material) -> None:
    """Add the compact final tab carried by the fourth hinge."""
    part.visual(
        Cylinder(radius=LUG_RADIUS, length=LINK_THICKNESS),
        origin=Origin(),
        material=material,
        name="proximal_lug",
    )
    part.visual(
        Box((0.118, LINK_WIDTH, LINK_THICKNESS)),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=material,
        name="tab_bar",
    )
    part.visual(
        Cylinder(radius=0.028, length=LINK_THICKNESS),
        origin=Origin(xyz=(0.128, 0.0, 0.0)),
        material=material,
        name="end_pad",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.128, 0.0, 0.014)),
        material=accent_material,
        name="tool_hole",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_four_joint_revolute_chain")

    bracket_mat = model.material("matte_grounded_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    pin_mat = model.material("brushed_pin_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    blue_mat = model.material("blue_anodized_link", rgba=(0.08, 0.28, 0.78, 1.0))
    amber_mat = model.material("amber_anodized_link", rgba=(0.92, 0.48, 0.08, 1.0))
    dark_mat = model.material("dark_recess", rgba=(0.015, 0.015, 0.018, 1.0))

    root = model.part("root_bracket")
    root.visual(
        Box((0.170, 0.160, 0.025)),
        origin=Origin(xyz=(-0.050, 0.0, 0.0125)),
        material=bracket_mat,
        name="base_plate",
    )
    root.visual(
        Box((0.070, 0.052, 0.112)),
        origin=Origin(xyz=(-0.066, 0.0, 0.078)),
        material=bracket_mat,
        name="upright_web",
    )
    root.visual(
        Box((0.030, LINK_WIDTH + 0.014, 0.087)),
        origin=Origin(xyz=(-0.058, 0.0, HINGE_Z)),
        material=bracket_mat,
        name="hinge_bridge",
    )
    for name, z in (("lower_leaf", HINGE_Z - LEAF_Z), ("upper_leaf", HINGE_Z + LEAF_Z)):
        root.visual(
            Cylinder(radius=LEAF_RADIUS, length=LEAF_THICKNESS),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=bracket_mat,
            name=name,
        )
    root.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        material=pin_mat,
        name="hinge_pin",
    )
    for i, (x, y) in enumerate(((-0.100, -0.055), (-0.100, 0.055), (0.000, -0.055), (0.000, 0.055))):
        root.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.028)),
            material=pin_mat,
            name=f"base_bolt_{i}",
        )

    link_1 = model.part("link_1")
    link_2 = model.part("link_2")
    link_3 = model.part("link_3")
    end_tab = model.part("end_tab")

    link_lengths = (0.205, 0.180, 0.155)
    _add_revolute_link(link_1, length=link_lengths[0], material=blue_mat, pin_material=pin_mat)
    _add_revolute_link(link_2, length=link_lengths[1], material=amber_mat, pin_material=pin_mat)
    _add_revolute_link(link_3, length=link_lengths[2], material=blue_mat, pin_material=pin_mat)
    _add_end_tab(end_tab, material=amber_mat, accent_material=dark_mat)

    joint_limits = MotionLimits(effort=12.0, velocity=2.5, lower=-1.35, upper=1.35)
    model.articulation(
        "root_to_link_1",
        ArticulationType.REVOLUTE,
        parent=root,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(link_lengths[0], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(link_lengths[1], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_3_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=end_tab,
        origin=Origin(xyz=(link_lengths[2], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=joint_limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joint_names = (
        "root_to_link_1",
        "link_1_to_link_2",
        "link_2_to_link_3",
        "link_3_to_end_tab",
    )
    joints = [object_model.get_articulation(name) for name in joint_names]

    ctx.check("four serial joints", len(joints) == 4, details=f"joints={joint_names}")
    ctx.check(
        "all joints are revolute",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=str([j.articulation_type for j in joints]),
    )
    ctx.check(
        "joint axes are parallel",
        all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=str([j.axis for j in joints]),
    )

    hinge_pairs = (
        ("root_bracket", "link_1", "hinge_pin"),
        ("link_1", "link_2", "distal_pin"),
        ("link_2", "link_3", "distal_pin"),
        ("link_3", "end_tab", "distal_pin"),
    )
    for parent, child, pin_elem in hinge_pairs:
        ctx.allow_overlap(
            parent,
            child,
            elem_a=pin_elem,
            elem_b="proximal_lug",
            reason="The visible hinge pin is intentionally captured through the child tongue.",
        )
        ctx.expect_within(
            parent,
            child,
            axes="xy",
            inner_elem=pin_elem,
            outer_elem="proximal_lug",
            margin=0.0,
            name=f"{parent} pin centered in {child} lug",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="z",
            elem_a=pin_elem,
            elem_b="proximal_lug",
            min_overlap=0.020,
            name=f"{parent} pin passes through {child} lug",
        )

    tab = object_model.get_part("end_tab")
    rest_pos = ctx.part_world_position(tab)
    with ctx.pose({"root_to_link_1": 0.65}):
        swept_pos = ctx.part_world_position(tab)
    ctx.check(
        "chain sweeps in horizontal plane",
        rest_pos is not None
        and swept_pos is not None
        and abs(swept_pos[2] - rest_pos[2]) < 1e-6
        and swept_pos[1] > rest_pos[1] + 0.05,
        details=f"rest={rest_pos}, swept={swept_pos}",
    )

    return ctx.report()


object_model = build_object_model()
