from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("concrete", rgba=(0.63, 0.63, 0.61, 1.0))
    steel = model.material("bridge_steel", rgba=(0.24, 0.29, 0.34, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.14, 0.16, 0.18, 1.0))
    asphalt = model.material("asphalt", rgba=(0.20, 0.21, 0.22, 1.0))
    curb_paint = model.material("curb_paint", rgba=(0.78, 0.77, 0.67, 1.0))

    hinge_axis_height = 1.00
    roadway_top = 1.18
    leaf_length = 12.00
    leaf_width = 5.80

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((4.60, 7.80, 0.78)),
        origin=Origin(xyz=(-2.30, 0.00, 0.39)),
        material=concrete,
        name="foundation_block",
    )
    shore_frame.visual(
        Box((3.20, 5.70, 0.40)),
        origin=Origin(xyz=(-1.60, 0.00, roadway_top - 0.20)),
        material=asphalt,
        name="approach_deck",
    )
    shore_frame.visual(
        Box((0.75, 0.56, 1.05)),
        origin=Origin(xyz=(-0.28, -3.70, 1.305)),
        material=concrete,
        name="left_side_pier",
    )
    shore_frame.visual(
        Box((0.75, 0.56, 1.05)),
        origin=Origin(xyz=(-0.28, 3.70, 1.305)),
        material=concrete,
        name="right_side_pier",
    )
    shore_frame.visual(
        Box((0.70, 7.40, 0.28)),
        origin=Origin(xyz=(-0.30, 0.00, 1.69)),
        material=dark_steel,
        name="transverse_portal_beam",
    )
    shore_frame.visual(
        Box((0.40, 0.36, 0.44)),
        origin=Origin(xyz=(0.06, -3.24, hinge_axis_height)),
        material=steel,
        name="bearing_left",
    )
    shore_frame.visual(
        Box((0.40, 0.36, 0.44)),
        origin=Origin(xyz=(0.06, 3.24, hinge_axis_height)),
        material=steel,
        name="bearing_right",
    )
    shore_frame.inertial = Inertial.from_geometry(
        Box((4.60, 7.80, 1.90)),
        mass=180000.0,
        origin=Origin(xyz=(-2.10, 0.00, 0.95)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((leaf_length, leaf_width, 0.16)),
        origin=Origin(xyz=(leaf_length / 2.0, 0.00, 0.10)),
        material=asphalt,
        name="leaf_deck",
    )
    bridge_leaf.visual(
        Box((11.60, 0.30, 0.82)),
        origin=Origin(xyz=(6.02, -2.55, -0.31)),
        material=steel,
        name="left_main_girder",
    )
    bridge_leaf.visual(
        Box((11.60, 0.30, 0.82)),
        origin=Origin(xyz=(6.02, 2.55, -0.31)),
        material=steel,
        name="right_main_girder",
    )
    bridge_leaf.visual(
        Box((10.80, 0.34, 0.62)),
        origin=Origin(xyz=(6.30, 0.00, -0.21)),
        material=steel,
        name="center_girder",
    )
    bridge_leaf.visual(
        Box((0.60, leaf_width, 0.56)),
        origin=Origin(xyz=(0.30, 0.00, -0.12)),
        material=dark_steel,
        name="hinge_floorbeam",
    )
    bridge_leaf.visual(
        Box((0.48, leaf_width, 0.50)),
        origin=Origin(xyz=(leaf_length - 0.24, 0.00, -0.09)),
        material=dark_steel,
        name="free_end_beam",
    )
    bridge_leaf.visual(
        Box((leaf_length, 0.16, 0.14)),
        origin=Origin(xyz=(leaf_length / 2.0, -2.82, 0.15)),
        material=curb_paint,
        name="left_curb",
    )
    bridge_leaf.visual(
        Box((leaf_length, 0.16, 0.14)),
        origin=Origin(xyz=(leaf_length / 2.0, 2.82, 0.15)),
        material=curb_paint,
        name="right_curb",
    )
    bridge_leaf.visual(
        Box((0.30, 0.20, 0.44)),
        origin=Origin(xyz=(0.16, -2.96, -0.04)),
        material=dark_steel,
        name="left_bearing_web",
    )
    bridge_leaf.visual(
        Box((0.30, 0.20, 0.44)),
        origin=Origin(xyz=(0.16, 2.96, -0.04)),
        material=dark_steel,
        name="right_bearing_web",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.18, length=0.20),
        origin=Origin(
            xyz=(0.12, -2.96, 0.00),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="side_journal_left",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.18, length=0.20),
        origin=Origin(
            xyz=(0.12, 2.96, 0.00),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="side_journal_right",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((leaf_length, leaf_width, 1.10)),
        mass=45000.0,
        origin=Origin(xyz=(leaf_length / 2.0, 0.00, -0.18)),
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.00, 0.00, hinge_axis_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=300000.0,
            velocity=0.20,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("shore_to_leaf")

    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="x",
        positive_elem="leaf_deck",
        negative_elem="approach_deck",
        min_gap=0.0,
        max_gap=0.01,
        name="leaf closes with a narrow shore-side hinge gap",
    )
    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="y",
        elem_a="leaf_deck",
        elem_b="approach_deck",
        min_overlap=5.60,
        name="leaf deck aligns with the approach width",
    )
    ctx.expect_contact(
        shore_frame,
        bridge_leaf,
        elem_a="bearing_left",
        elem_b="side_journal_left",
        contact_tol=1e-6,
        name="left side bearing supports the leaf journal",
    )
    ctx.expect_contact(
        shore_frame,
        bridge_leaf,
        elem_a="bearing_right",
        elem_b="side_journal_right",
        contact_tol=1e-6,
        name="right side bearing supports the leaf journal",
    )

    rest_aabb = ctx.part_element_world_aabb(bridge_leaf, elem="free_end_beam")
    upper_limit = 1.10
    if hinge.motion_limits is not None and hinge.motion_limits.upper is not None:
        upper_limit = hinge.motion_limits.upper
    with ctx.pose({hinge: upper_limit}):
        raised_aabb = ctx.part_element_world_aabb(bridge_leaf, elem="free_end_beam")

    rest_top = None if rest_aabb is None else rest_aabb[1][2]
    raised_top = None if raised_aabb is None else raised_aabb[1][2]
    ctx.check(
        "bridge leaf opens upward from the shore hinge",
        rest_top is not None and raised_top is not None and raised_top > rest_top + 7.0,
        details=f"rest_top={rest_top}, raised_top={raised_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
