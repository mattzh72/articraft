from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

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
    mesh_from_cadquery,
)


CASE_WIDTH = 0.038
CASE_DEPTH = 0.014
LOWER_HEIGHT = 0.039
LID_HEIGHT = 0.020
WALL = 0.0014
HINGE_CLEAR = 0.0028
HINGE_X = -CASE_WIDTH / 2.0 - HINGE_CLEAR
OPEN_ANGLE = 1.95

INSERT_WIDTH = 0.031
INSERT_DEPTH = 0.010
INSERT_BODY_HEIGHT = 0.037
INSERT_SEAT_Z = WALL


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="matte_pocket_lighter")

    matte_black = model.material("matte_black", rgba=(0.025, 0.026, 0.027, 1.0))
    case_edge = model.material("worn_edge", rgba=(0.12, 0.12, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.22, 1.0))
    black_cavity = model.material("black_cavity", rgba=(0.002, 0.002, 0.002, 1.0))
    wick_fiber = model.material("wick_fiber", rgba=(0.76, 0.68, 0.54, 1.0))
    brass = model.material("warm_brass", rgba=(0.72, 0.57, 0.30, 1.0))

    def rounded_box(width: float, depth: float, height: float, radius: float):
        return (
            cq.Workplane("XY")
            .box(width, depth, height)
            .edges("|Z")
            .fillet(radius)
            .translate((0.0, 0.0, height / 2.0))
        )

    def open_lower_shell():
        outer = rounded_box(CASE_WIDTH, CASE_DEPTH, LOWER_HEIGHT, 0.0031)
        inner_height = LOWER_HEIGHT - WALL + 0.004
        inner = (
            cq.Workplane("XY")
            .box(CASE_WIDTH - 2.0 * WALL, CASE_DEPTH - 2.0 * WALL, inner_height)
            .edges("|Z")
            .fillet(0.0017)
            .translate((0.0, 0.0, WALL + inner_height / 2.0))
        )
        return outer.cut(inner)

    def closed_lid_shell():
        center_x = HINGE_CLEAR + CASE_WIDTH / 2.0
        outer = (
            cq.Workplane("XY")
            .box(CASE_WIDTH, CASE_DEPTH, LID_HEIGHT)
            .edges("|Z")
            .fillet(0.0031)
            .translate((center_x, 0.0, LID_HEIGHT / 2.0))
        )
        cutter_height = LID_HEIGHT - WALL + 0.002
        inner = (
            cq.Workplane("XY")
            .box(CASE_WIDTH - 2.0 * WALL, CASE_DEPTH - 2.0 * WALL, cutter_height)
            .edges("|Z")
            .fillet(0.0017)
            .translate((center_x, 0.0, -0.001 + cutter_height / 2.0))
        )
        return outer.cut(inner).rotate((0, 0, 0), (0, 0, 1), OPEN_ANGLE * 180.0 / pi)

    outer_case = model.part("outer_case")
    outer_case.visual(
        mesh_from_cadquery(open_lower_shell(), "lower_shell"),
        material=matte_black,
        name="lower_shell",
    )
    # A very thin rubbed rim makes the black case read as matte-coated metal.
    outer_case.visual(
        Box((CASE_WIDTH - 0.002, 0.00045, 0.0010)),
        origin=Origin(xyz=(0.0, CASE_DEPTH / 2.0 - 0.00015, LOWER_HEIGHT - 0.0006)),
        material=case_edge,
        name="front_rim",
    )
    outer_case.visual(
        Box((CASE_WIDTH - 0.002, 0.00045, 0.0010)),
        origin=Origin(xyz=(0.0, -CASE_DEPTH / 2.0 + 0.00015, LOWER_HEIGHT - 0.0006)),
        material=case_edge,
        name="rear_rim",
    )
    outer_case.visual(
        Box((0.00045, CASE_DEPTH - 0.002, 0.0010)),
        origin=Origin(xyz=(CASE_WIDTH / 2.0 - 0.00015, 0.0, LOWER_HEIGHT - 0.0006)),
        material=case_edge,
        name="side_rim",
    )
    # Case-side hinge leaf and interleaved barrels.
    outer_case.visual(
        Box((HINGE_CLEAR + 0.0007, 0.0008, LID_HEIGHT + 0.007)),
        origin=Origin(
            xyz=(-CASE_WIDTH / 2.0 - HINGE_CLEAR / 2.0, -0.0046, LOWER_HEIGHT + LID_HEIGHT / 2.0 - 0.0035)
        ),
        material=matte_black,
        name="case_hinge_leaf",
    )
    outer_case.visual(
        Box((HINGE_CLEAR + 0.0008, 0.0050, 0.0072)),
        origin=Origin(xyz=(-CASE_WIDTH / 2.0 - HINGE_CLEAR / 2.0, -0.00225, LOWER_HEIGHT + 0.0034)),
        material=matte_black,
        name="case_hinge_bridge_0",
    )
    outer_case.visual(
        Box((HINGE_CLEAR + 0.0008, 0.0050, 0.0068)),
        origin=Origin(xyz=(-CASE_WIDTH / 2.0 - HINGE_CLEAR / 2.0, -0.00225, LOWER_HEIGHT + 0.0168)),
        material=matte_black,
        name="case_hinge_bridge_1",
    )
    for index, z_center in enumerate((LOWER_HEIGHT + 0.0038, LOWER_HEIGHT + 0.0164)):
        outer_case.visual(
            Cylinder(radius=0.00135, length=0.0064),
            origin=Origin(xyz=(HINGE_X, 0.0, z_center)),
            material=dark_steel,
            name=f"case_hinge_barrel_{index}",
        )
    outer_case.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, LOWER_HEIGHT + LID_HEIGHT)),
        mass=0.050,
        origin=Origin(xyz=(0.0, 0.0, (LOWER_HEIGHT + LID_HEIGHT) / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(closed_lid_shell(), "lid_shell"),
        material=matte_black,
        name="lid_shell",
    )
    leaf_center = (HINGE_CLEAR * 0.52 * cos(OPEN_ANGLE), HINGE_CLEAR * 0.52 * sin(OPEN_ANGLE), LID_HEIGHT / 2.0)
    lid.visual(
        Box((HINGE_CLEAR + 0.0008, 0.0038, 0.0054)),
        origin=Origin(xyz=(leaf_center[0], leaf_center[1], 0.0101), rpy=(0.0, 0.0, OPEN_ANGLE)),
        material=matte_black,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.00125, length=0.0052),
        origin=Origin(xyz=(0.0, 0.0, 0.0101)),
        material=dark_steel,
        name="lid_hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, LID_HEIGHT)),
        mass=0.018,
        origin=Origin(xyz=(CASE_WIDTH / 2.0, 0.0, LID_HEIGHT / 2.0)),
    )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(rounded_box(INSERT_WIDTH, INSERT_DEPTH, INSERT_BODY_HEIGHT, 0.0015), "insert_body"),
        material=brushed_steel,
        name="insert_body",
    )
    insert.visual(
        Box((INSERT_WIDTH + 0.0012, INSERT_DEPTH + 0.0008, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, INSERT_BODY_HEIGHT + 0.0004)),
        material=brushed_steel,
        name="insert_lip",
    )
    insert.visual(
        Box((0.018, 0.00075, 0.017)),
        origin=Origin(xyz=(-0.003, INSERT_DEPTH / 2.0, INSERT_BODY_HEIGHT + 0.0093)),
        material=brushed_steel,
        name="front_windshield",
    )
    insert.visual(
        Box((0.018, 0.00075, 0.017)),
        origin=Origin(xyz=(-0.003, -INSERT_DEPTH / 2.0, INSERT_BODY_HEIGHT + 0.0093)),
        material=brushed_steel,
        name="rear_windshield",
    )
    insert.visual(
        Box((0.0010, INSERT_DEPTH, 0.014)),
        origin=Origin(xyz=(-0.012, 0.0, INSERT_BODY_HEIGHT + 0.0080)),
        material=brushed_steel,
        name="chimney_side",
    )
    for face_index, y in enumerate((INSERT_DEPTH / 2.0 + 0.00042, -INSERT_DEPTH / 2.0 - 0.00042)):
        for row, z in enumerate((INSERT_BODY_HEIGHT + 0.0056, INSERT_BODY_HEIGHT + 0.0102, INSERT_BODY_HEIGHT + 0.0148)):
            for col, x in enumerate((-0.0085, -0.0040, 0.0005, 0.0050)):
                insert.visual(
                    Cylinder(radius=0.00055, length=0.00030),
                    origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
                    material=black_cavity,
                    name=f"vent_{face_index}_{row}_{col}",
                )
    insert.visual(
        Cylinder(radius=0.00225, length=0.00045),
        origin=Origin(xyz=(-0.006, 0.0, INSERT_BODY_HEIGHT + 0.00115)),
        material=black_cavity,
        name="wick_opening",
    )
    insert.visual(
        Cylinder(radius=0.00105, length=0.0085),
        origin=Origin(xyz=(-0.006, 0.0, INSERT_BODY_HEIGHT + 0.0053)),
        material=wick_fiber,
        name="wick",
    )
    insert.visual(
        Cylinder(radius=0.0012, length=0.012),
        origin=Origin(xyz=(0.0065, 0.0, INSERT_BODY_HEIGHT + 0.0060)),
        material=brass,
        name="flint_tube",
    )
    insert.visual(
        Box((0.0042, 0.0008, 0.016)),
        origin=Origin(xyz=(0.009, INSERT_DEPTH / 2.0 + 0.00045, INSERT_BODY_HEIGHT + 0.008)),
        material=brushed_steel,
        name="front_wheel_yoke",
    )
    insert.visual(
        Box((0.0042, 0.0008, 0.016)),
        origin=Origin(xyz=(0.009, -INSERT_DEPTH / 2.0 - 0.00045, INSERT_BODY_HEIGHT + 0.008)),
        material=brushed_steel,
        name="rear_wheel_yoke",
    )
    insert.inertial = Inertial.from_geometry(
        Box((INSERT_WIDTH, INSERT_DEPTH, INSERT_BODY_HEIGHT + 0.020)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, (INSERT_BODY_HEIGHT + 0.020) / 2.0)),
    )

    striker_wheel = model.part("striker_wheel")
    striker_wheel.visual(
        Cylinder(radius=0.0042, length=0.0068),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_core",
    )
    for tooth in range(18):
        angle = 2.0 * pi * tooth / 18.0
        striker_wheel.visual(
            Box((0.00105, 0.0074, 0.00085)),
            origin=Origin(
                xyz=(0.00435 * cos(angle), 0.0, 0.00435 * sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=brushed_steel,
            name=f"wheel_tooth_{tooth}",
        )
    striker_wheel.visual(
        Cylinder(radius=0.00075, length=0.0110),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="wheel_axle",
    )
    striker_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0042, length=0.0068),
        mass=0.002,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=outer_case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=5.0, lower=-OPEN_ANGLE, upper=0.15),
    )
    model.articulation(
        "insert_slide",
        ArticulationType.PRISMATIC,
        parent=outer_case,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, INSERT_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=0.0, upper=0.024),
    )
    model.articulation(
        "striker_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=striker_wheel,
        origin=Origin(xyz=(0.009, 0.0, INSERT_BODY_HEIGHT + 0.0140)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.04, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_case = object_model.get_part("outer_case")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")
    striker_wheel = object_model.get_part("striker_wheel")
    lid_hinge = object_model.get_articulation("lid_hinge")
    insert_slide = object_model.get_articulation("insert_slide")

    ctx.allow_overlap(
        outer_case,
        insert,
        elem_a="lower_shell",
        elem_b="insert_body",
        reason="The removable metal insert is intentionally seated inside the lower case sleeve proxy.",
    )

    ctx.expect_within(
        insert,
        outer_case,
        axes="xy",
        inner_elem="insert_body",
        outer_elem="lower_shell",
        margin=0.002,
        name="insert fits inside lower case footprint",
    )
    ctx.expect_overlap(
        insert,
        outer_case,
        axes="z",
        elem_a="insert_body",
        elem_b="lower_shell",
        min_overlap=0.032,
        name="insert remains deeply seated at rest",
    )
    ctx.expect_overlap(
        striker_wheel,
        insert,
        axes="xy",
        elem_a="wheel_axle",
        elem_b="front_wheel_yoke",
        min_overlap=0.0003,
        name="striker wheel axle is carried by insert yoke",
    )

    rest_pos = ctx.part_world_position(insert)
    with ctx.pose({insert_slide: 0.024}):
        ctx.expect_within(
            insert,
            outer_case,
            axes="xy",
            inner_elem="insert_body",
            outer_elem="lower_shell",
            margin=0.002,
            name="raised insert stays guided in shell",
        )
        ctx.expect_overlap(
            insert,
            outer_case,
            axes="z",
            elem_a="insert_body",
            elem_b="lower_shell",
            min_overlap=0.010,
            name="raised insert keeps retained insertion",
        )
        raised_pos = ctx.part_world_position(insert)
    ctx.check(
        "insert slides upward along case height",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.020,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({lid_hinge: -OPEN_ANGLE}):
        ctx.expect_overlap(
            lid,
            outer_case,
            axes="xy",
            elem_a="lid_shell",
            elem_b="lower_shell",
            min_overlap=0.012,
            name="closed lid covers lower case plan",
        )
        ctx.expect_gap(
            lid,
            outer_case,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="lower_shell",
            max_gap=0.001,
            max_penetration=0.0001,
            name="closed lid sits on lower case rim",
        )

    return ctx.report()


object_model = build_object_model()
