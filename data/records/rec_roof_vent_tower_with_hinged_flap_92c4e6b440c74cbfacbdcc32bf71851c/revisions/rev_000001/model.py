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
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.61, 0.60, 1.0))
    dark_interior = model.material("dark_interior", rgba=(0.025, 0.028, 0.030, 1.0))
    roof_membrane = model.material("roof_membrane", rgba=(0.13, 0.13, 0.12, 1.0))
    rubber_edge = model.material("black_rubber", rgba=(0.03, 0.03, 0.028, 1.0))

    housing = model.part("housing")

    # A low roof flashing plate anchors the tower and makes it read as rooftop equipment.
    housing.visual(
        Box((1.05, 0.86, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=roof_membrane,
        name="roof_flashing",
    )
    housing.visual(
        Box((0.70, 0.72, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0925)),
        material=galvanized,
        name="curb_cap",
    )

    # Thick rectangular duct walls.  The front is intentionally open at the upper
    # outlet, with only a lower apron below the framed flap.
    housing.visual(
        Box((0.54, 0.070, 0.82)),
        origin=Origin(xyz=(0.0, 0.315, 0.51)),
        material=galvanized,
        name="side_wall_0",
    )
    housing.visual(
        Box((0.54, 0.070, 0.82)),
        origin=Origin(xyz=(0.0, -0.315, 0.51)),
        material=galvanized,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.070, 0.70, 0.82)),
        origin=Origin(xyz=(-0.255, 0.0, 0.51)),
        material=galvanized,
        name="rear_wall",
    )
    housing.visual(
        Box((0.070, 0.70, 0.265)),
        origin=Origin(xyz=(0.255, 0.0, 0.2325)),
        material=galvanized,
        name="front_apron",
    )

    # Heavy frame around the moving weather flap.
    housing.visual(
        Box((0.105, 0.095, 0.60)),
        origin=Origin(xyz=(0.3025, 0.2875, 0.625)),
        material=galvanized,
        name="frame_jamb_0",
    )
    housing.visual(
        Box((0.105, 0.095, 0.60)),
        origin=Origin(xyz=(0.3025, -0.2875, 0.625)),
        material=galvanized,
        name="frame_jamb_1",
    )
    housing.visual(
        Box((0.105, 0.67, 0.085)),
        origin=Origin(xyz=(0.3025, 0.0, 0.9025)),
        material=galvanized,
        name="top_frame_rail",
    )
    housing.visual(
        Box((0.105, 0.67, 0.085)),
        origin=Origin(xyz=(0.3025, 0.0, 0.3475)),
        material=galvanized,
        name="bottom_frame_rail",
    )
    housing.visual(
        Box((0.018, 0.485, 0.455)),
        origin=Origin(xyz=(0.253, 0.0, 0.625)),
        material=dark_interior,
        name="outlet_shadow",
    )

    # End hinge brackets bolted to the heavy top rail, leaving the rotating barrel
    # clear between them.
    for i, y in enumerate((-0.292, 0.292)):
        housing.visual(
            Box((0.068, 0.038, 0.118)),
            origin=Origin(xyz=(0.382, y, 0.872)),
            material=galvanized,
            name=f"hinge_ear_{i}",
        )
        housing.visual(
            Box((0.070, 0.040, 0.050)),
            origin=Origin(xyz=(0.346, y, 0.872)),
            material=galvanized,
            name=f"hinge_foot_{i}",
        )
    housing.visual(
        Cylinder(radius=0.010, length=0.600),
        origin=Origin(xyz=(0.375, 0.0, 0.872), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber_edge,
        name="hinge_pin",
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.032, 0.480, 0.455)),
        # The child frame is the hinge line; the closed panel hangs down from it.
        origin=Origin(xyz=(0.036, 0.0, -0.2475)),
        material=galvanized,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.020, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.030, 0.440, 0.030)),
        origin=Origin(xyz=(0.018, 0.0, -0.030)),
        material=galvanized,
        name="hinge_leaf",
    )
    # Folded perimeter and pressed stiffeners make the flap read as a robust
    # weather panel rather than a thin placeholder sheet.
    flap.visual(
        Box((0.018, 0.505, 0.030)),
        origin=Origin(xyz=(0.053, 0.0, -0.475)),
        material=galvanized,
        name="lower_fold",
    )
    flap.visual(
        Box((0.018, 0.030, 0.395)),
        origin=Origin(xyz=(0.053, 0.0, -0.255)),
        material=galvanized,
        name="center_stiffener",
    )
    flap.visual(
        Box((0.018, 0.455, 0.028)),
        origin=Origin(xyz=(0.053, 0.0, -0.185)),
        material=galvanized,
        name="cross_stiffener",
    )
    flap.visual(
        Box((0.010, 0.500, 0.018)),
        origin=Origin(xyz=(0.021, 0.0, -0.044)),
        material=rubber_edge,
        name="top_seal",
    )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(0.375, 0.0, 0.872)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("housing_to_flap")

    ctx.allow_overlap(
        housing,
        flap,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The hinge pin is intentionally captured inside the flap's rolled barrel.",
    )

    ctx.check(
        "single top-edge revolute flap",
        hinge.articulation_type == ArticulationType.REVOLUTE and len(object_model.articulations) == 1,
        details=f"joint={hinge}",
    )
    ctx.check(
        "hinge axis is horizontal across outlet",
        abs(hinge.axis[1]) > 0.99 and abs(hinge.axis[0]) < 0.01 and abs(hinge.axis[2]) < 0.01,
        details=f"axis={hinge.axis}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_within(
            housing,
            flap,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem="hinge_barrel",
            margin=0.002,
            name="hinge pin is concentric in the flap barrel",
        )
        ctx.expect_overlap(
            housing,
            flap,
            axes="y",
            min_overlap=0.45,
            elem_a="hinge_pin",
            elem_b="hinge_barrel",
            name="hinge pin spans the rotating barrel",
        )
        ctx.expect_gap(
            flap,
            housing,
            axis="x",
            min_gap=0.035,
            max_gap=0.055,
            positive_elem="flap_panel",
            negative_elem="top_frame_rail",
            name="closed flap sits proud of heavy frame",
        )
        ctx.expect_overlap(
            flap,
            housing,
            axes="yz",
            min_overlap=0.40,
            elem_a="flap_panel",
            elem_b="outlet_shadow",
            name="flap covers the framed outlet opening",
        )
        closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    with ctx.pose({hinge: 1.15}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    if closed_aabb is not None and open_aabb is not None:
        closed_center_x = (closed_aabb[0][0] + closed_aabb[1][0]) * 0.5
        open_center_x = (open_aabb[0][0] + open_aabb[1][0]) * 0.5
        closed_center_z = (closed_aabb[0][2] + closed_aabb[1][2]) * 0.5
        open_center_z = (open_aabb[0][2] + open_aabb[1][2]) * 0.5
        ctx.check(
            "flap opens outward and upward",
            open_center_x > closed_center_x + 0.17 and open_center_z > closed_center_z + 0.12,
            details=(
                f"closed_center=({closed_center_x:.3f}, {closed_center_z:.3f}), "
                f"open_center=({open_center_x:.3f}, {open_center_z:.3f})"
            ),
        )
    else:
        ctx.fail("flap pose measurement available", "Could not read flap panel AABBs.")

    return ctx.report()


object_model = build_object_model()
