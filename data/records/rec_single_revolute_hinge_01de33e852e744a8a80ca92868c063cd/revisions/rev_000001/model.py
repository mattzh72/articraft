from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    ClevisBracketGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PIN_HEIGHT = 0.055
CLEVIS_WIDTH = 0.090
CLEVIS_GAP = 0.046
CLEVIS_DEPTH = 0.075
CLEVIS_HEIGHT = 0.085
BASE_THICKNESS = 0.014
PIN_DIAMETER = 0.016

PLATE_WIDTH = 0.040
PLATE_LENGTH = 0.165
PLATE_THICKNESS = 0.007
PLATE_START_Y = 0.012
PLATE_CENTER_Y = PLATE_START_Y + PLATE_LENGTH * 0.5
BARREL_LENGTH = 0.038
BARREL_OUTER_RADIUS = 0.017
BARREL_BORE_RADIUS = 0.007


def _moving_plate_leaf() -> cq.Workplane:
    """Rounded moving leaf with two through-bolt holes, in the hinge frame."""
    plate = (
        cq.Workplane("XY")
        .box(PLATE_WIDTH, PLATE_LENGTH, PLATE_THICKNESS)
        .translate((0.0, PLATE_CENTER_Y, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )
    bolt_cutter = (
        cq.Workplane("XY")
        .pushPoints([(0.0, PLATE_START_Y + 0.055), (0.0, PLATE_START_Y + 0.115)])
        .circle(0.006)
        .extrude(PLATE_THICKNESS * 4.0, both=True)
    )
    return plate.cut(bolt_cutter)


def _hinge_barrel() -> cq.Workplane:
    """Hollow central barrel around the revolute joint axis."""
    return (
        cq.Workplane("XY")
        .circle(BARREL_OUTER_RADIUS)
        .circle(BARREL_BORE_RADIUS)
        .extrude(BARREL_LENGTH * 0.5, both=True)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clevis_backed_hinge_unit")

    dark_steel = model.material("dark_burnished_steel", rgba=(0.16, 0.17, 0.17, 1.0))
    zinc = model.material("brushed_zinc", rgba=(0.58, 0.60, 0.58, 1.0))
    blue = model.material("blue_powder_coat", rgba=(0.05, 0.20, 0.46, 1.0))
    black = model.material("shadowed_recess", rgba=(0.015, 0.015, 0.014, 1.0))

    support = model.part("fixed_support")
    clevis_mesh = mesh_from_geometry(
        ClevisBracketGeometry(
            (CLEVIS_WIDTH, CLEVIS_DEPTH, CLEVIS_HEIGHT),
            gap_width=CLEVIS_GAP,
            bore_diameter=PIN_DIAMETER + 0.002,
            bore_center_z=PIN_HEIGHT,
            base_thickness=BASE_THICKNESS,
            corner_radius=0.004,
            center=False,
        ),
        "clevis_body",
    )
    support.visual(clevis_mesh, material=dark_steel, name="clevis_body")

    # Only the pin heads are modeled as fixed caps; the barrel itself is hollow
    # and rotates as part of the moving plate.  The shaft is given a tiny
    # interference fit in tests so the bearing reads physically captured.
    support.visual(
        Cylinder(radius=BARREL_BORE_RADIUS + 0.0002, length=CLEVIS_WIDTH + 0.004),
        origin=Origin(xyz=(0.0, 0.0, PIN_HEIGHT), rpy=(0.0, pi / 2.0, 0.0)),
        material=zinc,
        name="hinge_pin",
    )
    cap_length = 0.004
    cap_radius = 0.012
    cap_x = CLEVIS_WIDTH * 0.5 + cap_length * 0.5 - 0.0005
    for i, x in enumerate((-cap_x, cap_x)):
        support.visual(
            Cylinder(radius=cap_radius, length=cap_length),
            origin=Origin(xyz=(x, 0.0, PIN_HEIGHT), rpy=(0.0, pi / 2.0, 0.0)),
            material=zinc,
            name=f"pin_head_{i}",
        )

    # Dark flush disks suggest countersunk mounting holes in the fixed base.
    for i, x in enumerate((-0.026, 0.026)):
        support.visual(
            Cylinder(radius=0.006, length=0.0008),
            origin=Origin(xyz=(x, -0.022, BASE_THICKNESS + 0.0004)),
            material=black,
            name=f"base_hole_{i}",
        )

    plate = model.part("moving_plate")
    plate.visual(
        mesh_from_cadquery(_moving_plate_leaf(), "plate_leaf", tolerance=0.0006),
        material=blue,
        name="plate_leaf",
    )
    plate.visual(
        mesh_from_cadquery(_hinge_barrel(), "hinge_barrel", tolerance=0.0005),
        material=zinc,
        name="hinge_barrel",
    )

    model.articulation(
        "hinge_pin",
        ArticulationType.REVOLUTE,
        parent=support,
        child=plate,
        origin=Origin(xyz=(0.0, 0.0, PIN_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.5, lower=0.0, upper=1.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("fixed_support")
    plate = object_model.get_part("moving_plate")
    hinge = object_model.get_articulation("hinge_pin")

    ctx.allow_overlap(
        support,
        plate,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason=(
            "The fixed pin is intentionally modeled with a slight press-fit "
            "inside the moving barrel bore so the clevis hinge is visibly captured."
        ),
    )

    ctx.check(
        "single revolute hinge joint",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(hinge.axis) == (1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations!r}, axis={hinge.axis}",
    )

    ctx.expect_within(
        plate,
        support,
        axes="x",
        inner_elem="hinge_barrel",
        outer_elem="clevis_body",
        margin=0.0,
        name="barrel is centered between fixed cheeks",
    )
    ctx.expect_overlap(
        plate,
        support,
        axes="y",
        elem_a="hinge_barrel",
        elem_b="clevis_body",
        min_overlap=0.025,
        name="barrel sits in the clevis root",
    )
    ctx.expect_within(
        support,
        plate,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="pin is coaxial inside barrel bore",
    )
    ctx.expect_overlap(
        support,
        plate,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.036,
        name="pin passes through the full barrel length",
    )

    rest_aabb = ctx.part_element_world_aabb(plate, elem="plate_leaf")
    with ctx.pose({hinge: 1.35}):
        raised_aabb = ctx.part_element_world_aabb(plate, elem="plate_leaf")
    rest_center_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
    raised_center_z = None if raised_aabb is None else (raised_aabb[0][2] + raised_aabb[1][2]) * 0.5
    ctx.check(
        "moving plate rotates upward about hinge pin",
        rest_center_z is not None
        and raised_center_z is not None
        and raised_center_z > rest_center_z + 0.055,
        details=f"rest_center_z={rest_center_z}, raised_center_z={raised_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
