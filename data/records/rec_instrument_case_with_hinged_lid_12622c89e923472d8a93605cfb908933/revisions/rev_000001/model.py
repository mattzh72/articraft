from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CASE_L = 0.56
CASE_W = 0.34
BASE_H = 0.155
LID_H = 0.085
WALL = 0.014
CORNER_R = 0.045
HINGE_Z = BASE_H + 0.005
HINGE_X = -CASE_L / 2.0 - 0.014
LID_X_OFFSET = 0.028


def _rounded_prism(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    """Rounded rectangle extruded upward from z=0, centered in x/y."""
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )


def _base_shell() -> cq.Workplane:
    outer = _rounded_prism(CASE_L, CASE_W, BASE_H, CORNER_R)
    inner = _rounded_prism(
        CASE_L - 2.0 * WALL,
        CASE_W - 2.0 * WALL,
        BASE_H + 0.010,
        CORNER_R - WALL,
    ).translate((0.0, 0.0, WALL))
    return outer.cut(inner)


def _lid_shell() -> cq.Workplane:
    # The lid part frame is on the rear hinge line.  The closed shell extends
    # forward along local +X from that line, matching the revolute convention.
    outer = _rounded_prism(CASE_L, CASE_W, LID_H, CORNER_R).translate(
        (CASE_L / 2.0 + LID_X_OFFSET, 0.0, 0.0)
    )
    inner = _rounded_prism(
        CASE_L - 2.0 * WALL,
        CASE_W - 2.0 * WALL,
        LID_H - WALL + 0.008,
        CORNER_R - WALL,
    ).translate((CASE_L / 2.0 + LID_X_OFFSET, 0.0, -0.004))
    return outer.cut(inner)


def _gasket_ring() -> cq.Workplane:
    outer = _rounded_prism(CASE_L - 0.025, CASE_W - 0.025, 0.004, CORNER_R - 0.012)
    inner = _rounded_prism(CASE_L - 0.070, CASE_W - 0.070, 0.007, CORNER_R - 0.035).translate(
        (0.0, 0.0, -0.001)
    )
    return outer.cut(inner)


def _foam_insert() -> cq.Workplane:
    foam = _rounded_prism(CASE_L - 0.075, CASE_W - 0.075, 0.036, 0.025)
    instrument_pocket = _rounded_prism(0.265, 0.135, 0.022, 0.018).translate((0.015, 0.0, 0.020))
    cable_pocket = _rounded_prism(0.105, 0.075, 0.024, 0.012).translate((-0.185, 0.0, 0.018))
    finger_notch = cq.Workplane("XY").circle(0.030).extrude(0.024).translate((0.162, 0.0, 0.020))
    return foam.cut(instrument_pocket).cut(cable_pocket).cut(finger_notch)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_instrument_case")

    case_plastic = model.material("textured_charcoal_plastic", rgba=(0.055, 0.060, 0.060, 1.0))
    edge_plastic = model.material("slightly_lighter_edges", rgba=(0.090, 0.095, 0.090, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    foam = model.material("dark_eggcrate_foam", rgba=(0.025, 0.027, 0.030, 1.0))
    latch_metal = model.material("brushed_dark_latch_metal", rgba=(0.45, 0.46, 0.43, 1.0))
    pin_metal = model.material("hinge_pin_steel", rgba=(0.67, 0.68, 0.64, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shell(), "base_shell", tolerance=0.0012, angular_tolerance=0.08),
        material=case_plastic,
        name="base_shell",
    )
    base.visual(
        mesh_from_cadquery(_gasket_ring(), "base_gasket", tolerance=0.001, angular_tolerance=0.08),
        origin=Origin(xyz=(0.0, 0.0, BASE_H)),
        material=rubber,
        name="base_gasket",
    )
    base.visual(
        mesh_from_cadquery(_foam_insert(), "foam_insert", tolerance=0.001, angular_tolerance=0.08),
        origin=Origin(xyz=(0.0, 0.0, WALL)),
        material=foam,
        name="foam_insert",
    )

    # Two fixed latch stacks on the flat front face, with separate keeper plates
    # on the lid so the seam reads as an equipment case closure.
    for i, y in enumerate((-0.105, 0.105)):
        base.visual(
            Box((0.009, 0.072, 0.052)),
            origin=Origin(xyz=(CASE_L / 2.0 + 0.0045, y, BASE_H - 0.041)),
            material=latch_metal,
            name=f"latch_plate_{i}",
        )
        base.visual(
            Box((0.014, 0.052, 0.020)),
            origin=Origin(xyz=(CASE_L / 2.0 + 0.0115, y, BASE_H - 0.022)),
            material=pin_metal,
            name=f"latch_pull_{i}",
        )
        base.visual(
            Cylinder(radius=0.0045, length=0.064),
            origin=Origin(
                xyz=(CASE_L / 2.0 + 0.0125, y, BASE_H - 0.057),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=pin_metal,
            name=f"latch_pin_{i}",
        )

    # Rear hinge leaves and alternating knuckles on the fixed base half.
    for i, y in enumerate((-0.130, 0.130)):
        base.visual(
            Box((0.014, 0.070, 0.050)),
            origin=Origin(xyz=(-CASE_L / 2.0 - 0.007, y, BASE_H - 0.020)),
            material=latch_metal,
            name=f"base_hinge_leaf_{i}",
        )
    for i, y in enumerate((-0.130, 0.130)):
        base.visual(
            Cylinder(radius=0.010, length=0.080),
            origin=Origin(
                xyz=(HINGE_X, y, HINGE_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=pin_metal,
            name=f"base_hinge_knuckle_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell", tolerance=0.0012, angular_tolerance=0.08),
        material=edge_plastic,
        name="lid_shell",
    )
    for i, y in enumerate((-0.105, 0.105)):
        lid.visual(
            Box((0.008, 0.076, 0.024)),
            origin=Origin(xyz=(CASE_L + LID_X_OFFSET + 0.004, y, 0.020)),
            material=latch_metal,
            name=f"keeper_plate_{i}",
        )

    for i, y in enumerate((0.0,)):
        lid.visual(
            Box((0.038, 0.120, 0.034)),
            origin=Origin(xyz=(0.009, y, 0.020)),
            material=latch_metal,
            name=f"lid_hinge_leaf_{i}",
        )
        lid.visual(
            Cylinder(radius=0.010, length=0.180),
            origin=Origin(
                xyz=(0.0, y, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=pin_metal,
            name=f"lid_hinge_knuckle_{i}",
        )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("rear_hinge")

    ctx.check(
        "single rear lid hinge",
        len(object_model.articulations) == 1
        and hinge.motion_limits is not None
        and abs(hinge.motion_limits.upper - math.radians(100.0)) < 1e-6,
        details="The case should have exactly one rear revolute joint with a 100 degree opening limit.",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=0.001,
        max_gap=0.008,
        positive_elem="lid_shell",
        negative_elem="base_shell",
        name="closed shells meet at a narrow seam",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        min_overlap=0.25,
        elem_a="lid_shell",
        elem_b="base_shell",
        name="closed lid footprint covers the base",
    )

    closed_pos = ctx.part_world_position(lid)
    with ctx.pose({hinge: math.radians(100.0)}):
        lid_aabb = ctx.part_world_aabb(lid)
        base_aabb = ctx.part_world_aabb(base)
        ctx.check(
            "lid opens upward and rearward",
            closed_pos is not None
            and lid_aabb is not None
            and base_aabb is not None
            and lid_aabb[1][2] > base_aabb[1][2] + 0.20
            and lid_aabb[0][0] < base_aabb[0][0] - 0.05,
            details=f"closed={closed_pos}, lid_aabb={lid_aabb}, base_aabb={base_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
