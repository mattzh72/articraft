from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


BODY_DEPTH = 0.24
BODY_WIDTH = 0.34
BODY_HEIGHT = 1.05
BASE_THICKNESS = 0.035
WALL = 0.028

OUTLET_WIDTH = 0.30
OUTLET_HEIGHT = 0.48
OUTLET_CENTER_Z = 0.73
FRAME_DEPTH = 0.115
FRAME_WIDTH = 0.44
FRAME_HEIGHT = 0.62

FRONT_X = BODY_DEPTH / 2.0
FRAME_FRONT_X = FRONT_X + FRAME_DEPTH - 0.012
HINGE_X = 0.285
HINGE_Z = OUTLET_CENTER_Z + OUTLET_HEIGHT / 2.0 + 0.06

FLAP_WIDTH = 0.36
FLAP_HEIGHT = 0.54


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _housing_geometry() -> cq.Workplane:
    tower_center_z = BASE_THICKNESS + BODY_HEIGHT / 2.0

    outer = _box((BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT), (0.0, 0.0, tower_center_z))
    inner_cavity = _box(
        (BODY_DEPTH - 2.0 * WALL, BODY_WIDTH - 2.0 * WALL, BODY_HEIGHT + 0.08),
        (0.0, 0.0, tower_center_z),
    )
    outlet_cut = _box(
        (0.22, OUTLET_WIDTH, OUTLET_HEIGHT),
        (FRONT_X + 0.08, 0.0, OUTLET_CENTER_Z),
    )
    tower = outer.cut(inner_cavity).cut(outlet_cut)

    base_plate = _box((0.54, 0.66, BASE_THICKNESS), (0.0, 0.0, BASE_THICKNESS / 2.0))
    curb_outer = _box((0.33, 0.43, 0.085), (0.0, 0.0, BASE_THICKNESS + 0.0425))
    curb_inner = _box((0.22, 0.32, 0.11), (0.0, 0.0, BASE_THICKNESS + 0.045))
    curb = curb_outer.cut(curb_inner)

    frame_center_x = FRONT_X + FRAME_DEPTH / 2.0 - 0.012
    frame_center_z = OUTLET_CENTER_Z
    frame_outer = _box((FRAME_DEPTH, FRAME_WIDTH, FRAME_HEIGHT), (frame_center_x, 0.0, frame_center_z))
    frame_hole = _box((FRAME_DEPTH + 0.04, OUTLET_WIDTH, OUTLET_HEIGHT), (frame_center_x, 0.0, frame_center_z))
    outlet_frame = frame_outer.cut(frame_hole)

    top_header = _box(
        (0.095, FRAME_WIDTH, 0.075),
        (FRAME_FRONT_X - 0.035, 0.0, HINGE_Z - 0.020),
    )
    lower_drip_lip = _box(
        (0.060, FRAME_WIDTH + 0.035, 0.035),
        (FRAME_FRONT_X - 0.020, 0.0, OUTLET_CENTER_Z - OUTLET_HEIGHT / 2.0 - 0.035),
    )
    side_seam_0 = _box((0.018, 0.020, 0.88), (FRONT_X + 0.010, BODY_WIDTH / 2.0 - 0.026, 0.56))
    side_seam_1 = _box((0.018, 0.020, 0.88), (FRONT_X + 0.010, -BODY_WIDTH / 2.0 + 0.026, 0.56))

    housing = (
        base_plate.union(curb)
        .union(tower)
        .union(outlet_frame)
        .union(top_header)
        .union(lower_drip_lip)
        .union(side_seam_0)
        .union(side_seam_1)
    )

    # Fixed hinge ears are outside the flap's center barrel, leaving the middle
    # knuckle on the moving flap free to rotate about the same line.
    for y in (-0.235, 0.235):
        ear = _cylinder_y(0.065, 0.018, (HINGE_X, y, HINGE_Z))
        tab = _box((0.052, 0.078, 0.055), (HINGE_X - 0.026, y, HINGE_Z - 0.032))
        housing = housing.union(tab).union(ear)

    for x, y in ((0.205, 0.265), (0.205, -0.265), (-0.205, 0.265), (-0.205, -0.265)):
        screw = cq.Workplane("XY").cylinder(0.012, 0.018).translate((x, y, BASE_THICKNESS + 0.006))
        housing = housing.union(screw)

    return housing


def _flap_geometry() -> cq.Workplane:
    panel = _box((0.024, FLAP_WIDTH, FLAP_HEIGHT), (0.018, 0.0, -0.018 - FLAP_HEIGHT / 2.0))
    crown = _box((0.032, FLAP_WIDTH - 0.020, 0.032), (0.014, 0.0, -0.020))
    barrel = _cylinder_y(0.250, 0.016, (0.0, 0.0, 0.0))

    side_lip_0 = _box((0.046, 0.018, FLAP_HEIGHT - 0.060), (0.006, FLAP_WIDTH / 2.0 - 0.009, -0.045 - (FLAP_HEIGHT - 0.060) / 2.0))
    side_lip_1 = _box((0.046, 0.018, FLAP_HEIGHT - 0.060), (0.006, -FLAP_WIDTH / 2.0 + 0.009, -0.045 - (FLAP_HEIGHT - 0.060) / 2.0))
    bottom_hem = _box((0.038, FLAP_WIDTH, 0.028), (0.010, 0.0, -FLAP_HEIGHT - 0.020))

    return panel.union(crown).union(barrel).union(side_lip_0).union(side_lip_1).union(bottom_hem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = Material("dull_galvanized_steel", rgba=(0.62, 0.66, 0.67, 1.0))
    darker_metal = Material("slightly_darker_flap", rgba=(0.48, 0.52, 0.54, 1.0))
    shadow = Material("dark_outlet_void", rgba=(0.025, 0.028, 0.030, 1.0))
    roof = Material("dark_roof_membrane", rgba=(0.055, 0.050, 0.045, 1.0))
    model.materials.extend([galvanized, darker_metal, shadow, roof])

    housing = model.part("housing")
    housing.visual(
        Box((0.78, 0.90, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=roof,
        name="roof_patch",
    )
    housing.visual(
        mesh_from_cadquery(_housing_geometry(), "housing_shell", tolerance=0.0007),
        material=galvanized,
        name="housing_shell",
    )
    housing.visual(
        Cylinder(radius=0.006, length=0.54),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_pin",
    )
    housing.visual(
        Box((0.012, OUTLET_WIDTH + 0.025, OUTLET_HEIGHT + 0.020)),
        origin=Origin(xyz=(FRONT_X - 0.035, 0.0, OUTLET_CENTER_Z)),
        material=shadow,
        name="outlet_shadow",
    )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_flap_geometry(), "flap_panel", tolerance=0.0007),
        material=darker_metal,
        name="flap_panel",
    )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.18),
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
        elem_b="flap_panel",
        reason="The fixed hinge pin intentionally runs through the flap's rolled top barrel.",
    )

    ctx.check(
        "single top-edge weather flap hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in hinge.axis) == (0.0, -1.0, 0.0)
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and 1.0 <= hinge.motion_limits.upper <= 1.3,
        details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            housing,
            flap,
            axes="y",
            min_overlap=0.20,
            elem_a="hinge_pin",
            elem_b="flap_panel",
            name="hinge pin spans the flap barrel",
        )
        ctx.expect_within(
            housing,
            flap,
            axes="xz",
            margin=0.002,
            inner_elem="hinge_pin",
            outer_elem="flap_panel",
            name="hinge pin lies inside the rolled barrel projection",
        )
        ctx.expect_overlap(
            flap,
            housing,
            axes="yz",
            min_overlap=0.30,
            elem_a="flap_panel",
            elem_b="housing_shell",
            name="closed flap covers the framed outlet",
        )
        rest_aabb = ctx.part_world_aabb(flap)

    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_aabb = ctx.part_world_aabb(flap)

    ctx.check(
        "positive hinge motion lifts the weather flap outward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > rest_aabb[1][0] + 0.20
        and open_aabb[0][2] > rest_aabb[0][2] + 0.12,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
