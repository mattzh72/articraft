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


HANDLE_LENGTH = 0.118
HANDLE_WIDTH = 0.020
HANDLE_HEIGHT = 0.030
CAVITY_WIDTH = 0.013
CAVITY_HEIGHT = 0.014
CAVITY_Z = -0.001
NOSE_X = HANDLE_LENGTH / 2.0
SLIDE_TRAVEL = 0.026


def _rounded_handle_shell() -> cq.Workplane:
    """One-piece hollow handle shell with a nose opening and thumb slot."""

    outer = (
        cq.Workplane("XY")
        .box(HANDLE_LENGTH, HANDLE_WIDTH, HANDLE_HEIGHT)
        .edges("|X")
        .fillet(0.006)
    )

    blade_tunnel = (
        cq.Workplane("XY")
        .box(0.112, CAVITY_WIDTH, CAVITY_HEIGHT)
        .translate((0.006, 0.0, CAVITY_Z))
    )
    thumb_slot = (
        cq.Workplane("XY")
        .box(0.061, 0.008, 0.030)
        .translate((-0.012, 0.0, 0.012))
    )

    return outer.cut(blade_tunnel).cut(thumb_slot)


def _blade_shape() -> cq.Workplane:
    """Thin trapezoid utility blade, modeled in the carriage frame."""

    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.008, -0.0044),
                (0.034, -0.0044),
                (0.049, 0.0000),
                (0.034, 0.0044),
                (-0.008, 0.0044),
            ]
        )
        .close()
        .extrude(0.0028, both=True)
        .edges()
        .chamfer(0.00035)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_safety_knife")

    orange = model.material("safety_orange", rgba=(1.0, 0.33, 0.03, 1.0))
    black = model.material("molded_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    dark = model.material("blackened_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    steel = model.material("satin_blade_steel", rgba=(0.78, 0.79, 0.76, 1.0))
    graphite = model.material("graphite_plastic", rgba=(0.11, 0.12, 0.13, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_rounded_handle_shell(), "hollow_handle_shell", tolerance=0.00045),
        material=orange,
        name="handle_shell",
    )

    # Dark overmolded grip pads on the two broad side faces.
    for y, name in ((0.0111, "side_grip_0"), (-0.0111, "side_grip_1")):
        handle.visual(
            Box((0.062, 0.0022, 0.011)),
            origin=Origin(xyz=(-0.015, y, -0.001)),
            material=black,
            name=name,
        )

    # Rectangular metal liner around the nose aperture.
    handle.visual(
        Box((0.004, 0.020, 0.003)),
        origin=Origin(xyz=(NOSE_X + 0.001, 0.0, CAVITY_Z + CAVITY_HEIGHT / 2.0 + 0.0015)),
        material=dark,
        name="nose_top_liner",
    )
    handle.visual(
        Box((0.004, 0.020, 0.003)),
        origin=Origin(xyz=(NOSE_X + 0.001, 0.0, CAVITY_Z - CAVITY_HEIGHT / 2.0 - 0.0015)),
        material=dark,
        name="nose_bottom_liner",
    )
    for y, name in ((0.0087, "nose_side_liner_0"), (-0.0087, "nose_side_liner_1")):
        handle.visual(
            Box((0.004, 0.003, 0.017)),
            origin=Origin(xyz=(NOSE_X + 0.001, y, CAVITY_Z)),
            material=dark,
            name=name,
        )

    # Two fixed hinge ears at the nose, leaving the moving flap knuckle centered.
    hinge_x = NOSE_X + 0.004
    hinge_z = HANDLE_HEIGHT / 2.0 + 0.002
    for y, name in ((0.007, "hinge_ear_0"), (-0.007, "hinge_ear_1")):
        handle.visual(
            Box((0.006, 0.004, 0.004)),
            origin=Origin(xyz=(hinge_x - 0.001, y, HANDLE_HEIGHT / 2.0 + 0.0008)),
            material=dark,
            name=f"{name}_saddle",
        )
        handle.visual(
            Cylinder(radius=0.002, length=0.004),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=name,
        )

    carriage = model.part("blade_carriage")
    carriage.visual(
        Box((0.046, 0.0082, 0.0095)),
        origin=Origin(xyz=(-0.020, 0.0, CAVITY_Z)),
        material=graphite,
        name="carriage_body",
    )
    carriage.visual(
        mesh_from_cadquery(_blade_shape(), "utility_blade", tolerance=0.00025),
        origin=Origin(xyz=(0.006, 0.0, CAVITY_Z)),
        material=steel,
        name="blade",
    )
    carriage.visual(
        Box((0.007, 0.005, 0.015)),
        origin=Origin(xyz=(-0.018, 0.0, 0.0085)),
        material=graphite,
        name="thumb_stem",
    )
    carriage.visual(
        Box((0.017, 0.010, 0.004)),
        origin=Origin(xyz=(-0.018, 0.0, 0.0165)),
        material=black,
        name="thumb_slider",
    )

    guard = model.part("guard_flap")
    guard.visual(
        Cylinder(radius=0.0021, length=0.008),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="flap_knuckle",
    )
    guard.visual(
        Box((0.0032, 0.0155, 0.018)),
        origin=Origin(xyz=(0.0016, 0.0, -0.010)),
        material=dark,
        name="guard_plate",
    )
    guard.visual(
        Box((0.005, 0.011, 0.003)),
        origin=Origin(xyz=(0.0030, 0.0, -0.0185)),
        material=black,
        name="guard_tip_pad",
    )

    model.articulation(
        "handle_to_carriage",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_TRAVEL, effort=25.0, velocity=0.18),
    )
    model.articulation(
        "handle_to_guard",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=guard,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=2.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    carriage = object_model.get_part("blade_carriage")
    guard = object_model.get_part("guard_flap")
    slide = object_model.get_articulation("handle_to_carriage")
    hinge = object_model.get_articulation("handle_to_guard")

    ctx.expect_within(
        carriage,
        handle,
        axes="yz",
        inner_elem="carriage_body",
        outer_elem="handle_shell",
        margin=0.001,
        name="carriage runs inside the handle envelope",
    )
    ctx.expect_within(
        carriage,
        handle,
        axes="x",
        inner_elem="blade",
        outer_elem="handle_shell",
        margin=0.0,
        name="blade is fully retracted at the lower slide stop",
    )
    ctx.expect_overlap(
        carriage,
        handle,
        axes="x",
        elem_a="carriage_body",
        elem_b="handle_shell",
        min_overlap=0.035,
        name="carriage remains retained in the handle",
    )

    rest_carriage = ctx.part_world_position(carriage)
    rest_blade = ctx.part_element_world_aabb(carriage, elem="blade")
    rest_guard = ctx.part_world_aabb(guard)
    handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({slide: SLIDE_TRAVEL, hinge: 1.15}):
        extended_carriage = ctx.part_world_position(carriage)
        extended_blade = ctx.part_element_world_aabb(carriage, elem="blade")
        open_guard = ctx.part_world_aabb(guard)
        ctx.expect_overlap(
            carriage,
            handle,
            axes="x",
            elem_a="carriage_body",
            elem_b="handle_shell",
            min_overlap=0.020,
            name="extended carriage still has retained insertion",
        )

    ctx.check(
        "blade carriage slides toward the nose",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.020,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )
    ctx.check(
        "blade protrudes past the nose when extended",
        handle_aabb is not None
        and extended_blade is not None
        and extended_blade[1][0] > handle_aabb[1][0] + 0.006,
        details=f"handle={handle_aabb}, blade={extended_blade}",
    )
    ctx.check(
        "guard flap swings forward from the opening",
        rest_guard is not None
        and open_guard is not None
        and open_guard[1][0] > rest_guard[1][0] + 0.004,
        details=f"closed={rest_guard}, open={open_guard}",
    )

    return ctx.report()


object_model = build_object_model()
