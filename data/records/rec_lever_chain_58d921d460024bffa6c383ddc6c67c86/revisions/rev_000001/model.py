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


PLATE_THICKNESS = 0.006
PIN_MID_Z = 0.056
BOTTOM_LAYER_Z = 0.050
TOP_LAYER_Z = 0.062
PIN_RADIUS = 0.0045
PIN_LENGTH = 0.026


def _capsule_link(
    dx: float,
    dy: float,
    *,
    outer_radius: float = 0.014,
    hole_radius: float | None = 0.0068,
    hole_at_start: bool = False,
    hole_at_end: bool = True,
) -> cq.Workplane:
    """Flat rounded lever link from the local joint origin to an offset pin."""

    distance = math.hypot(dx, dy)
    ux, uy = dx / distance, dy / distance
    nx, ny = -uy, ux
    r = outer_radius

    start = (0.0, 0.0)
    end = (dx, dy)
    outline = (
        cq.Workplane("XY")
        .moveTo(start[0] + nx * r, start[1] + ny * r)
        .lineTo(end[0] + nx * r, end[1] + ny * r)
        .threePointArc((end[0] + ux * r, end[1] + uy * r), (end[0] - nx * r, end[1] - ny * r))
        .lineTo(start[0] - nx * r, start[1] - ny * r)
        .threePointArc((start[0] - ux * r, start[1] - uy * r), (start[0] + nx * r, start[1] + ny * r))
        .close()
    )
    body = outline.extrude(PLATE_THICKNESS).translate((0.0, 0.0, -PLATE_THICKNESS / 2.0))

    if hole_radius is not None:
        hole_points: list[tuple[float, float]] = []
        if hole_at_start:
            hole_points.append(start)
        if hole_at_end:
            hole_points.append(end)
        if hole_points:
            cutters = (
                cq.Workplane("XY")
                .pushPoints(hole_points)
                .circle(hole_radius)
                .extrude(PLATE_THICKNESS * 4.0)
                .translate((0.0, 0.0, -PLATE_THICKNESS * 2.0))
            )
            body = body.cut(cutters)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_lever_chain")

    painted_blue = model.material("painted_blue", color=(0.12, 0.26, 0.72, 1.0))
    painted_orange = model.material("painted_orange", color=(0.88, 0.40, 0.08, 1.0))
    dark_steel = model.material("dark_steel", color=(0.12, 0.12, 0.13, 1.0))
    brass = model.material("brass_pins", color=(0.86, 0.62, 0.24, 1.0))

    root_vec = (0.090, 0.018)
    link_1_vec = (0.078, -0.030)
    link_2_vec = (0.075, 0.028)
    link_3_vec = (0.060, -0.018)

    root = model.part("root_bracket")
    root.visual(
        Box((0.115, 0.076, 0.008)),
        origin=Origin(xyz=(0.006, 0.000, 0.004)),
        material=dark_steel,
        name="base_plate",
    )
    root.visual(
        Box((0.032, 0.052, 0.041)),
        origin=Origin(xyz=(-0.005, 0.000, 0.0285)),
        material=dark_steel,
        name="pedestal",
    )
    # Grounded first lever: the inboard end is fixed into the bracket, while the
    # outboard hole carries the first moving link's pin.
    root.visual(
        mesh_from_cadquery(
            _capsule_link(*root_vec, outer_radius=0.014, hole_at_start=False, hole_at_end=True),
            "link_0_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_LAYER_Z)),
        material=painted_blue,
        name="link_0_plate",
    )
    root.visual(
        Cylinder(radius=0.0065, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, PIN_MID_Z)),
        material=brass,
        name="fixed_pivot_pin",
    )
    for i, (x, y) in enumerate([(-0.030, -0.024), (-0.030, 0.024), (0.040, -0.024), (0.040, 0.024)]):
        root.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(xyz=(x, y, 0.0095)),
            material=brass,
            name=f"mount_bolt_{i}",
        )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(
            _capsule_link(*link_1_vec, outer_radius=0.013, hole_at_start=False, hole_at_end=True),
            "link_1_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, TOP_LAYER_Z - PIN_MID_Z)),
        material=painted_orange,
        name="link_1_plate",
    )
    link_1.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(),
        material=brass,
        name="joint_pin",
    )
    link_1.visual(
        Cylinder(radius=0.0082, length=0.0063),
        origin=Origin(xyz=(0.0, 0.0, 0.00015)),
        material=brass,
        name="bearing_spacer",
    )
    link_1.visual(
        Cylinder(radius=0.0078, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=brass,
        name="upper_washer",
    )
    link_1.visual(
        Cylinder(radius=0.0078, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=brass,
        name="lower_washer",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(
            _capsule_link(*link_2_vec, outer_radius=0.0125, hole_at_start=False, hole_at_end=True),
            "link_2_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_LAYER_Z - PIN_MID_Z)),
        material=painted_blue,
        name="link_2_plate",
    )
    link_2.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(),
        material=brass,
        name="joint_pin",
    )
    link_2.visual(
        Cylinder(radius=0.0082, length=0.0063),
        origin=Origin(xyz=(0.0, 0.0, -0.00015)),
        material=brass,
        name="bearing_spacer",
    )
    link_2.visual(
        Cylinder(radius=0.0078, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=brass,
        name="upper_washer",
    )
    link_2.visual(
        Cylinder(radius=0.0078, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=brass,
        name="lower_washer",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(
            _capsule_link(*link_3_vec, outer_radius=0.0105, hole_radius=0.0038, hole_at_start=False, hole_at_end=True),
            "link_3_tab",
        ),
        origin=Origin(xyz=(0.0, 0.0, TOP_LAYER_Z - PIN_MID_Z)),
        material=painted_orange,
        name="link_3_tab",
    )
    link_3.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(),
        material=brass,
        name="joint_pin",
    )
    link_3.visual(
        Cylinder(radius=0.0082, length=0.0063),
        origin=Origin(xyz=(0.0, 0.0, 0.00015)),
        material=brass,
        name="bearing_spacer",
    )
    link_3.visual(
        Cylinder(radius=0.0078, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=brass,
        name="upper_washer",
    )
    link_3.visual(
        Cylinder(radius=0.0078, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=brass,
        name="lower_washer",
    )

    limits = MotionLimits(effort=8.0, velocity=4.0, lower=-1.15, upper=1.15)
    model.articulation(
        "root_to_link_1",
        ArticulationType.REVOLUTE,
        parent=root,
        child=link_1,
        origin=Origin(xyz=(root_vec[0], root_vec[1], PIN_MID_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(link_1_vec[0], link_1_vec[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(link_2_vec[0], link_2_vec[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    joint_0 = object_model.get_articulation("root_to_link_1")
    joint_1 = object_model.get_articulation("link_1_to_link_2")
    joint_2 = object_model.get_articulation("link_2_to_link_3")

    revolute_joints = [joint_0, joint_1, joint_2]
    ctx.check(
        "three serial revolute joints",
        len(revolute_joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in revolute_joints)
        and all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in revolute_joints),
        details=f"joints={[(j.name, j.articulation_type, j.axis) for j in revolute_joints]}",
    )

    ctx.expect_overlap(
        root,
        link_1,
        axes="xy",
        elem_a="link_0_plate",
        elem_b="joint_pin",
        min_overlap=0.006,
        name="first pin sits in root link eye",
    )
    ctx.expect_overlap(
        link_1,
        link_2,
        axes="xy",
        elem_a="link_1_plate",
        elem_b="joint_pin",
        min_overlap=0.006,
        name="second pin sits in link eye",
    )
    ctx.expect_overlap(
        link_2,
        link_3,
        axes="xy",
        elem_a="link_2_plate",
        elem_b="joint_pin",
        min_overlap=0.006,
        name="third pin sits in link eye",
    )

    rest_tip = ctx.part_world_position(link_3)
    with ctx.pose({joint_0: 0.55, joint_1: -0.45, joint_2: 0.35}):
        moved_tip = ctx.part_world_position(link_3)
    ctx.check(
        "end tab follows planar chain",
        rest_tip is not None
        and moved_tip is not None
        and abs(moved_tip[2] - rest_tip[2]) < 1e-6
        and math.hypot(moved_tip[0] - rest_tip[0], moved_tip[1] - rest_tip[1]) > 0.025,
        details=f"rest={rest_tip}, moved={moved_tip}",
    )

    return ctx.report()


object_model = build_object_model()
