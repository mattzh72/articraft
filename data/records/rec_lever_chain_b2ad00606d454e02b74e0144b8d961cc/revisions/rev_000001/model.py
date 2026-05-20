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


def _plate_body(
    length: float,
    width: float,
    thickness: float,
    *,
    corner_radius: float,
    hole_centers: tuple[tuple[float, float], ...] = (),
    hole_radius: float = 0.0,
) -> cq.Workplane:
    """Flat rounded mounting plate, z=0..thickness in local coordinates."""
    body = (
        cq.Workplane("XY")
        .rect(length, width)
        .extrude(thickness)
        .edges("|Z")
        .fillet(corner_radius)
    )
    if hole_centers and hole_radius > 0.0:
        cutters = (
            cq.Workplane("XY")
            .pushPoints(list(hole_centers))
            .circle(hole_radius)
            .extrude(thickness + 0.004)
            .translate((0.0, 0.0, -0.002))
        )
        body = body.cut(cutters)
    return body


def _screw_head(radius: float, height: float, slot_width: float) -> cq.Workplane:
    """Low pan-head screw with a shallow straight slot."""
    head = cq.Workplane("XY").circle(radius).extrude(height)
    slot = (
        cq.Workplane("XY")
        .box(radius * 1.55, slot_width, height * 0.70, centered=(True, True, False))
        .translate((0.0, 0.0, height * 0.55))
    )
    return head.cut(slot)


def _capsule_link(
    dx: float,
    dy: float,
    *,
    width: float,
    web_width: float,
    thickness: float,
    hole_radius_start: float | None = None,
    hole_radius_end: float | None = None,
) -> cq.Workplane:
    """A flat dog-bone link from local (0,0) to (dx,dy), z=0..thickness."""
    span = math.hypot(dx, dy)
    angle_deg = math.degrees(math.atan2(dy, dx))
    end_radius = width * 0.5

    web = cq.Workplane("XY").center(span * 0.5, 0.0).rect(span, web_width).extrude(thickness)
    start_boss = cq.Workplane("XY").circle(end_radius).extrude(thickness)
    end_boss = cq.Workplane("XY").center(span, 0.0).circle(end_radius).extrude(thickness)
    body = web.union(start_boss).union(end_boss)

    cutters: list[cq.Workplane] = []
    if hole_radius_start is not None and hole_radius_start > 0.0:
        cutters.append(
            cq.Workplane("XY")
            .circle(hole_radius_start)
            .extrude(thickness + 0.004)
            .translate((0.0, 0.0, -0.002))
        )
    if hole_radius_end is not None and hole_radius_end > 0.0:
        cutters.append(
            cq.Workplane("XY")
            .center(span, 0.0)
            .circle(hole_radius_end)
            .extrude(thickness + 0.004)
            .translate((0.0, 0.0, -0.002))
        )
    for cutter in cutters:
        body = body.cut(cutter)

    if abs(angle_deg) > 1.0e-9:
        body = body.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_lever_chain")

    black_oxide = model.material("black_oxide", rgba=(0.015, 0.016, 0.017, 1.0))
    dark_pin = model.material("dark_pins", rgba=(0.05, 0.052, 0.055, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    satin_tab = model.material("satin_tab", rgba=(0.48, 0.52, 0.54, 1.0))

    plate_length = 0.250
    plate_width = 0.105
    plate_thickness = 0.008
    link_thickness = 0.006
    tab_thickness = 0.005
    hinge_pin_radius = 0.0075
    link_hole_radius = 0.0090
    washer_radius = 0.0175
    washer_thickness = 0.002

    root_joint_xy = (0.085, 0.0)
    link_0_tip = (0.190, 0.022)
    link_1_tip = (0.165, -0.026)
    tab_tip = (0.082, 0.010)

    screw_points = (
        (-0.085, -0.033),
        (-0.085, 0.033),
        (0.030, -0.033),
        (0.030, 0.033),
    )

    plate_mesh = mesh_from_cadquery(
        _plate_body(
            plate_length,
            plate_width,
            plate_thickness,
            corner_radius=0.010,
            hole_centers=screw_points,
            hole_radius=0.006,
        ),
        "mounting_plate_shell",
        tolerance=0.0007,
    )
    screw_mesh = mesh_from_cadquery(_screw_head(0.011, 0.0025, 0.0028), "slotted_screw_head")
    link_0_mesh = mesh_from_cadquery(
        _capsule_link(
            *link_0_tip,
            width=0.046,
            web_width=0.026,
            thickness=link_thickness,
            hole_radius_start=link_hole_radius,
            hole_radius_end=None,
        ),
        "offset_link_0_body",
        tolerance=0.0007,
    )
    link_1_mesh = mesh_from_cadquery(
        _capsule_link(
            *link_1_tip,
            width=0.041,
            web_width=0.023,
            thickness=link_thickness,
            hole_radius_start=link_hole_radius,
            hole_radius_end=None,
        ),
        "offset_link_1_body",
        tolerance=0.0007,
    )
    tab_mesh = mesh_from_cadquery(
        _capsule_link(
            *tab_tip,
            width=0.034,
            web_width=0.020,
            thickness=tab_thickness,
            hole_radius_start=link_hole_radius,
            hole_radius_end=0.0050,
        ),
        "distal_end_tab_body",
        tolerance=0.0007,
    )

    plate = model.part("mounting_plate")
    plate.visual(plate_mesh, name="plate_shell", material=black_oxide)
    for index, (sx, sy) in enumerate(screw_points):
        plate.visual(
            screw_mesh,
            origin=Origin(xyz=(sx, sy, plate_thickness)),
            name=f"screw_{index}",
            material=dark_pin,
        )
    plate.visual(
        Cylinder(radius=0.019, length=0.0014),
        origin=Origin(xyz=(root_joint_xy[0], root_joint_xy[1], plate_thickness + 0.0007)),
        name="root_lower_boss",
        material=dark_pin,
    )
    plate.visual(
        Cylinder(radius=hinge_pin_radius, length=0.010),
        origin=Origin(xyz=(root_joint_xy[0], root_joint_xy[1], 0.013)),
        name="root_pin",
        material=dark_pin,
    )
    plate.visual(
        Cylinder(radius=washer_radius, length=washer_thickness),
        origin=Origin(xyz=(root_joint_xy[0], root_joint_xy[1], 0.017)),
        name="root_pin_cap",
        material=dark_pin,
    )

    link_0 = model.part("link_0")
    link_0.visual(link_0_mesh, name="link_0_body", material=brushed_steel)
    link_0.visual(
        Cylinder(radius=hinge_pin_radius, length=0.010),
        origin=Origin(xyz=(link_0_tip[0], link_0_tip[1], 0.011)),
        name="link_0_pin",
        material=dark_pin,
    )
    link_0.visual(
        Cylinder(radius=washer_radius, length=washer_thickness),
        origin=Origin(xyz=(link_0_tip[0], link_0_tip[1], 0.015)),
        name="link_0_pin_cap",
        material=dark_pin,
    )

    link_1 = model.part("link_1")
    link_1.visual(link_1_mesh, name="link_1_body", material=brushed_steel)
    link_1.visual(
        Cylinder(radius=hinge_pin_radius, length=0.010),
        origin=Origin(xyz=(link_1_tip[0], link_1_tip[1], 0.0105)),
        name="link_1_pin",
        material=dark_pin,
    )
    link_1.visual(
        Cylinder(radius=0.015, length=washer_thickness),
        origin=Origin(xyz=(link_1_tip[0], link_1_tip[1], 0.014)),
        name="link_1_pin_cap",
        material=dark_pin,
    )

    end_tab = model.part("end_tab")
    end_tab.visual(tab_mesh, name="end_tab_body", material=satin_tab)

    model.articulation(
        "plate_to_link_0",
        ArticulationType.REVOLUTE,
        parent=plate,
        child=link_0,
        origin=Origin(xyz=(root_joint_xy[0], root_joint_xy[1], 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(link_0_tip[0], link_0_tip[1], 0.008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "link_1_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=end_tab,
        origin=Origin(xyz=(link_1_tip[0], link_1_tip[1], 0.008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.2, lower=-1.40, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plate = object_model.get_part("mounting_plate")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    end_tab = object_model.get_part("end_tab")
    j0 = object_model.get_articulation("plate_to_link_0")
    j1 = object_model.get_articulation("link_0_to_link_1")
    j2 = object_model.get_articulation("link_1_to_end_tab")

    ctx.check(
        "three planar revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations)
        and all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in object_model.articulations),
        details="The lever chain must bend through three Z-axis revolute pivots.",
    )
    ctx.expect_gap(
        link_0,
        plate,
        axis="z",
        positive_elem="link_0_body",
        negative_elem="plate_shell",
        min_gap=0.001,
        max_gap=0.0035,
        name="first link rides just above the mounting plate",
    )
    ctx.expect_overlap(
        plate,
        link_0,
        axes="xy",
        elem_a="root_pin",
        elem_b="link_0_body",
        min_overlap=0.010,
        name="root pin passes through the first link knuckle",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="xy",
        elem_a="link_0_pin",
        elem_b="link_1_body",
        min_overlap=0.010,
        name="middle pin passes through the second link knuckle",
    )
    ctx.expect_overlap(
        link_1,
        end_tab,
        axes="xy",
        elem_a="link_1_pin",
        elem_b="end_tab_body",
        min_overlap=0.009,
        name="distal pin passes through the small end tab knuckle",
    )

    rest_pos = ctx.part_world_position(end_tab)
    with ctx.pose({j0: 0.55, j1: -0.75, j2: 0.45}):
        bent_pos = ctx.part_world_position(end_tab)
    ctx.check(
        "chain bends in the horizontal plane",
        rest_pos is not None
        and bent_pos is not None
        and abs(bent_pos[2] - rest_pos[2]) < 0.001
        and math.hypot(bent_pos[0] - rest_pos[0], bent_pos[1] - rest_pos[1]) > 0.035,
        details=f"rest={rest_pos}, bent={bent_pos}",
    )

    return ctx.report()


object_model = build_object_model()
