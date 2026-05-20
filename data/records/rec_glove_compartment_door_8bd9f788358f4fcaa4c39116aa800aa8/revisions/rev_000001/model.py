from __future__ import annotations

import math

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
import cadquery as cq


DOOR_TRAVEL = math.radians(70.0)


def _dashboard_and_tub_shape() -> cq.Workplane:
    """Dashboard face with a fixed, hollow glove-box storage tub behind it."""

    dash_thickness = 0.045
    dash_width = 0.80
    dash_height = 0.48
    opening_width = 0.58
    opening_height = 0.32

    dash = cq.Workplane("XY").box(dash_thickness, dash_width, dash_height)
    opening_cutter = cq.Workplane("XY").box(
        dash_thickness + 0.04,
        opening_width,
        opening_height,
    )
    dash = dash.cut(opening_cutter)

    tub_depth = 0.325
    tub_width = 0.62
    tub_height = 0.36
    wall = 0.025
    front_x = -0.015
    back_x = front_x - tub_depth
    tub_center_x = (front_x + back_x) / 2.0

    outer_tub = cq.Workplane("XY").box(tub_depth, tub_width, tub_height).translate(
        (tub_center_x, 0.0, 0.0)
    )
    cavity = cq.Workplane("XY").box(
        tub_depth + 0.06,
        tub_width - 2.0 * wall,
        tub_height - 2.0 * wall,
    ).translate(((back_x + wall + 0.04) / 2.0, 0.0, 0.0))
    tub = outer_tub.cut(cavity)

    return dash.union(tub).edges("|X").fillet(0.006)


def _curved_door_panel_shape() -> cq.Workplane:
    """Convex glove-box door skin in the door local frame.

    The door frame origin is on the lower hinge axis.  The panel extends upward
    along +Z and bows outward along +X.
    """

    width = 0.62
    height = 0.35
    inner_x = 0.002
    skin = 0.030
    crown = 0.022

    profile: list[tuple[float, float]] = [(inner_x, 0.0), (inner_x, height)]
    for i in range(18, -1, -1):
        z = height * i / 18.0
        bulge = crown * math.sin(math.pi * z / height)
        profile.append((inner_x + skin + bulge, z))

    panel = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(width)
        .translate((0.0, width / 2.0, 0.0))
    )
    return panel


def _side_hinge_arm_shape(y_center: float) -> cq.Workplane:
    """Thin side hinge arm/cheek plate attached to one lower door corner."""

    thickness_y = 0.020
    side_plate = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.012, -0.014),
                (0.074, 0.010),
                (0.060, 0.118),
                (0.020, 0.118),
                (0.010, 0.020),
            ]
        )
        .close()
        .extrude(thickness_y)
        .translate((0.0, y_center - thickness_y / 2.0, 0.0))
    )
    return side_plate.edges("|Y").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="passenger_glove_box")

    dash_material = Material("soft_dark_dashboard", color=(0.06, 0.065, 0.07, 1.0))
    tub_material = Material("black_flocked_tub", color=(0.015, 0.015, 0.018, 1.0))
    door_material = Material("warm_gray_curved_door", color=(0.23, 0.22, 0.20, 1.0))
    hinge_material = Material("satin_black_hinge", color=(0.005, 0.005, 0.006, 1.0))
    seam_material = Material("shadow_gap", color=(0.0, 0.0, 0.0, 1.0))

    dashboard = model.part("dashboard")
    dashboard.visual(
        mesh_from_cadquery(_dashboard_and_tub_shape(), "dashboard_tub"),
        material=dash_material,
        name="dashboard_tub",
    )
    dashboard.visual(
        Box((0.008, 0.566, 0.306)),
        origin=Origin(xyz=(-0.328, 0.0, 0.0)),
        material=tub_material,
        name="tub_back_liner",
    )
    dashboard.visual(
        Box((0.310, 0.566, 0.008)),
        origin=Origin(xyz=(-0.174, 0.0, -0.156)),
        material=tub_material,
        name="tub_floor_liner",
    )
    dashboard.visual(
        Box((0.310, 0.566, 0.008)),
        origin=Origin(xyz=(-0.174, 0.0, 0.156)),
        material=tub_material,
        name="tub_top_liner",
    )
    dashboard.visual(
        Box((0.310, 0.008, 0.306)),
        origin=Origin(xyz=(-0.174, 0.286, 0.0)),
        material=tub_material,
        name="tub_side_liner_0",
    )
    dashboard.visual(
        Box((0.310, 0.008, 0.306)),
        origin=Origin(xyz=(-0.174, -0.286, 0.0)),
        material=tub_material,
        name="tub_side_liner_1",
    )
    dashboard.visual(
        Box((0.040, 0.68, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.223)),
        material=hinge_material,
        name="lower_hinge_rail",
    )
    dashboard.visual(
        Box((0.056, 0.032, 0.074)),
        origin=Origin(xyz=(0.028, 0.344, -0.180)),
        material=hinge_material,
        name="near_hinge_bracket",
    )
    dashboard.visual(
        Cylinder(radius=0.014, length=0.025),
        origin=Origin(xyz=(0.050, 0.332, -0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_material,
        name="near_pin_cap",
    )
    dashboard.visual(
        Box((0.056, 0.032, 0.074)),
        origin=Origin(xyz=(0.028, -0.344, -0.180)),
        material=hinge_material,
        name="far_hinge_bracket",
    )
    dashboard.visual(
        Cylinder(radius=0.014, length=0.025),
        origin=Origin(xyz=(0.050, -0.332, -0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_material,
        name="far_pin_cap",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_curved_door_panel_shape(), "curved_door_panel"),
        material=door_material,
        name="curved_panel",
    )
    door.visual(
        Box((0.010, 0.54, 0.018)),
        origin=Origin(xyz=(0.006, 0.0, 0.336)),
        material=seam_material,
        name="upper_shadow_seal",
    )
    door.visual(
        Box((0.010, 0.012, 0.285)),
        origin=Origin(xyz=(0.006, 0.286, 0.170)),
        material=seam_material,
        name="side_shadow_seal_0",
    )
    door.visual(
        Box((0.010, 0.012, 0.285)),
        origin=Origin(xyz=(0.006, -0.286, 0.170)),
        material=seam_material,
        name="side_shadow_seal_1",
    )
    door.visual(
        Box((0.020, 0.24, 0.028)),
        origin=Origin(xyz=(0.047, 0.0, 0.285)),
        material=seam_material,
        name="pull_recess",
    )
    for side, y in (("hinge_arm_0", 0.306), ("hinge_arm_1", -0.306)):
        door.visual(
            mesh_from_cadquery(_side_hinge_arm_shape(y), side),
            material=hinge_material,
            name=side,
        )
    door.visual(
        Cylinder(radius=0.017, length=0.052),
        origin=Origin(xyz=(0.0, 0.306, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_material,
        name="hinge_barrel_0",
    )
    door.visual(
        Cylinder(radius=0.017, length=0.052),
        origin=Origin(xyz=(0.0, -0.306, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_material,
        name="hinge_barrel_1",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=dashboard,
        child=door,
        origin=Origin(xyz=(0.050, 0.0, -0.185)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=DOOR_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dashboard = object_model.get_part("dashboard")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    ctx.allow_overlap(
        dashboard,
        door,
        elem_a="near_pin_cap",
        elem_b="hinge_barrel_0",
        reason="The fixed hinge pin is intentionally captured inside the moving door barrel.",
    )
    ctx.allow_overlap(
        dashboard,
        door,
        elem_a="far_pin_cap",
        elem_b="hinge_barrel_1",
        reason="The fixed hinge pin is intentionally captured inside the moving door barrel.",
    )

    ctx.check(
        "door hinge travel is about 70 degrees",
        abs(hinge.motion_limits.upper - DOOR_TRAVEL) < math.radians(1.0),
        details=f"upper={hinge.motion_limits.upper}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            dashboard,
            axis="x",
            min_gap=0.001,
            max_gap=0.030,
            positive_elem="curved_panel",
            negative_elem="dashboard_tub",
            name="closed door sits just proud of dashboard face",
        )
        ctx.expect_overlap(
            door,
            dashboard,
            axes="yz",
            min_overlap=0.25,
            elem_a="curved_panel",
            elem_b="dashboard_tub",
            name="closed door covers the glove box opening",
        )
        ctx.expect_overlap(
            dashboard,
            door,
            axes="xyz",
            min_overlap=0.005,
            elem_a="near_pin_cap",
            elem_b="hinge_barrel_0",
            name="near hinge pin is captured in barrel",
        )
        ctx.expect_overlap(
            dashboard,
            door,
            axes="xyz",
            min_overlap=0.005,
            elem_a="far_pin_cap",
            elem_b="hinge_barrel_1",
            name="far hinge pin is captured in barrel",
        )
        closed_aabb = ctx.part_element_world_aabb(door, elem="curved_panel")

    with ctx.pose({hinge: DOOR_TRAVEL}):
        open_aabb = ctx.part_element_world_aabb(door, elem="curved_panel")
        ctx.expect_gap(
            door,
            dashboard,
            axis="x",
            min_gap=0.020,
            positive_elem="curved_panel",
            negative_elem="dashboard_tub",
            name="opened door skin clears dashboard face",
        )
        ctx.expect_overlap(
            door,
            dashboard,
            axes="y",
            min_overlap=0.005,
            elem_a="hinge_barrel_0",
            elem_b="near_pin_cap",
            name="side hinge hardware remains aligned",
        )

    if closed_aabb is not None and open_aabb is not None:
        closed_center_x = (closed_aabb[0][0] + closed_aabb[1][0]) / 2.0
        open_center_x = (open_aabb[0][0] + open_aabb[1][0]) / 2.0
        closed_top_z = closed_aabb[1][2]
        open_top_z = open_aabb[1][2]
        ctx.check(
            "door swings outward and downward",
            open_center_x > closed_center_x + 0.11 and open_top_z < closed_top_z - 0.11,
            details=(
                f"closed_center_x={closed_center_x}, open_center_x={open_center_x}, "
                f"closed_top_z={closed_top_z}, open_top_z={open_top_z}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
