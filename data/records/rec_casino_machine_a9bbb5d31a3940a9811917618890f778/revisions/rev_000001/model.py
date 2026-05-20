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


SHELF_TOP_Z = 0.276
BUTTON_TRAVEL = 0.006
TRAY_HINGE_XYZ = (0.0, -0.207, 0.085)
TRAY_OPEN_ANGLE = 1.05


def _base_with_recess() -> cq.Workplane:
    """Lower cabinet block with a blind, real cash tray pocket cut into it."""
    width = 0.56
    depth = 0.38
    height = 0.235
    front_y = -depth / 2.0

    body = cq.Workplane("XY").box(width, depth, height).translate((0.0, 0.0, height / 2.0))

    # A blind rectangular subtraction from the front creates the actual tray
    # sidewalls, top, bottom, and back instead of a painted-on rectangle.
    pocket = (
        cq.Workplane("XY")
        .box(0.34, 0.110, 0.092)
        .translate((0.0, front_y + 0.110 / 2.0 - 0.014, 0.126))
    )
    return body.cut(pocket)


def _button_ring() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.033).circle(0.0255).extrude(0.006)


def _rx(local: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = local
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (x, y * ca - z * sa, y * sa + z * ca)


def _panel_point(
    center: tuple[float, float, float],
    angle: float,
    local: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx, dy, dz = _rx(local, angle)
    return (center[0] + dx, center[1] + dy, center[2] + dz)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_top_video_poker")

    shell = model.material("warm_black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    side = model.material("dark_burgundy_side", rgba=(0.18, 0.020, 0.035, 1.0))
    shelf_mat = model.material("satin_control_shelf", rgba=(0.055, 0.052, 0.050, 1.0))
    glass = model.material("blue_black_glass", rgba=(0.015, 0.040, 0.070, 1.0))
    screen_blue = model.material("screen_blue", rgba=(0.05, 0.30, 0.90, 1.0))
    screen_green = model.material("screen_green", rgba=(0.05, 0.80, 0.28, 1.0))
    screen_gold = model.material("screen_gold", rgba=(1.00, 0.68, 0.08, 1.0))
    tray_mat = model.material("deep_tray_black", rgba=(0.005, 0.005, 0.006, 1.0))
    hinge_mat = model.material("brushed_hinge_pin", rgba=(0.58, 0.55, 0.50, 1.0))
    red = model.material("red_play_button", rgba=(0.86, 0.04, 0.025, 1.0))
    amber = model.material("amber_play_button", rgba=(1.0, 0.48, 0.03, 1.0))
    green = model.material("green_play_button", rgba=(0.05, 0.72, 0.16, 1.0))
    blue = model.material("blue_play_button", rgba=(0.05, 0.18, 0.85, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_base_with_recess(), "cabinet_base", tolerance=0.001),
        material=shell,
        name="cabinet_base",
    )

    # Short horizontal button shelf protruding from the shallow countertop body.
    cabinet.visual(
        Box((0.54, 0.205, 0.044)),
        origin=Origin(xyz=(0.0, -0.125, 0.254)),
        material=shelf_mat,
        name="control_shelf",
    )
    cabinet.visual(
        Box((0.54, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.236, 0.290)),
        material=shell,
        name="front_shelf_lip",
    )

    # Leaning upper display pod: a shallow slab attached to the shelf and base.
    panel_angle = -0.365
    panel_center = (0.0, 0.006, 0.420)
    cabinet.visual(
        Box((0.55, 0.055, 0.290)),
        origin=Origin(xyz=panel_center, rpy=(panel_angle, 0.0, 0.0)),
        material=side,
        name="sloped_screen_pod",
    )

    # One dark screen with a raised frame and a few thin "lit" UI strips.
    cabinet.visual(
        Box((0.405, 0.006, 0.180)),
        origin=Origin(
            xyz=_panel_point(panel_center, panel_angle, (0.0, -0.0295, 0.020)),
            rpy=(panel_angle, 0.0, 0.0),
        ),
        material=glass,
        name="screen_glass",
    )
    for name, local, size in (
        ("screen_bezel_top", (0.0, -0.031, 0.132), (0.455, 0.014, 0.030)),
        ("screen_bezel_bottom", (0.0, -0.031, -0.094), (0.455, 0.014, 0.030)),
        ("screen_bezel_side_0", (-0.230, -0.031, 0.020), (0.030, 0.014, 0.224)),
        ("screen_bezel_side_1", (0.230, -0.031, 0.020), (0.030, 0.014, 0.224)),
    ):
        cabinet.visual(
            Box(size),
            origin=Origin(xyz=_panel_point(panel_center, panel_angle, local), rpy=(panel_angle, 0.0, 0.0)),
            material=shell,
            name=name,
        )
    for name, local, size, mat in (
        ("screen_title_bar", (0.0, -0.0335, 0.080), (0.310, 0.004, 0.018), screen_blue),
        ("screen_card_row", (-0.070, -0.0335, 0.020), (0.185, 0.004, 0.040), screen_gold),
        ("screen_credit_bar", (0.065, -0.0335, -0.052), (0.255, 0.004, 0.016), screen_green),
    ):
        cabinet.visual(
            Box(size),
            origin=Origin(xyz=_panel_point(panel_center, panel_angle, local), rpy=(panel_angle, 0.0, 0.0)),
            material=mat,
            name=name,
        )

    # Recess interior treatment: floor and back wall are set well behind the
    # front face, so the cash-out area reads as a true pocket.
    cabinet.visual(
        Box((0.310, 0.076, 0.006)),
        origin=Origin(xyz=(0.0, -0.143, 0.083)),
        material=tray_mat,
        name="tray_floor",
    )
    cabinet.visual(
        Box((0.310, 0.006, 0.070)),
        origin=Origin(xyz=(0.0, -0.097, 0.126)),
        material=tray_mat,
        name="tray_back",
    )
    cabinet.visual(
        Box((0.024, 0.024, 0.030)),
        origin=Origin(xyz=(-0.184, -0.197, 0.084)),
        material=shell,
        name="hinge_bracket_0",
    )
    cabinet.visual(
        Box((0.024, 0.024, 0.030)),
        origin=Origin(xyz=(0.184, -0.197, 0.084)),
        material=shell,
        name="hinge_bracket_1",
    )

    ring_mesh = mesh_from_cadquery(_button_ring(), "button_ring", tolerance=0.0008)
    button_xs = (-0.180, -0.090, 0.0, 0.090, 0.180)
    button_mats = (red, amber, green, blue, red)
    for i, x in enumerate(button_xs):
        cabinet.visual(
            ring_mesh,
            origin=Origin(xyz=(x, -0.158, SHELF_TOP_Z)),
            material=tray_mat,
            name=f"button_ring_{i}",
        )

    tray_flap = model.part("tray_flap")
    tray_flap.visual(
        Box((0.300, 0.010, 0.068)),
        origin=Origin(xyz=(0.0, 0.011, 0.034)),
        material=shelf_mat,
        name="flap_panel",
    )
    tray_flap.visual(
        Cylinder(radius=0.007, length=0.344),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="flap_hinge_barrel",
    )
    tray_flap.visual(
        Box((0.120, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.010, 0.056)),
        material=tray_mat,
        name="flap_pull_lip",
    )
    model.articulation(
        "cabinet_to_tray_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=tray_flap,
        origin=Origin(xyz=TRAY_HINGE_XYZ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=0.0, upper=TRAY_OPEN_ANGLE),
    )

    for i, (x, mat) in enumerate(zip(button_xs, button_mats)):
        button = model.part(f"button_{i}")
        button.visual(
            Cylinder(radius=0.023, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=tray_mat,
            name="button_stem",
        )
        model.articulation(
            f"cabinet_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, -0.158, SHELF_TOP_Z + 0.006)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=-BUTTON_TRAVEL, upper=0.0),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def coord(vec, index: int) -> float:
        try:
            return float(vec[index])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[index])

    cabinet = object_model.get_part("cabinet")
    tray_flap = object_model.get_part("tray_flap")
    tray_joint = object_model.get_articulation("cabinet_to_tray_flap")

    button_joints = [object_model.get_articulation(f"cabinet_to_button_{i}") for i in range(5)]
    ctx.check(
        "five independent prismatic play buttons",
        len(button_joints) == 5
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in button_joints)
        and len({j.child for j in button_joints}) == 5,
        details=f"button_joints={[j.name for j in button_joints]}",
    )

    for i, joint in enumerate(button_joints):
        button = object_model.get_part(f"button_{i}")
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: -BUTTON_TRAVEL}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{i} depresses downward",
            rest is not None and pressed is not None and pressed[2] < rest[2] - 0.004,
            details=f"rest={rest}, pressed={pressed}",
        )

    ctx.check(
        "cash tray flap uses bottom hinge",
        tray_joint.articulation_type == ArticulationType.REVOLUTE
        and tray_joint.axis == (1.0, 0.0, 0.0)
        and tray_joint.motion_limits is not None
        and tray_joint.motion_limits.lower == 0.0
        and tray_joint.motion_limits.upper is not None
        and tray_joint.motion_limits.upper > 0.8,
        details=f"axis={tray_joint.axis}, limits={tray_joint.motion_limits}",
    )

    closed_flap = ctx.part_element_world_aabb(tray_flap, elem="flap_panel")
    with ctx.pose({tray_joint: TRAY_OPEN_ANGLE}):
        open_flap = ctx.part_element_world_aabb(tray_flap, elem="flap_panel")
    ctx.check(
        "tray flap swings outward and downward",
        closed_flap is not None
        and open_flap is not None
        and coord(open_flap[0], 1) < coord(closed_flap[0], 1) - 0.025
        and coord(open_flap[1], 2) < coord(closed_flap[1], 2) - 0.020,
        details=f"closed={closed_flap}, open={open_flap}",
    )

    base_box = ctx.part_element_world_aabb(cabinet, elem="cabinet_base")
    tray_back = ctx.part_element_world_aabb(cabinet, elem="tray_back")
    tray_floor = ctx.part_element_world_aabb(cabinet, elem="tray_floor")
    ctx.check(
        "cash tray is a recessed pocket",
        base_box is not None
        and tray_back is not None
        and tray_floor is not None
        and coord(tray_back[0], 1) > coord(base_box[0], 1) + 0.085
        and coord(tray_floor[1], 2) < SHELF_TOP_Z - 0.15,
        details=f"base={base_box}, back={tray_back}, floor={tray_floor}",
    )

    cabinet_box = ctx.part_world_aabb(cabinet)
    ctx.check(
        "bar-top gaming terminal scale",
        cabinet_box is not None
        and 0.48 <= coord(cabinet_box[1], 0) - coord(cabinet_box[0], 0) <= 0.70
        and 0.42 <= coord(cabinet_box[1], 2) - coord(cabinet_box[0], 2) <= 0.65,
        details=f"cabinet_box={cabinet_box}",
    )

    return ctx.report()


object_model = build_object_model()
