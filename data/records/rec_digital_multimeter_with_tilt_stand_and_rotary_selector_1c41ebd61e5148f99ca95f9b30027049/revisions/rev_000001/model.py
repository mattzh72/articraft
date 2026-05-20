from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_W = 0.086
BODY_T = 0.039
BODY_H = 0.170
BUMPER_W = 0.100
BUMPER_T = 0.056
BUMPER_H = 0.188
FRONT_FACE_Y = BODY_T / 2.0
BACK_FACE_Y = -BODY_T / 2.0
PANEL_Y = FRONT_FACE_Y - 0.0035
BUTTON_MOUNT_Y = PANEL_Y - 0.0004

SCREEN_Z = 0.050
BUTTON_Z = 0.012
SELECTOR_Z = -0.026
HINGE_Z = -0.073
HINGE_Y = -0.0243
BUTTON_X = (-0.0285, -0.0095, 0.0095, 0.0285)


def _case_shell_mesh():
    case = cq.Workplane("XY").box(BODY_W, BODY_T, BODY_H).edges().fillet(0.006)

    front_panel_recess = (
        cq.Workplane("XY")
        .box(0.074, 0.007, 0.136)
        .translate((0.0, FRONT_FACE_Y - 0.0035, -0.004))
    )
    screen_cut = (
        cq.Workplane("XY")
        .box(0.061, 0.010, 0.036)
        .translate((0.0, FRONT_FACE_Y - 0.005, SCREEN_Z))
    )
    back_stand_pocket = (
        cq.Workplane("XY")
        .box(0.056, 0.006, 0.108)
        .translate((0.0, BACK_FACE_Y + 0.003, 0.004))
    )

    case = case.cut(front_panel_recess).cut(screen_cut).cut(back_stand_pocket)

    for x in BUTTON_X:
        button_slot = (
            cq.Workplane("XY")
            .box(0.0092, 0.008, 0.0068)
            .translate((x, PANEL_Y - 0.004, BUTTON_Z))
        )
        case = case.cut(button_slot)

    return mesh_from_cadquery(case, "multimeter_case_shell")


def _bumper_mesh():
    outer = cq.Workplane("XY").box(BUMPER_W, BUMPER_T, BUMPER_H).edges().fillet(0.011)
    inner = cq.Workplane("XY").box(0.082, 0.040, 0.166).edges().fillet(0.008)
    front_window = (
        cq.Workplane("XY")
        .box(0.078, 0.040, 0.146)
        .translate((0.0, 0.012, 0.002))
    )
    back_window = (
        cq.Workplane("XY")
        .box(0.074, 0.040, 0.156)
        .translate((0.0, -0.012, -0.004))
    )
    bumper = outer.cut(inner).cut(front_window).cut(back_window)
    return mesh_from_cadquery(bumper, "multimeter_bumper")


def _stand_mesh():
    plate = (
        cq.Workplane("XY")
        .box(0.048, 0.003, 0.090, centered=(True, True, False))
        .translate((0.0, -0.0015, 0.006))
    )
    foot = (
        cq.Workplane("XY")
        .box(0.036, 0.008, 0.014, centered=(True, True, False))
        .translate((0.0, -0.0045, 0.082))
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.024, 0.006, 0.012, centered=(True, True, False))
        .translate((0.0, -0.0015, -0.006))
    )
    barrel = (
        cq.Workplane("YZ")
        .circle(0.0048)
        .extrude(0.011, both=True)
    )
    stand = plate.union(foot).union(bridge).union(barrel)
    return mesh_from_cadquery(stand, "multimeter_stand")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_multimeter")

    body_black = model.material("body_black", rgba=(0.16, 0.17, 0.18, 1.0))
    bumper_yellow = model.material("bumper_yellow", rgba=(0.90, 0.73, 0.14, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.05, 0.05, 0.06, 1.0))
    dial_gray = model.material("dial_gray", rgba=(0.23, 0.24, 0.25, 1.0))
    button_gray = model.material("button_gray", rgba=(0.69, 0.71, 0.74, 1.0))
    socket_black = model.material("socket_black", rgba=(0.07, 0.07, 0.08, 1.0))
    socket_red = model.material("socket_red", rgba=(0.73, 0.12, 0.10, 1.0))
    stand_gray = model.material("stand_gray", rgba=(0.22, 0.23, 0.25, 1.0))

    body = model.part("body")
    body.visual(_case_shell_mesh(), material=body_black, name="case_shell")
    body.visual(_bumper_mesh(), material=bumper_yellow, name="bumper")
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.062, 0.032),
                (0.078, 0.050),
                0.009,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.005,
                outer_corner_radius=0.008,
                center=False,
            ),
            "multimeter_display_bezel",
        ),
        origin=Origin(xyz=(0.0, FRONT_FACE_Y - 0.0003, SCREEN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bezel_black,
        name="display_bezel",
    )
    for index, (x, material_name) in enumerate(
        zip((-0.022, 0.0, 0.022), (socket_black, socket_black, socket_red))
    ):
        body.visual(
            Cylinder(radius=0.0074, length=0.005),
            origin=Origin(xyz=(x, FRONT_FACE_Y + 0.002, -0.071), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material_name,
            name=f"jack_ring_{index}",
        )
        body.visual(
            Cylinder(radius=0.0044, length=0.006),
            origin=Origin(xyz=(x, FRONT_FACE_Y + 0.0015, -0.071), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=socket_black,
            name=f"jack_well_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((BUMPER_W, BUMPER_T, BUMPER_H)),
        mass=0.58,
    )

    selector = model.part("selector")
    selector.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.018,
                body_style="tapered",
                top_diameter=0.041,
                base_diameter=0.058,
                edge_radius=0.0018,
                crown_radius=0.0012,
                center=False,
            ),
            "multimeter_selector_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_gray,
        name="selector_knob",
    )
    selector.visual(
        Cylinder(radius=0.010, length=0.0031),
        origin=Origin(xyz=(0.0, -0.00155, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_gray,
        name="selector_shaft",
    )
    selector.inertial = Inertial.from_geometry(
        Box((0.060, 0.022, 0.060)),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
    )

    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, BUTTON_MOUNT_Y, SELECTOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )

    button_specs = (
        ("hold_button", "body_to_hold_button", BUTTON_X[0]),
        ("range_button", "body_to_range_button", BUTTON_X[1]),
        ("min_max_button", "body_to_min_max_button", BUTTON_X[2]),
        ("backlight_button", "body_to_backlight_button", BUTTON_X[3]),
    )
    for part_name, joint_name, x in button_specs:
        button = model.part(part_name)
        button.visual(
            Box((0.015, 0.0036, 0.009)),
            origin=Origin(xyz=(0.0, 0.0018, 0.0)),
            material=button_gray,
            name="button_cap",
        )
        button.visual(
            Box((0.0092, 0.004, 0.0068)),
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=button_gray,
            name="button_stem",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.015, 0.008, 0.009)),
            mass=0.008,
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, BUTTON_MOUNT_Y, BUTTON_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=0.05,
                lower=0.0,
                upper=0.0018,
            ),
        )

    stand = model.part("stand")
    stand.visual(_stand_mesh(), material=stand_gray, name="stand_spine")
    stand.inertial = Inertial.from_geometry(
        Box((0.050, 0.016, 0.104)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.002, 0.044)),
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    stand = object_model.get_part("stand")
    selector = object_model.get_part("selector")

    hold_button = object_model.get_part("hold_button")
    range_button = object_model.get_part("range_button")
    min_max_button = object_model.get_part("min_max_button")
    backlight_button = object_model.get_part("backlight_button")

    selector_joint = object_model.get_articulation("body_to_selector")
    stand_joint = object_model.get_articulation("body_to_stand")
    button_joints = {
        "hold_button": object_model.get_articulation("body_to_hold_button"),
        "range_button": object_model.get_articulation("body_to_range_button"),
        "min_max_button": object_model.get_articulation("body_to_min_max_button"),
        "backlight_button": object_model.get_articulation("body_to_backlight_button"),
    }
    button_parts = {
        "hold_button": hold_button,
        "range_button": range_button,
        "min_max_button": min_max_button,
        "backlight_button": backlight_button,
    }

    ctx.expect_gap(
        range_button,
        hold_button,
        axis="x",
        min_gap=0.004,
        name="hold and range buttons stay visually separate",
    )
    ctx.expect_gap(
        min_max_button,
        range_button,
        axis="x",
        min_gap=0.004,
        name="range and min_max buttons stay visually separate",
    )
    ctx.expect_gap(
        backlight_button,
        min_max_button,
        axis="x",
        min_gap=0.004,
        name="min_max and backlight buttons stay visually separate",
    )
    ctx.expect_gap(
        body,
        stand,
        axis="y",
        positive_elem="case_shell",
        negative_elem="stand_spine",
        max_gap=0.006,
        max_penetration=0.0,
        name="rear stand stows close to the back shell",
    )

    selector_rest = ctx.part_world_position(selector)
    with ctx.pose({selector_joint: math.pi / 2.0}):
        selector_turned = ctx.part_world_position(selector)
    ctx.check(
        "selector spins about its center without drifting",
        selector_rest is not None
        and selector_turned is not None
        and abs(selector_rest[0] - selector_turned[0]) < 1e-6
        and abs(selector_rest[1] - selector_turned[1]) < 1e-6
        and abs(selector_rest[2] - selector_turned[2]) < 1e-6,
        details=f"rest={selector_rest}, turned={selector_turned}",
    )

    stand_closed = ctx.part_element_world_aabb(stand, elem="stand_spine")
    with ctx.pose({stand_joint: 1.0}):
        stand_open = ctx.part_element_world_aabb(stand, elem="stand_spine")
    ctx.check(
        "stand swings rearward when opened",
        stand_closed is not None
        and stand_open is not None
        and float(stand_open[0][1]) < float(stand_closed[0][1]) - 0.020,
        details=f"closed={stand_closed}, open={stand_open}",
    )

    rest_positions = {
        name: ctx.part_world_position(part)
        for name, part in button_parts.items()
    }
    for active_name, joint in button_joints.items():
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        if upper is None:
            ctx.fail(f"{active_name} has upper travel", "Expected a finite button travel limit.")
            continue
        with ctx.pose({joint: upper}):
            active_positions = {
                name: ctx.part_world_position(part)
                for name, part in button_parts.items()
            }
        ctx.check(
            f"{active_name} presses inward",
            rest_positions[active_name] is not None
            and active_positions[active_name] is not None
            and float(active_positions[active_name][1]) < float(rest_positions[active_name][1]) - 0.001,
            details=f"rest={rest_positions[active_name]}, pressed={active_positions[active_name]}",
        )
        for other_name in button_parts:
            if other_name == active_name:
                continue
            ctx.check(
                f"{active_name} does not drag {other_name}",
                rest_positions[other_name] is not None
                and active_positions[other_name] is not None
                and abs(float(active_positions[other_name][1]) - float(rest_positions[other_name][1])) < 1e-6,
                details=f"rest={rest_positions[other_name]}, active={active_positions[other_name]}",
            )

    return ctx.report()


object_model = build_object_model()
