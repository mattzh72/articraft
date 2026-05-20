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


BODY_LENGTH = 0.188
BODY_REAR_WIDTH = 0.046
BODY_FRONT_WIDTH = 0.040
BODY_THICKNESS = 0.014
TOP_Z = BODY_THICKNESS * 0.5
BACK_Z = -BODY_THICKNESS * 0.5

BUTTON_MOUNT_Z = TOP_Z + 0.0012
ROCKER_HINGE_Z = TOP_Z + 0.00083
COVER_MOUNT_Z = BACK_Z - 0.00012

NAV_CENTER_X = 0.024


def _soft_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(min(radius, 0.45 * height, 0.22 * min(length, width)))
    )


def _body_shape() -> cq.Workplane:
    outline = [
        (-BODY_LENGTH * 0.5, -BODY_REAR_WIDTH * 0.5),
        (BODY_LENGTH * 0.5, -BODY_FRONT_WIDTH * 0.5),
        (BODY_LENGTH * 0.5, BODY_FRONT_WIDTH * 0.5),
        (-BODY_LENGTH * 0.5, BODY_REAR_WIDTH * 0.5),
    ]
    body = (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(BODY_THICKNESS)
        .translate((0.0, 0.0, -BODY_THICKNESS * 0.5))
        .edges("|Z")
        .fillet(0.0062)
        .edges("#Z")
        .fillet(0.0016)
    )

    return body


def _power_rocker_shape() -> cq.Workplane:
    cap = _soft_box(0.013, 0.0105, 0.00175, 0.0026).translate((0.0042, 0.0, 0.0))
    hinge_barrel = (
        cq.Workplane("XY")
        .circle(0.00085)
        .extrude(0.0082)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0004, 0.0, 0.00072))
    )
    return cap.union(hinge_barrel)


def _battery_cover_shape() -> cq.Workplane:
    panel = _soft_box(0.090, 0.032, 0.0017, 0.0035).translate((0.0, 0.0, -0.0017))
    left_rib = _soft_box(0.068, 0.0028, 0.00075, 0.001).translate((0.0, -0.012, -0.00245))
    right_rib = _soft_box(0.068, 0.0028, 0.00075, 0.001).translate((0.0, 0.012, -0.00245))
    finger_scoop = (
        cq.Workplane("XY")
        .circle(0.0068)
        .extrude(0.0035)
        .translate((-0.034, 0.0, -0.0034))
    )
    return panel.union(left_rib).union(right_rib).cut(finger_scoop)


def _add_button(
    model: ArticulatedObject,
    *,
    name: str,
    origin_xyz: tuple[float, float, float],
    geometry,
    material,
    visual_name: str = "button_cap",
    visual_origin: Origin | None = None,
    travel: float = 0.0006,
) -> None:
    button = model.part(name)
    button.visual(
        geometry,
        origin=visual_origin or Origin(),
        material=material,
        name=visual_name,
    )
    model.articulation(
        f"body_to_{name}",
        ArticulationType.PRISMATIC,
        parent="body",
        child=button,
        origin=Origin(xyz=origin_xyz),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.08,
            lower=0.0,
            upper=travel,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="media_remote")

    body_mat = model.material("body_mat", rgba=(0.09, 0.09, 0.10, 1.0))
    trim_mat = model.material("trim_mat", rgba=(0.13, 0.13, 0.14, 1.0))
    button_mat = model.material("button_mat", rgba=(0.22, 0.22, 0.24, 1.0))
    center_mat = model.material("center_mat", rgba=(0.18, 0.18, 0.20, 1.0))
    rocker_mat = model.material("rocker_mat", rgba=(0.58, 0.10, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "remote_body"),
        material=body_mat,
        name="housing",
    )
    body.visual(
        Box((0.088, 0.031, 0.0013)),
        origin=Origin(xyz=(0.006, 0.0, TOP_Z + 0.00005)),
        material=trim_mat,
        name="control_panel",
    )
    body.visual(
        Box((0.084, 0.040, 0.0022)),
        origin=Origin(xyz=(-0.028, 0.0, TOP_Z + 0.0001)),
        material=trim_mat,
        name="back_hump",
    )
    body.visual(
        Box((0.013, 0.0105, 0.0013)),
        origin=Origin(xyz=(0.0715, 0.0, TOP_Z + 0.00005)),
        material=trim_mat,
        name="rocker_saddle",
    )
    body.visual(
        Box((0.096, 0.034, 0.0012)),
        origin=Origin(xyz=(-0.026, 0.0, BACK_Z - 0.0003)),
        material=trim_mat,
        name="battery_track",
    )

    rocker = model.part("power_rocker")
    rocker.visual(
        mesh_from_cadquery(_power_rocker_shape(), "power_rocker"),
        material=rocker_mat,
        name="rocker_shell",
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(0.069, 0.0, ROCKER_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=1.5,
            lower=0.0,
            upper=0.16,
        ),
    )

    nav_long_mesh = mesh_from_cadquery(_soft_box(0.0085, 0.0165, 0.0016, 0.0024), "nav_long")
    nav_wide_mesh = mesh_from_cadquery(_soft_box(0.014, 0.007, 0.0016, 0.0019), "nav_wide")

    _add_button(
        model,
        name="nav_front",
        origin_xyz=(NAV_CENTER_X + 0.0088, 0.0, BUTTON_MOUNT_Z),
        geometry=nav_long_mesh,
        material=button_mat,
        visual_name="button_cap",
    )
    _add_button(
        model,
        name="nav_rear",
        origin_xyz=(NAV_CENTER_X - 0.0088, 0.0, BUTTON_MOUNT_Z),
        geometry=nav_long_mesh,
        material=button_mat,
        visual_name="button_cap",
    )
    _add_button(
        model,
        name="nav_left",
        origin_xyz=(NAV_CENTER_X, -0.0088, BUTTON_MOUNT_Z),
        geometry=nav_wide_mesh,
        material=button_mat,
        visual_name="button_cap",
    )
    _add_button(
        model,
        name="nav_right",
        origin_xyz=(NAV_CENTER_X, 0.0088, BUTTON_MOUNT_Z),
        geometry=nav_wide_mesh,
        material=button_mat,
        visual_name="button_cap",
    )
    _add_button(
        model,
        name="nav_center",
        origin_xyz=(NAV_CENTER_X, 0.0, BUTTON_MOUNT_Z + 0.00005),
        geometry=Cylinder(radius=0.0063, length=0.00155),
        material=center_mat,
        visual_name="button_cap",
        visual_origin=Origin(xyz=(0.0, 0.0, 0.000775)),
        travel=0.00045,
    )

    for name, x_pos, y_pos in (
        ("back_button", -0.006, -0.009),
        ("home_button", -0.006, 0.009),
        ("play_button", -0.028, -0.009),
        ("mute_button", -0.028, 0.009),
    ):
        _add_button(
            model,
            name=name,
            origin_xyz=(x_pos, y_pos, BUTTON_MOUNT_Z),
            geometry=Cylinder(radius=0.0045, length=0.00145),
            material=button_mat,
            visual_name="button_cap",
            visual_origin=Origin(xyz=(0.0, 0.0, 0.000725)),
            travel=0.00035,
        )

    battery_cover = model.part("battery_cover")
    battery_cover.visual(
        mesh_from_cadquery(_battery_cover_shape(), "battery_cover"),
        material=trim_mat,
        name="cover_shell",
    )
    model.articulation(
        "body_to_battery_cover",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_cover,
        origin=Origin(xyz=(-0.022, 0.0, COVER_MOUNT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.032,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    power_rocker = object_model.get_part("power_rocker")
    nav_center = object_model.get_part("nav_center")
    home_button = object_model.get_part("home_button")
    battery_cover = object_model.get_part("battery_cover")

    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    nav_center_joint = object_model.get_articulation("body_to_nav_center")
    cover_joint = object_model.get_articulation("body_to_battery_cover")

    rocker_limits = rocker_joint.motion_limits
    nav_limits = nav_center_joint.motion_limits
    cover_limits = cover_joint.motion_limits

    ctx.expect_gap(
        nav_center,
        body,
        axis="z",
        positive_elem="button_cap",
        negative_elem="control_panel",
        min_gap=0.00045,
        max_gap=0.00095,
        name="nav center sits slightly proud of the front shell",
    )
    ctx.expect_within(
        battery_cover,
        body,
        axes="y",
        margin=0.004,
        name="battery cover stays within the remote width",
    )

    nav_rest = ctx.part_world_position(nav_center)
    home_rest = ctx.part_world_position(home_button)
    cover_rest = ctx.part_world_position(battery_cover)
    rocker_rest_aabb = ctx.part_world_aabb(power_rocker)

    if nav_limits is not None and nav_limits.upper is not None:
        with ctx.pose({nav_center_joint: nav_limits.upper}):
            ctx.expect_gap(
                nav_center,
                body,
                axis="z",
                positive_elem="button_cap",
                negative_elem="control_panel",
                min_gap=0.0,
                max_gap=0.00035,
                name="nav center remains clear of the shell when pressed",
            )
            nav_pressed = ctx.part_world_position(nav_center)
            home_while_nav_pressed = ctx.part_world_position(home_button)
        ctx.check(
            "nav center depresses downward",
            nav_rest is not None
            and nav_pressed is not None
            and nav_pressed[2] < nav_rest[2] - 0.00035,
            details=f"rest={nav_rest}, pressed={nav_pressed}",
        )
        ctx.check(
            "front buttons depress independently",
            home_rest is not None
            and home_while_nav_pressed is not None
            and abs(home_while_nav_pressed[2] - home_rest[2]) < 1e-6,
            details=f"home_rest={home_rest}, home_while_nav_pressed={home_while_nav_pressed}",
        )

    if rocker_limits is not None and rocker_limits.upper is not None:
        with ctx.pose({rocker_joint: rocker_limits.upper}):
            rocker_upper_aabb = ctx.part_world_aabb(power_rocker)
        ctx.expect_gap(
            power_rocker,
            body,
            axis="z",
            positive_elem="rocker_shell",
            negative_elem="rocker_saddle",
            min_gap=-1e-5,
            max_gap=0.0002,
            name="power rocker seats on its saddle at rest",
        )
        ctx.check(
            "power rocker lifts at its free edge",
            rocker_rest_aabb is not None
            and rocker_upper_aabb is not None
            and rocker_upper_aabb[1][2] > rocker_rest_aabb[1][2] + 0.00045,
            details=f"rest={rocker_rest_aabb}, upper={rocker_upper_aabb}",
        )

    if cover_limits is not None and cover_limits.upper is not None:
        with ctx.pose({cover_joint: cover_limits.upper}):
            cover_open = ctx.part_world_position(battery_cover)
        ctx.check(
            "battery cover slides toward the rear",
            cover_rest is not None
            and cover_open is not None
            and cover_open[0] < cover_rest[0] - 0.025,
            details=f"rest={cover_rest}, open={cover_open}",
        )

    return ctx.report()


object_model = build_object_model()
