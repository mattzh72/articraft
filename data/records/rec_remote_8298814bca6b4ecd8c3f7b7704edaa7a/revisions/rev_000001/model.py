from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (math.cos(math.tau * i / segments) * radius, math.sin(math.tau * i / segments) * radius)
        for i in range(segments)
    ]


def _remote_body_geometry() -> LoftGeometry:
    """A shallow convex rounded-rectangle shell: narrow, long, and slightly crowned."""

    sections = [
        [(x, y, -0.0090) for x, y in rounded_rect_profile(0.041, 0.188, 0.018, corner_segments=10)],
        [(x, y, -0.0020) for x, y in rounded_rect_profile(0.046, 0.194, 0.021, corner_segments=10)],
        [(x, y, 0.0065) for x, y in rounded_rect_profile(0.043, 0.190, 0.019, corner_segments=10)],
        [(x, y, 0.0090) for x, y in rounded_rect_profile(0.039, 0.182, 0.017, corner_segments=10)],
    ]
    return LoftGeometry(sections, cap=True, closed=True)


def _rounded_panel_mesh(width: float, length: float, radius: float, thickness: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(width, length, radius, corner_segments=8), thickness, center=True),
        name,
    )


def _nav_ring_mesh():
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.0165, segments=64),
            [_circle_profile(0.0097, segments=48)],
            0.0030,
            center=True,
        ),
        "navigation_ring",
    )


def _side_rocker_mesh():
    rocker = ExtrudeGeometry(
        rounded_rect_profile(0.0075, 0.035, 0.0035, corner_segments=8),
        0.0040,
        center=True,
    ).rotate_y(math.pi / 2.0)
    return mesh_from_geometry(rocker, "side_volume_rocker")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="streaming_remote")

    shell_mat = model.material("soft_black_plastic", rgba=(0.025, 0.026, 0.028, 1.0))
    deck_mat = model.material("matte_graphite", rgba=(0.055, 0.058, 0.062, 1.0))
    button_mat = model.material("rubber_button", rgba=(0.105, 0.108, 0.112, 1.0))
    nav_mat = model.material("navigation_black", rgba=(0.015, 0.015, 0.017, 1.0))
    text_mat = model.material("button_mark_white", rgba=(0.88, 0.90, 0.92, 1.0))
    red_mat = model.material("power_red", rgba=(0.68, 0.05, 0.04, 1.0))
    door_mat = model.material("battery_door_satin", rgba=(0.045, 0.047, 0.050, 1.0))
    glass_mat = model.material("ir_window", rgba=(0.10, 0.02, 0.018, 0.55))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_remote_body_geometry(), "remote_body"),
        material=shell_mat,
        name="body_shell",
    )
    housing.visual(
        _rounded_panel_mesh(0.033, 0.145, 0.014, 0.0008, "front_inset"),
        origin=Origin(xyz=(0.0, 0.001, 0.0094)),
        material=deck_mat,
        name="front_inset",
    )
    housing.visual(
        Box((0.023, 0.006, 0.0010)),
        origin=Origin(xyz=(0.0, 0.083, 0.00925)),
        material=glass_mat,
        name="ir_window",
    )
    housing.visual(
        Box((0.0030, 0.086, 0.0022)),
        origin=Origin(xyz=(-0.0170, -0.032, -0.0101)),
        material=door_mat,
        name="battery_rail_0",
    )
    housing.visual(
        Box((0.0030, 0.086, 0.0022)),
        origin=Origin(xyz=(0.0170, -0.032, -0.0101)),
        material=door_mat,
        name="battery_rail_1",
    )
    housing.visual(
        Box((0.010, 0.004, 0.0012)),
        origin=Origin(xyz=(0.0, 0.019, -0.0096)),
        material=door_mat,
        name="door_stop_lip",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.046, 0.194, 0.020)),
        mass=0.085,
        origin=Origin(),
    )

    top_z = 0.0098

    nav_pad = model.part("nav_pad")
    nav_pad.visual(
        _nav_ring_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=nav_mat,
        name="nav_ring",
    )
    for mark_name, x, y, sx, sy in (
        ("up_mark", 0.0, 0.0108, 0.0040, 0.0012),
        ("down_mark", 0.0, -0.0108, 0.0040, 0.0012),
        ("side_mark_0", -0.0108, 0.0, 0.0012, 0.0040),
        ("side_mark_1", 0.0108, 0.0, 0.0012, 0.0040),
    ):
        nav_pad.visual(
            Box((sx, sy, 0.0006)),
            origin=Origin(xyz=(x, y, 0.0033)),
            material=text_mat,
            name=mark_name,
        )
    nav_pad.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0165, length=0.0030),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
    )
    model.articulation(
        "housing_to_nav_pad",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=nav_pad,
        origin=Origin(xyz=(0.0, 0.025, top_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.0018),
    )

    center_button = model.part("center_button")
    center_button.visual(
        Cylinder(radius=0.0077, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.0016)),
        material=button_mat,
        name="center_disc",
    )
    center_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0077, length=0.0032),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, 0.0016)),
    )
    model.articulation(
        "housing_to_center_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=center_button,
        origin=Origin(xyz=(0.0, 0.025, top_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=0.08, lower=0.0, upper=0.0016),
    )

    button_specs = [
        ("power_button", "power_cap", (0.0, 0.064), 0.0053, red_mat, 0.0016),
        ("back_button", "back_cap", (-0.0105, -0.005), 0.0046, button_mat, 0.0017),
        ("home_button", "home_cap", (0.0105, -0.005), 0.0046, button_mat, 0.0017),
        ("play_button", "play_cap", (0.0, -0.026), 0.0054, button_mat, 0.0018),
        ("app_button", "app_cap", (0.0, -0.049), 0.0051, button_mat, 0.0017),
    ]
    for part_name, visual_name, (x, y), radius, material, travel in button_specs:
        button = model.part(part_name)
        button.visual(
            Cylinder(radius=radius, length=0.0030),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=material,
            name=visual_name,
        )
        button.inertial = Inertial.from_geometry(
            Cylinder(radius=radius, length=0.0030),
            mass=0.0018,
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        )
        model.articulation(
            f"housing_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(x, y, top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.2, velocity=0.08, lower=0.0, upper=travel),
        )

    side_rocker = model.part("side_rocker")
    side_rocker.visual(
        _side_rocker_mesh(),
        origin=Origin(xyz=(0.0020, 0.0, 0.0)),
        material=button_mat,
        name="rocker_cap",
    )
    side_rocker.visual(
        Box((0.0010, 0.0060, 0.0009)),
        origin=Origin(xyz=(0.0044, 0.0085, 0.0)),
        material=text_mat,
        name="plus_mark",
    )
    side_rocker.visual(
        Box((0.0010, 0.0060, 0.0009)),
        origin=Origin(xyz=(0.0044, -0.0085, 0.0)),
        material=text_mat,
        name="minus_mark",
    )
    side_rocker.inertial = Inertial.from_geometry(
        Box((0.008, 0.034, 0.008)),
        mass=0.0035,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
    )
    model.articulation(
        "housing_to_side_rocker",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=side_rocker,
        origin=Origin(xyz=(0.02255, 0.008, 0.0000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.45, velocity=2.5, lower=-0.23, upper=0.23),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        _rounded_panel_mesh(0.0275, 0.071, 0.0065, 0.0024, "battery_door_panel"),
        origin=Origin(),
        material=door_mat,
        name="door_panel",
    )
    battery_door.visual(
        Cylinder(radius=0.0042, length=0.0008),
        origin=Origin(xyz=(0.0, -0.015, -0.0016)),
        material=deck_mat,
        name="thumb_dimple",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.0275, 0.071, 0.0024)),
        mass=0.006,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=battery_door,
        origin=Origin(xyz=(0.0, -0.030, -0.0102)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.12, lower=0.0, upper=0.032),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    nav_pad = object_model.get_part("nav_pad")
    center_button = object_model.get_part("center_button")
    side_rocker = object_model.get_part("side_rocker")
    battery_door = object_model.get_part("battery_door")
    nav_joint = object_model.get_articulation("housing_to_nav_pad")
    center_joint = object_model.get_articulation("housing_to_center_button")
    rocker_joint = object_model.get_articulation("housing_to_side_rocker")
    door_joint = object_model.get_articulation("housing_to_battery_door")

    for name in ("power_button", "back_button", "home_button", "play_button", "app_button"):
        button = object_model.get_part(name)
        joint = object_model.get_articulation(f"housing_to_{name}")
        ctx.expect_gap(
            button,
            housing,
            axis="z",
            min_gap=-0.00005,
            max_gap=0.00025,
            negative_elem="front_inset",
            name=f"{name} sits on front shell",
        )
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: joint.motion_limits.upper}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"{name} depresses inward",
            rest is not None and pressed is not None and pressed[2] < rest[2] - 0.0010,
            details=f"rest={rest}, pressed={pressed}",
        )

    ctx.expect_within(
        center_button,
        nav_pad,
        axes="xy",
        margin=0.010,
        name="select button stays inside circular pad footprint",
    )
    rest_center = ctx.part_world_position(center_button)
    with ctx.pose({center_joint: center_joint.motion_limits.upper}):
        pressed_center = ctx.part_world_position(center_button)
    ctx.check(
        "center select depresses independently",
        rest_center is not None
        and pressed_center is not None
        and pressed_center[2] < rest_center[2] - 0.0010,
        details=f"rest={rest_center}, pressed={pressed_center}",
    )

    rest_nav = ctx.part_world_position(nav_pad)
    with ctx.pose({nav_joint: nav_joint.motion_limits.upper}):
        pressed_nav = ctx.part_world_position(nav_pad)
    ctx.check(
        "navigation ring depresses",
        rest_nav is not None and pressed_nav is not None and pressed_nav[2] < rest_nav[2] - 0.0010,
        details=f"rest={rest_nav}, pressed={pressed_nav}",
    )

    ctx.expect_gap(
        housing,
        battery_door,
        axis="z",
        min_gap=-0.00005,
        max_gap=0.00025,
        positive_elem="body_shell",
        negative_elem="door_panel",
        name="battery door rides on back shell",
    )
    ctx.expect_within(
        battery_door,
        housing,
        axes="x",
        margin=0.003,
        inner_elem="door_panel",
        outer_elem="body_shell",
        name="battery door captured between side rails",
    )
    rest_door = ctx.part_world_position(battery_door)
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        extended_door = ctx.part_world_position(battery_door)
        ctx.expect_overlap(
            battery_door,
            housing,
            axes="y",
            elem_a="door_panel",
            elem_b="body_shell",
            min_overlap=0.020,
            name="slid battery door remains retained on back",
        )
    ctx.check(
        "battery door slides toward tail",
        rest_door is not None and extended_door is not None and extended_door[1] < rest_door[1] - 0.025,
        details=f"rest={rest_door}, extended={extended_door}",
    )

    rest_rocker = ctx.part_world_position(side_rocker)
    rest_rocker_box = ctx.part_element_world_aabb(side_rocker, elem="rocker_cap")
    with ctx.pose({rocker_joint: rocker_joint.motion_limits.upper}):
        tilted_rocker = ctx.part_world_position(side_rocker)
        tilted_rocker_box = ctx.part_element_world_aabb(side_rocker, elem="rocker_cap")
    ctx.check(
        "side rocker pivots on local hinge",
        rest_rocker is not None
        and tilted_rocker is not None
        and rest_rocker_box is not None
        and tilted_rocker_box is not None
        and abs(rocker_joint.axis[1]) > 0.9
        and abs(tilted_rocker_box[0][2] - rest_rocker_box[0][2]) > 0.00035,
        details=(
            f"axis={rocker_joint.axis}, rest={rest_rocker}, tilted={tilted_rocker}, "
            f"rest_box={rest_rocker_box}, tilted_box={tilted_rocker_box}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
