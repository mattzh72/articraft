from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], center: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape.translate(center)


def _build_housing_shape() -> cq.Workplane:
    housing = _rounded_box((0.280, 0.230, 0.040), (0.040, 0.000, 0.020), 0.018)
    housing = housing.union(_rounded_box((0.120, 0.130, 0.220), (-0.020, 0.000, 0.150), 0.014))
    housing = housing.union(_rounded_box((0.105, 0.120, 0.085), (0.010, 0.000, 0.250), 0.012))
    housing = housing.union(_rounded_box((0.190, 0.120, 0.080), (0.070, 0.000, 0.320), 0.016))
    housing = housing.union(_rounded_box((0.120, 0.110, 0.040), (-0.005, 0.000, 0.358), 0.010))
    housing = housing.union(_rounded_box((0.055, 0.095, 0.060), (0.137, 0.000, 0.286), 0.010))

    for y_center in (-0.126, 0.126):
        housing = housing.union(_rounded_box((0.022, 0.026, 0.235), (0.044, y_center, 0.158), 0.007))
        housing = housing.union(_rounded_box((0.044, 0.034, 0.060), (0.066, y_center, 0.255), 0.008))
        housing = housing.union(_rounded_box((0.050, 0.038, 0.020), (0.044, y_center, 0.050), 0.007))

    bowl_window = cq.Workplane("XY").box(0.220, 0.200, 0.180).translate((0.078, 0.000, 0.130))
    button_recess = cq.Workplane("XY").box(0.024, 0.014, 0.012).translate((-0.028, 0.067, 0.212))
    return housing.cut(bowl_window).cut(button_recess)


def _build_platform_shape() -> cq.Workplane:
    platform = _rounded_box((0.136, 0.150, 0.012), (0.014, 0.000, 0.000), 0.006)
    platform = platform.union(_rounded_box((0.020, 0.176, 0.014), (-0.070, 0.000, 0.018), 0.004))
    platform = platform.union(_rounded_box((0.076, 0.050, 0.016), (-0.022, -0.079, 0.010), 0.005))
    platform = platform.union(_rounded_box((0.076, 0.050, 0.016), (-0.022, 0.079, 0.010), 0.005))
    platform = platform.union(_rounded_box((0.022, 0.018, 0.090), (-0.056, -0.103, 0.039), 0.004))
    platform = platform.union(_rounded_box((0.022, 0.018, 0.090), (-0.056, 0.103, 0.039), 0.004))

    locator = cq.Workplane("XY").circle(0.071).circle(0.055).extrude(0.010).translate((0.014, 0.000, 0.004))
    return platform.union(locator)


def _build_bowl_geometry() -> LatheGeometry:
    outer_profile = [
        (0.024, 0.000),
        (0.051, 0.010),
        (0.081, 0.042),
        (0.093, 0.094),
        (0.097, 0.140),
        (0.099, 0.148),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.045, 0.013),
        (0.075, 0.045),
        (0.087, 0.094),
        (0.091, 0.142),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        end_cap="round",
        lip_samples=8,
    )


def _build_paddle_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").cylinder(0.044, 0.006).translate((0.000, 0.000, -0.022))
    collar = cq.Workplane("XY").cylinder(0.016, 0.013).translate((0.000, 0.000, -0.043))
    neck = cq.Workplane("XZ").center(0.000, -0.060).rect(0.020, 0.036).extrude(0.012, both=True)
    blade = cq.Workplane("XZ").center(0.000, -0.104).rect(0.054, 0.084).extrude(0.012, both=True)
    blade_window = cq.Workplane("XZ").center(0.000, -0.104).rect(0.026, 0.040).extrude(0.016, both=True)
    lower_bridge = cq.Workplane("XZ").center(0.000, -0.136).rect(0.046, 0.016).extrude(0.012, both=True)
    blade = blade.cut(blade_window).union(lower_bridge)
    return shaft.union(collar).union(neck).union(blade)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_bowl_lift_mixer")

    body_paint = model.material("body_paint", rgba=(0.84, 0.20, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.86, 0.88, 0.90, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.10, 0.12, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shape(), "housing_shell"),
        material=body_paint,
        name="housing_shell",
    )

    platform = model.part("platform")
    platform.visual(
        mesh_from_cadquery(_build_platform_shape(), "platform_frame"),
        material=steel,
        name="platform_frame",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(_build_bowl_geometry(), "bowl_shell"),
        material=steel,
        name="bowl_shell",
    )

    lift_lever = model.part("lift_lever")
    lift_lever.visual(
        Box((0.050, 0.010, 0.008)),
        origin=Origin(xyz=(0.014, 0.009, -0.010), rpy=(0.000, -0.38, 0.000)),
        material=dark_trim,
        name="lever_arm",
    )
    lift_lever.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.030, 0.011, -0.019), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=dark_trim,
        name="lever_grip",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.018, 0.008, 0.012)),
        origin=Origin(xyz=(0.000, 0.0035, 0.000)),
        material=dark_trim,
        name="button_cap",
    )

    drive = model.part("drive")
    drive.visual(
        Cylinder(radius=0.016, length=0.026),
        origin=Origin(xyz=(0.000, 0.000, -0.013)),
        material=dark_trim,
        name="drive_hub",
    )

    paddle = model.part("paddle")
    paddle.visual(
        mesh_from_cadquery(_build_paddle_shape(), "paddle_beater"),
        material=steel,
        name="paddle_beater",
    )

    model.articulation(
        "housing_to_platform",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=platform,
        origin=Origin(xyz=(0.100, 0.000, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.10, lower=0.0, upper=0.060),
    )
    model.articulation(
        "platform_to_bowl",
        ArticulationType.FIXED,
        parent=platform,
        child=bowl,
        origin=Origin(xyz=(0.070, 0.000, 0.014)),
    )
    model.articulation(
        "housing_to_lift_lever",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lift_lever,
        origin=Origin(xyz=(-0.030, 0.061, 0.166)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "housing_to_release_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=release_button,
        origin=Origin(xyz=(-0.028, 0.065, 0.212)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.04, lower=0.0, upper=0.004),
    )
    model.articulation(
        "housing_to_drive",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=drive,
        origin=Origin(xyz=(0.165, 0.000, 0.256)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "drive_to_paddle",
        ArticulationType.FIXED,
        parent=drive,
        child=paddle,
        origin=Origin(xyz=(0.000, 0.000, -0.024)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    platform = object_model.get_part("platform")
    bowl = object_model.get_part("bowl")
    paddle = object_model.get_part("paddle")
    lift_lever = object_model.get_part("lift_lever")
    release_button = object_model.get_part("release_button")

    platform_joint = object_model.get_articulation("housing_to_platform")
    lever_joint = object_model.get_articulation("housing_to_lift_lever")
    button_joint = object_model.get_articulation("housing_to_release_button")
    drive_joint = object_model.get_articulation("housing_to_drive")

    ctx.allow_overlap(
        "housing",
        "platform",
        elem_a="housing_shell",
        elem_b="platform_frame",
        reason="The platform tabs are simplified as solid sliders running inside the side-arm guide proxies.",
    )
    ctx.allow_overlap(
        "housing",
        "drive",
        elem_a="housing_shell",
        elem_b="drive_hub",
        reason="The drive hub is intentionally socketed into the head to represent the hidden motor coupling.",
    )
    ctx.allow_overlap(
        "housing",
        "release_button",
        elem_a="housing_shell",
        elem_b="button_cap",
        reason="The release button is intentionally modeled seated into a shallow side recess.",
    )
    ctx.allow_overlap(
        "housing",
        "lift_lever",
        elem_a="housing_shell",
        elem_b="lever_arm",
        reason="The lift lever root is intentionally nested into a short side pivot pocket.",
    )

    rest_platform_pos = ctx.part_world_position(platform)
    rest_button_pos = ctx.part_world_position(release_button)
    rest_grip_aabb = ctx.part_element_world_aabb(lift_lever, elem="lever_grip")

    with ctx.pose({platform_joint: 0.0}):
        ctx.expect_overlap(
            bowl,
            paddle,
            axes="xy",
            min_overlap=0.018,
            name="lowered bowl remains under the paddle axis",
        )

    with ctx.pose({platform_joint: 0.060, drive_joint: math.pi / 2.0}):
        raised_platform_pos = ctx.part_world_position(platform)
        ctx.expect_overlap(
            bowl,
            paddle,
            axes="xy",
            min_overlap=0.018,
            name="raised bowl stays centered beneath the paddle",
        )
    ctx.check(
        "platform lifts vertically",
        rest_platform_pos is not None
        and raised_platform_pos is not None
        and raised_platform_pos[2] > rest_platform_pos[2] + 0.050,
        details=f"rest={rest_platform_pos}, raised={raised_platform_pos}",
    )

    with ctx.pose({button_joint: 0.004}):
        pressed_button_pos = ctx.part_world_position(release_button)
    ctx.check(
        "release button presses into the housing",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] < rest_button_pos[1] - 0.003,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    with ctx.pose({lever_joint: math.radians(45.0)}):
        raised_grip_aabb = ctx.part_element_world_aabb(lift_lever, elem="lever_grip")
    ctx.check(
        "lift lever rotates upward",
        rest_grip_aabb is not None
        and raised_grip_aabb is not None
        and raised_grip_aabb[1][2] > rest_grip_aabb[1][2] + 0.012,
        details=f"rest={rest_grip_aabb}, raised={raised_grip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
