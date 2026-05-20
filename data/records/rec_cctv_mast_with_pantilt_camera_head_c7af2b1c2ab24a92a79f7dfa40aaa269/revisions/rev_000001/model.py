from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parapet_cctv_bracket")

    white = model.material("powder_coated_white", rgba=(0.88, 0.90, 0.86, 1.0))
    dark = model.material("dark_anodized_metal", rgba=(0.08, 0.09, 0.09, 1.0))
    black = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    glass = model.material("smoked_lens_glass", rgba=(0.02, 0.025, 0.03, 0.82))

    wall_clamp = model.part("wall_clamp")
    wall_clamp.visual(
        Box((0.040, 0.180, 0.280)),
        origin=Origin(xyz=(0.000, 0.000, 0.140)),
        material=white,
        name="rectangular_clamp",
    )
    wall_clamp.visual(
        Box((0.025, 0.130, 0.140)),
        origin=Origin(xyz=(0.0325, 0.000, 0.200)),
        material=white,
        name="raised_arm_pad",
    )
    wall_clamp.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.024, -0.060, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="bolt_0",
    )
    wall_clamp.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.024, 0.060, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="bolt_1",
    )
    wall_clamp.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.024, -0.060, 0.240), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="bolt_2",
    )
    wall_clamp.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.024, 0.060, 0.240), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="bolt_3",
    )
    wall_clamp.visual(
        Cylinder(radius=0.022, length=0.275),
        origin=Origin(xyz=(0.180, 0.000, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white,
        name="horizontal_arm",
    )
    wall_clamp.visual(
        Box((0.145, 0.012, 0.055)),
        origin=Origin(xyz=(0.105, -0.031, 0.169), rpy=(0.0, -0.33, 0.0)),
        material=white,
        name="web_0",
    )
    wall_clamp.visual(
        Box((0.145, 0.012, 0.055)),
        origin=Origin(xyz=(0.105, 0.031, 0.169), rpy=(0.0, -0.33, 0.0)),
        material=white,
        name="web_1",
    )
    wall_clamp.visual(
        Cylinder(radius=0.047, length=0.025),
        origin=Origin(xyz=(0.320, 0.000, 0.1825)),
        material=white,
        name="fixed_bearing_socket",
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, -0.010)),
        material=white,
        name="azimuth_cap",
    )
    pan_yoke.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.000, 0.000, -0.050)),
        material=white,
        name="drop_post",
    )
    pan_yoke.visual(
        Box((0.046, 0.130, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, -0.084)),
        material=white,
        name="yoke_bridge",
    )
    pan_yoke.visual(
        Box((0.026, 0.012, 0.120)),
        origin=Origin(xyz=(0.000, -0.059, -0.135)),
        material=white,
        name="side_arm_0",
    )
    pan_yoke.visual(
        Box((0.026, 0.012, 0.120)),
        origin=Origin(xyz=(0.000, 0.059, -0.135)),
        material=white,
        name="side_arm_1",
    )
    pan_yoke.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.000, -0.071, -0.135), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="pivot_boss_0",
    )
    pan_yoke.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.000, 0.071, -0.135), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="pivot_boss_1",
    )

    camera_housing = model.part("camera_housing")
    camera_housing.visual(
        Cylinder(radius=0.018, length=0.106),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="pivot_pin",
    )
    camera_housing.visual(
        Cylinder(radius=0.034, length=0.165),
        origin=Origin(xyz=(0.085, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white,
        name="body_shell",
    )
    camera_housing.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.176, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="front_bezel",
    )
    camera_housing.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.185, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens_glass",
    )
    camera_housing.visual(
        Box((0.190, 0.104, 0.012)),
        origin=Origin(xyz=(0.085, 0.000, 0.039)),
        material=white,
        name="sun_hood_top",
    )
    camera_housing.visual(
        Box((0.170, 0.008, 0.030)),
        origin=Origin(xyz=(0.092, -0.052, 0.025)),
        material=white,
        name="sun_hood_side_0",
    )
    camera_housing.visual(
        Box((0.170, 0.008, 0.030)),
        origin=Origin(xyz=(0.092, 0.052, 0.025)),
        material=white,
        name="sun_hood_side_1",
    )

    model.articulation(
        "azimuth_bearing",
        ArticulationType.CONTINUOUS,
        parent=wall_clamp,
        child=pan_yoke,
        origin=Origin(xyz=(0.320, 0.000, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5),
    )
    model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=camera_housing,
        origin=Origin(xyz=(0.000, 0.000, -0.135)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=-0.80, upper=0.80),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_clamp = object_model.get_part("wall_clamp")
    pan_yoke = object_model.get_part("pan_yoke")
    camera_housing = object_model.get_part("camera_housing")
    azimuth = object_model.get_articulation("azimuth_bearing")
    tilt = object_model.get_articulation("tilt_hinge")

    def axis_close(axis: tuple[float, float, float], target: tuple[float, float, float]) -> bool:
        return all(abs(a - b) < 1e-6 for a, b in zip(axis, target))

    def aabb_center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[axis_index] + hi[axis_index]) * 0.5

    ctx.check(
        "azimuth bearing is continuous",
        azimuth.articulation_type == ArticulationType.CONTINUOUS and axis_close(azimuth.axis, (0.0, 0.0, 1.0)),
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}",
    )

    limits = tilt.motion_limits
    ctx.check(
        "camera tilt hinge is limited revolute",
        (
            tilt.articulation_type == ArticulationType.REVOLUTE
            and axis_close(tilt.axis, (0.0, 1.0, 0.0))
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower <= -0.75
            and limits.upper >= 0.75
        ),
        details=f"type={tilt.articulation_type}, axis={tilt.axis}, limits={limits}",
    )

    ctx.expect_origin_gap(
        pan_yoke,
        wall_clamp,
        axis="x",
        min_gap=0.28,
        max_gap=0.36,
        name="short arm places pan head at arm tip",
    )
    ctx.expect_contact(
        pan_yoke,
        wall_clamp,
        elem_a="azimuth_cap",
        elem_b="fixed_bearing_socket",
        contact_tol=0.001,
        name="azimuth cap seats against fixed bearing socket",
    )
    ctx.expect_contact(
        camera_housing,
        pan_yoke,
        elem_a="pivot_pin",
        elem_b="side_arm_0",
        contact_tol=0.001,
        name="tilt pin reaches one yoke cheek",
    )
    ctx.expect_contact(
        camera_housing,
        pan_yoke,
        elem_a="pivot_pin",
        elem_b="side_arm_1",
        contact_tol=0.001,
        name="tilt pin reaches opposite yoke cheek",
    )
    ctx.expect_within(
        camera_housing,
        pan_yoke,
        axes="y",
        inner_elem="body_shell",
        outer_elem="yoke_bridge",
        margin=0.0,
        name="camera body fits between yoke cheeks",
    )

    rest_body_aabb = ctx.part_element_world_aabb(camera_housing, elem="body_shell")
    rest_body_x = aabb_center(rest_body_aabb, 0)
    rest_body_y = aabb_center(rest_body_aabb, 1)
    with ctx.pose({azimuth: math.pi / 2.0}):
        turned_body_aabb = ctx.part_element_world_aabb(camera_housing, elem="body_shell")
    turned_body_x = aabb_center(turned_body_aabb, 0)
    turned_body_y = aabb_center(turned_body_aabb, 1)
    ctx.check(
        "azimuth pose swings camera around vertical bearing",
        (
            rest_body_x is not None
            and rest_body_y is not None
            and turned_body_x is not None
            and turned_body_y is not None
            and turned_body_y > rest_body_y + 0.050
            and turned_body_x < rest_body_x - 0.050
        ),
        details=f"rest=({rest_body_x}, {rest_body_y}), turned=({turned_body_x}, {turned_body_y})",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(camera_housing, elem="front_bezel")
    rest_lens_z = aabb_center(rest_lens_aabb, 2)
    with ctx.pose({tilt: -0.50}):
        raised_lens_aabb = ctx.part_element_world_aabb(camera_housing, elem="front_bezel")
    raised_lens_z = aabb_center(raised_lens_aabb, 2)
    ctx.check(
        "tilt pose raises camera nose",
        rest_lens_z is not None and raised_lens_z is not None and raised_lens_z > rest_lens_z + 0.050,
        details=f"rest_z={rest_lens_z}, raised_z={raised_lens_z}",
    )

    return ctx.report()


object_model = build_object_model()
