from __future__ import annotations

from math import pi

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


def _add_box(part, size, xyz, *, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(part, radius, length, xyz, rpy, *, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pneumatic_overbed_table")

    model.material("base_paint", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("column_paint", rgba=(0.74, 0.77, 0.80, 1.0))
    model.material("tray_surface", rgba=(0.92, 0.92, 0.88, 1.0))
    model.material("tray_trim", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("control_dark", rgba=(0.18, 0.22, 0.26, 1.0))
    model.material("wheel_rubber", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("wheel_hub", rgba=(0.47, 0.49, 0.52, 1.0))

    wheel_radius = 0.024
    wheel_thickness = 0.018
    axle_z = 0.026
    base_top_z = 0.065

    sleeve_center = (-0.16, -0.06)
    sleeve_outer = (0.090, 0.070)
    sleeve_wall = 0.007
    sleeve_height = 0.460
    sleeve_bottom_z = 0.080
    sleeve_top_z = sleeve_bottom_z + sleeve_height

    inner_tube_size = (0.062, 0.044, 0.600)
    inner_tube_hidden = 0.340
    inner_tube_exposed = inner_tube_size[2] - inner_tube_hidden
    inner_tube_center_z = (inner_tube_exposed - inner_tube_hidden) / 2.0

    tray_tilt_origin = (0.0, 0.160, 0.300)
    tray_size = (0.600, 0.400, 0.026)
    tray_center = (0.100, 0.000, 0.036)
    wing_size = (0.220, 0.260, 0.018)

    base = model.part("base")
    _add_box(base, (0.440, 0.080, 0.030), (0.000, -0.100, base_top_z), material="base_paint", name="rear_beam")
    _add_box(base, (0.060, 0.540, 0.030), (-0.160, 0.170, base_top_z), material="base_paint", name="rail_0")
    _add_box(base, (0.060, 0.540, 0.030), (0.160, 0.170, base_top_z), material="base_paint", name="rail_1")
    _add_box(base, (0.110, 0.100, 0.040), (-0.160, -0.050, 0.100), material="base_paint", name="column_plinth")

    _add_box(
        base,
        (sleeve_outer[0], sleeve_wall, sleeve_height),
        (sleeve_center[0], sleeve_center[1] - (sleeve_outer[1] / 2.0) + (sleeve_wall / 2.0), sleeve_bottom_z + (sleeve_height / 2.0)),
        material="column_paint",
        name="outer_sleeve_back",
    )
    _add_box(
        base,
        (sleeve_outer[0], sleeve_wall, sleeve_height),
        (sleeve_center[0], sleeve_center[1] + (sleeve_outer[1] / 2.0) - (sleeve_wall / 2.0), sleeve_bottom_z + (sleeve_height / 2.0)),
        material="column_paint",
        name="outer_sleeve_front",
    )
    _add_box(
        base,
        (sleeve_wall, sleeve_outer[1], sleeve_height),
        (sleeve_center[0] - (sleeve_outer[0] / 2.0) + (sleeve_wall / 2.0), sleeve_center[1], sleeve_bottom_z + (sleeve_height / 2.0)),
        material="column_paint",
        name="outer_sleeve_left",
    )
    _add_box(
        base,
        (sleeve_wall, sleeve_outer[1], sleeve_height),
        (sleeve_center[0] + (sleeve_outer[0] / 2.0) - (sleeve_wall / 2.0), sleeve_center[1], sleeve_bottom_z + (sleeve_height / 2.0)),
        material="column_paint",
        name="outer_sleeve_right",
    )
    _add_box(
        base,
        (sleeve_outer[0], sleeve_outer[1], sleeve_wall),
        (sleeve_center[0], sleeve_center[1], sleeve_bottom_z + (sleeve_wall / 2.0)),
        material="column_paint",
        name="outer_sleeve",
    )

    caster_positions = (
        ("caster_0", (-0.160, 0.420)),
        ("caster_1", (0.160, 0.420)),
        ("caster_2", (-0.160, -0.100)),
        ("caster_3", (0.160, -0.100)),
    )
    fork_gap = wheel_thickness + 0.004
    fork_plate = 0.005
    for index, (caster_name, (caster_x, caster_y)) in enumerate(caster_positions):
        _add_box(
            base,
            (0.032, 0.032, 0.020),
            (caster_x, caster_y, 0.060),
            material="base_paint",
            name=f"caster_mount_{index}",
        )
        for side, x_sign in enumerate((-1.0, 1.0)):
            _add_box(
                base,
                (fork_plate, 0.032, 0.048),
                (caster_x + x_sign * ((fork_gap / 2.0) + (fork_plate / 2.0)), caster_y, 0.026),
                material="base_paint",
                name=f"caster_fork_{index}_{side}",
            )

    inner_column = model.part("inner_column")
    _add_box(
        inner_column,
        inner_tube_size,
        (0.000, 0.000, inner_tube_center_z),
        material="column_paint",
        name="inner_tube",
    )
    _add_box(inner_column, (0.007, 0.024, 0.100), (0.0345, 0.000, -0.220), material="tray_trim", name="guide_pad_right")
    _add_box(inner_column, (0.007, 0.024, 0.100), (-0.0345, 0.000, -0.220), material="tray_trim", name="guide_pad_left")
    _add_box(inner_column, (0.038, 0.006, 0.100), (0.000, 0.025, -0.220), material="tray_trim", name="guide_pad_front")
    _add_box(inner_column, (0.038, 0.006, 0.100), (0.000, -0.025, -0.220), material="tray_trim", name="guide_pad_back")
    _add_box(inner_column, (0.074, 0.056, 0.040), (0.000, 0.000, 0.280), material="column_paint", name="head_cap")
    _add_box(inner_column, (0.050, 0.240, 0.032), (0.000, 0.120, 0.274), material="column_paint", name="head_arm")
    _add_box(inner_column, (0.060, 0.010, 0.032), (0.000, 0.145, 0.300), material="column_paint", name="tilt_ear_0")
    _add_box(inner_column, (0.060, 0.010, 0.032), (0.000, 0.175, 0.300), material="column_paint", name="tilt_ear_1")

    tray = model.part("tray")
    _add_box(tray, tray_size, tray_center, material="tray_surface", name="tray_top")
    _add_box(tray, (0.600, 0.016, 0.012), (0.100, 0.192, 0.031), material="tray_trim", name="front_lip")
    _add_box(tray, (0.600, 0.016, 0.012), (0.100, -0.192, 0.031), material="tray_trim", name="rear_lip")
    _add_box(tray, (0.016, 0.368, 0.012), (-0.192, 0.000, 0.031), material="tray_trim", name="left_lip")
    _add_box(tray, (0.016, 0.368, 0.012), (0.392, 0.000, 0.031), material="tray_trim", name="right_lip")
    _add_box(tray, (0.056, 0.022, 0.024), (0.000, 0.000, 0.011), material="tray_trim", name="tilt_block")
    _add_cylinder(tray, 0.008, 0.100, (0.000, 0.000, 0.000), (0.000, pi / 2.0, 0.000), material="tray_trim", name="tilt_barrel")
    _add_box(tray, (0.050, 0.090, 0.012), (0.340, 0.000, 0.001), material="tray_trim", name="paddle_mount")
    _add_box(tray, (0.032, 0.090, 0.028), (0.332, 0.000, 0.009), material="tray_trim", name="paddle_bridge")
    _add_box(tray, (0.012, 0.016, 0.022), (0.360, -0.034, -0.015), material="tray_trim", name="paddle_ear_0")
    _add_box(tray, (0.012, 0.016, 0.022), (0.360, 0.034, -0.015), material="tray_trim", name="paddle_ear_1")
    _add_box(tray, (0.058, 0.100, 0.018), (-0.229, 0.000, 0.014), material="tray_trim", name="wing_bracket")
    _add_box(tray, (0.032, 0.012, 0.010), (-0.249, -0.036, 0.019), material="tray_trim", name="wing_bridge_0")
    _add_box(tray, (0.032, 0.012, 0.010), (-0.249, 0.036, 0.019), material="tray_trim", name="wing_bridge_1")
    _add_box(tray, (0.012, 0.016, 0.030), (-0.270, -0.036, 0.009), material="tray_trim", name="wing_ear_0")
    _add_box(tray, (0.012, 0.016, 0.030), (-0.270, 0.036, 0.009), material="tray_trim", name="wing_ear_1")

    release_paddle = model.part("release_paddle")
    _add_cylinder(
        release_paddle,
        0.006,
        0.056,
        (0.000, 0.000, 0.000),
        (pi / 2.0, 0.000, 0.000),
        material="control_dark",
        name="paddle_pivot",
    )
    _add_box(release_paddle, (0.022, 0.050, 0.030), (0.010, 0.000, -0.020), material="control_dark", name="paddle_neck")
    _add_box(release_paddle, (0.110, 0.035, 0.010), (0.055, 0.000, -0.030), material="control_dark", name="paddle_body")

    wing = model.part("wing")
    _add_cylinder(
        wing,
        0.007,
        0.056,
        (0.000, 0.000, 0.007),
        (pi / 2.0, 0.000, 0.000),
        material="tray_trim",
        name="wing_barrel",
    )
    _add_box(wing, (0.020, 0.060, 0.012), (-0.010, 0.000, 0.012), material="tray_trim", name="wing_neck")
    _add_box(wing, wing_size, (-0.120, 0.000, 0.015), material="tray_surface", name="wing_panel")
    _add_box(wing, (0.220, 0.014, 0.010), (-0.120, 0.123, 0.025), material="tray_trim", name="wing_front_lip")
    _add_box(wing, (0.220, 0.014, 0.010), (-0.120, -0.123, 0.025), material="tray_trim", name="wing_rear_lip")
    _add_box(wing, (0.014, 0.232, 0.010), (-0.223, 0.000, 0.025), material="tray_trim", name="wing_outer_lip")

    for caster_name, _ in caster_positions:
        caster = model.part(caster_name)
        _add_cylinder(caster, wheel_radius, wheel_thickness, (0.000, 0.000, 0.000), (0.000, pi / 2.0, 0.000), material="wheel_rubber", name="wheel")
        _add_cylinder(caster, 0.012, 0.024, (0.000, 0.000, 0.000), (0.000, pi / 2.0, 0.000), material="wheel_hub", name="hub")

    model.articulation(
        "column_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_column,
        origin=Origin(xyz=(sleeve_center[0], sleeve_center[1], sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.200, effort=180.0, velocity=0.10),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=inner_column,
        child=tray,
        origin=Origin(xyz=tray_tilt_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.650, effort=25.0, velocity=1.40),
    )
    model.articulation(
        "paddle_pivot",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=release_paddle,
        origin=Origin(xyz=(0.360, 0.000, -0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.500, effort=4.0, velocity=3.0),
    )
    model.articulation(
        "wing_hinge",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=wing,
        origin=Origin(xyz=(-0.270, 0.000, 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.850, effort=6.0, velocity=1.5),
    )

    for caster_name, (caster_x, caster_y) in ((name, pos) for name, pos in caster_positions):
        model.articulation(
            f"base_to_{caster_name}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster_name,
            origin=Origin(xyz=(caster_x, caster_y, axle_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    inner_column = object_model.get_part("inner_column")
    tray = object_model.get_part("tray")
    wing = object_model.get_part("wing")
    release_paddle = object_model.get_part("release_paddle")

    column_lift = object_model.get_articulation("column_lift")
    tray_tilt = object_model.get_articulation("tray_tilt")
    wing_hinge = object_model.get_articulation("wing_hinge")
    paddle_pivot = object_model.get_articulation("paddle_pivot")

    lift_limits = column_lift.motion_limits
    tilt_limits = tray_tilt.motion_limits
    wing_limits = wing_hinge.motion_limits
    paddle_limits = paddle_pivot.motion_limits

    ctx.expect_contact(
        inner_column,
        base,
        elem_a="guide_pad_right",
        elem_b="outer_sleeve_right",
        name="right guide pad bears on the sleeve",
    )
    ctx.expect_contact(
        inner_column,
        base,
        elem_a="guide_pad_front",
        elem_b="outer_sleeve_front",
        name="front guide pad bears on the sleeve",
    )
    ctx.expect_overlap(
        inner_column,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve_left",
        min_overlap=0.300,
        name="inner tube remains inserted at rest",
    )
    ctx.expect_gap(
        tray,
        inner_column,
        axis="z",
        min_gap=0.010,
        positive_elem="tray_top",
        negative_elem="head_arm",
        name="tray clears the support head at rest",
    )

    rest_column_pos = ctx.part_world_position(inner_column)
    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({column_lift: lift_limits.upper}):
            ctx.expect_contact(
                inner_column,
                base,
                elem_a="guide_pad_right",
                elem_b="outer_sleeve_right",
                name="right guide pad stays engaged when raised",
            )
            ctx.expect_overlap(
                inner_column,
                base,
                axes="z",
                elem_a="inner_tube",
                elem_b="outer_sleeve_left",
                min_overlap=0.100,
                name="inner tube keeps retained insertion when raised",
            )
            raised_column_pos = ctx.part_world_position(inner_column)
        ctx.check(
            "column lift moves the support upward",
            rest_column_pos is not None and raised_column_pos is not None and raised_column_pos[2] > rest_column_pos[2] + 0.18,
            details=f"rest={rest_column_pos}, raised={raised_column_pos}",
        )

    tray_rest_aabb = ctx.part_element_world_aabb(tray, elem="tray_top")
    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({tray_tilt: tilt_limits.upper}):
            tray_tilted_aabb = ctx.part_element_world_aabb(tray, elem="tray_top")
        ctx.check(
            "tray tilt raises the reading surface",
            tray_rest_aabb is not None
            and tray_tilted_aabb is not None
            and tray_tilted_aabb[1][2] > tray_rest_aabb[1][2] + 0.10,
            details=f"rest={tray_rest_aabb}, tilted={tray_tilted_aabb}",
        )

    wing_rest_aabb = ctx.part_element_world_aabb(wing, elem="wing_panel")
    if wing_limits is not None and wing_limits.upper is not None:
        with ctx.pose({wing_hinge: wing_limits.upper}):
            wing_open_aabb = ctx.part_element_world_aabb(wing, elem="wing_panel")
        ctx.check(
            "reading wing can angle upward beside the tray",
            wing_rest_aabb is not None
            and wing_open_aabb is not None
            and wing_open_aabb[1][2] > wing_rest_aabb[1][2] + 0.08,
            details=f"rest={wing_rest_aabb}, open={wing_open_aabb}",
        )

    paddle_rest_aabb = ctx.part_element_world_aabb(release_paddle, elem="paddle_body")
    if paddle_limits is not None and paddle_limits.upper is not None:
        with ctx.pose({paddle_pivot: paddle_limits.upper}):
            paddle_raised_aabb = ctx.part_element_world_aabb(release_paddle, elem="paddle_body")
        ctx.check(
            "release paddle rotates upward under the right side",
            paddle_rest_aabb is not None
            and paddle_raised_aabb is not None
            and paddle_raised_aabb[1][2] > paddle_rest_aabb[1][2] + 0.02,
            details=f"rest={paddle_rest_aabb}, raised={paddle_raised_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
