from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _handle_mesh(name: str, *, width: float, drop: float, proud: float, tube_radius: float):
    return _mesh(
        name,
        tube_from_spline_points(
            [
                (-width * 0.50, 0.0, 0.0),
                (-width * 0.45, -proud * 0.72, -drop * 0.44),
                (-width * 0.34, -proud, -drop),
                (width * 0.34, -proud, -drop),
                (width * 0.45, -proud * 0.72, -drop * 0.44),
                (width * 0.50, 0.0, 0.0),
            ],
            radius=tube_radius,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rack_router")

    chassis_black = model.material("chassis_black", rgba=(0.12, 0.13, 0.14, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.07, 0.08, 0.09, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    pivot_dark = model.material("pivot_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    switch_black = model.material("switch_black", rgba=(0.04, 0.04, 0.05, 1.0))
    led_green = model.material("led_green", rgba=(0.18, 0.78, 0.30, 1.0))
    led_amber = model.material("led_amber", rgba=(0.90, 0.62, 0.15, 1.0))

    overall_width = 0.4826
    ear_width = 0.024
    body_width = overall_width - 2.0 * ear_width
    body_height = 0.0445
    body_depth = 0.285
    shell_t = 0.0022
    panel_t = 0.0032
    rear_t = 0.0020

    switch_open_w = 0.025
    switch_open_h = 0.016

    handle_width = 0.050
    handle_drop = 0.030
    handle_proud = 0.014
    handle_tube_r = 0.0028
    handle_pivot_z = 0.010
    handle_pivot_y = -0.003
    handle_center_x = body_width * 0.5 - 0.014

    router_body = model.part("router_body")
    router_body.inertial = Inertial.from_geometry(
        Box((overall_width, body_depth, body_height)),
        mass=7.5,
        origin=Origin(xyz=(0.0, body_depth * 0.5, 0.0)),
    )

    # Hollow shallow rack enclosure.
    router_body.visual(
        Box((body_width, body_depth, shell_t)),
        origin=Origin(xyz=(0.0, body_depth * 0.5, body_height * 0.5 - shell_t * 0.5)),
        material=chassis_black,
        name="top_cover",
    )
    router_body.visual(
        Box((body_width, body_depth, shell_t)),
        origin=Origin(xyz=(0.0, body_depth * 0.5, -body_height * 0.5 + shell_t * 0.5)),
        material=chassis_black,
        name="bottom_cover",
    )
    router_body.visual(
        Box((shell_t, body_depth, body_height - 2.0 * shell_t)),
        origin=Origin(xyz=(-body_width * 0.5 + shell_t * 0.5, body_depth * 0.5, 0.0)),
        material=chassis_black,
        name="left_side_wall",
    )
    router_body.visual(
        Box((shell_t, body_depth, body_height - 2.0 * shell_t)),
        origin=Origin(xyz=(body_width * 0.5 - shell_t * 0.5, body_depth * 0.5, 0.0)),
        material=chassis_black,
        name="right_side_wall",
    )
    router_body.visual(
        Box((body_width, rear_t, body_height)),
        origin=Origin(xyz=(0.0, body_depth - rear_t * 0.5, 0.0)),
        material=chassis_black,
        name="rear_panel",
    )

    # Rack ears and their reinforcing front-corner brackets.
    ear_center_x = overall_width * 0.5 - ear_width * 0.5
    ear_brace_depth = 0.030
    ear_brace_height = body_height * 0.72
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        x_center = x_sign * ear_center_x
        router_body.visual(
            Box((ear_width, panel_t, body_height)),
            origin=Origin(xyz=(x_center, panel_t * 0.5, 0.0)),
            material=chassis_black,
            name=f"{side}_rack_ear",
        )
        router_body.visual(
            Box((ear_width, ear_brace_depth, ear_brace_height)),
            origin=Origin(xyz=(x_center, ear_brace_depth * 0.5, 0.0)),
            material=chassis_black,
            name=f"{side}_ear_brace",
        )
        for z_offset, hole_name in ((0.0135, "upper"), (-0.0135, "lower")):
            router_body.visual(
                Cylinder(radius=0.0036, length=0.0016),
                origin=Origin(
                    xyz=(x_center, -0.0008, z_offset),
                    rpy=(pi * 0.5, 0.0, 0.0),
                ),
                material=pivot_dark,
                name=f"{side}_{hole_name}_mount_ring",
            )

    # Front panel frame with a true switch opening.
    front_band_h = (body_height - switch_open_h) * 0.5
    side_strip_w = (body_width - switch_open_w) * 0.5
    side_strip_x = switch_open_w * 0.25 + body_width * 0.25
    front_band_z = switch_open_h * 0.25 + body_height * 0.25
    router_body.visual(
        Box((body_width, panel_t, front_band_h)),
        origin=Origin(xyz=(0.0, panel_t * 0.5, front_band_z)),
        material=chassis_black,
        name="front_top_band",
    )
    router_body.visual(
        Box((body_width, panel_t, front_band_h)),
        origin=Origin(xyz=(0.0, panel_t * 0.5, -front_band_z)),
        material=chassis_black,
        name="front_bottom_band",
    )
    router_body.visual(
        Box((side_strip_w, panel_t, switch_open_h)),
        origin=Origin(xyz=(-side_strip_x, panel_t * 0.5, 0.0)),
        material=chassis_black,
        name="front_left_center_strip",
    )
    router_body.visual(
        Box((side_strip_w, panel_t, switch_open_h)),
        origin=Origin(xyz=(side_strip_x, panel_t * 0.5, 0.0)),
        material=chassis_black,
        name="front_right_center_strip",
    )
    router_body.visual(
        Box((body_width - 0.028, 0.006, body_height - 0.010)),
        origin=Origin(xyz=(0.0, 0.0048, 0.0)),
        material=fascia_black,
        name="inner_fascia",
    )
    router_body.visual(
        Box((0.110, 0.004, 0.012)),
        origin=Origin(xyz=(-0.105, 0.0062, 0.0)),
        material=fascia_black,
        name="left_vent_block",
    )
    router_body.visual(
        Box((0.110, 0.004, 0.012)),
        origin=Origin(xyz=(0.105, 0.0062, 0.0)),
        material=fascia_black,
        name="right_vent_block",
    )
    router_body.visual(
        Cylinder(radius=0.0019, length=0.0026),
        origin=Origin(xyz=(-0.020, -0.0003, -0.011), rpy=(pi * 0.5, 0.0, 0.0)),
        material=led_green,
        name="status_led_power",
    )
    router_body.visual(
        Cylinder(radius=0.0019, length=0.0026),
        origin=Origin(xyz=(0.020, -0.0003, -0.011), rpy=(pi * 0.5, 0.0, 0.0)),
        material=led_amber,
        name="status_led_link",
    )

    pivot_bar_length = handle_width + 0.010
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        router_body.visual(
            Cylinder(radius=0.0030, length=pivot_bar_length),
            origin=Origin(
                xyz=(x_sign * handle_center_x, handle_pivot_y, handle_pivot_z),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=pivot_dark,
            name=f"{side}_handle_pivot_bar",
        )

    left_handle = model.part("left_handle")
    left_handle.inertial = Inertial.from_geometry(
        Box((handle_width, handle_proud + 0.010, handle_drop + 0.008)),
        mass=0.16,
        origin=Origin(xyz=(0.0, -0.008, -handle_drop * 0.45)),
    )
    left_handle.visual(
        _handle_mesh(
            "left_handle_frame",
            width=handle_width,
            drop=handle_drop,
            proud=handle_proud,
            tube_radius=handle_tube_r,
        ),
        material=handle_metal,
        name="handle_frame",
    )

    right_handle = model.part("right_handle")
    right_handle.inertial = Inertial.from_geometry(
        Box((handle_width, handle_proud + 0.010, handle_drop + 0.008)),
        mass=0.16,
        origin=Origin(xyz=(0.0, -0.008, -handle_drop * 0.45)),
    )
    right_handle.visual(
        _handle_mesh(
            "right_handle_frame",
            width=handle_width,
            drop=handle_drop,
            proud=handle_proud,
            tube_radius=handle_tube_r,
        ),
        material=handle_metal,
        name="handle_frame",
    )

    switch_width = 0.019
    switch_depth = 0.006
    switch_height = 0.013

    power_switch = model.part("power_switch")
    power_switch.inertial = Inertial.from_geometry(
        Box((switch_width, switch_depth, switch_height)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.0005, 0.0)),
    )
    power_switch.visual(
        Box((switch_width, switch_depth, switch_height * 0.52)),
        origin=Origin(xyz=(0.0, -0.0007, switch_height * 0.24)),
        material=switch_black,
        name="switch_upper",
    )
    power_switch.visual(
        Box((switch_width, switch_depth, switch_height * 0.52)),
        origin=Origin(xyz=(0.0, -0.0007, -switch_height * 0.24)),
        material=switch_black,
        name="switch_lower",
    )
    power_switch.visual(
        Cylinder(radius=0.0014, length=switch_width * 0.90),
        origin=Origin(xyz=(0.0, 0.0001, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=pivot_dark,
        name="switch_axle",
    )

    model.articulation(
        "left_handle_fold",
        ArticulationType.REVOLUTE,
        parent=router_body,
        child=left_handle,
        origin=Origin(xyz=(-handle_center_x, handle_pivot_y, handle_pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "right_handle_fold",
        ArticulationType.REVOLUTE,
        parent=router_body,
        child=right_handle,
        origin=Origin(xyz=(handle_center_x, handle_pivot_y, handle_pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "power_switch_toggle",
        ArticulationType.REVOLUTE,
        parent=router_body,
        child=power_switch,
        origin=Origin(xyz=(0.0, panel_t * 0.5, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-0.24, upper=0.24),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    router_body = object_model.get_part("router_body")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    power_switch = object_model.get_part("power_switch")

    left_joint = object_model.get_articulation("left_handle_fold")
    right_joint = object_model.get_articulation("right_handle_fold")
    switch_joint = object_model.get_articulation("power_switch_toggle")

    ctx.check(
        "handle hinge axes fold outward from the front panel",
        left_joint.axis == (-1.0, 0.0, 0.0) and right_joint.axis == (-1.0, 0.0, 0.0),
        details=f"left_axis={left_joint.axis}, right_axis={right_joint.axis}",
    )
    ctx.check(
        "power switch rocks on a transverse axis",
        switch_joint.axis == (1.0, 0.0, 0.0),
        details=f"switch_axis={switch_joint.axis}",
    )
    ctx.expect_origin_distance(
        power_switch,
        router_body,
        axes="xz",
        max_dist=0.003,
        name="power switch stays near the front-panel center",
    )

    left_rest_aabb = ctx.part_world_aabb(left_handle)
    right_rest_aabb = ctx.part_world_aabb(right_handle)
    left_pos = ctx.part_world_position(left_handle)
    right_pos = ctx.part_world_position(right_handle)
    ctx.check(
        "handles are mirrored at the front corners",
        left_pos is not None
        and right_pos is not None
        and abs(left_pos[0] + right_pos[0]) < 0.003
        and abs(left_pos[2] - right_pos[2]) < 0.001
        and left_rest_aabb is not None
        and right_rest_aabb is not None
        and abs((left_rest_aabb[0][2] + left_rest_aabb[1][2]) - (right_rest_aabb[0][2] + right_rest_aabb[1][2])) < 0.002,
        details=f"left_pos={left_pos}, right_pos={right_pos}, left_aabb={left_rest_aabb}, right_aabb={right_rest_aabb}",
    )

    left_upper = left_joint.motion_limits.upper if left_joint.motion_limits is not None else None
    right_upper = right_joint.motion_limits.upper if right_joint.motion_limits is not None else None
    if left_rest_aabb is not None and left_upper is not None:
        with ctx.pose({left_joint: left_upper}):
            left_open_aabb = ctx.part_world_aabb(left_handle)
        ctx.check(
            "left handle swings outward when opened",
            left_open_aabb is not None and left_open_aabb[0][1] < left_rest_aabb[0][1] - 0.012,
            details=f"rest={left_rest_aabb}, open={left_open_aabb}",
        )
    if right_rest_aabb is not None and right_upper is not None:
        with ctx.pose({right_joint: right_upper}):
            right_open_aabb = ctx.part_world_aabb(right_handle)
        ctx.check(
            "right handle swings outward when opened",
            right_open_aabb is not None and right_open_aabb[0][1] < right_rest_aabb[0][1] - 0.012,
            details=f"rest={right_rest_aabb}, open={right_open_aabb}",
        )

    switch_lower = switch_joint.motion_limits.lower if switch_joint.motion_limits is not None else None
    switch_upper = switch_joint.motion_limits.upper if switch_joint.motion_limits is not None else None
    if switch_lower is not None and switch_upper is not None:
        upper_neg = None
        upper_pos = None
        with ctx.pose({switch_joint: switch_lower}):
            upper_neg = ctx.part_element_world_aabb(power_switch, elem="switch_upper")
        with ctx.pose({switch_joint: switch_upper}):
            upper_pos = ctx.part_element_world_aabb(power_switch, elem="switch_upper")
        ctx.check(
            "switch upper rocker edge tips forward in one toggle direction",
            upper_neg is not None and upper_pos is not None and upper_pos[0][1] < upper_neg[0][1] - 0.0012,
            details=f"negative_pose={upper_neg}, positive_pose={upper_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
