from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    base_paint = model.material("base_paint", rgba=(0.20, 0.22, 0.24, 1.0))
    bearing_gray = model.material("bearing_gray", rgba=(0.46, 0.48, 0.50, 1.0))
    frame_metal = model.material("frame_metal", rgba=(0.68, 0.71, 0.74, 1.0))
    stainless = model.material("stainless", rgba=(0.84, 0.86, 0.88, 1.0))
    trim_black = model.material("trim_black", rgba=(0.07, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.22, 1.22, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=base_paint,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.19, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=base_paint,
        name="pedestal_shell",
    )
    base.visual(
        Cylinder(radius=0.23, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.61)),
        material=trim_black,
        name="upper_mount_flange",
    )
    base.inertial = Inertial.from_geometry(
        Box((1.22, 1.22, 0.65)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
    )

    bearing_module = model.part("bearing_module")
    bearing_module.visual(
        Cylinder(radius=0.17, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=bearing_gray,
        name="bearing_body",
    )
    bearing_module.visual(
        Cylinder(radius=0.21, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=trim_black,
        name="bearing_cap",
    )
    bearing_module.inertial = Inertial.from_geometry(
        Cylinder(radius=0.21, length=0.16),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    fixed_frame = model.part("fixed_frame")
    post_size = 0.06
    frame_half_x = 0.56
    frame_half_y = 0.56
    frame_height = 1.10
    top_rail_z = 1.07
    bottom_rail_z = 0.13
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            fixed_frame.visual(
                Box((post_size, post_size, frame_height)),
                origin=Origin(xyz=(x_sign * frame_half_x, y_sign * frame_half_y, frame_height * 0.5)),
                material=frame_metal,
                name=f"post_{'l' if x_sign < 0 else 'r'}_{'f' if y_sign < 0 else 'b'}",
            )
    for y_sign in (-1.0, 1.0):
        fixed_frame.visual(
            Box((frame_half_x * 2.0, post_size, post_size)),
            origin=Origin(xyz=(0.0, y_sign * frame_half_y, top_rail_z)),
            material=frame_metal,
            name=f"top_cross_{'front' if y_sign < 0 else 'back'}",
        )
        fixed_frame.visual(
            Box((frame_half_x * 2.0, post_size, post_size)),
            origin=Origin(xyz=(0.0, y_sign * frame_half_y, bottom_rail_z)),
            material=frame_metal,
            name=f"bottom_cross_{'front' if y_sign < 0 else 'back'}",
        )
    for x_sign in (-1.0, 1.0):
        fixed_frame.visual(
            Box((post_size, frame_half_y * 2.0, post_size)),
            origin=Origin(xyz=(x_sign * frame_half_x, 0.0, top_rail_z)),
            material=frame_metal,
            name=f"top_side_{'left' if x_sign < 0 else 'right'}",
        )
        fixed_frame.visual(
            Box((post_size, frame_half_y * 2.0, post_size)),
            origin=Origin(xyz=(x_sign * frame_half_x, 0.0, bottom_rail_z)),
            material=frame_metal,
            name=f"bottom_side_{'left' if x_sign < 0 else 'right'}",
        )
    fixed_frame.visual(
        Box((frame_half_x * 2.0, post_size, post_size)),
        origin=Origin(xyz=(0.0, 0.0, top_rail_z)),
        material=trim_black,
        name="top_center_beam",
    )
    fixed_frame.inertial = Inertial.from_geometry(
        Box((1.18, 1.18, 1.10)),
        mass=54.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    head_module_left = model.part("head_module_left")
    head_module_left.visual(
        Box((0.22, 0.26, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=base_paint,
        name="left_head_body",
    )
    head_module_left.visual(
        Box((0.08, 0.14, 0.03)),
        origin=Origin(xyz=(0.07, 0.0, 0.015)),
        material=trim_black,
        name="left_sensor_nose",
    )
    head_module_left.inertial = Inertial.from_geometry(
        Box((0.22, 0.26, 0.12)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    head_module_right = model.part("head_module_right")
    head_module_right.visual(
        Box((0.22, 0.26, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=base_paint,
        name="right_head_body",
    )
    head_module_right.visual(
        Box((0.08, 0.14, 0.03)),
        origin=Origin(xyz=(-0.07, 0.0, 0.015)),
        material=trim_black,
        name="right_sensor_nose",
    )
    head_module_right.inertial = Inertial.from_geometry(
        Box((0.22, 0.26, 0.12)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    arm_hub = model.part("arm_hub")
    arm_hub.visual(
        Cylinder(radius=0.055, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=trim_black,
        name="hub_spindle",
    )
    arm_hub.visual(
        Cylinder(radius=0.095, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=base_paint,
        name="hub_drum",
    )
    arm_hub.visual(
        Cylinder(radius=0.11, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=stainless,
        name="hub_top_plate",
    )
    arm_radius = 0.026
    arm_length = 0.44
    arm_center_radius = 0.22
    for index in range(3):
        angle = 2.0 * math.pi * index / 3.0
        arm_hub.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(
                xyz=(math.cos(angle) * arm_center_radius, math.sin(angle) * arm_center_radius, 0.09),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=stainless,
            name=f"arm_{index}",
        )
        arm_hub.visual(
            Cylinder(radius=0.031, length=0.07),
            origin=Origin(
                xyz=(math.cos(angle) * 0.42, math.sin(angle) * 0.42, 0.09),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=trim_black,
            name=f"arm_tip_{index}",
        )
    arm_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.24, length=0.15),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    model.articulation(
        "base_to_bearing",
        ArticulationType.FIXED,
        parent=base,
        child=bearing_module,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
    )
    model.articulation(
        "base_to_frame",
        ArticulationType.FIXED,
        parent=base,
        child=fixed_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )
    model.articulation(
        "frame_to_head_left",
        ArticulationType.FIXED,
        parent=fixed_frame,
        child=head_module_left,
        origin=Origin(xyz=(-0.42, 0.0, 0.92)),
    )
    model.articulation(
        "frame_to_head_right",
        ArticulationType.FIXED,
        parent=fixed_frame,
        child=head_module_right,
        origin=Origin(xyz=(0.42, 0.0, 0.92)),
    )
    model.articulation(
        "bearing_to_hub",
        ArticulationType.CONTINUOUS,
        parent=bearing_module,
        child=arm_hub,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=4.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bearing_module = object_model.get_part("bearing_module")
    fixed_frame = object_model.get_part("fixed_frame")
    head_module_left = object_model.get_part("head_module_left")
    head_module_right = object_model.get_part("head_module_right")
    arm_hub = object_model.get_part("arm_hub")
    hub_spin = object_model.get_articulation("bearing_to_hub")

    ctx.expect_contact(base, bearing_module, name="bearing module seats on the base pedestal")
    ctx.expect_contact(base, fixed_frame, name="fixed frame stands on the floor plate")
    ctx.expect_contact(fixed_frame, head_module_left, name="left head module mounts to the fixed frame")
    ctx.expect_contact(fixed_frame, head_module_right, name="right head module mounts to the fixed frame")
    ctx.expect_gap(
        head_module_left,
        arm_hub,
        axis="z",
        min_gap=0.02,
        name="head modules stay above the rotating arm assembly",
    )
    ctx.expect_gap(
        arm_hub,
        bearing_module,
        axis="z",
        min_gap=0.0,
        max_gap=0.0,
        name="hub spindle sits directly on the bearing cap",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[index] + high[index]) * 0.5 for index in range(3))

    rest_arm_center = _aabb_center(ctx.part_element_world_aabb(arm_hub, elem="arm_0"))
    with ctx.pose({hub_spin: 2.0 * math.pi / 3.0}):
        turned_arm_center = _aabb_center(ctx.part_element_world_aabb(arm_hub, elem="arm_0"))

    ctx.check(
        "continuous hub rotation advances the tripod arms around the vertical axis",
        rest_arm_center is not None
        and turned_arm_center is not None
        and turned_arm_center[1] > rest_arm_center[1] + 0.15
        and turned_arm_center[0] < rest_arm_center[0] - 0.25,
        details=f"rest_center={rest_arm_center}, turned_center={turned_arm_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
