from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan, pi

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
    model = ArticulatedObject(name="wheelie_bin")

    body_green = model.material("body_green", rgba=(0.18, 0.33, 0.20, 1.0))
    lid_green = model.material("lid_green", rgba=(0.14, 0.27, 0.17, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.05, 0.05, 0.05, 1.0))
    axle_dark = model.material("axle_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.58, 0.60, 0.62, 1.0))

    top_width = 0.74
    bottom_width = 0.58
    top_depth = 0.56
    bottom_depth = 0.42
    wall_bottom_z = 0.03
    body_top_z = 0.82
    wall_height = body_top_z - wall_bottom_z
    wall_thickness = 0.018
    lid_thickness = 0.020
    wheel_radius = 0.145
    wheel_width = 0.060
    axle_radius = 0.018
    axle_y = -0.22
    axle_z = wheel_radius
    wheel_x = 0.39
    hinge_y = -0.285
    hinge_z = 0.845

    side_taper = atan((top_width - bottom_width) * 0.5 / wall_height)
    depth_taper = atan((top_depth - bottom_depth) * 0.5 / wall_height)

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((top_width, top_depth, body_top_z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, body_top_z * 0.5)),
    )

    body.visual(
        Box((bottom_width - 2.0 * wall_thickness, bottom_depth - 2.0 * wall_thickness, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=body_green,
        name="floor_pan",
    )
    body.visual(
        Box((0.18, 0.08, 0.03)),
        origin=Origin(xyz=(0.0, 0.20, 0.015)),
        material=body_green,
        name="front_foot",
    )
    body.visual(
        Box((0.64, wall_thickness, (wall_height**2 + ((top_depth - bottom_depth) * 0.5) ** 2) ** 0.5)),
        origin=Origin(
            xyz=(0.0, 0.245, (body_top_z + wall_bottom_z) * 0.5),
            rpy=(-depth_taper, 0.0, 0.0),
        ),
        material=body_green,
        name="front_shell",
    )
    body.visual(
        Box((0.64, wall_thickness, (wall_height**2 + ((top_depth - bottom_depth) * 0.5) ** 2) ** 0.5)),
        origin=Origin(
            xyz=(0.0, -0.245, (body_top_z + wall_bottom_z) * 0.5),
            rpy=(depth_taper, 0.0, 0.0),
        ),
        material=body_green,
        name="rear_shell",
    )
    body.visual(
        Box((wall_thickness, 0.49, (wall_height**2 + ((top_width - bottom_width) * 0.5) ** 2) ** 0.5)),
        origin=Origin(
            xyz=(0.33, 0.0, (body_top_z + wall_bottom_z) * 0.5),
            rpy=(0.0, side_taper, 0.0),
        ),
        material=body_green,
        name="left_shell",
    )
    body.visual(
        Box((wall_thickness, 0.49, (wall_height**2 + ((top_width - bottom_width) * 0.5) ** 2) ** 0.5)),
        origin=Origin(
            xyz=(-0.33, 0.0, (body_top_z + wall_bottom_z) * 0.5),
            rpy=(0.0, -side_taper, 0.0),
        ),
        material=body_green,
        name="right_shell",
    )
    body.visual(
        Box((0.70, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.265, 0.80)),
        material=body_green,
        name="front_rim",
    )
    body.visual(
        Box((0.03, 0.50, 0.04)),
        origin=Origin(xyz=(0.355, 0.0, 0.80)),
        material=body_green,
        name="left_rim",
    )
    body.visual(
        Box((0.03, 0.50, 0.04)),
        origin=Origin(xyz=(-0.355, 0.0, 0.80)),
        material=body_green,
        name="right_rim",
    )
    body.visual(
        Box((0.22, 0.035, 0.04)),
        origin=Origin(xyz=(0.0, -0.268, 0.785)),
        material=body_green,
        name="rear_hinge_mount",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.18),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=axle_dark,
        name="center_hinge_barrel",
    )
    body.visual(
        Box((0.035, 0.018, 0.07)),
        origin=Origin(xyz=(0.055, -0.285, 0.815)),
        material=body_green,
        name="left_hinge_cheek",
    )
    body.visual(
        Box((0.035, 0.018, 0.07)),
        origin=Origin(xyz=(-0.055, -0.285, 0.815)),
        material=body_green,
        name="right_hinge_cheek",
    )
    body.visual(
        Box((0.08, 0.12, 0.20)),
        origin=Origin(xyz=(0.265, axle_y, 0.19)),
        material=body_green,
        name="left_axle_pod",
    )
    body.visual(
        Box((0.08, 0.12, 0.20)),
        origin=Origin(xyz=(-0.265, axle_y, 0.19)),
        material=body_green,
        name="right_axle_pod",
    )
    body.visual(
        Cylinder(radius=axle_radius, length=0.713),
        origin=Origin(xyz=(0.0, axle_y, axle_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=axle_dark,
        name="rear_axle",
    )
    body.visual(
        Box((0.30, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, -0.29, 0.67)),
        material=body_green,
        name="rear_handle_grip",
    )
    body.visual(
        Box((0.03, 0.02, 0.22)),
        origin=Origin(xyz=(0.11, -0.285, 0.57)),
        material=body_green,
        name="rear_handle_left_post",
    )
    body.visual(
        Box((0.03, 0.02, 0.22)),
        origin=Origin(xyz=(-0.11, -0.285, 0.57)),
        material=body_green,
        name="rear_handle_right_post",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.74, 0.56, 0.08)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.275, 0.0)),
    )
    lid.visual(
        Box((0.74, 0.50, lid_thickness)),
        origin=Origin(xyz=(0.0, 0.275, -0.015)),
        material=lid_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.10, 0.09, 0.028)),
        origin=Origin(xyz=(0.24, 0.022, 0.002)),
        material=lid_green,
        name="left_hinge_web",
    )
    lid.visual(
        Box((0.10, 0.09, 0.028)),
        origin=Origin(xyz=(-0.24, 0.022, 0.002)),
        material=lid_green,
        name="right_hinge_web",
    )
    lid.visual(
        Box((0.68, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.525, -0.003)),
        material=lid_green,
        name="front_lip",
    )
    lid.visual(
        Box((0.022, 0.45, 0.018)),
        origin=Origin(xyz=(0.381, 0.275, -0.014)),
        material=lid_green,
        name="left_skirt",
    )
    lid.visual(
        Box((0.022, 0.45, 0.018)),
        origin=Origin(xyz=(-0.381, 0.275, -0.014)),
        material=lid_green,
        name="right_skirt",
    )
    lid.visual(
        Box((0.20, 0.025, 0.025)),
        origin=Origin(xyz=(0.0, 0.46, 0.002)),
        material=lid_green,
        name="lid_handle",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.14),
        origin=Origin(xyz=(0.24, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=lid_green,
        name="left_lid_barrel",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.14),
        origin=Origin(xyz=(-0.24, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=lid_green,
        name="right_lid_barrel",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.2,
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
    )
    left_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=wheel_black,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.088, length=wheel_width * 0.68),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=hub_gray,
        name="rim",
    )
    left_wheel.visual(
        Cylinder(radius=0.040, length=wheel_width * 1.12),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=axle_dark,
        name="hub",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.2,
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
    )
    right_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=wheel_black,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.088, length=wheel_width * 0.68),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=hub_gray,
        name="rim",
    )
    right_wheel.visual(
        Cylinder(radius=0.040, length=wheel_width * 1.12),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=axle_dark,
        name="hub",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.55),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(wheel_x, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(-wheel_x, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")

    ctx.allow_overlap(
        body,
        left_wheel,
        elem_a="rear_axle",
        elem_b="hub",
        reason="The rear axle intentionally passes through the left wheel hub.",
    )
    ctx.allow_overlap(
        body,
        right_wheel,
        elem_a="rear_axle",
        elem_b="hub",
        reason="The rear axle intentionally passes through the right wheel hub.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_rim",
            max_gap=0.006,
            max_penetration=0.001,
            name="closed lid sits flush on the front rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a="lid_panel",
            elem_b="front_rim",
            min_overlap=0.66,
            name="lid spans nearly the full bin width",
        )
        left_pos = ctx.part_world_position(left_wheel)
        right_pos = ctx.part_world_position(right_wheel)
        ctx.check(
            "rear wheels sit outside the body sides",
            left_pos is not None
            and right_pos is not None
            and left_pos[0] > 0.36
            and right_pos[0] < -0.36,
            details=f"left={left_pos}, right={right_pos}",
        )

    front_lip_rest = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({lid_hinge: 1.25}):
        front_lip_open = ctx.part_element_world_aabb(lid, elem="front_lip")
        ctx.check(
            "lid opens upward from the rear hinge",
            front_lip_rest is not None
            and front_lip_open is not None
            and front_lip_open[1][2] > front_lip_rest[1][2] + 0.18,
            details=f"rest={front_lip_rest}, open={front_lip_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
