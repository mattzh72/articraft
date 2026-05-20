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


def _add_wheel(part, *, rubber, hub_plastic, hubcap, radius: float, width: float) -> None:
    spin_origin = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=radius, length=width),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=radius * 0.58, length=width + 0.012),
        origin=spin_origin,
        material=hub_plastic,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=radius * 0.21, length=width + 0.016),
        origin=spin_origin,
        material=hubcap,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    wall_green = model.material("wall_green", rgba=(0.18, 0.36, 0.22, 1.0))
    lid_green = model.material("lid_green", rgba=(0.16, 0.33, 0.20, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.05, 0.05, 0.05, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.28, 0.30, 0.31, 1.0))
    hubcap_gray = model.material("hubcap_gray", rgba=(0.56, 0.58, 0.60, 1.0))

    wall_t = 0.018
    body_height = 0.84
    top_depth = 0.58
    bottom_depth = 0.42
    top_width = 0.48
    bottom_width = 0.38
    bottom_t = 0.020
    shell_base_z = 0.045
    front_back_pitch = atan(((top_depth - bottom_depth) * 0.5) / body_height)
    side_roll = atan(((top_width - bottom_width) * 0.5) / body_height)

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.62, 0.54, 0.98)),
        mass=12.5,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    body.visual(
        Box((bottom_depth + 0.006, bottom_width + 0.006, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_base_z + bottom_t * 0.5)),
        material=wall_green,
        name="floor_pan",
    )
    body.visual(
        Box((wall_t, top_width, body_height)),
        origin=Origin(
            xyz=((top_depth + bottom_depth) * 0.25, 0.0, shell_base_z + body_height * 0.5),
            rpy=(0.0, front_back_pitch, 0.0),
        ),
        material=wall_green,
        name="front_panel",
    )
    body.visual(
        Box((wall_t, top_width - 0.030, body_height * 0.92)),
        origin=Origin(
            xyz=(-(top_depth + bottom_depth) * 0.25 + 0.004, 0.0, shell_base_z + body_height * 0.46),
            rpy=(0.0, -front_back_pitch * 0.92, 0.0),
        ),
        material=wall_green,
        name="rear_panel",
    )
    body.visual(
        Box((top_depth - 0.060, wall_t, body_height)),
        origin=Origin(
            xyz=(0.0, (top_width + bottom_width) * 0.25, shell_base_z + body_height * 0.5),
            rpy=(-side_roll, 0.0, 0.0),
        ),
        material=wall_green,
        name="left_panel",
    )
    body.visual(
        Box((top_depth - 0.060, wall_t, body_height)),
        origin=Origin(
            xyz=(0.0, -(top_width + bottom_width) * 0.25, shell_base_z + body_height * 0.5),
            rpy=(side_roll, 0.0, 0.0),
        ),
        material=wall_green,
        name="right_panel",
    )

    rim_z = shell_base_z + body_height - 0.010
    body.visual(
        Box((0.062, top_width + 0.004, 0.018)),
        origin=Origin(xyz=(top_depth * 0.5 - 0.018, 0.0, rim_z)),
        material=wall_green,
        name="front_rim",
    )
    body.visual(
        Box((top_depth - 0.024, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, top_width * 0.5 - 0.006, rim_z)),
        material=wall_green,
        name="left_rim",
    )
    body.visual(
        Box((top_depth - 0.024, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -top_width * 0.5 + 0.006, rim_z)),
        material=wall_green,
        name="right_rim",
    )
    body.visual(
        Box((0.080, top_width - 0.070, 0.050)),
        origin=Origin(xyz=(-top_depth * 0.5 + 0.018, 0.0, shell_base_z + body_height - 0.043)),
        material=wall_green,
        name="rear_handle_rail",
    )
    for sign, name in ((1.0, "left_hinge_pad"), (-1.0, "right_hinge_pad")):
        body.visual(
            Box((0.055, 0.030, 0.012)),
            origin=Origin(xyz=(-0.250, sign * 0.216, 0.879)),
            material=wall_green,
            name=name,
        )

    axle_center = (-0.185, 0.0, 0.115)
    axle_length = 0.530
    body.visual(
        Cylinder(radius=0.014, length=axle_length),
        origin=Origin(xyz=axle_center, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="rear_axle",
    )
    arm_y = bottom_width * 0.5 + 0.0375
    for sign, name in ((1.0, "left_axle_arm"), (-1.0, "right_axle_arm")):
        body.visual(
            Box((0.082, 0.035, 0.190)),
            origin=Origin(xyz=(-0.185, sign * arm_y, 0.140)),
            material=wall_green,
            name=name,
        )
    for sign, name in ((1.0, "left_foot"), (-1.0, "right_foot")):
        body.visual(
            Box((0.088, 0.036, shell_base_z + 0.006)),
            origin=Origin(xyz=(0.110, sign * 0.128, (shell_base_z + 0.006) * 0.5)),
            material=wall_green,
            name=name,
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.64, 0.52, 0.080)),
        mass=2.8,
        origin=Origin(xyz=(0.320, 0.0, -0.005)),
    )
    lid.visual(
        Cylinder(radius=0.011, length=0.430),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=lid_green,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.620, 0.505, 0.018)),
        origin=Origin(xyz=(0.310, 0.0, -0.004)),
        material=lid_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.030, 0.505, 0.052)),
        origin=Origin(xyz=(0.635, 0.0, -0.021)),
        material=lid_green,
        name="front_skirt",
    )
    lid.visual(
        Box((0.600, 0.020, 0.045)),
        origin=Origin(xyz=(0.302, 0.2625, -0.018)),
        material=lid_green,
        name="left_skirt",
    )
    lid.visual(
        Box((0.600, 0.020, 0.045)),
        origin=Origin(xyz=(0.302, -0.2625, -0.018)),
        material=lid_green,
        name="right_skirt",
    )
    lid.visual(
        Box((0.120, 0.120, 0.028)),
        origin=Origin(xyz=(0.500, 0.0, 0.019)),
        material=lid_green,
        name="lid_grip",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.042),
        mass=1.4,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel(
        left_wheel,
        rubber=wheel_black,
        hub_plastic=hub_gray,
        hubcap=hubcap_gray,
        radius=0.115,
        width=0.042,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.042),
        mass=1.4,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel(
        right_wheel,
        rubber=wheel_black,
        hub_plastic=hub_gray,
        hubcap=hubcap_gray,
        radius=0.115,
        width=0.042,
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-top_depth * 0.5 - 0.010, 0.0, shell_base_z + body_height + 0.013)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-0.185, 0.292, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(-0.185, -0.292, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_contact(
        left_wheel,
        body,
        elem_a="hub_shell",
        elem_b="rear_axle",
        name="left wheel mounts onto rear axle",
    )
    ctx.expect_contact(
        right_wheel,
        body,
        elem_a="hub_shell",
        elem_b="rear_axle",
        name="right wheel mounts onto rear axle",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.012,
            max_penetration=0.0,
            positive_elem="lid_panel",
            negative_elem="front_rim",
            name="closed lid sits close to the bin rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.40,
            name="lid covers the body opening in plan view",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({lid_hinge: 1.20}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
