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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    body_plastic = model.material("body_plastic", rgba=(0.18, 0.27, 0.22, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.23, 0.34, 0.28, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.32, 0.33, 0.35, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.52, 0.46, 0.98)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    wall_thickness = 0.012
    wall_height = 0.76
    side_tilt = 0.075
    front_tilt = 0.085

    body.visual(
        Box((0.42, 0.34, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        material=body_plastic,
        name="floor_panel",
    )
    body.visual(
        Box((0.012, 0.40, wall_height)),
        origin=Origin(xyz=(0.230, 0.0, 0.509), rpy=(0.0, front_tilt, 0.0)),
        material=body_plastic,
        name="front_wall",
    )
    body.visual(
        Box((0.012, 0.40, wall_height)),
        origin=Origin(xyz=(-0.230, 0.0, 0.509), rpy=(0.0, -front_tilt, 0.0)),
        material=body_plastic,
        name="back_wall",
    )
    body.visual(
        Box((0.46, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, 0.190, 0.509), rpy=(-side_tilt, 0.0, 0.0)),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((0.46, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, -0.190, 0.509), rpy=(side_tilt, 0.0, 0.0)),
        material=body_plastic,
        name="right_wall",
    )

    body.visual(
        Box((0.480, 0.020, 0.028)),
        origin=Origin(xyz=(0.006, 0.225, 0.892)),
        material=body_plastic,
        name="left_top_frame",
    )
    body.visual(
        Box((0.480, 0.020, 0.028)),
        origin=Origin(xyz=(0.006, -0.225, 0.892)),
        material=body_plastic,
        name="right_top_frame",
    )
    body.visual(
        Box((0.020, 0.436, 0.028)),
        origin=Origin(xyz=(0.246, 0.0, 0.892)),
        material=body_plastic,
        name="front_top_frame",
    )
    body.visual(
        Box((0.020, 0.452, 0.028)),
        origin=Origin(xyz=(-0.236, 0.0, 0.892)),
        material=body_plastic,
        name="rear_top_frame",
    )

    body.visual(
        Box((0.060, 0.110, 0.150)),
        origin=Origin(xyz=(-0.205, 0.145, 0.165)),
        material=body_plastic,
        name="left_axle_support",
    )
    body.visual(
        Box((0.060, 0.110, 0.150)),
        origin=Origin(xyz=(-0.205, -0.145, 0.165)),
        material=body_plastic,
        name="right_axle_support",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.400),
        origin=Origin(xyz=(-0.225, 0.0, 0.110), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="rear_axle",
    )
    body.visual(
        Box((0.070, 0.380, 0.020)),
        origin=Origin(xyz=(-0.205, 0.0, 0.195)),
        material=body_plastic,
        name="axle_bridge",
    )
    body.visual(
        Box((0.060, 0.065, 0.130)),
        origin=Origin(xyz=(0.185, 0.155, 0.065)),
        material=body_plastic,
        name="left_front_foot",
    )
    body.visual(
        Box((0.060, 0.065, 0.130)),
        origin=Origin(xyz=(0.185, -0.155, 0.065)),
        material=body_plastic,
        name="right_front_foot",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.400),
        origin=Origin(xyz=(-0.240, 0.0, 0.909), rpy=(pi / 2.0, 0.0, 0.0)),
        material=body_plastic,
        name="hinge_bar",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.56, 0.48, 0.07)),
        mass=2.3,
        origin=Origin(xyz=(0.28, 0.0, 0.0)),
    )
    lid.visual(
        Box((0.500, 0.468, 0.018)),
        origin=Origin(xyz=(0.290, 0.0, 0.009)),
        material=lid_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.025, 0.444, 0.055)),
        origin=Origin(xyz=(0.552, 0.0, -0.018)),
        material=lid_plastic,
        name="front_lip",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.450),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=lid_plastic,
        name="hinge_roll",
    )
    lid.visual(
        Box((0.050, 0.028, 0.018)),
        origin=Origin(xyz=(0.022, 0.218, 0.004)),
        material=lid_plastic,
        name="left_hinge_strap",
    )
    lid.visual(
        Box((0.050, 0.028, 0.018)),
        origin=Origin(xyz=(0.022, -0.218, 0.004)),
        material=lid_plastic,
        name="right_hinge_strap",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.040),
        mass=1.4,
    )
    left_wheel.visual(
        Cylinder(radius=0.115, length=0.040),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_tire",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.040),
        mass=1.4,
    )
    right_wheel.visual(
        Cylinder(radius=0.115, length=0.040),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_tire",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.264, 0.0, 0.909)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-0.225, 0.220, 0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(-0.225, -0.220, 0.110)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_joint = object_model.get_articulation("body_to_lid")
    left_wheel_joint = object_model.get_articulation("body_to_left_wheel")
    right_wheel_joint = object_model.get_articulation("body_to_right_wheel")

    ctx.expect_contact(
        lid,
        body,
        elem_a="hinge_roll",
        elem_b="hinge_bar",
        name="lid hinge roll contacts the rear hinge bar",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_top_frame",
        max_gap=0.012,
        max_penetration=0.0,
        name="closed lid sits just above front frame",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="y",
        elem_a="lid_panel",
        elem_b="front_top_frame",
        min_overlap=0.42,
        name="lid spans the full bin width",
    )
    ctx.expect_contact(
        left_wheel,
        body,
        elem_a="left_tire",
        elem_b="left_axle_support",
        name="left wheel is mounted against the left axle support",
    )
    ctx.expect_contact(
        right_wheel,
        body,
        elem_a="right_tire",
        elem_b="right_axle_support",
        name="right wheel is mounted against the right axle support",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    left_wheel_rest = ctx.part_world_position(left_wheel)
    right_wheel_rest = ctx.part_world_position(right_wheel)
    with ctx.pose({lid_joint: 1.2, left_wheel_joint: 1.4, right_wheel_joint: -1.4}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_lip")
        left_wheel_spun = ctx.part_world_position(left_wheel)
        right_wheel_spun = ctx.part_world_position(right_wheel)
    ctx.check(
        "lid opens upward from the front edge",
        closed_front is not None
        and opened_front is not None
        and opened_front[1][2] > closed_front[1][2] + 0.18,
        details=f"closed={closed_front}, open={opened_front}",
    )
    ctx.check(
        "wheels spin in place about the axle",
        left_wheel_rest is not None
        and right_wheel_rest is not None
        and left_wheel_spun is not None
        and right_wheel_spun is not None
        and max(abs(a - b) for a, b in zip(left_wheel_rest, left_wheel_spun)) < 1e-6
        and max(abs(a - b) for a, b in zip(right_wheel_rest, right_wheel_spun)) < 1e-6,
        details=(
            f"left_rest={left_wheel_rest}, left_spun={left_wheel_spun}, "
            f"right_rest={right_wheel_rest}, right_spun={right_wheel_spun}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
