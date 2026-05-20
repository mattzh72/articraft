from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_motion_tv_wall_mount")

    powder_black = model.material("powder_black", rgba=(0.015, 0.017, 0.018, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.38, 0.39, 0.38, 1.0))
    screw_black = model.material("screw_black", rgba=(0.005, 0.005, 0.006, 1.0))

    # The root frame is the vertical wall-side swing axis.  +X points away from
    # the wall, +Z is up, and the empty screen side would be beyond the head.
    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.040, 0.340, 0.460)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=powder_black,
        name="wall_backplate",
    )
    wall_plate.visual(
        Box((0.018, 0.220, 0.030)),
        origin=Origin(xyz=(0.006, 0.0, 0.165)),
        material=dark_steel,
        name="upper_rib",
    )
    wall_plate.visual(
        Box((0.018, 0.220, 0.030)),
        origin=Origin(xyz=(0.006, 0.0, -0.165)),
        material=dark_steel,
        name="lower_rib",
    )
    wall_plate.visual(
        Cylinder(radius=0.030, length=0.360),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=hinge_steel,
        name="wall_pivot_post",
    )
    for index, (y, z) in enumerate(
        ((-0.120, 0.170), (0.120, 0.170), (-0.120, -0.170), (0.120, -0.170))
    ):
        wall_plate.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(0.004, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=screw_black,
            name=f"wall_screw_{index}",
        )

    wall_arm = model.part("wall_arm")
    wall_arm.visual(
        Box((0.390, 0.090, 0.050)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=dark_steel,
        name="wall_arm_bar",
    )
    wall_arm.visual(
        Box((0.090, 0.026, 0.130)),
        origin=Origin(xyz=(0.035, -0.043, 0.0)),
        material=dark_steel,
        name="wall_clevis_0",
    )
    wall_arm.visual(
        Box((0.090, 0.026, 0.130)),
        origin=Origin(xyz=(0.035, 0.043, 0.0)),
        material=dark_steel,
        name="wall_clevis_1",
    )
    wall_arm.visual(
        Cylinder(radius=0.032, length=0.110),
        origin=Origin(xyz=(0.460, 0.0, 0.0)),
        material=hinge_steel,
        name="elbow_pivot_barrel",
    )
    wall_arm.visual(
        Box((0.040, 0.070, 0.040)),
        origin=Origin(xyz=(0.430, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_neck",
    )

    head_arm = model.part("head_arm")
    head_arm.visual(
        Box((0.350, 0.090, 0.050)),
        origin=Origin(xyz=(0.235, 0.0, 0.0)),
        material=dark_steel,
        name="head_arm_bar",
    )
    head_arm.visual(
        Box((0.090, 0.026, 0.130)),
        origin=Origin(xyz=(0.035, -0.045, 0.0)),
        material=dark_steel,
        name="elbow_clevis_0",
    )
    head_arm.visual(
        Box((0.090, 0.026, 0.130)),
        origin=Origin(xyz=(0.035, 0.045, 0.0)),
        material=dark_steel,
        name="elbow_clevis_1",
    )
    head_arm.visual(
        Cylinder(radius=0.032, length=0.110),
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        material=hinge_steel,
        name="swivel_pivot_barrel",
    )
    head_arm.visual(
        Box((0.040, 0.070, 0.040)),
        origin=Origin(xyz=(0.390, 0.0, 0.0)),
        material=dark_steel,
        name="swivel_neck",
    )

    swivel_support = model.part("swivel_support")
    swivel_support.visual(
        Box((0.090, 0.026, 0.130)),
        origin=Origin(xyz=(0.035, -0.045, 0.0)),
        material=dark_steel,
        name="swivel_clevis_0",
    )
    swivel_support.visual(
        Box((0.090, 0.026, 0.130)),
        origin=Origin(xyz=(0.035, 0.045, 0.0)),
        material=dark_steel,
        name="swivel_clevis_1",
    )
    swivel_support.visual(
        Box((0.080, 0.070, 0.060)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=dark_steel,
        name="swivel_bridge",
    )
    swivel_support.visual(
        Box((0.060, 0.025, 0.120)),
        origin=Origin(xyz=(0.150, -0.0525, 0.0)),
        material=dark_steel,
        name="tilt_fork_0",
    )
    swivel_support.visual(
        Box((0.060, 0.025, 0.120)),
        origin=Origin(xyz=(0.150, 0.0525, 0.0)),
        material=dark_steel,
        name="tilt_fork_1",
    )
    swivel_support.visual(
        Box((0.030, 0.120, 0.045)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=dark_steel,
        name="fork_backstrap",
    )

    tilt_support = model.part("tilt_support")
    tilt_support.visual(
        Cylinder(radius=0.024, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="tilt_axle",
    )
    tilt_support.visual(
        Box((0.050, 0.060, 0.050)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=dark_steel,
        name="tilt_hub_block",
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        Box((0.040, 0.420, 0.036)),
        origin=Origin(xyz=(0.070, 0.0, 0.150)),
        material=powder_black,
        name="top_rail",
    )
    head_frame.visual(
        Box((0.040, 0.420, 0.036)),
        origin=Origin(xyz=(0.070, 0.0, -0.150)),
        material=powder_black,
        name="bottom_rail",
    )
    head_frame.visual(
        Box((0.040, 0.036, 0.300)),
        origin=Origin(xyz=(0.070, -0.192, 0.0)),
        material=powder_black,
        name="side_rail_0",
    )
    head_frame.visual(
        Box((0.040, 0.036, 0.300)),
        origin=Origin(xyz=(0.070, 0.192, 0.0)),
        material=powder_black,
        name="side_rail_1",
    )
    head_frame.visual(
        Box((0.040, 0.275, 0.030)),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material=powder_black,
        name="middle_rail",
    )
    head_frame.visual(
        Box((0.040, 0.040, 0.270)),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material=powder_black,
        name="center_spine",
    )
    for index, (y, z) in enumerate(
        ((-0.160, 0.115), (0.160, 0.115), (-0.160, -0.115), (0.160, -0.115))
    ):
        head_frame.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(0.093, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_steel,
            name=f"frame_bolt_{index}",
        )

    model.articulation(
        "wall_to_wall_arm",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=wall_arm,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "wall_arm_to_head_arm",
        ArticulationType.REVOLUTE,
        parent=wall_arm,
        child=head_arm,
        origin=Origin(xyz=(0.460, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=-2.45, upper=2.45),
    )
    model.articulation(
        "head_arm_to_swivel",
        ArticulationType.REVOLUTE,
        parent=head_arm,
        child=swivel_support,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.3, lower=-1.40, upper=1.40),
    )
    model.articulation(
        "swivel_to_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_support,
        child=tilt_support,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "tilt_to_frame",
        ArticulationType.FIXED,
        parent=tilt_support,
        child=head_frame,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_yaw = object_model.get_articulation("wall_to_wall_arm")
    elbow_yaw = object_model.get_articulation("wall_arm_to_head_arm")
    head_yaw = object_model.get_articulation("head_arm_to_swivel")
    head_tilt = object_model.get_articulation("swivel_to_tilt")

    ctx.check(
        "vertical yaw axes",
        wall_yaw.axis == (0.0, 0.0, 1.0)
        and elbow_yaw.axis == (0.0, 0.0, 1.0)
        and head_yaw.axis == (0.0, 0.0, 1.0),
        details=f"axes={wall_yaw.axis}, {elbow_yaw.axis}, {head_yaw.axis}",
    )
    ctx.check(
        "horizontal tilt axis",
        head_tilt.axis == (0.0, 1.0, 0.0),
        details=f"axis={head_tilt.axis}",
    )

    ctx.expect_contact(
        "wall_arm",
        "wall_plate",
        elem_a="wall_clevis_1",
        elem_b="wall_pivot_post",
        contact_tol=1e-6,
        name="wall clevis locates on post",
    )
    ctx.expect_contact(
        "head_arm",
        "wall_arm",
        elem_a="elbow_clevis_1",
        elem_b="elbow_pivot_barrel",
        contact_tol=1e-6,
        name="elbow clevis locates on barrel",
    )
    ctx.expect_overlap(
        "wall_arm",
        "head_arm",
        axes="xz",
        elem_a="elbow_pivot_barrel",
        elem_b="elbow_clevis_0",
        min_overlap=0.040,
        name="elbow clevis wraps pivot",
    )
    ctx.expect_contact(
        "swivel_support",
        "tilt_support",
        elem_a="tilt_fork_1",
        elem_b="tilt_axle",
        contact_tol=1e-6,
        name="tilt fork locates axle",
    )
    ctx.expect_contact(
        "head_frame",
        "tilt_support",
        elem_a="center_spine",
        elem_b="tilt_hub_block",
        contact_tol=1e-6,
        name="tilt hub supports frame",
    )

    rest_aabb = ctx.part_world_aabb("head_frame")
    with ctx.pose({"wall_to_wall_arm": 0.65, "wall_arm_to_head_arm": -0.55, "head_arm_to_swivel": 0.35}):
        folded_aabb = ctx.part_world_aabb("head_frame")
    if rest_aabb is not None and folded_aabb is not None:
        rest_center_y = (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
        folded_center_y = (folded_aabb[0][1] + folded_aabb[1][1]) / 2.0
        rest_span_x = rest_aabb[1][0] - rest_aabb[0][0]
        folded_span_x = folded_aabb[1][0] - folded_aabb[0][0]
        motion_ok = abs(folded_center_y - rest_center_y) > 0.20 and folded_span_x > 0.20
    else:
        motion_ok = False
    ctx.check(
        "folding arm changes head pose",
        motion_ok,
        details=f"rest_aabb={rest_aabb}, folded_aabb={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
