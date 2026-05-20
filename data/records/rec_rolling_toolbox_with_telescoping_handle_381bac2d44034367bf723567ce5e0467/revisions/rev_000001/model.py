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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="contractor_rolling_toolbox")

    safety_yellow = model.material("safety_yellow", rgba=(0.94, 0.64, 0.10, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.025, 0.028, 0.030, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    bare_metal = model.material("bare_metal", rgba=(0.58, 0.58, 0.54, 1.0))

    shell = model.part("lower_shell")

    # Tall open lower body: a molded jobsite-box tub with a separate front frame
    # instead of a solid block, so the drawers read as sliding into the shell.
    shell.visual(Box((0.48, 0.68, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.1175)), material=safety_yellow, name="bottom_pan")
    shell.visual(Box((0.035, 0.68, 0.60)), origin=Origin(xyz=(-0.2225, 0.0, 0.40)), material=safety_yellow, name="rear_wall")
    shell.visual(Box((0.48, 0.035, 0.60)), origin=Origin(xyz=(0.0, 0.3225, 0.40)), material=safety_yellow, name="side_wall_0")
    shell.visual(Box((0.48, 0.035, 0.60)), origin=Origin(xyz=(0.0, -0.3225, 0.40)), material=safety_yellow, name="side_wall_1")
    shell.visual(Box((0.030, 0.050, 0.60)), origin=Origin(xyz=(0.245, 0.315, 0.40)), material=safety_yellow, name="front_stile_0")
    shell.visual(Box((0.030, 0.050, 0.60)), origin=Origin(xyz=(0.245, -0.315, 0.40)), material=safety_yellow, name="front_stile_1")
    shell.visual(Box((0.030, 0.68, 0.045)), origin=Origin(xyz=(0.245, 0.0, 0.6775)), material=safety_yellow, name="front_top_rail")
    shell.visual(Box((0.030, 0.68, 0.035)), origin=Origin(xyz=(0.245, 0.0, 0.510)), material=safety_yellow, name="drawer_divider")
    shell.visual(Box((0.030, 0.68, 0.035)), origin=Origin(xyz=(0.245, 0.0, 0.285)), material=safety_yellow, name="drawer_sill")
    shell.visual(Box((0.030, 0.68, 0.150)), origin=Origin(xyz=(0.245, 0.0, 0.185)), material=safety_yellow, name="front_kick_panel")
    shell.visual(Box((0.030, 0.66, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.685)), material=safety_yellow, name="top_rim")

    # Straight guide runners inside the front openings.
    for idx, y in enumerate((0.300, -0.300)):
        shell.visual(Box((0.31, 0.018, 0.018)), origin=Origin(xyz=(0.075, y, 0.538)), material=bare_metal, name=f"accessory_runner_{idx}")
        shell.visual(Box((0.34, 0.018, 0.018)), origin=Origin(xyz=(0.060, y, 0.325)), material=bare_metal, name=f"drawer_runner_{idx}")

    # Rear C-channel guides for the telescoping handle.
    for idx, y in enumerate((0.250, -0.250)):
        shell.visual(Box((0.059, 0.058, 0.54)), origin=Origin(xyz=(-0.2655, y, 0.390)), material=graphite, name=f"rear_guide_{idx}_back")
        shell.visual(Box((0.034, 0.012, 0.54)), origin=Origin(xyz=(-0.310, y + 0.035, 0.390)), material=graphite, name=f"rear_guide_{idx}_lip_0")
        shell.visual(Box((0.034, 0.012, 0.54)), origin=Origin(xyz=(-0.310, y - 0.035, 0.390)), material=graphite, name=f"rear_guide_{idx}_lip_1")

    # Hinge knuckles mounted to the rear rim.
    for idx, y in enumerate((-0.280, 0.0, 0.280)):
        shell.visual(Cylinder(radius=0.014, length=0.100), origin=Origin(xyz=(-0.245, y, 0.705), rpy=(pi / 2.0, 0.0, 0.0)), material=bare_metal, name=f"fixed_hinge_barrel_{idx}")
        shell.visual(Box((0.020, 0.100, 0.018)), origin=Origin(xyz=(-0.245, y, 0.691)), material=bare_metal, name=f"fixed_hinge_leaf_{idx}")

    # Wheel yokes and front feet are molded into the lower shell.
    for idx, y in enumerate((0.420, -0.420)):
        sign = 1.0 if y > 0.0 else -1.0
        shell.visual(Box((0.12, 0.014, 0.11)), origin=Origin(xyz=(-0.180, y, 0.075)), material=safety_yellow, name=f"wheel_fork_{idx}")
        shell.visual(Box((0.15, 0.090, 0.025)), origin=Origin(xyz=(-0.175, 0.382 * sign, 0.165)), material=safety_yellow, name=f"wheel_fender_{idx}")
        shell.visual(Box((0.10, 0.014, 0.050)), origin=Origin(xyz=(-0.180, y, 0.140)), material=safety_yellow, name=f"fork_web_{idx}")
        shell.visual(Box((0.12, 0.08, 0.052)), origin=Origin(xyz=(0.170, 0.220 * sign, 0.076)), material=dark_plastic, name=f"front_foot_{idx}")

    lid = model.part("lid")
    lid.visual(Box((0.520, 0.700, 0.045)), origin=Origin(xyz=(0.280, 0.0, 0.0175)), material=safety_yellow, name="lid_panel")
    lid.visual(Box((0.040, 0.690, 0.055)), origin=Origin(xyz=(0.550, 0.0, -0.002)), material=safety_yellow, name="front_lip")
    lid.visual(Box((0.500, 0.026, 0.055)), origin=Origin(xyz=(0.290, 0.355, -0.002)), material=safety_yellow, name="side_lip_0")
    lid.visual(Box((0.500, 0.026, 0.055)), origin=Origin(xyz=(0.290, -0.355, -0.002)), material=safety_yellow, name="side_lip_1")
    lid.visual(Box((0.34, 0.040, 0.018)), origin=Origin(xyz=(0.330, 0.0, 0.049)), material=dark_plastic, name="top_grip_pad")
    for idx, y in enumerate((-0.140, 0.140)):
        lid.visual(Cylinder(radius=0.014, length=0.120), origin=Origin(xyz=(-0.030, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=bare_metal, name=f"lid_hinge_barrel_{idx}")
        lid.visual(Box((0.070, 0.120, 0.014)), origin=Origin(xyz=(0.005, y, 0.004)), material=bare_metal, name=f"lid_hinge_leaf_{idx}")

    handle = model.part("rear_handle")
    for idx, y in enumerate((0.250, -0.250)):
        handle.visual(Box((0.022, 0.022, 0.660)), origin=Origin(xyz=(0.0, y, -0.070)), material=bare_metal, name=f"handle_rail_{idx}")
        handle.visual(Box((0.040, 0.050, 0.050)), origin=Origin(xyz=(0.0, y, -0.365)), material=graphite, name=f"slide_foot_{idx}")
    handle.visual(Cylinder(radius=0.020, length=0.580), origin=Origin(xyz=(0.0, 0.0, 0.265), rpy=(pi / 2.0, 0.0, 0.0)), material=dark_plastic, name="top_grip")
    handle.visual(Box((0.026, 0.060, 0.070)), origin=Origin(xyz=(0.0, 0.250, 0.235)), material=dark_plastic, name="grip_post_0")
    handle.visual(Box((0.026, 0.060, 0.070)), origin=Origin(xyz=(0.0, -0.250, 0.235)), material=dark_plastic, name="grip_post_1")

    accessory_drawer = model.part("accessory_drawer")
    accessory_drawer.visual(Box((0.026, 0.560, 0.100)), origin=Origin(xyz=(0.013, 0.0, 0.0)), material=dark_plastic, name="drawer_front")
    accessory_drawer.visual(Box((0.290, 0.500, 0.072)), origin=Origin(xyz=(-0.145, 0.0, -0.006)), material=graphite, name="drawer_box")
    accessory_drawer.visual(Box((0.280, 0.014, 0.020)), origin=Origin(xyz=(-0.140, 0.284, -0.043)), material=bare_metal, name="side_runner_0")
    accessory_drawer.visual(Box((0.280, 0.014, 0.020)), origin=Origin(xyz=(-0.140, -0.284, -0.043)), material=bare_metal, name="side_runner_1")
    accessory_drawer.visual(Box((0.015, 0.250, 0.025)), origin=Origin(xyz=(0.027, 0.0, 0.006)), material=black_rubber, name="small_pull")

    drawer = model.part("shallow_drawer")
    drawer.visual(Box((0.028, 0.575, 0.125)), origin=Origin(xyz=(0.014, 0.0, 0.0)), material=safety_yellow, name="drawer_front")
    drawer.visual(Box((0.320, 0.520, 0.095)), origin=Origin(xyz=(-0.160, 0.0, -0.006)), material=graphite, name="drawer_box")
    drawer.visual(Box((0.300, 0.014, 0.020)), origin=Origin(xyz=(-0.150, 0.284, -0.055)), material=bare_metal, name="side_runner_0")
    drawer.visual(Box((0.300, 0.014, 0.020)), origin=Origin(xyz=(-0.150, -0.284, -0.055)), material=bare_metal, name="side_runner_1")
    drawer.visual(Box((0.016, 0.300, 0.030)), origin=Origin(xyz=(0.030, 0.0, 0.010)), material=black_rubber, name="finger_pull")

    wheel_0 = model.part("wheel_0")
    wheel_0.visual(Cylinder(radius=0.070, length=0.055), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=black_rubber, name="tire")
    wheel_0.visual(Cylinder(radius=0.034, length=0.064), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=graphite, name="hub")
    wheel_1 = model.part("wheel_1")
    wheel_1.visual(Cylinder(radius=0.070, length=0.055), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=black_rubber, name="tire")
    wheel_1.visual(Cylinder(radius=0.034, length=0.064), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=graphite, name="hub")

    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(-0.245, 0.0, 0.705)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "shell_to_handle",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=handle,
        origin=Origin(xyz=(-0.315, 0.0, 0.640)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.300),
    )
    model.articulation(
        "shell_to_accessory_drawer",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=accessory_drawer,
        origin=Origin(xyz=(0.266, 0.0, 0.590)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.30, lower=0.0, upper=0.220),
    )
    model.articulation(
        "shell_to_drawer",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=drawer,
        origin=Origin(xyz=(0.266, 0.0, 0.385)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.30, lower=0.0, upper=0.260),
    )
    model.articulation(
        "shell_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=wheel_0,
        origin=Origin(xyz=(-0.180, 0.385, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    model.articulation(
        "shell_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=wheel_1,
        origin=Origin(xyz=(-0.180, -0.385, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("rear_handle")
    accessory_drawer = object_model.get_part("accessory_drawer")
    drawer = object_model.get_part("shallow_drawer")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    lid_hinge = object_model.get_articulation("shell_to_lid")
    handle_slide = object_model.get_articulation("shell_to_handle")
    accessory_slide = object_model.get_articulation("shell_to_accessory_drawer")
    drawer_slide = object_model.get_articulation("shell_to_drawer")
    wheel_spin_0 = object_model.get_articulation("shell_to_wheel_0")
    wheel_spin_1 = object_model.get_articulation("shell_to_wheel_1")

    ctx.expect_gap(
        accessory_drawer,
        shell,
        axis="x",
        min_gap=0.001,
        max_gap=0.012,
        positive_elem="drawer_front",
        negative_elem="front_top_rail",
        name="upper drawer front is separate from shell face",
    )
    ctx.expect_gap(
        drawer,
        shell,
        axis="x",
        min_gap=0.001,
        max_gap=0.012,
        positive_elem="drawer_front",
        negative_elem="drawer_sill",
        name="lower drawer front is separate from shell face",
    )
    ctx.expect_within(
        accessory_drawer,
        shell,
        axes="yz",
        margin=0.010,
        inner_elem="drawer_box",
        name="accessory drawer is aligned within lower shell",
    )
    ctx.expect_within(
        drawer,
        shell,
        axes="yz",
        margin=0.010,
        inner_elem="drawer_box",
        name="shallow drawer is aligned within lower shell",
    )
    ctx.expect_overlap(
        accessory_drawer,
        shell,
        axes="x",
        min_overlap=0.10,
        elem_a="drawer_box",
        name="accessory drawer retains insertion at rest",
    )
    ctx.expect_overlap(
        drawer,
        shell,
        axes="x",
        min_overlap=0.10,
        elem_a="drawer_box",
        name="shallow drawer retains insertion at rest",
    )
    ctx.expect_overlap(
        handle,
        shell,
        axes="z",
        min_overlap=0.30,
        elem_a="handle_rail_0",
        elem_b="rear_guide_0_back",
        name="handle rail is captured in guide at rest",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    rest_handle_pos = ctx.part_world_position(handle)
    rest_accessory_pos = ctx.part_world_position(accessory_drawer)
    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({lid_hinge: 1.0, handle_slide: 0.300, accessory_slide: 0.220, drawer_slide: 0.260}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        extended_handle_pos = ctx.part_world_position(handle)
        extended_accessory_pos = ctx.part_world_position(accessory_drawer)
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            handle,
            shell,
            axes="z",
            min_overlap=0.08,
            elem_a="handle_rail_0",
            elem_b="rear_guide_0_back",
            name="handle rail remains inserted when extended",
        )
        ctx.expect_overlap(
            accessory_drawer,
            shell,
            axes="x",
            min_overlap=0.04,
            elem_a="drawer_box",
            name="accessory drawer remains on runners when extended",
        )
        ctx.expect_overlap(
            drawer,
            shell,
            axes="x",
            min_overlap=0.04,
            elem_a="drawer_box",
            name="shallow drawer remains on runners when extended",
        )

    ctx.check(
        "lid opens upward on rear hinge",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.15,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "rear handle slides upward",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.25,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )
    ctx.check(
        "accessory drawer slides forward",
        rest_accessory_pos is not None
        and extended_accessory_pos is not None
        and extended_accessory_pos[0] > rest_accessory_pos[0] + 0.20,
        details=f"rest={rest_accessory_pos}, extended={extended_accessory_pos}",
    )
    ctx.check(
        "shallow drawer slides forward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.24,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )
    ctx.check(
        "transport wheels are continuous spin joints",
        wheel_spin_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin_1.articulation_type == ArticulationType.CONTINUOUS
        and wheel_0.name == "wheel_0"
        and wheel_1.name == "wheel_1",
    )

    return ctx.report()


object_model = build_object_model()
