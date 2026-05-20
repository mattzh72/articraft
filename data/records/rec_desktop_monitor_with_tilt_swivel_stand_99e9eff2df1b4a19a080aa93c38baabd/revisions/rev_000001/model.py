from __future__ import annotations

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


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_monitor")

    shell_black = model.material("shell_black", rgba=(0.10, 0.11, 0.12, 1.0))
    stand_black = model.material("stand_black", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.44, 0.46, 0.48, 1.0))
    screen_dark = model.material("screen_dark", rgba=(0.04, 0.05, 0.06, 1.0))
    power_blue = model.material("power_blue", rgba=(0.19, 0.50, 0.95, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.260, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=stand_black,
        name="pedestal",
    )
    base.visual(
        Box((0.215, 0.145, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin_gray,
        name="pedestal_cap",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=stand_black,
        name="swivel_boss",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_gray,
        name="swivel_hub",
    )
    neck.visual(
        Box((0.040, 0.012, 0.210)),
        origin=Origin(xyz=(0.0, 0.010, 0.115)),
        material=stand_black,
        name="front_web",
    )
    neck.visual(
        Box((0.006, 0.022, 0.190)),
        origin=Origin(xyz=(-0.017, 0.011, 0.115)),
        material=stand_black,
        name="side_rail_0",
    )
    neck.visual(
        Box((0.006, 0.022, 0.190)),
        origin=Origin(xyz=(0.017, 0.011, 0.115)),
        material=stand_black,
        name="side_rail_1",
    )
    neck.visual(
        Box((0.040, 0.022, 0.014)),
        origin=Origin(xyz=(0.0, 0.011, 0.017)),
        material=stand_black,
        name="lower_bridge",
    )
    neck.visual(
        Box((0.116, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, 0.011, 0.224)),
        material=stand_black,
        name="upper_bridge",
    )
    neck.visual(
        Box((0.016, 0.010, 0.044)),
        origin=Origin(xyz=(-0.050, 0.004, 0.250)),
        material=stand_black,
        name="yoke_arm_0",
    )
    neck.visual(
        Box((0.016, 0.010, 0.044)),
        origin=Origin(xyz=(0.050, 0.004, 0.250)),
        material=stand_black,
        name="yoke_arm_1",
    )
    neck.visual(
        Cylinder(radius=0.0025, length=0.040),
        origin=Origin(xyz=(0.014, 0.024, 0.065)),
        material=satin_gray,
        name="door_knuckle_0",
    )
    neck.visual(
        Cylinder(radius=0.0025, length=0.040),
        origin=Origin(xyz=(0.014, 0.024, 0.185)),
        material=satin_gray,
        name="door_knuckle_1",
    )

    monitor = model.part("monitor")
    monitor.visual(
        Cylinder(radius=0.010, length=0.082),
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
        material=satin_gray,
        name="hinge_barrel",
    )
    monitor.visual(
        Box((0.080, 0.014, 0.060)),
        origin=Origin(xyz=(0.0, 0.005, 0.010)),
        material=satin_gray,
        name="hinge_bracket",
    )
    monitor.visual(
        Box((0.544, 0.036, 0.338)),
        origin=Origin(xyz=(0.0, -0.019, 0.095)),
        material=shell_black,
        name="rear_shell",
    )
    monitor.visual(
        Box((0.538, 0.006, 0.332)),
        origin=Origin(xyz=(0.0, -0.035, 0.098)),
        material=shell_black,
        name="front_bezel",
    )
    monitor.visual(
        Box((0.516, 0.002, 0.290)),
        origin=Origin(xyz=(0.0, -0.039, 0.101)),
        material=screen_dark,
        name="screen_panel",
    )
    monitor.visual(
        Box((0.180, 0.010, 0.024)),
        origin=Origin(xyz=(0.135, -0.031, -0.057)),
        material=satin_gray,
        name="control_shelf",
    )

    cable_door = model.part("cable_door")
    cable_door.visual(
        Box((0.028, 0.003, 0.170)),
        origin=Origin(xyz=(-0.014, 0.0015, 0.0)),
        material=shell_black,
        name="door_panel",
    )
    cable_door.visual(
        Cylinder(radius=0.0025, length=0.070),
        origin=Origin(xyz=(0.0, 0.0025, 0.0)),
        material=satin_gray,
        name="door_knuckle",
    )

    button_x_positions = (0.104, 0.126, 0.148, 0.170)
    for index, button_x in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.012, 0.004, 0.006)),
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
            material=satin_gray,
            name="cap",
        )
        model.articulation(
            f"monitor_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=monitor,
            child=button,
            origin=Origin(xyz=(button_x, -0.0420, -0.057)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.05, lower=0.0, upper=0.0008),
        )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=power_blue,
        name="cap",
    )
    model.articulation(
        "monitor_to_power_button",
        ArticulationType.PRISMATIC,
        parent=monitor,
        child=power_button,
        origin=Origin(xyz=(0.198, -0.0420, -0.057)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.05, lower=0.0, upper=0.0008),
    )

    model.articulation(
        "base_to_neck",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
    )
    model.articulation(
        "neck_to_monitor",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=monitor,
        origin=Origin(xyz=(0.0, 0.0, 0.252)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.30, upper=0.45),
    )
    model.articulation(
        "neck_to_cable_door",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=cable_door,
        origin=Origin(xyz=(0.014, 0.022, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    neck = object_model.get_part("neck")
    monitor = object_model.get_part("monitor")
    cable_door = object_model.get_part("cable_door")
    button_0 = object_model.get_part("button_0")
    power_button = object_model.get_part("power_button")

    swivel = object_model.get_articulation("base_to_neck")
    tilt = object_model.get_articulation("neck_to_monitor")
    door_hinge = object_model.get_articulation("neck_to_cable_door")
    button_0_slide = object_model.get_articulation("monitor_to_button_0")
    power_button_slide = object_model.get_articulation("monitor_to_power_button")

    ctx.expect_gap(
        monitor,
        base,
        axis="z",
        min_gap=0.14,
        name="display housing sits well above the pedestal",
    )
    ctx.expect_overlap(
        monitor,
        base,
        axes="xy",
        min_overlap=0.05,
        name="display stays centered over the pedestal footprint",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            cable_door,
            neck,
            axis="y",
            positive_elem="door_panel",
            negative_elem="upper_bridge",
            max_gap=0.003,
            max_penetration=0.0,
            name="cable door panel closes flush with the stand frame",
        )
        ctx.expect_overlap(
            cable_door,
            neck,
            axes="xz",
            min_overlap=0.025,
            name="cable door covers the rear cable channel opening",
        )

    ctx.expect_overlap(
        button_0,
        monitor,
        axes="xz",
        elem_a="cap",
        elem_b="control_shelf",
        min_overlap=0.004,
        name="front menu buttons sit on the lower control shelf",
    )
    ctx.expect_overlap(
        power_button,
        monitor,
        axes="xz",
        elem_a="cap",
        elem_b="control_shelf",
        min_overlap=0.004,
        name="power button sits on the lower control shelf",
    )

    door_closed_aabb = None
    door_open_aabb = None
    with ctx.pose({door_hinge: 0.0}):
        door_closed_aabb = ctx.part_world_aabb(cable_door)
    with ctx.pose({door_hinge: 1.20}):
        door_open_aabb = ctx.part_world_aabb(cable_door)

    door_closed_min_x = door_closed_aabb[0][0] if door_closed_aabb is not None else None
    door_open_min_x = door_open_aabb[0][0] if door_open_aabb is not None else None
    ctx.check(
        "cable door swings clear of the rear opening",
        door_closed_min_x is not None
        and door_open_min_x is not None
        and door_open_min_x > door_closed_min_x + 0.010,
        details=f"closed_min_x={door_closed_min_x}, open_min_x={door_open_min_x}",
    )

    tilt_lower_center = None
    tilt_upper_center = None
    limits = tilt.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({tilt: limits.lower}):
            tilt_lower_center = _aabb_center(ctx.part_element_world_aabb(monitor, elem="screen_panel"))
        with ctx.pose({tilt: limits.upper}):
            tilt_upper_center = _aabb_center(ctx.part_element_world_aabb(monitor, elem="screen_panel"))

    ctx.check(
        "display tilts upward and backward at the upper stop",
        tilt_lower_center is not None
        and tilt_upper_center is not None
        and tilt_upper_center[1] > tilt_lower_center[1] + 0.040
        and tilt_upper_center[2] > tilt_lower_center[2] + 0.020,
        details=f"lower_center={tilt_lower_center}, upper_center={tilt_upper_center}",
    )

    swivel_rest_center = None
    swivel_turned_center = None
    with ctx.pose({swivel: 0.0}):
        swivel_rest_center = _aabb_center(ctx.part_element_world_aabb(monitor, elem="screen_panel"))
    with ctx.pose({swivel: 0.70}):
        swivel_turned_center = _aabb_center(ctx.part_element_world_aabb(monitor, elem="screen_panel"))

    ctx.check(
        "stand swivel rotates the display around the vertical axis",
        swivel_rest_center is not None
        and swivel_turned_center is not None
        and abs(swivel_turned_center[0] - swivel_rest_center[0]) > 0.010,
        details=f"rest_center={swivel_rest_center}, turned_center={swivel_turned_center}",
    )

    button_rest_y = None
    button_pressed_y = None
    with ctx.pose({button_0_slide: 0.0}):
        button_rest = ctx.part_world_position(button_0)
        button_rest_y = button_rest[1] if button_rest is not None else None
    with ctx.pose({button_0_slide: 0.0008}):
        button_pressed = ctx.part_world_position(button_0)
        button_pressed_y = button_pressed[1] if button_pressed is not None else None

    ctx.check(
        "front menu button depresses inward",
        button_rest_y is not None
        and button_pressed_y is not None
        and button_pressed_y > button_rest_y + 0.0005,
        details=f"rest_y={button_rest_y}, pressed_y={button_pressed_y}",
    )

    power_rest_y = None
    power_pressed_y = None
    with ctx.pose({power_button_slide: 0.0}):
        power_rest = ctx.part_world_position(power_button)
        power_rest_y = power_rest[1] if power_rest is not None else None
    with ctx.pose({power_button_slide: 0.0008}):
        power_pressed = ctx.part_world_position(power_button)
        power_pressed_y = power_pressed[1] if power_pressed is not None else None

    ctx.check(
        "power button depresses inward",
        power_rest_y is not None
        and power_pressed_y is not None
        and power_pressed_y > power_rest_y + 0.0005,
        details=f"rest_y={power_rest_y}, pressed_y={power_pressed_y}",
    )

    return ctx.report()


object_model = build_object_model()
