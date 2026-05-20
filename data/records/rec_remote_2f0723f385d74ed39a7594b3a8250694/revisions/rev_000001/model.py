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
    model = ArticulatedObject(name="motion_gaming_remote")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.96, 1.0))
    shell_grey = model.material("shell_grey", rgba=(0.79, 0.81, 0.84, 1.0))
    plastic_dark = model.material("plastic_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    button_grey = model.material("button_grey", rgba=(0.68, 0.71, 0.75, 1.0))
    sensor_black = model.material("sensor_black", rgba=(0.08, 0.09, 0.10, 1.0))
    bay_dark = model.material("bay_dark", rgba=(0.22, 0.24, 0.27, 1.0))

    body_width = 0.040
    body_depth = 0.018
    body_length = 0.156

    remote_body = model.part("remote_body")
    remote_body.visual(
        Box((0.036, 0.011, 0.148)),
        origin=Origin(xyz=(0.0, 0.0025, 0.0)),
        material=shell_white,
        name="main_shell_core",
    )
    remote_body.visual(
        Box((0.002, body_depth, 0.152)),
        origin=Origin(xyz=(-0.019, 0.0, 0.0)),
        material=shell_white,
        name="left_side_shell",
    )
    remote_body.visual(
        Box((0.002, body_depth, 0.152)),
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
        material=shell_white,
        name="right_side_shell",
    )
    remote_body.visual(
        Box((0.036, body_depth, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        material=shell_white,
        name="top_cap",
    )
    remote_body.visual(
        Box((0.036, body_depth, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.077)),
        material=shell_white,
        name="bottom_cap",
    )
    remote_body.visual(
        Box((0.036, 0.002, 0.073)),
        origin=Origin(xyz=(0.0, -0.008, 0.0405)),
        material=shell_white,
        name="back_cover_upper",
    )
    remote_body.visual(
        Box((0.006, 0.002, 0.064)),
        origin=Origin(xyz=(-0.017, -0.008, -0.034)),
        material=shell_white,
        name="battery_frame_left",
    )
    remote_body.visual(
        Box((0.006, 0.002, 0.064)),
        origin=Origin(xyz=(0.017, -0.008, -0.034)),
        material=shell_white,
        name="battery_frame_right",
    )
    remote_body.visual(
        Box((0.028, 0.002, 0.006)),
        origin=Origin(xyz=(0.0, -0.008, 0.001)),
        material=shell_white,
        name="battery_frame_header",
    )
    remote_body.visual(
        Box((0.028, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, -0.008, -0.072)),
        material=shell_white,
        name="battery_frame_bottom",
    )
    remote_body.visual(
        Box((0.024, 0.003, 0.054)),
        origin=Origin(xyz=(0.0, -0.0045, -0.034)),
        material=bay_dark,
        name="battery_bay_floor",
    )
    remote_body.visual(
        Box((0.003, 0.003, 0.052)),
        origin=Origin(xyz=(-0.0095, -0.0045, -0.034)),
        material=bay_dark,
        name="battery_bay_left_guide",
    )
    remote_body.visual(
        Box((0.003, 0.003, 0.052)),
        origin=Origin(xyz=(0.0095, -0.0045, -0.034)),
        material=bay_dark,
        name="battery_bay_right_guide",
    )
    remote_body.visual(
        Box((0.018, 0.001, 0.020)),
        origin=Origin(xyz=(0.0, 0.0085, 0.056)),
        material=sensor_black,
        name="sensor_window",
    )
    remote_body.visual(
        Cylinder(radius=0.0075, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0087, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_grey,
        name="primary_button",
    )
    remote_body.visual(
        Cylinder(radius=0.0040, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0086, -0.031), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_grey,
        name="home_button",
    )
    for index, x_pos in enumerate((-0.006, 0.0, 0.006), start=1):
        remote_body.visual(
            Box((0.002, 0.0012, 0.016)),
            origin=Origin(xyz=(x_pos, 0.0086, -0.054)),
            material=shell_grey,
            name=f"speaker_slot_{index}",
        )
    remote_body.visual(
        Cylinder(radius=0.0055, length=0.0036),
        origin=Origin(xyz=(0.0202, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_grey,
        name="joystick_socket",
    )
    remote_body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_length)),
        mass=0.30,
        origin=Origin(),
    )

    joystick_nub = model.part("joystick_nub")
    joystick_nub.visual(
        Cylinder(radius=0.0030, length=0.0024),
        origin=Origin(xyz=(0.0012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_dark,
        name="nub_base",
    )
    joystick_nub.visual(
        Cylinder(radius=0.0022, length=0.0054),
        origin=Origin(xyz=(0.0045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_dark,
        name="nub_stem",
    )
    joystick_nub.visual(
        Cylinder(radius=0.0042, length=0.0046),
        origin=Origin(xyz=(0.0087, 0.0018, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_dark,
        name="nub_cap",
    )
    joystick_nub.inertial = Inertial.from_geometry(
        Box((0.015, 0.011, 0.011)),
        mass=0.015,
        origin=Origin(xyz=(0.006, 0.001, 0.0)),
    )

    model.articulation(
        "body_to_joystick",
        ArticulationType.CONTINUOUS,
        parent=remote_body,
        child=joystick_nub,
        origin=Origin(xyz=(0.0220, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.0272, 0.0015, 0.0615)),
        origin=Origin(xyz=(0.0, 0.00135, -0.03075)),
        material=shell_grey,
        name="door_panel",
    )
    battery_door.visual(
        Box((0.0228, 0.0020, 0.0530)),
        origin=Origin(xyz=(0.0, 0.00305, -0.0325)),
        material=shell_grey,
        name="door_inner_lip",
    )
    battery_door.visual(
        Box((0.010, 0.0020, 0.0045)),
        origin=Origin(xyz=(0.0, 0.0035, -0.0585)),
        material=plastic_dark,
        name="door_latch_tab",
    )
    battery_door.visual(
        Cylinder(radius=0.0015, length=0.008),
        origin=Origin(xyz=(-0.0085, 0.0009, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_grey,
        name="door_hinge_barrel_left",
    )
    battery_door.visual(
        Cylinder(radius=0.0015, length=0.008),
        origin=Origin(xyz=(0.0085, 0.0009, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_grey,
        name="door_hinge_barrel_right",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.028, 0.006, 0.063)),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0025, -0.030)),
    )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=remote_body,
        child=battery_door,
        origin=Origin(xyz=(0.0, -0.0096, -0.0020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.5,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    remote_body = object_model.get_part("remote_body")
    joystick_nub = object_model.get_part("joystick_nub")
    battery_door = object_model.get_part("battery_door")
    joystick_joint = object_model.get_articulation("body_to_joystick")
    door_joint = object_model.get_articulation("body_to_battery_door")

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    ctx.check(
        "remote parts are present",
        remote_body is not None and joystick_nub is not None and battery_door is not None,
        details="Expected remote body, side joystick nub, and hinged battery door parts.",
    )

    joystick_limits = joystick_joint.motion_limits
    ctx.check(
        "joystick nub uses a continuous vertical spin joint",
        joystick_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in joystick_joint.axis) == (0.0, 0.0, 1.0)
        and joystick_limits is not None
        and joystick_limits.lower is None
        and joystick_limits.upper is None,
        details=(
            f"type={joystick_joint.articulation_type}, axis={joystick_joint.axis}, "
            f"limits={joystick_limits}"
        ),
    )

    door_limits = door_joint.motion_limits
    ctx.check(
        "battery door hinges from its top edge",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(value, 6) for value in door_joint.axis) == (-1.0, 0.0, 0.0)
        and door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and door_limits.upper >= 1.4,
        details=(
            f"type={door_joint.articulation_type}, axis={door_joint.axis}, "
            f"limits={door_limits}"
        ),
    )

    with ctx.pose({door_joint: 0.0}):
        body_aabb = ctx.part_world_aabb(remote_body)
        closed_panel_aabb = ctx.part_element_world_aabb(battery_door, elem="door_panel")
        ctx.check(
            "battery door sits flush on the lower back face when closed",
            body_aabb is not None
            and closed_panel_aabb is not None
            and abs(closed_panel_aabb[0][1] - body_aabb[0][1]) <= 0.0004
            and closed_panel_aabb[0][0] > body_aabb[0][0] + 0.004
            and closed_panel_aabb[1][0] < body_aabb[1][0] - 0.004,
            details=f"body_aabb={body_aabb}, closed_panel_aabb={closed_panel_aabb}",
        )

    with ctx.pose({door_joint: door_limits.upper}):
        opened_panel_aabb = ctx.part_element_world_aabb(battery_door, elem="door_panel")

    ctx.check(
        "battery door swings outward when opened",
        closed_panel_aabb is not None
        and opened_panel_aabb is not None
        and opened_panel_aabb[0][1] < closed_panel_aabb[0][1] - 0.010,
        details=(
            f"closed_panel_aabb={closed_panel_aabb}, "
            f"opened_panel_aabb={opened_panel_aabb}"
        ),
    )

    with ctx.pose({joystick_joint: 0.0}):
        joystick_cap_aabb_rest = ctx.part_element_world_aabb(joystick_nub, elem="nub_cap")
    with ctx.pose({joystick_joint: 1.2}):
        joystick_cap_aabb_rotated = ctx.part_element_world_aabb(joystick_nub, elem="nub_cap")

    rest_center = aabb_center(joystick_cap_aabb_rest)
    rotated_center = aabb_center(joystick_cap_aabb_rotated)
    ctx.check(
        "side joystick nub rotates about the vertical axis",
        rest_center is not None
        and rotated_center is not None
        and abs(rotated_center[2] - rest_center[2]) <= 0.001
        and (
            abs(rotated_center[0] - rest_center[0]) >= 0.0015
            or abs(rotated_center[1] - rest_center[1]) >= 0.0015
        ),
        details=(
            f"rest_center={rest_center}, rotated_center={rotated_center}, "
            f"rest_aabb={joystick_cap_aabb_rest}, rotated_aabb={joystick_cap_aabb_rotated}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
