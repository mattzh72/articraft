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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rr_section(width: float, length: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, length, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="media_center_remote")

    shell_dark = model.material("shell_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    shell_trim = model.material("shell_trim", rgba=(0.23, 0.24, 0.27, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.32, 0.34, 0.37, 1.0))
    door_finish = model.material("door_finish", rgba=(0.18, 0.19, 0.21, 1.0))

    body_width = 0.045
    body_length = 0.188
    body_thickness = 0.0138
    body_half_t = body_thickness * 0.5

    wheel_radius = 0.006
    wheel_length = 0.017
    wheel_center_y = 0.026
    wheel_center_z = body_half_t + wheel_radius + 0.0002

    body = model.part("body")
    body_shell_geom = section_loft(
        [
            _rr_section(0.0440, 0.1840, 0.0102, -body_half_t),
            _rr_section(0.0460, 0.1880, 0.0108, -0.0018),
            _rr_section(0.0452, 0.1870, 0.0106, 0.0026),
            _rr_section(0.0436, 0.1830, 0.0098, body_half_t),
        ]
    )
    body.visual(
        mesh_from_geometry(body_shell_geom, "remote_body_shell"),
        material=shell_dark,
        name="body_shell",
    )
    body.visual(
        Box((0.029, 0.020, 0.0016)),
        origin=Origin(xyz=(0.0, wheel_center_y, body_half_t + 0.0006)),
        material=shell_trim,
        name="wheel_saddle",
    )
    body.visual(
        Box((0.0030, 0.020, 0.0098)),
        origin=Origin(xyz=(0.0105, wheel_center_y, body_half_t + 0.0047)),
        material=shell_trim,
        name="wheel_right_cheek",
    )
    body.visual(
        Box((0.0030, 0.020, 0.0098)),
        origin=Origin(xyz=(-0.0105, wheel_center_y, body_half_t + 0.0047)),
        material=shell_trim,
        name="wheel_left_cheek",
    )
    body.visual(
        Box((0.035, 0.0032, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0335, -body_half_t - 0.0001)),
        material=shell_trim,
        name="door_upper_guide",
    )
    body.visual(
        Box((0.035, 0.0032, 0.0010)),
        origin=Origin(xyz=(0.0, -0.0575, -body_half_t - 0.0001)),
        material=shell_trim,
        name="door_lower_guide",
    )
    body.visual(
        Box((0.010, 0.024, 0.0009)),
        origin=Origin(xyz=(-0.014, -0.012, -body_half_t - 0.00005)),
        material=shell_trim,
        name="door_stop_pad",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.046, 0.190, 0.016)),
        mass=0.21,
        origin=Origin(),
    )

    scroll_wheel = model.part("scroll_wheel")
    scroll_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_length),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_rubber,
        name="scroll_wheel_roller",
    )
    scroll_wheel.visual(
        Cylinder(radius=0.0042, length=wheel_length + 0.0008),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_core,
        name="scroll_wheel_core",
    )
    scroll_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_length),
        mass=0.012,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.028, 0.086, 0.0065, corner_segments=6),
                0.0018,
                center=True,
            ),
            "remote_battery_door",
        ),
        material=door_finish,
        name="battery_door_panel",
    )
    battery_door.visual(
        Cylinder(radius=0.0014, length=0.016),
        origin=Origin(xyz=(0.0093, 0.0, -0.0012), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shell_trim,
        name="battery_door_grip",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.028, 0.086, 0.004)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, -0.0008)),
    )

    model.articulation(
        "body_to_scroll_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=scroll_wheel,
        origin=Origin(xyz=(0.0, wheel_center_y, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=18.0),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, -0.012, -body_half_t - 0.0009)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.007),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    scroll_wheel = object_model.get_part("scroll_wheel")
    battery_door = object_model.get_part("battery_door")
    wheel_joint = object_model.get_articulation("body_to_scroll_wheel")
    door_joint = object_model.get_articulation("body_to_battery_door")

    body_shell = body.get_visual("body_shell")
    wheel_roller = scroll_wheel.get_visual("scroll_wheel_roller")
    door_panel = battery_door.get_visual("battery_door_panel")

    ctx.check(
        "scroll wheel uses a continuous horizontal axis",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.check(
        "battery door slides along the short x axis",
        door_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(door_joint.axis) == (1.0, 0.0, 0.0)
        and door_joint.motion_limits is not None
        and door_joint.motion_limits.upper is not None
        and door_joint.motion_limits.upper >= 0.006,
        details=f"type={door_joint.articulation_type}, axis={door_joint.axis}, limits={door_joint.motion_limits}",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            body,
            battery_door,
            axis="z",
            positive_elem=body_shell,
            negative_elem=door_panel,
            max_gap=0.0003,
            max_penetration=0.0,
            name="battery door sits flush on the rear face",
        )
        ctx.expect_overlap(
            battery_door,
            body,
            axes="xy",
            elem_a=door_panel,
            elem_b=body_shell,
            min_overlap=0.02,
            name="battery door covers the battery compartment footprint",
        )
        ctx.expect_gap(
            scroll_wheel,
            body,
            axis="z",
            positive_elem=wheel_roller,
            negative_elem=body_shell,
            max_gap=0.001,
            max_penetration=0.0,
            name="scroll wheel stays proud of the front shell",
        )
        rest_door_position = ctx.part_world_position(battery_door)

    upper = 0.0
    if door_joint.motion_limits is not None and door_joint.motion_limits.upper is not None:
        upper = door_joint.motion_limits.upper

    with ctx.pose({door_joint: upper}):
        ctx.expect_gap(
            body,
            battery_door,
            axis="z",
            positive_elem=body_shell,
            negative_elem=door_panel,
            max_gap=0.0003,
            max_penetration=0.0,
            name="battery door stays flush while slid open",
        )
        ctx.expect_overlap(
            battery_door,
            body,
            axes="xy",
            elem_a=door_panel,
            elem_b=body_shell,
            min_overlap=0.02,
            name="battery door remains guided on the rear face",
        )
        open_door_position = ctx.part_world_position(battery_door)

    ctx.check(
        "battery door opens sideways across the back",
        rest_door_position is not None
        and open_door_position is not None
        and open_door_position[0] > rest_door_position[0] + 0.005,
        details=f"rest={rest_door_position}, open={open_door_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
