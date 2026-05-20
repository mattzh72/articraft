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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trackball_desktop_controller")

    shell_dark = model.material("shell_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    shell_trim = model.material("shell_trim", rgba=(0.11, 0.12, 0.13, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.44, 0.46, 0.49, 1.0))
    trackball_red = model.material("trackball_red", rgba=(0.68, 0.10, 0.12, 1.0))
    trackball_highlight = model.material("trackball_highlight", rgba=(0.92, 0.78, 0.78, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((0.228, 0.158, 0.052)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    housing.visual(
        Box((0.228, 0.158, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=shell_dark,
        name="bottom_plate",
    )
    housing.visual(
        Box((0.228, 0.004, 0.039)),
        origin=Origin(xyz=(0.0, -0.077, 0.0235)),
        material=shell_dark,
        name="front_wall",
    )
    housing.visual(
        Box((0.228, 0.004, 0.039)),
        origin=Origin(xyz=(0.0, 0.077, 0.0235)),
        material=shell_dark,
        name="rear_wall",
    )
    housing.visual(
        Box((0.004, 0.150, 0.039)),
        origin=Origin(xyz=(-0.112, 0.0, 0.0235)),
        material=shell_dark,
        name="left_wall",
    )
    housing.visual(
        Box((0.004, 0.150, 0.039)),
        origin=Origin(xyz=(0.112, 0.0, 0.0235)),
        material=shell_dark,
        name="right_wall",
    )
    housing.visual(
        Box((0.228, 0.035, 0.004)),
        origin=Origin(xyz=(0.0, -0.0575, 0.045)),
        material=shell_dark,
        name="top_front_strip",
    )
    housing.visual(
        Box((0.228, 0.035, 0.004)),
        origin=Origin(xyz=(0.0, 0.0575, 0.045)),
        material=shell_dark,
        name="top_rear_strip",
    )
    housing.visual(
        Box((0.072, 0.080, 0.004)),
        origin=Origin(xyz=(-0.074, 0.0, 0.045)),
        material=shell_dark,
        name="top_left_strip",
    )
    housing.visual(
        Box((0.072, 0.080, 0.004)),
        origin=Origin(xyz=(0.074, 0.0, 0.045)),
        material=shell_dark,
        name="top_right_strip",
    )

    socket_bezel_geom = ExtrudeWithHolesGeometry(
        _circle_profile(0.041, segments=56),
        [_circle_profile(0.032, segments=56)],
        height=0.004,
        center=True,
    )
    housing.visual(
        mesh_from_geometry(socket_bezel_geom, "trackball_socket_bezel"),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=shell_trim,
        name="socket_bezel",
    )

    housing.visual(
        Box((0.006, 0.034, 0.020)),
        origin=Origin(xyz=(0.113, 0.0, 0.024)),
        material=shell_trim,
        name="wheel_mount_block",
    )
    housing.visual(
        Box((0.012, 0.005, 0.018)),
        origin=Origin(xyz=(0.117, -0.0125, 0.024)),
        material=shell_trim,
        name="wheel_front_ear",
    )
    housing.visual(
        Box((0.012, 0.005, 0.018)),
        origin=Origin(xyz=(0.117, 0.0125, 0.024)),
        material=shell_trim,
        name="wheel_rear_ear",
    )

    ball_radius = 0.032
    ball_center_z = 0.0365
    bearing_radius = 0.003
    bearing_ring_radius = 0.025
    bearing_center_z = ball_center_z - math.sqrt(
        (ball_radius + bearing_radius) ** 2 - bearing_ring_radius**2
    )
    bearing_post_length = bearing_center_z - bearing_radius - 0.004
    bearing_post_center_z = 0.004 + bearing_post_length * 0.5
    for bearing_index, angle in enumerate((math.pi / 2.0, 7.0 * math.pi / 6.0, 11.0 * math.pi / 6.0)):
        bearing_x = bearing_ring_radius * math.cos(angle)
        bearing_y = bearing_ring_radius * math.sin(angle)
        housing.visual(
            Cylinder(radius=0.0018, length=bearing_post_length),
            origin=Origin(xyz=(bearing_x, bearing_y, bearing_post_center_z)),
            material=wheel_core,
            name=f"support_post_{bearing_index}",
        )
        housing.visual(
            Sphere(radius=bearing_radius),
            origin=Origin(xyz=(bearing_x, bearing_y, bearing_center_z)),
            material=wheel_core,
            name=f"support_bearing_{bearing_index}",
        )

    for foot_index, x in enumerate((-0.075, 0.075)):
        for rear_index, y in enumerate((-0.050, 0.050)):
            housing.visual(
                Box((0.024, 0.020, 0.004)),
                origin=Origin(xyz=(x, y, -0.002)),
                material=foot_rubber,
                name=f"foot_{foot_index}_{rear_index}",
            )

    trackball = model.part("trackball")
    trackball.visual(
        Sphere(radius=0.032),
        material=trackball_red,
        name="ball_shell",
    )
    trackball.visual(
        Sphere(radius=0.0045),
        origin=Origin(xyz=(0.0, 0.0255, 0.0175)),
        material=trackball_highlight,
        name="ball_marker",
    )
    trackball.inertial = Inertial.from_geometry(
        Sphere(radius=0.032),
        mass=0.18,
    )

    scroll_wheel = model.part("scroll_wheel")
    scroll_wheel.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="wheel_tire",
    )
    scroll_wheel.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_core,
        name="wheel_core",
    )
    scroll_wheel.visual(
        Box((0.004, 0.003, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=wheel_core,
        name="wheel_marker",
    )
    scroll_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.016),
        mass=0.03,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_trackball",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=trackball,
        origin=Origin(xyz=(0.0, 0.0, ball_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=12.0),
    )
    model.articulation(
        "housing_to_scroll_wheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=scroll_wheel,
        origin=Origin(xyz=(0.129, 0.0, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    trackball = object_model.get_part("trackball")
    scroll_wheel = object_model.get_part("scroll_wheel")
    trackball_joint = object_model.get_articulation("housing_to_trackball")
    wheel_joint = object_model.get_articulation("housing_to_scroll_wheel")

    for bearing_name in ("support_bearing_0", "support_bearing_1", "support_bearing_2"):
        ctx.allow_overlap(
            housing,
            trackball,
            elem_a=bearing_name,
            elem_b="ball_shell",
            reason="The support balls intentionally stand in for the trackball's bearing contact points inside the socket.",
        )

    ctx.expect_origin_gap(
        trackball,
        housing,
        axis="z",
        min_gap=0.033,
        max_gap=0.037,
        name="trackball center sits above the housing body",
    )
    ctx.expect_origin_gap(
        scroll_wheel,
        housing,
        axis="x",
        min_gap=0.124,
        max_gap=0.133,
        name="scroll wheel is mounted on the right side face",
    )
    ctx.expect_origin_gap(
        scroll_wheel,
        housing,
        axis="z",
        min_gap=0.020,
        max_gap=0.028,
        name="scroll wheel rides at thumb height",
    )
    ctx.expect_overlap(
        trackball,
        housing,
        axes="xy",
        min_overlap=0.060,
        elem_a="ball_shell",
        name="trackball sits centered over the top socket area",
    )
    ctx.expect_contact(
        trackball,
        housing,
        elem_a="ball_shell",
        elem_b="support_bearing_0",
        contact_tol=1e-6,
        name="trackball rests on an internal support bearing",
    )
    ctx.check(
        "trackball joint uses a horizontal spin axis",
        tuple(trackball_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={trackball_joint.axis}",
    )
    ctx.check(
        "scroll wheel joint uses a horizontal spin axis",
        tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={wheel_joint.axis}",
    )

    rest_trackball_pos = ctx.part_world_position(trackball)
    rest_wheel_pos = ctx.part_world_position(scroll_wheel)
    with ctx.pose({trackball_joint: 1.1, wheel_joint: 1.7}):
        spun_trackball_pos = ctx.part_world_position(trackball)
        spun_wheel_pos = ctx.part_world_position(scroll_wheel)

    ctx.check(
        "continuous controls rotate in place",
        rest_trackball_pos is not None
        and rest_wheel_pos is not None
        and spun_trackball_pos is not None
        and spun_wheel_pos is not None
        and max(abs(a - b) for a, b in zip(rest_trackball_pos, spun_trackball_pos)) < 1e-6
        and max(abs(a - b) for a, b in zip(rest_wheel_pos, spun_wheel_pos)) < 1e-6,
        details=(
            f"rest_trackball={rest_trackball_pos}, spun_trackball={spun_trackball_pos}, "
            f"rest_wheel={rest_wheel_pos}, spun_wheel={spun_wheel_pos}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
