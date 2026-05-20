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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


LEFT_REAR_LEG = "left_rear_leg"
RIGHT_REAR_LEG = "right_rear_leg"
LEFT_AXLE_SHAFT = "left_axle_shaft"
RIGHT_AXLE_SHAFT = "right_axle_shaft"
LEFT_WHEEL_HUB = "left_wheel_hub"
RIGHT_WHEEL_HUB = "right_wheel_hub"


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _caster_wheel_mesh(name: str):
    outer_profile = [
        (0.014, -0.012),
        (0.026, -0.012),
        (0.036, -0.011),
        (0.043, -0.008),
        (0.048, -0.004),
        (0.050, 0.0),
        (0.048, 0.004),
        (0.043, 0.008),
        (0.036, 0.011),
        (0.026, 0.012),
        (0.014, 0.012),
    ]
    inner_profile = [
        (0.010, -0.008),
        (0.011, -0.005),
        (0.011, 0.005),
        (0.010, 0.008),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_caster_walker")

    anodized_aluminum = model.material("anodized_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    light_aluminum = model.material("light_aluminum", rgba=(0.84, 0.85, 0.86, 1.0))
    grip_foam = model.material("grip_foam", rgba=(0.22, 0.22, 0.24, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.29, 0.30, 0.32, 1.0))
    fork_paint = model.material("fork_paint", rgba=(0.63, 0.66, 0.71, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.65, 0.67, 0.70, 1.0))
    tip_rubber = model.material("tip_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    upper_body = model.part("upper_body")

    left_front_socket = (0.285, 0.215, 0.140)
    right_front_socket = (-0.285, 0.215, 0.140)
    left_front_top = (0.285, 0.190, 0.815)
    right_front_top = (-0.285, 0.190, 0.815)
    left_rear_handle = (0.285, -0.185, 0.840)
    right_rear_handle = (-0.285, -0.185, 0.840)
    left_rear_socket = (0.285, -0.185, 0.793)
    right_rear_socket = (-0.285, -0.185, 0.793)
    left_side_brace_front = (0.285, 0.201, 0.505)
    right_side_brace_front = (-0.285, 0.201, 0.505)
    left_side_brace_rear = (0.285, -0.145, 0.746)
    right_side_brace_rear = (-0.285, -0.145, 0.746)

    tube_radius = 0.012

    for a, b, name in [
        (left_front_socket, left_front_top, "left_front_upright"),
        (right_front_socket, right_front_top, "right_front_upright"),
        (left_front_top, left_rear_handle, "left_handle_bar"),
        (right_front_top, right_rear_handle, "right_handle_bar"),
        (left_side_brace_front, left_side_brace_rear, "left_side_brace"),
        (right_side_brace_front, right_side_brace_rear, "right_side_brace"),
        ((-0.285, 0.194, 0.710), (0.285, 0.194, 0.710), "front_crossbar"),
        ((-0.285, -0.185, 0.785), (0.285, -0.185, 0.785), "rear_crossbar"),
        ((-0.285, 0.210, 0.270), (0.285, 0.210, 0.270), "lower_front_stretcher"),
    ]:
        _add_member(upper_body, a, b, radius=tube_radius, material=anodized_aluminum, name=name)

    upper_body.visual(
        Box((0.056, 0.050, 0.016)),
        origin=Origin(xyz=(left_front_socket[0], left_front_socket[1], 0.133)),
        material=light_aluminum,
        name="left_front_socket_block",
    )
    upper_body.visual(
        Box((0.056, 0.050, 0.016)),
        origin=Origin(xyz=(right_front_socket[0], right_front_socket[1], 0.133)),
        material=light_aluminum,
        name="right_front_socket_block",
    )
    upper_body.visual(
        Box((0.060, 0.080, 0.094)),
        origin=Origin(xyz=left_rear_socket),
        material=light_aluminum,
        name="left_rear_receiver",
    )
    upper_body.visual(
        Box((0.060, 0.080, 0.094)),
        origin=Origin(xyz=right_rear_socket),
        material=light_aluminum,
        name="right_rear_receiver",
    )
    for x_pos, side_name in ((0.285, "left"), (-0.285, "right")):
        for x_offset, ear_name in ((0.020, "outer"), (-0.020, "inner")):
            upper_body.visual(
                Box((0.012, 0.022, 0.036)),
                origin=Origin(xyz=(x_pos + x_offset, 0.215, 0.107)),
                material=light_aluminum,
                name=f"{side_name}_front_socket_{ear_name}",
            )

    upper_body.visual(
        Cylinder(radius=0.020, length=0.120),
        origin=Origin(xyz=(0.285, -0.010, 0.835), rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
        material=grip_foam,
        name="left_hand_grip",
    )
    upper_body.visual(
        Cylinder(radius=0.020, length=0.120),
        origin=Origin(xyz=(-0.285, -0.010, 0.835), rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
        material=grip_foam,
        name="right_hand_grip",
    )

    for x_sign, name in [(1.0, "left"), (-1.0, "right")]:
        upper_body.visual(
            Cylinder(radius=0.006, length=0.050),
            origin=Origin(
                xyz=(0.285 * x_sign, 0.235, 0.133),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_hardware,
            name=f"{name}_caster_socket_bolt",
        )
        upper_body.visual(
            Cylinder(radius=0.006, length=0.048),
            origin=Origin(
                xyz=(0.285 * x_sign, -0.145, 0.785),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_hardware,
            name=f"{name}_rear_receiver_bolt",
        )

    upper_body.inertial = Inertial.from_geometry(
        Box((0.68, 0.52, 0.90)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.015, 0.450)),
    )

    left_rear_support = model.part("left_rear_support")
    _add_member(
        left_rear_support,
        (0.0, 0.0, 0.010),
        (0.0, -0.025, -0.748),
        radius=0.0115,
        material=anodized_aluminum,
        name=LEFT_REAR_LEG,
    )
    _add_member(
        left_rear_support,
        (0.0, -0.001, -0.016),
        (0.0, -0.003, -0.074),
        radius=0.018,
        material=light_aluminum,
        name="left_rear_collar",
    )
    left_rear_support.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.0, -0.025, -0.773)),
        material=tip_rubber,
        name="left_rear_tip",
    )
    left_rear_support.visual(
        Box((0.034, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, -0.004, 0.0086)),
        material=light_aluminum,
        name="left_rear_mount_flange",
    )
    left_rear_support.inertial = Inertial.from_geometry(
        Box((0.06, 0.08, 0.82)),
        mass=0.65,
        origin=Origin(xyz=(0.0, -0.012, -0.390)),
    )

    right_rear_support = model.part("right_rear_support")
    _add_member(
        right_rear_support,
        (0.0, 0.0, 0.010),
        (0.0, -0.025, -0.748),
        radius=0.0115,
        material=anodized_aluminum,
        name=RIGHT_REAR_LEG,
    )
    _add_member(
        right_rear_support,
        (0.0, -0.001, -0.016),
        (0.0, -0.003, -0.074),
        radius=0.018,
        material=light_aluminum,
        name="right_rear_collar",
    )
    right_rear_support.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.0, -0.025, -0.773)),
        material=tip_rubber,
        name="right_rear_tip",
    )
    right_rear_support.visual(
        Box((0.034, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, -0.004, 0.0086)),
        material=light_aluminum,
        name="right_rear_mount_flange",
    )
    right_rear_support.inertial = Inertial.from_geometry(
        Box((0.06, 0.08, 0.82)),
        mass=0.65,
        origin=Origin(xyz=(0.0, -0.012, -0.390)),
    )

    model.articulation(
        "left_rear_support_mount",
        ArticulationType.FIXED,
        parent=upper_body,
        child=left_rear_support,
        origin=Origin(xyz=(left_rear_socket[0], left_rear_socket[1], 0.7354)),
    )
    model.articulation(
        "right_rear_support_mount",
        ArticulationType.FIXED,
        parent=upper_body,
        child=right_rear_support,
        origin=Origin(xyz=(right_rear_socket[0], right_rear_socket[1], 0.7354)),
    )

    for side_name, x_pos in [("left", 0.285), ("right", -0.285)]:
        axle_shaft_name = LEFT_AXLE_SHAFT if side_name == "left" else RIGHT_AXLE_SHAFT
        wheel_hub_name = LEFT_WHEEL_HUB if side_name == "left" else RIGHT_WHEEL_HUB
        fork = model.part(f"{side_name}_caster_fork")
        fork.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=fork_paint,
            name=f"{side_name}_swivel_head",
        )
        fork.visual(
            Cylinder(radius=0.008, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -0.008)),
            material=fork_paint,
            name=f"{side_name}_swivel_stem",
        )
        fork.visual(
            Box((0.036, 0.022, 0.010)),
            origin=Origin(xyz=(0.0, -0.013, -0.020)),
            material=fork_paint,
            name=f"{side_name}_fork_crown",
        )
        fork.visual(
            Box((0.008, 0.010, 0.058)),
            origin=Origin(xyz=(0.018, -0.024, -0.049)),
            material=fork_paint,
            name=f"{side_name}_outer_blade",
        )
        fork.visual(
            Box((0.008, 0.010, 0.058)),
            origin=Origin(xyz=(-0.018, -0.024, -0.049)),
            material=fork_paint,
            name=f"{side_name}_inner_blade",
        )
        fork.visual(
            Cylinder(radius=0.005, length=0.044),
            origin=Origin(
                xyz=(0.0, -0.024, -0.077),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_hardware,
            name=axle_shaft_name,
        )
        fork.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(
                xyz=(0.026, -0.024, -0.077),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=wheel_core,
            name=f"{side_name}_axle_head_outer",
        )
        fork.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(
                xyz=(-0.026, -0.024, -0.077),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=wheel_core,
            name=f"{side_name}_axle_head_inner",
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.10, 0.08, 0.20)),
            mass=0.22,
            origin=Origin(xyz=(0.0, -0.020, -0.100)),
        )

        wheel = model.part(f"{side_name}_caster_wheel")
        wheel.visual(
            _caster_wheel_mesh(f"{side_name}_caster_wheel_shell"),
            material=wheel_rubber,
            name=f"{side_name}_wheel_shell",
        )
        wheel.visual(
            Cylinder(radius=0.015, length=0.028),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_core,
            name=wheel_hub_name,
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.050, length=0.024),
            mass=0.16,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

        model.articulation(
            f"{side_name}_caster_swivel",
            ArticulationType.REVOLUTE,
            parent=upper_body,
            child=fork,
            origin=Origin(xyz=(x_pos, 0.215, 0.089)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=4.0,
                lower=-1.57,
                upper=1.57,
            ),
        )
        model.articulation(
            f"{side_name}_caster_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, -0.024, -0.077)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=18.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper_body = object_model.get_part("upper_body")
    left_rear_support = object_model.get_part("left_rear_support")
    right_rear_support = object_model.get_part("right_rear_support")
    left_caster_fork = object_model.get_part("left_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_fork = object_model.get_part("right_caster_fork")
    right_caster_wheel = object_model.get_part("right_caster_wheel")

    ctx.allow_overlap(
        left_caster_fork,
        left_caster_wheel,
        elem_a=LEFT_AXLE_SHAFT,
        elem_b=LEFT_WHEEL_HUB,
        reason="The wheel hub is a simplified solid proxy for a bearing hub that rotates around the fork axle.",
    )
    ctx.allow_overlap(
        right_caster_fork,
        right_caster_wheel,
        elem_a=RIGHT_AXLE_SHAFT,
        elem_b=RIGHT_WHEEL_HUB,
        reason="The wheel hub is a simplified solid proxy for a bearing hub that rotates around the fork axle.",
    )

    ctx.expect_gap(
        upper_body,
        left_rear_support,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="left_rear_receiver",
        negative_elem=LEFT_REAR_LEG,
        name="left rear support reaches receiver underside",
    )
    ctx.expect_gap(
        upper_body,
        right_rear_support,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="right_rear_receiver",
        negative_elem=RIGHT_REAR_LEG,
        name="right rear support reaches receiver underside",
    )
    ctx.expect_overlap(
        left_rear_support,
        upper_body,
        axes="xy",
        min_overlap=0.02,
        elem_a=LEFT_REAR_LEG,
        elem_b="left_rear_receiver",
        name="left rear support stays under receiver clamp",
    )
    ctx.expect_overlap(
        right_rear_support,
        upper_body,
        axes="xy",
        min_overlap=0.02,
        elem_a=RIGHT_REAR_LEG,
        elem_b="right_rear_receiver",
        name="right rear support stays under receiver clamp",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
