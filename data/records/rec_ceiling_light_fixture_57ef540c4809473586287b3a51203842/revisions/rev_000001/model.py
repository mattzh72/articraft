from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    z_top: float,
    z_bottom: float,
    segments: int = 64,
) -> LatheGeometry:
    """Lathe a hollow collar whose axis is local +Z."""
    return LatheGeometry(
        [
            (inner_radius, z_top),
            (outer_radius, z_top),
            (outer_radius, z_bottom),
            (inner_radius, z_bottom),
        ],
        segments=segments,
    )


def _threaded_ring() -> LatheGeometry:
    """Axisymmetric retaining ring with two raised thread/knurl ridges."""
    return LatheGeometry(
        [
            (0.035, 0.000),
            (0.052, 0.000),
            (0.052, -0.004),
            (0.055, -0.006),
            (0.052, -0.008),
            (0.052, -0.014),
            (0.055, -0.016),
            (0.052, -0.018),
            (0.052, -0.026),
            (0.035, -0.026),
        ],
        segments=72,
    )


def _globe_shade() -> LatheGeometry:
    """Thin frosted glass globe/bell shade, open at the neck and bottom."""
    return LatheGeometry.from_shell_profiles(
        [
            (0.042, -0.024),
            (0.046, -0.046),
            (0.072, -0.072),
            (0.094, -0.112),
            (0.086, -0.154),
            (0.064, -0.188),
        ],
        [
            (0.034, -0.028),
            (0.038, -0.048),
            (0.064, -0.075),
            (0.086, -0.112),
            (0.078, -0.151),
            (0.057, -0.181),
        ],
        segments=80,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_fan_light_kit")

    brushed_nickel = model.material("brushed_nickel", rgba=(0.63, 0.60, 0.54, 1.0))
    dark_groove = model.material("dark_groove", rgba=(0.18, 0.17, 0.16, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.92, 0.90, 0.82, 0.42))
    warm_glass = model.material("warm_bulb_glass", rgba=(1.0, 0.82, 0.44, 0.68))

    fan_hub = model.part("fan_hub")
    fan_hub.visual(
        Cylinder(radius=0.13, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=brushed_nickel,
        name="motor_housing",
    )
    fan_hub.visual(
        Cylinder(radius=0.055, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=brushed_nickel,
        name="lower_collar",
    )
    fan_hub.visual(
        Cylinder(radius=0.035, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=brushed_nickel,
        name="downrod_stub",
    )
    for index in range(3):
        theta = index * math.tau / 3.0
        fan_hub.visual(
            Box((0.22, 0.055, 0.026)),
            origin=Origin(
                xyz=(0.160 * math.cos(theta), 0.160 * math.sin(theta), 0.115),
                rpy=(0.0, 0.0, theta),
            ),
            material=brushed_nickel,
            name=f"blade_root_{index}",
        )

    holder_plate = model.part("holder_plate")
    holder_plate.visual(
        Cylinder(radius=0.19, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_nickel,
        name="plate_disk",
    )
    holder_plate.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.176, tube=0.007, radial_segments=16, tubular_segments=80),
            "rolled_plate_rim",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=brushed_nickel,
        name="rolled_plate_rim",
    )
    holder_plate.visual(
        Cylinder(radius=0.040, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
        material=brushed_nickel,
        name="upper_spacer",
    )
    holder_plate.visual(
        Cylinder(radius=0.060, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=brushed_nickel,
        name="central_boss",
    )

    socket_radius = 0.145
    socket_bottom_z = -0.095
    socket_top_z = -0.015
    for index in range(3):
        theta = math.pi / 2.0 + index * math.tau / 3.0
        c = math.cos(theta)
        s = math.sin(theta)
        x = socket_radius * c
        y = socket_radius * s

        holder_plate.visual(
            Box((0.185, 0.024, 0.022)),
            origin=Origin(
                xyz=(0.074 * c, 0.074 * s, -0.030),
                rpy=(0.0, 0.0, theta),
            ),
            material=brushed_nickel,
            name=f"support_arm_{index}",
        )
        holder_plate.visual(
            Cylinder(radius=0.052, length=socket_top_z - socket_bottom_z),
            origin=Origin(xyz=(x, y, 0.5 * (socket_top_z + socket_bottom_z))),
            material=brushed_nickel,
            name=f"socket_{index}",
        )
        holder_plate.visual(
            Cylinder(radius=0.010, length=0.092),
            origin=Origin(xyz=(x, y, socket_bottom_z - 0.046)),
            material=dark_groove,
            name=f"bulb_stem_{index}",
        )
        holder_plate.visual(
            Sphere(radius=0.025),
            origin=Origin(xyz=(x, y, socket_bottom_z - 0.108)),
            material=warm_glass,
            name=f"bulb_{index}",
        )
        holder_plate.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(0.115 * c, 0.115 * s, 0.0148)),
            material=dark_groove,
            name=f"plate_screw_{index}",
        )

        shade = model.part(f"shade_{index}")
        shade.visual(
            mesh_from_geometry(_threaded_ring(), f"threaded_ring_{index}"),
            material=brushed_nickel,
            name="threaded_ring",
        )
        shade.visual(
            Cylinder(radius=0.004, length=0.027),
            origin=Origin(xyz=(0.064, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_groove,
            name="thumb_screw",
        )
        shade.visual(
            mesh_from_geometry(_globe_shade(), f"globe_shade_{index}"),
            material=frosted_glass,
            name="frosted_globe",
        )
        shade.visual(
            mesh_from_geometry(
                _ring_band(outer_radius=0.067, inner_radius=0.056, z_top=-0.181, z_bottom=-0.190),
                f"bottom_rim_{index}",
            ),
            material=frosted_glass,
            name="bottom_rim",
        )
        model.articulation(
            f"holder_to_shade_{index}",
            ArticulationType.REVOLUTE,
            parent=holder_plate,
            child=shade,
            origin=Origin(xyz=(x, y, socket_bottom_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=1.5,
                lower=-2.0 * math.pi,
                upper=2.0 * math.pi,
            ),
        )

    model.articulation(
        "hub_to_holder_plate",
        ArticulationType.FIXED,
        parent=fan_hub,
        child=holder_plate,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    holder = object_model.get_part("holder_plate")
    hub = object_model.get_part("fan_hub")

    ctx.expect_contact(
        holder,
        hub,
        elem_a="upper_spacer",
        elem_b="lower_collar",
        contact_tol=0.001,
        name="holder plate is fixed directly under the fan hub",
    )

    for index in range(3):
        shade = object_model.get_part(f"shade_{index}")
        joint = object_model.get_articulation(f"holder_to_shade_{index}")
        ctx.check(
            f"shade {index} uses a revolute threaded-ring joint",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"joint type is {joint.articulation_type}",
        )
        ctx.expect_contact(
            shade,
            holder,
            elem_a="threaded_ring",
            elem_b=f"socket_{index}",
            contact_tol=0.001,
            name=f"shade {index} retaining ring seats against its socket",
        )
        ctx.expect_within(
            shade,
            holder,
            axes="xy",
            inner_elem="threaded_ring",
            outer_elem=f"socket_{index}",
            margin=0.004,
            name=f"shade {index} threaded ring is centered on the socket neck",
        )

        rest_pos = ctx.part_world_position(shade)
        with ctx.pose({joint: math.pi / 2.0}):
            turned_pos = ctx.part_world_position(shade)
        ctx.check(
            f"shade {index} rotates in place about the vertical neck",
            rest_pos is not None
            and turned_pos is not None
            and abs(rest_pos[0] - turned_pos[0]) < 0.001
            and abs(rest_pos[1] - turned_pos[1]) < 0.001,
            details=f"rest={rest_pos}, turned={turned_pos}",
        )

    return ctx.report()


object_model = build_object_model()
