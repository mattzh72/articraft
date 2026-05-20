from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_disc(outer_radius: float, inner_radius: float, width: float) -> cq.Workplane:
    """Annular disc in the XZ plane, centered on the Y axis."""
    outer = cq.Workplane("XZ").circle(outer_radius).extrude(width / 2.0, both=True)
    bore = cq.Workplane("XZ").circle(inner_radius).extrude(width, both=True)
    return outer.cut(bore)


def _add_box(part, name: str, size, xyz, material: Material, rpy=(0.0, 0.0, 0.0)):
    return part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_cylinder_y(part, name: str, radius: float, length: float, xyz, material: Material):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _radial_rpy(theta: float) -> tuple[float, float, float]:
    # Local +X is radial in the wheel XZ plane.
    return (0.0, -theta, 0.0)


def _tangent_rpy(theta: float) -> tuple[float, float, float]:
    # Local +X lies tangent to a circle in the wheel XZ plane.
    return (0.0, math.pi / 2.0 - theta, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_undershot_waterwheel")

    weathered_oak = model.material("weathered_oak", rgba=(0.55, 0.36, 0.18, 1.0))
    endgrain = model.material("dark_endgrain", rgba=(0.31, 0.20, 0.11, 1.0))
    wrought_iron = model.material("blackened_wrought_iron", rgba=(0.045, 0.047, 0.043, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.50, 0.54, 0.55, 1.0))
    concrete = model.material("aged_concrete", rgba=(0.45, 0.43, 0.38, 1.0))
    safety_blue = model.material("service_blue", rgba=(0.02, 0.16, 0.34, 1.0))
    brass = model.material("grease_brass", rgba=(0.85, 0.62, 0.20, 1.0))

    frame = model.part("frame")

    # Timber/concrete sill and water-channel structure tying both side frames into
    # one grounded assembly.
    _add_box(frame, "stone_sill", (2.10, 1.62, 0.10), (0.0, 0.0, 0.05), concrete)
    for i, (x, z) in enumerate(((-0.86, 0.13), (0.0, 0.04), (0.86, 0.13))):
        _add_box(frame, f"cross_tie_{i}", (0.18, 1.48, 0.12), (x, 0.0, z), weathered_oak)
    for i, y in enumerate((-0.62, 0.62)):
        _add_box(frame, f"side_sill_{i}", (1.95, 0.14, 0.16), (0.0, y, 0.22), weathered_oak)
        _add_box(frame, f"bearing_pedestal_{i}", (0.42, 0.22, 0.74), (0.0, y, 0.60), weathered_oak)
        _add_box(frame, f"pedestal_cap_{i}", (0.54, 0.26, 0.08), (0.0, y, 0.925), wrought_iron)

    # A-frame cheeks and lateral bracing with old-school heavy timber proportions.
    leg_pairs = [
        (-0.86, 0.24, -0.28, 1.38),
        (0.86, 0.24, 0.28, 1.38),
    ]
    for side_idx, y in enumerate((-0.62, 0.62)):
        for leg_idx, (x0, z0, x1, z1) in enumerate(leg_pairs):
            dx = x1 - x0
            dz = z1 - z0
            length = math.hypot(dx, dz)
            theta = math.atan2(dz, dx)
            _add_box(
                frame,
                f"side_leg_{side_idx}_{leg_idx}",
                (length, 0.12, 0.12),
                ((x0 + x1) / 2.0, y, (z0 + z1) / 2.0),
                weathered_oak,
                rpy=(0.0, -theta, 0.0),
            )
        _add_box(frame, f"side_post_{side_idx}_0", (0.13, 0.12, 0.88), (-0.86, y, 0.66), weathered_oak)
        _add_box(frame, f"side_post_{side_idx}_1", (0.13, 0.12, 0.88), (0.86, y, 0.66), weathered_oak)
        _add_box(frame, f"lower_gusset_{side_idx}_0", (0.28, 0.13, 0.07), (-0.68, y, 0.33), wrought_iron, rpy=(0.0, -0.52, 0.0))
        _add_box(frame, f"lower_gusset_{side_idx}_1", (0.28, 0.13, 0.07), (0.68, y, 0.33), wrought_iron, rpy=(0.0, 0.52, 0.0))

    # Cross members are either above the swept wheel envelope or set into the
    # outer side planes, so the rotating paddles have a real open bay.
    _add_box(frame, "top_cross_beam", (0.24, 1.48, 0.14), (0.25, 0.0, 2.06), weathered_oak)
    _add_box(frame, "rear_cross_beam", (1.86, 0.12, 0.12), (0.0, 0.62, 0.83), weathered_oak)
    _add_box(frame, "front_splash_rail", (0.10, 1.48, 0.10), (-1.025, 0.0, 0.34), galvanized)
    for i, y in enumerate((-0.62, 0.62)):
        _add_box(frame, f"crown_post_{i}", (0.12, 0.12, 1.08), (0.25, y, 1.53), weathered_oak)

    # Split-looking bearing plates use true annular meshes so the rotating axle
    # has clearance instead of colliding with a solid block.
    bearing_mesh = mesh_from_cadquery(_annular_disc(0.18, 0.067, 0.085), "open_side_bearing_ring")
    for i, y in enumerate((-0.62, 0.62)):
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, y, 1.05)),
            material=wrought_iron,
            name=f"bearing_ring_{i}",
        )
        _add_box(frame, f"bearing_saddle_{i}", (0.120, 0.085, 0.030), (0.0, y, 0.983), brass)
        flange_y = y
        _add_box(frame, f"bearing_flange_{i}_top", (0.46, 0.055, 0.070), (0.0, flange_y, 1.255), galvanized)
        _add_box(frame, f"bearing_flange_{i}_bottom", (0.46, 0.055, 0.070), (0.0, flange_y, 0.845), galvanized)
        _add_box(frame, f"bearing_flange_{i}_side_0", (0.070, 0.055, 0.46), (-0.205, flange_y, 1.05), galvanized)
        _add_box(frame, f"bearing_flange_{i}_side_1", (0.070, 0.055, 0.46), (0.205, flange_y, 1.05), galvanized)

        hatch_y = y + (0.123 if y > 0 else -0.123)
        handle_y = y + (0.151 if y > 0 else -0.151)
        latch_y = y + (0.153 if y > 0 else -0.153)
        _add_box(frame, f"service_hatch_{i}", (0.32, 0.026, 0.22), (0.0, hatch_y, 0.61), safety_blue)
        _add_box(frame, f"hatch_handle_{i}", (0.12, 0.030, 0.030), (0.0, handle_y, 0.61), galvanized)
        _add_box(frame, f"hatch_latch_{i}", (0.035, 0.034, 0.075), (0.12, latch_y, 0.61), wrought_iron)

        for b, (x, z) in enumerate(((-0.17, 0.845), (0.17, 0.845), (-0.17, 1.255), (0.17, 1.255))):
            _add_cylinder_y(
                frame,
                f"bearing_bolt_{i}_{b}",
                0.018,
                0.020,
                (x, y + (0.035 if y > 0 else -0.035), z),
                wrought_iron,
            )
        _add_cylinder_y(
            frame,
            f"grease_cup_{i}",
            0.025,
            0.040,
            (0.0, y, 1.25),
            brass,
        )

    wheel = model.part("wheel")
    # Centered steel axle.  The child frame is exactly on the axle centerline.
    _add_cylinder_y(wheel, "axle", 0.052, 1.42, (0.0, 0.0, 0.0), wrought_iron)
    _add_cylinder_y(wheel, "hub", 0.15, 0.58, (0.0, 0.0, 0.0), galvanized)
    _add_cylinder_y(wheel, "hub_sleeve", 0.095, 0.78, (0.0, 0.0, 0.0), wrought_iron)

    rim_mesh = mesh_from_cadquery(_annular_disc(0.74, 0.65, 0.052), "wood_side_rim")
    strap_mesh = mesh_from_cadquery(_annular_disc(0.765, 0.740, 0.030), "iron_tire_strap")
    for i, y in enumerate((-0.235, 0.235)):
        wheel.visual(rim_mesh, origin=Origin(xyz=(0.0, y, 0.0)), material=weathered_oak, name=f"side_rim_{i}")
        wheel.visual(strap_mesh, origin=Origin(xyz=(0.0, y, 0.0)), material=wrought_iron, name=f"rim_strap_{i}")
        _add_cylinder_y(wheel, f"adapter_plate_{i}", 0.22, 0.024, (0.0, y, 0.0), galvanized)

    spoke_count = 12
    for i in range(spoke_count):
        theta = 2.0 * math.pi * i / spoke_count
        for side_idx, y in enumerate((-0.235, 0.235)):
            # Spokes overlap both the hub and side rim, preserving a single rigid wheel.
            _add_box(
                wheel,
                f"spoke_{side_idx}_{i}",
                (0.63, 0.075, 0.055),
                (0.405 * math.cos(theta), y, 0.405 * math.sin(theta)),
                weathered_oak,
                rpy=_radial_rpy(theta),
            )
        # Cross-rib joins the pair of side rims at each blade station.
        _add_box(
            wheel,
            f"rim_cross_rib_{i}",
            (0.060, 0.54, 0.070),
            (0.695 * math.cos(theta), 0.0, 0.695 * math.sin(theta)),
            endgrain,
            rpy=_radial_rpy(theta),
        )

    paddle_count = 16
    for i in range(paddle_count):
        theta = 2.0 * math.pi * i / paddle_count
        _add_box(
            wheel,
            f"paddle_{i}",
            (0.18, 0.56, 0.050),
            (0.79 * math.cos(theta), 0.0, 0.79 * math.sin(theta)),
            weathered_oak,
            rpy=_radial_rpy(theta),
        )
        _add_box(
            wheel,
            f"paddle_iron_cap_{i}",
            (0.030, 0.58, 0.062),
            (0.895 * math.cos(theta), 0.0, 0.895 * math.sin(theta)),
            wrought_iron,
            rpy=_radial_rpy(theta),
        )

    for side_idx, y in enumerate((-0.255, 0.255)):
        for i in range(8):
            theta = 2.0 * math.pi * i / 8.0
            _add_cylinder_y(
                wheel,
                f"adapter_bolt_{side_idx}_{i}",
                0.012,
                0.018,
                (0.18 * math.cos(theta), y, 0.18 * math.sin(theta)),
                wrought_iron,
            )

    wheel_joint = model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=2.0),
        motion_properties=MotionProperties(damping=0.06, friction=0.015),
    )
    wheel_joint.meta["qc_samples"] = [0.0, math.pi / 8.0, math.pi / 4.0]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wheel = object_model.get_part("wheel")
    frame = object_model.get_part("frame")
    joint = object_model.get_articulation("frame_to_wheel")

    ctx.check(
        "wheel rotates about centered side axle",
        joint.axis == (0.0, 1.0, 0.0),
        details=f"axis={joint.axis}",
    )

    ctx.expect_overlap(
        wheel,
        frame,
        axes="xz",
        elem_a="axle",
        elem_b="bearing_ring_0",
        min_overlap=0.09,
        name="axle centered in first side bearing",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="xz",
        elem_a="axle",
        elem_b="bearing_ring_1",
        min_overlap=0.09,
        name="axle centered in second side bearing",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="bearing_ring_0",
        min_overlap=0.07,
        name="axle passes through first bearing width",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="bearing_ring_1",
        min_overlap=0.07,
        name="axle passes through second bearing width",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({joint: math.pi / 3.0}):
        rotated_pos = ctx.part_world_position(wheel)
        ctx.expect_overlap(
            wheel,
            frame,
            axes="xz",
            elem_a="axle",
            elem_b="bearing_ring_0",
            min_overlap=0.09,
            name="rotated axle remains centered in bearing",
        )
    ctx.check(
        "wheel rotation preserves axle origin",
        rest_pos is not None
        and rotated_pos is not None
        and max(abs(rest_pos[i] - rotated_pos[i]) for i in range(3)) < 1.0e-6,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
