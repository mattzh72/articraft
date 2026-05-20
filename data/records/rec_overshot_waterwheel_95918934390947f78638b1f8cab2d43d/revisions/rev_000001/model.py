from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin

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
)


def _beam_length_and_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[float, Origin]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = (dx * dx + dy * dy + dz * dz) ** 0.5
    mid = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    yaw = atan2(dy, dx)
    pitch = -atan2(dz, hypot(dx, dy))
    return length, Origin(xyz=mid, rpy=(0.0, pitch, yaw))


def _add_beam(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    length, origin = _beam_length_and_origin(start, end)
    part.visual(
        Box((length, width, depth)),
        origin=origin,
        material=material,
        name=name,
    )


def _make_bucket_mesh(width: float):
    profile = [
        (-0.20, 0.06),
        (-0.14, 0.15),
        (0.02, 0.15),
        (0.16, 0.07),
        (0.20, -0.02),
        (0.14, -0.14),
        (-0.02, -0.20),
        (-0.16, -0.12),
        (-0.20, -0.04),
    ]
    bucket_geom = ExtrudeGeometry(profile, width, center=True, cap=True, closed=True).rotate_x(-pi / 2.0)
    return mesh_from_geometry(bucket_geom, "overshot_bucket")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_wheel")

    weathered_timber = model.material("weathered_timber", rgba=(0.52, 0.40, 0.25, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.38, 0.27, 0.16, 1.0))
    iron = model.material("iron", rgba=(0.33, 0.34, 0.36, 1.0))
    hatch_paint = model.material("hatch_paint", rgba=(0.45, 0.29, 0.18, 1.0))

    axle_height = 1.80
    wheel_radius = 1.35
    ring_radius = 1.12
    wheel_width = 0.46
    ring_y = 0.24
    support_y = 0.44
    bucket_count = 12

    bucket_mesh = _make_bucket_mesh(width=wheel_width + 0.02)

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((3.20, 1.18, 2.40)),
        mass=650.0,
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
    )

    # Ground sills and transverse ties.
    frame.visual(
        Box((3.10, 0.14, 0.18)),
        origin=Origin(xyz=(0.0, -0.50, 0.09)),
        material=weathered_timber,
        name="left_ground_sill",
    )
    frame.visual(
        Box((3.10, 0.14, 0.18)),
        origin=Origin(xyz=(0.0, 0.50, 0.09)),
        material=weathered_timber,
        name="right_ground_sill",
    )
    for x_pos, name in [(-1.15, "front_tie"), (1.15, "rear_tie"), (0.0, "center_tie")]:
        frame.visual(
            Box((0.18, 1.12, 0.14)),
            origin=Origin(xyz=(x_pos, 0.0, 0.15)),
            material=weathered_timber,
            name=name,
        )

    # Side posts and head beams.
    for x_pos, side_name in [(-0.98, "front"), (0.98, "rear")]:
        for y_pos, y_name in [(-support_y, "left"), (support_y, "right")]:
            frame.visual(
                Box((0.16, 0.12, 2.20)),
                origin=Origin(xyz=(x_pos, y_pos, 1.10)),
                material=weathered_timber,
                name=f"{side_name}_{y_name}_post",
            )

    for y_pos, y_name in [(-support_y, "left"), (support_y, "right")]:
        frame.visual(
            Box((0.20, 0.12, 2.04)),
            origin=Origin(xyz=(0.0, y_pos, 1.02)),
            material=dark_timber,
            name=f"{y_name}_center_post",
        )
        frame.visual(
            Box((2.22, 0.14, 0.18)),
            origin=Origin(xyz=(0.0, y_pos, 2.19)),
            material=weathered_timber,
            name=f"{y_name}_head_beam",
        )

    # Upper and lower cross ties between the two side frames.
    for x_pos, z_pos, name in [
        (-1.00, 0.48, "front_lower_tie"),
        (1.00, 0.48, "rear_lower_tie"),
    ]:
        frame.visual(
            Box((0.16, 0.92, 0.12)),
            origin=Origin(xyz=(x_pos, 0.0, z_pos)),
            material=weathered_timber,
            name=name,
        )

    # Diagonal timber braces in each side plane.
    for y_pos, y_name in [(-support_y, "left"), (support_y, "right")]:
        _add_beam(
            frame,
            (-0.98, y_pos, 0.30),
            (-0.05, y_pos, 1.52),
            width=0.10,
            depth=0.12,
            material=dark_timber,
            name=f"{y_name}_front_brace",
        )
        _add_beam(
            frame,
            (0.98, y_pos, 0.30),
            (0.05, y_pos, 1.52),
            width=0.10,
            depth=0.12,
            material=dark_timber,
            name=f"{y_name}_rear_brace",
        )

    # Bearing support boxes around the axle line.
    for (
        y_pos,
        top_name,
        bottom_name,
        front_upright_name,
        rear_upright_name,
    ) in [
        (
            -support_y,
            "left_support_top",
            "left_support_bottom",
            "left_support_front_upright",
            "left_support_rear_upright",
        ),
        (
            support_y,
            "right_support_top",
            "right_support_bottom",
            "right_support_front_upright",
            "right_support_rear_upright",
        ),
    ]:
        frame.visual(
            Box((0.52, 0.14, 0.10)),
            origin=Origin(xyz=(0.0, y_pos, axle_height + 0.20)),
            material=dark_timber,
            name=top_name,
        )
        frame.visual(
            Box((0.52, 0.14, 0.10)),
            origin=Origin(xyz=(0.0, y_pos, axle_height - 0.20)),
            material=dark_timber,
            name=bottom_name,
        )
        frame.visual(
            Box((0.08, 0.14, 0.32)),
            origin=Origin(xyz=(-0.22, y_pos, axle_height)),
            material=dark_timber,
            name=front_upright_name,
        )
        frame.visual(
            Box((0.08, 0.14, 0.32)),
            origin=Origin(xyz=(0.22, y_pos, axle_height)),
            material=dark_timber,
            name=rear_upright_name,
        )

    # Thin bearing liners bridge from the support boxes to the axle journals.
    for y_pos, name in [(-0.37, "left_bearing_liner"), (0.37, "right_bearing_liner")]:
        frame.visual(
            Box((0.20, 0.02, 0.18)),
            origin=Origin(xyz=(0.0, y_pos, axle_height)),
            material=iron,
            name=name,
        )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=0.74),
        mass=320.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    wheel.visual(
        Cylinder(radius=0.055, length=0.72),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.18, length=0.54),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_timber,
        name="hub_drum",
    )
    for y_pos, name in [(-0.33, "left_collar"), (0.33, "right_collar")]:
        wheel.visual(
            Cylinder(radius=0.10, length=0.05),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=name,
        )

    # Radial timber spokes spanning between the two shroud rings.
    spoke_length = 1.03
    spoke_mid_radius = 0.65
    for index in range(bucket_count):
        angle = 2.0 * pi * index / bucket_count
        wheel.visual(
            Box((spoke_length, wheel_width - 0.04, 0.10)),
            origin=Origin(
                xyz=(cos(angle) * spoke_mid_radius, 0.0, sin(angle) * spoke_mid_radius),
                rpy=(0.0, -angle, 0.0),
            ),
            material=weathered_timber,
            name=f"spoke_{index}",
        )

    # Outer shroud rings on both sides of the wheel.
    ring_segments = bucket_count * 2
    segment_length = 2.0 * pi * ring_radius / ring_segments * 0.96
    for side_y, side_name in [(-ring_y, "inner_face"), (ring_y, "outer_face")]:
        for index in range(ring_segments):
            angle = 2.0 * pi * index / ring_segments
            wheel.visual(
                Box((segment_length, 0.05, 0.16)),
                origin=Origin(
                    xyz=(cos(angle) * ring_radius, side_y, sin(angle) * ring_radius),
                    rpy=(0.0, -angle - pi / 2.0, 0.0),
                ),
                material=dark_timber,
                name=f"{side_name}_ring_segment_{index}",
            )

    # Curved rim buckets.
    bucket_radius = 1.19
    for index in range(bucket_count):
        angle = pi / 2.0 + 2.0 * pi * index / bucket_count
        wheel.visual(
            bucket_mesh,
            origin=Origin(
                xyz=(cos(angle) * bucket_radius, 0.0, sin(angle) * bucket_radius),
                rpy=(0.0, -angle - pi / 2.0, 0.0),
            ),
            material=weathered_timber,
            name=f"bucket_{index}",
        )

    service_hatch = model.part("service_hatch")
    service_hatch.inertial = Inertial.from_geometry(
        Box((0.31, 0.05, 0.32)),
        mass=12.0,
        origin=Origin(xyz=(0.15, 0.025, 0.0)),
    )
    service_hatch.visual(
        Box((0.30, 0.028, 0.30)),
        origin=Origin(xyz=(0.15, 0.014, 0.0)),
        material=hatch_paint,
        name="service_hatch_panel",
    )
    service_hatch.visual(
        Box((0.24, 0.020, 0.04)),
        origin=Origin(xyz=(0.15, 0.030, 0.10)),
        material=dark_timber,
        name="service_hatch_upper_batten",
    )
    service_hatch.visual(
        Box((0.24, 0.020, 0.04)),
        origin=Origin(xyz=(0.15, 0.030, -0.10)),
        material=dark_timber,
        name="service_hatch_lower_batten",
    )
    for z_pos, name in [(0.11, "upper_hinge_barrel"), (0.0, "center_hinge_barrel"), (-0.11, "lower_hinge_barrel")]:
        service_hatch.visual(
            Cylinder(radius=0.012, length=0.04),
            origin=Origin(xyz=(0.0, 0.015, z_pos)),
            material=iron,
            name=name,
        )
    service_hatch.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(xyz=(0.25, 0.038, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="service_hatch_handle",
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.6),
    )
    model.articulation(
        "service_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=service_hatch,
        origin=Origin(xyz=(-0.15, support_y + 0.07, axle_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    service_hatch = object_model.get_part("service_hatch")
    wheel_spin = object_model.get_articulation("wheel_spin")
    service_hatch_hinge = object_model.get_articulation("service_hatch_hinge")

    ctx.check(
        "wheel spin articulation is continuous on the axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 4) for value in wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.check(
        "service hatch uses a vertical side hinge",
        service_hatch_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(value, 4) for value in service_hatch_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={service_hatch_hinge.articulation_type}, axis={service_hatch_hinge.axis}",
    )

    ctx.check(
        "wheel axle sits high enough for an overshot layout",
        ctx.part_world_position(wheel) is not None and ctx.part_world_position(wheel)[2] > 1.65,
        details=f"wheel_position={ctx.part_world_position(wheel)}",
    )

    with ctx.pose({service_hatch_hinge: 0.0}):
        ctx.expect_gap(
            service_hatch,
            frame,
            axis="y",
            positive_elem="service_hatch_panel",
            negative_elem="right_support_top",
            max_gap=0.008,
            max_penetration=0.0,
            name="closed service hatch sits flush on the support box",
        )
        ctx.expect_contact(
            wheel,
            frame,
            elem_a="axle_shaft",
            elem_b="left_bearing_liner",
            contact_tol=1e-6,
            name="axle bears on the left support liner",
        )
        ctx.expect_contact(
            wheel,
            frame,
            elem_a="axle_shaft",
            elem_b="right_bearing_liner",
            contact_tol=1e-6,
            name="axle bears on the right support liner",
        )
        closed_panel_aabb = ctx.part_element_world_aabb(service_hatch, elem="service_hatch_panel")

    with ctx.pose({service_hatch_hinge: 1.20}):
        open_panel_aabb = ctx.part_element_world_aabb(service_hatch, elem="service_hatch_panel")

    hatch_opens_outward = (
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.10
    )
    ctx.check(
        "service hatch swings outward from the support box",
        hatch_opens_outward,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    with ctx.pose({wheel_spin: 0.0}):
        bucket_rest = ctx.part_element_world_aabb(wheel, elem="bucket_0")
    with ctx.pose({wheel_spin: pi / 2.0}):
        bucket_quarter_turn = ctx.part_element_world_aabb(wheel, elem="bucket_0")

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    bucket_rest_z = _aabb_center_z(bucket_rest)
    bucket_quarter_z = _aabb_center_z(bucket_quarter_turn)
    ctx.check(
        "wheel rotation carries the buckets around the rim",
        bucket_rest_z is not None
        and bucket_quarter_z is not None
        and abs(bucket_rest_z - bucket_quarter_z) > 0.70,
        details=f"rest={bucket_rest}, quarter_turn={bucket_quarter_turn}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
