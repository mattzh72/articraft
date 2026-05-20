from __future__ import annotations

import math

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
    TorusGeometry,
    mesh_from_geometry,
)


AXLE_Z = 5.20
WHEEL_RADIUS = 3.45
INNER_RING_RADIUS = 3.05
WHEEL_HALF_WIDTH = 0.38
GONDOLA_COUNT = 8


def _origin_for_cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        raise ValueError("Cannot make a tube from coincident points.")
    ux, uy, uz = vx / length, vy / length, vz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material | str,
    *,
    name: str,
    extra: float = 0.0,
) -> None:
    origin, length = _origin_for_cylinder_between(start, end)
    part.visual(
        Cylinder(radius=radius, length=length + 2.0 * extra),
        origin=origin,
        material=material,
        name=name,
    )


def _ring_mesh(radius: float, tube_radius: float, name: str):
    # TorusGeometry is authored in the local XY plane.  Rotate it into the
    # observation-wheel plane so the wheel spins about its horizontal Y axle.
    return mesh_from_geometry(
        TorusGeometry(radius=radius, tube=tube_radius, radial_segments=96, tubular_segments=16).rotate_x(
            math.pi / 2.0
        ),
        name,
    )


def _rim_point(angle: float, radius: float = WHEEL_RADIUS) -> tuple[float, float]:
    return radius * math.cos(angle), radius * math.sin(angle)


def _add_gondola_visuals(part, metal, seat_mat, trim, *, index: int) -> None:
    # The gondola frame origin is the hanger pivot.  All cabin geometry hangs
    # below local -Z so it can be counter-rotated to remain level.
    sleeve_len = 0.52
    part.visual(
        Cylinder(radius=0.065, length=sleeve_len),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hanger_sleeve",
    )

    for side_i, y in enumerate((-0.25, 0.25)):
        # Two continuous side hanger loops read as the open triangular yokes
        # that suspend the passenger basket from the pivot sleeve.
        pts = [
            (0.0, y, -0.085),
            (-0.42, y, -0.62),
            (-0.42, y, -1.05),
            (0.42, y, -1.05),
            (0.42, y, -0.62),
            (0.0, y, -0.085),
        ]
        for seg_i, (a, b) in enumerate(zip(pts, pts[1:])):
            _add_tube(
                part,
                a,
                b,
                0.022,
                metal,
                name=f"side_frame_{side_i}_{seg_i}",
                extra=0.012,
            )

    for bar_i, (x, z, r) in enumerate(
        [
            (-0.42, -1.05, 0.020),
            (0.42, -1.05, 0.020),
            (-0.36, -0.72, 0.018),
            (0.36, -0.72, 0.018),
            (0.0, -0.84, 0.018),
        ]
    ):
        _add_tube(
            part,
            (x, -0.27, z),
            (x, 0.27, z),
            r,
            metal,
            name=f"cross_rail_{bar_i}",
            extra=0.010,
        )

    # Open bucket-like passenger area: a bench pan, back, small side cheeks,
    # and a front safety rail, all visibly tied into the tubular frame.
    part.visual(
        Box((0.84, 0.50, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -1.12)),
        material=seat_mat,
        name="seat_pan",
    )
    part.visual(
        Box((0.84, 0.055, 0.42)),
        origin=Origin(xyz=(0.0, 0.245, -0.90)),
        material=seat_mat,
        name="back_rest",
    )
    part.visual(
        Box((0.055, 0.50, 0.30)),
        origin=Origin(xyz=(-0.43, 0.0, -0.94)),
        material=seat_mat,
        name="side_cheek_0",
    )
    part.visual(
        Box((0.055, 0.50, 0.30)),
        origin=Origin(xyz=(0.43, 0.0, -0.94)),
        material=seat_mat,
        name="side_cheek_1",
    )
    _add_tube(
        part,
        (-0.43, -0.25, -0.76),
        (0.43, -0.25, -0.76),
        0.024,
        trim,
        name="safety_bar",
        extra=0.012,
    )
    # Small pendant plate gives every seat a readable numbered fairground car.
    part.visual(
        Box((0.20, 0.020, 0.11)),
        origin=Origin(xyz=(0.0, -0.26, -1.02)),
        material=trim,
        name=f"number_plate_{index}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fairground_observation_wheel")

    painted_steel = model.material("painted_steel", rgba=(0.82, 0.08, 0.08, 1.0))
    cream_steel = model.material("cream_steel", rgba=(0.92, 0.85, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.15, 0.16, 0.17, 1.0))
    bearing_blue = model.material("bearing_blue", rgba=(0.08, 0.24, 0.56, 1.0))
    brass = model.material("brass", rgba=(0.86, 0.63, 0.22, 1.0))
    seat_red = model.material("seat_red", rgba=(0.72, 0.06, 0.05, 1.0))
    seat_yellow = model.material("seat_yellow", rgba=(0.98, 0.78, 0.17, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    support = model.part("support")
    # Ground skids and cross ties make one connected base between the two
    # A-frame side supports.
    for i, y in enumerate((-0.82, 0.82)):
        _add_tube(
            support,
            (-2.85, y, 0.12),
            (2.85, y, 0.12),
            0.080,
            dark_steel,
            name=f"base_skid_{i}",
            extra=0.05,
        )
        _add_tube(
            support,
            (-2.55, y, 0.12),
            (0.0, y, AXLE_Z),
            0.085,
            painted_steel,
            name=f"leg_{i}_0",
            extra=0.06,
        )
        _add_tube(
            support,
            (2.55, y, 0.12),
            (0.0, y, AXLE_Z),
            0.085,
            painted_steel,
            name=f"leg_{i}_1",
            extra=0.06,
        )
        _add_tube(
            support,
            (-1.65, y, 1.95),
            (1.65, y, 1.95),
            0.050,
            cream_steel,
            name=f"lower_tie_{i}",
            extra=0.04,
        )
        _add_tube(
            support,
            (-1.05, y, 3.25),
            (1.05, y, 3.25),
            0.045,
            cream_steel,
            name=f"upper_tie_{i}",
            extra=0.035,
        )
        _add_tube(
            support,
            (-2.00, y, 0.12),
            (1.05, y, 3.25),
            0.040,
            cream_steel,
            name=f"diagonal_brace_{i}_0",
            extra=0.04,
        )
        _add_tube(
            support,
            (2.00, y, 0.12),
            (-1.05, y, 3.25),
            0.040,
            cream_steel,
            name=f"diagonal_brace_{i}_1",
            extra=0.04,
        )
        support.visual(
            Cylinder(radius=0.22, length=0.22),
            origin=Origin(xyz=(0.0, y, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bearing_blue,
            name=f"bearing_housing_{i}",
        )

    for i, x in enumerate((-2.55, 0.0, 2.55)):
        _add_tube(
            support,
            (x, -0.82, 0.12),
            (x, 0.82, 0.12),
            0.070,
            dark_steel,
            name=f"base_cross_tie_{i}",
            extra=0.04,
        )
    _add_tube(
        support,
        (0.0, -0.95, AXLE_Z),
        (0.0, 0.95, AXLE_Z),
        0.095,
        dark_steel,
        name="fixed_axle",
        extra=0.02,
    )
    support.visual(
        Box((1.20, 0.22, 0.16)),
        origin=Origin(xyz=(-2.25, 0.0, 0.16)),
        material=rubber,
        name="left_foot_pad",
    )
    support.visual(
        Box((1.20, 0.22, 0.16)),
        origin=Origin(xyz=(2.25, 0.0, 0.16)),
        material=rubber,
        name="right_foot_pad",
    )

    wheel = model.part("wheel")
    outer_ring = _ring_mesh(WHEEL_RADIUS, 0.052, "outer_rim")
    inner_ring = _ring_mesh(INNER_RING_RADIUS, 0.036, "inner_rim")
    for i, y in enumerate((-WHEEL_HALF_WIDTH, WHEEL_HALF_WIDTH)):
        wheel.visual(outer_ring, origin=Origin(xyz=(0.0, y, 0.0)), material=painted_steel, name=f"outer_rim_{i}")
        wheel.visual(inner_ring, origin=Origin(xyz=(0.0, y, 0.0)), material=cream_steel, name=f"inner_rim_{i}")
        for spoke_i in range(16):
            angle = 2.0 * math.pi * spoke_i / 16.0
            _add_tube(
                wheel,
                (0.34 * math.cos(angle), y, 0.34 * math.sin(angle)),
                (INNER_RING_RADIUS * math.cos(angle), y, INNER_RING_RADIUS * math.sin(angle)),
                0.026,
                cream_steel,
                name=f"spoke_{i}_{spoke_i}",
                extra=0.025,
            )
        for tie_i in range(GONDOLA_COUNT):
            angle = 2.0 * math.pi * tie_i / GONDOLA_COUNT
            _add_tube(
                wheel,
                (INNER_RING_RADIUS * math.cos(angle), y, INNER_RING_RADIUS * math.sin(angle)),
                (WHEEL_RADIUS * math.cos(angle), y, WHEEL_RADIUS * math.sin(angle)),
                0.030,
                painted_steel,
                name=f"rim_tie_{i}_{tie_i}",
                extra=0.020,
            )
    wheel.visual(
        Cylinder(radius=0.32, length=0.92),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_blue,
        name="rotating_hub",
    )
    wheel.visual(
        Cylinder(radius=0.13, length=1.02),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_barrel",
    )
    for car_i in range(GONDOLA_COUNT):
        angle = 2.0 * math.pi * car_i / GONDOLA_COUNT
        x, z = _rim_point(angle)
        wheel.visual(
            Cylinder(radius=0.034, length=0.90),
            origin=Origin(xyz=(x, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"pivot_pin_{car_i}",
        )

    model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.35),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    for car_i in range(GONDOLA_COUNT):
        gondola = model.part(f"gondola_{car_i}")
        _add_gondola_visuals(
            gondola,
            brass,
            seat_red if car_i % 2 == 0 else seat_yellow,
            dark_steel,
            index=car_i + 1,
        )
        angle = 2.0 * math.pi * car_i / GONDOLA_COUNT
        x, z = _rim_point(angle)
        model.articulation(
            f"wheel_to_gondola_{car_i}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=(x, 0.0, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.5, velocity=2.5),
            motion_properties=MotionProperties(damping=0.05, friction=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wheel = object_model.get_part("wheel")
    support = object_model.get_part("support")
    main_joint = object_model.get_articulation("support_to_wheel")

    ctx.check(
        "main wheel uses continuous rotation",
        main_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={main_joint.articulation_type}",
    )
    ctx.allow_overlap(
        support,
        wheel,
        elem_a="fixed_axle",
        elem_b="hub_barrel",
        reason="The rotating hub barrel is intentionally modeled as a bushing captured around the fixed support axle.",
    )
    ctx.allow_overlap(
        support,
        wheel,
        elem_a="fixed_axle",
        elem_b="rotating_hub",
        reason="The visible hub boss surrounds the same fixed axle as part of the captured bearing assembly.",
    )
    ctx.expect_contact(
        support,
        wheel,
        elem_a="fixed_axle",
        elem_b="hub_barrel",
        contact_tol=0.001,
        name="rotating hub is captured on the fixed axle",
    )
    ctx.expect_contact(
        support,
        wheel,
        elem_a="fixed_axle",
        elem_b="rotating_hub",
        contact_tol=0.001,
        name="visible hub boss is seated around the axle",
    )

    for car_i in range(GONDOLA_COUNT):
        gondola = object_model.get_part(f"gondola_{car_i}")
        joint = object_model.get_articulation(f"wheel_to_gondola_{car_i}")
        pin_name = f"pivot_pin_{car_i}"
        ctx.check(
            f"gondola {car_i} has a continuous hanger pivot",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.allow_overlap(
            wheel,
            gondola,
            elem_a=pin_name,
            elem_b="hanger_sleeve",
            reason="The gondola sleeve is intentionally modeled as a captured bushing around the rim hanger pin.",
        )
        ctx.expect_within(
            wheel,
            gondola,
            axes="xz",
            inner_elem=pin_name,
            outer_elem="hanger_sleeve",
            margin=0.004,
            name=f"gondola {car_i} sleeve surrounds its rim pin",
        )
        ctx.expect_overlap(
            wheel,
            gondola,
            axes="y",
            elem_a=pin_name,
            elem_b="hanger_sleeve",
            min_overlap=0.45,
            name=f"gondola {car_i} remains clipped along the hanger pin",
        )

    # At a quarter-turn main-wheel pose, a free gondola can counter-rotate about
    # its captured pin and keep the seat below the hanger instead of flipping
    # with the rim.
    gondola_0 = object_model.get_part("gondola_0")
    gondola_joint_0 = object_model.get_articulation("wheel_to_gondola_0")
    with ctx.pose({main_joint: math.pi / 2.0, gondola_joint_0: -math.pi / 2.0}):
        pivot_position = ctx.part_world_position(gondola_0)
        seat_aabb = ctx.part_element_world_aabb(gondola_0, elem="seat_pan")
        seat_center_z = None
        if seat_aabb is not None:
            seat_center_z = (seat_aabb[0][2] + seat_aabb[1][2]) * 0.5
        ctx.check(
            "counter-rotated gondola hangs below its pivot",
            pivot_position is not None
            and seat_center_z is not None
            and seat_center_z < pivot_position[2] - 0.85,
            details=f"pivot={pivot_position}, seat_center_z={seat_center_z}",
        )
        ctx.expect_within(
            wheel,
            gondola_0,
            axes="xz",
            inner_elem="pivot_pin_0",
            outer_elem="hanger_sleeve",
            margin=0.004,
            name="rotated wheel keeps gondola 0 pin inside sleeve",
        )

    return ctx.report()


object_model = build_object_model()
