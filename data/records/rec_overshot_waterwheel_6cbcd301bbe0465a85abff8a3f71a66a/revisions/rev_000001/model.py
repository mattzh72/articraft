from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


WHEEL_CENTER_Z = 1.05
WHEEL_RADIUS = 0.72
BUCKET_OUTER_RADIUS = 0.78
WHEEL_WIDTH = 0.22


def _origin_for_cylinder_between(start, end) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("beam endpoints must be distinct")
    theta = math.acos(max(-1.0, min(1.0, dz / length)))
    yaw = math.atan2(dy, dx)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, theta, yaw),
        ),
        length,
    )


def _add_round_beam(part, start, end, radius, material, name: str) -> None:
    origin, length = _origin_for_cylinder_between(start, end)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    wood = model.material("weathered_wood", rgba=(0.48, 0.29, 0.13, 1.0))
    dark_wood = model.material("dark_wet_wood", rgba=(0.30, 0.17, 0.08, 1.0))
    metal = model.material("dull_iron", rgba=(0.30, 0.31, 0.30, 1.0))
    water = model.material("chute_water", rgba=(0.30, 0.62, 0.95, 0.58))

    frame = model.part("frame")

    # Low timber base and trestle feet.
    frame.visual(Box((0.10, 2.10, 0.10)), origin=Origin(xyz=(-0.56, -0.12, 0.05)), material=wood, name="base_skid_0")
    frame.visual(Box((0.10, 2.10, 0.10)), origin=Origin(xyz=(0.56, -0.12, 0.05)), material=wood, name="base_skid_1")
    for i, y in enumerate((-1.05, -0.93, -0.58, 0.62)):
        frame.visual(Box((1.28, 0.10, 0.10)), origin=Origin(xyz=(0.0, y, 0.08)), material=wood, name=f"base_cross_{i}")

    # Two A-frame side supports, one at each end of the axle.
    beam_radius = 0.038
    for side_i, x in enumerate((-0.50, 0.50)):
        _add_round_beam(frame, (x, -0.58, 0.10), (x, -0.105, WHEEL_CENTER_Z), beam_radius, wood, f"side_leg_{side_i}_0")
        _add_round_beam(frame, (x, 0.58, 0.10), (x, 0.105, WHEEL_CENTER_Z), beam_radius, wood, f"side_leg_{side_i}_1")
        _add_round_beam(frame, (x, 0.0, 0.08), (x, 0.0, WHEEL_CENTER_Z - 0.11), beam_radius * 0.85, wood, f"king_post_{side_i}")
        frame.visual(
            mesh_from_geometry(TorusGeometry(0.075, 0.020, radial_segments=18, tubular_segments=36), f"bearing_ring_{side_i}"),
            origin=Origin(xyz=(x, 0.0, WHEEL_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"bearing_ring_{side_i}",
        )
        frame.visual(
            Box((0.13, 0.18, 0.050)),
            origin=Origin(xyz=(x, 0.0, WHEEL_CENTER_Z - 0.070)),
            material=metal,
            name=f"bearing_saddle_{side_i}",
        )

    # Short sloping overshot chute above the rim.  It is open-topped and braced
    # by posts tied into the base, not a floating decorative box.
    chute_roll = -0.16
    chute_center_y = -0.61
    chute_center_z = 2.00
    frame.visual(
        Box((0.38, 0.78, 0.035)),
        origin=Origin(xyz=(0.0, chute_center_y, chute_center_z), rpy=(chute_roll, 0.0, 0.0)),
        material=dark_wood,
        name="chute_bottom",
    )
    frame.visual(
        Box((0.035, 0.78, 0.17)),
        origin=Origin(xyz=(-0.205, chute_center_y, chute_center_z + 0.075), rpy=(chute_roll, 0.0, 0.0)),
        material=wood,
        name="chute_side_0",
    )
    frame.visual(
        Box((0.035, 0.78, 0.17)),
        origin=Origin(xyz=(0.205, chute_center_y, chute_center_z + 0.075), rpy=(chute_roll, 0.0, 0.0)),
        material=wood,
        name="chute_side_1",
    )
    frame.visual(
        Box((0.27, 0.60, 0.020)),
        origin=Origin(xyz=(0.0, chute_center_y + 0.02, chute_center_z + 0.018), rpy=(chute_roll, 0.0, 0.0)),
        material=water,
        name="water_ribbon",
    )
    for i, x in enumerate((-0.25, 0.25)):
        _add_round_beam(frame, (x, -0.93, 0.10), (x, -0.93, 2.04), 0.024, wood, f"chute_post_{i}")
    frame.visual(Box((0.62, 0.045, 0.045)), origin=Origin(xyz=(0.0, -0.93, 2.04)), material=wood, name="chute_crossbar")

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                WHEEL_RADIUS,
                0.16,
                rim=WheelRim(inner_radius=0.57, flange_height=0.035, flange_thickness=0.018, bead_seat_depth=0.0),
                hub=WheelHub(radius=0.10, width=0.20, cap_style="flat"),
                face=WheelFace(dish_depth=0.0, front_inset=0.006, rear_inset=0.006),
                spokes=WheelSpokes(style="straight", count=12, thickness=0.018, window_radius=0.040),
                bore=WheelBore(style="round", diameter=0.080),
            ),
            "wooden_spoked_wheel",
        ),
        material=wood,
        name="spoked_wheel",
    )
    # Axle aligned with local/world X so the visible wheel and the joint axis agree.
    wheel.visual(
        Cylinder(radius=0.045, length=1.08),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="axle",
    )

    # Shallow wooden buckets around the outside.  Each trough overlaps the outer
    # rim slightly, keeping the rotating assembly visually and mechanically tied
    # together while leaving clear gaps between bucket mouths.
    bucket_count = 16
    for i in range(bucket_count):
        theta = 2.0 * math.pi * i / bucket_count
        y = math.cos(theta) * (BUCKET_OUTER_RADIUS - 0.035)
        z = math.sin(theta) * (BUCKET_OUTER_RADIUS - 0.035)
        bucket_origin = Origin(xyz=(0.0, y, z), rpy=(theta, 0.0, 0.0))
        wheel.visual(
            Box((WHEEL_WIDTH, 0.080, 0.155)),
            origin=bucket_origin,
            material=dark_wood,
            name=f"bucket_floor_{i}",
        )
        lip_y = math.cos(theta) * (BUCKET_OUTER_RADIUS + 0.020)
        lip_z = math.sin(theta) * (BUCKET_OUTER_RADIUS + 0.020)
        wheel.visual(
            Box((WHEEL_WIDTH, 0.025, 0.155)),
            origin=Origin(xyz=(0.0, lip_y, lip_z), rpy=(theta, 0.0, 0.0)),
            material=wood,
            name=f"bucket_lip_{i}",
        )
        for side, x in enumerate((-WHEEL_WIDTH * 0.5, WHEEL_WIDTH * 0.5)):
            wheel.visual(
                Box((0.018, 0.090, 0.155)),
                origin=Origin(xyz=(x, y, z), rpy=(theta, 0.0, 0.0)),
                material=wood,
                name=f"bucket_cheek_{i}_{side}",
            )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    joint = object_model.get_articulation("frame_to_wheel")

    ctx.check("continuous_axle_joint", joint is not None and joint.articulation_type == ArticulationType.CONTINUOUS, "Wheel should rotate continuously.")
    ctx.check("axle_axis_horizontal", joint is not None and tuple(joint.axis) == (1.0, 0.0, 0.0), f"axis={getattr(joint, 'axis', None)!r}")

    wheel_aabb = ctx.part_world_aabb(wheel)
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check("large_wheel_present", wheel_aabb is not None and (wheel_aabb[1][2] - wheel_aabb[0][2]) > 1.4, f"wheel_aabb={wheel_aabb!r}")
    ctx.check("frame_taller_than_wheel_center", frame_aabb is not None and frame_aabb[1][2] > WHEEL_CENTER_Z + WHEEL_RADIUS, f"frame_aabb={frame_aabb!r}")

    ctx.expect_overlap(wheel, frame, axes="x", min_overlap=0.40, elem_a="axle", name="axle spans both side bearings")
    ctx.expect_gap(frame, wheel, axis="z", positive_elem="chute_bottom", max_gap=0.18, name="chute sits just over top of wheel")

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
        ctx.expect_gap(frame, wheel, axis="z", positive_elem="chute_bottom", max_gap=0.22, name="wheel turns below chute")
    ctx.check("rotation_keeps_axle_centered", rest_pos == turned_pos, f"rest={rest_pos!r}, turned={turned_pos!r}")

    return ctx.report()


object_model = build_object_model()
