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
    SpurGear,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel_with_pinion")

    aged_oak = model.material("aged_oak", rgba=(0.55, 0.32, 0.14, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.35, 0.20, 0.09, 1.0))
    stone = model.material("dressed_stone", rgba=(0.50, 0.48, 0.44, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.62, 0.43, 0.20, 1.0))
    iron = model.material("dark_iron", rgba=(0.08, 0.08, 0.075, 1.0))
    water = model.material("falling_water", rgba=(0.18, 0.48, 0.85, 0.72))

    frame = model.part("frame")
    frame.visual(
        Box((1.90, 1.22, 0.12)),
        origin=Origin(xyz=(0.0, 0.03, -0.86)),
        material=stone,
        name="stone_base",
    )

    for suffix, y in (("near", -0.38), ("far", 0.38)):
        frame.visual(
            Box((0.50, 0.18, 0.14)),
            origin=Origin(xyz=(0.0, y, -0.78)),
            material=stone,
            name=f"{suffix}_plinth",
        )
        frame.visual(
            Box((0.16, 0.18, 0.75)),
            origin=Origin(xyz=(0.0, y, -0.45)),
            material=stone,
            name=f"{suffix}_saddle",
        )
        for x in (-0.20, 0.20):
            frame.visual(
                Box((0.13, 0.18, 1.30)),
                origin=Origin(xyz=(x, y, -0.18)),
                material=stone,
                name=f"{suffix}_cheek_{'neg' if x < 0 else 'pos'}",
            )
        frame.visual(
            Box((0.52, 0.18, 0.16)),
            origin=Origin(xyz=(0.0, y, 0.55)),
            material=stone,
            name=f"{suffix}_cap",
        )
        frame.visual(
            mesh_from_geometry(TorusGeometry(0.065, 0.015, radial_segments=18, tubular_segments=32), f"{suffix}_bearing_ring"),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bronze,
            name=f"{suffix}_bearing_ring",
        )

    # Overshot flume: a sloped wooden trough above the wheel with a small
    # translucent stream leaving its lip.
    flume_angle = 0.18
    frame.visual(
        Box((0.86, 0.58, 0.045)),
        origin=Origin(xyz=(-0.55, 0.0, 0.94), rpy=(0.0, flume_angle, 0.0)),
        material=aged_oak,
        name="flume_floor",
    )
    for y in (-0.30, 0.30):
        frame.visual(
            Box((0.86, 0.045, 0.16)),
            origin=Origin(xyz=(-0.55, y, 1.01), rpy=(0.0, flume_angle, 0.0)),
            material=aged_oak,
            name=f"flume_wall_{'near' if y < 0 else 'far'}",
        )
    frame.visual(
        Box((0.70, 0.46, 0.018)),
        origin=Origin(xyz=(-0.57, 0.0, 0.985), rpy=(0.0, flume_angle, 0.0)),
        material=water,
        name="water_in_flume",
    )
    frame.visual(
        Box((0.11, 0.38, 0.20)),
        origin=Origin(xyz=(-0.21, 0.0, 0.86), rpy=(0.0, 0.08, 0.0)),
        material=water,
        name="falling_water_sheet",
    )
    for y in (-0.31, 0.31):
        frame.visual(
            Box((0.09, 0.09, 1.83)),
            origin=Origin(xyz=(-0.82, y, 0.115)),
            material=aged_oak,
            name=f"flume_post_{'near' if y < 0 else 'far'}",
        )

    wheel = model.part("wheel_assembly")
    wheel.visual(
        Cylinder(radius=0.050, length=1.42),
        origin=Origin(xyz=(0.0, 0.10, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.12, length=0.54),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_oak,
        name="hub",
    )
    for suffix, y in (("near", -0.22), ("far", 0.22)):
        wheel.visual(
            mesh_from_geometry(TorusGeometry(0.66, 0.035, radial_segments=24, tubular_segments=72), f"{suffix}_main_rim"),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aged_oak,
            name=f"{suffix}_main_rim",
        )

    for i in range(8):
        theta = i * math.tau / 8.0
        wheel.visual(
            Box((0.60, 0.08, 0.055)),
            origin=Origin(
                xyz=(0.37 * math.cos(theta), 0.0, 0.37 * math.sin(theta)),
                rpy=(0.0, -theta, 0.0),
            ),
            material=dark_oak,
            name=f"spoke_{i}",
        )

    # Broad trough-like buckets around the outer rim make the wheel read as an
    # overshot waterwheel rather than a plain flywheel.
    for i in range(16):
        theta = i * math.tau / 16.0 + math.pi / 32.0
        wheel.visual(
            Box((0.20, 0.46, 0.060)),
            origin=Origin(
                xyz=(0.685 * math.cos(theta), 0.0, 0.685 * math.sin(theta)),
                rpy=(0.0, math.pi / 2.0 - theta, 0.0),
            ),
            material=aged_oak,
            name=f"bucket_{i}",
        )

    pinion_shape = SpurGear(
        module=0.020,
        teeth_number=18,
        width=0.09,
        pressure_angle=20.0,
    ).build(
        bore_d=0.075,
        hub_d=0.16,
        hub_length=0.13,
        n_spokes=6,
        spoke_width=0.030,
        spokes_id=0.10,
        spokes_od=0.27,
        chamfer=0.004,
    )
    wheel.visual(
        mesh_from_cadquery(pinion_shape, "side_pinion", tolerance=0.0008, angular_tolerance=0.08),
        origin=Origin(xyz=(0.0, 0.54, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="side_pinion",
    )
    wheel.visual(
        Cylinder(radius=0.070, length=0.24),
        origin=Origin(xyz=(0.0, 0.62, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="pinion_hub_collar",
    )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel_assembly")
    joint = object_model.get_articulation("frame_to_wheel")

    ctx.check(
        "waterwheel has continuous horizontal axle joint",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    near_rim = ctx.part_element_world_aabb(wheel, elem="near_main_rim")
    far_rim = ctx.part_element_world_aabb(wheel, elem="far_main_rim")
    near_cap = ctx.part_element_world_aabb(frame, elem="near_cap")
    far_cap = ctx.part_element_world_aabb(frame, elem="far_cap")
    pinion = ctx.part_element_world_aabb(wheel, elem="side_pinion")
    ctx.check(
        "main wheel sits between side support blocks",
        near_rim is not None
        and far_rim is not None
        and near_cap is not None
        and far_cap is not None
        and near_cap[1][1] < near_rim[0][1]
        and far_cap[0][1] > far_rim[1][1],
        details=f"near_cap={near_cap}, near_rim={near_rim}, far_cap={far_cap}, far_rim={far_rim}",
    )
    ctx.check(
        "drive pinion is outside the main rim on the axle end",
        pinion is not None and far_rim is not None and pinion[0][1] > far_rim[1][1] + 0.10,
        details=f"pinion={pinion}, far_rim={far_rim}",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="near_bearing_ring",
        elem_b="axle",
        reason="The axle is intentionally captured through the bronze bearing ring on the support block.",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="far_bearing_ring",
        elem_b="axle",
        reason="The axle is intentionally captured through the bronze bearing ring on the support block.",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle",
        elem_b="near_bearing_ring",
        contact_tol=0.002,
        name="near bearing supports axle",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle",
        elem_b="far_bearing_ring",
        contact_tol=0.002,
        name="far bearing supports axle",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
    ctx.check(
        "continuous joint rotates about fixed axle center",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(a - b) < 1.0e-6 for a, b in zip(rest_pos, turned_pos)),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
