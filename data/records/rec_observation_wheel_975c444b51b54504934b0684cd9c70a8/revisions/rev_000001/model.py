from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _tube_between(name: str, p0, p1, *, radius: float, radial_segments: int = 16):
    return _mesh(
        name,
        wire_from_points(
            [p0, p1],
            radius=radius,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_fairground_wheel")

    frame_blue = model.material("frame_blue", rgba=(0.05, 0.22, 0.52, 1.0))
    painted_red = model.material("painted_red", rgba=(0.78, 0.07, 0.06, 1.0))
    painted_yellow = model.material("painted_yellow", rgba=(1.0, 0.72, 0.08, 1.0))
    painted_green = model.material("painted_green", rgba=(0.10, 0.58, 0.26, 1.0))
    painted_teal = model.material("painted_teal", rgba=(0.02, 0.54, 0.62, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.66, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.14, 0.15, 0.16, 1.0))
    deck = model.material("painted_deck", rgba=(0.30, 0.32, 0.35, 1.0))

    axle_height = 2.00
    wheel_radius = 1.16
    pivot_radius = 1.04
    side_offset = 0.56
    ring_y = 0.25
    bucket_count = 8

    frame = model.part("frame")
    frame.visual(
        Box((1.85, 1.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=deck,
        name="base_plate",
    )
    frame.visual(
        Box((1.70, 0.12, 0.10)),
        origin=Origin(xyz=(-0.02, -side_offset, 0.11)),
        material=frame_blue,
        name="foot_rail_0",
    )
    frame.visual(
        Box((1.70, 0.12, 0.10)),
        origin=Origin(xyz=(-0.02, side_offset, 0.11)),
        material=frame_blue,
        name="foot_rail_1",
    )

    for tower_index, y in enumerate((-side_offset, side_offset)):
        triangle = wire_from_points(
            [(-0.74, y, 0.10), (0.0, y, 2.90), (0.74, y, 0.10)],
            radius=0.035,
            radial_segments=18,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=0.045,
            corner_segments=8,
        )
        frame.visual(
            _mesh(f"tower_{tower_index}_triangle", triangle),
            material=frame_blue,
            name=f"tower_{tower_index}_triangle",
        )
        frame.visual(
            _tube_between(
                f"tower_{tower_index}_bearing_drop",
                (0.0, y, axle_height + 0.13),
                (0.0, y, 2.90),
                radius=0.032,
            ),
            material=frame_blue,
            name=f"bearing_drop_{tower_index}",
        )
        frame.visual(
            Cylinder(radius=0.13, length=0.14),
            origin=Origin(xyz=(0.0, y, axle_height), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"bearing_{tower_index}",
        )

    frame.visual(
        _tube_between(
            "base_diagonal_0",
            (-0.74, -side_offset, 0.14),
            (0.74, side_offset, 0.14),
            radius=0.020,
        ),
        material=steel,
        name="base_diagonal_0",
    )
    frame.visual(
        _tube_between(
            "base_diagonal_1",
            (0.74, -side_offset, 0.14),
            (-0.74, side_offset, 0.14),
            radius=0.020,
        ),
        material=steel,
        name="base_diagonal_1",
    )

    wheel = model.part("wheel")
    ring_mesh = _mesh(
        "wheel_side_ring",
        TorusGeometry(
            radius=wheel_radius,
            tube=0.025,
            radial_segments=18,
            tubular_segments=96,
        ).rotate_x(math.pi / 2.0),
    )
    wheel.visual(ring_mesh, origin=Origin(xyz=(0.0, -ring_y, 0.0)), material=painted_red, name="side_ring_0")
    wheel.visual(ring_mesh, origin=Origin(xyz=(0.0, ring_y, 0.0)), material=painted_red, name="side_ring_1")
    wheel.visual(
        Cylinder(radius=0.045, length=1.26),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.14, length=0.68),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="central_hub",
    )

    bucket_colors = [
        painted_yellow,
        painted_green,
        painted_teal,
        painted_red,
        painted_yellow,
        painted_green,
        painted_teal,
        painted_red,
    ]
    bucket_parts = []
    pivot_points = []
    for index in range(bucket_count):
        angle = math.pi / 2.0 + (2.0 * math.pi * index / bucket_count)
        x = pivot_radius * math.cos(angle)
        z = pivot_radius * math.sin(angle)
        pivot_points.append((x, z))
        rim_x = wheel_radius * math.cos(angle)
        rim_z = wheel_radius * math.sin(angle)

        wheel.visual(
            Cylinder(radius=0.028, length=0.64),
            origin=Origin(xyz=(x, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"rim_pin_{index}",
        )
        for y in (-ring_y, ring_y):
            wheel.visual(
                _tube_between(
                    f"spoke_{index}_{'neg' if y < 0 else 'pos'}",
                    (0.0, y, 0.0),
                    (rim_x, y, rim_z),
                    radius=0.017,
                    radial_segments=14,
                ),
                material=steel,
                name=f"spoke_{index}_{'neg' if y < 0 else 'pos'}",
            )

        bucket = model.part(f"bucket_{index}")
        body_material = bucket_colors[index]
        bucket.visual(
            Box((0.32, 0.34, 0.040)),
            origin=Origin(xyz=(0.0, 0.0, -0.46)),
            material=body_material,
            name="floor",
        )
        bucket.visual(
            Box((0.040, 0.34, 0.20)),
            origin=Origin(xyz=(-0.18, 0.0, -0.38)),
            material=body_material,
            name="back_wall",
        )
        bucket.visual(
            Box((0.040, 0.34, 0.14)),
            origin=Origin(xyz=(0.18, 0.0, -0.41)),
            material=body_material,
            name="front_lip",
        )
        bucket.visual(
            Box((0.32, 0.040, 0.20)),
            origin=Origin(xyz=(0.0, -0.17, -0.38)),
            material=body_material,
            name="side_wall_0",
        )
        bucket.visual(
            Box((0.32, 0.040, 0.20)),
            origin=Origin(xyz=(0.0, 0.17, -0.38)),
            material=body_material,
            name="side_wall_1",
        )
        bucket.visual(
            _mesh(
                f"bucket_{index}_hanger_yoke",
                wire_from_points(
                    [(0.0, -0.15, -0.33), (0.0, -0.15, 0.0), (0.0, 0.15, 0.0), (0.0, 0.15, -0.33)],
                    radius=0.014,
                    radial_segments=14,
                    cap_ends=True,
                    corner_mode="fillet",
                    corner_radius=0.035,
                    corner_segments=8,
                ),
            ),
            material=dark_steel,
            name="hanger_pivot",
        )
        bucket.visual(
            Box((0.20, 0.018, 0.035)),
            origin=Origin(xyz=(0.0, -0.15, -0.32)),
            material=dark_steel,
            name="hanger_bracket_0",
        )
        bucket.visual(
            Box((0.20, 0.018, 0.035)),
            origin=Origin(xyz=(0.0, 0.15, -0.32)),
            material=dark_steel,
            name="hanger_bracket_1",
        )
        bucket_parts.append(bucket)

    model.articulation(
        "wheel_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.65),
    )

    for index, bucket in enumerate(bucket_parts):
        x, z = pivot_points[index]
        model.articulation(
            f"bucket_pivot_{index}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=bucket,
            origin=Origin(xyz=(x, 0.0, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.2),
            mimic=Mimic(joint="wheel_rotation", multiplier=-1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("wheel_rotation")

    ctx.check(
        "wheel has continuous horizontal axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )

    for bearing_index in range(2):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=f"bearing_{bearing_index}",
            elem_b="axle_shaft",
            reason="The rotating axle is intentionally captured inside the solid proxy bearing housing.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a=f"bearing_{bearing_index}",
            elem_b="axle_shaft",
            min_overlap=0.055,
            name=f"axle retained in bearing {bearing_index}",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="xz",
            elem_a=f"bearing_{bearing_index}",
            elem_b="axle_shaft",
            min_overlap=0.075,
            name=f"axle centered in bearing {bearing_index}",
        )

    for index in range(8):
        bucket = object_model.get_part(f"bucket_{index}")
        pivot = object_model.get_articulation(f"bucket_pivot_{index}")
        ctx.check(
            f"bucket {index} counter-rotates from wheel",
            pivot.mimic is not None
            and pivot.mimic.joint == "wheel_rotation"
            and abs(pivot.mimic.multiplier + 1.0) < 1e-9,
            details=f"mimic={pivot.mimic}",
        )
        ctx.allow_overlap(
            wheel,
            bucket,
            elem_a=f"rim_pin_{index}",
            elem_b="hanger_pivot",
            reason="The bucket hanger pivot is intentionally sleeved around the wheel rim pin.",
        )
        ctx.expect_overlap(
            wheel,
            bucket,
            axes="y",
            elem_a=f"rim_pin_{index}",
            elem_b="hanger_pivot",
            min_overlap=0.25,
            name=f"bucket {index} pivot spans rim pin",
        )
        ctx.expect_overlap(
            wheel,
            bucket,
            axes="xz",
            elem_a=f"rim_pin_{index}",
            elem_b="hanger_pivot",
            min_overlap=0.025,
            name=f"bucket {index} pivot shares pin axis",
        )

    bucket_0 = object_model.get_part("bucket_0")
    rest_pos = ctx.part_world_position(bucket_0)
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        rotated_pos = ctx.part_world_position(bucket_0)
        floor_aabb = ctx.part_element_world_aabb(bucket_0, elem="floor")
        if floor_aabb is None:
            ctx.fail("bucket floor remains horizontal at quarter turn", "floor AABB unavailable")
        else:
            floor_min, floor_max = floor_aabb
            floor_extents = [floor_max[i] - floor_min[i] for i in range(3)]
            ctx.check(
                "bucket floor remains horizontal at quarter turn",
                floor_extents[2] < 0.07 and floor_extents[0] > 0.25,
                details=f"floor_extents={floor_extents}",
            )
    ctx.check(
        "top bucket travels around rim",
        rest_pos is not None
        and rotated_pos is not None
        and rest_pos[2] > 2.95
        and rotated_pos[0] > 0.95
        and abs(rotated_pos[2] - 2.00) < 0.12,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
