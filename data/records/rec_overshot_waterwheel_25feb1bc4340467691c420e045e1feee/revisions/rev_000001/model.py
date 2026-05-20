from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BUCKET_NAMES = (
    "bucket_0",
    "bucket_1",
    "bucket_2",
    "bucket_3",
    "bucket_4",
    "bucket_5",
    "bucket_6",
    "bucket_7",
    "bucket_8",
    "bucket_9",
    "bucket_10",
    "bucket_11",
    "bucket_12",
    "bucket_13",
    "bucket_14",
    "bucket_15",
)

BUCKET_LIP_NAMES = (
    "bucket_lip_0",
    "bucket_lip_1",
    "bucket_lip_2",
    "bucket_lip_3",
    "bucket_lip_4",
    "bucket_lip_5",
    "bucket_lip_6",
    "bucket_lip_7",
    "bucket_lip_8",
    "bucket_lip_9",
    "bucket_lip_10",
    "bucket_lip_11",
    "bucket_lip_12",
    "bucket_lip_13",
    "bucket_lip_14",
    "bucket_lip_15",
)


def _annular_prism_x(
    *,
    outer_radius: float,
    inner_radius: float,
    width: float,
    segments: int = 96,
) -> MeshGeometry:
    """A rectangular-section hoop centered on the X axle."""
    geom = MeshGeometry()
    verts: dict[tuple[int, int, int], int] = {}
    for side, x in enumerate((-width / 2.0, width / 2.0)):
        for radial, radius in enumerate((outer_radius, inner_radius)):
            for i in range(segments):
                theta = 2.0 * math.pi * i / segments
                verts[(side, radial, i)] = geom.add_vertex(
                    x,
                    radius * math.sin(theta),
                    radius * math.cos(theta),
                )

    for i in range(segments):
        j = (i + 1) % segments
        # Outer cylindrical wall.
        geom.add_face(verts[(0, 0, i)], verts[(1, 0, i)], verts[(1, 0, j)])
        geom.add_face(verts[(0, 0, i)], verts[(1, 0, j)], verts[(0, 0, j)])
        # Inner cylindrical wall.
        geom.add_face(verts[(0, 1, i)], verts[(0, 1, j)], verts[(1, 1, j)])
        geom.add_face(verts[(0, 1, i)], verts[(1, 1, j)], verts[(1, 1, i)])
        # Negative-X annular face.
        geom.add_face(verts[(0, 0, i)], verts[(0, 0, j)], verts[(0, 1, j)])
        geom.add_face(verts[(0, 0, i)], verts[(0, 1, j)], verts[(0, 1, i)])
        # Positive-X annular face.
        geom.add_face(verts[(1, 0, i)], verts[(1, 1, i)], verts[(1, 1, j)])
        geom.add_face(verts[(1, 0, i)], verts[(1, 1, j)], verts[(1, 0, j)])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_mill_waterwheel")

    aged_oak = model.material("aged_oak", rgba=(0.50, 0.30, 0.14, 1.0))
    wet_oak = model.material("wet_oak", rgba=(0.34, 0.20, 0.10, 1.0))
    end_grain = model.material("end_grain", rgba=(0.64, 0.43, 0.22, 1.0))
    dark_bushing = model.material("dark_iron_bushing", rgba=(0.05, 0.05, 0.045, 1.0))
    stone = model.material("weathered_stone", rgba=(0.45, 0.43, 0.38, 1.0))
    water = model.material("dark_trough_water", rgba=(0.05, 0.16, 0.21, 0.72))

    wheel_center_z = 1.35

    frame = model.part("mill_frame")

    # Stone sill and timber trough under the lower arc of the wheel.
    frame.visual(
        Box((2.10, 2.75, 0.16)),
        origin=Origin(xyz=(0.0, 0.18, 0.08)),
        material=stone,
        name="stone_sill",
    )
    frame.visual(
        Box((0.88, 1.70, 0.10)),
        origin=Origin(xyz=(0.0, -0.06, 0.21)),
        material=wet_oak,
        name="trough_floor",
    )
    frame.visual(
        Box((0.08, 1.70, 0.30)),
        origin=Origin(xyz=(-0.48, -0.06, 0.38)),
        material=wet_oak,
        name="trough_wall_0",
    )
    frame.visual(
        Box((0.08, 1.70, 0.30)),
        origin=Origin(xyz=(0.48, -0.06, 0.38)),
        material=wet_oak,
        name="trough_wall_1",
    )
    frame.visual(
        Box((0.70, 1.28, 0.020)),
        origin=Origin(xyz=(0.0, -0.08, 0.270)),
        material=water,
        name="trough_water",
    )

    # Two trestle supports carry the solid bearing blocks on either side.
    for side_index, x in enumerate((-0.62, 0.62)):
        frame.visual(
            Box((0.16, 0.16, 1.12)),
            origin=Origin(xyz=(x, -0.28, 0.72)),
            material=aged_oak,
            name=f"trestle_post_{side_index}_0",
        )
        frame.visual(
            Box((0.16, 0.16, 1.12)),
            origin=Origin(xyz=(x, 0.28, 0.72)),
            material=aged_oak,
            name=f"trestle_post_{side_index}_1",
        )
        frame.visual(
            Box((0.26, 0.68, 0.24)),
            origin=Origin(xyz=(x, 0.0, wheel_center_z)),
            material=aged_oak,
            name=f"axle_block_{side_index}",
        )
        frame.visual(
            Box((0.20, 0.78, 0.12)),
            origin=Origin(xyz=(x, 0.0, 1.14)),
            material=aged_oak,
            name=f"support_cap_{side_index}",
        )
        frame.visual(
            mesh_from_geometry(
                _annular_prism_x(outer_radius=0.145, inner_radius=0.086, width=0.028, segments=48),
                f"bearing_ring_{side_index}",
            ),
            origin=Origin(xyz=(x, 0.0, wheel_center_z)),
            material=dark_bushing,
            name=f"bearing_ring_{side_index}",
        )

    # Mill wall framing stands just behind the wheel and trough.
    for x in (-0.88, 0.88):
        frame.visual(
            Box((0.16, 0.16, 1.65)),
            origin=Origin(xyz=(x, 1.25, 0.905)),
            material=aged_oak,
            name=f"mill_post_{0 if x < 0 else 1}",
        )
    frame.visual(
        Box((1.96, 0.16, 0.16)),
        origin=Origin(xyz=(0.0, 1.25, 1.74)),
        material=aged_oak,
        name="mill_top_beam",
    )
    frame.visual(
        Box((1.96, 0.14, 0.14)),
        origin=Origin(xyz=(0.0, 1.25, 0.92)),
        material=aged_oak,
        name="mill_mid_beam",
    )
    # Diagonal braces are deliberately shallow so they read as framing without
    # touching the rotating wheel envelope.
    frame.visual(
        Box((1.18, 0.12, 0.11)),
        origin=Origin(xyz=(-0.40, 1.25, 1.28), rpy=(0.0, -0.62, 0.0)),
        material=aged_oak,
        name="diagonal_brace_0",
    )
    frame.visual(
        Box((1.18, 0.12, 0.11)),
        origin=Origin(xyz=(0.40, 1.25, 1.28), rpy=(0.0, 0.62, 0.0)),
        material=aged_oak,
        name="diagonal_brace_1",
    )

    wheel = model.part("wheel")
    # A through axle and massive hub define the continuous horizontal rotation.
    wheel.visual(
        Cylinder(radius=0.075, length=1.42),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wet_oak,
        name="wheel_axle",
    )
    wheel.visual(
        Cylinder(radius=0.18, length=0.44),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=end_grain,
        name="hub",
    )

    # Deep paired side hoops leave the center open for the scoop buckets.
    side_hoop_mesh = _annular_prism_x(outer_radius=1.05, inner_radius=0.82, width=0.085, segments=128)
    for side_index, x in enumerate((-0.235, 0.235)):
        wheel.visual(
            mesh_from_geometry(side_hoop_mesh.copy(), f"side_hoop_{side_index}"),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=aged_oak,
            name=f"side_hoop_{side_index}",
        )

    # Slender inner keeper hoops make the rim read deep and timber-framed rather
    # than as a single flat disk.
    keeper_mesh = _annular_prism_x(outer_radius=0.84, inner_radius=0.78, width=0.045, segments=96)
    for side_index, x in enumerate((-0.175, 0.175)):
        wheel.visual(
            mesh_from_geometry(keeper_mesh.copy(), f"inner_hoop_{side_index}"),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=wet_oak,
            name=f"inner_hoop_{side_index}",
        )

    spoke_count = 8
    for i in range(spoke_count):
        theta = 2.0 * math.pi * i / spoke_count
        y = 0.50 * math.sin(theta)
        z = 0.50 * math.cos(theta)
        for side_index, x in enumerate((-0.175, 0.175)):
            wheel.visual(
                Box((0.095, 0.070, 0.675)),
                origin=Origin(xyz=(x, y, z), rpy=(-theta, 0.0, 0.0)),
                material=aged_oak,
                name=f"spoke_{i}_{side_index}",
            )

    bucket_count = 16
    for i in range(bucket_count):
        theta = 2.0 * math.pi * i / bucket_count
        y = 0.925 * math.sin(theta)
        z = 0.925 * math.cos(theta)
        # Each bucket is a wide timber plank spanning both side hoops.  It is
        # slightly tilted tangent to the wheel so the evenly spaced paddles read
        # as scoops rather than flat radial slats.
        wheel.visual(
            Box((0.56, 0.255, 0.065)),
            origin=Origin(xyz=(0.0, y, z), rpy=(-theta - 0.18, 0.0, 0.0)),
            material=wet_oak,
            name=BUCKET_NAMES[i],
        )
        lip_y = 1.000 * math.sin(theta)
        lip_z = 1.000 * math.cos(theta)
        wheel.visual(
            Box((0.50, 0.050, 0.105)),
            origin=Origin(xyz=(0.0, lip_y, lip_z), rpy=(-theta - 0.18, 0.0, 0.0)),
            material=end_grain,
            name=BUCKET_LIP_NAMES[i],
        )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("mill_frame")
    wheel = object_model.get_part("wheel")
    joint = object_model.get_articulation("frame_to_wheel")

    ctx.check(
        "wheel uses continuous horizontal axle",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    # The rotating axle is intentionally captured inside simplified solid
    # timber bearing blocks.  Scope the overlap to the shaft/block contacts and
    # prove both block fits with exact retained insertion checks.
    for side_index in (0, 1):
        block_name = f"axle_block_{side_index}"
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=block_name,
            elem_b="wheel_axle",
            reason="The waterwheel shaft is intentionally seated through the solid bearing block proxy.",
        )
        ctx.expect_within(
            wheel,
            frame,
            axes="yz",
            inner_elem="wheel_axle",
            outer_elem=block_name,
            margin=0.0,
            name=f"axle centered in {block_name}",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="x",
            elem_a="wheel_axle",
            elem_b=block_name,
            min_overlap=0.10,
            name=f"axle retained by {block_name}",
        )

    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem="bucket_8",
        negative_elem="trough_floor",
        min_gap=0.02,
        name="lowest bucket clears trough floor",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
        ctx.expect_gap(
            wheel,
            frame,
            axis="z",
            positive_elem="bucket_12",
            negative_elem="trough_floor",
            min_gap=0.02,
            name="rotated bucket clears trough floor",
        )

    ctx.check(
        "continuous spin keeps axle origin fixed",
        rest_pos is not None and turned_pos is not None and rest_pos == turned_pos,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
