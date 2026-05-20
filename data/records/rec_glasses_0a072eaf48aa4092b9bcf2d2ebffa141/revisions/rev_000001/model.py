from __future__ import annotations

from math import pi

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
    mesh_from_cadquery,
)
import cadquery as cq


FRAME_THICKNESS = 0.006


def _plate_from_polygon(points: list[tuple[float, float]], thickness: float = FRAME_THICKNESS):
    return cq.Workplane("XY").polyline(points).close().extrude(thickness, both=True)


def _ring_from_polygons(
    outer: list[tuple[float, float]],
    inner: list[tuple[float, float]],
    thickness: float = FRAME_THICKNESS,
):
    outer_solid = _plate_from_polygon(outer, thickness)
    inner_plug = _plate_from_polygon(inner, thickness * 2.2)
    return outer_solid.cut(inner_plug)


def _mirror_x(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(-x, z) for x, z in reversed(points)]


def _scaled_profile(
    points: list[tuple[float, float]],
    center: tuple[float, float],
    scale: float,
) -> list[tuple[float, float]]:
    cx, cz = center
    return [(cx + (x - cx) * scale, cz + (z - cz) * scale) for x, z in points]


def _build_front_frame_mesh():
    """Cat-eye front frame in a local X/Z sketch plane, extruded through local Y."""
    left_outer = [
        (-0.013, -0.013),
        (-0.032, -0.022),
        (-0.058, -0.018),
        (-0.071, -0.005),
        (-0.075, 0.013),
        (-0.061, 0.031),
        (-0.036, 0.026),
        (-0.014, 0.014),
    ]
    left_inner = [
        (-0.019, -0.0085),
        (-0.036, -0.0155),
        (-0.056, -0.0125),
        (-0.064, -0.002),
        (-0.064, 0.009),
        (-0.054, 0.020),
        (-0.036, 0.017),
        (-0.020, 0.0085),
    ]
    right_outer = _mirror_x(left_outer)
    right_inner = _mirror_x(left_inner)

    bridge = [
        (-0.020, 0.011),
        (-0.012, 0.016),
        (0.012, 0.016),
        (0.020, 0.011),
        (0.017, -0.002),
        (0.007, 0.002),
        (0.000, 0.004),
        (-0.007, 0.002),
        (-0.017, -0.002),
    ]

    left_ring = _ring_from_polygons(left_outer, left_inner)
    right_ring = _ring_from_polygons(right_outer, right_inner)
    bridge_solid = _plate_from_polygon(bridge)
    return left_ring.union(right_ring).union(bridge_solid)


def _build_lens_profile(left: bool):
    left_inner = [
        (-0.019, -0.0085),
        (-0.036, -0.0155),
        (-0.056, -0.0125),
        (-0.064, -0.002),
        (-0.064, 0.009),
        (-0.054, 0.020),
        (-0.036, 0.017),
        (-0.020, 0.0085),
    ]
    profile = _scaled_profile(left_inner, (-0.041, 0.002), 1.035)
    if not left:
        profile = _mirror_x(profile)
    return _plate_from_polygon(profile, 0.0014)


def _build_pad_shell():
    # Thin oval silicone pad, modeled as an extruded rounded ellipse.
    return cq.Workplane("XY").ellipse(0.0042, 0.0074).extrude(0.0024, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cat_eye_eyeglasses")

    frame_mat = Material("dark_tortoise_frame", rgba=(0.10, 0.055, 0.030, 1.0))
    amber_mat = Material("amber_brow_highlight", rgba=(0.55, 0.31, 0.10, 1.0))
    metal_mat = Material("warm_silver_hardware", rgba=(0.72, 0.68, 0.58, 1.0))
    lens_mat = Material("smoky_transparent_lens", rgba=(0.62, 0.78, 0.88, 0.34))
    silicone_mat = Material("clear_silicone_pad", rgba=(0.88, 0.94, 0.96, 0.58))

    front = model.part("front_frame")
    front.visual(
        mesh_from_cadquery(_build_front_frame_mesh(), "cat_eye_front_frame", tolerance=0.0006),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="rim_shell",
    )
    front.visual(
        mesh_from_cadquery(_build_lens_profile(True), "left_lens", tolerance=0.0006),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=lens_mat,
        name="left_lens",
    )
    front.visual(
        mesh_from_cadquery(_build_lens_profile(False), "right_lens", tolerance=0.0006),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=lens_mat,
        name="right_lens",
    )

    # Small outer hinge blocks bonded into the rim corners, proud to the rear.
    front.visual(
        Box((0.008, 0.006, 0.019)),
        origin=Origin(xyz=(-0.073, 0.000, 0.006)),
        material=frame_mat,
        name="left_hinge_block",
    )
    front.visual(
        Box((0.008, 0.006, 0.019)),
        origin=Origin(xyz=(0.073, 0.000, 0.006)),
        material=frame_mat,
        name="right_hinge_block",
    )

    # Rigid metal nose-pad arms carried by the front frame.
    for side_name, sx in (("left", -1.0), ("right", 1.0)):
        front.visual(
            Box((0.0020, 0.0020, 0.014)),
            origin=Origin(xyz=(sx * 0.0145, 0.0055, -0.0025)),
            material=metal_mat,
            name=f"{side_name}_pad_stem",
        )
        front.visual(
            Box((0.0068, 0.0032, 0.0024)),
            origin=Origin(xyz=(sx * 0.0122, 0.0076, -0.0095)),
            material=metal_mat,
            name=f"{side_name}_pad_arm",
        )
        front.visual(
            Cylinder(radius=0.00135, length=0.006),
            origin=Origin(xyz=(sx * 0.010, 0.010, -0.0095), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"{side_name}_pad_pivot",
        )

    for side_name, sx, axis_z in (("left", -1.0, -1.0), ("right", 1.0, 1.0)):
        temple = model.part(f"{side_name}_temple")
        temple.visual(
            Cylinder(radius=0.0024, length=0.018),
            origin=Origin(xyz=(0.0, 0.0062, 0.0)),
            material=metal_mat,
            name="hinge_knuckle",
        )
        temple.visual(
            Box((0.0055, 0.114, 0.0042)),
            origin=Origin(xyz=(0.0, 0.064, 0.001)),
            material=frame_mat,
            name="slim_arm",
        )
        temple.visual(
            Box((0.0052, 0.046, 0.0038)),
            origin=Origin(xyz=(0.0, 0.138, -0.010), rpy=(-0.48, 0.0, 0.0)),
            material=frame_mat,
            name="ear_curve",
        )
        model.articulation(
            f"{side_name}_temple_hinge",
            ArticulationType.REVOLUTE,
            parent=front,
            child=temple,
            origin=Origin(xyz=(sx * 0.076, 0.0, 0.006)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=0.35, velocity=3.0, lower=0.0, upper=1.75),
        )

    for side_name, sx in (("left", -1.0), ("right", 1.0)):
        pad = model.part(f"{side_name}_nose_pad")
        pad.visual(
            Cylinder(radius=0.00175, length=0.0062),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=silicone_mat,
            name="pad_eyelet",
        )
        pad.visual(
            Box((0.0025, 0.0018, 0.0055)),
            origin=Origin(xyz=(0.0, 0.0010, -0.0030)),
            material=silicone_mat,
            name="pad_neck",
        )
        pad.visual(
            mesh_from_cadquery(_build_pad_shell(), f"{side_name}_nose_pad_shell", tolerance=0.00035),
            origin=Origin(xyz=(sx * 0.0015, 0.0016, -0.0090), rpy=(pi / 2.0, 0.0, sx * 0.16)),
            material=silicone_mat,
            name="pad_shell",
        )
        model.articulation(
            f"{side_name}_pad_pivot",
            ArticulationType.REVOLUTE,
            parent=front,
            child=pad,
            origin=Origin(xyz=(sx * 0.010, 0.010, -0.0095)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.04, velocity=1.2, lower=-0.26, upper=0.26),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_pad = object_model.get_part("left_nose_pad")
    right_pad = object_model.get_part("right_nose_pad")
    left_temple_hinge = object_model.get_articulation("left_temple_hinge")
    right_temple_hinge = object_model.get_articulation("right_temple_hinge")
    left_pad_pivot = object_model.get_articulation("left_pad_pivot")
    right_pad_pivot = object_model.get_articulation("right_pad_pivot")

    for side_name, pad in (("left", left_pad), ("right", right_pad)):
        ctx.allow_overlap(
            front,
            pad,
            elem_a=f"{side_name}_pad_pivot",
            elem_b="pad_eyelet",
            reason="The soft nose-pad eyelet is intentionally captured around the small metal pivot pin.",
        )
        ctx.expect_overlap(
            pad,
            front,
            axes="xyz",
            elem_a="pad_eyelet",
            elem_b=f"{side_name}_pad_pivot",
            min_overlap=0.001,
            name=f"{side_name} nose-pad eyelet surrounds pivot pin",
        )

    ctx.expect_gap(
        left_temple,
        front,
        axis="y",
        positive_elem="hinge_knuckle",
        negative_elem="left_hinge_block",
        min_gap=0.0,
        max_gap=0.002,
        name="left temple hinge knuckle sits behind block",
    )
    ctx.expect_gap(
        right_temple,
        front,
        axis="y",
        positive_elem="hinge_knuckle",
        negative_elem="right_hinge_block",
        min_gap=0.0,
        max_gap=0.002,
        name="right temple hinge knuckle sits behind block",
    )

    left_rest = ctx.part_world_aabb(left_temple)
    right_rest = ctx.part_world_aabb(right_temple)
    with ctx.pose({left_temple_hinge: 1.25, right_temple_hinge: 1.25}):
        left_folded = ctx.part_world_aabb(left_temple)
        right_folded = ctx.part_world_aabb(right_temple)

    ctx.check(
        "temples fold inward on separate hinges",
        left_rest is not None
        and right_rest is not None
        and left_folded is not None
        and right_folded is not None
        and left_folded[1][0] > left_rest[1][0] + 0.030
        and right_folded[0][0] < right_rest[0][0] - 0.030,
        details=f"left_rest={left_rest}, left_folded={left_folded}, right_rest={right_rest}, right_folded={right_folded}",
    )

    left_pad_rest = ctx.part_world_aabb(left_pad)
    with ctx.pose({left_pad_pivot: 0.24, right_pad_pivot: -0.24}):
        left_pad_rotated = ctx.part_world_aabb(left_pad)
        right_pad_rotated = ctx.part_world_aabb(right_pad)
    ctx.check(
        "nose pads swivel slightly on bracket pivots",
        left_pad_rest is not None
        and left_pad_rotated is not None
        and right_pad_rotated is not None
        and abs(left_pad_rotated[1][1] - left_pad_rest[1][1]) > 0.0015,
        details=f"left_rest={left_pad_rest}, left_rotated={left_pad_rotated}, right_rotated={right_pad_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
