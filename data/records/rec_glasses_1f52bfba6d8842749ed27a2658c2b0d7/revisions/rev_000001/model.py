from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_DEPTH = 0.012
HINGE_X = 0.078
HINGE_Y = -0.006
HINGE_Z = 0.006


def _catmull_rom_closed(points: list[tuple[float, float]], samples: int = 5) -> list[tuple[float, float]]:
    """Small closed-loop smoothing helper for the lens holes."""

    result: list[tuple[float, float]] = []
    count = len(points)
    for i in range(count):
        p0 = points[(i - 1) % count]
        p1 = points[i]
        p2 = points[(i + 1) % count]
        p3 = points[(i + 2) % count]
        for j in range(samples):
            t = j / samples
            t2 = t * t
            t3 = t2 * t
            x = 0.5 * (
                (2.0 * p1[0])
                + (-p0[0] + p2[0]) * t
                + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
                + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
            )
            z = 0.5 * (
                (2.0 * p1[1])
                + (-p0[1] + p2[1]) * t
                + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
                + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
            )
            result.append((x, z))
    return result


def _scaled_profile(
    points: list[tuple[float, float]],
    center: tuple[float, float],
    scale: float,
) -> list[tuple[float, float]]:
    return [
        (center[0] + (x - center[0]) * scale, center[1] + (z - center[1]) * scale)
        for x, z in points
    ]


def _extruded_profile(profile: list[tuple[float, float]], depth: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .polyline(profile)
        .close()
        .extrude(depth)
        .translate((0.0, 0.0, -depth / 2.0))
    )


def _left_lens_hole() -> list[tuple[float, float]]:
    return _catmull_rom_closed(
        [
            (-0.064, 0.011),
            (-0.056, 0.019),
            (-0.020, 0.017),
            (-0.014, 0.008),
            (-0.020, -0.014),
            (-0.033, -0.020),
            (-0.057, -0.018),
            (-0.067, -0.007),
        ],
        samples=5,
    )


def _front_frame_shape() -> cq.Workplane:
    """One-piece thick acetate wayfarer front with two cut-through lens openings."""

    outer = _catmull_rom_closed(
        [
            (-0.078, 0.016),
            (-0.070, 0.030),
            (-0.018, 0.028),
            (-0.006, 0.023),
            (0.006, 0.023),
            (0.018, 0.028),
            (0.070, 0.030),
            (0.078, 0.016),
            (0.069, -0.026),
            (0.028, -0.029),
            (0.015, -0.021),
            (0.000, -0.024),
            (-0.015, -0.021),
            (-0.028, -0.029),
            (-0.069, -0.026),
        ],
        samples=3,
    )
    left_hole = _left_lens_hole()
    right_hole = [(-x, z) for x, z in left_hole]

    frame = _extruded_profile(outer, FRAME_DEPTH)
    for hole in (left_hole, right_hole):
        cutter = _extruded_profile(hole, FRAME_DEPTH * 3.0)
        frame = frame.cut(cutter)

    # Softened front/back edges keep the thick acetate from reading as a flat plate.
    try:
        frame = frame.edges("|Z").fillet(0.0012)
    except Exception:
        pass
    return frame


def _lens_shape(profile: list[tuple[float, float]], center: tuple[float, float]) -> cq.Workplane:
    # Slightly oversize the pane so it disappears under the frame lip like a seated lens.
    return _extruded_profile(_scaled_profile(profile, center, 1.045), 0.0022)


def _temple_shape() -> cq.Workplane:
    """Broad, straight, rounded acetate temple, modeled in the hinge frame."""

    arm = cq.Workplane("XY").box(0.010, 0.112, 0.013).translate((0.0, -0.062, 0.0))
    tip = cq.Workplane("XY").box(0.008, 0.026, 0.010).translate((0.0, -0.128, -0.001))
    front_block = cq.Workplane("XY").box(0.011, 0.013, 0.014).translate((0.0, -0.009, 0.0))
    temple = arm.union(tip).union(front_block)
    try:
        temple = temple.edges().fillet(0.0015)
    except Exception:
        pass
    return temple


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="thick_frame_wayfarer_glasses")

    model.material("gloss_black_acetate", rgba=(0.015, 0.012, 0.010, 1.0))
    model.material("smoky_lens", rgba=(0.09, 0.12, 0.13, 0.42))
    model.material("brushed_steel", rgba=(0.72, 0.70, 0.64, 1.0))

    front_frame = model.part("front_frame")
    front_frame.visual(
        mesh_from_cadquery(_front_frame_shape(), "front_frame"),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="gloss_black_acetate",
        name="acetate_rim",
    )

    left_lens = _left_lens_hole()
    right_lens = [(-x, z) for x, z in left_lens]
    front_frame.visual(
        mesh_from_cadquery(_lens_shape(left_lens, (-0.041, 0.000)), "left_lens"),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="smoky_lens",
        name="left_lens",
    )
    front_frame.visual(
        mesh_from_cadquery(_lens_shape(right_lens, (0.041, 0.000)), "right_lens"),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="smoky_lens",
        name="right_lens",
    )

    # Fixed outer hinge leaves and top/bottom barrels on the rigid frame.
    front_frame.visual(
        Box((0.009, 0.004, 0.017)),
        origin=Origin(xyz=(-0.0735, -0.006, HINGE_Z)),
        material="brushed_steel",
        name="left_hinge_leaf",
    )
    front_frame.visual(
        Cylinder(radius=0.0022, length=0.0050),
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, HINGE_Z + 0.0062)),
        material="brushed_steel",
        name="left_frame_barrel_upper",
    )
    front_frame.visual(
        Cylinder(radius=0.0022, length=0.0050),
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, HINGE_Z - 0.0062)),
        material="brushed_steel",
        name="left_frame_barrel_lower",
    )
    front_frame.visual(
        Cylinder(radius=0.0023, length=0.0015),
        origin=Origin(xyz=(-0.060, 0.0067, 0.018), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="left_front_rivet",
    )
    front_frame.visual(
        Box((0.009, 0.004, 0.017)),
        origin=Origin(xyz=(0.0735, -0.006, HINGE_Z)),
        material="brushed_steel",
        name="right_hinge_leaf",
    )
    front_frame.visual(
        Cylinder(radius=0.0022, length=0.0050),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z + 0.0062)),
        material="brushed_steel",
        name="right_frame_barrel_upper",
    )
    front_frame.visual(
        Cylinder(radius=0.0022, length=0.0050),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z - 0.0062)),
        material="brushed_steel",
        name="right_frame_barrel_lower",
    )
    front_frame.visual(
        Cylinder(radius=0.0023, length=0.0015),
        origin=Origin(xyz=(0.060, 0.0067, 0.018), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="right_front_rivet",
    )

    for name, sx, axis in (
        ("left_temple", -1.0, (0.0, 0.0, 1.0)),
        ("right_temple", 1.0, (0.0, 0.0, -1.0)),
    ):
        temple = model.part(name)
        temple.visual(
            mesh_from_cadquery(_temple_shape(), f"{name}_arm"),
            material="gloss_black_acetate",
            name="straight_temple",
        )
        temple.visual(
            Box((0.008, 0.020, 0.010)),
            origin=Origin(xyz=(0.0, -0.015, 0.0)),
            material="brushed_steel",
            name="temple_hinge_leaf",
        )
        temple.visual(
            Box((0.003, 0.006, 0.006)),
            origin=Origin(xyz=(sx * 0.003, -0.004, 0.0)),
            material="brushed_steel",
            name="barrel_web",
        )
        temple.visual(
            Cylinder(radius=0.0021, length=0.0050),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="brushed_steel",
            name="temple_barrel",
        )

        model.articulation(
            f"front_to_{name}",
            ArticulationType.REVOLUTE,
            parent=front_frame,
            child=temple,
            origin=Origin(xyz=(sx * HINGE_X, HINGE_Y, HINGE_Z)),
            axis=axis,
            motion_limits=MotionLimits(lower=0.0, upper=1.62, effort=0.35, velocity=2.5),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_frame = object_model.get_part("front_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_hinge = object_model.get_articulation("front_to_left_temple")
    right_hinge = object_model.get_articulation("front_to_right_temple")

    ctx.check(
        "rigid front with two folding temples",
        len(object_model.parts) == 3 and len(object_model.articulations) == 2,
        details=f"parts={len(object_model.parts)} articulations={len(object_model.articulations)}",
    )
    ctx.check(
        "both temple joints are vertical revolute hinges",
        left_hinge.articulation_type == ArticulationType.REVOLUTE
        and right_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(left_hinge.axis) == (0.0, 0.0, 1.0)
        and tuple(right_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"left={left_hinge.axis} right={right_hinge.axis}",
    )
    ctx.check(
        "no separate nose-pad hardware parts",
        all("nose" not in part.name and "pad" not in part.name for part in object_model.parts),
        details=", ".join(part.name for part in object_model.parts),
    )
    ctx.expect_overlap(
        front_frame,
        left_temple,
        axes="xy",
        elem_a="left_frame_barrel_upper",
        elem_b="temple_barrel",
        min_overlap=0.002,
        name="left hinge barrels share a common vertical line",
    )
    ctx.expect_overlap(
        front_frame,
        right_temple,
        axes="xy",
        elem_a="right_frame_barrel_upper",
        elem_b="temple_barrel",
        min_overlap=0.002,
        name="right hinge barrels share a common vertical line",
    )

    left_rest = ctx.part_world_aabb(left_temple)
    right_rest = ctx.part_world_aabb(right_temple)
    with ctx.pose({left_hinge: left_hinge.motion_limits.upper, right_hinge: right_hinge.motion_limits.upper}):
        left_folded = ctx.part_world_aabb(left_temple)
        right_folded = ctx.part_world_aabb(right_temple)

    def _aabb_center_x(aabb):
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) * 0.5

    left_rest_x = _aabb_center_x(left_rest)
    right_rest_x = _aabb_center_x(right_rest)
    left_folded_x = _aabb_center_x(left_folded)
    right_folded_x = _aabb_center_x(right_folded)
    ctx.check(
        "temples fold inward toward the frame center",
        left_rest_x is not None
        and right_rest_x is not None
        and left_folded_x is not None
        and right_folded_x is not None
        and left_folded_x > left_rest_x + 0.035
        and right_folded_x < right_rest_x - 0.035,
        details=(
            f"left rest/folded={left_rest_x}/{left_folded_x}, "
            f"right rest/folded={right_rest_x}/{right_folded_x}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
