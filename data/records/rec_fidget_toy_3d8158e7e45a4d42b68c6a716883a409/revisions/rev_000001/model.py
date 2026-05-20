from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SEGMENT_COUNT = 12
PITCH = 0.026
THICKNESS = 0.0075
BARREL_RADIUS = 0.0028
EAR_LENGTH = 0.0042
CLEVIS_GAP = 0.0052
TONGUE_LENGTH = CLEVIS_GAP
EAR_CENTER_Y = CLEVIS_GAP / 2.0 + EAR_LENGTH / 2.0
LEFT_BRIDGE_LENGTH = 0.008
LEFT_BRIDGE_WIDTH = 0.0036
LEFT_BRIDGE_CENTER_X = 0.0055
BODY_START_X = 0.0072
BODY_MID_X = 0.018
BODY_END_X = 0.021
BODY_HALF_WIDTH_START = 0.0077
BODY_HALF_WIDTH_MID = 0.0063
BODY_HALF_WIDTH_END = 0.0050
NECK_LENGTH = 0.0068
NECK_WIDTH = TONGUE_LENGTH
NECK_CENTER_X = 0.0223
HINGE_LIMIT = math.pi / 2.0


def _segment_body_profile() -> list[tuple[float, float]]:
    return [
        (BODY_START_X, -BODY_HALF_WIDTH_START),
        (BODY_MID_X, -BODY_HALF_WIDTH_MID),
        (BODY_END_X, -BODY_HALF_WIDTH_END),
        (BODY_END_X, BODY_HALF_WIDTH_END),
        (BODY_MID_X, BODY_HALF_WIDTH_MID),
        (BODY_START_X, BODY_HALF_WIDTH_START),
    ]


def _add_segment_geometry(part, *, mesh_name: str, material) -> None:
    body_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(_segment_body_profile(), THICKNESS),
        mesh_name,
    )
    part.visual(body_mesh, material=material, name="body")

    for idx, y in enumerate((-EAR_CENTER_Y, EAR_CENTER_Y)):
        part.visual(
            Box((LEFT_BRIDGE_LENGTH, LEFT_BRIDGE_WIDTH, THICKNESS)),
            origin=Origin(xyz=(LEFT_BRIDGE_CENTER_X, y, 0.0)),
            material=material,
            name=f"ear_bridge_{idx}",
        )
        part.visual(
            Cylinder(radius=BARREL_RADIUS, length=EAR_LENGTH),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=f"ear_barrel_{idx}",
        )

    part.visual(
        Box((NECK_LENGTH, NECK_WIDTH, THICKNESS)),
        origin=Origin(xyz=(NECK_CENTER_X, 0.0, 0.0)),
        material=material,
        name="tongue_neck",
    )
    part.visual(
        Cylinder(radius=BARREL_RADIUS, length=TONGUE_LENGTH),
        origin=Origin(xyz=(PITCH, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="tail_knuckle",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
    return ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wacky_tracks")

    palette = [
        model.material("plastic_red", rgba=(0.91, 0.22, 0.19, 1.0)),
        model.material("plastic_yellow", rgba=(0.97, 0.78, 0.17, 1.0)),
        model.material("plastic_green", rgba=(0.18, 0.70, 0.35, 1.0)),
        model.material("plastic_blue", rgba=(0.18, 0.45, 0.86, 1.0)),
    ]

    segments = []
    for idx in range(SEGMENT_COUNT):
        segment = model.part(f"segment_{idx}")
        _add_segment_geometry(
            segment,
            mesh_name=f"segment_body_{idx}",
            material=palette[idx % len(palette)],
        )
        segments.append(segment)

    for idx, (parent, child) in enumerate(zip(segments, segments[1:])):
        model.articulation(
            f"hinge_{idx}",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=child,
            origin=Origin(xyz=(PITCH, 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.2,
                velocity=10.0,
                lower=-HINGE_LIMIT,
                upper=HINGE_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    segments = [object_model.get_part(f"segment_{idx}") for idx in range(SEGMENT_COUNT)]

    for idx, (parent, child) in enumerate(zip(segments, segments[1:])):
        ctx.expect_origin_gap(
            child,
            parent,
            axis="x",
            min_gap=PITCH - 1e-4,
            max_gap=PITCH + 1e-4,
            name=f"hinge_{idx} keeps one segment pitch",
        )
        ctx.expect_origin_distance(
            child,
            parent,
            axes="yz",
            max_dist=1e-6,
            name=f"hinge_{idx} stays centered transversely",
        )

    hinge_0 = object_model.get_articulation("hinge_0")
    limits = hinge_0.motion_limits
    segment_1 = object_model.get_part("segment_1")
    rest_tail = _aabb_center(ctx.part_element_world_aabb(segment_1, elem="tail_knuckle"))

    if limits is not None and limits.upper is not None and limits.lower is not None:
        with ctx.pose({hinge_0: limits.upper}):
            raised_tail = _aabb_center(ctx.part_element_world_aabb(segment_1, elem="tail_knuckle"))
        with ctx.pose({hinge_0: limits.lower}):
            lowered_tail = _aabb_center(ctx.part_element_world_aabb(segment_1, elem="tail_knuckle"))

        ctx.check(
            "hinge_0 raises the next link at the upper stop",
            rest_tail is not None
            and raised_tail is not None
            and raised_tail[2] > rest_tail[2] + 0.018,
            details=f"rest_tail={rest_tail}, raised_tail={raised_tail}",
        )
        ctx.check(
            "hinge_0 lowers the next link at the lower stop",
            rest_tail is not None
            and lowered_tail is not None
            and lowered_tail[2] < rest_tail[2] - 0.018,
            details=f"rest_tail={rest_tail}, lowered_tail={lowered_tail}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
