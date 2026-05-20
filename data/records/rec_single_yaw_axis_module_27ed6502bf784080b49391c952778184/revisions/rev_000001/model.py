from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.240
BASE_WIDTH = 0.160
BASE_THICKNESS = 0.012
BASE_CORNER_RADIUS = 0.006

CHEEK_LENGTH = 0.060
CHEEK_THICKNESS = 0.018
CHEEK_HEIGHT = 0.010
CHEEK_CENTER_X = 0.056

HUB_BODY_RADIUS = 0.024
HUB_BODY_HEIGHT = 0.012
HUB_PILOT_RADIUS = 0.016
HUB_PILOT_HEIGHT = 0.004

TOP_LENGTH = 0.120
TOP_WIDTH = 0.082
TOP_THICKNESS = 0.010
TOP_CORNER_RADIUS = 0.004
TOP_CAP_RADIUS = 0.018
TOP_CAP_HEIGHT = 0.003

MOUNT_HOLE_DIAMETER = 0.010
MOUNT_HOLE_POINTS = (
    (-0.085, -0.052),
    (-0.085, 0.052),
    (0.085, -0.052),
    (0.085, 0.052),
)

TOP_PLATE_HOLE_DIAMETER = 0.008
TOP_PLATE_HOLE_POINTS = (
    (-0.036, -0.022),
    (-0.036, 0.022),
    (0.036, -0.022),
    (0.036, 0.022),
)

YAW_LIMIT = 2.70
YAW_ORIGIN_Z = BASE_THICKNESS + HUB_BODY_HEIGHT + HUB_PILOT_HEIGHT


def _make_cheek(x_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(CHEEK_THICKNESS, CHEEK_LENGTH, CHEEK_HEIGHT, centered=(True, True, False))
        .translate((x_center, 0.0, BASE_THICKNESS))
    )


def _make_base_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(BASE_CORNER_RADIUS)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(MOUNT_HOLE_POINTS)
        .hole(MOUNT_HOLE_DIAMETER)
    )

    hub_body = (
        cq.Workplane("XY")
        .circle(HUB_BODY_RADIUS)
        .extrude(HUB_BODY_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    hub_pilot = (
        cq.Workplane("XY")
        .circle(HUB_PILOT_RADIUS)
        .extrude(HUB_PILOT_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + HUB_BODY_HEIGHT))
    )

    return (
        base_plate.union(_make_cheek(-CHEEK_CENTER_X))
        .union(_make_cheek(CHEEK_CENTER_X))
        .union(hub_body)
        .union(hub_pilot)
    )


def _make_top_plate_shape() -> cq.Workplane:
    top_plate = (
        cq.Workplane("XY")
        .box(TOP_LENGTH, TOP_WIDTH, TOP_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(TOP_CORNER_RADIUS)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(TOP_PLATE_HOLE_POINTS)
        .hole(TOP_PLATE_HOLE_DIAMETER)
    )
    cap = (
        cq.Workplane("XY")
        .circle(TOP_CAP_RADIUS)
        .extrude(TOP_CAP_HEIGHT)
        .translate((0.0, 0.0, TOP_THICKNESS))
    )
    return top_plate.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_yaw_stage")

    model.material("painted_steel", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))

    base = model.part("base_plate")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_plate"),
        material="painted_steel",
        name="base_stage",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, YAW_ORIGIN_Z)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z / 2.0)),
    )

    top_plate = model.part("top_plate")
    top_plate.visual(
        mesh_from_cadquery(_make_top_plate_shape(), "top_plate"),
        material="machined_aluminum",
        name="turntable_top",
    )
    top_plate.inertial = Inertial.from_geometry(
        Box((TOP_LENGTH, TOP_WIDTH, TOP_THICKNESS + TOP_CAP_HEIGHT)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, (TOP_THICKNESS + TOP_CAP_HEIGHT) / 2.0)),
    )

    model.articulation(
        "base_to_top_plate",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
            effort=12.0,
            velocity=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base_plate")
    top_plate = object_model.get_part("top_plate")
    yaw = object_model.get_articulation("base_to_top_plate")

    ctx.expect_origin_distance(
        base,
        top_plate,
        axes="xy",
        min_dist=0.0,
        max_dist=0.001,
        name="top plate stays centered on hub axis",
    )
    ctx.expect_gap(
        top_plate,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        name="top plate seats on the hub without sinking into the base",
    )
    ctx.expect_overlap(
        top_plate,
        base,
        axes="xy",
        min_overlap=0.060,
        name="top plate remains supported within the base footprint",
    )

    limits = yaw.motion_limits
    ctx.check(
        "yaw axis is vertical",
        yaw.axis == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "yaw joint has broad symmetric sweep",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -2.5
        and limits.upper >= 2.5,
        details=f"limits={limits}",
    )

    rest_pos = ctx.part_world_position(top_plate)
    rest_aabb = ctx.part_world_aabb(top_plate)

    with ctx.pose({yaw: pi / 4.0}):
        turned_pos = ctx.part_world_position(top_plate)
        turned_aabb = ctx.part_world_aabb(top_plate)

    def _span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: str) -> float | None:
        if aabb is None:
            return None
        axis_index = "xyz".index(axis)
        return aabb[1][axis_index] - aabb[0][axis_index]

    rest_x = _span(rest_aabb, "x")
    turned_x = _span(turned_aabb, "x")

    ctx.check(
        "top plate rotates in place about the hub",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) <= 1e-6
        and abs(rest_pos[1] - turned_pos[1]) <= 1e-6
        and abs(rest_pos[2] - turned_pos[2]) <= 1e-6,
        details=f"rest_pos={rest_pos}, turned_pos={turned_pos}",
    )
    ctx.check(
        "rectangular top plate changes footprint when yawed",
        rest_x is not None and turned_x is not None and turned_x > rest_x + 0.010,
        details=f"rest_x={rest_x}, turned_x={turned_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
