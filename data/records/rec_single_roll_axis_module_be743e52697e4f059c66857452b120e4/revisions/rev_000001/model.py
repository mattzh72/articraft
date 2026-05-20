from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.24
BASE_WIDTH = 0.18
BASE_HEIGHT = 0.02

COLUMN_LENGTH = 0.09
COLUMN_WIDTH = 0.10
COLUMN_HEIGHT = 0.20
COLUMN_CENTER_X = -0.03

HEAD_LENGTH = 0.07
HEAD_WIDTH = 0.11
HEAD_HEIGHT = 0.03
HEAD_BOTTOM_Z = 0.185

SPINDLE_AXIS_X = 0.06
SPINDLE_AXIS_Z = 0.255

HOUSING_LENGTH = 0.074
HOUSING_OUTER_RADIUS = 0.052
HOUSING_INNER_RADIUS = 0.034

BRIDGE_LENGTH = 0.09
BRIDGE_WIDTH = 0.08
BRIDGE_HEIGHT = 0.018
BRIDGE_CENTER_X = 0.025
BRIDGE_BOTTOM_Z = SPINDLE_AXIS_Z - HOUSING_OUTER_RADIUS - 0.005

SPINDLE_BODY_RADIUS = 0.030
SPINDLE_BODY_LENGTH = 0.094
OUTPUT_FACE_RADIUS = 0.037
OUTPUT_FACE_LENGTH = 0.010
INDEX_DOT_RADIUS = 0.0035
INDEX_DOT_LENGTH = 0.003
INDEX_DOT_OFFSET = 0.014

SPINDLE_LIMIT_LOWER = -math.pi
SPINDLE_LIMIT_UPPER = math.pi


def _box_on_floor(length: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height, centered=(True, True, False))


def make_pedestal_body() -> cq.Workplane:
    base = _box_on_floor(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT).edges("|Z").fillet(0.01)
    column = (
        _box_on_floor(COLUMN_LENGTH, COLUMN_WIDTH, COLUMN_HEIGHT)
        .translate((COLUMN_CENTER_X, 0.0, BASE_HEIGHT))
        .edges("|Z")
        .fillet(0.008)
    )
    head = (
        _box_on_floor(HEAD_LENGTH, HEAD_WIDTH, HEAD_HEIGHT)
        .translate((0.0, 0.0, HEAD_BOTTOM_Z))
        .edges("|Z")
        .fillet(0.006)
    )
    bridge = (
        _box_on_floor(BRIDGE_LENGTH, BRIDGE_WIDTH, BRIDGE_HEIGHT)
        .translate((BRIDGE_CENTER_X, 0.0, BRIDGE_BOTTOM_Z))
        .edges("|Z")
        .fillet(0.005)
    )
    return base.union(column).union(head).union(bridge)


def make_cartridge_housing() -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .circle(HOUSING_OUTER_RADIUS)
        .extrude(HOUSING_LENGTH)
        .translate((SPINDLE_AXIS_X - HOUSING_LENGTH / 2.0, 0.0, SPINDLE_AXIS_Z))
    )
    inner = (
        cq.Workplane("YZ")
        .circle(HOUSING_INNER_RADIUS)
        .extrude(HOUSING_LENGTH + 0.006)
        .translate((SPINDLE_AXIS_X - (HOUSING_LENGTH + 0.006) / 2.0, 0.0, SPINDLE_AXIS_Z))
    )
    return outer.cut(inner)


def make_spindle_body() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(SPINDLE_BODY_RADIUS)
        .extrude(SPINDLE_BODY_LENGTH)
        .translate((-SPINDLE_BODY_LENGTH / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_roll_module")

    painted_steel = model.material("painted_steel", color=(0.28, 0.30, 0.34))
    machined_steel = model.material("machined_steel", color=(0.74, 0.76, 0.79))
    marker_black = model.material("marker_black", color=(0.10, 0.10, 0.12))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(make_pedestal_body(), "pedestal_body"),
        material=painted_steel,
        name="pedestal_body",
    )
    pedestal.visual(
        mesh_from_cadquery(make_cartridge_housing(), "cartridge_housing"),
        material=machined_steel,
        name="cartridge_housing",
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(make_spindle_body(), "spindle_body"),
        material=machined_steel,
        name="spindle_body",
    )
    spindle.visual(
        Cylinder(radius=OUTPUT_FACE_RADIUS, length=OUTPUT_FACE_LENGTH),
        origin=Origin(
            xyz=(
                HOUSING_LENGTH / 2.0 + OUTPUT_FACE_LENGTH / 2.0,
                0.0,
                0.0,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machined_steel,
        name="output_face",
    )
    spindle.visual(
        Cylinder(radius=INDEX_DOT_RADIUS, length=INDEX_DOT_LENGTH),
        origin=Origin(
            xyz=(
                HOUSING_LENGTH / 2.0 + OUTPUT_FACE_LENGTH - INDEX_DOT_LENGTH / 2.0,
                0.0,
                INDEX_DOT_OFFSET,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=marker_black,
        name="index_dot",
    )

    model.articulation(
        "pedestal_to_spindle",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=spindle,
        origin=Origin(xyz=(SPINDLE_AXIS_X, 0.0, SPINDLE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=6.0,
            lower=SPINDLE_LIMIT_LOWER,
            upper=SPINDLE_LIMIT_UPPER,
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

    pedestal = object_model.get_part("pedestal")
    spindle = object_model.get_part("spindle")
    spindle_joint = object_model.get_articulation("pedestal_to_spindle")

    ctx.check(
        "spindle joint axis is longitudinal",
        tuple(spindle_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spindle_joint.axis}",
    )

    limits = spindle_joint.motion_limits
    ctx.check(
        "spindle joint has symmetric roll limits",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - SPINDLE_LIMIT_LOWER) < 1e-6
        and abs(limits.upper - SPINDLE_LIMIT_UPPER) < 1e-6,
        details=f"limits={limits}",
    )

    ctx.expect_overlap(
        spindle,
        pedestal,
        axes="yz",
        elem_a="spindle_body",
        elem_b="cartridge_housing",
        min_overlap=0.058,
        name="spindle stays coaxially nested in the cartridge housing footprint",
    )
    ctx.expect_contact(
        pedestal,
        spindle,
        elem_a="cartridge_housing",
        elem_b="output_face",
        name="output face seats against the cartridge nose",
    )

    pedestal_housing_aabb = ctx.part_element_world_aabb(pedestal, elem="cartridge_housing")
    output_face_aabb = ctx.part_element_world_aabb(spindle, elem="output_face")
    ctx.check(
        "output face projects beyond the housing nose",
        pedestal_housing_aabb is not None
        and output_face_aabb is not None
        and output_face_aabb[1][0] > pedestal_housing_aabb[1][0] + 0.004,
        details=f"housing={pedestal_housing_aabb}, output_face={output_face_aabb}",
    )

    spindle_origin = ctx.part_world_position(spindle)
    ctx.check(
        "spindle is mounted high on the tower pedestal",
        spindle_origin is not None
        and spindle_origin[2] > 0.22
        and spindle_origin[0] > 0.04,
        details=f"spindle_origin={spindle_origin}",
    )

    def aabb_center(aabb):
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

    rest_dot_aabb = ctx.part_element_world_aabb(spindle, elem="index_dot")
    with ctx.pose({spindle_joint: math.pi / 2.0}):
        rolled_dot_aabb = ctx.part_element_world_aabb(spindle, elem="index_dot")
        rolled_origin = ctx.part_world_position(spindle)

    rest_dot_center = aabb_center(rest_dot_aabb) if rest_dot_aabb is not None else None
    rolled_dot_center = aabb_center(rolled_dot_aabb) if rolled_dot_aabb is not None else None
    ctx.check(
        "index marker follows spindle roll about the x axis",
        spindle_origin is not None
        and rolled_origin is not None
        and rest_dot_center is not None
        and rolled_dot_center is not None
        and abs(rolled_origin[0] - spindle_origin[0]) < 1e-6
        and abs(rolled_origin[1] - spindle_origin[1]) < 1e-6
        and abs(rolled_origin[2] - spindle_origin[2]) < 1e-6
        and rest_dot_center[2] > spindle_origin[2] + 0.010
        and abs(rest_dot_center[1] - spindle_origin[1]) < 0.003
        and rolled_dot_center[1] < spindle_origin[1] - 0.010
        and abs(rolled_dot_center[2] - spindle_origin[2]) < 0.003,
        details=(
            f"spindle_origin={spindle_origin}, rolled_origin={rolled_origin}, "
            f"rest_dot_center={rest_dot_center}, rolled_dot_center={rolled_dot_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
