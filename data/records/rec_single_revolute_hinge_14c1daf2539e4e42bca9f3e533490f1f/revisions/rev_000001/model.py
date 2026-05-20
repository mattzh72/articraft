from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


LEAF_HEIGHT = 0.078
LEAF_WIDTH = 0.029
PLATE_THICKNESS = 0.0024
CORNER_RADIUS = 0.004
HOLE_DIAMETER = 0.0046
PLATE_SEAM_OFFSET = 0.0028

PIN_RADIUS = 0.0020
PIN_CLEARANCE = 0.0
KNUCKLE_OUTER_RADIUS = 0.0044
KNUCKLE_INNER_RADIUS = PIN_RADIUS + PIN_CLEARANCE

END_MARGIN = 0.0015
KNUCKLE_GAP = 0.0013
OUTER_KNUCKLE_LENGTH = 0.024
CENTER_KNUCKLE_LENGTH = (
    LEAF_HEIGHT
    - 2.0 * END_MARGIN
    - 2.0 * KNUCKLE_GAP
    - 2.0 * OUTER_KNUCKLE_LENGTH
)

TOP_KNUCKLE_Z = LEAF_HEIGHT / 2.0 - END_MARGIN - OUTER_KNUCKLE_LENGTH / 2.0
BOTTOM_KNUCKLE_Z = -TOP_KNUCKLE_Z

PIN_HEAD_RADIUS = 0.0029
PIN_HEAD_THICKNESS = 0.0012
PIN_SHAFT_LENGTH = LEAF_HEIGHT + 2.0 * END_MARGIN

HINGE_OPEN_LIMIT = 1.52


def _leaf_plate(side: float) -> cq.Workplane:
    plate = (
        cq.Workplane("XZ")
        .sketch()
        .rect(LEAF_WIDTH, LEAF_HEIGHT, tag="outer")
        .vertices(tag="outer")
        .fillet(CORNER_RADIUS)
        .finalize()
        .extrude(PLATE_THICKNESS / 2.0, both=True)
        .translate((side * (LEAF_WIDTH / 2.0 + PLATE_SEAM_OFFSET), 0.0, 0.0))
    )

    hole_offsets = (
        -LEAF_HEIGHT * 0.27,
        0.0,
        LEAF_HEIGHT * 0.27,
    )
    return (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, z) for z in hole_offsets])
        .hole(HOLE_DIAMETER)
    )


def _half_knuckle(side: float, length: float, z_center: float) -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .circle(KNUCKLE_OUTER_RADIUS)
        .circle(KNUCKLE_INNER_RADIUS)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    half_space = (
        cq.Workplane("XY")
        .box(
            KNUCKLE_OUTER_RADIUS + 0.0004,
            2.0 * (KNUCKLE_OUTER_RADIUS + 0.0008),
            length,
        )
        .translate((side * (KNUCKLE_OUTER_RADIUS + 0.0004) / 2.0, 0.0, 0.0))
    )
    return ring.intersect(half_space).translate((0.0, 0.0, z_center))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="production_hinge")

    model.material("satin_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("bright_pin", rgba=(0.84, 0.85, 0.88, 1.0))

    fixed_leaf = model.part("fixed_leaf")
    fixed_leaf.visual(
        mesh_from_cadquery(_leaf_plate(-1.0), "fixed_leaf_plate"),
        material="satin_steel",
        name="leaf_plate",
    )
    fixed_leaf.visual(
        mesh_from_cadquery(
            _half_knuckle(-1.0, OUTER_KNUCKLE_LENGTH, TOP_KNUCKLE_Z),
            "fixed_top_knuckle",
        ),
        material="satin_steel",
        name="top_knuckle",
    )
    fixed_leaf.visual(
        mesh_from_cadquery(
            _half_knuckle(-1.0, OUTER_KNUCKLE_LENGTH, BOTTOM_KNUCKLE_Z),
            "fixed_bottom_knuckle",
        ),
        material="satin_steel",
        name="bottom_knuckle",
    )

    pin = model.part("pin")
    pin.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_SHAFT_LENGTH),
        material="bright_pin",
        name="pin_shaft",
    )
    pin.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
        origin=Origin(
            xyz=(0.0, 0.0, PIN_SHAFT_LENGTH / 2.0 - PIN_HEAD_THICKNESS / 2.0)
        ),
        material="bright_pin",
        name="top_head",
    )
    pin.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
        origin=Origin(
            xyz=(0.0, 0.0, -PIN_SHAFT_LENGTH / 2.0 + PIN_HEAD_THICKNESS / 2.0)
        ),
        material="bright_pin",
        name="bottom_head",
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        mesh_from_cadquery(_leaf_plate(1.0), "moving_leaf_plate"),
        material="satin_steel",
        name="leaf_plate",
    )
    moving_leaf.visual(
        mesh_from_cadquery(
            _half_knuckle(1.0, CENTER_KNUCKLE_LENGTH, 0.0),
            "moving_center_knuckle",
        ),
        material="satin_steel",
        name="center_knuckle",
    )

    model.articulation(
        "fixed_leaf_to_pin",
        ArticulationType.FIXED,
        parent=fixed_leaf,
        child=pin,
        origin=Origin(),
    )
    model.articulation(
        "pin_to_moving_leaf",
        ArticulationType.REVOLUTE,
        parent=pin,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=HINGE_OPEN_LIMIT,
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

    fixed_leaf = object_model.get_part("fixed_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    pin = object_model.get_part("pin")
    hinge = object_model.get_articulation("pin_to_moving_leaf")

    limits = hinge.motion_limits
    ctx.check(
        "hinge articulation uses the pin axis",
        hinge.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and isclose(limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and isclose(limits.upper or 0.0, HINGE_OPEN_LIMIT, abs_tol=1e-9),
        details=f"axis={hinge.axis}, limits={limits}",
    )

    ctx.expect_overlap(
        fixed_leaf,
        pin,
        axes="z",
        elem_a="top_knuckle",
        elem_b="pin_shaft",
        min_overlap=OUTER_KNUCKLE_LENGTH - 0.0005,
        name="top knuckle rides on the pin",
    )
    ctx.expect_overlap(
        moving_leaf,
        pin,
        axes="z",
        elem_a="center_knuckle",
        elem_b="pin_shaft",
        min_overlap=CENTER_KNUCKLE_LENGTH - 0.0005,
        name="center knuckle rides on the pin",
    )
    ctx.expect_gap(
        fixed_leaf,
        moving_leaf,
        axis="z",
        positive_elem="top_knuckle",
        negative_elem="center_knuckle",
        min_gap=0.0008,
        max_gap=0.0022,
        name="upper knuckle keeps a realistic barrel gap",
    )

    rest_aabb = ctx.part_element_world_aabb(moving_leaf, elem="leaf_plate")
    with ctx.pose({hinge: HINGE_OPEN_LIMIT}):
        folded_aabb = ctx.part_element_world_aabb(moving_leaf, elem="leaf_plate")

    rest_center = None
    folded_center = None
    if rest_aabb is not None:
        rest_center = tuple(
            (rest_aabb[0][i] + rest_aabb[1][i]) / 2.0 for i in range(3)
        )
    if folded_aabb is not None:
        folded_center = tuple(
            (folded_aabb[0][i] + folded_aabb[1][i]) / 2.0 for i in range(3)
        )

    ctx.check(
        "moving leaf swings around the barrel",
        rest_center is not None
        and folded_center is not None
        and rest_center[0] > 0.010
        and abs(rest_center[1]) < 0.003
        and folded_center[1] > 0.010
        and folded_center[0] < rest_center[0] - 0.010,
        details=f"rest_center={rest_center}, folded_center={folded_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
