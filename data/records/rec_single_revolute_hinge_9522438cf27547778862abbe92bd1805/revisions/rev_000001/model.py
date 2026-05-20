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


LEAF_HEIGHT = 0.080
LEAF_REACH = 0.028
LEAF_THICKNESS = 0.0028
LEAF_CORNER_RADIUS = 0.0030
PLATE_BARREL_OVERLAP = 0.0008

BARREL_OUTER_RADIUS = 0.0058
PIN_RADIUS = 0.0026
PIN_HEAD_RADIUS = 0.0038
PIN_HEAD_THICKNESS = 0.0016
PIN_PEEN_RADIUS = 0.0032
PIN_PEEN_THICKNESS = 0.0011
PIN_INTERFACE_OVERLAP = 0.0002

KNUCKLE_LENGTH = 0.0132
KNUCKLE_GAP = 0.0012
KNUCKLE_PITCH = KNUCKLE_LENGTH + KNUCKLE_GAP
GROUND_KNUCKLE_ZS = (-2.0 * KNUCKLE_PITCH, 0.0, 2.0 * KNUCKLE_PITCH)
MOVING_KNUCKLE_ZS = (-1.0 * KNUCKLE_PITCH, 1.0 * KNUCKLE_PITCH)
BARREL_TOTAL_HEIGHT = 5.0 * KNUCKLE_LENGTH + 4.0 * KNUCKLE_GAP
GROUND_BORE_RADIUS = PIN_RADIUS - 0.00005
MOVING_BORE_RADIUS = PIN_RADIUS + 0.00045

SCREW_CLEARANCE_DIA = 0.0045
SCREW_CSK_DIA = 0.0088
SCREW_ZS = (-0.024, 0.0, 0.024)


def _leaf_plate(is_grounded: bool) -> cq.Workplane:
    plate_length = LEAF_REACH + PLATE_BARREL_OVERLAP
    x_center = (
        -(BARREL_OUTER_RADIUS + LEAF_REACH / 2.0) + PLATE_BARREL_OVERLAP / 2.0
        if is_grounded
        else (BARREL_OUTER_RADIUS + LEAF_REACH / 2.0) - PLATE_BARREL_OVERLAP / 2.0
    )

    plate = (
        cq.Workplane("XZ")
        .rect(plate_length, LEAF_HEIGHT)
        .extrude(LEAF_THICKNESS)
        .translate((x_center, -LEAF_THICKNESS / 2.0, 0.0))
        .edges("|Y")
        .fillet(LEAF_CORNER_RADIUS)
    )

    return (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, z_pos) for z_pos in SCREW_ZS])
        .cskHole(SCREW_CLEARANCE_DIA, SCREW_CSK_DIA, 90.0)
    )


def _knuckle(z_center: float, inner_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(BARREL_OUTER_RADIUS)
        .circle(inner_radius)
        .extrude(KNUCKLE_LENGTH)
        .translate((0.0, 0.0, z_center - KNUCKLE_LENGTH / 2.0))
    )


def _grounded_barrel() -> cq.Workplane:
    barrel = _knuckle(GROUND_KNUCKLE_ZS[0], GROUND_BORE_RADIUS)
    for z_pos in GROUND_KNUCKLE_ZS[1:]:
        barrel = barrel.union(_knuckle(z_pos, GROUND_BORE_RADIUS))

    pin_shaft = (
        cq.Workplane("XY")
        .circle(PIN_RADIUS)
        .extrude(BARREL_TOTAL_HEIGHT + 2.0 * PIN_INTERFACE_OVERLAP)
        .translate((0.0, 0.0, -BARREL_TOTAL_HEIGHT / 2.0 - PIN_INTERFACE_OVERLAP))
    )
    pin_head = (
        cq.Workplane("XY")
        .circle(PIN_HEAD_RADIUS)
        .extrude(PIN_HEAD_THICKNESS)
        .translate((0.0, 0.0, BARREL_TOTAL_HEIGHT / 2.0))
    )
    pin_peen = (
        cq.Workplane("XY")
        .circle(PIN_PEEN_RADIUS)
        .extrude(PIN_PEEN_THICKNESS)
        .translate((0.0, 0.0, -BARREL_TOTAL_HEIGHT / 2.0 - PIN_PEEN_THICKNESS))
    )

    return barrel.union(pin_shaft).union(pin_head).union(pin_peen)


def _moving_barrel() -> cq.Workplane:
    barrel = _knuckle(MOVING_KNUCKLE_ZS[0], MOVING_BORE_RADIUS)
    for z_pos in MOVING_KNUCKLE_ZS[1:]:
        barrel = barrel.union(_knuckle(z_pos, MOVING_BORE_RADIUS))
    return barrel


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="barrel_leaf_hinge")

    steel = model.material("steel_leaf", rgba=(0.71, 0.73, 0.76, 1.0))

    grounded_leaf = model.part("grounded_leaf")
    grounded_leaf.visual(
        mesh_from_cadquery(_leaf_plate(True), "grounded_plate"),
        material=steel,
        name="grounded_plate",
    )
    grounded_leaf.visual(
        mesh_from_cadquery(_grounded_barrel(), "grounded_barrel"),
        material=steel,
        name="grounded_barrel",
    )
    grounded_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_REACH + 2.0 * BARREL_OUTER_RADIUS, 2.0 * BARREL_OUTER_RADIUS, LEAF_HEIGHT)),
        mass=0.18,
        origin=Origin(xyz=(-0.011, 0.0, 0.0)),
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        mesh_from_cadquery(_leaf_plate(False), "moving_plate"),
        material=steel,
        name="moving_plate",
    )
    moving_leaf.visual(
        mesh_from_cadquery(_moving_barrel(), "moving_barrel"),
        material=steel,
        name="moving_barrel",
    )
    moving_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_REACH + BARREL_OUTER_RADIUS, 2.0 * BARREL_OUTER_RADIUS, LEAF_HEIGHT)),
        mass=0.15,
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=grounded_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=2.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    grounded_leaf = object_model.get_part("grounded_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("leaf_hinge")
    grounded_plate = grounded_leaf.get_visual("grounded_plate")
    moving_plate = moving_leaf.get_visual("moving_plate")

    limits = hinge.motion_limits

    ctx.check("grounded leaf present", grounded_leaf is not None)
    ctx.check("moving leaf present", moving_leaf is not None)
    ctx.check(
        "hinge axis follows pin axis",
        tuple(float(v) for v in hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "hinge has broad opening travel",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper > 2.0,
        details=f"limits={limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            moving_leaf,
            grounded_leaf,
            axis="x",
            positive_elem=moving_plate,
            negative_elem=grounded_plate,
            min_gap=0.009,
            name="leaf plates sit on opposite sides of the hinge barrel at rest",
        )
        ctx.expect_overlap(
            moving_leaf,
            grounded_leaf,
            axes="z",
            elem_a=moving_plate,
            elem_b=grounded_plate,
            min_overlap=0.075,
            name="leaf plates match in vertical span at rest",
        )
        rest_plate_center = _aabb_center(
            ctx.part_element_world_aabb(moving_leaf, elem="moving_plate")
        )

    open_pose = min(1.35, limits.upper if limits is not None and limits.upper is not None else pi / 2.0)
    with ctx.pose({hinge: open_pose}):
        ctx.expect_overlap(
            moving_leaf,
            grounded_leaf,
            axes="z",
            elem_a=moving_plate,
            elem_b=grounded_plate,
            min_overlap=0.075,
            name="leaf plates keep the same pin height while opening",
        )
        open_plate_center = _aabb_center(
            ctx.part_element_world_aabb(moving_leaf, elem="moving_plate")
        )

    ctx.check(
        "positive hinge rotation swings the moving leaf toward +Y",
        rest_plate_center is not None
        and open_plate_center is not None
        and open_plate_center[1] > rest_plate_center[1] + 0.015,
        details=f"rest_center={rest_plate_center}, open_center={open_plate_center}",
    )
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
