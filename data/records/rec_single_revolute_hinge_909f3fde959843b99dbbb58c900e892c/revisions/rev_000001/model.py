from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


HINGE_LENGTH = 0.072
FIXED_LEAF_WIDTH = 0.015
MOVING_LEAF_WIDTH = 0.011
LEAF_THICKNESS = 0.0014
BARREL_OUTER_RADIUS = 0.0025
BARREL_INNER_RADIUS = 0.0011
EDGE_BREAK = 0.00045
HOLE_SPACING = 0.036
FIXED_HOLE_DIAMETER = 0.0036
MOVING_HOLE_DIAMETER = 0.0032
END_MARGIN = 0.004
BARREL_GAP = 0.0015
CENTER_KNUCKLE_LENGTH = 0.016

OUTER_KNUCKLE_LENGTH = (
    HINGE_LENGTH - 2.0 * END_MARGIN - 2.0 * BARREL_GAP - CENTER_KNUCKLE_LENGTH
) / 2.0
LEFT_KNUCKLE_START = -HINGE_LENGTH / 2.0 + END_MARGIN
CENTER_KNUCKLE_START = LEFT_KNUCKLE_START + OUTER_KNUCKLE_LENGTH + BARREL_GAP
RIGHT_KNUCKLE_START = CENTER_KNUCKLE_START + CENTER_KNUCKLE_LENGTH + BARREL_GAP


def _leaf_plate(width: float, hole_diameter: float, y_sign: float) -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(HINGE_LENGTH, width, LEAF_THICKNESS, centered=(True, True, False))
        .translate((0.0, y_sign * width / 2.0, 0.0))
        .edges("|Z")
        .fillet(EDGE_BREAK)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rarray(HOLE_SPACING, 1.0, 2, 1)
        .hole(hole_diameter)
    )
    return plate


def _clip_box(
    x_start: float,
    length: float,
    *,
    y_center: float,
    y_size: float,
    z_size: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, y_size, z_size, centered=(True, True, False))
        .translate((x_start + length / 2.0, y_center, 0.0))
    )


def _knuckle_segment(x_start: float, length: float, leaf_side: str) -> cq.Workplane:
    tube = (
        cq.Workplane("YZ")
        .circle(BARREL_OUTER_RADIUS)
        .extrude(length)
        .cut(cq.Workplane("YZ").circle(BARREL_INNER_RADIUS).extrude(length))
        .translate((x_start, 0.0, BARREL_OUTER_RADIUS))
    )

    relief_height = LEAF_THICKNESS + 0.00015
    relief_depth = max(FIXED_LEAF_WIDTH, MOVING_LEAF_WIDTH) + BARREL_OUTER_RADIUS
    if leaf_side == "fixed":
        relief = _clip_box(
            x_start - 0.0001,
            length + 0.0002,
            y_center=relief_depth / 2.0,
            y_size=relief_depth,
            z_size=relief_height,
        )
    else:
        relief = _clip_box(
            x_start - 0.0001,
            length + 0.0002,
            y_center=-relief_depth / 2.0,
            y_size=relief_depth,
            z_size=relief_height,
        )
    return tube.cut(relief)


def _fixed_leaf_shape() -> cq.Workplane:
    body = _leaf_plate(FIXED_LEAF_WIDTH, FIXED_HOLE_DIAMETER, y_sign=-1.0)
    body = body.union(
        _knuckle_segment(CENTER_KNUCKLE_START, CENTER_KNUCKLE_LENGTH, "fixed")
    )
    return body


def _moving_leaf_shape() -> cq.Workplane:
    body = _leaf_plate(MOVING_LEAF_WIDTH, MOVING_HOLE_DIAMETER, y_sign=1.0)
    body = body.union(_knuckle_segment(LEFT_KNUCKLE_START, OUTER_KNUCKLE_LENGTH, "moving"))
    body = body.union(
        _knuckle_segment(RIGHT_KNUCKLE_START, OUTER_KNUCKLE_LENGTH, "moving")
    )
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_lid_hinge")

    model.material("satin_stainless", rgba=(0.71, 0.73, 0.76, 1.0))
    model.material("brushed_steel", rgba=(0.63, 0.66, 0.70, 1.0))

    fixed_leaf = model.part("fixed_leaf")
    fixed_leaf.visual(
        mesh_from_cadquery(_fixed_leaf_shape(), "fixed_leaf_body"),
        material="satin_stainless",
        name="fixed_leaf_body",
    )
    fixed_leaf.inertial = Inertial.from_geometry(
        Box((HINGE_LENGTH, FIXED_LEAF_WIDTH, 2.0 * BARREL_OUTER_RADIUS)),
        mass=0.08,
        origin=Origin(
            xyz=(0.0, -FIXED_LEAF_WIDTH / 2.0, BARREL_OUTER_RADIUS),
        ),
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        mesh_from_cadquery(_moving_leaf_shape(), "moving_leaf_body"),
        material="brushed_steel",
        name="moving_leaf_body",
    )
    moving_leaf.inertial = Inertial.from_geometry(
        Box((HINGE_LENGTH, MOVING_LEAF_WIDTH, 2.0 * BARREL_OUTER_RADIUS)),
        mass=0.06,
        origin=Origin(
            xyz=(0.0, MOVING_LEAF_WIDTH / 2.0, BARREL_OUTER_RADIUS),
        ),
    )

    model.articulation(
        "hinge_pin",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=moving_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=2.15,
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
    hinge_pin = object_model.get_articulation("hinge_pin")

    ctx.check(
        "hinge articulation uses the barrel centerline",
        hinge_pin.axis == (1.0, 0.0, 0.0)
        and hinge_pin.motion_limits is not None
        and hinge_pin.motion_limits.lower == 0.0
        and hinge_pin.motion_limits.upper is not None
        and hinge_pin.motion_limits.upper >= 2.0,
        details=(
            f"axis={hinge_pin.axis}, "
            f"limits={hinge_pin.motion_limits}"
        ),
    )

    ctx.expect_origin_distance(
        fixed_leaf,
        moving_leaf,
        axes="xyz",
        max_dist=1e-6,
        name="leaves share the hinge pin origin",
    )

    rest_aabb = ctx.part_world_aabb(moving_leaf)
    with ctx.pose({hinge_pin: 1.2}):
        opened_aabb = ctx.part_world_aabb(moving_leaf)

    ctx.check(
        "moving leaf lifts when the hinge opens",
        rest_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > rest_aabb[1][2] + 0.006,
        details=f"rest={rest_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
