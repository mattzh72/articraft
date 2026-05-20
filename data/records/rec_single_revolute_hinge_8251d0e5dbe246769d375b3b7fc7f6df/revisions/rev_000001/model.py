from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_HEIGHT = 0.120
KNUCKLE_RADIUS = 0.006
PIN_RADIUS = 0.0036
PIN_HEAD_RADIUS = 0.0045
PIN_HEAD_LENGTH = 0.002
KNUCKLE_GAP = 0.0015
WEB_PLATE_OVERLAP = 0.0005
WEB_INNER_CLEARANCE = 0.0040
WEB_THICKNESS = 0.0040

FIXED_LEAF_WIDTH = 0.046
FIXED_LEAF_THICKNESS = 0.0045
MOVING_LEAF_WIDTH = 0.034
MOVING_LEAF_THICKNESS = 0.0040

FIXED_HOLE_DIAMETER = 0.0055
FIXED_CSK_DIAMETER = 0.0105
MOVING_HOLE_DIAMETER = 0.0048
MOVING_CSK_DIAMETER = 0.0095

KNUCKLE_PATTERN = (
    ("fixed", 0.0230),
    ("moving", 0.0225),
    ("fixed", 0.0230),
    ("moving", 0.0225),
    ("fixed", 0.0230),
)


def _knuckle_spans() -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    z_start = -HINGE_HEIGHT / 2.0
    fixed_spans: list[tuple[float, float]] = []
    moving_spans: list[tuple[float, float]] = []

    for index, (owner, length) in enumerate(KNUCKLE_PATTERN):
        span = (z_start, length)
        if owner == "fixed":
            fixed_spans.append(span)
        else:
            moving_spans.append(span)

        z_start += length
        if index < len(KNUCKLE_PATTERN) - 1:
            z_start += KNUCKLE_GAP

    return fixed_spans, moving_spans


def _make_leaf_shape(
    *,
    leaf_width: float,
    leaf_thickness: float,
    leaf_direction: float,
    knuckle_spans: list[tuple[float, float]],
    hole_z_positions: list[float],
    hole_diameter: float,
    csk_diameter: float,
    hollow_knuckles: bool = False,
    include_pin: bool = False,
    include_pin_heads: bool = False,
):
    plate_center_x = leaf_direction * (KNUCKLE_RADIUS + leaf_width / 2.0)
    hole_x = leaf_direction * 0.004

    leaf = (
        cq.Workplane("XY")
        .box(leaf_width, leaf_thickness, HINGE_HEIGHT)
        .translate((plate_center_x, 0.0, 0.0))
    )

    if hole_z_positions:
        leaf = (
            leaf.faces(">Y")
            .workplane(centerOption="CenterOfMass")
            .pushPoints([(hole_x, z_pos) for z_pos in hole_z_positions])
            .cskHole(hole_diameter, csk_diameter, 90.0)
        )

    for z_pos, length in knuckle_spans:
        knuckle = cq.Workplane("XY").circle(KNUCKLE_RADIUS).extrude(length).translate(
            (0.0, 0.0, z_pos)
        )
        if hollow_knuckles:
            knuckle = knuckle.cut(
                cq.Workplane("XY")
                .circle(PIN_RADIUS)
                .extrude(length + 0.0004)
                .translate((0.0, 0.0, z_pos - 0.0002))
            )
        if leaf_direction < 0.0:
            web_x_min = -KNUCKLE_RADIUS - WEB_PLATE_OVERLAP
            web_x_max = -WEB_INNER_CLEARANCE
        else:
            web_x_min = WEB_INNER_CLEARANCE
            web_x_max = KNUCKLE_RADIUS + WEB_PLATE_OVERLAP

        web = (
            cq.Workplane("XY")
            .box(web_x_max - web_x_min, WEB_THICKNESS, length)
            .translate(
                (
                    (web_x_min + web_x_max) / 2.0,
                    0.0,
                    z_pos + length / 2.0,
                )
            )
        )
        segment = knuckle.union(web).combine()
        leaf = leaf.union(segment).combine()

    if include_pin:
        pin = (
            cq.Workplane("XY")
            .circle(PIN_RADIUS)
            .extrude(HINGE_HEIGHT)
            .translate((0.0, 0.0, -HINGE_HEIGHT / 2.0))
        )
        leaf = leaf.union(pin).combine()

    if include_pin_heads:
        top_head = (
            cq.Workplane("XY")
            .circle(PIN_HEAD_RADIUS)
            .extrude(PIN_HEAD_LENGTH)
            .translate((0.0, 0.0, HINGE_HEIGHT / 2.0))
        )
        bottom_head = (
            cq.Workplane("XY")
            .circle(PIN_HEAD_RADIUS)
            .extrude(PIN_HEAD_LENGTH)
            .translate((0.0, 0.0, -HINGE_HEIGHT / 2.0 - PIN_HEAD_LENGTH))
        )
        leaf = leaf.union(top_head).union(bottom_head)

    return leaf


def _make_pin_shape():
    pin = (
        cq.Workplane("XY")
        .circle(PIN_RADIUS)
        .extrude(HINGE_HEIGHT)
        .translate((0.0, 0.0, -HINGE_HEIGHT / 2.0))
    )
    top_head = (
        cq.Workplane("XY")
        .circle(PIN_HEAD_RADIUS)
        .extrude(PIN_HEAD_LENGTH)
        .translate((0.0, 0.0, HINGE_HEIGHT / 2.0))
    )
    bottom_head = (
        cq.Workplane("XY")
        .circle(PIN_HEAD_RADIUS)
        .extrude(PIN_HEAD_LENGTH)
        .translate((0.0, 0.0, -HINGE_HEIGHT / 2.0 - PIN_HEAD_LENGTH))
    )
    return pin.union(top_head).union(bottom_head).combine()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mortised_equipment_hinge")

    model.material("support_leaf_finish", rgba=(0.53, 0.56, 0.60, 1.0))
    model.material("moving_leaf_finish", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("pin_finish", rgba=(0.28, 0.30, 0.32, 1.0))

    fixed_knuckles, moving_knuckles = _knuckle_spans()

    fixed_leaf = model.part("fixed_leaf")
    fixed_leaf.visual(
        mesh_from_cadquery(
            _make_leaf_shape(
                leaf_width=FIXED_LEAF_WIDTH,
                leaf_thickness=FIXED_LEAF_THICKNESS,
                leaf_direction=-1.0,
                knuckle_spans=fixed_knuckles,
                hole_z_positions=[-0.035, 0.0, 0.035],
                hole_diameter=FIXED_HOLE_DIAMETER,
                csk_diameter=FIXED_CSK_DIAMETER,
            ),
            "fixed_leaf_shell",
        ),
        material="support_leaf_finish",
        name="fixed_leaf_shell",
    )
    fixed_leaf.visual(
        mesh_from_cadquery(_make_pin_shape(), "hinge_pin"),
        material="pin_finish",
        name="hinge_pin",
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        mesh_from_cadquery(
            _make_leaf_shape(
                leaf_width=MOVING_LEAF_WIDTH,
                leaf_thickness=MOVING_LEAF_THICKNESS,
                leaf_direction=1.0,
                knuckle_spans=moving_knuckles,
                hole_z_positions=[-0.025, 0.025],
                hole_diameter=MOVING_HOLE_DIAMETER,
                csk_diameter=MOVING_CSK_DIAMETER,
                hollow_knuckles=True,
            ),
            "moving_leaf",
        ),
        material="moving_leaf_finish",
        name="moving_leaf_body",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=0.0,
            upper=1.85,
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
    hinge = object_model.get_articulation("leaf_hinge")
    limits = hinge.motion_limits

    ctx.allow_overlap(
        fixed_leaf,
        moving_leaf,
        elem_a="hinge_pin",
        elem_b="moving_leaf_body",
        reason=(
            "The moving knuckles are modeled as a zero-clearance bearing fit "
            "around the fixed hinge pin."
        ),
    )

    ctx.check(
        "hinge axis follows the pin barrel",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 1.75,
        details=(
            f"type={hinge.articulation_type}, axis={hinge.axis}, "
            f"limits={limits}"
        ),
    )
    ctx.expect_origin_distance(
        fixed_leaf,
        moving_leaf,
        axes="xy",
        max_dist=1e-6,
        name="leaf frames meet at the hinge axis",
    )
    ctx.expect_overlap(
        fixed_leaf,
        moving_leaf,
        axes="z",
        min_overlap=0.118,
        name="leaves share the full barrel height",
    )

    rest_aabb = ctx.part_world_aabb(moving_leaf)
    with ctx.pose({hinge: 1.2}):
        folded_aabb = ctx.part_world_aabb(moving_leaf)

    ctx.check(
        "moving leaf swings away around the barrel",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][1] > rest_aabb[1][1] + 0.020
        and folded_aabb[1][0] < rest_aabb[1][0] - 0.010,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
