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


BODY_L = 0.18
BODY_W = 0.14
BODY_H = 0.11

AXIS_Z = 0.074
NECK_LEN = 0.038
NECK_W = 0.082
NECK_H = 0.060
NECK_BASE_Z = AXIS_Z - NECK_H / 2.0

COLLAR_LEN = 0.008
COLLAR_RAD = 0.042
HOUSING_LEN = 0.028
HOUSING_RAD = 0.034
BORE_RAD = 0.0195
GUSSET_W = 0.050

SHAFT_RAD = 0.017
SHAFT_INSERT = 0.0
EXPOSED_SHAFT_LEN = 0.036
HUB_RAD = 0.026
HUB_START_X = 0.0
HUB_LEN = 0.020

PLATE_RAD = 0.053
PLATE_THICK = 0.012
PLATE_BACK_X = 0.028
BOLT_CIRCLE_R = 0.030
PLATE_HOLE_RAD = 0.0055

INDEX_PIN_RADIUS = 0.0045
INDEX_PIN_LENGTH = 0.006
INDEX_PIN_X = PLATE_BACK_X + PLATE_THICK - 0.001
INDEX_PIN_Y = 0.026

JOINT_X = BODY_L / 2.0 + NECK_LEN + HOUSING_LEN


def _build_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_L, BODY_W, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
        .edges(">Z")
        .fillet(0.004)
    )

    neck = (
        cq.Workplane("XY")
        .box(NECK_LEN, NECK_W, NECK_H)
        .translate((BODY_L / 2.0 + NECK_LEN / 2.0, 0.0, NECK_BASE_Z + NECK_H / 2.0))
    )

    collar = (
        cq.Workplane("YZ")
        .circle(COLLAR_RAD)
        .extrude(COLLAR_LEN)
        .translate((BODY_L / 2.0 + NECK_LEN - COLLAR_LEN, 0.0, AXIS_Z))
    )

    housing = (
        cq.Workplane("YZ")
        .circle(HOUSING_RAD)
        .extrude(HOUSING_LEN)
        .translate((BODY_L / 2.0 + NECK_LEN, 0.0, AXIS_Z))
    )

    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (BODY_L / 2.0 - 0.020, 0.020),
                (BODY_L / 2.0 + 0.006, 0.020),
                (BODY_L / 2.0 + NECK_LEN - 0.006, AXIS_Z - HOUSING_RAD + 0.006),
                (BODY_L / 2.0 + NECK_LEN - 0.006, AXIS_Z - HOUSING_RAD + 0.021),
                (BODY_L / 2.0 - 0.020, 0.046),
            ]
        )
        .close()
        .extrude(GUSSET_W, both=True)
    )

    support = body.union(neck).union(collar).union(housing).union(gusset)

    spindle_bore = (
        cq.Workplane("YZ")
        .circle(BORE_RAD)
        .extrude(NECK_LEN + HOUSING_LEN + COLLAR_LEN + 0.016)
        .translate((BODY_L / 2.0 - 0.004, 0.0, AXIS_Z))
    )

    return support.cut(spindle_bore)


def _build_spindle_stage_shape() -> cq.Workplane:
    shaft = (
        cq.Workplane("YZ")
        .circle(SHAFT_RAD)
        .extrude(SHAFT_INSERT + EXPOSED_SHAFT_LEN)
        .translate((-SHAFT_INSERT, 0.0, 0.0))
    )

    hub = (
        cq.Workplane("YZ")
        .circle(HUB_RAD)
        .extrude(HUB_LEN)
        .translate((HUB_START_X, 0.0, 0.0))
    )

    return shaft.union(hub)


def _build_output_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("YZ")
        .circle(PLATE_RAD)
        .extrude(PLATE_THICK)
        .translate((PLATE_BACK_X, 0.0, 0.0))
    )

    hole_cutters = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (BOLT_CIRCLE_R, 0.0),
                (-BOLT_CIRCLE_R, 0.0),
                (0.0, BOLT_CIRCLE_R),
                (0.0, -BOLT_CIRCLE_R),
            ]
        )
        .circle(PLATE_HOLE_RAD)
        .extrude(PLATE_THICK + 0.002)
        .translate((PLATE_BACK_X - 0.001, 0.0, 0.0))
    )

    return plate.cut(hole_cutters)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="production_roll_module")

    model.material("housing_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("machined_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("plate_aluminum", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("pin_black", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "roll_module_body"),
        material="housing_gray",
        name="body_shell",
    )

    output = model.part("output_stage")
    output.visual(
        mesh_from_cadquery(_build_spindle_stage_shape(), "roll_module_spindle"),
        material="machined_steel",
        name="spindle_stage",
    )
    output.visual(
        mesh_from_cadquery(_build_output_plate_shape(), "roll_module_output_plate"),
        material="plate_aluminum",
        name="output_plate",
    )
    output.visual(
        Cylinder(radius=INDEX_PIN_RADIUS, length=INDEX_PIN_LENGTH),
        origin=Origin(
            xyz=(INDEX_PIN_X, INDEX_PIN_Y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="pin_black",
        name="index_pin",
    )

    model.articulation(
        "body_to_output_stage",
        ArticulationType.REVOLUTE,
        parent=body,
        child=output,
        origin=Origin(xyz=(JOINT_X, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=10.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
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

    body = object_model.get_part("body")
    output = object_model.get_part("output_stage")
    spin_joint = object_model.get_articulation("body_to_output_stage")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body sits on the ground",
        body_aabb is not None and abs(body_aabb[0][2]) <= 0.001,
        details=f"aabb={body_aabb}",
    )

    ctx.expect_within(
        output,
        body,
        axes="yz",
        inner_elem="spindle_stage",
        margin=0.001,
        name="spindle stays centered inside the supported body path",
    )
    ctx.expect_contact(
        output,
        body,
        elem_a="spindle_stage",
        name="spindle stage bears directly on the housing support face",
    )
    ctx.expect_gap(
        output,
        body,
        axis="x",
        positive_elem="output_plate",
        min_gap=0.020,
        max_gap=0.034,
        name="output plate stands proud of the housing nose",
    )

    limits = spin_joint.motion_limits
    axis_ok = spin_joint.axis == (1.0, 0.0, 0.0)
    limits_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
    )
    ctx.check(
        "joint is a shaft-axis revolute",
        axis_ok and limits_ok,
        details=f"axis={spin_joint.axis}, limits={limits}",
    )

    rest_pin = _aabb_center(ctx.part_element_world_aabb(output, elem="index_pin"))
    with ctx.pose({spin_joint: math.pi / 2.0}):
        quarter_turn_pin = _aabb_center(ctx.part_element_world_aabb(output, elem="index_pin"))

    moved_as_rotary = (
        rest_pin is not None
        and quarter_turn_pin is not None
        and abs(quarter_turn_pin[0] - rest_pin[0]) <= 0.002
        and abs(quarter_turn_pin[1] - rest_pin[1]) >= 0.018
        and abs(quarter_turn_pin[2] - rest_pin[2]) >= 0.018
    )
    ctx.check(
        "output stage rotates about the spindle axis",
        moved_as_rotary,
        details=f"rest_pin={rest_pin}, quarter_turn_pin={quarter_turn_pin}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
