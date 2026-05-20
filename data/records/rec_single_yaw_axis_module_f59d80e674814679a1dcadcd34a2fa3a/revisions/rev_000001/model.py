from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_W = 0.280
BASE_D = 0.200
BASE_T = 0.016

ARM_T = 0.022
ARM_D = 0.160
ARM_H = 0.096
ARM_INNER_SPAN = 0.126
ARM_X = ARM_INNER_SPAN / 2.0 + ARM_T / 2.0

COLLAR_OUTER_R = 0.028
COLLAR_INNER_R = 0.0215
COLLAR_H = 0.034

PLATE_W = 0.106
PLATE_D = 0.082
PLATE_T = 0.010
PLATE_CORNER_R = 0.007

PLATE_UNDERSIDE_Z = BASE_T + COLLAR_H

PAN_LIMIT = 2.80


def _fork_arm_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(ARM_T, ARM_D, ARM_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.009)
        .faces(">Z")
        .edges()
        .fillet(0.004)
    )


def _hub_collar_shape() -> cq.Workplane:
    lower_boss = cq.Workplane("XY").circle(COLLAR_OUTER_R + 0.005).extrude(0.012)
    center_barrel = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .circle(COLLAR_OUTER_R - 0.004)
        .extrude(COLLAR_H - 0.016)
    )
    top_flange = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_H - 0.004)
        .circle(COLLAR_OUTER_R)
        .extrude(0.004)
    )
    return (
        lower_boss.union(center_barrel)
        .union(top_flange)
        .faces(">Z")
        .edges()
        .fillet(0.0015)
    )


def _pan_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_W, PLATE_D, PLATE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(PLATE_CORNER_R)
    )
    return (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.026, 0.0), (0.026, 0.0)])
        .slot2D(0.022, 0.006, 90)
        .cutThruAll()
    )
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_supported_pan_plate")

    model.material("powder_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("bearing_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("plate_gray", rgba=(0.78, 0.79, 0.82, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_W, BASE_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="powder_black",
        name="base_plate",
    )
    frame.visual(
        mesh_from_cadquery(_fork_arm_shape(), "left_fork_arm"),
        origin=Origin(xyz=(-ARM_X, 0.0, BASE_T)),
        material="powder_black",
        name="left_arm",
    )
    frame.visual(
        mesh_from_cadquery(_fork_arm_shape(), "right_fork_arm"),
        origin=Origin(xyz=(ARM_X, 0.0, BASE_T)),
        material="powder_black",
        name="right_arm",
    )
    frame.visual(
        mesh_from_cadquery(_hub_collar_shape(), "hub_collar"),
        origin=Origin(xyz=(0.0, 0.0, BASE_T)),
        material="bearing_steel",
        name="hub_pedestal",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_T + ARM_H)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + ARM_H) / 2.0)),
    )

    pan_plate = model.part("pan_plate")
    pan_plate.visual(
        mesh_from_cadquery(_pan_plate_shape(), "pan_plate_body"),
        material="plate_gray",
        name="pan_plate",
    )
    pan_plate.inertial = Inertial.from_geometry(
        Box((PLATE_W, PLATE_D, PLATE_T)),
        mass=0.78,
        origin=Origin(xyz=(0.0, 0.0, PLATE_T / 2.0)),
    )

    model.articulation(
        "frame_to_pan_plate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=pan_plate,
        origin=Origin(xyz=(0.0, 0.0, PLATE_UNDERSIDE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-PAN_LIMIT,
            upper=PAN_LIMIT,
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

    frame = object_model.get_part("frame")
    pan_plate = object_model.get_part("pan_plate")
    pan_joint = object_model.get_articulation("frame_to_pan_plate")

    ctx.check("frame part present", frame is not None)
    ctx.check("pan plate part present", pan_plate is not None)
    ctx.check(
        "pan axis is vertical",
        tuple(round(v, 6) for v in pan_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pan_joint.axis}",
    )

    limits = pan_joint.motion_limits
    ctx.check(
        "pan joint has realistic bidirectional travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < -1.5
        and limits.upper > 1.5,
        details=f"limits={limits}",
    )

    with ctx.pose({pan_joint: 0.0}):
        ctx.expect_origin_distance(
            pan_plate,
            frame,
            axes="xy",
            max_dist=1e-6,
            name="plate stays centered on the fork hub axis",
        )
        ctx.expect_gap(
            pan_plate,
            frame,
            axis="x",
            positive_elem="pan_plate",
            negative_elem="left_arm",
            min_gap=0.008,
            max_gap=0.013,
            name="plate clears the left fork arm",
        )
        ctx.expect_gap(
            frame,
            pan_plate,
            axis="x",
            positive_elem="right_arm",
            negative_elem="pan_plate",
            min_gap=0.008,
            max_gap=0.013,
            name="plate clears the right fork arm",
        )
        ctx.expect_gap(
            pan_plate,
            frame,
            axis="z",
            positive_elem="pan_plate",
            negative_elem="base_plate",
            min_gap=0.033,
            max_gap=0.035,
            name="plate sits above the grounded base",
        )
        ctx.expect_contact(
            frame,
            pan_plate,
            elem_a="hub_pedestal",
            elem_b="pan_plate",
            contact_tol=1e-6,
            name="plate is supported on the short hub pedestal",
        )
        ctx.expect_overlap(
            frame,
            pan_plate,
            axes="xy",
            elem_a="hub_pedestal",
            elem_b="pan_plate",
            min_overlap=0.050,
            name="hub pedestal stays centered under the plate footprint",
        )

    with ctx.pose({pan_joint: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(pan_plate, elem="pan_plate")
        rest_pos = ctx.part_world_position(pan_plate)
    with ctx.pose({pan_joint: math.pi / 2.0}):
        quarter_aabb = ctx.part_element_world_aabb(pan_plate, elem="pan_plate")
        quarter_pos = ctx.part_world_position(pan_plate)

    def _span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis] - aabb[0][axis]

    rest_dx = _span(rest_aabb, 0)
    rest_dy = _span(rest_aabb, 1)
    quarter_dx = _span(quarter_aabb, 0)
    quarter_dy = _span(quarter_aabb, 1)

    ctx.check(
        "plate rotates about the vertical hub axis",
        rest_dx is not None
        and rest_dy is not None
        and quarter_dx is not None
        and quarter_dy is not None
        and rest_dx > rest_dy
        and quarter_dy > quarter_dx,
        details=(
            f"rest_dx={rest_dx}, rest_dy={rest_dy}, "
            f"quarter_dx={quarter_dx}, quarter_dy={quarter_dy}"
        ),
    )
    ctx.check(
        "plate origin remains fixed while panning",
        rest_pos is not None
        and quarter_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_pos, quarter_pos)),
        details=f"rest_pos={rest_pos}, quarter_pos={quarter_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
