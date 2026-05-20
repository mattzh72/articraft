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


PLATE_X = 0.230
PLATE_Y = 0.180
PLATE_T = 0.012
SLOT_LEN = 0.020
SLOT_W = 0.010

HOUSING_X = 0.130
HOUSING_Y = 0.108
HOUSING_H = 0.024
SHOULDER_D = 0.100
SHOULDER_T = 0.008
COLUMN_D = 0.078
COLUMN_H = 0.030

HEAD_FLANGE_D = 0.092
HEAD_FLANGE_T = 0.010
JOINT_Z = PLATE_T + HOUSING_H + COLUMN_H


def _make_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .rect(PLATE_X, PLATE_Y)
        .extrude(PLATE_T)
        .edges("|Z")
        .fillet(0.008)
    )
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.080, -0.060),
                (-0.080, 0.060),
                (0.080, -0.060),
                (0.080, 0.060),
            ]
        )
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .cutThruAll()
    )

    housing = (
        cq.Workplane("XY")
        .rect(HOUSING_X, HOUSING_Y)
        .extrude(HOUSING_H)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, PLATE_T))
    )

    shoulder = (
        cq.Workplane("XY")
        .circle(SHOULDER_D / 2.0)
        .extrude(SHOULDER_T)
        .translate((0.0, 0.0, PLATE_T + HOUSING_H - SHOULDER_T * 0.5))
    )

    column = (
        cq.Workplane("XY")
        .circle(COLUMN_D / 2.0)
        .extrude(COLUMN_H)
        .edges(">Z")
        .fillet(0.003)
        .translate((0.0, 0.0, PLATE_T + HOUSING_H))
    )

    x_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.030, PLATE_T),
                (0.064, PLATE_T),
                (0.030, PLATE_T + HOUSING_H + 0.020),
            ]
        )
        .close()
        .extrude(0.012)
        .translate((0.0, 0.027, 0.0))
    )
    x_rib_neg = x_rib.mirror("YZ")
    x_ribs = x_rib.union(x_rib_neg)
    x_ribs = x_ribs.union(x_ribs.mirror("XZ"))

    y_rib = (
        cq.Workplane("YZ")
        .polyline(
            [
                (0.030, PLATE_T),
                (0.054, PLATE_T),
                (0.030, PLATE_T + HOUSING_H + 0.018),
            ]
        )
        .close()
        .extrude(0.012)
        .translate((0.032, 0.0, 0.0))
    )
    y_rib_neg = y_rib.mirror("YZ")
    y_ribs = y_rib.union(y_rib_neg)
    y_ribs = y_ribs.union(y_ribs.mirror("XZ"))

    return (
        plate.union(housing)
        .union(shoulder)
        .union(column)
        .union(x_ribs)
        .union(y_ribs)
    )


def _make_head_shape() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .circle(HEAD_FLANGE_D / 2.0)
        .extrude(HEAD_FLANGE_T)
        .faces(">Z")
        .edges("%CIRCLE")
        .fillet(0.002)
    )

    neck = (
        cq.Workplane("XY")
        .center(0.016, 0.0)
        .rect(0.036, 0.048)
        .extrude(0.014)
        .translate((0.0, 0.0, HEAD_FLANGE_T))
    )

    cradle = (
        cq.Workplane("XY")
        .center(0.044, 0.0)
        .rect(0.072, 0.052)
        .extrude(0.026)
        .translate((0.0, 0.0, HEAD_FLANGE_T + 0.014))
    )

    groove = (
        cq.Workplane("XZ")
        .center(0.056, HEAD_FLANGE_T + 0.040)
        .circle(0.022)
        .extrude(0.070)
        .translate((0.0, -0.035, 0.0))
    )

    front_relief = (
        cq.Workplane("XY")
        .center(0.072, 0.0)
        .rect(0.020, 0.030)
        .extrude(0.020)
        .translate((0.0, 0.0, HEAD_FLANGE_T + 0.024))
    )

    return flange.union(neck).union(cradle).cut(groove).cut(front_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_axis_rotary_module")

    housing_gray = model.material("housing_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    machined_silver = model.material(
        "machined_silver", rgba=(0.67, 0.69, 0.72, 1.0)
    )

    base_housing = model.part("base_housing")
    base_housing.visual(
        mesh_from_cadquery(_make_base_shape(), "base_housing_shell"),
        material=housing_gray,
        name="housing_shell",
    )

    rotary_head = model.part("rotary_head")
    rotary_head.visual(
        mesh_from_cadquery(_make_head_shape(), "rotary_head_shell"),
        material=machined_silver,
        name="head_shell",
    )

    model.articulation(
        "housing_to_head",
        ArticulationType.REVOLUTE,
        parent=base_housing,
        child=rotary_head,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-2.8,
            upper=2.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_housing = object_model.get_part("base_housing")
    rotary_head = object_model.get_part("rotary_head")
    yaw = object_model.get_articulation("housing_to_head")
    limits = yaw.motion_limits

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "joint_is_vertical_revolute",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and all(abs(a - b) < 1e-9 for a, b in zip(yaw.axis, (0.0, 0.0, 1.0))),
        f"expected a vertical revolute joint, got type={yaw.articulation_type} axis={yaw.axis}",
    )
    ctx.check(
        "joint_has_realistic_yaw_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and limits.upper >= 2.5
        and limits.lower <= -2.5,
        f"expected broad bidirectional yaw range, got limits={limits}",
    )

    ctx.expect_origin_gap(
        rotary_head,
        base_housing,
        axis="z",
        min_gap=0.060,
        max_gap=0.070,
        name="head_sits_above_squat_housing",
    )
    ctx.expect_gap(
        rotary_head,
        base_housing,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="head_seats_on_bearing_face",
    )
    ctx.expect_overlap(
        rotary_head,
        base_housing,
        axes="xy",
        min_overlap=0.050,
        name="head_overlaps_housing_footprint",
    )
    ctx.expect_within(
        rotary_head,
        base_housing,
        axes="xy",
        margin=0.001,
        name="head_stays_within_broad_mounting_plate",
    )

    with ctx.pose({yaw: 1.57}):
        ctx.expect_gap(
            rotary_head,
            base_housing,
            axis="z",
            max_gap=0.002,
            max_penetration=0.0,
            name="head_seats_on_bearing_face_at_turn",
        )
        ctx.expect_within(
            rotary_head,
            base_housing,
            axes="xy",
            margin=0.001,
            name="head_stays_within_mounting_plate_when_turned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
