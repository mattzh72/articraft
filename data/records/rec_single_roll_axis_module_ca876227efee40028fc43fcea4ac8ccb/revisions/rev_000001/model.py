from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def _build_frame_body() -> tuple[object, dict[str, float]]:
    plate_t = 0.016
    plate_w = 0.160
    plate_h = 0.220

    neck_len = 0.028
    neck_w = 0.078
    neck_h = 0.100

    body_rear_x = plate_t + neck_len
    body_len = 0.072
    body_w = 0.122
    body_h = 0.110

    axis_x = body_rear_x + body_len / 2.0
    bearing_journal_r = 0.0235
    through_bore_r = 0.0255

    bolt_hole_r = 0.006
    bolt_y = 0.052
    bolt_z = 0.078

    plate = (
        cq.Workplane("YZ")
        .rect(plate_w, plate_h)
        .extrude(plate_t)
        .edges("|X")
        .fillet(0.010)
    )
    plate = plate.cut(
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-bolt_y, -bolt_z),
                (-bolt_y, bolt_z),
                (bolt_y, -bolt_z),
                (bolt_y, bolt_z),
            ]
        )
        .circle(bolt_hole_r)
        .extrude(plate_t + 0.002)
    )

    neck = (
        cq.Workplane("YZ")
        .workplane(offset=plate_t)
        .rect(neck_w, neck_h)
        .extrude(body_rear_x - plate_t + 0.008)
        .edges("|X")
        .fillet(0.008)
    )

    top_rib = (
        cq.Workplane("YZ")
        .workplane(offset=plate_t)
        .center(0.0, 0.056)
        .rect(0.118, 0.018)
        .extrude(body_rear_x - plate_t + 0.010)
        .edges("|X")
        .fillet(0.004)
    )
    bottom_rib = (
        cq.Workplane("YZ")
        .workplane(offset=plate_t)
        .center(0.0, -0.056)
        .rect(0.118, 0.018)
        .extrude(body_rear_x - plate_t + 0.010)
        .edges("|X")
        .fillet(0.004)
    )

    cartridge = (
        cq.Workplane("YZ")
        .workplane(offset=body_rear_x)
        .rect(body_w, body_h)
        .extrude(body_len)
        .edges("|X")
        .fillet(0.010)
    )
    cartridge = cartridge.cut(
        cq.Workplane("YZ")
        .workplane(offset=body_rear_x - 0.004)
        .circle(through_bore_r)
        .extrude(body_len + 0.008)
    )

    body = plate.union(neck).union(top_rib).union(bottom_rib).union(cartridge)
    body = body.cut(
        cq.Workplane("YZ")
        .workplane(offset=plate_t)
        .circle(through_bore_r)
        .extrude(body_rear_x + body_len - plate_t + 0.002)
    )
    return body, {
        "plate_t": plate_t,
        "body_rear_x": body_rear_x,
        "body_len": body_len,
        "body_w": body_w,
        "axis_x": axis_x,
        "bearing_journal_r": bearing_journal_r,
        "through_bore_r": through_bore_r,
    }


def _build_spindle_core() -> tuple[object, dict[str, float]]:
    shaft_r = 0.018
    shaft_rear = 0.046
    shaft_len = 0.140

    rear_journal_start = -0.046
    rear_journal_len = 0.010
    front_journal_start = 0.026
    front_journal_len = 0.010
    journal_r = 0.0235

    collar_start = 0.052
    collar_len = 0.036
    collar_r = 0.030

    tip_start = collar_start + collar_len
    tip_len = 0.020
    tip_r = 0.018

    shaft = (
        cq.Workplane("YZ")
        .workplane(offset=-shaft_rear)
        .circle(shaft_r)
        .extrude(shaft_len)
    )

    nose = (
        cq.Workplane("YZ")
        .workplane(offset=collar_start)
        .circle(collar_r)
        .extrude(collar_len)
    ).union(
        cq.Workplane("YZ")
        .workplane(offset=tip_start)
        .circle(tip_r)
        .extrude(tip_len)
    )
    nose = nose.edges(">X").chamfer(0.0035)

    return shaft, {
        "nose": nose,
        "collar_r": collar_r,
        "shaft_r": shaft_r,
        "shaft_start": -shaft_rear,
        "shaft_len": shaft_len,
        "rear_journal_start": rear_journal_start,
        "rear_journal_len": rear_journal_len,
        "front_journal_start": front_journal_start,
        "front_journal_len": front_journal_len,
        "journal_r": journal_r,
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_roll_spindle_head")

    frame_paint = model.material("frame_paint", rgba=(0.78, 0.80, 0.82, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.63, 0.65, 0.67, 1.0))
    lug_finish = model.material("lug_finish", rgba=(0.14, 0.14, 0.15, 1.0))

    frame_body_shape, frame_dims = _build_frame_body()
    spindle_shaft_shape, spindle_dims = _build_spindle_core()
    side_bearing_len = 0.042
    side_bearing_r = 0.016
    side_bearing_y = spindle_dims["journal_r"] + side_bearing_len / 2.0
    rear_station_x = (
        frame_dims["axis_x"]
        + spindle_dims["rear_journal_start"]
        + spindle_dims["rear_journal_len"] / 2.0
    )
    front_station_x = (
        frame_dims["axis_x"]
        + spindle_dims["front_journal_start"]
        + spindle_dims["front_journal_len"] / 2.0
    )

    support_frame = model.part("support_frame")
    support_frame.visual(
        mesh_from_cadquery(frame_body_shape, "support_frame_body"),
        material=frame_paint,
        name="frame_body",
    )
    support_frame.visual(
        Cylinder(radius=side_bearing_r, length=side_bearing_len),
        origin=Origin(xyz=(rear_station_x, side_bearing_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="rear_bearing_left",
    )
    support_frame.visual(
        Cylinder(radius=side_bearing_r, length=side_bearing_len),
        origin=Origin(xyz=(rear_station_x, -side_bearing_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="rear_bearing_right",
    )
    support_frame.visual(
        Cylinder(radius=side_bearing_r, length=side_bearing_len),
        origin=Origin(xyz=(front_station_x, side_bearing_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="front_bearing_left",
    )
    support_frame.visual(
        Cylinder(radius=side_bearing_r, length=side_bearing_len),
        origin=Origin(xyz=(front_station_x, -side_bearing_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="front_bearing_right",
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(spindle_shaft_shape, "spindle_shaft_core"),
        material=spindle_steel,
        name="shaft_core",
    )
    spindle.visual(
        Cylinder(
            radius=spindle_dims["journal_r"],
            length=spindle_dims["rear_journal_len"],
        ),
        origin=Origin(
            xyz=(
                spindle_dims["rear_journal_start"] + spindle_dims["rear_journal_len"] / 2.0,
                0.0,
                0.0,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=bearing_steel,
        name="rear_journal",
    )
    spindle.visual(
        Cylinder(
            radius=spindle_dims["journal_r"],
            length=spindle_dims["front_journal_len"],
        ),
        origin=Origin(
            xyz=(
                spindle_dims["front_journal_start"] + spindle_dims["front_journal_len"] / 2.0,
                0.0,
                0.0,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=bearing_steel,
        name="front_journal",
    )
    spindle.visual(
        mesh_from_cadquery(spindle_dims["nose"], "spindle_nose"),
        material=spindle_steel,
        name="nose_body",
    )
    spindle.visual(
        Box((0.022, 0.012, 0.012)),
        origin=Origin(xyz=(0.076, 0.0, spindle_dims["collar_r"] + 0.002)),
        material=lug_finish,
        name="drive_lug",
    )

    model.articulation(
        "spindle_roll",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=spindle,
        origin=Origin(xyz=(frame_dims["axis_x"], 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
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

    support_frame = object_model.get_part("support_frame")
    spindle = object_model.get_part("spindle")
    roll_joint = object_model.get_articulation("spindle_roll")

    ctx.check(
        "spindle roll joint uses the shaft axis",
        roll_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={roll_joint.axis}",
    )

    ctx.expect_contact(
        spindle,
        support_frame,
        elem_a="rear_journal",
        elem_b="rear_bearing_left",
        contact_tol=0.0005,
        name="rear left side bearing supports the rear journal",
    )
    ctx.expect_contact(
        spindle,
        support_frame,
        elem_a="rear_journal",
        elem_b="rear_bearing_right",
        contact_tol=0.0005,
        name="rear right side bearing supports the rear journal",
    )
    ctx.expect_contact(
        spindle,
        support_frame,
        elem_a="front_journal",
        elem_b="front_bearing_left",
        contact_tol=0.0005,
        name="front left side bearing supports the front journal",
    )
    ctx.expect_contact(
        spindle,
        support_frame,
        elem_a="front_journal",
        elem_b="front_bearing_right",
        contact_tol=0.0005,
        name="front right side bearing supports the front journal",
    )
    ctx.expect_gap(
        spindle,
        support_frame,
        axis="x",
        positive_elem="nose_body",
        negative_elem="frame_body",
        min_gap=0.012,
        max_gap=0.040,
        name="spindle nose projects ahead of the short front face",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_lug = aabb_center(ctx.part_element_world_aabb(spindle, elem="drive_lug"))
    with ctx.pose({roll_joint: pi / 2.0}):
        rolled_lug = aabb_center(ctx.part_element_world_aabb(spindle, elem="drive_lug"))

    ctx.check(
        "drive lug visibly rolls with the spindle",
        rest_lug is not None
        and rolled_lug is not None
        and rest_lug[2] > rolled_lug[2] + 0.015
        and rolled_lug[1] < rest_lug[1] - 0.015,
        details=f"rest={rest_lug}, rolled={rolled_lug}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
