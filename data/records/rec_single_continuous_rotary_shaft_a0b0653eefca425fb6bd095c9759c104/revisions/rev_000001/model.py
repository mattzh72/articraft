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


PLATE_THICKNESS = 0.012
PLATE_WIDTH = 0.180
PLATE_HEIGHT = 0.220
PLATE_CORNER_RADIUS = 0.010
MOUNT_HOLE_DIAMETER = 0.012
MOUNT_HOLE_Y = 0.060
MOUNT_HOLE_Z = 0.082

SHAFT_RADIUS = 0.011
SUPPORT_BORE_RADIUS = 0.0155
JOURNAL_BEARING_RADIUS = 0.0162

INNER_SUPPORT_RADIUS = 0.031
INNER_SUPPORT_LENGTH = 0.022

OUTER_SUPPORT_START_X = 0.050
OUTER_SUPPORT_LENGTH = 0.022
OUTER_SUPPORT_WIDTH = 0.070
OUTER_SUPPORT_HEIGHT = 0.078
OUTER_SUPPORT_FILLET = 0.008

RIB_WIDTH = 0.052
RIB_HEIGHT = 0.014
RIB_Z = 0.031

SHAFT_REAR_STUB = 0.015
SHAFT_FORWARD_LENGTH = 0.130
SHAFT_TOTAL_LENGTH = SHAFT_REAR_STUB + SHAFT_FORWARD_LENGTH

HUB_START_X = 0.086
HUB_LENGTH = 0.026
HUB_RADIUS = 0.016

OUTPUT_PLATE_START_X = 0.098
OUTPUT_PLATE_THICKNESS = 0.008
OUTPUT_PLATE_RADIUS = 0.042

NOSE_FLAT_START_X = 0.114
NOSE_FLAT_LENGTH = 0.020


def _make_side_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)
    plate = plate.edges("|X").fillet(PLATE_CORNER_RADIUS)
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-MOUNT_HOLE_Y, -MOUNT_HOLE_Z),
                (-MOUNT_HOLE_Y, MOUNT_HOLE_Z),
                (MOUNT_HOLE_Y, -MOUNT_HOLE_Z),
                (MOUNT_HOLE_Y, MOUNT_HOLE_Z),
            ]
        )
        .hole(MOUNT_HOLE_DIAMETER)
    )

    inner_support = (
        cq.Workplane("YZ", origin=(PLATE_THICKNESS / 2.0, 0.0, 0.0))
        .circle(INNER_SUPPORT_RADIUS)
        .extrude(INNER_SUPPORT_LENGTH)
    )

    outer_support = (
        cq.Workplane("YZ", origin=(OUTER_SUPPORT_START_X, 0.0, 0.0))
        .rect(OUTER_SUPPORT_WIDTH, OUTER_SUPPORT_HEIGHT)
        .extrude(OUTER_SUPPORT_LENGTH)
    )
    outer_support = outer_support.edges("|X").fillet(OUTER_SUPPORT_FILLET)

    rib_length = OUTER_SUPPORT_START_X - PLATE_THICKNESS / 2.0
    top_rib = cq.Workplane("XY").box(rib_length, RIB_WIDTH, RIB_HEIGHT).translate(
        (PLATE_THICKNESS / 2.0 + rib_length / 2.0, 0.0, RIB_Z)
    )
    bottom_rib = cq.Workplane("XY").box(rib_length, RIB_WIDTH, RIB_HEIGHT).translate(
        (PLATE_THICKNESS / 2.0 + rib_length / 2.0, 0.0, -RIB_Z)
    )

    frame = plate.union(inner_support).union(outer_support).union(top_rib).union(bottom_rib)

    bore_length = OUTER_SUPPORT_START_X + OUTER_SUPPORT_LENGTH + PLATE_THICKNESS
    bore = (
        cq.Workplane("YZ", origin=(-PLATE_THICKNESS / 2.0 - 0.002, 0.0, 0.0))
        .circle(SUPPORT_BORE_RADIUS)
        .extrude(bore_length)
    )
    return frame.cut(bore)


def _make_journal_bearing_shape() -> cq.Workplane:
    inner_bearing = (
        cq.Workplane("YZ", origin=(PLATE_THICKNESS / 2.0, 0.0, 0.0))
        .circle(JOURNAL_BEARING_RADIUS)
        .extrude(INNER_SUPPORT_LENGTH)
    )
    outer_bearing = (
        cq.Workplane("YZ", origin=(OUTER_SUPPORT_START_X, 0.0, 0.0))
        .circle(JOURNAL_BEARING_RADIUS)
        .extrude(OUTER_SUPPORT_LENGTH)
    )
    return inner_bearing.union(outer_bearing)


def _make_shaft_core_shape() -> cq.Workplane:
    shaft_core = (
        cq.Workplane("YZ", origin=(-SHAFT_REAR_STUB, 0.0, 0.0))
        .circle(SHAFT_RADIUS)
        .extrude(SHAFT_TOTAL_LENGTH)
    )
    hub = (
        cq.Workplane("YZ", origin=(HUB_START_X, 0.0, 0.0))
        .circle(HUB_RADIUS)
        .extrude(HUB_LENGTH)
    )
    shaft_core = shaft_core.union(hub)

    flat_cutter = cq.Workplane("XY").box(NOSE_FLAT_LENGTH, 0.030, 0.018).translate(
        (
            NOSE_FLAT_START_X + NOSE_FLAT_LENGTH / 2.0,
            0.0,
            SHAFT_RADIUS + 0.006,
        )
    )
    return shaft_core.cut(flat_cutter)


def _make_output_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("YZ", origin=(OUTPUT_PLATE_START_X, 0.0, 0.0))
        .circle(OUTPUT_PLATE_RADIUS)
        .extrude(OUTPUT_PLATE_THICKNESS)
    )

    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (0.024, 0.000),
                (-0.012, 0.021),
                (-0.012, -0.021),
            ]
        )
        .hole(0.010)
    )
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.000, 0.031)])
        .hole(0.006)
    )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_spindle_assembly")

    model.material("painted_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("shaft_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("plate_steel", rgba=(0.80, 0.82, 0.85, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_make_side_plate_shape(), "side_plate_frame"),
        name="frame",
        material="painted_steel",
    )
    side_plate.visual(
        mesh_from_cadquery(_make_journal_bearing_shape(), "side_plate_journal_bearings"),
        name="journal_bearings",
        material="plate_steel",
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(_make_shaft_core_shape(), "spindle_shaft_core"),
        name="shaft_core",
        material="shaft_steel",
    )
    spindle.visual(
        mesh_from_cadquery(_make_output_plate_shape(), "spindle_output_plate"),
        name="output_plate",
        material="plate_steel",
    )

    model.articulation(
        "side_plate_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=side_plate,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
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
    side_plate = object_model.get_part("side_plate")
    spindle = object_model.get_part("spindle")
    spindle_joint = object_model.get_articulation("side_plate_to_spindle")

    ctx.check(
        "spindle is driven by one continuous rotary joint",
        spindle_joint.articulation_type == ArticulationType.CONTINUOUS
        and spindle_joint.motion_limits is not None
        and spindle_joint.motion_limits.lower is None
        and spindle_joint.motion_limits.upper is None,
        details=(
            f"type={spindle_joint.articulation_type}, "
            f"limits={spindle_joint.motion_limits}"
        ),
    )
    ctx.check(
        "spindle joint axis follows the shaft axis",
        spindle_joint.axis is not None
        and abs(spindle_joint.axis[0] - 1.0) < 1e-6
        and abs(spindle_joint.axis[1]) < 1e-6
        and abs(spindle_joint.axis[2]) < 1e-6,
        details=f"axis={spindle_joint.axis}",
    )
    ctx.allow_overlap(
        side_plate,
        spindle,
        elem_a="journal_bearings",
        elem_b="shaft_core",
        reason="Solid journal-bearing proxies intentionally stand in for hidden rolling bearings inside the paired supports.",
    )

    ctx.expect_overlap(
        spindle,
        side_plate,
        axes="x",
        elem_a="shaft_core",
        elem_b="journal_bearings",
        min_overlap=0.030,
        name="shaft spans both journal bearing locations",
    )
    ctx.expect_gap(
        spindle,
        side_plate,
        axis="x",
        positive_elem="output_plate",
        negative_elem="frame",
        min_gap=0.018,
        name="output plate sits ahead of the side wall frame",
    )

    rest_position = ctx.part_world_position(spindle)
    with ctx.pose({spindle_joint: 1.2}):
        spun_position = ctx.part_world_position(spindle)
        ctx.expect_gap(
            spindle,
            side_plate,
            axis="x",
            positive_elem="output_plate",
            negative_elem="frame",
            min_gap=0.018,
            name="output plate remains forward while spinning",
        )

    ctx.check(
        "continuous spin does not translate the spindle",
        rest_position is not None
        and spun_position is not None
        and max(abs(a - b) for a, b in zip(rest_position, spun_position)) < 1e-9,
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
