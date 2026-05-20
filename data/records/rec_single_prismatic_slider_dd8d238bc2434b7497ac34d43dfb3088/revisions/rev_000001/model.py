from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RAIL_LENGTH = 0.360
RAIL_WIDTH = 0.026
BASE_HEIGHT = 0.016
GUIDE_LENGTH = 0.360
GUIDE_WIDTH = 0.026
GUIDE_HEIGHT = 0.016

CARRIAGE_LENGTH = 0.090
CARRIAGE_WIDTH = 0.072
CARRIAGE_HEIGHT = 0.032
CHANNEL_WIDTH = 0.026
CHANNEL_DEPTH = 0.016
CARRIAGE_CAVITY_WIDTH = 0.028
TOP_BRIDGE_THICKNESS = 0.014
PAD_LENGTH = 0.060
PAD_WIDTH = 0.007
PAD_THICKNESS = 0.002

SIDE_JAW_WIDTH = (CARRIAGE_WIDTH - CARRIAGE_CAVITY_WIDTH) / 2.0
TOP_BRIDGE_Z = CARRIAGE_HEIGHT - TOP_BRIDGE_THICKNESS
PAD_Y_OFFSET = 0.006

PLATE_LENGTH = 0.120
PLATE_WIDTH = 0.090
PLATE_THICKNESS = 0.008

STOP_LENGTH = 0.014
STOP_WIDTH = 0.040
STOP_HEIGHT = 0.024
STOP_INSET = 0.004
END_CLEARANCE = 0.001

STOP_CENTER_X = (RAIL_LENGTH / 2.0) + (STOP_LENGTH / 2.0)
SLIDE_TRAVEL = (RAIL_LENGTH / 2.0) - (CARRIAGE_LENGTH / 2.0) - END_CLEARANCE


def _make_rail_body() -> cq.Workplane:
    rail = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.120, 0.0),
                (-0.040, 0.0),
                (0.040, 0.0),
                (0.120, 0.0),
            ]
        )
        .hole(0.006)
    )
    return rail.edges("|Z").fillet(0.0015)


def _make_carriage() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_WIDTH,
        CARRIAGE_HEIGHT,
        centered=(True, True, False),
    )
    channel = cq.Workplane("XY").box(
        CARRIAGE_LENGTH + 0.004,
        CHANNEL_WIDTH,
        CHANNEL_DEPTH,
        centered=(True, True, False),
    )
    return carriage.cut(channel)


def _make_tooling_plate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS, centered=(True, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.040, -0.025),
                (-0.040, 0.025),
                (0.040, -0.025),
                (0.040, 0.025),
            ]
        )
        .hole(0.006)
    )


def _make_end_stop() -> cq.Workplane:
    return cq.Workplane("XY").box(
        STOP_LENGTH,
        STOP_WIDTH,
        STOP_HEIGHT,
        centered=(True, True, False),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_slide_module")

    rail_mat = model.material("rail_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    carriage_mat = model.material("carriage_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    plate_mat = model.material("tooling_plate_blue", rgba=(0.28, 0.40, 0.72, 1.0))
    stop_mat = model.material("stop_black", rgba=(0.10, 0.10, 0.10, 1.0))

    rail_body = model.part("rail_body")
    rail_body.visual(
        mesh_from_cadquery(_make_rail_body(), "rail_body"),
        origin=Origin(),
        material=rail_mat,
        name="rail_shell",
    )

    left_stop = model.part("left_stop")
    left_stop.visual(
        mesh_from_cadquery(_make_end_stop(), "left_stop"),
        origin=Origin(),
        material=stop_mat,
        name="left_stop_shell",
    )

    right_stop = model.part("right_stop")
    right_stop.visual(
        mesh_from_cadquery(_make_end_stop(), "right_stop"),
        origin=Origin(),
        material=stop_mat,
        name="right_stop_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, TOP_BRIDGE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TOP_BRIDGE_Z + (TOP_BRIDGE_THICKNESS / 2.0))),
        material=carriage_mat,
        name="carriage_top_bridge",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, SIDE_JAW_WIDTH, CARRIAGE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (CARRIAGE_CAVITY_WIDTH / 2.0) + (SIDE_JAW_WIDTH / 2.0),
                CARRIAGE_HEIGHT / 2.0,
            )
        ),
        material=carriage_mat,
        name="carriage_left_jaw",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, SIDE_JAW_WIDTH, CARRIAGE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -((CARRIAGE_CAVITY_WIDTH / 2.0) + (SIDE_JAW_WIDTH / 2.0)),
                CARRIAGE_HEIGHT / 2.0,
            )
        ),
        material=carriage_mat,
        name="carriage_right_jaw",
    )
    carriage.visual(
        Box((PAD_LENGTH, PAD_WIDTH, PAD_THICKNESS)),
        origin=Origin(
            xyz=(0.0, PAD_Y_OFFSET, TOP_BRIDGE_Z - (PAD_THICKNESS / 2.0))
        ),
        material=carriage_mat,
        name="carriage_left_pad",
    )
    carriage.visual(
        Box((PAD_LENGTH, PAD_WIDTH, PAD_THICKNESS)),
        origin=Origin(
            xyz=(0.0, -PAD_Y_OFFSET, TOP_BRIDGE_Z - (PAD_THICKNESS / 2.0))
        ),
        material=carriage_mat,
        name="carriage_right_pad",
    )

    tooling_plate = model.part("tooling_plate")
    tooling_plate.visual(
        mesh_from_cadquery(_make_tooling_plate(), "tooling_plate"),
        origin=Origin(),
        material=plate_mat,
        name="tooling_plate_shell",
    )

    model.articulation(
        "rail_to_left_stop",
        ArticulationType.FIXED,
        parent=rail_body,
        child=left_stop,
        origin=Origin(xyz=(-STOP_CENTER_X, 0.0, 0.0)),
    )
    model.articulation(
        "rail_to_right_stop",
        ArticulationType.FIXED,
        parent=rail_body,
        child=right_stop,
        origin=Origin(xyz=(STOP_CENTER_X, 0.0, 0.0)),
    )
    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_body,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.40,
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_tooling_plate",
        ArticulationType.FIXED,
        parent=carriage,
        child=tooling_plate,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_body = object_model.get_part("rail_body")
    left_stop = object_model.get_part("left_stop")
    right_stop = object_model.get_part("right_stop")
    carriage = object_model.get_part("carriage")
    tooling_plate = object_model.get_part("tooling_plate")
    slide = object_model.get_articulation("rail_to_carriage")

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
        "all_parts_present",
        all(
            part is not None
            for part in (rail_body, left_stop, right_stop, carriage, tooling_plate)
        ),
        "Expected rail body, two end stops, carriage, and tooling plate.",
    )
    ctx.check(
        "slide_joint_is_prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        f"Expected PRISMATIC articulation, got {slide.articulation_type}.",
    )
    ctx.check(
        "slide_joint_axis_is_x",
        tuple(round(v, 6) for v in slide.axis) == (1.0, 0.0, 0.0),
        f"Expected slide axis (1, 0, 0), got {slide.axis}.",
    )
    ctx.check(
        "slide_joint_limits_span_both_directions",
        slide.motion_limits is not None
        and slide.motion_limits.lower is not None
        and slide.motion_limits.upper is not None
        and slide.motion_limits.lower < 0.0 < slide.motion_limits.upper,
        "Expected symmetric bidirectional prismatic travel around the centered pose.",
    )

    ctx.expect_contact(left_stop, rail_body, name="left_stop_is_mounted")
    ctx.expect_contact(right_stop, rail_body, name="right_stop_is_mounted")
    ctx.expect_contact(carriage, rail_body, name="carriage_is_supported_on_rail")
    ctx.expect_contact(
        tooling_plate,
        carriage,
        name="tooling_plate_is_mounted_to_carriage",
    )

    ctx.expect_overlap(
        carriage,
        rail_body,
        axes="xy",
        min_overlap=0.020,
        name="carriage_overlaps_rail_footprint",
    )
    ctx.expect_overlap(
        tooling_plate,
        carriage,
        axes="xy",
        min_overlap=0.060,
        name="tooling_plate_sits_over_carriage",
    )

    lower = slide.motion_limits.lower
    upper = slide.motion_limits.upper

    with ctx.pose({slide: lower}):
        ctx.expect_gap(
            carriage,
            left_stop,
            axis="x",
            min_gap=0.0005,
            max_gap=0.0020,
            name="left_end_stop_clearance_at_lower_limit",
        )

    with ctx.pose({slide: upper}):
        ctx.expect_gap(
            right_stop,
            carriage,
            axis="x",
            min_gap=0.0005,
            max_gap=0.0020,
            name="right_end_stop_clearance_at_upper_limit",
        )

    with ctx.pose({slide: lower}):
        lower_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: upper}):
        upper_pos = ctx.part_world_position(carriage)

    if lower_pos is not None and upper_pos is not None:
        expected_shift = upper - lower
        measured_shift = upper_pos[0] - lower_pos[0]
        yz_drift = max(abs(upper_pos[1] - lower_pos[1]), abs(upper_pos[2] - lower_pos[2]))
        ctx.check(
            "carriage_motion_tracks_slide_axis",
            abs(measured_shift - expected_shift) <= 1e-6 and yz_drift <= 1e-6,
            (
                f"Expected x shift {expected_shift:.6f} m with negligible y/z drift; "
                f"measured shift {measured_shift:.6f} m and drift {yz_drift:.6f} m."
            ),
        )
    else:
        ctx.fail(
            "carriage_motion_tracks_slide_axis",
            "Could not resolve carriage world positions at slide limits.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
