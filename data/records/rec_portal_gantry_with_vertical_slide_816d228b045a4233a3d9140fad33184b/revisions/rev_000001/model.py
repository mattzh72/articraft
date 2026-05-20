from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

import cadquery as cq


BEAM_LENGTH = 1.42
BEAM_DEPTH = 0.22
BEAM_HEIGHT = 0.18
BEAM_FRONT_LAND_DEPTH = 0.012
RAIL_LENGTH = 1.08
RAIL_DEPTH = 0.014
RAIL_HEIGHT = 0.018
RAIL_Z_OFFSET = 0.052
FRAME_JOINT_X = 0.60

RIDER_LENGTH = 0.24
RIDER_DEPTH = 0.15
RIDER_HEIGHT = 0.27
RIDER_REAR_INTERFACE_Y = 0.0
RIDER_BODY_CENTER_Y = 0.089
RIDER_POCKET_BACK_Y = 0.079
RIDER_FRONT_Y = RIDER_BODY_CENTER_Y + (RIDER_DEPTH / 2.0)

X_TRAVEL_LOWER = -0.41
X_TRAVEL_UPPER = 0.41

Z_TRAVEL_LOWER = 0.0
Z_TRAVEL_UPPER = 0.42

BEAM_TO_RIDER_Y = (BEAM_DEPTH / 2.0) + BEAM_FRONT_LAND_DEPTH + RAIL_DEPTH
RIDER_TO_Z_ORIGIN = (0.0, 0.068, 0.070)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gantry_robot_axis")

    model.material("frame_gray", rgba=(0.57, 0.60, 0.64, 1.0))
    model.material("beam_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("rail_steel", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("rider_graphite", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("slide_silver", rgba=(0.84, 0.86, 0.88, 1.0))
    model.material("cable_black", rgba=(0.11, 0.12, 0.14, 1.0))
    model.material("buffer_black", rgba=(0.07, 0.07, 0.07, 1.0))

    top_beam = model.part("top_beam")
    top_beam.visual(
        mesh_from_cadquery(_beam_body_shape(), "top_beam_body"),
        material="beam_aluminum",
        name="beam_body",
    )
    top_beam.visual(
        mesh_from_cadquery(_beam_cable_channel_shape(), "beam_cable_channel"),
        material="cable_black",
        name="cable_channel",
    )
    top_beam.visual(
        Box((RAIL_LENGTH, RAIL_DEPTH, RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (BEAM_DEPTH / 2.0) + BEAM_FRONT_LAND_DEPTH + (RAIL_DEPTH / 2.0),
                RAIL_Z_OFFSET,
            )
        ),
        material="rail_steel",
        name="upper_rail",
    )
    top_beam.visual(
        Box((RAIL_LENGTH, RAIL_DEPTH, RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (BEAM_DEPTH / 2.0) + BEAM_FRONT_LAND_DEPTH + (RAIL_DEPTH / 2.0),
                -RAIL_Z_OFFSET,
            )
        ),
        material="rail_steel",
        name="lower_rail",
    )
    top_beam.visual(
        Box((0.04, 0.024, 0.07)),
        origin=Origin(xyz=(-0.58, 0.124, 0.0)),
        material="buffer_black",
        name="left_buffer",
    )
    top_beam.visual(
        Box((0.04, 0.024, 0.07)),
        origin=Origin(xyz=(0.58, 0.124, 0.0)),
        material="buffer_black",
        name="right_buffer",
    )
    top_beam.inertial = Inertial.from_geometry(
        Box((BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT)),
        mass=24.0,
        origin=Origin(),
    )

    left_frame = model.part("left_frame")
    left_frame.visual(
        mesh_from_cadquery(_side_frame_shape(), "left_frame_body"),
        material="frame_gray",
        name="frame_body",
    )
    left_frame.inertial = Inertial.from_geometry(
        Box((0.28, 0.34, 1.56)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.78)),
    )

    right_frame = model.part("right_frame")
    right_frame.visual(
        mesh_from_cadquery(_side_frame_shape(), "right_frame_body"),
        material="frame_gray",
        name="frame_body",
    )
    right_frame.inertial = Inertial.from_geometry(
        Box((0.28, 0.34, 1.56)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.78)),
    )

    beam_rider = model.part("beam_rider")
    beam_rider.visual(
        Box((0.20, 0.038, 0.22)),
        origin=Origin(xyz=(0.0, 0.019, 0.0)),
        material="rider_graphite",
        name="rear_block",
    )
    beam_rider.visual(
        Box((0.17, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, 0.007, RAIL_Z_OFFSET)),
        material="rail_steel",
        name="upper_guide_shoe",
    )
    beam_rider.visual(
        Box((0.17, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, 0.007, -RAIL_Z_OFFSET)),
        material="rail_steel",
        name="lower_guide_shoe",
    )
    beam_rider.visual(
        Box((0.032, 0.072, 0.24)),
        origin=Origin(xyz=(-0.056, 0.074, -0.01)),
        material="rider_graphite",
        name="left_cheek",
    )
    beam_rider.visual(
        Box((0.032, 0.072, 0.24)),
        origin=Origin(xyz=(0.056, 0.074, -0.01)),
        material="rider_graphite",
        name="right_cheek",
    )
    beam_rider.visual(
        Box((0.148, 0.024, 0.032)),
        origin=Origin(xyz=(0.0, 0.102, 0.112)),
        material="rider_graphite",
        name="top_bridge",
    )
    beam_rider.visual(
        Box((0.008, 0.014, 0.18)),
        origin=Origin(xyz=(-0.044, 0.075, -0.040)),
        material="rider_graphite",
        name="left_guide_pad",
    )
    beam_rider.visual(
        Box((0.008, 0.014, 0.18)),
        origin=Origin(xyz=(0.044, 0.075, -0.040)),
        material="rider_graphite",
        name="right_guide_pad",
    )
    beam_rider.inertial = Inertial.from_geometry(
        Box((0.24, 0.14, 0.26)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.06, -0.01)),
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((0.074, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material="slide_silver",
        name="top_head",
    )
    z_carriage.visual(
        Box((0.072, 0.026, 0.30)),
        origin=Origin(xyz=(0.0, 0.013, -0.16)),
        material="slide_silver",
        name="slide_body",
    )
    z_carriage.visual(
        Box((0.008, 0.014, 0.18)),
        origin=Origin(xyz=(-0.036, 0.007, -0.06)),
        material="rail_steel",
        name="left_runner",
    )
    z_carriage.visual(
        Box((0.008, 0.014, 0.18)),
        origin=Origin(xyz=(0.036, 0.007, -0.06)),
        material="rail_steel",
        name="right_runner",
    )
    z_carriage.visual(
        Box((0.132, 0.070, 0.10)),
        origin=Origin(xyz=(0.0, 0.035, -0.315)),
        material="slide_silver",
        name="mount_plate",
    )
    z_carriage.visual(
        Box((0.05, 0.048, 0.07)),
        origin=Origin(xyz=(0.0, 0.059, -0.395)),
        material="rider_graphite",
        name="tool_block",
    )
    z_carriage.inertial = Inertial.from_geometry(
        Box((0.14, 0.07, 0.40)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.035, -0.22)),
    )

    model.articulation(
        "beam_to_left_frame",
        ArticulationType.FIXED,
        parent=top_beam,
        child=left_frame,
        origin=Origin(xyz=(-FRAME_JOINT_X, 0.0, -(BEAM_HEIGHT / 2.0))),
    )
    model.articulation(
        "beam_to_right_frame",
        ArticulationType.FIXED,
        parent=top_beam,
        child=right_frame,
        origin=Origin(xyz=(FRAME_JOINT_X, 0.0, -(BEAM_HEIGHT / 2.0))),
    )
    model.articulation(
        "beam_to_rider",
        ArticulationType.PRISMATIC,
        parent=top_beam,
        child=beam_rider,
        origin=Origin(xyz=(0.0, BEAM_TO_RIDER_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=X_TRAVEL_LOWER,
            upper=X_TRAVEL_UPPER,
            effort=900.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "rider_to_z",
        ArticulationType.PRISMATIC,
        parent=beam_rider,
        child=z_carriage,
        origin=Origin(xyz=RIDER_TO_Z_ORIGIN),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=Z_TRAVEL_LOWER,
            upper=Z_TRAVEL_UPPER,
            effort=500.0,
            velocity=0.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_beam = object_model.get_part("top_beam")
    left_frame = object_model.get_part("left_frame")
    right_frame = object_model.get_part("right_frame")
    beam_rider = object_model.get_part("beam_rider")
    z_carriage = object_model.get_part("z_carriage")
    beam_to_rider = object_model.get_articulation("beam_to_rider")
    rider_to_z = object_model.get_articulation("rider_to_z")

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
        "beam_rider_moves_horizontally",
        tuple(round(v, 6) for v in beam_to_rider.axis) == (1.0, 0.0, 0.0),
        details=f"expected horizontal X-axis motion, got {beam_to_rider.axis}",
    )
    ctx.check(
        "z_carriage_moves_vertically",
        tuple(round(v, 6) for v in rider_to_z.axis) == (0.0, 0.0, -1.0),
        details=f"expected downward Z-axis motion, got {rider_to_z.axis}",
    )

    ctx.expect_contact(left_frame, top_beam, name="left_frame_supports_beam")
    ctx.expect_contact(right_frame, top_beam, name="right_frame_supports_beam")
    ctx.expect_contact(
        beam_rider,
        top_beam,
        contact_tol=0.0015,
        name="beam_rider_is_guided_by_beam",
    )
    ctx.expect_contact(
        z_carriage,
        beam_rider,
        contact_tol=0.0015,
        name="z_carriage_is_nested_into_rider_face",
    )
    ctx.expect_within(
        z_carriage,
        beam_rider,
        axes="x",
        margin=0.02,
        name="z_carriage_stays_within_rider_face_envelope",
    )

    with ctx.pose({beam_to_rider: X_TRAVEL_UPPER, rider_to_z: Z_TRAVEL_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_at_right_low_limit")
        ctx.expect_gap(
            top_beam,
            beam_rider,
            axis="x",
            positive_elem="right_buffer",
            min_gap=0.02,
            name="right_end_buffer_clearance",
        )
        ctx.expect_within(
            z_carriage,
            beam_rider,
            axes="x",
            margin=0.02,
            name="z_carriage_tracks_centered_at_right_limit",
        )

    with ctx.pose({beam_to_rider: X_TRAVEL_LOWER, rider_to_z: Z_TRAVEL_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_at_left_low_limit")
        ctx.expect_gap(
            beam_rider,
            top_beam,
            axis="x",
            negative_elem="left_buffer",
            min_gap=0.02,
            name="left_end_buffer_clearance",
        )
        ctx.expect_within(
            z_carriage,
            beam_rider,
            axes="x",
            margin=0.02,
            name="z_carriage_tracks_centered_at_left_limit",
        )

    return ctx.report()


def _beam_body_shape():
    beam = cq.Workplane("XY").box(BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT)
    beam = (
        beam.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .rect(BEAM_LENGTH - 0.28, 0.06)
        .cutBlind(-0.022)
    )
    beam = (
        beam.faces("<Z")
        .workplane(centerOption="CenterOfMass")
        .rect(BEAM_LENGTH - 0.30, BEAM_DEPTH - 0.08)
        .cutBlind(0.022)
    )
    beam = beam.union(
        cq.Workplane("XY")
        .box(RAIL_LENGTH + 0.05, BEAM_FRONT_LAND_DEPTH, 0.042)
        .translate((0.0, (BEAM_DEPTH / 2.0) + (BEAM_FRONT_LAND_DEPTH / 2.0), RAIL_Z_OFFSET))
    )
    beam = beam.union(
        cq.Workplane("XY")
        .box(RAIL_LENGTH + 0.05, BEAM_FRONT_LAND_DEPTH, 0.042)
        .translate((0.0, (BEAM_DEPTH / 2.0) + (BEAM_FRONT_LAND_DEPTH / 2.0), -RAIL_Z_OFFSET))
    )
    beam = beam.union(
        cq.Workplane("XY")
        .box(0.14, 0.18, 0.022)
        .translate((-FRAME_JOINT_X, 0.0, -(BEAM_HEIGHT / 2.0) + 0.011))
    )
    beam = beam.union(
        cq.Workplane("XY")
        .box(0.14, 0.18, 0.022)
        .translate((FRAME_JOINT_X, 0.0, -(BEAM_HEIGHT / 2.0) + 0.011))
    )
    return beam


def _beam_cable_channel_shape():
    tray = (
        cq.Workplane("XY")
        .box(0.82, 0.08, 0.05)
        .translate((0.0, -0.032, (BEAM_HEIGHT / 2.0) + 0.057))
    )
    tray = (
        tray.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(0.79, 0.052)
        .cutBlind(-0.042)
    )
    for x_pos in (-0.28, 0.0, 0.28):
        tray = tray.union(
            cq.Workplane("XY")
            .box(0.022, 0.046, 0.032)
            .translate((x_pos, -0.032, (BEAM_HEIGHT / 2.0) + 0.016))
        )
    return tray


def _side_frame_shape():
    frame = cq.Workplane("XY").box(0.20, 0.24, 0.10).translate((0.0, 0.0, -0.05))
    frame = frame.union(
        cq.Workplane("XY").box(0.14, 0.20, 1.22).translate((0.0, 0.0, -0.71))
    )
    frame = frame.union(
        cq.Workplane("XY").box(0.28, 0.34, 0.08).translate((0.0, 0.0, -1.50))
    )
    frame = frame.union(
        cq.Workplane("XY")
        .box(0.14, 0.06, 0.84)
        .translate((0.0, -0.085, -0.98))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -13.0)
    )
    frame = frame.union(
        cq.Workplane("XY")
        .box(0.12, 0.05, 0.42)
        .translate((0.0, 0.092, -0.28))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 19.0)
    )
    frame = frame.cut(
        cq.Workplane("XY").box(0.078, 0.22, 0.70).translate((0.0, 0.0, -0.78))
    )
    frame = frame.cut(
        cq.Workplane("XY").box(0.09, 0.12, 0.05).translate((0.0, 0.0, -0.05))
    )
    frame = frame.union(
        cq.Workplane("XY").box(0.08, 0.08, 0.028).translate((0.0, -0.105, -1.554))
    )
    frame = frame.union(
        cq.Workplane("XY").box(0.08, 0.08, 0.028).translate((0.0, 0.105, -1.554))
    )
    return frame


def _rider_shape():
    rider = cq.Workplane("XY").box(0.20, 0.05, 0.22).translate((0.0, 0.025, 0.0))
    rider = rider.union(
        cq.Workplane("XY").box(0.03, 0.05, 0.22).translate((-0.06, 0.085, -0.01))
    )
    rider = rider.union(
        cq.Workplane("XY").box(0.03, 0.05, 0.22).translate((0.06, 0.085, -0.01))
    )
    rider = rider.union(
        cq.Workplane("XY").box(0.15, 0.028, 0.045).translate((0.0, 0.074, 0.108))
    )
    rider = rider.union(
        cq.Workplane("XY").box(0.18, 0.016, 0.036).translate((0.0, 0.008, RAIL_Z_OFFSET))
    )
    rider = rider.union(
        cq.Workplane("XY").box(0.18, 0.016, 0.036).translate((0.0, 0.008, -RAIL_Z_OFFSET))
    )
    rider = rider.union(
        cq.Workplane("XY").box(0.044, 0.012, 0.18).translate((0.0, 0.054, -0.02))
    )
    rider = rider.union(
        cq.Workplane("XY").box(0.018, 0.05, 0.11).translate((0.109, 0.058, 0.03))
    )
    rider = rider.union(
        cq.Workplane("XY").box(0.12, 0.018, 0.03).translate((0.0, 0.112, 0.084))
    )
    return rider


def _z_carriage_shape():
    slide = cq.Workplane("XY").box(0.084, 0.026, 0.30).translate((0.0, 0.013, -0.15))
    slide = slide.union(
        cq.Workplane("XY").box(0.056, 0.012, 0.18).translate((0.0, 0.006, -0.03))
    )
    slide = slide.union(
        cq.Workplane("XY").box(0.14, 0.07, 0.11).translate((0.0, 0.035, -0.305))
    )
    slide = slide.union(
        cq.Workplane("XY").box(0.05, 0.045, 0.07).translate((0.0, 0.06, -0.39))
    )
    return slide


# >>> USER_CODE_END

object_model = build_object_model()
