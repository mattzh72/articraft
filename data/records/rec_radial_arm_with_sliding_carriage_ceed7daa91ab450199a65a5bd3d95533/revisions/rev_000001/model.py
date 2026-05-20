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


BASE_SIZE = 0.42
BASE_THICKNESS = 0.05
PLINTH_SIZE = 0.24
PLINTH_HEIGHT = 0.08

COLUMN_RADIUS = 0.06
COLUMN_HEIGHT = 0.65
CAP_RADIUS = 0.09
CAP_THICKNESS = 0.025
SUPPORT_Z = BASE_THICKNESS + PLINTH_HEIGHT + COLUMN_HEIGHT + CAP_THICKNESS

COLLAR_OUTER_RADIUS = 0.11
COLLAR_INNER_RADIUS = 0.064
COLLAR_HEIGHT = 0.18

BEAM_LENGTH = 0.68
BEAM_WIDTH = 0.10
BEAM_HEIGHT = 0.12
BEAM_CENTER_Z = 0.09
ARM_START_X = 0.11
ARM_CENTER_X = ARM_START_X + (BEAM_LENGTH / 2.0)

CARRIAGE_LENGTH = 0.15
CARRIAGE_WIDTH = 0.18
CARRIAGE_HEIGHT = 0.14
CARRIAGE_HOME_X = 0.30
CARRIAGE_TRAVEL = 0.40


def _make_pedestal_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_SIZE, BASE_SIZE, BASE_THICKNESS).translate(
        (0.0, 0.0, BASE_THICKNESS / 2.0)
    )
    plinth = cq.Workplane("XY").box(PLINTH_SIZE, PLINTH_SIZE, PLINTH_HEIGHT).translate(
        (0.0, 0.0, BASE_THICKNESS + (PLINTH_HEIGHT / 2.0))
    )
    column = cq.Workplane("XY").circle(COLUMN_RADIUS).extrude(COLUMN_HEIGHT).translate(
        (0.0, 0.0, BASE_THICKNESS + PLINTH_HEIGHT)
    )
    cap = cq.Workplane("XY").circle(CAP_RADIUS).extrude(CAP_THICKNESS).translate(
        (0.0, 0.0, BASE_THICKNESS + PLINTH_HEIGHT + COLUMN_HEIGHT)
    )
    return base.union(plinth).union(column).union(cap)


def _make_beam_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(COLLAR_OUTER_RADIUS).extrude(COLLAR_HEIGHT)
    collar = collar.cut(
        cq.Workplane("XY").circle(COLLAR_INNER_RADIUS).extrude(COLLAR_HEIGHT)
    )

    arm = cq.Workplane("XY").box(BEAM_LENGTH, BEAM_WIDTH, BEAM_HEIGHT).translate(
        (ARM_CENTER_X, 0.0, BEAM_CENTER_Z)
    )

    gusset = (
        cq.Workplane("XZ")
        .moveTo(0.02, 0.02)
        .lineTo(0.22, 0.02)
        .lineTo(0.22, 0.05)
        .lineTo(0.10, 0.15)
        .lineTo(0.02, 0.15)
        .close()
        .extrude(0.08, both=True)
    )

    end_stop = cq.Workplane("XY").box(0.02, 0.12, 0.16).translate(
        (ARM_START_X + BEAM_LENGTH + 0.01, 0.0, 0.08)
    )

    return collar.union(arm).union(gusset).union(end_stop)


def _make_carriage_shape() -> cq.Workplane:
    slide_shoe = cq.Workplane("XY").box(0.14, 0.11, 0.024).translate((0.0, 0.0, 0.012))
    housing = cq.Workplane("XY").box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, 0.11).translate(
        (0.0, 0.0, 0.079)
    )
    left_skirt = cq.Workplane("XY").box(0.12, 0.02, 0.08).translate((0.0, 0.066, 0.04))
    right_skirt = cq.Workplane("XY").box(0.12, 0.02, 0.08).translate((0.0, -0.066, 0.04))
    motor_bump = cq.Workplane("XY").circle(0.028).extrude(0.045).translate((0.03, 0.0, 0.134))

    return slide_shoe.union(housing).union(left_skirt).union(right_skirt).union(motor_bump)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turret_carriage_radial_arm")

    cast_gray = model.material("cast_gray", color=(0.45, 0.47, 0.50))
    beam_blue = model.material("beam_blue", color=(0.22, 0.36, 0.55))
    carriage_gray = model.material("carriage_gray", color=(0.68, 0.70, 0.72))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_make_pedestal_shape(), "pedestal_body"),
        material=cast_gray,
        name="pedestal_body",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((BASE_SIZE, BASE_SIZE, SUPPORT_Z)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_Z / 2.0)),
    )

    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(_make_beam_shape(), "rotating_beam"),
        material=beam_blue,
        name="rotating_beam",
    )
    beam.inertial = Inertial.from_geometry(
        Box((ARM_START_X + BEAM_LENGTH + 0.03, 0.22, COLLAR_HEIGHT)),
        mass=24.0,
        origin=Origin(xyz=(0.36, 0.0, COLLAR_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "beam_carriage"),
        material=carriage_gray,
        name="beam_carriage",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT / 2.0)),
    )

    model.articulation(
        "turret_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=1.0,
            lower=-2.2,
            upper=2.2,
        ),
    )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_HOME_X, 0.0, BEAM_CENTER_Z + (BEAM_HEIGHT / 2.0))),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    turret_yaw = object_model.get_articulation("turret_yaw")
    carriage_slide = object_model.get_articulation("carriage_slide")

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

    ctx.expect_gap(
        beam,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="beam_seated_on_column_cap",
    )
    ctx.expect_overlap(
        beam,
        pedestal,
        axes="xy",
        min_overlap=0.12,
        name="beam_collar_registered_over_column_cap",
    )

    with ctx.pose({turret_yaw: 0.0, carriage_slide: 0.18}):
        ctx.expect_contact(
            carriage,
            beam,
            contact_tol=0.001,
            name="carriage_contacts_beam_at_midspan",
        )
        ctx.expect_overlap(
            carriage,
            beam,
            axes="xy",
            min_overlap=0.10,
            name="carriage_sits_over_beam_footprint",
        )

    with ctx.pose({turret_yaw: 0.0, carriage_slide: CARRIAGE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            beam,
            contact_tol=0.001,
            name="carriage_remains_guided_at_extension",
        )

    with ctx.pose({turret_yaw: 0.0, carriage_slide: 0.18}):
        neutral_carriage_position = ctx.part_world_position(carriage)
    with ctx.pose({turret_yaw: 0.7, carriage_slide: 0.18}):
        swung_carriage_position = ctx.part_world_position(carriage)

    turret_ok = (
        neutral_carriage_position is not None
        and swung_carriage_position is not None
        and swung_carriage_position[1] > neutral_carriage_position[1] + 0.12
        and abs(swung_carriage_position[2] - neutral_carriage_position[2]) <= 0.002
    )
    ctx.check(
        "turret_rotates_carriage_around_column_axis",
        turret_ok,
        details=(
            f"neutral={neutral_carriage_position}, swung={swung_carriage_position}; "
            "expected positive turret motion to swing the carriage toward +Y "
            "without lifting it."
        ),
    )

    with ctx.pose({turret_yaw: 0.0, carriage_slide: 0.0}):
        retracted_carriage_position = ctx.part_world_position(carriage)
    with ctx.pose({turret_yaw: 0.0, carriage_slide: 0.40}):
        extended_carriage_position = ctx.part_world_position(carriage)

    slide_ok = (
        retracted_carriage_position is not None
        and extended_carriage_position is not None
        and extended_carriage_position[0] > retracted_carriage_position[0] + 0.30
        and abs(extended_carriage_position[1] - retracted_carriage_position[1]) <= 0.002
        and abs(extended_carriage_position[2] - retracted_carriage_position[2]) <= 0.002
    )
    ctx.check(
        "carriage_translates_out_along_beam",
        slide_ok,
        details=(
            f"retracted={retracted_carriage_position}, "
            f"extended={extended_carriage_position}; "
            "expected prismatic motion to move along +X at q=0."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
