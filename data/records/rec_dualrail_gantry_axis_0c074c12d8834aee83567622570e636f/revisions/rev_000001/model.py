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


BASE_LENGTH = 0.72
BASE_WIDTH = 0.42
BASE_THICKNESS = 0.022

GUIDEWAY_LENGTH = 0.60
GUIDEWAY_WIDTH = 0.032
GUIDEWAY_BASE_HEIGHT = 0.010
GUIDEWAY_RIB_WIDTH = 0.018
GUIDEWAY_RIB_HEIGHT = 0.008
GUIDEWAY_HEIGHT = GUIDEWAY_BASE_HEIGHT + GUIDEWAY_RIB_HEIGHT
GUIDEWAY_OFFSET_Y = 0.14

BEAM_LENGTH = 0.11
BEAM_SPAN = 0.324
BEAM_BODY_HEIGHT = 0.036
BEAM_POCKET_LENGTH = 0.068
BEAM_POCKET_WIDTH = 0.220
BEAM_POCKET_DEPTH = 0.020
BEAM_GUIDE_WIDTH = 0.092
BEAM_GUIDE_LENGTH = 0.304
BEAM_GUIDE_HEIGHT = 0.012
BEAM_TOTAL_HEIGHT = BEAM_BODY_HEIGHT + BEAM_GUIDE_HEIGHT
BEAM_TRAVEL_HALF = 0.20

CARRIAGE_BODY_LENGTH = 0.160
CARRIAGE_BODY_WIDTH = 0.120
CARRIAGE_BODY_HEIGHT = 0.028
CARRIAGE_SADDLE_LENGTH = 0.100
CARRIAGE_SADDLE_WIDTH = 0.120
CARRIAGE_SADDLE_HEIGHT = 0.022
CARRIAGE_TOTAL_HEIGHT = CARRIAGE_SADDLE_HEIGHT + CARRIAGE_BODY_HEIGHT
CARRIAGE_TRAVEL_HALF = 0.09

CONTACT_TOL = 2e-4


def _base_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )
    plate = plate.edges("|Z").fillet(0.004)
    slot_points = [
        (-0.26, -0.145),
        (-0.26, 0.145),
        (0.26, -0.145),
        (0.26, 0.145),
    ]
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(slot_points)
        .slot2D(0.040, 0.012, angle=0)
        .cutBlind(-BASE_THICKNESS)
    )
    return plate


def _guideway_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        GUIDEWAY_LENGTH,
        GUIDEWAY_WIDTH,
        GUIDEWAY_BASE_HEIGHT,
        centered=(True, True, False),
    )
    rib = (
        cq.Workplane("XY")
        .box(
            GUIDEWAY_LENGTH,
            GUIDEWAY_RIB_WIDTH,
            GUIDEWAY_RIB_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, GUIDEWAY_BASE_HEIGHT))
    )
    rail = base.union(rib)
    hole_points = [(-0.22, 0.0), (-0.11, 0.0), (0.0, 0.0), (0.11, 0.0), (0.22, 0.0)]
    rail = (
        rail.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(hole_points)
        .hole(0.0055)
    )
    return rail


def _gantry_beam_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        BEAM_LENGTH,
        BEAM_SPAN,
        BEAM_BODY_HEIGHT,
        centered=(True, True, False),
    )
    center_pocket = cq.Workplane("XY").box(
        BEAM_POCKET_LENGTH,
        BEAM_POCKET_WIDTH,
        BEAM_POCKET_DEPTH,
        centered=(True, True, False),
    )
    top_guide = (
        cq.Workplane("XY")
        .box(
            BEAM_GUIDE_WIDTH,
            BEAM_GUIDE_LENGTH,
            BEAM_GUIDE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BEAM_BODY_HEIGHT))
    )
    beam = body.cut(center_pocket).union(top_guide)
    return beam


def _cross_slide_shape() -> cq.Workplane:
    saddle = cq.Workplane("XY").box(
        CARRIAGE_SADDLE_LENGTH,
        CARRIAGE_SADDLE_WIDTH,
        CARRIAGE_SADDLE_HEIGHT,
        centered=(True, True, False),
    )
    body = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_BODY_LENGTH,
            CARRIAGE_BODY_WIDTH,
            CARRIAGE_BODY_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CARRIAGE_SADDLE_HEIGHT))
    )
    carriage = saddle.union(body)
    carriage = (
        carriage.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.040, 0.0), (0.040, 0.0)])
        .hole(0.006)
    )
    return carriage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_rail_gantry")

    model.material("plate_gray", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("rail_steel", rgba=(0.54, 0.57, 0.61, 1.0))
    model.material("beam_aluminum", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("carriage_dark", rgba=(0.22, 0.24, 0.27, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate"),
        material="plate_gray",
        name="plate_shell",
    )
    base_plate.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    left_guideway = model.part("left_guideway")
    left_guideway.visual(
        mesh_from_cadquery(_guideway_shape(), "left_guideway"),
        material="rail_steel",
        name="left_rail_shell",
    )
    left_guideway.inertial = Inertial.from_geometry(
        Box((GUIDEWAY_LENGTH, GUIDEWAY_WIDTH, GUIDEWAY_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, GUIDEWAY_HEIGHT / 2.0)),
    )

    right_guideway = model.part("right_guideway")
    right_guideway.visual(
        mesh_from_cadquery(_guideway_shape(), "right_guideway"),
        material="rail_steel",
        name="right_rail_shell",
    )
    right_guideway.inertial = Inertial.from_geometry(
        Box((GUIDEWAY_LENGTH, GUIDEWAY_WIDTH, GUIDEWAY_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, GUIDEWAY_HEIGHT / 2.0)),
    )

    gantry_beam = model.part("gantry_beam")
    gantry_beam.visual(
        mesh_from_cadquery(_gantry_beam_shape(), "gantry_beam"),
        material="beam_aluminum",
        name="beam_shell",
    )
    gantry_beam.inertial = Inertial.from_geometry(
        Box((BEAM_LENGTH, BEAM_SPAN, BEAM_TOTAL_HEIGHT)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, BEAM_TOTAL_HEIGHT / 2.0)),
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        mesh_from_cadquery(_cross_slide_shape(), "cross_slide"),
        material="carriage_dark",
        name="carriage_shell",
    )
    cross_slide.inertial = Inertial.from_geometry(
        Box((CARRIAGE_BODY_LENGTH, CARRIAGE_BODY_WIDTH, CARRIAGE_TOTAL_HEIGHT)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_TOTAL_HEIGHT / 2.0)),
    )

    model.articulation(
        "base_to_left_guideway",
        ArticulationType.FIXED,
        parent=base_plate,
        child=left_guideway,
        origin=Origin(xyz=(0.0, GUIDEWAY_OFFSET_Y, BASE_THICKNESS)),
    )
    model.articulation(
        "base_to_right_guideway",
        ArticulationType.FIXED,
        parent=base_plate,
        child=right_guideway,
        origin=Origin(xyz=(0.0, -GUIDEWAY_OFFSET_Y, BASE_THICKNESS)),
    )
    model.articulation(
        "base_to_beam",
        ArticulationType.PRISMATIC,
        parent=base_plate,
        child=gantry_beam,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + GUIDEWAY_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BEAM_TRAVEL_HALF,
            upper=BEAM_TRAVEL_HALF,
            effort=350.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "beam_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=gantry_beam,
        child=cross_slide,
        origin=Origin(xyz=(0.0, 0.0, BEAM_TOTAL_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CARRIAGE_TRAVEL_HALF,
            upper=CARRIAGE_TRAVEL_HALF,
            effort=180.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_plate = object_model.get_part("base_plate")
    left_guideway = object_model.get_part("left_guideway")
    right_guideway = object_model.get_part("right_guideway")
    gantry_beam = object_model.get_part("gantry_beam")
    cross_slide = object_model.get_part("cross_slide")
    beam_slide = object_model.get_articulation("base_to_beam")
    cross_slide_joint = object_model.get_articulation("beam_to_cross_slide")

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

    ctx.expect_contact(
        left_guideway,
        base_plate,
        contact_tol=CONTACT_TOL,
        name="left guideway seats on base plate",
    )
    ctx.expect_contact(
        right_guideway,
        base_plate,
        contact_tol=CONTACT_TOL,
        name="right guideway seats on base plate",
    )
    ctx.expect_within(
        left_guideway,
        base_plate,
        axes="xy",
        margin=0.0,
        name="left guideway footprint stays on base plate",
    )
    ctx.expect_within(
        right_guideway,
        base_plate,
        axes="xy",
        margin=0.0,
        name="right guideway footprint stays on base plate",
    )
    ctx.expect_contact(
        gantry_beam,
        left_guideway,
        contact_tol=CONTACT_TOL,
        name="beam is supported by left guideway",
    )
    ctx.expect_contact(
        gantry_beam,
        right_guideway,
        contact_tol=CONTACT_TOL,
        name="beam is supported by right guideway",
    )
    ctx.expect_contact(
        cross_slide,
        gantry_beam,
        contact_tol=CONTACT_TOL,
        name="cross-slide sits on beam guide",
    )

    with ctx.pose({beam_slide: beam_slide.motion_limits.lower}):
        beam_low = ctx.part_world_position(gantry_beam)
        ctx.expect_within(
            gantry_beam,
            base_plate,
            axes="x",
            margin=0.0,
            name="beam remains over base at lower travel",
        )
        ctx.expect_contact(
            gantry_beam,
            left_guideway,
            contact_tol=CONTACT_TOL,
            name="beam remains on left guideway at lower travel",
        )
        ctx.expect_contact(
            gantry_beam,
            right_guideway,
            contact_tol=CONTACT_TOL,
            name="beam remains on right guideway at lower travel",
        )

    with ctx.pose({beam_slide: beam_slide.motion_limits.upper}):
        beam_high = ctx.part_world_position(gantry_beam)
        ctx.expect_within(
            gantry_beam,
            base_plate,
            axes="x",
            margin=0.0,
            name="beam remains over base at upper travel",
        )
        ctx.expect_contact(
            gantry_beam,
            left_guideway,
            contact_tol=CONTACT_TOL,
            name="beam remains on left guideway at upper travel",
        )
        ctx.expect_contact(
            gantry_beam,
            right_guideway,
            contact_tol=CONTACT_TOL,
            name="beam remains on right guideway at upper travel",
        )

    with ctx.pose({cross_slide_joint: cross_slide_joint.motion_limits.lower}):
        carriage_low = ctx.part_world_position(cross_slide)
        ctx.expect_contact(
            cross_slide,
            gantry_beam,
            contact_tol=CONTACT_TOL,
            name="cross-slide stays seated at lower cross travel",
        )
        ctx.expect_within(
            cross_slide,
            gantry_beam,
            axes="y",
            margin=0.0,
            name="cross-slide stays within beam span at lower cross travel",
        )

    with ctx.pose({cross_slide_joint: cross_slide_joint.motion_limits.upper}):
        carriage_high = ctx.part_world_position(cross_slide)
        ctx.expect_contact(
            cross_slide,
            gantry_beam,
            contact_tol=CONTACT_TOL,
            name="cross-slide stays seated at upper cross travel",
        )
        ctx.expect_within(
            cross_slide,
            gantry_beam,
            axes="y",
            margin=0.0,
            name="cross-slide stays within beam span at upper cross travel",
        )

    ctx.check(
        "beam prismatic joint moves along x",
        beam_low is not None
        and beam_high is not None
        and (beam_high[0] - beam_low[0]) > 0.39
        and abs(beam_high[1] - beam_low[1]) < 5e-5
        and abs(beam_high[2] - beam_low[2]) < 5e-5,
        details=f"beam positions: lower={beam_low}, upper={beam_high}",
    )
    ctx.check(
        "cross-slide prismatic joint moves along y",
        carriage_low is not None
        and carriage_high is not None
        and (carriage_high[1] - carriage_low[1]) > 0.17
        and abs(carriage_high[0] - carriage_low[0]) < 5e-5
        and abs(carriage_high[2] - carriage_low[2]) < 5e-5,
        details=f"cross-slide positions: lower={carriage_low}, upper={carriage_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
