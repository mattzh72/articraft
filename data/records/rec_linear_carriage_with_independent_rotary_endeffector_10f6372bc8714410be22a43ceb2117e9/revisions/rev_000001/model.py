from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BODY_LENGTH = 0.34
BODY_WIDTH = 0.17
BASE_THICKNESS = 0.018
FOOT_HEIGHT = 0.012
FOOT_LENGTH = 0.045
FOOT_WIDTH = 0.030
RAIL_LENGTH = 0.23
RAIL_WIDTH = 0.022
RAIL_HEIGHT = 0.016
RAIL_SPACING = 0.090
RAIL_CENTER_X = 0.015
REAR_HOUSING_LENGTH = 0.090
REAR_HOUSING_WIDTH = 0.142
REAR_HOUSING_HEIGHT = 0.074
BODY_HEIGHT = BASE_THICKNESS + REAR_HOUSING_HEIGHT

CARRIAGE_LENGTH = 0.115
CARRIAGE_WIDTH = 0.098
CARRIAGE_BODY_HEIGHT = 0.034
CARRIAGE_PAD_HEIGHT = 0.008
CARRIAGE_PAD_LENGTH = 0.100
CARRIAGE_PAD_WIDTH = 0.030
CARRIAGE_HEIGHT = CARRIAGE_PAD_HEIGHT + CARRIAGE_BODY_HEIGHT
SLIDE_LOWER = -0.035
SLIDE_UPPER = 0.055

SPINDLE_FLANGE_RADIUS = 0.030
SPINDLE_FLANGE_LENGTH = 0.014
SPINDLE_BODY_RADIUS = 0.021
SPINDLE_BODY_LENGTH = 0.050
SPINDLE_NOSE_RADIUS = 0.012
SPINDLE_NOSE_LENGTH = 0.022
SPINDLE_TAPER_LENGTH = 0.018
SPINDLE_TIP_RADIUS = 0.004
SPINDLE_TIP_LENGTH = 0.010
SPINDLE_TOTAL_LENGTH = (
    SPINDLE_FLANGE_LENGTH
    + SPINDLE_BODY_LENGTH
    + SPINDLE_NOSE_LENGTH
    + SPINDLE_TAPER_LENGTH
    + SPINDLE_TIP_LENGTH
)


def _body_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BODY_LENGTH,
        BODY_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )

    feet = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.110, -0.055),
                (-0.110, 0.055),
                (0.105, -0.055),
                (0.105, 0.055),
            ]
        )
        .box(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, -FOOT_HEIGHT))
    )

    rails = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (RAIL_CENTER_X, -RAIL_SPACING / 2.0),
                (RAIL_CENTER_X, RAIL_SPACING / 2.0),
            ]
        )
        .box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_THICKNESS))
    )

    rear_housing = (
        cq.Workplane("XY")
        .box(
            REAR_HOUSING_LENGTH,
            REAR_HOUSING_WIDTH,
            REAR_HOUSING_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                -BODY_LENGTH / 2.0 + REAR_HOUSING_LENGTH / 2.0 + 0.020,
                0.0,
                BASE_THICKNESS,
            )
        )
        .edges("|Z")
        .fillet(0.006)
    )

    service_cap = (
        cq.Workplane("XY")
        .box(0.050, 0.082, 0.020, centered=(True, True, False))
        .translate(
            (
                -BODY_LENGTH / 2.0 + REAR_HOUSING_LENGTH / 2.0 + 0.006,
                0.0,
                BASE_THICKNESS + REAR_HOUSING_HEIGHT - 0.004,
            )
        )
        .edges("|Z")
        .fillet(0.004)
    )

    return base.union(feet).union(rails).union(rear_housing).union(service_cap)


def _carriage_shape() -> cq.Workplane:
    carriage_body = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_LENGTH,
            CARRIAGE_WIDTH,
            CARRIAGE_BODY_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CARRIAGE_PAD_HEIGHT))
    )

    guide_pads = (
        cq.Workplane("XY")
        .pushPoints([(0.0, -RAIL_SPACING / 2.0), (0.0, RAIL_SPACING / 2.0)])
        .box(
            CARRIAGE_PAD_LENGTH,
            CARRIAGE_PAD_WIDTH,
            CARRIAGE_PAD_HEIGHT,
            centered=(True, True, False),
        )
    )

    cable_hump = (
        cq.Workplane("XY")
        .box(0.040, 0.060, 0.018, centered=(True, True, False))
        .translate((-0.018, 0.0, CARRIAGE_HEIGHT - 0.006))
        .edges("|Z")
        .fillet(0.003)
    )

    carriage = carriage_body.union(guide_pads).union(cable_hump)
    return (
        carriage.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .circle(0.018)
        .cutBlind(-0.004)
    )


def _spindle_shape() -> cq.Workplane:
    flange = cq.Workplane("YZ").circle(SPINDLE_FLANGE_RADIUS).extrude(SPINDLE_FLANGE_LENGTH)
    body = (
        cq.Workplane("YZ")
        .circle(SPINDLE_BODY_RADIUS)
        .extrude(SPINDLE_BODY_LENGTH)
        .translate((SPINDLE_FLANGE_LENGTH, 0.0, 0.0))
    )
    nose = (
        cq.Workplane("YZ")
        .circle(SPINDLE_NOSE_RADIUS)
        .extrude(SPINDLE_NOSE_LENGTH)
        .translate((SPINDLE_FLANGE_LENGTH + SPINDLE_BODY_LENGTH, 0.0, 0.0))
    )
    taper = (
        cq.Workplane("YZ")
        .circle(SPINDLE_NOSE_RADIUS)
        .workplane(offset=SPINDLE_TAPER_LENGTH)
        .circle(SPINDLE_TIP_RADIUS)
        .loft(combine=True)
        .translate(
            (
                SPINDLE_FLANGE_LENGTH + SPINDLE_BODY_LENGTH + SPINDLE_NOSE_LENGTH,
                0.0,
                0.0,
            )
        )
    )
    tip = (
        cq.Workplane("YZ")
        .circle(SPINDLE_TIP_RADIUS)
        .extrude(SPINDLE_TIP_LENGTH)
        .translate(
            (
                SPINDLE_FLANGE_LENGTH
                + SPINDLE_BODY_LENGTH
                + SPINDLE_NOSE_LENGTH
                + SPINDLE_TAPER_LENGTH,
                0.0,
                0.0,
            )
        )
    )
    service_lug = (
        cq.Workplane("XY")
        .box(0.028, 0.018, 0.018, centered=(True, True, False))
        .translate((0.046, 0.026, 0.024))
        .edges("|Z")
        .fillet(0.0025)
    )
    mounting_rib = (
        cq.Workplane("XY")
        .box(0.022, 0.012, 0.012, centered=(True, True, False))
        .translate((0.046, 0.018, 0.015))
    )

    return (
        flange.union(body)
        .union(nose)
        .union(taper)
        .union(tip)
        .union(mounting_rib)
        .union(service_lug)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_shuttle_with_spindle")

    model.material("body_paint", rgba=(0.22, 0.29, 0.35, 1.0))
    model.material("carriage_alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("spindle_steel", rgba=(0.56, 0.58, 0.60, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="body_paint",
        name="base_plate",
    )
    for index, (x_pos, y_pos) in enumerate(
        [
            (-0.110, -0.055),
            (-0.110, 0.055),
            (0.105, -0.055),
            (0.105, 0.055),
        ],
        start=1,
    ):
        body.visual(
            Box((FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT)),
            origin=Origin(xyz=(x_pos, y_pos, -FOOT_HEIGHT / 2.0)),
            material="body_paint",
            name=f"foot_{index}",
        )
    for side, y_pos in (("left", -RAIL_SPACING / 2.0), ("right", RAIL_SPACING / 2.0)):
        body.visual(
            Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
            origin=Origin(
                xyz=(RAIL_CENTER_X, y_pos, BASE_THICKNESS + RAIL_HEIGHT / 2.0)
            ),
            material="body_paint",
            name=f"{side}_rail",
        )
    body.visual(
        Box((REAR_HOUSING_LENGTH, REAR_HOUSING_WIDTH, REAR_HOUSING_HEIGHT)),
        origin=Origin(
            xyz=(
                -BODY_LENGTH / 2.0 + REAR_HOUSING_LENGTH / 2.0 + 0.020,
                0.0,
                BASE_THICKNESS + REAR_HOUSING_HEIGHT / 2.0,
            )
        ),
        material="body_paint",
        name="rear_housing",
    )
    body.visual(
        Box((0.050, 0.082, 0.020)),
        origin=Origin(
            xyz=(
                -BODY_LENGTH / 2.0 + REAR_HOUSING_LENGTH / 2.0 + 0.006,
                0.0,
                BASE_THICKNESS + REAR_HOUSING_HEIGHT + 0.010,
            )
        ),
        material="body_paint",
        name="service_cap",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT + FOOT_HEIGHT)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, (BODY_HEIGHT - FOOT_HEIGHT) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BODY_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, CARRIAGE_PAD_HEIGHT + CARRIAGE_BODY_HEIGHT / 2.0)
        ),
        material="carriage_alloy",
        name="carriage_shell",
    )
    for side, y_pos in (("left", -RAIL_SPACING / 2.0), ("right", RAIL_SPACING / 2.0)):
        carriage.visual(
            Box((CARRIAGE_PAD_LENGTH, CARRIAGE_PAD_WIDTH, CARRIAGE_PAD_HEIGHT)),
            origin=Origin(xyz=(0.0, y_pos, CARRIAGE_PAD_HEIGHT / 2.0)),
            material="carriage_alloy",
            name=f"{side}_pad",
        )
    carriage.visual(
        Box((0.040, 0.060, 0.018)),
        origin=Origin(xyz=(-0.018, 0.0, 0.045)),
        material="carriage_alloy",
        name="cable_hump",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT / 2.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(_spindle_shape(), "service_shuttle_spindle"),
        material="spindle_steel",
        name="spindle_shell",
    )
    spindle.inertial = Inertial.from_geometry(
        Box((SPINDLE_TOTAL_LENGTH, 0.060, 0.060)),
        mass=0.7,
        origin=Origin(xyz=(SPINDLE_TOTAL_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, BASE_THICKNESS + RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
            effort=220.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(CARRIAGE_LENGTH / 2.0, 0.0, 0.021)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-2.0 * pi,
            upper=2.0 * pi,
            effort=18.0,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("body_to_carriage")
    spin = object_model.get_articulation("carriage_to_spindle")

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
        carriage,
        body,
        name="carriage sits on the body guide rails",
    )
    ctx.expect_contact(
        spindle,
        carriage,
        name="spindle cartridge seats against the carriage nose",
    )
    ctx.expect_within(
        carriage,
        body,
        axes="yz",
        margin=0.004,
        name="carriage stays centered over the guideway",
    )
    ctx.expect_overlap(
        carriage,
        body,
        axes="x",
        min_overlap=0.090,
        name="carriage keeps substantial guide engagement at home",
    )

    carriage_rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_UPPER}):
        ctx.expect_contact(
            carriage,
            body,
            name="carriage remains supported at maximum extension",
        )
        ctx.expect_within(
            carriage,
            body,
            axes="yz",
            margin=0.004,
            name="extended carriage stays aligned to the rails",
        )
        ctx.expect_overlap(
            carriage,
            body,
            axes="x",
            min_overlap=0.080,
            name="extended carriage retains rail engagement",
        )
        carriage_extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage slides forward along the shuttle body",
        carriage_rest_pos is not None
        and carriage_extended_pos is not None
        and carriage_extended_pos[0] > carriage_rest_pos[0] + 0.04,
        details=f"rest={carriage_rest_pos}, extended={carriage_extended_pos}",
    )

    spindle_rest_aabb = ctx.part_world_aabb(spindle)
    with ctx.pose({spin: pi / 2.0}):
        spindle_quarter_aabb = ctx.part_world_aabb(spindle)

    spindle_rest_z = spindle_rest_aabb[1][2] if spindle_rest_aabb is not None else None
    spindle_quarter_z = (
        spindle_quarter_aabb[1][2] if spindle_quarter_aabb is not None else None
    )
    spindle_rest_ymin = spindle_rest_aabb[0][1] if spindle_rest_aabb is not None else None
    spindle_quarter_ymin = (
        spindle_quarter_aabb[0][1] if spindle_quarter_aabb is not None else None
    )
    ctx.check(
        "spindle rotates about its longitudinal axis",
        spindle_rest_z is not None
        and spindle_quarter_z is not None
        and spindle_rest_ymin is not None
        and spindle_quarter_ymin is not None
        and spindle_quarter_z < spindle_rest_z - 0.006
        and spindle_quarter_ymin < spindle_rest_ymin - 0.006,
        details=(
            f"rest_aabb={spindle_rest_aabb}, quarter_turn_aabb={spindle_quarter_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
