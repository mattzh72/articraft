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


RAIL_LENGTH = 0.82
BASE_WIDTH = 0.12
BASE_HEIGHT = 0.03
GUIDE_LENGTH = 0.74
GUIDE_WIDTH = 0.06
GUIDE_HEIGHT = 0.05
GUIDE_CENTER_Z = BASE_HEIGHT + GUIDE_HEIGHT / 2.0

CAR_LENGTH = 0.16
CAR_TOP_WIDTH = 0.14
CAR_TOP_THICKNESS = 0.02
CAR_TOP_CENTER_Z = 0.047
CAR_SIDE_WALL_WIDTH = 0.02
CAR_SIDE_WALL_HEIGHT = 0.03
CAR_SIDE_WALL_CENTER_Z = 0.04
CAR_PAD_WIDTH = 0.014
CAR_PAD_THICKNESS = 0.01
CAR_PAD_CENTER_Z = GUIDE_HEIGHT / 2.0 + CAR_PAD_THICKNESS / 2.0
HOUSING_LENGTH = 0.082
HOUSING_WIDTH = 0.09
HOUSING_HEIGHT = 0.035
HOUSING_BOTTOM_Z = 0.057
HOUSING_CENTER_Z = HOUSING_BOTTOM_Z + HOUSING_HEIGHT / 2.0
SPINDLE_AXIS_Z = HOUSING_BOTTOM_Z + HOUSING_HEIGHT
BEARING_BORE_RADIUS = 0.0145

SLIDE_HOME_X = -0.22
SLIDE_TRAVEL = 0.44


def _make_rail_body() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, BASE_WIDTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.006)
    )

    left_foot = (
        cq.Workplane("XY")
        .box(0.12, 0.18, 0.02)
        .translate((-0.29, 0.0, 0.01))
        .edges("|Z")
        .fillet(0.005)
    )
    right_foot = (
        cq.Workplane("XY")
        .box(0.12, 0.18, 0.02)
        .translate((0.29, 0.0, 0.01))
        .edges("|Z")
        .fillet(0.005)
    )

    guide = (
        cq.Workplane("XY")
        .box(GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT + GUIDE_HEIGHT / 2.0))
        .edges("|X")
        .fillet(0.004)
        .edges("|Z")
        .fillet(0.003)
    )

    return base.union(left_foot).union(right_foot).union(guide)


def _make_carriage() -> cq.Workplane:
    top_bridge = (
        cq.Workplane("XY")
        .box(CAR_LENGTH, CAR_TOP_WIDTH, CAR_TOP_THICKNESS)
        .translate((0.0, 0.0, CAR_TOP_CENTER_Z))
        .edges("|Z")
        .fillet(0.006)
    )

    side_wall_center_y = GUIDE_WIDTH / 2.0 + 0.018
    left_wall = (
        cq.Workplane("XY")
        .box(CAR_LENGTH, CAR_SIDE_WALL_WIDTH, CAR_SIDE_WALL_HEIGHT)
        .translate((0.0, side_wall_center_y, CAR_SIDE_WALL_CENTER_Z))
        .edges("|X")
        .fillet(0.003)
    )
    right_wall = (
        cq.Workplane("XY")
        .box(CAR_LENGTH, CAR_SIDE_WALL_WIDTH, CAR_SIDE_WALL_HEIGHT)
        .translate((0.0, -side_wall_center_y, CAR_SIDE_WALL_CENTER_Z))
        .edges("|X")
        .fillet(0.003)
    )

    pad_center_y = 0.018
    left_pad = (
        cq.Workplane("XY")
        .box(CAR_LENGTH, CAR_PAD_WIDTH, CAR_PAD_THICKNESS)
        .translate((0.0, pad_center_y, CAR_PAD_CENTER_Z))
        .edges("|X")
        .fillet(0.0015)
    )
    right_pad = (
        cq.Workplane("XY")
        .box(CAR_LENGTH, CAR_PAD_WIDTH, CAR_PAD_THICKNESS)
        .translate((0.0, -pad_center_y, CAR_PAD_CENTER_Z))
        .edges("|X")
        .fillet(0.0015)
    )

    housing = (
        cq.Workplane("XY")
        .box(HOUSING_LENGTH, HOUSING_WIDTH, HOUSING_HEIGHT)
        .translate((0.0, 0.0, HOUSING_CENTER_Z))
        .edges("|Y")
        .fillet(0.006)
        .edges("|X")
        .fillet(0.004)
    )

    rear_web = (
        cq.Workplane("XY")
        .box(0.028, CAR_TOP_WIDTH, 0.028)
        .translate((-0.045, 0.0, 0.047))
        .edges("|Z")
        .fillet(0.003)
    )
    front_web = (
        cq.Workplane("XY")
        .box(0.028, CAR_TOP_WIDTH, 0.028)
        .translate((0.045, 0.0, 0.047))
        .edges("|Z")
        .fillet(0.003)
    )

    body = (
        top_bridge.union(left_wall)
        .union(right_wall)
        .union(left_pad)
        .union(right_pad)
        .union(housing)
        .union(rear_web)
        .union(front_web)
    )

    spindle_bore = (
        cq.Workplane("XY")
        .circle(BEARING_BORE_RADIUS)
        .extrude(SPINDLE_AXIS_Z + 0.002)
    )

    return body.cut(spindle_bore)


def _make_spindle() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(0.026).extrude(0.008)
    body = cq.Workplane("XY").circle(0.022).extrude(0.03).translate((0.0, 0.0, 0.008))
    tool_face = cq.Workplane("XY").circle(0.03).extrude(0.008).translate((0.0, 0.0, 0.038))
    nose = cq.Workplane("XY").circle(0.01).extrude(0.01).translate((0.0, 0.0, 0.046))

    spindle = flange.union(body).union(tool_face).union(nose)
    spindle = spindle.faces(">Z").workplane(centerOption="CenterOfMass").circle(0.008).cutBlind(-0.004)
    return spindle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_carriage_rotary_tool")

    anodized = model.material("anodized_aluminum", rgba=(0.68, 0.70, 0.74, 1.0))
    dark_body = model.material("dark_machine_body", rgba=(0.22, 0.24, 0.28, 1.0))
    steel = model.material("ground_steel", rgba=(0.78, 0.80, 0.82, 1.0))

    rail_body = model.part("rail_body")
    rail_body.visual(
        mesh_from_cadquery(_make_rail_body(), "rail_body"),
        name="rail_shell",
        material=anodized,
    )
    rail_body.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, 0.18, 0.08)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage(), "carriage"),
        name="carriage_shell",
        material=dark_body,
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CAR_LENGTH, CAR_TOP_WIDTH, 0.15)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(_make_spindle(), "spindle"),
        name="spindle_shell",
        material=steel,
    )
    spindle.inertial = Inertial.from_geometry(
        Box((0.06, 0.06, 0.06)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_body,
        child=carriage,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, GUIDE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.45,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=6.0,
            lower=-3.0,
            upper=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_body = object_model.get_part("rail_body")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    rail_to_carriage = object_model.get_articulation("rail_to_carriage")
    carriage_to_spindle = object_model.get_articulation("carriage_to_spindle")

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
        "parts_present",
        all(part is not None for part in (rail_body, carriage, spindle)),
        "Expected rail_body, carriage, and spindle parts.",
    )
    ctx.check(
        "slide_joint_is_prismatic_x",
        rail_to_carriage.articulation_type == ArticulationType.PRISMATIC
        and rail_to_carriage.axis == (1.0, 0.0, 0.0),
        f"Expected prismatic X slide, got type={rail_to_carriage.articulation_type!r}, axis={rail_to_carriage.axis!r}.",
    )
    ctx.check(
        "spindle_joint_is_revolute_z",
        carriage_to_spindle.articulation_type == ArticulationType.REVOLUTE
        and carriage_to_spindle.axis == (0.0, 0.0, 1.0),
        f"Expected revolute Z spindle, got type={carriage_to_spindle.articulation_type!r}, axis={carriage_to_spindle.axis!r}.",
    )

    with ctx.pose({rail_to_carriage: 0.0, carriage_to_spindle: 0.0}):
        ctx.expect_contact(
            carriage,
            rail_body,
            contact_tol=0.001,
            name="carriage_is_supported_on_rail_at_home",
        )
        ctx.expect_contact(
            spindle,
            carriage,
            contact_tol=0.001,
            name="spindle_is_supported_by_bearing_housing_at_home",
        )

    with ctx.pose({rail_to_carriage: SLIDE_TRAVEL, carriage_to_spindle: 1.35}):
        ctx.expect_contact(
            carriage,
            rail_body,
            contact_tol=0.001,
            name="carriage_stays_supported_at_full_travel",
        )
        ctx.expect_contact(
            spindle,
            carriage,
            contact_tol=0.001,
            name="spindle_stays_supported_when_rotated",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_articulated_pose")

    with ctx.pose({rail_to_carriage: 0.0}):
        home_pos = ctx.part_world_position(carriage)
    with ctx.pose({rail_to_carriage: SLIDE_TRAVEL}):
        far_pos = ctx.part_world_position(carriage)

    travel_ok = (
        home_pos is not None
        and far_pos is not None
        and (far_pos[0] - home_pos[0]) > 0.40
        and abs(far_pos[1] - home_pos[1]) < 1e-6
        and abs(far_pos[2] - home_pos[2]) < 1e-6
    )
    ctx.check(
        "carriage_translates_along_rail",
        travel_ok,
        f"Expected mostly pure +X travel; home={home_pos}, far={far_pos}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
