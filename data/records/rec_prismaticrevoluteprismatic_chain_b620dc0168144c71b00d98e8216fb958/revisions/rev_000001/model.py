from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_LENGTH = 0.90
BASE_WIDTH = 0.34
FOOT_HEIGHT = 0.018
DECK_LENGTH = 0.84
DECK_WIDTH = 0.24
DECK_THICKNESS = 0.018
RAIL_LENGTH = 0.74
RAIL_WIDTH = 0.040
RAIL_HEIGHT = 0.026
RAIL_Y = 0.105
RAIL_TOP_Z = FOOT_HEIGHT + DECK_THICKNESS + RAIL_HEIGHT

BASE_SLIDE_START_X = -0.18
BASE_SLIDE_TRAVEL = 0.36

CARR_BODY_LENGTH = 0.23
CARR_BODY_WIDTH = 0.25
CARR_BODY_THICKNESS = 0.028
CARR_BODY_Z = -0.058
CARR_BODY_X = -0.155
RUNNER_LENGTH = 0.18
RUNNER_WIDTH = 0.056
RUNNER_HEIGHT = 0.024
AXIS_TO_RAIL_TOP = 0.10
RUNNER_Z = -(AXIS_TO_RAIL_TOP - RUNNER_HEIGHT / 2.0)
RUNNER_X = -0.145
RUNNER_BRACKET_LENGTH = 0.108
RUNNER_BRACKET_WIDTH = 0.040
RUNNER_BRACKET_HEIGHT = 0.012
RUNNER_BRACKET_Z = -0.072
CLEVIS_Y = 0.072
CHEEK_LENGTH = 0.032
CHEEK_WIDTH = 0.024
CHEEK_HEIGHT = 0.080
CHEEK_Z = -0.004
CLEVIS_BRIDGE_LENGTH = 0.090
CLEVIS_BRIDGE_WIDTH = 0.174
CLEVIS_BRIDGE_HEIGHT = 0.018
CLEVIS_BRIDGE_X = -0.040
CLEVIS_BRIDGE_Z = -0.040

HINGE_BARREL_LENGTH = 0.120
HINGE_BARREL_THICKNESS = 0.024
FRAME_COLLAR_LENGTH = 0.030
FRAME_COLLAR_WIDTH = 0.088
FRAME_COLLAR_HEIGHT = 0.046
FRAME_COLLAR_X = 0.022
SLEEVE_LENGTH = 0.16
SLEEVE_X = 0.120
SLEEVE_OUTER_WIDTH = 0.050
SLEEVE_OUTER_HEIGHT = 0.050
SLEEVE_INNER_WIDTH = 0.034
SLEEVE_INNER_HEIGHT = 0.034
SLEEVE_FRONT_X = SLEEVE_X + SLEEVE_LENGTH / 2.0
UPPER_SPINE_LENGTH = 0.128
UPPER_SPINE_WIDTH = 0.032
UPPER_SPINE_HEIGHT = 0.012
UPPER_SPINE_X = 0.080
UPPER_SPINE_Z = 0.020
LOWER_SPINE_LENGTH = 0.108
LOWER_SPINE_WIDTH = 0.026
LOWER_SPINE_HEIGHT = 0.010
LOWER_SPINE_X = 0.072
LOWER_SPINE_Z = -0.020

NOSE_ROD_LENGTH = 0.12
NOSE_ROD_WIDTH = 0.026
NOSE_ROD_HEIGHT = 0.026
NOSE_ROD_X = -0.060
NOSE_FRONT_TIP_LENGTH = 0.040
NOSE_FRONT_TIP_WIDTH = 0.018
NOSE_FRONT_TIP_HEIGHT = 0.018
NOSE_STOP_LENGTH = 0.010
NOSE_STOP_WIDTH = 0.044
NOSE_STOP_HEIGHT = 0.044
NOSE_EXTENSION = 0.07


def _make_base_shape() -> cq.Workplane:
    shape = (
        cq.Workplane("XY")
        .box(DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)
        .translate((0.0, 0.0, FOOT_HEIGHT + DECK_THICKNESS / 2.0))
    )

    for y in (-0.125, 0.125):
        shape = shape.union(
            cq.Workplane("XY")
            .box(BASE_LENGTH, 0.048, FOOT_HEIGHT)
            .translate((0.0, y, FOOT_HEIGHT / 2.0))
        )

    shape = shape.union(
        cq.Workplane("XY")
        .box(0.16, DECK_WIDTH, 0.026)
        .translate((0.0, 0.0, FOOT_HEIGHT + 0.013))
    )

    for y in (-RAIL_Y, RAIL_Y):
        shape = shape.union(
            cq.Workplane("XY")
            .box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)
            .translate((0.0, y, FOOT_HEIGHT + DECK_THICKNESS + RAIL_HEIGHT / 2.0))
        )
        for x in (-RAIL_LENGTH / 2.0 + 0.045, RAIL_LENGTH / 2.0 - 0.045):
            shape = shape.union(
                cq.Workplane("XY")
                .box(0.055, 0.058, 0.020)
                .translate((x, y, FOOT_HEIGHT + DECK_THICKNESS + 0.010))
            )

    return shape


def _make_carriage_shape() -> cq.Workplane:
    shape = (
        cq.Workplane("XY")
        .box(CARR_BODY_LENGTH, CARR_BODY_WIDTH, CARR_BODY_THICKNESS)
        .translate((CARR_BODY_X, 0.0, CARR_BODY_Z))
    )

    shape = shape.union(
        cq.Workplane("XY")
        .box(0.11, 0.14, 0.020)
        .translate((-0.070, 0.0, -0.028))
    )

    for y in (-RAIL_Y, RAIL_Y):
        shape = shape.union(
            cq.Workplane("XY")
            .box(RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)
            .translate((RUNNER_X, y, RUNNER_Z))
        )
        shape = shape.union(
            cq.Workplane("XY")
            .box(RUNNER_BRACKET_LENGTH, RUNNER_BRACKET_WIDTH, RUNNER_BRACKET_HEIGHT)
            .translate((-0.080, y, RUNNER_BRACKET_Z))
        )

    for y in (-CLEVIS_Y, CLEVIS_Y):
        shape = shape.union(
            cq.Workplane("XY")
            .box(CHEEK_LENGTH, CHEEK_WIDTH, CHEEK_HEIGHT)
            .translate((0.0, y, CHEEK_Z))
        )

    shape = shape.union(
        cq.Workplane("XY")
        .box(CLEVIS_BRIDGE_LENGTH, CLEVIS_BRIDGE_WIDTH, CLEVIS_BRIDGE_HEIGHT)
        .translate((CLEVIS_BRIDGE_X, 0.0, CLEVIS_BRIDGE_Z))
    )

    return shape


def _make_pivot_frame_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("XY")
        .box(HINGE_BARREL_THICKNESS, HINGE_BARREL_LENGTH, HINGE_BARREL_THICKNESS)
    )

    collar = (
        cq.Workplane("XY")
        .box(FRAME_COLLAR_LENGTH, FRAME_COLLAR_WIDTH, FRAME_COLLAR_HEIGHT)
        .translate((FRAME_COLLAR_X, 0.0, 0.0))
    )

    sleeve = (
        cq.Workplane("XY")
        .box(SLEEVE_LENGTH, SLEEVE_OUTER_WIDTH, SLEEVE_OUTER_HEIGHT)
        .translate((SLEEVE_X, 0.0, 0.0))
    )
    sleeve = sleeve.cut(
        cq.Workplane("XY")
        .box(SLEEVE_LENGTH + 0.004, SLEEVE_INNER_WIDTH, SLEEVE_INNER_HEIGHT)
        .translate((SLEEVE_X + 0.002, 0.0, 0.0))
    )

    upper_spine = (
        cq.Workplane("XY")
        .box(UPPER_SPINE_LENGTH, UPPER_SPINE_WIDTH, UPPER_SPINE_HEIGHT)
        .translate((UPPER_SPINE_X, 0.0, UPPER_SPINE_Z))
    )
    lower_spine = (
        cq.Workplane("XY")
        .box(LOWER_SPINE_LENGTH, LOWER_SPINE_WIDTH, LOWER_SPINE_HEIGHT)
        .translate((LOWER_SPINE_X, 0.0, LOWER_SPINE_Z))
    )

    return barrel.union(collar).union(sleeve).union(upper_spine).union(lower_spine)


def _make_nose_shape() -> cq.Workplane:
    rod = (
        cq.Workplane("XY")
        .box(NOSE_ROD_LENGTH, NOSE_ROD_WIDTH, NOSE_ROD_HEIGHT)
        .translate((NOSE_ROD_X, 0.0, 0.0))
    )
    collar = (
        cq.Workplane("XY")
        .box(NOSE_STOP_LENGTH, NOSE_STOP_WIDTH, NOSE_STOP_HEIGHT)
        .translate((NOSE_STOP_LENGTH / 2.0, 0.0, 0.0))
    )
    tip = (
        cq.Workplane("XY")
        .box(NOSE_FRONT_TIP_LENGTH, NOSE_FRONT_TIP_WIDTH, NOSE_FRONT_TIP_HEIGHT)
        .translate((NOSE_STOP_LENGTH + NOSE_FRONT_TIP_LENGTH / 2.0, 0.0, 0.0))
    )

    return rod.union(collar).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_rig")

    base_mat = model.material("powdercoat_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    carriage_mat = model.material("painted_slate", rgba=(0.32, 0.39, 0.46, 1.0))
    frame_mat = model.material("machined_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    nose_mat = model.material("anodized_black", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "inspection_rig_base"),
        material=base_mat,
        name="base_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "inspection_rig_carriage"),
        material=carriage_mat,
        name="carriage_body",
    )

    pivot_frame = model.part("pivot_frame")
    pivot_frame.visual(
        mesh_from_cadquery(_make_pivot_frame_shape(), "inspection_rig_pivot_frame"),
        material=frame_mat,
        name="pivot_frame_body",
    )

    nose = model.part("nose")
    nose.visual(
        mesh_from_cadquery(_make_nose_shape(), "inspection_rig_nose"),
        material=nose_mat,
        name="nose_body",
    )

    model.articulation(
        "base_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(BASE_SLIDE_START_X, 0.0, RAIL_TOP_Z + AXIS_TO_RAIL_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.30,
            lower=0.0,
            upper=BASE_SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=pivot_frame,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.4,
            lower=-0.45,
            upper=1.05,
        ),
    )

    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=pivot_frame,
        child=nose,
        origin=Origin(xyz=(SLEEVE_FRONT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.18,
            lower=0.0,
            upper=NOSE_EXTENSION,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    pivot_frame = object_model.get_part("pivot_frame")
    nose = object_model.get_part("nose")

    base_slide = object_model.get_articulation("base_slide")
    carriage_hinge = object_model.get_articulation("carriage_hinge")
    nose_slide = object_model.get_articulation("nose_slide")

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
    ctx.allow_overlap(
        carriage,
        pivot_frame,
        reason=(
            "Simplified hinge-pin packaging: the pivot frame's hinge block is "
            "represented as nested into the carriage clevis instead of modeling "
            "separate pin bores and bushings."
        ),
    )
    ctx.allow_overlap(
        nose,
        pivot_frame,
        reason=(
            "Simplified telescoping guide: the nose slider is represented as an "
            "inserted member inside the guide sleeve without explicit internal "
            "runner clearance geometry."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    with ctx.pose({base_slide: 0.18, carriage_hinge: 0.0, nose_slide: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            contact_tol=5e-4,
            name="carriage rides on the base rails",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="xy",
            min_overlap=0.16,
            name="carriage stays over the rail bed",
        )
        ctx.expect_overlap(
            pivot_frame,
            carriage,
            axes="yz",
            min_overlap=0.05,
            name="pivot frame stays inside the carriage support span",
        )
        ctx.expect_within(
            nose,
            pivot_frame,
            axes="yz",
            margin=0.02,
            name="retracted nose remains laterally guided by the sleeve",
        )
        ctx.expect_overlap(
            nose,
            pivot_frame,
            axes="x",
            min_overlap=0.10,
            name="retracted nose keeps deep telescoping engagement",
        )

    with ctx.pose({base_slide: 0.18, carriage_hinge: 0.0, nose_slide: NOSE_EXTENSION}):
        ctx.expect_overlap(
            nose,
            pivot_frame,
            axes="x",
            min_overlap=0.04,
            name="extended nose still retains sleeve overlap",
        )

    with ctx.pose({base_slide: 0.0, carriage_hinge: 0.0, nose_slide: 0.0}):
        carriage_start = ctx.part_world_position(carriage)
    with ctx.pose({base_slide: 0.30, carriage_hinge: 0.0, nose_slide: 0.0}):
        carriage_shifted = ctx.part_world_position(carriage)

    slide_ok = (
        carriage_start is not None
        and carriage_shifted is not None
        and carriage_shifted[0] > carriage_start[0] + 0.29
        and abs(carriage_shifted[1] - carriage_start[1]) < 1e-5
        and abs(carriage_shifted[2] - carriage_start[2]) < 1e-5
    )
    ctx.check(
        "base slide translates the carriage along +x",
        slide_ok,
        details=(
            f"start={carriage_start}, shifted={carriage_shifted}"
            if not slide_ok
            else ""
        ),
    )

    with ctx.pose({base_slide: 0.18, carriage_hinge: 0.0, nose_slide: 0.0}):
        nose_level = ctx.part_world_position(nose)
    with ctx.pose({base_slide: 0.18, carriage_hinge: 0.85, nose_slide: 0.0}):
        nose_lifted = ctx.part_world_position(nose)

    hinge_ok = (
        nose_level is not None
        and nose_lifted is not None
        and nose_lifted[2] > nose_level[2] + 0.12
    )
    ctx.check(
        "hinge lifts the pivoted nose upward for positive rotation",
        hinge_ok,
        details=(
            f"level={nose_level}, lifted={nose_lifted}"
            if not hinge_ok
            else ""
        ),
    )

    with ctx.pose({base_slide: 0.18, carriage_hinge: 0.35, nose_slide: 0.0}):
        nose_retracted = ctx.part_world_position(nose)
    with ctx.pose({base_slide: 0.18, carriage_hinge: 0.35, nose_slide: 0.06}):
        nose_extended = ctx.part_world_position(nose)

    extension_dist = (
        math.dist(nose_retracted, nose_extended)
        if nose_retracted is not None and nose_extended is not None
        else None
    )
    extension_ok = extension_dist is not None and 0.055 <= extension_dist <= 0.065
    ctx.check(
        "terminal slider extends the nose by the commanded stroke",
        extension_ok,
        details=(
            f"retracted={nose_retracted}, extended={nose_extended}, dist={extension_dist}"
            if not extension_ok
            else ""
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
