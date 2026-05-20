from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


BODY_BASE_LENGTH = 0.220
BODY_BASE_WIDTH = 0.180
BODY_BASE_HEIGHT = 0.022
BODY_HOUSING_LENGTH = 0.180
BODY_HOUSING_WIDTH = 0.140
BODY_HOUSING_HEIGHT = 0.105
BODY_FRONT_FACE_X = BODY_HOUSING_LENGTH / 2.0
SPINDLE_AXIS_Z = BODY_BASE_HEIGHT + 0.063

BEARING_BOSS_RADIUS = 0.036
BEARING_BOSS_LENGTH = 0.022
NOSE_RADIUS = 0.029
NOSE_LENGTH = 0.012
BORE_RADIUS = 0.019
SPINDLE_SUPPORT_X = BODY_FRONT_FACE_X + BEARING_BOSS_LENGTH + NOSE_LENGTH

SHAFT_RADIUS = 0.016
HUB_RADIUS = 0.028
HUB_LENGTH = 0.020
SHAFT_LENGTH = 0.110
OUTPUT_PLATE_RADIUS = 0.046
OUTPUT_PLATE_THICKNESS = 0.012
OUTPUT_PLATE_X = 0.098
PILOT_RADIUS = 0.018
PILOT_LENGTH = 0.006
PILOT_X = OUTPUT_PLATE_X + OUTPUT_PLATE_THICKNESS
SPINDLE_TOTAL_LENGTH = PILOT_X + PILOT_LENGTH
OUTPUT_BOLT_CIRCLE = 0.028
OUTPUT_BOLT_RADIUS = 0.0042


def _make_body_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BODY_BASE_LENGTH,
        BODY_BASE_WIDTH,
        BODY_BASE_HEIGHT,
        centered=(True, True, False),
    )

    housing = (
        cq.Workplane("XY")
        .box(
            BODY_HOUSING_LENGTH,
            BODY_HOUSING_WIDTH,
            BODY_HOUSING_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BODY_BASE_HEIGHT))
        .edges("|Z")
        .fillet(0.010)
        .faces(">Z")
        .edges()
        .fillet(0.006)
    )

    front_boss = (
        cq.Workplane("YZ")
        .circle(BEARING_BOSS_RADIUS)
        .extrude(BEARING_BOSS_LENGTH)
        .translate((BODY_FRONT_FACE_X, 0.0, SPINDLE_AXIS_Z))
    )
    nose = (
        cq.Workplane("YZ")
        .circle(NOSE_RADIUS)
        .extrude(NOSE_LENGTH)
        .translate((BODY_FRONT_FACE_X + BEARING_BOSS_LENGTH, 0.0, SPINDLE_AXIS_Z))
    )

    body = base.union(housing).union(front_boss).union(nose)

    top_panel_recess = (
        cq.Workplane("XY")
        .rect(0.126, 0.090)
        .extrude(0.0025)
        .translate((0.0, 0.0, BODY_BASE_HEIGHT + BODY_HOUSING_HEIGHT - 0.0025))
    )
    side_relief = (
        cq.Workplane("XZ")
        .rect(0.090, 0.050)
        .extrude(0.006, both=True)
        .translate((-0.020, BODY_HOUSING_WIDTH / 2.0 - 0.003, BODY_BASE_HEIGHT + 0.050))
    )
    opposite_side_relief = side_relief.mirror("XZ")
    spindle_bore = (
        cq.Workplane("YZ")
        .circle(BORE_RADIUS)
        .extrude(0.046)
        .translate((BODY_FRONT_FACE_X - 0.010, 0.0, SPINDLE_AXIS_Z))
    )

    return body.cut(top_panel_recess).cut(side_relief).cut(opposite_side_relief).cut(spindle_bore)


def _make_shaft_shape() -> cq.Workplane:
    shaft = cq.Workplane("YZ").circle(SHAFT_RADIUS).extrude(SHAFT_LENGTH)
    hub = cq.Workplane("YZ").circle(HUB_RADIUS).extrude(HUB_LENGTH)
    output_plate = (
        cq.Workplane("YZ")
        .circle(OUTPUT_PLATE_RADIUS)
        .extrude(OUTPUT_PLATE_THICKNESS)
        .translate((OUTPUT_PLATE_X, 0.0, 0.0))
    )
    pilot = (
        cq.Workplane("YZ")
        .circle(PILOT_RADIUS)
        .extrude(PILOT_LENGTH)
        .translate((PILOT_X, 0.0, 0.0))
    )

    spindle = shaft.union(hub).union(output_plate).union(pilot)

    for i in range(4):
        angle = 0.5 * pi * i
        hole_y = OUTPUT_BOLT_CIRCLE * cos(angle)
        hole_z = OUTPUT_BOLT_CIRCLE * sin(angle)
        bolt_hole = (
            cq.Workplane("YZ")
            .circle(OUTPUT_BOLT_RADIUS)
            .extrude(OUTPUT_PLATE_THICKNESS + PILOT_LENGTH + 0.004)
            .translate((OUTPUT_PLATE_X - 0.002, hole_y, hole_z))
        )
        spindle = spindle.cut(bolt_hole)

    return spindle


def _close_tuple(a: tuple[float, float, float], b: tuple[float, float, float], tol: float) -> bool:
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="production_spindle_unit")

    model.material("housing_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("machined_face", rgba=(0.84, 0.86, 0.88, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "spindle_body"),
        material="housing_gray",
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_BASE_LENGTH, BODY_BASE_WIDTH, BODY_BASE_HEIGHT + BODY_HOUSING_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (BODY_BASE_HEIGHT + BODY_HOUSING_HEIGHT) / 2.0)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        mesh_from_cadquery(_make_shaft_shape(), "spindle_shaft"),
        material="machined_face",
        name="shaft_assembly",
    )
    shaft.inertial = Inertial.from_geometry(
        Box((SPINDLE_TOTAL_LENGTH, OUTPUT_PLATE_RADIUS * 2.0, OUTPUT_PLATE_RADIUS * 2.0)),
        mass=2.6,
        origin=Origin(
            xyz=(
                SPINDLE_TOTAL_LENGTH / 2.0,
                0.0,
                0.0,
            )
        ),
    )

    model.articulation(
        "body_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=shaft,
        origin=Origin(xyz=(SPINDLE_SUPPORT_X, 0.0, SPINDLE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=18.0),
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
    body = object_model.get_part("body")
    shaft = object_model.get_part("shaft")
    spindle_joint = object_model.get_articulation("body_to_shaft")

    ctx.check("body exists", body is not None, details="missing body part")
    ctx.check("shaft exists", shaft is not None, details="missing shaft part")
    ctx.check(
        "spindle uses a continuous joint",
        spindle_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spindle_joint.articulation_type}",
    )
    ctx.check(
        "spindle axis is along the shaft centerline",
        _close_tuple(spindle_joint.axis, (1.0, 0.0, 0.0), 1e-9),
        details=f"axis={spindle_joint.axis}",
    )

    with ctx.pose({spindle_joint: 0.0}):
        ctx.expect_gap(
            shaft,
            body,
            axis="x",
            max_gap=0.0005,
            max_penetration=1e-6,
            name="shaft starts at the supported nose face",
        )
        ctx.expect_overlap(
            shaft,
            body,
            axes="yz",
            min_overlap=0.030,
            name="shaft stays aligned within the body support envelope",
        )

        rest_pos = ctx.part_world_position(shaft)

    with ctx.pose({spindle_joint: 1.4}):
        turned_pos = ctx.part_world_position(shaft)

    ctx.check(
        "shaft rotates in place about its own centerline",
        rest_pos is not None and turned_pos is not None and _close_tuple(rest_pos, turned_pos, 1e-9),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
