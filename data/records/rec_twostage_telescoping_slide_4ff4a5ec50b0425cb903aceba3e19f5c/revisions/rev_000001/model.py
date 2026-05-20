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


PLATE_LENGTH = 0.280
PLATE_WIDTH = 0.120
PLATE_THICKNESS = 0.005

GUIDE_OFFSET_X = 0.020
GUIDE_LENGTH = 0.240
GUIDE_OUTER_WIDTH = 0.062
GUIDE_OUTER_HEIGHT = 0.024
GUIDE_WALL = 0.004
GUIDE_FLOOR = 0.004
GUIDE_REAR_STOP = 0.012

RUNNER_LENGTH = 0.190
RUNNER_WEB_WIDTH = 0.038
RUNNER_WEB_HEIGHT = 0.009
RUNNER_DECK_WIDTH = 0.052
RUNNER_DECK_THICKNESS = 0.004
RUNNER_TOTAL_HEIGHT = RUNNER_WEB_HEIGHT + RUNNER_DECK_THICKNESS
RUNNER_CLEARANCE_Z = 0.0

RUNNER_TRAVEL = 0.110


def _base_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS, centered=(False, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.095, -0.040),
                (-0.095, 0.040),
                (0.095, -0.040),
                (0.095, 0.040),
            ]
        )
        .hole(0.006)
    )
    return plate


def _guide_body_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(
        GUIDE_LENGTH,
        GUIDE_OUTER_WIDTH,
        GUIDE_OUTER_HEIGHT,
        centered=(False, True, False),
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            GUIDE_LENGTH - GUIDE_REAR_STOP,
            GUIDE_OUTER_WIDTH - 2.0 * GUIDE_WALL,
            GUIDE_OUTER_HEIGHT,
            centered=(False, True, False),
        )
        .translate((GUIDE_REAR_STOP, 0.0, GUIDE_FLOOR))
    )
    return shell.cut(cavity)


def _runner_shape() -> cq.Workplane:
    web = cq.Workplane("XY").box(
        RUNNER_LENGTH,
        RUNNER_WEB_WIDTH,
        RUNNER_WEB_HEIGHT,
        centered=(False, True, False),
    )
    deck = (
        cq.Workplane("XY")
        .box(
            RUNNER_LENGTH,
            RUNNER_DECK_WIDTH,
            RUNNER_DECK_THICKNESS,
            centered=(False, True, False),
        )
        .translate((0.0, 0.0, RUNNER_WEB_HEIGHT))
    )
    runner = web.union(deck)
    return runner.translate((-RUNNER_LENGTH, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_telescoping_slide")

    model.material("body_finish", rgba=(0.44, 0.47, 0.50, 1.0))
    model.material("runner_finish", rgba=(0.76, 0.78, 0.80, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_base_plate_shape(), "body_base_plate"),
        origin=Origin(),
        material="body_finish",
        name="base_plate",
    )
    body.visual(
        mesh_from_cadquery(_guide_body_shape(), "body_guide_body"),
        origin=Origin(xyz=(GUIDE_OFFSET_X, 0.0, PLATE_THICKNESS)),
        material="body_finish",
        name="guide_body",
    )
    body.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, GUIDE_OUTER_HEIGHT + PLATE_THICKNESS)),
        mass=1.15,
        origin=Origin(
            xyz=(
                PLATE_LENGTH / 2.0,
                0.0,
                (GUIDE_OUTER_HEIGHT + PLATE_THICKNESS) / 2.0,
            )
        ),
    )

    runner = model.part("runner")
    runner.visual(
        mesh_from_cadquery(_runner_shape(), "slide_runner"),
        origin=Origin(),
        material="runner_finish",
        name="runner_shell",
    )
    runner.inertial = Inertial.from_geometry(
        Box((RUNNER_LENGTH, RUNNER_DECK_WIDTH, RUNNER_TOTAL_HEIGHT)),
        mass=0.24,
        origin=Origin(xyz=(-RUNNER_LENGTH / 2.0, 0.0, RUNNER_TOTAL_HEIGHT / 2.0)),
    )

    model.articulation(
        "body_to_runner",
        ArticulationType.PRISMATIC,
        parent=body,
        child=runner,
        origin=Origin(
            xyz=(
                GUIDE_OFFSET_X + GUIDE_LENGTH,
                0.0,
                PLATE_THICKNESS + GUIDE_FLOOR + RUNNER_CLEARANCE_Z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=RUNNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    runner = object_model.get_part("runner")
    slide = object_model.get_articulation("body_to_runner")

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

    ctx.expect_within(
        runner,
        body,
        axes="yz",
        inner_elem="runner_shell",
        outer_elem="guide_body",
        name="runner stays centered inside guide body at rest",
    )
    ctx.expect_overlap(
        runner,
        body,
        axes="x",
        elem_a="runner_shell",
        elem_b="guide_body",
        min_overlap=0.17,
        name="runner has substantial retained overlap when retracted",
    )

    rest_pos = ctx.part_world_position(runner)
    with ctx.pose({slide: RUNNER_TRAVEL}):
        ctx.expect_within(
            runner,
            body,
            axes="yz",
            inner_elem="runner_shell",
            outer_elem="guide_body",
            name="runner stays centered inside guide body when extended",
        )
        ctx.expect_overlap(
            runner,
            body,
            axes="x",
            elem_a="runner_shell",
            elem_b="guide_body",
            min_overlap=0.07,
            name="runner still retains visible overlap when extended",
        )
        extended_pos = ctx.part_world_position(runner)

    ctx.check(
        "runner extends along +X",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.09,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
