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


BASE_LENGTH = 1.08
BASE_WIDTH = 0.82
RAIL_CENTER_X = 0.32
FOOT_HEIGHT = 0.04
RAIL_BODY_WIDTH = 0.09
RAIL_BODY_HEIGHT = 0.14
RAIL_CAP_WIDTH = 0.038
RAIL_CAP_HEIGHT = 0.012
CROSSMEMBER_WIDTH_Y = 0.09
CROSSMEMBER_HEIGHT = 0.08
CROSSMEMBER_CENTER_X = 0.57
CROSSMEMBER_Z = FOOT_HEIGHT + CROSSMEMBER_HEIGHT / 2.0

BEAM_LENGTH = 0.66
BEAM_DEPTH = 0.09
BEAM_HEIGHT = 0.10
BEAM_RAIL_WIDTH = 0.026
BEAM_RAIL_HEIGHT = 0.010
BEAM_CENTER_Z = 0.255
TRUCK_WIDTH = 0.15
TRUCK_LENGTH = 0.18
TRUCK_HEIGHT = 0.09
TRUCK_CENTER_Z = -0.018

RUNNER_LENGTH = 0.18
RUNNER_DEPTH = 0.18
RUNNER_BASE_THICKNESS = 0.028
RUNNER_BODY_HEIGHT = 0.090
RUNNER_BODY_DEPTH = 0.11
RUNNER_PAYLOAD_BLOCK_DEPTH = 0.07
RUNNER_PAYLOAD_BLOCK_HEIGHT = 0.11

BEAM_TRAVEL = 0.62
RUNNER_TRAVEL = 0.44


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _build_base_shape() -> cq.Workplane:
    frame = _box(
        (RAIL_BODY_WIDTH, BASE_LENGTH, RAIL_BODY_HEIGHT),
        (-RAIL_CENTER_X, 0.0, FOOT_HEIGHT + RAIL_BODY_HEIGHT / 2.0),
    ).union(
        _box(
            (RAIL_BODY_WIDTH, BASE_LENGTH, RAIL_BODY_HEIGHT),
            (RAIL_CENTER_X, 0.0, FOOT_HEIGHT + RAIL_BODY_HEIGHT / 2.0),
        )
    )

    frame = frame.union(
        _box(
            (RAIL_CAP_WIDTH, BASE_LENGTH - 0.08, RAIL_CAP_HEIGHT),
            (-RAIL_CENTER_X, 0.0, FOOT_HEIGHT + RAIL_BODY_HEIGHT + RAIL_CAP_HEIGHT / 2.0),
        )
    ).union(
        _box(
            (RAIL_CAP_WIDTH, BASE_LENGTH - 0.08, RAIL_CAP_HEIGHT),
            (RAIL_CENTER_X, 0.0, FOOT_HEIGHT + RAIL_BODY_HEIGHT + RAIL_CAP_HEIGHT / 2.0),
        )
    )

    for y in (-0.44, 0.0, 0.44):
        frame = frame.union(
            _box(
                (CROSSMEMBER_CENTER_X * 2.0, CROSSMEMBER_WIDTH_Y, CROSSMEMBER_HEIGHT),
                (0.0, y, CROSSMEMBER_Z),
            )
        )

    foot_size = (0.14, 0.11, FOOT_HEIGHT)
    for x in (-RAIL_CENTER_X, RAIL_CENTER_X):
        for y in (-0.48, 0.48):
            frame = frame.union(_box(foot_size, (x, y, FOOT_HEIGHT / 2.0)))

    return frame


def _build_beam_shape() -> cq.Workplane:
    beam = _box((BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT), (0.0, 0.0, 0.0))
    beam = beam.union(
        _box(
            (BEAM_LENGTH - 0.14, BEAM_RAIL_WIDTH, BEAM_RAIL_HEIGHT),
            (0.0, 0.0, BEAM_HEIGHT / 2.0 + BEAM_RAIL_HEIGHT / 2.0),
        )
    )

    for x in (-RAIL_CENTER_X, RAIL_CENTER_X):
        beam = beam.union(_box((TRUCK_WIDTH, TRUCK_LENGTH, TRUCK_HEIGHT), (x, 0.0, TRUCK_CENTER_Z)))

    return beam


def _build_runner_shape() -> cq.Workplane:
    runner = _box(
        (RUNNER_LENGTH, RUNNER_DEPTH, RUNNER_BASE_THICKNESS),
        (0.0, 0.0, RUNNER_BASE_THICKNESS / 2.0),
    )
    runner = runner.union(
        _box(
            (0.12, RUNNER_BODY_DEPTH, RUNNER_BODY_HEIGHT),
            (
                0.0,
                0.0,
                RUNNER_BASE_THICKNESS + RUNNER_BODY_HEIGHT / 2.0,
            ),
        )
    )
    runner = runner.union(
        _box(
            (0.08, RUNNER_PAYLOAD_BLOCK_DEPTH, RUNNER_PAYLOAD_BLOCK_HEIGHT),
            (
                0.0,
                0.105,
                RUNNER_BASE_THICKNESS + RUNNER_PAYLOAD_BLOCK_HEIGHT / 2.0,
            ),
        )
    )
    return runner


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_lite_dual_rail_gantry")

    model.material("base_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("beam_aluminum", rgba=(0.74, 0.77, 0.79, 1.0))
    model.material("runner_carriage", rgba=(0.50, 0.54, 0.58, 1.0))

    base = model.part("base_chassis")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base_chassis"),
        material="base_steel",
        name="base_frame",
    )

    beam = model.part("portal_beam")
    beam.visual(
        mesh_from_cadquery(_build_beam_shape(), "portal_beam"),
        material="beam_aluminum",
        name="beam_assembly",
    )

    runner = model.part("central_runner")
    runner.visual(
        mesh_from_cadquery(_build_runner_shape(), "central_runner"),
        material="runner_carriage",
        name="runner_body",
    )

    model.articulation(
        "base_to_beam",
        ArticulationType.PRISMATIC,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, BEAM_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BEAM_TRAVEL / 2.0,
            upper=BEAM_TRAVEL / 2.0,
            effort=180.0,
            velocity=0.50,
        ),
    )
    model.articulation(
        "beam_to_runner",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=runner,
        origin=Origin(xyz=(0.0, 0.0, BEAM_HEIGHT / 2.0 + BEAM_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-RUNNER_TRAVEL / 2.0,
            upper=RUNNER_TRAVEL / 2.0,
            effort=80.0,
            velocity=0.60,
        ),
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
    base = object_model.get_part("base_chassis")
    beam = object_model.get_part("portal_beam")
    runner = object_model.get_part("central_runner")
    base_to_beam = object_model.get_articulation("base_to_beam")
    beam_to_runner = object_model.get_articulation("beam_to_runner")

    ctx.check("base chassis exists", base is not None)
    ctx.check("portal beam exists", beam is not None)
    ctx.check("central runner exists", runner is not None)

    beam_rest = ctx.part_world_position(beam)
    with ctx.pose({base_to_beam: BEAM_TRAVEL / 2.0}):
        beam_forward = ctx.part_world_position(beam)

    ctx.check(
        "beam translates along +Y rail direction",
        beam_rest is not None
        and beam_forward is not None
        and beam_forward[1] > beam_rest[1] + 0.20
        and abs(beam_forward[0] - beam_rest[0]) < 1e-6
        and abs(beam_forward[2] - beam_rest[2]) < 1e-6,
        details=f"rest={beam_rest}, forward={beam_forward}",
    )

    runner_rest = ctx.part_world_position(runner)
    with ctx.pose({beam_to_runner: RUNNER_TRAVEL / 2.0}):
        runner_outboard = ctx.part_world_position(runner)

    ctx.check(
        "runner translates along +X beam direction",
        runner_rest is not None
        and runner_outboard is not None
        and runner_outboard[0] > runner_rest[0] + 0.14
        and abs(runner_outboard[1] - runner_rest[1]) < 1e-6
        and abs(runner_outboard[2] - runner_rest[2]) < 1e-6,
        details=f"rest={runner_rest}, outboard={runner_outboard}",
    )

    ctx.expect_gap(
        runner,
        beam,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="runner seats on top of beam without penetrating it",
    )
    ctx.expect_contact(
        beam,
        base,
        contact_tol=0.002,
        name="beam trucks stay in contact with base rails at rest",
    )

    with ctx.pose({base_to_beam: BEAM_TRAVEL / 2.0, beam_to_runner: RUNNER_TRAVEL / 2.0}):
        ctx.expect_gap(
            runner,
            beam,
            axis="z",
            min_gap=0.0,
            max_gap=0.002,
            name="runner stays seated on the beam at full travel",
        )
        ctx.expect_overlap(
            runner,
            beam,
            axes="x",
            min_overlap=0.10,
            name="runner retains meaningful overlap with the beam at full travel",
        )
        ctx.expect_contact(
            beam,
            base,
            contact_tol=0.002,
            name="beam trucks stay in contact with base rails at forward travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
