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


BODY_LENGTH = 0.42
BODY_WIDTH = 0.22
BODY_BASE_HEIGHT = 0.024
BODY_CORE_LENGTH = 0.36
BODY_CORE_WIDTH = 0.14
BODY_CORE_HEIGHT = 0.042
BODY_RAIL_LENGTH = 0.40
BODY_RAIL_WIDTH = 0.024
BODY_RAIL_HEIGHT = 0.012
BODY_RAIL_Y = 0.072
BODY_RAIL_TOP_Z = BODY_BASE_HEIGHT + BODY_CORE_HEIGHT + BODY_RAIL_HEIGHT
BODY_REAR_HOUSING_LENGTH = 0.070
BODY_REAR_HOUSING_WIDTH = 0.180
BODY_REAR_HOUSING_HEIGHT = 0.060

MAIN_LENGTH = 0.180
MAIN_WIDTH = 0.200
MAIN_TOP_HEIGHT = 0.018
MAIN_RUNNER_LENGTH = 0.160
MAIN_RUNNER_WIDTH = BODY_RAIL_WIDTH
MAIN_RUNNER_HEIGHT = 0.014
MAIN_RUNNER_Y = BODY_RAIL_Y
MAIN_STAGE_RAIL_LENGTH = 0.160
MAIN_STAGE_RAIL_WIDTH = 0.018
MAIN_STAGE_RAIL_HEIGHT = 0.010
MAIN_STAGE_RAIL_Y = 0.045
MAIN_STAGE_RAIL_TOP_Z = MAIN_RUNNER_HEIGHT + MAIN_TOP_HEIGHT + MAIN_STAGE_RAIL_HEIGHT
MAIN_TRAVEL = 0.140

TERMINAL_LENGTH = 0.100
TERMINAL_WIDTH = 0.140
TERMINAL_TOP_HEIGHT = 0.014
TERMINAL_RUNNER_LENGTH = 0.085
TERMINAL_RUNNER_WIDTH = MAIN_STAGE_RAIL_WIDTH
TERMINAL_RUNNER_HEIGHT = 0.012
TERMINAL_RUNNER_Y = MAIN_STAGE_RAIL_Y
TERMINAL_TRAVEL = 0.060


def _box(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .translate((x, y, z))
    )


def _rounded_box(
    length: float,
    width: float,
    height: float,
    *,
    radius: float,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return _box(length, width, height, x=x, y=y, z=z).edges("|Z").fillet(radius)


def _body_shell_shape() -> cq.Workplane:
    rear_x = -BODY_LENGTH / 2.0 + BODY_REAR_HOUSING_LENGTH / 2.0 + 0.010
    shell = _rounded_box(BODY_LENGTH, BODY_WIDTH, BODY_BASE_HEIGHT, radius=0.006)
    shell = shell.union(
        _rounded_box(
            BODY_CORE_LENGTH,
            BODY_CORE_WIDTH,
            BODY_CORE_HEIGHT,
            radius=0.006,
            z=BODY_BASE_HEIGHT,
        )
    )
    shell = shell.union(
        _rounded_box(
            BODY_REAR_HOUSING_LENGTH,
            BODY_REAR_HOUSING_WIDTH,
            BODY_REAR_HOUSING_HEIGHT,
            radius=0.010,
            x=rear_x,
            z=BODY_BASE_HEIGHT,
        )
    )
    mount_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.160, -0.080),
                (-0.160, 0.080),
                (0.160, -0.080),
                (0.160, 0.080),
            ]
        )
        .circle(0.007)
        .extrude(BODY_BASE_HEIGHT + 0.003)
        .translate((0.0, 0.0, -0.0015))
    )
    return shell.cut(mount_holes)


def _body_rails_shape() -> cq.Workplane:
    rail_z = BODY_BASE_HEIGHT + BODY_CORE_HEIGHT
    span_width = 2.0 * BODY_RAIL_Y + BODY_RAIL_WIDTH
    rails = _box(BODY_RAIL_LENGTH, BODY_RAIL_WIDTH, BODY_RAIL_HEIGHT, y=BODY_RAIL_Y, z=rail_z)
    rails = rails.union(
        _box(BODY_RAIL_LENGTH, BODY_RAIL_WIDTH, BODY_RAIL_HEIGHT, y=-BODY_RAIL_Y, z=rail_z)
    )
    rails = rails.union(
        _box(
            0.026,
            span_width,
            BODY_RAIL_HEIGHT,
            x=-BODY_RAIL_LENGTH / 2.0 + 0.013,
            z=rail_z,
        )
    )
    return rails


def _main_carriage_shape() -> cq.Workplane:
    body = _rounded_box(MAIN_LENGTH, MAIN_WIDTH, MAIN_TOP_HEIGHT, radius=0.004, z=MAIN_RUNNER_HEIGHT)
    body = body.union(
        _rounded_box(
            0.135,
            0.080,
            0.010,
            radius=0.003,
            z=MAIN_RUNNER_HEIGHT + MAIN_TOP_HEIGHT,
        )
    )
    body = body.union(_rounded_box(0.155, 0.022, 0.020, radius=0.003, y=0.089, z=0.010))
    body = body.union(_rounded_box(0.155, 0.022, 0.020, radius=0.003, y=-0.089, z=0.010))
    top_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.050, -0.050),
                (-0.050, 0.050),
                (0.050, -0.050),
                (0.050, 0.050),
            ]
        )
        .circle(0.005)
        .extrude(MAIN_TOP_HEIGHT + 0.002)
        .translate((0.0, 0.0, MAIN_RUNNER_HEIGHT - 0.001))
    )
    return body.cut(top_holes)


def _main_lower_runners_shape() -> cq.Workplane:
    span_width = 2.0 * MAIN_RUNNER_Y + MAIN_RUNNER_WIDTH
    runners = _box(MAIN_RUNNER_LENGTH, MAIN_RUNNER_WIDTH, MAIN_RUNNER_HEIGHT, y=MAIN_RUNNER_Y)
    runners = runners.union(
        _box(MAIN_RUNNER_LENGTH, MAIN_RUNNER_WIDTH, MAIN_RUNNER_HEIGHT, y=-MAIN_RUNNER_Y)
    )
    runners = runners.union(_box(0.024, span_width, MAIN_RUNNER_HEIGHT, x=-0.046, z=0.006))
    return runners


def _main_stage_rails_shape() -> cq.Workplane:
    rail_z = MAIN_RUNNER_HEIGHT + MAIN_TOP_HEIGHT
    span_width = 2.0 * MAIN_STAGE_RAIL_Y + MAIN_STAGE_RAIL_WIDTH
    rails = _box(MAIN_STAGE_RAIL_LENGTH, MAIN_STAGE_RAIL_WIDTH, MAIN_STAGE_RAIL_HEIGHT, y=MAIN_STAGE_RAIL_Y, z=rail_z)
    rails = rails.union(
        _box(MAIN_STAGE_RAIL_LENGTH, MAIN_STAGE_RAIL_WIDTH, MAIN_STAGE_RAIL_HEIGHT, y=-MAIN_STAGE_RAIL_Y, z=rail_z)
    )
    rails = rails.union(_box(0.022, span_width, MAIN_STAGE_RAIL_HEIGHT, x=-0.042, z=rail_z))
    return rails


def _terminal_carriage_shape() -> cq.Workplane:
    body = _rounded_box(
        TERMINAL_LENGTH,
        TERMINAL_WIDTH,
        TERMINAL_TOP_HEIGHT,
        radius=0.0035,
        z=TERMINAL_RUNNER_HEIGHT,
    )
    body = body.union(
        _rounded_box(
            0.060,
            0.060,
            0.010,
            radius=0.0025,
            z=TERMINAL_RUNNER_HEIGHT + TERMINAL_TOP_HEIGHT,
        )
    )
    top_holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.022, 0.0), (0.022, 0.0)])
        .circle(0.004)
        .extrude(TERMINAL_TOP_HEIGHT + 0.002)
        .translate((0.0, 0.0, TERMINAL_RUNNER_HEIGHT - 0.001))
    )
    return body.cut(top_holes)


def _terminal_runners_shape() -> cq.Workplane:
    span_width = 2.0 * TERMINAL_RUNNER_Y + TERMINAL_RUNNER_WIDTH
    runners = _box(
        TERMINAL_RUNNER_LENGTH,
        TERMINAL_RUNNER_WIDTH,
        TERMINAL_RUNNER_HEIGHT,
        y=TERMINAL_RUNNER_Y,
    )
    runners = runners.union(
        _box(
            TERMINAL_RUNNER_LENGTH,
            TERMINAL_RUNNER_WIDTH,
            TERMINAL_RUNNER_HEIGHT,
            y=-TERMINAL_RUNNER_Y,
        )
    )
    runners = runners.union(_box(0.020, span_width, TERMINAL_RUNNER_HEIGHT, x=-0.024, z=0.005))
    return runners


def _add_mesh_visual(part, shape: cq.Workplane, mesh_name: str, *, material: str, visual_name: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_two_stage_axis")

    model.material("body_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("guide_steel", rgba=(0.66, 0.69, 0.72, 1.0))
    model.material("carriage_alloy", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("terminal_alloy", rgba=(0.73, 0.76, 0.79, 1.0))

    body = model.part("body")
    main_carriage = model.part("main_carriage")
    terminal_carriage = model.part("terminal_carriage")

    _add_mesh_visual(
        body,
        _body_shell_shape(),
        "body_shell",
        material="body_graphite",
        visual_name="body_shell",
    )
    _add_mesh_visual(
        body,
        _body_rails_shape(),
        "body_rails",
        material="guide_steel",
        visual_name="body_rails",
    )

    _add_mesh_visual(
        main_carriage,
        _main_carriage_shape(),
        "main_carriage_body",
        material="carriage_alloy",
        visual_name="carriage_body",
    )
    _add_mesh_visual(
        main_carriage,
        _main_lower_runners_shape(),
        "main_lower_runners",
        material="guide_steel",
        visual_name="lower_runners",
    )
    _add_mesh_visual(
        main_carriage,
        _main_stage_rails_shape(),
        "main_stage_rails",
        material="guide_steel",
        visual_name="stage_rails",
    )

    _add_mesh_visual(
        terminal_carriage,
        _terminal_carriage_shape(),
        "terminal_carriage_body",
        material="terminal_alloy",
        visual_name="terminal_body",
    )
    _add_mesh_visual(
        terminal_carriage,
        _terminal_runners_shape(),
        "terminal_runners",
        material="guide_steel",
        visual_name="lower_runners",
    )

    model.articulation(
        "body_to_main",
        ArticulationType.PRISMATIC,
        parent=body,
        child=main_carriage,
        origin=Origin(xyz=(0.0, 0.0, BODY_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MAIN_TRAVEL,
            effort=500.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "main_to_terminal",
        ArticulationType.PRISMATIC,
        parent=main_carriage,
        child=terminal_carriage,
        origin=Origin(xyz=(0.0, 0.0, MAIN_STAGE_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TERMINAL_TRAVEL,
            effort=250.0,
            velocity=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    main_carriage = object_model.get_part("main_carriage")
    terminal_carriage = object_model.get_part("terminal_carriage")
    body_to_main = object_model.get_articulation("body_to_main")
    main_to_terminal = object_model.get_articulation("main_to_terminal")

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
        main_carriage,
        body,
        axis="z",
        positive_elem="lower_runners",
        negative_elem="body_rails",
        max_gap=0.0005,
        max_penetration=0.0005,
        name="main carriage runners seat on body rails",
    )
    ctx.expect_within(
        main_carriage,
        body,
        axes="y",
        inner_elem="lower_runners",
        outer_elem="body_rails",
        margin=0.001,
        name="main carriage stays centered across body rails",
    )
    ctx.expect_overlap(
        main_carriage,
        body,
        axes="x",
        elem_a="lower_runners",
        elem_b="body_rails",
        min_overlap=0.150,
        name="main carriage starts with long rail engagement",
    )

    ctx.expect_gap(
        terminal_carriage,
        main_carriage,
        axis="z",
        positive_elem="lower_runners",
        negative_elem="stage_rails",
        max_gap=0.0005,
        max_penetration=0.0005,
        name="terminal carriage runners seat on upper rails",
    )
    ctx.expect_within(
        terminal_carriage,
        main_carriage,
        axes="y",
        inner_elem="lower_runners",
        outer_elem="stage_rails",
        margin=0.001,
        name="terminal carriage stays centered across upper rails",
    )
    ctx.expect_overlap(
        terminal_carriage,
        main_carriage,
        axes="x",
        elem_a="lower_runners",
        elem_b="stage_rails",
        min_overlap=0.080,
        name="terminal carriage starts with deep upper-stage engagement",
    )

    main_rest = ctx.part_world_position(main_carriage)
    with ctx.pose({body_to_main: MAIN_TRAVEL}):
        ctx.expect_within(
            main_carriage,
            body,
            axes="y",
            inner_elem="lower_runners",
            outer_elem="body_rails",
            margin=0.001,
            name="extended main carriage stays centered on body rails",
        )
        ctx.expect_overlap(
            main_carriage,
            body,
            axes="x",
            elem_a="lower_runners",
            elem_b="body_rails",
            min_overlap=0.130,
            name="extended main carriage still retains body-rail insertion",
        )
        main_extended = ctx.part_world_position(main_carriage)

    terminal_rest = ctx.part_world_position(terminal_carriage)
    with ctx.pose({main_to_terminal: TERMINAL_TRAVEL}):
        ctx.expect_within(
            terminal_carriage,
            main_carriage,
            axes="y",
            inner_elem="lower_runners",
            outer_elem="stage_rails",
            margin=0.001,
            name="extended terminal carriage stays centered on upper rails",
        )
        ctx.expect_overlap(
            terminal_carriage,
            main_carriage,
            axes="x",
            elem_a="lower_runners",
            elem_b="stage_rails",
            min_overlap=0.060,
            name="extended terminal carriage still retains upper-stage insertion",
        )
        terminal_extended = ctx.part_world_position(terminal_carriage)

    with ctx.pose({body_to_main: MAIN_TRAVEL, main_to_terminal: TERMINAL_TRAVEL}):
        stacked_extended = ctx.part_world_position(terminal_carriage)

    ctx.check(
        "main carriage extends along +X",
        main_rest is not None
        and main_extended is not None
        and main_extended[0] > main_rest[0] + 0.10,
        details=f"rest={main_rest}, extended={main_extended}",
    )
    ctx.check(
        "terminal carriage extends along +X",
        terminal_rest is not None
        and terminal_extended is not None
        and terminal_extended[0] > terminal_rest[0] + 0.04,
        details=f"rest={terminal_rest}, extended={terminal_extended}",
    )
    ctx.check(
        "stacked stages produce additive reach",
        terminal_rest is not None
        and stacked_extended is not None
        and stacked_extended[0] > terminal_rest[0] + 0.18,
        details=f"rest={terminal_rest}, stacked_extended={stacked_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
