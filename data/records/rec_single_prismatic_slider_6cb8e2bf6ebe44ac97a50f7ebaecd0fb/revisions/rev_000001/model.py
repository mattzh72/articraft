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


CHANNEL_LEN = 0.42
CHANNEL_OUTER_W = 0.108
CHANNEL_H = 0.052
WALL_T = 0.005
FLOOR_T = 0.005
REAR_BRIDGE_T = 0.008
REAR_BRIDGE_H = 0.028
GUIDE_RAIL_LEN = 0.37
GUIDE_RAIL_W = 0.014
GUIDE_RAIL_H = 0.004
GUIDE_RAIL_Y = 0.034

TRAY_LEN = 0.38
TRAY_OUTER_W = 0.094
TRAY_FLOOR_T = 0.0035
TRAY_SIDE_T = 0.003
TRAY_WALL_H = 0.02
RUNNER_LEN = 0.32
RUNNER_W = 0.012
RUNNER_H = GUIDE_RAIL_H
RUNNER_Y = GUIDE_RAIL_Y
HANDLE_T = 0.024
SLIDE_TRAVEL = 0.16
RUNNING_Z = FLOOR_T + GUIDE_RAIL_H


def _box(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .translate((x, y, z))
    )


def _build_base_channel() -> cq.Workplane:
    side_wall_h = CHANNEL_H - FLOOR_T

    base = _box(CHANNEL_LEN, CHANNEL_OUTER_W, FLOOR_T)
    base = base.union(
        _box(
            CHANNEL_LEN,
            WALL_T,
            side_wall_h,
            y=(CHANNEL_OUTER_W - WALL_T) / 2,
            z=FLOOR_T,
        )
    )
    base = base.union(
        _box(
            CHANNEL_LEN,
            WALL_T,
            side_wall_h,
            y=-(CHANNEL_OUTER_W - WALL_T) / 2,
            z=FLOOR_T,
        )
    )
    base = base.union(
        _box(
            REAR_BRIDGE_T,
            CHANNEL_OUTER_W - 2 * WALL_T,
            REAR_BRIDGE_H,
            x=-(CHANNEL_LEN - REAR_BRIDGE_T) / 2,
            z=FLOOR_T,
        )
    )
    base = base.union(
        _box(
            GUIDE_RAIL_LEN,
            GUIDE_RAIL_W,
            GUIDE_RAIL_H,
            y=GUIDE_RAIL_Y,
            z=FLOOR_T,
        )
    )
    base = base.union(
        _box(
            GUIDE_RAIL_LEN,
            GUIDE_RAIL_W,
            GUIDE_RAIL_H,
            y=-GUIDE_RAIL_Y,
            z=FLOOR_T,
        )
    )
    return base.edges("|Z").fillet(0.0015)


def _build_carriage_tray() -> cq.Workplane:
    wall_run = TRAY_LEN - 2 * TRAY_SIDE_T

    carriage = _box(TRAY_LEN, TRAY_OUTER_W, TRAY_FLOOR_T, z=RUNNER_H)
    carriage = carriage.union(
        _box(
            wall_run,
            TRAY_SIDE_T,
            TRAY_WALL_H,
            y=(TRAY_OUTER_W - TRAY_SIDE_T) / 2,
            z=RUNNER_H + TRAY_FLOOR_T,
        )
    )
    carriage = carriage.union(
        _box(
            wall_run,
            TRAY_SIDE_T,
            TRAY_WALL_H,
            y=-(TRAY_OUTER_W - TRAY_SIDE_T) / 2,
            z=RUNNER_H + TRAY_FLOOR_T,
        )
    )
    carriage = carriage.union(
        _box(
            TRAY_SIDE_T,
            TRAY_OUTER_W - 2 * TRAY_SIDE_T,
            TRAY_WALL_H,
            x=-(TRAY_LEN - TRAY_SIDE_T) / 2,
            z=RUNNER_H + TRAY_FLOOR_T,
        )
    )
    carriage = carriage.union(
        _box(
            TRAY_SIDE_T,
            TRAY_OUTER_W - 2 * TRAY_SIDE_T,
            TRAY_WALL_H,
            x=(TRAY_LEN - TRAY_SIDE_T) / 2,
            z=RUNNER_H + TRAY_FLOOR_T,
        )
    )
    carriage = carriage.union(
        _box(
            RUNNER_LEN,
            RUNNER_W,
            RUNNER_H,
            x=-0.01,
            y=RUNNER_Y,
        )
    )
    carriage = carriage.union(
        _box(
            RUNNER_LEN,
            RUNNER_W,
            RUNNER_H,
            x=-0.01,
            y=-RUNNER_Y,
        )
    )
    carriage = carriage.union(
        _box(
            HANDLE_T,
            TRAY_OUTER_W,
            0.034,
            x=TRAY_LEN / 2 + HANDLE_T / 2,
            z=RUNNER_H,
        )
    )
    return carriage.edges(">X and |Z").fillet(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="instrument_slide")

    base_mat = model.material("base_anodized", rgba=(0.34, 0.37, 0.40, 1.0))
    carriage_mat = model.material("carriage_brushed", rgba=(0.75, 0.77, 0.80, 1.0))

    base = model.part("base_channel")
    base.visual(
        mesh_from_cadquery(
            _build_base_channel(),
            "base_channel",
            tolerance=0.0005,
            angular_tolerance=0.05,
        ),
        material=base_mat,
        name="base_channel_shell",
    )

    carriage = model.part("carriage_tray")
    carriage.visual(
        mesh_from_cadquery(
            _build_carriage_tray(),
            "carriage_tray",
            tolerance=0.0005,
            angular_tolerance=0.05,
        ),
        material=carriage_mat,
        name="carriage_shell",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RUNNING_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.30,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_channel")
    carriage = object_model.get_part("carriage_tray")
    slide = object_model.get_articulation("base_to_carriage")

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

    axis = tuple(round(v, 6) for v in slide.axis)
    limits = slide.motion_limits
    slide_type = getattr(slide.articulation_type, "name", str(slide.articulation_type))
    ctx.check(
        "single_prismatic_axis",
        slide_type.endswith("PRISMATIC") and axis == (1.0, 0.0, 0.0),
        f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.check(
        "slide_limits_match_channel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - 0.0) < 1e-9
        and abs(limits.upper - SLIDE_TRAVEL) < 1e-9,
        f"limits={limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            contact_tol=0.0002,
            name="closed_slide_keeps_guided_contact",
        )
        ctx.expect_within(
            carriage,
            base,
            axes="yz",
            margin=0.0005,
            name="carriage_section_stays_inside_channel",
        )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            base,
            contact_tol=0.0002,
            name="extended_slide_keeps_guided_contact",
        )
        ctx.expect_origin_gap(
            carriage,
            base,
            axis="x",
            min_gap=SLIDE_TRAVEL - 1e-6,
            max_gap=SLIDE_TRAVEL + 1e-6,
            name="carriage_moves_on_single_prismatic_axis",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
