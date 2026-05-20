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


BASE_L = 0.36
BASE_W = 0.18
BASE_T = 0.022
RAIL_L = 0.28
RAIL_W = 0.032
RAIL_H = 0.016
RAIL_Y = 0.048
OUTER_SHOULDER_Y = 0.072

CARRIAGE_L = 0.12
CARRIAGE_W = 0.112
CARRIAGE_TRAVEL = 0.15
CARRIAGE_RUNNER_W = 0.03
CARRIAGE_RUNNER_H = 0.022
CARRIAGE_TOP_H = 0.028
CARRIAGE_TOP_Z = 0.018
HINGE_X_ON_CARRIAGE = 0.01
HINGE_Z_ON_CARRIAGE = 0.055

PIVOT_FRAME_L = 0.14
PIVOT_FRAME_W = 0.04
PIVOT_FRAME_H = 0.044
PIVOT_SWEEP = 1.05

TERMINAL_SLIDER_L = 0.07
TERMINAL_SLIDER_W = 0.024
TERMINAL_SLIDER_H = 0.014
TERMINAL_SLIDER_TRAVEL = 0.055
TERMINAL_SLIDER_Z = 0.037
TERMINAL_SLIDER_X = 0.022


def _combine(*items: cq.Workplane) -> cq.Workplane:
    combined = cq.Workplane("XY")
    for item in items:
        combined = combined.add(item.val())
    return combined.combine(clean=True)


def _make_body_shape() -> cq.Workplane:
    slab = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
    left_rail = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_H, centered=(True, True, False))
        .translate((0.0, RAIL_Y, BASE_T))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_H, centered=(True, True, False))
        .translate((0.0, -RAIL_Y, BASE_T))
    )
    left_shoulder = (
        cq.Workplane("XY")
        .box(0.20, 0.022, 0.016, centered=(True, True, False))
        .translate((0.02, OUTER_SHOULDER_Y, BASE_T))
    )
    right_shoulder = (
        cq.Workplane("XY")
        .box(0.20, 0.022, 0.016, centered=(True, True, False))
        .translate((0.02, -OUTER_SHOULDER_Y, BASE_T))
    )
    service_spine = (
        cq.Workplane("XY")
        .box(0.12, 0.042, 0.016, centered=(True, True, False))
        .translate((-0.10, 0.0, BASE_T))
    )
    return _combine(slab, left_rail, right_rail, left_shoulder, right_shoulder, service_spine)


def _make_carriage_shape() -> cq.Workplane:
    left_runner = (
        cq.Workplane("XY")
        .box(CARRIAGE_L, CARRIAGE_RUNNER_W, CARRIAGE_RUNNER_H, centered=(True, True, False))
        .translate((0.0, RAIL_Y, 0.0))
    )
    right_runner = (
        cq.Workplane("XY")
        .box(CARRIAGE_L, CARRIAGE_RUNNER_W, CARRIAGE_RUNNER_H, centered=(True, True, False))
        .translate((0.0, -RAIL_Y, 0.0))
    )
    top_block = (
        cq.Workplane("XY")
        .box(CARRIAGE_L - 0.012, 0.096, CARRIAGE_TOP_H, centered=(True, True, False))
        .translate((0.0, 0.0, CARRIAGE_TOP_Z))
    )
    left_ear = (
        cq.Workplane("XY")
        .box(0.018, 0.018, 0.03, centered=(True, True, False))
        .translate((HINGE_X_ON_CARRIAGE, 0.031, 0.04))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.018, 0.018, 0.03, centered=(True, True, False))
        .translate((HINGE_X_ON_CARRIAGE, -0.031, 0.04))
    )
    return _combine(left_runner, right_runner, top_block, left_ear, right_ear)


def _make_pivot_frame_shape() -> cq.Workplane:
    left_foot = (
        cq.Workplane("XY")
        .box(0.018, 0.016, 0.014, centered=(True, True, False))
        .translate((0.0, 0.031, 0.015))
    )
    right_foot = (
        cq.Workplane("XY")
        .box(0.018, 0.016, 0.014, centered=(True, True, False))
        .translate((0.0, -0.031, 0.015))
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.024, 0.062, 0.008, centered=(False, True, False))
        .translate((0.0, 0.0, 0.029))
    )
    floor = (
        cq.Workplane("XY")
        .box(0.11, 0.028, 0.006, centered=(False, True, False))
        .translate((0.018, 0.0, 0.031))
    )
    left_wall = (
        cq.Workplane("XY")
        .box(0.11, 0.006, 0.022, centered=(False, True, False))
        .translate((0.018, 0.017, 0.037))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(0.11, 0.006, 0.022, centered=(False, True, False))
        .translate((0.018, -0.017, 0.037))
    )
    nose_cap = (
        cq.Workplane("XY")
        .box(0.01, 0.04, 0.018, centered=(False, True, False))
        .translate((0.118, 0.0, 0.037))
    )
    return _combine(left_foot, right_foot, bridge, floor, left_wall, right_wall, nose_cap)


def _make_terminal_slider_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(
        TERMINAL_SLIDER_L,
        TERMINAL_SLIDER_W,
        TERMINAL_SLIDER_H,
        centered=(False, True, False),
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[idx] + high[idx]) * 0.5 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_slide_link_slide_module")

    model.material("body_paint", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("carriage_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    model.material("frame_bluegray", rgba=(0.45, 0.51, 0.58, 1.0))
    model.material("slider_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "service_body"),
        material="body_paint",
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T + RAIL_H + 0.018)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "service_carriage"),
        material="carriage_gray",
        name="carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_L, CARRIAGE_W, 0.07)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    pivot_frame = model.part("pivot_frame")
    pivot_frame.visual(
        mesh_from_cadquery(_make_pivot_frame_shape(), "pivot_frame"),
        material="frame_bluegray",
        name="pivot_frame_shell",
    )
    pivot_frame.inertial = Inertial.from_geometry(
        Box((PIVOT_FRAME_L, 0.05, 0.06)),
        mass=0.9,
        origin=Origin(xyz=(0.07, 0.0, 0.04)),
    )

    terminal_slider = model.part("terminal_slider")
    terminal_slider.visual(
        mesh_from_cadquery(_make_terminal_slider_shape(), "terminal_slider"),
        material="slider_steel",
        name="terminal_slider_shell",
    )
    terminal_slider.inertial = Inertial.from_geometry(
        Box((TERMINAL_SLIDER_L, TERMINAL_SLIDER_W, TERMINAL_SLIDER_H)),
        mass=0.35,
        origin=Origin(xyz=(TERMINAL_SLIDER_L * 0.5, 0.0, TERMINAL_SLIDER_H * 0.5)),
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(-0.075, 0.0, BASE_T + RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=CARRIAGE_TRAVEL, effort=240.0, velocity=0.25),
    )
    model.articulation(
        "carriage_to_pivot_frame",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=pivot_frame,
        origin=Origin(xyz=(HINGE_X_ON_CARRIAGE, 0.0, HINGE_Z_ON_CARRIAGE)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=PIVOT_SWEEP, effort=20.0, velocity=1.6),
    )
    model.articulation(
        "pivot_frame_to_terminal_slider",
        ArticulationType.PRISMATIC,
        parent=pivot_frame,
        child=terminal_slider,
        origin=Origin(xyz=(TERMINAL_SLIDER_X, 0.0, TERMINAL_SLIDER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TERMINAL_SLIDER_TRAVEL, effort=80.0, velocity=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage")
    pivot_frame = object_model.get_part("pivot_frame")
    terminal_slider = object_model.get_part("terminal_slider")
    carriage_slide = object_model.get_articulation("body_to_carriage")
    frame_pivot = object_model.get_articulation("carriage_to_pivot_frame")
    slider_slide = object_model.get_articulation("pivot_frame_to_terminal_slider")

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
        "carriage_joint_is_prismatic_x",
        carriage_slide.axis == (1.0, 0.0, 0.0),
        details=f"Expected carriage axis (1, 0, 0), got {carriage_slide.axis}",
    )
    ctx.check(
        "pivot_joint_is_revolute_pitch",
        frame_pivot.axis == (0.0, -1.0, 0.0),
        details=f"Expected pivot axis (0, -1, 0), got {frame_pivot.axis}",
    )
    ctx.check(
        "terminal_slider_joint_is_prismatic_x",
        slider_slide.axis == (1.0, 0.0, 0.0),
        details=f"Expected terminal slider axis (1, 0, 0), got {slider_slide.axis}",
    )

    with ctx.pose(body_to_carriage=0.0, carriage_to_pivot_frame=0.0, pivot_frame_to_terminal_slider=0.0):
        ctx.expect_gap(
            carriage,
            body,
            axis="z",
            max_gap=0.002,
            max_penetration=0.0,
            name="carriage_seated_on_body_rails",
        )
        ctx.expect_overlap(
            carriage,
            body,
            axes="xy",
            min_overlap=0.08,
            name="carriage_remains_broadly_supported_by_body",
        )
        ctx.expect_within(
            terminal_slider,
            pivot_frame,
            axes="yz",
            margin=0.003,
            name="terminal_slider_fits_between_frame_guides",
        )
        ctx.expect_overlap(
            terminal_slider,
            pivot_frame,
            axes="x",
            min_overlap=0.04,
            name="terminal_slider_starts_retracted_inside_frame",
        )

    with ctx.pose(body_to_carriage=0.0, carriage_to_pivot_frame=0.0, pivot_frame_to_terminal_slider=0.0):
        carriage_home = ctx.part_world_position(carriage)
        slider_home = ctx.part_world_position(terminal_slider)
        pivot_closed_aabb = ctx.part_element_world_aabb(pivot_frame, elem="pivot_frame_shell")
    with ctx.pose(body_to_carriage=CARRIAGE_TRAVEL, carriage_to_pivot_frame=0.0, pivot_frame_to_terminal_slider=0.0):
        carriage_end = ctx.part_world_position(carriage)
    with ctx.pose(body_to_carriage=0.0, carriage_to_pivot_frame=PIVOT_SWEEP, pivot_frame_to_terminal_slider=0.0):
        pivot_open_aabb = ctx.part_element_world_aabb(pivot_frame, elem="pivot_frame_shell")
    with ctx.pose(body_to_carriage=0.0, carriage_to_pivot_frame=0.0, pivot_frame_to_terminal_slider=TERMINAL_SLIDER_TRAVEL):
        slider_end = ctx.part_world_position(terminal_slider)
    with ctx.pose(
        body_to_carriage=CARRIAGE_TRAVEL * 0.5,
        carriage_to_pivot_frame=PIVOT_SWEEP * 0.8,
        pivot_frame_to_terminal_slider=TERMINAL_SLIDER_TRAVEL,
    ):
        ctx.expect_within(
            terminal_slider,
            pivot_frame,
            axes="y",
            margin=0.003,
            name="terminal_slider_stays_guided_when_frame_is_open",
        )

    carriage_shift_ok = (
        carriage_home is not None
        and carriage_end is not None
        and abs((carriage_end[0] - carriage_home[0]) - CARRIAGE_TRAVEL) <= 0.003
        and abs(carriage_end[1] - carriage_home[1]) <= 1e-6
        and abs(carriage_end[2] - carriage_home[2]) <= 1e-6
    )
    ctx.check(
        "carriage_translates_cleanly_along_body",
        carriage_shift_ok,
        details=f"Home={carriage_home}, end={carriage_end}, expected Δx≈{CARRIAGE_TRAVEL:.3f}",
    )

    slider_shift_ok = (
        slider_home is not None
        and slider_end is not None
        and abs((slider_end[0] - slider_home[0]) - TERMINAL_SLIDER_TRAVEL) <= 0.003
        and abs(slider_end[1] - slider_home[1]) <= 1e-6
        and abs(slider_end[2] - slider_home[2]) <= 1e-6
    )
    ctx.check(
        "terminal_slider_extends_forward_from_frame",
        slider_shift_ok,
        details=f"Home={slider_home}, end={slider_end}, expected Δx≈{TERMINAL_SLIDER_TRAVEL:.3f}",
    )

    pivot_closed_center = _aabb_center(pivot_closed_aabb)
    pivot_open_center = _aabb_center(pivot_open_aabb)
    pivot_rise_ok = (
        pivot_closed_center is not None
        and pivot_open_center is not None
        and (pivot_open_center[2] - pivot_closed_center[2]) >= 0.025
        and (pivot_open_center[0] - pivot_closed_center[0]) <= -0.03
    )
    ctx.check(
        "pivot_frame_lifts_upward_when_opened",
        pivot_rise_ok,
        details=f"Closed center={pivot_closed_center}, open center={pivot_open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
