from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


BEAM_LENGTH = 0.54
BEAM_DEPTH = 0.18
BEAM_HEIGHT = 0.12
BEAM_CENTER = (0.12, 0.0, 0.88)
BEAM_FRONT_Y = BEAM_CENTER[1] + (BEAM_DEPTH / 2.0)

X_RAIL_LENGTH = 0.48
X_RAIL_DEPTH = 0.014
X_RAIL_HEIGHT = 0.020
X_RAIL_CENTER_X = 0.07
X_RAIL_CENTER_Y = BEAM_FRONT_Y + (X_RAIL_DEPTH / 2.0) - 0.001
X_RAIL_FRONT_Y = X_RAIL_CENTER_Y + (X_RAIL_DEPTH / 2.0)
X_RAIL_TOP_Z = 0.915
X_RAIL_BOTTOM_Z = 0.845

X_TRAVEL_LOWER = 0.0
X_TRAVEL_UPPER = 0.27
X_HOME_X = 0.02

CAR_Z_RAIL_DEPTH = 0.014
CAR_Z_RAIL_FRONT_Y = 0.094
CAR_Z_RAIL_CENTER_Y = CAR_Z_RAIL_FRONT_Y - (CAR_Z_RAIL_DEPTH / 2.0)
CAR_Z_RAIL_TOP_Z = 0.03
CAR_Z_RAIL_HEIGHT = 0.32
CAR_Z_RAIL_CENTER_Z = CAR_Z_RAIL_TOP_Z - (CAR_Z_RAIL_HEIGHT / 2.0)

Z_TRAVEL_LOWER = 0.0
Z_TRAVEL_UPPER = 0.16


def _filleted_box(size: tuple[float, float, float], center: tuple[float, float, float], radius: float) -> cq.Workplane:
    solid = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        solid = solid.edges("|Z").fillet(radius)
    return solid.translate(center)


def _frame_body_shape() -> cq.Workplane:
    base = _filleted_box((0.28, 0.40, 0.08), (-0.20, 0.0, 0.04), 0.010)
    wall = _filleted_box((0.18, 0.30, 0.78), (-0.21, 0.0, 0.47), 0.014)
    beam = _filleted_box((BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT), BEAM_CENTER, 0.012)
    rear_stiffener = _filleted_box((0.12, 0.18, 0.46), (-0.14, -0.01, 0.64), 0.010)
    rear_gusset = _filleted_box((0.18, 0.08, 0.24), (-0.10, -0.08, 0.75), 0.008)
    lower_gusset = _filleted_box((0.16, 0.08, 0.18), (-0.17, -0.08, 0.57), 0.008)
    return base.union(wall).union(beam).union(rear_stiffener).union(rear_gusset).union(lower_gusset)


def _frame_x_rails_shape() -> cq.Workplane:
    upper = cq.Workplane("XY").box(X_RAIL_LENGTH, X_RAIL_DEPTH, X_RAIL_HEIGHT).translate(
        (X_RAIL_CENTER_X, X_RAIL_CENTER_Y, X_RAIL_TOP_Z)
    )
    lower = cq.Workplane("XY").box(X_RAIL_LENGTH, X_RAIL_DEPTH, X_RAIL_HEIGHT).translate(
        (X_RAIL_CENTER_X, X_RAIL_CENTER_Y, X_RAIL_BOTTOM_Z)
    )
    return upper.union(lower)


def _carriage_body_shape() -> cq.Workplane:
    upper_bridge = _filleted_box((0.18, 0.048, 0.18), (0.0, 0.038, 0.0), 0.010)
    front_plate = _filleted_box((0.11, 0.032, 0.34), (0.0, 0.065, -0.13), 0.008)
    head_cap = _filleted_box((0.16, 0.062, 0.036), (0.0, 0.032, 0.078), 0.006)
    return upper_bridge.union(front_plate).union(head_cap)


def _carriage_x_runner_shape() -> cq.Workplane:
    upper = cq.Workplane("XY").box(0.12, 0.016, 0.045).translate((0.0, 0.008, 0.036))
    lower = cq.Workplane("XY").box(0.12, 0.016, 0.045).translate((0.0, 0.008, -0.036))
    tie = cq.Workplane("XY").box(0.06, 0.020, 0.036).translate((0.0, 0.014, 0.0))
    return upper.union(lower).union(tie)


def _carriage_z_rails_shape() -> cq.Workplane:
    left = cq.Workplane("XY").box(0.018, CAR_Z_RAIL_DEPTH, CAR_Z_RAIL_HEIGHT).translate(
        (-0.034, CAR_Z_RAIL_CENTER_Y, CAR_Z_RAIL_CENTER_Z)
    )
    right = cq.Workplane("XY").box(0.018, CAR_Z_RAIL_DEPTH, CAR_Z_RAIL_HEIGHT).translate(
        (0.034, CAR_Z_RAIL_CENTER_Y, CAR_Z_RAIL_CENTER_Z)
    )
    return left.union(right)


def _z_stage_runner_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.086, 0.016, 0.18).translate((0.0, 0.008, -0.09))
    cap = cq.Workplane("XY").box(0.060, 0.020, 0.028).translate((0.0, 0.014, -0.014))
    return plate.union(cap)


def _z_stage_body_shape() -> cq.Workplane:
    ram = _filleted_box((0.11, 0.054, 0.24), (0.0, 0.034, -0.16), 0.008)
    lower_nose = _filleted_box((0.090, 0.120, 0.090), (0.0, 0.073, -0.285), 0.006)
    return ram.union(lower_nose)


def _tool_face_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(0.13, 0.010, 0.14).translate((0.0, 0.138, -0.245))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_portal_module")

    model.material("frame_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("rail_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("carriage_graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("slide_gray", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("tool_face_gray", rgba=(0.83, 0.85, 0.88, 1.0))

    portal_frame = model.part("portal_frame")
    portal_frame.visual(
        mesh_from_cadquery(_frame_body_shape(), "portal_frame_body"),
        material="frame_aluminum",
        name="body",
    )
    portal_frame.visual(
        mesh_from_cadquery(_frame_x_rails_shape(), "portal_frame_x_rails"),
        material="rail_steel",
        name="x_rails",
    )
    portal_frame.inertial = Inertial.from_geometry(
        Box((0.73, 0.40, 0.94)),
        mass=88.0,
        origin=Origin(xyz=(0.025, 0.0, 0.47)),
    )

    beam_carriage = model.part("beam_carriage")
    beam_carriage.visual(
        mesh_from_cadquery(_carriage_body_shape(), "beam_carriage_body"),
        material="carriage_graphite",
        name="body",
    )
    beam_carriage.visual(
        mesh_from_cadquery(_carriage_x_runner_shape(), "beam_carriage_x_runner"),
        material="rail_steel",
        name="x_runner",
    )
    beam_carriage.visual(
        mesh_from_cadquery(_carriage_z_rails_shape(), "beam_carriage_z_rails"),
        material="rail_steel",
        name="z_rails",
    )
    beam_carriage.inertial = Inertial.from_geometry(
        Box((0.18, 0.094, 0.34)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.047, -0.09)),
    )

    z_stage = model.part("z_stage")
    z_stage.visual(
        mesh_from_cadquery(_z_stage_runner_shape(), "z_stage_runner"),
        material="rail_steel",
        name="z_runner",
    )
    z_stage.visual(
        mesh_from_cadquery(_z_stage_body_shape(), "z_stage_body"),
        material="slide_gray",
        name="body",
    )
    z_stage.visual(
        mesh_from_cadquery(_tool_face_shape(), "z_stage_tool_face"),
        material="tool_face_gray",
        name="tool_face",
    )
    z_stage.inertial = Inertial.from_geometry(
        Box((0.13, 0.071, 0.31)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0355, -0.155)),
    )

    model.articulation(
        "portal_to_beam_carriage",
        ArticulationType.PRISMATIC,
        parent=portal_frame,
        child=beam_carriage,
        origin=Origin(xyz=(X_HOME_X, X_RAIL_FRONT_Y, BEAM_CENTER[2])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=X_TRAVEL_LOWER,
            upper=X_TRAVEL_UPPER,
            effort=1600.0,
            velocity=0.60,
        ),
    )
    model.articulation(
        "beam_carriage_to_z_stage",
        ArticulationType.PRISMATIC,
        parent=beam_carriage,
        child=z_stage,
        origin=Origin(xyz=(0.0, CAR_Z_RAIL_FRONT_Y, CAR_Z_RAIL_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=Z_TRAVEL_LOWER,
            upper=Z_TRAVEL_UPPER,
            effort=900.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    portal_frame = object_model.get_part("portal_frame")
    beam_carriage = object_model.get_part("beam_carriage")
    z_stage = object_model.get_part("z_stage")
    x_slide = object_model.get_articulation("portal_to_beam_carriage")
    z_slide = object_model.get_articulation("beam_carriage_to_z_stage")

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
        "beam carriage prismatic axis is +x",
        tuple(x_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected +x axis, got {x_slide.axis}",
    )
    ctx.check(
        "z stage prismatic axis is -z",
        tuple(z_slide.axis) == (0.0, 0.0, -1.0),
        details=f"expected -z axis, got {z_slide.axis}",
    )

    with ctx.pose({x_slide: 0.0, z_slide: 0.0}):
        ctx.expect_contact(
            beam_carriage,
            portal_frame,
            elem_a="x_runner",
            elem_b="x_rails",
            contact_tol=1e-4,
            name="beam carriage runner touches beam rails at home",
        )
        ctx.expect_overlap(
            beam_carriage,
            portal_frame,
            elem_a="x_runner",
            elem_b="x_rails",
            axes="xz",
            min_overlap=0.018,
            name="beam carriage runner overlaps rail footprint at home",
        )
        ctx.expect_contact(
            z_stage,
            beam_carriage,
            elem_a="z_runner",
            elem_b="z_rails",
            contact_tol=1e-4,
            name="z stage runner touches carriage rails at home",
        )
        ctx.expect_overlap(
            z_stage,
            beam_carriage,
            elem_a="z_runner",
            elem_b="z_rails",
            axes="xz",
            min_overlap=0.018,
            name="z stage runner overlaps carriage rail footprint at home",
        )

    with ctx.pose({x_slide: X_TRAVEL_UPPER, z_slide: Z_TRAVEL_UPPER}):
        ctx.expect_contact(
            beam_carriage,
            portal_frame,
            elem_a="x_runner",
            elem_b="x_rails",
            contact_tol=1e-4,
            name="beam carriage runner stays on beam rails at max stroke",
        )
        ctx.expect_contact(
            z_stage,
            beam_carriage,
            elem_a="z_runner",
            elem_b="z_rails",
            contact_tol=1e-4,
            name="z stage runner stays on carriage rails at max stroke",
        )
        ctx.expect_gap(
            z_stage,
            portal_frame,
            axis="y",
            positive_elem="tool_face",
            min_gap=0.12,
            name="tool face remains proud of portal frame",
        )

    x_upper = x_slide.motion_limits.upper
    z_upper = z_slide.motion_limits.upper
    x_home_pos = ctx.part_world_position(beam_carriage)
    z_home_pos = ctx.part_world_position(z_stage)
    with ctx.pose({x_slide: x_upper}):
        x_far_pos = ctx.part_world_position(beam_carriage)
    with ctx.pose({z_slide: z_upper}):
        z_low_pos = ctx.part_world_position(z_stage)

    x_moves_positive = (
        x_home_pos is not None
        and x_far_pos is not None
        and (x_far_pos[0] - x_home_pos[0]) > 0.20
    )
    z_moves_down = (
        z_home_pos is not None
        and z_low_pos is not None
        and (z_low_pos[2] - z_home_pos[2]) < -0.10
    )
    ctx.check(
        "beam carriage translates positively along beam",
        x_moves_positive,
        details=f"home={x_home_pos}, max={x_far_pos}",
    )
    ctx.check(
        "z stage descends on positive command",
        z_moves_down,
        details=f"home={z_home_pos}, max={z_low_pos}",
    )

    limits_ok = (
        x_slide.motion_limits is not None
        and z_slide.motion_limits is not None
        and isclose(x_slide.motion_limits.lower or 0.0, X_TRAVEL_LOWER, abs_tol=1e-9)
        and isclose(x_slide.motion_limits.upper or 0.0, X_TRAVEL_UPPER, abs_tol=1e-9)
        and isclose(z_slide.motion_limits.lower or 0.0, Z_TRAVEL_LOWER, abs_tol=1e-9)
        and isclose(z_slide.motion_limits.upper or 0.0, Z_TRAVEL_UPPER, abs_tol=1e-9)
    )
    ctx.check(
        "prismatic travel limits match modeled strokes",
        limits_ok,
        details=(
            f"x_limits={x_slide.motion_limits}, "
            f"z_limits={z_slide.motion_limits}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
