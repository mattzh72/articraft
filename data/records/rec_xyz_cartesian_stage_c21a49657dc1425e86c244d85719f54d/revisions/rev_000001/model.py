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


BACK_W = 0.76
BACK_H = 0.92
BACK_T = 0.018

SUPPORT_GAP_D = 0.034
X_BEAM_W = 0.68
X_BEAM_D = 0.070
X_BEAM_H = 0.110
X_BEAM_Z0 = 0.51

X_RAIL_L = 0.60
X_RAIL_D = 0.018
X_RAIL_H = 0.012
X_RAIL_SPACING = 0.054

X_BEAM_FRONT_Y = BACK_T + SUPPORT_GAP_D + X_BEAM_D
X_RAIL_FRONT_Y = X_BEAM_FRONT_Y + X_RAIL_D
X_AXIS_Z = X_BEAM_Z0 + (X_BEAM_H / 2.0)

X_TRAVEL = 0.19
Y_TRAVEL = 0.06
Z_TRAVEL = 0.12

Y_RAIL_L = 0.17
Y_RAIL_W = 0.022
Y_RAIL_H = 0.012
Y_RAIL_X = 0.048
Y_RAIL_TOP_Z = 0.116

Z_RAIL_FRONT_Y = 0.118


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _add_mesh(
    part,
    shape: cq.Workplane,
    mesh_name: str,
    material: str,
    *,
    origin: Origin | None = None,
    visual_name: str,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        origin=origin or Origin(),
        material=material,
        name=visual_name,
    )


def _build_wall_support_shape() -> cq.Workplane:
    beam_center_y = BACK_T + SUPPORT_GAP_D + (X_BEAM_D / 2.0)
    beam_center_z = X_BEAM_Z0 + (X_BEAM_H / 2.0)
    rail_center_y = X_BEAM_FRONT_Y + (X_RAIL_D / 2.0)

    shape = _box((BACK_W, BACK_T, BACK_H), (0.0, BACK_T / 2.0, BACK_H / 2.0))
    shape = shape.union(_box((X_BEAM_W, X_BEAM_D, X_BEAM_H), (0.0, beam_center_y, beam_center_z)))

    for rib_x, rib_w, rib_h, rib_z in (
        (-0.25, 0.070, 0.28, 0.50),
        (0.0, 0.090, 0.34, 0.56),
        (0.25, 0.070, 0.28, 0.50),
    ):
        shape = shape.union(
            _box(
                (rib_w, SUPPORT_GAP_D, rib_h),
                (rib_x, BACK_T + (SUPPORT_GAP_D / 2.0), rib_z),
            )
        )

    shape = shape.union(_box((0.46, 0.026, 0.06), (0.0, BACK_T + 0.013, 0.72)))
    shape = shape.union(_box((0.22, 0.030, 0.10), (0.0, BACK_T + 0.015, 0.25)))

    for rail_z in (X_AXIS_Z - (X_RAIL_SPACING / 2.0), X_AXIS_Z + (X_RAIL_SPACING / 2.0)):
        shape = shape.union(_box((X_RAIL_L, X_RAIL_D, X_RAIL_H), (0.0, rail_center_y, rail_z)))

    return shape


def _build_x_carriage_shape() -> cq.Workplane:
    shape = _box((0.16, 0.022, 0.032), (0.0, 0.011, X_RAIL_SPACING / 2.0))
    shape = shape.union(_box((0.16, 0.022, 0.032), (0.0, 0.011, -X_RAIL_SPACING / 2.0)))
    shape = shape.union(_box((0.024, 0.055, 0.116), (-0.078, 0.028, 0.0)))
    shape = shape.union(_box((0.024, 0.055, 0.116), (0.078, 0.028, 0.0)))
    shape = shape.union(_box((0.18, 0.040, 0.116), (0.0, 0.070, 0.0)))
    shape = shape.union(_box((0.11, 0.060, 0.040), (0.0, 0.065, 0.072)))
    shape = shape.union(_box((0.26, 0.17, 0.018), (0.0, 0.085, 0.095)))

    shape = shape.union(_box((Y_RAIL_W, Y_RAIL_L, Y_RAIL_H), (-Y_RAIL_X, Y_RAIL_L / 2.0, 0.110)))
    shape = shape.union(_box((Y_RAIL_W, Y_RAIL_L, Y_RAIL_H), (Y_RAIL_X, Y_RAIL_L / 2.0, 0.110)))
    shape = shape.union(_box((0.19, 0.020, 0.012), (0.0, 0.145, 0.080)))

    return shape


def _build_y_stage_shape() -> cq.Workplane:
    shape = _box((0.14, 0.095, 0.024), (0.0, 0.0475, 0.012))
    shape = shape.union(_box((0.030, 0.095, 0.018), (-Y_RAIL_X, 0.0475, 0.009)))
    shape = shape.union(_box((0.030, 0.095, 0.018), (Y_RAIL_X, 0.0475, 0.009)))
    shape = shape.union(_box((0.11, 0.028, 0.20), (0.0, 0.090, 0.120)))
    shape = shape.union(_box((0.022, 0.055, 0.16), (-0.044, 0.073, 0.088)))
    shape = shape.union(_box((0.022, 0.055, 0.16), (0.044, 0.073, 0.088)))
    shape = shape.union(_box((0.10, 0.050, 0.018), (0.0, 0.070, 0.198)))
    shape = shape.union(_box((0.018, 0.014, 0.24), (-0.032, 0.111, 0.120)))
    shape = shape.union(_box((0.018, 0.014, 0.24), (0.032, 0.111, 0.120)))

    return shape


def _build_z_carriage_shape() -> cq.Workplane:
    shape = _box((0.026, 0.020, 0.11), (-0.032, 0.010, 0.055))
    shape = shape.union(_box((0.026, 0.020, 0.11), (0.032, 0.010, 0.055)))
    shape = shape.union(_box((0.10, 0.050, 0.12), (0.0, 0.035, 0.060)))
    shape = shape.union(_box((0.12, 0.012, 0.16), (0.0, 0.066, 0.070)))
    shape = shape.union(_box((0.060, 0.030, 0.022), (0.0, 0.042, 0.131)))

    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_xyz_positioner")

    model.material("support_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("carriage_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("stage_blue", rgba=(0.32, 0.42, 0.63, 1.0))
    model.material("tool_silver", rgba=(0.79, 0.81, 0.84, 1.0))

    wall_support = model.part("wall_support")
    _add_mesh(
        wall_support,
        _build_wall_support_shape(),
        "wall_support",
        "support_gray",
        visual_name="support_body",
    )
    wall_support.inertial = Inertial.from_geometry(
        Box((BACK_W, X_RAIL_FRONT_Y, BACK_H)),
        mass=18.0,
        origin=Origin(xyz=(0.0, X_RAIL_FRONT_Y / 2.0, BACK_H / 2.0)),
    )

    x_carriage = model.part("x_carriage")
    _add_mesh(
        x_carriage,
        _build_x_carriage_shape(),
        "x_carriage",
        "carriage_silver",
        visual_name="x_carriage_body",
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((0.26, 0.17, 0.16)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.085, 0.036)),
    )

    y_stage = model.part("y_stage")
    _add_mesh(
        y_stage,
        _build_y_stage_shape(),
        "y_stage",
        "stage_blue",
        visual_name="y_stage_body",
    )
    y_stage.inertial = Inertial.from_geometry(
        Box((0.14, 0.12, 0.24)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.06, 0.12)),
    )

    z_carriage = model.part("z_carriage")
    _add_mesh(
        z_carriage,
        _build_z_carriage_shape(),
        "z_carriage",
        "tool_silver",
        visual_name="z_carriage_body",
    )
    z_carriage.inertial = Inertial.from_geometry(
        Box((0.12, 0.078, 0.16)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.039, 0.08)),
    )

    model.articulation(
        "support_to_x",
        ArticulationType.PRISMATIC,
        parent=wall_support,
        child=x_carriage,
        origin=Origin(xyz=(0.0, X_RAIL_FRONT_Y, X_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=280.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, Y_RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Y_TRAVEL,
            effort=180.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_stage,
        child=z_carriage,
        origin=Origin(xyz=(0.0, Z_RAIL_FRONT_Y, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=140.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_support = object_model.get_part("wall_support")
    x_carriage = object_model.get_part("x_carriage")
    y_stage = object_model.get_part("y_stage")
    z_carriage = object_model.get_part("z_carriage")

    support_to_x = object_model.get_articulation("support_to_x")
    x_to_y = object_model.get_articulation("x_to_y")
    y_to_z = object_model.get_articulation("y_to_z")

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
        "all_primary_parts_present",
        all(part is not None for part in (wall_support, x_carriage, y_stage, z_carriage)),
        "Expected wall support, X carriage, Y stage, and Z carriage parts.",
    )
    ctx.check(
        "all_axes_are_prismatic",
        (
            support_to_x.articulation_type == ArticulationType.PRISMATIC
            and x_to_y.articulation_type == ArticulationType.PRISMATIC
            and y_to_z.articulation_type == ArticulationType.PRISMATIC
        ),
        "All three stacked axes must be prismatic joints.",
    )
    ctx.check(
        "joint_axes_match_xyz",
        (
            tuple(support_to_x.axis) == (1.0, 0.0, 0.0)
            and tuple(x_to_y.axis) == (0.0, 1.0, 0.0)
            and tuple(y_to_z.axis) == (0.0, 0.0, 1.0)
        ),
        (
            f"Expected axes X/Y/Z, got "
            f"{support_to_x.axis}, {x_to_y.axis}, {y_to_z.axis}."
        ),
    )

    ctx.expect_contact(x_carriage, wall_support, name="x_carriage_is_supported_on_wall_slide")
    ctx.expect_contact(y_stage, x_carriage, name="y_stage_is_supported_on_x_carriage")
    ctx.expect_contact(z_carriage, y_stage, name="z_carriage_is_supported_on_y_stage")

    ctx.expect_overlap(
        x_carriage,
        wall_support,
        axes="xz",
        min_overlap=0.12,
        name="x_carriage_overlaps_support_footprint",
    )
    ctx.expect_within(
        y_stage,
        x_carriage,
        axes="x",
        margin=0.025,
        name="y_stage_stays_nested_within_x_carriage_width",
    )
    ctx.expect_within(
        z_carriage,
        y_stage,
        axes="x",
        margin=0.010,
        name="z_carriage_stays_nested_within_y_stage_width",
    )

    home_x = ctx.part_world_position(x_carriage)
    with ctx.pose({support_to_x: X_TRAVEL}):
        x_pos = ctx.part_world_position(x_carriage)
    ctx.check(
        "positive_x_motion_moves_right",
        (
            home_x is not None
            and x_pos is not None
            and (x_pos[0] - home_x[0]) > 0.18
            and isclose(x_pos[1], home_x[1], abs_tol=1e-5)
            and isclose(x_pos[2], home_x[2], abs_tol=1e-5)
        ),
        f"Expected +X travel to move the carriage horizontally. home={home_x}, moved={x_pos}",
    )

    home_y = ctx.part_world_position(y_stage)
    with ctx.pose({x_to_y: Y_TRAVEL}):
        y_pos = ctx.part_world_position(y_stage)
    ctx.check(
        "positive_y_motion_moves_out_from_wall",
        (
            home_y is not None
            and y_pos is not None
            and (y_pos[1] - home_y[1]) > 0.05
            and isclose(y_pos[0], home_y[0], abs_tol=1e-5)
            and isclose(y_pos[2], home_y[2], abs_tol=1e-5)
        ),
        f"Expected +Y travel to move the upper stage outward. home={home_y}, moved={y_pos}",
    )

    home_z = ctx.part_world_position(z_carriage)
    with ctx.pose({y_to_z: Z_TRAVEL}):
        z_pos = ctx.part_world_position(z_carriage)
    ctx.check(
        "positive_z_motion_moves_up",
        (
            home_z is not None
            and z_pos is not None
            and (z_pos[2] - home_z[2]) > 0.10
            and isclose(z_pos[0], home_z[0], abs_tol=1e-5)
            and isclose(z_pos[1], home_z[1], abs_tol=1e-5)
        ),
        f"Expected +Z travel to lift the vertical carriage. home={home_z}, moved={z_pos}",
    )

    with ctx.pose({support_to_x: X_TRAVEL, x_to_y: Y_TRAVEL, y_to_z: Z_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_positive_travel_limits")
    with ctx.pose({support_to_x: -X_TRAVEL, x_to_y: Y_TRAVEL, y_to_z: Z_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_mixed_travel_limits")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
