from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_T = 0.050
PLATE_W = 0.780
PLATE_H = 1.020
FOOT_D = 0.180
FOOT_W = 0.460
FOOT_H = 0.080
FLANGE_D = 0.030
FLANGE_W = 0.040
FLANGE_H = 0.860
GUSSET_T = 0.030
Y_RAIL_D = 0.028
Y_RAIL_W = 0.660
Y_RAIL_H = 0.018
UPPER_RAIL_Z = 0.760
LOWER_RAIL_Z = 0.620

BEAM_D = 0.080
BEAM_L = 0.560
BEAM_H = 0.080
ARM_D = 0.060
ARM_W = 0.080
ARM_H = 0.220
ARM_Y_OFFSET = 0.220
RUNNER_D = 0.032
RUNNER_W = 0.090
RUNNER_H = 0.040
RUNNER_X = -0.076
CARRIAGE_X = 0.145
CARRIAGE_Z = 0.690
Y_TRAVEL = 0.110

Z_HOUSING_D = 0.120
Z_HOUSING_W = 0.120
Z_HOUSING_H = 0.300
Z_HOUSING_CENTER = (0.015, 0.000, -0.190)
Z_GUIDE_D = 0.018
Z_GUIDE_W = 0.018
Z_GUIDE_H = 0.340
Z_GUIDE_Y = 0.035
Z_GUIDE_X = 0.084
STAGE_JOINT_X = 0.128
STAGE_JOINT_Z = -0.030
Z_TRAVEL = 0.200

SADDLE_D = 0.070
SADDLE_W = 0.110
SADDLE_H = 0.140
MAST_D = 0.050
MAST_W = 0.080
MAST_H = 0.220
TOOLPLATE_D = 0.110
TOOLPLATE_W = 0.160
TOOLPLATE_H = 0.028
TOP_BRIDGE_D = 0.048
TOP_BRIDGE_W = 0.094
TOP_BRIDGE_H = 0.060
GUIDE_SHOE_D = 0.014
GUIDE_SHOE_W = 0.024
GUIDE_SHOE_H = 0.130
GUIDE_SHOE_X = -0.028
GUIDE_SHOE_Z = -0.050
NOSE_R = 0.022
NOSE_L = 0.050


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _make_frame_body() -> cq.Workplane:
    frame = _box((0.0, 0.0, PLATE_H / 2.0), (PLATE_T, PLATE_W, PLATE_H))
    frame = frame.union(_box((0.055, 0.0, FOOT_H / 2.0), (FOOT_D, FOOT_W, FOOT_H)))
    frame = frame.union(
        _box((0.040, PLATE_W / 2.0 - FLANGE_W / 2.0, 0.560), (FLANGE_D, FLANGE_W, FLANGE_H))
    )
    frame = frame.union(
        _box((0.040, -PLATE_W / 2.0 + FLANGE_W / 2.0, 0.560), (FLANGE_D, FLANGE_W, FLANGE_H))
    )

    gusset = (
        cq.Workplane("XZ")
        .polyline([(0.0, 0.0), (0.140, 0.0), (0.0, 0.240)])
        .close()
        .extrude(GUSSET_T, both=True)
    )
    frame = frame.union(gusset.translate((0.0, 0.185, 0.0)))
    frame = frame.union(gusset.translate((0.0, -0.185, 0.0)))
    return frame


def _make_carriage_body() -> cq.Workplane:
    carriage = _box((0.0, 0.0, 0.0), (BEAM_D, BEAM_L, BEAM_H))
    carriage = carriage.union(_box((-0.030, ARM_Y_OFFSET, -0.015), (ARM_D, ARM_W, ARM_H)))
    carriage = carriage.union(_box((-0.030, -ARM_Y_OFFSET, -0.015), (ARM_D, ARM_W, ARM_H)))
    carriage = carriage.union(_box(Z_HOUSING_CENTER, (Z_HOUSING_D, Z_HOUSING_W, Z_HOUSING_H)))
    return carriage


def _make_stage_body() -> cq.Workplane:
    stage = _box((-0.001, 0.0, -0.030), (TOP_BRIDGE_D, TOP_BRIDGE_W, TOP_BRIDGE_H))
    stage = stage.union(_box((0.018, 0.0, -0.090), (0.055, 0.080, 0.120)))
    stage = stage.union(_box((0.020, 0.0, -0.240), (0.045, MAST_W, 0.220)))
    stage = stage.union(_box((0.025, 0.0, -0.360), (0.090, TOOLPLATE_W, TOOLPLATE_H)))
    return stage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_yz_transfer_unit")

    model.material("painted_steel", rgba=(0.73, 0.76, 0.79, 1.0))
    model.material("rail_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("anodized_beam", rgba=(0.82, 0.84, 0.87, 1.0))
    model.material("runner_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("stage_gray", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("tool_steel", rgba=(0.45, 0.47, 0.50, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_body(), "frame_body"),
        material="painted_steel",
        name="frame_body",
    )
    frame.visual(
        Box((Y_RAIL_D, Y_RAIL_W, Y_RAIL_H)),
        origin=Origin(xyz=(PLATE_T / 2.0 + Y_RAIL_D / 2.0, 0.0, UPPER_RAIL_Z)),
        material="rail_steel",
        name="upper_y_rail",
    )
    frame.visual(
        Box((Y_RAIL_D, Y_RAIL_W, Y_RAIL_H)),
        origin=Origin(xyz=(PLATE_T / 2.0 + Y_RAIL_D / 2.0, 0.0, LOWER_RAIL_Z)),
        material="rail_steel",
        name="lower_y_rail",
    )

    carriage = model.part("beam_carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_body(), "beam_carriage_body"),
        material="anodized_beam",
        name="carriage_body",
    )
    for side_name, y_center in (("right", ARM_Y_OFFSET), ("left", -ARM_Y_OFFSET)):
        carriage.visual(
            Box((RUNNER_D, RUNNER_W, RUNNER_H)),
            origin=Origin(xyz=(RUNNER_X, y_center, 0.070)),
            material="runner_black",
            name=f"{side_name}_upper_runner",
        )
        carriage.visual(
            Box((RUNNER_D, RUNNER_W, RUNNER_H)),
            origin=Origin(xyz=(RUNNER_X, y_center, -0.070)),
            material="runner_black",
            name=f"{side_name}_lower_runner",
        )
    carriage.visual(
        Box((Z_GUIDE_D, Z_GUIDE_W, Z_GUIDE_H)),
        origin=Origin(xyz=(Z_GUIDE_X, Z_GUIDE_Y, -0.200)),
        material="rail_steel",
        name="right_z_guide",
    )
    carriage.visual(
        Box((Z_GUIDE_D, Z_GUIDE_W, Z_GUIDE_H)),
        origin=Origin(xyz=(Z_GUIDE_X, -Z_GUIDE_Y, -0.200)),
        material="rail_steel",
        name="left_z_guide",
    )

    stage = model.part("lower_stage")
    stage.visual(
        mesh_from_cadquery(_make_stage_body(), "lower_stage_body"),
        material="stage_gray",
        name="stage_body",
    )
    stage.visual(
        Box((GUIDE_SHOE_D, GUIDE_SHOE_W, GUIDE_SHOE_H)),
        origin=Origin(xyz=(GUIDE_SHOE_X, Z_GUIDE_Y, GUIDE_SHOE_Z)),
        material="runner_black",
        name="right_guide_shoe",
    )
    stage.visual(
        Box((GUIDE_SHOE_D, GUIDE_SHOE_W, GUIDE_SHOE_H)),
        origin=Origin(xyz=(GUIDE_SHOE_X, -Z_GUIDE_Y, GUIDE_SHOE_Z)),
        material="runner_black",
        name="left_guide_shoe",
    )
    stage.visual(
        Cylinder(radius=NOSE_R, length=NOSE_L),
        origin=Origin(xyz=(0.078, 0.0, -0.360), rpy=(0.0, pi / 2.0, 0.0)),
        material="tool_steel",
        name="tool_nose",
    )

    model.articulation(
        "frame_to_beam_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_X, 0.0, CARRIAGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=450.0,
            velocity=0.70,
        ),
    )
    model.articulation(
        "beam_carriage_to_lower_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=stage,
        origin=Origin(xyz=(STAGE_JOINT_X, 0.0, STAGE_JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=220.0,
            velocity=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("beam_carriage")
    stage = object_model.get_part("lower_stage")
    y_slide = object_model.get_articulation("frame_to_beam_carriage")
    z_slide = object_model.get_articulation("beam_carriage_to_lower_stage")

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
        "horizontal_beam_axis_is_crosswise_y",
        tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        f"expected Y-axis slide, got {y_slide.axis}",
    )
    ctx.check(
        "vertical_stage_axis_moves_downward",
        tuple(z_slide.axis) == (0.0, 0.0, -1.0),
        f"expected downward Z slide, got {z_slide.axis}",
    )

    with ctx.pose({y_slide: y_slide.motion_limits.upper}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="right_upper_runner",
            elem_b="upper_y_rail",
            name="positive_y_runner_stays_on_upper_rail",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="right_lower_runner",
            elem_b="lower_y_rail",
            name="positive_y_runner_stays_on_lower_rail",
        )

    with ctx.pose({y_slide: y_slide.motion_limits.lower}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="left_upper_runner",
            elem_b="upper_y_rail",
            name="negative_y_runner_stays_on_upper_rail",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="left_lower_runner",
            elem_b="lower_y_rail",
            name="negative_y_runner_stays_on_lower_rail",
        )

    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        ctx.expect_contact(
            stage,
            carriage,
            elem_a="left_guide_shoe",
            elem_b="left_z_guide",
            name="extended_stage_left_shoe_remains_supported_by_z_guide",
        )
        ctx.expect_contact(
            stage,
            carriage,
            elem_a="right_guide_shoe",
            elem_b="right_z_guide",
            name="extended_stage_right_shoe_remains_supported_by_z_guide",
        )
        ctx.expect_gap(
            stage,
            frame,
            axis="x",
            min_gap=0.080,
            name="extended_stage_hangs_clear_of_backplate",
        )

    with ctx.pose({y_slide: y_slide.motion_limits.lower}):
        left_pos = ctx.part_world_position(carriage)
    with ctx.pose({y_slide: y_slide.motion_limits.upper}):
        right_pos = ctx.part_world_position(carriage)
    ctx.check(
        "beam_carriage_translates_across_plate",
        left_pos is not None
        and right_pos is not None
        and (right_pos[1] - left_pos[1]) > (2.0 * Y_TRAVEL - 0.005),
        f"expected ~{2.0 * Y_TRAVEL:.3f} m of Y travel, got {None if left_pos is None or right_pos is None else right_pos[1] - left_pos[1]:.3f}",
    )

    with ctx.pose({z_slide: 0.0}):
        home_pos = ctx.part_world_position(stage)
    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        down_pos = ctx.part_world_position(stage)
    ctx.check(
        "lower_stage_translates_vertically",
        home_pos is not None
        and down_pos is not None
        and (home_pos[2] - down_pos[2]) > (Z_TRAVEL - 0.005),
        f"expected ~{Z_TRAVEL:.3f} m of downward travel, got {None if home_pos is None or down_pos is None else home_pos[2] - down_pos[2]:.3f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
