from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_L = 0.320
BODY_W = 0.096
BODY_H = 0.068
BODY_REAR_BLOCK_L = 0.042
BODY_BASE_L = 0.278
BODY_BASE_W = 0.072
BODY_BASE_H = 0.018
BODY_BASE_Z = -0.029
BODY_SIDE_WALL_L = 0.258
BODY_SIDE_WALL_W = 0.012
BODY_SIDE_WALL_H = 0.048
BODY_SIDE_WALL_Y = 0.042
BODY_WAY_X = 0.050
BODY_WAY_L = 0.244
BODY_WAY_W = 0.010
BODY_WAY_H = 0.018
BODY_WAY_Y = 0.032
BODY_WAY_Z = -0.007
BODY_FRONT_CHEEK_L = 0.028
BODY_FRONT_CHEEK_W = 0.011
BODY_FRONT_CHEEK_H = 0.060
BODY_FRONT_CHEEK_Y = 0.0425
BODY_FRONT_TOP_W = 0.074
BODY_FRONT_TOP_H = 0.008
BODY_FRONT_TOP_Z = 0.029
BODY_COVER_REAR_X = 0.072
BODY_COVER_REAR_L = 0.060
BODY_COVER_FRONT_X = 0.212
BODY_COVER_FRONT_L = 0.040
BODY_COVER_W = 0.008
BODY_COVER_H = 0.006
BODY_COVER_Z = 0.027
BODY_COVER_Y = 0.036
BODY_FOOT_L = 0.032
BODY_FOOT_W = 0.076
BODY_FOOT_H = 0.010
BODY_REAR_FOOT_X = 0.014
BODY_FRONT_FOOT_X = 0.274
BODY_FOOT_Z = -0.039

CARR_L = 0.170
CARR_TOP_W = 0.048
CARR_TOP_H = 0.008
CARR_TOP_Z = 0.028
CARR_WALL_W = 0.006
CARR_WALL_H = 0.032
CARR_WALL_Y = 0.021
CARR_WALL_Z = 0.008
CARR_RUNNER_X = 0.009
CARR_RUNNER_L = 0.152
CARR_RUNNER_W = 0.010
CARR_RUNNER_H = 0.020
CARR_RUNNER_Y = 0.022
CARR_RUNNER_Z = -0.013
CARR_PAD_T = 0.004
CARR_PAD_W = 0.048
CARR_PAD_H = 0.006
CARR_PAD_Z = 0.021

NOSE_L = 0.180
NOSE_BAR_W = 0.028
NOSE_BAR_H = 0.024
NOSE_BAR_Z = 0.004
NOSE_REAR_BLOCK_L = 0.032
NOSE_REAR_BLOCK_W = 0.038
NOSE_REAR_BLOCK_H = 0.032
NOSE_FRONT_PAD_L = 0.020
NOSE_FRONT_PAD_W = 0.044
NOSE_FRONT_PAD_H = 0.036
NOSE_FRONT_PAD_Z = 0.004
NOSE_RIB_X = 0.020
NOSE_RIB_L = 0.130
NOSE_RIB_W = 0.004
NOSE_RIB_H = 0.018
NOSE_RIB_Y = 0.016
NOSE_RIB_Z = 0.000
NOSE_TIP_PAD_T = 0.003
NOSE_TIP_PAD_W = 0.036
NOSE_TIP_PAD_H = 0.028
NOSE_TIP_PAD_Z = 0.004

OUTER_TRAVEL = 0.108
INNER_TRAVEL = 0.072
BODY_TO_CARRIAGE_X = 0.052
CARRIAGE_TO_NOSE_X = 0.028

VISUAL_OVERLAP = 0.0005
SCREW_HEAD_R = 0.0045
SCREW_HEAD_H = 0.004


def _center_x(x0: float, length: float) -> float:
    return x0 + (length / 2.0)


def _add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    x0: float,
    y: float = 0.0,
    z: float = 0.0,
    material: str,
):
    part.visual(
        Box(size),
        origin=Origin(xyz=(_center_x(x0, size[0]), y, z)),
        material=material,
        name=name,
    )


def _add_screw_head(
    part,
    *,
    name: str,
    x: float,
    y: float,
    z: float,
    radius: float = SCREW_HEAD_R,
    length: float = SCREW_HEAD_H,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z)),
        material="black_oxide",
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_linear_extension_unit")

    model.material("body_alloy", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("carriage_gray", rgba=(0.48, 0.51, 0.55, 1.0))
    model.material("stage_steel", rgba=(0.79, 0.81, 0.83, 1.0))
    model.material("polymer_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("black_oxide", rgba=(0.13, 0.13, 0.15, 1.0))

    outer_body = model.part("outer_body")
    _add_box(outer_body, name="rear_block", size=(BODY_REAR_BLOCK_L, BODY_W, BODY_H), x0=0.0, material="body_alloy")
    _add_box(outer_body, name="base_deck", size=(BODY_BASE_L, BODY_BASE_W, BODY_BASE_H), x0=BODY_REAR_BLOCK_L, z=BODY_BASE_Z, material="body_alloy")
    _add_box(outer_body, name="left_wall", size=(BODY_SIDE_WALL_L, BODY_SIDE_WALL_W, BODY_SIDE_WALL_H), x0=BODY_REAR_BLOCK_L, y=BODY_SIDE_WALL_Y, material="body_alloy")
    _add_box(outer_body, name="right_wall", size=(BODY_SIDE_WALL_L, BODY_SIDE_WALL_W, BODY_SIDE_WALL_H), x0=BODY_REAR_BLOCK_L, y=-BODY_SIDE_WALL_Y, material="body_alloy")
    _add_box(outer_body, name="left_way", size=(BODY_WAY_L, BODY_WAY_W, BODY_WAY_H), x0=BODY_WAY_X, y=BODY_WAY_Y, z=BODY_WAY_Z, material="stage_steel")
    _add_box(outer_body, name="right_way", size=(BODY_WAY_L, BODY_WAY_W, BODY_WAY_H), x0=BODY_WAY_X, y=-BODY_WAY_Y, z=BODY_WAY_Z, material="stage_steel")
    _add_box(outer_body, name="front_left_cheek", size=(BODY_FRONT_CHEEK_L, BODY_FRONT_CHEEK_W, BODY_FRONT_CHEEK_H), x0=BODY_L - BODY_FRONT_CHEEK_L, y=BODY_FRONT_CHEEK_Y, material="body_alloy")
    _add_box(outer_body, name="front_right_cheek", size=(BODY_FRONT_CHEEK_L, BODY_FRONT_CHEEK_W, BODY_FRONT_CHEEK_H), x0=BODY_L - BODY_FRONT_CHEEK_L, y=-BODY_FRONT_CHEEK_Y, material="body_alloy")
    _add_box(outer_body, name="front_top_bridge", size=(BODY_FRONT_CHEEK_L, BODY_FRONT_TOP_W, BODY_FRONT_TOP_H), x0=BODY_L - BODY_FRONT_CHEEK_L, z=BODY_FRONT_TOP_Z, material="body_alloy")
    _add_box(outer_body, name="rear_cover_left", size=(BODY_COVER_REAR_L, BODY_COVER_W, BODY_COVER_H), x0=BODY_COVER_REAR_X, y=BODY_COVER_Y, z=BODY_COVER_Z, material="body_alloy")
    _add_box(outer_body, name="rear_cover_right", size=(BODY_COVER_REAR_L, BODY_COVER_W, BODY_COVER_H), x0=BODY_COVER_REAR_X, y=-BODY_COVER_Y, z=BODY_COVER_Z, material="body_alloy")
    _add_box(outer_body, name="front_cover_left", size=(BODY_COVER_FRONT_L, BODY_COVER_W, BODY_COVER_H), x0=BODY_COVER_FRONT_X, y=BODY_COVER_Y, z=BODY_COVER_Z, material="body_alloy")
    _add_box(outer_body, name="front_cover_right", size=(BODY_COVER_FRONT_L, BODY_COVER_W, BODY_COVER_H), x0=BODY_COVER_FRONT_X, y=-BODY_COVER_Y, z=BODY_COVER_Z, material="body_alloy")
    _add_box(outer_body, name="rear_foot", size=(BODY_FOOT_L, BODY_FOOT_W, BODY_FOOT_H), x0=BODY_REAR_FOOT_X, z=BODY_FOOT_Z, material="polymer_black")
    _add_box(outer_body, name="front_foot", size=(BODY_FOOT_L, BODY_FOOT_W, BODY_FOOT_H), x0=BODY_FRONT_FOOT_X, z=BODY_FOOT_Z, material="polymer_black")
    _add_screw_head(outer_body, name="body_screw_1", x=0.088, y=-BODY_COVER_Y, z=0.031 - (SCREW_HEAD_H / 2.0) + VISUAL_OVERLAP)
    _add_screw_head(outer_body, name="body_screw_2", x=0.088, y=BODY_COVER_Y, z=0.031 - (SCREW_HEAD_H / 2.0) + VISUAL_OVERLAP)
    _add_screw_head(outer_body, name="body_screw_3", x=0.232, y=-BODY_COVER_Y, z=0.030 - (SCREW_HEAD_H / 2.0) + VISUAL_OVERLAP)
    _add_screw_head(outer_body, name="body_screw_4", x=0.232, y=BODY_COVER_Y, z=0.030 - (SCREW_HEAD_H / 2.0) + VISUAL_OVERLAP)

    carriage = model.part("carriage_sleeve")
    _add_box(carriage, name="top_bridge", size=(CARR_L, CARR_TOP_W, CARR_TOP_H), x0=0.0, z=CARR_TOP_Z, material="carriage_gray")
    _add_box(carriage, name="left_wall", size=(CARR_L, CARR_WALL_W, CARR_WALL_H), x0=0.0, y=CARR_WALL_Y, z=CARR_WALL_Z, material="carriage_gray")
    _add_box(carriage, name="right_wall", size=(CARR_L, CARR_WALL_W, CARR_WALL_H), x0=0.0, y=-CARR_WALL_Y, z=CARR_WALL_Z, material="carriage_gray")
    _add_box(carriage, name="left_runner", size=(CARR_RUNNER_L, CARR_RUNNER_W, CARR_RUNNER_H), x0=CARR_RUNNER_X, y=CARR_RUNNER_Y, z=CARR_RUNNER_Z, material="stage_steel")
    _add_box(carriage, name="right_runner", size=(CARR_RUNNER_L, CARR_RUNNER_W, CARR_RUNNER_H), x0=CARR_RUNNER_X, y=-CARR_RUNNER_Y, z=CARR_RUNNER_Z, material="stage_steel")
    _add_box(carriage, name="rear_pad", size=(CARR_PAD_T, CARR_PAD_W, CARR_PAD_H), x0=0.0, z=CARR_PAD_Z, material="polymer_black")
    _add_box(carriage, name="front_pad", size=(CARR_PAD_T, CARR_PAD_W, CARR_PAD_H), x0=CARR_L - CARR_PAD_T, z=CARR_PAD_Z, material="polymer_black")
    _add_screw_head(carriage, name="carriage_screw_1", x=0.054, y=-0.017, z=0.032 - (SCREW_HEAD_H / 2.0) + VISUAL_OVERLAP)
    _add_screw_head(carriage, name="carriage_screw_2", x=0.054, y=0.017, z=0.032 - (SCREW_HEAD_H / 2.0) + VISUAL_OVERLAP)
    _add_screw_head(carriage, name="carriage_screw_3", x=0.116, y=-0.017, z=0.032 - (SCREW_HEAD_H / 2.0) + VISUAL_OVERLAP)
    _add_screw_head(carriage, name="carriage_screw_4", x=0.116, y=0.017, z=0.032 - (SCREW_HEAD_H / 2.0) + VISUAL_OVERLAP)

    nose = model.part("nose_stage")
    _add_box(nose, name="main_bar", size=(NOSE_L, NOSE_BAR_W, NOSE_BAR_H), x0=0.0, z=NOSE_BAR_Z, material="stage_steel")
    _add_box(nose, name="rear_block", size=(NOSE_REAR_BLOCK_L, NOSE_REAR_BLOCK_W, NOSE_REAR_BLOCK_H), x0=0.0, z=NOSE_BAR_Z, material="stage_steel")
    _add_box(nose, name="front_block", size=(NOSE_FRONT_PAD_L, NOSE_FRONT_PAD_W, NOSE_FRONT_PAD_H), x0=NOSE_L - NOSE_FRONT_PAD_L, z=NOSE_FRONT_PAD_Z, material="stage_steel")
    _add_box(nose, name="left_rib", size=(NOSE_RIB_L, NOSE_RIB_W, NOSE_RIB_H), x0=NOSE_RIB_X, y=NOSE_RIB_Y, z=NOSE_RIB_Z, material="stage_steel")
    _add_box(nose, name="right_rib", size=(NOSE_RIB_L, NOSE_RIB_W, NOSE_RIB_H), x0=NOSE_RIB_X, y=-NOSE_RIB_Y, z=NOSE_RIB_Z, material="stage_steel")
    _add_box(nose, name="tip_pad", size=(NOSE_TIP_PAD_T, NOSE_TIP_PAD_W, NOSE_TIP_PAD_H), x0=NOSE_L - NOSE_TIP_PAD_T, z=NOSE_TIP_PAD_Z, material="polymer_black")
    _add_screw_head(nose, name="nose_screw_1", x=NOSE_L - 0.010, y=-0.010, z=0.024 - (0.0035 / 2.0) + VISUAL_OVERLAP, radius=0.0035, length=0.0035)
    _add_screw_head(nose, name="nose_screw_2", x=NOSE_L - 0.010, y=0.010, z=0.024 - (0.0035 / 2.0) + VISUAL_OVERLAP, radius=0.0035, length=0.0035)

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=carriage,
        origin=Origin(xyz=(BODY_TO_CARRIAGE_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=OUTER_TRAVEL, effort=320.0, velocity=0.35),
    )
    model.articulation(
        "carriage_to_nose",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=nose,
        origin=Origin(xyz=(CARRIAGE_TO_NOSE_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=INNER_TRAVEL, effort=180.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    outer_body = object_model.get_part("outer_body")
    carriage = object_model.get_part("carriage_sleeve")
    nose = object_model.get_part("nose_stage")
    outer_slide = object_model.get_articulation("body_to_carriage")
    inner_slide = object_model.get_articulation("carriage_to_nose")

    left_way = outer_body.get_visual("left_way")
    right_way = outer_body.get_visual("right_way")
    left_runner = carriage.get_visual("left_runner")
    right_runner = carriage.get_visual("right_runner")
    left_wall = carriage.get_visual("left_wall")
    right_wall = carriage.get_visual("right_wall")
    left_rib = nose.get_visual("left_rib")
    right_rib = nose.get_visual("right_rib")

    def _span(aabb, axis: int) -> float:
        return aabb[1][axis] - aabb[0][axis]

    body_aabb = ctx.part_world_aabb(outer_body)
    carriage_aabb = ctx.part_world_aabb(carriage)
    nose_aabb = ctx.part_world_aabb(nose)
    nested_scale_ok = (
        body_aabb is not None
        and carriage_aabb is not None
        and nose_aabb is not None
        and _span(body_aabb, 1) > _span(carriage_aabb, 1) > _span(nose_aabb, 1)
        and _span(body_aabb, 2) > _span(carriage_aabb, 2) > _span(nose_aabb, 2)
    )
    ctx.check(
        "nested_member_scale_order",
        nested_scale_ok,
        "Expected outer body cross-section > carriage sleeve cross-section > nose stage cross-section.",
    )

    slides_ok = (
        outer_slide.articulation_type == ArticulationType.PRISMATIC
        and inner_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(inner_slide.axis) == (1.0, 0.0, 0.0)
        and outer_slide.motion_limits is not None
        and inner_slide.motion_limits is not None
        and outer_slide.motion_limits.lower == 0.0
        and inner_slide.motion_limits.lower == 0.0
        and outer_slide.motion_limits.upper is not None
        and inner_slide.motion_limits.upper is not None
        and outer_slide.motion_limits.upper > 0.05
        and inner_slide.motion_limits.upper > 0.04
    )
    ctx.check(
        "serial_prismatic_slides_aligned",
        slides_ok,
        "Both joints should be prismatic, coaxial, and extend in the same positive slide direction.",
    )

    ctx.expect_within(carriage, outer_body, axes="yz", margin=0.0, name="carriage_nested_in_body_cross_section")
    ctx.expect_within(nose, carriage, axes="yz", margin=0.0, name="nose_nested_in_carriage_cross_section")
    ctx.expect_contact(outer_body, carriage, elem_a=left_way, elem_b=left_runner, name="left_outer_guide_contact")
    ctx.expect_contact(outer_body, carriage, elem_a=right_way, elem_b=right_runner, name="right_outer_guide_contact")
    ctx.expect_contact(carriage, nose, elem_a=left_wall, elem_b=left_rib, name="left_inner_guide_contact")
    ctx.expect_contact(carriage, nose, elem_a=right_wall, elem_b=right_rib, name="right_inner_guide_contact")

    with ctx.pose({outer_slide: outer_slide.motion_limits.upper}):
        ctx.expect_within(carriage, outer_body, axes="yz", margin=0.0, name="carriage_stays_guided_at_outer_extension")
        ctx.expect_overlap(carriage, outer_body, axes="x", min_overlap=0.12, name="carriage_retains_body_engagement")
        ctx.expect_contact(outer_body, carriage, elem_a=left_way, elem_b=left_runner, name="left_outer_guide_contact_at_extension")
        ctx.expect_contact(outer_body, carriage, elem_a=right_way, elem_b=right_runner, name="right_outer_guide_contact_at_extension")

    with ctx.pose({inner_slide: inner_slide.motion_limits.upper}):
        ctx.expect_within(nose, carriage, axes="yz", margin=0.0, name="nose_stays_guided_at_inner_extension")
        ctx.expect_overlap(nose, carriage, axes="x", min_overlap=0.05, name="nose_retains_carriage_engagement")
        ctx.expect_contact(carriage, nose, elem_a=left_wall, elem_b=left_rib, name="left_inner_guide_contact_at_extension")
        ctx.expect_contact(carriage, nose, elem_a=right_wall, elem_b=right_rib, name="right_inner_guide_contact_at_extension")

    with ctx.pose({outer_slide: outer_slide.motion_limits.upper, inner_slide: inner_slide.motion_limits.upper}):
        ctx.expect_contact(outer_body, carriage, elem_a=left_way, elem_b=left_runner, name="left_outer_guide_contact_full_extension")
        ctx.expect_contact(outer_body, carriage, elem_a=right_way, elem_b=right_runner, name="right_outer_guide_contact_full_extension")
        ctx.expect_contact(carriage, nose, elem_a=left_wall, elem_b=left_rib, name="left_inner_guide_contact_full_extension")
        ctx.expect_contact(carriage, nose, elem_a=right_wall, elem_b=right_rib, name="right_inner_guide_contact_full_extension")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
