from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import ArticulatedObject, ArticulationType, Box, MotionLimits, Origin, TestContext, TestReport


BASE_LENGTH = 0.74
BASE_RAIL_Y = 0.057
BASE_RAIL_W = 0.024
BASE_RAIL_H = 0.026
BASE_CROSSBAR_L = 0.07
BASE_CROSSBAR_W = 0.156
BASE_CROSSBAR_H = 0.018
BASE_SUPPORT_Z = BASE_RAIL_H

STAGE1_LENGTH = 0.48
STAGE1_WIDTH = 0.135
STAGE1_SHOE_L = 0.34
STAGE1_SHOE_W = 0.016
STAGE1_SHOE_H = 0.008
STAGE1_RISER_W = 0.020
STAGE1_RISER_H = 0.010
STAGE1_DECK_W = 0.11
STAGE1_DECK_T = 0.014
STAGE1_TOP_RAIL_H = 0.012
STAGE1_RAIL_Y = 0.034
STAGE1_SUPPORT_Z = STAGE1_SHOE_H + STAGE1_RISER_H + STAGE1_DECK_T + STAGE1_TOP_RAIL_H

STAGE2_LENGTH = 0.34
STAGE2_WIDTH = 0.095
STAGE2_SHOE_L = 0.25
STAGE2_SHOE_W = 0.012
STAGE2_SHOE_H = 0.006
STAGE2_RISER_W = 0.012
STAGE2_RISER_H = 0.008
STAGE2_DECK_W = 0.078
STAGE2_DECK_T = 0.012
STAGE2_TOP_RAIL_H = 0.010
STAGE2_RAIL_Y = 0.022
STAGE2_SUPPORT_Z = STAGE2_SHOE_H + STAGE2_RISER_H + STAGE2_DECK_T + STAGE2_TOP_RAIL_H

STAGE3_LENGTH = 0.23
STAGE3_WIDTH = 0.06
STAGE3_SHOE_L = 0.16
STAGE3_SHOE_W = 0.010
STAGE3_SHOE_H = 0.006
STAGE3_RISER_W = 0.016
STAGE3_RISER_H = 0.006
STAGE3_DECK_T = 0.012


def add_box(
    part,
    *,
    name: str,
    material: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_service_fixture")

    model.material("base_paint", rgba=(0.26, 0.28, 0.30, 1.0))
    model.material("stage_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("stage_metal_dark", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("fixture_pad", rgba=(0.16, 0.32, 0.54, 1.0))

    guide_body = model.part("guide_body")
    add_box(
        guide_body,
        name="left_rail",
        material="base_paint",
        size=(BASE_LENGTH, BASE_RAIL_W, BASE_RAIL_H),
        center=(BASE_LENGTH / 2, -BASE_RAIL_Y, BASE_RAIL_H / 2),
    )
    add_box(
        guide_body,
        name="right_rail",
        material="base_paint",
        size=(BASE_LENGTH, BASE_RAIL_W, BASE_RAIL_H),
        center=(BASE_LENGTH / 2, BASE_RAIL_Y, BASE_RAIL_H / 2),
    )
    add_box(
        guide_body,
        name="front_crossbar",
        material="base_paint",
        size=(BASE_CROSSBAR_L, BASE_CROSSBAR_W, BASE_CROSSBAR_H),
        center=(0.06, 0.0, BASE_CROSSBAR_H / 2),
    )
    add_box(
        guide_body,
        name="rear_crossbar",
        material="base_paint",
        size=(BASE_CROSSBAR_L, BASE_CROSSBAR_W, BASE_CROSSBAR_H),
        center=(BASE_LENGTH - 0.06, 0.0, BASE_CROSSBAR_H / 2),
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        add_box(
            guide_body,
            name=f"{side}_front_stop",
            material="base_paint",
            size=(0.022, 0.012, 0.038),
            center=(0.018, sign * 0.075, BASE_RAIL_H + 0.019),
        )
        add_box(
            guide_body,
            name=f"{side}_rear_stop",
            material="base_paint",
            size=(0.022, 0.012, 0.038),
            center=(BASE_LENGTH - 0.018, sign * 0.075, BASE_RAIL_H + 0.019),
        )

    stage1 = model.part("stage1_slider")
    for side, sign in (("left", -1.0), ("right", 1.0)):
        add_box(
            stage1,
            name=f"{side}_shoe",
            material="stage_metal_dark",
            size=(STAGE1_SHOE_L, STAGE1_SHOE_W, STAGE1_SHOE_H),
            center=(0.235, sign * BASE_RAIL_Y, STAGE1_SHOE_H / 2),
        )
        add_box(
            stage1,
            name=f"{side}_riser",
            material="stage_metal_dark",
            size=(0.40, STAGE1_RISER_W, STAGE1_RISER_H),
            center=(STAGE1_LENGTH / 2, sign * 0.045, STAGE1_SHOE_H + STAGE1_RISER_H / 2),
        )
    add_box(
        stage1,
        name="deck",
        material="stage_metal_dark",
        size=(STAGE1_LENGTH, STAGE1_DECK_W, STAGE1_DECK_T),
        center=(STAGE1_LENGTH / 2, 0.0, STAGE1_SHOE_H + STAGE1_RISER_H + STAGE1_DECK_T / 2),
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        add_box(
            stage1,
            name=f"{side}_top_rail",
            material="stage_metal_dark",
            size=(0.44, 0.008, STAGE1_TOP_RAIL_H),
            center=(STAGE1_LENGTH / 2, sign * STAGE1_RAIL_Y, STAGE1_SHOE_H + STAGE1_RISER_H + STAGE1_DECK_T + STAGE1_TOP_RAIL_H / 2),
        )
        add_box(
            stage1,
            name=f"{side}_front_lug",
            material="stage_metal_dark",
            size=(0.018, 0.008, 0.010),
            center=(0.018, sign * 0.037, STAGE1_SUPPORT_Z + 0.005),
        )
        add_box(
            stage1,
            name=f"{side}_rear_lug",
            material="stage_metal_dark",
            size=(0.018, 0.008, 0.010),
            center=(STAGE1_LENGTH - 0.018, sign * 0.037, STAGE1_SUPPORT_Z + 0.005),
        )

    stage2 = model.part("stage2_slider")
    for side, sign in (("left", -1.0), ("right", 1.0)):
        add_box(
            stage2,
            name=f"{side}_shoe",
            material="stage_metal",
            size=(STAGE2_SHOE_L, STAGE2_SHOE_W, STAGE2_SHOE_H),
            center=(0.17, sign * STAGE1_RAIL_Y, STAGE2_SHOE_H / 2),
        )
        add_box(
            stage2,
            name=f"{side}_riser",
            material="stage_metal",
            size=(0.28, STAGE2_RISER_W, STAGE2_RISER_H),
            center=(STAGE2_LENGTH / 2, sign * 0.023, STAGE2_SHOE_H + STAGE2_RISER_H / 2),
        )
    add_box(
        stage2,
        name="deck",
        material="stage_metal",
        size=(STAGE2_LENGTH, STAGE2_DECK_W, STAGE2_DECK_T),
        center=(STAGE2_LENGTH / 2, 0.0, STAGE2_SHOE_H + STAGE2_RISER_H + STAGE2_DECK_T / 2),
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        add_box(
            stage2,
            name=f"{side}_top_rail",
            material="stage_metal",
            size=(0.30, 0.008, STAGE2_TOP_RAIL_H),
            center=(STAGE2_LENGTH / 2, sign * STAGE2_RAIL_Y, STAGE2_SHOE_H + STAGE2_RISER_H + STAGE2_DECK_T + STAGE2_TOP_RAIL_H / 2),
        )
        add_box(
            stage2,
            name=f"{side}_front_lug",
            material="stage_metal",
            size=(0.016, 0.006, 0.008),
            center=(0.016, sign * 0.024, STAGE2_SUPPORT_Z + 0.004),
        )
        add_box(
            stage2,
            name=f"{side}_rear_lug",
            material="stage_metal",
            size=(0.016, 0.006, 0.008),
            center=(STAGE2_LENGTH - 0.016, sign * 0.024, STAGE2_SUPPORT_Z + 0.004),
        )

    stage3 = model.part("stage3_slider")
    for side, sign in (("left", -1.0), ("right", 1.0)):
        add_box(
            stage3,
            name=f"{side}_shoe",
            material="fixture_pad",
            size=(STAGE3_SHOE_L, STAGE3_SHOE_W, STAGE3_SHOE_H),
            center=(0.11, sign * STAGE2_RAIL_Y, STAGE3_SHOE_H / 2),
        )
        add_box(
            stage3,
            name=f"{side}_riser",
            material="fixture_pad",
            size=(0.18, STAGE3_RISER_W, STAGE3_RISER_H),
            center=(STAGE3_LENGTH / 2, sign * 0.015, STAGE3_SHOE_H + STAGE3_RISER_H / 2),
        )
    add_box(
        stage3,
        name="deck",
        material="fixture_pad",
        size=(STAGE3_LENGTH, STAGE3_WIDTH, STAGE3_DECK_T),
        center=(STAGE3_LENGTH / 2, 0.0, STAGE3_SHOE_H + STAGE3_RISER_H + STAGE3_DECK_T / 2),
    )
    add_box(
        stage3,
        name="tooling_pad",
        material="fixture_pad",
        size=(0.16, 0.068, 0.014),
        center=(STAGE3_LENGTH / 2, 0.0, STAGE3_SHOE_H + STAGE3_RISER_H + STAGE3_DECK_T + 0.007),
    )
    add_box(
        stage3,
        name="nose_stop",
        material="fixture_pad",
        size=(0.022, 0.05, 0.018),
        center=(STAGE3_LENGTH - 0.011, 0.0, STAGE3_SHOE_H + STAGE3_RISER_H + STAGE3_DECK_T + 0.009),
    )
    add_box(
        stage3,
        name="left_front_cleat",
        material="fixture_pad",
        size=(0.014, 0.012, 0.014),
        center=(0.019, -0.03, STAGE3_SHOE_H + STAGE3_RISER_H + STAGE3_DECK_T + 0.007),
    )
    add_box(
        stage3,
        name="right_front_cleat",
        material="fixture_pad",
        size=(0.014, 0.012, 0.014),
        center=(0.019, 0.03, STAGE3_SHOE_H + STAGE3_RISER_H + STAGE3_DECK_T + 0.007),
    )

    model.articulation(
        "guide_to_stage1",
        ArticulationType.PRISMATIC,
        parent=guide_body,
        child=stage1,
        origin=Origin(xyz=(0.0, 0.0, BASE_SUPPORT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.35, lower=0.0, upper=0.22),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(0.0, 0.0, STAGE1_SUPPORT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.30, lower=0.0, upper=0.10),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.PRISMATIC,
        parent=stage2,
        child=stage3,
        origin=Origin(xyz=(0.0, 0.0, STAGE2_SUPPORT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_body = object_model.get_part("guide_body")
    stage1 = object_model.get_part("stage1_slider")
    stage2 = object_model.get_part("stage2_slider")
    stage3 = object_model.get_part("stage3_slider")
    joint1 = object_model.get_articulation("guide_to_stage1")
    joint2 = object_model.get_articulation("stage1_to_stage2")
    joint3 = object_model.get_articulation("stage2_to_stage3")

    left_rail = guide_body.get_visual("left_rail")
    stage1_left_shoe = stage1.get_visual("left_shoe")
    stage2_left_shoe = stage2.get_visual("left_shoe")
    stage3_left_shoe = stage3.get_visual("left_shoe")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(stage1, guide_body, elem_a=stage1_left_shoe, elem_b=left_rail, name="stage1_carried_by_guide")
    ctx.expect_contact(stage2, stage1, elem_a=stage2_left_shoe, elem_b="left_top_rail", name="stage2_carried_by_stage1")
    ctx.expect_contact(stage3, stage2, elem_a=stage3_left_shoe, elem_b="left_top_rail", name="stage3_carried_by_stage2")

    ctx.expect_overlap(stage1, guide_body, axes="xy", min_overlap=0.10, name="stage1_overlaps_guide_footprint")
    ctx.expect_overlap(stage2, stage1, axes="xy", min_overlap=0.08, name="stage2_overlaps_stage1_footprint")
    ctx.expect_overlap(stage3, stage2, axes="xy", min_overlap=0.05, name="stage3_overlaps_stage2_footprint")

    with ctx.pose({joint1: 0.18, joint2: 0.08, joint3: 0.06}):
        ctx.expect_origin_gap(stage1, guide_body, axis="x", min_gap=0.179, max_gap=0.181, name="stage1_prismatic_axis")
        ctx.expect_origin_gap(stage2, stage1, axis="x", min_gap=0.079, max_gap=0.081, name="stage2_prismatic_axis")
        ctx.expect_origin_gap(stage3, stage2, axis="x", min_gap=0.059, max_gap=0.061, name="stage3_prismatic_axis")
        ctx.expect_overlap(stage3, stage2, axes="y", min_overlap=0.05, name="stage3_remains_laterally_supported")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
