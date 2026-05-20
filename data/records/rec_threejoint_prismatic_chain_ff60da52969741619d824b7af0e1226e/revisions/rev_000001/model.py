from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _rect_ring(part, prefix, outer_xy, bar, z, height, material):
    outer_x, outer_y = outer_xy
    half_x = outer_x / 2.0
    half_y = outer_y / 2.0
    _box(part, f"{prefix}_front", (outer_x, bar, height), (0.0, half_y - bar / 2.0, z), material)
    _box(part, f"{prefix}_rear", (outer_x, bar, height), (0.0, -half_y + bar / 2.0, z), material)
    _box(part, f"{prefix}_side_0", (bar, outer_y, height), (half_x - bar / 2.0, 0.0, z), material)
    _box(part, f"{prefix}_side_1", (bar, outer_y, height), (-half_x + bar / 2.0, 0.0, z), material)


def _four_posts(part, prefix, outer_xy, post, z_center, length, material):
    outer_x, outer_y = outer_xy
    for ix, x_sign in enumerate((-1.0, 1.0)):
        for iy, y_sign in enumerate((-1.0, 1.0)):
            _box(
                part,
                f"{prefix}_{ix}_{iy}",
                (post, post, length),
                (
                    x_sign * (outer_x / 2.0 - post / 2.0),
                    y_sign * (outer_y / 2.0 - post / 2.0),
                    z_center,
                ),
                material,
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_linear_stack")

    guide_mat = model.material("matte_black_anodized_guide", rgba=(0.02, 0.025, 0.03, 1.0))
    stage1_mat = model.material("brushed_aluminum_stage_1", rgba=(0.66, 0.68, 0.68, 1.0))
    stage2_mat = model.material("satin_steel_stage_2", rgba=(0.48, 0.52, 0.54, 1.0))
    stage3_mat = model.material("polished_steel_tip_stage", rgba=(0.78, 0.80, 0.78, 1.0))
    bearing_mat = model.material("dark_polymer_bearing_pads", rgba=(0.01, 0.012, 0.014, 1.0))
    accent_mat = model.material("red_travel_stop_marks", rgba=(0.85, 0.08, 0.04, 1.0))

    outer_guide = model.part("outer_guide")
    _box(outer_guide, "ground_foot", (0.36, 0.36, 0.04), (0.0, 0.0, 0.02), guide_mat)
    _rect_ring(outer_guide, "lower_collar", (0.192, 0.192), 0.026, 0.056, 0.032, guide_mat)
    _four_posts(outer_guide, "guide_post", (0.192, 0.192), 0.026, 0.255, 0.410, guide_mat)
    _rect_ring(outer_guide, "top_collar", (0.192, 0.192), 0.026, 0.460, 0.030, guide_mat)
    _box(outer_guide, "front_scale", (0.010, 0.004, 0.410), (0.0, 0.098, 0.255), accent_mat)

    stage_1 = model.part("stage_1")
    _four_posts(stage_1, "stage_1_post", (0.116, 0.116), 0.014, 0.030, 0.660, stage1_mat)
    _rect_ring(stage_1, "stage_1_lower_collar", (0.116, 0.116), 0.014, -0.240, 0.020, stage1_mat)
    _rect_ring(stage_1, "stage_1_upper_collar", (0.116, 0.116), 0.014, 0.352, 0.022, stage1_mat)
    _box(stage_1, "stage_1_front_bearing", (0.116, 0.012, 0.120), (0.0, 0.064, 0.000), bearing_mat)
    _box(stage_1, "stage_1_rear_bearing", (0.116, 0.012, 0.120), (0.0, -0.064, 0.000), bearing_mat)
    _box(stage_1, "stage_1_side_bearing_0", (0.012, 0.116, 0.120), (0.064, 0.0, 0.000), bearing_mat)
    _box(stage_1, "stage_1_side_bearing_1", (0.012, 0.116, 0.120), (-0.064, 0.0, 0.000), bearing_mat)

    stage_2 = model.part("stage_2")
    _four_posts(stage_2, "stage_2_post", (0.066, 0.066), 0.010, 0.035, 0.540, stage2_mat)
    _rect_ring(stage_2, "stage_2_lower_collar", (0.066, 0.066), 0.010, -0.205, 0.018, stage2_mat)
    _rect_ring(stage_2, "stage_2_upper_collar", (0.066, 0.066), 0.010, 0.292, 0.020, stage2_mat)
    _box(stage_2, "stage_2_front_bearing", (0.066, 0.012, 0.100), (0.0, 0.038, 0.000), bearing_mat)
    _box(stage_2, "stage_2_rear_bearing", (0.066, 0.012, 0.100), (0.0, -0.038, 0.000), bearing_mat)

    stage_3 = model.part("stage_3")
    _box(stage_3, "tip_blade", (0.026, 0.026, 0.420), (0.0, 0.0, 0.020), stage3_mat)
    _box(stage_3, "retainer_shoe_front", (0.032, 0.020, 0.330), (0.0, 0.013, -0.045), bearing_mat)
    _box(stage_3, "retainer_shoe_rear", (0.032, 0.020, 0.330), (0.0, -0.013, -0.045), bearing_mat)
    _box(stage_3, "flat_tip_cap", (0.050, 0.050, 0.020), (0.0, 0.0, 0.240), accent_mat)

    model.articulation(
        "guide_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.240),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.0, 0.0, 0.352)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=0.0, upper=0.210),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(0.0, 0.0, 0.292)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.25, lower=0.0, upper=0.180),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_guide = object_model.get_part("outer_guide")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")

    guide_to_stage_1 = object_model.get_articulation("guide_to_stage_1")
    stage_1_to_stage_2 = object_model.get_articulation("stage_1_to_stage_2")
    stage_2_to_stage_3 = object_model.get_articulation("stage_2_to_stage_3")

    for joint in (guide_to_stage_1, stage_1_to_stage_2, stage_2_to_stage_3):
        ctx.check(
            f"{joint.name} is prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"{joint.name} type={joint.articulation_type}",
        )

    ctx.expect_within(
        stage_1,
        outer_guide,
        axes="xy",
        margin=0.001,
        name="stage 1 nests inside the outer guide throat",
    )
    ctx.expect_within(
        stage_2,
        stage_1,
        axes="xy",
        margin=0.001,
        name="stage 2 nests inside stage 1 throat",
    )
    ctx.expect_within(
        stage_3,
        stage_2,
        axes="xy",
        margin=0.001,
        name="tip stage nests inside stage 2 throat",
    )

    rest_tip = ctx.part_world_position(stage_3)
    with ctx.pose(
        {
            guide_to_stage_1: 0.240,
            stage_1_to_stage_2: 0.210,
            stage_2_to_stage_3: 0.180,
        }
    ):
        extended_tip = ctx.part_world_position(stage_3)
        ctx.expect_overlap(
            stage_1,
            outer_guide,
            axes="z",
            min_overlap=0.012,
            name="stage 1 remains retained at full travel",
        )
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="z",
            min_overlap=0.012,
            name="stage 2 remains retained at full travel",
        )
        ctx.expect_overlap(
            stage_3,
            stage_2,
            axes="z",
            min_overlap=0.012,
            name="tip stage remains retained at full travel",
        )

    ctx.check(
        "serial stack extends upward",
        rest_tip is not None and extended_tip is not None and extended_tip[2] > rest_tip[2] + 0.55,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
