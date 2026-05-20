from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_rectangular_ram_stack")

    dark_cast = Material("dark_cast_housing", rgba=(0.08, 0.09, 0.10, 1.0))
    black = Material("black_wear_strips", rgba=(0.01, 0.012, 0.014, 1.0))
    stage0_mat = Material("oiled_steel_stage_0", rgba=(0.33, 0.36, 0.38, 1.0))
    stage1_mat = Material("brushed_steel_stage_1", rgba=(0.52, 0.55, 0.56, 1.0))
    stage2_mat = Material("polished_steel_stage_2", rgba=(0.72, 0.74, 0.73, 1.0))
    plate_mat = Material("flat_tip_plate", rgba=(0.18, 0.19, 0.19, 1.0))

    # Common ram axis is +X.  The base housing is a short, rigid, open-ended
    # rectangular guide box rather than a solid block, so the first sleeve can
    # sit visibly inside the aperture without colliding with the housing.
    base = model.part("base_housing")
    base.visual(Box((0.28, 0.26, 0.035)), origin=Origin(xyz=(-0.10, 0.0, 0.0825)), material=dark_cast, name="top_wall")
    base.visual(Box((0.28, 0.26, 0.035)), origin=Origin(xyz=(-0.10, 0.0, -0.0825)), material=dark_cast, name="bottom_wall")
    base.visual(Box((0.28, 0.035, 0.20)), origin=Origin(xyz=(-0.10, 0.1125, 0.0)), material=dark_cast, name="side_wall_0")
    base.visual(Box((0.28, 0.035, 0.20)), origin=Origin(xyz=(-0.10, -0.1125, 0.0)), material=dark_cast, name="side_wall_1")
    base.visual(Box((0.20, 0.31, 0.04)), origin=Origin(xyz=(-0.12, 0.0, -0.118)), material=dark_cast, name="mounting_foot")
    base.visual(Box((0.034, 0.31, 0.036)), origin=Origin(xyz=(0.045, 0.0, 0.118)), material=dark_cast, name="front_flange_top")
    base.visual(Box((0.034, 0.31, 0.036)), origin=Origin(xyz=(0.045, 0.0, -0.118)), material=dark_cast, name="front_flange_bottom")
    base.visual(Box((0.034, 0.036, 0.20)), origin=Origin(xyz=(0.045, 0.137, 0.0)), material=dark_cast, name="front_flange_side_0")
    base.visual(Box((0.034, 0.036, 0.20)), origin=Origin(xyz=(0.045, -0.137, 0.0)), material=dark_cast, name="front_flange_side_1")

    stage0 = model.part("sleeve_0")
    stage0.visual(Box((0.48, 0.145, 0.012)), origin=Origin(xyz=(0.06, 0.0, 0.0415)), material=stage0_mat, name="top_wall")
    stage0.visual(Box((0.48, 0.145, 0.012)), origin=Origin(xyz=(0.06, 0.0, -0.0415)), material=stage0_mat, name="bottom_wall")
    stage0.visual(Box((0.48, 0.012, 0.095)), origin=Origin(xyz=(0.06, 0.0665, 0.0)), material=stage0_mat, name="side_wall_0")
    stage0.visual(Box((0.48, 0.012, 0.095)), origin=Origin(xyz=(0.06, -0.0665, 0.0)), material=stage0_mat, name="side_wall_1")
    stage0.visual(Box((0.035, 0.151, 0.014)), origin=Origin(xyz=(0.292, 0.0, 0.045)), material=black, name="mouth_band_top")
    stage0.visual(Box((0.035, 0.151, 0.014)), origin=Origin(xyz=(0.292, 0.0, -0.045)), material=black, name="mouth_band_bottom")
    stage0.visual(Box((0.035, 0.014, 0.098)), origin=Origin(xyz=(0.292, 0.069, 0.0)), material=black, name="mouth_band_side_0")
    stage0.visual(Box((0.035, 0.014, 0.098)), origin=Origin(xyz=(0.292, -0.069, 0.0)), material=black, name="mouth_band_side_1")
    stage0.visual(Box((0.16, 0.060, 0.0185)), origin=Origin(xyz=(-0.09, 0.0, 0.05575)), material=black, name="base_guide_top")
    stage0.visual(Box((0.16, 0.060, 0.0185)), origin=Origin(xyz=(-0.09, 0.0, -0.05575)), material=black, name="base_guide_bottom")
    stage0.visual(Box((0.16, 0.0235, 0.050)), origin=Origin(xyz=(-0.09, 0.08325, 0.0)), material=black, name="base_guide_side_0")
    stage0.visual(Box((0.16, 0.0235, 0.050)), origin=Origin(xyz=(-0.09, -0.08325, 0.0)), material=black, name="base_guide_side_1")

    stage1 = model.part("sleeve_1")
    stage1.visual(Box((0.40, 0.105, 0.010)), origin=Origin(xyz=(0.02, 0.0, 0.0275)), material=stage1_mat, name="top_wall")
    stage1.visual(Box((0.40, 0.105, 0.010)), origin=Origin(xyz=(0.02, 0.0, -0.0275)), material=stage1_mat, name="bottom_wall")
    stage1.visual(Box((0.40, 0.010, 0.065)), origin=Origin(xyz=(0.02, 0.0475, 0.0)), material=stage1_mat, name="side_wall_0")
    stage1.visual(Box((0.40, 0.010, 0.065)), origin=Origin(xyz=(0.02, -0.0475, 0.0)), material=stage1_mat, name="side_wall_1")
    stage1.visual(Box((0.030, 0.111, 0.012)), origin=Origin(xyz=(0.215, 0.0, 0.036)), material=black, name="mouth_band_top")
    stage1.visual(Box((0.030, 0.111, 0.012)), origin=Origin(xyz=(0.215, 0.0, -0.036)), material=black, name="mouth_band_bottom")
    stage1.visual(Box((0.030, 0.012, 0.068)), origin=Origin(xyz=(0.215, 0.051, 0.0)), material=black, name="mouth_band_side_0")
    stage1.visual(Box((0.030, 0.012, 0.068)), origin=Origin(xyz=(0.215, -0.051, 0.0)), material=black, name="mouth_band_side_1")
    stage1.visual(Box((0.11, 0.045, 0.004)), origin=Origin(xyz=(-0.115, 0.0, 0.0335)), material=black, name="outer_guide_top")
    stage1.visual(Box((0.11, 0.045, 0.004)), origin=Origin(xyz=(-0.115, 0.0, -0.0335)), material=black, name="outer_guide_bottom")
    stage1.visual(Box((0.11, 0.009, 0.034)), origin=Origin(xyz=(-0.115, 0.056, 0.0)), material=black, name="outer_guide_side_0")
    stage1.visual(Box((0.11, 0.009, 0.034)), origin=Origin(xyz=(-0.115, -0.056, 0.0)), material=black, name="outer_guide_side_1")

    stage2 = model.part("sleeve_2")
    stage2.visual(Box((0.34, 0.073, 0.007)), origin=Origin(xyz=(0.01, 0.0, 0.015)), material=stage2_mat, name="top_wall")
    stage2.visual(Box((0.34, 0.073, 0.007)), origin=Origin(xyz=(0.01, 0.0, -0.015)), material=stage2_mat, name="bottom_wall")
    stage2.visual(Box((0.34, 0.007, 0.037)), origin=Origin(xyz=(0.01, 0.033, 0.0)), material=stage2_mat, name="side_wall_0")
    stage2.visual(Box((0.34, 0.007, 0.037)), origin=Origin(xyz=(0.01, -0.033, 0.0)), material=stage2_mat, name="side_wall_1")
    stage2.visual(Box((0.09, 0.036, 0.005)), origin=Origin(xyz=(-0.105, 0.0, 0.020)), material=black, name="middle_guide_top")
    stage2.visual(Box((0.09, 0.036, 0.005)), origin=Origin(xyz=(-0.105, 0.0, -0.020)), material=black, name="middle_guide_bottom")
    stage2.visual(Box((0.09, 0.007, 0.022)), origin=Origin(xyz=(-0.105, 0.039, 0.0)), material=black, name="middle_guide_side_0")
    stage2.visual(Box((0.09, 0.007, 0.022)), origin=Origin(xyz=(-0.105, -0.039, 0.0)), material=black, name="middle_guide_side_1")
    stage2.visual(Box((0.020, 0.135, 0.090)), origin=Origin(xyz=(0.190, 0.0, 0.0)), material=plate_mat, name="end_plate")

    model.articulation(
        "base_to_sleeve_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage0,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=0.0, upper=0.12),
    )
    model.articulation(
        "sleeve_0_to_sleeve_1",
        ArticulationType.PRISMATIC,
        parent=stage0,
        child=stage1,
        origin=Origin(xyz=(0.28, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.18, lower=0.0, upper=0.11),
    )
    model.articulation(
        "sleeve_1_to_sleeve_2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.18, lower=0.0, upper=0.09),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_housing")
    stage0 = object_model.get_part("sleeve_0")
    stage1 = object_model.get_part("sleeve_1")
    stage2 = object_model.get_part("sleeve_2")
    j0 = object_model.get_articulation("base_to_sleeve_0")
    j1 = object_model.get_articulation("sleeve_0_to_sleeve_1")
    j2 = object_model.get_articulation("sleeve_1_to_sleeve_2")

    ctx.check(
        "three common-axis prismatic stages",
        all(j.articulation_type == ArticulationType.PRISMATIC and tuple(j.axis) == (1.0, 0.0, 0.0) for j in (j0, j1, j2)),
        details=f"axes={[j.axis for j in (j0, j1, j2)]}",
    )

    ctx.expect_overlap(stage0, base, axes="x", min_overlap=0.05, name="outer sleeve remains inserted in base")
    ctx.expect_overlap(stage1, stage0, axes="x", min_overlap=0.08, name="middle sleeve remains inserted in outer sleeve")
    ctx.expect_overlap(stage2, stage1, axes="x", min_overlap=0.08, name="inner sleeve remains inserted in middle sleeve")
    ctx.expect_within(stage1, stage0, axes="yz", margin=0.0, inner_elem="top_wall", name="middle sleeve body fits within outer sleeve profile")
    ctx.expect_within(stage2, stage1, axes="yz", margin=0.0, inner_elem="top_wall", name="inner sleeve body fits within middle sleeve profile")

    rest_tip = ctx.part_element_world_aabb(stage2, elem="end_plate")
    with ctx.pose({j0: 0.12, j1: 0.11, j2: 0.09}):
        ctx.expect_overlap(stage0, base, axes="x", min_overlap=0.05, name="extended outer sleeve keeps base engagement")
        ctx.expect_overlap(stage1, stage0, axes="x", min_overlap=0.08, name="extended middle sleeve keeps outer engagement")
        ctx.expect_overlap(stage2, stage1, axes="x", min_overlap=0.08, name="extended inner sleeve keeps middle engagement")
        extended_tip = ctx.part_element_world_aabb(stage2, elem="end_plate")

    ctx.check(
        "end plate advances along ram axis",
        rest_tip is not None and extended_tip is not None and extended_tip[1][0] > rest_tip[1][0] + 0.25,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
