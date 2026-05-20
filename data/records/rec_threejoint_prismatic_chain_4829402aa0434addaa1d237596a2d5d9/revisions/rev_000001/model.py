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


OUTER_LEN = 0.74
OUTER_W = 0.16
OUTER_FOOT_H = 0.012
OUTER_BASE_T = 0.010
OUTER_RAIL_T = 0.022
OUTER_RAIL_H = 0.028
OUTER_END_BLOCK_L = 0.055
OUTER_JOINT_X = 0.080
OUTER_FLOOR_Z = OUTER_FOOT_H + OUTER_BASE_T
TRAVEL_1 = 0.20

STAGE1_LEN = 0.46
STAGE1_CARRIAGE_W = 0.192
STAGE1_WALL_T = 0.010
STAGE1_WALL_H = 0.044
STAGE1_BODY_W = 0.172
STAGE1_BODY_T = 0.014
STAGE1_SHOE_T = 0.006
STAGE1_BODY_Z = STAGE1_SHOE_T
STAGE1_GUIDE_X = 0.092
STAGE1_GUIDE_LEN = 0.30
STAGE1_GUIDE_W = 0.094
STAGE1_GUIDE_BASE_T = 0.008
STAGE1_GUIDE_RAIL_T = 0.015
STAGE1_GUIDE_RAIL_H = 0.020
STAGE1_GUIDE_BASE_Z = 0.028
STAGE1_GUIDE_FLOOR_Z = STAGE1_GUIDE_BASE_Z + STAGE1_GUIDE_BASE_T
STAGE1_RISER_LEN = 0.040
STAGE1_RISER_W = 0.060
TRAVEL_2 = 0.15

STAGE2_LEN = 0.30
STAGE2_CARRIAGE_W = 0.122
STAGE2_WALL_T = 0.008
STAGE2_WALL_H = 0.034
STAGE2_BODY_W = 0.106
STAGE2_BODY_T = 0.012
STAGE2_SHOE_T = 0.006
STAGE2_BODY_Z = STAGE2_SHOE_T
STAGE2_GUIDE_X = 0.062
STAGE2_GUIDE_LEN = 0.20
STAGE2_GUIDE_W = 0.070
STAGE2_GUIDE_BASE_T = 0.006
STAGE2_GUIDE_RAIL_T = 0.011
STAGE2_GUIDE_RAIL_H = 0.017
STAGE2_GUIDE_BASE_Z = 0.022
STAGE2_GUIDE_FLOOR_Z = STAGE2_GUIDE_BASE_Z + STAGE2_GUIDE_BASE_T
STAGE2_RISER_LEN = 0.034
STAGE2_RISER_W = 0.046
TRAVEL_3 = 0.11

STAGE3_LEN = 0.18
STAGE3_CARRIAGE_W = 0.094
STAGE3_WALL_T = 0.007
STAGE3_WALL_H = 0.028
STAGE3_BODY_W = 0.080
STAGE3_BODY_T = 0.010
STAGE3_SHOE_T = 0.005
STAGE3_BODY_Z = STAGE3_SHOE_T
STAGE3_PAD_LEN = 0.060
STAGE3_PAD_W = 0.082
STAGE3_PAD_T = 0.010
STAGE3_STEM_LEN = 0.030
STAGE3_STEM_W = 0.030
STAGE3_STEM_H = 0.012
STAGE3_STEM_Z = 0.018
STAGE3_PAD_Z = STAGE3_STEM_Z + STAGE3_STEM_H

OUTER_STAGE1_Z = OUTER_FLOOR_Z + OUTER_RAIL_H
STAGE1_STAGE2_Z = STAGE1_GUIDE_FLOOR_Z + STAGE1_GUIDE_RAIL_H
STAGE2_STAGE3_Z = STAGE2_GUIDE_FLOOR_Z + STAGE2_GUIDE_RAIL_H

EPS = 1e-6


def _box(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, False))
        .translate((x, y, z))
    )


def _outer_guide_shape() -> cq.Workplane:
    rail_y = OUTER_W / 2.0 - OUTER_RAIL_T / 2.0
    foot_y = OUTER_W / 2.0 - 0.034 / 2.0

    shape = _box(OUTER_LEN, OUTER_W, OUTER_BASE_T, z=OUTER_FOOT_H)
    shape = shape.union(_box(0.11, 0.034, OUTER_FOOT_H, x=0.02, y=foot_y))
    shape = shape.union(_box(0.11, 0.034, OUTER_FOOT_H, x=0.02, y=-foot_y))
    shape = shape.union(_box(0.11, 0.034, OUTER_FOOT_H, x=OUTER_LEN - 0.13, y=foot_y))
    shape = shape.union(_box(0.11, 0.034, OUTER_FOOT_H, x=OUTER_LEN - 0.13, y=-foot_y))

    shape = shape.union(
        _box(
            OUTER_LEN - 0.02,
            OUTER_RAIL_T,
            OUTER_RAIL_H,
            x=0.01,
            y=rail_y,
            z=OUTER_FLOOR_Z,
        )
    )
    shape = shape.union(
        _box(
            OUTER_LEN - 0.02,
            OUTER_RAIL_T,
            OUTER_RAIL_H,
            x=0.01,
            y=-rail_y,
            z=OUTER_FLOOR_Z,
        )
    )
    shape = shape.union(
        _box(
            OUTER_END_BLOCK_L,
            OUTER_W - 2.0 * OUTER_RAIL_T,
            OUTER_RAIL_H * 0.92,
            z=OUTER_FLOOR_Z,
        )
    )

    return shape


def _stage1_shape() -> cq.Workplane:
    carriage_wall_y = STAGE1_CARRIAGE_W / 2.0 - STAGE1_WALL_T / 2.0
    parent_rail_y = OUTER_W / 2.0 - OUTER_RAIL_T / 2.0
    guide_rail_y = STAGE1_GUIDE_W / 2.0 - STAGE1_GUIDE_RAIL_T / 2.0

    shape = _box(STAGE1_LEN, STAGE1_BODY_W, STAGE1_BODY_T, z=STAGE1_BODY_Z)
    shape = shape.union(
        _box(STAGE1_LEN, STAGE1_WALL_T, STAGE1_WALL_H, y=carriage_wall_y, z=STAGE1_BODY_Z)
    )
    shape = shape.union(
        _box(STAGE1_LEN, STAGE1_WALL_T, STAGE1_WALL_H, y=-carriage_wall_y, z=STAGE1_BODY_Z)
    )
    shape = shape.union(
        _box(
            STAGE1_LEN - 0.12,
            OUTER_RAIL_T - 0.004,
            STAGE1_SHOE_T,
            x=0.06,
            y=parent_rail_y,
        )
    )
    shape = shape.union(
        _box(
            STAGE1_LEN - 0.12,
            OUTER_RAIL_T - 0.004,
            STAGE1_SHOE_T,
            x=0.06,
            y=-parent_rail_y,
        )
    )
    shape = shape.union(
        _box(
            STAGE1_GUIDE_LEN,
            STAGE1_GUIDE_W,
            STAGE1_GUIDE_BASE_T,
            x=STAGE1_GUIDE_X,
            z=STAGE1_GUIDE_BASE_Z,
        )
    )
    shape = shape.union(
        _box(
            STAGE1_RISER_LEN,
            STAGE1_RISER_W,
            STAGE1_GUIDE_BASE_Z - (STAGE1_BODY_Z + STAGE1_BODY_T),
            x=STAGE1_GUIDE_X + 0.030,
            z=STAGE1_BODY_Z + STAGE1_BODY_T,
        )
    )
    shape = shape.union(
        _box(
            STAGE1_RISER_LEN,
            STAGE1_RISER_W,
            STAGE1_GUIDE_BASE_Z - (STAGE1_BODY_Z + STAGE1_BODY_T),
            x=STAGE1_GUIDE_X + STAGE1_GUIDE_LEN - STAGE1_RISER_LEN - 0.030,
            z=STAGE1_BODY_Z + STAGE1_BODY_T,
        )
    )
    shape = shape.union(
        _box(
            STAGE1_GUIDE_LEN,
            STAGE1_GUIDE_RAIL_T,
            STAGE1_GUIDE_RAIL_H,
            x=STAGE1_GUIDE_X,
            y=guide_rail_y,
            z=STAGE1_GUIDE_FLOOR_Z,
        )
    )
    shape = shape.union(
        _box(
            STAGE1_GUIDE_LEN,
            STAGE1_GUIDE_RAIL_T,
            STAGE1_GUIDE_RAIL_H,
            x=STAGE1_GUIDE_X,
            y=-guide_rail_y,
            z=STAGE1_GUIDE_FLOOR_Z,
        )
    )

    return shape


def _stage2_shape() -> cq.Workplane:
    carriage_wall_y = STAGE2_CARRIAGE_W / 2.0 - STAGE2_WALL_T / 2.0
    parent_rail_y = STAGE1_GUIDE_W / 2.0 - STAGE1_GUIDE_RAIL_T / 2.0
    guide_rail_y = STAGE2_GUIDE_W / 2.0 - STAGE2_GUIDE_RAIL_T / 2.0

    shape = _box(STAGE2_LEN, STAGE2_BODY_W, STAGE2_BODY_T, z=STAGE2_BODY_Z)
    shape = shape.union(
        _box(STAGE2_LEN, STAGE2_WALL_T, STAGE2_WALL_H, y=carriage_wall_y, z=STAGE2_BODY_Z)
    )
    shape = shape.union(
        _box(STAGE2_LEN, STAGE2_WALL_T, STAGE2_WALL_H, y=-carriage_wall_y, z=STAGE2_BODY_Z)
    )
    shape = shape.union(
        _box(
            STAGE2_LEN - 0.08,
            STAGE1_GUIDE_RAIL_T - 0.004,
            STAGE2_SHOE_T,
            x=0.04,
            y=parent_rail_y,
        )
    )
    shape = shape.union(
        _box(
            STAGE2_LEN - 0.08,
            STAGE1_GUIDE_RAIL_T - 0.004,
            STAGE2_SHOE_T,
            x=0.04,
            y=-parent_rail_y,
        )
    )
    shape = shape.union(
        _box(
            STAGE2_GUIDE_LEN,
            STAGE2_GUIDE_W,
            STAGE2_GUIDE_BASE_T,
            x=STAGE2_GUIDE_X,
            z=STAGE2_GUIDE_BASE_Z,
        )
    )
    shape = shape.union(
        _box(
            STAGE2_RISER_LEN,
            STAGE2_RISER_W,
            STAGE2_GUIDE_BASE_Z - (STAGE2_BODY_Z + STAGE2_BODY_T),
            x=STAGE2_GUIDE_X + 0.024,
            z=STAGE2_BODY_Z + STAGE2_BODY_T,
        )
    )
    shape = shape.union(
        _box(
            STAGE2_RISER_LEN,
            STAGE2_RISER_W,
            STAGE2_GUIDE_BASE_Z - (STAGE2_BODY_Z + STAGE2_BODY_T),
            x=STAGE2_GUIDE_X + STAGE2_GUIDE_LEN - STAGE2_RISER_LEN - 0.024,
            z=STAGE2_BODY_Z + STAGE2_BODY_T,
        )
    )
    shape = shape.union(
        _box(
            STAGE2_GUIDE_LEN,
            STAGE2_GUIDE_RAIL_T,
            STAGE2_GUIDE_RAIL_H,
            x=STAGE2_GUIDE_X,
            y=guide_rail_y,
            z=STAGE2_GUIDE_FLOOR_Z,
        )
    )
    shape = shape.union(
        _box(
            STAGE2_GUIDE_LEN,
            STAGE2_GUIDE_RAIL_T,
            STAGE2_GUIDE_RAIL_H,
            x=STAGE2_GUIDE_X,
            y=-guide_rail_y,
            z=STAGE2_GUIDE_FLOOR_Z,
        )
    )

    return shape


def _stage3_shape() -> cq.Workplane:
    carriage_wall_y = STAGE3_CARRIAGE_W / 2.0 - STAGE3_WALL_T / 2.0
    parent_rail_y = STAGE2_GUIDE_W / 2.0 - STAGE2_GUIDE_RAIL_T / 2.0

    shape = _box(STAGE3_LEN, STAGE3_BODY_W, STAGE3_BODY_T, z=STAGE3_BODY_Z)
    shape = shape.union(
        _box(STAGE3_LEN, STAGE3_WALL_T, STAGE3_WALL_H, y=carriage_wall_y, z=STAGE3_BODY_Z)
    )
    shape = shape.union(
        _box(STAGE3_LEN, STAGE3_WALL_T, STAGE3_WALL_H, y=-carriage_wall_y, z=STAGE3_BODY_Z)
    )
    shape = shape.union(
        _box(
            STAGE3_LEN - 0.06,
            STAGE2_GUIDE_RAIL_T - 0.003,
            STAGE3_SHOE_T,
            x=0.03,
            y=parent_rail_y,
        )
    )
    shape = shape.union(
        _box(
            STAGE3_LEN - 0.06,
            STAGE2_GUIDE_RAIL_T - 0.003,
            STAGE3_SHOE_T,
            x=0.03,
            y=-parent_rail_y,
        )
    )
    shape = shape.union(
        _box(
            STAGE3_STEM_LEN,
            STAGE3_STEM_W,
            STAGE3_STEM_H,
            x=STAGE3_LEN - STAGE3_STEM_LEN - 0.018,
            z=STAGE3_STEM_Z,
        )
    )
    shape = shape.union(
        _box(
            STAGE3_PAD_LEN,
            STAGE3_PAD_W,
            STAGE3_PAD_T,
            x=STAGE3_LEN - STAGE3_PAD_LEN,
            z=STAGE3_PAD_Z,
        )
    )

    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_linear_stack")

    model.material("frame_charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("stage_graphite", rgba=(0.33, 0.35, 0.39, 1.0))
    model.material("stage_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("tip_blue", rgba=(0.18, 0.39, 0.70, 1.0))

    outer = model.part("outer_guide")
    outer.visual(
        mesh_from_cadquery(_outer_guide_shape(), "outer_guide"),
        material="frame_charcoal",
        name="outer_shell",
    )

    stage1 = model.part("stage1")
    stage1.visual(
        mesh_from_cadquery(_stage1_shape(), "stage1"),
        material="stage_graphite",
        name="stage1_shell",
    )

    stage2 = model.part("stage2")
    stage2.visual(
        mesh_from_cadquery(_stage2_shape(), "stage2"),
        material="stage_silver",
        name="stage2_shell",
    )

    stage3 = model.part("stage3_tip")
    stage3.visual(
        mesh_from_cadquery(_stage3_shape(), "stage3_tip"),
        material="tip_blue",
        name="stage3_shell",
    )

    model.articulation(
        "outer_to_stage1",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=stage1,
        origin=Origin(xyz=(OUTER_JOINT_X, 0.0, OUTER_STAGE1_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.25,
            lower=0.0,
            upper=TRAVEL_1,
        ),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(STAGE1_GUIDE_X, 0.0, STAGE1_STAGE2_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.25,
            lower=0.0,
            upper=TRAVEL_2,
        ),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.PRISMATIC,
        parent=stage2,
        child=stage3,
        origin=Origin(xyz=(STAGE2_GUIDE_X, 0.0, STAGE2_STAGE3_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=TRAVEL_3,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_guide")
    stage1 = object_model.get_part("stage1")
    stage2 = object_model.get_part("stage2")
    stage3 = object_model.get_part("stage3_tip")

    slide_1 = object_model.get_articulation("outer_to_stage1")
    slide_2 = object_model.get_articulation("stage1_to_stage2")
    slide_3 = object_model.get_articulation("stage2_to_stage3")

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
        "three_serial_prismatic_joints",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            for joint in (slide_1, slide_2, slide_3)
        ),
        "All three moving stages must be prismatic joints.",
    )
    ctx.check(
        "all_slides_extend_along_positive_x",
        all(joint.axis == (1.0, 0.0, 0.0) for joint in (slide_1, slide_2, slide_3)),
        "Every stage should translate along the common +X extension axis.",
    )

    with ctx.pose({slide_1: 0.0, slide_2: 0.0, slide_3: 0.0}):
        ctx.expect_origin_gap(
            stage1,
            outer,
            axis="z",
            min_gap=OUTER_STAGE1_Z - EPS,
            max_gap=OUTER_STAGE1_Z + EPS,
            name="stage1_mount_plane_above_outer_rails",
        )
        ctx.expect_origin_gap(
            stage2,
            stage1,
            axis="z",
            min_gap=STAGE1_STAGE2_Z - EPS,
            max_gap=STAGE1_STAGE2_Z + EPS,
            name="stage2_mount_plane_above_stage1_rails",
        )
        ctx.expect_origin_gap(
            stage3,
            stage2,
            axis="z",
            min_gap=STAGE2_STAGE3_Z - EPS,
            max_gap=STAGE2_STAGE3_Z + EPS,
            name="stage3_mount_plane_above_stage2_rails",
        )

        ctx.expect_origin_distance(
            stage1,
            outer,
            axes="y",
            min_dist=0.0,
            max_dist=EPS,
            name="stage1_centered_on_outer_guide",
        )
        ctx.expect_origin_distance(
            stage2,
            stage1,
            axes="y",
            min_dist=0.0,
            max_dist=EPS,
            name="stage2_centered_on_stage1",
        )
        ctx.expect_origin_distance(
            stage3,
            stage2,
            axes="y",
            min_dist=0.0,
            max_dist=EPS,
            name="stage3_centered_on_stage2",
        )
        ctx.expect_contact(
            stage1,
            outer,
            contact_tol=5e-4,
            name="stage1_contacts_outer_guide",
        )
        ctx.expect_contact(
            stage2,
            stage1,
            contact_tol=5e-4,
            name="stage2_contacts_stage1",
        )
        ctx.expect_contact(
            stage3,
            stage2,
            contact_tol=5e-4,
            name="stage3_contacts_stage2",
        )
        ctx.expect_overlap(
            stage1,
            outer,
            axes="x",
            min_overlap=0.40,
            name="stage1_has_rest_overlap_with_outer",
        )
        ctx.expect_overlap(
            stage2,
            stage1,
            axes="x",
            min_overlap=0.28,
            name="stage2_has_rest_overlap_with_stage1",
        )
        ctx.expect_overlap(
            stage3,
            stage2,
            axes="x",
            min_overlap=0.17,
            name="stage3_has_rest_overlap_with_stage2",
        )

    with ctx.pose({slide_1: TRAVEL_1, slide_2: TRAVEL_2, slide_3: TRAVEL_3}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_full_extension")
        ctx.expect_origin_gap(
            stage1,
            outer,
            axis="x",
            min_gap=OUTER_JOINT_X + TRAVEL_1 - EPS,
            max_gap=OUTER_JOINT_X + TRAVEL_1 + EPS,
            name="stage1_reaches_full_extension",
        )
        ctx.expect_origin_gap(
            stage2,
            stage1,
            axis="x",
            min_gap=STAGE1_GUIDE_X + TRAVEL_2 - EPS,
            max_gap=STAGE1_GUIDE_X + TRAVEL_2 + EPS,
            name="stage2_reaches_full_extension",
        )
        ctx.expect_origin_gap(
            stage3,
            stage2,
            axis="x",
            min_gap=STAGE2_GUIDE_X + TRAVEL_3 - EPS,
            max_gap=STAGE2_GUIDE_X + TRAVEL_3 + EPS,
            name="stage3_reaches_full_extension",
        )
        ctx.expect_overlap(
            stage1,
            outer,
            axes="x",
            min_overlap=0.40,
            name="stage1_retains_outer_bearing_overlap",
        )
        ctx.expect_overlap(
            stage2,
            stage1,
            axes="x",
            min_overlap=0.20,
            name="stage2_retains_stage1_bearing_overlap",
        )
        ctx.expect_overlap(
            stage3,
            stage2,
            axes="x",
            min_overlap=0.10,
            name="stage3_retains_stage2_bearing_overlap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
