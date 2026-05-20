from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_RADIUS = 0.100
BASE_FOOT_H = 0.020
BASE_THRUST_H = 0.006
BASE_TOP_Z = BASE_FOOT_H + BASE_THRUST_H
PEDESTAL_RADIUS = 0.040
PEDESTAL_H = 0.018

MAIN_SUPPORT_RADIUS = 0.068
MAIN_SUPPORT_H = 0.008
MAIN_PLATE_RADIUS = 0.090
MAIN_PLATE_H = 0.014
MAIN_STAGE_TOP_Z = MAIN_SUPPORT_H + MAIN_PLATE_H

ARM_THICK = 0.028
ARM_OFFSET_X = 0.110
UPPER_SEAT_RADIUS = 0.032
UPPER_SEAT_H = 0.010
UPPER_AXIS_Z = 0.108

UPPER_COLLAR_RADIUS = 0.024
UPPER_COLLAR_H = 0.012
UPPER_SPINDLE_RADIUS = 0.012
UPPER_SPINDLE_H = 0.046
UPPER_FLANGE_RADIUS = 0.040
UPPER_FLANGE_H = 0.006
UPPER_CAP_RADIUS = 0.016
UPPER_CAP_H = 0.010
INDEX_TAB_X = 0.026
INDEX_TAB_Y = 0.012
INDEX_TAB_H = 0.004


def _make_arm_body() -> cq.Workplane:
    column = cq.Workplane("XY").box(0.030, ARM_THICK, 0.082).translate((0.072, 0.0, 0.063))
    bridge = cq.Workplane("XY").box(0.074, ARM_THICK, 0.018).translate((0.108, 0.0, 0.095))
    cheek = cq.Workplane("XY").box(0.018, ARM_THICK, 0.036).translate((0.131, 0.0, 0.086))
    gusset_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.056, MAIN_STAGE_TOP_Z),
                (0.090, MAIN_STAGE_TOP_Z),
                (0.118, 0.086),
                (0.086, 0.086),
            ]
        )
        .close()
    )
    gusset = gusset_profile.extrude(ARM_THICK / 2.0, both=True)
    arm = column.union(bridge).union(cheek).union(gusset)
    return arm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_axis_rotary_stack", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    anodized_gray = model.material("anodized_gray", rgba=(0.40, 0.42, 0.46, 1.0))
    accent = model.material("accent", rgba=(0.74, 0.50, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_FOOT_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_H / 2.0)),
        material=dark_steel,
        name="foot",
    )
    base.visual(
        Cylinder(radius=0.075, length=BASE_THRUST_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_H + BASE_THRUST_H / 2.0)),
        material=machined_steel,
        name="thrust_ring",
    )
    base.visual(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z - PEDESTAL_H / 2.0)),
        material=anodized_gray,
        name="pedestal",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_FOOT_H),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_H / 2.0)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=MAIN_SUPPORT_RADIUS, length=MAIN_SUPPORT_H),
        origin=Origin(xyz=(0.0, 0.0, MAIN_SUPPORT_H / 2.0)),
        material=machined_steel,
        name="support_ring",
    )
    lower_stage.visual(
        Cylinder(radius=MAIN_PLATE_RADIUS, length=MAIN_PLATE_H),
        origin=Origin(xyz=(0.0, 0.0, MAIN_SUPPORT_H + MAIN_PLATE_H / 2.0)),
        material=anodized_gray,
        name="turntable_plate",
    )
    lower_stage.visual(
        mesh_from_cadquery(_make_arm_body(), "lower_arm_body.obj", assets=ASSETS),
        material=anodized_gray,
        name="arm_body",
    )
    lower_stage.visual(
        Cylinder(radius=UPPER_SEAT_RADIUS, length=UPPER_SEAT_H),
        origin=Origin(xyz=(ARM_OFFSET_X, 0.0, UPPER_AXIS_Z - UPPER_SEAT_H / 2.0)),
        material=machined_steel,
        name="upper_seat",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Box((0.170, 0.120, 0.120)),
        mass=2.4,
        origin=Origin(xyz=(0.050, 0.0, 0.060)),
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        Cylinder(radius=UPPER_COLLAR_RADIUS, length=UPPER_COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, UPPER_COLLAR_H / 2.0)),
        material=machined_steel,
        name="lower_collar",
    )
    upper_stage.visual(
        Cylinder(radius=UPPER_SPINDLE_RADIUS, length=UPPER_SPINDLE_H),
        origin=Origin(
            xyz=(0.0, 0.0, UPPER_COLLAR_H + UPPER_SPINDLE_H / 2.0)
        ),
        material=machined_steel,
        name="spindle",
    )
    upper_stage.visual(
        Cylinder(radius=UPPER_FLANGE_RADIUS, length=UPPER_FLANGE_H),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                UPPER_COLLAR_H + UPPER_SPINDLE_H + UPPER_FLANGE_H / 2.0,
            )
        ),
        material=accent,
        name="top_flange",
    )
    upper_stage.visual(
        Box((INDEX_TAB_X, INDEX_TAB_Y, INDEX_TAB_H)),
        origin=Origin(
            xyz=(
                UPPER_FLANGE_RADIUS + INDEX_TAB_X / 2.0,
                0.0,
                UPPER_COLLAR_H
                + UPPER_SPINDLE_H
                + UPPER_FLANGE_H
                - INDEX_TAB_H / 2.0,
            )
        ),
        material=accent,
        name="index_tab",
    )
    upper_stage.visual(
        Cylinder(radius=UPPER_CAP_RADIUS, length=UPPER_CAP_H),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                UPPER_COLLAR_H
                + UPPER_SPINDLE_H
                + UPPER_FLANGE_H
                + UPPER_CAP_H / 2.0,
            )
        ),
        material=machined_steel,
        name="cap",
    )
    upper_stage.inertial = Inertial.from_geometry(
        Box((0.100, 0.080, 0.080)),
        mass=0.9,
        origin=Origin(xyz=(0.018, 0.0, 0.040)),
    )

    model.articulation(
        "base_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5),
    )
    model.articulation(
        "upper_spin",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=upper_stage,
        origin=Origin(xyz=(ARM_OFFSET_X, 0.0, UPPER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-math.radians(120.0),
            upper=math.radians(120.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    base_spin = object_model.get_articulation("base_spin")
    upper_spin = object_model.get_articulation("upper_spin")

    base_ring = base.get_visual("thrust_ring")
    pedestal = base.get_visual("pedestal")
    lower_ring = lower_stage.get_visual("support_ring")
    arm_body = lower_stage.get_visual("arm_body")
    upper_seat = lower_stage.get_visual("upper_seat")
    upper_collar = upper_stage.get_visual("lower_collar")
    index_tab = upper_stage.get_visual("index_tab")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(
        lower_stage,
        base,
        elem_a=lower_ring,
        elem_b=base_ring,
        name="lower_stage_is_seated_on_base_ring",
    )
    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        max_gap=1e-6,
        max_penetration=0.0,
        positive_elem=lower_ring,
        negative_elem=base_ring,
        name="lower_stage_has_zero_vertical_gap_to_base_ring",
    )
    ctx.expect_overlap(
        lower_stage,
        base,
        axes="xy",
        min_overlap=0.130,
        elem_a=lower_ring,
        elem_b=base_ring,
        name="lower_stage_support_ring_overlaps_base_ring",
    )
    ctx.expect_origin_distance(
        lower_stage,
        base,
        axes="xy",
        max_dist=1e-6,
        name="lower_stage_axis_matches_main_base_axis",
    )

    ctx.expect_contact(
        upper_stage,
        lower_stage,
        elem_a=upper_collar,
        elem_b=upper_seat,
        name="upper_stage_is_seated_on_offset_seat",
    )
    ctx.expect_gap(
        upper_stage,
        lower_stage,
        axis="z",
        max_gap=1e-6,
        max_penetration=0.0,
        positive_elem=upper_collar,
        negative_elem=upper_seat,
        name="upper_stage_has_zero_vertical_gap_to_offset_seat",
    )
    ctx.expect_overlap(
        upper_stage,
        lower_stage,
        axes="xy",
        min_overlap=0.048,
        elem_a=upper_collar,
        elem_b=upper_seat,
        name="upper_stage_collar_overlaps_offset_seat",
    )
    ctx.expect_origin_distance(
        upper_stage,
        base,
        axes="xy",
        min_dist=ARM_OFFSET_X - 0.002,
        max_dist=ARM_OFFSET_X + 0.002,
        name="upper_axis_is_laterally_offset_from_main_axis",
    )
    ctx.expect_origin_gap(
        upper_stage,
        base,
        axis="z",
        min_gap=BASE_TOP_Z + UPPER_AXIS_Z - 0.002,
        max_gap=BASE_TOP_Z + UPPER_AXIS_Z + 0.002,
        name="upper_axis_is_raised_above_base",
    )

    with ctx.pose({base_spin: 0.0}):
        ctx.expect_gap(
            lower_stage,
            base,
            axis="x",
            min_gap=0.014,
            positive_elem=arm_body,
            negative_elem=pedestal,
            name="base_arm_starts_on_positive_x_side_at_zero_pose",
        )
    with ctx.pose({base_spin: math.pi / 2.0}):
        ctx.expect_gap(
            lower_stage,
            base,
            axis="y",
            min_gap=0.014,
            positive_elem=arm_body,
            negative_elem=pedestal,
            name="base_arm_swings_to_positive_y_side_after_quarter_turn",
        )
        ctx.expect_origin_distance(
            upper_stage,
            base,
            axes="xy",
            min_dist=ARM_OFFSET_X - 0.002,
            max_dist=ARM_OFFSET_X + 0.002,
            name="upper_axis_keeps_constant_radius_during_base_spin",
        )

    with ctx.pose({upper_spin: 0.0}):
        ctx.expect_gap(
            upper_stage,
            lower_stage,
            axis="x",
            min_gap=0.007,
            positive_elem=index_tab,
            negative_elem=upper_seat,
            name="upper_index_tab_points_along_positive_x_at_zero_pose",
        )
    with ctx.pose({upper_spin: math.pi / 2.0}):
        ctx.expect_gap(
            upper_stage,
            lower_stage,
            axis="y",
            min_gap=0.007,
            positive_elem=index_tab,
            negative_elem=upper_seat,
            name="upper_index_tab_points_along_positive_y_after_quarter_turn",
        )
        ctx.expect_overlap(
            upper_stage,
            lower_stage,
            axes="x",
            min_overlap=0.010,
            elem_a=index_tab,
            elem_b=upper_seat,
            name="upper_index_tab_crosses_seat_x_projection_after_turn",
        )

    with ctx.pose({base_spin: -1.8, upper_spin: 1.9}):
        ctx.expect_contact(
            lower_stage,
            base,
            elem_a=lower_ring,
            elem_b=base_ring,
            name="lower_stage_remains_seated_in_combined_pose",
        )
        ctx.expect_contact(
            upper_stage,
            lower_stage,
            elem_a=upper_collar,
            elem_b=upper_seat,
            name="upper_stage_remains_seated_in_combined_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
