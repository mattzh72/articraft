from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
from pathlib import Path

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_drill_press", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.30, 0.33, 0.36, 1.0))
    machine_blue = model.material("machine_blue", rgba=(0.22, 0.37, 0.56, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.15, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.80, 0.56, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="base_slab",
    )
    base.visual(
        Box((0.24, 0.24, 0.10)),
        origin=Origin(xyz=(-0.22, 0.0, 0.13)),
        material=cast_iron,
        name="column_plinth",
    )
    base.visual(
        Box((0.16, 0.16, 0.22)),
        origin=Origin(xyz=(0.18, 0.0, 0.19)),
        material=cast_iron,
        name="table_pedestal",
    )
    base.visual(
        Box((0.34, 0.24, 0.03)),
        origin=Origin(xyz=(0.18, 0.0, 0.315)),
        material=cast_iron,
        name="work_table",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.085, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_trim,
        name="base_flange",
    )
    column.visual(
        Cylinder(radius=0.055, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=machine_blue,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.075, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.97)),
        material=dark_trim,
        name="top_cap",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.035, 0.18, 0.24)),
        origin=Origin(xyz=(-0.0725, 0.0, 0.0)),
        material=machine_blue,
        name="rear_clamp",
    )
    carriage.visual(
        Box((0.15, 0.035, 0.24)),
        origin=Origin(xyz=(0.02, 0.0725, 0.0)),
        material=machine_blue,
        name="side_clamp_left",
    )
    carriage.visual(
        Box((0.15, 0.035, 0.24)),
        origin=Origin(xyz=(0.02, -0.0725, 0.0)),
        material=machine_blue,
        name="side_clamp_right",
    )
    carriage.visual(
        Box((0.14, 0.18, 0.20)),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=machine_blue,
        name="front_housing",
    )
    carriage.visual(
        Box((0.48, 0.11, 0.014)),
        origin=Origin(xyz=(0.33, 0.0, -0.05)),
        material=dark_trim,
        name="arm_way_surface",
    )
    carriage.visual(
        Box((0.20, 0.11, 0.014)),
        origin=Origin(xyz=(0.18, 0.0, 0.057)),
        material=dark_trim,
        name="top_bridge",
    )
    carriage.visual(
        Box((0.26, 0.012, 0.08)),
        origin=Origin(xyz=(0.17, 0.051, 0.0)),
        material=dark_trim,
        name="guide_cheek_left",
    )
    carriage.visual(
        Box((0.26, 0.012, 0.08)),
        origin=Origin(xyz=(0.17, -0.051, 0.0)),
        material=dark_trim,
        name="guide_cheek_right",
    )

    arm = model.part("arm")
    arm.visual(
        Box((0.70, 0.09, 0.086)),
        origin=Origin(xyz=(0.55, 0.0, 0.0)),
        material=machine_blue,
        name="arm_beam",
    )
    arm.visual(
        Box((0.12, 0.088, 0.02)),
        origin=Origin(xyz=(0.28, 0.0, -0.033)),
        material=dark_trim,
        name="slide_block",
    )

    head = model.part("head")
    head.visual(
        Box((0.18, 0.16, 0.02)),
        origin=Origin(xyz=(0.09, 0.0, 0.053)),
        material=dark_trim,
        name="saddle_top",
    )
    head.visual(
        Box((0.18, 0.018, 0.12)),
        origin=Origin(xyz=(0.09, 0.054, -0.01)),
        material=dark_trim,
        name="side_cheek_left",
    )
    head.visual(
        Box((0.18, 0.018, 0.12)),
        origin=Origin(xyz=(0.09, -0.054, -0.01)),
        material=dark_trim,
        name="side_cheek_right",
    )
    head.visual(
        Box((0.12, 0.04, 0.04)),
        origin=Origin(xyz=(0.09, -0.067, 0.03)),
        material=dark_trim,
        name="motor_mount",
    )
    head.visual(
        Box((0.14, 0.09, 0.18)),
        origin=Origin(xyz=(0.10, -0.11, -0.07)),
        material=machine_blue,
        name="motor_box",
    )
    head.visual(
        Cylinder(radius=0.04, length=0.03),
        origin=Origin(xyz=(0.10, -0.11, 0.035)),
        material=machine_blue,
        name="motor_cap",
    )
    head.visual(
        Box((0.05, 0.08, 0.03)),
        origin=Origin(xyz=(0.08, 0.0, -0.08)),
        material=dark_trim,
        name="quill_cap",
    )
    head.visual(
        Box((0.03, 0.08, 0.14)),
        origin=Origin(xyz=(0.095, 0.0, -0.14)),
        material=machine_blue,
        name="quill_rear_web",
    )
    head.visual(
        Box((0.03, 0.08, 0.14)),
        origin=Origin(xyz=(0.165, 0.0, -0.14)),
        material=machine_blue,
        name="quill_front_web",
    )
    head.visual(
        Box((0.04, 0.01, 0.20)),
        origin=Origin(xyz=(0.13, 0.045, -0.10)),
        material=dark_trim,
        name="quill_guide_left",
    )
    head.visual(
        Box((0.04, 0.01, 0.20)),
        origin=Origin(xyz=(0.13, -0.045, -0.10)),
        material=dark_trim,
        name="quill_guide_right",
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.022, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=steel,
        name="quill_body",
    )
    quill.visual(
        Box((0.012, 0.02, 0.12)),
        origin=Origin(xyz=(0.0, 0.03, -0.06)),
        material=steel,
        name="guide_key_left",
    )
    quill.visual(
        Box((0.012, 0.02, 0.12)),
        origin=Origin(xyz=(0.0, -0.03, -0.06)),
        material=steel,
        name="guide_key_right",
    )
    quill.visual(
        Cylinder(radius=0.014, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
        material=steel,
        name="chuck",
    )
    quill.visual(
        Cylinder(radius=0.004, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, -0.245)),
        material=steel,
        name="drill_bit",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(-0.22, 0.0, 0.18)),
    )
    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.20,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.25,
            lower=0.0,
            upper=0.24,
        ),
    )
    model.articulation(
        "arm_to_head",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.52, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.28,
            lower=0.0,
            upper=0.26,
        ),
    )
    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.13, 0.0, -0.05)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.20,
            lower=0.0,
            upper=0.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")
    quill = object_model.get_part("quill")

    sleeve_lift = object_model.get_articulation("column_to_carriage")
    arm_slide = object_model.get_articulation("carriage_to_arm")
    head_slide = object_model.get_articulation("arm_to_head")
    quill_feed = object_model.get_articulation("head_to_quill")

    column_plinth = base.get_visual("column_plinth")
    work_table = base.get_visual("work_table")
    base_flange = column.get_visual("base_flange")
    column_shaft = column.get_visual("column_shaft")
    rear_clamp = carriage.get_visual("rear_clamp")
    carriage_way = carriage.get_visual("arm_way_surface")
    guide_cheek_left = carriage.get_visual("guide_cheek_left")
    arm_beam = arm.get_visual("arm_beam")
    head_saddle = head.get_visual("saddle_top")
    side_cheek_left = head.get_visual("side_cheek_left")
    quill_guide_left = head.get_visual("quill_guide_left")
    drill_bit = quill.get_visual("drill_bit")
    guide_key_left = quill.get_visual("guide_key_left")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.fail_if_isolated_parts(max_pose_samples=12)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    base_aabb = ctx.part_world_aabb(base)
    column_aabb = ctx.part_world_aabb(column)
    arm_aabb = ctx.part_world_aabb(arm)

    if base_aabb is not None:
        base_dx = base_aabb[1][0] - base_aabb[0][0]
        base_dy = base_aabb[1][1] - base_aabb[0][1]
        ctx.check(
            "wide_base_footprint",
            base_dx >= 0.75 and base_dy >= 0.52,
            f"base footprint was {base_dx:.3f} x {base_dy:.3f} m",
        )
    if column_aabb is not None:
        column_height = column_aabb[1][2] - column_aabb[0][2]
        ctx.check(
            "column_height_realistic",
            column_height >= 0.95,
            f"column height was {column_height:.3f} m",
        )
    if arm_aabb is not None:
        arm_length = arm_aabb[1][0] - arm_aabb[0][0]
        ctx.check(
            "radial_arm_reach_realistic",
            arm_length >= 0.70,
            f"arm length was {arm_length:.3f} m",
        )

    ctx.check(
        "carriage_lifts_vertically",
        carriage in (carriage,) and sleeve_lift.axis == (0.0, 0.0, 1.0),
        f"column_to_carriage axis was {sleeve_lift.axis}",
    )
    ctx.check(
        "arm_extends_horizontally",
        arm_slide.axis == (1.0, 0.0, 0.0),
        f"carriage_to_arm axis was {arm_slide.axis}",
    )
    ctx.check(
        "head_traverses_along_arm",
        head_slide.axis == (1.0, 0.0, 0.0),
        f"arm_to_head axis was {head_slide.axis}",
    )
    ctx.check(
        "quill_feeds_downward",
        quill_feed.axis == (0.0, 0.0, -1.0),
        f"head_to_quill axis was {quill_feed.axis}",
    )

    ctx.expect_within(column, base, axes="xy")
    ctx.expect_gap(
        column,
        base,
        axis="z",
        positive_elem=base_flange,
        negative_elem=column_plinth,
        max_gap=0.001,
        max_penetration=0.0,
        name="column_seats_on_base_plinth",
    )
    ctx.expect_contact(
        carriage,
        column,
        elem_a=rear_clamp,
        elem_b=column_shaft,
        name="carriage_clamps_column",
    )

    ctx.expect_overlap(carriage, column, axes="z", min_overlap=0.20)
    ctx.expect_overlap(arm, carriage, axes="xy", min_overlap=0.03)
    ctx.expect_gap(
        arm,
        carriage,
        axis="z",
        positive_elem=arm_beam,
        negative_elem=carriage_way,
        max_gap=0.001,
        max_penetration=0.0,
        name="arm_rides_on_carriage_way",
    )
    ctx.expect_contact(
        arm,
        carriage,
        elem_a=arm_beam,
        elem_b=guide_cheek_left,
        name="arm_guided_laterally_by_carriage",
    )
    ctx.expect_overlap(head, arm, axes="xy", min_overlap=0.04)
    ctx.expect_gap(
        head,
        arm,
        axis="z",
        positive_elem=head_saddle,
        negative_elem=arm_beam,
        max_gap=0.001,
        max_penetration=0.0,
        name="head_saddle_seats_on_arm",
    )
    ctx.expect_contact(
        head,
        arm,
        elem_a=side_cheek_left,
        elem_b=arm_beam,
        name="head_cheeks_capture_arm",
    )
    ctx.expect_contact(
        quill,
        head,
        elem_a=guide_key_left,
        elem_b=quill_guide_left,
        name="quill_guided_by_head_frame",
    )
    ctx.expect_gap(
        quill,
        base,
        axis="z",
        positive_elem=drill_bit,
        negative_elem=work_table,
        min_gap=0.02,
        max_gap=0.14,
        name="bit_hangs_above_work_table",
    )

    with ctx.pose({sleeve_lift: 0.18}):
        ctx.expect_gap(
            quill,
            base,
            axis="z",
            positive_elem=drill_bit,
            negative_elem=work_table,
            min_gap=0.18,
            name="raised_arm_lifts_tool_clear_of_table",
        )

    with ctx.pose({arm_slide: 0.24, head_slide: 0.26}):
        ctx.expect_overlap(arm, carriage, axes="xy", min_overlap=0.01)
        ctx.expect_overlap(head, arm, axes="xy", min_overlap=0.01)
        ctx.expect_gap(
            head,
            arm,
            axis="z",
            positive_elem=head_saddle,
            negative_elem=arm_beam,
            max_gap=0.001,
            max_penetration=0.0,
            name="head_remains_seated_when_fully_extended",
        )
        ctx.expect_gap(
            head,
            column,
            axis="x",
            min_gap=0.55,
            name="head_projects_far_forward_of_column",
        )
        ctx.fail_if_isolated_parts(name="full_reach_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="full_reach_no_overlap")

    with ctx.pose({quill_feed: 0.08}):
        ctx.expect_contact(
            quill,
            head,
            elem_a=guide_key_left,
            elem_b=quill_guide_left,
            name="quill_remains_guided_at_full_feed",
        )
        ctx.expect_gap(
            quill,
            base,
            axis="z",
            positive_elem=drill_bit,
            negative_elem=work_table,
            min_gap=0.015,
            max_gap=0.08,
            name="full_feed_still_clears_table",
        )
        ctx.fail_if_isolated_parts(name="full_feed_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="full_feed_no_overlap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
