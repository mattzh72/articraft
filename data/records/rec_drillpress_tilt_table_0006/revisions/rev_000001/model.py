from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import sys

ASSET_ROOT = "/"
_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir(ASSET_ROOT)
        return ASSET_ROOT


os.getcwd = _safe_getcwd
os.chdir(_safe_getcwd())
if "__file__" in globals() and not os.path.isabs(__file__):
    __file__ = f"{ASSET_ROOT.rstrip('/')}/{os.path.basename(__file__)}"
if sys.argv and sys.argv[0] and not os.path.isabs(sys.argv[0]):
    sys.argv[0] = f"{ASSET_ROOT.rstrip('/')}/{os.path.basename(sys.argv[0])}"

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="column_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.22, 0.23, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    machine_red = model.material("machine_red", rgba=(0.67, 0.12, 0.12, 1.0))
    table_gray = model.material("table_gray", rgba=(0.37, 0.38, 0.40, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.36, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.10, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, -0.09, 0.060)),
        material=cast_iron,
        name="column_plinth",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 0.045)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.025, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=steel,
        name="column_tube",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=0.78),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
    )

    head = model.part("head")
    head.visual(
        Box((0.12, 0.085, 0.14)),
        origin=Origin(xyz=(0.0, 0.0675, 0.0)),
        material=machine_red,
        name="column_clamp",
    )
    head.visual(
        Box((0.03, 0.14, 0.16)),
        origin=Origin(xyz=(-0.09, 0.18, 0.05)),
        material=machine_red,
        name="housing_left",
    )
    head.visual(
        Box((0.03, 0.14, 0.16)),
        origin=Origin(xyz=(0.09, 0.18, 0.05)),
        material=machine_red,
        name="housing_right",
    )
    head.visual(
        Box((0.18, 0.03, 0.16)),
        origin=Origin(xyz=(0.0, 0.125, 0.05)),
        material=machine_red,
        name="housing_rear",
    )
    head.visual(
        Box((0.18, 0.03, 0.14)),
        origin=Origin(xyz=(0.0, 0.245, 0.04)),
        material=machine_red,
        name="housing_front",
    )
    head.visual(
        Box((0.24, 0.14, 0.02)),
        origin=Origin(xyz=(0.0, 0.18, 0.14)),
        material=machine_red,
        name="speed_cover",
    )
    head.visual(
        Cylinder(radius=0.035, length=0.18),
        origin=Origin(xyz=(0.0, 0.18, -0.09)),
        material=steel,
        name="quill_housing",
    )
    head.visual(
        Box((0.18, 0.03, 0.10)),
        origin=Origin(xyz=(0.0, 0.135, -0.05)),
        material=machine_red,
        name="quill_bridge",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.24, 0.26, 0.32)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.16, 0.04)),
    )

    spindle = model.part("spindle")
    step_radii = (0.046, 0.041, 0.036, 0.031, 0.026)
    step_length = 0.012
    for idx, radius in enumerate(step_radii, start=1):
        center_z = step_length * (idx - 0.5)
        spindle.visual(
            Cylinder(radius=radius, length=step_length),
            origin=Origin(xyz=(0.0, 0.0, center_z)),
            material=steel,
            name=f"speed_step_{idx}",
        )
    spindle.visual(
        Cylinder(radius=0.011, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.020, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material=black,
        name="chuck_body",
    )
    spindle.visual(
        Cylinder(radius=0.0045, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, -0.275)),
        material=black,
        name="drill_bit",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.42),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )

    table_bracket = model.part("table_bracket")
    table_bracket.visual(
        Box((0.09, 0.08, 0.09)),
        origin=Origin(xyz=(0.0, 0.065, 0.0)),
        material=table_gray,
        name="bracket_clamp",
    )
    table_bracket.visual(
        Box((0.08, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.125, -0.040)),
        material=table_gray,
        name="support_arm",
    )
    table_bracket.visual(
        Box((0.008, 0.03, 0.06)),
        origin=Origin(xyz=(-0.035, 0.14, 0.0)),
        material=table_gray,
        name="left_ear",
    )
    table_bracket.visual(
        Box((0.008, 0.03, 0.06)),
        origin=Origin(xyz=(0.035, 0.14, 0.0)),
        material=table_gray,
        name="right_ear",
    )
    table_bracket.inertial = Inertial.from_geometry(
        Box((0.10, 0.18, 0.09)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.09, -0.005)),
    )

    work_table = model.part("work_table")
    work_table.visual(
        Cylinder(radius=0.018, length=0.062),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=table_gray,
        name="trunnion",
    )
    work_table.visual(
        Box((0.06, 0.12, 0.03)),
        origin=Origin(xyz=(0.0, 0.065, 0.015)),
        material=table_gray,
        name="table_neck",
    )
    work_table.visual(
        Cylinder(radius=0.12, length=0.02),
        origin=Origin(xyz=(0.0, 0.14, 0.03)),
        material=table_gray,
        name="table_disk",
    )
    work_table.inertial = Inertial.from_geometry(
        Cylinder(radius=0.12, length=0.02),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.14, 0.03)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, -0.09, 0.075)),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
    )
    model.articulation(
        "head_to_spindle",
        ArticulationType.REVOLUTE,
        parent=head,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.188, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=20.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "column_to_table_bracket",
        ArticulationType.FIXED,
        parent=column,
        child=table_bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
    )
    model.articulation(
        "bracket_to_work_table",
        ArticulationType.REVOLUTE,
        parent=table_bracket,
        child=work_table,
        origin=Origin(xyz=(0.0, 0.14, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.785,
            upper=0.785,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSET_ROOT)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    head = object_model.get_part("head")
    spindle = object_model.get_part("spindle")
    table_bracket = object_model.get_part("table_bracket")
    work_table = object_model.get_part("work_table")

    spindle_joint = object_model.get_articulation("head_to_spindle")
    table_hinge = object_model.get_articulation("bracket_to_work_table")

    base_plate = base.get_visual("base_plate")
    column_plinth = base.get_visual("column_plinth")
    column_tube = column.get_visual("column_tube")
    column_clamp = head.get_visual("column_clamp")
    quill_housing = head.get_visual("quill_housing")
    spindle_shaft = spindle.get_visual("spindle_shaft")
    drill_bit = spindle.get_visual("drill_bit")
    trunnion = work_table.get_visual("trunnion")
    table_disk = work_table.get_visual("table_disk")
    bracket_clamp = table_bracket.get_visual("bracket_clamp")
    left_ear = table_bracket.get_visual("left_ear")
    right_ear = table_bracket.get_visual("right_ear")

    speed_steps = [spindle.get_visual(f"speed_step_{idx}") for idx in range(1, 6)]

    ctx.allow_overlap(
        head,
        spindle,
        elem_a=quill_housing,
        elem_b=spindle_shaft,
        reason="The rotating spindle shaft is intentionally nested inside the fixed quill housing.",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    base_plate_aabb = ctx.part_element_world_aabb(base, elem=base_plate)
    if base_plate_aabb is not None:
        base_dx = base_plate_aabb[1][0] - base_plate_aabb[0][0]
        base_dy = base_plate_aabb[1][1] - base_plate_aabb[0][1]
        ctx.check(
            "square_base_planform",
            abs(base_dx - base_dy) <= 1e-6 and 0.35 <= base_dx <= 0.37,
            f"Expected a square base about 0.36 m wide, got {base_dx:.3f} x {base_dy:.3f} m.",
        )

    column_aabb = ctx.part_element_world_aabb(column, elem=column_tube)
    if column_aabb is not None:
        column_dx = column_aabb[1][0] - column_aabb[0][0]
        column_dy = column_aabb[1][1] - column_aabb[0][1]
        column_dz = column_aabb[1][2] - column_aabb[0][2]
        ctx.check(
            "round_column_proportions",
            abs(column_dx - column_dy) <= 1e-6 and 0.049 <= column_dx <= 0.051 and 0.77 <= column_dz <= 0.79,
            f"Expected a round steel column about 0.05 m diameter and 0.78 m tall, got {column_dx:.3f} x {column_dy:.3f} x {column_dz:.3f} m.",
        )

    ctx.check(
        "spindle_joint_is_vertical_revolute",
        spindle_joint.articulation_type == ArticulationType.REVOLUTE and spindle_joint.axis == (0.0, 0.0, 1.0),
        f"Expected a revolute spindle joint about +Z, got type={spindle_joint.articulation_type} axis={spindle_joint.axis}.",
    )
    ctx.check(
        "table_hinge_is_left_right_revolute",
        table_hinge.articulation_type == ArticulationType.REVOLUTE and table_hinge.axis == (1.0, 0.0, 0.0),
        f"Expected the tilting work table to hinge around +X, got type={table_hinge.articulation_type} axis={table_hinge.axis}.",
    )
    ctx.check(
        "five_speed_spindle_has_five_steps",
        len(speed_steps) == 5,
        f"Expected five spindle speed steps, got {len(speed_steps)}.",
    )

    ctx.expect_contact(column, base, elem_a=column_tube, elem_b=column_plinth)
    ctx.expect_gap(
        column,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=column_tube,
        negative_elem=column_plinth,
        name="column_seated_on_plinth",
    )
    ctx.expect_gap(
        head,
        column,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=column_clamp,
        negative_elem=column_tube,
        name="head_clamp_bears_on_column",
    )
    ctx.expect_gap(
        table_bracket,
        column,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=bracket_clamp,
        negative_elem=column_tube,
        name="table_bracket_bears_on_column",
    )
    ctx.expect_gap(
        spindle,
        head,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=speed_steps[0],
        negative_elem=quill_housing,
        name="speed_steps_start_above_quill_housing",
    )
    ctx.expect_overlap(spindle, work_table, axes="xy", min_overlap=0.02)
    ctx.expect_gap(
        spindle,
        work_table,
        axis="z",
        min_gap=0.010,
        max_gap=0.120,
        positive_elem=drill_bit,
        negative_elem=table_disk,
        name="drill_bit_hangs_above_table",
    )
    ctx.expect_gap(
        work_table,
        table_bracket,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=trunnion,
        negative_elem=left_ear,
        name="trunnion_seated_against_left_ear",
    )
    ctx.expect_gap(
        table_bracket,
        work_table,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_ear,
        negative_elem=trunnion,
        name="trunnion_seated_against_right_ear",
    )
    ctx.expect_contact(work_table, table_bracket, elem_a=trunnion, elem_b=left_ear)
    ctx.expect_contact(work_table, table_bracket, elem_a=trunnion, elem_b=right_ear)
    ctx.expect_origin_gap(head, work_table, axis="z", min_gap=0.30, name="head_above_table")

    for idx in range(4):
        ctx.expect_gap(
            spindle,
            spindle,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=speed_steps[idx + 1],
            negative_elem=speed_steps[idx],
            name=f"speed_step_{idx + 2}_stacked_on_step_{idx + 1}",
        )

    spindle_limits = spindle_joint.motion_limits
    if spindle_limits is not None and spindle_limits.lower is not None and spindle_limits.upper is not None:
        with ctx.pose({spindle_joint: spindle_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="spindle_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="spindle_lower_pose_no_floating")
        with ctx.pose({spindle_joint: spindle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="spindle_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="spindle_upper_pose_no_floating")

    table_limits = table_hinge.motion_limits
    if table_limits is not None and table_limits.lower is not None and table_limits.upper is not None:
        with ctx.pose({table_hinge: table_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="table_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="table_lower_pose_no_floating")
            ctx.expect_contact(work_table, table_bracket, elem_a=trunnion, elem_b=left_ear)
            ctx.expect_contact(work_table, table_bracket, elem_a=trunnion, elem_b=right_ear)
            ctx.expect_overlap(spindle, work_table, axes="x", min_overlap=0.02)
        with ctx.pose({table_hinge: table_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="table_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="table_upper_pose_no_floating")
            ctx.expect_contact(work_table, table_bracket, elem_a=trunnion, elem_b=left_ear)
            ctx.expect_contact(work_table, table_bracket, elem_a=trunnion, elem_b=right_ear)
            ctx.expect_overlap(spindle, work_table, axes="x", min_overlap=0.02)

    with ctx.pose({table_hinge: 0.5}):
        ctx.expect_gap(
            spindle,
            work_table,
            axis="z",
            min_gap=-0.08,
            max_gap=0.10,
            positive_elem=drill_bit,
            negative_elem=table_disk,
            name="tilted_table_stays_in_drilling_zone",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
