from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_arm_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.29, 0.33, 0.36, 1.0))
    machine_green = model.material("machine_green", rgba=(0.23, 0.45, 0.37, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.72, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))

    base_column = model.part("base_column")
    base_column.visual(
        Cylinder(radius=0.23, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="base_disk",
    )
    base_column.visual(
        Cylinder(radius=0.14, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=cast_iron,
        name="base_riser",
    )
    base_column.visual(
        Cylinder(radius=0.055, length=0.69),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=steel,
        name="column",
    )
    base_column.visual(
        Cylinder(radius=0.078, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=dark_steel,
        name="table_sleeve",
    )
    base_column.visual(
        Box((0.13, 0.07, 0.05)),
        origin=Origin(xyz=(0.085, 0.0, 0.34)),
        material=cast_iron,
        name="table_bracket",
    )
    base_column.visual(
        Cylinder(radius=0.16, length=0.03),
        origin=Origin(xyz=(0.22, 0.0, 0.36)),
        material=dark_steel,
        name="work_table",
    )
    radial_arm = model.part("radial_arm")
    radial_arm.visual(
        Cylinder(radius=0.045, length=0.09),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machine_green,
        name="pivot_barrel",
    )
    radial_arm.visual(
        Box((0.46, 0.09, 0.07)),
        origin=Origin(xyz=(0.23, 0.0, 0.0)),
        material=machine_green,
        name="arm_beam",
    )
    radial_arm.visual(
        Box((0.03, 0.10, 0.08)),
        origin=Origin(xyz=(0.455, 0.0, 0.0)),
        material=dark_steel,
        name="arm_stop",
    )

    drill_head = model.part("drill_head")
    drill_head.visual(
        Box((0.12, 0.09, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=machine_green,
        name="carriage_top",
    )
    drill_head.visual(
        Box((0.12, 0.09, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        material=machine_green,
        name="carriage_bottom",
    )
    drill_head.visual(
        Box((0.12, 0.025, 0.13)),
        origin=Origin(xyz=(0.0, -0.0575, 0.0)),
        material=machine_green,
        name="carriage_left",
    )
    drill_head.visual(
        Box((0.12, 0.025, 0.13)),
        origin=Origin(xyz=(0.0, 0.0575, 0.0)),
        material=machine_green,
        name="carriage_right",
    )
    drill_head.visual(
        Box((0.13, 0.085, 0.18)),
        origin=Origin(xyz=(0.015, -0.16, -0.08)),
        material=machine_green,
        name="head_body",
    )
    drill_head.visual(
        Cylinder(radius=0.05, length=0.11),
        origin=Origin(xyz=(-0.005, -0.155, 0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_green,
        name="motor_housing",
    )
    drill_head.visual(
        Cylinder(radius=0.042, length=0.06),
        origin=Origin(xyz=(0.015, -0.10, -0.07)),
        material=dark_steel,
        name="quill_housing",
    )
    drill_head.visual(
        Box((0.04, 0.08, 0.03)),
        origin=Origin(xyz=(-0.04, -0.145, -0.145)),
        material=black,
        name="feed_case",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.022, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        material=steel,
        name="spindle_collar",
    )
    spindle.visual(
        Cylinder(radius=0.016, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.032, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.1675)),
        material=dark_steel,
        name="chuck_body",
    )
    spindle.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.209)),
        material=dark_steel,
        name="chuck_mid",
    )
    spindle.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=steel,
        name="chuck_tip",
    )

    model.articulation(
        "column_to_arm",
        ArticulationType.REVOLUTE,
        parent=base_column,
        child=radial_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.785)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.8,
            lower=0.0,
            upper=math.radians(45.0),
        ),
    )
    model.articulation(
        "arm_to_head",
        ArticulationType.PRISMATIC,
        parent=radial_arm,
        child=drill_head,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.12,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "head_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=drill_head,
        child=spindle,
        origin=Origin(xyz=(0.015, -0.10, -0.10)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_column = object_model.get_part("base_column")
    radial_arm = object_model.get_part("radial_arm")
    drill_head = object_model.get_part("drill_head")
    spindle = object_model.get_part("spindle")

    column_to_arm = object_model.get_articulation("column_to_arm")
    arm_to_head = object_model.get_articulation("arm_to_head")
    head_to_spindle = object_model.get_articulation("head_to_spindle")

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
        "primary_parts_present",
        all(part is not None for part in (base_column, radial_arm, drill_head, spindle)),
        "Expected base_column, radial_arm, drill_head, and spindle parts.",
    )
    ctx.check(
        "articulation_axes_match_mechanisms",
        column_to_arm.axis == (0.0, -1.0, 0.0)
        and arm_to_head.axis == (1.0, 0.0, 0.0)
        and head_to_spindle.axis == (0.0, 0.0, -1.0),
        "Arm should pivot on horizontal Y, head should slide along X, spindle should spin on vertical Z.",
    )

    ctx.expect_contact(radial_arm, base_column, name="arm_pivot_contacts_column")
    ctx.expect_contact(drill_head, radial_arm, name="head_carriage_contacts_arm")
    ctx.expect_contact(spindle, drill_head, name="spindle_contacts_head")
    ctx.expect_overlap(
        spindle,
        base_column,
        axes="xy",
        min_overlap=0.012,
        elem_a="chuck_tip",
        elem_b="work_table",
        name="spindle_overhangs_table",
    )
    ctx.expect_gap(
        spindle,
        base_column,
        axis="z",
        min_gap=0.008,
        max_gap=0.08,
        positive_elem="chuck_tip",
        negative_elem="work_table",
        name="spindle_tip_clears_table",
    )

    with ctx.pose({column_to_arm: math.radians(32.0), arm_to_head: 0.08, head_to_spindle: 1.3}):
        ctx.expect_contact(radial_arm, base_column, name="posed_arm_pivot_contacts_column")
        ctx.expect_contact(drill_head, radial_arm, name="posed_head_carriage_contacts_arm")
        ctx.expect_contact(spindle, drill_head, name="posed_spindle_contacts_head")
        ctx.expect_gap(
            spindle,
            base_column,
            axis="z",
            min_gap=0.16,
            positive_elem="chuck_tip",
            negative_elem="work_table",
            name="raised_arm_lifts_spindle_over_table",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
