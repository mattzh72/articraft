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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.26, 0.27, 0.29, 1.0))
    machine_green = model.material("machine_green", rgba=(0.20, 0.33, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))
    black = model.material("black", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.26, 0.026)),
        origin=Origin(xyz=(0.0, 0.020, 0.013)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.18, 0.11, 0.032)),
        origin=Origin(xyz=(0.0, -0.055, 0.042)),
        material=cast_iron,
        name="rear_web",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(0.0, -0.060, 0.068)),
        material=cast_iron,
        name="column_boss",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.26, 0.026)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.020, 0.013)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.026, length=0.530),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=steel,
        name="column_shaft",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.530),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
    )

    head = model.part("head")
    head.visual(
        Box((0.110, 0.040, 0.160)),
        origin=Origin(xyz=(0.0, -0.046, 0.015)),
        material=machine_green,
        name="rear_clamp",
    )
    head.visual(
        Box((0.022, 0.160, 0.160)),
        origin=Origin(xyz=(0.037, 0.040, 0.015)),
        material=machine_green,
        name="left_cheek",
    )
    head.visual(
        Box((0.022, 0.160, 0.160)),
        origin=Origin(xyz=(-0.037, 0.040, 0.015)),
        material=machine_green,
        name="right_cheek",
    )
    head.visual(
        Box((0.185, 0.100, 0.110)),
        origin=Origin(xyz=(0.0, 0.100, 0.045)),
        material=machine_green,
        name="head_housing",
    )
    head.visual(
        Box((0.120, 0.100, 0.090)),
        origin=Origin(xyz=(0.0, -0.090, 0.075)),
        material=machine_green,
        name="motor_housing",
    )
    head.visual(
        Box((0.160, 0.110, 0.052)),
        origin=Origin(xyz=(0.0, 0.090, 0.126)),
        material=machine_green,
        name="belt_cover",
    )
    head.visual(
        Box((0.080, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.152, 0.000)),
        material=machine_green,
        name="quill_carriage",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, 0.175, -0.010)),
        material=black,
        name="quill_nose",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.185, 0.100, 0.110)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.100, 0.045)),
    )

    clamp = model.part("table_clamp")
    clamp.visual(
        Box((0.084, 0.028, 0.070)),
        origin=Origin(xyz=(0.0, -0.046, 0.0)),
        material=cast_iron,
        name="rear_bridge",
    )
    clamp.visual(
        Box((0.016, 0.102, 0.070)),
        origin=Origin(xyz=(0.034, 0.005, 0.0)),
        material=cast_iron,
        name="left_cheek",
    )
    clamp.visual(
        Box((0.016, 0.102, 0.070)),
        origin=Origin(xyz=(-0.034, 0.005, 0.0)),
        material=cast_iron,
        name="right_cheek",
    )
    clamp.visual(
        Box((0.010, 0.024, 0.040)),
        origin=Origin(xyz=(0.031, 0.068, 0.0)),
        material=cast_iron,
        name="left_ear",
    )
    clamp.visual(
        Box((0.010, 0.024, 0.040)),
        origin=Origin(xyz=(-0.031, 0.068, 0.0)),
        material=cast_iron,
        name="right_ear",
    )
    clamp.inertial = Inertial.from_geometry(
        Box((0.084, 0.028, 0.070)),
        mass=2.2,
        origin=Origin(xyz=(0.0, -0.046, 0.0)),
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.016, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="trunnion",
    )
    table.visual(
        Box((0.044, 0.120, 0.028)),
        origin=Origin(xyz=(0.0, 0.072, -0.002)),
        material=cast_iron,
        name="support_web",
    )
    table.visual(
        Cylinder(radius=0.100, length=0.016),
        origin=Origin(xyz=(0.0, 0.200, 0.006)),
        material=cast_iron,
        name="table_top",
    )
    table.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.120, -0.006)),
        material=cast_iron,
        name="underside_boss",
    )
    table.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.016),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.200, 0.006)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=steel,
        name="quill",
    )
    spindle.visual(
        Cylinder(radius=0.028, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.0825)),
        material=black,
        name="chuck_body",
    )
    spindle.visual(
        Cylinder(radius=0.005, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, -0.1475)),
        material=steel,
        name="drill_bit",
    )
    spindle.visual(
        Cylinder(radius=0.0055, length=0.018),
        origin=Origin(xyz=(0.033, 0.0, -0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="key_nub",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.060),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, -0.060, 0.078)),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
    )
    model.articulation(
        "column_to_table_clamp",
        ArticulationType.FIXED,
        parent=column,
        child=clamp,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )
    model.articulation(
        "head_to_spindle",
        ArticulationType.REVOLUTE,
        parent=head,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.175, -0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=10.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "clamp_to_table",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=table,
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-math.radians(20.0),
            upper=math.radians(20.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    head = object_model.get_part("head")
    clamp = object_model.get_part("table_clamp")
    table = object_model.get_part("table")
    spindle = object_model.get_part("spindle")

    spindle_spin = object_model.get_articulation("head_to_spindle")
    table_tilt = object_model.get_articulation("clamp_to_table")

    base_boss = base.get_visual("column_boss")
    column_shaft = column.get_visual("column_shaft")
    head_nose = head.get_visual("quill_nose")
    table_top = table.get_visual("table_top")
    spindle_quill = spindle.get_visual("quill")
    drill_bit = spindle.get_visual("drill_bit")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.check(
        "spindle_axis_is_vertical",
        spindle_spin.axis == (0.0, 0.0, 1.0),
        details=f"expected spindle axis (0, 0, 1), got {spindle_spin.axis}",
    )
    ctx.check(
        "table_axis_is_crosswise",
        table_tilt.axis == (1.0, 0.0, 0.0),
        details=f"expected table tilt axis (1, 0, 0), got {table_tilt.axis}",
    )
    spindle_limits = spindle_spin.motion_limits
    table_limits = table_tilt.motion_limits
    ctx.check(
        "spindle_has_full_turn_limits",
        spindle_limits is not None
        and spindle_limits.lower == -math.pi
        and spindle_limits.upper == math.pi,
        details="spindle should be able to rotate one full turn in either direction",
    )
    ctx.check(
        "table_has_realistic_tilt_limits",
        table_limits is not None
        and table_limits.lower == -math.radians(20.0)
        and table_limits.upper == math.radians(20.0),
        details="table should tilt about 20 degrees each way in this benchtop configuration",
    )

    ctx.expect_gap(
        column,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=column_shaft,
        negative_elem=base_boss,
    )
    ctx.expect_contact(head, column, contact_tol=1e-6)
    ctx.expect_contact(clamp, column, contact_tol=1e-6)
    ctx.expect_contact(head, spindle, elem_a=head_nose, elem_b=spindle_quill, contact_tol=1e-6)
    ctx.expect_contact(table, clamp, contact_tol=1e-6)
    ctx.expect_overlap(column, base, axes="xy", min_overlap=0.04)
    ctx.expect_overlap(head, column, axes="z", min_overlap=0.10)
    ctx.expect_overlap(clamp, column, axes="z", min_overlap=0.05)
    ctx.expect_gap(
        head,
        spindle,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=head_nose,
        negative_elem=spindle_quill,
    )
    ctx.expect_overlap(spindle, table, axes="xy", min_overlap=0.01)
    ctx.expect_gap(
        spindle,
        table,
        axis="z",
        min_gap=0.11,
        max_gap=0.14,
        positive_elem=drill_bit,
        negative_elem=table_top,
    )

    with ctx.pose({spindle_spin: math.pi / 2.0}):
        ctx.expect_gap(
            spindle,
            table,
            axis="z",
            min_gap=0.11,
            max_gap=0.14,
            positive_elem=drill_bit,
            negative_elem=table_top,
        )
        ctx.expect_overlap(spindle, table, axes="xy", min_overlap=0.01)
        ctx.fail_if_parts_overlap_in_current_pose(name="spindle_spin_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="spindle_spin_pose_no_floating")

    with ctx.pose({table_tilt: math.radians(15.0)}):
        ctx.expect_contact(table, clamp, contact_tol=1e-6)
        ctx.expect_gap(
            spindle,
            table,
            axis="z",
            min_gap=0.04,
            positive_elem=drill_bit,
            negative_elem=table_top,
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="table_tilt_up_no_overlap")
        ctx.fail_if_isolated_parts(name="table_tilt_up_no_floating")

    with ctx.pose({table_tilt: -math.radians(15.0)}):
        ctx.expect_contact(table, clamp, contact_tol=1e-6)
        ctx.expect_gap(
            spindle,
            table,
            axis="z",
            min_gap=0.10,
            positive_elem=drill_bit,
            negative_elem=table_top,
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="table_tilt_down_no_overlap")
        ctx.fail_if_isolated_parts(name="table_tilt_down_no_floating")

    if spindle_limits is not None and spindle_limits.lower is not None and spindle_limits.upper is not None:
        with ctx.pose({spindle_spin: spindle_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="spindle_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="spindle_lower_no_floating")
        with ctx.pose({spindle_spin: spindle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="spindle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="spindle_upper_no_floating")

    if table_limits is not None and table_limits.lower is not None and table_limits.upper is not None:
        with ctx.pose({table_tilt: table_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="table_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="table_lower_no_floating")
        with ctx.pose({table_tilt: table_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="table_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="table_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
