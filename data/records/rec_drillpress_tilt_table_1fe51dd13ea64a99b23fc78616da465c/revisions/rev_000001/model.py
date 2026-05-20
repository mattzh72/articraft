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
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


COLUMN_RADIUS = 0.06
COLUMN_HEIGHT = 1.25
COLUMN_SEGMENTS = 48
COLUMN_WORLD_Y = -0.06
COLUMN_BASE_Z = 0.225
HEAD_CLAMP_HEIGHT = 0.16


def _sleeve_mesh(inner_radius: float, outer_radius: float, height: float):
    half_height = height * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half_height), (outer_radius, half_height)],
        [(inner_radius, -half_height), (inner_radius, half_height)],
        segments=COLUMN_SEGMENTS,
        start_cap="flat",
        end_cap="flat",
    )


def _table_top_mesh():
    outer_profile = rounded_rect_profile(0.46, 0.34, 0.028, corner_segments=10)
    slot_profile = rounded_rect_profile(0.08, 0.19, 0.014, corner_segments=8)
    return ExtrudeWithHolesGeometry(
        outer_profile,
        [slot_profile],
        0.034,
        cap=True,
        center=True,
        closed=True,
    )


def _aabb_center(aabb):
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def _axis_matches(axis, target, tol: float = 1e-6) -> bool:
    return all(abs(float(a) - float(b)) <= tol for a, b in zip(axis, target))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.26, 0.30, 0.31, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.65, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.17, 0.19, 0.21, 1.0))
    machine_green = model.material("machine_green", rgba=(0.25, 0.36, 0.29, 1.0))
    phenolic_black = model.material("phenolic_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.66, 0.48, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.24, 0.18, 0.11)),
        origin=Origin(xyz=(0.0, COLUMN_WORLD_Y, 0.11)),
        material=cast_iron,
        name="column_plinth",
    )
    base.visual(
        Box((0.16, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, COLUMN_WORLD_Y - 0.045, 0.115)),
        material=cast_iron,
        name="rear_web",
    )
    base.visual(
        Cylinder(radius=0.09, length=0.06),
        origin=Origin(xyz=(0.0, COLUMN_WORLD_Y, 0.195)),
        material=machined_steel,
        name="column_boss",
    )

    column = model.part("column")
    column.visual(
        mesh_from_geometry(
            CylinderGeometry(
                radius=COLUMN_RADIUS,
                height=COLUMN_HEIGHT,
                radial_segments=COLUMN_SEGMENTS,
            ).translate(0.0, 0.0, COLUMN_HEIGHT * 0.5),
            "column_shaft",
        ),
        material=machined_steel,
        name="column_shaft",
    )
    column.visual(
        Box((0.018, 0.014, 0.82)),
        origin=Origin(xyz=(0.0, COLUMN_RADIUS + 0.001, 0.59)),
        material=dark_steel,
        name="table_rack",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, COLUMN_WORLD_Y, COLUMN_BASE_Z)),
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        Box((0.028, 0.18, 0.14)),
        origin=Origin(xyz=(-0.074, 0.0, 0.0)),
        material=machined_steel,
        name="left_clamp_jaw",
    )
    table_carriage.visual(
        Box((0.028, 0.18, 0.14)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        material=machined_steel,
        name="right_clamp_jaw",
    )
    table_carriage.visual(
        Box((0.176, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.135, -0.08)),
        material=cast_iron,
        name="support_arm",
    )
    table_carriage.visual(
        Box((0.028, 0.12, 0.14)),
        origin=Origin(xyz=(-0.074, 0.23, 0.0)),
        material=cast_iron,
        name="left_yoke",
    )
    table_carriage.visual(
        Box((0.028, 0.12, 0.14)),
        origin=Origin(xyz=(0.074, 0.23, 0.0)),
        material=cast_iron,
        name="right_yoke",
    )
    table_carriage.visual(
        Box((0.176, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, 0.19, -0.075)),
        material=cast_iron,
        name="yoke_bridge",
    )
    table_carriage.visual(
        Cylinder(radius=0.03, length=0.10),
        origin=Origin(xyz=(0.0, 0.105, -0.06), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="pinion_housing",
    )

    model.articulation(
        "column_to_table_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=table_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=-0.18,
            upper=0.22,
        ),
    )

    table = model.part("table")
    table.visual(
        mesh_from_geometry(_table_top_mesh(), "drill_press_table_top"),
        origin=Origin(xyz=(0.0, 0.06, 0.09)),
        material=cast_iron,
        name="table_top",
    )
    table.visual(
        Cylinder(radius=0.048, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="trunnion",
    )
    table.visual(
        Box((0.14, 0.08, 0.09)),
        origin=Origin(xyz=(0.0, 0.088, 0.040)),
        material=cast_iron,
        name="table_web",
    )
    table.visual(
        Box((0.06, 0.18, 0.045)),
        origin=Origin(xyz=(0.0, 0.12, 0.050)),
        material=cast_iron,
        name="center_rib",
    )
    table.visual(
        Box((0.045, 0.16, 0.038)),
        origin=Origin(xyz=(-0.13, 0.10, 0.056)),
        material=cast_iron,
        name="left_rib",
    )
    table.visual(
        Box((0.045, 0.16, 0.038)),
        origin=Origin(xyz=(0.13, 0.10, 0.056)),
        material=cast_iron,
        name="right_rib",
    )

    model.articulation(
        "carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.0, 0.29, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=0.9,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )

    head = model.part("head")
    head.visual(
        Box((0.028, 0.20, HEAD_CLAMP_HEIGHT)),
        origin=Origin(xyz=(-0.074, 0.0, 0.0)),
        material=machined_steel,
        name="left_clamp_jaw",
    )
    head.visual(
        Box((0.028, 0.20, HEAD_CLAMP_HEIGHT)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        material=machined_steel,
        name="right_clamp_jaw",
    )
    head.visual(
        Box((0.176, 0.07, HEAD_CLAMP_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.115, 0.0)),
        material=machined_steel,
        name="clamp_bridge",
    )
    head.visual(
        Box((0.18, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, 0.13, 0.04)),
        material=machine_green,
        name="head_neck",
    )
    head.visual(
        Box((0.46, 0.26, 0.28)),
        origin=Origin(xyz=(0.0, 0.21, 0.16)),
        material=machine_green,
        name="head_body",
    )
    head.visual(
        Box((0.40, 0.22, 0.12)),
        origin=Origin(xyz=(0.0, 0.23, 0.34)),
        material=machine_green,
        name="belt_cover",
    )
    head.visual(
        Box((0.05, 0.20, 0.12)),
        origin=Origin(xyz=(-0.11, -0.01, 0.24)),
        material=machine_green,
        name="left_motor_bracket",
    )
    head.visual(
        Box((0.05, 0.20, 0.12)),
        origin=Origin(xyz=(0.11, -0.01, 0.24)),
        material=machine_green,
        name="right_motor_bracket",
    )
    head.visual(
        Cylinder(radius=0.085, length=0.22),
        origin=Origin(xyz=(0.0, -0.18, 0.26), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="motor",
    )
    head.visual(
        Cylinder(radius=0.07, length=0.20),
        origin=Origin(xyz=(0.0, 0.27, 0.13), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machine_green,
        name="quill_housing",
    )
    head.visual(
        Cylinder(radius=0.036, length=0.16),
        origin=Origin(xyz=(0.0, 0.31, 0.04)),
        material=machined_steel,
        name="spindle",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.11),
        origin=Origin(xyz=(0.0, 0.31, -0.06)),
        material=dark_steel,
        name="chuck",
    )

    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_HEIGHT - (HEAD_CLAMP_HEIGHT * 0.5))),
    )

    quill_feed_lever = model.part("quill_feed_lever")
    quill_feed_lever.visual(
        Cylinder(radius=0.024, length=0.04),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="hub",
    )
    quill_feed_lever.visual(
        Cylinder(radius=0.009, length=0.08),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="hub_stub",
    )

    lever_angle = math.radians(-45.0)
    lever_length = 0.26
    lever_center_y = -0.5 * lever_length * math.sin(lever_angle)
    lever_center_z = 0.5 * lever_length * math.cos(lever_angle)
    lever_tip_y = -lever_length * math.sin(lever_angle)
    lever_tip_z = lever_length * math.cos(lever_angle)
    quill_feed_lever.visual(
        Cylinder(radius=0.008, length=lever_length),
        origin=Origin(
            xyz=(0.08, lever_center_y, lever_center_z),
            rpy=(lever_angle, 0.0, 0.0),
        ),
        material=machined_steel,
        name="lever_arm",
    )
    quill_feed_lever.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(0.08, lever_tip_y, lever_tip_z)),
        material=phenolic_black,
        name="lever_knob",
    )

    model.articulation(
        "head_to_quill_feed_lever",
        ArticulationType.REVOLUTE,
        parent=head,
        child=quill_feed_lever,
        origin=Origin(xyz=(0.23, 0.18, 0.16)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.95,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    table_carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    head = object_model.get_part("head")
    quill_feed_lever = object_model.get_part("quill_feed_lever")

    table_lift = object_model.get_articulation("column_to_table_carriage")
    table_tilt = object_model.get_articulation("carriage_to_table")
    lever_joint = object_model.get_articulation("head_to_quill_feed_lever")

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

    ctx.expect_contact(column, base, elem_a="column_shaft", elem_b="column_boss")
    ctx.expect_contact(head, column, elem_a="left_clamp_jaw", elem_b="column_shaft")
    ctx.expect_contact(head, column, elem_a="right_clamp_jaw", elem_b="column_shaft")
    ctx.expect_contact(
        table_carriage,
        column,
        elem_a="left_clamp_jaw",
        elem_b="column_shaft",
    )
    ctx.expect_contact(
        table_carriage,
        column,
        elem_a="right_clamp_jaw",
        elem_b="column_shaft",
    )
    ctx.expect_contact(table, table_carriage, elem_a="trunnion", elem_b="left_yoke")
    ctx.expect_contact(table, table_carriage, elem_a="trunnion", elem_b="right_yoke")
    ctx.expect_contact(
        quill_feed_lever,
        head,
        elem_a="hub",
        elem_b="head_body",
    )

    ctx.expect_gap(table, column, axis="y", min_gap=0.10, name="table_projects_forward")

    ctx.check(
        "table_lift_axis_is_vertical",
        _axis_matches(table_lift.axis, (0.0, 0.0, 1.0)),
        f"Expected vertical prismatic axis, got {table_lift.axis}",
    )
    ctx.check(
        "table_tilt_axis_is_horizontal",
        _axis_matches(table_tilt.axis, (1.0, 0.0, 0.0)),
        f"Expected left-right tilt axis, got {table_tilt.axis}",
    )
    ctx.check(
        "quill_feed_axis_is_horizontal",
        _axis_matches(lever_joint.axis, (1.0, 0.0, 0.0)),
        f"Expected quill feed lever axis along +x, got {lever_joint.axis}",
    )

    with ctx.pose({table_lift: table_lift.motion_limits.lower}):
        low_table_pos = ctx.part_world_position(table)
        ctx.expect_gap(table, base, axis="z", min_gap=0.28, name="table_clears_base_low")

    with ctx.pose({table_lift: table_lift.motion_limits.upper}):
        high_table_pos = ctx.part_world_position(table)
        ctx.expect_gap(head, table, axis="z", min_gap=0.12, name="table_clears_head_high")

    if low_table_pos is not None and high_table_pos is not None:
        ctx.check(
            "table_height_adjusts_along_column",
            (high_table_pos[2] - low_table_pos[2]) >= 0.38,
            f"Expected at least 0.38 m of lift travel, got {high_table_pos[2] - low_table_pos[2]:.3f} m",
        )

    rest_top_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    with ctx.pose({table_tilt: math.radians(30.0)}):
        tilted_top_aabb = ctx.part_element_world_aabb(table, elem="table_top")
        ctx.expect_contact(table, table_carriage, elem_a="trunnion", elem_b="left_yoke")
        ctx.expect_contact(table, table_carriage, elem_a="trunnion", elem_b="right_yoke")
    if rest_top_aabb is not None and tilted_top_aabb is not None:
        ctx.check(
            "table_tilt_changes_surface_attitude",
            tilted_top_aabb[1][2] >= rest_top_aabb[1][2] + 0.07,
            (
                "Expected the tilted table top AABB to rise noticeably; "
                f"got rest max z={rest_top_aabb[1][2]:.3f}, tilt max z={tilted_top_aabb[1][2]:.3f}"
            ),
        )

    with ctx.pose({lever_joint: lever_joint.motion_limits.lower}):
        low_knob_aabb = ctx.part_element_world_aabb(quill_feed_lever, elem="lever_knob")
        ctx.expect_contact(
            quill_feed_lever,
            head,
            elem_a="hub",
            elem_b="head_body",
            name="lever_hub_stays_seated_low",
        )
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        high_knob_aabb = ctx.part_element_world_aabb(quill_feed_lever, elem="lever_knob")
        ctx.expect_contact(
            quill_feed_lever,
            head,
            elem_a="hub",
            elem_b="head_body",
            name="lever_hub_stays_seated_high",
        )
    if low_knob_aabb is not None and high_knob_aabb is not None:
        low_knob_center = _aabb_center(low_knob_aabb)
        high_knob_center = _aabb_center(high_knob_aabb)
        ctx.check(
            "quill_feed_lever_rotates",
            abs(high_knob_center[2] - low_knob_center[2]) >= 0.12,
            (
                "Expected the feed lever knob to swing through a visible arc; "
                f"got dz={high_knob_center[2] - low_knob_center[2]:.3f} m"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
