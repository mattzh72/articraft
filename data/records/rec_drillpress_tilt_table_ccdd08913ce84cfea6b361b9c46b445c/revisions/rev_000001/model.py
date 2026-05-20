from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_base_drill_press")

    base_steel = model.material("base_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    painted_green = model.material("painted_green", rgba=(0.15, 0.36, 0.22, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_handle = model.material("dark_handle", rgba=(0.08, 0.08, 0.09, 1.0))
    table_gray = model.material("table_gray", rgba=(0.40, 0.42, 0.44, 1.0))
    cutter_steel = model.material("cutter_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    top_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.18, 0.06, 0.009),
            0.012,
            center=True,
        ),
        "base_top_plate",
    )
    mount_block_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.09, 0.05, 0.010),
            0.22,
            center=True,
        ),
        "head_mount_block",
    )
    main_housing_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.18, 0.10, 0.018),
            0.20,
            center=True,
        ),
        "head_main_housing",
    )
    gearbox_nose_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.076, 0.05, 0.010),
            0.04,
            center=True,
        ),
        "gearbox_nose",
    )

    base = model.part("magnetic_base")
    base.visual(
        Box((0.28, 0.22, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=base_steel,
        name="base_block",
    )
    base.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.045, 0.064)),
        material=dark_handle,
        name="top_plate",
    )
    base.visual(
        Box((0.032, 0.060, 0.028)),
        origin=Origin(xyz=(0.122, -0.055, 0.058 + 0.014)),
        material=painted_green,
        name="power_switch",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.28, 0.22, 0.058)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.034, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=machined_steel,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.042, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=base_steel,
        name="column_foot",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.46),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
    )

    head = model.part("head")
    head.visual(
        mount_block_mesh,
        origin=Origin(xyz=(0.0, 0.059, 0.0)),
        material=painted_green,
        name="head_mount_block",
    )
    head.visual(
        main_housing_mesh,
        origin=Origin(xyz=(0.0, 0.134, 0.005)),
        material=painted_green,
        name="main_housing",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.120),
        origin=Origin(xyz=(0.0, 0.112, 0.118), rpy=(pi / 2.0, 0.0, 0.0)),
        material=painted_green,
        name="motor_pod",
    )
    head.visual(
        gearbox_nose_mesh,
        origin=Origin(xyz=(0.0, 0.188, 0.095)),
        material=painted_green,
        name="gearbox_nose",
    )
    head.visual(
        Box((0.014, 0.078, 0.180)),
        origin=Origin(xyz=(-0.027, 0.183, -0.020)),
        material=machined_steel,
        name="guide_left",
    )
    head.visual(
        Box((0.014, 0.078, 0.180)),
        origin=Origin(xyz=(0.027, 0.183, -0.020)),
        material=machined_steel,
        name="guide_right",
    )
    head.visual(
        Box((0.068, 0.034, 0.020)),
        origin=Origin(xyz=(0.0, 0.183, 0.105)),
        material=machined_steel,
        name="guide_bridge",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.096, 0.132, 0.020), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="feed_boss",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.26)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.120, 0.010)),
    )

    quill = model.part("quill")
    quill.visual(
        Box((0.040, 0.028, 0.100)),
        origin=Origin(),
        material=machined_steel,
        name="quill_carriage",
    )
    quill.visual(
        Cylinder(radius=0.017, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=machined_steel,
        name="spindle_sleeve",
    )
    quill.inertial = Inertial.from_geometry(
        Box((0.050, 0.040, 0.190)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=machined_steel,
        name="spindle_shank",
    )
    spindle.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=cutter_steel,
        name="chuck_body",
    )
    spindle.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.081)),
        material=cutter_steel,
        name="annular_cutter",
    )
    spindle.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
        material=cutter_steel,
        name="pilot_tip",
    )
    spindle.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(xyz=(0.016, 0.0, -0.036), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_handle,
        name="set_screw",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.130),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
    )

    feed_wheel = model.part("feed_wheel")
    feed_wheel.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_handle,
        name="wheel_hub",
    )
    for index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        feed_wheel.visual(
            Cylinder(radius=0.0045, length=0.082),
            origin=Origin(
                xyz=(0.028, cos(angle) * 0.041, sin(angle) * 0.041),
                rpy=(angle - pi / 2.0, 0.0, 0.0),
            ),
            material=dark_handle,
            name=f"spoke_{index}",
        )
    for name, angle in (
        ("handle_knob_a", 0.0),
        ("handle_knob_b", 2.0 * pi / 3.0),
        ("handle_knob_c", 4.0 * pi / 3.0),
    ):
        feed_wheel.visual(
            Cylinder(radius=0.007, length=0.028),
            origin=Origin(
                xyz=(0.042, cos(angle) * 0.082, sin(angle) * 0.082),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=dark_handle,
            name=name,
        )
    feed_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.060),
        mass=0.8,
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    table_bracket = model.part("table_bracket")
    table_bracket.visual(
        Box((0.080, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=base_steel,
        name="rear_strap",
    )
    table_bracket.visual(
        Box((0.018, 0.068, 0.070)),
        origin=Origin(xyz=(-0.043, 0.0, 0.0)),
        material=base_steel,
        name="left_jaw",
    )
    table_bracket.visual(
        Box((0.018, 0.068, 0.070)),
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
        material=base_steel,
        name="right_jaw",
    )
    table_bracket.visual(
        Box((0.080, 0.050, 0.070)),
        origin=Origin(xyz=(0.0, 0.059, 0.0)),
        material=base_steel,
        name="front_clamp",
    )
    table_bracket.visual(
        Box((0.100, 0.050, 0.022)),
        origin=Origin(xyz=(0.0, 0.109, -0.030)),
        material=base_steel,
        name="support_arm",
    )
    table_bracket.visual(
        Box((0.018, 0.060, 0.060)),
        origin=Origin(xyz=(-0.059, 0.164, -0.030)),
        material=base_steel,
        name="left_ear",
    )
    table_bracket.visual(
        Box((0.018, 0.060, 0.060)),
        origin=Origin(xyz=(0.059, 0.164, -0.030)),
        material=base_steel,
        name="right_ear",
    )
    table_bracket.inertial = Inertial.from_geometry(
        Box((0.140, 0.240, 0.090)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.070, -0.015)),
    )

    table = model.part("tilt_table")
    table_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.160, 0.120, 0.008),
            [rounded_rect_profile(0.030, 0.070, 0.006)],
            height=0.012,
            center=True,
        ),
        "tilt_table_surface",
    )
    table.visual(
        Cylinder(radius=0.018, length=0.100),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=base_steel,
        name="trunnion",
    )
    table.visual(
        Box((0.080, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.055, 0.010)),
        material=base_steel,
        name="table_support",
    )
    table.visual(
        table_mesh,
        origin=Origin(xyz=(0.0, 0.110, 0.025)),
        material=table_gray,
        name="table_surface",
    )
    table.inertial = Inertial.from_geometry(
        Box((0.170, 0.140, 0.060)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.085, 0.020)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, -0.035, 0.058)),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
    )
    model.articulation(
        "head_to_feed_wheel",
        ArticulationType.REVOLUTE,
        parent=head,
        child=feed_wheel,
        origin=Origin(xyz=(0.110, 0.132, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.0, 0.205, 0.023)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.20, lower=0.0, upper=0.055),
    )
    model.articulation(
        "quill_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=quill,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "column_to_table_bracket",
        ArticulationType.FIXED,
        parent=column,
        child=table_bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
    )
    model.articulation(
        "bracket_to_tilt_table",
        ArticulationType.REVOLUTE,
        parent=table_bracket,
        child=table,
        origin=Origin(xyz=(0.0, 0.164, -0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.70,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("magnetic_base")
    column = object_model.get_part("column")
    head = object_model.get_part("head")
    feed_wheel = object_model.get_part("feed_wheel")
    quill = object_model.get_part("quill")
    spindle = object_model.get_part("spindle")
    table_bracket = object_model.get_part("table_bracket")
    table = object_model.get_part("tilt_table")

    feed_joint = object_model.get_articulation("head_to_feed_wheel")
    quill_joint = object_model.get_articulation("head_to_quill")
    spindle_joint = object_model.get_articulation("quill_to_spindle")
    table_joint = object_model.get_articulation("bracket_to_tilt_table")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check("base_exists", base is not None, "Magnetic base part is required.")
    ctx.check("column_exists", column is not None, "Round column part is required.")
    ctx.check("head_exists", head is not None, "Spindle head part is required.")
    ctx.check("feed_wheel_exists", feed_wheel is not None, "Feed wheel part is required.")
    ctx.check("quill_exists", quill is not None, "Quill part is required.")
    ctx.check("spindle_exists", spindle is not None, "Spindle part is required.")
    ctx.check("table_bracket_exists", table_bracket is not None, "Table bracket part is required.")
    ctx.check("tilt_table_exists", table is not None, "Tilt table part is required.")

    ctx.expect_contact(column, base, contact_tol=1e-4)
    ctx.expect_contact(head, column, contact_tol=1e-4)
    ctx.expect_contact(table_bracket, column, contact_tol=1e-4)
    ctx.expect_contact(feed_wheel, head, contact_tol=1e-4)
    ctx.expect_contact(quill, head, contact_tol=1e-4)
    ctx.expect_contact(spindle, quill, contact_tol=1e-4)
    ctx.expect_contact(table, table_bracket, contact_tol=1e-4)

    ctx.expect_overlap(spindle, table, axes="xy", min_overlap=0.015)
    ctx.expect_gap(spindle, table, axis="z", min_gap=0.055, max_gap=0.12)

    ctx.check(
        "feed_joint_axis",
        tuple(feed_joint.axis) == (1.0, 0.0, 0.0),
        f"Expected feed wheel axis (1, 0, 0), got {feed_joint.axis}.",
    )
    ctx.check(
        "quill_joint_axis",
        tuple(quill_joint.axis) == (0.0, 0.0, -1.0),
        f"Expected quill axis (0, 0, -1), got {quill_joint.axis}.",
    )
    ctx.check(
        "spindle_joint_axis",
        tuple(spindle_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected spindle axis (0, 0, 1), got {spindle_joint.axis}.",
    )
    ctx.check(
        "table_joint_axis",
        tuple(table_joint.axis) == (1.0, 0.0, 0.0),
        f"Expected table tilt axis (1, 0, 0), got {table_joint.axis}.",
    )

    def _center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    handle_rest_aabb = ctx.part_element_world_aabb(feed_wheel, elem="handle_knob_a")
    ctx.check(
        "feed_handle_visual_present",
        handle_rest_aabb is not None,
        "Named feed wheel handle visual missing.",
    )
    if handle_rest_aabb is not None:
        handle_rest = _center(handle_rest_aabb)
        with ctx.pose({feed_joint: 1.0}):
            moved_aabb = ctx.part_element_world_aabb(feed_wheel, elem="handle_knob_a")
            ctx.check(
                "feed_wheel_pose_moves_handle",
                moved_aabb is not None and abs(_center(moved_aabb)[2] - handle_rest[2]) > 0.03,
                "Feed wheel knob should move through a clear arc in Z.",
            )
            ctx.expect_contact(feed_wheel, head, contact_tol=1e-4)

    quill_rest = ctx.part_world_position(quill)
    spindle_rest = ctx.part_world_position(spindle)
    ctx.check(
        "quill_world_position_available",
        quill_rest is not None and spindle_rest is not None,
        "Quill and spindle positions should resolve in world space.",
    )
    if quill_rest is not None and spindle_rest is not None:
        quill_limits = quill_joint.motion_limits
        quill_upper = 0.055
        if quill_limits is not None and quill_limits.upper is not None:
            quill_upper = quill_limits.upper
        with ctx.pose({quill_joint: quill_upper}):
            quill_down = ctx.part_world_position(quill)
            spindle_down = ctx.part_world_position(spindle)
            ctx.check(
                "quill_moves_downward",
                quill_down is not None and quill_down[2] < quill_rest[2] - 0.045,
                "Quill should descend substantially under feed motion.",
            )
            ctx.check(
                "spindle_follows_quill",
                spindle_down is not None and spindle_down[2] < spindle_rest[2] - 0.045,
                "Spindle should descend with the quill.",
            )
            ctx.expect_contact(quill, head, contact_tol=1e-4)
            ctx.expect_contact(spindle, quill, contact_tol=1e-4)
            ctx.expect_gap(spindle, table, axis="z", min_gap=0.005, max_gap=0.03)

    set_screw_rest_aabb = ctx.part_element_world_aabb(spindle, elem="set_screw")
    ctx.check(
        "spindle_set_screw_present",
        set_screw_rest_aabb is not None,
        "Spindle set screw visual missing.",
    )
    if set_screw_rest_aabb is not None:
        set_screw_rest = _center(set_screw_rest_aabb)
        with ctx.pose({spindle_joint: 1.2}):
            set_screw_spin_aabb = ctx.part_element_world_aabb(spindle, elem="set_screw")
            ctx.check(
                "spindle_spin_rotates_detail",
                set_screw_spin_aabb is not None
                and abs(_center(set_screw_spin_aabb)[1] - set_screw_rest[1]) > 0.01,
                "Continuous spindle rotation should move the asymmetric set screw.",
            )
            ctx.expect_contact(spindle, quill, contact_tol=1e-4)

    table_rest_aabb = ctx.part_element_world_aabb(table, elem="table_surface")
    ctx.check(
        "table_surface_present",
        table_rest_aabb is not None,
        "Tilt table surface visual missing.",
    )
    if table_rest_aabb is not None:
        table_rest = _center(table_rest_aabb)
        with ctx.pose({table_joint: 0.60}):
            table_tilted_aabb = ctx.part_element_world_aabb(table, elem="table_surface")
            ctx.check(
                "table_tilts",
                table_tilted_aabb is not None and _center(table_tilted_aabb)[2] > table_rest[2] + 0.015,
                "Tilt table should raise its front edge when tilted.",
            )
            ctx.expect_contact(table, table_bracket, contact_tol=1e-4)

    for joint, name in ((feed_joint, "feed_wheel"), (quill_joint, "quill"), (table_joint, "tilt_table")):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{name}_upper_no_floating")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
