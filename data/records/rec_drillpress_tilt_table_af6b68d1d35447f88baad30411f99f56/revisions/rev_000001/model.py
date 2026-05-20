from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(x_pos: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y_pos, z_pos) for z_pos, y_pos in rounded_rect_profile(height, width, radius)]


def _build_hollow_sleeve(outer_radius: float, inner_radius: float, length: float):
    half_length = length * 0.5
    outer_profile = [(outer_radius, -half_length), (outer_radius, half_length)]
    inner_profile = [(inner_radius, -half_length), (inner_radius, half_length)]
    return LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=56)


def _build_head_shell():
    return section_loft(
        [
            _yz_section(0.030, 0.160, 0.170, 0.030),
            _yz_section(0.175, 0.182, 0.222, 0.038),
            _yz_section(0.305, 0.136, 0.172, 0.030),
        ]
    )


def _build_spindle_body():
    return LatheGeometry(
        [
            (0.015, 0.000),
            (0.015, -0.090),
            (0.022, -0.104),
            (0.028, -0.134),
            (0.031, -0.166),
            (0.024, -0.186),
            (0.013, -0.202),
            (0.006, -0.212),
        ],
        segments=56,
    )


def _build_laser_cover():
    cover = BoxGeometry((0.038, 0.120, 0.004)).translate(0.019, 0.000, -0.002)
    cover.merge(BoxGeometry((0.004, 0.120, 0.050)).translate(0.036, 0.000, -0.025))
    cover.merge(BoxGeometry((0.038, 0.004, 0.050)).translate(0.019, 0.058, -0.025))
    cover.merge(BoxGeometry((0.038, 0.004, 0.050)).translate(0.019, -0.058, -0.025))
    cover.merge(CylinderGeometry(radius=0.004, height=0.116).rotate_x(math.pi / 2.0))
    return cover


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laser_guide_bench_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.30, 0.31, 0.33, 1.0))
    machine_blue = model.material("machine_blue", rgba=(0.15, 0.24, 0.38, 1.0))
    dark_housing = model.material("dark_housing", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.42, 0.44, 0.47, 1.0))
    smoke_red = model.material("smoke_red", rgba=(0.88, 0.18, 0.18, 0.38))

    base = model.part("base")
    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.430, 0.280, 0.045), 0.028),
        "drill_press_base_plate",
    )
    base.visual(base_plate_mesh, origin=Origin(xyz=(0.000, 0.000, 0.014)), material=cast_iron, name="base_plate")
    base.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(-0.110, 0.000, 0.037)),
        material=cast_iron,
        name="column_pedestal",
    )
    base.visual(
        Box((0.115, 0.090, 0.022)),
        origin=Origin(xyz=(-0.055, 0.000, 0.039)),
        material=cast_iron,
        name="pedestal_web",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.430, 0.280, 0.060)),
        mass=13.0,
        origin=Origin(xyz=(0.000, 0.000, 0.030)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.030, length=0.720),
        origin=Origin(xyz=(0.000, 0.000, 0.372)),
        material=steel,
        name="steel_column",
    )
    column.visual(
        Cylinder(radius=0.044, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=dark_steel,
        name="base_flange",
    )
    column.visual(
        Box((0.008, 0.020, 0.460)),
        origin=Origin(xyz=(0.034, 0.000, 0.330)),
        material=dark_steel,
        name="rack_strip",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.732),
        mass=4.4,
        origin=Origin(xyz=(0.000, 0.000, 0.366)),
    )

    head = model.part("head")
    head.visual(
        Box((0.050, 0.120, 0.170)),
        origin=Origin(xyz=(0.055, 0.000, 0.000)),
        material=machine_blue,
        name="mount_sleeve",
    )
    head.visual(
        Box((0.145, 0.155, 0.118)),
        origin=Origin(xyz=(0.125, 0.000, 0.018)),
        material=machine_blue,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.058, length=0.132),
        origin=Origin(xyz=(0.118, 0.000, 0.062), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=dark_housing,
        name="motor_cap",
    )
    head.visual(
        Box((0.095, 0.118, 0.068)),
        origin=Origin(xyz=(0.215, 0.000, -0.010)),
        material=dark_housing,
        name="front_nose",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.044),
        origin=Origin(xyz=(0.190, 0.000, -0.063)),
        material=dark_housing,
        name="quill_sleeve",
    )
    head.visual(
        Box((0.082, 0.110, 0.024)),
        origin=Origin(xyz=(0.236, 0.000, -0.008)),
        material=dark_housing,
        name="laser_module",
    )
    head.visual(
        Box((0.012, 0.116, 0.016)),
        origin=Origin(xyz=(0.269, 0.000, 0.008)),
        material=dark_housing,
        name="cover_stop",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.360, 0.210, 0.240)),
        mass=9.2,
        origin=Origin(xyz=(0.150, 0.000, 0.000)),
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        Box((0.050, 0.110, 0.070)),
        origin=Origin(xyz=(-0.055, 0.000, 0.000)),
        material=cast_iron,
        name="column_collar",
    )
    table_carriage.visual(
        Box((0.070, 0.084, 0.042)),
        origin=Origin(xyz=(0.080, 0.000, 0.006)),
        material=cast_iron,
        name="carriage_body",
    )
    table_carriage.visual(
        Box((0.110, 0.022, 0.042)),
        origin=Origin(xyz=(-0.005, 0.045, 0.006)),
        material=cast_iron,
        name="upper_clamp_bridge",
    )
    table_carriage.visual(
        Box((0.110, 0.022, 0.042)),
        origin=Origin(xyz=(-0.005, -0.045, 0.006)),
        material=cast_iron,
        name="lower_clamp_bridge",
    )
    table_carriage.visual(
        Box((0.140, 0.064, 0.018)),
        origin=Origin(xyz=(0.125, 0.000, -0.014)),
        material=cast_iron,
        name="support_arm",
    )
    table_carriage.visual(
        Box((0.030, 0.040, 0.008)),
        origin=Origin(xyz=(0.195, 0.000, -0.006)),
        material=cast_iron,
        name="tilt_saddle",
    )
    table_carriage.visual(
        Box((0.026, 0.010, 0.030)),
        origin=Origin(xyz=(0.195, 0.025, 0.005)),
        material=cast_iron,
        name="left_yoke",
    )
    table_carriage.visual(
        Box((0.026, 0.010, 0.030)),
        origin=Origin(xyz=(0.195, -0.025, 0.005)),
        material=cast_iron,
        name="right_yoke",
    )
    table_carriage.visual(
        Box((0.034, 0.030, 0.026)),
        origin=Origin(xyz=(0.068, 0.000, -0.027)),
        material=cast_iron,
        name="pinion_housing",
    )
    table_carriage.visual(
        Box((0.032, 0.028, 0.022)),
        origin=Origin(xyz=(0.082, 0.000, -0.012)),
        material=cast_iron,
        name="pinion_neck",
    )
    table_carriage.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(0.082, 0.000, -0.027), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=dark_steel,
        name="crank_shaft",
    )
    table_carriage.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.082, 0.056, -0.027)),
        material=dark_steel,
        name="crank_knob",
    )
    table_carriage.inertial = Inertial.from_geometry(
        Box((0.230, 0.120, 0.100)),
        mass=3.4,
        origin=Origin(xyz=(0.120, 0.000, 0.000)),
    )

    work_table = model.part("work_table")
    work_table.visual(
        Cylinder(radius=0.125, length=0.016),
        origin=Origin(xyz=(0.115, 0.000, 0.032)),
        material=cast_iron,
        name="table_surface",
    )
    work_table.visual(
        Cylinder(radius=0.132, length=0.004),
        origin=Origin(xyz=(0.115, 0.000, 0.040)),
        material=cast_iron,
        name="table_rim",
    )
    work_table.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=cast_iron,
        name="hinge_barrel",
    )
    work_table.visual(
        Box((0.016, 0.034, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
        material=cast_iron,
        name="pivot_pad",
    )
    work_table.visual(
        Box((0.050, 0.024, 0.020)),
        origin=Origin(xyz=(0.030, 0.000, 0.010)),
        material=cast_iron,
        name="hinge_neck",
    )
    work_table.visual(
        Box((0.072, 0.052, 0.024)),
        origin=Origin(xyz=(0.070, 0.000, 0.014)),
        material=cast_iron,
        name="table_stem",
    )
    work_table.visual(
        Box((0.090, 0.016, 0.014)),
        origin=Origin(xyz=(0.095, 0.030, 0.018)),
        material=cast_iron,
        name="left_rib",
    )
    work_table.visual(
        Box((0.090, 0.016, 0.014)),
        origin=Origin(xyz=(0.095, -0.030, 0.018)),
        material=cast_iron,
        name="right_rib",
    )
    work_table.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.040),
        mass=4.8,
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_geometry(_build_spindle_body(), "spindle_body"),
        origin=Origin(xyz=(0.000, 0.000, -0.010)),
        material=steel,
        name="spindle_body",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.031, length=0.212),
        mass=1.1,
        origin=Origin(xyz=(0.000, 0.000, -0.106)),
    )

    laser_cover = model.part("laser_cover")
    laser_cover.visual(
        mesh_from_geometry(_build_laser_cover(), "laser_cover"),
        material=smoke_red,
        name="laser_cover",
    )
    laser_cover.inertial = Inertial.from_geometry(
        Box((0.040, 0.120, 0.055)),
        mass=0.08,
        origin=Origin(xyz=(0.020, 0.000, -0.025)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(-0.110, 0.000, 0.046)),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.000, 0.000, 0.650)),
    )
    model.articulation(
        "column_to_table_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=table_carriage,
        origin=Origin(xyz=(0.000, 0.000, 0.170)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=120.0, velocity=0.180, lower=0.000, upper=0.110),
    )
    model.articulation(
        "carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=work_table,
        origin=Origin(xyz=(0.195, 0.000, 0.008)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.800,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )
    model.articulation(
        "head_to_spindle",
        ArticulationType.REVOLUTE,
        parent=head,
        child=spindle,
        origin=Origin(xyz=(0.190, 0.000, -0.075)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=40.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "head_to_laser_cover",
        ArticulationType.REVOLUTE,
        parent=head,
        child=laser_cover,
        origin=Origin(xyz=(0.277, 0.000, 0.012)),
        axis=(0.000, -1.000, 0.000),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.000,
            upper=1.050,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    head = object_model.get_part("head")
    table_carriage = object_model.get_part("table_carriage")
    work_table = object_model.get_part("work_table")
    spindle = object_model.get_part("spindle")
    laser_cover = object_model.get_part("laser_cover")

    table_lift = object_model.get_articulation("column_to_table_carriage")
    table_tilt = object_model.get_articulation("carriage_to_table")
    spindle_joint = object_model.get_articulation("head_to_spindle")
    cover_joint = object_model.get_articulation("head_to_laser_cover")

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

    ctx.expect_contact(base, column, elem_a="column_pedestal", elem_b="base_flange", name="column seats on pedestal")
    ctx.expect_contact(head, column, elem_a="mount_sleeve", elem_b="steel_column", name="head clamps to column")
    ctx.expect_contact(
        table_carriage,
        column,
        elem_a="column_collar",
        elem_b="steel_column",
        name="table carriage bears on column",
    )
    ctx.expect_contact(
        work_table,
        table_carriage,
        elem_a="pivot_pad",
        elem_b="tilt_saddle",
        name="table bracket supports the pivot pad",
    )
    ctx.expect_contact(head, spindle, elem_a="quill_sleeve", elem_b="spindle_body", name="spindle runs in quill sleeve")
    ctx.expect_contact(
        laser_cover,
        head,
        elem_a="laser_cover",
        elem_b="cover_stop",
        name="laser cover remains hinged to the head",
    )

    ctx.expect_gap(spindle, work_table, axis="z", min_gap=0.120, max_gap=0.160, name="table starts below chuck")

    ctx.check(
        "table carriage uses a vertical prismatic lift",
        table_lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(table_lift.axis) == (0.0, 0.0, 1.0)
        and table_lift.motion_limits is not None
        and table_lift.motion_limits.upper is not None
        and table_lift.motion_limits.upper >= 0.10,
        details=f"type={table_lift.articulation_type}, axis={table_lift.axis}, limits={table_lift.motion_limits}",
    )
    ctx.check(
        "work table tilts on a front-back hinge",
        table_tilt.articulation_type == ArticulationType.REVOLUTE and tuple(table_tilt.axis) == (1.0, 0.0, 0.0),
        details=f"type={table_tilt.articulation_type}, axis={table_tilt.axis}",
    )
    ctx.check(
        "spindle rotates about the drill axis",
        spindle_joint.articulation_type == ArticulationType.REVOLUTE and tuple(spindle_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={spindle_joint.articulation_type}, axis={spindle_joint.axis}",
    )
    ctx.check(
        "laser cover hinges upward from the head front",
        cover_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(cover_joint.axis) == (0.0, -1.0, 0.0)
        and cover_joint.motion_limits is not None
        and cover_joint.motion_limits.upper is not None
        and cover_joint.motion_limits.upper > 0.8,
        details=f"type={cover_joint.articulation_type}, axis={cover_joint.axis}, limits={cover_joint.motion_limits}",
    )

    rest_carriage_pos = ctx.part_world_position(table_carriage)
    with ctx.pose({table_lift: table_lift.motion_limits.upper}):
        raised_carriage_pos = ctx.part_world_position(table_carriage)
        ctx.expect_contact(
            table_carriage,
            column,
            elem_a="column_collar",
            elem_b="steel_column",
            name="table carriage stays guided on column at full lift",
        )
        ctx.expect_gap(spindle, work_table, axis="z", min_gap=0.020, max_gap=0.040, name="raised table approaches chuck")
    ctx.check(
        "table lift raises the carriage upward",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.09,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    level_aabb = ctx.part_world_aabb(work_table)
    with ctx.pose({table_tilt: math.radians(30.0)}):
        tilted_aabb = ctx.part_world_aabb(work_table)
    level_z = None if level_aabb is None else level_aabb[1][2] - level_aabb[0][2]
    tilted_z = None if tilted_aabb is None else tilted_aabb[1][2] - tilted_aabb[0][2]
    ctx.check(
        "table tilt increases table height span",
        level_z is not None and tilted_z is not None and tilted_z > level_z + 0.08,
        details=f"level_z={level_z}, tilted_z={tilted_z}",
    )

    closed_cover_aabb = ctx.part_world_aabb(laser_cover)
    with ctx.pose({cover_joint: 0.95}):
        open_cover_aabb = ctx.part_world_aabb(laser_cover)
    closed_top = None if closed_cover_aabb is None else closed_cover_aabb[1][2]
    open_top = None if open_cover_aabb is None else open_cover_aabb[1][2]
    ctx.check(
        "laser cover lifts when opened",
        closed_top is not None and open_top is not None and open_top > closed_top + 0.02,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
