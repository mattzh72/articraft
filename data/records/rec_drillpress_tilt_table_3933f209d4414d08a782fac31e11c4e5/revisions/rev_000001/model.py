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
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry.copy())
    return merged


def _ring_shell(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 56,
) -> MeshGeometry:
    half = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _build_base_mesh() -> MeshGeometry:
    foot = ExtrudeGeometry(
        rounded_rect_profile(0.34, 0.24, 0.032, corner_segments=8),
        0.028,
        cap=True,
        center=True,
    ).translate(0.06, 0.0, 0.014)
    rear_pad = BoxGeometry((0.13, 0.16, 0.030)).translate(-0.015, 0.0, 0.043)
    bridge = BoxGeometry((0.085, 0.100, 0.028)).translate(0.015, 0.0, 0.042)
    column_boss = CylinderGeometry(radius=0.050, height=0.028, radial_segments=44).translate(
        0.0,
        0.0,
        0.042,
    )
    return _merge_geometries(foot, rear_pad, bridge, column_boss)


def _build_head_mesh() -> MeshGeometry:
    front_pad = BoxGeometry((0.030, 0.018, 0.120)).translate(0.050, 0.0, 0.030)
    left_clamp = BoxGeometry((0.030, 0.036, 0.120)).translate(0.052, 0.050, 0.030)
    right_clamp = BoxGeometry((0.030, 0.036, 0.120)).translate(0.052, -0.050, 0.030)
    body = BoxGeometry((0.130, 0.180, 0.110)).translate(0.135, 0.0, 0.060)
    belt_cover = BoxGeometry((0.160, 0.210, 0.090)).translate(0.120, 0.0, 0.105)
    motor = CylinderGeometry(radius=0.042, height=0.165, radial_segments=40).rotate_x(
        math.pi / 2.0
    ).translate(0.095, 0.0, 0.110)
    nose = _ring_shell(0.046, 0.028, 0.072).translate(
        0.110,
        0.0,
        -0.040,
    )
    feed_boss = CylinderGeometry(radius=0.032, height=0.056, radial_segments=32).rotate_x(
        math.pi / 2.0
    ).translate(0.075, 0.0915, -0.005)
    return _merge_geometries(
        front_pad,
        left_clamp,
        right_clamp,
        body,
        belt_cover,
        motor,
        nose,
        feed_boss,
    )


def _build_table_support_mesh() -> MeshGeometry:
    front_pad = BoxGeometry((0.028, 0.018, 0.082)).translate(0.048, 0.0, 0.000)
    left_clamp = BoxGeometry((0.030, 0.034, 0.082)).translate(0.050, 0.047, 0.000)
    right_clamp = BoxGeometry((0.030, 0.034, 0.082)).translate(0.050, -0.047, 0.000)
    bridge = BoxGeometry((0.045, 0.112, 0.026)).translate(0.058, 0.0, 0.028)
    knuckle = BoxGeometry((0.050, 0.050, 0.056)).translate(0.092, 0.0, -0.008)
    arm = BoxGeometry((0.136, 0.040, 0.008)).translate(0.150, 0.0, -0.065)
    pivot_pad = BoxGeometry((0.032, 0.040, 0.020)).translate(0.218, 0.0, -0.070)
    return _merge_geometries(
        front_pad,
        left_clamp,
        right_clamp,
        bridge,
        knuckle,
        arm,
        pivot_pad,
    )


def _build_table_mesh() -> MeshGeometry:
    pivot_block = BoxGeometry((0.026, 0.040, 0.020)).translate(0.0, 0.0, -0.030)
    stem = BoxGeometry((0.034, 0.032, 0.056)).translate(-0.030, 0.0, 0.008)
    support_block = BoxGeometry((0.050, 0.040, 0.012)).translate(-0.040, 0.0, 0.036)
    top = CylinderGeometry(radius=0.105, height=0.014, radial_segments=64).translate(
        -0.040,
        0.0,
        0.049,
    )
    return _merge_geometries(pivot_block, stem, support_block, top)


def _build_quill_mesh() -> MeshGeometry:
    collar = CylinderGeometry(radius=0.040, height=0.014, radial_segments=40).translate(
        0.0,
        0.0,
        -0.007,
    )
    quill_body = CylinderGeometry(radius=0.024, height=0.092, radial_segments=40).translate(
        0.0,
        0.0,
        -0.060,
    )
    chuck_body = CylinderGeometry(radius=0.018, height=0.040, radial_segments=32).translate(
        0.0,
        0.0,
        -0.125,
    )
    spindle = CylinderGeometry(radius=0.006, height=0.028, radial_segments=24).translate(
        0.0,
        0.0,
        -0.159,
    )
    tip = ConeGeometry(radius=0.010, height=0.018, radial_segments=24).translate(
        0.0,
        0.0,
        -0.182,
    )
    return _merge_geometries(collar, quill_body, chuck_body, spindle, tip)


def _build_feed_hub_mesh() -> MeshGeometry:
    hub = CylinderGeometry(radius=0.028, height=0.030, radial_segments=32).rotate_x(
        math.pi / 2.0
    ).translate(0.0, 0.015, 0.0)
    spokes: list[MeshGeometry] = [hub]
    arm_length = 0.110
    knob_distance = 0.135
    for angle in (math.pi / 2.0, -math.pi / 6.0, 7.0 * math.pi / 6.0):
        dx = math.cos(angle)
        dz = math.sin(angle)
        rod = CylinderGeometry(radius=0.007, height=arm_length, radial_segments=18).rotate_y(
            math.atan2(dx, dz)
        ).translate(dx * 0.079, 0.015, dz * 0.079)
        knob = SphereGeometry(radius=0.015, width_segments=18, height_segments=12).translate(
            dx * knob_distance,
            0.015,
            dz * knob_distance,
        )
        spokes.extend([rod, knob])
    return _merge_geometries(*spokes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_pillar_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.30, 0.31, 0.32, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.43, 0.45, 0.47, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_handle = model.material("dark_handle", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(_mesh("drill_press_base", _build_base_mesh()), material=cast_iron, name="base_casting")
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.24, 0.056)),
        mass=18.0,
        origin=Origin(xyz=(0.06, 0.0, 0.028)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.035, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=steel,
        name="column_tube",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.720),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
    )

    head = model.part("head")
    head.visual(
        Box((0.030, 0.018, 0.120)),
        origin=Origin(xyz=(0.050, 0.0, 0.060)),
        material=machine_gray,
        name="head_front_pad",
    )
    head.visual(
        Box((0.030, 0.036, 0.120)),
        origin=Origin(xyz=(0.052, 0.050, 0.060)),
        material=machine_gray,
        name="head_left_clamp",
    )
    head.visual(
        Box((0.030, 0.036, 0.120)),
        origin=Origin(xyz=(0.052, -0.050, 0.060)),
        material=machine_gray,
        name="head_right_clamp",
    )
    head.visual(
        Box((0.130, 0.180, 0.110)),
        origin=Origin(xyz=(0.135, 0.0, 0.055)),
        material=machine_gray,
        name="head_main_body",
    )
    head.visual(
        Box((0.160, 0.210, 0.090)),
        origin=Origin(xyz=(0.120, 0.0, 0.130)),
        material=machine_gray,
        name="belt_cover",
    )
    head.visual(
        Cylinder(radius=0.042, length=0.165),
        origin=Origin(xyz=(0.095, 0.0, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machine_gray,
        name="motor_barrel",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.060),
        origin=Origin(xyz=(0.075, 0.090, 0.040), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machine_gray,
        name="feed_boss",
    )
    head.visual(
        Box((0.080, 0.090, 0.040)),
        origin=Origin(xyz=(0.110, 0.0, 0.020)),
        material=machine_gray,
        name="quill_nose_block",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.245, 0.210, 0.300)),
        mass=14.0,
        origin=Origin(xyz=(0.075, 0.0, 0.040)),
    )

    table_support = model.part("table_support")
    table_support.visual(
        _mesh("drill_press_table_support", _build_table_support_mesh()),
        material=machine_gray,
        name="support_bracket",
    )
    table_support.inertial = Inertial.from_geometry(
        Box((0.160, 0.112, 0.090)),
        mass=4.0,
        origin=Origin(xyz=(0.080, 0.0, -0.005)),
    )

    table = model.part("table")
    table.visual(
        _mesh("drill_press_table", _build_table_mesh()),
        material=cast_iron,
        name="round_table",
    )
    table.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.070),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    quill = model.part("quill")
    quill.visual(
        _mesh("drill_press_quill", _build_quill_mesh()),
        material=steel,
        name="quill_spindle",
    )
    quill.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.190),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
    )

    feed_hub = model.part("feed_hub")
    feed_hub.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_handle,
        name="feed_hub_body",
    )
    for index, angle in enumerate((math.pi / 2.0, -math.pi / 6.0, 7.0 * math.pi / 6.0)):
        dx = math.cos(angle)
        dz = math.sin(angle)
        feed_hub.visual(
            Cylinder(radius=0.007, length=0.110),
            origin=Origin(
                xyz=(dx * 0.079, 0.015, dz * 0.079),
                rpy=(0.0, math.atan2(dx, dz), 0.0),
            ),
            material=steel,
            name=f"feed_arm_{index}",
        )
        feed_hub.visual(
            Box((0.024, 0.024, 0.024)),
            origin=Origin(xyz=(dx * 0.135, 0.015, dz * 0.135)),
            material=dark_handle,
            name=f"feed_knob_{index}",
        )
    feed_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.030),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.544)),
    )
    model.articulation(
        "table_swing",
        ArticulationType.REVOLUTE,
        parent=column,
        child=table_support,
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_support,
        child=table,
        origin=Origin(xyz=(0.247, 0.0, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.0,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )
    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.15,
            lower=0.0,
            upper=0.055,
        ),
    )
    model.articulation(
        "feed_rotation",
        ArticulationType.REVOLUTE,
        parent=head,
        child=feed_hub,
        origin=Origin(xyz=(0.075, 0.120, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.5,
            upper=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    head = object_model.get_part("head")
    table_support = object_model.get_part("table_support")
    table = object_model.get_part("table")
    quill = object_model.get_part("quill")
    feed_hub = object_model.get_part("feed_hub")

    table_swing = object_model.get_articulation("table_swing")
    table_tilt = object_model.get_articulation("table_tilt")
    head_to_quill = object_model.get_articulation("head_to_quill")
    feed_rotation = object_model.get_articulation("feed_rotation")

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
    ctx.allow_overlap(
        table,
        table_support,
        reason="Tilt-table trunnion is represented as a captured hinge within the support bracket.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(column, base, name="column_is_seated_on_base")
    ctx.expect_contact(head, column, name="head_clamps_to_column")
    ctx.expect_contact(table_support, column, name="table_support_clamps_to_column")
    ctx.expect_contact(table, table_support, name="table_trunnion_seats_in_support")
    ctx.expect_contact(feed_hub, head, name="feed_handle_hub_seats_on_head")
    ctx.expect_contact(quill, head, name="quill_runs_in_head_bore")

    ctx.expect_overlap(table, quill, axes="xy", min_overlap=0.040, name="table_sits_under_spindle_axis")
    ctx.expect_gap(quill, table, axis="z", min_gap=0.095, max_gap=0.115, name="resting_quill_clearance")
    ctx.expect_gap(head, table, axis="z", min_gap=0.240, name="head_clear_of_work_table")
    ctx.expect_within(quill, head, axes="xy", margin=0.0, name="quill_centered_within_head")

    ctx.check(
        "table_swing_axis_is_vertical",
        tuple(float(v) for v in table_swing.axis) == (0.0, 0.0, 1.0),
        f"unexpected table swing axis: {table_swing.axis}",
    )
    ctx.check(
        "table_tilt_axis_is_transverse",
        tuple(float(v) for v in table_tilt.axis) == (0.0, 1.0, 0.0),
        f"unexpected table tilt axis: {table_tilt.axis}",
    )
    ctx.check(
        "quill_axis_is_downward",
        tuple(float(v) for v in head_to_quill.axis) == (0.0, 0.0, -1.0),
        f"unexpected quill axis: {head_to_quill.axis}",
    )
    ctx.check(
        "feed_hub_axis_is_side_to_side",
        tuple(float(v) for v in feed_rotation.axis) == (0.0, 1.0, 0.0),
        f"unexpected feed axis: {feed_rotation.axis}",
    )

    table_rest = ctx.part_world_position(table)
    quill_rest = ctx.part_world_position(quill)
    table_rest_aabb = ctx.part_world_aabb(table)
    assert table_rest is not None
    assert quill_rest is not None
    assert table_rest_aabb is not None

    with ctx.pose({table_swing: math.pi / 2.0}):
        swung_table = ctx.part_world_position(table)
        assert swung_table is not None
        ctx.check(
            "table_swings_around_column",
            abs(swung_table[1]) > 0.08 and abs(swung_table[0]) < 0.03,
            f"swung table position was {swung_table}",
        )
        ctx.expect_contact(table_support, column, name="swung_support_stays_on_column")

    with ctx.pose({table_tilt: math.radians(30.0)}):
        tilted_aabb = ctx.part_world_aabb(table)
        assert tilted_aabb is not None
        ctx.check(
            "table_tilt_changes_surface_pose",
            tilted_aabb[1][2] > table_rest_aabb[1][2] + 0.015,
            f"tilted table aabb was {tilted_aabb}",
        )

    with ctx.pose({head_to_quill: 0.050, feed_rotation: 1.0}):
        quill_advanced = ctx.part_world_position(quill)
        assert quill_advanced is not None
        ctx.check(
            "quill_advances_downward",
            quill_advanced[2] < quill_rest[2] - 0.045,
            f"rest={quill_rest}, advanced={quill_advanced}",
        )
        ctx.expect_gap(
            quill,
            table,
            axis="z",
            min_gap=0.045,
            max_gap=0.065,
            name="advanced_quill_stays_above_table",
        )
        ctx.expect_contact(feed_hub, head, name="feed_hub_remains_seated_when_turned")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
