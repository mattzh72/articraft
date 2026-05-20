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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * cos((2.0 * pi * index) / segments),
            radius * sin((2.0 * pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annular_sector_profile(
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 40,
) -> list[tuple[float, float]]:
    if end_angle < start_angle:
        end_angle += 2.0 * pi
    outer = []
    inner = []
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        outer.append((outer_radius * cos(angle), outer_radius * sin(angle)))
        inner.append((inner_radius * cos(angle), inner_radius * sin(angle)))
    return outer + list(reversed(inner))


def _toothed_disc_profile(
    outer_radius: float,
    root_radius: float,
    teeth: int,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    total = teeth * 2
    for index in range(total):
        angle = (2.0 * pi * index) / total
        radius = outer_radius if index % 2 == 0 else root_radius
        points.append((radius * cos(angle), radius * sin(angle)))
    return points


def _knob_profile(
    base_radius: float,
    lobe_amplitude: float,
    lobes: int,
    *,
    segments: int = 96,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments):
        angle = (2.0 * pi * index) / segments
        radius = base_radius + lobe_amplitude * (0.5 + 0.5 * cos(lobes * angle))
        points.append((radius * cos(angle), radius * sin(angle)))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_miter_saw")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.73, 0.75, 0.77, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    motor_blue = model.material("motor_blue", rgba=(0.16, 0.33, 0.57, 1.0))
    handle_black = model.material("handle_black", rgba=(0.09, 0.09, 0.10, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    guard_silver = model.material("guard_silver", rgba=(0.67, 0.69, 0.72, 1.0))
    accent_black = model.material("accent_black", rgba=(0.12, 0.13, 0.14, 1.0))
    knob_red = model.material("knob_red", rgba=(0.73, 0.13, 0.11, 1.0))

    base = model.part("base")
    base_shell = ExtrudeGeometry(
        rounded_rect_profile(0.58, 0.38, 0.04),
        0.055,
        center=True,
    )
    base.visual(
        _mesh("miter_saw_base_shell", base_shell),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cast_aluminum,
        name="base_shell",
    )
    base.visual(
        Box((0.22, 0.08, 0.030)),
        origin=Origin(xyz=(0.0, 0.175, 0.040)),
        material=cast_aluminum,
        name="front_scale_lip",
    )
    base.visual(
        Cylinder(radius=0.106, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=satin_steel,
        name="miter_pedestal",
    )
    base.visual(
        Box((0.145, 0.055, 0.060)),
        origin=Origin(xyz=(0.0, -0.176, 0.085)),
        material=dark_steel,
        name="rear_pivot_tower",
    )
    base.visual(
        Box((0.018, 0.050, 0.125)),
        origin=Origin(xyz=(-0.064, -0.165, 0.1175)),
        material=dark_steel,
        name="left_pivot_cheek",
    )
    base.visual(
        Box((0.018, 0.050, 0.125)),
        origin=Origin(xyz=(0.064, -0.165, 0.1175)),
        material=dark_steel,
        name="right_pivot_cheek",
    )
    base.visual(
        Box((0.150, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.198, 0.100)),
        material=dark_steel,
        name="rear_pivot_crossbar",
    )
    base.visual(
        Box((0.230, 0.018, 0.060)),
        origin=Origin(xyz=(-0.170, -0.150, 0.115)),
        material=satin_steel,
        name="left_fence",
    )
    base.visual(
        Box((0.230, 0.018, 0.060)),
        origin=Origin(xyz=(0.170, -0.150, 0.115)),
        material=satin_steel,
        name="right_fence",
    )
    base.visual(
        Box((0.060, 0.022, 0.040)),
        origin=Origin(xyz=(-0.170, -0.156, 0.055)),
        material=cast_aluminum,
        name="left_fence_support",
    )
    base.visual(
        Box((0.060, 0.022, 0.040)),
        origin=Origin(xyz=(0.170, -0.156, 0.055)),
        material=cast_aluminum,
        name="right_fence_support",
    )
    base.visual(
        Box((0.028, 0.035, 0.063)),
        origin=Origin(xyz=(0.086, -0.138, 0.0865)),
        material=dark_steel,
        name="depth_stop_bracket",
    )
    base.visual(
        Box((0.032, 0.016, 0.022)),
        origin=Origin(xyz=(0.098, -0.155, 0.098)),
        material=dark_steel,
        name="depth_stop_body",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.043),
        origin=Origin(xyz=(0.121, -0.118, 0.115), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="depth_stop_shaft",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.005),
        origin=Origin(xyz=(0.102, -0.118, 0.115), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="depth_stop_thrust_washer",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.1405, -0.118, 0.115), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="depth_stop_retainer",
    )

    miter_table = model.part("miter_table")
    miter_table.visual(
        Cylinder(radius=0.131, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=satin_steel,
        name="table_disc",
    )
    miter_table.visual(
        Box((0.090, 0.045, 0.012)),
        origin=Origin(xyz=(0.0, 0.118, 0.006)),
        material=satin_steel,
        name="table_handle",
    )
    miter_table.visual(
        Box((0.096, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.000, 0.014)),
        material=accent_black,
        name="kerf_insert",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="pivot_tube",
    )
    left_arm_rail = tube_from_spline_points(
        [
            (-0.020, 0.006, 0.010),
            (-0.023, 0.036, 0.072),
            (-0.022, 0.064, 0.130),
            (-0.020, 0.090, 0.190),
        ],
        radius=0.009,
        samples_per_segment=16,
        radial_segments=14,
    )
    right_arm_rail = tube_from_spline_points(
        [
            (0.020, 0.006, 0.010),
            (0.023, 0.036, 0.072),
            (0.022, 0.064, 0.130),
            (0.020, 0.090, 0.190),
        ],
        radius=0.009,
        samples_per_segment=16,
        radial_segments=14,
    )
    arm_beam = left_arm_rail.merge(right_arm_rail)
    arm.visual(_mesh("miter_saw_arm_beam", arm_beam), material=dark_steel, name="arm_beam")
    arm.visual(
        Box((0.010, 0.040, 0.082)),
        origin=Origin(xyz=(-0.023, 0.100, 0.220)),
        material=dark_steel,
        name="left_yoke_cheek",
    )
    arm.visual(
        Box((0.010, 0.040, 0.082)),
        origin=Origin(xyz=(0.023, 0.100, 0.220)),
        material=dark_steel,
        name="right_yoke_cheek",
    )
    arm.visual(
        Box((0.056, 0.024, 0.032)),
        origin=Origin(xyz=(0.0, 0.086, 0.188)),
        material=dark_steel,
        name="yoke_bridge",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.018, length=0.038),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bevel_pivot_barrel",
    )
    head.visual(
        Box((0.020, 0.036, 0.082)),
        origin=Origin(xyz=(0.018, 0.026, -0.028)),
        material=motor_blue,
        name="head_support",
    )
    head.visual(
        Box((0.024, 0.062, 0.110)),
        origin=Origin(xyz=(0.032, 0.073, -0.042)),
        material=motor_blue,
        name="motor_mount",
    )
    upper_guard = ExtrudeGeometry(
        _annular_sector_profile(
            0.152,
            0.128,
            0.78 * pi,
            1.92 * pi,
            segments=48,
        ),
        0.044,
        center=True,
    ).rotate_y(pi / 2.0).translate(0.0, 0.175, -0.115)
    head.visual(
        _mesh("miter_saw_upper_guard", upper_guard),
        material=guard_silver,
        name="upper_guard",
    )
    head.visual(
        Cylinder(radius=0.042, length=0.116),
        origin=Origin(xyz=(0.095, 0.145, -0.035), rpy=(0.0, pi / 2.0, 0.0)),
        material=motor_blue,
        name="motor_housing",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.160, 0.145, -0.035), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="motor_end_cap",
    )
    head.visual(
        Cylinder(radius=0.025, length=0.008),
        origin=Origin(xyz=(-0.0055, 0.175, -0.115), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="arbor_flange",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.024, 0.175, -0.115), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="guard_hinge_boss",
    )
    head.visual(
        Box((0.052, 0.050, 0.042)),
        origin=Origin(xyz=(0.022, 0.125, -0.094)),
        material=dark_steel,
        name="gearbox_housing",
    )
    head.visual(
        Cylinder(radius=0.009, length=0.062),
        origin=Origin(xyz=(0.021, 0.150, -0.115), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="arbor_shaft",
    )
    handle_geom = tube_from_spline_points(
        [
            (0.022, 0.022, 0.006),
            (0.034, 0.050, 0.056),
            (0.062, 0.084, 0.106),
            (0.090, 0.062, 0.122),
            (0.112, 0.026, 0.094),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=16,
    )
    head.visual(
        _mesh("miter_saw_top_handle", handle_geom),
        material=handle_black,
        name="top_handle",
    )
    head.visual(
        Box((0.026, 0.038, 0.060)),
        origin=Origin(xyz=(0.024, 0.020, 0.030)),
        material=handle_black,
        name="handle_mount",
    )
    head.visual(
        Cylinder(radius=0.011, length=0.060),
        origin=Origin(xyz=(0.096, 0.026, 0.094), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_black,
        name="front_grip",
    )

    blade = model.part("blade")
    blade_geom = ExtrudeWithHolesGeometry(
        _toothed_disc_profile(0.115, 0.110, 42),
        [_circle_profile(0.013, segments=28)],
        0.003,
        center=True,
    ).rotate_y(pi / 2.0)
    blade.visual(
        _mesh("miter_saw_blade_disc", blade_geom),
        material=blade_steel,
        name="blade_disc",
    )

    lower_guard = model.part("lower_guard")
    lower_guard_sector = ExtrudeGeometry(
        _annular_sector_profile(
            0.127,
            0.118,
            0.02 * pi,
            1.10 * pi,
            segments=44,
        ),
        0.018,
        center=True,
    ).rotate_y(pi / 2.0).translate(0.014, 0.0, 0.0)
    lower_guard.visual(
        _mesh("miter_saw_lower_guard_shell", lower_guard_sector),
        material=guard_silver,
        name="lower_guard_shell",
    )
    lower_guard.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.014, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=guard_silver,
        name="guard_hub",
    )
    lower_guard_link = tube_from_spline_points(
        [
            (0.014, 0.015, -0.010),
            (0.014, 0.045, -0.045),
            (0.014, 0.082, -0.085),
        ],
        radius=0.006,
        samples_per_segment=16,
        radial_segments=14,
    )
    lower_guard.visual(
        _mesh("miter_saw_lower_guard_link", lower_guard_link),
        material=guard_silver,
        name="guard_link",
    )

    stop_knob = model.part("trench_stop_knob")
    knob_geom = ExtrudeWithHolesGeometry(
        _knob_profile(0.013, 0.004, 5),
        [_circle_profile(0.0073, segments=24)],
        0.033,
        center=True,
    ).rotate_y(pi / 2.0)
    stop_knob.visual(
        _mesh("miter_saw_trench_stop_knob", knob_geom),
        material=knob_red,
        name="trench_stop_knob",
    )

    model.articulation(
        "miter_table_rotate",
        ArticulationType.REVOLUTE,
        parent=base,
        child=miter_table,
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.82, upper=0.82),
    )
    model.articulation(
        "arm_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, -0.165, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=-1.10, upper=0.25),
    )
    model.articulation(
        "head_bevel",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.0, 0.118, 0.225)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.82, upper=0.10),
    )
    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=blade,
        origin=Origin(xyz=(0.0, 0.175, -0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=60.0),
    )
    model.articulation(
        "lower_guard_hinge",
        ArticulationType.REVOLUTE,
        parent=head,
        child=lower_guard,
        origin=Origin(xyz=(0.0, 0.175, -0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.30),
    )
    model.articulation(
        "trench_stop_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stop_knob,
        origin=Origin(xyz=(0.121, -0.118, 0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    miter_table = object_model.get_part("miter_table")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")
    blade = object_model.get_part("blade")
    lower_guard = object_model.get_part("lower_guard")
    stop_knob = object_model.get_part("trench_stop_knob")

    miter_joint = object_model.get_articulation("miter_table_rotate")
    arm_joint = object_model.get_articulation("arm_pivot")
    bevel_joint = object_model.get_articulation("head_bevel")
    blade_joint = object_model.get_articulation("blade_spin")
    guard_joint = object_model.get_articulation("lower_guard_hinge")
    knob_joint = object_model.get_articulation("trench_stop_knob_spin")

    pedestal = base.get_visual("miter_pedestal")
    shaft = base.get_visual("depth_stop_shaft")
    thrust_washer = base.get_visual("depth_stop_thrust_washer")
    retainer = base.get_visual("depth_stop_retainer")
    table_disc = miter_table.get_visual("table_disc")
    blade_disc = blade.get_visual("blade_disc")
    arbor_flange = head.get_visual("arbor_flange")
    guard_hinge_boss = head.get_visual("guard_hinge_boss")

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

    def axis_tuple(joint_obj) -> tuple[float, float, float]:
        return tuple(round(float(value), 6) for value in joint_obj.axis)

    ctx.check(
        "mechanism_axes_match_real_saw",
        (
            axis_tuple(miter_joint) == (0.0, 0.0, 1.0)
            and axis_tuple(arm_joint) == (1.0, 0.0, 0.0)
            and axis_tuple(bevel_joint) == (0.0, 1.0, 0.0)
            and axis_tuple(blade_joint) == (1.0, 0.0, 0.0)
            and axis_tuple(guard_joint) == (1.0, 0.0, 0.0)
            and axis_tuple(knob_joint) == (1.0, 0.0, 0.0)
        ),
        details="Miter should rotate about Z; arm, blade, guard, and knob about X; bevel about Y.",
    )

    ctx.expect_gap(
        miter_table,
        base,
        axis="z",
        positive_elem=table_disc,
        negative_elem=pedestal,
        max_gap=0.001,
        max_penetration=0.0,
        name="miter_table_seated_on_pedestal",
    )
    ctx.expect_overlap(
        miter_table,
        base,
        axes="xy",
        elem_a=table_disc,
        elem_b=pedestal,
        min_overlap=0.18,
        name="miter_table_centered_over_pedestal",
    )
    ctx.expect_contact(arm, base, name="arm_pivot_supported_by_base")
    ctx.expect_contact(head, arm, name="bevel_head_supported_by_arm")
    ctx.expect_contact(
        blade,
        head,
        elem_a=blade_disc,
        elem_b=arbor_flange,
        name="blade_seated_on_arbor_flange",
    )
    ctx.expect_contact(
        lower_guard,
        head,
        elem_b=guard_hinge_boss,
        name="lower_guard_hinged_to_head",
    )
    ctx.expect_contact(
        stop_knob,
        base,
        elem_b=thrust_washer,
        name="trench_stop_knob_seated_on_thrust_washer",
    )
    ctx.expect_overlap(
        stop_knob,
        base,
        axes="yz",
        elem_b=shaft,
        min_overlap=0.012,
        name="trench_stop_knob_centered_on_depth_stop_shaft",
    )
    ctx.expect_gap(
        base,
        stop_knob,
        axis="x",
        positive_elem=retainer,
        max_gap=0.003,
        max_penetration=0.0,
        name="trench_stop_retainer_keeps_knob_clipped",
    )
    ctx.expect_gap(
        blade,
        miter_table,
        axis="z",
        positive_elem=blade_disc,
        negative_elem=table_disc,
        min_gap=0.045,
        name="blade_clears_table_in_raised_pose",
    )

    with ctx.pose({miter_joint: 0.55}):
        ctx.expect_gap(
            miter_table,
            base,
            axis="z",
            positive_elem=table_disc,
            negative_elem=pedestal,
            max_gap=0.001,
            max_penetration=0.0,
            name="miter_table_stays_seated_when_rotated",
        )

    blade_rest_aabb = ctx.part_world_aabb(blade)
    assert blade_rest_aabb is not None
    with ctx.pose({arm_joint: -0.82}):
        blade_lowered_aabb = ctx.part_world_aabb(blade)
        assert blade_lowered_aabb is not None
        assert blade_lowered_aabb[0][2] < blade_rest_aabb[0][2] - 0.10
        ctx.expect_contact(arm, base, name="arm_pivot_remains_supported_when_lowered")

    blade_rest_pos = ctx.part_world_position(blade)
    assert blade_rest_pos is not None
    with ctx.pose({bevel_joint: -0.70}):
        blade_beveled_pos = ctx.part_world_position(blade)
        assert blade_beveled_pos is not None
        assert abs(blade_beveled_pos[0] - blade_rest_pos[0]) > 0.06
        ctx.expect_contact(head, arm, name="bevel_head_stays_on_yoke_when_tilted")

    guard_rest_aabb = ctx.part_world_aabb(lower_guard)
    assert guard_rest_aabb is not None
    with ctx.pose({guard_joint: 1.05}):
        guard_open_aabb = ctx.part_world_aabb(lower_guard)
        assert guard_open_aabb is not None
        assert guard_open_aabb[1][2] > guard_rest_aabb[1][2] + 0.02
        ctx.expect_contact(
            lower_guard,
            head,
            elem_b=guard_hinge_boss,
            name="lower_guard_remains_hinged_when_open",
        )

    with ctx.pose({knob_joint: 1.40}):
        ctx.expect_contact(
            stop_knob,
            base,
            elem_b=thrust_washer,
            name="trench_stop_knob_keeps_seat_while_rotating",
        )
        ctx.expect_gap(
            base,
            stop_knob,
            axis="x",
            positive_elem=retainer,
            max_gap=0.003,
            max_penetration=0.0,
            name="trench_stop_knob_does_not_lift_past_retainer",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
