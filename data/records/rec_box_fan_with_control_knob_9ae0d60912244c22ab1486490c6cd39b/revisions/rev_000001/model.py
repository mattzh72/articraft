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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def _merge_geometry(base, addition):
    if base is None:
        return addition
    base.merge(addition)
    return base


def _circle_points(radius: float, *, samples: int, y: float = 0.0) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / samples),
            y,
            radius * math.sin(2.0 * math.pi * index / samples),
        )
        for index in range(samples)
    ]


def _build_grille_mesh():
    grille = None
    for width, height, radius, wire_radius in (
        (0.374, 0.274, 0.050, 0.0028),
        (0.324, 0.238, 0.044, 0.0025),
        (0.274, 0.202, 0.038, 0.0023),
        (0.224, 0.166, 0.032, 0.0022),
    ):
        loop = wire_from_points(
            [(x, 0.0, z) for x, z in rounded_rect_profile(width, height, radius, corner_segments=8)],
            radius=wire_radius,
            closed_path=True,
            corner_mode="miter",
        )
        grille = _merge_geometry(grille, loop)

    hub_ring = wire_from_points(
        _circle_points(0.036, samples=28),
        radius=0.0026,
        closed_path=True,
        corner_mode="miter",
    )
    grille = _merge_geometry(grille, hub_ring)

    for spoke_angle in range(8):
        angle = spoke_angle * math.pi / 4.0
        spoke = wire_from_points(
            [
                (0.038 * math.cos(angle), 0.0, 0.038 * math.sin(angle)),
                (0.160 * math.cos(angle), 0.0, 0.120 * math.sin(angle)),
            ],
            radius=0.0023,
            cap_ends=True,
        )
        grille = _merge_geometry(grille, spoke)

    return grille


def _build_blade_mesh():
    blade_profile = [
        (0.034, -0.014),
        (0.050, -0.026),
        (0.082, -0.028),
        (0.106, -0.020),
        (0.120, -0.006),
        (0.122, 0.000),
        (0.116, 0.010),
        (0.094, 0.020),
        (0.060, 0.023),
        (0.038, 0.012),
    ]
    blade = ExtrudeGeometry(blade_profile, 0.008, center=True)
    blade.rotate_x(-math.pi / 2.0)
    blade.rotate((1.0, 0.0, 0.0), math.radians(18.0), origin=(0.034, 0.0, 0.0))
    return blade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="window_box_fan")

    housing_white = model.material("housing_white", rgba=(0.93, 0.93, 0.90, 1.0))
    light_gray = model.material("light_gray", rgba=(0.72, 0.75, 0.78, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.65, 0.69, 1.0))

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(Box((0.50, 0.13, 0.34)), mass=4.8)
    housing.visual(
        Box((0.50, 0.13, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=housing_white,
        name="top_shell",
    )
    housing.visual(
        Box((0.50, 0.13, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.158)),
        material=housing_white,
        name="bottom_shell",
    )
    housing.visual(
        Box((0.024, 0.13, 0.292)),
        origin=Origin(xyz=(-0.238, 0.0, 0.0)),
        material=housing_white,
        name="left_shell",
    )
    housing.visual(
        Box((0.024, 0.13, 0.292)),
        origin=Origin(xyz=(0.238, 0.0, 0.0)),
        material=housing_white,
        name="right_shell",
    )
    housing.visual(
        Box((0.420, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.056, 0.144)),
        material=housing_white,
        name="front_bezel_top",
    )
    housing.visual(
        Box((0.420, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.056, -0.144)),
        material=housing_white,
        name="front_bezel_bottom",
    )
    housing.visual(
        Box((0.032, 0.018, 0.268)),
        origin=Origin(xyz=(-0.184, 0.056, 0.0)),
        material=housing_white,
        name="front_bezel_left",
    )
    housing.visual(
        Box((0.032, 0.018, 0.268)),
        origin=Origin(xyz=(0.184, 0.056, 0.0)),
        material=housing_white,
        name="front_bezel_right",
    )
    housing.visual(
        Box((0.420, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -0.056, 0.146)),
        material=housing_white,
        name="rear_rim_top",
    )
    housing.visual(
        Box((0.420, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -0.056, -0.146)),
        material=housing_white,
        name="rear_rim_bottom",
    )
    housing.visual(
        Box((0.028, 0.018, 0.264)),
        origin=Origin(xyz=(-0.186, -0.056, 0.0)),
        material=housing_white,
        name="rear_rim_left",
    )
    housing.visual(
        Box((0.028, 0.018, 0.264)),
        origin=Origin(xyz=(0.186, -0.056, 0.0)),
        material=housing_white,
        name="rear_rim_right",
    )
    housing.visual(
        Box((0.080, 0.014, 0.052)),
        origin=Origin(xyz=(0.168, 0.058, 0.104)),
        material=housing_white,
        name="control_panel",
    )
    housing.visual(
        mesh_from_geometry(_build_grille_mesh(), "front_grille"),
        origin=Origin(xyz=(0.0, 0.054, 0.0)),
        material=housing_white,
        name="front_grille",
    )
    motor_mount = model.part("motor_mount")
    motor_mount.inertial = Inertial.from_geometry(
        Box((0.24, 0.08, 0.24)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
    )
    motor_mount.visual(
        Cylinder(radius=0.046, length=0.048),
        origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="motor_can",
    )
    motor_mount.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_cap",
    )
    motor_mount.visual(
        Box((0.150, 0.012, 0.014)),
        origin=Origin(xyz=(-0.100, -0.016, 0.0)),
        material=dark_gray,
        name="left_support",
    )
    motor_mount.visual(
        Box((0.150, 0.012, 0.014)),
        origin=Origin(xyz=(0.100, -0.016, 0.0)),
        material=dark_gray,
        name="right_support",
    )
    motor_mount.visual(
        Box((0.014, 0.012, 0.120)),
        origin=Origin(xyz=(0.0, -0.016, 0.090)),
        material=dark_gray,
        name="top_support",
    )
    motor_mount.visual(
        Box((0.014, 0.012, 0.120)),
        origin=Origin(xyz=(0.0, -0.016, -0.090)),
        material=dark_gray,
        name="bottom_support",
    )

    propeller = model.part("propeller")
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.036),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    blade_mesh = mesh_from_geometry(_build_blade_mesh(), "propeller_blade")
    for blade_index in range(5):
        propeller.visual(
            blade_mesh,
            origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(0.0, blade_index * 2.0 * math.pi / 5.0, 0.0)),
            material=light_gray,
            name=f"blade_{blade_index}",
        )
    propeller.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_gray,
        name="hub_shell",
    )
    propeller.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_core",
    )
    propeller.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=housing_white,
        name="nose_cap",
    )

    adapter_track = model.part("adapter_track")
    adapter_track.inertial = Inertial.from_geometry(
        Box((0.18, 0.10, 0.295)),
        mass=0.5,
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
    )
    adapter_track.visual(
        Box((0.18, 0.10, 0.025)),
        origin=Origin(xyz=(0.09, 0.0, 0.135)),
        material=housing_white,
        name="track_top",
    )
    adapter_track.visual(
        Box((0.18, 0.10, 0.025)),
        origin=Origin(xyz=(0.09, 0.0, -0.135)),
        material=housing_white,
        name="track_bottom",
    )
    adapter_track.visual(
        Box((0.18, 0.015, 0.295)),
        origin=Origin(xyz=(0.09, 0.0525, 0.0)),
        material=housing_white,
        name="track_front_lip",
    )
    adapter_track.visual(
        Box((0.18, 0.015, 0.295)),
        origin=Origin(xyz=(0.09, -0.0525, 0.0)),
        material=housing_white,
        name="track_rear_lip",
    )

    window_adapter = model.part("window_adapter")
    window_adapter.inertial = Inertial.from_geometry(
        Box((0.29, 0.09, 0.31)),
        mass=0.6,
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
    )
    window_adapter.visual(
        Box((0.18, 0.090, 0.245)),
        origin=Origin(xyz=(-0.090, 0.0, 0.0)),
        material=housing_white,
        name="slide_tongue",
    )
    window_adapter.visual(
        Box((0.012, 0.025, 0.300)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=housing_white,
        name="panel_inner_rail",
    )
    window_adapter.visual(
        Box((0.016, 0.025, 0.300)),
        origin=Origin(xyz=(0.102, 0.0, 0.0)),
        material=housing_white,
        name="panel_outer_rail",
    )
    window_adapter.visual(
        Box((0.110, 0.025, 0.016)),
        origin=Origin(xyz=(0.055, 0.0, 0.142)),
        material=housing_white,
        name="panel_top_rail",
    )
    window_adapter.visual(
        Box((0.110, 0.025, 0.016)),
        origin=Origin(xyz=(0.055, 0.0, -0.142)),
        material=housing_white,
        name="panel_bottom_rail",
    )
    window_adapter.visual(
        Box((0.094, 0.022, 0.012)),
        origin=Origin(xyz=(0.059, 0.0, 0.050)),
        material=housing_white,
        name="panel_crossbar_upper",
    )
    window_adapter.visual(
        Box((0.094, 0.022, 0.012)),
        origin=Origin(xyz=(0.059, 0.0, -0.050)),
        material=housing_white,
        name="panel_crossbar_lower",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.024),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    selector_knob.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="knob_body",
    )
    selector_knob.visual(
        Box((0.004, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.024, 0.010)),
        material=steel,
        name="indicator",
    )

    model.articulation(
        "housing_to_motor_mount",
        ArticulationType.FIXED,
        parent=housing,
        child=motor_mount,
        origin=Origin(),
    )
    model.articulation(
        "motor_mount_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=motor_mount,
        child=propeller,
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "housing_to_adapter_track",
        ArticulationType.FIXED,
        parent=housing,
        child=adapter_track,
        origin=Origin(xyz=(0.250, 0.0, 0.0)),
    )
    model.articulation(
        "adapter_track_to_window_adapter",
        ArticulationType.PRISMATIC,
        parent=adapter_track,
        child=window_adapter,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.15, lower=0.0, upper=0.10),
    )
    model.articulation(
        "housing_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=selector_knob,
        origin=Origin(xyz=(0.168, 0.065, 0.104)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=0.0, upper=1.5 * math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    motor_mount = object_model.get_part("motor_mount")
    propeller = object_model.get_part("propeller")
    adapter_track = object_model.get_part("adapter_track")
    window_adapter = object_model.get_part("window_adapter")
    selector_knob = object_model.get_part("selector_knob")

    fan_spin = object_model.get_articulation("motor_mount_to_propeller")
    adapter_slide = object_model.get_articulation("adapter_track_to_window_adapter")
    selector_turn = object_model.get_articulation("housing_to_selector_knob")

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

    ctx.expect_contact(motor_mount, housing, name="motor_mount_seated_in_housing")
    ctx.expect_contact(propeller, motor_mount, name="propeller_seated_on_motor_cap")
    ctx.expect_contact(adapter_track, housing, name="adapter_track_attached")
    ctx.expect_contact(window_adapter, adapter_track, name="window_adapter_guided_in_track")
    ctx.expect_contact(selector_knob, housing, name="selector_knob_flush_to_panel")

    ctx.expect_within(propeller, housing, axes="xz", margin=0.01, name="propeller_within_housing_opening")
    ctx.expect_gap(
        housing,
        propeller,
        axis="y",
        positive_elem="front_grille",
        min_gap=0.002,
        max_gap=0.020,
        name="propeller_clears_front_grille",
    )
    ctx.expect_overlap(
        window_adapter,
        adapter_track,
        axes="yz",
        min_overlap=0.08,
        name="window_adapter_overlaps_track_guides",
    )
    ctx.expect_within(window_adapter, housing, axes="z", margin=0.02, name="adapter_height_matches_housing")

    ctx.check(
        "fan_spin_axis_is_y",
        tuple(round(value, 3) for value in fan_spin.axis) == (0.0, 1.0, 0.0),
        f"unexpected fan axis {fan_spin.axis}",
    )
    ctx.check(
        "adapter_slide_axis_is_x",
        tuple(round(value, 3) for value in adapter_slide.axis) == (1.0, 0.0, 0.0),
        f"unexpected adapter axis {adapter_slide.axis}",
    )
    ctx.check(
        "selector_axis_is_y",
        tuple(round(value, 3) for value in selector_turn.axis) == (0.0, 1.0, 0.0),
        f"unexpected selector axis {selector_turn.axis}",
    )

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

    adapter_limits = adapter_slide.motion_limits
    if adapter_limits is not None and adapter_limits.lower is not None and adapter_limits.upper is not None:
        adapter_rest_pos = ctx.part_world_position(window_adapter)
        with ctx.pose({adapter_slide: adapter_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="window_adapter_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="window_adapter_lower_no_floating")
            ctx.expect_contact(window_adapter, adapter_track, name="window_adapter_lower_track_contact")
        with ctx.pose({adapter_slide: adapter_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="window_adapter_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="window_adapter_upper_no_floating")
            ctx.expect_contact(window_adapter, adapter_track, name="window_adapter_upper_track_contact")
            ctx.expect_overlap(
                window_adapter,
                adapter_track,
                axes="yz",
                min_overlap=0.08,
                name="window_adapter_upper_track_guidance",
            )
            adapter_extended_pos = ctx.part_world_position(window_adapter)
            ctx.check(
                "window_adapter_extends_outward",
                adapter_rest_pos is not None
                and adapter_extended_pos is not None
                and adapter_extended_pos[0] > adapter_rest_pos[0] + 0.095,
                f"rest={adapter_rest_pos} extended={adapter_extended_pos}",
            )

    selector_limits = selector_turn.motion_limits
    if selector_limits is not None and selector_limits.lower is not None and selector_limits.upper is not None:
        indicator_rest = ctx.part_element_world_aabb(selector_knob, elem="indicator")
        with ctx.pose({selector_turn: selector_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="selector_knob_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="selector_knob_lower_no_floating")
            ctx.expect_contact(selector_knob, housing, name="selector_knob_lower_contact")
        with ctx.pose({selector_turn: selector_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="selector_knob_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="selector_knob_upper_no_floating")
            ctx.expect_contact(selector_knob, housing, name="selector_knob_upper_contact")
            indicator_high = ctx.part_element_world_aabb(selector_knob, elem="indicator")
            if indicator_rest is not None and indicator_high is not None:
                rest_center = tuple((indicator_rest[0][i] + indicator_rest[1][i]) * 0.5 for i in range(3))
                high_center = tuple((indicator_high[0][i] + indicator_high[1][i]) * 0.5 for i in range(3))
                ctx.check(
                    "selector_indicator_rotates",
                    abs(high_center[0] - rest_center[0]) >= 0.009 and abs(high_center[2] - rest_center[2]) >= 0.009,
                    f"rest={rest_center} high={high_center}",
                )

    blade_rest = ctx.part_element_world_aabb(propeller, elem="blade_0")
    with ctx.pose({fan_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="fan_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="fan_quarter_turn_no_floating")
        ctx.expect_contact(propeller, motor_mount, name="propeller_quarter_turn_motor_contact")
        ctx.expect_gap(
            housing,
            propeller,
            axis="y",
            positive_elem="front_grille",
            min_gap=0.002,
            max_gap=0.020,
            name="fan_quarter_turn_grille_clearance",
        )
        blade_quarter = ctx.part_element_world_aabb(propeller, elem="blade_0")
        if blade_rest is not None and blade_quarter is not None:
            rest_center = tuple((blade_rest[0][i] + blade_rest[1][i]) * 0.5 for i in range(3))
            quarter_center = tuple((blade_quarter[0][i] + blade_quarter[1][i]) * 0.5 for i in range(3))
            ctx.check(
                "blade_0_rotates_about_axle",
                abs(quarter_center[2] - rest_center[2]) > 0.060,
                f"rest={rest_center} quarter={quarter_center}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
