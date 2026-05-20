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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
    rounded_rect_profile,
    section_loft,
    wire_from_points,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _circle_points(radius: float, *, segments: int = 24, y: float = 0.0) -> list[tuple[float, float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * index / segments), y, radius * math.sin(2.0 * math.pi * index / segments))
        for index in range(segments)
    ]


def _square_loop_points(half_extent: float) -> list[tuple[float, float, float]]:
    return [
        (half_extent, 0.0, half_extent),
        (-half_extent, 0.0, half_extent),
        (-half_extent, 0.0, -half_extent),
        (half_extent, 0.0, -half_extent),
    ]


def _blade_section(
    radius: float,
    *,
    y_shift: float,
    chord: float,
    sweep: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_t = thickness * 0.5
    return [
        (radius, y_shift - 0.95 * half_t, sweep - 0.56 * chord),
        (radius, y_shift + 1.00 * half_t, sweep - 0.08 * chord),
        (radius, y_shift + 0.70 * half_t, sweep + 0.52 * chord),
        (radius, y_shift - 0.82 * half_t, sweep + 0.12 * chord),
    ]


def _build_base_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.0),
            (0.060, 0.0),
            (0.130, 0.008),
            (0.176, 0.026),
            (0.172, 0.044),
            (0.124, 0.056),
            (0.050, 0.060),
            (0.0, 0.060),
        ],
        segments=72,
    )


def _build_pedestal_sleeve_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.044, 0.0),
            (0.044, 0.290),
            (0.041, 0.320),
            (0.041, 0.336),
        ],
        [
            (0.028, 0.0),
            (0.028, 0.290),
            (0.028, 0.320),
            (0.028, 0.336),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _build_housing_frame_mesh() -> MeshGeometry:
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.500, 0.500, 0.032, corner_segments=10),
        [rounded_rect_profile(0.392, 0.392, 0.020, corner_segments=8)],
        0.150,
        center=True,
    ).rotate_x(-math.pi / 2.0)


def _build_front_grille_mesh() -> MeshGeometry:
    grille = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.392, 0.392, 0.020, corner_segments=8),
        [rounded_rect_profile(0.360, 0.360, 0.014, corner_segments=6)],
        0.010,
        center=True,
    ).rotate_x(-math.pi / 2.0)

    wire_radius = 0.0032
    for half_extent in (0.170, 0.132, 0.094, 0.056):
        grille.merge(
            wire_from_points(
                _square_loop_points(half_extent),
                radius=wire_radius,
                radial_segments=16,
                closed_path=True,
                cap_ends=False,
                corner_mode="fillet",
                corner_radius=0.020,
                corner_segments=10,
            )
        )

    for x_pos in (-0.060, 0.0, 0.060):
        grille.merge(
            wire_from_points(
                [(x_pos, 0.0, -0.172), (x_pos, 0.0, 0.172)],
                radius=wire_radius,
                radial_segments=16,
                cap_ends=True,
                corner_mode="miter",
            )
        )
    for z_pos in (-0.060, 0.0, 0.060):
        grille.merge(
            wire_from_points(
                [(-0.172, 0.0, z_pos), (0.172, 0.0, z_pos)],
                radius=wire_radius,
                radial_segments=16,
                cap_ends=True,
                corner_mode="miter",
            )
        )

    grille.merge(
        wire_from_points(
            _circle_points(0.050, segments=28),
            radius=wire_radius,
            radial_segments=16,
            closed_path=True,
            cap_ends=False,
            corner_mode="miter",
        )
    )
    return grille


def _build_motor_struts_mesh() -> MeshGeometry:
    struts = MeshGeometry()
    strut_radius = 0.0065
    for start, end in [
        ((0.050, -0.060, 0.260), (0.196, 0.0, 0.260)),
        ((-0.050, -0.060, 0.260), (-0.196, 0.0, 0.260)),
        ((0.0, -0.060, 0.322), (0.0, 0.0, 0.456)),
        ((0.0, -0.060, 0.198), (0.0, 0.0, 0.064)),
    ]:
        struts.merge(
            wire_from_points(
                [start, end],
                radius=strut_radius,
                radial_segments=16,
                cap_ends=True,
                corner_mode="miter",
            )
        )
    return struts


def _build_propeller_blades_mesh() -> MeshGeometry:
    base_blade = section_loft(
        [
            _blade_section(0.032, y_shift=0.008, chord=0.074, sweep=-0.008, thickness=0.0095),
            _blade_section(0.092, y_shift=0.013, chord=0.112, sweep=0.000, thickness=0.0085),
            _blade_section(0.148, y_shift=0.020, chord=0.086, sweep=0.018, thickness=0.0065),
            _blade_section(0.176, y_shift=0.025, chord=0.044, sweep=0.032, thickness=0.0040),
        ]
    )
    blades = MeshGeometry()
    for blade_index in range(5):
        blades.merge(base_blade.copy().rotate_y(blade_index * 2.0 * math.pi / 5.0))
    return blades


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_box_fan")

    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    light_plastic = model.material("light_plastic", rgba=(0.76, 0.78, 0.80, 1.0))
    silver = model.material("silver", rgba=(0.74, 0.76, 0.79, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.61, 0.66, 1.0))
    translucent_blade = model.material("translucent_blade", rgba=(0.78, 0.84, 0.90, 0.72))

    base = model.part("base")
    base.visual(_save_mesh("fan_base_shell", _build_base_mesh()), material=charcoal, name="base_shell")
    base.visual(
        Box((0.014, 0.056, 0.280)),
        origin=Origin(xyz=(0.035, 0.0, 0.200)),
        material=silver,
        name="socket_rib_right",
    )
    base.visual(
        Box((0.014, 0.056, 0.280)),
        origin=Origin(xyz=(-0.035, 0.0, 0.200)),
        material=silver,
        name="socket_rib_left",
    )
    base.visual(
        Box((0.056, 0.014, 0.280)),
        origin=Origin(xyz=(0.0, 0.035, 0.200)),
        material=silver,
        name="socket_rib_front",
    )
    base.visual(
        Box((0.056, 0.014, 0.280)),
        origin=Origin(xyz=(0.0, -0.035, 0.200)),
        material=silver,
        name="socket_rib_back",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.176, length=0.396),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
    )

    column = model.part("telescoping_column")
    column.visual(
        Cylinder(radius=0.028, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=satin_steel,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.571)),
        material=light_plastic,
        name="column_cap",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=0.580),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
    )

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.034, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=light_plastic,
        name="mount_neck",
    )
    housing.visual(
        Box((0.090, 0.050, 0.090)),
        origin=Origin(xyz=(0.0, -0.045, 0.145)),
        material=light_plastic,
        name="mount_block",
    )
    housing.visual(
        _save_mesh("fan_housing_frame", _build_housing_frame_mesh()),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=light_plastic,
        name="housing_frame",
    )
    housing.visual(
        _save_mesh("fan_motor_struts", _build_motor_struts_mesh()),
        material=satin_steel,
        name="motor_struts",
    )
    housing.visual(
        Cylinder(radius=0.084, length=0.100),
        origin=Origin(xyz=(0.0, -0.110, 0.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_plastic,
        name="motor_pod",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.044),
        origin=Origin(xyz=(0.0, -0.182, 0.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_plastic,
        name="rear_control_boss",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, -0.030, 0.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="axle_spindle",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.500, 0.270, 0.520)),
        mass=3.2,
        origin=Origin(xyz=(0.0, -0.035, 0.260)),
    )

    front_grille = model.part("front_grille")
    front_grille.visual(
        _save_mesh("fan_front_grille", _build_front_grille_mesh()),
        origin=Origin(xyz=(0.0, 0.080, 0.260)),
        material=satin_steel,
        name="front_grille",
    )
    front_grille.inertial = Inertial.from_geometry(
        Box((0.392, 0.012, 0.392)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.080, 0.260)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        _save_mesh("fan_propeller_blades", _build_propeller_blades_mesh()),
        material=translucent_blade,
        name="blade_pack",
    )
    propeller.visual(
        Cylinder(radius=0.048, length=0.056),
        origin=Origin(xyz=(0.0, 0.028, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="hub",
    )
    propeller.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.0, 0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="hub_cap",
    )
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.220, length=0.060),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.028, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_face",
    )
    timer_knob.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="knob_shaft",
    )
    timer_knob.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="knob_body",
    )
    timer_knob.visual(
        Box((0.006, 0.012, 0.0025)),
        origin=Origin(xyz=(0.015, -0.016, 0.0)),
        material=light_plastic,
        name="indicator",
    )
    timer_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.020),
        mass=0.04,
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "column_extension",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.15, lower=0.0, upper=0.180),
    )
    model.articulation(
        "column_to_housing",
        ArticulationType.FIXED,
        parent=column,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.580)),
    )
    model.articulation(
        "housing_to_front_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=front_grille,
        origin=Origin(),
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=propeller,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=25.0),
    )
    model.articulation(
        "timer_turn",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=timer_knob,
        origin=Origin(xyz=(0.0, -0.204, 0.260)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=2.0, lower=0.0, upper=4.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base")
    column = object_model.get_part("telescoping_column")
    housing = object_model.get_part("housing")
    front_grille = object_model.get_part("front_grille")
    propeller = object_model.get_part("propeller")
    timer_knob = object_model.get_part("timer_knob")

    column_extension = object_model.get_articulation("column_extension")
    fan_spin = object_model.get_articulation("fan_spin")
    timer_turn = object_model.get_articulation("timer_turn")

    socket_rib_right = base.get_visual("socket_rib_right")
    column_shaft = column.get_visual("column_shaft")
    column_cap = column.get_visual("column_cap")
    mount_neck = housing.get_visual("mount_neck")
    spindle = housing.get_visual("axle_spindle")
    rear_control_boss = housing.get_visual("rear_control_boss")
    grille_visual = front_grille.get_visual("front_grille")
    blade_pack = propeller.get_visual("blade_pack")
    hub = propeller.get_visual("hub")
    knob_shaft = timer_knob.get_visual("knob_shaft")
    knob_body = timer_knob.get_visual("knob_body")
    indicator = timer_knob.get_visual("indicator")

    ctx.expect_contact(column, base, elem_a=column_shaft, elem_b=socket_rib_right, name="column_seats_in_base_guide")
    ctx.expect_contact(housing, column, elem_a=mount_neck, elem_b=column_cap, name="housing_mounts_to_column_cap")
    ctx.expect_contact(front_grille, housing, elem_a=grille_visual, elem_b="housing_frame", name="front_grille_mounts_to_housing")
    ctx.expect_contact(propeller, housing, elem_a=hub, elem_b=spindle, name="propeller_hub_seats_on_axle")
    ctx.expect_contact(timer_knob, housing, elem_a=knob_shaft, elem_b=rear_control_boss, name="timer_knob_mounts_to_rear_control_boss")

    ctx.expect_gap(
        front_grille,
        propeller,
        axis="y",
        min_gap=0.006,
        max_gap=0.028,
        positive_elem=grille_visual,
        negative_elem="hub",
        name="hub_clears_front_grille",
    )
    ctx.expect_overlap(propeller, front_grille, axes="xz", min_overlap=0.180, elem_a=blade_pack, elem_b=grille_visual, name="propeller_centered_behind_grille")
    ctx.expect_within(
        propeller,
        housing,
        axes="xz",
        margin=0.020,
        inner_elem=blade_pack,
        outer_elem="housing_frame",
        name="propeller_stays_within_housing_opening",
    )

    ctx.check(
        "column_extension_axis_is_vertical",
        column_extension.axis == (0.0, 0.0, 1.0) and column_extension.articulation_type == ArticulationType.PRISMATIC,
        details=f"Expected vertical prismatic column, got type={column_extension.articulation_type} axis={column_extension.axis}.",
    )
    ctx.check(
        "fan_spin_axis_is_fore_aft",
        fan_spin.axis == (0.0, 1.0, 0.0),
        details=f"Expected fan spin around the fore-aft axis, got {fan_spin.axis}.",
    )
    ctx.check(
        "timer_knob_axis_is_fore_aft",
        timer_turn.axis == (0.0, 1.0, 0.0) and timer_turn.articulation_type == ArticulationType.REVOLUTE,
        details=f"Expected rear timer knob on a revolute fore-aft axis, got type={timer_turn.articulation_type} axis={timer_turn.axis}.",
    )

    housing_rest = ctx.part_world_position(housing)
    assert housing_rest is not None
    with ctx.pose({column_extension: 0.180}):
        housing_high = ctx.part_world_position(housing)
        assert housing_high is not None
        ctx.expect_contact(column, base, elem_a=column_shaft, elem_b=socket_rib_right, name="extended_column_remains_guided_in_base_guide")
        ctx.check(
            "pedestal_extension_raises_housing",
            housing_high[2] - housing_rest[2] > 0.17 and abs(housing_high[0] - housing_rest[0]) < 1e-6 and abs(housing_high[1] - housing_rest[1]) < 1e-6,
            details=f"Housing moved from {housing_rest} to {housing_high} instead of rising vertically by the extension distance.",
        )

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    indicator_rest = ctx.part_element_world_aabb(timer_knob, elem=indicator)
    assert indicator_rest is not None
    with ctx.pose({timer_turn: 2.3}):
        indicator_turned = ctx.part_element_world_aabb(timer_knob, elem=indicator)
        assert indicator_turned is not None
        rest_center = _aabb_center(indicator_rest)
        turned_center = _aabb_center(indicator_turned)
        ctx.expect_contact(timer_knob, housing, elem_a=knob_shaft, elem_b=rear_control_boss, name="timer_knob_stays_seated_when_turned")
        ctx.check(
            "timer_indicator_swings_around_knob_axis",
            abs(turned_center[0] - rest_center[0]) > 0.008 and abs(turned_center[2] - rest_center[2]) > 0.008,
            details=f"Indicator center did not move enough around the knob axis: rest={rest_center}, turned={turned_center}.",
        )

    with ctx.pose({fan_spin: math.pi / 5.0}):
        ctx.expect_contact(propeller, housing, elem_a=hub, elem_b=spindle, name="propeller_stays_on_axle_when_spun")
        ctx.expect_gap(
            front_grille,
            propeller,
            axis="y",
            min_gap=0.006,
            max_gap=0.028,
            positive_elem=grille_visual,
            negative_elem="hub",
            name="spun_propeller_hub_still_clears_front_grille",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
