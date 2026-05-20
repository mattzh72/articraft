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
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _radial_pattern(base_geom: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geom.copy().rotate_z(angle_offset + (index * math.tau / count)))
    return patterned


def _blade_section(
    x_pos: float,
    chord_width: float,
    thickness: float,
    pitch_deg: float,
    sweep_y: float,
    z_offset: float,
) -> list[tuple[float, float, float]]:
    half_width = chord_width * 0.5
    half_thickness = thickness * 0.5
    pitch = math.radians(pitch_deg)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)

    airfoil = [
        (0.94, 0.02),
        (0.40, 0.86),
        (-0.10, 1.00),
        (-0.78, 0.42),
        (-1.00, 0.00),
        (-0.70, -0.54),
        (-0.03, -0.84),
        (0.90, -0.18),
    ]

    loop: list[tuple[float, float, float]] = []
    for y_norm, z_norm in airfoil:
        y_local = y_norm * half_width
        z_local = z_norm * half_thickness
        y_rot = sweep_y + (y_local * cos_pitch) - (z_local * sin_pitch)
        z_rot = z_offset + (y_local * sin_pitch) + (z_local * cos_pitch)
        loop.append((x_pos, y_rot, z_rot))
    return loop


def _build_single_blade_mesh() -> MeshGeometry:
    sections = [
        _blade_section(0.142, 0.118, 0.0100, 15.0, -0.010, -0.031),
        _blade_section(0.274, 0.150, 0.0088, 13.0, 0.004, -0.033),
        _blade_section(0.414, 0.164, 0.0076, 11.0, 0.022, -0.036),
        _blade_section(0.548, 0.142, 0.0063, 9.0, 0.042, -0.039),
        _blade_section(0.652, 0.092, 0.0050, 7.0, 0.060, -0.041),
    ]
    return repair_loft(section_loft(sections), repair="mesh")


def _build_single_blade_iron_mesh() -> MeshGeometry:
    iron = MeshGeometry()
    iron.merge(BoxGeometry((0.024, 0.034, 0.012)).translate(0.091, 0.0, -0.046))
    iron.merge(
        BoxGeometry((0.076, 0.020, 0.008))
        .rotate_y(math.radians(7.5))
        .translate(0.122, 0.0, -0.042)
    )
    iron.merge(
        BoxGeometry((0.050, 0.040, 0.006))
        .rotate_y(math.radians(10.0))
        .translate(0.152, 0.0, -0.036)
    )
    iron.merge(CylinderGeometry(radius=0.0033, height=0.004, radial_segments=18).translate(0.147, 0.012, -0.033))
    iron.merge(CylinderGeometry(radius=0.0033, height=0.004, radial_segments=18).translate(0.147, -0.012, -0.033))
    iron.merge(CylinderGeometry(radius=0.0033, height=0.004, radial_segments=18).translate(0.086, 0.010, -0.043))
    iron.merge(CylinderGeometry(radius=0.0033, height=0.004, radial_segments=18).translate(0.086, -0.010, -0.043))
    return iron


def _build_rotor_cup_mesh() -> MeshGeometry:
    cup = LatheGeometry.from_shell_profiles(
        [
            (0.036, -0.040),
            (0.086, -0.036),
            (0.122, -0.064),
            (0.118, -0.104),
            (0.058, -0.118),
        ],
        [
            (0.028, -0.046),
            (0.074, -0.042),
            (0.106, -0.064),
            (0.100, -0.096),
            (0.048, -0.108),
        ],
        segments=72,
    )
    cup.merge(CylinderGeometry(radius=0.052, height=0.044, radial_segments=40).translate(0.0, 0.0, -0.062))
    cup.merge(CylinderGeometry(radius=0.124, height=0.012, radial_segments=56).translate(0.0, 0.0, -0.070))
    return cup


def _build_rotor_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.036, -0.040),
            (0.086, -0.036),
            (0.122, -0.064),
            (0.118, -0.104),
            (0.058, -0.118),
        ],
        [
            (0.028, -0.046),
            (0.074, -0.042),
            (0.106, -0.064),
            (0.100, -0.096),
            (0.048, -0.108),
        ],
        segments=72,
    )


def _build_canopy_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.016, -0.030),
            (0.058, -0.020),
            (0.078, 0.010),
            (0.072, 0.034),
        ],
        [
            (0.010, -0.028),
            (0.048, -0.018),
            (0.064, 0.010),
            (0.058, 0.031),
        ],
        segments=56,
    )


def _build_motor_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.034, -0.018),
            (0.082, -0.010),
            (0.108, 0.024),
            (0.094, 0.064),
            (0.040, 0.082),
        ],
        [
            (0.026, -0.010),
            (0.070, -0.004),
            (0.092, 0.024),
            (0.080, 0.058),
            (0.032, 0.072),
        ],
        segments=72,
    )


def _build_support_body_mesh() -> MeshGeometry:
    body = MeshGeometry()

    canopy = LatheGeometry.from_shell_profiles(
        [
            (0.016, -0.030),
            (0.058, -0.020),
            (0.078, 0.010),
            (0.072, 0.034),
        ],
        [
            (0.010, -0.028),
            (0.048, -0.018),
            (0.064, 0.010),
            (0.058, 0.031),
        ],
        segments=56,
    ).translate(-0.050, 0.0, 0.430)
    body.merge(canopy)
    body.merge(CylinderGeometry(radius=0.082, height=0.008, radial_segments=56).translate(-0.050, 0.0, 0.472))
    body.merge(CylinderGeometry(radius=0.017, height=0.032, radial_segments=28).translate(-0.050, 0.0, 0.414))

    rod_top = (-0.050, 0.0, 0.430)
    rod_bottom = (0.0, 0.0, 0.112)
    rod_dx = rod_top[0] - rod_bottom[0]
    rod_dz = rod_top[2] - rod_bottom[2]
    rod_length = math.hypot(rod_dx, rod_dz)
    rod_angle_y = math.atan2(rod_dx, rod_dz)
    body.merge(
        CylinderGeometry(radius=0.018, height=0.026, radial_segments=30)
        .translate(0.0, 0.0, 0.112)
    )
    body.merge(CylinderGeometry(radius=0.030, height=0.032, radial_segments=36).translate(0.0, 0.0, 0.106))
    body.merge(CylinderGeometry(radius=0.020, height=0.026, radial_segments=32).translate(0.0, 0.0, 0.086))
    body.merge(CylinderGeometry(radius=0.030, height=0.108, radial_segments=40).translate(0.0, 0.0, 0.020))

    upper_shell = LatheGeometry.from_shell_profiles(
        [
            (0.034, -0.018),
            (0.082, -0.010),
            (0.108, 0.024),
            (0.094, 0.064),
            (0.040, 0.082),
        ],
        [
            (0.026, -0.010),
            (0.070, -0.004),
            (0.092, 0.024),
            (0.080, 0.058),
            (0.032, 0.072),
        ],
        segments=72,
    )
    body.merge(upper_shell)

    # A small lower collar around the rod and the central motor core keep the
    # stationary body visually continuous down to the bearing plate.
    body.merge(
        CylinderGeometry(radius=0.012, height=rod_length, radial_segments=28)
        .rotate_y(rod_angle_y)
        .translate((rod_top[0] + rod_bottom[0]) * 0.5, 0.0, (rod_top[2] + rod_bottom[2]) * 0.5)
    )

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dc_motor_ceiling_fan")

    satin_nickel = model.material("satin_nickel", rgba=(0.73, 0.75, 0.78, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.33, 0.34, 0.36, 1.0))
    composite_blade = model.material("composite_blade", rgba=(0.42, 0.31, 0.23, 1.0))
    switch_black = model.material("switch_black", rgba=(0.08, 0.08, 0.09, 1.0))

    support = model.part("support_assembly")
    support.visual(
        _save_mesh("ceiling_fan_canopy_shell", _build_canopy_shell_mesh()),
        origin=Origin(xyz=(-0.050, 0.0, 0.430)),
        material=satin_nickel,
        name="canopy_shell",
    )

    rod_top = (-0.050, 0.0, 0.430)
    rod_bottom = (0.0, 0.0, 0.112)
    rod_dx = rod_top[0] - rod_bottom[0]
    rod_dz = rod_top[2] - rod_bottom[2]
    rod_length = math.hypot(rod_dx, rod_dz)
    rod_angle_y = math.atan2(rod_dx, rod_dz)
    support.visual(
        Cylinder(radius=0.012, length=rod_length),
        origin=Origin(
            xyz=((rod_top[0] + rod_bottom[0]) * 0.5, 0.0, (rod_top[2] + rod_bottom[2]) * 0.5),
            rpy=(0.0, rod_angle_y, 0.0),
        ),
        material=satin_nickel,
        name="downrod",
    )
    support.visual(
        Cylinder(radius=0.082, length=0.008),
        origin=Origin(xyz=(-0.050, 0.0, 0.466)),
        material=satin_nickel,
        name="canopy_trim",
    )
    support.visual(
        Cylinder(radius=0.017, length=0.036),
        origin=Origin(xyz=(-0.050, 0.0, 0.414)),
        material=satin_nickel,
        name="canopy_stem",
    )
    support.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=satin_nickel,
        name="downrod_coupler",
    )
    support.visual(
        Cylinder(radius=0.030, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=satin_nickel,
        name="yoke_knuckle",
    )
    support.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=satin_nickel,
        name="upper_spindle",
    )
    support.visual(
        _save_mesh("ceiling_fan_motor_shell", _build_motor_shell_mesh()),
        material=satin_nickel,
        name="motor_shell",
    )
    support.visual(
        Cylinder(radius=0.034, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=graphite,
        name="motor_neck",
    )
    support.visual(
        Cylinder(radius=0.030, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=graphite,
        name="motor_core",
    )
    support.visual(
        Cylinder(radius=0.090, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=graphite,
        name="stator_band",
    )
    support.visual(
        Cylinder(radius=0.058, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=graphite,
        name="bearing_plate",
    )
    support.visual(
        Box((0.024, 0.028, 0.026)),
        origin=Origin(xyz=(0.101, 0.0, 0.003)),
        material=graphite,
        name="switch_pad",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.22, 0.18, 0.56)),
        mass=8.0,
        origin=Origin(xyz=(-0.025, 0.0, 0.205)),
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        _save_mesh("ceiling_fan_rotor_shell", _build_rotor_shell_mesh()),
        material=graphite,
        name="rotor_shell",
    )
    blade_assembly.visual(
        Cylinder(radius=0.052, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=graphite,
        name="rotor_core",
    )
    blade_assembly.visual(
        Cylinder(radius=0.124, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=graphite,
        name="rotor_rim",
    )
    blade_assembly.visual(
        Cylinder(radius=0.056, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=graphite,
        name="hub_cap",
    )
    single_blade_mesh = _save_mesh("ceiling_fan_single_blade", _build_single_blade_mesh())
    for blade_index in range(5):
        angle = blade_index * math.tau / 5.0
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        def _xy(x_pos: float, y_pos: float = 0.0) -> tuple[float, float]:
            return (x_pos * cos_a) - (y_pos * sin_a), (x_pos * sin_a) + (y_pos * cos_a)

        root_x, root_y = _xy(0.090)
        blade_assembly.visual(
            Box((0.022, 0.034, 0.030)),
            origin=Origin(xyz=(root_x, root_y, -0.055), rpy=(0.0, 0.0, angle)),
            material=bracket_steel,
            name=f"blade_iron_root_{blade_index}",
        )
        arm_x, arm_y = _xy(0.126)
        blade_assembly.visual(
            Box((0.072, 0.022, 0.008)),
            origin=Origin(xyz=(arm_x, arm_y, -0.044), rpy=(0.0, 0.0, angle)),
            material=bracket_steel,
            name=f"blade_iron_arm_{blade_index}",
        )
        plate_x, plate_y = _xy(0.154)
        blade_assembly.visual(
            Box((0.052, 0.040, 0.006)),
            origin=Origin(xyz=(plate_x, plate_y, -0.037), rpy=(0.0, 0.0, angle)),
            material=bracket_steel,
            name=f"blade_iron_plate_{blade_index}",
        )
        bolt_in_x, bolt_in_y = _xy(0.151, 0.012)
        blade_assembly.visual(
            Box((0.006, 0.006, 0.004)),
            origin=Origin(xyz=(bolt_in_x, bolt_in_y, -0.034), rpy=(0.0, 0.0, angle)),
            material=bracket_steel,
            name=f"blade_bolt_inboard_{blade_index}",
        )
        bolt_out_x, bolt_out_y = _xy(0.151, -0.012)
        blade_assembly.visual(
            Box((0.006, 0.006, 0.004)),
            origin=Origin(xyz=(bolt_out_x, bolt_out_y, -0.034), rpy=(0.0, 0.0, angle)),
            material=bracket_steel,
            name=f"blade_bolt_outboard_{blade_index}",
        )
        blade_assembly.visual(
            single_blade_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=composite_blade,
            name=f"blade_{blade_index}",
        )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.66, length=0.12),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    reverse_toggle = model.part("reverse_toggle")
    reverse_toggle.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=switch_black,
        name="pivot_barrel",
    )
    reverse_toggle.visual(
        Box((0.018, 0.004, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=switch_black,
        name="toggle_stem",
    )
    reverse_toggle.visual(
        Box((0.008, 0.006, 0.014)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=switch_black,
        name="toggle_tip",
    )
    reverse_toggle.inertial = Inertial.from_geometry(
        Box((0.032, 0.020, 0.020)),
        mass=0.05,
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_blade_assembly",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=blade_assembly,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=26.0),
    )
    model.articulation(
        "support_to_reverse_toggle",
        ArticulationType.REVOLUTE,
        parent=support,
        child=reverse_toggle,
        origin=Origin(xyz=(0.114, 0.0, 0.005)),
        # The toggle lever extends along local +X from the pivot barrel, so use
        # -Y to make positive angles lift the tip upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-0.48,
            upper=0.48,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_assembly")
    blade_assembly = object_model.get_part("blade_assembly")
    reverse_toggle = object_model.get_part("reverse_toggle")
    blade_spin = object_model.get_articulation("support_to_blade_assembly")
    toggle_pivot = object_model.get_articulation("support_to_reverse_toggle")

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

    ctx.expect_contact(
        blade_assembly,
        support,
        elem_a="hub_cap",
        elem_b="bearing_plate",
        contact_tol=5e-4,
        name="blade assembly is carried on the central bearing plate",
    )
    ctx.expect_overlap(
        blade_assembly,
        support,
        axes="xy",
        elem_a="hub_cap",
        elem_b="bearing_plate",
        min_overlap=0.10,
        name="hub cap stays centered under the bearing plate",
    )
    ctx.expect_contact(
        reverse_toggle,
        support,
        elem_a="pivot_barrel",
        elem_b="switch_pad",
        contact_tol=5e-4,
        name="reverse toggle pivot sits on the housing switch pad",
    )

    ctx.check(
        "blade spin joint is a vertical continuous axle",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 3) for value in blade_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={blade_spin.articulation_type}, axis={blade_spin.axis}",
    )
    ctx.check(
        "reverse toggle pivots on a tangential hinge axis",
        toggle_pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(value, 3) for value in toggle_pivot.axis) == (0.0, -1.0, 0.0),
        details=f"type={toggle_pivot.articulation_type}, axis={toggle_pivot.axis}",
    )

    blade_aabb = ctx.part_world_aabb(blade_assembly)
    blade_span = None
    if blade_aabb is not None:
        blade_span = max(
            blade_aabb[1][0] - blade_aabb[0][0],
            blade_aabb[1][1] - blade_aabb[0][1],
        )
    ctx.check(
        "fan span reads as a full-size ceiling fan",
        blade_span is not None and 1.20 <= blade_span <= 1.45,
        details=f"span={blade_span}",
    )

    support_aabb = ctx.part_world_aabb(support)
    support_drop = None
    if support_aabb is not None:
        support_drop = support_aabb[1][2] - support_aabb[0][2]
    ctx.check(
        "support assembly provides a realistic ceiling drop",
        support_drop is not None and support_drop >= 0.44,
        details=f"drop={support_drop}",
    )

    downrod_aabb = ctx.part_element_world_aabb(support, elem="downrod")
    downrod_x_span = None
    if downrod_aabb is not None:
        downrod_x_span = downrod_aabb[1][0] - downrod_aabb[0][0]
    ctx.check(
        "downrod is visibly angled rather than vertical",
        downrod_x_span is not None and downrod_x_span >= 0.05,
        details=f"downrod_x_span={downrod_x_span}",
    )

    lower_tip_center_z = None
    upper_tip_center_z = None
    toggle_limits = toggle_pivot.motion_limits
    if toggle_limits is not None and toggle_limits.lower is not None and toggle_limits.upper is not None:
        with ctx.pose({toggle_pivot: toggle_limits.lower}):
            lower_tip_aabb = ctx.part_element_world_aabb(reverse_toggle, elem="toggle_tip")
            if lower_tip_aabb is not None:
                lower_tip_center_z = 0.5 * (lower_tip_aabb[0][2] + lower_tip_aabb[1][2])
        with ctx.pose({toggle_pivot: toggle_limits.upper}):
            upper_tip_aabb = ctx.part_element_world_aabb(reverse_toggle, elem="toggle_tip")
            if upper_tip_aabb is not None:
                upper_tip_center_z = 0.5 * (upper_tip_aabb[0][2] + upper_tip_aabb[1][2])
    ctx.check(
        "positive toggle motion lifts the switch tip",
        lower_tip_center_z is not None
        and upper_tip_center_z is not None
        and upper_tip_center_z > lower_tip_center_z + 0.010,
        details=f"lower_z={lower_tip_center_z}, upper_z={upper_tip_center_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
