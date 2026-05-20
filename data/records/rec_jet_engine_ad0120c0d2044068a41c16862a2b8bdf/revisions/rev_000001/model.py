from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _merged(items: list[MeshGeometry]) -> MeshGeometry:
    result = MeshGeometry()
    for item in items:
        result.merge(item)
    return result


def _as_world_x(geometry: MeshGeometry) -> MeshGeometry:
    """Convert engine-local +Z axis geometry to a horizontal world +X engine."""
    return geometry.rotate_y(math.pi / 2.0)


def _ring(
    outer_radius: float,
    inner_radius: float,
    z_center: float,
    width: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=width, radial_segments=segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=width + 0.010,
        radial_segments=segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _airfoil_loop(
    radius: float,
    tangential: float,
    z_pos: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    """A compact airfoil-like closed loop in radial/tangential/axial coordinates."""
    return [
        (radius, tangential - 0.52 * thickness, z_pos - 0.50 * chord),
        (radius, tangential + 0.10 * thickness, z_pos - 0.38 * chord),
        (radius, tangential + 0.54 * thickness, z_pos - 0.08 * chord),
        (radius, tangential + 0.38 * thickness, z_pos + 0.34 * chord),
        (radius, tangential - 0.06 * thickness, z_pos + 0.50 * chord),
        (radius, tangential - 0.46 * thickness, z_pos + 0.16 * chord),
    ]


def _loft(sections: list[list[tuple[float, float, float]]]) -> MeshGeometry:
    return repair_loft(section_loft(sections))


def _radial_pattern(base: MeshGeometry, count: int, *, phase: float = 0.0) -> MeshGeometry:
    pattern = MeshGeometry()
    for index in range(count):
        pattern.merge(base.copy().rotate_z(phase + index * math.tau / count))
    return pattern


def _nacelle_shell() -> MeshGeometry:
    # A thin, hollow turbofan nacelle with a rounded intake lip and tapered bypass duct.
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.850, -1.360),
            (0.970, -1.260),
            (1.065, -0.840),
            (1.030, -0.100),
            (0.930, 0.620),
            (0.760, 1.210),
            (0.630, 1.570),
        ],
        [
            (0.735, -1.350),
            (0.805, -1.180),
            (0.840, -0.560),
            (0.805, 0.320),
            (0.705, 0.980),
            (0.560, 1.500),
            (0.505, 1.620),
        ],
        segments=112,
        start_cap="round",
        end_cap="flat",
        lip_samples=12,
    )
    shell.merge(TorusGeometry(radius=0.790, tube=0.035, radial_segments=18, tubular_segments=96).translate(0.0, 0.0, -1.305))
    return _as_world_x(shell)


def _service_bands() -> MeshGeometry:
    bands = [
        _ring(1.075, 1.025, -0.820, 0.050, segments=96),
        _ring(1.005, 0.970, -0.050, 0.035, segments=96),
    ]
    return _as_world_x(_merged(bands))


def _core_case() -> MeshGeometry:
    # Open compressor-case cage: annular frames and axial longerons leave the
    # rotating spool visible, avoiding a solid proxy hidden around the blades.
    elements = [
        _ring(0.405, 0.275, -0.620, 0.070, segments=80),
        _ring(0.430, 0.295, -0.080, 0.055, segments=80),
        _ring(0.405, 0.285, 0.620, 0.055, segments=80),
        _ring(0.335, 0.230, 1.120, 0.065, segments=72),
    ]
    for angle in [i * math.tau / 8.0 for i in range(8)]:
        radius = 0.356
        elements.append(
            CylinderGeometry(radius=0.018, height=2.080, radial_segments=16).translate(
                radius * math.cos(angle),
                radius * math.sin(angle),
                0.170,
            )
        )
    return _as_world_x(_merged(elements))


def _fan_bearing_support() -> MeshGeometry:
    # Static shaft and spider support for the rotating front fan bearing.
    shaft = CylinderGeometry(radius=0.052, height=0.720, radial_segments=48).translate(0.0, 0.0, -0.900)
    cross_x = BoxGeometry((0.560, 0.042, 0.046)).translate(0.0, 0.0, -0.760)
    cross_y = BoxGeometry((0.042, 0.560, 0.046)).translate(0.0, 0.0, -0.760)
    return _as_world_x(_merged([shaft, cross_x, cross_y]))


def _core_bearing_support() -> MeshGeometry:
    sleeve = _ring(0.090, 0.045, -0.505, 0.085, segments=48)
    cross_x = BoxGeometry((0.540, 0.036, 0.050)).translate(0.0, 0.0, -0.505)
    cross_y = BoxGeometry((0.036, 0.540, 0.050)).translate(0.0, 0.0, -0.505)
    return _as_world_x(_merged([sleeve, cross_x, cross_y]))


def _exhaust_nozzle() -> MeshGeometry:
    nozzle = LatheGeometry.from_shell_profiles(
        [
            (0.560, 1.010),
            (0.525, 1.250),
            (0.455, 1.515),
            (0.375, 1.760),
        ],
        [
            (0.335, 1.000),
            (0.315, 1.250),
            (0.290, 1.515),
            (0.260, 1.780),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )
    nozzle.merge(_ring(0.565, 0.335, 1.030, 0.060, segments=80))
    nozzle.merge(_ring(0.405, 0.260, 1.750, 0.045, segments=80))
    return _as_world_x(nozzle)


def _chevron_petals() -> MeshGeometry:
    def petal_section(z_pos: float, inner: float, outer: float, width: float, sweep: float) -> list[tuple[float, float, float]]:
        half = width * 0.5
        return [
            (inner, -half, z_pos),
            (outer, -0.35 * half + sweep, z_pos),
            (outer, 0.35 * half + sweep, z_pos),
            (inner, half, z_pos),
        ]

    long_petal = _loft(
        [
            petal_section(1.505, 0.455, 0.500, 0.145, 0.000),
            petal_section(1.625, 0.405, 0.450, 0.105, -0.006),
            petal_section(1.770, 0.300, 0.352, 0.038, -0.014),
        ]
    )
    short_petal = _loft(
        [
            petal_section(1.525, 0.445, 0.485, 0.115, 0.000),
            petal_section(1.650, 0.390, 0.430, 0.078, 0.007),
            petal_section(1.735, 0.318, 0.360, 0.030, 0.012),
        ]
    )
    return _as_world_x(
        _merged(
            [
                _radial_pattern(long_petal, 14),
                _radial_pattern(short_petal, 14, phase=math.pi / 14.0),
            ]
        )
    )


def _stator_vanes() -> MeshGeometry:
    fan_exit_vane = _loft(
        [
            _airfoil_loop(0.300, -0.010, -0.720, 0.170, 0.030),
            _airfoil_loop(0.565, 0.010, -0.675, 0.138, 0.024),
            _airfoil_loop(0.875, 0.030, -0.615, 0.088, 0.016),
        ]
    )
    rear_strut = _loft(
        [
            _airfoil_loop(0.330, 0.000, 0.875, 0.185, 0.038),
            _airfoil_loop(0.575, 0.012, 0.925, 0.150, 0.032),
            _airfoil_loop(0.745, 0.018, 0.980, 0.112, 0.024),
        ]
    )
    vanes = _merged(
        [
            _ring(0.885, 0.850, -0.620, 0.060, segments=88),
            _ring(0.330, 0.260, -0.715, 0.065, segments=72),
            _radial_pattern(fan_exit_vane, 18, phase=math.pi / 18.0),
            _radial_pattern(rear_strut, 8, phase=math.pi / 8.0),
        ]
    )
    return _as_world_x(vanes)


def _pylon_fairing() -> MeshGeometry:
    # World-space tapered pylon saddle attached to the top of the nacelle.
    def section(x_pos: float, half_width: float, bottom: float, top: float) -> list[tuple[float, float, float]]:
        return [
            (x_pos, -half_width, bottom),
            (x_pos, half_width, bottom),
            (x_pos, half_width * 0.82, top),
            (x_pos, -half_width * 0.82, top),
        ]

    pylon = _loft(
        [
            section(-0.620, 0.145, 0.900, 1.320),
            section(0.100, 0.185, 0.940, 1.620),
            section(0.940, 0.150, 0.820, 1.520),
        ]
    )
    return pylon


def _spool_stack() -> MeshGeometry:
    shaft = CylinderGeometry(radius=0.052, height=1.930, radial_segments=48).translate(0.0, 0.0, 0.465)
    parts = [shaft]
    for idx, (z_pos, root, tip, count, phase) in enumerate(
        [
            (-0.395, 0.080, 0.228, 28, 0.00),
            (-0.145, 0.075, 0.248, 30, 0.03),
            (0.125, 0.072, 0.238, 28, 0.01),
            (0.395, 0.070, 0.214, 24, -0.02),
        ]
    ):
        parts.append(_ring(root + 0.030, 0.050, z_pos - 0.018, 0.030, segments=56))
        blade = _loft(
            [
                _airfoil_loop(root, -0.006, z_pos - 0.038, 0.075, 0.012),
                _airfoil_loop((root + tip) * 0.5, 0.004, z_pos, 0.060, 0.010),
                _airfoil_loop(tip, 0.014, z_pos + 0.034, 0.042, 0.006),
            ]
        )
        parts.append(_radial_pattern(blade, count, phase=phase + math.pi / count + idx * 0.02))

    for z_pos, root, tip, count, phase in [
        (0.865, 0.078, 0.224, 22, 0.02),
        (1.105, 0.070, 0.206, 20, -0.01),
        (1.305, 0.064, 0.178, 18, 0.04),
    ]:
        parts.append(_ring(root + 0.026, 0.050, z_pos, 0.032, segments=52))
        blade = _loft(
            [
                _airfoil_loop(root, -0.004, z_pos - 0.030, 0.062, 0.012),
                _airfoil_loop((root + tip) * 0.5, 0.006, z_pos, 0.050, 0.009),
                _airfoil_loop(tip, 0.012, z_pos + 0.028, 0.034, 0.006),
            ]
        )
        parts.append(_radial_pattern(blade, count, phase=phase + math.pi / count))
    return _merged(parts)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_bypass_turbofan")

    nacelle_paint = model.material("ceramic_white", rgba=(0.80, 0.83, 0.86, 1.0))
    intake_metal = model.material("polished_lip", rgba=(0.70, 0.73, 0.78, 1.0))
    dark_metal = model.material("dark_titanium", rgba=(0.16, 0.17, 0.19, 1.0))
    blade_metal = model.material("fan_titanium", rgba=(0.55, 0.58, 0.62, 1.0))
    hot_metal = model.material("heat_stained_alloy", rgba=(0.50, 0.40, 0.31, 1.0))

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(_nacelle_shell(), "nacelle_shell"),
        material=nacelle_paint,
        name="nacelle_shell",
    )
    nacelle.visual(
        mesh_from_geometry(_service_bands(), "service_bands"),
        material=intake_metal,
        name="service_bands",
    )
    nacelle.visual(
        mesh_from_geometry(_core_case(), "core_case"),
        material=dark_metal,
        name="core_case",
    )
    nacelle.visual(
        mesh_from_geometry(_stator_vanes(), "stator_vanes"),
        material=dark_metal,
        name="stator_vanes",
    )
    nacelle.visual(
        mesh_from_geometry(_fan_bearing_support(), "fan_bearing_support"),
        material=dark_metal,
        name="fan_bearing_support",
    )
    nacelle.visual(
        mesh_from_geometry(_core_bearing_support(), "core_bearing_support"),
        material=dark_metal,
        name="core_bearing_support",
    )
    nacelle.visual(
        mesh_from_geometry(_exhaust_nozzle(), "exhaust_nozzle"),
        material=hot_metal,
        name="exhaust_nozzle",
    )
    nacelle.visual(
        mesh_from_geometry(_chevron_petals(), "chevron_petals"),
        material=hot_metal,
        name="chevron_petals",
    )
    nacelle.visual(
        mesh_from_geometry(_pylon_fairing(), "pylon_fairing"),
        material=nacelle_paint,
        name="pylon_fairing",
    )
    fan = model.part("fan")
    fan.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                outer_radius=0.775,
                hub_radius=0.185,
                blade_count=24,
                thickness=0.115,
                blade_pitch_deg=33.0,
                blade_sweep_deg=31.0,
                blade=FanRotorBlade(
                    shape="scimitar",
                    tip_pitch_deg=13.0,
                    camber=0.18,
                    tip_clearance=0.018,
                ),
                hub=FanRotorHub(
                    style="spinner",
                    rear_collar_height=0.045,
                    rear_collar_radius=0.180,
                    bore_diameter=0.090,
                ),
            ),
            "fan_rotor",
        ),
        material=blade_metal,
        name="fan_rotor",
    )

    core_spool = model.part("core_spool")
    core_spool.visual(
        mesh_from_geometry(_spool_stack(), "compressor_turbine_spool"),
        material=blade_metal,
        name="compressor_turbine_spool",
    )

    model.articulation(
        "nacelle_to_fan",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan,
        origin=Origin(xyz=(-1.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=180.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.0),
    )
    model.articulation(
        "nacelle_to_core_spool",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=core_spool,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=260.0),
        motion_properties=MotionProperties(damping=0.015, friction=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    nacelle = object_model.get_part("nacelle")
    fan = object_model.get_part("fan")
    spool = object_model.get_part("core_spool")
    fan_joint = object_model.get_articulation("nacelle_to_fan")
    spool_joint = object_model.get_articulation("nacelle_to_core_spool")

    ctx.allow_overlap(
        nacelle,
        fan,
        elem_a="fan_bearing_support",
        elem_b="fan_rotor",
        reason="The fan rotor hub is intentionally captured on the static bearing shaft; the small local overlap represents the seated bearing fit.",
    )
    ctx.expect_overlap(
        nacelle,
        fan,
        axes="x",
        elem_a="fan_bearing_support",
        elem_b="fan_rotor",
        min_overlap=0.080,
        name="fan bearing remains inserted through the hub",
    )
    ctx.expect_within(
        nacelle,
        fan,
        axes="yz",
        inner_elem="fan_bearing_support",
        outer_elem="fan_rotor",
        margin=0.600,
        name="fan bearing is coaxial with the rotor hub",
    )
    ctx.allow_overlap(
        nacelle,
        spool,
        elem_a="core_bearing_support",
        elem_b="compressor_turbine_spool",
        reason="The core spool shaft is intentionally captured inside a static front bearing sleeve.",
    )
    ctx.expect_overlap(
        nacelle,
        spool,
        axes="x",
        elem_a="core_bearing_support",
        elem_b="compressor_turbine_spool",
        min_overlap=0.030,
        name="core spool bearing retains the shaft",
    )

    ctx.expect_within(
        fan,
        nacelle,
        axes="yz",
        inner_elem="fan_rotor",
        outer_elem="nacelle_shell",
        margin=0.020,
        name="fan disk is contained inside the intake annulus",
    )
    ctx.expect_overlap(
        fan,
        nacelle,
        axes="x",
        elem_a="fan_rotor",
        elem_b="nacelle_shell",
        min_overlap=0.050,
        name="fan sits in the nacelle intake depth",
    )
    ctx.expect_within(
        spool,
        nacelle,
        axes="yz",
        inner_elem="compressor_turbine_spool",
        outer_elem="core_case",
        margin=0.020,
        name="core spool is centered inside the hollow core casing",
    )

    fan_rest = ctx.part_world_position(fan)
    spool_rest = ctx.part_world_position(spool)
    with ctx.pose({fan_joint: 1.25, spool_joint: -1.70}):
        fan_rotated = ctx.part_world_position(fan)
        spool_rotated = ctx.part_world_position(spool)

    ctx.check(
        "coaxial rotors spin without translating",
        fan_rest is not None
        and fan_rotated is not None
        and spool_rest is not None
        and spool_rotated is not None
        and max(abs(fan_rest[i] - fan_rotated[i]) for i in range(3)) < 1e-6
        and max(abs(spool_rest[i] - spool_rotated[i]) for i in range(3)) < 1e-6,
        details=f"fan {fan_rest}->{fan_rotated}; spool {spool_rest}->{spool_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
