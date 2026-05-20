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
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _x_axis_lathe(
    profile: list[tuple[float, float]],
    *,
    segments: int = 80,
) -> MeshGeometry:
    return LatheGeometry(profile, segments=segments).rotate_y(math.pi / 2.0)


def _x_axis_shell(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 88,
    start_cap: str = "flat",
    end_cap: str = "flat",
    lip_samples: int = 8,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap=start_cap,
        end_cap=end_cap,
        lip_samples=lip_samples,
    ).rotate_y(math.pi / 2.0)


def _radial_pattern_x(
    base_geometry: MeshGeometry,
    count: int,
    *,
    angle_offset: float = 0.0,
) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geometry.copy().rotate_x(angle_offset + (index * math.tau / count)))
    return patterned


def _fan_blade_section(
    radius: float,
    x_center: float,
    chord: float,
    thickness: float,
    swirl: float,
) -> list[tuple[float, float, float]]:
    half_t = thickness * 0.5
    return [
        (x_center - 0.50 * chord, radius, swirl - 0.90 * half_t),
        (x_center - 0.12 * chord, radius, swirl + 1.00 * half_t),
        (x_center + 0.46 * chord, radius, swirl + 0.24 * half_t),
        (x_center + 0.24 * chord, radius, swirl - 0.78 * half_t),
    ]


def _build_nacelle_shell_mesh() -> MeshGeometry:
    return _x_axis_shell(
        [
            (1.080, -0.120),
            (1.160, 0.020),
            (1.220, 0.360),
            (1.235, 1.100),
            (1.210, 1.950),
            (1.120, 2.880),
            (0.960, 3.760),
            (0.820, 4.260),
        ],
        [
            (0.950, -0.140),
            (1.030, -0.010),
            (0.990, 0.320),
            (0.970, 1.150),
            (0.915, 2.000),
            (0.800, 2.980),
            (0.670, 3.880),
            (0.590, 4.300),
        ],
        segments=104,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )


def _build_core_shell_mesh() -> MeshGeometry:
    return _x_axis_shell(
        [
            (0.340, 1.140),
            (0.410, 1.460),
            (0.445, 2.020),
            (0.425, 2.760),
            (0.365, 3.430),
            (0.295, 3.980),
        ],
        [
            (0.225, 1.110),
            (0.295, 1.470),
            (0.325, 2.040),
            (0.305, 2.800),
            (0.255, 3.470),
            (0.195, 4.040),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )


def _build_core_mount_mesh() -> MeshGeometry:
    return _x_axis_lathe(
        [
            (0.000, 0.920),
            (0.080, 0.920),
            (0.110, 0.980),
            (0.175, 1.080),
            (0.270, 1.180),
            (0.335, 1.255),
            (0.000, 1.255),
        ],
        segments=72,
    )


def _build_center_shaft_mesh() -> MeshGeometry:
    return CylinderGeometry(radius=0.082, height=1.260, radial_segments=48).rotate_y(
        math.pi / 2.0
    ).translate(1.690, 0.0, 0.0)


def _build_exhaust_plug_mesh() -> MeshGeometry:
    return _x_axis_lathe(
        [
            (0.082, 2.250),
            (0.140, 2.420),
            (0.205, 2.880),
            (0.218, 3.380),
            (0.188, 3.820),
            (0.118, 4.120),
            (0.040, 4.280),
            (0.000, 4.340),
        ],
        segments=72,
    )


def _build_support_struts_mesh() -> MeshGeometry:
    base = BoxGeometry((0.360, 0.640, 0.050)).translate(1.280, 0.670, 0.0)
    return _radial_pattern_x(base, 4, angle_offset=math.pi / 4.0)


def _build_rotor_mesh() -> MeshGeometry:
    spinner = _x_axis_lathe(
        [
            (0.000, -0.780),
            (0.065, -0.752),
            (0.145, -0.680),
            (0.238, -0.545),
            (0.300, -0.375),
            (0.275, -0.210),
            (0.210, -0.085),
            (0.000, -0.085),
        ],
        segments=72,
    )
    hub = CylinderGeometry(radius=0.205, height=0.265, radial_segments=56).rotate_y(
        math.pi / 2.0
    ).translate(-0.205, 0.0, 0.0)
    rear_flange = CylinderGeometry(radius=0.245, height=0.060, radial_segments=56).rotate_y(
        math.pi / 2.0
    ).translate(-0.055, 0.0, 0.0)

    blade = section_loft(
        [
            _fan_blade_section(0.185, -0.330, 0.480, 0.052, -0.008),
            _fan_blade_section(0.420, -0.285, 0.410, 0.040, 0.010),
            _fan_blade_section(0.670, -0.215, 0.310, 0.026, 0.036),
            _fan_blade_section(0.860, -0.165, 0.215, 0.014, 0.060),
        ]
    )
    blades = _radial_pattern_x(blade, 16, angle_offset=math.pi / 16.0)

    return _merge_geometries([spinner, hub, rear_flange, blades])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="airliner_turbofan_access_panel")

    nacelle_white = model.material("nacelle_white", rgba=(0.83, 0.85, 0.88, 1.0))
    intake_gray = model.material("intake_gray", rgba=(0.58, 0.61, 0.66, 1.0))
    rotor_metal = model.material("rotor_metal", rgba=(0.68, 0.72, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.76, 0.79, 0.82, 1.0))

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(_build_nacelle_shell_mesh(), "nacelle_shell"),
        material=nacelle_white,
        name="nacelle_shell",
    )
    nacelle.visual(
        mesh_from_geometry(_build_core_shell_mesh(), "core_shell"),
        material=intake_gray,
        name="core_shell",
    )
    nacelle.visual(
        mesh_from_geometry(_build_support_struts_mesh(), "core_support_struts"),
        material=dark_steel,
        name="core_support_struts",
    )
    nacelle.visual(
        mesh_from_geometry(_build_core_mount_mesh(), "core_mount"),
        material=dark_steel,
        name="bearing_hub",
    )
    nacelle.visual(
        mesh_from_geometry(_build_center_shaft_mesh(), "center_shaft"),
        material=dark_steel,
        name="center_shaft",
    )
    nacelle.visual(
        mesh_from_geometry(_build_exhaust_plug_mesh(), "exhaust_plug"),
        material=intake_gray,
        name="exhaust_plug",
    )
    nacelle.visual(
        Cylinder(radius=0.018, length=0.130),
        origin=Origin(xyz=(0.560, -0.155, -1.290), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle_left",
    )
    nacelle.visual(
        Cylinder(radius=0.018, length=0.130),
        origin=Origin(xyz=(0.560, 0.155, -1.290), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle_right",
    )
    nacelle.visual(
        Box((0.080, 0.080, 0.090)),
        origin=Origin(xyz=(0.560, -0.155, -1.230)),
        material=dark_steel,
        name="hinge_bracket_left",
    )
    nacelle.visual(
        Box((0.080, 0.080, 0.090)),
        origin=Origin(xyz=(0.560, 0.155, -1.230)),
        material=dark_steel,
        name="hinge_bracket_right",
    )
    nacelle.inertial = Inertial.from_geometry(
        Cylinder(radius=1.24, length=4.36),
        mass=1800.0,
        origin=Origin(xyz=(2.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        mesh_from_geometry(_build_rotor_mesh(), "fan_rotor"),
        material=rotor_metal,
        name="fan_rotor",
    )
    fan_rotor.visual(
        Cylinder(radius=0.090, length=0.110),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rotor_shaft",
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.88, length=0.82),
        mass=320.0,
        origin=Origin(xyz=(-0.350, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((0.580, 0.420, 0.012)),
        origin=Origin(xyz=(0.310, 0.0, -0.024)),
        material=panel_gray,
        name="panel_leaf",
    )
    access_panel.visual(
        Box((0.420, 0.220, 0.010)),
        origin=Origin(xyz=(0.340, 0.0, -0.014)),
        material=panel_gray,
        name="panel_stiffener",
    )
    access_panel.visual(
        Cylinder(radius=0.018, length=0.180),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="panel_hinge_barrel",
    )
    access_panel.visual(
        Box((0.060, 0.050, 0.024)),
        origin=Origin(xyz=(0.035, -0.060, -0.012)),
        material=dark_steel,
        name="panel_hinge_arm_left",
    )
    access_panel.visual(
        Box((0.060, 0.050, 0.024)),
        origin=Origin(xyz=(0.035, 0.060, -0.012)),
        material=dark_steel,
        name="panel_hinge_arm_right",
    )
    access_panel.inertial = Inertial.from_geometry(
        Box((0.600, 0.420, 0.050)),
        mass=18.0,
        origin=Origin(xyz=(0.300, 0.0, -0.020)),
    )

    model.articulation(
        "nacelle_to_fan_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan_rotor,
        origin=Origin(xyz=(0.920, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=220.0),
    )

    model.articulation(
        "nacelle_to_access_panel",
        ArticulationType.REVOLUTE,
        parent=nacelle,
        child=access_panel,
        origin=Origin(xyz=(0.560, 0.0, -1.290)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    nacelle = object_model.get_part("nacelle")
    fan_rotor = object_model.get_part("fan_rotor")
    access_panel = object_model.get_part("access_panel")
    fan_joint = object_model.get_articulation("nacelle_to_fan_rotor")
    panel_joint = object_model.get_articulation("nacelle_to_access_panel")

    fan_limits = fan_joint.motion_limits
    panel_limits = panel_joint.motion_limits

    ctx.check(
        "all prompt-critical parts exist",
        nacelle is not None and fan_rotor is not None and access_panel is not None,
        details="Expected nacelle, fan_rotor, and access_panel parts.",
    )
    ctx.check(
        "fan rotor spins continuously about engine axis",
        fan_joint.joint_type == ArticulationType.CONTINUOUS
        and fan_joint.axis == (1.0, 0.0, 0.0)
        and fan_limits is not None
        and fan_limits.lower is None
        and fan_limits.upper is None,
        details=(
            f"type={fan_joint.joint_type}, axis={fan_joint.axis}, "
            f"limits={fan_limits}"
        ),
    )
    ctx.check(
        "access panel hinges downward from the lower nacelle",
        panel_joint.joint_type == ArticulationType.REVOLUTE
        and panel_joint.axis == (0.0, 1.0, 0.0)
        and panel_limits is not None
        and panel_limits.lower == 0.0
        and panel_limits.upper is not None
        and panel_limits.upper >= 1.2,
        details=(
            f"type={panel_joint.joint_type}, axis={panel_joint.axis}, "
            f"limits={panel_limits}"
        ),
    )

    with ctx.pose({panel_joint: 0.0}):
        ctx.expect_contact(
            access_panel,
            nacelle,
            elem_a="panel_hinge_barrel",
            elem_b="hinge_knuckle_left",
            name="panel hinge barrel meets left nacelle knuckle",
        )
        ctx.expect_gap(
            nacelle,
            access_panel,
            axis="z",
            positive_elem="nacelle_shell",
            negative_elem="panel_leaf",
            max_gap=0.090,
            max_penetration=0.0,
            name="closed panel sits just beneath the nacelle skin",
        )
        ctx.expect_within(
            fan_rotor,
            nacelle,
            axes="yz",
            inner_elem="fan_rotor",
            outer_elem="nacelle_shell",
            margin=0.020,
            name="fan rotor stays inside the intake envelope",
        )
        ctx.expect_contact(
            fan_rotor,
            nacelle,
            elem_a="rotor_shaft",
            elem_b="bearing_hub",
            name="fan shaft seats against the bearing hub",
        )

    with ctx.pose({panel_joint: 1.20}):
        ctx.expect_gap(
            nacelle,
            access_panel,
            axis="z",
            positive_elem="nacelle_shell",
            negative_elem="panel_leaf",
            min_gap=0.070,
            name="opened panel drops clear of the nacelle",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
