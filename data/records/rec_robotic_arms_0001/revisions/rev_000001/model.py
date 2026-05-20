from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, rgba=rgba)
    except TypeError:
        return Material(name=name, color=rgba)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _make_link_shell(
    name: str,
    sections: list[tuple[float, float, float, float]],
    exponent: float = 2.8,
    segments: int = 56,
):
    geom = superellipse_side_loft(
        sections,
        exponents=exponent,
        segments=segments,
        cap=True,
        closed=True,
    )
    geom.rotate_z(-math.pi / 2.0)
    return _save_mesh(name, geom)


def _make_palm_shell_mesh() -> object:
    geom = superellipse_side_loft(
        [
            (0.000, -0.016, 0.014, 0.048),
            (0.014, -0.021, 0.018, 0.060),
            (0.032, -0.026, 0.022, 0.074),
            (0.054, -0.028, 0.025, 0.086),
            (0.078, -0.022, 0.024, 0.090),
            (0.100, -0.014, 0.018, 0.076),
            (0.118, -0.008, 0.013, 0.056),
        ],
        exponents=[2.0, 2.3, 2.7, 3.0, 2.8, 2.4, 2.1],
        segments=64,
        cap=True,
        closed=True,
    )
    geom.rotate_z(-math.pi / 2.0)
    geom.merge(
        superellipse_side_loft(
            [
                (0.000, -0.010, 0.012, 0.030),
                (0.016, -0.016, 0.018, 0.040),
                (0.036, -0.014, 0.020, 0.048),
                (0.052, -0.010, 0.016, 0.040),
            ],
            exponents=[2.2, 2.5, 2.7, 2.3],
            segments=40,
            cap=True,
            closed=True,
        )
        .rotate_z(-math.pi / 2.0)
        .translate(0.010, -0.031, -0.004)
    )
    geom.merge(
        superellipse_side_loft(
            [
                (0.000, -0.010, 0.010, 0.022),
                (0.018, -0.016, 0.016, 0.032),
                (0.036, -0.014, 0.016, 0.036),
                (0.050, -0.010, 0.012, 0.026),
            ],
            exponents=[2.1, 2.4, 2.5, 2.2],
            segments=36,
            cap=True,
            closed=True,
        )
        .rotate_z(-math.pi / 2.0)
        .translate(0.018, 0.031, -0.006)
    )
    geom.merge(
        superellipse_side_loft(
            [
                (0.000, -0.004, 0.007, 0.050),
                (0.012, -0.006, 0.010, 0.076),
                (0.028, -0.005, 0.009, 0.088),
            ],
            exponents=[2.0, 2.3, 2.0],
            segments=30,
            cap=True,
            closed=True,
        )
        .rotate_z(-math.pi / 2.0)
        .translate(0.092, 0.000, 0.014)
    )
    geom.merge(
        superellipse_side_loft(
            [
                (0.000, -0.006, 0.010, 0.028),
                (0.014, -0.010, 0.014, 0.040),
                (0.030, -0.008, 0.012, 0.034),
            ],
            exponents=2.2,
            segments=28,
            cap=True,
            closed=True,
        )
        .rotate_z(-math.pi / 2.0)
        .translate(0.048, 0.000, -0.010)
    )
    geom.merge(SphereGeometry(0.013).translate(0.095, -0.035, 0.014))
    geom.merge(SphereGeometry(0.0120).translate(0.101, -0.012, 0.016))
    geom.merge(SphereGeometry(0.0115).translate(0.100, 0.013, 0.014))
    geom.merge(SphereGeometry(0.0105).translate(0.091, 0.036, 0.010))
    geom.merge(SphereGeometry(0.012).translate(0.044, -0.029, 0.012))
    geom.merge(
        CylinderGeometry(0.0068, 0.040, radial_segments=20)
        .rotate_y(math.pi / 2.0)
        .translate(0.050, -0.020, 0.018)
    )
    geom.merge(
        CylinderGeometry(0.0058, 0.036, radial_segments=20)
        .rotate_y(math.pi / 2.0)
        .translate(0.056, 0.012, 0.018)
    )
    return _save_mesh("robotic_arm_hand_shell.obj", geom)


def _make_phalanx_mesh(
    name: str,
    *,
    length: float,
    width: float,
    thickness: float,
    boss_radius: float,
    boss_span: float,
    tip_pad: float = 0.0,
) -> object:
    geom = superellipse_side_loft(
        [
            (0.000, -thickness * 0.52, thickness * 0.42, width * 0.96),
            (length * 0.12, -thickness * 0.50, thickness * 0.54, width * 0.92),
            (length * 0.30, -thickness * 0.40, thickness * 0.52, width * 0.84),
            (length * 0.56, -thickness * 0.26, thickness * 0.44, width * 0.66),
            (length * 0.82, -thickness * 0.10, thickness * 0.32, width * 0.42),
            (length, 0.0, thickness * 0.22, width * 0.24),
        ],
        exponents=[2.0, 2.3, 2.8, 3.0, 2.5, 2.1],
        segments=44,
        cap=True,
        closed=True,
    )
    geom.rotate_z(-math.pi / 2.0)
    geom.merge(
        CylinderGeometry(boss_radius * 1.02, boss_span, radial_segments=28).rotate_x(-math.pi / 2.0)
    )
    geom.merge(SphereGeometry(boss_radius * 1.08).translate(0.0, 0.0, 0.0))
    geom.merge(SphereGeometry(thickness * 0.18).translate(length * 0.18, 0.0, thickness * 0.16))
    geom.merge(SphereGeometry(thickness * 0.22).translate(length * 0.56, 0.0, thickness * 0.10))
    geom.merge(SphereGeometry(thickness * 0.18).translate(length * 0.90, 0.0, 0.0))
    geom.merge(
        CylinderGeometry(thickness * 0.08, max(length * 0.28, 0.010), radial_segments=16)
        .rotate_y(math.pi / 2.0)
        .translate(length * 0.32, width * 0.12, thickness * 0.22)
    )
    geom.merge(
        CylinderGeometry(thickness * 0.08, max(length * 0.28, 0.010), radial_segments=16)
        .rotate_y(math.pi / 2.0)
        .translate(length * 0.32, -width * 0.12, thickness * 0.22)
    )
    geom.merge(
        superellipse_side_loft(
            [
                (0.000, -thickness * 0.18, thickness * 0.08, width * 0.34),
                (max(length * 0.18, 0.008), -thickness * 0.28, thickness * 0.10, width * 0.46),
                (max(length * 0.34, 0.014), -thickness * 0.22, thickness * 0.08, width * 0.36),
            ],
            exponents=[1.9, 2.1, 1.9],
            segments=24,
            cap=True,
            closed=True,
        )
        .rotate_z(-math.pi / 2.0)
        .translate(length * 0.58, 0.0, -thickness * 0.18)
    )
    if tip_pad > 0.0:
        geom.merge(
            superellipse_side_loft(
                [
                    (0.000, -thickness * 0.20, thickness * 0.10, width * 0.30),
                    (tip_pad * 0.48, -thickness * 0.28, thickness * 0.12, width * 0.38),
                    (tip_pad, -thickness * 0.16, thickness * 0.08, width * 0.26),
                ],
                exponents=[1.8, 2.0, 1.8],
                segments=22,
                cap=True,
                closed=True,
            )
            .rotate_z(-math.pi / 2.0)
            .translate(length, 0.0, -thickness * 0.14)
        )
        geom.merge(
            SphereGeometry(thickness * 0.16).translate(
                length + tip_pad * 0.92, 0.0, -thickness * 0.08
            )
        )
    return _save_mesh(name, geom)


def _add_phalanx_part(
    part,
    *,
    mesh,
    shell_material: Material,
    joint_material: Material,
    pad_material: Material,
    length: float,
    width: float,
    thickness: float,
    boss_radius: float,
    boss_span: float,
    mass: float,
    tip_pad: float = 0.0,
) -> None:
    part.visual(mesh, material=shell_material, name="shell")
    part.visual(
        Cylinder(radius=boss_radius * 0.92, length=boss_span * 1.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_material,
        name="hinge_spool",
    )
    if tip_pad > 0.0:
        part.visual(
            Cylinder(radius=thickness * 0.16, length=tip_pad * 0.92),
            origin=Origin(
                xyz=(length + tip_pad * 0.46, 0.0, -thickness * 0.12),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=pad_material,
            name="pad_core",
        )
        part.visual(
            Sphere(radius=thickness * 0.14),
            origin=Origin(xyz=(length + tip_pad * 0.94, 0.0, -thickness * 0.10)),
            material=pad_material,
            name="pad_tip",
        )
    part.inertial = Inertial.from_geometry(
        Box((length + tip_pad + boss_radius * 0.8, width, thickness)),
        mass=mass,
        origin=Origin(xyz=(length * 0.46, 0.0, 0.0)),
    )


def _add_digit_chain(
    model: ArticulatedObject,
    *,
    parent: str,
    part_names: tuple[str, str, str],
    joint_names: tuple[str, str, str],
    base_xyz: tuple[float, float, float],
    base_rpy: tuple[float, float, float],
    lengths: tuple[float, float, float],
    widths: tuple[float, float, float],
    thicknesses: tuple[float, float, float],
    boss_radii: tuple[float, float, float],
    boss_spans: tuple[float, float, float],
    upper_limits: tuple[float, float, float],
    masses: tuple[float, float, float],
    shell_material: Material,
    joint_material: Material,
    pad_material: Material,
) -> None:
    for idx, name in enumerate(part_names):
        part = model.part(name)
        tip_pad = 0.010 if idx == 2 else 0.0
        mesh = _make_phalanx_mesh(
            f"{name}.obj",
            length=lengths[idx],
            width=widths[idx],
            thickness=thicknesses[idx],
            boss_radius=boss_radii[idx],
            boss_span=boss_spans[idx],
            tip_pad=tip_pad,
        )
        _add_phalanx_part(
            part,
            mesh=mesh,
            shell_material=shell_material,
            joint_material=joint_material,
            pad_material=pad_material,
            length=lengths[idx],
            width=widths[idx],
            thickness=thicknesses[idx],
            boss_radius=boss_radii[idx],
            boss_span=boss_spans[idx],
            mass=masses[idx],
            tip_pad=tip_pad,
        )

    parents = (parent, part_names[0], part_names[1])
    origins = (
        Origin(xyz=base_xyz, rpy=base_rpy),
        Origin(xyz=(lengths[0], 0.0, 0.0)),
        Origin(xyz=(lengths[1], 0.0, 0.0)),
    )
    efforts = (12.0, 8.0, 5.0)
    velocities = (4.6, 5.0, 5.4)
    for idx, child_name in enumerate(part_names):
        model.articulation(
            joint_names[idx],
            ArticulationType.REVOLUTE,
            parent=parents[idx],
            child=child_name,
            origin=origins[idx],
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=efforts[idx],
                velocity=velocities[idx],
                lower=0.0,
                upper=upper_limits[idx],
            ),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_arm", assets=ASSETS)

    materials = {
        "ceramic": _material("ceramic", (0.87, 0.88, 0.90, 1.0)),
        "graphite": _material("graphite", (0.17, 0.18, 0.20, 1.0)),
        "titanium": _material("titanium", (0.66, 0.69, 0.72, 1.0)),
        "copper": _material("copper", (0.75, 0.44, 0.25, 1.0)),
        "elastomer": _material("elastomer", (0.06, 0.06, 0.07, 1.0)),
    }
    model.materials.extend(list(materials.values()))

    base_shell = _save_mesh(
        "robotic_arm_base_shell.obj",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.164, 0.0),
                (0.160, 0.018),
                (0.144, 0.050),
                (0.124, 0.084),
                (0.114, 0.108),
                (0.0, 0.108),
            ],
            segments=64,
        ),
    )
    upper_arm_shell = _make_link_shell(
        "robotic_arm_upper_arm_shell.obj",
        [
            (0.000, -0.054, 0.060, 0.140),
            (0.070, -0.060, 0.072, 0.152),
            (0.180, -0.046, 0.064, 0.128),
            (0.280, -0.033, 0.048, 0.094),
            (0.330, -0.026, 0.040, 0.080),
        ],
        exponent=2.65,
    )
    forearm_shell = _make_link_shell(
        "robotic_arm_forearm_shell.obj",
        [
            (0.000, -0.046, 0.052, 0.114),
            (0.060, -0.050, 0.060, 0.122),
            (0.150, -0.040, 0.056, 0.108),
            (0.240, -0.030, 0.044, 0.084),
            (0.290, -0.024, 0.034, 0.068),
        ],
        exponent=2.60,
    )
    wrist_shell = _make_link_shell(
        "robotic_arm_wrist_shell.obj",
        [
            (0.000, -0.030, 0.030, 0.072),
            (0.030, -0.034, 0.042, 0.084),
            (0.070, -0.025, 0.036, 0.064),
            (0.100, -0.020, 0.028, 0.052),
        ],
        exponent=2.45,
        segments=48,
    )
    palm_shell = _make_palm_shell_mesh()

    base = model.part("base")
    base.visual(base_shell, material=materials["ceramic"], name="base_shell")
    base.visual(
        Cylinder(radius=0.104, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=materials["graphite"],
        name="slew_ring",
    )
    base.visual(
        Box((0.072, 0.026, 0.046)),
        origin=Origin(xyz=(-0.090, 0.0, 0.058)),
        material=materials["graphite"],
        name="service_panel",
    )
    base.visual(
        Box((0.034, 0.124, 0.006)),
        origin=Origin(xyz=(0.074, 0.0, 0.024)),
        material=materials["copper"],
        name="base_accent_band",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.165, length=0.108),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.050, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=materials["ceramic"],
        name="turret_column",
    )
    turret.visual(
        Cylinder(radius=0.062, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=materials["graphite"],
        name="slew_drive_housing",
    )
    turret.visual(
        Box((0.082, 0.018, 0.154)),
        origin=Origin(xyz=(0.0, -0.055, 0.116)),
        material=materials["ceramic"],
        name="left_yoke_cheek",
    )
    turret.visual(
        Box((0.082, 0.018, 0.154)),
        origin=Origin(xyz=(0.0, 0.055, 0.116)),
        material=materials["ceramic"],
        name="right_yoke_cheek",
    )
    turret.visual(
        Box((0.040, 0.128, 0.044)),
        origin=Origin(xyz=(-0.022, 0.0, 0.192)),
        material=materials["ceramic"],
        name="yoke_bridge",
    )
    turret.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.0, -0.064, 0.192), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["titanium"],
        name="left_shoulder_cap",
    )
    turret.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.0, 0.064, 0.192), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["titanium"],
        name="right_shoulder_cap",
    )
    turret.visual(
        Box((0.012, 0.076, 0.094)),
        origin=Origin(xyz=(0.044, 0.0, 0.092)),
        material=materials["copper"],
        name="turret_front_accent",
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.144, 0.130, 0.224)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
    )
    upper_arm = model.part("upper_arm")
    upper_arm.visual(upper_arm_shell, material=materials["ceramic"], name="upper_arm_shell")
    upper_arm.visual(
        Cylinder(radius=0.034, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["titanium"],
        name="shoulder_hub",
    )
    upper_arm.visual(
        Cylinder(radius=0.030, length=0.072),
        origin=Origin(xyz=(0.330, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["titanium"],
        name="elbow_cap",
    )
    upper_arm.visual(
        Box((0.164, 0.022, 0.018)),
        origin=Origin(xyz=(0.164, 0.0, 0.044)),
        material=materials["graphite"],
        name="extensor_spine",
    )
    upper_arm.visual(
        Box((0.126, 0.008, 0.030)),
        origin=Origin(xyz=(0.134, -0.043, 0.014)),
        material=materials["copper"],
        name="upper_arm_side_trace",
    )
    upper_arm.visual(
        Box((0.096, 0.014, 0.020)),
        origin=Origin(xyz=(0.190, 0.038, -0.020)),
        material=materials["graphite"],
        name="bicep_service_channel",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.360, 0.150, 0.118)),
        mass=6.2,
        origin=Origin(xyz=(0.170, 0.0, 0.004)),
    )
    forearm = model.part("forearm")
    forearm.visual(forearm_shell, material=materials["ceramic"], name="forearm_shell")
    forearm.visual(
        Cylinder(radius=0.030, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["titanium"],
        name="elbow_hub",
    )
    forearm.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=Origin(xyz=(0.290, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["titanium"],
        name="wrist_cap",
    )
    forearm.visual(
        Box((0.122, 0.020, 0.016)),
        origin=Origin(xyz=(0.148, 0.0, 0.034)),
        material=materials["graphite"],
        name="forearm_tendon_channel",
    )
    forearm.visual(
        Box((0.102, 0.008, 0.026)),
        origin=Origin(xyz=(0.128, 0.038, 0.008)),
        material=materials["copper"],
        name="forearm_side_trace",
    )
    forearm.visual(
        Box((0.074, 0.018, 0.014)),
        origin=Origin(xyz=(0.214, -0.030, -0.020)),
        material=materials["graphite"],
        name="ulnar_service_band",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.312, 0.122, 0.100)),
        mass=4.8,
        origin=Origin(xyz=(0.145, 0.0, 0.004)),
    )
    wrist_pitch = model.part("wrist_pitch")
    wrist_pitch.visual(wrist_shell, material=materials["ceramic"], name="wrist_shell")
    wrist_pitch.visual(
        Cylinder(radius=0.026, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["titanium"],
        name="wrist_pitch_hub",
    )
    wrist_pitch.visual(
        Cylinder(radius=0.023, length=0.016),
        origin=Origin(xyz=(0.094, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["titanium"],
        name="wrist_roll_flange",
    )
    wrist_pitch.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["graphite"],
        name="wrist_mid_band",
    )
    wrist_pitch.inertial = Inertial.from_geometry(
        Box((0.112, 0.084, 0.082)),
        mass=2.0,
        origin=Origin(xyz=(0.050, 0.0, 0.006)),
    )
    wrist_roll = model.part("wrist_roll")
    wrist_roll.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(0.000, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["titanium"],
        name="roll_spindle",
    )
    wrist_roll.visual(
        Cylinder(radius=0.025, length=0.036),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["graphite"],
        name="roll_rear_collar",
    )
    wrist_roll.visual(palm_shell, material=materials["ceramic"], name="palm_shell")
    wrist_roll.visual(
        Cylinder(radius=0.0066, length=0.090),
        origin=Origin(xyz=(0.095, 0.0, 0.021), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["titanium"],
        name="knuckle_bridge",
    )
    wrist_roll.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(0.055, -0.023, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["graphite"],
        name="radial_tendon_cover",
    )
    wrist_roll.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(0.059, 0.004, 0.019), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["graphite"],
        name="center_tendon_cover",
    )
    wrist_roll.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.024, -0.040, -0.009)),
        material=materials["copper"],
        name="thumb_saddle",
    )
    wrist_roll.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(0.027, -0.040, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["copper"],
        name="thumb_saddle_spine",
    )
    wrist_roll.inertial = Inertial.from_geometry(
        Box((0.126, 0.100, 0.064)),
        mass=1.6,
        origin=Origin(xyz=(0.050, 0.0, 0.000)),
    )
    _add_digit_chain(
        model,
        parent="wrist_roll",
        part_names=("index_proximal", "index_middle", "index_distal"),
        joint_names=("index_mcp_pitch", "index_pip_pitch", "index_dip_pitch"),
        base_xyz=(0.098, -0.034, 0.010),
        base_rpy=(0.0, 0.0, -0.06),
        lengths=(0.050, 0.031, 0.024),
        widths=(0.0165, 0.0150, 0.0135),
        thicknesses=(0.021, 0.018, 0.015),
        boss_radii=(0.0090, 0.0078, 0.0068),
        boss_spans=(0.022, 0.019, 0.016),
        upper_limits=(1.20, 1.38, 1.05),
        masses=(0.11, 0.08, 0.05),
        shell_material=materials["ceramic"],
        joint_material=materials["titanium"],
        pad_material=materials["elastomer"],
    )
    _add_digit_chain(
        model,
        parent="wrist_roll",
        part_names=("middle_proximal", "middle_middle", "middle_distal"),
        joint_names=("middle_mcp_pitch", "middle_pip_pitch", "middle_dip_pitch"),
        base_xyz=(0.104, -0.010, 0.012),
        base_rpy=(0.0, 0.0, -0.01),
        lengths=(0.056, 0.036, 0.028),
        widths=(0.0180, 0.0165, 0.0145),
        thicknesses=(0.022, 0.019, 0.016),
        boss_radii=(0.0094, 0.0082, 0.0070),
        boss_spans=(0.024, 0.020, 0.017),
        upper_limits=(1.24, 1.42, 1.08),
        masses=(0.13, 0.09, 0.06),
        shell_material=materials["ceramic"],
        joint_material=materials["titanium"],
        pad_material=materials["elastomer"],
    )
    _add_digit_chain(
        model,
        parent="wrist_roll",
        part_names=("ring_proximal", "ring_middle", "ring_distal"),
        joint_names=("ring_mcp_pitch", "ring_pip_pitch", "ring_dip_pitch"),
        base_xyz=(0.102, 0.016, 0.010),
        base_rpy=(0.0, 0.0, 0.03),
        lengths=(0.052, 0.034, 0.026),
        widths=(0.0170, 0.0155, 0.0140),
        thicknesses=(0.021, 0.0185, 0.0155),
        boss_radii=(0.0090, 0.0079, 0.0068),
        boss_spans=(0.023, 0.019, 0.016),
        upper_limits=(1.26, 1.40, 1.06),
        masses=(0.12, 0.085, 0.055),
        shell_material=materials["ceramic"],
        joint_material=materials["titanium"],
        pad_material=materials["elastomer"],
    )
    _add_digit_chain(
        model,
        parent="wrist_roll",
        part_names=("little_proximal", "little_middle", "little_distal"),
        joint_names=("little_mcp_pitch", "little_pip_pitch", "little_dip_pitch"),
        base_xyz=(0.094, 0.040, 0.006),
        base_rpy=(0.0, 0.0, 0.10),
        lengths=(0.044, 0.028, 0.022),
        widths=(0.0145, 0.0130, 0.0118),
        thicknesses=(0.018, 0.0155, 0.0135),
        boss_radii=(0.0078, 0.0068, 0.0060),
        boss_spans=(0.019, 0.016, 0.014),
        upper_limits=(1.30, 1.42, 1.10),
        masses=(0.08, 0.055, 0.038),
        shell_material=materials["ceramic"],
        joint_material=materials["titanium"],
        pad_material=materials["elastomer"],
    )
    _add_digit_chain(
        model,
        parent="wrist_roll",
        part_names=("thumb_metacarpal", "thumb_proximal", "thumb_distal"),
        joint_names=("thumb_cmc_pitch", "thumb_mcp_pitch", "thumb_ip_pitch"),
        base_xyz=(0.028, -0.044, -0.010),
        base_rpy=(0.0, -0.28, -0.92),
        lengths=(0.036, 0.028, 0.022),
        widths=(0.018, 0.016, 0.014),
        thicknesses=(0.020, 0.017, 0.014),
        boss_radii=(0.0088, 0.0076, 0.0064),
        boss_spans=(0.021, 0.018, 0.015),
        upper_limits=(1.05, 1.18, 1.00),
        masses=(0.09, 0.065, 0.045),
        shell_material=materials["ceramic"],
        joint_material=materials["titanium"],
        pad_material=materials["elastomer"],
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent="base",
        child="turret",
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.8,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent="turret",
        child="upper_arm",
        origin=Origin(xyz=(0.0, 0.0, 0.192)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=1.6,
            lower=-0.25,
            upper=1.25,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent="upper_arm",
        child="forearm",
        origin=Origin(xyz=(0.330, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.9,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "wrist_pitch_joint",
        ArticulationType.REVOLUTE,
        parent="forearm",
        child="wrist_pitch",
        origin=Origin(xyz=(0.290, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=2.5,
            lower=-1.5,
            upper=1.5,
        ),
    )
    model.articulation(
        "wrist_roll_joint",
        ArticulationType.REVOLUTE,
        parent="wrist_pitch",
        child="wrist_roll",
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=3.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.012)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
    )

    ctx.expect_aabb_overlap("turret", "base", axes="xy", min_overlap=0.120)
    ctx.expect_origin_distance("turret", "base", axes="xy", max_dist=0.020)
    ctx.expect_aabb_overlap("upper_arm", "turret", axes="xy", min_overlap=0.050)
    ctx.expect_aabb_overlap("forearm", "upper_arm", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("wrist_pitch", "forearm", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_overlap("wrist_roll", "wrist_pitch", axes="xy", min_overlap=0.012)

    ctx.expect_joint_motion_axis(
        "shoulder_joint",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.050,
    )
    ctx.expect_joint_motion_axis(
        "elbow_joint",
        "forearm",
        world_axis="z",
        direction="positive",
        min_delta=0.040,
    )
    ctx.expect_joint_motion_axis(
        "wrist_pitch_joint",
        "wrist_pitch",
        world_axis="z",
        direction="positive",
        min_delta=0.012,
    )
    ctx.expect_joint_motion_axis(
        "middle_mcp_pitch",
        "middle_proximal",
        world_axis="z",
        direction="negative",
        min_delta=0.008,
    )

    rest_wrist = ctx.part_world_position("wrist_roll")
    middle_rest = ctx.part_world_position("middle_distal")
    index_rest = ctx.part_world_position("index_distal")
    ring_rest = ctx.part_world_position("ring_distal")
    little_rest = ctx.part_world_position("little_distal")
    thumb_rest = ctx.part_world_position("thumb_distal")

    _require(
        rest_wrist[0] > 0.68, "rest pose should keep the hand well forward of the shoulder stack"
    )
    _require(
        abs(rest_wrist[1]) < 0.02,
        "rest pose hand should remain centered laterally before yaw rotation",
    )
    _require(rest_wrist[2] > 0.28, "rest pose hand should sit above the base deck")
    _require(
        middle_rest[0] > rest_wrist[0] + 0.08,
        "middle finger chain should extend well beyond the palm",
    )
    _require(
        index_rest[1] < middle_rest[1] - 0.012,
        "index finger should sit on the radial side of the hand",
    )
    _require(
        little_rest[1] > ring_rest[1] + 0.010,
        "little finger should sit on the ulnar edge of the hand",
    )
    _require(
        thumb_rest[1] < index_rest[1] - 0.008,
        "thumb should mount outside the index finger on the radial side",
    )

    with ctx.pose(base_yaw=math.pi / 2.0):
        side_wrist = ctx.part_world_position("wrist_roll")
        _require(
            abs(side_wrist[0]) < 0.08, "base yaw should swing the hand to the side of the pedestal"
        )
        _require(side_wrist[1] > 0.66, "base yaw should move the hand deep into +Y")
        _require(
            abs(side_wrist[2] - rest_wrist[2]) < 0.03, "pure base yaw should preserve hand height"
        )

    with ctx.pose(shoulder_joint=1.20, elbow_joint=0.0, wrist_pitch_joint=0.0):
        raised_wrist = ctx.part_world_position("wrist_roll")
        _require(
            raised_wrist[2] > rest_wrist[2] + 0.30,
            "raised configuration should lift the hand clearly above rest",
        )
        _require(
            raised_wrist[0] < 0.36,
            "raised pose should pull the hand back toward the shoulder column",
        )
        _require(
            abs(raised_wrist[1]) < 0.05,
            "raising without yaw should keep the hand near the center plane",
        )

    with ctx.pose(shoulder_joint=-0.20, elbow_joint=0.0, wrist_pitch_joint=-0.35):
        lowered_wrist = ctx.part_world_position("wrist_roll")
        _require(
            lowered_wrist[0] > 0.62, "lowered reach should still project the hand well forward"
        )
        _require(
            lowered_wrist[2] < rest_wrist[2] - 0.10,
            "negative shoulder pitch should noticeably lower the hand",
        )

    with ctx.pose(
        middle_mcp_pitch=1.00,
        middle_pip_pitch=1.18,
        middle_dip_pitch=0.92,
        index_mcp_pitch=0.92,
        index_pip_pitch=1.04,
        index_dip_pitch=0.82,
    ):
        curled_middle = ctx.part_world_position("middle_distal")
        curled_index = ctx.part_world_position("index_distal")
        _require(
            curled_middle[2] < middle_rest[2] - 0.050,
            "middle finger should curl downward through the palm plane",
        )
        _require(
            curled_index[2] < index_rest[2] - 0.040,
            "index finger should also curl downward under flexion",
        )
        _require(
            curled_middle[0] < middle_rest[0] - 0.030,
            "curled fingers should pull their distal segments back",
        )

    with ctx.pose(thumb_cmc_pitch=0.82, thumb_mcp_pitch=0.92, thumb_ip_pitch=0.78):
        thumb_flex = ctx.part_world_position("thumb_distal")
        _require(
            thumb_flex[2] < thumb_rest[2] - 0.020, "thumb should fold toward the palm under flexion"
        )
        _require(
            thumb_flex[0] < thumb_rest[0] - 0.010,
            "thumb flexion should retract the distal thumb segment slightly",
        )

    with ctx.pose(wrist_roll_joint=math.pi / 2.0):
        rolled_index = ctx.part_world_position("index_distal")
        rolled_little = ctx.part_world_position("little_distal")
        _require(
            abs(rolled_index[1] - rolled_little[1]) < 0.030,
            "quarter-turn wrist roll should rotate finger spread mostly out of the world-Y axis",
        )
        _require(
            rolled_little[2] - rolled_index[2] > 0.040,
            "quarter-turn wrist roll should stack the finger fan vertically in world Z",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
