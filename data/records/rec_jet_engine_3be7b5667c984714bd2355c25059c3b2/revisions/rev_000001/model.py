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


ENGINE_FRONT_Z = -0.86
ENGINE_REAR_Z = 0.78
ENGINE_CENTER_Z = 0.5 * (ENGINE_FRONT_Z + ENGINE_REAR_Z)
PETAL_COUNT = 12
PETAL_HINGE_RADIUS = 0.228
PETAL_HINGE_Z = ENGINE_REAR_Z


def _merge_meshes(meshes: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for mesh in meshes:
        merged.merge(mesh)
    return merged


def _radial_pattern(
    base_mesh: MeshGeometry,
    count: int,
    *,
    angle_offset: float = 0.0,
) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_mesh.copy().rotate_z(angle_offset + (index * math.tau / count)))
    return patterned


def _mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _loft_mesh(sections: list[list[tuple[float, float, float]]]) -> MeshGeometry:
    return repair_loft(section_loft(sections), repair="mesh")


def _blade_section(
    radius: float,
    sweep_y: float,
    center_z: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_t = thickness * 0.5
    return [
        (radius, sweep_y - 0.95 * half_t, center_z - 0.48 * chord),
        (radius, sweep_y + 0.22 * half_t, center_z - 0.10 * chord),
        (radius, sweep_y + 1.00 * half_t, center_z + 0.50 * chord),
        (radius, sweep_y - 0.24 * half_t, center_z + 0.14 * chord),
    ]


def _petal_section(
    z_pos: float,
    x_inner: float,
    x_outer: float,
    width: float,
    *,
    skew: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_width = 0.5 * width
    return [
        (x_inner, -half_width, z_pos),
        (x_outer, skew - 0.40 * half_width, z_pos),
        (x_outer, skew + 0.40 * half_width, z_pos),
        (x_inner, half_width, z_pos),
    ]


def _build_core_shell_mesh() -> MeshGeometry:
    outer_profile = [
        (0.360, -0.84),
        (0.402, -0.80),
        (0.422, -0.64),
        (0.425, -0.26),
        (0.410, 0.16),
        (0.378, 0.48),
        (0.332, 0.68),
        (0.252, 0.78),
    ]
    inner_profile = [
        (0.286, -0.86),
        (0.308, -0.80),
        (0.314, -0.68),
        (0.320, -0.58),
        (0.320, -0.48),
        (0.258, -0.30),
        (0.228, 0.10),
        (0.220, 0.48),
        (0.198, 0.78),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=88,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )


def _build_spinner_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, -0.22),
            (0.020, -0.21),
            (0.055, -0.17),
            (0.094, -0.11),
            (0.118, -0.03),
            (0.122, 0.03),
            (0.090, 0.08),
            (0.0, 0.06),
        ],
        segments=64,
    )


def _build_rotor_blades_mesh() -> MeshGeometry:
    blade = _loft_mesh(
        [
            _blade_section(0.090, -0.014, -0.08, 0.19, 0.024),
            _blade_section(0.152, -0.008, -0.03, 0.16, 0.020),
            _blade_section(0.214, 0.004, 0.03, 0.12, 0.015),
            _blade_section(0.282, 0.020, 0.10, 0.08, 0.010),
        ]
    )
    return _radial_pattern(blade, 14, angle_offset=math.pi / 14.0)


def _build_petal_skin_mesh(layer_offset: float, width_bias: float) -> MeshGeometry:
    return _loft_mesh(
        [
            _petal_section(0.010, layer_offset + 0.001, layer_offset + 0.005, 0.112 + width_bias),
            _petal_section(0.090, layer_offset + 0.004, layer_offset + 0.009, 0.148 + width_bias, skew=-0.003),
            _petal_section(0.200, layer_offset + 0.010, layer_offset + 0.015, 0.160 + width_bias, skew=-0.006),
            _petal_section(0.285, layer_offset + 0.016, layer_offset + 0.020, 0.094 + width_bias, skew=-0.008),
        ]
    )


def _vec_close(a: tuple[float, float, float], b: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fighter_jet_engine")

    casing = model.material("casing", rgba=(0.45, 0.47, 0.50, 1.0))
    rotor_metal = model.material("rotor_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    hub_dark = model.material("hub_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    nozzle_metal = model.material("nozzle_metal", rgba=(0.42, 0.43, 0.45, 1.0))
    heat_stain = model.material("heat_stain", rgba=(0.53, 0.47, 0.38, 1.0))

    core_shell_mesh = _mesh(_build_core_shell_mesh(), "engine_core_shell_v2")
    spinner_mesh = _mesh(_build_spinner_mesh(), "front_spinner")
    rotor_blades_mesh = _mesh(_build_rotor_blades_mesh(), "front_rotor_blades")
    petal_skin_inner = _mesh(_build_petal_skin_mesh(0.000, 0.000), "nozzle_petal_skin_inner")
    petal_skin_outer = _mesh(_build_petal_skin_mesh(0.010, 0.008), "nozzle_petal_skin_outer")

    body = model.part("engine_body")
    body.visual(
        core_shell_mesh,
        material=casing,
        name="core_shell",
    )
    body.visual(
        Cylinder(radius=0.076, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.43)),
        material=hub_dark,
        name="front_bearing_support",
    )
    body.visual(
        Box((0.62, 0.024, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.43)),
        material=hub_dark,
        name="front_spider_strut_x",
    )
    body.visual(
        Box((0.024, 0.62, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.43)),
        material=hub_dark,
        name="front_spider_strut_y",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.42, length=ENGINE_REAR_Z - ENGINE_FRONT_Z),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, ENGINE_CENTER_Z)),
    )

    rotor = model.part("front_rotor")
    rotor.visual(
        Cylinder(radius=0.082, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hub_dark,
        name="hub_body",
    )
    rotor.visual(
        Cylinder(radius=0.072, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=hub_dark,
        name="hub_bearing",
    )
    rotor.visual(
        spinner_mesh,
        material=rotor_metal,
        name="spinner",
    )
    rotor.visual(
        rotor_blades_mesh,
        material=rotor_metal,
        name="fan_blades",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.29, length=0.36),
        mass=16.0,
        origin=Origin(),
    )

    model.articulation(
        "body_to_front_rotor",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.66)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=45.0),
    )

    for index in range(PETAL_COUNT):
        petal = model.part(f"nozzle_petal_{index:02d}")
        is_outer = index % 2 == 1
        layer_offset = 0.010 if is_outer else 0.000
        root_width = 0.112 if is_outer else 0.104
        petal.visual(
            Box((0.012, root_width, 0.024)),
            origin=Origin(xyz=(layer_offset + 0.006, 0.0, 0.012)),
            material=nozzle_metal,
            name="petal_root",
        )
        petal.visual(
            petal_skin_outer if is_outer else petal_skin_inner,
            material=heat_stain if is_outer else nozzle_metal,
            name="petal_skin",
        )
        petal.inertial = Inertial.from_geometry(
            Box((0.032, 0.17, 0.30)),
            mass=1.25,
            origin=Origin(xyz=(layer_offset + 0.012, 0.0, 0.14)),
        )

        angle = index * math.tau / PETAL_COUNT
        model.articulation(
            f"body_to_nozzle_petal_{index:02d}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=petal,
            origin=Origin(
                xyz=(
                    PETAL_HINGE_RADIUS * math.cos(angle),
                    PETAL_HINGE_RADIUS * math.sin(angle),
                    PETAL_HINGE_Z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=1.5,
                lower=0.0,
                upper=0.55,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("engine_body")
    rotor = object_model.get_part("front_rotor")
    rotor_joint = object_model.get_articulation("body_to_front_rotor")

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

    ctx.check("engine_body_present", body is not None, "engine_body part is missing")
    ctx.check("front_rotor_present", rotor is not None, "front_rotor part is missing")
    ctx.check(
        "rotor_joint_is_continuous",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"expected continuous rotor joint, got {rotor_joint.articulation_type}",
    )
    ctx.check(
        "rotor_joint_axis",
        _vec_close(rotor_joint.axis, (0.0, 0.0, 1.0)),
        f"rotor joint axis is {rotor_joint.axis}",
    )
    ctx.expect_origin_distance(
        rotor,
        body,
        axes="xy",
        max_dist=1e-6,
        name="rotor_on_engine_centerline",
    )
    ctx.expect_contact(
        rotor,
        body,
        elem_a="hub_bearing",
        elem_b="front_bearing_support",
        name="rotor_bearing_contacts_core",
    )
    with ctx.pose({rotor_joint: math.pi / 2.0}):
        ctx.expect_contact(
            rotor,
            body,
            elem_a="hub_bearing",
            elem_b="front_bearing_support",
            name="rotor_bearing_stays_mounted_while_spinning",
        )

    for index in range(PETAL_COUNT):
        petal = object_model.get_part(f"nozzle_petal_{index:02d}")
        petal_joint = object_model.get_articulation(f"body_to_nozzle_petal_{index:02d}")
        limits = petal_joint.motion_limits

        ctx.check(
            f"{petal.name}_present",
            petal is not None,
            f"{petal.name} part is missing",
        )
        ctx.check(
            f"{petal_joint.name}_axis",
            _vec_close(petal_joint.axis, (0.0, 1.0, 0.0)),
            f"petal joint axis is {petal_joint.axis}",
        )
        ctx.check(
            f"{petal_joint.name}_limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower - 0.0) <= 1e-6
            and 0.45 <= limits.upper <= 0.60,
            f"unexpected petal limits: {limits}",
        )
        ctx.expect_contact(
            petal,
            body,
            elem_a="petal_root",
            elem_b="core_shell",
            name=f"{petal.name}_hinge_mount_contacts_exhaust_ring",
        )
        ctx.expect_origin_distance(
            petal,
            body,
            axes="xy",
            min_dist=0.220,
            max_dist=0.236,
            name=f"{petal.name}_mounted_on_nozzle_ring",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
