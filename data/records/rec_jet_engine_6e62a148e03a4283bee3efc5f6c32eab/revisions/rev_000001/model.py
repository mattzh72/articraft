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


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _radial_pattern(
    base_geometry: MeshGeometry,
    count: int,
    *,
    angle_offset: float = 0.0,
) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(
            base_geometry.copy().rotate_z(angle_offset + (index * math.tau / count))
        )
    return patterned


def _ring_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z_start: float,
    z_end: float,
    segments: int = 64,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_start), (outer_radius, z_end)],
        [(inner_radius, z_start), (inner_radius, z_end)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _blade_section(
    radius: float,
    tangential_bias: float,
    z_pos: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_thickness = thickness * 0.5
    return [
        (radius, tangential_bias - 0.95 * half_thickness, z_pos - 0.52 * chord),
        (radius, tangential_bias + 0.20 * half_thickness, z_pos - 0.08 * chord),
        (radius, tangential_bias + 0.95 * half_thickness, z_pos + 0.48 * chord),
        (radius, tangential_bias - 0.30 * half_thickness, z_pos + 0.14 * chord),
    ]


def _build_outer_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.450, 0.000),
            (0.462, 0.070),
            (0.438, 0.240),
            (0.432, 0.800),
            (0.430, 1.700),
            (0.420, 2.500),
            (0.392, 2.920),
            (0.330, 3.230),
            (0.260, 3.420),
        ],
        [
            (0.355, 0.000),
            (0.380, 0.070),
            (0.350, 0.240),
            (0.340, 0.800),
            (0.330, 1.700),
            (0.318, 2.460),
            (0.280, 2.900),
            (0.220, 3.220),
            (0.165, 3.420),
        ],
        segments=96,
        start_cap="round",
        end_cap="flat",
    )


def _build_intake_cone_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.000, -0.110),
            (0.020, -0.085),
            (0.055, -0.020),
            (0.100, 0.120),
            (0.132, 0.245),
            (0.112, 0.300),
            (0.000, 0.300),
        ],
        segments=72,
    )


def _build_center_shaft_mesh() -> MeshGeometry:
    shaft = CylinderGeometry(radius=0.085, height=0.760, radial_segments=48).translate(
        0.0,
        0.0,
        0.660,
    )
    rear_spool = CylinderGeometry(
        radius=0.110,
        height=0.160,
        radial_segments=48,
    ).translate(0.0, 0.0, 0.920)
    return _merge_geometries([shaft, rear_spool])


def _build_intake_struts_mesh() -> MeshGeometry:
    strut = BoxGeometry((0.032, 0.320, 0.180)).translate(0.0, 0.230, 0.690)
    return _radial_pattern(strut, 4, angle_offset=math.pi / 4.0)


def _build_rotor_hub_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.138, 0.332),
            (0.148, 0.354),
            (0.148, 0.426),
            (0.138, 0.448),
        ],
        [
            (0.088, 0.332),
            (0.090, 0.352),
            (0.090, 0.424),
            (0.088, 0.442),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _build_rotor_blades_mesh() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _blade_section(0.146, -0.018, 0.402, 0.116, 0.034),
                _blade_section(0.228, 0.002, 0.470, 0.112, 0.026),
                _blade_section(0.314, 0.018, 0.548, 0.074, 0.016),
            ]
        ),
        repair="mesh",
    )
    return _radial_pattern(blade, 14, angle_offset=math.pi / 14.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="military_turbojet")

    casing_gray = model.material("casing_gray", rgba=(0.66, 0.68, 0.72, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.24, 0.26, 0.30, 1.0))
    compressor_metal = model.material(
        "compressor_metal",
        rgba=(0.72, 0.75, 0.79, 1.0),
    )
    heat_stained = model.material("heat_stained", rgba=(0.46, 0.42, 0.38, 1.0))

    engine_body = model.part("engine_body")
    engine_body.visual(
        mesh_from_geometry(_build_outer_shell_mesh(), "outer_shell"),
        material=casing_gray,
        name="outer_shell",
    )
    engine_body.visual(
        mesh_from_geometry(_build_intake_cone_mesh(), "intake_cone"),
        material=steel_dark,
        name="intake_cone",
    )
    engine_body.visual(
        mesh_from_geometry(_build_center_shaft_mesh(), "center_shaft"),
        material=steel_dark,
        name="center_shaft",
    )
    engine_body.visual(
        mesh_from_geometry(
            _ring_shell(
                outer_radius=0.114,
                inner_radius=0.085,
                z_start=0.292,
                z_end=0.332,
            ),
            "retainer_front",
        ),
        material=compressor_metal,
        name="retainer_front",
    )
    engine_body.visual(
        mesh_from_geometry(
            _ring_shell(
                outer_radius=0.114,
                inner_radius=0.085,
                z_start=0.450,
                z_end=0.470,
            ),
            "retainer_rear",
        ),
        material=compressor_metal,
        name="retainer_rear",
    )
    engine_body.visual(
        mesh_from_geometry(_build_intake_struts_mesh(), "intake_struts"),
        material=heat_stained,
        name="intake_struts",
    )
    engine_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.462, length=3.420),
        mass=210.0,
        origin=Origin(xyz=(0.0, 0.0, 1.710)),
    )

    compressor_rotor = model.part("compressor_rotor")
    compressor_rotor.visual(
        mesh_from_geometry(_build_rotor_hub_mesh(), "rotor_hub"),
        material=compressor_metal,
        name="rotor_hub",
    )
    compressor_rotor.visual(
        mesh_from_geometry(_build_rotor_blades_mesh(), "rotor_blades"),
        material=compressor_metal,
        name="rotor_blades",
    )
    compressor_rotor.inertial = Inertial.from_geometry(
        Box((0.640, 0.640, 0.210)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
    )

    model.articulation(
        "compressor_spin",
        ArticulationType.CONTINUOUS,
        parent=engine_body,
        child=compressor_rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=450.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    engine_body = object_model.get_part("engine_body")
    compressor_rotor = object_model.get_part("compressor_rotor")
    compressor_spin = object_model.get_articulation("compressor_spin")

    intake_cone = engine_body.get_visual("intake_cone")
    retainer_front = engine_body.get_visual("retainer_front")
    retainer_rear = engine_body.get_visual("retainer_rear")
    rotor_hub = compressor_rotor.get_visual("rotor_hub")
    rotor_blades = compressor_rotor.get_visual("rotor_blades")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "rotor_joint_is_continuous",
        compressor_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="The visible compressor stage should spin continuously.",
    )
    ctx.check(
        "rotor_joint_axis_is_engine_axis",
        tuple(round(value, 6) for value in compressor_spin.axis) == (0.0, 0.0, 1.0),
        details=f"Expected rotor axis (0, 0, 1), got {compressor_spin.axis!r}.",
    )

    ctx.expect_origin_distance(
        compressor_rotor,
        engine_body,
        axes="xy",
        max_dist=0.001,
        name="rotor_stays_coaxial_with_engine_body",
    )
    ctx.expect_gap(
        compressor_rotor,
        engine_body,
        axis="z",
        positive_elem=rotor_hub,
        negative_elem=retainer_front,
        min_gap=0.0,
        max_gap=0.004,
        name="rotor_is_close_to_front_retainer",
    )
    ctx.expect_gap(
        engine_body,
        compressor_rotor,
        axis="z",
        positive_elem=retainer_rear,
        negative_elem=rotor_hub,
        min_gap=0.0,
        max_gap=0.006,
        name="rotor_is_close_to_rear_retainer",
    )
    ctx.expect_overlap(
        compressor_rotor,
        engine_body,
        axes="xy",
        elem_a=rotor_blades,
        elem_b=intake_cone,
        min_overlap=0.18,
        name="rotor_visually_wraps_the_intake_cone",
    )

    with ctx.pose({compressor_spin: math.pi / 6.0}):
        ctx.expect_gap(
            compressor_rotor,
            engine_body,
            axis="z",
            positive_elem=rotor_hub,
            negative_elem=retainer_front,
            min_gap=0.0,
            max_gap=0.004,
            name="front_capture_clearance_persists_when_spinning",
        )
        ctx.expect_gap(
            engine_body,
            compressor_rotor,
            axis="z",
            positive_elem=retainer_rear,
            negative_elem=rotor_hub,
            min_gap=0.0,
            max_gap=0.006,
            name="rear_capture_clearance_persists_when_spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
