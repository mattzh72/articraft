from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
IDENTITY = Origin(xyz=(0.0, 0.0, 0.0))

FAN_JOINT_X = 0.505

OUTER_SHELL_SECTIONS = [
    (0.00, 0.70, 0.82),
    (0.10, 0.74, 0.84),
    (0.34, 0.76, 0.83),
    (0.95, 0.68, 0.89),
    (1.55, 0.54, 0.82),
    (2.30, 0.36, 0.63),
    (2.95, 0.29, 0.55),
]


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    material = None
    for kwargs in (
        {"name": name, "color": rgba},
        {"name": name, "rgba": rgba},
        {"name": name},
    ):
        try:
            material = Material(**kwargs)
            break
        except TypeError:
            continue
    if material is None:
        material = Material(name=name)
    for attr in ("color", "rgba"):
        if hasattr(material, attr):
            try:
                setattr(material, attr, rgba)
            except Exception:
                pass
    return material


def _combine_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    parts = [geom for geom in geometries if geom is not None]
    if not parts:
        return MeshGeometry()
    combined = parts[0].clone()
    for geom in parts[1:]:
        combined.merge(geom)
    return combined


def _annular_shell_geometry(
    sections: list[tuple[float, float, float]],
    radial_segments: int = 80,
    phase: float = 0.0,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer_rings: list[list[int]] = []
    inner_rings: list[list[int]] = []
    angles = [phase + 2.0 * math.pi * i / radial_segments for i in range(radial_segments)]

    for x, inner_radius, outer_radius in sections:
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for angle in angles:
            outer_ring.append(
                geom.add_vertex(x, outer_radius * math.cos(angle), outer_radius * math.sin(angle))
            )
        for angle in angles:
            inner_ring.append(
                geom.add_vertex(x, inner_radius * math.cos(angle), inner_radius * math.sin(angle))
            )
        outer_rings.append(outer_ring)
        inner_rings.append(inner_ring)

    def bridge(ring_a: list[int], ring_b: list[int], outward: bool) -> None:
        for i in range(radial_segments):
            j = (i + 1) % radial_segments
            a0 = ring_a[i]
            a1 = ring_a[j]
            b0 = ring_b[i]
            b1 = ring_b[j]
            if outward:
                geom.add_face(a0, a1, b0)
                geom.add_face(a1, b1, b0)
            else:
                geom.add_face(a0, b0, a1)
                geom.add_face(a1, b0, b1)

    for idx in range(len(sections) - 1):
        bridge(outer_rings[idx], outer_rings[idx + 1], outward=True)
        bridge(inner_rings[idx], inner_rings[idx + 1], outward=False)

    front_outer = outer_rings[0]
    front_inner = inner_rings[0]
    rear_outer = outer_rings[-1]
    rear_inner = inner_rings[-1]

    for i in range(radial_segments):
        j = (i + 1) % radial_segments
        geom.add_face(front_outer[i], front_inner[i], front_outer[j])
        geom.add_face(front_outer[j], front_inner[i], front_inner[j])
        geom.add_face(rear_outer[i], rear_outer[j], rear_inner[i])
        geom.add_face(rear_outer[j], rear_inner[j], rear_inner[i])

    return geom


def _lathe_along_x(profile_xr: list[tuple[float, float]], segments: int = 80) -> MeshGeometry:
    geom = LatheGeometry([(radius, x) for x, radius in profile_xr], segments=segments)
    geom.rotate_y(math.pi / 2.0)
    return geom


def _airfoil_loop(
    chord: float,
    thickness: float,
    camber: float = 0.0,
    samples_per_side: int = 8,
) -> list[tuple[float, float]]:
    xs = [
        (1.0 - math.cos(math.pi * i / (samples_per_side - 1))) * 0.5
        for i in range(samples_per_side)
    ]
    half_thickness = max(thickness * 0.5, 1e-4)
    upper: list[tuple[float, float]] = []
    lower: list[tuple[float, float]] = []
    for x_frac in xs:
        yt = (
            5.0
            * half_thickness
            * (
                0.2969 * math.sqrt(max(x_frac, 1e-6))
                - 0.1260 * x_frac
                - 0.3516 * x_frac**2
                + 0.2843 * x_frac**3
                - 0.1015 * x_frac**4
            )
        )
        yc = camber * (1.0 - 4.0 * (x_frac - 0.5) ** 2)
        x = (x_frac - 0.25) * chord
        upper.append((x, yc + yt))
        lower.append((x, yc - yt))
    return upper + list(reversed(lower[1:-1]))


def _blade_geometry(
    stations: list[dict[str, float]],
    samples_per_side: int = 8,
) -> MeshGeometry:
    profiles: list[list[tuple[float, float, float]]] = []
    for station in stations:
        loop = _airfoil_loop(
            chord=station["chord"],
            thickness=station["thickness"],
            camber=station.get("camber", 0.0),
            samples_per_side=samples_per_side,
        )
        twist = math.radians(station.get("twist_deg", 0.0))
        cos_t = math.cos(twist)
        sin_t = math.sin(twist)
        x_offset = station["x_offset"]
        radius = station["radius"]
        profile: list[tuple[float, float, float]] = []
        for x_local, y_local in loop:
            x_world = x_local * cos_t - y_local * sin_t + x_offset
            y_world = x_local * sin_t + y_local * cos_t
            profile.append((x_world, y_world, radius))
        profiles.append(profile)
    return LoftGeometry(profiles, cap=True, closed=True)


def _radial_array(base_geometry: MeshGeometry, count: int, phase: float = 0.0) -> MeshGeometry:
    copies = []
    for idx in range(count):
        geom = base_geometry.clone()
        geom.rotate_x(phase + 2.0 * math.pi * idx / count)
        copies.append(geom)
    return _combine_geometries(*copies)


def _register_materials(model: ArticulatedObject) -> dict[str, Material]:
    materials = {
        "paint": _make_material("nacelle_paint", (0.95, 0.96, 0.98, 1.0)),
        "lip": _make_material("intake_lip_titanium", (0.79, 0.81, 0.84, 1.0)),
        "inner": _make_material("duct_primer_gray", (0.24, 0.25, 0.27, 1.0)),
        "steel": _make_material("compressor_steel", (0.58, 0.61, 0.64, 1.0)),
        "hot": _make_material("burnt_nozzle_alloy", (0.42, 0.38, 0.34, 1.0)),
        "spinner": _make_material("spinner_alloy", (0.72, 0.74, 0.77, 1.0)),
        "blade": _make_material("fan_blade_composite", (0.18, 0.19, 0.22, 1.0)),
    }
    try:
        model.materials.extend(materials.values())
    except Exception:
        pass
    return materials


def _expect(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def _as_triplet(value) -> tuple[float, float, float]:
    if isinstance(value, (list, tuple)) and len(value) == 3:
        return float(value[0]), float(value[1]), float(value[2])
    raise TypeError(f"Cannot interpret {value!r} as a 3-vector")


def _aabb_bounds(aabb) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if isinstance(aabb, dict):
        for min_key, max_key in (("min", "max"), ("minimum", "maximum"), ("lower", "upper")):
            if min_key in aabb and max_key in aabb:
                return _as_triplet(aabb[min_key]), _as_triplet(aabb[max_key])
    if isinstance(aabb, (list, tuple)) and len(aabb) == 2:
        return _as_triplet(aabb[0]), _as_triplet(aabb[1])

    for min_name, max_name in (("min", "max"), ("minimum", "maximum"), ("lower", "upper")):
        if hasattr(aabb, min_name) and hasattr(aabb, max_name):
            return _as_triplet(getattr(aabb, min_name)), _as_triplet(getattr(aabb, max_name))

    for min_names, max_names in (
        (("min_x", "min_y", "min_z"), ("max_x", "max_y", "max_z")),
        (("xmin", "ymin", "zmin"), ("xmax", "ymax", "zmax")),
        (("x_min", "y_min", "z_min"), ("x_max", "y_max", "z_max")),
    ):
        if all(hasattr(aabb, name) for name in min_names + max_names):
            mins = tuple(float(getattr(aabb, name)) for name in min_names)
            maxs = tuple(float(getattr(aabb, name)) for name in max_names)
            return mins, maxs

    raise TypeError(f"Unsupported AABB representation: {aabb!r}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="realistic_turbofan_engine", assets=ASSETS)
    materials = _register_materials(model)

    nacelle = model.part("nacelle")

    outer_shell = _annular_shell_geometry(OUTER_SHELL_SECTIONS, radial_segments=96)
    intake_lip = _annular_shell_geometry(
        [
            (0.00, 0.70, 0.82),
            (0.05, 0.725, 0.835),
            (0.16, 0.752, 0.835),
        ],
        radial_segments=96,
    )
    intake_liner = _annular_shell_geometry(
        [
            (0.02, 0.700, 0.712),
            (0.18, 0.748, 0.760),
            (0.55, 0.735, 0.747),
            (1.25, 0.595, 0.607),
        ],
        radial_segments=88,
    )
    hot_liner = _annular_shell_geometry(
        [
            (1.55, 0.540, 0.552),
            (2.30, 0.360, 0.372),
            (2.95, 0.290, 0.302),
        ],
        radial_segments=88,
    )
    exhaust_band = _annular_shell_geometry(
        [
            (2.30, 0.615, 0.637),
            (2.68, 0.570, 0.592),
            (2.95, 0.545, 0.567),
        ],
        radial_segments=88,
    )
    core_body = _lathe_along_x(
        [
            (0.508, 0.0),
            (0.60, 0.10),
            (0.78, 0.23),
            (1.22, 0.27),
            (1.85, 0.24),
            (2.35, 0.18),
            (2.95, 0.09),
            (3.06, 0.0),
        ],
        segments=88,
    )
    ogv_blade = _blade_geometry(
        [
            {
                "radius": 0.215,
                "chord": 0.18,
                "thickness": 0.020,
                "camber": 0.010,
                "x_offset": 0.60,
                "twist_deg": -10.0,
            },
            {
                "radius": 0.470,
                "chord": 0.15,
                "thickness": 0.015,
                "camber": 0.008,
                "x_offset": 0.66,
                "twist_deg": 1.0,
            },
            {
                "radius": 0.748,
                "chord": 0.11,
                "thickness": 0.011,
                "camber": 0.004,
                "x_offset": 0.72,
                "twist_deg": 11.0,
            },
        ],
        samples_per_side=7,
    )
    ogv_array = _radial_array(ogv_blade, 12, phase=math.radians(8.0))
    rear_strut = _blade_geometry(
        [
            {
                "radius": 0.175,
                "chord": 0.095,
                "thickness": 0.010,
                "camber": 0.002,
                "x_offset": 2.23,
                "twist_deg": -4.0,
            },
            {
                "radius": 0.370,
                "chord": 0.078,
                "thickness": 0.008,
                "camber": 0.001,
                "x_offset": 2.33,
                "twist_deg": 8.0,
            },
        ],
        samples_per_side=7,
    )
    rear_struts = _radial_array(rear_strut, 6, phase=math.radians(30.0))

    pylon_profile = [
        (0.92, -0.025),
        (0.98, 0.070),
        (1.12, 0.185),
        (1.30, 0.245),
        (1.48, 0.235),
        (1.62, 0.145),
        (1.68, 0.040),
        (1.63, -0.025),
    ]
    pylon = ExtrudeGeometry(pylon_profile, height=0.26, cap=True, center=True, closed=True)
    pylon.rotate_x(math.pi / 2.0)
    pylon.translate(0.0, 0.0, 0.80)

    core_structure = _combine_geometries(core_body, ogv_array, rear_struts)

    nacelle.visual(
        mesh_from_geometry(outer_shell, ASSETS.mesh_path("engine_outer_shell.obj")),
        origin=IDENTITY,
        material=materials["paint"],
        name="outer_shell",
    )
    nacelle.visual(
        mesh_from_geometry(intake_lip, ASSETS.mesh_path("engine_intake_lip.obj")),
        origin=IDENTITY,
        material=materials["lip"],
        name="intake_lip",
    )
    nacelle.visual(
        mesh_from_geometry(intake_liner, ASSETS.mesh_path("engine_intake_liner.obj")),
        origin=IDENTITY,
        material=materials["inner"],
        name="intake_liner",
    )
    nacelle.visual(
        mesh_from_geometry(core_structure, ASSETS.mesh_path("engine_core_structure.obj")),
        origin=IDENTITY,
        material=materials["steel"],
        name="core_structure",
    )
    nacelle.visual(
        mesh_from_geometry(hot_liner, ASSETS.mesh_path("engine_hot_liner.obj")),
        origin=IDENTITY,
        material=materials["hot"],
        name="hot_liner",
    )
    nacelle.visual(
        mesh_from_geometry(exhaust_band, ASSETS.mesh_path("engine_exhaust_band.obj")),
        origin=IDENTITY,
        material=materials["hot"],
        name="exhaust_band",
    )
    nacelle.visual(
        mesh_from_geometry(pylon, ASSETS.mesh_path("engine_pylon.obj")),
        origin=IDENTITY,
        material=materials["paint"],
        name="pylon",
    )
    nacelle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.89, length=3.06),
        mass=2450.0,
        origin=Origin(xyz=(1.53, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    fan_rotor = model.part("fan_rotor")

    spinner = _lathe_along_x(
        [
            (-0.48, 0.0),
            (-0.39, 0.035),
            (-0.28, 0.080),
            (-0.16, 0.130),
            (-0.06, 0.170),
            (0.00, 0.185),
            (0.00, 0.0),
        ],
        segments=88,
    )
    hub = CylinderGeometry(radius=0.185, height=0.24, radial_segments=56, closed=True)
    hub.rotate_y(math.pi / 2.0)
    hub.translate(-0.12, 0.0, 0.0)
    rotor_metal = _combine_geometries(spinner, hub)

    fan_blade = _blade_geometry(
        [
            {
                "radius": 0.190,
                "chord": 0.220,
                "thickness": 0.020,
                "camber": 0.012,
                "x_offset": -0.19,
                "twist_deg": 48.0,
            },
            {
                "radius": 0.450,
                "chord": 0.180,
                "thickness": 0.014,
                "camber": 0.008,
                "x_offset": -0.13,
                "twist_deg": 27.0,
            },
            {
                "radius": 0.700,
                "chord": 0.112,
                "thickness": 0.009,
                "camber": 0.004,
                "x_offset": -0.045,
                "twist_deg": 7.0,
            },
        ],
        samples_per_side=8,
    )
    fan_blades = _radial_array(fan_blade, 18, phase=math.radians(6.0))

    fan_rotor.visual(
        mesh_from_geometry(rotor_metal, ASSETS.mesh_path("engine_rotor_metal.obj")),
        origin=IDENTITY,
        material=materials["spinner"],
        name="rotor_metal",
    )
    fan_rotor.visual(
        mesh_from_geometry(fan_blades, ASSETS.mesh_path("engine_fan_blades.obj")),
        origin=IDENTITY,
        material=materials["blade"],
        name="fan_blades",
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.71, length=0.50),
        mass=340.0,
        origin=Origin(xyz=(-0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent="nacelle",
        child="fan_rotor",
        origin=Origin(xyz=(FAN_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=320.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "fan_rotor",
        "nacelle",
        reason="Thin swept fan aerofoils produce conservative collision hulls; visual blade-tip clearance remains generous.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.003, overlap_volume_tol=0.0)

    articulation = object_model.get_articulation("fan_spin")
    axis = tuple(float(value) for value in articulation.axis)
    _expect(
        articulation.articulation_type == ArticulationType.CONTINUOUS,
        "The fan should articulate as a continuous rotor.",
    )
    _expect(
        abs(abs(axis[0]) - 1.0) < 1e-6 and abs(axis[1]) < 1e-6 and abs(axis[2]) < 1e-6,
        f"Fan axis should align with the engine centerline, got {axis}.",
    )

    nacelle_mins, nacelle_maxs = _aabb_bounds(ctx.part_world_aabb("nacelle", use="collision"))
    nacelle_length = nacelle_maxs[0] - nacelle_mins[0]
    nacelle_span_y = nacelle_maxs[1] - nacelle_mins[1]
    nacelle_span_z = nacelle_maxs[2] - nacelle_mins[2]

    _expect(
        nacelle_length > 2.9,
        f"Expected a long transport-class engine, got length {nacelle_length:.3f} m.",
    )
    _expect(
        nacelle_span_y > 1.6,
        f"Expected a wide nacelle silhouette, got span_y {nacelle_span_y:.3f} m.",
    )
    _expect(
        nacelle_span_z > 1.7,
        f"Expected nacelle plus pylon height, got span_z {nacelle_span_z:.3f} m.",
    )
    _expect(
        nacelle_maxs[2] > abs(nacelle_mins[2]) + 0.08,
        "The upper pylon fairing should make the engine visibly taller above the centerline than below it.",
    )

    for angle in (0.0, math.pi / 7.0, 2.0 * math.pi / 7.0, math.pi / 2.0):
        with ctx.pose(fan_spin=angle):
            ctx.expect_aabb_overlap("fan_rotor", "nacelle", axes="xy", min_overlap=0.20)
            fan_mins, fan_maxs = _aabb_bounds(ctx.part_world_aabb("fan_rotor", use="collision"))
            fan_center_y = 0.5 * (fan_mins[1] + fan_maxs[1])
            fan_center_z = 0.5 * (fan_mins[2] + fan_maxs[2])
            fan_front = fan_mins[0]
            fan_back = fan_maxs[0]
            fan_length = fan_back - fan_front
            fan_span_y = fan_maxs[1] - fan_mins[1]
            fan_span_z = fan_maxs[2] - fan_mins[2]

            _expect(
                abs(fan_center_y) < 0.02,
                f"Fan should stay centered laterally, got y={fan_center_y:.4f} m at angle {angle:.3f}.",
            )
            _expect(
                abs(fan_center_z) < 0.02,
                f"Fan should stay centered vertically, got z={fan_center_z:.4f} m at angle {angle:.3f}.",
            )
            _expect(
                fan_front < nacelle_mins[0] + 0.18,
                f"Fan nose should sit just inside the intake lip, got front x={fan_front:.3f}.",
            )
            _expect(
                fan_back < nacelle_mins[0] + 0.58,
                f"Rotor should remain in the intake/fan section, got back x={fan_back:.3f}.",
            )
            _expect(
                nacelle_maxs[0] - fan_back > 2.2,
                "The engine should retain a long visible core and exhaust section aft of the fan.",
            )
            _expect(
                fan_length > 0.40,
                f"Fan assembly should have real axial depth, got {fan_length:.3f} m.",
            )
            _expect(
                fan_span_y < nacelle_span_y - 0.18,
                f"Fan should remain comfortably inside the nacelle sidewalls, got span_y {fan_span_y:.3f} m.",
            )
            _expect(
                fan_span_z < nacelle_span_z - 0.40,
                f"Fan should remain visually enclosed below the pylon and nacelle crown, got span_z {fan_span_z:.3f} m.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
