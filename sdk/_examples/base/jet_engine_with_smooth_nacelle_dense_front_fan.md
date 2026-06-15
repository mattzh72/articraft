---
title: 'Jet Engine with Smooth Nacelle and Dense Front Fan'
description: 'Base SDK turbofan example with lathed nacelle shells, lofted fan and stator blades, radial blade patterns, layered compressor and turbine stacks, and bypass chevrons.'
tags:
  - sdk
  - base sdk
  - jet engine
  - turbofan
  - turbojet
  - aircraft engine
  - aerospace
  - nacelle
  - smooth nacelle
  - inlet lip
  - spinner
  - fan hub
  - fan blades
  - dense front fan
  - front fan blades
  - stator vanes
  - compressor
  - compressor stack
  - turbine
  - turbine stack
  - turbine shaft
  - engine core
  - core cowl
  - bypass nozzle
  - chevrons
  - exhaust cone
  - shell profiles
  - lathe geometry
  - lofted blades
  - blade section
  - airfoil
  - radial pattern
  - boolean difference
  - ring band
  - mesh geometry
  - mesh from geometry
  - section loft
  - repair loft
  - fixed articulation
---
# Jet Engine with Smooth Nacelle and Dense Front Fan

This base-SDK example is a strong reference for a realistic jet engine or turbofan with a smooth nacelle, dense front fan blades, layered compressor and turbine detail, bypass nozzle petals, and a deep rear exhaust cone. It is useful for queries such as `jet engine`, `turbofan`, `aircraft engine`, `smooth nacelle`, `fan blades`, `compressor stack`, `turbine stack`, `spinner`, `stator vanes`, `bypass nozzle`, `chevrons`, `lathe geometry shell`, `section_loft blade`, `repair_loft`, `radial pattern`, and `mesh_from_geometry`.

The modeling patterns worth copying are:

- `LatheGeometry.from_shell_profiles(...)` for outer nacelle, nozzle shells, and cowl forms.
- `section_loft(...)` plus `repair_loft(...)` for fan blades, stator vanes, turbine blades, and chevron petals.
- radial duplication with `rotate_z(...)` to build dense blade rings.
- ring construction with `boolean_difference(...)` over concentric cylinders.
- separate parts for nacelle, turbine core, and fan assembly with fixed articulations and named visuals.

```python
from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    Origin,
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, filename)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _airfoil_loop(
    radius: float,
    center_y: float,
    center_z: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    return [
        (radius, center_y + 0.00 * thickness, center_z - 0.50 * chord),
        (radius, center_y + 0.50 * thickness, center_z - 0.22 * chord),
        (radius, center_y + 0.58 * thickness, center_z + 0.10 * chord),
        (radius, center_y + 0.18 * thickness, center_z + 0.48 * chord),
        (radius, center_y - 0.10 * thickness, center_z + 0.50 * chord),
        (radius, center_y - 0.54 * thickness, center_z + 0.16 * chord),
        (radius, center_y - 0.52 * thickness, center_z - 0.18 * chord),
        (radius, center_y - 0.14 * thickness, center_z - 0.44 * chord),
    ]


def _blade_section(
    radius: float,
    center_y: float,
    center_z: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_thickness = thickness * 0.5
    return [
        (radius, center_y - 0.96 * half_thickness, center_z - 0.54 * chord),
        (radius, center_y + 0.22 * half_thickness, center_z - 0.10 * chord),
        (radius, center_y + 1.00 * half_thickness, center_z + 0.48 * chord),
        (radius, center_y - 0.26 * half_thickness, center_z + 0.12 * chord),
    ]


def _petal_section(
    z_pos: float,
    inner_radius: float,
    outer_radius: float,
    width: float,
    outer_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    return [
        (inner_radius, -half_width, z_pos),
        (outer_radius, outer_shift - 0.42 * half_width, z_pos),
        (outer_radius, outer_shift + 0.42 * half_width, z_pos),
        (inner_radius, half_width, z_pos),
    ]


def _lofted_shell(sections: list[list[tuple[float, float, float]]]) -> MeshGeometry:
    return repair_loft(section_loft(sections))


def _radial_pattern(base_geom: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geom.copy().rotate_z(angle_offset + (index * math.tau / count)))
    return patterned


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    z_center: float,
    height: float,
    radial_segments: int = 56,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.004,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _build_nacelle_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.360, -0.402),
            (0.424, -0.344),
            (0.458, -0.196),
            (0.478, 0.016),
            (0.460, 0.238),
            (0.398, 0.448),
            (0.340, 0.586),
            (0.308, 0.656),
            (0.276, 0.722),
            (0.248, 0.786),
        ],
        [
            (0.334, -0.430),
            (0.382, -0.356),
            (0.380, -0.198),
            (0.396, 0.010),
            (0.382, 0.236),
            (0.326, 0.450),
            (0.286, 0.602),
            (0.274, 0.674),
            (0.264, 0.736),
            (0.256, 0.800),
        ],
        segments=88,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )
    shell.merge(
        TorusGeometry(
            radius=0.146,
            tube=0.007,
            radial_segments=16,
            tubular_segments=48,
        ).translate(0.0, 0.0, 0.676)
    )
    return shell


def _build_outer_chevron_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.206, 0.672),
            (0.212, 0.724),
            (0.208, 0.782),
            (0.194, 0.842),
        ],
        [
            (0.198, 0.668),
            (0.202, 0.724),
            (0.198, 0.784),
            (0.188, 0.846),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_bypass_nozzle_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.196, 0.536),
            (0.214, 0.608),
            (0.218, 0.696),
            (0.210, 0.786),
            (0.194, 0.858),
        ],
        [
            (0.172, 0.522),
            (0.182, 0.606),
            (0.186, 0.700),
            (0.180, 0.798),
            (0.166, 0.884),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    shell.merge(
        _ring_band(
            outer_radius=0.216,
            inner_radius=0.182,
            z_center=0.620,
            height=0.032,
            radial_segments=56,
        )
    )
    shell.merge(
        _ring_band(
            outer_radius=0.208,
            inner_radius=0.178,
            z_center=0.774,
            height=0.022,
            radial_segments=56,
        )
    )
    return shell


def _build_bypass_chevron_mesh() -> MeshGeometry:
    main_petal = _lofted_shell(
        [
            _petal_section(0.686, 0.182, 0.220, 0.088),
            _petal_section(0.758, 0.178, 0.214, 0.076, outer_shift=-0.004),
            _petal_section(0.836, 0.168, 0.198, 0.056, outer_shift=-0.008),
            _petal_section(0.884, 0.156, 0.176, 0.026, outer_shift=-0.010),
        ]
    )
    overlap_petal = _lofted_shell(
        [
            _petal_section(0.700, 0.184, 0.214, 0.072),
            _petal_section(0.774, 0.178, 0.208, 0.062, outer_shift=0.004),
            _petal_section(0.846, 0.166, 0.192, 0.046, outer_shift=0.008),
            _petal_section(0.870, 0.158, 0.180, 0.022, outer_shift=0.010),
        ]
    )
    return _merge_geometries(
        [
            _radial_pattern(main_petal, 12),
            _radial_pattern(overlap_petal, 12, angle_offset=math.pi / 12.0),
        ]
    )


def _build_center_shaft_mesh() -> MeshGeometry:
    shaft = CylinderGeometry(radius=0.024, height=1.100, radial_segments=36).translate(
        0.0,
        0.0,
        0.170,
    )
    collar = _ring_band(
        outer_radius=0.070,
        inner_radius=0.034,
        z_center=0.070,
        height=0.054,
        radial_segments=48,
    )
    return _merge_geometries([shaft, collar])


def _build_stator_vanes_mesh() -> MeshGeometry:
    vane = _lofted_shell(
        [
            _blade_section(0.118, -0.004, 0.014, 0.136, 0.019),
            _blade_section(0.226, 0.008, 0.090, 0.108, 0.016),
            _blade_section(0.356, 0.020, 0.172, 0.068, 0.010),
        ]
    )
    rings = [
        _ring_band(
            outer_radius=0.374,
            inner_radius=0.356,
            z_center=0.160,
            height=0.024,
            radial_segments=56,
        ),
        _ring_band(
            outer_radius=0.130,
            inner_radius=0.108,
            z_center=0.014,
            height=0.028,
            radial_segments=48,
        ),
    ]
    rings.append(_radial_pattern(vane, 14, angle_offset=math.pi / 14.0))
    return _merge_geometries(rings)


def _build_spinner_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, -0.320),
            (0.018, -0.306),
            (0.044, -0.272),
            (0.078, -0.212),
            (0.118, -0.132),
            (0.152, -0.046),
            (0.172, 0.020),
            (0.172, 0.046),
            (0.154, 0.080),
            (0.0, 0.070),
        ],
        segments=72,
    )


def _build_fan_hub_mesh() -> MeshGeometry:
    body = CylinderGeometry(radius=0.150, height=0.216, radial_segments=56).translate(
        0.0,
        0.0,
        -0.026,
    )
    rear_flange = CylinderGeometry(radius=0.192, height=0.040, radial_segments=56).translate(
        0.0,
        0.0,
        0.048,
    )
    front_collar = _ring_band(
        outer_radius=0.170,
        inner_radius=0.144,
        z_center=-0.102,
        height=0.028,
        radial_segments=48,
    )
    return _merge_geometries([body, rear_flange, front_collar])


def _build_fan_blades_mesh() -> MeshGeometry:
    blade = _lofted_shell(
        [
            _blade_section(0.150, -0.034, -0.200, 0.330, 0.038),
            _blade_section(0.204, -0.026, -0.164, 0.306, 0.034),
            _blade_section(0.268, -0.010, -0.098, 0.268, 0.028),
            _blade_section(0.338, 0.014, -0.004, 0.202, 0.021),
            _blade_section(0.398, 0.032, 0.106, 0.132, 0.012),
            _blade_section(0.432, 0.040, 0.178, 0.076, 0.007),
        ]
    )
    return _radial_pattern(blade, 18, angle_offset=math.pi / 18.0)


def _build_compressor_stack_mesh() -> MeshGeometry:
    stage_geometries: list[MeshGeometry] = []
    stage_specs = [
        (0.068, 0.224, 0.078, 0.014, 28, -0.018),
        (0.156, 0.206, 0.074, 0.014, 26, -0.008),
        (0.246, 0.188, 0.068, 0.014, 24, 0.004),
        (0.338, 0.170, 0.062, 0.014, 22, 0.014),
    ]
    for z_center, tip_radius, root_radius, disk_height, blade_count, phase in stage_specs:
        disk = _ring_band(
            outer_radius=tip_radius,
            inner_radius=root_radius - 0.010,
            z_center=z_center,
            height=disk_height,
            radial_segments=52,
        )
        blade = _lofted_shell(
            [
                _blade_section(root_radius, -0.006, z_center - 0.016, 0.092, 0.016),
                _blade_section(
                    0.5 * (root_radius + tip_radius),
                    0.004,
                    z_center + 0.006,
                    0.070,
                    0.011,
                ),
                _blade_section(tip_radius, 0.014, z_center + 0.030, 0.044, 0.007),
            ]
        )
        stage_geometries.append(disk)
        stage_geometries.append(
            _radial_pattern(blade, blade_count, angle_offset=phase + math.pi / blade_count)
        )
    return _merge_geometries(stage_geometries)


def _build_turbine_stack_mesh() -> MeshGeometry:
    stage_geometries: list[MeshGeometry] = []
    stage_specs = [
        (0.456, 0.154, 0.060, 0.016, 18, 0.012),
        (0.576, 0.138, 0.056, 0.016, 16, 0.000),
        (0.676, 0.122, 0.052, 0.014, 14, -0.010),
    ]
    for z_center, tip_radius, root_radius, disk_height, blade_count, phase in stage_specs:
        disk = _ring_band(
            outer_radius=tip_radius,
            inner_radius=root_radius - 0.008,
            z_center=z_center,
            height=disk_height,
            radial_segments=48,
        )
        blade = _lofted_shell(
            [
                _blade_section(root_radius, -0.002, z_center - 0.012, 0.066, 0.012),
                _blade_section(
                    0.5 * (root_radius + tip_radius),
                    0.006,
                    z_center + 0.004,
                    0.048,
                    0.009,
                ),
                _blade_section(tip_radius, 0.012, z_center + 0.024, 0.034, 0.0065),
            ]
        )
        stage_geometries.append(disk)
        stage_geometries.append(
            _radial_pattern(blade, blade_count, angle_offset=phase + math.pi / blade_count)
        )
    return _merge_geometries(stage_geometries)


def _build_turbine_shaft_mesh() -> MeshGeometry:
    return CylinderGeometry(radius=0.048, height=1.000, radial_segments=40).translate(
        0.0,
        0.0,
        0.220,
    )


def _build_core_cowl_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.134, 0.526),
            (0.158, 0.590),
            (0.182, 0.670),
            (0.188, 0.754),
            (0.178, 0.832),
            (0.162, 0.892),
        ],
        [
            (0.108, 0.512),
            (0.126, 0.588),
            (0.146, 0.672),
            (0.150, 0.758),
            (0.144, 0.842),
            (0.132, 0.912),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    shell.merge(
        _ring_band(
            outer_radius=0.186,
            inner_radius=0.126,
            z_center=0.604,
            height=0.034,
            radial_segments=48,
        )
    )
    shell.merge(
        _ring_band(
            outer_radius=0.180,
            inner_radius=0.142,
            z_center=0.744,
            height=0.022,
            radial_segments=48,
        )
    )
    shell.merge(
        _ring_band(
            outer_radius=0.166,
            inner_radius=0.136,
            z_center=0.860,
            height=0.020,
            radial_segments=48,
        )
    )
    return shell


def _build_exhaust_cone_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.612),
            (0.066, 0.612),
            (0.102, 0.668),
            (0.122, 0.752),
            (0.118, 0.840),
            (0.098, 0.920),
            (0.070, 0.980),
            (0.036, 1.022),
            (0.014, 1.042),
            (0.0, 1.054),
        ],
        segments=64,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jet_engine")

    nacelle_paint = model.material("nacelle_paint", rgba=(0.78, 0.80, 0.84, 1.0))
    bright_metal = model.material("bright_metal", rgba=(0.62, 0.66, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    compressor_metal = model.material("compressor_metal", rgba=(0.58, 0.61, 0.66, 1.0))
    hot_metal = model.material("hot_metal", rgba=(0.58, 0.46, 0.34, 1.0))

    nacelle = model.part("nacelle")
    nacelle.visual(
        _save_mesh(_build_nacelle_shell_mesh(), "nacelle_shell.obj"),
        material=nacelle_paint,
        name="nacelle_shell",
    )
    nacelle.visual(
        _save_mesh(_build_center_shaft_mesh(), "center_shaft.obj"),
        material=dark_steel,
        name="center_shaft",
    )
    nacelle.visual(
        _save_mesh(_build_stator_vanes_mesh(), "stator_vanes.obj"),
        material=dark_steel,
        name="stator_vanes",
    )
    nacelle.visual(
        _save_mesh(_build_bypass_nozzle_shell_mesh(), "bypass_nozzle_shell.obj"),
        material=dark_steel,
        name="bypass_nozzle_shell",
    )
    nacelle.visual(
        _save_mesh(_build_bypass_chevron_mesh(), "bypass_chevrons.obj"),
        material=dark_steel,
        name="bypass_chevrons",
    )
    nacelle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.40, length=1.12),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
    )

    turbine = model.part("turbine_assembly")
    turbine.visual(
        _save_mesh(_build_turbine_shaft_mesh(), "turbine_shaft.obj"),
        material=dark_steel,
        name="turbine_shaft",
    )
    turbine.visual(
        _save_mesh(_build_compressor_stack_mesh(), "compressor_stack.obj"),
        material=compressor_metal,
        name="compressor_stack",
    )
    turbine.visual(
        _save_mesh(_build_turbine_stack_mesh(), "turbine_stack.obj"),
        material=hot_metal,
        name="turbine_stack",
    )
    turbine.visual(
        _save_mesh(_build_core_cowl_mesh(), "core_cowl.obj"),
        material=compressor_metal,
        name="core_cowl",
    )
    turbine.visual(
        _save_mesh(_build_exhaust_cone_mesh(), "exhaust_cone.obj"),
        material=dark_steel,
        name="exhaust_cone",
    )
    turbine.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=1.02),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
    )

    fan = model.part("fan_assembly")
    fan.visual(
        _save_mesh(_build_spinner_mesh(), "spinner.obj"),
        material=bright_metal,
        name="spinner",
    )
    fan.visual(
        _save_mesh(_build_fan_hub_mesh(), "fan_hub.obj"),
        material=dark_steel,
        name="fan_hub",
    )
    fan.visual(
        _save_mesh(_build_fan_blades_mesh(), "fan_blades.obj"),
        material=bright_metal,
        name="fan_blades",
    )
    fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.31, length=0.28),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    model.articulation(
        "nacelle_to_turbine",
        ArticulationType.FIXED,
        parent=nacelle,
        child=turbine,
        origin=Origin(),
    )
    model.articulation(
        "turbine_to_fan",
        ArticulationType.FIXED,
        parent=turbine,
        child=fan,
        origin=Origin(),
    )

    return model
object_model = build_object_model()
```
