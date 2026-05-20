from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Inertial,
    LatheGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merge_geometries(*geometries):
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return merged


def _build_nacelle_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.50, -0.78),
            (0.57, -0.70),
            (0.61, -0.40),
            (0.60, 0.12),
            (0.53, 0.48),
            (0.41, 0.80),
            (0.33, 0.94),
        ],
        [
            (0.46, -0.80),
            (0.49, -0.70),
            (0.50, -0.40),
            (0.49, 0.10),
            (0.44, 0.48),
            (0.34, 0.81),
            (0.27, 0.94),
        ],
        segments=88,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )


def _build_core_geometry():
    core_body = LatheGeometry(
        [
            (0.0, -0.30),
            (0.09, -0.30),
            (0.14, -0.18),
            (0.22, 0.22),
            (0.20, 0.54),
            (0.14, 0.88),
            (0.07, 1.07),
            (0.03, 1.13),
            (0.0, 1.16),
        ],
        segments=72,
    )

    struts = []
    for index in range(4):
        angle = index * (math.pi / 2.0)
        strut = (
            BoxGeometry((0.4322, 0.028, 0.10))
            .translate(0.276, 0.0, -0.08)
            .rotate_z(angle)
        )
        struts.append(strut)

    return _merge_geometries(core_body, *struts)


def _build_fan_geometry():
    rotor = FanRotorGeometry(
        0.46,
        0.13,
        18,
        thickness=0.16,
        blade_pitch_deg=35.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(
            shape="scimitar",
            tip_pitch_deg=18.0,
            camber=0.16,
            tip_clearance=0.012,
        ),
        hub=FanRotorHub(
            style="spinner",
            rear_collar_height=0.05,
            rear_collar_radius=0.12,
        ),
    )
    shaft = CylinderGeometry(radius=0.044, height=0.26, radial_segments=36).translate(0.0, 0.0, 0.13)
    return _merge_geometries(rotor, shaft)


def _build_nozzle_ring():
    return LatheGeometry.from_shell_profiles(
        [
            (0.315, 0.94),
            (0.315, 0.99),
        ],
        [
            (0.255, 0.94),
            (0.255, 0.99),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_petal_panel():
    return ExtrudeGeometry.from_z0(
        [
            (-0.090, -0.042),
            (0.010, -0.028),
            (0.010, 0.028),
            (-0.090, 0.042),
        ],
        0.22,
    )


def _build_petal_barrel():
    return CylinderGeometry(radius=0.0085, height=0.058, radial_segments=20).rotate_x(math.pi / 2.0)


def _build_petal_root_web():
    return BoxGeometry((0.040, 0.050, 0.050)).translate(-0.020, 0.0, 0.032)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turbofan_engine")

    nacelle_paint = model.material("nacelle_paint", rgba=(0.82, 0.84, 0.88, 1.0))
    dark_titanium = model.material("dark_titanium", rgba=(0.26, 0.28, 0.31, 1.0))
    bright_titanium = model.material("bright_titanium", rgba=(0.62, 0.65, 0.70, 1.0))
    hot_metal = model.material("hot_metal", rgba=(0.53, 0.44, 0.35, 1.0))

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(_build_nacelle_shell(), "nacelle_shell"),
        material=nacelle_paint,
        name="shell",
    )
    nacelle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.58, length=1.72),
        mass=48.0,
    )

    core = model.part("core")
    core.visual(
        mesh_from_geometry(_build_core_geometry(), "engine_core"),
        material=dark_titanium,
        name="core_body",
    )

    fan = model.part("fan")
    fan.visual(
        mesh_from_geometry(_build_fan_geometry(), "front_fan"),
        material=bright_titanium,
        name="fan_rotor",
    )
    fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.46, length=0.16),
        mass=12.0,
    )

    model.articulation(
        "nacelle_to_core",
        ArticulationType.FIXED,
        parent=nacelle,
        child=core,
        origin=Origin(),
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan,
        origin=Origin(xyz=(0.0, 0.0, -0.56)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=60.0),
    )

    nozzle_ring = model.part("nozzle_ring")
    nozzle_ring.visual(
        mesh_from_geometry(_build_nozzle_ring(), "nozzle_ring"),
        material=dark_titanium,
        name="ring",
    )

    model.articulation(
        "nacelle_to_nozzle_ring",
        ArticulationType.FIXED,
        parent=nacelle,
        child=nozzle_ring,
        origin=Origin(),
    )

    for index in range(8):
        angle = index * (math.tau / 8.0)
        petal = model.part(f"nozzle_petal_{index}")
        petal.visual(
            mesh_from_geometry(_build_petal_barrel(), f"nozzle_petal_barrel_{index}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0085)),
            material=dark_titanium,
            name="pivot_barrel",
        )
        petal.visual(
            mesh_from_geometry(_build_petal_panel(), f"nozzle_petal_panel_{index}"),
            origin=Origin(xyz=(0.0, 0.0, 0.055), rpy=(0.0, -0.58, 0.0)),
            material=hot_metal,
            name="petal_panel",
        )
        petal.visual(
            mesh_from_geometry(_build_petal_root_web(), f"nozzle_petal_root_web_{index}"),
            material=hot_metal,
            name="root_web",
        )

        articulation_kwargs = {}
        if index != 0:
            articulation_kwargs["mimic"] = Mimic(joint="petal_0_hinge")

        model.articulation(
            f"petal_{index}_hinge",
            ArticulationType.REVOLUTE,
            parent=nozzle_ring,
            child=petal,
            origin=Origin(
                xyz=(0.300 * math.cos(angle), 0.300 * math.sin(angle), 0.99),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=0.48),
            **articulation_kwargs,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    nacelle = object_model.get_part("nacelle")
    core = object_model.get_part("core")
    fan = object_model.get_part("fan")
    nozzle_ring = object_model.get_part("nozzle_ring")
    petal_0 = object_model.get_part("nozzle_petal_0")
    petal_0_hinge = object_model.get_articulation("petal_0_hinge")

    ctx.expect_within(
        fan,
        nacelle,
        axes="xy",
        inner_elem="fan_rotor",
        outer_elem="shell",
        margin=0.13,
        name="fan stays within nacelle envelope",
    )
    ctx.expect_origin_distance(
        fan,
        core,
        axes="xy",
        max_dist=0.001,
        name="fan and core stay on one centerline",
    )
    ctx.expect_origin_gap(
        core,
        fan,
        axis="z",
        min_gap=0.45,
        name="core sits aft of the front fan plane",
    )
    ctx.expect_contact(
        nozzle_ring,
        nacelle,
        elem_a="ring",
        elem_b="shell",
        name="rear nozzle ring seats against nacelle tail",
    )
    ctx.expect_overlap(
        petal_0,
        nozzle_ring,
        axes="xy",
        elem_a="pivot_barrel",
        elem_b="ring",
        min_overlap=0.01,
        name="closed petal pivot stays registered to the nozzle ring",
    )
    ctx.expect_overlap(
        petal_0,
        nozzle_ring,
        axes="xy",
        elem_a="petal_panel",
        elem_b="ring",
        min_overlap=0.02,
        name="closed petal remains rooted over the rear nozzle ring",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(petal_0, elem="petal_panel")
    with ctx.pose({petal_0_hinge: 0.48}):
        ctx.expect_overlap(
            petal_0,
            nozzle_ring,
            axes="xy",
            elem_a="pivot_barrel",
            elem_b="ring",
            min_overlap=0.01,
            name="petal pivot stays aligned over the nozzle ring when opened",
        )
        open_panel_aabb = ctx.part_element_world_aabb(petal_0, elem="petal_panel")

    closed_max_z = None if closed_panel_aabb is None else float(closed_panel_aabb[1][2])
    open_max_z = None if open_panel_aabb is None else float(open_panel_aabb[1][2])
    ctx.check(
        "nozzle petal opens outward",
        closed_max_z is not None and open_max_z is not None and open_max_z > closed_max_z + 0.015,
        details=f"closed_max_z={closed_max_z}, open_max_z={open_max_z}",
    )

    return ctx.report()


object_model = build_object_model()
