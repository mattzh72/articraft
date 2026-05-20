from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
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
    repair_loft,
    section_loft,
)


ENGINE_LENGTH = 1.34
ENGINE_CENTER_Z = 0.55
ROLLER_TOP_Z = 0.16
SUPPORT_XS = (-0.16, 0.16)
PETAL_COUNT = 8


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_engine_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.280, -0.690),
            (0.330, -0.620),
            (0.350, -0.500),
            (0.356, -0.120),
            (0.346, 0.150),
            (0.314, 0.420),
            (0.252, 0.605),
            (0.210, 0.690),
        ],
        [
            (0.250, -0.700),
            (0.292, -0.622),
            (0.312, -0.500),
            (0.318, -0.120),
            (0.306, 0.150),
            (0.276, 0.430),
            (0.222, 0.620),
            (0.170, 0.710),
        ],
        segments=88,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )


def _build_spinner_mesh():
    return LatheGeometry(
        [
            (0.000, -0.150),
            (0.020, -0.145),
            (0.050, -0.120),
            (0.085, -0.075),
            (0.122, -0.010),
            (0.144, 0.048),
            (0.132, 0.086),
            (0.000, 0.074),
        ],
        segments=72,
    )


def _build_ring_shell_mesh(outer_radius: float, inner_radius: float, length: float):
    half_length = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -half_length),
            (outer_radius, half_length),
        ],
        [
            (inner_radius, -half_length),
            (inner_radius, half_length),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _build_exhaust_plug_mesh():
    return LatheGeometry(
        [
            (0.000, 0.150),
            (0.056, 0.160),
            (0.094, 0.245),
            (0.112, 0.385),
            (0.106, 0.510),
            (0.078, 0.635),
            (0.030, 0.745),
            (0.000, 0.770),
        ],
        segments=64,
    )


def _build_petal_mesh():
    def loop(x_pos: float, half_width: float, z_outer: float, z_inner: float):
        return [
            (x_pos, -half_width, z_outer),
            (x_pos, half_width, z_outer),
            (x_pos, 0.92 * half_width, z_inner),
            (x_pos, -0.92 * half_width, z_inner),
        ]

    petal = repair_loft(
        section_loft(
            [
                loop(0.000, 0.058, -0.002, -0.024),
                loop(0.105, 0.050, -0.010, -0.034),
                loop(0.225, 0.035, -0.020, -0.044),
            ]
        )
    )
    petal.merge(
        CylinderGeometry(radius=0.0065, height=0.112, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, 0.0)
    )
    return petal


def _petal_origin(index: int, radius: float, x_pos: float) -> Origin:
    angle = (2.0 * math.pi * index) / PETAL_COUNT + (math.pi / 2.0)
    return Origin(
        xyz=(x_pos, radius * math.cos(angle), radius * math.sin(angle)),
        rpy=(angle - (math.pi / 2.0), 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turbojet_module")

    cradle_paint = model.material("cradle_paint", rgba=(0.24, 0.28, 0.33, 1.0))
    shell_metal = model.material("shell_metal", rgba=(0.44, 0.46, 0.49, 1.0))
    band_metal = model.material("band_metal", rgba=(0.30, 0.32, 0.35, 1.0))
    compressor_metal = model.material("compressor_metal", rgba=(0.72, 0.75, 0.79, 1.0))
    hot_metal = model.material("hot_metal", rgba=(0.53, 0.46, 0.38, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    mount_band_mesh = _save_mesh("mount_band_shell", _build_ring_shell_mesh(0.376, 0.352, 0.080))

    cradle = model.part("cradle")
    cradle.visual(
        Box((1.12, 0.09, 0.08)),
        origin=Origin(xyz=(0.0, -0.31, 0.04)),
        material=cradle_paint,
        name="side_rail_0",
    )
    cradle.visual(
        Box((1.12, 0.09, 0.08)),
        origin=Origin(xyz=(0.0, 0.31, 0.04)),
        material=cradle_paint,
        name="side_rail_1",
    )
    cradle.visual(
        Box((0.12, 0.71, 0.08)),
        origin=Origin(xyz=(-0.46, 0.0, 0.04)),
        material=cradle_paint,
        name="cross_rail_front",
    )
    cradle.visual(
        Box((0.12, 0.71, 0.08)),
        origin=Origin(xyz=(0.46, 0.0, 0.04)),
        material=cradle_paint,
        name="cross_rail_rear",
    )
    cradle.visual(
        Box((0.78, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cradle_paint,
        name="center_rail",
    )
    for support_index, x_pos in enumerate(SUPPORT_XS):
        cradle.visual(
            Box((0.08, 0.08, 0.10)),
            origin=Origin(xyz=(x_pos, -0.18, 0.09)),
            material=cradle_paint,
            name=f"upright_{support_index}_0",
        )
        cradle.visual(
            Box((0.08, 0.08, 0.10)),
            origin=Origin(xyz=(x_pos, 0.18, 0.09)),
            material=cradle_paint,
            name=f"upright_{support_index}_1",
        )
        cradle.visual(
            Box((0.12, 0.54, 0.06)),
            origin=Origin(xyz=(x_pos, 0.0, 0.09)),
            material=cradle_paint,
            name=f"support_beam_{support_index}",
        )
        cradle.visual(
            Cylinder(radius=0.055, length=0.26),
            origin=Origin(xyz=(x_pos, 0.0, 0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=band_metal,
            name="front_roller" if support_index == 0 else "rear_roller",
        )
    cradle.inertial = Inertial.from_geometry(
        Box((1.12, 0.71, 0.26)),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )

    engine = model.part("engine")
    engine.visual(
        _save_mesh("engine_shell", _build_engine_shell_mesh()),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_metal,
        name="engine_shell",
    )
    engine.visual(
        mount_band_mesh,
        origin=Origin(xyz=(-0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=band_metal,
        name="mount_band_front",
    )
    engine.visual(
        mount_band_mesh,
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=band_metal,
        name="mount_band_rear",
    )
    engine.visual(
        Cylinder(radius=0.284, length=0.050),
        origin=Origin(xyz=(-0.655, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="intake_lip",
    )
    engine.visual(
        Cylinder(radius=0.205, length=0.080),
        origin=Origin(xyz=(0.68, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=band_metal,
        name="exhaust_band",
    )
    engine.visual(
        Box((0.09, 0.18, 0.10)),
        origin=Origin(xyz=(-0.16, 0.0, ROLLER_TOP_Z + 0.05 - ENGINE_CENTER_Z)),
        material=band_metal,
        name="support_shoe_front",
    )
    engine.visual(
        Box((0.09, 0.18, 0.10)),
        origin=Origin(xyz=(0.16, 0.0, ROLLER_TOP_Z + 0.05 - ENGINE_CENTER_Z)),
        material=band_metal,
        name="support_shoe_rear",
    )
    engine.visual(
        Cylinder(radius=0.214, length=0.100),
        origin=Origin(xyz=(0.68, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="nozzle_ring",
    )
    engine.visual(
        _save_mesh("exhaust_plug", _build_exhaust_plug_mesh()),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hot_metal,
        name="exhaust_plug",
    )
    engine.inertial = Inertial.from_geometry(
        Cylinder(radius=0.36, length=ENGINE_LENGTH),
        mass=210.0,
        origin=Origin(),
    )

    rotor = model.part("compressor_rotor")
    rotor.visual(
        _save_mesh(
            "compressor_rotor",
            FanRotorGeometry(
                outer_radius=0.236,
                hub_radius=0.074,
                blade_count=18,
                thickness=0.048,
                blade_pitch_deg=36.0,
                blade_sweep_deg=25.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.12),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.020, rear_collar_radius=0.066),
            ),
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=compressor_metal,
        name="rotor_blades",
    )
    rotor.visual(
        _save_mesh("compressor_spinner", _build_spinner_mesh()),
        origin=Origin(xyz=(-0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=compressor_metal,
        name="spinner",
    )
    rotor.visual(
        Cylinder(radius=0.038, length=0.360),
        origin=Origin(xyz=(0.15, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rotor_shaft",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.24, length=0.42),
        mass=18.0,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    petal_mesh = _save_mesh("nozzle_petal", _build_petal_mesh())
    petals: list[str] = []
    for index in range(PETAL_COUNT):
        part_name = f"petal_{index}"
        petals.append(part_name)
        petal = model.part(part_name)
        petal.visual(
            petal_mesh,
            material=hot_metal,
            name="petal_skin",
        )
        petal.inertial = Inertial.from_geometry(
            Box((0.23, 0.12, 0.05)),
            mass=0.9,
            origin=Origin(xyz=(0.11, 0.0, -0.02)),
        )

    model.articulation(
        "cradle_to_engine",
        ArticulationType.FIXED,
        parent=cradle,
        child=engine,
        origin=Origin(xyz=(0.0, 0.0, ENGINE_CENTER_Z)),
    )
    model.articulation(
        "engine_to_compressor_rotor",
        ArticulationType.CONTINUOUS,
        parent=engine,
        child=rotor,
        origin=Origin(xyz=(-0.465, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=35.0),
    )

    for index, part_name in enumerate(petals):
        articulation_name = f"{part_name}_hinge"
        articulation_kwargs = {}
        if index > 0:
            articulation_kwargs["mimic"] = Mimic(joint="petal_0_hinge")
        model.articulation(
            articulation_name,
            ArticulationType.REVOLUTE,
            parent=engine,
            child=part_name,
            origin=_petal_origin(index, radius=0.208, x_pos=0.7365),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.5,
                lower=0.0,
                upper=math.radians(32.0),
            ),
            **articulation_kwargs,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    engine = object_model.get_part("engine")
    rotor = object_model.get_part("compressor_rotor")
    cradle = object_model.get_part("cradle")
    rotor_joint = object_model.get_articulation("engine_to_compressor_rotor")
    petal_joint = object_model.get_articulation("petal_0_hinge")
    top_petal = object_model.get_part("petal_0")

    ctx.expect_gap(
        engine,
        cradle,
        axis="z",
        positive_elem="support_shoe_front",
        negative_elem="front_roller",
        max_gap=0.002,
        max_penetration=0.0,
        name="front cradle roller seats under engine band",
    )
    ctx.expect_gap(
        engine,
        cradle,
        axis="z",
        positive_elem="support_shoe_rear",
        negative_elem="rear_roller",
        max_gap=0.002,
        max_penetration=0.0,
        name="rear cradle roller seats under engine band",
    )
    ctx.expect_within(
        rotor,
        engine,
        axes="yz",
        inner_elem="rotor_blades",
        outer_elem="intake_lip",
        margin=0.03,
        name="compressor rotor stays inside intake opening",
    )
    ctx.check(
        "compressor rotor uses continuous articulation",
        getattr(rotor_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"type={getattr(rotor_joint, 'articulation_type', None)}",
    )

    rest_aabb = ctx.part_element_world_aabb(top_petal, elem="petal_skin")
    with ctx.pose({petal_joint: math.radians(26.0), rotor_joint: 1.0}):
        open_aabb = ctx.part_element_world_aabb(top_petal, elem="petal_skin")

    ctx.check(
        "top nozzle petal opens outward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > rest_aabb[1][2] + 0.035,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
