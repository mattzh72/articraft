from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _yoke_geometry() -> MeshGeometry:
    """Small protected U-yoke around a transverse pivot pin."""
    geom = MeshGeometry()
    for size, xyz in (
        ((0.060, 0.018, 0.080), (0.0, 0.058, 0.0)),
        ((0.060, 0.018, 0.080), (0.0, -0.058, 0.0)),
        ((0.060, 0.134, 0.016), (0.0, 0.0, -0.044)),
    ):
        geom.merge(BoxGeometry(size).translate(*xyz))
    return geom


def _x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _add_wand_segment(
    part,
    *,
    material_tube: str,
    material_rubber: str,
    material_metal: str,
    distal: bool,
) -> None:
    """Straight weather-sealed wand segment with a pivot boss at its root."""
    part.visual(
        Box((0.040, 0.086, 0.070)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=material_rubber,
        name="proximal_pivot_boss",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.134),
        origin=_y_cylinder_origin(0.0, 0.0, 0.0),
        material=material_metal,
        name="proximal_pivot_pin",
    )
    part.visual(
        Cylinder(radius=0.041, length=0.090),
        origin=_x_cylinder_origin(0.073, 0.0, 0.0),
        material=material_rubber,
        name="proximal_collar",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.470),
        origin=_x_cylinder_origin(0.340, 0.0, 0.0),
        material=material_tube,
        name="sealed_wand_tube",
    )
    part.visual(
        Cylinder(radius=0.033, length=0.030),
        origin=_x_cylinder_origin(0.160, 0.0, 0.0),
        material=material_rubber,
        name="proximal_o_ring",
    )
    part.visual(
        Cylinder(radius=0.033, length=0.030),
        origin=_x_cylinder_origin(0.500, 0.0, 0.0),
        material=material_rubber,
        name="distal_o_ring",
    )
    if distal:
        part.visual(
            Cylinder(radius=0.038, length=0.080),
            origin=_x_cylinder_origin(0.568, 0.0, 0.0),
            material=material_rubber,
            name="distal_collar",
        )
        part.visual(
            mesh_from_geometry(_yoke_geometry(), f"{part.name}_distal_yoke"),
            origin=Origin(xyz=(0.620, 0.0, 0.0)),
            material=material_rubber,
            name="distal_yoke",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_vacuum")

    model.material("moss_polymer", rgba=(0.16, 0.28, 0.20, 1.0))
    model.material("dark_polymer", rgba=(0.05, 0.06, 0.055, 1.0))
    model.material("black_rubber", rgba=(0.005, 0.006, 0.005, 1.0))
    model.material("wand_anodized", rgba=(0.38, 0.44, 0.43, 1.0))
    model.material("stainless", rgba=(0.78, 0.80, 0.76, 1.0))
    model.material("warning_amber", rgba=(1.0, 0.55, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.580, 0.420, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material="dark_polymer",
        name="sealed_base_skid",
    )
    body.visual(
        Box((0.480, 0.340, 0.380)),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material="moss_polymer",
        name="tank_shell",
    )
    for x in (-0.235, 0.235):
        for y in (-0.165, 0.165):
            body.visual(
                Cylinder(radius=0.034, length=0.380),
                origin=Origin(xyz=(x, y, 0.250)),
                material="black_rubber",
                name=f"corner_bumper_{'p' if x > 0 else 'n'}x_{'p' if y > 0 else 'n'}y",
            )
    body.visual(
        Box((0.580, 0.420, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material="dark_polymer",
        name="sealed_lid",
    )
    body.visual(
        Box((0.640, 0.480, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material="dark_polymer",
        name="drip_overhang_roof",
    )
    body.visual(
        Box((0.605, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, 0.213, 0.443)),
        material="black_rubber",
        name="side_gasket_0",
    )
    body.visual(
        Box((0.605, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, -0.213, 0.443)),
        material="black_rubber",
        name="side_gasket_1",
    )
    body.visual(
        Box((0.026, 0.420, 0.030)),
        origin=Origin(xyz=(0.293, 0.0, 0.443)),
        material="black_rubber",
        name="front_gasket",
    )
    body.visual(
        Box((0.026, 0.420, 0.030)),
        origin=Origin(xyz=(-0.293, 0.0, 0.443)),
        material="black_rubber",
        name="rear_gasket",
    )
    body.visual(
        Box((0.170, 0.210, 0.025)),
        origin=Origin(xyz=(0.307, 0.0, 0.357)),
        material="dark_polymer",
        name="inlet_rain_hood",
    )
    body.visual(
        Box((0.030, 0.235, 0.035)),
        origin=Origin(xyz=(0.372, 0.0, 0.336)),
        material="dark_polymer",
        name="hood_drip_lip",
    )
    body.visual(
        Box((0.110, 0.170, 0.170)),
        origin=Origin(xyz=(0.265, 0.0, 0.265)),
        material="moss_polymer",
        name="front_bulkhead_boss",
    )
    body.visual(
        Cylinder(radius=0.052, length=0.125),
        origin=_x_cylinder_origin(0.3305, 0.0, 0.265),
        material="black_rubber",
        name="front_socket",
    )
    body.visual(
        Cylinder(radius=0.066, length=0.030),
        origin=_x_cylinder_origin(0.302, 0.0, 0.265),
        material="black_rubber",
        name="inlet_seal_ring",
    )
    body.visual(
        mesh_from_geometry(_yoke_geometry(), "body_front_yoke"),
        origin=Origin(xyz=(0.405, 0.0, 0.265)),
        material="black_rubber",
        name="front_yoke",
    )

    handle_mesh = tube_from_spline_points(
        [
            (-0.185, 0.0, 0.545),
            (-0.120, 0.0, 0.610),
            (0.120, 0.0, 0.610),
            (0.185, 0.0, 0.545),
        ],
        radius=0.014,
        samples_per_segment=16,
        radial_segments=20,
    )
    body.visual(
        mesh_from_geometry(handle_mesh, "arched_carry_handle"),
        material="dark_polymer",
        name="carry_handle",
    )
    body.visual(
        Box((0.080, 0.055, 0.030)),
        origin=Origin(xyz=(-0.185, 0.0, 0.548)),
        material="dark_polymer",
        name="handle_foot_0",
    )
    body.visual(
        Box((0.080, 0.055, 0.030)),
        origin=Origin(xyz=(0.185, 0.0, 0.548)),
        material="dark_polymer",
        name="handle_foot_1",
    )
    for y in (-0.201, 0.201):
        body.visual(
            Box((0.050, 0.018, 0.115)),
            origin=Origin(xyz=(0.105, y, 0.455)),
            material="stainless",
            name=f"stainless_latch_{'p' if y > 0 else 'n'}",
        )
        body.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=_y_cylinder_origin(0.105, y, 0.482),
            material="stainless",
            name=f"upper_screw_{'p' if y > 0 else 'n'}",
        )
        body.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=_y_cylinder_origin(0.105, y, 0.428),
            material="stainless",
            name=f"lower_screw_{'p' if y > 0 else 'n'}",
        )
    body.visual(
        Box((0.105, 0.066, 0.014)),
        origin=Origin(xyz=(-0.185, 0.125, 0.549)),
        material="black_rubber",
        name="switch_gasket",
    )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Box((0.078, 0.044, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material="warning_amber",
        name="sealed_rocker_cap",
    )
    power_switch.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=_y_cylinder_origin(0.0, 0.0, 0.002),
        material="stainless",
        name="rocker_pin",
    )
    model.articulation(
        "body_to_power_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_switch,
        origin=Origin(xyz=(-0.185, 0.125, 0.556)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.22, upper=0.22),
    )

    wand_0 = model.part("wand_0")
    _add_wand_segment(
        wand_0,
        material_tube="wand_anodized",
        material_rubber="black_rubber",
        material_metal="stainless",
        distal=True,
    )
    model.articulation(
        "body_to_wand_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand_0,
        origin=Origin(xyz=(0.405, 0.0, 0.265)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-0.45, upper=0.85),
    )

    wand_1 = model.part("wand_1")
    _add_wand_segment(
        wand_1,
        material_tube="wand_anodized",
        material_rubber="black_rubber",
        material_metal="stainless",
        distal=True,
    )
    model.articulation(
        "wand_0_to_wand_1",
        ArticulationType.REVOLUTE,
        parent=wand_0,
        child=wand_1,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.2, lower=-0.70, upper=0.90),
    )

    nozzle = model.part("floor_nozzle")
    nozzle.visual(
        Box((0.040, 0.086, 0.070)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material="black_rubber",
        name="proximal_pivot_boss",
    )
    nozzle.visual(
        Cylinder(radius=0.010, length=0.134),
        origin=_y_cylinder_origin(0.0, 0.0, 0.0),
        material="stainless",
        name="proximal_pivot_pin",
    )
    nozzle.visual(
        Cylinder(radius=0.041, length=0.090),
        origin=_x_cylinder_origin(0.073, 0.0, 0.0),
        material="black_rubber",
        name="proximal_collar",
    )
    neck_mesh = tube_from_spline_points(
        [
            (0.095, 0.0, 0.0),
            (0.155, 0.0, -0.075),
            (0.210, 0.0, -0.155),
            (0.235, 0.0, -0.195),
        ],
        radius=0.024,
        samples_per_segment=14,
        radial_segments=20,
    )
    nozzle.visual(
        mesh_from_geometry(neck_mesh, "nozzle_drop_neck"),
        material="black_rubber",
        name="drop_neck",
    )
    nozzle.visual(
        Box((0.340, 0.500, 0.070)),
        origin=Origin(xyz=(0.360, 0.0, -0.230)),
        material="dark_polymer",
        name="nozzle_shell",
    )
    nozzle.visual(
        Box((0.380, 0.540, 0.018)),
        origin=Origin(xyz=(0.360, 0.0, -0.187)),
        material="dark_polymer",
        name="drip_visor",
    )
    nozzle.visual(
        Box((0.040, 0.525, 0.035)),
        origin=Origin(xyz=(0.185, 0.0, -0.2475)),
        material="black_rubber",
        name="rear_squeegee",
    )
    nozzle.visual(
        Box((0.040, 0.525, 0.035)),
        origin=Origin(xyz=(0.535, 0.0, -0.2475)),
        material="black_rubber",
        name="front_squeegee",
    )
    nozzle.visual(
        Box((0.270, 0.055, 0.026)),
        origin=Origin(xyz=(0.360, 0.245, -0.257)),
        material="black_rubber",
        name="side_glide_0",
    )
    nozzle.visual(
        Box((0.270, 0.055, 0.026)),
        origin=Origin(xyz=(0.360, -0.245, -0.257)),
        material="black_rubber",
        name="side_glide_1",
    )
    model.articulation(
        "wand_1_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=wand_1,
        child=nozzle,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=-0.90, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    wand_0 = object_model.get_part("wand_0")
    wand_1 = object_model.get_part("wand_1")
    nozzle = object_model.get_part("floor_nozzle")
    switch = object_model.get_part("power_switch")

    body_joint = object_model.get_articulation("body_to_wand_0")
    elbow_joint = object_model.get_articulation("wand_0_to_wand_1")
    nozzle_joint = object_model.get_articulation("wand_1_to_nozzle")

    ctx.allow_overlap(
        body,
        wand_0,
        elem_a="front_yoke",
        elem_b="proximal_pivot_pin",
        reason="The stainless shoulder pin is intentionally captured through the protected rubber yoke cheeks.",
    )
    ctx.allow_overlap(
        wand_0,
        wand_1,
        elem_a="distal_yoke",
        elem_b="proximal_pivot_pin",
        reason="The elbow pivot pin is intentionally captured through the distal yoke cheeks.",
    )
    ctx.allow_overlap(
        wand_1,
        nozzle,
        elem_a="distal_yoke",
        elem_b="proximal_pivot_pin",
        reason="The floor-nozzle pivot pin is intentionally captured through the wand-end yoke cheeks.",
    )

    ctx.expect_gap(
        wand_0,
        body,
        axis="x",
        positive_elem="proximal_pivot_boss",
        negative_elem="front_socket",
        max_gap=0.002,
        max_penetration=0.0,
        name="body socket bears against wand pivot boss",
    )
    ctx.expect_gap(
        wand_1,
        wand_0,
        axis="x",
        positive_elem="proximal_pivot_boss",
        negative_elem="distal_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="wand elbow has hard collar contact",
    )
    ctx.expect_gap(
        nozzle,
        wand_1,
        axis="x",
        positive_elem="proximal_pivot_boss",
        negative_elem="distal_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="nozzle pivot has hard collar contact",
    )
    ctx.expect_overlap(
        body,
        wand_0,
        axes="yz",
        elem_a="front_yoke",
        elem_b="proximal_pivot_pin",
        min_overlap=0.010,
        name="body yoke captures shoulder pin",
    )
    ctx.expect_overlap(
        wand_0,
        wand_1,
        axes="yz",
        elem_a="distal_yoke",
        elem_b="proximal_pivot_pin",
        min_overlap=0.010,
        name="elbow yoke captures pivot pin",
    )
    ctx.expect_overlap(
        wand_1,
        nozzle,
        axes="yz",
        elem_a="distal_yoke",
        elem_b="proximal_pivot_pin",
        min_overlap=0.010,
        name="nozzle yoke captures pivot pin",
    )

    nozzle_aabb = ctx.part_world_aabb(nozzle)
    ctx.check(
        "floor nozzle sits on floor plane",
        nozzle_aabb is not None and abs(nozzle_aabb[0][2]) <= 0.006,
        details=f"nozzle_aabb={nozzle_aabb}",
    )

    rest_wand_1 = ctx.part_world_position(wand_1)
    with ctx.pose({body_joint: 0.55}):
        raised_wand_1 = ctx.part_world_position(wand_1)
    ctx.check(
        "shoulder joint raises the articulated wand",
        rest_wand_1 is not None
        and raised_wand_1 is not None
        and raised_wand_1[2] > rest_wand_1[2] + 0.25,
        details=f"rest={rest_wand_1}, raised={raised_wand_1}",
    )

    rest_nozzle = ctx.part_world_position(nozzle)
    with ctx.pose({elbow_joint: 0.60}):
        folded_nozzle = ctx.part_world_position(nozzle)
    ctx.check(
        "elbow joint changes nozzle height",
        rest_nozzle is not None
        and folded_nozzle is not None
        and folded_nozzle[2] > rest_nozzle[2] + 0.20,
        details=f"rest={rest_nozzle}, folded={folded_nozzle}",
    )

    with ctx.pose({nozzle_joint: 0.45}):
        pitched_nozzle_aabb = ctx.part_world_aabb(nozzle)
    ctx.check(
        "nozzle pitch joint visibly tips the floor head",
        nozzle_aabb is not None
        and pitched_nozzle_aabb is not None
        and pitched_nozzle_aabb[1][2] > nozzle_aabb[1][2] + 0.06,
        details=f"rest={nozzle_aabb}, pitched={pitched_nozzle_aabb}",
    )

    ctx.expect_gap(
        switch,
        body,
        axis="z",
        positive_elem="sealed_rocker_cap",
        negative_elem="switch_gasket",
        max_gap=0.006,
        max_penetration=0.0,
        name="sealed rocker rests on gasket",
    )

    return ctx.report()


object_model = build_object_model()
