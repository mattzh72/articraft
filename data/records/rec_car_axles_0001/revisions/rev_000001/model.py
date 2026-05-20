from __future__ import annotations

import math

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    Sphere,
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
        try:
            return Material(name=name, color=rgba)
        except TypeError:
            return Material(name, rgba)


def _origin_x(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _origin_y(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_cover_bolts(part, hardware_material: Material) -> None:
    bolt_circle_radius = 0.104
    for idx in range(12):
        angle = (2.0 * math.pi * idx) / 12.0
        y = bolt_circle_radius * math.cos(angle)
        z = bolt_circle_radius * math.sin(angle)
        part.visual(
            Cylinder(radius=0.008, length=0.014),
            origin=_origin_x((-0.168, y, z)),
            material=hardware_material,
        )


def _build_hub(
    part, sign: float, steel: Material, hardware: Material, black_oxide: Material
) -> None:
    part.visual(
        Cylinder(radius=0.042, length=0.058),
        origin=_origin_y((0.0, sign * 0.029, 0.0)),
        material=steel,
    )
    part.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=_origin_y((0.0, sign * 0.022, 0.0)),
        material=black_oxide,
    )
    part.visual(
        Cylinder(radius=0.152, length=0.012),
        origin=_origin_y((0.0, sign * 0.050, 0.0)),
        material=steel,
    )
    part.visual(
        Cylinder(radius=0.073, length=0.034),
        origin=_origin_y((0.0, sign * 0.050, 0.0)),
        material=steel,
    )
    part.visual(
        Cylinder(radius=0.095, length=0.024),
        origin=_origin_y((0.0, sign * 0.070, 0.0)),
        material=steel,
    )
    part.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=_origin_y((0.0, sign * 0.100, 0.0)),
        material=black_oxide,
    )
    part.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.0, sign * 0.114, 0.0)),
        material=black_oxide,
    )

    stud_radius = 0.062
    for idx in range(6):
        angle = (2.0 * math.pi * idx) / 6.0
        x = stud_radius * math.cos(angle)
        z = stud_radius * math.sin(angle)
        part.visual(
            Cylinder(radius=0.007, length=0.030),
            origin=_origin_y((x, sign * 0.095, z)),
            material=hardware,
        )

    part.inertial = Inertial.from_geometry(
        Box((0.32, 0.14, 0.32)),
        mass=14.5,
        origin=Origin(xyz=(0.0, sign * 0.072, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="car_axle_assembly", assets=ASSETS)

    cast_iron = _material("cast_iron", (0.20, 0.20, 0.22, 1.0))
    machined_steel = _material("machined_steel", (0.58, 0.60, 0.62, 1.0))
    zinc_hardware = _material("zinc_hardware", (0.73, 0.75, 0.78, 1.0))
    rubber_isolator = _material("rubber_isolator", (0.09, 0.09, 0.10, 1.0))
    black_oxide = _material("black_oxide", (0.14, 0.14, 0.15, 1.0))
    model.materials.extend([cast_iron, machined_steel, zinc_hardware, rubber_isolator, black_oxide])

    housing = model.part("axle_housing")
    pumpkin_sections = [
        (-0.220, -0.098, 0.086, 0.160),
        (-0.145, -0.136, 0.112, 0.248),
        (-0.060, -0.170, 0.144, 0.332),
        (0.060, -0.162, 0.152, 0.304),
        (0.155, -0.124, 0.112, 0.224),
        (0.220, -0.094, 0.086, 0.160),
    ]
    pumpkin_mesh = mesh_from_geometry(
        superellipse_side_loft(
            pumpkin_sections,
            exponents=2.7,
            segments=72,
            cap=True,
            closed=True,
        ),
        ASSETS.mesh_path("differential_housing.obj"),
    )
    housing.visual(pumpkin_mesh, material=cast_iron)

    for side in (-1.0, 1.0):
        housing.visual(
            Cylinder(radius=0.050, length=0.640),
            origin=_origin_y((0.0, side * 0.500, 0.0)),
            material=cast_iron,
        )
        housing.visual(
            Cylinder(radius=0.058, length=0.060),
            origin=_origin_y((0.0, side * 0.770, 0.0)),
            material=cast_iron,
        )
        housing.visual(
            Cylinder(radius=0.110, length=0.010),
            origin=_origin_y((0.0, side * 0.802, 0.0)),
            material=black_oxide,
        )
        housing.visual(
            Box((0.190, 0.120, 0.014)),
            origin=Origin(xyz=(0.0, side * 0.340, 0.057)),
            material=cast_iron,
        )
        housing.visual(
            Box((0.160, 0.090, 0.008)),
            origin=Origin(xyz=(0.0, side * 0.340, 0.068)),
            material=rubber_isolator,
        )
        housing.visual(
            Box((0.050, 0.120, 0.025)),
            origin=Origin(xyz=(0.0, side * 0.340, 0.074)),
            material=cast_iron,
        )
        housing.visual(
            Box((0.012, 0.050, 0.110)),
            origin=Origin(xyz=(-0.036, side * 0.520, 0.090)),
            material=cast_iron,
        )
        housing.visual(
            Box((0.012, 0.050, 0.110)),
            origin=Origin(xyz=(0.036, side * 0.520, 0.090)),
            material=cast_iron,
        )
        housing.visual(
            Box((0.012, 0.060, 0.100)),
            origin=Origin(xyz=(-0.040, side * 0.205, -0.070)),
            material=cast_iron,
        )
        housing.visual(
            Box((0.012, 0.060, 0.100)),
            origin=Origin(xyz=(0.040, side * 0.205, -0.070)),
            material=cast_iron,
        )
        housing.visual(
            Box((0.092, 0.040, 0.014)),
            origin=Origin(xyz=(0.0, side * 0.205, -0.120)),
            material=cast_iron,
        )

    housing.visual(
        Cylinder(radius=0.132, length=0.046),
        origin=_origin_x((-0.145, 0.0, 0.0)),
        material=cast_iron,
    )
    housing.visual(
        Cylinder(radius=0.146, length=0.012),
        origin=_origin_x((-0.120, 0.0, 0.0)),
        material=cast_iron,
    )
    _add_cover_bolts(housing, zinc_hardware)

    housing.visual(
        Cylinder(radius=0.072, length=0.104),
        origin=_origin_x((0.170, 0.0, 0.0)),
        material=cast_iron,
    )
    housing.visual(
        Cylinder(radius=0.048, length=0.080),
        origin=_origin_x((0.245, 0.0, 0.0)),
        material=cast_iron,
    )
    housing.visual(
        Box((0.112, 0.030, 0.118)),
        origin=Origin(xyz=(0.012, -0.075, -0.010)),
        material=cast_iron,
    )
    housing.visual(
        Box((0.112, 0.030, 0.118)),
        origin=Origin(xyz=(0.012, 0.075, -0.010)),
        material=cast_iron,
    )
    housing.visual(
        Box((0.128, 0.080, 0.022)),
        origin=Origin(xyz=(0.020, 0.0, -0.112)),
        material=cast_iron,
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.030, -0.015, 0.154)),
        material=black_oxide,
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=_origin_x((-0.172, 0.042, 0.072)),
        material=zinc_hardware,
    )
    housing.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=_origin_x((-0.172, 0.000, -0.108)),
        material=zinc_hardware,
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.56, 1.66, 0.38)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    left_hub = model.part("left_hub")
    _build_hub(left_hub, -1.0, machined_steel, zinc_hardware, black_oxide)

    right_hub = model.part("right_hub")
    _build_hub(right_hub, 1.0, machined_steel, zinc_hardware, black_oxide)

    pinion = model.part("pinion_flange")
    pinion.visual(
        Cylinder(radius=0.034, length=0.042),
        origin=_origin_x((0.021, 0.0, 0.0)),
        material=machined_steel,
    )
    pinion.visual(
        Cylinder(radius=0.062, length=0.016),
        origin=_origin_x((0.046, 0.0, 0.0)),
        material=machined_steel,
    )
    pinion.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=_origin_x((0.066, 0.0, 0.0)),
        material=black_oxide,
    )
    pinion.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=_origin_x((0.086, 0.0, 0.0)),
        material=black_oxide,
    )
    for dy, dz in ((0.034, 0.0), (-0.034, 0.0), (0.0, 0.034), (0.0, -0.034)):
        pinion.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=_origin_x((0.052, dy, dz)),
            material=zinc_hardware,
        )
    pinion.inertial = Inertial.from_geometry(
        Box((0.11, 0.14, 0.14)),
        mass=3.4,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
    )

    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent="axle_housing",
        child="left_hub",
        origin=Origin(xyz=(0.0, -0.828, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=40.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent="axle_housing",
        child="right_hub",
        origin=Origin(xyz=(0.0, 0.828, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=40.0),
    )
    model.articulation(
        "pinion_spin",
        ArticulationType.CONTINUOUS,
        parent="axle_housing",
        child="pinion_flange",
        origin=Origin(xyz=(0.292, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=90.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    def require(condition: bool, message: str) -> None:
        if not condition:
            raise AssertionError(message)

    def max_delta(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return max(abs(a[i] - b[i]) for i in range(3))

    housing_pos = ctx.part_world_position("axle_housing")
    left_pos = ctx.part_world_position("left_hub")
    right_pos = ctx.part_world_position("right_hub")
    pinion_pos = ctx.part_world_position("pinion_flange")

    require(
        abs(housing_pos[0]) < 1e-6 and abs(housing_pos[1]) < 1e-6,
        "housing should sit at the object origin",
    )
    require(
        abs(left_pos[0] - housing_pos[0]) < 1e-6,
        "left hub should stay centered on the axle's longitudinal plane",
    )
    require(
        abs(right_pos[0] - housing_pos[0]) < 1e-6,
        "right hub should stay centered on the axle's longitudinal plane",
    )
    require(
        abs(left_pos[2] - housing_pos[2]) < 1e-6, "left hub should share the housing center height"
    )
    require(
        abs(right_pos[2] - housing_pos[2]) < 1e-6,
        "right hub should share the housing center height",
    )
    require(left_pos[1] < -0.80, "left hub should sit outboard on the negative-Y side")
    require(right_pos[1] > 0.80, "right hub should sit outboard on the positive-Y side")
    require(
        abs(left_pos[1] + right_pos[1]) < 1e-6,
        "hub joints should be mirrored across the differential center",
    )
    require(
        (right_pos[1] - left_pos[1]) > 1.64,
        "assembly should have believable automotive track width",
    )
    require(
        pinion_pos[0] > housing_pos[0] + 0.28,
        "pinion flange should project forward of the carrier housing",
    )
    require(
        abs(pinion_pos[1] - housing_pos[1]) < 1e-6,
        "pinion flange should stay on the axle centerline",
    )
    require(
        abs(pinion_pos[2] - housing_pos[2]) < 1e-6,
        "pinion flange should stay level with the carrier center",
    )

    ctx.expect_origin_distance("pinion_flange", "axle_housing", axes="xy", max_dist=0.30)
    ctx.expect_origin_distance("left_hub", "right_hub", axes="xy", max_dist=1.67)

    reference_positions = {
        "axle_housing": housing_pos,
        "left_hub": left_pos,
        "right_hub": right_pos,
        "pinion_flange": pinion_pos,
    }
    for pose in (
        {"left_hub_spin": math.pi / 2.0},
        {"right_hub_spin": -math.pi / 3.0, "pinion_spin": math.pi / 4.0},
        {
            "left_hub_spin": math.pi,
            "right_hub_spin": -math.pi / 2.0,
            "pinion_spin": 1.3,
        },
    ):
        with ctx.pose(pose):
            for part_name, ref_pos in reference_positions.items():
                require(
                    max_delta(ctx.part_world_position(part_name), ref_pos) < 1e-6,
                    f"{part_name} origin drifted under a pure rotational articulation pose",
                )
            current_left = ctx.part_world_position("left_hub")
            current_right = ctx.part_world_position("right_hub")
            current_pinion = ctx.part_world_position("pinion_flange")
            require(current_left[1] < -0.80, "left hub must remain outboard through spin poses")
            require(current_right[1] > 0.80, "right hub must remain outboard through spin poses")
            require(
                current_pinion[0] > housing_pos[0] + 0.28,
                "pinion flange must remain forward of the carrier through spin poses",
            )
            ctx.expect_origin_distance("pinion_flange", "axle_housing", axes="xy", max_dist=0.30)
            ctx.expect_origin_distance("left_hub", "right_hub", axes="xy", max_dist=1.67)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
