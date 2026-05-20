from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


AXLE_X = -0.08
AXLE_Z = 0.42
WHEEL_Y = 0.505
TRUNNION_X = 0.12
TRUNNION_Z = 0.74


def _cheek_plate_mesh(name: str):
    """A flat wooden carriage cheek plate, profiled in X/Z and extruded in Y."""

    cheek_profile_xz = [
        (-0.66, 0.36),
        (0.50, 0.44),
        (0.48, 0.64),
        (0.23, 0.81),
        (0.19, 0.81),
        (0.175, 0.725),
        (0.120, 0.665),
        (0.065, 0.725),
        (0.050, 0.805),
        (-0.42, 0.78),
        (-0.66, 0.58),
    ]
    return mesh_from_geometry(
        ExtrudeGeometry(cheek_profile_xz, 0.055, center=True),
        name,
    )


def _barrel_mesh():
    """Lathed, tapered cast-iron cannon barrel with integral cascabel knob."""

    profile_radius_x = [
        (0.010, -0.690),
        (0.044, -0.675),
        (0.058, -0.630),
        (0.050, -0.585),
        (0.034, -0.548),
        (0.034, -0.535),
        (0.046, -0.515),
        (0.065, -0.505),
        (0.086, -0.455),
        (0.110, -0.405),
        (0.112, -0.270),
        (0.098, -0.215),
        (0.088, 0.080),
        (0.074, 0.600),
        (0.058, 1.060),
        (0.074, 1.115),
        (0.077, 1.245),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile_radius_x, segments=64),
        "tapered_barrel",
    )


def _wooden_wheel_mesh(name: str):
    return mesh_from_geometry(
        WheelGeometry(
            0.370,
            0.066,
            rim=WheelRim(
                inner_radius=0.292,
                flange_height=0.012,
                flange_thickness=0.006,
                bead_seat_depth=0.003,
            ),
            hub=WheelHub(radius=0.064, width=0.090, cap_style="domed"),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=12, thickness=0.017, window_radius=0.020),
            bore=WheelBore(style="round", diameter=0.034),
        ),
        name,
    )


def _iron_tire_mesh(name: str):
    return mesh_from_geometry(
        TireGeometry(
            0.420,
            0.074,
            inner_radius=0.371,
            sidewall=TireSidewall(style="square", bulge=0.01),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swedish_regimental_three_pounder")

    aged_oak = model.material("aged_oak", rgba=(0.52, 0.31, 0.14, 1.0))
    endgrain_oak = model.material("dark_endgrain_oak", rgba=(0.37, 0.21, 0.09, 1.0))
    cast_iron = model.material("blackened_cast_iron", rgba=(0.055, 0.060, 0.060, 1.0))
    worn_iron = model.material("worn_iron", rgba=(0.18, 0.18, 0.17, 1.0))

    carriage = model.part("carriage")
    carriage.visual(
        _cheek_plate_mesh("cheek_plate_notched_0"),
        origin=Origin(xyz=(0.0, -0.245, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_oak,
        name="cheek_plate_0",
    )
    carriage.visual(
        mesh_from_geometry(TorusGeometry(0.061, 0.014, radial_segments=48, tubular_segments=12), "bearing_mesh_0"),
        origin=Origin(xyz=(TRUNNION_X, -0.245, TRUNNION_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_iron,
        name="trunnion_bearing_0",
    )
    carriage.visual(
        _cheek_plate_mesh("cheek_plate_notched_1"),
        origin=Origin(xyz=(0.0, 0.245, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_oak,
        name="cheek_plate_1",
    )
    carriage.visual(
        mesh_from_geometry(TorusGeometry(0.061, 0.014, radial_segments=48, tubular_segments=12), "bearing_mesh_1"),
        origin=Origin(xyz=(TRUNNION_X, 0.245, TRUNNION_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_iron,
        name="trunnion_bearing_1",
    )

    carriage.visual(
        Box((0.220, 0.850, 0.130)),
        origin=Origin(xyz=(AXLE_X, 0.0, AXLE_Z)),
        material=aged_oak,
        name="axle_beam",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=1.180),
        origin=Origin(xyz=(AXLE_X, 0.0, AXLE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_iron,
        name="axle_iron",
    )
    carriage.visual(
        Box((0.190, 0.570, 0.110)),
        origin=Origin(xyz=(0.210, 0.0, 0.505)),
        material=aged_oak,
        name="front_transom",
    )
    carriage.visual(
        Box((0.170, 0.570, 0.120)),
        origin=Origin(xyz=(-0.585, 0.0, 0.360)),
        material=aged_oak,
        name="rear_transom",
    )
    carriage.visual(
        Box((1.070, 0.120, 0.100)),
        origin=Origin(xyz=(-1.055, 0.0, 0.285)),
        material=aged_oak,
        name="trail_stock",
    )
    carriage.visual(
        Box((0.155, 0.085, 0.085)),
        origin=Origin(xyz=(-1.570, 0.0, 0.290)),
        material=endgrain_oak,
        name="handle_neck",
    )
    carriage.visual(
        Cylinder(radius=0.027, length=0.455),
        origin=Origin(xyz=(-1.655, 0.0, 0.290), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=endgrain_oak,
        name="trail_handle",
    )
    for index, (x, z) in enumerate(((-0.470, 0.505), (0.345, 0.600))):
        carriage.visual(
            Cylinder(radius=0.014, length=0.615),
            origin=Origin(xyz=(x, 0.0, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=worn_iron,
            name=f"tie_bolt_{index}",
        )

    barrel = model.part("barrel")
    barrel.visual(
        _barrel_mesh(),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="barrel_tube",
    )
    barrel.visual(
        Cylinder(radius=0.0475, length=0.560),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="trunnion_pin",
    )

    model.articulation(
        "trunnion",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(TRUNNION_X, 0.0, TRUNNION_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=-0.10, upper=0.36),
    )

    for index, y in enumerate((-WHEEL_Y, WHEEL_Y)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            _wooden_wheel_mesh(f"wooden_wheel_{index}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=aged_oak,
            name="wooden_wheel",
        )
        wheel.visual(
            _iron_tire_mesh(f"iron_tire_{index}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=worn_iron,
            name="iron_tire",
        )
        model.articulation(
            f"axle_{index}",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel,
            origin=Origin(xyz=(AXLE_X, y, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    trunnion = object_model.get_articulation("trunnion")

    ctx.check("four_semantic_parts", all(p is not None for p in (carriage, barrel, wheel_0, wheel_1)))
    ctx.check("elevating_trunnion_joint", trunnion is not None and trunnion.motion_limits is not None)

    for wheel_name in ("wheel_0", "wheel_1"):
        ctx.allow_overlap(
            "carriage",
            wheel_name,
            elem_a="axle_iron",
            elem_b="wooden_wheel",
            reason="The iron axle journal is intentionally captured inside the wooden wheel hub bore.",
        )
    for bearing_name in ("trunnion_bearing_0", "trunnion_bearing_1"):
        ctx.allow_overlap(
            "carriage",
            "barrel",
            elem_a=bearing_name,
            elem_b="trunnion_pin",
            reason="The cast trunnion pin is intentionally seated through the iron bearing ring.",
        )

    ctx.expect_within(
        barrel,
        carriage,
        axes="y",
        inner_elem="trunnion_pin",
        outer_elem="front_transom",
        margin=0.0,
        name="trunnion_pin_between_cheeks",
    )
    ctx.expect_overlap(
        barrel,
        carriage,
        axes="xz",
        elem_a="trunnion_pin",
        elem_b="trunnion_bearing_0",
        min_overlap=0.040,
        name="trunnion_lines_up_with_bearing_0",
    )
    ctx.expect_overlap(
        barrel,
        carriage,
        axes="xz",
        elem_a="trunnion_pin",
        elem_b="trunnion_bearing_1",
        min_overlap=0.040,
        name="trunnion_lines_up_with_bearing_1",
    )
    ctx.expect_gap(
        wheel_1,
        carriage,
        axis="y",
        min_gap=0.004,
        max_gap=0.050,
        positive_elem="wooden_wheel",
        negative_elem="axle_beam",
        name="positive_wheel_seated_on_axle",
    )
    ctx.expect_gap(
        carriage,
        wheel_0,
        axis="y",
        min_gap=0.004,
        max_gap=0.050,
        positive_elem="axle_beam",
        negative_elem="wooden_wheel",
        name="negative_wheel_seated_on_axle",
    )
    ctx.expect_overlap(
        wheel_0,
        carriage,
        axes="xz",
        elem_a="wooden_wheel",
        elem_b="axle_beam",
        min_overlap=0.090,
        name="wheel_0_centered_on_axle_beam",
    )
    ctx.expect_overlap(
        wheel_0,
        carriage,
        axes="y",
        elem_a="wooden_wheel",
        elem_b="axle_iron",
        min_overlap=0.050,
        name="wheel_0_hub_captured_by_iron_axle",
    )
    ctx.expect_overlap(
        wheel_1,
        carriage,
        axes="xz",
        elem_a="wooden_wheel",
        elem_b="axle_beam",
        min_overlap=0.090,
        name="wheel_1_centered_on_axle_beam",
    )
    ctx.expect_overlap(
        wheel_1,
        carriage,
        axes="y",
        elem_a="wooden_wheel",
        elem_b="axle_iron",
        min_overlap=0.050,
        name="wheel_1_hub_captured_by_iron_axle",
    )

    rest_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_tube")
    with ctx.pose({trunnion: 0.30}):
        raised_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_tube")

    if rest_aabb is not None and raised_aabb is not None:
        rest_z = float(rest_aabb[1][2])
        raised_z = float(raised_aabb[1][2])
        ctx.check(
            "positive_trunnion_elevates_muzzle",
            raised_z > rest_z + 0.25,
            details=f"rest_z={rest_z:.3f}, raised_z={raised_z:.3f}",
        )
    else:
        ctx.fail("positive_trunnion_elevates_muzzle", "Could not measure muzzle bore AABB.")

    return ctx.report()


object_model = build_object_model()
