from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _cylinder_x() -> Origin:
    """Rotate a URDF Z cylinder onto the local +X axis."""
    return Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _cylinder_y() -> Origin:
    """Rotate a URDF Z cylinder onto the local +Y axis."""
    return Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _torus_x_mesh(name: str, major: float, tube: float):
    torus = TorusGeometry(major, tube, radial_segments=24, tubular_segments=48)
    torus.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(torus, name)


def _torus_y_mesh(name: str, major: float, tube: float):
    torus = TorusGeometry(major, tube, radial_segments=24, tubular_segments=48)
    torus.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(torus, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saddle_body_multi_axis_wrist")

    cast_iron = model.material("dark_cast_housing", rgba=(0.12, 0.13, 0.14, 1.0))
    black = model.material("black_bearing_rubber", rgba=(0.015, 0.016, 0.018, 1.0))
    yaw_blue = model.material("blue_anodized_yaw", rgba=(0.08, 0.20, 0.42, 1.0))
    pitch_steel = model.material("brushed_pitch_steel", rgba=(0.52, 0.55, 0.56, 1.0))
    roll_silver = model.material("machined_roll_cartridge", rgba=(0.78, 0.75, 0.68, 1.0))
    amber = model.material("amber_index_mark", rgba=(0.95, 0.55, 0.12, 1.0))

    # Fixed grounded housing.  The broad foot and pedestal make the wrist read as
    # bolted to a machine frame rather than suspended in space.
    housing = model.part("housing")
    housing.visual(
        Box((0.58, 0.42, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=cast_iron,
        name="grounded_base",
    )
    housing.visual(
        Cylinder(radius=0.205, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=cast_iron,
        name="pedestal_cap",
    )
    housing.visual(
        Cylinder(radius=0.125, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=black,
        name="fixed_bearing_shadow",
    )
    housing.visual(
        mesh_from_geometry(TorusGeometry(0.160, 0.012, radial_segments=28, tubular_segments=56), "base_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=black,
        name="base_bearing_ring",
    )
    for i, (x, y) in enumerate(
        ((-0.235, -0.165), (-0.235, 0.165), (0.235, -0.165), (0.235, 0.165))
    ):
        housing.visual(
            Box((0.060, 0.050, 0.020)),
            origin=Origin(xyz=(x, y, 0.010)),
            material=black,
            name=f"mount_foot_{i}",
        )

    # Yaw stage: a turntable topped by a saddle/yoke.  Its frame sits exactly on
    # the vertical yaw axis at the top bearing plane.
    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.134, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=yaw_blue,
        name="yaw_bearing_plate",
    )
    yaw_stage.visual(
        Cylinder(radius=0.096, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=yaw_blue,
        name="yaw_neck",
    )
    yaw_stage.visual(
        Box((0.270, 0.305, 0.045)),
        origin=Origin(xyz=(0.095, 0.0, 0.0725)),
        material=yaw_blue,
        name="saddle_floor",
    )
    yaw_stage.visual(
        Box((0.105, 0.085, 0.155)),
        origin=Origin(xyz=(-0.015, 0.0, 0.145)),
        material=yaw_blue,
        name="rear_saddle_web",
    )
    for y, name in ((-0.125, "pitch_cheek_0"), (0.125, "pitch_cheek_1")):
        yaw_stage.visual(
            Box((0.145, 0.045, 0.240)),
            origin=Origin(xyz=(0.160, y, 0.170)),
            material=yaw_blue,
            name=name,
        )
        yaw_stage.visual(
            _torus_y_mesh(f"{name}_bearing_cup", 0.047, 0.010),
            origin=Origin(xyz=(0.160, 0.1025 if y > 0 else -0.1025, 0.170)),
            material=black,
            name=f"{name}_bearing_cup",
        )

    model.articulation(
        "housing_to_yaw",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )

    # Pitch stage: a crosswise trunnion captured between the yaw cheeks, then a
    # smaller saddle that carries the roll bearing.  Positive pitch lifts the
    # output nose upward.
    pitch_support = model.part("pitch_support")
    pitch_support.visual(
        Cylinder(radius=0.032, length=0.205),
        origin=_cylinder_y(),
        material=pitch_steel,
        name="pitch_trunnion",
    )
    pitch_support.visual(
        Box((0.150, 0.092, 0.082)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=pitch_steel,
        name="pitch_saddle_body",
    )
    pitch_support.visual(
        Cylinder(radius=0.035, length=0.118),
        origin=Origin(xyz=(0.142, 0.0, 0.0), rpy=_cylinder_x().rpy),
        material=pitch_steel,
        name="tapered_roll_neck",
    )
    for y, name in ((-0.058, "roll_side_rail_0"), (0.058, "roll_side_rail_1")):
        pitch_support.visual(
            Box((0.172, 0.024, 0.064)),
            origin=Origin(xyz=(0.165, y, 0.0)),
            material=pitch_steel,
            name=name,
        )
    pitch_support.visual(
        _torus_x_mesh("roll_bearing_ring", 0.058, 0.012),
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        material=black,
        name="roll_bearing_ring",
    )
    pitch_support.visual(
        Box((0.080, 0.030, 0.014)),
        origin=Origin(xyz=(0.195, 0.0, 0.055)),
        material=pitch_steel,
        name="top_roll_bridge",
    )
    pitch_support.visual(
        Box((0.060, 0.030, 0.010)),
        origin=Origin(xyz=(0.225, 0.0, 0.043)),
        material=black,
        name="top_roll_guide",
    )
    for y, name in ((-0.043, "roll_guide_0"), (0.043, "roll_guide_1")):
        pitch_support.visual(
            Box((0.060, 0.010, 0.026)),
            origin=Origin(xyz=(0.225, y, 0.0)),
            material=black,
            name=name,
        )

    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_support,
        origin=Origin(xyz=(0.160, 0.0, 0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=-0.85, upper=0.85),
    )

    # Roll cartridge: compact output module, smaller than the upstream saddle.
    # The small amber key makes roll motion visible despite the cylindrical body.
    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        Cylinder(radius=0.038, length=0.160),
        origin=Origin(xyz=(0.063, 0.0, 0.0), rpy=_cylinder_x().rpy),
        material=roll_silver,
        name="roll_barrel",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(0.156, 0.0, 0.0), rpy=_cylinder_x().rpy),
        material=roll_silver,
        name="output_flange",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.022, length=0.094),
        origin=Origin(xyz=(0.218, 0.0, 0.0), rpy=_cylinder_x().rpy),
        material=roll_silver,
        name="output_spigot",
    )
    roll_cartridge.visual(
        Box((0.052, 0.014, 0.014)),
        origin=Origin(xyz=(0.105, 0.0, 0.044)),
        material=amber,
        name="roll_index_key",
    )

    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_support,
        child=roll_cartridge,
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_support = object_model.get_part("pitch_support")
    roll_cartridge = object_model.get_part("roll_cartridge")
    yaw = object_model.get_articulation("housing_to_yaw")
    pitch = object_model.get_articulation("yaw_to_pitch")
    roll = object_model.get_articulation("pitch_to_roll")

    ctx.check(
        "three serial revolute wrist joints",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and pitch.articulation_type == ArticulationType.REVOLUTE
        and roll.articulation_type == ArticulationType.REVOLUTE,
        details=f"types={[yaw.articulation_type, pitch.articulation_type, roll.articulation_type]}",
    )
    ctx.check(
        "yaw pitch roll axes are vertical crosswise longitudinal",
        yaw.axis == (0.0, 0.0, 1.0)
        and pitch.axis == (0.0, -1.0, 0.0)
        and roll.axis == (1.0, 0.0, 0.0),
        details=f"axes={[yaw.axis, pitch.axis, roll.axis]}",
    )
    ctx.expect_gap(
        yaw_stage,
        housing,
        axis="z",
        positive_elem="yaw_bearing_plate",
        negative_elem="pedestal_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="yaw bearing plate is seated on the grounded pedestal",
    )
    ctx.expect_overlap(
        yaw_stage,
        housing,
        axes="xy",
        elem_a="yaw_bearing_plate",
        elem_b="pedestal_cap",
        min_overlap=0.18,
        name="yaw bearing footprint sits inside pedestal",
    )
    ctx.expect_within(
        roll_cartridge,
        pitch_support,
        axes="yz",
        inner_elem="roll_barrel",
        outer_elem="roll_bearing_ring",
        margin=0.004,
        name="roll barrel is centered inside the bearing ring clearance",
    )

    rest_pitch_position = ctx.part_world_position(pitch_support)
    with ctx.pose({yaw: 0.60}):
        yawed_pitch_position = ctx.part_world_position(pitch_support)
    ctx.check(
        "yaw motion sweeps pitch stage about the vertical axis",
        rest_pitch_position is not None
        and yawed_pitch_position is not None
        and yawed_pitch_position[1] > rest_pitch_position[1] + 0.05,
        details=f"rest={rest_pitch_position}, yawed={yawed_pitch_position}",
    )

    rest_roll_position = ctx.part_world_position(roll_cartridge)
    with ctx.pose({pitch: 0.55}):
        pitched_roll_position = ctx.part_world_position(roll_cartridge)
    ctx.check(
        "positive pitch raises the compact roll cartridge",
        rest_roll_position is not None
        and pitched_roll_position is not None
        and pitched_roll_position[2] > rest_roll_position[2] + 0.08,
        details=f"rest={rest_roll_position}, pitched={pitched_roll_position}",
    )

    key_aabb_rest = ctx.part_element_world_aabb(roll_cartridge, elem="roll_index_key")
    with ctx.pose({roll: 0.75}):
        key_aabb_rolled = ctx.part_element_world_aabb(roll_cartridge, elem="roll_index_key")
    if key_aabb_rest is not None and key_aabb_rolled is not None:
        rest_center_y = 0.5 * (key_aabb_rest[0][1] + key_aabb_rest[1][1])
        rolled_center_y = 0.5 * (key_aabb_rolled[0][1] + key_aabb_rolled[1][1])
        rest_center_z = 0.5 * (key_aabb_rest[0][2] + key_aabb_rest[1][2])
        rolled_center_z = 0.5 * (key_aabb_rolled[0][2] + key_aabb_rolled[1][2])
        roll_motion_ok = abs(rolled_center_y - rest_center_y) > 0.020 and rolled_center_z < rest_center_z
    else:
        roll_motion_ok = False
    ctx.check(
        "roll key orbits around the longitudinal output axis",
        roll_motion_ok,
        details=f"rest={key_aabb_rest}, rolled={key_aabb_rolled}",
    )

    return ctx.report()


object_model = build_object_model()
