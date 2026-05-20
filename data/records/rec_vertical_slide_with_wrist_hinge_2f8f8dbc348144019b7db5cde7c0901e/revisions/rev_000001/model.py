from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="z_axis_carriage_wrist")

    aluminum = model.material("clear_anodized_aluminum", rgba=(0.72, 0.74, 0.74, 1.0))
    dark_anodized = model.material("dark_anodized_carriage", rgba=(0.12, 0.14, 0.16, 1.0))
    rail_steel = model.material("polished_linear_rail_steel", rgba=(0.82, 0.84, 0.83, 1.0))
    bearing_black = model.material("black_bearing_blocks", rgba=(0.04, 0.045, 0.05, 1.0))
    blue_seal = model.material("blue_bearing_end_seals", rgba=(0.05, 0.18, 0.42, 1.0))
    wrist_steel = model.material("brushed_wrist_steel", rgba=(0.46, 0.48, 0.50, 1.0))
    amber = model.material("amber_index_mark", rgba=(1.0, 0.52, 0.08, 1.0))

    rail_frame = model.part("rail_frame")
    rail_frame.visual(
        Box((0.42, 0.22, 0.050)),
        origin=Origin(xyz=(0.0, -0.030, 0.025)),
        material=aluminum,
        name="bottom_plate",
    )
    rail_frame.visual(
        Box((0.42, 0.22, 0.050)),
        origin=Origin(xyz=(0.0, -0.030, 0.815)),
        material=aluminum,
        name="top_plate",
    )
    rail_frame.visual(
        Box((0.050, 0.050, 0.746)),
        origin=Origin(xyz=(0.0, -0.115, 0.420)),
        material=aluminum,
        name="rear_spine",
    )
    rail_frame.visual(
        Cylinder(radius=0.011, length=0.746),
        origin=Origin(xyz=(-0.130, 0.0, 0.420)),
        material=rail_steel,
        name="rail_0",
    )
    rail_frame.visual(
        Cylinder(radius=0.011, length=0.746),
        origin=Origin(xyz=(0.130, 0.0, 0.420)),
        material=rail_steel,
        name="rail_1",
    )
    rail_frame.visual(
        Cylinder(radius=0.006, length=0.746),
        origin=Origin(xyz=(0.0, -0.075, 0.420)),
        material=wrist_steel,
        name="center_screw",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.235, 0.020, 0.215)),
        origin=Origin(xyz=(0.0, -0.047, 0.0)),
        material=dark_anodized,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.110, 0.070, 0.085)),
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        material=dark_anodized,
        name="wrist_mount_block",
    )
    carriage.visual(
        Cylinder(radius=0.047, length=0.020),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="fixed_wrist_face",
    )

    for rail_x in (-0.130, 0.130):
        rail_index = 0 if rail_x < 0 else 1
        for block_z in (-0.075, 0.075):
            block_index = 0 if block_z < 0 else 1
            prefix = f"bearing_{rail_index}_{block_index}"
            carriage.visual(
                Box((0.064, 0.028, 0.052)),
                origin=Origin(xyz=(rail_x, -0.040, block_z)),
                material=bearing_black,
                name=f"{prefix}_rear_bridge",
            )
            carriage.visual(
                Box((0.012, 0.062, 0.052)),
                origin=Origin(xyz=(rail_x - 0.017, -0.004, block_z)),
                material=bearing_black,
                name=f"{prefix}_side_0",
            )
            carriage.visual(
                Box((0.012, 0.062, 0.052)),
                origin=Origin(xyz=(rail_x + 0.017, -0.004, block_z)),
                material=bearing_black,
                name=f"{prefix}_side_1",
            )
            carriage.visual(
                Box((0.052, 0.006, 0.058)),
                origin=Origin(xyz=(rail_x, 0.030, block_z)),
                material=blue_seal,
                name=f"{prefix}_front_seal",
            )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.030, length=0.052),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wrist_steel,
        name="wrist_shaft",
    )
    wrist.visual(
        Cylinder(radius=0.056, length=0.018),
        origin=Origin(xyz=(0.0, 0.061, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wrist_steel,
        name="output_flange",
    )
    wrist.visual(
        Box((0.122, 0.016, 0.040)),
        origin=Origin(xyz=(0.0, 0.078, 0.0)),
        material=wrist_steel,
        name="tool_bar",
    )
    wrist.visual(
        Box((0.024, 0.012, 0.020)),
        origin=Origin(xyz=(0.050, 0.092, 0.0)),
        material=amber,
        name="index_tab",
    )

    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=rail_frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.35, lower=-0.180, upper=0.180),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist,
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    wrist = object_model.get_part("wrist")
    z_slide = object_model.get_articulation("z_slide")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.check("z slide is prismatic", z_slide.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("wrist is revolute", wrist_roll.articulation_type == ArticulationType.REVOLUTE)

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({z_slide: 0.160}):
        raised_position = ctx.part_world_position(carriage)
    ctx.check(
        "carriage travels upward on the Z axis",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.150,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    rest_tab_aabb = ctx.part_element_world_aabb(wrist, elem="index_tab")
    with ctx.pose({wrist_roll: math.pi / 2.0}):
        rotated_tab_aabb = ctx.part_element_world_aabb(wrist, elem="index_tab")
    rest_tab_center_z = None if rest_tab_aabb is None else (rest_tab_aabb[0][2] + rest_tab_aabb[1][2]) / 2.0
    rotated_tab_center_z = (
        None
        if rotated_tab_aabb is None
        else (rotated_tab_aabb[0][2] + rotated_tab_aabb[1][2]) / 2.0
    )
    ctx.check(
        "wrist index tab sweeps around the roll axis",
        rest_tab_center_z is not None
        and rotated_tab_center_z is not None
        and rotated_tab_center_z < rest_tab_center_z - 0.035,
        details=f"rest={rest_tab_aabb}, rotated={rotated_tab_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
