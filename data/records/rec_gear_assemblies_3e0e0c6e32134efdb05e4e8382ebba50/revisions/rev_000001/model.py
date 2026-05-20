from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sqrt

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_TOP = 0.020
SHAFT_RADIUS = 0.009
SHAFT_LENGTH = 0.300
SUPPORT_Y = 0.120
BLOCK_LENGTH = 0.050
BLOCK_WIDTH = 0.056
BLOCK_HOUSING_RADIUS = 0.024

INPUT_AXIS = (-0.100, 0.0, 0.170)
COUNTER_AXIS = (0.000, 0.0, 0.110)
OUTPUT_AXIS = (0.100, 0.0, 0.170)


def _centered_extrude_xz(workplane: cq.Workplane, length: float) -> cq.Workplane:
    return workplane.extrude(length).translate((0.0, -length / 2.0, 0.0))


def make_open_frame() -> cq.Workplane:
    rail_width = 0.050
    rail_length = 0.285
    cross_length = 0.255
    cross_width = 0.042
    feet_size = (0.070, 0.050, 0.014)
    foot_drop = feet_size[2] / 2.0

    members = [
        cq.Workplane("XY").box(rail_width, rail_length, FRAME_TOP).translate((-0.100, 0.0, FRAME_TOP / 2.0)),
        cq.Workplane("XY").box(rail_width, rail_length, FRAME_TOP).translate((0.000, 0.0, FRAME_TOP / 2.0)),
        cq.Workplane("XY").box(rail_width, rail_length, FRAME_TOP).translate((0.100, 0.0, FRAME_TOP / 2.0)),
        cq.Workplane("XY").box(cross_length, cross_width, FRAME_TOP).translate((0.000, -0.132, FRAME_TOP / 2.0)),
        cq.Workplane("XY").box(cross_length, cross_width, FRAME_TOP).translate((0.000, 0.132, FRAME_TOP / 2.0)),
        cq.Workplane("XY").box(0.180, 0.034, FRAME_TOP).translate((0.000, 0.000, FRAME_TOP / 2.0)),
        cq.Workplane("XY").box(*feet_size).translate((-0.125, -0.132, foot_drop)),
        cq.Workplane("XY").box(*feet_size).translate((0.125, -0.132, foot_drop)),
        cq.Workplane("XY").box(*feet_size).translate((-0.125, 0.132, foot_drop)),
        cq.Workplane("XY").box(*feet_size).translate((0.125, 0.132, foot_drop)),
    ]

    frame = members[0]
    for member in members[1:]:
        frame = frame.union(member)
    return frame


def make_bearing_block(axis_z: float) -> cq.Workplane:
    drop = axis_z - FRAME_TOP
    cheek_height = BLOCK_HOUSING_RADIUS * 1.15
    slot_half_width = SHAFT_RADIUS * 1.25
    saddle_floor = -SHAFT_RADIUS
    profile = cq.Workplane("XZ").polyline(
        [
            (-BLOCK_WIDTH / 2.0, -drop),
            (-BLOCK_WIDTH / 2.0, cheek_height),
            (-slot_half_width, cheek_height),
            (-slot_half_width, saddle_floor),
            (slot_half_width, saddle_floor),
            (slot_half_width, cheek_height),
            (BLOCK_WIDTH / 2.0, cheek_height),
            (BLOCK_WIDTH / 2.0, -drop),
        ]
    ).close()
    return _centered_extrude_xz(profile, BLOCK_LENGTH)


def make_spur_gear(
    *,
    root_radius: float,
    tip_radius: float,
    width: float,
    tooth_count: int,
    hub_radius: float,
    hub_length: float,
) -> cq.Workplane:
    rim = _centered_extrude_xz(cq.Workplane("XZ").circle(root_radius), width)
    web = _centered_extrude_xz(cq.Workplane("XZ").circle(root_radius * 0.72), width * 0.78)
    hub = _centered_extrude_xz(cq.Workplane("XZ").circle(hub_radius), hub_length)
    bore = _centered_extrude_xz(cq.Workplane("XZ").circle(SHAFT_RADIUS * 1.04), hub_length + 0.004)

    tooth_depth = (tip_radius - root_radius) + 0.006
    tooth_width = max(root_radius * 0.18, width * 0.60)
    tooth_mid_radius = root_radius + tooth_depth / 2.0 - 0.003

    gear = rim.union(web).union(hub)
    for tooth_index in range(tooth_count):
        angle_deg = tooth_index * 360.0 / tooth_count
        tooth = (
            cq.Workplane("XY")
            .box(tooth_depth, width, tooth_width)
            .translate((tooth_mid_radius, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)
        )
        gear = gear.union(tooth)

    return gear.cut(bore)


def add_bearing_block(
    model: ArticulatedObject,
    frame,
    *,
    name: str,
    xyz: tuple[float, float, float],
    block_shape: cq.Workplane,
    material: str,
) -> None:
    block = model.part(name)
    block.visual(
        mesh_from_cadquery(block_shape, f"{name}_mesh"),
        material=material,
        name="block_body",
    )
    model.articulation(
        f"frame_to_{name}",
        ArticulationType.FIXED,
        parent=frame,
        child=block,
        origin=Origin(xyz=xyz),
    )


def add_rotating_shaft(
    model: ArticulatedObject,
    frame,
    *,
    name: str,
    axis_xyz: tuple[float, float, float],
    gear_shape: cq.Workplane,
) -> None:
    shaft = model.part(name)
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="shaft_steel",
        name="shaft_body",
    )
    shaft.visual(
        mesh_from_cadquery(gear_shape, f"{name}_gear_mesh"),
        material="gear_bronze",
        name="gear_body",
    )

    model.articulation(
        f"frame_to_{name}",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=axis_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=12.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_transfer_gearbox")

    model.material("frame_blue", rgba=(0.17, 0.24, 0.38, 1.0))
    model.material("bearing_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("shaft_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("gear_bronze", rgba=(0.74, 0.62, 0.31, 1.0))

    frame = model.part("base_frame")
    frame.visual(
        mesh_from_cadquery(make_open_frame(), "base_frame_mesh"),
        material="frame_blue",
        name="frame_body",
    )

    outer_block_shape = make_bearing_block(INPUT_AXIS[2])
    counter_block_shape = make_bearing_block(COUNTER_AXIS[2])

    add_bearing_block(
        model,
        frame,
        name="input_front_block",
        xyz=(INPUT_AXIS[0], -SUPPORT_Y, INPUT_AXIS[2]),
        block_shape=outer_block_shape,
        material="bearing_gray",
    )
    add_bearing_block(
        model,
        frame,
        name="input_rear_block",
        xyz=(INPUT_AXIS[0], SUPPORT_Y, INPUT_AXIS[2]),
        block_shape=outer_block_shape,
        material="bearing_gray",
    )
    add_bearing_block(
        model,
        frame,
        name="counter_front_block",
        xyz=(COUNTER_AXIS[0], -SUPPORT_Y, COUNTER_AXIS[2]),
        block_shape=counter_block_shape,
        material="bearing_gray",
    )
    add_bearing_block(
        model,
        frame,
        name="counter_rear_block",
        xyz=(COUNTER_AXIS[0], SUPPORT_Y, COUNTER_AXIS[2]),
        block_shape=counter_block_shape,
        material="bearing_gray",
    )
    add_bearing_block(
        model,
        frame,
        name="output_front_block",
        xyz=(OUTPUT_AXIS[0], -SUPPORT_Y, OUTPUT_AXIS[2]),
        block_shape=outer_block_shape,
        material="bearing_gray",
    )
    add_bearing_block(
        model,
        frame,
        name="output_rear_block",
        xyz=(OUTPUT_AXIS[0], SUPPORT_Y, OUTPUT_AXIS[2]),
        block_shape=outer_block_shape,
        material="bearing_gray",
    )

    input_gear = make_spur_gear(
        root_radius=0.034,
        tip_radius=0.042,
        width=0.024,
        tooth_count=18,
        hub_radius=0.018,
        hub_length=0.040,
    )
    counter_gear = make_spur_gear(
        root_radius=0.054,
        tip_radius=0.064,
        width=0.030,
        tooth_count=24,
        hub_radius=0.022,
        hub_length=0.048,
    )
    output_gear = make_spur_gear(
        root_radius=0.036,
        tip_radius=0.044,
        width=0.024,
        tooth_count=18,
        hub_radius=0.018,
        hub_length=0.040,
    )

    add_rotating_shaft(
        model,
        frame,
        name="input_shaft",
        axis_xyz=INPUT_AXIS,
        gear_shape=input_gear,
    )
    add_rotating_shaft(
        model,
        frame,
        name="countershaft",
        axis_xyz=COUNTER_AXIS,
        gear_shape=counter_gear,
    )
    add_rotating_shaft(
        model,
        frame,
        name="output_shaft",
        axis_xyz=OUTPUT_AXIS,
        gear_shape=output_gear,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    input_shaft = object_model.get_part("input_shaft")
    countershaft = object_model.get_part("countershaft")
    output_shaft = object_model.get_part("output_shaft")

    input_front = object_model.get_part("input_front_block")
    input_rear = object_model.get_part("input_rear_block")
    counter_front = object_model.get_part("counter_front_block")
    counter_rear = object_model.get_part("counter_rear_block")
    output_front = object_model.get_part("output_front_block")
    output_rear = object_model.get_part("output_rear_block")

    input_joint = object_model.get_articulation("frame_to_input_shaft")
    counter_joint = object_model.get_articulation("frame_to_countershaft")
    output_joint = object_model.get_articulation("frame_to_output_shaft")

    for joint in (input_joint, counter_joint, output_joint):
        ctx.check(
            f"{joint.name} uses a supported shaft axis",
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_overlap(
        input_shaft,
        input_front,
        axes="xz",
        min_overlap=0.017,
        name="input shaft aligns with front bearing block",
    )
    ctx.expect_overlap(
        input_shaft,
        input_rear,
        axes="xz",
        min_overlap=0.017,
        name="input shaft aligns with rear bearing block",
    )
    ctx.expect_overlap(
        countershaft,
        counter_front,
        axes="xz",
        min_overlap=0.017,
        name="countershaft aligns with front bearing block",
    )
    ctx.expect_overlap(
        countershaft,
        counter_rear,
        axes="xz",
        min_overlap=0.017,
        name="countershaft aligns with rear bearing block",
    )
    ctx.expect_overlap(
        output_shaft,
        output_front,
        axes="xz",
        min_overlap=0.017,
        name="output shaft aligns with front bearing block",
    )
    ctx.expect_overlap(
        output_shaft,
        output_rear,
        axes="xz",
        min_overlap=0.017,
        name="output shaft aligns with rear bearing block",
    )

    input_center_distance = sqrt((INPUT_AXIS[0] - COUNTER_AXIS[0]) ** 2 + (INPUT_AXIS[2] - COUNTER_AXIS[2]) ** 2)
    output_center_distance = sqrt((OUTPUT_AXIS[0] - COUNTER_AXIS[0]) ** 2 + (OUTPUT_AXIS[2] - COUNTER_AXIS[2]) ** 2)
    ctx.expect_origin_distance(
        input_shaft,
        countershaft,
        axes="xz",
        min_dist=input_center_distance - 0.002,
        max_dist=input_center_distance + 0.002,
        name="input shaft sits at the intended mesh distance from the countershaft",
    )
    ctx.expect_origin_distance(
        output_shaft,
        countershaft,
        axes="xz",
        min_dist=output_center_distance - 0.002,
        max_dist=output_center_distance + 0.002,
        name="output shaft sits at the intended mesh distance from the countershaft",
    )

    rest_input = ctx.part_world_position(input_shaft)
    rest_counter = ctx.part_world_position(countershaft)
    with ctx.pose({input_joint: 1.1, counter_joint: -0.8, output_joint: 0.9}):
        turned_input = ctx.part_world_position(input_shaft)
        turned_counter = ctx.part_world_position(countershaft)

    ctx.check(
        "input shaft rotates about a fixed supported axis",
        rest_input is not None and turned_input is not None and max(abs(a - b) for a, b in zip(rest_input, turned_input)) < 1e-6,
        details=f"rest={rest_input}, turned={turned_input}",
    )
    ctx.check(
        "countershaft rotates about a fixed supported axis",
        rest_counter is not None and turned_counter is not None and max(abs(a - b) for a, b in zip(rest_counter, turned_counter)) < 1e-6,
        details=f"rest={rest_counter}, turned={turned_counter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
