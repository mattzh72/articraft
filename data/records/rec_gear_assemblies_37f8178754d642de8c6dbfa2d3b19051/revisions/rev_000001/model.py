from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


SHAFT_Z = 0.32
SHAFT_RADIUS = 0.014
SHAFT_LENGTH = 0.66
GEAR_MODULE = 0.006
PINION_TEETH = 18
WHEEL_TEETH = 54
PINION_OUTER_RADIUS = GEAR_MODULE * (PINION_TEETH + 2) / 2.0
WHEEL_OUTER_RADIUS = GEAR_MODULE * (WHEEL_TEETH + 2) / 2.0
GEAR_TIP_CLEARANCE = 0.003
STAGE_CENTER_DISTANCE = PINION_OUTER_RADIUS + WHEEL_OUTER_RADIUS + GEAR_TIP_CLEARANCE
INPUT_X = -0.24
INTERMEDIATE_X = INPUT_X + STAGE_CENTER_DISTANCE
OUTPUT_X = INTERMEDIATE_X + STAGE_CENTER_DISTANCE
STAGE_1_Y = -0.10
STAGE_2_Y = 0.10


def _shaft_origin(y: float = 0.0) -> Origin:
    return Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _gear_mesh(name: str, teeth: int, width: float, *, large: bool):
    gear = SpurGear(
        module=GEAR_MODULE,
        teeth_number=teeth,
        width=width,
        backlash=0.00035,
    )
    if large:
        shape = gear.build(
            hub_d=0.070,
            hub_length=0.062,
            n_spokes=6,
            spoke_width=0.014,
            spokes_id=0.080,
            spokes_od=0.235,
            chamfer=0.0012,
        )
    else:
        shape = gear.build(
            hub_d=0.052,
            hub_length=0.058,
            chamfer=0.001,
        )
    return mesh_from_cadquery(
        shape,
        name,
        tolerance=0.0007,
        angular_tolerance=0.08,
    )


def _add_shaft_visuals(part, shaft_material, collar_material) -> None:
    part.visual(
        Cylinder(SHAFT_RADIUS, SHAFT_LENGTH),
        origin=_shaft_origin(),
        material=shaft_material,
        name="shaft",
    )
    for y, name in ((-0.185, "front_collar"), (0.185, "rear_collar")):
        part.visual(
            Cylinder(0.023, 0.024),
            origin=_shaft_origin(y),
            material=collar_material,
            name=name,
        )


def _add_frame_side(
    frame,
    *,
    y: float,
    prefix: str,
    frame_material,
    bearing_material,
    marker_material,
) -> None:
    x_center = (INPUT_X + OUTPUT_X) / 2.0
    width = (OUTPUT_X - INPUT_X) + 0.30
    side_thickness = 0.034
    rail_height = 0.050
    rail_z_low = 0.045
    rail_z_high = 0.585
    post_height = rail_z_high - 0.020
    frame.visual(
        Box((width, side_thickness, rail_height)),
        origin=Origin(xyz=(x_center, y, rail_z_low)),
        material=frame_material,
        name=f"{prefix}_lower_rail",
    )
    frame.visual(
        Box((width, side_thickness, rail_height)),
        origin=Origin(xyz=(x_center, y, rail_z_high)),
        material=frame_material,
        name=f"{prefix}_upper_rail",
    )
    for x, name, post_width in (
        (INPUT_X - 0.15, "end_post_0", 0.040),
        ((INPUT_X + INTERMEDIATE_X) / 2.0, "stage_post_0", 0.030),
        ((INTERMEDIATE_X + OUTPUT_X) / 2.0, "stage_post_1", 0.030),
        (OUTPUT_X + 0.15, "end_post_1", 0.040),
    ):
        frame.visual(
            Box((post_width, side_thickness, post_height)),
            origin=Origin(xyz=(x, y, 0.295)),
            material=frame_material,
            name=f"{prefix}_{name}",
        )

    bearing_mesh = mesh_from_geometry(
        TorusGeometry(0.032, 0.010, radial_segments=36, tubular_segments=12),
        f"{prefix}_bearing_ring_mesh",
    )
    for shaft_name, x in (
        ("input", INPUT_X),
        ("intermediate", INTERMEDIATE_X),
        ("output", OUTPUT_X),
    ):
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(x, y, SHAFT_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bearing_material,
            name=f"{prefix}_{shaft_name}_bearing",
        )
        frame.visual(
            Box((0.018, side_thickness, 0.008)),
            origin=Origin(xyz=(x, y, SHAFT_Z + SHAFT_RADIUS + 0.004)),
            material=bearing_material,
            name=f"{prefix}_{shaft_name}_upper_shoe",
        )
        frame.visual(
            Box((0.012, side_thickness, 0.026)),
            origin=Origin(xyz=(x, y, SHAFT_Z + SHAFT_RADIUS + 0.021)),
            material=frame_material,
            name=f"{prefix}_{shaft_name}_upper_cap",
        )
        frame.visual(
            Box((0.018, side_thickness, 0.008)),
            origin=Origin(xyz=(x, y, SHAFT_Z - SHAFT_RADIUS - 0.004)),
            material=bearing_material,
            name=f"{prefix}_{shaft_name}_lower_shoe",
        )
        frame.visual(
            Box((0.012, side_thickness, 0.026)),
            origin=Origin(xyz=(x, y, SHAFT_Z - SHAFT_RADIUS - 0.021)),
            material=frame_material,
            name=f"{prefix}_{shaft_name}_lower_cap",
        )
        frame.visual(
            Box((0.044, side_thickness, 0.220)),
            origin=Origin(xyz=(x, y, 0.463)),
            material=frame_material,
            name=f"{prefix}_{shaft_name}_upper_web",
        )
        frame.visual(
            Box((0.044, side_thickness, 0.220)),
            origin=Origin(xyz=(x, y, 0.177)),
            material=frame_material,
            name=f"{prefix}_{shaft_name}_lower_web",
        )
        frame.visual(
            Box((0.006, side_thickness + 0.004, 0.070)),
            origin=Origin(xyz=(x, y, 0.078)),
            material=marker_material,
            name=f"{prefix}_{shaft_name}_centerline",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_spur_reduction_rig")

    dark_steel = model.material("dark_steel", color=(0.15, 0.16, 0.17, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.58, 0.60, 0.58, 1.0))
    brass = model.material("machined_brass", color=(0.92, 0.66, 0.24, 1.0))
    frame_blue = model.material("painted_blue_frame", color=(0.08, 0.18, 0.33, 1.0))
    bearing_black = model.material("black_bearing", color=(0.02, 0.022, 0.025, 1.0))
    amber = model.material("spacing_marker", color=(1.0, 0.58, 0.06, 1.0))

    frame = model.part("frame")
    frame_width = (OUTPUT_X - INPUT_X) + 0.38
    frame_center_x = (INPUT_X + OUTPUT_X) / 2.0
    frame.visual(
        Box((frame_width, 0.62, 0.020)),
        origin=Origin(xyz=(frame_center_x, 0.0, 0.010)),
        material=frame_blue,
        name="base_plate",
    )
    for y, prefix in ((-0.245, "front"), (0.245, "rear")):
        _add_frame_side(
            frame,
            y=y,
            prefix=prefix,
            frame_material=frame_blue,
            bearing_material=bearing_black,
            marker_material=amber,
        )
    for center_x, length, name in (
        ((INPUT_X + INTERMEDIATE_X) / 2.0, STAGE_CENTER_DISTANCE, "stage1_spacing"),
        ((INTERMEDIATE_X + OUTPUT_X) / 2.0, STAGE_CENTER_DISTANCE, "stage2_spacing"),
    ):
        frame.visual(
            Box((length, 0.018, 0.010)),
            origin=Origin(xyz=(center_x, -0.292, 0.024)),
            material=amber,
            name=name,
        )

    input_shaft = model.part("input_shaft")
    _add_shaft_visuals(input_shaft, brushed_steel, dark_steel)
    input_shaft.visual(
        _gear_mesh("input_pinion_mesh", PINION_TEETH, 0.045, large=False),
        origin=Origin(xyz=(0.0, STAGE_1_Y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="input_pinion",
    )

    intermediate_shaft = model.part("intermediate_shaft")
    _add_shaft_visuals(intermediate_shaft, brushed_steel, dark_steel)
    intermediate_shaft.visual(
        _gear_mesh("stage1_wheel_mesh", WHEEL_TEETH, 0.045, large=True),
        origin=Origin(xyz=(0.0, STAGE_1_Y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="stage1_wheel",
    )
    intermediate_shaft.visual(
        _gear_mesh("stage2_pinion_mesh", PINION_TEETH, 0.045, large=False),
        origin=Origin(xyz=(0.0, STAGE_2_Y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="stage2_pinion",
    )

    output_shaft = model.part("output_shaft")
    _add_shaft_visuals(output_shaft, brushed_steel, dark_steel)
    output_shaft.visual(
        _gear_mesh("output_wheel_mesh", WHEEL_TEETH, 0.045, large=True),
        origin=Origin(xyz=(0.0, STAGE_2_Y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="output_wheel",
    )
    output_shaft.visual(
        Cylinder(0.032, 0.055),
        origin=_shaft_origin(0.315),
        material=dark_steel,
        name="output_coupling",
    )

    input_joint = model.articulation(
        "input_rotation",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=input_shaft,
        origin=Origin(xyz=(INPUT_X, 0.0, SHAFT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0, lower=-math.tau, upper=math.tau),
    )
    model.articulation(
        "intermediate_rotation",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=intermediate_shaft,
        origin=Origin(xyz=(INTERMEDIATE_X, 0.0, SHAFT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=10.0, lower=-math.tau, upper=math.tau),
        mimic=Mimic(joint=input_joint.name, multiplier=-(PINION_TEETH / WHEEL_TEETH)),
    )
    model.articulation(
        "output_rotation",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(OUTPUT_X, 0.0, SHAFT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=5.0, lower=-math.tau, upper=math.tau),
        mimic=Mimic(
            joint=input_joint.name,
            multiplier=(PINION_TEETH / WHEEL_TEETH) * (PINION_TEETH / WHEEL_TEETH),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    input_shaft = object_model.get_part("input_shaft")
    intermediate_shaft = object_model.get_part("intermediate_shaft")
    output_shaft = object_model.get_part("output_shaft")
    input_joint = object_model.get_articulation("input_rotation")
    intermediate_joint = object_model.get_articulation("intermediate_rotation")
    output_joint = object_model.get_articulation("output_rotation")

    for joint, x, name in (
        (input_joint, INPUT_X, "input"),
        (intermediate_joint, INTERMEDIATE_X, "intermediate"),
        (output_joint, OUTPUT_X, "output"),
    ):
        ctx.check(
            f"{name} shaft joint is on its bearing axis",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0)
            and abs(joint.origin.xyz[0] - x) < 1e-6
            and abs(joint.origin.xyz[2] - SHAFT_Z) < 1e-6,
            details=f"axis={joint.axis}, origin={joint.origin.xyz}",
        )

    ctx.expect_origin_gap(
        intermediate_shaft,
        input_shaft,
        axis="x",
        min_gap=STAGE_CENTER_DISTANCE - 0.001,
        max_gap=STAGE_CENTER_DISTANCE + 0.001,
        name="stage 1 shaft spacing is explicit",
    )
    ctx.expect_origin_gap(
        output_shaft,
        intermediate_shaft,
        axis="x",
        min_gap=STAGE_CENTER_DISTANCE - 0.001,
        max_gap=STAGE_CENTER_DISTANCE + 0.001,
        name="stage 2 shaft spacing is explicit",
    )
    ctx.expect_gap(
        intermediate_shaft,
        input_shaft,
        axis="x",
        positive_elem="stage1_wheel",
        negative_elem="input_pinion",
        min_gap=0.001,
        max_gap=0.018,
        name="stage 1 gear teeth have visible running clearance",
    )
    ctx.expect_gap(
        output_shaft,
        intermediate_shaft,
        axis="x",
        positive_elem="output_wheel",
        negative_elem="stage2_pinion",
        min_gap=0.001,
        max_gap=0.018,
        name="stage 2 gear teeth have visible running clearance",
    )
    for shaft, shaft_name in (
        (input_shaft, "input"),
        (intermediate_shaft, "intermediate"),
        (output_shaft, "output"),
    ):
        ctx.expect_overlap(
            shaft,
            frame,
            axes="yz",
            elem_a="shaft",
            elem_b=f"front_{shaft_name}_bearing",
            min_overlap=0.020,
            name=f"{shaft_name} shaft passes through front bearing",
        )
        ctx.expect_overlap(
            shaft,
            frame,
            axes="yz",
            elem_a="shaft",
            elem_b=f"rear_{shaft_name}_bearing",
            min_overlap=0.020,
            name=f"{shaft_name} shaft passes through rear bearing",
        )

    rest_output = ctx.part_world_position(output_shaft)
    with ctx.pose({input_joint: math.pi / 2.0}):
        turned_output = ctx.part_world_position(output_shaft)
        ctx.expect_gap(
            intermediate_shaft,
            input_shaft,
            axis="x",
            positive_elem="stage1_wheel",
            negative_elem="input_pinion",
            min_gap=0.001,
            max_gap=0.018,
            name="stage 1 clearance remains after input turn",
        )
        ctx.expect_gap(
            output_shaft,
            intermediate_shaft,
            axis="x",
            positive_elem="output_wheel",
            negative_elem="stage2_pinion",
            min_gap=0.001,
            max_gap=0.018,
            name="stage 2 clearance remains after input turn",
        )
    ctx.check(
        "output shaft remains on its fixed support axis while geared",
        rest_output is not None and turned_output is not None and rest_output == turned_output,
        details=f"rest={rest_output}, turned={turned_output}",
    )

    return ctx.report()


object_model = build_object_model()
