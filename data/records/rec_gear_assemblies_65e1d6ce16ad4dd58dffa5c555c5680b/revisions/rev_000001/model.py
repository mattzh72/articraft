from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BevelGearPair,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """A centered hollow bearing sleeve with its axis along local +Z."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0012, angular_tolerance=0.16)


def _centered_spur_mesh(
    module: float,
    teeth: int,
    width: float,
    bore: float,
    name: str,
):
    """A spur gear mesh centered on local origin, axis along local +Z."""
    gear = SpurGear(module=module, teeth_number=teeth, width=width, backlash=0.0012)
    shape = gear.build(bore_d=bore, chamfer=0.001).translate((0.0, 0.0, -width / 2.0))
    return mesh_from_cadquery(shape, name, tolerance=0.0012, angular_tolerance=0.14)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bevel_to_spur_transfer")

    cast_iron = model.material("painted_cast_iron", rgba=(0.12, 0.14, 0.15, 1.0))
    dark_trim = model.material("dark_bearing_caps", rgba=(0.025, 0.027, 0.03, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    brass = model.material("cut_brass_gears", rgba=(0.88, 0.62, 0.20, 1.0))
    oil_blue = model.material("blue_output_gear", rgba=(0.13, 0.25, 0.46, 1.0))

    # The bevel-pair helper's pinion axis sits about this far above the
    # vertical gear coordinate origin for the chosen 18/18, 45-degree pair.
    bevel_axis_height = 0.0585
    bevel_origin = (0.16, 0.0, 0.205)
    shaft_z = bevel_origin[2] + bevel_axis_height
    spur_x = bevel_origin[0] + 0.34
    output_y = 0.127

    base = model.part("base")
    base.visual(
        Box((0.86, 0.38, 0.04)),
        origin=Origin(xyz=(0.36, 0.07, 0.02)),
        material=cast_iron,
        name="base_plate",
    )
    # Vertical input-shaft support: a flanged lower bearing bolted to the base.
    base.visual(
        _tube_mesh(0.036, 0.014, 0.052, "lower_vertical_bearing_mesh"),
        origin=Origin(xyz=(bevel_origin[0], bevel_origin[1], 0.066)),
        material=dark_trim,
        name="lower_vertical_bearing",
    )

    # Two pillow-blocks support the horizontal transfer shaft.
    for x, pedestal_name, bearing_name in (
        (bevel_origin[0] + 0.12, "transfer_pedestal_0", "transfer_bearing_0"),
        (bevel_origin[0] + 0.50, "transfer_pedestal_1", "transfer_bearing_1"),
    ):
        base.visual(
            Box((0.052, 0.066, shaft_z - 0.078)),
            origin=Origin(xyz=(x, 0.0, 0.04 + (shaft_z - 0.078) / 2.0)),
            material=cast_iron,
            name=pedestal_name,
        )
        base.visual(
            _tube_mesh(0.039, 0.0155, 0.040, f"{bearing_name}_mesh"),
            origin=Origin(xyz=(x, 0.0, shaft_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_trim,
            name=bearing_name,
        )

    # The output shaft is parallel to the transfer shaft and carries the driven
    # spur gear, so it gets its own separate bearing pair.
    for x, pedestal_name, bearing_name in (
        (spur_x - 0.13, "output_pedestal_0", "output_bearing_0"),
        (spur_x + 0.14, "output_pedestal_1", "output_bearing_1"),
    ):
        base.visual(
            Box((0.052, 0.066, shaft_z - 0.078)),
            origin=Origin(xyz=(x, output_y, 0.04 + (shaft_z - 0.078) / 2.0)),
            material=cast_iron,
            name=pedestal_name,
        )
        base.visual(
            _tube_mesh(0.039, 0.0155, 0.040, f"{bearing_name}_mesh"),
            origin=Origin(xyz=(x, output_y, shaft_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_trim,
            name=bearing_name,
        )

    bevel_pair = BevelGearPair(
        module=0.006,
        gear_teeth=18,
        pinion_teeth=18,
        face_width=0.040,
        axis_angle=90.0,
        backlash=0.0015,
    )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        Cylinder(radius=0.010, length=0.194),
        origin=Origin(xyz=(0.0, 0.0, -0.067)),
        material=steel,
        name="input_shaft_bar",
    )
    input_shaft.visual(
        mesh_from_cadquery(
            bevel_pair.assemble(build_pinion=False, gear_build_args={"bore_d": 0.014}),
            "input_bevel_gear",
            tolerance=0.0012,
            angular_tolerance=0.14,
        ),
        material=brass,
        name="input_bevel",
    )
    input_shaft.visual(
        Cylinder(radius=0.021, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=steel,
        name="input_hub",
    )
    input_shaft.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.109)),
        material=steel,
        name="input_bearing_collar",
    )

    transfer_shaft = model.part("transfer_shaft")
    transfer_shaft.visual(
        Cylinder(radius=0.010, length=0.620),
        origin=Origin(xyz=(0.230, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="transfer_shaft_bar",
    )
    transfer_shaft.visual(
        mesh_from_cadquery(
            bevel_pair.assemble(build_gear=False, pinion_build_args={"bore_d": 0.014}),
            "transfer_bevel_gear",
            tolerance=0.0012,
            angular_tolerance=0.14,
        ),
        origin=Origin(xyz=(0.0, 0.0, -bevel_axis_height)),
        material=brass,
        name="transfer_bevel",
    )
    transfer_shaft.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(-0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="bevel_hub",
    )
    for i, x in enumerate((0.096, 0.144, 0.476, 0.524)):
        transfer_shaft.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"transfer_collar_{i}",
        )
    transfer_shaft.visual(
        _centered_spur_mesh(0.006, 18, 0.036, 0.014, "transfer_spur_gear"),
        origin=Origin(xyz=(spur_x - bevel_origin[0], 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="transfer_spur",
    )
    transfer_shaft.visual(
        Cylinder(radius=0.021, length=0.044),
        origin=Origin(xyz=(spur_x - bevel_origin[0], 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="transfer_spur_hub",
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        Cylinder(radius=0.010, length=0.430),
        origin=Origin(xyz=(0.300, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="output_shaft_bar",
    )
    for i, x in enumerate((0.186, 0.234, 0.456, 0.504)):
        output_shaft.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"output_collar_{i}",
        )
    output_shaft.visual(
        _centered_spur_mesh(0.006, 24, 0.040, 0.014, "output_spur_gear"),
        origin=Origin(xyz=(spur_x - bevel_origin[0], 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=oil_blue,
        name="output_spur",
    )
    output_shaft.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(spur_x - bevel_origin[0], 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="output_spur_hub",
    )

    model.articulation(
        "input_axis",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=input_shaft,
        origin=Origin(xyz=bevel_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "transfer_axis",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=transfer_shaft,
        origin=Origin(xyz=(bevel_origin[0], bevel_origin[1], shaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
        mimic=Mimic("input_axis", multiplier=-1.0),
    )
    model.articulation(
        "output_axis",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=output_shaft,
        origin=Origin(xyz=(bevel_origin[0], output_y, shaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
        mimic=Mimic("transfer_axis", multiplier=-18.0 / 24.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    input_shaft = object_model.get_part("input_shaft")
    transfer_shaft = object_model.get_part("transfer_shaft")
    output_shaft = object_model.get_part("output_shaft")
    input_axis = object_model.get_articulation("input_axis")
    transfer_axis = object_model.get_articulation("transfer_axis")
    output_axis = object_model.get_articulation("output_axis")

    ctx.check(
        "three supported rotating shafts",
        input_axis.articulation_type == ArticulationType.CONTINUOUS
        and transfer_axis.articulation_type == ArticulationType.CONTINUOUS
        and output_axis.articulation_type == ArticulationType.CONTINUOUS,
        details="input, transfer, and output shafts should each have a distinct continuous revolute joint",
    )
    ctx.check(
        "shaft axes are vertical then parallel horizontal",
        tuple(input_axis.axis) == (0.0, 0.0, 1.0)
        and tuple(transfer_axis.axis) == (1.0, 0.0, 0.0)
        and tuple(output_axis.axis) == (1.0, 0.0, 0.0),
        details=f"axes: input={input_axis.axis}, transfer={transfer_axis.axis}, output={output_axis.axis}",
    )

    ctx.expect_within(
        input_shaft,
        base,
        axes="xy",
        inner_elem="input_shaft_bar",
        outer_elem="lower_vertical_bearing",
        margin=0.001,
        name="input shaft runs through lower bearing bore",
    )
    ctx.expect_overlap(
        input_shaft,
        base,
        axes="z",
        elem_a="input_shaft_bar",
        elem_b="lower_vertical_bearing",
        min_overlap=0.030,
        name="input shaft passes through lower bearing",
    )
    ctx.expect_within(
        transfer_shaft,
        base,
        axes="yz",
        inner_elem="transfer_shaft_bar",
        outer_elem="transfer_bearing_0",
        margin=0.001,
        name="transfer shaft lies inside its pillow block",
    )
    ctx.expect_within(
        output_shaft,
        base,
        axes="yz",
        inner_elem="output_shaft_bar",
        outer_elem="output_bearing_0",
        margin=0.001,
        name="output shaft lies inside its pillow block",
    )
    ctx.expect_origin_gap(
        output_shaft,
        transfer_shaft,
        axis="y",
        min_gap=0.123,
        max_gap=0.130,
        name="parallel spur shafts have gear-stage spacing",
    )
    ctx.expect_contact(
        input_shaft,
        transfer_shaft,
        elem_a="input_bevel",
        elem_b="transfer_bevel",
        contact_tol=0.0015,
        name="bevel gears mesh at right angle",
    )
    ctx.expect_contact(
        transfer_shaft,
        output_shaft,
        elem_a="transfer_spur",
        elem_b="output_spur",
        contact_tol=0.001,
        name="spur gears mesh closely",
    )
    ctx.expect_overlap(
        transfer_shaft,
        output_shaft,
        axes="x",
        elem_a="transfer_spur",
        elem_b="output_spur",
        min_overlap=0.030,
        name="spur gears are aligned on the same face plane",
    )

    return ctx.report()


object_model = build_object_model()
