from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chained_plunger_mechanism")

    steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    bronze = model.material("bronze_bushings", rgba=(0.78, 0.48, 0.18, 1.0))
    blue = model.material("anodized_blue", rgba=(0.05, 0.22, 0.65, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    tube_length = 0.24
    outer_radius = 0.055
    inner_radius = 0.033
    sleeve_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -tube_length / 2.0), (outer_radius, tube_length / 2.0)],
            [(inner_radius, -tube_length / 2.0), (inner_radius, tube_length / 2.0)],
            segments=48,
        ),
        "hollow_guide_sleeve",
    )
    bushing_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.038, -0.009), (0.038, 0.009)],
            [(0.026, -0.009), (0.026, 0.009)],
            segments=40,
        ),
        "bronze_guide_bushing",
    )

    base = model.part("base")
    base.visual(
        Box((1.55, 0.26, 0.035)),
        origin=Origin(xyz=(0.06, 0.0, 0.0175)),
        material=dark_steel,
        name="base_plate",
    )
    base.visual(
        Box((1.48, 0.028, 0.024)),
        origin=Origin(xyz=(0.06, -0.095, 0.052)),
        material=steel,
        name="front_rail",
    )
    base.visual(
        Box((1.48, 0.028, 0.024)),
        origin=Origin(xyz=(0.06, 0.095, 0.052)),
        material=steel,
        name="rear_rail",
    )

    sleeve_centers = (-0.36, 0.08, 0.52)
    for i, x in enumerate(sleeve_centers):
        base.visual(
            sleeve_mesh,
            origin=Origin(xyz=(x, 0.0, 0.235), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"sleeve_{i}",
        )
        base.visual(
            Box((0.070, 0.170, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.176)),
            material=steel,
            name=f"saddle_{i}",
        )
        base.visual(
            Box((0.060, 0.024, 0.195)),
            origin=Origin(xyz=(x, -0.085, 0.124)),
            material=dark_steel,
            name=f"front_bracket_{i}",
        )
        base.visual(
            Box((0.060, 0.024, 0.195)),
            origin=Origin(xyz=(x, 0.085, 0.124)),
            material=dark_steel,
            name=f"rear_bracket_{i}",
        )
        base.visual(
            Box((0.090, 0.210, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.041)),
            material=dark_steel,
            name=f"foot_plate_{i}",
        )
        base.visual(
            bushing_mesh,
            origin=Origin(xyz=(x - 0.095, 0.0, 0.235), rpy=(0.0, pi / 2.0, 0.0)),
            material=bronze,
            name=f"bushing_{i}_0",
        )
        base.visual(
            bushing_mesh,
            origin=Origin(xyz=(x + 0.095, 0.0, 0.235), rpy=(0.0, pi / 2.0, 0.0)),
            material=bronze,
            name=f"bushing_{i}_1",
        )
        base.visual(
            Box((0.036, 0.018, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.2075)),
            material=bronze,
            name=f"wear_pad_{i}",
        )

    rod_radius = 0.022
    collar_radius = 0.040

    input_plunger = model.part("input_plunger")
    input_plunger.visual(
        Cylinder(radius=rod_radius, length=0.484),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rod",
    )
    input_plunger.visual(
        Cylinder(radius=0.055, length=0.075),
        origin=Origin(xyz=(-0.255, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="push_knob",
    )
    input_plunger.visual(
        Cylinder(radius=collar_radius, length=0.032),
        origin=Origin(xyz=(0.260, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue,
        name="drive_collar",
    )

    transfer_plunger = model.part("transfer_plunger")
    transfer_plunger.visual(
        Cylinder(radius=rod_radius, length=0.352),
        origin=Origin(xyz=(0.044, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rod",
    )
    transfer_plunger.visual(
        Cylinder(radius=collar_radius, length=0.032),
        origin=Origin(xyz=(-0.148, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue,
        name="rear_collar",
    )
    transfer_plunger.visual(
        Cylinder(radius=collar_radius, length=0.032),
        origin=Origin(xyz=(0.220, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue,
        name="front_collar",
    )

    output_plunger = model.part("output_plunger")
    output_plunger.visual(
        Cylinder(radius=rod_radius, length=0.400),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rod",
    )
    output_plunger.visual(
        Cylinder(radius=collar_radius, length=0.032),
        origin=Origin(xyz=(-0.188, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue,
        name="rear_collar",
    )
    output_plunger.visual(
        Cylinder(radius=0.048, length=0.055),
        origin=Origin(xyz=(0.2425, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue,
        name="output_pad",
    )

    travel = 0.120
    drive_limits = MotionLimits(lower=0.0, upper=travel, effort=120.0, velocity=0.30)
    model.articulation(
        "base_to_input",
        ArticulationType.PRISMATIC,
        parent=base,
        child=input_plunger,
        origin=Origin(xyz=(-0.36, 0.0, 0.235)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=drive_limits,
    )
    model.articulation(
        "base_to_transfer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=transfer_plunger,
        origin=Origin(xyz=(0.08, 0.0, 0.235)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=drive_limits,
        mimic=Mimic(joint="base_to_input"),
    )
    model.articulation(
        "base_to_output",
        ArticulationType.PRISMATIC,
        parent=base,
        child=output_plunger,
        origin=Origin(xyz=(0.52, 0.0, 0.235)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=drive_limits,
        mimic=Mimic(joint="base_to_input"),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    input_plunger = object_model.get_part("input_plunger")
    transfer_plunger = object_model.get_part("transfer_plunger")
    output_plunger = object_model.get_part("output_plunger")
    drive = object_model.get_articulation("base_to_input")
    travel = 0.120

    for plunger, sleeve in (
        (input_plunger, "sleeve_0"),
        (transfer_plunger, "sleeve_1"),
        (output_plunger, "sleeve_2"),
    ):
        ctx.expect_within(
            plunger,
            base,
            axes="yz",
            inner_elem="rod",
            outer_elem=sleeve,
            margin=0.0,
            name=f"{plunger.name} centered in {sleeve}",
        )
        ctx.expect_overlap(
            plunger,
            base,
            axes="x",
            elem_a="rod",
            elem_b=sleeve,
            min_overlap=0.12,
            name=f"{plunger.name} retained in {sleeve}",
        )

    ctx.expect_contact(
        input_plunger,
        transfer_plunger,
        elem_a="drive_collar",
        elem_b="rear_collar",
        contact_tol=1e-5,
        name="first plunger collar drives transfer collar",
    )
    ctx.expect_contact(
        transfer_plunger,
        output_plunger,
        elem_a="front_collar",
        elem_b="rear_collar",
        contact_tol=1e-5,
        name="transfer collar drives output collar",
    )

    rest_output = ctx.part_world_position(output_plunger)
    with ctx.pose({drive: travel}):
        for plunger, sleeve in (
            (input_plunger, "sleeve_0"),
            (transfer_plunger, "sleeve_1"),
            (output_plunger, "sleeve_2"),
        ):
            ctx.expect_within(
                plunger,
                base,
                axes="yz",
                inner_elem="rod",
                outer_elem=sleeve,
                margin=0.0,
                name=f"{plunger.name} stays coaxial at full stroke",
            )
            ctx.expect_overlap(
                plunger,
                base,
                axes="x",
                elem_a="rod",
                elem_b=sleeve,
                min_overlap=0.10,
                name=f"{plunger.name} remains captured at full stroke",
            )
        extended_output = ctx.part_world_position(output_plunger)

    ctx.check(
        "chained plungers translate in the positive stroke direction",
        rest_output is not None
        and extended_output is not None
        and extended_output[0] > rest_output[0] + 0.10,
        details=f"rest={rest_output}, extended={extended_output}",
    )

    return ctx.report()


object_model = build_object_model()
