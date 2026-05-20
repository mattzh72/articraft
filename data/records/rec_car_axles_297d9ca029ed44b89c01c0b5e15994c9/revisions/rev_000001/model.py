from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


AXLE_HALF_SPAN = 0.97
BEAM_Z = 0.34
BEAM_Y = 0.05
DIFF_CENTER = (0.0, -0.10, 0.31)


def _cyl_x(x: float, y: float, z: float) -> Origin:
    """Origin for a cylinder whose local Z axis is rotated onto world X."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_y(x: float, y: float, z: float) -> Origin:
    """Origin for a cylinder whose local Z axis is rotated onto world Y."""
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="de_dion_rear_axle")

    satin_black = model.material("satin_black", color=(0.01, 0.012, 0.012, 1.0))
    dark_steel = model.material("dark_steel", color=(0.09, 0.095, 0.10, 1.0))
    cast_iron = model.material("cast_iron", color=(0.23, 0.24, 0.24, 1.0))
    machined = model.material("machined_steel", color=(0.62, 0.64, 0.62, 1.0))
    brake = model.material("brake_rotor", color=(0.36, 0.35, 0.33, 1.0))
    rubber = model.material("rubber_bushing", color=(0.015, 0.013, 0.012, 1.0))

    # Chassis-side carrier: a compact black cradle that physically supports the
    # fixed differential and provides two visible saddle mounts for the De Dion
    # beam.  The De Dion tube itself remains a separate rigid part.
    carrier = model.part("carrier")
    carrier.visual(
        Box((0.78, 0.055, 0.060)),
        origin=Origin(xyz=(0.0, -0.28, 0.48)),
        material=satin_black,
        name="crossmember",
    )
    carrier.visual(
        Box((0.56, 0.305, 0.050)),
        origin=Origin(xyz=(0.0, -0.10, 0.085)),
        material=satin_black,
        name="diff_cradle",
    )
    for idx, x in enumerate((-0.25, 0.25)):
        carrier.visual(
            Box((0.045, 0.055, 0.380)),
            origin=Origin(xyz=(x, -0.2525, 0.295)),
            material=satin_black,
            name=f"cradle_upright_{idx}",
        )
    carrier.visual(
        Box((0.18, 0.095, 0.035)),
        origin=Origin(xyz=(-0.55, 0.105, 0.2825)),
        material=satin_black,
        name="beam_saddle_0",
    )
    carrier.visual(
        Box((0.18, 0.095, 0.035)),
        origin=Origin(xyz=(0.55, 0.105, 0.2825)),
        material=satin_black,
        name="beam_saddle_1",
    )
    for idx, x in enumerate((-0.34, 0.34)):
        rail = tube_from_spline_points(
            [
                (x, -0.28, 0.48),
                (x * 1.08, -0.18, 0.40),
                (x * 1.30, -0.02, 0.33),
                (math.copysign(0.55, x), 0.105, 0.235),
            ],
            radius=0.018,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        )
        carrier.visual(
            mesh_from_geometry(rail, f"carrier_rail_{idx}"),
            material=satin_black,
            name=f"support_rail_{idx}",
        )

    # Rigid curved De Dion tube beam: a single swept steel tube bows rearward
    # around the separately mounted differential and terminates in axle collars.
    beam = model.part("axle_beam")
    beam_tube = tube_from_spline_points(
        [
            (-0.82, BEAM_Y, BEAM_Z),
            (-0.56, 0.115, BEAM_Z + 0.005),
            (-0.22, 0.205, BEAM_Z + 0.035),
            (0.0, 0.235, BEAM_Z + 0.045),
            (0.22, 0.205, BEAM_Z + 0.035),
            (0.56, 0.115, BEAM_Z + 0.005),
            (0.82, BEAM_Y, BEAM_Z),
        ],
        radius=0.040,
        samples_per_segment=16,
        radial_segments=28,
        cap_ends=True,
    )
    beam.visual(
        mesh_from_geometry(beam_tube, "curved_de_dion_tube"),
        material=dark_steel,
        name="curved_tube",
    )
    beam.visual(
        Cylinder(radius=0.095, length=0.160),
        origin=_cyl_x(-0.89, BEAM_Y, BEAM_Z),
        material=dark_steel,
        name="bearing_collar_0",
    )
    beam.visual(
        Cylinder(radius=0.095, length=0.160),
        origin=_cyl_x(0.89, BEAM_Y, BEAM_Z),
        material=dark_steel,
        name="bearing_collar_1",
    )
    for idx, x in enumerate((-0.55, 0.55)):
        beam.visual(
            Box((0.16, 0.075, 0.030)),
            origin=Origin(xyz=(x, 0.105, 0.315)),
            material=dark_steel,
            name=f"saddle_pad_{idx}",
        )

    model.articulation(
        "carrier_to_beam",
        ArticulationType.FIXED,
        parent=carrier,
        child=beam,
        origin=Origin(),
    )

    # Static central differential housing, mounted to the carrier and separated
    # from the De Dion tube.  The short output stubs spin at its side flanges.
    differential = model.part("differential")
    pumpkin = SphereGeometry(0.18, width_segments=36, height_segments=20).scale(
        1.05, 0.78, 0.92
    )
    differential.visual(
        mesh_from_geometry(pumpkin, "differential_pumpkin"),
        material=cast_iron,
        name="pumpkin",
    )
    differential.visual(
        Cylinder(radius=0.120, length=0.070),
        origin=_cyl_y(0.0, -0.125, 0.0),
        material=cast_iron,
        name="rear_cover",
    )
    differential.visual(
        Cylinder(radius=0.105, length=0.100),
        origin=_cyl_x(-0.22, 0.0, 0.0),
        material=cast_iron,
        name="side_flange_0",
    )
    differential.visual(
        Cylinder(radius=0.105, length=0.100),
        origin=_cyl_x(0.22, 0.0, 0.0),
        material=cast_iron,
        name="side_flange_1",
    )
    differential.visual(
        Box((0.22, 0.14, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.1825)),
        material=cast_iron,
        name="mounting_foot",
    )
    differential.visual(
        Cylinder(radius=0.035, length=0.24),
        origin=_cyl_x(0.0, 0.0, -0.155),
        material=rubber,
        name="rubber_mount",
    )
    model.articulation(
        "carrier_to_differential",
        ArticulationType.FIXED,
        parent=carrier,
        child=differential,
        origin=Origin(xyz=DIFF_CENTER),
    )

    # The two wheel hubs spin about the lateral axle line at the beam collars.
    for idx, side in enumerate((-1.0, 1.0)):
        hub = model.part(f"hub_{idx}")
        hub.visual(
            Cylinder(radius=0.060, length=0.040),
            origin=_cyl_x(side * 0.020, 0.0, 0.0),
            material=machined,
            name="bearing",
        )
        hub.visual(
            Cylinder(radius=0.180, length=0.018),
            origin=_cyl_x(side * 0.049, 0.0, 0.0),
            material=brake,
            name="brake_disc",
        )
        hub.visual(
            Cylinder(radius=0.105, length=0.055),
            origin=_cyl_x(side * 0.078, 0.0, 0.0),
            material=machined,
            name="wheel_flange",
        )
        hub.visual(
            Sphere(radius=0.055),
            origin=Origin(xyz=(side * 0.122, 0.0, 0.0)),
            material=machined,
            name="center_cap",
        )
        for bolt_i in range(5):
            angle = 2.0 * math.pi * bolt_i / 5.0 + math.pi / 2.0
            y = 0.072 * math.cos(angle)
            z = 0.072 * math.sin(angle)
            hub.visual(
                Cylinder(radius=0.007, length=0.035),
                origin=_cyl_x(side * 0.123, y, z),
                material=machined,
                name=f"wheel_stud_{bolt_i}",
            )
        model.articulation(
            f"beam_to_hub_{idx}",
            ArticulationType.CONTINUOUS,
            parent=beam,
            child=hub,
            origin=Origin(xyz=(side * AXLE_HALF_SPAN, BEAM_Y, BEAM_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=250.0, velocity=80.0),
        )

    # Short driveshaft output stubs, deliberately ending well before the hubs to
    # emphasize that the differential is separate from the rigid De Dion tube.
    for idx, side in enumerate((-1.0, 1.0)):
        stub = model.part(f"stub_{idx}")
        stub.visual(
            Cylinder(radius=0.027, length=0.280),
            origin=_cyl_x(side * 0.140, 0.0, 0.0),
            material=machined,
            name="shaft",
        )
        stub.visual(
            Sphere(radius=0.045),
            origin=Origin(xyz=(side * 0.295, 0.0, 0.0)),
            material=machined,
            name="cv_bulb",
        )
        stub.visual(
            Box((0.032, 0.098, 0.026)),
            origin=Origin(xyz=(side * 0.205, 0.0, 0.0)),
            material=machined,
            name="yoke_lug",
        )
        model.articulation(
            f"differential_to_stub_{idx}",
            ArticulationType.CONTINUOUS,
            parent=differential,
            child=stub,
            origin=Origin(xyz=(side * 0.270, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=160.0, velocity=90.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    beam = object_model.get_part("axle_beam")
    differential = object_model.get_part("differential")
    hub_0 = object_model.get_part("hub_0")
    hub_1 = object_model.get_part("hub_1")
    stub_0 = object_model.get_part("stub_0")
    stub_1 = object_model.get_part("stub_1")

    hub_joint_0 = object_model.get_articulation("beam_to_hub_0")
    hub_joint_1 = object_model.get_articulation("beam_to_hub_1")
    stub_joint_0 = object_model.get_articulation("differential_to_stub_0")
    stub_joint_1 = object_model.get_articulation("differential_to_stub_1")

    spinning_joints = (hub_joint_0, hub_joint_1, stub_joint_0, stub_joint_1)
    ctx.check(
        "four lateral spin joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in spinning_joints)
        and all(tuple(j.axis) == (1.0, 0.0, 0.0) for j in spinning_joints),
        details=f"joints={[(j.name, j.articulation_type, j.axis) for j in spinning_joints]}",
    )

    ctx.expect_origin_gap(
        hub_1,
        hub_0,
        axis="x",
        min_gap=1.80,
        max_gap=2.10,
        name="wheel hubs sit at opposite beam ends",
    )
    ctx.expect_contact(
        hub_0,
        beam,
        elem_a="bearing",
        elem_b="bearing_collar_0",
        name="hub 0 bearing seats on beam collar",
    )
    ctx.expect_contact(
        hub_1,
        beam,
        elem_a="bearing",
        elem_b="bearing_collar_1",
        name="hub 1 bearing seats on beam collar",
    )
    ctx.expect_overlap(
        hub_0,
        beam,
        axes="yz",
        min_overlap=0.05,
        elem_a="bearing",
        elem_b="bearing_collar_0",
        name="hub 0 is coaxial with beam collar",
    )
    ctx.expect_overlap(
        hub_1,
        beam,
        axes="yz",
        min_overlap=0.05,
        elem_a="bearing",
        elem_b="bearing_collar_1",
        name="hub 1 is coaxial with beam collar",
    )

    ctx.expect_gap(
        differential,
        stub_0,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="side_flange_0",
        negative_elem="shaft",
        name="stub 0 begins at differential flange",
    )
    ctx.expect_gap(
        stub_1,
        differential,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="shaft",
        negative_elem="side_flange_1",
        name="stub 1 begins at differential flange",
    )
    ctx.expect_overlap(
        stub_0,
        differential,
        axes="yz",
        min_overlap=0.05,
        elem_a="shaft",
        elem_b="side_flange_0",
        name="stub 0 is coaxial with differential flange",
    )
    ctx.expect_overlap(
        stub_1,
        differential,
        axes="yz",
        min_overlap=0.05,
        elem_a="shaft",
        elem_b="side_flange_1",
        name="stub 1 is coaxial with differential flange",
    )
    ctx.expect_gap(
        stub_0,
        hub_0,
        axis="x",
        min_gap=0.25,
        name="stub 0 remains a short separate output",
    )
    ctx.expect_gap(
        hub_1,
        stub_1,
        axis="x",
        min_gap=0.25,
        name="stub 1 remains a short separate output",
    )

    rest_positions = {
        "hub_0": ctx.part_world_position(hub_0),
        "stub_0": ctx.part_world_position(stub_0),
    }
    with ctx.pose({hub_joint_0: math.pi / 2.0, stub_joint_0: math.pi / 2.0}):
        spun_positions = {
            "hub_0": ctx.part_world_position(hub_0),
            "stub_0": ctx.part_world_position(stub_0),
        }
        ctx.expect_contact(
            hub_0,
            beam,
            elem_a="bearing",
            elem_b="bearing_collar_0",
            name="hub bearing stays seated while spinning",
        )
        ctx.expect_gap(
            differential,
            stub_0,
            axis="x",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="side_flange_0",
            negative_elem="shaft",
            name="stub stays on side flange while spinning",
        )
    ctx.check(
        "spin joints rotate without translating centers",
        rest_positions == spun_positions,
        details=f"rest={rest_positions}, spun={spun_positions}",
    )

    return ctx.report()


object_model = build_object_model()
