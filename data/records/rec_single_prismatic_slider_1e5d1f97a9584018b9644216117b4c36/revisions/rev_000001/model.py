from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _chamfered_box(size: tuple[float, float, float], chamfer: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if chamfer > 0.0:
        shape = shape.edges().chamfer(chamfer)
    return shape


def _rail_way(length: float, bottom_width: float, top_width: float, height: float) -> cq.Workplane:
    """Long prismatic way with beveled sides and a flat precision bearing land."""
    points = [
        (-bottom_width / 2.0, -height / 2.0),
        (bottom_width / 2.0, -height / 2.0),
        (top_width / 2.0, height / 2.0),
        (-top_width / 2.0, height / 2.0),
    ]
    return cq.Workplane("YZ").polyline(points).close().extrude(length).translate((-length / 2.0, 0.0, 0.0))


def _cap_screw_head(radius: float = 0.006, height: float = 0.003) -> cq.Workplane:
    """Low socket-head cap screw, centered on its local origin."""
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .faces(">Z")
        .workplane()
        .polygon(6, radius * 1.05)
        .cutBlind(-height * 0.45)
        .edges("|Z")
        .chamfer(0.00035)
        .translate((0.0, 0.0, -height / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_single_axis_stage")

    dark_iron = Material("blackened_iron", rgba=(0.035, 0.035, 0.038, 1.0))
    bead_blast = Material("bead_blasted_aluminum", rgba=(0.58, 0.60, 0.61, 1.0))
    blue_gray = Material("blue_gray_anodized", rgba=(0.22, 0.29, 0.34, 1.0))
    ground_steel = Material("ground_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    bronze = Material("oiled_bronze", rgba=(0.62, 0.44, 0.22, 1.0))
    rubber = Material("matte_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    base_plate_mesh = mesh_from_cadquery(
        _chamfered_box((0.80, 0.20, 0.040), 0.004),
        "grounded_base_plate",
        tolerance=0.0008,
        angular_tolerance=0.18,
    )
    web_mesh = mesh_from_cadquery(
        _chamfered_box((0.70, 0.120, 0.018), 0.0025),
        "raised_center_web",
        tolerance=0.0008,
        angular_tolerance=0.18,
    )
    rail_mesh = mesh_from_cadquery(
        _rail_way(0.64, 0.026, 0.018, 0.012),
        "beveled_precision_way",
        tolerance=0.0006,
        angular_tolerance=0.15,
    )
    stop_mesh = mesh_from_cadquery(
        _chamfered_box((0.050, 0.140, 0.050), 0.0025),
        "travel_end_stop",
        tolerance=0.0008,
        angular_tolerance=0.18,
    )
    screw_mesh = mesh_from_cadquery(
        _cap_screw_head(),
        "socket_cap_screw",
        tolerance=0.0005,
        angular_tolerance=0.12,
    )
    shoe_mesh = mesh_from_cadquery(
        _chamfered_box((0.120, 0.118, 0.034), 0.003),
        "carriage_shoe_body",
        tolerance=0.0007,
        angular_tolerance=0.16,
    )
    pad_mesh = mesh_from_cadquery(
        _chamfered_box((0.105, 0.105, 0.016), 0.002),
        "square_instrument_pad",
        tolerance=0.0007,
        angular_tolerance=0.16,
    )

    lower_beam = model.part("lower_beam")
    lower_beam.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_iron,
        name="base_plate",
    )
    lower_beam.visual(
        web_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0485)),
        material=bead_blast,
        name="center_web",
    )
    lower_beam.visual(
        rail_mesh,
        origin=Origin(xyz=(0.0, -0.043, 0.063)),
        material=ground_steel,
        name="rail_0",
    )
    lower_beam.visual(
        rail_mesh,
        origin=Origin(xyz=(0.0, 0.043, 0.063)),
        material=ground_steel,
        name="rail_1",
    )
    for index, x in enumerate((-0.325, 0.325)):
        lower_beam.visual(
            stop_mesh,
            origin=Origin(xyz=(x, 0.0, 0.082)),
            material=bead_blast,
            name=f"end_stop_{index}",
        )
    lower_beam.visual(
        Box((0.009, 0.052, 0.018)),
        origin=Origin(xyz=(-0.2955, 0.0, 0.091)),
        material=rubber,
        name="stop_bumper_0",
    )
    lower_beam.visual(
        Box((0.009, 0.052, 0.018)),
        origin=Origin(xyz=(0.2955, 0.0, 0.091)),
        material=rubber,
        name="stop_bumper_1",
    )

    for index, (x, y) in enumerate(
        (
            (-0.330, -0.074),
            (-0.330, 0.074),
            (0.330, -0.074),
            (0.330, 0.074),
            (-0.185, -0.043),
            (-0.185, 0.043),
            (0.185, -0.043),
            (0.185, 0.043),
        )
    ):
        screw_z = 0.0705 if index >= 4 else 0.0712
        lower_beam.visual(
            screw_mesh,
            origin=Origin(xyz=(x, y, screw_z)),
            material=dark_iron,
            name=f"base_screw_{index}",
        )

    carriage_shoe = model.part("carriage_shoe")
    carriage_shoe.visual(
        shoe_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=blue_gray,
        name="shoe_body",
    )
    carriage_shoe.visual(
        Box((0.106, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, -0.043, 0.072)),
        material=bronze,
        name="bearing_pad_0",
    )
    carriage_shoe.visual(
        Box((0.106, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.043, 0.072)),
        material=bronze,
        name="bearing_pad_1",
    )
    for index, y in enumerate((-0.061, 0.061)):
        carriage_shoe.visual(
            Box((0.106, 0.006, 0.017)),
            origin=Origin(xyz=(0.0, y, 0.0785)),
            material=blue_gray,
            name=f"side_gib_{index}",
        )
    carriage_shoe.visual(
        pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.1145)),
        material=bead_blast,
        name="instrument_pad",
    )
    carriage_shoe.visual(
        Cylinder(radius=0.010, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.1233)),
        material=dark_iron,
        name="center_socket",
    )
    for index, (x, y) in enumerate(((-0.035, -0.035), (-0.035, 0.035), (0.035, -0.035), (0.035, 0.035))):
        carriage_shoe.visual(
            screw_mesh,
            origin=Origin(xyz=(x, y, 0.1240)),
            material=dark_iron,
            name=f"pad_screw_{index}",
        )

    model.articulation(
        "beam_to_shoe",
        ArticulationType.PRISMATIC,
        parent=lower_beam,
        child=carriage_shoe,
        origin=Origin(xyz=(-0.180, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.360),
        motion_properties=MotionProperties(damping=8.0, friction=1.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("lower_beam")
    shoe = object_model.get_part("carriage_shoe")
    slide = object_model.get_articulation("beam_to_shoe")

    ctx.check(
        "single prismatic moving stage",
        len(object_model.articulations) == 1
        and slide.articulation_type == ArticulationType.PRISMATIC
        and slide.child == "carriage_shoe",
        details=f"articulations={object_model.articulations}",
    )

    ctx.expect_gap(
        shoe,
        beam,
        axis="z",
        positive_elem="bearing_pad_0",
        negative_elem="rail_0",
        max_gap=0.0006,
        max_penetration=0.000001,
        name="near bearing pad rides on rail",
    )
    ctx.expect_gap(
        shoe,
        beam,
        axis="z",
        positive_elem="bearing_pad_1",
        negative_elem="rail_1",
        max_gap=0.0006,
        max_penetration=0.000001,
        name="far bearing pad rides on rail",
    )
    ctx.expect_within(
        shoe,
        beam,
        axes="y",
        inner_elem="bearing_pad_0",
        outer_elem="rail_0",
        margin=0.0015,
        name="near pad centered on machined way",
    )
    ctx.expect_within(
        shoe,
        beam,
        axes="y",
        inner_elem="bearing_pad_1",
        outer_elem="rail_1",
        margin=0.0015,
        name="far pad centered on machined way",
    )

    ctx.expect_gap(
        shoe,
        beam,
        axis="x",
        positive_elem="shoe_body",
        negative_elem="stop_bumper_0",
        min_gap=0.040,
        name="carriage clears left stop at parked end",
    )

    rest_position = ctx.part_world_position(shoe)
    with ctx.pose({slide: 0.360}):
        ctx.expect_gap(
            beam,
            shoe,
            axis="x",
            positive_elem="stop_bumper_1",
            negative_elem="shoe_body",
            min_gap=0.040,
            name="carriage clears right stop at travel end",
        )
        ctx.expect_gap(
            shoe,
            beam,
            axis="z",
            positive_elem="bearing_pad_0",
            negative_elem="rail_0",
            max_gap=0.0006,
            max_penetration=0.000001,
            name="near bearing pad remains on rail at full travel",
        )
        ctx.expect_gap(
            shoe,
            beam,
            axis="z",
            positive_elem="bearing_pad_1",
            negative_elem="rail_1",
            max_gap=0.0006,
            max_penetration=0.000001,
            name="far bearing pad remains on rail at full travel",
        )
        extended_position = ctx.part_world_position(shoe)

    ctx.check(
        "slide travels along guide direction",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.34
        and abs(extended_position[1] - rest_position[1]) < 0.001
        and abs(extended_position[2] - rest_position[2]) < 0.001,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
