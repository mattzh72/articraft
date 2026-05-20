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
    model = ArticulatedObject(name="side_wall_gantry_transfer_module")

    painted_steel = Material("charcoal_painted_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    rail_steel = Material("ground_steel_rail", rgba=(0.72, 0.72, 0.69, 1.0))
    dark_hardware = Material("blackened_hardware", rgba=(0.015, 0.016, 0.018, 1.0))
    crosshead_blue = Material("blue_anodized_crosshead", rgba=(0.05, 0.20, 0.42, 1.0))
    carriage_orange = Material("orange_carriage", rgba=(0.92, 0.38, 0.08, 1.0))
    wear_pad = Material("dark_polymer_wear_pad", rgba=(0.03, 0.03, 0.035, 1.0))

    frame = model.part("rear_frame")
    # A wall-mounted weldment: back plate, perimeter stiffeners, two linear rails,
    # and rail stand-off blocks are kept as one rigid root part.
    frame.visual(
        Box((1.55, 0.060, 0.92)),
        origin=Origin(xyz=(0.0, 0.000, 0.60)),
        material=painted_steel,
        name="back_plate",
    )
    frame.visual(
        Box((1.62, 0.090, 0.070)),
        origin=Origin(xyz=(0.0, 0.020, 1.085)),
        material=painted_steel,
        name="top_header",
    )
    frame.visual(
        Box((1.62, 0.090, 0.070)),
        origin=Origin(xyz=(0.0, 0.020, 0.115)),
        material=painted_steel,
        name="bottom_header",
    )
    frame.visual(
        Box((0.080, 0.090, 1.02)),
        origin=Origin(xyz=(-0.785, 0.020, 0.600)),
        material=painted_steel,
        name="side_upright_0",
    )
    frame.visual(
        Box((0.080, 0.090, 1.02)),
        origin=Origin(xyz=(0.785, 0.020, 0.600)),
        material=painted_steel,
        name="side_upright_1",
    )

    rail_length = 1.36
    rail_y = 0.076
    upper_rail_z = 0.835
    lower_rail_z = 0.365
    frame.visual(
        Box((rail_length, 0.062, 0.052)),
        origin=Origin(xyz=(0.0, rail_y, upper_rail_z)),
        material=rail_steel,
        name="upper_rail",
    )
    frame.visual(
        Box((rail_length, 0.062, 0.052)),
        origin=Origin(xyz=(0.0, rail_y, lower_rail_z)),
        material=rail_steel,
        name="lower_rail",
    )
    for ix, x in enumerate((-0.56, 0.0, 0.56)):
        frame.visual(
            Box((0.070, 0.096, 0.112)),
            origin=Origin(xyz=(x, 0.045, upper_rail_z)),
            material=painted_steel,
            name=f"upper_standoff_{ix}",
        )
        frame.visual(
            Box((0.070, 0.096, 0.112)),
            origin=Origin(xyz=(x, 0.045, lower_rail_z)),
            material=painted_steel,
            name=f"lower_standoff_{ix}",
        )
    for ix, x in enumerate((-0.66, -0.22, 0.22, 0.66)):
        for iz, z in enumerate((0.17, 1.03)):
            frame.visual(
                Cylinder(radius=0.020, length=0.010),
                origin=Origin(xyz=(x, 0.034, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=dark_hardware,
                name=f"wall_bolt_{ix}_{iz}",
            )

    crosshead = model.part("crosshead")
    # The moving bridge is a compact box-section crosshead with two bearing shoes
    # that ride against the front datum faces of the fixed base rails.
    crosshead.visual(
        Box((0.230, 0.055, 0.650)),
        origin=Origin(xyz=(0.0, 0.095, 0.0)),
        material=crosshead_blue,
        name="crosshead_web",
    )
    crosshead.visual(
        Box((0.230, 0.055, 0.095)),
        origin=Origin(xyz=(0.0, 0.0275, 0.235)),
        material=wear_pad,
        name="upper_shoe",
    )
    crosshead.visual(
        Box((0.230, 0.055, 0.095)),
        origin=Origin(xyz=(0.0, 0.0275, -0.235)),
        material=wear_pad,
        name="lower_shoe",
    )
    crosshead.visual(
        Box((0.230, 0.070, 0.065)),
        origin=Origin(xyz=(0.0, 0.065, 0.235)),
        material=crosshead_blue,
        name="upper_shoe_bridge",
    )
    crosshead.visual(
        Box((0.230, 0.070, 0.065)),
        origin=Origin(xyz=(0.0, 0.065, -0.235)),
        material=crosshead_blue,
        name="lower_shoe_bridge",
    )
    crosshead.visual(
        Box((0.026, 0.052, 0.585)),
        origin=Origin(xyz=(-0.098, 0.1485, 0.0)),
        material=rail_steel,
        name="guide_lip_0",
    )
    crosshead.visual(
        Box((0.026, 0.052, 0.585)),
        origin=Origin(xyz=(0.098, 0.1485, 0.0)),
        material=rail_steel,
        name="guide_lip_1",
    )
    crosshead.visual(
        Box((0.230, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, 0.161, 0.300)),
        material=dark_hardware,
        name="upper_travel_stop",
    )
    crosshead.visual(
        Box((0.230, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, 0.161, -0.300)),
        material=dark_hardware,
        name="lower_travel_stop",
    )
    for iz, z in enumerate((-0.245, 0.245)):
        for ix, x in enumerate((-0.075, 0.075)):
            crosshead.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(xyz=(x, 0.176, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=dark_hardware,
                name=f"shoe_cap_screw_{iz}_{ix}",
            )

    carriage = model.part("carriage")
    # The smaller nested carriage travels vertically inside the crosshead face
    # lips, leaving the guide lips visible on both sides.
    carriage.visual(
        Box((0.135, 0.052, 0.200)),
        origin=Origin(xyz=(0.0, 0.026, 0.100)),
        material=carriage_orange,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.102, 0.018, 0.150)),
        origin=Origin(xyz=(0.0, 0.061, 0.100)),
        material=dark_hardware,
        name="front_tool_plate",
    )
    carriage.visual(
        Cylinder(radius=0.042, length=0.026),
        origin=Origin(xyz=(0.0, 0.083, 0.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="tool_mount_boss",
    )
    carriage.visual(
        Box((0.032, 0.024, 0.055)),
        origin=Origin(xyz=(0.0, 0.062, 0.205)),
        material=dark_hardware,
        name="cable_socket",
    )

    crosshead_joint = model.articulation(
        "frame_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=crosshead,
        origin=Origin(xyz=(-0.450, rail_y + 0.031, (upper_rail_z + lower_rail_z) / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.55, lower=0.0, upper=0.900),
    )
    crosshead_joint.meta["description"] = "Horizontal transfer stroke along the two fixed base rails."

    carriage_joint = model.articulation(
        "crosshead_to_carriage",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.1225, -0.260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.320),
    )
    carriage_joint.meta["description"] = "Nested vertical carriage stroke inside the crosshead face."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("rear_frame")
    crosshead = object_model.get_part("crosshead")
    carriage = object_model.get_part("carriage")
    crosshead_joint = object_model.get_articulation("frame_to_crosshead")
    carriage_joint = object_model.get_articulation("crosshead_to_carriage")

    ctx.check(
        "module has the requested two prismatic stages",
        len(object_model.articulations) == 2
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )

    ctx.expect_gap(
        crosshead,
        frame,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="upper_shoe",
        negative_elem="upper_rail",
        name="upper bearing shoe seats on upper rail face",
    )
    ctx.expect_gap(
        crosshead,
        frame,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lower_shoe",
        negative_elem="lower_rail",
        name="lower bearing shoe seats on lower rail face",
    )
    ctx.expect_overlap(
        crosshead,
        frame,
        axes="xz",
        min_overlap=0.045,
        elem_a="upper_shoe",
        elem_b="upper_rail",
        name="upper shoe remains registered on the rail",
    )
    ctx.expect_gap(
        carriage,
        crosshead,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="carriage_body",
        negative_elem="crosshead_web",
        name="nested carriage is seated on the crosshead face",
    )
    ctx.expect_within(
        carriage,
        crosshead,
        axes="xz",
        inner_elem="carriage_body",
        outer_elem="crosshead_web",
        margin=0.020,
        name="carriage body stays inside the crosshead face envelope",
    )

    rest_crosshead = ctx.part_world_position(crosshead)
    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({crosshead_joint: 0.900, carriage_joint: 0.320}):
        ctx.expect_overlap(
            crosshead,
            frame,
            axes="xz",
            min_overlap=0.045,
            elem_a="upper_shoe",
            elem_b="upper_rail",
            name="extended crosshead still rides on the rail",
        )
        ctx.expect_within(
            carriage,
            crosshead,
            axes="xz",
            inner_elem="carriage_body",
            outer_elem="crosshead_web",
            margin=0.020,
            name="raised carriage remains nested in the crosshead face",
        )
        extended_crosshead = ctx.part_world_position(crosshead)
        raised_carriage = ctx.part_world_position(carriage)

    ctx.check(
        "crosshead prismatic joint moves horizontally",
        rest_crosshead is not None
        and extended_crosshead is not None
        and extended_crosshead[0] > rest_crosshead[0] + 0.85,
        details=f"rest={rest_crosshead}, extended={extended_crosshead}",
    )
    ctx.check(
        "nested carriage prismatic joint moves vertically",
        rest_carriage is not None
        and raised_carriage is not None
        and raised_carriage[2] > rest_carriage[2] + 0.30,
        details=f"rest={rest_carriage}, raised={raised_carriage}",
    )

    return ctx.report()


object_model = build_object_model()
