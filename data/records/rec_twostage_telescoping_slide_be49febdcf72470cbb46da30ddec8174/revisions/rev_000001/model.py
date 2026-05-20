from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_LEN = 0.420
INTER_LEN = 0.360
CARRIAGE_LEN = 0.250
INTER_TRAVEL = 0.180
CARRIAGE_TRAVEL = 0.160


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _x_cylinder(
    x0: float,
    length: float,
    y: float,
    z: float,
    radius: float,
) -> cq.Workplane:
    return cq.Workplane("YZ").center(y, z).circle(radius).extrude(length).translate((x0, 0.0, 0.0))


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _outer_rail_shape() -> cq.Workplane:
    length = OUTER_LEN
    width = 0.070
    height = 0.030
    wall = 0.004
    lip = 0.012
    side_y = width / 2.0 - wall / 2.0
    lip_y = width / 2.0 - wall - lip / 2.0
    shapes = [
        _box((length, width, wall), (length / 2.0, 0.0, wall / 2.0)),
        _box((length, wall, height - wall), (length / 2.0, side_y, wall + (height - wall) / 2.0)),
        _box((length, wall, height - wall), (length / 2.0, -side_y, wall + (height - wall) / 2.0)),
        _box((length, lip, wall), (length / 2.0, lip_y, height - wall / 2.0)),
        _box((length, lip, wall), (length / 2.0, -lip_y, height - wall / 2.0)),
        # Folded/rolled inner beads at the lips make the outer member read as sheet
        # metal rather than a plain rectangular trough.
        _x_cylinder(0.0, length, lip_y - lip / 2.0, height - 0.0045, 0.0020),
        _x_cylinder(0.0, length, -(lip_y - lip / 2.0), height - 0.0045, 0.0020),
        # Stop lugs are carried on the side lips, outside the moving central slot.
        _box((0.014, 0.006, 0.010), (0.018, 0.030, height + 0.0040)),
        _box((0.014, 0.006, 0.010), (0.018, -0.030, height + 0.0040)),
        _box((0.014, 0.006, 0.010), (length - 0.018, 0.030, height + 0.0040)),
        _box((0.014, 0.006, 0.010), (length - 0.018, -0.030, height + 0.0040)),
    ]
    return _union_all(shapes)


def _intermediate_rail_shape() -> cq.Workplane:
    length = INTER_LEN
    x0 = 0.040
    top_z = 0.0215
    flange_z = 0.0130
    lower_z = 0.0075
    shapes = [
        _box((length, 0.034, 0.0030), (x0 + length / 2.0, 0.0, top_z)),
        _box((length, 0.0035, 0.0140), (x0 + length / 2.0, 0.01825, flange_z)),
        _box((length, 0.0035, 0.0140), (x0 + length / 2.0, -0.01825, flange_z)),
        _box((length, 0.0080, 0.0030), (x0 + length / 2.0, 0.0140, lower_z)),
        _box((length, 0.0080, 0.0030), (x0 + length / 2.0, -0.0140, lower_z)),
        _x_cylinder(x0, length, 0.0100, lower_z + 0.0005, 0.0015),
        _x_cylinder(x0, length, -0.0100, lower_z + 0.0005, 0.0015),
        # Outboard raised stop ears are clear of the terminal carriage's central plate.
        _box((0.012, 0.006, 0.007), (x0 + 0.014, 0.0185, 0.0248)),
        _box((0.012, 0.006, 0.007), (x0 + 0.014, -0.0185, 0.0248)),
        _box((0.012, 0.006, 0.007), (x0 + length - 0.014, 0.0185, 0.0248)),
        _box((0.012, 0.006, 0.007), (x0 + length - 0.014, -0.0185, 0.0248)),
    ]
    return _union_all(shapes)


def _carriage_plate_shape() -> cq.Workplane:
    length = CARRIAGE_LEN
    x0 = 0.120
    return _box((length, 0.030, 0.006), (x0 + length / 2.0, 0.0, 0.0290))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_extension_slide")

    zinc = model.material("brushed_zinc", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_zinc = model.material("shadowed_steel", rgba=(0.30, 0.32, 0.32, 1.0))
    polymer = model.material("black_acetal_wear", rgba=(0.025, 0.024, 0.022, 1.0))
    hardware_black = model.material("blackened_recess", rgba=(0.02, 0.02, 0.018, 1.0))

    outer_rail = model.part("outer_rail")
    outer_width = 0.070
    outer_height = 0.030
    outer_wall = 0.004
    outer_lip = 0.012
    outer_side_y = outer_width / 2.0 - outer_wall / 2.0
    outer_lip_y = outer_width / 2.0 - outer_wall / 2.0 - outer_lip / 2.0
    outer_rail.visual(
        Box((OUTER_LEN, outer_width, outer_wall)),
        origin=Origin(xyz=(OUTER_LEN / 2.0, 0.0, outer_wall / 2.0)),
        material=zinc,
        name="bottom_web",
    )
    for idx, y in enumerate((outer_side_y, -outer_side_y)):
        outer_rail.visual(
            Box((OUTER_LEN, outer_wall, outer_height - outer_wall)),
            origin=Origin(xyz=(OUTER_LEN / 2.0, y, outer_wall + (outer_height - outer_wall) / 2.0)),
            material=zinc,
            name=f"side_wall_{idx}",
        )
        outer_rail.visual(
            Box((OUTER_LEN, outer_lip, outer_wall)),
            origin=Origin(xyz=(OUTER_LEN / 2.0, math.copysign(outer_lip_y, y), outer_height - outer_wall / 2.0)),
            material=zinc,
            name=f"return_lip_{idx}",
        )
        outer_rail.visual(
            Cylinder(radius=0.0015, length=OUTER_LEN),
            origin=Origin(xyz=(OUTER_LEN / 2.0, math.copysign(0.0210, y), 0.0258), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=zinc,
            name=f"rolled_bead_{idx}",
        )
    for idx, (x, y) in enumerate(
        (
            (0.018, 0.030),
            (0.018, -0.030),
            (OUTER_LEN - 0.018, 0.030),
            (OUTER_LEN - 0.018, -0.030),
        )
    ):
        outer_rail.visual(
            Box((0.014, 0.006, 0.010)),
            origin=Origin(xyz=(x, y, outer_height + 0.0040)),
            material=zinc,
            name=f"stop_tab_{idx}",
        )
    # Low-friction strips are proud of the bottom web and slightly embedded so
    # they visibly seat into the fixed channel.
    outer_rail.visual(
        Box((OUTER_LEN - 0.050, 0.006, 0.0014)),
        origin=Origin(xyz=(OUTER_LEN / 2.0, -0.014, 0.0047)),
        material=polymer,
        name="wear_strip_0",
    )
    outer_rail.visual(
        Box((OUTER_LEN - 0.050, 0.006, 0.0014)),
        origin=Origin(xyz=(OUTER_LEN / 2.0, 0.014, 0.0047)),
        material=polymer,
        name="wear_strip_1",
    )
    for idx, (x, y) in enumerate(((0.070, 0.023), (OUTER_LEN - 0.070, -0.023))):
        outer_rail.visual(
            Cylinder(radius=0.0040, length=0.0030),
            origin=Origin(xyz=(x, y, 0.0310)),
            material=dark_zinc,
            name=f"rivet_{idx}",
        )

    intermediate_rail = model.part("intermediate_rail")
    inter_x0 = 0.040
    inter_top_z = 0.0215
    inter_flange_z = 0.0130
    inter_lower_z = 0.0069
    intermediate_rail.visual(
        Box((INTER_LEN, 0.034, 0.0030)),
        origin=Origin(xyz=(inter_x0 + INTER_LEN / 2.0, 0.0, inter_top_z)),
        material=dark_zinc,
        name="top_web",
    )
    intermediate_rail.visual(
        Box((INTER_LEN, 0.0035, 0.0140)),
        origin=Origin(xyz=(inter_x0 + INTER_LEN / 2.0, 0.01825, inter_flange_z)),
        material=dark_zinc,
        name="side_web_0",
    )
    intermediate_rail.visual(
        Box((INTER_LEN, 0.0080, 0.0030)),
        origin=Origin(xyz=(inter_x0 + INTER_LEN / 2.0, 0.0140, inter_lower_z)),
        material=dark_zinc,
        name="lower_flange_0",
    )
    intermediate_rail.visual(
        Cylinder(radius=0.0015, length=INTER_LEN),
        origin=Origin(xyz=(inter_x0 + INTER_LEN / 2.0, 0.0100, inter_lower_z + 0.0005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_zinc,
        name="rolled_bead_0",
    )
    intermediate_rail.visual(
        Box((INTER_LEN, 0.0035, 0.0140)),
        origin=Origin(xyz=(inter_x0 + INTER_LEN / 2.0, -0.01825, inter_flange_z)),
        material=dark_zinc,
        name="side_web_1",
    )
    intermediate_rail.visual(
        Box((INTER_LEN, 0.0080, 0.0030)),
        origin=Origin(xyz=(inter_x0 + INTER_LEN / 2.0, -0.0140, inter_lower_z)),
        material=dark_zinc,
        name="lower_flange_1",
    )
    intermediate_rail.visual(
        Cylinder(radius=0.0015, length=INTER_LEN),
        origin=Origin(xyz=(inter_x0 + INTER_LEN / 2.0, -0.0100, inter_lower_z + 0.0005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_zinc,
        name="rolled_bead_1",
    )
    for idx, (x, y) in enumerate(
        (
            (inter_x0 + 0.014, 0.0190),
            (inter_x0 + 0.014, -0.0190),
            (inter_x0 + INTER_LEN - 0.014, 0.0190),
            (inter_x0 + INTER_LEN - 0.014, -0.0190),
        )
    ):
        intermediate_rail.visual(
            Box((0.012, 0.005, 0.006)),
            origin=Origin(xyz=(x, y, 0.0240)),
            material=dark_zinc,
            name=f"stop_ear_{idx}",
        )
    intermediate_rail.visual(
        Box((INTER_LEN - 0.040, 0.0045, 0.0012)),
        origin=Origin(xyz=(0.040 + INTER_LEN / 2.0, -0.011, 0.0236)),
        material=polymer,
        name="wear_strip_0",
    )
    intermediate_rail.visual(
        Box((INTER_LEN - 0.040, 0.0045, 0.0012)),
        origin=Origin(xyz=(0.040 + INTER_LEN / 2.0, 0.011, 0.0236)),
        material=polymer,
        name="wear_strip_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LEN, 0.030, 0.006)),
        origin=Origin(xyz=(0.120 + CARRIAGE_LEN / 2.0, 0.0, 0.0290)),
        material=zinc,
        name="plate",
    )
    carriage.visual(
        Box((CARRIAGE_LEN - 0.040, 0.0040, 0.0030)),
        origin=Origin(xyz=(0.120 + CARRIAGE_LEN / 2.0, -0.011, 0.0257)),
        material=polymer,
        name="guide_rib_0",
    )
    carriage.visual(
        Box((CARRIAGE_LEN - 0.040, 0.0040, 0.0030)),
        origin=Origin(xyz=(0.120 + CARRIAGE_LEN / 2.0, 0.011, 0.0257)),
        material=polymer,
        name="guide_rib_1",
    )
    for idx, x in enumerate((0.170, 0.320)):
        carriage.visual(
            Box((0.046, 0.008, 0.0009)),
            origin=Origin(xyz=(x, 0.0, 0.03245)),
            material=hardware_black,
            name=f"mount_slot_{idx}",
        )
        carriage.visual(
            Cylinder(radius=0.0042, length=0.0010),
            origin=Origin(xyz=(x - 0.023, 0.0, 0.0328)),
            material=hardware_black,
            name=f"slot_round_{idx}_0",
        )
        carriage.visual(
            Cylinder(radius=0.0042, length=0.0010),
            origin=Origin(xyz=(x + 0.023, 0.0, 0.0328)),
            material=hardware_black,
            name=f"slot_round_{idx}_1",
        )

    model.articulation(
        "outer_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=outer_rail,
        child=intermediate_rail,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.35, lower=0.0, upper=INTER_TRAVEL),
    )
    model.articulation(
        "intermediate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=intermediate_rail,
        child=carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=CARRIAGE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_rail")
    intermediate = object_model.get_part("intermediate_rail")
    carriage = object_model.get_part("carriage")
    first_slide = object_model.get_articulation("outer_to_intermediate")
    second_slide = object_model.get_articulation("intermediate_to_carriage")

    with ctx.pose({first_slide: 0.0, second_slide: 0.0}):
        ctx.expect_overlap(
            intermediate,
            outer,
            axes="x",
            min_overlap=0.320,
            name="stowed intermediate remains deeply nested in fixed rail",
        )
        ctx.expect_overlap(
            carriage,
            intermediate,
            axes="x",
            elem_a="plate",
            elem_b="top_web",
            min_overlap=0.220,
            name="stowed carriage overlaps the moving rail support span",
        )
        ctx.expect_within(
            intermediate,
            outer,
            axes="yz",
            margin=0.001,
            name="intermediate section fits inside fixed channel clearance envelope",
        )
        ctx.expect_gap(
            intermediate,
            outer,
            axis="z",
            positive_elem="lower_flange_1",
            negative_elem="wear_strip_0",
            min_gap=0.0,
            max_gap=0.0005,
            name="intermediate is seated on fixed wear strip without penetration",
        )
        ctx.expect_gap(
            carriage,
            intermediate,
            axis="z",
            positive_elem="guide_rib_0",
            negative_elem="wear_strip_0",
            min_gap=0.0,
            max_gap=0.0005,
            name="carriage guide rib seats on intermediate wear strip",
        )

    rest_intermediate = ctx.part_world_position(intermediate)
    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({first_slide: INTER_TRAVEL, second_slide: CARRIAGE_TRAVEL}):
        ctx.expect_overlap(
            intermediate,
            outer,
            axes="x",
            min_overlap=0.120,
            name="extended intermediate still retains insertion in fixed rail",
        )
        ctx.expect_overlap(
            carriage,
            intermediate,
            axes="x",
            elem_a="plate",
            elem_b="top_web",
            min_overlap=0.080,
            name="extended carriage remains supported by intermediate rail",
        )
        ctx.expect_within(
            intermediate,
            outer,
            axes="yz",
            margin=0.001,
            name="extended intermediate stays inside fixed channel width and height",
        )
        ctx.expect_gap(
            carriage,
            intermediate,
            axis="z",
            positive_elem="guide_rib_0",
            negative_elem="wear_strip_0",
            min_gap=0.0,
            max_gap=0.0005,
            name="extended carriage guide rib remains seated on its wear strip",
        )
        extended_intermediate = ctx.part_world_position(intermediate)
        extended_carriage = ctx.part_world_position(carriage)

    ctx.check(
        "serial stages extend along the same rail axis",
        rest_intermediate is not None
        and rest_carriage is not None
        and extended_intermediate is not None
        and extended_carriage is not None
        and extended_intermediate[0] > rest_intermediate[0] + INTER_TRAVEL * 0.95
        and extended_carriage[0] > rest_carriage[0] + (INTER_TRAVEL + CARRIAGE_TRAVEL) * 0.95
        and abs(extended_intermediate[1] - rest_intermediate[1]) < 0.001
        and abs(extended_carriage[2] - rest_carriage[2]) < 0.001,
        details=(
            f"intermediate {rest_intermediate}->{extended_intermediate}, "
            f"carriage {rest_carriage}->{extended_carriage}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
