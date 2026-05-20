from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


PLATE_X = 0.72
PLATE_Y = 0.025
PLATE_Z = 0.42
PLATE_CENTER_Z = 0.22
RAIL_Y = 0.060
RAIL_Z_LOW = 0.170
RAIL_Z_HIGH = 0.300
RAIL_MID_Z = (RAIL_Z_LOW + RAIL_Z_HIGH) / 2.0
CARRIAGE_HOME_X = -0.190
CARRIAGE_TRAVEL = 0.380
CARRIAGE_LEN = 0.115
CARRIAGE_FRONT_Y = 0.095
FACE_JOINT_Y = CARRIAGE_FRONT_Y + 0.001


def _carriage_mesh() -> cq.Workplane:
    """Linear carriage with two real clearance bores around the guide rods."""
    outer_r = 0.026
    inner_r = 0.0185

    def bearing(z_offset: float) -> cq.Workplane:
        # CadQuery's YZ workplane extrudes along global X, matching the rails.
        return (
            cq.Workplane("YZ")
            .circle(outer_r)
            .circle(inner_r)
            .extrude(CARRIAGE_LEN)
            .translate((-CARRIAGE_LEN / 2.0, 0.0, z_offset))
        )

    front_bridge = (
        cq.Workplane("XY")
        .box(CARRIAGE_LEN, 0.072, 0.190)
        .translate((0.0, 0.060, 0.0))
    )
    shallow_recess = (
        cq.Workplane("XY")
        .box(0.070, 0.006, 0.070)
        .translate((0.0, CARRIAGE_FRONT_Y + 0.001, 0.0))
    )

    return bearing(RAIL_Z_HIGH - RAIL_MID_Z).union(
        bearing(RAIL_Z_LOW - RAIL_MID_Z)
    ).union(front_bridge).cut(shallow_recess)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_carriage_rotary_face")

    matte_plate = Material("matte_charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    bracket_black = Material("blackened_bracket", rgba=(0.025, 0.028, 0.030, 1.0))
    polished_steel = Material("polished_steel", rgba=(0.73, 0.75, 0.76, 1.0))
    blue_anodized = Material("blue_anodized_carriage", rgba=(0.05, 0.18, 0.46, 1.0))
    bearing_bronze = Material("bearing_bronze", rgba=(0.68, 0.43, 0.18, 1.0))
    orange_face = Material("orange_rotary_face", rgba=(0.95, 0.42, 0.10, 1.0))
    dark_rubber = Material("black_face_grip", rgba=(0.015, 0.014, 0.013, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((PLATE_X, PLATE_Y, PLATE_Z)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_CENTER_Z)),
        material=matte_plate,
        name="wall_plate",
    )
    side_plate.visual(
        Box((PLATE_X, 0.120, 0.020)),
        origin=Origin(xyz=(0.0, 0.035, 0.010)),
        material=matte_plate,
        name="floor_flange",
    )

    for x in (-0.315, 0.315):
        side_plate.visual(
            Box((0.050, 0.075, 0.190)),
            origin=Origin(xyz=(x, 0.040, RAIL_MID_Z)),
            material=bracket_black,
            name=f"end_bracket_{'rear' if x < 0.0 else 'front'}",
        )

    rail_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    side_plate.visual(
        Cylinder(radius=0.014, length=0.640),
        origin=Origin(xyz=(0.0, RAIL_Y, RAIL_Z_HIGH), rpy=rail_origin.rpy),
        material=polished_steel,
        name="guide_rod_upper",
    )
    side_plate.visual(
        Cylinder(radius=0.014, length=0.640),
        origin=Origin(xyz=(0.0, RAIL_Y, RAIL_Z_LOW), rpy=rail_origin.rpy),
        material=polished_steel,
        name="guide_rod_lower",
    )

    screw_origin = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))
    for i, (x, z) in enumerate(
        ((-0.280, 0.070), (0.280, 0.070), (-0.280, 0.370), (0.280, 0.370))
    ):
        side_plate.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, PLATE_Y / 2.0 + 0.003, z), rpy=screw_origin.rpy),
            material=polished_steel,
            name=f"mount_screw_{i}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_mesh(), "carriage_with_bearing_bores"),
        material=blue_anodized,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.090, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.017, RAIL_Z_HIGH - RAIL_MID_Z)),
        material=bearing_bronze,
        name="upper_bearing_shoe",
    )
    carriage.visual(
        Box((0.090, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.017, RAIL_Z_LOW - RAIL_MID_Z)),
        material=bearing_bronze,
        name="lower_bearing_shoe",
    )

    model.articulation(
        "plate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_HOME_X, RAIL_Y, RAIL_MID_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=0.45, lower=0.0, upper=CARRIAGE_TRAVEL
        ),
    )

    rotary_face = model.part("rotary_face")
    face_cylinder_origin = Origin(xyz=(0.0, 0.011, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0))
    rotary_face.visual(
        Cylinder(radius=0.052, length=0.022),
        origin=face_cylinder_origin,
        material=orange_face,
        name="face_disk",
    )
    rotary_face.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="center_cap",
    )
    rotary_face.visual(
        Box((0.012, 0.007, 0.060)),
        origin=Origin(xyz=(0.0, 0.026, 0.018)),
        material=dark_rubber,
        name="pointer_bar",
    )

    model.articulation(
        "carriage_to_face",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=rotary_face,
        origin=Origin(xyz=(0.0, FACE_JOINT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    carriage = object_model.get_part("carriage")
    rotary_face = object_model.get_part("rotary_face")
    slide = object_model.get_articulation("plate_to_carriage")
    spin = object_model.get_articulation("carriage_to_face")

    ctx.expect_within(
        carriage,
        side_plate,
        axes="z",
        inner_elem="carriage_body",
        outer_elem="wall_plate",
        margin=0.0,
        name="carriage stays within plate height",
    )
    ctx.expect_overlap(
        carriage,
        side_plate,
        axes="x",
        elem_a="carriage_body",
        elem_b="guide_rod_upper",
        min_overlap=0.10,
        name="carriage is engaged on the straight guide",
    )
    ctx.expect_gap(
        rotary_face,
        carriage,
        axis="y",
        positive_elem="face_disk",
        negative_elem="carriage_body",
        min_gap=0.0,
        max_gap=0.003,
        name="rotary face is seated on carriage front",
    )
    ctx.expect_overlap(
        rotary_face,
        carriage,
        axes="xz",
        elem_a="face_disk",
        elem_b="carriage_body",
        min_overlap=0.050,
        name="face disk is centered on carriage front",
    )

    rest_pos = ctx.part_world_position(carriage)
    rest_face_pos = ctx.part_world_position(rotary_face)
    with ctx.pose({slide: CARRIAGE_TRAVEL}):
        extended_pos = ctx.part_world_position(carriage)
        extended_face_pos = ctx.part_world_position(rotary_face)
        ctx.expect_overlap(
            carriage,
            side_plate,
            axes="x",
            elem_a="carriage_body",
            elem_b="guide_rod_upper",
            min_overlap=0.10,
            name="extended carriage remains on guide",
        )
    ctx.check(
        "prismatic joint moves carriage forward along guide",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + CARRIAGE_TRAVEL - 0.005,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "rotary face is carried by translating carriage",
        rest_face_pos is not None
        and extended_face_pos is not None
        and extended_face_pos[0] > rest_face_pos[0] + CARRIAGE_TRAVEL - 0.005,
        details=f"rest={rest_face_pos}, extended={extended_face_pos}",
    )

    def element_center(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) / 2.0,
            (lo[1] + hi[1]) / 2.0,
            (lo[2] + hi[2]) / 2.0,
        )

    pointer_rest = element_center(rotary_face, "pointer_bar")
    with ctx.pose({spin: math.pi / 2.0}):
        pointer_rotated = element_center(rotary_face, "pointer_bar")
    ctx.check(
        "rotary face spins its pointer about the carried front axis",
        pointer_rest is not None
        and pointer_rotated is not None
        and pointer_rotated[0] > pointer_rest[0] + 0.012
        and abs(pointer_rotated[2] - pointer_rest[2]) > 0.012,
        details=f"rest={pointer_rest}, rotated={pointer_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
