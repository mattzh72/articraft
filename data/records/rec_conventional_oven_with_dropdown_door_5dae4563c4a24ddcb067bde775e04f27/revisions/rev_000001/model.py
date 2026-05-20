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


WIDTH = 0.72
DEPTH = 0.44
HEIGHT = 0.38
WALL = 0.035
FRONT_Y = -DEPTH / 2.0
HINGE_Y = FRONT_Y - 0.050
HINGE_Z = 0.075
SPIT_Y = -0.020
SPIT_Z = 0.215


def _box_solid(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _housing_shell():
    """Connected hollow stainless oven body with an open front mouth."""
    front_lip_depth = 0.032
    side_rail = 0.070
    top_rail = 0.058
    bottom_rail = 0.070
    lip_center_y = FRONT_Y - front_lip_depth / 2.0

    solids = [
        # Main hollow box: top, bottom, side walls, and back.
        _box_solid((WIDTH, DEPTH, WALL), (0.0, 0.0, HEIGHT - WALL / 2.0)),
        _box_solid((WIDTH, DEPTH, WALL), (0.0, 0.0, WALL / 2.0)),
        _box_solid((WALL, DEPTH, HEIGHT), (-WIDTH / 2.0 + WALL / 2.0, 0.0, HEIGHT / 2.0)),
        _box_solid((WALL, DEPTH, HEIGHT), (WIDTH / 2.0 - WALL / 2.0, 0.0, HEIGHT / 2.0)),
        _box_solid((WIDTH, WALL, HEIGHT), (0.0, DEPTH / 2.0 - WALL / 2.0, HEIGHT / 2.0)),
        # Proud front stainless frame around the glass door opening.
        _box_solid((WIDTH, front_lip_depth, top_rail), (0.0, lip_center_y, HEIGHT - top_rail / 2.0)),
        _box_solid((WIDTH, front_lip_depth, bottom_rail), (0.0, lip_center_y, bottom_rail / 2.0)),
        _box_solid((side_rail, front_lip_depth, HEIGHT), (-WIDTH / 2.0 + side_rail / 2.0, lip_center_y, HEIGHT / 2.0)),
        _box_solid((side_rail, front_lip_depth, HEIGHT), (WIDTH / 2.0 - side_rail / 2.0, lip_center_y, HEIGHT / 2.0)),
        # A shallow lower hinge rail visibly carries the drop-down door hinge.
        _box_solid((0.62, 0.025, 0.025), (0.0, FRONT_Y - 0.017, HINGE_Z)),
    ]
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _door_frame():
    """One-piece metal perimeter frame for the glass drop-down door."""
    door_w = 0.62
    door_h = 0.292
    rail = 0.040
    thick = 0.026
    bottom_z = 0.015

    solids = [
        _box_solid((door_w, thick, rail), (0.0, 0.0, bottom_z + rail / 2.0)),
        _box_solid((door_w, thick, rail), (0.0, 0.0, bottom_z + door_h - rail / 2.0)),
        _box_solid((rail, thick, door_h), (-door_w / 2.0 + rail / 2.0, 0.0, bottom_z + door_h / 2.0)),
        _box_solid((rail, thick, door_h), (door_w / 2.0 - rail / 2.0, 0.0, bottom_z + door_h / 2.0)),
        # A small knuckle fused to the bottom rail, centered on the hinge line.
        cq.Workplane("YZ")
        .circle(0.012)
        .extrude(0.46)
        .translate((-0.23, 0.0088, 0.004)),
    ]
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotisserie_oven")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_liner = model.material("dark_enamel_liner", rgba=(0.025, 0.026, 0.026, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.26, 0.30, 0.38))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    warm_element = model.material("heating_element_red", rgba=(0.80, 0.10, 0.04, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "stainless_housing", tolerance=0.001),
        material=stainless,
        name="stainless_shell",
    )

    # Dark inner panels make the body read as a hollow oven cavity rather than a solid block.
    housing.visual(
        Box((WIDTH - 2 * WALL - 0.012, 0.006, HEIGHT - 2 * WALL - 0.020)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - WALL - 0.003, HEIGHT / 2.0)),
        material=dark_liner,
        name="rear_liner",
    )
    housing.visual(
        Box((WIDTH - 2 * WALL - 0.010, DEPTH - WALL - 0.030, 0.006)),
        origin=Origin(xyz=(0.0, -0.010, HEIGHT - WALL - 0.003)),
        material=dark_liner,
        name="top_liner",
    )
    housing.visual(
        Box((WIDTH - 2 * WALL - 0.010, DEPTH - WALL - 0.030, 0.006)),
        origin=Origin(xyz=(0.0, -0.010, WALL + 0.003)),
        material=dark_liner,
        name="bottom_liner",
    )

    # The rotisserie axle is supported by visible end sockets inside the side walls.
    housing.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(
            xyz=(-(WIDTH / 2.0 - WALL - 0.007), SPIT_Y, SPIT_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="socket_0",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(
            xyz=((WIDTH / 2.0 - WALL - 0.007), SPIT_Y, SPIT_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="socket_1",
    )

    # Fixed heating tubes and side rack rails are anchored into the shell walls.
    for name, z in (("upper_element", 0.302), ("lower_element", 0.108)):
        housing.visual(
            Cylinder(radius=0.006, length=WIDTH - 0.050),
            origin=Origin(xyz=(0.0, 0.135, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_element,
            name=name,
        )
    for side, sx in (("rack_rail_0", -1.0), ("rack_rail_1", 1.0)):
        housing.visual(
            Box((0.010, 0.300, 0.012)),
            origin=Origin(xyz=(sx * (WIDTH / 2.0 - WALL), -0.010, 0.160)),
            material=stainless,
            name=side,
        )

    for idx, (x, y) in enumerate(
        (
            (-0.265, -0.145),
            (0.265, -0.145),
            (-0.265, 0.145),
            (0.265, 0.145),
        )
    ):
        housing.visual(
            Cylinder(radius=0.030, length=0.024),
            origin=Origin(xyz=(x, y, -0.012)),
            material=black_rubber,
            name=f"foot_{idx}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_frame(), "door_frame", tolerance=0.0008),
        material=stainless,
        name="door_frame",
    )
    door.visual(
        Box((0.545, 0.006, 0.230)),
        origin=Origin(xyz=(0.0, -0.004, 0.170)),
        material=smoked_glass,
        name="glass_panel",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.480),
        origin=Origin(xyz=(0.0, -0.064, 0.188), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="bar_handle",
    )
    for name, x in (("handle_post_0", -0.205), ("handle_post_1", 0.205)):
        door.visual(
            Cylinder(radius=0.008, length=0.066),
            origin=Origin(xyz=(x, -0.037, 0.188), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=name,
        )

    spit = model.part("spit")
    rod_length = WIDTH - 2 * WALL - 0.028
    spit.visual(
        Cylinder(radius=0.006, length=rod_length),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="spit_rod",
    )
    for name, x, sign in (("fork_0", -0.135, 1.0), ("fork_1", 0.135, -1.0)):
        spit.visual(
            Cylinder(radius=0.017, length=0.024),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name=f"{name}_collar",
        )
        for dz in (-0.018, 0.018):
            spit.visual(
                Box((0.118, 0.006, 0.006)),
                origin=Origin(xyz=(x + sign * 0.050, 0.010, dz * 0.667)),
                material=stainless,
                name=f"{name}_tine_{'low' if dz < 0 else 'high'}",
            )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.55),
    )
    model.articulation(
        "spit_axle",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=spit,
        origin=Origin(xyz=(0.0, SPIT_Y, SPIT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    spit = object_model.get_part("spit")
    door_hinge = object_model.get_articulation("door_hinge")
    spit_axle = object_model.get_articulation("spit_axle")

    ctx.allow_overlap(
        housing,
        door,
        elem_a="stainless_shell",
        elem_b="door_frame",
        reason="The lower door hinge knuckle is lightly captured against the stainless hinge rail.",
    )

    def coord(vec, index: int) -> float:
        names = ("x", "y", "z")
        if hasattr(vec, names[index]):
            return float(getattr(vec, names[index]))
        return float(vec[index])

    ctx.check(
        "door uses a bottom revolute hinge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in door_hinge.axis) == (1.0, 0.0, 0.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > 1.4,
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )
    ctx.check(
        "spit uses a continuous horizontal axle",
        spit_axle.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in spit_axle.axis) == (1.0, 0.0, 0.0),
        details=f"type={spit_axle.articulation_type}, axis={spit_axle.axis}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            housing,
            door,
            axis="y",
            positive_elem="stainless_shell",
            negative_elem="glass_panel",
            min_gap=0.010,
            max_gap=0.024,
            name="closed glass panel sits proud of stainless face",
        )
        ctx.expect_gap(
            housing,
            door,
            axis="y",
            positive_elem="stainless_shell",
            negative_elem="door_frame",
            max_penetration=0.004,
            name="hinge knuckle is only lightly seated in hinge rail",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="xz",
            elem_a="door_frame",
            elem_b="stainless_shell",
            min_overlap=0.20,
            name="closed glass door covers the oven mouth",
        )
        ctx.expect_within(
            door,
            door,
            axes="xz",
            inner_elem="glass_panel",
            outer_elem="door_frame",
            margin=0.002,
            name="glass pane is seated inside the metal door frame",
        )

    closed_box = ctx.part_element_world_aabb(door, elem="door_frame")
    with ctx.pose({door_hinge: 1.20}):
        opened_box = ctx.part_element_world_aabb(door, elem="door_frame")
    ctx.check(
        "positive hinge angle drops the door outward",
        closed_box is not None
        and opened_box is not None
        and coord(opened_box[0], 1) < coord(closed_box[0], 1) - 0.12
        and coord(opened_box[1], 2) < coord(closed_box[1], 2) - 0.09,
        details=f"closed={closed_box}, opened={opened_box}",
    )

    ctx.expect_within(
        spit,
        housing,
        axes="yz",
        inner_elem="spit_rod",
        outer_elem="stainless_shell",
        margin=0.0,
        name="spit rod runs through the oven cavity",
    )
    ctx.expect_gap(
        spit,
        housing,
        axis="x",
        positive_elem="spit_rod",
        negative_elem="socket_0",
        min_gap=0.0,
        max_gap=0.004,
        name="spit tip nearly seats in one end socket",
    )
    ctx.expect_gap(
        housing,
        spit,
        axis="x",
        positive_elem="socket_1",
        negative_elem="spit_rod",
        min_gap=0.0,
        max_gap=0.004,
        name="spit tip nearly seats in opposite end socket",
    )

    return ctx.report()


object_model = build_object_model()
