from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges().fillet(radius)
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.12)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_bulk_candy_vender")

    red = model.material("painted_red", rgba=(0.78, 0.03, 0.02, 1.0))
    dark_red = model.material("dark_red_shadow", rgba=(0.38, 0.01, 0.01, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.78, 0.70, 1.0))
    dark = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("clear_globe", rgba=(0.70, 0.94, 1.0, 0.34))
    white = model.material("white_price_card", rgba=(0.95, 0.91, 0.78, 1.0))
    candy_red = model.material("candy_red", rgba=(0.95, 0.04, 0.03, 1.0))
    candy_yellow = model.material("candy_yellow", rgba=(1.0, 0.85, 0.04, 1.0))
    candy_green = model.material("candy_green", rgba=(0.05, 0.72, 0.15, 1.0))
    candy_blue = model.material("candy_blue", rgba=(0.03, 0.24, 0.95, 1.0))
    candy_orange = model.material("candy_orange", rgba=(1.0, 0.36, 0.02, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_box_mesh((0.34, 0.34, 0.15), 0.018, "square_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=red,
        name="base_shell",
    )
    body.visual(
        Cylinder(radius=0.148, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=chrome,
        name="top_trim",
    )
    body.visual(
        Cylinder(radius=0.122, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=red,
        name="globe_seat",
    )

    globe_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.122, 0.165),
            (0.154, 0.186),
            (0.154, 0.383),
            (0.135, 0.418),
            (0.075, 0.452),
            (0.018, 0.462),
        ],
        inner_profile=[
            (0.112, 0.171),
            (0.145, 0.191),
            (0.145, 0.379),
            (0.126, 0.410),
            (0.070, 0.441),
            (0.016, 0.451),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(globe_shell, "clear_cylindrical_globe"),
        material=glass,
        name="globe_shell",
    )
    body.visual(
        Cylinder(radius=0.085, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.466)),
        material=chrome,
        name="top_cap",
    )
    body.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.494)),
        material=red,
        name="cap_finial",
    )

    body.visual(
        Cylinder(radius=0.112, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.187)),
        material=candy_orange,
        name="candy_bed",
    )
    candy_positions = [
        ((-0.058, -0.052, 0.207), candy_red, 0.017),
        ((-0.022, -0.069, 0.209), candy_yellow, 0.016),
        ((0.028, -0.060, 0.208), candy_green, 0.017),
        ((0.063, -0.034, 0.210), candy_blue, 0.016),
        ((-0.071, 0.003, 0.209), candy_blue, 0.016),
        ((-0.031, 0.000, 0.222), candy_orange, 0.018),
        ((0.021, 0.002, 0.223), candy_red, 0.017),
        ((0.061, 0.023, 0.210), candy_yellow, 0.016),
        ((-0.044, 0.054, 0.210), candy_green, 0.016),
        ((0.006, 0.062, 0.209), candy_blue, 0.016),
        ((0.043, 0.049, 0.224), candy_orange, 0.017),
        ((-0.006, 0.029, 0.238), candy_yellow, 0.016),
    ]
    for index, (xyz, material, radius) in enumerate(candy_positions):
        body.visual(
            Sphere(radius=radius),
            origin=Origin(xyz=xyz),
            material=material,
            name=f"candy_{index}",
        )

    body.visual(
        _rounded_box_mesh((0.105, 0.160, 0.105), 0.010, "coin_head"),
        origin=Origin(xyz=(0.2225, 0.0, 0.136)),
        material=chrome,
        name="coin_head",
    )
    body.visual(
        Box((0.006, 0.078, 0.012)),
        origin=Origin(xyz=(0.2775, 0.0, 0.164)),
        material=dark,
        name="coin_slot",
    )
    body.visual(
        Box((0.006, 0.090, 0.026)),
        origin=Origin(xyz=(0.2775, 0.0, 0.103)),
        material=white,
        name="price_card",
    )
    body.visual(
        Box((0.010, 0.160, 0.012)),
        origin=Origin(xyz=(0.176, 0.0, 0.086)),
        material=dark_red,
        name="door_header",
    )
    body.visual(
        Box((0.010, 0.016, 0.056)),
        origin=Origin(xyz=(0.176, -0.072, 0.047)),
        material=dark_red,
        name="door_jamb_0",
    )
    body.visual(
        Box((0.010, 0.016, 0.056)),
        origin=Origin(xyz=(0.176, 0.072, 0.047)),
        material=dark_red,
        name="door_jamb_1",
    )
    for y in (-0.076, 0.076):
        body.visual(
            Box((0.018, 0.030, 0.006)),
            origin=Origin(xyz=(0.174, y, 0.018)),
            material=chrome,
            name=f"hinge_leaf_{0 if y < 0 else 1}",
        )
        body.visual(
            Cylinder(radius=0.007, length=0.026),
            origin=Origin(xyz=(0.181, y, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"fixed_hinge_knuckle_{0 if y < 0 else 1}",
        )
    for foot_index, (x, y) in enumerate(
        ((-0.115, -0.115), (-0.115, 0.115), (0.115, -0.115), (0.115, 0.115))
    ):
        body.visual(
            Box((0.052, 0.052, 0.012)),
            origin=Origin(xyz=(x, y, -0.006)),
            material=dark,
            name=f"rubber_foot_{foot_index}",
        )

    dispensing_knob = model.part("dispensing_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.088,
            0.030,
            body_style="lobed",
            base_diameter=0.064,
            top_diameter=0.084,
            crown_radius=0.003,
            edge_radius=0.0015,
            grip=KnobGrip(style="ribbed", count=12, depth=0.0013, width=0.003),
            bore=KnobBore(style="round", diameter=0.012),
        ),
        "lobed_dispensing_knob",
    )
    dispensing_knob.visual(
        Cylinder(radius=0.012, length=0.105),
        origin=Origin(xyz=(-0.0325, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="shaft",
    )
    dispensing_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="knob_cap",
    )
    model.articulation(
        "body_to_dispensing_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dispensing_knob,
        origin=Origin(xyz=(0.277, 0.0, 0.136)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    candy_door = model.part("candy_door")
    candy_door.visual(
        Cylinder(radius=0.0065, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    candy_door.visual(
        Box((0.010, 0.116, 0.064)),
        origin=Origin(xyz=(0.006, 0.0, 0.029)),
        material=chrome,
        name="door_panel",
    )
    candy_door.visual(
        Box((0.006, 0.074, 0.018)),
        origin=Origin(xyz=(0.012, 0.0, 0.038)),
        material=dark,
        name="door_pull_recess",
    )
    model.articulation(
        "body_to_candy_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=candy_door,
        origin=Origin(xyz=(0.181, 0.0, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    knob = object_model.get_part("dispensing_knob")
    door = object_model.get_part("candy_door")
    knob_joint = object_model.get_articulation("body_to_dispensing_knob")
    door_joint = object_model.get_articulation("body_to_candy_door")

    ctx.allow_overlap(
        body,
        knob,
        elem_a="coin_head",
        elem_b="shaft",
        reason="The dispensing knob shaft is intentionally captured inside the coin head bearing.",
    )
    ctx.expect_within(
        knob,
        body,
        axes="yz",
        inner_elem="shaft",
        outer_elem="coin_head",
        margin=0.002,
        name="shaft is centered through the coin head",
    )
    ctx.expect_overlap(
        knob,
        body,
        axes="x",
        elem_a="shaft",
        elem_b="coin_head",
        min_overlap=0.060,
        name="shaft passes through the coin head",
    )

    ctx.check(
        "dispensing knob uses a continuous rotary joint",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={knob_joint.articulation_type}",
    )
    ctx.check(
        "dispensing knob axis is horizontal through the front coin head",
        tuple(round(v, 3) for v in knob_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={knob_joint.axis}",
    )

    ctx.expect_gap(
        door,
        body,
        axis="x",
        positive_elem="door_panel",
        negative_elem="base_shell",
        min_gap=0.006,
        max_gap=0.025,
        name="closed candy door sits proud of the base front",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        elem_a="door_panel",
        elem_b="base_shell",
        min_overlap=0.050,
        name="closed candy door covers the front opening",
    )

    rest_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 1.15}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "candy door rotates downward and outward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > rest_aabb[1][0] + 0.025
        and open_aabb[1][2] < rest_aabb[1][2] - 0.012,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
