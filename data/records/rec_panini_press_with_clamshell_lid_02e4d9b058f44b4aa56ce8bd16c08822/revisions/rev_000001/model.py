from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _chamfered_box(size: tuple[float, float, float], chamfer: float, name: str):
    shape = cq.Workplane("XY").box(*size)
    if chamfer > 0.0:
        shape = shape.edges().chamfer(chamfer)
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.12)


def _rounded_section(width: float, depth: float, z: float, radius: float):
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=10)]


def _domed_platen_mesh(name: str):
    sections = [
        _rounded_section(0.500, 0.360, -0.036, 0.046),
        _rounded_section(0.492, 0.352, -0.006, 0.052),
        _rounded_section(0.462, 0.318, 0.032, 0.062),
        _rounded_section(0.405, 0.248, 0.062, 0.070),
    ]
    return mesh_from_geometry(LoftGeometry(sections, cap=True, closed=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_panini_press")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_iron = model.material("seasoned_cast_iron", rgba=(0.035, 0.035, 0.032, 1.0))
    black = model.material("black_phenolic", rgba=(0.005, 0.005, 0.004, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    red = model.material("red_index", rgba=(0.85, 0.05, 0.025, 1.0))

    # Root frame: +Y is the front slot direction, +X is the right side panel, +Z is up.
    base = model.part("base")
    base.visual(
        Box((0.585, 0.440, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=stainless,
        name="bottom_pan",
    )
    for sx, name in [(-1.0, "side_panel_0"), (1.0, "side_panel_1")]:
        base.visual(
            _chamfered_box((0.026, 0.430, 0.118), 0.004, name),
            origin=Origin(xyz=(sx * 0.280, 0.0, 0.082)),
            material=stainless,
            name=name,
        )
    base.visual(
        _chamfered_box((0.585, 0.036, 0.060), 0.004, "front_apron"),
        origin=Origin(xyz=(0.0, 0.222, 0.105)),
        material=stainless,
        name="front_apron",
    )
    base.visual(
        Box((0.585, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.226, 0.038)),
        material=stainless,
        name="slot_lower_lip",
    )
    base.visual(
        _chamfered_box((0.585, 0.032, 0.118), 0.004, "rear_apron"),
        origin=Origin(xyz=(0.0, -0.222, 0.090)),
        material=stainless,
        name="rear_apron",
    )

    # Lower grill tray: a dark ribbed cooking plate captured in a stainless rim.
    base.visual(
        Box((0.500, 0.320, 0.018)),
        origin=Origin(xyz=(0.0, 0.005, 0.139)),
        material=dark_iron,
        name="lower_plate",
    )
    base.visual(
        Box((0.535, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.177, 0.140)),
        material=stainless,
        name="front_grill_rim",
    )
    base.visual(
        Box((0.535, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, -0.167, 0.140)),
        material=stainless,
        name="rear_grill_rim",
    )
    for sx, name in [(-1.0, "side_grill_rim_0"), (1.0, "side_grill_rim_1")]:
        base.visual(
            Box((0.018, 0.342, 0.022)),
            origin=Origin(xyz=(sx * 0.259, 0.005, 0.140)),
            material=stainless,
            name=name,
        )
    for i, n in enumerate(range(-5, 6)):
        if n == 0:
            continue
        x = n * 0.045
        base.visual(
            Box((0.012, 0.285, 0.012)),
            origin=Origin(xyz=(x, 0.006, 0.154)),
            material=dark_iron,
            name=f"lower_rib_{i}",
        )
    base.visual(
        Box((0.012, 0.285, 0.012)),
        origin=Origin(xyz=(0.0, 0.006, 0.154)),
        material=dark_iron,
        name="lower_rib_5",
    )

    # Rear hinge hardware carried by the base.
    for sx, name in [(-1.0, "hinge_bracket_0"), (1.0, "hinge_bracket_1")]:
        base.visual(
            _chamfered_box((0.032, 0.040, 0.128), 0.003, name),
            origin=Origin(xyz=(sx * 0.260, -0.226, 0.173)),
            material=stainless,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.008, length=0.548),
        origin=Origin(xyz=(0.0, -0.226, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_pin",
    )

    for x, y, name in [
        (-0.225, 0.168, "foot_0"),
        (0.225, 0.168, "foot_1"),
        (-0.225, -0.168, "foot_2"),
        (0.225, -0.168, "foot_3"),
    ]:
        base.visual(
            Cylinder(radius=0.026, length=0.028),
            origin=Origin(xyz=(x, y, 0.003)),
            material=rubber,
            name=name,
        )

    # Static temperature scale marks printed on the right side panel.
    for i, (y, z, h) in enumerate([(0.000, 0.088, 0.018), (0.048, 0.138, 0.020), (0.096, 0.088, 0.018)]):
        base.visual(
            Box((0.003, 0.006, h)),
            origin=Origin(xyz=(0.2915, y, z)),
            material=black,
            name=f"dial_mark_{i}",
        )
    base.visual(
        Box((0.003, 0.007, 0.020)),
        origin=Origin(xyz=(0.2915, 0.112, 0.126)),
        material=red,
        name="dial_hot_mark",
    )

    upper = model.part("upper_platen")
    upper.visual(
        _domed_platen_mesh("domed_shell"),
        origin=Origin(xyz=(0.0, 0.200, 0.0)),
        material=stainless,
        name="domed_shell",
    )
    upper.visual(
        Cylinder(radius=0.014, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_sleeve",
    )
    upper.visual(
        Box((0.430, 0.026, 0.026)),
        origin=Origin(xyz=(0.0, 0.014, -0.027)),
        material=stainless,
        name="rear_cover_flange",
    )
    for sx, name in [(-1.0, "hinge_lug_0"), (1.0, "hinge_lug_1")]:
        upper.visual(
            Box((0.070, 0.036, 0.030)),
            origin=Origin(xyz=(sx * 0.165, 0.020, -0.028)),
            material=stainless,
            name=name,
        )
    upper.visual(
        Box((0.455, 0.288, 0.014)),
        origin=Origin(xyz=(0.0, 0.205, -0.043)),
        material=dark_iron,
        name="upper_plate",
    )
    for i, n in enumerate(range(-5, 6)):
        if n == 0:
            continue
        x = n * 0.043
        upper.visual(
            Box((0.011, 0.258, 0.010)),
            origin=Origin(xyz=(x, 0.205, -0.055)),
            material=dark_iron,
            name=f"upper_rib_{i}",
        )
    upper.visual(
        Box((0.011, 0.258, 0.010)),
        origin=Origin(xyz=(0.0, 0.205, -0.055)),
        material=dark_iron,
        name="upper_rib_5",
    )
    # Centered heat-resistant handle with feet visibly fastened to the domed cover.
    for sx, name in [(-1.0, "handle_post_0"), (1.0, "handle_post_1")]:
        upper.visual(
            Box((0.030, 0.038, 0.070)),
            origin=Origin(xyz=(sx * 0.120, 0.210, 0.082)),
            material=black,
            name=name,
        )
        upper.visual(
            Box((0.070, 0.052, 0.014)),
            origin=Origin(xyz=(sx * 0.120, 0.210, 0.067)),
            material=black,
            name=f"handle_foot_{0 if sx < 0 else 1}",
        )
    upper.visual(
        Cylinder(radius=0.020, length=0.330),
        origin=Origin(xyz=(0.0, 0.210, 0.120), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="handle_grip",
    )

    model.articulation(
        "upper_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, -0.226, 0.225)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=0.0, upper=1.15),
    )

    dial = model.part("temperature_dial")
    knob = KnobGeometry(
        0.058,
        0.030,
        body_style="skirted",
        base_diameter=0.066,
        top_diameter=0.048,
        edge_radius=0.0015,
        grip=KnobGrip(style="fluted", count=24, depth=0.0016),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
    )
    dial.visual(
        mesh_from_geometry(knob, "temperature_dial"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="dial_cap",
    )
    model.articulation(
        "temperature_dial_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.308, 0.048, 0.088)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    tray = model.part("drip_tray")
    tray.visual(
        Box((0.410, 0.280, 0.012)),
        origin=Origin(xyz=(0.0, -0.110, 0.000)),
        material=stainless,
        name="tray_floor",
    )
    for sx, name in [(-1.0, "tray_side_0"), (1.0, "tray_side_1")]:
        tray.visual(
            Box((0.014, 0.280, 0.026)),
            origin=Origin(xyz=(sx * 0.205, -0.110, 0.010)),
            material=stainless,
            name=name,
        )
    tray.visual(
        Box((0.430, 0.016, 0.030)),
        origin=Origin(xyz=(0.0, 0.038, 0.012)),
        material=stainless,
        name="tray_front_lip",
    )
    tray.visual(
        Box((0.150, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.051, 0.018)),
        material=black,
        name="tray_pull",
    )
    model.articulation(
        "drip_tray_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.214, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.180),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper = object_model.get_part("upper_platen")
    dial = object_model.get_part("temperature_dial")
    tray = object_model.get_part("drip_tray")
    hinge = object_model.get_articulation("upper_hinge")
    tray_slide = object_model.get_articulation("drip_tray_slide")
    dial_spin = object_model.get_articulation("temperature_dial_spin")

    ctx.allow_overlap(
        base,
        upper,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The hinge pin is intentionally captured inside the rotating rear sleeve.",
    )
    ctx.expect_overlap(
        base,
        upper,
        axes="xyz",
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.010,
        name="hinge pin is retained inside sleeve",
    )

    ctx.expect_overlap(
        base,
        upper,
        axes="xy",
        elem_a="lower_plate",
        elem_b="upper_plate",
        min_overlap=0.240,
        name="upper and lower grill plates oppose each other",
    )
    ctx.expect_gap(
        upper,
        base,
        axis="z",
        positive_elem="upper_rib_5",
        negative_elem="lower_rib_5",
        min_gap=0.001,
        max_gap=0.012,
        name="closed grill ribs have a small cooking gap",
    )
    ctx.expect_within(
        tray,
        base,
        axes="x",
        inner_elem="tray_floor",
        outer_elem="front_apron",
        margin=0.010,
        name="drip tray fits within front slot width",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="y",
        elem_a="tray_floor",
        elem_b="front_apron",
        min_overlap=0.010,
        name="closed drip tray remains inserted behind apron",
    )

    closed_aabb = ctx.part_world_aabb(upper)
    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({hinge: 0.85, tray_slide: 0.160, dial_spin: math.pi}):
        opened_aabb = ctx.part_world_aabb(upper)
        extended_tray_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            base,
            axes="x",
            elem_a="tray_floor",
            elem_b="front_apron",
            min_overlap=0.300,
            name="extended drip tray stays laterally guided by the slot",
        )

    ctx.check(
        "upper platen opens upward on rear hinge",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.09,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )
    ctx.check(
        "drip tray slides out toward front",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] > rest_tray_pos[1] + 0.14,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )
    ctx.check(
        "dial is a side-mounted continuous rotary control",
        getattr(dial_spin, "articulation_type", None) == ArticulationType.CONTINUOUS
        and dial is not None,
        details=str(dial_spin),
    )

    return ctx.report()


object_model = build_object_model()
