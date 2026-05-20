from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


ENGINE_Z = 0.480
CASE_INNER_RADIUS = 0.165
ROTOR_CENTER_X = -0.300
ROTOR_RADIUS = 0.145
ROTOR_THICKNESS = 0.032


def _tube_x(outer_radius: float, inner_radius: float, length: float, center_x: float = 0.0):
    """Open annular tube with its axis along local X."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length, both=True)
        .translate((center_x, 0.0, 0.0))
    )


def _frustum_tube_x(
    *,
    start_x: float,
    length: float,
    outer_start: float,
    outer_end: float,
    inner_start: float,
    inner_end: float,
):
    """Hollow conical tube running from start_x to start_x + length."""
    outer = (
        cq.Workplane("YZ")
        .circle(outer_start)
        .workplane(offset=length)
        .circle(outer_end)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("YZ")
        .circle(inner_start)
        .workplane(offset=length)
        .circle(inner_end)
        .loft(combine=True)
    )
    return outer.cut(inner).translate((start_x, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_turbojet_module")

    brushed_aluminum = Material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.013, 1.0))
    stand_paint = Material("maintenance_blue", rgba=(0.08, 0.18, 0.34, 1.0))
    rotor_metal = Material("compressor_titanium", rgba=(0.42, 0.46, 0.50, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.86, 0.46, 0.035)),
        origin=Origin(xyz=(0.03, 0.0, 0.0175)),
        material=stand_paint,
        name="base_plate",
    )
    for name, x in (("cross_beam_0", -0.20), ("cross_beam_1", 0.16)):
        stand.visual(Box((0.095, 0.40, 0.045)), origin=Origin(xyz=(x, 0.0, 0.0575)), material=stand_paint, name=name)
    for name, x, y in (
        ("post_0_0", -0.20, -0.145),
        ("post_0_1", -0.20, 0.145),
        ("post_1_0", 0.16, -0.145),
        ("post_1_1", 0.16, 0.145),
    ):
        stand.visual(Box((0.050, 0.050, 0.180)), origin=Origin(xyz=(x, y, 0.1575)), material=stand_paint, name=name)
    stand.visual(
        Box((0.105, 0.250, 0.025)),
        origin=Origin(xyz=(-0.20, 0.0, 0.2515)),
        material=rubber,
        name="saddle_pad_0",
    )
    stand.visual(
        Box((0.105, 0.250, 0.025)),
        origin=Origin(xyz=(0.16, 0.0, 0.2515)),
        material=rubber,
        name="saddle_pad_1",
    )

    case = model.part("case")
    main_shell = (
        _tube_x(0.210, CASE_INNER_RADIUS, 0.720)
        .union(_tube_x(0.225, 0.154, 0.055, center_x=-0.372))
        .union(_frustum_tube_x(
            start_x=0.360,
            length=0.150,
            outer_start=0.188,
            outer_end=0.130,
            inner_start=0.148,
            inner_end=0.102,
        ))
        .union(cq.Workplane("XY").box(0.105, 0.045, 0.030).translate((-0.02, 0.0, 0.224)))
    )
    case.visual(
        mesh_from_cadquery(main_shell, "main_shell", tolerance=0.001, angular_tolerance=0.08),
        material=brushed_aluminum,
        name="main_shell",
    )
    case.visual(
        mesh_from_cadquery(_tube_x(0.216, 0.164, 0.034, center_x=-0.20), "saddle_band_0", tolerance=0.001, angular_tolerance=0.08),
        material=dark_steel,
        name="saddle_band_0",
    )
    case.visual(
        mesh_from_cadquery(_tube_x(0.216, 0.164, 0.034, center_x=0.16), "saddle_band_1", tolerance=0.001, angular_tolerance=0.08),
        material=dark_steel,
        name="saddle_band_1",
    )
    case.visual(
        Box((0.120, 0.190, 0.016)),
        origin=Origin(xyz=(-0.20, 0.0, -0.208)),
        material=dark_steel,
        name="support_shoe_0",
    )
    case.visual(
        Box((0.120, 0.190, 0.016)),
        origin=Origin(xyz=(0.16, 0.0, -0.208)),
        material=dark_steel,
        name="support_shoe_1",
    )

    # A fixed center shaft, spider struts, and retaining collars live inside the
    # intake.  The rotor is a separate continuous child that spins around this
    # stationary shaft and is axially captured between the two collars.
    case.visual(
        Cylinder(radius=0.015, length=0.360),
        origin=Origin(xyz=(ROTOR_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="center_shaft",
    )
    for name, y, z, size in (
        ("spider_y", 0.0, 0.0, (0.016, 0.350, 0.014)),
        ("spider_z", 0.0, 0.0, (0.016, 0.014, 0.350)),
    ):
        case.visual(
            Box(size),
            origin=Origin(xyz=(-0.225, y, z)),
            material=dark_steel,
            name=name,
        )
    case.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(ROTOR_CENTER_X - 0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_collar",
    )
    case.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(ROTOR_CENTER_X + 0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_collar",
    )
    case.visual(
        Cylinder(radius=0.145, length=0.010),
        origin=Origin(xyz=(-0.175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=Material("intake_shadow", rgba=(0.0, 0.0, 0.0, 1.0)),
        name="dark_compressor_backing",
    )

    rotor = model.part("rotor")
    rotor_geometry = FanRotorGeometry(
        ROTOR_RADIUS,
        0.050,
        13,
        thickness=ROTOR_THICKNESS,
        blade_pitch_deg=34.0,
        blade_sweep_deg=28.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.18, tip_clearance=0.006),
        hub=FanRotorHub(style="spinner", bore_diameter=0.044),
    )
    rotor.visual(
        mesh_from_geometry(rotor_geometry, "compressor_rotor"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rotor_metal,
        name="compressor_rotor",
    )
    rotor.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rotor_metal,
        name="bearing_sleeve",
    )

    model.articulation(
        "stand_to_case",
        ArticulationType.FIXED,
        parent=stand,
        child=case,
        origin=Origin(xyz=(0.0, 0.0, ENGINE_Z)),
    )
    model.articulation(
        "case_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=rotor,
        origin=Origin(xyz=(ROTOR_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=250.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    rotor = object_model.get_part("rotor")
    stand = object_model.get_part("stand")
    spin = object_model.get_articulation("case_to_rotor")

    ctx.allow_overlap(
        case,
        rotor,
        elem_a="center_shaft",
        elem_b="bearing_sleeve",
        reason="The visible compressor rotor is clipped to a fixed center shaft by a bearing sleeve proxy.",
    )

    ctx.check(
        "compressor rotor uses continuous spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type was {spin.articulation_type}",
    )

    ctx.expect_within(
        rotor,
        case,
        axes="yz",
        inner_elem="compressor_rotor",
        outer_elem="main_shell",
        margin=0.002,
        name="rotor stays inside intake case envelope",
    )
    ctx.expect_overlap(
        case,
        rotor,
        axes="yz",
        elem_a="center_shaft",
        elem_b="compressor_rotor",
        min_overlap=0.020,
        name="rotor remains centered on shaft footprint",
    )
    ctx.expect_within(
        case,
        rotor,
        axes="yz",
        inner_elem="center_shaft",
        outer_elem="bearing_sleeve",
        margin=0.0,
        name="shaft is nested inside rotor sleeve",
    )
    ctx.expect_overlap(
        case,
        rotor,
        axes="x",
        elem_a="center_shaft",
        elem_b="bearing_sleeve",
        min_overlap=0.045,
        name="rotor sleeve has retained shaft insertion",
    )
    ctx.expect_gap(
        rotor,
        case,
        axis="x",
        positive_elem="compressor_rotor",
        negative_elem="front_collar",
        max_gap=0.025,
        max_penetration=0.0,
        name="front collar captures rotor without collision",
    )
    ctx.expect_gap(
        case,
        rotor,
        axis="x",
        positive_elem="rear_collar",
        negative_elem="compressor_rotor",
        max_gap=0.025,
        max_penetration=0.0,
        name="rear collar captures rotor without collision",
    )
    ctx.expect_gap(
        case,
        stand,
        axis="z",
        positive_elem="support_shoe_0",
        negative_elem="saddle_pad_0",
        max_gap=0.004,
        max_penetration=0.001,
        name="front cradle pad supports case band",
    )
    ctx.expect_gap(
        case,
        stand,
        axis="z",
        positive_elem="support_shoe_1",
        negative_elem="saddle_pad_1",
        max_gap=0.004,
        max_penetration=0.001,
        name="rear cradle pad supports case band",
    )

    with ctx.pose({spin: 1.25}):
        ctx.expect_within(
            rotor,
            case,
            axes="yz",
            inner_elem="compressor_rotor",
            outer_elem="main_shell",
            margin=0.002,
            name="spinning rotor remains radially captured",
        )
        ctx.expect_overlap(
            case,
            rotor,
            axes="yz",
            elem_a="center_shaft",
            elem_b="compressor_rotor",
            min_overlap=0.020,
            name="spinning rotor stays clipped to shaft",
        )

    return ctx.report()


object_model = build_object_model()
