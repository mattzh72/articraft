from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="survey_head_yaw_module")

    cast_metal = model.material("cast_metal", rgba=(0.45, 0.47, 0.46, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.05, 0.055, 0.055, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    bearing_steel = model.material("brushed_bearing_steel", rgba=(0.68, 0.69, 0.66, 1.0))
    glass = model.material("dark_blue_glass", rgba=(0.06, 0.13, 0.20, 0.82))
    survey_yellow = model.material("survey_yellow_trim", rgba=(0.95, 0.72, 0.10, 1.0))

    pedestal = model.part("pedestal")
    pedestal_profile = [
        (0.0, 0.000),
        (0.170, 0.000),
        (0.170, 0.018),
        (0.135, 0.035),
        (0.083, 0.095),
        (0.060, 0.185),
        (0.066, 0.215),
        (0.094, 0.236),
        (0.104, 0.252),
        (0.104, 0.260),
        (0.0, 0.260),
    ]
    pedestal.visual(
        mesh_from_geometry(LatheGeometry(pedestal_profile, segments=72), "pedestal_body"),
        material=cast_metal,
        name="pedestal_body",
    )
    pedestal.visual(
        Cylinder(radius=0.118, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=bearing_steel,
        name="bearing_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.045, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.281)),
        material=black_rubber,
        name="center_socket",
    )

    instrument_pad = model.part("instrument_pad")
    instrument_pad.visual(
        Cylinder(radius=0.088, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_metal,
        name="head_drum",
    )
    instrument_pad.visual(
        Cylinder(radius=0.050, length=0.033),
        origin=Origin(xyz=(0.0, 0.0, 0.0765)),
        material=dark_metal,
        name="neck_post",
    )
    instrument_pad.visual(
        Box((0.170, 0.074, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=dark_metal,
        name="pad_saddle",
    )
    pad_body_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.430, 0.110, 0.018, corner_segments=8), 0.046),
        "pad_body",
    )
    instrument_pad.visual(
        pad_body_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=dark_metal,
        name="pad_body",
    )
    top_window_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.325, 0.058, 0.010, corner_segments=6), 0.006),
        "top_window",
    )
    instrument_pad.visual(
        top_window_mesh,
        origin=Origin(xyz=(0.018, 0.0, 0.1515)),
        material=glass,
        name="top_window",
    )
    instrument_pad.visual(
        Box((0.060, 0.116, 0.014)),
        origin=Origin(xyz=(-0.196, 0.0, 0.126)),
        material=survey_yellow,
        name="end_cap_0",
    )
    instrument_pad.visual(
        Box((0.060, 0.116, 0.014)),
        origin=Origin(xyz=(0.196, 0.0, 0.126)),
        material=survey_yellow,
        name="end_cap_1",
    )

    model.articulation(
        "pedestal_to_pad",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=instrument_pad,
        origin=Origin(xyz=(0.0, 0.0, 0.292)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    instrument_pad = object_model.get_part("instrument_pad")
    yaw = object_model.get_articulation("pedestal_to_pad")

    ctx.expect_gap(
        instrument_pad,
        pedestal,
        axis="z",
        positive_elem="head_drum",
        negative_elem="center_socket",
        max_gap=0.001,
        max_penetration=0.0005,
        name="rotary head sits on pedestal bearing",
    )
    ctx.expect_overlap(
        instrument_pad,
        pedestal,
        axes="xy",
        elem_a="head_drum",
        elem_b="pedestal_body",
        min_overlap=0.08,
        name="rotary head is centered over pedestal axis",
    )

    rest_aabb = ctx.part_element_world_aabb(instrument_pad, elem="pad_body")
    with ctx.pose({yaw: pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(instrument_pad, elem="pad_body")

    def _span(aabb, axis_index: int) -> float:
        if aabb is None:
            return 0.0
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.check(
        "yaw joint rotates the narrow pad about the vertical axis",
        rest_aabb is not None
        and turned_aabb is not None
        and _span(rest_aabb, 0) > _span(rest_aabb, 1) * 2.5
        and _span(turned_aabb, 1) > _span(turned_aabb, 0) * 2.5,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
