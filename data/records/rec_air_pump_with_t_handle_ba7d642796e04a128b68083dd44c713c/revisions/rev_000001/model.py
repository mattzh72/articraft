from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _lathe_shell(name: str, outer_profile, inner_profile):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        ),
        name,
    )


def _ring_shell(name: str, outer_radius: float, inner_radius: float, z0: float, z1: float):
    return _lathe_shell(
        name,
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_station_pump")

    red_enamel = model.material("red_enamel", rgba=(0.72, 0.05, 0.03, 1.0))
    dark_cast = model.material("dark_cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.022, 1.0))
    gauge_cream = model.material("gauge_cream", rgba=(0.92, 0.88, 0.72, 1.0))
    glass = model.material("gauge_glass", rgba=(0.72, 0.88, 0.94, 0.55))
    white_mark = model.material("white_mark", rgba=(0.95, 0.95, 0.90, 1.0))

    body = model.part("pump_body")
    body.visual(
        Box((0.48, 0.38, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_cast,
        name="heavy_base",
    )
    body.visual(
        Cylinder(radius=0.125, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=dark_cast,
        name="base_flange",
    )
    body.visual(
        _lathe_shell(
            "barrel_shell",
            [
                (0.078, 0.100),
                (0.086, 0.150),
                (0.086, 0.960),
                (0.074, 1.055),
            ],
            [
                (0.044, 0.135),
                (0.052, 0.240),
                (0.052, 0.980),
                (0.040, 1.055),
            ],
        ),
        material=red_enamel,
        name="barrel_shell",
    )
    body.visual(
        _ring_shell("top_guide_ring", 0.074, 0.025, 1.020, 1.090),
        material=satin_steel,
        name="top_guide_ring",
    )
    body.visual(
        _ring_shell("lower_band", 0.092, 0.058, 0.195, 0.230),
        material=satin_steel,
        name="lower_band",
    )

    # The front gauge assembly is built as connected, shallow castings on the
    # barrel: a neck, a round gauge housing, glass face, and a lower selector panel.
    body.visual(
        Cylinder(radius=0.040, length=0.090),
        origin=Origin(xyz=(0.0, -0.120, 0.800), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_cast,
        name="gauge_neck",
    )
    body.visual(
        Cylinder(radius=0.112, length=0.065),
        origin=Origin(xyz=(0.0, -0.166, 0.800), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red_enamel,
        name="gauge_housing",
    )
    body.visual(
        Cylinder(radius=0.091, length=0.008),
        origin=Origin(xyz=(0.0, -0.202, 0.800), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gauge_cream,
        name="gauge_face",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.091, tube=0.007, radial_segments=18, tubular_segments=72), "gauge_bezel"),
        origin=Origin(xyz=(0.0, -0.207, 0.800), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_cast,
        name="gauge_bezel",
    )
    body.visual(
        Cylinder(radius=0.084, length=0.003),
        origin=Origin(xyz=(0.0, -0.207, 0.800), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="gauge_glass",
    )
    body.visual(
        Box((0.010, 0.004, 0.067)),
        origin=Origin(xyz=(0.020, -0.208, 0.820), rpy=(0.0, -0.62, 0.0)),
        material=dark_cast,
        name="gauge_needle",
    )
    body.visual(
        Box((0.180, 0.052, 0.112)),
        origin=Origin(xyz=(0.030, -0.110, 0.655)),
        material=red_enamel,
        name="selector_panel",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.060, -0.140, 0.655), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_cast,
        name="selector_boss",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.025),
        origin=Origin(xyz=(0.060, -0.1645, 0.655), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="selector_shaft",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.017, length=0.650),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=satin_steel,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0252, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=satin_steel,
        name="guide_bushing",
    )
    plunger.visual(
        Cylinder(radius=0.026, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.318), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="t_handle_bar",
    )
    plunger.visual(
        Cylinder(radius=0.034, length=0.095),
        origin=Origin(xyz=(-0.145, 0.0, 0.318), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="handle_grip_0",
    )
    plunger.visual(
        Cylinder(radius=0.034, length=0.095),
        origin=Origin(xyz=(0.145, 0.0, 0.318), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="handle_grip_1",
    )

    selector = model.part("selector_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.058,
            0.045,
            body_style="faceted",
            top_diameter=0.046,
            base_diameter=0.060,
            edge_radius=0.001,
            grip=KnobGrip(style="fluted", count=18, depth=0.0013),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "selector_knob",
    )
    selector.visual(
        knob_mesh,
        material=black_rubber,
        name="knob_body",
    )
    selector.visual(
        Box((0.007, 0.030, 0.003)),
        origin=Origin(xyz=(0.0, -0.006, 0.0455)),
        material=white_mark,
        name="knob_pointer",
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 1.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.45, lower=0.0, upper=0.28),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector,
        # Rotate the joint frame so the knob's local +Z shaft axis points out of
        # the front of the pump along world -Y.
        origin=Origin(xyz=(0.060, -0.177, 0.655), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("pump_body")
    plunger = object_model.get_part("plunger")
    selector = object_model.get_part("selector_knob")
    slide = object_model.get_articulation("body_to_plunger")
    selector_joint = object_model.get_articulation("body_to_selector_knob")

    ctx.allow_overlap(
        body,
        plunger,
        elem_a="top_guide_ring",
        elem_b="guide_bushing",
        reason="The sliding bushing is intentionally represented as a tiny captured fit inside the top guide ring.",
    )

    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="plunger_rod",
        outer_elem="barrel_shell",
        margin=0.002,
        name="plunger rod is centered in the barrel",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="plunger_rod",
        elem_b="barrel_shell",
        min_overlap=0.30,
        name="collapsed plunger remains inserted in the barrel",
    )
    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="guide_bushing",
        outer_elem="top_guide_ring",
        margin=0.001,
        name="guide bushing stays captured by the top guide ring",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="guide_bushing",
        elem_b="top_guide_ring",
        min_overlap=0.050,
        name="guide bushing is retained in the top guide ring",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            plunger,
            body,
            axes="xy",
            inner_elem="plunger_rod",
            outer_elem="barrel_shell",
            margin=0.002,
            name="extended plunger stays coaxial with the barrel",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="plunger_rod",
            elem_b="barrel_shell",
            min_overlap=0.06,
            name="extended plunger keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(plunger)
    ctx.check(
        "plunger slides upward along the barrel axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.25
        and abs(extended_pos[0] - rest_pos[0]) < 1e-6
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_gap(
        body,
        selector,
        axis="y",
        positive_elem="selector_shaft",
        negative_elem="knob_body",
        max_gap=0.004,
        max_penetration=0.001,
        name="selector knob seats on its shaft face",
    )
    ctx.expect_overlap(
        selector,
        body,
        axes="xz",
        elem_a="knob_body",
        elem_b="selector_shaft",
        min_overlap=0.020,
        name="selector knob is coaxial with its shaft",
    )
    knob_rest = ctx.part_world_position(selector)
    with ctx.pose({selector_joint: math.pi / 2.0}):
        knob_rotated = ctx.part_world_position(selector)
    ctx.check(
        "selector knob rotates about a fixed shaft",
        knob_rest is not None
        and knob_rotated is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(knob_rest, knob_rotated)),
        details=f"rest={knob_rest}, rotated={knob_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
