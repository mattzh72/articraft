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
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _tube_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    name: str,
):
    """Open-ended hollow round tube in local coordinates."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(z_max - z_min)
        .translate((0.0, 0.0, z_min))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0008, angular_tolerance=0.08)


def _floodlight_shell_mesh(name: str):
    """Boxy floodlight body with a recessed front opening and a back plate."""
    outer = cq.Workplane("XY").box(0.220, 0.140, 0.160).translate((0.0, 0.070, 0.0))
    cutter = cq.Workplane("XY").box(0.172, 0.130, 0.108).translate((0.0, 0.086, 0.0))
    shell = outer.cut(cutter)
    shell = shell.edges("|Y").fillet(0.006)
    return mesh_from_cadquery(shell, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_floodlight_mast")

    dark_metal = model.material("powder_coated_dark_metal", rgba=(0.045, 0.048, 0.052, 1.0))
    black = model.material("matte_black_housing", rgba=(0.010, 0.011, 0.012, 1.0))
    satin_steel = model.material("satin_anodized_aluminum", rgba=(0.68, 0.70, 0.70, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    warm_lens = model.material("warm_diffusing_lens", rgba=(1.0, 0.82, 0.34, 0.82))
    amber_led = model.material("amber_led_board", rgba=(1.0, 0.58, 0.08, 1.0))

    base = model.part("ballast_base")
    base.visual(
        Box((0.660, 0.440, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_metal,
        name="weighted_case",
    )
    base.visual(
        Box((0.600, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.235, 0.028)),
        material=rubber,
        name="front_rubber_foot",
    )
    base.visual(
        Box((0.600, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.235, 0.028)),
        material=rubber,
        name="rear_rubber_foot",
    )
    base.visual(
        Box((0.180, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, -0.150, 0.129)),
        material=black,
        name="carry_handle_pad",
    )
    base.visual(
        _tube_mesh(
            outer_radius=0.045,
            inner_radius=0.035,
            z_min=0.100,
            z_max=0.670,
            name="base_receiver_tube",
        ),
        material=satin_steel,
        name="receiver_sleeve",
    )
    base.visual(
        _tube_mesh(
            outer_radius=0.041,
            inner_radius=0.030,
            z_min=0.610,
            z_max=0.668,
            name="base_guide_bushing",
        ),
        material=black,
        name="guide_bushing",
    )
    base.visual(
        _tube_mesh(
            outer_radius=0.060,
            inner_radius=0.045,
            z_min=0.120,
            z_max=0.150,
            name="socket_flange_ring",
        ),
        material=dark_metal,
        name="socket_flange",
    )

    lower = model.part("lower_pole")
    lower.visual(
        _tube_mesh(
            outer_radius=0.030,
            inner_radius=0.023,
            z_min=-0.500,
            z_max=1.050,
            name="lower_stage_tube",
        ),
        material=satin_steel,
        name="lower_tube",
    )
    lower.visual(
        _tube_mesh(
            outer_radius=0.034,
            inner_radius=0.020,
            z_min=1.010,
            z_max=1.055,
            name="lower_locking_collar",
        ),
        material=black,
        name="lower_locking_collar",
    )
    lower.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.064, 0.0, 1.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lower_clamp_knob",
    )

    upper = model.part("upper_pole")
    upper.visual(
        _tube_mesh(
            outer_radius=0.020,
            inner_radius=0.014,
            z_min=-0.400,
            z_max=0.800,
            name="upper_stage_tube",
        ),
        material=satin_steel,
        name="upper_tube",
    )
    upper.visual(
        Cylinder(radius=0.024, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        material=black,
        name="top_locking_collar",
    )
    upper.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.050, 0.0, 0.780), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="top_clamp_knob",
    )

    bracket = model.part("bracket")
    bracket.visual(
        Cylinder(radius=0.026, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=black,
        name="stem_collar",
    )
    bracket.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.340, 0.100, 0.260),
                span_width=0.255,
                trunnion_diameter=0.018,
                trunnion_center_z=0.170,
                base_thickness=0.050,
                corner_radius=0.004,
                center=False,
            ),
            "tilt_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=black,
        name="tilt_yoke",
    )

    head = model.part("floodlight_head")
    head.visual(
        _floodlight_shell_mesh("floodlight_shell"),
        material=black,
        name="head_shell",
    )
    head.visual(
        Box((0.178, 0.007, 0.112)),
        origin=Origin(xyz=(0.0, 0.139, 0.0)),
        material=warm_lens,
        name="glass_lens",
    )
    head.visual(
        Box((0.140, 0.006, 0.072)),
        origin=Origin(xyz=(0.0, 0.133, 0.0)),
        material=amber_led,
        name="led_panel",
    )
    for index, z in enumerate((-0.054, -0.027, 0.0, 0.027, 0.054)):
        head.visual(
            Box((0.190, 0.024, 0.009)),
            origin=Origin(xyz=(0.0, -0.010, z)),
            material=black,
            name=f"heat_sink_fin_{index}",
        )
    head.visual(
        Cylinder(radius=0.009, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="tilt_pin",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.350),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 1.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.22, lower=0.0, upper=0.300),
    )
    model.articulation(
        "upper_to_bracket",
        ArticulationType.FIXED,
        parent=upper,
        child=bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
    )
    model.articulation(
        "bracket_to_head",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.65, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("ballast_base")
    lower = object_model.get_part("lower_pole")
    upper = object_model.get_part("upper_pole")
    bracket = object_model.get_part("bracket")
    head = object_model.get_part("floodlight_head")
    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_upper = object_model.get_articulation("lower_to_upper")
    bracket_to_head = object_model.get_articulation("bracket_to_head")

    ctx.allow_overlap(
        base,
        lower,
        elem_a="guide_bushing",
        elem_b="lower_tube",
        reason="The lower mast stage is intentionally captured in a simplified plastic guide bushing at the top of the base sleeve.",
    )
    ctx.allow_overlap(
        lower,
        upper,
        elem_a="lower_locking_collar",
        elem_b="upper_tube",
        reason="The upper mast stage is intentionally captured by the simplified clamp collar at the lower tube entry.",
    )
    ctx.allow_overlap(
        bracket,
        head,
        elem_a="tilt_yoke",
        elem_b="tilt_pin",
        reason="The floodlight tilt pin is intentionally captured in the yoke bore as a local hinge fit.",
    )

    ctx.expect_within(
        lower,
        base,
        axes="xy",
        inner_elem="lower_tube",
        outer_elem="receiver_sleeve",
        margin=0.002,
        name="lower pole is centered inside the base receiver",
    )
    ctx.expect_overlap(
        lower,
        base,
        axes="z",
        elem_a="lower_tube",
        elem_b="receiver_sleeve",
        min_overlap=0.45,
        name="collapsed lower pole remains deeply inserted",
    )
    ctx.expect_within(
        lower,
        base,
        axes="xy",
        inner_elem="lower_tube",
        outer_elem="guide_bushing",
        margin=0.001,
        name="lower pole is retained by the base guide bushing",
    )
    ctx.expect_overlap(
        lower,
        base,
        axes="z",
        elem_a="lower_tube",
        elem_b="guide_bushing",
        min_overlap=0.055,
        name="lower pole passes through the guide bushing",
    )
    ctx.expect_within(
        upper,
        lower,
        axes="xy",
        inner_elem="upper_tube",
        outer_elem="lower_tube",
        margin=0.002,
        name="upper pole is centered inside the lower tube",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="z",
        elem_a="upper_tube",
        elem_b="lower_tube",
        min_overlap=0.35,
        name="collapsed upper pole remains inserted",
    )
    ctx.expect_within(
        upper,
        lower,
        axes="xy",
        inner_elem="upper_tube",
        outer_elem="lower_locking_collar",
        margin=0.001,
        name="upper pole is retained by the lower clamp collar",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="z",
        elem_a="upper_tube",
        elem_b="lower_locking_collar",
        min_overlap=0.040,
        name="upper pole passes through the lower clamp collar",
    )
    ctx.expect_gap(
        bracket,
        upper,
        axis="z",
        positive_elem="stem_collar",
        negative_elem="upper_tube",
        max_gap=0.001,
        max_penetration=0.0,
        name="bracket collar seats on the upper pole",
    )
    ctx.expect_within(
        head,
        bracket,
        axes="x",
        inner_elem="head_shell",
        outer_elem="tilt_yoke",
        margin=0.005,
        name="floodlight head fits between the yoke cheeks",
    )
    ctx.expect_within(
        head,
        bracket,
        axes="yz",
        inner_elem="tilt_pin",
        outer_elem="tilt_yoke",
        margin=0.001,
        name="tilt pin is centered in the yoke bore envelope",
    )
    ctx.expect_overlap(
        head,
        bracket,
        axes="x",
        elem_a="tilt_pin",
        elem_b="tilt_yoke",
        min_overlap=0.30,
        name="tilt pin spans both yoke cheeks",
    )

    with ctx.pose({base_to_lower: 0.350, lower_to_upper: 0.300}):
        ctx.expect_overlap(
            lower,
            base,
            axes="z",
            elem_a="lower_tube",
            elem_b="receiver_sleeve",
            min_overlap=0.14,
            name="extended lower pole keeps retained insertion",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="z",
            elem_a="upper_tube",
            elem_b="lower_tube",
            min_overlap=0.09,
            name="extended upper pole keeps retained insertion",
        )

    rest_lens = ctx.part_element_world_aabb(head, elem="glass_lens")
    with ctx.pose({bracket_to_head: 0.90}):
        raised_lens = ctx.part_element_world_aabb(head, elem="glass_lens")
    rest_center_z = None if rest_lens is None else (rest_lens[0][2] + rest_lens[1][2]) * 0.5
    raised_center_z = None if raised_lens is None else (raised_lens[0][2] + raised_lens[1][2]) * 0.5
    ctx.check(
        "positive hinge tilt raises the floodlight beam",
        rest_center_z is not None
        and raised_center_z is not None
        and raised_center_z > rest_center_z + 0.045,
        details=f"rest_lens_z={rest_center_z}, raised_lens_z={raised_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
