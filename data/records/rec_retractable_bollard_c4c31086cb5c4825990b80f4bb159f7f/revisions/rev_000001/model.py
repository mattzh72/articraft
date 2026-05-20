from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_shell(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    name: str,
    *,
    segments: int = 72,
):
    """Build a simple closed hollow cylindrical shell in local/world Z."""

    shell = LatheGeometry.from_shell_profiles(
        outer_profile=((outer_radius, z_min), (outer_radius, z_max)),
        inner_profile=((inner_radius, z_min), (inner_radius, z_max)),
        segments=segments,
    )
    return mesh_from_geometry(shell, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_security_post")

    galvanized = model.material("satin_galvanized_steel", rgba=(0.62, 0.64, 0.61, 1.0))
    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.035, 0.038, 0.04, 1.0))
    black = model.material("black_rubber_shadow", rgba=(0.005, 0.005, 0.004, 1.0))
    brass = model.material("weathered_brass_lock", rgba=(0.86, 0.63, 0.23, 1.0))
    reflective = model.material("amber_reflective_band", rgba=(1.0, 0.72, 0.08, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        _annular_shell(0.122, 0.145, 0.000, 0.620, "recessed_sleeve"),
        material=dark_steel,
        name="recessed_sleeve",
    )
    sleeve.visual(
        _annular_shell(0.122, 0.180, 0.600, 0.820, "guide_collar"),
        material=dark_steel,
        name="guide_collar",
    )
    sleeve.visual(
        _annular_shell(0.145, 0.260, 0.580, 0.640, "mounting_flange"),
        material=dark_steel,
        name="mounting_flange",
    )
    sleeve.visual(
        Box((0.025, 0.055, 0.210)),
        origin=Origin(xyz=(0.1175, 0.0, 0.710)),
        material=black,
        name="guide_pad_0",
    )
    sleeve.visual(
        Box((0.025, 0.055, 0.210)),
        origin=Origin(xyz=(-0.1175, 0.0, 0.710)),
        material=black,
        name="guide_pad_1",
    )
    sleeve.visual(
        Box((0.055, 0.025, 0.210)),
        origin=Origin(xyz=(0.0, 0.1175, 0.710)),
        material=black,
        name="guide_pad_2",
    )
    sleeve.visual(
        Box((0.055, 0.025, 0.210)),
        origin=Origin(xyz=(0.0, -0.1175, 0.710)),
        material=black,
        name="guide_pad_3",
    )
    for i, angle in enumerate((math.radians(45), math.radians(135), math.radians(225), math.radians(315))):
        x = 0.215 * math.cos(angle)
        y = 0.215 * math.sin(angle)
        sleeve.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(x, y, 0.6485)),
            material=galvanized,
            name=f"bolt_{i}",
        )
        sleeve.visual(
            Box((0.026, 0.006, 0.003)),
            origin=Origin(xyz=(x, y, 0.659), rpy=(0.0, 0.0, angle)),
            material=black,
            name=f"bolt_slot_{i}",
        )

    upper_post = model.part("upper_post")
    upper_post.visual(
        _annular_shell(0.087, 0.105, -0.550, 0.800, "post_tube"),
        material=galvanized,
        name="post_tube",
    )
    upper_post.visual(
        _annular_shell(0.104, 0.109, 0.430, 0.485, "lower_reflector"),
        material=reflective,
        name="lower_reflector",
    )
    upper_post.visual(
        _annular_shell(0.104, 0.109, 0.555, 0.610, "upper_reflector"),
        material=reflective,
        name="upper_reflector",
    )
    upper_post.visual(
        Cylinder(radius=0.056, length=0.018),
        origin=Origin(xyz=(0.108, 0.0, 0.630), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="service_boss",
    )

    service_cap = model.part("service_cap")
    service_cap.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="cap_disc",
    )
    service_cap.visual(
        Cylinder(radius=0.019, length=0.004),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lock_core",
    )
    service_cap.visual(
        Box((0.003, 0.030, 0.007)),
        origin=Origin(xyz=(0.0135, 0.0, 0.0)),
        material=black,
        name="key_slot",
    )

    top_cap = model.part("top_cap")
    top_cap.visual(
        Cylinder(radius=0.113, length=0.050),
        material=dark_steel,
        name="cap_body",
    )
    top_cap.visual(
        Cylinder(radius=0.070, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=dark_steel,
        name="plug_stem",
    )
    top_cap.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=brass,
        name="lock_face",
    )
    top_cap.visual(
        Box((0.056, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=black,
        name="keyway",
    )

    model.articulation(
        "sleeve_to_upper_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=upper_post,
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.350),
    )
    model.articulation(
        "post_to_service_cap",
        ArticulationType.CONTINUOUS,
        parent=upper_post,
        child=service_cap,
        origin=Origin(xyz=(0.126, 0.0, 0.630)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    model.articulation(
        "post_to_top_cap",
        ArticulationType.CONTINUOUS,
        parent=upper_post,
        child=top_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    upper_post = object_model.get_part("upper_post")
    service_cap = object_model.get_part("service_cap")
    top_cap = object_model.get_part("top_cap")
    slide = object_model.get_articulation("sleeve_to_upper_post")
    service_spin = object_model.get_articulation("post_to_service_cap")
    top_spin = object_model.get_articulation("post_to_top_cap")

    ctx.check(
        "post slides vertically in sleeve",
        slide.axis == (0.0, 0.0, 1.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == 0.350,
        details=f"axis={slide.axis}, limits={slide.motion_limits}",
    )
    ctx.check(
        "caps are continuous keyed controls",
        service_spin.articulation_type == ArticulationType.CONTINUOUS
        and top_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"service={service_spin.articulation_type}, top={top_spin.articulation_type}",
    )

    ctx.expect_overlap(
        upper_post,
        sleeve,
        axes="z",
        elem_a="post_tube",
        elem_b="guide_collar",
        min_overlap=0.200,
        name="post remains guided by collar when retracted",
    )
    ctx.expect_within(
        upper_post,
        sleeve,
        axes="xy",
        inner_elem="post_tube",
        outer_elem="guide_collar",
        margin=0.020,
        name="post tube stays centered inside collar footprint",
    )
    ctx.expect_contact(
        upper_post,
        sleeve,
        elem_a="post_tube",
        elem_b="guide_pad_0",
        contact_tol=0.001,
        name="guide pad bears on retracted post",
    )
    ctx.expect_contact(
        service_cap,
        upper_post,
        elem_a="cap_disc",
        elem_b="service_boss",
        contact_tol=0.0015,
        name="service cap seats on separate boss",
    )
    ctx.expect_contact(
        top_cap,
        upper_post,
        elem_a="cap_body",
        elem_b="post_tube",
        contact_tol=0.0015,
        name="keyed top cap sits on post tube rim",
    )

    rest_position = ctx.part_world_position(upper_post)
    with ctx.pose({slide: 0.350, service_spin: math.pi / 2.0, top_spin: math.pi / 2.0}):
        extended_position = ctx.part_world_position(upper_post)
        ctx.expect_overlap(
            upper_post,
            sleeve,
            axes="z",
            elem_a="post_tube",
            elem_b="guide_collar",
            min_overlap=0.180,
            name="post remains retained by collar at full extension",
        )
        ctx.expect_within(
            upper_post,
            sleeve,
            axes="xy",
            inner_elem="post_tube",
            outer_elem="guide_collar",
            margin=0.020,
            name="extended post stays centered in collar",
        )
        ctx.expect_contact(
            upper_post,
            sleeve,
            elem_a="post_tube",
            elem_b="guide_pad_0",
            contact_tol=0.001,
            name="guide pad still bears at full extension",
        )

    ctx.check(
        "upper bollard extends upward",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 0.300,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
