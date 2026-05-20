from __future__ import annotations

import cadquery as cq
import math

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


def _annular_cylinder(outer_radius: float, inner_radius: float, z_min: float, z_max: float):
    """CadQuery annular sleeve/flange with a real central opening."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(z_max - z_min)
        .translate((0.0, 0.0, z_min))
    )


def _pavement_slab():
    slab = cq.Workplane("XY").box(1.10, 1.10, 0.08).translate((0.0, 0.0, -0.04))
    cutter = cq.Workplane("XY").circle(0.17).extrude(0.12).translate((0.0, 0.0, -0.10))
    return slab.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_security_bollard")

    asphalt = Material("dark_asphalt", rgba=(0.045, 0.045, 0.040, 1.0))
    galvanized = Material("galvanized_steel", rgba=(0.55, 0.57, 0.56, 1.0))
    worn_steel = Material("worn_bolt_steel", rgba=(0.34, 0.35, 0.34, 1.0))
    safety_yellow = Material("safety_yellow_powdercoat", rgba=(1.0, 0.72, 0.05, 1.0))
    black = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    dark_recess = Material("dark_key_recess", rgba=(0.0, 0.0, 0.0, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_pavement_slab(), "pavement_slab", tolerance=0.002),
        material=asphalt,
        name="pavement",
    )
    sleeve.visual(
        mesh_from_cadquery(_annular_cylinder(0.140, 0.082, -0.55, 0.025), "sleeve_wall"),
        material=galvanized,
        name="sleeve_wall",
    )
    sleeve.visual(
        mesh_from_cadquery(_annular_cylinder(0.210, 0.082, -0.002, 0.040), "flange_collar"),
        material=galvanized,
        name="flange_collar",
    )
    for index in range(6):
        angle = index * 1.0471975512
        sleeve.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(
                xyz=(0.165 * math.cos(angle), 0.165 * math.sin(angle), 0.047)
            ),
            material=worn_steel,
            name=f"bolt_{index}",
        )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.075, length=1.20),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=safety_yellow,
        name="post_tube",
    )
    for index, z in enumerate((0.23, 0.43)):
        post.visual(
            Cylinder(radius=0.078, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=black,
            name=f"reflective_band_{index}",
        )
    post.visual(
        Cylinder(radius=0.084, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.500)),
        material=black,
        name="guide_ring",
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.083, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=worn_steel,
        name="cap_disc",
    )
    cap.visual(
        Box((0.088, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_recess,
        name="key_slot",
    )
    cap.visual(
        Cylinder(radius=0.013, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=dark_recess,
        name="key_core",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.25, lower=0.0, upper=0.45),
    )
    model.articulation(
        "post_to_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    cap = object_model.get_part("cap")
    lift = object_model.get_articulation("sleeve_to_post")
    spin = object_model.get_articulation("post_to_cap")

    ctx.allow_overlap(
        sleeve,
        post,
        elem_a="sleeve_wall",
        elem_b="guide_ring",
        reason=(
            "A hidden low-friction guide ring is shown as a slight interference fit "
            "inside the below-grade sleeve so the sliding post is retained and not floating."
        ),
    )

    ctx.expect_within(
        post,
        sleeve,
        axes="xy",
        inner_elem="post_tube",
        outer_elem="sleeve_wall",
        margin=0.0,
        name="post is centered inside sleeve guide",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_tube",
        elem_b="sleeve_wall",
        min_overlap=0.50,
        name="lower post remains deeply inserted at rest",
    )
    ctx.expect_within(
        post,
        sleeve,
        axes="xy",
        inner_elem="guide_ring",
        outer_elem="sleeve_wall",
        margin=0.0,
        name="guide ring stays inside sleeve bore",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="guide_ring",
        elem_b="sleeve_wall",
        min_overlap=0.04,
        name="guide ring is captured by sleeve wall",
    )
    ctx.expect_contact(
        cap,
        post,
        elem_a="cap_disc",
        elem_b="post_tube",
        contact_tol=1e-5,
        name="separate cap is seated on post tube",
    )

    rest_position = ctx.part_world_position(post)
    with ctx.pose({lift: 0.45}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_tube",
            elem_b="sleeve_wall",
            min_overlap=0.08,
            name="extended post still has retained insertion",
        )
        extended_position = ctx.part_world_position(post)

    ctx.check(
        "prismatic joint raises post vertically",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 0.40,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    with ctx.pose({spin: 1.5708}):
        ctx.expect_contact(
            cap,
            post,
            elem_a="cap_disc",
            elem_b="post_tube",
            contact_tol=1e-5,
            name="cap stays seated while rotating",
        )

    return ctx.report()


object_model = build_object_model()
