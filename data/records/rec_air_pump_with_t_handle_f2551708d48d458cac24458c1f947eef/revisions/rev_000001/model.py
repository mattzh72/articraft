from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    """Centered rounded rectangular solid used for stamped plastic plates."""
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .edges("|Z")
        .fillet(min(radius, length * 0.45, width * 0.45, height * 0.45))
    )


def _annular_tube(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Open cylindrical shell/collar with a real central bore."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_camping_pump")

    dark_plastic = model.material("matte_black_plastic", rgba=(0.02, 0.025, 0.025, 1.0))
    barrel_blue = model.material("anodized_blue_barrel", rgba=(0.05, 0.28, 0.62, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    metal = model.material("brushed_steel", rgba=(0.72, 0.74, 0.75, 1.0))

    base_barrel = model.part("base_barrel")
    base_barrel.visual(
        mesh_from_cadquery(_rounded_box(0.180, 0.070, 0.018, 0.014), "base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_plastic,
        name="base_plate",
    )
    base_barrel.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=dark_plastic,
        name="barrel_socket",
    )
    base_barrel.visual(
        mesh_from_cadquery(_annular_tube(0.033, 0.026, 0.240), "barrel_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=barrel_blue,
        name="barrel_shell",
    )
    base_barrel.visual(
        mesh_from_cadquery(_annular_tube(0.038, 0.014, 0.016), "top_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.258)),
        material=dark_plastic,
        name="top_collar",
    )
    base_barrel.visual(
        mesh_from_cadquery(_annular_tube(0.014, 0.006, 0.012), "guide_bushing"),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=metal,
        name="guide_bushing",
    )

    # Four fixed hinge cheeks make two side clevises for the folding feet.
    for lug_name, x, y in (
        ("side_0_hinge_lug_0", -0.048, 0.041),
        ("side_0_hinge_lug_1", 0.048, 0.041),
        ("side_1_hinge_lug_0", -0.048, -0.041),
        ("side_1_hinge_lug_1", 0.048, -0.041),
    ):
        base_barrel.visual(
            Box((0.014, 0.012, 0.026)),
            origin=Origin(xyz=(x, y, 0.031)),
            material=dark_plastic,
            name=lug_name,
        )
    for cap_name, x, y in (
        ("side_0_hinge_cap_0", -0.059, 0.041),
        ("side_0_hinge_cap_1", 0.059, 0.041),
        ("side_1_hinge_cap_0", -0.059, -0.041),
        ("side_1_hinge_cap_1", 0.059, -0.041),
    ):
        base_barrel.visual(
            Cylinder(radius=0.004, length=0.008),
            origin=Origin(xyz=(x, y, 0.031), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=cap_name,
        )

    handle_rod = model.part("handle_rod")
    handle_rod.visual(
        Cylinder(radius=0.006, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=metal,
        name="rod",
    )
    handle_rod.visual(
        Cylinder(radius=0.014, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="t_grip",
    )
    for x in (-0.085, 0.085):
        handle_rod.visual(
            Sphere(radius=0.0145),
            origin=Origin(xyz=(x, 0.0, 0.225)),
            material=rubber,
            name=f"grip_end_{0 if x < 0 else 1}",
        )
    handle_rod.visual(
        Box((0.030, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=rubber,
        name="grip_hub",
    )

    foot_plate_mesh = mesh_from_cadquery(_rounded_box(0.135, 0.085, 0.010, 0.012), "foot_plate")
    for foot_name in ("foot_0", "foot_1"):
        foot = model.part(foot_name)
        foot.visual(
            Cylinder(radius=0.006, length=0.082),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="hinge_barrel",
        )
        foot.visual(
            Box((0.030, 0.045, 0.018)),
            origin=Origin(xyz=(0.0, 0.016, -0.014)),
            material=dark_plastic,
            name="hinge_web",
        )
        foot.visual(
            foot_plate_mesh,
            origin=Origin(xyz=(0.0, 0.062, -0.025)),
            material=dark_plastic,
            name="foot_plate",
        )

    model.articulation(
        "barrel_to_handle",
        ArticulationType.PRISMATIC,
        parent=base_barrel,
        child=handle_rod,
        origin=Origin(xyz=(0.0, 0.0, 0.274)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.30, lower=0.0, upper=0.100),
    )
    model.articulation(
        "base_to_foot_0",
        ArticulationType.REVOLUTE,
        parent=base_barrel,
        child="foot_0",
        origin=Origin(xyz=(0.0, 0.047, 0.031)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "base_to_foot_1",
        ArticulationType.REVOLUTE,
        parent=base_barrel,
        child="foot_1",
        origin=Origin(xyz=(0.0, -0.047, 0.031), rpy=(0.0, 0.0, math.pi)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_barrel")
    handle = object_model.get_part("handle_rod")
    foot_0 = object_model.get_part("foot_0")
    foot_1 = object_model.get_part("foot_1")
    handle_slide = object_model.get_articulation("barrel_to_handle")
    foot_joint_0 = object_model.get_articulation("base_to_foot_0")
    foot_joint_1 = object_model.get_articulation("base_to_foot_1")

    ctx.check(
        "portable_pump_parts_present",
        all(part is not None for part in (base, handle, foot_0, foot_1))
        and all(joint is not None for joint in (handle_slide, foot_joint_0, foot_joint_1)),
        "Expected base_barrel, handle_rod, two folding feet, and three articulations.",
    )
    if not all(part is not None for part in (base, handle, foot_0, foot_1)):
        return ctx.report()

    ctx.allow_overlap(
        base,
        handle,
        elem_a="guide_bushing",
        elem_b="rod",
        reason="The pump rod is intentionally represented as a close sliding fit through the top guide bushing.",
    )
    ctx.expect_within(
        handle,
        base,
        axes="xy",
        inner_elem="rod",
        outer_elem="guide_bushing",
        margin=0.001,
        name="rod centered in guide bushing",
    )
    ctx.expect_overlap(
        handle,
        base,
        axes="z",
        elem_a="rod",
        elem_b="guide_bushing",
        min_overlap=0.010,
        name="rod passes through guide bushing",
    )
    ctx.expect_overlap(
        handle,
        base,
        axes="z",
        elem_a="rod",
        elem_b="barrel_shell",
        min_overlap=0.12,
        name="collapsed rod remains inside barrel",
    )

    rest_handle_pos = ctx.part_world_position(handle)
    if handle_slide is not None:
        with ctx.pose({handle_slide: 0.100}):
            ctx.expect_overlap(
                handle,
                base,
                axes="z",
                elem_a="rod",
                elem_b="barrel_shell",
                min_overlap=0.035,
                name="extended rod still retained in barrel",
            )
            extended_handle_pos = ctx.part_world_position(handle)
        ctx.check(
            "handle translates upward",
            rest_handle_pos is not None
            and extended_handle_pos is not None
            and extended_handle_pos[2] > rest_handle_pos[2] + 0.095,
            details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
        )

    ctx.expect_contact(
        foot_0,
        base,
        elem_a="hinge_barrel",
        elem_b="side_0_hinge_lug_0",
        contact_tol=0.001,
        name="foot_0 hinge seated in clevis",
    )
    ctx.expect_contact(
        foot_1,
        base,
        elem_a="hinge_barrel",
        elem_b="side_1_hinge_lug_0",
        contact_tol=0.001,
        name="foot_1 hinge seated in clevis",
    )

    foot_0_rest = ctx.part_world_aabb(foot_0)
    foot_1_rest = ctx.part_world_aabb(foot_1)
    if foot_0_rest is not None and foot_1_rest is not None:
        ctx.check(
            "feet deployed beside narrow base",
            foot_0_rest[1][1] > 0.130 and foot_1_rest[0][1] < -0.130,
            details=f"foot_0_aabb={foot_0_rest}, foot_1_aabb={foot_1_rest}",
        )

    if foot_joint_0 is not None and foot_joint_1 is not None and foot_0_rest is not None and foot_1_rest is not None:
        with ctx.pose({foot_joint_0: 1.20, foot_joint_1: 1.20}):
            foot_0_folded = ctx.part_world_aabb(foot_0)
            foot_1_folded = ctx.part_world_aabb(foot_1)
        ctx.check(
            "feet rotate upward on side hinges",
            foot_0_folded is not None
            and foot_1_folded is not None
            and foot_0_folded[0][2] > foot_0_rest[0][2] + 0.015
            and foot_1_folded[0][2] > foot_1_rest[0][2] + 0.015,
            details=f"rest={foot_0_rest, foot_1_rest}, folded={foot_0_folded, foot_1_folded}",
        )

    return ctx.report()


object_model = build_object_model()
