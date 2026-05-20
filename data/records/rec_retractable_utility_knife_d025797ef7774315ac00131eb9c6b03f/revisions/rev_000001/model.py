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


def _handle_shell_mesh():
    """Tapered utility-knife handle shell with an internal carrier tunnel and top slot."""
    outer = (
        cq.Workplane("XZ")
        .moveTo(-0.080, 0.000)
        .lineTo(0.082, 0.002)
        .lineTo(0.080, 0.017)
        .lineTo(-0.080, 0.024)
        .close()
        .extrude(0.034)
        .translate((0.0, 0.017, 0.0))
    )

    # A rectangular tunnel leaves real clearance for the blade carrier and
    # opens at the nose, while preserving a closed butt at the rear.
    carrier_tunnel = cq.Workplane("XY").box(0.150, 0.020, 0.012).translate((0.018, 0.0, 0.012))
    thumb_slot = cq.Workplane("XY").box(0.104, 0.012, 0.030).translate((0.006, 0.0, 0.026))
    return outer.cut(carrier_tunnel).cut(thumb_slot)


def _blade_mesh():
    """Thin trapezoid utility blade held by the sliding carrier."""
    return (
        cq.Workplane("XZ")
        .moveTo(0.036, 0.007)
        .lineTo(0.063, 0.007)
        .lineTo(0.073, 0.011)
        .lineTo(0.063, 0.015)
        .lineTo(0.036, 0.015)
        .close()
        .extrude(0.0012)
        .translate((0.0, 0.0006, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_utility_knife")

    yellow = model.material("yellow_handle", rgba=(0.95, 0.68, 0.10, 1.0))
    black = model.material("black_rubber", rgba=(0.025, 0.025, 0.023, 1.0))
    dark_metal = model.material("dark_carrier", rgba=(0.10, 0.11, 0.12, 1.0))
    blade_steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    screw_metal = model.material("dull_screw", rgba=(0.32, 0.33, 0.33, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shell_mesh(), "handle_shell", tolerance=0.0008),
        material=yellow,
        name="handle_shell",
    )
    # Slightly inset rubber side panels give the shell believable molded grip areas.
    for i, y in enumerate((-0.0175, 0.0175)):
        handle.visual(
            Box((0.088, 0.002, 0.010)),
            origin=Origin(xyz=(-0.014, y, 0.011)),
            material=black,
            name=f"side_grip_{i}",
        )
    for i, x in enumerate((-0.052, 0.052)):
        for j, y in enumerate((-0.0178, 0.0178)):
            handle.visual(
                Cylinder(radius=0.0032, length=0.0024),
                origin=Origin(xyz=(x, y, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=screw_metal,
                name=f"screw_{i}_{j}",
            )

    carrier = model.part("carrier")
    carrier.visual(
        Box((0.096, 0.012, 0.004)),
        origin=Origin(xyz=(-0.002, 0.0, 0.008)),
        material=dark_metal,
        name="carrier_rail",
    )
    carrier.visual(
        mesh_from_cadquery(_blade_mesh(), "blade", tolerance=0.0004),
        material=blade_steel,
        name="blade",
    )
    carrier.visual(
        Box((0.012, 0.006, 0.014)),
        origin=Origin(xyz=(-0.020, 0.0, 0.017)),
        material=dark_metal,
        name="thumb_stem",
    )
    carrier.visual(
        Box((0.024, 0.020, 0.004)),
        origin=Origin(xyz=(-0.020, 0.0, 0.026)),
        material=black,
        name="thumb_pad",
    )
    for i, x in enumerate((-0.027, -0.020, -0.013)):
        carrier.visual(
            Box((0.002, 0.018, 0.0016)),
            origin=Origin(xyz=(x, 0.0, 0.0288)),
            material=Material("pad_rib_dark", rgba=(0.004, 0.004, 0.004, 1.0)),
            name=f"grip_rib_{i}",
        )

    model.articulation(
        "handle_to_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carrier,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.18, lower=0.0, upper=0.045),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    carrier = object_model.get_part("carrier")
    slide = object_model.get_articulation("handle_to_carrier")

    ctx.check(
        "carrier uses prismatic centerline slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (1.0, 0.0, 0.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == 0.045,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            carrier,
            handle,
            axes="y",
            inner_elem="carrier_rail",
            outer_elem="handle_shell",
            margin=0.002,
            name="carrier rail is centered in handle tunnel",
        )
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="carrier_rail",
            elem_b="handle_shell",
            min_overlap=0.080,
            name="retracted carrier remains captured",
        )
        blade_aabb = ctx.part_element_world_aabb(carrier, elem="blade")
        handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_shell")
        ctx.check(
            "blade retracts inside nose",
            blade_aabb is not None
            and handle_aabb is not None
            and blade_aabb[1][0] < handle_aabb[1][0] - 0.004,
            details=f"blade={blade_aabb}, handle={handle_aabb}",
        )

    rest_pos = ctx.part_world_position(carrier)
    with ctx.pose({slide: 0.045}):
        ctx.expect_within(
            carrier,
            handle,
            axes="y",
            inner_elem="carrier_rail",
            outer_elem="handle_shell",
            margin=0.002,
            name="extended carrier stays centered",
        )
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="carrier_rail",
            elem_b="handle_shell",
            min_overlap=0.045,
            name="extended carrier remains inserted",
        )
        blade_aabb = ctx.part_element_world_aabb(carrier, elem="blade")
        handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_shell")
        pad_aabb = ctx.part_element_world_aabb(carrier, elem="thumb_pad")
        ctx.check(
            "blade projects from nose when extended",
            blade_aabb is not None
            and handle_aabb is not None
            and blade_aabb[1][0] > handle_aabb[1][0] + 0.025,
            details=f"blade={blade_aabb}, handle={handle_aabb}",
        )
        ctx.check(
            "thumb pad stays within top slot travel",
            pad_aabb is not None and pad_aabb[0][0] > -0.045 and pad_aabb[1][0] < 0.057,
            details=f"pad={pad_aabb}",
        )
        extended_pos = ctx.part_world_position(carrier)

    ctx.check(
        "positive slide moves carrier toward blade tip",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.040,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
