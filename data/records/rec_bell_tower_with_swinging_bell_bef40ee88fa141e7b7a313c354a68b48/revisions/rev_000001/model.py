from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _arched_prism(width: float, bottom: float, spring: float, top: float, depth: float):
    """Extrude an arched X/Z profile through Y."""
    half = width / 2.0
    return (
        cq.Workplane("XZ")
        .moveTo(-half, bottom)
        .lineTo(-half, spring)
        .threePointArc((0.0, top), (half, spring))
        .lineTo(half, bottom)
        .close()
        .extrude(depth / 2.0, both=True)
    )


def _mission_wall_body():
    """One solid stucco wall with a rounded mission crown and one arched opening."""
    wall_width = 1.70
    wall_depth = 0.28
    spring = 1.58
    top = 2.36
    half = wall_width / 2.0

    body = (
        cq.Workplane("XZ")
        .moveTo(-half, 0.0)
        .lineTo(half, 0.0)
        .lineTo(half, spring)
        .threePointArc((0.0, top), (-half, spring))
        .close()
        .extrude(wall_depth / 2.0, both=True)
    )

    opening = _arched_prism(
        width=0.92,
        bottom=0.56,
        spring=1.43,
        top=1.89,
        depth=wall_depth * 3.0,
    )
    return body.cut(opening)


def _arch_trim(width: float, bottom: float, spring: float, top: float, trim_width: float, depth: float):
    """Raised arched surround for the opening."""
    outer = _arched_prism(width + 2.0 * trim_width, bottom - trim_width, spring, top + trim_width, depth)
    inner = _arched_prism(width, bottom, spring, top, depth * 3.0)
    return outer.cut(inner)


def _bell_shell():
    """Hollow bronze bell shell, open at the mouth, revolved about local Z."""
    outer = [
        (0.085, -0.245),
        (0.120, -0.280),
        (0.170, -0.395),
        (0.215, -0.565),
        (0.295, -0.730),
    ]
    inner = [
        (0.242, -0.688),
        (0.185, -0.555),
        (0.137, -0.390),
        (0.066, -0.278),
    ]
    shell = (
        cq.Workplane("XY")
        .moveTo(*outer[0])
        .spline(outer[1:])
        .lineTo(*inner[0])
        .spline(inner[1:])
        .close()
    )
    return shell.revolve(360.0, (0.0, -1.0, 0.0), (0.0, 1.0, 0.0)).rotate(
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        90.0,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mexican_mission_bell_wall_tower")

    stucco = model.material("sun_baked_stucco", rgba=(0.72, 0.59, 0.42, 1.0))
    light_stucco = model.material("limewashed_arch_trim", rgba=(0.86, 0.78, 0.62, 1.0))
    stone = model.material("weathered_stone", rgba=(0.46, 0.38, 0.31, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.55, 0.37, 0.13, 1.0))
    dark_metal = model.material("dark_iron", rgba=(0.05, 0.045, 0.04, 1.0))
    timber = model.material("dark_headstock_wood", rgba=(0.22, 0.13, 0.06, 1.0))

    wall = model.part("arch_wall")
    wall.visual(
        mesh_from_cadquery(_mission_wall_body(), "arched_masonry_wall", tolerance=0.0015),
        material=stucco,
        name="arched_masonry_wall",
    )
    wall.visual(
        mesh_from_cadquery(_arch_trim(0.92, 0.56, 1.43, 1.89, 0.075, 0.045), "front_arch_trim"),
        origin=Origin(xyz=(0.0, -0.155, 0.0)),
        material=light_stucco,
        name="front_arch_trim",
    )
    wall.visual(
        mesh_from_cadquery(_arch_trim(0.92, 0.56, 1.43, 1.89, 0.075, 0.045), "rear_arch_trim"),
        origin=Origin(xyz=(0.0, 0.155, 0.0)),
        material=light_stucco,
        name="rear_arch_trim",
    )
    wall.visual(
        Box((1.95, 0.40, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=stone,
        name="lower_step",
    )
    wall.visual(
        Box((1.78, 0.34, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=stone,
        name="base_plinth",
    )
    # Bearing blocks are embedded in the side jambs at the spring line of the arch.
    for i, x in enumerate((-0.505, 0.505)):
        wall.visual(
            Box((0.115, 0.24, 0.17)),
            origin=Origin(xyz=(x, 0.0, 1.55)),
            material=stone,
            name=f"hinge_seat_{i}",
        )

    # A few shallow stones break up the stucco face without turning the wall into loose pieces.
    stone_specs = [
        (-0.63, -0.125, 0.46, 0.22, 0.060),
        (0.61, -0.125, 0.47, 0.20, 0.060),
        (-0.67, -0.125, 0.87, 0.18, 0.055),
        (0.66, -0.125, 0.88, 0.21, 0.055),
        (-0.62, -0.125, 1.25, 0.19, 0.055),
        (0.62, -0.125, 1.24, 0.18, 0.055),
    ]
    for i, (x, y, z, sx, sz) in enumerate(stone_specs):
        wall.visual(
            Box((sx, 0.085, sz)),
            origin=Origin(xyz=(x, y, z)),
            material=stone,
            name=f"face_stone_{i}",
        )

    bell = model.part("bell")
    bell.visual(
        mesh_from_cadquery(_bell_shell(), "hollow_bell_shell", tolerance=0.001),
        material=bronze,
        name="hollow_bell_shell",
    )
    bell.visual(
        Box((0.62, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=timber,
        name="headstock",
    )
    bell.visual(
        Cylinder(radius=0.022, length=0.96),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="pivot_axle",
    )
    for i, x in enumerate((-0.085, 0.085)):
        bell.visual(
            Box((0.030, 0.055, 0.260)),
            origin=Origin(xyz=(x, 0.0, -0.180)),
            material=dark_metal,
            name=f"hanger_strap_{i}",
        )
    bell.visual(
        Cylinder(radius=0.010, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, -0.355)),
        material=dark_metal,
        name="clapper_rod",
    )
    bell.visual(
        Sphere(radius=0.048),
        origin=Origin(xyz=(0.0, 0.0, -0.630)),
        material=dark_metal,
        name="clapper_ball",
    )

    model.articulation(
        "wall_to_bell",
        ArticulationType.REVOLUTE,
        parent=wall,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("arch_wall")
    bell = object_model.get_part("bell")
    swing = object_model.get_articulation("wall_to_bell")

    for seat_name in ("hinge_seat_0", "hinge_seat_1"):
        ctx.allow_overlap(
            wall,
            bell,
            elem_a=seat_name,
            elem_b="pivot_axle",
            reason="The iron pivot axle is intentionally captured in the masonry bearing seat.",
        )
        ctx.expect_overlap(
            wall,
            bell,
            axes="xyz",
            min_overlap=0.020,
            elem_a=seat_name,
            elem_b="pivot_axle",
            name=f"{seat_name} captures the bell pivot axle",
        )
    ctx.allow_overlap(
        wall,
        bell,
        elem_a="arched_masonry_wall",
        elem_b="pivot_axle",
        reason="The pivot axle ends pass into small bearing pockets in the arched masonry wall.",
    )
    ctx.expect_overlap(
        wall,
        bell,
        axes="x",
        min_overlap=0.040,
        elem_a="arched_masonry_wall",
        elem_b="pivot_axle",
        name="pivot axle is retained by the masonry wall",
    )

    ctx.expect_overlap(
        bell,
        wall,
        axes="x",
        min_overlap=0.45,
        elem_a="hollow_bell_shell",
        elem_b="arched_masonry_wall",
        name="bell hangs centered in the arch opening width",
    )
    axle_aabb = ctx.part_element_world_aabb(bell, elem="pivot_axle")
    shell_aabb = ctx.part_element_world_aabb(bell, elem="hollow_bell_shell")
    ctx.check(
        "pivot axle is above the bronze bell body",
        axle_aabb is not None and shell_aabb is not None and axle_aabb[0][2] > shell_aabb[1][2] + 0.18,
        details=f"axle_aabb={axle_aabb}, shell_aabb={shell_aabb}",
    )

    rest_aabb = ctx.part_world_aabb(bell)
    with ctx.pose({swing: 0.45}):
        swung_aabb = ctx.part_world_aabb(bell)
    ctx.check(
        "bell swings forward on the crown hinge",
        rest_aabb is not None
        and swung_aabb is not None
        and swung_aabb[1][1] > rest_aabb[1][1] + 0.10,
        details=f"rest_aabb={rest_aabb}, swung_aabb={swung_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
