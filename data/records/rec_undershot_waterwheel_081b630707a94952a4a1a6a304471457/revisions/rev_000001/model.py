from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


AXLE_Z = 0.78


def _ring_along_y(outer_radius: float, inner_radius: float, width: float):
    """A centered annular ring whose rotation axis is the model Y axis."""
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(width, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    weathered_wood = model.material("weathered_wood", rgba=(0.47, 0.30, 0.16, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.28, 0.17, 0.09, 1.0))
    iron = model.material("blackened_iron", rgba=(0.12, 0.13, 0.13, 1.0))
    steel = model.material("worn_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    stone = model.material("stone_base", rgba=(0.46, 0.45, 0.41, 1.0))
    water = model.material("shallow_water", rgba=(0.20, 0.50, 0.78, 0.55))

    rim_mesh = mesh_from_cadquery(_ring_along_y(0.295, 0.245, 0.045), "waterwheel_wood_rim")
    bearing_mesh = mesh_from_cadquery(_ring_along_y(0.108, 0.034, 0.060), "waterwheel_bearing_collar")

    frame = model.part("frame")
    frame.visual(
        Box((1.20, 1.36, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=stone,
        name="base_slab",
    )

    # Two tall side frames hold the axle well above the ground plane.
    for side_index, y in enumerate((-0.55, 0.55)):
        frame.visual(
            Box((0.09, 0.07, 0.82)),
            origin=Origin(xyz=(-0.33, y, 0.49)),
            material=weathered_wood,
            name=f"post_{side_index}_0",
        )
        frame.visual(
            Box((0.09, 0.07, 0.82)),
            origin=Origin(xyz=(0.33, y, 0.49)),
            material=weathered_wood,
            name=f"post_{side_index}_1",
        )
        frame.visual(
            Box((0.78, 0.075, 0.085)),
            origin=Origin(xyz=(0.0, y, 0.915)),
            material=weathered_wood,
            name=f"top_beam_{side_index}",
        )
        frame.visual(
            Box((0.72, 0.060, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.18)),
            material=dark_wood,
            name=f"lower_tie_{side_index}",
        )

        # Square bearing yoke around the open axle hole.  The visible iron collar
        # is hollow, so the rotating axle can pass through without collision.
        frame.visual(
            Box((0.050, 0.090, 0.220)),
            origin=Origin(xyz=(-0.095, y, AXLE_Z)),
            material=dark_wood,
            name=f"bearing_cheek_{side_index}_0",
        )
        frame.visual(
            Box((0.050, 0.090, 0.220)),
            origin=Origin(xyz=(0.095, y, AXLE_Z)),
            material=dark_wood,
            name=f"bearing_cheek_{side_index}_1",
        )
        frame.visual(
            Box((0.240, 0.090, 0.048)),
            origin=Origin(xyz=(0.0, y, AXLE_Z + 0.105)),
            material=dark_wood,
            name=f"bearing_cap_{side_index}",
        )
        frame.visual(
            Box((0.240, 0.090, 0.048)),
            origin=Origin(xyz=(0.0, y, AXLE_Z - 0.105)),
            material=dark_wood,
            name=f"bearing_sill_{side_index}",
        )
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, y, AXLE_Z)),
            material=iron,
            name=("bearing_collar_0", "bearing_collar_1")[side_index],
        )

    # A simple upstream trough/sluice edge aimed below the wheel, mounted into
    # the side frames.  It reads as an undershot flume rather than an overshot
    # bucket race.
    frame.visual(
        Box((0.56, 0.98, 0.030)),
        origin=Origin(xyz=(-0.62, 0.0, 0.455)),
        material=weathered_wood,
        name="trough_floor",
    )
    frame.visual(
        Box((0.56, 0.070, 0.130)),
        origin=Origin(xyz=(-0.62, -0.525, 0.515)),
        material=weathered_wood,
        name="trough_side_0",
    )
    frame.visual(
        Box((0.56, 0.070, 0.130)),
        origin=Origin(xyz=(-0.62, 0.525, 0.515)),
        material=weathered_wood,
        name="trough_side_1",
    )
    frame.visual(
        Box((0.040, 0.98, 0.085)),
        origin=Origin(xyz=(-0.355, 0.0, 0.505)),
        material=dark_wood,
        name="trough_lip",
    )
    frame.visual(
        Box((0.46, 0.86, 0.010)),
        origin=Origin(xyz=(-0.66, 0.0, 0.475)),
        material=water,
        name="water_sheet",
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.035, length=1.22),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.075, length=0.52),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub",
    )
    for side_index, y in enumerate((-0.18, 0.18)):
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=weathered_wood,
            name=f"rim_{side_index}",
        )
        for spoke_index in range(8):
            theta = 2.0 * math.pi * spoke_index / 8.0
            mid_radius = 0.165
            wheel.visual(
                Box((0.220, 0.052, 0.026)),
                origin=Origin(
                    xyz=(math.cos(theta) * mid_radius, y, math.sin(theta) * mid_radius),
                    rpy=(0.0, -theta, 0.0),
                ),
                material=weathered_wood,
                name=f"spoke_{side_index}_{spoke_index}",
            )

    for paddle_index in range(12):
        theta = 2.0 * math.pi * paddle_index / 12.0
        radius = 0.305
        # Long dimension is tangential, middle dimension spans the wheel width,
        # and the thin dimension is radial.
        wheel.visual(
            Box((0.115, 0.48, 0.040)),
            origin=Origin(
                xyz=(math.cos(theta) * radius, 0.0, math.sin(theta) * radius),
                rpy=(0.0, math.pi / 2.0 - theta, 0.0),
            ),
            material=dark_wood,
            name=f"paddle_{paddle_index}",
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.check(
        "wheel uses a continuous axle joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    axle_pos = ctx.part_world_position(wheel)
    ctx.check(
        "rotating axle is high above the base",
        axle_pos is not None and axle_pos[2] > 0.72,
        details=f"wheel origin={axle_pos}",
    )
    for collar_name in ("bearing_collar_0", "bearing_collar_1"):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=collar_name,
            elem_b="axle",
            reason="The axle is intentionally captured in a very slightly snug bearing collar so the rotating stage is visibly supported.",
        )
        ctx.expect_within(
            wheel,
            frame,
            axes="xz",
            inner_elem="axle",
            outer_elem=collar_name,
            margin=0.0,
            name=f"axle is centered in {collar_name}",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="y",
            elem_a="axle",
            elem_b=collar_name,
            min_overlap=0.045,
            name=f"axle passes through {collar_name}",
        )
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        min_gap=0.62,
        positive_elem="axle",
        negative_elem="base_slab",
        name="axle clears the base on tall supports",
    )

    rest_aabb = ctx.part_element_world_aabb(wheel, elem="paddle_0")
    rest_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(wheel, elem="paddle_0")
        turned_z = None if turned_aabb is None else (turned_aabb[0][2] + turned_aabb[1][2]) * 0.5
    ctx.check(
        "continuous rotation carries a paddle around the axle",
        rest_z is not None and turned_z is not None and turned_z < rest_z - 0.25,
        details=f"rest_z={rest_z}, turned_z={turned_z}",
    )

    return ctx.report()


object_model = build_object_model()
