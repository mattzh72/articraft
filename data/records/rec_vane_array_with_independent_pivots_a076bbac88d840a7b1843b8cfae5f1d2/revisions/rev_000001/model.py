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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pivoted_service_vent")

    galvanized = Material("galvanized_frame", rgba=(0.56, 0.58, 0.56, 1.0))
    dark = Material("dark_recess", rgba=(0.07, 0.075, 0.075, 1.0))
    blade_metal = Material("brushed_blade", rgba=(0.72, 0.74, 0.70, 1.0))
    pivot_metal = Material("dark_pivot", rgba=(0.18, 0.19, 0.18, 1.0))

    outer_w = 0.68
    outer_h = 0.50
    frame_depth = 0.080
    side_w = 0.050
    rail_h = 0.050
    inner_x = outer_w / 2.0 - side_w
    inner_z = outer_h / 2.0 - rail_h

    frame = model.part("frame")
    frame.visual(
        Box((side_w, frame_depth, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + side_w / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="side_rail_0",
    )
    frame.visual(
        Box((side_w, frame_depth, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - side_w / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="side_rail_1",
    )
    frame.visual(
        Box((outer_w, frame_depth, rail_h)),
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2.0 - rail_h / 2.0)),
        material=galvanized,
        name="top_rail",
    )
    frame.visual(
        Box((outer_w, frame_depth, rail_h)),
        origin=Origin(xyz=(0.0, 0.0, -outer_h / 2.0 + rail_h / 2.0)),
        material=galvanized,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.016, frame_depth * 0.72, inner_z * 2.0)),
        origin=Origin(xyz=(-inner_x - 0.012, 0.0, 0.0)),
        material=dark,
        name="left_shadow_return",
    )
    frame.visual(
        Box((0.016, frame_depth * 0.72, inner_z * 2.0)),
        origin=Origin(xyz=(inner_x + 0.012, 0.0, 0.0)),
        material=dark,
        name="right_shadow_return",
    )

    screw_positions = (
        (-0.305, -0.215),
        (0.305, -0.215),
        (-0.305, 0.215),
        (0.305, 0.215),
    )
    for screw_i, (sx, sz) in enumerate(screw_positions):
        frame.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(
                xyz=(sx, -frame_depth / 2.0 - 0.001, sz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=pivot_metal,
            name=f"screw_head_{screw_i}",
        )

    blade_centers = [-0.165, -0.099, -0.033, 0.033, 0.099, 0.165]
    blade_width = 0.520
    blade_chord = 0.075
    blade_thickness = 0.014
    shaft_length = 0.556
    shaft_radius = 0.010
    rest_angle = 0.32
    motion = MotionLimits(effort=1.5, velocity=2.5, lower=-0.45, upper=0.45)

    for i, z in enumerate(blade_centers):
        frame.visual(
            Cylinder(radius=0.021, length=0.026),
            origin=Origin(
                xyz=(-inner_x + 0.001, 0.0, z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=galvanized,
            name=f"pivot_boss_{i}_0",
        )
        frame.visual(
            Cylinder(radius=0.021, length=0.026),
            origin=Origin(
                xyz=(inner_x - 0.001, 0.0, z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=galvanized,
            name=f"pivot_boss_{i}_1",
        )

        blade = model.part(f"blade_{i}")
        blade.visual(
            Box((blade_width, blade_chord, blade_thickness)),
            origin=Origin(rpy=(rest_angle, 0.0, 0.0)),
            material=blade_metal,
            name="blade_panel",
        )
        blade.visual(
            Cylinder(radius=shaft_radius, length=shaft_length),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pivot_metal,
            name="pivot_shaft",
        )

        model.articulation(
            f"frame_to_blade_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=blade,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=motion,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    blades = [object_model.get_part(f"blade_{i}") for i in range(6)]
    joints = [object_model.get_articulation(f"frame_to_blade_{i}") for i in range(6)]

    ctx.check("six independent blades", len(blades) == 6 and len(joints) == 6)

    for i, (blade, joint) in enumerate(zip(blades, joints)):
        ctx.check(
            f"blade_{i} has revolute joint",
            joint.articulation_type == ArticulationType.REVOLUTE,
        )
        ctx.expect_contact(
            blade,
            frame,
            elem_a="pivot_shaft",
            contact_tol=0.001,
            name=f"blade_{i} shaft reaches frame bosses",
        )
        ctx.expect_within(
            blade,
            frame,
            axes="xz",
            inner_elem="blade_panel",
            margin=0.001,
            name=f"blade_{i} panel stays inside frame outline",
        )

    for i in range(len(blades) - 1):
        ctx.expect_gap(
            blades[i + 1],
            blades[i],
            axis="z",
            min_gap=0.010,
            positive_elem="blade_panel",
            negative_elem="blade_panel",
            name=f"blade_{i} and blade_{i + 1} have louver spacing",
        )

    driven = joints[2]
    rest_aabb = ctx.part_element_world_aabb(blades[2], elem="blade_panel")
    with ctx.pose({driven: driven.motion_limits.upper}):
        moved_aabb = ctx.part_element_world_aabb(blades[2], elem="blade_panel")
    rest_z = None if rest_aabb is None else rest_aabb[1][2] - rest_aabb[0][2]
    moved_z = None if moved_aabb is None else moved_aabb[1][2] - moved_aabb[0][2]
    ctx.check(
        "one blade pivots about its own horizontal shaft",
        rest_z is not None and moved_z is not None and moved_z > rest_z + 0.008,
        details=f"rest_z={rest_z}, moved_z={moved_z}",
    )

    return ctx.report()


object_model = build_object_model()
