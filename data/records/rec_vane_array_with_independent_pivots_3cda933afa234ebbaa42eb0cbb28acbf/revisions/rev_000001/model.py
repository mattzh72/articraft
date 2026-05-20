from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="independent_vane_array")

    model.material("powder_coated_frame", rgba=(0.12, 0.14, 0.16, 1.0))
    model.material("dark_bearing", rgba=(0.045, 0.048, 0.052, 1.0))
    model.material("satin_aluminum", rgba=(0.74, 0.77, 0.78, 1.0))
    model.material("brushed_pivot", rgba=(0.54, 0.50, 0.42, 1.0))

    frame_width = 1.12
    frame_depth = 0.14
    side_width = 0.08
    frame_height = 0.62
    rail_height = 0.055
    bearing_radius = 0.024
    bearing_length = 0.032
    bearing_x = 0.466
    vane_zs = (-0.20, -0.12, -0.04, 0.04, 0.12, 0.20)

    frame = model.part("frame")
    frame.visual(
        Box((side_width, frame_depth, frame_height)),
        origin=Origin(xyz=(-frame_width / 2 + side_width / 2, 0.0, 0.0)),
        material="powder_coated_frame",
        name="side_stile_0",
    )
    frame.visual(
        Box((side_width, frame_depth, frame_height)),
        origin=Origin(xyz=(frame_width / 2 - side_width / 2, 0.0, 0.0)),
        material="powder_coated_frame",
        name="side_stile_1",
    )
    frame.visual(
        Box((frame_width, frame_depth, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, frame_height / 2 - rail_height / 2)),
        material="powder_coated_frame",
        name="top_rail",
    )
    frame.visual(
        Box((frame_width, frame_depth, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, -frame_height / 2 + rail_height / 2)),
        material="powder_coated_frame",
        name="bottom_rail",
    )

    # Small round bearing pads on the inside faces make the end support for each
    # independently pivoting vane legible without mechanically coupling them.
    for index, z in enumerate(vane_zs):
        frame.visual(
            Cylinder(radius=bearing_radius, length=bearing_length),
            origin=Origin(xyz=(-bearing_x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dark_bearing",
            name=f"bearing_0_{index}",
        )
        frame.visual(
            Cylinder(radius=bearing_radius, length=bearing_length),
            origin=Origin(xyz=(bearing_x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dark_bearing",
            name=f"bearing_1_{index}",
        )

    blade_length = 0.80
    shaft_length = 0.90
    blade_chord = 0.075
    blade_thickness = 0.018
    blade_profile = superellipse_profile(
        blade_thickness,
        blade_chord,
        exponent=2.7,
        segments=40,
    )
    blade_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(blade_profile, blade_length),
        "rounded_vane_blade",
    )

    for index, z in enumerate(vane_zs):
        vane = model.part(f"vane_{index}")
        vane.visual(
            blade_mesh,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material="satin_aluminum",
            name="blade",
        )
        vane.visual(
            Cylinder(radius=0.008, length=shaft_length),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_pivot",
            name="shaft",
        )
        model.articulation(
            f"frame_to_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=3.0,
                lower=-1.05,
                upper=1.05,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    vanes = [object_model.get_part(f"vane_{index}") for index in range(6)]
    joints = [object_model.get_articulation(f"frame_to_vane_{index}") for index in range(6)]

    ctx.check(
        "six independent vane joints",
        len(joints) == 6
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(j.mimic is None for j in joints),
        details="Each vane should have its own uncoupled revolute pivot.",
    )

    for index, vane in enumerate(vanes):
        ctx.expect_gap(
            vane,
            frame,
            axis="x",
            min_gap=-0.001,
            max_gap=0.002,
            positive_elem="shaft",
            negative_elem=f"bearing_0_{index}",
            name=f"vane {index} shaft seated at bearing 0",
        )
        ctx.expect_gap(
            frame,
            vane,
            axis="x",
            min_gap=-0.001,
            max_gap=0.002,
            positive_elem=f"bearing_1_{index}",
            negative_elem="shaft",
            name=f"vane {index} shaft seated at bearing 1",
        )

    for lower, upper in zip(vanes, vanes[1:]):
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.045,
            name=f"{upper.name} clears {lower.name}",
        )

    def _aabb_key(aabb):
        return tuple(tuple(round(float(value), 6) for value in point) for point in aabb)

    driven = joints[2]
    driven_vane = vanes[2]
    neighbor = vanes[3]
    rest_driven = ctx.part_element_world_aabb(driven_vane, elem="blade")
    rest_neighbor = ctx.part_element_world_aabb(neighbor, elem="blade")
    with ctx.pose({driven: 0.8}):
        tilted_driven = ctx.part_element_world_aabb(driven_vane, elem="blade")
        tilted_neighbor = ctx.part_element_world_aabb(neighbor, elem="blade")

    if rest_driven and tilted_driven and rest_neighbor and tilted_neighbor:
        rest_driven_z = rest_driven[1][2] - rest_driven[0][2]
        tilted_driven_z = tilted_driven[1][2] - tilted_driven[0][2]
        ctx.check(
            "one selected vane tilts about its long axis",
            tilted_driven_z > rest_driven_z + 0.025,
            details=f"rest_z={rest_driven_z}, tilted_z={tilted_driven_z}",
        )
        ctx.check(
            "adjacent vane remains mechanically independent",
            _aabb_key(rest_neighbor) == _aabb_key(tilted_neighbor),
            details=f"rest={rest_neighbor}, posed={tilted_neighbor}",
        )
    else:
        ctx.fail("pose aabb available", "Could not measure vane blade AABBs.")

    return ctx.report()


object_model = build_object_model()
