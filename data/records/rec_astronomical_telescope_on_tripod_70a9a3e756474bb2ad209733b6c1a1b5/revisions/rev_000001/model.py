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


def _annular_cylinder(axis: str, outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Return a CadQuery annular sleeve centered on the origin and aligned to an axis."""
    plane = {"x": "YZ", "y": "XZ", "z": "XY"}[axis]
    return (
        cq.Workplane(plane)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="goto_single_arm_newtonian_reflector")

    mount_black = model.material("matte_black", rgba=(0.015, 0.017, 0.020, 1.0))
    mount_blue = model.material("deep_blue_mount", rgba=(0.03, 0.06, 0.13, 1.0))
    tube_white = model.material("warm_white_tube", rgba=(0.88, 0.86, 0.78, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.05, 0.055, 0.060, 1.0))
    machined = model.material("machined_aluminum", rgba=(0.55, 0.57, 0.58, 1.0))
    glass = model.material("faint_mirror_glass", rgba=(0.50, 0.62, 0.68, 0.75))

    # A compact flat tripod head with three radial socket pads and a fixed lower
    # half of the azimuth bearing.
    tripod_head = model.part("tripod_head")
    tripod_head.visual(
        Cylinder(radius=0.30, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=mount_black,
        name="flat_plate",
    )
    tripod_head.visual(
        Cylinder(radius=0.18, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=machined,
        name="fixed_bearing_race",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        tripod_head.visual(
            Box((0.22, 0.075, 0.038)),
            origin=Origin(
                xyz=(0.32 * math.cos(angle), 0.32 * math.sin(angle), 0.026),
                rpy=(0.0, 0.0, angle),
            ),
            material=mount_black,
            name=f"tripod_socket_{i}",
        )

    # Rotating computerized single-arm fork mount.  Its child frame is on the
    # azimuth axis at the top of the tripod head, so z=0 is the bearing plane.
    fork_arm = model.part("fork_arm")
    fork_arm.visual(
        Cylinder(radius=0.215, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=mount_blue,
        name="azimuth_motor_housing",
    )
    fork_arm.visual(
        Cylinder(radius=0.150, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=machined,
        name="rotating_bearing_race",
    )

    arm_shell = (
        cq.Workplane("XY")
        .box(0.16, 0.14, 0.68)
        .edges("|Z")
        .fillet(0.025)
        .translate((0.0, -0.265, 0.405))
    )
    fork_arm.visual(
        mesh_from_cadquery(arm_shell, "fork_arm_shell", tolerance=0.0015),
        material=mount_blue,
        name="fork_arm_shell",
    )
    fork_arm.visual(
        Box((0.25, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -0.235, 0.105)),
        material=mount_blue,
        name="arm_foot_fillet",
    )
    fork_arm.visual(
        Box((0.105, 0.020, 0.22)),
        origin=Origin(xyz=(0.055, -0.342, 0.395), rpy=(0.0, 0.0, -0.10)),
        material=dark_graphite,
        name="electronics_panel",
    )

    altitude_bearing = _annular_cylinder("y", outer_radius=0.108, inner_radius=0.055, length=0.080)
    fork_arm.visual(
        mesh_from_cadquery(altitude_bearing, "altitude_bearing_ring", tolerance=0.001),
        origin=Origin(xyz=(0.0, -0.190, 0.740)),
        material=machined,
        name="altitude_bearing_ring",
    )
    fork_arm.visual(
        Cylinder(radius=0.048, length=0.030),
        origin=Origin(xyz=(0.0, -0.236, 0.740), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="altitude_motor_cap",
    )

    # Tube, cradle rings, focuser, and the visible altitude axle rotate together.
    # The part frame is at the altitude axis on the inside face of the fork arm.
    tube_cradle = model.part("tube_cradle")

    tube_shell = _annular_cylinder("x", outer_radius=0.115, inner_radius=0.104, length=1.10).translate(
        (0.0, 0.190, 0.0)
    )
    tube_cradle.visual(
        mesh_from_cadquery(tube_shell, "open_newtonian_tube", tolerance=0.0012),
        material=tube_white,
        name="open_tube",
    )
    tube_cradle.visual(
        mesh_from_cadquery(
            _annular_cylinder("x", outer_radius=0.124, inner_radius=0.102, length=0.026).translate(
                (0.555, 0.190, 0.0)
            ),
            "front_black_rim",
            tolerance=0.001,
        ),
        material=dark_graphite,
        name="front_rim",
    )
    tube_cradle.visual(
        Cylinder(radius=0.116, length=0.026),
        origin=Origin(xyz=(-0.552, 0.190, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="rear_cell",
    )
    tube_cradle.visual(
        Cylinder(radius=0.074, length=0.006),
        origin=Origin(xyz=(-0.568, 0.190, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="primary_mirror",
    )

    for x in (-0.285, 0.285):
        tube_cradle.visual(
            mesh_from_cadquery(
                _annular_cylinder("x", outer_radius=0.148, inner_radius=0.112, length=0.045).translate(
                    (x, 0.190, 0.0)
                ),
                f"cradle_ring_{'front' if x > 0 else 'rear'}",
                tolerance=0.001,
            ),
            material=mount_black,
            name=f"cradle_ring_{'front' if x > 0 else 'rear'}",
        )
    tube_cradle.visual(
        Box((0.76, 0.045, 0.030)),
        origin=Origin(xyz=(0.0, 0.190, -0.135)),
        material=mount_black,
        name="dovetail_rail",
    )
    tube_cradle.visual(
        Box((0.32, 0.120, 0.090)),
        origin=Origin(xyz=(0.0, 0.102, 0.0)),
        material=mount_black,
        name="side_saddle",
    )
    tube_cradle.visual(
        Cylinder(radius=0.040, length=0.205),
        origin=Origin(xyz=(0.0, 0.1025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="altitude_axle",
    )
    tube_cradle.visual(
        Cylinder(radius=0.061, length=0.026),
        origin=Origin(xyz=(0.0, 0.053, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="axle_collar",
    )

    # Newtonian side focuser near the open end: a raised base, drawtube, and
    # eyepiece identify the optical tube as a reflector rather than a refractor.
    tube_cradle.visual(
        Box((0.120, 0.070, 0.035)),
        origin=Origin(xyz=(0.330, 0.190, 0.116)),
        material=dark_graphite,
        name="focuser_base",
    )
    tube_cradle.visual(
        Cylinder(radius=0.034, length=0.110),
        origin=Origin(xyz=(0.330, 0.190, 0.180)),
        material=mount_black,
        name="focuser_drawtube",
    )
    tube_cradle.visual(
        Cylinder(radius=0.026, length=0.070),
        origin=Origin(xyz=(0.330, 0.190, 0.270)),
        material=machined,
        name="eyepiece",
    )

    model.articulation(
        "azimuth_bearing",
        ArticulationType.CONTINUOUS,
        parent=tripod_head,
        child=fork_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.65),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=fork_arm,
        child=tube_cradle,
        origin=Origin(xyz=(0.0, -0.190, 0.740)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.45, lower=0.0, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_head = object_model.get_part("tripod_head")
    fork_arm = object_model.get_part("fork_arm")
    tube_cradle = object_model.get_part("tube_cradle")
    azimuth = object_model.get_articulation("azimuth_bearing")
    altitude = object_model.get_articulation("altitude_axis")

    ctx.expect_gap(
        fork_arm,
        tripod_head,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="azimuth_motor_housing",
        negative_elem="fixed_bearing_race",
        name="azimuth turntable rests on fixed bearing",
    )
    ctx.expect_overlap(
        fork_arm,
        tripod_head,
        axes="xy",
        min_overlap=0.12,
        elem_a="azimuth_motor_housing",
        elem_b="fixed_bearing_race",
        name="azimuth bearing races are coaxial",
    )
    ctx.expect_within(
        tube_cradle,
        fork_arm,
        axes="xz",
        margin=0.004,
        inner_elem="altitude_axle",
        outer_elem="altitude_bearing_ring",
        name="altitude axle is captured inside bearing ring",
    )

    front_rest = ctx.part_element_world_aabb(tube_cradle, elem="front_rim")
    pivot_rest = ctx.part_world_position(tube_cradle)
    with ctx.pose({altitude: 1.0}):
        front_raised = ctx.part_element_world_aabb(tube_cradle, elem="front_rim")
    with ctx.pose({azimuth: 1.0}):
        pivot_rotated = ctx.part_world_position(tube_cradle)

    def _mid_z(aabb):
        return None if aabb is None else 0.5 * (aabb[0][2] + aabb[1][2])

    ctx.check(
        "altitude joint raises open aperture",
        front_rest is not None
        and front_raised is not None
        and _mid_z(front_raised) is not None
        and _mid_z(front_rest) is not None
        and _mid_z(front_raised) > _mid_z(front_rest) + 0.35,
        details=f"rest={front_rest}, raised={front_raised}",
    )
    ctx.check(
        "azimuth joint slews whole fork assembly",
        pivot_rest is not None
        and pivot_rotated is not None
        and abs(pivot_rotated[0] - pivot_rest[0]) > 0.10
        and abs(pivot_rotated[1] - pivot_rest[1]) > 0.03,
        details=f"rest={pivot_rest}, rotated={pivot_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
