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
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
)


OUTER_WIDTH = 0.72
OUTER_HEIGHT = 1.20
FRAME_DEPTH = 0.050
STILE_WIDTH = 0.070
RAIL_HEIGHT = 0.075
LOUVER_COUNT = 8
LOUVER_LENGTH = OUTER_WIDTH - 2.0 * STILE_WIDTH - 0.040
LOUVER_HEIGHT = 0.105
LOUVER_THICKNESS = 0.022
PIVOT_LENGTH = 0.055
PIVOT_RADIUS = 0.008
LOUVER_TRAVEL = math.radians(80.0)


def _longitudinal_mesh(profile_width: float, profile_thickness: float, length: float, name: str):
    """Return a rounded, wide louver-like extrusion running along local +X."""
    geom = ExtrudeGeometry(
        superellipse_profile(profile_width, profile_thickness, exponent=2.35, segments=40),
        length,
        cap=True,
        center=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _pivot_ring_mesh():
    ring = TorusGeometry(radius=0.017, tube=0.003, radial_segments=20, tubular_segments=40)
    ring.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(ring, "pivot_ring")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plantation_shutter")

    painted_white = Material("painted_white", rgba=(0.90, 0.86, 0.76, 1.0))
    louver_white = Material("louver_white", rgba=(0.96, 0.93, 0.84, 1.0))
    shadow = Material("recess_shadow", rgba=(0.20, 0.18, 0.15, 1.0))
    pin_material = Material("nylon_pivot_pin", rgba=(0.78, 0.74, 0.64, 1.0))

    frame = model.part("frame")

    side_x = OUTER_WIDTH / 2.0 - STILE_WIDTH / 2.0
    center_z = OUTER_HEIGHT / 2.0
    for index, x in enumerate((-side_x, side_x)):
        frame.visual(
            Box((STILE_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
            origin=Origin(xyz=(x, 0.0, center_z)),
            material=painted_white,
            name=f"side_stile_{index}",
        )

    rail_z = OUTER_HEIGHT / 2.0 - RAIL_HEIGHT / 2.0
    for index, z in enumerate((RAIL_HEIGHT / 2.0, OUTER_HEIGHT - RAIL_HEIGHT / 2.0)):
        frame.visual(
            Box((OUTER_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=painted_white,
            name=f"rail_{index}",
        )

    # Shallow dark reveals just inside the rigid frame make the open louver bay read
    # as a real routed shutter frame rather than four plain bars.
    reveal_depth = 0.006
    reveal_width = 0.010
    inner_half_width = OUTER_WIDTH / 2.0 - STILE_WIDTH
    for index, x in enumerate((-inner_half_width - reveal_width / 2.0, inner_half_width + reveal_width / 2.0)):
        frame.visual(
            Box((reveal_width, reveal_depth, OUTER_HEIGHT - 2.0 * RAIL_HEIGHT)),
            origin=Origin(xyz=(x, -FRAME_DEPTH / 2.0 - reveal_depth / 2.0, center_z)),
            material=shadow,
            name=f"side_reveal_{index}",
        )
    for index, z in enumerate((RAIL_HEIGHT + reveal_width / 2.0, OUTER_HEIGHT - RAIL_HEIGHT - reveal_width / 2.0)):
        frame.visual(
            Box((OUTER_WIDTH - 2.0 * STILE_WIDTH, reveal_depth, reveal_width)),
            origin=Origin(xyz=(0.0, -FRAME_DEPTH / 2.0 - reveal_depth / 2.0, z)),
            material=shadow,
            name=f"rail_reveal_{index}",
        )

    blade_mesh = _longitudinal_mesh(LOUVER_HEIGHT, LOUVER_THICKNESS, LOUVER_LENGTH, "rounded_louver")
    ring_mesh = _pivot_ring_mesh()

    opening_height = OUTER_HEIGHT - 2.0 * RAIL_HEIGHT
    pitch = opening_height / LOUVER_COUNT
    z_positions = [RAIL_HEIGHT + pitch * (i + 0.5) for i in range(LOUVER_COUNT)]
    pivot_center_offset = LOUVER_LENGTH / 2.0 + PIVOT_LENGTH / 2.0 - 0.007

    for louver_index, z in enumerate(z_positions):
        # Fixed bearing rings are visible on the side stiles around each moving pin.
        for side_index, x in enumerate((-inner_half_width - 0.002, inner_half_width + 0.002)):
            frame.visual(
                ring_mesh,
                origin=Origin(xyz=(x, 0.0, z)),
                material=pin_material,
                name=f"pivot_ring_{louver_index}_{side_index}",
            )

        louver = model.part(f"louver_{louver_index}")
        louver.visual(
            blade_mesh,
            origin=Origin(),
            material=louver_white,
            name="blade",
        )
        for side_index, x in enumerate((-pivot_center_offset, pivot_center_offset)):
            louver.visual(
                Cylinder(radius=PIVOT_RADIUS, length=PIVOT_LENGTH),
                origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=pin_material,
                name=f"pivot_{side_index}",
            )

        model.articulation(
            f"frame_to_louver_{louver_index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=louver,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=2.0,
                lower=-LOUVER_TRAVEL / 2.0,
                upper=LOUVER_TRAVEL / 2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    louvers = [object_model.get_part(f"louver_{i}") for i in range(LOUVER_COUNT)]
    joints = [object_model.get_articulation(f"frame_to_louver_{i}") for i in range(LOUVER_COUNT)]

    ctx.check(
        "several independent louvers",
        len(louvers) == LOUVER_COUNT and len(joints) == LOUVER_COUNT,
        details=f"louvers={len(louvers)}, joints={len(joints)}",
    )

    for i, joint in enumerate(joints):
        limits = joint.motion_limits
        travel = None if limits is None else (limits.upper - limits.lower)
        ctx.check(
            f"louver_{i} has 80 degree travel",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(travel - LOUVER_TRAVEL) < math.radians(1.0)
            and joint.axis == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}, limits={limits}",
        )

    for i in range(1, LOUVER_COUNT):
        ctx.expect_gap(
            louvers[i],
            louvers[i - 1],
            axis="z",
            min_gap=0.015,
            max_gap=0.040,
            positive_elem="blade",
            negative_elem="blade",
            name=f"visible gap between louver_{i - 1} and louver_{i}",
        )

    # The visible end pins are intentionally seated into solid side-stile
    # proxies; the exact overlap is local to the pivot pins, like a pin entering
    # a drilled bearing hole in the frame.
    for i, louver in enumerate(louvers):
        for side_index, stile_name in enumerate(("side_stile_0", "side_stile_1")):
            pivot_name = f"pivot_{side_index}"
            ctx.allow_overlap(
                louver,
                frame,
                elem_a=pivot_name,
                elem_b=stile_name,
                reason="The louver pivot pin is intentionally seated into a drilled side-stile bearing.",
            )
            ctx.expect_overlap(
                louver,
                frame,
                axes="x",
                elem_a=pivot_name,
                elem_b=stile_name,
                min_overlap=0.010,
                name=f"{louver.name} {pivot_name} remains inserted in {stile_name}",
            )
            ctx.expect_within(
                louver,
                frame,
                axes="yz",
                inner_elem=pivot_name,
                outer_elem=stile_name,
                margin=0.002,
                name=f"{louver.name} {pivot_name} is centered in {stile_name}",
            )

    test_louver = louvers[3]
    neighbor = louvers[4]
    test_joint = joints[3]
    rest_aabb = ctx.part_element_world_aabb(test_louver, elem="blade")
    rest_neighbor_aabb = ctx.part_element_world_aabb(neighbor, elem="blade")
    with ctx.pose({test_joint: LOUVER_TRAVEL / 2.0}):
        tilted_aabb = ctx.part_element_world_aabb(test_louver, elem="blade")
        neighbor_aabb = ctx.part_element_world_aabb(neighbor, elem="blade")

    rest_y = rest_aabb[1][1] - rest_aabb[0][1] if rest_aabb is not None else 0.0
    tilted_y = tilted_aabb[1][1] - tilted_aabb[0][1] if tilted_aabb is not None else 0.0
    neighbor_rest_y = rest_neighbor_aabb[1][1] - rest_neighbor_aabb[0][1] if rest_neighbor_aabb is not None else 0.0
    neighbor_y = neighbor_aabb[1][1] - neighbor_aabb[0][1] if neighbor_aabb is not None else 0.0
    ctx.check(
        "one louver rotates without driving its neighbor",
        tilted_y > rest_y + 0.040 and abs(neighbor_y - neighbor_rest_y) < 0.005,
        details=f"test rest_y={rest_y}, tilted_y={tilted_y}; neighbor rest_y={neighbor_rest_y}, posed_y={neighbor_y}",
    )

    return ctx.report()


object_model = build_object_model()
