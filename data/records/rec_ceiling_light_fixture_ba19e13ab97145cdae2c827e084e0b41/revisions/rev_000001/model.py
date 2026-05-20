from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BRANCH_COUNT = 5
BRANCH_NAMES = tuple(f"branch_{index}" for index in range(BRANCH_COUNT))
JOINT_NAMES = tuple(f"canopy_to_branch_{index}" for index in range(BRANCH_COUNT))

CANOPY_RADIUS = 0.092
CANOPY_THICKNESS = 0.014
TRIM_RADIUS = 0.060
TRIM_THICKNESS = 0.010
HUB_RADIUS = 0.046
HUB_HEIGHT = 0.034
FINIAL_RADIUS = 0.024
FINIAL_THICKNESS = 0.012

HINGE_RADIUS = 0.0515
HINGE_Z = -0.034
HUB_CENTER_Z = -CANOPY_THICKNESS - HUB_HEIGHT / 2.0
TRIM_CENTER_Z = -CANOPY_THICKNESS - TRIM_THICKNESS / 2.0
FINIAL_CENTER_Z = -CANOPY_THICKNESS - HUB_HEIGHT - FINIAL_THICKNESS / 2.0
REST_BRANCH_PITCH = 0.45

CHEEK_RADIUS = 0.006
CHEEK_LENGTH = 0.006
CHEEK_OFFSET = 0.007

BARREL_RADIUS = 0.0052
BARREL_LENGTH = 0.008
COLLAR_RADIUS = 0.0075
COLLAR_LENGTH = 0.022
BRIDGE_RADIUS = 0.0035
BRIDGE_LENGTH = 0.012
STEM_RADIUS = 0.005
STEM_LENGTH = 0.134
NECK_RADIUS = 0.010
NECK_LENGTH = 0.018
SOCKET_OFFSET = 0.150


def _cylinder_along_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _cylinder_along_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _build_socket_shape() -> cq.Workplane:
    outer_collar = cq.Workplane("YZ").circle(0.0125).extrude(0.010)
    outer_body = cq.Workplane("YZ").workplane(offset=0.008).circle(0.0160).extrude(0.030)
    outer_rim = cq.Workplane("YZ").workplane(offset=0.036).circle(0.0175).extrude(0.006)
    cavity = cq.Workplane("YZ").workplane(offset=0.010).circle(0.0108).extrude(0.032)
    return outer_collar.union(outer_body).union(outer_rim).cut(cavity)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_flush_branch_light")

    aged_brass = model.material("aged_brass", rgba=(0.70, 0.58, 0.32, 1.0))
    dark_socket = model.material("dark_socket", rgba=(0.17, 0.17, 0.19, 1.0))

    socket_mesh = mesh_from_cadquery(_build_socket_shape(), "branch_socket")

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=CANOPY_RADIUS, length=CANOPY_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -CANOPY_THICKNESS / 2.0)),
        material=aged_brass,
        name="canopy_plate",
    )
    canopy.visual(
        Cylinder(radius=TRIM_RADIUS, length=TRIM_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, TRIM_CENTER_Z)),
        material=aged_brass,
        name="trim_ring",
    )
    canopy.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, HUB_CENTER_Z)),
        material=aged_brass,
        name="hub_body",
    )
    canopy.visual(
        Cylinder(radius=FINIAL_RADIUS, length=FINIAL_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, FINIAL_CENTER_Z)),
        material=aged_brass,
        name="lower_finial",
    )

    barrel_geometry, barrel_orientation = _cylinder_along_y(BARREL_RADIUS, BARREL_LENGTH)
    collar_geometry, collar_orientation = _cylinder_along_x(COLLAR_RADIUS, COLLAR_LENGTH)
    bridge_geometry, bridge_orientation = _cylinder_along_x(BRIDGE_RADIUS, BRIDGE_LENGTH)
    stem_geometry, stem_orientation = _cylinder_along_x(STEM_RADIUS, STEM_LENGTH)
    neck_geometry, neck_orientation = _cylinder_along_x(NECK_RADIUS, NECK_LENGTH)

    for index in range(BRANCH_COUNT):
        angle = 2.0 * math.pi * index / BRANCH_COUNT
        c = math.cos(angle)
        s = math.sin(angle)
        tangent = (-s, c, 0.0)

        hinge_xyz = (HINGE_RADIUS * c, HINGE_RADIUS * s, HINGE_Z)
        for cheek_index, cheek_sign in enumerate((-1.0, 1.0)):
            cheek_xyz = (
                hinge_xyz[0] + cheek_sign * CHEEK_OFFSET * tangent[0],
                hinge_xyz[1] + cheek_sign * CHEEK_OFFSET * tangent[1],
                hinge_xyz[2],
            )
            canopy.visual(
                Cylinder(radius=CHEEK_RADIUS, length=CHEEK_LENGTH),
                origin=Origin(xyz=cheek_xyz, rpy=(-math.pi / 2.0, 0.0, angle)),
                material=aged_brass,
                name=f"hinge_cheek_{index}_{cheek_index}",
            )

        branch = model.part(BRANCH_NAMES[index])
        branch.visual(
            barrel_geometry,
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=barrel_orientation.rpy),
            material=aged_brass,
            name="hinge_barrel",
        )
        branch.visual(
            collar_geometry,
            origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=collar_orientation.rpy),
            material=aged_brass,
            name="shoulder_collar",
        )
        branch.visual(
            bridge_geometry,
            origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=bridge_orientation.rpy),
            material=aged_brass,
            name="pivot_stub",
        )
        branch.visual(
            stem_geometry,
            origin=Origin(xyz=(0.079, 0.0, 0.0), rpy=stem_orientation.rpy),
            material=aged_brass,
            name="stem",
        )
        branch.visual(
            neck_geometry,
            origin=Origin(xyz=(0.147, 0.0, 0.0), rpy=neck_orientation.rpy),
            material=dark_socket,
            name="socket_neck",
        )
        branch.visual(
            socket_mesh,
            origin=Origin(xyz=(SOCKET_OFFSET, 0.0, 0.0)),
            material=dark_socket,
            name="socket_shell",
        )

        model.articulation(
            JOINT_NAMES[index],
            ArticulationType.REVOLUTE,
            parent=canopy,
            child=branch,
            origin=Origin(xyz=hinge_xyz, rpy=(0.0, REST_BRANCH_PITCH, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.2,
                lower=-0.25,
                upper=0.55,
            ),
        )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[i] + maximum[i]) / 2.0 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    branches = [object_model.get_part(name) for name in BRANCH_NAMES]
    joints = [object_model.get_articulation(name) for name in JOINT_NAMES]

    ctx.check(
        "five articulated branches are present",
        len(branches) == BRANCH_COUNT and len(joints) == BRANCH_COUNT,
        details=f"branches={len(branches)}, joints={len(joints)}",
    )

    adjacent_gap = 2.0 * HINGE_RADIUS * math.sin(math.pi / BRANCH_COUNT)
    for index in range(BRANCH_COUNT):
        ctx.expect_origin_distance(
            branches[index],
            branches[(index + 1) % BRANCH_COUNT],
            axes="xy",
            min_dist=adjacent_gap - 0.003,
            max_dist=adjacent_gap + 0.003,
            name=f"adjacent hinge spacing {index}",
        )

    canopy_aabb = ctx.part_world_aabb(canopy)
    canopy_bottom = canopy_aabb[0][2] if canopy_aabb is not None else None
    socket_centers: list[tuple[float, float, float]] = []

    for branch in branches:
        socket_center = _aabb_center(ctx.part_element_world_aabb(branch, elem="socket_shell"))
        if socket_center is not None:
            socket_centers.append(socket_center)

    socket_ring_ok = len(socket_centers) == BRANCH_COUNT
    if socket_ring_ok and canopy_bottom is not None:
        socket_ring_ok = all(
            0.19 <= math.hypot(center[0], center[1]) <= 0.25 and center[2] < canopy_bottom - 0.020
            for center in socket_centers
        )
    ctx.check(
        "five sockets form a lowered ring around the canopy",
        socket_ring_ok,
        details=f"canopy_bottom={canopy_bottom}, socket_centers={socket_centers}",
    )

    angle_distribution_ok = False
    if len(socket_centers) == BRANCH_COUNT:
        sorted_angles = sorted(math.atan2(center[1], center[0]) % (2.0 * math.pi) for center in socket_centers)
        angle_steps = [
            (sorted_angles[(index + 1) % BRANCH_COUNT] - sorted_angles[index]) % (2.0 * math.pi)
            for index in range(BRANCH_COUNT)
        ]
        angle_distribution_ok = all(0.95 <= step <= 1.55 for step in angle_steps)
        ctx.check(
            "socket layout stays evenly distributed",
            angle_distribution_ok,
            details=f"angles={sorted_angles}, steps={angle_steps}",
        )
    else:
        ctx.fail("socket layout stays evenly distributed", details=f"socket_centers={socket_centers}")

    reference_branch = branches[0]
    neighboring_branch = branches[1]
    reference_joint = joints[0]
    reference_upper = reference_joint.motion_limits.upper if reference_joint.motion_limits is not None else None

    reference_rest = _aabb_center(ctx.part_element_world_aabb(reference_branch, elem="socket_shell"))
    neighbor_rest = _aabb_center(ctx.part_element_world_aabb(neighboring_branch, elem="socket_shell"))
    moved_reference = None
    moved_neighbor = None

    if reference_upper is not None:
        with ctx.pose({reference_joint: reference_upper}):
            moved_reference = _aabb_center(ctx.part_element_world_aabb(reference_branch, elem="socket_shell"))
            moved_neighbor = _aabb_center(ctx.part_element_world_aabb(neighboring_branch, elem="socket_shell"))

    branch_motion_ok = (
        reference_rest is not None
        and neighbor_rest is not None
        and moved_reference is not None
        and moved_neighbor is not None
        and moved_reference[2] < reference_rest[2] - 0.030
        and abs(moved_neighbor[2] - neighbor_rest[2]) < 0.005
    )
    ctx.check(
        "branches pivot independently at the canopy",
        branch_motion_ok,
        details=(
            f"reference_rest={reference_rest}, moved_reference={moved_reference}, "
            f"neighbor_rest={neighbor_rest}, moved_neighbor={moved_neighbor}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
