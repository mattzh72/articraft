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

CANOPY_RADIUS = 0.065
CANOPY_HEIGHT = 0.020
CANOPY_WALL = 0.0025
PIVOT_RING_RADIUS = 0.038
PIVOT_PEDESTAL_RADIUS = 0.009
PIVOT_PEDESTAL_HEIGHT = 0.006

ARM_COLLAR_RADIUS = 0.012
ARM_COLLAR_HEIGHT = 0.006
ARM_TUBE_RADIUS = 0.006
ARM_YOKE_X = 0.128
ARM_YOKE_Z = -0.026
YOKE_INNER_FACE_Y = 0.032
YOKE_PLATE_THICKNESS = 0.003
YOKE_PLATE_CENTER_Y = YOKE_INNER_FACE_Y + (YOKE_PLATE_THICKNESS / 2.0)

SPOT_FRONT_X = 0.064
SPOT_REAR_X = -0.002
SPOT_BODY_RADIUS = 0.025
SPOT_BEZEL_RADIUS = 0.028
SPOT_REAR_RADIUS = 0.018
SPOT_CAVITY_RADIUS = 0.022
SPOT_TRUNNION_RADIUS = 0.0045
SPOT_TRUNNION_HALF_SPAN = 0.0373
SPOT_REST_PITCH = 0.0


def _build_canopy_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").workplane(offset=-CANOPY_HEIGHT).circle(CANOPY_RADIUS).extrude(CANOPY_HEIGHT)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=-(CANOPY_HEIGHT - CANOPY_WALL))
        .circle(CANOPY_RADIUS - CANOPY_WALL)
        .extrude(CANOPY_HEIGHT + 0.002)
    )
    canopy = shell.cut(cavity)
    canopy = canopy.union(
        cq.Workplane("XY").workplane(offset=-(CANOPY_HEIGHT + 0.004)).circle(0.018).extrude(0.004)
    )

    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        canopy = canopy.union(
            cq.Workplane("XY")
            .workplane(offset=-(CANOPY_HEIGHT + PIVOT_PEDESTAL_HEIGHT))
            .center(PIVOT_RING_RADIUS * math.cos(angle), PIVOT_RING_RADIUS * math.sin(angle))
            .circle(PIVOT_PEDESTAL_RADIUS)
            .extrude(PIVOT_PEDESTAL_HEIGHT)
        )

    return canopy


def _make_cylinder_between(start_xyz, end_xyz, radius: float) -> cq.Solid:
    dx = end_xyz[0] - start_xyz[0]
    dy = end_xyz[1] - start_xyz[1]
    dz = end_xyz[2] - start_xyz[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start_xyz), cq.Vector(dx, dy, dz))


def _build_arm_shape() -> cq.Workplane:
    shape = cq.Solid.makeCylinder(
        ARM_COLLAR_RADIUS,
        ARM_COLLAR_HEIGHT,
        cq.Vector(0.0, 0.0, -ARM_COLLAR_HEIGHT),
        cq.Vector(0.0, 0.0, 1.0),
    )

    arm_points = [
        (0.010, 0.0, -0.003),
        (0.038, 0.0, -0.008),
        (0.072, 0.0, -0.018),
        (0.100, 0.0, -0.036),
        (0.116, 0.0, -0.050),
    ]
    for start_xyz, end_xyz in zip(arm_points[:-1], arm_points[1:]):
        shape = shape.fuse(_make_cylinder_between(start_xyz, end_xyz, ARM_TUBE_RADIUS))
        shape = shape.fuse(cq.Solid.makeSphere(ARM_TUBE_RADIUS, cq.Vector(*end_xyz)))

    shape = shape.fuse(cq.Workplane("XY").box(0.020, 0.082, 0.006).translate((0.122, 0.0, -0.053)).val())
    return cq.Workplane(obj=shape)


def _build_spot_shape() -> cq.Workplane:
    shell_profile = [
        (SPOT_REAR_X, 0.0),
        (SPOT_REAR_X, SPOT_REAR_RADIUS),
        (0.002, 0.020),
        (0.010, 0.023),
        (0.050, SPOT_BODY_RADIUS),
        (SPOT_FRONT_X, SPOT_BEZEL_RADIUS),
        (SPOT_FRONT_X, SPOT_CAVITY_RADIUS),
        (0.008, SPOT_CAVITY_RADIUS),
        (0.008, 0.0),
    ]
    shell = (
        cq.Workplane("XZ")
        .polyline(shell_profile)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
        .val()
    )
    shell = shell.fuse(
        cq.Solid.makeCylinder(
            SPOT_TRUNNION_RADIUS,
            2.0 * SPOT_TRUNNION_HALF_SPAN,
            cq.Vector(0.0, -SPOT_TRUNNION_HALF_SPAN, 0.0),
            cq.Vector(0.0, 1.0, 0.0),
        )
    )
    return cq.Workplane(obj=shell)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_head_ceiling_light")

    model.material("satin_white", rgba=(0.92, 0.92, 0.90, 1.0))
    model.material("graphite", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("lens", rgba=(0.95, 0.92, 0.82, 1.0))

    canopy_mesh = mesh_from_cadquery(_build_canopy_shape(), "canopy_shell")
    arm_mesh = mesh_from_cadquery(_build_arm_shape(), "arm_body")
    spot_mesh = mesh_from_cadquery(_build_spot_shape(), "spot_shell")

    canopy = model.part("canopy")
    canopy.visual(canopy_mesh, material="satin_white", name="canopy_shell")

    arm_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)

    for index, angle in enumerate(arm_angles):
        arm = model.part(f"arm_{index}")
        arm.visual(arm_mesh, material="satin_white", name="arm_body")
        for rod_index, y in enumerate((-0.040, 0.040)):
            arm.visual(
                Cylinder(radius=0.0028, length=0.028),
                origin=Origin(xyz=(0.126, y, -0.040)),
                material="satin_white",
                name=f"yoke_{rod_index}",
            )

        spot = model.part(f"spot_{index}")
        spot.visual(spot_mesh, material="graphite", name="shell")
        spot.visual(
            Cylinder(radius=SPOT_CAVITY_RADIUS + 0.0001, length=0.0012),
            origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="lens",
            name="lens",
        )

        model.articulation(
            f"canopy_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=canopy,
            child=arm,
            origin=Origin(
                xyz=(
                    PIVOT_RING_RADIUS * math.cos(angle),
                    PIVOT_RING_RADIUS * math.sin(angle),
                    -(CANOPY_HEIGHT + PIVOT_PEDESTAL_HEIGHT),
                ),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                lower=-1.75,
                upper=1.75,
                effort=10.0,
                velocity=1.5,
            ),
        )
        model.articulation(
            f"arm_{index}_to_spot_{index}",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=spot,
            origin=Origin(xyz=(ARM_YOKE_X, 0.0, ARM_YOKE_Z), rpy=(0.0, SPOT_REST_PITCH, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                lower=-0.75,
                upper=0.90,
                effort=4.0,
                velocity=1.8,
            ),
        )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((low + high) / 2.0 for low, high in zip(aabb[0], aabb[1]))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    arm_0 = object_model.get_part("arm_0")
    spot_0 = object_model.get_part("spot_0")
    spot_1 = object_model.get_part("spot_1")
    arm_joint = object_model.get_articulation("canopy_to_arm_0")
    tilt_joint = object_model.get_articulation("arm_0_to_spot_0")

    ctx.expect_contact(
        arm_0,
        canopy,
        elem_a="arm_body",
        elem_b="canopy_shell",
        contact_tol=0.001,
        name="arm_0 shoulder seats against canopy pivot pad",
    )
    ctx.expect_origin_distance(
        spot_0,
        canopy,
        axes="xy",
        min_dist=0.10,
        name="spot_0 projects outward from the compact canopy",
    )
    ctx.expect_origin_distance(
        spot_0,
        spot_1,
        axes="xy",
        min_dist=0.11,
        name="adjacent spots stay separated around the canopy",
    )

    rest_pos = ctx.part_world_position(spot_0)
    with ctx.pose({arm_joint: 0.8}):
        swung_pos = ctx.part_world_position(spot_0)
    ctx.check(
        "arm_0 swings around the canopy",
        rest_pos is not None and swung_pos is not None and swung_pos[1] > rest_pos[1] + 0.05,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    with ctx.pose({tilt_joint: -0.55}):
        low_lens_center = _aabb_center(ctx.part_element_world_aabb(spot_0, elem="lens"))
    with ctx.pose({tilt_joint: 0.60}):
        raised_lens_center = _aabb_center(ctx.part_element_world_aabb(spot_0, elem="lens"))
    ctx.check(
        "spot_0 tilt raises the lamp head",
        low_lens_center is not None
        and raised_lens_center is not None
        and raised_lens_center[2] > low_lens_center[2] + 0.025,
        details=f"low={low_lens_center}, raised={raised_lens_center}",
    )

    return ctx.report()


object_model = build_object_model()
