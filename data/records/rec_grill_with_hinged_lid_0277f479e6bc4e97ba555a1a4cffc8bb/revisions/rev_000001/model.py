from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BLACK_ENAMEL = "black_enamel"
STEEL = "brushed_steel"
RUBBER = "matte_rubber"
WOOD = "warm_wood"


def _rpy_for_cylinder_between(
    p0: tuple[float, float, float], p1: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be distinct")
    vx, vy, vz = dx / length, dy / length, dz / length
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(math.sqrt(vx * vx + vy * vy), vz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    material: str = STEEL,
) -> None:
    mid = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    length = math.dist(p0, p1)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=_rpy_for_cylinder_between(p0, p1)),
        material=material,
        name=name,
    )


def _lower_bowl_mesh():
    # Profiles are (radius, z).  This is a real open thin-walled fire bowl
    # with an annular lip rather than a filled solid hemisphere.
    outer = [
        (0.070, 0.442),
        (0.145, 0.468),
        (0.220, 0.525),
        (0.270, 0.600),
        (0.296, 0.680),
        (0.305, 0.718),
    ]
    inner = [
        (0.052, 0.462),
        (0.117, 0.486),
        (0.190, 0.540),
        (0.238, 0.610),
        (0.262, 0.690),
        (0.270, 0.710),
    ]
    return LatheGeometry.from_shell_profiles(
        outer, inner, segments=72, start_cap="flat", end_cap="flat"
    )


def _lid_dome_mesh():
    r = 0.285
    # Local frame: the hinge axis is the local Y axis at the rear tangent of
    # the lid.  The closed dome extends along +X from that hinge line.
    outer = [
        (0.285, 0.008),
        (0.270, 0.055),
        (0.230, 0.130),
        (0.165, 0.205),
        (0.080, 0.247),
        (0.022, 0.258),
    ]
    inner = [
        (0.258, 0.008),
        (0.238, 0.045),
        (0.200, 0.110),
        (0.140, 0.170),
        (0.065, 0.205),
        (0.020, 0.220),
    ]
    geom = LatheGeometry.from_shell_profiles(
        outer, inner, segments=72, start_cap="flat", end_cap="flat"
    )
    return geom.translate(r, 0.0, 0.0)


def _support_ring_mesh(outer_radius: float, inner_radius: float, height: float):
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height * 0.5, both=True)
    )


def _wheel_mesh():
    # A broad rubber wheel with a real through-bore so it can spin around the
    # shared fixed axle without intersecting it.
    return (
        cq.Workplane("XY")
        .circle(0.086)
        .circle(0.016)
        .extrude(0.026, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kettle_charcoal_grill")
    model.material(BLACK_ENAMEL, rgba=(0.015, 0.014, 0.012, 1.0))
    model.material(STEEL, rgba=(0.58, 0.58, 0.55, 1.0))
    model.material(RUBBER, rgba=(0.025, 0.025, 0.023, 1.0))
    model.material(WOOD, rgba=(0.58, 0.34, 0.16, 1.0))

    body = model.part("grill_body")
    body.visual(
        mesh_from_geometry(_lower_bowl_mesh(), "lower_fire_bowl"),
        material=BLACK_ENAMEL,
        name="fire_bowl",
    )
    body.visual(
        mesh_from_cadquery(_support_ring_mesh(0.310, 0.268, 0.020), "rolled_bowl_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.718)),
        material=BLACK_ENAMEL,
        name="bowl_rim",
    )
    body.visual(
        mesh_from_cadquery(_support_ring_mesh(0.235, 0.180, 0.024), "stand_support_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.510)),
        material=STEEL,
        name="support_ring",
    )

    _add_tube(body, "rear_leg_0", (-0.190, -0.120, 0.510), (-0.340, -0.285, 0.130), 0.014)
    _add_tube(body, "rear_leg_1", (-0.190, 0.120, 0.510), (-0.340, 0.285, 0.130), 0.014)
    _add_tube(body, "front_leg", (0.205, 0.0, 0.510), (0.355, 0.0, 0.040), 0.014)
    _add_tube(body, "front_brace_0", (0.340, 0.0, 0.070), (-0.340, -0.205, 0.130), 0.010)
    _add_tube(body, "front_brace_1", (0.340, 0.0, 0.070), (-0.340, 0.205, 0.130), 0.010)
    _add_tube(body, "wheel_axle", (-0.340, -0.375, 0.130), (-0.340, 0.375, 0.130), 0.012)
    for idx, y in enumerate((-0.360, 0.360)):
        body.visual(
            Cylinder(radius=0.035, length=0.012),
            origin=Origin(xyz=(-0.340, y, 0.130), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=STEEL,
            name=f"axle_collar_{idx}",
        )
    body.visual(
        Cylinder(radius=0.050, length=0.050),
        origin=Origin(xyz=(0.355, 0.0, 0.025)),
        material=RUBBER,
        name="front_foot",
    )

    # Rear hinge knuckles and welded tabs fixed to the bowl rim.
    for idx, y in enumerate((-0.115, 0.115)):
        body.visual(
            Box((0.080, 0.040, 0.024)),
            origin=Origin(xyz=(-0.292, y, 0.712)),
            material=STEEL,
            name=f"hinge_tab_{idx}",
        )
        body.visual(
            Cylinder(radius=0.018, length=0.070),
            origin=Origin(xyz=(-0.305, y, 0.742), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=STEEL,
            name=f"hinge_knuckle_{idx}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_lid_dome_mesh(), "domed_lid"),
        material=BLACK_ENAMEL,
        name="lid_dome",
    )
    lid.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(-0.020, 0.0, 0.012), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="hinge_sleeve",
    )
    lid.visual(
        Box((0.090, 0.095, 0.024)),
        origin=Origin(xyz=(0.030, 0.0, 0.026)),
        material=STEEL,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.285, -0.065, 0.245)),
        material=STEEL,
        name="handle_post_0",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.285, 0.065, 0.245)),
        material=STEEL,
        name="handle_post_1",
    )
    lid.visual(
        Cylinder(radius=0.026, length=0.180),
        origin=Origin(xyz=(0.285, 0.0, 0.312), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=WOOD,
        name="top_handle",
    )

    wheel_mesh = mesh_from_cadquery(_wheel_mesh(), "bored_wheel", tolerance=0.001)
    for idx, y in enumerate((-0.392, 0.392)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=RUBBER,
            name="wheel",
        )
        model.articulation(
            f"axle_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.340, y, 0.130)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.285, 0.0, 0.730)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=math.radians(100.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("grill_body")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    hinge = object_model.get_articulation("rear_lid_hinge")
    wheel_joint_0 = object_model.get_articulation("axle_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("axle_to_wheel_1")

    ctx.check(
        "lid hinge opens about 100 degrees",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and abs(hinge.motion_limits.upper - math.radians(100.0)) < 1.0e-6,
        details=f"limits={hinge.motion_limits}",
    )
    ctx.check(
        "wheels are continuous spins on a shared axis",
        wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS
        and tuple(wheel_joint_0.axis) == (0.0, 1.0, 0.0)
        and tuple(wheel_joint_1.axis) == (0.0, 1.0, 0.0),
        details=f"types={wheel_joint_0.articulation_type},{wheel_joint_1.articulation_type}",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_dome",
        negative_elem="bowl_rim",
        min_gap=0.0,
        max_gap=0.025,
        name="closed lid is seated just above the bowl rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_dome",
        elem_b="bowl_rim",
        min_overlap=0.45,
        name="round lid footprint covers the fire bowl",
    )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: math.radians(100.0), wheel_joint_0: math.pi, wheel_joint_1: -math.pi}):
        opened_aabb = ctx.part_world_aabb(lid)
        ctx.expect_origin_gap(wheel_1, wheel_0, axis="y", min_gap=0.75, max_gap=0.90)
    ctx.check(
        "lid rises when opened",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
