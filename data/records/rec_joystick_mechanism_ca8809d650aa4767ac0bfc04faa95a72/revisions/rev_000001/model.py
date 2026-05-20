from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PIVOT_Z = 0.112


def _base_cheek_shape() -> object:
    """Rounded, bored outer support cheek, local bottom on z=0."""
    thickness_x = 0.018
    depth_y = 0.078
    bore_z = PIVOT_Z - 0.044
    bore_radius = 0.0125

    lower = (
        cq.Workplane("XY")
        .box(thickness_x, depth_y, bore_z)
        .translate((0.0, 0.0, bore_z * 0.5))
    )
    cap = (
        cq.Workplane("YZ")
        .circle(depth_y * 0.5)
        .extrude(thickness_x, both=True)
        .translate((0.0, 0.0, bore_z))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(bore_radius)
        .extrude(thickness_x * 3.0, both=True)
        .translate((0.0, 0.0, bore_z))
    )
    return lower.union(cap).cut(bore)


def _inner_cheek_shape() -> object:
    """Rounded, bored cheek on the rotating outer gimbal, centered on the pivot."""
    width_x = 0.066
    thickness_y = 0.014
    lower_depth = 0.046
    bore_radius = 0.0105

    lower = (
        cq.Workplane("XY")
        .box(width_x, thickness_y, lower_depth)
        .translate((0.0, 0.0, -lower_depth * 0.5))
    )
    cap = cq.Workplane("XZ").circle(width_x * 0.5).extrude(thickness_y, both=True)
    bore = cq.Workplane("XZ").circle(bore_radius).extrude(thickness_y * 3.0, both=True)
    return lower.union(cap).cut(bore)


def _cylinder_rpy_between(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_control_stick")

    powder_black = model.material("powder_black", rgba=(0.08, 0.085, 0.09, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.35, 0.36, 0.37, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    rubber = model.material("molded_rubber", rgba=(0.025, 0.025, 0.027, 1.0))
    safety_label = model.material("safety_label", rgba=(0.94, 0.73, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.220, 0.220, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=powder_black,
        name="ground_plate",
    )
    base.visual(
        Box((0.150, 0.130, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=graphite,
        name="raised_plinth",
    )
    base.visual(
        Cylinder(radius=0.062, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=dark_steel,
        name="round_mount",
    )

    base_cheek_mesh = mesh_from_cadquery(_base_cheek_shape(), "base_outer_cheek")
    base.visual(
        base_cheek_mesh,
        origin=Origin(xyz=(-0.078, 0.0, 0.044)),
        material=graphite,
        name="base_cheek_0",
    )
    base.visual(
        base_cheek_mesh,
        origin=Origin(xyz=(0.078, 0.0, 0.044)),
        material=graphite,
        name="base_cheek_1",
    )

    bearing_ring_mesh = mesh_from_geometry(
        TorusGeometry(0.0144, 0.0022, radial_segments=28, tubular_segments=10),
        "base_bearing_ring",
    )
    for index, x in enumerate((-0.0875, 0.0875)):
        base.visual(
            bearing_ring_mesh,
            origin=Origin(xyz=(x, 0.0, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"outer_bearing_{index}",
        )

    for index, (x, y) in enumerate(((-0.086, -0.086), (0.086, -0.086), (0.086, 0.086), (-0.086, 0.086))):
        base.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, y, 0.023)),
            material=brushed_steel,
            name=f"mount_bolt_{index}",
        )
    base.visual(
        Box((0.085, 0.018, 0.002)),
        origin=Origin(xyz=(0.0, -0.104, 0.021)),
        material=safety_label,
        name="front_warning_label",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.220, 0.220, 0.160)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    outer_gimbal = model.part("outer_gimbal")
    inner_cheek_mesh = mesh_from_cadquery(_inner_cheek_shape(), "outer_gimbal_inner_cheek")
    outer_gimbal.visual(
        inner_cheek_mesh,
        origin=Origin(xyz=(0.0, -0.052, 0.0)),
        material=dark_steel,
        name="inner_cheek_0",
    )
    outer_gimbal.visual(
        inner_cheek_mesh,
        origin=Origin(xyz=(0.0, 0.052, 0.0)),
        material=dark_steel,
        name="inner_cheek_1",
    )
    outer_gimbal.visual(
        Box((0.118, 0.118, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=dark_steel,
        name="lower_bridge",
    )
    outer_gimbal.visual(
        Box((0.116, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.052, -0.038)),
        material=dark_steel,
        name="cheek_foot_0",
    )
    outer_gimbal.visual(
        Box((0.116, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.052, -0.038)),
        material=dark_steel,
        name="cheek_foot_1",
    )
    outer_gimbal.visual(
        Cylinder(radius=0.0074, length=0.036),
        origin=Origin(xyz=(-0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="outer_trunnion_0",
    )
    outer_gimbal.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(-0.05625, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="trunnion_collar_0",
    )
    outer_gimbal.visual(
        Cylinder(radius=0.0074, length=0.036),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="outer_trunnion_1",
    )
    outer_gimbal.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.05625, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="trunnion_collar_1",
    )
    for index, (a, b) in enumerate(
        (
            ((-0.056, 0.0, 0.0), (-0.033, -0.052, 0.0)),
            ((-0.056, 0.0, 0.0), (-0.033, 0.052, 0.0)),
            ((0.056, 0.0, 0.0), (0.033, -0.052, 0.0)),
            ((0.056, 0.0, 0.0), (0.033, 0.052, 0.0)),
        )
    ):
        midpoint = tuple((a[i] + b[i]) * 0.5 for i in range(3))
        length = math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
        outer_gimbal.visual(
            Cylinder(radius=0.0052, length=length),
            origin=Origin(xyz=midpoint, rpy=_cylinder_rpy_between(a, b)),
            material=dark_steel,
            name=f"side_strut_{index}",
        )
    inner_bearing_mesh = mesh_from_geometry(
        TorusGeometry(0.00875, 0.00245, radial_segments=28, tubular_segments=10),
        "inner_bearing_ring",
    )
    outer_gimbal.visual(
        inner_bearing_mesh,
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="inner_bearing_0",
    )
    outer_gimbal.visual(
        inner_bearing_mesh,
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="inner_bearing_1",
    )
    outer_gimbal.inertial = Inertial.from_geometry(
        Box((0.155, 0.135, 0.090)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0064, length=0.146),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="inner_trunnion",
    )
    handle.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="central_hub",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=brushed_steel,
        name="steel_stem",
    )
    handle.visual(
        Cylinder(radius=0.025, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=rubber,
        name="grip_flange",
    )
    handle.visual(
        Cylinder(radius=0.020, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=rubber,
        name="rubber_grip",
    )
    for index, z in enumerate((0.118, 0.146, 0.174, 0.202)):
        handle.visual(
            Cylinder(radius=0.0218, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=rubber,
            name=f"grip_rib_{index}",
        )
    handle.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=rubber,
        name="rounded_cap",
    )
    handle.visual(
        Box((0.006, 0.024, 0.003)),
        origin=Origin(xyz=(0.0, -0.018, 0.215)),
        material=safety_label,
        name="cap_index_mark",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.054, 0.154, 0.238)),
        mass=0.58,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    model.articulation(
        "base_to_outer_gimbal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_gimbal,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "outer_gimbal_to_handle",
        ArticulationType.REVOLUTE,
        parent=outer_gimbal,
        child=handle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.2, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer = object_model.get_part("outer_gimbal")
    handle = object_model.get_part("handle")
    base_joint = object_model.get_articulation("base_to_outer_gimbal")
    handle_joint = object_model.get_articulation("outer_gimbal_to_handle")

    ctx.check(
        "orthogonal gimbal axes",
        abs(sum(a * b for a, b in zip(base_joint.axis, handle_joint.axis))) < 1e-6,
        details=f"axes were {base_joint.axis} and {handle_joint.axis}",
    )
    ctx.expect_overlap(
        outer,
        base,
        axes="x",
        elem_a="outer_trunnion_0",
        elem_b="base_cheek_0",
        min_overlap=0.010,
        name="outer trunnion passes through first base cheek",
    )
    ctx.expect_overlap(
        outer,
        base,
        axes="x",
        elem_a="outer_trunnion_1",
        elem_b="base_cheek_1",
        min_overlap=0.010,
        name="outer trunnion passes through second base cheek",
    )
    ctx.expect_within(
        handle,
        outer,
        axes="xz",
        inner_elem="inner_trunnion",
        outer_elem="inner_cheek_0",
        margin=0.002,
        name="handle trunnion is captured by first inner cheek",
    )
    ctx.expect_within(
        handle,
        outer,
        axes="xz",
        inner_elem="inner_trunnion",
        outer_elem="inner_cheek_1",
        margin=0.002,
        name="handle trunnion is captured by second inner cheek",
    )
    ctx.allow_overlap(
        handle,
        outer,
        elem_a="inner_trunnion",
        elem_b="inner_bearing_0",
        reason="The handle shaft is intentionally captured with a tiny seated interference in the first bearing ring.",
    )
    ctx.allow_overlap(
        handle,
        outer,
        elem_a="inner_trunnion",
        elem_b="inner_bearing_1",
        reason="The handle shaft is intentionally captured with a tiny seated interference in the second bearing ring.",
    )
    ctx.expect_within(
        handle,
        outer,
        axes="xz",
        inner_elem="inner_trunnion",
        outer_elem="inner_bearing_0",
        margin=0.001,
        name="shaft is centered in first bearing ring",
    )
    ctx.expect_within(
        handle,
        outer,
        axes="xz",
        inner_elem="inner_trunnion",
        outer_elem="inner_bearing_1",
        margin=0.001,
        name="shaft is centered in second bearing ring",
    )
    ctx.expect_overlap(
        handle,
        outer,
        axes="y",
        elem_a="inner_trunnion",
        elem_b="inner_bearing_0",
        min_overlap=0.003,
        name="shaft passes through first bearing ring",
    )
    ctx.expect_overlap(
        handle,
        outer,
        axes="y",
        elem_a="inner_trunnion",
        elem_b="inner_bearing_1",
        min_overlap=0.003,
        name="shaft passes through second bearing ring",
    )

    rest_cap = ctx.part_element_world_aabb(handle, elem="rounded_cap")
    with ctx.pose({base_joint: 0.40}):
        tilted_cap_y = ctx.part_element_world_aabb(handle, elem="rounded_cap")
    with ctx.pose({handle_joint: 0.40}):
        tilted_cap_x = ctx.part_element_world_aabb(handle, elem="rounded_cap")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_center = _aabb_center(rest_cap)
    y_tilt_center = _aabb_center(tilted_cap_y)
    x_tilt_center = _aabb_center(tilted_cap_x)
    ctx.check(
        "outer gimbal tilts the handle across y",
        rest_center is not None
        and y_tilt_center is not None
        and abs(y_tilt_center[1] - rest_center[1]) > 0.045,
        details=f"rest={rest_center}, tilted={y_tilt_center}",
    )
    ctx.check(
        "inner gimbal tilts the handle across x",
        rest_center is not None
        and x_tilt_center is not None
        and abs(x_tilt_center[0] - rest_center[0]) > 0.045,
        details=f"rest={rest_center}, tilted={x_tilt_center}",
    )

    return ctx.report()


object_model = build_object_model()
