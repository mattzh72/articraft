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

BASE_BAR_LENGTH = 0.58
BASE_BAR_WIDTH = 0.055
BASE_BAR_THICKNESS = 0.028
BASE_BAR_Z = 0.099
HUB_RADIUS = 0.11
HUB_THICKNESS = 0.032
HUB_Z = 0.100
COLUMN_RADIUS = 0.022
COLUMN_HEIGHT = 0.90
SHOULDER_Z = 1.050
LOWER_ARM_LENGTH = 0.42
UPPER_ARM_LENGTH = 0.38
WHEEL_RADIUS = 0.022


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY", origin=center).box(*size)


def _cylinder_x(
    center: tuple[float, float, float], radius: float, length: float
) -> cq.Workplane:
    return cq.Workplane("YZ", origin=center).circle(radius).extrude(length / 2.0, both=True)


def _cylinder_y(
    center: tuple[float, float, float], radius: float, length: float
) -> cq.Workplane:
    return cq.Workplane("XZ", origin=center).circle(radius).extrude(length / 2.0, both=True)


def _cylinder_z(
    center: tuple[float, float, float], radius: float, length: float
) -> cq.Workplane:
    return cq.Workplane("XY", origin=center).circle(radius).extrude(length / 2.0, both=True)


def _combine(shapes: list[cq.Workplane]) -> cq.Workplane:
    shape = shapes[0]
    for extra in shapes[1:]:
        shape = shape.union(extra)
    return shape


def _caster_shape(x: float, y: float, *, wheel_axis: str) -> cq.Workplane:
    bracket_size = (0.022, 0.036, 0.028) if wheel_axis == "y" else (0.036, 0.022, 0.028)
    wheel = (
        _cylinder_y((x, y, WHEEL_RADIUS), WHEEL_RADIUS, 0.018)
        if wheel_axis == "y"
        else _cylinder_x((x, y, WHEEL_RADIUS), WHEEL_RADIUS, 0.018)
    )
    return _combine(
        [
            _cylinder_z((x, y, 0.070), 0.008, 0.030),
            _cylinder_z((x, y, 0.056), 0.012, 0.012),
            _box((x, y, 0.041), bracket_size),
            wheel,
        ]
    )


def _build_base_shape() -> cq.Workplane:
    hub_top_z = HUB_Z + HUB_THICKNESS / 2.0
    column_top_z = hub_top_z + COLUMN_HEIGHT
    caster_offset = BASE_BAR_LENGTH / 2.0 - 0.020

    shapes = [
        _box((0.0, 0.0, BASE_BAR_Z), (BASE_BAR_LENGTH, BASE_BAR_WIDTH, BASE_BAR_THICKNESS)),
        _box((0.0, 0.0, BASE_BAR_Z), (BASE_BAR_WIDTH, BASE_BAR_LENGTH, BASE_BAR_THICKNESS)),
        _cylinder_z((0.0, 0.0, HUB_Z), HUB_RADIUS, HUB_THICKNESS),
        _cylinder_z((0.0, 0.0, 0.128), 0.050, 0.040),
        _cylinder_z((0.0, 0.0, hub_top_z + COLUMN_HEIGHT / 2.0), COLUMN_RADIUS, COLUMN_HEIGHT),
        _box((0.0, 0.0, column_top_z - 0.018), (0.026, 0.042, 0.044)),
        _box((0.030, 0.0, SHOULDER_Z - 0.022), (0.060, 0.028, 0.022)),
        _caster_shape(caster_offset, 0.0, wheel_axis="y"),
        _caster_shape(-caster_offset, 0.0, wheel_axis="y"),
        _caster_shape(0.0, caster_offset, wheel_axis="x"),
        _caster_shape(0.0, -caster_offset, wheel_axis="x"),
    ]
    return _combine(shapes)


def _build_lower_arm_shape() -> cq.Workplane:
    shapes = [
        _box((0.030, 0.0, 0.0), (0.060, 0.028, 0.022)),
        _box((0.095, 0.020, 0.0), (0.110, 0.016, 0.018)),
        _box((0.095, -0.020, 0.0), (0.110, 0.016, 0.018)),
        _box((0.220, 0.030, 0.0), (0.250, 0.012, 0.016)),
        _box((0.220, -0.030, 0.0), (0.250, 0.012, 0.016)),
        _box((0.360, 0.020, 0.0), (0.060, 0.016, 0.020)),
        _box((0.360, -0.020, 0.0), (0.060, 0.016, 0.020)),
        _box((0.402, 0.018, 0.0), (0.036, 0.008, 0.058)),
        _box((0.402, -0.018, 0.0), (0.036, 0.008, 0.058)),
    ]
    return _combine(shapes)


def _build_upper_arm_shape() -> cq.Workplane:
    shapes = [
        _box((0.028, 0.0, 0.0), (0.056, 0.028, 0.020)),
        _box((0.082, 0.018, 0.0), (0.100, 0.014, 0.016)),
        _box((0.082, -0.018, 0.0), (0.100, 0.014, 0.016)),
        _box((0.190, 0.026, 0.0), (0.240, 0.010, 0.014)),
        _box((0.190, -0.026, 0.0), (0.240, 0.010, 0.014)),
        _box((0.326, 0.018, 0.0), (0.056, 0.014, 0.018)),
        _box((0.326, -0.018, 0.0), (0.056, 0.014, 0.018)),
        _box((0.366, 0.018, 0.0), (0.028, 0.008, 0.046)),
        _box((0.366, -0.018, 0.0), (0.028, 0.008, 0.046)),
    ]
    return _combine(shapes)


def _build_head_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("YZ", origin=(0.082, 0.0, 0.095))
        .circle(0.105)
        .circle(0.079)
        .extrude(0.015, both=True)
    )
    shapes = [
        _box((0.024, 0.0, 0.026), (0.048, 0.028, 0.052)),
        _box((0.062, 0.0, 0.010), (0.050, 0.036, 0.022)),
        ring,
        _box((0.101, 0.025, 0.193), (0.014, 0.010, 0.016)),
        _box((0.101, -0.025, 0.193), (0.014, 0.010, 0.016)),
    ]
    return _combine(shapes)


def _build_cover_shape() -> cq.Workplane:
    cover_disk = _cylinder_x((0.0, 0.0, -0.103), 0.103, 0.008)
    shapes = [
        _cylinder_y((0.0, 0.0, 0.0), 0.006, 0.040),
        cover_disk,
        _box((0.012, 0.0, -0.194), (0.020, 0.034, 0.010)),
    ]
    return _combine(shapes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnifier_floor_lamp")

    model.material("base_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("arm_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("head_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("cover_black", rgba=(0.16, 0.16, 0.18, 1.0))
    model.material("lens_glass", rgba=(0.78, 0.89, 0.96, 0.45))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "lamp_base"),
        material="base_gray",
        name="base_shell",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_build_lower_arm_shape(), "lower_arm"),
        material="arm_silver",
        name="lower_arm_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_build_upper_arm_shape(), "upper_arm"),
        material="arm_silver",
        name="upper_arm_shell",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_build_head_shape(), "head_shell"),
        material="head_black",
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.079, length=0.006),
        origin=Origin(xyz=(0.088, 0.0, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="lens_glass",
        name="lens",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_build_cover_shape(), "lens_cover"),
        material="cover_black",
        name="cover_plate",
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.036, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.2,
            lower=math.radians(-35.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=math.radians(-20.0),
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "upper_arm_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.6,
            lower=math.radians(-45.0),
            upper=math.radians(45.0),
        ),
    )
    model.articulation(
        "head_to_cover",
        ArticulationType.REVOLUTE,
        parent=head,
        child=cover,
        origin=Origin(xyz=(0.114, 0.0, 0.195)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head = object_model.get_part("head")
    cover = object_model.get_part("cover")
    upper_arm = object_model.get_part("upper_arm")

    base_hinge = object_model.get_articulation("base_to_lower_arm")
    elbow_hinge = object_model.get_articulation("lower_arm_to_upper_arm")
    head_hinge = object_model.get_articulation("upper_arm_to_head")
    cover_hinge = object_model.get_articulation("head_to_cover")

    base_upper = base_hinge.motion_limits.upper if base_hinge.motion_limits else None
    elbow_upper = elbow_hinge.motion_limits.upper if elbow_hinge.motion_limits else None
    head_upper = head_hinge.motion_limits.upper if head_hinge.motion_limits else None

    def aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    ctx.expect_overlap(
        cover,
        head,
        axes="yz",
        elem_a="cover_plate",
        elem_b="lens",
        min_overlap=0.150,
        name="closed cover spans the magnifier lens",
    )
    ctx.expect_gap(
        cover,
        head,
        axis="x",
        positive_elem="cover_plate",
        negative_elem="lens",
        min_gap=0.010,
        max_gap=0.022,
        name="closed cover sits just ahead of the lens",
    )

    elbow_rest = ctx.part_world_position(upper_arm)
    with ctx.pose({base_hinge: base_upper}):
        elbow_raised = ctx.part_world_position(upper_arm)
    ctx.check(
        "lower arm raises the elbow",
        elbow_rest is not None
        and elbow_raised is not None
        and elbow_raised[2] > elbow_rest[2] + 0.20,
        details=f"rest={elbow_rest}, raised={elbow_raised}",
    )

    head_rest = ctx.part_world_position(head)
    with ctx.pose({elbow_hinge: elbow_upper}):
        head_raised = ctx.part_world_position(head)
    ctx.check(
        "upper arm raises the head",
        head_rest is not None and head_raised is not None and head_raised[2] > head_rest[2] + 0.16,
        details=f"rest={head_rest}, raised={head_raised}",
    )

    lens_rest = ctx.part_element_world_aabb(head, elem="lens")
    with ctx.pose({head_hinge: head_upper}):
        lens_tilted = ctx.part_element_world_aabb(head, elem="lens")
    ctx.check(
        "head tilts upward",
        aabb_center_z(lens_rest) is not None
        and aabb_center_z(lens_tilted) is not None
        and aabb_center_z(lens_tilted) > aabb_center_z(lens_rest) + 0.030,
        details=f"rest={lens_rest}, tilted={lens_tilted}",
    )

    with ctx.pose({cover_hinge: math.radians(115.0)}):
        open_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
        head_shell_open = ctx.part_element_world_aabb(head, elem="head_shell")
    ctx.check(
        "cover flips above the head",
        open_cover_aabb is not None
        and head_shell_open is not None
        and open_cover_aabb[1][2] > head_shell_open[1][2] + 0.030,
        details=f"open_cover={open_cover_aabb}, head={head_shell_open}",
    )

    return ctx.report()


object_model = build_object_model()
