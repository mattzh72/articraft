from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


RING_CENTER_Z = 0.36
OUTER_RING_RADIUS = 0.118
OUTER_RING_TUBE = 0.018


def _torus_in_yz(radius: float, tube: float) -> object:
    """Torus whose central opening normal is the local X axis."""
    return TorusGeometry(
        radius,
        tube,
        radial_segments=48,
        tubular_segments=18,
    ).rotate_y(math.pi / 2.0)


def _torus_axis_y(radius: float, tube: float) -> object:
    """Bearing torus with its bore along local Y."""
    return TorusGeometry(
        radius,
        tube,
        radial_segments=36,
        tubular_segments=14,
    ).rotate_x(math.pi / 2.0)


def _inner_cradle_frame() -> object:
    """A thin rounded rectangular cradle frame, extruded in X."""
    outer = rounded_rect_profile(0.145, 0.120, 0.016, corner_segments=8)
    inner = rounded_rect_profile(0.095, 0.070, 0.010, corner_segments=8)
    return ExtrudeWithHolesGeometry(
        outer,
        [inner],
        0.018,
        cap=True,
        center=True,
    ).rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_pitch_roll_module")

    model.material("frame_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("outer_blue", rgba=(0.05, 0.20, 0.42, 1.0))
    model.material("cradle_silver", rgba=(0.72, 0.75, 0.76, 1.0))
    model.material("bearing_bronze", rgba=(0.76, 0.54, 0.25, 1.0))
    model.material("axis_black", rgba=(0.02, 0.02, 0.022, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        Box((0.30, 0.46, 0.035)),
        origin=Origin(xyz=(-0.055, 0.0, 0.0175)),
        material="frame_dark",
        name="base_plate",
    )
    for i, y in enumerate((-0.185, 0.185)):
        rear_frame.visual(
            Box((0.045, 0.035, 0.445)),
            origin=Origin(xyz=(-0.130, y, 0.2375)),
            material="frame_dark",
            name=f"rear_post_{i}",
        )
    rear_frame.visual(
        Box((0.045, 0.405, 0.045)),
        origin=Origin(xyz=(-0.130, 0.0, 0.465)),
        material="frame_dark",
        name="top_bridge",
    )
    for i, y in enumerate((-0.168, 0.168)):
        rear_frame.visual(
            Box((0.130, 0.032, 0.032)),
            origin=Origin(xyz=(-0.085, y, RING_CENTER_Z)),
            material="frame_dark",
            name=f"bearing_arm_{i}",
        )
        rear_frame.visual(
            mesh_from_geometry(_torus_axis_y(0.0155, 0.0055), f"pitch_bearing_{i}"),
            origin=Origin(xyz=(0.0, y, RING_CENTER_Z)),
            material="bearing_bronze",
            name=f"pitch_bearing_{i}",
        )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_geometry(_torus_in_yz(OUTER_RING_RADIUS, OUTER_RING_TUBE), "outer_ring_torus"),
        material="outer_blue",
        name="ring_torus",
    )
    for i, y in enumerate((-0.158, 0.158)):
        outer_ring.visual(
            Cylinder(radius=0.010, length=0.055),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="axis_black",
            name=f"pitch_trunnion_{i}",
        )
    for i, y in enumerate((-0.126, 0.126)):
        outer_ring.visual(
            Cylinder(radius=0.024, length=0.016),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="outer_blue",
            name=f"side_hub_{i}",
        )
    for i, x in enumerate((-0.014, 0.014)):
        outer_ring.visual(
            mesh_from_geometry(_torus_in_yz(0.0105, 0.0048), f"roll_bearing_{i}"),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material="bearing_bronze",
            name=f"roll_bearing_{i}",
        )
        for j, z in enumerate((-0.0625, 0.0625)):
            outer_ring.visual(
                Box((0.008, 0.008, 0.096)),
                origin=Origin(xyz=(x, 0.0, z)),
                material="outer_blue",
                name=f"roll_spoke_{i}_{j}",
            )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        mesh_from_geometry(_inner_cradle_frame(), "inner_cradle_frame"),
        material="cradle_silver",
        name="cradle_frame",
    )
    inner_cradle.visual(
        Cylinder(radius=0.006, length=0.080),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="axis_black",
        name="roll_axle",
    )
    inner_cradle.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="cradle_silver",
        name="central_hub",
    )
    inner_cradle.visual(
        Box((0.018, 0.090, 0.012)),
        origin=Origin(),
        material="cradle_silver",
        name="hub_spoke",
    )

    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, RING_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=inner_cradle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.70, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rear_frame = object_model.get_part("rear_frame")
    outer_ring = object_model.get_part("outer_ring")
    inner_cradle = object_model.get_part("inner_cradle")
    pitch_axis = object_model.get_articulation("pitch_axis")
    roll_axis = object_model.get_articulation("roll_axis")

    def _aabb_extent(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return float(aabb[1][axis_index] - aabb[0][axis_index])

    pitch_dot_roll = sum(a * b for a, b in zip(pitch_axis.axis, roll_axis.axis))
    ctx.check(
        "pitch and roll axes are perpendicular",
        abs(pitch_dot_roll) < 1.0e-6,
        details=f"dot={pitch_dot_roll}",
    )

    for i in range(2):
        ctx.allow_overlap(
            outer_ring,
            rear_frame,
            elem_a=f"pitch_trunnion_{i}",
            elem_b=f"pitch_bearing_{i}",
            reason=(
                "The outer ring trunnion is intentionally captured in the "
                "rear-frame bearing bore so the pitch axis reads supported."
            ),
        )
        ctx.expect_within(
            outer_ring,
            rear_frame,
            axes="xz",
            inner_elem=f"pitch_trunnion_{i}",
            outer_elem=f"pitch_bearing_{i}",
            margin=0.001,
            name=f"pitch trunnion {i} is centered in its rear bearing",
        )
        ctx.expect_overlap(
            outer_ring,
            rear_frame,
            axes="y",
            elem_a=f"pitch_trunnion_{i}",
            elem_b=f"pitch_bearing_{i}",
            min_overlap=0.006,
            name=f"pitch trunnion {i} remains inserted in bearing",
        )

    for i in range(2):
        ctx.allow_overlap(
            inner_cradle,
            outer_ring,
            elem_a="roll_axle",
            elem_b=f"roll_bearing_{i}",
            reason=(
                "The smaller cradle axle is intentionally captured in the "
                "outer-ring bearing bore for a visibly supported roll axis."
            ),
        )
        ctx.expect_within(
            inner_cradle,
            outer_ring,
            axes="yz",
            inner_elem="roll_axle",
            outer_elem=f"roll_bearing_{i}",
            margin=0.001,
            name=f"roll axle is centered in bearing {i}",
        )
        ctx.expect_overlap(
            inner_cradle,
            outer_ring,
            axes="x",
            elem_a="roll_axle",
            elem_b=f"roll_bearing_{i}",
            min_overlap=0.004,
            name=f"roll axle remains inserted in bearing {i}",
        )

    ctx.expect_within(
        inner_cradle,
        outer_ring,
        axes="yz",
        inner_elem="cradle_frame",
        outer_elem="ring_torus",
        margin=0.0,
        name="inner cradle frame is nested inside the outer ring envelope",
    )

    rest_outer_aabb = ctx.part_world_aabb(outer_ring)
    rest_outer_x = _aabb_extent(rest_outer_aabb, 0)
    with ctx.pose({pitch_axis: 0.50}):
        pitched_outer_aabb = ctx.part_world_aabb(outer_ring)
    pitched_outer_x = _aabb_extent(pitched_outer_aabb, 0)
    ctx.check(
        "outer ring visibly pitches about the rear-supported axis",
        rest_outer_x is not None
        and pitched_outer_x is not None
        and pitched_outer_x > rest_outer_x + 0.045,
        details=f"rest_x={rest_outer_x}, pitched_x={pitched_outer_x}",
    )

    rest_inner_aabb = ctx.part_world_aabb(inner_cradle)
    rest_inner_y = _aabb_extent(rest_inner_aabb, 1)
    with ctx.pose({roll_axis: 0.60}):
        rolled_inner_aabb = ctx.part_world_aabb(inner_cradle)
    rolled_inner_y = _aabb_extent(rolled_inner_aabb, 1)
    ctx.check(
        "inner cradle visibly rolls on the perpendicular axis",
        rest_inner_y is not None
        and rolled_inner_y is not None
        and rolled_inner_y > rest_inner_y + 0.035,
        details=f"rest_y={rest_inner_y}, rolled_y={rolled_inner_y}",
    )

    return ctx.report()


object_model = build_object_model()
