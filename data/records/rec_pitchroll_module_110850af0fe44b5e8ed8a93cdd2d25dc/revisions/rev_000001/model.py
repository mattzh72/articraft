from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_instrument_head")

    powder_black = model.material("powder_black", rgba=(0.05, 0.055, 0.060, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.15, 0.16, 0.17, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.45, 0.47, 0.49, 1.0))
    face_glass = model.material("face_glass", rgba=(0.08, 0.13, 0.16, 0.55))
    bolt_finish = model.material("black_oxide", rgba=(0.03, 0.03, 0.032, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.105, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=powder_black,
        name="round_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.072, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_graphite,
        name="base_step",
    )
    pedestal.visual(
        Cylinder(radius=0.026, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=powder_black,
        name="column",
    )
    pedestal.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=dark_graphite,
        name="top_collar",
    )
    pedestal.visual(
        Box((0.260, 0.048, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=dark_graphite,
        name="yoke_bridge",
    )
    for side, x, bearing_name in (
        ("left", -0.142, "left_bearing_cap"),
        ("right", 0.142, "right_bearing_cap"),
    ):
        pedestal.visual(
            Box((0.030, 0.064, 0.240)),
            origin=Origin(xyz=(x, 0.0, 0.520)),
            material=powder_black,
            name=f"{side}_cheek",
        )
        cap_x = -0.1205 if side == "left" else 0.1205
        pedestal.visual(
            Cylinder(radius=0.028, length=0.014),
            origin=Origin(xyz=(cap_x, 0.0, 0.520), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=bearing_name,
        )
        bolt_x = -0.1105 if side == "left" else 0.1105
        pedestal.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(bolt_x, 0.020, 0.545), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_finish,
            name=f"{side}_cap_bolt_0",
        )
        pedestal.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(bolt_x, -0.020, 0.495), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_finish,
            name=f"{side}_cap_bolt_1",
        )

    pedestal.inertial = Inertial.from_geometry(
        Box((0.320, 0.220, 0.540)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
    )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_geometry(TorusGeometry(radius=0.100, tube=0.0085, radial_segments=20, tubular_segments=72), "outer_roll_ring"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="roll_ring",
    )
    outer_ring.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.0985, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="roll_shaft",
    )
    outer_ring.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(-0.0985, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="roll_shaft_0",
    )
    for side_index, sx in enumerate((-1.0, 1.0)):
        for strut_index, (y, z) in enumerate(((0.070, 0.070), (-0.070, 0.070), (0.070, -0.070), (-0.070, -0.070))):
            _add_member(
                outer_ring,
                (sx * 0.113, 0.0, 0.0),
                (0.0, y, z),
                0.005,
                satin_aluminum,
                name=f"roll_strut_{side_index}_{strut_index}",
            )
    for index, y, bearing_name in (
        (0, -0.100, "pitch_bearing_0"),
        (1, 0.100, "pitch_bearing_1"),
    ):
        outer_ring.visual(
            Cylinder(radius=0.020, length=0.020),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=bearing_name,
        )

    outer_ring.inertial = Inertial.from_geometry(
        Box((0.260, 0.260, 0.260)),
        mass=0.85,
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        Cylinder(radius=0.010, length=0.180),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pitch_shaft",
    )
    inner_cradle.visual(
        Cylinder(radius=0.052, length=0.026),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="center_flange",
    )
    inner_cradle.visual(
        Cylinder(radius=0.042, length=0.007),
        origin=Origin(xyz=(0.0365, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=face_glass,
        name="instrument_face",
    )
    inner_cradle.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="flange_boss",
    )
    inner_cradle.visual(
        Box((0.035, 0.050, 0.016)),
        origin=Origin(xyz=(0.015, 0.0, 0.045)),
        material=dark_graphite,
        name="upper_cradle_lip",
    )
    inner_cradle.visual(
        Box((0.035, 0.050, 0.016)),
        origin=Origin(xyz=(0.015, 0.0, -0.045)),
        material=dark_graphite,
        name="lower_cradle_lip",
    )
    for index, (y, z) in enumerate(((0.028, 0.028), (-0.028, 0.028), (-0.028, -0.028), (0.028, -0.028))):
        inner_cradle.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(0.040, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_finish,
            name=f"flange_bolt_{index}",
        )

    inner_cradle.inertial = Inertial.from_geometry(
        Box((0.080, 0.200, 0.120)),
        mass=0.55,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_outer_ring",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "outer_ring_to_inner_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=inner_cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.6, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    outer_ring = object_model.get_part("outer_ring")
    inner_cradle = object_model.get_part("inner_cradle")
    roll = object_model.get_articulation("pedestal_to_outer_ring")
    pitch = object_model.get_articulation("outer_ring_to_inner_cradle")

    ctx.check("two_revolute_axes_present", roll is not None and pitch is not None)
    if roll is not None and pitch is not None:
        dot = sum(float(roll.axis[i]) * float(pitch.axis[i]) for i in range(3))
        ctx.check("roll_pitch_axes_perpendicular", abs(dot) < 1e-6, details=f"dot={dot}")

    ctx.expect_contact(
        outer_ring,
        pedestal,
        elem_a="roll_shaft",
        elem_b="right_bearing_cap",
        contact_tol=0.0015,
        name="roll shaft seats in bearing cap",
    )
    ctx.expect_contact(
        inner_cradle,
        outer_ring,
        elem_a="pitch_shaft",
        elem_b="pitch_bearing_1",
        contact_tol=0.0015,
        name="pitch shaft seats in outer bearing",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    rest_face = _aabb_center(ctx.part_element_world_aabb(inner_cradle, elem="instrument_face"))
    with ctx.pose({pitch: -0.35}):
        pitched_face = _aabb_center(ctx.part_element_world_aabb(inner_cradle, elem="instrument_face"))
    ctx.check(
        "pitch tilts center flange",
        rest_face is not None and pitched_face is not None and pitched_face[2] > rest_face[2] + 0.008,
        details=f"rest={rest_face}, pitched={pitched_face}",
    )

    rest_lip = _aabb_center(ctx.part_element_world_aabb(inner_cradle, elem="upper_cradle_lip"))
    with ctx.pose({roll: 0.45}):
        rolled_lip = _aabb_center(ctx.part_element_world_aabb(inner_cradle, elem="upper_cradle_lip"))
    ctx.check(
        "roll swings cradle lip",
        rest_lip is not None and rolled_lip is not None and abs(rolled_lip[1] - rest_lip[1]) > 0.015,
        details=f"rest={rest_lip}, rolled={rolled_lip}",
    )

    return ctx.report()


object_model = build_object_model()
