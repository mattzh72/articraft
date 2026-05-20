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


ENVELOPE_LENGTH = 18.0
ENVELOPE_RADIUS = 2.2
GONDOLA_Z = -2.95


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = vector
    length = math.sqrt((x * x) + (y * y) + (z * z))
    return (x / length, y / length, z / length)


def _axis_to_rpy(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    ax, ay, az = _normalize(axis)
    yaw = math.atan2(ay, ax)
    pitch = math.atan2(math.sqrt((ax * ax) + (ay * ay)), az)
    return (0.0, pitch, yaw)


def _circle_loft_x(stations: list[tuple[float, float]]):
    loft = cq.Workplane("YZ", origin=(stations[0][0], 0.0, 0.0)).circle(stations[0][1])
    previous_x = stations[0][0]
    for x_pos, radius in stations[1:]:
        loft = loft.workplane(offset=x_pos - previous_x).circle(radius)
        previous_x = x_pos
    return loft.loft(combine=True, ruled=False)


def _build_envelope_shape():
    return _circle_loft_x(
        [
            (-9.00, 0.10),
            (-8.10, 0.62),
            (-6.60, 1.48),
            (-3.20, 2.05),
            (0.00, 2.20),
            (3.00, 2.08),
            (5.90, 1.68),
            (7.80, 0.96),
            (9.00, 0.16),
        ]
    )


def _build_nacelle_shape():
    return _circle_loft_x(
        [
            (-0.72, 0.10),
            (-0.50, 0.22),
            (-0.10, 0.30),
            (0.28, 0.31),
            (0.56, 0.24),
            (0.72, 0.12),
        ]
    )


def _build_rudder_surface_shape():
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, 0.0),
                (-0.16, 0.0),
                (-0.56, 0.26),
                (-0.49, 1.08),
                (-0.22, 1.36),
                (0.0, 1.36),
            ]
        )
        .close()
        .extrude(0.05, both=True)
    )


def _build_elevator_surface_shape():
    return (
        cq.Workplane("XY")
        .polyline(
            [
                (0.0, -0.63),
                (-0.16, -0.63),
                (-0.62, -0.22),
                (-0.58, 0.22),
                (-0.18, 0.63),
                (0.0, 0.63),
            ]
        )
        .close()
        .extrude(0.06, both=True)
    )


def _add_rod(
    part,
    *,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
):
    axis = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    mid = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    length = math.sqrt((axis[0] * axis[0]) + (axis[1] * axis[1]) + (axis[2] * axis[2]))
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=_axis_to_rpy(axis)),
        material=material,
        name=name,
    )


def _rod_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> Origin:
    axis = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    mid = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    return Origin(xyz=mid, rpy=_axis_to_rpy(axis))


def _rod_length(start: tuple[float, float, float], end: tuple[float, float, float]) -> float:
    axis = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    return math.sqrt((axis[0] * axis[0]) + (axis[1] * axis[1]) + (axis[2] * axis[2]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="advertising_blimp")

    envelope_paint = model.material("envelope_paint", rgba=(0.95, 0.93, 0.86, 1.0))
    gondola_paint = model.material("gondola_paint", rgba=(0.82, 0.83, 0.86, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.86, 0.87, 0.90, 1.0))
    dark_window = model.material("dark_window", rgba=(0.15, 0.19, 0.24, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    prop_paint = model.material("prop_paint", rgba=(0.16, 0.17, 0.18, 1.0))

    envelope = model.part("envelope")
    envelope.visual(
        mesh_from_cadquery(_build_envelope_shape(), "envelope_shell"),
        material=envelope_paint,
        name="envelope_shell",
    )
    envelope.visual(
        Box((1.15, 0.12, 1.55)),
        origin=Origin(xyz=(-7.54, 0.0, 1.42)),
        material=envelope_paint,
        name="upper_fin",
    )
    envelope.visual(
        Box((0.96, 0.10, 1.10)),
        origin=Origin(xyz=(-7.80, 0.0, -1.12)),
        material=envelope_paint,
        name="lower_fin",
    )
    envelope.visual(
        Box((0.18, 0.14, 2.55)),
        origin=Origin(xyz=(-8.18, 0.0, 0.12)),
        material=envelope_paint,
        name="fin_post",
    )
    envelope.visual(
        Box((1.08, 1.68, 0.10)),
        origin=Origin(xyz=(-7.62, 1.18, 0.04)),
        material=envelope_paint,
        name="stabilizer_0",
    )
    envelope.visual(
        Box((1.08, 1.68, 0.10)),
        origin=Origin(xyz=(-7.62, -1.18, 0.04)),
        material=envelope_paint,
        name="stabilizer_1",
    )
    envelope.visual(
        Cylinder(radius=0.035, length=1.32),
        origin=Origin(xyz=(-8.145, 0.0, 1.38)),
        material=dark_metal,
        name="rudder_hinge",
    )
    envelope.visual(
        Cylinder(radius=0.030, length=1.30),
        origin=Origin(xyz=(-8.145, 1.18, 0.04), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="elevator_hinge_0",
    )
    envelope.visual(
        Cylinder(radius=0.030, length=1.30),
        origin=Origin(xyz=(-8.145, -1.18, 0.04), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="elevator_hinge_1",
    )
    envelope.visual(
        Box((0.22, 0.18, 0.22)),
        origin=Origin(xyz=(1.06, 0.86, -2.03)),
        material=dark_metal,
        name="gondola_pad_front_0",
    )
    envelope.visual(
        Box((0.22, 0.18, 0.22)),
        origin=Origin(xyz=(1.06, -0.86, -2.03)),
        material=dark_metal,
        name="gondola_pad_front_1",
    )
    envelope.visual(
        Box((0.24, 0.18, 0.22)),
        origin=Origin(xyz=(-0.80, 0.70, -2.03)),
        material=dark_metal,
        name="gondola_pad_rear_0",
    )
    envelope.visual(
        Box((0.24, 0.18, 0.22)),
        origin=Origin(xyz=(-0.80, -0.70, -2.03)),
        material=dark_metal,
        name="gondola_pad_rear_1",
    )
    envelope.visual(
        Box((0.18, 0.16, 0.12)),
        origin=Origin(xyz=(-0.31, 2.10, -0.48)),
        material=dark_metal,
        name="nacelle_pad_front_0",
    )
    envelope.visual(
        Box((0.18, 0.16, 0.12)),
        origin=Origin(xyz=(-0.31, -2.10, -0.48)),
        material=dark_metal,
        name="nacelle_pad_front_1",
    )
    envelope.visual(
        Box((0.18, 0.16, 0.12)),
        origin=Origin(xyz=(-0.71, 2.10, -0.54)),
        material=dark_metal,
        name="nacelle_pad_rear_0",
    )
    envelope.visual(
        Box((0.18, 0.16, 0.12)),
        origin=Origin(xyz=(-0.71, -2.10, -0.54)),
        material=dark_metal,
        name="nacelle_pad_rear_1",
    )

    gondola = model.part("gondola")
    gondola.visual(
        Box((2.95, 0.95, 0.88)),
        origin=Origin(xyz=(0.15, 0.0, GONDOLA_Z)),
        material=gondola_paint,
        name="cabin_shell",
    )
    gondola.visual(
        Box((0.80, 0.90, 0.34)),
        origin=Origin(xyz=(1.20, 0.0, GONDOLA_Z + 0.14)),
        material=dark_window,
        name="windshield_band",
    )
    for name, size, xyz in (
        ("wheel_post", (0.12, 0.08, 0.40), (-1.07, 0.0, -3.34)),
        ("wheel_bridge", (0.34, 0.20, 0.10), (-1.24, 0.0, -3.26)),
        ("fork_arm_0", (0.22, 0.03, 0.34), (-1.34, 0.065, -3.43)),
        ("fork_arm_1", (0.22, 0.03, 0.34), (-1.34, -0.065, -3.43)),
    ):
        gondola.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=dark_metal,
            name=name,
        )
    gondola.visual(
        Cylinder(radius=0.016, length=0.16),
        origin=Origin(xyz=(-1.34, 0.0, -3.58), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="wheel_axle",
    )
    front_strut_0_start = (0.95, 0.26, GONDOLA_Z + 0.46)
    front_strut_0_end = (1.06, 0.86, -2.14)
    gondola.visual(
        Cylinder(radius=0.055, length=_rod_length(front_strut_0_start, front_strut_0_end)),
        origin=_rod_origin(front_strut_0_start, front_strut_0_end),
        material=dark_metal,
        name="front_strut_0",
    )
    front_strut_1_start = (0.95, -0.26, GONDOLA_Z + 0.46)
    front_strut_1_end = (1.06, -0.86, -2.14)
    gondola.visual(
        Cylinder(radius=0.055, length=_rod_length(front_strut_1_start, front_strut_1_end)),
        origin=_rod_origin(front_strut_1_start, front_strut_1_end),
        material=dark_metal,
        name="front_strut_1",
    )
    rear_strut_0_start = (-0.60, 0.24, GONDOLA_Z + 0.46)
    rear_strut_0_end = (-0.80, 0.70, -2.14)
    gondola.visual(
        Cylinder(radius=0.055, length=_rod_length(rear_strut_0_start, rear_strut_0_end)),
        origin=_rod_origin(rear_strut_0_start, rear_strut_0_end),
        material=dark_metal,
        name="rear_strut_0",
    )
    rear_strut_1_start = (-0.60, -0.24, GONDOLA_Z + 0.46)
    rear_strut_1_end = (-0.80, -0.70, -2.14)
    gondola.visual(
        Cylinder(radius=0.055, length=_rod_length(rear_strut_1_start, rear_strut_1_end)),
        origin=_rod_origin(rear_strut_1_start, rear_strut_1_end),
        material=dark_metal,
        name="rear_strut_1",
    )

    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        parent=envelope,
        child=gondola,
        origin=Origin(),
    )

    nacelle_centers = {
        "nacelle_0": (-0.45, 3.08, -1.05),
        "nacelle_1": (-0.45, -3.08, -1.05),
    }

    for index, (nacelle_name, center) in enumerate(nacelle_centers.items()):
        x_center, y_center, z_center = center
        nacelle = model.part(nacelle_name)
        nacelle.visual(
            mesh_from_cadquery(_build_nacelle_shape(), f"{nacelle_name}_shell"),
            origin=Origin(xyz=center),
            material=nacelle_paint,
            name="nacelle_shell",
        )
        nacelle.visual(
            Cylinder(radius=0.055, length=0.14),
            origin=Origin(xyz=(x_center + 0.67, y_center, z_center), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name="prop_shaft",
        )
        mount_y = 2.18 if y_center > 0.0 else -2.18
        mount_front_start = (x_center + 0.22, y_center, z_center + 0.26)
        mount_front_end = (x_center + 0.14, mount_y, -0.48)
        nacelle.visual(
            Cylinder(radius=0.06, length=_rod_length(mount_front_start, mount_front_end)),
            origin=_rod_origin(mount_front_start, mount_front_end),
            material=dark_metal,
            name="mount_front",
        )
        mount_rear_start = (x_center - 0.18, y_center, z_center + 0.24)
        mount_rear_end = (x_center - 0.26, mount_y, -0.54)
        nacelle.visual(
            Cylinder(radius=0.06, length=_rod_length(mount_rear_start, mount_rear_end)),
            origin=_rod_origin(mount_rear_start, mount_rear_end),
            material=dark_metal,
            name="mount_rear",
        )

        propeller = model.part(f"propeller_{index}")
        propeller.visual(
            Cylinder(radius=0.11, length=0.12),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name="hub",
        )
        propeller.visual(
            Box((0.04, 1.22, 0.14)),
            origin=Origin(rpy=(0.0, math.radians(12.0), 0.0)),
            material=prop_paint,
            name="blade_pair",
        )

        model.articulation(
            f"envelope_to_{nacelle_name}",
            ArticulationType.FIXED,
            parent=envelope,
            child=nacelle,
            origin=Origin(),
        )
        model.articulation(
            f"{nacelle_name}_to_propeller_{index}",
            ArticulationType.CONTINUOUS,
            parent=nacelle,
            child=propeller,
            origin=Origin(xyz=(x_center + 0.80, y_center, z_center)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=35.0),
        )

    rudder = model.part("rudder")
    rudder.visual(
        Cylinder(radius=0.020, length=1.28),
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        material=dark_metal,
        name="rudder_barrel",
    )
    rudder.visual(
        mesh_from_cadquery(_build_rudder_surface_shape(), "rudder_surface"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=envelope_paint,
        name="rudder_surface",
    )

    for index, y_pos in enumerate((1.18, -1.18)):
        elevator = model.part(f"elevator_{index}")
        elevator.visual(
            Cylinder(radius=0.020, length=1.24),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hinge_barrel",
        )
        elevator.visual(
            mesh_from_cadquery(_build_elevator_surface_shape(), f"elevator_surface_{index}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=envelope_paint,
            name="elevator_surface",
        )
        model.articulation(
            f"envelope_to_elevator_{index}",
            ArticulationType.REVOLUTE,
            parent=envelope,
            child=elevator,
            origin=Origin(xyz=(-8.18, y_pos, 0.04)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.45, upper=0.45),
        )

    mooring_wheel = model.part("mooring_wheel")
    mooring_wheel.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="wheel_tire",
    )
    mooring_wheel.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="wheel_hub",
    )

    model.articulation(
        "envelope_to_rudder",
        ArticulationType.REVOLUTE,
        parent=envelope,
        child=rudder,
        origin=Origin(xyz=(-8.18, 0.0, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "gondola_to_mooring_wheel",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=mooring_wheel,
        origin=Origin(xyz=(-1.34, 0.0, -3.58)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    envelope = object_model.get_part("envelope")
    gondola = object_model.get_part("gondola")
    nacelle_0 = object_model.get_part("nacelle_0")
    nacelle_1 = object_model.get_part("nacelle_1")
    rudder = object_model.get_part("rudder")
    elevator_0 = object_model.get_part("elevator_0")

    ctx.expect_gap(
        envelope,
        gondola,
        axis="z",
        positive_elem="envelope_shell",
        negative_elem="cabin_shell",
        min_gap=0.18,
        max_gap=0.60,
        name="gondola cabin hangs below the envelope",
    )
    ctx.expect_gap(
        nacelle_0,
        envelope,
        axis="y",
        positive_elem="nacelle_shell",
        negative_elem="envelope_shell",
        min_gap=0.18,
        name="starboard nacelle stays outboard of the envelope skin",
    )
    ctx.expect_gap(
        envelope,
        nacelle_1,
        axis="y",
        positive_elem="envelope_shell",
        negative_elem="nacelle_shell",
        min_gap=0.18,
        name="port nacelle stays outboard of the envelope skin",
    )

    ctx.allow_overlap(
        envelope,
        gondola,
        elem_a="gondola_pad_front_0",
        elem_b="front_strut_0",
        reason="The forward gondola strut seats into a simplified hardpoint pad on the envelope.",
    )
    ctx.allow_overlap(
        envelope,
        gondola,
        elem_a="gondola_pad_front_1",
        elem_b="front_strut_1",
        reason="The mirrored forward gondola strut seats into a simplified hardpoint pad on the envelope.",
    )
    ctx.allow_overlap(
        envelope,
        nacelle_0,
        elem_a="nacelle_pad_front_0",
        elem_b="mount_front",
        reason="The nacelle pylon nests slightly into a simplified envelope-side hardpoint pad.",
    )
    ctx.allow_overlap(
        envelope,
        nacelle_1,
        elem_a="nacelle_pad_front_1",
        elem_b="mount_front",
        reason="The nacelle pylon nests slightly into a simplified envelope-side hardpoint pad.",
    )
    ctx.allow_overlap(
        envelope,
        gondola,
        elem_a="gondola_pad_rear_0",
        elem_b="rear_strut_0",
        reason="The aft gondola strut seats into a simplified hardpoint pad on the envelope.",
    )
    ctx.allow_overlap(
        envelope,
        gondola,
        elem_a="gondola_pad_rear_1",
        elem_b="rear_strut_1",
        reason="The mirrored aft gondola strut seats into a simplified hardpoint pad on the envelope.",
    )
    ctx.allow_overlap(
        envelope,
        nacelle_0,
        elem_a="nacelle_pad_rear_0",
        elem_b="mount_rear",
        reason="The rear nacelle pylon nests slightly into a simplified envelope-side hardpoint pad.",
    )
    ctx.allow_overlap(
        envelope,
        nacelle_1,
        elem_a="nacelle_pad_rear_1",
        elem_b="mount_rear",
        reason="The rear nacelle pylon nests slightly into a simplified envelope-side hardpoint pad.",
    )
    ctx.allow_overlap(
        envelope,
        rudder,
        elem_a="rudder_hinge",
        elem_b="rudder_barrel",
        reason="The rudder barrel intentionally nests into the fixed hinge sleeve.",
    )
    ctx.allow_overlap(
        envelope,
        rudder,
        elem_a="upper_fin",
        elem_b="rudder_barrel",
        reason="The rudder barrel passes through the upper tail fin root as part of the hinge structure.",
    )
    ctx.allow_overlap(
        envelope,
        rudder,
        elem_a="fin_post",
        elem_b="rudder_surface",
        reason="The rudder root is simplified as partially embedded within the fixed tail post at the hinge line.",
    )
    ctx.allow_overlap(
        envelope,
        rudder,
        elem_a="fin_post",
        elem_b="rudder_barrel",
        reason="The rudder hinge barrel runs through the tail post as the simplified hinge spine.",
    )
    ctx.allow_overlap(
        envelope,
        "elevator_0",
        elem_a="elevator_hinge_0",
        elem_b="hinge_barrel",
        reason="The elevator hinge barrel intentionally nests into the fixed hinge sleeve.",
    )
    ctx.allow_overlap(
        envelope,
        "elevator_1",
        elem_a="elevator_hinge_1",
        elem_b="hinge_barrel",
        reason="The mirrored elevator hinge barrel intentionally nests into the fixed hinge sleeve.",
    )
    ctx.allow_overlap(
        gondola,
        "mooring_wheel",
        elem_a="wheel_axle",
        elem_b="wheel_hub",
        reason="The mooring wheel hub intentionally rotates around the fixed axle.",
    )
    ctx.allow_overlap(
        gondola,
        "mooring_wheel",
        elem_a="wheel_axle",
        elem_b="wheel_tire",
        reason="The mooring wheel is simplified as a solid tire body around the axle, so the axle intentionally passes through it.",
    )

    for joint_name in (
        "nacelle_0_to_propeller_0",
        "nacelle_1_to_propeller_1",
        "gondola_to_mooring_wheel",
    ):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    rudder_joint = object_model.get_articulation("envelope_to_rudder")
    rudder_limits = rudder_joint.motion_limits
    if rudder_limits is not None and rudder_limits.upper is not None:
        rest_box = ctx.part_element_world_aabb(rudder, elem="rudder_surface")
        with ctx.pose({rudder_joint: rudder_limits.upper}):
            deflected_box = ctx.part_element_world_aabb(rudder, elem="rudder_surface")
        rest_center_y = None if rest_box is None else 0.5 * (rest_box[0][1] + rest_box[1][1])
        deflected_center_y = None if deflected_box is None else 0.5 * (deflected_box[0][1] + deflected_box[1][1])
        ctx.check(
            "rudder swings laterally at positive deflection",
            rest_center_y is not None
            and deflected_center_y is not None
            and deflected_center_y < rest_center_y - 0.12,
            details=f"rest_center_y={rest_center_y}, deflected_center_y={deflected_center_y}",
        )

    elevator_joint = object_model.get_articulation("envelope_to_elevator_0")
    elevator_limits = elevator_joint.motion_limits
    if elevator_limits is not None and elevator_limits.upper is not None:
        rest_box = ctx.part_element_world_aabb(elevator_0, elem="elevator_surface")
        with ctx.pose({elevator_joint: elevator_limits.upper}):
            pitched_box = ctx.part_element_world_aabb(elevator_0, elem="elevator_surface")
        rest_center_z = None if rest_box is None else 0.5 * (rest_box[0][2] + rest_box[1][2])
        pitched_center_z = None if pitched_box is None else 0.5 * (pitched_box[0][2] + pitched_box[1][2])
        ctx.check(
            "elevator trailing edge rises at positive deflection",
            rest_center_z is not None
            and pitched_center_z is not None
            and pitched_center_z > rest_center_z + 0.06,
            details=f"rest_center_z={rest_center_z}, pitched_center_z={pitched_center_z}",
        )

    return ctx.report()


object_model = build_object_model()
