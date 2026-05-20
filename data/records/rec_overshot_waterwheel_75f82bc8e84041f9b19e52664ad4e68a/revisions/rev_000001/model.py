from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel_sluice")

    timber = model.material("weathered_timber", rgba=(0.55, 0.34, 0.16, 1.0))
    dark_timber = model.material("dark_wet_timber", rgba=(0.32, 0.19, 0.09, 1.0))
    iron = model.material("blackened_iron", rgba=(0.06, 0.055, 0.05, 1.0))
    water = model.material("shallow_water", rgba=(0.18, 0.45, 0.80, 0.55))

    frame = model.part("timber_frame")

    # Ground sill and two side support towers that hold the wheel axle.
    frame.visual(
        Box((2.45, 1.05, 0.08)),
        origin=Origin(xyz=(-0.37, 0.0, 0.04)),
        material=timber,
        name="ground_sill",
    )
    for y, suffix, bearing_name in (
        (-0.44, "0", "iron_bearing_0"),
        (0.44, "1", "iron_bearing_1"),
    ):
        frame.visual(
            Box((0.12, 0.12, 1.08)),
            origin=Origin(xyz=(0.0, y, 0.60)),
            material=timber,
            name=f"side_post_{suffix}",
        )
        frame.visual(
            Box((0.24, 0.14, 0.20)),
            origin=Origin(xyz=(0.0, y, 1.05)),
            material=dark_timber,
            name=f"bearing_block_{suffix}",
        )
        frame.visual(
            Cylinder(radius=0.075, length=0.026),
            origin=Origin(xyz=(0.0, y * 0.86, 1.05), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=bearing_name,
        )

    # A high timber feed chute slopes down toward the top of the wheel.
    chute_pitch = math.atan2(0.30, 1.20)
    chute_center = (-0.85, 0.0, 1.97)
    frame.visual(
        Box((1.24, 0.54, 0.055)),
        origin=Origin(xyz=chute_center, rpy=(0.0, chute_pitch, 0.0)),
        material=dark_timber,
        name="chute_floor",
    )
    for y in (-0.305, 0.305):
        frame.visual(
            Box((1.24, 0.055, 0.24)),
            origin=Origin(xyz=(-0.85, y, 2.055), rpy=(0.0, chute_pitch, 0.0)),
            material=timber,
            name=f"chute_wall_{0 if y < 0 else 1}",
        )
    frame.visual(
        Box((0.34, 0.50, 0.045)),
        origin=Origin(xyz=(-0.31, 0.0, 1.82), rpy=(0.0, chute_pitch, 0.0)),
        material=dark_timber,
        name="chute_lip",
    )
    frame.visual(
        Box((0.82, 0.46, 0.018)),
        origin=Origin(xyz=(-0.82, 0.0, 1.99), rpy=(0.0, chute_pitch, 0.0)),
        material=water,
        name="water_sheet",
    )

    # Posts below the chute keep it visibly supported and tie it into the sill.
    for x, zc, h in ((-1.36, 1.08, 2.08), (-0.56, 0.98, 1.88)):
        for y in (-0.34, 0.34):
            frame.visual(
                Box((0.09, 0.09, h)),
                origin=Origin(xyz=(x, y, zc)),
                material=timber,
                name=f"chute_post_{x}_{0 if y < 0 else 1}",
            )
    frame.visual(
        Box((0.78, 0.10, 0.10)),
        origin=Origin(xyz=(-0.96, -0.34, 1.90), rpy=(0.0, chute_pitch, 0.0)),
        material=timber,
        name="lower_side_ledger",
    )
    frame.visual(
        Box((0.78, 0.10, 0.10)),
        origin=Origin(xyz=(-0.96, 0.34, 1.90), rpy=(0.0, chute_pitch, 0.0)),
        material=timber,
        name="upper_side_ledger",
    )

    # Four close lips form vertical guide grooves for the sliding sluice gate.
    guide_z = 2.22
    guide_h = 1.06
    for x, y, guide_name in (
        (-0.605, -0.285, "guide_lip_-0.605_-0.285"),
        (-0.505, -0.285, "guide_lip_-0.505_-0.285"),
        (-0.605, 0.285, "guide_lip_-0.605_0.285"),
        (-0.505, 0.285, "guide_lip_-0.505_0.285"),
    ):
        frame.visual(
            Box((0.035, 0.070, guide_h)),
            origin=Origin(xyz=(x, y, guide_z)),
            material=dark_timber,
            name=guide_name,
        )
    frame.visual(
        Box((0.17, 0.68, 0.06)),
        origin=Origin(xyz=(-0.555, 0.0, 2.75)),
        material=timber,
        name="guide_top_tie",
    )
    frame.visual(
        Box((0.17, 0.68, 0.06)),
        origin=Origin(xyz=(-0.555, 0.0, 1.69)),
        material=timber,
        name="guide_bottom_tie",
    )

    wheel = model.part("bucket_wheel")
    rim_radius = 0.58
    rim_tube = 0.026
    for y, label in ((-0.18, "near"), (0.18, "far")):
        rim = TorusGeometry(rim_radius, rim_tube, radial_segments=20, tubular_segments=72)
        rim.rotate_x(math.pi / 2.0).translate(0.0, y, 0.0)
        wheel.visual(
            mesh_from_geometry(rim, f"{label}_wooden_rim"),
            material=timber,
            name=f"{label}_rim",
        )

    wheel.visual(
        Cylinder(radius=0.095, length=0.46),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_timber,
        name="central_hub",
    )
    wheel.visual(
        Cylinder(radius=0.034, length=0.731),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )

    spoke_len = 0.47
    spoke_mid = 0.33
    for i in range(10):
        angle = 2.0 * math.pi * i / 10.0
        wheel.visual(
            Box((spoke_len, 0.40, 0.045)),
            origin=Origin(
                xyz=(spoke_mid * math.cos(angle), 0.0, spoke_mid * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=timber,
            name=f"spoke_{i}",
        )

    # Overshot buckets: angled timber trough boards distributed around the rim.
    for i in range(16):
        angle = 2.0 * math.pi * i / 16.0
        radius = 0.635
        x = radius * math.cos(angle)
        z = radius * math.sin(angle)
        pitch = math.pi / 2.0 - angle
        wheel.visual(
            Box((0.18, 0.40, 0.075)),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, pitch, 0.0)),
            material=timber,
            name=f"bucket_floor_{i}",
        )
        lip_radius = 0.675
        wheel.visual(
            Box((0.035, 0.40, 0.13)),
            origin=Origin(
                xyz=(lip_radius * math.cos(angle + 0.11), 0.0, lip_radius * math.sin(angle + 0.11)),
                rpy=(0.0, pitch + 0.42, 0.0),
            ),
            material=dark_timber,
            name=f"bucket_lip_{i}",
        )

    gate = model.part("sluice_gate")
    gate.visual(
        Box((0.045, 0.50, 0.42)),
        origin=Origin(),
        material=dark_timber,
        name="gate_panel",
    )
    gate.visual(
        Box((0.065, 0.035, 0.42)),
        origin=Origin(xyz=(0.0, -0.257, 0.0)),
        material=timber,
        name="side_tongue_0",
    )
    gate.visual(
        Box((0.065, 0.035, 0.42)),
        origin=Origin(xyz=(0.0, 0.257, 0.0)),
        material=timber,
        name="side_tongue_1",
    )
    gate.visual(
        Cylinder(radius=0.035, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.242), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="lift_handle",
    )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=2.0),
    )
    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(-0.555, 0.0, 2.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.28, effort=120.0, velocity=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("timber_frame")
    wheel = object_model.get_part("bucket_wheel")
    gate = object_model.get_part("sluice_gate")
    wheel_joint = object_model.get_articulation("frame_to_wheel")
    gate_joint = object_model.get_articulation("frame_to_gate")

    ctx.check(
        "waterwheel has a continuous horizontal axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.check(
        "sluice gate slides vertically in its guides",
        gate_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in gate_joint.axis) == (0.0, 0.0, 1.0)
        and gate_joint.motion_limits is not None
        and gate_joint.motion_limits.upper >= 0.25,
        details=f"type={gate_joint.articulation_type}, axis={gate_joint.axis}, limits={gate_joint.motion_limits}",
    )

    ctx.expect_contact(
        wheel,
        frame,
        elem_a="axle",
        elem_b="iron_bearing_1",
        contact_tol=0.002,
        name="axle end is seated at the bearing",
    )

    def _gate_clipped_in_groove(check_name: str) -> None:
        tongue_aabb = ctx.part_element_world_aabb(gate, elem="side_tongue_0")
        front_aabb = ctx.part_element_world_aabb(frame, elem="guide_lip_-0.605_-0.285")
        rear_aabb = ctx.part_element_world_aabb(frame, elem="guide_lip_-0.505_-0.285")
        ok = False
        details = f"tongue={tongue_aabb}, front={front_aabb}, rear={rear_aabb}"
        if tongue_aabb and front_aabb and rear_aabb:
            tongue_min, tongue_max = tongue_aabb
            front_min, front_max = front_aabb
            rear_min, rear_max = rear_aabb
            ok = (
                abs(tongue_min[0] - front_max[0]) <= 0.003
                and abs(tongue_max[0] - rear_min[0]) <= 0.003
                and tongue_min[2] >= front_min[2] - 0.003
                and tongue_max[2] <= front_max[2] + 0.003
            )
        ctx.check(check_name, ok, details=details)

    with ctx.pose({gate_joint: 0.0}):
        ctx.expect_contact(
            gate,
            frame,
            elem_a="side_tongue_0",
            elem_b="guide_lip_-0.605_-0.285",
            contact_tol=0.003,
            name="lowered gate tongue touches guide lip",
        )
        ctx.expect_overlap(
            gate,
            frame,
            axes="z",
            elem_a="side_tongue_0",
            elem_b="guide_lip_-0.605_-0.285",
            min_overlap=0.38,
            name="lowered gate remains captured vertically",
        )
        _gate_clipped_in_groove("lowered gate is clipped between the guide lips")

    rest_gate_pos = ctx.part_world_position(gate)
    with ctx.pose({gate_joint: gate_joint.motion_limits.upper}):
        raised_gate_pos = ctx.part_world_position(gate)
        ctx.expect_contact(
            gate,
            frame,
            elem_a="side_tongue_0",
            elem_b="guide_lip_-0.605_-0.285",
            contact_tol=0.003,
            name="raised gate tongue stays in the guide lip",
        )
        ctx.expect_overlap(
            gate,
            frame,
            axes="z",
            elem_a="side_tongue_0",
            elem_b="guide_lip_-0.605_-0.285",
            min_overlap=0.30,
            name="raised gate remains captured vertically",
        )
        _gate_clipped_in_groove("raised gate is still clipped between the guide lips")

    ctx.check(
        "positive gate travel raises the sluice board",
        rest_gate_pos is not None
        and raised_gate_pos is not None
        and raised_gate_pos[2] > rest_gate_pos[2] + 0.25,
        details=f"rest={rest_gate_pos}, raised={raised_gate_pos}",
    )

    wheel_rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        wheel_rotated_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel rotation stays centered on the bearing axis",
        wheel_rest_pos is not None
        and wheel_rotated_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(wheel_rest_pos, wheel_rotated_pos)),
        details=f"rest={wheel_rest_pos}, rotated={wheel_rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
