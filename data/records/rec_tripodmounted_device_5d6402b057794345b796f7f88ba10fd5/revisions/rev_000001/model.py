from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LEG_PITCH = 0.95
LEG_BEAM_LEN = 1.08
LEG_BEAM_START = 0.040
LEG_FOOT_OVERLAP = 0.080


def _beam_center(start: float, length: float, pitch: float) -> tuple[float, float, float]:
    return (
        start + 0.5 * length * math.cos(pitch),
        0.0,
        -0.5 * length * math.sin(pitch),
    )


def _beam_end(start: float, length: float, pitch: float) -> tuple[float, float, float]:
    return (
        start + length * math.cos(pitch),
        0.0,
        -length * math.sin(pitch),
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_measurement_station")

    dark = model.material("dark", rgba=(0.20, 0.21, 0.23, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.29, 0.31, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    amber = model.material("amber", rgba=(0.90, 0.56, 0.16, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    collar_inner = 0.058
    collar_wall = 0.012
    collar_outer = collar_inner + 2.0 * collar_wall
    collar_height = 0.110
    collar_center_z = 0.030

    crown = model.part("crown")
    crown.visual(
        Box((collar_inner, collar_wall, collar_height)),
        origin=Origin(
            xyz=(0.0, collar_outer * 0.5 - collar_wall * 0.5, collar_center_z),
        ),
        material=graphite,
        name="collar_front",
    )
    crown.visual(
        Box((collar_inner, collar_wall, collar_height)),
        origin=Origin(
            xyz=(0.0, -collar_outer * 0.5 + collar_wall * 0.5, collar_center_z),
        ),
        material=graphite,
        name="collar_rear",
    )
    crown.visual(
        Box((collar_wall, collar_outer, collar_height)),
        origin=Origin(
            xyz=(collar_outer * 0.5 - collar_wall * 0.5, 0.0, collar_center_z),
        ),
        material=graphite,
        name="collar_right",
    )
    crown.visual(
        Box((collar_wall, collar_outer, collar_height)),
        origin=Origin(
            xyz=(-collar_outer * 0.5 + collar_wall * 0.5, 0.0, collar_center_z),
        ),
        material=graphite,
        name="collar_left",
    )

    arm_length = 0.070
    arm_width = 0.054
    arm_height = 0.040
    arm_center_z = -0.008
    lug_length = 0.020
    lug_width = 0.060
    lug_height = 0.044
    arm_center_x = collar_outer * 0.5 + arm_length * 0.5
    lug_center_x = collar_outer * 0.5 + arm_length + lug_length * 0.5
    hinge_radius = collar_outer * 0.5 + arm_length + lug_length

    for index in range(3):
        yaw = index * (2.0 * math.pi / 3.0)
        crown.visual(
            Box((arm_length, arm_width, arm_height)),
            origin=Origin(
                xyz=(
                    arm_center_x * math.cos(yaw),
                    arm_center_x * math.sin(yaw),
                    arm_center_z,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark,
            name=f"arm_{index}",
        )
        crown.visual(
            Box((lug_length, lug_width, lug_height)),
            origin=Origin(
                xyz=(
                    lug_center_x * math.cos(yaw),
                    lug_center_x * math.sin(yaw),
                    arm_center_z,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark,
            name=f"lug_{index}",
        )

    for index in range(3):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Box((0.055, 0.048, 0.036)),
            origin=Origin(xyz=(0.0325, 0.0, 0.0)),
            material=dark,
            name="hinge_cap",
        )
        leg.visual(
            Box((0.016, 0.032, 0.010)),
            origin=Origin(xyz=(0.003, 0.0, 0.019)),
            material=graphite,
            name="hinge_shoe",
        )
        leg.visual(
            Box((LEG_BEAM_LEN, 0.050, 0.032)),
            origin=Origin(
                xyz=_beam_center(LEG_BEAM_START, LEG_BEAM_LEN, LEG_PITCH),
                rpy=(0.0, LEG_PITCH, 0.0),
            ),
            material=aluminum,
            name="leg_beam",
        )
        foot_length = 0.120
        beam_tip = _beam_end(LEG_BEAM_START, LEG_BEAM_LEN, LEG_PITCH)
        foot_center = (
            beam_tip[0] + (0.5 * foot_length - LEG_FOOT_OVERLAP) * math.cos(LEG_PITCH),
            0.0,
            beam_tip[2] - (0.5 * foot_length - LEG_FOOT_OVERLAP) * math.sin(LEG_PITCH),
        )
        leg.visual(
            Box((foot_length, 0.080, 0.018)),
            origin=Origin(
                xyz=foot_center,
                rpy=(0.0, LEG_PITCH, 0.0),
            ),
            material=rubber,
            name="foot_pad",
        )

        yaw = index * (2.0 * math.pi / 3.0)
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(
                    hinge_radius * math.cos(yaw),
                    hinge_radius * math.sin(yaw),
                    arm_center_z,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=35.0,
                velocity=1.2,
                lower=0.0,
                upper=1.85,
            ),
        )

    mast = model.part("mast")
    mast_length = 1.000
    mast_center_z = 0.180
    mast.visual(
        Box((0.058, 0.058, mast_length)),
        origin=Origin(xyz=(0.0, 0.0, mast_center_z)),
        material=aluminum,
        name="mast_tube",
    )
    mast_cap_height = 0.030
    mast_cap_center_z = mast_center_z + mast_length * 0.5 + mast_cap_height * 0.5
    mast.visual(
        Box((0.062, 0.062, mast_cap_height)),
        origin=Origin(xyz=(0.0, 0.0, mast_cap_center_z)),
        material=dark,
        name="mast_cap",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.052, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark,
        name="turntable",
    )
    head.visual(
        Box((0.082, 0.118, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=dark,
        name="yoke_base",
    )
    head.visual(
        Box((0.072, 0.012, 0.104)),
        origin=Origin(xyz=(0.0, 0.047, 0.100)),
        material=graphite,
        name="cheek_0",
    )
    head.visual(
        Box((0.072, 0.012, 0.104)),
        origin=Origin(xyz=(0.0, -0.047, 0.100)),
        material=graphite,
        name="cheek_1",
    )

    device = model.part("device")
    device.visual(
        Box((0.150, 0.074, 0.105)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=amber,
        name="body_shell",
    )
    device.visual(
        Cylinder(radius=0.010, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=graphite,
        name="tilt_axle",
    )
    device.visual(
        Cylinder(radius=0.028, length=0.045),
        origin=Origin(xyz=(0.146, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark,
        name="optic_tube",
    )
    device.visual(
        Box((0.072, 0.024, 0.020)),
        origin=Origin(xyz=(0.018, 0.0, 0.0625)),
        material=dark,
        name="handle_bar",
    )

    mast_joint = model.articulation(
        "crown_to_mast",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, collar_center_z + collar_height * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=0.280,
        ),
    )

    mast_top = mast_cap_center_z + mast_cap_height * 0.5
    pan_joint = model.articulation(
        "mast_to_head",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, mast_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )

    tilt_joint = model.articulation(
        "head_to_device",
        ArticulationType.REVOLUTE,
        parent=head,
        child=device,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-0.60,
            upper=0.55,
        ),
    )

    # Keep local references live for readability in debug/probe workflows.
    assert mast_joint is not None and pan_joint is not None and tilt_joint is not None
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    mast = object_model.get_part("mast")
    head = object_model.get_part("head")
    device = object_model.get_part("device")
    leg_0 = object_model.get_part("leg_0")

    mast_joint = object_model.get_articulation("crown_to_mast")
    pan_joint = object_model.get_articulation("mast_to_head")
    tilt_joint = object_model.get_articulation("head_to_device")
    leg_joint = object_model.get_articulation("crown_to_leg_0")

    for index in range(3):
        ctx.allow_overlap(
            crown,
            object_model.get_part(f"leg_{index}"),
            elem_a=f"lug_{index}",
            elem_b="hinge_shoe",
            reason="Each folding leg is simplified as a captured hinge shoe nested into the crown lug at the pivot.",
        )

    ctx.expect_origin_distance(
        mast,
        crown,
        axes="xy",
        max_dist=0.001,
        name="mast stays centered over the tripod crown",
    )
    ctx.expect_overlap(
        mast,
        crown,
        axes="z",
        elem_a="mast_tube",
        min_overlap=0.050,
        name="mast remains inserted in the crown at rest",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({mast_joint: 0.280}):
        ctx.expect_overlap(
            mast,
            crown,
            axes="z",
            elem_a="mast_tube",
            min_overlap=0.020,
            name="mast keeps retained insertion at full extension",
        )
        extended_head_pos = ctx.part_world_position(head)

    ctx.check(
        "mast extends upward",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[2] > rest_head_pos[2] + 0.200,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    rest_device_center = _aabb_center(ctx.part_world_aabb(device))
    with ctx.pose({pan_joint: math.pi * 0.5}):
        panned_device_center = _aabb_center(ctx.part_world_aabb(device))

    ctx.check(
        "pan head swings the device around the mast",
        rest_device_center is not None
        and panned_device_center is not None
        and panned_device_center[1] > rest_device_center[1] + 0.055
        and abs(panned_device_center[0]) < rest_device_center[0],
        details=f"rest={rest_device_center}, panned={panned_device_center}",
    )

    rest_optic_center = _aabb_center(ctx.part_element_world_aabb(device, elem="optic_tube"))
    with ctx.pose({tilt_joint: 0.45}):
        tilted_optic_center = _aabb_center(ctx.part_element_world_aabb(device, elem="optic_tube"))

    ctx.check(
        "positive tilt raises the instrument nose",
        rest_optic_center is not None
        and tilted_optic_center is not None
        and tilted_optic_center[2] > rest_optic_center[2] + 0.035,
        details=f"rest={rest_optic_center}, tilted={tilted_optic_center}",
    )

    rest_leg_aabb = ctx.part_world_aabb(leg_0)
    with ctx.pose({leg_joint: 1.75}):
        folded_leg_aabb = ctx.part_world_aabb(leg_0)

    ctx.check(
        "leg folds upward toward the crown",
        rest_leg_aabb is not None
        and folded_leg_aabb is not None
        and folded_leg_aabb[0][2] > rest_leg_aabb[0][2] + 0.450,
        details=f"rest={rest_leg_aabb}, folded={folded_leg_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
