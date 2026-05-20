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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="survey_tripod_instrument")

    tripod_black = model.material("tripod_black", rgba=(0.16, 0.16, 0.17, 1.0))
    instrument_gray = model.material("instrument_gray", rgba=(0.79, 0.80, 0.77, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.61, 0.63, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.24, 0.28, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.11, length=0.05),
        material=tripod_black,
        name="crown_plate",
    )
    crown.visual(
        Cylinder(radius=0.055, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=tripod_black,
        name="center_hub",
    )
    crown.visual(
        Cylinder(radius=0.030, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        material=steel,
        name="hub_tip",
    )
    crown.visual(
        Cylinder(radius=0.06, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=tripod_black,
        name="pan_mount",
    )

    hinge_radius = 0.084
    deployed_leg_angle = math.radians(28.0)
    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)

    def leg_segment_origin(distance_along_leg: float) -> Origin:
        return Origin(
            xyz=(
                0.0,
                -distance_along_leg * math.sin(deployed_leg_angle),
                -distance_along_leg * math.cos(deployed_leg_angle),
            ),
            rpy=(-deployed_leg_angle, 0.0, 0.0),
        )

    for index, angle in enumerate(leg_angles):
        crown.visual(
            Box((0.070, 0.050, 0.040)),
            origin=Origin(
                xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), 0.020),
                rpy=(0.0, 0.0, angle),
            ),
            material=tripod_black,
            name=f"hinge_lug_{index}",
        )

        leg = model.part(f"leg_{index}")
        leg.visual(
            Box((0.055, 0.042, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.040)),
            material=tripod_black,
            name="hinge_block",
        )
        leg.visual(
            Box((0.050, 0.032, 0.12)),
            origin=leg_segment_origin(0.10),
            material=tripod_black,
            name="leg_shoulder",
        )
        leg.visual(
            Box((0.040, 0.026, 0.90)),
            origin=leg_segment_origin(0.56),
            material=tripod_black,
            name="leg_beam",
        )
        leg.visual(
            Cylinder(radius=0.012, length=0.08),
            origin=leg_segment_origin(1.04),
            material=steel,
            name="foot_collar",
        )
        leg.visual(
            Cylinder(radius=0.0035, length=0.10),
            origin=leg_segment_origin(1.13),
            material=steel,
            name="foot_spike",
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), 0.0),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=80.0,
                velocity=1.2,
                lower=-0.15,
                upper=0.95,
            ),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.070, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_trim,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.040, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=dark_trim,
        name="pedestal",
    )
    pan_head.visual(
        Box((0.10, 0.19, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=dark_trim,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.045, 0.030, 0.16)),
        origin=Origin(xyz=(0.0, -0.082, 0.220)),
        material=dark_trim,
        name="arm_0",
    )
    pan_head.visual(
        Box((0.045, 0.030, 0.16)),
        origin=Origin(xyz=(0.0, 0.082, 0.220)),
        material=dark_trim,
        name="arm_1",
    )

    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.5),
    )

    instrument = model.part("instrument")
    instrument.visual(
        Cylinder(radius=0.014, length=0.017),
        origin=Origin(xyz=(0.0, -0.0585, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="trunnion_0",
    )
    instrument.visual(
        Cylinder(radius=0.014, length=0.017),
        origin=Origin(xyz=(0.0, 0.0585, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="trunnion_1",
    )
    instrument.visual(
        Box((0.18, 0.10, 0.14)),
        origin=Origin(xyz=(0.080, 0.0, 0.045)),
        material=instrument_gray,
        name="body_shell",
    )
    instrument.visual(
        Box((0.070, 0.10, 0.050)),
        origin=Origin(xyz=(0.020, 0.0, -0.012)),
        material=instrument_gray,
        name="underslung_base",
    )
    instrument.visual(
        Cylinder(radius=0.045, length=0.035),
        origin=Origin(xyz=(0.115, 0.0, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="barrel_shroud",
    )
    instrument.visual(
        Cylinder(radius=0.033, length=0.18),
        origin=Origin(xyz=(0.210, 0.0, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="telescope_barrel",
    )
    instrument.visual(
        Box((0.10, 0.08, 0.050)),
        origin=Origin(xyz=(0.060, 0.0, 0.128)),
        material=instrument_gray,
        name="upper_housing",
    )
    instrument.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(-0.035, 0.0, 0.038), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="eyepiece",
    )
    instrument.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.050, 0.056, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="knob_boss",
    )

    model.articulation(
        "pan_head_to_instrument",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=instrument,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-math.radians(20.0),
            upper=math.radians(55.0),
        ),
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shaft",
    )
    clamp_knob.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="hub",
    )
    clamp_knob.visual(
        Box((0.042, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.023, 0.0)),
        material=dark_trim,
        name="wing",
    )
    clamp_knob.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, 0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cap",
    )
    clamp_knob.visual(
        Box((0.010, 0.010, 0.012)),
        origin=Origin(xyz=(0.022, 0.026, 0.012)),
        material=steel,
        name="tab",
    )

    model.articulation(
        "instrument_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=instrument,
        child=clamp_knob,
        origin=Origin(xyz=(0.050, 0.062, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    pan_joint = object_model.get_articulation("crown_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_instrument")
    knob_joint = object_model.get_articulation("instrument_to_clamp_knob")
    instrument = object_model.get_part("instrument")
    clamp_knob = object_model.get_part("clamp_knob")

    for index in range(3):
        leg = object_model.get_part(f"leg_{index}")
        foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_spike")
        foot_min_z = None if foot_aabb is None else foot_aabb[0][2]
        ctx.check(
            f"leg_{index} reaches ground plane",
            foot_min_z is not None and -1.08 <= foot_min_z <= -0.98,
            details=f"foot_min_z={foot_min_z}",
        )

        rest_center = aabb_center(foot_aabb)
        hinge = object_model.get_articulation(f"crown_to_leg_{index}")
        with ctx.pose({hinge: 0.75}):
            folded_center = aabb_center(ctx.part_element_world_aabb(leg, elem="foot_spike"))
        rest_radius = None if rest_center is None else math.hypot(rest_center[0], rest_center[1])
        folded_radius = None if folded_center is None else math.hypot(folded_center[0], folded_center[1])
        ctx.check(
            f"leg_{index} folds inward",
            rest_radius is not None and folded_radius is not None and folded_radius < rest_radius - 0.12,
            details=f"rest_radius={rest_radius}, folded_radius={folded_radius}",
        )

    barrel_rest = aabb_center(ctx.part_element_world_aabb(instrument, elem="telescope_barrel"))
    with ctx.pose({pan_joint: math.pi / 2.0}):
        barrel_panned = aabb_center(ctx.part_element_world_aabb(instrument, elem="telescope_barrel"))
    ctx.check(
        "pan head turns the telescope around the vertical axis",
        barrel_rest is not None
        and barrel_panned is not None
        and barrel_rest[0] > 0.16
        and abs(barrel_rest[1]) < 0.03
        and barrel_panned[1] > 0.16
        and abs(barrel_panned[0]) < 0.05,
        details=f"rest={barrel_rest}, panned={barrel_panned}",
    )

    barrel_level = aabb_center(ctx.part_element_world_aabb(instrument, elem="telescope_barrel"))
    with ctx.pose({tilt_joint: math.radians(45.0)}):
        barrel_tilted = aabb_center(ctx.part_element_world_aabb(instrument, elem="telescope_barrel"))
    ctx.check(
        "instrument tilts upward about the side trunnions",
        barrel_level is not None
        and barrel_tilted is not None
        and barrel_tilted[2] > barrel_level[2] + 0.10,
        details=f"level={barrel_level}, tilted={barrel_tilted}",
    )

    tab_rest = aabb_center(ctx.part_element_world_aabb(clamp_knob, elem="tab"))
    with ctx.pose({knob_joint: math.pi / 2.0}):
        tab_turned = aabb_center(ctx.part_element_world_aabb(clamp_knob, elem="tab"))
    ctx.check(
        "clamp knob rotates on its shaft",
        tab_rest is not None
        and tab_turned is not None
        and abs(tab_rest[0] - tab_turned[0]) > 0.008
        and abs(tab_rest[2] - tab_turned[2]) > 0.02,
        details=f"rest={tab_rest}, turned={tab_turned}",
    )

    return ctx.report()


object_model = build_object_model()
