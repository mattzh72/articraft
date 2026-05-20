from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


TRIPOD_HINGE_Z = 1.30
TRIPOD_HINGE_RADIUS = 0.155
LEG_SPLAY = math.radians(28.0)
LEG_LENGTH = 1.42
LEG_BEAM_LENGTH = 1.37
LEG_BEAM_TOP_OFFSET = 0.05


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surveying_device_tripod")

    hardware_black = model.material("hardware_black", rgba=(0.12, 0.12, 0.13, 1.0))
    survey_yellow = model.material("survey_yellow", rgba=(0.86, 0.67, 0.21, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    instrument_gray = model.material("instrument_gray", rgba=(0.83, 0.84, 0.86, 1.0))
    optic_dark = model.material("optic_dark", rgba=(0.10, 0.14, 0.15, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.63, 0.65, 0.68, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.105, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, TRIPOD_HINGE_Z - 0.035)),
        material=hardware_black,
        name="hub",
    )
    crown.visual(
        Cylinder(radius=0.060, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, TRIPOD_HINGE_Z + 0.015)),
        material=hardware_black,
        name="mast",
    )
    crown.visual(
        Cylinder(radius=0.075, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, TRIPOD_HINGE_Z + 0.085)),
        material=hardware_black,
        name="pan_seat",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        crown.visual(
            Box((0.140, 0.036, 0.050)),
            origin=Origin(
                xyz=(0.095 * c, 0.095 * s, TRIPOD_HINGE_Z - 0.025),
                rpy=(0.0, 0.0, angle),
            ),
            material=hardware_black,
            name=f"hinge_lug_{index}",
        )

    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)

        leg = model.part(f"leg_{index}")
        leg.visual(
            Box((0.060, 0.034, 0.090)),
            origin=Origin(xyz=(0.0, 0.0, -0.045)),
            material=hardware_black,
            name="hinge_block",
        )
        leg.visual(
            Box((0.060, 0.036, LEG_BEAM_LENGTH)),
            origin=Origin(xyz=(0.0, 0.0, -(LEG_BEAM_TOP_OFFSET + LEG_BEAM_LENGTH / 2.0))),
            material=survey_yellow,
            name="beam",
        )
        leg.visual(
            Box((0.100, 0.060, 0.050)),
            origin=Origin(xyz=(0.0, 0.0, -(LEG_BEAM_TOP_OFFSET + LEG_BEAM_LENGTH + 0.025))),
            material=rubber_black,
            name="foot",
        )
        leg.visual(
            Cylinder(radius=0.007, length=0.045),
            origin=Origin(xyz=(0.0, 0.0, -(LEG_BEAM_TOP_OFFSET + LEG_BEAM_LENGTH + 0.0475))),
            material=metal_gray,
            name="spike",
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(TRIPOD_HINGE_RADIUS * c, TRIPOD_HINGE_RADIUS * s, TRIPOD_HINGE_Z),
                rpy=(0.0, -LEG_SPLAY, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=200.0,
                velocity=0.6,
                lower=math.radians(-10.0),
                upper=math.radians(18.0),
            ),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.072, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=hardware_black,
        name="pan_base",
    )
    pan_head.visual(
        Box((0.100, 0.130, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=hardware_black,
        name="pedestal",
    )
    pan_head.visual(
        Box((0.100, 0.190, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=hardware_black,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.050, 0.014, 0.160)),
        origin=Origin(xyz=(0.0, -0.095, 0.140)),
        material=hardware_black,
        name="left_arm",
    )
    pan_head.visual(
        Box((0.050, 0.014, 0.160)),
        origin=Origin(xyz=(0.0, 0.095, 0.140)),
        material=hardware_black,
        name="right_arm",
    )

    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, TRIPOD_HINGE_Z + 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.5),
    )

    body = model.part("body")
    body.visual(
        Box((0.190, 0.110, 0.150)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=instrument_gray,
        name="housing",
    )
    body.visual(
        Box((0.110, 0.090, 0.050)),
        origin=Origin(xyz=(0.020, 0.0, 0.095)),
        material=instrument_gray,
        name="top_cap",
    )
    body.visual(
        Box((0.020, 0.020, 0.060)),
        origin=Origin(xyz=(-0.025, 0.0, 0.125)),
        material=instrument_gray,
        name="handle_post_0",
    )
    body.visual(
        Box((0.020, 0.020, 0.060)),
        origin=Origin(xyz=(0.065, 0.0, 0.125)),
        material=instrument_gray,
        name="handle_post_1",
    )
    body.visual(
        Box((0.130, 0.020, 0.020)),
        origin=Origin(xyz=(0.020, 0.0, 0.165)),
        material=instrument_gray,
        name="carry_handle",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.170),
        origin=Origin(xyz=(0.170, 0.0, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=optic_dark,
        name="front_optic",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.255, 0.0, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="front_bezel",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.090),
        origin=Origin(xyz=(-0.115, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=optic_dark,
        name="rear_optic",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.034),
        origin=Origin(xyz=(0.0, -0.071, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="left_trunnion",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.034),
        origin=Origin(xyz=(0.0, 0.071, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="right_trunnion",
    )

    model.articulation(
        "pan_head_to_body",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=math.radians(-35.0),
            upper=math.radians(70.0),
        ),
    )

    level_knob = model.part("level_knob")
    level_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.022,
                body_style="cylindrical",
                edge_radius=0.002,
                grip=KnobGrip(style="fluted", count=14, depth=0.0015),
                center=False,
            ),
            "level_knob",
        ),
        material=hardware_black,
        name="knob_cap",
    )
    level_knob.visual(
        Cylinder(radius=0.008, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=metal_gray,
        name="shaft",
    )
    level_knob.visual(
        Box((0.012, 0.004, 0.014)),
        origin=Origin(xyz=(0.020, 0.0, 0.029)),
        material=metal_gray,
        name="grip_tab",
    )

    model.articulation(
        "body_to_level_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=level_knob,
        origin=Origin(xyz=(0.070, 0.055, -0.030), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[index] + high[index]) / 2.0 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    body = object_model.get_part("body")
    knob = object_model.get_part("level_knob")
    leg_0 = object_model.get_part("leg_0")

    pan_joint = object_model.get_articulation("crown_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_body")
    knob_joint = object_model.get_articulation("body_to_level_knob")
    leg_joint = object_model.get_articulation("crown_to_leg_0")

    for index in range(3):
        ctx.allow_overlap(
            crown,
            object_model.get_part(f"leg_{index}"),
            elem_a=f"hinge_lug_{index}",
            elem_b="hinge_block",
            reason=(
                "The deployed survey tripod uses nested crown lugs and leg hinge blocks; "
                "the solids intentionally proxy an enclosed pinned hinge."
            ),
        )

    ctx.expect_gap(
        pan_head,
        crown,
        axis="z",
        positive_elem="pan_base",
        negative_elem="pan_seat",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan head sits on the crown bearing",
    )
    ctx.expect_overlap(
        pan_head,
        crown,
        axes="xy",
        elem_a="pan_base",
        elem_b="pan_seat",
        min_overlap=0.050,
        name="pan head stays centered on the crown",
    )

    ctx.expect_gap(
        body,
        pan_head,
        axis="y",
        positive_elem="left_trunnion",
        negative_elem="left_arm",
        max_gap=0.001,
        max_penetration=0.0,
        name="left trunnion meets the left yoke arm",
    )
    ctx.expect_gap(
        pan_head,
        body,
        axis="y",
        positive_elem="right_arm",
        negative_elem="right_trunnion",
        max_gap=0.001,
        max_penetration=0.0,
        name="right trunnion meets the right yoke arm",
    )

    ctx.expect_gap(
        knob,
        body,
        axis="y",
        positive_elem="shaft",
        negative_elem="housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="level knob shaft seats against the body side",
    )
    ctx.expect_overlap(
        knob,
        body,
        axes="xz",
        elem_a="shaft",
        elem_b="housing",
        min_overlap=0.014,
        name="level knob shaft stays aligned with its mount",
    )

    for leg_name in ("leg_0", "leg_1", "leg_2"):
        leg = object_model.get_part(leg_name)
        aabb = ctx.part_world_aabb(leg)
        foot_on_ground = aabb is not None and abs(aabb[0][2]) <= 0.030
        ctx.check(
            f"{leg_name} reaches the ground",
            foot_on_ground,
            details=f"aabb={aabb}",
        )

    rest_front = _aabb_center(ctx.part_element_world_aabb(body, elem="front_optic"))
    with ctx.pose({pan_joint: math.pi / 2.0}):
        panned_front = _aabb_center(ctx.part_element_world_aabb(body, elem="front_optic"))
    ctx.check(
        "pan head yaws the instrument around the vertical axis",
        rest_front is not None
        and panned_front is not None
        and abs(panned_front[1]) > abs(rest_front[1]) + 0.12
        and abs(panned_front[0]) < abs(rest_front[0]) - 0.08,
        details=f"rest_front={rest_front}, panned_front={panned_front}",
    )

    with ctx.pose({tilt_joint: math.radians(35.0)}):
        tilted_front = _aabb_center(ctx.part_element_world_aabb(body, elem="front_optic"))
    ctx.check(
        "positive tilt raises the survey body",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.070
        and tilted_front[0] < rest_front[0] - 0.020,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    rest_tab = _aabb_center(ctx.part_element_world_aabb(knob, elem="grip_tab"))
    with ctx.pose({knob_joint: math.pi / 2.0}):
        turned_tab = _aabb_center(ctx.part_element_world_aabb(knob, elem="grip_tab"))
    ctx.check(
        "level knob rotates on its side shaft",
        rest_tab is not None
        and turned_tab is not None
        and abs(turned_tab[0] - rest_tab[0]) > 0.015
        and abs(turned_tab[2] - rest_tab[2]) > 0.015,
        details=f"rest_tab={rest_tab}, turned_tab={turned_tab}",
    )

    rest_leg = ctx.part_world_aabb(leg_0)
    rest_leg_min_z = None if rest_leg is None else rest_leg[0][2]
    with ctx.pose({leg_joint: leg_joint.motion_limits.lower}):
        lower_leg = ctx.part_world_aabb(leg_0)
    with ctx.pose({leg_joint: leg_joint.motion_limits.upper}):
        upper_leg = ctx.part_world_aabb(leg_0)
    lower_min_z = None if lower_leg is None else lower_leg[0][2]
    upper_min_z = None if upper_leg is None else upper_leg[0][2]
    ctx.check(
        "tripod leg hinge changes the leg height",
        rest_leg_min_z is not None
        and lower_min_z is not None
        and upper_min_z is not None
        and max(lower_min_z, upper_min_z) > rest_leg_min_z + 0.050,
        details=(
            f"rest_leg_min_z={rest_leg_min_z}, "
            f"lower_min_z={lower_min_z}, upper_min_z={upper_min_z}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
