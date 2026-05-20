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
    Sphere,
    TestContext,
    TestReport,
)


LEG_HUB_Z = 0.66
UPRIGHT_HINGE_Z = 0.78
UPRIGHT_HEIGHT = 1.16
HEAD_PIVOT_Z = UPRIGHT_HEIGHT
LEG_DOWN_ANGLE = math.radians(33.0)


def _leg_direction(yaw: float) -> tuple[float, float, float]:
    return (
        math.cos(yaw) * math.cos(LEG_DOWN_ANGLE),
        math.sin(yaw) * math.cos(LEG_DOWN_ANGLE),
        -math.sin(LEG_DOWN_ANGLE),
    )


def _cylinder_rpy_along_leg(yaw: float) -> tuple[float, float, float]:
    # URDF cylinders point along local +Z.  Pitching by 90 deg + the downward
    # leg angle points local +Z into the sloping radial leg direction; yaw
    # then spins that radial plane around the hub.
    return (0.0, math.pi / 2.0 + LEG_DOWN_ANGLE, yaw)


def _scaled(v: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_studio_floodlight")

    matte_black = Material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    satin_black = Material("satin_black", rgba=(0.03, 0.032, 0.035, 1.0))
    dark_metal = Material("dark_metal", rgba=(0.16, 0.16, 0.15, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.62, 0.61, 0.56, 1.0))
    warm_glass = Material("warm_glass", rgba=(1.0, 0.72, 0.22, 0.78))
    rubber = Material("rubber", rgba=(0.005, 0.005, 0.005, 1.0))

    model.materials.extend(
        [matte_black, satin_black, dark_metal, brushed_steel, warm_glass, rubber]
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.075, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, LEG_HUB_Z)),
        material=dark_metal,
        name="lower_hub",
    )
    hub.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(0.0, 0.0, LEG_HUB_Z + 0.015)),
        material=dark_metal,
        name="round_collar",
    )
    hub.visual(
        Box((0.035, 0.13, 0.13)),
        origin=Origin(xyz=(-0.085, 0.0, UPRIGHT_HINGE_Z)),
        material=dark_metal,
        name="hinge_cheek_0",
    )
    hub.visual(
        Box((0.035, 0.13, 0.13)),
        origin=Origin(xyz=(0.085, 0.0, UPRIGHT_HINGE_Z)),
        material=dark_metal,
        name="hinge_cheek_1",
    )
    hub.visual(
        Cylinder(radius=0.018, length=0.24),
        origin=Origin(
            xyz=(0.0, 0.0, UPRIGHT_HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brushed_steel,
        name="upright_hinge_pin",
    )

    leg_yaws = (math.radians(90.0), math.radians(210.0), math.radians(330.0))
    leg_joints = []
    for i, yaw in enumerate(leg_yaws):
        d = _leg_direction(yaw)
        sleeve_center = _add((0.0, 0.0, LEG_HUB_Z), _scaled(d, 0.23))
        hub.visual(
            Cylinder(radius=0.034, length=0.34),
            origin=Origin(xyz=sleeve_center, rpy=_cylinder_rpy_along_leg(yaw)),
            material=satin_black,
            name=f"sleeve_{i}",
        )

        leg = model.part(f"leg_{i}")
        tube_start = 0.03
        tube_end = 1.12
        tube_center = _scaled(d, (tube_start + tube_end) / 2.0)
        leg.visual(
            Cylinder(radius=0.019, length=tube_end - tube_start),
            origin=Origin(xyz=tube_center, rpy=_cylinder_rpy_along_leg(yaw)),
            material=brushed_steel,
            name="inner_tube",
        )
        foot_center = _scaled(d, 1.13)
        leg.visual(
            Box((0.16, 0.075, 0.026)),
            origin=Origin(xyz=foot_center, rpy=(0.0, 0.0, yaw)),
            material=rubber,
            name="rubber_foot",
        )

        joint_origin = _add((0.0, 0.0, LEG_HUB_Z), _scaled(d, 0.08))
        leg_joints.append(
            model.articulation(
                f"leg_slide_{i}",
                ArticulationType.PRISMATIC,
                parent=hub,
                child=leg,
                origin=Origin(xyz=joint_origin),
                axis=d,
                motion_limits=MotionLimits(effort=90.0, velocity=0.28, lower=-0.04, upper=0.22),
            )
        )

    upright = model.part("upright")
    upright.visual(
        Cylinder(radius=0.041, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_knuckle",
    )
    upright.visual(
        Cylinder(radius=0.027, length=UPRIGHT_HEIGHT - 0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.04 + (UPRIGHT_HEIGHT - 0.04) / 2.0)),
        material=satin_black,
        name="upright_tube",
    )
    upright.visual(
        Cylinder(radius=0.043, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, UPRIGHT_HEIGHT - 0.015)),
        material=dark_metal,
        name="top_collar",
    )
    upright.visual(
        Box((0.050, 0.080, 0.340)),
        origin=Origin(xyz=(0.0, 0.000, HEAD_PIVOT_Z + 0.140)),
        material=dark_metal,
        name="yoke_neck",
    )
    upright.visual(
        Box((0.72, 0.080, 0.060)),
        origin=Origin(xyz=(0.0, 0.060, HEAD_PIVOT_Z + 0.245)),
        material=dark_metal,
        name="yoke_bridge",
    )
    upright.visual(
        Box((0.045, 0.100, 0.56)),
        origin=Origin(xyz=(-0.335, 0.130, HEAD_PIVOT_Z)),
        material=dark_metal,
        name="yoke_arm_0",
    )
    upright.visual(
        Cylinder(radius=0.041, length=0.020),
        origin=Origin(xyz=(-0.365, 0.130, HEAD_PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="pivot_boss_0",
    )
    upright.visual(
        Box((0.045, 0.100, 0.56)),
        origin=Origin(xyz=(0.335, 0.130, HEAD_PIVOT_Z)),
        material=dark_metal,
        name="yoke_arm_1",
    )
    upright.visual(
        Cylinder(radius=0.041, length=0.020),
        origin=Origin(xyz=(0.365, 0.130, HEAD_PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="pivot_boss_1",
    )

    upright_tilt = model.articulation(
        "upright_tilt",
        ArticulationType.REVOLUTE,
        parent=hub,
        child=upright,
        origin=Origin(xyz=(0.0, 0.0, UPRIGHT_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.35, upper=0.35),
    )

    head = model.part("flood_head")
    head.visual(
        Box((0.56, 0.22, 0.34)),
        origin=Origin(xyz=(0.0, 0.200, 0.0)),
        material=matte_black,
        name="head_housing",
    )
    head.visual(
        Box((0.47, 0.012, 0.255)),
        origin=Origin(xyz=(0.0, 0.316, 0.0)),
        material=warm_glass,
        name="front_lens",
    )
    head.visual(
        Box((0.56, 0.020, 0.045)),
        origin=Origin(xyz=(0.0, 0.318, 0.167)),
        material=satin_black,
        name="top_bezel",
    )
    head.visual(
        Box((0.56, 0.020, 0.045)),
        origin=Origin(xyz=(0.0, 0.318, -0.167)),
        material=satin_black,
        name="bottom_bezel",
    )
    head.visual(
        Box((0.045, 0.020, 0.34)),
        origin=Origin(xyz=(-0.277, 0.318, 0.0)),
        material=satin_black,
        name="side_bezel_0",
    )
    head.visual(
        Box((0.045, 0.020, 0.34)),
        origin=Origin(xyz=(0.277, 0.318, 0.0)),
        material=satin_black,
        name="side_bezel_1",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.680),
        origin=Origin(xyz=(0.0, 0.130, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="tilt_trunnion",
    )
    for i, x in enumerate((-0.20, -0.10, 0.0, 0.10, 0.20)):
        head.visual(
            Box((0.026, 0.040, 0.290)),
            origin=Origin(xyz=(x, 0.070, 0.0)),
            material=dark_metal,
            name=f"rear_fin_{i}",
        )

    head_tilt = model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=upright,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, HEAD_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-0.75, upper=0.75),
    )

    model.meta["leg_slide_joints"] = tuple(j.name for j in leg_joints)
    model.meta["tilt_joints"] = (upright_tilt.name, head_tilt.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hub = object_model.get_part("hub")
    head = object_model.get_part("flood_head")
    upright = object_model.get_part("upright")

    ctx.allow_overlap(
        hub,
        upright,
        elem_a="upright_hinge_pin",
        elem_b="hinge_knuckle",
        reason="The upright tilt hinge pin is intentionally captured through the rotating knuckle.",
    )
    ctx.expect_overlap(
        hub,
        upright,
        axes="xyz",
        elem_a="upright_hinge_pin",
        elem_b="hinge_knuckle",
        min_overlap=0.030,
        name="upright hinge pin is captured by the knuckle",
    )

    leg_joints = [object_model.get_articulation(f"leg_slide_{i}") for i in range(3)]
    for i, joint in enumerate(leg_joints):
        leg = object_model.get_part(f"leg_{i}")
        ctx.allow_overlap(
            hub,
            leg,
            elem_a=f"sleeve_{i}",
            elem_b="inner_tube",
            reason="The smaller telescoping leg tube is intentionally nested inside the fixed outer sleeve.",
        )
        ctx.expect_overlap(
            leg,
            hub,
            axes="xyz",
            elem_a="inner_tube",
            elem_b=f"sleeve_{i}",
            min_overlap=0.018,
            name=f"leg_{i} retained in sleeve at rest",
        )
        with ctx.pose({joint: 0.22}):
            ctx.expect_overlap(
                leg,
                hub,
                axes="xyz",
                elem_a="inner_tube",
                elem_b=f"sleeve_{i}",
                min_overlap=0.006,
                name=f"leg_{i} retained in sleeve when extended",
            )

        rest = ctx.part_world_position(leg)
        with ctx.pose({joint: 0.22}):
            extended = ctx.part_world_position(leg)
        axis = joint.axis
        travel = None
        if rest is not None and extended is not None and axis is not None:
            travel = sum((extended[k] - rest[k]) * axis[k] for k in range(3))
        ctx.check(
            f"leg_{i} extends along its sloping tube",
            travel is not None and travel > 0.20,
            details=f"travel={travel}, rest={rest}, extended={extended}, axis={axis}",
        )

    upright_tilt = object_model.get_articulation("upright_tilt")
    head_tilt = object_model.get_articulation("head_tilt")
    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({upright_tilt: 0.28}):
        tilted_head_pos = ctx.part_world_position(head)
    ctx.check(
        "upright hinge tilts the raised head",
        rest_head_pos is not None
        and tilted_head_pos is not None
        and abs(tilted_head_pos[1] - rest_head_pos[1]) > 0.20,
        details=f"rest={rest_head_pos}, tilted={tilted_head_pos}",
    )

    def _center_z(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[2] + hi[2]) / 2.0

    rest_lens_z = _center_z(ctx.part_element_world_aabb(head, elem="front_lens"))
    with ctx.pose({head_tilt: 0.60}):
        tilted_lens_z = _center_z(ctx.part_element_world_aabb(head, elem="front_lens"))
    ctx.check(
        "flood head tilts on the yoke trunnion",
        rest_lens_z is not None and tilted_lens_z is not None and tilted_lens_z > rest_lens_z + 0.04,
        details=f"rest_lens_z={rest_lens_z}, tilted_lens_z={tilted_lens_z}",
    )

    ctx.allow_overlap(
        upright,
        head,
        elem_a="yoke_arm_0",
        elem_b="tilt_trunnion",
        reason="The head tilt trunnion is intentionally seated through one yoke arm.",
    )
    ctx.allow_overlap(
        upright,
        head,
        elem_a="yoke_arm_1",
        elem_b="tilt_trunnion",
        reason="The head tilt trunnion is intentionally seated through the opposite yoke arm.",
    )
    ctx.expect_overlap(
        upright,
        head,
        axes="x",
        elem_a="yoke_arm_0",
        elem_b="tilt_trunnion",
        min_overlap=0.001,
        name="head trunnion reaches one yoke arm",
    )
    ctx.expect_overlap(
        upright,
        head,
        axes="x",
        elem_a="yoke_arm_1",
        elem_b="tilt_trunnion",
        min_overlap=0.001,
        name="head trunnion reaches opposite yoke arm",
    )

    return ctx.report()


object_model = build_object_model()
