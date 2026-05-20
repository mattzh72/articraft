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


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _xy_radius(point: tuple[float, float, float] | None) -> float | None:
    if point is None:
        return None
    return math.hypot(point[0], point[1])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_camera")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.12, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.24, 0.26, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.38, 0.40, 0.43, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    glass = model.material("glass", rgba=(0.12, 0.16, 0.20, 1.0))
    accent = model.material("accent", rgba=(0.52, 0.54, 0.58, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=dark_gray,
        name="top_cap",
    )
    crown.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=gunmetal,
        name="core",
    )
    for index, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        crown.visual(
            Box((0.024, 0.012, 0.008)),
            origin=Origin(
                xyz=(0.016 * math.cos(yaw), 0.016 * math.sin(yaw), -0.010),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_gray,
            name=f"leg_mount_{index}",
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=gunmetal,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=gunmetal,
        name="pan_column",
    )
    pan_head.visual(
        Box((0.034, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark_gray,
        name="yoke_bridge",
    )
    for side, sign in (("near", -1.0), ("far", 1.0)):
        pan_head.visual(
            Box((0.016, 0.004, 0.018)),
            origin=Origin(xyz=(0.0, sign * 0.013, 0.033)),
            material=dark_gray,
            name=f"yoke_side_{side}",
        )
    pan_head.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(-0.015, 0.0, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="pan_knob",
    )

    camera = model.part("camera")
    camera.visual(
        Box((0.038, 0.024, 0.004)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=gunmetal,
        name="tilt_plate",
    )
    camera.visual(
        Cylinder(radius=0.0035, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="tilt_axle",
    )
    camera.visual(
        Box((0.058, 0.030, 0.034)),
        origin=Origin(xyz=(0.008, 0.0, 0.019)),
        material=matte_black,
        name="body",
    )
    camera.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.046, 0.0, 0.021), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens",
    )
    camera.visual(
        Box((0.003, 0.024, 0.020)),
        origin=Origin(xyz=(-0.0215, 0.0, 0.020)),
        material=glass,
        name="screen",
    )
    camera.visual(
        Box((0.020, 0.020, 0.008)),
        origin=Origin(xyz=(-0.004, 0.0, 0.040)),
        material=matte_black,
        name="top_hump",
    )
    camera.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(-0.002, 0.0, 0.042)),
        material=accent,
        name="shutter_button",
    )

    leg_pitch = 1.10
    leg_length = 0.122
    hinge_radius = 0.028
    for index, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Box((0.012, 0.016, 0.008)),
            origin=Origin(xyz=(0.006, 0.0, 0.0)),
            material=dark_gray,
            name="hinge_block",
        )
        leg.visual(
            Box((0.058, 0.012, 0.006)),
            origin=Origin(xyz=(0.041, 0.0, 0.0)),
            material=gunmetal,
            name="upper_leg",
        )
        leg.visual(
            Box((0.050, 0.010, 0.005)),
            origin=Origin(xyz=(0.095, 0.0, 0.0)),
            material=gunmetal,
            name="lower_leg",
        )
        leg.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(leg_length, 0.0, -0.0025)),
            material=rubber,
            name="foot_pad",
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * math.cos(yaw), hinge_radius * math.sin(yaw), -0.012),
                rpy=(0.0, leg_pitch, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.5,
                lower=0.0,
                upper=1.20,
            ),
        )

    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )
    model.articulation(
        "pan_head_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=math.radians(-35.0),
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    camera = object_model.get_part("camera")
    legs = [object_model.get_part(f"leg_{index}") for index in range(3)]

    pan_joint = object_model.get_articulation("crown_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_camera")
    leg_joint = object_model.get_articulation("crown_to_leg_0")

    ctx.expect_contact(
        pan_head,
        crown,
        elem_a="pan_base",
        elem_b="top_cap",
        name="pan base seats on crown cap",
    )
    ctx.expect_overlap(
        camera,
        pan_head,
        axes="xy",
        elem_a="tilt_plate",
        elem_b="yoke_bridge",
        min_overlap=0.020,
        name="tilt plate stays centered over the yoke",
    )

    foot_centers = [
        _aabb_center(ctx.part_element_world_aabb(leg, elem="foot_pad"))
        for leg in legs
    ]
    feet_available = all(center is not None for center in foot_centers)
    if feet_available:
        foot_z = [center[2] for center in foot_centers if center is not None]
        pair_dists = []
        for index in range(3):
            a = foot_centers[index]
            b = foot_centers[(index + 1) % 3]
            assert a is not None and b is not None
            pair_dists.append(math.hypot(a[0] - b[0], a[1] - b[1]))
        ctx.check(
            "tripod feet sit near one plane",
            max(foot_z) - min(foot_z) <= 0.004,
            details=f"foot_z={foot_z}",
        )
        ctx.check(
            "tripod stance is broadly even",
            min(pair_dists) >= 0.11 and max(pair_dists) - min(pair_dists) <= 0.012,
            details=f"pair_dists={pair_dists}",
        )
    else:
        ctx.fail("tripod feet can be measured", details=f"foot_centers={foot_centers}")

    rest_lens = _aabb_center(ctx.part_element_world_aabb(camera, elem="lens"))
    with ctx.pose({pan_joint: math.pi / 2.0}):
        turned_lens = _aabb_center(ctx.part_element_world_aabb(camera, elem="lens"))
    ctx.check(
        "pan head swings the lens around the vertical axis",
        rest_lens is not None
        and turned_lens is not None
        and rest_lens[0] > 0.040
        and abs(turned_lens[0]) < 0.018
        and turned_lens[1] > 0.040,
        details=f"rest_lens={rest_lens}, turned_lens={turned_lens}",
    )
    ctx.check(
        "pan articulation is continuous",
        pan_joint.articulation_type == ArticulationType.CONTINUOUS
        and pan_joint.motion_limits is not None
        and pan_joint.motion_limits.lower is None
        and pan_joint.motion_limits.upper is None,
        details=f"type={pan_joint.articulation_type}, limits={pan_joint.motion_limits}",
    )

    tilt_upper = tilt_joint.motion_limits.upper if tilt_joint.motion_limits is not None else None
    raised_lens = None
    if tilt_upper is not None:
        with ctx.pose({tilt_joint: tilt_upper}):
            raised_lens = _aabb_center(ctx.part_element_world_aabb(camera, elem="lens"))
    ctx.check(
        "camera tilts upward at the top of its range",
        rest_lens is not None
        and raised_lens is not None
        and raised_lens[2] > rest_lens[2] + 0.012,
        details=f"rest_lens={rest_lens}, raised_lens={raised_lens}",
    )

    deployed_foot = _aabb_center(ctx.part_element_world_aabb(legs[0], elem="foot_pad"))
    folded_foot = None
    if leg_joint.motion_limits is not None and leg_joint.motion_limits.upper is not None:
        with ctx.pose({leg_joint: leg_joint.motion_limits.upper}):
            folded_foot = _aabb_center(ctx.part_element_world_aabb(legs[0], elem="foot_pad"))
    ctx.check(
        "one tripod leg folds up toward the crown",
        deployed_foot is not None
        and folded_foot is not None
        and folded_foot[2] > deployed_foot[2] + 0.090,
        details=f"deployed_foot={deployed_foot}, folded_foot={folded_foot}",
    )

    return ctx.report()


object_model = build_object_model()
