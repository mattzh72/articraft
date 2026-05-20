from __future__ import annotations

from math import cos, pi, sin

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


MAST_DEPTH = 0.030
MAST_WIDTH = 0.045
MAST_HEIGHT = 1.60
CROWN_Z = 1.535
HINGE_Z = 1.49
CLAMP_Z = 1.22


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    return (
        (x * cos(yaw)) - (y * sin(yaw)),
        (x * sin(yaw)) + (y * cos(yaw)),
    )


def _add_hinge_ears(
    frame,
    *,
    name_prefix: str,
    hinge_xyz: tuple[float, float, float],
    yaw: float,
    material: str,
) -> None:
    hinge_x, hinge_y, hinge_z = hinge_xyz
    ear_box = (0.020, 0.008, 0.056)
    for index, side in enumerate((-1.0, 1.0)):
        dx, dy = _rotate_xy(0.0, side * 0.021, yaw)
        frame.visual(
            Box(ear_box),
            origin=Origin(
                xyz=(hinge_x + dx, hinge_y + dy, hinge_z),
                rpy=(0.0, 0.0, yaw),
            ),
            material=material,
            name=f"{name_prefix}_ear_{index}",
        )


def _add_leg(
    leg,
    *,
    length: float,
    open_pitch: float,
    wood_material: str,
    foot_material: str,
) -> None:
    direction_x = sin(open_pitch)
    direction_z = -cos(open_pitch)
    strut_center = (0.5 * length * direction_x, 0.0, 0.5 * length * direction_z)

    leg.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="hinge_steel",
        name="barrel",
    )
    leg.visual(
        Box((0.030, 0.024, 0.090)),
        origin=Origin(
            xyz=(0.045 * direction_x, 0.0, 0.045 * direction_z),
            rpy=(0.0, -open_pitch, 0.0),
        ),
        material=wood_material,
        name="head",
    )
    leg.visual(
        Box((0.026, 0.020, length)),
        origin=Origin(xyz=strut_center, rpy=(0.0, -open_pitch, 0.0)),
        material=wood_material,
        name="strut",
    )

    foot_len = 0.040
    foot_center = (
        (length - (0.5 * foot_len)) * direction_x,
        0.0,
        (length - (0.5 * foot_len)) * direction_z,
    )
    leg.visual(
        Box((0.044, 0.032, foot_len)),
        origin=Origin(xyz=foot_center, rpy=(0.0, -open_pitch, 0.0)),
        material=foot_material,
        name="foot",
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo = aabb[0]
    hi = aabb[1]
    return tuple((low + high) * 0.5 for low, high in zip(lo, hi))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_easel")

    model.material("wood", rgba=(0.64, 0.46, 0.28, 1.0))
    model.material("hinge_steel", rgba=(0.40, 0.42, 0.45, 1.0))
    model.material("rubber", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("clamp_pad", rgba=(0.18, 0.19, 0.20, 1.0))

    frame = model.part("frame")
    top_clamp = model.part("top_clamp")
    front_left_leg = model.part("front_left_leg")
    front_right_leg = model.part("front_right_leg")
    rear_leg = model.part("rear_leg")

    frame.visual(
        Box((MAST_DEPTH, MAST_WIDTH, MAST_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT * 0.5)),
        material="wood",
        name="mast",
    )
    frame.visual(
        Box((0.076, 0.120, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 1.5425)),
        material="wood",
        name="crown",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.600)),
        material="wood",
        name="finial",
    )
    frame.visual(
        Box((0.058, 0.105, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.715)),
        material="wood",
        name="ledge_block",
    )
    frame.visual(
        Box((0.180, 0.235, 0.022)),
        origin=Origin(xyz=(0.102, 0.0, 0.675)),
        material="wood",
        name="ledge_panel",
    )
    frame.visual(
        Box((0.022, 0.235, 0.026)),
        origin=Origin(xyz=(0.180, 0.0, 0.687)),
        material="wood",
        name="ledge_lip",
    )
    frame.visual(
        Box((0.020, 0.060, 0.220)),
        origin=Origin(xyz=(0.020, 0.0, 0.815)),
        material="wood",
        name="center_stay",
    )

    _add_hinge_ears(
        frame,
        name_prefix="front_left",
        hinge_xyz=(0.0, 0.053, HINGE_Z),
        yaw=0.82,
        material="hinge_steel",
    )
    _add_hinge_ears(
        frame,
        name_prefix="front_right",
        hinge_xyz=(0.0, -0.053, HINGE_Z),
        yaw=-0.82,
        material="hinge_steel",
    )
    _add_hinge_ears(
        frame,
        name_prefix="rear",
        hinge_xyz=(-0.032, 0.0, HINGE_Z),
        yaw=pi,
        material="hinge_steel",
    )

    top_clamp.visual(
        Box((0.008, 0.072, 0.110)),
        origin=Origin(xyz=(-0.021, 0.0, 0.0)),
        material="hinge_steel",
        name="back_plate",
    )
    for side, suffix in ((0.026, "0"), (-0.026, "1")):
        top_clamp.visual(
            Box((0.052, 0.008, 0.110)),
            origin=Origin(xyz=(0.004, side, 0.0)),
            material="hinge_steel",
            name=f"side_{suffix}",
        )
    top_clamp.visual(
        Box((0.012, 0.052, 0.036)),
        origin=Origin(xyz=(0.031, 0.0, 0.016)),
        material="clamp_pad",
        name="jaw_pad",
    )
    top_clamp.visual(
        Box((0.022, 0.052, 0.010)),
        origin=Origin(xyz=(0.028, 0.0, -0.010)),
        material="clamp_pad",
        name="jaw_lip",
    )
    top_clamp.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.010, 0.036, 0.028), rpy=(pi / 2.0, 0.0, 0.0)),
        material="hinge_steel",
        name="thumb_knob",
    )

    _add_leg(
        front_left_leg,
        length=1.56,
        open_pitch=0.37,
        wood_material="wood",
        foot_material="rubber",
    )
    _add_leg(
        front_right_leg,
        length=1.56,
        open_pitch=0.37,
        wood_material="wood",
        foot_material="rubber",
    )
    _add_leg(
        rear_leg,
        length=1.58,
        open_pitch=0.43,
        wood_material="wood",
        foot_material="rubber",
    )

    model.articulation(
        "clamp_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=top_clamp,
        origin=Origin(xyz=(0.0, 0.0, CLAMP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-0.18,
            upper=0.18,
            effort=25.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "front_left_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_left_leg,
        origin=Origin(xyz=(0.0, 0.053, HINGE_Z), rpy=(0.0, 0.0, 0.82)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.10,
            upper=0.18,
            effort=20.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "front_right_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_right_leg,
        origin=Origin(xyz=(0.0, -0.053, HINGE_Z), rpy=(0.0, 0.0, -0.82)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.10,
            upper=0.18,
            effort=20.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_leg,
        origin=Origin(xyz=(-0.032, 0.0, HINGE_Z), rpy=(0.0, 0.0, pi)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.08,
            upper=0.20,
            effort=20.0,
            velocity=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    top_clamp = object_model.get_part("top_clamp")
    front_left_leg = object_model.get_part("front_left_leg")
    front_right_leg = object_model.get_part("front_right_leg")
    rear_leg = object_model.get_part("rear_leg")

    clamp_slide = object_model.get_articulation("clamp_slide")
    front_left_hinge = object_model.get_articulation("front_left_hinge")
    rear_hinge = object_model.get_articulation("rear_hinge")

    ctx.expect_origin_distance(
        top_clamp,
        frame,
        axes="xy",
        max_dist=0.001,
        name="clamp remains centered on the mast",
    )
    ctx.expect_gap(
        top_clamp,
        frame,
        axis="z",
        positive_elem="jaw_pad",
        negative_elem="ledge_panel",
        min_gap=0.30,
        name="top clamp stays above the support ledge",
    )

    clamp_rest = ctx.part_world_position(top_clamp)
    with ctx.pose({clamp_slide: 0.16}):
        clamp_raised = ctx.part_world_position(top_clamp)
    ctx.check(
        "top clamp slides upward",
        clamp_rest is not None
        and clamp_raised is not None
        and clamp_raised[2] > clamp_rest[2] + 0.12,
        details=f"rest={clamp_rest}, raised={clamp_raised}",
    )

    left_rest = _aabb_center(ctx.part_element_world_aabb(front_left_leg, elem="foot"))
    right_rest = _aabb_center(ctx.part_element_world_aabb(front_right_leg, elem="foot"))
    ctx.check(
        "front legs form a wide tripod stance",
        left_rest is not None
        and right_rest is not None
        and left_rest[1] > 0.30
        and right_rest[1] < -0.30,
        details=f"left={left_rest}, right={right_rest}",
    )

    with ctx.pose({front_left_hinge: 0.12}):
        left_folded = _aabb_center(ctx.part_element_world_aabb(front_left_leg, elem="foot"))
    ctx.check(
        "front left leg folds inward at the crown hinge",
        left_rest is not None
        and left_folded is not None
        and abs(left_folded[1]) < abs(left_rest[1]) - 0.08
        and left_folded[0] < left_rest[0] - 0.08,
        details=f"rest={left_rest}, folded={left_folded}",
    )

    rear_rest = _aabb_center(ctx.part_element_world_aabb(rear_leg, elem="foot"))
    with ctx.pose({rear_hinge: 0.14}):
        rear_folded = _aabb_center(ctx.part_element_world_aabb(rear_leg, elem="foot"))
    ctx.check(
        "rear leg swings toward the mast on its crown hinge",
        rear_rest is not None
        and rear_folded is not None
        and rear_folded[0] > rear_rest[0] + 0.12
        and abs(rear_folded[1] - rear_rest[1]) < 0.02,
        details=f"rest={rear_rest}, folded={rear_folded}",
    )

    return ctx.report()


object_model = build_object_model()
