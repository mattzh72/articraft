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


def _rotate_y(point: tuple[float, float, float], pitch: float) -> tuple[float, float, float]:
    x, y, z = point
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    return (x * cp + z * sp, y, -x * sp + z * cp)


def _rotate_z(point: tuple[float, float, float], yaw: float) -> tuple[float, float, float]:
    x, y, z = point
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (x * cy - y * sy, x * sy + y * cy, z)


def _translate(
    origin: tuple[float, float, float], offset: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (origin[0] + offset[0], origin[1] + offset[1], origin[2] + offset[2])


def _add_crown_yoke(
    stand,
    *,
    hinge_xyz: tuple[float, float, float],
    yaw: float,
    wood_material,
    hardware_material,
    prefix: str,
) -> None:
    cheek_size = (0.028, 0.008, 0.042)
    cheek_offset = 0.018
    for index, sign in enumerate((-1.0, 1.0)):
        cheek_center = _translate(
            hinge_xyz,
            _rotate_z((0.0, sign * cheek_offset, 0.0), yaw),
        )
        stand.visual(
            Box(cheek_size),
            origin=Origin(xyz=cheek_center, rpy=(0.0, 0.0, yaw)),
            material=wood_material,
            name=f"{prefix}_cheek_{index}",
        )

    for index, sign in enumerate((-1.0, 1.0)):
        head_center = _translate(
            hinge_xyz,
            _rotate_z((0.0, sign * (cheek_offset + 0.005), 0.0), yaw),
        )
        stand.visual(
            Cylinder(radius=0.006, length=0.007),
            origin=Origin(xyz=head_center, rpy=(math.pi / 2.0, 0.0, yaw)),
            material=hardware_material,
            name=f"{prefix}_pin_head_{index}",
        )


def _add_leg_visuals(
    leg,
    *,
    shaft_length: float,
    rest_splay: float,
    wood_material,
    hardware_material,
    rubber_material,
) -> None:
    pitch = -rest_splay
    shaft_top_drop = 0.036
    shaft_size = (0.026, 0.018, shaft_length)
    foot_length = 0.050

    shaft_center = _rotate_y((0.0, 0.0, -(shaft_top_drop + shaft_length * 0.5)), pitch)
    leg.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name="barrel",
    )
    leg.visual(
        Box((0.020, 0.018, 0.030)),
        origin=Origin(
            xyz=_rotate_y((0.0, 0.0, -0.018), pitch),
            rpy=(0.0, pitch, 0.0),
        ),
        material=wood_material,
        name="knuckle",
    )
    leg.visual(
        Box((0.036, 0.020, 0.070)),
        origin=Origin(
            xyz=_rotate_y((0.0, 0.0, -0.050), pitch),
            rpy=(0.0, pitch, 0.0),
        ),
        material=wood_material,
        name="upper_block",
    )
    leg.visual(
        Box(shaft_size),
        origin=Origin(xyz=shaft_center, rpy=(0.0, pitch, 0.0)),
        material=wood_material,
        name="shaft",
    )
    leg.visual(
        Box((0.046, 0.030, foot_length)),
        origin=Origin(
            xyz=_rotate_y(
                (0.0, 0.0, -(shaft_top_drop + shaft_length + foot_length * 0.5 - 0.010)),
                pitch,
            ),
            rpy=(0.0, pitch, 0.0),
        ),
        material=rubber_material,
        name="foot",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_field_easel")

    birch = model.material("birch", rgba=(0.74, 0.64, 0.43, 1.0))
    hardware = model.material("hardware", rgba=(0.19, 0.20, 0.22, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.71, 0.58, 0.24, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.048, 0.030, 1.260)),
        origin=Origin(xyz=(0.0, 0.0, 0.730)),
        material=birch,
        name="mast",
    )
    stand.visual(
        Box((0.040, 0.026, 0.320)),
        origin=Origin(xyz=(0.0, 0.0, 1.470)),
        material=birch,
        name="upper_mast",
    )
    stand.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.445)),
        material=birch,
        name="crown_hub",
    )
    stand.visual(
        Box((0.096, 0.064, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 1.355)),
        material=birch,
        name="crown_block",
    )
    stand.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.650)),
        material=brass,
        name="top_cap",
    )

    hinge_height = 1.445
    hinge_radius = 0.062
    leg_layout = (
        ("front_leg", math.pi / 2.0, math.radians(18.0)),
        ("rear_leg_0", 7.0 * math.pi / 6.0, math.radians(20.0)),
        ("rear_leg_1", 11.0 * math.pi / 6.0, math.radians(20.0)),
    )

    for name, yaw, _ in leg_layout:
        hinge_xyz = (
            hinge_radius * math.cos(yaw),
            hinge_radius * math.sin(yaw),
            hinge_height,
        )
        _add_crown_yoke(
            stand,
            hinge_xyz=hinge_xyz,
            yaw=yaw,
            wood_material=birch,
            hardware_material=hardware,
            prefix=name,
        )

    for name, yaw, rest_splay in leg_layout:
        leg = model.part(name)
        _add_leg_visuals(
            leg,
            shaft_length=1.560,
            rest_splay=rest_splay,
            wood_material=birch,
            hardware_material=hardware,
            rubber_material=rubber,
        )
        model.articulation(
            f"{name}_hinge",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=leg,
            origin=Origin(
                xyz=(
                    hinge_radius * math.cos(yaw),
                    hinge_radius * math.sin(yaw),
                    hinge_height,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=1.4,
                lower=-0.10,
                upper=2.05,
            ),
        )

    canvas_rail = model.part("canvas_rail")
    canvas_rail.visual(
        Box((0.088, 0.012, 0.118)),
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
        material=hardware,
        name="front_carriage",
    )
    canvas_rail.visual(
        Box((0.012, 0.052, 0.118)),
        origin=Origin(xyz=(-0.037, 0.005, 0.0)),
        material=hardware,
        name="left_guide",
    )
    canvas_rail.visual(
        Box((0.012, 0.052, 0.118)),
        origin=Origin(xyz=(0.037, 0.005, 0.0)),
        material=hardware,
        name="right_guide",
    )
    canvas_rail.visual(
        Box((0.052, 0.014, 0.720)),
        origin=Origin(xyz=(0.0, 0.034, 0.245)),
        material=birch,
        name="rail",
    )
    canvas_rail.visual(
        Box((0.112, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, 0.047, 0.590)),
        material=birch,
        name="top_clip",
    )
    canvas_rail.visual(
        Box((0.320, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.073, -0.078)),
        material=birch,
        name="tray",
    )
    canvas_rail.visual(
        Box((0.320, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, 0.118, -0.071)),
        material=birch,
        name="tray_lip",
    )
    canvas_rail.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(
            xyz=(0.049, 0.012, 0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware,
        name="lock_stem",
    )
    canvas_rail.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(
            xyz=(0.062, 0.012, 0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="lock_knob",
    )

    model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=canvas_rail,
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.14,
            lower=0.0,
            upper=0.240,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    canvas_rail = object_model.get_part("canvas_rail")
    rail_slide = object_model.get_articulation("rail_slide")

    rail_upper = 0.0
    if rail_slide.motion_limits is not None and rail_slide.motion_limits.upper is not None:
        rail_upper = rail_slide.motion_limits.upper

    ctx.expect_gap(
        canvas_rail,
        stand,
        axis="y",
        positive_elem="front_carriage",
        negative_elem="mast",
        min_gap=0.007,
        max_gap=0.013,
        name="canvas carriage rides just ahead of the mast",
    )
    ctx.expect_gap(
        canvas_rail,
        stand,
        axis="x",
        positive_elem="right_guide",
        negative_elem="mast",
        min_gap=0.006,
        max_gap=0.012,
        name="right guide clears the mast side",
    )
    ctx.expect_gap(
        stand,
        canvas_rail,
        axis="x",
        positive_elem="mast",
        negative_elem="left_guide",
        min_gap=0.006,
        max_gap=0.012,
        name="left guide clears the mast side",
    )
    ctx.expect_overlap(
        canvas_rail,
        stand,
        axes="yz",
        elem_a="left_guide",
        elem_b="mast",
        min_overlap=0.025,
        name="left guide remains engaged beside the mast",
    )

    rest_rail_pos = ctx.part_world_position(canvas_rail)
    with ctx.pose({rail_slide: rail_upper}):
        ctx.expect_gap(
            canvas_rail,
            stand,
            axis="y",
            positive_elem="front_carriage",
            negative_elem="mast",
            min_gap=0.007,
            max_gap=0.013,
            name="carriage stays in front of the mast at full lift",
        )
        ctx.expect_overlap(
            canvas_rail,
            stand,
            axes="yz",
            elem_a="right_guide",
            elem_b="mast",
            min_overlap=0.025,
            name="right guide stays aligned with the mast at full lift",
        )
        raised_rail_pos = ctx.part_world_position(canvas_rail)

    ctx.check(
        "canvas rail slides upward",
        rest_rail_pos is not None
        and raised_rail_pos is not None
        and raised_rail_pos[2] > rest_rail_pos[2] + 0.18,
        details=f"rest={rest_rail_pos}, raised={raised_rail_pos}",
    )

    for joint_name in ("front_leg_hinge", "rear_leg_0_hinge", "rear_leg_1_hinge"):
        leg_name = joint_name.removesuffix("_hinge")
        leg = object_model.get_part(leg_name)
        hinge = object_model.get_articulation(joint_name)
        fold_pose = 1.75
        if hinge.motion_limits is not None and hinge.motion_limits.upper is not None:
            fold_pose = min(fold_pose, hinge.motion_limits.upper)
        rest_foot = ctx.part_element_world_aabb(leg, elem="foot")
        with ctx.pose({hinge: fold_pose}):
            folded_foot = ctx.part_element_world_aabb(leg, elem="foot")
        ctx.check(
            f"{leg_name} folds toward the crown",
            rest_foot is not None
            and folded_foot is not None
            and folded_foot[0][2] > rest_foot[0][2] + 0.80,
            details=f"rest={rest_foot}, folded={folded_foot}",
        )

    return ctx.report()


object_model = build_object_model()
