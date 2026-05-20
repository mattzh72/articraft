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


CROWN_Z = 1.50
FRONT_HINGE_Y = 0.047
FRONT_HINGE_Z = CROWN_Z - 0.020
REAR_HINGE_RADIUS = 0.040
REAR_HINGE_Z = CROWN_Z - 0.031
REAR_LEG_YAWS = (2.58, -2.58)


def _radial_xy_from_yaw(yaw: float, radius: float) -> tuple[float, float]:
    return (-math.sin(yaw) * radius, math.cos(yaw) * radius)


def _add_hinge_ears(
    crown,
    *,
    joint_xyz: tuple[float, float, float],
    yaw: float,
    material: str,
    name_prefix: str,
    z_offset: float = 0.0,
) -> None:
    axis_dx = math.cos(yaw)
    axis_dy = math.sin(yaw)
    for index, sign in enumerate((-1.0, 1.0)):
        crown.visual(
            Box((0.010, 0.024, 0.028)),
            origin=Origin(
                xyz=(
                    joint_xyz[0] + (sign * 0.017 * axis_dx),
                    joint_xyz[1] + (sign * 0.017 * axis_dy),
                    joint_xyz[2] + z_offset,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=material,
            name=f"{name_prefix}_ear_{index}",
        )


def _add_leg_geometry(
    leg,
    *,
    spar_size: tuple[float, float],
    spar_length: float,
    foot_size: tuple[float, float, float],
    wood: str,
    hardware: str,
    rubber: str,
    spar_name: str,
    foot_name: str,
) -> None:
    spar_x, spar_y = spar_size
    foot_x, foot_y, foot_z = foot_size

    leg.visual(
        Cylinder(radius=0.0085, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="hinge_barrel",
    )
    leg.visual(
        Box((0.020, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=hardware,
        name="hinge_shoe",
    )
    leg.visual(
        Box((spar_x, spar_y, spar_length)),
        origin=Origin(xyz=(0.0, 0.0, -(0.014 + spar_length / 2.0))),
        material=wood,
        name=spar_name,
    )
    leg.visual(
        Box((foot_x, foot_y, foot_z)),
        origin=Origin(xyz=(0.0, 0.0, -(0.014 + spar_length + foot_z / 2.0))),
        material=rubber,
        name=foot_name,
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_field_easel")

    model.material("beech", rgba=(0.73, 0.59, 0.38, 1.0))
    model.material("walnut", rgba=(0.54, 0.39, 0.24, 1.0))
    model.material("brass", rgba=(0.76, 0.63, 0.34, 1.0))
    model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    model.material("felt", rgba=(0.26, 0.29, 0.22, 1.0))

    crown = model.part("crown")
    crown.visual(
        Box((0.120, 0.070, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, CROWN_Z)),
        material="walnut",
        name="crown_block",
    )
    crown.visual(
        Box((0.050, 0.036, 0.170)),
        origin=Origin(xyz=(0.0, 0.004, CROWN_Z + 0.1075)),
        material="walnut",
        name="head_post",
    )
    crown.visual(
        Box((0.094, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.020, CROWN_Z + 0.175)),
        material="walnut",
        name="head_cap",
    )

    _add_hinge_ears(
        crown,
        joint_xyz=(0.0, FRONT_HINGE_Y, FRONT_HINGE_Z),
        yaw=0.0,
        material="brass",
        name_prefix="front_hinge",
    )

    rear_joint_positions: list[tuple[float, float, float]] = []
    for index, yaw in enumerate(REAR_LEG_YAWS):
        joint_x, joint_y = _radial_xy_from_yaw(yaw, REAR_HINGE_RADIUS)
        joint_xyz = (joint_x, joint_y, REAR_HINGE_Z)
        rear_joint_positions.append(joint_xyz)

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brass",
        name="hinge_barrel",
    )
    mast.visual(
        Box((0.024, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material="brass",
        name="hinge_shoe",
    )
    mast.visual(
        Box((0.036, 0.028, 1.470)),
        origin=Origin(xyz=(0.0, 0.0, -0.751)),
        material="beech",
        name="mast_spar",
    )
    mast.visual(
        Box((0.014, 0.010, 1.020)),
        origin=Origin(xyz=(0.0, 0.019, -0.600)),
        material="walnut",
        name="guide_strip",
    )
    mast.visual(
        Box((0.082, 0.036, 0.120)),
        origin=Origin(xyz=(0.0, 0.018, -0.075)),
        material="walnut",
        name="top_head",
    )
    mast.visual(
        Box((0.094, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.032, -0.030)),
        material="walnut",
        name="top_lip",
    )
    mast.visual(
        Box((0.066, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -1.494)),
        material="rubber",
        name="front_foot",
    )

    model.articulation(
        "crown_to_mast",
        ArticulationType.REVOLUTE,
        parent=crown,
        child=mast,
        origin=Origin(xyz=(0.0, FRONT_HINGE_Y, FRONT_HINGE_Z), rpy=(0.16, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.18,
        ),
    )

    for index, (yaw, joint_xyz) in enumerate(zip(REAR_LEG_YAWS, rear_joint_positions)):
        leg = model.part(f"rear_leg_{index}")
        _add_leg_geometry(
            leg,
            spar_size=(0.026, 0.022),
            spar_length=1.580,
            foot_size=(0.050, 0.028, 0.018),
            wood="beech",
            hardware="brass",
            rubber="rubber",
            spar_name="rear_spar",
            foot_name="rear_foot",
        )
        model.articulation(
            f"crown_to_rear_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=joint_xyz, rpy=(0.42, 0.0, yaw)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=16.0,
                velocity=1.2,
                lower=-0.50,
                upper=0.20,
            ),
        )

    tray = model.part("tray")
    tray.visual(
        Box((0.010, 0.050, 0.230)),
        origin=Origin(xyz=(-0.023, 0.005, -0.020)),
        material="brass",
        name="guide_left",
    )
    tray.visual(
        Box((0.010, 0.050, 0.230)),
        origin=Origin(xyz=(0.023, 0.005, -0.020)),
        material="brass",
        name="guide_right",
    )
    tray.visual(
        Box((0.068, 0.008, 0.230)),
        origin=Origin(xyz=(0.0, -0.018, -0.020)),
        material="brass",
        name="guide_back",
    )
    tray.visual(
        Box((0.360, 0.072, 0.018)),
        origin=Origin(xyz=(0.0, 0.066, -0.050)),
        material="beech",
        name="tray_board",
    )
    tray.visual(
        Box((0.300, 0.044, 0.002)),
        origin=Origin(xyz=(0.0, 0.067, -0.040)),
        material="felt",
        name="tray_pad",
    )
    tray.visual(
        Box((0.360, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.107, -0.044)),
        material="walnut",
        name="tray_lip",
    )
    tray.visual(
        Box((0.120, 0.012, 0.042)),
        origin=Origin(xyz=(0.0, 0.103, -0.079)),
        material="brass",
        name="box_hanger_neck",
    )
    tray.visual(
        Box((0.120, 0.022, 0.054)),
        origin=Origin(xyz=(0.0, 0.118, -0.118)),
        material="brass",
        name="box_hanger_plate",
    )
    tray.visual(
        Box((0.230, 0.120, 0.010)),
        origin=Origin(xyz=(0.0, 0.188, -0.145)),
        material="walnut",
        name="box_bottom",
    )
    tray.visual(
        Box((0.010, 0.120, 0.056)),
        origin=Origin(xyz=(-0.110, 0.188, -0.117)),
        material="walnut",
        name="box_side_0",
    )
    tray.visual(
        Box((0.010, 0.120, 0.056)),
        origin=Origin(xyz=(0.110, 0.188, -0.117)),
        material="walnut",
        name="box_side_1",
    )
    tray.visual(
        Box((0.230, 0.010, 0.056)),
        origin=Origin(xyz=(0.0, 0.128, -0.117)),
        material="walnut",
        name="box_rear",
    )
    tray.visual(
        Box((0.230, 0.010, 0.044)),
        origin=Origin(xyz=(0.0, 0.248, -0.123)),
        material="walnut",
        name="box_front",
    )
    tray.visual(
        Box((0.006, 0.012, 0.016)),
        origin=Origin(xyz=(-0.118, 0.122, -0.087)),
        material="brass",
        name="lid_tab_0",
    )
    tray.visual(
        Box((0.006, 0.012, 0.016)),
        origin=Origin(xyz=(0.118, 0.122, -0.087)),
        material="brass",
        name="lid_tab_1",
    )

    model.articulation(
        "mast_to_tray",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, -0.930)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.20,
            lower=0.0,
            upper=0.40,
        ),
    )

    box_lid = model.part("box_lid")
    box_lid.visual(
        Cylinder(radius=0.005, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brass",
        name="hinge_barrel",
    )
    box_lid.visual(
        Box((0.228, 0.124, 0.008)),
        origin=Origin(xyz=(0.0, 0.062, 0.004)),
        material="beech",
        name="lid_panel",
    )
    box_lid.visual(
        Box((0.054, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.119, 0.010)),
        material="brass",
        name="pull_lip",
    )

    model.articulation(
        "tray_to_box_lid",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=box_lid,
        origin=Origin(xyz=(0.0, 0.123, -0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    rear_leg_0 = object_model.get_part("rear_leg_0")
    rear_leg_1 = object_model.get_part("rear_leg_1")
    tray = object_model.get_part("tray")
    box_lid = object_model.get_part("box_lid")

    front_hinge = object_model.get_articulation("crown_to_mast")
    rear_hinge_0 = object_model.get_articulation("crown_to_rear_leg_0")
    tray_slide = object_model.get_articulation("mast_to_tray")
    lid_hinge = object_model.get_articulation("tray_to_box_lid")

    ctx.expect_origin_distance(
        tray,
        mast,
        axes="x",
        max_dist=0.001,
        name="tray stays centered on the mast",
    )
    ctx.expect_gap(
        box_lid,
        tray,
        axis="z",
        elem_a="lid_panel",
        elem_b="box_rear",
        min_gap=0.0,
        max_gap=0.008,
        name="closed lid sits just above the box rim",
    )
    ctx.expect_overlap(
        box_lid,
        tray,
        axes="x",
        elem_a="lid_panel",
        elem_b="box_bottom",
        min_overlap=0.220,
        name="lid covers the storage box width",
    )

    front_foot = _aabb_center(ctx.part_element_world_aabb(mast, elem="front_foot"))
    rear_foot_0 = _aabb_center(ctx.part_element_world_aabb(rear_leg_0, elem="rear_foot"))
    rear_foot_1 = _aabb_center(ctx.part_element_world_aabb(rear_leg_1, elem="rear_foot"))
    ctx.check(
        "tripod stance spreads the feet",
        front_foot is not None
        and rear_foot_0 is not None
        and rear_foot_1 is not None
        and front_foot[1] > rear_foot_0[1] + 0.45
        and front_foot[1] > rear_foot_1[1] + 0.45
        and rear_foot_0[0] * rear_foot_1[0] < 0.0
        and abs(rear_foot_0[0] - rear_foot_1[0]) > 0.55,
        details=f"front={front_foot}, rear0={rear_foot_0}, rear1={rear_foot_1}",
    )

    tray_rest = ctx.part_world_position(tray)
    tray_upper = tray_slide.motion_limits.upper if tray_slide.motion_limits is not None else None
    tray_extended = None
    if tray_upper is not None:
        with ctx.pose({tray_slide: tray_upper}):
            tray_extended = ctx.part_world_position(tray)
    ctx.check(
        "tray slides upward on the mast",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[2] > tray_rest[2] + 0.22,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    lid_rest = _aabb_center(ctx.part_element_world_aabb(box_lid, elem="pull_lip"))
    lid_open = None
    with ctx.pose({lid_hinge: 1.15}):
        lid_open = _aabb_center(ctx.part_element_world_aabb(box_lid, elem="pull_lip"))
    ctx.check(
        "storage lid opens upward",
        lid_rest is not None and lid_open is not None and lid_open[2] > lid_rest[2] + 0.08,
        details=f"closed={lid_rest}, open={lid_open}",
    )

    front_fold = None
    with ctx.pose({front_hinge: -0.55}):
        front_fold = _aabb_center(ctx.part_element_world_aabb(mast, elem="front_foot"))
    ctx.check(
        "mast leg folds at the crown hinge",
        front_foot is not None
        and front_fold is not None
        and front_fold[1] < front_foot[1] - 0.45
        and front_fold[2] > front_foot[2] + 0.02,
        details=f"open={front_foot}, folded={front_fold}",
    )

    rear_fold = None
    with ctx.pose({rear_hinge_0: -0.35}):
        rear_fold = _aabb_center(ctx.part_element_world_aabb(rear_leg_0, elem="rear_foot"))
    ctx.check(
        "rear leg folds at the crown hinge",
        rear_foot_0 is not None
        and rear_fold is not None
        and math.hypot(rear_fold[0], rear_fold[1]) < math.hypot(rear_foot_0[0], rear_foot_0[1]) - 0.30,
        details=f"open={rear_foot_0}, folded={rear_fold}",
    )

    return ctx.report()


object_model = build_object_model()
