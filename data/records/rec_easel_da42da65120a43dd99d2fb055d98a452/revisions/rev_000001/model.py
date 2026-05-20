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


UPRIGHT_X = 0.58
UPRIGHT_W = 0.045
UPRIGHT_D = 0.032
UPRIGHT_H = 1.72

CROWN_W = (2.0 * UPRIGHT_X) + UPRIGHT_W
CROWN_D = 0.055
CROWN_H = 0.060
CROWN_Z = UPRIGHT_H - (CROWN_H * 0.5)

BOARD_W = 1.090
BOARD_H = 0.860
BOARD_RAIL = 0.050
BOARD_D = 0.018
BOARD_CENTER_Z = 0.970

TRAY_TRAVEL = 0.520
TRAY_JOINT_Z = 0.580
SLEEVE_H = 0.160
SLEEVE_CLEARANCE = 0.000
SLEEVE_WALL = 0.006

REAR_LEG_REST_ANGLE = 0.34
REAR_LEG_LEN = 1.70
REAR_LEG_START_Y = 0.023
REAR_LEG_START_Z = -0.040
HINGE_Y = 0.045
HINGE_Z = 1.690


def _add_tray_sleeve(part, index: int, x_center: float, material: str) -> tuple[str, ...]:
    inner_w = UPRIGHT_W + SLEEVE_CLEARANCE
    inner_d = UPRIGHT_D + SLEEVE_CLEARANCE
    outer_d = inner_d + (2.0 * SLEEVE_WALL)
    wall_offset_x = (inner_w * 0.5) + (SLEEVE_WALL * 0.5)
    wall_offset_y = (inner_d * 0.5) + (SLEEVE_WALL * 0.5)

    names = (
        f"sleeve_{index}_front",
        f"sleeve_{index}_back",
        f"sleeve_{index}_inboard",
        f"sleeve_{index}_outboard",
    )

    part.visual(
        Box((inner_w, SLEEVE_WALL, SLEEVE_H)),
        origin=Origin(xyz=(x_center, -wall_offset_y, 0.0)),
        material=material,
        name=names[0],
    )
    part.visual(
        Box((inner_w, SLEEVE_WALL, SLEEVE_H)),
        origin=Origin(xyz=(x_center, wall_offset_y, 0.0)),
        material=material,
        name=names[1],
    )

    inboard_sign = -1.0 if x_center > 0.0 else 1.0
    outboard_sign = -inboard_sign
    part.visual(
        Box((SLEEVE_WALL, outer_d, SLEEVE_H)),
        origin=Origin(xyz=(x_center + (inboard_sign * wall_offset_x), 0.0, 0.0)),
        material=material,
        name=names[2],
    )
    part.visual(
        Box((SLEEVE_WALL, outer_d, SLEEVE_H)),
        origin=Origin(xyz=(x_center + (outboard_sign * wall_offset_x), 0.0, 0.0)),
        material=material,
        name=names[3],
    )
    return names


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_whiteboard_easel")

    model.material("aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("powder_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    model.material("board_white", rgba=(0.97, 0.98, 0.98, 1.0))
    model.material("backer_gray", rgba=(0.86, 0.88, 0.90, 1.0))
    model.material("tray_dark", rgba=(0.22, 0.25, 0.28, 1.0))
    model.material("leg_dark", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("hinge_steel", rgba=(0.69, 0.72, 0.75, 1.0))
    model.material("foot_black", rgba=(0.10, 0.11, 0.12, 1.0))

    front_frame = model.part("front_frame")
    for index, x_pos in enumerate((-UPRIGHT_X, UPRIGHT_X)):
        front_frame.visual(
            Box((UPRIGHT_W, UPRIGHT_D, UPRIGHT_H)),
            origin=Origin(xyz=(x_pos, 0.0, UPRIGHT_H * 0.5)),
            material="powder_gray",
            name=f"upright_{index}",
        )
        front_frame.visual(
            Box((0.095, 0.075, 0.020)),
            origin=Origin(xyz=(x_pos, 0.0, 0.010)),
            material="foot_black",
            name=f"front_foot_{index}",
        )

    front_frame.visual(
        Box((CROWN_W, CROWN_D, CROWN_H)),
        origin=Origin(xyz=(0.0, 0.0, CROWN_Z)),
        material="aluminum",
        name="crown",
    )
    front_frame.visual(
        Box((BOARD_W, BOARD_D, BOARD_RAIL)),
        origin=Origin(
            xyz=(0.0, 0.0, BOARD_CENTER_Z + ((BOARD_H * 0.5) - (BOARD_RAIL * 0.5)))
        ),
        material="aluminum",
        name="board_top",
    )
    front_frame.visual(
        Box((BOARD_W, BOARD_D, BOARD_RAIL)),
        origin=Origin(
            xyz=(0.0, 0.0, BOARD_CENTER_Z - ((BOARD_H * 0.5) - (BOARD_RAIL * 0.5)))
        ),
        material="aluminum",
        name="board_bottom",
    )
    for index, x_pos in enumerate(
        (
            -((BOARD_W * 0.5) - (BOARD_RAIL * 0.5)),
            (BOARD_W * 0.5) - (BOARD_RAIL * 0.5),
        )
    ):
        front_frame.visual(
            Box((BOARD_RAIL, BOARD_D, BOARD_H - (2.0 * BOARD_RAIL))),
            origin=Origin(xyz=(x_pos, 0.0, BOARD_CENTER_Z)),
            material="aluminum",
            name=f"board_side_{index}",
        )
    for index, x_pos in enumerate((-0.55125, 0.55125)):
        front_frame.visual(
            Box((0.015, BOARD_D, 0.220)),
            origin=Origin(xyz=(x_pos, 0.0, BOARD_CENTER_Z)),
            material="aluminum",
            name=f"board_bracket_{index}",
        )

    front_frame.visual(
        Box((BOARD_W - (2.0 * BOARD_RAIL), 0.010, BOARD_H - (2.0 * BOARD_RAIL))),
        origin=Origin(xyz=(0.0, -0.002, BOARD_CENTER_Z)),
        material="backer_gray",
        name="board_backing",
    )
    front_frame.visual(
        Box((BOARD_W - (2.0 * BOARD_RAIL) - 0.010, 0.004, BOARD_H - (2.0 * BOARD_RAIL) - 0.010)),
        origin=Origin(xyz=(0.0, -0.007, BOARD_CENTER_Z)),
        material="board_white",
        name="white_surface",
    )
    front_frame.visual(
        Box((1.115, 0.028, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material="aluminum",
        name="lower_spreader",
    )
    front_frame.visual(
        Box((0.050, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, 0.033, 1.676)),
        material="hinge_steel",
        name="hinge_mount",
    )
    front_frame.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="hinge_steel",
        name="hinge_knuckle",
    )

    tray = model.part("tray")
    tray.visual(
        Box((1.000, 0.075, 0.006)),
        origin=Origin(xyz=(0.0, -0.060, -0.070)),
        material="tray_dark",
        name="tray_floor",
    )
    tray.visual(
        Box((1.000, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, -0.094, -0.058)),
        material="tray_dark",
        name="tray_front_lip",
    )
    tray.visual(
        Box((1.000, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.022, -0.062)),
        material="tray_dark",
        name="tray_rear_lip",
    )
    for index, x_pos in enumerate((-0.5285, 0.5285)):
        tray.visual(
            Box((0.058, 0.016, 0.052)),
            origin=Origin(xyz=(x_pos, -0.028, -0.052)),
            material="tray_dark",
            name=f"tray_bridge_{index}",
        )
    _add_tray_sleeve(tray, 0, -UPRIGHT_X, "tray_dark")
    _add_tray_sleeve(tray, 1, UPRIGHT_X, "tray_dark")

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(-0.044, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="hinge_steel",
        name="hinge_knuckle_0",
    )
    rear_leg.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.044, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="hinge_steel",
        name="hinge_knuckle_1",
    )
    rear_leg.visual(
        Box((0.026, 0.014, 0.022)),
        origin=Origin(xyz=(-0.044, 0.007, -0.012)),
        material="leg_dark",
        name="knuckle_mount_0",
    )
    rear_leg.visual(
        Box((0.026, 0.014, 0.022)),
        origin=Origin(xyz=(0.044, 0.007, -0.012)),
        material="leg_dark",
        name="knuckle_mount_1",
    )
    rear_leg.visual(
        Box((0.140, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.023, -0.022)),
        material="leg_dark",
        name="yoke_block",
    )

    beam_center = (
        0.0,
        REAR_LEG_START_Y + math.sin(REAR_LEG_REST_ANGLE) * (REAR_LEG_LEN * 0.5),
        REAR_LEG_START_Z - math.cos(REAR_LEG_REST_ANGLE) * (REAR_LEG_LEN * 0.5),
    )
    foot_center = (
        0.0,
        REAR_LEG_START_Y + math.sin(REAR_LEG_REST_ANGLE) * REAR_LEG_LEN,
        REAR_LEG_START_Z - math.cos(REAR_LEG_REST_ANGLE) * REAR_LEG_LEN + 0.009,
    )
    rear_leg.visual(
        Box((0.052, 0.028, REAR_LEG_LEN)),
        origin=Origin(xyz=beam_center, rpy=(REAR_LEG_REST_ANGLE, 0.0, 0.0)),
        material="leg_dark",
        name="leg_beam",
    )
    rear_leg.visual(
        Box((0.140, 0.040, 0.018)),
        origin=Origin(xyz=foot_center),
        material="foot_black",
        name="rear_foot",
    )

    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, TRAY_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )
    model.articulation(
        "rear_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.25,
            lower=-0.28,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    tray = object_model.get_part("tray")
    rear_leg = object_model.get_part("rear_leg")
    tray_slide = object_model.get_articulation("tray_slide")
    rear_leg_hinge = object_model.get_articulation("rear_leg_hinge")

    def merged_aabb(part_name: str, elem_names: tuple[str, ...]) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
        bounds = [ctx.part_element_world_aabb(part_name, elem=elem_name) for elem_name in elem_names]
        if any(bound is None for bound in bounds):
            return None
        mins = [min(bound[0][axis] for bound in bounds if bound is not None) for axis in range(3)]
        maxs = [max(bound[1][axis] for bound in bounds if bound is not None) for axis in range(3)]
        return (tuple(mins), tuple(maxs))

    def interval_overlap(aabb_a, aabb_b, axis: int) -> float:
        return min(aabb_a[1][axis], aabb_b[1][axis]) - max(aabb_a[0][axis], aabb_b[0][axis])

    def check_tray_guidance(prefix: str) -> None:
        sleeve_groups = {
            0: ("sleeve_0_front", "sleeve_0_back", "sleeve_0_inboard", "sleeve_0_outboard"),
            1: ("sleeve_1_front", "sleeve_1_back", "sleeve_1_inboard", "sleeve_1_outboard"),
        }
        for index in (0, 1):
            upright_aabb = ctx.part_element_world_aabb("front_frame", elem=f"upright_{index}")
            sleeve_aabb = merged_aabb("tray", sleeve_groups[index])
            ok = upright_aabb is not None and sleeve_aabb is not None
            if ok:
                assert upright_aabb is not None
                assert sleeve_aabb is not None
                x_captured = sleeve_aabb[0][0] <= upright_aabb[0][0] and sleeve_aabb[1][0] >= upright_aabb[1][0]
                y_captured = sleeve_aabb[0][1] <= upright_aabb[0][1] and sleeve_aabb[1][1] >= upright_aabb[1][1]
                z_overlap = interval_overlap(upright_aabb, sleeve_aabb, 2)
                ok = x_captured and y_captured and z_overlap >= 0.145
                details = (
                    f"upright={upright_aabb}, sleeve={sleeve_aabb}, "
                    f"x_captured={x_captured}, y_captured={y_captured}, z_overlap={z_overlap:.4f}"
                )
            else:
                details = f"upright={upright_aabb}, sleeve={sleeve_aabb}"
            ctx.check(f"{prefix} sleeve_{index} captures upright_{index}", ok, details=details)

    ctx.expect_gap(
        front_frame,
        tray,
        axis="z",
        positive_elem="board_bottom",
        negative_elem="tray_front_lip",
        min_gap=0.006,
        max_gap=0.020,
        name="tray rests just below the writing board",
    )
    ctx.expect_gap(
        rear_leg,
        front_frame,
        axis="y",
        positive_elem="rear_foot",
        negative_elem="board_backing",
        min_gap=0.45,
        name="rear foot braces behind the board",
    )
    check_tray_guidance("rest")

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        check_tray_guidance("raised")
        raised_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "tray slides upward along the front frame",
        rest_tray_pos is not None
        and raised_tray_pos is not None
        and raised_tray_pos[2] > rest_tray_pos[2] + 0.45,
        details=f"rest={rest_tray_pos}, raised={raised_tray_pos}",
    )

    rear_foot_rest = ctx.part_element_world_aabb(rear_leg, elem="rear_foot")
    with ctx.pose({rear_leg_hinge: -0.24}):
        rear_foot_folded = ctx.part_element_world_aabb(rear_leg, elem="rear_foot")
    rear_rest_y = None if rear_foot_rest is None else (rear_foot_rest[0][1] + rear_foot_rest[1][1]) * 0.5
    rear_folded_y = None if rear_foot_folded is None else (rear_foot_folded[0][1] + rear_foot_folded[1][1]) * 0.5
    ctx.check(
        "rear leg folds toward the front frame",
        rear_rest_y is not None and rear_folded_y is not None and rear_folded_y < rear_rest_y - 0.20,
        details=f"rest_y={rear_rest_y}, folded_y={rear_folded_y}",
    )

    return ctx.report()


object_model = build_object_model()
