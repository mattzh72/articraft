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


CABINET_L = 1.48
CABINET_D = 0.84
CABINET_H = 0.94
PLINTH_H = 0.10
OUTER_T = 0.045
LINER_T = 0.005
COLLAR_T = 0.040
INNER_FLOOR_T = 0.032
INNER_FLOOR_TOP = 0.175
CAVITY_L = 1.30
CAVITY_D = 0.66

TRACK_X = CAVITY_L
HIGH_TRACK_T = 0.014
LOW_TRACK_T = 0.014
TRACK_W = 0.018
HIGH_TRACK_TOP = 0.914
LOW_TRACK_TOP = 0.914
FRONT_TRACK_Y = 0.315
REAR_TRACK_Y = -0.315
FRONT_CENTER_TRACK_Y = 0.055
REAR_CENTER_TRACK_Y = -0.055

LID_L = 0.72
LID_D = 0.30
LID_BORDER = 0.028
LID_FRAME_T = 0.008
LID_GLASS_T = 0.004
LID_RUNNER_T = 0.006
LID_TRAVEL = 0.50
LID_REST_X = 0.25
FRONT_LID_Y = 0.185
REAR_LID_Y = -0.185
FRONT_LID_Z = HIGH_TRACK_TOP + LID_RUNNER_T
REAR_LID_Z = LOW_TRACK_TOP + LID_RUNNER_T

LOCK_Y = 0.19
LOCK_Z = 0.57
LOCK_FLAP_W = 0.060
LOCK_FLAP_H = 0.095


def _add_slider_lid(
    model: ArticulatedObject,
    *,
    name: str,
    frame_material,
    glass_material,
    handle_material,
    track_y: float,
    center_track_y: float,
    handle_x: float,
) -> object:
    lid = model.part(name)

    lid.visual(
        Box((LID_L, LID_BORDER, LID_FRAME_T)),
        origin=Origin(xyz=(0.0, (LID_D * 0.5) - (LID_BORDER * 0.5), 0.004)),
        material=frame_material,
        name="front_frame",
    )
    lid.visual(
        Box((LID_L, LID_BORDER, LID_FRAME_T)),
        origin=Origin(xyz=(0.0, -(LID_D * 0.5) + (LID_BORDER * 0.5), 0.004)),
        material=frame_material,
        name="rear_frame",
    )
    lid.visual(
        Box((LID_BORDER, LID_D, LID_FRAME_T)),
        origin=Origin(xyz=(-(LID_L * 0.5) + (LID_BORDER * 0.5), 0.0, 0.004)),
        material=frame_material,
        name="left_frame",
    )
    lid.visual(
        Box((LID_BORDER, LID_D, LID_FRAME_T)),
        origin=Origin(xyz=((LID_L * 0.5) - (LID_BORDER * 0.5), 0.0, 0.004)),
        material=frame_material,
        name="right_frame",
    )
    lid.visual(
        Box((LID_L - (2.0 * LID_BORDER) + 0.004, LID_D - (2.0 * LID_BORDER) + 0.004, LID_GLASS_T)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=glass_material,
        name="glass",
    )
    lid.visual(
        Box((LID_L - 0.020, TRACK_W, LID_RUNNER_T)),
        origin=Origin(xyz=(0.0, track_y, -0.003)),
        material=frame_material,
        name="outer_runner",
    )
    lid.visual(
        Box((LID_L - 0.020, TRACK_W, LID_RUNNER_T)),
        origin=Origin(xyz=(0.0, center_track_y, -0.003)),
        material=frame_material,
        name="center_runner",
    )
    lid.visual(
        Box((0.060, 0.092, 0.010)),
        origin=Origin(xyz=(handle_x, 0.0, 0.011)),
        material=handle_material,
        name="pull",
    )

    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convenience_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.94, 0.95, 0.96, 1.0))
    plinth_grey = model.material("plinth_grey", rgba=(0.34, 0.36, 0.39, 1.0))
    liner_white = model.material("liner_white", rgba=(0.97, 0.98, 0.98, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.67, 0.69, 0.72, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.67, 0.84, 0.92, 0.30))
    handle_black = model.material("handle_black", rgba=(0.12, 0.13, 0.14, 1.0))
    lock_metal = model.material("lock_metal", rgba=(0.72, 0.74, 0.77, 1.0))

    cabinet = model.part("cabinet")

    wall_height = CABINET_H - PLINTH_H
    wall_center_z = PLINTH_H + (wall_height * 0.5)
    inner_wall_height = (CABINET_H - COLLAR_T) - INNER_FLOOR_TOP
    inner_wall_center_z = INNER_FLOOR_TOP + (inner_wall_height * 0.5)

    cabinet.visual(
        Box((CABINET_L, CABINET_D, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H * 0.5)),
        material=plinth_grey,
        name="plinth",
    )
    cabinet.visual(
        Box((CABINET_L, OUTER_T, wall_height)),
        origin=Origin(xyz=(0.0, (CABINET_D * 0.5) - (OUTER_T * 0.5), wall_center_z)),
        material=cabinet_white,
        name="front_wall",
    )
    cabinet.visual(
        Box((CABINET_L, OUTER_T, wall_height)),
        origin=Origin(xyz=(0.0, -(CABINET_D * 0.5) + (OUTER_T * 0.5), wall_center_z)),
        material=cabinet_white,
        name="rear_wall",
    )
    cabinet.visual(
        Box((OUTER_T, CABINET_D, wall_height)),
        origin=Origin(xyz=((CABINET_L * 0.5) - (OUTER_T * 0.5), 0.0, wall_center_z)),
        material=cabinet_white,
        name="side_wall",
    )
    cabinet.visual(
        Box((OUTER_T, CABINET_D, wall_height)),
        origin=Origin(xyz=(-(CABINET_L * 0.5) + (OUTER_T * 0.5), 0.0, wall_center_z)),
        material=cabinet_white,
        name="end_wall",
    )

    cabinet.visual(
        Box((CAVITY_L + (2.0 * LINER_T), CAVITY_D + (2.0 * LINER_T), INNER_FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, INNER_FLOOR_TOP - (INNER_FLOOR_T * 0.5))),
        material=liner_white,
        name="liner_floor",
    )
    cabinet.visual(
        Box((CAVITY_L, LINER_T, inner_wall_height)),
        origin=Origin(xyz=(0.0, (CAVITY_D * 0.5) + (LINER_T * 0.5), inner_wall_center_z)),
        material=liner_white,
        name="liner_front",
    )
    cabinet.visual(
        Box((CAVITY_L, LINER_T, inner_wall_height)),
        origin=Origin(xyz=(0.0, -(CAVITY_D * 0.5) - (LINER_T * 0.5), inner_wall_center_z)),
        material=liner_white,
        name="liner_rear",
    )
    cabinet.visual(
        Box((LINER_T, CAVITY_D + (2.0 * LINER_T), inner_wall_height)),
        origin=Origin(xyz=((CAVITY_L * 0.5) + (LINER_T * 0.5), 0.0, inner_wall_center_z)),
        material=liner_white,
        name="liner_side",
    )
    cabinet.visual(
        Box((LINER_T, CAVITY_D + (2.0 * LINER_T), inner_wall_height)),
        origin=Origin(xyz=(-(CAVITY_L * 0.5) - (LINER_T * 0.5), 0.0, inner_wall_center_z)),
        material=liner_white,
        name="liner_end",
    )

    collar_gap = ((CABINET_D - (2.0 * OUTER_T)) - (CAVITY_D + (2.0 * LINER_T))) * 0.5
    side_gap = ((CABINET_L - (2.0 * OUTER_T)) - (CAVITY_L + (2.0 * LINER_T))) * 0.5
    collar_z = CABINET_H - (COLLAR_T * 0.5)

    cabinet.visual(
        Box((CABINET_L - (2.0 * OUTER_T), collar_gap, COLLAR_T)),
        origin=Origin(
            xyz=(
                0.0,
                ((CABINET_D * 0.5) - OUTER_T + (CAVITY_D * 0.5) + LINER_T) * 0.5,
                collar_z,
            )
        ),
        material=cabinet_white,
        name="front_collar",
    )
    cabinet.visual(
        Box((CABINET_L - (2.0 * OUTER_T), collar_gap, COLLAR_T)),
        origin=Origin(
            xyz=(
                0.0,
                -((CABINET_D * 0.5) - OUTER_T + (CAVITY_D * 0.5) + LINER_T) * 0.5,
                collar_z,
            )
        ),
        material=cabinet_white,
        name="rear_collar",
    )
    cabinet.visual(
        Box((side_gap, CABINET_D - (2.0 * OUTER_T), COLLAR_T)),
        origin=Origin(
            xyz=(
                ((CABINET_L * 0.5) - OUTER_T + (CAVITY_L * 0.5) + LINER_T) * 0.5,
                0.0,
                collar_z,
            )
        ),
        material=cabinet_white,
        name="side_collar",
    )
    cabinet.visual(
        Box((side_gap, CABINET_D - (2.0 * OUTER_T), COLLAR_T)),
        origin=Origin(
            xyz=(
                -((CABINET_L * 0.5) - OUTER_T + (CAVITY_L * 0.5) + LINER_T) * 0.5,
                0.0,
                collar_z,
            )
        ),
        material=cabinet_white,
        name="end_collar",
    )

    cabinet.visual(
        Box((TRACK_X, TRACK_W, HIGH_TRACK_T)),
        origin=Origin(xyz=(0.0, FRONT_TRACK_Y, HIGH_TRACK_TOP - (HIGH_TRACK_T * 0.5))),
        material=rail_aluminum,
        name="front_track",
    )
    cabinet.visual(
        Box((TRACK_X, TRACK_W, LOW_TRACK_T)),
        origin=Origin(xyz=(0.0, REAR_TRACK_Y, LOW_TRACK_TOP - (LOW_TRACK_T * 0.5))),
        material=rail_aluminum,
        name="rear_track",
    )
    cabinet.visual(
        Box((TRACK_X, TRACK_W, HIGH_TRACK_T)),
        origin=Origin(
            xyz=(0.0, FRONT_CENTER_TRACK_Y, HIGH_TRACK_TOP - (HIGH_TRACK_T * 0.5))
        ),
        material=rail_aluminum,
        name="front_center_track",
    )
    cabinet.visual(
        Box((TRACK_X, TRACK_W, LOW_TRACK_T)),
        origin=Origin(
            xyz=(0.0, REAR_CENTER_TRACK_Y, LOW_TRACK_TOP - (LOW_TRACK_T * 0.5))
        ),
        material=rail_aluminum,
        name="rear_center_track",
    )

    cabinet.visual(
        Box((0.008, 0.068, 0.092)),
        origin=Origin(xyz=((CABINET_L * 0.5) + 0.004, LOCK_Y, LOCK_Z)),
        material=lock_metal,
        name="key_escutcheon",
    )
    cabinet.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(
            xyz=((CABINET_L * 0.5) + 0.011, LOCK_Y, LOCK_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=lock_metal,
        name="key_cylinder",
    )
    cabinet.visual(
        Box((0.006, 0.032, 0.014)),
        origin=Origin(xyz=((CABINET_L * 0.5) + 0.003, LOCK_Y, LOCK_Z + (LOCK_FLAP_H * 0.5))),
        material=lock_metal,
        name="hinge_mount",
    )

    front_lid = _add_slider_lid(
        model,
        name="front_lid",
        frame_material=rail_aluminum,
        glass_material=glass_tint,
        handle_material=handle_black,
        track_y=FRONT_TRACK_Y - FRONT_LID_Y,
        center_track_y=FRONT_CENTER_TRACK_Y - FRONT_LID_Y,
        handle_x=LID_L * 0.5 - 0.055,
    )
    rear_lid = _add_slider_lid(
        model,
        name="rear_lid",
        frame_material=rail_aluminum,
        glass_material=glass_tint,
        handle_material=handle_black,
        track_y=REAR_TRACK_Y - REAR_LID_Y,
        center_track_y=REAR_CENTER_TRACK_Y - REAR_LID_Y,
        handle_x=-(LID_L * 0.5 - 0.055),
    )

    lock_flap = model.part("lock_flap")
    lock_flap.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=lock_metal,
        name="hinge_barrel",
    )
    lock_flap.visual(
        Box((0.014, 0.030, 0.012)),
        origin=Origin(xyz=(0.007, 0.0, -0.006)),
        material=cabinet_white,
        name="hinge_leaf",
    )
    lock_flap.visual(
        Box((0.008, LOCK_FLAP_W, LOCK_FLAP_H)),
        origin=Origin(xyz=(0.013, 0.0, -(LOCK_FLAP_H * 0.5))),
        material=cabinet_white,
        name="cover",
    )
    lock_flap.visual(
        Box((0.012, 0.018, 0.010)),
        origin=Origin(xyz=(0.015, 0.0, -LOCK_FLAP_H + 0.018)),
        material=handle_black,
        name="tab",
    )

    model.articulation(
        "front_lid_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=front_lid,
        origin=Origin(xyz=(-LID_REST_X, FRONT_LID_Y, FRONT_LID_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.35,
            lower=0.0,
            upper=LID_TRAVEL,
        ),
    )
    model.articulation(
        "rear_lid_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=rear_lid,
        origin=Origin(xyz=(LID_REST_X, REAR_LID_Y, REAR_LID_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.35,
            lower=0.0,
            upper=LID_TRAVEL,
        ),
    )
    model.articulation(
        "lock_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lock_flap,
        origin=Origin(
            xyz=(
                (CABINET_L * 0.5) + 0.012,
                LOCK_Y,
                LOCK_Z + (LOCK_FLAP_H * 0.5),
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    front_lid = object_model.get_part("front_lid")
    rear_lid = object_model.get_part("rear_lid")
    lock_flap = object_model.get_part("lock_flap")

    front_slide = object_model.get_articulation("front_lid_slide")
    rear_slide = object_model.get_articulation("rear_lid_slide")
    flap_hinge = object_model.get_articulation("lock_flap_hinge")

    front_limits = front_slide.motion_limits
    rear_limits = rear_slide.motion_limits
    flap_limits = flap_hinge.motion_limits

    with ctx.pose({front_slide: 0.0, rear_slide: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            front_lid,
            cabinet,
            axis="z",
            positive_elem="outer_runner",
            negative_elem="front_track",
            max_gap=0.001,
            max_penetration=0.0,
            name="front lid outer runner seats on front track",
        )
        ctx.expect_gap(
            front_lid,
            cabinet,
            axis="z",
            positive_elem="center_runner",
            negative_elem="front_center_track",
            max_gap=0.001,
            max_penetration=0.0,
            name="front lid center runner seats on center track",
        )
        ctx.expect_gap(
            rear_lid,
            cabinet,
            axis="z",
            positive_elem="outer_runner",
            negative_elem="rear_track",
            max_gap=0.001,
            max_penetration=0.0,
            name="rear lid outer runner seats on rear track",
        )
        ctx.expect_gap(
            rear_lid,
            cabinet,
            axis="z",
            positive_elem="center_runner",
            negative_elem="rear_center_track",
            max_gap=0.001,
            max_penetration=0.0,
            name="rear lid center runner seats on center track",
        )
        ctx.expect_overlap(
            lock_flap,
            cabinet,
            axes="yz",
            elem_a="cover",
            elem_b="key_escutcheon",
            min_overlap=0.045,
            name="lock flap covers the side key area when closed",
        )
        ctx.expect_gap(
            lock_flap,
            cabinet,
            axis="x",
            positive_elem="cover",
            negative_elem="key_cylinder",
            min_gap=0.003,
            max_gap=0.018,
            name="closed lock flap clears the key cylinder",
        )

        front_rest = ctx.part_world_position(front_lid)
        rear_rest = ctx.part_world_position(rear_lid)
        closed_cover = ctx.part_element_world_aabb(lock_flap, elem="cover")

    if front_limits is not None and front_limits.upper is not None:
        with ctx.pose({front_slide: front_limits.upper, rear_slide: 0.0, flap_hinge: 0.0}):
            ctx.expect_overlap(
                front_lid,
                cabinet,
                axes="x",
                elem_a="outer_runner",
                elem_b="front_track",
                min_overlap=0.25,
                name="front lid retains long track engagement at full travel",
            )
            front_open = ctx.part_world_position(front_lid)
        ctx.check(
            "front lid slides toward positive x",
            front_rest is not None
            and front_open is not None
            and front_open[0] > front_rest[0] + 0.20,
            details=f"rest={front_rest}, open={front_open}",
        )

    if rear_limits is not None and rear_limits.upper is not None:
        with ctx.pose({front_slide: 0.0, rear_slide: rear_limits.upper, flap_hinge: 0.0}):
            ctx.expect_overlap(
                rear_lid,
                cabinet,
                axes="x",
                elem_a="outer_runner",
                elem_b="rear_track",
                min_overlap=0.25,
                name="rear lid retains long track engagement at full travel",
            )
            rear_open = ctx.part_world_position(rear_lid)
        ctx.check(
            "rear lid slides toward negative x",
            rear_rest is not None
            and rear_open is not None
            and rear_open[0] < rear_rest[0] - 0.20,
            details=f"rest={rear_rest}, open={rear_open}",
        )

    if flap_limits is not None and flap_limits.upper is not None:
        with ctx.pose({front_slide: 0.0, rear_slide: 0.0, flap_hinge: flap_limits.upper}):
            open_cover = ctx.part_element_world_aabb(lock_flap, elem="cover")
        ctx.check(
            "lock flap swings outward from the side wall",
            closed_cover is not None
            and open_cover is not None
            and open_cover[1][0] > closed_cover[1][0] + 0.035,
            details=f"closed={closed_cover}, open={open_cover}",
        )

    return ctx.report()


object_model = build_object_model()
