from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


PLATE_W = 0.14
PLATE_D = 0.14
PLATE_T = 0.018

STEM_R = 0.014
STEM_L = 0.028

ROOF_W = 0.20
ROOF_D = 0.12
ROOF_H = 0.026
SKIRT_W = 0.24
SKIRT_D = 0.16
SKIRT_H = 0.012

BODY_W = 0.28
BODY_D = 0.18
BODY_H = 0.29
FRAME_T = 0.018
GLASS_T = 0.004

BODY_TOP_Z = -(PLATE_T + STEM_L + ROOF_H + SKIRT_H)
BODY_BOTTOM_Z = BODY_TOP_Z - BODY_H

TOP_RING_Z = BODY_TOP_Z + (FRAME_T / 2.0)
BOTTOM_RING_Z = BODY_BOTTOM_Z + (FRAME_T / 2.0)
POST_Z = (BODY_TOP_Z + BODY_BOTTOM_Z) / 2.0
GLASS_Z = (BODY_TOP_Z + BODY_BOTTOM_Z) / 2.0
GLASS_H = BODY_H - (2.0 * FRAME_T) + 0.002

OPEN_W = BODY_W - (2.0 * FRAME_T)
OPEN_D = BODY_D - (2.0 * FRAME_T)
DOOR_W = OPEN_W - 0.004
DOOR_D = OPEN_D - 0.004
DOOR_T = 0.012
DOOR_GLASS_T = 0.004

HINGE_R = 0.006
HINGE_EMBED = 0.001
HINGE_OFFSET = HINGE_R - HINGE_EMBED
HINGE_Y = (BODY_D / 2.0) - FRAME_T - HINGE_OFFSET
HINGE_Z = BODY_BOTTOM_Z + HINGE_OFFSET

DOOR_BACK_OFFSET = HINGE_OFFSET
DOOR_TOP_OFFSET = -HINGE_OFFSET
DOOR_VISUAL_Z = DOOR_TOP_OFFSET - (DOOR_T / 2.0)

LATCH_PIVOT_LEN = 0.010
LATCH_PIVOT_R = 0.0035
LATCH_PIVOT_X = (DOOR_W / 2.0) - (FRAME_T / 2.0)
LATCH_PIVOT_Y = DOOR_BACK_OFFSET - (DOOR_D * 0.68)
LATCH_PIVOT_Z = DOOR_TOP_OFFSET - DOOR_T - (LATCH_PIVOT_LEN / 2.0)
LATCH_ARM_L = 0.022
LATCH_ARM_W = 0.008
LATCH_ARM_T = 0.004

KEEPER_X = (BODY_W / 2.0) - (FRAME_T / 2.0)
KEEPER_Y = HINGE_Y + LATCH_PIVOT_Y
KEEPER_Z = BODY_BOTTOM_Z - 0.007
KEEPER_SIZE = (0.012, 0.024, 0.016)

DOOR_OPEN_ANGLE = 1.25
LATCH_OPEN_ANGLE = 1.05


def _interval_overlap(a_min: float, a_max: float, b_min: float, b_max: float) -> float:
    return max(0.0, min(a_max, b_max) - max(a_min, b_min))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lantern_ceiling_light")

    model.material("bronze", rgba=(0.23, 0.20, 0.16, 1.0))
    model.material("bronze_dark", rgba=(0.16, 0.14, 0.12, 1.0))
    model.material("glass", rgba=(0.84, 0.88, 0.92, 0.35))
    model.material("lamp", rgba=(0.95, 0.92, 0.78, 0.85))

    housing = model.part("housing")

    housing.visual(
        Box((PLATE_W, PLATE_D, PLATE_T)),
        origin=Origin(xyz=(0.0, 0.0, -PLATE_T / 2.0)),
        material="bronze",
        name="ceiling_plate",
    )
    housing.visual(
        Cylinder(radius=STEM_R, length=STEM_L),
        origin=Origin(xyz=(0.0, 0.0, -PLATE_T - (STEM_L / 2.0))),
        material="bronze",
        name="stem",
    )
    housing.visual(
        Box((ROOF_W, ROOF_D, ROOF_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z + SKIRT_H + (ROOF_H / 2.0))),
        material="bronze",
        name="roof_cap",
    )
    housing.visual(
        Box((SKIRT_W, SKIRT_D, SKIRT_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z + (SKIRT_H / 2.0))),
        material="bronze_dark",
        name="roof_skirt",
    )

    for sign_y, name in ((-1.0, "front_top_rail"), (1.0, "rear_top_rail")):
        housing.visual(
            Box((BODY_W, FRAME_T, FRAME_T)),
            origin=Origin(xyz=(0.0, sign_y * ((BODY_D / 2.0) - (FRAME_T / 2.0)), TOP_RING_Z)),
            material="bronze",
            name=name,
        )

    for sign_x, name in ((-1.0, "side_top_rail_0"), (1.0, "side_top_rail_1")):
        housing.visual(
            Box((FRAME_T, BODY_D - (2.0 * FRAME_T), FRAME_T)),
            origin=Origin(xyz=(sign_x * ((BODY_W / 2.0) - (FRAME_T / 2.0)), 0.0, TOP_RING_Z)),
            material="bronze",
            name=name,
        )

    for sign_y, name in ((-1.0, "front_bottom_rail"), (1.0, "rear_bottom_rail")):
        housing.visual(
            Box((BODY_W, FRAME_T, FRAME_T)),
            origin=Origin(
                xyz=(0.0, sign_y * ((BODY_D / 2.0) - (FRAME_T / 2.0)), BOTTOM_RING_Z)
            ),
            material="bronze",
            name=name,
        )

    for sign_x, name in ((-1.0, "side_bottom_rail_0"), (1.0, "side_bottom_rail_1")):
        housing.visual(
            Box((FRAME_T, BODY_D - (2.0 * FRAME_T), FRAME_T)),
            origin=Origin(xyz=(sign_x * ((BODY_W / 2.0) - (FRAME_T / 2.0)), 0.0, BOTTOM_RING_Z)),
            material="bronze",
            name=name,
        )

    post_index = 0
    for sign_x in (-1.0, 1.0):
        for sign_y in (-1.0, 1.0):
            housing.visual(
                Box((FRAME_T, FRAME_T, BODY_H)),
                origin=Origin(
                    xyz=(
                        sign_x * ((BODY_W / 2.0) - (FRAME_T / 2.0)),
                        sign_y * ((BODY_D / 2.0) - (FRAME_T / 2.0)),
                        POST_Z,
                    )
                ),
                material="bronze",
                name=f"post_{post_index}",
            )
            post_index += 1

    for sign_y, name in ((-1.0, "front_glass"), (1.0, "rear_glass")):
        housing.visual(
            Box((BODY_W - (2.0 * FRAME_T) + 0.002, GLASS_T, GLASS_H)),
            origin=Origin(
                xyz=(
                    0.0,
                    sign_y * (((BODY_D / 2.0) - FRAME_T) - (GLASS_T / 2.0) + 0.001),
                    GLASS_Z,
                )
            ),
            material="glass",
            name=name,
        )

    for sign_x, name in ((-1.0, "side_glass_0"), (1.0, "side_glass_1")):
        housing.visual(
            Box((GLASS_T, BODY_D - (2.0 * FRAME_T) + 0.002, GLASS_H)),
            origin=Origin(
                xyz=(
                    sign_x * (((BODY_W / 2.0) - FRAME_T) - (GLASS_T / 2.0) + 0.001),
                    0.0,
                    GLASS_Z,
                )
            ),
            material="glass",
            name=name,
        )

    lamp_stem_len = BODY_H * 0.48
    lamp_stem_center_z = BODY_TOP_Z - (lamp_stem_len / 2.0)
    housing.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z - 0.012)),
        material="bronze_dark",
        name="lamp_socket",
    )
    housing.visual(
        Cylinder(radius=0.008, length=lamp_stem_len),
        origin=Origin(xyz=(0.0, 0.0, lamp_stem_center_z)),
        material="bronze_dark",
        name="lamp_stem",
    )
    lamp_globe_z = lamp_stem_center_z - (lamp_stem_len / 2.0) - 0.028
    housing.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.0, 0.0, lamp_globe_z)),
        material="lamp",
        name="lamp_globe",
    )

    housing.visual(
        Cylinder(radius=HINGE_R, length=0.082),
        origin=Origin(
            xyz=(0.0, HINGE_Y, HINGE_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="bronze_dark",
        name="hinge_barrel",
    )
    housing.visual(
        Box(KEEPER_SIZE),
        origin=Origin(xyz=(KEEPER_X, KEEPER_Y, KEEPER_Z)),
        material="bronze_dark",
        name="keeper",
    )

    door = model.part("door")

    door.visual(
        Box((DOOR_W, FRAME_T, DOOR_T)),
        origin=Origin(xyz=(0.0, DOOR_BACK_OFFSET - (FRAME_T / 2.0), DOOR_VISUAL_Z)),
        material="bronze",
        name="hinge_rail",
    )
    door.visual(
        Box((DOOR_W, FRAME_T, DOOR_T)),
        origin=Origin(xyz=(0.0, DOOR_BACK_OFFSET - DOOR_D + (FRAME_T / 2.0), DOOR_VISUAL_Z)),
        material="bronze",
        name="free_rail",
    )
    door.visual(
        Box((FRAME_T, DOOR_D - (2.0 * FRAME_T), DOOR_T)),
        origin=Origin(xyz=(-(DOOR_W / 2.0) + (FRAME_T / 2.0), DOOR_BACK_OFFSET - (DOOR_D / 2.0), DOOR_VISUAL_Z)),
        material="bronze",
        name="side_rail_0",
    )
    door.visual(
        Box((FRAME_T, DOOR_D - (2.0 * FRAME_T), DOOR_T)),
        origin=Origin(xyz=((DOOR_W / 2.0) - (FRAME_T / 2.0), DOOR_BACK_OFFSET - (DOOR_D / 2.0), DOOR_VISUAL_Z)),
        material="bronze",
        name="side_rail_1",
    )
    door.visual(
        Box((DOOR_W - (2.0 * FRAME_T) + 0.002, DOOR_D - (2.0 * FRAME_T) + 0.002, DOOR_GLASS_T)),
        origin=Origin(
            xyz=(
                0.0,
                DOOR_BACK_OFFSET - (DOOR_D / 2.0),
                DOOR_TOP_OFFSET - (DOOR_GLASS_T / 2.0),
            )
        ),
        material="glass",
        name="pane",
    )

    for x_pos, name in ((-0.081, "hinge_knuckle_0"), (0.081, "hinge_knuckle_1")):
        door.visual(
            Cylinder(radius=HINGE_R, length=0.050),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="bronze_dark",
            name=name,
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=LATCH_PIVOT_R, length=LATCH_PIVOT_LEN),
        origin=Origin(),
        material="bronze_dark",
        name="pivot",
    )
    latch.visual(
        Box((LATCH_ARM_L, LATCH_ARM_W, LATCH_ARM_T)),
        origin=Origin(xyz=(LATCH_ARM_L / 2.0, 0.0, 0.0)),
        material="bronze_dark",
        name="arm",
    )
    latch.visual(
        Box((0.006, 0.014, LATCH_ARM_T)),
        origin=Origin(xyz=(LATCH_ARM_L + 0.003, 0.0, -0.001)),
        material="bronze_dark",
        name="tip",
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=DOOR_OPEN_ANGLE,
            effort=8.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(LATCH_PIVOT_X, LATCH_PIVOT_Y, LATCH_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LATCH_OPEN_ANGLE,
            effort=1.0,
            velocity=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_joint = object_model.get_articulation("housing_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    with ctx.pose({door_joint: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            housing,
            door,
            axis="z",
            positive_elem="front_bottom_rail",
            negative_elem="free_rail",
            max_gap=0.0015,
            max_penetration=0.0,
            name="door closes flush beneath the front frame rail",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="xy",
            min_overlap=0.12,
            name="door covers the bottom opening footprint",
        )
        ctx.expect_overlap(
            latch,
            housing,
            axes="xy",
            elem_a="arm",
            elem_b="keeper",
            min_overlap=0.006,
            name="latch arm sits under the keeper footprint",
        )
        ctx.expect_gap(
            housing,
            latch,
            axis="z",
            positive_elem="keeper",
            negative_elem="arm",
            max_gap=0.002,
            max_penetration=0.0,
            name="keeper hovers just above the latch arm",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: DOOR_OPEN_ANGLE}):
        open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door swings downward on the long hinge edge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < closed_aabb[0][2] - 0.10,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    with ctx.pose({door_joint: 0.0, latch_joint: 0.0}):
        closed_tab = ctx.part_element_world_aabb(latch, elem="arm")
        keeper = ctx.part_element_world_aabb(housing, elem="keeper")
    with ctx.pose({door_joint: 0.0, latch_joint: LATCH_OPEN_ANGLE}):
        rotated_tab = ctx.part_element_world_aabb(latch, elem="arm")

    closed_xy_overlap = None
    rotated_xy_overlap = None
    if closed_tab is not None and keeper is not None:
        closed_xy_overlap = (
            _interval_overlap(closed_tab[0][0], closed_tab[1][0], keeper[0][0], keeper[1][0]),
            _interval_overlap(closed_tab[0][1], closed_tab[1][1], keeper[0][1], keeper[1][1]),
        )
    if rotated_tab is not None and keeper is not None:
        rotated_xy_overlap = (
            _interval_overlap(rotated_tab[0][0], rotated_tab[1][0], keeper[0][0], keeper[1][0]),
            _interval_overlap(rotated_tab[0][1], rotated_tab[1][1], keeper[0][1], keeper[1][1]),
        )

    ctx.check(
        "latch rotates away from the keeper",
        closed_xy_overlap is not None
        and rotated_xy_overlap is not None
        and (
            rotated_xy_overlap[0] < (closed_xy_overlap[0] * 0.6)
            or rotated_xy_overlap[1] < (closed_xy_overlap[1] * 0.6)
        ),
        details=(
            f"closed_xy_overlap={closed_xy_overlap}, "
            f"rotated_xy_overlap={rotated_xy_overlap}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
