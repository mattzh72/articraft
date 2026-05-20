from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CASE_W = 0.038
CASE_D = 0.014
BODY_H = 0.039
LID_H = 0.018
WALL = 0.0008
CORNER_R = 0.0012

HINGE_R = 0.0011
HINGE_Z = 0.0026
BODY_KNUCKLE_LEN = 0.0038
LID_KNUCKLE_LEN = 0.0039
HINGE_GAP_Y = 0.0042
HINGE_PAD_W = 0.0021
HINGE_PAD_H = 0.0060
HINGE_PAD_D = 0.0105

INSERT_W = CASE_W - 0.0040
INSERT_D = CASE_D - 0.0032
INSERT_BODY_H = 0.024
CHIMNEY_W = 0.015
CHIMNEY_D = 0.0082
CHIMNEY_H = 0.011
CHIMNEY_WALL = 0.0007
INSERT_SEAT_Z = WALL

WHEEL_R = 0.0030
WHEEL_LEN = 0.0100
EAR_T = 0.0014
EAR_D = 0.0076
EAR_H = 0.0086
WHEEL_Z = INSERT_BODY_H + 0.0068

CAM_PIVOT_X = 0.0042
CAM_PIVOT_Z = 0.0042
CAM_HUB_R = 0.0011
CAM_HUB_LEN = 0.0034


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zippo_style_lighter")

    model.material("brushed_shell", rgba=(0.73, 0.74, 0.76, 1.0))
    model.material("stainless_insert", rgba=(0.63, 0.64, 0.66, 1.0))
    model.material("wheel_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    model.material("wick", rgba=(0.85, 0.82, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        Box((CASE_W, CASE_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material="brushed_shell",
        name="body_bottom",
    )
    body.visual(
        Box((CASE_W, WALL, BODY_H)),
        origin=Origin(xyz=(0.0, (CASE_D / 2.0) - (WALL / 2.0), BODY_H / 2.0)),
        material="brushed_shell",
        name="body_front_wall",
    )
    body.visual(
        Box((CASE_W, WALL, BODY_H)),
        origin=Origin(xyz=(0.0, -(CASE_D / 2.0) + (WALL / 2.0), BODY_H / 2.0)),
        material="brushed_shell",
        name="body_rear_wall",
    )
    body.visual(
        Box((WALL, CASE_D - (2.0 * WALL), BODY_H)),
        origin=Origin(xyz=((CASE_W / 2.0) - (WALL / 2.0), 0.0, BODY_H / 2.0)),
        material="brushed_shell",
        name="body_right_wall",
    )
    body.visual(
        Box((WALL, CASE_D - (2.0 * WALL), BODY_H)),
        origin=Origin(xyz=(-(CASE_W / 2.0) + (WALL / 2.0), 0.0, BODY_H / 2.0)),
        material="brushed_shell",
        name="body_left_wall",
    )
    for idx, (x_pos, y_pos) in enumerate(
        (
            (-(CASE_W / 2.0) + CORNER_R, -(CASE_D / 2.0) + CORNER_R),
            (-(CASE_W / 2.0) + CORNER_R, (CASE_D / 2.0) - CORNER_R),
            ((CASE_W / 2.0) - CORNER_R, -(CASE_D / 2.0) + CORNER_R),
            ((CASE_W / 2.0) - CORNER_R, (CASE_D / 2.0) - CORNER_R),
        )
    ):
        body.visual(
            Cylinder(radius=CORNER_R, length=BODY_H),
            origin=Origin(xyz=(x_pos, y_pos, BODY_H / 2.0)),
            material="brushed_shell",
            name=f"body_corner_{idx}",
        )
    body.visual(
        Box((HINGE_PAD_W, HINGE_PAD_D, HINGE_PAD_H)),
        origin=Origin(
            xyz=(
                -CASE_W / 2.0 + HINGE_PAD_W / 2.0,
                0.0,
                BODY_H + HINGE_Z - (HINGE_PAD_H / 2.0),
            )
        ),
        material="brushed_shell",
        name="body_hinge_pad",
    )
    for idx, y_pos in enumerate((-HINGE_GAP_Y, HINGE_GAP_Y)):
        body.visual(
            Cylinder(radius=HINGE_R, length=BODY_KNUCKLE_LEN),
            origin=Origin(
                xyz=(-CASE_W / 2.0, y_pos, BODY_H + HINGE_Z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material="brushed_shell",
            name=f"body_knuckle_{idx}",
        )

    insert = model.part("insert")
    insert.visual(
        Box((INSERT_W, INSERT_D, INSERT_BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, INSERT_BODY_H / 2.0)),
        material="stainless_insert",
        name="insert_body",
    )
    insert.visual(
        Box((CHIMNEY_WALL, CHIMNEY_D, CHIMNEY_H)),
        origin=Origin(
            xyz=(
                -(CHIMNEY_W / 2.0) + (CHIMNEY_WALL / 2.0),
                0.0,
                INSERT_BODY_H + (CHIMNEY_H / 2.0),
            )
        ),
        material="stainless_insert",
        name="chimney_left",
    )
    insert.visual(
        Box((CHIMNEY_WALL, CHIMNEY_D, CHIMNEY_H)),
        origin=Origin(
            xyz=(
                (CHIMNEY_W / 2.0) - (CHIMNEY_WALL / 2.0),
                0.0,
                INSERT_BODY_H + (CHIMNEY_H / 2.0),
            )
        ),
        material="stainless_insert",
        name="chimney_right",
    )
    insert.visual(
        Box((CHIMNEY_W - (2.0 * CHIMNEY_WALL), CHIMNEY_WALL, CHIMNEY_H)),
        origin=Origin(
            xyz=(
                0.0,
                -(CHIMNEY_D / 2.0) + (CHIMNEY_WALL / 2.0),
                INSERT_BODY_H + (CHIMNEY_H / 2.0),
            )
        ),
        material="stainless_insert",
        name="chimney_rear",
    )
    insert.visual(
        Box((CHIMNEY_W - (2.0 * CHIMNEY_WALL), CHIMNEY_WALL, CHIMNEY_H)),
        origin=Origin(
            xyz=(
                0.0,
                (CHIMNEY_D / 2.0) - (CHIMNEY_WALL / 2.0),
                INSERT_BODY_H + (CHIMNEY_H / 2.0),
            )
        ),
        material="stainless_insert",
        name="chimney_front",
    )
    for idx, sign in enumerate((-1.0, 1.0)):
        insert.visual(
            Box((EAR_T, EAR_D, EAR_H)),
            origin=Origin(
                xyz=(
                    sign * ((WHEEL_LEN / 2.0) + (EAR_T / 2.0)),
                    0.0,
                    WHEEL_Z,
                )
            ),
            material="stainless_insert",
            name=f"ear_{idx}",
        )
    insert.visual(
        Cylinder(radius=0.0011, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, INSERT_BODY_H + 0.0025)),
        material="wick",
        name="wick",
    )

    wheel = model.part("spark_wheel")
    wheel.visual(
        Cylinder(radius=WHEEL_R, length=WHEEL_LEN),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="wheel_dark",
        name="wheel_body",
    )

    lid = model.part("lid")
    lid.visual(
        Box((CASE_W, CASE_D, WALL)),
        origin=Origin(
            xyz=(CASE_W / 2.0, 0.0, LID_H - HINGE_Z - (WALL / 2.0))
        ),
        material="brushed_shell",
        name="lid_top",
    )
    lid.visual(
        Box((CASE_W, WALL, LID_H)),
        origin=Origin(
            xyz=(CASE_W / 2.0, (CASE_D / 2.0) - (WALL / 2.0), (LID_H / 2.0) - HINGE_Z)
        ),
        material="brushed_shell",
        name="lid_front_wall",
    )
    lid.visual(
        Box((CASE_W, WALL, LID_H)),
        origin=Origin(
            xyz=(CASE_W / 2.0, -(CASE_D / 2.0) + (WALL / 2.0), (LID_H / 2.0) - HINGE_Z)
        ),
        material="brushed_shell",
        name="lid_rear_wall",
    )
    lid.visual(
        Box((WALL, CASE_D - (2.0 * WALL), LID_H)),
        origin=Origin(
            xyz=(CASE_W - (WALL / 2.0), 0.0, (LID_H / 2.0) - HINGE_Z)
        ),
        material="brushed_shell",
        name="lid_right_wall",
    )
    lid.visual(
        Box((WALL, CASE_D - (2.0 * WALL), LID_H)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, (LID_H / 2.0) - HINGE_Z)),
        material="brushed_shell",
        name="lid_left_wall",
    )
    for idx, (x_pos, y_pos) in enumerate(
        (
            (CORNER_R, -(CASE_D / 2.0) + CORNER_R),
            (CORNER_R, (CASE_D / 2.0) - CORNER_R),
            (CASE_W - CORNER_R, -(CASE_D / 2.0) + CORNER_R),
            (CASE_W - CORNER_R, (CASE_D / 2.0) - CORNER_R),
        )
    ):
        lid.visual(
            Cylinder(radius=CORNER_R, length=LID_H),
            origin=Origin(xyz=(x_pos, y_pos, (LID_H / 2.0) - HINGE_Z)),
            material="brushed_shell",
            name=f"lid_corner_{idx}",
        )
    lid.visual(
        Box((HINGE_PAD_W, LID_KNUCKLE_LEN + 0.0012, HINGE_PAD_H)),
        origin=Origin(
            xyz=(HINGE_PAD_W / 2.0, 0.0, HINGE_Z - (HINGE_PAD_H / 2.0) + 0.0016)
        ),
        material="brushed_shell",
        name="lid_hinge_pad",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=LID_KNUCKLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_shell",
        name="lid_knuckle",
    )
    lid.visual(
        Box((0.0031, 0.0044, 0.0052)),
        origin=Origin(xyz=(0.00155, 0.0, CAM_PIVOT_Z)),
        material="brushed_shell",
        name="cam_boss",
    )

    cam = model.part("cam")
    cam.visual(
        Cylinder(radius=CAM_HUB_R, length=CAM_HUB_LEN),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="stainless_insert",
        name="cam_hub",
    )
    cam.visual(
        Box((0.0105, 0.0026, 0.0020)),
        origin=Origin(xyz=(0.0052, 0.0, 0.0004)),
        material="stainless_insert",
        name="cam_arm",
    )
    cam.visual(
        Box((0.0038, 0.0026, 0.0046)),
        origin=Origin(xyz=(0.0086, 0.0, -0.0013)),
        material="stainless_insert",
        name="cam_toe",
    )

    model.articulation(
        "body_to_insert",
        ArticulationType.FIXED,
        parent=body,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, INSERT_SEAT_Z)),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.1, velocity=20.0),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-CASE_W / 2.0, 0.0, BODY_H + HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.95, effort=2.0, velocity=5.0),
    )
    model.articulation(
        "lid_to_cam",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cam,
        origin=Origin(xyz=(CAM_PIVOT_X, 0.0, CAM_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.85, effort=0.2, velocity=10.0),
        mimic=Mimic(joint="lid_hinge", multiplier=0.55, offset=-0.24),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    insert = object_model.get_part("insert")
    wheel = object_model.get_part("spark_wheel")
    lid = object_model.get_part("lid")
    cam = object_model.get_part("cam")
    lid_hinge = object_model.get_articulation("lid_hinge")

    ctx.expect_within(
        insert,
        body,
        axes="xy",
        margin=0.0009,
        name="insert stays inside the case footprint",
    )
    ctx.expect_overlap(
        insert,
        body,
        axes="z",
        min_overlap=0.020,
        name="insert remains deeply seated in the lower shell",
    )
    ctx.expect_contact(
        insert,
        body,
        name="insert contacts the body shell floor",
    )
    ctx.expect_contact(
        wheel,
        insert,
        name="spark wheel is supported by the insert ears",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.012,
            name="closed lid covers the lighter body",
        )

        closed_lid_center = _aabb_center(ctx.part_world_aabb(lid))
        closed_cam_center = _aabb_center(ctx.part_world_aabb(cam))

    with ctx.pose({lid_hinge: 1.80}):
        open_lid_center = _aabb_center(ctx.part_world_aabb(lid))
        open_cam_center = _aabb_center(ctx.part_world_aabb(cam))

    ctx.check(
        "lid opens upward from the side hinge",
        closed_lid_center is not None
        and open_lid_center is not None
        and open_lid_center[2] > closed_lid_center[2] + 0.010
        and open_lid_center[0] < closed_lid_center[0] - 0.008,
        details=f"closed={closed_lid_center}, open={open_lid_center}",
    )
    ctx.check(
        "cam lever rotates when the lid opens",
        closed_cam_center is not None
        and open_cam_center is not None
        and abs(open_cam_center[0] - closed_cam_center[0]) > 0.001
        and abs(open_cam_center[2] - closed_cam_center[2]) > 0.001,
        details=f"closed={closed_cam_center}, open={open_cam_center}",
    )

    return ctx.report()


object_model = build_object_model()
