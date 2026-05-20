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
CASE_D = 0.013
LOWER_H = 0.038
LID_H = 0.018
SHELL_WALL = 0.0006

INSERT_W = 0.0352
INSERT_D = 0.0112
INSERT_BASE_Z = 0.0006
INSERT_BODY_H = 0.031

CHIMNEY_W = 0.016
CHIMNEY_D = 0.0088
CHIMNEY_H = 0.013
CHIMNEY_WALL = 0.00045
CHIMNEY_BASE_X = -0.0032
CHIMNEY_BASE_Z = 0.029

WHEEL_R = 0.0032
WHEEL_LEN = 0.0044
WHEEL_X = 0.0054
WHEEL_Z = 0.0344

HINGE_AXIS_X = (CASE_W * 0.5) + 0.0011
HINGE_AXIS_Z = LOWER_H + 0.0046
HINGE_LEAF_T = 0.0008
HINGE_BARREL_R = 0.0011
HINGE_BARREL_LEN = 0.0046

CAM_PIVOT_X = -0.0038
CAM_PIVOT_Z = -0.0008

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_top_lighter")

    brass = model.material("brass", rgba=(0.74, 0.61, 0.30, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    knurl_steel = model.material("knurl_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    wick_color = model.material("wick_color", rgba=(0.86, 0.82, 0.68, 1.0))

    lower_case = model.part("lower_case")
    lower_case.visual(
        Box((CASE_W, CASE_D, SHELL_WALL)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_WALL * 0.5)),
        material=brass,
        name="case_bottom",
    )
    lower_case.visual(
        Box((CASE_W - (2.0 * SHELL_WALL), SHELL_WALL, LOWER_H - SHELL_WALL)),
        origin=Origin(xyz=(0.0, (CASE_D * 0.5) - (SHELL_WALL * 0.5), (LOWER_H + SHELL_WALL) * 0.5)),
        material=brass,
        name="case_front_wall",
    )
    lower_case.visual(
        Box((CASE_W - (2.0 * SHELL_WALL), SHELL_WALL, LOWER_H - SHELL_WALL)),
        origin=Origin(xyz=(0.0, -((CASE_D * 0.5) - (SHELL_WALL * 0.5)), (LOWER_H + SHELL_WALL) * 0.5)),
        material=brass,
        name="case_back_wall",
    )
    lower_case.visual(
        Box((SHELL_WALL, CASE_D, LOWER_H - SHELL_WALL)),
        origin=Origin(xyz=((CASE_W * 0.5) - (SHELL_WALL * 0.5), 0.0, (LOWER_H + SHELL_WALL) * 0.5)),
        material=brass,
        name="case_hinge_side_wall",
    )
    lower_case.visual(
        Box((SHELL_WALL, CASE_D, LOWER_H - SHELL_WALL)),
        origin=Origin(xyz=(-((CASE_W * 0.5) - (SHELL_WALL * 0.5)), 0.0, (LOWER_H + SHELL_WALL) * 0.5)),
        material=brass,
        name="case_free_side_wall",
    )
    lower_case.visual(
        Box((HINGE_LEAF_T, 0.0062, 0.0138)),
        origin=Origin(xyz=((CASE_W * 0.5) + (HINGE_LEAF_T * 0.5), 0.0, 0.031)),
        material=brass,
        name="case_leaf",
    )
    for index, z in enumerate((0.027, 0.031, 0.035)):
        lower_case.visual(
            Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_LEN),
            origin=Origin(
                xyz=((CASE_W * 0.5) + HINGE_LEAF_T + HINGE_BARREL_R, 0.0, z),
                rpy=(pi * 0.5, 0.0, 0.0),
            ),
            material=brass,
            name=f"case_knuckle_{index}",
        )

    insert = model.part("insert")
    insert.visual(
        Box((INSERT_W, INSERT_D, INSERT_BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, INSERT_BODY_H * 0.5)),
        material=brushed_steel,
        name="insert_body",
    )
    insert.visual(
        Box((INSERT_W - 0.0024, INSERT_D - 0.0018, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, INSERT_BODY_H + 0.0015)),
        material=brushed_steel,
        name="insert_shoulder",
    )
    insert.visual(
        Box((CHIMNEY_W, CHIMNEY_WALL, CHIMNEY_H)),
        origin=Origin(xyz=(CHIMNEY_BASE_X, (CHIMNEY_D * 0.5) - (CHIMNEY_WALL * 0.5), CHIMNEY_BASE_Z + (CHIMNEY_H * 0.5))),
        material=brushed_steel,
        name="chimney_front_wall",
    )
    insert.visual(
        Box((CHIMNEY_W, CHIMNEY_WALL, CHIMNEY_H)),
        origin=Origin(xyz=(CHIMNEY_BASE_X, -((CHIMNEY_D * 0.5) - (CHIMNEY_WALL * 0.5)), CHIMNEY_BASE_Z + (CHIMNEY_H * 0.5))),
        material=brushed_steel,
        name="chimney_back_wall",
    )
    insert.visual(
        Box((CHIMNEY_WALL, CHIMNEY_D, CHIMNEY_H)),
        origin=Origin(xyz=(CHIMNEY_BASE_X + (CHIMNEY_W * 0.5) - (CHIMNEY_WALL * 0.5), 0.0, CHIMNEY_BASE_Z + (CHIMNEY_H * 0.5))),
        material=brushed_steel,
        name="chimney_side_wall_0",
    )
    insert.visual(
        Box((CHIMNEY_WALL, CHIMNEY_D, CHIMNEY_H)),
        origin=Origin(xyz=(CHIMNEY_BASE_X - ((CHIMNEY_W * 0.5) - (CHIMNEY_WALL * 0.5)), 0.0, CHIMNEY_BASE_Z + (CHIMNEY_H * 0.5))),
        material=brushed_steel,
        name="chimney_side_wall_1",
    )
    insert.visual(
        Cylinder(radius=0.0011, length=0.012),
        origin=Origin(xyz=(CHIMNEY_BASE_X - 0.0011, 0.0, CHIMNEY_BASE_Z + CHIMNEY_H - 0.006)),
        material=wick_color,
        name="wick",
    )
    insert.visual(
        Box((0.0024, 0.0013, 0.0084)),
        origin=Origin(xyz=(WHEEL_X, (WHEEL_LEN * 0.5) + 0.00065, WHEEL_Z)),
        material=brushed_steel,
        name="wheel_cheek_front",
    )
    insert.visual(
        Box((0.0024, 0.0013, 0.0084)),
        origin=Origin(xyz=(WHEEL_X, -((WHEEL_LEN * 0.5) + 0.00065), WHEEL_Z)),
        material=brushed_steel,
        name="wheel_cheek_back",
    )
    insert.visual(
        Box((0.0050, 0.0032, 0.0014)),
        origin=Origin(xyz=(WHEEL_X - 0.0006, 0.0, WHEEL_Z - 0.0036)),
        material=brushed_steel,
        name="wheel_bridge",
    )

    lid = model.part("lid")
    lid.visual(
        Box((CASE_W, CASE_D, SHELL_WALL)),
        origin=Origin(xyz=(-HINGE_AXIS_X, 0.0, (LOWER_H + LID_H - (SHELL_WALL * 0.5)) - HINGE_AXIS_Z)),
        material=brass,
        name="lid_roof",
    )
    lid.visual(
        Box((CASE_W - (2.0 * SHELL_WALL), SHELL_WALL, LID_H - SHELL_WALL)),
        origin=Origin(
            xyz=(
                -HINGE_AXIS_X,
                (CASE_D * 0.5) - (SHELL_WALL * 0.5),
                (LOWER_H + ((LID_H - SHELL_WALL) * 0.5)) - HINGE_AXIS_Z,
            )
        ),
        material=brass,
        name="lid_front_wall",
    )
    lid.visual(
        Box((CASE_W - (2.0 * SHELL_WALL), SHELL_WALL, LID_H - SHELL_WALL)),
        origin=Origin(
            xyz=(
                -HINGE_AXIS_X,
                -((CASE_D * 0.5) - (SHELL_WALL * 0.5)),
                (LOWER_H + ((LID_H - SHELL_WALL) * 0.5)) - HINGE_AXIS_Z,
            )
        ),
        material=brass,
        name="lid_back_wall",
    )
    lid.visual(
        Box((SHELL_WALL, CASE_D, LID_H - SHELL_WALL)),
        origin=Origin(
            xyz=(
                ((CASE_W * 0.5) - (SHELL_WALL * 0.5)) - HINGE_AXIS_X,
                0.0,
                (LOWER_H + ((LID_H - SHELL_WALL) * 0.5)) - HINGE_AXIS_Z,
            )
        ),
        material=brass,
        name="lid_hinge_side_wall",
    )
    lid.visual(
        Box((SHELL_WALL, CASE_D, LID_H - SHELL_WALL)),
        origin=Origin(
            xyz=(
                -((CASE_W * 0.5) - (SHELL_WALL * 0.5)) - HINGE_AXIS_X,
                0.0,
                (LOWER_H + ((LID_H - SHELL_WALL) * 0.5)) - HINGE_AXIS_Z,
            )
        ),
        material=brass,
        name="lid_free_side_wall",
    )
    lid.visual(
        Box((HINGE_LEAF_T, 0.0062, 0.0136)),
        origin=Origin(xyz=(-0.0016, 0.0, 0.0032)),
        material=brass,
        name="lid_leaf",
    )
    for index, z in enumerate((0.0012, 0.0052)):
        lid.visual(
            Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_LEN),
            origin=Origin(
                xyz=(-0.0001, 0.0, z),
                rpy=(pi * 0.5, 0.0, 0.0),
            ),
            material=brass,
            name=f"lid_knuckle_{index}",
        )
    lid.visual(
        Box((0.0020, 0.0010, 0.0052)),
        origin=Origin(xyz=(CAM_PIVOT_X + 0.0017, 0.00125, CAM_PIVOT_Z - 0.0008)),
        material=brass,
        name="cam_bracket_front",
    )
    lid.visual(
        Box((0.0020, 0.0010, 0.0052)),
        origin=Origin(xyz=(CAM_PIVOT_X + 0.0017, -0.00125, CAM_PIVOT_Z - 0.0008)),
        material=brass,
        name="cam_bracket_back",
    )

    striker_wheel = model.part("striker_wheel")
    striker_wheel.visual(
        Cylinder(radius=WHEEL_R, length=WHEEL_LEN),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=knurl_steel,
        name="wheel",
    )
    striker_wheel.visual(
        Cylinder(radius=WHEEL_R * 0.62, length=WHEEL_LEN * 1.02),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_hub",
    )

    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        Cylinder(radius=0.0008, length=0.0022),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="cam_hub",
    )
    cam_lever.visual(
        Box((0.0044, 0.0014, 0.0012)),
        origin=Origin(xyz=(-0.0025, 0.0, -0.0012)),
        material=dark_steel,
        name="cam_arm",
    )
    cam_lever.visual(
        Box((0.0012, 0.0014, 0.0030)),
        origin=Origin(xyz=(-0.0044, 0.0, -0.0001)),
        material=dark_steel,
        name="cam_toe",
    )

    model.articulation(
        "insert_mount",
        ArticulationType.FIXED,
        parent=lower_case,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, INSERT_BASE_Z)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_case,
        child=lid,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "striker_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=striker_wheel,
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=35.0),
    )
    model.articulation(
        "cam_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cam_lever,
        origin=Origin(xyz=(CAM_PIVOT_X, 0.0, CAM_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0, lower=-0.55, upper=0.22),
        mimic=Mimic(joint="lid_hinge", multiplier=-0.33, offset=0.05),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_case = object_model.get_part("lower_case")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    striker_wheel = object_model.get_part("striker_wheel")
    cam_lever = object_model.get_part("cam_lever")

    lid_hinge = object_model.get_articulation("lid_hinge")
    ctx.expect_within(
        insert,
        lower_case,
        axes="xy",
        margin=0.0015,
        name="insert stays nested within the lower case footprint",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            lower_case,
            axis="z",
            max_gap=0.0016,
            max_penetration=0.0,
            positive_elem="lid_front_wall",
            negative_elem="case_front_wall",
            name="closed lid sits tightly above the lower case",
        )
        ctx.expect_overlap(
            lid,
            lower_case,
            axes="xy",
            min_overlap=0.010,
            name="closed lid covers the lower case opening",
        )
        ctx.expect_origin_distance(
            striker_wheel,
            insert,
            axes="x",
            min_dist=0.004,
            max_dist=0.007,
            name="striker wheel sits beside the chimney instead of centered on it",
        )

    closed_lid_center = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_free_side_wall"))
    closed_cam_center = _aabb_center(ctx.part_element_world_aabb(cam_lever, elem="cam_arm"))
    with ctx.pose({lid_hinge: 1.95}):
        open_lid_center = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_free_side_wall"))
        open_cam_center = _aabb_center(ctx.part_element_world_aabb(cam_lever, elem="cam_arm"))

    ctx.check(
        "lid opens upward from the hinge side",
        closed_lid_center is not None
        and open_lid_center is not None
        and open_lid_center[2] > closed_lid_center[2] + 0.010,
        details=f"closed={closed_lid_center}, open={open_lid_center}",
    )
    ctx.check(
        "cam lever changes pose as the lid opens",
        closed_cam_center is not None
        and open_cam_center is not None
        and abs(open_cam_center[0] - closed_cam_center[0]) > 0.0008,
        details=f"closed={closed_cam_center}, open={open_cam_center}",
    )

    return ctx.report()


object_model = build_object_model()
