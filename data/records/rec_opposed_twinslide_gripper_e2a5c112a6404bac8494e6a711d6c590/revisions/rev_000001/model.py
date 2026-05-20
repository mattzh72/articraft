from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_W = 0.176
FRAME_H = 0.102
BACK_T = 0.014
BACK_WINDOW_W = 0.082
BACK_WINDOW_H = 0.038

HOUSING_W = 0.050
HOUSING_D = 0.050
HOUSING_H = 0.060
HOUSING_Y = (BACK_T / 2.0) + (HOUSING_D / 2.0)

GUIDE_LEN = 0.060
GUIDE_D = 0.030
GUIDE_H = 0.024
GUIDE_CENTER_X = (HOUSING_W / 2.0) + (GUIDE_LEN / 2.0)
GUIDE_CENTER_Y = HOUSING_Y

SLEEVE_L = 0.048
SLEEVE_D = 0.040
SLEEVE_H = 0.042
SLEEVE_OPEN_X = 0.062
JAW_TRAVEL = 0.020
SLEEVE_TOP_T = (SLEEVE_H - GUIDE_H) / 2.0
SLEEVE_SIDE_T = (SLEEVE_D - GUIDE_D) / 2.0
SLEEVE_TOP_Z = (GUIDE_H / 2.0) + (SLEEVE_TOP_T / 2.0)
SLEEVE_SIDE_Y = (GUIDE_D / 2.0) + (SLEEVE_SIDE_T / 2.0)

ARM_X = 0.018
ARM_Y = 0.038
ARM_Z = 0.060
ARM_X_CENTER = 0.014
ARM_Y_CENTER = (SLEEVE_D / 2.0) + 0.009

PAD_X = 0.010
PAD_Y = 0.016
PAD_Z = 0.028
PAD_X_CENTER = 0.019
PAD_Y_CENTER = 0.052


def _rear_frame_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(FRAME_W, BACK_T, FRAME_H)
    window = cq.Workplane("XY").box(BACK_WINDOW_W, BACK_T + 0.004, BACK_WINDOW_H)
    lower_bridge = cq.Workplane("XY").box(0.110, BACK_T * 0.65, 0.018).translate(
        (0.0, BACK_T * 0.15, -(FRAME_H * 0.26))
    )
    return plate.cut(window).union(lower_bridge)


def _housing_shape() -> cq.Workplane:
    housing = cq.Workplane("XY").box(HOUSING_W, HOUSING_D, HOUSING_H)
    front_cap = cq.Workplane("XY").box(
        HOUSING_W * 0.82,
        0.008,
        HOUSING_H * 0.72,
    ).translate((0.0, (HOUSING_D / 2.0) + 0.004, 0.0))
    return housing.union(front_cap)


def _guideway_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(GUIDE_LEN, GUIDE_D, GUIDE_H)


def _sleeve_shape() -> cq.Workplane:
    top = cq.Workplane("XY").box(SLEEVE_L, SLEEVE_D, (SLEEVE_H - GUIDE_H) / 2.0).translate(
        (0.0, 0.0, (GUIDE_H / 2.0) + ((SLEEVE_H - GUIDE_H) / 4.0))
    )
    bottom = cq.Workplane("XY").box(
        SLEEVE_L,
        SLEEVE_D,
        (SLEEVE_H - GUIDE_H) / 2.0,
    ).translate((0.0, 0.0, -((GUIDE_H / 2.0) + ((SLEEVE_H - GUIDE_H) / 4.0))))
    front = cq.Workplane("XY").box(
        SLEEVE_L,
        (SLEEVE_D - GUIDE_D) / 2.0,
        GUIDE_H,
    ).translate((0.0, (GUIDE_D / 2.0) + ((SLEEVE_D - GUIDE_D) / 4.0), 0.0))
    back = cq.Workplane("XY").box(
        SLEEVE_L,
        (SLEEVE_D - GUIDE_D) / 2.0,
        GUIDE_H,
    ).translate((0.0, -((GUIDE_D / 2.0) + ((SLEEVE_D - GUIDE_D) / 4.0)), 0.0))
    return top.union(bottom).union(front).union(back)


def _jaw_arm_shape(side: float) -> cq.Workplane:
    arm = cq.Workplane("XY").box(ARM_X, ARM_Y, ARM_Z).translate(
        (side * ARM_X_CENTER, ARM_Y_CENTER, 0.0)
    )
    root = cq.Workplane("XY").box(0.024, 0.014, SLEEVE_H * 0.88).translate(
        (side * 0.006, (SLEEVE_D / 2.0) + 0.001, 0.0)
    )
    return arm.union(root)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_parallel_gripper")

    frame_dark = model.material("frame_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.66, 0.68, 0.72, 1.0))
    carriage_dark = model.material("carriage_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    pad_steel = model.material("pad_steel", rgba=(0.56, 0.58, 0.60, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_rear_frame_shape(), "rear_frame"),
        name="rear_frame",
        material=frame_dark,
    )
    frame.visual(
        mesh_from_cadquery(_housing_shape(), "center_housing"),
        origin=Origin(xyz=(0.0, HOUSING_Y, 0.0)),
        name="center_housing",
        material=frame_dark,
    )
    frame.visual(
        mesh_from_cadquery(_guideway_shape(), "left_guideway"),
        origin=Origin(xyz=(-GUIDE_CENTER_X, GUIDE_CENTER_Y, 0.0)),
        name="left_guideway",
        material=rail_steel,
    )
    frame.visual(
        mesh_from_cadquery(_guideway_shape(), "right_guideway"),
        origin=Origin(xyz=(GUIDE_CENTER_X, GUIDE_CENTER_Y, 0.0), rpy=(0.0, 0.0, 3.141592653589793)),
        name="right_guideway",
        material=rail_steel,
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_W, HOUSING_Y + (HOUSING_D / 2.0), FRAME_H)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
    )

    left_carriage = model.part("left_carriage")
    left_carriage.visual(
        Box((SLEEVE_L, SLEEVE_D, SLEEVE_TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_TOP_Z)),
        name="sleeve_top",
        material=carriage_dark,
    )
    left_carriage.visual(
        Box((SLEEVE_L, SLEEVE_D, SLEEVE_TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, -SLEEVE_TOP_Z)),
        name="sleeve_bottom",
        material=carriage_dark,
    )
    left_carriage.visual(
        Box((SLEEVE_L, SLEEVE_SIDE_T, GUIDE_H)),
        origin=Origin(xyz=(0.0, SLEEVE_SIDE_Y, 0.0)),
        name="sleeve_front",
        material=carriage_dark,
    )
    left_carriage.visual(
        Box((SLEEVE_L, SLEEVE_SIDE_T, GUIDE_H)),
        origin=Origin(xyz=(0.0, -SLEEVE_SIDE_Y, 0.0)),
        name="sleeve_back",
        material=carriage_dark,
    )
    left_carriage.visual(
        mesh_from_cadquery(_jaw_arm_shape(1.0), "left_jaw_arm"),
        name="jaw_arm",
        material=carriage_dark,
    )
    left_carriage.visual(
        Box((PAD_X, PAD_Y, PAD_Z)),
        origin=Origin(xyz=(PAD_X_CENTER, PAD_Y_CENTER, 0.0)),
        name="inner_pad",
        material=pad_steel,
    )
    left_carriage.inertial = Inertial.from_geometry(
        Box((SLEEVE_L, ARM_Y + 0.020, ARM_Z)),
        mass=0.45,
        origin=Origin(xyz=(0.010, 0.024, 0.0)),
    )

    right_carriage = model.part("right_carriage")
    right_carriage.visual(
        Box((SLEEVE_L, SLEEVE_D, SLEEVE_TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_TOP_Z)),
        name="sleeve_top",
        material=carriage_dark,
    )
    right_carriage.visual(
        Box((SLEEVE_L, SLEEVE_D, SLEEVE_TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, -SLEEVE_TOP_Z)),
        name="sleeve_bottom",
        material=carriage_dark,
    )
    right_carriage.visual(
        Box((SLEEVE_L, SLEEVE_SIDE_T, GUIDE_H)),
        origin=Origin(xyz=(0.0, SLEEVE_SIDE_Y, 0.0)),
        name="sleeve_front",
        material=carriage_dark,
    )
    right_carriage.visual(
        Box((SLEEVE_L, SLEEVE_SIDE_T, GUIDE_H)),
        origin=Origin(xyz=(0.0, -SLEEVE_SIDE_Y, 0.0)),
        name="sleeve_back",
        material=carriage_dark,
    )
    right_carriage.visual(
        mesh_from_cadquery(_jaw_arm_shape(-1.0), "right_jaw_arm"),
        name="jaw_arm",
        material=carriage_dark,
    )
    right_carriage.visual(
        Box((PAD_X, PAD_Y, PAD_Z)),
        origin=Origin(xyz=(-PAD_X_CENTER, PAD_Y_CENTER, 0.0)),
        name="inner_pad",
        material=pad_steel,
    )
    right_carriage.inertial = Inertial.from_geometry(
        Box((SLEEVE_L, ARM_Y + 0.020, ARM_Z)),
        mass=0.45,
        origin=Origin(xyz=(-0.010, 0.024, 0.0)),
    )

    model.articulation(
        "frame_to_left_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=left_carriage,
        origin=Origin(xyz=(-SLEEVE_OPEN_X, GUIDE_CENTER_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=JAW_TRAVEL,
            effort=140.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "frame_to_right_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=right_carriage,
        origin=Origin(xyz=(SLEEVE_OPEN_X, GUIDE_CENTER_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=JAW_TRAVEL,
            effort=140.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_carriage = object_model.get_part("left_carriage")
    right_carriage = object_model.get_part("right_carriage")
    left_slide = object_model.get_articulation("frame_to_left_carriage")
    right_slide = object_model.get_articulation("frame_to_right_carriage")
    left_upper = left_slide.motion_limits.upper
    right_upper = right_slide.motion_limits.upper

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        frame,
        left_carriage,
        elem_a="left_guideway",
        elem_b="sleeve_top",
        name="left sleeve is mounted on left guideway",
    )
    ctx.expect_contact(
        frame,
        right_carriage,
        elem_a="right_guideway",
        elem_b="sleeve_top",
        name="right sleeve is mounted on right guideway",
    )
    ctx.expect_overlap(
        frame,
        left_carriage,
        axes="x",
        elem_a="left_guideway",
        elem_b="sleeve_top",
        min_overlap=0.045,
        name="left sleeve retains substantial guide engagement at rest",
    )
    ctx.expect_overlap(
        frame,
        right_carriage,
        axes="x",
        elem_a="right_guideway",
        elem_b="sleeve_top",
        min_overlap=0.045,
        name="right sleeve retains substantial guide engagement at rest",
    )
    ctx.expect_gap(
        right_carriage,
        left_carriage,
        axis="x",
        positive_elem="inner_pad",
        negative_elem="inner_pad",
        min_gap=0.072,
        max_gap=0.080,
        name="jaw pads start in a visibly open stance",
    )

    left_rest = ctx.part_world_position(left_carriage)
    right_rest = ctx.part_world_position(right_carriage)
    with ctx.pose({left_slide: left_upper, right_slide: right_upper}):
        ctx.expect_contact(
            frame,
            left_carriage,
            elem_a="left_guideway",
            elem_b="sleeve_top",
            name="left sleeve stays supported at full close",
        )
        ctx.expect_contact(
            frame,
            right_carriage,
            elem_a="right_guideway",
            elem_b="sleeve_top",
            name="right sleeve stays supported at full close",
        )
        ctx.expect_overlap(
            frame,
            left_carriage,
            axes="x",
            elem_a="left_guideway",
            elem_b="sleeve_top",
            min_overlap=0.040,
            name="left sleeve keeps retained insertion at full close",
        )
        ctx.expect_overlap(
            frame,
            right_carriage,
            axes="x",
            elem_a="right_guideway",
            elem_b="sleeve_top",
            min_overlap=0.040,
            name="right sleeve keeps retained insertion at full close",
        )
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            positive_elem="inner_pad",
            negative_elem="inner_pad",
            min_gap=0.032,
            max_gap=0.040,
            name="jaw pads narrow without colliding when closed",
        )
        left_closed = ctx.part_world_position(left_carriage)
        right_closed = ctx.part_world_position(right_carriage)

    ctx.check(
        "left carriage closes inward along +x",
        left_rest is not None
        and left_closed is not None
        and left_closed[0] > left_rest[0] + 0.015,
        details=f"rest={left_rest}, closed={left_closed}",
    )
    ctx.check(
        "right carriage closes inward along -x",
        right_rest is not None
        and right_closed is not None
        and right_closed[0] < right_rest[0] - 0.015,
        details=f"rest={right_rest}, closed={right_closed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
