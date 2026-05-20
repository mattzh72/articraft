from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HOUSING_LENGTH = 0.180
HOUSING_OUTER_W = 0.160
HOUSING_OUTER_H = 0.120
HOUSING_WALL = 0.010
HOUSING_BACK_CAP = 0.014
STOP_CLEARANCE = 0.0
HOUSING_INNER_H = HOUSING_OUTER_H - 2.0 * HOUSING_WALL
HOUSING_INNER_W = HOUSING_OUTER_W - 2.0 * HOUSING_WALL

STAGE1_TUBE_LEN = 0.150
STAGE1_OUTER_W = 0.137
STAGE1_OUTER_H = 0.097
STAGE1_WALL = 0.0075
STAGE1_FLANGE_W = 0.152
STAGE1_FLANGE_H = 0.112
STAGE1_FLANGE_T = 0.006
STAGE1_HOME_X = HOUSING_LENGTH - STAGE1_TUBE_LEN + STOP_CLEARANCE
STAGE1_TRAVEL = 0.095
STAGE1_PAD_T = (HOUSING_INNER_H - STAGE1_OUTER_H) / 2.0
STAGE1_PAD_W = 0.028
STAGE1_PAD_LEN = 0.090

STAGE2_TUBE_LEN = 0.136
STAGE2_OUTER_W = 0.118
STAGE2_OUTER_H = 0.078
STAGE2_WALL = 0.0065
STAGE2_FLANGE_W = 0.132
STAGE2_FLANGE_H = 0.092
STAGE2_FLANGE_T = 0.006
STAGE2_HOME_X = STAGE1_TUBE_LEN + STAGE1_FLANGE_T - STAGE2_TUBE_LEN + STOP_CLEARANCE
STAGE2_TRAVEL = 0.088
STAGE2_INNER_H = STAGE2_OUTER_H - 2.0 * STAGE2_WALL
STAGE1_INNER_H = STAGE1_OUTER_H - 2.0 * STAGE1_WALL
STAGE2_PAD_T = (STAGE1_INNER_H - STAGE2_OUTER_H) / 2.0
STAGE2_PAD_W = 0.024
STAGE2_PAD_LEN = 0.082

STAGE3_TUBE_LEN = 0.122
STAGE3_OUTER_W = 0.099
STAGE3_OUTER_H = 0.059
STAGE3_WALL = 0.0055
STAGE3_PLATE_W = 0.112
STAGE3_PLATE_H = 0.072
STAGE3_PLATE_T = 0.008
STAGE3_HOME_X = STAGE2_TUBE_LEN + STAGE2_FLANGE_T - STAGE3_TUBE_LEN
STAGE3_TRAVEL = 0.080
STAGE2_INNER_W = STAGE2_OUTER_W - 2.0 * STAGE2_WALL
STAGE2_INNER_H = STAGE2_OUTER_H - 2.0 * STAGE2_WALL
STAGE3_PAD_T = (STAGE2_INNER_H - STAGE3_OUTER_H) / 2.0
STAGE3_PAD_W = 0.020
STAGE3_PAD_LEN = 0.075


def _add_box(part, name: str, size: tuple[float, float, float], center: tuple[float, float, float], material) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_sleeve(
    *,
    part,
    prefix: str,
    length: float,
    outer_w: float,
    outer_h: float,
    wall: float,
    material,
    front_w: float | None = None,
    front_h: float | None = None,
    front_t: float = 0.0,
    pad_t: float = 0.0,
    pad_w: float = 0.0,
    pad_len: float = 0.0,
) -> None:
    inner_w = outer_w - 2.0 * wall
    inner_h = outer_h - 2.0 * wall

    _add_box(
        part,
        f"{prefix}_top_wall",
        (length, inner_w, wall),
        (length / 2.0, 0.0, outer_h / 2.0 - wall / 2.0),
        material,
    )
    _add_box(
        part,
        f"{prefix}_bottom_wall",
        (length, inner_w, wall),
        (length / 2.0, 0.0, -outer_h / 2.0 + wall / 2.0),
        material,
    )
    _add_box(
        part,
        f"{prefix}_left_wall",
        (length, wall, outer_h),
        (length / 2.0, outer_w / 2.0 - wall / 2.0, 0.0),
        material,
    )
    _add_box(
        part,
        f"{prefix}_right_wall",
        (length, wall, outer_h),
        (length / 2.0, -outer_w / 2.0 + wall / 2.0, 0.0),
        material,
    )

    if front_w is not None and front_h is not None and front_t > 0.0:
        side_band = (front_w - outer_w) / 2.0
        top_band = (front_h - outer_h) / 2.0
        _add_box(
            part,
            f"{prefix}_front_top",
            (front_t, front_w, top_band),
            (length + front_t / 2.0, 0.0, outer_h / 2.0 + top_band / 2.0),
            material,
        )
        _add_box(
            part,
            f"{prefix}_front_bottom",
            (front_t, front_w, top_band),
            (length + front_t / 2.0, 0.0, -outer_h / 2.0 - top_band / 2.0),
            material,
        )
        _add_box(
            part,
            f"{prefix}_front_left",
            (front_t, side_band, outer_h),
            (length + front_t / 2.0, outer_w / 2.0 + side_band / 2.0, 0.0),
            material,
        )
        _add_box(
            part,
            f"{prefix}_front_right",
            (front_t, side_band, outer_h),
            (length + front_t / 2.0, -outer_w / 2.0 - side_band / 2.0, 0.0),
            material,
        )

    if pad_t > 0.0 and pad_w > 0.0 and pad_len > 0.0:
        pad_x = max(wall + pad_len / 2.0, length * 0.45)
        _add_box(
            part,
            f"{prefix}_top_pad",
            (pad_len, pad_w, pad_t),
            (pad_x, 0.0, outer_h / 2.0 + pad_t / 2.0),
            material,
        )
        _add_box(
            part,
            f"{prefix}_bottom_pad",
            (pad_len, pad_w, pad_t),
            (pad_x, 0.0, -outer_h / 2.0 - pad_t / 2.0),
            material,
        )


def _add_housing(
    *,
    part,
    material,
) -> None:
    inner_h = HOUSING_OUTER_H - 2.0 * HOUSING_WALL
    _add_box(
        part,
        "housing_top_wall",
        (HOUSING_LENGTH, HOUSING_OUTER_W, HOUSING_WALL),
        (HOUSING_LENGTH / 2.0, 0.0, HOUSING_OUTER_H / 2.0 - HOUSING_WALL / 2.0),
        material,
    )
    _add_box(
        part,
        "housing_bottom_wall",
        (HOUSING_LENGTH, HOUSING_OUTER_W, HOUSING_WALL),
        (HOUSING_LENGTH / 2.0, 0.0, -HOUSING_OUTER_H / 2.0 + HOUSING_WALL / 2.0),
        material,
    )
    _add_box(
        part,
        "housing_left_wall",
        (HOUSING_LENGTH, HOUSING_WALL, inner_h),
        (HOUSING_LENGTH / 2.0, HOUSING_OUTER_W / 2.0 - HOUSING_WALL / 2.0, 0.0),
        material,
    )
    _add_box(
        part,
        "housing_right_wall",
        (HOUSING_LENGTH, HOUSING_WALL, inner_h),
        (HOUSING_LENGTH / 2.0, -HOUSING_OUTER_W / 2.0 + HOUSING_WALL / 2.0, 0.0),
        material,
    )
    _add_box(
        part,
        "housing_back_cap",
        (HOUSING_BACK_CAP, HOUSING_OUTER_W, HOUSING_OUTER_H),
        (HOUSING_BACK_CAP / 2.0, 0.0, 0.0),
        material,
    )
    _add_box(part, "housing_foot_left", (0.110, 0.030, 0.012), (0.085, 0.047, -0.066), material)
    _add_box(part, "housing_foot_right", (0.110, 0.030, 0.012), (0.085, -0.047, -0.066), material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_rectangular_ram_stack")

    housing_color = model.material("housing_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    sleeve_color = model.material("sleeve_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    tip_color = model.material("tip_plate", rgba=(0.82, 0.84, 0.87, 1.0))

    housing = model.part("housing")
    _add_housing(
        part=housing,
        material=housing_color,
    )

    stage1 = model.part("stage1")
    _add_sleeve(
        part=stage1,
        prefix="stage1",
        length=STAGE1_TUBE_LEN,
        outer_w=STAGE1_OUTER_W,
        outer_h=STAGE1_OUTER_H,
        wall=STAGE1_WALL,
        material=sleeve_color,
        front_w=STAGE1_FLANGE_W,
        front_h=STAGE1_FLANGE_H,
        front_t=STAGE1_FLANGE_T,
        pad_t=STAGE1_PAD_T,
        pad_w=STAGE1_PAD_W,
        pad_len=STAGE1_PAD_LEN,
    )

    stage2 = model.part("stage2")
    _add_sleeve(
        part=stage2,
        prefix="stage2",
        length=STAGE2_TUBE_LEN,
        outer_w=STAGE2_OUTER_W,
        outer_h=STAGE2_OUTER_H,
        wall=STAGE2_WALL,
        material=sleeve_color,
        front_w=STAGE2_FLANGE_W,
        front_h=STAGE2_FLANGE_H,
        front_t=STAGE2_FLANGE_T,
        pad_t=STAGE2_PAD_T,
        pad_w=STAGE2_PAD_W,
        pad_len=STAGE2_PAD_LEN,
    )

    stage3 = model.part("stage3")
    _add_sleeve(
        part=stage3,
        prefix="stage3",
        length=STAGE3_TUBE_LEN,
        outer_w=STAGE3_OUTER_W,
        outer_h=STAGE3_OUTER_H,
        wall=STAGE3_WALL,
        material=tip_color,
        pad_t=STAGE3_PAD_T,
        pad_w=STAGE3_PAD_W,
        pad_len=STAGE3_PAD_LEN,
    )
    _add_box(
        stage3,
        "stage3_end_plate",
        (STAGE3_PLATE_T, STAGE3_PLATE_W, STAGE3_PLATE_H),
        (STAGE3_TUBE_LEN + STAGE3_PLATE_T / 2.0, 0.0, 0.0),
        tip_color,
    )

    model.articulation(
        "housing_to_stage1",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=stage1,
        origin=Origin(xyz=(STAGE1_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3500.0,
            velocity=0.25,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(STAGE2_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.25,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.PRISMATIC,
        parent=stage2,
        child=stage3,
        origin=Origin(xyz=(STAGE3_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.25,
            lower=0.0,
            upper=STAGE3_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    stage1 = object_model.get_part("stage1")
    stage2 = object_model.get_part("stage2")
    stage3 = object_model.get_part("stage3")

    joint1 = object_model.get_articulation("housing_to_stage1")
    joint2 = object_model.get_articulation("stage1_to_stage2")
    joint3 = object_model.get_articulation("stage2_to_stage3")

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

    ctx.check(
        "all_stage_joints_are_prismatic",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            for joint in (joint1, joint2, joint3)
        ),
        "All three sliding stages should be prismatic articulations.",
    )
    ctx.check(
        "all_stage_axes_share_common_x_axis",
        all(tuple(joint.axis) == (1.0, 0.0, 0.0) for joint in (joint1, joint2, joint3)),
        "All three stages should slide on the same +X axis.",
    )
    ctx.check(
        "stage_travels_are_positive_and_ordered",
        (
            joint1.motion_limits is not None
            and joint2.motion_limits is not None
            and joint3.motion_limits is not None
            and joint1.motion_limits.lower == 0.0
            and joint2.motion_limits.lower == 0.0
            and joint3.motion_limits.lower == 0.0
            and joint1.motion_limits.upper == STAGE1_TRAVEL
            and joint2.motion_limits.upper == STAGE2_TRAVEL
            and joint3.motion_limits.upper == STAGE3_TRAVEL
        ),
        "Each sleeve should have a finite forward travel from a fully nested home position.",
    )

    with ctx.pose({joint1: 0.0, joint2: 0.0, joint3: 0.0}):
        ctx.expect_contact(housing, stage1, name="stage1_seats_against_housing")
        ctx.expect_contact(stage1, stage2, name="stage2_seats_against_stage1")
        ctx.expect_contact(stage2, stage3, name="stage3_end_plate_seats_against_stage2")
        ctx.expect_overlap(stage1, housing, axes="yz", min_overlap=0.090, name="stage1_nested_in_housing")
        ctx.expect_overlap(stage2, stage1, axes="yz", min_overlap=0.070, name="stage2_nested_in_stage1")
        ctx.expect_overlap(stage3, stage2, axes="yz", min_overlap=0.055, name="stage3_nested_in_stage2")
        ctx.expect_origin_distance(
            stage1,
            housing,
            axes="yz",
            max_dist=1e-6,
            name="stage1_centered_on_housing_axis",
        )
        ctx.expect_origin_distance(
            stage2,
            stage1,
            axes="yz",
            max_dist=1e-6,
            name="stage2_centered_on_stage1_axis",
        )
        ctx.expect_origin_distance(
            stage3,
            stage2,
            axes="yz",
            max_dist=1e-6,
            name="stage3_centered_on_stage2_axis",
        )

    with ctx.pose({joint1: 0.050, joint2: 0.040, joint3: 0.030}):
        ctx.expect_origin_gap(
            stage1,
            housing,
            axis="x",
            min_gap=STAGE1_HOME_X + 0.050 - 0.001,
            max_gap=STAGE1_HOME_X + 0.050 + 0.001,
            name="stage1_translates_forward",
        )
        ctx.expect_origin_gap(
            stage2,
            stage1,
            axis="x",
            min_gap=STAGE2_HOME_X + 0.040 - 0.001,
            max_gap=STAGE2_HOME_X + 0.040 + 0.001,
            name="stage2_translates_forward",
        )
        ctx.expect_origin_gap(
            stage3,
            stage2,
            axis="x",
            min_gap=STAGE3_HOME_X + 0.030 - 0.001,
            max_gap=STAGE3_HOME_X + 0.030 + 0.001,
            name="stage3_translates_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
