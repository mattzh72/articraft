from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PLINTH_L = 0.34
PLINTH_W = 0.26
PLINTH_H = 0.048
PAD_R = 0.125
PAD_H = 0.012

ROTARY_R = 0.098
ROTARY_H = 0.018
SADDLE_L = 0.34
SADDLE_W = 0.132
SADDLE_H = 0.028

CHEEK_X = 0.175
CHEEK_T = 0.030
CHEEK_POST_W = 0.110
CHEEK_POST_H = 0.094
CHEEK_POST_Z = 0.065
CHEEK_RIB_T = 0.012
CHEEK_RIB_W = 0.020
CHEEK_RIB_H = 0.092
CHEEK_RIB_Z = 0.064
CHEEK_RIB_Y = 0.040

CAP_R = 0.052
CAP_T = 0.008
CAP_X = CHEEK_X + (CHEEK_T / 2.0) + (CAP_T / 2.0)
CAP_BOLT_R = 0.005
CAP_BOLT_L = 0.005
CAP_BOLT_CIRCLE = 0.040

TABLE_L = 0.274
TABLE_W = 0.190
TABLE_TOP_T = 0.026
TABLE_TOP_Z = 0.018
TABLE_WEB_L = 0.184
TABLE_WEB_W = 0.122
TABLE_WEB_H = 0.048
TABLE_WEB_Z = -0.010
TABLE_RIB_L = 0.232
TABLE_RIB_W = 0.016
TABLE_RIB_H = 0.026
TABLE_RIB_Z = -0.012
TABLE_RIB_Y = 0.062
TABLE_SLOT_L = 0.150
TABLE_SLOT_W = 0.010
TABLE_SLOT_H = 0.006
TABLE_SLOT_Z = 0.026

TABLE_HALF_X = TABLE_L / 2.0
GUSSET_L = 0.022
GUSSET_W = 0.060
GUSSET_H = 0.040
GUSSET_X = TABLE_HALF_X + (GUSSET_L / 2.0)
TRUNNION_HUB_R = 0.031
TRUNNION_HUB_L = 0.010
TRUNNION_HUB_X = 0.154
TRUNNION_SHAFT_R = 0.022
TRUNNION_SHAFT_L = 0.027
TRUNNION_SHAFT_X = 0.1725
TRUNNION_COLLAR_R = 0.031
TRUNNION_COLLAR_L = 0.004
TRUNNION_COLLAR_X = 0.188

YAW_Z = PLINTH_H + PAD_H
PITCH_Z = 0.160
YAW_LOWER = -pi
YAW_UPPER = pi
TILT_LOWER = -1.15
TILT_UPPER = 1.15


def _ox(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _add_cap_bolts(part, sign: float) -> None:
    x = sign * (CAP_X + (CAP_BOLT_L / 2.0))
    for idx, (y, z) in enumerate(
        (
            (CAP_BOLT_CIRCLE, PITCH_Z),
            (-CAP_BOLT_CIRCLE, PITCH_Z),
            (0.0, PITCH_Z + CAP_BOLT_CIRCLE),
            (0.0, PITCH_Z - CAP_BOLT_CIRCLE),
        )
    ):
        part.visual(
            Cylinder(radius=CAP_BOLT_R, length=CAP_BOLT_L),
            origin=_ox((x, y, z)),
            material="black_oxide",
            name=f"{'left' if sign < 0 else 'right'}_cap_bolt_{idx + 1}",
        )


def _add_flange_bolts(part) -> None:
    bolt_positions = (
        (0.082, 0.0),
        (-0.082, 0.0),
        (0.058, 0.058),
        (0.058, -0.058),
        (-0.058, 0.058),
        (-0.058, -0.058),
        (0.0, 0.082),
        (0.0, -0.082),
    )
    for idx, (x, y) in enumerate(bolt_positions):
        part.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, y, 0.021)),
            material="black_oxide",
            name=f"flange_bolt_{idx + 1}",
        )


def _add_side_cheek_set(part, sign: float) -> None:
    x = sign * CHEEK_X
    rib_x = sign * (CHEEK_X + ((CHEEK_T - CHEEK_RIB_T) / 2.0))
    side = "left" if sign < 0 else "right"

    part.visual(
        Box((CHEEK_T, CHEEK_POST_W, CHEEK_POST_H)),
        origin=Origin(xyz=(x, 0.0, CHEEK_POST_Z)),
        material="machine_gray",
        name=f"{side}_post",
    )
    part.visual(
        Box((CHEEK_RIB_T, CHEEK_RIB_W, CHEEK_RIB_H)),
        origin=Origin(xyz=(rib_x, CHEEK_RIB_Y, CHEEK_RIB_Z)),
        material="machine_gray",
        name=f"{side}_front_rib",
    )
    part.visual(
        Box((CHEEK_RIB_T, CHEEK_RIB_W, CHEEK_RIB_H)),
        origin=Origin(xyz=(rib_x, -CHEEK_RIB_Y, CHEEK_RIB_Z)),
        material="machine_gray",
        name=f"{side}_rear_rib",
    )
    part.visual(
        Cylinder(radius=CAP_R, length=CAP_T),
        origin=_ox((sign * CAP_X, 0.0, PITCH_Z)),
        material="black_oxide",
        name=f"{side}_cap",
    )
    _add_cap_bolts(part, sign)


def _add_side_trunnion(part, sign: float) -> None:
    side = "left" if sign < 0 else "right"
    sx = sign

    part.visual(
        Box((GUSSET_L, GUSSET_W, GUSSET_H)),
        origin=Origin(xyz=(sx * GUSSET_X, 0.0, 0.0)),
        material="ground_steel",
        name=f"{side}_gusset",
    )
    part.visual(
        Cylinder(radius=TRUNNION_HUB_R, length=TRUNNION_HUB_L),
        origin=_ox((sx * TRUNNION_HUB_X, 0.0, 0.0)),
        material="ground_steel",
        name=f"{side}_hub",
    )
    part.visual(
        Cylinder(radius=TRUNNION_SHAFT_R, length=TRUNNION_SHAFT_L),
        origin=_ox((sx * TRUNNION_SHAFT_X, 0.0, 0.0)),
        material="ground_steel",
        name=f"{side}_shaft",
    )
    part.visual(
        Cylinder(radius=TRUNNION_COLLAR_R, length=TRUNNION_COLLAR_L),
        origin=_ox((sx * TRUNNION_COLLAR_X, 0.0, 0.0)),
        material="ground_steel",
        name=f"{side}_collar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trunnion_table_fixture")

    model.material("cast_iron", rgba=(0.24, 0.25, 0.27, 1.0))
    model.material("machine_gray", rgba=(0.54, 0.56, 0.58, 1.0))
    model.material("ground_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("black_oxide", rgba=(0.18, 0.18, 0.19, 1.0))

    base_housing = model.part("base_housing")
    base_housing.visual(
        Box((PLINTH_L, PLINTH_W, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H / 2.0)),
        material="cast_iron",
        name="housing",
    )
    base_housing.visual(
        Cylinder(radius=PAD_R, length=PAD_H),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H + (PAD_H / 2.0))),
        material="cast_iron",
        name="rotary_pad",
    )
    for idx, (x, y) in enumerate(((0.118, 0.082), (0.118, -0.082), (-0.118, 0.082), (-0.118, -0.082))):
        base_housing.visual(
            Cylinder(radius=0.009, length=0.005),
            origin=Origin(xyz=(x, y, PLINTH_H + 0.0025)),
            material="black_oxide",
            name=f"base_bolt_{idx + 1}",
        )
    base_housing.inertial = Inertial.from_geometry(
        Box((PLINTH_L, PLINTH_W, PLINTH_H + PAD_H)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (PLINTH_H + PAD_H) / 2.0)),
    )

    rotary_cradle = model.part("rotary_cradle")
    rotary_cradle.visual(
        Cylinder(radius=ROTARY_R, length=ROTARY_H),
        origin=Origin(xyz=(0.0, 0.0, ROTARY_H / 2.0)),
        material="machine_gray",
        name="lower_base",
    )
    rotary_cradle.visual(
        Box((SADDLE_L, SADDLE_W, SADDLE_H)),
        origin=Origin(xyz=(0.0, 0.0, ROTARY_H + (SADDLE_H / 2.0))),
        material="machine_gray",
        name="saddle",
    )
    _add_flange_bolts(rotary_cradle)
    _add_side_cheek_set(rotary_cradle, -1.0)
    _add_side_cheek_set(rotary_cradle, 1.0)
    rotary_cradle.inertial = Inertial.from_geometry(
        Box((0.40, 0.14, 0.22)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    tilt_table = model.part("tilt_table")
    tilt_table.visual(
        Box((TABLE_L, TABLE_W, TABLE_TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, TABLE_TOP_Z)),
        material="ground_steel",
        name="table_top",
    )
    tilt_table.visual(
        Box((TABLE_WEB_L, TABLE_WEB_W, TABLE_WEB_H)),
        origin=Origin(xyz=(0.0, 0.0, TABLE_WEB_Z)),
        material="ground_steel",
        name="table_web",
    )
    tilt_table.visual(
        Box((TABLE_RIB_L, TABLE_RIB_W, TABLE_RIB_H)),
        origin=Origin(xyz=(0.0, TABLE_RIB_Y, TABLE_RIB_Z)),
        material="ground_steel",
        name="front_rib",
    )
    tilt_table.visual(
        Box((TABLE_RIB_L, TABLE_RIB_W, TABLE_RIB_H)),
        origin=Origin(xyz=(0.0, -TABLE_RIB_Y, TABLE_RIB_Z)),
        material="ground_steel",
        name="rear_rib",
    )
    for idx, x in enumerate((-0.075, 0.0, 0.075)):
        tilt_table.visual(
            Box((TABLE_SLOT_L, TABLE_SLOT_W, TABLE_SLOT_H)),
            origin=Origin(xyz=(x, 0.0, TABLE_SLOT_Z)),
            material="black_oxide",
            name=f"t_slot_{idx + 1}",
        )
    _add_side_trunnion(tilt_table, -1.0)
    _add_side_trunnion(tilt_table, 1.0)
    tilt_table.inertial = Inertial.from_geometry(
        Box((0.39, TABLE_W, 0.07)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base_housing,
        child=rotary_cradle,
        origin=Origin(xyz=(0.0, 0.0, YAW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=YAW_LOWER, upper=YAW_UPPER, effort=80.0, velocity=1.2),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=rotary_cradle,
        child=tilt_table,
        origin=Origin(xyz=(0.0, 0.0, PITCH_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=TILT_LOWER, upper=TILT_UPPER, effort=40.0, velocity=0.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_housing = object_model.get_part("base_housing")
    rotary_cradle = object_model.get_part("rotary_cradle")
    tilt_table = object_model.get_part("tilt_table")

    base_yaw = object_model.get_articulation("base_yaw")
    table_tilt = object_model.get_articulation("table_tilt")

    housing = base_housing.get_visual("housing")
    rotary_pad = base_housing.get_visual("rotary_pad")
    lower_base = rotary_cradle.get_visual("lower_base")
    left_post = rotary_cradle.get_visual("left_post")
    right_post = rotary_cradle.get_visual("right_post")
    left_cap = rotary_cradle.get_visual("left_cap")
    right_cap = rotary_cradle.get_visual("right_cap")
    table_top = tilt_table.get_visual("table_top")
    left_collar = tilt_table.get_visual("left_collar")
    right_collar = tilt_table.get_visual("right_collar")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "base_yaw_axis_vertical",
        base_yaw.axis == (0.0, 0.0, 1.0),
        f"expected vertical yaw axis, got {base_yaw.axis}",
    )
    ctx.check(
        "table_tilt_axis_horizontal",
        table_tilt.axis == (1.0, 0.0, 0.0),
        f"expected horizontal trunnion axis, got {table_tilt.axis}",
    )
    ctx.check(
        "yaw_range_reads_like_turntable",
        base_yaw.motion_limits is not None
        and base_yaw.motion_limits.lower is not None
        and base_yaw.motion_limits.upper is not None
        and base_yaw.motion_limits.lower <= -3.0
        and base_yaw.motion_limits.upper >= 3.0,
        "base should read like a full rotary table",
    )
    ctx.check(
        "tilt_range_reads_like_broad_fixture",
        table_tilt.motion_limits is not None
        and table_tilt.motion_limits.lower is not None
        and table_tilt.motion_limits.upper is not None
        and table_tilt.motion_limits.lower <= -1.0
        and table_tilt.motion_limits.upper >= 1.0,
        "table should tilt through a broad usable fixture range",
    )

    ctx.expect_origin_distance(
        rotary_cradle,
        base_housing,
        axes="xy",
        max_dist=0.001,
        name="yaw_stack_is_coaxial",
    )
    ctx.expect_origin_gap(
        tilt_table,
        rotary_cradle,
        axis="z",
        min_gap=0.159,
        max_gap=0.161,
        name="trunnion_axis_height_matches_yoke",
    )

    with ctx.pose({base_yaw: 0.0, table_tilt: 0.0}):
        ctx.expect_gap(
            rotary_cradle,
            base_housing,
            axis="z",
            positive_elem=lower_base,
            negative_elem=rotary_pad,
            max_gap=0.0015,
            max_penetration=1e-5,
            name="rotary_base_seats_on_housing",
        )
        ctx.expect_overlap(
            rotary_cradle,
            base_housing,
            axes="xy",
            elem_a=lower_base,
            elem_b=housing,
            min_overlap=0.18,
            name="rotary_base_has_real_bearing_footprint",
        )
        ctx.expect_contact(
            tilt_table,
            rotary_cradle,
            elem_a=left_collar,
            elem_b=left_cap,
            name="left_trunnion_collared_against_cap",
        )
        ctx.expect_contact(
            tilt_table,
            rotary_cradle,
            elem_a=right_collar,
            elem_b=right_cap,
            name="right_trunnion_collared_against_cap",
        )
        ctx.expect_gap(
            tilt_table,
            rotary_cradle,
            axis="z",
            min_gap=0.060,
            positive_elem=table_top,
            negative_elem=lower_base,
            name="table_top_clears_rotary_base",
        )
        ctx.expect_gap(
            tilt_table,
            rotary_cradle,
            axis="x",
            positive_elem=table_top,
            negative_elem=left_post,
            min_gap=0.020,
            name="table_is_between_left_and_right_cheeks_left_side",
        )
        ctx.expect_gap(
            rotary_cradle,
            tilt_table,
            axis="x",
            positive_elem=right_post,
            negative_elem=table_top,
            min_gap=0.020,
            name="table_is_between_left_and_right_cheeks_right_side",
        )

    with ctx.pose({table_tilt: TILT_LOWER}):
        ctx.expect_gap(
            tilt_table,
            rotary_cradle,
            axis="z",
            min_gap=0.010,
            negative_elem=lower_base,
            name="lower_tilt_pose_clears_base",
        )

    with ctx.pose({table_tilt: TILT_UPPER}):
        ctx.expect_gap(
            tilt_table,
            rotary_cradle,
            axis="z",
            min_gap=0.010,
            negative_elem=lower_base,
            name="upper_tilt_pose_clears_base",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=18)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
