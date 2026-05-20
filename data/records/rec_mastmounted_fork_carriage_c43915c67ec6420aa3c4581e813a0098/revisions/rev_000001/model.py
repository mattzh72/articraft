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


JOIN_OVERLAP = 0.004

BASE_FOOT_W = 1.18
BASE_FOOT_D = 0.34
BASE_FOOT_H = 0.028

BASE_BEAM_W = 1.04
BASE_BEAM_D = 0.22
BASE_BEAM_H = 0.16
BASE_BEAM_Z = 0.108

UPRIGHT_X = 0.42
UPRIGHT_OUTER_W = 0.14
UPRIGHT_OUTER_D = 0.16
UPRIGHT_WALL = 0.014
UPRIGHT_H = 1.94
UPRIGHT_BOTTOM_Z = 0.18

HEADER_W = 0.98
HEADER_D = 0.18
HEADER_H = 0.14
HEADER_Z = 2.06

MID_TIE_W = 0.76
MID_TIE_D = 0.055
MID_TIE_H = 0.10
MID_TIE_Z = 1.19
MID_TIE_Y = -0.108

GUIDE_RAIL_W = 0.03
GUIDE_RAIL_D = 0.12
GUIDE_RAIL_H = 1.56
GUIDE_RAIL_X = 0.335
GUIDE_RAIL_Z = 1.15

BOTTOM_STOP_W = 0.09
BOTTOM_STOP_D = 0.10
BOTTOM_STOP_H = 0.035
BOTTOM_STOP_Z = 0.335

TOP_STOP_W = 0.09
TOP_STOP_D = 0.10
TOP_STOP_H = 0.045
TOP_STOP_Z = 1.955

CARRIAGE_HOME_Z = 0.86
CARRIAGE_TRAVEL = 0.60

FACEPLATE_W = 0.90
FACEPLATE_D = 0.03
FACEPLATE_H = 1.10
FACEPLATE_Y = 0.110
FACEPLATE_Z = 0.05

RIB_W = 0.06
RIB_D = 0.024
RIB_H = 0.86
RIB_Y = 0.135
RIB_Z = 0.04

CHANNEL_WEB_X = 0.25
CHANNEL_CENTER_Y = 0.05
CHANNEL_CENTER_Z = 0.00
CHANNEL_H = 1.02
CHANNEL_WEB_T = 0.02
CHANNEL_DEPTH = 0.09
CHANNEL_FLANGE_W = 0.055
CHANNEL_FLANGE_T = 0.018

TOP_BEAM_W = 0.56
TOP_BEAM_D = 0.08
TOP_BEAM_H = 0.09
TOP_BEAM_Y = 0.132
TOP_BEAM_Z = 0.48

MID_BEAM_W = 0.56
MID_BEAM_D = 0.08
MID_BEAM_H = 0.07
MID_BEAM_Y = 0.132
MID_BEAM_Z = 0.02

LOWER_BEAM_W = 0.62
LOWER_BEAM_D = 0.10
LOWER_BEAM_H = 0.14
LOWER_BEAM_Y = 0.145
LOWER_BEAM_Z = -0.32

MODULE_X = 0.255
MODULE_Y = 0.00
MODULE_W = 0.09
MODULE_D = 0.09
MODULE_H = 0.22
TOP_MODULE_Z = 0.30
BOTTOM_MODULE_Z = -0.32

KEEPER_X = 0.303
KEEPER_Y = 0.045
KEEPER_W = 0.024
KEEPER_D = 0.026
KEEPER_H = 0.20

WHEEL_X = 0.288
WHEEL_RADIUS = 0.032
WHEEL_LENGTH = 0.055
WHEEL_BOLT_RADIUS = 0.018
WHEEL_BOLT_LENGTH = 0.012
WHEEL_BOLT_Y = 0.032

ARM_X = 0.24
ARM_Y = 0.29
ARM_Z = -0.43
ARM_W = 0.16
ARM_D = 0.30
ARM_H = 0.10

ARM_WEAR_W = 0.148
ARM_WEAR_D = 0.23
ARM_WEAR_H = 0.014
ARM_WEAR_Y = 0.31
ARM_WEAR_Z = -0.386

GUSSET_X_OFFSET = 0.050
GUSSET_Y = 0.19
GUSSET_Z = -0.36
GUSSET_W = 0.028
GUSSET_D = 0.17
GUSSET_H = 0.17
GUSSET_PITCH = -0.75


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    *,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    *,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_box_upright(
    part,
    *,
    prefix: str,
    x: float,
    y: float,
    bottom_z: float,
    width: float,
    depth: float,
    height: float,
    wall: float,
    material: str,
) -> None:
    z = bottom_z + (height / 2.0)
    span_x = (width / 2.0) - (wall / 2.0)
    span_y = (depth / 2.0) - (wall / 2.0)
    face_width = width - (2.0 * wall) + JOIN_OVERLAP

    _add_box(
        part,
        (wall + JOIN_OVERLAP, depth, height),
        (x - span_x, y, z),
        material,
        name=f"{prefix}_outer_wall",
    )
    _add_box(
        part,
        (wall + JOIN_OVERLAP, depth, height),
        (x + span_x, y, z),
        material,
        name=f"{prefix}_inner_wall",
    )
    _add_box(
        part,
        (face_width, wall + JOIN_OVERLAP, height),
        (x, y + span_y, z),
        material,
        name=f"{prefix}_front_wall",
    )
    _add_box(
        part,
        (face_width, wall + JOIN_OVERLAP, height),
        (x, y - span_y, z),
        material,
        name=f"{prefix}_rear_wall",
    )


def _add_vertical_channel(
    part,
    *,
    prefix: str,
    sign: float,
    material: str,
) -> None:
    web_x = sign * CHANNEL_WEB_X
    flange_x = sign * (
        CHANNEL_WEB_X - (CHANNEL_WEB_T / 2.0) - (CHANNEL_FLANGE_W / 2.0) + (JOIN_OVERLAP / 2.0)
    )
    front_y = CHANNEL_CENTER_Y + (CHANNEL_DEPTH / 2.0) - (CHANNEL_FLANGE_T / 2.0)
    rear_y = CHANNEL_CENTER_Y - (CHANNEL_DEPTH / 2.0) + (CHANNEL_FLANGE_T / 2.0)

    _add_box(
        part,
        (CHANNEL_WEB_T, CHANNEL_DEPTH, CHANNEL_H),
        (web_x, CHANNEL_CENTER_Y, CHANNEL_CENTER_Z),
        material,
        name=f"{prefix}_channel_web",
    )
    _add_box(
        part,
        (CHANNEL_FLANGE_W, CHANNEL_FLANGE_T + JOIN_OVERLAP, CHANNEL_H),
        (flange_x, front_y, CHANNEL_CENTER_Z),
        material,
        name=f"{prefix}_channel_front_flange",
    )
    _add_box(
        part,
        (CHANNEL_FLANGE_W, CHANNEL_FLANGE_T + JOIN_OVERLAP, CHANNEL_H),
        (flange_x, rear_y, CHANNEL_CENTER_Z),
        material,
        name=f"{prefix}_channel_rear_flange",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="die_handling_lift_mast")

    model.material("mast_paint", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("carriage_paint", rgba=(0.88, 0.58, 0.16, 1.0))
    model.material("rail_steel", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("roller_black", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("bolt_zinc", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("wear_steel", rgba=(0.46, 0.48, 0.50, 1.0))

    mast = model.part("mast")
    _add_box(
        mast,
        (BASE_FOOT_W, BASE_FOOT_D, BASE_FOOT_H),
        (0.0, 0.0, BASE_FOOT_H / 2.0),
        "mast_paint",
        name="base_foot",
    )
    _add_box(
        mast,
        (BASE_BEAM_W, BASE_BEAM_D, BASE_BEAM_H),
        (0.0, 0.0, BASE_BEAM_Z),
        "mast_paint",
        name="base_beam",
    )
    _add_box_upright(
        mast,
        prefix="left_upright",
        x=-UPRIGHT_X,
        y=0.0,
        bottom_z=UPRIGHT_BOTTOM_Z,
        width=UPRIGHT_OUTER_W,
        depth=UPRIGHT_OUTER_D,
        height=UPRIGHT_H,
        wall=UPRIGHT_WALL,
        material="mast_paint",
    )
    _add_box_upright(
        mast,
        prefix="right_upright",
        x=UPRIGHT_X,
        y=0.0,
        bottom_z=UPRIGHT_BOTTOM_Z,
        width=UPRIGHT_OUTER_W,
        depth=UPRIGHT_OUTER_D,
        height=UPRIGHT_H,
        wall=UPRIGHT_WALL,
        material="mast_paint",
    )
    _add_box(
        mast,
        (HEADER_W, HEADER_D, HEADER_H),
        (0.0, 0.0, HEADER_Z),
        "mast_paint",
        name="top_header",
    )
    _add_box(
        mast,
        (MID_TIE_W, MID_TIE_D, MID_TIE_H),
        (0.0, MID_TIE_Y, MID_TIE_Z),
        "mast_paint",
        name="mid_tie_beam",
    )
    _add_box(
        mast,
        (GUIDE_RAIL_W, GUIDE_RAIL_D, GUIDE_RAIL_H),
        (-GUIDE_RAIL_X, 0.0, GUIDE_RAIL_Z),
        "rail_steel",
        name="left_guide_rail",
    )
    _add_box(
        mast,
        (GUIDE_RAIL_W, GUIDE_RAIL_D, GUIDE_RAIL_H),
        (GUIDE_RAIL_X, 0.0, GUIDE_RAIL_Z),
        "rail_steel",
        name="right_guide_rail",
    )
    _add_box(
        mast,
        (BOTTOM_STOP_W, BOTTOM_STOP_D, BOTTOM_STOP_H),
        (-GUIDE_RAIL_X, 0.0, BOTTOM_STOP_Z),
        "wear_steel",
        name="left_bottom_stop",
    )
    _add_box(
        mast,
        (BOTTOM_STOP_W, BOTTOM_STOP_D, BOTTOM_STOP_H),
        (GUIDE_RAIL_X, 0.0, BOTTOM_STOP_Z),
        "wear_steel",
        name="right_bottom_stop",
    )
    _add_box(
        mast,
        (TOP_STOP_W, TOP_STOP_D, TOP_STOP_H),
        (-GUIDE_RAIL_X, 0.0, TOP_STOP_Z),
        "wear_steel",
        name="left_top_stop",
    )
    _add_box(
        mast,
        (TOP_STOP_W, TOP_STOP_D, TOP_STOP_H),
        (GUIDE_RAIL_X, 0.0, TOP_STOP_Z),
        "wear_steel",
        name="right_top_stop",
    )
    _add_box(
        mast,
        (0.07, 0.05, 0.18),
        (-UPRIGHT_X, -0.055, 0.28),
        "mast_paint",
        name="left_base_gusset",
        rpy=(0.72, 0.0, 0.0),
    )
    _add_box(
        mast,
        (0.07, 0.05, 0.18),
        (UPRIGHT_X, -0.055, 0.28),
        "mast_paint",
        name="right_base_gusset",
        rpy=(0.72, 0.0, 0.0),
    )
    mast.inertial = Inertial.from_geometry(
        Box((BASE_FOOT_W, BASE_FOOT_D, 2.15)),
        mass=620.0,
        origin=Origin(xyz=(0.0, 0.0, 1.075)),
    )

    carriage = model.part("carriage")
    _add_box(
        carriage,
        (FACEPLATE_W, FACEPLATE_D, FACEPLATE_H),
        (0.0, FACEPLATE_Y, FACEPLATE_Z),
        "carriage_paint",
        name="faceplate",
    )
    _add_box(
        carriage,
        (RIB_W, RIB_D, RIB_H),
        (-0.22, RIB_Y, RIB_Z),
        "carriage_paint",
        name="left_faceplate_rib",
    )
    _add_box(
        carriage,
        (RIB_W, RIB_D, RIB_H),
        (0.0, RIB_Y, RIB_Z),
        "carriage_paint",
        name="center_faceplate_rib",
    )
    _add_box(
        carriage,
        (RIB_W, RIB_D, RIB_H),
        (0.22, RIB_Y, RIB_Z),
        "carriage_paint",
        name="right_faceplate_rib",
    )
    _add_vertical_channel(carriage, prefix="left", sign=-1.0, material="carriage_paint")
    _add_vertical_channel(carriage, prefix="right", sign=1.0, material="carriage_paint")
    _add_box(
        carriage,
        (TOP_BEAM_W, TOP_BEAM_D, TOP_BEAM_H),
        (0.0, TOP_BEAM_Y, TOP_BEAM_Z),
        "carriage_paint",
        name="top_beam",
    )
    _add_box(
        carriage,
        (MID_BEAM_W, MID_BEAM_D, MID_BEAM_H),
        (0.0, MID_BEAM_Y, MID_BEAM_Z),
        "carriage_paint",
        name="mid_beam",
    )
    _add_box(
        carriage,
        (LOWER_BEAM_W, LOWER_BEAM_D, LOWER_BEAM_H),
        (0.0, LOWER_BEAM_Y, LOWER_BEAM_Z),
        "carriage_paint",
        name="lower_beam",
    )
    _add_box(
        carriage,
        (0.36, 0.05, 0.28),
        (0.0, 0.118, -0.24),
        "carriage_paint",
        name="center_root_stiffener",
    )

    for prefix, sign in (("left", -1.0), ("right", 1.0)):
        for level_name, z in (("top", TOP_MODULE_Z), ("bottom", BOTTOM_MODULE_Z)):
            _add_box(
                carriage,
                (MODULE_W, MODULE_D, MODULE_H),
                (sign * MODULE_X, MODULE_Y, z),
                "carriage_paint",
                name=f"{prefix}_{level_name}_bracket",
            )
            _add_box(
                carriage,
                (KEEPER_W, KEEPER_D, KEEPER_H),
                (sign * KEEPER_X, KEEPER_Y, z),
                "wear_steel",
                name=f"{prefix}_{level_name}_front_keeper",
            )
            _add_box(
                carriage,
                (KEEPER_W, KEEPER_D, KEEPER_H),
                (sign * KEEPER_X, -KEEPER_Y, z),
                "wear_steel",
                name=f"{prefix}_{level_name}_rear_keeper",
            )
            _add_cylinder(
                carriage,
                WHEEL_RADIUS,
                WHEEL_LENGTH,
                (sign * WHEEL_X, 0.0, z),
                "roller_black",
                name=f"{prefix}_{level_name}_wheel",
                rpy=(pi / 2.0, 0.0, 0.0),
            )
            _add_cylinder(
                carriage,
                WHEEL_BOLT_RADIUS,
                WHEEL_BOLT_LENGTH,
                (sign * WHEEL_X, WHEEL_BOLT_Y, z),
                "bolt_zinc",
                name=f"{prefix}_{level_name}_bolt_front",
                rpy=(pi / 2.0, 0.0, 0.0),
            )
            _add_cylinder(
                carriage,
                WHEEL_BOLT_RADIUS,
                WHEEL_BOLT_LENGTH,
                (sign * WHEEL_X, -WHEEL_BOLT_Y, z),
                "bolt_zinc",
                name=f"{prefix}_{level_name}_bolt_rear",
                rpy=(pi / 2.0, 0.0, 0.0),
            )

    _add_box(
        carriage,
        (ARM_W, ARM_D, ARM_H),
        (-ARM_X, ARM_Y, ARM_Z),
        "carriage_paint",
        name="left_arm",
    )
    _add_box(
        carriage,
        (ARM_W, ARM_D, ARM_H),
        (ARM_X, ARM_Y, ARM_Z),
        "carriage_paint",
        name="right_arm",
    )
    _add_box(
        carriage,
        (ARM_WEAR_W, ARM_WEAR_D, ARM_WEAR_H),
        (-ARM_X, ARM_WEAR_Y, ARM_WEAR_Z),
        "wear_steel",
        name="left_arm_wear",
    )
    _add_box(
        carriage,
        (ARM_WEAR_W, ARM_WEAR_D, ARM_WEAR_H),
        (ARM_X, ARM_WEAR_Y, ARM_WEAR_Z),
        "wear_steel",
        name="right_arm_wear",
    )
    for arm_sign in (-1.0, 1.0):
        arm_center_x = arm_sign * ARM_X
        for rib_sign in (-1.0, 1.0):
            _add_box(
                carriage,
                (GUSSET_W, GUSSET_D, GUSSET_H),
                (arm_center_x + (rib_sign * GUSSET_X_OFFSET), GUSSET_Y, GUSSET_Z),
                "carriage_paint",
                name=(
                    f"{'left' if arm_sign < 0.0 else 'right'}_"
                    f"{'outer' if rib_sign == arm_sign else 'inner'}_gusset"
                ),
                rpy=(GUSSET_PITCH, 0.0, 0.0),
            )

    carriage.inertial = Inertial.from_geometry(
        Box((0.90, 0.48, 1.10)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.20, 0.05)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8000.0,
            velocity=0.25,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=20)

    lift_limits = lift.motion_limits
    ctx.check(
        "mast lift is a vertical prismatic joint",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in lift.axis) == (0.0, 0.0, 1.0)
        and lift_limits is not None
        and lift_limits.lower == 0.0
        and abs((lift_limits.upper or 0.0) - CARRIAGE_TRAVEL) < 1e-9,
        details="Expected one vertical prismatic carriage lift with the full mast travel range.",
    )

    ctx.expect_overlap(
        carriage,
        mast,
        axes="x",
        min_overlap=0.85,
        name="carriage faceplate spans the mast width",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a="left_top_wheel",
        elem_b="left_guide_rail",
        contact_tol=0.0005,
        name="left guide wheel bears on the rail",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a="right_top_wheel",
        elem_b="right_guide_rail",
        contact_tol=0.0005,
        name="right guide wheel bears on the rail",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="y",
        positive_elem="left_arm",
        negative_elem="base_beam",
        min_gap=0.02,
        max_gap=0.05,
        name="stub load arm starts ahead of the mast base beam",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_gap(
            carriage,
            mast,
            axis="z",
            positive_elem="left_bottom_bracket",
            negative_elem="left_bottom_stop",
            min_gap=0.05,
            max_gap=0.10,
            name="left lower stop keeps the carriage grounded without collision",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="z",
            positive_elem="right_bottom_bracket",
            negative_elem="right_bottom_stop",
            min_gap=0.05,
            max_gap=0.10,
            name="right lower stop keeps the carriage grounded without collision",
        )

    with ctx.pose({lift: CARRIAGE_TRAVEL}):
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem="left_top_stop",
            negative_elem="left_top_bracket",
            min_gap=0.03,
            max_gap=0.08,
            name="left upper stop clears the lifted carriage",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem="right_top_stop",
            negative_elem="right_top_bracket",
            min_gap=0.03,
            max_gap=0.08,
            name="right upper stop clears the lifted carriage",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
