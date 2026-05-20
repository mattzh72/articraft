from __future__ import annotations

from pathlib import Path

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent

BASE_DECK_HEIGHT = 0.14
BALLAST_PACK_HEIGHT = 0.06
SOCKET_HEIGHT = 0.18

LOWER_STAGE_LENGTH = 1.16
UPPER_STAGE_LENGTH = 0.90
LOWER_TO_UPPER_START = 0.36
MAST_EXTENSION = 0.42

SOCKET_OUTER = 0.18
SOCKET_INNER = 0.148
LOWER_STAGE_OUTER = 0.14
LOWER_STAGE_INNER = 0.108
UPPER_STAGE_OUTER = 0.10
UPPER_STAGE_INNER = 0.072
UPPER_TOP_CAP_THICKNESS = 0.014

LOWER_GUIDE_HEIGHT = 0.12
UPPER_GUIDE_HEIGHT = 0.14

BRACKET_AXIS_HEIGHT = 0.17
BRACKET_CHEEK_CENTER_Y = 0.170
BRACKET_CHEEK_THICKNESS = 0.012

TRUNNION_RADIUS = 0.016
TRUNNION_LENGTH = 0.032
TRUNNION_CENTER_Y = 0.148

HEAD_TILT_LOWER = -0.60
HEAD_TILT_UPPER = 0.70


def _add_square_tube(
    part,
    *,
    outer: float,
    inner: float,
    height: float,
    material,
    name_prefix: str,
    bottom_z: float = 0.0,
) -> None:
    wall = (outer - inner) * 0.5
    side_center = inner * 0.5 + wall * 0.5
    mid_z = bottom_z + height * 0.5

    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(side_center, 0.0, mid_z)),
        material=material,
        name=f"{name_prefix}_right_wall",
    )
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(-side_center, 0.0, mid_z)),
        material=material,
        name=f"{name_prefix}_left_wall",
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, side_center, mid_z)),
        material=material,
        name=f"{name_prefix}_front_wall",
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, -side_center, mid_z)),
        material=material,
        name=f"{name_prefix}_rear_wall",
    )


def _add_guide_pads(
    part,
    *,
    carrier_outer: float,
    target_inner: float,
    height: float,
    material,
    name_prefix: str,
    bottom_z: float = 0.0,
) -> None:
    pad = (target_inner - carrier_outer) * 0.5
    if pad <= 0.0:
        return

    side_center = carrier_outer * 0.5 + pad * 0.5
    mid_z = bottom_z + height * 0.5
    span = carrier_outer * 0.72

    part.visual(
        Box((pad, span, height)),
        origin=Origin(xyz=(side_center, 0.0, mid_z)),
        material=material,
        name=f"{name_prefix}_right_pad",
    )
    part.visual(
        Box((pad, span, height)),
        origin=Origin(xyz=(-side_center, 0.0, mid_z)),
        material=material,
        name=f"{name_prefix}_left_pad",
    )
    part.visual(
        Box((span, pad, height)),
        origin=Origin(xyz=(0.0, side_center, mid_z)),
        material=material,
        name=f"{name_prefix}_front_pad",
    )
    part.visual(
        Box((span, pad, height)),
        origin=Origin(xyz=(0.0, -side_center, mid_z)),
        material=material,
        name=f"{name_prefix}_rear_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_floodlight_mast", assets=ASSETS)

    ballast = model.material("ballast", rgba=(0.25, 0.27, 0.29, 1.0))
    ballast_dark = model.material("ballast_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.84, 0.84, 0.82, 1.0))
    head_housing = model.material("head_housing", rgba=(0.16, 0.17, 0.18, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.77, 0.84, 0.90, 0.55))

    base = model.part("base")
    base.visual(
        Box((1.02, 0.70, BASE_DECK_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_DECK_HEIGHT * 0.5)),
        material=ballast,
        name="deck",
    )
    base.visual(
        Box((0.30, 0.26, BALLAST_PACK_HEIGHT)),
        origin=Origin(
            xyz=(-0.27, 0.0, BASE_DECK_HEIGHT + BALLAST_PACK_HEIGHT * 0.5)
        ),
        material=ballast_dark,
        name="left_ballast_pack",
    )
    base.visual(
        Box((0.30, 0.26, BALLAST_PACK_HEIGHT)),
        origin=Origin(
            xyz=(0.27, 0.0, BASE_DECK_HEIGHT + BALLAST_PACK_HEIGHT * 0.5)
        ),
        material=ballast_dark,
        name="right_ballast_pack",
    )
    base.visual(
        Box((0.09, 0.09, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, BASE_DECK_HEIGHT + 0.02)),
        material=ballast_dark,
        name="mast_plinth",
    )
    _add_square_tube(
        base,
        outer=SOCKET_OUTER,
        inner=SOCKET_INNER,
        height=SOCKET_HEIGHT,
        material=painted_steel,
        name_prefix="mast_socket",
        bottom_z=BASE_DECK_HEIGHT,
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            base.visual(
                Box((0.10, 0.07, 0.02)),
                origin=Origin(xyz=(0.35 * x_sign, 0.24 * y_sign, 0.01)),
                material=ballast_dark,
            )
    base.inertial = Inertial.from_geometry(
        Box((1.02, 0.70, 0.22)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    lower_stage = model.part("lower_stage")
    _add_square_tube(
        lower_stage,
        outer=LOWER_STAGE_OUTER,
        inner=LOWER_STAGE_INNER,
        height=LOWER_STAGE_LENGTH,
        material=painted_steel,
        name_prefix="lower_shell",
    )
    _add_guide_pads(
        lower_stage,
        carrier_outer=LOWER_STAGE_OUTER,
        target_inner=SOCKET_INNER,
        height=LOWER_GUIDE_HEIGHT,
        material=steel,
        name_prefix="socket_guides",
        bottom_z=0.02,
    )
    lower_stage.inertial = Inertial.from_geometry(
        Box((SOCKET_INNER, SOCKET_INNER, LOWER_STAGE_LENGTH)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STAGE_LENGTH * 0.5)),
    )

    upper_stage = model.part("upper_stage")
    _add_square_tube(
        upper_stage,
        outer=UPPER_STAGE_OUTER,
        inner=UPPER_STAGE_INNER,
        height=UPPER_STAGE_LENGTH,
        material=steel,
        name_prefix="upper_shell",
    )
    _add_guide_pads(
        upper_stage,
        carrier_outer=UPPER_STAGE_OUTER,
        target_inner=LOWER_STAGE_INNER,
        height=UPPER_GUIDE_HEIGHT,
        material=steel,
        name_prefix="stage_guides",
        bottom_z=0.02,
    )
    upper_stage.visual(
        Box((0.14, 0.14, UPPER_TOP_CAP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, UPPER_STAGE_LENGTH + UPPER_TOP_CAP_THICKNESS * 0.5)
        ),
        material=painted_steel,
        name="top_cap",
    )
    upper_stage.inertial = Inertial.from_geometry(
        Box((0.14, 0.14, UPPER_STAGE_LENGTH + UPPER_TOP_CAP_THICKNESS)),
        mass=6.0,
        origin=Origin(
            xyz=(0.0, 0.0, (UPPER_STAGE_LENGTH + UPPER_TOP_CAP_THICKNESS) * 0.5)
        ),
    )

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.16, 0.14, 0.016)),
        origin=Origin(xyz=(-0.02, 0.0, 0.008)),
        material=head_housing,
        name="base_plate",
    )
    bracket.visual(
        Box((0.032, 0.110, 0.10)),
        origin=Origin(xyz=(-0.074, -0.120, 0.066)),
        material=painted_steel,
        name="left_gusset",
    )
    bracket.visual(
        Box((0.032, 0.110, 0.10)),
        origin=Origin(xyz=(-0.074, 0.120, 0.066)),
        material=painted_steel,
        name="right_gusset",
    )
    bracket.visual(
        Box((0.012, 0.34, 0.014)),
        origin=Origin(xyz=(-0.054, 0.0, 0.301)),
        material=painted_steel,
        name="top_tie",
    )
    bracket.visual(
        Box((0.080, BRACKET_CHEEK_THICKNESS, 0.28)),
        origin=Origin(xyz=(-0.020, -BRACKET_CHEEK_CENTER_Y, BRACKET_AXIS_HEIGHT)),
        material=painted_steel,
        name="left_cheek",
    )
    bracket.visual(
        Box((0.080, BRACKET_CHEEK_THICKNESS, 0.28)),
        origin=Origin(xyz=(-0.020, BRACKET_CHEEK_CENTER_Y, BRACKET_AXIS_HEIGHT)),
        material=painted_steel,
        name="right_cheek",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.16, 0.36, 0.31)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    head = model.part("head")
    head.visual(
        Box((0.18, 0.24, 0.14)),
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
        material=head_housing,
        name="housing",
    )
    head.visual(
        Box((0.032, 0.20, 0.10)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=head_housing,
        name="rear_cap",
    )
    head.visual(
        Box((0.018, 0.22, 0.12)),
        origin=Origin(xyz=(0.171, 0.0, 0.0)),
        material=painted_steel,
        name="bezel",
    )
    head.visual(
        Box((0.006, 0.19, 0.09)),
        origin=Origin(xyz=(0.183, 0.0, 0.0)),
        material=lens_glass,
        name="lens",
    )
    head.visual(
        Box((0.070, 0.24, 0.018)),
        origin=Origin(xyz=(0.145, 0.0, 0.079)),
        material=head_housing,
        name="visor",
    )
    head.visual(
        Box((0.030, 0.024, 0.060)),
        origin=Origin(xyz=(0.008, -0.132, 0.0)),
        material=head_housing,
        name="left_boss",
    )
    head.visual(
        Box((0.030, 0.024, 0.060)),
        origin=Origin(xyz=(0.008, 0.132, 0.0)),
        material=head_housing,
        name="right_boss",
    )
    head.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, -TRUNNION_CENTER_Y, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=steel,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, TRUNNION_CENTER_Y, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=steel,
        name="right_trunnion",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.19, 0.32, 0.16)),
        mass=4.2,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.FIXED,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_DECK_HEIGHT)),
    )
    model.articulation(
        "lower_to_upper_stage",
        ArticulationType.PRISMATIC,
        parent=lower_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TO_UPPER_START)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.25,
            lower=0.0,
            upper=MAST_EXTENSION,
        ),
    )
    model.articulation(
        "upper_stage_to_bracket",
        ArticulationType.FIXED,
        parent=upper_stage,
        child=bracket,
        origin=Origin(xyz=(0.0, 0.0, UPPER_STAGE_LENGTH + UPPER_TOP_CAP_THICKNESS)),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, BRACKET_AXIS_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=HEAD_TILT_LOWER,
            upper=HEAD_TILT_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    bracket = object_model.get_part("bracket")
    head = object_model.get_part("head")
    mast_extend = object_model.get_articulation("lower_to_upper_stage")
    head_tilt = object_model.get_articulation("head_tilt")

    deck = base.get_visual("deck")
    socket_right_wall = base.get_visual("mast_socket_right_wall")
    socket_front_wall = base.get_visual("mast_socket_front_wall")
    lower_shell_right = lower_stage.get_visual("lower_shell_right_wall")
    lower_shell_front = lower_stage.get_visual("lower_shell_front_wall")
    lower_socket_right_pad = lower_stage.get_visual("socket_guides_right_pad")
    lower_socket_front_pad = lower_stage.get_visual("socket_guides_front_pad")
    upper_stage_right_pad = upper_stage.get_visual("stage_guides_right_pad")
    upper_stage_front_pad = upper_stage.get_visual("stage_guides_front_pad")
    top_cap = upper_stage.get_visual("top_cap")
    base_plate = bracket.get_visual("base_plate")
    left_cheek = bracket.get_visual("left_cheek")
    right_cheek = bracket.get_visual("right_cheek")
    housing = head.get_visual("housing")
    lens = head.get_visual("lens")
    left_trunnion = head.get_visual("left_trunnion")
    right_trunnion = head.get_visual("right_trunnion")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=12)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.expect_origin_distance(lower_stage, base, axes="xy", max_dist=0.002)
    ctx.expect_contact(
        lower_stage,
        base,
        elem_a=lower_socket_right_pad,
        elem_b=socket_right_wall,
    )
    ctx.expect_contact(
        lower_stage,
        base,
        elem_a=lower_socket_front_pad,
        elem_b=socket_front_wall,
    )
    ctx.expect_origin_gap(
        upper_stage,
        lower_stage,
        axis="z",
        min_gap=LOWER_TO_UPPER_START - 0.001,
        max_gap=LOWER_TO_UPPER_START + 0.001,
    )
    ctx.expect_origin_distance(upper_stage, lower_stage, axes="xy", max_dist=0.002)
    ctx.expect_contact(
        upper_stage,
        lower_stage,
        elem_a=upper_stage_right_pad,
        elem_b=lower_shell_right,
    )
    ctx.expect_contact(
        upper_stage,
        lower_stage,
        elem_a=upper_stage_front_pad,
        elem_b=lower_shell_front,
    )
    ctx.expect_contact(bracket, upper_stage, elem_a=base_plate, elem_b=top_cap)
    ctx.expect_gap(
        head,
        upper_stage,
        axis="z",
        min_gap=0.10,
        positive_elem=housing,
        negative_elem=top_cap,
    )
    ctx.expect_contact(head, bracket, elem_a=left_trunnion, elem_b=left_cheek)
    ctx.expect_contact(head, bracket, elem_a=right_trunnion, elem_b=right_cheek)

    with ctx.pose({mast_extend: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="mast_retracted_no_overlap")
        ctx.fail_if_isolated_parts(name="mast_retracted_no_floating")

    with ctx.pose({mast_extend: MAST_EXTENSION}):
        ctx.fail_if_parts_overlap_in_current_pose(name="mast_extended_no_overlap")
        ctx.fail_if_isolated_parts(name="mast_extended_no_floating")
        ctx.expect_origin_gap(
            upper_stage,
            lower_stage,
            axis="z",
            min_gap=LOWER_TO_UPPER_START + MAST_EXTENSION - 0.001,
            max_gap=LOWER_TO_UPPER_START + MAST_EXTENSION + 0.001,
        )
        ctx.expect_origin_distance(upper_stage, lower_stage, axes="xy", max_dist=0.002)
        ctx.expect_contact(
            upper_stage,
            lower_stage,
            elem_a=upper_stage_right_pad,
            elem_b=lower_shell_right,
            name="extended_right_guide_contact",
        )
        ctx.expect_contact(
            upper_stage,
            lower_stage,
            elem_a=upper_stage_front_pad,
            elem_b=lower_shell_front,
            name="extended_front_guide_contact",
        )
        ctx.expect_gap(
            bracket,
            base,
            axis="z",
            min_gap=1.45,
            positive_elem=base_plate,
            negative_elem=deck,
        )
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=1.60,
            positive_elem=lens,
            negative_elem=deck,
        )

    with ctx.pose({head_tilt: HEAD_TILT_LOWER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="head_tilt_down_no_overlap")
        ctx.fail_if_isolated_parts(name="head_tilt_down_no_floating")
        ctx.expect_contact(head, bracket, elem_a=left_trunnion, elem_b=left_cheek)
        ctx.expect_contact(head, bracket, elem_a=right_trunnion, elem_b=right_cheek)
        ctx.expect_gap(
            head,
            upper_stage,
            axis="z",
            min_gap=0.008,
            positive_elem=housing,
            negative_elem=top_cap,
        )

    with ctx.pose({head_tilt: HEAD_TILT_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="head_tilt_up_no_overlap")
        ctx.fail_if_isolated_parts(name="head_tilt_up_no_floating")
        ctx.expect_contact(head, bracket, elem_a=left_trunnion, elem_b=left_cheek)
        ctx.expect_contact(head, bracket, elem_a=right_trunnion, elem_b=right_cheek)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
