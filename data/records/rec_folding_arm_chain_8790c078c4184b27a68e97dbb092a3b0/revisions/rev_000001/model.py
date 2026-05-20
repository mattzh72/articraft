from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


EYE_WIDTH = 0.008
BEAM_WIDTH = 0.012
BODY_HEIGHT = 0.022
ROOT_NUB_LEN = 0.016
DISTAL_NUB_LEN = 0.016
TRANSITION_LEN = 0.022
TIP_LEN = 0.030
BEAM_SIDE_OFFSET = 0.009
LINK_ENVELOPE_WIDTH = 2.0 * BEAM_SIDE_OFFSET + BEAM_WIDTH
ROOT_JOINT_X = 0.026
BRACKET_LUG_LEN = 0.018

LINK_1_LENGTH = 0.185
LINK_2_LENGTH = 0.160
LINK_3_LENGTH = 0.135

BRACKET_WIDTH = 0.072
BACK_PLATE_HEIGHT = 0.122
BACK_PLATE_THICKNESS = 0.010
FOOT_DEPTH = 0.060
FOOT_THICKNESS = 0.010
FOOT_Z = -0.063


def _beam_center(side: int) -> float:
    return side * BEAM_SIDE_OFFSET


def _box_bar(
    x0: float,
    x1: float,
    width: float,
    center_y: float,
    *,
    height: float = BODY_HEIGHT,
) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, width, height).translate(
        ((x0 + x1) / 2.0, center_y, 0.0)
    )


def _beam_slot(center_x: float, length: float, side: int) -> cq.Workplane:
    y_start = _beam_center(side) - BEAM_WIDTH / 2.0
    return (
        cq.Workplane("XZ")
        .slot2D(length, BODY_HEIGHT * 0.40)
        .extrude(BEAM_WIDTH)
        .translate((center_x, y_start, 0.0))
    )


def _link_shape(
    length: float,
    *,
    beam_side: int,
    tip_style: bool = False,
    slot_scale: float = 1.0,
) -> cq.Workplane:
    beam_y = _beam_center(beam_side)
    beam_start = ROOT_NUB_LEN + TRANSITION_LEN
    beam_end = length - (TIP_LEN if tip_style else (DISTAL_NUB_LEN + TRANSITION_LEN))

    root_nub = _box_bar(0.0, ROOT_NUB_LEN, EYE_WIDTH, 0.0)
    root_transition = _box_bar(
        ROOT_NUB_LEN,
        beam_start,
        abs(beam_y) + (BEAM_WIDTH + EYE_WIDTH) / 2.0,
        beam_y / 2.0,
        height=BODY_HEIGHT * 0.90,
    )
    beam = _box_bar(beam_start, beam_end, BEAM_WIDTH, beam_y, height=BODY_HEIGHT * 0.88)

    shape = root_nub.union(root_transition).union(beam)

    if tip_style:
        tip_body = _box_bar(length - TIP_LEN, length, BEAM_WIDTH, beam_y, height=BODY_HEIGHT * 0.86)
        tip_nose = (
            cq.Workplane("XY")
            .box(0.014, BEAM_WIDTH, BODY_HEIGHT * 0.70)
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -24.0)
            .translate((length - 0.007, beam_y, 0.0))
        )
        shape = shape.union(tip_body).union(tip_nose)
    else:
        distal_transition = _box_bar(
            beam_end,
            length - DISTAL_NUB_LEN,
            abs(beam_y) + (BEAM_WIDTH + EYE_WIDTH) / 2.0,
            beam_y / 2.0,
            height=BODY_HEIGHT * 0.90,
        )
        distal_nub = _box_bar(length - DISTAL_NUB_LEN, length, EYE_WIDTH, 0.0)
        shape = shape.union(distal_transition).union(distal_nub)

    slot_length = (beam_end - beam_start - 0.026) * slot_scale
    if slot_length > 0.040:
        shape = shape.cut(_beam_slot((beam_start + beam_end) / 2.0, slot_length, beam_side))

    return shape


def _base_bracket_shape() -> cq.Workplane:
    back_plate = cq.Workplane("XY").box(
        BACK_PLATE_THICKNESS, BRACKET_WIDTH, BACK_PLATE_HEIGHT
    ).translate((-0.036, 0.0, -0.006))
    foot = cq.Workplane("XY").box(FOOT_DEPTH, BRACKET_WIDTH, FOOT_THICKNESS).translate(
        (-0.010, 0.0, FOOT_Z)
    )
    front_lug = _box_bar(ROOT_JOINT_X - BRACKET_LUG_LEN, ROOT_JOINT_X, EYE_WIDTH, 0.0)
    spine = _box_bar(-0.036, ROOT_JOINT_X - BRACKET_LUG_LEN, EYE_WIDTH, 0.0, height=BODY_HEIGHT)
    side_cheek = _box_bar(-0.032, -0.010, BEAM_WIDTH, -0.012, height=0.054)
    center_post = _box_bar(-0.026, -0.012, EYE_WIDTH, 0.0, height=0.030).translate(
        (0.0, 0.0, -0.014)
    )
    gusset = (
        cq.Workplane("XZ")
        .polyline([(-0.034, FOOT_Z + 0.004), (-0.034, -0.014), (-0.004, FOOT_Z + 0.004)])
        .close()
        .extrude(BEAM_WIDTH)
        .translate((0.0, -0.018, 0.0))
    )
    center_web = (
        cq.Workplane("XZ")
        .polyline([(-0.034, FOOT_Z + 0.004), (-0.034, -0.020), (-0.014, FOOT_Z + 0.004)])
        .close()
        .extrude(BRACKET_WIDTH / 2.0, both=True)
    )

    bracket = (
        back_plate.union(foot)
        .union(front_lug)
        .union(spine)
        .union(side_cheek)
        .union(center_post)
        .union(gusset)
        .union(center_web)
    )

    plate_holes = (
        cq.Workplane("YZ")
        .pushPoints([(-0.025, 0.031), (0.025, 0.031), (-0.025, -0.031), (0.025, -0.031)])
        .circle(0.0045)
        .extrude(0.040, both=True)
    )
    foot_holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.014, -0.024), (0.014, -0.024), (-0.014, 0.024), (0.014, 0.024)])
        .circle(0.0045)
        .extrude(0.030, both=True)
        .translate((0.0, 0.0, FOOT_Z))
    )

    return bracket.cut(plate_holes).cut(foot_holes)


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    *,
    mesh_name: str,
    material: str,
    visual_name: str,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_chain")

    model.material("powder_coat", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("anodized_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("dark_bushing", rgba=(0.14, 0.15, 0.17, 1.0))

    base_bracket = model.part("base_bracket")
    _add_mesh_visual(
        base_bracket,
        _base_bracket_shape(),
        mesh_name="base_bracket",
        material="powder_coat",
        visual_name="bracket_shell",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((0.060, BRACKET_WIDTH, BACK_PLATE_HEIGHT)),
        mass=2.6,
        origin=Origin(xyz=(-0.010, 0.0, -0.006)),
    )

    link_1 = model.part("link_1")
    _add_mesh_visual(
        link_1,
        _link_shape(LINK_1_LENGTH, beam_side=1, slot_scale=1.0),
        mesh_name="link_1",
        material="anodized_aluminum",
        visual_name="link_1_shell",
    )
    link_1.inertial = Inertial.from_geometry(
        Box((LINK_1_LENGTH, LINK_ENVELOPE_WIDTH, BODY_HEIGHT)),
        mass=0.60,
        origin=Origin(xyz=(LINK_1_LENGTH / 2.0, 0.0, 0.0)),
    )

    link_2 = model.part("link_2")
    _add_mesh_visual(
        link_2,
        _link_shape(LINK_2_LENGTH, beam_side=-1, slot_scale=0.92),
        mesh_name="link_2",
        material="anodized_aluminum",
        visual_name="link_2_shell",
    )
    link_2.inertial = Inertial.from_geometry(
        Box((LINK_2_LENGTH, LINK_ENVELOPE_WIDTH, BODY_HEIGHT)),
        mass=0.50,
        origin=Origin(xyz=(LINK_2_LENGTH / 2.0, 0.0, 0.0)),
    )

    link_3 = model.part("link_3")
    _add_mesh_visual(
        link_3,
        _link_shape(LINK_3_LENGTH, beam_side=1, tip_style=True, slot_scale=0.78),
        mesh_name="link_3",
        material="dark_bushing",
        visual_name="link_3_shell",
    )
    link_3.inertial = Inertial.from_geometry(
        Box((LINK_3_LENGTH, EYE_WIDTH, BODY_HEIGHT)),
        mass=0.36,
        origin=Origin(xyz=(LINK_3_LENGTH / 2.0, 0.0, 0.0)),
    )

    common_axis = (0.0, -1.0, 0.0)

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=link_1,
        origin=Origin(xyz=(ROOT_JOINT_X, 0.0, 0.0)),
        axis=common_axis,
        motion_limits=MotionLimits(lower=-1.15, upper=1.35, effort=32.0, velocity=1.4),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=common_axis,
        motion_limits=MotionLimits(lower=-2.45, upper=2.45, effort=22.0, velocity=1.8),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=common_axis,
        motion_limits=MotionLimits(lower=-2.45, upper=2.45, effort=18.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    joint_1 = object_model.get_articulation("base_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")
    joint_3 = object_model.get_articulation("link_2_to_link_3")

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
        "three_parallel_planar_revolute_axes",
        (
            joint_1.axis == joint_2.axis == joint_3.axis == (0.0, -1.0, 0.0)
            and isclose(joint_1.origin.xyz[1], 0.0, abs_tol=1e-9)
            and isclose(joint_2.origin.xyz[1], 0.0, abs_tol=1e-9)
            and isclose(joint_3.origin.xyz[1], 0.0, abs_tol=1e-9)
            and isclose(joint_1.origin.xyz[2], 0.0, abs_tol=1e-9)
            and isclose(joint_2.origin.xyz[2], 0.0, abs_tol=1e-9)
            and isclose(joint_3.origin.xyz[2], 0.0, abs_tol=1e-9)
        ),
        "joint axes or offsets drift out of the common folding plane",
    )

    with ctx.pose({joint_1: 0.0, joint_2: 0.0, joint_3: 0.0}):
        ctx.expect_contact(base_bracket, link_1, name="root_bracket_supports_link_1")
        ctx.expect_contact(link_1, link_2, name="link_1_supports_link_2")
        ctx.expect_contact(link_2, link_3, name="link_2_supports_link_3")
        ctx.expect_origin_gap(
            link_2,
            link_1,
            axis="x",
            min_gap=LINK_1_LENGTH - 0.001,
            max_gap=LINK_1_LENGTH + 0.001,
            name="link_1_joint_spacing",
        )
        ctx.expect_origin_gap(
            link_3,
            link_2,
            axis="x",
            min_gap=LINK_2_LENGTH - 0.001,
            max_gap=LINK_2_LENGTH + 0.001,
            name="link_2_joint_spacing",
        )

    def _forward_tip_x(part) -> float | None:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return aabb[1][0]

    straight_tip_x = None
    folded_tip_x = None

    with ctx.pose({joint_1: 0.0, joint_2: 0.0, joint_3: 0.0}):
        straight_tip_x = _forward_tip_x(link_3)

    with ctx.pose({joint_1: 0.78, joint_2: 2.20, joint_3: -2.08}):
        folded_tip_x = _forward_tip_x(link_3)

    fold_ok = (
        straight_tip_x is not None
        and folded_tip_x is not None
        and straight_tip_x > 0.46
        and folded_tip_x < 0.28
        and folded_tip_x < straight_tip_x - 0.16
    )
    ctx.check(
        "arm_packs_close_then_reaches_straight",
        fold_ok,
        (
            f"straight_tip_x={straight_tip_x}, folded_tip_x={folded_tip_x}; "
            "expected a long straight reach and a much tighter folded envelope"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
