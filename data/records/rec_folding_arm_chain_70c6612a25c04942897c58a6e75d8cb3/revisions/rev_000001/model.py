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


PIN_RADIUS = 0.0035
EAR_THICKNESS = 0.004
FORK_GAP = 0.012
FORK_WIDTH = FORK_GAP + 2.0 * EAR_THICKNESS
EAR_Y_CENTER = FORK_GAP / 2.0 + EAR_THICKNESS / 2.0

BEAM_WIDTH = 0.016
BEAM_HEIGHT = 0.012

TONGUE_BACKSET = 0.002
TONGUE_LENGTH = 0.024
TONGUE_THICKNESS = 0.012
TONGUE_HEIGHT = 0.016

CLEVIS_BODY_LENGTH = 0.028
CLEVIS_BODY_HEIGHT = 0.014
CLEVIS_ARM_LENGTH = 0.022
CLEVIS_ARM_HEIGHT = 0.022
ROOT_EAR_LENGTH = 0.018
ROOT_EAR_HEIGHT = 0.028

LOWER_LINK_LENGTH = 0.175
MIDDLE_LINK_LENGTH = 0.160
OUTER_LINK_LENGTH = 0.120


def _box_solid(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Shape:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center).val()


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Shape:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1] + length / 2.0, center[2]))
        .val()
    )


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Shape:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] + length / 2.0, center[1], center[2]))
        .val()
    )


def _union_all(solids: list[cq.Shape]) -> cq.Shape:
    result = solids[0]
    for solid in solids[1:]:
        result = result.fuse(solid)
    return result


def _make_link_tongue() -> cq.Shape:
    tongue = _box_solid(
        (TONGUE_LENGTH, TONGUE_THICKNESS, TONGUE_HEIGHT),
        ((TONGUE_LENGTH / 2.0) - TONGUE_BACKSET, 0.0, 0.0),
    )
    return tongue.cut(_cyl_y(PIN_RADIUS, FORK_WIDTH + 0.008, (0.0, 0.0, 0.0)))


def _make_distal_clevis(length: float) -> cq.Shape:
    clevis_body = _box_solid(
        (CLEVIS_BODY_LENGTH, BEAM_WIDTH, CLEVIS_BODY_HEIGHT),
        (length - 0.032, 0.0, 0.0),
    )
    left_arm = _box_solid(
        (CLEVIS_ARM_LENGTH, EAR_THICKNESS, CLEVIS_ARM_HEIGHT),
        (length - 0.011, -EAR_Y_CENTER, 0.0),
    )
    right_arm = _box_solid(
        (CLEVIS_ARM_LENGTH, EAR_THICKNESS, CLEVIS_ARM_HEIGHT),
        (length - 0.011, EAR_Y_CENTER, 0.0),
    )
    clevis = _union_all([clevis_body, left_arm, right_arm])
    return clevis.cut(_cyl_y(PIN_RADIUS, FORK_WIDTH + 0.008, (length, 0.0, 0.0)))


def _make_root_bracket() -> cq.Shape:
    back_plate = _box_solid((0.012, 0.060, 0.110), (-0.044, 0.0, 0.0))
    lower_foot = _box_solid((0.022, 0.046, 0.014), (-0.030, 0.0, -0.042))
    upper_tab = _box_solid((0.018, 0.040, 0.018), (-0.032, 0.0, 0.036))
    left_strut = _box_solid((0.036, 0.010, 0.026), (-0.020, -0.010, 0.0))
    right_strut = _box_solid((0.036, 0.010, 0.026), (-0.020, 0.010, 0.0))
    left_ear = _box_solid((ROOT_EAR_LENGTH, EAR_THICKNESS, ROOT_EAR_HEIGHT), (-0.009, -EAR_Y_CENTER, 0.0))
    right_ear = _box_solid((ROOT_EAR_LENGTH, EAR_THICKNESS, ROOT_EAR_HEIGHT), (-0.009, EAR_Y_CENTER, 0.0))

    bracket = _union_all(
        [
            back_plate,
            lower_foot,
            upper_tab,
            left_strut,
            right_strut,
            left_ear,
            right_ear,
        ]
    )

    bracket = bracket.cut(_cyl_y(PIN_RADIUS, FORK_WIDTH + 0.008, (0.0, 0.0, 0.0)))

    for hole_z in (-0.028, 0.028):
        bracket = bracket.cut(_cyl_x(0.0045, 0.012, (-0.033, 0.0, hole_z)))

    return bracket


def _make_clevis_link(length: float) -> cq.Shape:
    beam_start = TONGUE_LENGTH - TONGUE_BACKSET
    beam_end = length - 0.043
    beam_len = beam_end - beam_start

    prox_tongue = _make_link_tongue()
    main_beam = _box_solid(
        (beam_len, BEAM_WIDTH, BEAM_HEIGHT),
        (beam_start + beam_len / 2.0, 0.0, 0.0),
    )
    distal_clevis = _make_distal_clevis(length)

    return _union_all([prox_tongue, main_beam, distal_clevis])


def _make_outer_arm() -> cq.Shape:
    beam_start = TONGUE_LENGTH - TONGUE_BACKSET
    beam_len = OUTER_LINK_LENGTH - beam_start

    prox_tongue = _make_link_tongue()
    main_beam = _box_solid(
        (beam_len, BEAM_WIDTH, BEAM_HEIGHT),
        (beam_start + beam_len / 2.0, 0.0, 0.0),
    )
    tip_stem = _box_solid((0.034, 0.012, 0.010), (OUTER_LINK_LENGTH + 0.006, 0.0, 0.0))

    return _union_all([prox_tongue, main_beam, tip_stem])


def _make_tip_pad() -> cq.Shape:
    return (
        cq.Workplane("XY")
        .box(0.044, 0.030, 0.008)
        .edges("|Z")
        .fillet(0.003)
        .translate((OUTER_LINK_LENGTH + 0.040, 0.0, 0.0))
        .val()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="task_light_folding_arm")

    model.material("bracket_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("arm_silver", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("pad_gray", rgba=(0.38, 0.40, 0.42, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(_make_root_bracket(), "root_bracket_mesh"),
        material="bracket_black",
        name="root_bracket_shell",
    )
    root_bracket.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.105)),
        mass=0.55,
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
    )

    lower_link = model.part("lower_link")
    lower_link.visual(
        mesh_from_cadquery(_make_clevis_link(LOWER_LINK_LENGTH), "lower_link_mesh"),
        material="arm_silver",
        name="lower_link_shell",
    )
    lower_link.inertial = Inertial.from_geometry(
        Box((LOWER_LINK_LENGTH, BEAM_WIDTH, 0.028)),
        mass=0.24,
        origin=Origin(xyz=(LOWER_LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    middle_link = model.part("middle_link")
    middle_link.visual(
        mesh_from_cadquery(_make_clevis_link(MIDDLE_LINK_LENGTH), "middle_link_mesh"),
        material="arm_silver",
        name="middle_link_shell",
    )
    middle_link.inertial = Inertial.from_geometry(
        Box((MIDDLE_LINK_LENGTH, BEAM_WIDTH, 0.028)),
        mass=0.21,
        origin=Origin(xyz=(MIDDLE_LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    outer_link = model.part("outer_link")
    outer_link.visual(
        mesh_from_cadquery(_make_outer_arm(), "outer_link_arm_mesh"),
        material="arm_silver",
        name="outer_link_arm",
    )
    outer_link.visual(
        mesh_from_cadquery(_make_tip_pad(), "outer_tip_pad_mesh"),
        material="pad_gray",
        name="outer_tip_pad",
    )
    outer_link.inertial = Inertial.from_geometry(
        Box((OUTER_LINK_LENGTH + 0.070, 0.030, 0.020)),
        mass=0.19,
        origin=Origin(xyz=(OUTER_LINK_LENGTH / 2.0 + 0.020, 0.0, 0.0)),
    )

    model.articulation(
        "root_to_lower",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=lower_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=1.35, effort=16.0, velocity=1.8),
    )
    model.articulation(
        "lower_to_middle",
        ArticulationType.REVOLUTE,
        parent=lower_link,
        child=middle_link,
        origin=Origin(xyz=(LOWER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.10, upper=2.35, effort=12.0, velocity=2.0),
    )
    model.articulation(
        "middle_to_outer",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=outer_link,
        origin=Origin(xyz=(MIDDLE_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.10, upper=2.10, effort=8.0, velocity=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    root_bracket = object_model.get_part("root_bracket")
    lower_link = object_model.get_part("lower_link")
    middle_link = object_model.get_part("middle_link")
    outer_link = object_model.get_part("outer_link")

    root_to_lower = object_model.get_articulation("root_to_lower")
    lower_to_middle = object_model.get_articulation("lower_to_middle")
    middle_to_outer = object_model.get_articulation("middle_to_outer")

    for joint_name, joint in (
        ("root hinge", root_to_lower),
        ("middle hinge", lower_to_middle),
        ("outer hinge", middle_to_outer),
    ):
        ctx.check(
            f"{joint_name} rotates in the arm plane",
            tuple(round(value, 6) for value in joint.axis) == (0.0, -1.0, 0.0),
            details=f"axis={joint.axis}",
        )

    with ctx.pose({root_to_lower: 0.0, lower_to_middle: 0.0, middle_to_outer: 0.0}):
        lower_pos = ctx.part_world_position(lower_link)
        middle_pos = ctx.part_world_position(middle_link)
        outer_pos = ctx.part_world_position(outer_link)

        ctx.expect_origin_distance(
            outer_link,
            root_bracket,
            axes="xz",
            min_dist=0.32,
            max_dist=0.36,
            name="extended arm reaches outward from the bracket",
        )
        ctx.check(
            "extended arm origins march outward along x",
            lower_pos is not None
            and middle_pos is not None
            and outer_pos is not None
            and lower_pos[0] > -0.001
            and middle_pos[0] > lower_pos[0] + 0.16
            and outer_pos[0] > middle_pos[0] + 0.14,
            details=f"lower={lower_pos}, middle={middle_pos}, outer={outer_pos}",
        )
        ctx.check(
            "extended arm stays in a single xz plane",
            lower_pos is not None
            and middle_pos is not None
            and outer_pos is not None
            and abs(lower_pos[1]) < 1e-6
            and abs(middle_pos[1]) < 1e-6
            and abs(outer_pos[1]) < 1e-6
            and isclose(lower_pos[2], 0.0, abs_tol=1e-6)
            and isclose(middle_pos[2], 0.0, abs_tol=1e-6)
            and isclose(outer_pos[2], 0.0, abs_tol=1e-6),
            details=f"lower={lower_pos}, middle={middle_pos}, outer={outer_pos}",
        )

    folded_pose = {
        root_to_lower: 1.15,
        lower_to_middle: 2.25,
        middle_to_outer: 1.75,
    }
    with ctx.pose(folded_pose):
        lower_pos = ctx.part_world_position(lower_link)
        middle_pos = ctx.part_world_position(middle_link)
        outer_pos = ctx.part_world_position(outer_link)

        ctx.expect_origin_distance(
            outer_link,
            root_bracket,
            axes="xz",
            max_dist=0.18,
            name="folded arm tucks back near the root bracket",
        )
        ctx.check(
            "folded arm still stays in the same motion plane",
            lower_pos is not None
            and middle_pos is not None
            and outer_pos is not None
            and abs(lower_pos[1]) < 1e-6
            and abs(middle_pos[1]) < 1e-6
            and abs(outer_pos[1]) < 1e-6,
            details=f"lower={lower_pos}, middle={middle_pos}, outer={outer_pos}",
        )
        ctx.check(
            "folded arm wraps above and back toward the bracket",
            outer_pos is not None and outer_pos[0] < 0.02 and outer_pos[2] > 0.08,
            details=f"outer={outer_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
