from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_LENGTH = 0.155
BASE_WIDTH = 0.060
BASE_DECK_THICK = 0.012
BASE_JOINT_X = -0.050
BASE_JOINT_Z = 0.066

FORK_GAP = 0.014
EAR_THICK = 0.008
EAR_LENGTH = 0.016
BOSS_LENGTH = 0.004
ROOT_BARREL_RADIUS = 0.0095
OUTER_BOSS_RADIUS = 0.0115

BODY_WIDTH = 0.010
BODY_THICK = 0.008
BRIDGE_WIDTH = 0.008
RIB_WIDTH = 0.003
RIB_HEIGHT = 0.003

STAGE_1_LENGTH = 0.118
STAGE_2_LENGTH = 0.112
STAGE_3_LENGTH = 0.102
STAGE_4_LENGTH = 0.088

STAGE_1_BODY_Z = 0.022
STAGE_2_BODY_Z = 0.034
STAGE_3_BODY_Z = 0.046
STAGE_4_BODY_Z = 0.058
JOINT_CENTER_Z = 0.0
TERMINAL_PAD_DROP = 0.018


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _extrude_xz_profile(
    points: list[tuple[float, float]],
    width: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(width / 2.0, both=True).translate(center)


def _fuse_all(*pieces: cq.Workplane) -> cq.Workplane:
    shape = pieces[0]
    for piece in pieces[1:]:
        shape = shape.union(piece)
    return shape


def _base_shoe_shape() -> cq.Workplane:
    deck = _box((0.118, BASE_WIDTH, BASE_DECK_THICK), (0.018, 0.0, BASE_DECK_THICK / 2.0))
    heel = _box((0.050, BASE_WIDTH, 0.020), (-0.064, 0.0, 0.010))

    nose_wedge = _extrude_xz_profile(
        [(0.0, 0.0), (0.030, 0.0), (0.030, 0.008), (0.010, 0.012), (0.0, 0.012)],
        BASE_WIDTH,
        (0.062, 0.0, 0.0),
    )

    side_rails = [
        _box((0.098, 0.007, 0.006), (0.012, y_pos, 0.015))
        for y_pos in (-0.019, 0.019)
    ]

    cheek_supports = [
        _box((0.020, EAR_THICK, 0.012), (BASE_JOINT_X - 0.003, y_pos, 0.020))
        for y_pos in (-(FORK_GAP / 2.0 + EAR_THICK / 2.0), FORK_GAP / 2.0 + EAR_THICK / 2.0)
    ]
    cheeks = [
        _box((0.020, EAR_THICK, 0.020), (BASE_JOINT_X, y_pos, BASE_JOINT_Z))
        for y_pos in (-(FORK_GAP / 2.0 + EAR_THICK / 2.0), FORK_GAP / 2.0 + EAR_THICK / 2.0)
    ]
    cheek_bosses = [
        _y_cylinder(
            OUTER_BOSS_RADIUS,
            BOSS_LENGTH,
            (
                BASE_JOINT_X,
                y_pos,
                BASE_JOINT_Z,
            ),
        )
        for y_pos in (
            -(FORK_GAP / 2.0 + EAR_THICK + BOSS_LENGTH / 2.0),
            FORK_GAP / 2.0 + EAR_THICK + BOSS_LENGTH / 2.0,
        )
    ]

    folded_stop = _extrude_xz_profile(
        [(0.0, 0.0), (0.014, 0.0), (0.006, 0.009)],
        0.014,
        (BASE_JOINT_X + 0.017, 0.0, 0.018),
    )

    rear_buttress = _extrude_xz_profile(
        [(0.0, 0.0), (0.022, 0.0), (0.022, 0.016), (0.009, 0.026), (0.0, 0.026)],
        0.018,
        (BASE_JOINT_X - 0.026, 0.0, 0.012),
    )

    fastener_bosses = []
    for x_pos in (-0.010, 0.040):
        for y_pos in (-0.018, 0.018):
            fastener_bosses.append(
                cq.Workplane("XY").circle(0.0055).extrude(0.002).translate((x_pos, y_pos, 0.012))
            )

    return _fuse_all(
        deck,
        heel,
        nose_wedge,
        *side_rails,
        *cheek_supports,
        *cheeks,
        *cheek_bosses,
        folded_stop,
        rear_buttress,
        *fastener_bosses,
    )


def _link_shape(length: float, body_z: float, tip_joint_z: float, direction: float) -> cq.Workplane:
    beam_len = length - 0.058
    beam_center_x = direction * (0.031 + beam_len / 2.0)
    rib_len = beam_len * 0.66
    rib_center_x = direction * (0.034 + rib_len / 2.0)
    body_bottom = body_z - BODY_THICK / 2.0
    web_bottom = ROOT_BARREL_RADIUS - 0.001
    web_height = max(0.008, body_bottom - web_bottom)
    root_barrel = _y_cylinder(ROOT_BARREL_RADIUS, FORK_GAP, (0.0, 0.0, 0.0))
    root_cap = _box((0.012, BODY_WIDTH, 0.006), (direction * 0.028, 0.0, body_bottom + 0.003))
    root_webs = [
        _box(
            (0.012, 0.003, web_height),
            (direction * 0.020, y_pos, web_bottom + web_height / 2.0),
        )
        for y_pos in (-0.005, 0.005)
    ]
    beam = _box((beam_len, BODY_WIDTH, BODY_THICK), (beam_center_x, 0.0, body_z))
    ribs = [
        _box((rib_len, RIB_WIDTH, RIB_HEIGHT), (rib_center_x, y_pos, body_z + BODY_THICK / 2.0 + RIB_HEIGHT / 2.0))
        for y_pos in (-0.005, 0.005)
    ]

    tip_side_z = (tip_joint_z + body_bottom) / 2.0
    tip_side_height = max(0.012, tip_joint_z - body_bottom + 0.010)
    side_struts = [
        _box(
            (0.014, EAR_THICK, tip_side_height),
            (direction * (length - 0.010), y_pos, tip_side_z),
        )
        for y_pos in (-(FORK_GAP / 2.0 + EAR_THICK / 2.0), FORK_GAP / 2.0 + EAR_THICK / 2.0)
    ]
    ears = [
        _box((EAR_LENGTH, EAR_THICK, 0.018), (direction * length, y_pos, tip_joint_z))
        for y_pos in (-(FORK_GAP / 2.0 + EAR_THICK / 2.0), FORK_GAP / 2.0 + EAR_THICK / 2.0)
    ]
    ear_bosses = [
        _y_cylinder(
            OUTER_BOSS_RADIUS,
            BOSS_LENGTH,
            (
                direction * length,
                y_pos,
                tip_joint_z,
            ),
        )
        for y_pos in (
            -(FORK_GAP / 2.0 + EAR_THICK + BOSS_LENGTH / 2.0),
            FORK_GAP / 2.0 + EAR_THICK + BOSS_LENGTH / 2.0,
        )
    ]
    stop_lug = _box((0.010, 0.010, 0.006), (direction * (length - 0.018), 0.0, tip_joint_z - 0.012))

    return _fuse_all(
        root_barrel,
        root_cap,
        *root_webs,
        beam,
        *ribs,
        *side_struts,
        *ears,
        *ear_bosses,
        stop_lug,
    )


def _terminal_stage_shape(length: float, body_z: float, direction: float) -> cq.Workplane:
    beam_len = length - 0.040
    beam_center_x = direction * (0.024 + beam_len / 2.0)
    rib_len = beam_len * 0.68
    rib_center_x = direction * (0.028 + rib_len / 2.0)
    body_bottom = body_z - BODY_THICK / 2.0
    web_bottom = ROOT_BARREL_RADIUS - 0.001
    web_height = max(0.008, body_bottom - web_bottom)

    root_barrel = _y_cylinder(ROOT_BARREL_RADIUS, FORK_GAP, (0.0, 0.0, 0.0))
    root_cap = _box((0.030, BODY_WIDTH, 0.006), (direction * 0.018, 0.0, body_bottom + 0.003))
    root_webs = [
        _box(
            (0.022, 0.003, web_height),
            (direction * 0.014, y_pos, web_bottom + web_height / 2.0),
        )
        for y_pos in (-0.005, 0.005)
    ]
    beam = _box((beam_len, BODY_WIDTH, BODY_THICK), (beam_center_x, 0.0, body_z))
    ribs = [
        _box((rib_len, RIB_WIDTH, RIB_HEIGHT), (rib_center_x, y_pos, body_z + BODY_THICK / 2.0 + RIB_HEIGHT / 2.0))
        for y_pos in (-0.005, 0.005)
    ]

    tip_block = _box((0.026, BODY_WIDTH, 0.010), (direction * (length - 0.010), 0.0, body_bottom + 0.005))
    stalk = _box((0.018, 0.010, 0.022), (direction * (length + 0.008), 0.0, body_z - (TERMINAL_PAD_DROP - 0.005)))
    pad_plate = _box((0.052, 0.014, 0.004), (direction * (length + 0.032), 0.0, body_z - TERMINAL_PAD_DROP))
    pad_ribs = [
        _box((0.020, 0.003, 0.012), (direction * (length + 0.015), y_pos, body_z - (TERMINAL_PAD_DROP - 0.006)))
        for y_pos in (-0.004, 0.004)
    ]
    nose_stop = _box((0.010, 0.010, 0.006), (direction * (length - 0.012), 0.0, 0.004))

    return _fuse_all(root_barrel, root_cap, *root_webs, beam, *ribs, tip_block, stalk, pad_plate, *pad_ribs, nose_stop)


def _support_cap_shape(length: float, body_z: float, direction: float) -> cq.Workplane:
    return _box((0.046, 0.011, 0.002), (direction * (length + 0.032), 0.0, body_z - TERMINAL_PAD_DROP + 0.001))


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    *,
    mesh_name: str,
    material: str,
    visual_name: str,
) -> None:
    part.visual(mesh_from_cadquery(shape, mesh_name), material=material, name=visual_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_hatch_deployable_arm")

    model.material("shoe_steel", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("link_steel", rgba=(0.63, 0.67, 0.71, 1.0))
    model.material("bushing_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("pad_rubber", rgba=(0.09, 0.10, 0.11, 1.0))

    base_shoe = model.part("base_shoe")
    _add_mesh_visual(
        base_shoe,
        _base_shoe_shape(),
        mesh_name="base_shoe",
        material="shoe_steel",
        visual_name="shoe_shell",
    )
    base_shoe.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.042)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
    )

    stage_1 = model.part("stage_1")
    _add_mesh_visual(
        stage_1,
        _link_shape(STAGE_1_LENGTH, STAGE_1_BODY_Z, JOINT_CENTER_Z, 1.0),
        mesh_name="stage_1",
        material="link_steel",
        visual_name="stage_shell",
    )
    stage_1.inertial = Inertial.from_geometry(
        Box((STAGE_1_LENGTH + 0.020, 0.038, 0.018)),
        mass=0.55,
        origin=Origin(xyz=(STAGE_1_LENGTH / 2.0, 0.0, STAGE_1_BODY_Z)),
    )

    stage_2 = model.part("stage_2")
    _add_mesh_visual(
        stage_2,
        _link_shape(STAGE_2_LENGTH, STAGE_2_BODY_Z, JOINT_CENTER_Z, 1.0),
        mesh_name="stage_2",
        material="link_steel",
        visual_name="stage_shell",
    )
    stage_2.inertial = Inertial.from_geometry(
        Box((STAGE_2_LENGTH + 0.020, 0.038, 0.030)),
        mass=0.50,
        origin=Origin(xyz=(STAGE_2_LENGTH / 2.0, 0.0, STAGE_2_BODY_Z)),
    )

    stage_3 = model.part("stage_3")
    _add_mesh_visual(
        stage_3,
        _link_shape(STAGE_3_LENGTH, STAGE_3_BODY_Z, JOINT_CENTER_Z, 1.0),
        mesh_name="stage_3",
        material="link_steel",
        visual_name="stage_shell",
    )
    stage_3.inertial = Inertial.from_geometry(
        Box((STAGE_3_LENGTH + 0.020, 0.038, 0.040)),
        mass=0.45,
        origin=Origin(xyz=(STAGE_3_LENGTH / 2.0, 0.0, STAGE_3_BODY_Z)),
    )

    stage_4 = model.part("stage_4")
    _add_mesh_visual(
        stage_4,
        _terminal_stage_shape(STAGE_4_LENGTH, STAGE_4_BODY_Z, 1.0),
        mesh_name="stage_4",
        material="link_steel",
        visual_name="stage_shell",
    )
    _add_mesh_visual(
        stage_4,
        _support_cap_shape(STAGE_4_LENGTH, STAGE_4_BODY_Z, 1.0),
        mesh_name="stage_4_support_cap",
        material="pad_rubber",
        visual_name="support_cap",
    )
    stage_4.inertial = Inertial.from_geometry(
        Box((STAGE_4_LENGTH + 0.080, 0.038, 0.050)),
        mass=0.42,
        origin=Origin(xyz=(STAGE_4_LENGTH / 2.0, 0.0, STAGE_4_BODY_Z - 0.004)),
    )

    model.articulation(
        "base_to_stage_1",
        ArticulationType.REVOLUTE,
        parent=base_shoe,
        child=stage_1,
        origin=Origin(xyz=(BASE_JOINT_X, 0.0, BASE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.18, effort=45.0, velocity=1.0),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.REVOLUTE,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(STAGE_1_LENGTH, 0.0, JOINT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=3.02, effort=35.0, velocity=1.2),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.REVOLUTE,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(STAGE_2_LENGTH, 0.0, JOINT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=3.02, effort=30.0, velocity=1.3),
    )
    model.articulation(
        "stage_3_to_stage_4",
        ArticulationType.REVOLUTE,
        parent=stage_3,
        child=stage_4,
        origin=Origin(xyz=(STAGE_3_LENGTH, 0.0, JOINT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=3.02, effort=24.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_shoe = object_model.get_part("base_shoe")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")
    stage_4 = object_model.get_part("stage_4")

    base_to_stage_1 = object_model.get_articulation("base_to_stage_1")
    stage_1_to_stage_2 = object_model.get_articulation("stage_1_to_stage_2")
    stage_2_to_stage_3 = object_model.get_articulation("stage_2_to_stage_3")
    stage_3_to_stage_4 = object_model.get_articulation("stage_3_to_stage_4")

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
    ctx.allow_overlap(
        base_shoe,
        stage_1,
        reason="Base clevis and first-stage hinge collars intentionally share a simplified pin envelope; support contact is checked explicitly.",
    )
    ctx.allow_overlap(
        stage_1,
        stage_2,
        reason="Adjacent folding links use simplified hinge knuckle solids around the first elbow pin; hinge support is checked explicitly.",
    )
    ctx.allow_overlap(
        stage_2,
        stage_3,
        reason="Adjacent folding links use simplified hinge knuckle solids around the second elbow pin; hinge support is checked explicitly.",
    )
    ctx.allow_overlap(
        stage_3,
        stage_4,
        reason="Adjacent folding links use simplified hinge knuckle solids around the terminal elbow pin; hinge support is checked explicitly.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    joints = [base_to_stage_1, stage_1_to_stage_2, stage_2_to_stage_3, stage_3_to_stage_4]
    ctx.check(
        "four_revolute_y_joints",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 1.0, 0.0)
            for joint in joints
        ),
        "Expected a four-joint planar revolute chain with all axes along +Y.",
    )

    ctx.expect_contact(base_shoe, stage_1, name="base_hinge_supported")
    ctx.expect_contact(stage_1, stage_2, name="stage_1_hinge_supported")
    ctx.expect_contact(stage_2, stage_3, name="stage_2_hinge_supported")
    ctx.expect_contact(stage_3, stage_4, name="stage_3_hinge_supported")

    with ctx.pose(
        {
            base_to_stage_1: 0.18,
            stage_1_to_stage_2: 2.92,
            stage_2_to_stage_3: 2.86,
            stage_3_to_stage_4: 2.80,
        }
    ):
        ctx.expect_origin_distance(stage_4, base_shoe, axes="x", max_dist=0.085, name="folded_pad_stays_near_base")
        ctx.expect_origin_distance(stage_4, base_shoe, axes="z", max_dist=0.110, name="folded_stack_stays_shallow")
        ctx.expect_overlap(stage_1, stage_2, axes="x", min_overlap=0.030, name="folded_stage_1_stage_2_projection")
        ctx.expect_overlap(stage_2, stage_3, axes="x", min_overlap=0.025, name="folded_stage_2_stage_3_projection")

    with ctx.pose(
        {
            base_to_stage_1: 0.0,
            stage_1_to_stage_2: 0.0,
            stage_2_to_stage_3: 0.0,
            stage_3_to_stage_4: 0.0,
        }
    ):
        ctx.expect_origin_distance(stage_4, base_shoe, axes="xz", min_dist=0.27, name="deployed_reach")
        ctx.expect_origin_gap(stage_4, base_shoe, axis="x", min_gap=0.26, name="deployed_far_end_projects_forward")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
