from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.16
BASE_WIDTH = 0.08
BASE_THICK = 0.012
SPINE_HEIGHT = 0.38
SPINE_WEB_DEPTH = 0.028
SPINE_WEB_WIDTH = 0.014
SPINE_WEB_CENTER_X = -0.028
SHOULDER_Z = 0.38

HUB_THICK = 0.010
CHEEK_THICK = 0.004
JOINT_SIDE_CLEAR = 0.0015
CHEEK_OFFSET = HUB_THICK / 2.0 + JOINT_SIDE_CLEAR + CHEEK_THICK / 2.0
BEAM_THICK = 0.010
BEAM_HEIGHT = 0.020
CHEEK_HEIGHT = 0.028
FORK_BRIDGE_LENGTH = 0.010
FORK_BRIDGE_HEIGHT = 0.010
FORK_SPAN = 2.0 * CHEEK_OFFSET + CHEEK_THICK
CHEEK_LENGTH = 0.022
JOINT_SEAT_OVERLAP = 0.0
PROXIMAL_POD_LENGTH = 0.018
PROXIMAL_POD_HEIGHT = 0.024
SHOULDER_ROOT_LENGTH = 0.012
SHOULDER_ROOT_HEIGHT = 0.020

LINK_1_LENGTH = 0.26
LINK_2_LENGTH = 0.24
LINK_3_LENGTH = 0.18


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def make_spine() -> cq.Workplane:
    base = _box(
        (BASE_LENGTH, BASE_WIDTH, BASE_THICK),
        (-0.030, 0.0, BASE_THICK / 2.0),
    )

    web = _box(
        (SPINE_WEB_DEPTH, SPINE_WEB_WIDTH, SHOULDER_Z - 0.008),
        (SPINE_WEB_CENTER_X, 0.0, (SHOULDER_Z - 0.008) / 2.0),
    )

    shoulder_root = _box(
        (SHOULDER_ROOT_LENGTH, HUB_THICK, SHOULDER_ROOT_HEIGHT),
        (-SHOULDER_ROOT_LENGTH / 2.0, 0.0, SHOULDER_Z),
    )

    fork_bridge = _box(
        (FORK_BRIDGE_LENGTH, FORK_SPAN, FORK_BRIDGE_HEIGHT),
        (-0.007, 0.0, SHOULDER_Z),
    )

    shoulder_cheeks = base.union(web).union(shoulder_root).union(fork_bridge)
    for sign in (-1.0, 1.0):
        cheek = _box(
            (CHEEK_LENGTH, CHEEK_THICK, CHEEK_HEIGHT),
            (-CHEEK_LENGTH / 2.0, sign * CHEEK_OFFSET, SHOULDER_Z),
        )
        shoulder_cheeks = shoulder_cheeks.union(cheek)

    return shoulder_cheeks


def make_link(length: float) -> cq.Workplane:
    proximal_pod = _box(
        (PROXIMAL_POD_LENGTH, HUB_THICK, PROXIMAL_POD_HEIGHT),
        (PROXIMAL_POD_LENGTH / 2.0 - JOINT_SEAT_OVERLAP, 0.0, 0.0),
    )

    bridge_center_x = length - FORK_BRIDGE_LENGTH / 2.0
    bridge_min_x = bridge_center_x - FORK_BRIDGE_LENGTH / 2.0
    beam_start = PROXIMAL_POD_LENGTH - 0.002
    beam_end = bridge_min_x
    beam_length = beam_end - beam_start
    beam = _box(
        (beam_length, BEAM_THICK, BEAM_HEIGHT),
        (beam_start + beam_length / 2.0, 0.0, 0.0),
    )

    fork_bridge = _box(
        (FORK_BRIDGE_LENGTH, FORK_SPAN, FORK_BRIDGE_HEIGHT),
        (bridge_center_x, 0.0, 0.0),
    )

    body = proximal_pod.union(beam).union(fork_bridge)

    for sign in (-1.0, 1.0):
        cheek = _box(
            (CHEEK_LENGTH, CHEEK_THICK, CHEEK_HEIGHT),
            (length - CHEEK_LENGTH / 2.0, sign * CHEEK_OFFSET, 0.0),
        )
        body = body.union(cheek)

    return body


def make_end_tab() -> cq.Workplane:
    proximal_pod = _box(
        (PROXIMAL_POD_LENGTH, HUB_THICK, PROXIMAL_POD_HEIGHT),
        (PROXIMAL_POD_LENGTH / 2.0 - JOINT_SEAT_OVERLAP, 0.0, 0.0),
    )

    neck = _box((0.026, HUB_THICK, 0.016), (0.025, 0.0, 0.0))
    tab = (
        cq.Workplane("XZ")
        .center(0.052, 0.0)
        .slot2D(0.036, 0.014)
        .extrude(0.004, both=True)
    )

    return proximal_pod.union(neck).union(tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_service_chain")

    dark_frame = model.material("dark_frame", rgba=(0.22, 0.24, 0.26, 1.0))
    light_link = model.material("light_link", rgba=(0.72, 0.74, 0.76, 1.0))
    tool_dark = model.material("tool_dark", rgba=(0.28, 0.30, 0.33, 1.0))

    spine = model.part("spine")
    spine.visual(
        mesh_from_cadquery(make_spine(), "spine"),
        origin=Origin(),
        material=dark_frame,
        name="spine_body",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(make_link(LINK_1_LENGTH), "link_1"),
        origin=Origin(),
        material=light_link,
        name="link_body",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(make_link(LINK_2_LENGTH), "link_2"),
        origin=Origin(),
        material=light_link,
        name="link_body",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(make_link(LINK_3_LENGTH), "link_3"),
        origin=Origin(),
        material=light_link,
        name="link_body",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(make_end_tab(), "end_tab"),
        origin=Origin(),
        material=tool_dark,
        name="end_tab_body",
    )

    joint_limits = MotionLimits(effort=18.0, velocity=2.0, lower=-0.95 * pi, upper=0.95 * pi)

    model.articulation(
        "spine_to_link_1",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=2.3,
            lower=-0.95 * pi,
            upper=0.95 * pi,
        ),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.6,
            lower=-0.95 * pi,
            upper=0.95 * pi,
        ),
    )
    model.articulation(
        "link_3_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=end_tab,
        origin=Origin(xyz=(LINK_3_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-0.90 * pi,
            upper=0.90 * pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    spine = object_model.get_part("spine")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    end_tab = object_model.get_part("end_tab")

    ctx.allow_overlap(
        spine,
        link_1,
        reason="The grounded shoulder uses a compact nested knuckle envelope at the first revolute joint; the mesh-backed visuals intentionally share the nominal hinge barrel volume.",
    )

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

    shoulder = object_model.get_articulation("spine_to_link_1")
    elbow_1 = object_model.get_articulation("link_1_to_link_2")
    elbow_2 = object_model.get_articulation("link_2_to_link_3")
    wrist = object_model.get_articulation("link_3_to_end_tab")
    joints = [shoulder, elbow_1, elbow_2, wrist]

    ctx.check(
        "four serial revolute joints",
        len(joints) == 4
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in joints),
        details="Expected a four-joint serial revolute chain.",
    )
    ctx.check(
        "parallel joint axes",
        all(tuple(joint.axis) == (0.0, -1.0, 0.0) for joint in joints),
        details="All joints should share the same hinge axis direction.",
    )

    ctx.expect_contact(spine, link_1, contact_tol=0.0015, name="spine touches link 1 at shoulder")
    ctx.expect_contact(link_1, link_2, contact_tol=0.0015, name="link 1 touches link 2 at elbow")
    ctx.expect_contact(link_2, link_3, contact_tol=0.0015, name="link 2 touches link 3 at elbow")
    ctx.expect_contact(link_3, end_tab, contact_tol=0.0015, name="link 3 touches end tab at wrist")

    spine_aabb = ctx.part_world_aabb(spine)
    ctx.check(
        "grounded spine sits on floor",
        spine_aabb is not None and abs(spine_aabb[0][2]) <= 1e-4,
        details=f"Expected spine minimum z near 0.0, got {spine_aabb[0][2] if spine_aabb else 'missing'}.",
    )

    rest_end_pos = ctx.part_world_position(end_tab)
    ctx.check(
        "rest pose has slim forward reach",
        rest_end_pos is not None and rest_end_pos[0] > 0.60,
        details=f"Expected end tab origin beyond 0.60 m in +X, got {rest_end_pos}.",
    )
    with ctx.pose(
        {
            shoulder: 0.55,
            elbow_1: 0.70,
            elbow_2: 0.55,
            wrist: 0.35,
        }
    ):
        bent_end_pos = ctx.part_world_position(end_tab)
    ctx.check(
        "positive joint pose lifts end tab",
        rest_end_pos is not None
        and bent_end_pos is not None
        and bent_end_pos[2] > rest_end_pos[2] + 0.10
        and bent_end_pos[0] < rest_end_pos[0] - 0.05,
        details=f"Expected bent pose to raise and curl the chain, got rest={rest_end_pos}, bent={bent_end_pos}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
