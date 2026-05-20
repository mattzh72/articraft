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
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_PLATE_T = 0.018
FRAME_OUTER_W = 0.26
FRAME_OUTER_H = 0.58
FRAME_INNER_W = 0.18
FRAME_INNER_H = 0.44
FRAME_MID_Z = 0.14

SLIDE_RAIL_D = 0.03
SLIDE_RAIL_W = 0.064
SLIDE_RAIL_H = 0.42
SLIDE_RAIL_MID_Z = 0.11

CARRIAGE_D = 0.082
CARRIAGE_W = 0.12
CARRIAGE_H = 0.086
SHOULDER_X = 0.082
SHOULDER_Z = 0.014

PROXIMAL_LEN = 0.16
PROXIMAL_HUB_R = 0.018
PROXIMAL_HUB_L = 0.03

DISTAL_LEN = 0.13
DISTAL_HUB_R = 0.015
DISTAL_HUB_L = 0.026
PAD_T = 0.012
PAD_W = 0.055
PAD_H = 0.065

SLIDE_TRAVEL = 0.22


def _box(
    size: tuple[float, float, float],
    *,
    start: tuple[float, float, float] = (0.0, 0.0, 0.0),
    centered: tuple[bool, bool, bool] = (False, True, True),
) -> cq.Workplane:
    sx, sy, sz = size
    x0, y0, z0 = start
    return cq.Workplane("XY").box(sx, sy, sz, centered=centered).translate((x0, y0, z0))


def make_rear_frame() -> cq.Workplane:
    outer = _box(
        (FRAME_PLATE_T, FRAME_OUTER_W, FRAME_OUTER_H),
        start=(-FRAME_PLATE_T, 0.0, FRAME_MID_Z - FRAME_OUTER_H / 2.0),
        centered=(False, True, False),
    )
    inner = _box(
        (FRAME_PLATE_T + 0.01, FRAME_INNER_W, FRAME_INNER_H),
        start=(-FRAME_PLATE_T - 0.005, 0.0, FRAME_MID_Z - FRAME_INNER_H / 2.0),
        centered=(False, True, False),
    )
    ring = outer.cut(inner)

    rail = _box(
        (SLIDE_RAIL_D, SLIDE_RAIL_W, SLIDE_RAIL_H),
        start=(0.0, 0.0, SLIDE_RAIL_MID_Z - SLIDE_RAIL_H / 2.0),
        centered=(False, True, False),
    )

    bridge = _box(
        (0.045, 0.11, 0.04),
        start=(0.0, 0.0, FRAME_MID_Z + FRAME_OUTER_H / 2.0 - 0.07),
        centered=(False, True, False),
    )

    left_gusset = _box(
        (0.035, 0.03, 0.09),
        start=(0.0, -0.04, FRAME_MID_Z + FRAME_OUTER_H / 2.0 - 0.115),
        centered=(False, True, False),
    )
    right_gusset = _box(
        (0.035, 0.03, 0.09),
        start=(0.0, 0.04, FRAME_MID_Z + FRAME_OUTER_H / 2.0 - 0.115),
        centered=(False, True, False),
    )

    return ring.union(rail).union(bridge).union(left_gusset).union(right_gusset)


def make_carriage() -> cq.Workplane:
    shoe_len = 0.032
    shoe_w = (CARRIAGE_W - SLIDE_RAIL_W) / 2.0
    shoe_y = SLIDE_RAIL_W / 2.0 + shoe_w / 2.0

    left_shoe = _box((shoe_len, shoe_w, CARRIAGE_H), start=(0.0, -shoe_y, 0.0))
    right_shoe = _box((shoe_len, shoe_w, CARRIAGE_H), start=(0.0, shoe_y, 0.0))
    front_block = _box((CARRIAGE_D - shoe_len, CARRIAGE_W, CARRIAGE_H - 0.008), start=(shoe_len, 0.0, 0.0))

    ear_len = 0.024
    ear_th = 0.01
    ear_h = 0.038
    ear_y = PROXIMAL_HUB_L / 2.0 + ear_th / 2.0
    ear_start_x = SHOULDER_X - ear_len / 2.0

    left_ear = _box((ear_len, ear_th, ear_h), start=(ear_start_x, -ear_y, SHOULDER_Z))
    right_ear = _box((ear_len, ear_th, ear_h), start=(ear_start_x, ear_y, SHOULDER_Z))
    shoulder_web = _box((0.016, 0.06, 0.02), start=(CARRIAGE_D - 0.014, 0.0, SHOULDER_Z - 0.012))

    return left_shoe.union(right_shoe).union(front_block).union(left_ear).union(right_ear).union(shoulder_web)


def make_proximal_link() -> cq.Workplane:
    hub = cq.Workplane("XZ").circle(PROXIMAL_HUB_R).extrude(PROXIMAL_HUB_L, both=True)

    beam_start = 0.022
    beam = _box((PROXIMAL_LEN - beam_start, 0.022, 0.024), start=(beam_start, 0.0, 0.0))
    fork_base = _box((0.038, 0.044, 0.02), start=(PROXIMAL_LEN - 0.038, 0.0, 0.0))

    cheek_len = 0.018
    cheek_th = 0.009
    cheek_h = 0.032
    cheek_y = DISTAL_HUB_L / 2.0 + cheek_th / 2.0
    cheek_start_x = PROXIMAL_LEN - cheek_len

    left_cheek = _box((cheek_len, cheek_th, cheek_h), start=(cheek_start_x, -cheek_y, 0.0))
    right_cheek = _box((cheek_len, cheek_th, cheek_h), start=(cheek_start_x, cheek_y, 0.0))

    root_fairing = _box((0.048, 0.032, 0.03), start=(0.01, 0.0, 0.0))

    return hub.union(beam).union(root_fairing).union(fork_base).union(left_cheek).union(right_cheek)


def make_distal_body() -> cq.Workplane:
    hub = cq.Workplane("XZ").circle(DISTAL_HUB_R).extrude(DISTAL_HUB_L, both=True)
    beam_start = 0.018
    beam = _box((DISTAL_LEN - beam_start, 0.019, 0.022), start=(beam_start, 0.0, 0.0))
    nose = _box((0.024, 0.03, 0.028), start=(DISTAL_LEN - 0.024, 0.0, 0.0))
    root_fairing = _box((0.04, 0.03, 0.028), start=(0.008, 0.0, 0.0))
    return hub.union(beam).union(nose).union(root_fairing)


def make_pad() -> cq.Workplane:
    return _box((PAD_T, PAD_W, PAD_H), start=(DISTAL_LEN, 0.0, 0.0))


def _add_box_visual(part, name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_y_cylinder_visual(part, name: str, radius: float, length: float, xyz: tuple[float, float, float], material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_slide_arm")

    frame_finish = model.material("frame_finish", color=(0.22, 0.24, 0.26, 1.0))
    carriage_finish = model.material("carriage_finish", color=(0.72, 0.74, 0.77, 1.0))
    arm_finish = model.material("arm_finish", color=(0.57, 0.6, 0.63, 1.0))
    pad_finish = model.material("pad_finish", color=(0.08, 0.09, 0.1, 1.0))

    rear_frame = model.part("rear_frame")
    _add_box_visual(
        rear_frame,
        "left_upright",
        (FRAME_PLATE_T, 0.04, FRAME_OUTER_H),
        (-FRAME_PLATE_T / 2.0, -0.11, FRAME_MID_Z),
        frame_finish,
    )
    _add_box_visual(
        rear_frame,
        "right_upright",
        (FRAME_PLATE_T, 0.04, FRAME_OUTER_H),
        (-FRAME_PLATE_T / 2.0, 0.11, FRAME_MID_Z),
        frame_finish,
    )
    _add_box_visual(
        rear_frame,
        "bottom_crossbar",
        (FRAME_PLATE_T, FRAME_OUTER_W, 0.05),
        (-FRAME_PLATE_T / 2.0, 0.0, -0.125),
        frame_finish,
    )
    _add_box_visual(
        rear_frame,
        "top_crossbar",
        (FRAME_PLATE_T, FRAME_OUTER_W, 0.05),
        (-FRAME_PLATE_T / 2.0, 0.0, 0.405),
        frame_finish,
    )
    _add_box_visual(
        rear_frame,
        "slide_rail",
        (SLIDE_RAIL_D, SLIDE_RAIL_W, SLIDE_RAIL_H),
        (SLIDE_RAIL_D / 2.0, 0.0, SLIDE_RAIL_MID_Z),
        frame_finish,
    )
    _add_box_visual(
        rear_frame,
        "upper_bridge",
        (0.048, 0.12, 0.04),
        (0.024, 0.0, 0.385),
        frame_finish,
    )
    _add_box_visual(
        rear_frame,
        "left_gusset",
        (0.03, 0.025, 0.09),
        (0.015, -0.047, 0.325),
        frame_finish,
    )
    _add_box_visual(
        rear_frame,
        "right_gusset",
        (0.03, 0.025, 0.09),
        (0.015, 0.047, 0.325),
        frame_finish,
    )

    carriage = model.part("carriage")
    _add_box_visual(
        carriage,
        "left_shoe",
        (0.028, 0.028, CARRIAGE_H),
        (0.014, -0.046, 0.0),
        carriage_finish,
    )
    _add_box_visual(
        carriage,
        "right_shoe",
        (0.028, 0.028, CARRIAGE_H),
        (0.014, 0.046, 0.0),
        carriage_finish,
    )
    _add_box_visual(
        carriage,
        "carriage_block",
        (0.032, CARRIAGE_W, 0.07),
        (0.044, 0.0, 0.0),
        carriage_finish,
    )
    _add_box_visual(
        carriage,
        "left_ear",
        (0.016, 0.008, 0.038),
        (0.072, -0.019, SHOULDER_Z),
        carriage_finish,
    )
    _add_box_visual(
        carriage,
        "right_ear",
        (0.016, 0.008, 0.038),
        (0.072, 0.019, SHOULDER_Z),
        carriage_finish,
    )
    _add_box_visual(
        carriage,
        "left_ear_web",
        (0.008, 0.009, 0.02),
        (0.060, -0.0195, 0.004),
        carriage_finish,
    )
    _add_box_visual(
        carriage,
        "right_ear_web",
        (0.008, 0.009, 0.02),
        (0.060, 0.0195, 0.004),
        carriage_finish,
    )
    _add_box_visual(
        carriage,
        "shoulder_backer",
        (0.01, 0.05, 0.018),
        (0.055, 0.0, 0.0),
        carriage_finish,
    )

    proximal_link = model.part("proximal_link")
    _add_y_cylinder_visual(
        proximal_link,
        "shoulder_hub",
        PROXIMAL_HUB_R,
        PROXIMAL_HUB_L,
        (0.0, 0.0, 0.0),
        arm_finish,
    )
    _add_box_visual(
        proximal_link,
        "main_beam",
        (0.124, 0.022, 0.024),
        (0.08, 0.0, 0.0),
        arm_finish,
    )
    _add_box_visual(
        proximal_link,
        "root_fairing",
        (0.04, 0.03, 0.028),
        (0.036, 0.0, 0.0),
        arm_finish,
    )
    _add_box_visual(
        proximal_link,
        "elbow_backer",
        (0.012, 0.044, 0.018),
        (0.136, 0.0, 0.0),
        arm_finish,
    )
    _add_box_visual(
        proximal_link,
        "left_elbow_cheek",
        (0.018, 0.008, 0.034),
        (0.151, -0.017, 0.0),
        arm_finish,
    )
    _add_box_visual(
        proximal_link,
        "right_elbow_cheek",
        (0.018, 0.008, 0.034),
        (0.151, 0.017, 0.0),
        arm_finish,
    )

    distal_link = model.part("distal_link")
    _add_y_cylinder_visual(
        distal_link,
        "elbow_hub",
        DISTAL_HUB_R,
        DISTAL_HUB_L,
        (0.0, 0.0, 0.0),
        arm_finish,
    )
    _add_box_visual(
        distal_link,
        "distal_beam",
        (0.114, 0.019, 0.022),
        (0.073, 0.0, 0.0),
        arm_finish,
    )
    _add_box_visual(
        distal_link,
        "distal_root",
        (0.034, 0.028, 0.026),
        (0.031, 0.0, 0.0),
        arm_finish,
    )
    _add_box_visual(
        distal_link,
        "pad_mount",
        (0.022, 0.03, 0.026),
        (0.119, 0.0, 0.0),
        arm_finish,
    )
    _add_box_visual(
        distal_link,
        "pad",
        (PAD_T, PAD_W, PAD_H),
        (DISTAL_LEN + PAD_T / 2.0, 0.0, 0.0),
        material=pad_finish,
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rear_frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_proximal",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=proximal_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=-1.05,
            upper=1.2,
        ),
    )
    model.articulation(
        "proximal_to_distal",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=distal_link,
        origin=Origin(xyz=(PROXIMAL_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=-1.45,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    carriage = object_model.get_part("carriage")
    proximal_link = object_model.get_part("proximal_link")
    distal_link = object_model.get_part("distal_link")

    slide = object_model.get_articulation("frame_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_proximal")
    elbow = object_model.get_articulation("proximal_to_distal")

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

    ctx.expect_contact(carriage, rear_frame, name="carriage stays captured on slide rail")
    ctx.expect_contact(proximal_link, carriage, name="shoulder hub is seated in carriage clevis")
    ctx.expect_contact(distal_link, proximal_link, name="elbow hub is seated in proximal clevis")
    ctx.expect_gap(
        distal_link,
        rear_frame,
        axis="x",
        positive_elem="pad",
        min_gap=0.18,
        name="pad projects clearly in front of rear frame",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        carriage_raised = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            rear_frame,
            axes="y",
            margin=0.0,
            name="carriage remains centered within frame width at full lift",
        )

    slide_ok = (
        carriage_rest is not None
        and carriage_raised is not None
        and carriage_raised[2] > carriage_rest[2] + (SLIDE_TRAVEL - 0.01)
    )
    ctx.check(
        "prismatic slide lifts carriage upward",
        slide_ok,
        details=f"rest={carriage_rest}, raised={carriage_raised}",
    )

    pad_rest = ctx.part_element_world_aabb(distal_link, elem="pad")
    with ctx.pose({shoulder: 0.85, elbow: 0.75}):
        pad_lifted = ctx.part_element_world_aabb(distal_link, elem="pad")
        ctx.expect_gap(
            distal_link,
            rear_frame,
            axis="x",
            positive_elem="pad",
            min_gap=0.03,
            name="lifted arm keeps pad clear of rear frame",
        )

    pad_lift_ok = (
        pad_rest is not None
        and pad_lifted is not None
        and ((pad_lifted[0][2] + pad_lifted[1][2]) * 0.5)
        > ((pad_rest[0][2] + pad_rest[1][2]) * 0.5) + 0.12
    )
    ctx.check(
        "positive shoulder and elbow rotation lift the pad",
        pad_lift_ok,
        details=f"rest={pad_rest}, lifted={pad_lifted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
