from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
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
    mesh_from_cadquery,
)


OUTER_W = 0.026
CHEEK_T = 0.0055
INNER_GAP = OUTER_W - (2.0 * CHEEK_T)

BARREL_W = 0.010
SPACER_W = (INNER_GAP - BARREL_W) / 2.0

PIN_R = 0.0022
HOLE_R = 0.0026
BARREL_R = 0.0046
SPACER_R = 0.0049
HEAD_R = 0.0046
HEAD_W = 0.0014

CLEVIS_H = 0.016
PROXIMAL_LEN = 0.145
MIDDLE_LEN = 0.118
DISTAL_LEN = 0.094

FORK_REAR = 0.014
FORK_FRONT = 0.007

ROOT_UPPER = 0.82
MIDDLE_UPPER = 0.92
DISTAL_UPPER = 0.78


def _box_from_start(
    length: float,
    width: float,
    height: float,
    *,
    x0: float,
    y: float = 0.0,
    z: float = 0.0,
):
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, True))
        .translate((x0, y, z))
    )


def _y_cylinder(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
):
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length)
        .translate((0.0, y + (length / 2.0), 0.0))
    )


def _beam_body(length: float, width: float, root_h: float, mid_h: float, tip_h: float):
    profile = [
        (0.0, -root_h / 2.0),
        (0.018, -root_h / 2.0),
        (0.040, -mid_h / 2.0),
        (length - 0.025, -mid_h / 2.0),
        (length, -tip_h / 2.0),
        (length, tip_h / 2.0),
        (length - 0.025, mid_h / 2.0),
        (0.040, mid_h / 2.0),
        (0.018, root_h / 2.0),
        (0.0, root_h / 2.0),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(width)
        .translate((0.0, width / 2.0, 0.0))
    )


def _root_barrel():
    return _y_cylinder(BARREL_R, BARREL_W).cut(_y_cylinder(HOLE_R, BARREL_W + 0.002))


def _root_stack():
    return _root_barrel()


def _pin_head_pair(axis_x: float):
    embed = 0.00035
    washer_center_abs_y = (INNER_GAP / 2.0) + CHEEK_T + (SPACER_W / 2.0)
    center_abs_y = (INNER_GAP / 2.0) + CHEEK_T + SPACER_W + (HEAD_W / 2.0) - embed
    shank = _y_cylinder(PIN_R, OUTER_W, x=axis_x)
    positive_washer = _y_cylinder(SPACER_R, SPACER_W, x=axis_x, y=washer_center_abs_y)
    negative_washer = _y_cylinder(SPACER_R, SPACER_W, x=axis_x, y=-washer_center_abs_y)
    positive_head = _y_cylinder(HEAD_R, HEAD_W, x=axis_x, y=center_abs_y)
    negative_head = _y_cylinder(HEAD_R, HEAD_W, x=axis_x, y=-center_abs_y)
    return shank.union(positive_washer).union(negative_washer).union(positive_head).union(negative_head)


def _cheek_pair(axis_x: float, *, rear: float, front: float, height: float):
    length = rear + front
    y_pos = (INNER_GAP / 2.0) + (CHEEK_T / 2.0)
    positive = _box_from_start(length, CHEEK_T, height, x0=axis_x - rear, y=y_pos)
    negative = _box_from_start(length, CHEEK_T, height, x0=axis_x - rear, y=-y_pos)
    return positive.union(negative)


def _fork(axis_x: float, *, rear: float = FORK_REAR, front: float = FORK_FRONT, height: float = CLEVIS_H):
    cheeks = _cheek_pair(axis_x, rear=rear, front=front, height=height)
    bridge = _box_from_start(rear - (BARREL_R + 0.0022), OUTER_W, 0.0034, x0=axis_x - rear, z=-0.0063)
    fork = cheeks.union(bridge)
    return fork.cut(_y_cylinder(HOLE_R, OUTER_W + 0.002, x=axis_x))


def _slot_tip(length: float, width: float, height: float, x_center: float, z_center: float = 0.0):
    return (
        cq.Workplane("XY")
        .center(x_center, 0.0)
        .slot2D(length, width)
        .extrude(height)
        .translate((0.0, 0.0, z_center - (height / 2.0)))
    )


def _base_body_shape():
    housing = _box_from_start(0.062, 0.034, 0.018, x0=-0.078)
    neck = _box_from_start(0.024, 0.018, 0.010, x0=-0.038, z=-0.0035)
    root_fork = _fork(0.0)
    stop_block = _box_from_start(0.009, 0.009, 0.0036, x0=-0.0085, z=-0.0052)
    stop_stalk = _box_from_start(0.004, 0.014, 0.0075, x0=-0.0105, z=-0.0030)
    return housing.union(neck).union(root_fork).union(stop_block).union(stop_stalk)


def _proximal_body_shape():
    root = _root_stack()
    web = _box_from_start(0.012, 0.0080, 0.0065, x0=0.0025, z=-0.0002)
    heel = _box_from_start(0.010, 0.0075, 0.0030, x0=-0.0040, z=-0.0048)
    beam = _beam_body(PROXIMAL_LEN - FORK_REAR - 0.010, 0.0078, 0.0080, 0.0069, 0.0061).translate((0.010, 0.0, 0.0))
    tip_fork = _fork(PROXIMAL_LEN, height=0.0148)
    return root.union(web).union(heel).union(beam).union(tip_fork)


def _middle_body_shape():
    root = _root_stack()
    web = _box_from_start(0.011, 0.0076, 0.0062, x0=0.0025, z=-0.0002)
    beam = _beam_body(MIDDLE_LEN - FORK_REAR - 0.010, 0.0074, 0.0074, 0.0064, 0.0058).translate((0.010, 0.0, 0.0))
    tip_fork = _fork(MIDDLE_LEN, height=0.0138)
    return root.union(web).union(beam).union(tip_fork)


def _distal_body_shape():
    beam_len = DISTAL_LEN - 0.025
    root = _root_stack()
    web = _box_from_start(0.0105, 0.0070, 0.0058, x0=0.0025, z=-0.0002)
    beam = _beam_body(beam_len - 0.012, 0.0068, 0.0069, 0.0060, 0.0055).translate((0.010, 0.0, 0.0))
    spatula = _slot_tip(0.032, 0.016, 0.0056, x_center=beam_len + 0.015, z_center=0.0003)
    return root.union(web).union(beam).union(spatula)


def _fingertip_pad_shape():
    beam_len = DISTAL_LEN - 0.025
    return _slot_tip(0.034, 0.020, 0.003, x_center=beam_len + 0.016, z_center=0.0040)


def _root_stop_shape():
    stop_block = _box_from_start(0.009, 0.009, 0.0036, x0=-0.0085, z=-0.0052)
    stop_stalk = _box_from_start(0.004, 0.014, 0.0075, x0=-0.0105, z=-0.0030)
    return stop_block.union(stop_stalk)


def _add_box_visual(part_obj, size, center, *, material, name=None):
    part_obj.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_y_cylinder_visual(part_obj, radius, length, center, *, material, name=None):
    part_obj.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="probing_finger_chain")

    model.material("base_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("link_alloy", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("pin_steel", rgba=(0.56, 0.58, 0.62, 1.0))
    model.material("pad_rubber", rgba=(0.14, 0.15, 0.16, 1.0))

    cheek_y = (INNER_GAP / 2.0) + (CHEEK_T / 2.0)
    washer_y = (BARREL_W / 2.0) + (SPACER_W / 2.0)
    cap_y = (OUTER_W / 2.0) + (HEAD_W / 2.0) - 0.0009

    base_mount = model.part("base_mount")
    _add_box_visual(base_mount, (0.062, 0.034, 0.018), (-0.047, 0.0, 0.0), material="base_graphite", name="base_body")
    _add_box_visual(base_mount, (0.024, 0.018, 0.010), (-0.026, 0.0, -0.0035), material="base_graphite")
    _add_box_visual(base_mount, (FORK_REAR + FORK_FRONT, CHEEK_T, CLEVIS_H), (-0.0035, cheek_y, 0.0), material="base_graphite")
    _add_box_visual(base_mount, (FORK_REAR + FORK_FRONT, CHEEK_T, CLEVIS_H), (-0.0035, -cheek_y, 0.0), material="base_graphite")
    _add_box_visual(base_mount, (0.007, OUTER_W, 0.0034), (-0.0105, 0.0, -0.0063), material="base_graphite")
    _add_y_cylinder_visual(base_mount, HEAD_R, HEAD_W, (0.0, cap_y, 0.0), material="pin_steel", name="root_pinstack")
    _add_y_cylinder_visual(base_mount, HEAD_R, HEAD_W, (0.0, -cap_y, 0.0), material="pin_steel")
    _add_box_visual(base_mount, (0.009, 0.009, 0.0036), (-0.0040, 0.0, -0.0065), material="base_graphite", name="root_stop")
    _add_box_visual(base_mount, (0.004, 0.014, 0.0075), (-0.0085, 0.0, -0.00455), material="base_graphite")
    base_mount.inertial = Inertial.from_geometry(
        Box((0.072, 0.034, 0.020)),
        mass=0.55,
        origin=Origin(xyz=(-0.036, 0.0, 0.0)),
    )

    proximal = model.part("proximal_link")
    _add_y_cylinder_visual(proximal, BARREL_R, BARREL_W, (0.0, 0.0, 0.0), material="link_alloy", name="proximal_body")
    _add_y_cylinder_visual(proximal, SPACER_R, SPACER_W, (0.0, washer_y, 0.0), material="link_alloy")
    _add_y_cylinder_visual(proximal, SPACER_R, SPACER_W, (0.0, -washer_y, 0.0), material="link_alloy")
    _add_box_visual(proximal, (0.012, 0.0078, 0.0065), (0.0085, 0.0, -0.0002), material="link_alloy")
    _add_box_visual(proximal, (0.0070, 0.0075, 0.0028), (-0.0055, 0.0, -0.0033), material="link_alloy", name="root_heel")
    _add_box_visual(proximal, (0.007, 0.0078, 0.0058), (0.0110, 0.0, 0.0000), material="link_alloy")
    _add_box_visual(proximal, (0.108, 0.0078, 0.0064), (0.0680, 0.0, 0.0), material="link_alloy")
    _add_box_visual(proximal, (0.015, 0.0070, 0.0058), (0.1245, 0.0, -0.0001), material="link_alloy")
    _add_box_visual(proximal, (0.010, 0.0070, 0.0038), (0.1290, 0.0, -0.0043), material="link_alloy")
    _add_box_visual(proximal, (FORK_REAR + FORK_FRONT, CHEEK_T, 0.0148), (PROXIMAL_LEN - 0.0035, cheek_y, 0.0), material="link_alloy")
    _add_box_visual(proximal, (FORK_REAR + FORK_FRONT, CHEEK_T, 0.0148), (PROXIMAL_LEN - 0.0035, -cheek_y, 0.0), material="link_alloy")
    _add_box_visual(proximal, (0.007, OUTER_W, 0.0034), (PROXIMAL_LEN - 0.0105, 0.0, -0.0032), material="link_alloy")
    _add_y_cylinder_visual(proximal, HEAD_R, HEAD_W, (PROXIMAL_LEN, cap_y, 0.0), material="pin_steel", name="proximal_pinstack")
    _add_y_cylinder_visual(proximal, HEAD_R, HEAD_W, (PROXIMAL_LEN, -cap_y, 0.0), material="pin_steel")
    proximal.inertial = Inertial.from_geometry(
        Box((PROXIMAL_LEN, 0.026, 0.016)),
        mass=0.12,
        origin=Origin(xyz=(PROXIMAL_LEN / 2.0, 0.0, 0.0)),
    )

    middle = model.part("middle_link")
    _add_y_cylinder_visual(middle, BARREL_R, BARREL_W, (0.0, 0.0, 0.0), material="link_alloy", name="middle_body")
    _add_y_cylinder_visual(middle, SPACER_R, SPACER_W, (0.0, washer_y, 0.0), material="link_alloy")
    _add_y_cylinder_visual(middle, SPACER_R, SPACER_W, (0.0, -washer_y, 0.0), material="link_alloy")
    _add_box_visual(middle, (0.011, 0.0076, 0.0062), (0.0080, 0.0, -0.0002), material="link_alloy")
    _add_box_visual(middle, (0.006, 0.0074, 0.0056), (0.0110, 0.0, 0.0000), material="link_alloy")
    _add_box_visual(middle, (0.084, 0.0074, 0.0060), (0.0530, 0.0, 0.0), material="link_alloy")
    _add_box_visual(middle, (0.015, 0.0068, 0.0055), (0.0975, 0.0, -0.0001), material="link_alloy")
    _add_box_visual(middle, (0.010, 0.0068, 0.0038), (0.1020, 0.0, -0.0043), material="link_alloy")
    _add_box_visual(middle, (FORK_REAR + FORK_FRONT, CHEEK_T, 0.0138), (MIDDLE_LEN - 0.0035, cheek_y, 0.0), material="link_alloy")
    _add_box_visual(middle, (FORK_REAR + FORK_FRONT, CHEEK_T, 0.0138), (MIDDLE_LEN - 0.0035, -cheek_y, 0.0), material="link_alloy")
    _add_box_visual(middle, (0.007, OUTER_W, 0.0034), (MIDDLE_LEN - 0.0105, 0.0, -0.0032), material="link_alloy")
    _add_y_cylinder_visual(middle, HEAD_R, HEAD_W, (MIDDLE_LEN, cap_y, 0.0), material="pin_steel", name="middle_pinstack")
    _add_y_cylinder_visual(middle, HEAD_R, HEAD_W, (MIDDLE_LEN, -cap_y, 0.0), material="pin_steel")
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LEN, 0.026, 0.015)),
        mass=0.09,
        origin=Origin(xyz=(MIDDLE_LEN / 2.0, 0.0, 0.0)),
    )

    distal = model.part("distal_link")
    _add_y_cylinder_visual(distal, BARREL_R, BARREL_W, (0.0, 0.0, 0.0), material="link_alloy", name="distal_body")
    _add_y_cylinder_visual(distal, SPACER_R, SPACER_W, (0.0, washer_y, 0.0), material="link_alloy")
    _add_y_cylinder_visual(distal, SPACER_R, SPACER_W, (0.0, -washer_y, 0.0), material="link_alloy")
    _add_box_visual(distal, (0.0105, 0.0070, 0.0058), (0.0078, 0.0, -0.0002), material="link_alloy")
    _add_box_visual(distal, (0.005, 0.0068, 0.0054), (0.0135, 0.0, 0.0000), material="link_alloy")
    _add_box_visual(distal, (0.052, 0.0068, 0.0054), (0.0390, 0.0, 0.0), material="link_alloy")
    _add_box_visual(distal, (0.022, 0.0100, 0.0048), (0.0690, 0.0, 0.0001), material="link_alloy")
    _add_box_visual(distal, (0.024, 0.016, 0.0056), (0.0800, 0.0, 0.0002), material="link_alloy", name="fingertip_pad")
    _add_y_cylinder_visual(distal, 0.008, 0.016, (0.0900, 0.0, 0.0002), material="link_alloy")
    distal.inertial = Inertial.from_geometry(
        Box((DISTAL_LEN, 0.022, 0.012)),
        mass=0.06,
        origin=Origin(xyz=(DISTAL_LEN / 2.0, 0.0, 0.001)),
    )

    model.articulation(
        "root_knuckle",
        ArticulationType.REVOLUTE,
        parent=base_mount,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=ROOT_UPPER, effort=8.0, velocity=2.0),
    )
    model.articulation(
        "middle_knuckle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=MIDDLE_UPPER, effort=6.0, velocity=2.5),
    )
    model.articulation(
        "distal_knuckle",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=DISTAL_UPPER, effort=4.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_mount = object_model.get_part("base_mount")
    proximal = object_model.get_part("proximal_link")
    middle = object_model.get_part("middle_link")
    distal = object_model.get_part("distal_link")

    root_knuckle = object_model.get_articulation("root_knuckle")
    middle_knuckle = object_model.get_articulation("middle_knuckle")
    distal_knuckle = object_model.get_articulation("distal_knuckle")

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

    for joint, upper in (
        (root_knuckle, ROOT_UPPER),
        (middle_knuckle, MIDDLE_UPPER),
        (distal_knuckle, DISTAL_UPPER),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is single-plane revolute",
            joint.joint_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis} type={joint.joint_type}",
        )
        ctx.check(
            f"{joint.name} limits are grounded",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and abs(limits.upper - upper) < 1e-9,
            details=f"limits={limits}",
        )

    ctx.expect_contact(
        base_mount,
        proximal,
        contact_tol=0.0008,
        name="root knuckle has physical support contact",
    )
    ctx.expect_contact(
        proximal,
        middle,
        contact_tol=0.0008,
        name="middle knuckle has physical support contact",
    )
    ctx.expect_contact(
        middle,
        distal,
        contact_tol=0.0008,
        name="distal knuckle has physical support contact",
    )

    with ctx.pose({root_knuckle: 0.0}):
        ctx.expect_contact(
            base_mount,
            proximal,
            elem_a="root_stop",
            elem_b="root_heel",
            contact_tol=0.0006,
            name="root hard stop engages at full extension",
        )

    with ctx.pose({root_knuckle: 0.55}):
        ctx.expect_gap(
            proximal,
            base_mount,
            axis="z",
            positive_elem="root_heel",
            negative_elem="root_stop",
            min_gap=0.0015,
            name="root hard stop clears once the finger bends",
        )

    with ctx.pose({root_knuckle: 0.60, middle_knuckle: 0.72, distal_knuckle: 0.62}):
        ctx.fail_if_parts_overlap_in_current_pose(name="finger stays clear in a curled pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
