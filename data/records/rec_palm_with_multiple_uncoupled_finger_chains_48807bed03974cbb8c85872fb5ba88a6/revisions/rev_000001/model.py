from __future__ import annotations

import math

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


PALM_LENGTH = 0.165
PALM_WIDTH = 0.205
PALM_THICKNESS = 0.014
HINGE_Z = 0.024
LINK_WIDTH = 0.025
LINK_THICKNESS = 0.014
PIN_RADIUS = 0.006
FORK_GAP = 0.014
FORK_SIDE_WIDTH = 0.0048
FINGER_ROOT_X = 0.090
FINGER_Y_POSITIONS = (-0.060, -0.020, 0.020, 0.060)
FINGER_LENGTHS = (0.058, 0.046, 0.034)
THUMB_LENGTHS = (0.050, 0.040, 0.032)
THUMB_YAW = -0.72


def _rotated_xy(local_xy: tuple[float, float], yaw: float) -> tuple[float, float]:
    x, y = local_xy
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def _add_clevis(
    palm,
    *,
    root_xyz: tuple[float, float, float],
    yaw: float,
    prefix: str,
    material,
    pin_material,
) -> None:
    """Add the fixed fork that carries one proximal finger barrel."""

    root_x, root_y, root_z = root_xyz
    cheek_height = 0.026
    cheek_center_z = PALM_THICKNESS / 2.0 + cheek_height / 2.0 - 0.001
    cheek_len = 0.023
    cheek_y = FORK_GAP / 2.0 + FORK_SIDE_WIDTH / 2.0

    for sign, suffix in ((-1.0, "cheek_0"), (1.0, "cheek_1")):
        dx, dy = _rotated_xy((0.0, sign * cheek_y), yaw)
        palm.visual(
            Box((cheek_len, FORK_SIDE_WIDTH, cheek_height)),
            origin=Origin(
                xyz=(root_x + dx, root_y + dy, cheek_center_z),
                rpy=(0.0, 0.0, yaw),
            ),
            material=material,
            name=f"{prefix}_{suffix}",
        )

        head_center_y = sign * (FORK_GAP / 2.0 + FORK_SIDE_WIDTH + 0.0030)
        dx, dy = _rotated_xy((0.0, head_center_y), yaw)
        palm.visual(
            Cylinder(radius=PIN_RADIUS * 1.06, length=0.0062),
            origin=Origin(
                xyz=(root_x + dx, root_y + dy, root_z),
                rpy=(math.pi / 2.0, 0.0, yaw),
            ),
            material=pin_material,
            name=f"{prefix}_pin_{0 if sign < 0 else 1}",
        )


def _add_link_geometry(
    part,
    *,
    length: float,
    link_material,
    pin_material,
    pad_material=None,
    distal_fork: bool = True,
) -> None:
    """Rigid phalanx link with a central proximal barrel and optional distal fork."""

    barrel_length = FORK_GAP
    body_width = FORK_GAP
    body_start = PIN_RADIUS - 0.0022
    body_end = length - PIN_RADIUS - 0.0010
    body_len = max(0.010, body_end - body_start)

    part.visual(
        Cylinder(radius=PIN_RADIUS, length=barrel_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="barrel",
    )
    part.visual(
        Box((body_len, body_width, LINK_THICKNESS)),
        origin=Origin(xyz=(body_start + body_len / 2.0, 0.0, 0.0)),
        material=link_material,
        name="beam",
    )

    if distal_fork:
        fork_len = 2.0 * PIN_RADIUS + 0.005
        fork_x = length - 0.001
        side_y = FORK_GAP / 2.0 + FORK_SIDE_WIDTH / 2.0
        for sign, suffix in ((-1.0, "fork_0"), (1.0, "fork_1")):
            part.visual(
                Box((fork_len, FORK_SIDE_WIDTH, LINK_THICKNESS)),
                origin=Origin(xyz=(fork_x, sign * side_y, 0.0)),
                material=link_material,
                name=suffix,
            )
    else:
        cap_len = 0.015
        part.visual(
            Cylinder(radius=LINK_THICKNESS / 2.0, length=body_width),
            origin=Origin(
                xyz=(length - PIN_RADIUS, 0.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=link_material,
            name="rounded_tip",
        )
        if pad_material is not None:
            part.visual(
                Box((cap_len, body_width + 0.001, 0.004)),
                origin=Origin(
                    xyz=(length - cap_len / 2.0, 0.0, -LINK_THICKNESS / 2.0 - 0.0015)
                ),
                material=pad_material,
                name="tactile_pad",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_palm_anthropomorphic_gripper")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    bracket_mat = model.material("matte_black_brackets", rgba=(0.015, 0.017, 0.018, 1.0))
    link_mat = model.material("warm_gray_link_segments", rgba=(0.42, 0.43, 0.41, 1.0))
    pin_mat = model.material("brushed_steel_pins", rgba=(0.72, 0.70, 0.66, 1.0))
    rubber = model.material("black_rubber_pads", rgba=(0.01, 0.01, 0.009, 1.0))

    palm = model.part("palm")
    palm_plate = (
        cq.Workplane("XY")
        .box(PALM_LENGTH, PALM_WIDTH, PALM_THICKNESS)
        .edges("|Z")
        .fillet(0.014)
        .faces(">Z")
        .workplane()
        .pushPoints([(-0.052, -0.067), (-0.052, 0.067), (0.043, -0.067), (0.043, 0.067)])
        .hole(0.012)
    )
    palm.visual(
        mesh_from_cadquery(palm_plate, "rounded_palm_plate", tolerance=0.0008),
        material=anodized,
        name="rounded_plate",
    )

    # A rear wrist flange makes the plate read as a gripper palm rather than a loose slab.
    palm.visual(
        Box((0.032, PALM_WIDTH * 0.72, 0.018)),
        origin=Origin(xyz=(-PALM_LENGTH / 2.0 - 0.012, 0.0, 0.002)),
        material=anodized,
        name="wrist_flange",
    )

    for index, y in enumerate(FINGER_Y_POSITIONS):
        _add_clevis(
            palm,
            root_xyz=(FINGER_ROOT_X, y, HINGE_Z),
            yaw=0.0,
            prefix=f"finger_{index}_root",
            material=bracket_mat,
            pin_material=pin_mat,
        )

    thumb_root = (-0.020, -PALM_WIDTH / 2.0 - 0.006, HINGE_Z)
    palm.visual(
        Box((0.052, 0.030, 0.018)),
        origin=Origin(xyz=(thumb_root[0], -PALM_WIDTH / 2.0 - 0.004, PALM_THICKNESS / 2.0 + 0.006)),
        material=bracket_mat,
        name="thumb_side_mount",
    )
    _add_clevis(
        palm,
        root_xyz=thumb_root,
        yaw=THUMB_YAW,
        prefix="thumb_root",
        material=bracket_mat,
        pin_material=pin_mat,
    )

    root_limits = MotionLimits(effort=18.0, velocity=3.0, lower=0.0, upper=1.25)
    knuckle_limits = MotionLimits(effort=10.0, velocity=3.5, lower=0.0, upper=1.35)

    for finger_index, y in enumerate(FINGER_Y_POSITIONS):
        proximal = model.part(f"finger_{finger_index}_proximal")
        middle = model.part(f"finger_{finger_index}_middle")
        distal = model.part(f"finger_{finger_index}_distal")

        _add_link_geometry(
            proximal,
            length=FINGER_LENGTHS[0],
            link_material=link_mat,
            pin_material=pin_mat,
            distal_fork=True,
        )
        _add_link_geometry(
            middle,
            length=FINGER_LENGTHS[1],
            link_material=link_mat,
            pin_material=pin_mat,
            distal_fork=True,
        )
        _add_link_geometry(
            distal,
            length=FINGER_LENGTHS[2],
            link_material=link_mat,
            pin_material=pin_mat,
            pad_material=rubber,
            distal_fork=False,
        )

        model.articulation(
            f"finger_{finger_index}_root",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(FINGER_ROOT_X, y, HINGE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=root_limits,
        )
        model.articulation(
            f"finger_{finger_index}_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(FINGER_LENGTHS[0], 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=knuckle_limits,
        )
        model.articulation(
            f"finger_{finger_index}_distal",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(FINGER_LENGTHS[1], 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=knuckle_limits,
        )

    thumb_proximal = model.part("thumb_proximal")
    thumb_middle = model.part("thumb_middle")
    thumb_distal = model.part("thumb_distal")
    _add_link_geometry(
        thumb_proximal,
        length=THUMB_LENGTHS[0],
        link_material=link_mat,
        pin_material=pin_mat,
        distal_fork=True,
    )
    _add_link_geometry(
        thumb_middle,
        length=THUMB_LENGTHS[1],
        link_material=link_mat,
        pin_material=pin_mat,
        distal_fork=True,
    )
    _add_link_geometry(
        thumb_distal,
        length=THUMB_LENGTHS[2],
        link_material=link_mat,
        pin_material=pin_mat,
        pad_material=rubber,
        distal_fork=False,
    )

    model.articulation(
        "thumb_root",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_proximal,
        origin=Origin(xyz=thumb_root, rpy=(0.0, 0.0, THUMB_YAW)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=3.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "thumb_middle",
        ArticulationType.REVOLUTE,
        parent=thumb_proximal,
        child=thumb_middle,
        origin=Origin(xyz=(THUMB_LENGTHS[0], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=knuckle_limits,
    )
    model.articulation(
        "thumb_distal",
        ArticulationType.REVOLUTE,
        parent=thumb_middle,
        child=thumb_distal,
        origin=Origin(xyz=(THUMB_LENGTHS[1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=knuckle_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = object_model.articulations
    ctx.check(
        "five independent serial chains",
        len(joints) == 15
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(j.mimic is None for j in joints),
        details=f"joint_count={len(joints)}, mimics={[j.name for j in joints if j.mimic is not None]}",
    )
    for joint in joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_flexion_limits",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 1.0 <= limits.upper <= 1.4,
            details=f"limits={limits}",
        )

    palm = object_model.get_part("palm")
    for index in range(4):
        proximal = object_model.get_part(f"finger_{index}_proximal")
        ctx.expect_contact(
            proximal,
            palm,
            contact_tol=0.001,
            name=f"finger_{index}_root_aligned_with_clevis",
        )

    thumb = object_model.get_part("thumb_proximal")
    ctx.allow_overlap(
        palm,
        thumb,
        reason="The angled side thumb barrel is intentionally captured between the fixed clevis cheeks.",
    )
    ctx.expect_contact(
        thumb,
        palm,
        contact_tol=0.001,
        name="thumb_side_clevis_straddles_barrel",
    )

    thumb_middle = object_model.get_part("thumb_middle")
    thumb_distal = object_model.get_part("thumb_distal")
    ctx.allow_overlap(
        thumb_middle,
        thumb,
        reason="The oblique thumb interphalangeal barrel is intentionally captured between both fork cheeks.",
    )
    ctx.expect_contact(
        thumb_middle,
        thumb,
        contact_tol=0.001,
        name="thumb_middle_barrel_seated_in_fork",
    )
    ctx.allow_overlap(
        thumb_distal,
        thumb_middle,
        reason="The distal thumb barrel is intentionally captured between both middle-link fork cheeks.",
    )
    ctx.expect_contact(
        thumb_distal,
        thumb_middle,
        contact_tol=0.001,
        name="thumb_distal_barrel_seated_in_fork",
    )

    distal = object_model.get_part("finger_1_distal")
    neighbor_distal = object_model.get_part("finger_2_distal")
    root_joint = object_model.get_articulation("finger_1_root")
    rest_tip = ctx.part_world_position(distal)
    rest_neighbor = ctx.part_world_position(neighbor_distal)
    with ctx.pose({root_joint: 0.9}):
        curled_tip = ctx.part_world_position(distal)
        curled_neighbor = ctx.part_world_position(neighbor_distal)
    ctx.check(
        "finger_1_moves_without_coupling_neighbor",
        rest_tip is not None
        and curled_tip is not None
        and rest_neighbor is not None
        and curled_neighbor is not None
        and curled_tip[2] < rest_tip[2] - 0.025
        and abs(curled_neighbor[0] - rest_neighbor[0]) < 1e-6
        and abs(curled_neighbor[2] - rest_neighbor[2]) < 1e-6,
        details=f"rest_tip={rest_tip}, curled_tip={curled_tip}, neighbor={rest_neighbor}->{curled_neighbor}",
    )

    return ctx.report()


object_model = build_object_model()
