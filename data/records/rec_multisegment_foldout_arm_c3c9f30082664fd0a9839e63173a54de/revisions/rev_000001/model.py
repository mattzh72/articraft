from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PLATE_LENGTH = 0.150
PLATE_WIDTH = 0.100
PLATE_THICKNESS = 0.012

BASE_JOINT_X = -0.030
BASE_BRACKET_LENGTH = 0.026
BASE_BRACKET_WIDTH = 0.028
BASE_BRACKET_HEIGHT = 0.028

EAR_LENGTH = 0.022
EAR_THICKNESS = 0.007
JOINT_GAP = 0.010
OUTER_WIDTH = 2.0 * EAR_THICKNESS + JOINT_GAP
BODY_WIDTH = 0.014
TANG_THICKNESS = JOINT_GAP
TANG_HEIGHT = 0.016
EAR_HEIGHT = 0.026

BASE_JOINT_Z = 0.052
EAR_BOTTOM_Z = BASE_JOINT_Z - 0.5 * EAR_HEIGHT

LINK_PITCH = 0.155
LINK_HEIGHT = 0.022
TANG_BACK = 0.012
TANG_FRONT = 0.014
TANG_LENGTH = TANG_BACK + TANG_FRONT
TANG_CENTER_X = 0.5 * (TANG_FRONT - TANG_BACK)

BEAM_START = 0.013
BEAM_END = 0.134
BEAM_LENGTH = BEAM_END - BEAM_START
BEAM_CENTER_X = 0.5 * (BEAM_START + BEAM_END)

FORK_CENTER_X = LINK_PITCH - 0.5 * EAR_LENGTH
EAR_OFFSET_Y = 0.5 * JOINT_GAP + 0.5 * EAR_THICKNESS

TERMINAL_NECK_LENGTH = 0.030
TERMINAL_NECK_CENTER_X = 0.026
PAD_BACK_LENGTH = 0.036
PAD_BACK_CENTER_X = 0.045
TERMINAL_BODY_WIDTH = 0.052
PAD_PRONG_LENGTH = 0.028
PAD_PRONG_CENTER_X = 0.073
PAD_PRONG_WIDTH = 0.015
PAD_PRONG_OFFSET_Y = 0.018
TERMINAL_THICKNESS = 0.012


def add_box_visual(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def add_base_visuals(base, material) -> None:
    add_box_visual(
        base,
        (PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS),
        (0.0, 0.0, 0.5 * PLATE_THICKNESS),
        material,
        "plate",
    )
    add_box_visual(
        base,
        (BASE_BRACKET_LENGTH, BASE_BRACKET_WIDTH, BASE_BRACKET_HEIGHT),
        (
            BASE_JOINT_X - 0.5 * BASE_BRACKET_LENGTH,
            0.0,
            PLATE_THICKNESS + 0.5 * BASE_BRACKET_HEIGHT,
        ),
        material,
        "pedestal",
    )
    add_box_visual(
        base,
        (EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT),
        (BASE_JOINT_X - 0.5 * EAR_LENGTH, EAR_OFFSET_Y, BASE_JOINT_Z),
        material,
        "left_ear",
    )
    add_box_visual(
        base,
        (EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT),
        (BASE_JOINT_X - 0.5 * EAR_LENGTH, -EAR_OFFSET_Y, BASE_JOINT_Z),
        material,
        "right_ear",
    )


def add_link_visuals(link, material, *, prefix: str) -> None:
    add_box_visual(
        link,
        (TANG_LENGTH, TANG_THICKNESS, TANG_HEIGHT),
        (TANG_CENTER_X, 0.0, 0.0),
        material,
        f"{prefix}_tang",
    )
    add_box_visual(
        link,
        (BEAM_LENGTH, BODY_WIDTH, LINK_HEIGHT),
        (BEAM_CENTER_X, 0.0, 0.0),
        material,
        f"{prefix}_beam",
    )
    add_box_visual(
        link,
        (EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT),
        (FORK_CENTER_X, EAR_OFFSET_Y, 0.0),
        material,
        f"{prefix}_left_fork",
    )
    add_box_visual(
        link,
        (EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT),
        (FORK_CENTER_X, -EAR_OFFSET_Y, 0.0),
        material,
        f"{prefix}_right_fork",
    )


def add_terminal_pad_visuals(terminal_pad, material) -> None:
    add_box_visual(
        terminal_pad,
        (TANG_LENGTH, TANG_THICKNESS, TANG_HEIGHT),
        (TANG_CENTER_X, 0.0, 0.0),
        material,
        "terminal_tang",
    )
    add_box_visual(
        terminal_pad,
        (TERMINAL_NECK_LENGTH, BODY_WIDTH, TANG_HEIGHT),
        (TERMINAL_NECK_CENTER_X, 0.0, 0.0),
        material,
        "terminal_neck",
    )
    add_box_visual(
        terminal_pad,
        (PAD_BACK_LENGTH, TERMINAL_BODY_WIDTH, TERMINAL_THICKNESS),
        (PAD_BACK_CENTER_X, 0.0, 0.0),
        material,
        "terminal_back",
    )
    add_box_visual(
        terminal_pad,
        (PAD_PRONG_LENGTH, PAD_PRONG_WIDTH, TERMINAL_THICKNESS),
        (PAD_PRONG_CENTER_X, PAD_PRONG_OFFSET_Y, 0.0),
        material,
        "terminal_left_prong",
    )
    add_box_visual(
        terminal_pad,
        (PAD_PRONG_LENGTH, PAD_PRONG_WIDTH, TERMINAL_THICKNESS),
        (PAD_PRONG_CENTER_X, -PAD_PRONG_OFFSET_Y, 0.0),
        material,
        "terminal_right_prong",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_boom")

    base_mat = model.material("base_plate_finish", rgba=(0.20, 0.21, 0.24, 1.0))
    link_mat = model.material("boom_link_finish", rgba=(0.78, 0.63, 0.19, 1.0))
    terminal_mat = model.material("terminal_pad_finish", rgba=(0.30, 0.31, 0.35, 1.0))

    base = model.part("base_plate")
    add_base_visuals(base, base_mat)
    base.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, BASE_JOINT_Z + 0.5 * EAR_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (BASE_JOINT_Z + 0.5 * EAR_HEIGHT))),
    )

    links = []
    for index in range(4):
        link = model.part(f"link_{index + 1}")
        add_link_visuals(link, link_mat, prefix=f"link_{index + 1}")
        link.inertial = Inertial.from_geometry(
            Box((LINK_PITCH + TANG_BACK, OUTER_WIDTH, EAR_HEIGHT)),
            mass=0.22,
            origin=Origin(xyz=(0.5 * (LINK_PITCH - TANG_BACK), 0.0, 0.0)),
        )
        links.append(link)

    terminal_pad = model.part("terminal_pad")
    add_terminal_pad_visuals(terminal_pad, terminal_mat)
    terminal_pad.inertial = Inertial.from_geometry(
        Box((0.100, TERMINAL_BODY_WIDTH, TANG_HEIGHT)),
        mass=0.12,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(xyz=(BASE_JOINT_X, 0.0, BASE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.35,
            upper=1.35,
        ),
    )
    for parent_index in range(3):
        model.articulation(
            f"link_{parent_index + 1}_to_link_{parent_index + 2}",
            ArticulationType.REVOLUTE,
            parent=links[parent_index],
            child=links[parent_index + 1],
            origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=2.4,
                lower=-2.15,
                upper=2.15,
            ),
        )
    model.articulation(
        "link_4_to_terminal_pad",
        ArticulationType.FIXED,
        parent=links[3],
        child=terminal_pad,
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    part_names = (
        "base_plate",
        "link_1",
        "link_2",
        "link_3",
        "link_4",
        "terminal_pad",
    )
    joint_names = (
        "base_to_link_1",
        "link_1_to_link_2",
        "link_2_to_link_3",
        "link_3_to_link_4",
        "link_4_to_terminal_pad",
    )

    base = object_model.get_part("base_plate")
    links = [object_model.get_part(f"link_{index}") for index in range(1, 5)]
    terminal_pad = object_model.get_part("terminal_pad")

    joints = [object_model.get_articulation(name) for name in joint_names]
    revolute_joints = joints[:-1]
    terminal_mount = joints[-1]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name in part_names:
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)
    for joint_name in joint_names:
        ctx.check(f"{joint_name}_present", object_model.get_articulation(joint_name) is not None)

    for joint in revolute_joints:
        ctx.check(
            f"{joint.name}_axis_is_planar_y",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{joint.name} axis was {joint.axis}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_has_real_range",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and (limits.upper - limits.lower) > 1.5,
            f"{joint.name} limits were {limits}",
        )

    ctx.check(
        "terminal_pad_mount_is_fixed",
        terminal_mount.articulation_type == ArticulationType.FIXED,
        f"terminal articulation was {terminal_mount.articulation_type}",
    )

    ctx.expect_contact("base_plate", "link_1", name="base_plate_contacts_link_1")
    ctx.expect_contact("link_1", "link_2", name="link_1_contacts_link_2")
    ctx.expect_contact("link_2", "link_3", name="link_2_contacts_link_3")
    ctx.expect_contact("link_3", "link_4", name="link_3_contacts_link_4")
    ctx.expect_contact("link_4", "terminal_pad", name="link_4_contacts_terminal_pad")

    ctx.expect_origin_gap(
        "link_1",
        "base_plate",
        axis="z",
        min_gap=BASE_JOINT_Z - 0.002,
        max_gap=BASE_JOINT_Z + 0.002,
        name="first_link_joint_is_raised_above_plate",
    )
    ctx.expect_origin_gap(
        "link_2",
        "link_1",
        axis="x",
        min_gap=LINK_PITCH - 0.002,
        max_gap=LINK_PITCH + 0.002,
        name="link_2_pitch_spacing",
    )
    ctx.expect_origin_gap(
        "link_3",
        "link_2",
        axis="x",
        min_gap=LINK_PITCH - 0.002,
        max_gap=LINK_PITCH + 0.002,
        name="link_3_pitch_spacing",
    )
    ctx.expect_origin_gap(
        "link_4",
        "link_3",
        axis="x",
        min_gap=LINK_PITCH - 0.002,
        max_gap=LINK_PITCH + 0.002,
        name="link_4_pitch_spacing",
    )
    ctx.expect_origin_gap(
        "terminal_pad",
        "link_4",
        axis="x",
        min_gap=LINK_PITCH - 0.002,
        max_gap=LINK_PITCH + 0.002,
        name="terminal_pad_pitch_spacing",
    )
    ctx.expect_origin_gap(
        "terminal_pad",
        "base_plate",
        axis="x",
        min_gap=0.57,
        max_gap=0.61,
        name="boom_has_long_open_reach",
    )

    stowed_pose = {
        revolute_joints[0]: -1.20,
        revolute_joints[1]: 2.05,
        revolute_joints[2]: -2.05,
        revolute_joints[3]: 1.35,
    }
    with ctx.pose(stowed_pose):
        base_pos = ctx.part_world_position(base)
        link_2_pos = ctx.part_world_position(links[1])
        link_3_pos = ctx.part_world_position(links[2])
        link_4_pos = ctx.part_world_position(links[3])
        terminal_pos = ctx.part_world_position(terminal_pad)

        stowed_dx = terminal_pos[0] - base_pos[0]
        stowed_dz = terminal_pos[2] - base_pos[2]

        ctx.check(
            "stowed_pose_is_compact",
            0.30 <= stowed_dx <= 0.36 and 0.15 <= stowed_dz <= 0.24,
            f"stowed terminal offset was dx={stowed_dx:.4f}, dz={stowed_dz:.4f}",
        )
        ctx.check(
            "stowed_pose_zigzags",
            link_2_pos[2] > link_3_pos[2] + 0.06 and link_4_pos[2] > link_3_pos[2] + 0.10,
            (
                "expected alternating joint heights in stowed pose, got "
                f"z2={link_2_pos[2]:.4f}, z3={link_3_pos[2]:.4f}, z4={link_4_pos[2]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
