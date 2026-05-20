from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


PALM_WIDTH = 0.092
PALM_LENGTH = 0.066
PALM_THICKNESS = 0.024

FINGER_BARREL_RADIUS = 0.0042
FINGER_BARREL_LENGTH = 0.010
FINGER_CHEEK_THICKNESS = 0.003
FINGER_CLEVIS_DEPTH = 0.010
FINGER_BODY_START = 0.003

THUMB_BARREL_RADIUS = 0.0044
THUMB_BARREL_LENGTH = 0.008
THUMB_PLATE_THICKNESS = 0.003
THUMB_CLEVIS_DEPTH = 0.010
THUMB_BODY_START = 0.008

FINGER_ROOT_Y = PALM_LENGTH * 0.5 + 0.007
THUMB_ROOT_X = -PALM_WIDTH * 0.5 - 0.006
THUMB_ROOT_Y = -0.018
THUMB_ROOT_Z = -0.020

FINGER_SPECS = (
    ("index", -0.028, (0.046, 0.032, 0.024), (0.018, 0.016, 0.014)),
    ("middle", -0.008, (0.051, 0.036, 0.026), (0.019, 0.017, 0.015)),
    ("ring", 0.012, (0.048, 0.034, 0.024), (0.018, 0.016, 0.014)),
    ("little", 0.030, (0.040, 0.029, 0.021), (0.016, 0.014, 0.012)),
)

THUMB_LENGTHS = (0.036, 0.028)
THUMB_WIDTHS = (0.016, 0.014)
THUMB_THICKNESSES = (0.014, 0.012)


def _finger_segment(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    width: float,
    thickness: float,
    structure_material,
    joint_material,
    pad_material,
    terminal: bool,
):
    part = model.part(name)

    body_length = length - 0.001 if terminal else length - 0.005
    part.visual(
        Cylinder(radius=FINGER_BARREL_RADIUS, length=FINGER_BARREL_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_material,
        name="proximal_barrel",
    )
    part.visual(
        Box((width * 0.82, body_length, thickness * 0.82)),
        origin=Origin(xyz=(0.0, FINGER_BODY_START + body_length * 0.5, 0.0)),
        material=structure_material,
        name="beam",
    )
    part.visual(
        Box((width * 0.60, body_length * 0.82, thickness * 0.42)),
        origin=Origin(
            xyz=(0.0, FINGER_BODY_START + body_length * 0.50, -thickness * 0.10)
        ),
        material=joint_material,
        name="stiffener",
    )

    if terminal:
        tip_length = min(0.010, length * 0.42)
        part.visual(
            Box((width * 0.72, tip_length, thickness * 0.34)),
            origin=Origin(
                xyz=(0.0, length - tip_length * 0.5, -thickness * 0.18)
            ),
            material=pad_material,
            name="tip_pad",
        )
    else:
        cheek_x = FINGER_BARREL_LENGTH * 0.5 + FINGER_CHEEK_THICKNESS * 0.5
        cheek_y = length - FINGER_CLEVIS_DEPTH * 0.5
        for side, sign in (("left", -1.0), ("right", 1.0)):
            part.visual(
                Box((FINGER_CHEEK_THICKNESS, FINGER_CLEVIS_DEPTH, thickness)),
                origin=Origin(xyz=(sign * cheek_x, cheek_y, 0.0)),
                material=joint_material,
                name=f"distal_clevis_{side}",
            )

    part.inertial = Inertial.from_geometry(
        Box((width, max(length, 0.012), thickness)),
        mass=max(0.035, length * width * thickness * 2800.0),
        origin=Origin(xyz=(0.0, length * 0.52, 0.0)),
    )
    return part


def _thumb_segment(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    width: float,
    thickness: float,
    structure_material,
    joint_material,
    pad_material,
    terminal: bool,
):
    part = model.part(name)

    part.visual(
        Cylinder(radius=THUMB_BARREL_RADIUS, length=THUMB_BARREL_LENGTH),
        material=joint_material,
        name="proximal_barrel",
    )

    if terminal:
        beam_length = length - 0.002
        part.visual(
            Box((width * 0.92, beam_length, thickness * 0.78)),
            origin=Origin(xyz=(0.0, beam_length * 0.5, 0.0)),
            material=structure_material,
            name="beam",
        )
        part.visual(
            Box((width * 0.58, beam_length * 0.72, thickness * 0.34)),
            origin=Origin(xyz=(0.0, beam_length * 0.46, -thickness * 0.12)),
            material=joint_material,
            name="stiffener",
        )
        tip_length = min(0.010, length * 0.45)
        part.visual(
            Box((width * 0.74, tip_length, thickness * 0.30)),
            origin=Origin(
                xyz=(0.0, length - tip_length * 0.5, -thickness * 0.18)
            ),
            material=pad_material,
            name="tip_pad",
        )
    else:
        beam_length = length - THUMB_CLEVIS_DEPTH + 0.001
        part.visual(
            Box((width * 0.84, beam_length, thickness * 0.60)),
            origin=Origin(xyz=(0.0, beam_length * 0.5, 0.0)),
            material=structure_material,
            name="beam",
        )
        part.visual(
            Box((width * 0.54, beam_length * 0.70, thickness * 0.26)),
            origin=Origin(xyz=(0.0, beam_length * 0.44, -thickness * 0.10)),
            material=joint_material,
            name="stiffener",
        )
        plate_z = THUMB_BARREL_LENGTH * 0.5 + THUMB_PLATE_THICKNESS * 0.5
        plate_y = length - THUMB_CLEVIS_DEPTH * 0.5 - 0.0005
        for plate_name, sign in (("upper", 1.0), ("lower", -1.0)):
            part.visual(
                Box((width, THUMB_CLEVIS_DEPTH, THUMB_PLATE_THICKNESS)),
                origin=Origin(xyz=(0.0, plate_y, sign * plate_z)),
                material=joint_material,
                name=f"distal_plate_{plate_name}",
            )

    part.inertial = Inertial.from_geometry(
        Box((width, max(length, 0.012), thickness)),
        mass=max(0.03, length * width * thickness * 2600.0),
        origin=Origin(xyz=(0.0, length * 0.52, 0.0)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_palm")

    palm_shell = model.material("palm_shell", rgba=(0.17, 0.19, 0.22, 1.0))
    palm_cover = model.material("palm_cover", rgba=(0.23, 0.25, 0.28, 1.0))
    link_metal = model.material("link_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    joint_metal = model.material("joint_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    grip_pad = model.material("grip_pad", rgba=(0.08, 0.09, 0.10, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_WIDTH, PALM_LENGTH, PALM_THICKNESS)),
        material=palm_shell,
        name="palm_block",
    )
    palm.visual(
        Box((PALM_WIDTH * 0.78, PALM_LENGTH * 0.34, PALM_THICKNESS * 0.50)),
        origin=Origin(xyz=(0.0, -0.008, PALM_THICKNESS * 0.38)),
        material=palm_cover,
        name="dorsal_cover",
    )
    palm.visual(
        Box((PALM_WIDTH * 0.60, 0.020, PALM_THICKNESS * 0.62)),
        origin=Origin(xyz=(0.0, -PALM_LENGTH * 0.5 - 0.010, -PALM_THICKNESS * 0.02)),
        material=palm_cover,
        name="wrist_mount",
    )

    for finger_name, x_center, lengths, widths in FINGER_SPECS:
        knuckle_width = widths[0] + 0.004
        palm.visual(
            Box((knuckle_width, 0.012, PALM_THICKNESS * 0.80)),
            origin=Origin(xyz=(x_center, FINGER_ROOT_Y - 0.008, 0.0)),
            material=palm_cover,
            name=f"{finger_name}_root_boss",
        )
        cheek_x = x_center + (FINGER_BARREL_LENGTH * 0.5 + FINGER_CHEEK_THICKNESS * 0.5)
        cheek_y = FINGER_ROOT_Y - 0.002
        for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
            palm.visual(
                Box(
                    (
                        FINGER_CHEEK_THICKNESS,
                        FINGER_CLEVIS_DEPTH,
                        max(widths[0] * 0.72, 0.014),
                    )
                ),
                origin=Origin(
                    xyz=(
                        x_center + side_sign * (FINGER_BARREL_LENGTH * 0.5 + FINGER_CHEEK_THICKNESS * 0.5),
                        cheek_y,
                        0.0,
                    )
                ),
                material=joint_metal,
                name=f"{finger_name}_root_{side_name}_clevis",
            )

    for plate_name, offset_y in (("front", -0.006), ("rear", 0.006)):
        palm.visual(
            Box((0.003, 0.012, 0.016)),
            origin=Origin(
                xyz=(
                    -PALM_WIDTH * 0.5 + 0.0015,
                    THUMB_ROOT_Y + offset_y,
                    THUMB_ROOT_Z,
                )
            ),
            material=joint_metal,
            name=f"thumb_root_{plate_name}_strap",
        )

    palm.inertial = Inertial.from_geometry(
        Box((PALM_WIDTH, PALM_LENGTH + 0.020, PALM_THICKNESS + 0.010)),
        mass=0.85,
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
    )

    for finger_name, x_center, lengths, widths in FINGER_SPECS:
        proximal = _finger_segment(
            model,
            name=f"{finger_name}_proximal",
            length=lengths[0],
            width=widths[0],
            thickness=0.016,
            structure_material=link_metal,
            joint_material=joint_metal,
            pad_material=grip_pad,
            terminal=False,
        )
        middle = _finger_segment(
            model,
            name=f"{finger_name}_middle",
            length=lengths[1],
            width=widths[1],
            thickness=0.014,
            structure_material=link_metal,
            joint_material=joint_metal,
            pad_material=grip_pad,
            terminal=False,
        )
        distal = _finger_segment(
            model,
            name=f"{finger_name}_distal",
            length=lengths[2],
            width=widths[2],
            thickness=0.012,
            structure_material=link_metal,
            joint_material=joint_metal,
            pad_material=grip_pad,
            terminal=True,
        )

        model.articulation(
            f"palm_to_{finger_name}_proximal",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(x_center, FINGER_ROOT_Y, 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=2.0,
                lower=0.0,
                upper=math.radians(78.0),
            ),
        )
        model.articulation(
            f"{finger_name}_proximal_to_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(0.0, lengths[0], 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.2,
                lower=0.0,
                upper=math.radians(92.0),
            ),
        )
        model.articulation(
            f"{finger_name}_middle_to_distal",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(0.0, lengths[1], 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=2.4,
                lower=0.0,
                upper=math.radians(85.0),
            ),
        )

    thumb_base = _thumb_segment(
        model,
        name="thumb_base",
        length=THUMB_LENGTHS[0],
        width=THUMB_WIDTHS[0],
        thickness=THUMB_THICKNESSES[0],
        structure_material=link_metal,
        joint_material=joint_metal,
        pad_material=grip_pad,
        terminal=False,
    )
    thumb_distal = _thumb_segment(
        model,
        name="thumb_distal",
        length=THUMB_LENGTHS[1],
        width=THUMB_WIDTHS[1],
        thickness=THUMB_THICKNESSES[1],
        structure_material=link_metal,
        joint_material=joint_metal,
        pad_material=grip_pad,
        terminal=True,
    )

    model.articulation(
        "palm_to_thumb_base",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_base,
        origin=Origin(
            xyz=(THUMB_ROOT_X, THUMB_ROOT_Y, THUMB_ROOT_Z),
            rpy=(0.0, 0.0, -0.40),
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "thumb_base_to_distal",
        ArticulationType.REVOLUTE,
        parent=thumb_base,
        child=thumb_distal,
        origin=Origin(xyz=(0.0, THUMB_LENGTHS[0], 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.4,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(65.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    palm = object_model.get_part("palm")

    finger_parts: dict[str, tuple[object, object, object]] = {}
    finger_joints: dict[str, tuple[object, object, object]] = {}
    for finger_name, _, lengths, _ in FINGER_SPECS:
        finger_parts[finger_name] = (
            object_model.get_part(f"{finger_name}_proximal"),
            object_model.get_part(f"{finger_name}_middle"),
            object_model.get_part(f"{finger_name}_distal"),
        )
        finger_joints[finger_name] = (
            object_model.get_articulation(f"palm_to_{finger_name}_proximal"),
            object_model.get_articulation(f"{finger_name}_proximal_to_middle"),
            object_model.get_articulation(f"{finger_name}_middle_to_distal"),
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

    root_positions = []
    for finger_name, x_center, lengths, _ in FINGER_SPECS:
        proximal, middle, distal = finger_parts[finger_name]
        root_joint, mid_joint, tip_joint = finger_joints[finger_name]

        ctx.expect_contact(proximal, palm, name=f"{finger_name}_root_knuckle_contact")
        ctx.expect_contact(
            middle,
            proximal,
            name=f"{finger_name}_proximal_middle_knuckle_contact",
        )
        ctx.expect_contact(
            distal,
            middle,
            name=f"{finger_name}_middle_distal_knuckle_contact",
        )
        ctx.expect_origin_gap(
            proximal,
            palm,
            axis="y",
            min_gap=FINGER_ROOT_Y - 0.002,
            max_gap=FINGER_ROOT_Y + 0.002,
            name=f"{finger_name}_root_forward_of_palm",
        )
        ctx.check(
            f"{finger_name}_hinge_axes",
            root_joint.axis == (-1.0, 0.0, 0.0)
            and mid_joint.axis == (-1.0, 0.0, 0.0)
            and tip_joint.axis == (-1.0, 0.0, 0.0),
            "Finger joints should all flex about parallel local knuckle axes.",
        )

        root_position = ctx.part_world_position(proximal)
        assert root_position is not None
        root_positions.append(root_position)

    ctx.check(
        "finger_roots_spread_across_palm",
        all(root_positions[index][0] < root_positions[index + 1][0] for index in range(3)),
        "The four finger roots should be laid out left-to-right across the palm block.",
    )

    thumb_base = object_model.get_part("thumb_base")
    thumb_distal = object_model.get_part("thumb_distal")
    thumb_root_joint = object_model.get_articulation("palm_to_thumb_base")
    thumb_tip_joint = object_model.get_articulation("thumb_base_to_distal")

    ctx.expect_contact(thumb_base, palm, name="thumb_root_knuckle_contact")
    ctx.expect_contact(
        thumb_distal,
        thumb_base,
        name="thumb_interphalangeal_knuckle_contact",
    )
    ctx.check(
        "thumb_hinge_axes",
        thumb_root_joint.axis == (0.0, 0.0, -1.0)
        and thumb_tip_joint.axis == (0.0, 0.0, -1.0),
        "The thumb should oppose the fingers with its own side-mounted hinge axes.",
    )

    index_middle = finger_parts["index"][1]
    index_distal = finger_parts["index"][2]
    index_distal_rest = ctx.part_world_position(index_distal)
    assert index_distal_rest is not None
    with ctx.pose(
        {
            finger_joints["index"][0]: math.radians(52.0),
            finger_joints["index"][1]: math.radians(58.0),
            finger_joints["index"][2]: math.radians(42.0),
        }
    ):
        index_distal_curled = ctx.part_world_position(index_distal)
        assert index_distal_curled is not None
        ctx.check(
            "index_chain_curls_downward",
            index_distal_curled[1] < index_distal_rest[1] - 0.018
            and index_distal_curled[2] < index_distal_rest[2] - 0.012,
            "The index finger distal link should travel inward and downward when flexed.",
        )
        ctx.expect_contact(
            index_middle,
            index_distal,
            name="index_distal_stays_captured_while_flexed",
        )

    thumb_distal_rest = ctx.part_world_position(thumb_distal)
    assert thumb_distal_rest is not None
    with ctx.pose(
        {
            thumb_root_joint: math.radians(40.0),
            thumb_tip_joint: math.radians(34.0),
        }
    ):
        thumb_distal_closed = ctx.part_world_position(thumb_distal)
        assert thumb_distal_closed is not None
        ctx.check(
            "thumb_moves_inward_to_oppose",
            thumb_distal_closed[0] > thumb_distal_rest[0] + 0.010,
            "The thumb should sweep inward from the palm side toward the finger workspace.",
        )
        ctx.expect_contact(
            thumb_base,
            thumb_distal,
            name="thumb_links_stay_captured_while_flexed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
