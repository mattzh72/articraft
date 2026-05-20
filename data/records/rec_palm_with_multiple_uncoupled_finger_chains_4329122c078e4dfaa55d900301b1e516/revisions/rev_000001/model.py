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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PALM_LENGTH = 0.110
PALM_WIDTH = 0.095
PALM_THICKNESS = 0.020
FINGER_ROOT_X = 0.0595
FINGER_ROOT_Y = (-0.033, -0.011, 0.011, 0.033)
FINGER_ROOT_SPACING = FINGER_ROOT_Y[1] - FINGER_ROOT_Y[0]
FINGER_ROOT_AXIS = (0.0, 1.0, 0.0)
THUMB_BASE_ORIGIN = (0.018, 0.0520, 0.0)
THUMB_BASE_YAW = 0.78
THUMB_BASE_AXIS = (0.0, 0.0, -1.0)


def _barrel_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "y":
        return (math.pi / 2.0, 0.0, 0.0)
    if axis == "z":
        return (0.0, 0.0, 0.0)
    raise ValueError(f"Unsupported barrel axis: {axis}")


def _build_chain_segment(
    part,
    *,
    total_length: float,
    width: float,
    thickness: float,
    root_axis: str,
    root_barrel_radius: float,
    root_barrel_length: float,
    neck_length: float,
    neck_width: float,
    body_material,
    joint_material,
    fork_length: float = 0.0,
    fork_gap: float = 0.0,
    fork_cheek_width: float = 0.0,
    tip_pad_length: float = 0.0,
    tip_pad_thickness: float = 0.0,
    pad_material=None,
) -> None:
    part.visual(
        Cylinder(radius=root_barrel_radius, length=root_barrel_length),
        origin=Origin(rpy=_barrel_rpy(root_axis)),
        material=joint_material,
        name="root_barrel",
    )

    neck_thickness = thickness * 0.88
    part.visual(
        Box((neck_length + 0.001, neck_width, neck_thickness)),
        origin=Origin(xyz=(neck_length * 0.5 - 0.0005, 0.0, 0.0)),
        material=body_material,
        name="root_neck",
    )

    body_length = total_length - neck_length - fork_length
    if body_length <= 0.0:
        raise ValueError(f"Invalid segment length for part {part.name}")
    part.visual(
        Box((body_length + 0.001, width, thickness)),
        origin=Origin(
            xyz=(neck_length + body_length * 0.5 - 0.0005, 0.0, 0.0)
        ),
        material=body_material,
        name="body_shell",
    )

    if fork_length > 0.0:
        cheek_y = fork_gap * 0.5 + fork_cheek_width * 0.5
        for side_name, sign in (("inner", -1.0), ("outer", 1.0)):
            part.visual(
                Box((fork_length + 0.001, fork_cheek_width, thickness * 0.95)),
                origin=Origin(
                    xyz=(
                        total_length - fork_length * 0.5 - 0.0005,
                        sign * cheek_y,
                        0.0,
                    )
                ),
                material=joint_material,
                name=f"distal_fork_{side_name}",
            )

    if tip_pad_length > 0.0 and tip_pad_thickness > 0.0 and pad_material is not None:
        part.visual(
            Box((tip_pad_length, width * 0.72, tip_pad_thickness)),
            origin=Origin(
                xyz=(
                    total_length - tip_pad_length * 0.5,
                    0.0,
                    -thickness * 0.18,
                )
            ),
            material=pad_material,
            name="tip_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_palm_hand")

    palm_material = model.material("palm_housing", rgba=(0.18, 0.20, 0.23, 1.0))
    frame_material = model.material("frame_dark", rgba=(0.10, 0.11, 0.13, 1.0))
    finger_material = model.material("finger_shell", rgba=(0.46, 0.49, 0.54, 1.0))
    thumb_material = model.material("thumb_shell", rgba=(0.52, 0.50, 0.46, 1.0))
    joint_material = model.material("joint_black", rgba=(0.07, 0.08, 0.09, 1.0))
    pad_material = model.material("pad_rubber", rgba=(0.18, 0.25, 0.31, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_LENGTH, PALM_WIDTH, PALM_THICKNESS)),
        material=palm_material,
        name="palm_shell",
    )
    palm.visual(
        Box((0.032, 0.056, 0.016)),
        origin=Origin(xyz=(-0.066, 0.0, -0.001)),
        material=frame_material,
        name="wrist_stub",
    )

    finger_mount_depth = 0.010
    finger_mount_height = 0.014
    finger_mount_gap = 0.0080
    finger_mount_cheek_width = 0.0032
    finger_mount_offset = finger_mount_gap * 0.5 + finger_mount_cheek_width * 0.5
    finger_mount_x = 0.0575

    for index, center_y in enumerate(FINGER_ROOT_Y, start=1):
        for side_name, sign in (("left", -1.0), ("right", 1.0)):
            palm.visual(
                Box((finger_mount_depth, finger_mount_cheek_width, finger_mount_height)),
                origin=Origin(
                    xyz=(
                        finger_mount_x,
                        center_y + sign * finger_mount_offset,
                        0.0,
                    )
                ),
                material=frame_material,
                name=f"finger_{index}_mount_{side_name}",
            )

    thumb_plate_x = 0.018
    thumb_plate_y = 0.0490
    thumb_plate_z = 0.0075
    thumb_plate_size = (0.016, 0.008, 0.003)
    palm.visual(
        Box(thumb_plate_size),
        origin=Origin(xyz=(thumb_plate_x, thumb_plate_y, thumb_plate_z)),
        material=frame_material,
        name="thumb_mount_upper",
    )
    palm.visual(
        Box(thumb_plate_size),
        origin=Origin(xyz=(thumb_plate_x, thumb_plate_y, -thumb_plate_z)),
        material=frame_material,
        name="thumb_mount_lower",
    )

    finger_specs = (
        {
            "prefix": "finger_1",
            "prox_len": 0.038,
            "mid_len": 0.030,
            "tip_len": 0.024,
            "width": 0.0142,
            "mid_width": 0.0128,
            "tip_width": 0.0112,
        },
        {
            "prefix": "finger_2",
            "prox_len": 0.041,
            "mid_len": 0.032,
            "tip_len": 0.025,
            "width": 0.0148,
            "mid_width": 0.0134,
            "tip_width": 0.0118,
        },
        {
            "prefix": "finger_3",
            "prox_len": 0.040,
            "mid_len": 0.031,
            "tip_len": 0.024,
            "width": 0.0144,
            "mid_width": 0.0130,
            "tip_width": 0.0114,
        },
        {
            "prefix": "finger_4",
            "prox_len": 0.036,
            "mid_len": 0.028,
            "tip_len": 0.022,
            "width": 0.0134,
            "mid_width": 0.0120,
            "tip_width": 0.0106,
        },
    )

    for index, spec in enumerate(finger_specs):
        prefix = spec["prefix"]
        proximal = model.part(f"{prefix}_proximal")
        middle = model.part(f"{prefix}_middle")
        distal = model.part(f"{prefix}_distal")

        _build_chain_segment(
            proximal,
            total_length=spec["prox_len"],
            width=spec["width"],
            thickness=0.012,
            root_axis="y",
            root_barrel_radius=0.0038,
            root_barrel_length=0.0080,
            neck_length=0.0070,
            neck_width=0.0068,
            fork_length=0.0075,
            fork_gap=0.0076,
            fork_cheek_width=0.0031,
            body_material=finger_material,
            joint_material=joint_material,
        )
        _build_chain_segment(
            middle,
            total_length=spec["mid_len"],
            width=spec["mid_width"],
            thickness=0.011,
            root_axis="y",
            root_barrel_radius=0.0032,
            root_barrel_length=0.0076,
            neck_length=0.0065,
            neck_width=0.0066,
            fork_length=0.0070,
            fork_gap=0.0072,
            fork_cheek_width=0.0030,
            body_material=finger_material,
            joint_material=joint_material,
        )
        _build_chain_segment(
            distal,
            total_length=spec["tip_len"],
            width=spec["tip_width"],
            thickness=0.010,
            root_axis="y",
            root_barrel_radius=0.0030,
            root_barrel_length=0.0072,
            neck_length=0.0060,
            neck_width=0.0064,
            body_material=finger_material,
            joint_material=joint_material,
            tip_pad_length=min(0.010, spec["tip_len"] * 0.45),
            tip_pad_thickness=0.0032,
            pad_material=pad_material,
        )

        model.articulation(
            f"palm_to_{prefix}_root",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(FINGER_ROOT_X, FINGER_ROOT_Y[index], 0.0)),
            axis=FINGER_ROOT_AXIS,
            motion_limits=MotionLimits(
                effort=9.0,
                velocity=2.6,
                lower=0.0,
                upper=1.20,
            ),
        )
        model.articulation(
            f"{prefix}_proximal_to_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(spec["prox_len"], 0.0, 0.0)),
            axis=FINGER_ROOT_AXIS,
            motion_limits=MotionLimits(
                effort=7.0,
                velocity=2.8,
                lower=0.0,
                upper=1.35,
            ),
        )
        model.articulation(
            f"{prefix}_middle_to_distal",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(spec["mid_len"], 0.0, 0.0)),
            axis=FINGER_ROOT_AXIS,
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=3.2,
                lower=0.0,
                upper=1.10,
            ),
        )

    thumb_base = model.part("thumb_base")
    thumb_middle = model.part("thumb_middle")
    thumb_distal = model.part("thumb_distal")

    _build_chain_segment(
        thumb_base,
        total_length=0.032,
        width=0.0135,
        thickness=0.011,
        root_axis="z",
        root_barrel_radius=0.0036,
        root_barrel_length=0.012,
        neck_length=0.0060,
        neck_width=0.0100,
        fork_length=0.0070,
        fork_gap=0.0070,
        fork_cheek_width=0.0030,
        body_material=thumb_material,
        joint_material=joint_material,
    )
    _build_chain_segment(
        thumb_middle,
        total_length=0.026,
        width=0.0128,
        thickness=0.010,
        root_axis="y",
        root_barrel_radius=0.0031,
        root_barrel_length=0.0070,
        neck_length=0.0060,
        neck_width=0.0062,
        fork_length=0.0065,
        fork_gap=0.0068,
        fork_cheek_width=0.0028,
        body_material=thumb_material,
        joint_material=joint_material,
    )
    _build_chain_segment(
        thumb_distal,
        total_length=0.020,
        width=0.0110,
        thickness=0.009,
        root_axis="y",
        root_barrel_radius=0.0028,
        root_barrel_length=0.0068,
        neck_length=0.0050,
        neck_width=0.0058,
        body_material=thumb_material,
        joint_material=joint_material,
        tip_pad_length=0.0085,
        tip_pad_thickness=0.0030,
        pad_material=pad_material,
    )

    model.articulation(
        "palm_to_thumb_base",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_base,
        origin=Origin(xyz=THUMB_BASE_ORIGIN, rpy=(0.0, 0.0, THUMB_BASE_YAW)),
        axis=THUMB_BASE_AXIS,
        motion_limits=MotionLimits(
            effort=7.0,
            velocity=2.2,
            lower=0.0,
            upper=0.80,
        ),
    )
    model.articulation(
        "thumb_base_to_middle",
        ArticulationType.REVOLUTE,
        parent=thumb_base,
        child=thumb_middle,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        axis=FINGER_ROOT_AXIS,
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.8,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "thumb_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=thumb_middle,
        child=thumb_distal,
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        axis=FINGER_ROOT_AXIS,
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=1.00,
        ),
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

    palm = object_model.get_part("palm")
    finger_proximals = []
    finger_distals = []
    finger_root_joints = []

    for index in range(1, 5):
        proximal = object_model.get_part(f"finger_{index}_proximal")
        middle = object_model.get_part(f"finger_{index}_middle")
        distal = object_model.get_part(f"finger_{index}_distal")
        root_joint = object_model.get_articulation(f"palm_to_finger_{index}_root")
        middle_joint = object_model.get_articulation(f"finger_{index}_proximal_to_middle")
        distal_joint = object_model.get_articulation(f"finger_{index}_middle_to_distal")

        finger_proximals.append(proximal)
        finger_distals.append(distal)
        finger_root_joints.append(root_joint)

        ctx.check(
            f"finger {index} root joint is revolute about palm knuckle axis",
            root_joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(root_joint.axis) == FINGER_ROOT_AXIS,
            details=f"type={root_joint.articulation_type}, axis={root_joint.axis}",
        )
        ctx.check(
            f"finger {index} chain keeps secondary hinges aligned",
            middle_joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(middle_joint.axis) == FINGER_ROOT_AXIS
            and distal_joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(distal_joint.axis) == FINGER_ROOT_AXIS,
            details=(
                f"middle_axis={middle_joint.axis}, distal_axis={distal_joint.axis}, "
                f"middle_type={middle_joint.articulation_type}, distal_type={distal_joint.articulation_type}"
            ),
        )
        ctx.expect_origin_gap(
            proximal,
            palm,
            axis="x",
            min_gap=0.057,
            max_gap=0.062,
            name=f"finger {index} root sits on the palm front edge",
        )

    thumb_base = object_model.get_part("thumb_base")
    thumb_distal = object_model.get_part("thumb_distal")
    thumb_root_joint = object_model.get_articulation("palm_to_thumb_base")

    ctx.check(
        "thumb base joint uses its own side-mounted revolute axis",
        thumb_root_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(thumb_root_joint.axis) == THUMB_BASE_AXIS,
        details=f"type={thumb_root_joint.articulation_type}, axis={thumb_root_joint.axis}",
    )
    ctx.expect_origin_gap(
        thumb_base,
        palm,
        axis="y",
        min_gap=0.049,
        max_gap=0.055,
        name="thumb base sits on the palm side mount",
    )

    root_positions = [ctx.part_world_position(part) for part in finger_proximals]
    spacing_ok = all(position is not None for position in root_positions)
    if spacing_ok:
        y_positions = [position[1] for position in root_positions if position is not None]
        x_positions = [position[0] for position in root_positions if position is not None]
        z_positions = [position[2] for position in root_positions if position is not None]
        spacing_ok = (
            max(abs((y_positions[i + 1] - y_positions[i]) - FINGER_ROOT_SPACING) for i in range(3))
            <= 1e-6
            and max(abs(x - FINGER_ROOT_X) for x in x_positions) <= 1e-6
            and max(abs(z) for z in z_positions) <= 1e-6
        )
    ctx.check(
        "four finger roots are evenly spaced across one front edge",
        spacing_ok,
        details=f"root_positions={root_positions}",
    )

    rest_distal_positions = [ctx.part_world_position(part) for part in finger_distals]
    for moving_index, root_joint in enumerate(finger_root_joints):
        with ctx.pose({root_joint: 0.70}):
            posed_distal_positions = [ctx.part_world_position(part) for part in finger_distals]
        moved = posed_distal_positions[moving_index]
        rest = rest_distal_positions[moving_index]
        moved_ok = (
            moved is not None
            and rest is not None
            and moved[2] < rest[2] - 0.020
        )
        others_static = True
        for check_index, posed in enumerate(posed_distal_positions):
            rest_other = rest_distal_positions[check_index]
            if check_index == moving_index or posed is None or rest_other is None:
                continue
            if any(abs(posed[axis] - rest_other[axis]) > 1e-7 for axis in range(3)):
                others_static = False
                break
        ctx.check(
            f"finger {moving_index + 1} root drives only its own chain",
            moved_ok and others_static,
            details=(
                f"rest={rest_distal_positions}, posed={posed_distal_positions}, "
                f"moving_index={moving_index}"
            ),
        )

    thumb_rest = ctx.part_world_position(thumb_distal)
    with ctx.pose({thumb_root_joint: 0.70}):
        thumb_opposed = ctx.part_world_position(thumb_distal)
    ctx.check(
        "thumb base sweeps inward from the palm side",
        thumb_rest is not None
        and thumb_opposed is not None
        and thumb_opposed[1] < thumb_rest[1] - 0.025
        and thumb_opposed[0] > thumb_rest[0] + 0.010,
        details=f"rest={thumb_rest}, opposed={thumb_opposed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
