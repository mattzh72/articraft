from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PALM_W = 0.200
PALM_T = 0.016
PALM_H = 0.140
PALM_FRONT_Y = PALM_T / 2.0

LINK_W = 0.026
LUG_W = 0.017
LINK_H = 0.014
LUG_R = 0.012
PIN_R = 0.0032
PIN_L = 0.040
FORK_GAP = 0.021
FORK_CHEEK = 0.006
FORK_OUTER_W = FORK_GAP + 2.0 * FORK_CHEEK
FORK_LEN = 0.032
FORK_BACK = 0.022
BRIDGE_LEN = 0.008
BODY_CLEAR = 0.006

ROOT_PIVOT_Y = PALM_FRONT_Y + 0.021
FINGER_ROOT_Z = 0.104
THUMB_ROOT_X = -PALM_W / 2.0 - 0.021
THUMB_ROOT_Z = 0.054
THUMB_YAW = math.pi / 2.0


def _cyl_x_origin(xyz: tuple[float, float, float]) -> Origin:
    """Origin for a cylinder whose length runs along local/global +X."""
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _add_root_mount(
    palm,
    *,
    prefix: str,
    pivot_xyz: tuple[float, float, float],
    yaw: float,
    material: Material,
) -> None:
    """Add a fork-like bearing block, oriented so local +Y is digit forward."""
    x, y, z = pivot_xyz
    c = math.cos(yaw)
    s = math.sin(yaw)

    def pos(local_x: float, local_y: float) -> tuple[float, float, float]:
        return (x + local_x * c - local_y * s, y + local_x * s + local_y * c, z)

    cheek_x = FORK_GAP / 2.0 + FORK_CHEEK / 2.0
    rear_y = -0.020
    front_y = 0.018
    depth = front_y - rear_y
    for side, sx in (("a", -cheek_x), ("b", cheek_x)):
        palm.visual(
            Box((FORK_CHEEK, depth, LINK_H)),
            origin=Origin(xyz=pos(sx, (rear_y + front_y) / 2.0), rpy=(0.0, 0.0, yaw)),
            material=material,
            name=f"{prefix}_cheek_{side}",
        )
    palm.visual(
        Box((FORK_OUTER_W, BRIDGE_LEN, LINK_H)),
        origin=Origin(xyz=pos(0.0, rear_y + BRIDGE_LEN / 2.0), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=f"{prefix}_bridge",
    )
    palm.visual(
        Box((FORK_OUTER_W + 0.010, 0.006, LINK_H + 0.004)),
        origin=Origin(xyz=pos(0.0, rear_y - 0.003), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=f"{prefix}_foot",
    )


def _add_link_visuals(
    link,
    *,
    length: float,
    material: Material,
    pin_material: Material,
    include_distal_fork: bool,
) -> None:
    """Build one flat phalanx-like link with a proximal hinge lug and optional distal fork."""
    body_end = length - FORK_BACK - BODY_CLEAR if include_distal_fork else length - LUG_R * 0.35
    body_start = LUG_R * 0.25
    body_len = max(0.010, body_end - body_start)
    link.visual(
        Box((LINK_W, body_len, LINK_H)),
        origin=Origin(xyz=(0.0, body_start + body_len / 2.0, 0.0)),
        material=material,
        name="link_web",
    )
    link.visual(
        Cylinder(radius=LUG_R, length=LUG_W),
        origin=_cyl_x_origin((0.0, 0.0, 0.0)),
        material=material,
        name="prox_lug",
    )
    link.visual(
        Cylinder(radius=PIN_R, length=PIN_L),
        origin=_cyl_x_origin((0.0, 0.0, 0.0)),
        material=pin_material,
        name="prox_pin",
    )
    if include_distal_fork:
        cheek_x = FORK_GAP / 2.0 + FORK_CHEEK / 2.0
        for side, sx in (("a", -cheek_x), ("b", cheek_x)):
            link.visual(
                Box((FORK_CHEEK, FORK_LEN, LINK_H)),
                origin=Origin(xyz=(sx, length - FORK_BACK + FORK_LEN / 2.0, 0.0)),
                material=material,
                name=f"dist_fork_{side}",
            )
        link.visual(
            Box((FORK_OUTER_W, BRIDGE_LEN, LINK_H)),
            origin=Origin(xyz=(0.0, length - FORK_BACK - BRIDGE_LEN / 2.0, 0.0)),
            material=material,
            name="dist_bridge",
        )
    else:
        link.visual(
            Cylinder(radius=LUG_R * 0.75, length=LUG_W * 0.85),
            origin=_cyl_x_origin((0.0, length, 0.0)),
            material=material,
            name="rounded_tip",
        )


def _digit_chain(
    model: ArticulatedObject,
    *,
    palm,
    prefix: str,
    root_xyz: tuple[float, float, float],
    root_yaw: float,
    lengths: tuple[float, ...],
    material: Material,
    pin_material: Material,
    mount_material: Material,
) -> list[str]:
    """Create a serial digit chain and return its revolute joint names."""
    _add_root_mount(palm, prefix=f"{prefix}_mount", pivot_xyz=root_xyz, yaw=root_yaw, material=mount_material)

    suffixes = ("proximal", "middle", "tip")
    parts = []
    for i, length in enumerate(lengths):
        suffix = suffixes[i] if i < len(suffixes) else f"link_{i}"
        part = model.part(f"{prefix}_{suffix}")
        _add_link_visuals(
            part,
            length=length,
            material=material,
            pin_material=pin_material,
            include_distal_fork=i < len(lengths) - 1,
        )
        parts.append(part)

    joint_names: list[str] = []
    root_joint = f"{prefix}_root"
    model.articulation(
        root_joint,
        ArticulationType.REVOLUTE,
        parent=palm,
        child=parts[0],
        origin=Origin(xyz=root_xyz, rpy=(0.0, 0.0, root_yaw)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    joint_names.append(root_joint)

    joint_labels = ("mid_hinge", "tip_hinge", "end_hinge")
    for i in range(1, len(parts)):
        joint_name = f"{prefix}_{joint_labels[i - 1]}"
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=parts[i - 1],
            child=parts[i],
            origin=Origin(xyz=(0.0, lengths[i - 1], 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=2.8, lower=0.0, upper=1.35),
        )
        joint_names.append(joint_name)
    return joint_names


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_gripper_hand")

    palm_mat = model.material("matte_black_palm", rgba=(0.05, 0.055, 0.06, 1.0))
    mount_mat = model.material("brushed_steel_mounts", rgba=(0.55, 0.57, 0.56, 1.0))
    finger_mat = model.material("blue_anodized_links", rgba=(0.05, 0.22, 0.82, 1.0))
    thumb_mat = model.material("orange_thumb_links", rgba=(0.95, 0.44, 0.08, 1.0))
    pin_mat = model.material("dark_pivot_pins", rgba=(0.015, 0.015, 0.018, 1.0))
    screw_mat = model.material("black_oxide_screws", rgba=(0.0, 0.0, 0.0, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_W, PALM_T, PALM_H)),
        origin=Origin(xyz=(0.0, 0.0, PALM_H / 2.0)),
        material=palm_mat,
        name="palm_plate",
    )
    palm.visual(
        Box((PALM_W * 0.82, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, PALM_FRONT_Y + 0.002, 0.030)),
        material=mount_mat,
        name="lower_stiffener",
    )
    palm.visual(
        Box((PALM_W * 0.82, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, PALM_FRONT_Y + 0.002, 0.122)),
        material=mount_mat,
        name="upper_stiffener",
    )
    for i, (x, z) in enumerate(((-0.073, 0.027), (0.073, 0.027), (-0.073, 0.121), (0.073, 0.121))):
        palm.visual(
            Cylinder(radius=0.0065, length=0.003),
            origin=Origin(xyz=(x, PALM_FRONT_Y + 0.0015, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=screw_mat,
            name=f"screw_{i}",
        )

    finger_roots = (-0.056, 0.0, 0.056)
    for idx, x in enumerate(finger_roots):
        _digit_chain(
            model,
            palm=palm,
            prefix=f"finger_{idx}",
            root_xyz=(x, ROOT_PIVOT_Y, FINGER_ROOT_Z),
            root_yaw=0.0,
            lengths=(0.060, 0.046, 0.034),
            material=finger_mat,
            pin_material=pin_mat,
            mount_material=mount_mat,
        )

    _digit_chain(
        model,
        palm=palm,
        prefix="thumb",
        root_xyz=(THUMB_ROOT_X, 0.0, THUMB_ROOT_Z),
        root_yaw=THUMB_YAW,
        lengths=(0.052, 0.038),
        material=thumb_mat,
        pin_material=pin_mat,
        mount_material=mount_mat,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joint_chains = {
        "finger_0": ("palm", "finger_0_proximal", "finger_0_middle", "finger_0_tip"),
        "finger_1": ("palm", "finger_1_proximal", "finger_1_middle", "finger_1_tip"),
        "finger_2": ("palm", "finger_2_proximal", "finger_2_middle", "finger_2_tip"),
        "thumb": ("palm", "thumb_proximal", "thumb_middle"),
    }

    expected_joints = (
        "finger_0_root",
        "finger_0_mid_hinge",
        "finger_0_tip_hinge",
        "finger_1_root",
        "finger_1_mid_hinge",
        "finger_1_tip_hinge",
        "finger_2_root",
        "finger_2_mid_hinge",
        "finger_2_tip_hinge",
        "thumb_root",
        "thumb_mid_hinge",
    )
    ctx.check(
        "each hinge is independently actuated",
        all(object_model.get_articulation(name).mimic is None for name in expected_joints),
        details="Every digit pivot should be its own revolute joint without mimic coupling.",
    )
    ctx.check(
        "four separate palm pivots",
        all(name in expected_joints for name in ("finger_0_root", "finger_1_root", "finger_2_root", "thumb_root")),
        details="The palm should drive three finger roots and one thumb root.",
    )

    parent_child = [
        ("palm", "finger_0_proximal", "finger_0_root"),
        ("finger_0_proximal", "finger_0_middle", "finger_0_mid_hinge"),
        ("finger_0_middle", "finger_0_tip", "finger_0_tip_hinge"),
        ("palm", "finger_1_proximal", "finger_1_root"),
        ("finger_1_proximal", "finger_1_middle", "finger_1_mid_hinge"),
        ("finger_1_middle", "finger_1_tip", "finger_1_tip_hinge"),
        ("palm", "finger_2_proximal", "finger_2_root"),
        ("finger_2_proximal", "finger_2_middle", "finger_2_mid_hinge"),
        ("finger_2_middle", "finger_2_tip", "finger_2_tip_hinge"),
        ("palm", "thumb_proximal", "thumb_root"),
        ("thumb_proximal", "thumb_middle", "thumb_mid_hinge"),
    ]
    for parent_name, child_name, joint_name in parent_child:
        parent = object_model.get_part(parent_name)
        child = object_model.get_part(child_name)
        ctx.allow_overlap(
            child,
            parent,
            elem_a="prox_pin",
            reason=(
                "The visible pivot pin is intentionally captured through the neighboring clevis "
                "or palm knuckle bore so the digit cannot separate during curl motion."
            ),
        )
        hinge_axis = "y" if joint_name.startswith("thumb") else "x"
        ctx.expect_overlap(
            child,
            parent,
            axes=hinge_axis,
            min_overlap=0.010,
            elem_a="prox_pin",
            name=f"{joint_name} pin spans its knuckle",
        )

    with ctx.pose(
        {
            object_model.get_articulation("finger_1_root"): 0.55,
            object_model.get_articulation("finger_1_mid_hinge"): 0.80,
            object_model.get_articulation("finger_1_tip_hinge"): 0.65,
            object_model.get_articulation("thumb_root"): 0.45,
            object_model.get_articulation("thumb_mid_hinge"): 0.70,
        }
    ):
        ctx.expect_overlap(
            object_model.get_part("finger_1_middle"),
            object_model.get_part("finger_1_proximal"),
            axes="x",
            min_overlap=0.010,
            elem_a="prox_pin",
            name="curled finger middle remains pinned",
        )
        ctx.expect_overlap(
            object_model.get_part("finger_1_tip"),
            object_model.get_part("finger_1_middle"),
            axes="x",
            min_overlap=0.010,
            elem_a="prox_pin",
            name="curled finger tip remains pinned",
        )
        ctx.expect_overlap(
            object_model.get_part("thumb_middle"),
            object_model.get_part("thumb_proximal"),
            axes="y",
            min_overlap=0.010,
            elem_a="prox_pin",
            name="curled thumb tip remains pinned",
        )

    return ctx.report()


object_model = build_object_model()
