from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


FLEX_ANGLE = math.radians(80.0)
PALM_X = 0.120
PALM_Y = 0.190
PALM_Z = 0.018
ROOT_X = PALM_X / 2.0 + 0.021
ROOT_Z = PALM_Z / 2.0 + 0.013
BARREL_R = 0.010
BARREL_LEN = 0.024
FINGER_WIDTH = 0.020
FINGER_HEIGHT = 0.014


def _barrel_origin(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_segment_visuals(
    part,
    *,
    length: float,
    body_material,
    metal_material,
    tip_material=None,
    terminal: bool = False,
) -> None:
    """Add a rigid phalanx-like link with its hinge barrel at the part frame."""
    part.visual(
        Cylinder(radius=BARREL_R, length=BARREL_LEN),
        origin=_barrel_origin(),
        material=metal_material,
        name="hinge_barrel",
    )

    beam_start = 0.0045
    beam_end = length - (0.005 if terminal else BARREL_R + 0.001)
    part.visual(
        Box((beam_end - beam_start, FINGER_WIDTH, FINGER_HEIGHT)),
        origin=Origin(xyz=((beam_start + beam_end) / 2.0, 0.0, 0.0)),
        material=body_material,
        name="beam",
    )

    part.visual(
        Box((beam_end - beam_start - 0.006, 0.004, FINGER_HEIGHT + 0.004)),
        origin=Origin(xyz=((beam_start + beam_end) / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="center_spine",
    )

    if not terminal:
        part.visual(
            Box((0.004, FINGER_WIDTH, 0.021)),
            origin=Origin(xyz=(length - 0.014, 0.0, 0.0035)),
            material=body_material,
            name="distal_upright",
        )
        part.visual(
            Box((0.026, FINGER_WIDTH, 0.004)),
            origin=Origin(xyz=(length - 0.002, 0.0, BARREL_R + 0.002)),
            material=body_material,
            name="distal_top_strap",
        )

    if terminal:
        part.visual(
            Sphere(radius=0.0105),
            origin=Origin(xyz=(length, 0.0, 0.0)),
            material=tip_material or body_material,
            name="rounded_tip",
        )


def _add_palm_finger_knuckle(palm, *, y: float, name: str, palm_material, metal_material) -> None:
    """Stationary clevis around one front-edge finger root barrel."""
    palm.visual(
        Box((0.015, 0.034, 0.024)),
        origin=Origin(xyz=(ROOT_X - 0.0185, y, ROOT_Z - 0.002)),
        material=palm_material,
        name=f"{name}_base",
    )
    for suffix, dy in (("side_0", -0.015), ("side_1", 0.015)):
        palm.visual(
            Box((0.022, 0.006, 0.024)),
            origin=Origin(xyz=(ROOT_X - 0.001, y + dy, ROOT_Z - 0.002)),
            material=palm_material,
            name=f"{name}_{suffix}",
        )
    palm.visual(
        Cylinder(radius=0.003, length=0.043),
        origin=_barrel_origin(ROOT_X, y, ROOT_Z),
        material=metal_material,
        name=f"{name}_pin",
    )


def _add_digit(
    model: ArticulatedObject,
    *,
    parent,
    prefix: str,
    root_origin: Origin,
    lengths: tuple[float, ...],
    body_material,
    metal_material,
    tip_material,
) -> None:
    previous = parent
    for idx, length in enumerate(lengths):
        segment = model.part(f"{prefix}_segment_{idx}")
        _add_segment_visuals(
            segment,
            length=length,
            body_material=body_material,
            metal_material=metal_material,
            tip_material=tip_material,
            terminal=idx == len(lengths) - 1,
        )
        joint_origin = root_origin if idx == 0 else Origin(xyz=(lengths[idx - 1], 0.0, 0.0))
        model.articulation(
            f"{prefix}_joint_{idx}",
            ArticulationType.REVOLUTE,
            parent=previous,
            child=segment,
            origin=joint_origin,
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.5, velocity=4.0, lower=0.0, upper=FLEX_ANGLE),
        )
        previous = segment


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="anthropomorphic_gripper_palm")

    palm_mat = model.material("matte_black_anodized", rgba=(0.02, 0.024, 0.030, 1.0))
    dark_link = model.material("dark_polymer_links", rgba=(0.09, 0.10, 0.11, 1.0))
    fingertip = model.material("soft_gray_fingertips", rgba=(0.32, 0.34, 0.35, 1.0))
    metal = model.material("brushed_steel_pins", rgba=(0.72, 0.72, 0.68, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_X, PALM_Y, PALM_Z)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=palm_mat,
        name="palm_plate",
    )
    palm.visual(
        Box((0.070, 0.092, 0.010)),
        origin=Origin(xyz=(-0.026, 0.0, PALM_Z / 2.0 + 0.004)),
        material=palm_mat,
        name="raised_center_rib",
    )
    palm.visual(
        Box((0.025, 0.150, 0.009)),
        origin=Origin(xyz=(PALM_X / 2.0 - 0.018, 0.0, PALM_Z / 2.0 + 0.0035)),
        material=palm_mat,
        name="front_reinforcement",
    )

    for screw_idx, (x, y) in enumerate(((-0.038, -0.064), (-0.038, 0.064), (0.030, -0.064), (0.030, 0.064))):
        palm.visual(
            Cylinder(radius=0.006, length=0.0035),
            origin=Origin(xyz=(x, y, PALM_Z / 2.0 + 0.0015)),
            material=metal,
            name=f"screw_head_{screw_idx}",
        )

    finger_y = (-0.063, -0.021, 0.021, 0.063)
    for i, y in enumerate(finger_y):
        _add_palm_finger_knuckle(
            palm,
            y=y,
            name=f"finger_{i}_knuckle",
            palm_material=palm_mat,
            metal_material=metal,
        )
        _add_digit(
            model,
            parent=palm,
            prefix=f"finger_{i}",
            root_origin=Origin(xyz=(ROOT_X, y, ROOT_Z)),
            lengths=(0.056, 0.045, 0.034),
            body_material=dark_link,
            metal_material=metal,
            tip_material=fingertip,
        )

    thumb_yaw = -1.04
    thumb_root = (0.016, -PALM_Y / 2.0 - 0.021, ROOT_Z)
    mount_back = -0.026
    palm.visual(
        Box((0.030, 0.036, 0.022)),
        origin=Origin(
            xyz=(
                thumb_root[0] + math.cos(thumb_yaw) * mount_back,
                thumb_root[1] + math.sin(thumb_yaw) * mount_back,
                ROOT_Z - 0.002,
            ),
            rpy=(0.0, 0.0, thumb_yaw),
        ),
        material=palm_mat,
        name="thumb_side_mount",
    )
    for suffix, local_y in (("side_0", -0.016), ("side_1", 0.016)):
        palm.visual(
            Box((0.022, 0.006, 0.024)),
            origin=Origin(
                xyz=(
                    thumb_root[0] + math.cos(thumb_yaw + math.pi / 2.0) * local_y,
                    thumb_root[1] + math.sin(thumb_yaw + math.pi / 2.0) * local_y,
                    ROOT_Z - 0.002,
                ),
                rpy=(0.0, 0.0, thumb_yaw),
            ),
            material=palm_mat,
            name=f"thumb_knuckle_{suffix}",
        )
    palm.visual(
        Box((0.015, 0.040, 0.022)),
        origin=Origin(
            xyz=(
                thumb_root[0] + math.cos(thumb_yaw) * -0.018,
                thumb_root[1] + math.sin(thumb_yaw) * -0.018,
                ROOT_Z - 0.002,
            ),
            rpy=(0.0, 0.0, thumb_yaw),
        ),
        material=palm_mat,
        name="thumb_knuckle_base",
    )
    palm.visual(
        Cylinder(radius=0.003, length=0.043),
        origin=Origin(xyz=thumb_root, rpy=(math.pi / 2.0, 0.0, thumb_yaw)),
        material=metal,
        name="thumb_root_pin",
    )
    _add_digit(
        model,
        parent=palm,
        prefix="thumb",
        root_origin=Origin(xyz=thumb_root, rpy=(0.0, 0.0, thumb_yaw)),
        lengths=(0.052, 0.039),
        body_material=dark_link,
        metal_material=metal,
        tip_material=fingertip,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joint_names = [joint.name for joint in object_model.articulations]
    finger_joint_names = [f"finger_{i}_joint_{j}" for i in range(4) for j in range(3)]
    thumb_joint_names = [f"thumb_joint_{j}" for j in range(2)]
    all_digit_joint_names = finger_joint_names + thumb_joint_names

    ctx.check(
        "four fingers have three joints and thumb has two",
        len(joint_names) == 14 and set(joint_names) == set(all_digit_joint_names),
        details=f"authored joints={joint_names}",
    )

    all_limits_ok = True
    all_uncoupled = True
    for joint_name in all_digit_joint_names:
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        all_limits_ok = all_limits_ok and limits is not None and abs(limits.lower - 0.0) < 1e-6 and abs(limits.upper - FLEX_ANGLE) < 1e-6
        all_uncoupled = all_uncoupled and joint.mimic is None
    ctx.check(
        "every digit joint flexes independently through eighty degrees",
        all_limits_ok and all_uncoupled,
        details="Expected lower=0, upper=80 degrees, and no mimic coupling on every revolute joint.",
    )

    root_ys = [object_model.get_articulation(f"finger_{i}_joint_0").origin.xyz[1] for i in range(4)]
    root_xs = [object_model.get_articulation(f"finger_{i}_joint_0").origin.xyz[0] for i in range(4)]
    spacings = [root_ys[i + 1] - root_ys[i] for i in range(3)]
    ctx.check(
        "finger root knuckles are evenly spaced on the front edge",
        all(abs(x - ROOT_X) < 1e-6 for x in root_xs)
        and all(0.039 <= spacing <= 0.045 for spacing in spacings),
        details=f"root_xs={root_xs}, root_ys={root_ys}, spacings={spacings}",
    )

    for i in range(4):
        ctx.allow_overlap(
            "palm",
            f"finger_{i}_segment_0",
            elem_a=f"finger_{i}_knuckle_pin",
            elem_b="hinge_barrel",
            reason="The steel root pin is intentionally captured inside the proximal hinge barrel.",
        )
        ctx.expect_overlap(
            "palm",
            f"finger_{i}_segment_0",
            axes="xyz",
            elem_a=f"finger_{i}_knuckle_pin",
            elem_b="hinge_barrel",
            min_overlap=0.003,
            name=f"finger {i} root pin passes through barrel",
        )

    ctx.allow_overlap(
        "palm",
        "thumb_segment_0",
        elem_a="thumb_root_pin",
        elem_b="hinge_barrel",
        reason="The thumb root pin is intentionally captured inside the thumb hinge barrel.",
    )
    ctx.expect_overlap(
        "palm",
        "thumb_segment_0",
        axes="xyz",
        elem_a="thumb_root_pin",
        elem_b="hinge_barrel",
        min_overlap=0.003,
        name="thumb root pin passes through barrel",
    )

    def z_center(part_name: str, elem_name: str) -> float | None:
        bounds = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if bounds is None:
            return None
        return (bounds[0][2] + bounds[1][2]) / 2.0

    root_joint = object_model.get_articulation("finger_1_joint_0")
    rest_z = z_center("finger_1_segment_0", "beam")
    with ctx.pose({root_joint: FLEX_ANGLE}):
        flexed_z = z_center("finger_1_segment_0", "beam")
    ctx.check(
        "finger flexion curls the link downward from the palm",
        rest_z is not None and flexed_z is not None and flexed_z < rest_z - 0.020,
        details=f"rest_z={rest_z}, flexed_z={flexed_z}",
    )

    thumb_joint = object_model.get_articulation("thumb_joint_0")
    thumb_rest_z = z_center("thumb_segment_0", "beam")
    with ctx.pose({thumb_joint: FLEX_ANGLE}):
        thumb_flexed_z = z_center("thumb_segment_0", "beam")
    ctx.check(
        "thumb flexion curls its side chain downward",
        thumb_rest_z is not None and thumb_flexed_z is not None and thumb_flexed_z < thumb_rest_z - 0.015,
        details=f"rest_z={thumb_rest_z}, flexed_z={thumb_flexed_z}",
    )

    return ctx.report()


object_model = build_object_model()
