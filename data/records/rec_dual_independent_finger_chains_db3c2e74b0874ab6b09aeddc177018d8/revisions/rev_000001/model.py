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
    Sphere,
    TestContext,
    TestReport,
)


PALM_MATERIAL = Material("powder_coated_dark_grey", rgba=(0.12, 0.13, 0.14, 1.0))
LINK_MATERIAL = Material("anodized_blue_grey", rgba=(0.20, 0.32, 0.46, 1.0))
END_LINK_MATERIAL = Material("anodized_light_blue", rgba=(0.24, 0.42, 0.64, 1.0))
PIN_MATERIAL = Material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
PAD_MATERIAL = Material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

FINGER_Y = (-0.032, 0.032)
KNUCKLE_X = 0.020
KNUCKLE_Z = 0.105
PROXIMAL_LENGTH = 0.120
MIDDLE_LENGTH = 0.095
DISTAL_LENGTH = 0.072


def _cylinder_along_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_forked_phalanx(
    part,
    *,
    length: float,
    has_end_fork: bool,
    material: Material,
    prefix: str,
) -> None:
    """Add one link whose local frame sits on the proximal hinge axis.

    The link has a central start barrel that fits between the parent fork cheeks.
    Non-distal links also carry a pair of side fork arms at their distal end so
    the following link's start barrel has visible clearance instead of overlap.
    """

    barrel_radius = 0.014
    barrel_width = 0.018
    body_width = 0.022
    body_height = 0.022
    body_start = 0.010

    barrel_geom, barrel_origin = _cylinder_along_y(barrel_radius, barrel_width)
    part.visual(
        barrel_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=barrel_origin.rpy),
        material=PIN_MATERIAL,
        name=f"{prefix}_root_barrel",
    )

    if has_end_fork:
        body_end = length - 0.036
        fork_center_x = length - 0.014
    else:
        body_end = length - 0.020
        fork_center_x = None

    part.visual(
        Box((body_end - body_start, body_width, body_height)),
        origin=Origin(xyz=((body_start + body_end) / 2.0, 0.0, 0.0)),
        material=material,
        name=f"{prefix}_link_beam",
    )

    # A raised back strap makes each link read as a machined rigid member rather
    # than a featureless cuboid, while staying connected to the main beam.
    part.visual(
        Box((body_end - body_start - 0.006, body_width * 0.72, 0.006)),
        origin=Origin(xyz=((body_start + body_end) / 2.0, 0.0, body_height / 2.0 + 0.003)),
        material=material,
        name=f"{prefix}_top_rib",
    )

    if has_end_fork and fork_center_x is not None:
        arm_length = 0.044
        arm_width = 0.007
        arm_height = 0.024
        # The side fork cheeks just kiss the central barrel along Y.  That
        # contact gives the QC-visible support path without hiding broad
        # interpenetration between adjacent links.
        arm_y = 0.0125
        for sign, side in ((-1.0, "lower"), (1.0, "upper")):
            part.visual(
                Box((arm_length, arm_width, arm_height)),
                origin=Origin(xyz=(fork_center_x, sign * arm_y, 0.0)),
                material=material,
                name=f"{prefix}_{side}_fork_arm",
            )
            lug_geom, lug_origin = _cylinder_along_y(0.0135, arm_width)
            part.visual(
                lug_geom,
                origin=Origin(xyz=(length, sign * arm_y, 0.0), rpy=lug_origin.rpy),
                material=PIN_MATERIAL,
                name=f"{prefix}_{side}_fork_lug",
            )
    else:
        part.visual(
            Sphere(radius=0.017),
            origin=Origin(xyz=(length - 0.007, 0.0, 0.0)),
            material=PAD_MATERIAL,
            name=f"{prefix}_rounded_tip",
        )
        part.visual(
            Box((0.018, 0.020, 0.006)),
            origin=Origin(xyz=(length - 0.011, 0.0, -0.014)),
            material=PAD_MATERIAL,
            name=f"{prefix}_contact_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="forked_independent_palm")

    palm = model.part("palm")
    palm.visual(
        Box((0.160, 0.170, 0.034)),
        origin=Origin(xyz=(-0.072, 0.0, 0.017)),
        material=PALM_MATERIAL,
        name="grounded_palm_beam",
    )
    palm.visual(
        Box((0.060, 0.170, 0.090)),
        origin=Origin(xyz=(-0.028, 0.0, 0.074)),
        material=PALM_MATERIAL,
        name="upright_knuckle_web",
    )
    palm.visual(
        Box((0.095, 0.135, 0.018)),
        origin=Origin(xyz=(-0.006, 0.0, KNUCKLE_Z - 0.032)),
        material=PALM_MATERIAL,
        name="front_support_shelf",
    )

    for finger_index, finger_y in enumerate(FINGER_Y):
        # Root yoke: two cheeks leave a clear central bay for the moving
        # proximal barrel, so the articulated part is visibly mounted but not
        # intersecting the fixed palm.
        for sign, side in ((-1.0, "lower"), (1.0, "upper")):
            palm.visual(
                Box((0.068, 0.007, 0.042)),
                origin=Origin(xyz=(KNUCKLE_X - 0.020, finger_y + sign * 0.0125, KNUCKLE_Z)),
                material=PALM_MATERIAL,
                name=f"finger_{finger_index}_{side}_knuckle_cheek",
            )
            cheek_geom, cheek_origin = _cylinder_along_y(0.0145, 0.007)
            palm.visual(
                cheek_geom,
                origin=Origin(
                    xyz=(KNUCKLE_X, finger_y + sign * 0.0125, KNUCKLE_Z),
                    rpy=cheek_origin.rpy,
                ),
                material=PIN_MATERIAL,
                name=f"finger_{finger_index}_{side}_knuckle_boss",
            )

        proximal = model.part(f"proximal_{finger_index}")
        middle = model.part(f"middle_{finger_index}")
        distal = model.part(f"distal_{finger_index}")

        _add_forked_phalanx(
            proximal,
            length=PROXIMAL_LENGTH,
            has_end_fork=True,
            material=LINK_MATERIAL,
            prefix=f"proximal_{finger_index}",
        )
        _add_forked_phalanx(
            middle,
            length=MIDDLE_LENGTH,
            has_end_fork=True,
            material=LINK_MATERIAL,
            prefix=f"middle_{finger_index}",
        )
        _add_forked_phalanx(
            distal,
            length=DISTAL_LENGTH,
            has_end_fork=False,
            material=END_LINK_MATERIAL,
            prefix=f"distal_{finger_index}",
        )

        model.articulation(
            f"knuckle_{finger_index}",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(KNUCKLE_X, finger_y, KNUCKLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=0.0, upper=1.15),
        )
        model.articulation(
            f"middle_joint_{finger_index}",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=3.2, lower=0.0, upper=1.25),
        )
        model.articulation(
            f"distal_joint_{finger_index}",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=7.0, velocity=3.5, lower=0.0, upper=1.10),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    expected_parts = {"palm"}
    expected_joints = set()
    for finger_index in (0, 1):
        expected_parts.update(
            {
                f"proximal_{finger_index}",
                f"middle_{finger_index}",
                f"distal_{finger_index}",
            }
        )
        expected_joints.update(
            {
                f"knuckle_{finger_index}",
                f"middle_joint_{finger_index}",
                f"distal_joint_{finger_index}",
            }
        )

    actual_parts = {part.name for part in object_model.parts}
    actual_joints = {joint.name for joint in object_model.articulations}
    ctx.check("one palm plus six finger links", actual_parts == expected_parts, details=str(actual_parts))
    ctx.check("six independent revolute joints", actual_joints == expected_joints, details=str(actual_joints))

    for finger_index in (0, 1):
        chain = (
            (f"knuckle_{finger_index}", "palm", f"proximal_{finger_index}"),
            (f"middle_joint_{finger_index}", f"proximal_{finger_index}", f"middle_{finger_index}"),
            (f"distal_joint_{finger_index}", f"middle_{finger_index}", f"distal_{finger_index}"),
        )
        for joint_name, parent_name, child_name in chain:
            joint = object_model.get_articulation(joint_name)
            ctx.check(
                f"{joint_name} is uncoupled revolute",
                joint.articulation_type == ArticulationType.REVOLUTE and joint.mimic is None,
                details=f"type={joint.articulation_type}, mimic={joint.mimic}",
            )
            ctx.check(
                f"{joint_name} stays within its finger chain",
                joint.parent == parent_name and joint.child == child_name,
                details=f"parent={joint.parent}, child={joint.child}",
            )
            ctx.check(
                f"{joint_name} has flexion limits",
                joint.motion_limits is not None
                and joint.motion_limits.lower == 0.0
                and joint.motion_limits.upper is not None
                and joint.motion_limits.upper >= 1.0,
                details=str(joint.motion_limits),
            )

        ctx.expect_contact(
            "palm",
            f"proximal_{finger_index}",
            contact_tol=0.0005,
            elem_a=f"finger_{finger_index}_upper_knuckle_boss",
            elem_b=f"proximal_{finger_index}_root_barrel",
            name=f"finger {finger_index} root barrel is seated in palm yoke",
        )
        ctx.expect_contact(
            f"proximal_{finger_index}",
            f"middle_{finger_index}",
            contact_tol=0.0005,
            elem_a=f"proximal_{finger_index}_upper_fork_lug",
            elem_b=f"middle_{finger_index}_root_barrel",
            name=f"finger {finger_index} middle hinge is captured by fork",
        )
        ctx.expect_contact(
            f"middle_{finger_index}",
            f"distal_{finger_index}",
            contact_tol=0.0005,
            elem_a=f"middle_{finger_index}_upper_fork_lug",
            elem_b=f"distal_{finger_index}_root_barrel",
            name=f"finger {finger_index} distal hinge is captured by fork",
        )

    return ctx.report()


object_model = build_object_model()
