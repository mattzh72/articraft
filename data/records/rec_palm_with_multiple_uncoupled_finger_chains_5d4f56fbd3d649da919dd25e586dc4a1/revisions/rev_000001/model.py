from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)


PALM_LENGTH = 0.080
PALM_WIDTH = 0.180
PALM_THICKNESS = 0.010
HINGE_BLOCK_SIZE = (0.012, 0.028, 0.016)
HINGE_X = PALM_LENGTH * 0.5
HINGE_Z = 0.013
FINGER_PITCH = 0.045
HINGE_Y_POSITIONS = tuple((-1.5 + index) * FINGER_PITCH for index in range(4))


def _add_finger_chain(part, *, metal, pad) -> None:
    hinge_roll = Origin(rpy=(pi * 0.5, 0.0, 0.0))

    part.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=hinge_roll.rpy),
        material=metal,
        name="base_hub",
    )
    part.visual(
        Box((0.048, 0.019, 0.008)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=metal,
        name="proximal_link",
    )
    part.visual(
        Cylinder(radius=0.005, length=0.019),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=hinge_roll.rpy),
        material=metal,
        name="mid_knuckle",
    )
    part.visual(
        Box((0.040, 0.016, 0.008)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=metal,
        name="middle_link",
    )
    part.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.105, 0.0, 0.0), rpy=hinge_roll.rpy),
        material=metal,
        name="distal_knuckle",
    )
    part.visual(
        Box((0.030, 0.014, 0.007)),
        origin=Origin(xyz=(0.1245, 0.0, 0.0)),
        material=metal,
        name="tip_link",
    )
    part.visual(
        Box((0.010, 0.014, 0.004)),
        origin=Origin(xyz=(0.144, 0.0, -0.0015)),
        material=pad,
        name="tip_pad",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.150, 0.025, 0.010)),
        mass=0.12,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="planar_robotic_palm")

    palm_plate = model.material("palm_plate", rgba=(0.17, 0.20, 0.24, 1.0))
    palm_brace = model.material("palm_brace", rgba=(0.12, 0.14, 0.17, 1.0))
    finger_metal = model.material("finger_metal", rgba=(0.74, 0.77, 0.80, 1.0))
    finger_pad = model.material("finger_pad", rgba=(0.12, 0.13, 0.15, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_LENGTH, PALM_WIDTH, PALM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PALM_THICKNESS * 0.5)),
        material=palm_plate,
        name="palm_plate",
    )
    palm.visual(
        Box((0.012, 0.150, 0.020)),
        origin=Origin(xyz=(-0.028, 0.0, 0.010)),
        material=palm_brace,
        name="rear_mount_bar",
    )
    palm.visual(
        Box((0.060, 0.010, 0.012)),
        origin=Origin(xyz=(-0.006, -0.028, -0.001)),
        material=palm_brace,
        name="left_stiffener",
    )
    palm.visual(
        Box((0.060, 0.010, 0.012)),
        origin=Origin(xyz=(-0.006, 0.028, -0.001)),
        material=palm_brace,
        name="right_stiffener",
    )
    for index, hinge_y in enumerate(HINGE_Y_POSITIONS, start=1):
        palm.visual(
            Box(HINGE_BLOCK_SIZE),
            origin=Origin(xyz=(0.034, hinge_y, HINGE_Z)),
            material=palm_brace,
            name=f"hinge_block_{index}",
        )
    palm.inertial = Inertial.from_geometry(
        Box((0.090, 0.190, 0.028)),
        mass=0.9,
        origin=Origin(xyz=(-0.006, 0.0, 0.010)),
    )

    for index, hinge_y in enumerate(HINGE_Y_POSITIONS, start=1):
        finger = model.part(f"finger_{index}")
        _add_finger_chain(finger, metal=finger_metal, pad=finger_pad)
        model.articulation(
            f"palm_to_finger_{index}",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=finger,
            origin=Origin(xyz=(HINGE_X, hinge_y, HINGE_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.6,
                lower=0.0,
                upper=1.15,
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
    fingers = [object_model.get_part(f"finger_{index}") for index in range(1, 5)]
    joints = [object_model.get_articulation(f"palm_to_finger_{index}") for index in range(1, 5)]

    joint_axes_ok = all(joint.axis == (0.0, -1.0, 0.0) for joint in joints)
    joint_limits_ok = all(
        joint.motion_limits is not None
        and joint.motion_limits.lower == 0.0
        and joint.motion_limits.upper == 1.15
        for joint in joints
    )
    ctx.check(
        "four parallel finger base joints are configured",
        joint_axes_ok and joint_limits_ok,
        details=str(
            [
                {
                    "name": joint.name,
                    "axis": joint.axis,
                    "limits": (
                        None
                        if joint.motion_limits is None
                        else (joint.motion_limits.lower, joint.motion_limits.upper)
                    ),
                }
                for joint in joints
            ]
        ),
    )

    y_positions = [ctx.part_world_position(finger)[1] for finger in fingers]
    finger_pitch_ok = all(abs((y_positions[index + 1] - y_positions[index]) - FINGER_PITCH) < 1e-9 for index in range(3))
    ctx.check(
        "finger hinges are evenly spaced in a straight row",
        finger_pitch_ok,
        details=f"finger_y_positions={y_positions}",
    )

    with ctx.pose({joint: 0.0 for joint in joints}):
        for index, finger in enumerate(fingers, start=1):
            ctx.expect_gap(
                finger,
                palm,
                axis="x",
                min_gap=0.0,
                max_gap=0.001,
                negative_elem=f"hinge_block_{index}",
                name=f"{finger.name} seats against hinge block {index}",
            )
            ctx.expect_overlap(
                finger,
                palm,
                axes="yz",
                elem_b=f"hinge_block_{index}",
                min_overlap=0.008,
                name=f"{finger.name} aligns with hinge block {index}",
            )

    for finger, joint in zip(fingers, joints, strict=True):
        rest_tip = ctx.part_element_world_aabb(finger, elem="tip_link")
        with ctx.pose({joint: 0.85}):
            raised_tip = ctx.part_element_world_aabb(finger, elem="tip_link")
        tip_lifts = (
            rest_tip is not None
            and raised_tip is not None
            and raised_tip[1][2] > rest_tip[1][2] + 0.050
        )
        ctx.check(
            f"{finger.name} lifts upward about its own base hinge",
            tip_lifts,
            details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
        )

    neighbor_rest = ctx.part_world_aabb(fingers[1])
    with ctx.pose({joints[0]: 0.85}):
        neighbor_moved = ctx.part_world_aabb(fingers[1])
    independent = (
        neighbor_rest is not None
        and neighbor_moved is not None
        and all(
            abs(neighbor_rest[corner][axis] - neighbor_moved[corner][axis]) < 1e-9
            for corner in range(2)
            for axis in range(3)
        )
    )
    ctx.check(
        "each finger base rotates independently",
        independent,
        details=f"neighbor_rest={neighbor_rest}, neighbor_moved={neighbor_moved}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
