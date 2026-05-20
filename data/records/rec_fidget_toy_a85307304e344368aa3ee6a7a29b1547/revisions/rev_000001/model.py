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


LINK_COUNT = 8
PITCH = 0.032
BODY_WIDTH = 0.024
BODY_LENGTH = 0.024
HINGE_DEPTH = 0.008
CENTER_KNUCKLE_LENGTH = 0.012
EAR_BLOCK_THICKNESS = 0.008
EAR_BLOCK_LENGTH = 0.012
EAR_OFFSET = 0.010
PIN_RADIUS = 0.0022
PIN_CAP_LENGTH = 0.004
SEGMENT_MASS = 0.035


def _segment_axis_type(index: int) -> str:
    return "x" if index % 2 == 1 else "y"


def _outgoing_axis_type(index: int) -> str:
    return "y" if _segment_axis_type(index) == "x" else "x"


def _segment_palette(index: int) -> str:
    return "plastic_blue" if index % 2 == 1 else "plastic_orange"


def _add_incoming_knuckle(part, *, axis: str, material, pin_material) -> None:
    if axis == "x":
        part.visual(
            Box((CENTER_KNUCKLE_LENGTH, EAR_BLOCK_THICKNESS, HINGE_DEPTH)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=material,
            name="incoming_knuckle",
        )
        for sign in (-1.0, 1.0):
            part.visual(
                Cylinder(radius=PIN_RADIUS, length=PIN_CAP_LENGTH),
                origin=Origin(
                    xyz=(sign * (CENTER_KNUCKLE_LENGTH * 0.5 + PIN_CAP_LENGTH * 0.5), 0.0, 0.0),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=pin_material,
                name=f"incoming_pin_cap_{'left' if sign < 0.0 else 'right'}",
            )
    else:
        part.visual(
            Box((EAR_BLOCK_THICKNESS, CENTER_KNUCKLE_LENGTH, HINGE_DEPTH)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=material,
            name="incoming_knuckle",
        )
        for sign in (-1.0, 1.0):
            part.visual(
                Cylinder(radius=PIN_RADIUS, length=PIN_CAP_LENGTH),
                origin=Origin(
                    xyz=(0.0, sign * (CENTER_KNUCKLE_LENGTH * 0.5 + PIN_CAP_LENGTH * 0.5), 0.0),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=pin_material,
                name=f"incoming_pin_cap_{'left' if sign < 0.0 else 'right'}",
            )


def _add_outgoing_ears(part, *, axis: str, z: float, material) -> None:
    bridge_height = max((z - HINGE_DEPTH * 0.5) - BODY_LENGTH, 0.0)
    bridge_center_z = BODY_LENGTH + bridge_height * 0.5
    if axis == "x":
        for sign in (-1.0, 1.0):
            part.visual(
                Box((EAR_BLOCK_LENGTH, EAR_BLOCK_THICKNESS, HINGE_DEPTH)),
                origin=Origin(xyz=(0.0, sign * EAR_OFFSET, z)),
                material=material,
                name=f"outgoing_ear_{'lower' if sign < 0.0 else 'upper'}",
            )
            if bridge_height > 0.0:
                part.visual(
                    Box((EAR_BLOCK_LENGTH * 0.72, EAR_BLOCK_THICKNESS * 0.72, bridge_height)),
                    origin=Origin(xyz=(0.0, sign * EAR_OFFSET, bridge_center_z)),
                    material=material,
                    name=f"outgoing_web_{'lower' if sign < 0.0 else 'upper'}",
                )
    else:
        for sign in (-1.0, 1.0):
            part.visual(
                Box((EAR_BLOCK_THICKNESS, EAR_BLOCK_LENGTH, HINGE_DEPTH)),
                origin=Origin(xyz=(sign * EAR_OFFSET, 0.0, z)),
                material=material,
                name=f"outgoing_ear_{'left' if sign < 0.0 else 'right'}",
            )
            if bridge_height > 0.0:
                part.visual(
                    Box((EAR_BLOCK_THICKNESS * 0.72, EAR_BLOCK_LENGTH * 0.72, bridge_height)),
                    origin=Origin(xyz=(sign * EAR_OFFSET, 0.0, bridge_center_z)),
                    material=material,
                    name=f"outgoing_web_{'left' if sign < 0.0 else 'right'}",
                )


def _build_segment(model: ArticulatedObject, index: int, pin_material) -> object:
    part = model.part(f"segment_{index}")
    body_material = _segment_palette(index)

    part.visual(
        Box((BODY_WIDTH, BODY_WIDTH, BODY_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, BODY_LENGTH * 0.5)),
        material=body_material,
        name="body_shell",
    )
    part.visual(
        Box((BODY_WIDTH * 0.82, BODY_WIDTH * 0.82, BODY_LENGTH * 0.52)),
        origin=Origin(xyz=(0.0, 0.0, BODY_LENGTH * 0.50)),
        material="plastic_panel",
        name="body_inset",
    )

    _add_incoming_knuckle(
        part,
        axis=_segment_axis_type(index),
        material=body_material,
        pin_material=pin_material,
    )
    _add_outgoing_ears(
        part,
        axis=_outgoing_axis_type(index),
        z=PITCH,
        material=body_material,
    )

    part.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_WIDTH, PITCH + HINGE_DEPTH)),
        mass=SEGMENT_MASS,
        origin=Origin(xyz=(0.0, 0.0, (PITCH + HINGE_DEPTH) * 0.5 - HINGE_DEPTH * 0.5)),
    )
    return part


def _joint_axis(index: int) -> tuple[float, float, float]:
    return (1.0, 0.0, 0.0) if _outgoing_axis_type(index) == "x" else (0.0, 1.0, 0.0)


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snake_fidget_chain")

    model.material("plastic_blue", rgba=(0.22, 0.44, 0.78, 1.0))
    model.material("plastic_orange", rgba=(0.92, 0.46, 0.16, 1.0))
    model.material("plastic_panel", rgba=(0.16, 0.18, 0.20, 1.0))
    pin_material = model.material("pin_metal", rgba=(0.68, 0.70, 0.74, 1.0))

    segments = [_build_segment(model, index, pin_material) for index in range(1, LINK_COUNT + 1)]

    for index in range(1, LINK_COUNT):
        model.articulation(
            f"segment_{index}_to_segment_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=segments[index - 1],
            child=segments[index],
            origin=Origin(xyz=(0.0, 0.0, PITCH)),
            axis=_joint_axis(index),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=6.0,
                lower=-math.pi / 2.0,
                upper=math.pi / 2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    segments = [object_model.get_part(f"segment_{index}") for index in range(1, LINK_COUNT + 1)]
    joints = [
        object_model.get_articulation(f"segment_{index}_to_segment_{index + 1}")
        for index in range(1, LINK_COUNT)
    ]

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

    ctx.check(
        "eight_segments_present",
        len(segments) == LINK_COUNT and len(joints) == LINK_COUNT - 1,
        details=f"Expected {LINK_COUNT} segments and {LINK_COUNT - 1} joints.",
    )

    expected_axes = [
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
    ]
    ctx.check(
        "alternating_joint_axes",
        all(tuple(joint.axis) == axis for joint, axis in zip(joints, expected_axes)),
        details="Adjacent hinge pins should alternate between Y and X axes.",
    )

    for index, (parent, child) in enumerate(zip(segments, segments[1:]), start=1):
        ctx.expect_contact(parent, child, name=f"segment_{index}_contacts_segment_{index + 1}")
        ctx.expect_overlap(
            parent,
            child,
            axes="xy",
            min_overlap=BODY_WIDTH * 0.45,
            name=f"segment_{index}_aligned_with_segment_{index + 1}",
        )

    first_rest_aabb = ctx.part_element_world_aabb(segments[1], elem="body_shell")
    second_rest_aabb = ctx.part_element_world_aabb(segments[2], elem="body_shell")
    if first_rest_aabb is None or second_rest_aabb is None:
        ctx.fail("segment_body_aabbs_available", "Expected named body visuals on downstream segments.")
    else:
        first_rest_center = _aabb_center(first_rest_aabb)
        second_rest_center = _aabb_center(second_rest_aabb)

        with ctx.pose({joints[0]: math.radians(55)}):
            first_folded_aabb = ctx.part_element_world_aabb(segments[1], elem="body_shell")
        with ctx.pose({joints[1]: math.radians(-55)}):
            second_folded_aabb = ctx.part_element_world_aabb(segments[2], elem="body_shell")

        if first_folded_aabb is None or second_folded_aabb is None:
            ctx.fail("posed_segment_body_aabbs_available", "Expected posed body visual AABBs to be measurable.")
        else:
            first_motion = math.dist(first_rest_center, _aabb_center(first_folded_aabb))
            second_motion = math.dist(second_rest_center, _aabb_center(second_folded_aabb))
            ctx.check(
                "joint_1_moves_segment_2_body",
                first_motion > 0.01,
                details=f"Segment 2 body only moved {first_motion:.4f} m under joint_1 actuation.",
            )
            ctx.check(
                "joint_2_moves_segment_3_body",
                second_motion > 0.01,
                details=f"Segment 3 body only moved {second_motion:.4f} m under joint_2 actuation.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
