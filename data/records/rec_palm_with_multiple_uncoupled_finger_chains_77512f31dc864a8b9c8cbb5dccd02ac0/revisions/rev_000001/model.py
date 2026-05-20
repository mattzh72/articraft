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


HALF_PI = math.pi / 2.0

PALM_WIDTH = 0.090
PALM_DEPTH = 0.075
PALM_THICKNESS = 0.006

JOINT_RADIUS = 0.005
CENTER_BARREL_WIDTH = 0.008
SIDE_BARREL_WIDTH = 0.007
DIGIT_WIDTH = CENTER_BARREL_WIDTH + 2.0 * SIDE_BARREL_WIDTH
SIDE_BARREL_OFFSET = 0.5 * (CENTER_BARREL_WIDTH + SIDE_BARREL_WIDTH)

DIGIT_THICKNESS = 0.010
BODY_THICKNESS = 0.0085
BODY_WIDTH = 0.016

ROOT_PIVOT_Z = PALM_THICKNESS + JOINT_RADIUS
FORWARD_ROOT_Y = (PALM_DEPTH * 0.5) + 0.0015
THUMB_ROOT_X = -(PALM_WIDTH * 0.5) - 0.004
THUMB_ROOT_Y = -0.006

FORWARD_DIGIT_SPECS = (
    ("finger_left", -0.026, (0.033, 0.026, 0.022)),
    ("finger_center", 0.000, (0.036, 0.029, 0.024)),
    ("finger_right", 0.026, (0.033, 0.026, 0.022)),
)

THUMB_DIGIT_SPEC = ("thumb", THUMB_ROOT_X, THUMB_ROOT_Y, (0.028, 0.022, 0.018))


def _add_x_barrel(part, *, name: str, center: tuple[float, float, float], material) -> None:
    part.visual(
        Cylinder(radius=JOINT_RADIUS, length=CENTER_BARREL_WIDTH),
        origin=Origin(xyz=center, rpy=(0.0, HALF_PI, 0.0)),
        material=material,
        name=name,
    )


def _add_x_side_barrel(
    part,
    *,
    name: str,
    center: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Cylinder(radius=JOINT_RADIUS, length=SIDE_BARREL_WIDTH),
        origin=Origin(xyz=center, rpy=(0.0, HALF_PI, 0.0)),
        material=material,
        name=name,
    )


def _add_y_barrel(part, *, name: str, center: tuple[float, float, float], material) -> None:
    part.visual(
        Cylinder(radius=JOINT_RADIUS, length=CENTER_BARREL_WIDTH),
        origin=Origin(xyz=center, rpy=(HALF_PI, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_y_side_barrel(
    part,
    *,
    name: str,
    center: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Cylinder(radius=JOINT_RADIUS, length=SIDE_BARREL_WIDTH),
        origin=Origin(xyz=center, rpy=(HALF_PI, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _populate_forward_intermediate_link(part, *, span: float, body_material, pin_material) -> None:
    prefix = part.name
    neck_length = 0.012
    main_start = neck_length
    main_end = span - 0.006
    main_length = max(0.010, main_end - main_start)
    main_center_y = 0.5 * (main_start + main_end)

    _add_x_barrel(
        part,
        name=f"{prefix}_prox_barrel",
        center=(0.0, 0.0, 0.0),
        material=pin_material,
    )
    part.visual(
        Box((CENTER_BARREL_WIDTH, neck_length, DIGIT_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.5 * neck_length, 0.0)),
        material=body_material,
        name=f"{prefix}_neck",
    )
    part.visual(
        Box((BODY_WIDTH, main_length, BODY_THICKNESS)),
        origin=Origin(xyz=(0.0, main_center_y, 0.0)),
        material=body_material,
        name=f"{prefix}_beam",
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        side_x = side_sign * SIDE_BARREL_OFFSET
        part.visual(
            Box((SIDE_BARREL_WIDTH, 0.010, BODY_THICKNESS)),
            origin=Origin(xyz=(side_x, span - 0.005, 0.0)),
            material=body_material,
            name=f"{prefix}_{side_name}_yoke",
        )
        _add_x_side_barrel(
            part,
            name=f"{prefix}_{side_name}_distal_barrel",
            center=(side_x, span, 0.0),
            material=pin_material,
        )


def _populate_forward_terminal_link(part, *, reach: float, body_material, pin_material) -> None:
    prefix = part.name
    neck_length = 0.012

    _add_x_barrel(
        part,
        name=f"{prefix}_prox_barrel",
        center=(0.0, 0.0, 0.0),
        material=pin_material,
    )
    part.visual(
        Box((CENTER_BARREL_WIDTH, neck_length, DIGIT_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.5 * neck_length, 0.0)),
        material=body_material,
        name=f"{prefix}_neck",
    )
    part.visual(
        Box((BODY_WIDTH, reach, BODY_THICKNESS)),
        origin=Origin(xyz=(0.0, neck_length + 0.5 * reach, 0.0)),
        material=body_material,
        name=f"{prefix}_beam",
    )
    part.visual(
        Box((0.014, 0.010, DIGIT_THICKNESS)),
        origin=Origin(xyz=(0.0, neck_length + reach + 0.005, 0.0)),
        material=body_material,
        name=f"{prefix}_tip_block",
    )
    part.visual(
        Box((0.010, 0.007, 0.0065)),
        origin=Origin(xyz=(0.0, neck_length + reach + 0.0125, 0.0)),
        material=body_material,
        name=f"{prefix}_tip_pad",
    )


def _populate_thumb_intermediate_link(part, *, span: float, body_material, pin_material) -> None:
    prefix = part.name
    neck_length = 0.012
    main_start = -neck_length
    main_end = -span + 0.006
    main_length = max(0.010, abs(main_end - main_start))
    main_center_x = 0.5 * (main_start + main_end)

    _add_y_barrel(
        part,
        name=f"{prefix}_prox_barrel",
        center=(0.0, 0.0, 0.0),
        material=pin_material,
    )
    part.visual(
        Box((neck_length, CENTER_BARREL_WIDTH, DIGIT_THICKNESS)),
        origin=Origin(xyz=(-0.5 * neck_length, 0.0, 0.0)),
        material=body_material,
        name=f"{prefix}_neck",
    )
    part.visual(
        Box((main_length, BODY_WIDTH, BODY_THICKNESS)),
        origin=Origin(xyz=(main_center_x, 0.0, 0.0)),
        material=body_material,
        name=f"{prefix}_beam",
    )

    for side_name, side_sign in (("lower", -1.0), ("upper", 1.0)):
        side_y = side_sign * SIDE_BARREL_OFFSET
        part.visual(
            Box((0.010, SIDE_BARREL_WIDTH, BODY_THICKNESS)),
            origin=Origin(xyz=(-span + 0.005, side_y, 0.0)),
            material=body_material,
            name=f"{prefix}_{side_name}_yoke",
        )
        _add_y_side_barrel(
            part,
            name=f"{prefix}_{side_name}_distal_barrel",
            center=(-span, side_y, 0.0),
            material=pin_material,
        )


def _populate_thumb_terminal_link(part, *, reach: float, body_material, pin_material) -> None:
    prefix = part.name
    neck_length = 0.012

    _add_y_barrel(
        part,
        name=f"{prefix}_prox_barrel",
        center=(0.0, 0.0, 0.0),
        material=pin_material,
    )
    part.visual(
        Box((neck_length, CENTER_BARREL_WIDTH, DIGIT_THICKNESS)),
        origin=Origin(xyz=(-0.5 * neck_length, 0.0, 0.0)),
        material=body_material,
        name=f"{prefix}_neck",
    )
    part.visual(
        Box((reach, BODY_WIDTH, BODY_THICKNESS)),
        origin=Origin(xyz=(-(neck_length + 0.5 * reach), 0.0, 0.0)),
        material=body_material,
        name=f"{prefix}_beam",
    )
    part.visual(
        Box((0.010, 0.014, DIGIT_THICKNESS)),
        origin=Origin(xyz=(-(neck_length + reach + 0.005), 0.0, 0.0)),
        material=body_material,
        name=f"{prefix}_tip_block",
    )
    part.visual(
        Box((0.007, 0.010, 0.0065)),
        origin=Origin(xyz=(-(neck_length + reach + 0.0115), 0.0, 0.0)),
        material=body_material,
        name=f"{prefix}_tip_pad",
    )


def _forward_link_inertial(span: float, mass: float) -> Inertial:
    return Inertial.from_geometry(
        Box((DIGIT_WIDTH, span + 0.012, DIGIT_THICKNESS)),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.5 * span, 0.0)),
    )


def _forward_terminal_inertial(reach: float, mass: float) -> Inertial:
    return Inertial.from_geometry(
        Box((BODY_WIDTH, reach + 0.026, DIGIT_THICKNESS)),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.5 * (reach + 0.012), 0.0)),
    )


def _thumb_link_inertial(span: float, mass: float) -> Inertial:
    return Inertial.from_geometry(
        Box((span + 0.012, DIGIT_WIDTH, DIGIT_THICKNESS)),
        mass=mass,
        origin=Origin(xyz=(-0.5 * span, 0.0, 0.0)),
    )


def _thumb_terminal_inertial(reach: float, mass: float) -> Inertial:
    return Inertial.from_geometry(
        Box((reach + 0.026, BODY_WIDTH, DIGIT_THICKNESS)),
        mass=mass,
        origin=Origin(xyz=(-0.5 * (reach + 0.012), 0.0, 0.0)),
    )


def _axis_matches(axis: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-9) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(axis, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_gripper_hand")

    palm_plate_material = model.material("palm_plate", rgba=(0.62, 0.66, 0.70, 1.0))
    link_material = model.material("link_metal", rgba=(0.24, 0.27, 0.31, 1.0))
    pin_material = model.material("pin_steel", rgba=(0.84, 0.86, 0.88, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_WIDTH, PALM_DEPTH, PALM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PALM_THICKNESS * 0.5)),
        material=palm_plate_material,
        name="palm_plate",
    )
    palm.visual(
        Box((0.064, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.020, PALM_THICKNESS + 0.004)),
        material=palm_plate_material,
        name="front_rib",
    )
    palm.visual(
        Box((0.040, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, -0.024, PALM_THICKNESS + 0.003)),
        material=palm_plate_material,
        name="rear_stiffener",
    )

    for digit_name, root_x, _ in FORWARD_DIGIT_SPECS:
        for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
            mount_x = root_x + (side_sign * SIDE_BARREL_OFFSET)
            palm.visual(
                Box((SIDE_BARREL_WIDTH, 0.018, ROOT_PIVOT_Z)),
                origin=Origin(xyz=(mount_x, FORWARD_ROOT_Y - 0.008, ROOT_PIVOT_Z * 0.5)),
                material=palm_plate_material,
                name=f"{digit_name}_{side_name}_mount_leg",
            )
            _add_x_side_barrel(
                palm,
                name=f"{digit_name}_{side_name}_mount_barrel",
                center=(mount_x, FORWARD_ROOT_Y, ROOT_PIVOT_Z),
                material=pin_material,
            )

    for side_name, side_sign in (("lower", -1.0), ("upper", 1.0)):
        mount_y = THUMB_ROOT_Y + (side_sign * SIDE_BARREL_OFFSET)
        palm.visual(
            Box((0.018, SIDE_BARREL_WIDTH, ROOT_PIVOT_Z)),
            origin=Origin(xyz=(THUMB_ROOT_X + 0.008, mount_y, ROOT_PIVOT_Z * 0.5)),
            material=palm_plate_material,
            name=f"thumb_{side_name}_mount_leg",
        )
        _add_y_side_barrel(
            palm,
            name=f"thumb_{side_name}_mount_barrel",
            center=(THUMB_ROOT_X, mount_y, ROOT_PIVOT_Z),
            material=pin_material,
        )

    palm.inertial = Inertial.from_geometry(
        Box((0.104, 0.082, 0.026)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    for digit_name, root_x, spans in FORWARD_DIGIT_SPECS:
        proximal = model.part(f"{digit_name}_proximal")
        middle = model.part(f"{digit_name}_middle")
        distal = model.part(f"{digit_name}_distal")

        _populate_forward_intermediate_link(
            proximal,
            span=spans[0],
            body_material=link_material,
            pin_material=pin_material,
        )
        _populate_forward_intermediate_link(
            middle,
            span=spans[1],
            body_material=link_material,
            pin_material=pin_material,
        )
        _populate_forward_terminal_link(
            distal,
            reach=spans[2],
            body_material=link_material,
            pin_material=pin_material,
        )

        proximal.inertial = _forward_link_inertial(spans[0], mass=0.055)
        middle.inertial = _forward_link_inertial(spans[1], mass=0.042)
        distal.inertial = _forward_terminal_inertial(spans[2], mass=0.030)

        root_joint = model.articulation(
            f"palm_to_{digit_name}_proximal",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(root_x, FORWARD_ROOT_Y, ROOT_PIVOT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=0.0,
                upper=1.10,
            ),
        )
        _ = root_joint

        model.articulation(
            f"{digit_name}_proximal_to_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(0.0, spans[0], 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=2.2,
                lower=0.0,
                upper=1.25,
            ),
        )
        model.articulation(
            f"{digit_name}_middle_to_distal",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(0.0, spans[1], 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.4,
                lower=0.0,
                upper=1.15,
            ),
        )

    thumb_name, thumb_root_x, thumb_root_y, thumb_spans = THUMB_DIGIT_SPEC
    thumb_proximal = model.part(f"{thumb_name}_proximal")
    thumb_middle = model.part(f"{thumb_name}_middle")
    thumb_distal = model.part(f"{thumb_name}_distal")

    _populate_thumb_intermediate_link(
        thumb_proximal,
        span=thumb_spans[0],
        body_material=link_material,
        pin_material=pin_material,
    )
    _populate_thumb_intermediate_link(
        thumb_middle,
        span=thumb_spans[1],
        body_material=link_material,
        pin_material=pin_material,
    )
    _populate_thumb_terminal_link(
        thumb_distal,
        reach=thumb_spans[2],
        body_material=link_material,
        pin_material=pin_material,
    )

    thumb_proximal.inertial = _thumb_link_inertial(thumb_spans[0], mass=0.050)
    thumb_middle.inertial = _thumb_link_inertial(thumb_spans[1], mass=0.036)
    thumb_distal.inertial = _thumb_terminal_inertial(thumb_spans[2], mass=0.026)

    model.articulation(
        "palm_to_thumb_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_proximal,
        origin=Origin(xyz=(thumb_root_x, thumb_root_y, ROOT_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=7.0,
            velocity=2.0,
            lower=0.0,
            upper=1.00,
        ),
    )
    model.articulation(
        "thumb_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=thumb_proximal,
        child=thumb_middle,
        origin=Origin(xyz=(-thumb_spans[0], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.2,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "thumb_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=thumb_middle,
        child=thumb_distal,
        origin=Origin(xyz=(-thumb_spans[1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.4,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")

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

    forward_root_joints = []
    forward_inner_joints = []

    for digit_name, _, _ in FORWARD_DIGIT_SPECS:
        proximal = object_model.get_part(f"{digit_name}_proximal")
        middle = object_model.get_part(f"{digit_name}_middle")
        distal = object_model.get_part(f"{digit_name}_distal")

        root_joint = object_model.get_articulation(f"palm_to_{digit_name}_proximal")
        middle_joint = object_model.get_articulation(f"{digit_name}_proximal_to_middle")
        distal_joint = object_model.get_articulation(f"{digit_name}_middle_to_distal")

        forward_root_joints.append(root_joint)
        forward_inner_joints.extend((middle_joint, distal_joint))

        ctx.expect_contact(proximal, palm, contact_tol=1e-5)
        ctx.expect_contact(middle, proximal, contact_tol=1e-5)
        ctx.expect_contact(distal, middle, contact_tol=1e-5)

        rest_position = ctx.part_world_position(distal)
        with ctx.pose({root_joint: 0.72, middle_joint: 0.92, distal_joint: 0.65}):
            curled_position = ctx.part_world_position(distal)
            ctx.expect_contact(proximal, palm, contact_tol=1e-5)
            ctx.expect_contact(middle, proximal, contact_tol=1e-5)
            ctx.expect_contact(distal, middle, contact_tol=1e-5)
            ctx.check(
                f"{digit_name}_curls_upward",
                rest_position is not None
                and curled_position is not None
                and curled_position[2] > rest_position[2] + 0.018,
                details=f"rest={rest_position}, curled={curled_position}",
            )

    thumb_proximal = object_model.get_part("thumb_proximal")
    thumb_middle = object_model.get_part("thumb_middle")
    thumb_distal = object_model.get_part("thumb_distal")
    thumb_root_joint = object_model.get_articulation("palm_to_thumb_proximal")
    thumb_middle_joint = object_model.get_articulation("thumb_proximal_to_middle")
    thumb_distal_joint = object_model.get_articulation("thumb_middle_to_distal")

    ctx.expect_contact(thumb_proximal, palm, contact_tol=1e-5)
    ctx.expect_contact(thumb_middle, thumb_proximal, contact_tol=1e-5)
    ctx.expect_contact(thumb_distal, thumb_middle, contact_tol=1e-5)

    thumb_rest = ctx.part_world_position(thumb_distal)
    with ctx.pose(
        {
            thumb_root_joint: 0.68,
            thumb_middle_joint: 0.88,
            thumb_distal_joint: 0.58,
        }
    ):
        thumb_curled = ctx.part_world_position(thumb_distal)
        ctx.expect_contact(thumb_proximal, palm, contact_tol=1e-5)
        ctx.expect_contact(thumb_middle, thumb_proximal, contact_tol=1e-5)
        ctx.expect_contact(thumb_distal, thumb_middle, contact_tol=1e-5)
        ctx.check(
            "thumb_curls_upward",
            thumb_rest is not None
            and thumb_curled is not None
            and thumb_curled[2] > thumb_rest[2] + 0.014,
            details=f"rest={thumb_rest}, curled={thumb_curled}",
        )

    right_distal_rest = ctx.part_world_position(object_model.get_part("finger_right_distal"))
    with ctx.pose({"palm_to_finger_left_proximal": 0.85}):
        right_distal_after = ctx.part_world_position(object_model.get_part("finger_right_distal"))
        uncoupled = (
            right_distal_rest is not None
            and right_distal_after is not None
            and max(abs(a - b) for a, b in zip(right_distal_rest, right_distal_after)) <= 1e-9
        )
        ctx.check(
            "digits_move_independently",
            uncoupled,
            details=f"right_distal_rest={right_distal_rest}, right_distal_after={right_distal_after}",
        )

    ctx.check(
        "forward_hinges_use_x_axis",
        all(_axis_matches(joint.axis, (1.0, 0.0, 0.0)) for joint in forward_root_joints + forward_inner_joints),
        details=str([joint.axis for joint in forward_root_joints + forward_inner_joints]),
    )
    ctx.check(
        "thumb_hinges_use_y_axis",
        all(
            _axis_matches(joint.axis, (0.0, 1.0, 0.0))
            for joint in (thumb_root_joint, thumb_middle_joint, thumb_distal_joint)
        ),
        details=str([thumb_root_joint.axis, thumb_middle_joint.axis, thumb_distal_joint.axis]),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
