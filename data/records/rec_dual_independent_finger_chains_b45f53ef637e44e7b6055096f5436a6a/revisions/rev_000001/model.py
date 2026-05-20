from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PALM_LENGTH = 0.050
PALM_WIDTH = 0.046
PALM_HEIGHT = 0.028
FINGER_Y_OFFSET = 0.0125

BODY_WIDTH = 0.0074
BARREL_LENGTH = 0.0054
KNUCKLE_RADIUS = 0.0046
PIN_SHAFT_RADIUS = 0.0020
PIN_HEAD_RADIUS = 0.0016
PIN_HEAD_THICKNESS = 0.0009
ROOT_START_X = KNUCKLE_RADIUS
PALM_SUPPORT_THICKNESS = BODY_WIDTH * 0.76

PALM_ROOT_AXIS_X = PALM_LENGTH / 2.0 + KNUCKLE_RADIUS + 0.0015

PROXIMAL_LENGTH = 0.042
MIDDLE_LENGTH = 0.032
DISTAL_LENGTH = 0.028

PROXIMAL_ROOT_HEIGHT = 0.0110
PROXIMAL_TIP_HEIGHT = 0.0092
MIDDLE_ROOT_HEIGHT = 0.0092
MIDDLE_TIP_HEIGHT = 0.0075
DISTAL_ROOT_HEIGHT = 0.0078
DISTAL_TIP_HEIGHT = 0.0058

TIP_PAD_LENGTH = 0.010
TIP_PAD_THICKNESS = 0.0032


def _fuse_all(*shapes: cq.Workplane) -> cq.Workplane:
    fused = shapes[0]
    for shape in shapes[1:]:
        fused = fused.union(shape)
    return fused


def _beam(length: float, root_height: float, tip_height: float) -> cq.Workplane:
    taper_start = length * 0.58
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.0, -root_height / 2.0)
        .lineTo(taper_start, -tip_height / 2.0)
        .lineTo(length, -tip_height / 2.0)
        .lineTo(length, tip_height / 2.0)
        .lineTo(taper_start, tip_height / 2.0)
        .lineTo(0.0, root_height / 2.0)
        .close()
    )
    return profile.extrude(BODY_WIDTH / 2.0, both=True)


def _pin_head(axis_x: float, y_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(PIN_HEAD_RADIUS)
        .extrude(PIN_HEAD_THICKNESS / 2.0, both=True)
        .translate((axis_x, y_center, 0.0))
    )


def _knuckle_barrel(axis_x: float, barrel_radius: float) -> cq.Workplane:
    head_offset_y = BARREL_LENGTH / 2.0 + PIN_HEAD_THICKNESS / 2.0
    return _fuse_all(
        cq.Workplane("XZ")
        .circle(barrel_radius)
        .extrude(BARREL_LENGTH / 2.0, both=True)
        .translate((axis_x, 0.0, 0.0)),
        _pin_head(axis_x, head_offset_y),
        _pin_head(axis_x, -head_offset_y),
    )


def _root_pad(root_height: float) -> cq.Workplane:
    pad_length = 0.0060
    return (
        cq.Workplane("XY")
        .box(pad_length, BODY_WIDTH, root_height * 0.86, centered=(False, True, True))
        .translate((ROOT_START_X, 0.0, 0.0))
    )


def _phalanx_with_tip_knuckle(
    *,
    length: float,
    root_height: float,
    tip_height: float,
) -> cq.Workplane:
    barrel_radius = max(KNUCKLE_RADIUS, tip_height * 0.48)
    beam_start_x = ROOT_START_X
    beam_end_x = length - barrel_radius * 0.92
    return _fuse_all(
        _root_pad(root_height),
        _beam(beam_end_x - beam_start_x, root_height, tip_height).translate(
            (beam_start_x, 0.0, 0.0)
        ),
        _knuckle_barrel(length, barrel_radius),
    )


def _distal_phalanx() -> cq.Workplane:
    beam_start_x = ROOT_START_X
    nose_radius = DISTAL_TIP_HEIGHT / 2.0
    nose_center_x = DISTAL_LENGTH - nose_radius
    beam_end_x = nose_center_x - 0.0008
    nose = (
        cq.Workplane("XZ")
        .circle(nose_radius)
        .extrude(BODY_WIDTH / 2.0, both=True)
        .translate((nose_center_x, 0.0, 0.0))
    )
    return _fuse_all(
        _root_pad(DISTAL_ROOT_HEIGHT),
        _beam(beam_end_x - beam_start_x, DISTAL_ROOT_HEIGHT, DISTAL_TIP_HEIGHT).translate(
            (beam_start_x, 0.0, 0.0)
        ),
        nose,
    )


def _tip_pad() -> cq.Workplane:
    pad_back_x = DISTAL_LENGTH - TIP_PAD_LENGTH - 0.002
    pad_body = (
        cq.Workplane("XY")
        .box(
            TIP_PAD_LENGTH,
            BODY_WIDTH * 0.93,
            TIP_PAD_THICKNESS,
            centered=(False, True, False),
        )
        .translate(
            (
                pad_back_x,
                0.0,
                -(DISTAL_TIP_HEIGHT / 2.0 + TIP_PAD_THICKNESS) + 0.0007,
            )
        )
    )
    pad_nose = (
        cq.Workplane("XZ")
        .circle(TIP_PAD_THICKNESS / 2.0)
        .extrude(BODY_WIDTH * 0.93 / 2.0, both=True)
        .translate(
            (
                DISTAL_LENGTH - TIP_PAD_THICKNESS / 2.0,
                0.0,
                -(DISTAL_TIP_HEIGHT / 2.0 + TIP_PAD_THICKNESS / 2.0) + 0.0007,
            )
        )
    )
    return _fuse_all(pad_body, pad_nose)


def _palm_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(PALM_LENGTH, PALM_WIDTH, PALM_HEIGHT)
    top_rib = (
        cq.Workplane("XY")
        .box(0.030, PALM_WIDTH * 0.55, 0.006, centered=True)
        .translate((-0.004, 0.0, PALM_HEIGHT / 2.0 - 0.002))
    )
    front_face_x = PALM_LENGTH / 2.0
    left_support = (
        cq.Workplane("XY")
        .box(
            PALM_ROOT_AXIS_X - front_face_x,
            PALM_SUPPORT_THICKNESS,
            0.0105,
            centered=(False, True, True),
        )
        .translate((front_face_x, FINGER_Y_OFFSET, 0.0))
    )
    right_support = (
        cq.Workplane("XY")
        .box(
            PALM_ROOT_AXIS_X - front_face_x,
            PALM_SUPPORT_THICKNESS,
            0.0105,
            centered=(False, True, True),
        )
        .translate((front_face_x, -FINGER_Y_OFFSET, 0.0))
    )
    left_mount = _knuckle_barrel(PALM_ROOT_AXIS_X, 0.0052).translate((0.0, FINGER_Y_OFFSET, 0.0))
    right_mount = _knuckle_barrel(PALM_ROOT_AXIS_X, 0.0052).translate((0.0, -FINGER_Y_OFFSET, 0.0))
    return _fuse_all(base, top_rib, left_support, right_support, left_mount, right_mount)


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    *,
    mesh_name: str,
    material: str,
    visual_name: str,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        origin=Origin(),
        material=material,
        name=visual_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_finger_mechanism")

    model.material("palm_finish", rgba=(0.21, 0.24, 0.28, 1.0))
    model.material("finger_finish", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("pad_finish", rgba=(0.12, 0.12, 0.13, 1.0))

    palm = model.part("palm")
    _add_mesh_visual(
        palm,
        _palm_shape(),
        mesh_name="palm_shell",
        material="palm_finish",
        visual_name="palm_shell",
    )

    left_proximal = model.part("left_proximal")
    _add_mesh_visual(
        left_proximal,
        _phalanx_with_tip_knuckle(
            length=PROXIMAL_LENGTH,
            root_height=PROXIMAL_ROOT_HEIGHT,
            tip_height=PROXIMAL_TIP_HEIGHT,
        ),
        mesh_name="left_proximal_shell",
        material="finger_finish",
        visual_name="shell",
    )

    left_middle = model.part("left_middle")
    _add_mesh_visual(
        left_middle,
        _phalanx_with_tip_knuckle(
            length=MIDDLE_LENGTH,
            root_height=MIDDLE_ROOT_HEIGHT,
            tip_height=MIDDLE_TIP_HEIGHT,
        ),
        mesh_name="left_middle_shell",
        material="finger_finish",
        visual_name="shell",
    )

    left_distal = model.part("left_distal")
    _add_mesh_visual(
        left_distal,
        _distal_phalanx(),
        mesh_name="left_distal_shell",
        material="finger_finish",
        visual_name="shell",
    )
    _add_mesh_visual(
        left_distal,
        _tip_pad(),
        mesh_name="left_tip_pad",
        material="pad_finish",
        visual_name="tip_pad",
    )

    right_proximal = model.part("right_proximal")
    _add_mesh_visual(
        right_proximal,
        _phalanx_with_tip_knuckle(
            length=PROXIMAL_LENGTH,
            root_height=PROXIMAL_ROOT_HEIGHT,
            tip_height=PROXIMAL_TIP_HEIGHT,
        ),
        mesh_name="right_proximal_shell",
        material="finger_finish",
        visual_name="shell",
    )

    right_middle = model.part("right_middle")
    _add_mesh_visual(
        right_middle,
        _phalanx_with_tip_knuckle(
            length=MIDDLE_LENGTH,
            root_height=MIDDLE_ROOT_HEIGHT,
            tip_height=MIDDLE_TIP_HEIGHT,
        ),
        mesh_name="right_middle_shell",
        material="finger_finish",
        visual_name="shell",
    )

    right_distal = model.part("right_distal")
    _add_mesh_visual(
        right_distal,
        _distal_phalanx(),
        mesh_name="right_distal_shell",
        material="finger_finish",
        visual_name="shell",
    )
    _add_mesh_visual(
        right_distal,
        _tip_pad(),
        mesh_name="right_tip_pad",
        material="pad_finish",
        visual_name="tip_pad",
    )

    joint_limits = (
        MotionLimits(effort=3.0, velocity=3.2, lower=0.0, upper=1.10),
        MotionLimits(effort=2.4, velocity=3.4, lower=0.0, upper=1.25),
        MotionLimits(effort=1.8, velocity=3.6, lower=0.0, upper=1.10),
    )

    model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_proximal,
        origin=Origin(xyz=(PALM_ROOT_AXIS_X, FINGER_Y_OFFSET, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits[0],
    )
    model.articulation(
        "left_proximal_to_left_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits[1],
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits[2],
    )

    model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_proximal,
        origin=Origin(xyz=(PALM_ROOT_AXIS_X, -FINGER_Y_OFFSET, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits[0],
    )
    model.articulation(
        "right_proximal_to_right_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits[1],
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits[2],
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    palm = object_model.get_part("palm")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    left_root = object_model.get_articulation("palm_to_left_proximal")
    left_mid_joint = object_model.get_articulation("left_proximal_to_left_middle")
    left_tip_joint = object_model.get_articulation("left_middle_to_left_distal")
    right_root = object_model.get_articulation("palm_to_right_proximal")
    right_mid_joint = object_model.get_articulation("right_proximal_to_right_middle")
    right_tip_joint = object_model.get_articulation("right_middle_to_right_distal")

    for name, joint in (
        ("left_root_axis", left_root),
        ("left_middle_axis", left_mid_joint),
        ("left_distal_axis", left_tip_joint),
        ("right_root_axis", right_root),
        ("right_middle_axis", right_mid_joint),
        ("right_distal_axis", right_tip_joint),
    ):
        ctx.check(
            name,
            tuple(round(value, 6) for value in joint.axis) == (0.0, 1.0, 0.0),
            f"{joint.name} axis should be +Y for in-plane finger curl, got {joint.axis}",
        )

    ctx.expect_origin_gap(
        left_proximal,
        right_proximal,
        axis="y",
        min_gap=0.020,
        name="finger_roots_are_side_by_side",
    )

    for name, link_a, link_b in (
        ("left_root_knuckle_contact", palm, left_proximal),
        ("left_middle_knuckle_contact", left_proximal, left_middle),
        ("left_distal_knuckle_contact", left_middle, left_distal),
        ("right_root_knuckle_contact", palm, right_proximal),
        ("right_middle_knuckle_contact", right_proximal, right_middle),
        ("right_distal_knuckle_contact", right_middle, right_distal),
    ):
        ctx.expect_contact(link_a, link_b, contact_tol=1e-4, name=name)

    def _center_from_aabb(aabb):
        return tuple((low + high) / 2.0 for low, high in zip(aabb[0], aabb[1]))

    rest_left_tip_aabb = ctx.part_element_world_aabb(left_distal, elem="tip_pad")
    rest_right_distal_pos = ctx.part_world_position(right_distal)

    with ctx.pose(
        {
            left_root: 0.55,
            left_mid_joint: 0.85,
            left_tip_joint: 0.60,
        }
    ):
        bent_left_tip_aabb = ctx.part_element_world_aabb(left_distal, elem="tip_pad")
        bent_right_distal_pos = ctx.part_world_position(right_distal)
        ctx.expect_contact(
            palm,
            left_proximal,
            contact_tol=1e-4,
            name="left_root_stays_supported_when_bent",
        )
        ctx.expect_contact(
            left_proximal,
            left_middle,
            contact_tol=1e-4,
            name="left_middle_stays_supported_when_bent",
        )
        ctx.expect_contact(
            left_middle,
            left_distal,
            contact_tol=1e-4,
            name="left_distal_stays_supported_when_bent",
        )

    if rest_left_tip_aabb is not None and bent_left_tip_aabb is not None:
        rest_left_tip_center = _center_from_aabb(rest_left_tip_aabb)
        bent_left_tip_center = _center_from_aabb(bent_left_tip_aabb)
        ctx.check(
            "left_tip_curls_downward",
            bent_left_tip_center[2] < rest_left_tip_center[2] - 0.012
            and bent_left_tip_center[0] < rest_left_tip_center[0] - 0.008,
            (
                "left fingertip pad should move down and back during curl; "
                f"rest={rest_left_tip_center}, bent={bent_left_tip_center}"
            ),
        )

    if rest_right_distal_pos is not None and bent_right_distal_pos is not None:
        max_delta = max(
            abs(before - after)
            for before, after in zip(rest_right_distal_pos, bent_right_distal_pos)
        )
        ctx.check(
            "right_chain_is_uncoupled_from_left_chain",
            max_delta <= 1e-9,
            f"right distal origin moved by {max_delta} m when only left joints posed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
