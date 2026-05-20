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


PALM_LENGTH = 0.102
PALM_WIDTH = 0.088
PALM_BASE_HEIGHT = 0.018
PALM_BRIDGE_HEIGHT = 0.008
PALM_ROOT_X = 0.035
PALM_ROOT_Z = 0.044
FINGER_OFFSET_Y = 0.026

LINK_OUTER_WIDTH = 0.020
SIDE_PLATE_THICKNESS = 0.004
SIDE_PLATE_HEIGHT = 0.012
SIDE_PLATE_Z = 0.006
HINGE_BARREL_WIDTH = 0.012
HINGE_BARREL_RADIUS = 0.0055
HINGE_GUSSET_LENGTH = 0.010
HINGE_GUSSET_WIDTH = 0.003
HINGE_GUSSET_HEIGHT = 0.004
MID_TIE_HEIGHT = 0.004
MID_TIE_Z = 0.002
FRONT_YOKE_LENGTH = 0.012
FRONT_YOKE_HEIGHT = 0.010
FRONT_YOKE_Z = 0.001
JOINT_TANGENCY_BACKOFF = HINGE_BARREL_RADIUS

LEFT_LINK_LENGTHS = (0.060, 0.046, 0.036)
RIGHT_LINK_LENGTHS = (0.054, 0.047, 0.030)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(
    radius: float,
    length: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _make_palm_shape() -> cq.Workplane:
    base = _box((PALM_LENGTH, PALM_WIDTH, PALM_BASE_HEIGHT), (0.0, 0.0, PALM_BASE_HEIGHT / 2.0))
    center_window = _box((0.050, 0.040, 0.024), (-0.008, 0.0, 0.010))
    side_window = _box((0.024, 0.056, 0.018), (0.018, 0.0, 0.009))
    base = base.cut(center_window).cut(side_window)

    front_bridge = _box(
        (0.040, PALM_WIDTH, PALM_BRIDGE_HEIGHT),
        (0.012, 0.0, PALM_BASE_HEIGHT + PALM_BRIDGE_HEIGHT / 2.0),
    )
    rear_spine = _box((0.030, 0.026, 0.010), (-0.022, 0.0, 0.020))
    center_rib = _box((0.028, 0.022, 0.012), (0.012, 0.0, 0.030))

    mount_shapes: list[cq.Workplane] = [base, front_bridge, rear_spine, center_rib]
    for finger_y in (FINGER_OFFSET_Y, -FINGER_OFFSET_Y):
        pedestal = _box(
            (0.018, LINK_OUTER_WIDTH, 0.020),
            (PALM_ROOT_X - JOINT_TANGENCY_BACKOFF - 0.009, finger_y, 0.028),
        )
        inner_cheek = _box(
            (0.010, SIDE_PLATE_THICKNESS, 0.014),
            (PALM_ROOT_X - JOINT_TANGENCY_BACKOFF - 0.005, finger_y - 0.008, PALM_ROOT_Z),
        )
        outer_cheek = _box(
            (0.010, SIDE_PLATE_THICKNESS, 0.014),
            (PALM_ROOT_X - JOINT_TANGENCY_BACKOFF - 0.005, finger_y + 0.008, PALM_ROOT_Z),
        )
        mount_shapes.extend((pedestal, inner_cheek, outer_cheek))

    return _union_all(mount_shapes)


def _make_link_shape(length: float, *, terminal: bool = False) -> cq.Workplane:
    side_start = 0.010
    side_end = length - (0.006 if terminal else 0.008)
    side_length = side_end - side_start
    side_center_x = side_start + side_length / 2.0
    mid_tie_length = min(max(length * 0.42, 0.016), 0.026)

    shapes = [
        _cylinder_y(HINGE_BARREL_RADIUS, HINGE_BARREL_WIDTH),
        _box(
            (HINGE_GUSSET_LENGTH, HINGE_GUSSET_WIDTH, HINGE_GUSSET_HEIGHT),
            (0.008, 0.0075, 0.002),
        ),
        _box(
            (HINGE_GUSSET_LENGTH, HINGE_GUSSET_WIDTH, HINGE_GUSSET_HEIGHT),
            (0.008, -0.0075, 0.002),
        ),
        _box(
            (side_length, SIDE_PLATE_THICKNESS, SIDE_PLATE_HEIGHT),
            (side_center_x, 0.008, SIDE_PLATE_Z),
        ),
        _box(
            (side_length, SIDE_PLATE_THICKNESS, SIDE_PLATE_HEIGHT),
            (side_center_x, -0.008, SIDE_PLATE_Z),
        ),
        _box((mid_tie_length, LINK_OUTER_WIDTH, MID_TIE_HEIGHT), (length * 0.44, 0.0, MID_TIE_Z)),
    ]

    if terminal:
        nose = _box((0.012, 0.018, 0.010), (length - 0.006, 0.0, 0.004))
        tip_round = _cylinder_y(0.005, 0.018, center=(length - 0.002, 0.0, 0.004))
        shapes.extend((nose, tip_round))
    else:
        front_tie = _box(
            (0.008, LINK_OUTER_WIDTH, MID_TIE_HEIGHT),
            (length - JOINT_TANGENCY_BACKOFF - 0.004, 0.0, MID_TIE_Z),
        )
        left_yoke = _box(
            (FRONT_YOKE_LENGTH, SIDE_PLATE_THICKNESS, FRONT_YOKE_HEIGHT),
            (length - JOINT_TANGENCY_BACKOFF - FRONT_YOKE_LENGTH / 2.0, 0.008, FRONT_YOKE_Z),
        )
        right_yoke = _box(
            (FRONT_YOKE_LENGTH, SIDE_PLATE_THICKNESS, FRONT_YOKE_HEIGHT),
            (length - JOINT_TANGENCY_BACKOFF - FRONT_YOKE_LENGTH / 2.0, -0.008, FRONT_YOKE_Z),
        )
        shapes.extend((front_tie, left_yoke, right_yoke))

    return _union_all(shapes)


def _add_mesh_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    material: str,
) -> object:
    part = model.part(name)
    part.visual(mesh_from_cadquery(shape, f"{name}_mesh", tolerance=0.0004), material=material, name=f"{name}_shell")
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_dual_finger_rig")

    model.material("palm_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("link_metal", rgba=(0.71, 0.74, 0.78, 1.0))
    model.material("tip_metal", rgba=(0.78, 0.80, 0.82, 1.0))

    palm = _add_mesh_part(model, name="palm", shape=_make_palm_shape(), material="palm_dark")

    left_proximal = _add_mesh_part(
        model,
        name="left_proximal",
        shape=_make_link_shape(LEFT_LINK_LENGTHS[0]),
        material="link_metal",
    )
    left_middle = _add_mesh_part(
        model,
        name="left_middle",
        shape=_make_link_shape(LEFT_LINK_LENGTHS[1]),
        material="link_metal",
    )
    left_distal = _add_mesh_part(
        model,
        name="left_distal",
        shape=_make_link_shape(LEFT_LINK_LENGTHS[2], terminal=True),
        material="tip_metal",
    )

    right_proximal = _add_mesh_part(
        model,
        name="right_proximal",
        shape=_make_link_shape(RIGHT_LINK_LENGTHS[0]),
        material="link_metal",
    )
    right_middle = _add_mesh_part(
        model,
        name="right_middle",
        shape=_make_link_shape(RIGHT_LINK_LENGTHS[1]),
        material="link_metal",
    )
    right_distal = _add_mesh_part(
        model,
        name="right_distal",
        shape=_make_link_shape(RIGHT_LINK_LENGTHS[2], terminal=True),
        material="tip_metal",
    )

    joint_limits_root = MotionLimits(effort=8.0, velocity=2.4, lower=0.0, upper=0.80)
    joint_limits_mid = MotionLimits(effort=6.0, velocity=2.8, lower=0.0, upper=1.05)
    joint_limits_tip = MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=0.90)

    model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_proximal,
        origin=Origin(xyz=(PALM_ROOT_X, FINGER_OFFSET_Y, PALM_ROOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits_root,
    )
    model.articulation(
        "left_proximal_to_left_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(LEFT_LINK_LENGTHS[0], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits_mid,
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(LEFT_LINK_LENGTHS[1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits_tip,
    )

    model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_proximal,
        origin=Origin(xyz=(PALM_ROOT_X, -FINGER_OFFSET_Y, PALM_ROOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits_root,
    )
    model.articulation(
        "right_proximal_to_right_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(RIGHT_LINK_LENGTHS[0], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits_mid,
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(RIGHT_LINK_LENGTHS[1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits_tip,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    left_root = object_model.get_articulation("palm_to_left_proximal")
    left_mid = object_model.get_articulation("left_proximal_to_left_middle")
    left_tip = object_model.get_articulation("left_middle_to_left_distal")
    right_root = object_model.get_articulation("palm_to_right_proximal")
    right_mid = object_model.get_articulation("right_proximal_to_right_middle")
    right_tip = object_model.get_articulation("right_middle_to_right_distal")

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

    for part_name, part in (
        ("palm", palm),
        ("left_proximal", left_proximal),
        ("left_middle", left_middle),
        ("left_distal", left_distal),
        ("right_proximal", right_proximal),
        ("right_middle", right_middle),
        ("right_distal", right_distal),
    ):
        ctx.check(f"{part_name}_present", part is not None, details=f"Missing required part {part_name}.")

    ctx.expect_contact(left_proximal, palm, contact_tol=0.0008, name="left_root_mount_contact")
    ctx.expect_contact(left_middle, left_proximal, contact_tol=0.0008, name="left_middle_mount_contact")
    ctx.expect_contact(left_distal, left_middle, contact_tol=0.0008, name="left_distal_mount_contact")
    ctx.expect_contact(right_proximal, palm, contact_tol=0.0008, name="right_root_mount_contact")
    ctx.expect_contact(right_middle, right_proximal, contact_tol=0.0008, name="right_middle_mount_contact")
    ctx.expect_contact(right_distal, right_middle, contact_tol=0.0008, name="right_distal_mount_contact")

    ctx.expect_origin_gap(
        left_proximal,
        right_proximal,
        axis="y",
        min_gap=0.048,
        name="finger_root_spacing",
    )

    with ctx.pose():
        left_open = ctx.part_world_position(left_distal)
        right_open = ctx.part_world_position(right_distal)
        left_tip_box = ctx.part_world_aabb(left_distal)
        right_tip_box = ctx.part_world_aabb(right_distal)

    with ctx.pose(
        **{
            left_root.name: 0.42,
            left_mid.name: 0.58,
            left_tip.name: 0.34,
            right_root.name: 0.38,
            right_mid.name: 0.54,
            right_tip.name: 0.30,
        }
    ):
        left_flexed = ctx.part_world_position(left_distal)
        right_flexed = ctx.part_world_position(right_distal)

    left_tip_x = left_tip_box[1][0] if left_tip_box is not None else None
    right_tip_x = right_tip_box[1][0] if right_tip_box is not None else None
    ctx.check(
        "finger_reach_is_asymmetric",
        left_tip_x is not None and right_tip_x is not None and abs(left_tip_x - right_tip_x) > 0.004,
        details=f"Left/right fingertip reach difference too small: left={left_tip_x}, right={right_tip_x}",
    )
    ctx.check(
        "left_finger_positive_motion_curls_downward",
        left_open is not None and left_flexed is not None and left_flexed[2] < left_open[2] - 0.012,
        details=f"Left distal origin did not move downward under positive flex: open={left_open}, flexed={left_flexed}",
    )
    ctx.check(
        "right_finger_positive_motion_curls_downward",
        right_open is not None and right_flexed is not None and right_flexed[2] < right_open[2] - 0.012,
        details=f"Right distal origin did not move downward under positive flex: open={right_open}, flexed={right_flexed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
