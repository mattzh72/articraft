from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


YAW_AXIS_HEIGHT = 0.135

BASE_LENGTH = 0.150
BASE_WIDTH = 0.096
BASE_THICKNESS = 0.012

COLLAR_LENGTH = 0.018
COLLAR_RADIUS = 0.032
SUPPORT_WIDTH = 0.086
GUSSET_WIDTH = 0.040

CARTRIDGE_LENGTH = 0.040
CARTRIDGE_RADIUS = 0.032

YOKE_LENGTH = 0.056

NOSE_BODY_LENGTH = 0.060
NOSE_TIP_LENGTH = 0.026
NOSE_RADIUS = 0.018
NOSE_TIP_RADIUS = 0.008


def _make_support_shape():
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .translate((-0.055, 0.0, 0.0))
    )

    cheek = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.036, BASE_THICKNESS),
                (-0.092, BASE_THICKNESS),
                (-0.098, 0.058),
                (-0.076, 0.120),
                (-0.042, 0.168),
                (-0.018, 0.180),
                (-0.018, BASE_THICKNESS),
            ]
        )
        .close()
        .extrude(SUPPORT_WIDTH / 2.0, both=True)
    )

    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.120, BASE_THICKNESS),
                (-0.018, BASE_THICKNESS),
                (-0.018, 0.094),
                (-0.078, 0.070),
            ]
        )
        .close()
        .extrude(GUSSET_WIDTH / 2.0, both=True)
    )

    collar = (
        cq.Workplane("YZ")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_LENGTH)
        .translate((-COLLAR_LENGTH - 0.004, 0.0, YAW_AXIS_HEIGHT))
    )

    return base.union(cheek).union(gusset).union(collar)


def _make_yaw_cartridge_shape():
    rear_flange = (
        cq.Workplane("YZ")
        .circle(0.036)
        .extrude(0.006)
    )

    drum = (
        cq.Workplane("YZ")
        .circle(CARTRIDGE_RADIUS)
        .extrude(0.028)
        .translate((0.006, 0.0, 0.0))
    )

    front_block = (
        cq.Workplane("XY")
        .box(0.010, 0.050, 0.058, centered=(False, True, True))
        .translate((0.030, 0.0, 0.0))
    )

    top_cover = (
        cq.Workplane("XY")
        .box(0.014, 0.032, 0.016, centered=(False, True, True))
        .translate((0.018, 0.0, 0.022))
    )

    return rear_flange.union(drum).union(front_block).union(top_cover)


def _make_pitch_yoke_shape():
    rear_frame = (
        cq.Workplane("XY")
        .box(0.016, 0.068, 0.052, centered=(False, True, True))
        .cut(
            cq.Workplane("XY")
            .box(0.010, 0.034, 0.026, centered=(False, True, True))
            .translate((0.006, 0.0, 0.0))
        )
    )

    left_arm = (
        cq.Workplane("XY")
        .box(0.040, 0.012, 0.028, centered=(False, True, True))
        .translate((0.016, 0.027, 0.0))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(0.040, 0.012, 0.028, centered=(False, True, True))
        .translate((0.016, -0.027, 0.0))
    )

    front_bridge = (
        cq.Workplane("XY")
        .box(0.012, 0.060, 0.016, centered=(False, True, True))
        .translate((0.032, 0.0, -0.016))
    )

    return rear_frame.union(left_arm).union(right_arm).union(front_bridge)


def _make_roll_nose_shape():
    rear_barrel = (
        cq.Workplane("YZ")
        .circle(0.020)
        .extrude(0.018)
        .translate((-0.006, 0.0, 0.0))
    )

    main_body = (
        cq.Workplane("YZ")
        .circle(NOSE_RADIUS)
        .extrude(NOSE_BODY_LENGTH)
    )

    tip = (
        cq.Workplane("YZ")
        .workplane(offset=NOSE_BODY_LENGTH)
        .circle(NOSE_RADIUS)
        .workplane(offset=NOSE_TIP_LENGTH)
        .circle(NOSE_TIP_RADIUS)
        .loft(combine=True)
    )

    lower_flat = (
        cq.Workplane("XY")
        .box(0.030, 0.020, 0.010, centered=(False, True, True))
        .translate((0.020, 0.0, -0.018))
    )

    return rear_barrel.union(main_body).union(tip).cut(lower_flat)


def _aabb_center(aabb):
    return tuple((aabb[0][idx] + aabb[1][idx]) * 0.5 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_cheek_yaw_pitch_roll_module")

    model.material("support_charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("cartridge_silver", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("yoke_graphite", rgba=(0.28, 0.30, 0.34, 1.0))
    model.material("nose_silver", rgba=(0.79, 0.80, 0.82, 1.0))
    model.material("optic_black", rgba=(0.08, 0.08, 0.09, 1.0))

    support = model.part("side_support")
    support.visual(
        mesh_from_cadquery(_make_support_shape(), "side_support_shell"),
        material="support_charcoal",
        name="support_shell",
    )
    support.visual(
        Box((0.004, 0.050, 0.060)),
        origin=Origin(xyz=(-0.002, 0.0, YAW_AXIS_HEIGHT)),
        material="support_charcoal",
        name="mount_pad",
    )

    yaw_cartridge = model.part("yaw_cartridge")
    yaw_cartridge.visual(
        mesh_from_cadquery(_make_yaw_cartridge_shape(), "yaw_cartridge_shell"),
        material="cartridge_silver",
        name="cartridge_shell",
    )
    yaw_cartridge.visual(
        Box((0.004, 0.048, 0.056)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material="cartridge_silver",
        name="back_pad",
    )
    yaw_cartridge.visual(
        Box((0.004, 0.042, 0.050)),
        origin=Origin(xyz=(CARTRIDGE_LENGTH - 0.002, 0.0, 0.0)),
        material="cartridge_silver",
        name="front_pad",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        mesh_from_cadquery(_make_pitch_yoke_shape(), "pitch_yoke_shell"),
        material="yoke_graphite",
        name="yoke_shell",
    )
    pitch_yoke.visual(
        Box((0.004, 0.040, 0.048)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material="yoke_graphite",
        name="rear_pad",
    )
    pitch_yoke.visual(
        Box((0.008, 0.020, 0.036)),
        origin=Origin(xyz=(0.046, 0.024, 0.0)),
        material="yoke_graphite",
        name="left_bearing",
    )
    pitch_yoke.visual(
        Box((0.008, 0.020, 0.036)),
        origin=Origin(xyz=(0.046, -0.024, 0.0)),
        material="yoke_graphite",
        name="right_bearing",
    )

    roll_nose = model.part("roll_nose")
    roll_nose.visual(
        mesh_from_cadquery(_make_roll_nose_shape(), "roll_nose_shell"),
        material="nose_silver",
        name="nose_shell",
    )
    roll_nose.visual(
        Box((0.018, 0.028, 0.032)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material="nose_silver",
        name="roll_collar",
    )
    roll_nose.visual(
        Box((0.028, 0.018, 0.010)),
        origin=Origin(xyz=(0.042, 0.0, 0.018)),
        material="optic_black",
        name="index_hood",
    )

    model.articulation(
        "support_to_yaw",
        ArticulationType.REVOLUTE,
        parent=support,
        child=yaw_cartridge,
        origin=Origin(xyz=(0.0, 0.0, YAW_AXIS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.40,
            upper=1.40,
            effort=12.0,
            velocity=2.0,
        ),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_cartridge,
        child=pitch_yoke,
        origin=Origin(xyz=(CARTRIDGE_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.70,
            upper=1.05,
            effort=8.0,
            velocity=2.4,
        ),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_nose,
        origin=Origin(xyz=(YOKE_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.50,
            upper=1.50,
            effort=6.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("side_support")
    yaw_cartridge = object_model.get_part("yaw_cartridge")
    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_nose = object_model.get_part("roll_nose")

    yaw_joint = object_model.get_articulation("support_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")
    roll_joint = object_model.get_articulation("pitch_to_roll")

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

    support_aabb = ctx.part_world_aabb(support)
    ctx.check(
        "support_grounded_at_floor",
        support_aabb is not None and abs(support_aabb[0][2]) <= 0.001,
        f"support min z={None if support_aabb is None else support_aabb[0][2]:.6f}",
    )

    ctx.expect_contact(
        support,
        yaw_cartridge,
        elem_a="mount_pad",
        elem_b="back_pad",
        contact_tol=1e-5,
        name="support_mount_contacts_yaw_cartridge",
    )
    ctx.expect_overlap(
        support,
        yaw_cartridge,
        elem_a="mount_pad",
        elem_b="back_pad",
        axes="yz",
        min_overlap=0.040,
        name="support_mount_pad_carries_yaw_cartridge",
    )

    ctx.expect_contact(
        yaw_cartridge,
        pitch_yoke,
        elem_a="front_pad",
        elem_b="rear_pad",
        contact_tol=1e-5,
        name="yaw_cartridge_faces_pitch_yoke",
    )
    ctx.expect_overlap(
        yaw_cartridge,
        pitch_yoke,
        elem_a="front_pad",
        elem_b="rear_pad",
        axes="yz",
        min_overlap=0.030,
        name="pitch_yoke_has_full_yaw_mount_footprint",
    )

    ctx.expect_contact(
        pitch_yoke,
        roll_nose,
        elem_a="left_bearing",
        elem_b="roll_collar",
        contact_tol=1e-5,
        name="left_roll_bearing_contacts_nose",
    )
    ctx.expect_contact(
        pitch_yoke,
        roll_nose,
        elem_a="right_bearing",
        elem_b="roll_collar",
        contact_tol=1e-5,
        name="right_roll_bearing_contacts_nose",
    )

    ctx.check(
        "yaw_axis_is_vertical",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch_axis_is_lateral",
        pitch_joint.axis == (0.0, -1.0, 0.0),
        f"axis={pitch_joint.axis}",
    )
    ctx.check(
        "roll_axis_is_longitudinal",
        roll_joint.axis == (1.0, 0.0, 0.0),
        f"axis={roll_joint.axis}",
    )

    rest_nose_pos = ctx.part_world_position(roll_nose)
    with ctx.pose({yaw_joint: 0.45}):
        yawed_nose_pos = ctx.part_world_position(roll_nose)
    ctx.check(
        "positive_yaw_swings_module_toward_positive_y",
        rest_nose_pos is not None
        and yawed_nose_pos is not None
        and yawed_nose_pos[1] > rest_nose_pos[1] + 0.020,
        f"rest={rest_nose_pos}, yawed={yawed_nose_pos}",
    )

    with ctx.pose({pitch_joint: 0.45}):
        pitched_nose_pos = ctx.part_world_position(roll_nose)
    ctx.check(
        "positive_pitch_raises_roll_nose",
        rest_nose_pos is not None
        and pitched_nose_pos is not None
        and pitched_nose_pos[2] > rest_nose_pos[2] + 0.020,
        f"rest={rest_nose_pos}, pitched={pitched_nose_pos}",
    )

    hood_rest_aabb = ctx.part_element_world_aabb(roll_nose, elem="index_hood")
    with ctx.pose({roll_joint: 0.50}):
        hood_rolled_aabb = ctx.part_element_world_aabb(roll_nose, elem="index_hood")
    hood_rest_center = None if hood_rest_aabb is None else _aabb_center(hood_rest_aabb)
    hood_rolled_center = None if hood_rolled_aabb is None else _aabb_center(hood_rolled_aabb)
    ctx.check(
        "positive_roll_moves_index_hood_toward_negative_y",
        hood_rest_center is not None
        and hood_rolled_center is not None
        and hood_rolled_center[1] < hood_rest_center[1] - 0.004,
        f"rest={hood_rest_center}, rolled={hood_rolled_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
