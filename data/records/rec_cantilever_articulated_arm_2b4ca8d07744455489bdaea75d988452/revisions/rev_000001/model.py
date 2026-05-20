from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


SHOULDER_X = 0.015
SHOULDER_Z = 0.162
UPPER_ARM_LENGTH = 0.42
FOREARM_LENGTH = 0.30


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _side_frame(
    x_start: float,
    x_end: float,
    y_center: float,
    *,
    thickness: float,
    height: float,
    window_margin_x: float,
    window_margin_z: float,
) -> cq.Workplane:
    length = x_end - x_start
    center_x = (x_start + x_end) * 0.5
    cheek = _box((length, thickness, height), (center_x, y_center, 0.0))
    window_length = max(length - 2.0 * window_margin_x, 0.01)
    window_height = max(height - 2.0 * window_margin_z, 0.01)
    window = _box((window_length, thickness * 4.0, window_height), (center_x, y_center, 0.0))
    return cheek.cut(window)


def _fuse_all(solids: list[cq.Workplane]) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _make_pedestal() -> cq.Workplane:
    base = _box((0.28, 0.22, 0.018), (0.0, 0.0, 0.009))
    column = _box((0.07, 0.07, 0.12), (-0.05, 0.0, 0.078))
    rear_cap = _box((0.03, 0.09, 0.025), (-0.01, 0.0, 0.1505))
    left_ear = _box((0.055, 0.01, 0.078), (-0.0125, 0.048, 0.158))
    right_ear = _box((0.055, 0.01, 0.078), (-0.0125, -0.048, 0.158))
    left_web = (
        cq.Workplane("XZ")
        .polyline([(-0.11, 0.018), (-0.015, 0.018), (0.005, 0.145), (-0.085, 0.145)])
        .close()
        .extrude(0.014, both=True)
        .translate((0.0, 0.053, 0.0))
    )
    right_web = (
        cq.Workplane("XZ")
        .polyline([(-0.11, 0.018), (-0.015, 0.018), (0.005, 0.145), (-0.085, 0.145)])
        .close()
        .extrude(0.014, both=True)
        .translate((0.0, -0.053, 0.0))
    )
    return _fuse_all([base, column, rear_cap, left_ear, right_ear, left_web, right_web])


def _make_upper_arm() -> cq.Workplane:
    side_t = 0.008
    side_gap = 0.058
    frame_h = 0.09
    y_offset = side_gap * 0.5 + side_t * 0.5

    left_cheek = _side_frame(
        0.05,
        UPPER_ARM_LENGTH,
        y_offset,
        thickness=side_t,
        height=frame_h,
        window_margin_x=0.065,
        window_margin_z=0.018,
    )
    right_cheek = _side_frame(
        0.05,
        UPPER_ARM_LENGTH,
        -y_offset,
        thickness=side_t,
        height=frame_h,
        window_margin_x=0.065,
        window_margin_z=0.018,
    )
    shoulder_hub = _y_cylinder(0.029, 0.086, (0.029, 0.0, 0.0))
    top_rib = _box((0.18, side_gap, 0.01), (0.18, 0.0, 0.033))
    bottom_rib = _box((0.18, side_gap, 0.01), (0.18, 0.0, -0.033))
    distal_top_bridge = _box((0.026, side_gap, 0.008), (UPPER_ARM_LENGTH - 0.013, 0.0, 0.036))
    distal_bottom_bridge = _box((0.026, side_gap, 0.008), (UPPER_ARM_LENGTH - 0.013, 0.0, -0.036))
    return _fuse_all(
        [
            left_cheek,
            right_cheek,
            shoulder_hub,
            top_rib,
            bottom_rib,
            distal_top_bridge,
            distal_bottom_bridge,
        ]
    )


def _make_forearm() -> cq.Workplane:
    side_t = 0.008
    side_gap = 0.05
    frame_h = 0.075
    y_offset = side_gap * 0.5 + side_t * 0.5

    left_cheek = _side_frame(
        0.04,
        FOREARM_LENGTH,
        y_offset,
        thickness=side_t,
        height=frame_h,
        window_margin_x=0.05,
        window_margin_z=0.016,
    )
    right_cheek = _side_frame(
        0.04,
        FOREARM_LENGTH,
        -y_offset,
        thickness=side_t,
        height=frame_h,
        window_margin_x=0.05,
        window_margin_z=0.016,
    )
    elbow_hub = _y_cylinder(0.025, 0.058, (0.025, 0.0, 0.0))
    top_rib = _box((0.14, side_gap, 0.009), (0.145, 0.0, 0.028))
    bottom_rib = _box((0.14, side_gap, 0.009), (0.145, 0.0, -0.028))
    distal_top_bridge = _box((0.024, side_gap, 0.008), (FOREARM_LENGTH - 0.012, 0.0, 0.03))
    distal_bottom_bridge = _box((0.024, side_gap, 0.008), (FOREARM_LENGTH - 0.012, 0.0, -0.03))
    return _fuse_all(
        [
            left_cheek,
            right_cheek,
            elbow_hub,
            top_rib,
            bottom_rib,
            distal_top_bridge,
            distal_bottom_bridge,
        ]
    )


def _make_wrist_plate() -> cq.Workplane:
    hub = _y_cylinder(0.022, 0.05, (0.022, 0.0, 0.0))
    bracket = _box((0.088, 0.038, 0.05), (0.05, 0.0, 0.0))
    bracket_window = _box((0.046, 0.07, 0.022), (0.047, 0.0, 0.0))
    bracket = bracket.cut(bracket_window)

    plate = _box((0.008, 0.10, 0.12), (0.096, 0.0, 0.0))
    slot_left = _box((0.02, 0.016, 0.048), (0.096, 0.026, 0.0))
    slot_right = _box((0.02, 0.016, 0.048), (0.096, -0.026, 0.0))
    plate = plate.cut(slot_left).cut(slot_right)

    return _fuse_all([hub, bracket, plate])


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_cantilever_arm")

    model.material("pedestal_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("arm_aluminum", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("wrist_dark", rgba=(0.16, 0.17, 0.19, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.30, 0.24, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
        material="pedestal_gray",
    )
    pedestal.visual(
        Box((0.08, 0.08, 0.13)),
        origin=Origin(xyz=(-0.07, 0.0, 0.085)),
        name="mast",
        material="pedestal_gray",
    )
    pedestal.visual(
        Box((0.05, 0.09, 0.025)),
        origin=Origin(xyz=(-0.02, 0.0, 0.1495)),
        name="shoulder_cap",
        material="pedestal_gray",
    )
    pedestal.visual(
        Box((0.03, 0.012, 0.09)),
        origin=Origin(xyz=(0.0, 0.045, SHOULDER_Z)),
        name="left_ear",
        material="pedestal_gray",
    )
    pedestal.visual(
        Box((0.03, 0.012, 0.09)),
        origin=Origin(xyz=(0.0, -0.045, SHOULDER_Z)),
        name="right_ear",
        material="pedestal_gray",
    )
    pedestal.visual(
        Box((0.11, 0.012, 0.13)),
        origin=Origin(xyz=(-0.055, 0.045, 0.083)),
        name="left_strut",
        material="pedestal_gray",
    )
    pedestal.visual(
        Box((0.11, 0.012, 0.13)),
        origin=Origin(xyz=(-0.055, -0.045, 0.083)),
        name="right_strut",
        material="pedestal_gray",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.016, length=0.078),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        name="shoulder_drum",
        material="arm_aluminum",
    )
    upper_arm.visual(
        Box((0.032, 0.052, 0.07)),
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        name="proximal_block",
        material="arm_aluminum",
    )
    for name, y_pos, z_pos in (
        ("rail_lt", 0.028, 0.03),
        ("rail_lb", 0.028, -0.03),
        ("rail_rt", -0.028, 0.03),
        ("rail_rb", -0.028, -0.03),
    ):
        upper_arm.visual(
            Box((0.332, 0.008, 0.012)),
            origin=Origin(xyz=(0.23, y_pos, z_pos)),
            name=name,
            material="arm_aluminum",
        )
    upper_arm.visual(
        Box((0.024, 0.064, 0.06)),
        origin=Origin(xyz=(0.408, 0.0, 0.0)),
        name="elbow_mount",
        material="arm_aluminum",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.015, length=0.064),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        name="elbow_drum",
        material="arm_aluminum",
    )
    forearm.visual(
        Box((0.026, 0.044, 0.056)),
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
        name="forearm_root",
        material="arm_aluminum",
    )
    for name, y_pos, z_pos in (
        ("fore_rail_lt", 0.024, 0.026),
        ("fore_rail_lb", 0.024, -0.026),
        ("fore_rail_rt", -0.024, 0.026),
        ("fore_rail_rb", -0.024, -0.026),
    ):
        forearm.visual(
            Box((0.226, 0.007, 0.01)),
            origin=Origin(xyz=(0.169, y_pos, z_pos)),
            name=name,
            material="arm_aluminum",
        )
    forearm.visual(
        Box((0.018, 0.056, 0.05)),
        origin=Origin(xyz=(0.291, 0.0, 0.0)),
        name="wrist_mount",
        material="arm_aluminum",
    )

    wrist_plate = model.part("wrist_plate")
    wrist_plate.visual(
        Cylinder(radius=0.014, length=0.056),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        name="wrist_drum",
        material="wrist_dark",
    )
    wrist_plate.visual(
        Box((0.032, 0.04, 0.05)),
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        name="wrist_block",
        material="wrist_dark",
    )
    wrist_plate.visual(
        Box((0.03, 0.036, 0.022)),
        origin=Origin(xyz=(0.057, 0.0, 0.0)),
        name="plate_neck",
        material="wrist_dark",
    )
    wrist_plate.visual(
        Box((0.032, 0.012, 0.09)),
        origin=Origin(xyz=(0.052, 0.03, 0.0)),
        name="left_plate_rib",
        material="wrist_dark",
    )
    wrist_plate.visual(
        Box((0.032, 0.012, 0.09)),
        origin=Origin(xyz=(0.052, -0.03, 0.0)),
        name="right_plate_rib",
        material="wrist_dark",
    )
    wrist_plate.visual(
        Box((0.012, 0.12, 0.14)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        name="wrist_plate_body",
        material="wrist_dark",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.8, upper=1.2),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.6, lower=-1.45, upper=1.65),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_plate,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-1.7, upper=1.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_plate = object_model.get_part("wrist_plate")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

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

    ctx.expect_contact(upper_arm, pedestal, name="shoulder_parts_touch")
    ctx.expect_contact(forearm, upper_arm, name="elbow_parts_touch")
    ctx.expect_contact(wrist_plate, forearm, name="wrist_parts_touch")

    ctx.expect_origin_gap(
        upper_arm,
        pedestal,
        axis="z",
        min_gap=0.155,
        max_gap=0.17,
        name="shoulder_axis_height",
    )
    ctx.expect_origin_gap(
        forearm,
        upper_arm,
        axis="x",
        min_gap=0.41,
        max_gap=0.43,
        name="upper_arm_reach",
    )
    ctx.expect_origin_gap(
        wrist_plate,
        forearm,
        axis="x",
        min_gap=0.29,
        max_gap=0.31,
        name="forearm_reach",
    )
    ctx.check(
        "joint_axes_share_vertical_plane",
        shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and wrist.axis == (0.0, -1.0, 0.0),
        "shoulder, elbow, and wrist should all pitch about the shared Y axis",
    )

    with ctx.pose({shoulder: 0.6, elbow: 0.0, wrist: 0.0}):
        elbow_pos = ctx.part_world_position(forearm)
        wrist_pos = ctx.part_world_position(wrist_plate)
        ctx.check(
            "shoulder_positive_raises_main_span",
            elbow_pos is not None
            and wrist_pos is not None
            and elbow_pos[2] > SHOULDER_Z + 0.22
            and wrist_pos[2] > elbow_pos[2] + 0.14,
            "positive shoulder motion should lift the elbow and the wrist upward",
        )

    with ctx.pose({shoulder: 0.0, elbow: 0.8, wrist: 0.0}):
        elbow_pos = ctx.part_world_position(forearm)
        wrist_pos = ctx.part_world_position(wrist_plate)
        ctx.check(
            "elbow_positive_lifts_forearm",
            elbow_pos is not None
            and wrist_pos is not None
            and wrist_pos[2] > elbow_pos[2] + 0.18,
            "positive elbow motion should raise the distal wrist joint above the elbow axis",
        )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.0}):
        wrist_rest_center = _aabb_center(ctx.part_element_world_aabb(wrist_plate, elem="wrist_plate_body"))
    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.9}):
        wrist_tilted_center = _aabb_center(ctx.part_element_world_aabb(wrist_plate, elem="wrist_plate_body"))
    ctx.check(
        "wrist_joint_rotates_end_plate",
        wrist_rest_center is not None
        and wrist_tilted_center is not None
        and wrist_tilted_center[2] > wrist_rest_center[2] + 0.02
        and wrist_tilted_center[0] < wrist_rest_center[0] - 0.005,
        "wrist rotation should pitch the end plate upward about the distal hinge",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
