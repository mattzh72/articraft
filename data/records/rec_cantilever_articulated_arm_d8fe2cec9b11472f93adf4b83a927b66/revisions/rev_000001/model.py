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


FRAME_FOOT_LENGTH = 0.26
FRAME_FOOT_WIDTH = 0.06
FRAME_FOOT_HEIGHT = 0.03
FRAME_POST_SIZE = 0.05
FRAME_POST_HEIGHT = 0.62
FRAME_POST_X = -0.08
FRAME_POST_SPAN_Y = 0.24
FRAME_TOP_Z = 0.655
FRAME_MID_Z = 0.23

SHOULDER_X = 0.08
SHOULDER_Y = 0.184
SHOULDER_Z = 0.56

UPPER_LINK_LENGTH = 0.78
FORELINK_LENGTH = 0.46

SPINDLE_RADIUS = 0.028
SPINDLE_LENGTH = 0.028
FLANGE_RADIUS = 0.048
FLANGE_LENGTH = 0.008
HUB_RADIUS = 0.058
HUB_LENGTH = 0.020
BORE_RADIUS = 0.030

WRIST_HUB_RADIUS = 0.050
OUTBOARD_Y = 0.038
POSITIVE_KNUCKLE_RADIUS = 0.040
POSITIVE_KNUCKLE_LENGTH = 0.018
NEGATIVE_KNUCKLE_RADIUS = 0.044
NEGATIVE_KNUCKLE_LENGTH = 0.022

LUG_WIDTH_Y = 0.018
EAR_WIDTH_Y = 0.014
EAR_OFFSET_Y = (LUG_WIDTH_Y + EAR_WIDTH_Y) / 2.0
PIVOT_HOLE_RADIUS = 0.010


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _center_lug(
    start_x: float,
    end_x: float,
    *,
    width_y: float = LUG_WIDTH_Y,
    height_z: float = 0.064,
    center_z: float = 0.0,
) -> cq.Workplane:
    return _box((end_x - start_x, width_y, height_z), ((start_x + end_x) / 2.0, 0.0, center_z))


def _fork_ears(
    start_x: float,
    end_x: float,
    *,
    width_y: float = EAR_WIDTH_Y,
    offset_y: float = EAR_OFFSET_Y,
    height_z: float = 0.072,
    center_z: float = 0.0,
) -> cq.Workplane:
    lower = _box((end_x - start_x, width_y, height_z), ((start_x + end_x) / 2.0, -offset_y, center_z))
    upper = _box((end_x - start_x, width_y, height_z), ((start_x + end_x) / 2.0, offset_y, center_z))
    return lower.union(upper)


def _pivot_hole(x_center: float, *, y_center: float = 0.0, z_center: float = 0.0, length: float = 0.080) -> cq.Workplane:
    return _cylinder_y(PIVOT_HOLE_RADIUS, length, (x_center, y_center, z_center))


def _rear_frame_shape() -> cq.Workplane:
    left_y = -FRAME_POST_SPAN_Y / 2.0
    right_y = FRAME_POST_SPAN_Y / 2.0
    shoulder_bracket_y = SHOULDER_Y - 0.016
    shoulder_support_front = SHOULDER_X

    frame = _box((FRAME_FOOT_LENGTH, 0.07, FRAME_FOOT_HEIGHT), (-0.05, left_y, 0.015))
    frame = frame.union(_box((FRAME_FOOT_LENGTH, 0.07, FRAME_FOOT_HEIGHT), (-0.05, right_y, 0.015)))
    frame = frame.union(_box((0.10, FRAME_POST_SPAN_Y + 0.07, 0.05), (-0.15, 0.0, 0.03)))
    frame = frame.union(_box((0.12, FRAME_POST_SPAN_Y - 0.02, 0.04), (-0.01, 0.0, 0.02)))
    frame = frame.union(_box((0.06, 0.06, FRAME_POST_HEIGHT), (FRAME_POST_X, left_y, FRAME_POST_HEIGHT / 2.0)))
    frame = frame.union(_box((0.06, 0.06, FRAME_POST_HEIGHT), (FRAME_POST_X, right_y, FRAME_POST_HEIGHT / 2.0)))
    frame = frame.union(_box((0.06, FRAME_POST_SPAN_Y + 0.06, 0.06), (FRAME_POST_X, 0.0, 0.63)))
    frame = frame.union(_box((0.04, FRAME_POST_SPAN_Y + 0.04, 0.04), (FRAME_POST_X, 0.0, FRAME_MID_Z)))
    frame = frame.union(_box((0.12, 0.05, 0.20), (shoulder_support_front - 0.12, 0.138, 0.44)))
    frame = frame.union(_box((0.10, 0.040, 0.13), (shoulder_support_front - 0.09, 0.140, SHOULDER_Z)))
    frame = frame.union(_box((0.05, 0.032, 0.090), (shoulder_support_front - 0.025, SHOULDER_Y - 0.026, SHOULDER_Z)))
    frame = frame.union(_box((0.050, EAR_WIDTH_Y, 0.078), (shoulder_support_front - 0.025, shoulder_bracket_y, SHOULDER_Z)))

    return frame


def _upper_link_shape() -> cq.Workplane:
    link = _center_lug(0.000, 0.095, height_z=0.068)
    link = link.union(_box((0.52, LUG_WIDTH_Y, 0.086), (0.355, 0.0, 0.0)))
    link = link.union(_box((0.16, 0.048, 0.074), (0.620, 0.0, 0.0)))
    link = link.union(_box((0.06, 0.048, 0.070), (0.730, 0.0, 0.0)))
    link = link.union(_fork_ears(0.700, UPPER_LINK_LENGTH, height_z=0.074))
    link = link.cut(_pivot_hole(0.0, length=LUG_WIDTH_Y + 0.004))
    link = link.cut(_pivot_hole(UPPER_LINK_LENGTH, y_center=-EAR_OFFSET_Y, length=EAR_WIDTH_Y + 0.004))
    link = link.cut(_pivot_hole(UPPER_LINK_LENGTH, y_center=EAR_OFFSET_Y, length=EAR_WIDTH_Y + 0.004))
    return link


def _forelink_shape() -> cq.Workplane:
    link = _center_lug(0.000, 0.090, height_z=0.064)
    link = link.union(_box((0.28, LUG_WIDTH_Y, 0.074), (0.220, 0.0, 0.0)))
    link = link.union(_box((0.10, 0.044, 0.068), (0.340, 0.0, 0.0)))
    link = link.union(_box((0.06, 0.044, 0.064), (0.415, 0.0, 0.0)))
    link = link.union(_fork_ears(0.400, FORELINK_LENGTH, height_z=0.070))
    link = link.cut(_pivot_hole(0.0, length=LUG_WIDTH_Y + 0.004))
    link = link.cut(_pivot_hole(FORELINK_LENGTH, y_center=-EAR_OFFSET_Y, length=EAR_WIDTH_Y + 0.004))
    link = link.cut(_pivot_hole(FORELINK_LENGTH, y_center=EAR_OFFSET_Y, length=EAR_WIDTH_Y + 0.004))
    return link


def _wrist_plate_shape() -> cq.Workplane:
    plate = _center_lug(0.000, 0.070, height_z=0.054)
    plate = plate.union(_box((0.11, 0.014, 0.090), (0.105, 0.0, 0.0)))
    plate = plate.union(_box((0.050, 0.026, 0.032), (0.080, 0.0, -0.020)))
    plate = plate.cut(_pivot_hole(0.0, length=LUG_WIDTH_Y + 0.004))

    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_supported_cantilever_arm")

    model.material("powder_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("anodized_dark", rgba=(0.24, 0.26, 0.29, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        mesh_from_cadquery(_rear_frame_shape(), "rear_frame"),
        material="powder_steel",
        name="rear_frame_shell",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(_upper_link_shape(), "upper_link"),
        material="aluminum",
        name="upper_link_shell",
    )

    forelink = model.part("forelink")
    forelink.visual(
        mesh_from_cadquery(_forelink_shape(), "forelink"),
        material="aluminum",
        name="forelink_shell",
    )

    wrist_plate = model.part("wrist_plate")
    wrist_plate.visual(
        mesh_from_cadquery(_wrist_plate_shape(), "wrist_plate"),
        material="anodized_dark",
        name="wrist_plate_shell",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=upper_link,
        origin=Origin(xyz=(SHOULDER_X, SHOULDER_Y, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=1.15, effort=65.0, velocity=1.5),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forelink,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=45.0, velocity=1.8),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forelink,
        child=wrist_plate,
        origin=Origin(xyz=(FORELINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.0, upper=1.05, effort=20.0, velocity=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    upper_link = object_model.get_part("upper_link")
    forelink = object_model.get_part("forelink")
    wrist_plate = object_model.get_part("wrist_plate")
    shoulder = object_model.get_articulation("shoulder_joint")
    elbow = object_model.get_articulation("elbow_joint")
    wrist = object_model.get_articulation("wrist_joint")

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

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.0}):
        ctx.expect_contact(rear_frame, upper_link, contact_tol=0.003, name="shoulder_joint_surfaces_touch")
        ctx.expect_contact(upper_link, forelink, contact_tol=0.003, name="elbow_joint_surfaces_touch")
        ctx.expect_contact(forelink, wrist_plate, contact_tol=0.003, name="wrist_joint_surfaces_touch")
        ctx.expect_origin_gap(wrist_plate, rear_frame, axis="x", min_gap=1.15, name="arm_projects_forward_from_frame")

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.0}):
        wrist_closed = ctx.part_world_position(wrist_plate)
        plate_closed_aabb = ctx.part_world_aabb(wrist_plate)
    with ctx.pose({shoulder: 0.70, elbow: 0.0, wrist: 0.0}):
        wrist_shoulder_up = ctx.part_world_position(wrist_plate)
    with ctx.pose({shoulder: 0.0, elbow: 0.95, wrist: 0.0}):
        wrist_elbow_up = ctx.part_world_position(wrist_plate)
    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.70}):
        plate_wrist_up_aabb = ctx.part_world_aabb(wrist_plate)

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))

    plate_closed_center = _aabb_center(plate_closed_aabb)
    plate_wrist_up_center = _aabb_center(plate_wrist_up_aabb)

    ctx.check(
        "shoulder_positive_lifts_chain",
        wrist_closed is not None and wrist_shoulder_up is not None and wrist_shoulder_up[2] > wrist_closed[2] + 0.25,
        details=f"closed={wrist_closed}, lifted={wrist_shoulder_up}",
    )
    ctx.check(
        "elbow_positive_raises_forearm_and_pulls_it_back",
        wrist_closed is not None
        and wrist_elbow_up is not None
        and wrist_elbow_up[2] > wrist_closed[2] + 0.12
        and wrist_elbow_up[0] < wrist_closed[0] - 0.08,
        details=f"closed={wrist_closed}, elbow_up={wrist_elbow_up}",
    )
    ctx.check(
        "wrist_positive_tilts_plate_upward",
        plate_closed_center is not None
        and plate_wrist_up_center is not None
        and plate_wrist_up_center[2] > plate_closed_center[2] + 0.02,
        details=f"closed_center={plate_closed_center}, wrist_up_center={plate_wrist_up_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
