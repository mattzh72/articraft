from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_revolute_revolute_chain", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    machined_gray = model.material("machined_gray", rgba=(0.62, 0.65, 0.68, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.87, 0.42, 0.12, 1.0))
    tool_blue = model.material("tool_blue", rgba=(0.22, 0.40, 0.67, 1.0))

    rail = model.part("base_rail")
    rail.visual(
        Box((0.34, 0.07, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_steel,
        name="rail_foot",
    )
    rail.visual(
        Box((0.34, 0.052, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_steel,
        name="rail_rib",
    )
    rail.inertial = Inertial.from_geometry(
        Box((0.34, 0.07, 0.024)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.07, 0.011, 0.026)),
        origin=Origin(xyz=(0.0, -0.0245, 0.013)),
        material=machined_gray,
        name="left_skirt",
    )
    carriage.visual(
        Box((0.07, 0.011, 0.026)),
        origin=Origin(xyz=(0.0, 0.0245, 0.013)),
        material=machined_gray,
        name="right_skirt",
    )
    carriage.visual(
        Box((0.07, 0.06, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=machined_gray,
        name="top_bridge",
    )
    carriage.visual(
        Box((0.05, 0.038, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=machined_gray,
        name="top_pad",
    )
    carriage.visual(
        Box((0.018, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, -0.012, 0.060)),
        material=machined_gray,
        name="left_clevis_ear",
    )
    carriage.visual(
        Box((0.018, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, 0.060)),
        material=machined_gray,
        name="right_clevis_ear",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.07, 0.06, 0.072)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_orange,
        name="root_barrel",
    )
    shoulder.visual(
        Box((0.014, 0.01, 0.082)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=safety_orange,
        name="main_web",
    )
    shoulder.visual(
        Box((0.016, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=safety_orange,
        name="elbow_crosshead",
    )
    shoulder.visual(
        Box((0.014, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, -0.010, 0.095)),
        material=safety_orange,
        name="left_fork_ear",
    )
    shoulder.visual(
        Box((0.014, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.010, 0.095)),
        material=safety_orange,
        name="right_fork_ear",
    )
    shoulder.inertial = Inertial.from_geometry(
        Box((0.016, 0.026, 0.118)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
    )

    forelink = model.part("forelink")
    forelink.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tool_blue,
        name="elbow_barrel",
    )
    forelink.visual(
        Box((0.012, 0.010, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=tool_blue,
        name="drop_web",
    )
    forelink.visual(
        Box((0.022, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=tool_blue,
        name="toe_pad",
    )
    forelink.inertial = Inertial.from_geometry(
        Box((0.022, 0.014, 0.048)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )

    model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.35,
            lower=-0.075,
            upper=0.075,
        ),
    )
    model.articulation(
        "shoulder_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=2.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forelink,
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.5,
            lower=-math.radians(75.0),
            upper=math.radians(75.0),
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    rail = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    shoulder = object_model.get_part("shoulder_link")
    forelink = object_model.get_part("forelink")

    slide = object_model.get_articulation("rail_slide")
    shoulder_hinge = object_model.get_articulation("shoulder_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_gap(
        carriage,
        rail,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage_sits_on_rail",
    )
    ctx.expect_within(
        carriage,
        rail,
        axes="x",
        margin=0.0,
        name="carriage_stays_within_rail_length_at_rest",
    )
    ctx.expect_contact(
        shoulder,
        carriage,
        name="shoulder_barrel_seats_in_carriage_clevis",
    )
    ctx.expect_contact(
        forelink,
        shoulder,
        name="forelink_barrel_seats_in_elbow_fork",
    )

    with ctx.pose({slide: -0.075}):
        left_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.075}):
        right_pos = ctx.part_world_position(carriage)
    slide_ok = (
        left_pos is not None
        and right_pos is not None
        and abs((right_pos[0] - left_pos[0]) - 0.15) <= 1e-6
        and abs(right_pos[1] - left_pos[1]) <= 1e-6
        and abs(right_pos[2] - left_pos[2]) <= 1e-6
    )
    ctx.check(
        "rail_slide_has_150mm_travel_along_x",
        slide_ok,
        details=f"left={left_pos}, right={right_pos}",
    )

    with ctx.pose({shoulder_hinge: -math.pi / 2.0}):
        shoulder_neg_center = _aabb_center(ctx.part_world_aabb(shoulder))
        ctx.expect_contact(
            shoulder,
            carriage,
            name="shoulder_stays_seated_at_negative_limit",
        )
    with ctx.pose({shoulder_hinge: math.pi / 2.0}):
        shoulder_pos_center = _aabb_center(ctx.part_world_aabb(shoulder))
        ctx.expect_contact(
            shoulder,
            carriage,
            name="shoulder_stays_seated_at_positive_limit",
        )
    shoulder_plane_ok = (
        shoulder_neg_center is not None
        and shoulder_pos_center is not None
        and shoulder_neg_center[0] < -0.02
        and shoulder_pos_center[0] > 0.02
        and abs(shoulder_neg_center[1]) <= 1e-6
        and abs(shoulder_pos_center[1]) <= 1e-6
    )
    ctx.check(
        "shoulder_revolute_bends_in_xz_plane",
        shoulder_plane_ok,
        details=f"negative={shoulder_neg_center}, positive={shoulder_pos_center}",
    )

    with ctx.pose({elbow_hinge: -math.radians(75.0)}):
        forelink_neg_center = _aabb_center(ctx.part_world_aabb(forelink))
        ctx.expect_contact(
            forelink,
            shoulder,
            name="forelink_stays_seated_at_negative_limit",
        )
    with ctx.pose({elbow_hinge: math.radians(75.0)}):
        forelink_pos_center = _aabb_center(ctx.part_world_aabb(forelink))
        ctx.expect_contact(
            forelink,
            shoulder,
            name="forelink_stays_seated_at_positive_limit",
        )
    elbow_plane_ok = (
        forelink_neg_center is not None
        and forelink_pos_center is not None
        and forelink_neg_center[0] < -0.012
        and forelink_pos_center[0] > 0.012
        and abs(forelink_neg_center[1]) <= 1e-6
        and abs(forelink_pos_center[1]) <= 1e-6
    )
    ctx.check(
        "elbow_revolute_bends_in_same_xz_plane",
        elbow_plane_ok,
        details=f"negative={forelink_neg_center}, positive={forelink_pos_center}",
    )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
