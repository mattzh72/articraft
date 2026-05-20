from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BRIDGE_LENGTH = 0.240
BRIDGE_DEPTH = 0.048
BRIDGE_HEIGHT = 0.028

FRAME_THICKNESS = 0.020
FRAME_DEPTH = 0.050
FRAME_HEIGHT = 0.200
FRAME_BOTTOM_Z = -0.180
FRAME_FOOT_WIDTH = 0.038
FRAME_FOOT_DEPTH = 0.070
FRAME_FOOT_HEIGHT = 0.012
FRAME_WINDOW_DEPTH = 0.028
FRAME_WINDOW_HEIGHT = 0.132
FRAME_WINDOW_BOTTOM = -0.145

TRUCK_LENGTH = 0.040
TRUCK_TOP_WIDTH = 0.030
TRUCK_OUTER_WIDTH = 0.052
TRUCK_SIDE_WIDTH = 0.006
TRUCK_BOTTOM_Z = -0.048

QUILL_PLATE_WIDTH = 0.028
QUILL_PLATE_THICKNESS = 0.012
QUILL_PLATE_LENGTH = 0.096
QUILL_SHOE_WIDTH = 0.034
QUILL_SHOE_DEPTH = 0.028
QUILL_SHOE_HEIGHT = 0.018

TRUCK_TRAVEL = 0.180
QUILL_TRAVEL = 0.120


def _left_frame_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(FRAME_THICKNESS, FRAME_DEPTH, FRAME_HEIGHT, centered=(True, True, False))
        .translate((-FRAME_THICKNESS / 2.0, 0.0, FRAME_BOTTOM_Z))
    )
    window = (
        cq.Workplane("XY")
        .box(FRAME_THICKNESS * 1.5, FRAME_WINDOW_DEPTH, FRAME_WINDOW_HEIGHT, centered=(True, True, False))
        .translate((-FRAME_THICKNESS / 2.0, 0.0, FRAME_WINDOW_BOTTOM))
    )
    foot = (
        cq.Workplane("XY")
        .box(FRAME_FOOT_WIDTH, FRAME_FOOT_DEPTH, FRAME_FOOT_HEIGHT, centered=(True, True, False))
        .translate((-FRAME_FOOT_WIDTH / 2.0, 0.0, FRAME_BOTTOM_Z - FRAME_FOOT_HEIGHT))
    )
    brace = (
        cq.Workplane("XY")
        .box(0.010, FRAME_DEPTH * 0.92, 0.038, centered=(True, True, False))
        .translate((-0.015, 0.0, -0.055))
    )
    return plate.cut(window).union(foot).union(brace)


def _right_frame_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(FRAME_THICKNESS, FRAME_DEPTH, FRAME_HEIGHT, centered=(True, True, False))
        .translate((FRAME_THICKNESS / 2.0, 0.0, FRAME_BOTTOM_Z))
    )
    window = (
        cq.Workplane("XY")
        .box(FRAME_THICKNESS * 1.5, FRAME_WINDOW_DEPTH, FRAME_WINDOW_HEIGHT, centered=(True, True, False))
        .translate((FRAME_THICKNESS / 2.0, 0.0, FRAME_WINDOW_BOTTOM))
    )
    foot = (
        cq.Workplane("XY")
        .box(FRAME_FOOT_WIDTH, FRAME_FOOT_DEPTH, FRAME_FOOT_HEIGHT, centered=(True, True, False))
        .translate((FRAME_FOOT_WIDTH / 2.0, 0.0, FRAME_BOTTOM_Z - FRAME_FOOT_HEIGHT))
    )
    brace = (
        cq.Workplane("XY")
        .box(0.010, FRAME_DEPTH * 0.92, 0.038, centered=(True, True, False))
        .translate((0.015, 0.0, -0.055))
    )
    return plate.cut(window).union(foot).union(brace)


def _bridge_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(BRIDGE_LENGTH, BRIDGE_DEPTH, BRIDGE_HEIGHT)


def _truck_shape() -> cq.Workplane:
    top_skid = (
        cq.Workplane("XY")
        .box(TRUCK_LENGTH, TRUCK_TOP_WIDTH, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, -0.006))
    )
    side_offset = TRUCK_OUTER_WIDTH / 2.0 - TRUCK_SIDE_WIDTH / 2.0
    left_side = (
        cq.Workplane("XY")
        .box(TRUCK_LENGTH, TRUCK_SIDE_WIDTH, 0.022, centered=(True, True, False))
        .translate((0.0, -side_offset, -0.028))
    )
    right_side = (
        cq.Workplane("XY")
        .box(TRUCK_LENGTH, TRUCK_SIDE_WIDTH, 0.022, centered=(True, True, False))
        .translate((0.0, side_offset, -0.028))
    )
    lower_tie = (
        cq.Workplane("XY")
        .box(0.032, 0.036, 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, -0.038))
    )
    quill_mount = (
        cq.Workplane("XY")
        .box(0.022, 0.018, 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, TRUCK_BOTTOM_Z))
    )
    return top_skid.union(left_side).union(right_side).union(lower_tie).union(quill_mount)


def _quill_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(QUILL_PLATE_WIDTH, QUILL_PLATE_THICKNESS, QUILL_PLATE_LENGTH, centered=(True, True, False))
        .translate((0.0, 0.0, -QUILL_PLATE_LENGTH))
    )
    shoe = (
        cq.Workplane("XY")
        .box(QUILL_SHOE_WIDTH, QUILL_SHOE_DEPTH, QUILL_SHOE_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, -(QUILL_PLATE_LENGTH + QUILL_SHOE_HEIGHT)))
    )
    nose = (
        cq.Workplane("XZ")
        .circle(0.005)
        .extrude(0.018, both=True)
        .translate((0.0, 0.020, -(QUILL_PLATE_LENGTH + QUILL_SHOE_HEIGHT * 0.55)))
    )
    return plate.union(shoe).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry_module", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.76, 0.78, 0.80, 1.0))
    bridge_paint = model.material("bridge_paint", rgba=(0.52, 0.56, 0.60, 1.0))
    truck_paint = model.material("truck_paint", rgba=(0.78, 0.47, 0.20, 1.0))
    quill_paint = model.material("quill_paint", rgba=(0.28, 0.31, 0.35, 1.0))

    bridge = model.part("bridge")
    bridge.visual(
        mesh_from_cadquery(_bridge_shape(), "bridge.obj", assets=ASSETS),
        name="bridge_shell",
        material=bridge_paint,
    )
    bridge.inertial = Inertial.from_geometry(
        Box((BRIDGE_LENGTH, BRIDGE_DEPTH, BRIDGE_HEIGHT + 0.006)),
        mass=2.8,
    )

    left_frame = model.part("left_frame")
    left_frame.visual(
        mesh_from_cadquery(_left_frame_shape(), "left_frame.obj", assets=ASSETS),
        name="left_frame_shell",
        material=frame_paint,
    )
    left_frame.inertial = Inertial.from_geometry(
        Box((FRAME_FOOT_WIDTH, FRAME_FOOT_DEPTH, FRAME_HEIGHT + FRAME_FOOT_HEIGHT)),
        mass=2.0,
        origin=Origin(xyz=(-FRAME_FOOT_WIDTH / 2.0, 0.0, FRAME_BOTTOM_Z + (FRAME_HEIGHT + FRAME_FOOT_HEIGHT) / 2.0)),
    )

    right_frame = model.part("right_frame")
    right_frame.visual(
        mesh_from_cadquery(_right_frame_shape(), "right_frame.obj", assets=ASSETS),
        name="right_frame_shell",
        material=frame_paint,
    )
    right_frame.inertial = Inertial.from_geometry(
        Box((FRAME_FOOT_WIDTH, FRAME_FOOT_DEPTH, FRAME_HEIGHT + FRAME_FOOT_HEIGHT)),
        mass=2.0,
        origin=Origin(xyz=(FRAME_FOOT_WIDTH / 2.0, 0.0, FRAME_BOTTOM_Z + (FRAME_HEIGHT + FRAME_FOOT_HEIGHT) / 2.0)),
    )

    truck = model.part("truck")
    truck.visual(
        mesh_from_cadquery(_truck_shape(), "truck.obj", assets=ASSETS),
        name="truck_shell",
        material=truck_paint,
    )
    truck.inertial = Inertial.from_geometry(
        Box((TRUCK_LENGTH, TRUCK_OUTER_WIDTH, -TRUCK_BOTTOM_Z)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, TRUCK_BOTTOM_Z / 2.0)),
    )

    quill = model.part("quill")
    quill.visual(
        mesh_from_cadquery(_quill_shape(), "quill.obj", assets=ASSETS),
        name="quill_shell",
        material=quill_paint,
    )
    quill.inertial = Inertial.from_geometry(
        Box((QUILL_SHOE_WIDTH, QUILL_SHOE_DEPTH, QUILL_PLATE_LENGTH + QUILL_SHOE_HEIGHT)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -(QUILL_PLATE_LENGTH + QUILL_SHOE_HEIGHT) / 2.0)),
    )

    model.articulation(
        "bridge_to_left_frame",
        ArticulationType.FIXED,
        parent=bridge,
        child=left_frame,
        origin=Origin(xyz=(-BRIDGE_LENGTH / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "bridge_to_right_frame",
        ArticulationType.FIXED,
        parent=bridge,
        child=right_frame,
        origin=Origin(xyz=(BRIDGE_LENGTH / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(0.0, 0.0, -BRIDGE_HEIGHT / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=-TRUCK_TRAVEL / 2.0,
            upper=TRUCK_TRAVEL / 2.0,
        ),
    )
    model.articulation(
        "truck_to_quill",
        ArticulationType.PRISMATIC,
        parent=truck,
        child=quill,
        origin=Origin(xyz=(0.0, 0.0, TRUCK_BOTTOM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.18,
            lower=0.0,
            upper=QUILL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bridge = object_model.get_part("bridge")
    left_frame = object_model.get_part("left_frame")
    right_frame = object_model.get_part("right_frame")
    truck = object_model.get_part("truck")
    quill = object_model.get_part("quill")
    truck_slide = object_model.get_articulation("bridge_to_truck")
    quill_slide = object_model.get_articulation("truck_to_quill")

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

    ctx.check(
        "expected_part_count",
        len(object_model.parts) == 5,
        f"expected 5 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "expected_joint_count",
        len(object_model.articulations) == 4,
        f"expected 4 articulations, found {len(object_model.articulations)}",
    )

    ctx.expect_contact(bridge, left_frame, name="left_frame_supports_bridge")
    ctx.expect_contact(bridge, right_frame, name="right_frame_supports_bridge")
    ctx.expect_overlap(bridge, left_frame, axes="yz", min_overlap=0.025, name="left_frame_has_bridge_seat")
    ctx.expect_overlap(bridge, right_frame, axes="yz", min_overlap=0.025, name="right_frame_has_bridge_seat")

    ctx.expect_contact(truck, bridge, name="truck_rides_under_bridge")
    ctx.expect_overlap(truck, bridge, axes="xy", min_overlap=0.030, name="truck_stays_under_bridge_footprint")
    ctx.expect_contact(truck, quill, name="quill_hangs_from_truck_at_home")
    ctx.expect_origin_distance(truck, quill, axes="xy", max_dist=0.001, name="quill_is_centered_under_truck")

    with ctx.pose({truck_slide: -TRUCK_TRAVEL / 2.0}):
        ctx.expect_contact(truck, bridge, name="truck_contacts_bridge_at_left_extreme")
        ctx.expect_origin_distance(
            truck,
            bridge,
            axes="x",
            min_dist=TRUCK_TRAVEL / 2.0 - 0.001,
            max_dist=TRUCK_TRAVEL / 2.0 + 0.001,
            name="truck_reaches_left_span_limit",
        )
        ctx.expect_origin_distance(truck, quill, axes="xy", max_dist=0.001, name="quill_tracks_truck_left")

    with ctx.pose({truck_slide: TRUCK_TRAVEL / 2.0, quill_slide: QUILL_TRAVEL}):
        ctx.expect_contact(truck, bridge, name="truck_contacts_bridge_at_right_extreme")
        ctx.expect_origin_distance(
            truck,
            bridge,
            axes="x",
            min_dist=TRUCK_TRAVEL / 2.0 - 0.001,
            max_dist=TRUCK_TRAVEL / 2.0 + 0.001,
            name="truck_reaches_right_span_limit",
        )
        ctx.expect_origin_gap(
            truck,
            quill,
            axis="z",
            min_gap=0.167,
            max_gap=0.169,
            name="quill_extends_about_120mm_down",
        )
        ctx.expect_origin_distance(quill, truck, axes="xy", max_dist=0.001, name="quill_stays_aligned_when_extended")

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
