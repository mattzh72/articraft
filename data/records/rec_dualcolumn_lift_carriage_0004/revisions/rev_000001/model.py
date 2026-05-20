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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_X = 0.30
BASE_Y = 0.18
BASE_Z = 0.04

POST_X = 0.07
POST_R = 0.014
POST_H = 0.31

BRIDGE_X = 0.19
BRIDGE_Y = 0.06
BRIDGE_Z = 0.025

SLEEVE_INNER_R = 0.0162
SLEEVE_OUTER_R = 0.024
SLEEVE_FLANGE_R = 0.027
SLEEVE_LEN = 0.06

STOP_OUTER_R = 0.0205
STOP_LOWER_Z0 = 0.055
STOP_H = 0.008
TRAVEL = 0.20
STOP_UPPER_Z0 = STOP_LOWER_Z0 + TRAVEL
LOWER_CARRIAGE_CENTER_Z = STOP_LOWER_Z0 + STOP_H + (SLEEVE_LEN * 0.5)


def _centered_ring(outer_r: float, inner_r: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
    )


def _make_bridge_shape() -> cq.Workplane:
    bridge = cq.Workplane("XY").box(BRIDGE_X, BRIDGE_Y, BRIDGE_Z, centered=(True, True, True))
    bridge_window = cq.Workplane("XY").box(0.085, BRIDGE_Y + 0.004, 0.010, centered=(True, True, True))
    return bridge.cut(bridge_window.translate((0.0, 0.0, -0.003)))


def _make_tooling_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XZ")
        .center(0.0, 0.039)
        .rect(0.235, 0.078)
        .extrude(0.012)
        .translate((0.0, 0.052, 0.0))
    )

    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.070, 0.0), (0.070, 0.0)])
        .slot2D(0.030, 0.010, 90)
        .cutThruAll()
    )

    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.095, -0.022),
                (0.095, -0.022),
                (-0.095, 0.022),
                (0.095, 0.022),
            ]
        )
        .hole(0.008)
    )
    return plate


def _make_sleeve_shape() -> cq.Workplane:
    return _centered_ring(SLEEVE_OUTER_R, SLEEVE_INNER_R, SLEEVE_LEN)


def _make_flange_shape() -> cq.Workplane:
    return _centered_ring(SLEEVE_FLANGE_R, SLEEVE_INNER_R, 0.008)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    tooling_gray = model.material("tooling_gray", rgba=(0.60, 0.62, 0.66, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.45, 0.48, 0.52, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_X, BASE_Y, BASE_Z)),
        material=dark_steel,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * BASE_Z)),
        name="base_block",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(-POST_X, 0.0, BASE_Z + 0.005)),
        material=dark_steel,
        name="left_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(POST_X, 0.0, BASE_Z + 0.005)),
        material=dark_steel,
        name="right_pedestal",
    )
    frame.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(-POST_X, 0.0, BASE_Z + (0.5 * POST_H))),
        material=satin_steel,
        name="left_post",
    )
    frame.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(POST_X, 0.0, BASE_Z + (0.5 * POST_H))),
        material=satin_steel,
        name="right_post",
    )
    frame.visual(
        mesh_from_cadquery(_make_bridge_shape(), "lift_bridge.obj", assets=ASSETS),
        material=dark_steel,
        origin=Origin(xyz=(0.0, 0.0, BASE_Z + POST_H + (0.5 * BRIDGE_Z))),
        name="top_bridge",
    )
    frame.visual(
        mesh_from_cadquery(_centered_ring(STOP_OUTER_R, POST_R, STOP_H), "left_lower_stop.obj", assets=ASSETS),
        material=dark_steel,
        origin=Origin(xyz=(-POST_X, 0.0, STOP_LOWER_Z0 + (0.5 * STOP_H))),
        name="left_lower_stop",
    )
    frame.visual(
        mesh_from_cadquery(_centered_ring(STOP_OUTER_R, POST_R, STOP_H), "right_lower_stop.obj", assets=ASSETS),
        material=dark_steel,
        origin=Origin(xyz=(POST_X, 0.0, STOP_LOWER_Z0 + (0.5 * STOP_H))),
        name="right_lower_stop",
    )
    frame.visual(
        mesh_from_cadquery(_centered_ring(STOP_OUTER_R, POST_R, STOP_H), "left_upper_stop.obj", assets=ASSETS),
        material=dark_steel,
        origin=Origin(xyz=(-POST_X, 0.0, STOP_UPPER_Z0 + (0.5 * STOP_H))),
        name="left_upper_stop",
    )
    frame.visual(
        mesh_from_cadquery(_centered_ring(STOP_OUTER_R, POST_R, STOP_H), "right_upper_stop.obj", assets=ASSETS),
        material=dark_steel,
        origin=Origin(xyz=(POST_X, 0.0, STOP_UPPER_Z0 + (0.5 * STOP_H))),
        name="right_upper_stop",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_X, BASE_Y, BASE_Z + POST_H + BRIDGE_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_Z + POST_H + BRIDGE_Z) * 0.5)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_sleeve_shape(), "left_sleeve.obj", assets=ASSETS),
        material=tooling_gray,
        origin=Origin(xyz=(-POST_X, 0.0, 0.0)),
        name="left_sleeve",
    )
    carriage.visual(
        mesh_from_cadquery(_make_sleeve_shape(), "right_sleeve.obj", assets=ASSETS),
        material=tooling_gray,
        origin=Origin(xyz=(POST_X, 0.0, 0.0)),
        name="right_sleeve",
    )
    carriage.visual(
        mesh_from_cadquery(_make_flange_shape(), "left_bottom_flange.obj", assets=ASSETS),
        material=tooling_gray,
        origin=Origin(xyz=(-POST_X, 0.0, -0.026)),
        name="left_bottom_flange",
    )
    carriage.visual(
        mesh_from_cadquery(_make_flange_shape(), "right_bottom_flange.obj", assets=ASSETS),
        material=tooling_gray,
        origin=Origin(xyz=(POST_X, 0.0, -0.026)),
        name="right_bottom_flange",
    )
    carriage.visual(
        mesh_from_cadquery(_make_flange_shape(), "left_top_flange.obj", assets=ASSETS),
        material=tooling_gray,
        origin=Origin(xyz=(-POST_X, 0.0, 0.026)),
        name="left_top_flange",
    )
    carriage.visual(
        mesh_from_cadquery(_make_flange_shape(), "right_top_flange.obj", assets=ASSETS),
        material=tooling_gray,
        origin=Origin(xyz=(POST_X, 0.0, 0.026)),
        name="right_top_flange",
    )
    carriage.visual(
        Box((0.170, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=tooling_gray,
        name="rear_tie",
    )
    carriage.visual(
        Box((0.036, 0.060, 0.022)),
        origin=Origin(xyz=(-0.036, 0.026, 0.002)),
        material=tooling_gray,
        name="left_arm",
    )
    carriage.visual(
        Box((0.036, 0.060, 0.022)),
        origin=Origin(xyz=(0.036, 0.026, 0.002)),
        material=tooling_gray,
        name="right_arm",
    )
    carriage.visual(
        Box((0.090, 0.022, 0.050)),
        origin=Origin(xyz=(0.0, 0.041, 0.031)),
        material=tooling_gray,
        name="center_rib",
    )
    carriage.visual(
        mesh_from_cadquery(_make_tooling_plate(), "tooling_plate.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=satin_steel,
        name="tooling_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.235, 0.090, 0.090)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.008, 0.030)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_CARRIAGE_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.20,
            lower=0.0,
            upper=TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("frame_to_carriage")
    tooling_plate = carriage.get_visual("tooling_plate")

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

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="carriage clears frame across stroke",
    )

    ctx.check("frame exists", frame is not None, "frame part lookup failed")
    ctx.check("carriage exists", carriage is not None, "carriage part lookup failed")
    ctx.check("slide articulation exists", slide is not None, "slide articulation lookup failed")

    ctx.expect_origin_distance(
        carriage,
        frame,
        axes="xy",
        max_dist=0.001,
        name="carriage stays centered between columns",
    )
    ctx.expect_contact(
        carriage,
        frame,
        name="carriage rests on lower hard stops",
    )
    ctx.expect_within(
        carriage,
        frame,
        axes="x",
        margin=0.0,
        name="carriage remains within base width",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="x",
        min_overlap=0.14,
        name="carriage spans both columns",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="y",
        min_overlap=0.02,
        name="carriage nests around frame depth",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    carriage_aabb = ctx.part_world_aabb(carriage)
    rest_pos = ctx.part_world_position(carriage)

    if frame_aabb is not None:
        frame_height = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame height reads as tall two-column structure",
            frame_height > 0.34,
            f"frame height too small: {frame_height:.4f} m",
        )

    if carriage_aabb is not None:
        carriage_width = carriage_aabb[1][0] - carriage_aabb[0][0]
        carriage_depth = carriage_aabb[1][1] - carriage_aabb[0][1]
        ctx.check(
            "carriage is wide enough for a tooling plate",
            carriage_width > 0.22,
            f"carriage width too small: {carriage_width:.4f} m",
        )
        ctx.check(
            "carriage has visible sleeve-plus-plate depth",
            carriage_depth > 0.07,
            f"carriage depth too small: {carriage_depth:.4f} m",
        )

    ctx.check(
        "slide stroke is about 200 mm",
        abs((slide.motion_limits.upper - slide.motion_limits.lower) - 0.20) < 1e-6,
        f"unexpected stroke: {slide.motion_limits.upper - slide.motion_limits.lower:.4f} m",
    )

    with ctx.pose({slide: TRAVEL}):
        ctx.expect_contact(
            carriage,
            frame,
            name="carriage reaches upper hard stops",
        )

        top_pos = ctx.part_world_position(carriage)
        if rest_pos is not None and top_pos is not None:
            dx = abs(top_pos[0] - rest_pos[0])
            dy = abs(top_pos[1] - rest_pos[1])
            dz = top_pos[2] - rest_pos[2]
            ctx.check(
                "prismatic motion is vertical",
                dx < 1e-6 and dy < 1e-6 and 0.199 <= dz <= 0.201,
                f"unexpected carriage delta: dx={dx:.6f}, dy={dy:.6f}, dz={dz:.6f}",
            )

        top_aabb = ctx.part_world_aabb(carriage)
        if frame_aabb is not None and top_aabb is not None:
            bridge_clearance = frame_aabb[1][2] - top_aabb[1][2]
            ctx.check(
                "carriage remains below top bridge at max travel",
                bridge_clearance > 0.004,
                f"top clearance too small: {bridge_clearance:.6f} m",
            )

    plate_aabb = ctx.part_element_world_aabb(carriage, elem=tooling_plate)
    if plate_aabb is not None:
        plate_width = plate_aabb[1][0] - plate_aabb[0][0]
        plate_height = plate_aabb[1][2] - plate_aabb[0][2]
        ctx.check(
            "tooling plate is visibly broad",
            plate_width > 0.22 and plate_height > 0.07,
            f"tooling plate dims too small: {plate_width:.4f} x {plate_height:.4f} m",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
