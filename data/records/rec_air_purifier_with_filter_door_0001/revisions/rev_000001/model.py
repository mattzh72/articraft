from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    VentGrilleGeometry,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
BODY_HEIGHT = 0.56
BODY_WIDTH = 0.32
BODY_DEPTH = 0.218


def _grille_panel_geometry(
    panel_size,
    thickness,
    *,
    frame,
    slat_pitch,
    slat_width,
    slat_angle_deg,
    corner_radius,
    center=True,
):
    geom = VentGrilleGeometry(
        panel_size,
        frame=frame,
        face_thickness=thickness,
        duct_depth=max(0.0015, thickness * 0.75),
        duct_wall=max(0.001, min(frame * 0.45, thickness * 0.75)),
        slat_pitch=slat_pitch,
        slat_width=slat_width,
        slat_angle_deg=slat_angle_deg,
        corner_radius=corner_radius,
    )
    if not center:
        geom.translate(0.0, 0.0, thickness * 0.5)
    return geom


def _profile_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=10)]


def _mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="air_purifier", assets=ASSETS)

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.96, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.78, 0.79, 0.80, 1.0))
    vent_dark = model.material("vent_dark", rgba=(0.19, 0.21, 0.23, 1.0))
    glass_black = model.material("glass_black", rgba=(0.09, 0.10, 0.11, 0.96))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    body_shell_geom = LoftGeometry(
        [
            _profile_section(0.320, 0.218, 0.036, 0.000),
            _profile_section(0.318, 0.216, 0.036, 0.025),
            _profile_section(0.308, 0.208, 0.034, 0.120),
            _profile_section(0.298, 0.200, 0.032, 0.320),
            _profile_section(0.276, 0.184, 0.028, 0.495),
            _profile_section(0.246, 0.162, 0.024, 0.560),
        ],
        cap=True,
        closed=True,
    )
    body_shell = _mesh(body_shell_geom, "body_shell.obj")

    front_intake_geom = _grille_panel_geometry(
        panel_size=(0.226, 0.300),
        thickness=0.006,
        frame=0.010,
        slat_pitch=0.018,
        slat_width=0.008,
        slat_angle_deg=28.0,
        corner_radius=0.016,
        center=False,
    )
    front_intake_mesh = _mesh(front_intake_geom, "front_intake.obj")

    top_grille_geom = _grille_panel_geometry(
        panel_size=(0.148, 0.074),
        thickness=0.008,
        frame=0.007,
        slat_pitch=0.014,
        slat_width=0.006,
        slat_angle_deg=22.0,
        corner_radius=0.012,
        center=False,
    )
    top_grille_mesh = _mesh(top_grille_geom, "top_grille.obj")

    control_pod_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.150, 0.074, 0.014, corner_segments=10),
        0.006,
        cap=True,
        closed=True,
    )
    control_pod_mesh = _mesh(control_pod_geom, "control_pod.obj")

    filter_door_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.236, 0.300, 0.018, corner_segments=10),
        0.010,
        cap=True,
        closed=True,
    )
    filter_door_geom.rotate_x(math.pi / 2.0)
    filter_door_mesh = _mesh(filter_door_geom, "filter_door.obj")

    filter_insert_geom = _grille_panel_geometry(
        panel_size=(0.176, 0.216),
        thickness=0.004,
        frame=0.008,
        slat_pitch=0.016,
        slat_width=0.007,
        slat_angle_deg=18.0,
        corner_radius=0.010,
        center=False,
    )
    filter_insert_geom.rotate_x(math.pi / 2.0)
    filter_insert_mesh = _mesh(filter_insert_geom, "filter_insert.obj")

    dial_geom = LatheGeometry(
        [
            (0.0, 0.000),
            (0.026, 0.000),
            (0.030, 0.004),
            (0.030, 0.016),
            (0.026, 0.022),
            (0.020, 0.025),
            (0.0, 0.025),
        ],
        segments=40,
    )
    selector_dial_mesh = _mesh(dial_geom, "selector_dial.obj")

    rocker_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.030, 0.050, 0.006, corner_segments=8),
        0.008,
        cap=True,
        closed=True,
    )
    power_rocker_mesh = _mesh(rocker_geom, "power_rocker.obj")

    body = model.part("body_shell")
    body.visual(body_shell, material=body_white)
    body.visual(
        Box((0.110, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, 0.103, 0.432)),
        material=glass_black,
        name="status_window",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(-0.100, -0.055, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_left_foot",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.100, -0.055, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_right_foot",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(-0.100, 0.055, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_left_foot",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.100, 0.055, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_right_foot",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.30, 0.20, BODY_HEIGHT)),
        mass=8.1,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    front_intake = model.part("front_intake")
    front_intake.visual(
        front_intake_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=vent_dark,
    )
    front_intake.inertial = Inertial.from_geometry(
        Box((0.226, 0.006, 0.300)),
        mass=0.35,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    top_grille = model.part("top_grille")
    top_grille.visual(top_grille_mesh, material=vent_dark)
    top_grille.inertial = Inertial.from_geometry(
        Box((0.148, 0.074, 0.008)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    control_pod = model.part("control_pod")
    control_pod.visual(control_pod_mesh, material=glass_black)
    control_pod.visual(
        Box((0.038, 0.058, 0.0024)),
        origin=Origin(xyz=(0.044, 0.0, 0.0052)),
        material=warm_gray,
        name="power_switch_bezel",
    )
    control_pod.inertial = Inertial.from_geometry(
        Box((0.150, 0.074, 0.006)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        filter_door_mesh,
        origin=Origin(xyz=(-0.118, 0.0, 0.0)),
        material=body_white,
    )
    filter_door.visual(
        filter_insert_mesh,
        origin=Origin(xyz=(-0.118, -0.003, 0.0)),
        material=warm_gray,
    )
    filter_door.visual(
        Box((0.010, 0.003, 0.050)),
        origin=Origin(xyz=(-0.226, -0.006, 0.078)),
        material=vent_dark,
        name="finger_pull",
    )
    filter_door.inertial = Inertial.from_geometry(
        Box((0.236, 0.010, 0.300)),
        mass=0.95,
        origin=Origin(xyz=(-0.118, -0.005, 0.0)),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(selector_dial_mesh, material=glass_black)
    selector_dial.visual(
        Cylinder(radius=0.031, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=steel,
        name="dial_trim_ring",
    )
    selector_dial.visual(
        Box((0.003, 0.012, 0.0014)),
        origin=Origin(xyz=(0.0, 0.018, 0.024)),
        material=steel,
        name="dial_indicator",
    )
    selector_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.031, length=0.025),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(power_rocker_mesh, material=vent_dark)
    power_rocker.visual(
        Box((0.004, 0.014, 0.0015)),
        origin=Origin(xyz=(0.0, 0.012, 0.0085)),
        material=steel,
        name="power_mark",
    )
    power_rocker.inertial = Inertial.from_geometry(
        Box((0.030, 0.050, 0.008)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "body_to_front_intake",
        ArticulationType.FIXED,
        parent="body_shell",
        child="front_intake",
        origin=Origin(xyz=(0.0, 0.105, 0.205)),
    )
    model.articulation(
        "body_to_top_grille",
        ArticulationType.FIXED,
        parent="body_shell",
        child="top_grille",
        origin=Origin(xyz=(0.0, -0.036, 0.5508)),
    )
    model.articulation(
        "body_to_control_pod",
        ArticulationType.FIXED,
        parent="body_shell",
        child="control_pod",
        origin=Origin(xyz=(0.0, 0.048, 0.5515)),
    )
    model.articulation(
        "body_to_filter_door",
        ArticulationType.REVOLUTE,
        parent="body_shell",
        child="filter_door",
        origin=Origin(xyz=(0.120, -0.1045, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "control_pod_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent="control_pod",
        child="selector_dial",
        origin=Origin(xyz=(-0.036, 0.0, 0.0096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )
    model.articulation(
        "control_pod_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent="control_pod",
        child="power_rocker",
        origin=Origin(xyz=(0.044, 0.0, 0.0123)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=-0.18, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_coplanar_surfaces(max_pose_samples=32)
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("front_intake", "body_shell")
    ctx.expect_aabb_overlap("front_intake", "body_shell", axes="xz", min_overlap=0.20)

    ctx.expect_aabb_contact("top_grille", "body_shell")
    ctx.expect_aabb_overlap("top_grille", "body_shell", axes="xy", min_overlap=0.06)
    ctx.expect_aabb_gap("top_grille", "body_shell", axis="z", max_gap=0.003, max_penetration=0.011)

    ctx.expect_aabb_contact("control_pod", "body_shell")
    ctx.expect_aabb_overlap("control_pod", "body_shell", axes="xy", min_overlap=0.06)

    ctx.expect_aabb_contact("filter_door", "body_shell")
    ctx.expect_aabb_overlap("filter_door", "body_shell", axes="xz", min_overlap=0.20)
    ctx.expect_joint_motion_axis(
        "body_to_filter_door",
        "filter_door",
        world_axis="y",
        direction="negative",
        min_delta=0.04,
    )
    with ctx.pose(body_to_filter_door=1.10):
        ctx.expect_aabb_overlap("filter_door", "body_shell", axes="z", min_overlap=0.24)

    ctx.expect_aabb_gap(
        "selector_dial",
        "control_pod",
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
    )
    ctx.expect_aabb_overlap("selector_dial", "control_pod", axes="xy", min_overlap=0.05)
    with ctx.pose(control_pod_to_selector_dial=1.57):
        ctx.expect_aabb_gap(
            "selector_dial",
            "control_pod",
            axis="z",
            min_gap=0.0,
            max_gap=0.004,
        )
        ctx.expect_aabb_overlap("selector_dial", "control_pod", axes="xy", min_overlap=0.05)

    ctx.expect_aabb_gap(
        "power_rocker",
        "control_pod",
        axis="z",
        min_gap=0.0,
        max_gap=0.0065,
    )
    ctx.expect_aabb_overlap("power_rocker", "control_pod", axes="xy", min_overlap=0.02)
    ctx.expect_joint_motion_axis(
        "control_pod_to_power_rocker",
        "power_rocker",
        world_axis="y",
        direction="negative",
        min_delta=0.001,
    )
    with ctx.pose(control_pod_to_power_rocker=0.18):
        ctx.expect_aabb_gap(
            "power_rocker",
            "control_pod",
            axis="z",
            min_gap=0.0,
            max_gap=0.007,
        )
    with ctx.pose(control_pod_to_power_rocker=-0.18):
        ctx.expect_aabb_gap(
            "power_rocker",
            "control_pod",
            axis="z",
            min_gap=0.0,
            max_gap=0.007,
        )

    front_pos = ctx.part_world_position("front_intake")
    top_pos = ctx.part_world_position("top_grille")
    pod_pos = ctx.part_world_position("control_pod")
    door_pos = ctx.part_world_position("filter_door")

    if front_pos[1] <= 0.09 or front_pos[2] >= 0.30:
        raise AssertionError("Front vent should read as a lower-front intake surface.")
    if top_pos[2] <= 0.55 or top_pos[1] >= 0.0:
        raise AssertionError("Top vent should sit high on the rear half of the purifier.")
    if pod_pos[2] <= 0.55 or pod_pos[1] <= 0.02:
        raise AssertionError("Controls should sit on the forward portion of the top deck.")
    if door_pos[1] >= -0.09:
        raise AssertionError("Filter access door should be mounted on the rear service face.")

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
