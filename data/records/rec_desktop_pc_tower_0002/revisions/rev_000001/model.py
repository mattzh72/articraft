from __future__ import annotations

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
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
import cadquery as cq

CASE_W = 0.235
CASE_D = 0.455
CASE_H = 0.485
SHELL_T = 0.0018
FOOT_H = 0.018

SIDE_PANEL_T = 0.004
SIDE_PANEL_DEPTH = CASE_D - 0.034
SIDE_PANEL_HEIGHT = CASE_H - 0.024

FRONT_DOOR_T = 0.004
FRONT_DOOR_W = CASE_W - 0.018
FRONT_DOOR_H = CASE_H - 0.040


def _build_chassis_visual() -> cq.Workplane:
    shell = cq.Workplane("XY").box(CASE_W, CASE_D, CASE_H)
    shell = shell.edges("|Z").fillet(0.004)
    inner = cq.Workplane("XY").box(
        CASE_W - 2 * SHELL_T,
        CASE_D - 2 * SHELL_T,
        CASE_H - 2 * SHELL_T,
    )
    shell = shell.cut(inner)

    left_window_opening = cq.Workplane("XY").box(
        0.014,
        CASE_D - 0.036,
        CASE_H - 0.032,
    )
    left_window_opening = left_window_opening.translate((-CASE_W / 2 + 0.003, 0.0, 0.0))

    front_opening = cq.Workplane("XY").box(
        CASE_W - 0.028,
        0.014,
        CASE_H - 0.052,
    )
    front_opening = front_opening.translate((0.0, -CASE_D / 2 + 0.003, 0.0))
    shell = shell.cut(left_window_opening).cut(front_opening)

    front_bracket = cq.Workplane("XY").box(
        CASE_W - 0.024,
        0.003,
        CASE_H - 0.086,
    )
    front_bracket = front_bracket.translate((0.0, -CASE_D / 2 + SHELL_T + 0.0015, 0.0))
    fan_holes = (
        cq.Workplane("XZ")
        .pushPoints([(0.0, -0.145), (0.0, 0.0), (0.0, 0.145)])
        .circle(0.063)
        .extrude(0.006)
        .translate((0.0, -CASE_D / 2 + SHELL_T - 0.0005, 0.0))
    )
    front_bracket = front_bracket.cut(fan_holes)
    shell = shell.union(front_bracket)

    psu_shroud = cq.Workplane("XY").box(
        CASE_W - 0.038,
        CASE_D * 0.48,
        0.138,
    )
    psu_shroud = psu_shroud.translate((0.0, 0.060, -CASE_H / 2 + 0.069))
    psu_shroud_window = cq.Workplane("XY").box(0.090, CASE_D * 0.20, 0.040)
    psu_shroud_window = psu_shroud_window.translate((-0.050, -0.010, -CASE_H / 2 + 0.108))
    psu_shroud = psu_shroud.cut(psu_shroud_window)
    shell = shell.union(psu_shroud)

    top_vent_points = [(-0.065 + 0.013 * i, 0.020 + 0.012 * j) for j in range(2) for i in range(11)]
    shell = (
        shell.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(top_vent_points)
        .slot2D(0.010, 0.0035, 90)
        .cutBlind(-0.006)
    )

    rear_io_cutout = cq.Workplane("XY").box(0.046, 0.012, 0.128)
    rear_io_cutout = rear_io_cutout.translate((-0.060, CASE_D / 2 - 0.003, 0.095))
    rear_fan_cutout = (
        cq.Workplane("XZ")
        .circle(0.057)
        .extrude(0.012)
        .translate((0.055, CASE_D / 2 - 0.010, 0.115))
    )
    psu_cutout = cq.Workplane("XY").box(0.146, 0.012, 0.086)
    psu_cutout = psu_cutout.translate((0.030, CASE_D / 2 - 0.003, -0.150))
    shell = shell.cut(rear_io_cutout).cut(rear_fan_cutout).cut(psu_cutout)

    expansion_slot_x = [-0.010 - 0.014 * i for i in range(7)]
    for slot_x in expansion_slot_x:
        slot = cq.Workplane("XY").box(0.011, 0.012, 0.105)
        slot = slot.translate((slot_x, CASE_D / 2 - 0.003, -0.020))
        shell = shell.cut(slot)

    top_io_pod = cq.Workplane("XY").box(0.072, 0.028, 0.010)
    top_io_pod = top_io_pod.edges("|Z").fillet(0.003)
    top_io_pod = top_io_pod.translate((0.055, -0.115, CASE_H / 2))
    shell = shell.union(top_io_pod)

    foot_centers = [
        (-CASE_W / 2 + 0.040, -CASE_D / 2 + 0.060),
        (CASE_W / 2 - 0.040, -CASE_D / 2 + 0.060),
        (-CASE_W / 2 + 0.040, CASE_D / 2 - 0.060),
        (CASE_W / 2 - 0.040, CASE_D / 2 - 0.060),
    ]
    for foot_x, foot_y in foot_centers:
        foot = cq.Workplane("XY").box(0.034, 0.026, FOOT_H)
        foot = foot.translate((foot_x, foot_y, -CASE_H / 2 - FOOT_H / 2))
        shell = shell.union(foot)

    return shell


def _build_side_panel_frame() -> cq.Workplane:
    frame = cq.Workplane("XY").box(
        SIDE_PANEL_T,
        SIDE_PANEL_DEPTH,
        SIDE_PANEL_HEIGHT,
    )
    frame = frame.translate((0.0, SIDE_PANEL_DEPTH / 2, 0.0))

    window_opening = cq.Workplane("XY").box(
        SIDE_PANEL_T * 4,
        SIDE_PANEL_DEPTH - 0.040,
        SIDE_PANEL_HEIGHT - 0.168,
    )
    window_opening = window_opening.translate((0.0, SIDE_PANEL_DEPTH / 2, 0.044))
    frame = frame.cut(window_opening)

    rear_latch_tab = cq.Workplane("XY").box(0.006, 0.025, 0.050)
    rear_latch_tab = rear_latch_tab.translate((0.0, SIDE_PANEL_DEPTH - 0.010, -0.050))
    frame = frame.union(rear_latch_tab)

    for z_center in (-0.165, 0.0, 0.165):
        hinge_barrel = cq.Workplane("XY").circle(0.006).extrude(0.040)
        hinge_barrel = hinge_barrel.translate((0.0, 0.003, z_center - 0.020))
        frame = frame.union(hinge_barrel)

    return frame


def _build_side_panel_glass() -> cq.Workplane:
    glass = cq.Workplane("XY").box(
        0.0024,
        SIDE_PANEL_DEPTH - 0.028,
        SIDE_PANEL_HEIGHT - 0.156,
    )
    glass = glass.translate((-0.0008, SIDE_PANEL_DEPTH / 2, 0.044))
    return glass


def _build_front_door_frame() -> cq.Workplane:
    frame = cq.Workplane("XY").box(FRONT_DOOR_W, FRONT_DOOR_T, FRONT_DOOR_H)
    frame = frame.translate((-FRONT_DOOR_W / 2, 0.0, 0.0))

    hinge_stile = 0.016
    free_stile = 0.026
    top_bottom_rail = 0.018
    insert_w = FRONT_DOOR_W - hinge_stile - free_stile
    insert_h = FRONT_DOOR_H - 2 * top_bottom_rail
    insert_center_x = -(hinge_stile + insert_w / 2)

    opening = cq.Workplane("XY").box(insert_w, FRONT_DOOR_T * 4, insert_h)
    opening = opening.translate((insert_center_x, 0.0, 0.0))
    frame = frame.cut(opening)

    finger_pull = cq.Workplane("XY").box(0.010, FRONT_DOOR_T * 4, 0.160)
    finger_pull = finger_pull.translate((-FRONT_DOOR_W + 0.007, 0.0, 0.000))
    frame = frame.cut(finger_pull)

    for z_center in (-0.150, 0.0, 0.150):
        hinge_barrel = cq.Workplane("XY").circle(0.0055).extrude(0.038)
        hinge_barrel = hinge_barrel.translate((0.0, 0.000, z_center - 0.019))
        frame = frame.union(hinge_barrel)

    return frame


def _build_front_door_mesh() -> cq.Workplane:
    hinge_stile = 0.016
    free_stile = 0.026
    top_bottom_rail = 0.018
    insert_w = FRONT_DOOR_W - hinge_stile - free_stile
    insert_h = FRONT_DOOR_H - 2 * top_bottom_rail
    insert_center_x = -(hinge_stile + insert_w / 2)

    mesh = cq.Workplane("XY").box(insert_w, 0.0014, insert_h)
    mesh = mesh.translate((insert_center_x, -0.0007, 0.0))

    slot_start = insert_center_x - insert_w / 2 + 0.006
    for index in range(25):
        slot = cq.Workplane("XY").box(0.0030, 0.0040, insert_h - 0.024)
        slot = slot.translate((slot_start + 0.0078 * index, -0.0007, 0.0))
        mesh = mesh.cut(slot)

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="enthusiast_pc_tower", assets=ASSETS)

    model.material("case_charcoal", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("case_mesh", rgba=(0.06, 0.07, 0.08, 1.0))
    model.material("glass_smoke", rgba=(0.52, 0.62, 0.68, 0.28))
    model.material("io_shield", rgba=(0.63, 0.64, 0.66, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        mesh_from_cadquery(
            _build_chassis_visual(),
            "pc_tower_chassis.obj",
            assets=ASSETS,
        ),
        material="case_charcoal",
    )
    chassis.visual(
        Box((0.048, 0.0012, 0.128)),
        origin=Origin(xyz=(-0.060, CASE_D / 2 - 0.0006, 0.095)),
        material="io_shield",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, CASE_H + FOOT_H)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, -FOOT_H / 2)),
    )

    side_panel = model.part("side_panel")
    side_panel.visual(
        mesh_from_cadquery(
            _build_side_panel_frame(),
            "pc_tower_side_panel_frame.obj",
            assets=ASSETS,
        ),
        material="case_charcoal",
    )
    side_panel.visual(
        mesh_from_cadquery(
            _build_side_panel_glass(),
            "pc_tower_side_panel_glass.obj",
            assets=ASSETS,
        ),
        material="glass_smoke",
    )
    side_panel.inertial = Inertial.from_geometry(
        Box((SIDE_PANEL_T, SIDE_PANEL_DEPTH, SIDE_PANEL_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(0.0, SIDE_PANEL_DEPTH / 2, 0.0)),
    )

    front_door = model.part("front_door")
    front_door.visual(
        mesh_from_cadquery(
            _build_front_door_frame(),
            "pc_tower_front_door_frame.obj",
            assets=ASSETS,
        ),
        material="case_charcoal",
    )
    front_door.visual(
        mesh_from_cadquery(
            _build_front_door_mesh(),
            "pc_tower_front_door_mesh.obj",
            assets=ASSETS,
        ),
        material="case_mesh",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((FRONT_DOOR_W, FRONT_DOOR_T, FRONT_DOOR_H)),
        mass=0.95,
        origin=Origin(xyz=(-FRONT_DOOR_W / 2, 0.0, 0.0)),
    )

    model.articulation(
        "chassis_to_side_panel",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="side_panel",
        origin=Origin(
            xyz=(-CASE_W / 2 + 0.001, -CASE_D / 2 + 0.017, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    model.articulation(
        "chassis_to_front_door",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="front_door",
        origin=Origin(
            xyz=(CASE_W / 2 - 0.012, -CASE_D / 2 + 0.002, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("side_panel", "chassis", axes="xy", min_overlap=0.002)
    ctx.expect_aabb_overlap("front_door", "chassis", axes="xy", min_overlap=0.002)

    ctx.expect_joint_motion_axis(
        "chassis_to_side_panel",
        "side_panel",
        world_axis="x",
        direction="negative",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "chassis_to_front_door",
        "front_door",
        world_axis="y",
        direction="negative",
        min_delta=0.05,
    )

    with ctx.pose(chassis_to_side_panel=1.35):
        ctx.expect_origin_distance("side_panel", "chassis", axes="xy", max_dist=0.38)

    with ctx.pose(chassis_to_front_door=1.15):
        ctx.expect_origin_distance("front_door", "chassis", axes="xy", max_dist=0.35)

    with ctx.pose(chassis_to_side_panel=0.0, chassis_to_front_door=0.0):
        ctx.expect_aabb_overlap("side_panel", "chassis", axes="xy", min_overlap=0.002)
        ctx.expect_aabb_overlap("front_door", "chassis", axes="xy", min_overlap=0.002)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
