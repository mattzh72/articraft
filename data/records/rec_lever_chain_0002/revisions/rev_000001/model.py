from __future__ import annotations

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
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
BASE_PLATE_SIZE = (0.18, 0.10, 0.012)
BASE_PEDESTAL_SIZE = (0.09, 0.046, 0.028)
UPRIGHT_SIZE = (0.022, 0.008, 0.09)
UPRIGHT_OFFSET_Y = 0.015
BODY_BRACE_SIZE = (0.04, 0.016, 0.05)
BODY_BRACE_ORIGIN = (-0.02, 0.0, -0.025)
PIVOT_BRIDGE_SIZE = (0.018, 0.04, 0.02)
PIVOT_BRIDGE_ORIGIN = (-0.038, 0.0, 0.01)

WEB_HEIGHT = 0.018
WEB_THICKNESS = 0.008
CHEEK_THICKNESS = 0.0035
FORK_CHEEK_OFFSET = 0.007
BOSS_RADIUS = 0.022
HOLE_RADIUS = 0.0065
DISTAL_FORK_LENGTH = 0.048
JOINT_CLEARANCE = 0.009

LINK_SPECS = (
    ("link1", 0.18, 0.30, True),
    ("link2", 0.16, 0.27, True),
    ("link3", 0.14, 0.24, True),
    ("link4", 0.12, 0.20, False),
)


def _simple_bar_profile(length: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .sketch()
        .push([(0.0, 0.0), (length, 0.0)])
        .circle(BOSS_RADIUS)
        .reset()
        .push([(length / 2.0, 0.0)])
        .rect(length, WEB_HEIGHT)
        .reset()
        .push([(0.0, 0.0), (length, 0.0)])
        .circle(HOLE_RADIUS, mode="s")
        .clean()
        .finalize()
    )


def _forked_bar_shape(length: float) -> cq.Workplane:
    main_length = length - DISTAL_FORK_LENGTH + 0.01
    main_body = (
        cq.Workplane("XZ")
        .sketch()
        .push([(0.0, 0.0)])
        .circle(BOSS_RADIUS)
        .reset()
        .push([(main_length / 2.0, 0.0)])
        .rect(main_length, WEB_HEIGHT)
        .reset()
        .push([(0.0, 0.0)])
        .circle(HOLE_RADIUS, mode="s")
        .clean()
        .finalize()
        .extrude(WEB_THICKNESS / 2.0, both=True)
    )

    cheek_profile = (
        cq.Workplane("XZ")
        .sketch()
        .push([(length, 0.0)])
        .circle(BOSS_RADIUS)
        .reset()
        .push([(length - (DISTAL_FORK_LENGTH / 2.0), 0.0)])
        .rect(DISTAL_FORK_LENGTH, WEB_HEIGHT)
        .reset()
        .push([(length, 0.0)])
        .circle(HOLE_RADIUS, mode="s")
        .clean()
        .finalize()
    )

    left_cheek = cheek_profile.extrude(CHEEK_THICKNESS).translate(
        (0.0, FORK_CHEEK_OFFSET - (CHEEK_THICKNESS / 2.0), 0.0)
    )
    right_cheek = cheek_profile.extrude(CHEEK_THICKNESS).translate(
        (0.0, -FORK_CHEEK_OFFSET - (CHEEK_THICKNESS / 2.0), 0.0)
    )
    return main_body.union(left_cheek).union(right_cheek)


def _simple_bar_shape(length: float) -> cq.Workplane:
    return _simple_bar_profile(length).extrude(WEB_THICKNESS / 2.0, both=True)


def _base_frame_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(*BASE_PLATE_SIZE).translate((0.0, 0.0, -0.06))
    pedestal = cq.Workplane("XY").box(*BASE_PEDESTAL_SIZE).translate((-0.028, 0.0, -0.04))
    left_upright = cq.Workplane("XY").box(*UPRIGHT_SIZE).translate((0.0, UPRIGHT_OFFSET_Y, -0.015))
    right_upright = (
        cq.Workplane("XY").box(*UPRIGHT_SIZE).translate((0.0, -UPRIGHT_OFFSET_Y, -0.015))
    )
    body_brace = cq.Workplane("XY").box(*BODY_BRACE_SIZE).translate(BODY_BRACE_ORIGIN)
    pivot_bridge = cq.Workplane("XY").box(*PIVOT_BRIDGE_SIZE).translate(PIVOT_BRIDGE_ORIGIN)
    return (
        plate.union(pedestal)
        .union(left_upright)
        .union(right_upright)
        .union(body_brace)
        .union(pivot_bridge)
    )


def _add_link_part(
    model: ArticulatedObject,
    name: str,
    length: float,
    mass: float,
    has_distal_fork: bool,
):
    link = model.part(name)
    link_shape = _forked_bar_shape(length) if has_distal_fork else _simple_bar_shape(length)
    link.visual(
        mesh_from_cadquery(link_shape, f"{name}.obj", assets=ASSETS), material="lever_steel"
    )


    if has_distal_fork:
        main_start = BOSS_RADIUS + JOINT_CLEARANCE
        main_end = length - DISTAL_FORK_LENGTH + 0.01
        main_length = main_end - main_start


        cheek_start = length - DISTAL_FORK_LENGTH + 0.01
        cheek_end = length - 0.004
        cheek_length = cheek_end - cheek_start
        for cheek_y in (FORK_CHEEK_OFFSET, -FORK_CHEEK_OFFSET):
            pass
    else:
        body_start = BOSS_RADIUS + JOINT_CLEARANCE
        body_end = length - 0.004
        body_length = body_end - body_start



    link.inertial = Inertial.from_geometry(
        Box((max(length - 0.02, 0.04), 0.016, 0.03)),
        mass=mass,
        origin=Origin(xyz=(length / 2.0, 0.0, 0.0)),
    )
    return link


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serial_lever_chain", assets=ASSETS)
    model.material("frame_paint", rgba=(0.24, 0.27, 0.31, 1.0))
    model.material("lever_steel", rgba=(0.74, 0.76, 0.79, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(_base_frame_shape(), "base_frame.obj", assets=ASSETS),
        material="frame_paint",
    )






    base_frame.inertial = Inertial.from_geometry(
        Box((0.16, 0.08, 0.10)),
        mass=2.8,
        origin=Origin(xyz=(-0.01, 0.0, -0.03)),
    )

    links = []
    for name, length, mass, has_distal_fork in LINK_SPECS:
        links.append(_add_link_part(model, name, length, mass, has_distal_fork))

    base_to_link1 = model.articulation(
        "base_to_link1",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=links[0],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.6, upper=0.35, effort=14.0, velocity=1.6),
    )
    link1_to_link2 = model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=links[0],
        child=links[1],
        origin=Origin(xyz=(LINK_SPECS[0][1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.65, upper=0.55, effort=12.0, velocity=1.8),
    )
    link2_to_link3 = model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent=links[1],
        child=links[2],
        origin=Origin(xyz=(LINK_SPECS[1][1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.65, upper=0.55, effort=10.0, velocity=1.8),
    )
    link3_to_link4 = model.articulation(
        "link3_to_link4",
        ArticulationType.REVOLUTE,
        parent=links[2],
        child=links[3],
        origin=Origin(xyz=(LINK_SPECS[2][1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.65, upper=0.55, effort=8.0, velocity=2.0),
    )

    base_to_link1.meta["role"] = "root_hinge"
    link1_to_link2.meta["role"] = "serial_hinge"
    link2_to_link3.meta["role"] = "serial_hinge"
    link3_to_link4.meta["role"] = "serial_hinge"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("link1", "base_frame", axes="xy", max_dist=0.12)
    ctx.expect_origin_distance("link2", "link1", axes="xy", max_dist=0.19)
    ctx.expect_origin_distance("link3", "link2", axes="xy", max_dist=0.17)
    ctx.expect_origin_distance("link4", "link3", axes="xy", max_dist=0.15)
    ctx.expect_origin_distance("link4", "base_frame", axes="xy", max_dist=0.60)

    ctx.expect_aabb_gap("link1", "base_frame", axis="z", max_gap=0.03, max_penetration=0.05)
    ctx.expect_aabb_gap("link2", "link1", axis="z", max_gap=0.01, max_penetration=0.02)
    ctx.expect_aabb_gap("link3", "link2", axis="z", max_gap=0.01, max_penetration=0.02)
    ctx.expect_aabb_gap("link4", "link3", axis="z", max_gap=0.01, max_penetration=0.02)

    ctx.expect_joint_motion_axis(
        "base_to_link1",
        "link1",
        world_axis="z",
        direction="negative",
        min_delta=0.008,
    )
    ctx.expect_joint_motion_axis(
        "link1_to_link2",
        "link2",
        world_axis="z",
        direction="negative",
        min_delta=0.008,
    )
    ctx.expect_joint_motion_axis(
        "link2_to_link3",
        "link3",
        world_axis="z",
        direction="negative",
        min_delta=0.008,
    )
    ctx.expect_joint_motion_axis(
        "link3_to_link4",
        "link4",
        world_axis="z",
        direction="negative",
        min_delta=0.008,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
