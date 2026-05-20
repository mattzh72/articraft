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
MODEL_NAME = "layered_slat_mechanism"
FRAME_WIDTH = 0.42
FRAME_HEIGHT = 0.56
RAIL_THICKNESS = 0.03
RAIL_DEPTH = 0.025
BAR_HEIGHT = 0.03
INNER_WIDTH = FRAME_WIDTH - 2.0 * RAIL_THICKNESS
BEARING_SPAN = FRAME_WIDTH - RAIL_THICKNESS

SLAT_COUNT = 6
SLAT_PITCH = 0.075
SLAT_CHORD = 0.055
SLAT_THICKNESS = 0.008
SLAT_AXIS_OFFSET_Y = 0.012
JOURNAL_RADIUS = 0.0045
JOURNAL_LENGTH = 0.018
SLAT_LIMIT = 0.82

LEFT_BEARING_X = -FRAME_WIDTH * 0.5 + RAIL_THICKNESS * 0.5
SLAT_ZS = tuple((((SLAT_COUNT - 1) * 0.5) - idx) * SLAT_PITCH for idx in range(SLAT_COUNT))
SLAT_NAMES = tuple(f"slat_{idx}" for idx in range(SLAT_COUNT))
JOINT_NAMES = tuple(f"{name}_hinge" for name in SLAT_NAMES)


def _make_frame_shape() -> cq.Workplane:
    left_rail = (
        cq.Workplane("XY")
        .box(RAIL_THICKNESS, RAIL_DEPTH, FRAME_HEIGHT)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(RAIL_DEPTH * 0.72, FRAME_HEIGHT - 2.0 * BAR_HEIGHT - 0.02)
        .cutBlind(-RAIL_THICKNESS * 0.22)
        .translate((-FRAME_WIDTH * 0.5 + RAIL_THICKNESS * 0.5, 0.0, 0.0))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(RAIL_THICKNESS, RAIL_DEPTH, FRAME_HEIGHT)
        .faces("<X")
        .workplane(centerOption="CenterOfMass")
        .rect(RAIL_DEPTH * 0.72, FRAME_HEIGHT - 2.0 * BAR_HEIGHT - 0.02)
        .cutBlind(-RAIL_THICKNESS * 0.22)
        .translate((FRAME_WIDTH * 0.5 - RAIL_THICKNESS * 0.5, 0.0, 0.0))
    )
    top_bar = (
        cq.Workplane("XY")
        .box(INNER_WIDTH, RAIL_DEPTH * 0.92, BAR_HEIGHT)
        .translate((0.0, 0.0, FRAME_HEIGHT * 0.5 - BAR_HEIGHT * 0.5))
    )
    bottom_bar = (
        cq.Workplane("XY")
        .box(INNER_WIDTH, RAIL_DEPTH * 0.92, BAR_HEIGHT)
        .translate((0.0, 0.0, -FRAME_HEIGHT * 0.5 + BAR_HEIGHT * 0.5))
    )
    return left_rail.union(right_rail).union(top_bar).union(bottom_bar)


def _make_slat_shape() -> cq.Workplane:
    blade = (
        cq.Workplane("YZ")
        .ellipse(SLAT_CHORD * 0.5, SLAT_THICKNESS * 0.5)
        .extrude(INNER_WIDTH)
        .translate((RAIL_THICKNESS * 0.5, SLAT_AXIS_OFFSET_Y, 0.0))
    )
    left_journal = (
        cq.Workplane("YZ")
        .circle(JOURNAL_RADIUS)
        .extrude(JOURNAL_LENGTH)
        .translate((-JOURNAL_LENGTH * 0.5, 0.0, 0.0))
    )
    right_journal = (
        cq.Workplane("YZ")
        .circle(JOURNAL_RADIUS)
        .extrude(JOURNAL_LENGTH)
        .translate((BEARING_SPAN - JOURNAL_LENGTH * 0.5, 0.0, 0.0))
    )
    return blade.union(left_journal).union(right_journal)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name=MODEL_NAME, assets=ASSETS)
    model.material("rail_finish", rgba=(0.23, 0.24, 0.27, 1.0))
    model.material("slat_finish", rgba=(0.87, 0.89, 0.91, 1.0))

    frame_mesh = mesh_from_cadquery(_make_frame_shape(), "frame.obj", assets=ASSETS)
    slat_mesh = mesh_from_cadquery(_make_slat_shape(), "slat.obj", assets=ASSETS)

    frame = model.part("frame")
    frame.visual(frame_mesh, material="rail_finish")

    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, RAIL_DEPTH, FRAME_HEIGHT)),
        mass=2.8,
    )

    for index, (slat_name, slat_z) in enumerate(zip(SLAT_NAMES, SLAT_ZS)):
        slat = model.part(slat_name)
        slat.visual(slat_mesh, material="slat_finish")
        slat.inertial = Inertial.from_geometry(
            Box((INNER_WIDTH, SLAT_CHORD * 0.9, SLAT_THICKNESS * 0.85)),
            mass=0.18,
            origin=Origin(xyz=(RAIL_THICKNESS * 0.5 + INNER_WIDTH * 0.5, SLAT_AXIS_OFFSET_Y, 0.0)),
        )

        model.articulation(
            f"{slat_name}_hinge",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(LEFT_BEARING_X, 0.0, slat_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-SLAT_LIMIT,
                upper=SLAT_LIMIT,
                effort=2.0,
                velocity=2.5,
            ),
            meta={"index": index},
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)

    for slat_name in SLAT_NAMES:
        ctx.allow_overlap(
            "frame",
            slat_name,
            reason="Journal ends are intentionally seated inside the simplified side-rail collision bodies.",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)

    for slat_name, joint_name in zip(SLAT_NAMES, JOINT_NAMES):
        ctx.expect_joint_motion_axis(
            joint_name,
            slat_name,
            world_axis="z",
            direction="positive",
            min_delta=0.01,
        )
        ctx.expect_aabb_overlap(slat_name, "frame", axes="xy", min_overlap=0.01)

    for upper_slat, lower_slat in zip(SLAT_NAMES, SLAT_NAMES[1:]):
        ctx.expect_aabb_gap(upper_slat, lower_slat, axis="z", max_gap=0.07, max_penetration=0.0)

    with ctx.pose({joint_name: SLAT_LIMIT for joint_name in JOINT_NAMES}):
        for upper_slat, lower_slat in zip(SLAT_NAMES, SLAT_NAMES[1:]):
            ctx.expect_aabb_gap(upper_slat, lower_slat, axis="z", max_gap=0.055, max_penetration=0.001)
        for slat_name in SLAT_NAMES:
            ctx.expect_aabb_overlap(slat_name, "frame", axes="xy", min_overlap=0.01)

    with ctx.pose({joint_name: -SLAT_LIMIT for joint_name in JOINT_NAMES}):
        for upper_slat, lower_slat in zip(SLAT_NAMES, SLAT_NAMES[1:]):
            ctx.expect_aabb_gap(upper_slat, lower_slat, axis="z", max_gap=0.055, max_penetration=0.001)

    with ctx.pose(
        {
            joint_name: (SLAT_LIMIT if index % 2 == 0 else -SLAT_LIMIT)
            for index, joint_name in enumerate(JOINT_NAMES)
        }
    ):
        for upper_slat, lower_slat in zip(SLAT_NAMES, SLAT_NAMES[1:]):
            ctx.expect_aabb_gap(upper_slat, lower_slat, axis="z", max_gap=0.055, max_penetration=0.001)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
