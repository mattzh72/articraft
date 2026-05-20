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
PANEL_WIDTH = 0.32
PANEL_HEIGHT = 0.52
PANEL_DEPTH = 0.035
FRAME_BORDER = 0.022

FLAP_COUNT = 4
OPENING_WIDTH = PANEL_WIDTH - 2.0 * FRAME_BORDER
OPENING_HEIGHT = PANEL_HEIGHT - 2.0 * FRAME_BORDER
FLAP_PITCH = OPENING_HEIGHT / FLAP_COUNT
FLAP_GAP = 0.016

BRACE_DEPTH = 0.006
BRACE_HEIGHT = 0.010
BRACE_Y = -0.007

FLAP_WIDTH = OPENING_WIDTH - 0.012
FLAP_HEIGHT = FLAP_PITCH - FLAP_GAP
FLAP_PLATE_DEPTH = 0.006
FLAP_PLATE_Y = 0.0045
FLAP_PLATE_CENTER_Z = -FLAP_HEIGHT / 2.0 - 0.001
FLAP_BARREL_RADIUS = 0.006
FLAP_BARREL_LENGTH = FLAP_WIDTH - 0.016
FLAP_OPEN_LIMIT = 1.2


def _hinge_z(index: int) -> float:
    return PANEL_HEIGHT / 2.0 - FRAME_BORDER - FLAP_GAP - index * FLAP_PITCH


def _make_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(PANEL_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)
    opening = cq.Workplane("XY").box(
        OPENING_WIDTH,
        PANEL_DEPTH + 0.004,
        OPENING_HEIGHT,
    )
    frame = frame.cut(opening)

    for index in range(FLAP_COUNT):
        brace = (
            cq.Workplane("XY")
            .box(OPENING_WIDTH, BRACE_DEPTH, BRACE_HEIGHT)
            .translate((0.0, BRACE_Y, _hinge_z(index)))
        )
        frame = frame.union(brace)

    return frame


def _make_flap_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("YZ").circle(FLAP_BARREL_RADIUS).extrude(FLAP_BARREL_LENGTH / 2.0, both=True)
    )
    plate = (
        cq.Workplane("XY")
        .box(FLAP_WIDTH, FLAP_PLATE_DEPTH, FLAP_HEIGHT)
        .translate((0.0, FLAP_PLATE_Y, FLAP_PLATE_CENTER_Z))
    )
    return barrel.union(plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_flap_panel", assets=ASSETS)

    model.material("frame_finish", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("flap_finish", rgba=(0.78, 0.80, 0.83, 1.0))

    frame_shape = _make_frame_shape()
    flap_shape = _make_flap_shape()
    frame_mesh = mesh_from_cadquery(frame_shape, "panel_frame.obj", assets=ASSETS)
    flap_mesh = mesh_from_cadquery(flap_shape, "panel_flap.obj", assets=ASSETS)

    frame = model.part("frame")
    frame.visual(frame_mesh, material="frame_finish")
    frame.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)),
        mass=4.2,
    )

    side_rail_x = OPENING_WIDTH / 2.0 + FRAME_BORDER / 2.0
    top_rail_z = OPENING_HEIGHT / 2.0 + FRAME_BORDER / 2.0





    for index in range(FLAP_COUNT):
        pass

    for index in range(FLAP_COUNT):
        flap_name = f"flap_{index + 1}"
        joint_name = f"frame_to_flap_{index + 1}"
        flap = model.part(flap_name)
        flap.visual(flap_mesh, material="flap_finish")


        flap.inertial = Inertial.from_geometry(
            Box((FLAP_WIDTH, FLAP_PLATE_DEPTH, FLAP_HEIGHT)),
            mass=0.28,
            origin=Origin(xyz=(0.0, FLAP_PLATE_Y, FLAP_PLATE_CENTER_Z)),
        )

        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=flap,
            origin=Origin(xyz=(0.0, 0.0, _hinge_z(index))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=0.0,
                upper=FLAP_OPEN_LIMIT,
                effort=3.0,
                velocity=1.5,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )

    flap_names = [f"flap_{index + 1}" for index in range(FLAP_COUNT)]
    joint_names = [f"frame_to_flap_{index + 1}" for index in range(FLAP_COUNT)]

    for flap_name, joint_name in zip(flap_names, joint_names):
        ctx.expect_origin_distance(flap_name, "frame", axes="xy", max_dist=0.01)
        ctx.expect_aabb_overlap(flap_name, "frame", axes="xy", min_overlap=0.004)
        ctx.expect_joint_motion_axis(
            joint_name,
            flap_name,
            world_axis="y",
            direction="positive",
            min_delta=0.01,
        )

    for upper_flap, lower_flap in zip(flap_names, flap_names[1:]):
        ctx.expect_aabb_gap(upper_flap, lower_flap, axis="z", max_gap=0.02, max_penetration=0.0)

    joint_positions = []
    for index, joint_name in enumerate(joint_names):
        articulation = object_model.get_articulation(joint_name)
        joint_positions.append(articulation.origin.xyz[2])
        assert tuple(articulation.axis) == (1.0, 0.0, 0.0)
        assert abs(articulation.origin.xyz[2] - _hinge_z(index)) < 1e-9

    for upper_z, lower_z in zip(joint_positions, joint_positions[1:]):
        assert abs((upper_z - lower_z) - FLAP_PITCH) < 1e-9

    assert len(object_model.parts) == FLAP_COUNT + 1
    assert len(object_model.articulations) == FLAP_COUNT

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
