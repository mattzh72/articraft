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
FRAME_OUTER_WIDTH = 0.42
FRAME_OUTER_HEIGHT = 0.36
FRAME_DEPTH = 0.045
SIDE_RAIL_THICKNESS = 0.028
TOP_BAR_THICKNESS = 0.03

INNER_WIDTH = FRAME_OUTER_WIDTH - 2.0 * SIDE_RAIL_THICKNESS
INNER_HEIGHT = FRAME_OUTER_HEIGHT - 2.0 * TOP_BAR_THICKNESS

BLADE_COUNT = 5
BLADE_CHORD = 0.038
BLADE_THICKNESS = 0.006
SHAFT_RADIUS = 0.004
BLADE_LIMIT = 0.95

BLADE_PITCH = INNER_HEIGHT / BLADE_COUNT
LEFT_PIVOT_X = -(INNER_WIDTH / 2.0 + SIDE_RAIL_THICKNESS / 2.0)
RIGHT_PIVOT_X = INNER_WIDTH / 2.0 + SIDE_RAIL_THICKNESS / 2.0
SHAFT_SPAN = RIGHT_PIVOT_X - LEFT_PIVOT_X

BODY_START = SIDE_RAIL_THICKNESS * 0.58
BODY_LENGTH = INNER_WIDTH - 0.002

BLADE_COLLISION_LENGTH = BODY_LENGTH - 0.006
BLADE_COLLISION_CHORD = BLADE_CHORD * 0.9
BLADE_COLLISION_THICKNESS = BLADE_THICKNESS * 0.75
BLADE_COLLISION_ORIGIN_X = BODY_START + BODY_LENGTH / 2.0
BLADE_Y_OFFSET = 0.008

BLADE_Z_OFFSETS = [INNER_HEIGHT / 2.0 - BLADE_PITCH * (index + 0.5) for index in range(BLADE_COUNT)]
BLADE_NAMES = [f"blade_{index + 1}" for index in range(BLADE_COUNT)]


def _make_frame_shape():
    outer = cq.Workplane("XY").box(FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)
    opening = cq.Workplane("XY").box(INNER_WIDTH, FRAME_DEPTH + 0.01, INNER_HEIGHT)
    return outer.cut(opening)


def _make_blade_shape():
    shaft = cq.Workplane("YZ").circle(SHAFT_RADIUS).extrude(SHAFT_SPAN)
    airfoil = (
        cq.Workplane("YZ")
        .moveTo(-BLADE_CHORD / 2.0, 0.0)
        .threePointArc(
            (-BLADE_CHORD * 0.14, BLADE_THICKNESS * 0.62),
            (BLADE_CHORD / 2.0, 0.0),
        )
        .threePointArc(
            (-BLADE_CHORD * 0.14, -BLADE_THICKNESS * 0.62),
            (-BLADE_CHORD / 2.0, 0.0),
        )
        .close()
        .extrude(BODY_LENGTH)
        .translate((BODY_START, BLADE_Y_OFFSET, 0.0))
    )
    return shaft.union(airfoil)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vane_array", assets=ASSETS)

    model.material("frame_finish", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("blade_finish", rgba=(0.78, 0.80, 0.83, 1.0))

    frame_mesh = mesh_from_cadquery(_make_frame_shape(), "frame.obj", assets=ASSETS)
    blade_mesh = mesh_from_cadquery(_make_blade_shape(), "blade.obj", assets=ASSETS)

    frame = model.part("frame")
    frame.visual(frame_mesh, material="frame_finish")




    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        mass=2.4,
    )

    for index, (blade_name, z_offset) in enumerate(zip(BLADE_NAMES, BLADE_Z_OFFSETS)):
        blade = model.part(blade_name)
        blade.visual(blade_mesh, material="blade_finish")

        blade.inertial = Inertial.from_geometry(
            Box(
                (
                    BLADE_COLLISION_LENGTH,
                    BLADE_COLLISION_CHORD,
                    BLADE_COLLISION_THICKNESS,
                )
            ),
            mass=0.14,
            origin=Origin(xyz=(BLADE_COLLISION_ORIGIN_X, BLADE_Y_OFFSET, 0.0)),
        )

        model.articulation(
            f"frame_to_blade_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=blade,
            origin=Origin(xyz=(LEFT_PIVOT_X, 0.0, z_offset)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-BLADE_LIMIT,
                upper=BLADE_LIMIT,
                effort=1.0,
                velocity=2.5,
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
        max_pose_samples=160,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    for blade_name in BLADE_NAMES:
        ctx.expect_origin_distance(blade_name, "frame", axes="xy", max_dist=0.2)
        ctx.expect_aabb_overlap(blade_name, "frame", axes="xy", min_overlap=0.012)

    for upper_blade, lower_blade in zip(BLADE_NAMES[:-1], BLADE_NAMES[1:]):
        ctx.expect_aabb_gap(upper_blade, lower_blade, axis="z", max_gap=0.06, max_penetration=0.0)

    ctx.expect_origin_gap("blade_1", "blade_5", axis="z", min_gap=0.22)

    for index, blade_name in enumerate(BLADE_NAMES, start=1):
        ctx.expect_joint_motion_axis(
            f"frame_to_blade_{index}",
            blade_name,
            world_axis="z",
            direction="positive",
            min_delta=0.01,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
