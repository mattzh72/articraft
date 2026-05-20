from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

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
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BEAM_RADIUS = 0.05
BEAM_Z = 2.25
JOINT_Z = BEAM_Z - BEAM_RADIUS
FRAME_HALF_SPAN = 1.05
LEG_FOOT_X = 0.82
LEG_BOTTOM_Z = 0.05
LEG_TOP_Z = 2.22
LEG_RADIUS = 0.042
CROSS_BRACE_Z = 0.92
CROSS_BRACE_RADIUS = 0.028
CHAIN_Y = 0.18
SEAT_DEPTH = 0.18
SEAT_WIDTH = 0.46
SEAT_THICKNESS = 0.028
SEAT_CENTER_Z = -1.59


def _leg_x_at_z(z: float) -> float:
    t = (z - LEG_BOTTOM_Z) / (LEG_TOP_Z - LEG_BOTTOM_Z)
    return LEG_FOOT_X * (1.0 - t)


def _add_leg(part, side_y: float, foot_x: float) -> None:
    dx = -foot_x
    dz = LEG_TOP_Z - LEG_BOTTOM_Z
    length = sqrt((dx * dx) + (dz * dz))
    angle = atan2(dx, dz)
    part.visual(
        Cylinder(radius=LEG_RADIUS, length=length),
        origin=Origin(
            xyz=(foot_x * 0.5, side_y, (LEG_TOP_Z + LEG_BOTTOM_Z) * 0.5),
            rpy=(0.0, angle, 0.0),
        ),
    )


def _make_chain_link_meshes():
    major_radius = 0.025
    link_thickness = 0.004

    yz_link = TorusGeometry(
        radius=major_radius,
        tube=link_thickness,
        radial_segments=20,
        tubular_segments=28,
    )
    yz_link.scale(1.0, 0.55, 0.55).rotate_y(pi * 0.5)

    xz_link = TorusGeometry(
        radius=major_radius,
        tube=link_thickness,
        radial_segments=20,
        tubular_segments=28,
    )
    xz_link.scale(0.55, 1.0, 0.55).rotate_x(pi * 0.5)

    return (
        mesh_from_geometry(yz_link, ASSETS.mesh_path("swing_chain_link_yz.obj")),
        mesh_from_geometry(xz_link, ASSETS.mesh_path("swing_chain_link_xz.obj")),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_swing", assets=ASSETS)

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=BEAM_RADIUS, length=2.30),
        origin=Origin(xyz=(0.0, 0.0, BEAM_Z), rpy=(pi * 0.5, 0.0, 0.0)),
    )

    beam_brace_x = _leg_x_at_z(CROSS_BRACE_Z)
    for side_y in (-FRAME_HALF_SPAN, FRAME_HALF_SPAN):
        for foot_x in (-LEG_FOOT_X, LEG_FOOT_X):
            _add_leg(frame, side_y, foot_x)
            frame.visual(
                Box((0.16, 0.10, 0.10)),
                origin=Origin(xyz=(foot_x, side_y, 0.05)),
            )

        frame.visual(
            Cylinder(radius=CROSS_BRACE_RADIUS, length=beam_brace_x * 2.0),
            origin=Origin(
                xyz=(0.0, side_y, CROSS_BRACE_Z),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
        )
        frame.visual(
            Box((0.16, 0.12, 0.11)),
            origin=Origin(xyz=(0.0, side_y, BEAM_Z - 0.03)),
        )

    for chain_y in (-CHAIN_Y, CHAIN_Y):
        frame.visual(
            Box((0.04, 0.03, 0.028)),
            origin=Origin(xyz=(0.0, chain_y, JOINT_Z + 0.014)),
        )

    frame.inertial = Inertial.from_geometry(
        Box((1.70, 2.45, 2.35)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 1.175)),
    )

    swing = model.part("swing_assembly")
    chain_link_yz, chain_link_xz = _make_chain_link_meshes()

    swing.visual(
        Cylinder(radius=0.004, length=(CHAIN_Y * 2.0) + 0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.012), rpy=(pi * 0.5, 0.0, 0.0)),
    )

    swing.visual(
        Box((SEAT_DEPTH, SEAT_WIDTH, SEAT_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, SEAT_CENTER_Z)),
    )
    swing.visual(
        Cylinder(radius=0.013, length=SEAT_WIDTH),
        origin=Origin(
            xyz=(SEAT_DEPTH * 0.5, 0.0, SEAT_CENTER_Z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
    )
    swing.visual(
        Cylinder(radius=0.013, length=SEAT_WIDTH),
        origin=Origin(
            xyz=(-SEAT_DEPTH * 0.5, 0.0, SEAT_CENTER_Z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
    )
    swing.visual(
        Box((0.10, SEAT_WIDTH - 0.05, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, SEAT_CENTER_Z - 0.018)),
    )

    link_start_z = -0.095
    link_pitch = 0.066
    link_count = 22
    chain_center_z = -0.79
    chain_length = 1.49
    chain_backbone_length = 1.54

    for side_y in (-CHAIN_Y, CHAIN_Y):
        swing.visual(
            Cylinder(radius=0.004, length=0.028),
            origin=Origin(xyz=(0.0, side_y, -0.026)),
        )
        swing.visual(
            Box((0.012, 0.010, chain_backbone_length)),
            origin=Origin(xyz=(0.0, side_y, -0.79)),
        )
        swing.visual(
            Cylinder(radius=0.004, length=0.06),
            origin=Origin(xyz=(0.0, side_y, -1.505)),
        )
        swing.visual(
            Box((0.02, 0.03, 0.08)),
            origin=Origin(xyz=(0.0, side_y, -1.54)),
        )

        for idx in range(link_count):
            swing.visual(
                chain_link_yz if idx % 2 == 0 else chain_link_xz,
                origin=Origin(xyz=(0.0, side_y, link_start_z - (idx * link_pitch))),
            )

    swing.inertial = Inertial.from_geometry(
        Box((0.24, 0.52, 1.66)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, -0.83)),
    )

    model.articulation(
        "swing_hinge",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="swing_assembly",
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=2.2,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        "swing_hinge",
        "swing_assembly",
        world_axis="x",
        direction="negative",
        min_delta=0.20,
    )

    with ctx.pose(swing_hinge=0.0):
        ctx.expect_origin_distance("swing_assembly", "frame", axes="xy", max_dist=0.03)
        ctx.expect_aabb_overlap("swing_assembly", "frame", axes="xy", min_overlap=0.18)

    with ctx.pose(swing_hinge=0.65):
        ctx.expect_aabb_overlap("swing_assembly", "frame", axes="xy", min_overlap=0.18)

    with ctx.pose(swing_hinge=-0.65):
        ctx.expect_aabb_overlap("swing_assembly", "frame", axes="xy", min_overlap=0.18)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
