from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
FRONT_HINGE_X = 0.095
REAR_HINGE_X = -0.095
HINGE_Z = 0.782


def _translate_profile(profile, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _scale_profile(profile, sx: float = 1.0, sy: float | None = None):
    sy = sx if sy is None else sy
    return [(x * sx, y * sy) for x, y in profile]


def _board_profile():
    control_points = [
        (-0.675, -0.188),
        (-0.480, -0.188),
        (-0.120, -0.184),
        (0.180, -0.170),
        (0.420, -0.138),
        (0.610, -0.082),
        (0.695, 0.000),
        (0.610, 0.082),
        (0.420, 0.138),
        (0.180, 0.170),
        (-0.120, 0.184),
        (-0.480, 0.188),
        (-0.675, 0.188),
    ]
    return sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=10,
        closed=True,
    )


def _board_shell_mesh():
    outer = _board_profile()
    slot_profile = rounded_rect_profile(0.130, 0.030, radius=0.010, corner_segments=10)
    slots = [
        _translate_profile(slot_profile, dx=-0.230),
        _translate_profile(slot_profile, dx=-0.030),
        _translate_profile(slot_profile, dx=0.170),
        _translate_profile(
            rounded_rect_profile(0.090, 0.026, radius=0.009, corner_segments=8), dx=0.355
        ),
    ]
    shell_geom = ExtrudeWithHolesGeometry(
        outer,
        slots,
        0.006,
        cap=True,
        center=False,
        closed=True,
    )
    shell_geom.translate(0.0, 0.0, 0.792)
    return mesh_from_geometry(shell_geom, ASSETS.mesh_path("ironing_board_shell.obj"))


def _board_cover_mesh():
    cover_profile = _scale_profile(_board_profile(), 0.988, 0.984)
    cover_geom = ExtrudeGeometry.from_z0(cover_profile, 0.018, cap=True, closed=True)
    cover_geom.translate(0.0, 0.0, 0.798)
    return mesh_from_geometry(cover_geom, ASSETS.mesh_path("ironing_board_cover.obj"))


def _leg_frame_mesh(
    *,
    foot_x: float,
    foot_z: float = -0.786,
    span_y: float = 0.305,
    top_z: float = 0.002,
    mid_x: float | None = None,
    mid_z: float = -0.400,
    tube_radius: float = 0.0095,
):
    if mid_x is None:
        mid_x = foot_x * 0.45
    half_span = span_y * 0.5
    frame_points = [
        (0.0, -half_span, top_z),
        (mid_x, -half_span, mid_z),
        (foot_x, -half_span, foot_z),
        (foot_x, half_span, foot_z),
        (mid_x, half_span, mid_z),
        (0.0, half_span, top_z),
    ]
    frame_geom = tube_from_spline_points(
        frame_points,
        radius=tube_radius,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    brace_geom = wire_from_points(
        [
            (mid_x, -half_span, mid_z),
            (mid_x, half_span, mid_z),
        ],
        radius=tube_radius * 0.80,
        radial_segments=16,
        closed_path=False,
        cap_ends=True,
        corner_mode="miter",
        up_hint=(0.0, 0.0, 1.0),
    )
    frame_geom.merge(brace_geom)
    return frame_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ironing_board", assets=ASSETS)

    board = model.part("board")
    board.visual(_board_shell_mesh(), origin=Origin())
    board.visual(_board_cover_mesh(), origin=Origin())
    board.visual(
        Box((0.155, 0.345, 0.004)),
        origin=Origin(xyz=(-0.595, 0.0, 0.794)),
    )
    board.visual(
        Box((0.090, 0.340, 0.010)),
        origin=Origin(xyz=(-0.720, 0.0, 0.805)),
    )
    board.visual(
        Cylinder(radius=0.028, length=0.335),
        origin=Origin(xyz=(0.676, 0.0, 0.807), rpy=(math.pi * 0.5, 0.0, 0.0)),
    )
    board.visual(
        Box((0.135, 0.050, 0.016)),
        origin=Origin(xyz=(0.300, 0.0, 0.800)),
    )
    board.visual(
        Box((0.080, 0.050, 0.004)),
        origin=Origin(xyz=(0.300, 0.0, 0.792)),
    )
    for hinge_x in (FRONT_HINGE_X, REAR_HINGE_X):
        board.visual(
            Box((0.048, 0.092, 0.008)),
            origin=Origin(xyz=(hinge_x, 0.0, 0.784)),
        )
    board.inertial = Inertial.from_geometry(
        Box((1.420, 0.390, 0.045)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.804)),
    )

    front_frame = model.part("front_frame")
    front_frame.visual(
        mesh_from_geometry(
            _leg_frame_mesh(
                foot_x=-0.240,
                mid_x=-0.120,
                foot_z=-0.786,
                mid_z=-0.405,
                top_z=0.002,
                tube_radius=0.0095,
            ),
            ASSETS.mesh_path("front_leg_frame.obj"),
        ),
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
    )
    front_frame.visual(
        Box((0.028, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, -0.009)),
    )
    for foot_y in (-0.1525, 0.1525):
        front_frame.visual(
            Cylinder(radius=0.014, length=0.020),
            origin=Origin(
                xyz=(-0.240, foot_y + 0.028, -0.792),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
        )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.260, 0.330, 0.800)),
        mass=1.15,
        origin=Origin(xyz=(-0.118, 0.028, -0.400)),
    )

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        mesh_from_geometry(
            _leg_frame_mesh(
                foot_x=0.240,
                mid_x=0.120,
                foot_z=-0.786,
                mid_z=-0.405,
                top_z=0.002,
                tube_radius=0.0095,
            ),
            ASSETS.mesh_path("rear_leg_frame.obj"),
        ),
        origin=Origin(xyz=(0.0, -0.028, 0.0)),
    )
    rear_frame.visual(
        Box((0.028, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, -0.009)),
    )
    for foot_y in (-0.1525, 0.1525):
        rear_frame.visual(
            Cylinder(radius=0.014, length=0.020),
            origin=Origin(
                xyz=(0.240, foot_y - 0.028, -0.792),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
        )
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.260, 0.330, 0.800)),
        mass=1.05,
        origin=Origin(xyz=(0.118, -0.028, -0.400)),
    )

    model.articulation(
        "front_frame_hinge",
        ArticulationType.REVOLUTE,
        parent="board",
        child="front_frame",
        origin=Origin(xyz=(FRONT_HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.16,
        ),
    )
    model.articulation(
        "rear_frame_hinge",
        ArticulationType.REVOLUTE,
        parent="board",
        child="rear_frame",
        origin=Origin(xyz=(REAR_HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.16,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "board",
        "front_frame",
        reason="folded front leg nest sits directly beneath the stamped underside hinge bracket",
    )
    ctx.allow_overlap(
        "board",
        "rear_frame",
        reason="folded rear leg nest sits directly beneath the stamped underside hinge bracket",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("front_frame", "board", axes="xy", min_overlap=0.14)
    ctx.expect_aabb_overlap("rear_frame", "board", axes="xy", min_overlap=0.14)
    ctx.expect_aabb_overlap("front_frame", "rear_frame", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_gap("board", "front_frame", axis="z", max_gap=0.025, max_penetration=0.008)
    ctx.expect_aabb_gap("board", "rear_frame", axis="z", max_gap=0.025, max_penetration=0.008)
    ctx.expect_origin_distance("front_frame", "rear_frame", axes="xy", max_dist=0.24)
    ctx.expect_joint_motion_axis(
        "front_frame_hinge",
        "front_frame",
        world_axis="z",
        direction="positive",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "rear_frame_hinge",
        "rear_frame",
        world_axis="z",
        direction="positive",
        min_delta=0.10,
    )

    with ctx.pose(front_frame_hinge=1.10):
        ctx.expect_aabb_overlap("front_frame", "board", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_overlap("front_frame", "rear_frame", axes="xy", min_overlap=0.08)
        ctx.expect_origin_distance("front_frame", "rear_frame", axes="xy", max_dist=0.24)

    with ctx.pose(rear_frame_hinge=1.10):
        ctx.expect_aabb_overlap("rear_frame", "board", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_overlap("front_frame", "rear_frame", axes="xy", min_overlap=0.08)
        ctx.expect_origin_distance("front_frame", "rear_frame", axes="xy", max_dist=0.24)

    with ctx.pose(front_frame_hinge=1.10, rear_frame_hinge=1.10):
        ctx.expect_aabb_overlap("front_frame", "board", axes="xy", min_overlap=0.08)
        ctx.expect_aabb_overlap("rear_frame", "board", axes="xy", min_overlap=0.08)
        ctx.expect_origin_distance("front_frame", "rear_frame", axes="xy", max_dist=0.22)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
