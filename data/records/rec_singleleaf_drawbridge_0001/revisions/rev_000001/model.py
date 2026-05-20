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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _origin(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    return Origin(xyz=xyz, rpy=rpy)


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=_origin(xyz, rpy))


def _add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=_origin(xyz, rpy))


def _add_brace(
    part,
    y: float,
    start: tuple[float, float],
    end: tuple[float, float],
    thickness: float = 0.05,
) -> None:
    dx = end[0] - start[0]
    dz = end[1] - start[1]
    length = math.hypot(dx, dz)
    angle = -math.atan2(dz, dx)
    _add_box(
        part,
        (length, 0.06, thickness),
        ((start[0] + end[0]) * 0.5, y, (start[1] + end[1]) * 0.5),
        (0.0, angle, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge", assets=ASSETS)

    support = model.part("support_frame")
    _add_box(support, (2.75, 4.60, 0.75), (-1.425, 0.0, 0.375))
    _add_box(support, (0.90, 4.00, 0.95), (-1.95, 0.0, 1.125))
    _add_box(support, (0.40, 2.60, 0.18), (-0.20, 0.0, 0.495))
    for y in (-1.70, 1.70):
        _add_box(support, (1.00, 0.34, 1.55), (-0.55, y, 1.025))
        _add_box(support, (0.32, 0.30, 0.40), (-0.15, y * 0.482, 1.25))
        _add_cylinder(
            support,
            radius=0.16,
            length=0.28,
            xyz=(-0.12, y * 0.482, 1.25),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        )
        _add_box(support, (0.62, 0.12, 0.14), (-0.41, y * 0.482, 1.25))
        _add_box(support, (0.20, 0.90, 0.14), (-0.48, y * 0.64, 1.25))
        _add_box(support, (0.42, 0.46, 0.46), (-0.48, y * 0.90, 0.93))
    _add_box(support, (0.60, 3.10, 0.18), (-0.55, 0.0, 1.71))
    _add_box(support, (0.11, 2.60, 0.18), (-0.06, 0.0, 1.25))
    _add_box(support, (0.90, 1.10, 0.70), (-1.95, 1.15, 1.40))
    _add_box(support, (1.00, 1.20, 0.10), (-1.95, 1.15, 1.78))
    _add_box(support, (0.80, 0.90, 0.06), (-1.45, -1.15, 1.08))
    _add_box(support, (0.24, 0.22, 0.18), (-0.82, -1.52, 1.30))
    _add_cylinder(
        support,
        radius=0.16,
        length=0.22,
        xyz=(-0.95, -1.34, 1.30),
        rpy=(math.pi * 0.5, 0.0, 0.0),
    )
    support.inertial = Inertial.from_geometry(
        Box((2.75, 4.60, 0.75)),
        mass=18000.0,
        origin=_origin((-1.425, 0.0, 0.375)),
    )

    bridge_seat = model.part("bridge_seat")
    _add_box(bridge_seat, (0.72, 2.30, 0.20), (0.36, 0.0, 0.10))
    for y in (-0.78, 0.78):
        _add_box(bridge_seat, (0.44, 0.18, 0.02), (0.28, y, 0.19))
    _add_box(bridge_seat, (0.10, 2.10, 0.10), (0.67, 0.0, 0.11))
    bridge_seat.inertial = Inertial.from_geometry(
        Box((0.72, 2.30, 0.20)),
        mass=1200.0,
        origin=_origin((0.36, 0.0, 0.10)),
    )

    leaf = model.part("bridge_leaf")
    _add_box(leaf, (5.45, 3.00, 0.17), (2.725, 0.0, -0.375))
    _add_box(leaf, (5.05, 2.30, 0.025), (2.925, 0.0, -0.2775))
    for y in (-1.42, 1.42):
        _add_box(leaf, (5.20, 0.16, 0.18), (2.85, y, -0.27))
    _add_box(leaf, (0.85, 1.34, 0.22), (0.44, 0.0, -0.25))
    _add_box(leaf, (0.16, 1.20, 0.32), (0.08, 0.0, -0.16))
    _add_box(leaf, (0.10, 0.80, 0.14), (0.055, 0.0, -0.07))
    for y in (-0.82, 0.82):
        _add_box(leaf, (0.28, 0.30, 0.30), (0.20, y, -0.13))
        _add_cylinder(
            leaf,
            radius=0.11,
            length=0.28,
            xyz=(0.18, y, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        )
        _add_box(leaf, (4.35, 0.18, 0.48), (3.125, y, -0.67))
    for y in (-1.34, 1.34):
        _add_box(leaf, (4.45, 0.10, 0.16), (3.175, y, -0.50))
        _add_box(leaf, (4.10, 0.10, 0.10), (3.00, y, -0.87))
        for x in (1.00, 1.68, 2.36, 3.04, 3.72, 4.40, 5.08):
            _add_box(leaf, (0.06, 0.10, 0.44), (x, y, -0.67))
        brace_nodes = (1.00, 1.68, 2.36, 3.04, 3.72, 4.40, 5.08)
        for i in range(len(brace_nodes) - 1):
            x0 = brace_nodes[i]
            x1 = brace_nodes[i + 1]
            if i % 2 == 0:
                _add_brace(leaf, y, (x0, -0.87), (x1, -0.49))
            else:
                _add_brace(leaf, y, (x0, -0.49), (x1, -0.87))
    for x in (1.00, 1.68, 2.36, 3.04, 3.72, 4.40, 5.08):
        _add_box(leaf, (0.12, 2.45, 0.20), (x, 0.0, -0.55))
    _add_box(leaf, (0.18, 2.90, 0.26), (5.45, 0.0, -0.46))
    for y in (-1.30, 1.30):
        for x in (0.95, 1.75, 2.55, 3.35, 4.15, 4.95):
            _add_box(leaf, (0.05, 0.10, 0.36), (x, y, -0.11))
        _add_cylinder(
            leaf,
            radius=0.02,
            length=4.25,
            xyz=(3.025, y, 0.08),
            rpy=(0.0, math.pi * 0.5, 0.0),
        )
        _add_cylinder(
            leaf,
            radius=0.016,
            length=4.25,
            xyz=(3.025, y, -0.06),
            rpy=(0.0, math.pi * 0.5, 0.0),
        )
        _add_box(leaf, (0.12, 0.12, 0.24), (5.30, y, -0.08))
    leaf.inertial = Inertial.from_geometry(
        Box((5.45, 3.00, 0.17)),
        mass=6500.0,
        origin=_origin((2.725, 0.0, -0.375)),
    )

    model.articulation(
        "seat_mount",
        ArticulationType.FIXED,
        parent="support_frame",
        child="bridge_seat",
        origin=_origin((0.0, 0.0, 0.585)),
    )
    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent="support_frame",
        child="bridge_leaf",
        origin=_origin((0.0, 0.0, 1.25)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45000.0,
            velocity=0.8,
            lower=0.0,
            upper=1.0,
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
        "bridge_leaf",
        "support_frame",
        reason="hinge barrel and trunnion housings intentionally wrap the shared pivot at full opening",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.expect_aabb_overlap("bridge_leaf", "bridge_seat", axes="xy", min_overlap=0.68)
    ctx.expect_origin_distance("bridge_leaf", "bridge_seat", axes="xy", max_dist=2.40)
    ctx.expect_joint_motion_axis(
        "leaf_hinge",
        "bridge_leaf",
        world_axis="z",
        direction="positive",
        min_delta=1.00,
    )
    ctx.expect_joint_motion_axis(
        "leaf_hinge",
        "bridge_leaf",
        world_axis="x",
        direction="negative",
        min_delta=0.90,
    )
    with ctx.pose(leaf_hinge=0.60):
        ctx.expect_aabb_overlap("bridge_leaf", "bridge_seat", axes="xy", min_overlap=0.55)
        ctx.expect_origin_distance("bridge_leaf", "bridge_seat", axes="xy", max_dist=2.20)
    with ctx.pose(leaf_hinge=1.0):
        ctx.expect_aabb_overlap("bridge_leaf", "bridge_seat", axes="xy", min_overlap=0.24)
        ctx.expect_origin_distance("bridge_leaf", "bridge_seat", axes="xy", max_dist=1.10)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
