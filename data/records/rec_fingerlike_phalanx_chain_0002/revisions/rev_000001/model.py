from __future__ import annotations

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
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
PI = 3.141592653589793

PLATE_THICKNESS = 0.004
LINK_WIDTH = 0.014
BODY_WIDTH_PROX = 0.0108
BODY_WIDTH_MID = 0.0100
BODY_WIDTH_DIST = 0.0092

HINGE_RADIUS = 0.0045
PIN_RADIUS = 0.00155
CENTER_BARREL_WIDTH = 0.0052
JOINT_CLEARANCE = 0.0007
SIDE_BARREL_WIDTH = (LINK_WIDTH - CENTER_BARREL_WIDTH - (2.0 * JOINT_CLEARANCE)) / 2.0
SIDE_BARREL_Y = 0.5 * CENTER_BARREL_WIDTH + JOINT_CLEARANCE + 0.5 * SIDE_BARREL_WIDTH

BASE_BLOCK_X = 0.014
BASE_BLOCK_Y = 0.020
BASE_BLOCK_Z = 0.024
BASE_PAD_Z = 0.008

PROXIMAL_SPAN = 0.035
MIDDLE_SPAN = 0.027
DISTAL_LENGTH = 0.023


def _box(
    size_x: float, size_y: float, size_z: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0
):
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate((x, y, z))


def _y_cylinder(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length)
        .translate((0.0, y - (0.5 * length), 0.0))
    )


def _y_slot(
    length: float, diameter: float, depth: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0
):
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .slot2D(length, diameter, 90.0)
        .extrude(depth)
        .translate((0.0, y - (0.5 * depth), 0.0))
    )


def _add_visual_mesh(part, shape, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _make_base_mount_shape():
    block_center_z = -((0.5 * BASE_BLOCK_Z) + (0.60 * HINGE_RADIUS))
    pad_center_z = -(BASE_BLOCK_Z + (0.5 * BASE_PAD_Z) + (0.60 * HINGE_RADIUS))
    web_center_z = -0.55 * HINGE_RADIUS
    web_depth = 1.25 * HINGE_RADIUS

    mount = _box(
        BASE_BLOCK_X,
        BASE_BLOCK_Y,
        BASE_BLOCK_Z,
        z=block_center_z,
    ).union(
        _box(
            1.35 * BASE_BLOCK_X,
            1.15 * BASE_BLOCK_Y,
            BASE_PAD_Z,
            z=pad_center_z,
        )
    )

    for side_sign in (-1.0, 1.0):
        mount = mount.union(
            _y_cylinder(
                HINGE_RADIUS,
                SIDE_BARREL_WIDTH,
                y=side_sign * SIDE_BARREL_Y,
                z=0.0,
            )
        ).union(
            _box(
                1.15 * PLATE_THICKNESS,
                SIDE_BARREL_WIDTH,
                web_depth,
                y=side_sign * SIDE_BARREL_Y,
                z=web_center_z,
            )
        )

    mount = mount.cut(_y_cylinder(PIN_RADIUS, LINK_WIDTH + 0.004, z=0.0))
    return mount


def _make_phalanx_shape(span: float, body_width: float, *, has_distal_joint: bool):
    body_start_z = 0.50 * HINGE_RADIUS
    body_end_z = span - ((1.10 * HINGE_RADIUS) if has_distal_joint else (0.62 * HINGE_RADIUS))

    link = _box(
        PLATE_THICKNESS,
        body_width,
        body_end_z - body_start_z,
        z=0.5 * (body_start_z + body_end_z),
    ).union(_y_cylinder(HINGE_RADIUS, CENTER_BARREL_WIDTH, z=0.0))

    if has_distal_joint:
        web_depth = 1.35 * HINGE_RADIUS
        web_center_z = span - (0.65 * HINGE_RADIUS)
        for side_sign in (-1.0, 1.0):
            link = link.union(
                _y_cylinder(
                    HINGE_RADIUS,
                    SIDE_BARREL_WIDTH,
                    y=side_sign * SIDE_BARREL_Y,
                    z=span,
                )
            ).union(
                _box(
                    PLATE_THICKNESS,
                    SIDE_BARREL_WIDTH,
                    web_depth,
                    y=side_sign * SIDE_BARREL_Y,
                    z=web_center_z,
                )
            )
    else:
        tip_radius = 0.82 * HINGE_RADIUS
        link = link.union(
            _y_cylinder(
                tip_radius,
                0.82 * body_width,
                z=span - (0.20 * tip_radius),
            )
        )

    slot_length = min(0.50 * span, max(0.010, body_end_z - body_start_z - 0.004))
    if slot_length > 0.006:
        link = link.cut(
            _y_slot(
                slot_length,
                0.85 * PLATE_THICKNESS,
                0.62 * body_width,
                z=body_start_z + (0.56 * (body_end_z - body_start_z)),
            )
        )

    link = link.cut(_y_cylinder(PIN_RADIUS, LINK_WIDTH + 0.004, z=0.0))
    if has_distal_joint:
        link = link.cut(_y_cylinder(PIN_RADIUS, LINK_WIDTH + 0.004, z=span))

    return link


def _add_base_mount_collisions(part, *, mass: float) -> None:
    block_center_z = -((0.5 * BASE_BLOCK_Z) + (0.60 * HINGE_RADIUS))
    pad_center_z = -(BASE_BLOCK_Z + (0.5 * BASE_PAD_Z) + (0.60 * HINGE_RADIUS))




    for side_sign in (-1.0, 1.0):
        pass

    part.inertial = Inertial.from_geometry(
        Box((1.25 * BASE_BLOCK_X, BASE_BLOCK_Y, BASE_BLOCK_Z + BASE_PAD_Z)),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
    )


def _add_phalanx_collisions(
    part, *, span: float, body_width: float, mass: float, has_distal_joint: bool
) -> None:
    body_start_z = 0.62 * HINGE_RADIUS
    body_end_z = span - ((1.00 * HINGE_RADIUS) if has_distal_joint else (0.52 * HINGE_RADIUS))




    if has_distal_joint:
        for side_sign in (-1.0, 1.0):
            pass
    else:
        pass

    part.inertial = Inertial.from_geometry(
        Box((0.90 * PLATE_THICKNESS, 0.92 * body_width, span)),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * span)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_articulated_digit", assets=ASSETS)

    model.material("mount_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("plate_aluminum", rgba=(0.72, 0.74, 0.78, 1.0))

    base_mount = model.part("base_mount")
    proximal = model.part("proximal")
    middle = model.part("middle")
    distal = model.part("distal")

    base_shape = _make_base_mount_shape()
    proximal_shape = _make_phalanx_shape(PROXIMAL_SPAN, BODY_WIDTH_PROX, has_distal_joint=True)
    middle_shape = _make_phalanx_shape(MIDDLE_SPAN, BODY_WIDTH_MID, has_distal_joint=True)
    distal_shape = _make_phalanx_shape(DISTAL_LENGTH, BODY_WIDTH_DIST, has_distal_joint=False)

    _add_visual_mesh(base_mount, base_shape, "base_mount.obj", "mount_dark")
    _add_visual_mesh(proximal, proximal_shape, "proximal.obj", "plate_aluminum")
    _add_visual_mesh(middle, middle_shape, "middle.obj", "plate_aluminum")
    _add_visual_mesh(distal, distal_shape, "distal.obj", "plate_aluminum")

    _add_base_mount_collisions(base_mount, mass=0.18)
    _add_phalanx_collisions(
        proximal,
        span=PROXIMAL_SPAN,
        body_width=BODY_WIDTH_PROX,
        mass=0.040,
        has_distal_joint=True,
    )
    _add_phalanx_collisions(
        middle,
        span=MIDDLE_SPAN,
        body_width=BODY_WIDTH_MID,
        mass=0.030,
        has_distal_joint=True,
    )
    _add_phalanx_collisions(
        distal,
        span=DISTAL_LENGTH,
        body_width=BODY_WIDTH_DIST,
        mass=0.022,
        has_distal_joint=False,
    )

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base_mount,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.35,
            effort=3.0,
            velocity=4.0,
        ),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, PROXIMAL_SPAN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.50,
            effort=2.5,
            velocity=5.0,
        ),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_SPAN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.35,
            effort=2.0,
            velocity=5.0,
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
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_distance("proximal", "base_mount", axes="xy", max_dist=0.012)
    ctx.expect_origin_distance("middle", "proximal", axes="xy", max_dist=0.010)
    ctx.expect_origin_distance("distal", "middle", axes="xy", max_dist=0.010)

    ctx.expect_aabb_overlap("proximal", "base_mount", axes="xy", min_overlap=0.003)
    ctx.expect_aabb_overlap("middle", "proximal", axes="xy", min_overlap=0.003)
    ctx.expect_aabb_overlap("distal", "middle", axes="xy", min_overlap=0.003)

    ctx.expect_aabb_gap("proximal", "base_mount", axis="z", max_gap=0.004, max_penetration=0.012)
    ctx.expect_aabb_gap("middle", "proximal", axis="z", max_gap=0.004, max_penetration=0.012)
    ctx.expect_aabb_gap("distal", "middle", axis="z", max_gap=0.004, max_penetration=0.012)

    ctx.expect_origin_gap("middle", "base_mount", axis="z", min_gap=0.018)
    ctx.expect_origin_gap("distal", "base_mount", axis="z", min_gap=0.045)

    ctx.expect_joint_motion_axis(
        "base_to_proximal",
        "proximal",
        world_axis="x",
        direction="positive",
        min_delta=0.008,
    )
    ctx.expect_joint_motion_axis(
        "proximal_to_middle",
        "middle",
        world_axis="x",
        direction="positive",
        min_delta=0.008,
    )
    ctx.expect_joint_motion_axis(
        "middle_to_distal",
        "distal",
        world_axis="x",
        direction="positive",
        min_delta=0.006,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
