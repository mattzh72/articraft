from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

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

PALM_LENGTH = 0.038
PALM_WIDTH = 0.034
PALM_HEIGHT = 0.020
MOUNT_DEPTH = 0.009
MOUNT_HEIGHT = 0.013

JOINT_RADIUS = 0.0035
BARREL_LENGTH = 0.0052
CHEEK_THICKNESS = 0.0016
FORK_DEPTH = 0.008

LEFT_Y = 0.0105
RIGHT_Y = -0.0105

PROX_LENGTH = 0.042
MID_LENGTH = 0.031
DIST_LENGTH = 0.024


def _make_palm() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(PALM_LENGTH, PALM_WIDTH, PALM_HEIGHT)
        .translate((-PALM_LENGTH / 2.0, 0.0, 0.0))
    )
    try:
        body = body.edges("|Z").fillet(0.0014)
    except Exception:
        pass

    palm = body
    cheek_offset = BARREL_LENGTH / 2.0 + CHEEK_THICKNESS / 2.0
    for finger_y in (LEFT_Y, RIGHT_Y):
        for sign in (-1.0, 1.0):
            cheek = (
                cq.Workplane("XY")
                .box(MOUNT_DEPTH, CHEEK_THICKNESS, MOUNT_HEIGHT)
                .translate((MOUNT_DEPTH / 2.0, finger_y + sign * cheek_offset, 0.0))
            )
            palm = palm.union(cheek)
    return palm


def _make_phalanx(
    *,
    length: float,
    base_height: float,
    tip_height: float,
    body_width: float,
    add_fork: bool,
) -> cq.Workplane:
    body_end = length - (FORK_DEPTH if add_fork else 0.0)
    body = (
        cq.Workplane("XZ")
        .polyline(
            [
                (JOINT_RADIUS * 0.7, -base_height / 2.0),
                (body_end * 0.62, -base_height * 0.43),
                (body_end, -tip_height / 2.0),
                (length, -tip_height * 0.34),
                (length, tip_height * 0.34),
                (body_end, tip_height / 2.0),
                (body_end * 0.62, base_height * 0.43),
                (JOINT_RADIUS * 0.7, base_height / 2.0),
            ]
        )
        .close()
        .extrude(body_width / 2.0, both=True)
    )

    base_barrel = cq.Workplane("XZ").circle(JOINT_RADIUS).extrude(BARREL_LENGTH / 2.0, both=True)
    link = body.union(base_barrel)

    if add_fork:
        cheek_offset = BARREL_LENGTH / 2.0 + CHEEK_THICKNESS / 2.0
        cheek_height = max(tip_height * 0.95, JOINT_RADIUS * 1.7)
        for sign in (-1.0, 1.0):
            cheek = (
                cq.Workplane("XY")
                .box(FORK_DEPTH, CHEEK_THICKNESS, cheek_height)
                .translate((length - FORK_DEPTH / 2.0, sign * cheek_offset, 0.0))
            )
            link = link.union(cheek)
    else:
        fingertip = (
            cq.Workplane("XY")
            .box(length * 0.28, body_width * 0.9, tip_height * 0.95)
            .translate((length - length * 0.14, 0.0, 0.0))
        )
        try:
            fingertip = fingertip.edges("|Y").fillet(min(tip_height, body_width) * 0.18)
        except Exception:
            pass
        link = link.union(fingertip)

    return link


def _add_link(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    base_height: float,
    tip_height: float,
    body_width: float,
    add_fork: bool,
    filename: str,
    material,
    mass: float,
):
    part = model.part(name)
    part.visual(
        mesh_from_cadquery(
            _make_phalanx(
                length=length,
                base_height=base_height,
                tip_height=tip_height,
                body_width=body_width,
                add_fork=add_fork,
            ),
            filename,
            assets=ASSETS,
        ),
        name="shell",
        material=material,
    )
    part.inertial = Inertial.from_geometry(
        Box((length + JOINT_RADIUS * 2.0, body_width + CHEEK_THICKNESS * 2.0, base_height)),
        mass=mass,
        origin=Origin(xyz=(length / 2.0, 0.0, 0.0)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_finger_mechanism", assets=ASSETS)

    anodized = model.material("anodized_aluminum", rgba=(0.45, 0.48, 0.52, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.16, 0.17, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.24, 1.0))

    palm = model.part("palm")
    palm.visual(
        mesh_from_cadquery(_make_palm(), "palm.obj", assets=ASSETS),
        name="shell",
        material=anodized,
    )
    palm.inertial = Inertial.from_geometry(
        Box((PALM_LENGTH + MOUNT_DEPTH, PALM_WIDTH, PALM_HEIGHT)),
        mass=0.28,
        origin=Origin(xyz=((MOUNT_DEPTH - PALM_LENGTH) / 2.0, 0.0, 0.0)),
    )

    left_proximal = _add_link(
        model,
        name="left_proximal",
        length=PROX_LENGTH,
        base_height=0.012,
        tip_height=0.010,
        body_width=0.0082,
        add_fork=True,
        filename="left_proximal.obj",
        material=dark_polymer,
        mass=0.060,
    )
    left_middle = _add_link(
        model,
        name="left_middle",
        length=MID_LENGTH,
        base_height=0.010,
        tip_height=0.0086,
        body_width=0.0076,
        add_fork=True,
        filename="left_middle.obj",
        material=graphite,
        mass=0.040,
    )
    left_distal = _add_link(
        model,
        name="left_distal",
        length=DIST_LENGTH,
        base_height=0.0085,
        tip_height=0.0072,
        body_width=0.0068,
        add_fork=False,
        filename="left_distal.obj",
        material=dark_polymer,
        mass=0.025,
    )

    right_proximal = _add_link(
        model,
        name="right_proximal",
        length=PROX_LENGTH,
        base_height=0.012,
        tip_height=0.010,
        body_width=0.0082,
        add_fork=True,
        filename="right_proximal.obj",
        material=dark_polymer,
        mass=0.060,
    )
    right_middle = _add_link(
        model,
        name="right_middle",
        length=MID_LENGTH,
        base_height=0.010,
        tip_height=0.0086,
        body_width=0.0076,
        add_fork=True,
        filename="right_middle.obj",
        material=graphite,
        mass=0.040,
    )
    right_distal = _add_link(
        model,
        name="right_distal",
        length=DIST_LENGTH,
        base_height=0.0085,
        tip_height=0.0072,
        body_width=0.0068,
        add_fork=False,
        filename="right_distal.obj",
        material=dark_polymer,
        mass=0.025,
    )

    model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_proximal,
        origin=Origin(xyz=(MOUNT_DEPTH, LEFT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=0.95),
    )
    model.articulation(
        "left_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(PROX_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "left_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(MID_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.5, lower=0.0, upper=1.10),
    )

    model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_proximal,
        origin=Origin(xyz=(MOUNT_DEPTH, RIGHT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=0.95),
    )
    model.articulation(
        "right_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(PROX_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "right_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(MID_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.5, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    palm = object_model.get_part("palm")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_proximal, palm, name="left_proximal_contacts_palm_mount")
    ctx.expect_contact(right_proximal, palm, name="right_proximal_contacts_palm_mount")
    ctx.expect_contact(left_middle, left_proximal, name="left_middle_contacts_proximal_knuckle")
    ctx.expect_contact(left_distal, left_middle, name="left_distal_contacts_middle_knuckle")
    ctx.expect_contact(right_middle, right_proximal, name="right_middle_contacts_proximal_knuckle")
    ctx.expect_contact(right_distal, right_middle, name="right_distal_contacts_middle_knuckle")

    ctx.expect_origin_distance(
        left_proximal,
        right_proximal,
        axes="y",
        min_dist=0.018,
        max_dist=0.024,
        name="finger_base_pivots_are_side_by_side",
    )
    ctx.expect_overlap(
        left_proximal,
        palm,
        axes="yz",
        min_overlap=0.006,
        name="left_finger_mount_has_shared_knuckle_footprint",
    )
    ctx.expect_overlap(
        right_proximal,
        palm,
        axes="yz",
        min_overlap=0.006,
        name="right_finger_mount_has_shared_knuckle_footprint",
    )

    rest_left = ctx.part_world_position(left_distal)
    rest_right = ctx.part_world_position(right_distal)
    if rest_left is None or rest_right is None:
        ctx.fail("rest_pose_positions_available", "Could not measure distal-link world positions in the rest pose.")
    else:
        with ctx.pose(
            palm_to_left_proximal=0.55,
            left_proximal_to_middle=0.70,
            left_middle_to_distal=0.62,
        ):
            bent_left = ctx.part_world_position(left_distal)
            steady_right = ctx.part_world_position(right_distal)
        with ctx.pose(
            palm_to_right_proximal=0.52,
            right_proximal_to_middle=0.66,
            right_middle_to_distal=0.60,
        ):
            bent_right = ctx.part_world_position(right_distal)
            steady_left = ctx.part_world_position(left_distal)

        if bent_left is None or steady_right is None or bent_right is None or steady_left is None:
            ctx.fail("posed_positions_available", "Could not measure distal-link positions in articulated poses.")
        else:
            left_dx = bent_left[0] - rest_left[0]
            left_dy = bent_left[1] - rest_left[1]
            left_dz = bent_left[2] - rest_left[2]
            right_dx = bent_right[0] - rest_right[0]
            right_dy = bent_right[1] - rest_right[1]
            right_dz = bent_right[2] - rest_right[2]

            right_independent = math.dist(rest_right, steady_right)
            left_independent = math.dist(rest_left, steady_left)

            ctx.check(
                "left_finger_bends_in_one_plane",
                math.hypot(left_dx, left_dz) > 0.010 and abs(left_dy) < 5e-4,
                (
                    f"Expected left finger to sweep in x-z with negligible y drift; "
                    f"dx={left_dx:.5f}, dy={left_dy:.5f}, dz={left_dz:.5f}"
                ),
            )
            ctx.check(
                "right_finger_bends_in_one_plane",
                math.hypot(right_dx, right_dz) > 0.010 and abs(right_dy) < 5e-4,
                (
                    f"Expected right finger to sweep in x-z with negligible y drift; "
                    f"dx={right_dx:.5f}, dy={right_dy:.5f}, dz={right_dz:.5f}"
                ),
            )
            ctx.check(
                "left_chain_does_not_drive_right_chain",
                right_independent < 1e-7,
                f"Right distal moved {right_independent:.6f} m when only left joints were posed.",
            )
            ctx.check(
                "right_chain_does_not_drive_left_chain",
                left_independent < 1e-7,
                f"Left distal moved {left_independent:.6f} m when only right joints were posed.",
            )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=40,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
