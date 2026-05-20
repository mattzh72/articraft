from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.18
BASE_THICKNESS = 0.035
COLUMN_RADIUS = 0.07
COLUMN_HEIGHT = 0.31
HEAD_RADIUS = 0.10
HEAD_THICKNESS = 0.04
ROOT_HEIGHT = BASE_THICKNESS + COLUMN_HEIGHT + HEAD_THICKNESS

HUB_RADIUS = 0.095
HUB_THICKNESS = 0.05

MOUNT_X0 = 0.02
MOUNT_LENGTH = 0.22
MOUNT_WIDTH = 0.10
MOUNT_Z0 = 0.03
MOUNT_HEIGHT = 0.055

CARRIER_X0 = 0.24
CARRIER_LENGTH = 0.56
CARRIER_WIDTH = 0.11
CARRIER_Z0 = 0.03
CARRIER_HEIGHT = 0.05
TRACK_X0 = 0.28
TRACK_LENGTH = 0.48
TRACK_WIDTH = 0.07
TRACK_Z0 = CARRIER_Z0 + CARRIER_HEIGHT
TRACK_HEIGHT = 0.012

SLIDE_ORIGIN_X = 0.34
SLIDE_ORIGIN_Z = TRACK_Z0 + TRACK_HEIGHT
SLIDE_STROKE = 0.16
SLIDE_BAR_LENGTH = 0.38
SLIDE_BAR_WIDTH = 0.06
SLIDE_BAR_HEIGHT = 0.026
NOSE_BLOCK_LENGTH = 0.05
NOSE_BLOCK_WIDTH = 0.068
NOSE_BLOCK_HEIGHT = 0.032
PAD_THICKNESS = 0.012
PAD_WIDTH = 0.10
PAD_HEIGHT = 0.10

ROOT_SWEEP_LOWER = -2.4
ROOT_SWEEP_UPPER = 2.4


def _pedestal_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    column = (
        cq.Workplane("XY")
        .circle(COLUMN_RADIUS)
        .extrude(COLUMN_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    head = (
        cq.Workplane("XY")
        .circle(HEAD_RADIUS)
        .extrude(HEAD_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT))
    )
    return base.union(column).union(head)


def _beam_root_turntable() -> cq.Workplane:
    return cq.Workplane("XY").circle(HUB_RADIUS).extrude(HUB_THICKNESS)


def _beam_carrier_shape() -> cq.Workplane:
    mount = (
        cq.Workplane("XY")
        .box(MOUNT_LENGTH, MOUNT_WIDTH, MOUNT_HEIGHT)
        .translate(
            (
                MOUNT_X0 + MOUNT_LENGTH / 2.0,
                0.0,
                MOUNT_Z0 + MOUNT_HEIGHT / 2.0,
            )
        )
    )
    beam_body = (
        cq.Workplane("XY")
        .box(CARRIER_LENGTH, CARRIER_WIDTH, CARRIER_HEIGHT)
        .translate(
            (
                CARRIER_X0 + CARRIER_LENGTH / 2.0,
                0.0,
                CARRIER_Z0 + CARRIER_HEIGHT / 2.0,
            )
        )
    )
    track = (
        cq.Workplane("XY")
        .box(TRACK_LENGTH, TRACK_WIDTH, TRACK_HEIGHT)
        .translate(
            (
                TRACK_X0 + TRACK_LENGTH / 2.0,
                0.0,
                TRACK_Z0 + TRACK_HEIGHT / 2.0,
            )
        )
    )
    return mount.union(beam_body).union(track)


def _slide_carriage_shape() -> cq.Workplane:
    bar = (
        cq.Workplane("XY")
        .box(SLIDE_BAR_LENGTH, SLIDE_BAR_WIDTH, SLIDE_BAR_HEIGHT)
        .translate((SLIDE_BAR_LENGTH / 2.0, 0.0, SLIDE_BAR_HEIGHT / 2.0))
    )
    nose_block = (
        cq.Workplane("XY")
        .box(NOSE_BLOCK_LENGTH, NOSE_BLOCK_WIDTH, NOSE_BLOCK_HEIGHT)
        .translate(
            (
                SLIDE_BAR_LENGTH + NOSE_BLOCK_LENGTH / 2.0,
                0.0,
                NOSE_BLOCK_HEIGHT / 2.0,
            )
        )
    )
    return bar.union(nose_block)


def _tool_pad_shape() -> cq.Workplane:
    pad = (
        cq.Workplane("XY")
        .box(PAD_THICKNESS, PAD_WIDTH, PAD_HEIGHT)
        .translate(
            (
                SLIDE_BAR_LENGTH + NOSE_BLOCK_LENGTH + PAD_THICKNESS / 2.0,
                0.0,
                PAD_HEIGHT / 2.0,
            )
        )
    )
    return pad.edges("|X").fillet(0.008)


def _axis_matches(axis: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) < 1e-9 for a, b in zip(axis, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_rotary_slide_arm")

    pedestal_color = model.material("pedestal_graphite", rgba=(0.20, 0.20, 0.22, 1.0))
    beam_color = model.material("beam_silver", rgba=(0.66, 0.69, 0.72, 1.0))
    slide_color = model.material("slide_steel", rgba=(0.43, 0.46, 0.49, 1.0))
    pad_color = model.material("tool_pad_black", rgba=(0.09, 0.09, 0.10, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shape(), "pedestal_body"),
        material=pedestal_color,
        name="pedestal_body",
    )

    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(_beam_root_turntable(), "beam_root_turntable"),
        material=beam_color,
        name="root_turntable",
    )
    beam.visual(
        mesh_from_cadquery(_beam_carrier_shape(), "beam_carrier_body"),
        material=beam_color,
        name="carrier_body",
    )

    nose = model.part("nose")
    nose.visual(
        mesh_from_cadquery(_slide_carriage_shape(), "nose_slide_carriage"),
        material=slide_color,
        name="slide_carriage",
    )
    nose.visual(
        mesh_from_cadquery(_tool_pad_shape(), "nose_tool_pad"),
        material=pad_color,
        name="tool_pad",
    )

    model.articulation(
        "pedestal_to_beam",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, ROOT_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=ROOT_SWEEP_LOWER,
            upper=ROOT_SWEEP_UPPER,
        ),
    )

    model.articulation(
        "beam_to_nose",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=nose,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, SLIDE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.4,
            lower=0.0,
            upper=SLIDE_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    beam = object_model.get_part("beam")
    nose = object_model.get_part("nose")
    root = object_model.get_articulation("pedestal_to_beam")
    slide = object_model.get_articulation("beam_to_nose")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.check("pedestal_present", pedestal is not None, "pedestal part missing")
    ctx.check("beam_present", beam is not None, "beam part missing")
    ctx.check("nose_present", nose is not None, "nose part missing")

    ctx.check(
        "root_joint_is_revolute_vertical",
        root.articulation_type == ArticulationType.REVOLUTE
        and _axis_matches(root.axis, (0.0, 0.0, 1.0)),
        f"expected revolute root on +Z, got type={root.articulation_type} axis={root.axis}",
    )
    ctx.check(
        "nose_joint_is_prismatic_along_beam",
        slide.articulation_type == ArticulationType.PRISMATIC
        and _axis_matches(slide.axis, (1.0, 0.0, 0.0)),
        f"expected prismatic slide on +X, got type={slide.articulation_type} axis={slide.axis}",
    )
    ctx.check(
        "joint_limits_match_mechanism",
        root.motion_limits is not None
        and slide.motion_limits is not None
        and root.motion_limits.lower == ROOT_SWEEP_LOWER
        and root.motion_limits.upper == ROOT_SWEEP_UPPER
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == SLIDE_STROKE,
        "root or slide motion limits do not match the intended mechanism",
    )

    with ctx.pose({root: 0.0, slide: 0.0}):
        ctx.expect_contact(
            beam,
            pedestal,
            elem_a="root_turntable",
            elem_b="pedestal_body",
            name="beam_turntable_seats_on_pedestal_head",
        )
        ctx.expect_contact(
            nose,
            beam,
            elem_a="slide_carriage",
            elem_b="carrier_body",
            name="nose_carriage_bears_on_carrier_guides",
        )
        ctx.expect_within(
            nose,
            beam,
            axes="y",
            inner_elem="slide_carriage",
            outer_elem="carrier_body",
            margin=0.0,
            name="slide_carriage_stays_laterally_within_beam_width",
        )

    with ctx.pose({root: 0.0, slide: SLIDE_STROKE}):
        ctx.expect_contact(
            nose,
            beam,
            elem_a="slide_carriage",
            elem_b="carrier_body",
            name="extended_carriage_still_contacts_guides",
        )
        ctx.expect_gap(
            nose,
            beam,
            axis="x",
            positive_elem="tool_pad",
            negative_elem="carrier_body",
            min_gap=0.12,
            name="tool_pad_projects_past_beam_when_extended",
        )

    with ctx.pose({root: 0.0, slide: 0.0}):
        nose_rest = ctx.part_world_position(nose)
    with ctx.pose({root: 0.0, slide: SLIDE_STROKE}):
        nose_extended = ctx.part_world_position(nose)
    with ctx.pose({root: 1.1, slide: 0.10}):
        nose_swept = ctx.part_world_position(nose)
    with ctx.pose({root: 0.0, slide: 0.10}):
        nose_unswept = ctx.part_world_position(nose)

    prismatic_dx = abs(nose_extended[0] - nose_rest[0])
    prismatic_dy = abs(nose_extended[1] - nose_rest[1])
    prismatic_dz = abs(nose_extended[2] - nose_rest[2])
    ctx.check(
        "nose_slide_translates_along_beam_axis",
        abs(prismatic_dx - SLIDE_STROKE) < 1e-6 and prismatic_dy < 1e-6 and prismatic_dz < 1e-6,
        (
            "expected pure +X slide motion at root zero; "
            f"got dx={prismatic_dx:.6f}, dy={prismatic_dy:.6f}, dz={prismatic_dz:.6f}"
        ),
    )

    sweep_xy = math.hypot(nose_swept[0] - nose_unswept[0], nose_swept[1] - nose_unswept[1])
    sweep_dz = abs(nose_swept[2] - nose_unswept[2])
    ctx.check(
        "root_joint_sweeps_arm_in_horizontal_plane",
        sweep_xy > 0.25 and sweep_dz < 1e-6,
        (
            "expected yaw-like root motion with preserved height; "
            f"got xy_shift={sweep_xy:.6f}, z_shift={sweep_dz:.6f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
