from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_WIDTH = 0.022
CHEEK_GAP = 0.012
CHEEK_THICKNESS = (OUTER_WIDTH - CHEEK_GAP) / 2.0
BODY_WIDTH = 0.0095

PIN_RADIUS = 0.0044
BARREL_LENGTH = CHEEK_GAP

ROOT_LENGTH = 0.046
ROOT_HEIGHT = 0.032
ROOT_FORK_HEIGHT = 0.024
ROOT_FRONT = 0.008
ROOT_SLOT_BACK = 0.013

PROXIMAL_LENGTH = 0.060
MIDDLE_LENGTH = 0.046
DISTAL_LENGTH = 0.034

PROXIMAL_HEIGHT = 0.020
MIDDLE_HEIGHT = 0.017
DISTAL_HEIGHT = 0.014

FORK_BACK = 0.014
FORK_FORWARD = 0.006
FORK_BRIDGE_DEPTH = 0.006


def _beam_solid(length: float, height_start: float, height_end: float, width: float) -> cq.Workplane:
    shoulder_x = max(length * 0.72, 0.012)
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, -height_start / 2.0),
                (shoulder_x, -height_start / 2.0),
                (length, -height_end / 2.0),
                (length, height_end / 2.0),
                (shoulder_x, height_start / 2.0),
                (0.0, height_start / 2.0),
            ]
        )
        .close()
        .extrude(width / 2.0, both=True)
    )


def _box_span(
    x_min: float,
    x_max: float,
    y_size: float,
    z_size: float,
    *,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(x_max - x_min, y_size, z_size)
        .translate(((x_min + x_max) * 0.5, y_center, z_center))
    )


def _cheek_box(x_min: float, x_max: float, height: float, side: float) -> cq.Workplane:
    y_center = side * (CHEEK_GAP * 0.5 + CHEEK_THICKNESS * 0.5)
    return _box_span(x_min, x_max, CHEEK_THICKNESS, height, y_center=y_center)


def _paired_cheeks(x_min: float, x_max: float, height: float) -> cq.Workplane:
    return _cheek_box(x_min, x_max, height, -1.0).union(_cheek_box(x_min, x_max, height, 1.0))


def _barrel() -> cq.Workplane:
    return cq.Workplane("XZ").circle(PIN_RADIUS).extrude(BARREL_LENGTH / 2.0, both=True)


def _root_block_shape() -> cq.Workplane:
    rear_body = _box_span(-ROOT_LENGTH, -ROOT_SLOT_BACK, OUTER_WIDTH, ROOT_HEIGHT)
    front_cheeks = _paired_cheeks(-ROOT_SLOT_BACK, ROOT_FRONT, ROOT_FORK_HEIGHT)
    return rear_body.union(front_cheeks)


def _link_body_with_fork_shape(length: float, height_start: float, height_end: float) -> cq.Workplane:
    fork_height = max(height_end + 0.007, PIN_RADIUS * 2.0 + 0.010)
    beam_length = length - FORK_BACK + FORK_BRIDGE_DEPTH
    beam = _beam_solid(beam_length, height_start, max(height_end, height_start * 0.86), BODY_WIDTH)
    bridge = _box_span(
        length - FORK_BACK,
        length - FORK_BACK + FORK_BRIDGE_DEPTH,
        OUTER_WIDTH,
        fork_height,
    )
    cheeks = _paired_cheeks(length - FORK_BACK, length + FORK_FORWARD, fork_height)
    return beam.union(bridge).union(cheeks)


def _distal_body_shape(length: float, height_start: float, height_end: float) -> cq.Workplane:
    beam = _beam_solid(length, height_start, height_end, BODY_WIDTH)
    fingertip = (
        cq.Workplane("XZ")
        .moveTo(length, 0.0)
        .circle(height_end / 2.0)
        .extrude(BODY_WIDTH / 2.0, both=True)
    )
    return beam.union(fingertip)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_cheek_finger_chain")

    model.material("root_gray", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("finger_alloy", rgba=(0.72, 0.74, 0.77, 1.0))

    root_block = model.part("root_block")
    root_block.visual(
        mesh_from_cadquery(_root_block_shape(), "root_block_shell"),
        material="root_gray",
        name="root_shell",
    )
    root_block.inertial = Inertial.from_geometry(
        Box((ROOT_LENGTH, OUTER_WIDTH, ROOT_HEIGHT)),
        mass=0.30,
        origin=Origin(xyz=(-ROOT_LENGTH * 0.55, 0.0, 0.0)),
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(
        mesh_from_cadquery(
            _link_body_with_fork_shape(PROXIMAL_LENGTH, PROXIMAL_HEIGHT, MIDDLE_HEIGHT + 0.002),
            "proximal_link_body",
        ),
        material="finger_alloy",
        name="proximal_body",
    )
    proximal_link.visual(
        mesh_from_cadquery(_barrel(), "proximal_link_barrel"),
        material="finger_alloy",
        name="proximal_barrel",
    )
    proximal_link.inertial = Inertial.from_geometry(
        Box((PROXIMAL_LENGTH, OUTER_WIDTH, PROXIMAL_HEIGHT)),
        mass=0.11,
        origin=Origin(xyz=(PROXIMAL_LENGTH * 0.48, 0.0, 0.0)),
    )

    middle_link = model.part("middle_link")
    middle_link.visual(
        mesh_from_cadquery(
            _link_body_with_fork_shape(MIDDLE_LENGTH, MIDDLE_HEIGHT, DISTAL_HEIGHT + 0.001),
            "middle_link_body",
        ),
        material="finger_alloy",
        name="middle_body",
    )
    middle_link.visual(
        mesh_from_cadquery(_barrel(), "middle_link_barrel"),
        material="finger_alloy",
        name="middle_barrel",
    )
    middle_link.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, OUTER_WIDTH, MIDDLE_HEIGHT)),
        mass=0.08,
        origin=Origin(xyz=(MIDDLE_LENGTH * 0.48, 0.0, 0.0)),
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        mesh_from_cadquery(
            _distal_body_shape(DISTAL_LENGTH, DISTAL_HEIGHT, DISTAL_HEIGHT * 0.72),
            "distal_link_body",
        ),
        material="finger_alloy",
        name="distal_body",
    )
    distal_link.visual(
        mesh_from_cadquery(_barrel(), "distal_link_barrel"),
        material="finger_alloy",
        name="distal_barrel",
    )
    distal_link.inertial = Inertial.from_geometry(
        Box((DISTAL_LENGTH, BODY_WIDTH, DISTAL_HEIGHT)),
        mass=0.05,
        origin=Origin(xyz=(DISTAL_LENGTH * 0.45, 0.0, 0.0)),
    )

    model.articulation(
        "root_knuckle",
        ArticulationType.REVOLUTE,
        parent=root_block,
        child=proximal_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=8.0, velocity=2.2),
    )
    model.articulation(
        "middle_knuckle",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=middle_link,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=5.0, velocity=2.5),
    )
    model.articulation(
        "distal_knuckle",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=distal_link,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=3.0, velocity=2.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    root_block = object_model.get_part("root_block")
    proximal_link = object_model.get_part("proximal_link")
    middle_link = object_model.get_part("middle_link")
    distal_link = object_model.get_part("distal_link")

    root_shell = root_block.get_visual("root_shell")
    proximal_body = proximal_link.get_visual("proximal_body")
    proximal_barrel = proximal_link.get_visual("proximal_barrel")
    middle_body = middle_link.get_visual("middle_body")
    middle_barrel = middle_link.get_visual("middle_barrel")
    distal_body = distal_link.get_visual("distal_body")
    distal_barrel = distal_link.get_visual("distal_barrel")

    root_knuckle = object_model.get_articulation("root_knuckle")
    middle_knuckle = object_model.get_articulation("middle_knuckle")
    distal_knuckle = object_model.get_articulation("distal_knuckle")

    joints = (root_knuckle, middle_knuckle, distal_knuckle)
    axes_ok = all(
        isclose(joint.axis[0], 0.0, abs_tol=1e-9)
        and isclose(joint.axis[1], -1.0, abs_tol=1e-9)
        and isclose(joint.axis[2], 0.0, abs_tol=1e-9)
        for joint in joints
    )
    ctx.check(
        "all knuckles pitch in one plane about -Y",
        axes_ok,
        details=f"axes={[joint.axis for joint in joints]}",
    )
    ctx.check(
        "finger links decrease in length",
        PROXIMAL_LENGTH > MIDDLE_LENGTH > DISTAL_LENGTH,
        details=(
            f"proximal={PROXIMAL_LENGTH:.3f}, middle={MIDDLE_LENGTH:.3f}, "
            f"distal={DISTAL_LENGTH:.3f}"
        ),
    )

    ctx.allow_overlap(
        root_block,
        proximal_link,
        elem_a=root_shell,
        elem_b=proximal_barrel,
        reason="The root-side clevis intentionally captures the proximal hinge barrel inside the knuckle envelope.",
    )
    ctx.allow_overlap(
        proximal_link,
        middle_link,
        elem_a=proximal_body,
        elem_b=middle_barrel,
        reason="The proximal fork intentionally captures the middle hinge barrel inside the knuckle envelope.",
    )
    ctx.allow_overlap(
        middle_link,
        distal_link,
        elem_a=middle_body,
        elem_b=distal_barrel,
        reason="The middle fork intentionally captures the distal hinge barrel inside the knuckle envelope.",
    )

    ctx.expect_within(
        proximal_link,
        root_block,
        axes="yz",
        inner_elem=proximal_barrel,
        outer_elem=root_shell,
        margin=0.0,
        name="proximal hinge barrel stays captured within the root knuckle envelope",
    )
    ctx.expect_within(
        middle_link,
        proximal_link,
        axes="yz",
        inner_elem=middle_barrel,
        outer_elem=proximal_body,
        margin=0.0,
        name="middle hinge barrel stays captured within the proximal knuckle envelope",
    )
    ctx.expect_within(
        distal_link,
        middle_link,
        axes="yz",
        inner_elem=distal_barrel,
        outer_elem=middle_body,
        margin=0.0,
        name="distal hinge barrel stays captured within the middle knuckle envelope",
    )
    ctx.expect_overlap(
        root_block,
        proximal_link,
        axes="x",
        elem_a=root_shell,
        elem_b=proximal_barrel,
        min_overlap=PIN_RADIUS * 1.9,
        name="root knuckle spans the proximal hinge barrel along the finger axis",
    )
    ctx.expect_overlap(
        proximal_link,
        middle_link,
        axes="x",
        elem_a=proximal_body,
        elem_b=middle_barrel,
        min_overlap=PIN_RADIUS * 1.9,
        name="proximal knuckle spans the middle hinge barrel along the finger axis",
    )
    ctx.expect_overlap(
        middle_link,
        distal_link,
        axes="x",
        elem_a=middle_body,
        elem_b=distal_barrel,
        min_overlap=PIN_RADIUS * 1.9,
        name="middle knuckle spans the distal hinge barrel along the finger axis",
    )

    ctx.expect_contact(
        middle_link,
        middle_link,
        elem_a=middle_barrel,
        elem_b=middle_body,
        contact_tol=5e-5,
        name="middle hinge barrel is fused to the front-supported middle link body",
    )
    ctx.expect_contact(
        distal_link,
        distal_link,
        elem_a=distal_barrel,
        elem_b=distal_body,
        contact_tol=5e-5,
        name="distal hinge barrel is fused to the distal link body",
    )
    ctx.expect_contact(
        proximal_link,
        proximal_link,
        elem_a=proximal_barrel,
        elem_b=proximal_body,
        contact_tol=5e-5,
        name="proximal hinge barrel is fused to the proximal link body",
    )

    root_aabb = ctx.part_world_aabb(root_block)
    proximal_aabb = ctx.part_world_aabb(proximal_link)
    middle_aabb = ctx.part_world_aabb(middle_link)
    distal_aabb = ctx.part_world_aabb(distal_link)

    forward_order_ok = (
        root_aabb is not None
        and proximal_aabb is not None
        and middle_aabb is not None
        and distal_aabb is not None
        and root_aabb[1][0] < proximal_aabb[1][0] < middle_aabb[1][0] < distal_aabb[1][0]
    )
    ctx.check(
        "straight pose steps forward from root to fingertip",
        forward_order_ok,
        details=(
            f"root={root_aabb}, proximal={proximal_aabb}, "
            f"middle={middle_aabb}, distal={distal_aabb}"
        ),
    )

    rest_prox_center = _aabb_center(proximal_aabb)
    rest_mid_center = _aabb_center(middle_aabb)
    rest_dist_center = _aabb_center(distal_aabb)

    with ctx.pose({root_knuckle: 0.80}):
        raised_prox_center = _aabb_center(ctx.part_world_aabb(proximal_link))
    root_lifts_ok = (
        rest_prox_center is not None
        and raised_prox_center is not None
        and raised_prox_center[2] > rest_prox_center[2] + 0.010
        and abs(raised_prox_center[1] - rest_prox_center[1]) < 5e-4
    )
    ctx.check(
        "root knuckle lifts the proximal link upward without leaving plane",
        root_lifts_ok,
        details=f"rest={rest_prox_center}, raised={raised_prox_center}",
    )

    with ctx.pose({middle_knuckle: 0.95}):
        raised_mid_center = _aabb_center(ctx.part_world_aabb(middle_link))
    middle_lifts_ok = (
        rest_mid_center is not None
        and raised_mid_center is not None
        and raised_mid_center[2] > rest_mid_center[2] + 0.008
        and abs(raised_mid_center[1] - rest_mid_center[1]) < 5e-4
    )
    ctx.check(
        "middle knuckle lifts the middle link upward without leaving plane",
        middle_lifts_ok,
        details=f"rest={rest_mid_center}, raised={raised_mid_center}",
    )

    with ctx.pose({distal_knuckle: 0.95}):
        raised_dist_center = _aabb_center(ctx.part_world_aabb(distal_link))
    distal_lifts_ok = (
        rest_dist_center is not None
        and raised_dist_center is not None
        and raised_dist_center[2] > rest_dist_center[2] + 0.004
        and abs(raised_dist_center[1] - rest_dist_center[1]) < 5e-4
    )
    ctx.check(
        "distal knuckle lifts the fingertip upward without leaving plane",
        distal_lifts_ok,
        details=f"rest={rest_dist_center}, raised={raised_dist_center}",
    )

    with ctx.pose({root_knuckle: 0.65, middle_knuckle: 0.90, distal_knuckle: 0.85}):
        curled_dist_center = _aabb_center(ctx.part_world_aabb(distal_link))
    curl_ok = (
        rest_dist_center is not None
        and curled_dist_center is not None
        and curled_dist_center[2] > rest_dist_center[2] + 0.025
        and curled_dist_center[0] < rest_dist_center[0] - 0.015
    )
    ctx.check(
        "combined pose curls the fingertip inward",
        curl_ok,
        details=f"rest={rest_dist_center}, curled={curled_dist_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
