from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BODY_TOP_Z = 0.066
SPOUT_TIP_ENTRY = (0.192, 0.0, 0.296)
TIP_PITCH = math.atan2(0.020, 0.028)
SPRAY_TRAVEL = 0.160


def _spout_socket_mesh():
    outer_profile = [
        (0.0148, 0.000),
        (0.0175, 0.008),
        (0.0175, 0.030),
        (0.0160, 0.040),
    ]
    inner_profile = [
        (0.0112, 0.000),
        (0.0138, 0.008),
        (0.0138, 0.030),
        (0.0126, 0.040),
    ]
    socket = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    socket.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(socket, "tip_socket")


def _spout_tube_mesh():
    tube = tube_from_spline_points(
        [
            (0.000, 0.0, 0.010),
            (0.000, 0.0, 0.125),
            (0.006, 0.0, 0.240),
            (0.045, 0.0, 0.338),
            (0.122, 0.0, 0.356),
            (0.178, 0.0, 0.322),
            SPOUT_TIP_ENTRY,
        ],
        radius=0.014,
        samples_per_segment=20,
        radial_segments=24,
        cap_ends=True,
    )
    return mesh_from_geometry(tube, "spout_tube")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prep_faucet")

    stainless = model.material("stainless", rgba=(0.80, 0.81, 0.83, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    hose_black = model.material("hose_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=stainless,
        name="deck_flange",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=stainless,
        name="body_column",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=stainless,
        name="body_cap",
    )
    body.visual(
        Cylinder(radius=0.0085, length=0.020),
        origin=Origin(xyz=(0.006, 0.031, 0.044), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="handle_boss",
    )
    body.visual(
        Box((0.018, 0.016, 0.020)),
        origin=Origin(xyz=(0.008, 0.022, 0.042)),
        material=stainless,
        name="handle_brace",
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stainless,
        name="swivel_collar",
    )
    spout.visual(
        _spout_tube_mesh(),
        material=stainless,
        name="spout_tube",
    )
    spout.visual(
        _spout_socket_mesh(),
        origin=Origin(xyz=SPOUT_TIP_ENTRY, rpy=(0.0, TIP_PITCH, 0.0)),
        material=stainless,
        name="tip_socket",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="pivot_barrel",
    )
    handle.visual(
        Box((0.072, 0.010, 0.012)),
        origin=Origin(xyz=(0.038, 0.006, 0.008)),
        material=stainless,
        name="lever_arm",
    )
    handle.visual(
        Box((0.028, 0.014, 0.018)),
        origin=Origin(xyz=(0.082, 0.007, 0.014)),
        material=stainless,
        name="lever_tip",
    )
    handle.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.083, 0.007, 0.022)),
        material=dark_trim,
        name="lever_pad",
    )

    spray_head = model.part("spray_head")
    spray_head.visual(
        Cylinder(radius=0.0105, length=0.192),
        origin=Origin(xyz=(-0.084, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hose_black,
        name="hose_stem",
    )
    spray_head.visual(
        Cylinder(radius=0.0130, length=0.052),
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="spray_body",
    )
    spray_head.visual(
        Cylinder(radius=0.0110, length=0.008),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="spray_face",
    )

    model.articulation(
        "spout_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.5),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.006, 0.041, 0.044)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.95,
        ),
    )
    model.articulation(
        "spray_slide",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray_head,
        origin=Origin(xyz=SPOUT_TIP_ENTRY, rpy=(0.0, TIP_PITCH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.45,
            lower=0.0,
            upper=SPRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    handle = object_model.get_part("handle")
    spray_head = object_model.get_part("spray_head")

    spout_turn = object_model.get_articulation("spout_turn")
    handle_pivot = object_model.get_articulation("handle_pivot")
    spray_slide = object_model.get_articulation("spray_slide")

    ctx.allow_overlap(
        spout,
        spray_head,
        elem_a="spout_tube",
        elem_b="hose_stem",
        reason="The pull-down hose stem is intentionally represented inside the solid gooseneck tube proxy.",
    )
    ctx.allow_overlap(
        spout,
        spray_head,
        elem_a="tip_socket",
        elem_b="spray_body",
        reason="The seated spray head is intentionally represented as nested inside the tip socket at rest.",
    )

    ctx.expect_gap(
        spout,
        body,
        axis="z",
        positive_elem="swivel_collar",
        negative_elem="body_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel collar seats cleanly on the body cap",
    )

    rest_spray_origin = ctx.part_world_position(spray_head)
    with ctx.pose({spout_turn: math.pi / 2.0}):
        quarter_turn_spray_origin = ctx.part_world_position(spray_head)
    ctx.check(
        "spout rotates about the base axis",
        rest_spray_origin is not None
        and quarter_turn_spray_origin is not None
        and abs(quarter_turn_spray_origin[0]) < 0.060
        and quarter_turn_spray_origin[1] > 0.150
        and abs(quarter_turn_spray_origin[2] - rest_spray_origin[2]) < 0.010,
        details=f"rest={rest_spray_origin}, quarter_turn={quarter_turn_spray_origin}",
    )

    rest_handle_tip = ctx.part_element_world_aabb(handle, elem="lever_tip")
    handle_limits = handle_pivot.motion_limits
    handle_upper = 0.95 if handle_limits is None or handle_limits.upper is None else handle_limits.upper
    with ctx.pose({handle_pivot: handle_upper}):
        raised_handle_tip = ctx.part_element_world_aabb(handle, elem="lever_tip")
    ctx.check(
        "side handle lifts on its short pivot",
        rest_handle_tip is not None
        and raised_handle_tip is not None
        and raised_handle_tip[1][2] > rest_handle_tip[1][2] + 0.040,
        details=f"rest_tip={rest_handle_tip}, raised_tip={raised_handle_tip}",
    )

    slide_limits = spray_slide.motion_limits
    slide_upper = SPRAY_TRAVEL if slide_limits is None or slide_limits.upper is None else slide_limits.upper
    with ctx.pose({spray_slide: slide_upper}):
        extended_spray_origin = ctx.part_world_position(spray_head)
    ctx.check(
        "spray head pulls down from the tip",
        rest_spray_origin is not None
        and extended_spray_origin is not None
        and extended_spray_origin[0] > rest_spray_origin[0] + 0.110
        and extended_spray_origin[2] < rest_spray_origin[2] - 0.070,
        details=f"rest={rest_spray_origin}, extended={extended_spray_origin}",
    )

    return ctx.report()


object_model = build_object_model()
