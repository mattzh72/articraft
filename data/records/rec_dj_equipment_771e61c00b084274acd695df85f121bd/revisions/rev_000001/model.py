from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


HOUSING_W = 0.58
HOUSING_D = 0.42
HOUSING_H = 0.075
DECK_T = 0.003
DECK_TOP = HOUSING_H + DECK_T

PAD_W = 0.065
PAD_D = 0.052
PAD_H = 0.012
PAD_SKIRT_H = 0.006
PAD_TRAVEL = 0.006
PAD_PITCH_X = 0.078
PAD_PITCH_Y = 0.065
PAD_CUT_W = 0.073
PAD_CUT_D = 0.060
PAD_RECESS_D = 0.020

KNOB_X = 0.215
KNOB_Y = 0.135
KNOB_COLLAR_H = 0.0035


def _pad_positions() -> list[tuple[int, int, float, float]]:
    """Return row/column plus local housing XY centers for the 4 x 4 pad grid."""
    origin_x = -0.150
    origin_y = 0.050
    return [
        (row, col, origin_x + col * PAD_PITCH_X, origin_y - row * PAD_PITCH_Y)
        for row in range(4)
        for col in range(4)
    ]


def _rounded_rect_prism(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .sketch()
        .rect(width, depth)
        .vertices()
        .fillet(radius)
        .finalize()
        .extrude(height)
    )


def _make_housing_body() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(HOUSING_W, HOUSING_D, HOUSING_H)
        .translate((0.0, 0.0, HOUSING_H / 2.0))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.003)
    )

    for _, _, x, y in _pad_positions():
        cutter = (
            _rounded_rect_prism(PAD_CUT_W, PAD_CUT_D, PAD_RECESS_D + 0.001, 0.007)
            .translate((x, y, HOUSING_H - PAD_RECESS_D))
        )
        body = body.cut(cutter)

    return body


def _make_deck_plate() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .box(HOUSING_W - 0.055, HOUSING_D - 0.050, DECK_T)
        .translate((0.0, 0.0, HOUSING_H + DECK_T / 2.0))
        .edges("|Z")
        .fillet(0.006)
    )

    for _, _, x, y in _pad_positions():
        cutter = (
            _rounded_rect_prism(PAD_CUT_W, PAD_CUT_D, DECK_T + 0.004, 0.007)
            .translate((x, y, HOUSING_H - 0.001))
        )
        deck = deck.cut(cutter)

    knob_hole = (
        cq.Workplane("XY")
        .circle(0.017)
        .extrude(DECK_T + 0.004)
        .translate((KNOB_X, KNOB_Y, HOUSING_H - 0.001))
    )
    deck = deck.cut(knob_hole)
    return deck


def _make_pad_shape() -> cq.Workplane:
    flange = _rounded_rect_prism(PAD_CUT_W + 0.004, PAD_CUT_D + 0.004, 0.002, 0.008)
    top = _rounded_rect_prism(PAD_W, PAD_D, PAD_H, 0.008).edges(">Z").fillet(0.002)
    skirt = _rounded_rect_prism(PAD_W - 0.014, PAD_D - 0.012, PAD_SKIRT_H, 0.005).translate(
        (0.0, 0.0, -PAD_SKIRT_H)
    )
    return flange.union(top).union(skirt)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roland_style_drum_machine")

    model.material("charcoal_plastic", rgba=(0.055, 0.058, 0.060, 1.0))
    model.material("black_faceplate", rgba=(0.005, 0.006, 0.007, 1.0))
    model.material("soft_rubber", rgba=(0.018, 0.019, 0.020, 1.0))
    model.material("dark_rubber", rgba=(0.035, 0.036, 0.038, 1.0))
    model.material("burnt_orange", rgba=(0.92, 0.31, 0.10, 1.0))
    model.material("muted_display", rgba=(0.02, 0.14, 0.11, 1.0))
    model.material("led_red", rgba=(1.0, 0.08, 0.02, 1.0))
    model.material("encoder_metal", rgba=(0.18, 0.18, 0.17, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_make_housing_body(), "housing_body", tolerance=0.0008),
        material="charcoal_plastic",
        name="body_shell",
    )
    housing.visual(
        mesh_from_cadquery(_make_deck_plate(), "faceplate", tolerance=0.0008),
        material="black_faceplate",
        name="faceplate",
    )
    housing.visual(
        Box((HOUSING_W - 0.055, 0.012, 0.0012)),
        origin=Origin(xyz=(0.0, -0.180, DECK_TOP + 0.0006)),
        material="burnt_orange",
        name="front_accent",
    )
    housing.visual(
        Box((0.165, 0.045, 0.002)),
        origin=Origin(xyz=(-0.175, 0.155, DECK_TOP + 0.001)),
        material="muted_display",
        name="display_window",
    )
    housing.visual(
        Cylinder(radius=0.023, length=KNOB_COLLAR_H),
        origin=Origin(xyz=(KNOB_X, KNOB_Y, DECK_TOP + KNOB_COLLAR_H / 2.0)),
        material="encoder_metal",
        name="encoder_collar",
    )
    for row, col, x, y in _pad_positions():
        # Tiny surface LEDs are flush indicators, not separate controls.
        housing.visual(
            Box((0.011, 0.004, 0.0012)),
            origin=Origin(xyz=(x, y + PAD_D / 2.0 + 0.008, DECK_TOP + 0.0006)),
            material="led_red" if row == 0 else "encoder_metal",
            name=f"pad_led_{row}_{col}",
        )

    pad_mesh = mesh_from_cadquery(_make_pad_shape(), "velocity_pad", tolerance=0.0006)
    for row, col, x, y in _pad_positions():
        pad = model.part(f"pad_{row}_{col}")
        pad.visual(
            pad_mesh,
            material="soft_rubber" if (row + col) % 2 == 0 else "dark_rubber",
            name="pad_shell",
        )
        model.articulation(
            f"housing_to_pad_{row}_{col}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=pad,
            origin=Origin(xyz=(x, y, DECK_TOP)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=0.12,
                lower=0.0,
                upper=PAD_TRAVEL,
            ),
            motion_properties=MotionProperties(damping=0.45, friction=0.08),
        )

    encoder_knob = model.part("encoder_knob")
    knob_geometry = KnobGeometry(
        0.038,
        0.026,
        body_style="cylindrical",
        edge_radius=0.0012,
        grip=KnobGrip(style="knurled", count=40, depth=0.00075, helix_angle_deg=18.0),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=30.0),
        center=False,
    )
    encoder_knob.visual(
        mesh_from_geometry(knob_geometry, "encoder_knob"),
        material=Material("encoder_cap", rgba=(0.030, 0.031, 0.033, 1.0)),
        name="knob_cap",
    )
    model.articulation(
        "housing_to_encoder",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=encoder_knob,
        origin=Origin(xyz=(KNOB_X, KNOB_Y, DECK_TOP + KNOB_COLLAR_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.015),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    housing = object_model.get_part("housing")
    encoder = object_model.get_part("encoder_knob")

    pad_parts = [
        object_model.get_part(f"pad_{row}_{col}") for row in range(4) for col in range(4)
    ]
    pad_joints = [
        object_model.get_articulation(f"housing_to_pad_{row}_{col}")
        for row in range(4)
        for col in range(4)
    ]
    encoder_joint = object_model.get_articulation("housing_to_encoder")

    ctx.check("sixteen pads are separate articulated parts", len(pad_parts) == 16)
    ctx.check(
        "all pads use short vertical prismatic compression",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == (0.0, 0.0, -1.0)
            and joint.motion_limits is not None
            and math.isclose(joint.motion_limits.lower, 0.0)
            and math.isclose(joint.motion_limits.upper, PAD_TRAVEL)
            for joint in pad_joints
        ),
    )
    ctx.check(
        "encoder is a continuous rotary joint",
        encoder_joint.articulation_type == ArticulationType.CONTINUOUS
        and encoder_joint.axis == (0.0, 0.0, 1.0),
    )

    # Projected overlap proves every pad is centered in its panel recess rather
    # than floating beside the housing; actual collision is handled by baseline QC.
    for pad in pad_parts:
        ctx.expect_overlap(
            pad,
            housing,
            axes="xy",
            min_overlap=0.045,
            elem_a="pad_shell",
            elem_b="faceplate",
            name=f"{pad.name} sits in the pad field",
        )

    # One representative pad is checked at full travel to prove compression
    # moves downward by the authored short stroke.
    sample_pad = object_model.get_part("pad_0_0")
    sample_joint = object_model.get_articulation("housing_to_pad_0_0")
    rest_aabb = ctx.part_world_aabb(sample_pad)
    with ctx.pose({sample_joint: PAD_TRAVEL}):
        pressed_aabb = ctx.part_world_aabb(sample_pad)
    ctx.check(
        "pad compression lowers the pad top",
        rest_aabb is not None
        and pressed_aabb is not None
        and pressed_aabb[1][2] < rest_aabb[1][2] - PAD_TRAVEL * 0.75,
        details=f"rest={rest_aabb}, pressed={pressed_aabb}",
    )

    ctx.expect_contact(
        encoder,
        housing,
        elem_a="knob_cap",
        elem_b="encoder_collar",
        contact_tol=0.0008,
        name="encoder knob seats on its collar",
    )

    return ctx.report()


object_model = build_object_model()
