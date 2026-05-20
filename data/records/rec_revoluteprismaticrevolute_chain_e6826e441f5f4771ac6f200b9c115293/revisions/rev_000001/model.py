from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


BASE_LENGTH = 0.240
BASE_WIDTH = 0.200
BASE_THICK = 0.028
BASE_FILLET = 0.012

RING_RADIUS = 0.072
RING_HEIGHT = 0.028
POCKET_RADIUS = 0.038
POCKET_DEPTH = 0.018
POCKET_FLOOR_Z = BASE_THICK + RING_HEIGHT - POCKET_DEPTH

SPINDLE_SHOULDER_RADIUS = 0.024
SPINDLE_SHOULDER_HEIGHT = 0.006
SPINDLE_RADIUS = 0.016
SPINDLE_HEIGHT = 0.044

REVOLUTE_Z = 0.066

TURNTABLE_RADIUS = 0.078
TURNTABLE_THICK = 0.016
TURNTABLE_BORE_RADIUS = 0.020
TURNTABLE_FACE_RECESS_RADIUS = 0.030
TURNTABLE_FACE_RECESS_DEPTH = 0.002
TURNFASTENER_RADIUS = 0.0028
TURNFASTENER_PCD = 0.056

COLLAR_OUTER_RADIUS = 0.036
COLLAR_INNER_RADIUS = 0.019
COLLAR_HEIGHT = 0.010

GUIDE_BODY_LENGTH = 0.170
GUIDE_BODY_WIDTH = 0.110
GUIDE_BODY_HEIGHT = 0.078
GUIDE_BODY_CENTER_X = 0.120
GUIDE_BODY_CENTER_Z = 0.047

BEAM_AXIS_Z = GUIDE_BODY_CENTER_Z
TUNNEL_WIDTH = 0.072
TUNNEL_HEIGHT = 0.056

FRONT_SLEEVE_LENGTH = 0.024
FRONT_SLEEVE_WIDTH = 0.094
FRONT_SLEEVE_HEIGHT = 0.066
FRONT_SLEEVE_CENTER_X = 0.208

REAR_SLEEVE_LENGTH = 0.020
REAR_SLEEVE_WIDTH = 0.092
REAR_SLEEVE_HEIGHT = 0.064
REAR_SLEEVE_CENTER_X = 0.036

PRISMATIC_ORIGIN_X = 0.095
BEAM_TRAVEL = 0.150

BEAM_BAR_LENGTH = 0.290
BEAM_BAR_WIDTH = 0.064
BEAM_BAR_HEIGHT = 0.048
BEAM_BAR_CENTER_X = 0.035

WEAR_STRIP_LENGTH = 0.130
WEAR_STRIP_WIDTH = 0.068
WEAR_STRIP_HEIGHT = 0.004
WEAR_STRIP_CENTER_X = -0.065
WEAR_STRIP_TOP_Z = 0.026
WEAR_STRIP_BOTTOM_Z = -0.026

CARRIER_LENGTH = 0.060
CARRIER_WIDTH = 0.076
CARRIER_HEIGHT = 0.064
CARRIER_CENTER_X = 0.220
CARRIER_BORE_RADIUS = 0.0142
TIP_BRIDGE_LENGTH = 0.080
TIP_BRIDGE_WIDTH = 0.078
TIP_BRIDGE_HEIGHT = 0.064
TIP_BRIDGE_CENTER_X = 0.170

NOSE_JOINT_X = CARRIER_CENTER_X
NOSE_SHAFT_RADIUS = CARRIER_BORE_RADIUS
NOSE_SHAFT_LENGTH = CARRIER_LENGTH + 0.016
NOSE_COLLAR_THICK = 0.006
NOSE_COLLAR_RADIUS = 0.020
NOSE_COLLAR_CENTER_X = CARRIER_LENGTH / 2.0 + NOSE_COLLAR_THICK / 2.0 + 0.0005
NOSE_BODY_START_X = NOSE_COLLAR_CENTER_X + NOSE_COLLAR_THICK / 2.0
NOSE_BODY_LENGTH = 0.062
NOSE_BODY_RADIUS = 0.0135
NOSE_TIP_LENGTH = 0.018
NOSE_TIP_RADIUS = 0.010

TOOL_PAD_LENGTH = 0.030
TOOL_PAD_WIDTH = 0.018
TOOL_PAD_HEIGHT = 0.010
TOOL_PAD_CENTER_X = 0.070
TOOL_PAD_CENTER_Z = -0.016


def _bolt_points(radius: float, count: int) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / count), radius * sin(2.0 * pi * i / count))
        for i in range(count)
    ]


def _pedestal_shell_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICK,
        centered=(True, True, False),
    )
    base = base.edges("|Z").fillet(BASE_FILLET)

    ring = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICK)
        .circle(RING_RADIUS)
        .extrude(RING_HEIGHT)
        .faces(">Z")
        .workplane()
        .circle(POCKET_RADIUS)
        .cutBlind(-POCKET_DEPTH)
    )

    return base.union(ring)


def _pedestal_spindle_shape() -> cq.Workplane:
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=POCKET_FLOOR_Z)
        .circle(SPINDLE_SHOULDER_RADIUS)
        .extrude(SPINDLE_SHOULDER_HEIGHT)
    )
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=POCKET_FLOOR_Z + SPINDLE_SHOULDER_HEIGHT)
        .circle(SPINDLE_RADIUS)
        .extrude(SPINDLE_HEIGHT)
    )
    return shoulder.union(shaft)


def _turntable_disk_shape() -> cq.Workplane:
    disk = (
        cq.Workplane("XY")
        .workplane(offset=-TURNTABLE_THICK / 2.0)
        .circle(TURNTABLE_RADIUS)
        .circle(TURNTABLE_BORE_RADIUS)
        .extrude(TURNTABLE_THICK)
        .faces(">Z")
        .workplane()
        .circle(TURNTABLE_FACE_RECESS_RADIUS)
        .cutBlind(-TURNTABLE_FACE_RECESS_DEPTH)
    )
    disk = (
        disk.faces(">Z")
        .workplane()
        .pushPoints(_bolt_points(TURNFASTENER_PCD, 6))
        .hole(2.0 * TURNFASTENER_RADIUS, depth=TURNTABLE_FACE_RECESS_DEPTH)
    )
    return disk


def _bearing_collar_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .workplane(offset=-(TURNTABLE_THICK / 2.0 + COLLAR_HEIGHT))
        .circle(COLLAR_OUTER_RADIUS)
        .extrude(COLLAR_HEIGHT)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-(TURNTABLE_THICK / 2.0 + COLLAR_HEIGHT + 0.001))
        .circle(COLLAR_INNER_RADIUS)
        .extrude(COLLAR_HEIGHT + 0.002)
    )
    return outer.cut(inner)


def _guide_housing_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        GUIDE_BODY_LENGTH,
        GUIDE_BODY_WIDTH,
        GUIDE_BODY_HEIGHT,
    )
    body = body.translate((GUIDE_BODY_CENTER_X, 0.0, GUIDE_BODY_CENTER_Z))

    tunnel = cq.Workplane("XY").box(
        GUIDE_BODY_LENGTH + 0.030,
        TUNNEL_WIDTH,
        TUNNEL_HEIGHT,
    )
    tunnel = tunnel.translate((GUIDE_BODY_CENTER_X, 0.0, BEAM_AXIS_Z))
    body = body.cut(tunnel)

    for side_y in (-0.038, 0.038):
        window = cq.Workplane("XY").box(0.092, 0.034, 0.040)
        window = window.translate((GUIDE_BODY_CENTER_X + 0.018, side_y, GUIDE_BODY_CENTER_Z + 0.004))
        body = body.cut(window)

    return body


def _guide_sleeve_shape(
    *,
    length: float,
    width: float,
    height: float,
    center_x: float,
    name_clearance: float,
) -> cq.Workplane:
    sleeve = cq.Workplane("XY").box(length, width, height)
    sleeve = sleeve.translate((center_x, 0.0, BEAM_AXIS_Z))
    tunnel = cq.Workplane("XY").box(length + 0.012, TUNNEL_WIDTH + name_clearance, TUNNEL_HEIGHT + name_clearance)
    tunnel = tunnel.translate((center_x, 0.0, BEAM_AXIS_Z))
    return sleeve.cut(tunnel)


def _beam_bar_shape() -> cq.Workplane:
    bar = cq.Workplane("XY").box(BEAM_BAR_LENGTH, BEAM_BAR_WIDTH, BEAM_BAR_HEIGHT)
    bar = bar.translate((BEAM_BAR_CENTER_X, 0.0, 0.0))

    side_relief = cq.Workplane("XY").box(0.150, 0.010, 0.024)
    side_relief = side_relief.translate((0.020, 0.030, 0.0))
    bar = bar.cut(side_relief)
    bar = bar.cut(side_relief.mirror("YZ"))
    return bar


def _wear_strip_shape(z_center: float) -> cq.Workplane:
    return cq.Workplane("XY").box(WEAR_STRIP_LENGTH, WEAR_STRIP_WIDTH, WEAR_STRIP_HEIGHT).translate(
        (WEAR_STRIP_CENTER_X, 0.0, z_center)
    )


def _beam_core_shape() -> cq.Workplane:
    bridge = cq.Workplane("XY").box(TIP_BRIDGE_LENGTH, TIP_BRIDGE_WIDTH, TIP_BRIDGE_HEIGHT).translate(
        (TIP_BRIDGE_CENTER_X, 0.0, 0.0)
    )
    carrier = cq.Workplane("XY").box(CARRIER_LENGTH, CARRIER_WIDTH, CARRIER_HEIGHT).translate(
        (CARRIER_CENTER_X, 0.0, 0.0)
    )
    sleeve_outer = (
        cq.Workplane("YZ")
        .workplane(offset=CARRIER_CENTER_X - CARRIER_LENGTH / 2.0)
        .circle(CARRIER_HEIGHT * 0.34)
        .extrude(CARRIER_LENGTH)
    )
    spindle_bore = (
        cq.Workplane("YZ")
        .workplane(offset=CARRIER_CENTER_X - CARRIER_LENGTH / 2.0 - 0.001)
        .circle(CARRIER_BORE_RADIUS)
        .extrude(CARRIER_LENGTH + 0.002)
    )
    clamp_flat = cq.Workplane("XY").box(CARRIER_LENGTH * 0.60, CARRIER_WIDTH * 0.76, CARRIER_HEIGHT * 0.28).translate(
        (CARRIER_CENTER_X, 0.0, CARRIER_HEIGHT * 0.21)
    )
    body = bridge.union(carrier).union(sleeve_outer).cut(clamp_flat)
    return body.cut(spindle_bore).edges("|X").fillet(0.004)


def _nose_shaft_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .workplane(offset=-NOSE_SHAFT_LENGTH / 2.0)
        .circle(NOSE_SHAFT_RADIUS)
        .extrude(NOSE_SHAFT_LENGTH)
    )


def _nose_ring_shape(center_x: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .workplane(offset=center_x - NOSE_COLLAR_THICK / 2.0)
        .circle(NOSE_COLLAR_RADIUS)
        .extrude(NOSE_COLLAR_THICK)
    )


def _nose_hub_shape() -> cq.Workplane:
    hub = _nose_shaft_shape()
    hub = hub.union(_nose_ring_shape(-NOSE_COLLAR_CENTER_X, NOSE_COLLAR_RADIUS))
    hub = hub.union(_nose_ring_shape(NOSE_COLLAR_CENTER_X, NOSE_COLLAR_RADIUS))
    return hub


def _nose_shell_shape() -> cq.Workplane:
    hub = _nose_hub_shape()
    body = (
        cq.Workplane("YZ")
        .workplane(offset=NOSE_BODY_START_X)
        .circle(NOSE_BODY_RADIUS)
        .extrude(NOSE_BODY_LENGTH)
    )
    tip = (
        cq.Workplane("YZ")
        .workplane(offset=NOSE_BODY_START_X + NOSE_BODY_LENGTH)
        .circle(NOSE_TIP_RADIUS)
        .extrude(NOSE_TIP_LENGTH)
    )
    tool_pad = cq.Workplane("XY").box(TOOL_PAD_LENGTH, TOOL_PAD_WIDTH, TOOL_PAD_HEIGHT).translate(
        (TOOL_PAD_CENTER_X, 0.0, TOOL_PAD_CENTER_Z)
    )
    return hub.union(body).union(tip).union(tool_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_rotary_prismatic_manipulator")

    model.material("powder_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("machined_steel", rgba=(0.62, 0.66, 0.70, 1.0))
    model.material("anodized_mid", rgba=(0.48, 0.52, 0.57, 1.0))
    model.material("anodized_dark", rgba=(0.30, 0.33, 0.37, 1.0))
    model.material("black_oxide", rgba=(0.11, 0.12, 0.13, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shell_shape(), "pedestal_shell"),
        material="powder_dark",
        name="pedestal_shell",
    )
    pedestal.visual(
        mesh_from_cadquery(_pedestal_spindle_shape(), "pedestal_spindle"),
        material="machined_steel",
        name="pedestal_spindle",
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        mesh_from_cadquery(_turntable_disk_shape(), "turntable_disk"),
        material="machined_steel",
        name="turntable_disk",
    )
    rotary_stage.visual(
        mesh_from_cadquery(_bearing_collar_shape(), "bearing_collar"),
        material="anodized_mid",
        name="bearing_collar",
    )
    rotary_stage.visual(
        mesh_from_cadquery(_guide_housing_shape(), "guide_housing"),
        material="anodized_dark",
        name="guide_housing",
    )
    rotary_stage.visual(
        mesh_from_cadquery(
            _guide_sleeve_shape(
                length=FRONT_SLEEVE_LENGTH,
                width=FRONT_SLEEVE_WIDTH,
                height=FRONT_SLEEVE_HEIGHT,
                center_x=FRONT_SLEEVE_CENTER_X,
                name_clearance=0.004,
            ),
            "front_sleeve",
        ),
        material="machined_steel",
        name="front_sleeve",
    )
    rotary_stage.visual(
        mesh_from_cadquery(
            _guide_sleeve_shape(
                length=REAR_SLEEVE_LENGTH,
                width=REAR_SLEEVE_WIDTH,
                height=REAR_SLEEVE_HEIGHT,
                center_x=REAR_SLEEVE_CENTER_X,
                name_clearance=0.004,
            ),
            "rear_sleeve",
        ),
        material="machined_steel",
        name="rear_sleeve",
    )

    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(_beam_bar_shape(), "beam_bar"),
        material="anodized_mid",
        name="beam_bar",
    )
    beam.visual(
        mesh_from_cadquery(_wear_strip_shape(WEAR_STRIP_TOP_Z), "top_wear_strip"),
        material="black_oxide",
        name="top_wear_strip",
    )
    beam.visual(
        mesh_from_cadquery(_wear_strip_shape(WEAR_STRIP_BOTTOM_Z), "bottom_wear_strip"),
        material="black_oxide",
        name="bottom_wear_strip",
    )
    beam.visual(
        mesh_from_cadquery(_beam_core_shape(), "beam_core"),
        material="anodized_dark",
        name="beam_core",
    )

    nose = model.part("nose")
    nose.visual(
        mesh_from_cadquery(_nose_shell_shape(), "nose_shell"),
        material="machined_steel",
        name="nose_shell",
    )

    model.articulation(
        "pedestal_to_rotary",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, REVOLUTE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.3, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "rotary_to_beam",
        ArticulationType.PRISMATIC,
        parent=rotary_stage,
        child=beam,
        origin=Origin(xyz=(PRISMATIC_ORIGIN_X, 0.0, BEAM_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.32, lower=0.0, upper=BEAM_TRAVEL),
    )
    model.articulation(
        "beam_to_nose",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=nose,
        origin=Origin(xyz=(NOSE_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    rotary_stage = object_model.get_part("rotary_stage")
    beam = object_model.get_part("beam")
    nose = object_model.get_part("nose")

    base_turn = object_model.get_articulation("pedestal_to_rotary")
    beam_slide = object_model.get_articulation("rotary_to_beam")
    nose_roll = object_model.get_articulation("beam_to_nose")

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
    ctx.allow_overlap(
        beam,
        nose,
        elem_a="beam_core",
        elem_b="nose_shell",
        reason="captured roll spindle intentionally nests inside the bored beam carrier as the supported nose-axis bearing interface",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(
        rotary_stage,
        pedestal,
        axes="xy",
        min_dist=0.0,
        max_dist=0.0005,
        name="rotary_axis_stays_centered_on_pedestal",
    )
    ctx.expect_gap(
        rotary_stage,
        pedestal,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="pedestal_shell",
        min_gap=0.0015,
        max_gap=0.0035,
        name="turntable_face_clearance",
    )
    ctx.expect_overlap(
        rotary_stage,
        pedestal,
        axes="xy",
        min_overlap=0.120,
        elem_a="turntable_disk",
        elem_b="pedestal_shell",
        name="turntable_footprint_covers_bearing_pocket",
    )
    ctx.expect_within(
        beam,
        rotary_stage,
        axes="yz",
        inner_elem="beam_bar",
        outer_elem="guide_housing",
        margin=0.0,
        name="beam_bar_stays_inside_guide_envelope_retracted",
    )
    ctx.expect_contact(
        beam,
        rotary_stage,
        elem_a="top_wear_strip",
        elem_b="guide_housing",
        name="top_wear_strip_supports_beam",
    )
    ctx.expect_contact(
        beam,
        rotary_stage,
        elem_a="bottom_wear_strip",
        elem_b="guide_housing",
        name="bottom_wear_strip_supports_beam",
    )
    ctx.expect_contact(
        nose,
        beam,
        elem_a="nose_shell",
        elem_b="beam_core",
        name="nose_shell_is_supported_by_roll_sleeve",
    )
    ctx.expect_within(
        nose,
        beam,
        axes="yz",
        inner_elem="nose_shell",
        outer_elem="beam_core",
        margin=0.008,
        name="nose_shell_stays_centered_in_roll_support",
    )

    with ctx.pose({beam_slide: BEAM_TRAVEL, nose_roll: 1.1}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_articulated_pose")
        ctx.expect_within(
            beam,
            rotary_stage,
            axes="yz",
            inner_elem="beam_bar",
            outer_elem="guide_housing",
            margin=0.0,
            name="beam_bar_stays_inside_guide_envelope_extended",
        )
        ctx.expect_within(
            nose,
            beam,
            axes="yz",
            inner_elem="nose_shell",
            outer_elem="beam_core",
            margin=0.008,
            name="nose_shell_stays_centered_when_rolled",
        )

    with ctx.pose({base_turn: 1.0, beam_slide: 0.08, nose_roll: 0.3}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_rotated_base_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
