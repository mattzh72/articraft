from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, radians, sin

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


BASE_PLATE_LENGTH = 0.58
BASE_PLATE_WIDTH = 0.18
BASE_PLATE_THICKNESS = 0.014
BASE_TOP_Z = -0.048
ROOT_AXIS_Z = 0.045
PIVOT_HOLE_RADIUS = 0.0095
LUG_OFFSET_Y = 0.019
LUG_LENGTH_Y = 0.010
CENTER_BOSS_LENGTH_Y = 0.018

INPUT_ANGLE_DEG = 24.0
INPUT_LINK_LENGTH = 0.145
INPUT_DISTAL = (
    INPUT_LINK_LENGTH * cos(radians(INPUT_ANGLE_DEG)),
    0.0,
    INPUT_LINK_LENGTH * sin(radians(INPUT_ANGLE_DEG)),
)

TRANSFER_ANGLE_DEG = 8.0
TRANSFER_LINK_LENGTH = 0.215
TRANSFER_DISTAL = (
    TRANSFER_LINK_LENGTH * cos(radians(TRANSFER_ANGLE_DEG)),
    0.0,
    TRANSFER_LINK_LENGTH * sin(radians(TRANSFER_ANGLE_DEG)),
)

OUTPUT_ANGLE_DEG = 14.0
OUTPUT_LINK_LENGTH = 0.302


def _point_on_ray(length: float, angle_deg: float) -> tuple[float, float, float]:
    angle = radians(angle_deg)
    return (length * cos(angle), 0.0, length * sin(angle))


def _y_boss(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center[0], center[2])
        .circle(radius)
        .extrude(length)
        .translate((0.0, center[1] - (length * 0.5), 0.0))
    )


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(center[0], center[1])
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, center[2] - (length * 0.5)))
    )


def _beam_box(
    length: float,
    thickness: float,
    height: float,
    center: tuple[float, float, float],
    angle_deg: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, thickness, height)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)
        .translate(center)
    )


def _plain_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    angle_deg: float = 0.0,
) -> cq.Workplane:
    box = cq.Workplane("XY").box(*size)
    if abs(angle_deg) > 1e-9:
        box = box.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)
    return box.translate(center)


def make_base_frame() -> cq.Workplane:
    body = _plain_box(
        (BASE_PLATE_LENGTH, BASE_PLATE_WIDTH, BASE_PLATE_THICKNESS),
        (0.18, 0.0, BASE_TOP_Z - (BASE_PLATE_THICKNESS * 0.5)),
    )

    body = body.union(_plain_box((0.50, 0.026, 0.018), (0.18, 0.058, -0.072)))
    body = body.union(_plain_box((0.50, 0.026, 0.018), (0.18, -0.058, -0.072)))
    body = body.union(_plain_box((0.070, 0.052, 0.040), (-0.006, 0.0, -0.004)))
    body = body.union(_plain_box((0.028, LUG_LENGTH_Y, 0.036), (0.004, LUG_OFFSET_Y, 0.036)))
    body = body.union(_plain_box((0.028, LUG_LENGTH_Y, 0.036), (0.004, -LUG_OFFSET_Y, 0.036)))
    body = body.union(_y_boss(0.018, LUG_LENGTH_Y, (0.0, LUG_OFFSET_Y, ROOT_AXIS_Z)))
    body = body.union(_y_boss(0.018, LUG_LENGTH_Y, (0.0, -LUG_OFFSET_Y, ROOT_AXIS_Z)))
    body = body.union(_beam_box(0.062, 0.010, 0.012, (-0.010, LUG_OFFSET_Y, 0.006), 55.0))
    body = body.union(_beam_box(0.062, 0.010, 0.012, (-0.010, -LUG_OFFSET_Y, 0.006), 55.0))

    pivot_hole = _y_boss(PIVOT_HOLE_RADIUS, 0.060, (0.0, 0.0, ROOT_AXIS_Z))

    mount_holes = [
        _z_cylinder(0.0065, 0.09, (x, y, -0.060))
        for x in (-0.045, 0.405)
        for y in (-0.055, 0.055)
    ]
    for hole in [pivot_hole, *mount_holes]:
        body = body.cut(hole)

    return body


def make_input_crank() -> cq.Workplane:
    center_beam = _point_on_ray(0.042, INPUT_ANGLE_DEG)
    side_beam = _point_on_ray(0.109, INPUT_ANGLE_DEG)
    lug_pad = _point_on_ray(0.128, INPUT_ANGLE_DEG)

    body = _y_boss(0.020, CENTER_BOSS_LENGTH_Y, (0.0, 0.0, 0.0))
    body = body.union(_beam_box(0.084, 0.012, 0.014, center_beam, INPUT_ANGLE_DEG))
    body = body.union(_beam_box(0.052, 0.008, 0.012, (side_beam[0], 0.012, side_beam[2]), INPUT_ANGLE_DEG))
    body = body.union(_beam_box(0.052, 0.008, 0.012, (side_beam[0], -0.012, side_beam[2]), INPUT_ANGLE_DEG))
    body = body.union(_beam_box(0.030, 0.010, 0.012, (lug_pad[0], LUG_OFFSET_Y, lug_pad[2]), INPUT_ANGLE_DEG))
    body = body.union(_beam_box(0.030, 0.010, 0.012, (lug_pad[0], -LUG_OFFSET_Y, lug_pad[2]), INPUT_ANGLE_DEG))
    body = body.union(_y_boss(0.0155, LUG_LENGTH_Y, (INPUT_DISTAL[0], LUG_OFFSET_Y, INPUT_DISTAL[2])))
    body = body.union(_y_boss(0.0155, LUG_LENGTH_Y, (INPUT_DISTAL[0], -LUG_OFFSET_Y, INPUT_DISTAL[2])))
    body = body.union(_beam_box(0.030, 0.006, 0.010, _point_on_ray(0.020, INPUT_ANGLE_DEG), INPUT_ANGLE_DEG))

    body = body.cut(_y_boss(PIVOT_HOLE_RADIUS, 0.030, (0.0, 0.0, 0.0)))
    body = body.cut(_y_boss(0.0085, 0.060, INPUT_DISTAL))
    return body


def make_transfer_lever() -> cq.Workplane:
    center_beam = _point_on_ray(0.056, TRANSFER_ANGLE_DEG)
    side_beam = _point_on_ray(0.172, TRANSFER_ANGLE_DEG)
    lug_pad = _point_on_ray(0.194, TRANSFER_ANGLE_DEG)

    body = _y_boss(0.019, CENTER_BOSS_LENGTH_Y, (0.0, 0.0, 0.0))
    body = body.union(_beam_box(0.112, 0.012, 0.015, center_beam, TRANSFER_ANGLE_DEG))
    body = body.union(_beam_box(0.056, 0.008, 0.012, (side_beam[0], 0.012, side_beam[2]), TRANSFER_ANGLE_DEG))
    body = body.union(_beam_box(0.056, 0.008, 0.012, (side_beam[0], -0.012, side_beam[2]), TRANSFER_ANGLE_DEG))
    body = body.union(_beam_box(0.032, 0.010, 0.012, (lug_pad[0], LUG_OFFSET_Y, lug_pad[2]), TRANSFER_ANGLE_DEG))
    body = body.union(_beam_box(0.032, 0.010, 0.012, (lug_pad[0], -LUG_OFFSET_Y, lug_pad[2]), TRANSFER_ANGLE_DEG))
    body = body.union(_beam_box(0.040, 0.006, 0.010, _point_on_ray(0.026, TRANSFER_ANGLE_DEG), TRANSFER_ANGLE_DEG))
    body = body.union(_y_boss(0.0155, LUG_LENGTH_Y, (TRANSFER_DISTAL[0], LUG_OFFSET_Y, TRANSFER_DISTAL[2])))
    body = body.union(_y_boss(0.0155, LUG_LENGTH_Y, (TRANSFER_DISTAL[0], -LUG_OFFSET_Y, TRANSFER_DISTAL[2])))

    body = body.cut(_y_boss(0.0088, 0.030, (0.0, 0.0, 0.0)))
    body = body.cut(_y_boss(0.0085, 0.060, TRANSFER_DISTAL))
    return body


def make_output_lever() -> cq.Workplane:
    beam_mid = _point_on_ray(0.104, OUTPUT_ANGLE_DEG)
    pad_mid = _point_on_ray(0.242, OUTPUT_ANGLE_DEG)
    nose_center = _point_on_ray(0.290, OUTPUT_ANGLE_DEG)
    rib_mid = _point_on_ray(0.126, OUTPUT_ANGLE_DEG)
    hole_a = _point_on_ray(0.230, OUTPUT_ANGLE_DEG)
    hole_b = _point_on_ray(0.268, OUTPUT_ANGLE_DEG)

    body = _y_boss(0.019, CENTER_BOSS_LENGTH_Y, (0.0, 0.0, 0.0))
    body = body.union(_beam_box(0.188, 0.012, 0.018, beam_mid, OUTPUT_ANGLE_DEG))
    body = body.union(_beam_box(0.094, 0.018, 0.044, pad_mid, OUTPUT_ANGLE_DEG))
    body = body.union(_y_boss(0.022, 0.018, nose_center))
    body = body.union(
        _beam_box(0.164, 0.006, 0.012, (rib_mid[0], 0.008, rib_mid[2]), OUTPUT_ANGLE_DEG)
    )

    body = body.cut(_y_boss(0.0085, 0.030, (0.0, 0.0, 0.0)))
    body = body.cut(_y_boss(0.0060, 0.026, hole_a))
    body = body.cut(_y_boss(0.0060, 0.026, hole_b))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_linkage_study")

    base_finish = model.material("base_finish", rgba=(0.23, 0.24, 0.26, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.36, 0.38, 0.40, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.60, 0.62, 0.64, 1.0))
    blued_steel = model.material("blued_steel", rgba=(0.34, 0.38, 0.44, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(make_base_frame(), "base_frame"),
        origin=Origin(),
        material=base_finish,
        name="base_frame",
    )

    input_crank = model.part("input_crank")
    input_crank.visual(
        mesh_from_cadquery(make_input_crank(), "input_crank"),
        origin=Origin(),
        material=blued_steel,
        name="input_crank",
    )

    transfer_lever = model.part("transfer_lever")
    transfer_lever.visual(
        mesh_from_cadquery(make_transfer_lever(), "transfer_lever"),
        origin=Origin(),
        material=satin_steel,
        name="transfer_lever",
    )

    output_lever = model.part("output_lever")
    output_lever.visual(
        mesh_from_cadquery(make_output_lever(), "output_lever"),
        origin=Origin(),
        material=dark_steel,
        name="output_lever",
    )

    model.articulation(
        "base_to_input_crank",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=input_crank,
        origin=Origin(xyz=(0.0, 0.0, ROOT_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.15,
        ),
    )
    model.articulation(
        "input_to_transfer",
        ArticulationType.REVOLUTE,
        parent=input_crank,
        child=transfer_lever,
        origin=Origin(xyz=INPUT_DISTAL),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.45,
            upper=0.18,
        ),
    )
    model.articulation(
        "transfer_to_output",
        ArticulationType.REVOLUTE,
        parent=transfer_lever,
        child=output_lever,
        origin=Origin(xyz=TRANSFER_DISTAL),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.25,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    input_crank = object_model.get_part("input_crank")
    transfer_lever = object_model.get_part("transfer_lever")
    output_lever = object_model.get_part("output_lever")

    base_to_input = object_model.get_articulation("base_to_input_crank")
    input_to_transfer = object_model.get_articulation("input_to_transfer")
    transfer_to_output = object_model.get_articulation("transfer_to_output")

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
        base_frame,
        input_crank,
        reason="Interleaved grounded hinge knuckles share the revolute envelope around an unmodeled through-pin.",
    )
    ctx.allow_overlap(
        input_crank,
        transfer_lever,
        reason="The crank fork and transfer center knuckle nest at the coupling pin envelope.",
    )
    ctx.allow_overlap(
        transfer_lever,
        output_lever,
        reason="The transfer fork and output root knuckle share the modeled pin envelope.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "linkage_parts_present",
        all(
            part is not None
            for part in (base_frame, input_crank, transfer_lever, output_lever)
        ),
        "Expected base frame, input crank, transfer lever, and output lever.",
    )
    ctx.check(
        "serial_linkage_chain",
        base_to_input.parent == "base_frame"
        and base_to_input.child == "input_crank"
        and input_to_transfer.parent == "input_crank"
        and input_to_transfer.child == "transfer_lever"
        and transfer_to_output.parent == "transfer_lever"
        and transfer_to_output.child == "output_lever",
        "The study should be a grounded three-joint serial linkage.",
    )
    ctx.check(
        "parallel_hinge_axes",
        all(
            axis == (0.0, 1.0, 0.0)
            for axis in (
                base_to_input.axis,
                input_to_transfer.axis,
                transfer_to_output.axis,
            )
        ),
        "All three revolute axes should be parallel and aligned to world +Y.",
    )
    ctx.check(
        "progressive_lever_proportions",
        INPUT_LINK_LENGTH < TRANSFER_LINK_LENGTH < OUTPUT_LINK_LENGTH,
        "The linkage should read as short input, medium transfer, and longer output.",
    )
    ctx.expect_origin_gap(
        output_lever,
        input_crank,
        axis="x",
        min_gap=0.25,
        name="clear_input_output_side_bias",
    )

    with ctx.pose(
        base_to_input_crank=-0.35,
        input_to_transfer=-0.45,
        transfer_to_output=-0.25,
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_low_pose")

    with ctx.pose(
        base_to_input_crank=0.15,
        input_to_transfer=0.18,
        transfer_to_output=0.22,
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_high_pose")

    with ctx.pose(
        base_to_input_crank=0.15,
        input_to_transfer=-0.45,
        transfer_to_output=0.22,
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_crossed_output_pose")

    with ctx.pose(
        base_to_input_crank=-0.35,
        input_to_transfer=0.18,
        transfer_to_output=-0.25,
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="clearance_crossed_transfer_pose")

    for articulation in (base_to_input, input_to_transfer, transfer_to_output):
        limits = articulation.motion_limits
        ctx.check(
            f"{articulation.name}_has_motion_limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.effort > 0.0
            and limits.velocity > 0.0,
            f"{articulation.name} should have realistic finite revolute limits.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
