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


PLATE_L = 0.36
PLATE_W = 0.22
PLATE_T = 0.018

HOUSING_L = 0.165
HOUSING_W = 0.145
HOUSING_H = 0.100

RIB_L = 0.135
RIB_T = 0.018
RIB_H = 0.052

LOWER_BOSS_R = 0.040
LOWER_BOSS_H = 0.016
UPPER_FLANGE_R = 0.050
UPPER_FLANGE_H = 0.012

BORE_R = 0.023
SHAFT_R = 0.019
SHAFT_LEN = 0.108
COLLAR_R = 0.029
COLLAR_T = 0.008
TABLE_R = 0.068
TABLE_T = 0.018
PILOT_R = 0.022
PILOT_H = 0.010
HUB_R = 0.032
HUB_H = 0.014
FASTENER_R = 0.0055
FASTENER_H = 0.003
FASTENER_SINK = 0.001
FUSE_EPS = 0.001
HEAD_TABLE_TOP_Z = COLLAR_T + HUB_H + TABLE_T

ARTIC_Z = PLATE_T + HOUSING_H + LOWER_BOSS_H + UPPER_FLANGE_H


def polar_points(radius: float, count: int, start_deg: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(math.radians(start_deg + step * 360.0 / count)),
            radius * math.sin(math.radians(start_deg + step * 360.0 / count)),
        )
        for step in range(count)
    ]


def make_base_body() -> cq.Workplane:
    slot_points = [
        (-0.125, -0.075),
        (-0.125, 0.075),
        (0.125, -0.075),
        (0.125, 0.075),
    ]

    plate = (
        cq.Workplane("XY")
        .box(PLATE_L, PLATE_W, PLATE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
    )
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(slot_points)
        .slot2D(0.028, 0.010, angle=0)
        .cutBlind(-PLATE_T)
    )

    housing = (
        cq.Workplane("XY")
        .box(HOUSING_L, HOUSING_W, HOUSING_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, PLATE_T))
    )

    rib_seed = (
        cq.Workplane("XY")
        .box(RIB_L, RIB_T, RIB_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    rib_offset = HOUSING_W * 0.5 + RIB_T * 0.5 - 0.005
    left_rib = rib_seed.translate((0.0, rib_offset, PLATE_T))
    right_rib = rib_seed.translate((0.0, -rib_offset, PLATE_T))

    lower_boss = (
        cq.Workplane("XY")
        .circle(LOWER_BOSS_R)
        .extrude(LOWER_BOSS_H)
        .translate((0.0, 0.0, PLATE_T + HOUSING_H))
    )
    upper_flange = (
        cq.Workplane("XY")
        .circle(UPPER_FLANGE_R)
        .extrude(UPPER_FLANGE_H)
        .translate((0.0, 0.0, PLATE_T + HOUSING_H + LOWER_BOSS_H))
    )

    front_pointer = (
        cq.Workplane("XY")
        .box(0.014, 0.040, 0.004, centered=(True, True, False))
        .translate((0.0, UPPER_FLANGE_R + 0.007, ARTIC_Z - 0.004))
    )

    body = (
        plate.union(housing)
        .union(left_rib)
        .union(right_rib)
        .union(lower_boss)
        .union(upper_flange)
        .union(front_pointer)
    )

    bore_depth = ARTIC_Z - 0.030
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(BORE_R)
        .cutBlind(-bore_depth)
    )

    return body.clean()


def make_head_body() -> cq.Workplane:
    body = cq.Workplane("XY").circle(COLLAR_R).extrude(COLLAR_T)
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(HUB_R)
        .extrude(HUB_H)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(TABLE_R)
        .extrude(TABLE_T)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(PILOT_R)
        .extrude(PILOT_H)
    )

    fasteners = (
        cq.Workplane("XY")
        .pushPoints(polar_points(0.045, 4, start_deg=45.0))
        .circle(FASTENER_R)
        .extrude(FASTENER_H)
        .translate((0.0, 0.0, HEAD_TABLE_TOP_Z - FASTENER_SINK))
    )

    return body.union(fasteners).clean()


def make_index_marker() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.014, 0.008, 0.0025, centered=(True, True, False))
        .translate((TABLE_R - 0.012, 0.0, HEAD_TABLE_TOP_Z))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_rotary_module")

    model.material("painted_casting", color=(0.70, 0.73, 0.77))
    model.material("machined_steel", color=(0.74, 0.76, 0.78))
    model.material("indicator_red", color=(0.75, 0.16, 0.14))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_body(), "base_body"),
        origin=Origin(),
        material="painted_casting",
        name="base_body",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(make_head_body(), "head_body"),
        origin=Origin(),
        material="machined_steel",
        name="head_body",
    )
    head.visual(
        mesh_from_cadquery(make_index_marker(), "index_marker"),
        origin=Origin(),
        material="indicator_red",
        name="index_marker",
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, ARTIC_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotary = object_model.get_articulation("base_to_head")

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

    ctx.check(
        "parts_present",
        base is not None and head is not None and rotary is not None,
        "Expected base, head, and base_to_head articulation.",
    )
    ctx.check(
        "rotary_axis_vertical",
        tuple(round(v, 6) for v in rotary.axis) == (0.0, 0.0, 1.0),
        f"Rotary axis should be vertical +Z, got {rotary.axis!r}.",
    )
    limits = rotary.motion_limits
    ctx.check(
        "rotary_limits_span_zero",
        limits is not None and limits.lower is not None and limits.upper is not None and limits.lower < 0.0 < limits.upper,
        "Rotary head should rotate about its rest pose in both directions.",
    )

    ctx.expect_contact(
        head,
        base,
        elem_a="head_body",
        elem_b="base_body",
        contact_tol=5e-4,
        name="head_seated_on_bearing_support",
    )
    ctx.expect_origin_gap(
        head,
        base,
        axis="z",
        min_gap=0.140,
        max_gap=0.152,
        name="head_origin_above_support_column",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="xy",
        elem_a="head_body",
        elem_b="base_body",
        min_overlap=0.110,
        name="head_centered_over_grounded_base",
    )

    marker_rest = ctx.part_element_world_aabb(head, elem="index_marker")
    marker_rest_center = None
    if marker_rest is not None:
        marker_rest_center = tuple(
            (marker_rest[0][i] + marker_rest[1][i]) * 0.5 for i in range(3)
        )

    with ctx.pose({rotary: 1.2}):
        marker_turned = ctx.part_element_world_aabb(head, elem="index_marker")

    marker_moved = False
    marker_details = "Marker AABB could not be resolved."
    if marker_rest_center is not None and marker_turned is not None:
        marker_turned_center = tuple(
            (marker_turned[0][i] + marker_turned[1][i]) * 0.5 for i in range(3)
        )
        xy_shift = math.hypot(
            marker_turned_center[0] - marker_rest_center[0],
            marker_turned_center[1] - marker_rest_center[1],
        )
        z_shift = abs(marker_turned_center[2] - marker_rest_center[2])
        marker_moved = xy_shift > 0.020 and z_shift < 1e-5
        marker_details = (
            f"Expected visible rotary motion; marker xy_shift={xy_shift:.4f} m, "
            f"z_shift={z_shift:.6f} m."
        )
    ctx.check("index_marker_moves_with_head", marker_moved, marker_details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
