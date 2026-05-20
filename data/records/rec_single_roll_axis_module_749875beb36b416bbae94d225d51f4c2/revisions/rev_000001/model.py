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


CENTERLINE_Z = 0.155
SUPPORT_X = 0.195
SUPPORT_DEPTH = 0.024

RAIL_LENGTH = 0.520
RAIL_Y = 0.090
RAIL_W = 0.032
RAIL_H = 0.028
CROSSBAR_X = 0.072
CROSSBAR_Y = 0.200
FOOT_X = 0.100
FOOT_Y = 0.210
FOOT_H = 0.010

OUTER_HOOP_R = 0.120
HOOP_INNER_R = 0.096
HUB_OUTER_R = 0.050
HUB_INNER_R = 0.033
SPOKE_W = 0.018
SUPPORT_LOCAL_Z = CENTERLINE_Z - RAIL_H
SHOE_X = 0.072
SHOE_Y = 0.130
SHOE_H = 0.012
POST_X = 0.030
POST_Y = 0.018
POST_OFFSET_Y = 0.075
POST_H = 0.118

CARRIER_OUTER_R = 0.080
CARRIER_INNER_R = 0.072
CARRIER_TUBE_L = 0.300
SHAFT_R = HUB_INNER_R
SHAFT_L = 0.438
COLLAR_R = HUB_OUTER_R
COLLAR_T = 0.012
COLLAR_CENTER_X = SUPPORT_X + SUPPORT_DEPTH / 2.0 + COLLAR_T / 2.0

BRACKET_LEN = 0.070
BRACKET_W = 0.030
BRACKET_BASE_H = 0.016
BRACKET_FLANGE_W = 0.014
BRACKET_FLANGE_H = 0.050
BRACKET_BASE_Z0 = CARRIER_OUTER_R - 0.008
BRACKET_FLANGE_X = 0.010
BRACKET_FLANGE_Z0 = CARRIER_OUTER_R + 0.004


def _centered_annulus(outer_r: float, inner_r: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _centered_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate(
        (-length / 2.0, 0.0, 0.0)
    )


def make_base_geometry() -> cq.Workplane:
    left_rail = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_W, RAIL_H, centered=(True, True, False))
        .translate((0.0, RAIL_Y, 0.0))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_W, RAIL_H, centered=(True, True, False))
        .translate((0.0, -RAIL_Y, 0.0))
    )
    front_crossbar = (
        cq.Workplane("XY")
        .box(CROSSBAR_X, CROSSBAR_Y, RAIL_H, centered=(True, True, False))
        .translate((-SUPPORT_X, 0.0, 0.0))
    )
    rear_crossbar = (
        cq.Workplane("XY")
        .box(CROSSBAR_X, CROSSBAR_Y, RAIL_H, centered=(True, True, False))
        .translate((SUPPORT_X, 0.0, 0.0))
    )
    front_foot = (
        cq.Workplane("XY")
        .box(FOOT_X, FOOT_Y, FOOT_H, centered=(True, True, False))
        .translate((-SUPPORT_X, 0.0, 0.0))
    )
    rear_foot = (
        cq.Workplane("XY")
        .box(FOOT_X, FOOT_Y, FOOT_H, centered=(True, True, False))
        .translate((SUPPORT_X, 0.0, 0.0))
    )
    return (
        left_rail.union(right_rail)
        .union(front_crossbar)
        .union(rear_crossbar)
        .union(front_foot)
        .union(rear_foot)
    )


def make_support_geometry() -> cq.Workplane:
    hoop = _centered_annulus(OUTER_HOOP_R, HOOP_INNER_R, SUPPORT_DEPTH).translate(
        (0.0, 0.0, SUPPORT_LOCAL_Z)
    )
    hub = _centered_annulus(HUB_OUTER_R, HUB_INNER_R, SUPPORT_DEPTH).translate(
        (0.0, 0.0, SUPPORT_LOCAL_Z)
    )
    rib_seed = (
        cq.Workplane("YZ")
        .rect(0.052, SPOKE_W)
        .extrude(SUPPORT_DEPTH)
        .translate((-SUPPORT_DEPTH / 2.0, 0.0, SUPPORT_LOCAL_Z + 0.067))
    )
    rib_axis_start = (0.0, 0.0, SUPPORT_LOCAL_Z)
    rib_axis_end = (1.0, 0.0, SUPPORT_LOCAL_Z)
    ribs = (
        rib_seed.rotate(rib_axis_start, rib_axis_end, 35.0)
        .union(rib_seed.rotate(rib_axis_start, rib_axis_end, -35.0))
        .union(rib_seed.rotate(rib_axis_start, rib_axis_end, 145.0))
        .union(rib_seed.rotate(rib_axis_start, rib_axis_end, -145.0))
    )
    shoe = cq.Workplane("XY").box(SHOE_X, SHOE_Y, SHOE_H, centered=(True, True, False))
    left_post = (
        cq.Workplane("XY")
        .box(POST_X, POST_Y, POST_H, centered=(True, True, False))
        .translate((0.0, POST_OFFSET_Y, 0.0))
    )
    right_post = (
        cq.Workplane("XY")
        .box(POST_X, POST_Y, POST_H, centered=(True, True, False))
        .translate((0.0, -POST_OFFSET_Y, 0.0))
    )
    return (
        shoe.union(left_post)
        .union(right_post)
        .union(hoop)
        .union(hub)
        .union(ribs)
    )


def make_bracket_geometry() -> cq.Workplane:
    saddle_blank = (
        cq.Workplane("XY")
        .box(BRACKET_LEN, BRACKET_W, BRACKET_BASE_H, centered=(True, True, False))
        .translate((0.0, 0.0, BRACKET_BASE_Z0))
    )
    flange = (
        cq.Workplane("XY")
        .box(
            BRACKET_FLANGE_W,
            BRACKET_W * 1.75,
            BRACKET_FLANGE_H,
            centered=(True, True, False),
        )
        .translate((BRACKET_FLANGE_X, 0.0, BRACKET_FLANGE_Z0))
    )
    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.012, CARRIER_OUTER_R + BRACKET_BASE_H),
                (0.016, CARRIER_OUTER_R + BRACKET_BASE_H),
                (0.016, CARRIER_OUTER_R + BRACKET_BASE_H + 0.028),
            ]
        )
        .close()
        .extrude(BRACKET_W * 0.75)
        .translate((0.0, -BRACKET_W * 0.375, 0.0))
    )
    saddle_cut = _centered_cylinder(CARRIER_OUTER_R - 0.001, BRACKET_LEN + 0.020)
    return saddle_blank.union(flange).union(gusset).cut(saddle_cut)


def make_carrier_geometry() -> cq.Workplane:
    tube = _centered_annulus(CARRIER_OUTER_R, CARRIER_INNER_R, CARRIER_TUBE_L)
    shaft = _centered_cylinder(SHAFT_R, SHAFT_L)
    return tube.union(shaft)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tube_rotator_module")

    frame_mat = model.material("frame_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    carrier_mat = model.material("carrier_satin", rgba=(0.76, 0.78, 0.80, 1.0))
    bracket_mat = model.material("bracket_orange", rgba=(0.82, 0.38, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_geometry(), "tube_rotator_base"),
        origin=Origin(),
        material=frame_mat,
        name="base_shell",
    )

    front_support = model.part("front_support")
    front_support.visual(
        mesh_from_cadquery(make_support_geometry(), "tube_rotator_front_support"),
        origin=Origin(),
        material=frame_mat,
        name="support_shell",
    )

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(make_support_geometry(), "tube_rotator_rear_support"),
        origin=Origin(),
        material=frame_mat,
        name="support_shell",
    )

    carrier = model.part("carrier")
    carrier.visual(
        mesh_from_cadquery(make_carrier_geometry(), "tube_rotator_carrier"),
        origin=Origin(),
        material=carrier_mat,
        name="carrier_shell",
    )
    carrier.visual(
        mesh_from_cadquery(make_bracket_geometry(), "tube_rotator_bracket"),
        origin=Origin(),
        material=bracket_mat,
        name="bracket_shell",
    )

    model.articulation(
        "base_to_front_support",
        ArticulationType.FIXED,
        parent=base,
        child=front_support,
        origin=Origin(xyz=(-SUPPORT_X, 0.0, RAIL_H)),
    )

    model.articulation(
        "base_to_rear_support",
        ArticulationType.FIXED,
        parent=base,
        child=rear_support,
        origin=Origin(xyz=(SUPPORT_X, 0.0, RAIL_H)),
    )

    model.articulation(
        "carrier_roll",
        ArticulationType.REVOLUTE,
        parent=base,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, CENTERLINE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    front_support = object_model.get_part("front_support")
    rear_support = object_model.get_part("rear_support")
    carrier = object_model.get_part("carrier")
    carrier_roll = object_model.get_articulation("carrier_roll")
    bracket_visual = carrier.get_visual("bracket_shell")
    carrier_shell = carrier.get_visual("carrier_shell")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        carrier,
        front_support,
        reason="Hidden coaxial sleeve-bearing interface is modeled as a zero-clearance nest inside the front hoop hub.",
    )
    ctx.allow_overlap(
        carrier,
        rear_support,
        reason="Hidden coaxial sleeve-bearing interface is modeled as a zero-clearance nest inside the rear hoop hub.",
    )

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
        "tube_rotator_parts_present",
        all(part is not None for part in (base, front_support, rear_support, carrier)),
        "base, both support hoops, and carrier must all exist",
    )
    ctx.check(
        "carrier_has_bracket_visual",
        bracket_visual is not None,
        "carrier should include a mounted bracket visual",
    )
    ctx.check(
        "carrier_roll_axis_is_longitudinal",
        tuple(round(v, 6) for v in carrier_roll.axis) == (1.0, 0.0, 0.0),
        f"expected x-axis roll, got {carrier_roll.axis}",
    )
    ctx.expect_origin_distance(
        front_support,
        rear_support,
        axes="yz",
        min_dist=0.0,
        max_dist=0.001,
        name="support_hoops_share_centerline_height",
    )
    ctx.expect_origin_gap(
        rear_support,
        front_support,
        axis="x",
        min_gap=0.389,
        max_gap=0.391,
        name="support_hoops_are_balanced_about_carrier",
    )
    ctx.expect_contact(
        front_support,
        base,
        contact_tol=0.001,
        name="front_support_mounted_to_base",
    )
    ctx.expect_contact(
        rear_support,
        base,
        contact_tol=0.001,
        name="rear_support_mounted_to_base",
    )
    ctx.expect_overlap(
        carrier,
        front_support,
        axes="yz",
        elem_a=carrier_shell,
        min_overlap=0.150,
        name="carrier_aligned_with_front_hoop",
    )
    ctx.expect_overlap(
        carrier,
        rear_support,
        axes="yz",
        elem_a=carrier_shell,
        min_overlap=0.150,
        name="carrier_aligned_with_rear_hoop",
    )

    with ctx.pose({carrier_roll: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no_interference_at_quarter_turn"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
