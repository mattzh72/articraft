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


def make_support_frame() -> cq.Workplane:
    plate_w = 0.140
    plate_d = 0.090
    plate_t = 0.010
    plate_z = 0.039

    rib_x = 0.014
    rib_y = 0.058
    rib_z = 0.042
    rib_center_z = 0.013
    rib_center_x = 0.036

    slot_centers = [
        (-0.045, -0.028),
        (-0.045, 0.028),
        (0.045, -0.028),
        (0.045, 0.028),
    ]

    plate = (
        cq.Workplane("XY")
        .box(plate_w, plate_d, plate_t)
        .edges("|Z")
        .fillet(0.003)
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints(slot_centers)
        .slot2D(0.016, 0.006, 0.0)
        .cutBlind(-plate_t)
        .translate((0.0, 0.0, plate_z))
    )

    rib_blank = (
        cq.Workplane("XY")
        .box(rib_x, rib_y, rib_z)
        .faces(">X")
        .workplane(centerOption="CenterOfBoundBox")
        .rect(0.036, 0.020)
        .cutThruAll()
        .edges("|X")
        .fillet(0.0015)
    )
    left_rib = rib_blank.translate((-rib_center_x, 0.0, rib_center_z))
    right_rib = rib_blank.translate((rib_center_x, 0.0, rib_center_z))

    return plate.union(left_rib).union(right_rib)


def make_bearing_housing() -> cq.Workplane:
    ring_outer_r = 0.030
    ring_bore_r = 0.016
    ring_t = 0.024

    return (
        cq.Workplane("XY")
        .workplane(offset=-ring_t / 2.0)
        .circle(ring_outer_r)
        .circle(ring_bore_r)
        .extrude(ring_t)
        .edges("%CIRCLE")
        .fillet(0.0015)
    )


def make_rotor_journal() -> cq.Workplane:
    neck_r = 0.0145
    neck_h = 0.034
    neck_center_z = -0.005

    collar_r = 0.027
    collar_h = 0.006
    collar_center_z = -0.0156

    neck = (
        cq.Workplane("XY")
        .workplane(offset=neck_center_z - neck_h / 2.0)
        .circle(neck_r)
        .extrude(neck_h)
    )
    collar = (
        cq.Workplane("XY")
        .workplane(offset=collar_center_z - collar_h / 2.0)
        .circle(collar_r)
        .extrude(collar_h)
    )
    return neck.union(collar)


def make_rotor_body() -> cq.Workplane:
    body_r = 0.036
    body_h = 0.052
    body_center_z = -0.042

    flange_r = 0.041
    flange_h = 0.012
    flange_center_z = -0.074
    body = (
        cq.Workplane("XY")
        .workplane(offset=body_center_z - body_h / 2.0)
        .circle(body_r)
        .extrude(body_h)
    )
    flange = (
        cq.Workplane("XY")
        .workplane(offset=flange_center_z - flange_h / 2.0)
        .circle(flange_r)
        .extrude(flange_h)
    )

    rotor_body = body.union(flange)
    return (
        rotor_body.faces("<Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints([(0.0, 0.022), (0.022, 0.0), (0.0, -0.022), (-0.022, 0.0)])
        .hole(0.006, depth=0.006)
        .faces("<Z")
        .workplane(centerOption="CenterOfBoundBox")
        .circle(0.010)
        .cutBlind(0.004)
        .edges("%CIRCLE and <Z")
        .fillet(0.0015)
    )


def make_service_lug() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.024, 0.016, 0.020)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.031, -0.040))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_yaw_rotary_module")

    bracket_color = model.material(
        "powder_coat_graphite",
        rgba=(0.24, 0.25, 0.27, 1.0),
    )
    rotor_color = model.material(
        "anodized_black",
        rgba=(0.10, 0.10, 0.11, 1.0),
    )

    support_bracket = model.part("support_bracket")
    support_bracket.visual(
        mesh_from_cadquery(make_support_frame(), "support_bracket_frame"),
        origin=Origin(),
        material=bracket_color,
        name="bracket_frame",
    )
    support_bracket.visual(
        mesh_from_cadquery(make_bearing_housing(), "bearing_housing"),
        origin=Origin(),
        material=bracket_color,
        name="bearing_housing",
    )

    rotor_member = model.part("rotor_member")
    rotor_member.visual(
        mesh_from_cadquery(make_rotor_body(), "rotor_body"),
        origin=Origin(),
        material=rotor_color,
        name="rotor_body",
    )
    rotor_member.visual(
        mesh_from_cadquery(make_rotor_journal(), "rotor_journal"),
        origin=Origin(),
        material=rotor_color,
        name="rotor_journal",
    )
    rotor_member.visual(
        mesh_from_cadquery(make_service_lug(), "service_lug"),
        origin=Origin(),
        material=rotor_color,
        name="service_lug",
    )

    model.articulation(
        "support_to_rotor",
        ArticulationType.REVOLUTE,
        parent=support_bracket,
        child=rotor_member,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=2.2,
            lower=-2.6,
            upper=2.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_bracket = object_model.get_part("support_bracket")
    rotor_member = object_model.get_part("rotor_member")
    yaw_joint = object_model.get_articulation("support_to_rotor")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.allow_isolated_part(
        rotor_member,
        reason=(
            "The rotor is intentionally carried with bearing clearance inside the "
            "under-slung housing rather than by zero-clearance mesh contact."
        ),
    )
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

    ctx.expect_origin_distance(
        rotor_member,
        support_bracket,
        axes="xy",
        max_dist=0.0005,
        name="rotor stays concentric with the vertical support axis",
    )
    ctx.expect_within(
        rotor_member,
        support_bracket,
        axes="xy",
        inner_elem="rotor_journal",
        outer_elem="bearing_housing",
        margin=0.004,
        name="rotor journal stays inside the bearing housing envelope",
    )
    ctx.expect_overlap(
        rotor_member,
        support_bracket,
        axes="z",
        elem_a="rotor_journal",
        elem_b="bearing_housing",
        min_overlap=0.020,
        name="rotor journal remains retained inside the under-slung housing",
    )

    limits = yaw_joint.motion_limits
    ctx.check(
        "yaw module uses one vertical revolute axis",
        yaw_joint.joint_type == ArticulationType.REVOLUTE
        and yaw_joint.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=(
            f"type={yaw_joint.joint_type}, axis={yaw_joint.axis}, "
            f"limits={None if limits is None else (limits.lower, limits.upper)}"
        ),
    )

    def aabb_center(aabb) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    rest_lug_aabb = ctx.part_element_world_aabb(rotor_member, elem="service_lug")
    with ctx.pose({yaw_joint: math.pi / 2.0}):
        quarter_turn_lug_aabb = ctx.part_element_world_aabb(rotor_member, elem="service_lug")

    rest_lug_center = aabb_center(rest_lug_aabb)
    turn_lug_center = aabb_center(quarter_turn_lug_aabb)
    ctx.check(
        "offset output hardware rotates around the vertical axis",
        rest_lug_center is not None
        and turn_lug_center is not None
        and abs(rest_lug_center[1]) > abs(rest_lug_center[0]) + 0.015
        and abs(turn_lug_center[0]) > abs(turn_lug_center[1]) + 0.015,
        details=(
            f"rest_center={rest_lug_center}, "
            f"quarter_turn_center={turn_lug_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
