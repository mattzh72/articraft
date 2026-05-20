from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_T = 0.012
PLATE_W = 0.120
PLATE_H = 0.180

SHOULDER_EAR_LEN = 0.032
SHOULDER_EAR_W = 0.012
SHOULDER_GAP = 0.014
SHOULDER_EAR_H = 0.050
SHOULDER_JOINT_X = 0.018
SHOULDER_PIN_R = 0.0055

PROX_TOTAL_LEN = 0.230
PROX_ROOT_LEN = 0.032
PROX_ROOT_W = SHOULDER_GAP
PROX_ROOT_H = 0.036
PROX_BEAM_LEN = 0.150
PROX_BEAM_W = 0.026
PROX_BEAM_H = 0.032
PROX_END_BLOCK_LEN = 0.022
PROX_END_BLOCK_W = 0.038
PROX_END_BLOCK_H = 0.038

ELBOW_EAR_LEN = 0.026
ELBOW_EAR_W = 0.012
ELBOW_GAP = 0.014
ELBOW_EAR_H = 0.038
ELBOW_PIN_R = 0.005
ELBOW_JOINT_X = PROX_TOTAL_LEN - 0.014

DIST_ROOT_LEN = 0.030
DIST_ROOT_W = ELBOW_GAP
DIST_ROOT_H = 0.032
DIST_BEAM_LEN = 0.060
DIST_BEAM_W = 0.024
DIST_BEAM_H = 0.030
GUIDE_REAR_BLOCK_LEN = 0.016
GUIDE_REAR_BLOCK_W = 0.034
GUIDE_REAR_BLOCK_H = 0.034
GUIDE_START = 0.096
GUIDE_LEN = 0.092
GUIDE_OUTER_W = 0.034
GUIDE_OUTER_H = 0.034
GUIDE_RAIL_T = 0.008

RAIL_LEN = 0.070
RAIL_W = 0.014
RAIL_H = 0.014
COLLAR_LEN = 0.012
COLLAR_W = 0.028
COLLAR_H = 0.028
NOSE_BODY_LEN = 0.048
NOSE_BODY_W = 0.026
NOSE_BODY_H = 0.026
NOSE_TIP_LEN = 0.012
NOSE_TIP_R = 0.012
SLIDER_ORIGIN_X = GUIDE_START + GUIDE_LEN - RAIL_LEN

SHOULDER_LIMITS = (-0.40, 1.05)
ELBOW_LIMITS = (0.0, 1.15)
SLIDE_LIMITS = (0.0, 0.040)


def _box_span(x0: float, x1: float, y_size: float, z_size: float, y_center: float = 0.0, z_center: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(x1 - x0, y_size, z_size)
        .translate(((x0 + x1) / 2.0, y_center, z_center))
    )


def _backplate_shape() -> cq.Shape:
    ear_offset = (SHOULDER_GAP / 2.0) + (SHOULDER_EAR_W / 2.0)

    plate = _box_span(-PLATE_T, 0.0, PLATE_W, PLATE_H)
    lower_rib = _box_span(-PLATE_T, 0.0, 0.060, 0.022, z_center=-0.055)
    upper_rib = _box_span(-PLATE_T, 0.0, 0.060, 0.022, z_center=0.055)
    left_ear = _box_span(0.0, SHOULDER_EAR_LEN, SHOULDER_EAR_W, SHOULDER_EAR_H, y_center=-ear_offset)
    right_ear = _box_span(0.0, SHOULDER_EAR_LEN, SHOULDER_EAR_W, SHOULDER_EAR_H, y_center=ear_offset)
    brace = _box_span(-0.002, 0.012, 0.038, 0.062)

    shape = plate.union(lower_rib).union(upper_rib).union(left_ear).union(right_ear).union(brace)
    return shape.val()


def _proximal_link_shape() -> cq.Shape:
    elbow_offset = (ELBOW_GAP / 2.0) + (ELBOW_EAR_W / 2.0)

    root = _box_span(-0.012, 0.020, PROX_ROOT_W, PROX_ROOT_H)
    beam = _box_span(0.020, 0.170, PROX_BEAM_W, PROX_BEAM_H)
    end_block = _box_span(0.170, 0.204, PROX_END_BLOCK_W, PROX_END_BLOCK_H)
    left_ear = _box_span(0.204, PROX_TOTAL_LEN, ELBOW_EAR_W, ELBOW_EAR_H, y_center=-elbow_offset)
    right_ear = _box_span(0.204, PROX_TOTAL_LEN, ELBOW_EAR_W, ELBOW_EAR_H, y_center=elbow_offset)

    shape = root.union(beam).union(end_block).union(left_ear).union(right_ear)
    return shape.val()


def _distal_link_shape() -> cq.Shape:
    outer_half_y = GUIDE_OUTER_W / 2.0
    outer_half_z = GUIDE_OUTER_H / 2.0
    rail_center_y = outer_half_y - (GUIDE_RAIL_T / 2.0)
    rail_center_z = outer_half_z - (GUIDE_RAIL_T / 2.0)

    root = _box_span(-0.012, 0.018, DIST_ROOT_W, DIST_ROOT_H)
    beam = _box_span(0.018, 0.080, DIST_BEAM_W, DIST_BEAM_H)
    rear_block = _box_span(0.080, GUIDE_START, GUIDE_REAR_BLOCK_W, GUIDE_REAR_BLOCK_H)
    top_rail = _box_span(GUIDE_START, GUIDE_START + GUIDE_LEN, GUIDE_OUTER_W, GUIDE_RAIL_T, z_center=rail_center_z)
    bottom_rail = _box_span(GUIDE_START, GUIDE_START + GUIDE_LEN, GUIDE_OUTER_W, GUIDE_RAIL_T, z_center=-rail_center_z)
    left_rail = _box_span(GUIDE_START, GUIDE_START + GUIDE_LEN, GUIDE_RAIL_T, GUIDE_OUTER_H, y_center=-rail_center_y)
    right_rail = _box_span(GUIDE_START, GUIDE_START + GUIDE_LEN, GUIDE_RAIL_T, GUIDE_OUTER_H, y_center=rail_center_y)

    shape = root.union(beam).union(rear_block).union(top_rail).union(bottom_rail).union(left_rail).union(right_rail)
    return shape.val()


def _output_slider_shape() -> cq.Shape:
    rail = _box_span(0.0, RAIL_LEN, RAIL_W, RAIL_H)
    collar = _box_span(RAIL_LEN, RAIL_LEN + COLLAR_LEN, COLLAR_W, COLLAR_H)
    body = _box_span(RAIL_LEN + COLLAR_LEN, RAIL_LEN + COLLAR_LEN + NOSE_BODY_LEN, NOSE_BODY_W, NOSE_BODY_H)
    tip = (
        cq.Workplane("YZ")
        .circle(NOSE_TIP_R)
        .extrude(NOSE_TIP_LEN)
        .translate((RAIL_LEN + COLLAR_LEN + NOSE_BODY_LEN, 0.0, 0.0))
    )
    shape = rail.union(collar).union(body).union(tip)
    return shape.val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_slider_arm")

    model.material("powder_black", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("cast_gray", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("slider_graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("pin_steel", rgba=(0.78, 0.80, 0.83, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_backplate_shape(), "backplate"),
        material="powder_black",
        name="backplate_shell",
    )
    backplate.visual(
        Cylinder(radius=SHOULDER_PIN_R, length=(2.0 * SHOULDER_EAR_W) + SHOULDER_GAP),
        origin=Origin(xyz=(SHOULDER_JOINT_X, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_steel",
        name="shoulder_pin",
    )

    proximal = model.part("proximal_link")
    proximal.visual(
        mesh_from_cadquery(_proximal_link_shape(), "proximal_link"),
        material="cast_gray",
        name="proximal_shell",
    )
    proximal.visual(
        Cylinder(radius=ELBOW_PIN_R, length=(2.0 * ELBOW_EAR_W) + ELBOW_GAP),
        origin=Origin(xyz=(ELBOW_JOINT_X, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_steel",
        name="elbow_pin",
    )

    distal = model.part("distal_link")
    distal.visual(
        mesh_from_cadquery(_distal_link_shape(), "distal_link"),
        material="cast_gray",
        name="distal_shell",
    )

    nose = model.part("output_nose")
    nose.visual(
        mesh_from_cadquery(_output_slider_shape(), "output_nose"),
        material="slider_graphite",
        name="nose_shell",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=proximal,
        origin=Origin(xyz=(SHOULDER_JOINT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.3,
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=distal,
        origin=Origin(xyz=(ELBOW_JOINT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
        ),
    )
    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=distal,
        child=nose,
        origin=Origin(xyz=(SLIDER_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.20,
            lower=SLIDE_LIMITS[0],
            upper=SLIDE_LIMITS[1],
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    proximal = object_model.get_part("proximal_link")
    distal = object_model.get_part("distal_link")
    nose = object_model.get_part("output_nose")
    backplate_shell = backplate.get_visual("backplate_shell")
    proximal_shell = proximal.get_visual("proximal_shell")
    distal_shell = distal.get_visual("distal_shell")
    nose_shell = nose.get_visual("nose_shell")
    shoulder_pin = backplate.get_visual("shoulder_pin")
    elbow_pin = proximal.get_visual("elbow_pin")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    nose_slide = object_model.get_articulation("nose_slide")

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
        backplate,
        proximal,
        elem_a=backplate_shell,
        elem_b=proximal_shell,
        reason="The shoulder is represented as a compact clevis-and-tongue hinge envelope without explicit drilled bores or side-running clearance, so the simplified solids intentionally interpenetrate around the hinge line.",
    )
    ctx.allow_overlap(
        backplate,
        proximal,
        elem_a=shoulder_pin,
        elem_b=proximal_shell,
        reason="Shoulder hinge pin occupies the omitted bore through the proximal root tongue.",
    )
    ctx.allow_overlap(
        proximal,
        distal,
        elem_a=proximal_shell,
        elem_b=distal_shell,
        reason="The elbow is represented as a compact clevis-and-tongue hinge envelope without explicit drilled bores or washer clearances, so the simplified solids intentionally interpenetrate around the hinge line.",
    )
    ctx.allow_overlap(
        proximal,
        distal,
        elem_a=elbow_pin,
        elem_b=distal_shell,
        reason="Elbow hinge pin occupies the omitted bore through the distal root tongue.",
    )
    ctx.allow_overlap(
        distal,
        nose,
        elem_a=distal_shell,
        elem_b=nose_shell,
        reason="The output stage is modeled as a simplified telescoping guide and carriage, so the slider rail intentionally occupies the guide envelope instead of carrying fully modeled internal bearing clearances.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, nose_slide: 0.0}):
        ctx.expect_contact(proximal, backplate, name="shoulder assembly is supported")
        ctx.expect_contact(distal, proximal, name="elbow assembly is supported")
        ctx.expect_contact(nose, distal, name="slider seats against guide face")
        ctx.expect_within(nose, distal, axes="yz", margin=0.0, name="slider remains centered in guide section")

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, nose_slide: 0.0}):
        elbow_rest = ctx.part_world_position(distal)
        nose_straight = ctx.part_world_position(nose)

    with ctx.pose({shoulder_joint: 0.75, elbow_joint: 0.0, nose_slide: 0.0}):
        elbow_raised = ctx.part_world_position(distal)

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.90, nose_slide: 0.0}):
        nose_bent = ctx.part_world_position(nose)

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, nose_slide: 0.05}):
        nose_extended = ctx.part_world_position(nose)

    ctx.check(
        "positive shoulder lifts elbow",
        elbow_rest is not None and elbow_raised is not None and elbow_raised[2] > elbow_rest[2] + 0.14,
        f"expected raised elbow z to exceed rest by at least 0.14 m; rest={elbow_rest}, raised={elbow_raised}",
    )
    ctx.check(
        "positive elbow lifts nose",
        nose_straight is not None and nose_bent is not None and nose_bent[2] > nose_straight[2] + 0.08,
        f"expected bent nose z to exceed straight pose by at least 0.08 m; straight={nose_straight}, bent={nose_bent}",
    )
    ctx.check(
        "slider extends outward",
        nose_straight is not None and nose_extended is not None and nose_extended[0] > nose_straight[0] + 0.045,
        f"expected slider extension to move nose forward along +X; retracted={nose_straight}, extended={nose_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
