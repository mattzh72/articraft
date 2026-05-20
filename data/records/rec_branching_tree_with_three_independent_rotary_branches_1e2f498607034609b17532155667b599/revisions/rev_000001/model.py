from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


MAST_WIDTH = 0.08
MAST_DEPTH = 0.06
MAST_HEIGHT = 0.46
BASE_PLATE_THICKNESS = 0.02

LOWER_JOINT_ORIGIN = (0.0, -0.052, 0.145)
FORWARD_JOINT_ORIGIN = (0.068, 0.0, 0.273)
UPPER_JOINT_ORIGIN = (0.132, 0.0, 0.405)


def box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((cx - length / 2.0, cy, cz))


def cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((cx, cy - length / 2.0, cz))


def cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((cx, cy, cz - length / 2.0))


def fuse_all(base: cq.Workplane, *extras: cq.Workplane) -> cq.Workplane:
    shape = base
    for extra in extras:
        shape = shape.union(extra)
    return shape


def rotate_about(
    shape: cq.Workplane,
    center: tuple[float, float, float],
    axis: tuple[float, float, float],
    angle_deg: float,
) -> cq.Workplane:
    cx, cy, cz = center
    ax, ay, az = axis
    return shape.rotate((cx, cy, cz), (cx + ax, cy + ay, cz + az), angle_deg)


def make_spine() -> cq.Workplane:
    base_plate = box_at((0.18, 0.16, BASE_PLATE_THICKNESS), (0.0, 0.0, BASE_PLATE_THICKNESS / 2.0))
    base_holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.055, -0.045), (-0.055, 0.045), (0.055, -0.045), (0.055, 0.045)])
        .circle(0.007)
        .extrude(0.035)
        .translate((0.0, 0.0, -0.007))
    )
    base_plate = base_plate.cut(base_holes)

    mast = box_at((MAST_WIDTH, MAST_DEPTH, MAST_HEIGHT), (0.0, 0.0, BASE_PLATE_THICKNESS + MAST_HEIGHT / 2.0))
    mast_top_cap = box_at((0.096, 0.072, 0.014), (0.0, 0.0, BASE_PLATE_THICKNESS + MAST_HEIGHT - 0.007))

    lower_back_plate = box_at((0.074, 0.014, 0.145), (0.0, -0.037, 0.145))
    lower_bearing_block = box_at((0.074, 0.03, 0.072), (0.0, -0.051, 0.145))
    lower_cap_top = box_at((0.058, 0.024, 0.016), (0.0, -0.051, 0.173))
    lower_cap_bottom = box_at((0.058, 0.024, 0.016), (0.0, -0.051, 0.117))
    lower_gusset = rotate_about(
        box_at((0.056, 0.016, 0.082), (0.0, -0.041, 0.103)),
        (0.0, -0.041, 0.103),
        (1.0, 0.0, 0.0),
        -24.0,
    )
    lower_support = fuse_all(lower_back_plate, lower_bearing_block, lower_cap_top, lower_cap_bottom, lower_gusset)
    lower_support = lower_support.cut(cyl_x(0.0165, 0.088, LOWER_JOINT_ORIGIN))
    lower_support = lower_support.cut(box_at((0.05, 0.022, 0.042), (0.0, -0.068, 0.145)))

    forward_back_plate = box_at((0.014, 0.078, 0.122), (0.04, 0.0, 0.273))
    forward_collar_block = box_at((0.026, 0.06, 0.056), (0.054, 0.0, 0.273))
    forward_top_cap = box_at((0.022, 0.044, 0.014), (0.054, 0.0, 0.306))
    forward_bottom_cap = box_at((0.022, 0.044, 0.014), (0.054, 0.0, 0.24))
    forward_gusset = rotate_about(
        box_at((0.046, 0.014, 0.078), (0.043, 0.0, 0.235)),
        (0.043, 0.0, 0.235),
        (0.0, 1.0, 0.0),
        26.0,
    )
    forward_support = fuse_all(
        forward_back_plate,
        forward_collar_block,
        forward_top_cap,
        forward_bottom_cap,
        forward_gusset,
    )
    forward_support = forward_support.cut(cyl_z(0.0155, 0.074, FORWARD_JOINT_ORIGIN))
    forward_support = forward_support.cut(
        cyl_z(0.0245, 0.012, (FORWARD_JOINT_ORIGIN[0], FORWARD_JOINT_ORIGIN[1], FORWARD_JOINT_ORIGIN[2] + 0.028))
    )

    upper_back_plate = box_at((0.016, 0.084, 0.12), (0.04, 0.0, 0.405))
    upper_left_strut = box_at((0.112, 0.012, 0.018), (0.08, -0.038, 0.405))
    upper_right_strut = box_at((0.112, 0.012, 0.018), (0.08, 0.038, 0.405))
    upper_ear_left = box_at((0.026, 0.012, 0.056), (0.119, -0.032, 0.405))
    upper_ear_right = box_at((0.026, 0.012, 0.056), (0.119, 0.032, 0.405))
    upper_left_brace_top = rotate_about(
        box_at((0.066, 0.012, 0.04), (0.084, -0.038, 0.432)),
        (0.084, -0.038, 0.432),
        (0.0, 1.0, 0.0),
        14.0,
    )
    upper_left_brace_bottom = rotate_about(
        box_at((0.066, 0.012, 0.04), (0.084, -0.038, 0.378)),
        (0.084, -0.038, 0.378),
        (0.0, 1.0, 0.0),
        -14.0,
    )
    upper_right_brace_top = rotate_about(
        box_at((0.066, 0.012, 0.04), (0.084, 0.038, 0.432)),
        (0.084, 0.038, 0.432),
        (0.0, 1.0, 0.0),
        14.0,
    )
    upper_right_brace_bottom = rotate_about(
        box_at((0.066, 0.012, 0.04), (0.084, 0.038, 0.378)),
        (0.084, 0.038, 0.378),
        (0.0, 1.0, 0.0),
        -14.0,
    )
    upper_shaft = cyl_y(0.0145, 0.052, UPPER_JOINT_ORIGIN)
    upper_support = fuse_all(
        upper_back_plate,
        upper_left_strut,
        upper_right_strut,
        upper_ear_left,
        upper_ear_right,
        upper_left_brace_top,
        upper_left_brace_bottom,
        upper_right_brace_top,
        upper_right_brace_bottom,
        upper_shaft,
    )

    return fuse_all(base_plate, mast, mast_top_cap, lower_support, forward_support, upper_support)


def make_lower_arm() -> cq.Workplane:
    hub = cyl_x(0.016, 0.052, (0.0, 0.0, 0.0))
    root_block = box_at((0.046, 0.018, 0.028), (0.0, -0.075, -0.01))
    beam = box_at((0.044, 0.118, 0.024), (0.0, -0.142, -0.018))
    neck = box_at((0.052, 0.038, 0.02), (0.0, -0.187, -0.02))
    pad = box_at((0.094, 0.058, 0.012), (0.0, -0.232, -0.022))
    gusset = rotate_about(
        box_at((0.048, 0.018, 0.04), (0.0, -0.102, -0.028)),
        (0.0, -0.102, -0.028),
        (1.0, 0.0, 0.0),
        20.0,
    )

    arm = fuse_all(hub, root_block, beam, neck, pad, gusset)
    slot_left = cq.Workplane("XY").slot2D(0.028, 0.01).extrude(0.03).translate((-0.022, -0.232, -0.038))
    slot_right = cq.Workplane("XY").slot2D(0.028, 0.01).extrude(0.03).translate((0.022, -0.232, -0.038))
    arm = arm.cut(slot_left).cut(slot_right)
    return arm


def make_forward_arm() -> cq.Workplane:
    post = cyl_z(0.015, 0.048, (0.0, 0.0, 0.0))
    thrust_plate = cyl_z(0.024, 0.01, (0.0, 0.0, 0.028))
    neck = box_at((0.024, 0.036, 0.018), (0.048, 0.0, 0.03))
    beam = box_at((0.118, 0.028, 0.022), (0.125, 0.0, 0.038))
    brace = rotate_about(
        box_at((0.046, 0.018, 0.032), (0.08, 0.0, 0.022)),
        (0.08, 0.0, 0.022),
        (0.0, 1.0, 0.0),
        -18.0,
    )
    flange_neck = box_at((0.032, 0.024, 0.024), (0.182, 0.0, 0.038))
    flange = cyl_x(0.036, 0.012, (0.204, 0.0, 0.038))
    arm = fuse_all(post, thrust_plate, neck, beam, brace, flange_neck, flange)
    arm = arm.cut(cyl_x(0.011, 0.022, (0.204, 0.0, 0.038)))
    return arm


def make_upper_arm() -> cq.Workplane:
    hub = box_at((0.04, 0.046, 0.034), (0.012, 0.0, 0.0))
    beam = box_at((0.126, 0.022, 0.02), (0.102, 0.0, 0.004))
    top_rib = rotate_about(
        box_at((0.05, 0.014, 0.024), (0.078, 0.0, 0.014)),
        (0.078, 0.0, 0.014),
        (0.0, 1.0, 0.0),
        12.0,
    )
    bottom_rib = rotate_about(
        box_at((0.046, 0.014, 0.02), (0.086, 0.0, -0.01)),
        (0.086, 0.0, -0.01),
        (0.0, 1.0, 0.0),
        -9.0,
    )
    pad_boss = box_at((0.03, 0.028, 0.034), (0.168, 0.0, 0.004))
    pad = box_at((0.012, 0.078, 0.078), (0.188, 0.0, 0.004))
    arm = fuse_all(hub, beam, top_rib, bottom_rib, pad_boss, pad)
    arm = arm.cut(cyl_y(0.0154, 0.062, (0.0, 0.0, 0.0)))
    slot_left = cq.Workplane("YZ").slot2D(0.034, 0.01, angle=90).extrude(0.024).translate((0.176, -0.022, 0.004))
    slot_right = cq.Workplane("YZ").slot2D(0.034, 0.01, angle=90).extrude(0.024).translate((0.176, 0.022, 0.004))
    arm = arm.cut(slot_left).cut(slot_right)
    return arm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_fixture_head")

    fixture_gray = model.material("fixture_gray", rgba=(0.29, 0.31, 0.33, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.18, 0.19, 0.2, 1.0))
    machine_steel = model.material("machine_steel", rgba=(0.53, 0.55, 0.58, 1.0))
    satin_alloy = model.material("satin_alloy", rgba=(0.66, 0.68, 0.72, 1.0))

    spine = model.part("spine")
    spine.visual(mesh_from_cadquery(make_spine(), "spine_fixture_head"), material=fixture_gray, name="spine_body")
    spine.inertial = Inertial.from_geometry(
        Box((0.18, 0.16, 0.49)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(mesh_from_cadquery(make_lower_arm(), "lower_arm_body"), material=dark_oxide, name="lower_arm_body")
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.10, 0.24, 0.06)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.12, 0.0)),
    )

    forward_arm = model.part("forward_arm")
    forward_arm.visual(
        mesh_from_cadquery(make_forward_arm(), "forward_arm_body"),
        material=machine_steel,
        name="forward_arm_body",
    )
    forward_arm.inertial = Inertial.from_geometry(
        Box((0.22, 0.09, 0.09)),
        mass=1.1,
        origin=Origin(xyz=(0.10, 0.0, 0.032)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(mesh_from_cadquery(make_upper_arm(), "upper_arm_body"), material=satin_alloy, name="upper_arm_body")
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.21, 0.09, 0.09)),
        mass=1.2,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
    )

    model.articulation(
        "lower_arm_joint",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_arm,
        origin=Origin(xyz=LOWER_JOINT_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.65, upper=0.82),
    )

    model.articulation(
        "forward_arm_joint",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=forward_arm,
        origin=Origin(xyz=FORWARD_JOINT_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=-0.95, upper=1.15),
    )

    model.articulation(
        "upper_arm_joint",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_arm,
        origin=Origin(xyz=UPPER_JOINT_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=-0.22, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    lower_arm = object_model.get_part("lower_arm")
    forward_arm = object_model.get_part("forward_arm")
    upper_arm = object_model.get_part("upper_arm")

    lower_joint = object_model.get_articulation("lower_arm_joint")
    forward_joint = object_model.get_articulation("forward_arm_joint")
    upper_joint = object_model.get_articulation("upper_arm_joint")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_isolated_part(
        upper_arm,
        reason="upper arm is mechanically captured on a longitudinal support shaft with modeled running clearance",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.001)
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
        "joint axes follow transverse vertical longitudinal brief",
        lower_joint.axis == (1.0, 0.0, 0.0)
        and forward_joint.axis == (0.0, 0.0, 1.0)
        and upper_joint.axis == (0.0, 1.0, 0.0),
        details=(
            f"lower={lower_joint.axis}, forward={forward_joint.axis}, upper={upper_joint.axis}"
        ),
    )

    ctx.expect_contact(lower_arm, spine, contact_tol=0.001, name="lower arm trunnion is seated in the side block")
    ctx.expect_contact(forward_arm, spine, contact_tol=0.001, name="forward arm post is seated in the collar")
    ctx.expect_contact(
        upper_arm,
        spine,
        contact_tol=0.005,
        name="upper arm journals run closely on the captured support shaft",
    )

    ctx.expect_origin_gap(
        positive_link=spine,
        negative_link=lower_arm,
        axis="y",
        min_gap=0.045,
        name="lower branch is carried off the side of the mast",
    )
    ctx.expect_origin_gap(
        positive_link=forward_arm,
        negative_link=spine,
        axis="x",
        min_gap=0.06,
        name="forward branch is carried proud of the mast face",
    )
    ctx.expect_origin_gap(
        positive_link=upper_arm,
        negative_link=spine,
        axis="x",
        min_gap=0.055,
        name="upper branch support projects ahead of the mast",
    )
    ctx.expect_origin_gap(
        positive_link=forward_arm,
        negative_link=lower_arm,
        axis="z",
        min_gap=0.10,
        name="forward branch sits well above the lower branch",
    )
    ctx.expect_origin_gap(
        positive_link=upper_arm,
        negative_link=forward_arm,
        axis="z",
        min_gap=0.11,
        name="upper branch sits well above the forward branch",
    )

    with ctx.pose({lower_joint: 0.82, upper_joint: -0.12}):
        ctx.fail_if_parts_overlap_in_current_pose(name="pitched branch clearance pose")
        ctx.expect_gap(
            positive_link=forward_arm,
            negative_link=lower_arm,
            axis="z",
            min_gap=0.025,
            name="raised lower arm stays below the forward branch",
        )
        ctx.expect_gap(
            positive_link=upper_arm,
            negative_link=forward_arm,
            axis="z",
            min_gap=0.02,
            name="dipped upper arm still clears the forward branch",
        )

    with ctx.pose({forward_joint: 1.15, lower_joint: -0.4, upper_joint: 0.18}):
        ctx.fail_if_parts_overlap_in_current_pose(name="yawed forward branch clearance pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
