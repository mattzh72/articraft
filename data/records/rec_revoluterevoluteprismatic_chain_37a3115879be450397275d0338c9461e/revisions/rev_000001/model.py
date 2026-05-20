from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_RADIUS = 0.085
BASE_THICKNESS = 0.012
PEDESTAL_WIDTH_X = 0.060
PEDESTAL_DEPTH_Y = 0.048
PEDESTAL_HEIGHT = 0.074

SHOULDER_GAP = 0.028
HINGE_PLATE_THICKNESS = 0.008
SHOULDER_CLEVIS_X = 0.018
SHOULDER_CLEVIS_HEIGHT = 0.052
SHOULDER_Z = 0.104

UPPER_LENGTH = 0.100
UPPER_BODY_WIDTH = 0.022
UPPER_BODY_HEIGHT = 0.022
ELBOW_X = 0.100

FOREARM_BODY_LENGTH = 0.086
FOREARM_BODY_WIDTH = 0.024
FOREARM_BODY_HEIGHT = 0.024
SLIDE_GUIDE_X = 0.194
NOSE_LENGTH = 0.082
NOSE_OUTER_RADIUS = 0.018
NOSE_INNER_RADIUS = 0.0115
SLIDE_STROKE = 0.060

PIN_RADIUS = 0.006
TANG_LENGTH = 0.018
TANG_WIDTH = SHOULDER_GAP
TANG_HEIGHT = 0.030
ARM_BEAM_WIDTH = 0.018
ARM_BEAM_HEIGHT = 0.018
CLEVIS_PLATE_LENGTH = 0.016
CLEVIS_PLATE_HEIGHT = 0.040
CLEVIS_OFFSET_Y = SHOULDER_GAP / 2.0 + HINGE_PLATE_THICKNESS / 2.0


def x_box(length: float, width_y: float, height_z: float, *, x0: float = 0.0, z: float = 0.0):
    return (
        cq.Workplane("YZ")
        .rect(width_y, height_z)
        .extrude(length)
        .translate((x0, 0.0, z))
    )


def x_cylinder(radius: float, length: float, *, x0: float = 0.0, z: float = 0.0):
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x0, 0.0, z))


def x_tube(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    x0: float = 0.0,
    z: float = 0.0,
):
    outer = x_cylinder(outer_radius, length, x0=x0, z=z)
    inner = x_cylinder(inner_radius, length + 0.002, x0=x0 - 0.001, z=z)
    return outer.cut(inner)


def y_cylinder(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((x, y - length / 2.0, z))
    )


def z_cylinder(radius: float, height: float, *, z0: float = 0.0):
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def build_base_shape():
    foot = z_cylinder(BASE_RADIUS, BASE_THICKNESS)
    pedestal = x_box(0.050, 0.046, PEDESTAL_HEIGHT, x0=-0.050, z=BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0)
    neck = x_box(0.018, 0.038, 0.020, x0=-0.024, z=0.076)
    rear_web = x_box(
        0.018,
        SHOULDER_GAP + 2.0 * HINGE_PLATE_THICKNESS,
        0.014,
        x0=-0.022,
        z=SHOULDER_Z - 0.018,
    )
    left_plate = x_box(
        0.014,
        HINGE_PLATE_THICKNESS,
        0.032,
        x0=-0.016,
        z=SHOULDER_Z,
    ).translate((0.0, CLEVIS_OFFSET_Y, 0.0))
    right_plate = x_box(
        0.014,
        HINGE_PLATE_THICKNESS,
        0.032,
        x0=-0.016,
        z=SHOULDER_Z,
    ).translate((0.0, -CLEVIS_OFFSET_Y, 0.0))
    left_pin_head = y_cylinder(0.010, 0.004, x=-0.004, y=CLEVIS_OFFSET_Y + 0.004, z=SHOULDER_Z)
    right_pin_head = y_cylinder(0.010, 0.004, x=-0.004, y=-(CLEVIS_OFFSET_Y + 0.008), z=SHOULDER_Z)
    shoulder_seat = z_cylinder(0.006, 0.001, z0=0.086).translate((0.006, 0.0, 0.0))

    return (
        foot.union(pedestal)
        .union(neck)
        .union(rear_web)
        .union(left_plate)
        .union(right_plate)
        .union(left_pin_head)
        .union(right_pin_head)
        .union(shoulder_seat)
    )


def build_upper_link_shape():
    shoulder_tang = x_box(0.012, TANG_WIDTH, 0.028, x0=0.000, z=0.0)
    body = x_box(0.068, 0.018, 0.018, x0=0.012, z=0.0)
    top_spine = x_box(0.042, 0.014, 0.010, x0=0.028, z=0.011)
    underside = x_box(0.022, 0.014, 0.008, x0=0.042, z=-0.011)
    elbow_web = x_box(0.012, SHOULDER_GAP + 2.0 * HINGE_PLATE_THICKNESS, 0.014, x0=ELBOW_X - 0.024, z=-0.010)
    elbow_left_plate = x_box(
        CLEVIS_PLATE_LENGTH,
        HINGE_PLATE_THICKNESS,
        0.034,
        x0=ELBOW_X - CLEVIS_PLATE_LENGTH,
        z=0.0,
    ).translate((0.0, CLEVIS_OFFSET_Y, 0.0))
    elbow_right_plate = x_box(
        CLEVIS_PLATE_LENGTH,
        HINGE_PLATE_THICKNESS,
        0.034,
        x0=ELBOW_X - CLEVIS_PLATE_LENGTH,
        z=0.0,
    ).translate((0.0, -CLEVIS_OFFSET_Y, 0.0))
    elbow_cap = x_box(
        0.010,
        SHOULDER_GAP + 2.0 * HINGE_PLATE_THICKNESS,
        0.010,
        x0=ELBOW_X - 0.026,
        z=0.015,
    )

    return (
        shoulder_tang.union(body)
        .union(top_spine)
        .union(underside)
        .union(elbow_web)
        .union(elbow_left_plate)
        .union(elbow_right_plate)
        .union(elbow_cap)
    )


def build_forearm_shape():
    nose_start_x = SLIDE_GUIDE_X - NOSE_LENGTH
    elbow_tang = x_box(0.012, TANG_WIDTH, 0.028, x0=0.000, z=0.0)
    body = x_box(0.086, 0.018, 0.018, x0=0.012, z=0.0)
    top_spine = x_box(0.054, 0.014, 0.010, x0=0.032, z=0.011)
    underside = x_box(0.026, 0.014, 0.008, x0=0.056, z=-0.011)
    slide_nose = x_tube(
        NOSE_OUTER_RADIUS,
        NOSE_INNER_RADIUS,
        NOSE_LENGTH,
        x0=nose_start_x,
        z=0.0,
    )
    nose_collar = x_tube(NOSE_OUTER_RADIUS + 0.002, NOSE_INNER_RADIUS, 0.012, x0=nose_start_x - 0.012, z=0.0)
    nose_brace = x_box(0.018, 0.016, 0.010, x0=nose_start_x - 0.024, z=0.0)

    return elbow_tang.union(body).union(top_spine).union(underside).union(nose_brace).union(nose_collar).union(slide_nose)


def build_slide_tip_shape():
    rod = x_cylinder(0.0105, 0.070, x0=-0.070, z=0.0)
    flange = x_box(0.010, 0.032, 0.032, x0=0.0, z=0.0)
    face_pad = x_box(0.004, 0.026, 0.026, x0=0.010, z=0.0)
    return rod.union(flange).union(face_pad)


def tuple_close(actual, expected, tol: float = 1e-6) -> bool:
    return len(actual) == len(expected) and all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_pedestal_arm")

    dark_base = model.material("dark_base", rgba=(0.16, 0.17, 0.19, 1.0))
    painted_link = model.material("painted_link", rgba=(0.74, 0.76, 0.79, 1.0))
    steel = model.material("steel", rgba=(0.83, 0.85, 0.88, 1.0))
    black_tip = model.material("black_tip", rgba=(0.22, 0.23, 0.25, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(build_base_shape(), "base_shell"),
        material=dark_base,
        name="base_shell",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(build_upper_link_shape(), "upper_link_shell"),
        material=painted_link,
        name="upper_link_shell",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(build_forearm_shape(), "forearm_shell"),
        material=painted_link,
        name="forearm_shell",
    )

    slide_tip = model.part("slide_tip")
    slide_tip.visual(
        mesh_from_cadquery(build_slide_tip_shape(), "slide_tip_shell"),
        material=steel,
        name="slide_tip_shell",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.75, upper=1.15),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(ELBOW_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.2, lower=0.0, upper=2.15),
    )
    model.articulation(
        "tool_extension",
        ArticulationType.PRISMATIC,
        parent=forearm,
        child=slide_tip,
        origin=Origin(xyz=(SLIDE_GUIDE_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.20, lower=0.0, upper=SLIDE_STROKE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    slide_tip = object_model.get_part("slide_tip")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    tool_extension = object_model.get_articulation("tool_extension")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0025)
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
        "prompt_parts_present",
        all(part is not None for part in (base, upper_link, forearm, slide_tip)),
        "Expected base, upper_link, forearm, and slide_tip parts.",
    )

    ctx.check(
        "joint_layout_matches_prompt",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and tool_extension.articulation_type == ArticulationType.PRISMATIC
        and tuple_close(shoulder.axis, (0.0, 1.0, 0.0))
        and tuple_close(elbow.axis, (0.0, 1.0, 0.0))
        and tuple_close(tool_extension.axis, (1.0, 0.0, 0.0)),
        "Shoulder and elbow should rotate about Y; tool extension should slide along X.",
    )

    ctx.expect_contact(
        base,
        upper_link,
        contact_tol=0.0025,
        name="shoulder_hub_supported_in_pedestal",
    )
    ctx.expect_contact(upper_link, forearm, contact_tol=1e-4, name="elbow_hub_supported_in_upper_link")
    ctx.expect_contact(forearm, slide_tip, contact_tol=1e-4, name="retracted_tool_seats_on_nose")

    ctx.expect_overlap(upper_link, forearm, axes="yz", min_overlap=0.028, name="elbow_alignment_yz")
    ctx.expect_overlap(forearm, slide_tip, axes="yz", min_overlap=0.022, name="tool_alignment_yz")

    upper_aabb = ctx.part_world_aabb(upper_link)
    forearm_aabb = ctx.part_world_aabb(forearm)
    slide_aabb = ctx.part_world_aabb(slide_tip)
    if upper_aabb is None or forearm_aabb is None or slide_aabb is None:
        ctx.fail("part_aabbs_available", "Expected exact AABBs for upper_link, forearm, and slide_tip.")
    else:
        upper_length = upper_aabb[1][0] - upper_aabb[0][0]
        forearm_length = forearm_aabb[1][0] - forearm_aabb[0][0]
        slide_width = slide_aabb[1][1] - slide_aabb[0][1]
        slide_height = slide_aabb[1][2] - slide_aabb[0][2]
        ctx.check(
            "forearm_longer_than_upper_link",
            forearm_length > upper_length + 0.05,
            f"Expected a distinctly longer forearm; got upper={upper_length:.3f} m forearm={forearm_length:.3f} m.",
        )
        ctx.check(
            "tool_tip_reads_as_square_flange",
            slide_width >= 0.030 and slide_height >= 0.030,
            f"Expected square flange dimensions near 32 mm; got {slide_width:.3f} x {slide_height:.3f} m.",
        )

    with ctx.pose(tool_extension=SLIDE_STROKE):
        ctx.expect_overlap(
            forearm,
            slide_tip,
            axes="yz",
            min_overlap=0.022,
            name="extended_tool_stays_coaxial",
        )

    with ctx.pose(shoulder=0.55, elbow=1.10):
        ctx.expect_contact(
            base,
            upper_link,
            contact_tol=0.0025,
            name="shoulder_remains_supported_in_raised_pose",
        )
        ctx.expect_contact(
            upper_link,
            forearm,
            contact_tol=1e-4,
            name="elbow_remains_supported_in_bent_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
