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


FOOT_LENGTH = 0.164
FOOT_WIDTH = 0.134
FOOT_HEIGHT = 0.008
BODY_LENGTH = 0.156
BODY_WIDTH = 0.126
BODY_HEIGHT = 0.030
BODY_CORNER_RADIUS = 0.020
SHOULDER_DIAMETER = 0.118
SHOULDER_HEIGHT = 0.012
JOINT_HEIGHT = FOOT_HEIGHT + BODY_HEIGHT + SHOULDER_HEIGHT
CAP_OUTER_DIAMETER = 0.104
CAP_INNER_DIAMETER = 0.086
CAP_HEIGHT = 0.006

LOWER_FLANGE_DIAMETER = 0.082
LOWER_FLANGE_HEIGHT = 0.008
COLLAR_DIAMETER = 0.074
COLLAR_HEIGHT = 0.016
COLLAR_CAP_DIAMETER = 0.062
COLLAR_CAP_HEIGHT = 0.006
PEDESTAL_LENGTH = 0.072
PEDESTAL_WIDTH = 0.046
PEDESTAL_HEIGHT = 0.010
TOP_PLATE_LENGTH = 0.110
TOP_PLATE_WIDTH = 0.072
TOP_PLATE_HEIGHT = 0.008
TOP_PLATE_CORNER_RADIUS = 0.008
TOP_PLATE_BOTTOM = LOWER_FLANGE_HEIGHT + COLLAR_HEIGHT + COLLAR_CAP_HEIGHT + PEDESTAL_HEIGHT
MOUNT_HOLE_DIAMETER = 0.006


def rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).edges("|Z").fillet(radius)


def annulus(outer_diameter: float, inner_diameter: float, height: float) -> cq.Workplane:
    profile = cq.Workplane("XY").circle(outer_diameter / 2.0)
    if inner_diameter > 0.0:
        profile = profile.circle(inner_diameter / 2.0)
    return profile.extrude(height)


def make_base_body() -> cq.Workplane:
    foot = rounded_box(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT, 0.024).translate(
        (0.0, 0.0, FOOT_HEIGHT / 2.0)
    )
    body = (
        rounded_box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT, BODY_CORNER_RADIUS)
        .translate((0.0, 0.0, FOOT_HEIGHT + BODY_HEIGHT / 2.0))
        .edges(">Z")
        .chamfer(0.0025)
    )
    shoulder = annulus(SHOULDER_DIAMETER, 0.052, SHOULDER_HEIGHT).translate((0.0, 0.0, FOOT_HEIGHT + BODY_HEIGHT))
    rib_z = FOOT_HEIGHT + BODY_HEIGHT - 0.001
    rib_pos_x = cq.Workplane("XY").box(0.020, 0.026, 0.014).translate((0.046, 0.0, rib_z))
    rib_neg_x = cq.Workplane("XY").box(0.020, 0.026, 0.014).translate((-0.046, 0.0, rib_z))
    rib_pos_y = cq.Workplane("XY").box(0.026, 0.020, 0.014).translate((0.0, 0.041, rib_z))
    rib_neg_y = cq.Workplane("XY").box(0.026, 0.020, 0.014).translate((0.0, -0.041, rib_z))

    return (
        foot.union(body)
        .union(shoulder)
        .union(rib_pos_x)
        .union(rib_neg_x)
        .union(rib_pos_y)
        .union(rib_neg_y)
    )


def make_base_cap() -> cq.Workplane:
    return annulus(CAP_OUTER_DIAMETER, CAP_INNER_DIAMETER, CAP_HEIGHT).translate(
        (0.0, 0.0, JOINT_HEIGHT)
    )


def make_rotary_collar() -> cq.Workplane:
    lower_flange = cq.Workplane("XY").circle(LOWER_FLANGE_DIAMETER / 2.0).extrude(LOWER_FLANGE_HEIGHT)
    upper_collar = (
        cq.Workplane("XY")
        .circle(COLLAR_DIAMETER / 2.0)
        .extrude(COLLAR_HEIGHT)
        .translate((0.0, 0.0, LOWER_FLANGE_HEIGHT))
    )
    collar_cap = (
        cq.Workplane("XY")
        .circle(COLLAR_CAP_DIAMETER / 2.0)
        .extrude(COLLAR_CAP_HEIGHT)
        .translate((0.0, 0.0, LOWER_FLANGE_HEIGHT + COLLAR_HEIGHT))
    )
    pedestal = rounded_box(PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT, 0.006).translate(
        (
            0.0,
            0.0,
            LOWER_FLANGE_HEIGHT
            + COLLAR_HEIGHT
            + COLLAR_CAP_HEIGHT
            + PEDESTAL_HEIGHT / 2.0,
        )
    )
    return lower_flange.union(upper_collar).union(collar_cap).union(pedestal)


def make_support_ribs() -> cq.Workplane:
    rib_bottom = LOWER_FLANGE_HEIGHT + COLLAR_HEIGHT - 0.002
    rib_height = 0.022
    rib_z = rib_bottom + rib_height / 2.0
    x_pos = COLLAR_DIAMETER / 2.0 + 0.010
    y_pos = COLLAR_DIAMETER / 2.0 + 0.008

    rib_pos_x = cq.Workplane("XY").box(0.026, 0.010, rib_height).translate((x_pos, 0.0, rib_z))
    rib_neg_x = cq.Workplane("XY").box(0.026, 0.010, rib_height).translate((-x_pos, 0.0, rib_z))
    rib_pos_y = cq.Workplane("XY").box(0.010, 0.022, rib_height).translate((0.0, y_pos, rib_z))
    rib_neg_y = cq.Workplane("XY").box(0.010, 0.022, rib_height).translate((0.0, -y_pos, rib_z))

    return rib_pos_x.union(rib_neg_x).union(rib_pos_y).union(rib_neg_y)


def make_top_plate() -> cq.Workplane:
    hole_x = 0.037
    hole_y = 0.020
    return (
        cq.Workplane("XY")
        .box(TOP_PLATE_LENGTH, TOP_PLATE_WIDTH, TOP_PLATE_HEIGHT)
        .edges("|Z")
        .fillet(TOP_PLATE_CORNER_RADIUS)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-hole_x, -hole_y),
                (-hole_x, hole_y),
                (hole_x, -hole_y),
                (hole_x, hole_y),
            ]
        )
        .hole(MOUNT_HOLE_DIAMETER)
        .translate((0.0, 0.0, TOP_PLATE_BOTTOM + TOP_PLATE_HEIGHT / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_sensor_base")

    base_finish = model.material("base_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    collar_finish = model.material("collar_finish", rgba=(0.34, 0.36, 0.40, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.44, 0.46, 0.49, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_body(), "sensor_base_body_v3"),
        material=base_finish,
        name="body",
    )
    base.visual(
        mesh_from_cadquery(make_base_cap(), "sensor_base_cap_v3"),
        material=base_finish,
        name="retainer_cap",
    )

    rotating_top = model.part("rotating_top")
    rotating_top.visual(
        mesh_from_cadquery(make_rotary_collar(), "sensor_rotary_collar_v3"),
        material=collar_finish,
        name="collar",
    )
    rotating_top.visual(
        mesh_from_cadquery(make_support_ribs(), "sensor_support_ribs_v3"),
        material=collar_finish,
        name="ribs",
    )
    rotating_top.visual(
        mesh_from_cadquery(make_top_plate(), "sensor_top_plate_v3"),
        material=plate_finish,
        name="top_plate",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotating_top,
        origin=Origin(xyz=(0.0, 0.0, JOINT_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-3.05,
            upper=3.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rotating_top = object_model.get_part("rotating_top")
    yaw = object_model.get_articulation("yaw")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    limits = yaw.motion_limits
    ctx.check(
        "yaw_axis_vertical",
        tuple(yaw.axis) == (0.0, 0.0, 1.0),
        f"unexpected yaw axis: {yaw.axis}",
    )
    ctx.check(
        "yaw_limits_span_both_directions",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        f"unexpected yaw limits: {limits}",
    )
    ctx.expect_origin_distance(
        rotating_top,
        base,
        axes="xy",
        max_dist=1e-6,
        name="rotating_top_stays_on_centerline",
    )
    ctx.expect_contact(
        rotating_top,
        base,
        contact_tol=1e-6,
        elem_a="collar",
        elem_b="body",
        name="rotary_collar_seats_on_base_shoulder",
    )
    ctx.expect_gap(
        rotating_top,
        base,
        axis="z",
        positive_elem="top_plate",
        min_gap=0.028,
        name="top_plate_clears_lower_body",
    )
    ctx.expect_overlap(
        rotating_top,
        base,
        axes="xy",
        min_overlap=0.060,
        elem_a="collar",
        name="collar_has_real_support_footprint",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
