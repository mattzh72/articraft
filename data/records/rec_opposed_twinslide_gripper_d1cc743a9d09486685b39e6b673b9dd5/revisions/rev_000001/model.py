from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LENGTH = 0.040
BODY_WIDTH = 0.050
BODY_HEIGHT = 0.056
BODY_CAP_LENGTH = 0.030
BODY_CAP_WIDTH = 0.036
BODY_CAP_HEIGHT = 0.020

RAIL_RADIUS = 0.0065
RAIL_LENGTH = 0.270
RAIL_Z_OFFSET = 0.018
GUIDEWAY_THICKNESS = 0.010
GUIDEWAY_Y_CENTER = -0.030

CARRIAGE_LENGTH = 0.045
CARRIAGE_WIDTH = 0.050
CARRIAGE_HEIGHT = 0.052
OPEN_CENTER_X = 0.084
JAW_TRAVEL = 0.018

FINGER_LENGTH = 0.042
FINGER_HEIGHT = 0.086
FINGER_THICKNESS = 0.008
FINGER_Y_OFFSET = 0.038
FINGER_Z_CENTER = 0.098

FORK_SLOT_LENGTH = 0.026
FORK_SLOT_HEIGHT = 0.040
FORK_SLOT_Z_CENTER = 0.094


def make_body_shell() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.006)
    )

    cap = (
        cq.Workplane("XY")
        .box(BODY_CAP_LENGTH, BODY_CAP_WIDTH, BODY_CAP_HEIGHT)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0 + BODY_CAP_HEIGHT / 2.0 - 0.004))
    )

    top_boss = (
        cq.Workplane("YZ")
        .circle(0.014)
        .extrude(0.030, both=True)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0 + 0.008))
    )

    cable_nose = (
        cq.Workplane("XZ")
        .box(0.022, 0.010, 0.012)
        .translate((0.0, -BODY_WIDTH / 2.0 + 0.006, BODY_HEIGHT / 2.0 + 0.010))
    )

    return body.union(cap).union(top_boss).union(cable_nose)


def make_carriage(sign: float) -> cq.Workplane:
    carriage = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)
        .edges("|X")
        .fillet(0.004)
    )

    support_mast = cq.Workplane("XY").box(0.016, 0.024, 0.080).translate(
        (sign * (CARRIAGE_LENGTH / 2.0 + 0.008), 0.028, 0.040)
    )
    carriage = carriage.union(support_mast)

    nose_relief = (
        cq.Workplane("XY")
        .box(0.012, CARRIAGE_WIDTH * 0.64, CARRIAGE_HEIGHT * 0.40)
        .translate((sign * (CARRIAGE_LENGTH / 2.0 - 0.006), 0.0, -0.006))
    )
    carriage = carriage.cut(nose_relief)

    return carriage


def make_fork_finger(sign: float) -> cq.Workplane:
    finger_center_x = sign * (CARRIAGE_LENGTH / 2.0 + FINGER_LENGTH / 2.0 - 0.003)
    finger = cq.Workplane("XY").box(
        FINGER_LENGTH, FINGER_THICKNESS, FINGER_HEIGHT
    ).translate((finger_center_x, FINGER_Y_OFFSET, FINGER_Z_CENTER))

    root_overlap = cq.Workplane("XY").box(0.018, 0.014, 0.030).translate(
        (sign * (CARRIAGE_LENGTH / 2.0 + 0.006), 0.034, 0.066)
    )

    slot_center_x = sign * (
        CARRIAGE_LENGTH / 2.0 + FINGER_LENGTH - FORK_SLOT_LENGTH / 2.0 - 0.003
    )
    fork_slot = cq.Workplane("XY").box(
        FORK_SLOT_LENGTH, FINGER_THICKNESS + 0.004, FORK_SLOT_HEIGHT
    ).translate((slot_center_x, FINGER_Y_OFFSET, FORK_SLOT_Z_CENTER))

    return finger.union(root_overlap).cut(fork_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wafer_handling_gripper")

    body_mat = model.material("body_gray", color=(0.30, 0.32, 0.35))
    rail_mat = model.material("rail_steel", color=(0.72, 0.75, 0.78))
    carriage_mat = model.material("carriage_gray", color=(0.54, 0.56, 0.60))
    finger_mat = model.material("finger_black", color=(0.12, 0.13, 0.15))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shell(), "body_shell"),
        material=body_mat,
        name="body_shell",
    )
    body.visual(
        Box((RAIL_LENGTH, GUIDEWAY_THICKNESS, GUIDEWAY_THICKNESS)),
        origin=Origin(xyz=(0.0, GUIDEWAY_Y_CENTER, RAIL_Z_OFFSET)),
        material=rail_mat,
        name="upper_guideway",
    )
    body.visual(
        Box((RAIL_LENGTH, GUIDEWAY_THICKNESS, GUIDEWAY_THICKNESS)),
        origin=Origin(xyz=(0.0, GUIDEWAY_Y_CENTER, -RAIL_Z_OFFSET)),
        material=rail_mat,
        name="lower_guideway",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.080)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        mesh_from_cadquery(make_carriage(+1.0), "left_carriage"),
        material=carriage_mat,
        name="carriage",
    )
    left_jaw.visual(
        mesh_from_cadquery(make_fork_finger(+1.0), "left_fork_tip"),
        material=finger_mat,
        name="fork_tip",
    )
    left_jaw.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.160)),
        mass=0.55,
        origin=Origin(xyz=(0.020, 0.020, 0.050)),
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        mesh_from_cadquery(make_carriage(-1.0), "right_carriage"),
        material=carriage_mat,
        name="carriage",
    )
    right_jaw.visual(
        mesh_from_cadquery(make_fork_finger(-1.0), "right_fork_tip"),
        material=finger_mat,
        name="fork_tip",
    )
    right_jaw.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.160)),
        mass=0.55,
        origin=Origin(xyz=(-0.020, 0.020, 0.050)),
    )

    model.articulation(
        "body_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_jaw,
        origin=Origin(xyz=(-OPEN_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_jaw,
        origin=Origin(xyz=(OPEN_CENTER_X, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("body_to_left_jaw")
    right_slide = object_model.get_articulation("body_to_right_jaw")

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
        "jaw slides are prismatic",
        left_slide.articulation_type == ArticulationType.PRISMATIC
        and right_slide.articulation_type == ArticulationType.PRISMATIC,
        "Both jaws should be authored as independent prismatic slides.",
    )
    ctx.check(
        "jaw slides share the closing axis",
        left_slide.axis == (1.0, 0.0, 0.0) and right_slide.axis == (-1.0, 0.0, 0.0),
        "Left and right slides should be collinear along the x-axis.",
    )

    ctx.expect_contact(
        left_jaw,
        body,
        contact_tol=0.001,
        name="left jaw remains mounted on the guide rails",
    )
    ctx.expect_contact(
        right_jaw,
        body,
        contact_tol=0.001,
        name="right jaw remains mounted on the guide rails",
    )
    ctx.expect_overlap(
        left_jaw,
        right_jaw,
        axes="yz",
        elem_a="fork_tip",
        elem_b="fork_tip",
        min_overlap=0.015,
        name="fork tips share a common pickup zone",
    )
    ctx.expect_gap(
        right_jaw,
        left_jaw,
        axis="x",
        positive_elem="fork_tip",
        negative_elem="fork_tip",
        min_gap=0.040,
        max_gap=0.055,
        name="open pose leaves a wafer entry gap",
    )

    with ctx.pose({left_slide: JAW_TRAVEL, right_slide: JAW_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="near-closed pose remains collision free"
        )
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            positive_elem="fork_tip",
            negative_elem="fork_tip",
            min_gap=0.008,
            max_gap=0.016,
            name="near-closed pose keeps a narrow centering gap",
        )
        ctx.expect_overlap(
            left_jaw,
            right_jaw,
            axes="yz",
            elem_a="fork_tip",
            elem_b="fork_tip",
            min_overlap=0.015,
            name="fork tips stay opposed when nearly closed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
