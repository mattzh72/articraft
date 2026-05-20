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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_PLATE_X = 0.42
BASE_PLATE_Y = 0.36
BASE_PLATE_T = 0.03
PEDESTAL_H = 0.12
COLUMN_R = 0.06
COLUMN_H = 0.59
BEARING_R = 0.095
BEARING_H = 0.06
ARM_HUB_R = 0.10
ARM_HUB_T = 0.024
ARM_TURRET_R = 0.055
ARM_TURRET_H = 0.05
ARM_BOOM_R = 0.042
ARM_BOOM_START = 0.08
ARM_BOOM_LENGTH = 0.72
ARM_BOOM_Z = 0.045
CARRIAGE_LENGTH = 0.10
CARRIAGE_OUTER_R = 0.072
CARRIAGE_TRAVEL = 0.48
ARM_JOINT_Z = BASE_PLATE_T + PEDESTAL_H + COLUMN_H + BEARING_H


def _make_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .rect(BASE_PLATE_X, BASE_PLATE_Y)
        .extrude(BASE_PLATE_T)
        .edges("|Z")
        .fillet(0.03)
    )

    anchor_cutters = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.145, -0.11),
                (-0.145, 0.11),
                (0.145, -0.11),
                (0.145, 0.11),
            ]
        )
        .circle(0.011)
        .extrude(BASE_PLATE_T + 0.002)
    )
    plate = plate.cut(anchor_cutters)

    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=BASE_PLATE_T)
        .circle(0.105)
        .workplane(offset=PEDESTAL_H)
        .circle(0.072)
        .loft(combine=False)
    )

    column = (
        cq.Workplane("XY")
        .workplane(offset=BASE_PLATE_T + PEDESTAL_H)
        .circle(COLUMN_R)
        .extrude(COLUMN_H, combine=False)
    )

    bearing = (
        cq.Workplane("XY")
        .workplane(offset=BASE_PLATE_T + PEDESTAL_H + COLUMN_H)
        .circle(BEARING_R)
        .extrude(BEARING_H, combine=False)
    )

    return plate.union(pedestal).union(column).union(bearing)


def _make_arm_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(ARM_HUB_R).extrude(ARM_HUB_T)
    turret = (
        cq.Workplane("XY")
        .workplane(offset=ARM_HUB_T)
        .circle(ARM_TURRET_R)
        .extrude(ARM_TURRET_H, combine=False)
    )

    boom = (
        cq.Workplane("XY")
        .circle(ARM_BOOM_R)
        .extrude(ARM_BOOM_LENGTH)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((ARM_BOOM_START, 0.0, ARM_BOOM_Z))
    )

    tip = (
        cq.Workplane("XY")
        .circle(0.048)
        .extrude(0.035)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((ARM_BOOM_START + ARM_BOOM_LENGTH - 0.035, 0.0, ARM_BOOM_Z))
    )

    drive_pod = (
        cq.Workplane("XY")
        .box(0.11, 0.085, 0.045)
        .translate((0.01, 0.0, 0.028))
    )

    return hub.union(turret).union(boom).union(tip).union(drive_pod)


def _make_carriage_shape() -> cq.Workplane:
    collar_outer = cq.Workplane("XY").circle(CARRIAGE_OUTER_R).extrude(CARRIAGE_LENGTH)
    collar_inner = cq.Workplane("XY").circle(ARM_BOOM_R).extrude(CARRIAGE_LENGTH + 0.004)
    collar = (
        collar_outer.cut(collar_inner.translate((0.0, 0.0, -0.002)))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((-0.5 * CARRIAGE_LENGTH, 0.0, ARM_BOOM_Z))
    )

    body = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, 0.12, 0.07)
        .translate((0.0, 0.0, -0.02))
    )

    neck = cq.Workplane("XY").box(0.04, 0.05, 0.10).translate((0.0, 0.0, -0.092))

    head_plate = (
        cq.Workplane("XY")
        .box(0.12, 0.09, 0.012)
        .translate((0.0, 0.0, -0.148))
        .edges("|Z")
        .fillet(0.012)
    )
    head_holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.03, 0.0), (0.03, 0.0)])
        .circle(0.006)
        .extrude(0.02)
        .translate((0.0, 0.0, -0.158))
    )
    head_plate = head_plate.cut(head_holes)

    arm_clearance = (
        cq.Workplane("XY")
        .circle(ARM_BOOM_R)
        .extrude(CARRIAGE_LENGTH + 0.04)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((-0.5 * (CARRIAGE_LENGTH + 0.04), 0.0, ARM_BOOM_Z))
    )

    return collar.union(body).union(neck).union(head_plate).cut(arm_clearance)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_arm_slider")

    base_paint = model.material("base_paint", color=(0.18, 0.19, 0.21))
    arm_finish = model.material("arm_finish", color=(0.75, 0.77, 0.80))
    carriage_finish = model.material("carriage_finish", color=(0.40, 0.42, 0.45))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_column"),
        material=base_paint,
        name="base_shell",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=ARM_HUB_R, length=ARM_HUB_T),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * ARM_HUB_T)),
        material=arm_finish,
        name="hub_disc",
    )
    arm.visual(
        Cylinder(radius=ARM_TURRET_R, length=ARM_TURRET_H),
        origin=Origin(xyz=(0.0, 0.0, ARM_HUB_T + 0.5 * ARM_TURRET_H)),
        material=arm_finish,
        name="turret_post",
    )
    arm.visual(
        Box((ARM_BOOM_LENGTH, 0.08, 0.05)),
        origin=Origin(xyz=(ARM_BOOM_START + 0.5 * ARM_BOOM_LENGTH, 0.0, ARM_BOOM_Z)),
        material=arm_finish,
        name="boom_rail",
    )
    arm.visual(
        Box((0.11, 0.085, 0.045)),
        origin=Origin(xyz=(0.03, 0.0, 0.028)),
        material=arm_finish,
        name="drive_pod",
    )
    arm.visual(
        Cylinder(radius=0.044, length=0.03),
        origin=Origin(
            xyz=(ARM_BOOM_START + ARM_BOOM_LENGTH, 0.0, ARM_BOOM_Z),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material=arm_finish,
        name="nose_cap",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, 0.16, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=carriage_finish,
        name="top_bridge",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, 0.028, 0.164)),
        origin=Origin(xyz=(0.0, 0.056, -0.057)),
        material=carriage_finish,
        name="left_cheek",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, 0.028, 0.164)),
        origin=Origin(xyz=(0.0, -0.056, -0.057)),
        material=carriage_finish,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.06, 0.14, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=carriage_finish,
        name="lower_bridge",
    )
    carriage.visual(
        Box((0.12, 0.09, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=carriage_finish,
        name="head_plate",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, ARM_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=-2.2,
            upper=2.2,
        ),
    )

    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.18, 0.0, ARM_BOOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    arm_joint = object_model.get_articulation("base_to_arm")
    carriage_joint = object_model.get_articulation("arm_to_carriage")

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

    ctx.expect_contact(
        base,
        arm,
        contact_tol=0.001,
        name="arm_bearing_contacts_base_column",
    )
    ctx.expect_contact(
        arm,
        carriage,
        elem_a="boom_rail",
        elem_b="top_bridge",
        contact_tol=0.001,
        name="carriage_supported_on_arm",
    )
    ctx.expect_contact(
        carriage,
        carriage,
        elem_a="top_bridge",
        elem_b="left_cheek",
        contact_tol=1e-6,
        name="left_cheek_connected_to_bridge",
    )
    ctx.expect_contact(
        carriage,
        carriage,
        elem_a="top_bridge",
        elem_b="right_cheek",
        contact_tol=1e-6,
        name="right_cheek_connected_to_bridge",
    )
    ctx.expect_contact(
        carriage,
        carriage,
        elem_a="left_cheek",
        elem_b="lower_bridge",
        contact_tol=1e-6,
        name="left_cheek_connected_to_lower_bridge",
    )
    ctx.expect_contact(
        carriage,
        carriage,
        elem_a="right_cheek",
        elem_b="lower_bridge",
        contact_tol=1e-6,
        name="right_cheek_connected_to_lower_bridge",
    )
    ctx.expect_contact(
        carriage,
        carriage,
        elem_a="lower_bridge",
        elem_b="head_plate",
        contact_tol=1e-6,
        name="head_plate_supported_by_lower_bridge",
    )

    ctx.check(
        "arm_joint_is_vertical_revolute",
        arm_joint.joint_type == ArticulationType.REVOLUTE
        and tuple(arm_joint.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical revolute axis, got type={arm_joint.joint_type}, axis={arm_joint.axis}",
    )
    ctx.check(
        "carriage_joint_is_axial_prismatic",
        carriage_joint.joint_type == ArticulationType.PRISMATIC
        and tuple(carriage_joint.axis) == (1.0, 0.0, 0.0),
        details=(
            f"Expected axial prismatic joint, got type={carriage_joint.joint_type}, "
            f"axis={carriage_joint.axis}"
        ),
    )

    with ctx.pose({arm_joint: 0.0, carriage_joint: 0.0}):
        inboard = ctx.part_world_position(carriage)
    with ctx.pose({arm_joint: 0.0, carriage_joint: CARRIAGE_TRAVEL}):
        outboard = ctx.part_world_position(carriage)

    if inboard is None or outboard is None:
        ctx.fail(
            "carriage_extension_probe_available",
            "Could not resolve carriage world position in one or more slider poses.",
        )
    else:
        ctx.check(
            "carriage_slides_out_along_arm",
            outboard[0] > inboard[0] + 0.35
            and abs(outboard[1] - inboard[1]) < 1e-6
            and abs(outboard[2] - inboard[2]) < 1e-6,
            details=f"inboard={inboard}, outboard={outboard}",
        )

    with ctx.pose({arm_joint: 0.0, carriage_joint: 0.24}):
        straight = ctx.part_world_position(carriage)
    with ctx.pose({arm_joint: 1.0, carriage_joint: 0.24}):
        swept = ctx.part_world_position(carriage)

    if straight is None or swept is None:
        ctx.fail(
            "arm_sweep_probe_available",
            "Could not resolve carriage world position in one or more arm poses.",
        )
    else:
        ctx.check(
            "arm_sweeps_carriage_in_plan",
            swept[1] > straight[1] + 0.25
            and swept[0] < straight[0] - 0.10
            and abs(swept[2] - straight[2]) < 1e-6,
            details=f"straight={straight}, swept={swept}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
