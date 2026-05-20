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


BACKPLATE_W = 0.22
BACKPLATE_H = 0.68
BACKPLATE_T = 0.008

SPINE_W = 0.062
SPINE_D = 0.046
SPINE_H = 0.56
SPINE_Z0 = 0.06
SPINE_CENTER_Y = BACKPLATE_T / 2.0 + SPINE_D / 2.0

HINGE_RADIUS = 0.058

PIN_RADIUS = 0.0065
POD_TOTAL_WIDTH = 0.026
EYE_INNER_RADIUS = PIN_RADIUS
EYE_OUTER_RADIUS = 0.013
EYE_WIDTH = 0.012

BRANCH_ARM_LENGTH = 0.164
END_PLATE_THICKNESS = 0.01

BRANCH_SPECS = (
    {
        "part": "lower_arm",
        "joint": "lower_branch_joint",
        "pod_visual": "lower_pod",
        "angle_deg": 155.0,
        "hinge_z": 0.16,
    },
    {
        "part": "middle_arm",
        "joint": "middle_branch_joint",
        "pod_visual": "middle_pod",
        "angle_deg": 90.0,
        "hinge_z": 0.34,
    },
    {
        "part": "upper_arm",
        "joint": "upper_branch_joint",
        "pod_visual": "upper_pod",
        "angle_deg": 25.0,
        "hinge_z": 0.52,
    },
)


def _polar_xy(radius: float, angle_deg: float) -> tuple[float, float]:
    angle_rad = math.radians(angle_deg)
    return (radius * math.cos(angle_rad), radius * math.sin(angle_rad))


def _place_branch_shape(shape: cq.Workplane, angle_deg: float, xyz: tuple[float, float, float]) -> cq.Workplane:
    return shape.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg).translate(xyz)


def _make_backplate_assembly() -> cq.Workplane:
    plate = cq.Workplane("XY").box(BACKPLATE_W, BACKPLATE_T, BACKPLATE_H, centered=(True, True, False))

    slot_cutters = (
        cq.Workplane("XZ")
        .pushPoints(
            [
                (-0.075, 0.14),
                (0.075, 0.14),
                (-0.075, 0.54),
                (0.075, 0.54),
            ]
        )
        .slot2D(0.028, 0.010, angle=90.0)
        .extrude(BACKPLATE_T * 2.0, both=True)
    )
    plate = plate.cut(slot_cutters)

    spine = (
        cq.Workplane("XY")
        .box(SPINE_W, SPINE_D, SPINE_H, centered=(True, True, False))
        .translate((0.0, SPINE_CENTER_Y, SPINE_Z0))
    )

    saddle = (
        cq.Workplane("XY")
        .box(0.102, 0.022, 0.24, centered=(True, True, False))
        .translate((0.0, BACKPLATE_T / 2.0 + 0.011, 0.22))
    )

    return plate.union(spine).union(saddle)


def _make_branch_pod() -> cq.Workplane:
    bridge = (
        cq.Workplane("XY")
        .box(0.040, 0.018, 0.022, centered=(False, True, True))
        .translate((-0.112, 0.0, 0.0))
    )
    housing = (
        cq.Workplane("XY")
        .box(0.046, 0.032, 0.046, centered=(False, True, True))
        .translate((-0.072, 0.0, 0.0))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(0.028, 0.005, 0.048, centered=(False, True, True))
        .translate((-0.028, 0.0105, 0.0))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.028, 0.005, 0.048, centered=(False, True, True))
        .translate((-0.028, -0.0105, 0.0))
    )
    pin = cq.Workplane("XZ").circle(PIN_RADIUS).extrude(POD_TOTAL_WIDTH / 2.0, both=True)
    return bridge.union(housing).union(left_cheek).union(right_cheek).union(pin)


def _make_arm_hub() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(EYE_OUTER_RADIUS)
        .circle(EYE_INNER_RADIUS * 0.92)
        .extrude(EYE_WIDTH / 2.0, both=True)
    )


def _make_branch_arm_body() -> cq.Workplane:
    neck_start = 0.030
    beam_length = BRANCH_ARM_LENGTH - END_PLATE_THICKNESS - neck_start

    neck = (
        cq.Workplane("XY")
        .box(0.020, 0.010, 0.018, centered=(False, True, True))
        .translate((0.010, 0.0, 0.0))
    )
    root_block = (
        cq.Workplane("XY")
        .box(0.020, 0.016, 0.018, centered=(False, True, True))
        .translate((neck_start, 0.0, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(beam_length, 0.016, 0.014, centered=(False, True, True))
        .translate((neck_start, 0.0, 0.0))
    )
    lower_flanges = (
        cq.Workplane("XZ")
        .rect(0.090, 0.010)
        .extrude(0.006, both=True)
        .translate((0.070, 0.0, -0.012))
    )

    return neck.union(root_block).union(beam).union(lower_flanges)


def _make_end_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(END_PLATE_THICKNESS, 0.072, 0.095, centered=(False, True, True))
        .translate((BRANCH_ARM_LENGTH - END_PLATE_THICKNESS, 0.0, 0.0))
    )
    hole_cutters = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, 0.026), (0.0, -0.026)])
        .circle(0.0035)
        .extrude(END_PLATE_THICKNESS * 2.0)
        .translate((BRANCH_ARM_LENGTH - END_PLATE_THICKNESS - 0.002, 0.0, 0.0))
    )
    return plate.cut(hole_cutters)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_panel_three_branch_bracket")

    frame_finish = model.material("frame_finish", rgba=(0.19, 0.20, 0.22, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.60, 0.62, 0.65, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.82, 0.84, 0.86, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_backplate_assembly(), "frame_backplate_assembly"),
        material=frame_finish,
        name="backplate_assembly",
    )

    pod_shape = _make_branch_pod()
    arm_hub_shape = _make_arm_hub()
    arm_body_shape = _make_branch_arm_body()
    end_plate_shape = _make_end_plate()

    arm_parts = {}
    for spec in BRANCH_SPECS:
        px, py = _polar_xy(HINGE_RADIUS, spec["angle_deg"])
        hinge_xyz = (px, SPINE_CENTER_Y + py, spec["hinge_z"])

        frame.visual(
            mesh_from_cadquery(
                _place_branch_shape(pod_shape, spec["angle_deg"], hinge_xyz),
                f"{spec['pod_visual']}_mesh",
            ),
            material=frame_finish,
            name=spec["pod_visual"],
        )

        arm = model.part(spec["part"])
        arm.visual(
            mesh_from_cadquery(arm_hub_shape, f"{spec['part']}_hub_mesh"),
            material=arm_finish,
            name="arm_hub",
        )
        arm.visual(
            mesh_from_cadquery(arm_body_shape, f"{spec['part']}_body_mesh"),
            material=arm_finish,
            name="arm_body",
        )
        arm.visual(
            mesh_from_cadquery(end_plate_shape, f"{spec['part']}_end_plate_mesh"),
            material=plate_finish,
            name="end_plate",
        )
        arm_parts[spec["part"]] = arm

        model.articulation(
            spec["joint"],
            ArticulationType.REVOLUTE,
            parent=frame,
            child=arm,
            origin=Origin(
                xyz=hinge_xyz,
                rpy=(0.0, 0.0, math.radians(spec["angle_deg"])),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.8,
                lower=-0.45,
                upper=1.10,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    lower_arm = object_model.get_part("lower_arm")
    middle_arm = object_model.get_part("middle_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_joint = object_model.get_articulation("lower_branch_joint")
    middle_joint = object_model.get_articulation("middle_branch_joint")
    upper_joint = object_model.get_articulation("upper_branch_joint")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(frame, lower_arm, elem_a="lower_pod", elem_b="arm_hub", reason="lower hinge pin is captured inside the lower arm hub")
    ctx.allow_overlap(frame, middle_arm, elem_a="middle_pod", elem_b="arm_hub", reason="middle hinge pin is captured inside the middle arm hub")
    ctx.allow_overlap(frame, upper_arm, elem_a="upper_pod", elem_b="arm_hub", reason="upper hinge pin is captured inside the upper arm hub")

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

    joints_are_independent = (
        lower_joint.articulation_type == ArticulationType.REVOLUTE
        and middle_joint.articulation_type == ArticulationType.REVOLUTE
        and upper_joint.articulation_type == ArticulationType.REVOLUTE
        and lower_joint.parent == "frame"
        and middle_joint.parent == "frame"
        and upper_joint.parent == "frame"
        and lower_joint.child == "lower_arm"
        and middle_joint.child == "middle_arm"
        and upper_joint.child == "upper_arm"
    )
    ctx.check(
        "three independent branch revolutes",
        joints_are_independent,
        "Expected three distinct frame-mounted revolute joints with one child arm each.",
    )

    ctx.expect_contact(frame, lower_arm, elem_a="lower_pod", elem_b="arm_hub", name="lower arm seated in lower pod")
    ctx.expect_contact(frame, middle_arm, elem_a="middle_pod", elem_b="arm_hub", name="middle arm seated in middle pod")
    ctx.expect_contact(frame, upper_arm, elem_a="upper_pod", elem_b="arm_hub", name="upper arm seated in upper pod")

    with ctx.pose({lower_joint: 0.85, middle_joint: 0.75, upper_joint: 0.90}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no branch collisions in lifted pose")

    def end_plate_center(part_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem="end_plate")
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    def check_branch_lifts(part_name: str, joint_name: str) -> None:
        with ctx.pose(**{joint_name: 0.0}):
            closed_center = end_plate_center(part_name)
        with ctx.pose(**{joint_name: 0.85}):
            open_center = end_plate_center(part_name)

        ok = (
            closed_center is not None
            and open_center is not None
            and open_center[2] > closed_center[2] + 0.045
        )
        details = (
            f"Closed center={closed_center}, open center={open_center}; "
            "expected positive joint motion to raise the end plate."
        )
        ctx.check(f"{part_name} opens upward", ok, details)

    check_branch_lifts("lower_arm", "lower_branch_joint")
    check_branch_lifts("middle_arm", "middle_branch_joint")
    check_branch_lifts("upper_arm", "upper_branch_joint")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
