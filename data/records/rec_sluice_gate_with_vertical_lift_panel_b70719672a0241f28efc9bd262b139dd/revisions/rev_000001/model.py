from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


OPENING_WIDTH = 1.00
PANEL_WIDTH = 1.10
PANEL_HEIGHT = 1.38
PANEL_THICKNESS = 0.045
PANEL_TRAVEL = 0.52

GUIDE_WIDTH = 0.12
GUIDE_HEIGHT = 2.05
GUIDE_BACK_THICKNESS = 0.03
GUIDE_KEEPER_THICKNESS = 0.018
GUIDE_OUTER_WALL_THICKNESS = 0.03

TOP_BEAM_WIDTH = OPENING_WIDTH + (2.0 * GUIDE_WIDTH)
TOP_BEAM_DEPTH = 0.18
TOP_BEAM_HEIGHT = 0.16
TOP_BEAM_BOTTOM = GUIDE_HEIGHT - 0.12
TOP_BEAM_CENTER_Z = TOP_BEAM_BOTTOM + (TOP_BEAM_HEIGHT / 2.0)

SILL_HEIGHT = 0.12
SILL_CENTER_Z = -(SILL_HEIGHT / 2.0)

PEDESTAL_SIZE = (0.36, 0.24, 0.16)
PEDESTAL_CENTER = (0.0, -0.02, TOP_BEAM_BOTTOM + TOP_BEAM_HEIGHT + (PEDESTAL_SIZE[2] / 2.0))

HINGE_AXIS = (0.0, -0.16, PEDESTAL_CENTER[2] + (PEDESTAL_SIZE[2] / 2.0))
CAP_OPEN_ANGLE = 1.15
HANDWHEEL_CENTER = (0.0, 0.11, 2.495)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sluice_gate")

    model.material("galvanized", rgba=(0.60, 0.62, 0.64, 1.0))
    model.material("epoxy_blue", rgba=(0.12, 0.34, 0.54, 1.0))
    model.material("wheel_red", rgba=(0.72, 0.16, 0.12, 1.0))
    model.material("cover_green", rgba=(0.26, 0.39, 0.31, 1.0))
    model.material("dark_steel", rgba=(0.28, 0.30, 0.32, 1.0))

    frame = model.part("frame")
    guide_center_y = -0.027
    guide_center_z = (-SILL_HEIGHT + GUIDE_HEIGHT) / 2.0
    left_guide_x = -((OPENING_WIDTH / 2.0) + (GUIDE_WIDTH / 2.0))
    right_guide_x = -left_guide_x

    for side_name, guide_x, wall_x in (
        ("left", left_guide_x, left_guide_x - ((GUIDE_WIDTH - GUIDE_OUTER_WALL_THICKNESS) / 2.0)),
        ("right", right_guide_x, right_guide_x + ((GUIDE_WIDTH - GUIDE_OUTER_WALL_THICKNESS) / 2.0)),
    ):
        frame.visual(
            Box((GUIDE_WIDTH, GUIDE_BACK_THICKNESS, GUIDE_HEIGHT)),
            origin=Origin(xyz=(guide_x, -0.075, guide_center_z)),
            material="galvanized",
            name=f"{side_name}_back_web",
        )
        frame.visual(
            Box((GUIDE_WIDTH, GUIDE_KEEPER_THICKNESS, GUIDE_HEIGHT)),
            origin=Origin(xyz=(guide_x, 0.021, guide_center_z)),
            material="galvanized",
            name=f"{side_name}_keeper",
        )
        frame.visual(
            Box((GUIDE_OUTER_WALL_THICKNESS, 0.12, GUIDE_HEIGHT)),
            origin=Origin(xyz=(wall_x, guide_center_y, guide_center_z)),
            material="galvanized",
            name=f"{side_name}_outer_wall",
        )

    frame.visual(
        Box((TOP_BEAM_WIDTH, TOP_BEAM_DEPTH, TOP_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.03, TOP_BEAM_CENTER_Z)),
        material="galvanized",
        name="top_beam",
    )
    frame.visual(
        Box((TOP_BEAM_WIDTH, 0.12, SILL_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.03, SILL_CENTER_Z)),
        material="galvanized",
        name="sill",
    )
    frame.visual(
        Box(PEDESTAL_SIZE),
        origin=Origin(xyz=PEDESTAL_CENTER),
        material="dark_steel",
        name="pedestal",
    )
    frame.visual(
        Box((0.10, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, -0.055, 2.279)),
        material="galvanized",
        name="screw_head",
    )
    frame.visual(
        Box((0.045, 0.045, 0.03)),
        origin=Origin(xyz=(0.0, -0.055, 2.324)),
        material="dark_steel",
        name="drive_block",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.22),
        origin=Origin(xyz=(0.0, 0.11, 2.36)),
        material="dark_steel",
        name="operator_post",
    )
    frame.visual(
        Box((0.16, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, -0.145, 2.225)),
        material="dark_steel",
        name="hinge_mount",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.16),
        origin=Origin(xyz=HINGE_AXIS, rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="hinge_knuckle",
    )

    panel = model.part("panel")
    panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.0225, PANEL_HEIGHT / 2.0)),
        material="epoxy_blue",
        name="leaf",
    )
    for side_name, seal_x in (("left", -0.53), ("right", 0.53)):
        panel.visual(
            Box((0.04, 0.012, PANEL_HEIGHT)),
            origin=Origin(xyz=(seal_x, 0.006, PANEL_HEIGHT / 2.0)),
            material="galvanized",
            name=f"{side_name}_seal",
        )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_geometry(TorusGeometry(radius=0.18, tube=0.015), "handwheel_ring"),
        material="wheel_red",
        name="ring",
    )
    for idx, yaw in enumerate((0.0, 2.0 * pi / 3.0, -2.0 * pi / 3.0)):
        handwheel.visual(
            Box((0.35, 0.018, 0.02)),
            origin=Origin(rpy=(0.0, 0.0, yaw)),
            material="wheel_red",
            name=f"spoke_{idx}",
        )
    handwheel.visual(
        Cylinder(radius=0.045, length=0.05),
        origin=Origin(),
        material="dark_steel",
        name="hub",
    )

    cap = model.part("cap")
    cap.visual(
        Box((0.44, 0.18, 0.022)),
        origin=Origin(xyz=(0.0, 0.09, 0.108)),
        material="cover_green",
        name="roof",
    )
    cap.visual(
        Box((0.018, 0.18, 0.11)),
        origin=Origin(xyz=(-0.211, 0.09, 0.055)),
        material="cover_green",
        name="left_wall",
    )
    cap.visual(
        Box((0.018, 0.18, 0.11)),
        origin=Origin(xyz=(0.211, 0.09, 0.055)),
        material="cover_green",
        name="right_wall",
    )
    cap.visual(
        Box((0.44, 0.018, 0.11)),
        origin=Origin(xyz=(0.0, 0.027, 0.055)),
        material="cover_green",
        name="rear_wall",
    )
    cap.visual(
        Box((0.44, 0.018, 0.065)),
        origin=Origin(xyz=(0.0, 0.171, 0.0325)),
        material="cover_green",
        name="front_lip",
    )
    for side_name, sleeve_x in (("left", -0.155), ("right", 0.155)):
        cap.visual(
            Cylinder(radius=0.016, length=0.14),
            origin=Origin(xyz=(sleeve_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="dark_steel",
            name=f"{side_name}_sleeve",
        )

    model.articulation(
        "frame_to_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=PANEL_TRAVEL, effort=3000.0, velocity=0.15),
    )
    model.articulation(
        "frame_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=handwheel,
        origin=Origin(xyz=HANDWHEEL_CENTER),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=4.0),
    )
    model.articulation(
        "frame_to_cap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cap,
        origin=Origin(xyz=HINGE_AXIS),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_OPEN_ANGLE, effort=40.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    handwheel = object_model.get_part("handwheel")
    cap = object_model.get_part("cap")

    slide = object_model.get_articulation("frame_to_panel")
    wheel_spin = object_model.get_articulation("frame_to_handwheel")
    cap_hinge = object_model.get_articulation("frame_to_cap")

    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        positive_elem="leaf",
        negative_elem="sill",
        max_gap=0.001,
        max_penetration=0.0,
        name="closure panel seats on the sill",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="x",
        elem_a="leaf",
        elem_b="top_beam",
        min_overlap=1.00,
        name="closure panel remains aligned with the guide span",
    )

    closed_panel_pos = ctx.part_world_position(panel)
    with ctx.pose({slide: PANEL_TRAVEL}):
        ctx.expect_gap(
            frame,
            panel,
            axis="z",
            positive_elem="top_beam",
            negative_elem="leaf",
            min_gap=0.02,
            name="raised panel stays below the operator beam",
        )
        open_panel_pos = ctx.part_world_position(panel)

    ctx.check(
        "closure panel lifts upward",
        closed_panel_pos is not None
        and open_panel_pos is not None
        and open_panel_pos[2] > closed_panel_pos[2] + 0.45,
        details=f"closed={closed_panel_pos}, open={open_panel_pos}",
    )

    wheel_rest = ctx.part_world_position(handwheel)
    with ctx.pose({wheel_spin: 1.35}):
        wheel_turned = ctx.part_world_position(handwheel)
    ctx.check(
        "handwheel spins in place above the frame",
        wheel_rest is not None
        and wheel_turned is not None
        and abs(wheel_rest[0] - wheel_turned[0]) < 1e-6
        and abs(wheel_rest[1] - wheel_turned[1]) < 1e-6
        and abs(wheel_rest[2] - wheel_turned[2]) < 1e-6
        and wheel_rest[2] > 2.40,
        details=f"rest={wheel_rest}, turned={wheel_turned}",
    )

    closed_front_lip = ctx.part_element_world_aabb(cap, elem="front_lip")
    with ctx.pose({cap_hinge: CAP_OPEN_ANGLE}):
        open_front_lip = ctx.part_element_world_aabb(cap, elem="front_lip")
    ctx.check(
        "protective cap swings upward on the rear hinge",
        closed_front_lip is not None
        and open_front_lip is not None
        and open_front_lip[1][2] > closed_front_lip[1][2] + 0.09,
        details=f"closed={closed_front_lip}, open={open_front_lip}",
    )

    return ctx.report()


object_model = build_object_model()
