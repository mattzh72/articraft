from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)


BASE_L = 0.34
BASE_W = 0.11
BASE_T = 0.012
GUIDE_RAIL_L = 0.25
GUIDE_RAIL_W = 0.036
GUIDE_RAIL_H = 0.012
END_STOP_L = 0.016
END_STOP_H = 0.010

CARRIAGE_L = 0.08
CARRIAGE_W = 0.09
CARRIAGE_TOP_T = 0.010
CARRIAGE_FRAME_Z = BASE_T + GUIDE_RAIL_H + (CARRIAGE_TOP_T / 2.0)
SKIRT_W = 0.015
SKIRT_H = 0.020
GUIDE_PAD_L = 0.06
GUIDE_PAD_T = 0.0065
GUIDE_PAD_H = 0.012
PIVOT_X = 0.028
EAR_T = 0.010
EAR_CENTER_Y = 0.032
EAR_H = 0.024
EAR_CENTER_Z_LOCAL = 0.017
BARREL_L = 0.054
BARREL_R = 0.008

LINK_RAIL_L = 0.158
LINK_RAIL_T = 0.010
LINK_RAIL_H = 0.016
LINK_INNER_W = 0.032
LINK_OUTER_W = LINK_INNER_W + (2.0 * LINK_RAIL_T)
LINK_RAIL_CENTER_Y = (LINK_INNER_W / 2.0) + (LINK_RAIL_T / 2.0)
LINK_RAIL_CENTER_X = LINK_RAIL_L / 2.0
GUIDE_FLOOR_L = 0.142
GUIDE_FLOOR_CENTER_X = 0.087
GUIDE_FLOOR_T = 0.004
FRONT_BRIDGE_L = 0.014
FRONT_BRIDGE_CENTER_X = 0.159

SLIDER_HOME_X = 0.046
SLIDER_TRAVEL = 0.082
SLIDER_L = 0.040
SLIDER_W = 0.031
SLIDER_H = 0.008
SLIDER_CAP_L = 0.024
SLIDER_CAP_W = 0.020
SLIDER_CAP_H = 0.004

BASE_SLIDE_TRAVEL = 0.14
HINGE_OPEN_LIMIT = 1.05


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_link_slide_mechanism")

    model.material("base_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("rail_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("carriage_orange", rgba=(0.88, 0.46, 0.14, 1.0))
    model.material("link_blue", rgba=(0.22, 0.39, 0.66, 1.0))
    model.material("slider_silver", rgba=(0.80, 0.82, 0.84, 1.0))
    model.material("fastener_dark", rgba=(0.14, 0.15, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_L, BASE_W, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="base_charcoal",
        name="base_plate",
    )
    base.visual(
        Box((GUIDE_RAIL_L, GUIDE_RAIL_W, GUIDE_RAIL_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T + (GUIDE_RAIL_H / 2.0))),
        material="rail_steel",
        name="guide_rail",
    )
    base.visual(
        Box((END_STOP_L, GUIDE_RAIL_W + 0.016, END_STOP_H)),
        origin=Origin(
            xyz=(
                -(GUIDE_RAIL_L / 2.0) + (END_STOP_L / 2.0),
                0.0,
                BASE_T + GUIDE_RAIL_H + (END_STOP_H / 2.0),
            )
        ),
        material="base_charcoal",
        name="left_end_stop",
    )
    base.visual(
        Box((END_STOP_L, GUIDE_RAIL_W + 0.016, END_STOP_H)),
        origin=Origin(
            xyz=(
                (GUIDE_RAIL_L / 2.0) - (END_STOP_L / 2.0),
                0.0,
                BASE_T + GUIDE_RAIL_H + (END_STOP_H / 2.0),
            )
        ),
        material="base_charcoal",
        name="right_end_stop",
    )
    base.visual(
        Cylinder(radius=0.007, length=BASE_T),
        origin=Origin(
            xyz=(-0.125, -0.036, BASE_T / 2.0),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material="fastener_dark",
        name="left_mount_boss",
    )
    base.visual(
        Cylinder(radius=0.007, length=BASE_T),
        origin=Origin(
            xyz=(-0.125, 0.036, BASE_T / 2.0),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material="fastener_dark",
        name="left_rear_mount_boss",
    )
    base.visual(
        Cylinder(radius=0.007, length=BASE_T),
        origin=Origin(
            xyz=(0.125, -0.036, BASE_T / 2.0),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material="fastener_dark",
        name="right_mount_boss",
    )
    base.visual(
        Cylinder(radius=0.007, length=BASE_T),
        origin=Origin(
            xyz=(0.125, 0.036, BASE_T / 2.0),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material="fastener_dark",
        name="right_rear_mount_boss",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T + GUIDE_RAIL_H + END_STOP_H)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_L, CARRIAGE_W, CARRIAGE_TOP_T)),
        material="carriage_orange",
        name="top_deck",
    )
    carriage.visual(
        Box((CARRIAGE_L, SKIRT_W, SKIRT_H)),
        origin=Origin(
            xyz=(
                0.0,
                (CARRIAGE_W / 2.0) - (SKIRT_W / 2.0),
                -0.007,
            )
        ),
        material="carriage_orange",
        name="left_skirt",
    )
    carriage.visual(
        Box((CARRIAGE_L, SKIRT_W, SKIRT_H)),
        origin=Origin(
            xyz=(
                0.0,
                -((CARRIAGE_W / 2.0) - (SKIRT_W / 2.0)),
                -0.007,
            )
        ),
        material="carriage_orange",
        name="right_skirt",
    )
    carriage.visual(
        Box((GUIDE_PAD_L, GUIDE_PAD_T, GUIDE_PAD_H)),
        origin=Origin(
            xyz=(
                0.0,
                (GUIDE_RAIL_W / 2.0) + 0.00375,
                -0.011,
            )
        ),
        material="rail_steel",
        name="left_guide_pad",
    )
    carriage.visual(
        Box((GUIDE_PAD_L, GUIDE_PAD_T, GUIDE_PAD_H)),
        origin=Origin(
            xyz=(
                0.0,
                -((GUIDE_RAIL_W / 2.0) + 0.00375),
                -0.011,
            )
        ),
        material="rail_steel",
        name="right_guide_pad",
    )
    carriage.visual(
        Box((0.024, 0.024, 0.012)),
        origin=Origin(xyz=(PIVOT_X - 0.008, 0.0, 0.001)),
        material="fastener_dark",
        name="pivot_pedestal",
    )
    carriage.visual(
        Box((0.020, EAR_T, EAR_H)),
        origin=Origin(xyz=(PIVOT_X, EAR_CENTER_Y, EAR_CENTER_Z_LOCAL)),
        material="carriage_orange",
        name="left_ear",
    )
    carriage.visual(
        Box((0.020, EAR_T, EAR_H)),
        origin=Origin(xyz=(PIVOT_X, -EAR_CENTER_Y, EAR_CENTER_Z_LOCAL)),
        material="carriage_orange",
        name="right_ear",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_L, CARRIAGE_W, 0.032)),
        mass=1.0,
    )

    link_frame = model.part("link_frame")
    link_frame.visual(
        Cylinder(radius=BARREL_R, length=BARREL_L),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material="fastener_dark",
        name="hinge_barrel",
    )
    link_frame.visual(
        Box((LINK_RAIL_L, LINK_RAIL_T, LINK_RAIL_H)),
        origin=Origin(
            xyz=(LINK_RAIL_CENTER_X, LINK_RAIL_CENTER_Y, 0.004),
        ),
        material="link_blue",
        name="left_rail",
    )
    link_frame.visual(
        Box((LINK_RAIL_L, LINK_RAIL_T, LINK_RAIL_H)),
        origin=Origin(
            xyz=(LINK_RAIL_CENTER_X, -LINK_RAIL_CENTER_Y, 0.004),
        ),
        material="link_blue",
        name="right_rail",
    )
    link_frame.visual(
        Box((GUIDE_FLOOR_L, LINK_INNER_W, GUIDE_FLOOR_T)),
        origin=Origin(xyz=(GUIDE_FLOOR_CENTER_X, 0.0, -0.006)),
        material="link_blue",
        name="guide_floor",
    )
    link_frame.visual(
        Box((FRONT_BRIDGE_L, LINK_OUTER_W, 0.018)),
        origin=Origin(xyz=(FRONT_BRIDGE_CENTER_X, 0.0, 0.003)),
        material="link_blue",
        name="front_bridge",
    )
    link_frame.inertial = Inertial.from_geometry(
        Box((LINK_RAIL_L + 0.010, LINK_OUTER_W, 0.020)),
        mass=0.65,
        origin=Origin(xyz=(0.085, 0.0, 0.001)),
    )

    terminal_slider = model.part("terminal_slider")
    terminal_slider.visual(
        Box((SLIDER_L, SLIDER_W, SLIDER_H)),
        material="slider_silver",
        name="shoe",
    )
    terminal_slider.visual(
        Box((SLIDER_CAP_L, SLIDER_CAP_W, SLIDER_CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="slider_silver",
        name="cap",
    )
    terminal_slider.inertial = Inertial.from_geometry(
        Box((SLIDER_L, SLIDER_W, 0.012)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    model.articulation(
        "base_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-BASE_SLIDE_TRAVEL / 2.0, 0.0, CARRIAGE_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=BASE_SLIDE_TRAVEL,
            effort=180.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "carriage_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=link_frame,
        origin=Origin(xyz=(PIVOT_X, 0.0, EAR_CENTER_Z_LOCAL)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=HINGE_OPEN_LIMIT,
            effort=30.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "link_slider",
        ArticulationType.PRISMATIC,
        parent=link_frame,
        child=terminal_slider,
        origin=Origin(xyz=(SLIDER_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDER_TRAVEL,
            effort=60.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    link_frame = object_model.get_part("link_frame")
    terminal_slider = object_model.get_part("terminal_slider")

    base_slide = object_model.get_articulation("base_slide")
    carriage_hinge = object_model.get_articulation("carriage_hinge")
    link_slider = object_model.get_articulation("link_slider")

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
        "stage_order_and_types",
        base_slide.articulation_type == ArticulationType.PRISMATIC
        and carriage_hinge.articulation_type == ArticulationType.REVOLUTE
        and link_slider.articulation_type == ArticulationType.PRISMATIC,
        details="Expected prismatic → revolute → prismatic articulation stack.",
    )
    ctx.check(
        "joint_axes_match_mechanism",
        base_slide.axis == (1.0, 0.0, 0.0)
        and carriage_hinge.axis == (0.0, -1.0, 0.0)
        and link_slider.axis == (1.0, 0.0, 0.0),
        details="Stage axes should be slide along X, hinge about -Y, and distal slide along link-local X.",
    )

    ctx.expect_contact(
        carriage,
        base,
        elem_a="left_skirt",
        elem_b="base_plate",
        name="carriage_left_skirt_supported",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a="right_skirt",
        elem_b="base_plate",
        name="carriage_right_skirt_supported",
    )
    ctx.expect_contact(
        link_frame,
        carriage,
        elem_a="hinge_barrel",
        elem_b="left_ear",
        name="hinge_barrel_captured_by_left_ear",
    )
    ctx.expect_contact(
        terminal_slider,
        link_frame,
        elem_a="shoe",
        elem_b="guide_floor",
        name="terminal_slider_supported_on_guide_floor",
    )
    ctx.expect_within(
        terminal_slider,
        link_frame,
        axes="yz",
        margin=0.002,
        name="terminal_slider_captured_in_link_frame",
    )

    with ctx.pose({base_slide: 0.0}):
        slide_home = ctx.part_world_position(carriage)
    with ctx.pose({base_slide: BASE_SLIDE_TRAVEL}):
        slide_end = ctx.part_world_position(carriage)
    if slide_home is not None and slide_end is not None:
        dx = slide_end[0] - slide_home[0]
        dy = slide_end[1] - slide_home[1]
        dz = slide_end[2] - slide_home[2]
        ctx.check(
            "base_slide_moves_carriage_linearly",
            dx > 0.135 and abs(dy) < 0.001 and abs(dz) < 0.001,
            details=f"Carriage displacement was dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}.",
        )
    else:
        ctx.fail("base_slide_moves_carriage_linearly", "Could not resolve carriage world positions.")

    with ctx.pose({carriage_hinge: 0.0}):
        bridge_closed = ctx.part_element_world_aabb(link_frame, elem="front_bridge")
    with ctx.pose({carriage_hinge: 0.85}):
        bridge_open = ctx.part_element_world_aabb(link_frame, elem="front_bridge")
    if bridge_closed is not None and bridge_open is not None:
        closed_center_z = (bridge_closed[0][2] + bridge_closed[1][2]) / 2.0
        open_center_z = (bridge_open[0][2] + bridge_open[1][2]) / 2.0
        ctx.check(
            "hinge_opens_link_upward",
            open_center_z > closed_center_z + 0.08,
            details=f"Front bridge center z moved from {closed_center_z:.4f} to {open_center_z:.4f}.",
        )
    else:
        ctx.fail("hinge_opens_link_upward", "Could not resolve front bridge AABBs.")

    with ctx.pose({carriage_hinge: 0.70, link_slider: 0.0}):
        slider_retracted = ctx.part_world_position(terminal_slider)
    with ctx.pose({carriage_hinge: 0.70, link_slider: SLIDER_TRAVEL}):
        slider_extended = ctx.part_world_position(terminal_slider)
    if slider_retracted is not None and slider_extended is not None:
        dx = slider_extended[0] - slider_retracted[0]
        dy = slider_extended[1] - slider_retracted[1]
        dz = slider_extended[2] - slider_retracted[2]
        ctx.check(
            "terminal_slider_follows_rotated_link_axis",
            dx > 0.05 and dz > 0.04 and abs(dy) < 0.002,
            details=f"Extended slider displacement was dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}.",
        )
    else:
        ctx.fail(
            "terminal_slider_follows_rotated_link_axis",
            "Could not resolve terminal slider world positions in the pitched pose.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
