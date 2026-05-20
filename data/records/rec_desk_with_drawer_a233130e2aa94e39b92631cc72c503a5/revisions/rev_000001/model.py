from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


WIDTH = 0.94
DEPTH = 0.42
HEIGHT = 1.24

HALF_W = WIDTH / 2.0
HALF_D = DEPTH / 2.0

SIDE_T = 0.02
BACK_T = 0.015
SHELF_T = 0.02
FACE_T = 0.018

PLINTH_H = 0.08

INNER_WIDTH = WIDTH - 2.0 * SIDE_T
INNER_DEPTH = DEPTH - BACK_T - FACE_T
INNER_CENTER_Y = (-HALF_D + FACE_T + HALF_D - BACK_T) / 2.0

BOTTOM_PANEL_CENTER_Z = PLINTH_H + SHELF_T / 2.0
BOTTOM_PANEL_TOP_Z = PLINTH_H + SHELF_T

DRAWER_OPEN_BOTTOM_Z = BOTTOM_PANEL_TOP_Z
DRAWER_OPEN_TOP_Z = 0.30
WRITING_HINGE_Z = DRAWER_OPEN_TOP_Z + SHELF_T
WRITING_OPEN_TOP_Z = 0.70
UPPER_OPEN_BOTTOM_Z = WRITING_OPEN_TOP_Z + SHELF_T
UPPER_OPEN_TOP_Z = HEIGHT - SHELF_T

DRAWER_FACE_W = INNER_WIDTH - 0.012
DRAWER_FACE_H = 0.192
DRAWER_SHELL_W = 0.84
DRAWER_SHELL_D = 0.34
DRAWER_SIDE_H = 0.16
DRAWER_BOTTOM_Z = DRAWER_OPEN_BOTTOM_Z + 0.004
DRAWER_ORIGIN_Y = -0.04
DRAWER_SLIDE_TRAVEL = 0.22

WRITING_PANEL_W = INNER_WIDTH - 0.012
WRITING_PANEL_H = 0.376

DOOR_W = 0.438
DOOR_H = 0.488
LEFT_DOOR_HINGE_X = -0.444
RIGHT_DOOR_HINGE_X = 0.444
DOOR_CENTER_Z = UPPER_OPEN_BOTTOM_Z + 0.006 + DOOR_H / 2.0

GUIDE_Z = DRAWER_BOTTOM_Z + 0.11
GUIDE_Y = 0.015
GUIDE_LEN = 0.32
RUNNER_LEN = 0.30

WRITING_HINGE_R = 0.0045
DOOR_HINGE_R = 0.005


def add_box(part, name, size, xyz, material):
    return part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def add_cylinder(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="secretary_desk")

    walnut = model.material("walnut", color=(0.36, 0.23, 0.14))
    interior_wood = model.material("interior_wood", color=(0.49, 0.35, 0.23))
    brass = model.material("brass", color=(0.78, 0.66, 0.28))
    steel = model.material("steel", color=(0.64, 0.66, 0.70))

    carcass = model.part("carcass")
    add_box(
        carcass,
        "left_side",
        (SIDE_T, DEPTH, HEIGHT),
        (-HALF_W + SIDE_T / 2.0, 0.0, HEIGHT / 2.0),
        walnut,
    )
    add_box(
        carcass,
        "right_side",
        (SIDE_T, DEPTH, HEIGHT),
        (HALF_W - SIDE_T / 2.0, 0.0, HEIGHT / 2.0),
        walnut,
    )
    add_box(
        carcass,
        "back_panel",
        (INNER_WIDTH, BACK_T, HEIGHT),
        (0.0, HALF_D - BACK_T / 2.0, HEIGHT / 2.0),
        interior_wood,
    )
    add_box(
        carcass,
        "bottom_panel",
        (INNER_WIDTH, INNER_DEPTH, SHELF_T),
        (0.0, INNER_CENTER_Y, BOTTOM_PANEL_CENTER_Z),
        interior_wood,
    )
    add_box(
        carcass,
        "writing_floor",
        (INNER_WIDTH, INNER_DEPTH, SHELF_T),
        (0.0, INNER_CENTER_Y, DRAWER_OPEN_TOP_Z + SHELF_T / 2.0),
        interior_wood,
    )
    add_box(
        carcass,
        "upper_cabinet_floor",
        (INNER_WIDTH, INNER_DEPTH, SHELF_T),
        (0.0, INNER_CENTER_Y, WRITING_OPEN_TOP_Z + SHELF_T / 2.0),
        interior_wood,
    )
    add_box(
        carcass,
        "top_panel",
        (INNER_WIDTH, INNER_DEPTH, SHELF_T),
        (0.0, INNER_CENTER_Y, HEIGHT - SHELF_T / 2.0),
        walnut,
    )
    add_box(
        carcass,
        "plinth_front",
        (INNER_WIDTH, FACE_T, PLINTH_H),
        (0.0, -HALF_D + FACE_T / 2.0, PLINTH_H / 2.0),
        walnut,
    )
    add_box(
        carcass,
        "upper_center_stile",
        (0.01, FACE_T, UPPER_OPEN_TOP_Z - UPPER_OPEN_BOTTOM_Z),
        (0.0, -HALF_D + FACE_T / 2.0, (UPPER_OPEN_TOP_Z + UPPER_OPEN_BOTTOM_Z) / 2.0),
        walnut,
    )
    add_box(
        carcass,
        "left_drawer_guide",
        (0.024, GUIDE_LEN, 0.03),
        (-0.438, GUIDE_Y, GUIDE_Z),
        steel,
    )
    add_box(
        carcass,
        "right_drawer_guide",
        (0.024, GUIDE_LEN, 0.03),
        (0.438, GUIDE_Y, GUIDE_Z),
        steel,
    )
    add_cylinder(
        carcass,
        "writing_hinge_sleeve",
        WRITING_HINGE_R,
        INNER_WIDTH,
        (0.0, -HALF_D - WRITING_HINGE_R, WRITING_HINGE_Z),
        steel,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    add_box(
        carcass,
        "organizer_shelf",
        (0.62, 0.20, 0.012),
        (0.0, 0.095, 0.582),
        interior_wood,
    )
    for idx, x_pos in enumerate((-0.16, 0.0, 0.16), start=1):
        add_box(
            carcass,
            f"organizer_divider_{idx}",
            (0.012, 0.20, 0.112),
            (x_pos, 0.095, 0.644),
            interior_wood,
        )

    writing_panel = model.part("writing_panel")
    add_box(
        writing_panel,
        "writing_front",
        (WRITING_PANEL_W, FACE_T, WRITING_PANEL_H),
        (0.0, WRITING_HINGE_R + FACE_T / 2.0, WRITING_PANEL_H / 2.0),
        walnut,
    )
    add_cylinder(
        writing_panel,
        "hinge_pin",
        WRITING_HINGE_R,
        WRITING_PANEL_W,
        (0.0, 0.0, 0.0),
        steel,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    add_cylinder(
        writing_panel,
        "pull_knob",
        0.0065,
        0.024,
        (0.0, WRITING_HINGE_R - 0.012, 0.25),
        brass,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )

    model.articulation(
        "writing_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=writing_panel,
        origin=Origin(xyz=(0.0, -HALF_D - WRITING_HINGE_R, WRITING_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )

    upper_left_door = model.part("upper_left_door")
    add_box(
        upper_left_door,
        "door_panel",
        (DOOR_W, FACE_T, DOOR_H),
        (DOOR_W / 2.0, DOOR_HINGE_R + FACE_T / 2.0, 0.0),
        walnut,
    )
    add_cylinder(
        upper_left_door,
        "hinge_pin",
        DOOR_HINGE_R,
        DOOR_H,
        (0.0, 0.0, 0.0),
        steel,
    )
    add_cylinder(
        upper_left_door,
        "door_knob",
        0.006,
        0.024,
        (DOOR_W - 0.05, DOOR_HINGE_R - 0.012, -0.04),
        brass,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )

    model.articulation(
        "left_upper_door_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=upper_left_door,
        origin=Origin(xyz=(-HALF_W + SIDE_T, -HALF_D - DOOR_HINGE_R, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(96.0),
        ),
    )

    upper_right_door = model.part("upper_right_door")
    add_box(
        upper_right_door,
        "door_panel",
        (DOOR_W, FACE_T, DOOR_H),
        (-DOOR_W / 2.0, DOOR_HINGE_R + FACE_T / 2.0, 0.0),
        walnut,
    )
    add_cylinder(
        upper_right_door,
        "hinge_pin",
        DOOR_HINGE_R,
        DOOR_H,
        (0.0, 0.0, 0.0),
        steel,
    )
    add_cylinder(
        upper_right_door,
        "door_knob",
        0.006,
        0.024,
        (-DOOR_W + 0.05, DOOR_HINGE_R - 0.012, -0.04),
        brass,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )

    model.articulation(
        "right_upper_door_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=upper_right_door,
        origin=Origin(xyz=(HALF_W - SIDE_T, -HALF_D - DOOR_HINGE_R, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(96.0),
        ),
    )

    base_drawer = model.part("base_drawer")
    add_box(
        base_drawer,
        "drawer_face",
        (DRAWER_FACE_W, 0.02, DRAWER_FACE_H),
        (0.0, -DRAWER_SHELL_D / 2.0 + 0.01, DRAWER_FACE_H / 2.0),
        walnut,
    )
    add_box(
        base_drawer,
        "drawer_left_side",
        (0.012, DRAWER_SHELL_D - 0.02, DRAWER_SIDE_H),
        (-DRAWER_SHELL_W / 2.0 + 0.006, 0.01, DRAWER_SIDE_H / 2.0),
        interior_wood,
    )
    add_box(
        base_drawer,
        "drawer_right_side",
        (0.012, DRAWER_SHELL_D - 0.02, DRAWER_SIDE_H),
        (DRAWER_SHELL_W / 2.0 - 0.006, 0.01, DRAWER_SIDE_H / 2.0),
        interior_wood,
    )
    add_box(
        base_drawer,
        "drawer_back",
        (DRAWER_SHELL_W - 0.024, 0.012, DRAWER_SIDE_H),
        (0.0, DRAWER_SHELL_D / 2.0 - 0.006, DRAWER_SIDE_H / 2.0),
        interior_wood,
    )
    add_box(
        base_drawer,
        "drawer_bottom",
        (DRAWER_SHELL_W - 0.024, DRAWER_SHELL_D - 0.032, 0.012),
        (0.0, 0.014, 0.006),
        interior_wood,
    )
    add_box(
        base_drawer,
        "left_runner",
        (0.018, RUNNER_LEN, 0.026),
        (-0.429, 0.02, 0.11),
        steel,
    )
    add_box(
        base_drawer,
        "right_runner",
        (0.018, RUNNER_LEN, 0.026),
        (0.429, 0.02, 0.11),
        steel,
    )
    add_cylinder(
        base_drawer,
        "pull_post_left",
        0.004,
        0.032,
        (-0.08, -0.186, 0.11),
        brass,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    add_cylinder(
        base_drawer,
        "pull_post_right",
        0.004,
        0.032,
        (0.08, -0.186, 0.11),
        brass,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    add_cylinder(
        base_drawer,
        "pull_bar",
        0.005,
        0.22,
        (0.0, -0.207, 0.11),
        brass,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )

    model.articulation(
        "base_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=base_drawer,
        origin=Origin(xyz=(0.0, DRAWER_ORIGIN_Y, DRAWER_BOTTOM_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    writing_panel = object_model.get_part("writing_panel")
    upper_left_door = object_model.get_part("upper_left_door")
    upper_right_door = object_model.get_part("upper_right_door")
    base_drawer = object_model.get_part("base_drawer")

    writing_hinge = object_model.get_articulation("writing_panel_hinge")
    left_upper_hinge = object_model.get_articulation("left_upper_door_hinge")
    right_upper_hinge = object_model.get_articulation("right_upper_door_hinge")
    drawer_slide = object_model.get_articulation("base_drawer_slide")

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
        writing_panel,
        carcass,
        elem_a=writing_panel.get_visual("hinge_pin"),
        elem_b=carcass.get_visual("writing_hinge_sleeve"),
        reason="The drop-front is carried by a captured piano-hinge pin inside the cabinet hinge sleeve.",
    )
    ctx.allow_overlap(
        base_drawer,
        carcass,
        elem_a=base_drawer.get_visual("left_runner"),
        elem_b=carcass.get_visual("left_drawer_guide"),
        reason="The left drawer runner remains nested inside the fixed side guide through its travel.",
    )
    ctx.allow_overlap(
        base_drawer,
        carcass,
        elem_a=base_drawer.get_visual("right_runner"),
        elem_b=carcass.get_visual("right_drawer_guide"),
        reason="The right drawer runner remains nested inside the fixed side guide through its travel.",
    )

    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_overlap(
        writing_panel,
        carcass,
        axes="yz",
        elem_a="hinge_pin",
        elem_b="writing_hinge_sleeve",
        min_overlap=0.008,
        name="writing_hinge_capture_closed",
    )
    ctx.expect_contact(
        upper_left_door,
        carcass,
        elem_a="hinge_pin",
        elem_b="left_side",
        name="left_door_hinge_capture_closed",
    )
    ctx.expect_contact(
        upper_right_door,
        carcass,
        elem_a="hinge_pin",
        elem_b="right_side",
        name="right_door_hinge_capture_closed",
    )
    ctx.expect_overlap(
        base_drawer,
        carcass,
        axes="yz",
        elem_a="left_runner",
        elem_b="left_drawer_guide",
        min_overlap=0.02,
        name="left_runner_engaged_closed",
    )
    ctx.expect_overlap(
        base_drawer,
        carcass,
        axes="yz",
        elem_a="right_runner",
        elem_b="right_drawer_guide",
        min_overlap=0.02,
        name="right_runner_engaged_closed",
    )

    def require_elem_aabb(part, elem_name):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        ctx.check(
            f"{part.name}_{elem_name}_aabb_exists",
            aabb is not None,
            f"Expected element '{elem_name}' on part '{part.name}' to have a world AABB.",
        )
        assert aabb is not None
        return aabb

    writing_closed = require_elem_aabb(writing_panel, "writing_front")
    left_door_closed = require_elem_aabb(upper_left_door, "door_panel")
    right_door_closed = require_elem_aabb(upper_right_door, "door_panel")
    drawer_closed = require_elem_aabb(base_drawer, "drawer_face")

    ctx.check(
        "writing_panel_closed_flush",
        abs(writing_closed[0][1] + HALF_D) < 0.003,
        f"Writing panel front sits at y={writing_closed[0][1]:.4f}, expected {-HALF_D:.4f}.",
    )
    ctx.check(
        "left_door_closed_flush",
        abs(left_door_closed[0][1] + HALF_D) < 0.003,
        f"Left upper door front sits at y={left_door_closed[0][1]:.4f}, expected {-HALF_D:.4f}.",
    )
    ctx.check(
        "right_door_closed_flush",
        abs(right_door_closed[0][1] + HALF_D) < 0.003,
        f"Right upper door front sits at y={right_door_closed[0][1]:.4f}, expected {-HALF_D:.4f}.",
    )
    ctx.check(
        "drawer_closed_flush",
        abs(drawer_closed[0][1] + HALF_D) < 0.003,
        f"Drawer face front sits at y={drawer_closed[0][1]:.4f}, expected {-HALF_D:.4f}.",
    )
    ctx.check(
        "drawer_face_below_writing_panel",
        drawer_closed[1][2] < writing_closed[0][2],
        f"Drawer face top z={drawer_closed[1][2]:.4f} should stay below writing panel bottom z={writing_closed[0][2]:.4f}.",
    )
    ctx.check(
        "upper_doors_above_writing_panel",
        left_door_closed[0][2] > writing_closed[1][2] and right_door_closed[0][2] > writing_closed[1][2],
        "Upper doors should sit above the closed writing panel opening.",
    )

    for joint in (writing_hinge, left_upper_hinge, right_upper_hinge, drawer_slide):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    with ctx.pose({writing_hinge: writing_hinge.motion_limits.upper}):
        writing_open = require_elem_aabb(writing_panel, "writing_front")
        writing_span_y = writing_open[1][1] - writing_open[0][1]
        writing_span_z = writing_open[1][2] - writing_open[0][2]
        ctx.check(
            "writing_panel_folds_down",
            writing_open[0][1] < -0.56 and writing_span_y > 0.34 and writing_span_z < 0.04,
            (
                "Writing panel did not read as folded horizontal in front of the desk: "
                f"min_y={writing_open[0][1]:.4f}, span_y={writing_span_y:.4f}, span_z={writing_span_z:.4f}."
            ),
        )
        ctx.expect_overlap(
            writing_panel,
            carcass,
            axes="yz",
            elem_a="hinge_pin",
            elem_b="writing_hinge_sleeve",
            min_overlap=0.008,
            name="writing_hinge_capture_open",
        )

    with ctx.pose({left_upper_hinge: left_upper_hinge.motion_limits.upper}):
        left_open = require_elem_aabb(upper_left_door, "door_panel")
        left_span_x = left_open[1][0] - left_open[0][0]
        left_span_y = left_open[1][1] - left_open[0][1]
        ctx.check(
            "left_upper_door_swings_out",
            left_open[0][1] < -0.60 and left_span_y > 0.40 and left_span_x < 0.08,
            (
                "Left door did not swing sideways into an open pose: "
                f"min_y={left_open[0][1]:.4f}, span_x={left_span_x:.4f}, span_y={left_span_y:.4f}."
            ),
        )
        ctx.expect_contact(
            upper_left_door,
            carcass,
            elem_a="hinge_pin",
            elem_b="left_side",
            name="left_door_hinge_capture_open",
        )

    with ctx.pose({right_upper_hinge: right_upper_hinge.motion_limits.upper}):
        right_open = require_elem_aabb(upper_right_door, "door_panel")
        right_span_x = right_open[1][0] - right_open[0][0]
        right_span_y = right_open[1][1] - right_open[0][1]
        ctx.check(
            "right_upper_door_swings_out",
            right_open[0][1] < -0.60 and right_span_y > 0.40 and right_span_x < 0.08,
            (
                "Right door did not swing sideways into an open pose: "
                f"min_y={right_open[0][1]:.4f}, span_x={right_span_x:.4f}, span_y={right_span_y:.4f}."
            ),
        )
        ctx.expect_contact(
            upper_right_door,
            carcass,
            elem_a="hinge_pin",
            elem_b="right_side",
            name="right_door_hinge_capture_open",
        )

    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        drawer_open = require_elem_aabb(base_drawer, "drawer_face")
        ctx.check(
            "drawer_slides_forward",
            drawer_open[0][1] < -0.42,
            f"Drawer face only reached y={drawer_open[0][1]:.4f}; expected a clear forward extension.",
        )
        ctx.expect_overlap(
            base_drawer,
            carcass,
            axes="yz",
            elem_a="left_runner",
            elem_b="left_drawer_guide",
            min_overlap=0.02,
            name="left_runner_engaged_open",
        )
        ctx.expect_overlap(
            base_drawer,
            carcass,
            axes="yz",
            elem_a="right_runner",
            elem_b="right_drawer_guide",
            min_overlap=0.02,
            name="right_runner_engaged_open",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    with ctx.pose(
        {
            writing_hinge: writing_hinge.motion_limits.upper,
            left_upper_hinge: left_upper_hinge.motion_limits.upper,
            right_upper_hinge: right_upper_hinge.motion_limits.upper,
            drawer_slide: drawer_slide.motion_limits.upper,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_open_no_overlap")
        ctx.fail_if_isolated_parts(name="all_open_no_floating")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
