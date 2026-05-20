from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CHASSIS_WIDTH = 0.430
CHASSIS_DEPTH = 0.330
CHASSIS_FRONT_X = -CHASSIS_DEPTH * 0.5
CHASSIS_BACK_X = CHASSIS_DEPTH * 0.5
SIDE_WALL_HEIGHT = 0.082
WALL_THICKNESS = 0.0025
FRONT_BEZEL_THICKNESS = 0.010
FLOOR_THICKNESS = 0.0025

DOOR_WIDTH = 0.322
DOOR_HEIGHT = 0.066
DOOR_HINGE_Y = 0.146
DOOR_CENTER_Z = 0.045
DOOR_PANEL_THICKNESS = 0.003
DOOR_PIVOT_X = -0.1635
DOOR_OPEN_LIMIT = 1.35

TOP_COVER_WIDTH = 0.421
TOP_COVER_DEPTH = 0.314
TOP_COVER_TOP_THICKNESS = 0.002
TOP_COVER_SIDE_HEIGHT = 0.011
TOP_COVER_FRONT_X = -0.149
TOP_COVER_RAIL_Z = SIDE_WALL_HEIGHT
TOP_COVER_TRAVEL = 0.110
RUNNER_LENGTH = 0.302
RUNNER_WIDTH = 0.016
RUNNER_HEIGHT = 0.0025
RAIL_LENGTH = 0.300
RAIL_WIDTH = 0.024
RAIL_HEIGHT = 0.0025


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="htpc_chassis")

    outer_shell = model.material("outer_shell", rgba=(0.14, 0.15, 0.17, 1.0))
    front_trim = model.material("front_trim", rgba=(0.24, 0.25, 0.28, 1.0))
    aluminum_panel = model.material("aluminum_panel", rgba=(0.68, 0.70, 0.74, 1.0))
    bay_black = model.material("bay_black", rgba=(0.10, 0.11, 0.12, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.22, 0.26, 0.55))
    accent_black = model.material("accent_black", rgba=(0.07, 0.07, 0.08, 1.0))

    chassis = model.part("chassis")

    side_shell_depth = CHASSIS_DEPTH - FRONT_BEZEL_THICKNESS
    side_shell_center_x = CHASSIS_BACK_X - side_shell_depth * 0.5
    wall_center_z = FLOOR_THICKNESS + SIDE_WALL_HEIGHT * 0.5

    chassis.visual(
        Box((side_shell_depth, CHASSIS_WIDTH, FLOOR_THICKNESS)),
        origin=Origin(xyz=(side_shell_center_x, 0.0, FLOOR_THICKNESS * 0.5)),
        material=outer_shell,
        name="floor_pan",
    )
    chassis.visual(
        Box((side_shell_depth, WALL_THICKNESS, SIDE_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                side_shell_center_x,
                -(CHASSIS_WIDTH * 0.5 - WALL_THICKNESS * 0.5),
                wall_center_z,
            )
        ),
        material=outer_shell,
        name="left_wall",
    )
    chassis.visual(
        Box((side_shell_depth, WALL_THICKNESS, SIDE_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                side_shell_center_x,
                CHASSIS_WIDTH * 0.5 - WALL_THICKNESS * 0.5,
                wall_center_z,
            )
        ),
        material=outer_shell,
        name="right_wall",
    )
    chassis.visual(
        Box((WALL_THICKNESS, CHASSIS_WIDTH - 2.0 * WALL_THICKNESS, SIDE_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                CHASSIS_BACK_X - WALL_THICKNESS * 0.5,
                0.0,
                wall_center_z,
            )
        ),
        material=outer_shell,
        name="rear_wall",
    )

    chassis.visual(
        Box((FRONT_BEZEL_THICKNESS, CHASSIS_WIDTH, 0.0095)),
        origin=Origin(xyz=(-0.160, 0.0, 0.00725)),
        material=front_trim,
        name="front_lower_bar",
    )
    chassis.visual(
        Box((FRONT_BEZEL_THICKNESS, 0.039, DOOR_HEIGHT)),
        origin=Origin(xyz=(-0.160, -0.1955, DOOR_CENTER_Z)),
        material=front_trim,
        name="front_left_stile",
    )
    chassis.visual(
        Box((FRONT_BEZEL_THICKNESS, 0.069, SIDE_WALL_HEIGHT - FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                -0.160,
                0.1805,
                FLOOR_THICKNESS + (SIDE_WALL_HEIGHT - FLOOR_THICKNESS) * 0.5,
            )
        ),
        material=front_trim,
        name="front_right_control_strip",
    )
    chassis.visual(
        Box((FRONT_BEZEL_THICKNESS, 0.361, 0.004)),
        origin=Origin(xyz=(-0.160, -0.0345, 0.080)),
        material=front_trim,
        name="front_top_bar",
    )

    chassis.visual(
        Box((0.008, 0.300, DOOR_HEIGHT)),
        origin=Origin(xyz=(-0.151, -0.015, DOOR_CENTER_Z)),
        material=bay_black,
        name="bay_module",
    )
    chassis.visual(
        Box((0.003, 0.286, 0.026)),
        origin=Origin(xyz=(-0.1535, -0.015, 0.059)),
        material=accent_black,
        name="upper_bay_face",
    )
    chassis.visual(
        Box((0.003, 0.286, 0.026)),
        origin=Origin(xyz=(-0.1535, -0.015, 0.031)),
        material=accent_black,
        name="lower_bay_face",
    )

    chassis.visual(
        Box((0.001, 0.040, 0.010)),
        origin=Origin(xyz=(-0.1645, 0.1805, 0.060)),
        material=smoked_glass,
        name="status_window",
    )
    chassis.visual(
        Box((0.001, 0.018, 0.018)),
        origin=Origin(xyz=(-0.1645, 0.1805, 0.024)),
        material=accent_black,
        name="power_button",
    )
    chassis.visual(
        Box((0.001, 0.026, 0.006)),
        origin=Origin(xyz=(-0.1645, 0.1805, 0.042)),
        material=accent_black,
        name="usb_port_cluster",
    )

    chassis.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.010, -0.2005, 0.08075)),
        material=outer_shell,
        name="left_rail",
    )
    chassis.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.010, 0.2005, 0.08075)),
        material=outer_shell,
        name="right_rail",
    )

    chassis.inertial = Inertial.from_geometry(
        Box((CHASSIS_DEPTH, CHASSIS_WIDTH, 0.095)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
    )

    front_door = model.part("front_door")
    front_door.visual(
        Box((DOOR_PANEL_THICKNESS, DOOR_WIDTH, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.0, -DOOR_WIDTH * 0.5, 0.0)),
        material=aluminum_panel,
        name="door_panel",
    )
    front_door.visual(
        Box((0.010, 0.016, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.0065, -0.008, 0.0)),
        material=front_trim,
        name="door_hinge_stile",
    )
    front_door.visual(
        Box((0.004, 0.018, 0.028)),
        origin=Origin(xyz=(-0.0035, -0.307, 0.0)),
        material=accent_black,
        name="door_pull",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((0.012, DOOR_WIDTH, DOOR_HEIGHT)),
        mass=0.8,
        origin=Origin(xyz=(0.004, -DOOR_WIDTH * 0.5, 0.0)),
    )

    model.articulation(
        "chassis_to_front_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(xyz=(DOOR_PIVOT_X, DOOR_HINGE_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=DOOR_OPEN_LIMIT,
        ),
    )

    top_cover = model.part("top_cover")
    top_cover.visual(
        Box((TOP_COVER_DEPTH, TOP_COVER_WIDTH, TOP_COVER_TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                TOP_COVER_DEPTH * 0.5,
                0.0,
                TOP_COVER_SIDE_HEIGHT + TOP_COVER_TOP_THICKNESS * 0.5,
            )
        ),
        material=outer_shell,
        name="top_skin",
    )
    top_cover.visual(
        Box((TOP_COVER_DEPTH, 0.004, TOP_COVER_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                TOP_COVER_DEPTH * 0.5,
                -(TOP_COVER_WIDTH * 0.5 - 0.002),
                TOP_COVER_SIDE_HEIGHT * 0.5,
            )
        ),
        material=outer_shell,
        name="left_skirt",
    )
    top_cover.visual(
        Box((TOP_COVER_DEPTH, 0.004, TOP_COVER_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                TOP_COVER_DEPTH * 0.5,
                TOP_COVER_WIDTH * 0.5 - 0.002,
                TOP_COVER_SIDE_HEIGHT * 0.5,
            )
        ),
        material=outer_shell,
        name="right_skirt",
    )
    top_cover.visual(
        Box((0.003, TOP_COVER_WIDTH, TOP_COVER_SIDE_HEIGHT)),
        origin=Origin(xyz=(0.0015, 0.0, TOP_COVER_SIDE_HEIGHT * 0.5)),
        material=front_trim,
        name="front_lip",
    )
    top_cover.visual(
        Box((0.003, TOP_COVER_WIDTH, TOP_COVER_SIDE_HEIGHT)),
        origin=Origin(xyz=(0.3100, 0.0, TOP_COVER_SIDE_HEIGHT * 0.5)),
        material=front_trim,
        name="rear_lip",
    )
    top_cover.visual(
        Box((RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(0.157, -0.1985, RUNNER_HEIGHT * 0.5)),
        material=front_trim,
        name="left_runner",
    )
    top_cover.visual(
        Box((RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(0.157, 0.1985, RUNNER_HEIGHT * 0.5)),
        material=front_trim,
        name="right_runner",
    )
    top_cover.inertial = Inertial.from_geometry(
        Box((TOP_COVER_DEPTH, TOP_COVER_WIDTH, 0.014)),
        mass=1.2,
        origin=Origin(xyz=(TOP_COVER_DEPTH * 0.5, 0.0, 0.007)),
    )

    model.articulation(
        "chassis_to_top_cover",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=top_cover,
        origin=Origin(xyz=(TOP_COVER_FRONT_X, 0.0, TOP_COVER_RAIL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=TOP_COVER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    front_door = object_model.get_part("front_door")
    top_cover = object_model.get_part("top_cover")
    door_hinge = object_model.get_articulation("chassis_to_front_door")
    cover_slide = object_model.get_articulation("chassis_to_top_cover")

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
        front_door,
        chassis,
        name="front door is mounted to and seated against the chassis frame",
    )
    ctx.expect_contact(
        top_cover,
        chassis,
        elem_a="left_runner",
        elem_b="left_rail",
        name="left runner sits on the left chassis rail",
    )
    ctx.expect_contact(
        top_cover,
        chassis,
        elem_a="right_runner",
        elem_b="right_rail",
        name="right runner sits on the right chassis rail",
    )
    ctx.expect_within(
        top_cover,
        chassis,
        axes="y",
        inner_elem="left_runner",
        outer_elem="left_rail",
        margin=0.0,
        name="left runner stays laterally captured by the left rail",
    )
    ctx.expect_within(
        top_cover,
        chassis,
        axes="y",
        inner_elem="right_runner",
        outer_elem="right_rail",
        margin=0.0,
        name="right runner stays laterally captured by the right rail",
    )
    ctx.expect_overlap(
        top_cover,
        chassis,
        axes="x",
        elem_a="left_runner",
        elem_b="left_rail",
        min_overlap=0.290,
        name="closed top cover has deep retained insertion on the left rail",
    )
    ctx.expect_overlap(
        top_cover,
        chassis,
        axes="x",
        elem_a="right_runner",
        elem_b="right_rail",
        min_overlap=0.290,
        name="closed top cover has deep retained insertion on the right rail",
    )

    rest_door_panel = ctx.part_element_world_aabb(front_door, elem="door_panel")
    with ctx.pose({door_hinge: 1.15}):
        open_door_panel = ctx.part_element_world_aabb(front_door, elem="door_panel")
    ctx.check(
        "front door swings outward on its right hinge",
        rest_door_panel is not None
        and open_door_panel is not None
        and open_door_panel[0][0] < rest_door_panel[0][0] - 0.045,
        details=f"rest_panel={rest_door_panel}, open_panel={open_door_panel}",
    )

    rest_cover_pos = ctx.part_world_position(top_cover)
    with ctx.pose({cover_slide: TOP_COVER_TRAVEL}):
        extended_cover_pos = ctx.part_world_position(top_cover)
        ctx.expect_overlap(
            top_cover,
            chassis,
            axes="x",
            elem_a="left_runner",
            elem_b="left_rail",
            min_overlap=0.185,
            name="extended top cover still retains left-rail engagement",
        )
        ctx.expect_overlap(
            top_cover,
            chassis,
            axes="x",
            elem_a="right_runner",
            elem_b="right_rail",
            min_overlap=0.185,
            name="extended top cover still retains right-rail engagement",
        )
    ctx.check(
        "top cover slides rearward for service access",
        rest_cover_pos is not None
        and extended_cover_pos is not None
        and extended_cover_pos[0] > rest_cover_pos[0] + 0.09,
        details=f"rest_cover_pos={rest_cover_pos}, extended_cover_pos={extended_cover_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
