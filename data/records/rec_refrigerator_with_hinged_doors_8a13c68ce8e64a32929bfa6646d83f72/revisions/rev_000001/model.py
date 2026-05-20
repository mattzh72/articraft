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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CABINET_WIDTH = 0.76
CABINET_DEPTH = 0.72
CABINET_HEIGHT = 1.78
SIDE_WALL_THICKNESS = 0.035
TOP_CAP_THICKNESS = 0.05
BOTTOM_PLINTH_HEIGHT = 0.10
BACK_PANEL_THICKNESS = 0.025
DIVIDER_THICKNESS = 0.03
FREEZER_OPENING_HEIGHT = 0.43
FRESH_OPENING_HEIGHT = 1.17
TOP_REVEAL = 0.016
BOTTOM_DOOR_REVEAL = 0.005
DOOR_SEAM = 0.006
FREEZER_DOOR_HEIGHT = 0.44
FRESH_DOOR_HEIGHT = (
    CABINET_HEIGHT
    - TOP_REVEAL
    - (BOTTOM_PLINTH_HEIGHT + BOTTOM_DOOR_REVEAL)
    - DOOR_SEAM
    - FREEZER_DOOR_HEIGHT
)
DOOR_WIDTH = 0.748
DOOR_THICKNESS = 0.065
HANDLE_DEPTH = 0.03
HINGE_PIN_RADIUS = 0.010
HINGE_SLEEVE_RADIUS = 0.013
HINGE_SEGMENT_LENGTH = 0.07
HINGE_OUTSET = 0.012
HINGE_SIDE_OFFSET = 0.018
DOOR_PANEL_X_OFFSET = 0.020
HINGE_CLEARANCE_X = HINGE_PIN_RADIUS + 0.001
DOOR_REAR_GAP = 0.004
HINGE_BRACKET_WIDTH = 0.038
FREEZER_HANDLE_HEIGHT = 0.23
FRESH_HANDLE_HEIGHT = 0.74

CABINET_FRONT_Y = CABINET_DEPTH / 2.0
HINGE_Y = CABINET_FRONT_Y + HINGE_OUTSET
HINGE_X = -(CABINET_WIDTH / 2.0) - HINGE_SIDE_OFFSET
HINGE_BRACKET_Y_MAX = HINGE_Y - HINGE_SLEEVE_RADIUS
HINGE_BRACKET_Y_MIN = CABINET_FRONT_Y - 0.015
INNER_WIDTH = CABINET_WIDTH - (2.0 * SIDE_WALL_THICKNESS)
FRESH_DOOR_BOTTOM_Z = BOTTOM_PLINTH_HEIGHT + BOTTOM_DOOR_REVEAL
FRESH_CENTER_Z = FRESH_DOOR_BOTTOM_Z + (FRESH_DOOR_HEIGHT / 2.0)
FREEZER_BOTTOM_Z = FRESH_DOOR_BOTTOM_Z + FRESH_DOOR_HEIGHT + DOOR_SEAM
FREEZER_CENTER_Z = FREEZER_BOTTOM_Z + (FREEZER_DOOR_HEIGHT / 2.0)


def _hinge_segment_offsets(door_height: float) -> tuple[float, float]:
    edge_offset = (door_height / 2.0) - (HINGE_SEGMENT_LENGTH / 2.0)
    return (edge_offset, -edge_offset)


def _add_door(
    model: ArticulatedObject,
    *,
    part_name: str,
    panel_name: str,
    top_sleeve_name: str,
    bottom_sleeve_name: str,
    handle_name: str,
    door_height: float,
    handle_height: float,
    body_color,
    handle_color,
) -> None:
    door = model.part(part_name)
    panel_y_min = (CABINET_FRONT_Y + DOOR_REAR_GAP) - HINGE_Y
    panel_center_y = panel_y_min + (DOOR_THICKNESS / 2.0)
    bridge_width = DOOR_PANEL_X_OFFSET - HINGE_CLEARANCE_X
    panel_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(DOOR_WIDTH, DOOR_THICKNESS, 0.014),
            door_height,
            center=True,
        ),
        f"{part_name}_panel_shell",
    )
    door.visual(
        panel_mesh,
        origin=Origin(
            xyz=(DOOR_PANEL_X_OFFSET + (DOOR_WIDTH / 2.0), panel_center_y, 0.0)
        ),
        material=body_color,
        name=panel_name,
    )
    door.visual(
        Box((bridge_width, DOOR_THICKNESS, door_height)),
        origin=Origin(
            xyz=(
                HINGE_CLEARANCE_X + (bridge_width / 2.0),
                panel_center_y,
                0.0,
            )
        ),
        material=body_color,
        name=f"{part_name}_hinge_rail",
    )
    handle_x = DOOR_PANEL_X_OFFSET + DOOR_WIDTH - 0.057
    handle_geom = ExtrudeGeometry(
        rounded_rect_profile(0.026, handle_height, 0.010),
        HANDLE_DEPTH,
        center=True,
    ).rotate_x(-math.pi / 2.0)
    handle_mesh = mesh_from_geometry(handle_geom, f"{part_name}_handle")
    door.visual(
        handle_mesh,
        origin=Origin(
            xyz=(
                handle_x,
                panel_y_min + DOOR_THICKNESS + (HANDLE_DEPTH / 2.0) - 0.001,
                0.0,
            )
        ),
        material=handle_color,
        name=handle_name,
    )
    top_offset, bottom_offset = _hinge_segment_offsets(door_height)
    door.visual(
        Cylinder(radius=HINGE_SLEEVE_RADIUS, length=HINGE_SEGMENT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, top_offset)),
        material=handle_color,
        name=top_sleeve_name,
    )
    door.visual(
        Cylinder(radius=HINGE_SLEEVE_RADIUS, length=HINGE_SEGMENT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, bottom_offset)),
        material=handle_color,
        name=bottom_sleeve_name,
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_PANEL_X_OFFSET + DOOR_WIDTH, 0.11, door_height)),
        mass=12.0 if door_height < 0.6 else 21.0,
        origin=Origin(
            xyz=((DOOR_PANEL_X_OFFSET + DOOR_WIDTH) / 2.0, 0.023, 0.0)
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_freezer_refrigerator")

    enamel_white = model.material("enamel_white", rgba=(0.94, 0.95, 0.96, 1.0))
    liner_gray = model.material("liner_gray", rgba=(0.84, 0.86, 0.88, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.63, 0.66, 0.69, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((SIDE_WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CABINET_WIDTH / 2.0) + (SIDE_WALL_THICKNESS / 2.0),
                0.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=enamel_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((SIDE_WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                (CABINET_WIDTH / 2.0) - (SIDE_WALL_THICKNESS / 2.0),
                0.0,
                CABINET_HEIGHT / 2.0,
            )
        ),
        material=enamel_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, TOP_CAP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - (TOP_CAP_THICKNESS / 2.0))),
        material=enamel_white,
        name="top_cap",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, BOTTOM_PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_PLINTH_HEIGHT / 2.0)),
        material=enamel_white,
        name="bottom_plinth",
    )
    cabinet.visual(
        Box(
            (
                CABINET_WIDTH - (2.0 * SIDE_WALL_THICKNESS),
                BACK_PANEL_THICKNESS,
                CABINET_HEIGHT - TOP_CAP_THICKNESS - BOTTOM_PLINTH_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH / 2.0) + (BACK_PANEL_THICKNESS / 2.0),
                BOTTOM_PLINTH_HEIGHT
                + (CABINET_HEIGHT - TOP_CAP_THICKNESS - BOTTOM_PLINTH_HEIGHT) / 2.0,
            )
        ),
        material=liner_gray,
        name="back_panel",
    )
    cabinet.visual(
        Box((INNER_WIDTH, 0.64, DIVIDER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BOTTOM_PLINTH_HEIGHT + FRESH_OPENING_HEIGHT + (DIVIDER_THICKNESS / 2.0),
            )
        ),
        material=liner_gray,
        name="compartment_divider",
    )
    cabinet.visual(
        Box((INNER_WIDTH, 0.60, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_PLINTH_HEIGHT + 0.43)),
        material=liner_gray,
        name="fresh_shelf_lower",
    )
    cabinet.visual(
        Box((INNER_WIDTH, 0.60, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_PLINTH_HEIGHT + 0.77)),
        material=liner_gray,
        name="fresh_shelf_upper",
    )
    cabinet.visual(
        Box((INNER_WIDTH, 0.54, 0.012)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BOTTOM_PLINTH_HEIGHT
                + FRESH_OPENING_HEIGHT
                + DIVIDER_THICKNESS
                + 0.18,
            )
        ),
        material=liner_gray,
        name="freezer_shelf",
    )
    cabinet.visual(
        Box((INNER_WIDTH, 0.24, 0.17)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH / 2.0) + BACK_PANEL_THICKNESS + 0.12,
                BOTTOM_PLINTH_HEIGHT + 0.085,
            )
        ),
        material=trim_gray,
        name="compressor_cover",
    )

    freezer_top_offset, freezer_bottom_offset = _hinge_segment_offsets(
        FREEZER_DOOR_HEIGHT
    )
    fresh_top_offset, fresh_bottom_offset = _hinge_segment_offsets(FRESH_DOOR_HEIGHT)

    bracket_y_size = HINGE_BRACKET_Y_MAX - HINGE_BRACKET_Y_MIN
    bracket_y_center = (HINGE_BRACKET_Y_MAX + HINGE_BRACKET_Y_MIN) / 2.0
    bracket_x_center = HINGE_X + 0.007
    freezer_bottom_z = FREEZER_CENTER_Z + freezer_bottom_offset
    fresh_top_z = FRESH_CENTER_Z + fresh_top_offset
    center_bracket_height = (freezer_bottom_z - fresh_top_z) + HINGE_SEGMENT_LENGTH
    center_bracket_z = (freezer_bottom_z + fresh_top_z) / 2.0

    cabinet.visual(
        Box((HINGE_BRACKET_WIDTH, bracket_y_size, HINGE_SEGMENT_LENGTH + 0.03)),
        origin=Origin(
            xyz=(bracket_x_center, bracket_y_center, FREEZER_CENTER_Z + freezer_top_offset)
        ),
        material=trim_gray,
        name="freezer_top_bracket",
    )
    cabinet.visual(
        Box((HINGE_BRACKET_WIDTH, bracket_y_size, center_bracket_height)),
        origin=Origin(xyz=(bracket_x_center, bracket_y_center, center_bracket_z)),
        material=trim_gray,
        name="center_hinge_bracket",
    )
    cabinet.visual(
        Box((HINGE_BRACKET_WIDTH, bracket_y_size, HINGE_SEGMENT_LENGTH + 0.03)),
        origin=Origin(
            xyz=(bracket_x_center, bracket_y_center, FRESH_CENTER_Z + fresh_bottom_offset)
        ),
        material=trim_gray,
        name="fresh_bottom_bracket",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT / 2.0)),
    )

    _add_door(
        model,
        part_name="freezer_door",
        panel_name="freezer_panel",
        top_sleeve_name="freezer_hinge_top_sleeve",
        bottom_sleeve_name="freezer_hinge_bottom_sleeve",
        handle_name="freezer_handle",
        door_height=FREEZER_DOOR_HEIGHT,
        handle_height=FREEZER_HANDLE_HEIGHT,
        body_color=enamel_white,
        handle_color=charcoal,
    )
    _add_door(
        model,
        part_name="fresh_food_door",
        panel_name="fresh_food_panel",
        top_sleeve_name="fresh_hinge_top_sleeve",
        bottom_sleeve_name="fresh_hinge_bottom_sleeve",
        handle_name="fresh_food_handle",
        door_height=FRESH_DOOR_HEIGHT,
        handle_height=FRESH_HANDLE_HEIGHT,
        body_color=enamel_white,
        handle_color=charcoal,
    )

    model.articulation(
        "freezer_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child="freezer_door",
        origin=Origin(xyz=(HINGE_X, HINGE_Y, FREEZER_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=0.0,
            upper=2.0,
        ),
    )
    model.articulation(
        "fresh_food_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child="fresh_food_door",
        origin=Origin(xyz=(HINGE_X, HINGE_Y, FRESH_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.6,
            lower=0.0,
            upper=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    freezer_door = object_model.get_part("freezer_door")
    fresh_food_door = object_model.get_part("fresh_food_door")
    freezer_hinge = object_model.get_articulation("freezer_hinge")
    fresh_food_hinge = object_model.get_articulation("fresh_food_hinge")

    freezer_panel = freezer_door.get_visual("freezer_panel")
    fresh_panel = fresh_food_door.get_visual("fresh_food_panel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.check(
        "fresh_door_taller_than_freezer_door",
        FRESH_DOOR_HEIGHT > (FREEZER_DOOR_HEIGHT * 2.0),
        "The fresh-food door should be substantially taller than the top freezer door.",
    )
    ctx.check(
        "hinges_share_same_side",
        math.isclose(
            freezer_hinge.origin.xyz[0], fresh_food_hinge.origin.xyz[0], abs_tol=1e-6
        )
        and math.isclose(
            freezer_hinge.origin.xyz[1], fresh_food_hinge.origin.xyz[1], abs_tol=1e-6
        ),
        "Both doors should hinge from the same cabinet side along the same front offset.",
    )
    ctx.check(
        "freezer_hinge_vertical_axis",
        freezer_hinge.axis == (0.0, 0.0, 1.0),
        f"Expected a vertical hinge axis, got {freezer_hinge.axis!r}.",
    )
    ctx.check(
        "fresh_hinge_vertical_axis",
        fresh_food_hinge.axis == (0.0, 0.0, 1.0),
        f"Expected a vertical hinge axis, got {fresh_food_hinge.axis!r}.",
    )
    ctx.check(
        "freezer_above_fresh",
        freezer_hinge.origin.xyz[2] > fresh_food_hinge.origin.xyz[2],
        "The freezer door hinge should sit above the fresh-food door hinge.",
    )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    with ctx.pose({freezer_hinge: 0.0, fresh_food_hinge: 0.0}):
        ctx.expect_contact(cabinet, freezer_door)
        ctx.expect_contact(cabinet, fresh_food_door)
        ctx.expect_gap(
            freezer_door,
            fresh_food_door,
            axis="z",
            min_gap=0.004,
            max_gap=0.008,
            positive_elem=freezer_panel,
            negative_elem=fresh_panel,
        )

    freezer_closed = ctx.part_element_world_aabb(freezer_door, elem=freezer_panel)
    fresh_closed = ctx.part_element_world_aabb(fresh_food_door, elem=fresh_panel)
    cabinet_box = ctx.part_world_aabb(cabinet)
    assert freezer_closed is not None
    assert fresh_closed is not None
    assert cabinet_box is not None

    freezer_width = freezer_closed[1][0] - freezer_closed[0][0]
    fresh_width = fresh_closed[1][0] - fresh_closed[0][0]
    cabinet_front_y = cabinet_box[1][1]

    ctx.check(
        "doors_span_cabinet_width",
        abs(freezer_width - CABINET_WIDTH) < 0.05 and abs(fresh_width - CABINET_WIDTH) < 0.05,
        "Both doors should nearly span the refrigerator case width.",
    )
    ctx.check(
        "doors_project_forward_of_case",
        freezer_closed[1][1] > cabinet_front_y + 0.05
        and fresh_closed[1][1] > cabinet_front_y + 0.05,
        "Closed doors should form the front face of the refrigerator and project ahead of the case.",
    )

    freezer_limits = freezer_hinge.motion_limits
    fresh_limits = fresh_food_hinge.motion_limits
    assert freezer_limits is not None
    assert fresh_limits is not None
    assert freezer_limits.upper is not None
    assert fresh_limits.upper is not None

    with ctx.pose({freezer_hinge: freezer_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="freezer_hinge_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="freezer_hinge_upper_no_floating")
        ctx.expect_contact(cabinet, freezer_door)
        freezer_open = ctx.part_element_world_aabb(freezer_door, elem=freezer_panel)
        assert freezer_open is not None
        ctx.check(
            "freezer_door_swings_forward",
            freezer_open[1][1] > freezer_closed[1][1] + 0.35,
            "The freezer door should swing out noticeably from the cabinet front.",
        )
        ctx.check(
            "freezer_door_swings_left",
            freezer_open[0][0] < freezer_closed[0][0] - 0.25,
            "The freezer door should swing about the left-side vertical hinge axis.",
        )

    with ctx.pose({fresh_food_hinge: fresh_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="fresh_hinge_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="fresh_hinge_upper_no_floating")
        ctx.expect_contact(cabinet, fresh_food_door)
        fresh_open = ctx.part_element_world_aabb(fresh_food_door, elem=fresh_panel)
        assert fresh_open is not None
        ctx.check(
            "fresh_food_door_swings_forward",
            fresh_open[1][1] > fresh_closed[1][1] + 0.35,
            "The fresh-food door should swing out noticeably from the cabinet front.",
        )
        ctx.check(
            "fresh_food_door_swings_left",
            fresh_open[0][0] < fresh_closed[0][0] - 0.25,
            "The fresh-food door should swing about the left-side vertical hinge axis.",
        )

    with ctx.pose({freezer_hinge: 1.0, fresh_food_hinge: 1.0}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="both_doors_half_open_no_overlap"
        )
        ctx.fail_if_isolated_parts(name="both_doors_half_open_no_floating")
        ctx.expect_contact(cabinet, freezer_door)
        ctx.expect_contact(cabinet, fresh_food_door)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
