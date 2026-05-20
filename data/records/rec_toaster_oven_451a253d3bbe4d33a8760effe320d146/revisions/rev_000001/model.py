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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_WIDTH = 0.58
BODY_DEPTH = 0.43
BODY_HEIGHT = 0.325
FOOT_HEIGHT = 0.015
SIDE_WALL = 0.016
TOP_WALL = 0.016
BOTTOM_WALL = 0.018
BACK_WALL = 0.014
FRONT_FRAME = 0.014

OPENING_WIDTH = 0.395
OPENING_HEIGHT = 0.195
OPENING_BOTTOM = 0.072
OPENING_CENTER_X = -0.0645

DOOR_WIDTH = 0.391
DOOR_HEIGHT = 0.205
DOOR_THICKNESS = 0.019
DOOR_AXIS_Z = 0.058
DOOR_AXIS_Y = -0.217

CONTROL_X = 0.2225
CONTROL_Y = -(BODY_DEPTH * 0.5 + 0.003)
CONTROL_ZS = (0.238, 0.170, 0.102)


def _add_x_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_y_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _build_knob(
    model: ArticulatedObject,
    *,
    part_name: str,
    parent,
    joint_name: str,
    joint_origin: tuple[float, float, float],
    material_body,
    material_marker,
) -> None:
    knob = model.part(part_name)

    _add_y_cylinder(
        knob,
        name="mount_collar",
        radius=0.014,
        length=0.004,
        xyz=(0.0, -0.002, 0.0),
        material=material_body,
    )
    _add_y_cylinder(
        knob,
        name="shaft",
        radius=0.006,
        length=0.010,
        xyz=(0.0, -0.009, 0.0),
        material=material_body,
    )
    _add_y_cylinder(
        knob,
        name="skirt",
        radius=0.031,
        length=0.004,
        xyz=(0.0, -0.016, 0.0),
        material=material_body,
    )
    _add_y_cylinder(
        knob,
        name="body",
        radius=0.029,
        length=0.020,
        xyz=(0.0, -0.028, 0.0),
        material=material_body,
    )
    knob.visual(
        Box((0.040, 0.003, 0.032)),
        origin=Origin(xyz=(0.0, -0.0395, 0.0)),
        material=material_body,
        name="datum_face",
    )
    knob.visual(
        Box((0.004, 0.002, 0.015)),
        origin=Origin(xyz=(0.0, -0.042, 0.012)),
        material=material_marker,
        name="pointer",
    )
    knob.visual(
        Box((0.006, 0.018, 0.038)),
        origin=Origin(xyz=(-0.020, -0.028, 0.0)),
        material=material_body,
        name="left_grip_rib",
    )
    knob.visual(
        Box((0.006, 0.018, 0.038)),
        origin=Origin(xyz=(0.020, -0.028, 0.0)),
        material=material_body,
        name="right_grip_rib",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.064, 0.043, 0.064)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.0215, 0.0)),
    )

    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=parent,
        child=knob,
        origin=Origin(xyz=joint_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-2.35,
            upper=2.35,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_toaster_oven")

    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.17, 0.18, 0.20, 1.0))
    graphite = model.material("graphite", rgba=(0.11, 0.12, 0.13, 1.0))
    glass = model.material("glass", rgba=(0.20, 0.26, 0.30, 0.35))
    marker = model.material("marker", rgba=(0.90, 0.92, 0.94, 1.0))
    heater = model.material("heater", rgba=(0.55, 0.27, 0.10, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")

    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BOTTOM_WALL)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_HEIGHT + BOTTOM_WALL * 0.5)),
        material=stainless,
        name="bottom_shell",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, TOP_WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - TOP_WALL * 0.5)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((SIDE_WALL, BODY_DEPTH, BODY_HEIGHT - FOOT_HEIGHT)),
        origin=Origin(
            xyz=(-BODY_WIDTH * 0.5 + SIDE_WALL * 0.5, 0.0, FOOT_HEIGHT + (BODY_HEIGHT - FOOT_HEIGHT) * 0.5)
        ),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((SIDE_WALL, BODY_DEPTH, BODY_HEIGHT - FOOT_HEIGHT)),
        origin=Origin(
            xyz=(BODY_WIDTH * 0.5 - SIDE_WALL * 0.5, 0.0, FOOT_HEIGHT + (BODY_HEIGHT - FOOT_HEIGHT) * 0.5)
        ),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((BODY_WIDTH, BACK_WALL, BODY_HEIGHT - FOOT_HEIGHT - TOP_WALL)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 - BACK_WALL * 0.5,
                FOOT_HEIGHT + (BODY_HEIGHT - FOOT_HEIGHT - TOP_WALL) * 0.5,
            )
        ),
        material=stainless,
        name="back_wall",
    )

    front_y = -BODY_DEPTH * 0.5 + FRONT_FRAME * 0.5
    left_jamb_width = 0.028
    center_mullion_width = 0.022
    control_panel_width = 0.135
    opening_left = OPENING_CENTER_X - OPENING_WIDTH * 0.5
    opening_right = OPENING_CENTER_X + OPENING_WIDTH * 0.5
    opening_center_z = OPENING_BOTTOM + OPENING_HEIGHT * 0.5

    body.visual(
        Box((BODY_WIDTH, FRONT_FRAME, OPENING_BOTTOM - FOOT_HEIGHT - 0.002)),
        origin=Origin(xyz=(0.0, front_y, FOOT_HEIGHT + (OPENING_BOTTOM - FOOT_HEIGHT - 0.002) * 0.5)),
        material=stainless,
        name="front_sill",
    )
    body.visual(
        Box((BODY_WIDTH, FRONT_FRAME, BODY_HEIGHT - (OPENING_BOTTOM + OPENING_HEIGHT))),
        origin=Origin(
            xyz=(
                0.0,
                front_y,
                OPENING_BOTTOM + OPENING_HEIGHT + (BODY_HEIGHT - (OPENING_BOTTOM + OPENING_HEIGHT)) * 0.5,
            )
        ),
        material=stainless,
        name="front_header",
    )
    body.visual(
        Box((left_jamb_width, FRONT_FRAME, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                -BODY_WIDTH * 0.5 + left_jamb_width * 0.5,
                front_y,
                opening_center_z,
            )
        ),
        material=dark_panel,
        name="left_jamb",
    )
    body.visual(
        Box((center_mullion_width, FRONT_FRAME, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                opening_right + center_mullion_width * 0.5,
                front_y,
                opening_center_z,
            )
        ),
        material=dark_panel,
        name="center_mullion",
    )
    body.visual(
        Box((control_panel_width, FRONT_FRAME, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                BODY_WIDTH * 0.5 - control_panel_width * 0.5,
                front_y,
                opening_center_z,
            )
        ),
        material=dark_panel,
        name="control_panel",
    )

    for index, knob_z in enumerate(CONTROL_ZS):
        body.visual(
            Box((0.100, 0.003, 0.060)),
            origin=Origin(xyz=(CONTROL_X, -(BODY_DEPTH * 0.5 + 0.0015), knob_z)),
            material=graphite,
            name=f"datum_pad_{index}",
        )
        body.visual(
            Box((0.004, 0.002, 0.016)),
            origin=Origin(xyz=(CONTROL_X, -(BODY_DEPTH * 0.5 + 0.004), knob_z + 0.033)),
            material=marker,
            name=f"index_mark_{index}",
        )
        body.visual(
            Box((0.004, 0.002, 0.010)),
            origin=Origin(xyz=(CONTROL_X - 0.022, -(BODY_DEPTH * 0.5 + 0.004), knob_z + 0.024)),
            material=marker,
            name=f"left_trim_mark_{index}",
        )
        body.visual(
            Box((0.004, 0.002, 0.010)),
            origin=Origin(xyz=(CONTROL_X + 0.022, -(BODY_DEPTH * 0.5 + 0.004), knob_z + 0.024)),
            material=marker,
            name=f"right_trim_mark_{index}",
        )

    inner_span_x = BODY_WIDTH - 2.0 * SIDE_WALL + 0.004
    for name, y_pos, z_pos in (
        ("upper_element_front", -0.070, 0.248),
        ("upper_element_rear", 0.070, 0.248),
        ("lower_element_front", -0.070, 0.103),
        ("lower_element_rear", 0.070, 0.103),
    ):
        _add_x_cylinder(
            body,
            name=name,
            radius=0.004,
            length=inner_span_x,
            xyz=(0.0, y_pos, z_pos),
            material=heater,
        )

    for index, x_pos in enumerate((-0.215, 0.215)):
        for side_name, y_pos in (("front", -0.150), ("rear", 0.150)):
            body.visual(
                Box((0.060, 0.060, FOOT_HEIGHT)),
                origin=Origin(xyz=(x_pos, y_pos, FOOT_HEIGHT * 0.5)),
                material=foot_rubber,
                name=f"foot_{side_name}_{index}",
            )

    body.visual(
        Box((0.058, 0.026, 0.032)),
        origin=Origin(xyz=(OPENING_CENTER_X - 0.165, -0.198, 0.056)),
        material=dark_panel,
        name="left_hinge_bracket",
    )
    body.visual(
        Box((0.058, 0.026, 0.032)),
        origin=Origin(xyz=(OPENING_CENTER_X + 0.165, -0.198, 0.056)),
        material=dark_panel,
        name="right_hinge_bracket",
    )
    body.visual(
        Box((0.026, 0.012, 0.024)),
        origin=Origin(xyz=(OPENING_CENTER_X - 0.132, -0.211, 0.068)),
        material=graphite,
        name="left_body_knuckle",
    )
    body.visual(
        Box((0.026, 0.012, 0.024)),
        origin=Origin(xyz=(OPENING_CENTER_X + 0.132, -0.211, 0.068)),
        material=graphite,
        name="right_body_knuckle",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    door = model.part("door")
    stile_width = 0.030
    rail_height_top = 0.028
    rail_height_bottom = 0.030
    half_door_width = DOOR_WIDTH * 0.5

    door.visual(
        Box((stile_width, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(-half_door_width + stile_width * 0.5, -DOOR_THICKNESS * 0.5, DOOR_HEIGHT * 0.5)),
        material=stainless,
        name="left_stile",
    )
    door.visual(
        Box((stile_width, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(half_door_width - stile_width * 0.5, -DOOR_THICKNESS * 0.5, DOOR_HEIGHT * 0.5)),
        material=stainless,
        name="right_stile",
    )
    door.visual(
        Box((DOOR_WIDTH - 2.0 * stile_width, DOOR_THICKNESS, rail_height_top)),
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS * 0.5, DOOR_HEIGHT - rail_height_top * 0.5 - 0.004)),
        material=stainless,
        name="top_rail",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, rail_height_bottom)),
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS * 0.5, rail_height_bottom * 0.5)),
        material=stainless,
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_WIDTH - 2.0 * stile_width - 0.010, 0.005, DOOR_HEIGHT - rail_height_top - rail_height_bottom - 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                -0.003,
                rail_height_bottom + (DOOR_HEIGHT - rail_height_top - rail_height_bottom) * 0.5 - 0.005,
            )
        ),
        material=glass,
        name="glass_panel",
    )
    door.visual(
        Box((0.024, 0.018, 0.022)),
        origin=Origin(xyz=(-0.132, -0.009, 0.011)),
        material=graphite,
        name="left_leaf",
    )
    door.visual(
        Box((0.024, 0.018, 0.022)),
        origin=Origin(xyz=(0.132, -0.009, 0.011)),
        material=graphite,
        name="right_leaf",
    )
    door.visual(
        Box((0.032, 0.010, 0.022)),
        origin=Origin(xyz=(-0.132, -0.004, 0.009)),
        material=graphite,
        name="left_hinge_knuckle",
    )
    door.visual(
        Box((0.032, 0.010, 0.022)),
        origin=Origin(xyz=(0.132, -0.004, 0.009)),
        material=graphite,
        name="right_hinge_knuckle",
    )
    _add_y_cylinder(
        door,
        name="handle_post_left",
        radius=0.006,
        length=0.034,
        xyz=(-0.112, -0.026, 0.152),
        material=graphite,
    )
    _add_y_cylinder(
        door,
        name="handle_post_right",
        radius=0.006,
        length=0.034,
        xyz=(0.112, -0.026, 0.152),
        material=graphite,
    )
    door.visual(
        Box((0.024, 0.012, 0.042)),
        origin=Origin(xyz=(-0.112, -0.006, 0.172)),
        material=graphite,
        name="handle_mount_left",
    )
    door.visual(
        Box((0.024, 0.012, 0.042)),
        origin=Origin(xyz=(0.112, -0.006, 0.172)),
        material=graphite,
        name="handle_mount_right",
    )
    door.visual(
        Box((0.320, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.034, 0.152)),
        material=graphite,
        name="handle_bridge",
    )
    _add_x_cylinder(
        door,
        name="handle_bar",
        radius=0.009,
        length=0.270,
        xyz=(0.0, -0.050, 0.152),
        material=graphite,
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, 0.060, DOOR_HEIGHT)),
        mass=2.1,
        origin=Origin(xyz=(0.0, -0.024, DOOR_HEIGHT * 0.5)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(OPENING_CENTER_X, DOOR_AXIS_Y, DOOR_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    _build_knob(
        model,
        part_name="temp_knob",
        parent=body,
        joint_name="body_to_temp_knob",
        joint_origin=(CONTROL_X, CONTROL_Y, CONTROL_ZS[0]),
        material_body=graphite,
        material_marker=marker,
    )
    _build_knob(
        model,
        part_name="mode_knob",
        parent=body,
        joint_name="body_to_mode_knob",
        joint_origin=(CONTROL_X, CONTROL_Y, CONTROL_ZS[1]),
        material_body=graphite,
        material_marker=marker,
    )
    _build_knob(
        model,
        part_name="timer_knob",
        parent=body,
        joint_name="body_to_timer_knob",
        joint_origin=(CONTROL_X, CONTROL_Y, CONTROL_ZS[2]),
        material_body=graphite,
        material_marker=marker,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    temp_knob = object_model.get_part("temp_knob")
    mode_knob = object_model.get_part("mode_knob")
    timer_knob = object_model.get_part("timer_knob")
    door_hinge = object_model.get_articulation("body_to_door")
    temp_joint = object_model.get_articulation("body_to_temp_knob")
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

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_contact(
            door,
            body,
            elem_a="left_hinge_knuckle",
            elem_b="left_body_knuckle",
            name="left_hinge_knuckle_contacts_body",
        )
        ctx.expect_contact(
            door,
            body,
            elem_a="right_hinge_knuckle",
            elem_b="right_body_knuckle",
            name="right_hinge_knuckle_contacts_body",
        )
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="left_jamb",
            negative_elem="left_stile",
            min_gap=0.0,
            max_gap=0.003,
            name="door_left_controlled_front_gap",
        )
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="center_mullion",
            negative_elem="right_stile",
            min_gap=0.0,
            max_gap=0.003,
            name="door_right_controlled_front_gap",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.18,
            name="door_covers_oven_front",
        )
        ctx.expect_contact(
            temp_knob,
            body,
            elem_b="datum_pad_0",
            name="temp_knob_seats_on_datum_pad",
        )
        ctx.expect_contact(
            mode_knob,
            body,
            elem_b="datum_pad_1",
            name="mode_knob_seats_on_datum_pad",
        )
        ctx.expect_contact(
            timer_knob,
            body,
            elem_b="datum_pad_2",
            name="timer_knob_seats_on_datum_pad",
        )
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="front_sill",
            negative_elem="bottom_rail",
            min_gap=0.001,
            max_gap=0.004,
            name="door_bottom_controlled_front_gap",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(low, high))

    closed_handle = _aabb_center(ctx.part_element_world_aabb(door, elem="handle_bar"))
    with ctx.pose({door_hinge: 1.15}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_no_overlaps")
        open_handle = _aabb_center(ctx.part_element_world_aabb(door, elem="handle_bar"))
    if closed_handle is None or open_handle is None:
        ctx.fail("door_handle_pose_measurement", "Could not measure door handle in one or more poses.")
    else:
        ctx.check(
            "door_opens_down_and_forward",
            open_handle[1] < closed_handle[1] - 0.10 and open_handle[2] < closed_handle[2] - 0.05,
            (
                f"Closed handle center={closed_handle}, "
                f"open handle center={open_handle}; expected more negative Y and lower Z in the open pose."
            ),
        )

    closed_pointer = _aabb_center(ctx.part_element_world_aabb(temp_knob, elem="pointer"))
    with ctx.pose({temp_joint: 1.2}):
        turned_pointer = _aabb_center(ctx.part_element_world_aabb(temp_knob, elem="pointer"))
    if closed_pointer is None or turned_pointer is None:
        ctx.fail("temp_knob_pointer_measurement", "Could not measure temp knob pointer.")
    else:
        ctx.check(
            "temp_knob_rotates_clockwise_from_front",
            turned_pointer[0] > closed_pointer[0] + 0.008
            and turned_pointer[2] < closed_pointer[2] - 0.006
            and abs(turned_pointer[1] - closed_pointer[1]) < 0.002,
            (
                f"Closed pointer center={closed_pointer}, turned pointer center={turned_pointer}; "
                "expected clockwise rotation about the front-to-back shaft axis."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
