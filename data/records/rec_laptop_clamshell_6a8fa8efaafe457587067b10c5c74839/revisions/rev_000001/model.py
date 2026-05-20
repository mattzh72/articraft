from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_WIDTH = 0.362
BASE_DEPTH = 0.282
BASE_HEIGHT = 0.054
BASE_WALL = 0.012
BOTTOM_THICKNESS = 0.006

LID_WIDTH = 0.356
LID_DEPTH = 0.274
LID_THICKNESS = 0.028
HINGE_Y = 0.133
HINGE_Z = BASE_HEIGHT

KEY_ROWS = 3
KEY_COLS = 5
KEY_WIDTH = 0.032
KEY_DEPTH = 0.020
KEY_HEIGHT = 0.007
KEY_TRAVEL = 0.0015
KEY_PITCH_X = 0.038
KEY_PITCH_Y = 0.028
KEY_CENTER_Z = 0.0425

LATCH_BARREL_RADIUS = 0.004
LATCH_BARREL_LENGTH = 0.028
LATCH_PIVOT_Y = -0.144
LATCH_PIVOT_Z = 0.020
LATCH_XS = (-0.126, 0.126)


def add_box_visual(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    material,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def add_cylinder_x_visual(
    part,
    *,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material,
    name: str | None = None,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def key_specs() -> list[dict[str, float | str]]:
    start_x = -0.5 * (KEY_COLS - 1) * KEY_PITCH_X
    start_y = 0.5 * (KEY_ROWS - 1) * KEY_PITCH_Y + 0.014
    specs: list[dict[str, float | str]] = []
    for row in range(KEY_ROWS):
        for col in range(KEY_COLS):
            specs.append(
                {
                    "name": f"key_r{row + 1}c{col + 1}",
                    "x": start_x + col * KEY_PITCH_X,
                    "y": start_y - row * KEY_PITCH_Y,
                }
            )
    return specs


def build_latch_hook(
    model: ArticulatedObject,
    *,
    name: str,
    x: float,
    hook_material,
    metal_material,
    body_part,
):
    hook = model.part(name=name)
    add_cylinder_x_visual(
        hook,
        radius=LATCH_BARREL_RADIUS,
        length=LATCH_BARREL_LENGTH,
        center=(0.0, 0.0, 0.0),
        material=metal_material,
        name="barrel",
    )
    add_box_visual(
        hook,
        (LATCH_BARREL_LENGTH, 0.010, 0.030),
        (0.0, 0.000, 0.015),
        material=hook_material,
        name="arm",
    )
    add_box_visual(
        hook,
        (LATCH_BARREL_LENGTH, 0.018, 0.006),
        (0.0, -0.006, 0.031),
        material=hook_material,
        name="nose",
    )
    add_box_visual(
        hook,
        (LATCH_BARREL_LENGTH, 0.006, 0.008),
        (0.0, -0.004, 0.027),
        material=hook_material,
        name="keeper",
    )
    hook.inertial = Inertial.from_geometry(
        Box((LATCH_BARREL_LENGTH, 0.018, 0.037)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.002, 0.0185)),
    )

    model.articulation(
        f"{name}_pivot",
        ArticulationType.REVOLUTE,
        parent=body_part,
        child=hook,
        origin=Origin(xyz=(x, LATCH_PIVOT_Y, LATCH_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=-0.12,
            upper=1.20,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_field_laptop")

    body_material = model.material("body_magnesium", rgba=(0.23, 0.26, 0.29, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.13, 0.14, 0.15, 1.0))
    bumper_material = model.material("bumper_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    key_material = model.material("keycap_black", rgba=(0.09, 0.10, 0.11, 1.0))
    key_legend_material = model.material("keycap_charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    metal_material = model.material("hinge_metal", rgba=(0.58, 0.61, 0.64, 1.0))
    screen_material = model.material("screen_glass", rgba=(0.06, 0.09, 0.11, 1.0))
    touchpad_material = model.material("touchpad", rgba=(0.18, 0.19, 0.20, 1.0))

    lower_body = model.part("lower_body")
    add_box_visual(
        lower_body,
        (BASE_WIDTH, BASE_DEPTH, BOTTOM_THICKNESS),
        (0.0, 0.0, BOTTOM_THICKNESS / 2.0),
        material=body_material,
        name="bottom_pan",
    )
    wall_height = BASE_HEIGHT - BOTTOM_THICKNESS
    wall_center_z = BOTTOM_THICKNESS + wall_height / 2.0
    add_box_visual(
        lower_body,
        (BASE_WALL, BASE_DEPTH, wall_height),
        (-BASE_WIDTH / 2.0 + BASE_WALL / 2.0, 0.0, wall_center_z),
        material=body_material,
        name="left_wall",
    )
    add_box_visual(
        lower_body,
        (BASE_WALL, BASE_DEPTH, wall_height),
        (BASE_WIDTH / 2.0 - BASE_WALL / 2.0, 0.0, wall_center_z),
        material=body_material,
        name="right_wall",
    )
    add_box_visual(
        lower_body,
        (BASE_WIDTH - 2.0 * BASE_WALL, BASE_WALL, wall_height),
        (0.0, -BASE_DEPTH / 2.0 + BASE_WALL / 2.0, wall_center_z),
        material=body_material,
        name="front_wall",
    )
    add_box_visual(
        lower_body,
        (BASE_WIDTH - 2.0 * BASE_WALL, 0.014, wall_height),
        (0.0, BASE_DEPTH / 2.0 - 0.007, wall_center_z),
        material=body_material,
        name="rear_wall",
    )
    add_box_visual(
        lower_body,
        (0.336, 0.104, 0.004),
        (0.0, -0.068, 0.037),
        material=dark_panel,
        name="palmrest_deck",
    )
    add_box_visual(
        lower_body,
        (0.248, 0.096, 0.003),
        (0.0, 0.014, 0.035),
        material=dark_panel,
        name="keyboard_tray",
    )
    add_box_visual(
        lower_body,
        (0.088, 0.054, 0.003),
        (0.0, -0.076, 0.0405),
        material=touchpad_material,
        name="touchpad",
    )
    add_box_visual(
        lower_body,
        (0.100, 0.022, 0.004),
        (0.0, 0.073, 0.036),
        material=dark_panel,
        name="function_bar",
    )

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            add_box_visual(
                lower_body,
                (0.024, 0.024, 0.018),
                (
                    sx * (BASE_WIDTH / 2.0 - 0.013),
                    sy * (BASE_DEPTH / 2.0 - 0.013),
                    0.045,
                ),
                material=bumper_material,
                name=f"base_bumper_{'l' if sx < 0 else 'r'}_{'f' if sy < 0 else 'r'}",
            )

    for index, latch_x in enumerate(LATCH_XS):
        add_box_visual(
            lower_body,
            (0.004, 0.012, 0.014),
            (latch_x - 0.016, -0.144, 0.020),
            material=metal_material,
            name=f"latch_ear_{index + 1}_outer",
        )
        add_box_visual(
            lower_body,
            (0.004, 0.012, 0.014),
            (latch_x + 0.016, -0.144, 0.020),
            material=metal_material,
            name=f"latch_ear_{index + 1}_inner",
        )
        add_box_visual(
            lower_body,
            (0.040, 0.010, 0.008),
            (latch_x, -0.137, 0.013),
            material=metal_material,
            name=f"latch_base_{index + 1}",
        )

    for hinge_index, hinge_x in enumerate((-0.105, 0.105)):
        add_box_visual(
            lower_body,
            (0.056, 0.020, 0.010),
            (hinge_x, 0.133, 0.045),
            material=metal_material,
            name=f"hinge_pedestal_{hinge_index + 1}",
        )
        add_cylinder_x_visual(
            lower_body,
            radius=0.006,
            length=0.050,
            center=(hinge_x, 0.143, 0.056),
            material=metal_material,
            name=f"hinge_barrel_{hinge_index + 1}",
        )

    well_wall_thickness = 0.0015
    well_height = 0.010
    well_center_z = 0.0415
    for spec in key_specs():
        key_x = float(spec["x"])
        key_y = float(spec["y"])
        add_box_visual(
            lower_body,
            (KEY_WIDTH + 2.0 * well_wall_thickness, well_wall_thickness, well_height),
            (key_x, key_y - KEY_DEPTH / 2.0 - well_wall_thickness / 2.0, well_center_z),
            material=dark_panel,
        )
        add_box_visual(
            lower_body,
            (KEY_WIDTH + 2.0 * well_wall_thickness, well_wall_thickness, well_height),
            (key_x, key_y + KEY_DEPTH / 2.0 + well_wall_thickness / 2.0, well_center_z),
            material=dark_panel,
        )
        add_box_visual(
            lower_body,
            (well_wall_thickness, KEY_DEPTH, well_height),
            (key_x - KEY_WIDTH / 2.0 - well_wall_thickness / 2.0, key_y, well_center_z),
            material=dark_panel,
        )
        add_box_visual(
            lower_body,
            (well_wall_thickness, KEY_DEPTH, well_height),
            (key_x + KEY_WIDTH / 2.0 + well_wall_thickness / 2.0, key_y, well_center_z),
            material=dark_panel,
        )

    lower_body.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    lid = model.part("display_lid")
    add_box_visual(
        lid,
        (LID_WIDTH, LID_DEPTH, 0.004),
        (0.0, -LID_DEPTH / 2.0, LID_THICKNESS - 0.002),
        material=body_material,
        name="top_skin",
    )
    add_box_visual(
        lid,
        (0.012, LID_DEPTH, LID_THICKNESS),
        (-LID_WIDTH / 2.0 + 0.006, -LID_DEPTH / 2.0, LID_THICKNESS / 2.0),
        material=body_material,
        name="left_rail",
    )
    add_box_visual(
        lid,
        (0.012, LID_DEPTH, LID_THICKNESS),
        (LID_WIDTH / 2.0 - 0.006, -LID_DEPTH / 2.0, LID_THICKNESS / 2.0),
        material=body_material,
        name="right_rail",
    )
    add_box_visual(
        lid,
        (LID_WIDTH - 0.020, 0.014, LID_THICKNESS),
        (0.0, -LID_DEPTH + 0.007, LID_THICKNESS / 2.0),
        material=body_material,
        name="front_beam",
    )
    add_box_visual(
        lid,
        (0.010, 0.210, 0.012),
        (-0.151, -0.150, 0.006),
        material=dark_panel,
        name="bezel_left",
    )
    add_box_visual(
        lid,
        (0.010, 0.210, 0.012),
        (0.151, -0.150, 0.006),
        material=dark_panel,
        name="bezel_right",
    )
    add_box_visual(
        lid,
        (0.312, 0.012, 0.012),
        (0.0, -0.052, 0.006),
        material=dark_panel,
        name="bezel_top",
    )
    add_box_visual(
        lid,
        (0.312, 0.012, 0.012),
        (0.0, -0.248, 0.006),
        material=dark_panel,
        name="bezel_bottom",
    )
    add_box_visual(
        lid,
        (0.292, 0.184, 0.004),
        (0.0, -0.150, 0.006),
        material=screen_material,
        name="screen",
    )
    add_box_visual(
        lid,
        (0.332, 0.228, 0.012),
        (0.0, -0.150, 0.018),
        material=dark_panel,
        name="inner_panel",
    )
    add_box_visual(
        lid,
        (0.050, 0.018, 0.016),
        (-0.105, -0.009, 0.020),
        material=metal_material,
        name="left_hinge_leaf",
    )
    add_box_visual(
        lid,
        (0.050, 0.018, 0.016),
        (0.105, -0.009, 0.020),
        material=metal_material,
        name="right_hinge_leaf",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            add_box_visual(
                lid,
                (0.022, 0.022, 0.010),
                (
                    sx * (LID_WIDTH / 2.0 - 0.012),
                    -LID_DEPTH / 2.0 + sy * (LID_DEPTH / 2.0 - 0.012),
                    0.023,
                ),
                material=bumper_material,
                name=f"lid_bumper_{'l' if sx < 0 else 'r'}_{'t' if sy > 0 else 'b'}",
            )

    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_DEPTH, LID_THICKNESS)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -LID_DEPTH / 2.0, LID_THICKNESS / 2.0)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=-1.95,
            upper=0.08,
        ),
    )

    for latch_name, latch_x in zip(("left_front_hook", "right_front_hook"), LATCH_XS):
        build_latch_hook(
            model,
            name=latch_name,
            x=latch_x,
            hook_material=bumper_material,
            metal_material=metal_material,
            body_part=lower_body,
        )

    for spec in key_specs():
        key_name = str(spec["name"])
        key = model.part(key_name)
        add_box_visual(
            key,
            (KEY_WIDTH, KEY_DEPTH, KEY_HEIGHT),
            (0.0, 0.0, 0.0),
            material=key_material,
            name="cap",
        )
        add_box_visual(
            key,
            (KEY_WIDTH - 0.006, KEY_DEPTH - 0.006, 0.0012),
            (0.0, 0.0, KEY_HEIGHT / 2.0 + 0.0006),
            material=key_legend_material,
            name="top_face",
        )
        key.inertial = Inertial.from_geometry(
            Box((KEY_WIDTH, KEY_DEPTH, KEY_HEIGHT)),
            mass=0.018,
            origin=Origin(),
        )
        model.articulation(
            f"{key_name}_travel",
            ArticulationType.PRISMATIC,
            parent=lower_body,
            child=key,
            origin=Origin(xyz=(float(spec["x"]), float(spec["y"]), KEY_CENTER_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.4,
                lower=-KEY_TRAVEL,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    display_lid = object_model.get_part("display_lid")
    lid_hinge = object_model.get_articulation("lid_hinge")
    left_front_hook = object_model.get_part("left_front_hook")
    right_front_hook = object_model.get_part("right_front_hook")
    left_hook_pivot = object_model.get_articulation("left_front_hook_pivot")
    right_hook_pivot = object_model.get_articulation("right_front_hook_pivot")

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
        "lid_hinge_axis_is_x",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"lid hinge axis was {lid_hinge.axis}",
    )
    ctx.check(
        "left_hook_axis_is_x",
        tuple(left_hook_pivot.axis) == (1.0, 0.0, 0.0),
        details=f"left hook axis was {left_hook_pivot.axis}",
    )
    ctx.check(
        "right_hook_axis_is_x",
        tuple(right_hook_pivot.axis) == (1.0, 0.0, 0.0),
        details=f"right hook axis was {right_hook_pivot.axis}",
    )

    ctx.expect_contact(
        left_front_hook,
        lower_body,
        elem_a="barrel",
        name="left_hook_clipped_to_lower_body",
    )
    ctx.expect_contact(
        right_front_hook,
        lower_body,
        elem_a="barrel",
        name="right_hook_clipped_to_lower_body",
    )
    ctx.expect_contact(
        left_front_hook,
        display_lid,
        elem_a="nose",
        elem_b="front_beam",
        name="left_hook_secures_lid_closed",
    )
    ctx.expect_contact(
        right_front_hook,
        display_lid,
        elem_a="nose",
        elem_b="front_beam",
        name="right_hook_secures_lid_closed",
    )
    ctx.expect_gap(
        display_lid,
        lower_body,
        axis="z",
        positive_elem="front_beam",
        negative_elem="front_wall",
        max_gap=0.001,
        max_penetration=0.0,
        name="lid_front_beam_seats_on_front_edge",
    )

    key_parts = []
    for spec in key_specs():
        key_name = str(spec["name"])
        key_parts.append(object_model.get_part(key_name))
        object_model.get_articulation(f"{key_name}_travel")

    for key in key_parts:
        ctx.expect_contact(key, lower_body, name=f"{key.name}_guided")
        ctx.expect_within(
            key,
            lower_body,
            axes="xy",
            margin=0.0,
            name=f"{key.name}_within_body_plan",
        )

    center_key = object_model.get_part("key_r2c3")
    center_key_joint = object_model.get_articulation("key_r2c3_travel")
    rest_center_key_pos = ctx.part_world_position(center_key)
    with ctx.pose({center_key_joint: -KEY_TRAVEL}):
        pressed_center_key_pos = ctx.part_world_position(center_key)
    ctx.check(
        "center_key_moves_down",
        rest_center_key_pos is not None
        and pressed_center_key_pos is not None
        and pressed_center_key_pos[2] < rest_center_key_pos[2] - 0.001,
        details=f"rest={rest_center_key_pos}, pressed={pressed_center_key_pos}",
    )

    front_left_key = object_model.get_part("key_r1c1")
    front_left_joint = object_model.get_articulation("key_r1c1_travel")
    rest_front_left_key_pos = ctx.part_world_position(front_left_key)
    with ctx.pose({front_left_joint: -KEY_TRAVEL}):
        pressed_front_left_key_pos = ctx.part_world_position(front_left_key)
    ctx.check(
        "front_left_key_moves_down",
        rest_front_left_key_pos is not None
        and pressed_front_left_key_pos is not None
        and pressed_front_left_key_pos[2] < rest_front_left_key_pos[2] - 0.001,
        details=f"rest={rest_front_left_key_pos}, pressed={pressed_front_left_key_pos}",
    )

    closed_left_hook_nose = ctx.part_element_world_aabb(left_front_hook, elem="nose")
    closed_right_hook_nose = ctx.part_element_world_aabb(right_front_hook, elem="nose")
    with ctx.pose({left_hook_pivot: 1.10, right_hook_pivot: 1.10}):
        ctx.expect_contact(
            left_front_hook,
            lower_body,
            elem_a="barrel",
            name="left_hook_remains_clipped_when_open",
        )
        ctx.expect_contact(
            right_front_hook,
            lower_body,
            elem_a="barrel",
            name="right_hook_remains_clipped_when_open",
        )
        open_left_hook_nose = ctx.part_element_world_aabb(left_front_hook, elem="nose")
        open_right_hook_nose = ctx.part_element_world_aabb(right_front_hook, elem="nose")
    ctx.check(
        "left_hook_swings_open",
        closed_left_hook_nose is not None
        and open_left_hook_nose is not None
        and open_left_hook_nose[0][1] < closed_left_hook_nose[0][1] - 0.015
        and open_left_hook_nose[1][2] < closed_left_hook_nose[1][2] - 0.006,
        details=f"closed={closed_left_hook_nose}, open={open_left_hook_nose}",
    )
    ctx.check(
        "right_hook_swings_open",
        open_right_hook_nose is not None
        and closed_right_hook_nose is not None
        and open_right_hook_nose[0][1] < closed_right_hook_nose[0][1] - 0.015
        and open_right_hook_nose[1][2] < closed_right_hook_nose[1][2] - 0.006,
        details=f"closed={closed_right_hook_nose}, open={open_right_hook_nose}",
    )

    with ctx.pose({lid_hinge: -1.20}):
        ctx.expect_gap(
            display_lid,
            lower_body,
            axis="z",
            positive_elem="front_beam",
            negative_elem="rear_wall",
            min_gap=0.180,
            name="lid_front_clears_body_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
