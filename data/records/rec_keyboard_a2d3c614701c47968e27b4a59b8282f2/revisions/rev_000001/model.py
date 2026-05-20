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


UNIT = 0.018
MAIN_LEFT = -0.198
NAV_LEFT = 0.084
NUMPAD_LEFT = 0.148
KEY_TRAVEL = 0.004
KEY_PART_NAMES: list[str] = []


def _append_row(
    placements: list[dict[str, float | str]],
    *,
    start_x: float,
    y: float,
    z: float,
    items: list[tuple[str | None, float]],
    style: str = "standard",
) -> None:
    cursor = start_x
    for name, width_units in items:
        if name is None:
            cursor += width_units * UNIT
            continue
        placements.append(
            {
                "name": name,
                "x": cursor + 0.5 * width_units * UNIT,
                "y": y,
                "z": z,
                "w_units": width_units,
                "d_units": 1.0,
                "style": style,
            }
        )
        cursor += width_units * UNIT


def _append_key(
    placements: list[dict[str, float | str]],
    *,
    name: str,
    start_x: float,
    column: float,
    y: float,
    z: float,
    width_units: float = 1.0,
    depth_units: float = 1.0,
    style: str = "standard",
) -> None:
    placements.append(
        {
            "name": name,
            "x": start_x + (column + 0.5 * width_units) * UNIT,
            "y": y,
            "z": z,
            "w_units": width_units,
            "d_units": depth_units,
            "style": style,
        }
    )


def _key_sizes(width_units: float, depth_units: float, style: str) -> tuple[float, float, float, float]:
    side_gap = 0.0022 if style == "function" else 0.0018
    front_gap = 0.0024 if style == "function" else 0.0020
    stem_height = 0.0040 if style == "function" else 0.0044
    cap_height = 0.0036 if style == "function" else 0.0042
    width = width_units * UNIT - side_gap
    depth = depth_units * UNIT - front_gap
    return width, depth, stem_height, cap_height


def _add_key(
    model: ArticulatedObject,
    housing,
    *,
    key_material,
    name: str,
    x: float,
    y: float,
    z: float,
    width_units: float,
    depth_units: float,
    style: str,
) -> None:
    width, depth, stem_height, cap_height = _key_sizes(width_units, depth_units, style)
    stem_width = max(0.006, width - 0.0034)
    stem_depth = max(0.006, depth - 0.0034)

    key = model.part(name)
    key.visual(
        Box((stem_width, stem_depth, stem_height)),
        origin=Origin(xyz=(0.0, 0.0, stem_height * 0.5)),
        material=key_material,
        name="plunger",
    )
    key.visual(
        Box((width, depth, cap_height)),
        origin=Origin(xyz=(0.0, 0.0, stem_height + cap_height * 0.5 - 0.00015)),
        material=key_material,
        name="cap",
    )
    total_height = stem_height + cap_height
    key.inertial = Inertial.from_geometry(
        Box((width, depth, total_height)),
        mass=0.006 + 0.003 * width_units * depth_units,
        origin=Origin(xyz=(0.0, 0.0, total_height * 0.5)),
    )

    model.articulation(
        f"housing_to_{name}",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=key,
        origin=Origin(xyz=(x, y, z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0034 if style == "function" else KEY_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_keyboard")
    KEY_PART_NAMES.clear()

    housing_dark = model.material("housing_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    frame_dark = model.material("frame_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    key_dark = model.material("key_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    key_secondary = model.material("key_secondary", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.445, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.063, 0.007)),
        material=housing_dark,
        name="front_bezel",
    )
    housing.visual(
        Box((0.445, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.063, 0.016)),
        material=housing_dark,
        name="rear_wall",
    )
    housing.visual(
        Box((0.008, 0.145, 0.024)),
        origin=Origin(xyz=(-0.2185, 0.0, 0.012)),
        material=housing_dark,
        name="left_wall",
    )
    housing.visual(
        Box((0.008, 0.145, 0.024)),
        origin=Origin(xyz=(0.2185, 0.0, 0.012)),
        material=housing_dark,
        name="right_wall",
    )
    housing.visual(
        Box((0.429, 0.080, 0.008)),
        origin=Origin(xyz=(0.0, -0.026, 0.004)),
        material=housing_dark,
        name="bottom_front",
    )
    housing.visual(
        Box((0.403, 0.080, 0.003)),
        origin=Origin(xyz=(0.0, -0.019, 0.0085)),
        material=frame_dark,
        name="switch_floor",
    )
    housing.visual(
        Box((0.429, 0.010, 0.005)),
        origin=Origin(xyz=(0.0, -0.056, 0.0155)),
        material=frame_dark,
        name="front_top_strip",
    )
    housing.visual(
        Box((0.429, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.058, 0.028)),
        material=frame_dark,
        name="rear_top_strip",
    )
    housing.visual(
        Box((0.010, 0.112, 0.018)),
        origin=Origin(xyz=(-0.206, 0.0, 0.019)),
        material=frame_dark,
        name="left_top_rail",
    )
    housing.visual(
        Box((0.010, 0.112, 0.018)),
        origin=Origin(xyz=(0.206, 0.0, 0.019)),
        material=frame_dark,
        name="right_top_rail",
    )
    housing.visual(
        Box((0.010, 0.092, 0.016)),
        origin=Origin(xyz=(0.078, -0.006, 0.018)),
        material=frame_dark,
        name="main_nav_divider",
    )
    housing.visual(
        Box((0.010, 0.092, 0.016)),
        origin=Origin(xyz=(0.143, -0.006, 0.018)),
        material=frame_dark,
        name="nav_numpad_divider",
    )
    housing.visual(
        Box((0.265, 0.020, 0.004)),
        origin=Origin(xyz=(-0.066, 0.049, 0.026)),
        material=frame_dark,
        name="function_row_bridge",
    )
    housing.visual(
        Box((0.040, 0.010, 0.002)),
        origin=Origin(xyz=(-0.150, -0.055, 0.001)),
        material=rubber,
        name="front_pad_left",
    )
    housing.visual(
        Box((0.040, 0.010, 0.002)),
        origin=Origin(xyz=(0.150, -0.055, 0.001)),
        material=rubber,
        name="front_pad_right",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.445, 0.145, 0.032)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    placements: list[dict[str, float | str]] = []
    _append_row(
        placements,
        start_x=MAIN_LEFT,
        y=0.048,
        z=0.029,
        style="function",
        items=[
            ("key_escape", 1.0),
            (None, 0.5),
            ("key_f1", 1.0),
            ("key_f2", 1.0),
            ("key_f3", 1.0),
            ("key_f4", 1.0),
            (None, 0.5),
            ("key_f5", 1.0),
            ("key_f6", 1.0),
            ("key_f7", 1.0),
            ("key_f8", 1.0),
            (None, 0.5),
            ("key_f9", 1.0),
            ("key_f10", 1.0),
            ("key_f11", 1.0),
            ("key_f12", 1.0),
            (None, 0.5),
            ("key_print_screen", 1.0),
            ("key_scroll_lock", 1.0),
            ("key_pause", 1.0),
        ],
    )
    _append_row(
        placements,
        start_x=MAIN_LEFT,
        y=0.028,
        z=0.027,
        items=[
            ("key_grave", 1.0),
            ("key_digit_1", 1.0),
            ("key_digit_2", 1.0),
            ("key_digit_3", 1.0),
            ("key_digit_4", 1.0),
            ("key_digit_5", 1.0),
            ("key_digit_6", 1.0),
            ("key_digit_7", 1.0),
            ("key_digit_8", 1.0),
            ("key_digit_9", 1.0),
            ("key_digit_0", 1.0),
            ("key_minus", 1.0),
            ("key_equals", 1.0),
            ("key_backspace", 2.0),
        ],
    )
    _append_row(
        placements,
        start_x=MAIN_LEFT,
        y=0.010,
        z=0.025,
        items=[
            ("key_tab", 1.5),
            ("key_q", 1.0),
            ("key_w", 1.0),
            ("key_e", 1.0),
            ("key_r", 1.0),
            ("key_t", 1.0),
            ("key_y", 1.0),
            ("key_u", 1.0),
            ("key_i", 1.0),
            ("key_o", 1.0),
            ("key_p", 1.0),
            ("key_left_bracket", 1.0),
            ("key_right_bracket", 1.0),
            ("key_backslash", 1.5),
        ],
    )
    _append_row(
        placements,
        start_x=MAIN_LEFT,
        y=-0.008,
        z=0.023,
        items=[
            ("key_caps_lock", 1.75),
            ("key_a", 1.0),
            ("key_s", 1.0),
            ("key_d", 1.0),
            ("key_f", 1.0),
            ("key_g", 1.0),
            ("key_h", 1.0),
            ("key_j", 1.0),
            ("key_k", 1.0),
            ("key_l", 1.0),
            ("key_semicolon", 1.0),
            ("key_quote", 1.0),
            ("key_enter", 2.25),
        ],
    )
    _append_row(
        placements,
        start_x=MAIN_LEFT,
        y=-0.026,
        z=0.021,
        items=[
            ("key_left_shift", 2.25),
            ("key_z", 1.0),
            ("key_x", 1.0),
            ("key_c", 1.0),
            ("key_v", 1.0),
            ("key_b", 1.0),
            ("key_n", 1.0),
            ("key_m", 1.0),
            ("key_comma", 1.0),
            ("key_period", 1.0),
            ("key_slash", 1.0),
            ("key_right_shift", 2.75),
        ],
    )
    _append_row(
        placements,
        start_x=MAIN_LEFT,
        y=-0.044,
        z=0.019,
        items=[
            ("key_left_ctrl", 1.25),
            ("key_left_meta", 1.25),
            ("key_left_alt", 1.25),
            ("key_space", 6.25),
            ("key_right_alt", 1.25),
            ("key_fn", 1.0),
            ("key_menu", 1.0),
            ("key_right_ctrl", 1.75),
        ],
    )

    _append_row(
        placements,
        start_x=NAV_LEFT,
        y=0.028,
        z=0.027,
        items=[("key_insert", 1.0), ("key_home", 1.0), ("key_page_up", 1.0)],
        style="function",
    )
    _append_row(
        placements,
        start_x=NAV_LEFT,
        y=0.010,
        z=0.025,
        items=[("key_delete", 1.0), ("key_end", 1.0), ("key_page_down", 1.0)],
        style="function",
    )
    _append_row(
        placements,
        start_x=NAV_LEFT,
        y=-0.026,
        z=0.021,
        items=[(None, 1.0), ("key_arrow_up", 1.0), (None, 1.0)],
        style="function",
    )
    _append_row(
        placements,
        start_x=NAV_LEFT,
        y=-0.044,
        z=0.019,
        items=[("key_arrow_left", 1.0), ("key_arrow_down", 1.0), ("key_arrow_right", 1.0)],
        style="function",
    )

    _append_row(
        placements,
        start_x=NUMPAD_LEFT,
        y=0.028,
        z=0.027,
        items=[
            ("key_numpad_num_lock", 1.0),
            ("key_numpad_divide", 1.0),
            ("key_numpad_multiply", 1.0),
            ("key_numpad_subtract", 1.0),
        ],
        style="function",
    )
    _append_row(
        placements,
        start_x=NUMPAD_LEFT,
        y=0.010,
        z=0.025,
        items=[("key_numpad_7", 1.0), ("key_numpad_8", 1.0), ("key_numpad_9", 1.0)],
    )
    _append_row(
        placements,
        start_x=NUMPAD_LEFT,
        y=-0.008,
        z=0.023,
        items=[("key_numpad_4", 1.0), ("key_numpad_5", 1.0), ("key_numpad_6", 1.0)],
    )
    _append_row(
        placements,
        start_x=NUMPAD_LEFT,
        y=-0.026,
        z=0.021,
        items=[("key_numpad_1", 1.0), ("key_numpad_2", 1.0), ("key_numpad_3", 1.0)],
    )
    _append_key(
        placements,
        name="key_numpad_add",
        start_x=NUMPAD_LEFT,
        column=3.0,
        y=0.001,
        z=0.024,
        depth_units=2.0,
    )
    _append_key(
        placements,
        name="key_numpad_enter",
        start_x=NUMPAD_LEFT,
        column=3.0,
        y=-0.035,
        z=0.020,
        depth_units=2.0,
    )
    _append_key(
        placements,
        name="key_numpad_0",
        start_x=NUMPAD_LEFT,
        column=0.0,
        y=-0.044,
        z=0.019,
        width_units=2.0,
    )
    _append_key(
        placements,
        name="key_numpad_decimal",
        start_x=NUMPAD_LEFT,
        column=2.0,
        y=-0.044,
        z=0.019,
    )

    secondary_keys = {
        "key_escape",
        "key_print_screen",
        "key_scroll_lock",
        "key_pause",
        "key_insert",
        "key_home",
        "key_page_up",
        "key_delete",
        "key_end",
        "key_page_down",
        "key_arrow_up",
        "key_arrow_left",
        "key_arrow_down",
        "key_arrow_right",
        "key_numpad_num_lock",
        "key_numpad_divide",
        "key_numpad_multiply",
        "key_numpad_subtract",
    }

    for placement in placements:
        name = str(placement["name"])
        KEY_PART_NAMES.append(name)
        _add_key(
            model,
            housing,
            key_material=key_secondary if name in secondary_keys or name.startswith("key_f") else key_dark,
            name=name,
            x=float(placement["x"]),
            y=float(placement["y"]),
            z=float(placement["z"]),
            width_units=float(placement["w_units"]),
            depth_units=float(placement["d_units"]),
            style=str(placement["style"]),
        )

    foot_specs = (
        ("left_rear_foot", -0.135),
        ("right_rear_foot", 0.135),
    )
    for foot_name, x in foot_specs:
        foot = model.part(foot_name)
        foot.visual(
            Cylinder(radius=0.0025, length=0.056),
            origin=Origin(xyz=(0.0, 0.0, -0.0015), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=frame_dark,
            name="hinge_barrel",
        )
        foot.visual(
            Box((0.052, 0.022, 0.004)),
            origin=Origin(xyz=(0.0, 0.011, -0.004)),
            material=housing_dark,
            name="foot_blade",
        )
        foot.visual(
            Box((0.044, 0.006, 0.0015)),
            origin=Origin(xyz=(0.0, 0.019, -0.00675)),
            material=rubber,
            name="foot_pad",
        )
        foot.inertial = Inertial.from_geometry(
            Box((0.056, 0.022, 0.008)),
            mass=0.025,
            origin=Origin(xyz=(0.0, 0.010, -0.003)),
        )
        model.articulation(
            f"housing_to_{foot_name}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=foot,
            origin=Origin(xyz=(x, 0.040, 0.008)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=3.0,
                lower=0.0,
                upper=1.08,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    for key_name in KEY_PART_NAMES:
        ctx.allow_isolated_part(
            key_name,
            reason="Each keycap is intentionally modeled with running clearance while its prismatic joint represents the enclosed switch guide.",
        )

    housing = object_model.get_part("housing")
    key_a = object_model.get_part("key_a")
    key_space = object_model.get_part("key_space")
    numpad_zero = object_model.get_part("key_numpad_0")
    left_foot = object_model.get_part("left_rear_foot")
    right_foot = object_model.get_part("right_rear_foot")

    a_joint = object_model.get_articulation("housing_to_key_a")
    space_joint = object_model.get_articulation("housing_to_key_space")
    left_foot_joint = object_model.get_articulation("housing_to_left_rear_foot")
    right_foot_joint = object_model.get_articulation("housing_to_right_rear_foot")

    housing_aabb = ctx.part_world_aabb(housing)
    housing_dims_ok = False
    if housing_aabb is not None:
        width = housing_aabb[1][0] - housing_aabb[0][0]
        depth = housing_aabb[1][1] - housing_aabb[0][1]
        height = housing_aabb[1][2] - housing_aabb[0][2]
        housing_dims_ok = 0.43 <= width <= 0.46 and 0.13 <= depth <= 0.16 and 0.02 <= height <= 0.04
    ctx.check(
        "housing reads as full-size office keyboard",
        housing_dims_ok,
        details=f"aabb={housing_aabb}",
    )

    ctx.expect_origin_distance(
        key_a,
        numpad_zero,
        axes="x",
        min_dist=0.18,
        name="numeric keypad is clearly separated from typing block",
    )

    rest_a_pos = ctx.part_world_position(key_a)
    with ctx.pose({a_joint: a_joint.motion_limits.upper}):
        pressed_a_pos = ctx.part_world_position(key_a)
        ctx.expect_gap(
            key_a,
            housing,
            axis="z",
            positive_elem="plunger",
            negative_elem="switch_floor",
            min_gap=0.001,
            name="pressed A key keeps travel clearance above switch floor",
        )
    ctx.check(
        "A key depresses downward",
        rest_a_pos is not None
        and pressed_a_pos is not None
        and pressed_a_pos[2] < rest_a_pos[2] - 0.003,
        details=f"rest={rest_a_pos}, pressed={pressed_a_pos}",
    )

    rest_space_pos = ctx.part_world_position(key_space)
    with ctx.pose({space_joint: space_joint.motion_limits.upper}):
        pressed_space_pos = ctx.part_world_position(key_space)
        ctx.expect_gap(
            key_space,
            housing,
            axis="z",
            positive_elem="plunger",
            negative_elem="switch_floor",
            min_gap=0.001,
            name="space bar keeps travel clearance above switch floor",
        )
    ctx.check(
        "space bar also plunges vertically",
        rest_space_pos is not None
        and pressed_space_pos is not None
        and pressed_space_pos[2] < rest_space_pos[2] - 0.003,
        details=f"rest={rest_space_pos}, pressed={pressed_space_pos}",
    )

    closed_left_aabb = ctx.part_world_aabb(left_foot)
    closed_right_aabb = ctx.part_world_aabb(right_foot)
    with ctx.pose({left_foot_joint: left_foot_joint.motion_limits.upper, right_foot_joint: right_foot_joint.motion_limits.upper}):
        open_left_aabb = ctx.part_world_aabb(left_foot)
        open_right_aabb = ctx.part_world_aabb(right_foot)

    ctx.check(
        "rear feet deploy downward from underside",
        housing_aabb is not None
        and closed_left_aabb is not None
        and open_left_aabb is not None
        and open_right_aabb is not None
        and open_left_aabb[0][2] < housing_aabb[0][2] - 0.01
        and open_right_aabb[0][2] < housing_aabb[0][2] - 0.01
        and open_left_aabb[0][2] < closed_left_aabb[0][2] - 0.01,
        details=(
            f"housing={housing_aabb}, closed_left={closed_left_aabb}, open_left={open_left_aabb}, "
            f"closed_right={closed_right_aabb}, open_right={open_right_aabb}"
        ),
    )

    ctx.check(
        "rear feet deploy symmetrically",
        open_left_aabb is not None
        and open_right_aabb is not None
        and abs(open_left_aabb[0][2] - open_right_aabb[0][2]) <= 0.002
        and abs((open_left_aabb[0][1] + open_left_aabb[1][1]) - (open_right_aabb[0][1] + open_right_aabb[1][1])) <= 0.002,
        details=f"open_left={open_left_aabb}, open_right={open_right_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
