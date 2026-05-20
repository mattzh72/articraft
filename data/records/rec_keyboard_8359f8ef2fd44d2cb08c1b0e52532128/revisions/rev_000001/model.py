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

UNIT = 0.019
CASE_WIDTH = 0.372
CASE_DEPTH = 0.142
DECK_TOP_Z = 0.017
KEY_REST_GAP = 0.0012
KEY_TRAVEL = 0.0011
MAIN_KEY_HEIGHT = 0.0075
FUNCTION_KEY_HEIGHT = 0.0066
MAIN_KEY_DEPTH = 0.0168
FUNCTION_KEY_DEPTH = 0.0138


def _key_specs() -> list[dict[str, float | str]]:
    left = -(18.5 * UNIT) * 0.5
    rows = (
        (
            0.037,
            FUNCTION_KEY_DEPTH,
            FUNCTION_KEY_HEIGHT,
            0.0009,
            (
                ("key_esc", 1.0),
                ("gap", 0.5),
                ("key_f1", 1.0),
                ("key_f2", 1.0),
                ("key_f3", 1.0),
                ("key_f4", 1.0),
                ("gap", 0.25),
                ("key_f5", 1.0),
                ("key_f6", 1.0),
                ("key_f7", 1.0),
                ("key_f8", 1.0),
                ("gap", 0.25),
                ("key_f9", 1.0),
                ("key_f10", 1.0),
                ("key_f11", 1.0),
                ("key_f12", 1.0),
                ("gap", 0.5),
                ("key_print_screen", 1.0),
                ("key_scroll_lock", 1.0),
                ("key_pause", 1.0),
            ),
        ),
        (
            0.016,
            MAIN_KEY_DEPTH,
            MAIN_KEY_HEIGHT,
            KEY_TRAVEL,
            (
                ("key_grave", 1.0),
                ("key_1", 1.0),
                ("key_2", 1.0),
                ("key_3", 1.0),
                ("key_4", 1.0),
                ("key_5", 1.0),
                ("key_6", 1.0),
                ("key_7", 1.0),
                ("key_8", 1.0),
                ("key_9", 1.0),
                ("key_0", 1.0),
                ("key_minus", 1.0),
                ("key_equal", 1.0),
                ("key_backspace", 2.0),
                ("gap", 0.5),
                ("key_insert", 1.0),
                ("key_home", 1.0),
                ("key_page_up", 1.0),
            ),
        ),
        (
            -0.003,
            MAIN_KEY_DEPTH,
            MAIN_KEY_HEIGHT,
            KEY_TRAVEL,
            (
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
                ("gap", 0.5),
                ("key_delete", 1.0),
                ("key_end", 1.0),
                ("key_page_down", 1.0),
            ),
        ),
        (
            -0.022,
            MAIN_KEY_DEPTH,
            MAIN_KEY_HEIGHT,
            KEY_TRAVEL,
            (
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
                ("key_apostrophe", 1.0),
                ("key_enter", 2.25),
            ),
        ),
        (
            -0.041,
            MAIN_KEY_DEPTH,
            MAIN_KEY_HEIGHT,
            KEY_TRAVEL,
            (
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
                ("gap", 1.0),
                ("key_up", 1.0),
            ),
        ),
        (
            -0.058,
            0.0162,
            0.0071,
            0.0010,
            (
                ("key_left_ctrl", 1.25),
                ("key_left_win", 1.25),
                ("key_left_alt", 1.25),
                ("key_space", 6.25),
                ("key_right_alt", 1.25),
                ("key_fn", 1.0),
                ("key_menu", 1.0),
                ("key_right_ctrl", 1.25),
                ("gap", 0.5),
                ("key_left", 1.0),
                ("key_down", 1.0),
                ("key_right", 1.0),
            ),
        ),
    )

    specs: list[dict[str, float | str]] = []
    for y, depth, height, travel, items in rows:
        cursor = left
        for name, width_u in items:
            if name == "gap":
                cursor += width_u * UNIT
                continue
            center_x = cursor + (width_u * UNIT * 0.5)
            specs.append(
                {
                    "name": name,
                    "x": center_x,
                    "y": y,
                    "width_u": width_u,
                    "depth": depth,
                    "height": height,
                    "travel": travel,
                }
            )
            cursor += width_u * UNIT
    return specs


def _add_key(
    model: ArticulatedObject,
    case_part,
    *,
    name: str,
    x: float,
    y: float,
    width_u: float,
    depth: float,
    height: float,
    travel: float,
    body_material: str,
    top_material: str,
    guide_material: str,
) -> None:
    key_part = model.part(name)
    cap_width = max(width_u * UNIT - 0.0022, 0.010)
    stem_width = min(max(cap_width * 0.32, 0.0038), 0.0062)
    stem_depth = min(max(depth * 0.32, 0.0038), 0.0062)
    body_height = max(height - 0.0015, 0.0048)
    top_height = height - body_height
    top_width = max(cap_width - 0.0024, cap_width * 0.82)
    top_depth = max(depth - 0.0022, depth * 0.82)

    case_part.visual(
        Box((stem_width, stem_depth, 0.015)),
        origin=Origin(xyz=(x, y, 0.0075)),
        material=guide_material,
        name=f"{name}_switch_guide",
    )
    key_part.visual(
        Box((cap_width, depth, body_height)),
        origin=Origin(xyz=(0.0, 0.0, KEY_REST_GAP + body_height * 0.5)),
        material=body_material,
        name="cap_body",
    )
    key_part.visual(
        Box((top_width, top_depth, top_height)),
        origin=Origin(
            xyz=(0.0, 0.0, KEY_REST_GAP + body_height + top_height * 0.5)
        ),
        material=top_material,
        name="cap_top",
    )
    key_part.visual(
        Box((stem_width - 0.0006, stem_depth - 0.0006, 0.0032)),
        origin=Origin(xyz=(0.0, 0.0, -0.0004)),
        material=body_material,
        name="stem",
    )
    key_part.inertial = Inertial.from_geometry(
        Box((cap_width, depth, height)),
        mass=max(0.006, 0.008 * width_u),
        origin=Origin(xyz=(0.0, 0.0, KEY_REST_GAP + height * 0.5)),
    )

    model.articulation(
        f"case_to_{name}",
        ArticulationType.PRISMATIC,
        parent=case_part,
        child=key_part,
        origin=Origin(xyz=(x, y, DECK_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=travel,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tenkeyless_gaming_keyboard")

    case_shell = model.material("case_shell", rgba=(0.10, 0.11, 0.12, 1.0))
    case_trim = model.material("case_trim", rgba=(0.15, 0.16, 0.18, 1.0))
    deck_finish = model.material("deck_finish", rgba=(0.07, 0.08, 0.09, 1.0))
    key_body = model.material("key_body", rgba=(0.11, 0.12, 0.13, 1.0))
    key_top = model.material("key_top", rgba=(0.18, 0.19, 0.21, 1.0))
    roller_metal = model.material("roller_metal", rgba=(0.60, 0.63, 0.68, 1.0))
    roller_dark = model.material("roller_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    case = model.part("case")
    case.visual(
        Box((0.360, 0.132, 0.003)),
        origin=Origin(xyz=(0.0, -0.001, 0.0015)),
        material=case_shell,
        name="bottom_panel",
    )
    case.visual(
        Box((0.360, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.067, 0.006)),
        material=case_shell,
        name="front_wall",
    )
    case.visual(
        Box((0.360, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.065, 0.012)),
        material=case_shell,
        name="rear_wall",
    )
    case.visual(
        Box((0.008, 0.132, 0.020)),
        origin=Origin(xyz=(-0.182, -0.001, 0.010)),
        material=case_shell,
        name="left_wall",
    )
    case.visual(
        Box((0.008, 0.132, 0.020)),
        origin=Origin(xyz=(0.182, -0.001, 0.010)),
        material=case_shell,
        name="right_wall",
    )
    case.visual(
        Box((0.352, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, -0.064, 0.0155)),
        material=deck_finish,
        name="deck_front_rail",
    )
    case.visual(
        Box((0.352, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, 0.056, 0.0155)),
        material=deck_finish,
        name="deck_rear_rail",
    )
    case.visual(
        Box((0.008, 0.122, 0.003)),
        origin=Origin(xyz=(-0.176, -0.004, 0.0155)),
        material=deck_finish,
        name="deck_left_rail",
    )
    case.visual(
        Box((0.008, 0.122, 0.003)),
        origin=Origin(xyz=(0.176, -0.004, 0.0155)),
        material=deck_finish,
        name="deck_right_rail",
    )
    case.visual(
        Box((0.352, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, 0.060, 0.0205)),
        material=case_trim,
        name="rear_top_lip",
    )
    case.visual(
        Box((0.336, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.069, 0.0065)),
        material=case_trim,
        name="front_trim",
    )
    case.visual(
        Box((0.005, 0.010, 0.015)),
        origin=Origin(xyz=(0.139, 0.054, 0.0235)),
        material=case_trim,
        name="roller_bracket_left",
    )
    case.visual(
        Box((0.005, 0.010, 0.015)),
        origin=Origin(xyz=(0.163, 0.054, 0.0235)),
        material=case_trim,
        name="roller_bracket_right",
    )
    case.visual(
        Box((0.030, 0.004, 0.008)),
        origin=Origin(xyz=(0.151, 0.061, 0.023)),
        material=case_trim,
        name="roller_bridge",
    )
    case.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, 0.028)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    for spec in _key_specs():
        _add_key(
            model,
            case,
            name=str(spec["name"]),
            x=float(spec["x"]),
            y=float(spec["y"]),
            width_u=float(spec["width_u"]),
            depth=float(spec["depth"]),
            height=float(spec["height"]),
            travel=float(spec["travel"]),
            body_material=key_body,
            top_material=key_top,
            guide_material=case_trim,
        )

    media_roller = model.part("media_roller")
    media_roller.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=roller_metal,
        name="roller_sleeve",
    )
    media_roller.visual(
        Cylinder(radius=0.0022, length=0.004),
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=roller_dark,
        name="roller_axle_left",
    )
    media_roller.visual(
        Cylinder(radius=0.0022, length=0.004),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=roller_dark,
        name="roller_axle_right",
    )
    media_roller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.006, length=0.018),
        mass=0.02,
        origin=Origin(),
    )
    model.articulation(
        "case_to_media_roller",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=media_roller,
        origin=Origin(xyz=(0.151, 0.054, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
    )

    for foot_name, foot_x in (("left_foot", -0.120), ("right_foot", 0.120)):
        foot = model.part(foot_name)
        foot.visual(
            Cylinder(radius=0.003, length=0.038),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=case_trim,
            name="barrel",
        )
        foot.visual(
            Box((0.032, 0.030, 0.004)),
            origin=Origin(xyz=(0.0, -0.014, -0.004)),
            material=case_trim,
            name="arm",
        )
        foot.visual(
            Box((0.026, 0.008, 0.002)),
            origin=Origin(xyz=(0.0, -0.027, -0.006)),
            material=foot_rubber,
            name="foot_pad",
        )
        foot.inertial = Inertial.from_geometry(
            Box((0.038, 0.032, 0.010)),
            mass=0.025,
            origin=Origin(xyz=(0.0, -0.012, -0.003)),
        )
        model.articulation(
            f"case_to_{foot_name}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=foot,
            origin=Origin(xyz=(foot_x, 0.061, 0.003)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.6,
                velocity=2.0,
                lower=0.0,
                upper=1.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    roller = object_model.get_part("media_roller")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")
    key_w = object_model.get_part("key_w")
    key_space = object_model.get_part("key_space")
    key_up = object_model.get_part("key_up")
    key_w_guide = case.get_visual("key_w_switch_guide")
    key_space_guide = case.get_visual("key_space_switch_guide")

    roller_joint = object_model.get_articulation("case_to_media_roller")
    left_foot_joint = object_model.get_articulation("case_to_left_foot")
    right_foot_joint = object_model.get_articulation("case_to_right_foot")
    key_w_joint = object_model.get_articulation("case_to_key_w")
    space_joint = object_model.get_articulation("case_to_key_space")

    missing_keys: list[str] = []
    for spec in _key_specs():
        try:
            object_model.get_part(str(spec["name"]))
        except Exception:
            missing_keys.append(str(spec["name"]))
    ctx.check(
        "dense tenkeyless key grid is present",
        not missing_keys and len(_key_specs()) >= 80,
        details=f"missing={missing_keys[:8]}, total_expected={len(_key_specs())}",
    )

    ctx.expect_contact(
        key_w,
        case,
        elem_a="stem",
        elem_b=key_w_guide,
        contact_tol=5e-5,
        name="W key stem is seated in its switch guide",
    )
    ctx.expect_contact(
        key_space,
        case,
        elem_a="stem",
        elem_b=key_space_guide,
        contact_tol=5e-5,
        name="space bar stem is seated in its switch guide",
    )

    w_rest = ctx.part_world_position(key_w)
    with ctx.pose({key_w_joint: 0.0011}):
        w_pressed = ctx.part_world_position(key_w)
    ctx.check(
        "W key travels downward when pressed",
        w_rest is not None
        and w_pressed is not None
        and w_pressed[2] < w_rest[2] - 0.0009,
        details=f"rest={w_rest}, pressed={w_pressed}",
    )

    space_rest = ctx.part_world_position(key_space)
    with ctx.pose({space_joint: 0.0010}):
        space_pressed = ctx.part_world_position(key_space)
    ctx.check(
        "space bar also plunges downward",
        space_rest is not None
        and space_pressed is not None
        and space_pressed[2] < space_rest[2] - 0.0008,
        details=f"rest={space_rest}, pressed={space_pressed}",
    )

    ctx.check(
        "media roller spins about a left-right axis",
        roller_joint.articulation_type == ArticulationType.CONTINUOUS
        and roller_joint.axis == (1.0, 0.0, 0.0),
        details=f"type={roller_joint.articulation_type}, axis={roller_joint.axis}",
    )
    ctx.expect_origin_gap(
        roller,
        case,
        axis="x",
        min_gap=0.14,
        name="media roller sits near the right edge",
    )
    ctx.expect_origin_gap(
        roller,
        case,
        axis="y",
        min_gap=0.045,
        name="media roller sits near the back edge",
    )

    left_pad_rest = ctx.part_element_world_aabb(left_foot, elem="foot_pad")
    with ctx.pose({left_foot_joint: 1.0}):
        left_pad_open = ctx.part_element_world_aabb(left_foot, elem="foot_pad")
    ctx.check(
        "left rear foot folds downward",
        left_pad_rest is not None
        and left_pad_open is not None
        and left_pad_open[0][2] < left_pad_rest[0][2] - 0.015,
        details=f"rest={left_pad_rest}, open={left_pad_open}",
    )
    ctx.check(
        "rear feet hinge about left-right axes",
        left_foot_joint.axis == (1.0, 0.0, 0.0)
        and right_foot_joint.axis == (1.0, 0.0, 0.0),
        details=f"left={left_foot_joint.axis}, right={right_foot_joint.axis}",
    )

    ctx.expect_origin_gap(
        key_up,
        case,
        axis="x",
        min_gap=0.13,
        name="arrow cluster sits on the right side of the deck",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
