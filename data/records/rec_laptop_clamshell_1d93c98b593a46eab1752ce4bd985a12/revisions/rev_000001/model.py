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


BASE_W = 0.315
BASE_D = 0.223
BASE_H = 0.014
DISPLAY_W = 0.307
DISPLAY_D = 0.210
DISPLAY_T = 0.009
BOTTOM_T = 0.0022
DECK_T = 0.0018
WALL_T = 0.0020
DISPLAY_BACK_T = 0.0016
DISPLAY_FACE_T = 0.0012
HINGE_RADIUS = 0.0042
HINGE_Y = 0.108
HINGE_Z = 0.0140
KEY_TRAVEL = 0.0014
KEY_GAP = 0.003
KEY_DEPTH = 0.016
KEY_BOTTOM_Z = 0.0106
KEYCAP_T = 0.0022
KEYSTEM_T = 0.0024
KEY_ROWS = [
    (
        0.055,
        [
            ("esc", 0.018),
            ("1", 0.018),
            ("2", 0.018),
            ("3", 0.018),
            ("4", 0.018),
            ("5", 0.018),
            ("6", 0.018),
            ("7", 0.018),
            ("8", 0.018),
            ("9", 0.018),
            ("0", 0.018),
            ("back", 0.024),
        ],
    ),
    (
        0.032,
        [
            ("tab", 0.024),
            ("q", 0.018),
            ("w", 0.018),
            ("e", 0.018),
            ("r", 0.018),
            ("t", 0.018),
            ("y", 0.018),
            ("u", 0.018),
            ("i", 0.018),
            ("o", 0.018),
            ("p", 0.018),
            ("slash", 0.022),
        ],
    ),
    (
        0.009,
        [
            ("caps", 0.028),
            ("a", 0.018),
            ("s", 0.018),
            ("d", 0.018),
            ("f", 0.018),
            ("g", 0.018),
            ("h", 0.018),
            ("j", 0.018),
            ("k", 0.018),
            ("l", 0.018),
            ("enter", 0.028),
        ],
    ),
    (
        -0.014,
        [
            ("shift_l", 0.034),
            ("z", 0.018),
            ("x", 0.018),
            ("c", 0.018),
            ("v", 0.018),
            ("b", 0.018),
            ("n", 0.018),
            ("m", 0.018),
            ("comma", 0.018),
            ("period", 0.018),
            ("shift_r", 0.030),
        ],
    ),
    (
        -0.037,
        [
            ("ctrl_l", 0.022),
            ("fn", 0.020),
            ("alt_l", 0.020),
            ("spacebar", 0.094),
            ("alt_r", 0.020),
            ("menu", 0.020),
            ("ctrl_r", 0.022),
        ],
    ),
]


def _row_centers(widths: list[float], *, gap: float = KEY_GAP) -> list[float]:
    total = sum(widths) + gap * (len(widths) - 1)
    cursor = -total * 0.5
    centers: list[float] = []
    for width in widths:
        centers.append(cursor + width * 0.5)
        cursor += width + gap
    return centers


def _add_key(
    model: ArticulatedObject,
    base,
    *,
    name: str,
    center_x: float,
    center_y: float,
    width: float,
    depth: float = KEY_DEPTH,
    travel: float = KEY_TRAVEL,
) -> None:
    key_part = model.part(f"key_{name}")
    key_part.visual(
        Box((width, depth, KEYCAP_T)),
        origin=Origin(xyz=(0.0, 0.0, KEYCAP_T * 0.5)),
        material="keycap",
        name="cap",
    )
    key_part.visual(
        Box((min(width * 0.45, 0.010), min(depth * 0.45, 0.007), KEYSTEM_T)),
        origin=Origin(xyz=(0.0, 0.0, -KEYSTEM_T * 0.5)),
        material="key_stem",
        name="stem",
    )
    model.articulation(
        f"base_to_key_{name}",
        ArticulationType.PRISMATIC,
        parent=base,
        child=key_part,
        origin=Origin(xyz=(center_x, center_y, KEY_BOTTOM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.03,
            lower=0.0,
            upper=travel,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convertible_laptop")

    model.material("aluminum_dark", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("aluminum_mid", rgba=(0.43, 0.45, 0.48, 1.0))
    model.material("keyboard_well", rgba=(0.17, 0.18, 0.19, 1.0))
    model.material("keycap", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("key_stem", rgba=(0.21, 0.22, 0.24, 1.0))
    model.material("touchpad", rgba=(0.58, 0.60, 0.64, 1.0))
    model.material("screen_glass", rgba=(0.08, 0.11, 0.14, 1.0))
    model.material("screen_bezel", rgba=(0.06, 0.06, 0.07, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_W, BASE_D, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_T * 0.5)),
        material="aluminum_dark",
        name="bottom_cover",
    )
    wall_height = BASE_H - BOTTOM_T
    wall_z = BOTTOM_T + wall_height * 0.5
    base.visual(
        Box((WALL_T, BASE_D, wall_height)),
        origin=Origin(xyz=(-BASE_W * 0.5 + WALL_T * 0.5, 0.0, wall_z)),
        material="aluminum_mid",
        name="left_wall",
    )
    base.visual(
        Box((WALL_T, BASE_D, wall_height)),
        origin=Origin(xyz=(BASE_W * 0.5 - WALL_T * 0.5, 0.0, wall_z)),
        material="aluminum_mid",
        name="right_wall",
    )
    base.visual(
        Box((BASE_W - 2.0 * WALL_T, WALL_T, 0.010)),
        origin=Origin(xyz=(0.0, -BASE_D * 0.5 + WALL_T * 0.5, 0.0072)),
        material="aluminum_mid",
        name="front_wall",
    )
    top_z = BASE_H - DECK_T * 0.5
    base.visual(
        Box((BASE_W - 2.0 * WALL_T, 0.074, DECK_T)),
        origin=Origin(xyz=(0.0, -0.074, top_z)),
        material="aluminum_mid",
        name="palmrest_deck",
    )
    base.visual(
        Box((0.018, 0.108, DECK_T)),
        origin=Origin(xyz=(-0.147, 0.010, top_z)),
        material="aluminum_mid",
        name="left_keyboard_rail",
    )
    base.visual(
        Box((0.018, 0.108, DECK_T)),
        origin=Origin(xyz=(0.147, 0.010, top_z)),
        material="aluminum_mid",
        name="right_keyboard_rail",
    )
    base.visual(
        Box((BASE_W - 2.0 * WALL_T, 0.024, DECK_T)),
        origin=Origin(xyz=(0.0, 0.086, top_z)),
        material="aluminum_mid",
        name="rear_deck_strip",
    )
    base.visual(
        Box((0.271, 0.112, 0.0012)),
        origin=Origin(xyz=(0.0, 0.010, 0.0058)),
        material="keyboard_well",
        name="keyboard_bed",
    )
    base.visual(
        Box((0.092, 0.056, 0.0007)),
        origin=Origin(xyz=(0.0, -0.070, BASE_H - 0.00035)),
        material="touchpad",
        name="touchpad_pad",
    )
    base.visual(
        Box((0.005, 0.112, 0.0058)),
        origin=Origin(xyz=(-0.138, 0.010, 0.0093)),
        material="keyboard_well",
        name="left_bed_riser",
    )
    base.visual(
        Box((0.005, 0.112, 0.0058)),
        origin=Origin(xyz=(0.138, 0.010, 0.0093)),
        material="keyboard_well",
        name="right_bed_riser",
    )
    base.visual(
        Box((0.276, 0.006, 0.0058)),
        origin=Origin(xyz=(0.0, -0.040, 0.0093)),
        material="keyboard_well",
        name="front_bed_riser",
    )
    base.visual(
        Box((0.276, 0.006, 0.0058)),
        origin=Origin(xyz=(0.0, 0.071, 0.0093)),
        material="keyboard_well",
        name="rear_bed_riser",
    )
    base.visual(
        Box((0.036, 0.010, 0.0075)),
        origin=Origin(xyz=(-0.123, 0.101, 0.00925)),
        material="aluminum_dark",
        name="left_hinge_support",
    )
    base.visual(
        Box((0.036, 0.010, 0.0075)),
        origin=Origin(xyz=(0.123, 0.101, 0.00925)),
        material="aluminum_dark",
        name="right_hinge_support",
    )
    base.visual(
        Box((BASE_W - 0.090, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.100, 0.0070)),
        material="aluminum_dark",
        name="rear_spine",
    )
    base.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.034),
        origin=Origin(xyz=(-0.122, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="aluminum_mid",
        name="left_hinge_barrel",
    )
    base.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.034),
        origin=Origin(xyz=(0.122, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="aluminum_mid",
        name="right_hinge_barrel",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_H)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, BASE_H * 0.5)),
    )

    display = model.part("display")
    display.visual(
        Box((DISPLAY_W, DISPLAY_D, DISPLAY_BACK_T)),
        origin=Origin(xyz=(0.0, -DISPLAY_D * 0.5, DISPLAY_T - DISPLAY_BACK_T * 0.5)),
        material="aluminum_dark",
        name="back_cover",
    )
    side_wall_h = DISPLAY_T - DISPLAY_BACK_T - DISPLAY_FACE_T
    side_wall_z = DISPLAY_FACE_T + side_wall_h * 0.5
    display.visual(
        Box((0.002, DISPLAY_D, side_wall_h)),
        origin=Origin(xyz=(-DISPLAY_W * 0.5 + 0.001, -DISPLAY_D * 0.5, side_wall_z)),
        material="aluminum_mid",
        name="left_frame",
    )
    display.visual(
        Box((0.002, DISPLAY_D, side_wall_h)),
        origin=Origin(xyz=(DISPLAY_W * 0.5 - 0.001, -DISPLAY_D * 0.5, side_wall_z)),
        material="aluminum_mid",
        name="right_frame",
    )
    display.visual(
        Box((DISPLAY_W - 0.004, 0.002, side_wall_h)),
        origin=Origin(xyz=(0.0, -DISPLAY_D + 0.001, side_wall_z)),
        material="aluminum_mid",
        name="top_frame",
    )
    display.visual(
        Box((0.140, 0.002, side_wall_h)),
        origin=Origin(xyz=(0.0, -0.001, side_wall_z)),
        material="aluminum_mid",
        name="bottom_frame",
    )
    display.visual(
        Box((0.120, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.007, 0.002)),
        material="aluminum_mid",
        name="hinge_bridge",
    )
    display.visual(
        Box((0.026, 0.008, 0.0062)),
        origin=Origin(xyz=(-0.081, -0.004, 0.0043)),
        material="aluminum_mid",
        name="left_hinge_leaf",
    )
    display.visual(
        Box((0.026, 0.008, 0.0062)),
        origin=Origin(xyz=(0.081, -0.004, 0.0043)),
        material="aluminum_mid",
        name="right_hinge_leaf",
    )
    display.visual(
        Box((0.010, DISPLAY_D - 0.026, DISPLAY_FACE_T)),
        origin=Origin(xyz=(-0.1485, -0.111, DISPLAY_FACE_T * 0.5)),
        material="screen_bezel",
        name="left_bezel",
    )
    display.visual(
        Box((0.010, DISPLAY_D - 0.026, DISPLAY_FACE_T)),
        origin=Origin(xyz=(0.1485, -0.111, DISPLAY_FACE_T * 0.5)),
        material="screen_bezel",
        name="right_bezel",
    )
    display.visual(
        Box((DISPLAY_W - 0.040, 0.010, DISPLAY_FACE_T)),
        origin=Origin(xyz=(0.0, -0.200, DISPLAY_FACE_T * 0.5)),
        material="screen_bezel",
        name="top_bezel",
    )
    display.visual(
        Box((DISPLAY_W - 0.040, 0.014, DISPLAY_FACE_T)),
        origin=Origin(xyz=(0.0, -0.014, DISPLAY_FACE_T * 0.5)),
        material="screen_bezel",
        name="bottom_bezel",
    )
    display.visual(
        Box((0.288, 0.194, 0.0012)),
        origin=Origin(xyz=(0.0, -0.107, DISPLAY_FACE_T * 0.5)),
        material="screen_glass",
        name="panel",
    )
    display.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.044),
        origin=Origin(xyz=(-0.081, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="aluminum_mid",
        name="left_display_barrel",
    )
    display.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.044),
        origin=Origin(xyz=(0.081, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="aluminum_mid",
        name="right_display_barrel",
    )
    display.inertial = Inertial.from_geometry(
        Box((DISPLAY_W, DISPLAY_D, DISPLAY_T)),
        mass=0.85,
        origin=Origin(xyz=(0.0, -DISPLAY_D * 0.5, DISPLAY_T * 0.5)),
    )

    model.articulation(
        "base_to_display",
        ArticulationType.REVOLUTE,
        parent=base,
        child=display,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    for center_y, row in KEY_ROWS:
        widths = [width for _, width in row]
        for center_x, (name, width) in zip(_row_centers(widths), row):
            _add_key(
                model,
                base,
                name=name,
                center_x=center_x,
                center_y=center_y,
                width=width,
                travel=0.0012 if name == "spacebar" else KEY_TRAVEL,
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    display = object_model.get_part("display")
    display_hinge = object_model.get_articulation("base_to_display")
    key_f = object_model.get_part("key_f")
    key_spacebar = object_model.get_part("key_spacebar")
    key_f_joint = object_model.get_articulation("base_to_key_f")
    spacebar_joint = object_model.get_articulation("base_to_key_spacebar")

    for _, row in KEY_ROWS:
        for key_name, _ in row:
            ctx.allow_isolated_part(
                object_model.get_part(f"key_{key_name}"),
                reason="Laptop keys are intentionally represented as spring-guided plungers hovering slightly above the switch well in the rest pose.",
            )

    with ctx.pose({display_hinge: 0.0}):
        ctx.expect_gap(
            display,
            base,
            axis="z",
            positive_elem="panel",
            negative_elem="palmrest_deck",
            min_gap=0.0,
            max_gap=0.001,
            name="closed display panel settles onto the palm rest deck line",
        )
        ctx.expect_gap(
            display,
            base,
            axis="z",
            positive_elem="bottom_bezel",
            negative_elem="rear_deck_strip",
            min_gap=0.0,
            max_gap=0.001,
            name="closed display lower bezel seats at the rear deck line",
        )
        ctx.expect_overlap(
            display,
            base,
            axes="xy",
            min_overlap=0.15,
            name="closed display covers the base footprint",
        )
        ctx.expect_gap(
            key_f,
            base,
            axis="z",
            positive_elem="stem",
            negative_elem="keyboard_bed",
            min_gap=0.001,
            max_gap=0.005,
            name="letter key stem floats above keyboard bed at rest",
        )
        ctx.expect_gap(
            key_spacebar,
            base,
            axis="z",
            positive_elem="stem",
            negative_elem="keyboard_bed",
            min_gap=0.001,
            max_gap=0.005,
            name="spacebar stem floats above keyboard bed at rest",
        )

    display_closed = None
    display_open = None
    with ctx.pose({display_hinge: 0.0}):
        display_closed = ctx.part_world_aabb(display)
    with ctx.pose({display_hinge: math.pi * 0.5}):
        display_open = ctx.part_world_aabb(display)
    ctx.check(
        "display opens upward from the rear hinge axis",
        display_closed is not None
        and display_open is not None
        and display_open[1][2] > display_closed[1][2] + 0.12,
        details=f"closed={display_closed}, open={display_open}",
    )

    display_folded = None
    with ctx.pose({display_hinge: math.pi}):
        display_folded = ctx.part_world_aabb(display)
        ctx.expect_gap(
            display,
            base,
            axis="x",
            positive_elem="left_display_barrel",
            negative_elem="left_hinge_support",
            min_gap=0.0005,
            max_gap=0.003,
            name="folded display barrel clears the left hinge support",
        )
        ctx.expect_gap(
            base,
            display,
            axis="x",
            positive_elem="right_hinge_support",
            negative_elem="right_display_barrel",
            min_gap=0.0005,
            max_gap=0.003,
            name="folded display barrel clears the right hinge support",
        )
    ctx.check(
        "display folds fully behind the keyboard deck",
        display_folded is not None and display_folded[0][1] > -0.001,
        details=f"folded={display_folded}",
    )

    key_rest = None
    key_pressed = None
    with ctx.pose({key_f_joint: 0.0}):
        key_rest = ctx.part_world_position(key_f)
    with ctx.pose({key_f_joint: key_f_joint.motion_limits.upper}):
        key_pressed = ctx.part_world_position(key_f)
    ctx.check(
        "letter key plunges downward",
        key_rest is not None and key_pressed is not None and key_pressed[2] < key_rest[2] - 0.001,
        details=f"rest={key_rest}, pressed={key_pressed}",
    )

    spacebar_rest = None
    spacebar_pressed = None
    with ctx.pose({spacebar_joint: 0.0}):
        spacebar_rest = ctx.part_world_position(key_spacebar)
    with ctx.pose({spacebar_joint: spacebar_joint.motion_limits.upper}):
        spacebar_pressed = ctx.part_world_position(key_spacebar)
    ctx.check(
        "spacebar also plunges vertically",
        spacebar_rest is not None
        and spacebar_pressed is not None
        and spacebar_pressed[2] < spacebar_rest[2] - 0.0008,
        details=f"rest={spacebar_rest}, pressed={spacebar_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
