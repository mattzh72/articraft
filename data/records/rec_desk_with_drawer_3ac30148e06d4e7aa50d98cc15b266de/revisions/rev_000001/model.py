from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


WOOD = Material("polished_dark_walnut", rgba=(0.23, 0.10, 0.035, 1.0))
WOOD_DARK = Material("darker_end_grain", rgba=(0.12, 0.055, 0.025, 1.0))
WOOD_LIGHT = Material("raised_walnut_panel", rgba=(0.32, 0.15, 0.055, 1.0))
LEATHER = Material("green_leather_inset", rgba=(0.025, 0.12, 0.075, 1.0))
BRASS = Material("aged_brass", rgba=(0.86, 0.58, 0.20, 1.0))
STEEL = Material("blued_steel_rails", rgba=(0.18, 0.19, 0.20, 1.0))
SHADOW = Material("deep_drawer_shadow", rgba=(0.025, 0.018, 0.012, 1.0))


DESK_LENGTH = 1.80
DESK_DEPTH = 0.90
DESK_HEIGHT = 0.78
TOP_THICKNESS = 0.08
PEDESTAL_WIDTH = 0.55
PEDESTAL_DEPTH = 0.72
PEDESTAL_HEIGHT = 0.70
SIDE_WALL = 0.035
OPENING_FACE_Y = PEDESTAL_DEPTH / 2.0
DRAWER_FRONT_T = 0.032
DRAWER_TRAVEL = 0.30

DRAWER_LAYOUT = (
    ("bottom", 0.205, 0.220),
    ("middle", 0.415, 0.180),
    ("top", 0.595, 0.150),
)


def _add_visual(part, geometry, xyz, name, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(geometry, origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_pedestal_carcass(desk, prefix: str, x_center: float, face_sign: float) -> None:
    """Static hollow pedestal: open toward face_sign * Y, backed on the opposite side."""

    half_w = PEDESTAL_WIDTH / 2.0
    half_d = PEDESTAL_DEPTH / 2.0
    inner_w = PEDESTAL_WIDTH - 2.0 * SIDE_WALL
    frame_y = face_sign * (half_d + 0.009)

    # Structural side walls, back, and bottom/top shelves make an open wooden box.
    for side_name, side in (("neg", -1.0), ("pos", 1.0)):
        _add_visual(
            desk,
            Box((SIDE_WALL, PEDESTAL_DEPTH, PEDESTAL_HEIGHT)),
            (x_center + side * (half_w - SIDE_WALL / 2.0), 0.0, PEDESTAL_HEIGHT / 2.0),
            f"{prefix}_side_wall_{side_name}",
            WOOD,
        )
    _add_visual(
        desk,
        Box((PEDESTAL_WIDTH, SIDE_WALL, PEDESTAL_HEIGHT)),
        (x_center, -face_sign * (half_d - SIDE_WALL / 2.0), PEDESTAL_HEIGHT / 2.0),
        f"{prefix}_back_wall",
        WOOD,
    )
    _add_visual(
        desk,
        Box((PEDESTAL_WIDTH, PEDESTAL_DEPTH, 0.045)),
        (x_center, 0.0, 0.0225),
        f"{prefix}_bottom_shelf",
        WOOD_DARK,
    )
    _add_visual(
        desk,
        Box((PEDESTAL_WIDTH, PEDESTAL_DEPTH, 0.038)),
        (x_center, 0.0, PEDESTAL_HEIGHT - 0.019),
        f"{prefix}_top_rail",
        WOOD,
    )

    # Horizontal drawer separators set back from the fronts so the moving faces clear them.
    for z, rail_name in ((0.320, "lower_separator"), (0.512, "upper_separator")):
        _add_visual(
            desk,
            Box((inner_w + 0.012, PEDESTAL_DEPTH - 0.070, 0.018)),
            (x_center, -face_sign * 0.018, z),
            f"{prefix}_{rail_name}",
            WOOD_DARK,
        )

    # Traditional face frame around the drawer stack, open in front of the drawer faces.
    _add_visual(
        desk,
        Box((0.040, 0.018, 0.610)),
        (x_center - half_w + 0.020, frame_y, 0.385),
        f"{prefix}_face_stile_0",
        WOOD_DARK,
    )
    _add_visual(
        desk,
        Box((0.040, 0.018, 0.610)),
        (x_center + half_w - 0.020, frame_y, 0.385),
        f"{prefix}_face_stile_1",
        WOOD_DARK,
    )
    for z, rail_name in (
        (0.082, "bottom_face_rail"),
        (0.320, "lower_face_rail"),
        (0.512, "upper_face_rail"),
        (0.682, "top_face_rail"),
    ):
        _add_visual(
            desk,
            Box((inner_w + 0.030, 0.018, 0.012)),
            (x_center, frame_y, z),
            f"{prefix}_{rail_name}",
            WOOD_DARK,
        )

    # Plinth blocks connect to the pedestal bottoms and give the desk a heavy traditional base.
    _add_visual(
        desk,
        Box((PEDESTAL_WIDTH + 0.045, 0.052, 0.070)),
        (x_center, face_sign * (half_d - 0.026), 0.035),
        f"{prefix}_front_plinth",
        WOOD_DARK,
    )
    _add_visual(
        desk,
        Box((PEDESTAL_WIDTH + 0.045, 0.052, 0.070)),
        (x_center, -face_sign * (half_d - 0.026), 0.035),
        f"{prefix}_rear_plinth",
        WOOD_DARK,
    )

    # Fixed guide rails on both side walls for each drawer.  The drawer runners touch
    # their inner faces and remain inserted when the prismatic joints extend.
    rail_w = 0.023
    rail_h = 0.018
    rail_len = 0.500
    rail_center_y = face_sign * (half_d - rail_len / 2.0 - 0.012)
    side_inner = half_w - SIDE_WALL
    for level, z_center, drawer_h in DRAWER_LAYOUT:
        rail_z = z_center - drawer_h / 2.0 + 0.045
        _add_visual(
            desk,
            Box((rail_w, rail_len, rail_h)),
            (x_center + side_inner - rail_w / 2.0 + 0.0015, rail_center_y, rail_z),
            f"{prefix}_{level}_rail_pos",
            STEEL,
        )
        _add_visual(
            desk,
            Box((rail_w, rail_len, rail_h)),
            (x_center - side_inner + rail_w / 2.0 - 0.0015, rail_center_y, rail_z),
            f"{prefix}_{level}_rail_neg",
            STEEL,
        )

    # A dark backboard visible past the opened drawers gives the cavities real depth.
    _add_visual(
        desk,
        Box((inner_w - 0.015, 0.012, 0.565)),
        (x_center, -face_sign * (half_d - SIDE_WALL - 0.006), 0.375),
        f"{prefix}_cavity_shadow",
        SHADOW,
    )


def _build_drawer(model, desk, prefix: str, level: str, x_center: float, z_center: float, height: float, face_sign: float):
    drawer = model.part(f"{prefix}_{level}_drawer")

    front_w = 0.455
    box_w = 0.390
    box_depth = 0.490
    box_h = height - 0.055
    runner_w = 0.023
    runner_h = 0.018
    rail_z = -height / 2.0 + 0.045
    box_center_y = -face_sign * (DRAWER_FRONT_T / 2.0 + box_depth / 2.0 - 0.004)

    _add_visual(drawer, Box((front_w, DRAWER_FRONT_T, height)), (0.0, 0.0, 0.0), "front_panel", WOOD_LIGHT)
    _add_visual(
        drawer,
        Box((front_w - 0.090, 0.006, height - 0.060)),
        (0.0, face_sign * (DRAWER_FRONT_T / 2.0 + 0.003), 0.0),
        "raised_field",
        WOOD,
    )
    _add_visual(drawer, Box((box_w, box_depth, box_h)), (0.0, box_center_y, 0.0), "drawer_box", WOOD_DARK)

    _add_visual(
        drawer,
        Box((runner_w, box_depth - 0.018, runner_h)),
        (box_w / 2.0 + runner_w / 2.0 - 0.0015, box_center_y, rail_z),
        "runner_pos",
        STEEL,
    )
    _add_visual(
        drawer,
        Box((runner_w, box_depth - 0.018, runner_h)),
        (-(box_w / 2.0 + runner_w / 2.0 - 0.0015), box_center_y, rail_z),
        "runner_neg",
        STEEL,
    )

    # Brass bail-pull represented by two stems and a horizontal grip bar.
    post_len = 0.034
    post_y = face_sign * (DRAWER_FRONT_T / 2.0 + post_len / 2.0)
    bar_y = face_sign * (DRAWER_FRONT_T / 2.0 + post_len - 0.004)
    for x in (-0.075, 0.075):
        _add_visual(
            drawer,
            Cylinder(radius=0.0065, length=post_len),
            (x, post_y, 0.0),
            "handle_post_0" if x < 0.0 else "handle_post_1",
            BRASS,
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )
    _add_visual(
        drawer,
        Cylinder(radius=0.009, length=0.195),
        (0.0, bar_y, 0.0),
        "handle_bar",
        BRASS,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )

    joint = model.articulation(
        f"{prefix}_{level}_slide",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=drawer,
        origin=Origin(xyz=(x_center, face_sign * (OPENING_FACE_Y + DRAWER_FRONT_T / 2.0), z_center)),
        axis=(0.0, face_sign, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=0.0, upper=DRAWER_TRAVEL),
    )
    return drawer, joint


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_partners_desk")

    desk = model.part("desk")

    # Wide writing top with a leather inset and molded perimeter, large enough for
    # partners to use from opposite sides.
    _add_visual(
        desk,
        Box((DESK_LENGTH, DESK_DEPTH, TOP_THICKNESS)),
        (0.0, 0.0, DESK_HEIGHT - TOP_THICKNESS / 2.0),
        "wide_top",
        WOOD,
    )
    _add_visual(
        desk,
        Box((1.48, 0.62, 0.006)),
        (0.0, 0.0, DESK_HEIGHT + 0.003),
        "leather_writing_inset",
        LEATHER,
    )
    _add_visual(desk, Box((DESK_LENGTH + 0.040, 0.045, 0.055)), (0.0, DESK_DEPTH / 2.0 - 0.022, 0.685), "front_top_molding", WOOD_DARK)
    _add_visual(desk, Box((DESK_LENGTH + 0.040, 0.045, 0.055)), (0.0, -DESK_DEPTH / 2.0 + 0.022, 0.685), "rear_top_molding", WOOD_DARK)
    _add_visual(desk, Box((0.045, DESK_DEPTH, 0.055)), (-DESK_LENGTH / 2.0 + 0.022, 0.0, 0.685), "end_top_molding_0", WOOD_DARK)
    _add_visual(desk, Box((0.045, DESK_DEPTH, 0.055)), (DESK_LENGTH / 2.0 - 0.022, 0.0, 0.685), "end_top_molding_1", WOOD_DARK)

    # A short central modesty/privacy panel fills only the knee-space between the
    # offset pedestals, avoiding the opposing drawer boxes.
    _add_visual(desk, Box((0.280, 0.040, 0.530)), (0.0, 0.0, 0.435), "central_modesty_panel", WOOD)
    _add_visual(desk, Box((0.320, 0.060, 0.040)), (0.0, 0.0, 0.165), "modesty_lower_rail", WOOD_DARK)

    # Two three-drawer pedestals, deliberately facing opposite sides of the desk.
    _add_pedestal_carcass(desk, "front", x_center=-0.44, face_sign=1.0)
    _add_pedestal_carcass(desk, "rear", x_center=0.44, face_sign=-1.0)

    for prefix, x_center, face_sign in (("front", -0.44, 1.0), ("rear", 0.44, -1.0)):
        for level, z_center, height in DRAWER_LAYOUT:
            _build_drawer(model, desk, prefix, level, x_center, z_center, height, face_sign)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    drawer_names = [
        f"{prefix}_{level}_drawer"
        for prefix in ("front", "rear")
        for level, _, _ in DRAWER_LAYOUT
    ]
    slide_names = [
        f"{prefix}_{level}_slide"
        for prefix in ("front", "rear")
        for level, _, _ in DRAWER_LAYOUT
    ]
    ctx.check("six separate sliding drawers", len(drawer_names) == 6 and len(slide_names) == 6)

    desk = object_model.get_part("desk")
    for prefix, face_sign in (("front", 1.0), ("rear", -1.0)):
        for level, _, _ in DRAWER_LAYOUT:
            drawer = object_model.get_part(f"{prefix}_{level}_drawer")
            slide = object_model.get_articulation(f"{prefix}_{level}_slide")
            limits = slide.motion_limits
            ctx.check(
                f"{prefix} {level} drawer has prismatic travel",
                slide.articulation_type == ArticulationType.PRISMATIC
                and limits is not None
                and abs((limits.upper or 0.0) - DRAWER_TRAVEL) < 1e-6,
            )

            # Prove the side runners are mounted on the fixed guide rails at rest.
            ctx.expect_gap(
                desk,
                drawer,
                axis="x",
                positive_elem=f"{prefix}_{level}_rail_pos",
                negative_elem="runner_pos",
                max_gap=0.003,
                max_penetration=0.001,
                name=f"{prefix} {level} positive runner rides guide rail",
            )
            ctx.expect_gap(
                drawer,
                desk,
                axis="x",
                positive_elem="runner_neg",
                negative_elem=f"{prefix}_{level}_rail_neg",
                max_gap=0.003,
                max_penetration=0.001,
                name=f"{prefix} {level} negative runner rides guide rail",
            )
            ctx.expect_overlap(
                drawer,
                desk,
                axes="yz",
                elem_a="runner_pos",
                elem_b=f"{prefix}_{level}_rail_pos",
                min_overlap=0.015,
                name=f"{prefix} {level} runner overlaps rail length",
            )

            rest_pos = ctx.part_world_position(drawer)
            with ctx.pose({slide: DRAWER_TRAVEL}):
                ctx.expect_overlap(
                    drawer,
                    desk,
                    axes="y",
                    elem_a="runner_pos",
                    elem_b=f"{prefix}_{level}_rail_pos",
                    min_overlap=0.160,
                    name=f"{prefix} {level} runner retained when extended",
                )
                extended_pos = ctx.part_world_position(drawer)
            ctx.check(
                f"{prefix} {level} drawer opens outward",
                rest_pos is not None
                and extended_pos is not None
                and face_sign * (extended_pos[1] - rest_pos[1]) > 0.25,
                details=f"rest={rest_pos}, extended={extended_pos}, face_sign={face_sign}",
            )

    return ctx.report()


object_model = build_object_model()
