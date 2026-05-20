from __future__ import annotations

from math import pi

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


BODY_WIDTH = 1.10
BODY_DEPTH = 0.55
BODY_BOTTOM_Z = 0.16
BODY_TOP_Z = 0.91
BODY_HEIGHT = BODY_TOP_Z - BODY_BOTTOM_Z
PANEL_THICKNESS = 0.025
FRONT_Y = -BODY_DEPTH / 2.0
BACK_Y = BODY_DEPTH / 2.0

DRAWER_COUNT = 5
DRAWER_FRONT_WIDTH = 1.04
DRAWER_FRONT_HEIGHT = 0.125
DRAWER_FRONT_THICKNESS = 0.024
DRAWER_FACE_Y = FRONT_Y - 0.018
DRAWER_TRAVEL = 0.42
DRAWER_BOX_WIDTH = 0.96
DRAWER_BOX_DEPTH = 0.46
SLIDE_LENGTH = 0.52
FRONT_DIVIDER_NAMES = (
    "front_divider_0",
    "front_divider_1",
    "front_divider_2",
    "front_divider_3",
    "front_divider_4",
    "front_divider_5",
)
FIXED_SLIDE_NAMES = (
    ("fixed_slide_0_0", "fixed_slide_0_1"),
    ("fixed_slide_1_0", "fixed_slide_1_1"),
    ("fixed_slide_2_0", "fixed_slide_2_1"),
    ("fixed_slide_3_0", "fixed_slide_3_1"),
    ("fixed_slide_4_0", "fixed_slide_4_1"),
)


def _drawer_center_z(index: int) -> float:
    """Bottom-to-top equal front spacing for the five stacked drawers."""

    interior_bottom = BODY_BOTTOM_Z + PANEL_THICKNESS
    vertical_gap = 0.012
    pitch = DRAWER_FRONT_HEIGHT + vertical_gap
    return interior_bottom + vertical_gap + DRAWER_FRONT_HEIGHT / 2.0 + index * pitch


def _add_drawer_visuals(
    drawer,
    *,
    paint: Material,
    dark: Material,
    chrome: Material,
) -> None:
    """A connected shallow steel drawer box with front pull and moving slide rails."""

    wall_t = 0.014
    rail_t = 0.014

    drawer.visual(
        Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
        origin=Origin(),
        material=paint,
        name="front_panel",
    )
    drawer.visual(
        Box((DRAWER_BOX_WIDTH, DRAWER_BOX_DEPTH, 0.012)),
        origin=Origin(xyz=(0.0, 0.012 + DRAWER_BOX_DEPTH / 2.0, -0.050)),
        material=dark,
        name="tray_bottom",
    )
    drawer.visual(
        Box((wall_t, DRAWER_BOX_DEPTH, 0.086)),
        origin=Origin(
            xyz=(
                -(DRAWER_BOX_WIDTH / 2.0 - wall_t / 2.0),
                0.012 + DRAWER_BOX_DEPTH / 2.0,
                -0.007,
            )
        ),
        material=paint,
        name="side_wall_0",
    )
    drawer.visual(
        Box((wall_t, DRAWER_BOX_DEPTH, 0.086)),
        origin=Origin(
            xyz=(
                DRAWER_BOX_WIDTH / 2.0 - wall_t / 2.0,
                0.012 + DRAWER_BOX_DEPTH / 2.0,
                -0.007,
            )
        ),
        material=paint,
        name="side_wall_1",
    )
    drawer.visual(
        Box((DRAWER_BOX_WIDTH, wall_t, 0.086)),
        origin=Origin(
            xyz=(0.0, 0.012 + DRAWER_BOX_DEPTH - wall_t / 2.0, -0.007)
        ),
        material=paint,
        name="rear_wall",
    )

    # The full-extension moving slide members are deliberately longer than the
    # drawer tray so that a useful length remains engaged when the drawer is
    # pulled to its stop.
    drawer.visual(
        Box((rail_t, SLIDE_LENGTH, 0.018)),
        origin=Origin(
            xyz=(
                -(DRAWER_BOX_WIDTH / 2.0 + rail_t / 2.0),
                0.012 + SLIDE_LENGTH / 2.0,
                -0.025,
            )
        ),
        material=chrome,
        name="moving_slide_0",
    )
    drawer.visual(
        Box((rail_t, SLIDE_LENGTH, 0.018)),
        origin=Origin(
            xyz=(
                DRAWER_BOX_WIDTH / 2.0 + rail_t / 2.0,
                0.012 + SLIDE_LENGTH / 2.0,
                -0.025,
            )
        ),
        material=chrome,
        name="moving_slide_1",
    )
    drawer.visual(
        Box((0.017, 0.480, 0.010)),
        origin=Origin(xyz=(-0.5025, 0.012 + 0.240, -0.025)),
        material=chrome,
        name="bearing_cage_0",
    )
    drawer.visual(
        Box((0.017, 0.480, 0.010)),
        origin=Origin(xyz=(0.5025, 0.012 + 0.240, -0.025)),
        material=chrome,
        name="bearing_cage_1",
    )
    for side_name, x in (("0", -0.493), ("1", 0.493)):
        for bead_i, y in enumerate((0.090, 0.185, 0.280, 0.375, 0.470)):
            drawer.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, y, -0.025), rpy=(0.0, pi / 2.0, 0.0)),
                material=chrome,
                name=f"bearing_{side_name}_{bead_i}",
            )

    # Flat bar pull handle mounted proud of the drawer face with two short tabs.
    drawer.visual(
        Box((0.64, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.050, 0.020)),
        material=chrome,
        name="bar_pull",
    )
    for tab_i, x in enumerate((-0.30, 0.30)):
        drawer.visual(
            Box((0.040, 0.040, 0.020)),
            origin=Origin(xyz=(x, -0.030, 0.020)),
            material=chrome,
            name=f"handle_mount_{tab_i}",
        )


def _add_caster_yoke_visuals(
    caster,
    *,
    metal: Material,
    dark: Material,
) -> None:
    """Swiveling caster yoke whose top disc seats against the cabinet bottom."""

    caster.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=metal,
        name="swivel_disc",
    )
    caster.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material=metal,
        name="stem",
    )
    caster.visual(
        Box((0.080, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=metal,
        name="fork_bridge",
    )
    caster.visual(
        Box((0.008, 0.066, 0.104)),
        origin=Origin(xyz=(-0.029, 0.0, -0.108)),
        material=metal,
        name="fork_arm_0",
    )
    caster.visual(
        Box((0.008, 0.066, 0.104)),
        origin=Origin(xyz=(0.029, 0.0, -0.108)),
        material=metal,
        name="fork_arm_1",
    )
    # A dark inner shadow makes the fork read as a U-shaped bracket rather than
    # a solid block, while staying attached to the bridge.
    caster.visual(
        Box((0.036, 0.058, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=dark,
        name="fork_shadow",
    )


def _add_wheel_visuals(wheel, *, rubber: Material, metal: Material) -> None:
    """Rubber caster wheel with a metal hub that reaches the fork arms."""

    wheel.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_tool_chest")

    red = model.material("red_powder_coated_steel", rgba=(0.72, 0.03, 0.02, 1.0))
    dark_red = model.material("dark_drawer_interiors", rgba=(0.20, 0.015, 0.012, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.78, 0.80, 0.78, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.025, 0.025, 0.030, 1.0))

    body = model.part("body")

    # Box-panel steel chest: open-front cabinet body with connected side, top,
    # bottom, back, and front divider rails.
    body.visual(
        Box((PANEL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-(BODY_WIDTH / 2.0 - PANEL_THICKNESS / 2.0), 0.0, BODY_BOTTOM_Z + BODY_HEIGHT / 2.0)),
        material=red,
        name="side_panel_0",
    )
    body.visual(
        Box((PANEL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(BODY_WIDTH / 2.0 - PANEL_THICKNESS / 2.0, 0.0, BODY_BOTTOM_Z + BODY_HEIGHT / 2.0)),
        material=red,
        name="side_panel_1",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z - PANEL_THICKNESS / 2.0)),
        material=red,
        name="top_panel",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + PANEL_THICKNESS / 2.0)),
        material=red,
        name="bottom_panel",
    )
    body.visual(
        Box((BODY_WIDTH, PANEL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, BACK_Y - PANEL_THICKNESS / 2.0, BODY_BOTTOM_Z + BODY_HEIGHT / 2.0)),
        material=red,
        name="back_panel",
    )

    for rail_i, z in enumerate(
        [BODY_BOTTOM_Z + PANEL_THICKNESS + 0.005]
        + [(_drawer_center_z(i) + DRAWER_FRONT_HEIGHT / 2.0 + _drawer_center_z(i + 1) - DRAWER_FRONT_HEIGHT / 2.0) / 2.0 for i in range(DRAWER_COUNT - 1)]
        + [BODY_TOP_Z - PANEL_THICKNESS - 0.005]
    ):
        body.visual(
            Box((BODY_WIDTH - 2.0 * PANEL_THICKNESS, 0.014, 0.010)),
            origin=Origin(xyz=(0.0, FRONT_Y + 0.008, z)),
            material=red,
            name=FRONT_DIVIDER_NAMES[rail_i],
        )

    # Fixed outer ball-bearing slide rails mounted to the inside side panels.
    rail_x_positions = (
        -(BODY_WIDTH / 2.0 - PANEL_THICKNESS - 0.007),
        BODY_WIDTH / 2.0 - PANEL_THICKNESS - 0.007,
    )
    for drawer_i in range(DRAWER_COUNT):
        z = _drawer_center_z(drawer_i) - 0.025
        for side_i, x in enumerate(rail_x_positions):
            body.visual(
                Box((0.014, 0.500, 0.020)),
                origin=Origin(xyz=(x, FRONT_Y + 0.025 + 0.250, z)),
                material=chrome,
                name=FIXED_SLIDE_NAMES[drawer_i][side_i],
            )

    drawers = []
    for drawer_i in range(DRAWER_COUNT):
        drawer = model.part(f"drawer_{drawer_i}")
        _add_drawer_visuals(drawer, paint=red, dark=dark_red, chrome=chrome)
        drawers.append(drawer)
        model.articulation(
            f"body_to_drawer_{drawer_i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, DRAWER_FACE_Y, _drawer_center_z(drawer_i))),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=140.0,
                velocity=0.35,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    caster_positions = (
        (-0.44, -0.205),
        (0.44, -0.205),
        (-0.44, 0.205),
        (0.44, 0.205),
    )
    for caster_i, (x, y) in enumerate(caster_positions):
        caster = model.part(f"caster_{caster_i}")
        _add_caster_yoke_visuals(caster, metal=chrome, dark=shadow)
        model.articulation(
            f"body_to_caster_{caster_i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=caster,
            origin=Origin(xyz=(x, y, BODY_BOTTOM_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=8.0),
        )

        wheel = model.part(f"wheel_{caster_i}")
        _add_wheel_visuals(wheel, rubber=black, metal=chrome)
        model.articulation(
            f"caster_{caster_i}_to_wheel_{caster_i}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.108)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(DRAWER_COUNT)]
    drawer_joints = [
        object_model.get_articulation(f"body_to_drawer_{i}") for i in range(DRAWER_COUNT)
    ]

    ctx.check(
        "five equal stacked drawers",
        len(drawers) == 5,
        details=f"drawer_count={len(drawers)}",
    )
    for i, joint in enumerate(drawer_joints):
        limits = joint.motion_limits
        ctx.check(
            f"drawer_{i} full extension prismatic slide",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == (0.0, -1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper == DRAWER_TRAVEL,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )
        ctx.expect_gap(
            body,
            drawers[i],
            axis="y",
            positive_elem="side_panel_0",
            negative_elem="front_panel",
            min_gap=0.002,
            max_gap=0.012,
            name=f"drawer_{i} front sits just proud of cabinet",
        )
        ctx.expect_overlap(
            drawers[i],
            body,
            axes="x",
            elem_a="front_panel",
            elem_b=FRONT_DIVIDER_NAMES[0],
            min_overlap=0.90,
            name=f"drawer_{i} wide front spans the tool chest",
        )

    rest_position = ctx.part_world_position(drawers[2])
    with ctx.pose({drawer_joints[2]: DRAWER_TRAVEL}):
        extended_position = ctx.part_world_position(drawers[2])
        ctx.expect_overlap(
            drawers[2],
            body,
            axes="y",
            elem_a="moving_slide_1",
            elem_b=FIXED_SLIDE_NAMES[2][1],
            min_overlap=0.055,
            name="middle drawer slide retains insertion at full extension",
        )
    ctx.check(
        "middle drawer extends outward on negative y",
        rest_position is not None
        and extended_position is not None
        and extended_position[1] < rest_position[1] - 0.35,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    for i in range(4):
        caster = object_model.get_part(f"caster_{i}")
        wheel = object_model.get_part(f"wheel_{i}")
        swivel = object_model.get_articulation(f"body_to_caster_{i}")
        axle = object_model.get_articulation(f"caster_{i}_to_wheel_{i}")
        ctx.check(
            f"caster_{i} swivels and wheel rolls",
            swivel.articulation_type == ArticulationType.CONTINUOUS
            and axle.articulation_type == ArticulationType.CONTINUOUS
            and axle.axis == (1.0, 0.0, 0.0),
            details=f"swivel={swivel.articulation_type}, axle={axle.articulation_type}, axis={axle.axis}",
        )
        ctx.expect_contact(
            caster,
            body,
            elem_a="swivel_disc",
            elem_b="bottom_panel",
            name=f"caster_{i} swivel disc is seated under cabinet",
        )
        ctx.expect_contact(
            wheel,
            caster,
            elem_a="hub",
            elem_b="fork_arm_0",
            contact_tol=0.001,
            name=f"wheel_{i} hub is captured by fork arm",
        )

    return ctx.report()


object_model = build_object_model()
