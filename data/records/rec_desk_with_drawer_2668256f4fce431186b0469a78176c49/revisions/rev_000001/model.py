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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="computer_desk_with_keyboard_tray_and_drawer")

    walnut = model.material("walnut", rgba=(0.50, 0.34, 0.22, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.18, 0.20, 1.0))
    black_steel = model.material("black_steel", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.58, 0.60, 0.63, 1.0))

    desk_frame = model.part("desk_frame")
    desk_frame.visual(
        Box((1.28, 0.65, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=walnut,
        name="desktop",
    )

    leg_size = (0.05, 0.05, 0.72)
    for name, x_pos, y_pos in (
        ("left_front_leg", -0.565, 0.245),
        ("right_front_leg", 0.565, 0.245),
        ("left_back_leg", -0.565, -0.245),
        ("right_back_leg", 0.565, -0.245),
    ):
        desk_frame.visual(
            Box(leg_size),
            origin=Origin(xyz=(x_pos, y_pos, 0.36)),
            material=graphite,
            name=name,
        )

    desk_frame.visual(
        Box((1.08, 0.03, 0.11)),
        origin=Origin(xyz=(0.0, -0.255, 0.665)),
        material=graphite,
        name="back_apron",
    )
    desk_frame.visual(
        Box((0.03, 0.46, 0.10)),
        origin=Origin(xyz=(-0.525, 0.0, 0.67)),
        material=graphite,
        name="left_side_apron",
    )
    desk_frame.visual(
        Box((0.03, 0.10, 0.10)),
        origin=Origin(xyz=(0.525, -0.17, 0.67)),
        material=graphite,
        name="right_side_apron",
    )

    desk_frame.visual(
        Box((0.60, 0.04, 0.045)),
        origin=Origin(xyz=(-0.10, 0.29, 0.6975)),
        material=black_steel,
        name="tray_front_mount",
    )
    desk_frame.visual(
        Box((0.60, 0.04, 0.045)),
        origin=Origin(xyz=(-0.10, -0.09, 0.6975)),
        material=black_steel,
        name="tray_rear_mount",
    )
    desk_frame.visual(
        Box((0.012, 0.34, 0.045)),
        origin=Origin(xyz=(-0.372, 0.10, 0.6525)),
        material=satin_metal,
        name="tray_left_outer_runner",
    )
    desk_frame.visual(
        Box((0.012, 0.34, 0.045)),
        origin=Origin(xyz=(0.172, 0.10, 0.6525)),
        material=satin_metal,
        name="tray_right_outer_runner",
    )

    desk_frame.visual(
        Box((0.016, 0.47, 0.15)),
        origin=Origin(xyz=(0.223, 0.045, 0.645)),
        material=graphite,
        name="drawer_housing_left_side",
    )
    desk_frame.visual(
        Box((0.016, 0.47, 0.15)),
        origin=Origin(xyz=(0.553, 0.045, 0.645)),
        material=graphite,
        name="drawer_housing_right_side",
    )
    desk_frame.visual(
        Box((0.346, 0.016, 0.15)),
        origin=Origin(xyz=(0.388, -0.19, 0.645)),
        material=graphite,
        name="drawer_housing_back",
    )
    desk_frame.visual(
        Box((0.346, 0.47, 0.016)),
        origin=Origin(xyz=(0.388, 0.045, 0.562)),
        material=graphite,
        name="drawer_housing_bottom",
    )
    desk_frame.visual(
        Box((0.346, 0.02, 0.04)),
        origin=Origin(xyz=(0.388, 0.27, 0.70)),
        material=black_steel,
        name="drawer_housing_header",
    )
    desk_frame.visual(
        Box((0.006, 0.47, 0.035)),
        origin=Origin(xyz=(0.234, 0.045, 0.627)),
        material=satin_metal,
        name="drawer_left_outer_runner",
    )
    desk_frame.visual(
        Box((0.006, 0.47, 0.035)),
        origin=Origin(xyz=(0.542, 0.045, 0.627)),
        material=satin_metal,
        name="drawer_right_outer_runner",
    )
    desk_frame.inertial = Inertial.from_geometry(
        Box((1.28, 0.65, 0.75)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
    )

    keyboard_tray = model.part("keyboard_tray")
    keyboard_tray.visual(
        Box((0.56, 0.30, 0.018)),
        origin=Origin(xyz=(0.0, 0.215, 0.0)),
        material=graphite,
        name="tray_panel",
    )
    keyboard_tray.visual(
        Box((0.018, 0.30, 0.03)),
        origin=Origin(xyz=(-0.271, 0.215, 0.006)),
        material=black_steel,
        name="tray_left_lip",
    )
    keyboard_tray.visual(
        Box((0.018, 0.30, 0.03)),
        origin=Origin(xyz=(0.271, 0.215, 0.006)),
        material=black_steel,
        name="tray_right_lip",
    )
    keyboard_tray.visual(
        Box((0.58, 0.025, 0.06)),
        origin=Origin(xyz=(0.0, 0.3725, 0.021)),
        material=walnut,
        name="tray_front",
    )
    keyboard_tray.visual(
        Box((0.008, 0.34, 0.014)),
        origin=Origin(xyz=(-0.264, 0.17, 0.028)),
        material=satin_metal,
        name="left_inner_runner",
    )
    keyboard_tray.visual(
        Box((0.008, 0.34, 0.014)),
        origin=Origin(xyz=(0.264, 0.17, 0.028)),
        material=satin_metal,
        name="right_inner_runner",
    )
    keyboard_tray.inertial = Inertial.from_geometry(
        Box((0.58, 0.40, 0.08)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.18, 0.02)),
    )

    model.articulation(
        "desk_to_keyboard_tray",
        ArticulationType.PRISMATIC,
        parent=desk_frame,
        child=keyboard_tray,
        origin=Origin(xyz=(-0.10, -0.06, 0.595)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.30,
            lower=0.0,
            upper=0.22,
        ),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.276, 0.38, 0.012)),
        origin=Origin(xyz=(0.0, 0.19, -0.039)),
        material=graphite,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.012, 0.40, 0.09)),
        origin=Origin(xyz=(-0.144, 0.20, 0.0)),
        material=graphite,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((0.012, 0.40, 0.09)),
        origin=Origin(xyz=(0.144, 0.20, 0.0)),
        material=graphite,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((0.276, 0.012, 0.09)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=graphite,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.33, 0.02, 0.12)),
        origin=Origin(xyz=(0.0, 0.40, 0.0)),
        material=walnut,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.12, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.419, 0.0)),
        material=black_steel,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.006, 0.40, 0.014)),
        origin=Origin(xyz=(-0.148, 0.20, 0.005)),
        material=satin_metal,
        name="drawer_left_inner_runner",
    )
    drawer.visual(
        Box((0.006, 0.40, 0.014)),
        origin=Origin(xyz=(0.148, 0.20, 0.005)),
        material=satin_metal,
        name="drawer_right_inner_runner",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.33, 0.42, 0.12)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.20, 0.0)),
    )

    model.articulation(
        "desk_to_drawer",
        ArticulationType.PRISMATIC,
        parent=desk_frame,
        child=drawer,
        origin=Origin(xyz=(0.388, -0.09, 0.625)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.28,
            lower=0.0,
            upper=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desk = object_model.get_part("desk_frame")
    keyboard_tray = object_model.get_part("keyboard_tray")
    drawer = object_model.get_part("drawer")
    tray_slide = object_model.get_articulation("desk_to_keyboard_tray")
    drawer_slide = object_model.get_articulation("desk_to_drawer")

    tray_upper = tray_slide.motion_limits.upper if tray_slide.motion_limits else 0.22
    drawer_upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits else 0.24

    ctx.expect_gap(
        desk,
        keyboard_tray,
        axis="z",
        positive_elem="tray_left_outer_runner",
        negative_elem="left_inner_runner",
        min_gap=0.0,
        max_gap=0.02,
        name="left keyboard guide runner sits just above the tray rail",
    )
    ctx.expect_gap(
        desk,
        keyboard_tray,
        axis="z",
        positive_elem="tray_right_outer_runner",
        negative_elem="right_inner_runner",
        min_gap=0.0,
        max_gap=0.02,
        name="right keyboard guide runner sits just above the tray rail",
    )
    ctx.expect_overlap(
        keyboard_tray,
        desk,
        axes="y",
        elem_a="left_inner_runner",
        elem_b="tray_left_outer_runner",
        min_overlap=0.30,
        name="keyboard tray remains deeply engaged in its left guide at rest",
    )
    ctx.expect_overlap(
        drawer,
        desk,
        axes="y",
        elem_a="drawer_left_inner_runner",
        elem_b="drawer_left_outer_runner",
        min_overlap=0.36,
        name="drawer remains deeply engaged in its left guide at rest",
    )
    ctx.expect_contact(
        drawer,
        desk,
        elem_a="drawer_left_inner_runner",
        elem_b="drawer_left_outer_runner",
        name="drawer runner physically bears on the cabinet guide",
    )
    ctx.expect_gap(
        drawer,
        keyboard_tray,
        axis="x",
        positive_elem="drawer_front",
        negative_elem="tray_front",
        min_gap=0.02,
        max_gap=0.05,
        name="drawer sits immediately beside the keyboard tray",
    )

    rest_tray_pos = ctx.part_world_position(keyboard_tray)
    rest_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({tray_slide: tray_upper, drawer_slide: drawer_upper}):
        ctx.expect_overlap(
            keyboard_tray,
            desk,
            axes="y",
            elem_a="left_inner_runner",
            elem_b="tray_left_outer_runner",
            min_overlap=0.11,
            name="extended keyboard tray keeps retained insertion in the guide",
        )
        ctx.expect_overlap(
            drawer,
            desk,
            axes="y",
            elem_a="drawer_left_inner_runner",
            elem_b="drawer_left_outer_runner",
            min_overlap=0.12,
            name="extended drawer keeps retained insertion in the guide",
        )
        ctx.expect_gap(
            drawer,
            keyboard_tray,
            axis="x",
            positive_elem="drawer_front",
            negative_elem="tray_front",
            min_gap=0.02,
            max_gap=0.05,
            name="drawer clears the tray when both slides are extended",
        )
        extended_tray_pos = ctx.part_world_position(keyboard_tray)
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "keyboard tray extends forward",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] > rest_tray_pos[1] + 0.18,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )
    ctx.check(
        "drawer extends forward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > rest_drawer_pos[1] + 0.20,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
