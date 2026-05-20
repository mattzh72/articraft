from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _extent(aabb, axis: str) -> float:
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    return aabb[1][axis_index] - aabb[0][axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_checkout_register")

    body_dark = model.material("body_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    body_mid = model.material("body_mid", rgba=(0.29, 0.31, 0.34, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    drawer_face = model.material("drawer_face", rgba=(0.33, 0.35, 0.38, 1.0))
    key_light = model.material("key_light", rgba=(0.85, 0.87, 0.89, 1.0))
    scale_metal = model.material("scale_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    display_glass = model.material("display_glass", rgba=(0.22, 0.40, 0.46, 0.55))
    key_metal = model.material("key_metal", rgba=(0.68, 0.70, 0.73, 1.0))
    key_accent = model.material("key_accent", rgba=(0.90, 0.72, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.44, 0.38, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body_dark,
        name="floor",
    )
    base.visual(
        Box((0.36, 0.032, 0.108)),
        origin=Origin(xyz=(-0.03, -0.174, 0.066)),
        material=body_mid,
        name="left_wall",
    )
    base.visual(
        Box((0.36, 0.032, 0.108)),
        origin=Origin(xyz=(-0.03, 0.174, 0.066)),
        material=body_mid,
        name="right_wall",
    )
    base.visual(
        Box((0.032, 0.316, 0.104)),
        origin=Origin(xyz=(-0.204, 0.0, 0.064)),
        material=body_mid,
        name="rear_wall",
    )
    base.visual(
        Box((0.44, 0.38, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=body_mid,
        name="top_deck",
    )
    base.visual(
        Box((0.022, 0.38, 0.020)),
        origin=Origin(xyz=(0.189, 0.0, 0.116)),
        material=trim_dark,
        name="drawer_slot_header",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.35, 0.30, 0.085)),
        origin=Origin(xyz=(0.175, 0.0, 0.0425)),
        material=body_dark,
        name="tray",
    )
    drawer.visual(
        Box((0.012, 0.35, 0.100)),
        origin=Origin(xyz=(0.356, 0.0, 0.050)),
        material=drawer_face,
        name="front_panel",
    )
    drawer.visual(
        Box((0.016, 0.105, 0.022)),
        origin=Origin(xyz=(0.364, 0.0, 0.054)),
        material=trim_dark,
        name="pull",
    )
    model.articulation(
        "base_to_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(-0.150, 0.0, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=0.160,
        ),
    )

    scale_plate = model.part("scale_plate")
    scale_plate.visual(
        Box((0.17, 0.17, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=scale_metal,
        name="plate",
    )
    scale_plate.visual(
        Box((0.148, 0.148, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
        material=trim_dark,
        name="plate_inset",
    )
    model.articulation(
        "base_to_scale_plate",
        ArticulationType.FIXED,
        parent=base,
        child=scale_plate,
        origin=Origin(xyz=(-0.110, -0.110, 0.132)),
    )

    console = model.part("console")
    console.visual(
        Box((0.24, 0.21, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=body_mid,
        name="lower_body",
    )
    console.visual(
        Box((0.17, 0.18, 0.068)),
        origin=Origin(xyz=(-0.048, 0.0, 0.124)),
        material=body_dark,
        name="upper_step",
    )
    console.visual(
        Box((0.19, 0.15, 0.016)),
        origin=Origin(xyz=(0.040, 0.0, 0.095), rpy=(0.0, -0.30, 0.0)),
        material=body_dark,
        name="keyboard_deck",
    )
    console.visual(
        Box((0.17, 0.13, 0.006)),
        origin=Origin(xyz=(0.038, 0.0, 0.104), rpy=(0.0, -0.30, 0.0)),
        material=trim_dark,
        name="key_field",
    )
    for row_index, y_pos in enumerate((-0.040, 0.0, 0.040)):
        for col_index, x_pos in enumerate((-0.015, 0.020, 0.055, 0.090)):
            console.visual(
                Box((0.028, 0.022, 0.009)),
                origin=Origin(
                    xyz=(x_pos, y_pos, 0.1065),
                    rpy=(0.0, -0.30, 0.0),
                ),
                material=key_light,
                name=f"key_{row_index}_{col_index}",
            )
    console.visual(
        Cylinder(radius=0.018, length=0.075),
        origin=Origin(xyz=(-0.095, 0.0, 0.195)),
        material=body_mid,
        name="neck",
    )
    console.visual(
        Box((0.050, 0.020, 0.034)),
        origin=Origin(xyz=(-0.006, 0.090, 0.153)),
        material=body_mid,
        name="switch_boss",
    )
    console.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(
            xyz=(-0.095, 0.0, 0.2325),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_dark,
        name="display_hinge_barrel",
    )
    model.articulation(
        "base_to_console",
        ArticulationType.FIXED,
        parent=base,
        child=console,
        origin=Origin(xyz=(0.100, 0.085, 0.132)),
    )

    display = model.part("display")
    display.visual(
        Box((0.042, 0.180, 0.075)),
        origin=Origin(xyz=(-0.027, 0.0, 0.0375)),
        material=body_mid,
        name="housing",
    )
    display.visual(
        Box((0.004, 0.150, 0.052)),
        origin=Origin(xyz=(-0.046, 0.0, 0.040)),
        material=display_glass,
        name="screen",
    )
    display.visual(
        Box((0.018, 0.180, 0.014)),
        origin=Origin(xyz=(-0.015, 0.0, 0.068)),
        material=trim_dark,
        name="visor",
    )
    model.articulation(
        "console_to_display",
        ArticulationType.REVOLUTE,
        parent=console,
        child=display,
        origin=Origin(xyz=(-0.095, 0.0, 0.2325)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.25,
            upper=0.45,
        ),
    )

    mode_key = model.part("mode_key")
    mode_key.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.003, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=key_metal,
        name="bezel",
    )
    mode_key.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(
            xyz=(0.0, 0.009, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_dark,
        name="shaft",
    )
    mode_key.visual(
        Box((0.018, 0.008, 0.018)),
        origin=Origin(xyz=(0.010, 0.020, 0.0)),
        material=key_accent,
        name="key_head",
    )
    mode_key.visual(
        Box((0.006, 0.004, 0.026)),
        origin=Origin(xyz=(0.018, 0.024, 0.0)),
        material=key_metal,
        name="key_blade",
    )
    model.articulation(
        "console_to_mode_key",
        ArticulationType.CONTINUOUS,
        parent=console,
        child=mode_key,
        origin=Origin(xyz=(-0.006, 0.100, 0.153)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    drawer = object_model.get_part("drawer")
    console = object_model.get_part("console")
    display = object_model.get_part("display")
    mode_key = object_model.get_part("mode_key")

    drawer_slide = object_model.get_articulation("base_to_drawer")
    display_hinge = object_model.get_articulation("console_to_display")
    key_joint = object_model.get_articulation("console_to_mode_key")

    ctx.expect_within(
        drawer,
        base,
        axes="yz",
        margin=0.0,
        name="drawer stays guided within the cabinet opening",
    )
    ctx.expect_overlap(
        drawer,
        base,
        axes="x",
        min_overlap=0.18,
        name="drawer is inserted into the cabinet at rest",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.160}):
        ctx.expect_within(
            drawer,
            base,
            axes="yz",
            margin=0.0,
            name="extended drawer remains laterally aligned with the cabinet",
        )
        ctx.expect_overlap(
            drawer,
            base,
            axes="x",
            min_overlap=0.18,
            name="extended drawer keeps retained insertion",
        )
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "drawer extends forward",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.12,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    ctx.expect_origin_gap(
        console,
        base,
        axis="z",
        min_gap=0.12,
        name="console is stacked above the drawer base",
    )

    key_field_aabb = ctx.part_element_world_aabb(console, elem="key_field")
    switch_aabb = ctx.part_world_aabb(mode_key)
    ctx.check(
        "mode key switch stays separate from the keyboard field",
        key_field_aabb is not None
        and switch_aabb is not None
        and switch_aabb[0][1] > key_field_aabb[1][1] + 0.03,
        details=f"key_field={key_field_aabb}, mode_key={switch_aabb}",
    )

    display_limits = display_hinge.motion_limits
    display_lower = display_limits.lower if display_limits is not None else None
    display_upper = display_limits.upper if display_limits is not None else None
    lower_screen = None
    upper_screen = None
    if display_lower is not None:
        with ctx.pose({display_hinge: display_lower}):
            lower_screen = ctx.part_element_world_aabb(display, elem="screen")
    if display_upper is not None:
        with ctx.pose({display_hinge: display_upper}):
            upper_screen = ctx.part_element_world_aabb(display, elem="screen")
    ctx.check(
        "rear display visibly tilts on the neck hinge",
        lower_screen is not None
        and upper_screen is not None
        and upper_screen[1][0] > lower_screen[1][0] + 0.02,
        details=f"lower={lower_screen}, upper={upper_screen}",
    )

    blade_upright = None
    blade_turned = None
    with ctx.pose({key_joint: 0.0}):
        blade_upright = ctx.part_element_world_aabb(mode_key, elem="key_blade")
    with ctx.pose({key_joint: math.pi / 2.0}):
        blade_turned = ctx.part_element_world_aabb(mode_key, elem="key_blade")
    ctx.check(
        "mode key rotates on its shaft",
        blade_upright is not None
        and blade_turned is not None
        and _extent(blade_upright, "z") > _extent(blade_upright, "x")
        and _extent(blade_turned, "x") > _extent(blade_turned, "z"),
        details=f"upright={blade_upright}, turned={blade_turned}",
    )

    return ctx.report()


object_model = build_object_model()
