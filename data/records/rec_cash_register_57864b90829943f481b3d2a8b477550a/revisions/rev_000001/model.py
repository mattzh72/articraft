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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_checkout_register")

    body = model.material("body", rgba=(0.18, 0.20, 0.22, 1.0))
    trim = model.material("trim", rgba=(0.38, 0.40, 0.43, 1.0))
    drawer_front = model.material("drawer_front", rgba=(0.23, 0.24, 0.26, 1.0))
    keycap = model.material("keycap", rgba=(0.07, 0.08, 0.09, 1.0))
    scale_metal = model.material("scale_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    glass = model.material("glass", rgba=(0.33, 0.53, 0.63, 0.45))

    base = model.part("base")
    base.visual(
        Box((0.38, 0.44, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=body,
        name="cabinet_floor",
    )
    base.visual(
        Box((0.38, 0.44, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        material=body,
        name="cabinet_roof",
    )
    base.visual(
        Box((0.38, 0.018, 0.088)),
        origin=Origin(xyz=(0.0, -0.211, 0.062)),
        material=body,
        name="cabinet_left",
    )
    base.visual(
        Box((0.38, 0.018, 0.088)),
        origin=Origin(xyz=(0.0, 0.211, 0.062)),
        material=body,
        name="cabinet_right",
    )
    base.visual(
        Box((0.018, 0.404, 0.088)),
        origin=Origin(xyz=(-0.181, 0.0, 0.062)),
        material=body,
        name="cabinet_rear",
    )
    base.visual(
        Box((0.20, 0.22, 0.025)),
        origin=Origin(xyz=(0.05, 0.10, 0.1265)),
        material=body,
        name="keyboard_deck",
    )
    base.visual(
        Box((0.15, 0.17, 0.016)),
        origin=Origin(xyz=(0.035, -0.125, 0.122)),
        material=trim,
        name="scale_riser",
    )
    base.visual(
        Box((0.138, 0.158, 0.008)),
        origin=Origin(xyz=(0.035, -0.125, 0.134)),
        material=scale_metal,
        name="scale_platter",
    )
    base.visual(
        Box((0.16, 0.05, 0.048)),
        origin=Origin(xyz=(-0.08, -0.175, 0.138)),
        material=body,
        name="bay_left",
    )
    base.visual(
        Box((0.16, 0.05, 0.048)),
        origin=Origin(xyz=(-0.08, 0.175, 0.138)),
        material=body,
        name="bay_right",
    )
    base.visual(
        Box((0.014, 0.284, 0.048)),
        origin=Origin(xyz=(-0.007, 0.0, 0.138)),
        material=body,
        name="bay_front",
    )
    base.visual(
        Box((0.018, 0.284, 0.036)),
        origin=Origin(xyz=(-0.171, 0.0, 0.132)),
        material=body,
        name="bay_rear",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.098),
        origin=Origin(xyz=(-0.196, 0.0, 0.211)),
        material=trim,
        name="neck",
    )
    base.visual(
        Cylinder(radius=0.025, length=0.024),
        origin=Origin(xyz=(-0.196, 0.0, 0.162)),
        material=trim,
        name="neck_collar",
    )

    key_rows = 5
    key_cols = 4
    for row in range(key_rows):
        for col in range(key_cols):
            key_x = -0.005 + row * 0.034
            key_y = 0.025 + col * 0.043
            base.visual(
                Box((0.022, 0.028, 0.004)),
                origin=Origin(xyz=(key_x, key_y, 0.1395)),
                material=keycap,
                name=f"key_{row}_{col}",
            )

    base.visual(
        Box((0.050, 0.040, 0.004)),
        origin=Origin(xyz=(0.118, 0.162, 0.1395)),
        material=keycap,
        name="key_wide",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.31, 0.36, 0.075)),
        origin=Origin(xyz=(0.155, 0.0, 0.0375)),
        material=trim,
        name="drawer_box",
    )
    drawer.visual(
        Box((0.018, 0.38, 0.082)),
        origin=Origin(xyz=(0.301, 0.0, 0.041)),
        material=drawer_front,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.012, 0.012, 0.020)),
        origin=Origin(xyz=(0.307, -0.038, 0.045)),
        material=trim,
        name="handle_post_0",
    )
    drawer.visual(
        Box((0.012, 0.012, 0.020)),
        origin=Origin(xyz=(0.307, 0.038, 0.045)),
        material=trim,
        name="handle_post_1",
    )
    drawer.visual(
        Cylinder(radius=0.007, length=0.120),
        origin=Origin(xyz=(0.315, 0.0, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="handle_bar",
    )

    receipt_cover = model.part("receipt_cover")
    receipt_cover.visual(
        Box((0.172, 0.304, 0.012)),
        origin=Origin(xyz=(0.086, 0.0, -0.006)),
        material=body,
        name="cover_panel",
    )
    receipt_cover.visual(
        Box((0.172, 0.008, 0.028)),
        origin=Origin(xyz=(0.086, -0.148, -0.014)),
        material=body,
        name="cover_left",
    )
    receipt_cover.visual(
        Box((0.172, 0.008, 0.028)),
        origin=Origin(xyz=(0.086, 0.148, -0.014)),
        material=body,
        name="cover_right",
    )
    receipt_cover.visual(
        Box((0.008, 0.304, 0.028)),
        origin=Origin(xyz=(0.173, 0.0, -0.014)),
        material=body,
        name="cover_lip",
    )
    receipt_cover.visual(
        Cylinder(radius=0.006, length=0.276),
        origin=Origin(xyz=(0.0, 0.0, -0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="cover_barrel",
    )

    rear_display = model.part("rear_display")
    rear_display.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(xyz=(0.0, -0.056, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="hinge_cap_0",
    )
    rear_display.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(xyz=(0.0, 0.056, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="hinge_cap_1",
    )
    rear_display.visual(
        Box((0.018, 0.146, 0.014)),
        origin=Origin(xyz=(0.009, 0.0, 0.007)),
        material=trim,
        name="hinge_block",
    )
    rear_display.visual(
        Box((0.030, 0.160, 0.104)),
        origin=Origin(xyz=(0.017, 0.0, 0.060)),
        material=body,
        name="display_shell",
    )
    rear_display.visual(
        Box((0.004, 0.132, 0.078)),
        origin=Origin(xyz=(0.031, 0.0, 0.065)),
        material=glass,
        name="screen_glass",
    )

    model.articulation(
        "base_to_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(-0.12, 0.0, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=0.0,
            upper=0.14,
        ),
    )
    model.articulation(
        "base_to_receipt_cover",
        ArticulationType.REVOLUTE,
        parent=base,
        child=receipt_cover,
        origin=Origin(xyz=(-0.168, 0.0, 0.174)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=0.0,
            upper=1.28,
        ),
    )
    model.articulation(
        "base_to_rear_display",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rear_display,
        origin=Origin(xyz=(-0.196, 0.0, 0.260)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=-0.45,
            upper=0.35,
        ),
    )

    return model


def _center_x(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][0] + aabb[1][0])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    drawer = object_model.get_part("drawer")
    receipt_cover = object_model.get_part("receipt_cover")
    rear_display = object_model.get_part("rear_display")

    drawer_slide = object_model.get_articulation("base_to_drawer")
    cover_hinge = object_model.get_articulation("base_to_receipt_cover")
    display_tilt = object_model.get_articulation("base_to_rear_display")

    drawer_limits = drawer_slide.motion_limits
    cover_limits = cover_hinge.motion_limits
    display_limits = display_tilt.motion_limits

    closed_drawer_face = ctx.part_element_world_aabb(drawer, elem="drawer_face")
    cabinet_roof = ctx.part_element_world_aabb(base, elem="cabinet_roof")
    drawer_flush = (
        closed_drawer_face is not None
        and cabinet_roof is not None
        and abs(closed_drawer_face[1][0] - cabinet_roof[1][0]) <= 0.002
    )
    ctx.check(
        "drawer front sits flush when closed",
        drawer_flush,
        details=f"drawer_face={closed_drawer_face}, cabinet_roof={cabinet_roof}",
    )

    if drawer_limits is not None and drawer_limits.upper is not None:
        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            extended_pos = ctx.part_world_position(drawer)
            ctx.expect_within(
                drawer,
                base,
                axes="yz",
                margin=0.0,
                name="drawer stays centered in the cabinet opening",
            )
            ctx.expect_overlap(
                drawer,
                base,
                axes="x",
                min_overlap=0.16,
                name="drawer remains retained inside the base at full extension",
            )

        extends_forward = (
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.10
        )
        ctx.check(
            "drawer extends forward",
            extends_forward,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_gap(
            receipt_cover,
            base,
            axis="z",
            positive_elem="cover_panel",
            negative_elem="bay_front",
            max_gap=0.004,
            max_penetration=1e-6,
            name="receipt cover seats onto the printer bay",
        )

    if cover_limits is not None and cover_limits.upper is not None:
        closed_cover = None
        open_cover = None
        with ctx.pose({cover_hinge: 0.0}):
            closed_cover = ctx.part_element_world_aabb(receipt_cover, elem="cover_panel")
        with ctx.pose({cover_hinge: cover_limits.upper}):
            open_cover = ctx.part_element_world_aabb(receipt_cover, elem="cover_panel")
        cover_opens = (
            closed_cover is not None
            and open_cover is not None
            and open_cover[1][2] > closed_cover[1][2] + 0.10
        )
        ctx.check(
            "receipt cover opens upward",
            cover_opens,
            details=f"closed={closed_cover}, open={open_cover}",
        )

    if display_limits is not None and display_limits.lower is not None and display_limits.upper is not None:
        low_glass = None
        high_glass = None
        with ctx.pose({display_tilt: display_limits.lower}):
            low_glass = ctx.part_element_world_aabb(rear_display, elem="screen_glass")
        with ctx.pose({display_tilt: display_limits.upper}):
            high_glass = ctx.part_element_world_aabb(rear_display, elem="screen_glass")
        glass_low_x = _center_x(low_glass)
        glass_high_x = _center_x(high_glass)
        ctx.check(
            "rear display tilts forward on its hinge",
            glass_low_x is not None and glass_high_x is not None and glass_high_x > glass_low_x + 0.012,
            details=f"low={low_glass}, high={high_glass}",
        )

    return ctx.report()


object_model = build_object_model()
