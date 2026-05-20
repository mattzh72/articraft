from __future__ import annotations

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

    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    body_mid = model.material("body_mid", rgba=(0.29, 0.30, 0.33, 1.0))
    drawer_front = model.material("drawer_front", rgba=(0.38, 0.39, 0.42, 1.0))
    scale_metal = model.material("scale_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    display_housing = model.material("display_housing", rgba=(0.22, 0.23, 0.25, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.20, 0.40, 0.46, 0.45))
    key_cream = model.material("key_cream", rgba=(0.93, 0.91, 0.84, 1.0))
    key_mint = model.material("key_mint", rgba=(0.78, 0.88, 0.78, 1.0))
    key_blue = model.material("key_blue", rgba=(0.76, 0.84, 0.94, 1.0))
    key_rose = model.material("key_rose", rgba=(0.92, 0.80, 0.78, 1.0))
    key_colors = (key_cream, key_mint, key_blue, key_rose)

    base_depth = 0.460
    base_width = 0.440
    base_height = 0.136
    wall = 0.012
    floor_t = 0.014
    top_t = 0.014
    side_height = base_height - floor_t

    base = model.part("base")
    base.visual(
        Box((base_depth, base_width, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t * 0.5)),
        material=body_dark,
        name="bottom_shell",
    )
    base.visual(
        Box((base_depth - wall, wall, side_height)),
        origin=Origin(
            xyz=(
                -wall * 0.5,
                base_width * 0.5 - wall * 0.5,
                floor_t + side_height * 0.5,
            )
        ),
        material=body_mid,
        name="side_wall_0",
    )
    base.visual(
        Box((base_depth - wall, wall, side_height)),
        origin=Origin(
            xyz=(
                -wall * 0.5,
                -base_width * 0.5 + wall * 0.5,
                floor_t + side_height * 0.5,
            )
        ),
        material=body_mid,
        name="side_wall_1",
    )
    base.visual(
        Box((wall, base_width - 2.0 * wall, side_height)),
        origin=Origin(
            xyz=(
                -base_depth * 0.5 + wall * 0.5,
                0.0,
                floor_t + side_height * 0.5,
            )
        ),
        material=body_mid,
        name="rear_wall",
    )
    base.visual(
        Box((base_depth, base_width, top_t)),
        origin=Origin(xyz=(0.0, 0.0, base_height - top_t * 0.5)),
        material=body_mid,
        name="top_shell",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.016, 0.406, 0.086)),
        origin=Origin(xyz=(0.008, 0.0, 0.043)),
        material=drawer_front,
        name="front_panel",
    )
    drawer.visual(
        Box((0.315, 0.392, 0.005)),
        origin=Origin(xyz=(-0.1575, 0.0, 0.0025)),
        material=body_dark,
        name="tray_bottom",
    )
    drawer.visual(
        Box((0.315, 0.006, 0.058)),
        origin=Origin(xyz=(-0.1575, 0.193, 0.029)),
        material=body_dark,
        name="tray_wall_0",
    )
    drawer.visual(
        Box((0.315, 0.006, 0.058)),
        origin=Origin(xyz=(-0.1575, -0.193, 0.029)),
        material=body_dark,
        name="tray_wall_1",
    )
    drawer.visual(
        Box((0.006, 0.392, 0.058)),
        origin=Origin(xyz=(-0.312, 0.0, 0.029)),
        material=body_dark,
        name="rear_panel",
    )
    drawer.visual(
        Box((0.008, 0.155, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, 0.044)),
        material=scale_metal,
        name="pull",
    )
    model.articulation(
        "base_to_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(base_depth * 0.5, 0.0, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=0.0,
            upper=0.180,
        ),
    )

    platter = model.part("platter")
    platter.visual(
        Box((0.214, 0.182, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=scale_metal,
        name="plate",
    )
    platter.visual(
        Box((0.214, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.087, 0.013)),
        material=scale_metal,
        name="lip_0",
    )
    platter.visual(
        Box((0.214, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, -0.087, 0.013)),
        material=scale_metal,
        name="lip_1",
    )
    platter.visual(
        Box((0.008, 0.166, 0.006)),
        origin=Origin(xyz=(0.103, 0.0, 0.013)),
        material=scale_metal,
        name="lip_2",
    )
    platter.visual(
        Box((0.008, 0.166, 0.006)),
        origin=Origin(xyz=(-0.103, 0.0, 0.013)),
        material=scale_metal,
        name="lip_3",
    )
    model.articulation(
        "base_to_platter",
        ArticulationType.FIXED,
        parent=base,
        child=platter,
        origin=Origin(xyz=(-0.020, -0.105, base_height)),
    )

    tower = model.part("tower")
    tower.visual(
        Box((0.235, 0.195, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=body_mid,
        name="pedestal",
    )
    tower.visual(
        Box((0.195, 0.165, 0.036)),
        origin=Origin(xyz=(-0.012, 0.0, 0.064)),
        material=body_dark,
        name="upper_deck",
    )
    tower.visual(
        Box((0.190, 0.160, 0.003)),
        origin=Origin(xyz=(-0.012, 0.0, 0.0835)),
        material=body_mid,
        name="key_plate",
    )
    model.articulation(
        "base_to_tower",
        ArticulationType.FIXED,
        parent=base,
        child=tower,
        origin=Origin(xyz=(-0.015, 0.090, base_height)),
    )

    key_rows = (0.042, 0.014, -0.014, -0.042)
    key_cols = (-0.062, -0.037, -0.012, 0.013, 0.038, 0.063)
    for row_index, key_x in enumerate(key_rows):
        for col_index, key_y in enumerate(key_cols):
            key_part = model.part(f"key_{row_index}_{col_index}")
            key_material = key_colors[row_index % len(key_colors)]
            key_part.visual(
                Box((0.018, 0.014, 0.003)),
                origin=Origin(xyz=(0.0, 0.0, 0.0025)),
                material=body_mid,
                name="plunger",
            )
            key_part.visual(
                Box((0.024, 0.020, 0.007)),
                origin=Origin(xyz=(0.0, 0.0, 0.0075)),
                material=key_material,
                name="cap",
            )
            model.articulation(
                f"tower_to_key_{row_index}_{col_index}",
                ArticulationType.PRISMATIC,
                parent=tower,
                child=key_part,
                origin=Origin(xyz=(key_x, key_y, 0.084)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=2.5,
                    velocity=0.04,
                    lower=0.0,
                    upper=0.001,
                ),
            )

    neck = model.part("neck")
    neck.visual(
        Box((0.040, 0.055, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=body_dark,
        name="foot",
    )
    neck.visual(
        Cylinder(radius=0.013, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=body_mid,
        name="post",
    )
    neck.visual(
        Cylinder(radius=0.007, length=0.064),
        origin=Origin(xyz=(-0.004, 0.0, 0.051), rpy=(0.0, 0.191, 0.0)),
        material=body_mid,
        name="brace",
    )
    neck.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(0.002, 0.0, 0.082), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=body_mid,
        name="hinge_barrel",
    )
    model.articulation(
        "tower_to_neck",
        ArticulationType.FIXED,
        parent=tower,
        child=neck,
        origin=Origin(xyz=(-0.090, 0.0, 0.085)),
    )

    display = model.part("display")
    display.visual(
        Box((0.010, 0.190, 0.110)),
        origin=Origin(xyz=(-0.009, 0.0, 0.055)),
        material=display_housing,
        name="bezel",
    )
    display.visual(
        Box((0.022, 0.160, 0.082)),
        origin=Origin(xyz=(-0.020, 0.0, 0.055)),
        material=display_housing,
        name="rear_body",
    )
    display.visual(
        Box((0.002, 0.166, 0.086)),
        origin=Origin(xyz=(-0.013, 0.0, 0.058)),
        material=screen_glass,
        name="screen",
    )
    model.articulation(
        "neck_to_display",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=display,
        origin=Origin(xyz=(0.002, 0.0, 0.082)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.25,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    drawer = object_model.get_part("drawer")
    tower = object_model.get_part("tower")
    display = object_model.get_part("display")
    key_0_0 = object_model.get_part("key_0_0")
    key_0_1 = object_model.get_part("key_0_1")

    drawer_joint = object_model.get_articulation("base_to_drawer")
    display_joint = object_model.get_articulation("neck_to_display")
    key_joint = object_model.get_articulation("tower_to_key_0_0")

    ctx.expect_overlap(
        drawer,
        base,
        axes="yz",
        min_overlap=0.08,
        name="drawer aligns with the base opening",
    )
    ctx.expect_overlap(
        key_0_0,
        tower,
        axes="xy",
        elem_a="cap",
        elem_b="key_plate",
        min_overlap=0.018,
        name="department key sits over the upper deck",
    )

    drawer_limits = drawer_joint.motion_limits
    if drawer_limits is not None and drawer_limits.upper is not None:
        drawer_rest = ctx.part_world_position(drawer)
        drawer_extended = None
        with ctx.pose({drawer_joint: drawer_limits.upper}):
            drawer_extended = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                base,
                axes="x",
                min_overlap=0.12,
                name="drawer retains insertion at full extension",
            )
            ctx.expect_overlap(
                drawer,
                base,
                axes="yz",
                min_overlap=0.08,
                name="drawer remains guided when extended",
            )
        ctx.check(
            "drawer slides out forward",
            drawer_rest is not None
            and drawer_extended is not None
            and drawer_extended[0] > drawer_rest[0] + 0.10,
            details=f"rest={drawer_rest}, extended={drawer_extended}",
        )

    display_limits = display_joint.motion_limits
    if display_limits is not None and display_limits.upper is not None:
        rest_aabb = ctx.part_world_aabb(display)
        tilted_aabb = None
        with ctx.pose({display_joint: display_limits.upper}):
            tilted_aabb = ctx.part_world_aabb(display)
        ctx.check(
            "display tilts on the neck hinge",
            rest_aabb is not None
            and tilted_aabb is not None
            and tilted_aabb[1][0] > rest_aabb[1][0] + 0.040,
            details=f"rest={rest_aabb}, tilted={tilted_aabb}",
        )

    key_limits = key_joint.motion_limits
    if key_limits is not None and key_limits.upper is not None:
        key_rest = ctx.part_world_position(key_0_0)
        neighbor_rest = ctx.part_world_position(key_0_1)
        key_pressed = None
        neighbor_pressed = None
        with ctx.pose({key_joint: key_limits.upper}):
            key_pressed = ctx.part_world_position(key_0_0)
            neighbor_pressed = ctx.part_world_position(key_0_1)
        ctx.check(
            "department key depresses downward",
            key_rest is not None
            and key_pressed is not None
            and key_pressed[2] < key_rest[2] - 0.0008,
            details=f"rest={key_rest}, pressed={key_pressed}",
        )
        ctx.check(
            "adjacent department key stays independent",
            neighbor_rest is not None
            and neighbor_pressed is not None
            and abs(neighbor_pressed[2] - neighbor_rest[2]) < 1e-6,
            details=f"rest={neighbor_rest}, posed={neighbor_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
