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


CABINET_WIDTH = 1.18
CABINET_DEPTH = 0.18
CABINET_HEIGHT = 0.36
WALL_THICKNESS = 0.012
BACK_THICKNESS = 0.008
DOOR_THICKNESS = 0.022

DRAWER_COUNT = 20
DRAWER_WIDTH = 0.0505
DRAWER_DEPTH = 0.120
DRAWER_HEIGHT = 0.150
DRAWER_WALL = 0.0035
DRAWER_BASE_Z = 0.094
DRAWER_JOINT_Y = 0.026
DRAWER_TRAVEL = 0.055

INNER_MARGIN_X = 0.016
DIVIDER_THICKNESS = 0.003
PIN_RADIUS = 0.0018
PIN_LENGTH = 0.094
PIN_START_Y = BACK_THICKNESS


def _drawer_part_name(index: int) -> str:
    return f"mini_drawer_{index + 1:02d}"


def _drawer_joint_name(index: int) -> str:
    return f"{_drawer_part_name(index)}_slide"


def _drawer_center_x(index: int) -> float:
    usable_span = CABINET_WIDTH - (2.0 * WALL_THICKNESS) - (2.0 * INNER_MARGIN_X)
    slot_pitch = usable_span / DRAWER_COUNT
    return -0.5 * usable_span + slot_pitch * (index + 0.5)


def _build_mini_drawer(part, *, drawer_width: float, body_material, trim_material, label_material) -> None:
    bottom_depth = DRAWER_DEPTH - 0.006
    wall_depth = DRAWER_DEPTH - 0.006

    part.visual(
        Box((drawer_width - 0.004, bottom_depth, 0.004)),
        origin=Origin(xyz=(0.0, 0.5 * bottom_depth + 0.004, 0.002)),
        material=body_material,
        name="drawer_bottom",
    )
    part.visual(
        Box((DRAWER_WALL, wall_depth, 0.070)),
        origin=Origin(
            xyz=(-(drawer_width * 0.5) + (DRAWER_WALL * 0.5), 0.5 * wall_depth + 0.004, 0.035)
        ),
        material=body_material,
        name="left_side",
    )
    part.visual(
        Box((DRAWER_WALL, wall_depth, 0.070)),
        origin=Origin(
            xyz=((drawer_width * 0.5) - (DRAWER_WALL * 0.5), 0.5 * wall_depth + 0.004, 0.035)
        ),
        material=body_material,
        name="right_side",
    )
    part.visual(
        Box((drawer_width, 0.004, 0.082)),
        origin=Origin(xyz=(0.0, 0.002, 0.041)),
        material=body_material,
        name="rear_wall",
    )
    part.visual(
        Box((drawer_width, 0.005, 0.092)),
        origin=Origin(xyz=(0.0, DRAWER_DEPTH - 0.0025, 0.046)),
        material=body_material,
        name="front_panel",
    )
    part.visual(
        Box((drawer_width * 0.52, 0.0025, 0.020)),
        origin=Origin(xyz=(0.0, DRAWER_DEPTH + 0.001, 0.052)),
        material=label_material,
        name="label_strip",
    )
    part.visual(
        Cylinder(radius=0.0038, length=drawer_width * 0.36),
        origin=Origin(
            xyz=(0.0, DRAWER_DEPTH + 0.005, 0.050),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_material,
        name="pull_bar",
    )
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        part.visual(
            Box((0.006, 0.082, 0.008)),
            origin=Origin(
                xyz=(side_sign * ((drawer_width * 0.5) - 0.001), 0.050, 0.008),
            ),
            material=trim_material,
            name=f"guide_shoe_{side_name}",
        )
    part.visual(
        Cylinder(radius=0.0022, length=0.046),
        origin=Origin(xyz=(0.0, 0.032, 0.027)),
        material=trim_material,
        name="hook_stem",
    )
    part.visual(
        Cylinder(radius=0.0018, length=0.012),
        origin=Origin(
            xyz=(0.005, 0.032, 0.050),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_material,
        name="hook_tip",
    )
    part.inertial = Inertial.from_geometry(
        Box((drawer_width + 0.010, DRAWER_DEPTH + 0.014, DRAWER_HEIGHT)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.5 * DRAWER_DEPTH, 0.5 * DRAWER_HEIGHT)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_key_cabinet")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.29, 0.31, 0.34, 1.0))
    drawer_paint = model.material("drawer_paint", rgba=(0.84, 0.82, 0.76, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    label_cream = model.material("label_cream", rgba=(0.94, 0.92, 0.85, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.58, 0.72, 0.80, 0.28))
    handle_dark = model.material("handle_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        Box((CABINET_WIDTH, BACK_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.5 * BACK_THICKNESS, 0.5 * CABINET_HEIGHT)),
        material=cabinet_paint,
        name="back_panel",
    )
    cabinet_body.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(-(CABINET_WIDTH * 0.5) + (WALL_THICKNESS * 0.5), 0.5 * CABINET_DEPTH, 0.5 * CABINET_HEIGHT)
        ),
        material=cabinet_paint,
        name="left_wall",
    )
    cabinet_body.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=((CABINET_WIDTH * 0.5) - (WALL_THICKNESS * 0.5), 0.5 * CABINET_DEPTH, 0.5 * CABINET_HEIGHT)
        ),
        material=cabinet_paint,
        name="right_wall",
    )
    cabinet_body.visual(
        Box((CABINET_WIDTH - (2.0 * WALL_THICKNESS), CABINET_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.5 * CABINET_DEPTH, 0.5 * WALL_THICKNESS)),
        material=cabinet_paint,
        name="bottom_panel",
    )
    cabinet_body.visual(
        Box((CABINET_WIDTH - (2.0 * WALL_THICKNESS), CABINET_DEPTH, WALL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.5 * CABINET_DEPTH, CABINET_HEIGHT - (0.5 * WALL_THICKNESS))
        ),
        material=cabinet_paint,
        name="top_panel",
    )

    support_depth = 0.136
    support_y = 0.5 * support_depth + 0.012
    cabinet_body.visual(
        Box((CABINET_WIDTH - (2.0 * WALL_THICKNESS), support_depth, 0.008)),
        origin=Origin(xyz=(0.0, support_y, DRAWER_BASE_Z - 0.004)),
        material=cabinet_paint,
        name="drawer_support_shelf",
    )
    cabinet_body.visual(
        Box((CABINET_WIDTH - (2.0 * WALL_THICKNESS), 0.122, 0.028)),
        origin=Origin(xyz=(0.0, 0.073, DRAWER_BASE_Z + DRAWER_HEIGHT + 0.014)),
        material=cabinet_paint,
        name="upper_bay_lintel",
    )
    cabinet_body.visual(
        Box((CABINET_WIDTH - (2.0 * WALL_THICKNESS), 0.024, 0.042)),
        origin=Origin(xyz=(0.0, 0.020, DRAWER_BASE_Z + 0.086)),
        material=cabinet_paint,
        name="rear_stiffener",
    )

    usable_span = CABINET_WIDTH - (2.0 * WALL_THICKNESS) - (2.0 * INNER_MARGIN_X)
    slot_pitch = usable_span / DRAWER_COUNT
    divider_depth = 0.122
    divider_height = DRAWER_HEIGHT + 0.024
    divider_center_y = 0.071
    divider_center_z = DRAWER_BASE_Z + 0.5 * divider_height
    for index in range(DRAWER_COUNT - 1):
        divider_x = -0.5 * usable_span + slot_pitch * (index + 1)
        cabinet_body.visual(
            Box((DIVIDER_THICKNESS, divider_depth, divider_height)),
            origin=Origin(xyz=(divider_x, divider_center_y, divider_center_z)),
            material=cabinet_paint,
            name=f"divider_{index + 1:02d}",
        )

    left_hinge_leaf_x = -(CABINET_WIDTH * 0.5) + 0.004
    cabinet_body.visual(
        Box((0.008, 0.018, CABINET_HEIGHT - 0.060)),
        origin=Origin(xyz=(left_hinge_leaf_x, CABINET_DEPTH - 0.009, 0.5 * CABINET_HEIGHT)),
        material=handle_dark,
        name="hinge_leaf",
    )

    for index in range(DRAWER_COUNT):
        center_x = _drawer_center_x(index)
        for side_sign, label in ((-1.0, "left"), (1.0, "right")):
            cabinet_body.visual(
                Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
                origin=Origin(
                    xyz=(center_x + side_sign * ((DRAWER_WIDTH * 0.5) + 0.006), PIN_START_Y + (0.5 * PIN_LENGTH), DRAWER_BASE_Z + 0.010),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=pin_steel,
                name=f"guide_pin_{index + 1:02d}_{label}",
            )

        drawer = model.part(_drawer_part_name(index))
        _build_mini_drawer(
            drawer,
            drawer_width=DRAWER_WIDTH,
            body_material=drawer_paint,
            trim_material=pin_steel,
            label_material=label_cream,
        )

        model.articulation(
            _drawer_joint_name(index),
            ArticulationType.PRISMATIC,
            parent=cabinet_body,
            child=drawer,
            origin=Origin(xyz=(center_x, DRAWER_JOINT_Y, DRAWER_BASE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.18,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    cabinet_body.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.5 * CABINET_DEPTH, 0.5 * CABINET_HEIGHT)),
    )

    outer_door = model.part("outer_door")
    stile_width = 0.038
    rail_height = 0.040
    outer_door.visual(
        Box((stile_width, DOOR_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(xyz=(0.5 * stile_width, 0.5 * DOOR_THICKNESS, 0.5 * CABINET_HEIGHT)),
        material=cabinet_paint,
        name="door_stile_left",
    )
    outer_door.visual(
        Box((stile_width, DOOR_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(CABINET_WIDTH - (0.5 * stile_width), 0.5 * DOOR_THICKNESS, 0.5 * CABINET_HEIGHT)
        ),
        material=cabinet_paint,
        name="door_stile_right",
    )
    outer_door.visual(
        Box((CABINET_WIDTH - (2.0 * stile_width), DOOR_THICKNESS, rail_height)),
        origin=Origin(
            xyz=(0.5 * CABINET_WIDTH, 0.5 * DOOR_THICKNESS, CABINET_HEIGHT - (0.5 * rail_height))
        ),
        material=cabinet_paint,
        name="door_rail_top",
    )
    outer_door.visual(
        Box((CABINET_WIDTH - (2.0 * stile_width), DOOR_THICKNESS, rail_height)),
        origin=Origin(xyz=(0.5 * CABINET_WIDTH, 0.5 * DOOR_THICKNESS, 0.5 * rail_height)),
        material=cabinet_paint,
        name="door_rail_bottom",
    )
    outer_door.visual(
        Box((CABINET_WIDTH - (2.0 * stile_width) + 0.002, 0.006, CABINET_HEIGHT - (2.0 * rail_height) + 0.002)),
        origin=Origin(
            xyz=(0.5 * CABINET_WIDTH, 0.5 * DOOR_THICKNESS + 0.001, 0.5 * CABINET_HEIGHT)
        ),
        material=smoked_glass,
        name="door_glass",
    )
    outer_door.visual(
        Box((0.012, 0.018, 0.092)),
        origin=Origin(
            xyz=(CABINET_WIDTH - 0.042, DOOR_THICKNESS - 0.001, 0.5 * CABINET_HEIGHT),
        ),
        material=handle_dark,
        name="door_pull",
    )
    for index, z_center in enumerate((0.046, 0.180, 0.314)):
        outer_door.visual(
            Cylinder(radius=0.006, length=0.066),
            origin=Origin(xyz=(0.0, 0.5 * DOOR_THICKNESS, z_center)),
            material=pin_steel,
            name=f"hinge_knuckle_{index + 1}",
        )
    outer_door.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, DOOR_THICKNESS, CABINET_HEIGHT)),
        mass=3.4,
        origin=Origin(xyz=(0.5 * CABINET_WIDTH, 0.5 * DOOR_THICKNESS, 0.5 * CABINET_HEIGHT)),
    )

    model.articulation(
        "cabinet_to_outer_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=outer_door,
        origin=Origin(xyz=(-(CABINET_WIDTH * 0.5), CABINET_DEPTH, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet_body = object_model.get_part("cabinet_body")
    outer_door = object_model.get_part("outer_door")
    door_joint = object_model.get_articulation("cabinet_to_outer_door")

    ctx.check(
        "all twenty mini drawers are authored",
        all(object_model.get_part(_drawer_part_name(index)) is not None for index in range(DRAWER_COUNT)),
        details=f"expected={DRAWER_COUNT}",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            outer_door,
            cabinet_body,
            axis="y",
            min_gap=0.0,
            max_gap=0.002,
            name="door seats flush at the cabinet front",
        )
        ctx.expect_overlap(
            outer_door,
            cabinet_body,
            axes="xz",
            min_overlap=0.28,
            name="door covers the cabinet face",
        )

    closed_door_edge = ctx.part_element_world_aabb(outer_door, elem="door_stile_right")
    with ctx.pose({door_joint: 1.20}):
        opened_door_edge = ctx.part_element_world_aabb(outer_door, elem="door_stile_right")
    ctx.check(
        "outer door swings outward on its left hinge",
        closed_door_edge is not None
        and opened_door_edge is not None
        and opened_door_edge[0][1] > closed_door_edge[0][1] + 0.35,
        details=f"closed={closed_door_edge}, opened={opened_door_edge}",
    )

    for index in (0, DRAWER_COUNT // 2, DRAWER_COUNT - 1):
        drawer = object_model.get_part(_drawer_part_name(index))
        slide = object_model.get_articulation(_drawer_joint_name(index))
        upper = 0.0 if slide.motion_limits is None or slide.motion_limits.upper is None else slide.motion_limits.upper

        ctx.expect_contact(
            drawer,
            cabinet_body,
            name=f"{drawer.name} rests on the internal support shelf",
        )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: upper}):
            ctx.expect_within(
                drawer,
                cabinet_body,
                axes="xz",
                margin=0.001,
                name=f"{drawer.name} stays laterally guided inside the cabinet",
            )
            ctx.expect_overlap(
                drawer,
                cabinet_body,
                axes="y",
                min_overlap=0.070,
                name=f"{drawer.name} keeps retained insertion on the guide pins",
            )
            extended_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"{drawer.name} slides outward along +Y",
            rest_pos is not None and extended_pos is not None and extended_pos[1] > rest_pos[1] + 0.040,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
