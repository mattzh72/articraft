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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_door_switchgear_panel")

    width = 1.20
    height = 2.10
    depth = 0.45
    shell_thickness = 0.03
    door_thickness = 0.025
    side_reveal = 0.0
    center_gap = 0.012
    door_bottom_gap = 0.04
    door_top_gap = 0.04
    door_height = height - door_bottom_gap - door_top_gap
    door_width = (width * 0.5) - side_reveal - (center_gap * 0.5)
    front_y = (depth * 0.5) + (door_thickness * 0.5)

    door_return_depth = 0.03
    door_return_width = 0.03
    hinge_barrel_radius = 0.009
    hinge_barrel_length = 0.08
    hinge_barrel_z = (0.26, door_height * 0.5, door_height - 0.26)

    lock_plate_thickness = 0.006
    lock_plate_size = (0.12, lock_plate_thickness, 0.26)
    lock_inset_from_meeting_edge = 0.095
    lock_z = 1.05

    shell_color = model.material("painted_steel", rgba=(0.81, 0.83, 0.85, 1.0))
    door_color = model.material("door_paint", rgba=(0.84, 0.86, 0.88, 1.0))
    gasket_color = model.material("gasket_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hardware_color = model.material("hardware_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    carcass = model.part("carcass")
    carcass.visual(
        Box((shell_thickness, depth, height)),
        origin=Origin(xyz=(-(width * 0.5) + (shell_thickness * 0.5), 0.0, height * 0.5)),
        material=shell_color,
        name="left_wall",
    )
    carcass.visual(
        Box((shell_thickness, depth, height)),
        origin=Origin(xyz=((width * 0.5) - (shell_thickness * 0.5), 0.0, height * 0.5)),
        material=shell_color,
        name="right_wall",
    )
    carcass.visual(
        Box((width - (2.0 * shell_thickness), depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, shell_thickness * 0.5)),
        material=shell_color,
        name="bottom_pan",
    )
    carcass.visual(
        Box((width - (2.0 * shell_thickness), depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, height - (shell_thickness * 0.5))),
        material=shell_color,
        name="top_pan",
    )
    carcass.visual(
        Box((width - (2.0 * shell_thickness), shell_thickness, height - (2.0 * shell_thickness))),
        origin=Origin(
            xyz=(0.0, -(depth * 0.5) + (shell_thickness * 0.5), height * 0.5),
        ),
        material=shell_color,
        name="back_panel",
    )
    carcass.visual(
        Box((0.014, 0.016, height - (2.0 * shell_thickness))),
        origin=Origin(
            xyz=(0.0, (depth * 0.5) - 0.012, height * 0.5),
        ),
        material=gasket_color,
        name="center_gasket",
    )

    def add_door(part_name: str, x_sign: float) -> object:
        door = model.part(part_name)
        door.visual(
            Box((door_width, door_thickness, door_height)),
            origin=Origin(xyz=(x_sign * (door_width * 0.5), 0.0, door_height * 0.5)),
            material=door_color,
            name="outer_skin",
        )
        door.visual(
            Box((door_width - (2.0 * door_return_width), door_return_depth, door_return_width)),
            origin=Origin(
                xyz=(
                    x_sign * (door_width * 0.5),
                    -(door_thickness * 0.5) - (door_return_depth * 0.5),
                    door_return_width * 0.5,
                )
            ),
            material=door_color,
            name="bottom_return",
        )
        door.visual(
            Box((door_width - (2.0 * door_return_width), door_return_depth, door_return_width)),
            origin=Origin(
                xyz=(
                    x_sign * (door_width * 0.5),
                    -(door_thickness * 0.5) - (door_return_depth * 0.5),
                    door_height - (door_return_width * 0.5),
                )
            ),
            material=door_color,
            name="top_return",
        )
        door.visual(
            Box((door_return_width, door_return_depth, door_height - (2.0 * door_return_width))),
            origin=Origin(
                xyz=(
                    -x_sign * (door_return_width * 0.5),
                    -(door_thickness * 0.5) - (door_return_depth * 0.5),
                    door_height * 0.5,
                )
            ),
            material=door_color,
            name="hinge_return",
        )
        door.visual(
            Box((door_return_width, door_return_depth, door_height - (2.0 * door_return_width))),
            origin=Origin(
                xyz=(
                    x_sign * (door_width - (door_return_width * 0.5)),
                    -(door_thickness * 0.5) - (door_return_depth * 0.5),
                    door_height * 0.5,
                )
            ),
            material=door_color,
            name="meeting_return",
        )
        for index, barrel_z in enumerate(hinge_barrel_z, start=1):
            door.visual(
                Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
                origin=Origin(xyz=(0.0, 0.0, barrel_z)),
                material=hardware_color,
                name=f"hinge_barrel_{index}",
            )
        return door

    left_door = add_door("left_door", x_sign=1.0)
    right_door = add_door("right_door", x_sign=-1.0)

    right_door.visual(
        Box(lock_plate_size),
        origin=Origin(
            xyz=(
                -door_width + lock_inset_from_meeting_edge,
                (door_thickness * 0.5) + (lock_plate_thickness * 0.5),
                lock_z - door_bottom_gap,
            )
        ),
        material=hardware_color,
        name="lock_plate",
    )

    locking_bar = model.part("right_door_locking_bar")
    locking_bar.visual(
        Cylinder(radius=0.035, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=hardware_color,
        name="hub",
    )
    locking_bar.visual(
        Box((0.05, 0.028, 0.95)),
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
        material=hardware_color,
        name="outer_bar",
    )
    locking_bar.visual(
        Box((0.09, 0.02, 0.08)),
        origin=Origin(xyz=(0.0, 0.038, 0.0)),
        material=hardware_color,
        name="center_grip",
    )

    model.articulation(
        "carcass_to_left_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(-(width * 0.5) + side_reveal, front_y, door_bottom_gap)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=2.35,
        ),
    )
    model.articulation(
        "carcass_to_right_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=((width * 0.5) - side_reveal, front_y, door_bottom_gap)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=2.35,
        ),
    )
    model.articulation(
        "right_door_to_locking_bar",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=locking_bar,
        origin=Origin(
            xyz=(
                -door_width + lock_inset_from_meeting_edge,
                (door_thickness * 0.5) + lock_plate_thickness,
                lock_z - door_bottom_gap,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-(math.pi * 0.5),
            upper=math.pi * 0.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    locking_bar = object_model.get_part("right_door_locking_bar")

    left_hinge = object_model.get_articulation("carcass_to_left_door")
    right_hinge = object_model.get_articulation("carcass_to_right_door")
    turn_handle = object_model.get_articulation("right_door_to_locking_bar")

    left_skin = left_door.get_visual("outer_skin")
    right_skin = right_door.get_visual("outer_skin")
    lock_plate = right_door.get_visual("lock_plate")
    lock_hub = locking_bar.get_visual("hub")

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, turn_handle: 0.0}):
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            positive_elem=right_skin,
            negative_elem=left_skin,
            min_gap=0.010,
            max_gap=0.014,
            name="double doors keep a narrow center seam",
        )
        ctx.expect_gap(
            left_door,
            carcass,
            axis="y",
            positive_elem=left_skin,
            max_gap=0.0005,
            max_penetration=0.0,
            name="left door skin closes flush with the carcass face",
        )
        ctx.expect_gap(
            right_door,
            carcass,
            axis="y",
            positive_elem=right_skin,
            max_gap=0.0005,
            max_penetration=0.0,
            name="right door skin closes flush with the carcass face",
        )
        ctx.expect_gap(
            locking_bar,
            right_door,
            axis="y",
            positive_elem=lock_hub,
            negative_elem=lock_plate,
            max_gap=0.0005,
            max_penetration=0.0,
            name="locking bar hub seats against its door plate",
        )

    left_closed = ctx.part_element_world_aabb(left_door, elem="outer_skin")
    with ctx.pose({left_hinge: 1.15}):
        left_open = ctx.part_element_world_aabb(left_door, elem="outer_skin")
    ctx.check(
        "left leaf swings outward on its outer hinge",
        left_closed is not None
        and left_open is not None
        and left_open[1][1] > left_closed[1][1] + 0.18,
        details=f"closed={left_closed}, open={left_open}",
    )

    right_closed = ctx.part_element_world_aabb(right_door, elem="outer_skin")
    with ctx.pose({right_hinge: 1.15}):
        right_open = ctx.part_element_world_aabb(right_door, elem="outer_skin")
    ctx.check(
        "right leaf swings outward on its outer hinge",
        right_closed is not None
        and right_open is not None
        and right_open[1][1] > right_closed[1][1] + 0.18,
        details=f"closed={right_closed}, open={right_open}",
    )

    bar_closed = ctx.part_element_world_aabb(locking_bar, elem="outer_bar")
    with ctx.pose({turn_handle: math.pi * 0.5}):
        bar_turned = ctx.part_element_world_aabb(locking_bar, elem="outer_bar")
        ctx.expect_gap(
            locking_bar,
            right_door,
            axis="y",
            positive_elem=lock_hub,
            negative_elem=lock_plate,
            max_gap=0.0005,
            max_penetration=0.0,
            name="turn handle stays seated while rotated",
        )

    def span(aabb: object, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    closed_x = span(bar_closed, 0)
    closed_z = span(bar_closed, 2)
    turned_x = span(bar_turned, 0)
    turned_z = span(bar_turned, 2)
    ctx.check(
        "locking bar rotates from vertical to crosswise",
        closed_x is not None
        and closed_z is not None
        and turned_x is not None
        and turned_z is not None
        and closed_z > (closed_x * 4.0)
        and turned_x > (turned_z * 4.0),
        details=(
            f"closed_spans=(x={closed_x}, z={closed_z}), "
            f"turned_spans=(x={turned_x}, z={turned_z})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
