from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_waffle_iron")

    shell = model.material("shell", rgba=(0.18, 0.18, 0.19, 1.0))
    plate = model.material("plate", rgba=(0.09, 0.09, 0.10, 1.0))
    grip = model.material("grip", rgba=(0.28, 0.29, 0.31, 1.0))
    button = model.material("button", rgba=(0.72, 0.74, 0.76, 1.0))

    base_shell = model.part("base_shell")
    base_shell.visual(
        Box((0.36, 0.30, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=shell,
        name="base_floor",
    )
    base_shell.visual(
        Box((0.340, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.144, 0.025)),
        material=shell,
        name="side_wall_0",
    )
    base_shell.visual(
        Box((0.340, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, -0.144, 0.025)),
        material=shell,
        name="side_wall_1",
    )
    base_shell.visual(
        Box((0.036, 0.300, 0.046)),
        origin=Origin(xyz=(0.162, 0.0, 0.023)),
        material=shell,
        name="front_wall",
    )
    base_shell.visual(
        Box((0.040, 0.300, 0.052)),
        origin=Origin(xyz=(-0.160, 0.0, 0.026)),
        material=shell,
        name="rear_wall",
    )
    base_shell.visual(
        Box((0.036, 0.300, 0.010)),
        origin=Origin(xyz=(0.162, 0.0, 0.057)),
        material=shell,
        name="front_rim",
    )
    base_shell.visual(
        Box((0.038, 0.300, 0.010)),
        origin=Origin(xyz=(-0.161, 0.0, 0.057)),
        material=shell,
        name="rear_rim",
    )
    base_shell.visual(
        Box((0.296, 0.032, 0.010)),
        origin=Origin(xyz=(0.0, 0.134, 0.057)),
        material=shell,
        name="side_rim_0",
    )
    base_shell.visual(
        Box((0.296, 0.032, 0.010)),
        origin=Origin(xyz=(0.0, -0.134, 0.057)),
        material=shell,
        name="side_rim_1",
    )
    base_shell.visual(
        Box((0.028, 0.276, 0.006)),
        origin=Origin(xyz=(0.130, 0.0, 0.041)),
        material=shell,
        name="plate_rail_front",
    )
    base_shell.visual(
        Box((0.028, 0.276, 0.006)),
        origin=Origin(xyz=(-0.126, 0.0, 0.041)),
        material=shell,
        name="plate_rail_rear",
    )
    base_shell.visual(
        Box((0.024, 0.140, 0.006)),
        origin=Origin(xyz=(-0.168, 0.0, 0.060)),
        material=shell,
        name="hinge_pad",
    )

    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((0.272, 0.212, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=plate,
        name="base_plate",
    )

    lid_shell = model.part("lid_shell")
    lid_shell.visual(
        Box((0.334, 0.292, 0.014)),
        origin=Origin(xyz=(0.167, 0.0, 0.029)),
        material=shell,
        name="lid_top",
    )
    lid_shell.visual(
        Box((0.028, 0.292, 0.024)),
        origin=Origin(xyz=(0.320, 0.0, 0.012)),
        material=shell,
        name="front_skirt",
    )
    lid_shell.visual(
        Box((0.312, 0.014, 0.024)),
        origin=Origin(xyz=(0.156, 0.139, 0.012)),
        material=shell,
        name="side_skirt_0",
    )
    lid_shell.visual(
        Box((0.312, 0.014, 0.024)),
        origin=Origin(xyz=(0.156, -0.139, 0.012)),
        material=shell,
        name="side_skirt_1",
    )
    lid_shell.visual(
        Box((0.024, 0.140, 0.022)),
        origin=Origin(xyz=(0.012, 0.0, 0.011)),
        material=shell,
        name="hinge_bridge",
    )

    for name, x, y in (
        ("front_button_bezel", 0.286, -0.095),
        ("rear_button_bezel", 0.246, -0.095),
    ):
        lid_shell.visual(
            Box((0.006, 0.020, 0.006)),
            origin=Origin(xyz=(x + 0.011, y, 0.039)),
            material=shell,
            name=f"{name}_front",
        )
        lid_shell.visual(
            Box((0.006, 0.020, 0.006)),
            origin=Origin(xyz=(x - 0.011, y, 0.039)),
            material=shell,
            name=f"{name}_rear",
        )
        lid_shell.visual(
            Box((0.016, 0.004, 0.006)),
            origin=Origin(xyz=(x, y + 0.008, 0.039)),
            material=shell,
            name=f"{name}_side_0",
        )
        lid_shell.visual(
            Box((0.016, 0.004, 0.006)),
            origin=Origin(xyz=(x, y - 0.008, 0.039)),
            material=shell,
            name=f"{name}_side_1",
        )

    lid_plate = model.part("lid_plate")
    lid_plate.visual(
        Box((0.270, 0.210, 0.010)),
        origin=Origin(xyz=(0.170, 0.0, 0.008)),
        material=plate,
        name="lid_plate",
    )
    lid_plate.visual(
        Box((0.020, 0.020, 0.012)),
        origin=Origin(xyz=(0.030, 0.090, 0.016)),
        material=plate,
        name="lid_tab_0",
    )
    lid_plate.visual(
        Box((0.020, 0.020, 0.012)),
        origin=Origin(xyz=(0.030, -0.090, 0.016)),
        material=plate,
        name="lid_tab_1",
    )

    front_grip = model.part("front_grip")
    front_grip.visual(
        Box((0.018, 0.018, 0.020)),
        origin=Origin(xyz=(0.343, 0.034, 0.014)),
        material=grip,
        name="grip_post_0",
    )
    front_grip.visual(
        Box((0.018, 0.018, 0.020)),
        origin=Origin(xyz=(0.343, -0.034, 0.014)),
        material=grip,
        name="grip_post_1",
    )
    front_grip.visual(
        Box((0.032, 0.110, 0.020)),
        origin=Origin(xyz=(0.360, 0.0, 0.014)),
        material=grip,
        name="grip_bar",
    )

    front_button = model.part("front_button")
    front_button.visual(
        Box((0.016, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=button,
        name="front_button_stem",
    )
    front_button.visual(
        Box((0.018, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=button,
        name="front_button_cap",
    )

    rear_button = model.part("rear_button")
    rear_button.visual(
        Box((0.016, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=button,
        name="rear_button_stem",
    )
    rear_button.visual(
        Box((0.018, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=button,
        name="rear_button_cap",
    )

    model.articulation(
        "base_plate_mount",
        ArticulationType.FIXED,
        parent=base_shell,
        child=base_plate,
        origin=Origin(),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base_shell,
        child=lid_shell,
        origin=Origin(xyz=(-0.168, 0.0, 0.063)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "lid_plate_mount",
        ArticulationType.FIXED,
        parent=lid_shell,
        child=lid_plate,
        origin=Origin(),
    )
    model.articulation(
        "grip_mount",
        ArticulationType.FIXED,
        parent=lid_shell,
        child=front_grip,
        origin=Origin(),
    )
    model.articulation(
        "front_button_press",
        ArticulationType.PRISMATIC,
        parent=lid_shell,
        child=front_button,
        origin=Origin(xyz=(0.286, -0.095, 0.042)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=0.004),
    )
    model.articulation(
        "rear_button_press",
        ArticulationType.PRISMATIC,
        parent=lid_shell,
        child=rear_button,
        origin=Origin(xyz=(0.246, -0.095, 0.042)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_plate = object_model.get_part("base_plate")
    lid_plate = object_model.get_part("lid_plate")
    lid_shell = object_model.get_part("lid_shell")
    front_grip = object_model.get_part("front_grip")
    front_button = object_model.get_part("front_button")
    rear_button = object_model.get_part("rear_button")

    lid_hinge = object_model.get_articulation("lid_hinge")
    front_button_press = object_model.get_articulation("front_button_press")
    rear_button_press = object_model.get_articulation("rear_button_press")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid_plate,
            base_plate,
            axis="z",
            positive_elem="lid_plate",
            negative_elem="base_plate",
            min_gap=0.008,
            max_gap=0.014,
            name="closed plates keep a shallow cooking gap",
        )
        ctx.expect_overlap(
            lid_plate,
            base_plate,
            axes="xy",
            elem_a="lid_plate",
            elem_b="base_plate",
            min_overlap=0.20,
            name="closed plates stay aligned over the cooking area",
        )
        ctx.expect_gap(
            front_button,
            rear_button,
            axis="x",
            positive_elem="front_button_cap",
            negative_elem="rear_button_cap",
            min_gap=0.010,
            name="program buttons remain visibly discrete",
        )

    hinge_limits = lid_hinge.motion_limits
    if hinge_limits is not None and hinge_limits.upper is not None:
        grip_rest = ctx.part_element_world_aabb(front_grip, elem="grip_bar")
        with ctx.pose({lid_hinge: hinge_limits.upper}):
            grip_open = ctx.part_element_world_aabb(front_grip, elem="grip_bar")
            lid_open = ctx.part_world_aabb(lid_shell)

        rest_mid_z = None if grip_rest is None else 0.5 * (grip_rest[0][2] + grip_rest[1][2])
        open_mid_z = None if grip_open is None else 0.5 * (grip_open[0][2] + grip_open[1][2])
        lid_open_min_z = None if lid_open is None else lid_open[0][2]
        ctx.check(
            "lid hinge lifts the front grip upward",
            rest_mid_z is not None and open_mid_z is not None and open_mid_z > rest_mid_z + 0.12,
            details=f"rest_mid_z={rest_mid_z}, open_mid_z={open_mid_z}",
        )
        ctx.check(
            "opened lid clears the base volume",
            lid_open_min_z is not None and lid_open_min_z > 0.03,
            details=f"lid_open_min_z={lid_open_min_z}",
        )

    front_limits = front_button_press.motion_limits
    rear_limits = rear_button_press.motion_limits
    if (
        front_limits is not None
        and rear_limits is not None
        and front_limits.upper is not None
        and rear_limits.upper is not None
    ):
        front_rest = ctx.part_world_position(front_button)
        rear_rest = ctx.part_world_position(rear_button)
        with ctx.pose({front_button_press: front_limits.upper}):
            front_pressed = ctx.part_world_position(front_button)
            rear_steady = ctx.part_world_position(rear_button)
        with ctx.pose({rear_button_press: rear_limits.upper}):
            rear_pressed = ctx.part_world_position(rear_button)
            front_steady = ctx.part_world_position(front_button)

        ctx.check(
            "front button presses downward on its own",
            front_rest is not None
            and front_pressed is not None
            and rear_rest is not None
            and rear_steady is not None
            and front_pressed[2] < front_rest[2] - 0.003
            and abs(rear_steady[2] - rear_rest[2]) < 0.001,
            details=(
                f"front_rest={front_rest}, front_pressed={front_pressed}, "
                f"rear_rest={rear_rest}, rear_steady={rear_steady}"
            ),
        )
        ctx.check(
            "rear button presses downward on its own",
            rear_rest is not None
            and rear_pressed is not None
            and front_rest is not None
            and front_steady is not None
            and rear_pressed[2] < rear_rest[2] - 0.003
            and abs(front_steady[2] - front_rest[2]) < 0.001,
            details=(
                f"rear_rest={rear_rest}, rear_pressed={rear_pressed}, "
                f"front_rest={front_rest}, front_steady={front_steady}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
