from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_stock_cart")

    frame_blue = model.material("frame_blue", rgba=(0.18, 0.33, 0.61, 1.0))
    deck_charcoal = model.material("deck_charcoal", rgba=(0.20, 0.21, 0.23, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.30, 0.31, 0.34, 1.0))
    steel_light = model.material("steel_light", rgba=(0.72, 0.74, 0.78, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))

    deck_length = 0.92
    deck_width = 0.58
    frame_height = 0.038
    frame_bottom_z = 0.102
    frame_center_z = frame_bottom_z + frame_height * 0.5
    deck_top_z = frame_bottom_z + frame_height
    rail_width = 0.046
    end_rail_thickness = 0.050
    deck_skin_thickness = 0.005

    platform = model.part("platform")
    platform.visual(
        Box((deck_length, rail_width, frame_height)),
        origin=Origin(xyz=(0.0, deck_width * 0.5 - rail_width * 0.5, frame_center_z)),
        material=frame_blue,
        name="right_side_rail",
    )
    platform.visual(
        Box((deck_length, rail_width, frame_height)),
        origin=Origin(xyz=(0.0, -deck_width * 0.5 + rail_width * 0.5, frame_center_z)),
        material=frame_blue,
        name="left_side_rail",
    )
    platform.visual(
        Box((end_rail_thickness, deck_width - 2.0 * rail_width, frame_height)),
        origin=Origin(xyz=(deck_length * 0.5 - end_rail_thickness * 0.5, 0.0, frame_center_z)),
        material=frame_blue,
        name="front_end_rail",
    )
    platform.visual(
        Box((end_rail_thickness, deck_width - 2.0 * rail_width, frame_height)),
        origin=Origin(xyz=(-deck_length * 0.5 + end_rail_thickness * 0.5, 0.0, frame_center_z)),
        material=frame_blue,
        name="rear_end_rail",
    )
    platform.visual(
        Box((deck_length - 0.10, 0.040, frame_height * 0.72)),
        origin=Origin(xyz=(0.0, 0.12, frame_bottom_z + frame_height * 0.36)),
        material=steel_dark,
        name="crossmember_upper",
    )
    platform.visual(
        Box((deck_length - 0.10, 0.040, frame_height * 0.72)),
        origin=Origin(xyz=(0.0, -0.12, frame_bottom_z + frame_height * 0.36)),
        material=steel_dark,
        name="crossmember_lower",
    )
    platform.visual(
        Box((deck_length - 0.04, deck_width - 0.04, deck_skin_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                deck_top_z - deck_skin_thickness * 0.5,
            )
        ),
        material=deck_charcoal,
        name="deck_surface",
    )
    platform.visual(
        Box((deck_length - 0.08, deck_width - 0.084, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, frame_bottom_z + 0.012)),
        material=steel_dark,
        name="undertray",
    )
    platform.inertial = Inertial.from_geometry(
        Box((deck_length, deck_width, deck_top_z)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, deck_top_z * 0.5)),
    )

    lip_width = deck_length - 0.10
    lip_height = 0.110
    lip_thickness = 0.012
    lip = model.part("lip")
    lip.visual(
        Cylinder(radius=0.007, length=lip_width),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=steel_light,
        name="hinge_barrel",
    )
    lip.visual(
        Box((lip_width, lip_height, lip_thickness)),
        origin=Origin(xyz=(0.0, -lip_height * 0.5, -0.001)),
        material=frame_blue,
        name="lip_panel",
    )
    lip.visual(
        Cylinder(radius=0.010, length=lip_width),
        origin=Origin(
            xyz=(0.0, -lip_height, 0.0),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=steel_light,
        name="lip_top_rail",
    )
    lip.visual(
        Box((0.020, lip_height * 0.76, lip_thickness)),
        origin=Origin(xyz=(lip_width * 0.5 - 0.010, -lip_height * 0.56, -0.001)),
        material=frame_blue,
        name="lip_right_gusset",
    )
    lip.visual(
        Box((0.020, lip_height * 0.76, lip_thickness)),
        origin=Origin(xyz=(-lip_width * 0.5 + 0.010, -lip_height * 0.56, -0.001)),
        material=frame_blue,
        name="lip_left_gusset",
    )
    lip.inertial = Inertial.from_geometry(
        Box((lip_width, lip_height, 0.020)),
        mass=4.0,
        origin=Origin(xyz=(0.0, -lip_height * 0.5, 0.010)),
    )

    caster_mount_x = deck_length * 0.5 - 0.070
    caster_mount_y = deck_width * 0.5 - 0.070
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            platform.visual(
                Box((0.100, 0.080, 0.008)),
                origin=Origin(
                    xyz=(
                        x_sign * caster_mount_x,
                        y_sign * caster_mount_y,
                        frame_bottom_z - 0.004,
                    )
                ),
                material=steel_dark,
                name=f"caster_plate_{'p' if x_sign > 0 else 'n'}x_{'p' if y_sign > 0 else 'n'}y",
            )
            platform.visual(
                Cylinder(radius=0.014, length=0.014),
                origin=Origin(
                    xyz=(
                        x_sign * caster_mount_x,
                        y_sign * caster_mount_y,
                        frame_bottom_z - 0.015,
                    )
                ),
                material=steel_light,
                name=f"caster_collar_{'p' if x_sign > 0 else 'n'}x_{'p' if y_sign > 0 else 'n'}y",
            )

    model.articulation(
        "platform_to_lip",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=lip,
        origin=Origin(
            xyz=(
                0.0,
                deck_width * 0.5 - rail_width - 0.004,
                deck_top_z + 0.007,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )

    def add_caster(name: str, mount_xyz: tuple[float, float, float]) -> None:
        fork = model.part(f"{name}_fork")
        fork.visual(
            Cylinder(radius=0.0115, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.009)),
            material=steel_light,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.034, 0.042, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=steel_dark,
            name="fork_head",
        )
        fork.visual(
            Box((0.030, 0.006, 0.066)),
            origin=Origin(xyz=(0.0, 0.019, -0.050)),
            material=steel_dark,
            name="right_cheek",
        )
        fork.visual(
            Box((0.030, 0.006, 0.066)),
            origin=Origin(xyz=(0.0, -0.019, -0.050)),
            material=steel_dark,
            name="left_cheek",
        )
        fork.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(
                xyz=(0.0, 0.021, -0.068),
                rpy=(pi * 0.5, 0.0, 0.0),
            ),
            material=steel_light,
            name="right_axle_boss",
        )
        fork.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(
                xyz=(0.0, -0.021, -0.068),
                rpy=(pi * 0.5, 0.0, 0.0),
            ),
            material=steel_light,
            name="left_axle_boss",
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.050, 0.050, 0.070)),
            mass=1.1,
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
        )

        wheel = model.part(f"{name}_wheel")
        wheel.visual(
            Cylinder(radius=0.042, length=0.028),
            origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
            material=rubber_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.024, length=0.020),
            origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
            material=steel_light,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.016, length=0.005),
            origin=Origin(
                xyz=(0.0, 0.0155, 0.0),
                rpy=(pi * 0.5, 0.0, 0.0),
            ),
            material=steel_dark,
            name="outer_washer",
        )
        wheel.visual(
            Cylinder(radius=0.016, length=0.005),
            origin=Origin(
                xyz=(0.0, -0.0155, 0.0),
                rpy=(pi * 0.5, 0.0, 0.0),
            ),
            material=steel_dark,
            name="inner_washer",
        )
        wheel.visual(
            Box((0.004, 0.004, 0.010)),
            origin=Origin(xyz=(0.031, 0.012, 0.012)),
            material=steel_light,
            name="valve_stem",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.042, length=0.028),
            mass=0.9,
            origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        )

        model.articulation(
            f"platform_to_{name}_swivel",
            ArticulationType.CONTINUOUS,
            parent=platform,
            child=fork,
            origin=Origin(xyz=mount_xyz),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=16.0,
                velocity=8.0,
            ),
        )
        model.articulation(
            f"{name}_fork_to_{name}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.068)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=30.0,
            ),
        )

    caster_mount_z = frame_bottom_z - 0.022
    add_caster("front_right", (caster_mount_x, caster_mount_y, caster_mount_z))
    add_caster("rear_right", (-caster_mount_x, caster_mount_y, caster_mount_z))
    add_caster("front_left", (caster_mount_x, -caster_mount_y, caster_mount_z))
    add_caster("rear_left", (-caster_mount_x, -caster_mount_y, caster_mount_z))
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    platform = object_model.get_part("platform")
    lip = object_model.get_part("lip")
    lip_joint = object_model.get_articulation("platform_to_lip")
    caster_names = ("front_right", "rear_right", "front_left", "rear_left")

    ctx.expect_gap(
        lip,
        platform,
        axis="z",
        positive_elem="lip_panel",
        negative_elem="deck_surface",
        max_gap=0.004,
        max_penetration=0.0,
        name="lip rests just above the deck in the folded pose",
    )
    ctx.expect_overlap(
        lip,
        platform,
        axes="x",
        elem_a="lip_panel",
        elem_b="deck_surface",
        min_overlap=0.75,
        name="lip spans most of the deck edge",
    )

    closed_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_panel")
    with ctx.pose({lip_joint: 1.45}):
        open_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_panel")

    ctx.check(
        "lip rotates upward into a retaining wall",
        closed_lip_aabb is not None
        and open_lip_aabb is not None
        and open_lip_aabb[1][2] > closed_lip_aabb[1][2] + 0.08,
        details=f"closed={closed_lip_aabb}, open={open_lip_aabb}",
    )

    for caster_name in caster_names:
        fork = object_model.get_part(f"{caster_name}_fork")
        wheel = object_model.get_part(f"{caster_name}_wheel")
        swivel = object_model.get_articulation(f"platform_to_{caster_name}_swivel")
        spin = object_model.get_articulation(f"{caster_name}_fork_to_{caster_name}_wheel_spin")

        ctx.expect_gap(
            platform,
            wheel,
            axis="z",
            min_gap=0.001,
            max_gap=0.035,
            name=f"{caster_name} wheel stays below the deck",
        )
        ctx.expect_within(
            wheel,
            fork,
            axes="y",
            margin=0.006,
            inner_elem="tire",
            name=f"{caster_name} wheel stays between the fork cheeks",
        )
        ctx.check(
            f"{caster_name} swivel axis is vertical",
            swivel.axis == (0.0, 0.0, 1.0) and swivel.joint_type == ArticulationType.CONTINUOUS,
            details=f"axis={swivel.axis}, type={swivel.joint_type}",
        )
        ctx.check(
            f"{caster_name} wheel spins on its axle",
            spin.axis == (0.0, 1.0, 0.0) and spin.joint_type == ArticulationType.CONTINUOUS,
            details=f"axis={spin.axis}, type={spin.joint_type}",
        )

    swivel_joint = object_model.get_articulation("platform_to_front_right_swivel")
    wheel_joint = object_model.get_articulation("front_right_fork_to_front_right_wheel_spin")
    rest_wheel_aabb = ctx.part_world_aabb(object_model.get_part("front_right_wheel"))
    with ctx.pose({swivel_joint: pi * 0.5}):
        turned_wheel_aabb = ctx.part_world_aabb(object_model.get_part("front_right_wheel"))
    with ctx.pose({wheel_joint: pi * 0.5}):
        spun_valve_aabb = ctx.part_element_world_aabb(object_model.get_part("front_right_wheel"), elem="valve_stem")
    rest_valve_aabb = ctx.part_element_world_aabb(object_model.get_part("front_right_wheel"), elem="valve_stem")

    ctx.check(
        "front right caster swivel rotates the wheel footprint",
        rest_wheel_aabb is not None
        and turned_wheel_aabb is not None
        and (rest_wheel_aabb[1][0] - rest_wheel_aabb[0][0]) > (rest_wheel_aabb[1][1] - rest_wheel_aabb[0][1]) + 0.03
        and (turned_wheel_aabb[1][1] - turned_wheel_aabb[0][1]) > (turned_wheel_aabb[1][0] - turned_wheel_aabb[0][0]) + 0.03,
        details=f"rest={rest_wheel_aabb}, turned={turned_wheel_aabb}",
    )
    ctx.check(
        "front right wheel spin moves the valve stem around the axle",
        rest_valve_aabb is not None
        and spun_valve_aabb is not None
        and abs(spun_valve_aabb[0][2] - rest_valve_aabb[0][2]) > 0.01,
        details=f"rest={rest_valve_aabb}, spun={spun_valve_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
