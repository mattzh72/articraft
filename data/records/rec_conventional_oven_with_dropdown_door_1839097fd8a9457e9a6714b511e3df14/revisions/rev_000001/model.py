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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freestanding_range_cooker")

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    chrome = model.material("chrome", rgba=(0.86, 0.88, 0.90, 1.0))
    enamel_black = model.material("enamel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.17, 0.17, 0.18, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.12, 0.14, 0.17, 0.65))

    body_width = 0.90
    body_depth = 0.64
    side_height = 0.89
    side_thickness = 0.018
    front_pillar_width = 0.055
    front_frame_depth = 0.020
    front_frame_center_y = 0.3005
    front_frame_face_y = front_frame_center_y + front_frame_depth / 2.0
    cavity_depth = 0.5925
    cavity_center_y = -0.00575

    door_width = 0.78
    door_height = 0.61
    door_thickness = 0.045
    door_hinge_z = 0.185
    door_hinge_y = front_frame_face_y + 0.010

    drawer_front_width = 0.78
    drawer_front_height = 0.145
    drawer_origin_y = front_frame_face_y

    body_shell = model.part("body_shell")
    body_shell.visual(
        Box((side_thickness, body_depth, side_height)),
        origin=Origin(
            xyz=(
                -(body_width / 2.0 - side_thickness / 2.0),
                0.0,
                side_height / 2.0,
            )
        ),
        material=stainless,
        name="left_side",
    )
    body_shell.visual(
        Box((side_thickness, body_depth, side_height)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - side_thickness / 2.0,
                0.0,
                side_height / 2.0,
            )
        ),
        material=stainless,
        name="right_side",
    )
    body_shell.visual(
        Box((body_width - 2.0 * side_thickness, side_thickness, side_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth / 2.0 - side_thickness / 2.0),
                side_height / 2.0,
            )
        ),
        material=stainless,
        name="back_panel",
    )
    body_shell.visual(
        Box((body_width - 2.0 * side_thickness, 0.6125, 0.020)),
        origin=Origin(xyz=(0.0, 0.00425, 0.010)),
        material=enamel_black,
        name="base_floor",
    )
    body_shell.visual(
        Box((front_pillar_width, front_frame_depth, side_height)),
        origin=Origin(
            xyz=(
                -(body_width / 2.0 - front_pillar_width / 2.0),
                front_frame_center_y,
                side_height / 2.0,
            )
        ),
        material=stainless,
        name="front_left_pillar",
    )
    body_shell.visual(
        Box((front_pillar_width, front_frame_depth, side_height)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - front_pillar_width / 2.0,
                front_frame_center_y,
                side_height / 2.0,
            )
        ),
        material=stainless,
        name="front_right_pillar",
    )
    body_shell.visual(
        Box((body_width - 2.0 * front_pillar_width, 0.040, 0.090)),
        origin=Origin(xyz=(0.0, 0.2905, 0.845)),
        material=stainless,
        name="upper_front_rail",
    )
    body_shell.visual(
        Box((body_width - 2.0 * front_pillar_width, front_frame_depth, 0.020)),
        origin=Origin(xyz=(0.0, front_frame_center_y, 0.195)),
        material=stainless,
        name="hinge_sill",
    )
    body_shell.visual(
        Box((0.015, cavity_depth, 0.590)),
        origin=Origin(xyz=(-0.3725, cavity_center_y, 0.500)),
        material=enamel_black,
        name="cavity_left_liner",
    )
    body_shell.visual(
        Box((0.015, cavity_depth, 0.590)),
        origin=Origin(xyz=(0.3725, cavity_center_y, 0.500)),
        material=enamel_black,
        name="cavity_right_liner",
    )
    body_shell.visual(
        Box((0.730, cavity_depth, 0.015)),
        origin=Origin(xyz=(0.0, cavity_center_y, 0.1975)),
        material=enamel_black,
        name="cavity_floor",
    )
    body_shell.visual(
        Box((0.730, cavity_depth, 0.015)),
        origin=Origin(xyz=(0.0, cavity_center_y, 0.7975)),
        material=enamel_black,
        name="cavity_ceiling",
    )
    body_shell.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, 1.03)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
    )

    hob_console = model.part("hob_console")
    hob_console.visual(
        Box((body_width, body_depth, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.9025)),
        material=stainless,
        name="cooktop_slab",
    )
    hob_console.visual(
        Box((0.82, 0.070, 0.100)),
        origin=Origin(xyz=(0.0, 0.285, 0.965)),
        material=stainless,
        name="control_fascia",
    )
    hob_console.visual(
        Box((body_width, 0.040, 0.120)),
        origin=Origin(xyz=(0.0, -0.290, 0.975)),
        material=stainless,
        name="backsplash",
    )

    burner_x_positions = (-0.27, 0.0, 0.27)
    burner_y_positions = (-0.17, 0.01)
    burner_index = 1
    for burner_y in burner_y_positions:
        for burner_x in burner_x_positions:
            hob_console.visual(
                Cylinder(radius=0.042, length=0.012),
                origin=Origin(xyz=(burner_x, burner_y, 0.921)),
                material=enamel_black,
                name=f"burner_cap_{burner_index}",
            )
            burner_index += 1

    grate_x_positions = (-0.27, 0.0, 0.27)
    grate_index = 1
    for grate_x in grate_x_positions:
        for side in (-1.0, 1.0):
            hob_console.visual(
                Box((0.015, 0.340, 0.014)),
                origin=Origin(xyz=(grate_x + side * 0.095, -0.08, 0.934)),
                material=cast_iron,
                name=f"grate_{grate_index}_long_{'l' if side < 0 else 'r'}",
            )
        for front in (-1.0, 1.0):
            hob_console.visual(
                Box((0.190, 0.015, 0.014)),
                origin=Origin(xyz=(grate_x, -0.08 + front * 0.130, 0.934)),
                material=cast_iron,
                name=f"grate_{grate_index}_cross_{'f' if front > 0 else 'r'}",
            )
        grate_index += 1

    knob_x_positions = (-0.30, -0.18, -0.06, 0.06, 0.18, 0.30)
    for knob_index, knob_x in enumerate(knob_x_positions, start=1):
        hob_console.visual(
            Cylinder(radius=0.017, length=0.030),
            origin=Origin(
                xyz=(knob_x, 0.335, 0.965),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"knob_{knob_index}",
        )

    hob_console.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, 0.16)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.955)),
    )

    oven_door = model.part("oven_door")
    frame_width = 0.070
    oven_door.visual(
        Box((door_width - 2.0 * frame_width, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.005, 0.010)),
        material=stainless,
        name="hinge_leaf",
    )
    oven_door.visual(
        Box((frame_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                -(door_width / 2.0 - frame_width / 2.0),
                door_thickness / 2.0,
                door_height / 2.0,
            )
        ),
        material=stainless,
        name="left_frame",
    )
    oven_door.visual(
        Box((frame_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                door_width / 2.0 - frame_width / 2.0,
                door_thickness / 2.0,
                door_height / 2.0,
            )
        ),
        material=stainless,
        name="right_frame",
    )
    oven_door.visual(
        Box((door_width, door_thickness, 0.100)),
        origin=Origin(xyz=(0.0, door_thickness / 2.0, 0.050)),
        material=stainless,
        name="bottom_frame",
    )
    oven_door.visual(
        Box((door_width, door_thickness, 0.090)),
        origin=Origin(xyz=(0.0, door_thickness / 2.0, door_height - 0.045)),
        material=stainless,
        name="top_frame",
    )
    oven_door.visual(
        Box((0.675, 0.008, 0.425)),
        origin=Origin(xyz=(0.0, 0.004, 0.3625)),
        material=enamel_black,
        name="inner_panel",
    )
    oven_door.visual(
        Box((0.665, 0.010, 0.420)),
        origin=Origin(xyz=(0.0, 0.030, 0.360)),
        material=glass_dark,
        name="door_glass",
    )
    oven_door.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(
            xyz=(-0.230, 0.052, 0.535),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="handle_post_left",
    )
    oven_door.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(
            xyz=(0.230, 0.052, 0.535),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="handle_post_right",
    )
    oven_door.visual(
        Cylinder(radius=0.012, length=0.600),
        origin=Origin(
            xyz=(0.0, 0.072, 0.535),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="handle_bar",
    )
    oven_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.080, door_height)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.040, door_height / 2.0)),
    )

    storage_drawer = model.part("storage_drawer")
    storage_drawer.visual(
        Box((drawer_front_width, 0.020, drawer_front_height)),
        origin=Origin(xyz=(0.0, 0.010, 0.0975)),
        material=stainless,
        name="drawer_front",
    )
    storage_drawer.visual(
        Box((0.720, 0.480, 0.012)),
        origin=Origin(xyz=(0.0, -0.240, 0.026)),
        material=enamel_black,
        name="drawer_bottom",
    )
    storage_drawer.visual(
        Box((0.012, 0.460, 0.100)),
        origin=Origin(xyz=(-0.354, -0.230, 0.070)),
        material=enamel_black,
        name="drawer_side_left",
    )
    storage_drawer.visual(
        Box((0.012, 0.460, 0.100)),
        origin=Origin(xyz=(0.354, -0.230, 0.070)),
        material=enamel_black,
        name="drawer_side_right",
    )
    storage_drawer.visual(
        Box((0.720, 0.012, 0.100)),
        origin=Origin(xyz=(0.0, -0.474, 0.070)),
        material=enamel_black,
        name="drawer_back",
    )
    storage_drawer.visual(
        Box((0.580, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.029, 0.150)),
        material=chrome,
        name="drawer_trim_handle",
    )
    storage_drawer.inertial = Inertial.from_geometry(
        Box((drawer_front_width, 0.520, drawer_front_height)),
        mass=7.0,
        origin=Origin(xyz=(0.0, -0.230, 0.0725)),
    )

    model.articulation(
        "body_to_hob_console",
        ArticulationType.FIXED,
        parent=body_shell,
        child=hob_console,
        origin=Origin(),
    )
    model.articulation(
        "body_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=body_shell,
        child=oven_door,
        origin=Origin(xyz=(0.0, door_hinge_y, door_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "body_to_storage_drawer",
        ArticulationType.PRISMATIC,
        parent=body_shell,
        child=storage_drawer,
        origin=Origin(xyz=(0.0, drawer_origin_y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.35,
            lower=0.0,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body_shell = object_model.get_part("body_shell")
    hob_console = object_model.get_part("hob_console")
    oven_door = object_model.get_part("oven_door")
    storage_drawer = object_model.get_part("storage_drawer")

    door_hinge = object_model.get_articulation("body_to_oven_door")
    drawer_slide = object_model.get_articulation("body_to_storage_drawer")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(hob_console, body_shell, elem_a="cooktop_slab", elem_b="left_side")
    ctx.expect_contact(oven_door, body_shell, elem_a="hinge_leaf", elem_b="hinge_sill")
    ctx.expect_contact(storage_drawer, body_shell, elem_a="drawer_bottom", elem_b="base_floor")

    ctx.expect_gap(
        oven_door,
        storage_drawer,
        axis="z",
        min_gap=0.010,
        max_gap=0.030,
        name="door_clears_drawer_stack_gap",
    )
    ctx.expect_within(
        storage_drawer,
        body_shell,
        axes="x",
        margin=0.0,
        name="drawer_stays_between_side_panels",
    )

    drawer_rest = ctx.part_world_position(storage_drawer)
    with ctx.pose({drawer_slide: 0.28}):
        drawer_open = ctx.part_world_position(storage_drawer)
        if drawer_rest is None or drawer_open is None:
            ctx.fail("drawer_pose_measurement_available", "Could not measure drawer world position.")
        else:
            ctx.check(
                "drawer_slides_forward",
                drawer_open[1] > drawer_rest[1] + 0.25,
                details=f"drawer rest y={drawer_rest[1]:.4f}, open y={drawer_open[1]:.4f}",
            )
        ctx.expect_contact(storage_drawer, body_shell, elem_a="drawer_bottom", elem_b="base_floor")

    door_closed_aabb = ctx.part_world_aabb(oven_door)
    with ctx.pose({door_hinge: math.radians(88.0)}):
        door_open_aabb = ctx.part_world_aabb(oven_door)
        if door_closed_aabb is None or door_open_aabb is None:
            ctx.fail("door_pose_measurement_available", "Could not measure door AABBs.")
        else:
            ctx.check(
                "door_drops_forward_on_bottom_hinge",
                door_open_aabb[1][1] > door_closed_aabb[1][1] + 0.40
                and door_open_aabb[1][2] < door_closed_aabb[1][2] - 0.18,
                details=(
                    f"closed max_y={door_closed_aabb[1][1]:.4f}, open max_y={door_open_aabb[1][1]:.4f}; "
                    f"closed max_z={door_closed_aabb[1][2]:.4f}, open max_z={door_open_aabb[1][2]:.4f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
