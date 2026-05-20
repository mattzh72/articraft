from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def _add_tray_visuals(
    tray_part,
    *,
    tray_depth: float,
    tray_width: float,
    tray_height: float,
    bezel_depth: float,
    bezel_width: float,
    bezel_height: float,
    material_body,
    material_bezel,
    material_handle,
    index: int,
) -> None:
    runner_length = tray_depth
    runner_width = 0.010
    runner_height = 0.010
    runner_y = tray_width * 0.5 + runner_width * 0.5

    floor_thickness = 0.003
    side_thickness = 0.004
    rear_thickness = 0.004
    basket_depth = tray_depth - 0.028
    basket_x0 = 0.010
    basket_inner_width = tray_width - 2.0 * side_thickness
    wall_height = tray_height - 0.008

    tray_part.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(runner_length * 0.5, -runner_y, runner_height * 0.5)),
        material=material_body,
        name="runner_left",
    )
    tray_part.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(runner_length * 0.5, runner_y, runner_height * 0.5)),
        material=material_body,
        name="runner_right",
    )
    tray_part.visual(
        Box((basket_depth, basket_inner_width, floor_thickness)),
        origin=Origin(
            xyz=(basket_x0 + basket_depth * 0.5, 0.0, runner_height + floor_thickness * 0.5)
        ),
        material=material_body,
        name="tray_floor",
    )
    tray_part.visual(
        Box((basket_depth, side_thickness, wall_height)),
        origin=Origin(
            xyz=(
                basket_x0 + basket_depth * 0.5,
                -(basket_inner_width * 0.5 + side_thickness * 0.5),
                runner_height + wall_height * 0.5,
            )
        ),
        material=material_body,
        name="tray_wall_left",
    )
    tray_part.visual(
        Box((basket_depth, side_thickness, wall_height)),
        origin=Origin(
            xyz=(
                basket_x0 + basket_depth * 0.5,
                basket_inner_width * 0.5 + side_thickness * 0.5,
                runner_height + wall_height * 0.5,
            )
        ),
        material=material_body,
        name="tray_wall_right",
    )
    tray_part.visual(
        Box((rear_thickness, tray_width, wall_height)),
        origin=Origin(
            xyz=(basket_x0 + rear_thickness * 0.5, 0.0, runner_height + wall_height * 0.5)
        ),
        material=material_body,
        name="tray_rear_wall",
    )
    tray_part.visual(
        Box((bezel_depth, bezel_width, bezel_height)),
        origin=Origin(
            xyz=(tray_depth - bezel_depth * 0.5, 0.0, runner_height + bezel_height * 0.5)
        ),
        material=material_bezel,
        name="tray_front_bezel",
    )
    tray_part.visual(
        Box((0.010, 0.050, 0.010)),
        origin=Origin(
            xyz=(tray_depth - 0.004, 0.0, runner_height + bezel_height * 0.52)
        ),
        material=material_handle,
        name="tray_pull",
    )
    tray_part.visual(
        Box((0.004, 0.020, 0.016)),
        origin=Origin(
            xyz=(tray_depth - 0.002, -0.050, runner_height + bezel_height * 0.50)
        ),
        material=material_handle,
        name=f"tray_latch_{index}",
    )
    tray_part.inertial = Inertial.from_geometry(
        Box((tray_depth, bezel_width, runner_height + bezel_height)),
        mass=0.60,
        origin=Origin(
            xyz=(tray_depth * 0.5, 0.0, (runner_height + bezel_height) * 0.5)
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nas_storage_tower")

    chassis_dark = model.material("chassis_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    chassis_panel = model.material("chassis_panel", rgba=(0.20, 0.22, 0.24, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.56, 0.58, 0.60, 1.0))
    tray_face = model.material("tray_face", rgba=(0.22, 0.23, 0.24, 1.0))
    handle_silver = model.material("handle_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.42, 0.44, 0.47, 1.0))
    door_shell = model.material("door_shell", rgba=(0.17, 0.18, 0.19, 1.0))
    door_accent = model.material("door_accent", rgba=(0.32, 0.35, 0.37, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body_depth = 0.300
    body_width = 0.190
    body_height = 0.500
    wall_th = 0.008
    top_th = 0.010
    bottom_th = 0.010
    back_th = 0.006
    front_frame_th = 0.014

    opening_bottom = 0.072
    opening_top = 0.452
    opening_height = opening_top - opening_bottom
    opening_width = body_width - 2.0 * wall_th - 0.006

    rail_start_x = -0.128
    rail_length = 0.248
    rail_width = 0.006
    rail_height = 0.010
    rail_center_y = body_width * 0.5 - wall_th - rail_width * 0.5

    tray_depth = 0.232
    tray_width = 0.146
    tray_height = 0.046
    tray_bezel_depth = 0.012
    tray_bezel_width = 0.152
    tray_bezel_height = 0.060
    tray_closed_origin_x = -0.110
    tray_travel = 0.160
    door_width = opening_width + 0.002
    door_height = opening_height + 0.004
    door_th = 0.012
    hinge_radius = 0.005
    hinge_y = -door_width * 0.5 - 0.004

    tray_pitch = 0.066
    tray_bottoms = [0.086 + bay_index * tray_pitch for bay_index in range(5)]

    chassis = model.part("chassis")
    chassis.visual(
        Box((body_depth, body_width, bottom_th)),
        origin=Origin(xyz=(0.0, 0.0, bottom_th * 0.5)),
        material=chassis_dark,
        name="base_pan",
    )
    chassis.visual(
        Box((body_depth, wall_th, body_height)),
        origin=Origin(
            xyz=(0.0, -(body_width * 0.5 - wall_th * 0.5), body_height * 0.5)
        ),
        material=chassis_panel,
        name="left_wall",
    )
    chassis.visual(
        Box((body_depth, wall_th, body_height)),
        origin=Origin(
            xyz=(0.0, body_width * 0.5 - wall_th * 0.5, body_height * 0.5)
        ),
        material=chassis_panel,
        name="right_wall",
    )
    chassis.visual(
        Box((back_th, body_width - 2.0 * wall_th, body_height)),
        origin=Origin(
            xyz=(-body_depth * 0.5 + back_th * 0.5, 0.0, body_height * 0.5)
        ),
        material=chassis_panel,
        name="rear_panel",
    )
    chassis.visual(
        Box((body_depth, body_width, top_th)),
        origin=Origin(xyz=(0.0, 0.0, body_height - top_th * 0.5)),
        material=chassis_dark,
        name="top_cover",
    )
    chassis.visual(
        Box((front_frame_th, body_width - 2.0 * wall_th, opening_bottom - bottom_th)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 - front_frame_th * 0.5,
                0.0,
                bottom_th + (opening_bottom - bottom_th) * 0.5,
            )
        ),
        material=chassis_panel,
        name="front_sill",
    )
    chassis.visual(
        Box((front_frame_th, body_width - 2.0 * wall_th, body_height - top_th - opening_top)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 - front_frame_th * 0.5,
                0.0,
                opening_top + (body_height - top_th - opening_top) * 0.5,
            )
        ),
        material=chassis_panel,
        name="front_header",
    )
    chassis.visual(
        Box((0.024, opening_width - 0.020, opening_height - 0.024)),
        origin=Origin(
            xyz=(
                -body_depth * 0.5 + 0.012,
                0.0,
                opening_bottom + opening_height * 0.5,
            )
        ),
        material=chassis_dark,
        name="drive_backplane",
    )
    for hinge_index, hinge_z in enumerate((opening_bottom + 0.053, opening_bottom + door_height * 0.5, opening_bottom + door_height - 0.057)):
        chassis.visual(
            Cylinder(radius=hinge_radius, length=0.040),
            origin=Origin(
                xyz=(body_depth * 0.5 - 0.003, hinge_y, hinge_z)
            ),
            material=door_accent,
            name=f"hinge_mount_{hinge_index + 1}",
        )
    for foot_index, (foot_x, foot_y) in enumerate(
        (
            (-0.100, -0.060),
            (-0.100, 0.060),
            (0.100, -0.060),
            (0.100, 0.060),
        )
    ):
        chassis.visual(
            Box((0.028, 0.028, 0.008)),
            origin=Origin(xyz=(foot_x, foot_y, 0.004)),
            material=foot_rubber,
            name=f"foot_{foot_index + 1}",
        )

    for bay_index, bay_bottom in enumerate(tray_bottoms):
        rail_z = bay_bottom + 0.006
        rail_x = rail_start_x + rail_length * 0.5
        chassis.visual(
            Box((rail_length, rail_width, rail_height)),
            origin=Origin(xyz=(rail_x, -rail_center_y, rail_z)),
            material=rail_steel,
            name=f"left_guide_rail_{bay_index + 1}",
        )
        chassis.visual(
            Box((rail_length, rail_width, rail_height)),
            origin=Origin(xyz=(rail_x, rail_center_y, rail_z)),
            material=rail_steel,
            name=f"right_guide_rail_{bay_index + 1}",
        )

    chassis.inertial = Inertial.from_geometry(
        Box((body_depth, body_width, body_height)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    front_door = model.part("front_door")
    front_door.visual(
        Box((door_th, door_width, door_height)),
        origin=Origin(xyz=(door_th * 0.5, door_width * 0.5, door_height * 0.5)),
        material=door_shell,
        name="door_panel",
    )
    front_door.visual(
        Box((0.004, door_width - 0.032, door_height - 0.040)),
        origin=Origin(xyz=(door_th + 0.002, door_width * 0.5, door_height * 0.5)),
        material=door_accent,
        name="door_center_spine",
    )
    front_door.visual(
        Box((0.010, 0.010, door_height - 0.080)),
        origin=Origin(
            xyz=(door_th + 0.004, door_width - 0.026, door_height * 0.5)
        ),
        material=handle_silver,
        name="door_pull",
    )
    for hinge_index, hinge_z in enumerate((0.055, door_height * 0.5, door_height - 0.055)):
        front_door.visual(
            Cylinder(radius=hinge_radius, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=door_accent,
            name=f"hinge_barrel_{hinge_index + 1}",
        )
    front_door.inertial = Inertial.from_geometry(
        Box((door_th + 0.012, door_width, door_height)),
        mass=1.1,
        origin=Origin(xyz=((door_th + 0.012) * 0.5, door_width * 0.5, door_height * 0.5)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(
            xyz=(
                body_depth * 0.5 + 0.002 + hinge_radius,
                hinge_y,
                opening_bottom - 0.002,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=0.0, upper=1.45),
    )

    for tray_number, bay_bottom in enumerate(reversed(tray_bottoms), start=1):
        tray = model.part(f"tray_{tray_number}")
        _add_tray_visuals(
            tray,
            tray_depth=tray_depth,
            tray_width=tray_width,
            tray_height=tray_height,
            bezel_depth=tray_bezel_depth,
            bezel_width=tray_bezel_width,
            bezel_height=tray_bezel_height,
            material_body=tray_metal,
            material_bezel=tray_face,
            material_handle=handle_silver,
            index=tray_number,
        )
        model.articulation(
            f"tray_{tray_number}_slide",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=tray,
            origin=Origin(xyz=(tray_closed_origin_x, 0.0, bay_bottom)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=0.18,
                lower=0.0,
                upper=tray_travel,
            ),
        )

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

    chassis = object_model.get_part("chassis")
    front_door = object_model.get_part("front_door")
    door_hinge = object_model.get_articulation("door_hinge")

    ctx.expect_overlap(
        front_door,
        chassis,
        axes="yz",
        min_overlap=0.150,
        elem_a="door_panel",
        name="front door covers the front bay opening",
    )

    closed_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_panel")
    ctx.check(
        "front door sits just proud of the chassis face",
        closed_door_aabb is not None
        and closed_door_aabb[0][0] > 0.151
        and closed_door_aabb[0][0] < 0.158,
        details=f"closed={closed_door_aabb}",
    )
    with ctx.pose({door_hinge: 1.30}):
        opened_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_panel")
    ctx.check(
        "front door swings outward on its hinge",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][0] > closed_door_aabb[1][0] + 0.100,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    tray_1 = object_model.get_part("tray_1")
    tray_1_slide = object_model.get_articulation("tray_1_slide")
    with ctx.pose({door_hinge: 1.55, tray_1_slide: 0.150}):
        open_door_aabb = ctx.part_world_aabb(front_door)
        extended_tray_aabb = ctx.part_world_aabb(tray_1)
    ctx.check(
        "opened door clears an extended drive tray",
        open_door_aabb is not None
        and extended_tray_aabb is not None
        and open_door_aabb[1][1] < extended_tray_aabb[0][1],
        details=f"door={open_door_aabb}, tray={extended_tray_aabb}",
    )

    for tray_number in range(1, 6):
        tray = object_model.get_part(f"tray_{tray_number}")
        slide = object_model.get_articulation(f"tray_{tray_number}_slide")

        ctx.expect_within(
            tray,
            chassis,
            axes="yz",
            margin=0.004,
            name=f"tray {tray_number} stays laterally aligned in its bay at rest",
        )

        rest_position = ctx.part_world_position(tray)
        with ctx.pose({slide: 0.150}):
            ctx.expect_within(
                tray,
                chassis,
                axes="yz",
                margin=0.004,
                name=f"tray {tray_number} stays on the guide rails when extended",
            )
            ctx.expect_overlap(
                tray,
                chassis,
                axes="x",
                min_overlap=0.080,
                name=f"tray {tray_number} retains insertion when extended",
            )
            extended_position = ctx.part_world_position(tray)

        ctx.check(
            f"tray {tray_number} extends forward along the bay rails",
            rest_position is not None
            and extended_position is not None
            and extended_position[0] > rest_position[0] + 0.120,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
