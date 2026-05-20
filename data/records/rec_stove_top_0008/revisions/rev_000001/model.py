from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def _build_worktop_mesh(width: float, depth: float, opening_width: float, opening_depth: float):
    geom = ExtrudeWithHolesGeometry(
        _rect_profile(width, depth),
        [_rect_profile(opening_width, opening_depth)],
        height=0.03,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path("stone_worktop.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="induction_cooktop_cabinet", assets=ASSETS)

    stone = model.material("stone", rgba=(0.77, 0.77, 0.74, 1.0))
    stone_edge = model.material("stone_edge", rgba=(0.67, 0.67, 0.64, 1.0))
    cabinet_paint = model.material("cabinet_paint", rgba=(0.70, 0.72, 0.74, 1.0))
    cabinet_shadow = model.material("cabinet_shadow", rgba=(0.42, 0.45, 0.47, 1.0))
    cooktop_glass = model.material("cooktop_glass", rgba=(0.08, 0.09, 0.10, 1.0))
    cooktop_mark = model.material("cooktop_mark", rgba=(0.42, 0.48, 0.52, 0.55))
    cooktop_metal = model.material("cooktop_metal", rgba=(0.30, 0.32, 0.34, 1.0))
    button_finish = model.material("button_finish", rgba=(0.82, 0.84, 0.86, 1.0))
    pull_finish = model.material("pull_finish", rgba=(0.18, 0.19, 0.20, 1.0))

    cabinet_width = 0.84
    cabinet_depth = 0.58
    cabinet_height = 0.72
    side_thickness = 0.018
    toe_kick_height = 0.10
    toe_kick_depth = 0.07
    top_rail_height = 0.05
    inner_width = cabinet_width - (2.0 * side_thickness)
    door_width = 0.760
    door_height = 0.556
    door_thickness = 0.022
    hinge_barrel_radius = 0.006
    hinge_barrel_length = 0.780

    worktop_width = 0.90
    worktop_depth = 0.64
    worktop_thickness = 0.03
    cooktop_opening_width = 0.724
    cooktop_opening_depth = 0.464

    glass_width = 0.716
    glass_depth = 0.456
    glass_thickness = 0.008
    cooktop_recess = 0.002
    flange_width = 0.760
    flange_depth = 0.500
    flange_thickness = 0.006
    shell_wall = 0.010
    shell_height = 0.020
    underbody_width = 0.700
    underbody_depth = 0.414
    underbody_height = 0.112
    front_wall_y = -0.212
    back_wall_y = 0.212
    shell_wall_z = -0.020
    underbody_z = -0.086
    button_column_x = 0.286

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-((cabinet_width * 0.5) - (side_thickness * 0.5)), 0.0, cabinet_height * 0.5)),
        material=cabinet_paint,
        name="left_side",
    )
    cabinet_body.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=((cabinet_width * 0.5) - (side_thickness * 0.5), 0.0, cabinet_height * 0.5)),
        material=cabinet_paint,
        name="right_side",
    )
    cabinet_body.visual(
        Box((inner_width, 0.560, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, toe_kick_height + 0.009)),
        material=cabinet_shadow,
        name="bottom_panel",
    )
    cabinet_body.visual(
        Box((inner_width, 0.012, cabinet_height - toe_kick_height)),
        origin=Origin(
            xyz=(
                0.0,
                (cabinet_depth * 0.5) - 0.006,
                toe_kick_height + ((cabinet_height - toe_kick_height) * 0.5),
            )
        ),
        material=cabinet_paint,
        name="back_panel",
    )
    cabinet_body.visual(
        Box((inner_width, 0.050, 0.028)),
        origin=Origin(xyz=(0.0, -((cabinet_depth * 0.5) - 0.025), 0.646)),
        material=cabinet_paint,
        name="front_header",
    )
    top_stretcher_width = 0.016
    top_stretcher_depth = 0.500
    top_stretcher_height = top_rail_height
    top_stretcher_x = (cabinet_width * 0.5) - side_thickness - (top_stretcher_width * 0.5)
    cabinet_body.visual(
        Box((top_stretcher_width, top_stretcher_depth, top_stretcher_height)),
        origin=Origin(xyz=(-top_stretcher_x, 0.0, cabinet_height - (top_stretcher_height * 0.5))),
        material=cabinet_paint,
        name="left_top_stretcher",
    )
    cabinet_body.visual(
        Box((top_stretcher_width, top_stretcher_depth, top_stretcher_height)),
        origin=Origin(xyz=(top_stretcher_x, 0.0, cabinet_height - (top_stretcher_height * 0.5))),
        material=cabinet_paint,
        name="right_top_stretcher",
    )
    cabinet_body.visual(
        Box((inner_width, toe_kick_depth, toe_kick_height)),
        origin=Origin(xyz=(0.0, -((cabinet_depth * 0.5) - (toe_kick_depth * 0.5)), toe_kick_height * 0.5)),
        material=cabinet_shadow,
        name="toe_kick",
    )
    hinge_block_width = 0.024
    hinge_block_depth = 0.018
    hinge_block_height = 0.018
    cabinet_body.visual(
        Box((hinge_block_width, hinge_block_depth, hinge_block_height)),
        origin=Origin(
            xyz=(
                -((hinge_barrel_length * 0.5) + (hinge_block_width * 0.5) - 0.002),
                -(cabinet_depth * 0.5),
                toe_kick_height + hinge_barrel_radius,
            )
        ),
        material=cabinet_shadow,
        name="left_hinge_block",
    )
    cabinet_body.visual(
        Box((hinge_block_width, hinge_block_depth, hinge_block_height)),
        origin=Origin(
            xyz=(
                (hinge_barrel_length * 0.5) + (hinge_block_width * 0.5) - 0.002,
                -(cabinet_depth * 0.5),
                toe_kick_height + hinge_barrel_radius,
            )
        ),
        material=cabinet_shadow,
        name="right_hinge_block",
    )
    cabinet_body.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    worktop = model.part("worktop")
    worktop.visual(
        _build_worktop_mesh(
            worktop_width,
            worktop_depth,
            cooktop_opening_width,
            cooktop_opening_depth,
        ),
        origin=Origin(xyz=(0.0, 0.0, worktop_thickness * 0.5)),
        material=stone,
        name="stone_slab",
    )
    worktop.inertial = Inertial.from_geometry(
        Box((worktop_width, worktop_depth, worktop_thickness)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, worktop_thickness * 0.5)),
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        Box((flange_width, flange_depth, flange_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -worktop_thickness - (flange_thickness * 0.5))),
        material=cooktop_metal,
        name="mount_flange",
    )
    cooktop.visual(
        Box((underbody_width, underbody_depth, underbody_height)),
        origin=Origin(xyz=(0.0, 0.0, underbody_z)),
        material=cooktop_metal,
        name="underbody",
    )
    cooktop.visual(
        Box((glass_width, glass_depth, glass_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -cooktop_recess - (glass_thickness * 0.5))),
        material=cooktop_glass,
        name="glass_top",
    )
    cooktop.visual(
        Box((shell_wall, 0.434, shell_height)),
        origin=Origin(xyz=(-(glass_width * 0.5) + (shell_wall * 0.5), 0.0, shell_wall_z)),
        material=cooktop_metal,
        name="left_wall",
    )
    cooktop.visual(
        Box((shell_wall, 0.434, shell_height)),
        origin=Origin(xyz=((glass_width * 0.5) - (shell_wall * 0.5), 0.0, shell_wall_z)),
        material=cooktop_metal,
        name="right_wall",
    )
    cooktop.visual(
        Box((glass_width - (2.0 * shell_wall), shell_wall, shell_height)),
        origin=Origin(xyz=(0.0, front_wall_y, shell_wall_z)),
        material=cooktop_metal,
        name="front_wall",
    )
    cooktop.visual(
        Box((glass_width - (2.0 * shell_wall), shell_wall, shell_height)),
        origin=Origin(xyz=(0.0, back_wall_y, shell_wall_z)),
        material=cooktop_metal,
        name="back_wall",
    )
    zone_specs = (
        ("zone_front_left", (-0.150, -0.105, -0.0016), 0.105),
        ("zone_rear_left", (-0.150, 0.110, -0.0016), 0.095),
        ("zone_front_right", (0.115, -0.105, -0.0016), 0.090),
        ("zone_rear_right", (0.115, 0.110, -0.0016), 0.105),
    )
    for zone_name, zone_xyz, zone_radius in zone_specs:
        cooktop.visual(
            Cylinder(radius=zone_radius, length=0.0008),
            origin=Origin(xyz=zone_xyz),
            material=cooktop_mark,
            name=zone_name,
        )
    cooktop.visual(
        Box((0.004, 0.012, 0.078)),
        origin=Origin(xyz=(button_column_x - 0.015, -0.223, -0.049)),
        material=cooktop_metal,
        name="control_left_rail",
    )
    cooktop.visual(
        Box((0.004, 0.012, 0.078)),
        origin=Origin(xyz=(button_column_x + 0.015, -0.223, -0.049)),
        material=cooktop_metal,
        name="control_right_rail",
    )
    cooktop.visual(
        Box((0.038, 0.006, 0.078)),
        origin=Origin(xyz=(button_column_x, -0.214, -0.049)),
        material=stone_edge,
        name="control_backplate",
    )
    cooktop.inertial = Inertial.from_geometry(
        Box((flange_width, flange_depth, 0.140)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    door = model.part("access_door")
    door.visual(
        Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=cabinet_shadow,
        name="hinge_barrel",
    )
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(door_thickness * 0.5),
                hinge_barrel_radius + (door_height * 0.5),
            )
        ),
        material=cabinet_paint,
        name="door_panel",
    )
    door.visual(
        Box((0.220, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -(door_thickness + 0.005), 0.485)),
        material=pull_finish,
        name="pull_strip",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height + 0.012)),
        mass=7.0,
        origin=Origin(xyz=(0.0, -(door_thickness * 0.5), 0.285)),
    )

    for button_name, button_z in (
        ("button_upper", -0.024),
        ("button_middle", -0.049),
        ("button_lower", -0.074),
    ):
        button = model.part(button_name)
        button.visual(
            Box((0.026, 0.008, 0.012)),
            material=button_finish,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.026, 0.008, 0.012)),
            mass=0.03,
        )
        model.articulation(
            f"cooktop_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=cooktop,
            child=button,
            origin=Origin(xyz=(button_column_x, -0.225, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.040,
                lower=0.0,
                upper=0.003,
            ),
        )

    model.articulation(
        "cabinet_to_worktop",
        ArticulationType.FIXED,
        parent=cabinet_body,
        child=worktop,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height)),
    )
    model.articulation(
        "worktop_to_cooktop",
        ArticulationType.FIXED,
        parent=worktop,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, worktop_thickness)),
    )
    model.articulation(
        "cabinet_to_access_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=door,
        origin=Origin(xyz=(0.0, -(cabinet_depth * 0.5), toe_kick_height + hinge_barrel_radius)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    cabinet_body = object_model.get_part("cabinet_body")
    worktop = object_model.get_part("worktop")
    cooktop = object_model.get_part("cooktop")
    door = object_model.get_part("access_door")
    button_upper = object_model.get_part("button_upper")
    button_middle = object_model.get_part("button_middle")
    button_lower = object_model.get_part("button_lower")

    door_hinge = object_model.get_articulation("cabinet_to_access_door")
    button_joints = (
        object_model.get_articulation("cooktop_to_button_upper"),
        object_model.get_articulation("cooktop_to_button_middle"),
        object_model.get_articulation("cooktop_to_button_lower"),
    )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(worktop, cabinet_body, name="worktop_supported_by_cabinet")
    ctx.expect_contact(cooktop, worktop, name="cooktop_clamped_to_worktop")
    ctx.expect_contact(door, cabinet_body, name="door_hinge_contact_closed")
    ctx.expect_contact(button_upper, cooktop, name="upper_button_guided_at_rest")
    ctx.expect_contact(button_middle, cooktop, name="middle_button_guided_at_rest")
    ctx.expect_contact(button_lower, cooktop, name="lower_button_guided_at_rest")

    worktop_aabb = ctx.part_world_aabb(worktop)
    cooktop_aabb = ctx.part_world_aabb(cooktop)
    door_closed_aabb = ctx.part_world_aabb(door)
    assert worktop_aabb is not None
    assert cooktop_aabb is not None
    assert door_closed_aabb is not None

    recess_amount = worktop_aabb[1][2] - cooktop_aabb[1][2]
    ctx.check(
        "cooktop_recessed_below_stone_surface",
        0.001 <= recess_amount <= 0.0045,
        details=f"expected cooktop top to sit 1-4.5 mm below stone, got {recess_amount:.4f} m",
    )
    ctx.check(
        "door_closed_covers_opening_height",
        door_closed_aabb[0][2] <= 0.11 and door_closed_aabb[1][2] >= 0.66,
        details=(
            "flip-down door should cover the main opening from just above the toe kick "
            f"to near the underside of the worktop, got z-range {door_closed_aabb[0][2]:.3f}..{door_closed_aabb[1][2]:.3f}"
        ),
    )

    rest_positions = {
        button_upper.name: ctx.part_world_position(button_upper),
        button_middle.name: ctx.part_world_position(button_middle),
        button_lower.name: ctx.part_world_position(button_lower),
    }
    assert rest_positions[button_upper.name] is not None
    assert rest_positions[button_middle.name] is not None
    assert rest_positions[button_lower.name] is not None

    for button_part, joint in zip((button_upper, button_middle, button_lower), button_joints):
        limits = joint.motion_limits
        assert limits is not None
        assert limits.upper is not None
        with ctx.pose({joint.name: limits.upper}):
            pressed_position = ctx.part_world_position(button_part)
            assert pressed_position is not None
            rest_position = rest_positions[button_part.name]
            assert rest_position is not None
            ctx.check(
                f"{button_part.name}_moves_inward",
                pressed_position[1] > rest_position[1] + 0.0025,
                details=(
                    f"expected {button_part.name} to translate inward along +y by about 3 mm; "
                    f"rest={rest_position}, pressed={pressed_position}"
                ),
            )
            ctx.check(
                f"{button_part.name}_stays_aligned",
                abs(pressed_position[0] - rest_position[0]) < 1e-6 and abs(pressed_position[2] - rest_position[2]) < 1e-6,
                details=f"expected {button_part.name} to move only on its prismatic axis",
            )
            ctx.expect_contact(button_part, cooktop, name=f"{button_part.name}_guided_when_pressed")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    door_limits = door_hinge.motion_limits
    assert door_limits is not None
    assert door_limits.upper is not None
    with ctx.pose({door_hinge.name: door_limits.upper}):
        door_open_aabb = ctx.part_world_aabb(door)
        assert door_open_aabb is not None
        ctx.check(
            "door_swings_downward",
            door_open_aabb[1][2] < door_closed_aabb[1][2] - 0.30 and door_open_aabb[0][1] < door_closed_aabb[0][1] - 0.10,
            details=(
                "expected the wide access door to rotate down and outward; "
                f"closed={door_closed_aabb}, open={door_open_aabb}"
            ),
        )
        ctx.expect_contact(door, cabinet_body, name="door_hinge_contact_open")
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")

    with ctx.pose({door_hinge.name: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="door_closed_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
