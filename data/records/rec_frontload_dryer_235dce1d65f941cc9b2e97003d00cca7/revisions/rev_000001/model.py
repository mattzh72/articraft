from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    name: str,
) -> object:
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        thickness,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _dryer_drum_shell_mesh(name: str) -> object:
    drum_radius = 0.276
    drum_length = 0.440

    shell = CylinderGeometry(
        drum_radius,
        drum_length,
        radial_segments=56,
        closed=False,
    )
    shell.rotate_x(math.pi / 2.0)

    back_cap = CylinderGeometry(
        drum_radius - 0.016,
        0.006,
        radial_segments=56,
    )
    back_cap.rotate_x(math.pi / 2.0).translate(0.0, 0.217, 0.0)
    shell.merge(back_cap)

    front_rim = TorusGeometry(
        drum_radius - 0.012,
        0.008,
        radial_segments=18,
        tubular_segments=48,
    )
    front_rim.rotate_x(math.pi / 2.0).translate(0.0, -0.212, 0.0)
    shell.merge(front_rim)

    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        paddle = BoxGeometry((0.020, 0.280, 0.075))
        paddle.translate(drum_radius - 0.050, 0.0, 0.0).rotate_y(angle)
        shell.merge(paddle)

    return mesh_from_geometry(shell, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="condenser_tumble_dryer")

    appliance_white = model.material("appliance_white", rgba=(0.95, 0.95, 0.96, 1.0))
    soft_grey = model.material("soft_grey", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.52, 0.68, 0.80, 0.28))
    display_glass = model.material("display_glass", rgba=(0.11, 0.15, 0.18, 0.85))
    water_window = model.material("water_window", rgba=(0.48, 0.72, 0.88, 0.45))

    body_width = 0.598
    body_depth = 0.625
    body_height = 0.850
    shell_thickness = 0.018
    front_fascia_depth = 0.024
    front_y = -body_depth / 2.0
    opening_center_z = 0.430
    door_axis_x = -0.255
    door_axis_y = front_y - 0.018
    door_center_offset = 0.255
    drawer_closed_y = front_y + 0.009
    drawer_center_z = 0.053
    drum_center_y = -0.005

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(xyz=(-body_width / 2.0 + shell_thickness / 2.0, 0.0, body_height / 2.0)),
        material=appliance_white,
        name="left_side_panel",
    )
    cabinet_body.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(xyz=(body_width / 2.0 - shell_thickness / 2.0, 0.0, body_height / 2.0)),
        material=appliance_white,
        name="right_side_panel",
    )
    cabinet_body.visual(
        Box((body_width - 2.0 * shell_thickness, body_depth, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=appliance_white,
        name="bottom_panel",
    )
    cabinet_body.visual(
        Box((body_width - 2.0 * shell_thickness, body_depth - shell_thickness, shell_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -shell_thickness / 2.0,
                body_height - shell_thickness / 2.0,
            )
        ),
        material=appliance_white,
        name="top_panel",
    )
    rear_panel_y = body_depth / 2.0 - shell_thickness / 2.0
    cabinet_body.visual(
        Box((body_width - 2.0 * shell_thickness, shell_thickness, 0.360)),
        origin=Origin(xyz=(0.0, rear_panel_y, 0.180)),
        material=appliance_white,
        name="rear_lower_panel",
    )
    cabinet_body.visual(
        Box((body_width - 2.0 * shell_thickness, shell_thickness, 0.350)),
        origin=Origin(xyz=(0.0, rear_panel_y, 0.675)),
        material=appliance_white,
        name="rear_upper_panel",
    )
    cabinet_body.visual(
        Box((0.230, shell_thickness, 0.144)),
        origin=Origin(xyz=(-0.166, rear_panel_y, opening_center_z)),
        material=appliance_white,
        name="rear_left_panel",
    )
    cabinet_body.visual(
        Box((0.230, shell_thickness, 0.144)),
        origin=Origin(xyz=(0.166, rear_panel_y, opening_center_z)),
        material=appliance_white,
        name="rear_right_panel",
    )
    cabinet_body.visual(
        Box((body_width, front_fascia_depth, 0.228)),
        origin=Origin(xyz=(0.0, front_y + front_fascia_depth / 2.0, 0.736)),
        material=appliance_white,
        name="control_fascia",
    )
    cabinet_body.visual(
        Box((0.086, front_fascia_depth, 0.532)),
        origin=Origin(xyz=(-0.256, front_y + front_fascia_depth / 2.0, 0.356)),
        material=appliance_white,
        name="front_left_stile",
    )
    cabinet_body.visual(
        Box((0.086, front_fascia_depth, 0.532)),
        origin=Origin(xyz=(0.256, front_y + front_fascia_depth / 2.0, 0.356)),
        material=appliance_white,
        name="front_right_stile",
    )
    cabinet_body.visual(
        Box((body_width, front_fascia_depth, 0.150)),
        origin=Origin(xyz=(0.0, front_y + front_fascia_depth / 2.0, 0.165)),
        material=appliance_white,
        name="lower_front_rail",
    )
    cabinet_body.visual(
        Box((body_width - 2.0 * shell_thickness, 0.160, 0.004)),
        origin=Origin(xyz=(0.0, -0.217, 0.014)),
        material=soft_grey,
        name="plinth_support_deck",
    )
    cabinet_body.visual(
        _annulus_mesh(
            outer_radius=0.205,
            inner_radius=0.178,
            thickness=0.042,
            name="dryer_opening_gasket",
        ),
        origin=Origin(xyz=(0.0, -0.292, opening_center_z)),
        material=dark_trim,
        name="opening_gasket",
    )
    cabinet_body.visual(
        Cylinder(radius=0.042, length=0.004),
        origin=Origin(
            xyz=(-0.175, front_y - 0.0015, 0.736),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=soft_grey,
        name="program_knob_bezel",
    )
    cabinet_body.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(
            xyz=(-0.175, front_y - 0.013, 0.736),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=soft_grey,
        name="program_knob",
    )
    cabinet_body.visual(
        Box((0.132, 0.006, 0.050)),
        origin=Origin(xyz=(0.162, front_y + 0.003, 0.742)),
        material=display_glass,
        name="display_window",
    )
    cabinet_body.visual(
        Box((0.112, 0.010, 0.012)),
        origin=Origin(xyz=(0.162, front_y - 0.002, 0.704)),
        material=dark_trim,
        name="button_strip",
    )
    for hinge_name, hinge_z in (("upper", 0.540), ("lower", 0.320)):
        cabinet_body.visual(
            Box((0.018, 0.024, 0.070)),
            origin=Origin(xyz=(door_axis_x - 0.028, door_axis_y + 0.0095, hinge_z)),
            material=soft_grey,
            name=f"{hinge_name}_hinge_mount",
        )
        cabinet_body.visual(
            Cylinder(radius=0.009, length=0.014),
            origin=Origin(xyz=(door_axis_x - 0.019, door_axis_y, hinge_z - 0.016)),
            material=soft_grey,
            name=f"{hinge_name}_hinge_knuckle_bottom",
        )
        cabinet_body.visual(
            Cylinder(radius=0.009, length=0.014),
            origin=Origin(xyz=(door_axis_x - 0.019, door_axis_y, hinge_z + 0.016)),
            material=soft_grey,
            name=f"{hinge_name}_hinge_knuckle_top",
        )
    cabinet_body.visual(
        _annulus_mesh(
            outer_radius=0.062,
            inner_radius=0.020,
            thickness=shell_thickness,
            name="dryer_rear_bearing_ring",
        ),
        origin=Origin(xyz=(0.0, rear_panel_y, opening_center_z)),
        material=appliance_white,
        name="rear_bearing_ring",
    )
    cabinet_body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    drum = model.part("drum")
    drum.visual(
        _dryer_drum_shell_mesh("dryer_drum_shell"),
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.028, length=0.0795),
        origin=Origin(xyz=(0.0, 0.25975, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.0, 0.2845, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="rear_axle_stub",
    )
    drum.inertial = Inertial.from_geometry(
        Box((0.560, 0.440, 0.560)),
        mass=7.5,
        origin=Origin(),
    )

    door = model.part("door")
    door.visual(
        _annulus_mesh(
            outer_radius=0.247,
            inner_radius=0.172,
            thickness=0.028,
            name="dryer_door_outer_ring",
        ),
        origin=Origin(xyz=(door_center_offset, 0.0, 0.0)),
        material=appliance_white,
        name="door_outer_ring",
    )
    door.visual(
        _annulus_mesh(
            outer_radius=0.180,
            inner_radius=0.148,
            thickness=0.012,
            name="dryer_door_inner_trim",
        ),
        origin=Origin(xyz=(door_center_offset, 0.007, 0.0)),
        material=dark_trim,
        name="door_inner_trim",
    )
    door.visual(
        Cylinder(radius=0.170, length=0.006),
        origin=Origin(
            xyz=(door_center_offset, 0.001, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=tinted_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.060, 0.018, 0.024)),
        origin=Origin(xyz=(door_center_offset + 0.177, -0.020, -0.054)),
        material=soft_grey,
        name="door_handle",
    )
    for hinge_name, local_z in (("upper", 0.110), ("lower", -0.110)):
        door.visual(
            Box((0.036, 0.016, 0.070)),
            origin=Origin(xyz=(0.018, 0.0, local_z)),
            material=soft_grey,
            name=f"{hinge_name}_hinge_leaf",
        )
        door.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, local_z)),
            material=soft_grey,
            name=f"{hinge_name}_hinge_barrel",
        )
    door.inertial = Inertial.from_geometry(
        Box((0.500, 0.060, 0.500)),
        mass=3.2,
        origin=Origin(xyz=(door_center_offset, 0.0, 0.0)),
    )

    water_drawer = model.part("water_drawer")
    water_drawer.visual(
        Box((0.558, 0.018, 0.074)),
        material=appliance_white,
        name="drawer_front",
    )
    water_drawer.visual(
        Box((0.220, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.014, 0.020)),
        material=dark_trim,
        name="drawer_pull",
    )
    water_drawer.visual(
        Box((0.088, 0.004, 0.038)),
        origin=Origin(xyz=(0.191, -0.011, 0.0)),
        material=water_window,
        name="drawer_level_window",
    )
    water_drawer.visual(
        Box((0.542, 0.150, 0.004)),
        origin=Origin(xyz=(0.0, 0.084, -0.035)),
        material=soft_grey,
        name="drawer_floor",
    )
    water_drawer.visual(
        Box((0.004, 0.150, 0.050)),
        origin=Origin(xyz=(-0.269, 0.084, -0.009)),
        material=soft_grey,
        name="drawer_left_wall",
    )
    water_drawer.visual(
        Box((0.004, 0.150, 0.050)),
        origin=Origin(xyz=(0.269, 0.084, -0.009)),
        material=soft_grey,
        name="drawer_right_wall",
    )
    water_drawer.visual(
        Box((0.542, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, 0.159, -0.009)),
        material=soft_grey,
        name="drawer_back_wall",
    )
    water_drawer.inertial = Inertial.from_geometry(
        Box((0.558, 0.168, 0.074)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.070, -0.010)),
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=drum,
        origin=Origin(xyz=(0.0, drum_center_y, opening_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=8.0,
            lower=-6.0,
            upper=6.0,
        ),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=door,
        origin=Origin(xyz=(door_axis_x, door_axis_y, opening_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-1.75,
            upper=0.0,
        ),
    )
    model.articulation(
        "body_to_water_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet_body,
        child=water_drawer,
        origin=Origin(xyz=(0.0, drawer_closed_y, drawer_center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=0.110,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet_body = object_model.get_part("cabinet_body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    water_drawer = object_model.get_part("water_drawer")

    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    drawer_joint = object_model.get_articulation("body_to_water_drawer")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "drum axle runs front-to-back",
        tuple(drum_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected drum axis (0, 1, 0), got {drum_joint.axis!r}",
    )
    ctx.check(
        "door uses left-edge vertical hinge",
        tuple(door_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected door axis (0, 0, 1), got {door_joint.axis!r}",
    )
    ctx.check(
        "drawer slides out of base plinth",
        tuple(drawer_joint.axis) == (0.0, -1.0, 0.0),
        f"Expected drawer axis (0, -1, 0), got {drawer_joint.axis!r}",
    )

    door_visual_names = {visual.name for visual in door.visuals}
    body_visual_names = {visual.name for visual in cabinet_body.visuals}
    ctx.check(
        "two barrel hinges are modeled",
        {
            "upper_hinge_barrel",
            "lower_hinge_barrel",
            "upper_hinge_leaf",
            "lower_hinge_leaf",
        }.issubset(door_visual_names)
        and {
            "upper_hinge_mount",
            "lower_hinge_mount",
            "upper_hinge_knuckle_top",
            "lower_hinge_knuckle_top",
        }.issubset(body_visual_names),
        "Expected paired upper and lower barrel hinge visuals on the door and cabinet.",
    )

    ctx.expect_contact(
        drum,
        cabinet_body,
        elem_a="rear_hub",
        elem_b="rear_bearing_ring",
        name="drum rear hub is seated against the rear bearing ring",
    )
    ctx.expect_contact(
        water_drawer,
        cabinet_body,
        elem_a="drawer_floor",
        elem_b="plinth_support_deck",
        name="drawer rides on plinth support deck",
    )
    ctx.expect_contact(
        door,
        cabinet_body,
        elem_a="upper_hinge_barrel",
        elem_b="upper_hinge_knuckle_top",
        name="upper barrel hinge is engaged with the cabinet knuckle",
    )
    ctx.expect_contact(
        door,
        cabinet_body,
        elem_a="lower_hinge_barrel",
        elem_b="lower_hinge_knuckle_top",
        name="lower barrel hinge is engaged with the cabinet knuckle",
    )
    ctx.expect_gap(
        cabinet_body,
        door,
        axis="y",
        positive_elem="opening_gasket",
        negative_elem="door_outer_ring",
        min_gap=0.0,
        max_gap=0.020,
        name="closed door sits just proud of the opening gasket",
    )
    ctx.expect_overlap(
        door,
        cabinet_body,
        axes="xz",
        elem_a="door_outer_ring",
        elem_b="opening_gasket",
        min_overlap=0.320,
        name="door porthole covers the opening",
    )
    ctx.expect_within(
        drum,
        cabinet_body,
        axes="xz",
        margin=0.0,
        name="drum stays inside cabinet width and height envelope",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(door, elem="door_handle")
    closed_drawer_front_aabb = ctx.part_element_world_aabb(water_drawer, elem="drawer_front")
    closed_drawer_pos = ctx.part_world_position(water_drawer)

    ctx.check(
        "drawer sits in the base plinth",
        closed_drawer_pos is not None and closed_drawer_pos[2] < 0.10,
        f"Expected drawer origin near the base plinth, got {closed_drawer_pos!r}",
    )

    with ctx.pose({door_joint: -1.25}):
        open_handle_aabb = ctx.part_element_world_aabb(door, elem="door_handle")
        ctx.expect_contact(
            door,
            cabinet_body,
            elem_a="upper_hinge_barrel",
            elem_b="upper_hinge_knuckle_top",
            name="upper barrel hinge stays engaged when the door opens",
        )
        ctx.check(
            "door swings outward to the left",
            closed_handle_aabb is not None
            and open_handle_aabb is not None
            and open_handle_aabb[0][1] < closed_handle_aabb[0][1] - 0.12,
            f"Expected opened door handle to move forward from {closed_handle_aabb!r} to {open_handle_aabb!r}",
        )

    with ctx.pose({drawer_joint: 0.090}):
        open_drawer_front_aabb = ctx.part_element_world_aabb(water_drawer, elem="drawer_front")
        ctx.expect_contact(
            water_drawer,
            cabinet_body,
            elem_a="drawer_floor",
            elem_b="plinth_support_deck",
            name="drawer remains guided by the plinth deck when extended",
        )
        ctx.check(
            "drawer extends forward",
            closed_drawer_front_aabb is not None
            and open_drawer_front_aabb is not None
            and open_drawer_front_aabb[0][1] < closed_drawer_front_aabb[0][1] - 0.08,
            f"Expected drawer front to move forward from {closed_drawer_front_aabb!r} to {open_drawer_front_aabb!r}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
