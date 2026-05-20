from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_tower_workstation")

    steel = model.material("powder_coated_steel", rgba=(0.075, 0.080, 0.085, 1.0))
    panel_steel = model.material("slightly_lighter_steel", rgba=(0.105, 0.112, 0.120, 1.0))
    black = model.material("matte_black_plastic", rgba=(0.010, 0.011, 0.012, 1.0))
    dark_trim = model.material("dark_drive_bay_trim", rgba=(0.025, 0.027, 0.030, 1.0))
    hinge_metal = model.material("brushed_hinge_pin", rgba=(0.55, 0.56, 0.54, 1.0))
    mesh_black = model.material("black_vent_mesh", rgba=(0.018, 0.020, 0.022, 1.0))

    width = 0.240
    depth = 0.580
    height = 0.660
    wall = 0.010

    half_w = width / 2.0
    half_d = depth / 2.0

    chassis = model.part("chassis")

    def add_chassis_box(
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material: Material = steel,
    ) -> None:
        chassis.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    # Steel shell with a right-side service opening and a framed top vent aperture.
    add_chassis_box("left_wall", (wall, depth, height), (-half_w + wall / 2, 0.0, height / 2))
    add_chassis_box("rear_wall", (width, wall, height), (0.0, half_d - wall / 2, height / 2))
    add_chassis_box("bottom_pan", (width, depth, wall), (0.0, 0.0, wall / 2))
    add_chassis_box("top_left_rail", (wall, depth, wall), (-half_w + wall / 2, 0.0, height - wall / 2))
    add_chassis_box("top_right_rail", (wall, depth, wall), (half_w - wall / 2, 0.0, height - wall / 2))
    add_chassis_box("top_front_rail", (width, wall, wall), (0.0, -half_d + wall / 2, height - wall / 2))
    add_chassis_box("top_rear_rail", (width, wall, wall), (0.0, half_d - wall / 2, height - wall / 2))

    # Front bay cage and face frame behind the swinging front door.
    add_chassis_box("front_left_stile", (wall, wall, height), (-half_w + wall / 2, -half_d + wall / 2, height / 2))
    add_chassis_box("front_right_stile", (wall, wall, height), (half_w - wall / 2, -half_d + wall / 2, height / 2))
    add_chassis_box("front_header", (width, wall, 0.036), (0.0, -half_d + wall / 2, height - 0.018))
    add_chassis_box("front_kick", (width, wall, 0.055), (0.0, -half_d + wall / 2, 0.0275))
    for idx, z in enumerate((0.250, 0.310, 0.370, 0.430, 0.490)):
        add_chassis_box(f"bay_shelf_{idx}", (width, wall, 0.006), (0.0, -half_d + wall / 2, z), dark_trim)

    # Fixed hinge receivers mounted to the rear side edge, front left edge, and rear top rail.
    side_hinge_x = half_w + 0.014
    side_hinge_y = half_d - 0.016
    side_pin_r = 0.0048
    receiver_max_x = side_hinge_x - side_pin_r
    for idx, z in enumerate((0.165, 0.505)):
        add_chassis_box(
            f"side_socket_{idx}",
            (0.024, 0.020, 0.105),
            (receiver_max_x - 0.012, side_hinge_y + 0.006, z),
            hinge_metal,
        )

    front_hinge_x = -half_w - 0.012
    front_hinge_y = -half_d - 0.013
    front_pin_r = 0.004
    for idx, z in enumerate((0.300, 0.510)):
        add_chassis_box(
            f"front_socket_{idx}",
            (0.013, 0.030, 0.095),
            (front_hinge_x + front_pin_r + 0.0065, front_hinge_y + 0.008, z),
            hinge_metal,
        )

    top_hinge_y = half_d - 0.020
    top_hinge_z = height + 0.014
    top_pin_r = 0.004
    for idx, x in enumerate((-0.070, 0.070)):
        add_chassis_box(
            f"top_socket_{idx}",
            (0.055, 0.020, 0.015),
            (x, top_hinge_y + 0.006, top_hinge_z - top_pin_r - 0.0075),
            hinge_metal,
        )

    # Large right side panel, hinged at the rear on two visible vertical pins.
    side_panel = model.part("side_panel")
    side_panel.visual(
        Box((0.006, 0.525, 0.600)),
        origin=Origin(xyz=(-0.005, -0.2825, 0.330)),
        material=panel_steel,
        name="side_sheet",
    )
    side_panel.visual(
        Box((0.002, 0.430, 0.480)),
        origin=Origin(xyz=(-0.001, -0.2825, 0.330)),
        material=Material("pressed_side_inset", rgba=(0.075, 0.080, 0.086, 1.0)),
        name="pressed_inset",
    )
    side_panel.visual(
        Box((0.010, 0.030, 0.115)),
        origin=Origin(xyz=(0.000, -0.015, 0.165)),
        material=panel_steel,
        name="side_leaf_0",
    )
    side_panel.visual(
        Box((0.010, 0.030, 0.115)),
        origin=Origin(xyz=(0.000, -0.015, 0.505)),
        material=panel_steel,
        name="side_leaf_1",
    )
    for idx, z in enumerate((0.165, 0.505)):
        side_panel.visual(
            Cylinder(radius=side_pin_r, length=0.105),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hinge_metal,
            name=f"side_pin_{idx}",
        )
    side_panel.visual(
        Box((0.005, 0.055, 0.160)),
        origin=Origin(xyz=(0.002, -0.500, 0.355)),
        material=dark_trim,
        name="side_pull",
    )

    model.articulation(
        "side_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(side_hinge_x, side_hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=1.45),
    )

    # Front drive-bay door, hinged on the left edge and detailed with separate bay outlines.
    front_door = model.part("front_door")
    front_door.visual(
        Box((0.230, 0.008, 0.382)),
        origin=Origin(xyz=(0.133, -0.004, 0.390)),
        material=panel_steel,
        name="door_panel",
    )
    for idx, z in enumerate((0.282, 0.340, 0.398, 0.456, 0.514)):
        front_door.visual(
            Box((0.158, 0.003, 0.038)),
            origin=Origin(xyz=(0.126, -0.0095, z)),
            material=dark_trim,
            name=f"drive_bay_{idx}",
        )
    front_door.visual(
        Box((0.010, 0.005, 0.190)),
        origin=Origin(xyz=(0.232, -0.0095, 0.390)),
        material=black,
        name="front_handle",
    )
    for idx, z in enumerate((0.300, 0.510)):
        front_door.visual(
            Cylinder(radius=front_pin_r, length=0.095),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hinge_metal,
            name=f"front_pin_{idx}",
        )
        front_door.visual(
            Box((0.028, 0.006, 0.090)),
            origin=Origin(xyz=(0.014, -0.006, z)),
            material=panel_steel,
            name=f"front_leaf_{idx}",
        )

    model.articulation(
        "front_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(xyz=(front_hinge_x, front_hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=0.0, upper=1.75),
    )

    # Removable perforated top ventilation lid, hinged at the rear on two short pins.
    top_lid = model.part("top_lid")
    vent_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.208, 0.525),
            0.004,
            hole_diameter=0.006,
            pitch=(0.014, 0.014),
            frame=0.012,
            corner_radius=0.006,
            stagger=True,
        ),
        "top_vent_mesh",
    )
    top_lid.visual(
        vent_mesh,
        origin=Origin(xyz=(0.0, -0.265, 0.008)),
        material=mesh_black,
        name="vent_mesh",
    )
    top_lid.visual(
        Box((0.224, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.532, 0.008)),
        material=panel_steel,
        name="front_lip",
    )
    top_lid.visual(
        Box((0.010, 0.525, 0.006)),
        origin=Origin(xyz=(-0.112, -0.265, 0.008)),
        material=panel_steel,
        name="lid_side_0",
    )
    top_lid.visual(
        Box((0.010, 0.525, 0.006)),
        origin=Origin(xyz=(0.112, -0.265, 0.008)),
        material=panel_steel,
        name="lid_side_1",
    )
    for idx, x in enumerate((-0.070, 0.070)):
        top_lid.visual(
            Cylinder(radius=top_pin_r, length=0.045),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"top_pin_{idx}",
        )
        top_lid.visual(
            Box((0.052, 0.030, 0.006)),
            origin=Origin(xyz=(x, -0.017, 0.004)),
            material=panel_steel,
            name=f"top_leaf_{idx}",
        )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=top_lid,
        origin=Origin(xyz=(0.0, top_hinge_y, top_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("side_panel")
    front_door = object_model.get_part("front_door")
    top_lid = object_model.get_part("top_lid")
    side_hinge = object_model.get_articulation("side_hinge")
    front_hinge = object_model.get_articulation("front_hinge")
    top_hinge = object_model.get_articulation("top_hinge")

    ctx.expect_overlap(
        side_panel,
        chassis,
        axes="yz",
        elem_a="side_sheet",
        min_overlap=0.50,
        name="side panel spans the tower height and depth",
    )
    ctx.expect_overlap(
        front_door,
        chassis,
        axes="xz",
        elem_a="door_panel",
        min_overlap=0.20,
        name="front door covers the drive bay frame",
    )
    ctx.expect_within(
        top_lid,
        chassis,
        axes="xy",
        elem_a="vent_mesh",
        margin=0.004,
        name="top vent lid covers the top aperture",
    )

    closed_side = ctx.part_world_aabb(side_panel)
    closed_front = ctx.part_world_aabb(front_door)
    closed_top = ctx.part_world_aabb(top_lid)

    with ctx.pose({side_hinge: 1.0, front_hinge: 1.0, top_hinge: 0.9}):
        open_side = ctx.part_world_aabb(side_panel)
        open_front = ctx.part_world_aabb(front_door)
        open_top = ctx.part_world_aabb(top_lid)

    ctx.check(
        "side panel opens outward",
        closed_side is not None
        and open_side is not None
        and open_side[1][0] > closed_side[1][0] + 0.08,
        details=f"closed={closed_side}, open={open_side}",
    )
    ctx.check(
        "front door opens forward",
        closed_front is not None
        and open_front is not None
        and open_front[0][1] < closed_front[0][1] - 0.05,
        details=f"closed={closed_front}, open={open_front}",
    )
    ctx.check(
        "top lid opens upward",
        closed_top is not None
        and open_top is not None
        and open_top[1][2] > closed_top[1][2] + 0.08,
        details=f"closed={closed_top}, open={open_top}",
    )

    return ctx.report()


object_model = build_object_model()
