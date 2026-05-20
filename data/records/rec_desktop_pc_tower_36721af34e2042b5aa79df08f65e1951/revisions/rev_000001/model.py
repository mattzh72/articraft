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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_tower_workstation")

    width = 0.235
    depth = 0.560
    body_height = 0.585
    foot_height = 0.015
    wall = 0.012
    front_wall = 0.014
    panel_gap = 0.0015
    side_panel_thickness = 0.006
    door_gap = 0.0015
    door_thickness = 0.010
    lid_gap = 0.0015
    lid_thickness = 0.008
    top_z = foot_height + body_height
    bond = 0.002

    def rect_loop(x0: float, x1: float, y0: float, y1: float):
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    def build_top_lid_mesh(lid_width: float, lid_depth: float, thickness: float):
        outer = rect_loop(-lid_width * 0.5, lid_width * 0.5, -lid_depth, 0.0)
        hole_profiles = []
        cols = 7
        rows = 9
        hole_w = 0.016
        hole_d = 0.022
        web_x = 0.006
        web_y = 0.007
        grid_w = cols * hole_w + (cols - 1) * web_x
        grid_d = rows * hole_d + (rows - 1) * web_y
        x_start = -grid_w * 0.5
        y_top = -0.040
        y_bottom = y_top - grid_d
        if y_bottom < -lid_depth + 0.026:
            shift = (-lid_depth + 0.026) - y_bottom
            y_top += shift
        for row in range(rows):
            y1 = y_top - row * (hole_d + web_y)
            y0 = y1 - hole_d
            for col in range(cols):
                x0 = x_start + col * (hole_w + web_x)
                x1 = x0 + hole_w
                hole_profiles.append(rect_loop(x0, x1, y0, y1))

        geom = ExtrudeWithHolesGeometry(
            outer,
            hole_profiles,
            thickness,
            cap=True,
            center=True,
            closed=True,
        )
        geom.translate(0.0, 0.0, thickness * 0.5)
        return mesh_from_geometry(geom, "tower_top_vent_lid")

    dark_steel = model.material("dark_steel", rgba=(0.17, 0.18, 0.20, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.27, 1.0))
    mesh_dark = model.material("mesh_dark", rgba=(0.21, 0.22, 0.24, 1.0))
    brushed_alloy = model.material("brushed_alloy", rgba=(0.54, 0.56, 0.59, 1.0))
    bay_grey = model.material("bay_grey", rgba=(0.38, 0.40, 0.43, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, foot_height + wall * 0.5)),
        material=dark_steel,
        name="floor_pan",
    )
    chassis.visual(
        Box((wall, depth - front_wall + bond, body_height - wall)),
        origin=Origin(
            xyz=(
                -width * 0.5 + wall * 0.5,
                front_wall * 0.5,
                foot_height + wall + (body_height - wall) * 0.5,
            )
        ),
        material=graphite,
        name="left_wall",
    )
    chassis.visual(
        Box((width, wall, body_height - wall)),
        origin=Origin(
            xyz=(
                0.0,
                depth * 0.5 - wall * 0.5,
                foot_height + wall + (body_height - wall) * 0.5,
            )
        ),
        material=graphite,
        name="rear_wall",
    )
    chassis.visual(
        Box((width, wall, 0.094)),
        origin=Origin(
            xyz=(0.0, -depth * 0.5 + front_wall * 0.5, foot_height + body_height - 0.047)
        ),
        material=dark_steel,
        name="front_top_header",
    )
    chassis.visual(
        Box((width, wall, 0.128)),
        origin=Origin(
            xyz=(0.0, -depth * 0.5 + front_wall * 0.5, foot_height + 0.064)
        ),
        material=dark_steel,
        name="front_bottom_header",
    )
    chassis.visual(
        Box((0.020, wall, body_height - 0.220)),
        origin=Origin(
            xyz=(
                -width * 0.5 + 0.010,
                -depth * 0.5 + front_wall * 0.5,
                foot_height + 0.128 + (body_height - 0.220) * 0.5,
            )
        ),
        material=dark_steel,
        name="front_left_jamb",
    )
    chassis.visual(
        Box((0.020, wall, body_height - 0.220)),
        origin=Origin(
            xyz=(
                width * 0.5 - 0.010,
                -depth * 0.5 + front_wall * 0.5,
                foot_height + 0.128 + (body_height - 0.220) * 0.5,
            )
        ),
        material=dark_steel,
        name="front_right_jamb",
    )

    top_rim = 0.022
    chassis.visual(
        Box((width, top_rim, wall)),
        origin=Origin(xyz=(0.0, -depth * 0.5 + top_rim * 0.5, top_z - wall * 0.5)),
        material=dark_steel,
        name="top_front_rail",
    )
    chassis.visual(
        Box((width, top_rim, wall)),
        origin=Origin(xyz=(0.0, depth * 0.5 - top_rim * 0.5, top_z - wall * 0.5)),
        material=dark_steel,
        name="top_rear_rail",
    )
    chassis.visual(
        Box((top_rim, depth - 2.0 * top_rim + bond, wall)),
        origin=Origin(xyz=(-width * 0.5 + top_rim * 0.5, 0.0, top_z - wall * 0.5)),
        material=dark_steel,
        name="top_left_rail",
    )
    chassis.visual(
        Box((top_rim, depth - 2.0 * top_rim + bond, wall)),
        origin=Origin(xyz=(width * 0.5 - top_rim * 0.5, 0.0, top_z - wall * 0.5)),
        material=dark_steel,
        name="top_right_rail",
    )

    chassis.visual(
        Box((0.170, 0.140, 0.230)),
        origin=Origin(xyz=(0.0, -depth * 0.5 + 0.072, foot_height + 0.330)),
        material=bay_grey,
        name="drive_bay_cage",
    )
    for index, z in enumerate((0.484, 0.435, 0.386, 0.337)):
        chassis.visual(
            Box((0.150, 0.010, 0.034)),
            origin=Origin(xyz=(0.0, -depth * 0.5 + 0.007, foot_height + z)),
            material=brushed_alloy,
            name=f"drive_bay_face_{index}",
        )
    for rail_index, x in enumerate((-0.052, 0.052)):
        chassis.visual(
            Box((0.016, 0.140, 0.215)),
            origin=Origin(xyz=(x, -depth * 0.5 + 0.072, foot_height + 0.1225)),
            material=bay_grey,
            name=f"drive_bay_support_{rail_index}",
        )

    for x_sign in (-1.0, 1.0):
        chassis.visual(
            Box((0.048, 0.034, foot_height)),
            origin=Origin(
                xyz=(
                    x_sign * (width * 0.5 - 0.038),
                    -depth * 0.5 + 0.050,
                    foot_height * 0.5,
                )
            ),
            material=rubber,
            name=f"front_foot_{'left' if x_sign < 0.0 else 'right'}",
        )
        chassis.visual(
            Box((0.048, 0.034, foot_height)),
            origin=Origin(
                xyz=(
                    x_sign * (width * 0.5 - 0.038),
                    depth * 0.5 - 0.050,
                    foot_height * 0.5,
                )
            ),
            material=rubber,
            name=f"rear_foot_{'left' if x_sign < 0.0 else 'right'}",
        )

    side_hinge_y = depth * 0.5 - 0.012
    for label, z in (("lower", foot_height + 0.060), ("upper", top_z - 0.090)):
        chassis.visual(
            Box((0.018, 0.020, 0.032)),
            origin=Origin(xyz=(width * 0.5 - 0.004, side_hinge_y, z)),
            material=dark_steel,
            name=f"side_hinge_block_{label}",
        )
        chassis.visual(
            Cylinder(radius=0.004, length=0.024),
            origin=Origin(xyz=(width * 0.5 + 0.0015, side_hinge_y, z)),
            material=brushed_alloy,
            name=f"side_hinge_pin_{label}",
        )

    front_hinge_y = -depth * 0.5 - 0.003
    for label, z in (("lower", foot_height + 0.182), ("upper", top_z - 0.118)):
        chassis.visual(
            Box((0.018, 0.018, 0.034)),
            origin=Origin(xyz=(-width * 0.5 + 0.006, -depth * 0.5 + 0.006, z)),
            material=dark_steel,
            name=f"door_hinge_block_{label}",
        )
        chassis.visual(
            Cylinder(radius=0.004, length=0.024),
            origin=Origin(xyz=(-width * 0.5 - 0.0015, front_hinge_y, z)),
            material=brushed_alloy,
            name=f"door_hinge_pin_{label}",
        )

    lid_hinge_z = top_z - 0.002
    lid_hinge_y = depth * 0.5 - 0.012
    for suffix, x in (("left", -0.072), ("right", 0.072)):
        chassis.visual(
            Box((0.022, 0.020, 0.016)),
            origin=Origin(xyz=(x, lid_hinge_y, top_z - 0.006)),
            material=dark_steel,
            name=f"top_lid_block_{suffix}",
        )
        chassis.visual(
            Cylinder(radius=0.0046, length=0.024),
            origin=Origin(
                xyz=(x, lid_hinge_y - 0.002, lid_hinge_z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=brushed_alloy,
            name=f"top_lid_pin_{suffix}",
        )

    chassis.inertial = Inertial.from_geometry(
        Box((width, depth, body_height + foot_height)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, (body_height + foot_height) * 0.5)),
    )

    side_panel = model.part("side_panel")
    panel_height = body_height - 0.078
    panel_depth = depth - 0.028
    side_panel.visual(
        Box((side_panel_thickness, panel_depth, panel_height)),
        origin=Origin(
            xyz=(
                0.009,
                -panel_depth * 0.5,
                panel_height * 0.5,
            )
        ),
        material=graphite,
        name="panel_sheet",
    )
    side_panel.visual(
        Box((0.010, panel_depth - 0.120, panel_height - 0.110)),
        origin=Origin(
            xyz=(
                0.016,
                -panel_depth * 0.5,
                panel_height * 0.5,
            )
        ),
        material=satin_black,
        name="panel_emboss",
    )
    side_panel.visual(
        Box((0.014, 0.020, panel_height)),
        origin=Origin(
            xyz=(0.010, -panel_depth + 0.010, panel_height * 0.5)
        ),
        material=graphite,
        name="front_latch_return",
    )
    side_panel.visual(
        Box((0.006, 0.018, 0.032)),
        origin=Origin(xyz=(0.0065, -0.009, 0.016)),
        material=graphite,
        name="panel_hinge_tab_lower",
    )
    side_panel.visual(
        Box((0.006, 0.018, 0.032)),
        origin=Origin(xyz=(0.0065, -0.009, panel_height - 0.019)),
        material=graphite,
        name="panel_hinge_tab_upper",
    )
    side_panel.inertial = Inertial.from_geometry(
        Box((0.018, panel_depth, panel_height)),
        mass=2.4,
        origin=Origin(xyz=(0.012, -panel_depth * 0.5, panel_height * 0.5)),
    )

    front_door = model.part("front_door")
    door_width = width - 0.012
    door_height = 0.412
    door_skin_width = door_width - 0.008
    front_door.visual(
        Box((door_skin_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(0.006 + door_skin_width * 0.5, -0.012, door_height * 0.5)
        ),
        material=graphite,
        name="door_skin",
    )
    front_door.visual(
        Box((door_skin_width - 0.026, 0.004, door_height - 0.070)),
        origin=Origin(
            xyz=(0.006 + door_skin_width * 0.5, -0.017, door_height * 0.5)
        ),
        material=satin_black,
        name="door_inset",
    )
    front_door.visual(
        Box((0.012, 0.018, 0.130)),
        origin=Origin(
            xyz=(door_skin_width - 0.006, -0.018, door_height * 0.5)
        ),
        material=brushed_alloy,
        name="door_pull",
    )
    front_door.visual(
        Box((0.006, 0.018, 0.036)),
        origin=Origin(xyz=(0.007, -0.008, 0.018)),
        material=graphite,
        name="door_hinge_tab_lower",
    )
    front_door.visual(
        Box((0.006, 0.018, 0.036)),
        origin=Origin(xyz=(0.007, -0.008, door_height - 0.086)),
        material=graphite,
        name="door_hinge_tab_upper",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.020, door_height)),
        mass=1.8,
        origin=Origin(xyz=(door_width * 0.5, -0.012, door_height * 0.5)),
    )

    top_lid = model.part("top_lid")
    lid_width = width - 0.044
    lid_depth = 0.440
    lid_mesh = build_top_lid_mesh(lid_width, lid_depth, lid_thickness)
    top_lid.visual(lid_mesh, material=mesh_dark, name="vent_mesh")
    top_lid.visual(
        Box((0.052, 0.014, lid_thickness)),
        origin=Origin(xyz=(0.0, -lid_depth + 0.007, lid_thickness * 0.5)),
        material=graphite,
        name="front_grip",
    )
    for suffix, x in (("left", -lid_width * 0.5 + 0.020), ("right", lid_width * 0.5 - 0.020)):
        top_lid.visual(
            Box((0.020, 0.016, lid_thickness + 0.010)),
            origin=Origin(xyz=(x, -0.008, (lid_thickness + 0.010) * 0.5 - 0.002)),
            material=graphite,
            name=f"lid_ear_{suffix}",
        )
    top_lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, lid_thickness + 0.006)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -lid_depth * 0.5, (lid_thickness + 0.006) * 0.5)),
    )

    model.articulation(
        "chassis_to_side_panel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(width * 0.5 + panel_gap, side_hinge_y, foot_height + 0.039)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=2.30),
    )
    model.articulation(
        "chassis_to_front_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(xyz=(-width * 0.5 - door_gap, -depth * 0.5 - door_gap, foot_height + 0.140)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=2.05),
    )
    model.articulation(
        "chassis_to_top_lid",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=top_lid,
        origin=Origin(xyz=(0.0, lid_hinge_y, top_z + 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.6, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("side_panel")
    front_door = object_model.get_part("front_door")
    top_lid = object_model.get_part("top_lid")
    side_hinge = object_model.get_articulation("chassis_to_side_panel")
    front_hinge = object_model.get_articulation("chassis_to_front_door")
    top_hinge = object_model.get_articulation("chassis_to_top_lid")

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

    ctx.expect_gap(
        side_panel,
        chassis,
        axis="x",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="panel_sheet",
        name="side panel skin sits just outside the right chassis wall",
    )
    ctx.expect_contact(
        side_panel,
        chassis,
        elem_a="panel_hinge_tab_lower",
        elem_b="side_hinge_block_lower",
        name="side panel lower hinge tab mounts to the lower rear hinge block",
    )
    ctx.expect_contact(
        side_panel,
        chassis,
        elem_a="panel_hinge_tab_upper",
        elem_b="side_hinge_block_upper",
        name="side panel upper hinge tab mounts to the upper rear hinge block",
    )
    ctx.expect_overlap(
        side_panel,
        chassis,
        axes="yz",
        min_overlap=0.40,
        name="side panel covers most of the case side",
    )
    ctx.expect_gap(
        chassis,
        front_door,
        axis="y",
        min_gap=0.001,
        max_gap=0.004,
        negative_elem="door_skin",
        name="front door skin sits just ahead of the front frame",
    )
    ctx.expect_contact(
        front_door,
        chassis,
        elem_a="door_hinge_tab_lower",
        elem_b="door_hinge_block_lower",
        name="front door lower hinge tab mounts to the lower left hinge block",
    )
    ctx.expect_contact(
        front_door,
        chassis,
        elem_a="door_hinge_tab_upper",
        elem_b="door_hinge_block_upper",
        name="front door upper hinge tab mounts to the upper left hinge block",
    )
    ctx.expect_overlap(
        front_door,
        chassis,
        axes="xz",
        min_overlap=0.18,
        name="front door spans the drive bay opening",
    )
    ctx.expect_gap(
        top_lid,
        chassis,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="vent_mesh",
        name="top ventilation mesh sits just above the roof frame",
    )
    ctx.expect_contact(
        top_lid,
        chassis,
        elem_a="lid_ear_left",
        elem_b="top_lid_block_left",
        name="left top lid ear mounts on the left rear hinge block",
    )
    ctx.expect_contact(
        top_lid,
        chassis,
        elem_a="lid_ear_right",
        elem_b="top_lid_block_right",
        name="right top lid ear mounts on the right rear hinge block",
    )
    ctx.expect_overlap(
        top_lid,
        chassis,
        axes="xy",
        min_overlap=0.17,
        name="top ventilation lid covers the roof vent area",
    )

    closed_side = ctx.part_world_aabb(side_panel)
    closed_door = ctx.part_world_aabb(front_door)
    closed_lid = ctx.part_world_aabb(top_lid)
    with ctx.pose({side_hinge: 1.18}):
        open_side = ctx.part_world_aabb(side_panel)
    with ctx.pose({front_hinge: 1.30}):
        open_door = ctx.part_world_aabb(front_door)
    with ctx.pose({top_hinge: 0.95}):
        open_lid = ctx.part_world_aabb(top_lid)
    with ctx.pose({side_hinge: 1.00, front_hinge: 1.15, top_hinge: 0.90}):
        ctx.fail_if_parts_overlap_in_current_pose(name="opened panels and lid stay clear")

    ctx.check(
        "side panel swings outward from the rear edge",
        closed_side is not None
        and open_side is not None
        and open_side[1][0] > closed_side[1][0] + 0.08,
        details=f"closed={closed_side}, open={open_side}",
    )
    ctx.check(
        "front door swings forward on the left hinge",
        closed_door is not None
        and open_door is not None
        and open_door[0][1] < closed_door[0][1] - 0.08,
        details=f"closed={closed_door}, open={open_door}",
    )
    ctx.check(
        "top ventilation lid lifts upward",
        closed_lid is not None
        and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.10,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
