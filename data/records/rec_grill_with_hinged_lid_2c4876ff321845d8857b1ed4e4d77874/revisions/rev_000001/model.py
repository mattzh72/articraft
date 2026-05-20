from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, radians, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _arc_points(
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int,
) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(start_angle + (end_angle - start_angle) * step / segments),
            radius * sin(start_angle + (end_angle - start_angle) * step / segments),
        )
        for step in range(segments + 1)
    ]


def _segment_profile(
    *,
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    segments: int = 36,
) -> list[tuple[float, float]]:
    outer = _arc_points(outer_radius, start_angle, end_angle, segments=segments)
    inner = _arc_points(inner_radius, end_angle, start_angle, segments=segments)
    return outer + inner


def _barrel_segment_mesh(
    *,
    name: str,
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    length: float,
):
    profile = _segment_profile(
        outer_radius=outer_radius,
        inner_radius=inner_radius,
        start_angle=start_angle,
        end_angle=end_angle,
    )
    return _save_mesh(
        name,
        ExtrudeGeometry.centered(profile, length, cap=True, closed=True).rotate_x(pi / 2.0),
    )


def _add_wagon_wheel(
    model: ArticulatedObject,
    part_name: str,
    *,
    radius: float,
    width: float,
    rim_material,
    hub_material,
    spoke_material,
) -> None:
    part = model.part(part_name)
    part.visual(
        _save_mesh(
            f"{part_name}_rim",
            TorusGeometry(
                radius=radius * 0.83,
                tube=0.010,
                radial_segments=14,
                tubular_segments=48,
            ).rotate_y(pi / 2.0),
        ),
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=radius * 0.22, length=width),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=radius * 0.08, length=width * 0.72),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=spoke_material,
        name="hub_cap",
    )

    spoke_mid_radius = radius * 0.42
    spoke_length = radius * 1.24
    for index in range(8):
        angle = index * (pi / 4.0)
        part.visual(
            Box((width * 0.72, 0.012, spoke_length)),
            origin=Origin(
                xyz=(
                    0.0,
                    spoke_mid_radius * sin(angle),
                    spoke_mid_radius * cos(angle),
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            material=spoke_material,
            name=f"spoke_{index}",
        )

    part.inertial = Inertial.from_geometry(
        Cylinder(radius=radius, length=width),
        mass=4.5,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_smoker_grill")

    painted_steel = model.material("painted_steel", rgba=(0.16, 0.16, 0.17, 1.0))
    firebox_steel = model.material("firebox_steel", rgba=(0.12, 0.12, 0.13, 1.0))
    cart_steel = model.material("cart_steel", rgba=(0.20, 0.20, 0.21, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.53, 0.37, 0.20, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.44, 0.45, 0.46, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.72, 0.73, 0.75, 1.0))

    barrel_radius = 0.29
    wall = 0.006
    barrel_length = 1.00
    lid_front_angle = radians(32.0)
    lid_rear_angle = radians(148.0)
    hinge_x = barrel_radius * cos(lid_rear_angle)
    hinge_z = barrel_radius * sin(lid_rear_angle)

    cart_frame = model.part("cart_frame")
    cart_frame.visual(
        Box((0.05, 1.02, 0.03)),
        origin=Origin(xyz=(-0.15, -0.01, -0.465)),
        material=cart_steel,
        name="left_rail",
    )
    cart_frame.visual(
        Box((0.05, 1.02, 0.03)),
        origin=Origin(xyz=(0.15, -0.01, -0.465)),
        material=cart_steel,
        name="right_rail",
    )
    cart_frame.visual(
        Box((0.35, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.35, -0.465)),
        material=cart_steel,
        name="front_crossbar",
    )
    cart_frame.visual(
        Box((0.35, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.02, -0.465)),
        material=cart_steel,
        name="mid_crossbar",
    )
    cart_frame.visual(
        Box((0.35, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, -0.31, -0.465)),
        material=cart_steel,
        name="rear_crossbar",
    )
    cart_frame.visual(
        Cylinder(radius=0.025, length=0.60),
        origin=Origin(xyz=(0.0, -0.11, -0.50), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="axle",
    )
    cart_frame.visual(
        Box((0.04, 0.04, 0.24)),
        origin=Origin(xyz=(-0.15, 0.42, -0.57)),
        material=cart_steel,
        name="left_leg",
    )
    cart_frame.visual(
        Box((0.04, 0.04, 0.24)),
        origin=Origin(xyz=(0.15, 0.42, -0.57)),
        material=cart_steel,
        name="right_leg",
    )
    cart_frame.visual(
        Box((0.28, 0.06, 0.02)),
        origin=Origin(xyz=(0.0, -0.54, -0.36)),
        material=cart_steel,
        name="firebox_support",
    )
    cart_frame.visual(
        Box((0.03, 0.48, 0.16)),
        origin=Origin(xyz=(-0.12, -0.01, -0.545)),
        material=cart_steel,
        name="left_lower_shelf_frame",
    )
    cart_frame.visual(
        Box((0.03, 0.48, 0.16)),
        origin=Origin(xyz=(0.12, -0.01, -0.545)),
        material=cart_steel,
        name="right_lower_shelf_frame",
    )
    cart_frame.visual(
        Box((0.04, 0.24, 0.04)),
        origin=Origin(xyz=(-0.10, -0.43, -0.45)),
        material=cart_steel,
        name="left_firebox_bar",
    )
    cart_frame.visual(
        Box((0.04, 0.24, 0.04)),
        origin=Origin(xyz=(0.10, -0.43, -0.45)),
        material=cart_steel,
        name="right_firebox_bar",
    )
    cart_frame.visual(
        Box((0.04, 0.04, 0.10)),
        origin=Origin(xyz=(-0.10, -0.54, -0.40)),
        material=cart_steel,
        name="left_firebox_post",
    )
    cart_frame.visual(
        Box((0.04, 0.04, 0.10)),
        origin=Origin(xyz=(0.10, -0.54, -0.40)),
        material=cart_steel,
        name="right_firebox_post",
    )
    for index, shelf_y in enumerate((-0.22, -0.08, 0.06, 0.20)):
        cart_frame.visual(
            Box((0.26, 0.04, 0.02)),
            origin=Origin(xyz=(0.0, shelf_y, -0.62)),
            material=warm_wood,
            name=f"shelf_slat_{index}",
        )
    cart_frame.visual(
        _save_mesh(
            "cart_pull_handle",
            wire_from_points(
                [
                    (-0.15, 0.48, -0.45),
                    (-0.18, 0.62, -0.34),
                    (0.18, 0.62, -0.34),
                    (0.15, 0.48, -0.45),
                ],
                radius=0.012,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.05,
                corner_segments=10,
            ),
        ),
        material=cart_steel,
        name="pull_handle",
    )
    cart_frame.inertial = Inertial.from_geometry(
        Box((0.70, 1.20, 0.40)),
        mass=20.0,
        origin=Origin(xyz=(0.0, 0.0, -0.50)),
    )

    main_chamber = model.part("main_chamber")
    main_chamber.visual(
        _barrel_segment_mesh(
            name="main_chamber_lower_shell",
            outer_radius=barrel_radius,
            inner_radius=barrel_radius - wall,
            start_angle=lid_rear_angle,
            end_angle=lid_front_angle + 2.0 * pi,
            length=barrel_length,
        ),
        material=painted_steel,
        name="barrel_shell",
    )
    main_chamber.visual(
        Box((0.08, 0.10, 0.10)),
        origin=Origin(xyz=(-0.315, 0.34, 0.06)),
        material=painted_steel,
        name="chimney_base",
    )
    main_chamber.visual(
        Cylinder(radius=0.033, length=0.22),
        origin=Origin(xyz=(-0.335, 0.34, 0.17)),
        material=painted_steel,
        name="chimney_stack",
    )
    main_chamber.visual(
        Box((0.16, 0.72, 0.02)),
        origin=Origin(xyz=(0.39, 0.00, -0.02)),
        material=warm_wood,
        name="front_shelf",
    )
    main_chamber.visual(
        Box((0.04, 0.04, 0.32)),
        origin=Origin(xyz=(0.30, -0.22, -0.18)),
        material=cart_steel,
        name="left_shelf_bracket",
    )
    main_chamber.visual(
        Box((0.04, 0.04, 0.32)),
        origin=Origin(xyz=(0.30, 0.22, -0.18)),
        material=cart_steel,
        name="right_shelf_bracket",
    )
    main_chamber.visual(
        Box((0.08, 0.11, 0.18)),
        origin=Origin(xyz=(-0.13, -0.10, -0.36)),
        material=painted_steel,
        name="left_saddle",
    )
    main_chamber.visual(
        Box((0.08, 0.11, 0.18)),
        origin=Origin(xyz=(0.13, -0.10, -0.36)),
        material=painted_steel,
        name="right_saddle",
    )
    main_chamber.visual(
        Box((0.08, 0.11, 0.18)),
        origin=Origin(xyz=(-0.13, 0.14, -0.36)),
        material=painted_steel,
        name="left_rear_saddle",
    )
    main_chamber.visual(
        Box((0.08, 0.11, 0.18)),
        origin=Origin(xyz=(0.13, 0.14, -0.36)),
        material=painted_steel,
        name="right_rear_saddle",
    )
    main_chamber.inertial = Inertial.from_geometry(
        Box((0.80, 1.10, 0.70)),
        mass=48.0,
        origin=Origin(),
    )

    main_lid = model.part("main_lid")
    main_lid.visual(
        _barrel_segment_mesh(
            name="main_lid_shell_mesh",
            outer_radius=barrel_radius,
            inner_radius=barrel_radius - wall,
            start_angle=lid_front_angle,
            end_angle=lid_rear_angle,
            length=barrel_length,
        ),
        origin=Origin(xyz=(-hinge_x, 0.0, -hinge_z)),
        material=painted_steel,
        name="lid_shell",
    )
    main_lid.visual(
        Box((0.06, 0.06, 0.03)),
        origin=Origin(xyz=(0.41, -0.22, -0.01)),
        material=painted_steel,
        name="left_handle_base",
    )
    main_lid.visual(
        Box((0.05, 0.05, 0.12)),
        origin=Origin(xyz=(0.445, -0.22, 0.015)),
        material=painted_steel,
        name="left_handle_post",
    )
    main_lid.visual(
        Box((0.06, 0.06, 0.03)),
        origin=Origin(xyz=(0.41, 0.22, -0.01)),
        material=painted_steel,
        name="right_handle_base",
    )
    main_lid.visual(
        Box((0.05, 0.05, 0.12)),
        origin=Origin(xyz=(0.445, 0.22, 0.015)),
        material=painted_steel,
        name="right_handle_post",
    )
    main_lid.visual(
        Cylinder(radius=0.013, length=0.56),
        origin=Origin(xyz=(0.466, 0.00, 0.072), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_metal,
        name="lid_handle",
    )
    main_lid.visual(
        Box((0.07, 0.024, 0.035)),
        origin=Origin(xyz=(0.355, 0.08, 0.102)),
        material=wheel_metal,
        name="thermometer_stem",
    )
    main_lid.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.405, 0.08, 0.125)),
        material=wheel_metal,
        name="thermometer",
    )
    main_lid.inertial = Inertial.from_geometry(
        Box((0.58, 1.02, 0.30)),
        mass=16.0,
        origin=Origin(xyz=(0.25, 0.0, 0.05)),
    )

    firebox_outer_x = 0.34
    firebox_outer_y = 0.36
    firebox_outer_z = 0.34
    throat_x = 0.16
    throat_z = 0.18
    door_width = 0.30
    door_height = 0.28
    door_thickness = 0.008

    firebox = model.part("firebox")
    firebox.visual(
        Box((firebox_outer_x, firebox_outer_y, wall)),
        origin=Origin(xyz=(0.0, 0.0, firebox_outer_z / 2.0 - wall / 2.0)),
        material=firebox_steel,
        name="top_plate",
    )
    firebox.visual(
        Box((firebox_outer_x, firebox_outer_y, wall)),
        origin=Origin(xyz=(0.0, 0.0, -firebox_outer_z / 2.0 + wall / 2.0)),
        material=firebox_steel,
        name="bottom_plate",
    )
    firebox.visual(
        Box((wall, firebox_outer_y, firebox_outer_z - 2.0 * wall)),
        origin=Origin(xyz=(-firebox_outer_x / 2.0 + wall / 2.0, 0.0, 0.0)),
        material=firebox_steel,
        name="left_wall",
    )
    firebox.visual(
        Box((wall, firebox_outer_y, firebox_outer_z - 2.0 * wall)),
        origin=Origin(xyz=(firebox_outer_x / 2.0 - wall / 2.0, 0.0, 0.0)),
        material=firebox_steel,
        name="right_wall",
    )

    back_strip_height = (firebox_outer_z - throat_z) / 2.0
    back_strip_width = (firebox_outer_x - throat_x) / 2.0
    firebox.visual(
        Box((firebox_outer_x, wall, back_strip_height)),
        origin=Origin(
            xyz=(0.0, firebox_outer_y / 2.0 - wall / 2.0, (firebox_outer_z + throat_z) / 4.0)
        ),
        material=firebox_steel,
        name="back_upper_strip",
    )
    firebox.visual(
        Box((firebox_outer_x, wall, back_strip_height)),
        origin=Origin(
            xyz=(0.0, firebox_outer_y / 2.0 - wall / 2.0, -(firebox_outer_z + throat_z) / 4.0)
        ),
        material=firebox_steel,
        name="back_lower_strip",
    )
    firebox.visual(
        Box((back_strip_width, wall, throat_z)),
        origin=Origin(
            xyz=(-(firebox_outer_x + throat_x) / 4.0, firebox_outer_y / 2.0 - wall / 2.0, 0.0)
        ),
        material=firebox_steel,
        name="back_left_strip",
    )
    firebox.visual(
        Box((back_strip_width, wall, throat_z)),
        origin=Origin(
            xyz=((firebox_outer_x + throat_x) / 4.0, firebox_outer_y / 2.0 - wall / 2.0, 0.0)
        ),
        material=firebox_steel,
        name="back_right_strip",
    )

    front_strip_height = (firebox_outer_z - door_height) / 2.0
    front_strip_width = (firebox_outer_x - door_width) / 2.0
    firebox.visual(
        Box((firebox_outer_x, wall, front_strip_height)),
        origin=Origin(
            xyz=(0.0, -firebox_outer_y / 2.0 + wall / 2.0, (firebox_outer_z + door_height) / 4.0)
        ),
        material=firebox_steel,
        name="front_upper_strip",
    )
    firebox.visual(
        Box((firebox_outer_x, wall, front_strip_height)),
        origin=Origin(
            xyz=(0.0, -firebox_outer_y / 2.0 + wall / 2.0, -(firebox_outer_z + door_height) / 4.0)
        ),
        material=firebox_steel,
        name="front_lower_strip",
    )
    firebox.visual(
        Box((front_strip_width, wall, door_height)),
        origin=Origin(
            xyz=(-(firebox_outer_x + door_width) / 4.0, -firebox_outer_y / 2.0 + wall / 2.0, 0.0)
        ),
        material=firebox_steel,
        name="front_left_strip",
    )
    firebox.visual(
        Box((front_strip_width, wall, door_height)),
        origin=Origin(
            xyz=((firebox_outer_x + door_width) / 4.0, -firebox_outer_y / 2.0 + wall / 2.0, 0.0)
        ),
        material=firebox_steel,
        name="front_right_strip",
    )
    firebox.visual(
        Box((0.26, 0.06, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material=firebox_steel,
        name="support_foot",
    )
    firebox.inertial = Inertial.from_geometry(
        Box((0.40, 0.40, 0.38)),
        mass=18.0,
        origin=Origin(),
    )

    firebox_door = model.part("firebox_door")
    firebox_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0, -door_thickness / 2.0, 0.0)),
        material=painted_steel,
        name="door_panel",
    )
    firebox_door.visual(
        Box((0.03, 0.03, 0.06)),
        origin=Origin(xyz=(door_width * 0.72, -0.020, 0.045)),
        material=painted_steel,
        name="upper_handle_mount",
    )
    firebox_door.visual(
        Box((0.03, 0.03, 0.06)),
        origin=Origin(xyz=(door_width * 0.72, -0.020, -0.045)),
        material=painted_steel,
        name="lower_handle_mount",
    )
    firebox_door.visual(
        Cylinder(radius=0.010, length=0.14),
        origin=Origin(xyz=(door_width * 0.72, -0.030, 0.0)),
        material=handle_metal,
        name="door_handle",
    )
    firebox_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.05, door_height)),
        mass=2.0,
        origin=Origin(xyz=(door_width / 2.0, -0.02, 0.0)),
    )

    _add_wagon_wheel(
        model,
        "left_wheel",
        radius=0.18,
        width=0.06,
        rim_material=wheel_metal,
        hub_material=cart_steel,
        spoke_material=wheel_metal,
    )
    _add_wagon_wheel(
        model,
        "right_wheel",
        radius=0.18,
        width=0.06,
        rim_material=wheel_metal,
        hub_material=cart_steel,
        spoke_material=wheel_metal,
    )

    model.articulation(
        "cart_to_main_chamber",
        ArticulationType.FIXED,
        parent=cart_frame,
        child=main_chamber,
        origin=Origin(),
    )
    model.articulation(
        "main_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=main_chamber,
        child=main_lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-1.20,
            upper=0.0,
        ),
    )
    model.articulation(
        "main_chamber_to_firebox",
        ArticulationType.FIXED,
        parent=main_chamber,
        child=firebox,
        origin=Origin(xyz=(0.0, -0.68, -0.17)),
    )
    model.articulation(
        "firebox_door_hinge",
        ArticulationType.REVOLUTE,
        parent=firebox,
        child=firebox_door,
        origin=Origin(xyz=(-door_width / 2.0, -firebox_outer_y / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-1.45,
            upper=0.0,
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=cart_frame,
        child="left_wheel",
        origin=Origin(xyz=(-0.33, -0.11, -0.50)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=cart_frame,
        child="right_wheel",
        origin=Origin(xyz=(0.33, -0.11, -0.50)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    cart_frame = object_model.get_part("cart_frame")
    main_chamber = object_model.get_part("main_chamber")
    main_lid = object_model.get_part("main_lid")
    firebox = object_model.get_part("firebox")
    firebox_door = object_model.get_part("firebox_door")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    lid_hinge = object_model.get_articulation("main_lid_hinge")
    firebox_hinge = object_model.get_articulation("firebox_door_hinge")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

    ctx.check(
        "key parts exist",
        all(
            part is not None
            for part in (
                cart_frame,
                main_chamber,
                main_lid,
                firebox,
                firebox_door,
                left_wheel,
                right_wheel,
            )
        ),
        "Expected smoker frame, chamber, lid, firebox, door, and both wheels.",
    )
    ctx.check(
        "main lid hinge axis is longitudinal",
        tuple(lid_hinge.axis) == (0.0, 1.0, 0.0),
        f"Axis was {lid_hinge.axis!r}",
    )
    ctx.check(
        "firebox door hinge axis is vertical",
        tuple(firebox_hinge.axis) == (0.0, 0.0, 1.0),
        f"Axis was {firebox_hinge.axis!r}",
    )
    ctx.check(
        "wheel spin axes share axle direction",
        tuple(left_spin.axis) == (1.0, 0.0, 0.0) and tuple(right_spin.axis) == (1.0, 0.0, 0.0),
        f"Left axis {left_spin.axis!r}, right axis {right_spin.axis!r}",
    )

    ctx.expect_contact(main_chamber, cart_frame, elem_a="left_saddle", elem_b="left_rail")
    ctx.expect_contact(main_chamber, cart_frame, elem_a="right_saddle", elem_b="right_rail")
    ctx.expect_contact(firebox, main_chamber)
    ctx.expect_contact(left_wheel, cart_frame, elem_a="hub", elem_b="axle")
    ctx.expect_contact(right_wheel, cart_frame, elem_a="hub", elem_b="axle")

    with ctx.pose({lid_hinge: 0.0, firebox_hinge: 0.0}):
        ctx.expect_contact(main_lid, main_chamber)
        ctx.expect_contact(firebox_door, firebox)

    chamber_pos = ctx.part_world_position(main_chamber)
    firebox_pos = ctx.part_world_position(firebox)
    left_wheel_pos = ctx.part_world_position(left_wheel)
    right_wheel_pos = ctx.part_world_position(right_wheel)
    ctx.check(
        "firebox sits off one chamber end",
        chamber_pos is not None
        and firebox_pos is not None
        and firebox_pos[1] < chamber_pos[1] - 0.55,
        f"Chamber position {chamber_pos}, firebox position {firebox_pos}",
    )
    ctx.check(
        "cart wheels sit below cooking chamber",
        chamber_pos is not None
        and left_wheel_pos is not None
        and right_wheel_pos is not None
        and left_wheel_pos[2] < chamber_pos[2] - 0.35
        and right_wheel_pos[2] < chamber_pos[2] - 0.35,
        (
            f"Chamber position {chamber_pos}, "
            f"left wheel {left_wheel_pos}, right wheel {right_wheel_pos}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
