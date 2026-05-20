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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 72,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _translated_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _front_oriented_mesh(
    outer_profile: list[tuple[float, float]],
    thickness: float,
    *,
    holes: list[list[tuple[float, float]]] | None = None,
):
    if holes:
        geometry = ExtrudeWithHolesGeometry(
            outer_profile,
            holes,
            height=thickness,
            center=True,
        )
    else:
        geometry = ExtrudeGeometry(
            outer_profile,
            thickness,
            center=True,
        )
    return geometry.rotate_x(math.pi / 2.0)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="washer_with_add_door")

    cabinet_width = 0.60
    cabinet_depth = 0.64
    cabinet_height = 0.85
    side_thickness = 0.018
    top_thickness = 0.025
    bottom_thickness = 0.035
    front_thickness = 0.015
    back_thickness = 0.012

    door_center_z = 0.41
    door_outer_radius = 0.245
    door_window_radius = 0.186
    door_hinge_y = 0.341
    door_hinge_x = -door_outer_radius

    add_door_center_rel = (0.028, 0.062)
    add_door_outer_size = (0.148, 0.100)
    add_door_hole_size = (0.156, 0.120)
    add_door_top_z = add_door_center_rel[1] + add_door_outer_size[1] * 0.5
    add_door_joint = (
        door_outer_radius + add_door_center_rel[0],
        0.006,
        add_door_top_z,
    )

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.19, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.54, 0.56, 0.59, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.20, 0.24, 0.28, 0.48))
    gasket_gray = model.material("gasket_gray", rgba=(0.44, 0.46, 0.49, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.09, 0.10, 1.0))
    soft_silver = model.material("soft_silver", rgba=(0.74, 0.76, 0.79, 1.0))

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    front_shell_mesh = _front_oriented_mesh(
        [
            (-cabinet_width * 0.5, -cabinet_height * 0.5),
            (cabinet_width * 0.5, -cabinet_height * 0.5),
            (cabinet_width * 0.5, cabinet_height * 0.5),
            (-cabinet_width * 0.5, cabinet_height * 0.5),
        ],
        front_thickness,
        holes=[
            _circle_profile(
                0.186,
                center=(0.0, door_center_z - cabinet_height * 0.5),
                segments=72,
            )
        ],
    )
    cabinet.visual(
        _mesh("cabinet_front_shell", front_shell_mesh),
        origin=Origin(
            xyz=(0.0, cabinet_depth * 0.5 - front_thickness * 0.5, cabinet_height * 0.5)
        ),
        material=body_white,
        name="front_shell",
    )
    cabinet.visual(
        Box((side_thickness, cabinet_depth - 0.002, cabinet_height - 0.015)),
        origin=Origin(
            xyz=(
                -cabinet_width * 0.5 + side_thickness * 0.5,
                0.0,
                0.4175,
            )
        ),
        material=body_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((side_thickness, cabinet_depth - 0.002, cabinet_height - 0.015)),
        origin=Origin(
            xyz=(
                cabinet_width * 0.5 - side_thickness * 0.5,
                0.0,
                0.4175,
            )
        ),
        material=body_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - top_thickness * 0.5)),
        material=body_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((cabinet_width - 0.010, cabinet_depth - 0.020, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness * 0.5)),
        material=body_white,
        name="base_tray",
    )
    cabinet.visual(
        Box((cabinet_width - 0.040, back_thickness, cabinet_height - 0.030)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth * 0.5 + back_thickness * 0.5,
                0.410,
            )
        ),
        material=body_white,
        name="back_panel",
    )
    cabinet.visual(
        Cylinder(radius=0.188, length=0.070),
        origin=Origin(
            xyz=(0.0, 0.281, door_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gasket_gray,
        name="drum_gasket",
    )
    cabinet.visual(
        Cylinder(radius=0.170, length=0.035),
        origin=Origin(
            xyz=(0.0, 0.235, door_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=control_black,
        name="drum_shadow",
    )
    cabinet.visual(
        Box((0.140, 0.030, 0.040)),
        origin=Origin(xyz=(-0.175, 0.330, 0.742)),
        material=body_white,
        name="detergent_drawer",
    )
    cabinet.visual(
        Box((0.180, 0.006, 0.055)),
        origin=Origin(xyz=(0.020, 0.318, 0.742)),
        material=control_black,
        name="display_panel",
    )
    cabinet.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(
            xyz=(0.190, 0.324, 0.742),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=soft_silver,
        name="program_knob",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.042),
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, 0.525)),
        material=hinge_gray,
        name="upper_hinge_barrel",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.042),
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, 0.295)),
        material=hinge_gray,
        name="lower_hinge_barrel",
    )
    cabinet.visual(
        Box((0.026, 0.028, 0.062)),
        origin=Origin(xyz=(door_hinge_x - 0.007, 0.326, 0.537)),
        material=hinge_gray,
        name="upper_hinge_bracket",
    )
    cabinet.visual(
        Box((0.026, 0.028, 0.062)),
        origin=Origin(xyz=(door_hinge_x - 0.007, 0.326, 0.283)),
        material=hinge_gray,
        name="lower_hinge_bracket",
    )

    main_door = model.part("main_door")
    main_door.inertial = Inertial.from_geometry(
        Cylinder(radius=door_outer_radius, length=0.060),
        mass=6.5,
        origin=Origin(
            xyz=(door_outer_radius, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
    )

    outer_bezel_mesh = _front_oriented_mesh(
        _circle_profile(door_outer_radius, segments=72),
        0.018,
        holes=[_circle_profile(0.172, segments=72)],
    )
    main_door.visual(
        _mesh("door_outer_bezel", outer_bezel_mesh),
        origin=Origin(xyz=(door_outer_radius, 0.015, 0.0)),
        material=dark_trim,
        name="outer_bezel",
    )

    rear_ring_mesh = _front_oriented_mesh(
        _circle_profile(0.215, segments=72),
        0.024,
        holes=[_circle_profile(0.150, segments=72)],
    )
    main_door.visual(
        _mesh("door_rear_ring", rear_ring_mesh),
        origin=Origin(xyz=(door_outer_radius, -0.007, 0.0)),
        material=dark_trim,
        name="rear_ring",
    )

    add_door_hole_profile = _translated_profile(
        rounded_rect_profile(
            add_door_hole_size[0],
            add_door_hole_size[1],
            0.022,
        ),
        dx=add_door_center_rel[0],
        dy=add_door_center_rel[1],
    )
    window_mesh = _front_oriented_mesh(
        _circle_profile(door_window_radius, segments=72),
        0.012,
        holes=[add_door_hole_profile],
    )
    main_door.visual(
        _mesh("door_window_panel", window_mesh),
        origin=Origin(xyz=(door_outer_radius, 0.001, 0.0)),
        material=tinted_glass,
        name="window_panel",
    )

    add_surround_mesh = _front_oriented_mesh(
        _translated_profile(
            rounded_rect_profile(0.172, 0.136, 0.028),
            dx=add_door_center_rel[0],
            dy=add_door_center_rel[1],
        ),
        0.006,
        holes=[add_door_hole_profile],
    )
    main_door.visual(
        _mesh("door_add_port_surround", add_surround_mesh),
        origin=Origin(xyz=(door_outer_radius, 0.010, 0.0)),
        material=dark_trim,
        name="add_port_surround",
    )

    main_door.visual(
        Cylinder(radius=0.010, length=0.188),
        origin=Origin(),
        material=hinge_gray,
        name="hinge_spine",
    )
    main_door.visual(
        Box((0.082, 0.020, 0.188)),
        origin=Origin(xyz=(0.041, -0.010, 0.0)),
        material=hinge_gray,
        name="hinge_arm",
    )
    main_door.visual(
        Box((0.032, 0.026, 0.126)),
        origin=Origin(xyz=(door_outer_radius + 0.184, 0.019, 0.0)),
        material=soft_silver,
        name="handle",
    )
    main_door.visual(
        Box((0.124, 0.010, 0.010)),
        origin=Origin(
            xyz=(add_door_joint[0], 0.004, add_door_joint[2] + 0.005),
        ),
        material=hinge_gray,
        name="add_hinge_header",
    )
    add_door = model.part("add_door")
    add_door.inertial = Inertial.from_geometry(
        Box((add_door_outer_size[0], 0.018, add_door_outer_size[1])),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, -add_door_outer_size[1] * 0.5)),
    )

    add_panel_mesh = _front_oriented_mesh(
        rounded_rect_profile(add_door_outer_size[0], add_door_outer_size[1], 0.020),
        0.014,
    )
    add_door.visual(
        _mesh("add_door_panel", add_panel_mesh),
        origin=Origin(xyz=(0.0, 0.001, -add_door_outer_size[1] * 0.5)),
        material=dark_trim,
        name="add_panel",
    )
    add_window_mesh = _front_oriented_mesh(
        rounded_rect_profile(0.114, 0.070, 0.015),
        0.005,
    )
    add_door.visual(
        _mesh("add_door_window", add_window_mesh),
        origin=Origin(xyz=(0.0, 0.007, -0.051)),
        material=tinted_glass,
        name="add_window",
    )
    add_door.visual(
        Box((0.106, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.000, -0.004)),
        material=hinge_gray,
        name="add_hinge_leaf",
    )
    add_door.visual(
        Box((0.018, 0.016, 0.055)),
        origin=Origin(xyz=(0.058, 0.009, -0.054)),
        material=soft_silver,
        name="add_handle",
    )

    model.articulation(
        "cabinet_to_main_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=main_door,
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=0.0,
            upper=2.0,
        ),
    )
    model.articulation(
        "main_door_to_add_door",
        ArticulationType.REVOLUTE,
        parent=main_door,
        child=add_door,
        origin=Origin(xyz=add_door_joint),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.3,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    main_door = object_model.get_part("main_door")
    add_door = object_model.get_part("add_door")
    main_hinge = object_model.get_articulation("cabinet_to_main_door")
    add_hinge = object_model.get_articulation("main_door_to_add_door")

    front_shell = cabinet.get_visual("front_shell")
    door_handle = main_door.get_visual("handle")
    rear_ring = main_door.get_visual("rear_ring")
    add_handle = add_door.get_visual("add_handle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        main_door,
        cabinet,
        elem_a="hinge_spine",
        elem_b="upper_hinge_barrel",
        name="main_door_upper_hinge_contact",
    )
    ctx.expect_contact(
        main_door,
        cabinet,
        elem_a="hinge_spine",
        elem_b="lower_hinge_barrel",
        name="main_door_lower_hinge_contact",
    )
    ctx.expect_gap(
        main_door,
        add_door,
        axis="z",
        positive_elem="add_hinge_header",
        negative_elem="add_hinge_leaf",
        min_gap=0.0,
        max_gap=0.0001,
        name="add_door_hinge_leaf_seats_under_header",
    )
    ctx.expect_overlap(
        add_door,
        main_door,
        axes="xy",
        elem_a="add_hinge_leaf",
        elem_b="add_hinge_header",
        min_overlap=0.005,
        name="add_door_hinge_leaf_aligned_with_header",
    )
    ctx.expect_gap(
        main_door,
        cabinet,
        axis="y",
        positive_elem=rear_ring,
        negative_elem=front_shell,
        min_gap=0.0005,
        max_gap=0.004,
        name="main_door_front_clearance",
    )
    ctx.expect_overlap(
        main_door,
        cabinet,
        axes="xz",
        elem_a="outer_bezel",
        elem_b="front_shell",
        min_overlap=0.35,
        name="main_door_covers_opening",
    )
    ctx.expect_within(
        add_door,
        main_door,
        axes="xz",
        margin=0.015,
        name="add_door_stays_within_main_door",
    )
    ctx.check(
        "main_hinge_axis_is_vertical",
        tuple(main_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {main_hinge.axis}",
    )
    ctx.check(
        "add_hinge_axis_is_horizontal",
        tuple(add_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {add_hinge.axis}",
    )

    def _center_of_aabb(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    with ctx.pose({main_hinge: 0.0}):
        closed_handle_aabb = ctx.part_element_world_aabb(main_door, elem=door_handle)
    with ctx.pose({main_hinge: 1.6}):
        open_handle_aabb = ctx.part_element_world_aabb(main_door, elem=door_handle)
        ctx.expect_contact(
            main_door,
            cabinet,
            elem_a="hinge_spine",
            elem_b="upper_hinge_barrel",
            name="main_door_remains_hinged_when_open",
        )

    if closed_handle_aabb is None or open_handle_aabb is None:
        ctx.fail("main_door_handle_aabb_available", "door handle AABB could not be evaluated")
    else:
        closed_handle_center = _center_of_aabb(closed_handle_aabb)
        open_handle_center = _center_of_aabb(open_handle_aabb)
        ctx.check(
            "main_door_swings_forward",
            open_handle_center[1] > closed_handle_center[1] + 0.10,
            details=(
                f"expected open-handle y > {closed_handle_center[1] + 0.10:.3f}, "
                f"got {open_handle_center[1]:.3f}"
            ),
        )
        ctx.check(
            "main_door_swings_left",
            open_handle_center[0] < closed_handle_center[0] - 0.14,
            details=(
                f"expected open-handle x < {closed_handle_center[0] - 0.14:.3f}, "
                f"got {open_handle_center[0]:.3f}"
            ),
        )

    with ctx.pose({add_hinge: 0.0}):
        closed_add_handle_aabb = ctx.part_element_world_aabb(add_door, elem=add_handle)
    with ctx.pose({add_hinge: 1.0}):
        open_add_handle_aabb = ctx.part_element_world_aabb(add_door, elem=add_handle)

    if closed_add_handle_aabb is None or open_add_handle_aabb is None:
        ctx.fail("add_door_handle_aabb_available", "add-door handle AABB could not be evaluated")
    else:
        closed_add_handle_center = _center_of_aabb(closed_add_handle_aabb)
        open_add_handle_center = _center_of_aabb(open_add_handle_aabb)
        ctx.check(
            "add_door_flips_outward",
            open_add_handle_center[1] > closed_add_handle_center[1] + 0.018,
            details=(
                f"expected open add-door handle y > {closed_add_handle_center[1] + 0.018:.3f}, "
                f"got {open_add_handle_center[1]:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
