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
    superellipse_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_mesh(name: str, *, outer_d: float, inner_d: float, thickness: float):
    ring = ExtrudeWithHolesGeometry(
        superellipse_profile(outer_d, outer_d, exponent=2.0, segments=56),
        [superellipse_profile(inner_d, inner_d, exponent=2.0, segments=40)],
        height=thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh(name, ring)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_drawer_washer_combo")

    white_enamel = model.material("white_enamel", rgba=(0.95, 0.96, 0.97, 1.0))
    warm_white = model.material("warm_white", rgba=(0.92, 0.93, 0.94, 1.0))
    black_trim = model.material("black_trim", rgba=(0.08, 0.09, 0.10, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.60, 0.78, 0.88, 0.38))
    gasket_gray = model.material("gasket_gray", rgba=(0.22, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    drawer_gray = model.material("drawer_gray", rgba=(0.84, 0.85, 0.87, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.11, 0.12, 1.0))
    display_black = model.material("display_black", rgba=(0.05, 0.07, 0.09, 1.0))

    pedestal_width = 0.685
    pedestal_depth = 0.775
    pedestal_height = 0.365
    pedestal_wall = 0.020
    pedestal_top = 0.025
    pedestal_bottom = 0.020
    pedestal_front_thickness = 0.020

    washer_width = 0.685
    washer_depth = 0.775
    washer_height = 0.980
    washer_wall = 0.018
    washer_top = 0.028
    washer_bottom = 0.020
    washer_front_thickness = 0.035

    door_outer_d = 0.490
    door_inner_d = 0.355
    door_glass_d = 0.358
    door_thickness = 0.050
    door_center_z = 0.470
    door_hinge_x = -door_outer_d / 2.0
    door_swing_open = 2.20

    drawer_front_width = 0.616
    drawer_front_height = 0.248
    drawer_front_thickness = 0.026
    drawer_tub_width = 0.580
    drawer_tub_length = 0.540
    drawer_side_thickness = 0.014
    drawer_bottom_thickness = 0.010
    drawer_side_height = 0.145
    drawer_back_thickness = 0.016
    drawer_slide_upper = 0.340
    drawer_joint_y = pedestal_depth / 2.0 - drawer_front_thickness

    body_bezel_mesh = _ring_mesh(
        "washer_body_bezel",
        outer_d=0.530,
        inner_d=0.385,
        thickness=0.030,
    )
    body_boot_mesh = _ring_mesh(
        "washer_boot_ring",
        outer_d=0.410,
        inner_d=0.315,
        thickness=0.030,
    )
    door_ring_mesh = _ring_mesh(
        "washer_door_ring",
        outer_d=door_outer_d,
        inner_d=door_inner_d,
        thickness=door_thickness,
    )
    door_inner_trim_mesh = _ring_mesh(
        "washer_door_inner_trim",
        outer_d=0.405,
        inner_d=0.318,
        thickness=0.020,
    )
    drawer_handle_mesh = _save_mesh(
        "pedestal_drawer_handle",
        tube_from_spline_points(
            [
                (-0.230, 0.0, 0.0),
                (-0.170, 0.014, 0.0),
                (0.170, 0.014, 0.0),
                (0.230, 0.0, 0.0),
            ],
            radius=0.009,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )

    pedestal = model.part("pedestal_cabinet")
    pedestal.visual(
        Box((pedestal_width, pedestal_depth, pedestal_top)),
        origin=Origin(xyz=(0.0, 0.0, pedestal_height - pedestal_top / 2.0)),
        material=white_enamel,
        name="top_deck",
    )
    pedestal.visual(
        Box((pedestal_width, pedestal_depth, pedestal_bottom)),
        origin=Origin(xyz=(0.0, 0.0, pedestal_bottom / 2.0)),
        material=warm_white,
        name="base_pan",
    )
    pedestal.visual(
        Box((pedestal_wall, pedestal_depth, pedestal_height - pedestal_top - pedestal_bottom)),
        origin=Origin(
            xyz=(
                -(pedestal_width - pedestal_wall) / 2.0,
                0.0,
                pedestal_bottom + (pedestal_height - pedestal_top - pedestal_bottom) / 2.0,
            )
        ),
        material=white_enamel,
        name="left_side",
    )
    pedestal.visual(
        Box((pedestal_wall, pedestal_depth, pedestal_height - pedestal_top - pedestal_bottom)),
        origin=Origin(
            xyz=(
                (pedestal_width - pedestal_wall) / 2.0,
                0.0,
                pedestal_bottom + (pedestal_height - pedestal_top - pedestal_bottom) / 2.0,
            )
        ),
        material=white_enamel,
        name="right_side",
    )
    pedestal.visual(
        Box((pedestal_width - 2.0 * pedestal_wall, 0.018, pedestal_height - pedestal_top - pedestal_bottom)),
        origin=Origin(
            xyz=(
                0.0,
                -(pedestal_depth - 0.018) / 2.0,
                pedestal_bottom + (pedestal_height - pedestal_top - pedestal_bottom) / 2.0,
            )
        ),
        material=white_enamel,
        name="back_panel",
    )
    pedestal.visual(
        Box((0.030, pedestal_front_thickness, 0.270)),
        origin=Origin(xyz=(-(pedestal_width - 0.030) / 2.0, pedestal_depth / 2.0 - pedestal_front_thickness / 2.0, 0.185)),
        material=white_enamel,
        name="front_left_stile",
    )
    pedestal.visual(
        Box((0.030, pedestal_front_thickness, 0.270)),
        origin=Origin(xyz=((pedestal_width - 0.030) / 2.0, pedestal_depth / 2.0 - pedestal_front_thickness / 2.0, 0.185)),
        material=white_enamel,
        name="front_right_stile",
    )
    pedestal.visual(
        Box((pedestal_width - 0.060, pedestal_front_thickness, 0.028)),
        origin=Origin(xyz=(0.0, pedestal_depth / 2.0 - pedestal_front_thickness / 2.0, 0.322)),
        material=white_enamel,
        name="front_top_rail",
    )
    pedestal.visual(
        Box((pedestal_width - 0.060, 0.045, 0.050)),
        origin=Origin(xyz=(0.0, pedestal_depth / 2.0 - 0.045 / 2.0, 0.025)),
        material=white_enamel,
        name="toe_kick",
    )
    pedestal.visual(
        Box((0.014, 0.640, 0.018)),
        origin=Origin(xyz=(-0.307, -0.070, 0.178)),
        material=dark_steel,
        name="left_outer_rail",
    )
    pedestal.visual(
        Box((0.014, 0.640, 0.018)),
        origin=Origin(xyz=(0.307, -0.070, 0.178)),
        material=dark_steel,
        name="right_outer_rail",
    )
    pedestal.visual(
        Box((0.018, 0.040, 0.030)),
        origin=Origin(xyz=(-0.315, 0.195, 0.178)),
        material=dark_steel,
        name="left_front_rail_bracket",
    )
    pedestal.visual(
        Box((0.018, 0.040, 0.030)),
        origin=Origin(xyz=(-0.315, -0.265, 0.178)),
        material=dark_steel,
        name="left_rear_rail_bracket",
    )
    pedestal.visual(
        Box((0.018, 0.040, 0.030)),
        origin=Origin(xyz=(0.315, 0.195, 0.178)),
        material=dark_steel,
        name="right_front_rail_bracket",
    )
    pedestal.visual(
        Box((0.018, 0.040, 0.030)),
        origin=Origin(xyz=(0.315, -0.265, 0.178)),
        material=dark_steel,
        name="right_rear_rail_bracket",
    )
    pedestal.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(-0.260, 0.270, 0.005)),
        material=dark_steel,
        name="front_left_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.260, 0.270, 0.005)),
        material=dark_steel,
        name="front_right_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(-0.260, -0.270, 0.005)),
        material=dark_steel,
        name="rear_left_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.260, -0.270, 0.005)),
        material=dark_steel,
        name="rear_right_foot",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((pedestal_width, pedestal_depth, pedestal_height)),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, pedestal_height / 2.0)),
    )

    washer = model.part("washer_body")
    washer.visual(
        Box((washer_width, washer_depth, washer_top)),
        origin=Origin(xyz=(0.0, 0.0, washer_height - washer_top / 2.0)),
        material=white_enamel,
        name="top_cover",
    )
    washer.visual(
        Box((washer_width - 2.0 * washer_wall, 0.018, washer_height - washer_top - washer_bottom)),
        origin=Origin(
            xyz=(
                0.0,
                -(washer_depth - 0.018) / 2.0,
                washer_bottom + (washer_height - washer_top - washer_bottom) / 2.0,
            )
        ),
        material=white_enamel,
        name="rear_panel",
    )
    washer.visual(
        Box((washer_wall, washer_depth, washer_height - washer_top - washer_bottom)),
        origin=Origin(
            xyz=(
                -(washer_width - washer_wall) / 2.0,
                0.0,
                washer_bottom + (washer_height - washer_top - washer_bottom) / 2.0,
            )
        ),
        material=white_enamel,
        name="left_wall",
    )
    washer.visual(
        Box((washer_wall, washer_depth, washer_height - washer_top - washer_bottom)),
        origin=Origin(
            xyz=(
                (washer_width - washer_wall) / 2.0,
                0.0,
                washer_bottom + (washer_height - washer_top - washer_bottom) / 2.0,
            )
        ),
        material=white_enamel,
        name="right_wall",
    )
    washer.visual(
        Box((washer_width, washer_depth, washer_bottom)),
        origin=Origin(xyz=(0.0, 0.0, washer_bottom + 0.010)),
        material=warm_white,
        name="base_ring",
    )
    washer.visual(
        Box((washer_width - 2.0 * washer_wall, washer_front_thickness, 0.185)),
        origin=Origin(xyz=(0.0, washer_depth / 2.0 - washer_front_thickness / 2.0, 0.1125)),
        material=white_enamel,
        name="lower_front_panel",
    )
    washer.visual(
        Box((0.060, washer_front_thickness, 0.575)),
        origin=Origin(xyz=(-(washer_width - 0.060) / 2.0, washer_depth / 2.0 - washer_front_thickness / 2.0, 0.505)),
        material=white_enamel,
        name="left_front_stile",
    )
    washer.visual(
        Box((0.060, washer_front_thickness, 0.575)),
        origin=Origin(xyz=((washer_width - 0.060) / 2.0, washer_depth / 2.0 - washer_front_thickness / 2.0, 0.505)),
        material=white_enamel,
        name="right_front_stile",
    )
    washer.visual(
        Box((washer_width - 2.0 * washer_wall, washer_front_thickness, 0.060)),
        origin=Origin(xyz=(0.0, washer_depth / 2.0 - washer_front_thickness / 2.0, 0.760)),
        material=white_enamel,
        name="mid_front_rail",
    )
    washer.visual(
        Box((washer_width - 2.0 * washer_wall, 0.120, 0.170)),
        origin=Origin(xyz=(0.0, washer_depth / 2.0 - 0.060, 0.875)),
        material=white_enamel,
        name="console_housing",
    )
    washer.visual(
        Box((0.250, 0.010, 0.080)),
        origin=Origin(xyz=(0.100, washer_depth / 2.0 + 0.005, 0.895)),
        material=display_black,
        name="display_window",
    )
    washer.visual(
        Cylinder(radius=0.038, length=0.028),
        origin=Origin(xyz=(0.238, washer_depth / 2.0 + 0.014, 0.895), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="program_knob",
    )
    washer.visual(
        Box((0.150, 0.015, 0.086)),
        origin=Origin(xyz=(-0.212, washer_depth / 2.0 + 0.0075, 0.892)),
        material=control_black,
        name="detergent_drawer_face",
    )
    washer.visual(
        body_bezel_mesh,
        origin=Origin(xyz=(0.0, 0.3375, door_center_z)),
        material=steel,
        name="front_bezel",
    )
    washer.visual(
        body_boot_mesh,
        origin=Origin(xyz=(0.0, 0.340, door_center_z)),
        material=gasket_gray,
        name="door_boot",
    )
    washer.visual(
        Cylinder(radius=0.220, length=0.620),
        origin=Origin(xyz=(0.0, -0.060, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_black,
        name="outer_tub_shell",
    )
    washer.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(-0.245, 0.260, 0.010)),
        material=dark_steel,
        name="front_left_foot",
    )
    washer.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.245, 0.260, 0.010)),
        material=dark_steel,
        name="front_right_foot",
    )
    washer.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(-0.245, -0.260, 0.010)),
        material=dark_steel,
        name="rear_left_foot",
    )
    washer.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.245, -0.260, 0.010)),
        material=dark_steel,
        name="rear_right_foot",
    )
    washer.visual(
        Box((0.024, 0.026, 0.110)),
        origin=Origin(xyz=(door_hinge_x + 0.014, washer_depth / 2.0 - 0.048, door_center_z + 0.115)),
        material=dark_steel,
        name="upper_hinge_leaf",
    )
    washer.visual(
        Box((0.024, 0.026, 0.110)),
        origin=Origin(xyz=(door_hinge_x + 0.014, washer_depth / 2.0 - 0.048, door_center_z - 0.115)),
        material=dark_steel,
        name="lower_hinge_leaf",
    )
    washer.visual(
        Box((0.063, 0.026, 0.380)),
        origin=Origin(xyz=(-0.251, washer_depth / 2.0 - 0.048, door_center_z)),
        material=dark_steel,
        name="hinge_reinforcement",
    )
    washer.inertial = Inertial.from_geometry(
        Box((washer_width, washer_depth, washer_height)),
        mass=78.0,
        origin=Origin(xyz=(0.0, 0.0, washer_height / 2.0)),
    )

    door = model.part("washer_door")
    door.visual(
        door_ring_mesh,
        origin=Origin(xyz=(door_outer_d / 2.0, -0.010, 0.0)),
        material=black_trim,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=door_glass_d / 2.0, length=0.012),
        origin=Origin(
            xyz=(door_outer_d / 2.0, -0.018, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=glass_blue,
        name="door_glass",
    )
    door.visual(
        door_inner_trim_mesh,
        origin=Origin(xyz=(door_outer_d / 2.0, -0.015, 0.0)),
        material=steel,
        name="door_inner_trim",
    )
    door.visual(
        Box((0.050, 0.050, 0.350)),
        origin=Origin(xyz=(0.025, -0.010, 0.0)),
        material=black_trim,
        name="hinge_stile",
    )
    door.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(0.010, -0.014, 0.115)),
        material=dark_steel,
        name="upper_hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(0.010, -0.014, -0.115)),
        material=dark_steel,
        name="lower_hinge_barrel",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_outer_d, door_thickness, door_outer_d)),
        mass=6.5,
        origin=Origin(xyz=(door_outer_d / 2.0, door_thickness / 2.0, 0.0)),
    )

    drawer = model.part("pedestal_drawer")
    drawer.visual(
        Box((drawer_front_width, drawer_front_thickness, drawer_front_height)),
        origin=Origin(xyz=(0.0, drawer_front_thickness / 2.0, 0.185)),
        material=white_enamel,
        name="drawer_front",
    )
    drawer.visual(
        Box((drawer_tub_width - 2.0 * drawer_side_thickness, drawer_tub_length, drawer_bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -drawer_tub_length / 2.0,
                0.096,
            )
        ),
        material=drawer_gray,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((drawer_side_thickness, drawer_tub_length, drawer_side_height)),
        origin=Origin(xyz=(-(drawer_tub_width - drawer_side_thickness) / 2.0, -drawer_tub_length / 2.0, 0.1625)),
        material=drawer_gray,
        name="left_drawer_side",
    )
    drawer.visual(
        Box((drawer_side_thickness, drawer_tub_length, drawer_side_height)),
        origin=Origin(xyz=((drawer_tub_width - drawer_side_thickness) / 2.0, -drawer_tub_length / 2.0, 0.1625)),
        material=drawer_gray,
        name="right_drawer_side",
    )
    drawer.visual(
        Box((drawer_tub_width - 2.0 * drawer_side_thickness, drawer_back_thickness, drawer_side_height)),
        origin=Origin(xyz=(0.0, -drawer_tub_length + drawer_back_thickness / 2.0, 0.1625)),
        material=drawer_gray,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.014, 0.600, 0.014)),
        origin=Origin(xyz=(-0.292, -0.300, 0.178)),
        material=dark_steel,
        name="left_inner_rail",
    )
    drawer.visual(
        Box((0.014, 0.600, 0.014)),
        origin=Origin(xyz=(0.292, -0.300, 0.178)),
        material=dark_steel,
        name="right_inner_rail",
    )
    drawer.visual(
        Box((0.020, 0.050, 0.020)),
        origin=Origin(xyz=(-0.170, 0.035, 0.185)),
        material=steel,
        name="left_handle_post",
    )
    drawer.visual(
        Box((0.020, 0.050, 0.020)),
        origin=Origin(xyz=(0.170, 0.035, 0.185)),
        material=steel,
        name="right_handle_post",
    )
    drawer.visual(
        drawer_handle_mesh,
        origin=Origin(xyz=(0.0, 0.060, 0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drawer_handle",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((drawer_front_width, drawer_tub_length + drawer_front_thickness, drawer_front_height)),
        mass=12.0,
        origin=Origin(xyz=(0.0, -0.250, 0.165)),
    )

    model.articulation(
        "washer_mount",
        ArticulationType.FIXED,
        parent=pedestal,
        child=washer,
        origin=Origin(xyz=(0.0, 0.0, pedestal_height)),
    )
    model.articulation(
        "washer_door_hinge",
        ArticulationType.REVOLUTE,
        parent=washer,
        child=door,
        origin=Origin(xyz=(door_hinge_x, washer_depth / 2.0, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.2,
            lower=0.0,
            upper=door_swing_open,
        ),
    )
    model.articulation(
        "pedestal_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=drawer,
        origin=Origin(xyz=(0.0, drawer_joint_y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.55,
            lower=0.0,
            upper=drawer_slide_upper,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal_cabinet")
    washer = object_model.get_part("washer_body")
    door = object_model.get_part("washer_door")
    drawer = object_model.get_part("pedestal_drawer")
    door_hinge = object_model.get_articulation("washer_door_hinge")
    drawer_slide = object_model.get_articulation("pedestal_drawer_slide")

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

    ctx.check(
        "all prompt-critical parts exist",
        all(part is not None for part in (pedestal, washer, door, drawer)),
        details="Missing pedestal, washer body, washer door, or drawer part.",
    )

    ctx.expect_gap(
        washer,
        pedestal,
        axis="z",
        positive_elem="front_left_foot",
        negative_elem="top_deck",
        max_gap=0.002,
        max_penetration=0.0,
        name="washer feet sit on the pedestal top deck",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            washer,
            axis="y",
            positive_elem="door_glass",
            negative_elem="door_boot",
            max_gap=0.010,
            max_penetration=0.0,
            name="closed door seals close to the boot ring",
        )
        ctx.expect_overlap(
            door,
            washer,
            axes="xz",
            elem_a="door_ring",
            elem_b="front_bezel",
            min_overlap=0.380,
            name="closed door covers the porthole opening",
        )
        ctx.expect_contact(
            door,
            washer,
            elem_a="upper_hinge_barrel",
            elem_b="upper_hinge_leaf",
            contact_tol=0.012,
            name="upper barrel hinge sits at the upper hinge leaf",
        )
        ctx.expect_contact(
            door,
            washer,
            elem_a="lower_hinge_barrel",
            elem_b="lower_hinge_leaf",
            contact_tol=0.012,
            name="lower barrel hinge sits at the lower hinge leaf",
        )

    closed_door_ring = ctx.part_element_world_aabb(door, elem="door_ring")
    with ctx.pose({door_hinge: 1.25}):
        opened_door_ring = ctx.part_element_world_aabb(door, elem="door_ring")
    ctx.check(
        "door opens outward from the left edge hinges",
        closed_door_ring is not None
        and opened_door_ring is not None
        and opened_door_ring[1][1] > closed_door_ring[1][1] + 0.120
        and opened_door_ring[1][0] < closed_door_ring[1][0] - 0.200,
        details=f"closed={closed_door_ring}, opened={opened_door_ring}",
    )

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_overlap(
            drawer,
            pedestal,
            axes="y",
            elem_a="left_inner_rail",
            elem_b="left_outer_rail",
            min_overlap=0.450,
            name="closed drawer left rail remains deeply engaged",
        )
        ctx.expect_contact(
            drawer,
            pedestal,
            elem_a="left_inner_rail",
            elem_b="left_outer_rail",
            contact_tol=0.002,
            name="closed drawer left rail runs on the fixed guide rail",
        )

    closed_drawer_front = ctx.part_element_world_aabb(drawer, elem="drawer_front")
    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        ctx.expect_overlap(
            drawer,
            pedestal,
            axes="y",
            elem_a="left_inner_rail",
            elem_b="left_outer_rail",
            min_overlap=0.140,
            name="full-extension drawer still retains rail insertion",
        )
        ctx.expect_contact(
            drawer,
            pedestal,
            elem_a="left_inner_rail",
            elem_b="left_outer_rail",
            contact_tol=0.002,
            name="full-extension drawer rails stay aligned on the guide rail",
        )
        open_drawer_front = ctx.part_element_world_aabb(drawer, elem="drawer_front")
    ctx.check(
        "drawer extends forward on the prismatic rails",
        closed_drawer_front is not None
        and open_drawer_front is not None
        and open_drawer_front[1][1] > closed_drawer_front[1][1] + 0.300,
        details=f"closed={closed_drawer_front}, open={open_drawer_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
