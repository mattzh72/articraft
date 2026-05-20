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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rear_arc_points(radius: float, center_y: float, samples: int) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for i in range(samples + 1):
        theta = math.pi * i / samples
        x = radius * math.cos(theta)
        y = center_y - radius * math.sin(theta)
        points.append((x, y))
    return points


def _car_floor_profile(
    *,
    half_width: float,
    front_y: float,
    side_join_y: float,
    rear_arc_samples: int = 24,
) -> list[tuple[float, float]]:
    profile = [
        (half_width, front_y),
        (half_width, side_join_y),
    ]
    profile.extend(_rear_arc_points(half_width, side_join_y, rear_arc_samples)[1:])
    profile.append((-half_width, front_y))
    return profile


def _car_side_panel_profile(
    *,
    outer_half_width: float,
    inner_half_width: float,
    opening_half_width_outer: float,
    opening_half_width_inner: float,
    outer_front_y: float,
    inner_front_y: float,
    side_join_y: float,
    rear_gap_half_width: float,
    rear_arc_samples: int = 24,
) -> list[tuple[float, float]]:
    outer_arc = _rear_arc_points(outer_half_width, side_join_y, rear_arc_samples)
    inner_arc = _rear_arc_points(inner_half_width, side_join_y, rear_arc_samples)
    outer_arc_right = [point for point in outer_arc if point[0] >= rear_gap_half_width]
    inner_arc_right = [point for point in inner_arc if point[0] >= rear_gap_half_width]

    profile = [(opening_half_width_outer, outer_front_y), (outer_half_width, outer_front_y)]
    profile.extend(outer_arc_right)
    profile.extend(reversed(inner_arc_right))
    profile.extend([(inner_half_width, inner_front_y), (opening_half_width_inner, inner_front_y)])
    return profile


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observation_tower_elevator")

    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.64, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    shaft_glass = model.material("shaft_glass", rgba=(0.74, 0.88, 0.96, 0.20))
    car_glass = model.material("car_glass", rgba=(0.80, 0.94, 0.99, 0.26))
    door_glass = model.material("door_glass", rgba=(0.84, 0.97, 1.0, 0.34))
    floor_finish = model.material("floor_finish", rgba=(0.46, 0.47, 0.49, 1.0))

    shaft_height = 9.40
    shaft_shell_bottom = 0.20
    shaft_shell_top = 9.20
    shaft_outer_radius = 1.10
    shaft_inner_radius = 1.07
    shaft_ring_radius = 1.18

    car_height = 2.26
    car_floor_thickness = 0.08
    car_roof_thickness = 0.08
    door_travel = 0.21

    car_outer_half_width = 0.74
    car_inner_half_width = 0.71
    car_opening_half_width_outer = 0.50
    car_opening_half_width_inner = 0.47
    car_outer_front_y = 0.64
    car_inner_front_y = 0.61
    car_side_join_y = 0.16
    car_rear_gap_half_width = 0.12

    shaft = model.part("shaft")

    shaft.visual(
        Cylinder(radius=1.28, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=concrete,
        name="base_podium",
    )
    base_ring_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(shaft_ring_radius, -0.03), (shaft_ring_radius, 0.03)],
            [(shaft_outer_radius, -0.03), (shaft_outer_radius, 0.03)],
            segments=72,
        ),
        "base_ring_shell",
    )
    shaft.visual(
        base_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=graphite,
        name="base_ring",
    )

    shaft_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(shaft_outer_radius, shaft_shell_bottom), (shaft_outer_radius, shaft_shell_top)],
            [(shaft_inner_radius, shaft_shell_bottom), (shaft_inner_radius, shaft_shell_top)],
            segments=72,
        ),
        "tower_shaft_shell",
    )
    shaft.visual(shaft_shell_mesh, material=shaft_glass, name="shaft_shell")

    belt_ring_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(shaft_ring_radius, -0.025), (shaft_ring_radius, 0.025)],
            [(shaft_outer_radius, -0.025), (shaft_outer_radius, 0.025)],
            segments=72,
        ),
        "belt_ring_shell",
    )
    for z in (2.30, 4.40, 6.50):
        shaft.visual(
            belt_ring_mesh,
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=steel,
            name=f"belt_ring_{int(z * 100)}",
        )

    crown_ring_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(1.20, -0.09), (1.20, 0.09)],
            [(shaft_outer_radius, -0.09), (shaft_outer_radius, 0.09)],
            segments=72,
        ),
        "crown_ring_shell",
    )
    shaft.visual(
        crown_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 9.29)),
        material=graphite,
        name="crown_ring",
    )

    mullion_radius = 0.018
    mullion_mid_z = 0.5 * (shaft_shell_bottom + shaft_shell_top)
    mullion_length = shaft_shell_top - shaft_shell_bottom
    for index, angle_deg in enumerate((20, 80, 140, 200, 260, 320), start=1):
        angle = math.radians(angle_deg)
        shaft.visual(
            Cylinder(radius=mullion_radius, length=mullion_length),
            origin=Origin(
                xyz=(0.98 * math.sin(angle), 0.98 * math.cos(angle), mullion_mid_z)
            ),
            material=steel,
            name=f"mullion_{index}",
        )

    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=1.28, length=shaft_height),
        mass=5200.0,
        origin=Origin(xyz=(0.0, 0.0, shaft_height * 0.5)),
    )

    left_guide_rail = model.part("left_guide_rail")
    left_guide_rail.visual(
        Box((0.03, 0.03, shaft_shell_top - shaft_shell_bottom)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (shaft_shell_top - shaft_shell_bottom))),
        material=steel,
        name="rail_web",
    )
    left_guide_rail.visual(
        Box((0.06, 0.012, shaft_shell_top - shaft_shell_bottom)),
        origin=Origin(xyz=(0.0, 0.019, 0.5 * (shaft_shell_top - shaft_shell_bottom))),
        material=steel,
        name="rail_flange",
    )
    left_guide_rail.inertial = Inertial.from_geometry(
        Box((0.06, 0.05, shaft_shell_top - shaft_shell_bottom)),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.01, 0.5 * (shaft_shell_top - shaft_shell_bottom))),
    )

    right_guide_rail = model.part("right_guide_rail")
    right_guide_rail.visual(
        Box((0.03, 0.03, shaft_shell_top - shaft_shell_bottom)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (shaft_shell_top - shaft_shell_bottom))),
        material=steel,
        name="rail_web",
    )
    right_guide_rail.visual(
        Box((0.06, 0.012, shaft_shell_top - shaft_shell_bottom)),
        origin=Origin(xyz=(0.0, 0.019, 0.5 * (shaft_shell_top - shaft_shell_bottom))),
        material=steel,
        name="rail_flange",
    )
    right_guide_rail.inertial = Inertial.from_geometry(
        Box((0.06, 0.05, shaft_shell_top - shaft_shell_bottom)),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.01, 0.5 * (shaft_shell_top - shaft_shell_bottom))),
    )

    car = model.part("car")

    floor_profile = _car_floor_profile(
        half_width=car_outer_half_width,
        front_y=car_outer_front_y,
        side_join_y=car_side_join_y,
    )
    right_wall_profile = _car_side_panel_profile(
        outer_half_width=car_outer_half_width,
        inner_half_width=car_inner_half_width,
        opening_half_width_outer=car_opening_half_width_outer,
        opening_half_width_inner=car_opening_half_width_inner,
        outer_front_y=car_outer_front_y,
        inner_front_y=car_inner_front_y,
        side_join_y=car_side_join_y,
        rear_gap_half_width=car_rear_gap_half_width,
    )
    left_wall_profile = [(-x, y) for x, y in right_wall_profile]

    floor_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(floor_profile, car_floor_thickness),
        "car_floor_slab",
    )
    car.visual(floor_mesh, material=floor_finish, name="floor_slab")

    roof_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(floor_profile, car_roof_thickness),
        "car_roof_slab",
    )
    car.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, car_height - car_roof_thickness)),
        material=graphite,
        name="roof_slab",
    )

    right_wall_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            right_wall_profile,
            car_height - car_floor_thickness - car_roof_thickness,
        ),
        "right_car_wall_panel",
    )
    car.visual(
        right_wall_mesh,
        origin=Origin(xyz=(0.0, 0.0, car_floor_thickness)),
        material=car_glass,
        name="right_wall_panel",
    )

    left_wall_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            left_wall_profile,
            car_height - car_floor_thickness - car_roof_thickness,
        ),
        "left_car_wall_panel",
    )
    car.visual(
        left_wall_mesh,
        origin=Origin(xyz=(0.0, 0.0, car_floor_thickness)),
        material=car_glass,
        name="left_wall_panel",
    )

    car.visual(
        Box((1.00, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.60, 0.04)),
        material=graphite,
        name="threshold_sill",
    )
    car.visual(
        Box((1.00, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.60, 2.12)),
        material=graphite,
        name="door_header",
    )

    for side in (-1.0, 1.0):
        car.visual(
            Box((0.04, 0.04, 2.10)),
            origin=Origin(xyz=(side * 0.52, 0.62, 1.13)),
            material=steel,
            name=f"{'left' if side < 0 else 'right'}_jamb",
        )

    car.visual(
        Box((1.08, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.63, 2.19)),
        material=steel,
        name="front_canopy_trim",
    )

    car.visual(
        Box((0.42, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, -0.535, 0.12)),
        material=graphite,
        name="rear_lower_crossbeam",
    )
    car.visual(
        Box((0.42, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, -0.535, 2.14)),
        material=graphite,
        name="rear_upper_crossbeam",
    )
    car.visual(
        Box((0.16, 0.05, 2.02)),
        origin=Origin(xyz=(0.0, -0.535, 1.13)),
        material=graphite,
        name="rear_service_mast",
    )
    for x in (-0.18, 0.18):
        car.visual(
            Box((0.06, 0.05, 2.02)),
            origin=Origin(xyz=(x, -0.53, 1.13)),
            material=graphite,
            name=f"rear_carrier_post_{'left' if x < 0 else 'right'}",
        )
        car.visual(
            Box((0.05, 0.16, 0.12)),
            origin=Origin(xyz=(x, -0.635, 0.66)),
            material=steel,
            name=f"lower_guide_shoe_{'left' if x < 0 else 'right'}",
        )
        car.visual(
            Box((0.05, 0.16, 0.12)),
            origin=Origin(xyz=(x, -0.635, 1.62)),
            material=steel,
            name=f"upper_guide_shoe_{'left' if x < 0 else 'right'}",
        )
        car.visual(
            Box((0.05, 0.01, 0.12)),
            origin=Origin(xyz=(x, -0.72, 0.66)),
            material=graphite,
            name=f"lower_guide_liner_{'left' if x < 0 else 'right'}",
        )
        car.visual(
            Box((0.05, 0.01, 0.12)),
            origin=Origin(xyz=(x, -0.72, 1.62)),
            material=graphite,
            name=f"upper_guide_liner_{'left' if x < 0 else 'right'}",
        )
    car.inertial = Inertial.from_geometry(
        Box((1.48, 1.28, car_height)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.02, car_height * 0.5)),
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((0.50, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=steel,
        name="bottom_rail",
    )
    left_door.visual(
        Box((0.50, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.925)),
        material=steel,
        name="top_rail",
    )
    left_door.visual(
        Box((0.035, 0.03, 1.90)),
        origin=Origin(xyz=(-0.2325, 0.0, 0.975)),
        material=steel,
        name="left_stile",
    )
    left_door.visual(
        Box((0.035, 0.03, 1.90)),
        origin=Origin(xyz=(0.2325, 0.0, 0.975)),
        material=steel,
        name="right_stile",
    )
    left_door.visual(
        Box((0.44, 0.012, 1.87)),
        origin=Origin(xyz=(0.0, 0.0, 0.975)),
        material=door_glass,
        name="glass_lite",
    )
    left_door.visual(
        Box((0.12, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.955)),
        material=graphite,
        name="hanger_shoe",
    )
    left_door.visual(
        Box((0.08, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=graphite,
        name="guide_shoe",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((0.50, 0.04, 2.00)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((0.50, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=steel,
        name="bottom_rail",
    )
    right_door.visual(
        Box((0.50, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.925)),
        material=steel,
        name="top_rail",
    )
    right_door.visual(
        Box((0.035, 0.03, 1.90)),
        origin=Origin(xyz=(-0.2325, 0.0, 0.975)),
        material=steel,
        name="left_stile",
    )
    right_door.visual(
        Box((0.035, 0.03, 1.90)),
        origin=Origin(xyz=(0.2325, 0.0, 0.975)),
        material=steel,
        name="right_stile",
    )
    right_door.visual(
        Box((0.44, 0.012, 1.87)),
        origin=Origin(xyz=(0.0, 0.0, 0.975)),
        material=door_glass,
        name="glass_lite",
    )
    right_door.visual(
        Box((0.12, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.955)),
        material=graphite,
        name="hanger_shoe",
    )
    right_door.visual(
        Box((0.08, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=graphite,
        name="guide_shoe",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((0.50, 0.04, 2.00)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
    )

    model.articulation(
        "car_lift",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=22000.0,
            velocity=1.8,
            lower=0.0,
            upper=6.60,
        ),
    )
    model.articulation(
        "shaft_to_left_guide_rail",
        ArticulationType.FIXED,
        parent=shaft,
        child=left_guide_rail,
        origin=Origin(xyz=(-0.16, -0.75, shaft_shell_bottom)),
    )
    model.articulation(
        "shaft_to_right_guide_rail",
        ArticulationType.FIXED,
        parent=shaft,
        child=right_guide_rail,
        origin=Origin(xyz=(0.16, -0.75, shaft_shell_bottom)),
    )
    model.articulation(
        "left_door_slide",
        ArticulationType.PRISMATIC,
        parent=car,
        child=left_door,
        origin=Origin(xyz=(-0.25, 0.57, 0.08)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.75,
            lower=0.0,
            upper=door_travel,
        ),
    )
    model.articulation(
        "right_door_slide",
        ArticulationType.PRISMATIC,
        parent=car,
        child=right_door,
        origin=Origin(xyz=(0.25, 0.57, 0.08)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.75,
            lower=0.0,
            upper=door_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    left_guide_rail = object_model.get_part("left_guide_rail")
    right_guide_rail = object_model.get_part("right_guide_rail")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")

    car_lift = object_model.get_articulation("car_lift")
    left_door_slide = object_model.get_articulation("left_door_slide")
    right_door_slide = object_model.get_articulation("right_door_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(car, left_guide_rail, name="car_contacts_left_guide_rail")
    ctx.expect_contact(car, right_guide_rail, name="car_contacts_right_guide_rail")
    ctx.expect_contact(left_door, car, name="left_door_is_mounted_in_car_track")
    ctx.expect_contact(right_door, car, name="right_door_is_mounted_in_car_track")
    ctx.expect_contact(left_door, right_door, name="door_leaves_meet_when_closed")
    ctx.expect_within(car, shaft, axes="xy", margin=0.0, name="car_stays_within_round_column")

    lift_limits = car_lift.motion_limits
    left_limits = left_door_slide.motion_limits
    right_limits = right_door_slide.motion_limits

    ctx.check(
        "lift_joint_is_vertical_prismatic",
        car_lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(car_lift.axis) == (0.0, 0.0, 1.0)
        and lift_limits is not None
        and lift_limits.lower == 0.0
        and lift_limits.upper is not None
        and lift_limits.upper >= 6.0,
        details="Car lift should be a long-travel vertical prismatic joint.",
    )
    ctx.check(
        "door_joints_are_bi_parting_horizontal_prismatics",
        left_door_slide.articulation_type == ArticulationType.PRISMATIC
        and right_door_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(left_door_slide.axis) == (-1.0, 0.0, 0.0)
        and tuple(right_door_slide.axis) == (1.0, 0.0, 0.0)
        and left_limits is not None
        and right_limits is not None
        and abs((left_limits.upper or 0.0) - 0.21) < 1e-9
        and abs((right_limits.upper or 0.0) - 0.21) < 1e-9,
        details="Door leaves should slide away from center on opposing horizontal tracks.",
    )

    rest_pos = ctx.part_world_position(car)
    if rest_pos is None:
        ctx.fail("car_world_position_available", "Car world position is unavailable in rest pose.")
    else:
        with ctx.pose({car_lift: 4.80}):
            raised_pos = ctx.part_world_position(car)
            ctx.expect_contact(car, left_guide_rail, name="left_guide_contact_when_raised")
            ctx.expect_contact(car, right_guide_rail, name="right_guide_contact_when_raised")
            ctx.check(
                "car_lift_moves_up_the_column",
                raised_pos is not None and raised_pos[2] > rest_pos[2] + 4.7,
                details="Raised car should translate upward several meters inside the column.",
            )

    with ctx.pose({left_door_slide: 0.21, right_door_slide: 0.21}):
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.40,
            name="doors_create_a_clear_center_opening",
        )
        ctx.expect_contact(left_door, car, name="left_door_stays_supported_when_open")
        ctx.expect_contact(right_door, car, name="right_door_stays_supported_when_open")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
