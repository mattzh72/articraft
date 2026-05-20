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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_drive_home_elevator")

    tower_paint = model.material("tower_paint", rgba=(0.17, 0.18, 0.20, 1.0))
    car_shell = model.material("car_shell", rgba=(0.84, 0.85, 0.86, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.63, 0.64, 0.66, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.33, 0.34, 0.36, 1.0))
    door_finish = model.material("door_finish", rgba=(0.91, 0.92, 0.93, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.70, 0.72, 0.75, 1.0))
    floor_finish = model.material("floor_finish", rgba=(0.38, 0.32, 0.24, 1.0))

    def _lead_screw_mesh(
        *,
        height: float,
        core_radius: float,
        thread_radius: float,
        pitch: float,
        samples_per_turn: int = 16,
    ):
        geom = CylinderGeometry(radius=core_radius, height=height, radial_segments=40)
        turns = height / pitch
        point_count = max(64, int(turns * samples_per_turn) + 1)
        helix_radius = core_radius + thread_radius * 0.85
        helix_points: list[tuple[float, float, float]] = []
        for index in range(point_count):
            t = index / (point_count - 1)
            angle = math.tau * turns * t
            z = -height * 0.5 + height * t
            helix_points.append(
                (
                    helix_radius * math.cos(angle),
                    helix_radius * math.sin(angle),
                    z,
                )
            )
        geom.merge(
            wire_from_points(
                helix_points,
                radius=thread_radius,
                radial_segments=14,
                cap_ends=False,
                closed_path=False,
                corner_mode="miter",
            )
        )
        return mesh_from_geometry(geom, "elevator_lead_screw")

    tower = model.part("tower")
    tower.visual(
        Box((1.36, 1.18, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=tower_paint,
        name="base_plinth",
    )
    tower.visual(
        Box((1.04, 0.18, 0.02)),
        origin=Origin(xyz=(-0.06, 0.68, 0.09)),
        material=trim_dark,
        name="landing_sill",
    )
    tower.visual(
        Box((0.08, 0.08, 2.70)),
        origin=Origin(xyz=(-0.62, 0.55, 1.43)),
        material=tower_paint,
        name="left_front_post",
    )
    tower.visual(
        Box((0.08, 0.08, 2.70)),
        origin=Origin(xyz=(-0.62, -0.55, 1.43)),
        material=tower_paint,
        name="left_rear_post",
    )
    tower.visual(
        Box((0.06, 0.06, 2.62)),
        origin=Origin(xyz=(0.50, 0.56, 1.39)),
        material=tower_paint,
        name="right_front_post",
    )
    tower.visual(
        Box((1.18, 0.08, 0.08)),
        origin=Origin(xyz=(-0.06, 0.56, 2.74)),
        material=tower_paint,
        name="front_header_beam",
    )
    tower.visual(
        Box((1.18, 0.08, 0.08)),
        origin=Origin(xyz=(-0.06, -0.55, 2.70)),
        material=tower_paint,
        name="rear_header_beam",
    )
    tower.visual(
        Box((0.08, 1.18, 0.06)),
        origin=Origin(xyz=(-0.62, 0.0, 2.70)),
        material=tower_paint,
        name="left_roof_rail",
    )
    tower.visual(
        Box((0.08, 0.88, 0.06)),
        origin=Origin(xyz=(0.50, 0.14, 2.72)),
        material=tower_paint,
        name="right_roof_rail",
    )
    tower.visual(
        Box((0.11, 0.18, 2.84)),
        origin=Origin(xyz=(0.52, -0.28, 1.50)),
        material=rail_steel,
        name="guide_mast",
    )
    tower.visual(
        Box((0.22, 0.34, 0.16)),
        origin=Origin(xyz=(0.47, 0.10, 2.82)),
        material=tower_paint,
        name="drive_head",
    )
    tower.visual(
        Box((0.12, 0.18, 0.12)),
        origin=Origin(xyz=(0.47, 0.27, 0.14)),
        material=tower_paint,
        name="lower_bearing_block",
    )
    tower.visual(
        _lead_screw_mesh(
            height=2.66,
            core_radius=0.022,
            thread_radius=0.010,
            pitch=0.10,
        ),
        origin=Origin(xyz=(0.47, 0.27, 1.41)),
        material=screw_steel,
        name="lead_screw",
    )
    tower.inertial = Inertial.from_geometry(
        Box((1.36, 1.18, 2.90)),
        mass=540.0,
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
    )

    car = model.part("car")
    car.visual(
        Box((0.92, 0.98, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=floor_finish,
        name="floor",
    )
    car.visual(
        Box((0.92, 0.03, 1.98)),
        origin=Origin(xyz=(0.0, -0.475, 1.04)),
        material=car_shell,
        name="rear_wall",
    )
    car.visual(
        Box((0.03, 0.95, 1.98)),
        origin=Origin(xyz=(-0.445, -0.015, 1.04)),
        material=car_shell,
        name="left_wall",
    )
    car.visual(
        Box((0.03, 0.95, 1.98)),
        origin=Origin(xyz=(0.445, -0.015, 1.04)),
        material=car_shell,
        name="right_wall",
    )
    car.visual(
        Box((0.92, 0.98, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 2.05)),
        material=car_shell,
        name="roof",
    )
    car.visual(
        Box((0.08, 0.03, 1.92)),
        origin=Origin(xyz=(-0.42, 0.475, 1.01)),
        material=car_shell,
        name="left_jamb",
    )
    car.visual(
        Box((0.08, 0.03, 1.92)),
        origin=Origin(xyz=(0.42, 0.475, 1.01)),
        material=car_shell,
        name="right_jamb",
    )
    car.visual(
        Box((0.76, 0.03, 0.08)),
        origin=Origin(xyz=(0.0, 0.475, 1.99)),
        material=car_shell,
        name="door_header",
    )
    car.visual(
        Box((0.76, 0.05, 0.02)),
        origin=Origin(xyz=(0.0, 0.465, 0.06)),
        material=trim_dark,
        name="threshold",
    )
    car.visual(
        Box((0.16, 0.48, 1.80)),
        origin=Origin(xyz=(0.52, -0.14, 1.00)),
        material=trim_dark,
        name="side_carriage_frame",
    )
    car.visual(
        Box((0.05, 0.18, 0.24)),
        origin=Origin(xyz=(0.60, -0.28, 1.00)),
        material=rail_steel,
        name="guide_shoe",
    )
    car.visual(
        Box((0.08, 0.10, 0.24)),
        origin=Origin(xyz=(0.63, 0.175, 0.96)),
        material=rail_steel,
        name="drive_nut_block",
    )
    car.visual(
        Box((0.08, 0.16, 0.22)),
        origin=Origin(xyz=(0.60, 0.07, 0.96)),
        material=rail_steel,
        name="drive_nut_bracket",
    )
    car.visual(
        Box((0.62, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, -0.40, 1.02)),
        material=handle_finish,
        name="rear_handrail",
    )
    car.visual(
        Box((0.02, 0.045, 0.10)),
        origin=Origin(xyz=(-0.24, -0.4375, 1.02)),
        material=handle_finish,
        name="rear_handrail_bracket_left",
    )
    car.visual(
        Box((0.02, 0.045, 0.10)),
        origin=Origin(xyz=(0.24, -0.4375, 1.02)),
        material=handle_finish,
        name="rear_handrail_bracket_right",
    )
    car.inertial = Inertial.from_geometry(
        Box((1.02, 1.10, 2.10)),
        mass=220.0,
        origin=Origin(xyz=(0.04, 0.0, 1.05)),
    )

    outer_door = model.part("outer_door_panel")
    outer_door.visual(
        Box((0.374, 0.024, 1.88)),
        origin=Origin(xyz=(0.187, 0.0, 0.94)),
        material=door_finish,
        name="outer_panel",
    )
    outer_door.visual(
        Box((0.018, 0.032, 0.72)),
        origin=Origin(xyz=(0.344, 0.010, 0.98)),
        material=handle_finish,
        name="outer_pull",
    )
    outer_door.inertial = Inertial.from_geometry(
        Box((0.374, 0.024, 1.88)),
        mass=18.0,
        origin=Origin(xyz=(0.187, 0.0, 0.94)),
    )

    inner_door = model.part("inner_door_panel")
    inner_door.visual(
        Box((0.374, 0.024, 1.88)),
        origin=Origin(xyz=(0.187, 0.0, 0.94)),
        material=door_finish,
        name="inner_panel",
    )
    inner_door.visual(
        Box((0.018, 0.032, 0.72)),
        origin=Origin(xyz=(0.050, 0.010, 0.98)),
        material=handle_finish,
        name="inner_pull",
    )
    inner_door.inertial = Inertial.from_geometry(
        Box((0.374, 0.024, 1.88)),
        mass=18.0,
        origin=Origin(xyz=(0.187, 0.0, 0.94)),
    )

    model.articulation(
        "tower_to_car",
        ArticulationType.PRISMATIC,
        parent=tower,
        child=car,
        origin=Origin(xyz=(-0.16, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.45,
            lower=0.0,
            upper=2.20,
        ),
    )
    model.articulation(
        "car_to_outer_door",
        ArticulationType.REVOLUTE,
        parent=car,
        child=outer_door,
        origin=Origin(xyz=(-0.38, 0.463, 0.07)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=0.0,
            upper=1.40,
        ),
    )
    model.articulation(
        "outer_to_inner_door",
        ArticulationType.REVOLUTE,
        parent=outer_door,
        child=inner_door,
        origin=Origin(xyz=(0.374, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.2,
            lower=-2.70,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    car = object_model.get_part("car")
    outer_door = object_model.get_part("outer_door_panel")
    inner_door = object_model.get_part("inner_door_panel")

    lift = object_model.get_articulation("tower_to_car")
    outer_hinge = object_model.get_articulation("car_to_outer_door")
    inner_hinge = object_model.get_articulation("outer_to_inner_door")

    def _aabb_center(aabb):
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

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

    ctx.expect_contact(car, tower)
    ctx.expect_contact(outer_door, car)
    ctx.expect_contact(inner_door, outer_door)

    ctx.check(
        "lift_axis_is_vertical",
        tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical prismatic axis, got {lift.axis!r}",
    )
    ctx.check(
        "door_hinges_are_vertical",
        tuple(outer_hinge.axis) == (0.0, 0.0, 1.0)
        and tuple(inner_hinge.axis) == (0.0, 0.0, 1.0),
        details=(
            "Expected both bifold hinges to rotate about +Z, "
            f"got {outer_hinge.axis!r} and {inner_hinge.axis!r}"
        ),
    )

    car_rest = ctx.part_world_position(car)
    outer_rest_aabb = ctx.part_world_aabb(outer_door)
    inner_rest_aabb = ctx.part_world_aabb(inner_door)
    if car_rest is None or outer_rest_aabb is None or inner_rest_aabb is None:
        ctx.fail("world_positions_available", "Missing rest-pose world positions for key parts.")
        return ctx.report()
    outer_rest_center = _aabb_center(outer_rest_aabb)
    inner_rest_center = _aabb_center(inner_rest_aabb)

    with ctx.pose({lift: 1.80}):
        ctx.expect_contact(car, tower)
        car_high = ctx.part_world_position(car)
        ctx.check(
            "car_lifts_straight_up",
            car_high is not None
            and abs(car_high[0] - car_rest[0]) < 1e-6
            and abs(car_high[1] - car_rest[1]) < 1e-6
            and abs((car_high[2] - car_rest[2]) - 1.80) < 1e-6,
            details=f"Rest={car_rest!r}, raised={car_high!r}",
        )

    with ctx.pose({outer_hinge: 1.15}):
        outer_open_aabb = ctx.part_world_aabb(outer_door)
        outer_open_center = _aabb_center(outer_open_aabb) if outer_open_aabb is not None else None
        ctx.check(
            "outer_panel_swings_on_jamb_hinge",
            outer_open_aabb is not None
            and outer_open_center is not None
            and outer_open_aabb[1][1] > outer_rest_aabb[1][1] + 0.18
            and outer_open_center[0] < outer_rest_center[0] - 0.08,
            details=f"Rest={outer_rest_aabb!r}, open={outer_open_aabb!r}",
        )

    with ctx.pose({outer_hinge: 1.15, inner_hinge: -2.30}):
        outer_folded_pose_aabb = ctx.part_world_aabb(outer_door)
        inner_folded_aabb = ctx.part_world_aabb(inner_door)
        outer_folded_center = (
            _aabb_center(outer_folded_pose_aabb) if outer_folded_pose_aabb is not None else None
        )
        inner_folded_center = (
            _aabb_center(inner_folded_aabb) if inner_folded_aabb is not None else None
        )
        ctx.check(
            "inner_panel_folds_back_on_second_hinge",
            outer_folded_pose_aabb is not None
            and inner_folded_aabb is not None
            and outer_folded_center is not None
            and inner_folded_center is not None
            and inner_folded_aabb[1][1] > inner_rest_aabb[1][1] + 0.18
            and inner_folded_center[0] > outer_folded_center[0] + 0.12
            and abs(inner_folded_aabb[1][1] - outer_folded_pose_aabb[1][1]) < 0.03,
            details=(
                f"Rest={inner_rest_aabb!r}, "
                f"outer_folded={outer_folded_pose_aabb!r}, "
                f"inner_folded={inner_folded_aabb!r}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
