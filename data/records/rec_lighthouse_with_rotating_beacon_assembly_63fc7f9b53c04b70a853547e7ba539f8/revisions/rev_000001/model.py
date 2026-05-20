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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_lighthouse_lantern")

    cast_iron = model.material("cast_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    weathered_iron = model.material("weathered_iron", rgba=(0.28, 0.30, 0.31, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.76, 0.89, 0.93, 0.24))
    optic_glass = model.material("optic_glass", rgba=(0.88, 0.94, 0.98, 0.36))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.28, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.15, 1.0))
    machinery_red = model.material("machinery_red", rgba=(0.43, 0.11, 0.08, 1.0))

    floor_radius = 1.12
    frame_radius = 1.01
    panel_radius = 0.985
    wall_base_z = 0.18
    glazing_base_z = 0.28
    glazing_top_z = 1.76
    glazing_height = glazing_top_z - glazing_base_z
    roof_base_z = 1.88

    bay_count = 16
    bay_step = math.tau / bay_count

    def annular_mesh(name: str, *, inner_radius: float, outer_radius: float, height: float):
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(outer_radius, 0.0), (outer_radius, height)],
                [(inner_radius, 0.0), (inner_radius, height)],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
            name,
        )

    roof_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (1.12, 0.00),
                (1.08, 0.06),
                (0.96, 0.18),
                (0.76, 0.40),
                (0.48, 0.63),
                (0.18, 0.79),
                (0.12, 0.84),
            ],
            [
                (0.95, 0.00),
                (0.92, 0.05),
                (0.83, 0.16),
                (0.66, 0.34),
                (0.42, 0.55),
                (0.16, 0.75),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
        ),
        "lantern_roof_shell",
    )

    lower_curb = annular_mesh(
        "lantern_lower_curb_v2",
        inner_radius=0.84,
        outer_radius=1.06,
        height=0.02,
    )
    upper_cornice = annular_mesh(
        "lantern_upper_cornice",
        inner_radius=0.90,
        outer_radius=1.10,
        height=0.12,
    )
    roof_vent_skirt = annular_mesh(
        "roof_vent_skirt",
        inner_radius=0.08,
        outer_radius=0.17,
        height=0.08,
    )

    lantern_body = model.part("lantern_body")
    lantern_body.visual(
        Cylinder(radius=floor_radius, length=wall_base_z),
        origin=Origin(xyz=(0.0, 0.0, wall_base_z * 0.5)),
        material=cast_iron,
        name="floor_drum",
    )
    lantern_body.visual(
        lower_curb,
        origin=Origin(xyz=(0.0, 0.0, wall_base_z)),
        material=weathered_iron,
        name="lower_curb",
    )

    for bay in range(bay_count):
        mullion_angle = (bay + 0.5) * bay_step
        lantern_body.visual(
            Cylinder(radius=0.024, length=glazing_top_z - wall_base_z),
            origin=Origin(
                xyz=(
                    frame_radius * math.cos(mullion_angle),
                    frame_radius * math.sin(mullion_angle),
                    (wall_base_z + glazing_top_z) * 0.5,
                )
            ),
            material=cast_iron,
            name=f"mullion_{bay:02d}",
        )

    panel_width = 0.26
    panel_thickness = 0.012
    for bay in range(1, bay_count):
        panel_angle = bay * bay_step
        lantern_body.visual(
            Box((panel_thickness, panel_width, glazing_height)),
            origin=Origin(
                xyz=(
                    panel_radius * math.cos(panel_angle),
                    panel_radius * math.sin(panel_angle),
                    (glazing_base_z + glazing_top_z) * 0.5,
                ),
                rpy=(0.0, 0.0, panel_angle),
            ),
            material=lantern_glass,
            name=f"glass_panel_{bay:02d}",
        )

    door_width = 0.32
    door_height = glazing_top_z - 0.20
    jamb_thickness = 0.045
    frame_depth = 0.04
    jamb_center_z = 0.20 + door_height * 0.5
    half_door_width = door_width * 0.5

    lantern_body.visual(
        Box((frame_depth, door_width + 0.10, 0.02)),
        origin=Origin(xyz=(panel_radius + frame_depth * 0.5, 0.0, 0.19)),
        material=weathered_iron,
        name="door_threshold",
    )
    lantern_body.visual(
        Box((frame_depth, jamb_thickness, door_height)),
        origin=Origin(
            xyz=(panel_radius + frame_depth * 0.5, -half_door_width - jamb_thickness * 0.5, jamb_center_z)
        ),
        material=cast_iron,
        name="door_jamb_left",
    )
    lantern_body.visual(
        Box((frame_depth, jamb_thickness, door_height)),
        origin=Origin(
            xyz=(panel_radius + frame_depth * 0.5, half_door_width + jamb_thickness * 0.5, jamb_center_z)
        ),
        material=cast_iron,
        name="door_jamb_right",
    )
    lantern_body.visual(
        Box((frame_depth, door_width + 2.0 * jamb_thickness, 0.04)),
        origin=Origin(
            xyz=(
                panel_radius + frame_depth * 0.5,
                0.0,
                glazing_top_z + 0.02,
            )
        ),
        material=weathered_iron,
        name="door_header",
    )

    lantern_body.visual(
        upper_cornice,
        origin=Origin(xyz=(0.0, 0.0, glazing_top_z)),
        material=weathered_iron,
        name="upper_cornice",
    )
    lantern_body.visual(
        roof_shell,
        origin=Origin(xyz=(0.0, 0.0, roof_base_z)),
        material=cast_iron,
        name="roof_shell",
    )
    lantern_body.visual(
        roof_vent_skirt,
        origin=Origin(xyz=(0.0, 0.0, roof_base_z + 0.84)),
        material=weathered_iron,
        name="roof_vent_skirt",
    )
    lantern_body.visual(
        Cylinder(radius=0.085, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, roof_base_z + 0.96)),
        material=dark_steel,
        name="roof_vent_stack",
    )
    lantern_body.visual(
        Sphere(radius=0.09),
        origin=Origin(xyz=(0.0, 0.0, roof_base_z + 1.10)),
        material=dark_steel,
        name="roof_finial",
    )
    lantern_body.inertial = Inertial.from_geometry(
        Cylinder(radius=1.14, length=3.0),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
    )

    central_shaft = model.part("central_shaft")
    central_shaft.visual(
        Cylinder(radius=0.17, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=weathered_iron,
        name="pedestal",
    )
    central_shaft.visual(
        Cylinder(radius=0.19, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=dark_steel,
        name="bearing_race",
    )
    central_shaft.visual(
        Cylinder(radius=0.045, length=1.72),
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        material=dark_steel,
        name="shaft_column",
    )
    central_shaft.visual(
        Cylinder(radius=0.075, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.94)),
        material=weathered_iron,
        name="top_bearing",
    )
    central_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=2.0),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
    )

    turntable_ring = annular_mesh(
        "turntable_ring",
        inner_radius=0.055,
        outer_radius=0.20,
        height=0.035,
    )
    lower_optic_band = annular_mesh(
        "lower_optic_band",
        inner_radius=0.17,
        outer_radius=0.24,
        height=0.05,
    )
    upper_optic_band = annular_mesh(
        "upper_optic_band",
        inner_radius=0.17,
        outer_radius=0.24,
        height=0.05,
    )
    optic_drum = annular_mesh(
        "optic_drum",
        inner_radius=0.14,
        outer_radius=0.21,
        height=0.56,
    )
    optic_cap = annular_mesh(
        "optic_cap_v2",
        inner_radius=0.10,
        outer_radius=0.22,
        height=0.10,
    )

    beacon_carriage = model.part("beacon_carriage")
    beacon_carriage.visual(turntable_ring, material=dark_steel, name="turntable_ring")
    for post_x in (-0.18, 0.18):
        for post_y in (-0.08, 0.08):
            beacon_carriage.visual(
                Cylinder(radius=0.017, length=0.72),
                origin=Origin(xyz=(post_x, post_y, 0.36)),
                material=brass,
                name=f"optic_post_{'p' if post_x > 0 else 'n'}x_{'p' if post_y > 0 else 'n'}y",
            )
    beacon_carriage.visual(
        lower_optic_band,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=brass,
        name="lower_optic_band",
    )
    beacon_carriage.visual(
        optic_drum,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=optic_glass,
        name="optic_drum",
    )
    beacon_carriage.visual(
        upper_optic_band,
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        material=brass,
        name="upper_optic_band",
    )
    beacon_carriage.visual(
        optic_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        material=brass,
        name="optic_cap",
    )
    beacon_carriage.visual(
        Box((0.20, 0.18, 0.04)),
        origin=Origin(xyz=(0.17, 0.0, 0.13)),
        material=dark_steel,
        name="motor_arm",
    )
    beacon_carriage.visual(
        Box((0.16, 0.12, 0.18)),
        origin=Origin(xyz=(0.34, 0.0, 0.20)),
        material=machinery_red,
        name="drive_motor",
    )
    beacon_carriage.visual(
        Box((0.11, 0.08, 0.12)),
        origin=Origin(xyz=(-0.31, 0.0, 0.18)),
        material=weathered_iron,
        name="counterweight_box",
    )
    beacon_carriage.visual(
        Box((0.18, 0.14, 0.04)),
        origin=Origin(xyz=(-0.20, 0.0, 0.13)),
        material=dark_steel,
        name="counterweight_arm",
    )
    beacon_carriage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.40, length=0.90),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
    )

    service_door = model.part("service_door")
    door_thickness = 0.03
    rail_width = 0.045
    service_door.visual(
        Box((door_thickness, rail_width, door_height)),
        origin=Origin(xyz=(door_thickness * 0.5, -rail_width * 0.5, door_height * 0.5)),
        material=cast_iron,
        name="hinge_stile",
    )
    service_door.visual(
        Box((door_thickness, rail_width, door_height)),
        origin=Origin(
            xyz=(door_thickness * 0.5, -door_width + rail_width * 0.5, door_height * 0.5)
        ),
        material=cast_iron,
        name="latch_stile",
    )
    service_door.visual(
        Box((door_thickness, door_width, rail_width)),
        origin=Origin(xyz=(door_thickness * 0.5, -door_width * 0.5, rail_width * 0.5)),
        material=weathered_iron,
        name="bottom_rail",
    )
    service_door.visual(
        Box((door_thickness, door_width, rail_width)),
        origin=Origin(
            xyz=(door_thickness * 0.5, -door_width * 0.5, door_height - rail_width * 0.5)
        ),
        material=weathered_iron,
        name="top_rail",
    )
    service_door.visual(
        Box((0.012, door_width - 2.0 * rail_width, door_height - 2.0 * rail_width)),
        origin=Origin(
            xyz=(
                door_thickness * 0.6,
                -door_width * 0.5,
                door_height * 0.5,
            )
        ),
        material=lantern_glass,
        name="door_pane",
    )
    service_door.visual(
        Cylinder(radius=0.010, length=0.08),
        origin=Origin(
            xyz=(door_thickness + 0.008, -door_width + 0.055, door_height * 0.5),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="door_handle",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((door_thickness, door_width, door_height)),
        mass=42.0,
        origin=Origin(xyz=(door_thickness * 0.5, -door_width * 0.5, door_height * 0.5)),
    )

    model.articulation(
        "body_to_shaft",
        ArticulationType.FIXED,
        parent=lantern_body,
        child=central_shaft,
        origin=Origin(),
    )
    model.articulation(
        "beacon_rotation",
        ArticulationType.CONTINUOUS,
        parent=central_shaft,
        child=beacon_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2),
    )
    model.articulation(
        "service_door_hinge",
        ArticulationType.REVOLUTE,
        parent=lantern_body,
        child=service_door,
        origin=Origin(xyz=(panel_radius, half_door_width, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.0,
            lower=0.0,
            upper=1.35,
        ),
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

    lantern_body = object_model.get_part("lantern_body")
    central_shaft = object_model.get_part("central_shaft")
    beacon_carriage = object_model.get_part("beacon_carriage")
    service_door = object_model.get_part("service_door")

    beacon_rotation = object_model.get_articulation("beacon_rotation")
    service_door_hinge = object_model.get_articulation("service_door_hinge")

    ctx.expect_contact(central_shaft, lantern_body, name="shaft_is_mounted_to_floor")
    ctx.expect_contact(beacon_carriage, central_shaft, name="beacon_turntable_contacts_bearing")
    ctx.expect_contact(service_door, lantern_body, name="door_is_seated_in_frame")
    ctx.expect_origin_distance(
        beacon_carriage,
        central_shaft,
        axes="xy",
        max_dist=1e-6,
        name="beacon_is_centered_on_shaft_axis",
    )

    ctx.check(
        "beacon_joint_axis_is_vertical",
        tuple(beacon_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"unexpected beacon axis: {beacon_rotation.axis}",
    )
    ctx.check(
        "door_joint_axis_is_vertical",
        tuple(service_door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"unexpected door axis: {service_door_hinge.axis}",
    )
    ctx.check(
        "beacon_joint_is_unbounded_spin",
        beacon_rotation.motion_limits is not None
        and beacon_rotation.motion_limits.lower is None
        and beacon_rotation.motion_limits.upper is None,
        details="continuous beacon should not have finite lower/upper limits",
    )
    ctx.check(
        "door_opens_outward_from_closed_pose",
        service_door_hinge.motion_limits is not None
        and service_door_hinge.motion_limits.lower == 0.0
        and service_door_hinge.motion_limits.upper is not None
        and service_door_hinge.motion_limits.upper >= 1.2,
        details="service door should start closed and open to a large outward angle",
    )

    def aabb_center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    motor_rest = ctx.part_element_world_aabb(beacon_carriage, elem="drive_motor")
    assert motor_rest is not None
    motor_rest_center = aabb_center(motor_rest)
    ctx.check(
        "beacon_motor_starts_off_axis",
        motor_rest_center[0] > 0.20 and abs(motor_rest_center[1]) < 0.08,
        details=f"unexpected motor rest center: {motor_rest_center}",
    )
    with ctx.pose({beacon_rotation: math.pi * 0.5}):
        motor_quarter = ctx.part_element_world_aabb(beacon_carriage, elem="drive_motor")
        assert motor_quarter is not None
        motor_quarter_center = aabb_center(motor_quarter)
        ctx.check(
            "beacon_motor_orbits_shaft_when_rotated",
            motor_quarter_center[1] > 0.20 and abs(motor_quarter_center[0]) < 0.08,
            details=f"unexpected motor quarter-turn center: {motor_quarter_center}",
        )
        ctx.expect_contact(beacon_carriage, central_shaft, name="beacon_stays_supported_at_quarter_turn")

    door_closed = ctx.part_element_world_aabb(service_door, elem="door_pane")
    assert door_closed is not None
    door_closed_center = aabb_center(door_closed)
    with ctx.pose({service_door_hinge: 1.10}):
        door_open = ctx.part_element_world_aabb(service_door, elem="door_pane")
        assert door_open is not None
        door_open_center = aabb_center(door_open)
        ctx.check(
            "door_leaf_swings_away_from_wall",
            door_open_center[0] > door_closed_center[0] + 0.10
            and door_open_center[1] > door_closed_center[1] + 0.05,
            details=f"door center did not move outward enough: closed={door_closed_center}, open={door_open_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
