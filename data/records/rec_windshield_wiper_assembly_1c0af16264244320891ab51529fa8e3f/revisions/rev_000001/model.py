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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _add_box(part, size, xyz, material, name, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(part, radius, length, xyz, material, name, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_arm_beam(part, points, width, thickness, material, mesh_name, visual_name) -> None:
    beam = sweep_profile_along_spline(
        points,
        profile=rounded_rect_profile(
            width,
            thickness,
            radius=min(width, thickness) * 0.35,
            corner_segments=5,
        ),
        samples_per_segment=12,
        cap_profile=True,
    )
    part.visual(
        mesh_from_geometry(beam, mesh_name),
        material=material,
        name=visual_name,
    )


def _add_blade_stack(
    part,
    start_xy,
    end_xy,
    z,
    rail_material,
    spoiler_material,
    rubber_material,
) -> None:
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    center = ((start_xy[0] + end_xy[0]) * 0.5, (start_xy[1] + end_xy[1]) * 0.5, z)

    _add_box(
        part,
        (length, 0.012, 0.004),
        center,
        rail_material,
        "blade_rail",
        rpy=(0.0, 0.0, yaw),
    )
    _add_box(
        part,
        (length * 0.92, 0.018, 0.006),
        (center[0], center[1], z + 0.005),
        spoiler_material,
        "blade_spoiler",
        rpy=(0.0, 0.0, yaw),
    )
    _add_box(
        part,
        (length * 0.98, 0.004, 0.008),
        (center[0], center[1], z - 0.006),
        rubber_material,
        "blade_rubber",
        rpy=(0.0, 0.0, yaw),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_windshield_wiper_assembly")

    painted_metal = model.material("painted_metal", color=(0.18, 0.19, 0.21))
    linkage_metal = model.material("linkage_metal", color=(0.56, 0.58, 0.60))
    satin_polymer = model.material("satin_polymer", color=(0.10, 0.10, 0.11))
    elastomer = model.material("elastomer", color=(0.04, 0.04, 0.04))

    body = model.part("cowl_module")
    _add_box(body, (0.94, 0.030, 0.018), (0.0, -0.030, 0.009), painted_metal, "rear_beam")
    _add_box(body, (0.90, 0.024, 0.014), (0.0, 0.026, 0.007), painted_metal, "front_beam")
    _add_box(body, (0.12, 0.078, 0.018), (-0.41, -0.002, 0.009), painted_metal, "left_mount")
    _add_box(body, (0.12, 0.078, 0.018), (0.41, -0.002, 0.009), painted_metal, "right_mount")

    for name, x in (("driver", -0.27), ("passenger", 0.28)):
        _add_box(body, (0.036, 0.014, 0.030), (x, -0.017, 0.015), satin_polymer, f"{name}_rear_lug")
        _add_box(body, (0.036, 0.014, 0.030), (x, 0.017, 0.015), satin_polymer, f"{name}_front_lug")

    _add_box(body, (0.082, 0.020, 0.026), (0.10, -0.017, -0.005), painted_metal, "motor_rear_brace")
    _add_box(body, (0.074, 0.018, 0.022), (0.10, 0.016, -0.006), painted_metal, "motor_front_brace")
    _add_cylinder(body, 0.009, 0.024, (0.10, 0.0, -0.022), linkage_metal, "motor_output")
    _add_cylinder(
        body,
        0.032,
        0.110,
        (0.10, -0.071, -0.048),
        satin_polymer,
        "motor_can",
        rpy=(math.pi * 0.5, 0.0, 0.0),
    )
    _add_cylinder(
        body,
        0.027,
        0.016,
        (0.10, -0.127, -0.048),
        satin_polymer,
        "motor_tail_cap",
        rpy=(math.pi * 0.5, 0.0, 0.0),
    )

    motor_crank = model.part("motor_crank")
    _add_cylinder(motor_crank, 0.008, 0.018, (0.0, 0.0, 0.009), linkage_metal, "hub_shaft")
    _add_cylinder(motor_crank, 0.016, 0.005, (0.0, 0.0, -0.001), linkage_metal, "hub_flange")
    _add_box(motor_crank, (0.056, 0.012, 0.005), (0.028, 0.0, -0.004), linkage_metal, "crank_arm")
    _add_box(motor_crank, (0.020, 0.010, 0.004), (0.014, -0.015, -0.004), linkage_metal, "counterweight")
    _add_cylinder(motor_crank, 0.006, 0.012, (0.056, 0.0, -0.004), linkage_metal, "drive_pin")

    driver = model.part("driver_wiper")
    _add_cylinder(driver, 0.007, 0.068, (0.0, 0.0, -0.034), linkage_metal, "spindle_core")
    _add_cylinder(driver, 0.019, 0.008, (0.0, 0.0, 0.004), satin_polymer, "base_cap")
    _add_box(driver, (0.090, 0.050, 0.006), (0.045, 0.0, -0.048), linkage_metal, "bellcrank_plate")
    _add_cylinder(driver, 0.005, 0.014, (0.084, 0.0, -0.048), linkage_metal, "link_pin")
    _add_arm_beam(
        driver,
        [
            (0.0, 0.0, 0.004),
            (0.018, 0.090, 0.006),
            (0.070, 0.210, 0.007),
            (0.145, 0.345, 0.007),
            (0.205, 0.445, 0.005),
        ],
        width=0.018,
        thickness=0.006,
        material=painted_metal,
        mesh_name="driver_arm_beam",
        visual_name="arm_beam",
    )
    _add_box(
        driver,
        (0.115, 0.018, 0.008),
        (0.050, 0.115, 0.008),
        satin_polymer,
        "arm_shroud",
        rpy=(0.0, 0.0, math.atan2(0.115, 0.050)),
    )
    _add_blade_stack(
        driver,
        start_xy=(0.070, 0.165),
        end_xy=(0.220, 0.485),
        z=0.004,
        rail_material=linkage_metal,
        spoiler_material=satin_polymer,
        rubber_material=elastomer,
    )

    passenger = model.part("passenger_wiper")
    _add_cylinder(passenger, 0.007, 0.068, (0.0, 0.0, -0.034), linkage_metal, "spindle_core")
    _add_cylinder(passenger, 0.019, 0.008, (0.0, 0.0, 0.004), satin_polymer, "base_cap")
    _add_box(passenger, (0.064, 0.090, 0.006), (-0.030, 0.0, -0.048), linkage_metal, "bellcrank_plate")
    _add_box(passenger, (0.160, 0.006, 0.006), (-0.090, -0.009, -0.048), linkage_metal, "motor_slot_rear")
    _add_box(passenger, (0.160, 0.006, 0.006), (-0.090, 0.009, -0.048), linkage_metal, "motor_slot_front")
    _add_box(passenger, (0.010, 0.024, 0.006), (-0.110, 0.0, -0.048), linkage_metal, "motor_slot_bridge")
    _add_box(passenger, (0.500, 0.006, 0.005), (-0.250, -0.028, -0.046), linkage_metal, "cross_link_rear")
    _add_box(passenger, (0.500, 0.006, 0.005), (-0.250, 0.028, -0.046), linkage_metal, "cross_link_front")
    _add_box(passenger, (0.012, 0.070, 0.005), (-0.500, 0.0, -0.046), linkage_metal, "cross_link_bridge")
    _add_arm_beam(
        passenger,
        [
            (0.0, 0.0, 0.004),
            (-0.018, 0.085, 0.005),
            (-0.072, 0.195, 0.006),
            (-0.145, 0.315, 0.006),
            (-0.215, 0.395, 0.004),
        ],
        width=0.017,
        thickness=0.006,
        material=painted_metal,
        mesh_name="passenger_arm_beam",
        visual_name="arm_beam",
    )
    _add_box(
        passenger,
        (0.100, 0.017, 0.008),
        (-0.045, 0.106, 0.008),
        satin_polymer,
        "arm_shroud",
        rpy=(0.0, 0.0, math.atan2(0.106, -0.045)),
    )
    _add_blade_stack(
        passenger,
        start_xy=(-0.060, 0.145),
        end_xy=(-0.205, 0.415),
        z=0.004,
        rail_material=linkage_metal,
        spoiler_material=satin_polymer,
        rubber_material=elastomer,
    )

    model.articulation(
        "motor_to_crank",
        ArticulationType.REVOLUTE,
        parent=body,
        child=motor_crank,
        origin=Origin(xyz=(0.10, 0.0, -0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=3.0, lower=-0.7, upper=0.8),
    )
    model.articulation(
        "body_to_driver_wiper",
        ArticulationType.REVOLUTE,
        parent=body,
        child=driver,
        origin=Origin(xyz=(-0.27, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=0.65),
    )
    model.articulation(
        "body_to_passenger_wiper",
        ArticulationType.REVOLUTE,
        parent=body,
        child=passenger,
        origin=Origin(xyz=(0.28, 0.0, 0.030)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cowl_module")
    motor_crank = object_model.get_part("motor_crank")
    driver = object_model.get_part("driver_wiper")
    passenger = object_model.get_part("passenger_wiper")
    motor_joint = object_model.get_articulation("motor_to_crank")
    driver_joint = object_model.get_articulation("body_to_driver_wiper")
    passenger_joint = object_model.get_articulation("body_to_passenger_wiper")

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

    ctx.expect_contact(
        driver,
        body,
        elem_a="base_cap",
        elem_b="driver_front_lug",
        name="driver spindle cap seated on support lug",
    )
    ctx.expect_contact(
        passenger,
        body,
        elem_a="base_cap",
        elem_b="passenger_front_lug",
        name="passenger spindle cap seated on support lug",
    )
    ctx.expect_contact(
        motor_crank,
        body,
        elem_a="hub_shaft",
        elem_b="motor_output",
        name="motor crank mounted on output stub",
    )
    ctx.expect_gap(
        driver,
        body,
        axis="z",
        positive_elem="blade_rubber",
        min_gap=0.006,
        name="driver blade clears cowl structure",
        negative_elem="front_beam",
    )
    ctx.expect_gap(
        passenger,
        body,
        axis="z",
        positive_elem="blade_rubber",
        min_gap=0.006,
        name="passenger blade clears cowl structure",
        negative_elem="front_beam",
    )
    ctx.expect_contact(
        motor_crank,
        passenger,
        elem_a="drive_pin",
        elem_b="motor_slot_rear",
        name="crank pin captured by rear slot rail",
    )
    ctx.expect_contact(
        motor_crank,
        passenger,
        elem_a="drive_pin",
        elem_b="motor_slot_front",
        name="crank pin captured by front slot rail",
    )
    ctx.expect_contact(
        passenger,
        driver,
        elem_a="cross_link_rear",
        elem_b="bellcrank_plate",
        name="transfer link touches driver bellcrank rear ear",
    )
    ctx.expect_contact(
        passenger,
        driver,
        elem_a="cross_link_front",
        elem_b="bellcrank_plate",
        name="transfer link touches driver bellcrank front ear",
    )

    with ctx.pose({motor_joint: 0.45, driver_joint: 0.12, passenger_joint: 0.04}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap in coordinated sweep pose")
        ctx.expect_gap(
            driver,
            body,
            axis="z",
            positive_elem="blade_rubber",
            min_gap=0.006,
            name="driver blade stays above cowl in sweep",
            negative_elem="front_beam",
        )
        ctx.expect_gap(
            passenger,
            body,
            axis="z",
            positive_elem="blade_rubber",
            min_gap=0.006,
            name="passenger blade stays above cowl in sweep",
            negative_elem="front_beam",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
