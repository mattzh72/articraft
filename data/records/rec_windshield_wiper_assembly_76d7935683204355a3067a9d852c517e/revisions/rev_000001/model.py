from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(
    radius: float,
    *,
    segments: int = 18,
    cx: float = 0.0,
    cy: float = 0.0,
) -> list[tuple[float, float]]:
    return [
        (cx + radius * cos((2.0 * pi * i) / segments), cy + radius * sin((2.0 * pi * i) / segments))
        for i in range(segments)
    ]


def _link_plate_mesh(
    name: str,
    *,
    span: float,
    width: float,
    thickness: float,
    hole_radius: float,
) -> object:
    outer = _shift_profile(rounded_rect_profile(span + width, width, width * 0.5, corner_segments=8), span * 0.5, 0.0)
    holes = [
        list(reversed(_circle_profile(hole_radius, segments=20, cx=0.0, cy=0.0))),
        list(reversed(_circle_profile(hole_radius, segments=20, cx=span, cy=0.0))),
    ]
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            outer,
            holes,
            thickness,
            center=True,
        ),
    )


def _flat_strip_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    width: float,
    thickness: float,
) -> object:
    profile = rounded_rect_profile(width, thickness, radius=min(width, thickness) * 0.45, corner_segments=6)
    return _save_mesh(
        name,
        sweep_profile_along_spline(
            points,
            profile=profile,
            samples_per_segment=12,
            cap_profile=True,
        ),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_wiper_module")

    coated_steel = model.material("coated_steel", rgba=(0.23, 0.25, 0.27, 1.0))
    zinc_steel = model.material("zinc_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.09, 0.10, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.04, 0.04, 0.04, 1.0))
    parkerized = model.material("parkerized", rgba=(0.18, 0.18, 0.19, 1.0))

    drive_link_span = 0.164
    transfer_link_span = 0.528
    pivot_height = 0.068
    linkage_plane = 0.052
    motor_height = 0.035

    carrier_frame = model.part("carrier_frame")
    carrier_frame.inertial = Inertial.from_geometry(
        Box((1.18, 0.22, 0.16)),
        mass=5.4,
        origin=Origin(xyz=(0.0, -0.02, 0.04)),
    )
    carrier_frame.visual(
        Box((1.18, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=coated_steel,
        name="center_beam",
    )
    carrier_frame.visual(
        Box((1.18, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, -0.027, 0.016)),
        material=coated_steel,
        name="rear_flange",
    )
    carrier_frame.visual(
        Box((1.18, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.025, 0.010)),
        material=coated_steel,
        name="front_lip",
    )
    carrier_frame.visual(
        Box((0.19, 0.060, 0.018)),
        origin=Origin(xyz=(0.08, -0.055, 0.019)),
        material=coated_steel,
        name="motor_shelf",
    )
    carrier_frame.visual(
        Box((0.04, 0.03, 0.020)),
        origin=Origin(xyz=(0.08, -0.090, 0.015)),
        material=coated_steel,
        name="motor_bracket",
    )
    carrier_frame.visual(
        Box((0.12, 0.08, 0.050)),
        origin=Origin(xyz=(0.08, -0.106, 0.043)),
        material=black_plastic,
        name="gearbox_housing",
    )
    carrier_frame.visual(
        Cylinder(radius=0.030, length=0.090),
        origin=Origin(xyz=(0.08, -0.151, 0.043), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="motor_can",
    )
    carrier_frame.visual(
        Box((0.036, 0.028, 0.024)),
        origin=Origin(xyz=(0.08, -0.055, 0.023)),
        material=zinc_steel,
        name="motor_pedestal",
    )
    carrier_frame.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(0.08, -0.055, 0.033)),
        material=zinc_steel,
        name="motor_cap",
    )
    for x_pos, prefix in ((0.36, "driver"), (-0.34, "passenger")):
        carrier_frame.visual(
            Box((0.054, 0.040, 0.030)),
            origin=Origin(xyz=(x_pos, 0.0, 0.015)),
            material=coated_steel,
            name=f"{prefix}_base",
        )
        carrier_frame.visual(
            Box((0.012, 0.010, 0.050)),
            origin=Origin(xyz=(x_pos, -0.017, 0.039)),
            material=zinc_steel,
            name=f"{prefix}_cheek_left",
        )
        carrier_frame.visual(
            Box((0.012, 0.010, 0.050)),
            origin=Origin(xyz=(x_pos, 0.017, 0.039)),
            material=zinc_steel,
            name=f"{prefix}_cheek_right",
        )
        carrier_frame.visual(
            Box((0.028, 0.024, 0.006)),
            origin=Origin(xyz=(x_pos, 0.0, 0.027)),
            material=zinc_steel,
            name=f"{prefix}_bridge",
        )
        carrier_frame.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(x_pos, 0.0, 0.066)),
            material=black_plastic,
            name=f"{prefix}_cap",
        )
    for x_pos, y_pos, name in ((0.52, 0.0, "right_mount"), (-0.52, 0.0, "left_mount"), (0.0, -0.020, "center_mount")):
        carrier_frame.visual(
            Box((0.060, 0.030, 0.012)),
            origin=Origin(xyz=(x_pos, y_pos, 0.010)),
            material=coated_steel,
            name=name,
        )

    motor_crank = model.part("motor_crank")
    motor_crank.inertial = Inertial.from_geometry(
        Box((0.08, 0.04, 0.03)),
        mass=0.18,
        origin=Origin(xyz=(0.022, 0.0, 0.010)),
    )
    motor_crank.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=zinc_steel,
        name="hub_washer",
    )
    motor_crank.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=zinc_steel,
        name="hub_post",
    )
    motor_crank.visual(
        Box((0.044, 0.012, 0.006)),
        origin=Origin(xyz=(0.022, 0.0, 0.013)),
        material=coated_steel,
        name="arm_bar",
    )
    motor_crank.visual(
        Box((0.016, 0.016, 0.006)),
        origin=Origin(xyz=(0.034, 0.0, 0.013)),
        material=coated_steel,
        name="arm_head",
    )
    motor_crank.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.042, 0.0, 0.016)),
        material=zinc_steel,
        name="crank_washer",
    )

    drive_link = model.part("drive_link")
    drive_link.inertial = Inertial.from_geometry(
        Box((0.18, 0.03, 0.01)),
        mass=0.24,
        origin=Origin(xyz=(drive_link_span * 0.5, 0.0, 0.002)),
    )
    drive_link.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=parkerized,
        name="motor_eye",
    )
    drive_link.visual(
        Box((drive_link_span, 0.012, 0.004)),
        origin=Origin(xyz=(drive_link_span * 0.5, 0.0, 0.002)),
        material=parkerized,
        name="bar",
    )
    drive_link.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(drive_link_span, 0.0, 0.002)),
        material=parkerized,
        name="driver_eye",
    )

    driver_wiper = model.part("driver_wiper")
    driver_wiper.inertial = Inertial.from_geometry(
        Box((0.78, 0.18, 0.20)),
        mass=1.55,
        origin=Origin(xyz=(-0.31, 0.03, 0.05)),
    )
    driver_wiper.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=zinc_steel,
        name="pivot_shoulder",
    )
    driver_wiper.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=zinc_steel,
        name="pivot_core",
    )
    driver_wiper.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=zinc_steel,
        name="spindle_shaft",
    )
    driver_wiper.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=black_plastic,
        name="pivot_collar",
    )
    driver_wiper.visual(
        Box((0.036, 0.020, 0.012)),
        origin=Origin(xyz=(-0.006, 0.002, 0.114)),
        material=coated_steel,
        name="arm_clamp",
    )
    driver_wiper.visual(
        Box((0.34, 0.014, 0.006)),
        origin=Origin(xyz=(-0.170, 0.010, 0.114)),
        material=coated_steel,
        name="arm_inner",
    )
    driver_wiper.visual(
        Box((0.18, 0.012, 0.006)),
        origin=Origin(xyz=(-0.430, 0.020, 0.114)),
        material=coated_steel,
        name="arm_outer",
    )
    driver_wiper.visual(
        Box((0.060, 0.018, 0.008)),
        origin=Origin(xyz=(-0.550, 0.026, 0.114)),
        material=parkerized,
        name="bridge_yoke",
    )
    driver_wiper.visual(
        Box((0.42, 0.012, 0.008)),
        origin=Origin(xyz=(-0.410, 0.028, 0.106)),
        material=parkerized,
        name="blade_spine",
    )
    driver_wiper.visual(
        Box((0.42, 0.008, 0.004)),
        origin=Origin(xyz=(-0.410, 0.028, 0.101)),
        material=dark_rubber,
        name="blade_rubber",
    )
    driver_wiper.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=zinc_steel,
        name="bellcrank_hub",
    )
    driver_wiper.visual(
        _flat_strip_mesh(
            "driver_drive_leg",
            [(0.0, 0.0, -0.018), (-0.038, -0.028, -0.018), (-0.074, -0.055, -0.014)],
            width=0.012,
            thickness=0.006,
        ),
        material=coated_steel,
        name="drive_leg",
    )
    driver_wiper.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(-0.074, -0.055, -0.014)),
        material=zinc_steel,
        name="drive_washer",
    )
    driver_wiper.visual(
        Box((0.086, 0.012, 0.006)),
        origin=Origin(xyz=(-0.043, 0.032, -0.018)),
        material=coated_steel,
        name="transfer_leg",
    )
    driver_wiper.visual(
        _flat_strip_mesh(
            "driver_transfer_web",
            [(-0.012, 0.008, -0.018), (-0.026, 0.022, -0.020), (-0.040, 0.032, -0.018)],
            width=0.008,
            thickness=0.004,
        ),
        material=coated_steel,
        name="transfer_web",
    )
    driver_wiper.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(-0.086, 0.032, -0.018)),
        material=zinc_steel,
        name="transfer_washer",
    )

    transfer_link = model.part("transfer_link")
    transfer_link.inertial = Inertial.from_geometry(
        Box((0.56, 0.03, 0.01)),
        mass=0.30,
        origin=Origin(xyz=(-transfer_link_span * 0.5, 0.0, 0.002)),
    )
    transfer_link.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=parkerized,
        name="driver_eye",
    )
    transfer_link.visual(
        Box((transfer_link_span, 0.012, 0.004)),
        origin=Origin(xyz=(-transfer_link_span * 0.5, 0.0, 0.002)),
        material=parkerized,
        name="bar",
    )
    transfer_link.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(-transfer_link_span, 0.0, 0.002)),
        material=parkerized,
        name="passenger_eye",
    )

    passenger_wiper = model.part("passenger_wiper")
    passenger_wiper.inertial = Inertial.from_geometry(
        Box((0.64, 0.16, 0.20)),
        mass=1.30,
        origin=Origin(xyz=(0.24, 0.03, 0.05)),
    )
    passenger_wiper.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=zinc_steel,
        name="pivot_shoulder",
    )
    passenger_wiper.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=zinc_steel,
        name="pivot_core",
    )
    passenger_wiper.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=zinc_steel,
        name="spindle_shaft",
    )
    passenger_wiper.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=black_plastic,
        name="pivot_collar",
    )
    passenger_wiper.visual(
        Box((0.036, 0.020, 0.012)),
        origin=Origin(xyz=(0.006, 0.002, 0.114)),
        material=coated_steel,
        name="arm_clamp",
    )
    passenger_wiper.visual(
        Box((0.22, 0.014, 0.006)),
        origin=Origin(xyz=(0.110, 0.002, 0.114)),
        material=coated_steel,
        name="arm_inner",
    )
    passenger_wiper.visual(
        Box((0.20, 0.012, 0.006)),
        origin=Origin(xyz=(0.320, -0.004, 0.114)),
        material=coated_steel,
        name="arm_outer",
    )
    passenger_wiper.visual(
        Box((0.060, 0.018, 0.008)),
        origin=Origin(xyz=(0.450, -0.010, 0.114)),
        material=parkerized,
        name="bridge_yoke",
    )
    passenger_wiper.visual(
        Box((0.34, 0.012, 0.008)),
        origin=Origin(xyz=(0.320, -0.014, 0.106)),
        material=parkerized,
        name="blade_spine",
    )
    passenger_wiper.visual(
        Box((0.34, 0.008, 0.004)),
        origin=Origin(xyz=(0.320, -0.014, 0.101)),
        material=dark_rubber,
        name="blade_rubber",
    )
    passenger_wiper.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=zinc_steel,
        name="bellcrank_hub",
    )
    passenger_wiper.visual(
        Box((0.086, 0.012, 0.006)),
        origin=Origin(xyz=(0.043, 0.032, -0.018)),
        material=coated_steel,
        name="transfer_leg",
    )
    passenger_wiper.visual(
        _flat_strip_mesh(
            "passenger_transfer_web",
            [(0.012, 0.008, -0.018), (0.026, 0.022, -0.020), (0.040, 0.032, -0.018)],
            width=0.008,
            thickness=0.004,
        ),
        material=coated_steel,
        name="transfer_web",
    )
    passenger_wiper.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.086, 0.032, -0.018)),
        material=zinc_steel,
        name="transfer_washer",
    )

    model.articulation(
        "motor_output",
        ArticulationType.CONTINUOUS,
        parent=carrier_frame,
        child=motor_crank,
        origin=Origin(xyz=(0.08, -0.055, motor_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0),
    )
    model.articulation(
        "drive_link_hinge",
        ArticulationType.REVOLUTE,
        parent=motor_crank,
        child=drive_link,
        origin=Origin(xyz=(0.042, 0.0, linkage_plane - motor_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-1.3, upper=1.3),
    )
    model.articulation(
        "driver_sweep",
        ArticulationType.REVOLUTE,
        parent=carrier_frame,
        child=driver_wiper,
        origin=Origin(xyz=(0.36, 0.0, pivot_height)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.12),
    )
    model.articulation(
        "transfer_link_hinge",
        ArticulationType.REVOLUTE,
        parent=driver_wiper,
        child=transfer_link,
        origin=Origin(xyz=(-0.086, 0.032, linkage_plane - pivot_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-0.8, upper=0.8),
    )
    model.articulation(
        "passenger_sweep",
        ArticulationType.REVOLUTE,
        parent=carrier_frame,
        child=passenger_wiper,
        origin=Origin(xyz=(-0.34, 0.0, pivot_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=0.0, upper=1.00),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carrier_frame = object_model.get_part("carrier_frame")
    motor_crank = object_model.get_part("motor_crank")
    drive_link = object_model.get_part("drive_link")
    driver_wiper = object_model.get_part("driver_wiper")
    transfer_link = object_model.get_part("transfer_link")
    passenger_wiper = object_model.get_part("passenger_wiper")

    motor_output = object_model.get_articulation("motor_output")
    drive_link_hinge = object_model.get_articulation("drive_link_hinge")
    driver_sweep = object_model.get_articulation("driver_sweep")
    transfer_link_hinge = object_model.get_articulation("transfer_link_hinge")
    passenger_sweep = object_model.get_articulation("passenger_sweep")

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

    ctx.expect_contact(driver_wiper, carrier_frame, elem_a="pivot_shoulder", elem_b="driver_cap")
    ctx.expect_contact(passenger_wiper, carrier_frame, elem_a="pivot_shoulder", elem_b="passenger_cap")
    ctx.expect_contact(motor_crank, carrier_frame, elem_a="hub_washer", elem_b="motor_cap")
    ctx.expect_contact(drive_link, motor_crank, elem_a="motor_eye", elem_b="crank_washer")
    ctx.expect_contact(drive_link, driver_wiper, elem_a="driver_eye", elem_b="drive_washer")
    ctx.expect_contact(transfer_link, driver_wiper, elem_a="driver_eye", elem_b="transfer_washer")
    ctx.expect_contact(transfer_link, passenger_wiper, elem_a="passenger_eye", elem_b="transfer_washer")

    driver_closed = ctx.part_element_world_aabb(driver_wiper, elem="blade_rubber")
    passenger_closed = ctx.part_element_world_aabb(passenger_wiper, elem="blade_rubber")

    with ctx.pose(
        {
            motor_output: 0.95,
            drive_link_hinge: 0.18,
            driver_sweep: 0.86,
            transfer_link_hinge: 0.07,
            passenger_sweep: 0.64,
        }
    ):
        driver_open = ctx.part_element_world_aabb(driver_wiper, elem="blade_rubber")
        passenger_open = ctx.part_element_world_aabb(passenger_wiper, elem="blade_rubber")
        driver_closed_center = _aabb_center(driver_closed)
        passenger_closed_center = _aabb_center(passenger_closed)
        driver_open_center = _aabb_center(driver_open)
        passenger_open_center = _aabb_center(passenger_open)

        ctx.check(
            "driver_blade_sweeps_upwind",
            driver_closed_center is not None
            and driver_open_center is not None
            and driver_open_center[1] > driver_closed_center[1] + 0.18,
            details=f"closed={driver_closed_center}, open={driver_open_center}",
        )
        ctx.check(
            "passenger_blade_sweeps_upwind",
            passenger_closed_center is not None
            and passenger_open_center is not None
            and passenger_open_center[1] > passenger_closed_center[1] + 0.14,
            details=f"closed={passenger_closed_center}, open={passenger_open_center}",
        )
        ctx.expect_overlap(driver_wiper, carrier_frame, axes="x", elem_a="blade_rubber", elem_b="center_beam", min_overlap=0.10, name="driver_blade_projects_over_frame")
        ctx.expect_overlap(passenger_wiper, carrier_frame, axes="x", elem_a="blade_rubber", elem_b="center_beam", min_overlap=0.10, name="passenger_blade_projects_over_frame")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
