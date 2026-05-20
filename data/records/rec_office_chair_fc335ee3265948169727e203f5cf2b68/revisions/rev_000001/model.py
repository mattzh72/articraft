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
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_loop(
    width: float,
    depth: float,
    radius: float,
    *,
    z: float,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
):
    return [
        (x + x_shift, y + y_shift, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)
    ]


def _seat_cushion_mesh():
    return section_loft(
        [
            _rounded_loop(0.50, 0.46, 0.070, z=0.022, y_shift=0.000),
            _rounded_loop(0.49, 0.45, 0.070, z=0.060, y_shift=0.008),
            _rounded_loop(0.45, 0.40, 0.058, z=0.095, y_shift=0.014),
        ]
    )


def _backrest_mesh():
    return section_loft(
        [
            _rounded_loop(0.32, 0.090, 0.040, z=0.000, y_shift=0.000),
            _rounded_loop(0.41, 0.100, 0.048, z=0.260, y_shift=-0.026),
            _rounded_loop(0.38, 0.082, 0.040, z=0.550, y_shift=-0.058),
        ]
    )


def _build_armrest(model: ArticulatedObject, name: str, sign: float, arm_pad, arm_frame, arm_mount):
    arm = model.part(name)
    arm.visual(
        Box((0.040, 0.052, 0.030)),
        origin=Origin(xyz=(0.020 * sign, -0.010, 0.015)),
        material=arm_mount,
        name="hinge_lug",
    )
    arm.visual(
        Box((0.040, 0.040, 0.045)),
        origin=Origin(xyz=(0.040 * sign, -0.012, 0.038)),
        material=arm_mount,
        name="hinge_web",
    )
    arm.visual(
        Box((0.030, 0.050, 0.118)),
        origin=Origin(xyz=(0.045 * sign, -0.012, 0.059)),
        material=arm_frame,
        name="upright_post",
    )
    arm.visual(
        Box((0.190, 0.285, 0.022)),
        origin=Origin(xyz=(0.118 * sign, 0.004, 0.122)),
        material=arm_frame,
        name="arm_beam",
    )
    arm.visual(
        Box((0.180, 0.250, 0.020)),
        origin=Origin(xyz=(0.118 * sign, 0.004, 0.142)),
        material=arm_pad,
        name="arm_pad",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.18, 0.29, 0.16)),
        mass=1.3,
        origin=Origin(xyz=(0.090 * sign, 0.000, 0.090)),
    )
    return arm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_office_chair")

    base_black = model.material("base_black", rgba=(0.13, 0.14, 0.15, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.19, 0.20, 0.22, 1.0))
    charcoal = model.material("charcoal", rgba=(0.24, 0.25, 0.27, 1.0))
    upholstery = model.material("upholstery", rgba=(0.16, 0.17, 0.18, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.86, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.30, 0.31, 0.34, 1.0))

    seat_mesh = _save_mesh("drafting_chair_seat_cushion", _seat_cushion_mesh())
    back_mesh = _save_mesh("drafting_chair_backrest", _backrest_mesh())
    lower_column_mesh = _save_mesh(
        "drafting_chair_lower_column_shell_v2",
        LatheGeometry.from_shell_profiles(
            [
                (0.056, 0.000),
                (0.054, 0.070),
                (0.051, 0.210),
                (0.048, 0.265),
            ],
            [
                (0.040, 0.010),
                (0.038, 0.070),
                (0.036, 0.206),
                (0.034, 0.257),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    foot_ring_mesh = _save_mesh(
        "drafting_chair_foot_ring",
        TorusGeometry(radius=0.215, tube=0.010, radial_segments=18, tubular_segments=72),
    )
    foot_ring_collar_mesh = _save_mesh(
        "drafting_chair_foot_ring_collar_v2",
        LatheGeometry.from_shell_profiles(
            [
                (0.044, -0.015),
                (0.044, 0.015),
            ],
            [
                (0.0315, -0.015),
                (0.0315, 0.015),
            ],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    base_star = model.part("base_star")
    base_star.visual(
        Cylinder(radius=0.095, length=0.060),
        origin=Origin(xyz=(0.000, 0.000, 0.072)),
        material=base_black,
        name="hub_drum",
    )
    base_star.visual(
        Cylinder(radius=0.072, length=0.024),
        origin=Origin(xyz=(0.000, 0.000, 0.102)),
        material=dark_frame,
        name="upper_shroud",
    )
    for index in range(5):
        angle = index * math.tau / 5.0
        c = math.cos(angle)
        s = math.sin(angle)
        base_star.visual(
            Box((0.285, 0.055, 0.030)),
            origin=Origin(xyz=(0.190 * c, 0.190 * s, 0.060), rpy=(0.000, 0.000, angle)),
            material=base_black,
            name=f"leg_main_{index}",
        )
        base_star.visual(
            Box((0.185, 0.040, 0.018)),
            origin=Origin(xyz=(0.115 * c, 0.115 * s, 0.078), rpy=(0.000, 0.000, angle)),
            material=charcoal,
            name=f"leg_rib_{index}",
        )
        base_star.visual(
            Cylinder(radius=0.020, length=0.010),
            origin=Origin(xyz=(0.345 * c, 0.345 * s, 0.047)),
            material=dark_frame,
            name=f"caster_socket_{index}",
        )
    base_star.inertial = Inertial.from_geometry(
        Box((0.78, 0.78, 0.14)),
        mass=6.5,
        origin=Origin(xyz=(0.000, 0.000, 0.070)),
    )

    lower_column = model.part("lower_column")
    lower_column.visual(lower_column_mesh, material=charcoal, name="column_shell")
    lower_column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=0.265),
        mass=1.6,
        origin=Origin(xyz=(0.000, 0.000, 0.1325)),
    )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Cylinder(radius=0.0315, length=0.595),
        origin=Origin(xyz=(0.000, 0.000, 0.3025)),
        material=chrome,
        name="gas_piston",
    )
    upper_column.visual(
        Cylinder(radius=0.064, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, 0.477)),
        material=dark_frame,
        name="height_collar",
    )
    upper_column.visual(
        Cylinder(radius=0.040, length=0.110),
        origin=Origin(xyz=(0.000, 0.000, 0.540)),
        material=dark_frame,
        name="seat_mount_spindle",
    )
    upper_column.visual(
        Cylinder(radius=0.072, length=0.016),
        origin=Origin(xyz=(0.000, 0.000, 0.587)),
        material=dark_frame,
        name="turntable_plate",
    )
    upper_column.visual(
        Cylinder(radius=0.038, length=0.032),
        origin=Origin(xyz=(0.000, 0.000, 0.050)),
        material=dark_frame,
        name="guide_bushing",
    )
    upper_column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.072, length=0.600),
        mass=1.4,
        origin=Origin(xyz=(0.000, 0.000, 0.300)),
    )

    foot_ring = model.part("foot_ring")
    foot_ring.visual(foot_ring_collar_mesh, material=dark_frame, name="ring_collar")
    foot_ring.visual(foot_ring_mesh, material=aluminum, name="ring_loop")
    for index in range(3):
        angle = index * math.tau / 3.0
        foot_ring.visual(
            Box((0.240, 0.018, 0.014)),
            origin=Origin(
                xyz=(0.110 * math.cos(angle), 0.110 * math.sin(angle), 0.000),
                rpy=(0.000, 0.000, angle),
            ),
            material=aluminum,
            name=f"ring_spoke_{index}",
        )
    foot_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.225, length=0.020),
        mass=1.2,
        origin=Origin(),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.082, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=dark_frame,
        name="seat_receiver",
    )
    seat.visual(
        Box((0.350, 0.290, 0.026)),
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
        material=dark_frame,
        name="seat_pan",
    )
    seat.visual(seat_mesh, material=upholstery, name="seat_cushion")
    seat.visual(
        Box((0.028, 0.070, 0.055)),
        origin=Origin(xyz=(-0.236, -0.025, 0.0325)),
        material=dark_frame,
        name="left_arm_mount",
    )
    seat.visual(
        Box((0.028, 0.070, 0.055)),
        origin=Origin(xyz=(0.236, -0.025, 0.0325)),
        material=dark_frame,
        name="right_arm_mount",
    )
    seat.visual(
        Box((0.190, 0.032, 0.070)),
        origin=Origin(xyz=(0.000, -0.214, 0.050)),
        material=dark_frame,
        name="back_hinge_beam",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.50, 0.46, 0.10)),
        mass=4.8,
        origin=Origin(xyz=(0.000, 0.000, 0.050)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.200, 0.050, 0.045)),
        origin=Origin(xyz=(0.000, -0.025, 0.0225)),
        material=dark_frame,
        name="hinge_block",
    )
    backrest.visual(
        Box((0.180, 0.020, 0.180)),
        origin=Origin(xyz=(0.000, -0.018, 0.110)),
        material=dark_frame,
        name="back_carrier",
    )
    backrest.visual(
        Box((0.110, 0.030, 0.220)),
        origin=Origin(xyz=(0.000, -0.015, 0.155)),
        material=dark_frame,
        name="back_spine_lower",
    )
    backrest.visual(
        Box((0.090, 0.026, 0.240)),
        origin=Origin(xyz=(0.000, 0.004, 0.310)),
        material=charcoal,
        name="back_spine_upper",
    )
    backrest.visual(
        back_mesh,
        origin=Origin(xyz=(0.000, -0.006, 0.060)),
        material=upholstery,
        name="back_cushion",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.44, 0.14, 0.64)),
        mass=3.4,
        origin=Origin(xyz=(0.000, -0.030, 0.320)),
    )

    left_armrest = _build_armrest(
        model,
        "left_armrest",
        -1.0,
        upholstery,
        charcoal,
        dark_frame,
    )
    right_armrest = _build_armrest(
        model,
        "right_armrest",
        1.0,
        upholstery,
        charcoal,
        dark_frame,
    )

    model.articulation(
        "base_to_lower_column",
        ArticulationType.FIXED,
        parent=base_star,
        child=lower_column,
        origin=Origin(xyz=(0.000, 0.000, 0.114)),
    )
    column_lift = model.articulation(
        "column_lift",
        ArticulationType.PRISMATIC,
        parent=lower_column,
        child=upper_column,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.25, lower=0.0, upper=0.120),
    )
    model.articulation(
        "upper_column_to_foot_ring",
        ArticulationType.FIXED,
        parent=upper_column,
        child=foot_ring,
        origin=Origin(xyz=(0.000, 0.000, 0.300)),
    )
    seat_swivel = model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=upper_column,
        child=seat,
        origin=Origin(xyz=(0.000, 0.000, 0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.5),
    )
    back_recline = model.articulation(
        "back_recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(0.000, -0.230, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=0.0, upper=0.420),
    )
    left_arm_flip = model.articulation(
        "left_arm_flip",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=left_armrest,
        origin=Origin(xyz=(-0.250, -0.030, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.350),
    )
    right_arm_flip = model.articulation(
        "right_arm_flip",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=right_armrest,
        origin=Origin(xyz=(0.250, -0.030, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.350),
    )

    for index in range(5):
        fork = model.part(f"caster_fork_{index}")
        fork.visual(
            Cylinder(radius=0.009, length=0.020),
            origin=Origin(xyz=(0.000, 0.000, -0.010)),
            material=dark_frame,
            name="swivel_stem",
        )
        fork.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(0.000, 0.000, -0.002)),
            material=dark_frame,
            name="swivel_cap",
        )
        fork.visual(
            Box((0.026, 0.036, 0.006)),
            origin=Origin(xyz=(0.000, 0.000, -0.016)),
            material=dark_frame,
            name="fork_bridge",
        )
        fork.visual(
            Box((0.020, 0.004, 0.044)),
            origin=Origin(xyz=(0.000, -0.019, -0.034)),
            material=dark_frame,
            name="left_fork_plate",
        )
        fork.visual(
            Box((0.020, 0.004, 0.044)),
            origin=Origin(xyz=(0.000, 0.019, -0.034)),
            material=dark_frame,
            name="right_fork_plate",
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.030, 0.040, 0.040)),
            mass=0.18,
            origin=Origin(xyz=(0.000, 0.000, -0.018)),
        )

        wheel = model.part(f"caster_wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.024, length=0.020),
            origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.000, 0.000)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.006, length=0.034),
            origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.000, 0.000)),
            material=wheel_hub,
            name="hub_barrel",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.024, length=0.034),
            mass=0.22,
            origin=Origin(rpy=(math.pi / 2.0, 0.000, 0.000)),
        )

        angle = index * math.tau / 5.0
        radius = 0.345
        caster_swivel = model.articulation(
            f"caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=base_star,
            child=fork,
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.042),
                rpy=(0.000, 0.000, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=12.0),
        )
        model.articulation(
            f"caster_wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.000, 0.000, -0.051)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=40.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=2e-4)
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    base_star = object_model.get_part("base_star")
    lower_column = object_model.get_part("lower_column")
    upper_column = object_model.get_part("upper_column")
    foot_ring = object_model.get_part("foot_ring")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    left_armrest = object_model.get_part("left_armrest")
    right_armrest = object_model.get_part("right_armrest")

    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        lower_column,
        upper_column,
        reason="The guide bushing telescopes within the lower gas-column shroud.",
    )
    ctx.allow_overlap(
        foot_ring,
        upper_column,
        reason="The foot-ring clamp collar sleeves around the center post.",
    )
    for index in range(5):
        ctx.allow_overlap(
            f"caster_fork_{index}",
            f"caster_wheel_{index}",
            reason="The caster hub rides tightly between fork cheeks at the axle boss.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    column_lift = object_model.get_articulation("column_lift")
    seat_swivel = object_model.get_articulation("seat_swivel")
    back_recline = object_model.get_articulation("back_recline")
    left_arm_flip = object_model.get_articulation("left_arm_flip")
    right_arm_flip = object_model.get_articulation("right_arm_flip")

    ctx.check(
        "column_lift_axis_is_vertical",
        tuple(column_lift.axis) == (0.0, 0.0, 1.0),
        details=f"axis={column_lift.axis}",
    )
    ctx.check(
        "seat_swivel_axis_is_vertical",
        tuple(seat_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"axis={seat_swivel.axis}",
    )
    ctx.check(
        "back_recline_axis_is_transverse",
        tuple(back_recline.axis) == (1.0, 0.0, 0.0),
        details=f"axis={back_recline.axis}",
    )
    ctx.check(
        "left_arm_flip_axis_is_fore_aft",
        tuple(left_arm_flip.axis)[0] == 0.0 and abs(tuple(left_arm_flip.axis)[1]) == 1.0 and tuple(left_arm_flip.axis)[2] == 0.0,
        details=f"axis={left_arm_flip.axis}",
    )
    ctx.check(
        "right_arm_flip_axis_is_fore_aft",
        tuple(right_arm_flip.axis)[0] == 0.0 and abs(tuple(right_arm_flip.axis)[1]) == 1.0 and tuple(right_arm_flip.axis)[2] == 0.0,
        details=f"axis={right_arm_flip.axis}",
    )

    ctx.expect_contact(lower_column, base_star, name="lower_column_is_mounted_to_base")
    ctx.expect_contact(
        upper_column,
        lower_column,
        contact_tol=2e-4,
        name="upper_column_is_seated_in_lower_column",
    )
    ctx.expect_contact(foot_ring, upper_column, name="foot_ring_is_clamped_to_column")
    ctx.expect_contact(seat, upper_column, name="seat_is_carried_by_column")
    ctx.expect_contact(backrest, seat, name="backrest_is_hinged_to_seat")
    ctx.expect_contact(left_armrest, seat, name="left_armrest_is_hinged_to_seat")
    ctx.expect_contact(right_armrest, seat, name="right_armrest_is_hinged_to_seat")

    ctx.expect_gap(foot_ring, base_star, axis="z", min_gap=0.18, max_gap=0.40, name="foot_ring_sits_above_base")
    ctx.expect_gap(seat, foot_ring, axis="z", min_gap=0.20, name="seat_sits_above_foot_ring")
    ctx.expect_gap(seat, base_star, axis="z", min_gap=0.52, name="seat_sits_high_above_wheeled_base")
    ctx.expect_overlap(foot_ring, upper_column, axes="xy", min_overlap=0.06, name="foot_ring_wraps_column")
    ctx.expect_origin_gap(seat, backrest, axis="y", min_gap=0.18, name="backrest_is_behind_seat_center")
    ctx.expect_origin_distance(
        left_armrest,
        right_armrest,
        axes="x",
        min_dist=0.45,
        max_dist=0.55,
        name="armrests_span_seat_width",
    )

    for index in range(5):
        fork = object_model.get_part(f"caster_fork_{index}")
        wheel = object_model.get_part(f"caster_wheel_{index}")
        swivel = object_model.get_articulation(f"caster_swivel_{index}")
        spin = object_model.get_articulation(f"caster_wheel_spin_{index}")

        ctx.check(
            f"caster_swivel_{index}_axis_is_vertical",
            tuple(swivel.axis) == (0.0, 0.0, 1.0),
            details=f"axis={swivel.axis}",
        )
        ctx.check(
            f"caster_wheel_spin_{index}_axis_is_axle_aligned",
            tuple(spin.axis) == (0.0, 1.0, 0.0),
            details=f"axis={spin.axis}",
        )
        ctx.expect_contact(fork, base_star, name=f"caster_fork_{index}_contacts_base")
        ctx.expect_contact(wheel, fork, name=f"caster_wheel_{index}_is_captured_by_fork")

    seat_rest = ctx.part_world_position(seat)
    foot_ring_rest = ctx.part_world_position(foot_ring)
    left_arm_rest_aabb = ctx.part_world_aabb(left_armrest)
    right_arm_rest_aabb = ctx.part_world_aabb(right_armrest)
    back_rest_aabb = ctx.part_world_aabb(backrest)
    left_arm_rest_pos = ctx.part_world_position(left_armrest)
    assert seat_rest is not None
    assert foot_ring_rest is not None
    assert left_arm_rest_aabb is not None
    assert right_arm_rest_aabb is not None
    assert back_rest_aabb is not None
    assert left_arm_rest_pos is not None

    with ctx.pose({column_lift: 0.10}):
        seat_high = ctx.part_world_position(seat)
        foot_ring_high = ctx.part_world_position(foot_ring)
        assert seat_high is not None
        assert foot_ring_high is not None
        ctx.check(
            "column_lift_raises_seat",
            seat_high[2] > seat_rest[2] + 0.095,
            details=f"rest_z={seat_rest[2]:.3f}, high_z={seat_high[2]:.3f}",
        )
        ctx.check(
            "column_lift_raises_foot_ring",
            foot_ring_high[2] > foot_ring_rest[2] + 0.095,
            details=f"rest_z={foot_ring_rest[2]:.3f}, high_z={foot_ring_high[2]:.3f}",
        )

    with ctx.pose({seat_swivel: math.pi / 2.0}):
        left_arm_swiveled = ctx.part_world_position(left_armrest)
        assert left_arm_swiveled is not None
        ctx.check(
            "seat_swivel_rotates_seat_children",
            abs(left_arm_swiveled[0] - left_arm_rest_pos[0]) > 0.12
            and abs(left_arm_swiveled[1] - left_arm_rest_pos[1]) > 0.12,
            details=f"rest={left_arm_rest_pos}, swiveled={left_arm_swiveled}",
        )

    with ctx.pose({back_recline: 0.35}):
        back_reclined_aabb = ctx.part_world_aabb(backrest)
        assert back_reclined_aabb is not None
        ctx.check(
            "back_recline_moves_backrest_rearward",
            back_reclined_aabb[0][1] < back_rest_aabb[0][1] - 0.04,
            details=f"rest_min_y={back_rest_aabb[0][1]:.3f}, reclined_min_y={back_reclined_aabb[0][1]:.3f}",
        )

    with ctx.pose({left_arm_flip: 1.10, right_arm_flip: 1.10}):
        left_arm_up_aabb = ctx.part_world_aabb(left_armrest)
        right_arm_up_aabb = ctx.part_world_aabb(right_armrest)
        assert left_arm_up_aabb is not None
        assert right_arm_up_aabb is not None
        ctx.check(
            "left_arm_flip_raises_armrest",
            left_arm_up_aabb[1][2] > left_arm_rest_aabb[1][2] + 0.03,
            details=f"rest_max_z={left_arm_rest_aabb[1][2]:.3f}, up_max_z={left_arm_up_aabb[1][2]:.3f}",
        )
        ctx.check(
            "right_arm_flip_raises_armrest",
            right_arm_up_aabb[1][2] > right_arm_rest_aabb[1][2] + 0.03,
            details=f"rest_max_z={right_arm_rest_aabb[1][2]:.3f}, up_max_z={right_arm_up_aabb[1][2]:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
