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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_section(
    width: float,
    depth: float,
    z: float,
    *,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
    radius: float | None = None,
) -> list[tuple[float, float, float]]:
    if radius is None:
        radius = min(width, depth) * 0.13
    return [
        (x + x_shift, y + y_shift, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=10)
    ]


def _build_star_base_mesh() -> object:
    leg_profile = rounded_rect_profile(0.062, 0.028, 0.010, corner_segments=6)
    leg_path = [
        (0.086, 0.0, 0.060),
        (0.180, 0.0, 0.067),
        (0.270, 0.0, 0.062),
        (0.325, 0.0, 0.050),
    ]
    base_geom = TorusGeometry(
        radius=0.075,
        tube=0.016,
        radial_segments=16,
        tubular_segments=54,
    ).translate(0.0, 0.0, 0.058)
    for index in range(5):
        angle = index * (2.0 * math.pi / 5.0)
        leg_geom = sweep_profile_along_spline(
            leg_path,
            profile=leg_profile,
            samples_per_segment=12,
            cap_profile=True,
        ).rotate_z(angle)
        base_geom.merge(leg_geom)
    return _mesh("task_chair_star_base", base_geom)


def _build_seat_cushion_mesh() -> object:
    cushion_geom = section_loft(
        [
            _xy_section(0.440, 0.390, 0.000, y_shift=-0.008, radius=0.052),
            _xy_section(0.495, 0.455, 0.034, y_shift=0.004, radius=0.062),
            _xy_section(0.500, 0.462, 0.072, y_shift=0.012, radius=0.060),
        ]
    )
    return _mesh("task_chair_seat_cushion", cushion_geom)


def _build_backrest_mesh(name: str, *, shell: bool) -> object:
    if shell:
        sections = [
            _xy_section(0.360, 0.024, 0.000, y_shift=0.008, radius=0.010),
            _xy_section(0.392, 0.020, 0.210, y_shift=-0.002, radius=0.009),
            _xy_section(0.334, 0.018, 0.505, y_shift=-0.022, radius=0.009),
        ]
    else:
        sections = [
            _xy_section(0.392, 0.060, 0.000, y_shift=0.010, radius=0.024),
            _xy_section(0.420, 0.055, 0.210, y_shift=-0.004, radius=0.022),
            _xy_section(0.350, 0.050, 0.515, y_shift=-0.028, radius=0.020),
        ]
    return _mesh(name, section_loft(sections))


def _add_column_clip(
    part,
    *,
    angles: tuple[float, ...],
    pin_radius: float,
    pin_center_radius: float,
    pin_length: float,
    arm_center_radius: float,
    arm_length: float,
    arm_height: float,
    z_center: float,
    material,
    prefix: str,
) -> None:
    for clip_index, angle in enumerate(angles):
        px = pin_center_radius * math.cos(angle)
        py = pin_center_radius * math.sin(angle)
        part.visual(
            Cylinder(radius=pin_radius, length=pin_length),
            origin=Origin(xyz=(px, py, z_center)),
            material=material,
            name=f"{prefix}_pin_{clip_index}",
        )
        ax = arm_center_radius * math.cos(angle)
        ay = arm_center_radius * math.sin(angle)
        part.visual(
            Box((arm_length, 0.010, arm_height)),
            origin=Origin(xyz=(ax, ay, z_center), rpy=(0.0, 0.0, angle)),
            material=material,
            name=f"{prefix}_arm_{clip_index}",
        )


def _add_caster_fork_geometry(part, *, material) -> None:
    part.visual(
        Cylinder(radius=0.0075, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=material,
        name="stem",
    )
    part.visual(
        Box((0.016, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=material,
        name="neck_block",
    )
    part.visual(
        Box((0.018, 0.036, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, -0.028)),
        material=material,
        name="bridge",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Box((0.004, 0.004, 0.044)),
            origin=Origin(xyz=(0.020, sign * 0.015, -0.046)),
            material=material,
            name=f"{side}_arm",
        )
    part.visual(
        Cylinder(radius=0.003, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, -0.054), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="axle",
    )


def _add_caster_wheel_geometry(part, *, material, hub_material) -> None:
    wheel_spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.023, length=0.012),
        origin=wheel_spin_origin,
        material=material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=wheel_spin_origin,
        material=hub_material,
        name="hub",
    )
    part.visual(
        Box((0.006, 0.005, 0.005)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=material,
        name="hub_mark",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="armless_task_chair")

    fabric = model.material("fabric", rgba=(0.14, 0.15, 0.17, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    structure_dark = model.material("structure_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    caster_rubber = model.material("caster_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    caster_hub = model.material("caster_hub", rgba=(0.28, 0.29, 0.31, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.86, 1.0))
    mechanism_metal = model.material("mechanism_metal", rgba=(0.32, 0.34, 0.36, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Cylinder(radius=0.090, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=shell_dark,
        name="lower_hub",
    )
    base_frame.visual(
        Cylinder(radius=0.074, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=shell_dark,
        name="hub_cap",
    )
    base_frame.visual(
        Cylinder(radius=0.047, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=shell_dark,
        name="hub_neck",
    )
    _add_column_clip(
        base_frame,
        angles=(0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0),
        pin_radius=0.008,
        pin_center_radius=0.045,
        pin_length=0.160,
        arm_center_radius=0.062,
        arm_length=0.034,
        arm_height=0.065,
        z_center=0.094,
        material=shell_dark,
        prefix="base_clip",
    )
    star_offset = math.pi / 2.0
    caster_radius = 0.344
    caster_z = 0.057
    base_frame.inertial = Inertial.from_geometry(
        Box((0.70, 0.70, 0.16)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )
    for index in range(5):
        angle = star_offset + index * (2.0 * math.pi / 5.0)
        base_frame.visual(
            Box((0.315, 0.042, 0.018)),
            origin=Origin(
                xyz=(
                    0.185 * math.cos(angle),
                    0.185 * math.sin(angle),
                    0.057,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=shell_dark,
            name=f"leg_{index}",
        )
        base_frame.visual(
            Box((0.038, 0.034, 0.028)),
            origin=Origin(
                xyz=(
                    caster_radius * math.cos(angle),
                    caster_radius * math.sin(angle),
                    0.071,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=structure_dark,
            name=f"caster_mount_{index}",
        )

    gas_column = model.part("gas_column")
    gas_column.visual(
        Cylinder(radius=0.037, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=shell_dark,
        name="lower_shroud",
    )
    gas_column.visual(
        Cylinder(radius=0.028, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=chrome,
        name="chrome_piston",
    )
    gas_column.visual(
        Cylinder(radius=0.038, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=mechanism_metal,
        name="top_collar",
    )
    gas_column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.320),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    seat_support = model.part("seat_support")
    _add_column_clip(
        seat_support,
        angles=(math.pi / 6.0, math.pi / 6.0 + 2.0 * math.pi / 3.0, math.pi / 6.0 + 4.0 * math.pi / 3.0),
        pin_radius=0.008,
        pin_center_radius=0.046,
        pin_length=0.090,
        arm_center_radius=0.076,
        arm_length=0.072,
        arm_height=0.030,
        z_center=0.004,
        material=mechanism_metal,
        prefix="seat_clip",
    )
    seat_support.visual(
        Box((0.220, 0.180, 0.040)),
        origin=Origin(xyz=(0.0, -0.025, 0.065)),
        material=mechanism_metal,
        name="tilt_mechanism",
    )
    seat_support.visual(
        Box((0.450, 0.390, 0.016)),
        origin=Origin(xyz=(0.0, 0.006, 0.055)),
        material=shell_dark,
        name="seat_shell",
    )
    seat_support.visual(
        _build_seat_cushion_mesh(),
        origin=Origin(xyz=(0.0, 0.010, 0.060)),
        material=fabric,
        name="seat_cushion",
    )
    seat_support.visual(
        Box((0.200, 0.040, 0.090)),
        origin=Origin(xyz=(0.0, -0.205, 0.110)),
        material=structure_dark,
        name="rear_mount_beam",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        seat_support.visual(
            Box((0.014, 0.024, 0.140)),
            origin=Origin(xyz=(sign * 0.097, -0.230, 0.170)),
            material=structure_dark,
            name=f"back_bracket_{side}",
        )
    seat_support.visual(
        Cylinder(radius=0.006, length=0.105),
        origin=Origin(
            xyz=(0.176, -0.030, 0.056),
            rpy=(0.0, math.pi / 2.0, -0.18),
        ),
        material=mechanism_metal,
        name="adjust_lever",
    )
    seat_support.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(
            xyz=(0.226, -0.038, 0.038),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=shell_dark,
        name="lever_knob",
    )
    seat_support.inertial = Inertial.from_geometry(
        Box((0.54, 0.50, 0.22)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.160, 0.018, 0.090)),
        origin=Origin(xyz=(0.0, 0.012, 0.055)),
        material=mechanism_metal,
        name="back_spine",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        backrest.visual(
            Cylinder(radius=0.013, length=0.026),
            origin=Origin(
                xyz=(sign * 0.077, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=mechanism_metal,
            name=f"pivot_boss_{side}",
        )
    backrest.visual(
        _build_backrest_mesh("task_chair_back_pad", shell=False),
        origin=Origin(xyz=(0.0, -0.008, 0.082)),
        material=fabric,
        name="back_pad",
    )
    backrest.visual(
        _build_backrest_mesh("task_chair_back_shell", shell=True),
        origin=Origin(xyz=(0.0, -0.030, 0.078)),
        material=shell_dark,
        name="back_shell",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.44, 0.10, 0.64)),
        mass=4.2,
        origin=Origin(xyz=(0.0, -0.015, 0.320)),
    )

    model.articulation(
        "gas_height",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=gas_column,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.22,
            lower=0.0,
            upper=0.10,
        ),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=gas_column,
        child=seat_support,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.6),
    )
    model.articulation(
        "back_recline",
        ArticulationType.REVOLUTE,
        parent=seat_support,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.230, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.4,
            lower=-0.10,
            upper=0.45,
        ),
    )

    for index in range(5):
        fork = model.part(f"caster_fork_{index}")
        _add_caster_fork_geometry(fork, material=structure_dark)
        fork.inertial = Inertial.from_geometry(
            Box((0.07, 0.07, 0.08)),
            mass=0.18,
            origin=Origin(xyz=(0.010, 0.0, -0.032)),
        )

        angle = star_offset + index * (2.0 * math.pi / 5.0)
        model.articulation(
            f"caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=base_frame,
            child=fork,
            origin=Origin(
                xyz=(
                    caster_radius * math.cos(angle),
                    caster_radius * math.sin(angle),
                    caster_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=9.0),
        )

        for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
            wheel = model.part(f"caster_wheel_{index}_{side_name}")
            _add_caster_wheel_geometry(
                wheel,
                material=caster_rubber,
                hub_material=caster_hub,
            )
            wheel.inertial = Inertial.from_geometry(
                Cylinder(radius=0.023, length=0.012),
                mass=0.08,
                origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            )
            model.articulation(
                f"caster_wheel_spin_{index}_{side_name}",
                ArticulationType.CONTINUOUS,
                parent=fork,
                child=wheel,
                origin=Origin(xyz=(0.020, side_sign * 0.015, -0.054)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=1.5, velocity=40.0),
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
    ctx.allow_overlap(
        base_frame := object_model.get_part("base_frame"),
        gas_column := object_model.get_part("gas_column"),
        reason="Base clip fingers intentionally press-fit the gas column to keep the pedestal captured.",
    )
    ctx.allow_overlap(
        gas_column,
        seat_support := object_model.get_part("seat_support"),
        reason="Seat support clip fingers intentionally capture the gas column so the seat remains carried during swivel and height motion.",
    )
    for index in range(5):
        fork = object_model.get_part(f"caster_fork_{index}")
        ctx.allow_overlap(
            base_frame,
            fork,
            reason="Each caster fork stem is nested into the five-star base socket as a swivel pivot.",
        )
        for side_name in ("left", "right"):
            wheel = object_model.get_part(f"caster_wheel_{index}_{side_name}")
            ctx.allow_overlap(
                fork,
                wheel,
                reason="Each caster wheel hub rotates around the fork axle passing through the hub center.",
            )
    ctx.fail_if_parts_overlap_in_current_pose()

    backrest = object_model.get_part("backrest")
    fork0 = object_model.get_part("caster_fork_0")
    wheel0_left = object_model.get_part("caster_wheel_0_left")

    gas_height = object_model.get_articulation("gas_height")
    seat_swivel = object_model.get_articulation("seat_swivel")
    back_recline = object_model.get_articulation("back_recline")
    caster_swivel_0 = object_model.get_articulation("caster_swivel_0")
    wheel_spin_0_left = object_model.get_articulation("caster_wheel_spin_0_left")

    ctx.expect_contact(
        gas_column,
        base_frame,
        contact_tol=5e-4,
        name="gas_column_clipped_into_base",
    )
    ctx.expect_contact(
        gas_column,
        seat_support,
        contact_tol=5e-4,
        name="seat_carried_by_column",
    )
    ctx.expect_contact(
        backrest,
        seat_support,
        contact_tol=5e-4,
        name="backrest_pivot_mounted_to_seat",
    )
    ctx.expect_contact(
        fork0,
        base_frame,
        contact_tol=5e-4,
        name="caster_fork_mounted_to_leg",
    )
    ctx.expect_contact(
        wheel0_left,
        fork0,
        contact_tol=5e-4,
        name="caster_wheel_carried_by_fork",
    )
    ctx.expect_origin_distance(
        gas_column,
        seat_support,
        axes="xy",
        max_dist=1e-6,
        name="seat_swivel_axis_centered_on_column",
    )

    base_aabb = ctx.part_world_aabb(base_frame)
    seat_cushion_aabb = ctx.part_element_world_aabb(seat_support, elem="seat_cushion")
    wheel_mark_aabb = ctx.part_element_world_aabb(wheel0_left, elem="hub_mark")
    lever_aabb = ctx.part_element_world_aabb(seat_support, elem="adjust_lever")
    back_pad_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
    assert base_aabb is not None
    assert seat_cushion_aabb is not None
    assert wheel_mark_aabb is not None
    assert lever_aabb is not None
    assert back_pad_aabb is not None

    base_span_x = base_aabb[1][0] - base_aabb[0][0]
    base_span_y = base_aabb[1][1] - base_aabb[0][1]
    seat_height = seat_cushion_aabb[1][2]
    ctx.check(
        "five_star_base_has_office_chair_span",
        0.62 <= base_span_x <= 0.72 and 0.62 <= base_span_y <= 0.72,
        details=f"base extents were ({base_span_x:.3f}, {base_span_y:.3f})",
    )
    ctx.check(
        "seat_height_is_task_chair_like_at_rest",
        0.43 <= seat_height <= 0.50,
        details=f"seat top was at z={seat_height:.3f}",
    )

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    lever_center_rest = _aabb_center(lever_aabb)
    wheel_mark_center_rest = _aabb_center(wheel_mark_aabb)
    back_pad_rest = back_pad_aabb
    wheel_pos_rest = ctx.part_world_position(wheel0_left)
    assert wheel_pos_rest is not None

    with ctx.pose({gas_height: 0.10}):
        raised_seat_cushion_aabb = ctx.part_element_world_aabb(seat_support, elem="seat_cushion")
        assert raised_seat_cushion_aabb is not None
        raised_seat_height = raised_seat_cushion_aabb[1][2]
        ctx.check(
            "gas_lift_raises_seat",
            raised_seat_height >= seat_height + 0.095,
            details=f"seat moved from {seat_height:.3f} to {raised_seat_height:.3f}",
        )
        ctx.expect_contact(gas_column, base_frame, contact_tol=5e-4)
        ctx.expect_contact(gas_column, seat_support, contact_tol=5e-4)

    with ctx.pose({seat_swivel: math.pi / 2.0}):
        lever_aabb_swiveled = ctx.part_element_world_aabb(seat_support, elem="adjust_lever")
        assert lever_aabb_swiveled is not None
        lever_center_swiveled = _aabb_center(lever_aabb_swiveled)
        ctx.check(
            "seat_swivel_rotates_underseat_lever_around_column",
            abs(lever_center_swiveled[0] - lever_center_rest[0]) > 0.08
            and abs(lever_center_swiveled[1] - lever_center_rest[1]) > 0.08
            and abs(lever_center_swiveled[2] - lever_center_rest[2]) < 0.01,
            details=(
                f"lever center moved from {lever_center_rest} "
                f"to {lever_center_swiveled}"
            ),
        )
        ctx.expect_contact(gas_column, seat_support, contact_tol=5e-4)

    with ctx.pose({back_recline: 0.38}):
        reclined_back_pad_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
        assert reclined_back_pad_aabb is not None
        ctx.check(
            "backrest_reclines_about_horizontal_pivot",
            reclined_back_pad_aabb[0][1] < back_pad_rest[0][1] - 0.04,
            details=(
                f"backrest min y moved from {back_pad_rest[0][1]:.3f} "
                f"to {reclined_back_pad_aabb[0][1]:.3f}"
            ),
        )
        ctx.expect_contact(backrest, seat_support, contact_tol=5e-4)

    with ctx.pose({caster_swivel_0: math.pi / 2.0}):
        wheel_pos_swiveled = ctx.part_world_position(wheel0_left)
        assert wheel_pos_swiveled is not None
        ctx.check(
            "caster_fork_swivels_about_vertical_axis",
            abs(wheel_pos_swiveled[0] - wheel_pos_rest[0]) > 0.01
            or abs(wheel_pos_swiveled[1] - wheel_pos_rest[1]) > 0.01,
            details=f"wheel position changed from {wheel_pos_rest} to {wheel_pos_swiveled}",
        )

    with ctx.pose({wheel_spin_0_left: math.pi / 2.0}):
        wheel_mark_aabb_spun = ctx.part_element_world_aabb(wheel0_left, elem="hub_mark")
        assert wheel_mark_aabb_spun is not None
        wheel_mark_center_spun = _aabb_center(wheel_mark_aabb_spun)
        ctx.check(
            "caster_wheel_spins_on_axle",
            math.dist(wheel_mark_center_rest, wheel_mark_center_spun) > 0.015,
            details=(
                f"hub mark moved from {wheel_mark_center_rest} "
                f"to {wheel_mark_center_spun}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
