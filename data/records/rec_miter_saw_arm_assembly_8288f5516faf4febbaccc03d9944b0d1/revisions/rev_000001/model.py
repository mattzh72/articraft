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
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _annular_sector_profile(
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    *,
    samples: int = 36,
) -> list[tuple[float, float]]:
    if end_angle <= start_angle:
        end_angle += 2.0 * math.pi
    outer = [
        (math.cos(angle) * outer_radius, math.sin(angle) * outer_radius)
        for angle in [
            start_angle + (end_angle - start_angle) * step / samples
            for step in range(samples + 1)
        ]
    ]
    inner = [
        (math.cos(angle) * inner_radius, math.sin(angle) * inner_radius)
        for angle in [
            end_angle - (end_angle - start_angle) * step / samples
            for step in range(samples + 1)
        ]
    ]
    return outer + inner


def _blade_outer_profile(
    outer_radius: float,
    root_radius: float,
    *,
    teeth: int = 40,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(teeth * 2):
        angle = (2.0 * math.pi * index) / (teeth * 2)
        radius = outer_radius if index % 2 == 0 else root_radius
        points.append((math.cos(angle) * radius, math.sin(angle) * radius))
    return points


def _circle_profile(radius: float, *, samples: int = 48) -> list[tuple[float, float]]:
    return [
        (
            math.cos((2.0 * math.pi * index) / samples) * radius,
            math.sin((2.0 * math.pi * index) / samples) * radius,
        )
        for index in range(samples)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="basic_chop_miter_saw")

    cast_gray = model.material("cast_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    housing_red = model.material("housing_red", rgba=(0.73, 0.10, 0.11, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.84, 0.85, 0.87, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.09, 0.10, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    amber_guard = model.material("amber_guard", rgba=(0.82, 0.58, 0.12, 0.60))

    table_center = (0.0, 0.030, 0.056)
    pivot_origin = (0.0, -0.130, 0.150)
    arbor_local = (0.0, 0.190, 0.145)
    arm_rest_pitch = 0.58
    blade_radius = 0.127

    upper_guard_shell = _save_mesh(
        "upper_guard_shell",
        ExtrudeGeometry(
            _annular_sector_profile(
                outer_radius=0.147,
                inner_radius=0.131,
                start_angle=3.25,
                end_angle=6.20,
                samples=52,
            ),
            0.018,
            center=True,
        ).rotate_y(math.pi / 2.0),
    )
    lower_guard_shell = _save_mesh(
        "lower_guard_shell",
        ExtrudeGeometry(
            _annular_sector_profile(
                outer_radius=0.145,
                inner_radius=0.020,
                start_angle=-0.35,
                end_angle=2.12,
                samples=52,
            ),
            0.018,
            center=True,
        ).rotate_y(math.pi / 2.0),
    )
    blade_mesh = _save_mesh(
        "miter_blade",
        ExtrudeWithHolesGeometry(
            _blade_outer_profile(blade_radius, blade_radius * 0.955, teeth=48),
            [_circle_profile(0.020, samples=36)],
            0.003,
            center=True,
        ).rotate_y(math.pi / 2.0),
    )
    handle_mesh = _save_mesh(
        "carry_handle",
        tube_from_spline_points(
            [
                (-0.040, 0.055, 0.236),
                (-0.040, 0.090, 0.268),
                (0.0, 0.122, 0.290),
                (0.040, 0.090, 0.268),
                (0.040, 0.055, 0.236),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    upper_guard_bracket = _save_mesh(
        "upper_guard_bracket",
        tube_from_spline_points(
            [
                (-0.022, 0.108, 0.206),
                (-0.022, 0.090, 0.236),
                (-0.022, 0.120, 0.264),
                (-0.022, 0.176, 0.279),
            ],
            radius=0.010,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    left_arm_rail = _save_mesh(
        "left_arm_rail",
        tube_from_spline_points(
            [
                (-0.058, 0.018, 0.046),
                (-0.060, 0.060, 0.076),
                (-0.058, 0.110, 0.105),
                (-0.053, 0.155, 0.130),
            ],
            radius=0.011,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    right_arm_rail = _save_mesh(
        "right_arm_rail",
        tube_from_spline_points(
            [
                (0.058, 0.018, 0.046),
                (0.056, 0.060, 0.076),
                (0.052, 0.096, 0.098),
                (0.048, 0.125, 0.116),
            ],
            radius=0.011,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    base = model.part("base")
    base.visual(
        Box((0.560, 0.400, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=cast_gray,
        name="base_casting",
    )
    base.visual(
        Box((0.350, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, 0.172, 0.048)),
        material=cast_gray,
        name="front_apron",
    )
    base.visual(
        Cylinder(radius=0.118, length=0.018),
        origin=Origin(xyz=table_center[:-1] + (0.047,)),
        material=dark_steel,
        name="turntable_pad",
    )
    base.visual(
        Box((0.460, 0.022, 0.082)),
        origin=Origin(xyz=(0.0, -0.108, 0.079)),
        material=machined_aluminum,
        name="fence",
    )
    base.visual(
        Box((0.086, 0.075, 0.136)),
        origin=Origin(xyz=(0.0, -0.130, 0.106)),
        material=cast_gray,
        name="pivot_block",
    )
    base.visual(
        Box((0.018, 0.070, 0.132)),
        origin=Origin(xyz=(0.055, -0.130, 0.104)),
        material=cast_gray,
        name="right_pivot_ear",
    )
    base.visual(
        Box((0.018, 0.070, 0.132)),
        origin=Origin(xyz=(-0.055, -0.130, 0.104)),
        material=cast_gray,
        name="left_pivot_ear",
    )
    base.visual(
        Box((0.128, 0.036, 0.034)),
        origin=Origin(xyz=(0.0, -0.130, 0.161)),
        material=cast_gray,
        name="pivot_bridge",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.560, 0.400, 0.250)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
    )

    table = model.part("miter_table")
    table.visual(
        Cylinder(radius=0.116, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=machined_aluminum,
        name="table_top",
    )
    table.visual(
        Cylinder(radius=0.050, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="table_hub",
    )
    table.visual(
        Box((0.050, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.108, 0.007)),
        material=machined_aluminum,
        name="miter_handle",
    )
    table.inertial = Inertial.from_geometry(
        Cylinder(radius=0.120, length=0.026),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    arm = model.part("saw_arm")
    arm.visual(
        Box((0.014, 0.060, 0.058)),
        origin=Origin(xyz=(-0.073, 0.000, 0.012)),
        material=dark_steel,
        name="left_yoke_cheek",
    )
    arm.visual(
        Box((0.014, 0.060, 0.058)),
        origin=Origin(xyz=(0.073, 0.000, 0.012)),
        material=dark_steel,
        name="right_yoke_cheek",
    )
    arm.visual(
        Box((0.150, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.000, 0.040)),
        material=dark_steel,
        name="rear_clip",
    )
    arm.visual(left_arm_rail, material=dark_steel, name="left_arm_rail")
    arm.visual(right_arm_rail, material=dark_steel, name="right_arm_rail")
    arm.visual(
        Box((0.100, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.090, 0.096)),
        material=dark_steel,
        name="rail_crossbrace",
    )
    arm.visual(
        Box((0.014, 0.132, 0.200)),
        origin=Origin(xyz=(-0.026, 0.146, 0.198)),
        material=dark_steel,
        name="arbor_support_web",
    )
    arm.visual(
        Box((0.014, 0.132, 0.200)),
        origin=Origin(xyz=(0.039, 0.146, 0.198)),
        material=dark_steel,
        name="guard_support_web",
    )
    arm.visual(
        Box((0.020, 0.050, 0.030)),
        origin=Origin(xyz=(-0.054, 0.150, 0.125)),
        material=dark_steel,
        name="head_strut",
    )
    arm.visual(
        Box((0.055, 0.100, 0.070)),
        origin=Origin(xyz=(-0.066, 0.163, 0.150)),
        material=housing_red,
        name="gearbox_bridge",
    )
    arm.visual(
        Cylinder(radius=0.046, length=0.056),
        origin=Origin(xyz=(-0.074, 0.186, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_red,
        name="gear_case",
    )
    arm.visual(
        Box((0.072, 0.070, 0.050)),
        origin=Origin(xyz=(-0.076, 0.128, 0.185)),
        material=housing_red,
        name="motor_mount",
    )
    arm.visual(
        Cylinder(radius=0.040, length=0.125),
        origin=Origin(xyz=(-0.078, 0.110, 0.208), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_red,
        name="motor_housing",
    )
    arm.visual(
        Box((0.018, 0.020, 0.086)),
        origin=Origin(xyz=(-0.040, 0.093, 0.239)),
        material=dark_steel,
        name="handle_post",
    )
    arm.visual(handle_mesh, material=grip_rubber, name="carry_handle")
    arm.visual(
        upper_guard_shell,
        origin=Origin(xyz=(-0.024, arbor_local[1], arbor_local[2])),
        material=housing_red,
        name="upper_guard",
    )
    arm.visual(upper_guard_bracket, material=housing_red, name="upper_guard_bracket")
    arm.visual(
        Cylinder(radius=0.035, length=0.010),
        origin=Origin(
            xyz=(-0.015, arbor_local[1], arbor_local[2]),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="arbor_flange",
    )
    arm.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(
            xyz=(0.026, arbor_local[1], arbor_local[2]),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="guard_bracket",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.220, 0.320, 0.220)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.115, 0.145)),
    )

    blade = model.part("blade")
    blade.visual(blade_mesh, material=blade_steel, name="blade_disk")
    blade.visual(
        Cylinder(radius=0.033, length=0.010),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="blade_hub",
    )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=blade_radius, length=0.010),
        mass=1.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    lower_guard = model.part("lower_guard")
    lower_guard.visual(
        lower_guard_shell,
        origin=Origin(xyz=(0.0108, 0.0, 0.0)),
        material=amber_guard,
        name="guard_shell",
    )
    lower_guard.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="guard_hub",
    )
    lower_guard.inertial = Inertial.from_geometry(
        Box((0.050, 0.180, 0.180)),
        mass=0.7,
        origin=Origin(xyz=(0.016, 0.030, -0.030)),
    )

    model.articulation(
        "base_to_table",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=table_center),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-math.radians(50.0),
            upper=math.radians(50.0),
        ),
    )
    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=pivot_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=-0.50,
            upper=0.08,
        ),
    )
    model.articulation(
        "arm_to_blade",
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=blade,
        origin=Origin(xyz=arbor_local),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=50.0),
    )
    model.articulation(
        "arm_to_lower_guard",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=lower_guard,
        origin=Origin(xyz=arbor_local),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base = object_model.get_part("base")
    table = object_model.get_part("miter_table")
    arm = object_model.get_part("saw_arm")
    blade = object_model.get_part("blade")
    lower_guard = object_model.get_part("lower_guard")

    table_turn = object_model.get_articulation("base_to_table")
    arm_pivot = object_model.get_articulation("base_to_arm")
    blade_spin = object_model.get_articulation("arm_to_blade")
    guard_hinge = object_model.get_articulation("arm_to_lower_guard")

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
        "miter table axis is vertical",
        table_turn.axis == (0.0, 0.0, 1.0),
        f"Expected vertical table axis, got {table_turn.axis!r}.",
    )
    ctx.check(
        "saw arm axis is horizontal",
        arm_pivot.axis == (1.0, 0.0, 0.0),
        f"Expected horizontal chop axis, got {arm_pivot.axis!r}.",
    )
    ctx.check(
        "blade spin axis matches arbor",
        blade_spin.axis == (1.0, 0.0, 0.0),
        f"Expected blade spin about X, got {blade_spin.axis!r}.",
    )
    ctx.check(
        "lower guard hinge matches arbor axis",
        guard_hinge.axis == (1.0, 0.0, 0.0),
        f"Expected guard hinge about X, got {guard_hinge.axis!r}.",
    )

    ctx.expect_contact(table, base, elem_a="table_top", elem_b="turntable_pad")
    ctx.expect_contact(arm, base)
    ctx.expect_contact(blade, arm, elem_a="blade_hub", elem_b="arbor_flange")
    ctx.expect_contact(lower_guard, arm, elem_a="guard_hub", elem_b="guard_bracket")
    ctx.expect_gap(blade, base, axis="y", min_gap=0.020, negative_elem="fence")

    rest_blade_aabb = ctx.part_world_aabb(blade)
    guard_rest_aabb = ctx.part_element_world_aabb(lower_guard, elem="guard_shell")
    handle_rest_aabb = ctx.part_element_world_aabb(table, elem="miter_handle")

    if rest_blade_aabb is None:
        ctx.fail("blade rest bounds available", "Blade AABB was unavailable in the rest pose.")
    if guard_rest_aabb is None:
        ctx.fail("guard rest bounds available", "Lower guard shell AABB was unavailable in the rest pose.")
    if handle_rest_aabb is None:
        ctx.fail("table handle bounds available", "Miter handle AABB was unavailable in the rest pose.")

    with ctx.pose({arm_pivot: -0.42, guard_hinge: 1.05}):
        ctx.expect_overlap(blade, table, axes="y", min_overlap=0.150)
        ctx.expect_origin_distance(blade, table, axes="x", max_dist=0.030)
        ctx.expect_gap(blade, table, axis="z", max_gap=0.018, max_penetration=0.004)
        ctx.expect_gap(blade, base, axis="y", min_gap=0.020, negative_elem="fence")

        cut_blade_aabb = ctx.part_world_aabb(blade)
        if rest_blade_aabb is not None and cut_blade_aabb is not None:
            ctx.check(
                "arm lowers blade toward table",
                cut_blade_aabb[0][2] < rest_blade_aabb[0][2] - 0.050,
                f"Rest min z={rest_blade_aabb[0][2]:.4f}, cut min z={cut_blade_aabb[0][2]:.4f}.",
            )
        else:
            ctx.fail("cut blade bounds available", "Blade AABB was unavailable in lowered pose.")

    with ctx.pose({arm_pivot: -0.42, guard_hinge: 0.0}):
        guard_closed_aabb = ctx.part_element_world_aabb(lower_guard, elem="guard_shell")
    with ctx.pose({arm_pivot: -0.42, guard_hinge: 1.05}):
        guard_open_aabb = ctx.part_element_world_aabb(lower_guard, elem="guard_shell")
    if guard_closed_aabb is not None and guard_open_aabb is not None:
        ctx.check(
            "lower guard retracts as it opens",
            guard_open_aabb[1][2] > guard_closed_aabb[1][2] + 0.040,
            f"Closed guard max z={guard_closed_aabb[1][2]:.4f}, open guard max z={guard_open_aabb[1][2]:.4f}.",
        )
    else:
        ctx.fail("guard comparison bounds available", "Guard shell AABB was unavailable in opened or closed pose.")

    with ctx.pose({table_turn: math.radians(35.0)}):
        ctx.expect_contact(table, base, elem_a="table_top", elem_b="turntable_pad")
        handle_turn_aabb = ctx.part_element_world_aabb(table, elem="miter_handle")
        if handle_rest_aabb is not None and handle_turn_aabb is not None:
            rest_handle_x = 0.5 * (handle_rest_aabb[0][0] + handle_rest_aabb[1][0])
            turned_handle_x = 0.5 * (handle_turn_aabb[0][0] + handle_turn_aabb[1][0])
            ctx.check(
                "miter table handle swings sideways",
                turned_handle_x < rest_handle_x - 0.040,
                f"Rest handle x={rest_handle_x:.4f}, turned handle x={turned_handle_x:.4f}.",
            )
        else:
            ctx.fail("turned handle bounds available", "Miter handle AABB was unavailable in turned pose.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
