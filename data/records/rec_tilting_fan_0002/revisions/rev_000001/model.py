from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    repair_loft,
    section_loft,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _radial_pattern_y(base_geometry: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geometry.copy().rotate_y(angle_offset + (index * math.tau / count)))
    return patterned


def _blade_section(radius: float, chord: float, thickness: float, twist: float, camber: float) -> list[tuple[float, float, float]]:
    loop_2d = [
        (-0.52 * chord, 0.00 * thickness + camber),
        (-0.18 * chord, 0.52 * thickness + camber),
        (0.32 * chord, 0.44 * thickness + camber),
        (0.54 * chord, 0.04 * thickness + camber),
        (0.16 * chord, -0.44 * thickness + camber),
        (-0.40 * chord, -0.24 * thickness + camber),
    ]
    c = math.cos(twist)
    s = math.sin(twist)
    return [
        (
            radius,
            (y_coord * c) - (z_coord * s),
            (y_coord * s) + (z_coord * c),
        )
        for y_coord, z_coord in loop_2d
    ]


def _build_blade_mesh() -> MeshGeometry:
    base_blade = repair_loft(
        section_loft(
            [
                _blade_section(0.050, 0.066, 0.012, twist=0.30, camber=0.000),
                _blade_section(0.102, 0.054, 0.010, twist=0.16, camber=0.003),
                _blade_section(0.155, 0.036, 0.006, twist=0.04, camber=0.006),
            ]
        )
    )
    return _radial_pattern_y(base_blade, 3, angle_offset=math.pi / 6.0).translate(0.0, 0.030, 0.0)


def _build_handle_mesh() -> MeshGeometry:
    return wire_from_points(
        [
            (-0.118, -0.090, 0.442),
            (-0.090, -0.108, 0.508),
            (0.000, -0.122, 0.542),
            (0.090, -0.108, 0.508),
            (0.118, -0.090, 0.442),
        ],
        radius=0.011,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.040,
        corner_segments=10,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_tilting_fan", assets=ASSETS)

    utility_yellow = model.material("utility_yellow", rgba=(0.83, 0.67, 0.14, 1.0))
    dark_charcoal = model.material("dark_charcoal", rgba=(0.20, 0.21, 0.22, 1.0))
    molded_black = model.material("molded_black", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    zinc_steel = model.material("zinc_steel", rgba=(0.71, 0.73, 0.76, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.73, 0.75, 0.79, 1.0))

    base_frame = model.part("base_frame")
    base_frame.inertial = Inertial.from_geometry(
        Box((0.54, 0.40, 0.46)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
    )
    base_frame.visual(Box((0.18, 0.34, 0.05)), origin=Origin(xyz=(-0.15, 0.0, 0.025)), material=dark_charcoal, name="left_base_rail")
    base_frame.visual(Box((0.18, 0.34, 0.05)), origin=Origin(xyz=(0.15, 0.0, 0.025)), material=dark_charcoal, name="right_base_rail")
    base_frame.visual(Box((0.42, 0.06, 0.04)), origin=Origin(xyz=(0.0, 0.12, 0.055)), material=utility_yellow, name="front_crossmember")
    base_frame.visual(Box((0.42, 0.06, 0.04)), origin=Origin(xyz=(0.0, -0.12, 0.055)), material=utility_yellow, name="rear_crossmember")
    base_frame.visual(Box((0.24, 0.18, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.055)), material=utility_yellow, name="ballast_housing")
    base_frame.visual(Box((0.06, 0.08, 0.30)), origin=Origin(xyz=(-0.15, 0.0, 0.20)), material=utility_yellow, name="left_upright")
    base_frame.visual(Box((0.06, 0.08, 0.30)), origin=Origin(xyz=(0.15, 0.0, 0.20)), material=utility_yellow, name="right_upright")
    base_frame.visual(Box((0.014, 0.06, 0.14)), origin=Origin(xyz=(-0.17, 0.02, 0.37)), material=utility_yellow, name="left_yoke_arm")
    base_frame.visual(Box((0.014, 0.06, 0.14)), origin=Origin(xyz=(0.17, 0.02, 0.37)), material=utility_yellow, name="right_yoke_arm")
    base_frame.visual(Box((0.05, 0.06, 0.10)), origin=Origin(xyz=(-0.15, 0.02, 0.26)), material=utility_yellow, name="left_front_gusset")
    base_frame.visual(Box((0.05, 0.06, 0.10)), origin=Origin(xyz=(0.15, 0.02, 0.26)), material=utility_yellow, name="right_front_gusset")
    base_frame.visual(Box((0.05, 0.06, 0.10)), origin=Origin(xyz=(-0.15, -0.04, 0.28)), material=utility_yellow, name="left_rear_gusset")
    base_frame.visual(Box((0.05, 0.06, 0.10)), origin=Origin(xyz=(0.15, -0.04, 0.28)), material=utility_yellow, name="right_rear_gusset")
    base_frame.visual(Cylinder(radius=0.022, length=0.018), origin=Origin(xyz=(-0.186, 0.02, 0.37), rpy=(0.0, math.pi / 2.0, 0.0)), material=zinc_steel, name="left_pivot_cap")
    base_frame.visual(Cylinder(radius=0.022, length=0.018), origin=Origin(xyz=(0.186, 0.02, 0.37), rpy=(0.0, math.pi / 2.0, 0.0)), material=zinc_steel, name="right_pivot_cap")
    base_frame.visual(Box((0.028, 0.04, 0.28)), origin=Origin(xyz=(-0.12, -0.12, 0.19)), material=molded_black, name="left_handle_post")
    base_frame.visual(Box((0.028, 0.04, 0.28)), origin=Origin(xyz=(0.12, -0.12, 0.19)), material=molded_black, name="right_handle_post")
    base_frame.visual(
        Cylinder(radius=0.012, length=0.24),
        origin=Origin(xyz=(0.0, -0.12, 0.33), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=molded_black,
        name="carry_handle",
    )
    for x_pos in (-0.17, 0.17):
        for y_pos in (-0.13, 0.13):
            suffix = f"{'left' if x_pos < 0.0 else 'right'}_{'rear' if y_pos < 0.0 else 'front'}"
            base_frame.visual(
                Cylinder(radius=0.022, length=0.012),
                origin=Origin(xyz=(x_pos, y_pos, 0.006)),
                material=rubber_black,
                name=f"{suffix}_foot",
            )
    for bolt_index, bolt_x in enumerate((-0.16, -0.055, 0.055, 0.16), start=1):
        base_frame.visual(
            Cylinder(radius=0.007, length=0.010),
            origin=Origin(xyz=(bolt_x, 0.12, 0.08)),
            material=zinc_steel,
            name=f"front_bolt_{bolt_index}",
        )

    head_assembly = model.part("head_assembly")
    head_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.34),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.17, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    head_assembly.visual(
        Cylinder(radius=0.074, length=0.12),
        origin=Origin(xyz=(0.0, 0.115, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=utility_yellow,
        name="motor_body",
    )
    head_assembly.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_charcoal,
        name="rear_cap",
    )
    head_assembly.visual(
        Cylinder(radius=0.088, length=0.030),
        origin=Origin(xyz=(0.0, 0.185, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_charcoal,
        name="front_collar",
    )
    head_assembly.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.203, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_charcoal,
        name="nose_cap",
    )
    head_assembly.visual(Box((0.070, 0.026, 0.036)), origin=Origin(xyz=(0.0, 0.012, -0.050)), material=molded_black, name="switch_box")
    head_assembly.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.0, -0.011, -0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=molded_black,
        name="speed_knob",
    )
    head_assembly.visual(Box((0.006, 0.012, 0.028)), origin=Origin(xyz=(0.0, -0.021, -0.050)), material=zinc_steel, name="speed_pointer")
    head_assembly.visual(Box((0.068, 0.06, 0.10)), origin=Origin(xyz=(-0.108, 0.090, 0.045)), material=utility_yellow, name="left_cheek")
    head_assembly.visual(Box((0.068, 0.06, 0.10)), origin=Origin(xyz=(0.108, 0.090, 0.045)), material=utility_yellow, name="right_cheek")
    head_assembly.visual(Box((0.060, 0.060, 0.040)), origin=Origin(xyz=(-0.102, 0.040, 0.015)), material=utility_yellow, name="left_pivot_bracket")
    head_assembly.visual(Box((0.060, 0.060, 0.040)), origin=Origin(xyz=(0.102, 0.040, 0.015)), material=utility_yellow, name="right_pivot_bracket")
    head_assembly.visual(Box((0.10, 0.05, 0.018)), origin=Origin(xyz=(0.0, 0.080, 0.070)), material=utility_yellow, name="top_rib")
    head_assembly.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(-0.143, 0.000, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_steel,
        name="left_pivot_boss",
    )
    head_assembly.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.143, 0.000, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_steel,
        name="right_pivot_boss",
    )
    head_assembly.visual(
        Cylinder(radius=0.052, length=0.016),
        origin=Origin(xyz=(0.0, 0.182, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_charcoal,
        name="guard_mount_collar",
    )
    head_assembly.visual(
        _save_mesh(
            "utility_fan_rear_guard_ring.obj",
            TorusGeometry(radius=0.205, tube=0.006, radial_segments=16, tubular_segments=48)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, 0.190, 0.0),
        ),
        material=molded_black,
        name="rear_guard_ring",
    )
    head_assembly.visual(
        _save_mesh(
            "utility_fan_mid_guard_ring.obj",
            TorusGeometry(radius=0.205, tube=0.005, radial_segments=14, tubular_segments=48)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, 0.230, 0.0),
        ),
        material=molded_black,
        name="mid_guard_ring",
    )
    head_assembly.visual(
        _save_mesh(
            "utility_fan_front_guard_ring.obj",
            TorusGeometry(radius=0.205, tube=0.006, radial_segments=16, tubular_segments=48)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, 0.270, 0.0),
        ),
        material=molded_black,
        name="front_guard_ring",
    )
    head_assembly.visual(
        _save_mesh(
            "utility_fan_rear_hub_ring.obj",
            TorusGeometry(radius=0.050, tube=0.004, radial_segments=12, tubular_segments=36)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, 0.190, 0.0),
        ),
        material=molded_black,
        name="rear_hub_ring",
    )
    for index, angle in enumerate([index * math.tau / 10.0 for index in range(10)], start=1):
        head_assembly.visual(
            Cylinder(radius=0.004, length=0.092),
            origin=Origin(
                xyz=(0.199 * math.cos(angle), 0.230, 0.199 * math.sin(angle)),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=molded_black,
            name=f"guard_strut_{index}",
        )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0), start=1):
        head_assembly.visual(
            Cylinder(radius=0.0045, length=0.156),
            origin=Origin(
                xyz=(0.127 * math.cos(angle), 0.190, 0.127 * math.sin(angle)),
                rpy=(0.0, (math.pi / 2.0) - angle, 0.0),
            ),
            material=zinc_steel,
            name=f"rear_guard_spoke_{index}",
        )
    for index, angle in enumerate([index * math.tau / 8.0 for index in range(8)], start=1):
        head_assembly.visual(
            Cylinder(radius=0.0035, length=0.120),
            origin=Origin(
                xyz=(0.150 * math.cos(angle), 0.270, 0.150 * math.sin(angle)),
                rpy=(0.0, (math.pi / 2.0) - angle, 0.0),
            ),
            material=molded_black,
            name=f"front_guard_spoke_{index}",
        )
    for fastener_index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0), start=1):
        head_assembly.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(
                xyz=(0.048 * math.cos(angle), 0.040, 0.048 * math.sin(angle)),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc_steel,
            name=f"rear_fastener_{fastener_index}",
        )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=0.10),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.240, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    blade_assembly.visual(
        Cylinder(radius=0.011, length=0.028),
        origin=Origin(xyz=(0.0, 0.230, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc_steel,
        name="shaft_stub",
    )
    blade_assembly.visual(
        Cylinder(radius=0.034, length=0.030),
        origin=Origin(xyz=(0.0, 0.248, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blade_gray,
        name="hub",
    )
    blade_assembly.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.268, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc_steel,
        name="retaining_nut",
    )
    for blade_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        blade_assembly.visual(
            Box((0.150, 0.010, 0.050)),
            origin=Origin(
                xyz=(0.075 * math.cos(angle), 0.248, 0.075 * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=blade_gray,
            name=f"blade_{blade_index}",
        )

    model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=head_assembly,
        origin=Origin(xyz=(0.0, 0.02, 0.37)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.6, lower=-0.30, upper=0.55),
    )
    model.articulation(
        "head_to_blades",
        ArticulationType.CONTINUOUS,
        parent=head_assembly,
        child=blade_assembly,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    head_assembly = object_model.get_part("head_assembly")
    blade_assembly = object_model.get_part("blade_assembly")
    tilt = object_model.get_articulation("base_to_head_tilt")
    blade_spin = object_model.get_articulation("head_to_blades")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_isolated_parts(max_pose_samples=10, name="sampled_pose_no_floating")

    ctx.check(
        "tilt_axis_is_lateral",
        abs(tilt.axis[0]) > 0.99 and abs(tilt.axis[1]) < 1e-6 and abs(tilt.axis[2]) < 1e-6,
        f"tilt axis should run left-right through the side pivots, got {tilt.axis}",
    )
    ctx.check(
        "blade_spin_axis_is_forward",
        abs(blade_spin.axis[1]) > 0.99 and abs(blade_spin.axis[0]) < 1e-6 and abs(blade_spin.axis[2]) < 1e-6,
        f"blade axis should align with the fan's front-to-back axis, got {blade_spin.axis}",
    )

    base_aabb = ctx.part_world_aabb(base_frame)
    guard_aabb = ctx.part_element_world_aabb(head_assembly, elem="front_guard_ring")
    if base_aabb is not None and guard_aabb is not None:
        base_size = tuple(high - low for low, high in zip(base_aabb[0], base_aabb[1]))
        guard_size = tuple(high - low for low, high in zip(guard_aabb[0], guard_aabb[1]))
        ctx.check(
            "utility_fan_proportions",
            0.42 <= base_size[0] <= 0.56 and 0.32 <= base_size[1] <= 0.42 and 0.40 <= guard_size[0] <= 0.44,
            f"base_size={base_size}, guard_size={guard_size}",
        )

    ctx.expect_contact(head_assembly, base_frame, elem_a="left_pivot_boss", elem_b="left_yoke_arm", name="left_pivot_contact_rest")
    ctx.expect_contact(head_assembly, base_frame, elem_a="right_pivot_boss", elem_b="right_yoke_arm", name="right_pivot_contact_rest")
    ctx.expect_contact(blade_assembly, head_assembly, elem_a="shaft_stub", elem_b="nose_cap", name="blade_stub_contacts_nose")
    ctx.expect_contact(head_assembly, head_assembly, elem_a="speed_knob", elem_b="switch_box", name="integrated_speed_knob_mount")
    ctx.expect_within(
        blade_assembly,
        head_assembly,
        axes="xz",
        outer_elem="front_guard_ring",
        margin=0.03,
        name="blade_envelope_within_guard",
    )
    ctx.expect_origin_gap(head_assembly, base_frame, axis="z", min_gap=0.30, max_gap=0.40, name="pivot_height_realistic")

    limits = tilt.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({tilt: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="tilt_lower_no_floating")
            ctx.expect_contact(head_assembly, base_frame, elem_a="left_pivot_boss", elem_b="left_yoke_arm", name="tilt_lower_left_pivot_contact")
            ctx.expect_contact(head_assembly, base_frame, elem_a="right_pivot_boss", elem_b="right_yoke_arm", name="tilt_lower_right_pivot_contact")
        with ctx.pose({tilt: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="tilt_upper_no_floating")
            ctx.expect_contact(head_assembly, base_frame, elem_a="left_pivot_boss", elem_b="left_yoke_arm", name="tilt_upper_left_pivot_contact")
            ctx.expect_contact(head_assembly, base_frame, elem_a="right_pivot_boss", elem_b="right_yoke_arm", name="tilt_upper_right_pivot_contact")

    with ctx.pose({blade_spin: 1.7}):
        ctx.fail_if_parts_overlap_in_current_pose(name="blade_spin_pose_no_overlap")
        ctx.expect_contact(blade_assembly, head_assembly, elem_a="shaft_stub", elem_b="nose_cap", name="blade_spin_stub_contact")
        ctx.expect_within(
            blade_assembly,
            head_assembly,
            axes="xz",
            outer_elem="front_guard_ring",
            margin=0.03,
            name="blade_spin_within_guard",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
