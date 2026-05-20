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
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
    section_loft,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annulus_geometry(outer_radius: float, inner_radius: float, height: float) -> MeshGeometry:
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=48),
        [_circle_profile(inner_radius, segments=32)],
        height,
        center=True,
    )


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _blade_section(
    *,
    radius: float,
    sweep_y: float,
    rise_z: float,
    chord: float,
    thickness: float,
    pitch: float,
    yaw: float,
) -> list[tuple[float, float, float]]:
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    points: list[tuple[float, float, float]] = []
    for local_y, local_z in (
        (-0.5 * chord, -0.5 * thickness),
        (0.5 * chord, -0.5 * thickness),
        (0.5 * chord, 0.5 * thickness),
        (-0.5 * chord, 0.5 * thickness),
    ):
        pitched_y = (local_y * cp) - (local_z * sp)
        pitched_z = (local_y * sp) + (local_z * cp)
        world_x = radius - (pitched_y * sy)
        world_y = sweep_y + (pitched_y * cy)
        world_z = rise_z + pitched_z
        points.append((world_x, world_y, world_z))
    return points


def _single_blade_geometry(upward: bool) -> MeshGeometry:
    pitch_sign = 1.0 if upward else -1.0
    sections = [
        _blade_section(
            radius=0.013,
            sweep_y=0.000,
            rise_z=0.0045,
            chord=0.014,
            thickness=0.0018,
            pitch=pitch_sign * 0.52,
            yaw=0.10,
        ),
        _blade_section(
            radius=0.036,
            sweep_y=0.003,
            rise_z=0.0060 if upward else 0.0030,
            chord=0.021,
            thickness=0.0016,
            pitch=pitch_sign * 0.18,
            yaw=0.17,
        ),
        _blade_section(
            radius=0.058,
            sweep_y=0.006,
            rise_z=0.0030 if upward else 0.0060,
            chord=0.016,
            thickness=0.0012,
            pitch=pitch_sign * -0.20,
            yaw=0.25,
        ),
    ]
    return section_loft(sections)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soup_blender")

    base_charcoal = model.material("base_charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    satin_black = model.material("satin_black", rgba=(0.09, 0.10, 0.11, 1.0))
    control_silver = model.material("control_silver", rgba=(0.68, 0.70, 0.73, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.82, 0.90, 0.96, 0.32))
    lid_black = model.material("lid_black", rgba=(0.16, 0.17, 0.18, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.72, 0.75, 0.79, 1.0))

    base = model.part("base")
    base_profile = [
        (0.0, 0.0),
        (0.115, 0.0),
        (0.122, 0.018),
        (0.119, 0.060),
        (0.109, 0.118),
        (0.098, 0.150),
        (0.084, 0.162),
        (0.074, 0.168),
        (0.074, 0.176),
        (0.0, 0.176),
    ]
    base.visual(
        mesh_from_geometry(LatheGeometry(base_profile, segments=72), "blender_base_shell"),
        material=base_charcoal,
        name="base_shell",
    )
    base.visual(
        Box((0.020, 0.078, 0.076)),
        origin=Origin(xyz=(0.112, 0.0, 0.084)),
        material=base_charcoal,
        name="control_pod",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.122, length=0.176),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="dial_bezel",
    )
    control_knob.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_silver,
        name="dial_body",
    )
    control_knob.visual(
        Box((0.004, 0.012, 0.003)),
        origin=Origin(xyz=(0.021, 0.0, 0.017)),
        material=satin_black,
        name="dial_pointer",
    )
    control_knob.inertial = Inertial.from_geometry(
        Box((0.028, 0.060, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
    )

    jar = model.part("jar")
    jar_shell = LatheGeometry.from_shell_profiles(
        [
            (0.067, 0.014),
            (0.072, 0.034),
            (0.077, 0.078),
            (0.080, 0.148),
            (0.083, 0.226),
            (0.084, 0.250),
            (0.082, 0.258),
        ],
        [
            (0.013, 0.014),
            (0.057, 0.034),
            (0.067, 0.078),
            (0.070, 0.148),
            (0.074, 0.224),
            (0.076, 0.252),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    jar.visual(
        mesh_from_geometry(jar_shell, "blender_jar_shell"),
        material=glass,
        name="jar_shell",
    )
    jar.visual(
        Cylinder(radius=0.074, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=gasket_black,
        name="lock_ring",
    )
    jar.visual(
        Box((0.022, 0.016, 0.026)),
        origin=Origin(xyz=(0.0, 0.084, 0.220)),
        material=gasket_black,
        name="handle_upper_boss",
    )
    jar.visual(
        Box((0.022, 0.016, 0.028)),
        origin=Origin(xyz=(0.0, 0.084, 0.046)),
        material=gasket_black,
        name="handle_lower_boss",
    )
    jar.visual(
        Box((0.014, 0.014, 0.196)),
        origin=Origin(xyz=(0.0, 0.079, 0.136)),
        material=gasket_black,
        name="handle_spine",
    )
    for lug_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radial = 0.073
        tangent = angle + (math.pi / 2.0)
        jar.visual(
            Box((0.016, 0.008, 0.008)),
            origin=Origin(
                xyz=(radial * math.cos(angle), radial * math.sin(angle), 0.006),
                rpy=(0.0, 0.0, tangent),
            ),
            material=gasket_black,
            name=f"bayonet_lug_{lug_index}",
        )
    jar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.258),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.101, 0.220),
                    (0.0, 0.114, 0.206),
                    (0.0, 0.126, 0.160),
                    (0.0, 0.126, 0.108),
                    (0.0, 0.116, 0.060),
                    (0.0, 0.099, 0.046),
                ],
                radius=0.009,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "blender_handle_loop",
        ),
        material=satin_black,
        name="handle_loop",
    )
    handle.visual(
        Box((0.020, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.101, 0.220)),
        material=satin_black,
        name="upper_mount",
    )
    handle.visual(
        Box((0.020, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.101, 0.046)),
        material=satin_black,
        name="lower_mount",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.030, 0.070, 0.190)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.104, 0.135)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_annulus_geometry(0.086, 0.024, 0.008), "blender_lid_flange"),
        material=lid_black,
        name="lid_flange",
    )
    lid.visual(
        mesh_from_geometry(_annulus_geometry(0.074, 0.024, 0.020), "blender_lid_insert"),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=lid_black,
        name="lid_insert",
    )
    lid.visual(
        Box((0.030, 0.008, 0.004)),
        origin=Origin(xyz=(-0.015, 0.014, 0.002)),
        material=lid_black,
        name="hinge_rib_left",
    )
    lid.visual(
        Box((0.030, 0.008, 0.004)),
        origin=Origin(xyz=(-0.015, -0.014, 0.002)),
        material=lid_black,
        name="hinge_rib_right",
    )
    lid.visual(
        Box((0.010, 0.010, 0.010)),
        origin=Origin(xyz=(-0.004, 0.014, 0.005)),
        material=lid_black,
        name="hinge_ear_left",
    )
    lid.visual(
        Box((0.010, 0.010, 0.010)),
        origin=Origin(xyz=(-0.004, -0.014, 0.005)),
        material=lid_black,
        name="hinge_ear_right",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.086, length=0.028),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lid_black,
        name="hinge_barrel",
    )
    cap.visual(
        Box((0.012, 0.016, 0.008)),
        origin=Origin(xyz=(0.004, 0.0, 0.003)),
        material=lid_black,
        name="hinge_arm",
    )
    cap.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(0.018, 0.0, 0.010)),
        material=lid_black,
        name="cap_cover",
    )
    cap.visual(
        Box((0.010, 0.014, 0.006)),
        origin=Origin(xyz=(0.040, 0.0, 0.015)),
        material=lid_black,
        name="cap_grip",
    )
    cap.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.020)),
        mass=0.06,
        origin=Origin(xyz=(0.022, 0.0, 0.010)),
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=blade_steel,
        name="hub",
    )
    blade_assembly.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=blade_steel,
        name="retaining_collar",
    )
    blade_assembly.visual(
        Cylinder(radius=0.004, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=blade_steel,
        name="drive_shaft",
    )
    blade_up_mesh = mesh_from_geometry(_single_blade_geometry(True), "blade_up")
    blade_down_mesh = mesh_from_geometry(_single_blade_geometry(False), "blade_down")
    for blade_name, blade_mesh, angle in (
        ("blade_a", blade_up_mesh, 0.0),
        ("blade_b", blade_down_mesh, math.pi / 2.0),
        ("blade_c", blade_up_mesh, math.pi),
        ("blade_d", blade_down_mesh, 3.0 * math.pi / 2.0),
    ):
        blade_assembly.visual(
            blade_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=blade_steel,
            name=blade_name,
        )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.012),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "base_to_control_knob",
        ArticulationType.FIXED,
        parent=base,
        child=control_knob,
        origin=Origin(xyz=(0.122, 0.0, 0.084)),
    )
    model.articulation(
        "base_to_jar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.2,
            lower=-0.40,
            upper=0.40,
        ),
    )
    model.articulation(
        "jar_to_handle",
        ArticulationType.FIXED,
        parent=jar,
        child=handle,
        origin=Origin(),
    )
    model.articulation(
        "jar_to_lid",
        ArticulationType.FIXED,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.262)),
    )
    model.articulation(
        "lid_to_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cap,
        origin=Origin(xyz=(-0.004, 0.0, 0.004)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "jar_to_blade_assembly",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=22.0,
            lower=0.0,
            upper=math.tau,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    control_knob = object_model.get_part("control_knob")
    jar = object_model.get_part("jar")
    handle = object_model.get_part("handle")
    lid = object_model.get_part("lid")
    cap = object_model.get_part("cap")
    blade_assembly = object_model.get_part("blade_assembly")

    jar_twist = object_model.get_articulation("base_to_jar")
    cap_hinge = object_model.get_articulation("lid_to_cap")
    blade_spin = object_model.get_articulation("jar_to_blade_assembly")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_isolated_part(
        blade_assembly,
        reason="The blade cartridge is intentionally modeled as spinning on a sealed jar-bottom bearing; the tiny visual clearance represents the bearing gap around the revolute spindle.",
    )

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

    def _has_part(name: str) -> bool:
        try:
            object_model.get_part(name)
            return True
        except Exception:
            return False

    for part_name in (
        "base",
        "control_knob",
        "jar",
        "handle",
        "lid",
        "cap",
        "blade_assembly",
    ):
        ctx.check(f"{part_name} present", _has_part(part_name), details=f"missing part: {part_name}")

    ctx.check(
        "jar twist-lock axis is vertical",
        tuple(jar_twist.axis) == (0.0, 0.0, 1.0),
        details=f"axis={jar_twist.axis}",
    )
    ctx.check(
        "blade axis is vertical",
        tuple(blade_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={blade_spin.axis}",
    )
    ctx.check(
        "steam cap hinge opens upward",
        tuple(cap_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={cap_hinge.axis}",
    )

    with ctx.pose({jar_twist: 0.0, cap_hinge: 0.0, blade_spin: 0.0}):
        ctx.expect_gap(
            jar,
            base,
            axis="z",
            positive_elem="lock_ring",
            max_gap=0.001,
            max_penetration=0.0,
            name="jar seats on base coupling",
        )
        ctx.expect_overlap(
            jar,
            base,
            axes="xy",
            elem_a="lock_ring",
            min_overlap=0.12,
            name="jar lock ring stays centered over base",
        )
        ctx.expect_contact(
            control_knob,
            base,
            elem_a="dial_bezel",
            name="control knob is mounted to base shell",
        )
        ctx.expect_contact(
            handle,
            jar,
            elem_a="upper_mount",
            name="handle upper mount contacts jar",
        )
        ctx.expect_contact(
            handle,
            jar,
            elem_a="lower_mount",
            name="handle lower mount contacts jar",
        )
        ctx.expect_gap(
            lid,
            jar,
            axis="z",
            positive_elem="lid_flange",
            max_gap=0.002,
            max_penetration=0.0,
            name="lid flange sits on jar rim",
        )
        ctx.expect_within(
            lid,
            jar,
            axes="xy",
            inner_elem="lid_insert",
            margin=0.010,
            name="lid insert stays within jar mouth footprint",
        )
        ctx.expect_gap(
            cap,
            lid,
            axis="z",
            positive_elem="cap_cover",
            max_gap=0.0015,
            max_penetration=0.0,
            name="vent cap closes flush to lid",
        )
        ctx.expect_within(
            blade_assembly,
            jar,
            axes="xy",
            inner_elem="hub",
            margin=0.010,
            name="blade hub stays within jar footprint",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    def _interval_overlap(a_min, a_max, b_min, b_max):
        return max(0.0, min(a_max, b_max) - max(a_min, b_min))

    closed_cap = None
    opened_cap = None
    closed_grip = None
    opened_grip = None
    closed_insert = None
    opened_insert = None
    with ctx.pose({cap_hinge: 0.0}):
        closed_cap = ctx.part_element_world_aabb(cap, elem="cap_cover")
        closed_grip = ctx.part_element_world_aabb(cap, elem="cap_grip")
        closed_insert = ctx.part_element_world_aabb(lid, elem="lid_insert")
    with ctx.pose({cap_hinge: cap_hinge.motion_limits.upper}):
        opened_cap = ctx.part_element_world_aabb(cap, elem="cap_cover")
        opened_grip = ctx.part_element_world_aabb(cap, elem="cap_grip")
        opened_insert = ctx.part_element_world_aabb(lid, elem="lid_insert")
        ctx.expect_gap(
            cap,
            lid,
            axis="z",
            positive_elem="cap_grip",
            min_gap=0.020,
            name="vent cap grip clears lid when opened",
        )
    ctx.check(
        "vent cap swings upward when opened",
        closed_cap is not None
        and opened_cap is not None
        and opened_cap[1][2] > closed_cap[1][2] + 0.025,
        details=f"closed={closed_cap}, opened={opened_cap}",
    )
    ctx.check(
        "vent cap moves off the steam opening when opened",
        closed_cap is not None
        and opened_cap is not None
        and closed_insert is not None
        and opened_insert is not None
        and _interval_overlap(closed_cap[0][0], closed_cap[1][0], closed_insert[0][0], closed_insert[1][0])
        > _interval_overlap(opened_cap[0][0], opened_cap[1][0], opened_insert[0][0], opened_insert[1][0]) + 0.02,
        details=f"closed_cap={closed_cap}, opened_cap={opened_cap}, lid_insert={closed_insert}",
    )
    ctx.check(
        "vent cap grip rises when opened",
        closed_grip is not None
        and opened_grip is not None
        and opened_grip[0][2] > closed_grip[0][2] + 0.020,
        details=f"closed_grip={closed_grip}, opened_grip={opened_grip}",
    )

    handle_center_rest = None
    handle_center_unlocked = None
    with ctx.pose({jar_twist: 0.0}):
        handle_center_rest = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_loop"))
    with ctx.pose({jar_twist: jar_twist.motion_limits.upper}):
        handle_center_unlocked = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_loop"))
    ctx.check(
        "jar rotates on bayonet twist-lock",
        handle_center_rest is not None
        and handle_center_unlocked is not None
        and (
            abs(handle_center_unlocked[0] - handle_center_rest[0]) > 0.025
            or abs(handle_center_unlocked[1] - handle_center_rest[1]) > 0.025
        ),
        details=f"rest={handle_center_rest}, unlocked={handle_center_unlocked}",
    )

    blade_center_rest = None
    blade_center_spun = None
    with ctx.pose({blade_spin: 0.0}):
        blade_center_rest = _aabb_center(ctx.part_element_world_aabb(blade_assembly, elem="blade_a"))
    with ctx.pose({blade_spin: math.pi / 2.0}):
        blade_center_spun = _aabb_center(ctx.part_element_world_aabb(blade_assembly, elem="blade_a"))
    ctx.check(
        "blade assembly rotates around central shaft",
        blade_center_rest is not None
        and blade_center_spun is not None
        and abs(blade_center_rest[2] - blade_center_spun[2]) < 0.004
        and (
            abs(blade_center_rest[0] - blade_center_spun[0]) > 0.015
            or abs(blade_center_rest[1] - blade_center_spun[1]) > 0.015
        ),
        details=f"rest={blade_center_rest}, spun={blade_center_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
