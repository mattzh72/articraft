from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")

ASSETS = AssetContext.from_script(__file__)

BASE_HEIGHT = 0.240
JAR_HEIGHT = 0.285
LID_SEAT_Z = JAR_HEIGHT - 0.014


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _circle_profile(
    radius: float,
    *,
    segments: int = 32,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((index * math.tau) / segments),
            cy + radius * math.sin((index * math.tau) / segments),
        )
        for index in range(segments)
    ]


def _rect_profile(
    width: float,
    height: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _base_section(
    width: float,
    depth: float,
    radius: float,
    *,
    z: float,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_shift, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)
    ]


def _build_base_shell_mesh():
    return section_loft(
        [
            _base_section(0.200, 0.182, 0.028, z=0.010, y_shift=0.000),
            _base_section(0.188, 0.170, 0.026, z=0.090, y_shift=0.004),
            _base_section(0.156, 0.144, 0.022, z=0.184, y_shift=0.012),
            _base_section(0.132, 0.124, 0.018, z=0.228, y_shift=0.018),
        ]
    )


def _build_jar_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.061, 0.014),
            (0.056, 0.018),
            (0.063, 0.030),
            (0.068, 0.210),
            (0.071, 0.270),
            (0.073, 0.285),
        ],
        [
            (0.052, 0.018),
            (0.052, 0.020),
            (0.059, 0.030),
            (0.064, 0.210),
            (0.067, 0.279),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )


def _build_handle_mesh():
    return tube_from_spline_points(
        [
            (0.078, 0.000, 0.060),
            (0.112, 0.000, 0.102),
            (0.122, 0.000, 0.154),
            (0.112, 0.000, 0.212),
            (0.078, 0.000, 0.225),
        ],
        radius=0.0085,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def _build_blade_mesh():
    blade_outline = [
        (0.000, -0.0038),
        (0.014, -0.0052),
        (0.036, -0.0038),
        (0.050, -0.0008),
        (0.052, 0.0000),
        (0.050, 0.0008),
        (0.036, 0.0038),
        (0.014, 0.0052),
        (0.000, 0.0038),
    ]
    return ExtrudeGeometry(blade_outline, 0.0016, center=True, cap=True, closed=True)


def _build_ring_disc_mesh(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    segments: int = 40,
):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=max(20, segments // 2))],
        height,
        cap=True,
        center=False,
        closed=True,
    )


def _build_lid_plate_mesh():
    hole_profiles = [
        _circle_profile(0.023, segments=28),
        _rect_profile(0.020, 0.006, center=(0.000, 0.046)),
        _rect_profile(0.020, 0.006, center=(0.000, -0.046)),
        _rect_profile(0.006, 0.020, center=(0.046, 0.000)),
        _rect_profile(0.006, 0.020, center=(-0.046, 0.000)),
    ]
    return ExtrudeWithHolesGeometry(
        _circle_profile(0.073, segments=40),
        hole_profiles,
        0.004,
        cap=True,
        center=False,
        closed=True,
    )


def _build_lid_skirt_mesh():
    return ExtrudeWithHolesGeometry(
        _circle_profile(0.072, segments=40),
        [_circle_profile(0.056, segments=28)],
        0.006,
        cap=True,
        center=False,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_performance_blender", assets=ASSETS)

    body_charcoal = model.material("body_charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    body_black = model.material("body_black", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.11, 0.12, 1.0))
    jar_glass = model.material("jar_glass", rgba=(0.86, 0.92, 0.97, 0.24))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.61, 0.64, 1.0))
    accent = model.material("accent", rgba=(0.80, 0.20, 0.16, 1.0))

    base = model.part("base")
    base_shell = base.visual(
        _save_mesh("blender_base_shell.obj", _build_base_shell_mesh()),
        material=body_charcoal,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(0.000, 0.018, 0.234)),
        material=body_black,
        name="base_collar",
    )
    base.visual(
        Box((0.078, 0.008, 0.084)),
        origin=Origin(xyz=(0.000, -0.083, 0.118)),
        material=body_black,
        name="front_panel",
    )
    for index, (x_pos, y_pos) in enumerate(((-0.070, -0.055), (0.070, -0.055), (-0.070, 0.055), (0.070, 0.055))):
        base.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, 0.005)),
            material=rubber_black,
            name=f"foot_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.200, 0.182, BASE_HEIGHT)),
        mass=4.8,
        origin=Origin(xyz=(0.000, 0.006, BASE_HEIGHT * 0.5)),
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.000, -0.002, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_black,
        name="dial_shaft",
    )
    speed_dial.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.000, -0.012, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="dial_body",
    )
    speed_dial.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.000, -0.022, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_black,
        name="dial_center",
    )
    speed_dial.visual(
        Box((0.004, 0.002, 0.016)),
        origin=Origin(xyz=(0.000, -0.022, 0.014)),
        material=accent,
        name="dial_indicator",
    )
    speed_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.020),
        mass=0.12,
        origin=Origin(xyz=(0.000, -0.012, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    jar = model.part("jar")
    jar.visual(
        _save_mesh("blender_jar_shell.obj", _build_jar_shell_mesh()),
        material=jar_glass,
        name="jar_shell",
    )
    jar.visual(
        _save_mesh("blender_jar_base_ring.obj", _build_ring_disc_mesh(0.061, 0.0205, 0.014, segments=40)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=body_black,
        name="jar_base_ring",
    )
    jar.visual(
        _save_mesh("blender_jar_floor.obj", _build_ring_disc_mesh(0.050, 0.0210, 0.004, segments=36)),
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
        material=body_black,
        name="jar_floor",
    )
    jar.visual(
        Box((0.028, 0.020, 0.034)),
        origin=Origin(xyz=(0.070, 0.000, 0.060)),
        material=body_black,
        name="handle_anchor_lower",
    )
    jar.visual(
        Box((0.028, 0.020, 0.036)),
        origin=Origin(xyz=(0.072, 0.000, 0.225)),
        material=body_black,
        name="handle_anchor_upper",
    )
    jar.visual(
        _save_mesh("blender_jar_handle.obj", _build_handle_mesh()),
        material=body_black,
        name="handle_loop",
    )
    jar.visual(
        Box((0.026, 0.012, 0.008)),
        origin=Origin(xyz=(0.000, -0.071, 0.280)),
        material=body_black,
        name="spout_lip",
    )
    jar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.078, length=JAR_HEIGHT),
        mass=1.45,
        origin=Origin(xyz=(0.000, 0.000, JAR_HEIGHT * 0.5)),
    )

    blades = model.part("blade_assembly")
    blades.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=body_black,
        name="blade_coupler",
    )
    blades.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=satin_steel,
        name="blade_hub",
    )
    blades.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.000, 0.000, -0.001)),
        material=satin_steel,
        name="blade_spindle",
    )
    blades.visual(
        Cylinder(radius=0.024, length=0.002),
        origin=Origin(xyz=(0.000, 0.000, -0.012)),
        material=satin_steel,
        name="blade_mount_flange",
    )
    blade_mesh = _save_mesh("blender_blade.obj", _build_blade_mesh())
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        blades.visual(
            blade_mesh,
            origin=Origin(xyz=(0.000, 0.000, 0.010), rpy=(0.0, 0.30, angle)),
            material=steel,
            name=f"blade_lower_{index}",
        )
    for index, angle in enumerate((math.pi / 3.0, math.pi, 5.0 * math.pi / 3.0)):
        blades.visual(
            blade_mesh,
            origin=Origin(xyz=(0.000, 0.000, 0.015), rpy=(0.0, -0.26, angle)),
            material=steel,
            name=f"blade_upper_{index}",
        )
    blades.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.028),
        mass=0.22,
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
    )

    lid = model.part("lid")
    lid.visual(
        _save_mesh("blender_lid_plate.obj", _build_lid_plate_mesh()),
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
        material=rubber_black,
        name="lid_plate",
    )
    lid.visual(
        _save_mesh("blender_lid_skirt.obj", _build_lid_skirt_mesh()),
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
        material=body_black,
        name="lid_skirt",
    )
    lid.visual(
        Cylinder(radius=0.0036, length=0.022),
        origin=Origin(xyz=(0.000, -0.023, 0.020), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=body_black,
        name="cap_hinge_socket",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.074, length=0.020),
        mass=0.24,
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
    )

    center_cap = model.part("center_cap")
    center_cap.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(xyz=(0.000, 0.023, 0.005)),
        material=body_black,
        name="cap_disc",
    )
    center_cap.visual(
        Cylinder(radius=0.0185, length=0.012),
        origin=Origin(xyz=(0.000, 0.023, -0.003)),
        material=rubber_black,
        name="cap_plug",
    )
    center_cap.visual(
        Cylinder(radius=0.003, length=0.026),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=satin_steel,
        name="cap_hinge_barrel",
    )
    center_cap.visual(
        Box((0.012, 0.010, 0.004)),
        origin=Origin(xyz=(0.000, 0.045, 0.008)),
        material=accent,
        name="cap_tab",
    )
    center_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.020),
        mass=0.05,
        origin=Origin(xyz=(0.000, 0.022, 0.002)),
    )

    model.articulation(
        "base_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(0.000, -0.087, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.5, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "base_to_jar",
        ArticulationType.FIXED,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.000, 0.018, BASE_HEIGHT)),
    )
    model.articulation(
        "jar_to_blade_assembly",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=blades,
        origin=Origin(xyz=(0.000, 0.000, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=45.0, lower=0.0, upper=math.tau),
    )
    model.articulation(
        "jar_to_lid",
        ArticulationType.FIXED,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.000, 0.000, LID_SEAT_Z)),
    )
    model.articulation(
        "lid_to_center_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=center_cap,
        origin=Origin(xyz=(0.000, -0.023, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=0.0, upper=1.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)

    base = object_model.get_part("base")
    speed_dial = object_model.get_part("speed_dial")
    jar = object_model.get_part("jar")
    blades = object_model.get_part("blade_assembly")
    lid = object_model.get_part("lid")
    center_cap = object_model.get_part("center_cap")

    dial_turn = object_model.get_articulation("base_to_speed_dial")
    blade_spin = object_model.get_articulation("jar_to_blade_assembly")
    cap_hinge = object_model.get_articulation("lid_to_center_cap")

    def aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        center_cap,
        lid,
        reason="Flip-cap hinge barrel is seated in the lid hinge socket.",
        elem_a="cap_hinge_barrel",
        elem_b="cap_hinge_socket",
    )
    ctx.fail_if_isolated_parts(max_pose_samples=24)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.check("blade_spin_axis", blade_spin.axis == (0.0, 0.0, 1.0), f"axis={blade_spin.axis}")
    ctx.check("cap_hinge_axis", cap_hinge.axis == (1.0, 0.0, 0.0), f"axis={cap_hinge.axis}")
    ctx.check("dial_axis", dial_turn.axis == (0.0, 1.0, 0.0), f"axis={dial_turn.axis}")

    ctx.expect_origin_gap(jar, base, axis="z", min_gap=0.235, max_gap=0.245)
    ctx.expect_origin_gap(lid, jar, axis="z", min_gap=0.270, max_gap=0.280)

    ctx.expect_contact(
        jar,
        base,
        contact_tol=0.0015,
        elem_a="jar_base_ring",
        elem_b="base_collar",
    )
    ctx.expect_overlap(
        jar,
        base,
        axes="xy",
        min_overlap=0.100,
        elem_a="jar_base_ring",
        elem_b="base_collar",
    )
    ctx.expect_contact(
        lid,
        jar,
        contact_tol=0.0015,
        elem_a="lid_skirt",
        elem_b="jar_shell",
    )
    ctx.expect_overlap(
        lid,
        jar,
        axes="xy",
        min_overlap=0.120,
        elem_a="lid_plate",
        elem_b="jar_shell",
    )
    ctx.expect_contact(
        speed_dial,
        base,
        contact_tol=1e-6,
        elem_a="dial_shaft",
        elem_b="front_panel",
    )
    ctx.expect_within(blades, jar, axes="xy", inner_elem="blade_lower_0", outer_elem="jar_shell")
    ctx.expect_within(blades, jar, axes="xy", inner_elem="blade_upper_0", outer_elem="jar_shell")
    ctx.expect_contact(
        blades,
        jar,
        contact_tol=1e-6,
        elem_a="blade_mount_flange",
        elem_b="jar_base_ring",
    )
    ctx.expect_gap(
        blades,
        jar,
        axis="z",
        positive_elem="blade_hub",
        negative_elem="jar_floor",
        min_gap=0.0015,
        max_gap=0.0100,
    )
    ctx.expect_gap(
        center_cap,
        lid,
        axis="z",
        positive_elem="cap_disc",
        negative_elem="lid_plate",
        min_gap=0.0010,
        max_gap=0.0080,
    )
    ctx.expect_overlap(
        center_cap,
        lid,
        axes="xy",
        min_overlap=0.045,
        elem_a="cap_disc",
        elem_b="lid_plate",
    )

    base_aabb = ctx.part_world_aabb(base)
    cap_aabb = ctx.part_world_aabb(center_cap)
    jar_shell_aabb = ctx.part_element_world_aabb(jar, elem="jar_shell")
    if base_aabb is None or cap_aabb is None:
        ctx.fail("overall_height_available", "Could not compute base or cap AABB.")
    else:
        total_height = cap_aabb[1][2] - base_aabb[0][2]
        ctx.check(
            "overall_height_realistic",
            0.50 <= total_height <= 0.56,
            f"total_height={total_height:.3f} m",
        )
    if jar_shell_aabb is None:
        ctx.fail("jar_shell_aabb_available", "Could not compute jar shell AABB.")
    else:
        jar_diameter = max(
            jar_shell_aabb[1][0] - jar_shell_aabb[0][0],
            jar_shell_aabb[1][1] - jar_shell_aabb[0][1],
        )
        ctx.check(
            "jar_diameter_realistic",
            0.13 <= jar_diameter <= 0.16,
            f"jar_diameter={jar_diameter:.3f} m",
        )

    blade_rest = ctx.part_element_world_aabb(blades, elem="blade_lower_0")
    with ctx.pose({blade_spin: math.pi / 2.0}):
        blade_turn = ctx.part_element_world_aabb(blades, elem="blade_lower_0")
        ctx.expect_within(
            blades,
            jar,
            axes="xy",
            inner_elem="blade_lower_0",
            outer_elem="jar_shell",
            name="blade_lower_rotated_within_jar",
        )
        ctx.expect_within(
            blades,
            jar,
            axes="xy",
            inner_elem="blade_upper_0",
            outer_elem="jar_shell",
            name="blade_upper_rotated_within_jar",
        )
    if blade_rest is None or blade_turn is None:
        ctx.fail("blade_pose_measurements", "Could not compare blade AABBs across poses.")
    else:
        rest_center = aabb_center(blade_rest)
        turn_center = aabb_center(blade_turn)
        ctx.check(
            "blade_joint_moves_geometry",
            abs(turn_center[0] - rest_center[0]) > 0.012 and abs(turn_center[1] - rest_center[1]) > 0.012,
            f"rest={rest_center}, turned={turn_center}",
        )

    dial_rest = ctx.part_element_world_aabb(speed_dial, elem="dial_indicator")
    with ctx.pose({dial_turn: 1.7}):
        dial_turn_aabb = ctx.part_element_world_aabb(speed_dial, elem="dial_indicator")
        ctx.expect_contact(
            speed_dial,
            base,
            contact_tol=1e-6,
            elem_a="dial_shaft",
            elem_b="front_panel",
            name="dial_rotated_stays_mounted",
        )
    if dial_rest is None or dial_turn_aabb is None:
        ctx.fail("dial_pose_measurements", "Could not compare speed dial indicator across poses.")
    else:
        rest_center = aabb_center(dial_rest)
        turn_center = aabb_center(dial_turn_aabb)
        ctx.check(
            "dial_rotates",
            abs(turn_center[0] - rest_center[0]) > 0.005 or abs(turn_center[1] - rest_center[1]) > 0.005,
            f"rest={rest_center}, turned={turn_center}",
        )

    cap_rest = ctx.part_element_world_aabb(center_cap, elem="cap_tab")
    with ctx.pose({cap_hinge: 1.05}):
        cap_open = ctx.part_element_world_aabb(center_cap, elem="cap_tab")
        ctx.expect_gap(
            center_cap,
            lid,
            axis="z",
            positive_elem="cap_tab",
            negative_elem="lid_plate",
            min_gap=0.020,
            name="cap_open_clearance",
        )
    if cap_rest is None or cap_open is None:
        ctx.fail("cap_pose_measurements", "Could not compare cap tab across poses.")
    else:
        rest_center = aabb_center(cap_rest)
        open_center = aabb_center(cap_open)
        ctx.check(
            "cap_flips_open",
            open_center[2] > rest_center[2] + 0.020,
            f"rest={rest_center}, open={open_center}",
        )

    for articulation in (dial_turn, blade_spin, cap_hinge):
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
