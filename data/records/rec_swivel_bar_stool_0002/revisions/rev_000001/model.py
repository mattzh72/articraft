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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_swivel_bar_stool", assets=ASSETS)

    powder_gray = model.material("powder_gray", rgba=(0.36, 0.38, 0.40, 1.0))
    dark_paint = model.material("dark_paint", rgba=(0.17, 0.18, 0.20, 1.0))
    molded_urethane = model.material("molded_urethane", rgba=(0.11, 0.11, 0.12, 1.0))
    zinc = model.material("zinc", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    base_plate_mesh = _save_mesh(
        "utility_stool_base_plate.obj",
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.175, 0.000),
                (0.220, 0.003),
                (0.238, 0.008),
                (0.242, 0.016),
                (0.232, 0.025),
                (0.190, 0.033),
                (0.095, 0.038),
                (0.020, 0.040),
                (0.0, 0.040),
            ],
            segments=72,
        ),
    )
    foot_ring_mesh = _save_mesh(
        "utility_stool_foot_ring.obj",
        TorusGeometry(radius=0.185, tube=0.017, radial_segments=18, tubular_segments=60),
    )
    upper_thrust_plate_mesh = _save_mesh(
        "utility_stool_upper_thrust_plate.obj",
        LatheGeometry(
            [
                (0.046, 0.000),
                (0.084, 0.000),
                (0.084, 0.008),
                (0.046, 0.008),
            ],
            segments=64,
        ),
    )
    lower_swivel_plate_mesh = _save_mesh(
        "utility_stool_lower_swivel_plate.obj",
        LatheGeometry(
            [
                (0.050, 0.000),
                (0.089, 0.000),
                (0.089, 0.008),
                (0.050, 0.008),
            ],
            segments=64,
        ),
    )
    swivel_sleeve_mesh = _save_mesh(
        "utility_stool_swivel_sleeve.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.073, 0.008),
                (0.078, 0.014),
                (0.079, 0.031),
                (0.074, 0.039),
            ],
            [
                (0.061, 0.009),
                (0.064, 0.015),
                (0.065, 0.030),
                (0.061, 0.038),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    seat_pan_mesh = _save_mesh(
        "utility_stool_seat_pan.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.000, 0.000),
                (0.055, 0.003),
                (0.110, 0.007),
                (0.150, 0.012),
                (0.168, 0.019),
            ],
            [
                (0.000, 0.0025),
                (0.050, 0.0045),
                (0.104, 0.0085),
                (0.145, 0.0132),
                (0.161, 0.0165),
            ],
            segments=60,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    seat_cushion_mesh = _save_mesh(
        "utility_stool_seat_cushion.obj",
        LatheGeometry(
            [
                (0.000, 0.040),
                (0.045, 0.042),
                (0.100, 0.046),
                (0.150, 0.044),
                (0.173, 0.038),
                (0.181, 0.029),
                (0.179, 0.017),
                (0.169, 0.006),
                (0.150, -0.004),
                (0.085, -0.010),
                (0.020, -0.009),
                (0.000, -0.008),
            ],
            segments=72,
        ),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(base_plate_mesh, material=powder_gray, name="base_plate")
    pedestal_base.visual(
        Cylinder(radius=0.224, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="floor_pad_ring",
    )
    pedestal_base.visual(
        Cylinder(radius=0.090, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=dark_paint,
        name="lower_column_shroud",
    )
    pedestal_base.visual(
        Cylinder(radius=0.055, length=0.525),
        origin=Origin(xyz=(0.0, 0.0, 0.3575)),
        material=powder_gray,
        name="main_column",
    )
    for index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        pedestal_base.visual(
            Box((0.118, 0.016, 0.088)),
            origin=Origin(xyz=(0.062 * math.cos(angle), 0.062 * math.sin(angle), 0.094), rpy=(0.0, -0.82, angle)),
            material=dark_paint,
            name=f"base_gusset_{index}",
        )
    pedestal_base.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=dark_paint,
        name="foot_ring",
    )
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0
        pedestal_base.visual(
            Box((0.145, 0.030, 0.055)),
            origin=Origin(
                xyz=(0.118 * math.cos(angle), 0.118 * math.sin(angle), 0.310),
                rpy=(0.0, 0.0, angle),
            ),
            material=powder_gray,
            name=f"foot_ring_bracket_{index}",
        )
        for bolt_index, bolt_r in enumerate((0.052, 0.158)):
            pedestal_base.visual(
                Cylinder(radius=0.007, length=0.010),
                origin=Origin(
                    xyz=(bolt_r * math.cos(angle), bolt_r * math.sin(angle), 0.342),
                ),
                material=zinc,
                name=f"foot_ring_bolt_{index}_{bolt_index}",
            )
    pedestal_base.visual(
        upper_thrust_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.667)),
        material=powder_gray,
        name="upper_thrust_plate",
    )
    pedestal_base.visual(
        Cylinder(radius=0.073, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.6475)),
        material=dark_paint,
        name="bearing_housing",
    )
    pedestal_base.visual(
        Cylinder(radius=0.043, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.691)),
        material=dark_paint,
        name="upper_spindle",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        pedestal_base.visual(
            Cylinder(radius=0.005, length=0.018),
            origin=Origin(
                xyz=(0.058 * math.cos(angle), 0.058 * math.sin(angle), 0.650),
                rpy=(math.pi / 2.0, 0.0, angle),
            ),
            material=zinc,
            name=f"bearing_set_bolt_{index}",
        )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.49, 0.49, 0.69)),
        mass=31.0,
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
    )

    seat_assembly = model.part("seat_assembly")
    seat_assembly.visual(
        lower_swivel_plate_mesh,
        material=dark_paint,
        name="lower_swivel_plate",
    )
    seat_assembly.visual(swivel_sleeve_mesh, material=powder_gray, name="swivel_sleeve")
    seat_assembly.visual(
        Cylinder(radius=0.122, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=powder_gray,
        name="seat_support_plate",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        seat_assembly.visual(
            Box((0.046, 0.018, 0.030)),
            origin=Origin(
                xyz=(0.088 * math.cos(angle), 0.088 * math.sin(angle), 0.030),
                rpy=(0.0, -0.60, angle),
            ),
            material=dark_paint,
            name=f"support_gusset_{index}",
        )
    seat_assembly.visual(
        seat_pan_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=dark_paint,
        name="seat_pan",
    )
    seat_assembly.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=molded_urethane,
        name="seat_cushion",
    )
    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        seat_assembly.visual(
            Cylinder(radius=0.0065, length=0.012),
            origin=Origin(
                xyz=(0.094 * math.cos(angle), 0.094 * math.sin(angle), 0.033),
            ),
            material=zinc,
            name=f"seat_mount_bolt_{index}",
        )
    seat_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.182, length=0.120),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=seat_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=3.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal_base = object_model.get_part("pedestal_base")
    seat_assembly = object_model.get_part("seat_assembly")
    seat_swivel = object_model.get_articulation("seat_swivel")

    upper_spindle = pedestal_base.get_visual("upper_spindle")
    upper_thrust_plate = pedestal_base.get_visual("upper_thrust_plate")
    foot_ring = pedestal_base.get_visual("foot_ring")
    base_plate = pedestal_base.get_visual("base_plate")

    lower_swivel_plate = seat_assembly.get_visual("lower_swivel_plate")
    swivel_sleeve = seat_assembly.get_visual("swivel_sleeve")
    seat_support_plate = seat_assembly.get_visual("seat_support_plate")
    support_gusset_0 = seat_assembly.get_visual("support_gusset_0")
    seat_pan = seat_assembly.get_visual("seat_pan")
    seat_cushion = seat_assembly.get_visual("seat_cushion")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        seat_assembly,
        pedestal_base,
        elem_a=lower_swivel_plate,
        elem_b=upper_thrust_plate,
        name="swivel_interface_contact",
    )
    ctx.expect_gap(
        seat_assembly,
        pedestal_base,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=lower_swivel_plate,
        negative_elem=upper_thrust_plate,
        name="swivel_interface_seated",
    )
    ctx.expect_overlap(
        seat_assembly,
        pedestal_base,
        axes="xy",
        min_overlap=0.12,
        elem_a=lower_swivel_plate,
        elem_b=upper_thrust_plate,
        name="swivel_plate_overlap",
    )
    ctx.expect_overlap(
        seat_assembly,
        pedestal_base,
        axes="xy",
        min_overlap=0.08,
        elem_a=swivel_sleeve,
        elem_b=upper_spindle,
        name="sleeve_stays_over_spindle",
    )
    ctx.expect_origin_distance(
        seat_assembly,
        pedestal_base,
        axes="xy",
        max_dist=0.002,
        name="seat_centered_over_column",
    )
    ctx.expect_gap(
        seat_assembly,
        pedestal_base,
        axis="z",
        min_gap=0.24,
        positive_elem=seat_cushion,
        negative_elem=foot_ring,
        name="seat_clear_of_foot_ring",
    )

    base_aabb = ctx.part_element_world_aabb(pedestal_base, elem=base_plate)
    foot_ring_aabb = ctx.part_element_world_aabb(pedestal_base, elem=foot_ring)
    support_plate_aabb = ctx.part_element_world_aabb(seat_assembly, elem=seat_support_plate)
    seat_pan_aabb = ctx.part_element_world_aabb(seat_assembly, elem=seat_pan)
    seat_cushion_aabb = ctx.part_element_world_aabb(seat_assembly, elem=seat_cushion)
    gusset_rest_aabb = ctx.part_element_world_aabb(seat_assembly, elem=support_gusset_0)

    if base_aabb is None or foot_ring_aabb is None or support_plate_aabb is None or seat_pan_aabb is None or seat_cushion_aabb is None:
        ctx.fail("critical_visual_bounds_present", "Expected key stool visuals to have measurable world AABBs.")
        return ctx.report()

    base_diameter = base_aabb[1][0] - base_aabb[0][0]
    foot_ring_height = foot_ring_aabb[0][2]
    support_diameter = support_plate_aabb[1][0] - support_plate_aabb[0][0]
    pan_top = seat_pan_aabb[1][2]
    cushion_diameter = seat_cushion_aabb[1][0] - seat_cushion_aabb[0][0]
    seat_top = seat_cushion_aabb[1][2]

    ctx.check(
        "stable_pedestal_diameter",
        0.44 <= base_diameter <= 0.50,
        f"Expected a wide pedestal base, got diameter {base_diameter:.3f} m.",
    )
    ctx.check(
        "foot_ring_height_range",
        0.27 <= foot_ring_height <= 0.33,
        f"Expected foot ring near bar-stool height zone, got lower surface at {foot_ring_height:.3f} m.",
    )
    ctx.check(
        "support_plate_is_clearly_smaller_than_seat",
        0.22 <= support_diameter <= 0.26 and cushion_diameter > support_diameter + 0.08,
        (
            f"Expected a compact circular support under a larger seat; "
            f"support diameter={support_diameter:.3f} m, seat diameter={cushion_diameter:.3f} m."
        ),
    )
    ctx.check(
        "seat_pan_below_cushion",
        0.010 <= seat_top - pan_top <= 0.060,
        f"Expected visible stacked pan/cushion construction, got top offset {seat_top - pan_top:.3f} m.",
    )
    ctx.check(
        "bar_stool_seat_height",
        0.76 <= seat_top <= 0.82,
        f"Expected practical bar stool seat height, got seat top at {seat_top:.3f} m.",
    )

    if gusset_rest_aabb is None:
        ctx.fail("support_gusset_bounds_present", "Expected the named support gusset to be measurable for articulation checks.")
        return ctx.report()

    rest_center = (
        (gusset_rest_aabb[0][0] + gusset_rest_aabb[1][0]) * 0.5,
        (gusset_rest_aabb[0][1] + gusset_rest_aabb[1][1]) * 0.5,
    )
    with ctx.pose({seat_swivel: math.radians(50.0)}):
        gusset_swiveled_aabb = ctx.part_element_world_aabb(seat_assembly, elem=support_gusset_0)
        if gusset_swiveled_aabb is None:
            ctx.fail("support_gusset_bounds_present_swiveled", "Expected support gusset bounds in posed swivel state.")
        else:
            swiveled_center = (
                (gusset_swiveled_aabb[0][0] + gusset_swiveled_aabb[1][0]) * 0.5,
                (gusset_swiveled_aabb[0][1] + gusset_swiveled_aabb[1][1]) * 0.5,
            )
            ctx.check(
                "seat_swivel_rotates_visible_structure",
                abs(swiveled_center[0] - rest_center[0]) > 0.02 and abs(swiveled_center[1] - rest_center[1]) > 0.02,
                (
                    "Expected a clearly visible support gusset to move in XY when the seat swivels; "
                    f"rest={rest_center}, swiveled={swiveled_center}."
                ),
            )
        ctx.expect_contact(
            seat_assembly,
            pedestal_base,
            elem_a=lower_swivel_plate,
            elem_b=upper_thrust_plate,
            name="swivel_contact_at_rotated_pose",
        )
        ctx.expect_overlap(
            seat_assembly,
            pedestal_base,
            axes="xy",
            min_overlap=0.08,
            elem_a=swivel_sleeve,
            elem_b=upper_spindle,
            name="sleeve_remains_centered_at_rotated_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
