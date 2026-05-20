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
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tracking_dish", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.69, 0.70, 0.72, 1.0))
    paint_white = model.material("paint_white", rgba=(0.89, 0.91, 0.93, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.09, 0.10, 1.0))

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    reflector_shell = _mesh(
        "reflector_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.00, -0.38),
                (0.08, -0.377),
                (0.20, -0.362),
                (0.35, -0.320),
                (0.50, -0.245),
                (0.62, -0.145),
                (0.72, 0.000),
            ],
            [
                (0.00, -0.355),
                (0.07, -0.352),
                (0.19, -0.338),
                (0.33, -0.299),
                (0.47, -0.228),
                (0.59, -0.136),
                (0.69, -0.016),
            ],
            segments=72,
            start_cap="flat",
            end_cap="round",
            lip_samples=8,
        ),
    )
    rim_ring = _mesh(
        "rim_ring.obj",
        TorusGeometry(radius=0.705, tube=0.022, radial_segments=18, tubular_segments=72),
    )
    rear_outer_ring = _mesh(
        "rear_outer_ring.obj",
        TorusGeometry(radius=0.525, tube=0.018, radial_segments=16, tubular_segments=64),
    )
    yoke_arm = _mesh(
        "yoke_arm.obj",
        sweep_profile_along_spline(
            [
                (0.00, -0.27, 0.06),
                (0.02, -0.31, 0.46),
                (0.08, -0.34, 0.82),
                (0.18, -0.24, 1.03),
                (0.28, -0.10, 1.18),
            ],
            profile=rounded_rect_profile(0.09, 0.14, radius=0.022, corner_segments=8),
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    yoke_brace = _mesh(
        "yoke_brace.obj",
        tube_from_spline_points(
            [
                (0.10, -0.18, 0.08),
                (0.14, -0.23, 0.54),
                (0.24, -0.14, 0.96),
            ],
            radius=0.030,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    yoke_backstay = _mesh(
        "yoke_backstay.obj",
        tube_from_spline_points(
            [
                (-0.04, -0.20, 0.07),
                (0.00, -0.28, 0.40),
                (0.18, -0.24, 0.92),
            ],
            radius=0.028,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    rear_rib = _mesh(
        "rear_rib.obj",
        tube_from_spline_points(
            [
                (0.06, 0.00, 0.10),
                (0.12, 0.00, 0.18),
                (0.20, 0.00, 0.34),
                (0.26, 0.00, 0.52),
            ],
            radius=0.016,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    rear_diagonal = _mesh(
        "rear_diagonal.obj",
        tube_from_spline_points(
            [
                (0.08, 0.03, 0.10),
                (0.15, 0.00, 0.24),
                (0.22, -0.07, 0.44),
            ],
            radius=0.012,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    feed_strut = _mesh(
        "feed_strut.obj",
        tube_from_spline_points(
            [
                (0.48, 0.00, 0.54),
                (0.60, 0.00, 0.22),
                (0.71, 0.00, 0.04),
            ],
            radius=0.012,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    feed_horn_shell = _mesh(
        "feed_horn_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.032, -0.055),
                (0.036, -0.035),
                (0.046, -0.008),
                (0.060, 0.026),
                (0.076, 0.060),
            ],
            [
                (0.018, -0.050),
                (0.020, -0.032),
                (0.028, -0.007),
                (0.040, 0.025),
                (0.056, 0.056),
            ],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    base = model.part("base")
    base.visual(
        Box((0.90, 0.90, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=concrete,
        name="footing",
    )
    base.visual(
        Cylinder(radius=0.18, length=0.48),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=machine_gray,
        name="column",
    )
    base.visual(
        Cylinder(radius=0.22, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=dark_steel,
        name="column_cap",
    )
    base.visual(
        Box((0.24, 0.34, 0.28)),
        origin=Origin(xyz=(-0.18, 0.0, 0.22)),
        material=machine_gray,
        name="drive_cabinet",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.90, 0.90, 0.68)),
        mass=380.0,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
    )

    azimuth_stage = model.part("azimuth_stage")
    azimuth_stage.visual(
        Cylinder(radius=0.23, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="turntable_base",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.34, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=machine_gray,
        name="turntable_deck",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.12, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=dark_steel,
        name="pivot_housing",
    )
    azimuth_stage.visual(
        Box((0.20, 0.24, 0.18)),
        origin=Origin(xyz=(-0.10, 0.0, 0.19)),
        material=machine_gray,
        name="azimuth_motor",
    )
    azimuth_stage.visual(yoke_arm, material=paint_white, name="side_yoke_arm")
    azimuth_stage.visual(yoke_brace, material=dark_steel, name="side_yoke_brace")
    azimuth_stage.visual(yoke_backstay, material=dark_steel, name="side_yoke_backstay")
    azimuth_stage.visual(
        Cylinder(radius=0.09, length=0.06),
        origin=Origin(xyz=(0.28, -0.03, 1.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="yoke_bearing_plate",
    )
    azimuth_stage.inertial = Inertial.from_geometry(
        Box((0.75, 0.80, 1.30)),
        mass=120.0,
        origin=Origin(xyz=(0.08, 0.16, 0.62)),
    )

    dish_assembly = model.part("dish_assembly")
    dish_assembly.visual(
        Cylinder(radius=0.11, length=0.08),
        origin=Origin(xyz=(0.0, 0.04, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="side_hub_disk",
    )
    dish_assembly.visual(
        Box((0.18, 0.08, 0.26)),
        origin=Origin(xyz=(0.02, 0.05, 0.0)),
        material=machine_gray,
        name="hub_block",
    )
    dish_assembly.visual(
        Cylinder(radius=0.075, length=0.18),
        origin=Origin(xyz=(0.08, 0.08, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_barrel",
    )
    dish_assembly.visual(
        Cylinder(radius=0.020, length=0.62),
        origin=Origin(xyz=(0.46, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="feed_support_boom",
    )
    dish_assembly.visual(
        reflector_shell,
        origin=Origin(xyz=(0.48, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=paint_white,
        name="main_reflector",
    )
    dish_assembly.visual(
        rim_ring,
        origin=Origin(xyz=(0.47, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="rim_ring",
    )
    dish_assembly.visual(
        rear_outer_ring,
        origin=Origin(xyz=(0.26, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_outer_ring",
    )
    for index in range(8):
        angle = index * (2.0 * math.pi / 8.0)
        dish_assembly.visual(
            rear_rib,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=dark_steel,
            name=f"rear_rib_{index:02d}",
        )
    for index in range(8):
        angle = index * (2.0 * math.pi / 8.0)
        dish_assembly.visual(
            rear_diagonal,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=machine_gray,
            name=f"rear_diagonal_{index:02d}",
        )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        dish_assembly.visual(
            feed_strut,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=aluminum,
            name=f"feed_strut_{index:02d}",
        )
    dish_assembly.visual(
        Box((0.14, 0.14, 0.14)),
        origin=Origin(xyz=(0.77, 0.0, 0.0)),
        material=matte_black,
        name="receiver_box",
    )
    dish_assembly.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(0.88, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="feed_adapter",
    )
    dish_assembly.visual(
        feed_horn_shell,
        origin=Origin(xyz=(0.973, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="feed_horn",
    )
    dish_assembly.inertial = Inertial.from_geometry(
        Box((1.08, 1.48, 1.48)),
        mass=72.0,
        origin=Origin(xyz=(0.44, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=azimuth_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.8),
    )
    model.articulation(
        "elevation_rotation",
        ArticulationType.REVOLUTE,
        parent=azimuth_stage,
        child=dish_assembly,
        origin=Origin(xyz=(0.28, 0.0, 1.18)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=95.0,
            velocity=1.1,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    azimuth_stage = object_model.get_part("azimuth_stage")
    dish_assembly = object_model.get_part("dish_assembly")
    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    elevation_rotation = object_model.get_articulation("elevation_rotation")

    column_cap = base.get_visual("column_cap")
    turntable_base = azimuth_stage.get_visual("turntable_base")
    turntable_deck = azimuth_stage.get_visual("turntable_deck")
    yoke_bearing_plate = azimuth_stage.get_visual("yoke_bearing_plate")
    main_reflector = dish_assembly.get_visual("main_reflector")
    side_hub_disk = dish_assembly.get_visual("side_hub_disk")
    feed_horn = dish_assembly.get_visual("feed_horn")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(max_pose_samples=8)
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=16)

    azimuth_limits = getattr(azimuth_rotation, "motion_limits", None)
    elevation_limits = getattr(elevation_rotation, "motion_limits", None)
    ctx.check(
        "azimuth_is_continuous_vertical_axis",
        getattr(azimuth_rotation, "articulation_type", None) == ArticulationType.CONTINUOUS
        and tuple(round(float(v), 4) for v in getattr(azimuth_rotation, "axis", ())) == (0.0, 0.0, 1.0)
        and getattr(azimuth_limits, "lower", None) is None
        and getattr(azimuth_limits, "upper", None) is None,
        "Azimuth stage should use an unrestricted vertical rotation axis.",
    )
    ctx.check(
        "elevation_range_matches_brief",
        getattr(elevation_rotation, "articulation_type", None) == ArticulationType.REVOLUTE
        and tuple(round(float(v), 4) for v in getattr(elevation_rotation, "axis", ())) == (0.0, -1.0, 0.0)
        and math.isclose(getattr(elevation_limits, "lower", -1.0), 0.0, abs_tol=1e-6)
        and math.isclose(getattr(elevation_limits, "upper", -1.0), math.radians(70.0), abs_tol=1e-6),
        "Elevation stage should tilt from level to about seventy degrees around a transverse axis.",
    )

    ctx.expect_overlap(
        azimuth_stage,
        base,
        axes="xy",
        min_overlap=0.22,
        elem_a=turntable_base,
        elem_b=column_cap,
        name="turntable_overlaps_column_cap",
    )
    ctx.expect_gap(
        azimuth_stage,
        base,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=turntable_base,
        negative_elem=column_cap,
        name="turntable_sits_on_column_cap",
    )
    ctx.expect_contact(
        dish_assembly,
        azimuth_stage,
        elem_a=side_hub_disk,
        elem_b=yoke_bearing_plate,
        name="dish_trunnion_contacts_side_yoke",
    )
    ctx.expect_overlap(
        dish_assembly,
        azimuth_stage,
        axes="xz",
        min_overlap=0.15,
        elem_a=side_hub_disk,
        elem_b=yoke_bearing_plate,
        name="dish_trunnion_overlaps_bearing_footprint",
    )
    ctx.expect_gap(
        dish_assembly,
        azimuth_stage,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=side_hub_disk,
        negative_elem=yoke_bearing_plate,
        name="dish_trunnion_seats_against_bearing_plate",
    )

    turntable_deck_aabb = ctx.part_element_world_aabb(azimuth_stage, elem=turntable_deck)
    rest_feed_aabb = ctx.part_element_world_aabb(dish_assembly, elem=feed_horn)
    rest_feed_center = None
    mid_feed_aabb = None
    mid_feed_center = None
    high_feed_aabb = None
    high_feed_center = None

    with ctx.pose({azimuth_rotation: math.pi}):
        ctx.expect_gap(
            azimuth_stage,
            base,
            axis="z",
            max_gap=0.0015,
            max_penetration=0.0,
            positive_elem=turntable_base,
            negative_elem=column_cap,
            name="azimuth_stage_remains_seated_through_full_sweep",
        )

    with ctx.pose({elevation_rotation: math.radians(35.0)}):
        ctx.expect_contact(
            dish_assembly,
            azimuth_stage,
            elem_a=side_hub_disk,
            elem_b=yoke_bearing_plate,
            name="dish_trunnion_keeps_contact_mid_elevation",
        )
        mid_feed_aabb = ctx.part_element_world_aabb(dish_assembly, elem=feed_horn)
        if mid_feed_aabb is not None:
            mid_feed_center = tuple((mid_feed_aabb[0][i] + mid_feed_aabb[1][i]) * 0.5 for i in range(3))

    with ctx.pose({elevation_rotation: math.radians(70.0)}):
        high_feed_aabb = ctx.part_element_world_aabb(dish_assembly, elem=feed_horn)
        if high_feed_aabb is not None:
            high_feed_center = tuple((high_feed_aabb[0][i] + high_feed_aabb[1][i]) * 0.5 for i in range(3))

    if rest_feed_aabb is None or turntable_deck_aabb is None:
        ctx.fail("feed_horn_clearance_measurable", "Could not measure feed horn clearance above the turntable.")
    else:
        rest_feed_center = tuple((rest_feed_aabb[0][i] + rest_feed_aabb[1][i]) * 0.5 for i in range(3))
        rest_gap = rest_feed_aabb[0][2] - turntable_deck_aabb[1][2]
        ctx.check(
            "feed_horn_clears_turntable_at_rest",
            rest_gap >= 0.08,
            f"Feed horn rest clearance above turntable is only {rest_gap:.4f} m.",
        )

    if (
        rest_feed_center is None
        or mid_feed_aabb is None
        or mid_feed_center is None
        or high_feed_aabb is None
        or high_feed_center is None
        or turntable_deck_aabb is None
    ):
        ctx.fail("feed_horn_pose_progression_measurable", "Could not measure feed horn pose progression.")
    else:
        mid_gap = mid_feed_aabb[0][2] - turntable_deck_aabb[1][2]
        high_gap = high_feed_aabb[0][2] - turntable_deck_aabb[1][2]
        ctx.check(
            "feed_horn_rises_with_elevation",
            mid_feed_center[2] > rest_feed_center[2] + 0.10
            and high_feed_center[2] > mid_feed_center[2] + 0.10,
            f"Feed horn centers should rise with elevation, got rest={rest_feed_center}, mid={mid_feed_center}, high={high_feed_center}.",
        )
        ctx.check(
            "feed_horn_clears_turntable_through_elevation",
            mid_gap >= 0.12 and high_gap >= 0.22,
            f"Feed horn gaps above turntable are rest={rest_gap:.4f}, mid={mid_gap:.4f}, high={high_gap:.4f} m.",
        )

    reflector_aabb = ctx.part_element_world_aabb(dish_assembly, elem=main_reflector)
    feed_aabb = ctx.part_element_world_aabb(dish_assembly, elem=feed_horn)
    if reflector_aabb is None:
        ctx.fail("reflector_aabb_available", "Could not evaluate reflector dimensions.")
    else:
        reflector_dims = tuple(reflector_aabb[1][i] - reflector_aabb[0][i] for i in range(3))
        ctx.check(
            "reflector_reads_as_deep_dish",
            0.30 <= reflector_dims[0] <= 0.46
            and 1.30 <= reflector_dims[1] <= 1.55
            and 1.30 <= reflector_dims[2] <= 1.55,
            f"Unexpected reflector extents {reflector_dims}.",
        )

    if reflector_aabb is None or feed_aabb is None:
        ctx.fail("feed_alignment_measurable", "Could not measure feed horn placement relative to reflector.")
    else:
        reflector_center = tuple((reflector_aabb[0][i] + reflector_aabb[1][i]) * 0.5 for i in range(3))
        feed_center = tuple((feed_aabb[0][i] + feed_aabb[1][i]) * 0.5 for i in range(3))
        ctx.check(
            "feed_horn_centered_on_dish_axis",
            abs(feed_center[1] - reflector_center[1]) <= 0.03
            and abs(feed_center[2] - reflector_center[2]) <= 0.03
            and feed_center[0] > reflector_center[0] + 0.22,
            f"Feed horn center {feed_center} should sit on dish axis ahead of reflector center {reflector_center}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
