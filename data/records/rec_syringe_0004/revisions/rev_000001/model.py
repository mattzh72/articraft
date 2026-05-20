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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _x_axis_origin(*, xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_syringe", assets=ASSETS)

    barrel_polymer = model.material("barrel_polymer", rgba=(0.80, 0.86, 0.92, 0.58))
    utility_gray = model.material("utility_gray", rgba=(0.30, 0.33, 0.36, 1.0))
    molded_black = model.material("molded_black", rgba=(0.11, 0.12, 0.13, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    rod_steel = model.material("rod_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    marking_black = model.material("marking_black", rgba=(0.12, 0.12, 0.12, 1.0))
    piston_gray = model.material("piston_gray", rgba=(0.70, 0.72, 0.75, 1.0))

    barrel = model.part("barrel")

    finger_flange_mesh = _save_mesh(
        "finger_flange.obj",
        ExtrudeGeometry(rounded_rect_profile(0.028, 0.029, 0.0065, corner_segments=8), 0.008),
    )
    thumb_pad_mesh = _save_mesh(
        "thumb_pad.obj",
        ExtrudeGeometry(rounded_rect_profile(0.018, 0.064, 0.008, corner_segments=10), 0.012),
    )
    reinforcement_band_mesh = _save_mesh(
        "barrel_reinforcement_band.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.0168, -0.003),
                (0.0168, 0.003),
            ],
            [
                (0.0152, -0.003),
                (0.0152, 0.003),
            ],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
    )

    barrel_shell_mesh = LatheGeometry.from_shell_profiles(
        [
            (0.0019, 0.000),
            (0.0031, 0.006),
            (0.0044, 0.015),
            (0.0087, 0.022),
            (0.0122, 0.029),
            (0.0155, 0.036),
            (0.0155, 0.132),
            (0.0172, 0.139),
            (0.0190, 0.146),
        ],
        [
            (0.0009, 0.000),
            (0.0012, 0.008),
            (0.0018, 0.015),
            (0.0028, 0.021),
            (0.0109, 0.029),
            (0.0110, 0.132),
            (0.0110, 0.146),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    ).rotate_y(math.pi / 2.0)
    barrel.visual(
        _save_mesh("barrel_shell.obj", barrel_shell_mesh),
        material=barrel_polymer,
        name="barrel_shell",
    )

    barrel.visual(
        finger_flange_mesh,
        origin=Origin(xyz=(0.128, 0.030, 0.0)),
        material=utility_gray,
        name="finger_flange_pos",
    )
    barrel.visual(
        finger_flange_mesh,
        origin=Origin(xyz=(0.128, -0.030, 0.0)),
        material=utility_gray,
        name="finger_flange_neg",
    )
    barrel.visual(
        Box((0.020, 0.010, 0.014)),
        origin=Origin(xyz=(0.128, 0.018, 0.0)),
        material=utility_gray,
        name="flange_gusset_pos",
    )
    barrel.visual(
        Box((0.020, 0.010, 0.014)),
        origin=Origin(xyz=(0.128, -0.018, 0.0)),
        material=utility_gray,
        name="flange_gusset_neg",
    )
    barrel.visual(
        Cylinder(radius=0.0076, length=0.008),
        origin=_x_axis_origin(xyz=(0.022, 0.0, 0.0)),
        material=utility_gray,
        name="tip_lock_collar",
    )
    barrel.visual(
        Box((0.005, 0.006, 0.0025)),
        origin=Origin(xyz=(0.022, 0.0085, 0.0)),
        material=utility_gray,
        name="tip_lock_tab_pos",
    )
    barrel.visual(
        Box((0.005, 0.006, 0.0025)),
        origin=Origin(xyz=(0.022, -0.0085, 0.0)),
        material=utility_gray,
        name="tip_lock_tab_neg",
    )
    barrel.visual(
        reinforcement_band_mesh,
        origin=Origin(xyz=(0.117, 0.0, 0.0)),
        material=utility_gray,
        name="barrel_reinforcement_band",
    )

    barrel.visual(
        Box((0.092, 0.028, 0.004)),
        origin=Origin(xyz=(0.196, 0.0, 0.012)),
        material=utility_gray,
        name="guide_top_rail",
    )
    barrel.visual(
        Box((0.092, 0.028, 0.004)),
        origin=Origin(xyz=(0.196, 0.0, -0.012)),
        material=utility_gray,
        name="guide_bottom_rail",
    )
    barrel.visual(
        Box((0.092, 0.004, 0.020)),
        origin=Origin(xyz=(0.196, 0.012, 0.0)),
        material=utility_gray,
        name="guide_side_rail_pos",
    )
    barrel.visual(
        Box((0.092, 0.004, 0.020)),
        origin=Origin(xyz=(0.196, -0.012, 0.0)),
        material=utility_gray,
        name="guide_side_rail_neg",
    )

    for name, xyz, size in [
        ("guide_top_brace", (0.149, 0.0, 0.012), (0.010, 0.022, 0.004)),
        ("guide_bottom_brace", (0.149, 0.0, -0.012), (0.010, 0.022, 0.004)),
        ("guide_side_brace_pos", (0.149, 0.012, 0.0), (0.010, 0.004, 0.020)),
        ("guide_side_brace_neg", (0.149, -0.012, 0.0), (0.010, 0.004, 0.020)),
    ]:
        barrel.visual(Box(size), origin=Origin(xyz=xyz), material=utility_gray, name=name)

    for index in range(11):
        x_pos = 0.043 + (0.008 * index)
        mark_len = 0.012 if index % 5 == 0 else 0.0075
        barrel.visual(
            Box((0.0014, mark_len, 0.0006)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0157)),
            material=marking_black,
            name=f"graduation_{index:02d}",
        )

    for name, xyz in [
        ("fastener_flange_pos_front", (0.120, 0.028, 0.005)),
        ("fastener_flange_pos_rear", (0.136, 0.028, 0.005)),
        ("fastener_flange_neg_front", (0.120, -0.028, 0.005)),
        ("fastener_flange_neg_rear", (0.136, -0.028, 0.005)),
        ("fastener_rail_front", (0.171, 0.0, 0.015)),
        ("fastener_rail_rear", (0.221, 0.0, 0.015)),
    ]:
        barrel.visual(
            Cylinder(radius=0.0018, length=0.002),
            origin=Origin(xyz=xyz),
            material=fastener_steel,
            name=name,
        )

    barrel.inertial = Inertial.from_geometry(
        Box((0.245, 0.090, 0.040)),
        mass=0.18,
        origin=Origin(xyz=(0.121, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0040, length=0.206),
        origin=_x_axis_origin(xyz=(-0.008, 0.0, 0.0)),
        material=rod_steel,
        name="rod_shaft",
    )
    plunger.visual(
        Box((0.014, 0.020, 0.020)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=molded_black,
        name="guide_block",
    )
    plunger.visual(
        Box((0.020, 0.016, 0.020)),
        origin=Origin(xyz=(0.106, 0.0, 0.0)),
        material=molded_black,
        name="thumb_stem",
    )
    plunger.visual(
        Cylinder(radius=0.0065, length=0.016),
        origin=_x_axis_origin(xyz=(0.096, 0.0, 0.0)),
        material=molded_black,
        name="thumb_collar",
    )
    plunger.visual(
        thumb_pad_mesh,
        origin=Origin(xyz=(0.118, 0.0, 0.0)),
        material=molded_black,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0096, length=0.010),
        origin=_x_axis_origin(xyz=(-0.111, 0.0, 0.0)),
        material=piston_gray,
        name="piston_core",
    )
    plunger.visual(
        Cylinder(radius=0.0106, length=0.003),
        origin=_x_axis_origin(xyz=(-0.115, 0.0, 0.0)),
        material=seal_rubber,
        name="piston_front_lip",
    )
    plunger.visual(
        Cylinder(radius=0.0106, length=0.003),
        origin=_x_axis_origin(xyz=(-0.107, 0.0, 0.0)),
        material=seal_rubber,
        name="piston_rear_lip",
    )

    for name, xyz in [
        ("thumb_fastener_pos", (0.118, 0.015, 0.007)),
        ("thumb_fastener_neg", (0.118, -0.015, 0.007)),
    ]:
        plunger.visual(
            Cylinder(radius=0.0018, length=0.002),
            origin=Origin(xyz=xyz),
            material=fastener_steel,
            name=name,
        )

    plunger.inertial = Inertial.from_geometry(
        Box((0.245, 0.070, 0.040)),
        mass=0.09,
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.146, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=0.078,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    plunger_slide = object_model.get_articulation("plunger_slide")

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

    axis_ok = tuple(plunger_slide.axis) == (1.0, 0.0, 0.0)
    limits = plunger_slide.motion_limits
    limits_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and 0.06 <= limits.upper <= 0.09
        and abs(limits.lower) < 1e-9
    )
    ctx.check("plunger_slide_axis", axis_ok, details=f"axis={plunger_slide.axis!r}")
    ctx.check(
        "plunger_slide_limits",
        limits_ok,
        details=(
            "Expected realistic syringe stroke near 78 mm with a fully-depressed "
            f"zero pose, got limits={limits!r}"
        ),
    )

    barrel_aabb = ctx.part_world_aabb(barrel)
    plunger_aabb = ctx.part_world_aabb(plunger)
    if barrel_aabb is not None:
        barrel_len = barrel_aabb[1][0] - barrel_aabb[0][0]
        barrel_width = barrel_aabb[1][1] - barrel_aabb[0][1]
        barrel_height = barrel_aabb[1][2] - barrel_aabb[0][2]
        ctx.check(
            "barrel_realistic_size",
            0.22 <= barrel_len <= 0.26 and 0.07 <= barrel_width <= 0.10 and 0.03 <= barrel_height <= 0.05,
            details=(
                "Barrel assembly should read like a large clinical utility syringe, "
                f"got length={barrel_len:.4f}, width={barrel_width:.4f}, height={barrel_height:.4f}"
            ),
        )
    else:
        ctx.fail("barrel_aabb_present", "Barrel world AABB is unavailable.")

    if plunger_aabb is not None:
        plunger_len = plunger_aabb[1][0] - plunger_aabb[0][0]
        ctx.check(
            "plunger_realistic_size",
            0.22 <= plunger_len <= 0.27,
            details=f"Plunger assembly length should be substantial, got {plunger_len:.4f}",
        )
    else:
        ctx.fail("plunger_aabb_present", "Plunger world AABB is unavailable.")

    ctx.expect_contact(plunger, barrel, elem_a="guide_block", elem_b="guide_top_rail")
    ctx.expect_contact(plunger, barrel, elem_a="guide_block", elem_b="guide_side_rail_pos")
    ctx.expect_overlap(plunger, barrel, axes="y", min_overlap=0.018, elem_a="guide_block", elem_b="guide_top_rail")
    ctx.expect_overlap(plunger, barrel, axes="z", min_overlap=0.018, elem_a="guide_block", elem_b="guide_side_rail_pos")
    ctx.expect_within(plunger, barrel, axes="yz", margin=0.0, inner_elem="piston_front_lip", outer_elem="barrel_shell")
    ctx.expect_contact(
        plunger,
        barrel,
        elem_a="thumb_stem",
        elem_b="guide_top_rail",
        name="thumb_stem_sets_rear_stop_at_rest",
    )

    plunger_rest = ctx.part_world_position(plunger)
    if plunger_rest is None:
        ctx.fail("plunger_rest_position_present", "Plunger world position unavailable at rest.")

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({plunger_slide: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="plunger_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="plunger_lower_no_floating")
            ctx.expect_contact(plunger, barrel, elem_a="guide_block", elem_b="guide_top_rail")
            ctx.expect_contact(plunger, barrel, elem_a="guide_block", elem_b="guide_side_rail_neg")
            ctx.expect_contact(
                plunger,
                barrel,
                elem_a="thumb_stem",
                elem_b="guide_top_rail",
                name="thumb_stem_sets_rear_stop_at_lower",
            )

        with ctx.pose({plunger_slide: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="plunger_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="plunger_upper_no_floating")
            ctx.expect_contact(plunger, barrel, elem_a="guide_block", elem_b="guide_top_rail")
            ctx.expect_contact(plunger, barrel, elem_a="guide_block", elem_b="guide_side_rail_pos")
            ctx.expect_gap(
                plunger,
                barrel,
                axis="x",
                min_gap=0.003,
                positive_elem="thumb_pad",
                negative_elem="guide_top_rail",
                name="thumb_pad_rear_of_track_at_upper",
            )

            plunger_extended = ctx.part_world_position(plunger)
            if plunger_rest is not None and plunger_extended is not None:
                travel = plunger_extended[0] - plunger_rest[0]
                lateral_shift = abs(plunger_extended[1] - plunger_rest[1]) + abs(
                    plunger_extended[2] - plunger_rest[2]
                )
                ctx.check(
                    "plunger_travel_matches_limits",
                    abs(travel - limits.upper) < 1e-6,
                    details=f"Expected x-travel {limits.upper:.6f}, got {travel:.6f}",
                )
                ctx.check(
                    "plunger_motion_is_axial",
                    lateral_shift < 1e-6,
                    details=f"Expected pure axial travel, got lateral shift {lateral_shift:.6f}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
