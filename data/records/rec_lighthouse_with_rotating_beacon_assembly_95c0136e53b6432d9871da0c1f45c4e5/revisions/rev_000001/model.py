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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _loop_at_z(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_concrete_lighthouse")

    concrete = model.material("concrete", rgba=(0.72, 0.72, 0.70, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.25, 0.28, 1.0))
    painted_metal = model.material("painted_metal", rgba=(0.46, 0.48, 0.50, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.72, 0.84, 0.92, 0.30))
    warm_lens = model.material("warm_lens", rgba=(0.93, 0.79, 0.50, 0.42))
    signal_red = model.material("signal_red", rgba=(0.55, 0.08, 0.08, 1.0))

    tower = model.part("tower")

    tower_geom = section_loft(
        [
            _loop_at_z(2.30, 2.30, 0.12, 0.00),
            _loop_at_z(2.18, 2.18, 0.11, 1.80),
            _loop_at_z(1.88, 1.88, 0.09, 5.20),
            _loop_at_z(1.58, 1.58, 0.08, 7.20),
        ]
    )
    tower.visual(
        mesh_from_geometry(tower_geom, "lighthouse_tower_shell"),
        material=concrete,
        name="tower_shell",
    )
    tower.visual(
        Box((2.80, 2.80, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=concrete,
        name="foundation_plinth",
    )
    tower.visual(
        Box((2.42, 2.42, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=concrete,
        name="base_course",
    )

    gallery_bottom = 7.20
    gallery_thickness = 0.14
    curb_thickness = 0.10
    lantern_base_z = gallery_bottom + gallery_thickness + curb_thickness
    lantern_wall_height = 0.70
    lantern_top_z = lantern_base_z + lantern_wall_height

    tower.visual(
        Box((1.92, 1.92, gallery_thickness)),
        origin=Origin(xyz=(0.0, 0.0, gallery_bottom + gallery_thickness * 0.5)),
        material=concrete,
        name="gallery_slab",
    )
    tower.visual(
        Box((0.68, 0.68, curb_thickness)),
        origin=Origin(xyz=(0.0, 0.0, gallery_bottom + gallery_thickness + curb_thickness * 0.5)),
        material=concrete,
        name="lantern_curb",
    )

    lantern_outer = 0.50
    lantern_half = lantern_outer * 0.5
    post_t = 0.04
    rail_bottom_h = 0.05
    rail_top_h = 0.06
    clear_h = lantern_wall_height - rail_bottom_h - rail_top_h
    post_center = lantern_half - post_t * 0.5
    frame_z = lantern_base_z + lantern_wall_height * 0.5
    bottom_rail_z = lantern_base_z + rail_bottom_h * 0.5
    top_rail_z = lantern_top_z - rail_top_h * 0.5
    clear_center_z = lantern_base_z + rail_bottom_h + clear_h * 0.5
    face_center = lantern_half - post_t * 0.5
    glass_face_y = face_center
    glass_face_x = face_center
    glass_t = 0.012

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower.visual(
                Box((post_t, post_t, lantern_wall_height)),
                origin=Origin(xyz=(sx * post_center, sy * post_center, frame_z)),
                material=painted_metal,
            )

    tower.visual(
        Box((lantern_outer - 2.0 * post_t, post_t, rail_bottom_h)),
        origin=Origin(xyz=(0.0, face_center, bottom_rail_z)),
        material=painted_metal,
        name="front_sill",
    )
    tower.visual(
        Box((lantern_outer - 2.0 * post_t, post_t, rail_bottom_h)),
        origin=Origin(xyz=(0.0, -face_center, bottom_rail_z)),
        material=painted_metal,
    )
    tower.visual(
        Box((lantern_outer - 2.0 * post_t, post_t, rail_top_h)),
        origin=Origin(xyz=(0.0, face_center, top_rail_z)),
        material=painted_metal,
        name="front_head",
    )
    tower.visual(
        Box((lantern_outer - 2.0 * post_t, post_t, rail_top_h)),
        origin=Origin(xyz=(0.0, -face_center, top_rail_z)),
        material=painted_metal,
    )
    tower.visual(
        Box((post_t, lantern_outer - 2.0 * post_t, rail_bottom_h)),
        origin=Origin(xyz=(face_center, 0.0, bottom_rail_z)),
        material=painted_metal,
    )
    tower.visual(
        Box((post_t, lantern_outer - 2.0 * post_t, rail_bottom_h)),
        origin=Origin(xyz=(-face_center, 0.0, bottom_rail_z)),
        material=painted_metal,
    )
    tower.visual(
        Box((post_t, lantern_outer - 2.0 * post_t, rail_top_h)),
        origin=Origin(xyz=(face_center, 0.0, top_rail_z)),
        material=painted_metal,
    )
    tower.visual(
        Box((post_t, lantern_outer - 2.0 * post_t, rail_top_h)),
        origin=Origin(xyz=(-face_center, 0.0, top_rail_z)),
        material=painted_metal,
    )

    door_width = 0.15
    front_mullion_t = 0.03
    right_front_glass_w = 0.24
    tower.visual(
        Box((front_mullion_t, post_t, clear_h)),
        origin=Origin(xyz=(-0.045, face_center, clear_center_z)),
        material=painted_metal,
        name="front_mullion",
    )
    tower.visual(
        Box((right_front_glass_w, glass_t, clear_h)),
        origin=Origin(xyz=(0.09, glass_face_y, clear_center_z)),
        material=lantern_glass,
        name="front_right_glass",
    )
    tower.visual(
        Box((lantern_outer - 2.0 * post_t, glass_t, clear_h)),
        origin=Origin(xyz=(0.0, -glass_face_y, clear_center_z)),
        material=lantern_glass,
        name="rear_glass",
    )
    tower.visual(
        Box((glass_t, lantern_outer - 2.0 * post_t, clear_h)),
        origin=Origin(xyz=(glass_face_x, 0.0, clear_center_z)),
        material=lantern_glass,
        name="right_glass",
    )
    tower.visual(
        Box((glass_t, lantern_outer - 2.0 * post_t, clear_h)),
        origin=Origin(xyz=(-glass_face_x, 0.0, clear_center_z)),
        material=lantern_glass,
        name="left_glass",
    )

    roof_base_z = lantern_top_z + 0.03
    roof_cap_bottom = lantern_top_z + 0.06
    tower.visual(
        Box((0.62, 0.62, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, roof_base_z)),
        material=dark_metal,
        name="roof_base",
    )
    roof_geom = section_loft(
        [
            _loop_at_z(0.56, 0.56, 0.03, 0.00, corner_segments=6),
            _loop_at_z(0.40, 0.40, 0.025, 0.07, corner_segments=6),
            _loop_at_z(0.18, 0.18, 0.02, 0.16, corner_segments=6),
        ]
    )
    tower.visual(
        mesh_from_geometry(roof_geom, "lighthouse_roof_cap"),
        origin=Origin(xyz=(0.0, 0.0, roof_cap_bottom)),
        material=dark_metal,
        name="roof_cap",
    )
    tower.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, roof_cap_bottom + 0.16 + 0.05)),
        material=signal_red,
        name="roof_beacon_vent",
    )

    pedestal_top_z = lantern_base_z + 0.22
    tower.visual(
        Cylinder(radius=0.085, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, lantern_base_z + 0.09)),
        material=dark_metal,
        name="pedestal_drum",
    )
    tower.visual(
        Cylinder(radius=0.11, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, pedestal_top_z - 0.02)),
        material=painted_metal,
        name="pedestal_cap",
    )

    beacon = model.part("beacon")
    beacon.visual(
        Cylinder(radius=0.10, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_metal,
        name="beacon_collar",
    )
    beacon.visual(
        Cylinder(radius=0.082, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=painted_metal,
        name="beacon_base",
    )
    beacon.visual(
        Cylinder(radius=0.11, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=warm_lens,
        name="beacon_lens",
    )
    beacon.visual(
        Box((0.05, 0.13, 0.14)),
        origin=Origin(xyz=(0.055, 0.0, 0.18)),
        material=dark_metal,
        name="lamp_hood",
    )
    beacon.visual(
        Cylinder(radius=0.026, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=painted_metal,
        name="vent_stem",
    )
    beacon.visual(
        Cylinder(radius=0.092, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=dark_metal,
        name="beacon_top_cap",
    )

    access_panel = model.part("access_panel")
    panel_height = clear_h
    panel_t = 0.018
    stile_t = 0.022
    access_panel.visual(
        Box((stile_t, panel_t, panel_height)),
        origin=Origin(xyz=(stile_t * 0.5, 0.0, panel_height * 0.5)),
        material=painted_metal,
        name="hinge_stile",
    )
    access_panel.visual(
        Box((stile_t, panel_t, panel_height)),
        origin=Origin(xyz=(door_width - stile_t * 0.5, 0.0, panel_height * 0.5)),
        material=painted_metal,
        name="free_stile",
    )
    access_panel.visual(
        Box((door_width, panel_t, stile_t)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, stile_t * 0.5)),
        material=painted_metal,
        name="bottom_rail",
    )
    access_panel.visual(
        Box((door_width, panel_t, stile_t)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, panel_height - stile_t * 0.5)),
        material=painted_metal,
        name="top_rail",
    )
    access_panel.visual(
        Box((door_width - 2.0 * stile_t, 0.010, panel_height - 2.0 * stile_t)),
        origin=Origin(xyz=(door_width * 0.5, 0.0, panel_height * 0.5)),
        material=lantern_glass,
        name="panel_glass",
    )

    model.articulation(
        "beacon_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, pedestal_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )
    model.articulation(
        "access_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=access_panel,
        origin=Origin(xyz=(-0.21, face_center, lantern_base_z + rail_bottom_h)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    access_panel = object_model.get_part("access_panel")
    beacon_spin = object_model.get_articulation("beacon_spin")
    access_panel_hinge = object_model.get_articulation("access_panel_hinge")

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

    ctx.expect_contact(beacon, tower, elem_a="beacon_collar", elem_b="pedestal_cap")
    ctx.expect_contact(access_panel, tower)
    ctx.expect_within(beacon, tower, axes="xy", outer_elem="roof_base", margin=0.03)
    ctx.expect_gap(
        tower,
        beacon,
        axis="z",
        positive_elem="roof_base",
        max_gap=0.20,
        min_gap=0.08,
    )

    ctx.check(
        "beacon_spin_is_vertical_continuous",
        beacon_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(beacon_spin.axis) == (0.0, 0.0, 1.0)
        and beacon_spin.motion_limits is not None
        and beacon_spin.motion_limits.lower is None
        and beacon_spin.motion_limits.upper is None,
        details="Beacon should rotate continuously about a vertical axis.",
    )
    ctx.check(
        "access_panel_hinge_is_vertical",
        access_panel_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(access_panel_hinge.axis) == (0.0, 0.0, 1.0)
        and access_panel_hinge.motion_limits is not None
        and access_panel_hinge.motion_limits.lower is not None
        and access_panel_hinge.motion_limits.upper is not None
        and access_panel_hinge.motion_limits.lower >= -1e-6
        and access_panel_hinge.motion_limits.upper >= 1.2,
        details="Access panel should be a side-hinged vertical revolute panel.",
    )

    closed_panel_aabb = ctx.part_world_aabb(access_panel)
    assert closed_panel_aabb is not None
    with ctx.pose({beacon_spin: math.pi * 0.5}):
        ctx.expect_contact(beacon, tower, elem_a="beacon_collar", elem_b="pedestal_cap")
        ctx.expect_within(beacon, tower, axes="xy", outer_elem="roof_base", margin=0.03)
        ctx.expect_gap(
            tower,
            beacon,
            axis="z",
            positive_elem="roof_base",
            max_gap=0.20,
            min_gap=0.08,
        )

    with ctx.pose({access_panel_hinge: 1.1}):
        open_panel_aabb = ctx.part_world_aabb(access_panel)
        assert open_panel_aabb is not None
        ctx.check(
            "access_panel_swings_outward",
            open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.07,
            details="Open access panel should move outward from the lantern face.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
