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
    LatheGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BARREL_NEUTRAL_ELEVATION = 0.98


def _build_stone_ring_mesh(outer_radius: float, inner_radius: float, height: float):
    outer_profile = [
        (outer_radius - 0.03, 0.0),
        (outer_radius + 0.02, height * 0.45),
        (outer_radius, height),
    ]
    inner_profile = [
        (inner_radius + 0.03, 0.0),
        (inner_radius, height),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=96),
        "stone_base_ring",
    )


def _build_barrel_mesh():
    outer_profile = [
        (0.19, -0.16),
        (0.24, -0.11),
        (0.30, -0.02),
        (0.33, 0.10),
        (0.34, 0.25),
        (0.34, 0.40),
        (0.32, 0.52),
        (0.29, 0.60),
        (0.27, 0.64),
    ]
    inner_profile = [
        (0.00, -0.02),
        (0.10, 0.02),
        (0.13, 0.16),
        (0.15, 0.40),
        (0.15, 0.60),
        (0.15, 0.64),
    ]

    barrel_shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    barrel_shell.rotate_y(math.pi / 2.0)

    muzzle_reinforce = CylinderGeometry(
        0.35,
        0.045,
        radial_segments=40,
        closed=True,
    )
    muzzle_reinforce.rotate_y(math.pi / 2.0)
    muzzle_reinforce.translate(0.62, 0.0, 0.0)

    trunnion_radius = 0.07
    trunnion_length = 0.17
    left_trunnion = CylinderGeometry(
        trunnion_radius,
        trunnion_length,
        radial_segments=40,
        closed=True,
    )
    left_trunnion.rotate_x(math.pi / 2.0)
    left_trunnion.translate(0.00, -0.435, 0.0)

    right_trunnion = CylinderGeometry(
        trunnion_radius,
        trunnion_length,
        radial_segments=40,
        closed=True,
    )
    right_trunnion.rotate_x(math.pi / 2.0)
    right_trunnion.translate(0.00, 0.435, 0.0)

    cascabel = SphereGeometry(0.085, width_segments=28, height_segments=20)
    cascabel.translate(-0.22, 0.0, 0.0)

    barrel_shell.merge(muzzle_reinforce)
    barrel_shell.merge(left_trunnion)
    barrel_shell.merge(right_trunnion)
    barrel_shell.merge(cascabel)
    barrel_shell.rotate_y(-BARREL_NEUTRAL_ELEVATION)
    return mesh_from_geometry(barrel_shell, "mortar_barrel")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fortress_mortar")

    stone = model.material("stone", rgba=(0.60, 0.60, 0.60, 1.0))
    stone_dark = model.material("stone_dark", rgba=(0.47, 0.47, 0.47, 1.0))
    weathered_oak = model.material("weathered_oak", rgba=(0.43, 0.28, 0.17, 1.0))
    aged_oak = model.material("aged_oak", rgba=(0.53, 0.36, 0.20, 1.0))
    iron = model.material("iron", rgba=(0.16, 0.16, 0.18, 1.0))
    iron_highlight = model.material("iron_highlight", rgba=(0.24, 0.24, 0.26, 1.0))

    ring_height = 0.24

    stone_base = model.part("stone_base_ring")
    stone_base.visual(
        _build_stone_ring_mesh(outer_radius=1.42, inner_radius=0.72, height=ring_height),
        material=stone,
        name="stone_ring",
    )
    traverse_platform = model.part("traverse_platform")
    traverse_platform.visual(
        Cylinder(radius=0.95, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=weathered_oak,
        name="turntable_shoe",
    )
    traverse_platform.visual(
        Cylinder(radius=0.22, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=aged_oak,
        name="pivot_drum",
    )
    for x_pos in (-0.70, 0.0, 0.70):
        traverse_platform.visual(
            Box((0.24, 1.54, 0.08)),
            origin=Origin(xyz=(x_pos, 0.0, 0.09)),
            material=weathered_oak,
            name=f"crossbeam_{str(x_pos).replace('-', 'm').replace('.', '_')}",
        )
    for y_pos in (-0.46, 0.46):
        traverse_platform.visual(
            Box((2.06, 0.16, 0.12)),
            origin=Origin(xyz=(0.0, y_pos, 0.18)),
            material=aged_oak,
            name=f"stringer_{'left' if y_pos < 0.0 else 'right'}",
        )
    traverse_platform.visual(
        Cylinder(radius=0.88, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=aged_oak,
        name="subdeck_disc",
    )
    for index, y_pos in enumerate((-0.63, -0.42, -0.21, 0.0, 0.21, 0.42, 0.63), start=1):
        traverse_platform.visual(
            Box((2.22, 0.16, 0.07)),
            origin=Origin(xyz=(0.0, y_pos, 0.275)),
            material=weathered_oak,
            name=f"deck_plank_{index}",
        )

    iron_bed = model.part("iron_bed")
    iron_bed.visual(
        Box((1.58, 1.24, 0.10)),
        origin=Origin(xyz=(0.06, 0.0, 0.05)),
        material=iron,
        name="bed_base",
    )
    iron_bed.visual(
        Box((0.18, 1.08, 0.16)),
        origin=Origin(xyz=(-0.56, 0.0, 0.13)),
        material=iron_highlight,
        name="rear_transom",
    )
    iron_bed.visual(
        Box((0.14, 1.04, 0.18)),
        origin=Origin(xyz=(0.66, 0.0, 0.15)),
        material=iron_highlight,
        name="front_tie",
    )
    for side_name, y_center in (("left", -0.56), ("right", 0.56)):
        iron_bed.visual(
            Box((1.30, 0.08, 0.14)),
            origin=Origin(xyz=(0.02, y_center, 0.17)),
            material=iron,
            name=f"{side_name}_sill",
        )
        iron_bed.visual(
            Box((0.12, 0.08, 0.35)),
            origin=Origin(xyz=(0.00, y_center, 0.175)),
            material=iron,
            name=f"{side_name}_trunnion_pedestal",
        )
        iron_bed.visual(
            Box((0.10, 0.08, 0.28)),
            origin=Origin(xyz=(-0.16, y_center, 0.29)),
            material=iron_highlight,
            name=f"{side_name}_rear_jaw",
        )
        iron_bed.visual(
            Box((0.10, 0.08, 0.28)),
            origin=Origin(xyz=(0.16, y_center, 0.29)),
            material=iron_highlight,
            name=f"{side_name}_front_jaw",
        )
        iron_bed.visual(
            Box((0.38, 0.08, 0.18)),
            origin=Origin(xyz=(-0.34, y_center, 0.19)),
            material=iron_highlight,
            name=f"{side_name}_rear_web",
        )
        iron_bed.visual(
            Box((0.48, 0.08, 0.18)),
            origin=Origin(xyz=(0.30, y_center, 0.19)),
            material=iron_highlight,
            name=f"{side_name}_front_web",
        )

    mortar_barrel = model.part("mortar_barrel")
    mortar_barrel.visual(
        _build_barrel_mesh(),
        material=iron_highlight,
        name="barrel_casting",
    )

    model.articulation(
        "traverse_rotation",
        ArticulationType.REVOLUTE,
        parent=stone_base,
        child=traverse_platform,
        origin=Origin(xyz=(0.0, 0.0, ring_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.8,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "platform_to_bed",
        ArticulationType.FIXED,
        parent=traverse_platform,
        child=iron_bed,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
    )
    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=iron_bed,
        child=mortar_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.6,
            lower=-0.30,
            upper=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stone_base = object_model.get_part("stone_base_ring")
    traverse_platform = object_model.get_part("traverse_platform")
    iron_bed = object_model.get_part("iron_bed")
    mortar_barrel = object_model.get_part("mortar_barrel")
    traverse_rotation = object_model.get_articulation("traverse_rotation")
    barrel_elevation = object_model.get_articulation("barrel_elevation")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=1e-4)
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
        "traverse_axis_vertical",
        tuple(traverse_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical traverse axis, got {traverse_rotation.axis}",
    )
    ctx.check(
        "elevation_axis_lateral",
        tuple(barrel_elevation.axis) == (0.0, 1.0, 0.0),
        details=f"expected side trunnion elevation axis, got {barrel_elevation.axis}",
    )

    ctx.expect_gap(
        traverse_platform,
        stone_base,
        axis="z",
        positive_elem="turntable_shoe",
        negative_elem="stone_ring",
        min_gap=0.0,
        max_gap=0.002,
        name="turntable_shoe_seats_on_stone_ring",
    )
    ctx.expect_overlap(
        traverse_platform,
        stone_base,
        axes="xy",
        elem_a="turntable_shoe",
        elem_b="stone_ring",
        min_overlap=1.40,
        name="turntable_shoe_has_broad_bearing_on_ring",
    )
    ctx.expect_gap(
        iron_bed,
        traverse_platform,
        axis="z",
        positive_elem="bed_base",
        min_gap=0.0,
        max_gap=0.002,
        name="iron_bed_seats_on_platform",
    )
    ctx.expect_within(
        iron_bed,
        traverse_platform,
        axes="xy",
        margin=0.04,
        name="iron_bed_stays_within_platform_plan",
    )
    ctx.expect_contact(
        mortar_barrel,
        iron_bed,
        contact_tol=0.001,
        name="barrel_contacts_bed_at_trunnions",
    )

    with ctx.pose({barrel_elevation: -0.22}):
        ctx.expect_contact(
            mortar_barrel,
            iron_bed,
            contact_tol=0.001,
            name="barrel_stays_bedded_at_low_elevation",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_low_elevation")

    with ctx.pose({barrel_elevation: 0.20, traverse_rotation: 0.75}):
        ctx.expect_contact(
            mortar_barrel,
            iron_bed,
            contact_tol=0.001,
            name="barrel_stays_bedded_when_traversed_and_raised",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_when_traversed_and_raised")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
