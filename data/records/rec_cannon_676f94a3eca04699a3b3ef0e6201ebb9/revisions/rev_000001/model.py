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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_carronade")

    deck_wood = model.material("deck_wood", rgba=(0.53, 0.40, 0.25, 1.0))
    rail_wood = model.material("rail_wood", rgba=(0.40, 0.27, 0.15, 1.0))
    carriage_wood = model.material("carriage_wood", rgba=(0.34, 0.22, 0.12, 1.0))
    iron_dark = model.material("iron_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    iron_mid = model.material("iron_mid", rgba=(0.24, 0.25, 0.27, 1.0))

    barrel_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.075, -0.44),
                (0.100, -0.40),
                (0.145, -0.34),
                (0.190, -0.24),
                (0.205, -0.10),
                (0.205, 0.06),
                (0.192, 0.28),
                (0.175, 0.52),
                (0.160, 0.67),
                (0.180, 0.72),
            ],
            inner_profile=[
                (0.0, -0.34),
                (0.030, -0.31),
                (0.082, -0.28),
                (0.094, 0.00),
                (0.098, 0.48),
                (0.098, 0.69),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "carronade_barrel_shell",
    )

    deck_mount = model.part("deck_mount")
    deck_mount.visual(
        Box((2.80, 2.80, 0.12)),
        origin=Origin(xyz=(0.85, 0.0, 0.06)),
        material=deck_wood,
        name="deck_section",
    )
    for index, y in enumerate((-1.00, -0.60, -0.20, 0.20, 0.60, 1.00)):
        deck_mount.visual(
            Box((2.78, 0.020, 0.006)),
            origin=Origin(xyz=(0.85, y, 0.117)),
            material=rail_wood,
            name=f"plank_seam_{index}",
        )
    deck_mount.inertial = Inertial.from_geometry(
        Box((2.80, 2.80, 0.18)),
        mass=950.0,
        origin=Origin(xyz=(0.85, 0.0, 0.09)),
    )

    slide_base = model.part("slide_base")
    slide_base.visual(
        Box((1.70, 0.14, 0.08)),
        origin=Origin(xyz=(0.86, 0.21, 0.04)),
        material=rail_wood,
        name="port_skid",
    )
    slide_base.visual(
        Box((1.70, 0.14, 0.08)),
        origin=Origin(xyz=(0.86, -0.21, 0.04)),
        material=rail_wood,
        name="starboard_skid",
    )
    slide_base.visual(
        Box((1.58, 0.10, 0.04)),
        origin=Origin(xyz=(0.88, 0.21, 0.10)),
        material=rail_wood,
        name="port_rail_cap",
    )
    slide_base.visual(
        Box((1.58, 0.10, 0.04)),
        origin=Origin(xyz=(0.88, -0.21, 0.10)),
        material=rail_wood,
        name="starboard_rail_cap",
    )
    slide_base.visual(
        Box((0.20, 0.42, 0.08)),
        origin=Origin(xyz=(0.10, 0.0, 0.04)),
        material=rail_wood,
        name="aft_transom",
    )
    slide_base.visual(
        Box((0.16, 0.42, 0.08)),
        origin=Origin(xyz=(1.60, 0.0, 0.04)),
        material=rail_wood,
        name="front_transom",
    )
    slide_base.visual(
        Box((0.20, 0.22, 0.04)),
        origin=Origin(xyz=(0.00, 0.0, 0.10)),
        material=iron_mid,
        name="pivot_bolster",
    )
    slide_base.visual(
        Cylinder(radius=0.030, length=0.12),
        origin=Origin(xyz=(0.00, 0.0, 0.18)),
        material=iron_dark,
        name="pivot_pin",
    )
    slide_base.inertial = Inertial.from_geometry(
        Box((1.80, 0.70, 0.20)),
        mass=175.0,
        origin=Origin(xyz=(0.90, 0.0, 0.10)),
    )

    carriage_bed = model.part("carriage_bed")
    carriage_bed.visual(
        Box((0.96, 0.10, 0.05)),
        origin=Origin(xyz=(0.00, 0.20, 0.025)),
        material=carriage_wood,
        name="port_runner",
    )
    carriage_bed.visual(
        Box((0.96, 0.10, 0.05)),
        origin=Origin(xyz=(0.00, -0.20, 0.025)),
        material=carriage_wood,
        name="starboard_runner",
    )
    carriage_bed.visual(
        Box((0.70, 0.80, 0.02)),
        origin=Origin(xyz=(0.02, 0.0, 0.01)),
        material=carriage_wood,
        name="center_bed",
    )
    carriage_bed.visual(
        Box((0.78, 0.06, 0.56)),
        origin=Origin(xyz=(0.02, 0.345, 0.28)),
        material=carriage_wood,
        name="port_cheek",
    )
    carriage_bed.visual(
        Box((0.78, 0.06, 0.56)),
        origin=Origin(xyz=(0.02, -0.345, 0.28)),
        material=carriage_wood,
        name="starboard_cheek",
    )
    carriage_bed.visual(
        Box((0.12, 0.82, 0.05)),
        origin=Origin(xyz=(-0.38, 0.0, 0.025)),
        material=carriage_wood,
        name="aft_crossbeam",
    )
    carriage_bed.visual(
        Box((0.10, 0.82, 0.05)),
        origin=Origin(xyz=(0.36, 0.0, 0.025)),
        material=carriage_wood,
        name="front_crossbeam",
    )
    carriage_bed.visual(
        Box((0.12, 0.06, 0.18)),
        origin=Origin(xyz=(0.00, 0.335, 0.55)),
        material=iron_mid,
        name="port_trunnion_support",
    )
    carriage_bed.visual(
        Box((0.12, 0.06, 0.18)),
        origin=Origin(xyz=(0.00, -0.335, 0.55)),
        material=iron_mid,
        name="starboard_trunnion_support",
    )
    carriage_bed.inertial = Inertial.from_geometry(
        Box((1.00, 0.66, 0.58)),
        mass=140.0,
        origin=Origin(xyz=(0.00, 0.0, 0.29)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        barrel_shell,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_dark,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.050, length=0.10),
        origin=Origin(xyz=(0.0, 0.255, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_mid,
        name="port_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.050, length=0.10),
        origin=Origin(xyz=(0.0, -0.255, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_mid,
        name="starboard_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.212, length=0.08),
        origin=Origin(xyz=(-0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_mid,
        name="base_ring",
    )
    barrel.visual(
        Cylinder(radius=0.182, length=0.05),
        origin=Origin(xyz=(0.64, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_mid,
        name="muzzle_band",
    )
    barrel.visual(
        Cylinder(radius=0.055, length=0.14),
        origin=Origin(xyz=(-0.40, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_mid,
        name="cascabel_neck",
    )
    barrel.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(-0.50, 0.0, 0.0)),
        material=iron_mid,
        name="cascabel_button",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((1.28, 0.52, 0.52)),
        mass=620.0,
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
    )

    model.articulation(
        "deck_to_slide_traverse",
        ArticulationType.REVOLUTE,
        parent=deck_mount,
        child=slide_base,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.8,
            lower=-math.radians(55.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "slide_to_carriage_recoil",
        ArticulationType.PRISMATIC,
        parent=slide_base,
        child=carriage_bed,
        origin=Origin(xyz=(0.95, 0.0, 0.12)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=1.4,
            lower=0.0,
            upper=0.36,
        ),
    )
    model.articulation(
        "carriage_to_barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage_bed,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.9,
            lower=-math.radians(6.0),
            upper=math.radians(22.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck_mount = object_model.get_part("deck_mount")
    slide_base = object_model.get_part("slide_base")
    carriage_bed = object_model.get_part("carriage_bed")
    barrel = object_model.get_part("barrel")

    traverse = object_model.get_articulation("deck_to_slide_traverse")
    recoil = object_model.get_articulation("slide_to_carriage_recoil")
    elevation = object_model.get_articulation("carriage_to_barrel_elevation")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.012)
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

    ctx.expect_contact(slide_base, deck_mount, name="slide_base_resting_on_deck")
    ctx.expect_contact(carriage_bed, slide_base, name="carriage_runners_on_slide")
    ctx.expect_gap(
        carriage_bed,
        barrel,
        axis="y",
        positive_elem="port_cheek",
        negative_elem="port_trunnion",
        min_gap=0.0,
        max_gap=0.012,
        name="port_trunnion_port_cheek_running_clearance",
    )
    ctx.expect_gap(
        barrel,
        carriage_bed,
        axis="y",
        positive_elem="starboard_trunnion",
        negative_elem="starboard_cheek",
        min_gap=0.0,
        max_gap=0.012,
        name="starboard_trunnion_starboard_cheek_running_clearance",
    )
    ctx.expect_overlap(carriage_bed, slide_base, axes="x", min_overlap=0.90, name="carriage_has_rail_engagement")

    carriage_rest = ctx.part_world_position(carriage_bed)
    muzzle_rest = ctx.part_element_world_aabb(barrel, elem="muzzle_band")
    assert carriage_rest is not None
    assert muzzle_rest is not None

    recoil_limits = recoil.motion_limits
    assert recoil_limits is not None and recoil_limits.upper is not None
    with ctx.pose({recoil: recoil_limits.upper}):
        carriage_recoiled = ctx.part_world_position(carriage_bed)
        muzzle_recoiled = ctx.part_element_world_aabb(barrel, elem="muzzle_band")
        assert carriage_recoiled is not None
        assert muzzle_recoiled is not None
        assert carriage_recoiled[0] < carriage_rest[0] - 0.30
        assert muzzle_recoiled[1][0] < muzzle_rest[1][0] - 0.28
        ctx.expect_contact(carriage_bed, slide_base, name="recoiled_carriage_stays_on_slide")
        ctx.fail_if_parts_overlap_in_current_pose(name="recoil_upper_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=0.012, name="recoil_upper_no_floating")

    traverse_limits = traverse.motion_limits
    assert traverse_limits is not None
    assert traverse_limits.lower is not None and traverse_limits.upper is not None
    with ctx.pose({traverse: traverse_limits.lower}):
        traverse_port = ctx.part_world_position(carriage_bed)
        assert traverse_port is not None
        assert traverse_port[1] < carriage_rest[1] - 0.70
        ctx.expect_contact(slide_base, deck_mount, name="port_traverse_keeps_deck_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="traverse_lower_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=0.012, name="traverse_lower_no_floating")
    with ctx.pose({traverse: traverse_limits.upper}):
        traverse_starboard = ctx.part_world_position(carriage_bed)
        assert traverse_starboard is not None
        assert traverse_starboard[1] > carriage_rest[1] + 0.70
        ctx.expect_contact(slide_base, deck_mount, name="starboard_traverse_keeps_deck_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="traverse_upper_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=0.012, name="traverse_upper_no_floating")

    elevation_limits = elevation.motion_limits
    assert elevation_limits is not None
    assert elevation_limits.lower is not None and elevation_limits.upper is not None
    with ctx.pose({elevation: elevation_limits.lower}):
        muzzle_low = ctx.part_element_world_aabb(barrel, elem="muzzle_band")
        assert muzzle_low is not None
        ctx.expect_gap(
            carriage_bed,
            barrel,
            axis="y",
            positive_elem="port_cheek",
            negative_elem="port_trunnion",
            min_gap=0.0,
            max_gap=0.012,
            name="lower_elevation_port_trunnion_clearance",
        )
        ctx.expect_gap(
            barrel,
            carriage_bed,
            axis="y",
            positive_elem="starboard_trunnion",
            negative_elem="starboard_cheek",
            min_gap=0.0,
            max_gap=0.012,
            name="lower_elevation_starboard_trunnion_clearance",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="elevation_lower_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=0.012, name="elevation_lower_no_floating")
    with ctx.pose({elevation: elevation_limits.upper}):
        muzzle_high = ctx.part_element_world_aabb(barrel, elem="muzzle_band")
        assert muzzle_high is not None
        assert muzzle_high[1][2] > muzzle_rest[1][2] + 0.20
        ctx.expect_gap(
            carriage_bed,
            barrel,
            axis="y",
            positive_elem="port_cheek",
            negative_elem="port_trunnion",
            min_gap=0.0,
            max_gap=0.012,
            name="upper_elevation_port_trunnion_clearance",
        )
        ctx.expect_gap(
            barrel,
            carriage_bed,
            axis="y",
            positive_elem="starboard_trunnion",
            negative_elem="starboard_cheek",
            min_gap=0.0,
            max_gap=0.012,
            name="upper_elevation_starboard_trunnion_clearance",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="elevation_upper_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=0.012, name="elevation_upper_no_floating")

    with ctx.pose(
        {
            traverse: math.radians(40.0),
            recoil: recoil_limits.upper,
            elevation: elevation_limits.upper,
        }
    ):
        ctx.expect_contact(slide_base, deck_mount, name="combined_pose_slide_contact")
        ctx.expect_contact(carriage_bed, slide_base, name="combined_pose_carriage_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=0.012, name="combined_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
