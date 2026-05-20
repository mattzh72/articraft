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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

_REAL_GETCWD = os.getcwd
ASSETS = AssetContext.from_script(__file__)


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except OSError:
            pass
        return "/"


os.getcwd = _safe_getcwd


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _circle_profile(
    radius: float,
    center: tuple[float, float] = (0.0, 0.0),
    *,
    segments: int = 28,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    cx, cy = center
    points = [
        (
            cx + (radius * math.cos((2.0 * math.pi * index) / segments)),
            cy + (radius * math.sin((2.0 * math.pi * index) / segments)),
        )
        for index in range(segments)
    ]
    return list(reversed(points)) if clockwise else points


def _build_ring_mesh(*, outer_radius: float, inner_radius: float, thickness: float):
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, 0.0),
            (outer_radius, thickness),
        ],
        [
            (inner_radius, 0.0),
            (inner_radius, thickness),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )


def _build_carriage_cheek_mesh(
    *,
    length: float,
    height: float,
    thickness: float,
    trunnion_radius: float,
    trunnion_center: tuple[float, float],
):
    half_len = 0.5 * length
    outer_profile = [
        (-half_len, 0.0),
        (half_len - 0.10, 0.0),
        (half_len, 0.07),
        (half_len, 0.62 * height),
        (0.22 * length, height),
        (-0.20 * length, height),
        (-half_len, 0.52 * height),
    ]
    trunnion_hole = _circle_profile(
        trunnion_radius,
        trunnion_center,
        segments=32,
        clockwise=True,
    )
    cheek = ExtrudeWithHolesGeometry(
        outer_profile,
        [trunnion_hole],
        thickness,
        center=True,
    )
    cheek.rotate_x(math.pi / 2.0)
    return cheek


def _build_barrel_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0, -0.76),
            (0.055, -0.73),
            (0.072, -0.69),
            (0.060, -0.63),
            (0.048, -0.58),
            (0.048, -0.55),
            (0.168, -0.55),
            (0.186, -0.47),
            (0.190, -0.34),
            (0.176, -0.14),
            (0.160, 0.00),
            (0.145, 0.42),
            (0.138, 0.92),
            (0.148, 1.18),
            (0.156, 1.35),
        ],
        [
            (0.0, -0.48),
            (0.032, -0.42),
            (0.070, -0.28),
            (0.074, 0.28),
            (0.076, 1.18),
            (0.080, 1.35),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_deck_cannon", assets=ASSETS)

    wood = model.material("weathered_oak", rgba=(0.43, 0.30, 0.18, 1.0))
    wood_dark = model.material("oak_caulking", rgba=(0.31, 0.21, 0.12, 1.0))
    bronze = model.material("naval_bronze", rgba=(0.74, 0.55, 0.28, 1.0))
    iron = model.material("forged_iron", rgba=(0.29, 0.30, 0.32, 1.0))
    iron_dark = model.material("gunmetal", rgba=(0.16, 0.18, 0.20, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((3.20, 2.80, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=wood,
        name="deck_planking",
    )
    deck.visual(
        Box((3.20, 0.02, 0.004)),
        origin=Origin(xyz=(0.0, -0.46, 0.102)),
        material=wood_dark,
        name="caulking_strip_a",
    )
    deck.visual(
        Box((3.20, 0.02, 0.004)),
        origin=Origin(xyz=(0.0, 0.00, 0.102)),
        material=wood_dark,
        name="caulking_strip_b",
    )
    deck.visual(
        Box((3.20, 0.02, 0.004)),
        origin=Origin(xyz=(0.0, 0.46, 0.102)),
        material=wood_dark,
        name="caulking_strip_c",
    )
    deck.inertial = Inertial.from_geometry(
        Box((3.20, 2.80, 0.10)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    deck_ring = model.part("deck_ring")
    deck_ring.visual(
        _save_mesh(
            _build_ring_mesh(outer_radius=0.72, inner_radius=0.54, thickness=0.03),
            "deck_ring.obj",
        ),
        material=iron_dark,
        name="ring_race",
    )
    deck_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.72, length=0.03),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.66, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=iron_dark,
        name="turntable",
    )
    carriage.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=iron,
        name="pivot_drum",
    )
    carriage.visual(
        Box((0.96, 0.42, 0.06)),
        origin=Origin(xyz=(-0.02, 0.0, 0.07)),
        material=iron_dark,
        name="carriage_bed",
    )
    carriage.visual(
        Box((0.58, 0.04, 0.42)),
        origin=Origin(xyz=(-0.08, 0.23, 0.24)),
        material=iron,
        name="left_cheek",
    )
    carriage.visual(
        Box((0.58, 0.04, 0.42)),
        origin=Origin(xyz=(-0.08, -0.23, 0.24)),
        material=iron,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.10, 0.42, 0.05)),
        origin=Origin(xyz=(-0.54, 0.0, 0.045)),
        material=iron_dark,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.14, 0.42, 0.08)),
        origin=Origin(xyz=(0.30, 0.0, 0.08)),
        material=iron_dark,
        name="front_crosshead",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((1.00, 0.70, 0.46)),
        mass=340.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        _save_mesh(_build_barrel_shell_mesh(), "bronze_barrel_shell.obj"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.190, length=0.08),
        origin=Origin(xyz=(-0.44, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="breech_ring",
    )
    barrel.visual(
        Cylinder(radius=0.158, length=0.10),
        origin=Origin(xyz=(1.24, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="muzzle_band",
    )
    barrel.visual(
        Cylinder(radius=0.036, length=0.21),
        origin=Origin(xyz=(0.0, 0.105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.036, length=0.21),
        origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="right_trunnion",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=1.95),
        mass=940.0,
        origin=Origin(xyz=(0.32, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "deck_to_ring",
        ArticulationType.FIXED,
        parent=deck,
        child=deck_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )
    model.articulation(
        "ring_to_carriage",
        ArticulationType.REVOLUTE,
        parent=deck_ring,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=0.45,
            lower=-2.35,
            upper=2.35,
        ),
    )
    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.35,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    deck = object_model.get_part("deck")
    deck_ring = object_model.get_part("deck_ring")
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    traverse = object_model.get_articulation("ring_to_carriage")
    elevate = object_model.get_articulation("carriage_to_barrel")

    deck_planking = deck.get_visual("deck_planking")
    ring_race = deck_ring.get_visual("ring_race")
    turntable = carriage.get_visual("turntable")
    carriage_bed = carriage.get_visual("carriage_bed")
    left_cheek = carriage.get_visual("left_cheek")
    right_cheek = carriage.get_visual("right_cheek")
    left_trunnion = barrel.get_visual("left_trunnion")
    right_trunnion = barrel.get_visual("right_trunnion")
    muzzle_band = barrel.get_visual("muzzle_band")
    barrel_shell = barrel.get_visual("barrel_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(
        deck_ring,
        deck,
        axes="xy",
        max_dist=0.001,
        name="deck_ring_stays_centered_on_deck",
    )
    ctx.expect_contact(
        deck_ring,
        deck,
        elem_a=ring_race,
        elem_b=deck_planking,
        name="deck_ring_bolted_down_to_planks",
    )
    ctx.expect_origin_distance(
        carriage,
        deck_ring,
        axes="xy",
        max_dist=0.001,
        name="carriage_traverses_about_ring_center",
    )
    ctx.expect_contact(
        carriage,
        deck_ring,
        elem_a=turntable,
        elem_b=ring_race,
        name="turntable_bears_on_ring_race",
    )
    ctx.expect_overlap(
        carriage,
        deck_ring,
        axes="xy",
        min_overlap=1.20,
        name="carriage_footprint_stays_over_ring",
    )
    ctx.expect_origin_gap(
        barrel,
        carriage,
        axis="z",
        min_gap=0.36,
        max_gap=0.38,
        name="trunnion_axis_sits_above_turntable",
    )
    ctx.expect_contact(
        barrel,
        carriage,
        elem_a=left_trunnion,
        elem_b=left_cheek,
        name="left_trunnion_bears_in_left_cheek",
    )
    ctx.expect_contact(
        barrel,
        carriage,
        elem_a=right_trunnion,
        elem_b=right_cheek,
        name="right_trunnion_bears_in_right_cheek",
    )
    ctx.expect_gap(
        barrel,
        carriage,
        axis="z",
        min_gap=0.06,
        positive_elem=barrel_shell,
        negative_elem=carriage_bed,
        name="level_barrel_clears_carriage_bed",
    )
    ctx.expect_gap(
        barrel,
        deck,
        axis="z",
        min_gap=0.20,
        positive_elem=muzzle_band,
        negative_elem=deck_planking,
        name="level_muzzle_clears_deck",
    )

    muzzle_rest = ctx.part_element_world_aabb(barrel, elem=muzzle_band)
    if muzzle_rest is not None:
        with ctx.pose({elevate: elevate.motion_limits.upper}):
            muzzle_high = ctx.part_element_world_aabb(barrel, elem=muzzle_band)
            if muzzle_high is not None:
                ctx.check(
                    "elevation_raises_muzzle",
                    _aabb_center(muzzle_high)[2] > _aabb_center(muzzle_rest)[2] + 0.20,
                    details=(
                        f"rest_z={_aabb_center(muzzle_rest)[2]:.3f}, "
                        f"high_z={_aabb_center(muzzle_high)[2]:.3f}"
                    ),
                )
            ctx.expect_gap(
                barrel,
                deck,
                axis="z",
                min_gap=0.45,
                positive_elem=muzzle_band,
                negative_elem=deck_planking,
                name="elevated_muzzle_stands_high_above_deck",
            )
            ctx.expect_gap(
                barrel,
                carriage,
                axis="z",
                min_gap=0.001,
                positive_elem=barrel_shell,
                negative_elem=carriage_bed,
                name="elevated_barrel_still_clears_bed",
            )

    with ctx.pose({elevate: elevate.motion_limits.lower}):
        ctx.expect_gap(
            barrel,
            carriage,
            axis="z",
            min_gap=0.06,
            positive_elem=barrel_shell,
            negative_elem=carriage_bed,
            name="depressed_barrel_still_clears_bed",
        )
        ctx.expect_gap(
            barrel,
            deck,
            axis="z",
            min_gap=0.20,
            positive_elem=muzzle_band,
            negative_elem=deck_planking,
            name="depressed_muzzle_still_clears_deck",
        )

    if muzzle_rest is not None:
        with ctx.pose({traverse: traverse.motion_limits.upper}):
            muzzle_traversed = ctx.part_element_world_aabb(barrel, elem=muzzle_band)
            if muzzle_traversed is not None:
                traversed_center = _aabb_center(muzzle_traversed)
                rest_center = _aabb_center(muzzle_rest)
                ctx.check(
                    "traverse_swings_muzzle_sideways",
                    abs(traversed_center[1]) > 0.75,
                    details=(
                        f"rest_center={rest_center!r}, "
                        f"traversed_center={traversed_center!r}"
                    ),
                )
            ctx.expect_gap(
                carriage,
                deck_ring,
                axis="z",
                max_gap=0.001,
                max_penetration=1e-5,
                positive_elem=turntable,
                negative_elem=ring_race,
                name="traversed_turntable_remains_seated_on_ring",
            )

    limits = elevate.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({elevate: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_to_barrel_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_to_barrel_lower_no_floating")
        with ctx.pose({elevate: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_to_barrel_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_to_barrel_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
