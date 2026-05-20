from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_cluster_launcher")

    deck_gray = model.material("deck_gray", rgba=(0.54, 0.57, 0.60, 1.0))
    haze_gray = model.material("haze_gray", rgba=(0.63, 0.66, 0.69, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.40, 0.43, 0.47, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.23, 0.25, 0.28, 1.0))
    canister_olive = model.material("canister_olive", rgba=(0.35, 0.39, 0.34, 1.0))
    soot_black = model.material("soot_black", rgba=(0.11, 0.12, 0.13, 1.0))

    fixed_base = model.part("fixed_base")
    fixed_base.visual(
        Box((3.20, 3.20, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=deck_gray,
        name="deck_plate",
    )
    fixed_base.visual(
        Box((1.45, 1.45, 0.42)),
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
        material=haze_gray,
        name="armored_skirt",
    )
    fixed_base.visual(
        Cylinder(radius=0.78, length=0.64),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=machinery_gray,
        name="center_tower",
    )
    fixed_base.visual(
        Box((0.96, 0.74, 0.28)),
        origin=Origin(xyz=(-0.16, 0.0, 0.34)),
        material=gunmetal,
        name="base_drive_house",
    )
    fixed_base.visual(
        Cylinder(radius=0.84, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        material=gunmetal,
        name="tower_collar",
    )
    fixed_base.visual(
        Cylinder(radius=0.98, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=gunmetal,
        name="lower_bearing_ring",
    )
    fixed_base.inertial = Inertial.from_geometry(
        Box((3.20, 3.20, 1.00)),
        mass=12000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )

    turntable_fork = model.part("turntable_fork")
    turntable_fork.visual(
        Cylinder(radius=0.96, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=gunmetal,
        name="upper_bearing_ring",
    )
    turntable_fork.visual(
        Cylinder(radius=0.74, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=machinery_gray,
        name="turntable_drum",
    )
    turntable_fork.visual(
        Box((1.38, 1.18, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=deck_gray,
        name="rotating_deck",
    )
    turntable_fork.visual(
        Box((0.60, 0.82, 0.40)),
        origin=Origin(xyz=(-0.60, 0.0, 0.84)),
        material=gunmetal,
        name="azimuth_drive_pack",
    )
    turntable_fork.visual(
        Box((0.36, 0.18, 0.92)),
        origin=Origin(xyz=(-0.16, -0.62, 1.10)),
        material=haze_gray,
        name="left_support",
    )
    turntable_fork.visual(
        Box((0.36, 0.18, 0.92)),
        origin=Origin(xyz=(-0.16, 0.62, 1.10)),
        material=haze_gray,
        name="right_support",
    )
    turntable_fork.visual(
        Box((0.54, 0.10, 1.04)),
        origin=Origin(xyz=(0.14, -0.53, 1.46)),
        material=haze_gray,
        name="left_cheek",
    )
    turntable_fork.visual(
        Box((0.54, 0.10, 1.04)),
        origin=Origin(xyz=(0.14, 0.53, 1.46)),
        material=haze_gray,
        name="right_cheek",
    )
    turntable_fork.visual(
        Box((0.18, 1.10, 0.22)),
        origin=Origin(xyz=(-0.43, 0.0, 1.40)),
        material=machinery_gray,
        name="rear_bridge",
    )
    turntable_fork.visual(
        Cylinder(radius=0.13, length=0.03),
        origin=Origin(xyz=(0.14, -0.465, 1.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="left_capture_pad",
    )
    turntable_fork.visual(
        Cylinder(radius=0.13, length=0.03),
        origin=Origin(xyz=(0.14, 0.465, 1.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="right_capture_pad",
    )
    turntable_fork.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.14, -0.63, 1.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="left_bearing_cap",
    )
    turntable_fork.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.14, 0.63, 1.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="right_bearing_cap",
    )
    turntable_fork.inertial = Inertial.from_geometry(
        Box((1.70, 1.45, 2.00)),
        mass=4800.0,
        origin=Origin(xyz=(-0.05, 0.0, 1.00)),
    )

    canister_cluster = model.part("canister_cluster")
    canister_cluster.visual(
        Cylinder(radius=0.11, length=0.90),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_shaft",
    )
    canister_cluster.visual(
        Cylinder(radius=0.14, length=0.08),
        origin=Origin(xyz=(0.0, -0.37, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="left_trunnion_boss",
    )
    canister_cluster.visual(
        Cylinder(radius=0.14, length=0.08),
        origin=Origin(xyz=(0.0, 0.37, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="right_trunnion_boss",
    )
    canister_cluster.visual(
        Box((0.26, 0.74, 1.22)),
        origin=Origin(xyz=(-0.18, 0.0, 0.0)),
        material=haze_gray,
        name="rear_manifold",
    )
    canister_cluster.visual(
        Box((0.18, 0.74, 0.26)),
        origin=Origin(xyz=(-0.02, 0.0, 0.0)),
        material=gunmetal,
        name="center_spine",
    )
    canister_cluster.visual(
        Box((0.44, 0.10, 1.22)),
        origin=Origin(xyz=(0.06, -0.32, 0.0)),
        material=haze_gray,
        name="left_side_armor",
    )
    canister_cluster.visual(
        Box((0.44, 0.10, 1.22)),
        origin=Origin(xyz=(0.06, 0.32, 0.0)),
        material=haze_gray,
        name="right_side_armor",
    )
    canister_cluster.visual(
        Box((0.44, 0.74, 0.10)),
        origin=Origin(xyz=(0.06, 0.0, 0.61)),
        material=haze_gray,
        name="top_armor",
    )
    canister_cluster.visual(
        Box((0.44, 0.74, 0.10)),
        origin=Origin(xyz=(0.06, 0.0, -0.61)),
        material=haze_gray,
        name="bottom_armor",
    )
    canister_cluster.visual(
        Box((0.18, 0.74, 0.24)),
        origin=Origin(xyz=(0.26, 0.0, 0.0)),
        material=gunmetal,
        name="front_web",
    )
    canister_cluster.visual(
        Box((0.12, 0.10, 1.24)),
        origin=Origin(xyz=(1.46, -0.32, 0.0)),
        material=haze_gray,
        name="left_front_frame",
    )
    canister_cluster.visual(
        Box((0.12, 0.10, 1.24)),
        origin=Origin(xyz=(1.46, 0.32, 0.0)),
        material=haze_gray,
        name="right_front_frame",
    )
    canister_cluster.visual(
        Box((0.12, 0.74, 0.10)),
        origin=Origin(xyz=(1.46, 0.0, 0.61)),
        material=haze_gray,
        name="upper_front_frame",
    )
    canister_cluster.visual(
        Box((0.12, 0.74, 0.10)),
        origin=Origin(xyz=(1.46, 0.0, -0.61)),
        material=haze_gray,
        name="lower_front_frame",
    )
    canister_cluster.visual(
        Box((1.28, 0.82, 0.08)),
        origin=Origin(xyz=(0.82, 0.0, 0.70)),
        material=haze_gray,
        name="rain_hood",
    )

    canister_rows = (-0.45, -0.15, 0.15, 0.45)
    canister_cols = (-0.16, 0.16)
    for row_index, z_pos in enumerate(canister_rows):
        for col_index, y_pos in enumerate(canister_cols):
            canister_cluster.visual(
                Cylinder(radius=0.12, length=1.50),
                origin=Origin(xyz=(0.74, y_pos, z_pos), rpy=(0.0, pi / 2.0, 0.0)),
                material=canister_olive,
                name=f"cell_{row_index}_{col_index}",
            )
            canister_cluster.visual(
                Cylinder(radius=0.132, length=0.05),
                origin=Origin(xyz=(1.505, y_pos, z_pos), rpy=(0.0, pi / 2.0, 0.0)),
                material=soot_black,
                name=f"muzzle_ring_{row_index}_{col_index}",
            )

    canister_cluster.inertial = Inertial.from_geometry(
        Box((1.90, 0.96, 1.36)),
        mass=2200.0,
        origin=Origin(xyz=(0.63, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=fixed_base,
        child=turntable_fork,
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90000.0, velocity=0.45),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=turntable_fork,
        child=canister_cluster,
        origin=Origin(xyz=(0.14, 0.0, 1.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55000.0, velocity=0.55, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_base = object_model.get_part("fixed_base")
    turntable_fork = object_model.get_part("turntable_fork")
    canister_cluster = object_model.get_part("canister_cluster")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_axis")

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

    ctx.check(
        "azimuth_axis_vertical",
        _axis_matches(azimuth.axis, (0.0, 0.0, 1.0)),
        f"azimuth axis was {azimuth.axis}",
    )
    ctx.check(
        "elevation_axis_horizontal",
        _axis_matches(elevation.axis, (0.0, 1.0, 0.0)),
        f"elevation axis was {elevation.axis}",
    )
    elevation_limits = elevation.motion_limits
    ctx.check(
        "elevation_limits_plausible",
        elevation_limits is not None
        and elevation_limits.lower is not None
        and elevation_limits.upper is not None
        and abs(elevation_limits.lower - 0.0) <= 1e-6
        and 1.0 <= elevation_limits.upper <= 1.2,
        f"elevation limits were {elevation_limits}",
    )

    with ctx.pose({azimuth: 0.0, elevation: 0.0}):
        ctx.expect_contact(
            turntable_fork,
            fixed_base,
            elem_a="upper_bearing_ring",
            elem_b="lower_bearing_ring",
            name="turntable_ring_seats_on_base_ring",
        )
        ctx.expect_overlap(
            turntable_fork,
            fixed_base,
            axes="xy",
            elem_a="upper_bearing_ring",
            elem_b="lower_bearing_ring",
            min_overlap=1.80,
            name="turntable_ring_stays_concentric_with_base_ring",
        )
        ctx.expect_contact(
            turntable_fork,
            canister_cluster,
            elem_a="left_capture_pad",
            elem_b="trunnion_shaft",
            name="left_capture_pad_contacts_trunnion",
        )
        ctx.expect_contact(
            turntable_fork,
            canister_cluster,
            elem_a="right_capture_pad",
            elem_b="trunnion_shaft",
            name="right_capture_pad_contacts_trunnion",
        )
        ctx.expect_gap(
            turntable_fork,
            canister_cluster,
            axis="y",
            positive_elem="right_cheek",
            negative_elem="right_side_armor",
            min_gap=0.10,
            max_gap=0.12,
            name="right_cheek_clips_outside_pack_body",
        )
        ctx.expect_gap(
            canister_cluster,
            turntable_fork,
            axis="y",
            positive_elem="left_side_armor",
            negative_elem="left_cheek",
            min_gap=0.10,
            max_gap=0.12,
            name="left_cheek_clips_outside_pack_body",
        )
        ctx.expect_overlap(
            turntable_fork,
            canister_cluster,
            axes="xz",
            elem_a="right_cheek",
            elem_b="trunnion_shaft",
            min_overlap=0.20,
            name="trunnion_axis_runs_inside_cheek_window",
        )
        ctx.expect_gap(
            canister_cluster,
            turntable_fork,
            axis="z",
            positive_elem="bottom_armor",
            negative_elem="rotating_deck",
            min_gap=0.03,
            max_gap=0.05,
            name="pack_clears_rotating_deck_in_level_pose",
        )
        ctx.expect_gap(
            canister_cluster,
            turntable_fork,
            axis="x",
            positive_elem="rear_manifold",
            negative_elem="rear_bridge",
            min_gap=0.15,
            name="rear_bridge_clears_pack_backface",
        )

    with ctx.pose({azimuth: 0.0, elevation: 1.0}):
        ctx.expect_contact(
            turntable_fork,
            canister_cluster,
            elem_a="left_capture_pad",
            elem_b="trunnion_shaft",
            name="left_capture_pad_holds_pack_at_high_elevation",
        )
        ctx.expect_contact(
            turntable_fork,
            canister_cluster,
            elem_a="right_capture_pad",
            elem_b="trunnion_shaft",
            name="right_capture_pad_holds_pack_at_high_elevation",
        )
        ctx.expect_gap(
            turntable_fork,
            canister_cluster,
            axis="y",
            positive_elem="right_cheek",
            negative_elem="right_side_armor",
            min_gap=0.10,
            max_gap=0.12,
            name="right_cheek_retains_pack_at_high_elevation",
        )
        ctx.expect_gap(
            canister_cluster,
            turntable_fork,
            axis="y",
            positive_elem="left_side_armor",
            negative_elem="left_cheek",
            min_gap=0.10,
            max_gap=0.12,
            name="left_cheek_retains_pack_at_high_elevation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
