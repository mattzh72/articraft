from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _shift_profile(profile, dx=0.0, dy=0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _material(name, rgba):
    return Material(name=name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="car_sunroof_cassette", assets=ASSETS)

    painted_metal = _material("painted_metal", (0.25, 0.27, 0.30, 1.0))
    brushed_aluminum = _material("brushed_aluminum", (0.69, 0.71, 0.73, 1.0))
    black_polymer = _material("black_polymer", (0.08, 0.09, 0.10, 1.0))
    rubber_seal = _material("rubber_seal", (0.03, 0.03, 0.03, 1.0))
    tinted_glass = _material("tinted_glass", (0.18, 0.26, 0.30, 0.45))
    satin_steel = _material("satin_steel", (0.56, 0.58, 0.61, 1.0))
    model.materials.extend(
        [
            painted_metal,
            brushed_aluminum,
            black_polymer,
            rubber_seal,
            tinted_glass,
            satin_steel,
        ]
    )

    opening_center_y = 0.14

    frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(1.02, 1.18, 0.055, corner_segments=10),
            [
                _shift_profile(
                    rounded_rect_profile(0.78, 0.58, 0.045, corner_segments=10),
                    0.0,
                    opening_center_y,
                )
            ],
            0.026,
            cap=True,
            center=False,
            closed=True,
        ),
        ASSETS.mesh_path("sunroof_frame_ring.obj"),
    )
    glass_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.76, 0.56, 0.042, corner_segments=10),
            0.005,
            cap=True,
            closed=True,
        ),
        ASSETS.mesh_path("sunroof_glass_panel.obj"),
    )
    frit_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.76, 0.56, 0.042, corner_segments=10),
            [rounded_rect_profile(0.68, 0.48, 0.028, corner_segments=8)],
            0.0015,
            cap=True,
            center=False,
            closed=True,
        ),
        ASSETS.mesh_path("sunroof_glass_frit.obj"),
    )
    left_cable_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.43, 0.44, 0.010),
                (-0.40, 0.22, 0.004),
                (-0.38, -0.14, -0.014),
                (-0.33, -0.48, -0.038),
            ],
            radius=0.009,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        ),
        ASSETS.mesh_path("sunroof_left_cable.obj"),
    )
    right_cable_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.43, 0.44, 0.010),
                (0.40, 0.22, 0.004),
                (0.38, -0.14, -0.014),
                (0.33, -0.48, -0.038),
            ],
            radius=0.009,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        ),
        ASSETS.mesh_path("sunroof_right_cable.obj"),
    )

    cassette = model.part("cassette_frame")
    cassette.visual(
        frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=painted_metal,
        name="roof_frame_ring",
    )
    for side, label in ((-1.0, "left"), (1.0, "right")):
        rail_x = side * 0.372
        cassette.visual(
            Box((0.050, 0.970, 0.026)),
            origin=Origin(xyz=(rail_x, -0.01, 0.013)),
            material=brushed_aluminum,
            name=f"{label}_guide_rail",
        )
        cassette.visual(
            Box((0.028, 0.920, 0.012)),
            origin=Origin(xyz=(rail_x, -0.01, 0.010)),
            material=black_polymer,
            name=f"{label}_track_insert",
        )
        cassette.visual(
            Box((0.014, 0.820, 0.008)),
            origin=Origin(xyz=(side * 0.325, 0.01, 0.006)),
            material=rubber_seal,
            name=f"{label}_opening_seal",
        )
        cassette.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(side * 0.392, 0.432, 0.005)),
            material=satin_steel,
            name=f"{label}_front_pulley",
        )
        cassette.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(side * 0.338, -0.490, -0.026)),
            material=satin_steel,
            name=f"{label}_rear_pulley",
        )
        cassette.visual(
            Box((0.050, 0.100, 0.018)),
            origin=Origin(xyz=(side * 0.480, 0.500, 0.008)),
            material=painted_metal,
            name=f"{label}_front_mount_bracket",
        )
        cassette.visual(
            Box((0.050, 0.100, 0.018)),
            origin=Origin(xyz=(side * 0.480, -0.500, 0.008)),
            material=painted_metal,
            name=f"{label}_rear_mount_bracket",
        )
    cassette.visual(
        Box((0.840, 0.055, 0.022)),
        origin=Origin(xyz=(0.0, 0.450, 0.013)),
        material=satin_steel,
        name="front_header_beam",
    )
    cassette.visual(
        Box((0.820, 0.036, 0.014)),
        origin=Origin(xyz=(0.0, 0.340, 0.008)),
        material=satin_steel,
        name="front_cross_tube",
    )
    cassette.visual(
        Box((0.840, 0.440, 0.016)),
        origin=Origin(xyz=(0.0, -0.340, -0.006)),
        material=black_polymer,
        name="rear_storage_pan",
    )
    cassette.visual(
        Box((0.720, 0.080, 0.016)),
        origin=Origin(xyz=(0.0, -0.540, -0.008)),
        material=black_polymer,
        name="rear_drain_trough",
    )
    cassette.visual(
        left_cable_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black_polymer,
        name="left_drive_cable",
    )
    cassette.visual(
        right_cable_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black_polymer,
        name="right_drive_cable",
    )
    cassette.inertial = Inertial.from_geometry(
        Box((1.02, 1.18, 0.16)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    glass_panel = model.part("glass_panel")
    glass_panel.visual(
        glass_mesh,
        origin=Origin(xyz=(0.360, 0.0, 0.024)),
        material=tinted_glass,
        name="glass_lite",
    )
    glass_panel.visual(
        frit_mesh,
        origin=Origin(xyz=(0.360, 0.0, 0.0225)),
        material=black_polymer,
        name="ceramic_frit_band",
    )
    glass_panel.visual(
        Box((0.034, 0.440, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin_steel,
        name="left_carrier_rail",
    )
    glass_panel.visual(
        Box((0.034, 0.440, 0.018)),
        origin=Origin(xyz=(0.720, 0.0, 0.014)),
        material=satin_steel,
        name="right_carrier_rail",
    )
    for side, x_center in (("left", 0.0), ("right", 0.720)):
        for label, y_center in (("front", 0.200), ("rear", -0.200)):
            glass_panel.visual(
                Box((0.040, 0.080, 0.020)),
                origin=Origin(xyz=(x_center, y_center, 0.015)),
                material=black_polymer,
                name=f"{label}_{side}_shoe",
            )
    glass_panel.visual(
        Box((0.660, 0.050, 0.012)),
        origin=Origin(xyz=(0.360, 0.230, 0.015)),
        material=satin_steel,
        name="front_reinforcement_bow",
    )
    glass_panel.visual(
        Box((0.580, 0.045, 0.010)),
        origin=Origin(xyz=(0.360, -0.220, 0.013)),
        material=satin_steel,
        name="rear_reinforcement_bow",
    )
    glass_panel.visual(
        Box((0.700, 0.015, 0.008)),
        origin=Origin(xyz=(0.360, 0.0, 0.011)),
        material=rubber_seal,
        name="underside_center_seal",
    )
    glass_panel.inertial = Inertial.from_geometry(
        Box((0.78, 0.58, 0.06)),
        mass=5.8,
        origin=Origin(xyz=(0.360, 0.0, 0.020)),
    )

    model.articulation(
        "panel_slide",
        ArticulationType.PRISMATIC,
        parent="cassette_frame",
        child="glass_panel",
        origin=Origin(xyz=(-0.360, opening_center_y, 0.014)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.6,
            lower=0.0,
            upper=0.34,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=144,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("glass_panel", "cassette_frame", axes="xy", min_overlap=0.45)
    ctx.expect_aabb_gap("glass_panel", "cassette_frame", axis="z", max_gap=0.010, max_penetration=0.030)
    ctx.expect_joint_motion_axis(
        "panel_slide",
        "glass_panel",
        world_axis="y",
        direction="negative",
        min_delta=0.12,
    )

    with ctx.pose(panel_slide=0.17):
        ctx.expect_aabb_overlap("glass_panel", "cassette_frame", axes="xy", min_overlap=0.44)
        ctx.expect_aabb_gap("glass_panel", "cassette_frame", axis="z", max_gap=0.012, max_penetration=0.030)

    with ctx.pose(panel_slide=0.34):
        ctx.expect_aabb_overlap("glass_panel", "cassette_frame", axes="xy", min_overlap=0.38)
        ctx.expect_aabb_gap("glass_panel", "cassette_frame", axis="z", max_gap=0.014, max_penetration=0.030)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
