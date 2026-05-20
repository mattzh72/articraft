from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_panel_panoramic_sunroof_cassette")

    anodized_black = model.material("anodized_black", rgba=(0.13, 0.13, 0.14, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.31, 0.33, 0.35, 1.0))
    tray_black = model.material("tray_black", rgba=(0.18, 0.19, 0.20, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.20, 0.30, 0.34, 0.45))

    cassette = model.part("cassette")
    cassette.visual(
        Box((1.62, 1.10, 0.012)),
        origin=Origin(xyz=(0.09, 0.0, 0.006)),
        material=tray_black,
        name="tray_base",
    )
    cassette.visual(
        Box((1.62, 0.090, 0.075)),
        origin=Origin(xyz=(0.09, 0.505, 0.0475)),
        material=anodized_black,
        name="left_side_rail",
    )
    cassette.visual(
        Box((1.62, 0.090, 0.075)),
        origin=Origin(xyz=(0.09, -0.505, 0.0475)),
        material=anodized_black,
        name="right_side_rail",
    )
    cassette.visual(
        Box((0.130, 1.10, 0.075)),
        origin=Origin(xyz=(-0.655, 0.0, 0.0475)),
        material=anodized_black,
        name="front_header",
    )
    cassette.visual(
        Box((1.54, 0.032, 0.028)),
        origin=Origin(xyz=(0.12, 0.434, 0.025)),
        material=rail_steel,
        name="left_track_ledge",
    )
    cassette.visual(
        Box((1.54, 0.032, 0.028)),
        origin=Origin(xyz=(0.12, -0.434, 0.025)),
        material=rail_steel,
        name="right_track_ledge",
    )
    cassette.visual(
        Box((0.56, 0.96, 0.024)),
        origin=Origin(xyz=(0.62, 0.0, 0.018)),
        material=tray_black,
        name="rear_stowage_floor",
    )
    cassette.visual(
        Box((0.080, 0.96, 0.040)),
        origin=Origin(xyz=(0.86, 0.0, 0.020)),
        material=anodized_black,
        name="rear_stop_wall",
    )

    aperture_outer = rounded_rect_profile(1.42, 1.10, 0.060, corner_segments=10)
    aperture_hole = rounded_rect_profile(1.34, 0.98, 0.045, corner_segments=10)
    cassette.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                aperture_outer,
                [aperture_hole],
                height=0.010,
                center=True,
            ),
            "sunroof_cassette_top_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=seal_rubber,
        name="aperture_frame",
    )
    cassette.inertial = Inertial.from_geometry(
        Box((1.62, 1.10, 0.10)),
        mass=22.0,
        origin=Origin(xyz=(0.09, 0.0, 0.050)),
    )

    front_panel = model.part("front_panel")
    front_panel.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.66, 0.90, 0.038, corner_segments=10),
                0.010,
            ),
            "front_sunroof_panel_slab",
        ),
        material=anodized_black,
        name="front_slab",
    )
    front_panel.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.59, 0.83, 0.034, corner_segments=10),
                0.005,
            ),
            "front_sunroof_panel_glass",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=glass_tint,
        name="front_glass",
    )
    front_panel.visual(
        Box((0.52, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, 0.407, -0.005)),
        material=rail_steel,
        name="front_left_carriage",
    )
    front_panel.visual(
        Box((0.52, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, -0.407, -0.005)),
        material=rail_steel,
        name="front_right_carriage",
    )
    front_panel.inertial = Inertial.from_geometry(
        Box((0.66, 0.90, 0.012)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    rear_panel = model.part("rear_panel")
    rear_panel.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.56, 0.90, 0.038, corner_segments=10),
                0.010,
            ),
            "rear_sunroof_panel_slab",
        ),
        material=anodized_black,
        name="rear_slab",
    )
    rear_panel.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.49, 0.83, 0.034, corner_segments=10),
                0.005,
            ),
            "rear_sunroof_panel_glass",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=glass_tint,
        name="rear_glass",
    )
    rear_panel.visual(
        Box((0.42, 0.022, 0.046)),
        origin=Origin(xyz=(0.0, 0.407, -0.013)),
        material=rail_steel,
        name="rear_left_carriage",
    )
    rear_panel.visual(
        Box((0.42, 0.022, 0.046)),
        origin=Origin(xyz=(0.0, -0.407, -0.013)),
        material=rail_steel,
        name="rear_right_carriage",
    )
    rear_panel.inertial = Inertial.from_geometry(
        Box((0.56, 0.90, 0.012)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    model.articulation(
        "cassette_to_front_panel",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=front_panel,
        origin=Origin(xyz=(-0.26, 0.0, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.40,
            lower=0.0,
            upper=0.46,
        ),
    )
    model.articulation(
        "cassette_to_rear_panel",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=rear_panel,
        origin=Origin(xyz=(0.38, 0.0, 0.062)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    cassette = object_model.get_part("cassette")
    front_panel = object_model.get_part("front_panel")
    rear_panel = object_model.get_part("rear_panel")
    front_slide = object_model.get_articulation("cassette_to_front_panel")
    rear_slide = object_model.get_articulation("cassette_to_rear_panel")

    with ctx.pose({front_slide: 0.0, rear_slide: 0.0}):
        ctx.expect_gap(
            rear_panel,
            front_panel,
            axis="x",
            min_gap=0.015,
            max_gap=0.035,
            positive_elem="rear_slab",
            negative_elem="front_slab",
            name="closed panels keep a narrow front-to-rear seam",
        )
        ctx.expect_gap(
            front_panel,
            cassette,
            axis="z",
            min_gap=0.030,
            max_gap=0.060,
            positive_elem="front_slab",
            negative_elem="tray_base",
            name="front panel rides above cassette tray",
        )
        ctx.expect_gap(
            rear_panel,
            cassette,
            axis="z",
            min_gap=0.040,
            max_gap=0.070,
            positive_elem="rear_slab",
            negative_elem="tray_base",
            name="rear panel rides above cassette tray",
        )

    front_rest = ctx.part_world_position(front_panel)
    with ctx.pose({front_slide: 0.46, rear_slide: 0.0}):
        front_open = ctx.part_world_position(front_panel)
        ctx.expect_overlap(
            front_panel,
            rear_panel,
            axes="x",
            min_overlap=0.38,
            elem_a="front_slab",
            elem_b="rear_slab",
            name="front panel retracts under the rear panel footprint",
        )
        ctx.expect_gap(
            rear_panel,
            front_panel,
            axis="z",
            min_gap=0.003,
            max_gap=0.020,
            positive_elem="rear_slab",
            negative_elem="front_slab",
            name="front panel stays below rear panel when retracted",
        )
    ctx.check(
        "front panel moves rearward",
        front_rest is not None and front_open is not None and front_open[0] > front_rest[0] + 0.30,
        details=f"rest={front_rest}, open={front_open}",
    )

    rear_rest = ctx.part_world_position(rear_panel)
    with ctx.pose({front_slide: 0.0, rear_slide: 0.28}):
        rear_open = ctx.part_world_position(rear_panel)
        ctx.expect_overlap(
            rear_panel,
            cassette,
            axes="x",
            min_overlap=0.40,
            elem_a="rear_slab",
            elem_b="rear_stowage_floor",
            name="rear panel remains captured over the stowage pocket",
        )
    ctx.check(
        "rear panel moves rearward",
        rear_rest is not None and rear_open is not None and rear_open[0] > rear_rest[0] + 0.20,
        details=f"rest={rear_rest}, open={rear_open}",
    )

    with ctx.pose({front_slide: 0.46, rear_slide: 0.28}):
        ctx.expect_overlap(
            front_panel,
            rear_panel,
            axes="x",
            min_overlap=0.10,
            elem_a="front_slab",
            elem_b="rear_slab",
            name="both panels can occupy their independent open positions together",
        )
        ctx.expect_gap(
            rear_panel,
            front_panel,
            axis="z",
            min_gap=0.003,
            max_gap=0.020,
            positive_elem="rear_slab",
            negative_elem="front_slab",
            name="simultaneous open pose keeps the stacked panel clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
