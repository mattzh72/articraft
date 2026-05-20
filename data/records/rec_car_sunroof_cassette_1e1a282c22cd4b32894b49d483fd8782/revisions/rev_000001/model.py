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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="popup_tilt_sunroof_cassette")

    aluminum = model.material("aluminum", rgba=(0.60, 0.62, 0.64, 1.0))
    black_frame = model.material("black_frame", rgba=(0.12, 0.13, 0.14, 1.0))
    seal_black = model.material("seal_black", rgba=(0.05, 0.05, 0.05, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.20, 0.28, 0.33, 0.42))
    ceramic_band = model.material("ceramic_band", rgba=(0.06, 0.06, 0.07, 1.0))

    cassette = model.part("cassette_frame")
    cassette.inertial = Inertial.from_geometry(
        Box((0.90, 0.62, 0.07)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    cassette.visual(
        _mesh(
            "cassette_top_ring",
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.90, 0.62, 0.030, corner_segments=8),
                [rounded_rect_profile(0.76, 0.46, 0.020, corner_segments=8)],
                height=0.012,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=black_frame,
        name="cassette_top_ring",
    )
    cassette.visual(
        Box((0.052, 0.500, 0.042)),
        origin=Origin(xyz=(-0.386, 0.0, 0.027)),
        material=black_frame,
        name="left_tray_wall",
    )
    cassette.visual(
        Box((0.052, 0.500, 0.042)),
        origin=Origin(xyz=(0.386, 0.0, 0.027)),
        material=black_frame,
        name="right_tray_wall",
    )
    cassette.visual(
        Box((0.736, 0.050, 0.024)),
        origin=Origin(xyz=(0.0, 0.245, 0.012)),
        material=black_frame,
        name="rear_tray_bridge",
    )
    cassette.visual(
        Box((0.84, 0.086, 0.030)),
        origin=Origin(xyz=(0.0, -0.272, 0.015)),
        material=black_frame,
        name="front_housing",
    )
    cassette.visual(
        Box((0.84, 0.032, 0.014)),
        origin=Origin(xyz=(0.0, -0.322, 0.007)),
        material=black_frame,
        name="deflector_pocket",
    )
    cassette.visual(
        Box((0.84, 0.060, 0.028)),
        origin=Origin(xyz=(0.0, 0.272, 0.018)),
        material=black_frame,
        name="rear_channel",
    )
    cassette.visual(
        Box((0.060, 0.440, 0.016)),
        origin=Origin(xyz=(-0.400, 0.0, 0.040)),
        material=aluminum,
        name="left_guide_rail",
    )
    cassette.visual(
        Box((0.060, 0.440, 0.016)),
        origin=Origin(xyz=(0.400, 0.0, 0.040)),
        material=aluminum,
        name="right_guide_rail",
    )
    cassette.visual(
        Box((0.760, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, -0.228, 0.059)),
        material=seal_black,
        name="front_seal_land",
    )
    cassette.visual(
        Box((0.050, 0.048, 0.018)),
        origin=Origin(xyz=(-0.366, -0.218, 0.039)),
        material=black_frame,
        name="left_front_mount",
    )
    cassette.visual(
        Box((0.050, 0.048, 0.018)),
        origin=Origin(xyz=(0.366, -0.218, 0.039)),
        material=black_frame,
        name="right_front_mount",
    )
    cassette.visual(
        Box((0.020, 0.020, 0.022)),
        origin=Origin(xyz=(-0.332, -0.316, 0.025)),
        material=black_frame,
        name="left_deflector_support",
    )
    cassette.visual(
        Box((0.020, 0.020, 0.022)),
        origin=Origin(xyz=(0.332, -0.316, 0.025)),
        material=black_frame,
        name="right_deflector_support",
    )

    glass = model.part("glass_panel")
    glass.inertial = Inertial.from_geometry(
        Box((0.81, 0.51, 0.024)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.255, 0.006)),
    )
    glass.visual(
        _mesh(
            "sunroof_glass_panel",
            ExtrudeGeometry(
                rounded_rect_profile(0.804, 0.504, 0.022, corner_segments=8),
                height=0.005,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.252, 0.006)),
        material=glass_tint,
        name="glass_glazing",
    )
    glass.visual(
        _mesh(
            "sunroof_glass_border",
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.800, 0.500, 0.020, corner_segments=8),
                [rounded_rect_profile(0.710, 0.410, 0.016, corner_segments=8)],
                height=0.0018,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.252, 0.0040)),
        material=ceramic_band,
        name="glass_ceramic_band",
    )
    glass.visual(
        Box((0.640, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.024, -0.001)),
        material=black_frame,
        name="front_carrier_beam",
    )
    glass.visual(
        Box((0.052, 0.026, 0.016)),
        origin=Origin(xyz=(-0.300, 0.008, -0.008)),
        material=black_frame,
        name="left_pivot_shoe",
    )
    glass.visual(
        Box((0.052, 0.026, 0.016)),
        origin=Origin(xyz=(0.300, 0.008, -0.008)),
        material=black_frame,
        name="right_pivot_shoe",
    )
    glass.visual(
        Box((0.760, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.494, 0.0035)),
        material=seal_black,
        name="rear_edge_seal",
    )

    deflector = model.part("wind_deflector")
    deflector.inertial = Inertial.from_geometry(
        Box((0.72, 0.05, 0.03)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.020, 0.010)),
    )
    deflector.visual(
        Cylinder(radius=0.005, length=0.700),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=black_frame,
        name="deflector_hinge_rod",
    )
    deflector.visual(
        Box((0.700, 0.042, 0.003)),
        origin=Origin(xyz=(0.0, -0.021, 0.0070)),
        material=black_frame,
        name="deflector_blade",
    )
    deflector.visual(
        Box((0.016, 0.040, 0.0105)),
        origin=Origin(xyz=(-0.332, -0.020, 0.00025)),
        material=black_frame,
        name="left_deflector_tab",
    )
    deflector.visual(
        Box((0.016, 0.040, 0.0105)),
        origin=Origin(xyz=(0.332, -0.020, 0.00025)),
        material=black_frame,
        name="right_deflector_tab",
    )

    model.articulation(
        "glass_tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=cassette,
        child=glass,
        origin=Origin(xyz=(0.0, -0.224, 0.059)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=0.0, upper=0.11),
    )
    model.articulation(
        "deflector_hinge",
        ArticulationType.REVOLUTE,
        parent=cassette,
        child=deflector,
        origin=Origin(xyz=(0.0, -0.316, 0.041)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=0.58),
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

    cassette = object_model.get_part("cassette_frame")
    glass = object_model.get_part("glass_panel")
    deflector = object_model.get_part("wind_deflector")
    glass_hinge = object_model.get_articulation("glass_tilt_hinge")
    deflector_hinge = object_model.get_articulation("deflector_hinge")

    with ctx.pose({glass_hinge: 0.0, deflector_hinge: 0.0}):
        ctx.expect_gap(
            glass,
            cassette,
            axis="z",
            positive_elem="glass_glazing",
            negative_elem="cassette_top_ring",
            min_gap=0.002,
            max_gap=0.008,
            name="closed glass sits just above cassette frame",
        )
        ctx.expect_overlap(
            glass,
            cassette,
            axes="xy",
            elem_a="glass_glazing",
            elem_b="cassette_top_ring",
            min_overlap=0.30,
            name="glass covers the cassette opening in plan",
        )
        ctx.expect_gap(
            glass,
            deflector,
            axis="z",
            positive_elem="glass_glazing",
            negative_elem="deflector_blade",
            min_gap=0.003,
            max_gap=0.015,
            name="stowed deflector clears the closed glass",
        )
        closed_glass_aabb = ctx.part_element_world_aabb(glass, elem="glass_glazing")
        closed_deflector_aabb = ctx.part_element_world_aabb(deflector, elem="deflector_blade")

    with ctx.pose({glass_hinge: 0.11, deflector_hinge: 0.58}):
        open_glass_aabb = ctx.part_element_world_aabb(glass, elem="glass_glazing")
        open_deflector_aabb = ctx.part_element_world_aabb(deflector, elem="deflector_blade")
        ctx.expect_gap(
            glass,
            cassette,
            axis="z",
            positive_elem="glass_glazing",
            negative_elem="cassette_top_ring",
            min_gap=0.002,
            name="opened glass remains above the cassette frame",
        )

    ctx.check(
        "rear edge of glass lifts upward",
        closed_glass_aabb is not None
        and open_glass_aabb is not None
        and open_glass_aabb[1][2] > closed_glass_aabb[1][2] + 0.040,
        details=f"closed={closed_glass_aabb}, open={open_glass_aabb}",
    )
    ctx.check(
        "wind deflector rises when the glass tilts",
        closed_deflector_aabb is not None
        and open_deflector_aabb is not None
        and open_deflector_aabb[1][2] > closed_deflector_aabb[1][2] + 0.018,
        details=f"closed={closed_deflector_aabb}, open={open_deflector_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
