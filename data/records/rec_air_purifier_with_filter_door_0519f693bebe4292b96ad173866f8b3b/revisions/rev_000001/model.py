from __future__ import annotations

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_cassette_air_purifier")

    ceiling = model.part("ceiling_plate")
    housing = model.part("housing")
    grille = model.part("front_grille")
    filter_block = model.part("filter_block")

    warm_white = model.material("powder_coated_warm_white", rgba=(0.86, 0.86, 0.80, 1.0))
    ceiling_white = model.material("matte_ceiling_white", rgba=(0.93, 0.93, 0.90, 1.0))
    shadow_gray = model.material("shadowed_dark_gray", rgba=(0.11, 0.12, 0.13, 1.0))
    filter_media = model.material("pleated_filter_media", rgba=(0.78, 0.74, 0.62, 1.0))
    rail_metal = model.material("galvanized_guide_metal", rgba=(0.55, 0.57, 0.58, 1.0))
    gasket_black = model.material("black_rubber_gasket", rgba=(0.025, 0.025, 0.025, 1.0))

    # A thin plate represents the finished ceiling plane; the purifier cassette
    # housing is mounted directly against its underside.
    ceiling.visual(
        Box((1.04, 0.70, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=ceiling_white,
        name="ceiling_tile",
    )

    # Shallow rectangular cassette body, open at the service grille.  Side walls,
    # a rear wall, an upper front fascia and lower flanges make the housing read
    # as a formed sheet-metal enclosure rather than a solid block.
    housing.visual(
        Box((0.86, 0.52, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=warm_white,
        name="top_pan",
    )
    housing.visual(
        Box((0.025, 0.52, 0.150)),
        origin=Origin(xyz=(-0.4175, 0.0, -0.085)),
        material=warm_white,
        name="side_wall_0",
    )
    housing.visual(
        Box((0.025, 0.52, 0.150)),
        origin=Origin(xyz=(0.4175, 0.0, -0.085)),
        material=warm_white,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.86, 0.025, 0.150)),
        origin=Origin(xyz=(0.0, 0.2475, -0.085)),
        material=warm_white,
        name="rear_wall",
    )
    housing.visual(
        Box((0.86, 0.025, 0.070)),
        origin=Origin(xyz=(0.0, -0.2475, -0.035)),
        material=warm_white,
        name="front_fascia",
    )
    housing.visual(
        Box((0.86, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.245, -0.154)),
        material=warm_white,
        name="rear_base_lip",
    )
    housing.visual(
        Box((0.030, 0.52, 0.012)),
        origin=Origin(xyz=(-0.405, 0.0, -0.154)),
        material=warm_white,
        name="side_base_lip_0",
    )
    housing.visual(
        Box((0.030, 0.52, 0.012)),
        origin=Origin(xyz=(0.405, 0.0, -0.154)),
        material=warm_white,
        name="side_base_lip_1",
    )
    housing.visual(
        Box((0.135, 0.020, 0.014)),
        origin=Origin(xyz=(-0.365, -0.248, -0.151)),
        material=warm_white,
        name="front_hinge_lip_0",
    )
    housing.visual(
        Box((0.135, 0.020, 0.014)),
        origin=Origin(xyz=(0.365, -0.248, -0.151)),
        material=warm_white,
        name="front_hinge_lip_1",
    )

    # Two metal guide rails at the base of the housing support the sliding filter
    # tray.  They are wide enough to touch the side walls and therefore read as
    # mounted channels, not loose strips.
    housing.visual(
        Box((0.090, 0.470, 0.015)),
        origin=Origin(xyz=(-0.360, 0.0, -0.145)),
        material=rail_metal,
        name="guide_rail_0",
    )
    housing.visual(
        Box((0.090, 0.470, 0.015)),
        origin=Origin(xyz=(0.360, 0.0, -0.145)),
        material=rail_metal,
        name="guide_rail_1",
    )

    # Stationary hinge knuckles carried by the side walls.  The grille owns the
    # alternate center knuckles, leaving visible hinge gaps instead of collisions.
    for x in (-0.340, 0.340):
        housing.visual(
            Cylinder(radius=0.012, length=0.180),
            origin=Origin(xyz=(x, -0.252, -0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rail_metal,
            name=f"hinge_knuckle_{'0' if x < 0 else '1'}",
        )

    model.articulation(
        "ceiling_to_housing",
        ArticulationType.FIXED,
        parent=ceiling,
        child=housing,
        origin=Origin(),
    )

    # The drop-down access grille is framed rather than a solid panel.  Its part
    # frame sits on the hinge pin line; the frame and louvers extend rearward
    # along +Y when closed.
    grille.visual(
        Box((0.780, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.027, -0.012)),
        material=warm_white,
        name="front_bar",
    )
    grille.visual(
        Box((0.780, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.465, -0.012)),
        material=warm_white,
        name="rear_bar",
    )
    grille.visual(
        Box((0.035, 0.465, 0.018)),
        origin=Origin(xyz=(-0.3725, 0.246, -0.012)),
        material=warm_white,
        name="side_bar_0",
    )
    grille.visual(
        Box((0.035, 0.465, 0.018)),
        origin=Origin(xyz=(0.3725, 0.246, -0.012)),
        material=warm_white,
        name="side_bar_1",
    )
    grille.visual(
        Box((0.030, 0.380, 0.014)),
        origin=Origin(xyz=(0.0, 0.246, -0.014)),
        material=warm_white,
        name="center_mullion",
    )
    for i, y in enumerate((0.085, 0.125, 0.165, 0.205, 0.245, 0.285, 0.325, 0.365, 0.405)):
        grille.visual(
            Box((0.705, 0.014, 0.012)),
            origin=Origin(xyz=(0.0, y, -0.014), rpy=(-0.22, 0.0, 0.0)),
            material=warm_white,
            name=f"louver_{i}",
        )
    grille.visual(
        Box((0.725, 0.440, 0.004)),
        origin=Origin(xyz=(0.0, 0.246, -0.020)),
        material=shadow_gray,
        name="dark_opening_shadow",
    )
    grille.visual(
        Box((0.720, 0.430, 0.003)),
        origin=Origin(xyz=(0.0, 0.246, -0.004)),
        material=gasket_black,
        name="perimeter_gasket",
    )
    for i, x in enumerate((-0.165, 0.165)):
        grille.visual(
            Box((0.050, 0.016, 0.010)),
            origin=Origin(xyz=(x, 0.014, -0.010)),
            material=rail_metal,
            name=f"knuckle_web_{i}",
        )
        grille.visual(
            Cylinder(radius=0.0105, length=0.170),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rail_metal,
            name=f"grille_knuckle_{i}",
        )

    model.articulation(
        "housing_to_front_grille",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=grille,
        origin=Origin(xyz=(0.0, -0.252, -0.160)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.30),
    )

    # Sliding filter cartridge: a shallow tray with pleated media and side
    # runners that sit on the metal guide rails.
    filter_block.visual(
        Box((0.610, 0.405, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=filter_media,
        name="filter_core",
    )
    filter_block.visual(
        Box((0.660, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, -0.2225, 0.0)),
        material=warm_white,
        name="front_filter_frame",
    )
    filter_block.visual(
        Box((0.660, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, 0.2225, 0.0)),
        material=warm_white,
        name="rear_filter_frame",
    )
    filter_block.visual(
        Box((0.030, 0.405, 0.055)),
        origin=Origin(xyz=(-0.330, 0.0, 0.0)),
        material=warm_white,
        name="side_filter_frame_0",
    )
    filter_block.visual(
        Box((0.030, 0.405, 0.055)),
        origin=Origin(xyz=(0.330, 0.0, 0.0)),
        material=warm_white,
        name="side_filter_frame_1",
    )
    for i, y in enumerate((-0.155, -0.105, -0.055, -0.005, 0.045, 0.095, 0.145)):
        filter_block.visual(
            Box((0.565, 0.012, 0.050)),
            origin=Origin(xyz=(0.0, y, 0.006), rpy=(0.0, 0.0, 0.0)),
            material=filter_media,
            name=f"pleat_{i}",
        )
    filter_block.visual(
        Box((0.030, 0.430, 0.012)),
        origin=Origin(xyz=(-0.310, 0.0, -0.0275)),
        material=rail_metal,
        name="runner_0",
    )
    filter_block.visual(
        Box((0.030, 0.430, 0.012)),
        origin=Origin(xyz=(0.310, 0.0, -0.0275)),
        material=rail_metal,
        name="runner_1",
    )
    filter_block.visual(
        Box((0.180, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.246, -0.002)),
        material=gasket_black,
        name="pull_grip",
    )

    model.articulation(
        "housing_to_filter_block",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_block,
        origin=Origin(xyz=(0.0, 0.0, -0.1040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.20, lower=0.0, upper=0.320),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ceiling = object_model.get_part("ceiling_plate")
    housing = object_model.get_part("housing")
    grille = object_model.get_part("front_grille")
    filter_block = object_model.get_part("filter_block")
    grille_hinge = object_model.get_articulation("housing_to_front_grille")
    filter_slide = object_model.get_articulation("housing_to_filter_block")

    ctx.expect_gap(
        ceiling,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="ceiling_tile",
        negative_elem="top_pan",
        name="housing mounts flush to ceiling plate",
    )
    ctx.expect_gap(
        housing,
        grille,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem="rear_base_lip",
        negative_elem="perimeter_gasket",
        name="closed grille seats just below base lip",
    )
    ctx.expect_overlap(
        grille,
        housing,
        axes="x",
        min_overlap=0.70,
        elem_a="rear_bar",
        elem_b="rear_base_lip",
        name="closed grille spans the service opening",
    )

    closed_grille_aabb = ctx.part_world_aabb(grille)
    with ctx.pose({grille_hinge: 1.20}):
        opened_grille_aabb = ctx.part_world_aabb(grille)
    ctx.check(
        "front grille drops downward on front hinge",
        closed_grille_aabb is not None
        and opened_grille_aabb is not None
        and opened_grille_aabb[0][2] < closed_grille_aabb[0][2] - 0.22,
        details=f"closed={closed_grille_aabb}, opened={opened_grille_aabb}",
    )

    ctx.expect_gap(
        filter_block,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="runner_0",
        negative_elem="guide_rail_0",
        name="filter runner rests on guide rail",
    )
    ctx.expect_overlap(
        filter_block,
        housing,
        axes="xy",
        min_overlap=0.008,
        elem_a="runner_0",
        elem_b="guide_rail_0",
        name="filter runner is captured over guide rail",
    )
    ctx.expect_overlap(
        filter_block,
        housing,
        axes="y",
        min_overlap=0.35,
        elem_a="runner_0",
        elem_b="guide_rail_0",
        name="closed filter remains deeply inserted in rails",
    )

    closed_filter_aabb = ctx.part_world_aabb(filter_block)
    with ctx.pose({grille_hinge: 1.20, filter_slide: 0.320}):
        extended_filter_aabb = ctx.part_world_aabb(filter_block)
        ctx.expect_overlap(
            filter_block,
            housing,
            axes="y",
            min_overlap=0.10,
            elem_a="runner_0",
            elem_b="guide_rail_0",
            name="extended filter keeps retained rail engagement",
        )
    ctx.check(
        "filter slides out toward the front service edge",
        closed_filter_aabb is not None
        and extended_filter_aabb is not None
        and extended_filter_aabb[0][1] < closed_filter_aabb[0][1] - 0.25,
        details=f"closed={closed_filter_aabb}, extended={extended_filter_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
