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
    model = ArticulatedObject(name="wall_hung_air_purifier")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.88, 0.86, 1.0))
    satin_gray = model.material("satin_gray_metal", rgba=(0.42, 0.44, 0.46, 1.0))
    dark_shadow = model.material("dark_vent_shadow", rgba=(0.035, 0.040, 0.045, 1.0))
    filter_media = model.material("pleated_filter_media", rgba=(0.78, 0.86, 0.88, 1.0))
    rubber = model.material("dark_rubber_gasket", rgba=(0.02, 0.022, 0.024, 1.0))

    bracket = model.part("wall_bracket")
    # A rigid open wall bracket: side rails plus top/bottom ties leave a clear
    # rear service window for the filter cartridge to slide through.
    bracket.visual(
        Box((0.68, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, 0.135, 0.355)),
        material=satin_gray,
        name="top_wall_rail",
    )
    bracket.visual(
        Box((0.68, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, 0.135, -0.355)),
        material=satin_gray,
        name="bottom_wall_rail",
    )
    for x, name in [(-0.3225, "side_wall_rail_0"), (0.3225, "side_wall_rail_1")]:
        bracket.visual(
            Box((0.035, 0.018, 0.72)),
            origin=Origin(xyz=(x, 0.135, 0.0)),
            material=satin_gray,
            name=name,
        )
    bracket.visual(
        Box((0.055, 0.054, 0.060)),
        origin=Origin(xyz=(-0.2775, 0.102, 0.240)),
        material=satin_gray,
        name="hanger_pad_0",
    )
    bracket.visual(
        Box((0.055, 0.054, 0.060)),
        origin=Origin(xyz=(0.2775, 0.102, 0.240)),
        material=satin_gray,
        name="hanger_pad_1",
    )
    bracket.visual(
        Box((0.055, 0.054, 0.060)),
        origin=Origin(xyz=(-0.2775, 0.102, -0.240)),
        material=satin_gray,
        name="hanger_pad_2",
    )
    bracket.visual(
        Box((0.055, 0.054, 0.060)),
        origin=Origin(xyz=(0.2775, 0.102, -0.240)),
        material=satin_gray,
        name="hanger_pad_3",
    )
    for x, z, name in [
        (-0.24, 0.355, "screw_head_0"),
        (0.24, 0.355, "screw_head_1"),
        (-0.24, -0.355, "screw_head_2"),
        (0.24, -0.355, "screw_head_3"),
    ]:
        bracket.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, 0.124, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_shadow,
            name=name,
        )

    housing = model.part("housing")
    housing.visual(
        Box((0.025, 0.150, 0.780)),
        origin=Origin(xyz=(-0.2775, 0.0, 0.0)),
        material=warm_white,
        name="side_wall_0",
    )
    housing.visual(
        Box((0.025, 0.150, 0.780)),
        origin=Origin(xyz=(0.2775, 0.0, 0.0)),
        material=warm_white,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.580, 0.150, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.3775)),
        material=warm_white,
        name="top_wall",
    )
    housing.visual(
        Box((0.580, 0.150, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.3775)),
        material=warm_white,
        name="bottom_wall",
    )
    housing.visual(
        Box((0.500, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, -0.081, 0.352)),
        material=rubber,
        name="front_gasket_top",
    )
    housing.visual(
        Box((0.500, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, -0.081, -0.352)),
        material=rubber,
        name="front_gasket_bottom",
    )
    housing.visual(
        Box((0.014, 0.135, 0.730)),
        origin=Origin(xyz=(-0.232, 0.0075, 0.0)),
        material=satin_gray,
        name="rear_guide_rail_0",
    )
    housing.visual(
        Box((0.014, 0.135, 0.730)),
        origin=Origin(xyz=(0.232, 0.0075, 0.0)),
        material=satin_gray,
        name="rear_guide_rail_1",
    )
    for i, z in enumerate([-0.180, -0.120, -0.060, 0.000, 0.060, 0.120, 0.180]):
        housing.visual(
            Box((0.006, 0.075, 0.014)),
            origin=Origin(xyz=(0.293, -0.005, z)),
            material=dark_shadow,
            name=f"side_vent_{i}",
        )

    # Two exposed side knuckle hinges.  Each hinge has fixed housing knuckle
    # segments above and below the moving cover knuckle, with small axial gaps.
    hinge_x = -0.305
    hinge_y = -0.099
    housing.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.278)),
        material=satin_gray,
        name="upper_housing_knuckle_top",
    )
    housing.visual(
        Box((0.038, 0.026, 0.034)),
        origin=Origin(xyz=(hinge_x + 0.011, -0.086, 0.278)),
        material=satin_gray,
        name="upper_hinge_leaf_top",
    )
    housing.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.162)),
        material=satin_gray,
        name="upper_housing_knuckle_bottom",
    )
    housing.visual(
        Box((0.038, 0.026, 0.034)),
        origin=Origin(xyz=(hinge_x + 0.011, -0.086, 0.162)),
        material=satin_gray,
        name="upper_hinge_leaf_bottom",
    )
    housing.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(hinge_x, hinge_y, -0.162)),
        material=satin_gray,
        name="lower_housing_knuckle_top",
    )
    housing.visual(
        Box((0.038, 0.026, 0.034)),
        origin=Origin(xyz=(hinge_x + 0.011, -0.086, -0.162)),
        material=satin_gray,
        name="lower_hinge_leaf_top",
    )
    housing.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(hinge_x, hinge_y, -0.278)),
        material=satin_gray,
        name="lower_housing_knuckle_bottom",
    )
    housing.visual(
        Box((0.038, 0.026, 0.034)),
        origin=Origin(xyz=(hinge_x + 0.011, -0.086, -0.278)),
        material=satin_gray,
        name="lower_hinge_leaf_bottom",
    )

    model.articulation(
        "bracket_to_housing",
        ArticulationType.FIXED,
        parent=bracket,
        child=housing,
        origin=Origin(),
    )

    cover = model.part("front_cover")
    cover.visual(
        Box((0.550, 0.016, 0.700)),
        origin=Origin(xyz=(0.305, 0.0, 0.0)),
        material=warm_white,
        name="front_panel",
    )
    cover.visual(
        Box((0.505, 0.006, 0.555)),
        origin=Origin(xyz=(0.330, -0.011, 0.0)),
        material=dark_shadow,
        name="recessed_intake_shadow",
    )
    for i, z in enumerate([-0.230, -0.165, -0.100, -0.035, 0.030, 0.095, 0.160, 0.225]):
        cover.visual(
            Box((0.430, 0.010, 0.018)),
            origin=Origin(xyz=(0.355, -0.013, z)),
            material=warm_white,
            name=f"front_louver_{i}",
        )
    cover.visual(
        Box((0.020, 0.012, 0.520)),
        origin=Origin(xyz=(0.565, -0.014, 0.0)),
        material=dark_shadow,
        name="pull_shadow",
    )
    cover.visual(
        Box((0.040, 0.012, 0.082)),
        origin=Origin(xyz=(0.029, 0.0, 0.220)),
        material=satin_gray,
        name="upper_hinge_strap",
    )
    cover.visual(
        Box((0.040, 0.012, 0.082)),
        origin=Origin(xyz=(0.029, 0.0, -0.220)),
        material=satin_gray,
        name="lower_hinge_strap",
    )
    cover.visual(
        Cylinder(radius=0.014, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=satin_gray,
        name="upper_cover_knuckle",
    )
    cover.visual(
        Cylinder(radius=0.014, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        material=satin_gray,
        name="lower_cover_knuckle",
    )
    model.articulation(
        "housing_to_front_cover",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cover,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.4, lower=0.0, upper=1.45),
    )

    cartridge = model.part("filter_cartridge")
    cartridge.visual(
        Box((0.360, 0.050, 0.500)),
        origin=Origin(xyz=(0.0, -0.025, 0.0)),
        material=filter_media,
        name="filter_media",
    )
    cartridge.visual(
        Box((0.420, 0.058, 0.030)),
        origin=Origin(xyz=(0.0, -0.025, 0.265)),
        material=satin_gray,
        name="filter_frame_top",
    )
    cartridge.visual(
        Box((0.420, 0.058, 0.030)),
        origin=Origin(xyz=(0.0, -0.025, -0.265)),
        material=satin_gray,
        name="filter_frame_bottom",
    )
    for x, name in [(-0.210, "filter_frame_side_0"), (0.210, "filter_frame_side_1")]:
        cartridge.visual(
            Box((0.030, 0.058, 0.560)),
            origin=Origin(xyz=(x, -0.025, 0.0)),
            material=satin_gray,
            name=name,
        )
    cartridge.visual(
        Box((0.010, 0.150, 0.520)),
        origin=Origin(xyz=(-0.218, -0.075, 0.0)),
        material=satin_gray,
        name="side_runner_0",
    )
    cartridge.visual(
        Box((0.010, 0.150, 0.520)),
        origin=Origin(xyz=(0.218, -0.075, 0.0)),
        material=satin_gray,
        name="side_runner_1",
    )
    for i, x in enumerate([-0.135, -0.090, -0.045, 0.000, 0.045, 0.090, 0.135]):
        cartridge.visual(
            Box((0.010, 0.052, 0.490)),
            origin=Origin(xyz=(x, -0.054, 0.0)),
            material=(filter_media if i % 2 == 0 else warm_white),
            name=f"filter_pleat_{i}",
        )
    cartridge.visual(
        Box((0.220, 0.024, 0.045)),
        origin=Origin(xyz=(0.0, 0.000, 0.0)),
        material=dark_shadow,
        name="rear_pull_tab",
    )
    model.articulation(
        "housing_to_filter_cartridge",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=cartridge,
        origin=Origin(xyz=(0.0, 0.083, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.25, lower=0.0, upper=0.090),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("wall_bracket")
    housing = object_model.get_part("housing")
    cover = object_model.get_part("front_cover")
    cartridge = object_model.get_part("filter_cartridge")
    cover_hinge = object_model.get_articulation("housing_to_front_cover")
    filter_slide = object_model.get_articulation("housing_to_filter_cartridge")

    # The air purifier is wall-hung: the bracket pads visibly seat against the
    # housing rear side walls instead of leaving the appliance floating.
    ctx.expect_contact(
        bracket,
        housing,
        elem_a="hanger_pad_0",
        elem_b="side_wall_0",
        contact_tol=1e-5,
        name="left upper hanger pad seats on housing",
    )
    ctx.expect_contact(
        bracket,
        housing,
        elem_a="hanger_pad_1",
        elem_b="side_wall_1",
        contact_tol=1e-5,
        name="right upper hanger pad seats on housing",
    )

    # Closed cover sits proud of the front rim and covers the face.
    ctx.expect_gap(
        housing,
        cover,
        axis="y",
        positive_elem="side_wall_1",
        negative_elem="front_panel",
        min_gap=0.006,
        max_gap=0.025,
        name="closed front cover is proud of housing face",
    )
    ctx.expect_overlap(
        cover,
        housing,
        axes="xz",
        elem_a="front_panel",
        min_overlap=0.30,
        name="front cover spans the purifier face height",
    )

    # The two side knuckle hinges are interleaved with small axial clearance.
    ctx.expect_gap(
        housing,
        cover,
        axis="z",
        positive_elem="upper_housing_knuckle_top",
        negative_elem="upper_cover_knuckle",
        min_gap=0.004,
        max_gap=0.018,
        name="upper hinge top knuckle has axial clearance",
    )
    ctx.expect_gap(
        cover,
        housing,
        axis="z",
        positive_elem="lower_cover_knuckle",
        negative_elem="lower_housing_knuckle_bottom",
        min_gap=0.004,
        max_gap=0.018,
        name="lower hinge bottom knuckle has axial clearance",
    )
    with ctx.pose({cover_hinge: 1.0}):
        ctx.expect_gap(
            housing,
            cover,
            axis="y",
            positive_elem="side_wall_1",
            negative_elem="front_panel",
            min_gap=0.035,
            name="front cover swings outward from the housing",
        )

    # The rear filter cartridge stays captured by its rear guide rails at rest
    # and when slid out for service.
    ctx.expect_within(
        cartridge,
        housing,
        axes="xz",
        inner_elem="filter_media",
        margin=0.02,
        name="filter media fits within housing height and width",
    )
    ctx.expect_gap(
        housing,
        cartridge,
        axis="x",
        positive_elem="rear_guide_rail_1",
        negative_elem="side_runner_1",
        min_gap=0.0005,
        max_gap=0.006,
        name="right runner has guide rail clearance",
    )
    ctx.expect_gap(
        cartridge,
        housing,
        axis="x",
        positive_elem="side_runner_0",
        negative_elem="rear_guide_rail_0",
        min_gap=0.0005,
        max_gap=0.006,
        name="left runner has guide rail clearance",
    )
    ctx.expect_overlap(
        cartridge,
        housing,
        axes="y",
        elem_a="side_runner_1",
        elem_b="rear_guide_rail_1",
        min_overlap=0.12,
        name="collapsed filter runner remains deeply inserted",
    )

    rest_pos = ctx.part_world_position(cartridge)
    with ctx.pose({filter_slide: 0.090}):
        ctx.expect_overlap(
            cartridge,
            housing,
            axes="y",
            elem_a="side_runner_1",
            elem_b="rear_guide_rail_1",
            min_overlap=0.045,
            name="extended filter runner remains captured",
        )
        extended_pos = ctx.part_world_position(cartridge)
    ctx.check(
        "filter cartridge slides rearward out of the purifier",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] > rest_pos[1] + 0.080,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
