from __future__ import annotations

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
    model = ArticulatedObject(name="wall_mounted_air_purifier")

    warm_white = model.material("warm_white_abs", color=(0.88, 0.90, 0.88, 1.0))
    light_gray = model.material("light_gray_trim", color=(0.70, 0.73, 0.72, 1.0))
    dark_gray = model.material("dark_vent_shadow", color=(0.08, 0.09, 0.10, 1.0))
    filter_blue = model.material("blue_filter_media", color=(0.55, 0.72, 0.82, 1.0))
    graphite = model.material("graphite_control", color=(0.18, 0.19, 0.20, 1.0))

    housing = model.part("housing")
    # Wall-appliance body: about 46 cm wide, 66 cm high, and 16 cm deep.
    housing.visual(
        Box((0.46, 0.025, 0.66)),
        origin=Origin(xyz=(0.0, 0.0675, 0.33)),
        material=warm_white,
        name="rear_wall_panel",
    )
    housing.visual(
        Box((0.025, 0.14, 0.66)),
        origin=Origin(xyz=(-0.2175, 0.0, 0.33)),
        material=warm_white,
        name="side_wall_0",
    )
    housing.visual(
        Box((0.025, 0.14, 0.66)),
        origin=Origin(xyz=(0.2175, 0.0, 0.33)),
        material=warm_white,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.46, 0.14, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.6475)),
        material=warm_white,
        name="top_wall",
    )
    housing.visual(
        Box((0.46, 0.14, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=warm_white,
        name="bottom_wall",
    )
    # Raised front rim leaves the central filter opening visible when the door is open.
    housing.visual(
        Box((0.045, 0.018, 0.58)),
        origin=Origin(xyz=(-0.1875, -0.078, 0.33)),
        material=light_gray,
        name="front_stile_0",
    )
    housing.visual(
        Box((0.045, 0.018, 0.58)),
        origin=Origin(xyz=(0.1875, -0.078, 0.33)),
        material=light_gray,
        name="front_stile_1",
    )
    housing.visual(
        Box((0.42, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.078, 0.610)),
        material=light_gray,
        name="front_rail_top",
    )
    housing.visual(
        Box((0.42, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.078, 0.050)),
        material=light_gray,
        name="front_rail_bottom",
    )
    # Internal tracks that visually support the removable filter cassette.
    housing.visual(
        Box((0.035, 0.105, 0.018)),
        origin=Origin(xyz=(-0.1875, -0.005, 0.120)),
        material=light_gray,
        name="filter_guide_0",
    )
    housing.visual(
        Box((0.035, 0.105, 0.018)),
        origin=Origin(xyz=(0.1875, -0.005, 0.120)),
        material=light_gray,
        name="filter_guide_1",
    )
    housing.visual(
        Box((0.035, 0.105, 0.018)),
        origin=Origin(xyz=(-0.1875, -0.005, 0.540)),
        material=light_gray,
        name="filter_guide_2",
    )
    housing.visual(
        Box((0.035, 0.105, 0.018)),
        origin=Origin(xyz=(0.1875, -0.005, 0.540)),
        material=light_gray,
        name="filter_guide_3",
    )
    # Side speed-control guide: a short dark slot on the right cheek of the case.
    housing.visual(
        Box((0.010, 0.056, 0.170)),
        origin=Origin(xyz=(0.235, -0.004, 0.480)),
        material=dark_gray,
        name="speed_slot",
    )
    housing.visual(
        Box((0.025, 0.030, 0.130)),
        origin=Origin(xyz=(-0.2175, -0.077, 0.170)),
        material=light_gray,
        name="hinge_mount_0",
    )
    housing.visual(
        Box((0.025, 0.030, 0.130)),
        origin=Origin(xyz=(-0.2175, -0.077, 0.490)),
        material=light_gray,
        name="hinge_mount_1",
    )

    door = model.part("front_door")
    # Door frame is on the hinge line.  Closed geometry extends along local +X.
    door.visual(
        Box((0.405, 0.014, 0.580)),
        origin=Origin(xyz=(0.2025, 0.0, 0.290)),
        material=warm_white,
        name="door_panel",
    )
    door.visual(
        Box((0.028, 0.018, 0.540)),
        origin=Origin(xyz=(0.391, -0.003, 0.290)),
        material=light_gray,
        name="pull_recess",
    )
    for index, z in enumerate((0.170, 0.210, 0.250, 0.290, 0.330, 0.370, 0.410)):
        door.visual(
            Box((0.250, 0.004, 0.010)),
            origin=Origin(xyz=(0.215, -0.0085, z)),
            material=dark_gray,
            name=f"intake_slot_{index}",
        )
    # Slim exposed hinge knuckles on the door edge.
    door.visual(
        Cylinder(radius=0.008, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=light_gray,
        name="hinge_barrel_0",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.450)),
        material=light_gray,
        name="hinge_barrel_1",
    )

    cassette = model.part("filter_cassette")
    cassette.visual(
        Box((0.340, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=light_gray,
        name="cassette_top_rail",
    )
    cassette.visual(
        Box((0.340, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=light_gray,
        name="cassette_bottom_rail",
    )
    cassette.visual(
        Box((0.018, 0.090, 0.450)),
        origin=Origin(xyz=(-0.161, 0.0, 0.0)),
        material=light_gray,
        name="cassette_side_0",
    )
    cassette.visual(
        Box((0.018, 0.090, 0.450)),
        origin=Origin(xyz=(0.161, 0.0, 0.0)),
        material=light_gray,
        name="cassette_side_1",
    )
    cassette.visual(
        Box((0.304, 0.018, 0.432)),
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
        material=filter_blue,
        name="filter_media",
    )
    for index, x in enumerate((-0.120, -0.080, -0.040, 0.0, 0.040, 0.080, 0.120)):
        cassette.visual(
            Box((0.008, 0.026, 0.370)),
            origin=Origin(xyz=(x, -0.024, 0.0)),
            material=filter_blue,
            name=f"filter_pleat_{index}",
        )
    cassette.visual(
        Box((0.080, 0.070, 0.028)),
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        material=light_gray,
        name="cassette_pull_tab",
    )

    slider = model.part("speed_slider")
    slider.visual(
        Box((0.024, 0.052, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=graphite,
        name="slider_thumb",
    )
    slider.visual(
        Box((0.008, 0.020, 0.030)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0)),
        material=graphite,
        name="slider_stem",
    )
    slider.visual(
        Box((0.004, 0.040, 0.006)),
        origin=Origin(xyz=(0.014, 0.0, 0.014)),
        material=light_gray,
        name="grip_ridge_0",
    )
    slider.visual(
        Box((0.004, 0.040, 0.006)),
        origin=Origin(xyz=(0.014, 0.0, -0.014)),
        material=light_gray,
        name="grip_ridge_1",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(-0.205, -0.100, 0.040)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=7.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "cassette_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=cassette,
        origin=Origin(xyz=(0.0, -0.005, 0.335)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.090),
    )
    model.articulation(
        "speed_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=slider,
        origin=Origin(xyz=(0.249, -0.004, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.12, lower=0.0, upper=0.100),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("front_door")
    cassette = object_model.get_part("filter_cassette")
    slider = object_model.get_part("speed_slider")
    door_hinge = object_model.get_articulation("door_hinge")
    cassette_slide = object_model.get_articulation("cassette_slide")
    speed_slide = object_model.get_articulation("speed_slide")

    ctx.allow_overlap(
        housing,
        slider,
        elem_a="speed_slot",
        elem_b="slider_stem",
        reason="The speed slider stem is intentionally captured in the solid proxy for the side guide slot.",
    )

    ctx.expect_gap(
        housing,
        door,
        axis="y",
        min_gap=0.004,
        max_gap=0.025,
        positive_elem="front_rail_top",
        negative_elem="door_panel",
        name="closed door sits just proud of the front rim",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        min_overlap=0.35,
        name="front door covers the purifier opening",
    )
    ctx.expect_within(
        cassette,
        housing,
        axes="xz",
        margin=0.0,
        name="filter cassette fits within the front opening",
    )
    ctx.expect_overlap(
        housing,
        slider,
        axes="yz",
        elem_a="speed_slot",
        elem_b="slider_stem",
        min_overlap=0.018,
        name="slider stem is retained in the side guide",
    )

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door hinge opens outward from the front",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < rest_door_aabb[0][1] - 0.14,
        details=f"rest={rest_door_aabb}, open={open_door_aabb}",
    )

    rest_cassette_pos = ctx.part_world_position(cassette)
    with ctx.pose({cassette_slide: 0.090}):
        extended_cassette_pos = ctx.part_world_position(cassette)
        ctx.expect_overlap(
            cassette,
            housing,
            axes="y",
            min_overlap=0.015,
            name="extended cassette remains partly inserted in the housing",
        )
    ctx.check(
        "filter cassette slides out from the front",
        rest_cassette_pos is not None
        and extended_cassette_pos is not None
        and extended_cassette_pos[1] < rest_cassette_pos[1] - 0.075,
        details=f"rest={rest_cassette_pos}, extended={extended_cassette_pos}",
    )

    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({speed_slide: 0.100}):
        raised_slider_pos = ctx.part_world_position(slider)
        ctx.expect_within(
            slider,
            housing,
            axes="yz",
            inner_elem="slider_stem",
            outer_elem="speed_slot",
            margin=0.002,
            name="raised slider remains inside the short guide",
        )
    ctx.check(
        "side fan-speed slider moves along its short vertical guide",
        rest_slider_pos is not None
        and raised_slider_pos is not None
        and raised_slider_pos[2] > rest_slider_pos[2] + 0.085,
        details=f"rest={rest_slider_pos}, raised={raised_slider_pos}",
    )

    return ctx.report()


object_model = build_object_model()
