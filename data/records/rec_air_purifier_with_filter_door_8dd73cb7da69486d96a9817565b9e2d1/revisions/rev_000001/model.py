from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_panel_air_purifier")

    warm_white = Material("warm_white", color=(0.86, 0.88, 0.86, 1.0))
    shadow_gray = Material("shadow_gray", color=(0.12, 0.14, 0.14, 1.0))
    gasket_black = Material("gasket_black", color=(0.02, 0.025, 0.025, 1.0))
    filter_paper = Material("filter_paper", color=(0.92, 0.86, 0.68, 1.0))
    filter_plastic = Material("filter_plastic", color=(0.72, 0.76, 0.72, 1.0))
    rail_gray = Material("rail_gray", color=(0.42, 0.45, 0.45, 1.0))

    housing = model.part("housing")
    # Overall appliance scale: 740 mm wide, 560 mm high, and a shallow
    # 120 mm wall-mounted depth.  The +Y direction is outward from the wall.
    housing.visual(
        Box((0.74, 0.018, 0.56)),
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
        material=warm_white,
        name="back_plate",
    )
    housing.visual(
        Box((0.035, 0.102, 0.56)),
        origin=Origin(xyz=(-0.3525, 0.069, 0.0)),
        material=warm_white,
        name="side_wall_0",
    )
    housing.visual(
        Box((0.035, 0.102, 0.56)),
        origin=Origin(xyz=(0.3525, 0.069, 0.0)),
        material=warm_white,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.74, 0.102, 0.035)),
        origin=Origin(xyz=(0.0, 0.069, 0.2625)),
        material=warm_white,
        name="top_wall",
    )
    housing.visual(
        Box((0.74, 0.102, 0.035)),
        origin=Origin(xyz=(0.0, 0.069, -0.2625)),
        material=warm_white,
        name="bottom_wall",
    )
    housing.visual(
        Box((0.030, 0.008, 0.48)),
        origin=Origin(xyz=(-0.353, 0.124, 0.0)),
        material=rail_gray,
        name="hinge_leaf",
    )
    for x, rail_name in [(-0.20, "guide_rail_0"), (0.20, "guide_rail_1")]:
        housing.visual(
            Box((0.040, 0.220, 0.008)),
            origin=Origin(xyz=(x, 0.120, -0.241)),
            material=rail_gray,
            name=rail_name,
        )
    housing.visual(
        Box((0.67, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.122, 0.222)),
        material=gasket_black,
        name="upper_door_seal",
    )
    housing.visual(
        Box((0.67, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.122, -0.222)),
        material=gasket_black,
        name="lower_door_seal",
    )

    access_door = model.part("access_door")
    access_door.visual(
        Box((0.66, 0.018, 0.44)),
        # The door frame is on the hinge pin.  In the closed pose the slab
        # extends along local +X from the vertical hinge line.
        origin=Origin(xyz=(0.33, 0.0, 0.0)),
        material=warm_white,
        name="door_panel",
    )
    access_door.visual(
        Cylinder(radius=0.012, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rail_gray,
        name="hinge_barrel",
    )
    access_door.visual(
        Box((0.026, 0.034, 0.20)),
        origin=Origin(xyz=(0.600, 0.026, 0.0)),
        material=rail_gray,
        name="front_handle",
    )
    for i, z in enumerate((-0.150, -0.110, -0.070, -0.030, 0.010, 0.050, 0.090, 0.130)):
        access_door.visual(
            Box((0.42, 0.003, 0.012)),
            origin=Origin(xyz=(0.345, 0.0105, z)),
            material=shadow_gray,
            name=f"intake_slot_{i}",
        )

    filter_cartridge = model.part("filter_cartridge")
    filter_cartridge.visual(
        Box((0.025, 0.045, 0.380)),
        origin=Origin(xyz=(-0.2575, 0.0, 0.0)),
        material=filter_plastic,
        name="side_frame_0",
    )
    filter_cartridge.visual(
        Box((0.025, 0.045, 0.380)),
        origin=Origin(xyz=(0.2575, 0.0, 0.0)),
        material=filter_plastic,
        name="side_frame_1",
    )
    filter_cartridge.visual(
        Box((0.540, 0.045, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.1775)),
        material=filter_plastic,
        name="upper_frame",
    )
    filter_cartridge.visual(
        Box((0.540, 0.045, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.1775)),
        material=filter_plastic,
        name="lower_frame",
    )
    filter_cartridge.visual(
        Box((0.490, 0.030, 0.330)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=filter_paper,
        name="pleated_media",
    )
    for i, x in enumerate((-0.210, -0.150, -0.090, -0.030, 0.030, 0.090, 0.150, 0.210)):
        filter_cartridge.visual(
            Box((0.012, 0.008, 0.330)),
            origin=Origin(xyz=(x, 0.019, 0.0)),
            material=filter_paper,
            name=f"pleat_rib_{i}",
        )
    for x, runner_name in [(-0.20, "runner_0"), (0.20, "runner_1")]:
        filter_cartridge.visual(
            Box((0.034, 0.220, 0.012)),
            origin=Origin(xyz=(x, 0.055, -0.196)),
            material=rail_gray,
            name=runner_name,
        )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=access_door,
        origin=Origin(xyz=(-0.330, 0.135, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "housing_to_filter",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_cartridge,
        origin=Origin(xyz=(0.0, 0.075, -0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=0.12),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    access_door = object_model.get_part("access_door")
    filter_cartridge = object_model.get_part("filter_cartridge")
    door_hinge = object_model.get_articulation("housing_to_door")
    filter_slide = object_model.get_articulation("housing_to_filter")

    ctx.expect_within(
        access_door,
        housing,
        axes="xz",
        inner_elem="door_panel",
        outer_elem="back_plate",
        margin=0.001,
        name="closed access door covers the front opening",
    )
    ctx.expect_contact(
        filter_cartridge,
        housing,
        elem_a="runner_0",
        elem_b="guide_rail_0",
        name="filter left runner sits on guide rail",
    )
    ctx.expect_contact(
        filter_cartridge,
        housing,
        elem_a="runner_1",
        elem_b="guide_rail_1",
        name="filter right runner sits on guide rail",
    )
    ctx.expect_within(
        filter_cartridge,
        housing,
        axes="xz",
        inner_elem="pleated_media",
        outer_elem="back_plate",
        margin=0.001,
        name="filter media is contained behind the access door",
    )

    closed_door_aabb = ctx.part_element_world_aabb(access_door, elem="door_panel")
    rest_filter_pos = ctx.part_world_position(filter_cartridge)
    with ctx.pose({door_hinge: 1.35, filter_slide: 0.12}):
        open_door_aabb = ctx.part_element_world_aabb(access_door, elem="door_panel")
        extended_filter_pos = ctx.part_world_position(filter_cartridge)
        ctx.expect_overlap(
            filter_cartridge,
            housing,
            axes="y",
            elem_a="runner_0",
            elem_b="guide_rail_0",
            min_overlap=0.015,
            name="extended filter remains captured on left guide rail",
        )
        ctx.expect_overlap(
            filter_cartridge,
            housing,
            axes="y",
            elem_a="runner_1",
            elem_b="guide_rail_1",
            min_overlap=0.015,
            name="extended filter remains captured on right guide rail",
        )

    ctx.check(
        "door opens outward from the left vertical hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.20,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )
    ctx.check(
        "filter cartridge slides outward",
        rest_filter_pos is not None
        and extended_filter_pos is not None
        and extended_filter_pos[1] > rest_filter_pos[1] + 0.10,
        details=f"rest={rest_filter_pos}, extended={extended_filter_pos}",
    )

    return ctx.report()


object_model = build_object_model()
