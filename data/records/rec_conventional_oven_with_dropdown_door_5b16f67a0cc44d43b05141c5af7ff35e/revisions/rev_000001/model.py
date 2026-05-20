from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="steam_combi_oven")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.72, 1.0))
    dark_steel = model.material("dark_cavity", rgba=(0.03, 0.035, 0.04, 1.0))
    black_glass = model.material("black_glass", rgba=(0.02, 0.025, 0.03, 0.58))
    chrome = model.material("polished_chrome", rgba=(0.90, 0.92, 0.95, 1.0))
    reservoir_blue = model.material("translucent_water_tank", rgba=(0.55, 0.82, 1.0, 0.42))
    display_green = model.material("green_display", rgba=(0.08, 0.55, 0.38, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    body = model.part("body")

    # A connected sheet-metal oven shell with a real front opening and a cut-out
    # on the left side for the sliding water reservoir.
    body.visual(Box((0.85, 0.62, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0175)), material=stainless, name="bottom_pan")
    body.visual(Box((0.85, 0.62, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.5325)), material=stainless, name="top_pan")
    body.visual(Box((0.85, 0.030, 0.55)), origin=Origin(xyz=(0.0, 0.295, 0.275)), material=stainless, name="rear_wall")
    body.visual(Box((0.030, 0.62, 0.55)), origin=Origin(xyz=(0.425, 0.0, 0.275)), material=stainless, name="side_wall")

    body.visual(Box((0.030, 0.62, 0.285)), origin=Origin(xyz=(-0.425, 0.0, 0.1425)), material=stainless, name="reservoir_lower_side")
    body.visual(Box((0.030, 0.62, 0.065)), origin=Origin(xyz=(-0.425, 0.0, 0.5175)), material=stainless, name="reservoir_upper_side")
    body.visual(Box((0.030, 0.170, 0.220)), origin=Origin(xyz=(-0.425, -0.225, 0.385)), material=stainless, name="reservoir_front_jamb")
    body.visual(Box((0.030, 0.170, 0.220)), origin=Origin(xyz=(-0.425, 0.225, 0.385)), material=stainless, name="reservoir_rear_jamb")

    body.visual(Box((0.055, 0.018, 0.450)), origin=Origin(xyz=(-0.3975, -0.306, 0.285)), material=stainless, name="front_jamb_0")
    body.visual(Box((0.055, 0.018, 0.450)), origin=Origin(xyz=(0.3975, -0.306, 0.285)), material=stainless, name="front_jamb_1")
    body.visual(Box((0.85, 0.018, 0.060)), origin=Origin(xyz=(0.0, -0.306, 0.055)), material=stainless, name="front_bottom_rail")
    body.visual(Box((0.85, 0.018, 0.055)), origin=Origin(xyz=(0.0, -0.306, 0.5225)), material=stainless, name="front_top_rail")
    body.visual(Box((0.550, 0.012, 0.360)), origin=Origin(xyz=(0.100, 0.170, 0.285)), material=dark_steel, name="cavity_shadow")
    body.visual(Box((0.550, 0.470, 0.012)), origin=Origin(xyz=(0.100, -0.070, 0.091)), material=dark_steel, name="cavity_floor")
    body.visual(Box((0.550, 0.470, 0.012)), origin=Origin(xyz=(0.100, -0.070, 0.489)), material=dark_steel, name="cavity_ceiling")
    body.visual(Box((0.012, 0.470, 0.386)), origin=Origin(xyz=(-0.181, -0.070, 0.290)), material=dark_steel, name="cavity_side_0")
    body.visual(Box((0.012, 0.470, 0.386)), origin=Origin(xyz=(0.381, -0.070, 0.290)), material=dark_steel, name="cavity_side_1")
    body.visual(Box((0.760, 0.004, 0.030)), origin=Origin(xyz=(0.0, -0.3155, 0.085)), material=rubber, name="gasket_bottom")
    body.visual(Box((0.760, 0.004, 0.030)), origin=Origin(xyz=(0.0, -0.3155, 0.485)), material=rubber, name="gasket_top")
    body.visual(Box((0.030, 0.004, 0.430)), origin=Origin(xyz=(-0.380, -0.3155, 0.285)), material=rubber, name="gasket_side_0")
    body.visual(Box((0.030, 0.004, 0.430)), origin=Origin(xyz=(0.380, -0.3155, 0.285)), material=rubber, name="gasket_side_1")

    body.visual(Box((0.030, 0.055, 0.060)), origin=Origin(xyz=(-0.415, -0.3325, 0.055)), material=chrome, name="door_hinge_lug_0")
    body.visual(Box((0.030, 0.055, 0.060)), origin=Origin(xyz=(0.415, -0.3325, 0.055)), material=chrome, name="door_hinge_lug_1")
    body.visual(Box((0.025, 0.025, 0.030)), origin=Origin(xyz=(-0.412, -0.090, 0.565)), material=chrome, name="cover_hinge_lug_0")
    body.visual(Box((0.025, 0.025, 0.030)), origin=Origin(xyz=(0.412, -0.090, 0.565)), material=chrome, name="cover_hinge_lug_1")

    door = model.part("door")
    door.visual(Box((0.055, 0.035, 0.450)), origin=Origin(xyz=(-0.3625, 0.0, 0.225)), material=stainless, name="door_side_0")
    door.visual(Box((0.055, 0.035, 0.450)), origin=Origin(xyz=(0.3625, 0.0, 0.225)), material=stainless, name="door_side_1")
    door.visual(Box((0.78, 0.035, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.030)), material=stainless, name="door_bottom_rail")
    door.visual(Box((0.78, 0.035, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.420)), material=stainless, name="door_top_rail")
    door.visual(Box((0.70, 0.008, 0.360)), origin=Origin(xyz=(0.0, -0.020, 0.235)), material=black_glass, name="glass_panel")
    door.visual(Cylinder(radius=0.018, length=0.64), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=chrome, name="hinge_barrel")
    door.visual(Cylinder(radius=0.018, length=0.62), origin=Origin(xyz=(0.0, -0.065, 0.360), rpy=(0.0, pi / 2.0, 0.0)), material=chrome, name="bar_handle")
    door.visual(Cylinder(radius=0.011, length=0.055), origin=Origin(xyz=(-0.265, -0.040, 0.360), rpy=(pi / 2.0, 0.0, 0.0)), material=chrome, name="handle_post_0")
    door.visual(Cylinder(radius=0.011, length=0.055), origin=Origin(xyz=(0.265, -0.040, 0.360), rpy=(pi / 2.0, 0.0, 0.0)), material=chrome, name="handle_post_1")

    reservoir = model.part("water_drawer")
    reservoir.visual(Box((0.240, 0.180, 0.150)), origin=Origin(xyz=(0.120, 0.0, 0.0)), material=reservoir_blue, name="water_tank")
    reservoir.visual(Box((0.026, 0.240, 0.190)), origin=Origin(xyz=(-0.013, 0.0, 0.0)), material=stainless, name="drawer_face")
    reservoir.visual(Box((0.012, 0.140, 0.045)), origin=Origin(xyz=(-0.032, 0.0, -0.030)), material=chrome, name="finger_pull")
    reservoir.visual(Box((0.010, 0.150, 0.025)), origin=Origin(xyz=(0.045, 0.0, -0.080)), material=dark_steel, name="lower_slide")
    reservoir.visual(Box((0.010, 0.150, 0.025)), origin=Origin(xyz=(0.045, 0.0, 0.080)), material=dark_steel, name="upper_slide")

    control_cover = model.part("control_cover")
    control_cover.visual(Box((0.780, 0.210, 0.025)), origin=Origin(xyz=(0.0, -0.105, 0.0125)), material=stainless, name="cover_panel")
    control_cover.visual(Cylinder(radius=0.012, length=0.70), origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(0.0, pi / 2.0, 0.0)), material=chrome, name="cover_hinge_barrel")
    control_cover.visual(Box((0.220, 0.055, 0.004)), origin=Origin(xyz=(-0.180, -0.122, 0.027)), material=black_glass, name="display_window")
    control_cover.visual(Box((0.150, 0.026, 0.004)), origin=Origin(xyz=(-0.180, -0.122, 0.030)), material=display_green, name="display_readout")
    control_cover.visual(Box((0.050, 0.030, 0.005)), origin=Origin(xyz=(0.070, -0.115, 0.0265)), material=rubber, name="touch_key_0")
    control_cover.visual(Box((0.050, 0.030, 0.005)), origin=Origin(xyz=(0.145, -0.115, 0.0265)), material=rubber, name="touch_key_1")
    control_cover.visual(Box((0.050, 0.030, 0.005)), origin=Origin(xyz=(0.220, -0.115, 0.0265)), material=rubber, name="touch_key_2")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -0.335, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=0.0, upper=1.45),
    )

    model.articulation(
        "reservoir_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=reservoir,
        origin=Origin(xyz=(-0.429, -0.020, 0.390)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.25, lower=0.0, upper=0.18),
    )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=control_cover,
        origin=Origin(xyz=(0.0, -0.100, 0.550)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    reservoir = object_model.get_part("water_drawer")
    cover = object_model.get_part("control_cover")
    door_hinge = object_model.get_articulation("door_hinge")
    reservoir_slide = object_model.get_articulation("reservoir_slide")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=0.0,
        max_gap=0.012,
        positive_elem="front_top_rail",
        negative_elem="door_top_rail",
        name="closed door sits just proud of the oven front",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.30,
        elem_a="glass_panel",
        elem_b="cavity_shadow",
        name="glass panel covers the cooking cavity",
    )
    ctx.expect_gap(
        reservoir,
        body,
        axis="z",
        min_gap=0.015,
        positive_elem="water_tank",
        negative_elem="reservoir_lower_side",
        name="water tank clears the lower edge of the side aperture",
    )
    ctx.expect_gap(
        body,
        reservoir,
        axis="z",
        min_gap=0.015,
        positive_elem="reservoir_upper_side",
        negative_elem="water_tank",
        name="water tank clears the upper edge of the side aperture",
    )
    ctx.expect_gap(
        reservoir,
        body,
        axis="y",
        min_gap=0.015,
        positive_elem="water_tank",
        negative_elem="reservoir_front_jamb",
        name="water tank clears the front edge of the side aperture",
    )
    ctx.expect_gap(
        body,
        reservoir,
        axis="y",
        min_gap=0.015,
        positive_elem="reservoir_rear_jamb",
        negative_elem="water_tank",
        name="water tank clears the rear edge of the side aperture",
    )
    ctx.expect_overlap(
        reservoir,
        body,
        axes="x",
        min_overlap=0.20,
        elem_a="water_tank",
        elem_b="top_pan",
        name="closed reservoir remains inserted into the oven body",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "bottom hinge folds door downward and outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.05
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.18,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rest_drawer_pos = ctx.part_world_position(reservoir)
    with ctx.pose({reservoir_slide: 0.18}):
        extended_drawer_pos = ctx.part_world_position(reservoir)
        ctx.expect_overlap(
            reservoir,
            body,
            axes="x",
            min_overlap=0.04,
            elem_a="water_tank",
            elem_b="top_pan",
            name="extended reservoir retains insertion in the side opening",
        )
    ctx.check(
        "reservoir drawer pulls out from the left side",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] < rest_drawer_pos[0] - 0.15,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 0.90}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "rear-hinged control cover lifts for service access",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.08
        and open_cover_aabb[0][1] > closed_cover_aabb[0][1] - 0.03,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
