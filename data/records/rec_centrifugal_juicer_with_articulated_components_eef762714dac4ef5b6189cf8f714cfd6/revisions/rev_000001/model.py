from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_feed_juicer")

    white = model.material("warm_white_plastic", rgba=(0.92, 0.90, 0.84, 1.0))
    dark = model.material("charcoal_gasket", rgba=(0.035, 0.038, 0.040, 1.0))
    clear = model.material("clear_blue_polycarbonate", rgba=(0.62, 0.84, 1.0, 0.34))
    smoke = model.material("smoked_clear_flap", rgba=(0.20, 0.28, 0.34, 0.46))
    steel = model.material("brushed_stainless", rgba=(0.74, 0.76, 0.73, 1.0))
    black = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.50, 0.38, 0.155)),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=white,
        name="broad_rectangular_base",
    )
    base.visual(
        Box((0.42, 0.30, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
        material=white,
        name="raised_lid_seat",
    )
    base.visual(
        mesh_from_geometry(
            BezelGeometry(
                opening_size=(0.215, 0.215),
                outer_size=(0.250, 0.250),
                depth=0.018,
                opening_shape="circle",
                outer_shape="circle",
                face=None,
            ),
            "motor_bowl_rim",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=dark,
        name="motor_bowl_rim",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=steel,
        name="motor_coupler",
    )
    base.visual(
        Box((0.105, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, -0.215, 0.118)),
        material=white,
        name="front_juice_spout_body",
    )
    base.visual(
        Box((0.066, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, -0.252, 0.105)),
        material=dark,
        name="front_spout_lip",
    )
    # Rear hinge supports mounted to the base, split so the lid knuckle can sit
    # between them without colliding.
    for x in (-0.135, 0.135):
        base.visual(
            Box((0.070, 0.025, 0.065)),
            origin=Origin(xyz=(x, 0.188, 0.185)),
            material=white,
            name=f"rear_hinge_pedestal_{0 if x < 0 else 1}",
        )
        base.visual(
            Cylinder(radius=0.017, length=0.074),
            origin=Origin(xyz=(x, 0.197, 0.210), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"rear_hinge_barrel_{0 if x < 0 else 1}",
        )
    for x in (-0.18, 0.18):
        for y in (-0.13, 0.13):
            base.visual(
                Cylinder(radius=0.030, length=0.014),
                origin=Origin(xyz=(x, y, -0.007)),
                material=black,
                name=f"rubber_foot_{'n' if y > 0 else 'f'}_{'p' if x > 0 else 'm'}",
            )

    basket = model.part("basket")
    basket.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                outer_radius=0.105,
                inner_radius=0.046,
                width=0.092,
                blade_count=28,
                blade_thickness=0.0028,
                blade_sweep_deg=18.0,
                backplate=True,
                shroud=True,
                center=False,
            ),
            "perforated_filter_basket",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=steel,
        name="perforated_filter_basket",
    )
    basket.visual(
        Cylinder(radius=0.050, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="basket_drive_hub",
    )

    lid = model.part("lid")
    # The lid frame is at the rear hinge line.  At q=0 it extends along local -Y.
    cover_y = -0.180
    lid.visual(
        Box((0.390, 0.014, 0.122)),
        origin=Origin(xyz=(0.0, cover_y - 0.157, 0.031)),
        material=clear,
        name="front_clear_skirt",
    )
    lid.visual(
        Box((0.390, 0.014, 0.122)),
        origin=Origin(xyz=(0.0, cover_y + 0.147, 0.031)),
        material=clear,
        name="rear_clear_skirt",
    )
    for x in (-0.195, 0.195):
        lid.visual(
            Box((0.014, 0.314, 0.122)),
            origin=Origin(xyz=(x, cover_y - 0.005, 0.031)),
            material=clear,
            name=f"side_clear_skirt_{0 if x < 0 else 1}",
        )
    lid.visual(
        mesh_from_geometry(
            BezelGeometry(
                opening_size=(0.170, 0.116),
                outer_size=(0.390, 0.294),
                depth=0.016,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.012,
                outer_corner_radius=0.030,
            ),
            "clear_lid_top_frame",
        ),
        origin=Origin(xyz=(0.0, cover_y, 0.100)),
        material=clear,
        name="clear_lid_top_frame",
    )
    # Wide rectangular feed chute: four transparent walls, open through the lid.
    chute_outer_x = 0.198
    chute_outer_y = 0.144
    chute_open_x = 0.164
    chute_open_y = 0.108
    chute_wall = 0.017
    chute_h = 0.176
    chute_z = 0.196
    lid.visual(
        Box((chute_outer_x, chute_wall, chute_h)),
        origin=Origin(xyz=(0.0, cover_y - chute_outer_y / 2.0 + chute_wall / 2.0, chute_z)),
        material=clear,
        name="front_chute_wall",
    )
    lid.visual(
        Box((chute_outer_x, chute_wall, chute_h)),
        origin=Origin(xyz=(0.0, cover_y + chute_outer_y / 2.0 - chute_wall / 2.0, chute_z)),
        material=clear,
        name="rear_chute_wall",
    )
    for x in (-(chute_open_x / 2.0 + chute_wall / 2.0), chute_open_x / 2.0 + chute_wall / 2.0):
        lid.visual(
            Box((chute_wall, chute_open_y, chute_h)),
            origin=Origin(xyz=(x, cover_y, chute_z)),
            material=clear,
            name=f"side_chute_wall_{0 if x < 0 else 1}",
        )
    lid.visual(
        Cylinder(radius=0.018, length=0.095),
        origin=Origin(xyz=(0.0, 0.016, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoke,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.120, 0.060, 0.026)),
        origin=Origin(xyz=(0.0, -0.012, 0.004)),
        material=clear,
        name="lid_hinge_leaf",
    )
    # Fixed knuckles for the small flap hinge on the chute rim.
    flap_hinge_y = cover_y + chute_outer_y / 2.0 + 0.0065
    flap_hinge_z = chute_z + chute_h / 2.0 - 0.003
    for x in (-0.067, 0.067):
        lid.visual(
            Cylinder(radius=0.0075, length=0.050),
            origin=Origin(xyz=(x, flap_hinge_y, flap_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"flap_fixed_knuckle_{0 if x < 0 else 1}",
        )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.146, 0.088, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=white,
        name="wide_pusher_block",
    )
    pusher.visual(
        Box((0.176, 0.118, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=white,
        name="pusher_top_cap",
    )
    pusher.visual(
        Box((0.085, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=dark,
        name="pusher_grip_pad",
    )
    for x in (-0.0775, 0.0775):
        pusher.visual(
            Box((0.009, 0.070, 0.108)),
            origin=Origin(xyz=(x, 0.0, -0.050)),
            material=white,
            name=f"side_pusher_guide_{0 if x < 0 else 1}",
        )

    flap = model.part("flap")
    # At q=0 the flap stands upright behind the chute.  Positive rotation about
    # +X folds it down across the top opening.
    flap.visual(
        Box((0.190, 0.010, 0.126)),
        origin=Origin(xyz=(0.0, 0.006, 0.077)),
        material=smoke,
        name="flap_cover_plate",
    )
    flap.visual(
        Cylinder(radius=0.0070, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoke,
        name="flap_center_knuckle",
    )
    flap.visual(
        Box((0.080, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.006, 0.014)),
        material=smoke,
        name="flap_hinge_leaf",
    )

    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=120.0),
    )
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.197, 0.210)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.25),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, cover_y, chute_z + chute_h / 2.0 + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.245),
    )
    model.articulation(
        "lid_to_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=flap,
        origin=Origin(xyz=(0.0, flap_hinge_y, flap_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    flap = object_model.get_part("flap")
    basket = object_model.get_part("basket")
    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    flap_hinge = object_model.get_articulation("lid_to_flap")
    spin = object_model.get_articulation("base_to_basket")

    ctx.check(
        "basket uses continuous motor-axis joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.allow_overlap(
        base,
        basket,
        elem_a="motor_coupler",
        elem_b="basket_drive_hub",
        reason="The stainless basket hub is intentionally seated over the motor coupler as a captured drive interface.",
    )
    ctx.expect_within(
        basket,
        base,
        axes="xy",
        inner_elem="basket_drive_hub",
        outer_elem="motor_coupler",
        margin=0.020,
        name="basket hub is centered on the motor coupler",
    )
    ctx.expect_overlap(
        base,
        basket,
        axes="z",
        elem_a="motor_coupler",
        elem_b="basket_drive_hub",
        min_overlap=0.010,
        name="basket hub remains seated on the drive coupler",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="xy",
        elem_a="wide_pusher_block",
        elem_b="clear_lid_top_frame",
        min_overlap=0.080,
        name="pusher aligns with the wide feed opening",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="wide_pusher_block",
        outer_elem="clear_lid_top_frame",
        margin=0.018,
        name="pusher stays centered inside the chute footprint",
    )
    with ctx.pose({lid_hinge: 0.9}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="front_clear_skirt",
            negative_elem="raised_lid_seat",
            min_gap=0.035,
            name="front of main lid lifts clear of the base on its rear hinge",
        )
    rest_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.245}):
        raised_pusher_pos = ctx.part_world_position(pusher)
        ctx.expect_gap(
            pusher,
            lid,
            axis="z",
            min_gap=0.020,
            positive_elem="wide_pusher_block",
            negative_elem="front_chute_wall",
            name="raised pusher clears the chute rim",
        )
    ctx.check(
        "pusher slides upward through the feed chute",
        rest_pusher_pos is not None
        and raised_pusher_pos is not None
        and raised_pusher_pos[2] > rest_pusher_pos[2] + 0.20,
        details=f"rest={rest_pusher_pos}, raised={raised_pusher_pos}",
    )
    with ctx.pose({pusher_slide: 0.245, flap_hinge: math.pi / 2.0}):
        ctx.expect_overlap(
            flap,
            lid,
            axes="xy",
            elem_a="flap_cover_plate",
            elem_b="clear_lid_top_frame",
            min_overlap=0.095,
            name="secondary flap covers the feed opening when folded down",
        )
        ctx.expect_gap(
            pusher,
            flap,
            axis="z",
            min_gap=0.010,
            positive_elem="wide_pusher_block",
            negative_elem="flap_cover_plate",
            name="withdrawn pusher clears the closed flap",
        )
    with ctx.pose({spin: 1.0}):
        ctx.expect_origin_distance(
            basket,
            base,
            axes="xy",
            max_dist=0.001,
            name="basket remains concentric with the base motor axis while spinning",
        )

    return ctx.report()


object_model = build_object_model()
