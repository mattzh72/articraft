from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_rolling_tool_chest")

    charcoal = model.material("charcoal_ribbed_plastic", rgba=(0.09, 0.10, 0.10, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    dark = model.material("dark_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    blue = model.material("blue_latch_button", rgba=(0.02, 0.16, 0.55, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.61, 1.0))

    body = model.part("body")

    # Deep molded lower tub: roughly garage-storage size, open at the top.
    body.visual(Box((0.80, 0.52, 0.080)), origin=Origin(xyz=(0.0, 0.0, 0.040)), material=charcoal, name="bottom_pan")
    body.visual(Box((0.055, 0.52, 0.430)), origin=Origin(xyz=(0.3725, 0.0, 0.275)), material=charcoal, name="front_wall")
    body.visual(Box((0.055, 0.52, 0.430)), origin=Origin(xyz=(-0.3725, 0.0, 0.275)), material=charcoal, name="rear_wall")
    body.visual(Box((0.80, 0.055, 0.430)), origin=Origin(xyz=(0.0, 0.2325, 0.275)), material=charcoal, name="side_wall_0")
    body.visual(Box((0.80, 0.055, 0.430)), origin=Origin(xyz=(0.0, -0.2325, 0.275)), material=charcoal, name="side_wall_1")
    body.visual(Box((0.80, 0.040, 0.030)), origin=Origin(xyz=(0.0, 0.240, 0.480)), material=dark, name="top_rim_0")
    body.visual(Box((0.80, 0.040, 0.030)), origin=Origin(xyz=(0.0, -0.240, 0.480)), material=dark, name="top_rim_1")
    body.visual(Box((0.045, 0.52, 0.030)), origin=Origin(xyz=(0.3775, 0.0, 0.480)), material=dark, name="top_rim_front")
    body.visual(Box((0.045, 0.52, 0.030)), origin=Origin(xyz=(-0.3775, 0.0, 0.480)), material=dark, name="top_rim_rear")

    # Raised molded ribs on the shell, embedded slightly into the tub walls.
    for i, z in enumerate((0.150, 0.245, 0.340, 0.430)):
        body.visual(Box((0.016, 0.440, 0.018)), origin=Origin(xyz=(0.403, 0.0, z)), material=dark, name=f"front_rib_{i}")
        body.visual(Box((0.016, 0.400, 0.016)), origin=Origin(xyz=(-0.403, 0.0, z)), material=dark, name=f"rear_rib_{i}")
    for side, y in enumerate((0.263, -0.263)):
        for i, z in enumerate((0.160, 0.265, 0.370)):
            body.visual(Box((0.600, 0.016, 0.018)), origin=Origin(xyz=(0.015, y, z)), material=dark, name=f"side_rib_{side}_{i}")
    for i, y in enumerate((-0.205, 0.205)):
        body.visual(Box((0.055, 0.035, 0.400)), origin=Origin(xyz=(0.403, y, 0.285)), material=dark, name=f"front_corner_post_{i}")

    # Rear pull-handle guide channels, molded into the back of the chest.
    body.visual(Box((0.020, 0.060, 0.475)), origin=Origin(xyz=(-0.410, -0.170, 0.2725)), material=dark, name="handle_channel_web_0")
    body.visual(Box((0.056, 0.010, 0.475)), origin=Origin(xyz=(-0.437, -0.205, 0.2725)), material=dark, name="handle_channel_cheek_0_0")
    body.visual(Box((0.056, 0.010, 0.475)), origin=Origin(xyz=(-0.437, -0.135, 0.2725)), material=dark, name="handle_channel_cheek_0_1")
    body.visual(Box((0.020, 0.060, 0.475)), origin=Origin(xyz=(-0.410, 0.170, 0.2725)), material=dark, name="handle_channel_web_1")
    body.visual(Box((0.056, 0.010, 0.475)), origin=Origin(xyz=(-0.437, 0.135, 0.2725)), material=dark, name="handle_channel_cheek_1_0")
    body.visual(Box((0.056, 0.010, 0.475)), origin=Origin(xyz=(-0.437, 0.205, 0.2725)), material=dark, name="handle_channel_cheek_1_1")

    # Visible fixed axle and fork pads for the two rear wheels.
    body.visual(Cylinder(radius=0.012, length=0.550), origin=Origin(xyz=(-0.325, 0.0, 0.095), rpy=(pi / 2, 0.0, 0.0)), material=steel, name="rear_axle")
    body.visual(Cylinder(radius=0.012, length=0.006), origin=Origin(xyz=(-0.325, 0.2765, 0.095), rpy=(pi / 2, 0.0, 0.0)), material=steel, name="axle_stub_0")
    body.visual(Cylinder(radius=0.035, length=0.006), origin=Origin(xyz=(-0.325, 0.281, 0.095), rpy=(pi / 2, 0.0, 0.0)), material=steel, name="axle_washer_0")
    body.visual(Cylinder(radius=0.035, length=0.006), origin=Origin(xyz=(-0.325, -0.2728, 0.095), rpy=(pi / 2, 0.0, 0.0)), material=steel, name="axle_washer_1")
    body.visual(Box((0.090, 0.022, 0.120)), origin=Origin(xyz=(-0.325, 0.266, 0.105)), material=charcoal, name="wheel_fork_0")
    body.visual(Box((0.090, 0.022, 0.120)), origin=Origin(xyz=(-0.325, -0.266, 0.105)), material=charcoal, name="wheel_fork_1")

    lid = model.part("lid")
    lid.visual(Box((0.800, 0.540, 0.045)), origin=Origin(xyz=(0.4125, 0.0, 0.0425)), material=charcoal, name="lid_top")
    lid.visual(Box((0.800, 0.035, 0.065)), origin=Origin(xyz=(0.4125, 0.2675, 0.0325)), material=charcoal, name="lid_side_skirt_0")
    lid.visual(Box((0.800, 0.035, 0.065)), origin=Origin(xyz=(0.4125, -0.2675, 0.0325)), material=charcoal, name="lid_side_skirt_1")
    # Front nose is built around a real central opening for the push button.
    lid.visual(Box((0.035, 0.160, 0.070)), origin=Origin(xyz=(0.825, 0.190, 0.035)), material=charcoal, name="lid_nose_side_0")
    lid.visual(Box((0.035, 0.160, 0.070)), origin=Origin(xyz=(0.825, -0.190, 0.035)), material=charcoal, name="lid_nose_side_1")
    lid.visual(Box((0.035, 0.220, 0.018)), origin=Origin(xyz=(0.825, 0.0, 0.061)), material=charcoal, name="lid_nose_top_rail")
    lid.visual(Box((0.035, 0.220, 0.018)), origin=Origin(xyz=(0.825, 0.0, 0.009)), material=charcoal, name="lid_nose_bottom_rail")
    lid.visual(Box((0.030, 0.010, 0.022)), origin=Origin(xyz=(0.8275, 0.035, 0.034)), material=dark, name="latch_guide_0")
    lid.visual(Box((0.030, 0.010, 0.022)), origin=Origin(xyz=(0.8275, -0.035, 0.034)), material=dark, name="latch_guide_1")
    for i, x in enumerate((0.145, 0.285, 0.425, 0.565, 0.705)):
        lid.visual(Box((0.040, 0.445, 0.014)), origin=Origin(xyz=(x, 0.0, 0.071)), material=dark, name=f"lid_rib_{i}")
    lid.visual(Box((0.750, 0.030, 0.018)), origin=Origin(xyz=(0.420, 0.0, 0.085)), material=dark, name="lid_center_raised_ridge")

    # Alternating hinge barrels sit on the rear hinge line.
    for i, y in enumerate((-0.180, 0.000, 0.180)):
        body.visual(Cylinder(radius=0.016, length=0.090), origin=Origin(xyz=(-0.425, y, 0.514), rpy=(pi / 2, 0.0, 0.0)), material=dark, name=f"body_hinge_knuckle_{i}")
    body.visual(Box((0.025, 0.500, 0.008)), origin=Origin(xyz=(-0.413, 0.0, 0.494)), material=dark, name="body_hinge_leaf")
    for i, y in enumerate((-0.090, 0.090)):
        lid.visual(Cylinder(radius=0.016, length=0.080), origin=Origin(xyz=(0.0, y, 0.014), rpy=(pi / 2, 0.0, 0.0)), material=dark, name=f"lid_hinge_knuckle_{i}")

    handle = model.part("pull_handle")
    handle.visual(Cylinder(radius=0.010, length=0.705), origin=Origin(xyz=(-0.030, -0.170, -0.1625)), material=steel, name="handle_tube_0")
    handle.visual(Cylinder(radius=0.010, length=0.705), origin=Origin(xyz=(-0.030, 0.170, -0.1625)), material=steel, name="handle_tube_1")
    handle.visual(Box((0.020, 0.060, 0.040)), origin=Origin(xyz=(-0.030, -0.170, -0.350)), material=dark, name="handle_slider_0")
    handle.visual(Box((0.020, 0.060, 0.040)), origin=Origin(xyz=(-0.030, 0.170, -0.350)), material=dark, name="handle_slider_1")
    handle.visual(Cylinder(radius=0.025, length=0.420), origin=Origin(xyz=(-0.030, 0.0, 0.215), rpy=(pi / 2, 0.0, 0.0)), material=dark, name="handle_grip")
    handle.visual(Box((0.035, 0.380, 0.030)), origin=Origin(xyz=(-0.030, 0.0, 0.185)), material=dark, name="handle_crossbar")

    latch = model.part("latch_button")
    latch.visual(Box((0.040, 0.060, 0.022)), origin=Origin(xyz=(-0.010, 0.0, 0.0)), material=dark, name="button_stem")
    latch.visual(Box((0.036, 0.140, 0.030)), origin=Origin(xyz=(0.021, 0.0, 0.0)), material=blue, name="button_face")

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.092,
            0.052,
            inner_radius=0.062,
            tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.003),),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.007, radius=0.003),
        ),
        "rear_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.062,
            0.050,
            rim=WheelRim(inner_radius=0.040, flange_height=0.005, flange_thickness=0.004, bead_seat_depth=0.003),
            hub=WheelHub(
                radius=0.025,
                width=0.040,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.030, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.004, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "rear_wheel",
    )
    for name, y in (("rear_wheel_0", 0.307), ("rear_wheel_1", -0.307)):
        wheel = model.part(name)
        wheel.visual(tire_mesh, origin=Origin(rpy=(0.0, 0.0, pi / 2)), material=black, name="tire")
        wheel.visual(wheel_mesh, origin=Origin(rpy=(0.0, 0.0, pi / 2)), material=steel, name="rim")
        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.325, y, 0.095)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=12.0),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.425, 0.0, 0.500)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_pull_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.425, 0.0, 0.525)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.420),
    )
    model.articulation(
        "lid_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.8425, 0.0, 0.034)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.10, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("pull_handle")
    latch = object_model.get_part("latch_button")
    wheel_0 = object_model.get_part("rear_wheel_0")
    wheel_1 = object_model.get_part("rear_wheel_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_slide = object_model.get_articulation("body_to_pull_handle")
    latch_slide = object_model.get_articulation("lid_to_latch_button")

    # Closed lid seats just above the lower tub, while its footprint covers the compartment.
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_side_skirt_0",
        negative_elem="top_rim_0",
        min_gap=0.0,
        max_gap=0.010,
        name="closed lid skirt seats on upper rim",
    )
    ctx.expect_overlap(lid, body, axes="xy", elem_a="lid_top", elem_b="bottom_pan", min_overlap=0.45, name="lid covers deep lower tub")

    # Telescoping handle remains captured in the two rear guide channels even when extended.
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="handle_tube_0",
        elem_b="handle_channel_web_0",
        min_overlap=0.250,
        name="stowed handle tube remains in guide channel",
    )
    stowed_handle = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: 0.420}):
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="handle_tube_0",
            elem_b="handle_channel_web_0",
            min_overlap=0.075,
            name="extended handle keeps retained insertion",
        )
        extended_handle = ctx.part_world_position(handle)
    ctx.check(
        "pull handle slides upward",
        stowed_handle is not None and extended_handle is not None and extended_handle[2] > stowed_handle[2] + 0.35,
        details=f"stowed={stowed_handle}, extended={extended_handle}",
    )

    closed_nose_aabb = ctx.part_element_world_aabb(lid, elem="lid_nose_top_rail")
    with ctx.pose({lid_hinge: 1.25}):
        open_nose_aabb = ctx.part_element_world_aabb(lid, elem="lid_nose_top_rail")
    ctx.check(
        "main lid rotates upward on rear hinge",
        closed_nose_aabb is not None
        and open_nose_aabb is not None
        and open_nose_aabb[1][2] > closed_nose_aabb[1][2] + 0.45,
        details=f"closed={closed_nose_aabb}, open={open_nose_aabb}",
    )

    rest_latch = ctx.part_world_position(latch)
    with ctx.pose({latch_slide: 0.018}):
        pressed_latch = ctx.part_world_position(latch)
    ctx.check(
        "front latch button presses into lid nose",
        rest_latch is not None and pressed_latch is not None and pressed_latch[0] < rest_latch[0] - 0.015,
        details=f"rest={rest_latch}, pressed={pressed_latch}",
    )

    ctx.check(
        "rear wheel joints are continuous spinners",
        object_model.get_articulation("body_to_rear_wheel_0").articulation_type == ArticulationType.CONTINUOUS
        and object_model.get_articulation("body_to_rear_wheel_1").articulation_type == ArticulationType.CONTINUOUS,
        details="both rear wheels must use continuous joints",
    )
    ctx.expect_origin_gap(wheel_0, wheel_1, axis="y", min_gap=0.55, max_gap=0.70, name="two rear wheels sit on opposite axle ends")

    return ctx.report()


object_model = build_object_model()
