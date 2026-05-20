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


def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((low + high) * 0.5 for low, high in zip(aabb[0], aabb[1]))


def _add_rack_geometry(
    part,
    *,
    name_prefix: str,
    material,
    runner_x: float,
    outer_d: float,
    top_z: float,
    tine_height: float,
    tine_rows: tuple[float, ...],
) -> None:
    wire = 0.006
    runner_w = 0.016
    runner_h = 0.010
    base_z = -0.045
    front_y = 0.0
    runner_depth = outer_d - 0.052
    rear_y = runner_depth
    post_h = top_z - base_z

    _add_box_visual(
        part,
        name=f"{name_prefix}_runner_0",
        size=(runner_w, runner_depth, runner_h),
        xyz=(-runner_x, runner_depth * 0.5, 0.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{name_prefix}_runner_1",
        size=(runner_w, runner_depth, runner_h),
        xyz=(runner_x, runner_depth * 0.5, 0.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{name_prefix}_runner_web_0",
        size=(0.006, runner_depth, 0.045),
        xyz=(-runner_x, runner_depth * 0.5, -0.0225),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{name_prefix}_runner_web_1",
        size=(0.006, runner_depth, 0.045),
        xyz=(runner_x, runner_depth * 0.5, -0.0225),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{name_prefix}_front_base",
        size=(2.0 * runner_x + runner_w, wire, wire),
        xyz=(0.0, front_y, base_z),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{name_prefix}_rear_base",
        size=(2.0 * runner_x + runner_w, wire, wire),
        xyz=(0.0, rear_y, base_z),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{name_prefix}_left_base",
        size=(wire, rear_y - front_y, wire),
        xyz=(-runner_x, (front_y + rear_y) * 0.5, base_z),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{name_prefix}_right_base",
        size=(wire, rear_y - front_y, wire),
        xyz=(runner_x, (front_y + rear_y) * 0.5, base_z),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{name_prefix}_front_top",
        size=(2.0 * runner_x + runner_w, wire, wire),
        xyz=(0.0, front_y, top_z),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{name_prefix}_rear_top",
        size=(2.0 * runner_x + runner_w, wire, wire),
        xyz=(0.0, rear_y, top_z),
        material=material,
    )
    for x_sign in (-1.0, 1.0):
        for y_pos, suffix in ((front_y, "front"), (rear_y, "rear")):
            _add_box_visual(
                part,
                name=f"{name_prefix}_{suffix}_post_{int(x_sign > 0)}",
                size=(wire, wire, post_h),
                xyz=(x_sign * runner_x, y_pos, (top_z + base_z) * 0.5),
                material=material,
            )

    for index, y_pos in enumerate(tine_rows):
        _add_box_visual(
            part,
            name=f"{name_prefix}_cross_{index}",
            size=(2.0 * runner_x - 0.006, 0.004, 0.004),
            xyz=(0.0, y_pos, base_z),
            material=material,
        )

    tine_x_positions = (
        -runner_x + 0.055,
        -runner_x + 0.125,
        -0.055,
        0.055,
        runner_x - 0.125,
        runner_x - 0.055,
    )
    for row_index, y_pos in enumerate(tine_rows):
        for col_index, x_pos in enumerate(tine_x_positions):
            _add_box_visual(
                part,
                name=f"{name_prefix}_tine_{row_index}_{col_index}",
                size=(0.004, 0.004, tine_height),
                xyz=(x_pos, y_pos, base_z + tine_height * 0.5),
                material=material,
            )


def _add_spray_arm_geometry(
    part,
    *,
    material,
    name_prefix: str,
    arm_length: float,
    wing_length: float,
) -> None:
    part.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=material,
        name=f"{name_prefix}_hub",
    )
    _add_box_visual(
        part,
        name=f"{name_prefix}_main_arm",
        size=(arm_length, 0.028, 0.010),
        xyz=(0.0, 0.0, 0.008),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{name_prefix}_wing",
        size=(wing_length, 0.018, 0.008),
        xyz=(0.0, 0.019, 0.008),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_dishwasher")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.79, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.20, 0.21, 0.23, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    liner_grey = model.material("liner_grey", rgba=(0.70, 0.72, 0.74, 1.0))
    rack_grey = model.material("rack_grey", rgba=(0.82, 0.83, 0.84, 1.0))
    accent = model.material("accent", rgba=(0.56, 0.58, 0.60, 1.0))

    appliance_w = 0.60
    appliance_d = 0.62
    appliance_h = 0.86
    front_y = -appliance_d * 0.5

    cabinet = model.part("cabinet")
    side_t = 0.018
    top_t = 0.018
    back_t = 0.014

    _add_box_visual(
        cabinet,
        name="left_wall",
        size=(side_t, appliance_d - 0.050, appliance_h),
        xyz=(-(appliance_w - side_t) * 0.5, 0.025, appliance_h * 0.5),
        material=dark_grey,
    )
    _add_box_visual(
        cabinet,
        name="right_wall",
        size=(side_t, appliance_d - 0.050, appliance_h),
        xyz=((appliance_w - side_t) * 0.5, 0.025, appliance_h * 0.5),
        material=dark_grey,
    )
    _add_box_visual(
        cabinet,
        name="top_panel",
        size=(appliance_w - 2.0 * side_t, appliance_d, top_t),
        xyz=(0.0, 0.0, appliance_h - 0.5 * top_t),
        material=charcoal,
    )
    _add_box_visual(
        cabinet,
        name="back_panel",
        size=(appliance_w - 2.0 * side_t, back_t, appliance_h - 0.10),
        xyz=(0.0, appliance_d * 0.5 - 0.5 * back_t, 0.43),
        material=dark_grey,
    )
    _add_box_visual(
        cabinet,
        name="lower_rear_frame",
        size=(appliance_w - 2.0 * side_t, 0.10, 0.018),
        xyz=(0.0, 0.275, 0.078),
        material=charcoal,
    )
    _add_box_visual(
        cabinet,
        name="toe_kick",
        size=(appliance_w - 0.004, 0.050, 0.088),
        xyz=(0.0, front_y + 0.095, 0.044),
        material=black,
    )
    _add_box_visual(
        cabinet,
        name="hinge_rail",
        size=(appliance_w - 2.0 * side_t, 0.008, 0.020),
        xyz=(0.0, front_y + 0.059, 0.010),
        material=charcoal,
    )

    tub = model.part("tub")
    tub_outer_w = 0.564
    tub_outer_d = 0.548
    tub_outer_h = 0.720
    tub_shell_t = 0.010
    tub_front_y = front_y + 0.028
    tub_bottom_z = 0.090

    _add_box_visual(
        tub,
        name="left_shell",
        size=(tub_shell_t, tub_outer_d, tub_outer_h),
        xyz=(-(tub_outer_w * 0.5 - tub_shell_t * 0.5), tub_outer_d * 0.5, tub_outer_h * 0.5),
        material=liner_grey,
    )
    _add_box_visual(
        tub,
        name="right_shell",
        size=(tub_shell_t, tub_outer_d, tub_outer_h),
        xyz=((tub_outer_w * 0.5 - tub_shell_t * 0.5), tub_outer_d * 0.5, tub_outer_h * 0.5),
        material=liner_grey,
    )
    _add_box_visual(
        tub,
        name="floor_shell",
        size=(tub_outer_w - 2.0 * tub_shell_t, tub_outer_d, tub_shell_t),
        xyz=(0.0, tub_outer_d * 0.5, tub_shell_t * 0.5),
        material=liner_grey,
    )
    _add_box_visual(
        tub,
        name="roof_shell",
        size=(tub_outer_w - 2.0 * tub_shell_t, tub_outer_d, tub_shell_t),
        xyz=(0.0, tub_outer_d * 0.5, tub_outer_h - tub_shell_t * 0.5),
        material=liner_grey,
    )
    _add_box_visual(
        tub,
        name="back_shell",
        size=(tub_outer_w - 2.0 * tub_shell_t, tub_shell_t, tub_outer_h - 2.0 * tub_shell_t),
        xyz=(0.0, tub_outer_d - tub_shell_t * 0.5, tub_outer_h * 0.5),
        material=liner_grey,
    )
    _add_box_visual(
        tub,
        name="upper_rail_0",
        size=(0.016, 0.440, 0.012),
        xyz=(-(tub_outer_w * 0.5 - tub_shell_t - 0.008), 0.280, 0.500),
        material=accent,
    )
    _add_box_visual(
        tub,
        name="upper_rail_1",
        size=(0.016, 0.440, 0.012),
        xyz=((tub_outer_w * 0.5 - tub_shell_t - 0.008), 0.280, 0.500),
        material=accent,
    )
    _add_box_visual(
        tub,
        name="lower_rail_0",
        size=(0.016, 0.430, 0.014),
        xyz=(-(tub_outer_w * 0.5 - tub_shell_t - 0.008), 0.280, 0.225),
        material=accent,
    )
    _add_box_visual(
        tub,
        name="lower_rail_1",
        size=(0.016, 0.430, 0.014),
        xyz=((tub_outer_w * 0.5 - tub_shell_t - 0.008), 0.280, 0.225),
        material=accent,
    )
    _add_box_visual(
        tub,
        name="lower_hub_pedestal",
        size=(0.070, 0.080, 0.070),
        xyz=(0.0, 0.300, 0.035),
        material=liner_grey,
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.FIXED,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, tub_front_y, tub_bottom_z)),
    )

    door = model.part("door")
    door_w = 0.598
    door_h = 0.780
    door_t = 0.055
    control_band_h = 0.110

    _add_box_visual(
        door,
        name="front_panel",
        size=(door_w, 0.012, door_h),
        xyz=(0.0, 0.006, door_h * 0.5),
        material=stainless,
    )
    _add_box_visual(
        door,
        name="left_return",
        size=(0.012, 0.049, door_h),
        xyz=(-(door_w * 0.5 - 0.006), 0.0245, door_h * 0.5),
        material=dark_grey,
    )
    _add_box_visual(
        door,
        name="right_return",
        size=(0.012, 0.049, door_h),
        xyz=((door_w * 0.5 - 0.006), 0.0245, door_h * 0.5),
        material=dark_grey,
    )
    _add_box_visual(
        door,
        name="bottom_hem",
        size=(door_w - 0.024, 0.049, 0.016),
        xyz=(0.0, 0.0245, 0.008),
        material=dark_grey,
    )
    _add_box_visual(
        door,
        name="control_strip",
        size=(door_w, 0.022, control_band_h),
        xyz=(0.0, 0.011, door_h - control_band_h * 0.5),
        material=charcoal,
    )
    _add_box_visual(
        door,
        name="inner_liner",
        size=(0.532, 0.006, 0.580),
        xyz=(0.0, door_t - 0.003, 0.410),
        material=liner_grey,
    )
    _add_box_visual(
        door,
        name="liner_spine_0",
        size=(0.024, 0.040, 0.580),
        xyz=(-0.245, 0.030, 0.410),
        material=liner_grey,
    )
    _add_box_visual(
        door,
        name="liner_spine_1",
        size=(0.024, 0.040, 0.580),
        xyz=(0.245, 0.030, 0.410),
        material=liner_grey,
    )
    _add_box_visual(
        door,
        name="inner_basin",
        size=(0.300, 0.008, 0.080),
        xyz=(0.0, door_t - 0.004, 0.040),
        material=liner_grey,
    )
    _add_box_visual(
        door,
        name="cup_pocket",
        size=(0.160, 0.012, 0.090),
        xyz=(0.0, door_t - 0.006, 0.325),
        material=liner_grey,
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, front_y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.3,
            lower=0.0,
            upper=1.65,
        ),
    )

    detergent_cover = model.part("detergent_cover")
    _add_box_visual(
        detergent_cover,
        name="cover_panel",
        size=(0.145, 0.012, 0.070),
        xyz=(0.0, 0.006, -0.035),
        material=accent,
    )
    _add_box_visual(
        detergent_cover,
        name="cover_handle",
        size=(0.045, 0.016, 0.010),
        xyz=(0.0, 0.008, -0.065),
        material=dark_grey,
    )

    model.articulation(
        "door_to_detergent_cover",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_cover,
        origin=Origin(xyz=(0.0, door_t, 0.360)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    lower_rack = model.part("lower_rack")
    _add_rack_geometry(
        lower_rack,
        name_prefix="lower_rack",
        material=rack_grey,
        runner_x=0.248,
        outer_d=0.492,
        top_z=0.082,
        tine_height=0.090,
        tine_rows=(0.115, 0.205, 0.295, 0.385),
    )
    model.articulation(
        "tub_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.060, 0.225)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.30,
            lower=0.0,
            upper=0.330,
        ),
    )

    upper_rack = model.part("upper_rack")
    _add_rack_geometry(
        upper_rack,
        name_prefix="upper_rack",
        material=rack_grey,
        runner_x=0.248,
        outer_d=0.468,
        top_z=0.060,
        tine_height=0.060,
        tine_rows=(0.120, 0.235, 0.350),
    )
    model.articulation(
        "tub_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.060, 0.500)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.30,
            lower=0.0,
            upper=0.285,
        ),
    )

    lower_spray_arm = model.part("lower_spray_arm")
    _add_spray_arm_geometry(
        lower_spray_arm,
        material=accent,
        name_prefix="lower_spray_arm",
        arm_length=0.340,
        wing_length=0.120,
    )
    model.articulation(
        "tub_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, 0.300, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    upper_spray_arm = model.part("upper_spray_arm")
    _add_spray_arm_geometry(
        upper_spray_arm,
        material=accent,
        name_prefix="upper_spray_arm",
        arm_length=0.260,
        wing_length=0.090,
    )
    model.articulation(
        "upper_rack_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, 0.235, -0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    button_x_positions = (-0.185, -0.110, -0.035, 0.040, 0.115)
    for index, center_x in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        _add_box_visual(
            button,
            name="cap",
            size=(0.050, 0.005, 0.016),
            xyz=(0.0, -0.0025, 0.0),
            material=accent,
        )
        model.articulation(
            f"door_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(center_x, 0.0, 0.725)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.06,
                lower=0.0,
                upper=0.0025,
            ),
        )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="knob_body",
    )
    selector_knob.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=accent,
        name="knob_cap",
    )
    model.articulation(
        "door_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=selector_knob,
        origin=Origin(xyz=(0.220, 0.0, 0.725)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    door = object_model.get_part("door")
    tub = object_model.get_part("tub")
    cabinet = object_model.get_part("cabinet")
    detergent_cover = object_model.get_part("detergent_cover")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    upper_spray_arm = object_model.get_part("upper_spray_arm")
    button_0 = object_model.get_part("button_0")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    cover_hinge = object_model.get_articulation("door_to_detergent_cover")
    lower_rack_slide = object_model.get_articulation("tub_to_lower_rack")
    upper_rack_slide = object_model.get_articulation("tub_to_upper_rack")
    button_0_slide = object_model.get_articulation("door_to_button_0")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.50,
            name="closed door covers cabinet opening",
        )
        ctx.expect_overlap(
            door,
            tub,
            axes="xz",
            min_overlap=0.48,
            name="closed door covers tub opening",
        )
        ctx.expect_within(
            lower_rack,
            tub,
            axes="xz",
            margin=0.024,
            name="lower rack sits within tub envelope",
        )
        ctx.expect_within(
            upper_rack,
            tub,
            axes="xz",
            margin=0.024,
            name="upper rack sits within tub envelope",
        )

    closed_door_pos = _aabb_center(ctx.part_element_world_aabb(door, elem="front_panel"))
    with ctx.pose({door_hinge: 1.50}):
        open_door_pos = _aabb_center(ctx.part_element_world_aabb(door, elem="front_panel"))

    ctx.check(
        "door opens downward and outward",
        closed_door_pos is not None
        and open_door_pos is not None
        and open_door_pos[1] < closed_door_pos[1] - 0.18
        and open_door_pos[2] < closed_door_pos[2] - 0.15,
        details=f"closed={closed_door_pos}, open={open_door_pos}",
    )

    closed_cover_pos = _aabb_center(ctx.part_element_world_aabb(detergent_cover, elem="cover_panel"))
    with ctx.pose({cover_hinge: 1.0}):
        open_cover_pos = _aabb_center(ctx.part_element_world_aabb(detergent_cover, elem="cover_panel"))

    ctx.check(
        "detergent cover lifts off inner door",
        closed_cover_pos is not None
        and open_cover_pos is not None
        and open_cover_pos[1] > closed_cover_pos[1] + 0.015,
        details=f"closed={closed_cover_pos}, open={open_cover_pos}",
    )

    lower_rack_closed = ctx.part_world_position(lower_rack)
    with ctx.pose({lower_rack_slide: 0.330}):
        lower_rack_open = ctx.part_world_position(lower_rack)
        ctx.expect_within(
            lower_rack,
            tub,
            axes="xz",
            margin=0.024,
            name="lower rack stays centered on rails when extended",
        )
        ctx.expect_overlap(
            lower_rack,
            tub,
            axes="y",
            min_overlap=0.120,
            name="lower rack retains insertion when extended",
        )

    ctx.check(
        "lower rack pulls outward",
        lower_rack_closed is not None
        and lower_rack_open is not None
        and lower_rack_open[1] < lower_rack_closed[1] - 0.20,
        details=f"closed={lower_rack_closed}, open={lower_rack_open}",
    )

    upper_rack_closed = ctx.part_world_position(upper_rack)
    with ctx.pose({upper_rack_slide: 0.285}):
        upper_rack_open = ctx.part_world_position(upper_rack)
        ctx.expect_within(
            upper_rack,
            tub,
            axes="xz",
            margin=0.024,
            name="upper rack stays centered on rails when extended",
        )
        ctx.expect_overlap(
            upper_rack,
            tub,
            axes="y",
            min_overlap=0.120,
            name="upper rack retains insertion when extended",
        )

    ctx.check(
        "upper rack pulls outward",
        upper_rack_closed is not None
        and upper_rack_open is not None
        and upper_rack_open[1] < upper_rack_closed[1] - 0.16,
        details=f"closed={upper_rack_closed}, open={upper_rack_open}",
    )

    ctx.expect_overlap(
        upper_spray_arm,
        upper_rack,
        axes="xy",
        min_overlap=0.045,
        name="upper spray arm stays under upper rack center",
    )

    closed_button_pos = ctx.part_world_position(button_0)
    with ctx.pose({button_0_slide: 0.0025}):
        pressed_button_pos = ctx.part_world_position(button_0)
    ctx.check(
        "control button presses inward",
        closed_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > closed_button_pos[1] + 0.0015,
        details=f"closed={closed_button_pos}, pressed={pressed_button_pos}",
    )

    ctx.check(
        "five control buttons are separate parts",
        all(object_model.get_part(f"button_{index}") is not None for index in range(5)),
        details="Expected button_0 through button_4 parts.",
    )

    return ctx.report()


object_model = build_object_model()
