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


def _add_rack(
    part,
    *,
    length: float,
    width: float,
    height: float,
    material,
    name_prefix: str,
) -> None:
    rail_t = 0.008
    slat_t = 0.006
    half_w = width * 0.5 - rail_t * 0.5
    base_z = 0.012
    rim_z = height

    part.visual(
        Box((length, rail_t, rail_t)),
        origin=Origin(xyz=(length * 0.5, -half_w, base_z)),
        material=material,
        name=f"{name_prefix}_base_left",
    )
    part.visual(
        Box((length, rail_t, rail_t)),
        origin=Origin(xyz=(length * 0.5, half_w, base_z)),
        material=material,
        name=f"{name_prefix}_base_right",
    )
    part.visual(
        Box((rail_t, width, rail_t)),
        origin=Origin(xyz=(rail_t * 0.5, 0.0, base_z)),
        material=material,
        name=f"{name_prefix}_base_rear",
    )
    part.visual(
        Box((rail_t, width, rail_t)),
        origin=Origin(xyz=(length - rail_t * 0.5, 0.0, base_z)),
        material=material,
        name=f"{name_prefix}_base_front",
    )

    part.visual(
        Box((length, rail_t, rail_t)),
        origin=Origin(xyz=(length * 0.5, -half_w, rim_z)),
        material=material,
        name=f"{name_prefix}_rim_left",
    )
    part.visual(
        Box((length, rail_t, rail_t)),
        origin=Origin(xyz=(length * 0.5, half_w, rim_z)),
        material=material,
        name=f"{name_prefix}_rim_right",
    )
    part.visual(
        Box((rail_t, width, rail_t)),
        origin=Origin(xyz=(rail_t * 0.5, 0.0, rim_z)),
        material=material,
        name=f"{name_prefix}_rim_rear",
    )
    part.visual(
        Box((rail_t, width, rail_t)),
        origin=Origin(xyz=(length - rail_t * 0.5, 0.0, rim_z)),
        material=material,
        name=f"{name_prefix}_rim_front",
    )

    post_x = (rail_t * 0.5, length - rail_t * 0.5)
    post_y = (-half_w, half_w)
    for ix, x in enumerate(post_x):
        for iy, y in enumerate(post_y):
            part.visual(
                Box((rail_t, rail_t, height)),
                origin=Origin(xyz=(x, y, height * 0.5)),
                material=material,
                name=f"{name_prefix}_post_{ix}_{iy}",
            )

    slat_positions = (
        length * 0.18,
        length * 0.34,
        length * 0.50,
        length * 0.66,
        length * 0.82,
    )
    for index, x in enumerate(slat_positions):
        part.visual(
            Box((slat_t, width - rail_t, slat_t)),
            origin=Origin(xyz=(x, 0.0, base_z + slat_t * 0.5)),
            material=material,
            name=f"{name_prefix}_slat_{index}",
        )


def _add_spray_arm(part, *, span: float, material, name_prefix: str) -> None:
    part.visual(
        Cylinder(radius=0.016, length=0.018),
        material=material,
        name=f"{name_prefix}_hub",
    )
    part.visual(
        Box((span, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=material,
        name=f"{name_prefix}_bar",
    )
    part.visual(
        Box((0.080, 0.014, 0.008)),
        origin=Origin(xyz=(-span * 0.08, 0.0, 0.005)),
        material=material,
        name=f"{name_prefix}_crossbar",
    )


def _add_button(part, *, material, name_prefix: str) -> None:
    part.visual(
        Box((0.006, 0.022, 0.009)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=material,
        name=f"{name_prefix}_cap",
    )
    part.visual(
        Box((0.004, 0.014, 0.006)),
        origin=Origin(xyz=(0.001, 0.0, -0.006)),
        material=material,
        name=f"{name_prefix}_stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slimline_dishwasher")

    panel_white = model.material("panel_white", rgba=(0.94, 0.95, 0.96, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.72, 0.74, 0.77, 1.0))
    tub_grey = model.material("tub_grey", rgba=(0.76, 0.78, 0.80, 1.0))
    rack_grey = model.material("rack_grey", rgba=(0.47, 0.50, 0.54, 1.0))
    control_black = model.material("control_black", rgba=(0.14, 0.15, 0.16, 1.0))
    spray_grey = model.material("spray_grey", rgba=(0.58, 0.61, 0.64, 1.0))

    body_width = 0.45
    body_depth = 0.54
    body_height = 0.82
    side_t = 0.018
    back_t = 0.030
    base_t = 0.050
    top_t = 0.032
    frame_t = 0.024

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((body_depth, side_t, body_height)),
        origin=Origin(xyz=(0.0, -(body_width * 0.5 - side_t * 0.5), body_height * 0.5)),
        material=panel_white,
        name="side_left",
    )
    cabinet.visual(
        Box((body_depth, side_t, body_height)),
        origin=Origin(xyz=(0.0, body_width * 0.5 - side_t * 0.5, body_height * 0.5)),
        material=panel_white,
        name="side_right",
    )
    cabinet.visual(
        Box((body_depth, body_width - 2.0 * side_t, top_t)),
        origin=Origin(xyz=(0.0, 0.0, body_height - top_t * 0.5)),
        material=panel_white,
        name="top_cap",
    )
    cabinet.visual(
        Box((body_depth, body_width - 2.0 * side_t, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t * 0.5)),
        material=tub_grey,
        name="base_pan",
    )
    cabinet.visual(
        Box((back_t, body_width - 2.0 * side_t, body_height - base_t - top_t)),
        origin=Origin(
            xyz=(
                -(body_depth * 0.5 - back_t * 0.5),
                0.0,
                base_t + (body_height - base_t - top_t) * 0.5,
            )
        ),
        material=tub_grey,
        name="back_panel",
    )
    cabinet.visual(
        Box((frame_t, body_width - 2.0 * side_t, 0.030)),
        origin=Origin(xyz=(body_depth * 0.5 - frame_t * 0.5, 0.0, body_height - 0.040)),
        material=tub_grey,
        name="front_rail",
    )
    cabinet.visual(
        Box((frame_t, side_t, body_height - 0.090)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 - frame_t * 0.5,
                -(body_width * 0.5 - side_t * 0.5),
                0.045 + (body_height - 0.090) * 0.5,
            )
        ),
        material=tub_grey,
        name="left_jamb",
    )
    cabinet.visual(
        Box((frame_t, side_t, body_height - 0.090)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 - frame_t * 0.5,
                body_width * 0.5 - side_t * 0.5,
                0.045 + (body_height - 0.090) * 0.5,
            )
        ),
        material=tub_grey,
        name="right_jamb",
    )
    cabinet.visual(
        Box((0.060, body_width - 2.0 * side_t, 0.065)),
        origin=Origin(xyz=(body_depth * 0.5 - 0.030, 0.0, 0.0325)),
        material=trim_grey,
        name="toe_kick",
    )
    for name, z in (
        ("lower_guide_left", 0.132),
        ("lower_guide_right", 0.132),
        ("upper_guide_left", 0.472),
        ("upper_guide_right", 0.472),
    ):
        y = -(body_width * 0.5 - side_t - 0.006) if "left" in name else body_width * 0.5 - side_t - 0.006
        cabinet.visual(
            Box((0.430, 0.012, 0.012)),
            origin=Origin(xyz=(0.015, y, z)),
            material=tub_grey,
            name=name,
        )

    door = model.part("door")
    door_height = 0.740
    door_thickness = 0.045
    door.visual(
        Box((door_thickness, 0.438, door_height)),
        origin=Origin(xyz=(0.0, 0.0, door_height * 0.5)),
        material=panel_white,
        name="outer_panel",
    )
    door.visual(
        Box((0.012, 0.390, 0.655)),
        origin=Origin(xyz=(-0.010, 0.0, 0.355)),
        material=tub_grey,
        name="inner_liner",
    )
    door.visual(
        Box((0.020, 0.018, 0.700)),
        origin=Origin(xyz=(-0.007, -0.190, 0.355)),
        material=tub_grey,
        name="inner_left_return",
    )
    door.visual(
        Box((0.020, 0.018, 0.700)),
        origin=Origin(xyz=(-0.007, 0.190, 0.355)),
        material=tub_grey,
        name="inner_right_return",
    )
    door.visual(
        Box((0.020, 0.382, 0.050)),
        origin=Origin(xyz=(-0.007, 0.0, 0.715)),
        material=tub_grey,
        name="inner_top_return",
    )
    door.visual(
        Box((0.020, 0.382, 0.060)),
        origin=Origin(xyz=(-0.007, 0.0, 0.030)),
        material=tub_grey,
        name="inner_bottom_return",
    )
    door.visual(
        Box((0.006, 0.310, 0.060)),
        origin=Origin(xyz=(0.020, 0.0, 0.705)),
        material=trim_grey,
        name="control_bank",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(body_depth * 0.5 + door_thickness * 0.5, 0.0, 0.045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    lower_rack = model.part("lower_rack")
    _add_rack(
        lower_rack,
        length=0.430,
        width=0.390,
        height=0.072,
        material=rack_grey,
        name_prefix="lower",
    )
    model.articulation(
        "cabinet_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_rack,
        origin=Origin(xyz=(-0.200, 0.0, 0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.250),
    )

    upper_rack = model.part("upper_rack")
    _add_rack(
        upper_rack,
        length=0.410,
        width=0.390,
        height=0.056,
        material=rack_grey,
        name_prefix="upper",
    )
    model.articulation(
        "cabinet_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_rack,
        origin=Origin(xyz=(-0.190, 0.0, 0.460)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.230),
    )

    mug_shelf = model.part("mug_shelf")
    shelf_width = 0.382
    shelf_height = 0.070
    mug_shelf.visual(
        Cylinder(radius=0.0035, length=shelf_width),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=rack_grey,
        name="pivot_rod",
    )
    mug_shelf.visual(
        Box((0.006, 0.006, shelf_height)),
        origin=Origin(xyz=(0.0, -(shelf_width * 0.5 - 0.003), shelf_height * 0.5)),
        material=rack_grey,
        name="side_left",
    )
    mug_shelf.visual(
        Box((0.006, 0.006, shelf_height)),
        origin=Origin(xyz=(0.0, shelf_width * 0.5 - 0.003, shelf_height * 0.5)),
        material=rack_grey,
        name="side_right",
    )
    mug_shelf.visual(
        Box((0.006, shelf_width - 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, shelf_height)),
        material=rack_grey,
        name="top_bar",
    )
    mug_shelf.visual(
        Box((0.006, shelf_width - 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, shelf_height * 0.55)),
        material=rack_grey,
        name="mid_bar",
    )
    model.articulation(
        "upper_rack_to_mug_shelf",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=mug_shelf,
        origin=Origin(xyz=(0.260, 0.0, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    lower_spray_arm = model.part("lower_spray_arm")
    _add_spray_arm(lower_spray_arm, span=0.255, material=spray_grey, name_prefix="lower_spray")
    lower_spray_arm.visual(
        Cylinder(radius=0.008, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.0225)),
        material=spray_grey,
        name="lower_spray_stem",
    )
    model.articulation(
        "cabinet_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.010, 0.0, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )

    upper_spray_arm = model.part("upper_spray_arm")
    _add_spray_arm(upper_spray_arm, span=0.225, material=spray_grey, name_prefix="upper_spray")
    upper_spray_arm.visual(
        Cylinder(radius=0.006, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=spray_grey,
        name="upper_spray_feed",
    )
    model.articulation(
        "upper_rack_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.205, 0.0, -0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=control_black,
        name="dial_knob",
    )
    selector_dial.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=control_black,
        name="dial_stem",
    )
    model.articulation(
        "door_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=selector_dial,
        origin=Origin(xyz=(door_thickness * 0.5, 0.0, 0.705)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    button_offsets = (-0.112, -0.070, 0.070, 0.112)
    for index, y in enumerate(button_offsets):
        button = model.part(f"button_{index}")
        _add_button(button, material=control_black, name_prefix=f"button_{index}")
        model.articulation(
            f"door_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(door_thickness * 0.5, y, 0.705)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.06,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    mug_shelf = object_model.get_part("mug_shelf")
    button_0 = object_model.get_part("button_0")
    object_model.get_part("button_1")
    object_model.get_part("button_2")
    object_model.get_part("button_3")

    door_hinge = object_model.get_articulation("cabinet_to_door")
    lower_rack_slide = object_model.get_articulation("cabinet_to_lower_rack")
    upper_rack_slide = object_model.get_articulation("cabinet_to_upper_rack")
    mug_shelf_hinge = object_model.get_articulation("upper_rack_to_mug_shelf")
    dial_joint = object_model.get_articulation("door_to_selector_dial")
    lower_spray_joint = object_model.get_articulation("cabinet_to_lower_spray_arm")
    upper_spray_joint = object_model.get_articulation("upper_rack_to_upper_spray_arm")
    button_joint = object_model.get_articulation("door_to_button_0")

    door_limits = door_hinge.motion_limits
    lower_limits = lower_rack_slide.motion_limits
    upper_limits = upper_rack_slide.motion_limits
    shelf_limits = mug_shelf_hinge.motion_limits
    button_limits = button_joint.motion_limits

    ctx.expect_overlap(
        door,
        cabinet,
        axes="yz",
        min_overlap=0.38,
        name="door covers the cabinet front footprint when closed",
    )
    ctx.expect_within(
        lower_rack,
        cabinet,
        axes="yz",
        margin=0.04,
        name="lower rack fits within the wash chamber envelope",
    )
    ctx.expect_within(
        upper_rack,
        cabinet,
        axes="yz",
        margin=0.05,
        name="upper rack fits within the wash chamber envelope",
    )

    if door_limits is not None and door_limits.upper is not None:
        closed_aabb = ctx.part_world_aabb(door)
        with ctx.pose({door_hinge: door_limits.upper}):
            open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door opens downward and outward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][0] > closed_aabb[1][0] + 0.14
            and open_aabb[1][2] < closed_aabb[1][2] - 0.14,
            details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
        )

    if lower_limits is not None and lower_limits.upper is not None and door_limits is not None and door_limits.upper is not None:
        lower_rest = ctx.part_world_position(lower_rack)
        with ctx.pose({door_hinge: door_limits.upper, lower_rack_slide: lower_limits.upper}):
            lower_extended = ctx.part_world_position(lower_rack)
            ctx.expect_overlap(
                lower_rack,
                cabinet,
                axes="x",
                min_overlap=0.16,
                name="lower rack remains engaged in the side guides when extended",
            )
            ctx.expect_within(
                lower_rack,
                cabinet,
                axes="yz",
                margin=0.04,
                name="lower rack stays centered while extended",
            )
        ctx.check(
            "lower rack slides outward",
            lower_rest is not None and lower_extended is not None and lower_extended[0] > lower_rest[0] + 0.18,
            details=f"rest={lower_rest}, extended={lower_extended}",
        )

    if upper_limits is not None and upper_limits.upper is not None and door_limits is not None and door_limits.upper is not None:
        upper_rest = ctx.part_world_position(upper_rack)
        with ctx.pose({door_hinge: door_limits.upper, upper_rack_slide: upper_limits.upper}):
            upper_extended = ctx.part_world_position(upper_rack)
            ctx.expect_overlap(
                upper_rack,
                cabinet,
                axes="x",
                min_overlap=0.14,
                name="upper rack remains engaged in the side guides when extended",
            )
            ctx.expect_within(
                upper_rack,
                cabinet,
                axes="yz",
                margin=0.05,
                name="upper rack stays centered while extended",
            )
        ctx.check(
            "upper rack slides outward",
            upper_rest is not None and upper_extended is not None and upper_extended[0] > upper_rest[0] + 0.16,
            details=f"rest={upper_rest}, extended={upper_extended}",
        )

    if (
        shelf_limits is not None
        and shelf_limits.upper is not None
        and upper_limits is not None
        and upper_limits.upper is not None
        and door_limits is not None
        and door_limits.upper is not None
    ):
        with ctx.pose({door_hinge: door_limits.upper, upper_rack_slide: upper_limits.upper * 0.55}):
            shelf_stowed = ctx.part_world_aabb(mug_shelf)
        with ctx.pose(
            {
                door_hinge: door_limits.upper,
                upper_rack_slide: upper_limits.upper * 0.55,
                mug_shelf_hinge: shelf_limits.upper,
            }
        ):
            shelf_deployed = ctx.part_world_aabb(mug_shelf)
        ctx.check(
            "mug shelf folds down toward the front",
            shelf_stowed is not None
            and shelf_deployed is not None
            and shelf_deployed[1][0] > shelf_stowed[1][0] + 0.04
            and shelf_deployed[1][2] < shelf_stowed[1][2] - 0.02,
            details=f"stowed={shelf_stowed}, deployed={shelf_deployed}",
        )

    if button_limits is not None and button_limits.upper is not None:
        button_rest = ctx.part_world_position(button_0)
        with ctx.pose({button_joint: button_limits.upper}):
            button_pressed = ctx.part_world_position(button_0)
        ctx.check(
            "front button presses inward",
            button_rest is not None and button_pressed is not None and button_pressed[0] < button_rest[0] - 0.001,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    ctx.check(
        "selector dial uses continuous rotation",
        dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"dial_limits={dial_joint.motion_limits}",
    )
    ctx.check(
        "spray arms use continuous rotation",
        lower_spray_joint.motion_limits is not None
        and lower_spray_joint.motion_limits.lower is None
        and lower_spray_joint.motion_limits.upper is None
        and upper_spray_joint.motion_limits is not None
        and upper_spray_joint.motion_limits.lower is None
        and upper_spray_joint.motion_limits.upper is None,
        details=f"lower_limits={lower_spray_joint.motion_limits}, upper_limits={upper_spray_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
