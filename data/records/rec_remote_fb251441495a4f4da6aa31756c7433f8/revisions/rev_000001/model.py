from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.038
BODY_L = 0.184
BODY_T = 0.016
BODY_R = 0.0065

FRONT_POCKET_W = 0.029
FRONT_POCKET_L = 0.144
FRONT_POCKET_D = 0.0008

NAV_Y = 0.028
POWER_Y = 0.073
VOICE_Y = 0.053
BACK_HOME_Y = -0.014
BACK_X = -0.009
HOME_X = 0.009
PLAY_Y = -0.047

BUTTON_TRAVEL = 0.0013
BUTTON_CAP_T = 0.0015
BUTTON_STEM_T = 0.0024
BUTTON_TOP_Z = BODY_T * 0.5 - 0.00005
BUTTON_CENTER_Z = BUTTON_TOP_Z - BUTTON_CAP_T * 0.5

ROCKER_Y = 0.020
ROCKER_Z = 0.0005

BATTERY_DOOR_W = 0.0292
BATTERY_DOOR_L = 0.092
BATTERY_DOOR_T = 0.0013
BATTERY_DOOR_Y = -0.010
BATTERY_DOOR_TRAVEL = 0.028
BATTERY_RECESS_D = 0.0015


def _rounded_plate(width: float, length: float, thickness: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(width, length, thickness).edges("|Z").fillet(radius)


def _housing_shape() -> cq.Workplane:
    body = _rounded_plate(BODY_W, BODY_L, BODY_T, BODY_R)

    front_pocket = (
        _rounded_plate(FRONT_POCKET_W, FRONT_POCKET_L, FRONT_POCKET_D * 2.4, 0.004)
        .translate((0.0, 0.004, BODY_T * 0.5 - FRONT_POCKET_D))
    )
    body = body.cut(front_pocket)

    nav_well = (
        cq.Workplane("XY")
        .circle(0.0145)
        .extrude(0.0036)
        .translate((0.0, NAV_Y, BODY_T * 0.5 - 0.0020))
    )
    body = body.cut(nav_well)

    center_button_well = (
        cq.Workplane("XY")
        .circle(0.0069)
        .extrude(0.0062)
        .translate((0.0, NAV_Y, BODY_T * 0.5 - 0.0031))
    )
    body = body.cut(center_button_well)

    for x_pos, y_pos, radius in (
        (0.0, POWER_Y, 0.0049),
        (0.0, VOICE_Y, 0.0056),
        (BACK_X, BACK_HOME_Y, 0.0052),
        (HOME_X, BACK_HOME_Y, 0.0052),
        (0.0, PLAY_Y, 0.0056),
    ):
        well = (
            cq.Workplane("XY")
            .circle(radius)
            .extrude(0.0060)
            .translate((x_pos, y_pos, BODY_T * 0.5 - 0.0030))
        )
        body = body.cut(well)

    rocker_pocket = (
        cq.Workplane("XY")
        .box(0.0032, 0.040, 0.0115)
        .translate((BODY_W * 0.5 - 0.0016, ROCKER_Y, ROCKER_Z))
    )
    body = body.cut(rocker_pocket)

    battery_recess = (
        _rounded_plate(BATTERY_DOOR_W + 0.0016, BATTERY_DOOR_L + 0.0035, BATTERY_RECESS_D * 2.4, 0.0038)
        .translate((0.0, BATTERY_DOOR_Y, -BODY_T * 0.5 + BATTERY_RECESS_D))
    )
    body = body.cut(battery_recess)

    finger_notch = (
        cq.Workplane("YZ")
        .circle(0.0058)
        .extrude(BODY_W * 0.70, both=True)
        .translate((0.0, BATTERY_DOOR_Y - BATTERY_DOOR_L * 0.5 + 0.008, -BODY_T * 0.5 + 0.0018))
    )
    body = body.cut(finger_notch)

    return body


def _battery_door_shape() -> cq.Workplane:
    door = _rounded_plate(BATTERY_DOOR_W, BATTERY_DOOR_L, BATTERY_DOOR_T, 0.0028)

    grip_pad = (
        cq.Workplane("XY")
        .box(BATTERY_DOOR_W * 0.44, 0.012, BATTERY_DOOR_T * 0.30)
        .translate((0.0, -BATTERY_DOOR_L * 0.5 + 0.014, -BATTERY_DOOR_T * 0.35))
    )
    door = door.union(grip_pad)

    guide_rail_height = 0.0007
    guide_rail_width = 0.0010
    guide_rail_span = BATTERY_DOOR_L * 0.74
    guide_rail_offset_x = (BATTERY_DOOR_W + 0.0016) * 0.5 - guide_rail_width * 0.5
    guide_rail_z = BATTERY_DOOR_T * 0.5 + guide_rail_height * 0.5 - 0.00005

    for rail_sign in (-1.0, 1.0):
        rail = (
            cq.Workplane("XY")
            .box(guide_rail_width, guide_rail_span, guide_rail_height)
            .translate((rail_sign * guide_rail_offset_x, 0.003, guide_rail_z))
        )
        door = door.union(rail)

    return door


def _nav_ring_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .circle(0.0140)
        .circle(0.0084)
        .extrude(0.0018)
    )
    locating_skirt = (
        cq.Workplane("XY")
        .circle(0.0128)
        .circle(0.0091)
        .extrude(0.0006)
        .translate((0.0, 0.0, -0.0006))
    )
    ring = ring.union(locating_skirt)
    for y_pos, x_pos, width, length in (
        (0.0082, 0.0, 0.0062, 0.0032),
        (-0.0082, 0.0, 0.0062, 0.0032),
        (0.0, 0.0082, 0.0032, 0.0062),
        (0.0, -0.0082, 0.0032, 0.0062),
    ):
        notch = (
            cq.Workplane("XY")
            .box(width, length, 0.0010)
            .translate((x_pos, y_pos, 0.00055))
        )
        ring = ring.cut(notch)
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="streaming_remote")

    body_black = model.material("body_black", rgba=(0.13, 0.14, 0.15, 1.0))
    button_black = model.material("button_black", rgba=(0.07, 0.07, 0.08, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.20, 0.22, 0.24, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "remote_housing"),
        material=body_black,
        name="shell",
    )

    nav_ring = model.part("nav_ring")
    nav_ring.visual(
        mesh_from_cadquery(_nav_ring_shape(), "nav_ring"),
        material=trim_grey,
        name="ring",
    )

    model.articulation(
        "housing_to_nav_ring",
        ArticulationType.FIXED,
        parent=housing,
        child=nav_ring,
        origin=Origin(xyz=(0.0, NAV_Y, BODY_T * 0.5 - 0.00140)),
    )

    def add_round_button(
        part_name: str,
        joint_name: str,
        *,
        x_pos: float,
        y_pos: float,
        radius: float,
        cap_thickness: float = BUTTON_CAP_T,
        stem_thickness: float = BUTTON_STEM_T,
        stem_radius_scale: float = 0.76,
        upper: float = BUTTON_TRAVEL,
    ) -> None:
        button = model.part(part_name)
        button.visual(
            Cylinder(radius=radius, length=cap_thickness),
            material=button_black,
            name="cap",
        )
        button.visual(
            Cylinder(radius=radius * stem_radius_scale, length=stem_thickness),
            origin=Origin(xyz=(0.0, 0.0, -(cap_thickness + stem_thickness) * 0.5 + 0.00012)),
            material=button_black,
            name="stem",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(x_pos, y_pos, BUTTON_CENTER_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.06,
                lower=0.0,
                upper=upper,
            ),
        )

    add_round_button(
        "center_button",
        "housing_to_center_button",
        x_pos=0.0,
        y_pos=NAV_Y,
        radius=0.0061,
        cap_thickness=0.0017,
        stem_thickness=0.0026,
        stem_radius_scale=0.78,
        upper=0.0015,
    )
    add_round_button(
        "power_button",
        "housing_to_power_button",
        x_pos=0.0,
        y_pos=POWER_Y,
        radius=0.0043,
        cap_thickness=0.0014,
        stem_thickness=0.0022,
        upper=0.0011,
    )
    add_round_button(
        "voice_button",
        "housing_to_voice_button",
        x_pos=0.0,
        y_pos=VOICE_Y,
        radius=0.0050,
        cap_thickness=0.0015,
        stem_thickness=0.0023,
        upper=0.0012,
    )
    add_round_button(
        "back_button",
        "housing_to_back_button",
        x_pos=BACK_X,
        y_pos=BACK_HOME_Y,
        radius=0.0046,
        cap_thickness=0.0014,
        stem_thickness=0.0022,
        upper=0.0012,
    )
    add_round_button(
        "home_button",
        "housing_to_home_button",
        x_pos=HOME_X,
        y_pos=BACK_HOME_Y,
        radius=0.0046,
        cap_thickness=0.0014,
        stem_thickness=0.0022,
        upper=0.0012,
    )
    add_round_button(
        "play_button",
        "housing_to_play_button",
        x_pos=0.0,
        y_pos=PLAY_Y,
        radius=0.0050,
        cap_thickness=0.0015,
        stem_thickness=0.0023,
        upper=0.0012,
    )

    volume_rocker = model.part("volume_rocker")
    volume_rocker.visual(
        Box((0.0024, 0.0175, 0.0092)),
        origin=Origin(xyz=(0.0, 0.0092, 0.0)),
        material=button_black,
        name="upper_pad",
    )
    volume_rocker.visual(
        Box((0.0024, 0.0175, 0.0092)),
        origin=Origin(xyz=(0.0, -0.0092, 0.0)),
        material=button_black,
        name="lower_pad",
    )
    volume_rocker.visual(
        Cylinder(radius=0.0041, length=0.0034),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim_grey,
        name="pivot",
    )
    volume_rocker.visual(
        Box((0.0020, 0.0058, 0.0100)),
        material=button_black,
        name="bridge",
    )
    volume_rocker.visual(
        Cylinder(radius=0.0017, length=0.0024),
        origin=Origin(xyz=(-0.00155, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim_grey,
        name="axle_stub",
    )

    model.articulation(
        "housing_to_volume_rocker",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=volume_rocker,
        origin=Origin(xyz=(BODY_W * 0.5 - 0.00045, ROCKER_Y, ROCKER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=-0.18,
            upper=0.18,
        ),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        mesh_from_cadquery(_battery_door_shape(), "battery_door"),
        material=button_black,
        name="door_panel",
    )

    model.articulation(
        "housing_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=battery_door,
        origin=Origin(
            xyz=(
                0.0,
                BATTERY_DOOR_Y,
                -BODY_T * 0.5 + BATTERY_DOOR_T * 0.5,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.12,
            lower=0.0,
            upper=BATTERY_DOOR_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    nav_ring = object_model.get_part("nav_ring")
    center_button = object_model.get_part("center_button")
    power_button = object_model.get_part("power_button")
    rocker = object_model.get_part("volume_rocker")
    battery_door = object_model.get_part("battery_door")
    center_joint = object_model.get_articulation("housing_to_center_button")
    power_joint = object_model.get_articulation("housing_to_power_button")
    rocker_joint = object_model.get_articulation("housing_to_volume_rocker")
    door_slide = object_model.get_articulation("housing_to_battery_door")

    ctx.expect_contact(
        nav_ring,
        housing,
        name="navigation ring seats on the front shell",
    )
    ctx.expect_within(
        battery_door,
        housing,
        axes="x",
        margin=0.001,
        name="battery door stays centered on the remote width",
    )
    ctx.expect_contact(
        battery_door,
        housing,
        name="battery door rests against the back shell",
    )

    rest_pos = ctx.part_world_position(battery_door)
    with ctx.pose({door_slide: BATTERY_DOOR_TRAVEL}):
        ctx.expect_within(
            battery_door,
            housing,
            axes="x",
            margin=0.001,
            name="battery door remains laterally guided when opened",
        )
        ctx.expect_contact(
            battery_door,
            housing,
            name="battery door stays on the shell track when opened",
        )
        open_pos = ctx.part_world_position(battery_door)

    ctx.check(
        "battery door slides toward the tail",
        rest_pos is not None and open_pos is not None and open_pos[1] < rest_pos[1] - 0.02,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    center_rest = ctx.part_world_position(center_button)
    with ctx.pose({center_joint: 0.0015}):
        center_pressed = ctx.part_world_position(center_button)
    ctx.check(
        "center button depresses inward",
        center_rest is not None and center_pressed is not None and center_pressed[2] < center_rest[2] - 0.0012,
        details=f"rest={center_rest}, pressed={center_pressed}",
    )

    power_rest = ctx.part_world_position(power_button)
    with ctx.pose({power_joint: 0.0011}):
        power_pressed = ctx.part_world_position(power_button)
    ctx.check(
        "power button depresses inward",
        power_rest is not None and power_pressed is not None and power_pressed[2] < power_rest[2] - 0.0009,
        details=f"rest={power_rest}, pressed={power_pressed}",
    )

    upper_rest = ctx.part_element_world_aabb(rocker, elem="upper_pad")
    lower_rest = ctx.part_element_world_aabb(rocker, elem="lower_pad")
    with ctx.pose({rocker_joint: 0.14}):
        upper_pressed = ctx.part_element_world_aabb(rocker, elem="upper_pad")
        lower_released = ctx.part_element_world_aabb(rocker, elem="lower_pad")
    rocker_ok = (
        upper_rest is not None
        and lower_rest is not None
        and upper_pressed is not None
        and lower_released is not None
        and upper_pressed[0][2] < upper_rest[0][2] - 0.0015
        and lower_released[1][2] > lower_rest[1][2] + 0.0015
    )
    ctx.check(
        "volume rocker pivots around its local hinge",
        rocker_ok,
        details=(
            f"upper_rest={upper_rest}, upper_pressed={upper_pressed}, "
            f"lower_rest={lower_rest}, lower_released={lower_released}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
