from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _steam_wand_mesh():
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.000, -0.010),
                (-0.006, 0.004, -0.026),
                (-0.010, 0.014, -0.086),
                (-0.016, 0.028, -0.145),
            ],
            radius=0.004,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "steam_wand_tube",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="espresso_machine")

    steel = model.material("steel", rgba=(0.80, 0.81, 0.83, 1.0))
    dark = model.material("dark", rgba=(0.14, 0.14, 0.15, 1.0))
    black = model.material("black", rgba=(0.07, 0.07, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.310, 0.290, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=steel,
        name="base",
    )
    body.visual(
        Box((0.006, 0.290, 0.270)),
        origin=Origin(xyz=(-0.152, 0.000, 0.153)),
        material=steel,
        name="left_wall",
    )
    body.visual(
        Box((0.006, 0.290, 0.270)),
        origin=Origin(xyz=(0.152, 0.000, 0.153)),
        material=steel,
        name="right_wall",
    )
    body.visual(
        Box((0.304, 0.006, 0.270)),
        origin=Origin(xyz=(0.000, -0.142, 0.153)),
        material=steel,
        name="rear_wall",
    )
    body.visual(
        Box((0.310, 0.020, 0.050)),
        origin=Origin(xyz=(0.000, 0.135, 0.071)),
        material=steel,
        name="front_apron",
    )
    body.visual(
        Box((0.110, 0.006, 0.200)),
        origin=Origin(xyz=(-0.100, 0.142, 0.188)),
        material=steel,
        name="front_left_panel",
    )
    body.visual(
        Box((0.110, 0.006, 0.200)),
        origin=Origin(xyz=(0.100, 0.142, 0.188)),
        material=steel,
        name="front_right_panel",
    )
    body.visual(
        Box((0.094, 0.006, 0.052)),
        origin=Origin(xyz=(0.000, 0.142, 0.262)),
        material=steel,
        name="front_top_panel",
    )
    body.visual(
        Box((0.274, 0.020, 0.012)),
        origin=Origin(xyz=(0.000, -0.129, 0.284)),
        material=steel,
        name="top_rear_strip",
    )
    body.visual(
        Box((0.274, 0.020, 0.012)),
        origin=Origin(xyz=(0.000, 0.067, 0.284)),
        material=steel,
        name="top_front_strip",
    )
    body.visual(
        Box((0.018, 0.178, 0.012)),
        origin=Origin(xyz=(-0.146, -0.021, 0.284)),
        material=steel,
        name="top_left_rail",
    )
    body.visual(
        Box((0.018, 0.178, 0.012)),
        origin=Origin(xyz=(0.146, -0.021, 0.284)),
        material=steel,
        name="top_right_rail",
    )
    body.visual(
        Box((0.090, 0.062, 0.045)),
        origin=Origin(xyz=(0.000, 0.112, 0.188)),
        material=steel,
        name="group_support",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.032),
        origin=Origin(xyz=(0.000, 0.133, 0.178), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="group_head",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.000, 0.142, 0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="brew_collar",
    )
    body.visual(
        Box((0.018, 0.180, 0.012)),
        origin=Origin(xyz=(-0.118, 0.032, 0.024)),
        material=dark,
        name="left_guide",
    )
    body.visual(
        Box((0.018, 0.180, 0.012)),
        origin=Origin(xyz=(0.118, 0.032, 0.024)),
        material=dark,
        name="right_guide",
    )
    body.visual(
        Box((0.240, 0.010, 0.012)),
        origin=Origin(xyz=(0.000, -0.058, 0.024)),
        material=dark,
        name="tray_stop",
    )
    body.visual(
        Box((0.028, 0.030, 0.055)),
        origin=Origin(xyz=(0.138, 0.131, 0.232)),
        material=steel,
        name="wand_mount",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.310, 0.290, 0.300)),
        mass=10.0,
        origin=Origin(xyz=(0.000, 0.000, 0.150)),
    )

    tank_lid = model.part("tank_lid")
    tank_lid.visual(
        Box((0.196, 0.158, 0.003)),
        origin=Origin(xyz=(0.000, 0.079, 0.0015)),
        material=steel,
        name="lid_panel",
    )
    tank_lid.visual(
        Box((0.080, 0.010, 0.014)),
        origin=Origin(xyz=(0.000, 0.129, 0.008)),
        material=dark,
        name="lid_handle",
    )
    tank_lid.inertial = Inertial.from_geometry(
        Box((0.196, 0.158, 0.018)),
        mass=0.6,
        origin=Origin(xyz=(0.000, 0.079, 0.009)),
    )
    model.articulation(
        "body_to_tank_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tank_lid,
        origin=Origin(xyz=(0.000, -0.100, 0.286)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=0.0,
            upper=1.35,
        ),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
        material=steel,
        name="lock_ring",
    )
    portafilter.visual(
        Cylinder(radius=0.029, length=0.030),
        origin=Origin(xyz=(0.000, 0.000, -0.025)),
        material=steel,
        name="basket",
    )
    portafilter.visual(
        Box((0.014, 0.012, 0.010)),
        origin=Origin(xyz=(0.026, 0.000, -0.006)),
        material=steel,
        name="lug_0",
    )
    portafilter.visual(
        Box((0.014, 0.012, 0.010)),
        origin=Origin(xyz=(-0.026, 0.000, -0.006)),
        material=steel,
        name="lug_1",
    )
    portafilter.visual(
        Cylinder(radius=0.013, length=0.026),
        origin=Origin(xyz=(0.000, 0.012, -0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="neck",
    )
    portafilter.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(0.000, 0.097, -0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="handle",
    )
    portafilter.visual(
        Box((0.020, 0.018, 0.010)),
        origin=Origin(xyz=(0.000, 0.026, -0.044)),
        material=steel,
        name="spout",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.080, 0.170, 0.060)),
        mass=0.8,
        origin=Origin(xyz=(0.000, 0.060, -0.022)),
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.000, 0.145, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.0,
            upper=0.20,
        ),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.284, 0.170, 0.003)),
        origin=Origin(xyz=(0.000, 0.085, 0.0015)),
        material=steel,
        name="tray_floor",
    )
    drip_tray.visual(
        Box((0.003, 0.170, 0.020)),
        origin=Origin(xyz=(-0.1405, 0.085, 0.005)),
        material=steel,
        name="tray_left_wall",
    )
    drip_tray.visual(
        Box((0.003, 0.170, 0.020)),
        origin=Origin(xyz=(0.1405, 0.085, 0.005)),
        material=steel,
        name="tray_right_wall",
    )
    drip_tray.visual(
        Box((0.284, 0.003, 0.020)),
        origin=Origin(xyz=(0.000, 0.0015, 0.005)),
        material=steel,
        name="tray_rear_wall",
    )
    drip_tray.visual(
        Box((0.284, 0.003, 0.020)),
        origin=Origin(xyz=(0.000, 0.1685, 0.005)),
        material=steel,
        name="tray_front_wall",
    )
    for index, y_pos in enumerate((0.025, 0.065, 0.105, 0.145)):
        drip_tray.visual(
            Box((0.278, 0.018, 0.004)),
            origin=Origin(xyz=(0.000, y_pos, 0.012)),
            material=dark,
            name=f"grate_{index}",
        )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.284, 0.170, 0.024)),
        mass=0.9,
        origin=Origin(xyz=(0.000, 0.085, 0.012)),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.000, -0.020, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.18,
            lower=0.0,
            upper=0.085,
        ),
    )

    steam_knob = model.part("steam_knob")
    steam_knob.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.000, 0.006, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="shaft",
    )
    steam_knob.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.000, 0.017, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob",
    )
    steam_knob.visual(
        Box((0.004, 0.004, 0.012)),
        origin=Origin(xyz=(0.000, 0.027, 0.015)),
        material=steel,
        name="indicator",
    )
    steam_knob.inertial = Inertial.from_geometry(
        Box((0.040, 0.032, 0.040)),
        mass=0.12,
        origin=Origin(xyz=(0.000, 0.016, 0.000)),
    )
    model.articulation(
        "body_to_steam_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=steam_knob,
        origin=Origin(xyz=(0.095, 0.145, 0.214)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=8.0,
        ),
    )

    paddle_switch = model.part("paddle_switch")
    paddle_switch.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.000, 0.005, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hub",
    )
    paddle_switch.visual(
        Box((0.020, 0.010, 0.044)),
        origin=Origin(xyz=(0.000, 0.011, -0.016)),
        material=black,
        name="paddle",
    )
    paddle_switch.visual(
        Box((0.022, 0.012, 0.006)),
        origin=Origin(xyz=(0.000, 0.018, -0.041)),
        material=dark,
        name="tip",
    )
    paddle_switch.inertial = Inertial.from_geometry(
        Box((0.026, 0.020, 0.054)),
        mass=0.08,
        origin=Origin(xyz=(0.000, 0.012, -0.020)),
    )
    model.articulation(
        "body_to_paddle_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=paddle_switch,
        origin=Origin(xyz=(0.050, 0.145, 0.214)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=2.0,
            lower=-0.75,
            upper=0.35,
        ),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=steel,
        name="pivot",
    )
    steam_wand.visual(
        _steam_wand_mesh(),
        material=steel,
        name="tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(-0.016, 0.028, -0.154)),
        material=steel,
        name="nozzle",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.180)),
        mass=0.18,
        origin=Origin(xyz=(-0.006, 0.008, -0.078)),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.138, 0.153, 0.246)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=1.8,
            lower=-1.30,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("tank_lid")
    portafilter = object_model.get_part("portafilter")
    tray = object_model.get_part("drip_tray")
    steam_knob = object_model.get_part("steam_knob")
    paddle = object_model.get_part("paddle_switch")
    wand = object_model.get_part("steam_wand")

    lid_joint = object_model.get_articulation("body_to_tank_lid")
    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    tray_joint = object_model.get_articulation("body_to_drip_tray")
    knob_joint = object_model.get_articulation("body_to_steam_knob")
    paddle_joint = object_model.get_articulation("body_to_paddle_switch")
    wand_joint = object_model.get_articulation("body_to_steam_wand")

    ctx.expect_overlap(
        tray,
        body,
        axes="x",
        elem_a="tray_floor",
        elem_b="left_guide",
        min_overlap=0.010,
        name="tray spans between body guides",
    )
    ctx.expect_gap(
        body,
        tray,
        axis="z",
        positive_elem="front_apron",
        negative_elem="tray_front_wall",
        min_gap=0.000,
        max_gap=0.070,
        name="tray sits below front apron",
    )

    lid_closed = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: lid_joint.motion_limits.upper}):
        lid_open = ctx.part_world_aabb(lid)
    ctx.check(
        "tank lid opens upward",
        lid_closed is not None
        and lid_open is not None
        and lid_open[1][2] > lid_closed[1][2] + 0.090,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    handle_locked = ctx.part_element_world_aabb(portafilter, elem="handle")
    with ctx.pose({portafilter_joint: -0.85}):
        handle_unlocked = ctx.part_element_world_aabb(portafilter, elem="handle")
    ctx.check(
        "portafilter rotates about brew axis",
        handle_locked is not None
        and handle_unlocked is not None
        and abs(
            ((handle_unlocked[0][0] + handle_unlocked[1][0]) * 0.5)
            - ((handle_locked[0][0] + handle_locked[1][0]) * 0.5)
        )
        > 0.045,
        details=f"locked={handle_locked}, unlocked={handle_unlocked}",
    )

    tray_home = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: tray_joint.motion_limits.upper}):
        tray_extended = ctx.part_world_position(tray)
    ctx.check(
        "drip tray slides forward",
        tray_home is not None
        and tray_extended is not None
        and tray_extended[1] > tray_home[1] + 0.060,
        details=f"home={tray_home}, extended={tray_extended}",
    )

    indicator_home = ctx.part_element_world_aabb(steam_knob, elem="indicator")
    with ctx.pose({knob_joint: math.pi / 2.0}):
        indicator_turned = ctx.part_element_world_aabb(steam_knob, elem="indicator")
    ctx.check(
        "steam knob rotates about front axis",
        indicator_home is not None
        and indicator_turned is not None
        and abs(
            ((indicator_turned[0][0] + indicator_turned[1][0]) * 0.5)
            - ((indicator_home[0][0] + indicator_home[1][0]) * 0.5)
        )
        > 0.008,
        details=f"home={indicator_home}, turned={indicator_turned}",
    )

    paddle_low = ctx.part_element_world_aabb(paddle, elem="tip")
    with ctx.pose({paddle_joint: paddle_joint.motion_limits.lower}):
        paddle_high = ctx.part_element_world_aabb(paddle, elem="tip")
    ctx.check(
        "paddle switch rocks on front pivot",
        paddle_low is not None
        and paddle_high is not None
        and abs(
            ((paddle_high[0][2] + paddle_high[1][2]) * 0.5)
            - ((paddle_low[0][2] + paddle_low[1][2]) * 0.5)
        )
        > 0.006,
        details=f"low={paddle_low}, high={paddle_high}",
    )

    nozzle_home = ctx.part_element_world_aabb(wand, elem="nozzle")
    with ctx.pose({wand_joint: wand_joint.motion_limits.lower}):
        nozzle_swung = ctx.part_element_world_aabb(wand, elem="nozzle")
    ctx.check(
        "steam wand swings from its support pivot",
        nozzle_home is not None
        and nozzle_swung is not None
        and abs(
            ((nozzle_swung[0][0] + nozzle_swung[1][0]) * 0.5)
            - ((nozzle_home[0][0] + nozzle_home[1][0]) * 0.5)
        )
        > 0.025,
        details=f"home={nozzle_home}, swung={nozzle_swung}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
