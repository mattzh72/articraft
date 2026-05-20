from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_WIDTH = 0.19
BASE_DEPTH = 0.22
BASE_HEIGHT = 0.11

PITCHER_BOTTOM_WIDTH = 0.106
PITCHER_BOTTOM_DEPTH = 0.106
PITCHER_TOP_WIDTH = 0.152
PITCHER_TOP_DEPTH = 0.152
PITCHER_HEIGHT = 0.235
PITCHER_WALL = 0.004
PITCHER_BOTTOM_THICKNESS = 0.010

LID_WIDTH = 0.156
LID_THICKNESS = 0.014
LID_PLUG_WIDTH = 0.126

CAP_TOP_DIAMETER = 0.056
CAP_PLUG_DIAMETER = 0.044
CAP_TOP_HEIGHT = 0.010
CAP_PLUG_HEIGHT = 0.014

BLADE_HUB_RADIUS = 0.012
BLADE_HUB_HEIGHT = 0.008


def _base_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .rect(BASE_WIDTH, BASE_DEPTH)
        .workplane(offset=0.072)
        .rect(0.164, 0.188)
        .workplane(offset=BASE_HEIGHT - 0.072)
        .rect(0.144, 0.166)
        .loft(combine=True)
    )
    deck = cq.Workplane("XY").box(0.118, 0.118, 0.010).translate((0.0, 0.0, BASE_HEIGHT + 0.005))
    return shell.union(deck)


def _cap_shape() -> cq.Workplane:
    top_disk = (
        cq.Workplane("XY")
        .circle(0.5 * CAP_TOP_DIAMETER)
        .extrude(CAP_TOP_HEIGHT)
        .translate((0.0, 0.0, 0.014))
    )
    plug = (
        cq.Workplane("XY")
        .circle(0.5 * CAP_PLUG_DIAMETER)
        .extrude(CAP_PLUG_HEIGHT)
        .translate((0.0, 0.0, -CAP_PLUG_HEIGHT))
    )
    collar = cq.Workplane("XY").circle(0.018).extrude(0.014)
    grip_bar_x = cq.Workplane("XY").box(0.030, 0.008, 0.006).translate((0.0, 0.0, 0.027))
    grip_bar_y = cq.Workplane("XY").box(0.008, 0.030, 0.006).translate((0.0, 0.0, 0.027))
    return top_disk.union(plug).union(collar).union(grip_bar_x).union(grip_bar_y)


def _blade_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(BLADE_HUB_RADIUS)
        .extrude(BLADE_HUB_HEIGHT)
        .translate((0.0, 0.0, -0.5 * BLADE_HUB_HEIGHT))
    )
    blade_x = (
        cq.Workplane("XY")
        .box(0.076, 0.012, 0.0028)
        .translate((0.0, 0.0, 0.006))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 16.0)
    )
    blade_y = (
        cq.Workplane("XY")
        .box(0.012, 0.066, 0.0028)
        .translate((0.0, 0.0, 0.008))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -18.0)
    )
    fin = cq.Workplane("XY").box(0.010, 0.010, 0.012).translate((0.0, 0.0, 0.004))
    return hub.union(blade_x).union(blade_y).union(fin)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_jug_blender")

    base_finish = model.material("base_finish", rgba=(0.14, 0.14, 0.15, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.60, 0.63, 0.67, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.07, 0.07, 0.08, 1.0))
    pitcher_clear = model.material("pitcher_clear", rgba=(0.84, 0.90, 0.98, 0.30))
    handle_finish = model.material("handle_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.13, 0.13, 0.14, 1.0))
    cap_clear = model.material("cap_clear", rgba=(0.82, 0.88, 0.94, 0.55))
    blade_finish = model.material("blade_finish", rgba=(0.76, 0.78, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_shell"),
        material=base_finish,
        name="base_shell",
    )
    wall_rise = PITCHER_HEIGHT - PITCHER_BOTTOM_THICKNESS
    wall_flare = 0.5 * (PITCHER_TOP_WIDTH - PITCHER_BOTTOM_WIDTH)
    wall_angle = math.atan2(wall_flare, wall_rise)
    wall_slant = math.hypot(wall_rise, wall_flare)
    wall_mid = 0.5 * (0.5 * PITCHER_BOTTOM_WIDTH + 0.5 * PITCHER_TOP_WIDTH)
    wall_mid_z = PITCHER_BOTTOM_THICKNESS + 0.5 * wall_rise

    base.visual(
        Box((0.084, 0.018, 0.046)),
        origin=Origin(xyz=(0.0, -0.101, 0.058)),
        material=panel_finish,
        name="control_panel",
    )

    pitcher = model.part("pitcher")
    pitcher.visual(
        Box((0.112, 0.112, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=pitcher_clear,
        name="pitcher_floor",
    )
    pitcher.visual(
        Box((0.132, PITCHER_WALL, wall_slant)),
        origin=Origin(xyz=(0.0, -wall_mid, wall_mid_z), rpy=(wall_angle, 0.0, 0.0)),
        material=pitcher_clear,
        name="pitcher_front",
    )
    pitcher.visual(
        Box((0.132, PITCHER_WALL, wall_slant)),
        origin=Origin(xyz=(0.0, wall_mid, wall_mid_z), rpy=(-wall_angle, 0.0, 0.0)),
        material=pitcher_clear,
        name="pitcher_rear",
    )
    pitcher.visual(
        Box((PITCHER_WALL, 0.132, wall_slant)),
        origin=Origin(xyz=(wall_mid, 0.0, wall_mid_z), rpy=(0.0, wall_angle, 0.0)),
        material=pitcher_clear,
        name="pitcher_side_0",
    )
    pitcher.visual(
        Box((PITCHER_WALL, 0.132, wall_slant)),
        origin=Origin(xyz=(-wall_mid, 0.0, wall_mid_z), rpy=(0.0, -wall_angle, 0.0)),
        material=pitcher_clear,
        name="pitcher_side_1",
    )
    pitcher.visual(
        Box((0.040, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.080, PITCHER_HEIGHT - 0.004), rpy=(wall_angle, 0.0, 0.0)),
        material=pitcher_clear,
        name="spout_lip",
    )

    handle = model.part("handle")
    upper_bridge_len = 0.038
    lower_bridge_len = 0.034
    wall_thickness = PITCHER_WALL

    def wall_x_at(z_wall: float) -> float:
        return wall_mid + (z_wall - wall_mid_z) * math.tan(wall_angle)

    def bridge_origin(z_wall: float, length: float) -> Origin:
        offset = 0.5 * (wall_thickness + length)
        return Origin(
            xyz=(
                wall_x_at(z_wall) + offset * math.cos(wall_angle),
                0.0,
                z_wall - offset * math.sin(wall_angle),
            ),
            rpy=(0.0, wall_angle, 0.0),
        )

    handle.visual(
        Box((0.024, 0.032, 0.165)),
        origin=Origin(xyz=(0.122, 0.0, 0.126)),
        material=handle_finish,
        name="handle_grip",
    )
    handle.visual(
        Box((upper_bridge_len, 0.028, 0.022)),
        origin=bridge_origin(0.205, upper_bridge_len),
        material=handle_finish,
        name="handle_upper",
    )
    handle.visual(
        Box((lower_bridge_len, 0.026, 0.020)),
        origin=Origin(xyz=(0.095, 0.0, 0.078), rpy=(0.0, wall_angle, 0.0)),
        material=handle_finish,
        name="handle_lower",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_WIDTH, 0.052, LID_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.052, 0.5 * LID_THICKNESS)),
        material=lid_finish,
        name="lid_front",
    )
    lid.visual(
        Box((LID_WIDTH, 0.052, LID_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.052, 0.5 * LID_THICKNESS)),
        material=lid_finish,
        name="lid_rear",
    )
    lid.visual(
        Box((0.052, 0.052, LID_THICKNESS)),
        origin=Origin(xyz=(0.052, 0.0, 0.5 * LID_THICKNESS)),
        material=lid_finish,
        name="lid_side_0",
    )
    lid.visual(
        Box((0.052, 0.052, LID_THICKNESS)),
        origin=Origin(xyz=(-0.052, 0.0, 0.5 * LID_THICKNESS)),
        material=lid_finish,
        name="lid_side_1",
    )
    lid.visual(
        Box((LID_PLUG_WIDTH, 0.037, 0.006)),
        origin=Origin(xyz=(0.0, -0.0445, -0.003)),
        material=lid_finish,
        name="lid_plug_front",
    )
    lid.visual(
        Box((LID_PLUG_WIDTH, 0.037, 0.006)),
        origin=Origin(xyz=(0.0, 0.0445, -0.003)),
        material=lid_finish,
        name="lid_plug_rear",
    )
    lid.visual(
        Box((0.037, 0.052, 0.006)),
        origin=Origin(xyz=(0.0445, 0.0, -0.003)),
        material=lid_finish,
        name="lid_plug_side_0",
    )
    lid.visual(
        Box((0.037, 0.052, 0.006)),
        origin=Origin(xyz=(-0.0445, 0.0, -0.003)),
        material=lid_finish,
        name="lid_plug_side_1",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_cap_shape(), "center_cap"),
        material=cap_clear,
        name="cap_shell",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.030,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.056, 0.006, flare=0.05),
                grip=KnobGrip(style="fluted", count=14, depth=0.0010),
                indicator=KnobIndicator(
                    style="line",
                    mode="engraved",
                    depth=0.0008,
                    angle_deg=0.0,
                ),
                center=False,
            ),
            "speed_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="dial_shell",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_blade_shape(), "blade_assembly"),
        material=blade_finish,
        name="blade_shell",
    )

    model.articulation(
        "base_to_pitcher",
        ArticulationType.FIXED,
        parent=base,
        child=pitcher,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + 0.010)),
    )
    model.articulation(
        "pitcher_to_handle",
        ArticulationType.FIXED,
        parent=pitcher,
        child=handle,
        origin=Origin(),
    )
    model.articulation(
        "pitcher_to_lid",
        ArticulationType.FIXED,
        parent=pitcher,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, PITCHER_HEIGHT)),
    )
    model.articulation(
        "lid_to_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=0.45),
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.0, -0.110, 0.056)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0),
    )
    model.articulation(
        "pitcher_to_blade",
        ArticulationType.CONTINUOUS,
        parent=pitcher,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, PITCHER_BOTTOM_THICKNESS + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    pitcher = object_model.get_part("pitcher")
    lid = object_model.get_part("lid")
    cap = object_model.get_part("cap")
    handle = object_model.get_part("handle")
    dial = object_model.get_part("dial")
    blade = object_model.get_part("blade")

    cap_joint = object_model.get_articulation("lid_to_cap")
    dial_joint = object_model.get_articulation("base_to_dial")
    blade_joint = object_model.get_articulation("pitcher_to_blade")

    ctx.expect_contact(pitcher, base, name="pitcher seats on the motor base")
    ctx.expect_overlap(
        pitcher,
        base,
        axes="xy",
        min_overlap=0.090,
        name="pitcher footprint stays over the base deck",
    )
    ctx.expect_contact(handle, pitcher, name="pitcher handle mounts to the pitcher wall")
    ctx.expect_contact(lid, pitcher, name="lid contacts the pitcher rim")
    ctx.expect_overlap(
        lid,
        pitcher,
        axes="xy",
        min_overlap=0.120,
        name="lid covers the pitcher opening",
    )
    ctx.expect_within(
        blade,
        pitcher,
        axes="xy",
        margin=0.0,
        name="blade assembly stays inside the pitcher footprint",
    )
    ctx.expect_overlap(
        cap,
        lid,
        axes="xy",
        min_overlap=0.040,
        name="center cap remains seated in the lid opening",
    )
    ctx.expect_contact(dial, base, name="front dial mounts on the control face")

    cap_limits = cap_joint.motion_limits
    ctx.check(
        "center cap uses a short twist-lock travel",
        cap_limits is not None and cap_limits.lower == 0.0 and 0.20 <= cap_limits.upper <= 0.60,
        details=f"limits={cap_limits!r}",
    )
    ctx.check(
        "front dial is continuous",
        str(dial_joint.articulation_type).endswith("CONTINUOUS"),
        details=f"type={dial_joint.articulation_type!r}",
    )
    ctx.check(
        "blade assembly is continuous",
        str(blade_joint.articulation_type).endswith("CONTINUOUS"),
        details=f"type={blade_joint.articulation_type!r}",
    )

    pitch_pos = ctx.part_world_position(pitcher)
    blade_pos = ctx.part_world_position(blade)
    ctx.check(
        "blade sits near the pitcher floor",
        pitch_pos is not None
        and blade_pos is not None
        and pitch_pos[2] + 0.010 <= blade_pos[2] <= pitch_pos[2] + 0.030,
        details=f"pitcher={pitch_pos!r}, blade={blade_pos!r}",
    )

    cap_rest = ctx.part_world_position(cap)
    with ctx.pose({cap_joint: 0.35, dial_joint: 1.2, blade_joint: 8.0}):
        cap_turned = ctx.part_world_position(cap)
        dial_turned = ctx.part_world_position(dial)
        blade_turned = ctx.part_world_position(blade)

    dial_rest = ctx.part_world_position(dial)
    blade_rest = ctx.part_world_position(blade)

    ctx.check(
        "rotating controls stay on-axis",
        cap_rest is not None
        and cap_turned is not None
        and dial_rest is not None
        and dial_turned is not None
        and blade_rest is not None
        and blade_turned is not None
        and max(abs(cap_rest[i] - cap_turned[i]) for i in range(3)) < 1e-6
        and max(abs(dial_rest[i] - dial_turned[i]) for i in range(3)) < 1e-6
        and max(abs(blade_rest[i] - blade_turned[i]) for i in range(3)) < 1e-6,
        details=(
            f"cap_rest={cap_rest!r}, cap_turned={cap_turned!r}, "
            f"dial_rest={dial_rest!r}, dial_turned={dial_turned!r}, "
            f"blade_rest={blade_rest!r}, blade_turned={blade_turned!r}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
