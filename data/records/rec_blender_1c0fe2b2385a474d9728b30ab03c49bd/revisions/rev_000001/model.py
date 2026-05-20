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


def _make_base_shell() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(0.32, 0.24, 0.094)
        .translate((0.0, 0.0, 0.047))
        .edges("|Z")
        .fillet(0.022)
    )
    pedestal = cq.Workplane("XY").box(0.18, 0.15, 0.014).translate((0.0, 0.0, 0.101))
    front_recess = cq.Workplane("XY").box(0.036, 0.126, 0.030).translate((0.146, 0.0, 0.041))
    slider_channel = cq.Workplane("XY").box(0.050, 0.108, 0.016).translate((0.145, 0.0, 0.041))
    return body.union(pedestal).cut(front_recess).cut(slider_channel)


def _make_pitcher_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .rect(0.145, 0.125)
        .workplane(offset=0.120)
        .rect(0.168, 0.148)
        .workplane(offset=0.116)
        .rect(0.190, 0.172)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.012))
        .rect(0.125, 0.105)
        .workplane(offset=0.116)
        .rect(0.150, 0.132)
        .workplane(offset=0.116)
        .rect(0.174, 0.156)
        .loft(combine=True)
    )
    return outer.cut(inner)


def _make_lid_shell() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(0.194, 0.188, 0.014)
        .translate((0.097, 0.0, 0.010))
        .edges("|Z")
        .fillet(0.008)
    )
    fill_cap = cq.Workplane("XY").cylinder(0.012, 0.034).translate((0.104, 0.0, 0.023))
    front_tab = cq.Workplane("XY").box(0.018, 0.050, 0.010).translate((0.194, 0.0, 0.008))
    return panel.union(fill_cap).union(front_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jug_blender")

    base_finish = model.material("base_finish", rgba=(0.15, 0.16, 0.18, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.05, 0.05, 0.06, 1.0))
    pitcher_glass = model.material("pitcher_glass", rgba=(0.78, 0.86, 0.92, 0.35))
    blade_metal = model.material("blade_metal", rgba=(0.78, 0.80, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shell(), "base_shell"),
        material=base_finish,
        name="base_shell",
    )

    pitcher = model.part("pitcher")
    pitcher.visual(
        mesh_from_cadquery(_make_pitcher_shell(), "pitcher_shell"),
        material=pitcher_glass,
        name="pitcher_shell",
    )
    pitcher.visual(
        Box((0.030, 0.044, 0.018)),
        origin=Origin(xyz=(-0.010, 0.090, 0.074)),
        material=trim_finish,
        name="handle_lower_mount",
    )
    pitcher.visual(
        Box((0.032, 0.048, 0.020)),
        origin=Origin(xyz=(-0.008, 0.092, 0.210)),
        material=trim_finish,
        name="handle_upper_mount",
    )
    pitcher.visual(
        Cylinder(radius=0.013, length=0.128),
        origin=Origin(xyz=(-0.008, 0.112, 0.142)),
        material=trim_finish,
        name="handle_grip",
    )
    pitcher.visual(
        Box((0.032, 0.056, 0.006)),
        origin=Origin(xyz=(0.100, 0.0, 0.233)),
        material=pitcher_glass,
        name="spout_lip",
    )
    pitcher.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=trim_finish,
        name="drive_socket",
    )
    pitcher.visual(
        Box((0.012, 0.046, 0.012)),
        origin=Origin(xyz=(-0.088, -0.064, 0.233)),
        material=trim_finish,
        name="hinge_pad_0",
    )
    pitcher.visual(
        Cylinder(radius=0.006, length=0.042),
        origin=Origin(xyz=(-0.094, -0.064, 0.233), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="hinge_barrel_0",
    )
    pitcher.visual(
        Box((0.012, 0.046, 0.012)),
        origin=Origin(xyz=(-0.088, 0.064, 0.233)),
        material=trim_finish,
        name="hinge_pad_1",
    )
    pitcher.visual(
        Cylinder(radius=0.006, length=0.042),
        origin=Origin(xyz=(-0.094, 0.064, 0.233), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="hinge_barrel_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shell(), "lid_shell"),
        material=trim_finish,
        name="lid_panel",
    )
    lid.visual(
        Box((0.138, 0.138, 0.008)),
        origin=Origin(xyz=(0.112, 0.0, -0.001)),
        material=trim_finish,
        name="lid_plug",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="hinge_barrel",
    )

    slider = model.part("slider")
    slider.visual(
        Box((0.044, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_finish,
        name="slider_rail",
    )
    slider.visual(
        Box((0.026, 0.036, 0.022)),
        origin=Origin(xyz=(0.027, 0.0, 0.003)),
        material=trim_finish,
        name="slider_knob",
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=blade_metal,
        name="hub",
    )
    blade.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=trim_finish,
        name="shaft",
    )
    blade.visual(
        Box((0.072, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(0.0, 0.22, 0.0)),
        material=blade_metal,
        name="lower_blade_a",
    )
    blade.visual(
        Box((0.072, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.011), rpy=(0.0, -0.22, math.pi / 2.0)),
        material=blade_metal,
        name="lower_blade_b",
    )
    blade.visual(
        Box((0.050, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.017), rpy=(0.0, -0.28, math.pi / 4.0)),
        material=blade_metal,
        name="upper_blade_a",
    )
    blade.visual(
        Box((0.050, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.016), rpy=(0.0, 0.28, -math.pi / 4.0)),
        material=blade_metal,
        name="upper_blade_b",
    )

    model.articulation(
        "base_to_pitcher",
        ArticulationType.FIXED,
        parent=base,
        child=pitcher,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=pitcher,
        child=lid,
        origin=Origin(xyz=(-0.097, 0.0, 0.233)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "speed_slider",
        ArticulationType.PRISMATIC,
        parent=base,
        child=slider,
        origin=Origin(xyz=(0.146, 0.0, 0.041)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.10,
            lower=-0.040,
            upper=0.040,
        ),
    )
    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=pitcher,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pitcher = object_model.get_part("pitcher")
    lid = object_model.get_part("lid")
    slider = object_model.get_part("slider")
    blade = object_model.get_part("blade")
    lid_hinge = object_model.get_articulation("lid_hinge")
    speed_slider = object_model.get_articulation("speed_slider")
    blade_spin = object_model.get_articulation("blade_spin")

    ctx.expect_overlap(
        lid,
        pitcher,
        axes="xy",
        elem_a="lid_panel",
        elem_b="pitcher_shell",
        min_overlap=0.14,
        name="lid covers the pitcher opening",
    )
    ctx.expect_gap(
        lid,
        pitcher,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="pitcher_shell",
        max_gap=0.010,
        max_penetration=0.0,
        name="closed lid sits on the pitcher rim",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    opened_lid_center = None
    if rest_lid_aabb is not None:
        rest_lid_center = tuple((rest_lid_aabb[0][i] + rest_lid_aabb[1][i]) * 0.5 for i in range(3))
    else:
        rest_lid_center = None

    with ctx.pose({lid_hinge: math.radians(90.0)}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
        if opened_lid_aabb is not None:
            opened_lid_center = tuple((opened_lid_aabb[0][i] + opened_lid_aabb[1][i]) * 0.5 for i in range(3))

    ctx.check(
        "lid rotates upward",
        rest_lid_center is not None
        and opened_lid_center is not None
        and opened_lid_center[2] > rest_lid_center[2] + 0.070,
        details=f"closed_center={rest_lid_center}, open_center={opened_lid_center}",
    )

    slider_limits = speed_slider.motion_limits
    lower_slider_pos = None
    upper_slider_pos = None
    if slider_limits is not None:
        with ctx.pose({speed_slider: slider_limits.lower}):
            lower_slider_pos = ctx.part_world_position(slider)
        with ctx.pose({speed_slider: slider_limits.upper}):
            upper_slider_pos = ctx.part_world_position(slider)

    ctx.check(
        "slider travels laterally across the front slot",
        lower_slider_pos is not None
        and upper_slider_pos is not None
        and upper_slider_pos[1] > lower_slider_pos[1] + 0.075
        and abs(upper_slider_pos[0] - lower_slider_pos[0]) < 0.002
        and abs(upper_slider_pos[2] - lower_slider_pos[2]) < 0.002,
        details=f"lower={lower_slider_pos}, upper={upper_slider_pos}",
    )
    ctx.allow_overlap(
        base,
        slider,
        elem_a="base_shell",
        elem_b="slider_rail",
        reason="The slider rail is intentionally represented as a retained member running inside the base slot cavity proxy.",
    )
    ctx.expect_overlap(
        slider,
        base,
        axes="xz",
        elem_a="slider_rail",
        elem_b="base_shell",
        min_overlap=0.012,
        name="slider rail remains captured in the base slot",
    )

    ctx.expect_within(
        blade,
        pitcher,
        axes="xy",
        outer_elem="pitcher_shell",
        margin=0.020,
        name="blade assembly stays inside the pitcher footprint",
    )
    ctx.check(
        "blade uses continuous rotation",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and blade_spin.motion_limits is not None
        and blade_spin.motion_limits.lower is None
        and blade_spin.motion_limits.upper is None
        and tuple(round(v, 6) for v in blade_spin.axis) == (0.0, 0.0, 1.0),
        details=(
            f"type={blade_spin.articulation_type}, axis={blade_spin.axis}, "
            f"limits={blade_spin.motion_limits}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
