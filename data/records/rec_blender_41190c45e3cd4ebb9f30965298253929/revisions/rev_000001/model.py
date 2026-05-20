from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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


BASE_DEPTH = 0.18
BASE_WIDTH = 0.16
BASE_HEIGHT = 0.162
PANEL_THICKNESS = 0.004
BUTTON_TRAVEL = 0.0025


def _build_base_shape() -> cq.Workplane:
    lower_body = (
        cq.Workplane("XY")
        .box(0.18, 0.16, 0.11, centered=(True, True, False))
        .edges("|Z").fillet(0.014)
        .edges(">Z").fillet(0.010)
    )
    upper_body = (
        cq.Workplane("XY")
        .box(0.13, 0.13, 0.04, centered=(True, True, False))
        .translate((0.0, 0.0, 0.11))
        .edges("|Z").fillet(0.010)
        .edges(">Z").fillet(0.008)
    )
    coupler_seat = (
        cq.Workplane("XY")
        .circle(0.043)
        .extrude(0.012)
        .translate((0.0, 0.0, 0.15))
    )

    body = lower_body.union(upper_body).union(coupler_seat)

    control_cavity = (
        cq.Workplane("XY")
        .box(0.026, 0.094, 0.064, centered=True)
        .translate((0.077, 0.0, 0.056))
    )
    cable_relief = (
        cq.Workplane("XY")
        .box(0.024, 0.040, 0.018, centered=True)
        .translate((-0.088, 0.0, 0.026))
    )
    return body.cut(control_cavity).cut(cable_relief)


def _build_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(PANEL_THICKNESS, 0.092, 0.060, centered=True)

    button_centers = (-0.024, 0.0, 0.024)
    for y in button_centers:
        button_cut = cq.Workplane("YZ").center(y, -0.014).rect(0.0145, 0.0105).extrude(0.010, both=True)
        panel = panel.cut(button_cut)

    dial_cut = cq.Workplane("YZ").center(0.0, 0.015).circle(0.0065).extrude(0.010, both=True)
    return panel.cut(dial_cut)


def _build_pitcher_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .rect(0.080, 0.064)
        .workplane(offset=0.105)
        .rect(0.124, 0.098)
        .workplane(offset=0.100)
        .rect(0.108, 0.086)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .rect(0.072, 0.056)
        .workplane(offset=0.105)
        .rect(0.116, 0.090)
        .workplane(offset=0.093)
        .rect(0.100, 0.078)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.006))
    )

    shell = outer.cut(inner)
    pour_notch = (
        cq.Workplane("XY")
        .box(0.022, 0.030, 0.020, centered=True)
        .translate((0.050, 0.0, 0.198))
    )
    center_opening = cq.Workplane("XY").circle(0.013).extrude(0.026)
    shell = shell.cut(pour_notch).cut(center_opening)

    upper_bridge = (
        cq.Workplane("XY")
        .box(0.020, 0.050, 0.024, centered=(True, True, False))
        .translate((0.0, 0.072, 0.145))
        .edges("|Z").fillet(0.004)
    )
    lower_bridge = (
        cq.Workplane("XY")
        .box(0.020, 0.050, 0.024, centered=(True, True, False))
        .translate((0.0, 0.072, 0.062))
        .edges("|Z").fillet(0.004)
    )
    grip = (
        cq.Workplane("XY")
        .circle(0.010)
        .extrude(0.115)
        .translate((0.0, 0.094, 0.056))
    )

    return shell.union(upper_bridge).union(lower_bridge).union(grip).translate((0.0, 0.0, 0.015))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smoothie_blender")

    housing = model.material("housing", rgba=(0.93, 0.94, 0.96, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.10, 0.11, 0.13, 1.0))
    trim = model.material("trim", rgba=(0.18, 0.19, 0.21, 1.0))
    button_finish = model.material("button_finish", rgba=(0.78, 0.80, 0.83, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.13, 0.13, 0.14, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.78, 0.79, 0.80, 1.0))
    pitcher_clear = model.material("pitcher_clear", rgba=(0.76, 0.88, 0.96, 0.35))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base"),
        material=housing,
        name="base_shell",
    )

    panel = model.part("panel")
    panel.visual(
        mesh_from_cadquery(_build_panel_shape(), "panel"),
        material=panel_finish,
        name="panel_plate",
    )

    model.articulation(
        "base_to_panel",
        ArticulationType.FIXED,
        parent=base,
        child=panel,
        origin=Origin(xyz=(0.088, 0.0, 0.056)),
    )

    pitcher = model.part("pitcher")
    pitcher.visual(
        mesh_from_cadquery(_build_pitcher_shell(), "pitcher"),
        material=pitcher_clear,
        name="pitcher_shell",
    )
    pitcher.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").circle(0.046).circle(0.015).extrude(0.020),
            "pitcher_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim,
        name="pitcher_collar",
    )
    pitcher.visual(
        Box((0.036, 0.036, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=trim,
        name="pitcher_drive_block",
    )

    model.articulation(
        "base_to_pitcher",
        ArticulationType.FIXED,
        parent=base,
        child=pitcher,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
    )

    button_y_positions = (-0.024, 0.0, 0.024)
    for index, y_pos in enumerate(button_y_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.010, 0.012, 0.008)),
            origin=Origin(xyz=(-0.005, 0.0, 0.0)),
            material=trim,
            name="button_stem",
        )
        button.visual(
            Box((0.003, 0.0145, 0.0105)),
            origin=Origin(xyz=(0.0015, 0.0, 0.0)),
            material=button_finish,
            name="button_throat",
        )
        button.visual(
            Box((0.004, 0.020, 0.014)),
            origin=Origin(xyz=(0.002, 0.0, 0.0)),
            material=button_finish,
            name="button_cap",
        )
        model.articulation(
            f"panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=panel,
            child=button,
            origin=Origin(xyz=(PANEL_THICKNESS / 2.0, y_pos, -0.014)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="dial_shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.018,
                body_style="skirted",
                top_diameter=0.033,
                skirt=KnobSkirt(0.047, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=14, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_finish,
        name="dial_body",
    )

    model.articulation(
        "panel_to_dial",
        ArticulationType.CONTINUOUS,
        parent=panel,
        child=dial,
        origin=Origin(xyz=(PANEL_THICKNESS / 2.0, 0.0, 0.015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=trim,
        name="blade_spindle",
    )
    blade.visual(
        Cylinder(radius=0.012, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=trim,
        name="blade_hub",
    )
    for index, yaw in enumerate((math.radians(25.0), math.radians(115.0), math.radians(205.0), math.radians(295.0))):
        blade.visual(
            Box((0.050, 0.010, 0.002)),
            origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(0.0, 0.16, yaw)),
            material=blade_metal,
            name=f"blade_fin_{index}",
        )

    model.articulation(
        "pitcher_to_blade",
        ArticulationType.CONTINUOUS,
        parent=pitcher,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    panel = object_model.get_part("panel")
    pitcher = object_model.get_part("pitcher")
    dial = object_model.get_part("dial")
    blade = object_model.get_part("blade")

    dial_joint = object_model.get_articulation("panel_to_dial")
    blade_joint = object_model.get_articulation("pitcher_to_blade")

    ctx.expect_gap(
        pitcher,
        base,
        axis="z",
        min_gap=-0.0005,
        max_gap=0.001,
        positive_elem="pitcher_collar",
        negative_elem="base_shell",
        name="pitcher collar seats on the base deck",
    )
    ctx.expect_overlap(
        pitcher,
        base,
        axes="xy",
        min_overlap=0.070,
        elem_a="pitcher_collar",
        elem_b="base_shell",
        name="pitcher footprint is centered on the base",
    )
    ctx.expect_gap(
        panel,
        base,
        axis="x",
        min_gap=-0.0045,
        max_gap=0.0005,
        positive_elem="panel_plate",
        negative_elem="base_shell",
        name="control panel sits flush in the front cavity",
    )
    ctx.expect_within(
        blade,
        pitcher,
        axes="xy",
        margin=0.0,
        outer_elem="pitcher_shell",
        name="blade assembly stays inside the pitcher footprint",
    )
    ctx.expect_origin_gap(
        blade,
        pitcher,
        axis="z",
        min_gap=0.028,
        max_gap=0.032,
        name="blade assembly is mounted near the pitcher floor",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        ctx.expect_within(
            button,
            panel,
            axes="yz",
            margin=0.002,
            inner_elem="button_cap",
            outer_elem="panel_plate",
            name=f"button_{index} stays within the control panel bounds",
        )
        ctx.expect_gap(
            button,
            panel,
            axis="x",
            min_gap=0.0,
            max_gap=0.010,
            positive_elem="button_cap",
            negative_elem="panel_plate",
            name=f"button_{index} cap stays proud of the panel",
        )

    button_positions = {
        index: ctx.part_world_position(object_model.get_part(f"button_{index}"))
        for index in range(3)
    }

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"panel_to_button_{index}")

        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed = ctx.part_world_position(button)
            ctx.check(
                f"button_{index} depresses inward",
                button_positions[index] is not None
                and pressed is not None
                and pressed[0] < button_positions[index][0] - 0.0015,
                details=f"rest={button_positions[index]!r}, pressed={pressed!r}",
            )
            for other_index in range(3):
                if other_index == index:
                    continue
                other_button = object_model.get_part(f"button_{other_index}")
                other_rest = button_positions[other_index]
                other_pose = ctx.part_world_position(other_button)
                ctx.check(
                    f"button_{index} does not drag button_{other_index}",
                    other_rest is not None
                    and other_pose is not None
                    and abs(other_pose[0] - other_rest[0]) <= 1e-6,
                    details=f"rest={other_rest!r}, posed={other_pose!r}",
                )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_turned = ctx.part_world_position(dial)
    ctx.check(
        "dial joint is continuous",
        getattr(dial_joint, "articulation_type", None) == ArticulationType.CONTINUOUS
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"type={getattr(dial_joint, 'articulation_type', None)!r}, limits={dial_joint.motion_limits!r}",
    )
    ctx.check(
        "dial rotates about a fixed shaft center",
        dial_rest is not None
        and dial_turned is not None
        and abs(dial_rest[0] - dial_turned[0]) <= 1e-6
        and abs(dial_rest[1] - dial_turned[1]) <= 1e-6
        and abs(dial_rest[2] - dial_turned[2]) <= 1e-6,
        details=f"rest={dial_rest!r}, turned={dial_turned!r}",
    )

    blade_rest = ctx.part_world_position(blade)
    with ctx.pose({blade_joint: math.pi / 3.0}):
        ctx.expect_within(
            blade,
            pitcher,
            axes="xy",
            margin=0.0,
            outer_elem="pitcher_shell",
            name="blade assembly remains inside the pitcher when spinning",
        )
        blade_spun = ctx.part_world_position(blade)
    ctx.check(
        "blade joint is continuous",
        getattr(blade_joint, "articulation_type", None) == ArticulationType.CONTINUOUS
        and blade_joint.motion_limits is not None
        and blade_joint.motion_limits.lower is None
        and blade_joint.motion_limits.upper is None,
        details=f"type={getattr(blade_joint, 'articulation_type', None)!r}, limits={blade_joint.motion_limits!r}",
    )
    ctx.check(
        "blade hub spins around a fixed center",
        blade_rest is not None
        and blade_spun is not None
        and abs(blade_rest[0] - blade_spun[0]) <= 1e-6
        and abs(blade_rest[1] - blade_spun[1]) <= 1e-6
        and abs(blade_rest[2] - blade_spun[2]) <= 1e-6,
        details=f"rest={blade_rest!r}, spun={blade_spun!r}",
    )

    return ctx.report()


object_model = build_object_model()
