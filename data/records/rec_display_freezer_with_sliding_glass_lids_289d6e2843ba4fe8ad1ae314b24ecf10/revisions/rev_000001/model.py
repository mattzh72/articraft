from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


BODY_L = 1.35
BODY_W = 0.74
BODY_H = 0.72
BODY_Z0 = 0.08
BODY_TOP_Z = BODY_Z0 + BODY_H
RAIL_TOP_Z = 0.846
LOWER_SLIDER_Z = 0.846
UPPER_SLIDER_Z = 0.880


def _deep_tub_shell() -> cq.Workplane:
    """Open, thick-walled chest-freezer tub authored as a true cavity."""
    wall = 0.055
    bottom = 0.115
    outer = cq.Workplane("XY").box(BODY_L, BODY_W, BODY_H, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(BODY_L - 2.0 * wall, BODY_W - 2.0 * wall, BODY_H - bottom + 0.04, centered=(True, True, False))
        .translate((0.0, 0.0, bottom))
    )
    shell = outer.cut(inner)
    # A modest top and corner radius keeps the plastic/insulated case from reading
    # as a razor-edged block while leaving the rail seats crisp.
    try:
        shell = shell.edges("|Z").fillet(0.018).edges(">Z").fillet(0.010)
    except Exception:
        pass
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_display_freezer")

    body_mat = model.material("warm_white_powdercoat", rgba=(0.93, 0.95, 0.93, 1.0))
    liner_mat = model.material("pale_liner", rgba=(0.74, 0.88, 0.96, 1.0))
    rail_mat = model.material("brushed_dark_aluminum", rgba=(0.30, 0.32, 0.34, 1.0))
    frame_mat = model.material("satin_aluminum_frame", rgba=(0.72, 0.74, 0.74, 1.0))
    glass_mat = model.material("cold_tinted_glass", rgba=(0.55, 0.82, 0.95, 0.34))
    black_mat = model.material("black_rubber", rgba=(0.02, 0.022, 0.024, 1.0))
    drain_mat = model.material("drain_cover_gray", rgba=(0.52, 0.55, 0.56, 1.0))
    decal_mat = model.material("printed_control_decal", rgba=(0.86, 0.88, 0.86, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_deep_tub_shell(), "deep_tub_shell", tolerance=0.002),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0)),
        material=body_mat,
        name="deep_tub_shell",
    )
    cabinet.visual(
        Box((1.18, 0.62, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=rail_mat,
        name="base_plinth",
    )
    cabinet.visual(
        Box((BODY_L - 0.18, BODY_W - 0.16, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0 + 0.122)),
        material=liner_mat,
        name="visible_tub_floor",
    )

    # Twin top rail assemblies: each side has a channel for the low and high
    # sliding glass panels.
    for y, side, base_name, outer_name, inner_name in [
        (-0.345, "front", "front_rail_base", "front_outer_lip", "front_inner_lip"),
        (0.345, "rear", "rear_rail_base", "rear_outer_lip", "rear_inner_lip"),
    ]:
        cabinet.visual(
            Box((1.39, 0.060, 0.030)),
            origin=Origin(xyz=(0.0, y, BODY_TOP_Z + 0.010)),
            material=rail_mat,
            name=base_name,
        )
        for offset, lip_name in [(-0.022, outer_name), (0.022, inner_name)]:
            cabinet.visual(
                Box((1.39, 0.010, 0.034)),
                origin=Origin(xyz=(0.0, y + offset, BODY_TOP_Z + 0.029)),
                material=rail_mat,
                name=lip_name,
            )

    # Short raised shoes under the higher bypass track give the rear glass panel
    # a real support path while leaving clearance over the lower panel.
    for y, shoe_name in [(-0.323, "front_upper_track"), (0.323, "rear_upper_track")]:
        cabinet.visual(
            Box((0.550, 0.012, 0.034)),
            origin=Origin(xyz=(0.390, y, (LOWER_SLIDER_Z + UPPER_SLIDER_Z) / 2.0)),
            material=rail_mat,
            name=shoe_name,
        )

    # End stops keep the sliders visually retained on their rails.
    for x, stop in [(-0.675, "end_stop_0"), (0.675, "end_stop_1")]:
        cabinet.visual(
            Box((0.025, 0.67, 0.032)),
            origin=Origin(xyz=(x, 0.0, BODY_TOP_Z + 0.030)),
            material=rail_mat,
            name=stop,
        )

    # Lower-corner drain recess and control dial decal are fixed to the shell;
    # the flap and knob below are separate articulated controls mounted to them.
    cabinet.visual(
        Box((0.150, 0.004, 0.118)),
        origin=Origin(xyz=(-0.520, -BODY_W / 2.0 + 0.002, 0.214)),
        material=black_mat,
        name="drain_pocket",
    )
    cabinet.visual(
        Cylinder(radius=0.060, length=0.006),
        origin=Origin(xyz=(BODY_L / 2.0 + 0.003, -0.225, 0.385), rpy=(0.0, pi / 2.0, 0.0)),
        material=decal_mat,
        name="control_dial",
    )

    frame_mesh = mesh_from_geometry(
        BezelGeometry(
            (0.625, 0.520),
            (0.750, 0.640),
            0.024,
            opening_shape="rounded_rect",
            outer_shape="rounded_rect",
            opening_corner_radius=0.020,
            outer_corner_radius=0.030,
        ),
        "slider_frame",
    )

    for index, x0, z0, handle_x, handle_z in [
        (0, -0.300, LOWER_SLIDER_Z, 0.280, 0.019),
        (1, 0.300, UPPER_SLIDER_Z, -0.280, 0.033),
    ]:
        slider = model.part(f"slider_{index}")
        slider.visual(
            frame_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=frame_mat,
            name="frame",
        )
        slider.visual(
            Box((0.660, 0.552, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=glass_mat,
            name="glass_pane",
        )
        slider.visual(
            Box((0.120, 0.035, 0.018)),
            origin=Origin(xyz=(handle_x, -0.210, handle_z)),
            material=frame_mat,
            name="finger_pull",
        )
        model.articulation(
            f"cabinet_to_slider_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=slider,
            origin=Origin(xyz=(x0, 0.0, z0)),
            axis=(1.0 if index == 0 else -1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.420),
        )

    drain_flap = model.part("drain_flap")
    drain_flap.visual(
        Box((0.124, 0.008, 0.110)),
        origin=Origin(xyz=(0.0, -0.004, -0.055)),
        material=drain_mat,
        name="flap_panel",
    )
    drain_flap.visual(
        Cylinder(radius=0.006, length=0.142),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail_mat,
        name="hinge_barrel",
    )
    drain_flap.visual(
        Box((0.050, 0.009, 0.016)),
        origin=Origin(xyz=(0.0, -0.011, -0.050)),
        material=rail_mat,
        name="pull_tab",
    )
    model.articulation(
        "cabinet_to_drain_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=drain_flap,
        origin=Origin(xyz=(-0.520, -BODY_W / 2.0, 0.270)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.2, lower=0.0, upper=1.10),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.062,
            0.030,
            body_style="skirted",
            top_diameter=0.050,
            edge_radius=0.0012,
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=25.0),
        ),
        "temperature_knob",
    )
    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_mat,
        name="shaft",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_mat,
        name="cap",
    )
    model.articulation(
        "cabinet_to_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(BODY_L / 2.0 + 0.006, -0.225, 0.385)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    slider_0 = object_model.get_part("slider_0")
    slider_1 = object_model.get_part("slider_1")
    drain_flap = object_model.get_part("drain_flap")
    knob = object_model.get_part("knob")
    slide_0 = object_model.get_articulation("cabinet_to_slider_0")
    slide_1 = object_model.get_articulation("cabinet_to_slider_1")
    flap_joint = object_model.get_articulation("cabinet_to_drain_flap")

    ctx.expect_gap(
        slider_0,
        cabinet,
        axis="z",
        positive_elem="frame",
        negative_elem="front_inner_lip",
        min_gap=0.0,
        max_gap=0.006,
        name="lower slider rides just above the lower rail lips",
    )
    ctx.expect_gap(
        slider_1,
        slider_0,
        axis="z",
        positive_elem="frame",
        negative_elem="frame",
        min_gap=0.006,
        max_gap=0.040,
        name="upper slider clears the lower slider in the bypass track",
    )
    ctx.expect_overlap(
        slider_0,
        cabinet,
        axes="xy",
        elem_a="glass_pane",
        elem_b="deep_tub_shell",
        min_overlap=0.50,
        name="first glass slider spans the freezer opening",
    )
    ctx.expect_overlap(
        slider_1,
        cabinet,
        axes="xy",
        elem_a="glass_pane",
        elem_b="deep_tub_shell",
        min_overlap=0.50,
        name="second glass slider spans the freezer opening",
    )

    ctx.expect_gap(
        cabinet,
        drain_flap,
        axis="y",
        positive_elem="drain_pocket",
        negative_elem="flap_panel",
        min_gap=0.0,
        max_gap=0.003,
        name="drain flap seats against the lower shell pocket",
    )
    ctx.expect_gap(
        knob,
        cabinet,
        axis="x",
        positive_elem="shaft",
        negative_elem="control_dial",
        min_gap=0.0,
        max_gap=0.002,
        name="knob shaft is mounted on the end-panel dial",
    )

    rest_slider_0 = ctx.part_world_position(slider_0)
    rest_slider_1 = ctx.part_world_position(slider_1)
    rest_flap_aabb = ctx.part_element_world_aabb(drain_flap, elem="flap_panel")
    with ctx.pose({slide_0: 0.30, slide_1: 0.30, flap_joint: 0.70}):
        open_slider_0 = ctx.part_world_position(slider_0)
        open_slider_1 = ctx.part_world_position(slider_1)
        open_flap_aabb = ctx.part_element_world_aabb(drain_flap, elem="flap_panel")

    ctx.check(
        "sliders translate in opposing directions",
        rest_slider_0 is not None
        and rest_slider_1 is not None
        and open_slider_0 is not None
        and open_slider_1 is not None
        and open_slider_0[0] > rest_slider_0[0] + 0.25
        and open_slider_1[0] < rest_slider_1[0] - 0.25,
        details=f"rest0={rest_slider_0}, open0={open_slider_0}, rest1={rest_slider_1}, open1={open_slider_1}",
    )
    rest_flap_y = None if rest_flap_aabb is None else (rest_flap_aabb[0][1] + rest_flap_aabb[1][1]) * 0.5
    open_flap_y = None if open_flap_aabb is None else (open_flap_aabb[0][1] + open_flap_aabb[1][1]) * 0.5
    ctx.check(
        "drain flap swings outward on its hinge",
        rest_flap_y is not None and open_flap_y is not None and open_flap_y < rest_flap_y - 0.025,
        details=f"rest_y={rest_flap_y}, open_y={open_flap_y}",
    )

    return ctx.report()


object_model = build_object_model()
