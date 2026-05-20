from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    mesh_from_geometry,
)

import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="end_cap_display_freezer")

    white = Material("warm_white_insulated_shell", rgba=(0.88, 0.91, 0.90, 1.0))
    plinth_mat = Material("dark_recessed_plinth", rgba=(0.08, 0.09, 0.10, 1.0))
    inner_mat = Material("dark_cold_well", rgba=(0.05, 0.07, 0.09, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.72, 0.76, 0.77, 1.0))
    black = Material("black_rubber_gasket", rgba=(0.01, 0.012, 0.014, 1.0))
    glass = Material("blue_tinted_glass", rgba=(0.55, 0.85, 1.0, 0.38))
    panel_mat = Material("black_control_panel", rgba=(0.02, 0.025, 0.03, 1.0))
    screen_mat = Material("blue_lit_display", rgba=(0.05, 0.30, 0.55, 1.0))
    tick_mat = Material("white_panel_markings", rgba=(0.92, 0.95, 0.93, 1.0))
    knob_mat = Material("matte_gray_knob", rgba=(0.34, 0.36, 0.36, 1.0))
    metal = Material("lock_chrome", rgba=(0.75, 0.76, 0.74, 1.0))

    cabinet = model.part("cabinet")

    # Grocery-store end-cap scale: a long insulated bin with a shallow kick plinth.
    cabinet.visual(Box((1.72, 0.70, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.05)), material=plinth_mat, name="plinth")
    cabinet.visual(Box((1.80, 0.82, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.14)), material=white, name="bottom_insulation")
    cabinet.visual(Box((1.64, 0.62, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.205)), material=inner_mat, name="recessed_well")
    cabinet.visual(Box((1.80, 0.08, 0.64)), origin=Origin(xyz=(0.0, 0.37, 0.50)), material=white, name="side_wall")
    cabinet.visual(Box((1.80, 0.08, 0.64)), origin=Origin(xyz=(0.0, -0.37, 0.50)), material=white, name="side_wall_1")
    cabinet.visual(Box((0.08, 0.82, 0.64)), origin=Origin(xyz=(-0.86, 0.0, 0.50)), material=white, name="end_wall")
    cabinet.visual(Box((0.08, 0.82, 0.64)), origin=Origin(xyz=(0.86, 0.0, 0.50)), material=white, name="rear_wall")

    # Low top rims and two raised sliding tracks on each long side.  The outer
    # lower track and inner upper track let the glass panels pass above each
    # other without colliding, as on supermarket sliding-top freezers.
    cabinet.visual(Box((1.80, 0.085, 0.04)), origin=Origin(xyz=(0.0, 0.3675, 0.84)), material=white, name="top_rim")
    cabinet.visual(Box((1.80, 0.085, 0.04)), origin=Origin(xyz=(0.0, -0.3675, 0.84)), material=white, name="top_rim_1")
    cabinet.visual(Box((1.70, 0.045, 0.044)), origin=Origin(xyz=(0.0, 0.3075, 0.88)), material=aluminum, name="lower_track")
    cabinet.visual(Box((1.70, 0.045, 0.044)), origin=Origin(xyz=(0.0, -0.3075, 0.88)), material=aluminum, name="lower_track_1")
    cabinet.visual(Box((1.70, 0.055, 0.074)), origin=Origin(xyz=(0.0, 0.3775, 0.897)), material=aluminum, name="upper_track")
    cabinet.visual(Box((1.70, 0.055, 0.074)), origin=Origin(xyz=(0.0, -0.3775, 0.897)), material=aluminum, name="upper_track_1")

    # Side lock cylinder and fixed hinge leaf for the protective flap.
    cabinet.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.45, 0.416, 0.55), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="key_cylinder",
    )
    cabinet.visual(Box((0.008, 0.003, 0.036)), origin=Origin(xyz=(0.45, 0.4235, 0.55)), material=black, name="key_slot")
    cabinet.visual(Box((0.024, 0.014, 0.110)), origin=Origin(xyz=(0.39, 0.417, 0.55)), material=metal, name="flap_hinge_leaf")

    def add_glass_top(
        name: str,
        *,
        x_center: float,
        z_center: float,
        y_side: float,
        glass_width: float,
        track_axis: tuple[float, float, float],
    ):
        lid = model.part(name)
        total_x = 0.84
        side_bar_w = 0.035
        end_bar_w = 0.035
        frame_thick = 0.030
        glass_x = total_x - 2.0 * end_bar_w

        lid.visual(Box((glass_x, glass_width, 0.012)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=glass, name="glass_pane")
        lid.visual(Box((total_x, side_bar_w, frame_thick)), origin=Origin(xyz=(0.0, y_side, 0.0)), material=aluminum, name="side_frame")
        lid.visual(Box((total_x, side_bar_w, frame_thick)), origin=Origin(xyz=(0.0, -y_side, 0.0)), material=aluminum, name="side_frame_1")
        lid.visual(Box((end_bar_w, side_bar_w, frame_thick)), origin=Origin(xyz=(total_x / 2.0 - end_bar_w / 2.0, y_side, 0.0)), material=aluminum, name="end_frame")
        lid.visual(Box((end_bar_w, side_bar_w, frame_thick)), origin=Origin(xyz=(-total_x / 2.0 + end_bar_w / 2.0, y_side, 0.0)), material=aluminum, name="end_frame_1")
        lid.visual(Box((end_bar_w, side_bar_w, frame_thick)), origin=Origin(xyz=(total_x / 2.0 - end_bar_w / 2.0, -y_side, 0.0)), material=aluminum, name="end_frame_2")
        lid.visual(Box((end_bar_w, side_bar_w, frame_thick)), origin=Origin(xyz=(-total_x / 2.0 + end_bar_w / 2.0, -y_side, 0.0)), material=aluminum, name="end_frame_3")
        lid.visual(Box((0.16, 0.018, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=black, name="finger_pull")

        joint = model.articulation(
            f"cabinet_to_{name}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=lid,
            origin=Origin(xyz=(x_center, 0.0, z_center)),
            axis=track_axis,
            motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.52),
        )
        return lid, joint

    lower_lid, lower_slide = add_glass_top(
        "glass_top_0",
        x_center=-0.43,
        z_center=0.917,
        y_side=0.2900,
        glass_width=0.545,
        track_axis=(1.0, 0.0, 0.0),
    )
    upper_lid, upper_slide = add_glass_top(
        "glass_top_1",
        x_center=0.43,
        z_center=0.949,
        y_side=0.3500,
        glass_width=0.665,
        track_axis=(-1.0, 0.0, 0.0),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(Box((0.025, 0.40, 0.20)), origin=Origin(), material=panel_mat, name="panel_plate")
    control_panel.visual(Box((0.004, 0.140, 0.045)), origin=Origin(xyz=(-0.0145, -0.095, 0.040)), material=screen_mat, name="display_window")
    control_panel.visual(Box((0.004, 0.018, 0.018)), origin=Origin(xyz=(-0.0145, -0.005, 0.080)), material=tick_mat, name="cold_mark")
    control_panel.visual(Box((0.004, 0.010, 0.035)), origin=Origin(xyz=(-0.0145, 0.095, 0.080)), material=tick_mat, name="tick_mark")
    control_panel.visual(Box((0.004, 0.035, 0.010)), origin=Origin(xyz=(-0.0145, 0.135, 0.035)), material=tick_mat, name="tick_mark_1")
    control_panel.visual(Box((0.004, 0.035, 0.010)), origin=Origin(xyz=(-0.0145, 0.055, 0.035)), material=tick_mat, name="tick_mark_2")
    model.articulation(
        "cabinet_to_control_panel",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_panel,
        origin=Origin(xyz=(-0.9125, 0.0, 0.47)),
    )

    temperature_knob = model.part("temperature_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.062,
            0.034,
            body_style="skirted",
            top_diameter=0.050,
            grip=KnobGrip(style="fluted", count=20, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "temperature_knob",
    )
    temperature_knob.visual(
        knob_mesh,
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=knob_mat,
        name="knob_cap",
    )
    model.articulation(
        "panel_to_temperature_knob",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=temperature_knob,
        origin=Origin(xyz=(-0.0125, 0.095, 0.005)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    lock_flap = model.part("lock_flap")
    lock_flap.visual(Box((0.090, 0.012, 0.090)), origin=Origin(xyz=(0.045, 0.000, 0.000)), material=metal, name="cover_plate")
    lock_flap.visual(Cylinder(radius=0.008, length=0.095), origin=Origin(), material=metal, name="hinge_barrel")
    lock_flap.visual(Box((0.018, 0.010, 0.050)), origin=Origin(xyz=(0.020, 0.001, 0.0)), material=metal, name="hinge_web")
    model.articulation(
        "cabinet_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lock_flap,
        origin=Origin(xyz=(0.39, 0.432, 0.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lower_lid = object_model.get_part("glass_top_0")
    upper_lid = object_model.get_part("glass_top_1")
    panel = object_model.get_part("control_panel")
    knob = object_model.get_part("temperature_knob")
    flap = object_model.get_part("lock_flap")

    lower_slide = object_model.get_articulation("cabinet_to_glass_top_0")
    upper_slide = object_model.get_articulation("cabinet_to_glass_top_1")
    flap_hinge = object_model.get_articulation("cabinet_to_lock_flap")

    ctx.expect_gap(
        lower_lid,
        cabinet,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="side_frame",
        negative_elem="lower_track",
        name="lower glass top rides on lower rail",
    )
    ctx.expect_gap(
        upper_lid,
        cabinet,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="side_frame",
        negative_elem="upper_track",
        name="upper glass top rides on upper rail",
    )
    ctx.expect_overlap(
        lower_lid,
        cabinet,
        axes="xy",
        min_overlap=0.015,
        elem_a="side_frame",
        elem_b="lower_track",
        name="lower lid side frame is captured by the long rail",
    )
    ctx.expect_overlap(
        upper_lid,
        cabinet,
        axes="xy",
        min_overlap=0.015,
        elem_a="side_frame",
        elem_b="upper_track",
        name="upper lid side frame is captured by the long rail",
    )

    ctx.expect_gap(
        cabinet,
        panel,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="end_wall",
        negative_elem="panel_plate",
        name="control panel is a separate end-wall module",
    )
    ctx.expect_gap(
        panel,
        knob,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="panel_plate",
        negative_elem="knob_cap",
        name="temperature knob seats on the panel face",
    )
    ctx.expect_gap(
        flap,
        cabinet,
        axis="y",
        min_gap=0.002,
        max_gap=0.010,
        positive_elem="cover_plate",
        negative_elem="key_cylinder",
        name="lock flap covers the key cylinder with a small clearance",
    )

    lower_rest = ctx.part_world_position(lower_lid)
    upper_rest = ctx.part_world_position(upper_lid)
    with ctx.pose({lower_slide: 0.52, upper_slide: 0.52}):
        lower_open = ctx.part_world_position(lower_lid)
        upper_open = ctx.part_world_position(upper_lid)

    ctx.check(
        "lower glass top slides toward the rear",
        lower_rest is not None and lower_open is not None and lower_open[0] > lower_rest[0] + 0.45,
        details=f"rest={lower_rest}, open={lower_open}",
    )
    ctx.check(
        "upper glass top slides toward the front",
        upper_rest is not None and upper_open is not None and upper_open[0] < upper_rest[0] - 0.45,
        details=f"rest={upper_rest}, open={upper_open}",
    )

    flap_rest = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 1.2}):
        flap_open = ctx.part_world_aabb(flap)
    ctx.check(
        "lock flap swings outward from side wall",
        flap_rest is not None and flap_open is not None and flap_open[1][1] > flap_rest[1][1] + 0.025,
        details=f"rest={flap_rest}, open={flap_open}",
    )

    return ctx.report()


object_model = build_object_model()
