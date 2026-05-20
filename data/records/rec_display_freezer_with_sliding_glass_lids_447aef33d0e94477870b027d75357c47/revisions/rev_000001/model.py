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


LENGTH = 2.70
WIDTH = 1.15
HEIGHT = 0.82
OPENING_LENGTH = 2.25
OPENING_WIDTH = 0.73


def _hollow_freezer_tub() -> cq.Workplane:
    """A real hollow insulated chest/tub rather than a solid block."""
    outer = cq.Workplane("XY").box(LENGTH, WIDTH, HEIGHT, centered=(True, True, False))
    cavity = (
        cq.Workplane("XY")
        .box(OPENING_LENGTH, OPENING_WIDTH, HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, 0.16))
    )
    return outer.cut(cavity)


def _add_lid_visuals(part, *, panel_length: float, panel_width: float, material_frame: str, material_glass: str, handle_x: float) -> None:
    bar = 0.050
    frame_h = 0.035
    glass_t = 0.008

    part.visual(
        Box((panel_length, bar, frame_h)),
        origin=Origin(xyz=(0.0, panel_width / 2.0 - bar / 2.0, 0.0)),
        material=material_frame,
        name="rear_frame",
    )
    part.visual(
        Box((panel_length, bar, frame_h)),
        origin=Origin(xyz=(0.0, -panel_width / 2.0 + bar / 2.0, 0.0)),
        material=material_frame,
        name="front_frame",
    )
    part.visual(
        Box((bar, panel_width, frame_h)),
        origin=Origin(xyz=(-panel_length / 2.0 + bar / 2.0, 0.0, 0.0)),
        material=material_frame,
        name="end_frame_0",
    )
    part.visual(
        Box((bar, panel_width, frame_h)),
        origin=Origin(xyz=(panel_length / 2.0 - bar / 2.0, 0.0, 0.0)),
        material=material_frame,
        name="end_frame_1",
    )
    # The glass slightly tucks under the metal lip so the visual assembly reads
    # as a captured framed pane rather than four loose bars.
    part.visual(
        Box((panel_length - 0.075, panel_width - 0.075, glass_t)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=material_glass,
        name="glass_pane",
    )
    part.visual(
        Box((0.20, 0.045, 0.030)),
        origin=Origin(xyz=(handle_x, -panel_width / 2.0 + 0.13, 0.020)),
        material="dark_gasket",
        name="finger_pull",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="supermarket_island_display_freezer")
    model.material("insulated_white", rgba=(0.92, 0.95, 0.96, 1.0))
    model.material("dark_gasket", rgba=(0.03, 0.035, 0.04, 1.0))
    model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("glass_blue", rgba=(0.58, 0.82, 0.95, 0.36))
    model.material("key_metal", rgba=(0.82, 0.78, 0.62, 1.0))
    model.material("flap_steel", rgba=(0.62, 0.66, 0.68, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_hollow_freezer_tub(), "hollow_insulated_tub", tolerance=0.003),
        material="insulated_white",
        name="hollow_tub",
    )

    # Dark bumper/gasket ring surrounding the open cold well.
    rim_z = HEIGHT + 0.0175
    cabinet.visual(
        Box((LENGTH + 0.04, 0.065, 0.035)),
        origin=Origin(xyz=(0.0, WIDTH / 2.0 - 0.0325, rim_z)),
        material="dark_gasket",
        name="side_rim_0",
    )
    cabinet.visual(
        Box((LENGTH + 0.04, 0.065, 0.035)),
        origin=Origin(xyz=(0.0, -WIDTH / 2.0 + 0.0325, rim_z)),
        material="dark_gasket",
        name="side_rim_1",
    )
    cabinet.visual(
        Box((0.080, WIDTH, 0.035)),
        origin=Origin(xyz=(LENGTH / 2.0 - 0.040, 0.0, rim_z)),
        material="dark_gasket",
        name="end_rim_0",
    )
    cabinet.visual(
        Box((0.080, WIDTH, 0.035)),
        origin=Origin(xyz=(-LENGTH / 2.0 + 0.040, 0.0, rim_z)),
        material="dark_gasket",
        name="end_rim_1",
    )

    # Two visible stepped aluminum track pairs carry the lower and upper sliding
    # glass lids, like a real supermarket island freezer.
    lower_track_top = HEIGHT + 0.035
    upper_track_top = 0.980
    cabinet.visual(
        Box((2.42, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, 0.395, lower_track_top - 0.0175)),
        material="brushed_aluminum",
        name="lower_track_rear",
    )
    cabinet.visual(
        Box((2.42, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, -0.395, lower_track_top - 0.0175)),
        material="brushed_aluminum",
        name="lower_track_front",
    )
    cabinet.visual(
        Box((2.42, 0.050, 0.085)),
        origin=Origin(xyz=(0.0, 0.335, upper_track_top - 0.0425)),
        material="brushed_aluminum",
        name="upper_track_rear",
    )
    cabinet.visual(
        Box((2.42, 0.050, 0.085)),
        origin=Origin(xyz=(0.0, -0.335, upper_track_top - 0.0425)),
        material="brushed_aluminum",
        name="upper_track_front",
    )
    for x in (-1.260, 1.260):
        for y in (-0.335, 0.335):
            cabinet.visual(
                Box((0.100, 0.050, upper_track_top - 0.085 - HEIGHT)),
                origin=Origin(xyz=(x, y, HEIGHT + (upper_track_top - 0.085 - HEIGHT) / 2.0)),
                material="brushed_aluminum",
                name=f"upper_track_support_{x:+.0f}_{y:+.0f}",
            )

    # Side lock hardware: a keyed cylinder on the +Y side wall and a hinge mount
    # for the protective cover.
    lock_x = 0.98
    lock_z = 0.425
    cabinet.visual(
        Cylinder(radius=0.038, length=0.034),
        origin=Origin(xyz=(lock_x, WIDTH / 2.0 + 0.017, lock_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="key_metal",
        name="key_cylinder",
    )
    cabinet.visual(
        Box((0.018, 0.006, 0.048)),
        origin=Origin(xyz=(lock_x, WIDTH / 2.0 + 0.036, lock_z)),
        material="dark_gasket",
        name="key_slot",
    )
    cabinet.visual(
        Box((0.260, 0.060, 0.055)),
        origin=Origin(xyz=(lock_x, WIDTH / 2.0 + 0.011, 0.515)),
        material="brushed_aluminum",
        name="flap_hinge_mount",
    )

    lid_0 = model.part("lid_0")
    _add_lid_visuals(
        lid_0,
        panel_length=1.34,
        panel_width=0.84,
        material_frame="brushed_aluminum",
        material_glass="glass_blue",
        handle_x=0.40,
    )
    lid_1 = model.part("lid_1")
    _add_lid_visuals(
        lid_1,
        panel_length=1.34,
        panel_width=0.72,
        material_frame="brushed_aluminum",
        material_glass="glass_blue",
        handle_x=-0.40,
    )

    model.articulation(
        "cabinet_to_lid_0",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_0,
        origin=Origin(xyz=(-0.455, 0.0, lower_track_top + 0.0175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.55),
    )
    model.articulation(
        "cabinet_to_lid_1",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_1,
        origin=Origin(xyz=(0.455, 0.0, upper_track_top + 0.0175)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.55),
    )

    lock_flap = model.part("lock_flap")
    lock_flap.visual(
        Box((0.210, 0.014, 0.155)),
        origin=Origin(xyz=(0.0, 0.0, -0.0775)),
        material="flap_steel",
        name="cover_panel",
    )
    lock_flap.visual(
        Cylinder(radius=0.014, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_aluminum",
        name="hinge_barrel",
    )
    model.articulation(
        "cabinet_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lock_flap,
        origin=Origin(xyz=(lock_x, WIDTH / 2.0 + 0.055, 0.505)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    flap = object_model.get_part("lock_flap")
    slide_0 = object_model.get_articulation("cabinet_to_lid_0")
    slide_1 = object_model.get_articulation("cabinet_to_lid_1")
    flap_hinge = object_model.get_articulation("cabinet_to_lock_flap")

    ctx.expect_gap(
        lid_0,
        cabinet,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="front_frame",
        negative_elem="lower_track_front",
        name="lower lid rides on lower front rail",
    )
    ctx.expect_gap(
        lid_1,
        cabinet,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="front_frame",
        negative_elem="upper_track_front",
        name="upper lid rides on upper front rail",
    )
    ctx.expect_overlap(
        lid_0,
        cabinet,
        axes="x",
        min_overlap=1.0,
        elem_a="front_frame",
        elem_b="lower_track_front",
        name="lower lid remains on long rail at rest",
    )
    ctx.expect_overlap(
        lid_1,
        cabinet,
        axes="x",
        min_overlap=1.0,
        elem_a="front_frame",
        elem_b="upper_track_front",
        name="upper lid remains on long rail at rest",
    )

    lid_0_rest = ctx.part_world_position(lid_0)
    with ctx.pose({slide_0: 0.55}):
        lid_0_open = ctx.part_world_position(lid_0)
        ctx.expect_overlap(
            lid_0,
            cabinet,
            axes="x",
            min_overlap=0.70,
            elem_a="front_frame",
            elem_b="lower_track_front",
            name="lower lid stays captured when slid",
        )
    ctx.check(
        "lid_0 slides along positive rail",
        lid_0_rest is not None and lid_0_open is not None and lid_0_open[0] > lid_0_rest[0] + 0.50,
        details=f"rest={lid_0_rest}, open={lid_0_open}",
    )

    lid_1_rest = ctx.part_world_position(lid_1)
    with ctx.pose({slide_1: 0.55}):
        lid_1_open = ctx.part_world_position(lid_1)
        ctx.expect_overlap(
            lid_1,
            cabinet,
            axes="x",
            min_overlap=0.70,
            elem_a="front_frame",
            elem_b="upper_track_front",
            name="upper lid stays captured when slid",
        )
    ctx.check(
        "lid_1 slides along opposite rail",
        lid_1_rest is not None and lid_1_open is not None and lid_1_open[0] < lid_1_rest[0] - 0.50,
        details=f"rest={lid_1_rest}, open={lid_1_open}",
    )

    ctx.expect_overlap(
        flap,
        cabinet,
        axes=("x", "z"),
        min_overlap=0.030,
        elem_a="cover_panel",
        elem_b="key_cylinder",
        name="lock flap covers key cylinder",
    )
    ctx.expect_gap(
        flap,
        cabinet,
        axis="y",
        min_gap=0.002,
        max_gap=0.020,
        positive_elem="cover_panel",
        negative_elem="key_cylinder",
        name="lock flap stands just proud of key cylinder",
    )

    closed_panel = ctx.part_element_world_aabb(flap, elem="cover_panel")
    with ctx.pose({flap_hinge: 1.10}):
        open_panel = ctx.part_element_world_aabb(flap, elem="cover_panel")
    if closed_panel is not None and open_panel is not None:
        closed_y_max = closed_panel[1][1]
        open_y_max = open_panel[1][1]
        closed_z_center = (closed_panel[0][2] + closed_panel[1][2]) / 2.0
        open_z_center = (open_panel[0][2] + open_panel[1][2]) / 2.0
        flap_opens = open_y_max > closed_y_max + 0.08 and open_z_center > closed_z_center + 0.035
    else:
        flap_opens = False
    ctx.check("lock flap hinges outward and upward", flap_opens, details=f"closed={closed_panel}, open={open_panel}")

    return ctx.report()


object_model = build_object_model()
