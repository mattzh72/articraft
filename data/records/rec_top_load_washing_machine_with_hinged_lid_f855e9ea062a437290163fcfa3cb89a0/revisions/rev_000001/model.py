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


def _top_deck_mesh(
    width: float,
    depth: float,
    thickness: float,
    center_y: float,
    center_z: float,
    opening_y: float,
    opening_radius: float,
    lock_y: float,
) -> object:
    """One continuous top deck with a real tub opening and lock socket cutout."""
    deck = cq.Workplane("XY").box(width, depth, thickness).translate((0.0, center_y, center_z))
    cutter_z = center_z - thickness * 1.5
    tub_cutter = (
        cq.Workplane("XY")
        .circle(opening_radius)
        .extrude(thickness * 3.0)
        .translate((0.0, opening_y, cutter_z))
    )
    lock_cutter = (
        cq.Workplane("XY")
        .rect(0.080, 0.055)
        .extrude(thickness * 3.0)
        .translate((0.0, lock_y, cutter_z))
    )
    return deck.cut(tub_cutter).cut(lock_cutter)


def _ring_mesh(outer_radius: float, inner_radius: float, height: float, center_y: float, center_z: float) -> object:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, center_y, center_z - height / 2.0))
    )


def _bezel_mesh(outer_radius: float, inner_radius: float, height: float, center_y: float, center_z: float, lock_y: float) -> object:
    ring = _ring_mesh(outer_radius, inner_radius, height, center_y, center_z)
    lock_notch = (
        cq.Workplane("XY")
        .rect(0.105, 0.090)
        .extrude(height * 3.0)
        .translate((0.0, lock_y, center_z - height * 1.5))
    )
    return ring.cut(lock_notch)


def _tub_mesh(outer_radius: float, inner_radius: float, depth: float) -> object:
    """Open cylindrical wash basket whose local origin is on the vertical spin axis at the rim."""
    wall = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(depth).translate((0.0, 0.0, -depth))
    bottom = cq.Workplane("XY").circle(outer_radius * 0.96).extrude(0.026).translate((0.0, 0.0, -depth))
    rim = (
        cq.Workplane("XY")
        .circle(outer_radius + 0.018)
        .circle(inner_radius - 0.010)
        .extrude(0.026)
        .translate((0.0, 0.0, -0.020))
    )
    return wall.union(bottom).union(rim)


def _agitator_mesh(radius: float, height: float) -> object:
    """Simple vaned central agitator fixed to the rotating basket."""
    post = cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, -height))
    vane_0 = cq.Workplane("XY").box(radius * 4.8, 0.018, height * 0.72).translate((0.0, 0.0, -height * 0.44))
    vane_1 = cq.Workplane("XY").box(0.018, radius * 4.8, height * 0.72).translate((0.0, 0.0, -height * 0.44))
    cap = cq.Workplane("XY").circle(radius * 1.35).extrude(0.030).translate((0.0, 0.0, -0.030))
    return post.union(vane_0).union(vane_1).union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_top_load_washer")

    porcelain = model.material("warm_white_enamel", rgba=(0.92, 0.91, 0.86, 1.0))
    shadow = model.material("dark_control_glass", rgba=(0.06, 0.07, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.012, 0.014, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.72, 1.0))
    plastic = model.material("soft_gray_plastic", rgba=(0.62, 0.64, 0.63, 1.0))
    blue = model.material("blue_button_plastic", rgba=(0.15, 0.28, 0.60, 1.0))
    white_mark = model.material("white_index_mark", rgba=(0.98, 0.98, 0.92, 1.0))

    width = 0.72
    depth = 0.74
    cabinet_h = 0.90
    top_t = 0.045
    top_z = cabinet_h + top_t / 2.0
    top_surface_z = cabinet_h + top_t
    tub_y = -0.045
    opening_r = 0.305
    lock_y = -0.335

    cabinet = model.part("cabinet")
    panel_t = 0.035

    # Boxy appliance cabinet: separate panels leave a real interior volume for the tub.
    cabinet.visual(Box((width, panel_t, 0.840)), origin=Origin(xyz=(0.0, -depth / 2.0 + panel_t / 2.0, 0.420)), material=porcelain, name="front_panel")
    cabinet.visual(Box((width, panel_t, cabinet_h)), origin=Origin(xyz=(0.0, depth / 2.0 - panel_t / 2.0, cabinet_h / 2.0)), material=porcelain, name="rear_panel")
    cabinet.visual(Box((panel_t, depth, cabinet_h)), origin=Origin(xyz=(-width / 2.0 + panel_t / 2.0, 0.0, cabinet_h / 2.0)), material=porcelain, name="side_panel_0")
    cabinet.visual(Box((panel_t, depth, cabinet_h)), origin=Origin(xyz=(width / 2.0 - panel_t / 2.0, 0.0, cabinet_h / 2.0)), material=porcelain, name="side_panel_1")
    cabinet.visual(Box((width, depth, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.0225)), material=rubber, name="black_plinth")
    cabinet.visual(
        mesh_from_cadquery(
            _top_deck_mesh(width, depth, top_t, 0.0, top_z, tub_y, opening_r, lock_y),
            "top_deck_with_wash_opening",
            tolerance=0.002,
        ),
        material=porcelain,
        name="top_deck",
    )
    cabinet.visual(
        mesh_from_cadquery(_bezel_mesh(0.322, opening_r, 0.018, tub_y, top_surface_z + 0.002, lock_y), "raised_tub_bezel"),
        material=porcelain,
        name="raised_tub_bezel",
    )
    cabinet.visual(Box((0.090, 0.012, 0.010)), origin=Origin(xyz=(0.0, lock_y - 0.034, top_surface_z + 0.005)), material=shadow, name="lock_socket_front")
    cabinet.visual(Box((0.090, 0.012, 0.010)), origin=Origin(xyz=(0.0, lock_y + 0.034, top_surface_z + 0.005)), material=shadow, name="lock_socket_rear")
    cabinet.visual(Box((0.012, 0.056, 0.010)), origin=Origin(xyz=(-0.046, lock_y, top_surface_z + 0.005)), material=shadow, name="lock_socket_side_0")
    cabinet.visual(Box((0.012, 0.056, 0.010)), origin=Origin(xyz=(0.046, lock_y, top_surface_z + 0.005)), material=shadow, name="lock_socket_side_1")

    # Rear console and dark control fascia.
    console_center_y = depth / 2.0 - 0.060
    console_front_y = console_center_y - 0.060
    cabinet.visual(Box((width, 0.120, 0.220)), origin=Origin(xyz=(0.0, console_center_y, top_surface_z + 0.110)), material=porcelain, name="rear_console")
    cabinet.visual(Box((0.640, 0.012, 0.150)), origin=Origin(xyz=(0.0, console_front_y - 0.006, top_surface_z + 0.112)), material=shadow, name="control_fascia")
    cabinet.visual(Box((width, 0.052, 0.040)), origin=Origin(xyz=(0.0, depth / 2.0 - 0.026, top_surface_z + 0.020)), material=porcelain, name="console_base_lip")

    # Fixed hinge supports on the cabinet; the moving lid barrel sits between them.
    hinge_y = 0.215
    hinge_z = top_surface_z + 0.030
    for x in (-0.245, 0.245):
        cabinet.visual(Box((0.085, 0.050, 0.050)), origin=Origin(xyz=(x, hinge_y + 0.034, hinge_z - 0.005)), material=porcelain, name=f"hinge_block_{0 if x < 0 else 1}")
        cabinet.visual(Cylinder(radius=0.018, length=0.085), origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)), material=steel, name=f"hinge_pin_socket_{0 if x < 0 else 1}")

    # Deep rotating wash tub, with open rim below the top deck and an attached agitator.
    tub = model.part("wash_tub")
    tub.visual(mesh_from_cadquery(_tub_mesh(0.258, 0.228, 0.520), "deep_open_wash_tub", tolerance=0.002), material=steel, name="tub_shell")
    tub.visual(mesh_from_cadquery(_agitator_mesh(0.043, 0.400), "center_agitator", tolerance=0.002), origin=Origin(xyz=(0.0, 0.0, -0.120)), material=plastic, name="agitator")
    tub.visual(Cylinder(radius=0.035, length=0.360), origin=Origin(xyz=(0.0, 0.0, -0.700)), material=steel, name="drive_spindle")
    model.articulation(
        "cabinet_to_wash_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, tub_y, top_surface_z - 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=10.0),
    )

    # Large top lid hinged along the rear edge. Its local -Y direction is forward.
    lid = model.part("top_lid")
    lid.visual(Box((0.610, 0.575, 0.045)), origin=Origin(xyz=(0.0, -0.3125, 0.000)), material=porcelain, name="lid_panel")
    lid.visual(Box((0.500, 0.405, 0.010)), origin=Origin(xyz=(0.0, -0.330, 0.026)), material=plastic, name="recessed_lid_inset")
    lid.visual(Box((0.250, 0.050, 0.025)), origin=Origin(xyz=(0.0, -0.025, -0.010)), material=porcelain, name="hinge_flange")
    lid.visual(Cylinder(radius=0.019, length=0.265), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=steel, name="hinge_barrel")
    model.articulation(
        "cabinet_to_top_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.75),
    )

    # Lid-lock plunger rides on the lid and moves vertically into the deck socket.
    plunger = model.part("lock_plunger")
    plunger.visual(Cylinder(radius=0.013, length=0.070), origin=Origin(xyz=(0.0, 0.0, -0.035)), material=steel, name="plunger_pin")
    plunger.visual(Box((0.038, 0.030, 0.014)), origin=Origin(xyz=(0.0, 0.0, -0.002)), material=plastic, name="plunger_head")
    model.articulation(
        "top_lid_to_lock_plunger",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=plunger,
        origin=Origin(xyz=(0.0, lock_y - hinge_y, -0.027)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.035),
    )

    # Cycle dial rotates continuously on the console face.
    dial = model.part("cycle_dial")
    dial.visual(Cylinder(radius=0.058, length=0.036), origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=plastic, name="dial_cap")
    dial.visual(Cylinder(radius=0.036, length=0.010), origin=Origin(xyz=(0.0, -0.041, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=white_mark, name="dial_center")
    dial.visual(Box((0.010, 0.008, 0.052)), origin=Origin(xyz=(0.0, -0.047, 0.030)), material=shadow, name="dial_index")
    model.articulation(
        "cabinet_to_cycle_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(-0.235, console_front_y - 0.011, top_surface_z + 0.112)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    # Four independent prismatic push buttons.
    button_xs = (-0.065, 0.030, 0.125, 0.220)
    for i, x in enumerate(button_xs):
        button = model.part(f"button_{i}")
        button.visual(Box((0.058, 0.026, 0.036)), origin=Origin(xyz=(0.0, -0.014, 0.0)), material=blue, name="button_cap")
        model.articulation(
            f"cabinet_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, console_front_y - 0.011, top_surface_z + 0.108)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.06, lower=0.0, upper=0.012),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("top_lid")
    tub = object_model.get_part("wash_tub")
    plunger = object_model.get_part("lock_plunger")
    lid_hinge = object_model.get_articulation("cabinet_to_top_lid")
    plunger_slide = object_model.get_articulation("top_lid_to_lock_plunger")

    ctx.expect_overlap(lid, cabinet, axes="xy", min_overlap=0.45, elem_a="lid_panel", elem_b="top_deck", name="closed lid covers the top deck opening")
    ctx.expect_gap(lid, cabinet, axis="z", max_gap=0.020, max_penetration=0.0, positive_elem="lid_panel", negative_elem="top_deck", name="closed lid sits just above the deck")
    ctx.expect_within(tub, cabinet, axes="xy", margin=0.0, inner_elem="tub_shell", outer_elem="raised_tub_bezel", name="wash tub is centered inside the circular opening")

    tub_aabb = ctx.part_element_world_aabb(tub, elem="tub_shell")
    ctx.check(
        "tub has deep hollow laundry cavity",
        tub_aabb is not None and (tub_aabb[1][2] - tub_aabb[0][2]) > 0.48 and tub_aabb[0][2] < 0.42,
        details=f"tub_shell_aabb={tub_aabb}",
    )
    deck_aabb = ctx.part_element_world_aabb(cabinet, elem="top_deck")
    ctx.check(
        "tub rim sits below the top deck surface",
        tub_aabb is not None and deck_aabb is not None and tub_aabb[1][2] < deck_aabb[1][2] - 0.005,
        details=f"tub_shell_aabb={tub_aabb}, deck_aabb={deck_aabb}",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.35}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid hinge opens upward",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.30,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_plunger = ctx.part_world_position(plunger)
    with ctx.pose({plunger_slide: 0.030}):
        pushed_plunger = ctx.part_world_position(plunger)
    ctx.check(
        "front lock plunger drives downward into the deck socket",
        rest_plunger is not None and pushed_plunger is not None and pushed_plunger[2] < rest_plunger[2] - 0.025,
        details=f"rest={rest_plunger}, pushed={pushed_plunger}",
    )

    dial_joint = object_model.get_articulation("cabinet_to_cycle_dial")
    ctx.check("cycle dial is continuous", dial_joint.articulation_type == ArticulationType.CONTINUOUS, details=str(dial_joint.articulation_type))

    for i in range(4):
        button = object_model.get_part(f"button_{i}")
        button_joint = object_model.get_articulation(f"cabinet_to_button_{i}")
        rest = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.010}):
            pushed = ctx.part_world_position(button)
        ctx.check(
            f"button_{i} pushes into the console",
            button_joint.articulation_type == ArticulationType.PRISMATIC
            and rest is not None
            and pushed is not None
            and pushed[1] > rest[1] + 0.008,
            details=f"rest={rest}, pushed={pushed}, type={button_joint.articulation_type}",
        )

    return ctx.report()


object_model = build_object_model()
