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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z0: float) -> cq.Workplane:
    """CadQuery annular cylinder with its bottom at z0."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _metal_jar_geometry() -> cq.Workplane:
    """Hollow stainless blender jar with a locking lower collar and rolled rim."""
    collar = _annular_cylinder(0.105, 0.055, 0.055, 0.0)
    wall = _annular_cylinder(0.095, 0.085, 0.365, 0.050)
    bottom = _annular_cylinder(0.086, 0.018, 0.020, 0.045)
    rim = _annular_cylinder(0.104, 0.084, 0.030, 0.405)
    return collar.union(wall).union(bottom).union(rim)


def _blade_geometry() -> cq.Workplane:
    """Connected four-blade cutter, hub, and drive shaft centered on local Z."""
    shaft = cq.Workplane("XY").circle(0.010).extrude(0.090).translate((0.0, 0.0, -0.060))
    bearing = cq.Workplane("XY").circle(0.028).extrude(0.008).translate((0.0, 0.0, -0.010))
    hub = cq.Workplane("XY").circle(0.026).extrude(0.018).translate((0.0, 0.0, -0.004))
    cutter = shaft.union(bearing).union(hub)
    for angle, z_offset in ((0.0, 0.014), (90.0, 0.018), (180.0, 0.014), (270.0, 0.018)):
        blade = (
            cq.Workplane("XY")
            .box(0.086, 0.018, 0.004)
            .translate((0.035, 0.0, z_offset))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        cutter = cutter.union(blade)
    return cutter


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_bar_blender")

    black = model.material("black_enamel", rgba=(0.015, 0.014, 0.013, 1.0))
    dark = model.material("dark_rubber", rgba=(0.02, 0.022, 0.024, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.72, 1.0))
    polished = model.material("polished_blade_steel", rgba=(0.86, 0.88, 0.88, 1.0))
    panel = model.material("black_control_panel", rgba=(0.005, 0.006, 0.007, 1.0))
    clear = model.material("clear_lid_cap", rgba=(0.80, 0.92, 1.0, 0.38))

    base = model.part("base")
    base.visual(Box((0.400, 0.320, 0.200)), origin=Origin(xyz=(0.0, 0.0, 0.100)), material=black, name="base_housing")
    base.visual(Box((0.340, 0.260, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.209)), material=stainless, name="top_deck")
    base.visual(Box((0.240, 0.010, 0.090)), origin=Origin(xyz=(0.0, -0.164, 0.095)), material=panel, name="front_panel")
    base.visual(Cylinder(radius=0.120, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.231)), material=stainless, name="socket_ring")
    base.visual(Cylinder(radius=0.083, length=0.028), origin=Origin(xyz=(0.0, 0.0, 0.232)), material=dark, name="drive_socket")
    for x in (-0.070, 0.070):
        base.visual(Box((0.045, 0.014, 0.004)), origin=Origin(xyz=(x, 0.085, 0.2435)), material=panel, name=f"bayonet_slot_{0 if x < 0 else 1}")

    speed_dial = model.part("speed_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.054,
            0.028,
            body_style="faceted",
            grip=KnobGrip(style="ribbed", count=18, depth=0.0012, width=0.0022),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        ),
        "speed_dial",
    )
    speed_dial.visual(
        dial_mesh,
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="dial_cap",
    )

    jar = model.part("jar")
    jar.visual(
        mesh_from_cadquery(_metal_jar_geometry(), "metal_jar", tolerance=0.0008, angular_tolerance=0.08),
        material=stainless,
        name="jar_shell",
    )
    for x in (-0.112, 0.112):
        jar.visual(Box((0.034, 0.026, 0.018)), origin=Origin(xyz=(x, 0.0, 0.024)), material=stainless, name=f"lock_lug_{0 if x < 0 else 1}")
    for x in (-0.065, 0.065):
        jar.visual(Box((0.018, 0.020, 0.030)), origin=Origin(xyz=(x, 0.104, 0.435)), material=dark, name=f"hinge_ear_{0 if x < 0 else 1}")
        jar.visual(Box((0.018, 0.024, 0.018)), origin=Origin(xyz=(x, 0.092, 0.423)), material=dark, name=f"hinge_root_{0 if x < 0 else 1}")
    jar.visual(Box((0.070, 0.008, 0.040)), origin=Origin(xyz=(0.0, 0.091, 0.340)), material=stainless, name="upper_handle_mount")
    jar.visual(Box((0.070, 0.008, 0.040)), origin=Origin(xyz=(0.0, 0.091, 0.150)), material=stainless, name="lower_handle_mount")

    handle = model.part("handle")
    handle.visual(Box((0.070, 0.018, 0.040)), origin=Origin(xyz=(0.0, 0.104, 0.340)), material=dark, name="upper_pad")
    handle.visual(Box((0.070, 0.018, 0.040)), origin=Origin(xyz=(0.0, 0.104, 0.150)), material=dark, name="lower_pad")
    handle.visual(Box((0.024, 0.080, 0.024)), origin=Origin(xyz=(0.0, 0.145, 0.340)), material=dark, name="upper_arm")
    handle.visual(Box((0.024, 0.080, 0.024)), origin=Origin(xyz=(0.0, 0.145, 0.150)), material=dark, name="lower_arm")
    handle.visual(Box((0.040, 0.032, 0.240)), origin=Origin(xyz=(0.0, 0.184, 0.245)), material=dark, name="grip")

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_blade_geometry(), "blade_cluster", tolerance=0.0005, angular_tolerance=0.08),
        material=polished,
        name="blade_cluster",
    )

    lid = model.part("lid")
    lid.visual(Cylinder(radius=0.103, length=0.026), origin=Origin(xyz=(0.0, -0.105, 0.013)), material=dark, name="lid_disk")
    lid.visual(Cylinder(radius=0.024, length=0.012), origin=Origin(xyz=(0.0, -0.105, 0.032)), material=clear, name="clear_cap")
    lid.visual(Box((0.070, 0.014, 0.006)), origin=Origin(xyz=(0.0, -0.008, 0.004)), material=dark, name="hinge_leaf")
    lid.visual(
        Cylinder(radius=0.007, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="hinge_barrel",
    )

    model.articulation(
        "base_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(0.0, -0.169, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "base_to_jar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.244)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=0.0, upper=0.42),
    )
    model.articulation("jar_to_handle", ArticulationType.FIXED, parent=jar, child=handle, origin=Origin())
    model.articulation(
        "jar_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=200.0),
    )
    model.articulation(
        "jar_to_lid",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, 0.105, 0.435)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    lid = object_model.get_part("lid")
    blade = object_model.get_part("blade")
    speed_dial = object_model.get_part("speed_dial")
    bayonet = object_model.get_articulation("base_to_jar")
    lid_hinge = object_model.get_articulation("jar_to_lid")
    blade_spin = object_model.get_articulation("jar_to_blade")

    ctx.check(
        "jar uses a limited bayonet twist",
        bayonet.articulation_type == ArticulationType.REVOLUTE
        and bayonet.motion_limits is not None
        and 0.30 <= bayonet.motion_limits.upper <= 0.55,
        details=f"type={bayonet.articulation_type}, limits={bayonet.motion_limits}",
    )
    ctx.check(
        "blade assembly spins continuously",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_spin.articulation_type}",
    )
    ctx.expect_contact(
        jar,
        base,
        elem_a="jar_shell",
        elem_b="socket_ring",
        contact_tol=0.0015,
        name="jar collar seats on base socket",
    )
    ctx.expect_contact(
        speed_dial,
        base,
        elem_a="dial_cap",
        elem_b="front_panel",
        contact_tol=0.0015,
        name="speed dial is mounted on front panel",
    )
    ctx.expect_gap(
        lid,
        jar,
        axis="z",
        positive_elem="lid_disk",
        negative_elem="jar_shell",
        max_gap=0.0015,
        max_penetration=0.0,
        name="closed lid rests on jar rim",
    )
    ctx.expect_within(
        blade,
        jar,
        axes="xy",
        inner_elem="blade_cluster",
        outer_elem="jar_shell",
        margin=0.0,
        name="blade stays inside jar diameter",
    )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.2}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge opens upward from rear edge",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.045,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
