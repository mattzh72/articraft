from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_panel(width: float, depth: float, height: float, radius: float):
    """Slightly radiused insulated refrigerator door slab."""
    return cq.Workplane("XY").box(width, depth, height).edges("|Z").fillet(radius)


def _barrel_sleeve(length: float, outer_radius: float, inner_radius: float):
    """Thin-walled hinge barrel around local Z with a clear bore for the pin."""
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="opposite_hinge_bottom_freezer_refrigerator")

    body_mat = model.material("warm_white_enamel", rgba=(0.90, 0.88, 0.82, 1.0))
    door_mat = model.material("gloss_white_door", rgba=(0.97, 0.96, 0.91, 1.0))
    trim_mat = model.material("satin_gray_trim", rgba=(0.48, 0.50, 0.52, 1.0))
    gasket_mat = model.material("black_magnetic_gasket", rgba=(0.015, 0.015, 0.018, 1.0))
    shadow_mat = model.material("dark_compartment_shadow", rgba=(0.04, 0.045, 0.05, 1.0))
    metal_mat = model.material("brushed_hinge_metal", rgba=(0.62, 0.63, 0.62, 1.0))
    handle_mat = model.material("brushed_handle", rgba=(0.78, 0.77, 0.72, 1.0))
    mark_mat = model.material("thermostat_mark_white", rgba=(1.0, 1.0, 0.94, 1.0))

    width = 0.82
    depth = 0.68
    height = 1.85
    wall = 0.035
    front_y = -depth / 2.0
    back_y = depth / 2.0
    hinge_y = front_y - 0.025

    cabinet = model.part("cabinet")
    # A connected insulated carcass: back, sides, bottom, top, and the horizontal
    # divider that separates the freezer from the fresh-food compartment.
    cabinet.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, back_y - wall / 2.0, height / 2.0)),
        material=body_mat,
        name="back_panel",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=body_mat,
        name="side_shell_0",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=body_mat,
        name="side_shell_1",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=body_mat,
        name="bottom_shell",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=body_mat,
        name="top_shell",
    )
    cabinet.visual(
        Box((width, depth, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        material=body_mat,
        name="compartment_divider",
    )
    cabinet.visual(
        Box((0.760, 0.020, 0.520)),
        origin=Origin(xyz=(0.0, front_y + 0.018, 0.395)),
        material=shadow_mat,
        name="freezer_cavity_shadow",
    )
    cabinet.visual(
        Box((0.760, 0.020, 0.920)),
        origin=Origin(xyz=(0.0, front_y + 0.018, 1.190)),
        material=shadow_mat,
        name="fresh_cavity_shadow",
    )
    cabinet.visual(
        Box((0.780, 0.045, 0.145)),
        origin=Origin(xyz=(0.0, front_y - 0.020, 1.735)),
        material=trim_mat,
        name="top_trim",
    )
    cabinet.visual(
        Box((0.760, 0.040, 0.080)),
        origin=Origin(xyz=(0.0, front_y - 0.018, 0.065)),
        material=trim_mat,
        name="toe_kick",
    )
    cabinet.visual(
        mesh_from_geometry(TorusGeometry(0.036, 0.003, radial_segments=36, tubular_segments=10), "thermostat_bezel"),
        origin=Origin(xyz=(0.0, front_y - 0.043, 1.735), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gasket_mat,
        name="thermostat_bezel",
    )

    sleeve_mesh_upper = mesh_from_geometry(_barrel_sleeve(0.175, 0.022, 0.010), "upper_hinge_barrel")
    sleeve_mesh_lower = mesh_from_geometry(_barrel_sleeve(0.120, 0.022, 0.010), "lower_hinge_barrel")
    upper_hinge_x = -width / 2.0 - 0.012
    lower_hinge_x = width / 2.0 + 0.012
    upper_hinge_z = 1.185
    lower_hinge_z = 0.390
    cabinet.visual(
        sleeve_mesh_upper,
        origin=Origin(xyz=(upper_hinge_x, hinge_y, upper_hinge_z - 0.365)),
        material=metal_mat,
        name="upper_barrel_bottom",
    )
    cabinet.visual(
        Box((0.032, 0.020, 0.175)),
        origin=Origin(xyz=(-0.392, front_y - 0.002, upper_hinge_z - 0.365)),
        material=metal_mat,
        name="upper_barrel_clip_bottom",
    )
    cabinet.visual(
        sleeve_mesh_upper,
        origin=Origin(xyz=(upper_hinge_x, hinge_y, upper_hinge_z)),
        material=metal_mat,
        name="upper_barrel_middle",
    )
    cabinet.visual(
        Box((0.032, 0.020, 0.175)),
        origin=Origin(xyz=(-0.392, front_y - 0.002, upper_hinge_z)),
        material=metal_mat,
        name="upper_barrel_clip_middle",
    )
    cabinet.visual(
        sleeve_mesh_upper,
        origin=Origin(xyz=(upper_hinge_x, hinge_y, upper_hinge_z + 0.365)),
        material=metal_mat,
        name="upper_barrel_top",
    )
    cabinet.visual(
        Box((0.032, 0.020, 0.175)),
        origin=Origin(xyz=(-0.392, front_y - 0.002, upper_hinge_z + 0.365)),
        material=metal_mat,
        name="upper_barrel_clip_top",
    )
    cabinet.visual(
        sleeve_mesh_lower,
        origin=Origin(xyz=(lower_hinge_x, hinge_y, lower_hinge_z - 0.220)),
        material=metal_mat,
        name="lower_barrel_bottom",
    )
    cabinet.visual(
        Box((0.032, 0.020, 0.120)),
        origin=Origin(xyz=(0.392, front_y - 0.002, lower_hinge_z - 0.220)),
        material=metal_mat,
        name="lower_barrel_clip_bottom",
    )
    cabinet.visual(
        sleeve_mesh_lower,
        origin=Origin(xyz=(lower_hinge_x, hinge_y, lower_hinge_z)),
        material=metal_mat,
        name="lower_barrel_middle",
    )
    cabinet.visual(
        Box((0.032, 0.020, 0.120)),
        origin=Origin(xyz=(0.392, front_y - 0.002, lower_hinge_z)),
        material=metal_mat,
        name="lower_barrel_clip_middle",
    )
    cabinet.visual(
        sleeve_mesh_lower,
        origin=Origin(xyz=(lower_hinge_x, hinge_y, lower_hinge_z + 0.220)),
        material=metal_mat,
        name="lower_barrel_top",
    )
    cabinet.visual(
        Box((0.032, 0.020, 0.120)),
        origin=Origin(xyz=(0.392, front_y - 0.002, lower_hinge_z + 0.220)),
        material=metal_mat,
        name="lower_barrel_clip_top",
    )

    upper_door = model.part("upper_door")
    upper_door.visual(
        mesh_from_cadquery(_rounded_panel(0.770, 0.065, 0.940, 0.018), "upper_door_panel"),
        origin=Origin(xyz=(0.410, -0.030, 0.0)),
        material=door_mat,
        name="door_panel",
    )
    upper_door.visual(
        Box((0.715, 0.008, 0.018)),
        origin=Origin(xyz=(0.410, -0.065, -0.470)),
        material=gasket_mat,
        name="lower_gasket",
    )
    upper_door.visual(
        Box((0.715, 0.008, 0.018)),
        origin=Origin(xyz=(0.410, -0.065, 0.470)),
        material=gasket_mat,
        name="upper_gasket",
    )
    upper_door.visual(
        Box((0.018, 0.008, 0.870)),
        origin=Origin(xyz=(0.780, -0.065, 0.0)),
        material=gasket_mat,
        name="free_edge_gasket",
    )
    upper_door.visual(
        Cylinder(radius=0.010, length=0.930),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal_mat,
        name="hinge_pin",
    )
    for idx, zc in enumerate((-0.190, 0.190)):
        upper_door.visual(
            Box((0.055, 0.028, 0.055)),
            origin=Origin(xyz=(0.026, -0.010, zc)),
            material=metal_mat,
            name=f"hinge_clip_{idx}",
        )
    upper_door.visual(
        Box((0.044, 0.035, 0.540)),
        origin=Origin(xyz=(0.705, -0.098, -0.030)),
        material=handle_mat,
        name="front_handle",
    )
    for idx, zc in enumerate((-0.235, 0.175)):
        upper_door.visual(
            Box((0.055, 0.040, 0.045)),
            origin=Origin(xyz=(0.705, -0.074, zc)),
            material=handle_mat,
            name=f"handle_post_{idx}",
        )

    lower_door = model.part("lower_door")
    lower_door.visual(
        mesh_from_cadquery(_rounded_panel(0.770, 0.065, 0.565, 0.018), "lower_door_panel"),
        origin=Origin(xyz=(-0.410, -0.030, 0.0)),
        material=door_mat,
        name="door_panel",
    )
    lower_door.visual(
        Box((0.715, 0.008, 0.016)),
        origin=Origin(xyz=(-0.410, -0.065, -0.282)),
        material=gasket_mat,
        name="lower_gasket",
    )
    lower_door.visual(
        Box((0.715, 0.008, 0.016)),
        origin=Origin(xyz=(-0.410, -0.065, 0.282)),
        material=gasket_mat,
        name="upper_gasket",
    )
    lower_door.visual(
        Box((0.018, 0.008, 0.500)),
        origin=Origin(xyz=(-0.780, -0.065, 0.0)),
        material=gasket_mat,
        name="free_edge_gasket",
    )
    lower_door.visual(
        Cylinder(radius=0.010, length=0.555),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal_mat,
        name="hinge_pin",
    )
    for idx, zc in enumerate((-0.110, 0.110)):
        lower_door.visual(
            Box((0.055, 0.028, 0.045)),
            origin=Origin(xyz=(-0.026, -0.010, zc)),
            material=metal_mat,
            name=f"hinge_clip_{idx}",
        )
    lower_door.visual(
        Box((0.044, 0.035, 0.320)),
        origin=Origin(xyz=(-0.705, -0.098, 0.000)),
        material=handle_mat,
        name="front_handle",
    )
    for idx, zc in enumerate((-0.115, 0.115)):
        lower_door.visual(
            Box((0.055, 0.040, 0.040)),
            origin=Origin(xyz=(-0.705, -0.074, zc)),
            material=handle_mat,
            name=f"handle_post_{idx}",
        )

    thermostat_knob = model.part("thermostat_knob")
    knob_geom = KnobGeometry(
        0.056,
        0.030,
        body_style="skirted",
        top_diameter=0.048,
        edge_radius=0.0015,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0010),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
    )
    thermostat_knob.visual(
        mesh_from_geometry(knob_geom, "thermostat_knob"),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="knob_cap",
    )
    thermostat_knob.visual(
        Box((0.006, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, -0.017, 0.010)),
        material=mark_mat,
        name="pointer_line",
    )

    model.articulation(
        "cabinet_to_upper_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=upper_door,
        origin=Origin(xyz=(upper_hinge_x, hinge_y, upper_hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.6, lower=0.0, upper=1.75),
    )
    model.articulation(
        "cabinet_to_lower_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lower_door,
        origin=Origin(xyz=(lower_hinge_x, hinge_y, lower_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "trim_to_thermostat",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=thermostat_knob,
        origin=Origin(xyz=(0.0, front_y - 0.0575, 1.735)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.0, lower=-2.35, upper=2.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    upper = object_model.get_part("upper_door")
    lower = object_model.get_part("lower_door")
    knob = object_model.get_part("thermostat_knob")
    upper_hinge = object_model.get_articulation("cabinet_to_upper_door")
    lower_hinge = object_model.get_articulation("cabinet_to_lower_door")
    thermostat = object_model.get_articulation("trim_to_thermostat")

    ctx.check(
        "opposite hinge axis signs",
        tuple(upper_hinge.axis) == (0.0, 0.0, -1.0) and tuple(lower_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"upper={upper_hinge.axis}, lower={lower_hinge.axis}",
    )
    ctx.check(
        "thermostat rotates normal to trim",
        tuple(thermostat.axis) == (0.0, -1.0, 0.0),
        details=f"axis={thermostat.axis}",
    )

    ctx.allow_overlap(
        cabinet,
        upper,
        elem_a="upper_barrel_bottom",
        elem_b="hinge_pin",
        reason="The upper fresh-food door hinge pin is intentionally captured inside the lower cabinet barrel.",
    )
    ctx.allow_overlap(
        cabinet,
        upper,
        elem_a="upper_barrel_middle",
        elem_b="hinge_pin",
        reason="The upper fresh-food door hinge pin is intentionally captured inside the center cabinet barrel.",
    )
    ctx.allow_overlap(
        cabinet,
        upper,
        elem_a="upper_barrel_top",
        elem_b="hinge_pin",
        reason="The upper fresh-food door hinge pin is intentionally captured inside the upper cabinet barrel.",
    )
    ctx.allow_overlap(
        cabinet,
        lower,
        elem_a="lower_barrel_bottom",
        elem_b="hinge_pin",
        reason="The lower freezer door hinge pin is intentionally captured inside the lower cabinet barrel.",
    )
    ctx.allow_overlap(
        cabinet,
        lower,
        elem_a="lower_barrel_middle",
        elem_b="hinge_pin",
        reason="The lower freezer door hinge pin is intentionally captured inside the center cabinet barrel.",
    )
    ctx.allow_overlap(
        cabinet,
        lower,
        elem_a="lower_barrel_top",
        elem_b="hinge_pin",
        reason="The lower freezer door hinge pin is intentionally captured inside the upper cabinet barrel.",
    )

    ctx.expect_within(
        upper,
        cabinet,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="upper_barrel_bottom",
        margin=0.002,
        name="upper pin is centered in lower barrel",
    )
    ctx.expect_overlap(
        upper,
        cabinet,
        axes="z",
        elem_a="hinge_pin",
        elem_b="upper_barrel_bottom",
        min_overlap=0.160,
        name="upper pin passes through lower barrel",
    )
    ctx.expect_within(
        upper,
        cabinet,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="upper_barrel_middle",
        margin=0.002,
        name="upper door pin is clipped inside barrel",
    )
    ctx.expect_overlap(
        upper,
        cabinet,
        axes="z",
        elem_a="hinge_pin",
        elem_b="upper_barrel_middle",
        min_overlap=0.160,
        name="upper door pin remains vertically captured",
    )
    ctx.expect_within(
        upper,
        cabinet,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="upper_barrel_top",
        margin=0.002,
        name="upper pin is centered in upper barrel",
    )
    ctx.expect_overlap(
        upper,
        cabinet,
        axes="z",
        elem_a="hinge_pin",
        elem_b="upper_barrel_top",
        min_overlap=0.160,
        name="upper pin passes through upper barrel",
    )
    ctx.expect_within(
        lower,
        cabinet,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="lower_barrel_bottom",
        margin=0.002,
        name="lower pin is centered in lower barrel",
    )
    ctx.expect_overlap(
        lower,
        cabinet,
        axes="z",
        elem_a="hinge_pin",
        elem_b="lower_barrel_bottom",
        min_overlap=0.105,
        name="lower pin passes through lower barrel",
    )
    ctx.expect_within(
        lower,
        cabinet,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="lower_barrel_middle",
        margin=0.002,
        name="lower door pin is clipped inside barrel",
    )
    ctx.expect_overlap(
        lower,
        cabinet,
        axes="z",
        elem_a="hinge_pin",
        elem_b="lower_barrel_middle",
        min_overlap=0.105,
        name="lower door pin remains vertically captured",
    )
    ctx.expect_within(
        lower,
        cabinet,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="lower_barrel_top",
        margin=0.002,
        name="lower pin is centered in upper barrel",
    )
    ctx.expect_overlap(
        lower,
        cabinet,
        axes="z",
        elem_a="hinge_pin",
        elem_b="lower_barrel_top",
        min_overlap=0.105,
        name="lower pin passes through upper barrel",
    )

    upper_closed = ctx.part_element_world_aabb(upper, elem="door_panel")
    lower_closed = ctx.part_element_world_aabb(lower, elem="door_panel")
    marker_closed = ctx.part_element_world_aabb(knob, elem="pointer_line")
    with ctx.pose({upper_hinge: 1.05, lower_hinge: 1.05, thermostat: 1.20}):
        ctx.expect_within(
            upper,
            cabinet,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="upper_barrel_middle",
            margin=0.002,
            name="upper pin stays captured while swinging",
        )
        ctx.expect_within(
            lower,
            cabinet,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="lower_barrel_middle",
            margin=0.002,
            name="lower pin stays captured while swinging",
        )
        upper_open = ctx.part_element_world_aabb(upper, elem="door_panel")
        lower_open = ctx.part_element_world_aabb(lower, elem="door_panel")
        marker_turned = ctx.part_element_world_aabb(knob, elem="pointer_line")

    def _min_y(aabb):
        return None if aabb is None else aabb[0][1]

    def _center_xz(aabb):
        if aabb is None:
            return None
        return ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][2] + aabb[1][2]) / 2.0)

    ctx.check(
        "upper fresh-food door swings forward",
        upper_closed is not None and upper_open is not None and _min_y(upper_open) < _min_y(upper_closed) - 0.12,
        details=f"closed={upper_closed}, open={upper_open}",
    )
    ctx.check(
        "lower freezer door swings forward",
        lower_closed is not None and lower_open is not None and _min_y(lower_open) < _min_y(lower_closed) - 0.12,
        details=f"closed={lower_closed}, open={lower_open}",
    )
    closed_marker = _center_xz(marker_closed)
    turned_marker = _center_xz(marker_turned)
    ctx.check(
        "thermostat pointer turns with knob",
        closed_marker is not None
        and turned_marker is not None
        and abs(turned_marker[0] - closed_marker[0]) > 0.006,
        details=f"closed={closed_marker}, turned={turned_marker}",
    )

    return ctx.report()


object_model = build_object_model()
