from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cylinder_between(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[Origin, float]:
    """Origin/length for a URDF cylinder whose local +Z runs from start to end."""

    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("segment length must be positive")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _tube_shell(
    outer_radius: float, inner_radius: float, length: float, *, segments: int = 64
):
    half = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="newtonian_reflector_equatorial_mount")

    brushed = model.material("brushed_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.04, 0.045, 0.05, 1.0))
    black = model.material("matte_black", rgba=(0.002, 0.002, 0.003, 1.0))
    white = model.material("gloss_white_tube", rgba=(0.88, 0.91, 0.94, 1.0))
    glass = model.material("blue_glass", rgba=(0.25, 0.55, 0.85, 0.55))
    mirror = model.material("silvered_mirror", rgba=(0.82, 0.90, 0.98, 1.0))

    tube_mesh = mesh_from_geometry(_tube_shell(0.160, 0.145, 1.10), "open_optical_tube")
    ring_mesh = mesh_from_geometry(_tube_shell(0.178, 0.161, 0.050), "tube_ring")
    cell_ring_mesh = mesh_from_geometry(_tube_shell(0.172, 0.137, 0.035), "mirror_cell_ring")
    focuser_sleeve_mesh = mesh_from_geometry(
        _tube_shell(0.043, 0.033, 0.100, segments=48), "focuser_sleeve"
    )
    focus_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.055,
            0.026,
            body_style="lobed",
            grip=KnobGrip(style="ribbed", count=10, depth=0.0015),
            edge_radius=0.001,
        ),
        "focus_knob",
    )

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.060, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        material=brushed,
        name="central_pier",
    )
    tripod.visual(
        Cylinder(radius=0.135, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        material=dark_metal,
        name="top_hub",
    )

    polar_elevation = math.radians(40.0)
    polar_rpy = (polar_elevation - math.pi / 2.0, 0.0, 0.0)
    tripod.visual(
        Cylinder(radius=0.090, length=0.120),
        origin=Origin(xyz=(0.0, 0.046, 0.959), rpy=polar_rpy),
        material=dark_metal,
        name="polar_socket",
    )

    for index, angle in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        top = (0.095 * math.cos(angle), 0.095 * math.sin(angle), 0.805)
        foot = (0.64 * math.cos(angle), 0.64 * math.sin(angle), 0.035)
        leg_origin, leg_len = _cylinder_between(top, foot)
        tripod.visual(
            Cylinder(radius=0.024, length=leg_len),
            origin=leg_origin,
            material=brushed,
            name=f"leg_{index}",
        )
        tripod.visual(
            Sphere(radius=0.045),
            origin=Origin(xyz=foot),
            material=dark_metal,
            name=f"rubber_foot_{index}",
        )
        tray_outer = (0.34 * math.cos(angle), 0.34 * math.sin(angle), 0.365)
        spreader_origin, spreader_len = _cylinder_between((0.0, 0.0, 0.365), tray_outer)
        tripod.visual(
            Cylinder(radius=0.012, length=spreader_len),
            origin=spreader_origin,
            material=dark_metal,
            name=f"spreader_{index}",
        )
    tripod.visual(
        Cylinder(radius=0.120, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=dark_metal,
        name="accessory_tray",
    )

    polar_head = model.part("polar_head")
    polar_head.visual(
        Cylinder(radius=0.070, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=dark_metal,
        name="ra_bearing",
    )
    polar_head.visual(
        Cylinder(radius=0.043, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=brushed,
        name="polar_shaft",
    )
    polar_head.visual(
        Cylinder(radius=0.113, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=brushed,
        name="ra_setting_circle",
    )
    polar_head.visual(
        Cylinder(radius=0.090, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=dark_metal,
        name="ra_clutch_ring",
    )
    polar_head.visual(
        Box((0.130, 0.085, 0.105)),
        origin=Origin(xyz=(0.118, 0.0, 0.270)),
        material=dark_metal,
        name="drive_motor_box",
    )
    polar_head.visual(
        Sphere(radius=0.025),
        origin=Origin(xyz=(0.195, 0.0, 0.305)),
        material=black,
        name="ra_lock_knob",
    )
    for side in (-1.0, 1.0):
        polar_head.visual(
            Box((0.170, 0.025, 0.135)),
            origin=Origin(xyz=(0.0, side * 0.085, 0.420)),
            material=dark_metal,
            name=f"declination_fork_{0 if side < 0 else 1}",
        )

    telescope = model.part("telescope")
    telescope.visual(
        Cylinder(radius=0.055, length=0.260),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="declination_hub",
    )
    telescope.visual(
        Cylinder(radius=0.018, length=0.760),
        origin=Origin(xyz=(-0.430, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="counterweight_shaft",
    )
    for index, x in enumerate((-0.470, -0.630)):
        telescope.visual(
            Cylinder(radius=0.085, length=0.090),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed,
            name=f"counterweight_{index}",
        )
    telescope.visual(
        Box((0.135, 0.115, 0.120)),
        origin=Origin(xyz=(0.118, 0.0, 0.045)),
        material=dark_metal,
        name="saddle_block",
    )
    telescope.visual(
        Box((0.085, 0.730, 0.035)),
        origin=Origin(xyz=(0.210, 0.0, 0.090)),
        material=dark_metal,
        name="dovetail_bar",
    )

    tube_center = (0.230, 0.0, 0.250)
    telescope.visual(
        tube_mesh,
        origin=Origin(xyz=tube_center, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="tube_shell",
    )
    for index, y in enumerate((-0.285, 0.285)):
        telescope.visual(
            ring_mesh,
            origin=Origin(xyz=(tube_center[0], y, tube_center[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"tube_ring_{index}",
        )
        telescope.visual(
            Box((0.120, 0.075, 0.080)),
            origin=Origin(xyz=(tube_center[0], y, 0.125)),
            material=dark_metal,
            name=f"ring_foot_{index}",
        )
    for name, y in (("front_cell", 0.550), ("rear_cell", -0.550)):
        telescope.visual(
            cell_ring_mesh,
            origin=Origin(xyz=(tube_center[0], y, tube_center[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=name,
        )
    telescope.visual(
        Cylinder(radius=0.130, length=0.012),
        origin=Origin(xyz=(tube_center[0], -0.533, tube_center[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mirror,
        name="primary_mirror",
    )
    for index, (clip_x, clip_z, clip_rpy) in enumerate(
        (
            (tube_center[0], tube_center[2] + 0.139, (0.0, 0.0, 0.0)),
            (tube_center[0] - 0.120, tube_center[2] - 0.070, (0.0, 0.0, math.radians(60.0))),
            (tube_center[0] + 0.120, tube_center[2] - 0.070, (0.0, 0.0, math.radians(-60.0))),
        )
    ):
        telescope.visual(
            Box((0.046, 0.020, 0.016)),
            origin=Origin(xyz=(clip_x, -0.536, clip_z), rpy=clip_rpy),
            material=black,
            name=f"primary_clip_{index}",
        )
    telescope.visual(
        Box((0.288, 0.008, 0.006)),
        origin=Origin(xyz=(tube_center[0], 0.535, tube_center[2])),
        material=black,
        name="spider_vane_x",
    )
    telescope.visual(
        Box((0.006, 0.008, 0.288)),
        origin=Origin(xyz=(tube_center[0], 0.535, tube_center[2])),
        material=black,
        name="spider_vane_z",
    )
    telescope.visual(
        Cylinder(radius=0.035, length=0.010),
        origin=Origin(xyz=(tube_center[0], 0.527, tube_center[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mirror,
        name="secondary_mirror",
    )

    telescope.visual(
        Cylinder(radius=0.027, length=0.480),
        origin=Origin(xyz=(0.085, 0.055, 0.480), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="finder_tube",
    )
    for index, y in enumerate((-0.145, 0.255)):
        telescope.visual(
            Cylinder(radius=0.031, length=0.030),
            origin=Origin(xyz=(0.085, y, 0.480), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=white,
            name=f"finder_ring_{index}",
        )
        telescope.visual(
            Box((0.045, 0.030, 0.090)),
            origin=Origin(xyz=(0.130, y, 0.430)),
            material=dark_metal,
            name=f"finder_bracket_{index}",
        )
    telescope.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(0.085, 0.298, 0.480), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="finder_front_lens",
    )

    telescope.visual(
        Box((0.135, 0.115, 0.040)),
        origin=Origin(xyz=(tube_center[0], 0.340, 0.420)),
        material=dark_metal,
        name="focuser_base",
    )
    telescope.visual(
        focuser_sleeve_mesh,
        origin=Origin(xyz=(tube_center[0], 0.340, 0.490)),
        material=black,
        name="focuser_sleeve",
    )
    telescope.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=Origin(xyz=(0.160, 0.340, 0.490), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="focus_knob_boss",
    )

    drawtube = model.part("focus_drawtube")
    drawtube.visual(
        Cylinder(radius=0.027, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=brushed,
        name="drawtube_barrel",
    )
    drawtube.visual(
        Cylinder(radius=0.034, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=black,
        name="eyepiece",
    )
    drawtube.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.181)),
        material=glass,
        name="eyepiece_glass",
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        focus_knob_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lobed_knob",
    )

    model.articulation(
        "polar_axis",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=polar_head,
        origin=Origin(xyz=(0.0, 0.0, 0.920), rpy=polar_rpy),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.45, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "declination_axis",
        ArticulationType.REVOLUTE,
        parent=polar_head,
        child=telescope,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.45, lower=-1.4, upper=1.4),
    )
    model.articulation(
        "focus_slide",
        ArticulationType.PRISMATIC,
        parent=telescope,
        child=drawtube,
        origin=Origin(xyz=(tube_center[0], 0.340, 0.440)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.040),
    )
    model.articulation(
        "focus_knob_axis",
        ArticulationType.CONTINUOUS,
        parent=telescope,
        child=focus_knob,
        origin=Origin(xyz=(0.117, 0.340, 0.490)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    polar_head = object_model.get_part("polar_head")
    telescope = object_model.get_part("telescope")
    drawtube = object_model.get_part("focus_drawtube")

    polar_axis = object_model.get_articulation("polar_axis")
    declination_axis = object_model.get_articulation("declination_axis")
    focus_slide = object_model.get_articulation("focus_slide")

    ctx.allow_overlap(
        tripod,
        polar_head,
        elem_a="polar_socket",
        elem_b="ra_bearing",
        reason="The polar bearing is intentionally seated inside the tripod's captured socket.",
    )
    ctx.allow_overlap(
        tripod,
        polar_head,
        elem_a="polar_socket",
        elem_b="ra_setting_circle",
        reason="The setting circle is nested against the polar socket as a captured collar at the base of the RA axis.",
    )
    ctx.allow_overlap(
        tripod,
        polar_head,
        elem_a="polar_socket",
        elem_b="polar_shaft",
        reason="The polar shaft is intentionally sleeved through the tripod polar socket.",
    )
    ctx.allow_overlap(
        polar_head,
        telescope,
        elem_a="polar_shaft",
        elem_b="declination_hub",
        reason="The declination housing is intentionally captured around the end of the polar shaft.",
    )
    ctx.expect_overlap(
        tripod,
        polar_head,
        axes="xyz",
        elem_a="polar_socket",
        elem_b="ra_bearing",
        min_overlap=0.020,
        name="polar bearing is retained in the socket",
    )
    ctx.expect_overlap(
        tripod,
        polar_head,
        axes="z",
        elem_a="polar_socket",
        elem_b="ra_setting_circle",
        min_overlap=0.015,
        name="setting circle is seated at polar socket",
    )
    ctx.expect_overlap(
        tripod,
        polar_head,
        axes="xyz",
        elem_a="polar_socket",
        elem_b="polar_shaft",
        min_overlap=0.030,
        name="polar shaft passes through socket sleeve",
    )
    ctx.expect_overlap(
        polar_head,
        telescope,
        axes="xyz",
        elem_a="polar_shaft",
        elem_b="declination_hub",
        min_overlap=0.020,
        name="declination hub captures polar shaft end",
    )

    ctx.expect_overlap(
        telescope,
        telescope,
        axes="y",
        elem_a="tube_ring_0",
        elem_b="tube_shell",
        min_overlap=0.035,
        name="rear ring wraps the optical tube",
    )
    ctx.expect_overlap(
        telescope,
        telescope,
        axes="y",
        elem_a="tube_ring_1",
        elem_b="tube_shell",
        min_overlap=0.035,
        name="front ring wraps the optical tube",
    )
    ctx.expect_within(
        drawtube,
        telescope,
        axes="x",
        inner_elem="drawtube_barrel",
        outer_elem="focuser_sleeve",
        margin=0.002,
        name="drawtube stays laterally centered in focuser sleeve",
    )
    ctx.expect_overlap(
        drawtube,
        telescope,
        axes="z",
        elem_a="drawtube_barrel",
        elem_b="focuser_sleeve",
        min_overlap=0.055,
        name="drawtube remains inserted at rest",
    )

    rest_tube = ctx.part_element_world_aabb(telescope, elem="tube_shell")
    with ctx.pose({declination_axis: 0.65}):
        dec_tube = ctx.part_element_world_aabb(telescope, elem="tube_shell")
    ctx.check(
        "declination axis rotates the optical tube",
        rest_tube is not None
        and dec_tube is not None
        and abs(((rest_tube[0][2] + rest_tube[1][2]) - (dec_tube[0][2] + dec_tube[1][2])) * 0.5)
        > 0.030,
        details=f"rest={rest_tube}, dec_pose={dec_tube}",
    )

    rest_head = ctx.part_element_world_aabb(telescope, elem="tube_shell")
    with ctx.pose({polar_axis: 0.55}):
        polar_head_posed = ctx.part_element_world_aabb(telescope, elem="tube_shell")
    ctx.check(
        "polar axis carries the declination assembly",
        rest_head is not None
        and polar_head_posed is not None
        and math.dist(
            (
                (rest_head[0][0] + rest_head[1][0]) * 0.5,
                (rest_head[0][1] + rest_head[1][1]) * 0.5,
                (rest_head[0][2] + rest_head[1][2]) * 0.5,
            ),
            (
                (polar_head_posed[0][0] + polar_head_posed[1][0]) * 0.5,
                (polar_head_posed[0][1] + polar_head_posed[1][1]) * 0.5,
                (polar_head_posed[0][2] + polar_head_posed[1][2]) * 0.5,
            ),
        )
        > 0.020,
        details=f"rest={rest_head}, polar_pose={polar_head_posed}",
    )

    rest_drawtube = ctx.part_world_position(drawtube)
    with ctx.pose({focus_slide: 0.040}):
        extended_drawtube = ctx.part_world_position(drawtube)
        ctx.expect_overlap(
            drawtube,
            telescope,
            axes="z",
            elem_a="drawtube_barrel",
            elem_b="focuser_sleeve",
            min_overlap=0.045,
            name="extended drawtube keeps sleeve engagement",
        )
    ctx.check(
        "focus slide extends the eyepiece outward",
        rest_drawtube is not None
        and extended_drawtube is not None
        and math.dist(rest_drawtube, extended_drawtube) > 0.035,
        details=f"rest={rest_drawtube}, extended={extended_drawtube}",
    )

    return ctx.report()


object_model = build_object_model()
