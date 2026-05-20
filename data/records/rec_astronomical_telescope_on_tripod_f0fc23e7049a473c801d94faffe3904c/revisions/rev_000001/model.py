from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    mesh_from_geometry,
)


def _rpy_for_z_to_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return an Origin.rpy that points a local +Z cylinder along *vector*."""
    x, y, z = vector
    radial = math.hypot(x, y)
    if radial < 1e-9:
        return (0.0, 0.0 if z >= 0.0 else math.pi, 0.0)
    yaw = math.atan2(y, x)
    pitch = math.atan2(radial, z)
    return (0.0, pitch, yaw)


def _add_cylinder_between(
    part,
    *,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    material: Material | str,
) -> None:
    vx = p1[0] - p0[0]
    vy = p1[1] - p0[1]
    vz = p1[2] - p0[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=_rpy_for_z_to_vector((vx, vy, vz)),
        ),
        material=material,
        name=name,
    )


def _hollow_x_cylinder_mesh(
    *,
    length: float,
    outer_radius: float,
    inner_radius: float,
    segments: int = 64,
) -> MeshGeometry:
    """Thin-walled open tube along local X with annular lips at both ends."""
    geom = MeshGeometry()
    outer_left: list[int] = []
    outer_right: list[int] = []
    inner_left: list[int] = []
    inner_right: list[int] = []
    half = length * 0.5

    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        ca = math.cos(angle)
        sa = math.sin(angle)
        outer_left.append(geom.add_vertex(-half, outer_radius * ca, outer_radius * sa))
        outer_right.append(geom.add_vertex(half, outer_radius * ca, outer_radius * sa))
        inner_left.append(geom.add_vertex(-half, inner_radius * ca, inner_radius * sa))
        inner_right.append(geom.add_vertex(half, inner_radius * ca, inner_radius * sa))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall
        geom.add_face(outer_left[i], outer_right[i], outer_right[j])
        geom.add_face(outer_left[i], outer_right[j], outer_left[j])
        # Inner wall, reversed so the visible normals face the bore.
        geom.add_face(inner_left[i], inner_right[j], inner_right[i])
        geom.add_face(inner_left[i], inner_left[j], inner_right[j])
        # Left annular lip.
        geom.add_face(outer_left[i], outer_left[j], inner_left[j])
        geom.add_face(outer_left[i], inner_left[j], inner_left[i])
        # Right annular lip.
        geom.add_face(outer_right[i], inner_right[i], inner_right[j])
        geom.add_face(outer_right[i], inner_right[j], outer_right[j])

    return geom


def _hollow_z_cylinder_mesh(
    *,
    length: float,
    outer_radius: float,
    inner_radius: float,
    segments: int = 48,
) -> MeshGeometry:
    """Thin-walled open cylinder along local Z."""
    geom = MeshGeometry()
    outer_low: list[int] = []
    outer_high: list[int] = []
    inner_low: list[int] = []
    inner_high: list[int] = []
    half = length * 0.5

    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        ca = math.cos(angle)
        sa = math.sin(angle)
        outer_low.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, -half))
        outer_high.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, half))
        inner_low.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, -half))
        inner_high.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, half))

    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(outer_low[i], outer_high[i], outer_high[j])
        geom.add_face(outer_low[i], outer_high[j], outer_low[j])
        geom.add_face(inner_low[i], inner_high[j], inner_high[i])
        geom.add_face(inner_low[i], inner_low[j], inner_high[j])
        geom.add_face(outer_low[i], outer_low[j], inner_low[j])
        geom.add_face(outer_low[i], inner_low[j], inner_low[i])
        geom.add_face(outer_high[i], inner_high[i], inner_high[j])
        geom.add_face(outer_high[i], inner_high[j], outer_high[j])

    return geom


def _center_z(aabb):
    return (aabb[0][2] + aabb[1][2]) * 0.5 if aabb is not None else None


def _center_xy(aabb):
    if aabb is None:
        return None
    return ((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][1] + aabb[1][1]) * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="amateur_observatory_telescope")

    satin_black = model.material("satin_black", rgba=(0.012, 0.013, 0.014, 1.0))
    matte_black = model.material("matte_black", rgba=(0.0, 0.0, 0.0, 1.0))
    white_enamel = model.material("white_enamel", rgba=(0.92, 0.94, 0.92, 1.0))
    anodized = model.material("anodized_aluminum", rgba=(0.18, 0.20, 0.22, 1.0))
    brushed = model.material("brushed_metal", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.022, 1.0))
    blue_glass = model.material("coated_glass", rgba=(0.14, 0.36, 0.55, 0.48))
    red_mark = model.material("red_index_mark", rgba=(0.8, 0.05, 0.035, 1.0))

    tripod = model.part("tripod")

    # Central tripod pier, spreader tray, and top turntable support.
    tripod.visual(
        Cylinder(radius=0.040, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        material=anodized,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.100, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        material=satin_black,
        name="leg_hub",
    )
    tripod.visual(
        Cylinder(radius=0.135, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 1.045)),
        material=brushed,
        name="azimuth_bearing_plate",
    )
    tripod.visual(
        Cylinder(radius=0.155, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=satin_black,
        name="accessory_tray",
    )
    tripod.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        material=matte_black,
        name="tray_center_boss",
    )
    tripod.visual(
        Cylinder(radius=0.072, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 1.023)),
        material=satin_black,
        name="bearing_neck",
    )

    upper_attach_radius = 0.105
    upper_exit_radius = 0.600
    attach_z = 0.94
    exit_z = 0.35
    lower_visible = 0.46
    lower_insertion = 0.22
    lower_total = lower_visible + lower_insertion
    lower_travel = 0.14

    for i, angle in enumerate((math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)):
        ca = math.cos(angle)
        sa = math.sin(angle)
        p_top = (upper_attach_radius * ca, upper_attach_radius * sa, attach_z)
        p_exit = (upper_exit_radius * ca, upper_exit_radius * sa, exit_z)
        p_tray = (0.32 * ca, 0.32 * sa, 0.49)
        p_foot_hint = ((upper_exit_radius + 0.34) * ca, (upper_exit_radius + 0.34) * sa, 0.04)

        _add_cylinder_between(
            tripod,
            name=f"upper_sleeve_{i}",
            p0=p_top,
            p1=p_exit,
            radius=0.030,
            material=anodized,
        )
        _add_cylinder_between(
            tripod,
            name=f"leg_socket_{i}",
            p0=(0.045 * ca, 0.045 * sa, 0.955),
            p1=p_top,
            radius=0.036,
            material=satin_black,
        )
        _add_cylinder_between(
            tripod,
            name=f"tray_strut_{i}",
            p0=(0.080 * ca, 0.080 * sa, 0.505),
            p1=p_tray,
            radius=0.012,
            material=brushed,
        )
        _add_cylinder_between(
            tripod,
            name=f"tray_leg_brace_{i}",
            p0=p_tray,
            p1=(0.35 * ca, 0.35 * sa, 0.405),
            radius=0.010,
            material=brushed,
        )

        leg_direction = (p_foot_hint[0] - p_exit[0], p_foot_hint[1] - p_exit[1], p_foot_hint[2] - p_exit[2])
        leg_rpy = _rpy_for_z_to_vector(leg_direction)
        collar_center = (
            p_exit[0] - 0.025 * leg_direction[0] / math.sqrt(sum(v * v for v in leg_direction)),
            p_exit[1] - 0.025 * leg_direction[1] / math.sqrt(sum(v * v for v in leg_direction)),
            p_exit[2] - 0.025 * leg_direction[2] / math.sqrt(sum(v * v for v in leg_direction)),
        )
        tripod.visual(
            mesh_from_geometry(
                _hollow_z_cylinder_mesh(length=0.055, outer_radius=0.040, inner_radius=0.029, segments=48),
                f"leg_clamp_collar_mesh_{i}",
            ),
            origin=Origin(xyz=collar_center, rpy=leg_rpy),
            material=satin_black,
            name=f"leg_clamp_collar_{i}",
        )
        # A small captive thumbscrew is fused into the collar so it reads as supported.
        knob_offset_angle = angle + math.pi / 2.0
        knob_base = (collar_center[0] + 0.032 * math.cos(knob_offset_angle), collar_center[1] + 0.032 * math.sin(knob_offset_angle), collar_center[2])
        knob_tip = (collar_center[0] + 0.070 * math.cos(knob_offset_angle), collar_center[1] + 0.070 * math.sin(knob_offset_angle), collar_center[2])
        _add_cylinder_between(
            tripod,
            name=f"leg_clamp_screw_{i}",
            p0=knob_base,
            p1=knob_tip,
            radius=0.009,
            material=satin_black,
        )
        tripod.visual(
            Sphere(radius=0.020),
            origin=Origin(xyz=knob_tip),
            material=rubber,
            name=f"leg_clamp_pad_{i}",
        )

        lower_leg = model.part(f"lower_leg_{i}")
        lower_leg.visual(
            Cylinder(radius=0.019, length=lower_total),
            origin=Origin(xyz=(0.0, 0.0, (lower_visible - lower_insertion) * 0.5)),
            material=brushed,
            name="lower_tube",
        )
        lower_leg.visual(
            Cylinder(radius=0.037, length=0.048),
            origin=Origin(xyz=(0.0, 0.0, lower_visible + 0.022)),
            material=rubber,
            name="rubber_foot",
        )
        lower_leg.visual(
            Cylinder(radius=0.025, length=0.035),
            origin=Origin(xyz=(0.0, 0.0, lower_visible - 0.020)),
            material=satin_black,
            name="foot_socket",
        )
        model.articulation(
            f"leg_slide_{i}",
            ArticulationType.PRISMATIC,
            parent=tripod,
            child=lower_leg,
            origin=Origin(xyz=p_exit, rpy=leg_rpy),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=-0.05, upper=lower_travel),
        )

    mount_head = model.part("mount_head")
    mount_head.visual(
        Cylinder(radius=0.140, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=satin_black,
        name="azimuth_turntable",
    )
    mount_head.visual(
        Cylinder(radius=0.152, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=brushed,
        name="setting_circle",
    )
    for i in range(24):
        a = 2.0 * math.pi * i / 24
        tick_len = 0.018 if i % 3 == 0 else 0.011
        tick_width = 0.0035 if i % 3 == 0 else 0.0022
        r = 0.145 - tick_len * 0.5
        mount_head.visual(
            Box((tick_len, tick_width, 0.004)),
            origin=Origin(xyz=(r * math.cos(a), r * math.sin(a), 0.051), rpy=(0.0, 0.0, a)),
            material=matte_black,
            name=f"azimuth_tick_{i}",
        )
    mount_head.visual(
        Cylinder(radius=0.058, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=anodized,
        name="mount_pedestal",
    )
    mount_head.visual(
        Box((0.210, 0.325, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, 0.191)),
        material=satin_black,
        name="yoke_base",
    )
    for y, cheek_name, cap_name in (
        (-0.1325, "yoke_cheek_0", "bearing_cap_0"),
        (0.1325, "yoke_cheek_1", "bearing_cap_1"),
    ):
        mount_head.visual(
            Box((0.168, 0.040, 0.245)),
            origin=Origin(xyz=(0.0, y, 0.342)),
            material=satin_black,
            name=cheek_name,
        )
        mount_head.visual(
            Cylinder(radius=0.041, length=0.020),
            origin=Origin(xyz=(0.0, y + (0.030 if y > 0.0 else -0.030), 0.342), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name=cap_name,
        )
    mount_head.visual(
        Box((0.120, 0.040, 0.028)),
        origin=Origin(xyz=(-0.090, -0.1725, 0.435)),
        material=satin_black,
        name="slow_motion_housing",
    )
    mount_head.visual(
        Cylinder(radius=0.023, length=0.030),
        origin=Origin(xyz=(-0.132, -0.205, 0.435), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="altitude_tension_knob",
    )
    mount_head.visual(
        Box((0.050, 0.006, 0.014)),
        origin=Origin(xyz=(0.065, -0.155, 0.405)),
        material=red_mark,
        name="altitude_index_mark",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=mount_head,
        origin=Origin(xyz=(0.0, 0.0, 1.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.7),
    )

    tube = model.part("tube")
    main_tube_mesh = mesh_from_geometry(
        _hollow_x_cylinder_mesh(length=1.08, outer_radius=0.092, inner_radius=0.078, segments=72),
        "main_optical_tube",
    )
    tube.visual(main_tube_mesh, material=white_enamel, name="main_tube_shell")
    dew_mesh = mesh_from_geometry(
        _hollow_x_cylinder_mesh(length=0.260, outer_radius=0.110, inner_radius=0.092, segments=72),
        "dew_shield",
    )
    tube.visual(dew_mesh, origin=Origin(xyz=(0.560, 0.0, 0.0)), material=white_enamel, name="dew_shield")
    tube.visual(
        Cylinder(radius=0.092, length=0.010),
        origin=Origin(xyz=(0.684, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue_glass,
        name="objective_glass",
    )
    tube.visual(
        Cylinder(radius=0.085, length=0.034),
        origin=Origin(xyz=(-0.555, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="rear_cell",
    )
    tube.visual(
        Cylinder(radius=0.036, length=0.125),
        origin=Origin(xyz=(-0.633, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="drawtube",
    )
    tube.visual(
        Cylinder(radius=0.025, length=0.075),
        origin=Origin(xyz=(-0.7325, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="eyepiece_barrel",
    )
    tube.visual(
        Cylinder(radius=0.033, length=0.018),
        origin=Origin(xyz=(-0.7775, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="eyecup",
    )
    for x in (-0.190, 0.190):
        ring_mesh = mesh_from_geometry(
            _hollow_x_cylinder_mesh(length=0.045, outer_radius=0.108, inner_radius=0.090, segments=64),
            f"tube_ring_mesh_{x:+.2f}",
        )
        tube.visual(ring_mesh, origin=Origin(xyz=(x, 0.0, 0.0)), material=satin_black, name=f"tube_ring_{'rear' if x < 0 else 'front'}")
    tube.visual(
        Box((0.500, 0.048, 0.027)),
        origin=Origin(xyz=(0.0, 0.0, -0.1055)),
        material=anodized,
        name="dovetail_bar",
    )
    tube.visual(
        Cylinder(radius=0.025, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="trunnion_pin",
    )
    tube.visual(
        Cylinder(radius=0.030, length=0.035),
        origin=Origin(xyz=(0.0, -0.128, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="side_bearing_0",
    )
    tube.visual(
        Cylinder(radius=0.030, length=0.035),
        origin=Origin(xyz=(0.0, 0.128, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="side_bearing_1",
    )

    # Finder scope and its two real brackets mounted on the top of the tube.
    finder_mesh = mesh_from_geometry(
        _hollow_x_cylinder_mesh(length=0.390, outer_radius=0.027, inner_radius=0.020, segments=40),
        "finder_scope_tube",
    )
    tube.visual(finder_mesh, origin=Origin(xyz=(0.050, 0.0, 0.150)), material=satin_black, name="finder_scope")
    tube.visual(
        Cylinder(radius=0.031, length=0.024),
        origin=Origin(xyz=(0.257, 0.0, 0.150), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="finder_front_lip",
    )
    tube.visual(
        Cylinder(radius=0.031, length=0.024),
        origin=Origin(xyz=(-0.157, 0.0, 0.150), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="finder_rear_lip",
    )
    for x in (-0.100, 0.170):
        tube.visual(
            Box((0.035, 0.050, 0.058)),
            origin=Origin(xyz=(x, 0.0, 0.116)),
            material=satin_black,
            name=f"finder_stand_{'rear' if x < 0.0 else 'front'}",
        )
        tube.visual(
            Box((0.070, 0.070, 0.015)),
            origin=Origin(xyz=(x, 0.0, 0.095)),
            material=satin_black,
            name=f"finder_foot_{'rear' if x < 0.0 else 'front'}",
        )

    # Focuser housing provides the mounted face for a separate rotary focus knob.
    tube.visual(
        Box((0.110, 0.043, 0.060)),
        origin=Origin(xyz=(-0.485, -0.103, 0.0)),
        material=satin_black,
        name="focuser_housing",
    )

    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=mount_head,
        child=tube,
        origin=Origin(xyz=(0.0, 0.0, 0.342)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.55, lower=-0.35, upper=1.18),
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.028,
                body_style="cylindrical",
                grip=KnobGrip(style="knurled", count=30, depth=0.0010),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            ),
            "focus_knob_cap",
        ),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="focus_knob_cap",
    )
    focus_knob.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="focus_shaft",
    )
    model.articulation(
        "focus",
        ArticulationType.CONTINUOUS,
        parent=tube,
        child=focus_knob,
        origin=Origin(xyz=(-0.485, -0.1245, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    mount_head = object_model.get_part("mount_head")
    tube = object_model.get_part("tube")
    focus_knob = object_model.get_part("focus_knob")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    for i in range(3):
        leg = object_model.get_part(f"lower_leg_{i}")
        slide = object_model.get_articulation(f"leg_slide_{i}")
        ctx.allow_overlap(
            tripod,
            leg,
            elem_a=f"upper_sleeve_{i}",
            elem_b="lower_tube",
            reason="The lower tripod tube is intentionally nested inside the upper sleeve with retained insertion.",
        )
        ctx.expect_overlap(
            leg,
            tripod,
            axes="xyz",
            elem_a="lower_tube",
            elem_b=f"upper_sleeve_{i}",
            min_overlap=0.025,
            name=f"leg {i} has retained insertion at rest",
        )
        rest = ctx.part_world_position(leg)
        with ctx.pose({slide: 0.14}):
            ctx.expect_overlap(
                leg,
                tripod,
                axes="xyz",
                elem_a="lower_tube",
                elem_b=f"upper_sleeve_{i}",
                min_overlap=0.012,
                name=f"leg {i} remains inserted when extended",
            )
            extended = ctx.part_world_position(leg)
        ctx.check(
            f"leg {i} extends downward",
            rest is not None and extended is not None and extended[2] < rest[2] - 0.04,
            details=f"rest={rest}, extended={extended}",
        )

    ctx.allow_overlap(
        mount_head,
        tube,
        elem_a="yoke_cheek_0",
        elem_b="side_bearing_0",
        reason="The altitude side bearing is intentionally represented as a captured disk seated in the simplified solid yoke cheek bore.",
    )
    ctx.allow_overlap(
        mount_head,
        tube,
        elem_a="yoke_cheek_1",
        elem_b="side_bearing_1",
        reason="The opposite altitude side bearing is intentionally represented as a captured disk seated in the simplified solid yoke cheek bore.",
    )
    ctx.expect_overlap(
        tube,
        mount_head,
        axes="xyz",
        elem_a="side_bearing_0",
        elem_b="yoke_cheek_0",
        min_overlap=0.010,
        name="one side bearing is captured in the yoke cheek",
    )
    ctx.expect_overlap(
        tube,
        mount_head,
        axes="xyz",
        elem_a="side_bearing_1",
        elem_b="yoke_cheek_1",
        min_overlap=0.010,
        name="opposite side bearing is captured in the yoke cheek",
    )
    ctx.allow_overlap(
        tube,
        focus_knob,
        elem_a="focuser_housing",
        elem_b="focus_shaft",
        reason="The focus shaft is intentionally inserted into the focuser housing as a captured rotary axle.",
    )
    ctx.expect_overlap(
        focus_knob,
        tube,
        axes="xyz",
        elem_a="focus_shaft",
        elem_b="focuser_housing",
        min_overlap=0.002,
        name="focus shaft remains inserted in the focuser housing",
    )

    ctx.expect_contact(
        mount_head,
        tripod,
        elem_a="azimuth_turntable",
        elem_b="azimuth_bearing_plate",
        contact_tol=0.006,
        name="azimuth head seats on the tripod bearing plate",
    )
    ctx.expect_contact(
        tube,
        mount_head,
        elem_a="trunnion_pin",
        elem_b="yoke_cheek_0",
        contact_tol=0.002,
        name="tube trunnion is captured by one yoke cheek",
    )
    ctx.expect_contact(
        tube,
        mount_head,
        elem_a="trunnion_pin",
        elem_b="yoke_cheek_1",
        contact_tol=0.002,
        name="tube trunnion is captured by the opposite yoke cheek",
    )
    ctx.expect_contact(
        focus_knob,
        tube,
        elem_a="focus_shaft",
        elem_b="focuser_housing",
        contact_tol=0.004,
        name="focus knob shaft is seated in the focuser housing",
    )

    front_rest_aabb = ctx.part_element_world_aabb(tube, elem="objective_glass")
    rest_front_z = _center_z(front_rest_aabb)
    with ctx.pose({elevation: 0.82}):
        front_raised_aabb = ctx.part_element_world_aabb(tube, elem="objective_glass")
        raised_front_z = _center_z(front_raised_aabb)
    ctx.check(
        "elevation joint raises the objective end",
        rest_front_z is not None and raised_front_z is not None and raised_front_z > rest_front_z + 0.35,
        details=f"rest_z={rest_front_z}, raised_z={raised_front_z}",
    )

    front_xy_rest = _center_xy(ctx.part_element_world_aabb(tube, elem="objective_glass"))
    with ctx.pose({azimuth: 1.05}):
        front_xy_rotated = _center_xy(ctx.part_element_world_aabb(tube, elem="objective_glass"))
    ctx.check(
        "azimuth joint sweeps the telescope around the vertical axis",
        front_xy_rest is not None
        and front_xy_rotated is not None
        and abs(front_xy_rotated[1] - front_xy_rest[1]) > 0.45,
        details=f"rest_xy={front_xy_rest}, rotated_xy={front_xy_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
