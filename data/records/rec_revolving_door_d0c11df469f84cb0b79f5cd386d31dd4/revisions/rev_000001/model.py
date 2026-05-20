from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _arc_wall_geometry(
    *,
    radius: float,
    thickness: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    segments: int = 36,
) -> MeshGeometry:
    """Closed thin cylindrical wall segment, authored directly in meters."""

    geom = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    outer_r = radius + thickness / 2.0
    inner_r = radius - thickness / 2.0
    for i in range(segments + 1):
        t = start_angle + (end_angle - start_angle) * i / segments
        c = math.cos(t)
        s = math.sin(t)
        outer_bottom.append(geom.add_vertex(outer_r * c, outer_r * s, z_min))
        outer_top.append(geom.add_vertex(outer_r * c, outer_r * s, z_max))
        inner_bottom.append(geom.add_vertex(inner_r * c, inner_r * s, z_min))
        inner_top.append(geom.add_vertex(inner_r * c, inner_r * s, z_max))

    for i in range(segments):
        _add_quad(geom, outer_bottom[i], outer_bottom[i + 1], outer_top[i + 1], outer_top[i])
        _add_quad(geom, inner_bottom[i + 1], inner_bottom[i], inner_top[i], inner_top[i + 1])
        _add_quad(geom, outer_top[i], outer_top[i + 1], inner_top[i + 1], inner_top[i])
        _add_quad(geom, outer_bottom[i + 1], outer_bottom[i], inner_bottom[i], inner_bottom[i + 1])

    _add_quad(geom, outer_bottom[0], outer_top[0], inner_top[0], inner_bottom[0])
    _add_quad(geom, outer_bottom[-1], inner_bottom[-1], inner_top[-1], outer_top[-1])
    return geom


def _annular_cylinder_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 72,
) -> MeshGeometry:
    """Flat cylindrical ring with a real central opening."""

    geom = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_min))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_max))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_min))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_max))

    for i in range(segments):
        j = (i + 1) % segments
        _add_quad(geom, outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        _add_quad(geom, inner_bottom[j], inner_bottom[i], inner_top[i], inner_top[j])
        _add_quad(geom, outer_top[i], outer_top[j], inner_top[j], inner_top[i])
        _add_quad(geom, outer_bottom[j], outer_bottom[i], inner_bottom[i], inner_bottom[j])
    return geom


def _polar(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="school_three_wing_revolving_door")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    glass = model.material("blue_tinted_safety_glass", rgba=(0.52, 0.76, 0.95, 0.34))
    floor_mat = model.material("speckled_school_floor", rgba=(0.34, 0.35, 0.34, 1.0))
    brake_gray = model.material("brake_housing_gray", rgba=(0.30, 0.32, 0.34, 1.0))
    warning = model.material("yellow_speed_label", rgba=(1.0, 0.76, 0.08, 1.0))

    drum_radius = 1.35
    wall_thickness = 0.024
    wall_half_angle = math.radians(47.0)
    door_height = 2.28

    frame = model.part("entrance_frame")
    frame.visual(
        Cylinder(radius=1.58, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=floor_mat,
        name="base_slab",
    )
    frame.visual(
        mesh_from_geometry(
            _annular_cylinder_geometry(outer_radius=1.52, inner_radius=0.62, z_min=2.42, z_max=2.54),
            "ceiling_canopy_ring",
        ),
        material=aluminum,
        name="ceiling_canopy",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(1.30, 0.020), "floor_drum_track"),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=aluminum,
        name="floor_drum_track",
    )
    frame.visual(
        mesh_from_geometry(
            _annular_cylinder_geometry(outer_radius=0.075, inner_radius=0.055, z_min=0.075, z_max=0.130),
            "lower_post_bearing",
        ),
        material=aluminum,
        name="floor_center_bearing",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(0.093, 0.014), "ceiling_center_bearing"),
        origin=Origin(xyz=(0.0, 0.0, 2.42)),
        material=aluminum,
        name="ceiling_center_bearing",
    )
    for idx, angle in enumerate((math.pi / 2.0, 7.0 * math.pi / 6.0, 11.0 * math.pi / 6.0)):
        frame.visual(
            Box((0.56, 0.035, 0.030)),
            origin=Origin(xyz=_polar(0.35, angle, 2.43), rpy=(0.0, 0.0, angle)),
            material=aluminum,
            name=f"ceiling_bearing_spoke_{idx}",
        )

    east_wall = _arc_wall_geometry(
        radius=drum_radius,
        thickness=wall_thickness,
        z_min=0.08,
        z_max=2.42,
        start_angle=-wall_half_angle,
        end_angle=wall_half_angle,
    )
    west_wall = _arc_wall_geometry(
        radius=drum_radius,
        thickness=wall_thickness,
        z_min=0.08,
        z_max=2.42,
        start_angle=math.pi - wall_half_angle,
        end_angle=math.pi + wall_half_angle,
    )
    frame.visual(
        mesh_from_geometry(east_wall, "east_curved_drum_wall"),
        material=glass,
        name="east_curved_wall",
    )
    frame.visual(
        mesh_from_geometry(west_wall, "west_curved_drum_wall"),
        material=glass,
        name="west_curved_wall",
    )
    for idx, angle in enumerate(
        (
            -wall_half_angle,
            wall_half_angle,
            math.pi - wall_half_angle,
            math.pi + wall_half_angle,
        )
    ):
        frame.visual(
            Cylinder(radius=0.037, length=2.34),
            origin=Origin(xyz=_polar(drum_radius, angle, 1.25)),
            material=aluminum,
            name=f"drum_mullion_{idx}",
        )

    # Visible gentle-speed brake mounted under the school entry canopy.
    frame.visual(
        Box((0.16, 0.12, 0.16)),
        origin=Origin(xyz=(0.44, 0.0, 2.43)),
        material=brake_gray,
        name="brake_housing",
    )
    frame.visual(
        Box((0.14, 0.038, 0.040)),
        origin=Origin(xyz=(0.307, 0.0, 2.36)),
        material=brake_gray,
        name="brake_arm",
    )
    frame.visual(
        Box((0.034, 0.10, 0.060)),
        origin=Origin(xyz=(0.220, 0.0, 2.36)),
        material=dark_rubber,
        name="brake_shoe",
    )
    frame.visual(
        Box((0.22, 0.10, 0.020)),
        origin=Origin(xyz=(0.55, 0.0, 2.515)),
        material=brake_gray,
        name="brake_mount_plate",
    )
    frame.visual(
        Box((0.070, 0.006, 0.032)),
        origin=Origin(xyz=(0.448, -0.061, 2.43)),
        material=warning,
        name="slow_speed_label",
    )

    rotor = model.part("wing_rotor")
    rotor.visual(
        Cylinder(radius=0.055, length=2.30),
        origin=Origin(xyz=(0.0, 0.0, 1.23)),
        material=aluminum,
        name="center_post",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=aluminum,
        name="bottom_hub",
    )
    rotor.visual(
        Cylinder(radius=0.15, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 2.335)),
        material=aluminum,
        name="top_hub",
    )
    rotor.visual(
        Cylinder(radius=0.200, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 2.36)),
        material=aluminum,
        name="brake_disk",
    )

    panel_start = 0.078
    panel_length = 1.15
    panel_center = panel_start + panel_length / 2.0
    outer_rail_center = panel_start + panel_length + 0.018
    sweep_center = panel_start + panel_length + 0.055
    panel_z = 1.24

    for idx in range(3):
        angle = idx * 2.0 * math.pi / 3.0
        yaw = Origin(rpy=(0.0, 0.0, angle))

        def wing_origin(r: float, z: float) -> Origin:
            return Origin(xyz=_polar(r, angle, z), rpy=yaw.rpy)

        rotor.visual(
            Box((panel_length, 0.026, door_height - 0.34)),
            origin=wing_origin(panel_center, panel_z),
            material=glass,
            name=f"glass_panel_{idx}",
        )
        rotor.visual(
            Box((0.048, 0.064, door_height - 0.18)),
            origin=wing_origin(panel_start + 0.016, panel_z),
            material=aluminum,
            name=f"center_rail_{idx}",
        )
        rotor.visual(
            Box((0.050, 0.070, door_height - 0.20)),
            origin=wing_origin(outer_rail_center, panel_z),
            material=aluminum,
            name=f"outer_rail_{idx}",
        )
        rotor.visual(
            Box((panel_length, 0.052, 0.052)),
            origin=wing_origin(panel_center, 0.205),
            material=aluminum,
            name=f"bottom_rail_{idx}",
        )
        rotor.visual(
            Box((panel_length, 0.052, 0.052)),
            origin=wing_origin(panel_center, 2.275),
            material=aluminum,
            name=f"top_rail_{idx}",
        )
        rotor.visual(
            Box((0.034, 0.085, door_height - 0.26)),
            origin=wing_origin(sweep_center, panel_z),
            material=dark_rubber,
            name=f"outer_sweep_{idx}",
        )
        # A low-profile push rail is bonded to the glass so each wing reads as an
        # entrance panel, not merely as a transparent blade.
        rotor.visual(
            Box((0.58, 0.020, 0.060)),
            origin=wing_origin(panel_start + 0.58, 1.06),
            material=aluminum,
            name=f"push_bar_{idx}",
        )

    spin = model.articulation(
        "frame_to_wing_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.65),
        motion_properties=MotionProperties(damping=4.5, friction=0.45),
    )
    spin.meta["description"] = "Continuous vertical shaft rotation with passive damping from the overhead gentle-speed brake."

    adjuster = model.part("brake_adjuster")
    adjuster.visual(
        Cylinder(radius=0.041, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="knob_body",
    )
    adjuster.visual(
        Box((0.006, 0.012, 0.054)),
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        material=warning,
        name="knob_pointer",
    )
    model.articulation(
        "brake_adjuster_turn",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=adjuster,
        origin=Origin(xyz=(0.52, 0.0, 2.43)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=1.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("entrance_frame")
    rotor = object_model.get_part("wing_rotor")
    adjuster = object_model.get_part("brake_adjuster")
    spin = object_model.get_articulation("frame_to_wing_rotor")

    ctx.allow_overlap(
        frame,
        rotor,
        elem_a="floor_center_bearing",
        elem_b="center_post",
        reason="The revolving door post is intentionally captured inside the lower bearing sleeve proxy.",
    )

    ctx.check(
        "wing rotor uses continuous vertical shaft joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "rotor joint has gentle-speed damping",
        spin.motion_properties is not None and (spin.motion_properties.damping or 0.0) >= 4.0,
        details=f"motion_properties={spin.motion_properties}",
    )
    ctx.expect_origin_distance(
        frame,
        rotor,
        axes="xy",
        max_dist=0.001,
        name="rotating wing post is centered in the drum",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="center_post",
        outer_elem="floor_center_bearing",
        margin=0.001,
        name="center post is retained by lower bearing",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="z",
        elem_a="center_post",
        elem_b="floor_center_bearing",
        min_overlap=0.045,
        name="center post has bearing insertion depth",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="outer_sweep_0",
        outer_elem="base_slab",
        margin=0.0,
        name="wing sweep fits inside circular floor footprint",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="x",
        positive_elem="brake_shoe",
        negative_elem="brake_disk",
        min_gap=0.003,
        max_gap=0.020,
        name="brake shoe runs close to rotating brake disk",
    )
    ctx.expect_overlap(
        frame,
        rotor,
        axes="yz",
        elem_a="brake_shoe",
        elem_b="brake_disk",
        min_overlap=0.015,
        name="brake shoe overlaps disk face area",
    )
    ctx.expect_contact(
        adjuster,
        frame,
        elem_a="knob_body",
        elem_b="brake_housing",
        contact_tol=0.001,
        name="brake adjuster knob is seated on housing",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="glass_panel_0")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="glass_panel_0")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

    rest_center = _aabb_center(rest_aabb)
    turned_center = _aabb_center(turned_aabb)
    ctx.check(
        "panel wing moves around vertical post",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.45
        and abs(rest_center[1]) < 0.08
        and turned_center[1] > 0.45
        and abs(turned_center[0]) < 0.08,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
