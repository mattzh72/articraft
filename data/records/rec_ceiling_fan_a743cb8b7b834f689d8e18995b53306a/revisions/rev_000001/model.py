from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_disc_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 72,
) -> MeshGeometry:
    """Build a connected annular cylinder around local Z."""
    geom = MeshGeometry()
    z_top = height * 0.5
    z_bottom = -height * 0.5
    outer_top: list[int] = []
    outer_bottom: list[int] = []
    inner_top: list[int] = []
    inner_bottom: list[int] = []

    for index in range(segments):
        angle = math.tau * index / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_top))
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_bottom))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_top))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_bottom))

    for index in range(segments):
        nxt = (index + 1) % segments
        # Outer wall.
        geom.add_face(outer_bottom[index], outer_bottom[nxt], outer_top[nxt])
        geom.add_face(outer_bottom[index], outer_top[nxt], outer_top[index])
        # Inner wall, reversed normal.
        geom.add_face(inner_bottom[index], inner_top[nxt], inner_bottom[nxt])
        geom.add_face(inner_bottom[index], inner_top[index], inner_top[nxt])
        # Top annular face.
        geom.add_face(inner_top[index], outer_top[nxt], inner_top[nxt])
        geom.add_face(inner_top[index], outer_top[index], outer_top[nxt])
        # Bottom annular face.
        geom.add_face(inner_bottom[index], inner_bottom[nxt], outer_bottom[nxt])
        geom.add_face(inner_bottom[index], outer_bottom[nxt], outer_bottom[index])

    return geom


def _blade_mesh(angle: float) -> MeshGeometry:
    """Tapered angular aluminium blade with a slight pitched section."""
    # A hard-edged, swept contemporary blade planform in the +X direction.
    profile = [
        (0.165, -0.060),
        (0.280, -0.112),
        (0.790, -0.142),
        (0.960, -0.088),
        (0.935, 0.042),
        (0.270, 0.108),
        (0.165, 0.070),
    ]
    geom = MeshGeometry()
    thickness = 0.012
    base_z = -0.050
    pitch_slope = 0.050
    camber = 0.006
    top: list[int] = []
    bottom: list[int] = []
    cx = sum(p[0] for p in profile) / len(profile)
    cy = sum(p[1] for p in profile) / len(profile)
    chord_span = 0.960 - 0.165

    for x, y in profile:
        radial_t = max(0.0, min(1.0, (x - 0.165) / chord_span))
        z_mid = base_z + pitch_slope * y + camber * math.sin(math.pi * radial_t)
        top.append(geom.add_vertex(x, y, z_mid + thickness * 0.5))
        bottom.append(geom.add_vertex(x, y, z_mid - thickness * 0.5))

    z_center = base_z + camber * 0.5
    top_center = geom.add_vertex(cx, cy, z_center + thickness * 0.5)
    bottom_center = geom.add_vertex(cx, cy, z_center - thickness * 0.5)
    count = len(profile)
    for index in range(count):
        nxt = (index + 1) % count
        geom.add_face(top_center, top[index], top[nxt])
        geom.add_face(bottom_center, bottom[nxt], bottom[index])
        geom.add_face(bottom[index], bottom[nxt], top[nxt])
        geom.add_face(bottom[index], top[nxt], top[index])

    return geom.rotate_z(angle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="contemporary_ceiling_fan")

    ceiling_white = model.material("ceiling_white", rgba=(0.92, 0.92, 0.89, 1.0))
    satin_aluminium = model.material("satin_aluminium", rgba=(0.72, 0.75, 0.77, 1.0))
    dark_shadow = model.material("dark_shadow", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    warm_diffuser = model.material("warm_diffuser", rgba=(1.0, 0.86, 0.56, 0.62))

    housing = model.part("housing")
    housing.visual(
        Box((0.86, 0.86, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=ceiling_white,
        name="ceiling_plane",
    )
    housing.visual(
        Cylinder(radius=0.215, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=ceiling_white,
        name="ceiling_canopy",
    )
    housing.visual(
        Cylinder(radius=0.255, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=brushed_steel,
        name="motor_disc",
    )
    housing.visual(
        Cylinder(radius=0.258, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.126)),
        material=satin_aluminium,
        name="lower_trim",
    )
    housing.visual(
        Cylinder(radius=0.054, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, -0.1325)),
        material=dark_shadow,
        name="bearing_socket",
    )
    # Two slim fixed hinge straps drop through the open center of the rotor ring.
    for y in (-0.045, 0.045):
        housing.visual(
            Box((0.012, 0.024, 0.092)),
            origin=Origin(xyz=(-0.105, y, -0.172)),
            material=brushed_steel,
            name=f"hinge_strap_{0 if y < 0 else 1}",
        )
        housing.visual(
            Cylinder(radius=0.008, length=0.034),
            origin=Origin(xyz=(-0.105, -0.052 if y < 0 else 0.052, -0.225), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"fixed_knuckle_{0 if y < 0 else 1}",
        )
    housing.inertial = Inertial.from_geometry(Cylinder(radius=0.26, length=0.14), mass=5.0)

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.040, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=dark_shadow,
        name="axle_stub",
    )
    rotor.visual(
        mesh_from_geometry(
            _annular_disc_mesh(outer_radius=0.190, inner_radius=0.130, height=0.026),
            "blade_carrier_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=brushed_steel,
        name="blade_carrier",
    )
    rotor.visual(
        Cylinder(radius=0.075, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
        material=satin_aluminium,
        name="hub_cap",
    )
    for index in range(4):
        angle = index * math.tau / 4.0
        rotor.visual(
            Box((0.180, 0.026, 0.014)),
            origin=Origin(
                xyz=(0.090 * math.cos(angle), 0.090 * math.sin(angle), -0.046),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_steel,
            name=f"spoke_{index}",
        )
    for index in range(4):
        angle = index * math.tau / 4.0
        rotor.visual(
            mesh_from_geometry(_blade_mesh(angle), f"angular_blade_{index}"),
            material=satin_aluminium,
            name=f"blade_{index}",
        )
        radius = 0.220
        rotor.visual(
            Box((0.145, 0.054, 0.014)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), -0.045),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_steel,
            name=f"blade_clamp_{index}",
        )
    rotor.inertial = Inertial.from_geometry(Cylinder(radius=0.95, length=0.04), mass=3.0)

    light_tray = model.part("light_tray")
    light_tray.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="moving_knuckle",
    )
    light_tray.visual(
        Box((0.128, 0.060, 0.006)),
        origin=Origin(xyz=(-0.044, 0.0, -0.008)),
        material=brushed_steel,
        name="tray_hinge_leaf",
    )
    light_tray.visual(
        Cylinder(radius=0.098, length=0.018),
        origin=Origin(xyz=(0.105, 0.0, -0.020)),
        material=satin_aluminium,
        name="tray_pan",
    )
    light_tray.visual(
        Cylinder(radius=0.084, length=0.006),
        origin=Origin(xyz=(0.105, 0.0, -0.032)),
        material=warm_diffuser,
        name="diffuser_lens",
    )
    light_tray.inertial = Inertial.from_geometry(Cylinder(radius=0.11, length=0.025), mass=0.45)

    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )
    model.articulation(
        "housing_to_light_tray",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=light_tray,
        origin=Origin(xyz=(-0.105, 0.0, -0.225)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    light_tray = object_model.get_part("light_tray")
    spin = object_model.get_articulation("housing_to_rotor")
    tray_hinge = object_model.get_articulation("housing_to_light_tray")

    ctx.check("four distinct blades", all(rotor.get_visual(f"blade_{i}") is not None for i in range(4)))
    ctx.check(
        "rotor uses central vertical axle",
        spin.axis == (0.0, 0.0, 1.0) and spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"axis={spin.axis}, type={spin.articulation_type}",
    )
    ctx.check(
        "light tray has limited service hinge",
        tray_hinge.motion_limits is not None
        and tray_hinge.motion_limits.lower == 0.0
        and 0.9 <= tray_hinge.motion_limits.upper <= 1.2,
        details=f"limits={tray_hinge.motion_limits}",
    )
    ctx.expect_gap(
        housing,
        rotor,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="bearing_socket",
        negative_elem="axle_stub",
        name="axle seats under bearing socket",
    )
    ctx.expect_within(
        light_tray,
        housing,
        axes="xy",
        inner_elem="tray_pan",
        outer_elem="motor_disc",
        margin=0.0,
        name="closed light tray sits inside motor disc footprint",
    )
    ctx.expect_gap(
        housing,
        light_tray,
        axis="z",
        min_gap=0.040,
        positive_elem="motor_disc",
        negative_elem="tray_pan",
        name="light tray hangs below housing underside",
    )

    rotor_aabb = ctx.part_world_aabb(rotor)
    if rotor_aabb is not None:
        mins, maxs = rotor_aabb
        diameter = max(float(maxs[0] - mins[0]), float(maxs[1] - mins[1]))
        ctx.check("ceiling fan blade span is full scale", diameter > 1.75, details=f"diameter={diameter:.3f}")
        ctx.check("rotor assembly remains low profile", float(maxs[2] - mins[2]) < 0.08, details=f"aabb={rotor_aabb}")

    rest_aabb = ctx.part_element_world_aabb(light_tray, elem="tray_pan")
    with ctx.pose({tray_hinge: 0.85}):
        open_aabb = ctx.part_element_world_aabb(light_tray, elem="tray_pan")
    if rest_aabb is not None and open_aabb is not None:
        rest_center_z = 0.5 * (float(rest_aabb[0][2]) + float(rest_aabb[1][2]))
        open_center_z = 0.5 * (float(open_aabb[0][2]) + float(open_aabb[1][2]))
        ctx.check(
            "light tray hinges downward",
            open_center_z < rest_center_z - 0.045,
            details=f"rest_z={rest_center_z:.3f}, open_z={open_center_z:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
