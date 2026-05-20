from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _ellipse_section(x: float, y_radius: float, z_radius: float, z_center: float, count: int = 32):
    """Closed ellipse loop in the local YZ plane, ordered consistently along X."""
    return [
        (
            x,
            y_radius * math.cos(2.0 * math.pi * i / count),
            z_center + z_radius * math.sin(2.0 * math.pi * i / count),
        )
        for i in range(count)
    ]


def _extrude_z_prism(points_xy, z_center: float, thickness: float) -> MeshGeometry:
    """Make a simple capped prism from a 2-D XY polygon."""
    geom = MeshGeometry()
    half = thickness / 2.0
    bottom = [geom.add_vertex(x, y, z_center - half) for x, y in points_xy]
    top = [geom.add_vertex(x, y, z_center + half) for x, y in points_xy]
    n = len(points_xy)
    for i in range(1, n - 1):
        geom.add_face(bottom[0], bottom[i + 1], bottom[i])
        geom.add_face(top[0], top[i], top[i + 1])
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])
    return geom


def _extrude_y_prism(points_xz, y_center: float, thickness: float) -> MeshGeometry:
    """Make a capped prism from a 2-D XZ polygon, extruded along Y."""
    geom = MeshGeometry()
    half = thickness / 2.0
    near = [geom.add_vertex(x, y_center - half, z) for x, z in points_xz]
    far = [geom.add_vertex(x, y_center + half, z) for x, z in points_xz]
    n = len(points_xz)
    for i in range(1, n - 1):
        geom.add_face(near[0], near[i], near[i + 1])
        geom.add_face(far[0], far[i + 1], far[i])
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(near[i], far[j], near[j])
        geom.add_face(near[i], far[i], far[j])
    return geom


def _fuselage_geometry() -> MeshGeometry:
    sections = [
        _ellipse_section(-0.360, 0.014, 0.016, 0.105),
        _ellipse_section(-0.300, 0.033, 0.032, 0.105),
        _ellipse_section(-0.120, 0.057, 0.050, 0.105),
        _ellipse_section(0.120, 0.064, 0.055, 0.105),
        _ellipse_section(0.300, 0.048, 0.045, 0.105),
        _ellipse_section(0.360, 0.018, 0.020, 0.105),
    ]
    return section_loft(sections)


def _wing_geometry(side: int) -> MeshGeometry:
    root_y = 0.018 * side
    tip_y = 0.405 * side
    return _extrude_z_prism(
        [
            (0.145, root_y),
            (0.088, tip_y),
            (-0.125, tip_y),
            (-0.168, root_y),
        ],
        z_center=0.120,
        thickness=0.018,
    )


def _tailplane_geometry(side: int) -> MeshGeometry:
    root_y = 0.018 * side
    tip_y = 0.205 * side
    return _extrude_z_prism(
        [
            (-0.240, root_y),
            (-0.265, tip_y),
            (-0.365, tip_y),
            (-0.375, root_y),
        ],
        z_center=0.132,
        thickness=0.014,
    )


def _fin_geometry() -> MeshGeometry:
    return _extrude_y_prism(
        [
            (-0.350, 0.130),
            (-0.315, 0.225),
            (-0.240, 0.198),
            (-0.230, 0.132),
        ],
        y_center=0.0,
        thickness=0.016,
    )


def _prop_blade_geometry(sign: int) -> MeshGeometry:
    # Flat, fabrication-friendly blade with a slight sweep in Y, rooted inside the hub.
    root = 0.018 * sign
    tip = 0.145 * sign
    return _extrude_z_prism(
        [
            (0.030, -0.018),
            (0.044, -0.012),
            (0.044, 0.012),
            (0.030, 0.018),
        ],
        z_center=(root + tip) / 2.0,
        thickness=abs(tip - root),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_model_plane")

    powder_blue = model.material("powder_coated_blue", rgba=(0.12, 0.33, 0.78, 1.0))
    seal_black = model.material("uv_black_elastomer", rgba=(0.015, 0.018, 0.018, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.70, 1.0))
    dark_prop = model.material("matte_graphite_prop", rgba=(0.06, 0.065, 0.07, 1.0))
    canopy = model.material("smoked_polycarbonate", rgba=(0.16, 0.24, 0.31, 0.82))
    base_mat = model.material("sealed_dark_base", rgba=(0.09, 0.10, 0.095, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.380, 0.280, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=base_mat,
        name="base_plate",
    )
    stand.visual(
        Cylinder(radius=0.026, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.304)),
        material=stainless,
        name="sealed_post",
    )
    stand.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.588)),
        material=powder_blue,
        name="post_drip_cap",
    )
    stand.visual(
        Box((0.078, 0.160, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=powder_blue,
        name="yoke_bridge",
    )
    for idx, (y, bushing_name) in enumerate(((-0.080, "pivot_bushing_0"), (0.080, "pivot_bushing_1"))):
        stand.visual(
            Box((0.052, 0.018, 0.082)),
            origin=Origin(xyz=(0.0, y, 0.582)),
            material=powder_blue,
            name=f"yoke_lower_{idx}",
        )
        stand.visual(
            Box((0.052, 0.018, 0.072)),
            origin=Origin(xyz=(0.0, y, 0.670)),
            material=powder_blue,
            name=f"yoke_upper_{idx}",
        )
        stand.visual(
            Cylinder(radius=0.027, length=0.022),
            origin=Origin(xyz=(0.0, y * 0.9375, 0.620), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=bushing_name,
        )
    for idx, (x, y) in enumerate(((-0.145, -0.095), (-0.145, 0.095), (0.145, -0.095), (0.145, 0.095))):
        stand.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(x, y, 0.028)),
            material=stainless,
            name=f"deck_screw_{idx}",
        )

    plane = model.part("plane_body")
    plane.visual(
        mesh_from_geometry(_fuselage_geometry(), "sealed_fuselage"),
        material=powder_blue,
        name="sealed_fuselage",
    )
    for idx, side in enumerate((-1, 1)):
        plane.visual(
            mesh_from_geometry(_wing_geometry(side), f"main_wing_{idx}"),
            material=powder_blue,
            name=f"main_wing_{idx}",
        )
        plane.visual(
            mesh_from_geometry(_tailplane_geometry(side), f"tailplane_{idx}"),
            material=powder_blue,
            name=f"tailplane_{idx}",
        )
        plane.visual(
            Box((0.185, 0.010, 0.010)),
            origin=Origin(xyz=(-0.015, 0.040 * side, 0.108)),
            material=seal_black,
            name=f"wing_root_gasket_{idx}",
        )
        plane.visual(
            Box((0.090, 0.008, 0.008)),
            origin=Origin(xyz=(-0.307, 0.033 * side, 0.124)),
            material=seal_black,
            name=f"tail_gasket_{idx}",
        )
    plane.visual(
        mesh_from_geometry(_fin_geometry(), "vertical_fin"),
        material=powder_blue,
        name="vertical_fin",
    )
    plane.visual(
        Box((0.176, 0.076, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 0.166)),
        material=canopy,
        name="sealed_canopy",
    )
    plane.visual(
        Box((0.198, 0.088, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, 0.149)),
        material=seal_black,
        name="canopy_gasket",
    )
    for idx, y in enumerate((-0.055, 0.055)):
        plane.visual(
            Box((0.360, 0.008, 0.006)),
            origin=Origin(xyz=(-0.035, y, 0.134)),
            material=seal_black,
            name=f"side_drip_rail_{idx}",
        )
    plane.visual(
        Box((0.074, 0.040, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=powder_blue,
        name="sealed_saddle",
    )
    plane.visual(
        Cylinder(radius=0.013, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="pivot_pin",
    )
    plane.visual(
        Cylinder(radius=0.036, length=0.025),
        origin=Origin(xyz=(0.364, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_black,
        name="nose_cowl_gasket",
    )
    plane.visual(
        Cylinder(radius=0.043, length=0.010),
        origin=Origin(xyz=(0.350, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_blue,
        name="nose_drip_lip",
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="prop_hub",
    )
    propeller.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_blue,
        name="spinner_cap",
    )
    for idx, sign in enumerate((-1, 1)):
        propeller.visual(
            mesh_from_geometry(_prop_blade_geometry(sign), f"prop_blade_{idx}"),
            material=dark_prop,
            name=f"prop_blade_{idx}",
        )

    model.articulation(
        "stand_pitch",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=plane,
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.7, lower=-0.25, upper=0.25),
    )
    model.articulation(
        "prop_spin",
        ArticulationType.CONTINUOUS,
        parent=plane,
        child=propeller,
        origin=Origin(xyz=(0.3765, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    plane = object_model.get_part("plane_body")
    propeller = object_model.get_part("propeller")
    pitch = object_model.get_articulation("stand_pitch")
    spin = object_model.get_articulation("prop_spin")

    ctx.expect_contact(
        stand,
        plane,
        elem_a="pivot_bushing_0",
        elem_b="pivot_pin",
        contact_tol=0.0015,
        name="negative yoke bushing captures the pitch pin",
    )
    ctx.expect_contact(
        stand,
        plane,
        elem_a="pivot_bushing_1",
        elem_b="pivot_pin",
        contact_tol=0.0015,
        name="positive yoke bushing captures the pitch pin",
    )
    ctx.expect_contact(
        plane,
        propeller,
        elem_a="nose_cowl_gasket",
        elem_b="prop_hub",
        contact_tol=0.0015,
        name="propeller hub seats against sealed nose cowl",
    )
    ctx.expect_gap(
        propeller,
        plane,
        axis="x",
        positive_elem="prop_hub",
        negative_elem="nose_cowl_gasket",
        max_gap=0.0015,
        max_penetration=0.0,
        name="propeller hub is externally mounted without buried overlap",
    )

    def center_from_aabb(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    nose_rest = center_from_aabb(ctx.part_element_world_aabb(plane, elem="nose_cowl_gasket"))
    blade_rest = center_from_aabb(ctx.part_element_world_aabb(propeller, elem="prop_blade_1"))
    with ctx.pose({pitch: 0.25}):
        nose_raised = center_from_aabb(ctx.part_element_world_aabb(plane, elem="nose_cowl_gasket"))
    with ctx.pose({spin: math.pi / 2.0}):
        blade_turned = center_from_aabb(ctx.part_element_world_aabb(propeller, elem="prop_blade_1"))

    ctx.check(
        "stand pitch raises the nose",
        nose_rest is not None and nose_raised is not None and nose_raised[2] > nose_rest[2] + 0.035,
        details=f"rest={nose_rest}, raised={nose_raised}",
    )
    ctx.check(
        "propeller visibly spins about the nose shaft",
        blade_rest is not None
        and blade_turned is not None
        and abs(blade_turned[1] - blade_rest[1]) > 0.050,
        details=f"rest={blade_rest}, turned={blade_turned}",
    )

    return ctx.report()


object_model = build_object_model()
