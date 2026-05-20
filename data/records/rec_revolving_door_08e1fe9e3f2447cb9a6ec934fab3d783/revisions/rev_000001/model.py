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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _curved_wall_mesh(
    radius: float,
    thickness: float,
    z_min: float,
    z_max: float,
    start_deg: float,
    end_deg: float,
    *,
    segments: int = 18,
) -> MeshGeometry:
    """Closed thin cylindrical wall segment used for curved drum glazing/rails."""
    geom = MeshGeometry()
    outer_r = radius + thickness / 2.0
    inner_r = radius - thickness / 2.0
    angles = [
        math.radians(start_deg + (end_deg - start_deg) * i / segments)
        for i in range(segments + 1)
    ]

    verts: list[tuple[int, int, int, int]] = []
    for a in angles:
        co, si = math.cos(a), math.sin(a)
        ob = geom.add_vertex(outer_r * co, outer_r * si, z_min)
        ot = geom.add_vertex(outer_r * co, outer_r * si, z_max)
        ib = geom.add_vertex(inner_r * co, inner_r * si, z_min)
        it = geom.add_vertex(inner_r * co, inner_r * si, z_max)
        verts.append((ob, ot, ib, it))

    for i in range(segments):
        ob0, ot0, ib0, it0 = verts[i]
        ob1, ot1, ib1, it1 = verts[i + 1]
        # Outer and inner faces.
        geom.add_face(ob0, ob1, ot1)
        geom.add_face(ob0, ot1, ot0)
        geom.add_face(ib1, ib0, it0)
        geom.add_face(ib1, it0, it1)
        # Top and bottom thickness faces.
        geom.add_face(ot0, ot1, it1)
        geom.add_face(ot0, it1, it0)
        geom.add_face(ob1, ob0, ib0)
        geom.add_face(ob1, ib0, ib1)

    # Close the two radial ends of the segment.
    for ob, ot, ib, it in (verts[0], verts[-1]):
        geom.add_face(ob, ot, it)
        geom.add_face(ob, it, ib)

    return geom


def _annular_cylinder_mesh(
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    """A hollow bronze canopy ring, open through the center."""
    geom = MeshGeometry()
    verts: list[tuple[int, int, int, int]] = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        co, si = math.cos(a), math.sin(a)
        ob = geom.add_vertex(outer_radius * co, outer_radius * si, z_min)
        ot = geom.add_vertex(outer_radius * co, outer_radius * si, z_max)
        ib = geom.add_vertex(inner_radius * co, inner_radius * si, z_min)
        it = geom.add_vertex(inner_radius * co, inner_radius * si, z_max)
        verts.append((ob, ot, ib, it))

    for i in range(segments):
        ob0, ot0, ib0, it0 = verts[i]
        ob1, ot1, ib1, it1 = verts[(i + 1) % segments]
        geom.add_face(ob0, ob1, ot1)
        geom.add_face(ob0, ot1, ot0)
        geom.add_face(ib1, ib0, it0)
        geom.add_face(ib1, it0, it1)
        geom.add_face(ot0, ot1, it1)
        geom.add_face(ot0, it1, it0)
        geom.add_face(ob1, ob0, ib0)
        geom.add_face(ob1, ib0, ib1)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heritage_four_wing_revolving_door")

    bronze = model.material("aged_bronze", rgba=(0.55, 0.34, 0.13, 1.0))
    dark_bronze = model.material("dark_bronze_shadow", rgba=(0.18, 0.11, 0.06, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.62, 0.86, 0.96, 0.34))

    drum = model.part("drum")
    rotor = model.part("wing_assembly")

    # Stationary bronze threshold/floor plate and open top canopy ring.
    drum.visual(
        Cylinder(radius=0.82, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=bronze,
        name="floor_plate",
    )
    drum.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.82, 0.64, 2.30, 2.42), "top_canopy_ring"),
        material=bronze,
        name="top_canopy_ring",
    )
    drum.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.82, 0.66, 0.10, 0.17), "low_threshold_ring"),
        material=dark_bronze,
        name="low_threshold_ring",
    )

    # Four curved stationary drum panels with bronze rails and vertical posts.
    panel_spans = ((18.0, 72.0), (108.0, 162.0), (198.0, 252.0), (288.0, 342.0))
    post_angles = sorted({angle for span in panel_spans for angle in span})
    for idx, (start, end) in enumerate(panel_spans):
        drum.visual(
            mesh_from_geometry(
                _curved_wall_mesh(0.735, 0.014, 0.32, 2.08, start, end),
                f"curved_glass_{idx}",
            ),
            material=glass,
            name=f"curved_glass_{idx}",
        )
        drum.visual(
            mesh_from_geometry(
                _curved_wall_mesh(0.735, 0.040, 0.23, 0.31, start, end),
                f"lower_curved_rail_{idx}",
            ),
            material=bronze,
            name=f"lower_curved_rail_{idx}",
        )
        drum.visual(
            mesh_from_geometry(
                _curved_wall_mesh(0.735, 0.040, 2.09, 2.18, start, end),
                f"upper_curved_rail_{idx}",
            ),
            material=bronze,
            name=f"upper_curved_rail_{idx}",
        )

    for idx, angle in enumerate(post_angles):
        a = math.radians(angle)
        drum.visual(
            Cylinder(radius=0.028, length=2.30),
            origin=Origin(xyz=(0.735 * math.cos(a), 0.735 * math.sin(a), 1.20)),
            material=bronze,
            name=f"drum_post_{idx}",
        )

    # Rotating round central post with decorative finial and four glazed wings.
    rotor.visual(
        Cylinder(radius=0.045, length=2.24),
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
        material=bronze,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.070, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=bronze,
        name="lower_post_collar",
    )
    rotor.visual(
        Cylinder(radius=0.070, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 2.155)),
        material=bronze,
        name="upper_post_collar",
    )
    rotor.visual(
        Cylinder(radius=0.110, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 2.3325)),
        material=bronze,
        name="finial_cap",
    )
    rotor.visual(
        Sphere(radius=0.065),
        origin=Origin(xyz=(0.0, 0.0, 2.405)),
        material=bronze,
        name="finial_ball",
    )
    rotor.visual(
        Cylinder(radius=0.028, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 2.495)),
        material=bronze,
        name="finial_tip",
    )

    inner_x = 0.075
    outer_x = 0.585
    wing_len = outer_x - inner_x
    wing_mid_x = (inner_x + outer_x) / 2.0
    for i in range(4):
        yaw = i * math.pi / 2.0
        rpy = (0.0, 0.0, yaw)
        rotor.visual(
            Box((wing_len - 0.040, 0.012, 1.740)),
            origin=Origin(xyz=(wing_mid_x, 0.0, 1.145), rpy=rpy),
            material=glass,
            name=f"wing_glass_{i}",
        )
        rotor.visual(
            Box((wing_len, 0.040, 0.060)),
            origin=Origin(xyz=(wing_mid_x, 0.0, 0.260), rpy=rpy),
            material=bronze,
            name=f"wing_bottom_rail_{i}",
        )
        rotor.visual(
            Box((wing_len, 0.040, 0.060)),
            origin=Origin(xyz=(wing_mid_x, 0.0, 2.030), rpy=rpy),
            material=bronze,
            name=f"wing_top_rail_{i}",
        )
        rotor.visual(
            Box((0.060, 0.044, 1.830)),
            origin=Origin(xyz=(inner_x, 0.0, 1.145), rpy=rpy),
            material=bronze,
            name=f"wing_inner_stile_{i}",
        )
        rotor.visual(
            Box((0.060, 0.044, 1.830)),
            origin=Origin(xyz=(outer_x, 0.0, 1.145), rpy=rpy),
            material=bronze,
            name=f"wing_outer_stile_{i}",
        )
        rotor.visual(
            Box((0.025, 0.030, 1.820)),
            origin=Origin(xyz=(wing_mid_x, 0.0, 1.145), rpy=rpy),
            material=bronze,
            name=f"wing_muntin_{i}",
        )
        rotor.visual(
            Box((wing_len - 0.020, 0.028, 0.030)),
            origin=Origin(xyz=(wing_mid_x, 0.0, 1.145), rpy=rpy),
            material=bronze,
            name=f"wing_midrail_{i}",
        )

    model.articulation(
        "drum_to_wing_assembly",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("drum")
    rotor = object_model.get_part("wing_assembly")
    spin = object_model.get_articulation("drum_to_wing_assembly")

    ctx.check(
        "wing assembly uses continuous rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spin.articulation_type}",
    )
    ctx.expect_gap(
        rotor,
        drum,
        axis="z",
        positive_elem="central_post",
        negative_elem="floor_plate",
        max_gap=0.001,
        max_penetration=0.00001,
        name="central post bears on bronze threshold plate",
    )
    ctx.expect_within(
        rotor,
        drum,
        axes="xy",
        outer_elem="floor_plate",
        margin=0.0,
        name="rotating wing sweep stays inside round drum footprint",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="wing_glass_0")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="wing_glass_0")
        ctx.expect_within(
            rotor,
            drum,
            axes="xy",
            outer_elem="floor_plate",
            margin=0.0,
            name="turned wing sweep remains inside drum footprint",
        )

    def _center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0)

    rest_center = _center_xy(rest_aabb)
    turned_center = _center_xy(turned_aabb)
    ctx.check(
        "quarter turn carries a wing around the post",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.25
        and abs(rest_center[1]) < 0.03
        and turned_center[1] > 0.25
        and abs(turned_center[0]) < 0.03,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
