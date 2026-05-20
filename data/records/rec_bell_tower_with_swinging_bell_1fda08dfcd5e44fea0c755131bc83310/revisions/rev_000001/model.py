from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _revolved_open_shell(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer_rings: list[list[int]] = []
    inner_rings: list[list[int]] = []

    for segment in range(segments):
        angle = 2.0 * math.pi * segment / segments
        c = math.cos(angle)
        s = math.sin(angle)

        outer_ring: list[int] = []
        for radius, z_pos in outer_profile:
            outer_ring.append(geom.add_vertex(radius * c, radius * s, z_pos))
        outer_rings.append(outer_ring)

        inner_ring: list[int] = []
        for radius, z_pos in inner_profile:
            inner_ring.append(geom.add_vertex(radius * c, radius * s, z_pos))
        inner_rings.append(inner_ring)

    for segment in range(segments):
        next_segment = (segment + 1) % segments

        for index in range(len(outer_profile) - 1):
            a = outer_rings[segment][index]
            b = outer_rings[next_segment][index]
            c = outer_rings[next_segment][index + 1]
            d = outer_rings[segment][index + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

        for index in range(len(inner_profile) - 1):
            a = inner_rings[segment][index]
            b = inner_rings[segment][index + 1]
            c = inner_rings[next_segment][index + 1]
            d = inner_rings[next_segment][index]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

        top_outer_a = outer_rings[segment][0]
        top_outer_b = outer_rings[next_segment][0]
        top_inner_b = inner_rings[next_segment][0]
        top_inner_a = inner_rings[segment][0]
        geom.add_face(top_outer_a, top_outer_b, top_inner_b)
        geom.add_face(top_outer_a, top_inner_b, top_inner_a)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_bell_post")

    painted_steel = model.material("painted_steel", rgba=(0.19, 0.21, 0.23, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    bronze = model.material("bronze", rgba=(0.63, 0.45, 0.23, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.44, 0.29, 0.16, 1.0))
    cord_tan = model.material("cord_tan", rgba=(0.73, 0.65, 0.52, 1.0))

    support_post = model.part("support_post")
    support_post.visual(
        Box((0.04, 0.04, 1.48)),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=painted_steel,
        name="main_post",
    )
    support_post.visual(
        Box((0.026, 0.026, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=weathered_steel,
        name="ground_stake",
    )
    support_post.visual(
        Box((0.39, 0.035, 0.035)),
        origin=Origin(xyz=(0.205, 0.0, 1.42)),
        material=painted_steel,
        name="cross_arm",
    )
    support_post.visual(
        Box((0.25, 0.014, 0.014)),
        origin=Origin(xyz=(0.12, 0.0, 1.34), rpy=(0.0, -0.80, 0.0)),
        material=painted_steel,
        name="brace_strut",
    )
    support_post.visual(
        Box((0.024, 0.004, 0.042)),
        origin=Origin(xyz=(0.32, 0.013, 1.382)),
        material=painted_steel,
        name="clevis_left",
    )
    support_post.visual(
        Box((0.024, 0.004, 0.042)),
        origin=Origin(xyz=(0.32, -0.013, 1.382)),
        material=painted_steel,
        name="clevis_right",
    )
    support_post.visual(
        Box((0.05, 0.05, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 1.485)),
        material=painted_steel,
        name="top_cap",
    )
    support_post.inertial = Inertial.from_geometry(
        Box((0.42, 0.06, 1.70)),
        mass=13.0,
        origin=Origin(xyz=(0.10, 0.0, 0.635)),
    )

    bell_outer_profile = [
        (0.016, 0.000),
        (0.027, -0.010),
        (0.046, -0.030),
        (0.064, -0.078),
        (0.076, -0.128),
        (0.086, -0.175),
        (0.093, -0.198),
        (0.097, -0.208),
    ]
    bell_inner_profile = [
        (0.010, 0.000),
        (0.018, -0.011),
        (0.034, -0.031),
        (0.049, -0.078),
        (0.060, -0.128),
        (0.069, -0.175),
        (0.076, -0.198),
        (0.081, -0.208),
    ]
    bell_shell_mesh = mesh_from_geometry(
        _revolved_open_shell(bell_outer_profile, bell_inner_profile, segments=80),
        "bronze_bell_shell",
    )

    bell = model.part("bell")
    bell.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_bronze,
        name="bell_trunnion",
    )
    bell.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=dark_bronze,
        name="bell_neck",
    )
    bell.visual(
        bell_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=bronze,
        name="bell_shell",
    )
    bell.visual(
        Box((0.022, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=dark_bronze,
        name="clapper_bridge",
    )
    bell.visual(
        Box((0.010, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, 0.0115, -0.051)),
        material=dark_bronze,
        name="clapper_hanger_left",
    )
    bell.visual(
        Box((0.010, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.0115, -0.051)),
        material=dark_bronze,
        name="clapper_hanger_right",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.23),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
    )

    clapper = model.part("clapper")
    clapper.visual(
        Cylinder(radius=0.0025, length=0.019),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=weathered_steel,
        name="clapper_axle",
    )
    clapper.visual(
        Cylinder(radius=0.004, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=weathered_steel,
        name="clapper_stem",
    )
    clapper.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.114)),
        material=dark_bronze,
        name="clapper_head",
    )
    clapper.visual(
        Cylinder(radius=0.0025, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.183)),
        material=cord_tan,
        name="pull_cord",
    )
    clapper.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.0, 0.0, -0.245)),
        material=cord_tan,
        name="pull_knob",
    )
    clapper.inertial = Inertial.from_geometry(
        Cylinder(radius=0.02, length=0.26),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
    )

    model.articulation(
        "support_to_bell",
        ArticulationType.REVOLUTE,
        parent=support_post,
        child=bell,
        origin=Origin(xyz=(0.32, 0.0, 1.372)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "bell_to_clapper",
        ArticulationType.REVOLUTE,
        parent=bell,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.051)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_post = object_model.get_part("support_post")
    bell = object_model.get_part("bell")
    clapper = object_model.get_part("clapper")
    bell_hinge = object_model.get_articulation("support_to_bell")
    clapper_hinge = object_model.get_articulation("bell_to_clapper")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        bell,
        support_post,
        elem_a="bell_trunnion",
        elem_b="clevis_left",
        name="bell trunnion bears on left clevis ear",
    )
    ctx.expect_contact(
        bell,
        support_post,
        elem_a="bell_trunnion",
        elem_b="clevis_right",
        name="bell trunnion bears on right clevis ear",
    )
    ctx.expect_contact(
        clapper,
        bell,
        elem_a="clapper_axle",
        elem_b="clapper_hanger_left",
        name="clapper axle bears on left hanger tab",
    )
    ctx.expect_contact(
        clapper,
        bell,
        elem_a="clapper_axle",
        elem_b="clapper_hanger_right",
        name="clapper axle bears on right hanger tab",
    )

    support_aabb = ctx.part_world_aabb(support_post)
    support_dims = None
    if support_aabb is not None:
        support_dims = tuple(
            support_aabb[1][axis] - support_aabb[0][axis] for axis in range(3)
        )
    ctx.check(
        "support reads as a tall garden post with projecting arm",
        support_dims is not None and support_dims[2] > 1.60 and support_dims[0] > 0.36,
        details=f"support_dims={support_dims}",
    )

    bell_rest_center = _aabb_center(ctx.part_element_world_aabb(bell, elem="bell_shell"))
    with ctx.pose({bell_hinge: 0.40}):
        bell_swung_center = _aabb_center(ctx.part_element_world_aabb(bell, elem="bell_shell"))
    ctx.check(
        "bell swings away from the post for positive hinge motion",
        bell_rest_center is not None
        and bell_swung_center is not None
        and bell_swung_center[0] > bell_rest_center[0] + 0.04,
        details=f"rest_center={bell_rest_center}, swung_center={bell_swung_center}",
    )

    pull_rest_center = _aabb_center(ctx.part_element_world_aabb(clapper, elem="pull_knob"))
    with ctx.pose({clapper_hinge: 0.35}):
        pull_moved_center = _aabb_center(ctx.part_element_world_aabb(clapper, elem="pull_knob"))
    ctx.check(
        "pull cord moves outward when the clapper is pulled",
        pull_rest_center is not None
        and pull_moved_center is not None
        and pull_moved_center[0] > pull_rest_center[0] + 0.02,
        details=f"rest_center={pull_rest_center}, moved_center={pull_moved_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
