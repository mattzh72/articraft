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
)


def _body_top_z(y: float) -> float:
    """Sloped top plane for the low wedge housing."""
    return 0.026 + ((y + 0.120) / 0.240) * (0.058 - 0.026)


def _wedge_prism(
    profile_xy: list[tuple[float, float]],
    *,
    bottom_z: float = 0.0,
    top_offset: float = 0.0,
    thickness: float | None = None,
) -> MeshGeometry:
    geom = MeshGeometry()
    n = len(profile_xy)

    if thickness is None:
        lower = [bottom_z for _x, _y in profile_xy]
        upper = [_body_top_z(y) + top_offset for _x, y in profile_xy]
    else:
        lower = [_body_top_z(y) + top_offset for _x, y in profile_xy]
        upper = [z + thickness for z in lower]

    for (x, y), z in zip(profile_xy, lower):
        geom.add_vertex(x, y, z)
    for (x, y), z in zip(profile_xy, upper):
        geom.add_vertex(x, y, z)

    # Bottom and top fans.
    for i in range(1, n - 1):
        geom.add_face(0, i + 1, i)
        geom.add_face(n, n + i, n + i + 1)

    # Side walls.
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(i, j, n + j)
        geom.add_face(i, n + j, n + i)

    return geom


def _antenna_blade() -> MeshGeometry:
    """Flat tapered router antenna blade in its hinge-local frame."""
    geom = MeshGeometry()
    z0, z1 = 0.036, 0.176
    y0, y1 = -0.0048, 0.0048
    half_bottom, half_top = 0.009, 0.014
    vertices = [
        (-half_bottom, y0, z0),
        (half_bottom, y0, z0),
        (half_bottom, y1, z0),
        (-half_bottom, y1, z0),
        (-half_top, y0, z1),
        (half_top, y0, z1),
        (half_top, y1, z1),
        (-half_top, y1, z1),
    ]
    for v in vertices:
        geom.add_vertex(*v)
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]
    for face in faces:
        geom.add_face(*face)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="six_antenna_gaming_router")

    matte_charcoal = model.material("matte_charcoal", rgba=(0.015, 0.017, 0.020, 1.0))
    satin_black = model.material("satin_black", rgba=(0.0, 0.0, 0.0, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.060, 0.065, 0.075, 1.0))
    blue_gloss = model.material("blue_gloss", rgba=(0.030, 0.100, 0.210, 1.0))
    green_led = model.material("green_led", rgba=(0.15, 1.0, 0.22, 1.0))
    white_mark = model.material("white_mark", rgba=(0.9, 0.92, 0.88, 1.0))

    body = model.part("housing")

    body_profile = [
        (-0.145, -0.120),
        (0.145, -0.120),
        (0.185, -0.070),
        (0.195, 0.120),
        (-0.195, 0.120),
        (-0.185, -0.070),
    ]
    body.visual(
        mesh_from_geometry(_wedge_prism(body_profile), "angular_wedge_housing"),
        material=matte_charcoal,
        name="wedge_shell",
    )

    top_profile = [
        (-0.110, -0.092),
        (0.110, -0.092),
        (0.148, -0.038),
        (0.158, 0.090),
        (-0.158, 0.090),
        (-0.148, -0.038),
    ]
    body.visual(
        mesh_from_geometry(
            _wedge_prism(top_profile, top_offset=-0.0010, thickness=0.003),
            "faceted_top_insert",
        ),
        material=blue_gloss,
        name="top_insert",
    )

    # Low top vents and status lights make the housing read as a gaming router.
    for idx, x in enumerate((-0.078, -0.026, 0.026, 0.078)):
        y = -0.020
        body.visual(
            Box((0.038, 0.006, 0.003)),
            origin=Origin(xyz=(x, y, _body_top_z(y) + 0.0010)),
            material=satin_black,
            name=f"vent_slot_{idx}",
        )

    for idx, x in enumerate((-0.038, -0.013, 0.013, 0.038)):
        body.visual(
            Box((0.012, 0.003, 0.004)),
            origin=Origin(xyz=(x, -0.1195, 0.020 + 0.002 * (idx % 2))),
            material=green_led,
            name=f"front_led_{idx}",
        )

    # Side opening for the rocker switch on the right side of the wedge.
    body.visual(
        Box((0.006, 0.058, 0.040)),
        origin=Origin(xyz=(0.189, -0.025, 0.037)),
        material=satin_black,
        name="switch_opening",
    )
    body.visual(
        Box((0.005, 0.064, 0.006)),
        origin=Origin(xyz=(0.191, -0.025, 0.060)),
        material=dark_graphite,
        name="switch_top_lip",
    )
    body.visual(
        Box((0.005, 0.064, 0.006)),
        origin=Origin(xyz=(0.191, -0.025, 0.014)),
        material=dark_graphite,
        name="switch_bottom_lip",
    )

    antenna_xs = [-0.158, -0.112, -0.066, 0.066, 0.112, 0.158]
    hinge_y = 0.126
    hinge_z = 0.076

    for idx, x in enumerate(antenna_xs):
        body.visual(
            Box((0.042, 0.036, 0.014)),
            origin=Origin(xyz=(x, 0.122, 0.058)),
            material=dark_graphite,
            name=f"pod_{idx}_base",
        )
        body.visual(
            Box((0.006, 0.030, 0.032)),
            origin=Origin(xyz=(x - 0.015, hinge_y, hinge_z)),
            material=dark_graphite,
            name=f"pod_{idx}_cheek_0",
        )
        body.visual(
            Box((0.006, 0.030, 0.032)),
            origin=Origin(xyz=(x + 0.015, hinge_y, hinge_z)),
            material=dark_graphite,
            name=f"pod_{idx}_cheek_1",
        )
        body.visual(
            Box((0.038, 0.006, 0.018)),
            origin=Origin(xyz=(x, 0.142, 0.074)),
            material=dark_graphite,
            name=f"pod_{idx}_rear_clip",
        )

        antenna = model.part(f"antenna_{idx}")
        antenna.visual(
            Cylinder(radius=0.011, length=0.024),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_graphite,
            name="hinge_barrel",
        )
        antenna.visual(
            Cylinder(radius=0.006, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, 0.021)),
            material=satin_black,
            name="lower_stem",
        )
        antenna.visual(
            mesh_from_geometry(_antenna_blade(), f"antenna_{idx}_blade_mesh"),
            material=satin_black,
            name="blade",
        )
        antenna.visual(
            Box((0.014, 0.011, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, 0.041)),
            material=dark_graphite,
            name="blade_root",
        )

        model.articulation(
            f"housing_to_antenna_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=antenna,
            origin=Origin(xyz=(x, hinge_y, hinge_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=-0.35, upper=1.35),
        )

    rocker = model.part("rocker_switch")
    rocker.visual(
        Box((0.008, 0.038, 0.025)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=dark_graphite,
        name="rocker_plate",
    )
    rocker.visual(
        Cylinder(radius=0.004, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="rocker_axle",
    )
    rocker.visual(
        Box((0.0015, 0.004, 0.013)),
        origin=Origin(xyz=(0.0077, 0.0, 0.004)),
        material=white_mark,
        name="power_mark",
    )

    model.articulation(
        "housing_to_rocker_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(0.192, -0.025, 0.037)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=1.5, lower=-0.25, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    rocker = object_model.get_part("rocker_switch")
    rocker_joint = object_model.get_articulation("housing_to_rocker_switch")

    ctx.allow_overlap(
        housing,
        rocker,
        elem_a="wedge_shell",
        elem_b="rocker_axle",
        reason="The rocker pivot axle is intentionally captured in the side-wall socket.",
    )

    ctx.expect_within(
        rocker,
        housing,
        axes="yz",
        inner_elem="rocker_axle",
        outer_elem="switch_opening",
        margin=0.004,
        name="rocker axle is centered in the side socket",
    )
    ctx.expect_overlap(
        rocker,
        housing,
        axes="x",
        elem_a="rocker_axle",
        elem_b="wedge_shell",
        min_overlap=0.002,
        name="rocker axle is retained inside the housing wall",
    )

    ctx.expect_within(
        rocker,
        housing,
        axes="yz",
        inner_elem="rocker_plate",
        outer_elem="switch_opening",
        margin=0.002,
        name="rocker sits inside the side opening",
    )

    rest_aabb = ctx.part_element_world_aabb(rocker, elem="rocker_plate")
    with ctx.pose({rocker_joint: 0.22}):
        tilted_aabb = ctx.part_element_world_aabb(rocker, elem="rocker_plate")
        ctx.expect_within(
            rocker,
            housing,
            axes="yz",
            inner_elem="rocker_plate",
            outer_elem="switch_opening",
            margin=0.004,
            name="rocker stays captured while tilted",
        )
    ctx.check(
        "rocker visibly tips on its side axis",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][0] > rest_aabb[1][0] + 0.001,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    for idx in range(6):
        antenna = object_model.get_part(f"antenna_{idx}")
        hinge = object_model.get_articulation(f"housing_to_antenna_{idx}")
        limits = hinge.motion_limits
        ctx.check(
            f"antenna {idx} has folding limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0
            and limits.upper > 1.0,
            details=f"limits={limits}",
        )
        ctx.expect_contact(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=f"pod_{idx}_base",
            contact_tol=0.001,
            name=f"antenna {idx} barrel rests on its pod",
        )
        ctx.expect_contact(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=f"pod_{idx}_cheek_0",
            contact_tol=0.001,
            name=f"antenna {idx} barrel is clipped by one cheek",
        )
        ctx.expect_contact(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=f"pod_{idx}_cheek_1",
            contact_tol=0.001,
            name=f"antenna {idx} barrel is clipped by the other cheek",
        )
        with ctx.pose({hinge: 1.20}):
            ctx.expect_contact(
                antenna,
                housing,
                elem_a="hinge_barrel",
                elem_b=f"pod_{idx}_base",
                contact_tol=0.0015,
                name=f"antenna {idx} remains clipped while folded",
            )

    return ctx.report()


object_model = build_object_model()
