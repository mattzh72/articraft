from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _helical_thread_mesh(
    *,
    root_radius: float,
    crest_radius: float,
    z0: float,
    height: float,
    pitch: float,
    thread_width: float,
    samples: int = 180,
) -> MeshGeometry:
    """Small triangular helical bead used for visible bottle-neck screw threads."""

    geom = MeshGeometry()
    turns = height / pitch
    theta_end = turns * 2.0 * math.pi
    rows: list[tuple[int, int, int]] = []

    for i in range(samples + 1):
        u = i / samples
        theta = theta_end * u
        z = z0 + height * u
        c = math.cos(theta)
        s = math.sin(theta)
        row = (
            geom.add_vertex(root_radius * c, root_radius * s, z - thread_width * 0.5),
            geom.add_vertex(crest_radius * c, crest_radius * s, z),
            geom.add_vertex(root_radius * c, root_radius * s, z + thread_width * 0.5),
        )
        rows.append(row)

    for prev, cur in zip(rows[:-1], rows[1:]):
        for a, b in ((0, 1), (1, 2), (2, 0)):
            geom.add_face(prev[a], cur[a], cur[b])
            geom.add_face(prev[a], cur[b], prev[b])

    geom.add_face(rows[0][0], rows[0][1], rows[0][2])
    geom.add_face(rows[-1][0], rows[-1][2], rows[-1][1])
    return geom


def _ribbed_cap_mesh(
    *,
    inner_radius: float,
    base_outer_radius: float,
    ridge_depth: float,
    height: float,
    top_thickness: float,
    ribs: int = 40,
    segments: int = 200,
) -> MeshGeometry:
    """Open-bottom, ribbed screw-cap shell with a solid top disk."""

    geom = MeshGeometry()
    z_mid = height - top_thickness
    top_center = geom.add_vertex(0.0, 0.0, height)
    under_center = geom.add_vertex(0.0, 0.0, z_mid)

    outer_bottom: list[int] = []
    outer_mid: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        c = math.cos(theta)
        s = math.sin(theta)
        # Squared cosine gives broad raised grip lands separated by shallow valleys.
        rib_wave = 0.5 + 0.5 * math.cos(ribs * theta)
        outer_radius = base_outer_radius + ridge_depth * (rib_wave * rib_wave)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, 0.0))
        outer_mid.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_mid))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, height))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, 0.0))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_mid))

    for i in range(segments):
        j = (i + 1) % segments

        # Corrugated exterior side.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_mid[j])
        geom.add_face(outer_bottom[i], outer_mid[j], outer_mid[i])
        geom.add_face(outer_mid[i], outer_mid[j], outer_top[j])
        geom.add_face(outer_mid[i], outer_top[j], outer_top[i])

        # Solid top face and the inside ceiling of the cap.
        geom.add_face(top_center, outer_top[i], outer_top[j])
        geom.add_face(under_center, inner_top[j], inner_top[i])
        geom.add_face(inner_top[i], inner_top[j], outer_mid[j])
        geom.add_face(inner_top[i], outer_mid[j], outer_mid[i])

        # Smooth inner wall and the open-bottom annular rim.
        geom.add_face(inner_bottom[j], inner_bottom[i], inner_top[i])
        geom.add_face(inner_bottom[j], inner_top[i], inner_top[j])
        geom.add_face(inner_bottom[i], inner_bottom[j], outer_bottom[j])
        geom.add_face(inner_bottom[i], outer_bottom[j], outer_bottom[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_plastic = model.material(
        "translucent_pet",
        rgba=(0.55, 0.82, 1.0, 0.42),
    )
    neck_plastic = model.material(
        "clear_neck_detail",
        rgba=(0.72, 0.92, 1.0, 0.58),
    )
    cap_plastic = model.material(
        "matte_white_cap",
        rgba=(0.94, 0.94, 0.90, 1.0),
    )

    bottle = model.part("bottle")

    # One thin-walled fixed body, shaped as three coaxial sections:
    # straight lower bottle, sloped shoulder, and a narrow threaded neck.
    bottle_shell = LatheGeometry.from_shell_profiles(
        [
            (0.048, 0.000),
            (0.055, 0.008),
            (0.055, 0.130),
            (0.050, 0.145),
            (0.032, 0.166),
            (0.022, 0.181),
            (0.020, 0.228),
        ],
        [
            (0.036, 0.006),
            (0.049, 0.014),
            (0.049, 0.126),
            (0.044, 0.139),
            (0.027, 0.158),
            (0.016, 0.179),
            (0.016, 0.224),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    bottle.visual(
        mesh_from_geometry(bottle_shell, "bottle_shell"),
        material=bottle_plastic,
        name="body_sections",
    )

    bottle.visual(
        Cylinder(radius=0.029, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.2015)),
        material=neck_plastic,
        name="neck_support_ring",
    )
    bottle.visual(
        mesh_from_geometry(
            _helical_thread_mesh(
                root_radius=0.0203,
                crest_radius=0.0234,
                z0=0.183,
                height=0.035,
                pitch=0.0115,
                thread_width=0.0028,
            ),
            "neck_thread",
        ),
        material=neck_plastic,
        name="neck_thread",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(
            _ribbed_cap_mesh(
                inner_radius=0.026,
                base_outer_radius=0.034,
                ridge_depth=0.0027,
                height=0.060,
                top_thickness=0.010,
            ),
            "ribbed_cap_shell",
        ),
        material=cap_plastic,
        name="cap_shell",
    )

    model.articulation(
        "neck_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.04),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    joint = object_model.get_articulation("neck_to_cap")

    ctx.check(
        "cap joint is continuous",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {joint.articulation_type}",
    )
    ctx.check(
        "cap joint axis follows bottle neck",
        tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={joint.axis}",
    )
    ctx.expect_contact(
        cap,
        bottle,
        elem_a="cap_shell",
        elem_b="neck_support_ring",
        contact_tol=0.001,
        name="cap rim seats on neck support ring",
    )

    rest_position = ctx.part_world_position(cap)
    with ctx.pose({joint: math.pi * 1.5}):
        turned_position = ctx.part_world_position(cap)
        ctx.expect_contact(
            cap,
            bottle,
            elem_a="cap_shell",
            elem_b="neck_support_ring",
            contact_tol=0.001,
            name="cap remains seated while rotating",
        )

    ctx.check(
        "cap rotation keeps coaxial origin",
        rest_position is not None
        and turned_position is not None
        and abs(rest_position[0] - turned_position[0]) < 1e-6
        and abs(rest_position[1] - turned_position[1]) < 1e-6
        and abs(rest_position[2] - turned_position[2]) < 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
