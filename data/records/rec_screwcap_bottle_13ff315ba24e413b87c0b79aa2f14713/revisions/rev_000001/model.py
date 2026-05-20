from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 96,
) -> MeshGeometry:
    """A clean annular band for collars, lips, and molded break lines."""
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _helical_thread(
    *,
    base_radius: float,
    crest_radius: float,
    z_min: float,
    z_max: float,
    width: float,
    turns: float,
    phase: float = 0.0,
    samples: int = 180,
) -> MeshGeometry:
    """Triangular helical ridge centered on the bottle/cap Z axis."""
    geom = MeshGeometry()
    for i in range(samples + 1):
        t = i / samples
        theta = phase + turns * math.tau * t
        z = z_min + (z_max - z_min) * t
        c = math.cos(theta)
        s = math.sin(theta)
        section = (
            (base_radius, z - 0.5 * width),
            (crest_radius, z),
            (base_radius, z + 0.5 * width),
        )
        for radius, zz in section:
            geom.add_vertex(radius * c, radius * s, zz)

    for i in range(samples):
        a = 3 * i
        b = 3 * (i + 1)
        # Three side strips around the triangular thread section.
        geom.add_face(a, b, a + 1)
        geom.add_face(a + 1, b, b + 1)
        geom.add_face(a + 1, b + 1, a + 2)
        geom.add_face(a + 2, b + 1, b + 2)
        geom.add_face(a + 2, b + 2, a)
        geom.add_face(a, b + 2, b)

    # Close the two thread starts so the ridge reads as a molded raised form.
    geom.add_face(0, 1, 2)
    end = 3 * samples
    geom.add_face(end, end + 2, end + 1)
    return geom


def _ribbed_sleeve(
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    ribs: int = 56,
    segments: int = 224,
    rib_depth: float = 0.0012,
) -> MeshGeometry:
    """Closed thin sleeve with fine vertical cap grip flutes."""
    geom = MeshGeometry()
    for i in range(segments):
        theta = math.tau * i / segments
        wave = 0.5 + 0.5 * math.cos(ribs * theta)
        # Raised flats with shallow troughs: restrained premium grip texture.
        outer_r = outer_radius - rib_depth * (1.0 - wave**2.4)
        c = math.cos(theta)
        s = math.sin(theta)
        for radius, z in (
            (inner_radius, z_min),
            (inner_radius, z_max),
            (outer_r, z_min),
            (outer_r, z_max),
        ):
            geom.add_vertex(radius * c, radius * s, z)

    for i in range(segments):
        j = (i + 1) % segments
        a = 4 * i
        b = 4 * j
        # Inner surface.
        geom.add_face(a, a + 1, b)
        geom.add_face(a + 1, b + 1, b)
        # Outer surface.
        geom.add_face(a + 2, b + 2, a + 3)
        geom.add_face(a + 3, b + 2, b + 3)
        # Bottom and top annular caps.
        geom.add_face(a, b, a + 2)
        geom.add_face(a + 2, b, b + 2)
        geom.add_face(a + 1, a + 3, b + 1)
        geom.add_face(a + 3, b + 3, b + 1)
    return geom


def _bottle_body_mesh() -> MeshGeometry:
    outer_profile = [
        (0.000, 0.006),
        (0.023, 0.004),
        (0.037, 0.008),
        (0.042, 0.018),
        (0.043, 0.050),
        (0.043, 0.128),
        (0.041, 0.148),
        (0.036, 0.164),
        (0.029, 0.176),
        (0.021, 0.185),
        (0.0164, 0.191),
        (0.0162, 0.216),
    ]
    inner_profile = [
        (0.000, 0.014),
        (0.026, 0.014),
        (0.036, 0.024),
        (0.038, 0.056),
        (0.038, 0.127),
        (0.036, 0.146),
        (0.031, 0.160),
        (0.024, 0.173),
        (0.0150, 0.187),
        (0.0112, 0.196),
        (0.0112, 0.216),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=128,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _cap_shell_mesh() -> MeshGeometry:
    outer_profile = [
        (0.0217, 0.000),
        (0.0230, 0.003),
        (0.0230, 0.038),
        (0.0222, 0.042),
        (0.0192, 0.046),
    ]
    inner_profile = [
        (0.0192, 0.004),
        (0.0192, 0.034),
        (0.0130, 0.040),
        (0.0000, 0.040),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=128,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_screwcap_bottle")

    frosted_glass = model.material("frosted_glass", rgba=(0.82, 0.88, 0.86, 0.54))
    satin_label = model.material("satin_label", rgba=(0.92, 0.90, 0.84, 0.44))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.68, 0.58, 1.0))
    neck_plastic = model.material("satin_neck", rgba=(0.78, 0.84, 0.82, 0.78))
    cap_matte = model.material("matte_cap", rgba=(0.035, 0.037, 0.041, 1.0))
    cap_satin = model.material("satin_cap_top", rgba=(0.12, 0.125, 0.13, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_bottle_body_mesh(), "body_shell"),
        material=frosted_glass,
        name="body_shell",
    )
    bottle.visual(
        mesh_from_geometry(
            _ring_band(outer_radius=0.0438, inner_radius=0.0426, z_min=0.055, z_max=0.136),
            "label_sleeve",
        ),
        material=satin_label,
        name="label_sleeve",
    )
    bottle.visual(
        mesh_from_geometry(
            _ring_band(outer_radius=0.0222, inner_radius=0.0144, z_min=0.182, z_max=0.187),
            "metal_collar",
        ),
        material=brushed_metal,
        name="metal_collar",
    )
    bottle.visual(
        mesh_from_geometry(
            _ring_band(outer_radius=0.0170, inner_radius=0.0114, z_min=0.188, z_max=0.193),
            "neck_land",
        ),
        material=neck_plastic,
        name="neck_land",
    )
    bottle.visual(
        mesh_from_geometry(
            _helical_thread(
                base_radius=0.0161,
                crest_radius=0.0180,
                z_min=0.193,
                z_max=0.212,
                width=0.0024,
                turns=2.15,
                phase=0.45,
            ),
            "neck_thread",
        ),
        material=neck_plastic,
        name="neck_thread",
    )
    bottle.visual(
        mesh_from_geometry(TorusGeometry(radius=0.0370, tube=0.0020, radial_segments=16, tubular_segments=96), "base_punt"),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=frosted_glass,
        name="base_punt",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.043, length=0.216),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_cap_shell_mesh(), "cap_shell"),
        material=cap_matte,
        name="cap_shell",
    )
    cap.visual(
        mesh_from_geometry(
            _ribbed_sleeve(inner_radius=0.02215, outer_radius=0.0240, z_min=0.005, z_max=0.037),
            "grip_sleeve",
        ),
        material=cap_matte,
        name="grip_sleeve",
    )
    cap.visual(
        mesh_from_geometry(
            _helical_thread(
                base_radius=0.0192,
                crest_radius=0.0178,
                z_min=0.006,
                z_max=0.025,
                width=0.0021,
                turns=2.15,
                phase=0.45,
            ),
            "cap_thread",
        ),
        material=cap_satin,
        name="cap_thread",
    )
    cap.visual(
        mesh_from_geometry(
            _ring_band(outer_radius=0.0182, inner_radius=0.0056, z_min=0.0448, z_max=0.0466),
            "top_insert",
        ),
        material=cap_satin,
        name="top_insert",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.047),
        mass=0.032,
        origin=Origin(xyz=(0.0, 0.0, 0.0235)),
    )

    model.articulation(
        "neck_to_cap",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=cap,
        # Child frame is the cap's bottom-center thread axis; it is coaxial with the bottle neck.
        origin=Origin(xyz=(0.0, 0.0, 0.1866)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.4, velocity=4.5, lower=0.0, upper=math.tau * 1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    joint = object_model.get_articulation("neck_to_cap")

    ctx.allow_overlap(
        bottle,
        cap,
        elem_a="metal_collar",
        elem_b="cap_shell",
        reason="The cap skirt is intentionally seated with a tiny compressed seam on the metal collar.",
    )
    ctx.expect_origin_distance(
        bottle,
        cap,
        axes="xy",
        max_dist=0.0005,
        name="cap rotation is coaxial with bottle neck",
    )
    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="neck_thread",
        outer_elem="cap_shell",
        margin=0.0,
        name="neck thread sits inside the cap diameter",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        elem_a="cap_thread",
        elem_b="neck_thread",
        min_overlap=0.014,
        name="cap and neck threads share practical engagement depth",
    )
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="metal_collar",
        max_gap=0.0002,
        max_penetration=0.0008,
        name="cap skirt seats tightly on collar seam",
    )

    rest = ctx.part_world_position(cap)
    with ctx.pose({joint: math.tau * 1.25}):
        rotated = ctx.part_world_position(cap)
        ctx.expect_origin_distance(
            bottle,
            cap,
            axes="xy",
            max_dist=0.0005,
            name="cap remains centered during threaded rotation",
        )
    ctx.check(
        "threaded rotation does not translate the coaxial cap frame",
        rest is not None
        and rotated is not None
        and abs(rest[0] - rotated[0]) < 0.0005
        and abs(rest[1] - rotated[1]) < 0.0005
        and abs(rest[2] - rotated[2]) < 0.0005,
        details=f"rest={rest}, rotated={rotated}",
    )

    return ctx.report()


object_model = build_object_model()
