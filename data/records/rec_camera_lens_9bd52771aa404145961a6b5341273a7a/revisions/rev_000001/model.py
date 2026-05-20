from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_tube(
    inner_radius: float,
    outer_radius: float,
    length: float,
    *,
    segments: int = 96,
) -> MeshGeometry:
    """Watertight annular cylinder centered on the local Z axis."""

    geom = MeshGeometry()
    z0 = -length / 2.0
    z1 = length / 2.0
    inner0: list[int] = []
    inner1: list[int] = []
    outer0: list[int] = []
    outer1: list[int] = []

    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        c = math.cos(theta)
        s = math.sin(theta)
        inner0.append(geom.add_vertex(inner_radius * c, inner_radius * s, z0))
        inner1.append(geom.add_vertex(inner_radius * c, inner_radius * s, z1))
        outer0.append(geom.add_vertex(outer_radius * c, outer_radius * s, z0))
        outer1.append(geom.add_vertex(outer_radius * c, outer_radius * s, z1))

    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        geom.add_face(inner0[j], inner0[i], inner1[i])
        geom.add_face(inner0[j], inner1[i], inner1[j])
        geom.add_face(inner1[i], outer1[i], outer1[j])
        geom.add_face(inner1[i], outer1[j], inner1[j])
        geom.add_face(inner0[i], inner0[j], outer0[j])
        geom.add_face(inner0[i], outer0[j], outer0[i])

    return geom


def _ribbed_ring(
    inner_radius: float,
    outer_radius: float,
    length: float,
    *,
    rib_depth: float,
    rib_count: int,
    segments: int = 144,
) -> MeshGeometry:
    """Rubber grip ring with fine longitudinal ribs and a true central bore."""

    geom = MeshGeometry()
    z0 = -length / 2.0
    z1 = length / 2.0
    inner0: list[int] = []
    inner1: list[int] = []
    outer0: list[int] = []
    outer1: list[int] = []

    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        # Raised, rounded ridges rather than a perfectly smooth sleeve.
        crest = max(0.0, math.cos(rib_count * theta)) ** 2.6
        local_outer = outer_radius + rib_depth * crest
        c = math.cos(theta)
        s = math.sin(theta)
        inner0.append(geom.add_vertex(inner_radius * c, inner_radius * s, z0))
        inner1.append(geom.add_vertex(inner_radius * c, inner_radius * s, z1))
        outer0.append(geom.add_vertex(local_outer * c, local_outer * s, z0))
        outer1.append(geom.add_vertex(local_outer * c, local_outer * s, z1))

    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        geom.add_face(inner0[j], inner0[i], inner1[i])
        geom.add_face(inner0[j], inner1[i], inner1[j])
        geom.add_face(inner1[i], outer1[i], outer1[j])
        geom.add_face(inner1[i], outer1[j], inner1[j])
        geom.add_face(inner0[i], inner0[j], outer0[j])
        geom.add_face(inner0[i], outer0[j], outer0[i])

    return geom


def _biconvex_lens(
    radius: float,
    center_thickness: float,
    edge_thickness: float,
    *,
    radial_samples: int = 14,
    segments: int = 96,
) -> MeshGeometry:
    """Transparent optical element with curved front and rear faces."""

    geom = MeshGeometry()
    front_center = geom.add_vertex(0.0, 0.0, center_thickness / 2.0)
    back_center = geom.add_vertex(0.0, 0.0, -center_thickness / 2.0)
    front_rings: list[list[int]] = []
    back_rings: list[list[int]] = []

    for sample in range(1, radial_samples + 1):
        r = radius * sample / radial_samples
        curvature = 1.0 - (r / radius) ** 2
        half_t = edge_thickness / 2.0 + (center_thickness - edge_thickness) * curvature / 2.0
        front_loop: list[int] = []
        back_loop: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            front_loop.append(geom.add_vertex(x, y, half_t))
            back_loop.append(geom.add_vertex(x, y, -half_t))
        front_rings.append(front_loop)
        back_rings.append(back_loop)

    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(front_center, front_rings[0][i], front_rings[0][j])
        geom.add_face(back_center, back_rings[0][j], back_rings[0][i])

    for ring_index in range(radial_samples - 1):
        fr0 = front_rings[ring_index]
        fr1 = front_rings[ring_index + 1]
        br0 = back_rings[ring_index]
        br1 = back_rings[ring_index + 1]
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(fr0[i], fr1[i], fr1[j])
            geom.add_face(fr0[i], fr1[j], fr0[j])
            geom.add_face(br0[i], br0[j], br1[j])
            geom.add_face(br0[i], br1[j], br1[i])

    front_outer = front_rings[-1]
    back_outer = back_rings[-1]
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(back_outer[i], back_outer[j], front_outer[j])
        geom.add_face(back_outer[i], front_outer[j], front_outer[i])

    return geom


def _radial_box_origin(radius: float, angle_deg: float, z: float) -> Origin:
    theta = math.radians(angle_deg)
    return Origin(
        xyz=(radius * math.cos(theta), radius * math.sin(theta), z),
        rpy=(0.0, 0.0, theta),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_camera_lens")

    anodized = model.material("satin_black_anodized", rgba=(0.015, 0.015, 0.014, 1.0))
    black = model.material("deep_matte_black", rgba=(0.004, 0.004, 0.004, 1.0))
    rubber = model.material("ribbed_black_rubber", rgba=(0.010, 0.010, 0.009, 1.0))
    white = model.material("engraved_white_paint", rgba=(0.92, 0.90, 0.84, 1.0))
    gold = model.material("thin_gold_accent", rgba=(0.92, 0.62, 0.18, 1.0))
    red = model.material("red_mount_index", rgba=(0.90, 0.04, 0.02, 1.0))
    metal = model.material("brushed_steel_mount", rgba=(0.58, 0.57, 0.54, 1.0))
    glass = model.material("blue_green_coated_glass", rgba=(0.30, 0.62, 0.76, 0.46))
    dark_glass = model.material("deep_internal_glass", rgba=(0.02, 0.08, 0.11, 0.62))

    barrel = model.part("barrel")

    # Continuous hollow stepped shell: rear metal mount, low core under both
    # rotating rings, proud shoulders, and a broad front filter rim.
    barrel_shell = LatheGeometry.from_shell_profiles(
        [
            (0.034, -0.086),
            (0.036, -0.081),
            (0.036, -0.072),
            (0.040, -0.070),
            (0.040, -0.058),
            (0.036, -0.056),
            (0.036, -0.044),
            (0.029, -0.042),
            (0.029, 0.016),
            (0.042, 0.018),
            (0.042, 0.028),
            (0.029, 0.030),
            (0.029, 0.082),
            (0.042, 0.084),
            (0.042, 0.095),
            (0.039, 0.098),
            (0.039, 0.125),
            (0.045, 0.128),
            (0.045, 0.147),
            (0.041, 0.152),
        ],
        [
            (0.020, -0.086),
            (0.020, -0.050),
            (0.022, 0.020),
            (0.024, 0.085),
            (0.028, 0.120),
            (0.030, 0.152),
        ],
        segments=128,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )
    barrel.visual(
        mesh_from_geometry(barrel_shell, "barrel_shell"),
        material=anodized,
        name="barrel_shell",
    )

    # Individual named shoulders are slightly embedded in the main shell so the
    # rotating rings read as captured between hard mechanical stops.
    barrel.visual(
        mesh_from_geometry(_annular_tube(0.029, 0.0425, 0.004), "zoom_rear_shoulder"),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=anodized,
        name="zoom_rear_shoulder",
    )
    barrel.visual(
        mesh_from_geometry(_annular_tube(0.029, 0.0425, 0.004), "separator_shoulder"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=anodized,
        name="separator_shoulder",
    )
    barrel.visual(
        mesh_from_geometry(_annular_tube(0.029, 0.0425, 0.004), "focus_front_shoulder"),
        origin=Origin(xyz=(0.0, 0.0, 0.0845)),
        material=anodized,
        name="focus_front_shoulder",
    )

    barrel.visual(
        mesh_from_geometry(_annular_tube(0.041, 0.0433, 0.0032), "gold_accent_band"),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=gold,
        name="gold_accent_band",
    )
    barrel.visual(
        mesh_from_geometry(_annular_tube(0.026, 0.040, 0.005), "front_glass_retainer"),
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        material=black,
        name="front_glass_retainer",
    )
    barrel.visual(
        mesh_from_geometry(_annular_tube(0.015, 0.026, 0.003), "aperture_stop"),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=black,
        name="aperture_stop",
    )
    barrel.visual(
        mesh_from_geometry(_annular_tube(0.0172, 0.024, 0.004), "rear_glass_retainer"),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=black,
        name="rear_glass_retainer",
    )
    barrel.visual(
        mesh_from_geometry(_biconvex_lens(0.0275, 0.009, 0.0024), "front_element"),
        origin=Origin(xyz=(0.0, 0.0, 0.138)),
        material=glass,
        name="front_element",
    )
    barrel.visual(
        mesh_from_geometry(_biconvex_lens(0.0185, 0.007, 0.002), "rear_element"),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=dark_glass,
        name="rear_element",
    )

    # Three bayonet tabs and a red alignment dot make the rear read as a real
    # interchangeable-lens mount.
    for idx, angle in enumerate((18.0, 138.0, 258.0)):
        barrel.visual(
            Box((0.012, 0.022, 0.0035)),
            origin=_radial_box_origin(0.0405, angle, -0.084),
            material=metal,
            name=f"bayonet_tab_{idx}",
        )
    barrel.visual(
        Sphere(0.0022),
        origin=_radial_box_origin(0.0357, 78.0, -0.052),
        material=red,
        name="mount_index_dot",
    )

    # Stationary white witness line on the fixed barrel.
    barrel.visual(
        Box((0.0011, 0.0035, 0.008)),
        origin=_radial_box_origin(0.0430, 92.0, 0.023),
        material=white,
        name="barrel_index_line",
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        mesh_from_geometry(
            _ribbed_ring(0.0335, 0.0405, 0.058, rib_depth=0.0017, rib_count=42),
            "zoom_grip",
        ),
        material=rubber,
        name="zoom_grip",
    )
    for idx, angle in enumerate((-34.0, -18.0, 0.0, 19.0, 39.0)):
        zoom_ring.visual(
            Box((0.0030, 0.0030, 0.012 + 0.002 * (idx % 2))),
            origin=_radial_box_origin(0.0417, angle, 0.015),
            material=white,
            name=f"zoom_tick_{idx}",
        )
    zoom_ring.visual(
        Box((0.0030, 0.0042, 0.020)),
        origin=_radial_box_origin(0.0419, 0.0, -0.016),
        material=white,
        name="zoom_marker",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_geometry(
            _ribbed_ring(0.0335, 0.0398, 0.053, rib_depth=0.0012, rib_count=58),
            "focus_grip",
        ),
        material=rubber,
        name="focus_grip",
    )
    for idx, angle in enumerate((-52.0, -32.0, -12.0, 12.0, 34.0, 54.0)):
        focus_ring.visual(
            Box((0.0025, 0.0024, 0.010 + 0.002 * (idx == 2 or idx == 3))),
            origin=_radial_box_origin(0.0408, angle, -0.010),
            material=white,
            name=f"focus_tick_{idx}",
        )
    focus_ring.visual(
        Box((0.0025, 0.0040, 0.018)),
        origin=_radial_box_origin(0.0410, 0.0, 0.014),
        material=white,
        name="focus_marker",
    )

    model.articulation(
        "barrel_to_zoom_ring",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=zoom_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.9, velocity=3.0, lower=-0.05, upper=1.25),
    )
    model.articulation(
        "barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.55, velocity=4.0, lower=-1.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    focus_ring = object_model.get_part("focus_ring")
    zoom_joint = object_model.get_articulation("barrel_to_zoom_ring")
    focus_joint = object_model.get_articulation("barrel_to_focus_ring")

    ctx.expect_gap(
        barrel,
        zoom_ring,
        axis="z",
        positive_elem="separator_shoulder",
        negative_elem="zoom_grip",
        max_gap=0.001,
        max_penetration=0.0,
        name="zoom ring is captured by the front shoulder",
    )
    ctx.expect_gap(
        barrel,
        focus_ring,
        axis="z",
        positive_elem="focus_front_shoulder",
        negative_elem="focus_grip",
        max_gap=0.001,
        max_penetration=0.0,
        name="focus ring is captured by the front shoulder",
    )

    zoom_marker_rest = ctx.part_element_world_aabb(zoom_ring, elem="zoom_marker")
    focus_marker_rest = ctx.part_element_world_aabb(focus_ring, elem="focus_marker")
    with ctx.pose({zoom_joint: 1.0, focus_joint: -1.0}):
        zoom_marker_rotated = ctx.part_element_world_aabb(zoom_ring, elem="zoom_marker")
        focus_marker_rotated = ctx.part_element_world_aabb(focus_ring, elem="focus_marker")

    def _aabb_center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0)

    zr0 = _aabb_center_xy(zoom_marker_rest)
    zr1 = _aabb_center_xy(zoom_marker_rotated)
    fr0 = _aabb_center_xy(focus_marker_rest)
    fr1 = _aabb_center_xy(focus_marker_rotated)
    ctx.check(
        "zoom marker follows ring rotation",
        zr0 is not None
        and zr1 is not None
        and math.hypot(zr1[0] - zr0[0], zr1[1] - zr0[1]) > 0.025,
        details=f"rest={zr0}, rotated={zr1}",
    )
    ctx.check(
        "focus marker follows ring rotation",
        fr0 is not None
        and fr1 is not None
        and math.hypot(fr1[0] - fr0[0], fr1[1] - fr0[1]) > 0.025,
        details=f"rest={fr0}, rotated={fr1}",
    )

    return ctx.report()


object_model = build_object_model()
