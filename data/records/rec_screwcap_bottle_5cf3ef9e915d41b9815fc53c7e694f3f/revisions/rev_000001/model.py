from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _helical_rib(
    *,
    radius_inner: float,
    radius_outer: float,
    z_start: float,
    z_end: float,
    turns: float,
    width: float,
    segments: int,
    phase: float = 0.0,
) -> MeshGeometry:
    """Raised rectangular-section helical rib wrapped around the bottle neck."""

    geom = MeshGeometry()
    rings: list[tuple[int, int, int, int]] = []
    for i in range(segments + 1):
        t = i / segments
        theta = phase + (turns * 2.0 * math.pi * t)
        z = z_start + (z_end - z_start) * t
        c = math.cos(theta)
        s = math.sin(theta)
        rings.append(
            (
                geom.add_vertex(radius_inner * c, radius_inner * s, z - width / 2.0),
                geom.add_vertex(radius_outer * c, radius_outer * s, z - width / 2.0),
                geom.add_vertex(radius_outer * c, radius_outer * s, z + width / 2.0),
                geom.add_vertex(radius_inner * c, radius_inner * s, z + width / 2.0),
            )
        )

    for a, b in zip(rings[:-1], rings[1:]):
        # inner face seated into the neck wall
        geom.add_face(a[0], b[0], b[3])
        geom.add_face(a[0], b[3], a[3])
        # outer crest
        geom.add_face(a[1], a[2], b[2])
        geom.add_face(a[1], b[2], b[1])
        # lower and upper flanks
        geom.add_face(a[0], a[1], b[1])
        geom.add_face(a[0], b[1], b[0])
        geom.add_face(a[3], b[3], b[2])
        geom.add_face(a[3], b[2], a[2])

    # End caps keep the rib a closed, mechanically attached solid.
    start = rings[0]
    end = rings[-1]
    geom.add_face(start[0], start[3], start[2])
    geom.add_face(start[0], start[2], start[1])
    geom.add_face(end[0], end[1], end[2])
    geom.add_face(end[0], end[2], end[3])
    return geom


def _ribbed_cap_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    top_thickness: float,
    rib_count: int,
    rib_depth: float,
    segments: int = 192,
) -> MeshGeometry:
    """Hollow, open-bottom screw cap with a shallow broad ribbed skirt."""

    geom = MeshGeometry()
    z_bottom = -height / 2.0
    z_top = height / 2.0
    z_cavity_top = z_top - top_thickness

    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        c = math.cos(theta)
        s = math.sin(theta)
        # Rounded rectangular ribs: mostly broad high lands with slight valleys.
        rib_wave = 0.5 + 0.5 * math.cos(rib_count * theta)
        r_outer = outer_radius + rib_depth * (rib_wave**3)
        outer_bottom.append(geom.add_vertex(r_outer * c, r_outer * s, z_bottom))
        outer_top.append(geom.add_vertex(r_outer * c, r_outer * s, z_top))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_bottom))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_cavity_top))

    top_center = geom.add_vertex(0.0, 0.0, z_top)
    underside_center = geom.add_vertex(0.0, 0.0, z_cavity_top)
    for i in range(segments):
        j = (i + 1) % segments
        # Wavy outer skirt.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        # Inner cylindrical wall of the open cavity.
        geom.add_face(inner_bottom[i], inner_top[j], inner_bottom[j])
        geom.add_face(inner_bottom[i], inner_top[i], inner_top[j])
        # Open-bottom annular rim.
        geom.add_face(inner_bottom[i], inner_bottom[j], outer_bottom[j])
        geom.add_face(inner_bottom[i], outer_bottom[j], outer_bottom[i])
        # Top disk.
        geom.add_face(top_center, outer_top[i], outer_top[j])
        # Underside of the solid cap top, closing the cavity.
        geom.add_face(underside_center, inner_top[j], inner_top[i])
        # Annular vertical shoulder from inner cavity ceiling to the top skin.
        geom.add_face(inner_top[i], outer_top[i], outer_top[j])
        geom.add_face(inner_top[i], outer_top[j], inner_top[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_plastic = model.material("slightly_blue_clear_plastic", rgba=(0.68, 0.88, 1.0, 0.38))
    cap_plastic = model.material("matte_white_cap_plastic", rgba=(0.93, 0.93, 0.88, 1.0))
    label_paper = model.material("satin_paper_label", rgba=(0.93, 0.96, 0.90, 1.0))
    label_ink = model.material("green_label_stripe", rgba=(0.10, 0.45, 0.25, 1.0))

    body = model.part("body")

    # Thin-walled, open-mouth bottle shell: rounded base, cylindrical sides,
    # sloped shoulders, and a narrow finish for the screw cap.
    bottle_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.028, 0.000),
            (0.041, 0.004),
            (0.047, 0.018),
            (0.047, 0.218),
            (0.043, 0.237),
            (0.030, 0.256),
            (0.016, 0.272),
            (0.015, 0.306),
        ],
        inner_profile=[
            (0.024, 0.012),
            (0.038, 0.020),
            (0.041, 0.214),
            (0.037, 0.232),
            (0.023, 0.254),
            (0.010, 0.274),
            (0.010, 0.306),
        ],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    body.visual(
        mesh_from_geometry(bottle_shell, "bottle_shell"),
        origin=Origin(),
        material=bottle_plastic,
        name="bottle_shell",
    )

    label_band = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.0482, 0.102), (0.0482, 0.171)],
        inner_profile=[(0.0470, 0.102), (0.0470, 0.171)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(label_band, "label_band"),
        origin=Origin(),
        material=label_paper,
        name="label_band",
    )
    label_stripe = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.0488, 0.127), (0.0488, 0.142)],
        inner_profile=[(0.0481, 0.127), (0.0481, 0.142)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(label_stripe, "label_stripe"),
        origin=Origin(),
        material=label_ink,
        name="label_stripe",
    )

    neck_threads = _helical_rib(
        radius_inner=0.0148,
        radius_outer=0.0180,
        z_start=0.276,
        z_end=0.291,
        turns=1.65,
        width=0.0032,
        segments=120,
    )
    body.visual(
        mesh_from_geometry(neck_threads, "neck_threads"),
        origin=Origin(),
        material=bottle_plastic,
        name="neck_threads",
    )
    tamper_bead = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.0192, 0.266), (0.0192, 0.271)],
        inner_profile=[(0.0148, 0.266), (0.0148, 0.271)],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=4,
    )
    body.visual(
        mesh_from_geometry(tamper_bead, "tamper_bead"),
        origin=Origin(),
        material=bottle_plastic,
        name="tamper_bead",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(
            _ribbed_cap_geometry(
                outer_radius=0.030,
                inner_radius=0.0180,
                height=0.026,
                top_thickness=0.005,
                rib_count=44,
                rib_depth=0.0022,
            ),
            "cap_shell",
        ),
        origin=Origin(),
        material=cap_plastic,
        name="cap_shell",
    )

    model.articulation(
        "neck_to_cap",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=12.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    joint = object_model.get_articulation("neck_to_cap")

    ctx.check(
        "cap uses a continuous screw rotation",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    cap_aabb = ctx.part_world_aabb(cap)
    if cap_aabb is None:
        ctx.fail("cap dimensions available", "cap AABB could not be computed")
    else:
        lo, hi = cap_aabb
        cap_width = max(hi[0] - lo[0], hi[1] - lo[1])
        cap_height = hi[2] - lo[2]
        ctx.check(
            "cap is broad and shallow",
            cap_width > 0.055 and cap_height < 0.030 and cap_width > 2.0 * cap_height,
            details=f"width={cap_width:.4f}, height={cap_height:.4f}",
        )

    ctx.expect_within(
        body,
        cap,
        axes="xy",
        inner_elem="neck_threads",
        outer_elem="cap_shell",
        margin=0.003,
        name="threaded neck sits under the cap footprint",
    )
    ctx.expect_overlap(
        cap,
        body,
        axes="z",
        elem_a="cap_shell",
        elem_b="neck_threads",
        min_overlap=0.0005,
        name="cap skirt engages the upper neck thread",
    )

    rest_position = ctx.part_world_position(cap)
    with ctx.pose({joint: math.pi * 1.25}):
        turned_position = ctx.part_world_position(cap)
        ctx.expect_within(
            body,
            cap,
            axes="xy",
            inner_elem="neck_threads",
            outer_elem="cap_shell",
            margin=0.003,
            name="cap remains coaxial after screw rotation",
        )
    ctx.check(
        "rotation does not translate cap",
        rest_position is not None
        and turned_position is not None
        and max(abs(rest_position[i] - turned_position[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
