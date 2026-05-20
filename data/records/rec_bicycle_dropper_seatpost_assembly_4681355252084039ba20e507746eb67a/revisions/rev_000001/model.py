from __future__ import annotations

from math import cos, pi, sin, tau

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
    tube_from_spline_points,
)


def _tube_shell_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 64,
) -> MeshGeometry:
    """Watertight open-bore cylindrical sleeve with annular end lips."""
    geom = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for index in range(segments):
        angle = tau * index / segments
        c = cos(angle)
        s = sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, 0.0))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, length))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, 0.0))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, length))

    for index in range(segments):
        nxt = (index + 1) % segments
        # Outside wall.
        geom.add_face(outer_bottom[index], outer_bottom[nxt], outer_top[nxt])
        geom.add_face(outer_bottom[index], outer_top[nxt], outer_top[index])
        # Inside wall.
        geom.add_face(inner_bottom[index], inner_top[index], inner_top[nxt])
        geom.add_face(inner_bottom[index], inner_top[nxt], inner_bottom[nxt])
        # Annular top and bottom lips.
        geom.add_face(outer_top[index], outer_top[nxt], inner_top[nxt])
        geom.add_face(outer_top[index], inner_top[nxt], inner_top[index])
        geom.add_face(outer_bottom[index], inner_bottom[index], inner_bottom[nxt])
        geom.add_face(outer_bottom[index], inner_bottom[nxt], outer_bottom[nxt])
    return geom


def _superellipse_section(
    x: float,
    *,
    half_width: float,
    thickness: float,
    center_z: float,
    segments: int = 36,
    exponent: float = 2.6,
) -> list[tuple[float, float, float]]:
    """A closed saddle cross-section in the YZ plane at a given X."""
    points: list[tuple[float, float, float]] = []
    for index in range(segments):
        angle = tau * index / segments
        ca = cos(angle)
        sa = sin(angle)
        y = half_width * (1.0 if ca >= 0 else -1.0) * abs(ca) ** (2.0 / exponent)
        z = center_z + (thickness * 0.5) * (1.0 if sa >= 0 else -1.0) * abs(sa) ** (2.0 / exponent)
        points.append((x, y, z))
    return points


def _saddle_pad_geometry() -> MeshGeometry:
    """Lofted road/MTB-style saddle: narrow nose, broad padded rear, slight rise."""
    sections = [
        _superellipse_section(-0.135, half_width=0.055, thickness=0.031, center_z=0.148),
        _superellipse_section(-0.095, half_width=0.071, thickness=0.037, center_z=0.153),
        _superellipse_section(-0.045, half_width=0.064, thickness=0.034, center_z=0.151),
        _superellipse_section(0.020, half_width=0.043, thickness=0.028, center_z=0.146),
        _superellipse_section(0.085, half_width=0.024, thickness=0.024, center_z=0.140),
        _superellipse_section(0.145, half_width=0.014, thickness=0.019, center_z=0.136),
    ]
    return section_loft(sections, cap=True, solid=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_travel_dropper_seatpost")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.038, 0.042, 1.0))
    anodized_grey = model.material("anodized_grey", rgba=(0.34, 0.36, 0.38, 1.0))
    polished_metal = model.material("polished_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    saddle_vinyl = model.material("saddle_vinyl", rgba=(0.020, 0.020, 0.022, 1.0))

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        mesh_from_geometry(
            _tube_shell_geometry(outer_radius=0.0175, inner_radius=0.0148, length=0.285),
            "outer_sleeve_shell",
        ),
        material=matte_black,
        name="outer_sleeve_shell",
    )
    outer_sleeve.visual(
        mesh_from_geometry(
            _tube_shell_geometry(outer_radius=0.0210, inner_radius=0.0132, length=0.026),
            "wiper_collar_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=satin_black,
        name="wiper_collar",
    )
    outer_sleeve.visual(
        Box((0.014, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.0215, 0.070)),
        material=satin_black,
        name="cable_stop_body",
    )
    outer_sleeve.visual(
        Cylinder(radius=0.0040, length=0.030),
        origin=Origin(xyz=(0.0, -0.033, 0.070), rpy=(pi / 2.0, 0.0, 0.0)),
        material=anodized_grey,
        name="cable_stop_nozzle",
    )
    for z in (0.105, 0.130, 0.155, 0.180):
        outer_sleeve.visual(
            Box((0.0016, 0.0055, 0.0018)),
            origin=Origin(xyz=(0.0179, 0.0, z)),
            material=anodized_grey,
            name=f"height_mark_{int(z * 1000)}",
        )

    inner_tube = model.part("inner_tube")
    inner_tube.visual(
        Cylinder(radius=0.0132, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, -0.0475)),
        material=anodized_grey,
        name="stanchion",
    )
    inner_tube.visual(
        Cylinder(radius=0.0160, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=satin_black,
        name="post_head",
    )
    inner_tube.visual(
        Box((0.066, 0.046, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=satin_black,
        name="lower_clamp_block",
    )
    inner_tube.visual(
        Box((0.092, 0.066, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=satin_black,
        name="rail_cradle",
    )
    inner_tube.visual(
        Cylinder(radius=0.0062, length=0.094),
        origin=Origin(xyz=(0.0, -0.026, 0.105), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="rail_seat_0",
    )
    inner_tube.visual(
        Cylinder(radius=0.0062, length=0.094),
        origin=Origin(xyz=(0.0, 0.026, 0.105), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="rail_seat_1",
    )
    inner_tube.visual(
        Box((0.025, 0.066, 0.012)),
        origin=Origin(xyz=(-0.033, 0.0, 0.121)),
        material=satin_black,
        name="bolt_bridge_0",
    )
    inner_tube.visual(
        Box((0.025, 0.066, 0.012)),
        origin=Origin(xyz=(0.033, 0.0, 0.121)),
        material=satin_black,
        name="bolt_bridge_1",
    )
    for x in (-0.033, 0.033):
        inner_tube.visual(
            Cylinder(radius=0.0042, length=0.050),
            origin=Origin(xyz=(x, 0.0, 0.108)),
            material=bolt_steel,
            name=f"clamp_bolt_{0 if x < 0 else 1}",
        )
        inner_tube.visual(
            Cylinder(radius=0.0085, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.136)),
            material=bolt_steel,
            name=f"bolt_head_{0 if x < 0 else 1}",
        )

    rail_path = [
        (-0.108, -0.026, 0.112),
        (-0.060, -0.026, 0.121),
        (0.035, -0.026, 0.120),
        (0.108, -0.026, 0.108),
        (0.118, 0.000, 0.107),
        (0.108, 0.026, 0.108),
        (0.035, 0.026, 0.120),
        (-0.060, 0.026, 0.121),
        (-0.108, 0.026, 0.112),
    ]
    inner_tube.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                rail_path,
                radius=0.0036,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            ),
            "saddle_rails",
        ),
        material=polished_metal,
        name="saddle_rails",
    )
    for x, y in ((-0.100, -0.026), (-0.100, 0.026), (0.100, -0.023), (0.100, 0.023)):
        inner_tube.visual(
            Cylinder(radius=0.0050, length=0.032),
            origin=Origin(xyz=(x, y, 0.124)),
            material=polished_metal,
            name=f"rail_anchor_{'rear' if x < 0 else 'front'}_{0 if y < 0 else 1}",
        )
    inner_tube.visual(
        mesh_from_geometry(_saddle_pad_geometry(), "curved_saddle_pad"),
        material=saddle_vinyl,
        name="saddle_pad",
    )
    inner_tube.visual(
        Box((0.116, 0.010, 0.0025)),
        origin=Origin(xyz=(-0.040, 0.0, 0.172)),
        material=satin_black,
        name="center_relief",
    )

    model.articulation(
        "sleeve_to_inner_tube",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=inner_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.35, lower=0.0, upper=0.060),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_sleeve = object_model.get_part("outer_sleeve")
    inner_tube = object_model.get_part("inner_tube")
    slide = object_model.get_articulation("sleeve_to_inner_tube")

    ctx.allow_overlap(
        inner_tube,
        outer_sleeve,
        elem_a="stanchion",
        elem_b="wiper_collar",
        reason="The visible top wiper seal is modeled as a solid annular proxy and intentionally compresses around the sliding stanchion.",
    )
    ctx.check(
        "dropper travel is 60 mm",
        slide.motion_limits is not None
        and abs((slide.motion_limits.upper or 0.0) - 0.060) < 1e-6
        and abs((slide.motion_limits.lower or 0.0) - 0.0) < 1e-6,
        details=f"limits={slide.motion_limits}",
    )
    ctx.expect_within(
        inner_tube,
        outer_sleeve,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="outer_sleeve_shell",
        margin=0.003,
        name="stanchion is centered inside the sleeve bore",
    )
    ctx.expect_within(
        inner_tube,
        outer_sleeve,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="wiper_collar",
        margin=0.004,
        name="stanchion passes through the wiper seal",
    )
    ctx.expect_overlap(
        inner_tube,
        outer_sleeve,
        axes="z",
        elem_a="stanchion",
        elem_b="outer_sleeve_shell",
        min_overlap=0.090,
        name="collapsed post keeps hidden insertion in the sleeve",
    )

    rest_position = ctx.part_world_position(inner_tube)
    with ctx.pose({slide: 0.060}):
        ctx.expect_within(
            inner_tube,
            outer_sleeve,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="outer_sleeve_shell",
            margin=0.003,
            name="extended stanchion remains centered in the sleeve bore",
        )
        ctx.expect_overlap(
            inner_tube,
            outer_sleeve,
            axes="z",
            elem_a="stanchion",
            elem_b="outer_sleeve_shell",
            min_overlap=0.080,
            name="extended post retains insertion in the sleeve",
        )
        extended_position = ctx.part_world_position(inner_tube)

    ctx.check(
        "upper pose raises the saddle assembly",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 0.055,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
