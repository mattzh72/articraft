from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


HALF_PI = math.pi / 2.0


def _annular_shell(
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    *,
    segments: int = 72,
):
    """Thin lathed tube/ring with a true through-bore."""
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _saddle_pad_geometry() -> MeshGeometry:
    """Low, firm dropper saddle: wide rear, short rounded nose, slight crown."""
    geom = MeshGeometry()
    # Longitudinal stations from rear (-X) to short nose (+X).
    xs = [-0.108, -0.090, -0.060, -0.025, 0.015, 0.055, 0.095, 0.130, 0.158]
    half_widths = [0.058, 0.064, 0.061, 0.052, 0.043, 0.034, 0.026, 0.018, 0.006]
    bottom_z = 0.048
    thickness = [0.024, 0.026, 0.027, 0.028, 0.027, 0.025, 0.023, 0.021, 0.017]
    side_samples = 12

    top_rows: list[list[int]] = []
    bottom_rows: list[list[int]] = []
    for i, (x, hw, th) in enumerate(zip(xs, half_widths, thickness)):
        top_row: list[int] = []
        bottom_row: list[int] = []
        # Elliptical cross-section: more points near the sides, crowned center.
        for j in range(side_samples + 1):
            t = -1.0 + 2.0 * j / side_samples
            y = hw * math.sin(t * HALF_PI)
            crown = 0.0045 * (1.0 - abs(t) ** 1.7)
            length_crown = 0.002 * math.sin(math.pi * i / (len(xs) - 1))
            top_z = bottom_z + th + crown + length_crown
            top_row.append(geom.add_vertex(x, y, top_z))
            bottom_row.append(geom.add_vertex(x, y, bottom_z))
        top_rows.append(top_row)
        bottom_rows.append(bottom_row)

    # Top and bottom skins.
    for i in range(len(xs) - 1):
        for j in range(side_samples):
            a = top_rows[i][j]
            b = top_rows[i + 1][j]
            c = top_rows[i + 1][j + 1]
            d = top_rows[i][j + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

            a = bottom_rows[i][j]
            b = bottom_rows[i][j + 1]
            c = bottom_rows[i + 1][j + 1]
            d = bottom_rows[i + 1][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    # Side walls.
    for i in range(len(xs) - 1):
        for j in (0, side_samples):
            a = top_rows[i][j]
            b = bottom_rows[i][j]
            c = bottom_rows[i + 1][j]
            d = top_rows[i + 1][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    # Rear and nose caps.
    for row_i in (0, len(xs) - 1):
        for j in range(side_samples):
            a = top_rows[row_i][j]
            b = top_rows[row_i][j + 1]
            c = bottom_rows[row_i][j + 1]
            d = bottom_rows[row_i][j]
            if row_i == 0:
                geom.add_face(a, b, c)
                geom.add_face(a, c, d)
            else:
                geom.add_face(a, c, b)
                geom.add_face(a, d, c)

    return geom


def _saddle_rail_geometry() -> MeshGeometry:
    """One connected metal rail assembly with cross ties at the embedded ends."""
    rail = MeshGeometry()
    for y in (-0.026, 0.026):
        rail.merge(
            tube_from_spline_points(
                [
                    (-0.092, y, 0.045),
                    (-0.072, y, 0.035),
                    (-0.030, y, 0.031),
                    (0.045, y, 0.031),
                    (0.100, y, 0.037),
                    (0.120, y, 0.045),
                ],
                radius=0.0030,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            )
        )
    for x, z in [(-0.092, 0.045), (0.120, 0.045)]:
        rail.merge(
            tube_from_spline_points(
                [(x, -0.026, z), (x, 0.026, z)],
                radius=0.0030,
                samples_per_segment=2,
                radial_segments=16,
                cap_ends=True,
            )
        )
    return rail


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="internally_routed_dropper_seatpost")

    black = Material("matte_black_anodized", rgba=(0.005, 0.006, 0.006, 1.0))
    collar_black = Material("satin_black_collar", rgba=(0.025, 0.025, 0.022, 1.0))
    polished = Material("brushed_inner_tube", rgba=(0.74, 0.76, 0.72, 1.0))
    steel = Material("dark_bolt_steel", rgba=(0.34, 0.34, 0.32, 1.0))
    rubber = Material("firm_black_padding", rgba=(0.018, 0.018, 0.017, 1.0))
    cable_black = Material("black_cable_housing", rgba=(0.0, 0.0, 0.0, 1.0))

    lower_sleeve = model.part("lower_sleeve")
    lower_sleeve.visual(
        mesh_from_geometry(_annular_shell(0.0175, 0.0145, 0.000, 0.260), "sleeve_shell"),
        material=black,
        name="sleeve_shell",
    )
    lower_sleeve.visual(
        mesh_from_geometry(_annular_shell(0.0220, 0.0172, 0.000, 0.060), "clamp_sleeve"),
        material=black,
        name="clamp_sleeve",
    )
    lower_sleeve.visual(
        mesh_from_geometry(_annular_shell(0.0222, 0.0172, 0.225, 0.267), "guide_collar"),
        material=collar_black,
        name="guide_collar",
    )
    # Split clamp ears and pinch bolt on the lower sleeve.
    for y, idx in [(-0.008, 0), (0.008, 1)]:
        lower_sleeve.visual(
            Box((0.016, 0.008, 0.040)),
            origin=Origin(xyz=(0.029, y, 0.030)),
            material=black,
            name=f"clamp_ear_{idx}",
        )
    lower_sleeve.visual(
        Cylinder(radius=0.0033, length=0.044),
        origin=Origin(xyz=(0.033, 0.0, 0.030), rpy=(-HALF_PI, 0.0, 0.0)),
        material=steel,
        name="pinch_bolt",
    )
    lower_sleeve.visual(
        Box((0.0015, 0.006, 0.048)),
        origin=Origin(xyz=(0.0227, 0.0, 0.030)),
        material=cable_black,
        name="clamp_split",
    )
    # Base pivot bracket and visible internal cable entry.
    for y, idx in [(-0.015, 0), (0.015, 1)]:
        lower_sleeve.visual(
            Box((0.010, 0.006, 0.034)),
            origin=Origin(xyz=(-0.024, y, 0.074)),
            material=black,
            name=f"pivot_tab_{idx}",
        )
        lower_sleeve.visual(
            Box((0.020, 0.008, 0.006)),
            origin=Origin(xyz=(-0.019, y, 0.055)),
            material=black,
            name=f"pivot_bridge_{idx}",
        )
    lower_sleeve.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(-0.062, 0.0, 0.065), (-0.040, 0.0, 0.070), (-0.018, 0.0, 0.078), (0.000, 0.0, 0.105)],
                radius=0.0021,
                samples_per_segment=10,
                radial_segments=12,
                cap_ends=True,
            ),
            "internal_cable",
        ),
        material=cable_black,
        name="internal_cable",
    )

    cable_pivot = model.part("cable_pivot")
    cable_pivot.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(rpy=(-HALF_PI, 0.0, 0.0)),
        material=steel,
        name="pivot_wheel",
    )
    cable_pivot.visual(
        Cylinder(radius=0.0030, length=0.030),
        origin=Origin(rpy=(-HALF_PI, 0.0, 0.0)),
        material=steel,
        name="pivot_axle",
    )
    cable_pivot.visual(
        Box((0.020, 0.004, 0.006)),
        origin=Origin(xyz=(-0.008, 0.0, -0.006), rpy=(0.0, 0.18, 0.0)),
        material=steel,
        name="cable_cam",
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0145, length=0.292),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=polished,
        name="inner_tube",
    )
    for z, idx in [(0.090, 0), (0.127, 1)]:
        inner_post.visual(
            Cylinder(radius=0.0133, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=collar_black,
            name=f"travel_mark_{idx}",
        )
    inner_post.visual(
        Cylinder(radius=0.0168, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=collar_black,
        name="limit_collar",
    )

    cartridge_clamp = model.part("cartridge_clamp")
    cartridge_clamp.visual(
        Cylinder(radius=0.013, length=0.080),
        origin=Origin(rpy=(-HALF_PI, 0.0, 0.0)),
        material=black,
        name="cartridge_barrel",
    )
    cartridge_clamp.visual(
        Cylinder(radius=0.011, length=0.033),
        origin=Origin(xyz=(0.0, 0.0, -0.0265)),
        material=black,
        name="neck",
    )
    cartridge_clamp.visual(
        Box((0.030, 0.060, 0.013)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=black,
        name="yoke_block",
    )

    side_bolt_0 = model.part("side_bolt_0")
    side_bolt_0.visual(
        Cylinder(radius=0.0032, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-HALF_PI, 0.0, 0.0)),
        material=steel,
        name="bolt_shank",
    )
    side_bolt_0.visual(
        Cylinder(radius=0.0070, length=0.0045),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-HALF_PI, 0.0, 0.0)),
        material=steel,
        name="bolt_head",
    )
    side_bolt_1 = model.part("side_bolt_1")
    side_bolt_1.visual(
        Cylinder(radius=0.0032, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(-HALF_PI, 0.0, 0.0)),
        material=steel,
        name="bolt_shank",
    )
    side_bolt_1.visual(
        Cylinder(radius=0.0070, length=0.0045),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(-HALF_PI, 0.0, 0.0)),
        material=steel,
        name="bolt_head",
    )

    rail_cradle = model.part("rail_cradle")
    rail_cradle.visual(
        Box((0.118, 0.070, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, 0.017)),
        material=black,
        name="lower_cradle",
    )
    for x, idx in [(-0.045, 0), (0.055, 1)]:
        rail_cradle.visual(
            Box((0.030, 0.072, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.040)),
            material=black,
            name=f"rail_cap_{idx}",
        )
        rail_cradle.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(xyz=(x, 0.0, 0.030)),
            material=black,
            name=f"bolt_boss_{idx}",
        )

    cradle_bolt_0 = model.part("cradle_bolt_0")
    cradle_bolt_1 = model.part("cradle_bolt_1")
    for bolt in (cradle_bolt_0, cradle_bolt_1):
        bolt.visual(
            Cylinder(radius=0.0068, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=steel,
            name="bolt_head",
        )
        bolt.visual(
            Cylinder(radius=0.0042, length=0.0015),
            origin=Origin(xyz=(0.0, 0.0, 0.0062)),
            material=collar_black,
            name="hex_recess",
        )

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_geometry(_saddle_pad_geometry(), "saddle_pad"),
        material=rubber,
        name="saddle_pad",
    )
    saddle.visual(
        mesh_from_geometry(_saddle_rail_geometry(), "saddle_rails"),
        material=steel,
        name="saddle_rails",
    )

    model.articulation(
        "sleeve_to_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_sleeve,
        child=cable_pivot,
        origin=Origin(xyz=(-0.026, 0.0, 0.074)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-0.35, upper=0.45),
    )
    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=lower_sleeve,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.6, lower=0.0, upper=0.100),
    )
    model.articulation(
        "post_to_cartridge",
        ArticulationType.FIXED,
        parent=inner_post,
        child=cartridge_clamp,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
    )
    model.articulation(
        "cartridge_to_cradle",
        ArticulationType.REVOLUTE,
        parent=cartridge_clamp,
        child=rail_cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.22, upper=0.22),
    )
    model.articulation(
        "cradle_to_saddle",
        ArticulationType.FIXED,
        parent=rail_cradle,
        child=saddle,
        origin=Origin(),
    )
    model.articulation(
        "cartridge_to_side_bolt_0",
        ArticulationType.REVOLUTE,
        parent=cartridge_clamp,
        child=side_bolt_0,
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "cartridge_to_side_bolt_1",
        ArticulationType.REVOLUTE,
        parent=cartridge_clamp,
        child=side_bolt_1,
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "cradle_to_bolt_0",
        ArticulationType.REVOLUTE,
        parent=rail_cradle,
        child=cradle_bolt_0,
        origin=Origin(xyz=(-0.045, 0.0, 0.043)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "cradle_to_bolt_1",
        ArticulationType.REVOLUTE,
        parent=rail_cradle,
        child=cradle_bolt_1,
        origin=Origin(xyz=(0.055, 0.0, 0.043)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_sleeve = object_model.get_part("lower_sleeve")
    cable_pivot = object_model.get_part("cable_pivot")
    inner_post = object_model.get_part("inner_post")
    cartridge = object_model.get_part("cartridge_clamp")
    rail_cradle = object_model.get_part("rail_cradle")
    saddle = object_model.get_part("saddle")
    post_slide = object_model.get_articulation("sleeve_to_post")
    cradle_tilt = object_model.get_articulation("cartridge_to_cradle")

    ctx.allow_overlap(
        lower_sleeve,
        inner_post,
        elem_a="sleeve_shell",
        elem_b="inner_tube",
        reason="The polished inner post is intentionally modeled as a close sliding member inside the lower sleeve bore.",
    )

    # The pivot axle is intentionally captured by the split base tabs.
    for idx in (0, 1):
        ctx.allow_overlap(
            lower_sleeve,
            cable_pivot,
            elem_a=f"pivot_tab_{idx}",
            elem_b="pivot_axle",
            reason="The base cable-pivot axle is intentionally seated through the clamp sleeve yoke tab.",
        )
        ctx.expect_overlap(
            lower_sleeve,
            cable_pivot,
            axes="y",
            elem_a=f"pivot_tab_{idx}",
            elem_b="pivot_axle",
            min_overlap=0.0005,
            name=f"pivot axle captured in tab {idx}",
        )

    ctx.expect_within(
        inner_post,
        lower_sleeve,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="guide_collar",
        margin=0.002,
        name="telescoping post stays centered through guide collar",
    )
    ctx.expect_overlap(
        inner_post,
        lower_sleeve,
        axes="z",
        elem_a="inner_tube",
        elem_b="guide_collar",
        min_overlap=0.020,
        name="collapsed inner post remains guided by top collar",
    )

    rest_pos = ctx.part_world_position(inner_post)
    with ctx.pose({post_slide: 0.100}):
        ctx.expect_within(
            inner_post,
            lower_sleeve,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="guide_collar",
            margin=0.002,
            name="extended post stays centered through guide collar",
        )
        ctx.expect_overlap(
            inner_post,
            lower_sleeve,
            axes="z",
            elem_a="inner_tube",
            elem_b="guide_collar",
            min_overlap=0.020,
            name="extended post retains insertion at travel limit",
        )
        extended_pos = ctx.part_world_position(inner_post)

    ctx.check(
        "dropper travel extends upward 100 mm",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.095,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.expect_gap(
        cartridge,
        inner_post,
        axis="z",
        positive_elem="neck",
        negative_elem="inner_tube",
        max_gap=0.001,
        max_penetration=0.0005,
        name="cartridge neck seated on inner post",
    )
    ctx.expect_overlap(
        saddle,
        rail_cradle,
        axes="xy",
        elem_a="saddle_rails",
        elem_b="lower_cradle",
        min_overlap=0.040,
        name="saddle rails pass through the two-bolt cradle footprint",
    )
    with ctx.pose({cradle_tilt: 0.18}):
        ctx.expect_gap(
            saddle,
            cartridge,
            axis="z",
            positive_elem="saddle_pad",
            negative_elem="cartridge_barrel",
            min_gap=0.004,
            name="tilted saddle remains above cartridge clamp",
        )

    return ctx.report()


object_model = build_object_model()
