from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _bell_shell_mesh():
    """A thin, hollow, flared bronze bell made as a revolved shell."""
    outer = [
        (0.245, -0.670),
        (0.235, -0.620),
        (0.205, -0.520),
        (0.160, -0.380),
        (0.115, -0.250),
        (0.078, -0.160),
    ]
    inner = [
        (0.195, -0.640),
        (0.185, -0.575),
        (0.155, -0.455),
        (0.112, -0.330),
        (0.070, -0.205),
        (0.036, -0.150),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=96,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )


def _yoke_arch_mesh():
    """A continuous arched yoke strap that joins the axle to the bell crown."""
    return tube_from_spline_points(
        [
            (-0.095, 0.0, -0.245),
            (-0.070, 0.0, -0.180),
            (0.000, 0.0, -0.135),
            (0.070, 0.0, -0.180),
            (0.095, 0.0, -0.245),
        ],
        radius=0.016,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def _beam_origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Return an Origin and length for a rectangular beam whose local +Z follows start->end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # These bell-cote timbers lie in the XZ plane, so a simple Y rotation keeps
    # their rectangular section square to the front elevation.
    pitch = math.atan2(dx, dz)
    return Origin(xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0), rpy=(0.0, pitch, 0.0)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_frame_bell_cote")

    timber = Material("weathered_oak", rgba=(0.50, 0.31, 0.15, 1.0))
    dark_endgrain = Material("dark_endgrain", rgba=(0.20, 0.11, 0.05, 1.0))
    iron = Material("blackened_iron", rgba=(0.035, 0.035, 0.033, 1.0))
    bronze = Material("aged_bronze", rgba=(0.70, 0.43, 0.16, 1.0))
    clapper_metal = Material("dark_bronze", rgba=(0.33, 0.22, 0.11, 1.0))

    frame = model.part("timber_frame")
    frame.visual(
        Box((1.90, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=timber,
        name="base_sill",
    )
    frame.visual(
        Box((1.48, 0.14, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=timber,
        name="lower_tie",
    )
    for name, start, end in (
        ("raked_post_0", (-0.86, -0.12, 0.105), (0.0, -0.12, 1.325)),
        ("raked_post_1", (0.86, -0.12, 0.105), (0.0, -0.12, 1.325)),
    ):
        origin, length = _beam_origin_between(start, end)
        frame.visual(
            Box((0.115, 0.145, length)),
            origin=origin,
            material=timber,
            name=name,
        )
    frame.visual(
        Box((0.18, 0.86, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 1.340)),
        material=timber,
        name="ridge_beam",
    )
    # Small dark end-grain caps and wedged joints emphasize timber-frame joinery.
    frame.visual(
        Box((0.22, 0.16, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.410)),
        material=dark_endgrain,
        name="apex_cap",
    )
    for y in (-0.30, 0.30):
        frame.visual(
            Box((0.025, 0.050, 0.135)),
            origin=Origin(xyz=(-0.058, y, 1.225)),
            material=iron,
            name=f"bearing_cheek_{'rear' if y < 0 else 'front'}_0",
        )
        frame.visual(
            Box((0.025, 0.050, 0.135)),
            origin=Origin(xyz=(0.058, y, 1.225)),
            material=iron,
            name=f"bearing_cheek_{'rear' if y < 0 else 'front'}_1",
        )
        frame.visual(
            Box((0.128, 0.050, 0.032)),
            origin=Origin(xyz=(0.0, y, 1.174)),
            material=iron,
            name=f"bearing_saddle_{'rear' if y < 0 else 'front'}",
        )

    bell_yoke = model.part("bell_yoke")
    bell_yoke.visual(
        Cylinder(radius=0.035, length=0.720),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="pivot_rod",
    )
    bell_yoke.visual(
        mesh_from_geometry(_yoke_arch_mesh(), "arched_yoke"),
        material=iron,
        name="arched_yoke",
    )
    bell_yoke.visual(
        mesh_from_geometry(_bell_shell_mesh(), "bell_shell"),
        material=bronze,
        name="bell_shell",
    )
    for x in (-0.045, 0.045):
        bell_yoke.visual(
            Box((0.030, 0.028, 0.215)),
            origin=Origin(xyz=(x, 0.0, -0.105)),
            material=iron,
            name=f"yoke_strap_{0 if x < 0 else 1}",
        )
    bell_yoke.visual(
        Cylinder(radius=0.080, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.160)),
        material=bronze,
        name="crown_cap",
    )
    bell_yoke.visual(
        Box((0.060, 0.200, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.190)),
        material=iron,
        name="clapper_crossbar",
    )
    for name, y in (("clapper_hanger_0", -0.085), ("clapper_hanger_1", 0.085)):
        bell_yoke.visual(
            Box((0.026, 0.014, 0.130)),
            origin=Origin(xyz=(0.0, y, -0.245)),
            material=iron,
            name=name,
        )

    clapper = model.part("clapper")
    clapper.visual(
        Cylinder(radius=0.012, length=0.166),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clapper_metal,
        name="clapper_pin",
    )
    clapper.visual(
        Cylinder(radius=0.010, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        material=clapper_metal,
        name="clapper_rod",
    )
    clapper.visual(
        Sphere(radius=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.360)),
        material=clapper_metal,
        name="clapper_ball",
    )

    model.articulation(
        "frame_to_bell",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell_yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.220)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.6, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "bell_to_clapper",
        ArticulationType.REVOLUTE,
        parent=bell_yoke,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-0.75, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("timber_frame")
    bell = object_model.get_part("bell_yoke")
    clapper = object_model.get_part("clapper")
    bell_joint = object_model.get_articulation("frame_to_bell")
    clapper_joint = object_model.get_articulation("bell_to_clapper")

    for saddle in ("bearing_saddle_rear", "bearing_saddle_front"):
        ctx.allow_overlap(
            bell,
            frame,
            elem_a="pivot_rod",
            elem_b=saddle,
            reason="The rotating bell axle is intentionally captured in the iron bearing saddle.",
        )
        ctx.expect_gap(
            bell,
            frame,
            axis="z",
            positive_elem="pivot_rod",
            negative_elem=saddle,
            max_penetration=0.008,
            max_gap=0.002,
            name=f"pivot rod is seated in {saddle}",
        )

    for post in ("raked_post_0", "raked_post_1"):
        ctx.allow_overlap(
            bell,
            frame,
            elem_a="pivot_rod",
            elem_b=post,
            reason="The bell axle is intentionally shown passing through a bored timber at the apex.",
        )
        ctx.expect_overlap(
            bell,
            frame,
            axes="yz",
            elem_a="pivot_rod",
            elem_b=post,
            min_overlap=0.050,
            name=f"pivot rod passes through {post}",
        )

    ctx.allow_overlap(
        clapper,
        bell,
        elem_a="clapper_pin",
        elem_b="clapper_hanger_0",
        reason="The clapper pin is intentionally captured in the hanger cheek.",
    )
    ctx.allow_overlap(
        clapper,
        bell,
        elem_a="clapper_pin",
        elem_b="clapper_hanger_1",
        reason="The clapper pin is intentionally captured in the hanger cheek.",
    )
    ctx.expect_gap(
        clapper,
        bell,
        axis="y",
        positive_elem="clapper_pin",
        negative_elem="clapper_hanger_0",
        max_penetration=0.008,
        max_gap=0.002,
        name="clapper pin seats in rear hanger cheek",
    )
    ctx.expect_gap(
        bell,
        clapper,
        axis="y",
        positive_elem="clapper_hanger_1",
        negative_elem="clapper_pin",
        max_penetration=0.008,
        max_gap=0.002,
        name="clapper pin seats in front hanger cheek",
    )

    ctx.check(
        "bell cote has two revolute mechanisms",
        bell_joint.articulation_type == ArticulationType.REVOLUTE
        and clapper_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"bell={bell_joint.articulation_type}, clapper={clapper_joint.articulation_type}",
    )
    ctx.expect_gap(
        frame,
        bell,
        axis="z",
        positive_elem="ridge_beam",
        negative_elem="pivot_rod",
        min_gap=0.015,
        max_gap=0.040,
        name="pivot rod hangs just under ridge beam",
    )
    ctx.expect_within(
        clapper,
        bell,
        axes="xy",
        inner_elem="clapper_ball",
        outer_elem="bell_shell",
        margin=0.0,
        name="clapper ball sits inside the bell mouth",
    )

    rest_bell_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    with ctx.pose({bell_joint: 0.50}):
        swung_bell_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    if rest_bell_aabb is not None and swung_bell_aabb is not None:
        rest_center_x = (rest_bell_aabb[0][0] + rest_bell_aabb[1][0]) / 2.0
        swung_center_x = (swung_bell_aabb[0][0] + swung_bell_aabb[1][0]) / 2.0
        bell_moves = abs(swung_center_x - rest_center_x) > 0.18
    else:
        bell_moves = False
        rest_center_x = swung_center_x = None
    ctx.check(
        "bell swings from the apex pivot",
        bell_moves,
        details=f"rest_x={rest_center_x}, swung_x={swung_center_x}",
    )

    rest_clapper_aabb = ctx.part_element_world_aabb(clapper, elem="clapper_ball")
    with ctx.pose({clapper_joint: 0.55}):
        swung_clapper_aabb = ctx.part_element_world_aabb(clapper, elem="clapper_ball")
    if rest_clapper_aabb is not None and swung_clapper_aabb is not None:
        rest_ball_x = (rest_clapper_aabb[0][0] + rest_clapper_aabb[1][0]) / 2.0
        swung_ball_x = (swung_clapper_aabb[0][0] + swung_clapper_aabb[1][0]) / 2.0
        clapper_moves = abs(swung_ball_x - rest_ball_x) > 0.12
    else:
        clapper_moves = False
        rest_ball_x = swung_ball_x = None
    ctx.check(
        "clapper swings on its secondary pin",
        clapper_moves,
        details=f"rest_x={rest_ball_x}, swung_x={swung_ball_x}",
    )

    return ctx.report()


object_model = build_object_model()
