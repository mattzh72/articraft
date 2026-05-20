from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _carriage_body_mesh():
    """A hollow sliding carriage with a forward bearing nose.

    The vertical bore is intentionally larger than the fixed steel column so the
    part can translate without relying on overlap allowances.
    """

    sleeve_h = 0.150
    outer_r = 0.055
    bore_r = 0.032
    sleeve = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(bore_r)
        .extrude(sleeve_h)
        .translate((0.0, 0.0, -sleeve_h / 2.0))
    )
    head_block = (
        cq.Workplane("XY")
        .box(0.130, 0.175, 0.080)
        .translate((0.0, -0.100, -0.006))
    )
    bearing_boss = (
        cq.Workplane("XY")
        .circle(0.034)
        .extrude(0.026)
        .translate((0.0, -0.145, -0.058))
    )
    bore_cutter = (
        cq.Workplane("XY")
        .circle(bore_r)
        .extrude(sleeve_h + 0.04)
        .translate((0.0, 0.0, -sleeve_h / 2.0 - 0.02))
    )
    return sleeve.union(head_block).union(bearing_boss).cut(bore_cutter)


def _locking_collar_mesh():
    """Black split-collar style band and screw lug around the column bore."""

    band_h = 0.038
    outer_r = 0.063
    bore_r = 0.032
    z0 = 0.055
    band = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(bore_r)
        .extrude(band_h)
        .translate((0.0, 0.0, z0))
    )
    lug = cq.Workplane("XY").box(0.034, 0.040, 0.032).translate((0.075, 0.0, z0 + 0.020))
    bore_cutter = cq.Workplane("XY").circle(bore_r).extrude(band_h + 0.010).translate((0.0, 0.0, z0 - 0.005))
    return band.union(lug).cut(bore_cutter)


def _chuck_body_mesh():
    """Tapered quick-release chuck shell, modeled as a lathe profile."""

    profile = [
        (0.018, -0.044),
        (0.026, -0.052),
        (0.028, -0.082),
        (0.023, -0.108),
        (0.014, -0.135),
        (0.009, -0.148),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=48), "tapered_chuck")


def _column_bushing_mesh():
    """Short bearing liner that lightly captures the column inside the sleeve."""

    height = 0.014
    return (
        cq.Workplane("XY")
        .circle(0.034)
        .circle(0.024)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cordless_drill_press_stand")

    painted_steel = model.material("deep_blue_painted_steel", color=(0.05, 0.09, 0.16, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.64, 0.66, 0.65, 1.0))
    dark_steel = model.material("dark_oxide_steel", color=(0.10, 0.11, 0.12, 1.0))
    black_plastic = model.material("black_plastic", color=(0.01, 0.01, 0.012, 1.0))
    red_release = model.material("red_quick_release", color=(0.70, 0.05, 0.03, 1.0))
    fence_yellow = model.material("anodized_fence_yellow", color=(0.86, 0.62, 0.12, 1.0))
    bushing_bronze = model.material("oilite_bronze", color=(0.62, 0.38, 0.15, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.560, 0.360, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=painted_steel,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.025, length=0.750),
        origin=Origin(xyz=(0.0, 0.105, 0.405)),
        material=brushed_steel,
        name="steel_column",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=Origin(xyz=(0.0, 0.105, 0.037)),
        material=dark_steel,
        name="column_flange",
    )
    for i, x in enumerate((-0.232, 0.232)):
        for j, y in enumerate((-0.142, 0.142)):
            base.visual(
                Cylinder(radius=0.008, length=0.005),
                origin=Origin(xyz=(x, y, 0.0325)),
                material=dark_steel,
                name=f"base_bolt_{i}_{j}",
            )
    base.visual(
        Box((0.440, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.145, 0.034)),
        material=dark_steel,
        name="front_fence_rail",
    )
    base.visual(
        Box((0.440, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.095, 0.034)),
        material=dark_steel,
        name="rear_fence_rail",
    )

    head_carriage = model.part("head_carriage")
    head_carriage.visual(
        mesh_from_cadquery(_carriage_body_mesh(), "carriage_body", tolerance=0.0008, angular_tolerance=0.08),
        material=dark_steel,
        name="carriage_body",
    )
    head_carriage.visual(
        mesh_from_cadquery(_locking_collar_mesh(), "locking_collar", tolerance=0.0008, angular_tolerance=0.08),
        material=black_plastic,
        name="locking_collar",
    )
    head_carriage.visual(
        mesh_from_cadquery(_column_bushing_mesh(), "lower_bushing", tolerance=0.0008, angular_tolerance=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=bushing_bronze,
        name="lower_bushing",
    )
    head_carriage.visual(
        mesh_from_cadquery(_column_bushing_mesh(), "upper_bushing", tolerance=0.0008, angular_tolerance=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=bushing_bronze,
        name="upper_bushing",
    )

    collar_knob = model.part("collar_knob")
    collar_knob.visual(
        Cylinder(radius=0.006, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="clamp_screw",
    )
    collar_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.024,
                body_style="lobed",
                grip=KnobGrip(style="ribbed", count=8, depth=0.0014),
                edge_radius=0.001,
            ),
            "collar_knob_cap",
        ),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="knob_cap",
    )

    chuck = model.part("chuck")
    chuck.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=brushed_steel,
        name="spindle_shank",
    )
    chuck.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=red_release,
        name="release_sleeve",
    )
    chuck.visual(
        _chuck_body_mesh(),
        material=brushed_steel,
        name="chuck_body",
    )
    chuck.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.146)),
        material=dark_steel,
        name="jaw_socket",
    )
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        r = 0.007
        chuck.visual(
            Box((0.006, 0.0035, 0.034)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), -0.157),
                rpy=(0.18 * math.sin(angle), -0.18 * math.cos(angle), angle),
            ),
            material=dark_steel,
            name=f"jaw_{idx}",
        )

    fence_guide = model.part("fence_guide")
    fence_guide.visual(
        Box((0.125, 0.070, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_steel,
        name="slide_shoe",
    )
    fence_guide.visual(
        Box((0.300, 0.014, 0.095)),
        origin=Origin(xyz=(0.0, 0.030, 0.0595)),
        material=fence_yellow,
        name="guide_plate",
    )
    fence_guide.visual(
        Box((0.280, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.017, 0.019)),
        material=fence_yellow,
        name="lower_rib",
    )

    model.articulation(
        "base_to_head_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_carriage,
        origin=Origin(xyz=(0.0, 0.105, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.18, lower=0.0, upper=0.220),
    )
    model.articulation(
        "head_to_collar_knob",
        ArticulationType.REVOLUTE,
        parent=head_carriage,
        child=collar_knob,
        origin=Origin(xyz=(0.092, 0.0, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "head_to_chuck",
        ArticulationType.REVOLUTE,
        parent=head_carriage,
        child=chuck,
        origin=Origin(xyz=(0.0, -0.145, -0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "base_to_fence_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fence_guide,
        origin=Origin(xyz=(0.0, -0.120, 0.038)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=-0.160, upper=0.160),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base")
    head = object_model.get_part("head_carriage")
    chuck = object_model.get_part("chuck")
    fence = object_model.get_part("fence_guide")
    head_slide = object_model.get_articulation("base_to_head_carriage")
    fence_slide = object_model.get_articulation("base_to_fence_guide")
    chuck_spin = object_model.get_articulation("head_to_chuck")

    ctx.allow_overlap(
        base,
        head,
        elem_a="steel_column",
        elem_b="lower_bushing",
        reason="The short bronze liner is intentionally modeled as a lightly captured sliding fit on the steel column.",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="steel_column",
        elem_b="upper_bushing",
        reason="The upper liner is intentionally modeled as a lightly captured sliding fit on the steel column.",
    )
    ctx.allow_overlap(
        chuck,
        head,
        elem_a="spindle_shank",
        elem_b="carriage_body",
        reason="The chuck spindle is intentionally nested into the carriage bearing socket for the revolute joint.",
    )

    ctx.expect_overlap(
        head,
        base,
        axes="xy",
        elem_a="carriage_body",
        elem_b="steel_column",
        min_overlap=0.040,
        name="carriage surrounds the round column in plan",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="xyz",
        elem_a="lower_bushing",
        elem_b="steel_column",
        min_overlap=0.010,
        name="lower bushing captures column locally",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="xyz",
        elem_a="upper_bushing",
        elem_b="steel_column",
        min_overlap=0.010,
        name="upper bushing captures column locally",
    )
    ctx.expect_gap(
        head,
        chuck,
        axis="z",
        positive_elem="carriage_body",
        negative_elem="spindle_shank",
        max_gap=0.001,
        max_penetration=0.020,
        name="chuck shank seats under carriage bearing",
    )
    ctx.expect_gap(
        fence,
        base,
        axis="z",
        positive_elem="slide_shoe",
        negative_elem="front_fence_rail",
        max_gap=0.001,
        max_penetration=0.000001,
        name="fence shoe rides on base rail",
    )

    head_rest = ctx.part_world_position(head)
    with ctx.pose({head_slide: 0.180}):
        head_raised = ctx.part_world_position(head)
        ctx.expect_overlap(
            head,
            base,
            axes="xy",
            elem_a="carriage_body",
            elem_b="steel_column",
            min_overlap=0.040,
            name="raised carriage remains guided by column",
        )
    ctx.check(
        "head carriage slides upward on column",
        head_rest is not None and head_raised is not None and head_raised[2] > head_rest[2] + 0.170,
        details=f"rest={head_rest}, raised={head_raised}",
    )

    fence_rest = ctx.part_world_position(fence)
    with ctx.pose({fence_slide: 0.120}):
        fence_shifted = ctx.part_world_position(fence)
    ctx.check(
        "fence guide slides along base face",
        fence_rest is not None and fence_shifted is not None and fence_shifted[0] > fence_rest[0] + 0.110,
        details=f"rest={fence_rest}, shifted={fence_shifted}",
    )

    chuck_rest = ctx.part_world_position(chuck)
    with ctx.pose({chuck_spin: 1.2}):
        chuck_rotated = ctx.part_world_position(chuck)
    ctx.check(
        "chuck revolute joint spins in place",
        chuck_rest is not None
        and chuck_rotated is not None
        and abs(chuck_rest[0] - chuck_rotated[0]) < 1e-6
        and abs(chuck_rest[1] - chuck_rotated[1]) < 1e-6
        and abs(chuck_rest[2] - chuck_rotated[2]) < 1e-6,
        details=f"rest={chuck_rest}, rotated={chuck_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
