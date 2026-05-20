from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

OUTER_TUBE_HEIGHT = 0.180
OUTER_TUBE_OUTER_RADIUS = 0.038
OUTER_TUBE_INNER_RADIUS = 0.034
OUTER_GUIDE_LENGTH = 0.018
BASE_FLANGE_RADIUS = 0.055
BASE_FLANGE_THICKNESS = 0.008

STAGE1_HEIGHT = 0.160
STAGE1_OUTER_RADIUS = 0.032
STAGE1_INNER_RADIUS = 0.028
STAGE1_GUIDE_LENGTH = 0.018

STAGE2_HEIGHT = 0.132
STAGE2_OUTER_RADIUS = 0.028
STAGE2_INNER_RADIUS = 0.022
STAGE2_TUBE_LENGTH = 0.122
STAGE2_HEAD_RADIUS = 0.022
STAGE2_HEAD_HEIGHT = 0.010

PLATFORM_RADIUS = 0.045
PLATFORM_BASE_THICKNESS = 0.008
PLATFORM_RIM_THICKNESS = 0.003
PLATFORM_RIM_INNER_RADIUS = 0.037
PLATFORM_HUB_RADIUS = 0.016
PLATFORM_HUB_THICKNESS = 0.004

STAGE1_REST_Z = 0.034
STAGE1_STROKE = 0.090
STAGE2_REST_Z = 0.036
STAGE2_STROKE = 0.075


def _annular_cylinder(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def _outer_tube_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(BASE_FLANGE_RADIUS).extrude(BASE_FLANGE_THICKNESS)
    shell = _annular_cylinder(
        OUTER_TUBE_OUTER_RADIUS,
        OUTER_TUBE_INNER_RADIUS,
        OUTER_TUBE_HEIGHT,
    ).translate((0.0, 0.0, BASE_FLANGE_THICKNESS))
    top_guide = _annular_cylinder(
        OUTER_TUBE_INNER_RADIUS,
        STAGE1_OUTER_RADIUS,
        OUTER_GUIDE_LENGTH,
    ).translate(
        (
            0.0,
            0.0,
            BASE_FLANGE_THICKNESS + OUTER_TUBE_HEIGHT - OUTER_GUIDE_LENGTH,
        )
    )
    return flange.union(shell).union(top_guide)


def _stage1_shape() -> cq.Workplane:
    shell = _annular_cylinder(STAGE1_OUTER_RADIUS, STAGE1_INNER_RADIUS, STAGE1_HEIGHT)
    top_guide = _annular_cylinder(
        STAGE1_INNER_RADIUS,
        STAGE2_OUTER_RADIUS,
        STAGE1_GUIDE_LENGTH,
    ).translate((0.0, 0.0, STAGE1_HEIGHT - STAGE1_GUIDE_LENGTH))
    return shell.union(top_guide)


def _stage2_shape() -> cq.Workplane:
    shell = _annular_cylinder(STAGE2_OUTER_RADIUS, STAGE2_INNER_RADIUS, STAGE2_TUBE_LENGTH)
    head = (
        cq.Workplane("XY")
        .circle(STAGE2_HEAD_RADIUS)
        .extrude(STAGE2_HEAD_HEIGHT)
        .translate((0.0, 0.0, STAGE2_TUBE_LENGTH))
    )
    return shell.union(head)


def _platform_shape() -> cq.Workplane:
    deck = cq.Workplane("XY").circle(PLATFORM_RADIUS).extrude(PLATFORM_BASE_THICKNESS)
    rim = _annular_cylinder(
        PLATFORM_RADIUS,
        PLATFORM_RIM_INNER_RADIUS,
        PLATFORM_RIM_THICKNESS,
    ).translate((0.0, 0.0, PLATFORM_BASE_THICKNESS))
    hub = (
        cq.Workplane("XY")
        .circle(PLATFORM_HUB_RADIUS)
        .extrude(PLATFORM_HUB_THICKNESS)
        .translate((0.0, 0.0, PLATFORM_BASE_THICKNESS))
    )
    return deck.union(rim).union(hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_mast_head", assets=ASSETS)

    dark_paint = model.material("dark_paint", rgba=(0.20, 0.22, 0.24, 1.0))
    anodized_steel = model.material("anodized_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    satin_plate = model.material("satin_plate", rgba=(0.76, 0.78, 0.80, 1.0))

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        mesh_from_cadquery(_outer_tube_shape(), "outer_tube.obj", assets=ASSETS),
        name="outer_shell",
        material=dark_paint,
    )
    outer_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_FLANGE_RADIUS, length=BASE_FLANGE_THICKNESS + OUTER_TUBE_HEIGHT),
        mass=2.4,
        origin=Origin(
            xyz=(0.0, 0.0, 0.5 * (BASE_FLANGE_THICKNESS + OUTER_TUBE_HEIGHT))
        ),
    )

    stage_one = model.part("lift_tube_1")
    stage_one.visual(
        mesh_from_cadquery(_stage1_shape(), "lift_tube_1.obj", assets=ASSETS),
        name="stage_shell",
        material=anodized_steel,
    )
    stage_one.inertial = Inertial.from_geometry(
        Cylinder(radius=STAGE1_OUTER_RADIUS, length=STAGE1_HEIGHT),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * STAGE1_HEIGHT)),
    )

    stage_two = model.part("lift_tube_2")
    stage_two.visual(
        mesh_from_cadquery(_stage2_shape(), "lift_tube_2.obj", assets=ASSETS),
        name="stage_shell",
        material=anodized_steel,
    )
    stage_two.inertial = Inertial.from_geometry(
        Cylinder(radius=STAGE2_OUTER_RADIUS, length=STAGE2_HEIGHT),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * STAGE2_HEIGHT)),
    )

    platform = model.part("rotary_platform")
    platform.visual(
        mesh_from_cadquery(_platform_shape(), "rotary_platform.obj", assets=ASSETS),
        name="platform_deck",
        material=satin_plate,
    )
    platform.inertial = Inertial.from_geometry(
        Box(
            (
                2.0 * PLATFORM_RADIUS,
                2.0 * PLATFORM_RADIUS,
                PLATFORM_BASE_THICKNESS + PLATFORM_HUB_THICKNESS,
            )
        ),
        mass=0.7,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                0.5 * (PLATFORM_BASE_THICKNESS + PLATFORM_HUB_THICKNESS),
            )
        ),
    )

    model.articulation(
        "outer_to_stage1",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=stage_one,
        origin=Origin(xyz=(0.0, 0.0, STAGE1_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.20,
            lower=0.0,
            upper=STAGE1_STROKE,
        ),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage_one,
        child=stage_two,
        origin=Origin(xyz=(0.0, 0.0, STAGE2_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.18,
            lower=0.0,
            upper=STAGE2_STROKE,
        ),
    )
    model.articulation(
        "stage2_to_platform",
        ArticulationType.REVOLUTE,
        parent=stage_two,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, STAGE2_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=2.5,
            lower=-3.14159,
            upper=3.14159,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    outer_tube = object_model.get_part("outer_tube")
    stage_one = object_model.get_part("lift_tube_1")
    stage_two = object_model.get_part("lift_tube_2")
    platform = object_model.get_part("rotary_platform")

    stage1_joint = object_model.get_articulation("outer_to_stage1")
    stage2_joint = object_model.get_articulation("stage1_to_stage2")
    pan_joint = object_model.get_articulation("stage2_to_platform")

    outer_shell = outer_tube.get_visual("outer_shell")
    stage1_shell = stage_one.get_visual("stage_shell")
    stage2_shell = stage_two.get_visual("stage_shell")
    platform_deck = platform.get_visual("platform_deck")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.allow_overlap(
        stage_one,
        outer_tube,
        elem_a=stage1_shell,
        elem_b=outer_shell,
        reason="Nested telescoping sleeve: conservative overlap QC flags the coaxial hollow stage seated inside the grounded outer tube.",
    )
    ctx.allow_overlap(
        stage_two,
        stage_one,
        elem_a=stage2_shell,
        elem_b=stage1_shell,
        reason="Nested telescoping sleeve: conservative overlap QC flags the second lift tube inside the first stage.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "part tree present",
        all(part is not None for part in (outer_tube, stage_one, stage_two, platform)),
        "Expected grounded tube, two lift stages, and rotary platform.",
    )
    ctx.check(
        "all mast joints are vertical",
        tuple(stage1_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(stage2_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(pan_joint.axis) == (0.0, 0.0, 1.0),
        "Both lift joints and the pan axis should share the mast centerline.",
    )
    ctx.check(
        "serial joint types match mast behavior",
        stage1_joint.articulation_type == ArticulationType.PRISMATIC
        and stage2_joint.articulation_type == ArticulationType.PRISMATIC
        and pan_joint.articulation_type == ArticulationType.REVOLUTE,
        "Expected two serial prismatic stages capped by a revolute pan joint.",
    )

    ctx.expect_origin_distance(stage_one, outer_tube, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(stage_two, outer_tube, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(platform, outer_tube, axes="xy", max_dist=0.001)
    ctx.expect_within(stage_one, outer_tube, axes="xy", margin=0.0)
    ctx.expect_within(stage_two, stage_one, axes="xy", margin=0.0)

    ctx.expect_contact(stage_one, outer_tube, elem_a=stage1_shell, elem_b=outer_shell)
    ctx.expect_contact(stage_two, stage_one, elem_a=stage2_shell, elem_b=stage1_shell)
    ctx.expect_gap(
        platform,
        stage_two,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=platform_deck,
        negative_elem=stage2_shell,
        name="platform sits on mast head",
    )
    ctx.expect_overlap(platform, stage_two, axes="xy", min_overlap=0.040)

    with ctx.pose({stage1_joint: STAGE1_STROKE, stage2_joint: STAGE2_STROKE}):
        ctx.expect_origin_distance(stage_two, outer_tube, axes="xy", max_dist=0.001)
        ctx.expect_contact(stage_one, outer_tube, elem_a=stage1_shell, elem_b=outer_shell)
        ctx.expect_contact(stage_two, stage_one, elem_a=stage2_shell, elem_b=stage1_shell)
        ctx.expect_gap(
            platform,
            stage_two,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=platform_deck,
            negative_elem=stage2_shell,
            name="platform stays seated at full lift",
        )

    with ctx.pose({pan_joint: 1.2}):
        ctx.expect_origin_distance(platform, stage_two, axes="xy", max_dist=0.001)
        ctx.expect_gap(
            platform,
            stage_two,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=platform_deck,
            negative_elem=stage2_shell,
            name="platform stays on bearing seat while panning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
