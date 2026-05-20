from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import degrees

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.300
BASE_WIDTH = 0.220
BASE_THICKNESS = 0.012

GUIDE_OUTER_X = 0.086
GUIDE_OUTER_Y = 0.056
GUIDE_INNER_X = 0.054
GUIDE_INNER_Y = 0.032
GUIDE_HEIGHT = 0.160
GUIDE_FLOOR = 0.006

STEM_X = GUIDE_INNER_X - 0.006
STEM_Y = GUIDE_INNER_Y - 0.006
STEM_INSERTION = GUIDE_HEIGHT - GUIDE_FLOOR
STEM_VISIBLE = 0.060
STEM_TOTAL = STEM_INSERTION + STEM_VISIBLE

HEAD_BLOCK_X = 0.026
HEAD_BLOCK_Y = 0.030
HEAD_BLOCK_Z0 = 0.050
HEAD_BLOCK_HEIGHT = 0.016

EAR_X = 0.018
EAR_Y = 0.007
EAR_Z0 = 0.058
EAR_HEIGHT = 0.048
EAR_CENTER_Y = 0.014
HINGE_CENTER_Z = 0.101

BARREL_RADIUS = 0.007
BARREL_BORE_RADIUS = 0.0043
BARREL_LENGTH = (2.0 * EAR_CENTER_Y) - EAR_Y

PLATFORM_LENGTH = 0.275
PLATFORM_WIDTH = 0.235
PLATFORM_THICKNESS = 0.0045
PLATFORM_REAR_OFFSET = 0.012
PLATFORM_BOTTOM_Z = 0.018
PLATFORM_REST_PITCH = 0.38

COOLING_SLOT_LENGTH = 0.165
COOLING_SLOT_DIAMETER = 0.094
COOLING_SLOT_CENTER_X = 0.152

CENTER_BRIDGE_LENGTH = 0.040
CENTER_BRIDGE_WIDTH = 0.014
SIDE_RIB_LENGTH = 0.180
SIDE_RIB_WIDTH = 0.012
SIDE_RIB_HEIGHT = 0.010
SIDE_RIB_CENTER_X = 0.125
SIDE_RIB_CENTER_Y = 0.082

FRONT_LIP_THICKNESS = 0.004
FRONT_LIP_HEIGHT = 0.010
FRONT_LIP_WIDTH = 0.160

SLIDE_UPPER = 0.085
PITCH_LOWER = -0.20
PITCH_UPPER = 0.45


def _base_plate_shape() -> cq.Workplane:
    return cq.Workplane("XY").ellipse(BASE_LENGTH / 2.0, BASE_WIDTH / 2.0).extrude(BASE_THICKNESS)


def _guide_sleeve_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .rect(GUIDE_OUTER_X, GUIDE_OUTER_Y)
        .extrude(GUIDE_HEIGHT)
        .edges("|Z")
        .fillet(0.007)
    )
    cavity = (
        cq.Workplane("XY")
        .rect(GUIDE_INNER_X, GUIDE_INNER_Y)
        .extrude(GUIDE_HEIGHT - GUIDE_FLOOR)
        .translate((0.0, 0.0, GUIDE_FLOOR))
    )
    return outer.cut(cavity).translate((0.0, 0.0, BASE_THICKNESS))


def _pedestal_stem_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .rect(STEM_X, STEM_Y)
        .extrude(STEM_TOTAL)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, -STEM_INSERTION))
    )


def _pedestal_head_shape() -> cq.Workplane:
    head_block = (
        cq.Workplane("XY")
        .rect(HEAD_BLOCK_X, HEAD_BLOCK_Y)
        .extrude(HEAD_BLOCK_HEIGHT)
        .translate((0.0, 0.0, HEAD_BLOCK_Z0))
        .edges("|Z")
        .fillet(0.003)
    )
    ear_blank = (
        cq.Workplane("XY")
        .rect(EAR_X, EAR_Y)
        .extrude(EAR_HEIGHT)
    )
    ear_hole = (
        cq.Workplane("XZ")
        .circle(BARREL_BORE_RADIUS)
        .extrude(EAR_Y, both=True)
        .translate((0.0, 0.0, HINGE_CENTER_Z))
    )
    ear = ear_blank.cut(ear_hole)
    left_ear = ear.translate((0.0, EAR_CENTER_Y, EAR_Z0))
    right_ear = ear.translate((0.0, -EAR_CENTER_Y, EAR_Z0))
    return head_block.union(left_ear).union(right_ear)


def _platform_shape() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .rect(PLATFORM_LENGTH, PLATFORM_WIDTH)
        .extrude(PLATFORM_THICKNESS)
        .translate((PLATFORM_REAR_OFFSET + PLATFORM_LENGTH / 2.0, 0.0, PLATFORM_BOTTOM_Z))
        .edges("|Z")
        .fillet(0.002)
    )
    cooling_slot = (
        cq.Workplane("XY")
        .slot2D(COOLING_SLOT_LENGTH, COOLING_SLOT_DIAMETER)
        .extrude(PLATFORM_THICKNESS + 0.010)
        .translate((COOLING_SLOT_CENTER_X, 0.0, PLATFORM_BOTTOM_Z - 0.005))
    )
    deck = deck.cut(cooling_slot)

    center_bridge = (
        cq.Workplane("XY")
        .rect(CENTER_BRIDGE_LENGTH, CENTER_BRIDGE_WIDTH)
        .extrude(PLATFORM_BOTTOM_Z)
        .translate((CENTER_BRIDGE_LENGTH / 2.0, 0.0, 0.0))
    )
    side_rib = (
        cq.Workplane("XY")
        .rect(SIDE_RIB_LENGTH, SIDE_RIB_WIDTH)
        .extrude(SIDE_RIB_HEIGHT)
        .translate((SIDE_RIB_CENTER_X, SIDE_RIB_CENTER_Y, PLATFORM_BOTTOM_Z - SIDE_RIB_HEIGHT))
    )
    mirrored_rib = side_rib.mirror(mirrorPlane="XZ", union=False)

    front_lip = (
        cq.Workplane("XY")
        .rect(FRONT_LIP_THICKNESS, FRONT_LIP_WIDTH)
        .extrude(FRONT_LIP_HEIGHT)
        .translate(
            (
                PLATFORM_REAR_OFFSET + PLATFORM_LENGTH - (FRONT_LIP_THICKNESS / 2.0),
                0.0,
                PLATFORM_BOTTOM_Z + PLATFORM_THICKNESS,
            )
        )
    )
    hinge_barrel_outer = (
        cq.Workplane("XZ")
        .circle(BARREL_RADIUS)
        .extrude(BARREL_LENGTH / 2.0, both=True)
    )
    hinge_barrel = hinge_barrel_outer

    platform = (
        hinge_barrel.union(center_bridge)
        .union(deck)
        .union(side_rib)
        .union(mirrored_rib)
        .union(front_lip)
    )
    return platform.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -degrees(PLATFORM_REST_PITCH))


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][2] + aabb[1][2]) / 2.0


def _aabb_max_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return aabb[1][2]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aluminum_laptop_stand")

    model.material("aluminum", rgba=(0.77, 0.79, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate"),
        material="aluminum",
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_guide_sleeve_shape(), "guide_sleeve"),
        material="aluminum",
        name="guide_sleeve",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_stem_shape(), "pedestal_stem"),
        material="aluminum",
        name="pedestal_stem",
    )
    pedestal.visual(
        mesh_from_cadquery(_pedestal_head_shape(), "pedestal_head"),
        material="aluminum",
        name="pedestal_head",
    )

    platform = model.part("platform")
    platform.visual(
        mesh_from_cadquery(_platform_shape(), "platform_shell"),
        material="aluminum",
        name="platform_shell",
    )

    model.articulation(
        "base_to_pedestal",
        ArticulationType.PRISMATIC,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + GUIDE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=SLIDE_UPPER,
        ),
    )
    model.articulation(
        "pedestal_to_platform",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, HINGE_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pedestal = object_model.get_part("pedestal")
    platform = object_model.get_part("platform")
    slide = object_model.get_articulation("base_to_pedestal")
    pitch = object_model.get_articulation("pedestal_to_platform")

    ctx.allow_overlap(
        pedestal,
        platform,
        elem_a="pedestal_head",
        elem_b="platform_shell",
        reason=(
            "The pitch hinge is represented as a close-fitted knuckle between the head ears and "
            "the platform hinge barrel, and the mesh-backed pivot line may register a tiny "
            "interpenetration at the hinge axis."
        ),
    )

    ctx.expect_gap(
        platform,
        base,
        axis="z",
        min_gap=0.090,
        name="platform clears base at rest",
    )
    ctx.expect_within(
        pedestal,
        base,
        axes="xy",
        inner_elem="pedestal_stem",
        outer_elem="guide_sleeve",
        margin=0.004,
        name="pedestal stem stays centered in the sleeve at rest",
    )
    ctx.expect_overlap(
        pedestal,
        base,
        axes="z",
        elem_a="pedestal_stem",
        elem_b="guide_sleeve",
        min_overlap=0.120,
        name="pedestal retains deep insertion at rest",
    )

    rest_pos = ctx.part_world_position(pedestal)
    with ctx.pose({slide: SLIDE_UPPER}):
        ctx.expect_within(
            pedestal,
            base,
            axes="xy",
            inner_elem="pedestal_stem",
            outer_elem="guide_sleeve",
            margin=0.004,
            name="pedestal stem stays centered at full lift",
        )
        ctx.expect_overlap(
            pedestal,
            base,
            axes="z",
            elem_a="pedestal_stem",
            elem_b="guide_sleeve",
            min_overlap=0.048,
            name="pedestal keeps retained insertion at full lift",
        )
        lifted_pos = ctx.part_world_position(pedestal)
        ctx.expect_gap(
            platform,
            base,
            axis="z",
            min_gap=0.150,
            name="platform clears base when fully raised",
        )

    ctx.check(
        "pedestal slides upward",
        rest_pos is not None
        and lifted_pos is not None
        and lifted_pos[2] > rest_pos[2] + 0.070,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    with ctx.pose({pitch: PITCH_LOWER}):
        flatter_aabb = ctx.part_world_aabb(platform)
    with ctx.pose({pitch: PITCH_UPPER}):
        steeper_aabb = ctx.part_world_aabb(platform)

    flatter_center_z = _aabb_center_z(flatter_aabb)
    steeper_center_z = _aabb_center_z(steeper_aabb)
    flatter_max_z = _aabb_max_z(flatter_aabb)
    steeper_max_z = _aabb_max_z(steeper_aabb)
    ctx.check(
        "platform pitches steeper around the head hinge",
        flatter_center_z is not None
        and steeper_center_z is not None
        and flatter_max_z is not None
        and steeper_max_z is not None
        and steeper_center_z > flatter_center_z + 0.020
        and steeper_max_z > flatter_max_z + 0.035,
        details=(
            f"flat_center_z={flatter_center_z}, steep_center_z={steeper_center_z}, "
            f"flat_max_z={flatter_max_z}, steep_max_z={steeper_max_z}"
        ),
    )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
