from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LEN = 0.34
BASE_W = 0.26
BASE_T = 0.018

CHEEK_LEN = 0.20
CHEEK_T = 0.018
CHEEK_H = 0.056
CHEEK_CENTER_X = 0.015
CHEEK_CENTER_Y = 0.102

REAR_BRIDGE_LEN = 0.034
REAR_BRIDGE_H = 0.012
REAR_BRIDGE_X = -0.096

PIVOT_Z = 0.052

PLATFORM_LEN = 0.22
PLATFORM_W = 0.16
PLATFORM_T = 0.012
DECK_X = 0.035
DECK_Z = 0.020

CARRIER_LEN = 0.11
CARRIER_W = 0.072
CARRIER_H = 0.028
CARRIER_X = 0.010
CARRIER_Z = -0.006

SHAFT_R = 0.010
BORE_R = SHAFT_R
OUTER_FACE_Y = CHEEK_CENTER_Y + CHEEK_T / 2.0
SHAFT_LEN = 2.0 * (OUTER_FACE_Y - 0.001)


def _make_base_plate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_LEN, BASE_W, BASE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(0.020, 0.0)
        .rect(0.230, 0.135)
        .cutBlind(-0.004)
    )


def _make_side_frame() -> cq.Workplane:
    cheek_z = BASE_T - 0.0005
    left_cheek = (
        cq.Workplane("XY")
        .box(CHEEK_LEN, CHEEK_T, CHEEK_H, centered=(True, True, False))
        .translate((CHEEK_CENTER_X, CHEEK_CENTER_Y, cheek_z))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(CHEEK_LEN, CHEEK_T, CHEEK_H, centered=(True, True, False))
        .translate((CHEEK_CENTER_X, -CHEEK_CENTER_Y, cheek_z))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(
            REAR_BRIDGE_LEN,
            2.0 * CHEEK_CENTER_Y + CHEEK_T,
            REAR_BRIDGE_H,
            centered=(True, True, False),
        )
        .translate((REAR_BRIDGE_X, 0.0, cheek_z))
    )
    frame = left_cheek.union(right_cheek).union(rear_bridge)
    bore = (
        cq.Workplane("XZ")
        .center(0.0, PIVOT_Z)
        .circle(BORE_R)
        .extrude(BASE_W, both=True)
    )
    return frame.cut(bore)


def _make_table_body() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .box(PLATFORM_LEN, PLATFORM_W, PLATFORM_T, centered=(True, True, False))
        .translate((DECK_X, 0.0, DECK_Z))
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.038, 0.0), (0.038, 0.0)])
        .slot2D(0.050, 0.012, angle=0.0)
        .cutBlind(-0.004)
    )
    carrier = (
        cq.Workplane("XY")
        .box(CARRIER_LEN, CARRIER_W, CARRIER_H, centered=(True, True, False))
        .translate((CARRIER_X, 0.0, CARRIER_Z))
    )
    return deck.union(carrier)


def _make_pivot_trunnion() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(SHAFT_R)
        .extrude(SHAFT_LEN / 2.0, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_table_pitch_stage")

    model.material("base_paint", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("table_alloy", rgba=(0.74, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_plate(), "base_plate"),
        material="base_paint",
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_make_side_frame(), "side_frame"),
        material="base_paint",
        name="side_frame",
    )

    platform = model.part("platform")
    platform.visual(
        mesh_from_cadquery(_make_table_body(), "tilt_platform_body"),
        material="table_alloy",
        name="table_body",
    )
    platform.visual(
        mesh_from_cadquery(_make_pivot_trunnion(), "tilt_platform_trunnion"),
        material="table_alloy",
        name="pivot_trunnion",
    )

    model.articulation(
        "base_to_platform_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.60,
        ),
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

    base = object_model.get_part("base")
    platform = object_model.get_part("platform")
    pitch = object_model.get_articulation("base_to_platform_pitch")
    limits = pitch.motion_limits

    ctx.allow_overlap(
        base,
        platform,
        elem_a="side_frame",
        elem_b="pivot_trunnion",
        reason="The platform is carried by an intentional trunnion fit inside the cheek bearings.",
    )

    ctx.check(
        "pitch joint uses a supported lateral axis",
        tuple(float(v) for v in pitch.axis) == (0.0, -1.0, 0.0),
        details=f"axis={pitch.axis}",
    )
    ctx.check(
        "pitch joint has bidirectional travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"limits={limits}",
    )

    ctx.expect_overlap(
        platform,
        base,
        axes="xy",
        elem_a="table_body",
        elem_b="base_plate",
        min_overlap=0.14,
        name="platform sits over the grounded base footprint",
    )
    ctx.expect_gap(
        platform,
        base,
        axis="z",
        positive_elem="table_body",
        negative_elem="base_plate",
        min_gap=0.012,
        name="rest pose keeps the table above the base deck",
    )
    ctx.expect_contact(
        platform,
        base,
        elem_a="pivot_trunnion",
        elem_b="side_frame",
        contact_tol=1e-6,
        name="platform stays supported by the cheek bearings",
    )

    rest_aabb = ctx.part_element_world_aabb(platform, elem="table_body")
    with ctx.pose({pitch: limits.upper}):
        ctx.expect_gap(
            platform,
            base,
            axis="z",
            positive_elem="table_body",
            negative_elem="base_plate",
            min_gap=0.003,
            name="raised pose still clears the base deck",
        )
        raised_aabb = ctx.part_element_world_aabb(platform, elem="table_body")

    with ctx.pose({pitch: limits.lower}):
        ctx.expect_gap(
            platform,
            base,
            axis="z",
            positive_elem="table_body",
            negative_elem="base_plate",
            min_gap=0.002,
            name="nose-down pose still clears the base deck",
        )

    ctx.check(
        "positive pitch lifts the table nose",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.040,
        details=f"rest_aabb={rest_aabb}, raised_aabb={raised_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
