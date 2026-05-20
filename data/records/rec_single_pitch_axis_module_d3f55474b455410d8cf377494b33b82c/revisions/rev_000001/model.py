from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_X = 0.160
PLATE_Y = 0.100
PLATE_T = 0.012

CHEEK_X = 0.092
CHEEK_T = 0.010
CHEEK_DROP = 0.090
INNER_YOKE_GAP = 0.076
CHEEK_CENTER_Y = INNER_YOKE_GAP / 2.0 + CHEEK_T / 2.0

TRUNNION_Z = -0.054
TRUNNION_BARREL_R = 0.012
TRUNNION_BARREL_LEN = INNER_YOKE_GAP

HEAD_X = 0.082
HEAD_Y = 0.060
HEAD_Z = 0.060
HEAD_X_OFFSET = 0.008
HEAD_Z_CENTER = -0.060
LENS_X = 0.018
LENS_Z = -0.086


def _top_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(PLATE_X, PLATE_Y, PLATE_T, centered=(True, True, False))
        .translate((0.0, 0.0, -PLATE_T))
        .edges("|Z")
        .fillet(0.003)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.045, 0.0), (0.045, 0.0)])
        .slot2D(0.030, 0.008, angle=0.0)
        .cutThruAll()
    )


def _yoke_body_shape() -> cq.Workplane:
    left_cheek = (
        cq.Workplane("XY")
        .box(CHEEK_X, CHEEK_T, CHEEK_DROP, centered=(True, True, False))
        .translate((0.0, CHEEK_CENTER_Y, -PLATE_T - CHEEK_DROP))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(CHEEK_X, CHEEK_T, CHEEK_DROP, centered=(True, True, False))
        .translate((0.0, -CHEEK_CENTER_Y, -PLATE_T - CHEEK_DROP))
    )

    return (
        left_cheek.union(right_cheek)
        .edges("|Z")
        .fillet(0.002)
    )


def _moving_head_shell() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(HEAD_X, HEAD_Y, HEAD_Z)
        .translate((HEAD_X_OFFSET, 0.0, HEAD_Z_CENTER))
        .edges("|Z")
        .fillet(0.006)
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.040, 0.054, 0.018)
        .translate((-0.010, 0.0, -0.030))
        .edges("|Z")
        .fillet(0.004)
    )
    pivot_block = (
        cq.Workplane("XY")
        .box(0.034, 0.050, 0.028)
        .translate((0.000, 0.0, -0.016))
        .edges("|Z")
        .fillet(0.004)
    )
    lens_shroud = (
        cq.Workplane("XY")
        .circle(0.026)
        .extrude(0.020)
        .translate((LENS_X, 0.0, -0.094))
    )
    lens_recess = (
        cq.Workplane("XY")
        .circle(0.016)
        .extrude(0.008)
        .translate((LENS_X, 0.0, -0.096))
    )
    trunnion_barrel = (
        cq.Workplane("XY")
        .circle(TRUNNION_BARREL_R)
        .extrude(TRUNNION_BARREL_LEN)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((0.0, -TRUNNION_BARREL_LEN / 2.0, 0.0))
    )
    return (
        body.union(top_cap)
        .union(pivot_block)
        .union(lens_shroud)
        .union(trunnion_barrel)
        .cut(lens_recess)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_pitch_fixture")

    model.material("powder_coat_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("matte_black", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("satin_black", rgba=(0.16, 0.17, 0.18, 1.0))

    support_bracket = model.part("support_bracket")
    support_bracket.visual(
        mesh_from_cadquery(_top_plate_shape(), "top_plate"),
        material="powder_coat_dark",
        name="top_plate",
    )
    support_bracket.visual(
        mesh_from_cadquery(_yoke_body_shape(), "yoke_body"),
        material="powder_coat_dark",
        name="yoke_body",
    )
    support_bracket.inertial = Inertial.from_geometry(
        Box((PLATE_X, PLATE_Y, CHEEK_DROP + PLATE_T)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -(CHEEK_DROP + PLATE_T) / 2.0)),
    )

    moving_head = model.part("moving_head")
    moving_head.visual(
        mesh_from_cadquery(_moving_head_shell(), "moving_head_shell"),
        material="matte_black",
        name="head_shell",
    )
    moving_head.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(LENS_X, 0.0, LENS_Z)),
        material="satin_black",
        name="lens_bezel",
    )
    moving_head.inertial = Inertial.from_geometry(
        Box((0.094, TRUNNION_BARREL_LEN, 0.100)),
        mass=0.85,
        origin=Origin(xyz=(0.010, 0.0, -0.055)),
    )

    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=support_bracket,
        child=moving_head,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.2,
            lower=-0.45,
            upper=1.00,
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

    support_bracket = object_model.get_part("support_bracket")
    moving_head = object_model.get_part("moving_head")
    head_pitch = object_model.get_articulation("head_pitch")

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    ctx.check(
        "pitch axis is horizontal trunnion axis",
        tuple(head_pitch.axis) == (0.0, -1.0, 0.0),
        details=f"axis={head_pitch.axis}",
    )
    ctx.expect_origin_gap(
        support_bracket,
        moving_head,
        axis="z",
        min_gap=0.045,
        max_gap=0.065,
        name="moving head hangs below the fixed support",
    )

    with ctx.pose({head_pitch: 0.0}):
        ctx.expect_gap(
            support_bracket,
            moving_head,
            axis="z",
            positive_elem="top_plate",
            negative_elem="lens_bezel",
            min_gap=0.070,
            name="lens sits well below the top support plate at rest",
        )
        rest_lens_aabb = ctx.part_element_world_aabb(moving_head, elem="lens_bezel")

    upper = head_pitch.motion_limits.upper if head_pitch.motion_limits is not None else None
    with ctx.pose({head_pitch: upper if upper is not None else 1.0}):
        ctx.expect_gap(
            support_bracket,
            moving_head,
            axis="z",
            positive_elem="top_plate",
            negative_elem="lens_bezel",
            min_gap=0.030,
            name="lens clears the support plate at max positive tilt",
        )
        pitched_lens_aabb = ctx.part_element_world_aabb(moving_head, elem="lens_bezel")

    rest_center = _aabb_center(rest_lens_aabb)
    pitched_center = _aabb_center(pitched_lens_aabb)
    ctx.check(
        "positive tilt pitches the head forward and upward",
        rest_center is not None
        and pitched_center is not None
        and pitched_center[0] > rest_center[0] + 0.050
        and pitched_center[2] > rest_center[2] + 0.030,
        details=f"rest_center={rest_center}, pitched_center={pitched_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
