from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 0.14
BASE_W = 0.10
BASE_T = 0.018

PEDESTAL_CORE_LEN = 0.072
PEDESTAL_W = 0.078
PEDESTAL_H = 0.082
SADDLE_LEN = 0.098
SADDLE_H = 0.016

GUIDE_LEN = 0.42
GUIDE_W = 0.048
GUIDE_H = 0.022

CARRIAGE_LEN = 0.11
CARRIAGE_W = 0.088
CARRIAGE_TOP_T = 0.024
CARRIAGE_SKIRT_T = 0.013
CARRIAGE_SKIRT_DROP = 0.016
CARRIAGE_BOSS_R = 0.011
CARRIAGE_BOSS_H = 0.008
CARRIAGE_BOSS_SPACING = 0.050

GUIDE_SEAT_Z = BASE_T + PEDESTAL_H + SADDLE_H
SLIDE_LOWER = -0.135
SLIDE_UPPER = 0.135


def _build_pedestal_shape() -> cq.Workplane:
    base_plate = cq.Workplane("XY").box(
        BASE_LEN,
        BASE_W,
        BASE_T,
        centered=(True, True, False),
    )

    pedestal_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-PEDESTAL_CORE_LEN / 2.0, BASE_T),
                (PEDESTAL_CORE_LEN / 2.0, BASE_T),
                (PEDESTAL_CORE_LEN / 2.0, BASE_T + PEDESTAL_H * 0.70),
                (SADDLE_LEN / 2.0, BASE_T + PEDESTAL_H * 0.93),
                (SADDLE_LEN / 2.0, GUIDE_SEAT_Z),
                (-SADDLE_LEN / 2.0, GUIDE_SEAT_Z),
                (-SADDLE_LEN / 2.0, BASE_T + PEDESTAL_H * 0.93),
                (-PEDESTAL_CORE_LEN / 2.0, BASE_T + PEDESTAL_H * 0.70),
            ]
        )
        .close()
        .extrude(PEDESTAL_W / 2.0, both=True)
    )

    return base_plate.union(pedestal_profile)


def _build_guide_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(GUIDE_LEN, GUIDE_W, GUIDE_H).edges("|X").fillet(0.0018)


def _build_carriage_shape() -> cq.Workplane:
    top_plate = cq.Workplane("XY").box(
        CARRIAGE_LEN,
        CARRIAGE_W,
        CARRIAGE_TOP_T,
        centered=(True, True, False),
    )

    skirt_length = CARRIAGE_LEN * 0.92
    skirt_y = (CARRIAGE_W / 2.0) - (CARRIAGE_SKIRT_T / 2.0)
    left_skirt = (
        cq.Workplane("XY")
        .box(
            skirt_length,
            CARRIAGE_SKIRT_T,
            CARRIAGE_SKIRT_DROP,
            centered=(True, True, False),
        )
        .translate((0.0, skirt_y, -CARRIAGE_SKIRT_DROP))
    )
    right_skirt = (
        cq.Workplane("XY")
        .box(
            skirt_length,
            CARRIAGE_SKIRT_T,
            CARRIAGE_SKIRT_DROP,
            centered=(True, True, False),
        )
        .translate((0.0, -skirt_y, -CARRIAGE_SKIRT_DROP))
    )

    bosses = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-CARRIAGE_BOSS_SPACING / 2.0, 0.0),
                (CARRIAGE_BOSS_SPACING / 2.0, 0.0),
            ]
        )
        .circle(CARRIAGE_BOSS_R)
        .extrude(CARRIAGE_BOSS_H)
        .translate((0.0, 0.0, CARRIAGE_TOP_T))
    )

    return top_plate.union(left_skirt).union(right_skirt).union(bosses)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="raised_guideway_slide")

    model.material("pedestal_gray", rgba=(0.31, 0.33, 0.36, 1.0))
    model.material("guide_steel", rgba=(0.71, 0.73, 0.76, 1.0))
    model.material("carriage_orange", rgba=(0.84, 0.42, 0.12, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_build_pedestal_shape(), "pedestal_support"),
        material="pedestal_gray",
        name="pedestal_body",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_W, GUIDE_SEAT_Z)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_SEAT_Z / 2.0)),
    )

    guide = model.part("guide")
    guide.visual(
        mesh_from_cadquery(_build_guide_shape(), "elevated_guide"),
        material="guide_steel",
        name="guide_body",
    )
    guide.inertial = Inertial.from_geometry(
        Box((GUIDE_LEN, GUIDE_W, GUIDE_H)),
        mass=1.4,
        origin=Origin(),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "guide_carriage"),
        material="carriage_orange",
        name="carriage_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LEN, CARRIAGE_W, CARRIAGE_TOP_T + CARRIAGE_SKIRT_DROP + CARRIAGE_BOSS_H)),
        mass=0.8,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (
                    CARRIAGE_TOP_T
                    + CARRIAGE_BOSS_H
                    - CARRIAGE_SKIRT_DROP
                )
                / 2.0,
            )
        ),
    )

    model.articulation(
        "pedestal_to_guide",
        ArticulationType.FIXED,
        parent=pedestal,
        child=guide,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_SEAT_Z + (GUIDE_H / 2.0))),
    )
    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_H / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
            effort=120.0,
            velocity=0.35,
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
    pedestal = object_model.get_part("pedestal")
    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("guide_to_carriage")

    lower = slide.motion_limits.lower if slide.motion_limits is not None else None
    upper = slide.motion_limits.upper if slide.motion_limits is not None else None

    ctx.expect_gap(
        guide,
        pedestal,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="guide seats flush on pedestal",
    )
    ctx.expect_contact(
        carriage,
        guide,
        contact_tol=0.0005,
        name="carriage rides on top of guide",
    )
    ctx.expect_overlap(
        carriage,
        guide,
        axes="x",
        min_overlap=0.10,
        name="carriage substantially overlaps guide at rest",
    )
    ctx.expect_origin_distance(
        carriage,
        guide,
        axes="y",
        max_dist=0.001,
        name="carriage stays centered laterally over guide",
    )

    if lower is not None and upper is not None:
        with ctx.pose({slide: lower}):
            low_pos = ctx.part_world_position(carriage)
            ctx.expect_overlap(
                carriage,
                guide,
                axes="x",
                min_overlap=0.07,
                name="carriage remains captured on guide at negative travel",
            )

        with ctx.pose({slide: upper}):
            high_pos = ctx.part_world_position(carriage)
            ctx.expect_overlap(
                carriage,
                guide,
                axes="x",
                min_overlap=0.07,
                name="carriage remains captured on guide at positive travel",
            )
            ctx.expect_contact(
                carriage,
                guide,
                contact_tol=0.0005,
                name="carriage stays seated at positive travel",
            )

        ctx.check(
            "positive slide motion moves carriage along guide axis",
            low_pos is not None
            and high_pos is not None
            and high_pos[0] > low_pos[0] + 0.20,
            details=f"low={low_pos}, high={high_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
