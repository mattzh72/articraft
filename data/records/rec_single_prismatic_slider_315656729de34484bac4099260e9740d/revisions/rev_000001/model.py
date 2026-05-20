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


BASE_L = 0.34
BASE_W = 0.14
BASE_T = 0.022

PEDESTAL_L = 0.28
PEDESTAL_W = 0.032
PEDESTAL_T = 0.012

GUIDE_L = 0.27
GUIDE_BOTTOM_W = 0.060
GUIDE_TOP_W = 0.036
GUIDE_H = 0.018
GUIDE_Z0 = BASE_T + PEDESTAL_T

SADDLE_L = 0.16
SADDLE_LOWER_W = 0.112
SADDLE_UPPER_W = 0.150
SADDLE_BOTTOM_Z = -0.006
SADDLE_MID_Z = 0.022
SADDLE_TOP_Z = 0.046
TOP_DECK_L = 0.152

SLIDE_LOWER = -0.08
SLIDE_UPPER = 0.08

SIDE_CLR = 0.0012
ROOF_CLR = 0.0010
FLOOR_CLR = 0.0006


def _dovetail_profile(bottom_width: float, top_width: float, bottom_z: float, height: float):
    top_z = bottom_z + height
    return [
        (-bottom_width / 2.0, bottom_z),
        (bottom_width / 2.0, bottom_z),
        (top_width / 2.0, top_z),
        (-top_width / 2.0, top_z),
    ]


def _build_base_platform_shape() -> cq.Workplane:
    hole_x = (BASE_L / 2.0) - 0.038
    hole_y = (BASE_W / 2.0) - 0.024

    plate = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-hole_x, -hole_y),
                (-hole_x, hole_y),
                (hole_x, -hole_y),
                (hole_x, hole_y),
            ]
        )
        .cboreHole(0.009, 0.017, 0.004)
    )

    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_L, PEDESTAL_W, PEDESTAL_T, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_T))
    )

    return plate.union(pedestal)


def _build_guide_shape() -> cq.Workplane:
    rail = (
        cq.Workplane("YZ")
        .polyline(
            _dovetail_profile(
                GUIDE_BOTTOM_W,
                GUIDE_TOP_W,
                GUIDE_Z0 - 0.0005,
                GUIDE_H + 0.0005,
            )
        )
        .close()
        .extrude(GUIDE_L / 2.0, both=True)
    )
    return rail


def _build_saddle_shape() -> cq.Workplane:
    lower_body = (
        cq.Workplane("XY")
        .box(
            SADDLE_L,
            SADDLE_LOWER_W,
            SADDLE_MID_Z - SADDLE_BOTTOM_Z,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, SADDLE_BOTTOM_Z))
    )
    top_deck = (
        cq.Workplane("XY")
        .box(
            TOP_DECK_L,
            SADDLE_UPPER_W,
            SADDLE_TOP_Z - SADDLE_MID_Z,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, SADDLE_MID_Z))
    )

    saddle = lower_body.union(top_deck)

    bore = (
        cq.Workplane("YZ")
        .polyline(
            _dovetail_profile(
                GUIDE_BOTTOM_W + (2.0 * SIDE_CLR),
                GUIDE_TOP_W + (2.0 * SIDE_CLR),
                -FLOOR_CLR,
                GUIDE_H + FLOOR_CLR + ROOF_CLR,
            )
        )
        .close()
        .extrude((SADDLE_L + 0.006) / 2.0, both=True)
    )

    saddle = saddle.cut(bore)

    underside_relief_top = 0.004
    underside_relief = (
        cq.Workplane("YZ")
        .center(0.0, (SADDLE_BOTTOM_Z + underside_relief_top) / 2.0)
        .rect(0.040, underside_relief_top - SADDLE_BOTTOM_Z)
        .extrude((SADDLE_L + 0.006) / 2.0, both=True)
    )
    saddle = saddle.cut(underside_relief)

    slot_y = 0.038
    slot_depth = 0.006
    saddle = (
        saddle.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, -slot_y), (0.0, slot_y)])
        .slot2D(0.075, 0.013, angle=0.0)
        .cutBlind(-slot_depth)
    )

    return saddle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dovetail_linear_slide")

    model.material("cast_iron", rgba=(0.30, 0.31, 0.33, 1.0))
    model.material("way_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("saddle_gray", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_platform_shape(), "slide_base_platform"),
        material="cast_iron",
        name="base_platform",
    )
    base.visual(
        mesh_from_cadquery(_build_guide_shape(), "slide_guide_rail"),
        material="way_steel",
        name="guide_rail",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, GUIDE_Z0 + GUIDE_H)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, (GUIDE_Z0 + GUIDE_H) / 2.0)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_cadquery(_build_saddle_shape(), "slide_saddle_body"),
        material="saddle_gray",
        name="saddle_body",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((SADDLE_L, SADDLE_UPPER_W, SADDLE_TOP_Z - SADDLE_BOTTOM_Z)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, (SADDLE_TOP_Z + SADDLE_BOTTOM_Z) / 2.0)),
    )

    model.articulation(
        "base_to_saddle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_Z0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
            effort=220.0,
            velocity=0.25,
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
    saddle = object_model.get_part("saddle")
    slide = object_model.get_articulation("base_to_saddle")

    ctx.allow_isolated_part(
        saddle,
        reason=(
            "The saddle is intentionally modeled with running clearance around the dovetail "
            "guide; the captured prismatic fit is represented by the guide geometry and joint."
        ),
    )

    ctx.check("base part exists", base is not None)
    ctx.check("saddle part exists", saddle is not None)
    ctx.check(
        "slide axis is guide axis",
        tuple(round(v, 6) for v in slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "slide limits are symmetric travel",
        slide.motion_limits is not None
        and slide.motion_limits.lower == SLIDE_LOWER
        and slide.motion_limits.upper == SLIDE_UPPER,
        details=f"limits={slide.motion_limits}",
    )

    ctx.expect_overlap(
        saddle,
        base,
        axes="x",
        elem_a="saddle_body",
        elem_b="guide_rail",
        min_overlap=0.15,
        name="saddle spans the guide at rest",
    )

    rest_pos = ctx.part_world_position(saddle)
    rest_saddle_aabb = ctx.part_element_world_aabb(saddle, elem="saddle_body")
    rest_guide_aabb = ctx.part_element_world_aabb(base, elem="guide_rail")
    wraparound_ok = (
        rest_saddle_aabb is not None
        and rest_guide_aabb is not None
        and rest_saddle_aabb[0][2] < rest_guide_aabb[0][2] - 0.003
        and rest_saddle_aabb[1][2] > rest_guide_aabb[1][2] + 0.012
        and abs(
            ((rest_saddle_aabb[0][1] + rest_saddle_aabb[1][1]) / 2.0)
            - ((rest_guide_aabb[0][1] + rest_guide_aabb[1][1]) / 2.0)
        )
        < 0.001
    )
    ctx.check(
        "saddle visibly wraps around the guide",
        wraparound_ok,
        details=f"saddle_aabb={rest_saddle_aabb}, guide_aabb={rest_guide_aabb}",
    )

    with ctx.pose({slide: slide.motion_limits.upper}):
        upper_pos = ctx.part_world_position(saddle)
        ctx.expect_overlap(
            saddle,
            base,
            axes="x",
            elem_a="saddle_body",
            elem_b="guide_rail",
            min_overlap=0.10,
            name="saddle retains insertion at max extension",
        )

    with ctx.pose({slide: slide.motion_limits.lower}):
        lower_pos = ctx.part_world_position(saddle)
        ctx.expect_overlap(
            saddle,
            base,
            axes="x",
            elem_a="saddle_body",
            elem_b="guide_rail",
            min_overlap=0.10,
            name="saddle retains insertion at max retraction",
        )

    ctx.check(
        "positive slide motion moves saddle in +x",
        rest_pos is not None
        and upper_pos is not None
        and lower_pos is not None
        and upper_pos[0] > rest_pos[0] + 0.05
        and lower_pos[0] < rest_pos[0] - 0.05,
        details=f"rest={rest_pos}, upper={upper_pos}, lower={lower_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
