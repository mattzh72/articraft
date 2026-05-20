from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_L = 0.74
BASE_W = 0.24
BASE_T = 0.018

SUPPORT_L = 0.080
SUPPORT_W = 0.180
SUPPORT_H = 0.035
SUPPORT_X = 0.265
SUPPORT_SINK = 0.001
SUPPORT_Z = BASE_T - SUPPORT_SINK + (SUPPORT_H / 2.0)

RAIL_LEN = 0.610
RAIL_R = 0.012
RAIL_SPAN = 0.140
RAIL_EMBED = 0.004
RAIL_Z = (BASE_T - SUPPORT_SINK + SUPPORT_H) + RAIL_R - RAIL_EMBED

CARR_L = 0.110
CARR_W = 0.176
DECK_T = 0.018
DECK_Z = 0.035
BRIDGE_L = 0.082
BRIDGE_W = 0.132
BRIDGE_H = 0.028
BRIDGE_Z = 0.014
HOUSING_R = 0.026
BORE_R = 0.0135
TRAVEL = 0.160


def _support_block_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(SUPPORT_L, SUPPORT_W, SUPPORT_H)
        .edges("|Z")
        .fillet(0.010)
        .faces(">Z")
        .edges("|X")
        .fillet(0.004)
    )


def _carriage_shape() -> cq.Workplane:
    left_housing = (
        cq.Workplane("YZ")
        .center(-RAIL_SPAN / 2.0, 0.0)
        .circle(HOUSING_R)
        .extrude(CARR_L / 2.0, both=True)
    )
    right_housing = (
        cq.Workplane("YZ")
        .center(RAIL_SPAN / 2.0, 0.0)
        .circle(HOUSING_R)
        .extrude(CARR_L / 2.0, both=True)
    )

    body = (
        left_housing.union(right_housing)
        .union(
            cq.Workplane("XY").box(
                BRIDGE_L,
                BRIDGE_W,
                BRIDGE_H,
            ).translate((0.0, 0.0, BRIDGE_Z))
        )
        .union(
            cq.Workplane("XY").box(
                CARR_L,
                CARR_W,
                DECK_T,
            ).translate((0.0, 0.0, DECK_Z))
        )
    )

    rail_bore_left = (
        cq.Workplane("YZ")
        .center(-RAIL_SPAN / 2.0, 0.0)
        .circle(BORE_R)
        .extrude((CARR_L / 2.0) + 0.010, both=True)
    )
    rail_bore_right = (
        cq.Workplane("YZ")
        .center(RAIL_SPAN / 2.0, 0.0)
        .circle(BORE_R)
        .extrude((CARR_L / 2.0) + 0.010, both=True)
    )

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.032, -0.050),
                (-0.032, 0.050),
                (0.032, -0.050),
                (0.032, 0.050),
            ]
        )
        .circle(0.0045)
        .extrude(0.030)
        .translate((0.0, 0.0, 0.020))
    )

    return (
        body.cut(rail_bore_left)
        .cut(rail_bore_right)
        .cut(mount_holes)
        .edges("|X and >Z")
        .fillet(0.004)
        .edges("|Z and (>Y or <Y)")
        .fillet(0.003)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_rail_carriage_slider")

    model.material("base_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("rail_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("carriage_orange", rgba=(0.88, 0.48, 0.14, 1.0))
    model.material("guide_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_L, BASE_W, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="base_gray",
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_support_block_shape(), "left_support_block"),
        origin=Origin(xyz=(-SUPPORT_X, 0.0, SUPPORT_Z)),
        material="base_gray",
        name="left_support",
    )
    base.visual(
        mesh_from_cadquery(_support_block_shape(), "right_support_block"),
        origin=Origin(xyz=(SUPPORT_X, 0.0, SUPPORT_Z)),
        material="base_gray",
        name="right_support",
    )
    base.visual(
        Cylinder(radius=RAIL_R, length=RAIL_LEN),
        origin=Origin(
            xyz=(0.0, -(RAIL_SPAN / 2.0), RAIL_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="rail_steel",
        name="rail_left",
    )
    base.visual(
        Cylinder(radius=RAIL_R, length=RAIL_LEN),
        origin=Origin(
            xyz=(0.0, RAIL_SPAN / 2.0, RAIL_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="rail_steel",
        name="rail_right",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, 0.100)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_body"),
        material="carriage_orange",
        name="carriage_body",
    )
    carriage.visual(
        Box((0.088, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, -(RAIL_SPAN / 2.0), 0.015)),
        material="guide_black",
        name="shoe_left",
    )
    carriage.visual(
        Box((0.088, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, RAIL_SPAN / 2.0, 0.015)),
        material="guide_black",
        name="shoe_right",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARR_L, CARR_W, 0.070)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL,
            upper=TRAVEL,
            effort=180.0,
            velocity=0.40,
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
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("base_to_carriage")

    ctx.check(
        "slider uses one x-axis prismatic joint",
        slide.articulation_type == ArticulationType.PRISMATIC and slide.axis == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            negative_elem="base_plate",
            min_gap=0.012,
            max_gap=0.022,
            name="carriage clears the grounded base plate",
        )

    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_gap(
            base,
            carriage,
            axis="x",
            positive_elem="right_support",
            negative_elem="carriage_body",
            min_gap=0.008,
            max_gap=0.020,
            name="carriage clears the right support at max extension",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="carriage_body",
            elem_b="rail_left",
            min_overlap=0.100,
            name="carriage remains engaged with the left rail at max extension",
        )
        upper_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: slide.motion_limits.lower}):
        ctx.expect_gap(
            carriage,
            base,
            axis="x",
            positive_elem="carriage_body",
            negative_elem="left_support",
            min_gap=0.008,
            max_gap=0.020,
            name="carriage clears the left support at max retraction",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="carriage_body",
            elem_b="rail_right",
            min_overlap=0.100,
            name="carriage remains engaged with the right rail at max retraction",
        )
        lower_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: 0.0}):
        rest_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along the rail direction",
        (
            rest_pos is not None
            and upper_pos is not None
            and lower_pos is not None
            and upper_pos[0] > rest_pos[0] + 0.10
            and lower_pos[0] < rest_pos[0] - 0.10
            and abs(upper_pos[1] - rest_pos[1]) < 1e-6
            and abs(upper_pos[2] - rest_pos[2]) < 1e-6
        ),
        details=f"lower={lower_pos}, rest={rest_pos}, upper={upper_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
