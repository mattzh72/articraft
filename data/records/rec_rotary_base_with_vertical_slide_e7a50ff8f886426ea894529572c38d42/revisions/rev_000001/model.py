from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import hypot

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


BASE_PLATE_X = 0.34
BASE_PLATE_Y = 0.26
BASE_PLATE_H = 0.018
BASE_PEDESTAL_X = 0.19
BASE_PEDESTAL_Y = 0.15
BASE_PEDESTAL_H = 0.048
BEARING_RING_OUTER = 0.078
BEARING_RING_INNER = 0.048
BEARING_RING_H = 0.014
REVOLUTE_Z = BASE_PLATE_H + BASE_PEDESTAL_H + BEARING_RING_H

PLATTER_R = 0.086
PLATTER_H = 0.016
GUIDE_X = 0.05
GUIDE_FOOT_X = 0.104
GUIDE_FOOT_Y = 0.066
GUIDE_FOOT_H = 0.040
GUIDE_MAST_X = 0.072
GUIDE_MAST_Y = 0.050
GUIDE_MAST_H = 0.250
GUIDE_CAP_X = 0.086
GUIDE_CAP_Y = 0.058
GUIDE_CAP_H = 0.012
GUIDE_RAIL_X = 0.036
GUIDE_RAIL_Y = 0.010
GUIDE_RAIL_Z = 0.220
GUIDE_BASE_Z = PLATTER_H
GUIDE_MAST_Z = GUIDE_BASE_Z + GUIDE_FOOT_H
GUIDE_TOP_Z = GUIDE_MAST_Z + GUIDE_MAST_H + GUIDE_CAP_H

CARRIAGE_OUTER_X = 0.112
CARRIAGE_OUTER_Y = 0.052
CARRIAGE_OUTER_Z = 0.090
CARRIAGE_POCKET_X = 0.078
CARRIAGE_POCKET_Y = GUIDE_MAST_Y
TOOL_PLATE_X = 0.094
TOOL_PLATE_Y = 0.018
TOOL_PLATE_Z = 0.070
TOOL_PAD_X = 0.064
TOOL_PAD_Y = 0.026
TOOL_PAD_Z = 0.014

LIFT_ORIGIN_Z = 0.090
LIFT_TRAVEL = 0.120


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name, tolerance=0.0007, angular_tolerance=0.08)


def _base_body_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_PLATE_X, BASE_PLATE_Y, BASE_PLATE_H, centered=(True, True, False)
    )
    plate = plate.edges("|Z").fillet(0.012)

    pedestal = (
        cq.Workplane("XY")
        .box(
            BASE_PEDESTAL_X,
            BASE_PEDESTAL_Y,
            BASE_PEDESTAL_H,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BASE_PLATE_H))
        .edges("|Z")
        .fillet(0.010)
    )

    body = plate.union(pedestal)
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.12, -0.085),
                (-0.12, 0.085),
                (0.12, -0.085),
                (0.12, 0.085),
            ]
        )
        .hole(0.016, depth=BASE_PLATE_H)
    )
    body = body.cut(
        cq.Workplane("XY")
        .box(0.082, 0.020, 0.024, centered=(True, True, False))
        .translate((0.0, BASE_PEDESTAL_Y / 2.0 - 0.010, BASE_PLATE_H))
    )
    return body


def _bearing_ring_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(BEARING_RING_OUTER).circle(BEARING_RING_INNER).extrude(
        BEARING_RING_H
    )


def _stage_platter_shape() -> cq.Workplane:
    platter = cq.Workplane("XY").circle(PLATTER_R).extrude(PLATTER_H)
    platter = platter.cut(
        cq.Workplane("XY").circle(0.040).extrude(0.006).translate((0.0, 0.0, 0.010))
    )
    return platter


def _guide_tower_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(GUIDE_FOOT_X, GUIDE_FOOT_Y, GUIDE_FOOT_H, centered=(True, True, False))
        .translate((GUIDE_X, 0.0, GUIDE_BASE_Z))
    )
    mast = (
        cq.Workplane("XY")
        .box(GUIDE_MAST_X, GUIDE_MAST_Y, GUIDE_MAST_H, centered=(True, True, False))
        .translate((GUIDE_X, 0.0, GUIDE_MAST_Z))
    )
    cap = (
        cq.Workplane("XY")
        .box(GUIDE_CAP_X, GUIDE_CAP_Y, GUIDE_CAP_H, centered=(True, True, False))
        .translate((GUIDE_X, 0.0, GUIDE_MAST_Z + GUIDE_MAST_H))
    )

    tower = foot.union(mast).union(cap)
    tower = tower.cut(
        cq.Workplane("XY")
        .box(0.032, 0.020, 0.180, centered=(True, True, False))
        .translate((GUIDE_X, 0.015, GUIDE_MAST_Z + 0.032))
    )
    tower = tower.cut(
        cq.Workplane("XZ")
        .center(GUIDE_X - 0.030, GUIDE_BASE_Z + 0.010)
        .rect(0.020, 0.020)
        .extrude(0.090)
        .translate((0.0, -0.045, 0.0))
    )
    return tower


def _carriage_sleeve_shape() -> cq.Workplane:
    sleeve = cq.Workplane("XY").box(
        CARRIAGE_OUTER_X,
        CARRIAGE_OUTER_Y,
        CARRIAGE_OUTER_Z,
        centered=(True, True, False),
    )
    return sleeve.edges("|Z").fillet(0.005)


def _tool_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(TOOL_PLATE_X, TOOL_PLATE_Y, TOOL_PLATE_Z, centered=(True, True, False))
        .translate((0.0, CARRIAGE_OUTER_Y / 2.0 + TOOL_PLATE_Y / 2.0, 0.010))
    )
    pad = (
        cq.Workplane("XY")
        .box(TOOL_PAD_X, TOOL_PAD_Y, TOOL_PAD_Z, centered=(True, True, False))
        .translate((0.0, CARRIAGE_OUTER_Y / 2.0 + TOOL_PAD_Y / 2.0 + 0.012, 0.022))
    )
    return plate.union(pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turntable_lift_module")

    model.material("base_paint", rgba=(0.24, 0.25, 0.27, 1.0))
    model.material("bearing_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("guide_gray", rgba=(0.55, 0.58, 0.61, 1.0))
    model.material("carriage_orange", rgba=(0.88, 0.48, 0.11, 1.0))
    model.material("dark_tool", rgba=(0.15, 0.16, 0.18, 1.0))

    base = model.part("base")
    base.visual(_mesh(_base_body_shape(), "base_body"), material="base_paint", name="base_body")
    base.visual(
        _mesh(_bearing_ring_shape(), "bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_H + BASE_PEDESTAL_H)),
        material="bearing_steel",
        name="bearing_ring",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_PLATE_X, BASE_PLATE_Y, REVOLUTE_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, REVOLUTE_Z / 2.0)),
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        _mesh(_stage_platter_shape(), "stage_platter"),
        material="bearing_steel",
        name="stage_platter",
    )
    rotary_stage.visual(
        _mesh(_guide_tower_shape(), "guide_tower"),
        material="guide_gray",
        name="guide_tower",
    )
    rotary_stage.visual(
        Box((GUIDE_RAIL_X, GUIDE_RAIL_Y, GUIDE_RAIL_Z)),
        origin=Origin(
            xyz=(
                GUIDE_X,
                GUIDE_MAST_Y / 2.0 + GUIDE_RAIL_Y / 2.0,
                GUIDE_MAST_Z + 0.018 + GUIDE_RAIL_Z / 2.0,
            )
        ),
        material="bearing_steel",
        name="guide_rail",
    )
    rotary_stage.inertial = Inertial.from_geometry(
        Box((0.22, 0.12, GUIDE_TOP_Z)),
        mass=9.5,
        origin=Origin(xyz=(GUIDE_X * 0.55, 0.0, GUIDE_TOP_Z / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        _mesh(_carriage_sleeve_shape(), "carriage_sleeve"),
        material="carriage_orange",
        name="carriage_sleeve",
    )
    carriage.visual(
        _mesh(_tool_plate_shape(), "tool_plate"),
        material="dark_tool",
        name="tool_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.13, 0.11, CARRIAGE_OUTER_Z)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.012, CARRIAGE_OUTER_Z / 2.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, REVOLUTE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.6,
            upper=2.6,
            effort=40.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "carriage_lift",
        ArticulationType.PRISMATIC,
        parent=rotary_stage,
        child=carriage,
        origin=Origin(
            xyz=(GUIDE_X, GUIDE_MAST_Y / 2.0 + GUIDE_RAIL_Y + CARRIAGE_OUTER_Y / 2.0, LIFT_ORIGIN_Z)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=180.0,
            velocity=0.28,
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
    rotary_stage = object_model.get_part("rotary_stage")
    carriage = object_model.get_part("carriage")
    base_yaw = object_model.get_articulation("base_yaw")
    carriage_lift = object_model.get_articulation("carriage_lift")

    ctx.check(
        "all module parts are present",
        all(part is not None for part in (base, rotary_stage, carriage)),
    )

    with ctx.pose({base_yaw: 0.0, carriage_lift: 0.0}):
        ctx.expect_gap(
            rotary_stage,
            base,
            axis="z",
            positive_elem="stage_platter",
            negative_elem="bearing_ring",
            max_gap=0.001,
            max_penetration=0.0,
            name="rotary stage seats on the bearing ring",
        )
        ctx.expect_overlap(
            carriage,
            rotary_stage,
            axes="x",
            elem_a="carriage_sleeve",
            elem_b="guide_rail",
            min_overlap=0.030,
            name="carriage block overlaps the guide rail in width",
        )
        ctx.expect_contact(
            carriage,
            rotary_stage,
            elem_a="carriage_sleeve",
            elem_b="guide_rail",
            name="carriage block rides on the guide rail at rest",
        )
        ctx.expect_overlap(
            carriage,
            rotary_stage,
            axes="z",
            elem_a="carriage_sleeve",
            elem_b="guide_rail",
            min_overlap=0.085,
            name="carriage block remains engaged on the guide rail at rest",
        )

        rest_pos = ctx.part_world_position(carriage)

    with ctx.pose({base_yaw: 0.0, carriage_lift: LIFT_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            rotary_stage,
            axes="x",
            elem_a="carriage_sleeve",
            elem_b="guide_rail",
            min_overlap=0.030,
            name="carriage block overlaps the guide rail in width at full lift",
        )
        ctx.expect_contact(
            carriage,
            rotary_stage,
            elem_a="carriage_sleeve",
            elem_b="guide_rail",
            name="carriage block rides on the guide rail at full lift",
        )
        ctx.expect_overlap(
            carriage,
            rotary_stage,
            axes="z",
            elem_a="carriage_sleeve",
            elem_b="guide_rail",
            min_overlap=0.080,
            name="carriage block retains insertion at full lift",
        )
        lifted_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage moves upward without lateral drift",
        rest_pos is not None
        and lifted_pos is not None
        and lifted_pos[2] > rest_pos[2] + 0.10
        and abs(lifted_pos[0] - rest_pos[0]) < 0.001
        and abs(lifted_pos[1] - rest_pos[1]) < 0.001,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    with ctx.pose({base_yaw: 1.1, carriage_lift: 0.0}):
        yawed_pos = ctx.part_world_position(carriage)

    ctx.check(
        "turntable yaw carries the guide and carriage around the vertical axis",
        rest_pos is not None
        and yawed_pos is not None
        and abs(yawed_pos[2] - rest_pos[2]) < 0.001
        and hypot(yawed_pos[0] - rest_pos[0], yawed_pos[1] - rest_pos[1]) > 0.045,
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
