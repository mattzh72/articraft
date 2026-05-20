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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_X = 0.28
BASE_Y = 0.24
BASE_T = 0.028
COLUMN_R = 0.052
COLUMN_H = 0.095
TURRET_R = 0.078
TURRET_H = 0.032

ARM_HUB_R = 0.068
ARM_HUB_H = 0.050
ARM_BEAM_START = 0.035
ARM_BEAM_L = 0.600
ARM_BEAM_W = 0.095
ARM_BEAM_H = 0.052

TRUCK_L = 0.125
TRUCK_SIDE_T = 0.014
TRUCK_SIDE_H = 0.062
TRUCK_CLEAR_Y = 0.004
TRUCK_TOP_T = 0.028
TRUCK_FACE_T = 0.014
TRUCK_FACE_H = 0.070
TRUCK_W = ARM_BEAM_W + 0.040
TRUCK_TOP_BLOCK_Z = (ARM_BEAM_H / 2.0) + (TRUCK_TOP_T / 2.0)
TRUCK_CHEEK_L = TRUCK_L - 0.020
TRUCK_CHEEK_X = 0.010 + (TRUCK_CHEEK_L / 2.0)
TRUCK_CHEEK_Y = (ARM_BEAM_W / 2.0) + TRUCK_CLEAR_Y + (TRUCK_SIDE_T / 2.0)
TRUCK_CHEEK_Z = 0.016
TRUCK_FACE_Z = (ARM_BEAM_H / 2.0) + (TRUCK_FACE_H / 2.0)
TRUCK_BOTTOM_Z = min(TRUCK_TOP_BLOCK_Z - (TRUCK_TOP_T / 2.0), TRUCK_CHEEK_Z - (TRUCK_SIDE_H / 2.0))
TRUCK_TOP_Z = max(TRUCK_TOP_BLOCK_Z + (TRUCK_TOP_T / 2.0), TRUCK_FACE_Z + (TRUCK_FACE_H / 2.0))
TRUCK_H = TRUCK_TOP_Z - TRUCK_BOTTOM_Z
TRUCK_COM_Z = (TRUCK_TOP_Z + TRUCK_BOTTOM_Z) / 2.0

TRUCK_HOME_X = 0.140
TRUCK_TRAVEL = 0.340

PEDESTAL_TOP_Z = BASE_T + COLUMN_H + TURRET_H
BASE_AXIS_Z = PEDESTAL_TOP_Z + (ARM_HUB_H / 2.0)


def _pedestal_shape() -> cq.Workplane:
    hole_points = (
        (-0.095, -0.075),
        (-0.095, 0.075),
        (0.095, -0.075),
        (0.095, 0.075),
    )

    base = (
        cq.Workplane("XY")
        .box(BASE_X, BASE_Y, BASE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .faces(">Z")
        .workplane()
        .pushPoints(hole_points)
        .hole(0.016)
    )

    column = cq.Workplane("XY").circle(COLUMN_R).extrude(COLUMN_H).translate((0.0, 0.0, BASE_T))
    shoulder = (
        cq.Workplane("XY")
        .circle(0.066)
        .extrude(0.016)
        .translate((0.0, 0.0, BASE_T + COLUMN_H - 0.006))
    )
    turret = cq.Workplane("XY").circle(TURRET_R).extrude(TURRET_H).translate((0.0, 0.0, BASE_T + COLUMN_H))

    return base.union(column).union(shoulder).union(turret)


def _arm_hub_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(ARM_HUB_R)
        .extrude(ARM_HUB_H)
        .translate((0.0, 0.0, -(ARM_HUB_H / 2.0)))
    )
    cap = (
        cq.Workplane("XY")
        .circle(0.050)
        .extrude(0.012)
        .translate((0.0, 0.0, (ARM_HUB_H / 2.0) - 0.004))
    )
    return hub.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_radial_beam_slide")

    model.material("pedestal_gray", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("arm_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("truck_orange", rgba=(0.88, 0.47, 0.13, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shape(), "pedestal_body"),
        material="pedestal_gray",
        name="pedestal_body",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((BASE_X, BASE_Y, BASE_AXIS_Z)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_AXIS_Z / 2.0)),
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_hub_shape(), "arm_hub"),
        material="arm_aluminum",
        name="arm_hub",
    )
    arm.visual(
        Box((ARM_BEAM_L, ARM_BEAM_W, ARM_BEAM_H)),
        origin=Origin(xyz=(ARM_BEAM_START + (ARM_BEAM_L / 2.0), 0.0, 0.0)),
        material="arm_aluminum",
        name="arm_beam",
    )
    arm.inertial = Inertial.from_geometry(
        Box((ARM_BEAM_START + ARM_BEAM_L + ARM_HUB_R, 0.15, ARM_HUB_H)),
        mass=8.5,
        origin=Origin(xyz=(0.30, 0.0, 0.0)),
    )

    truck = model.part("truck")
    truck.visual(
        Box((TRUCK_L, TRUCK_W, TRUCK_TOP_T)),
        origin=Origin(xyz=(TRUCK_L / 2.0, 0.0, TRUCK_TOP_BLOCK_Z)),
        material="truck_orange",
        name="truck_body",
    )
    truck.visual(
        Box((TRUCK_CHEEK_L, TRUCK_SIDE_T, TRUCK_SIDE_H)),
        origin=Origin(xyz=(TRUCK_CHEEK_X, TRUCK_CHEEK_Y, TRUCK_CHEEK_Z)),
        material="truck_orange",
        name="truck_left_cheek",
    )
    truck.visual(
        Box((TRUCK_CHEEK_L, TRUCK_SIDE_T, TRUCK_SIDE_H)),
        origin=Origin(xyz=(TRUCK_CHEEK_X, -TRUCK_CHEEK_Y, TRUCK_CHEEK_Z)),
        material="truck_orange",
        name="truck_right_cheek",
    )
    truck.visual(
        Box((TRUCK_FACE_T, TRUCK_W, TRUCK_FACE_H)),
        origin=Origin(xyz=(TRUCK_L - (TRUCK_FACE_T / 2.0), 0.0, TRUCK_FACE_Z)),
        material="truck_orange",
        name="truck_face",
    )
    truck.inertial = Inertial.from_geometry(
        Box((TRUCK_L, TRUCK_W, TRUCK_H)),
        mass=2.3,
        origin=Origin(xyz=(TRUCK_L / 2.0, 0.0, TRUCK_COM_Z)),
    )

    model.articulation(
        "pedestal_to_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, BASE_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.6, upper=2.6, effort=140.0, velocity=1.2),
    )
    model.articulation(
        "arm_to_truck",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=truck,
        origin=Origin(xyz=(TRUCK_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRUCK_TRAVEL, effort=80.0, velocity=0.30),
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
    arm = object_model.get_part("arm")
    truck = object_model.get_part("truck")
    base_yaw = object_model.get_articulation("pedestal_to_arm")
    truck_slide = object_model.get_articulation("arm_to_truck")

    with ctx.pose({base_yaw: 0.0, truck_slide: 0.0}):
        ctx.expect_gap(
            arm,
            pedestal,
            axis="z",
            positive_elem="arm_hub",
            negative_elem="pedestal_body",
            max_gap=0.001,
            max_penetration=0.0,
            name="arm hub seats on pedestal turntable",
        )
        ctx.expect_overlap(
            arm,
            pedestal,
            axes="xy",
            elem_a="arm_hub",
            elem_b="pedestal_body",
            min_overlap=0.12,
            name="arm hub footprint stays centered on pedestal",
        )
        ctx.expect_origin_distance(
            truck,
            arm,
            axes="yz",
            max_dist=0.001,
            name="truck stays aligned to arm centerline at home",
        )
        ctx.expect_overlap(
            truck,
            arm,
            axes="x",
            elem_a="truck_body",
            elem_b="arm_beam",
            min_overlap=0.09,
            name="truck remains inserted on arm beam at home",
        )

        home_pos = ctx.part_world_position(truck)

    with ctx.pose({base_yaw: 0.0, truck_slide: TRUCK_TRAVEL}):
        ctx.expect_origin_distance(
            truck,
            arm,
            axes="yz",
            max_dist=0.001,
            name="truck stays aligned to arm centerline when extended",
        )
        ctx.expect_overlap(
            truck,
            arm,
            axes="x",
            elem_a="truck_body",
            elem_b="arm_beam",
            min_overlap=0.09,
            name="truck retains beam engagement at full extension",
        )

        extended_pos = ctx.part_world_position(truck)

    with ctx.pose({base_yaw: 1.0, truck_slide: 0.0}):
        swept_pos = ctx.part_world_position(truck)

    ctx.check(
        "truck extends radially outward",
        home_pos is not None and extended_pos is not None and extended_pos[0] > home_pos[0] + 0.25,
        details=f"home={home_pos}, extended={extended_pos}",
    )
    ctx.check(
        "arm rotation sweeps truck counterclockwise about pedestal",
        home_pos is not None
        and swept_pos is not None
        and swept_pos[1] > home_pos[1] + 0.09
        and swept_pos[0] < home_pos[0] - 0.03,
        details=f"home={home_pos}, swept={swept_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
