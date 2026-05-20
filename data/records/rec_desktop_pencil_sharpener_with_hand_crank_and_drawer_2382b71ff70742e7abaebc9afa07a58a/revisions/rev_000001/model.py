from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOUSING_LENGTH = 0.115
HOUSING_WIDTH = 0.082
HOUSING_HEIGHT = 0.096
WALL_THICKNESS = 0.0035

TRAY_TRAVEL = 0.028
TRAY_JOINT_X = (HOUSING_LENGTH / 2.0) + 0.003
TRAY_JOINT_Z = 0.0168

SELECTOR_TRAVEL = 0.010
SELECTOR_JOINT_X = (HOUSING_LENGTH / 2.0) + 0.003
SELECTOR_JOINT_Y = 0.018
SELECTOR_JOINT_Z = 0.053

CRANK_JOINT_X = -0.041
CRANK_JOINT_Y = HOUSING_WIDTH / 2.0
CRANK_JOINT_Z = 0.064


def _housing_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(HOUSING_LENGTH, HOUSING_WIDTH, HOUSING_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .faces(">Z")
        .edges()
        .fillet(0.008)
    )

    inner = (
        cq.Workplane("XY", origin=(0.0, 0.0, WALL_THICKNESS))
        .box(
            HOUSING_LENGTH - (2.0 * WALL_THICKNESS),
            HOUSING_WIDTH - (2.0 * WALL_THICKNESS),
            HOUSING_HEIGHT - WALL_THICKNESS - 0.006,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.008)
        .faces(">Z")
        .edges()
        .fillet(0.005)
    )

    tray_pocket = cq.Workplane("XY", origin=(0.030, 0.0, 0.005)).box(
        0.066,
        0.060,
        0.024,
        centered=(True, True, False),
    )
    tray_bay_relief = cq.Workplane("XY", origin=(0.028, 0.0, 0.0)).box(
        0.070,
        0.062,
        0.030,
        centered=(True, True, False),
    )
    entry_boss = cq.Workplane("YZ", origin=(HOUSING_LENGTH / 2.0, -0.008, 0.061)).circle(0.012).extrude(0.006)
    selector_guide = cq.Workplane("XY", origin=(SELECTOR_JOINT_X - 0.0015, SELECTOR_JOINT_Y, 0.060)).box(
        0.003,
        0.008,
        0.034,
        centered=(True, True, True),
    )
    entry_hole = cq.Workplane("YZ", origin=((HOUSING_LENGTH / 2.0) + 0.004, -0.008, 0.061)).circle(0.0045).extrude(-0.038)

    housing = outer.union(entry_boss).union(selector_guide)
    housing = housing.cut(inner).cut(tray_pocket).cut(tray_bay_relief).cut(entry_hole)
    return housing


def _tray_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.045, 0.050, 0.018, centered=(True, True, True)).translate((-0.0225, 0.0, 0.0))
    inner = cq.Workplane("XY").box(0.039, 0.044, 0.014, centered=(True, True, True)).translate((-0.0245, 0.0, 0.0035))
    front_lip = cq.Workplane("XY").box(0.008, 0.066, 0.028, centered=(True, True, True)).translate((0.002, 0.0, 0.0))
    finger_scoop = cq.Workplane("YZ", origin=(0.006, 0.0, 0.0025)).circle(0.010).extrude(-0.006)
    return outer.union(front_lip).cut(inner).cut(finger_scoop)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_pencil_sharpener")

    shell_color = model.material("shell_color", rgba=(0.84, 0.85, 0.80, 1.0))
    tray_color = model.material("tray_color", rgba=(0.16, 0.17, 0.18, 1.0))
    metal_color = model.material("metal_color", rgba=(0.63, 0.65, 0.68, 1.0))
    control_color = model.material("control_color", rgba=(0.10, 0.11, 0.12, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "housing"),
        material=shell_color,
        name="housing_shell",
    )
    housing.visual(
        Box((0.002, 0.005, 0.024)),
        origin=Origin(xyz=((HOUSING_LENGTH / 2.0), 0.0305, TRAY_JOINT_Z)),
        material=shell_color,
        name="tray_stop_0",
    )
    housing.visual(
        Box((0.002, 0.005, 0.024)),
        origin=Origin(xyz=((HOUSING_LENGTH / 2.0), -0.0305, TRAY_JOINT_Z)),
        material=shell_color,
        name="tray_stop_1",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shape(), "tray"),
        material=tray_color,
        name="tray_bin",
    )

    mode_selector = model.part("mode_selector")
    mode_selector.visual(
        Box((0.004, 0.014, 0.012)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=control_color,
        name="selector_pad",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0058, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_color,
        name="hub",
    )
    crank.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_color,
        name="shaft",
    )
    crank.visual(
        Cylinder(radius=0.0028, length=0.024),
        origin=Origin(xyz=(-0.012, 0.020, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_color,
        name="arm",
    )
    crank.visual(
        Cylinder(radius=0.0042, length=0.020),
        origin=Origin(xyz=(-0.024, 0.020, 0.010)),
        material=control_color,
        name="grip",
    )
    crank.visual(
        Sphere(radius=0.0042),
        origin=Origin(xyz=(-0.024, 0.020, 0.020)),
        material=control_color,
        name="grip_cap",
    )

    model.articulation(
        "housing_to_tray",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=tray,
        origin=Origin(xyz=(TRAY_JOINT_X, 0.0, TRAY_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.12, lower=0.0, upper=TRAY_TRAVEL),
    )
    model.articulation(
        "housing_to_mode_selector",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=mode_selector,
        origin=Origin(xyz=(SELECTOR_JOINT_X, SELECTOR_JOINT_Y, SELECTOR_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=SELECTOR_TRAVEL),
    )
    model.articulation(
        "housing_to_crank",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crank,
        origin=Origin(xyz=(CRANK_JOINT_X, CRANK_JOINT_Y, CRANK_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    tray = object_model.get_part("tray")
    mode_selector = object_model.get_part("mode_selector")
    crank = object_model.get_part("crank")

    tray_slide = object_model.get_articulation("housing_to_tray")
    selector_slide = object_model.get_articulation("housing_to_mode_selector")
    crank_spin = object_model.get_articulation("housing_to_crank")

    tray_rest = ctx.part_world_position(tray)
    selector_rest = ctx.part_world_position(mode_selector)
    crank_grip_rest = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))

    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        ctx.expect_overlap(
            tray,
            housing,
            axes="y",
            min_overlap=0.040,
            name="tray stays centered in the housing opening",
        )
        ctx.expect_overlap(
            tray,
            housing,
            axes="z",
            min_overlap=0.015,
            name="tray stays at the lower front band of the housing",
        )
        ctx.expect_overlap(
            tray,
            housing,
            axes="x",
            min_overlap=0.012,
            name="tray keeps retained insertion at full extension",
        )
        tray_extended = ctx.part_world_position(tray)

    with ctx.pose({selector_slide: selector_slide.motion_limits.upper}):
        selector_high = ctx.part_world_position(mode_selector)
        ctx.expect_overlap(
            mode_selector,
            housing,
            axes="xy",
            min_overlap=0.003,
            name="mode selector remains guided beside the entry opening",
        )

    with ctx.pose({crank_spin: math.pi / 2.0}):
        crank_grip_quarter_turn = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))

    ctx.check(
        "tray slides outward from the lower front",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[0] > tray_rest[0] + 0.020,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )
    ctx.check(
        "mode selector slides upward along its short guide",
        selector_rest is not None
        and selector_high is not None
        and selector_high[2] > selector_rest[2] + 0.008,
        details=f"rest={selector_rest}, high={selector_high}",
    )
    ctx.check(
        "crank grip moves through a visible circular sweep",
        crank_grip_rest is not None
        and crank_grip_quarter_turn is not None
        and abs(crank_grip_quarter_turn[0] - crank_grip_rest[0]) > 0.010
        and abs(crank_grip_quarter_turn[2] - crank_grip_rest[2]) > 0.010,
        details=f"rest={crank_grip_rest}, quarter_turn={crank_grip_quarter_turn}",
    )

    return ctx.report()


object_model = build_object_model()
