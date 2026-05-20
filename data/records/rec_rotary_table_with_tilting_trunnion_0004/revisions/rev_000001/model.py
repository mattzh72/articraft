from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

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

BASE_RADIUS = 0.120
BASE_THICKNESS = 0.030
PEDESTAL_RADIUS = 0.042
PEDESTAL_HEIGHT = 0.050

STAGE_RADIUS = 0.090
STAGE_THICKNESS = 0.022
TRUNNION_CENTER_Z = 0.100
CHEEK_CENTER_Y = 0.054
CHEEK_THICKNESS = 0.014
CHEEK_HOLE_RADIUS = 0.0112
CHEEK_HEIGHT = TRUNNION_CENTER_Z - STAGE_THICKNESS + 0.026

WORKPLATE_RADIUS = 0.068
WORKPLATE_THICKNESS = 0.020
SHAFT_RADIUS = 0.0105
SHAFT_HALF_SPAN = CHEEK_CENTER_Y
COLLAR_RADIUS = 0.018
COLLAR_LENGTH = 0.005
FIXTURE_PAD_SIZE = (0.022, 0.006, 0.018)


def _cheek_mesh() -> object:
    profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.030, 0.000),
                (-0.030, 0.024),
                (-0.022, 0.052),
                (-0.012, CHEEK_HEIGHT - 0.006),
                (0.012, CHEEK_HEIGHT - 0.006),
                (0.022, 0.052),
                (0.030, 0.024),
                (0.030, 0.000),
            ]
        )
        .close()
        .moveTo(0.0, TRUNNION_CENTER_Z - STAGE_THICKNESS)
        .circle(CHEEK_HOLE_RADIUS)
        .extrude(CHEEK_THICKNESS)
        .translate((0.0, -CHEEK_THICKNESS / 2.0, 0.0))
    )
    return mesh_from_cadquery(profile, "cheek_side.obj", assets=ASSETS)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trunnion_table_module", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.23, 0.24, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.60, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    anodized = model.material("anodized_blue", rgba=(0.20, 0.33, 0.47, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=cast_iron,
        name="foot_shell",
    )
    base.visual(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0)),
        material=steel,
        name="pedestal_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_RADIUS * 2.0, BASE_RADIUS * 2.0, BASE_THICKNESS + PEDESTAL_HEIGHT)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + PEDESTAL_HEIGHT) / 2.0)),
    )

    stage = model.part("stage")
    stage.visual(
        Cylinder(radius=STAGE_RADIUS, length=STAGE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, STAGE_THICKNESS / 2.0)),
        material=steel,
        name="stage_platter",
    )
    cheek_mesh = _cheek_mesh()
    stage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, CHEEK_CENTER_Y, STAGE_THICKNESS)),
        material=steel,
        name="left_cheek",
    )
    stage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, -CHEEK_CENTER_Y, STAGE_THICKNESS), rpy=(0.0, 0.0, math.pi)),
        material=steel,
        name="right_cheek",
    )
    stage.inertial = Inertial.from_geometry(
        Box((STAGE_RADIUS * 2.0, STAGE_RADIUS * 2.0, CHEEK_HEIGHT)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, CHEEK_HEIGHT / 2.0)),
    )

    workplate = model.part("workplate")
    workplate.visual(
        Cylinder(radius=WORKPLATE_RADIUS, length=WORKPLATE_THICKNESS),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="plate_shell",
    )
    workplate.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HALF_SPAN * 2.0),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shaft_shell",
    )
    workplate.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(
            xyz=(0.0, CHEEK_CENTER_Y - CHEEK_THICKNESS / 2.0 + COLLAR_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_collar",
    )
    workplate.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(
            xyz=(0.0, -CHEEK_CENTER_Y + CHEEK_THICKNESS / 2.0 - COLLAR_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_collar",
    )
    workplate.visual(
        Box(FIXTURE_PAD_SIZE),
        origin=Origin(
            xyz=(
                0.0,
                WORKPLATE_THICKNESS / 2.0 + FIXTURE_PAD_SIZE[1] / 2.0,
                0.026,
            )
        ),
        material=anodized,
        name="fixture_pad",
    )
    workplate.inertial = Inertial.from_geometry(
        Box((WORKPLATE_RADIUS * 2.0, SHAFT_HALF_SPAN * 2.0, WORKPLATE_RADIUS * 2.0)),
        mass=3.0,
        origin=Origin(),
    )

    model.articulation(
        "base_to_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "stage_to_workplate",
        ArticulationType.REVOLUTE,
        parent=stage,
        child=workplate,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=2.0,
            lower=-1.35,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    stage = object_model.get_part("stage")
    workplate = object_model.get_part("workplate")

    base_to_stage = object_model.get_articulation("base_to_stage")
    stage_to_workplate = object_model.get_articulation("stage_to_workplate")

    pedestal_shell = base.get_visual("pedestal_shell")
    stage_platter = stage.get_visual("stage_platter")
    left_cheek = stage.get_visual("left_cheek")
    right_cheek = stage.get_visual("right_cheek")
    plate_shell = workplate.get_visual("plate_shell")
    shaft_shell = workplate.get_visual("shaft_shell")
    left_collar = workplate.get_visual("left_collar")
    right_collar = workplate.get_visual("right_collar")
    fixture_pad = workplate.get_visual("fixture_pad")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "part inventory",
        all(part is not None for part in (base, stage, workplate)),
        "base, stage, and workplate must all exist",
    )
    ctx.check(
        "joint axes",
        tuple(base_to_stage.axis) == (0.0, 0.0, 1.0)
        and tuple(stage_to_workplate.axis) == (0.0, 1.0, 0.0),
        "expected vertical rotary axis at the base and transverse trunnion axis at the workplate",
    )

    ctx.expect_overlap(stage, base, axes="xy", min_overlap=0.080, elem_a=stage_platter, elem_b=pedestal_shell)
    ctx.expect_gap(
        stage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=stage_platter,
        negative_elem=pedestal_shell,
    )

    ctx.expect_gap(
        workplate,
        stage,
        axis="z",
        min_gap=0.008,
        max_gap=0.020,
        positive_elem=plate_shell,
        negative_elem=stage_platter,
    )
    ctx.expect_origin_distance(workplate, stage, axes="xy", max_dist=0.001)
    ctx.expect_contact(workplate, stage, elem_a=left_collar, elem_b=left_cheek)
    ctx.expect_contact(workplate, stage, elem_a=right_collar, elem_b=right_cheek)
    ctx.expect_overlap(workplate, stage, axes="xz", min_overlap=0.020, elem_a=shaft_shell, elem_b=left_cheek)
    ctx.expect_overlap(workplate, stage, axes="xz", min_overlap=0.020, elem_a=shaft_shell, elem_b=right_cheek)

    with ctx.pose({base_to_stage: 1.10}):
        ctx.expect_overlap(stage, base, axes="xy", min_overlap=0.080, elem_a=stage_platter, elem_b=pedestal_shell)
        ctx.expect_gap(
            stage,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=stage_platter,
            negative_elem=pedestal_shell,
        )

    rest_pad_box = ctx.part_element_world_aabb(workplate, elem=fixture_pad)
    with ctx.pose({stage_to_workplate: 1.10}):
        tilted_pad_box = ctx.part_element_world_aabb(workplate, elem=fixture_pad)
        ctx.expect_gap(
            workplate,
            stage,
            axis="z",
            min_gap=0.008,
            positive_elem=plate_shell,
            negative_elem=stage_platter,
        )

    rest_pad_center = None
    tilted_pad_center = None
    if rest_pad_box is not None:
        rest_pad_center = tuple((lo + hi) / 2.0 for lo, hi in zip(rest_pad_box[0], rest_pad_box[1]))
    if tilted_pad_box is not None:
        tilted_pad_center = tuple((lo + hi) / 2.0 for lo, hi in zip(tilted_pad_box[0], tilted_pad_box[1]))
    ctx.check(
        "trunnion tilt moves fixture pad",
        rest_pad_center is not None
        and tilted_pad_center is not None
        and abs(tilted_pad_center[0] - rest_pad_center[0]) > 0.018
        and abs(tilted_pad_center[2] - rest_pad_center[2]) > 0.010,
        "fixture pad should sweep in x and z when the trunnion rotates",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
