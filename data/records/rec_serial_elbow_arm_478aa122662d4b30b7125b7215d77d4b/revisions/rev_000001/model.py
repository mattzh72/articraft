from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.46
BASE_WIDTH = 0.36
BASE_THICKNESS = 0.04

COLUMN_SIZE = 0.18
COLUMN_HEIGHT = 0.44
HEAD_LENGTH = 0.14
HEAD_WIDTH = 0.20
HEAD_HEIGHT = 0.06
SHOULDER_AXIS_Z = BASE_THICKNESS + COLUMN_HEIGHT + 0.14

SHOULDER_HUB_RADIUS = 0.07
SHOULDER_HUB_LENGTH = 0.13
PEDESTAL_CHEEK_THICKNESS = 0.032
PEDESTAL_CHEEK_HEIGHT = 0.24
PEDESTAL_CHEEK_GAP = SHOULDER_HUB_LENGTH
PEDESTAL_CHEEK_Y = (PEDESTAL_CHEEK_GAP / 2.0) + (PEDESTAL_CHEEK_THICKNESS / 2.0)

UPPER_ROOT_LENGTH = 0.12
UPPER_ROOT_WIDTH = 0.126
UPPER_ROOT_HEIGHT = 0.14
UPPER_BEAM_LENGTH = 0.34
UPPER_BEAM_WIDTH = 0.12
UPPER_BEAM_HEIGHT = 0.11
ELBOW_AXIS_X = 0.46
UPPER_CHEEK_THICKNESS = 0.028
UPPER_CHEEK_LENGTH = 0.11
UPPER_CHEEK_HEIGHT = 0.11

ELBOW_HUB_RADIUS = 0.045
ELBOW_HUB_LENGTH = 0.10
UPPER_CHEEK_GAP = ELBOW_HUB_LENGTH
UPPER_CHEEK_Y = (UPPER_CHEEK_GAP / 2.0) + (UPPER_CHEEK_THICKNESS / 2.0)

FOREARM_ROOT_LENGTH = 0.09
FOREARM_ROOT_WIDTH = 0.088
FOREARM_ROOT_HEIGHT = 0.10
FOREARM_BEAM_LENGTH = 0.29
FOREARM_BEAM_WIDTH = 0.088
FOREARM_BEAM_HEIGHT = 0.085
FOREARM_TIP_X = 0.34

END_PLATE_RADIUS = 0.075
END_PLATE_THICKNESS = 0.018


def _pedestal_body_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )
    column = (
        cq.Workplane("XY")
        .box(COLUMN_SIZE, COLUMN_SIZE, COLUMN_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )
    head = (
        cq.Workplane("XY")
        .box(HEAD_LENGTH, HEAD_WIDTH, HEAD_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT + HEAD_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.008)
    )
    rear_buttress = (
        cq.Workplane("XY")
        .box(0.12, 0.18, 0.06)
        .translate((-0.09, 0.0, BASE_THICKNESS + COLUMN_HEIGHT + 0.03))
        .edges("|Z")
        .fillet(0.01)
    )
    return base.union(column).union(head).union(rear_buttress)


def _upper_body_shape() -> cq.Workplane:
    root = (
        cq.Workplane("XY")
        .box(UPPER_ROOT_LENGTH, UPPER_ROOT_WIDTH, UPPER_ROOT_HEIGHT)
        .translate((0.04, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.012)
    )
    beam = (
        cq.Workplane("XY")
        .box(UPPER_BEAM_LENGTH, UPPER_BEAM_WIDTH, UPPER_BEAM_HEIGHT)
        .translate((0.24, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.01)
    )
    lower_rib = (
        cq.Workplane("XY")
        .box(0.18, 0.08, 0.07)
        .translate((0.18, 0.0, -0.055))
        .edges("|Z")
        .fillet(0.008)
    )
    return root.union(beam).union(lower_rib)


def _forearm_body_shape() -> cq.Workplane:
    root = (
        cq.Workplane("XY")
        .box(FOREARM_ROOT_LENGTH, FOREARM_ROOT_WIDTH, FOREARM_ROOT_HEIGHT)
        .translate((0.03, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.01)
    )
    beam = (
        cq.Workplane("XY")
        .box(FOREARM_BEAM_LENGTH, FOREARM_BEAM_WIDTH, FOREARM_BEAM_HEIGHT)
        .translate((0.19, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.008)
    )
    tip_pad = (
        cq.Workplane("XY")
        .box(0.045, 0.096, 0.10)
        .translate((0.3175, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.008)
    )
    return root.union(beam).union(tip_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trunnion_root_elbow_arm")

    dark_base = model.material("dark_base", rgba=(0.15, 0.16, 0.18, 1.0))
    link_gray = model.material("link_gray", rgba=(0.68, 0.69, 0.72, 1.0))
    forearm_gray = model.material("forearm_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.72, 0.74, 0.77, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_body_shape(), "pedestal_body"),
        material=dark_base,
        name="pedestal_body",
    )
    pedestal.visual(
        Box((0.13, PEDESTAL_CHEEK_THICKNESS, PEDESTAL_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, PEDESTAL_CHEEK_Y, SHOULDER_AXIS_Z - 0.02)),
        material=dark_base,
        name="shoulder_cheek_left",
    )
    pedestal.visual(
        Box((0.13, PEDESTAL_CHEEK_THICKNESS, PEDESTAL_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, -PEDESTAL_CHEEK_Y, SHOULDER_AXIS_Z - 0.02)),
        material=dark_base,
        name="shoulder_cheek_right",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=SHOULDER_HUB_RADIUS, length=SHOULDER_HUB_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=link_gray,
        name="shoulder_hub",
    )
    upper_link.visual(
        mesh_from_cadquery(_upper_body_shape(), "upper_link_body"),
        material=link_gray,
        name="upper_body",
    )
    upper_link.visual(
        Box((UPPER_CHEEK_LENGTH, UPPER_CHEEK_THICKNESS, UPPER_CHEEK_HEIGHT)),
        origin=Origin(xyz=(ELBOW_AXIS_X, UPPER_CHEEK_Y, 0.0)),
        material=link_gray,
        name="elbow_cheek_left",
    )
    upper_link.visual(
        Box((UPPER_CHEEK_LENGTH, UPPER_CHEEK_THICKNESS, UPPER_CHEEK_HEIGHT)),
        origin=Origin(xyz=(ELBOW_AXIS_X, -UPPER_CHEEK_Y, 0.0)),
        material=link_gray,
        name="elbow_cheek_right",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=ELBOW_HUB_RADIUS, length=ELBOW_HUB_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=forearm_gray,
        name="elbow_hub",
    )
    forearm.visual(
        mesh_from_cadquery(_forearm_body_shape(), "forearm_body"),
        material=forearm_gray,
        name="forearm_body",
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        Cylinder(radius=END_PLATE_RADIUS, length=END_PLATE_THICKNESS),
        origin=Origin(
            xyz=(END_PLATE_THICKNESS / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=plate_gray,
        name="end_plate_face",
    )

    model.articulation(
        "pedestal_to_upper_link",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.0,
            lower=-0.75,
            upper=1.25,
        ),
    )
    model.articulation(
        "upper_link_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(ELBOW_AXIS_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-1.65,
            upper=1.85,
        ),
    )
    model.articulation(
        "forearm_to_end_plate",
        ArticulationType.FIXED,
        parent=forearm,
        child=end_plate,
        origin=Origin(xyz=(FOREARM_TIP_X, 0.0, 0.0)),
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
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    end_plate = object_model.get_part("end_plate")

    shoulder = object_model.get_articulation("pedestal_to_upper_link")
    elbow = object_model.get_articulation("upper_link_to_forearm")

    shoulder_hub = upper_link.get_visual("shoulder_hub")
    elbow_hub = forearm.get_visual("elbow_hub")

    ctx.check(
        "all prompt parts exist",
        all(part is not None for part in (pedestal, upper_link, forearm, end_plate)),
    )
    ctx.check(
        "serial shoulder and elbow axes are parallel",
        tuple(shoulder.axis) == tuple(elbow.axis),
        details=f"shoulder_axis={shoulder.axis}, elbow_axis={elbow.axis}",
    )
    ctx.check(
        "shoulder hub is visibly larger than elbow hub",
        isinstance(shoulder_hub.geometry, Cylinder)
        and isinstance(elbow_hub.geometry, Cylinder)
        and shoulder_hub.geometry.radius > elbow_hub.geometry.radius * 1.4,
        details=(
            f"shoulder_radius={getattr(shoulder_hub.geometry, 'radius', None)}, "
            f"elbow_radius={getattr(elbow_hub.geometry, 'radius', None)}"
        ),
    )

    ctx.expect_gap(
        pedestal,
        upper_link,
        axis="y",
        positive_elem="shoulder_cheek_left",
        negative_elem="shoulder_hub",
        min_gap=0.0,
        max_gap=0.001,
        name="left shoulder cheek directly supports the shoulder hub",
    )
    ctx.expect_gap(
        upper_link,
        pedestal,
        axis="y",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_cheek_right",
        min_gap=0.0,
        max_gap=0.001,
        name="right shoulder cheek directly supports the shoulder hub",
    )
    ctx.expect_gap(
        upper_link,
        forearm,
        axis="y",
        positive_elem="elbow_cheek_left",
        negative_elem="elbow_hub",
        min_gap=0.0,
        max_gap=0.001,
        name="left elbow cheek directly supports the elbow hub",
    )
    ctx.expect_gap(
        forearm,
        upper_link,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_cheek_right",
        min_gap=0.0,
        max_gap=0.001,
        name="right elbow cheek directly supports the elbow hub",
    )
    ctx.expect_contact(
        end_plate,
        forearm,
        elem_a="end_plate_face",
        elem_b="forearm_body",
        contact_tol=0.001,
        name="end plate seats on the forearm tip",
    )

    rest_end_plate = ctx.part_world_position(end_plate)
    with ctx.pose({shoulder: 0.9}):
        shoulder_raised_end_plate = ctx.part_world_position(end_plate)
    ctx.check(
        "positive shoulder motion raises the arm",
        rest_end_plate is not None
        and shoulder_raised_end_plate is not None
        and shoulder_raised_end_plate[2] > rest_end_plate[2] + 0.12,
        details=f"rest={rest_end_plate}, raised={shoulder_raised_end_plate}",
    )

    with ctx.pose({elbow: 1.1}):
        elbow_raised_end_plate = ctx.part_world_position(end_plate)
    ctx.check(
        "positive elbow motion raises the forearm",
        rest_end_plate is not None
        and elbow_raised_end_plate is not None
        and elbow_raised_end_plate[2] > rest_end_plate[2] + 0.10,
        details=f"rest={rest_end_plate}, elbow_pose={elbow_raised_end_plate}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
