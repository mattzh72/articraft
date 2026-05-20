from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


GUIDE_LENGTH = 0.150
GUIDE_OUTER_RADIUS = 0.016
GUIDE_INNER_RADIUS = 0.0115
GUIDE_COLLAR_LENGTH = 0.014
GUIDE_COLLAR_RADIUS = 0.019

MOUNT_LENGTH = 0.072
MOUNT_WIDTH = 0.048
MOUNT_HEIGHT = 0.010
MOUNT_CENTER_X = 0.056
MOUNT_CENTER_Z = -GUIDE_OUTER_RADIUS - (MOUNT_HEIGHT / 2.0) + 0.003

PLUNGER_TRAVEL = 0.075
PLUNGER_LENGTH = 0.192
PLUNGER_ROD_RADIUS = 0.0096
PLUNGER_ROD_LENGTH = 0.165

CLEVIS_START_X = 0.152
CLEVIS_LENGTH = 0.040
CLEVIS_WIDTH = 0.014
CLEVIS_HEIGHT = 0.020
CLEVIS_CENTER_Z = 0.0015
CLEVIS_SLOT_WIDTH = 0.0074
CHEEK_THICKNESS = (CLEVIS_WIDTH - CLEVIS_SLOT_WIDTH) / 2.0
CLEVIS_CHEEK_START_X = 0.160
CLEVIS_CHEEK_LENGTH = 0.030
CLEVIS_CHEEK_HEIGHT = 0.015
CLEVIS_CHEEK_CENTER_Z = 0.0065
CLEVIS_BEAM_HEIGHT = 0.009
CLEVIS_BEAM_CENTER_Z = -0.0035

PIVOT_X = 0.171
PIVOT_Z = 0.008
PIVOT_HOLE_RADIUS = 0.0022
PIVOT_PIN_LENGTH = CLEVIS_WIDTH

LATCH_BODY_WIDTH = 0.0048
LATCH_OPEN_ANGLE = 1.0

BUSHING_LENGTH = 0.004
BUSHING_POSITIONS = (0.034, 0.094)


def _build_guide_shell() -> cq.Workplane:
    tube = cq.Workplane("YZ").circle(GUIDE_OUTER_RADIUS).circle(GUIDE_INNER_RADIUS).extrude(GUIDE_LENGTH)
    front_collar = (
        cq.Workplane("YZ")
        .workplane(offset=GUIDE_LENGTH - GUIDE_COLLAR_LENGTH)
        .circle(GUIDE_COLLAR_RADIUS)
        .circle(GUIDE_INNER_RADIUS)
        .extrude(GUIDE_COLLAR_LENGTH)
    )
    return tube.union(front_collar)


def _build_mount_block() -> cq.Workplane:
    mount = (
        cq.Workplane("XY")
        .box(MOUNT_LENGTH, MOUNT_WIDTH, MOUNT_HEIGHT)
        .faces("<Z")
        .workplane()
        .pushPoints([(-0.020, 0.0), (0.020, 0.0)])
        .hole(0.0055)
    )
    return mount.translate((MOUNT_CENTER_X, 0.0, MOUNT_CENTER_Z))


def _build_clevis() -> cq.Workplane:
    lower_beam = cq.Workplane("XY").box(CLEVIS_LENGTH, CLEVIS_WIDTH, CLEVIS_BEAM_HEIGHT).translate(
        (
            CLEVIS_START_X + (CLEVIS_LENGTH / 2.0),
            0.0,
            CLEVIS_BEAM_CENTER_Z,
        )
    )
    cheek_center_x = CLEVIS_CHEEK_START_X + (CLEVIS_CHEEK_LENGTH / 2.0)
    left_cheek = cq.Workplane("XY").box(
        CLEVIS_CHEEK_LENGTH,
        CHEEK_THICKNESS,
        CLEVIS_CHEEK_HEIGHT,
    ).translate(
        (
            cheek_center_x,
            (CLEVIS_SLOT_WIDTH / 2.0) + (CHEEK_THICKNESS / 2.0),
            CLEVIS_CHEEK_CENTER_Z,
        )
    )
    right_cheek = cq.Workplane("XY").box(
        CLEVIS_CHEEK_LENGTH,
        CHEEK_THICKNESS,
        CLEVIS_CHEEK_HEIGHT,
    ).translate(
        (
            cheek_center_x,
            -((CLEVIS_SLOT_WIDTH / 2.0) + (CHEEK_THICKNESS / 2.0)),
            CLEVIS_CHEEK_CENTER_Z,
        )
    )
    pivot_pin = (
        cq.Workplane("XZ")
        .center(PIVOT_X, PIVOT_Z)
        .circle(PIVOT_HOLE_RADIUS)
        .extrude(PIVOT_PIN_LENGTH / 2.0, both=True)
    )
    return lower_beam.union(left_cheek).union(right_cheek).union(pivot_pin)


def _build_nose_latch() -> cq.Workplane:
    pivot_ear = cq.Workplane("XZ").circle(0.0052).extrude(LATCH_BODY_WIDTH / 2.0, both=True)
    latch_blade = (
        cq.Workplane("XZ")
        .moveTo(0.0005, 0.0044)
        .lineTo(0.013, 0.0042)
        .lineTo(0.026, 0.0012)
        .lineTo(0.0215, -0.0045)
        .lineTo(0.0075, -0.0050)
        .lineTo(-0.0010, -0.0018)
        .close()
        .extrude(LATCH_BODY_WIDTH / 2.0, both=True)
    )
    pivot_hole = cq.Workplane("XZ").circle(PIVOT_HOLE_RADIUS).extrude(LATCH_BODY_WIDTH, both=True)
    return pivot_ear.union(latch_blade).cut(pivot_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_plunger_nose_latch")

    model.material("tube_zinc", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("plunger_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("latch_black_oxide", rgba=(0.13, 0.14, 0.15, 1.0))

    guide_tube = model.part("guide_tube")
    guide_tube.visual(
        mesh_from_cadquery(_build_guide_shell(), "guide_shell"),
        material="tube_zinc",
        name="guide_shell",
    )
    guide_tube.visual(
        mesh_from_cadquery(_build_mount_block(), "guide_mount"),
        material="tube_zinc",
        name="guide_mount",
    )
    guide_tube.inertial = Inertial.from_geometry(
        Box((GUIDE_LENGTH, MOUNT_WIDTH, (2.0 * GUIDE_OUTER_RADIUS) + MOUNT_HEIGHT)),
        mass=0.55,
        origin=Origin(xyz=(GUIDE_LENGTH / 2.0, 0.0, -0.0035)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=PLUNGER_ROD_RADIUS, length=PLUNGER_ROD_LENGTH),
        origin=Origin(
            xyz=(PLUNGER_ROD_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="plunger_steel",
        name="plunger_rod",
    )
    for idx, bushing_x in enumerate(BUSHING_POSITIONS, start=1):
        plunger.visual(
            Cylinder(radius=GUIDE_INNER_RADIUS, length=BUSHING_LENGTH),
            origin=Origin(
                xyz=(bushing_x, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="plunger_steel",
            name=f"plunger_bushing_{idx}",
        )
    plunger.visual(
        mesh_from_cadquery(_build_clevis(), "plunger_clevis"),
        material="plunger_steel",
        name="plunger_clevis",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((PLUNGER_LENGTH, CLEVIS_WIDTH, CLEVIS_HEIGHT)),
        mass=0.22,
        origin=Origin(xyz=(PLUNGER_LENGTH / 2.0, 0.0, 0.002)),
    )

    nose_latch = model.part("nose_latch")
    nose_latch.visual(
        mesh_from_cadquery(_build_nose_latch(), "nose_latch_body"),
        material="latch_black_oxide",
        name="nose_latch_body",
    )
    nose_latch.inertial = Inertial.from_geometry(
        Box((0.032, LATCH_BODY_WIDTH, 0.012)),
        mass=0.04,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    model.articulation(
        "guide_to_plunger",
        ArticulationType.PRISMATIC,
        parent=guide_tube,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PLUNGER_TRAVEL,
            effort=90.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "plunger_to_nose_latch",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=nose_latch,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LATCH_OPEN_ANGLE,
            effort=8.0,
            velocity=2.5,
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

    guide_tube = object_model.get_part("guide_tube")
    plunger = object_model.get_part("plunger")
    nose_latch = object_model.get_part("nose_latch")
    guide_to_plunger = object_model.get_articulation("guide_to_plunger")
    plunger_to_nose_latch = object_model.get_articulation("plunger_to_nose_latch")

    guide_shell = guide_tube.get_visual("guide_shell")
    plunger_rod = plunger.get_visual("plunger_rod")
    plunger_clevis = plunger.get_visual("plunger_clevis")
    latch_body = nose_latch.get_visual("nose_latch_body")

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(min_corner, max_corner))

    ctx.expect_overlap(
        plunger,
        guide_tube,
        axes="x",
        elem_a=plunger_rod,
        elem_b=guide_shell,
        min_overlap=0.14,
        name="plunger rod is deeply retained in the guide at rest",
    )
    ctx.expect_within(
        plunger,
        guide_tube,
        axes="y",
        inner_elem=plunger_rod,
        outer_elem=guide_shell,
        margin=0.0,
        name="plunger rod stays centered laterally in the guide tube",
    )
    ctx.expect_overlap(
        nose_latch,
        plunger,
        axes="xy",
        elem_a=latch_body,
        elem_b=plunger_clevis,
        min_overlap=0.003,
        name="nose latch is carried inside the plunger clevis footprint",
    )
    ctx.expect_within(
        nose_latch,
        plunger,
        axes="y",
        inner_elem=latch_body,
        outer_elem=plunger_clevis,
        margin=0.001,
        name="nose latch thickness fits between the clevis cheeks",
    )

    rest_plunger_pos = ctx.part_world_position(plunger)
    with ctx.pose({guide_to_plunger: PLUNGER_TRAVEL}):
        ctx.expect_overlap(
            plunger,
            guide_tube,
            axes="x",
            elem_a=plunger_rod,
            elem_b=guide_shell,
            min_overlap=0.070,
            name="plunger rod keeps retained insertion at full extension",
        )
        extended_plunger_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger extends forward along the guide axis",
        rest_plunger_pos is not None
        and extended_plunger_pos is not None
        and extended_plunger_pos[0] > rest_plunger_pos[0] + 0.060,
        details=f"rest={rest_plunger_pos}, extended={extended_plunger_pos}",
    )

    closed_latch_aabb = ctx.part_element_world_aabb(nose_latch, elem=latch_body)
    with ctx.pose({plunger_to_nose_latch: LATCH_OPEN_ANGLE}):
        open_latch_aabb = ctx.part_element_world_aabb(nose_latch, elem=latch_body)

    closed_center = aabb_center(closed_latch_aabb)
    open_center = aabb_center(open_latch_aabb)
    ctx.check(
        "nose latch opens upward from the plunger tip",
        closed_center is not None
        and open_center is not None
        and open_center[2] > closed_center[2] + 0.006,
        details=f"closed_center={closed_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
