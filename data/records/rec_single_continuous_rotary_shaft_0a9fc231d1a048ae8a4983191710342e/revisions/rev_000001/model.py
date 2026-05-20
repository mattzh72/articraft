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


FOOT_LENGTH = 0.78
FOOT_WIDTH = 0.28
FOOT_THICK = 0.08

LEFT_PEDESTAL_X = -0.17
RIGHT_PEDESTAL_X = 0.11
PEDESTAL_LENGTH = 0.11
PEDESTAL_BASE_WIDTH = 0.24
PEDESTAL_CROWN_WIDTH = 0.19
PEDESTAL_HEIGHT = 0.20

SPINDLE_AXIS_Z = 0.215
SHAFT_RADIUS = 0.045
BORE_RADIUS = 0.049

SHAFT_LEFT_END_X = -0.25
SHAFT_LENGTH = 0.52
HUB_RADIUS = 0.072
HUB_LENGTH = 0.05
FACEPLATE_RADIUS = 0.12
FACEPLATE_THICK = 0.032


def _pedestal_profile() -> cq.Workplane:
    shoulder_z = 0.04
    crown_shoulder_z = PEDESTAL_HEIGHT - 0.05
    return (
        cq.Workplane("YZ")
        .moveTo(-PEDESTAL_BASE_WIDTH / 2.0, 0.0)
        .lineTo(PEDESTAL_BASE_WIDTH / 2.0, 0.0)
        .lineTo(PEDESTAL_BASE_WIDTH / 2.0 - 0.012, shoulder_z)
        .lineTo(PEDESTAL_CROWN_WIDTH / 2.0 + 0.012, crown_shoulder_z)
        .threePointArc(
            (0.0, PEDESTAL_HEIGHT + 0.014),
            (-PEDESTAL_CROWN_WIDTH / 2.0 - 0.012, crown_shoulder_z),
        )
        .lineTo(-(PEDESTAL_BASE_WIDTH / 2.0 - 0.012), shoulder_z)
        .close()
    )


def _pedestal_body(x_center: float) -> cq.Workplane:
    pedestal = _pedestal_profile().extrude(PEDESTAL_LENGTH)
    pedestal = pedestal.translate((x_center - PEDESTAL_LENGTH / 2.0, 0.0, FOOT_THICK))
    pedestal = pedestal.edges("|X").fillet(0.012)
    return pedestal


def _bore_cutter(x_center: float) -> cq.Workplane:
    bore_length = PEDESTAL_LENGTH + 0.03
    return (
        cq.Workplane("YZ")
        .center(0.0, SPINDLE_AXIS_Z)
        .circle(BORE_RADIUS)
        .extrude(bore_length)
        .translate((x_center - bore_length / 2.0, 0.0, 0.0))
    )


def _build_base_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(
        FOOT_LENGTH,
        FOOT_WIDTH,
        FOOT_THICK,
        centered=(True, True, False),
    )
    foot = foot.edges("|Z").fillet(0.018)
    foot = (
        foot.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.27, -0.09),
                (-0.27, 0.09),
                (0.27, -0.09),
                (0.27, 0.09),
            ]
        )
        .slot2D(0.06, 0.02, angle=0.0)
        .cutThruAll()
    )

    base = foot.union(_pedestal_body(LEFT_PEDESTAL_X)).union(_pedestal_body(RIGHT_PEDESTAL_X))
    base = base.cut(_bore_cutter(LEFT_PEDESTAL_X)).cut(_bore_cutter(RIGHT_PEDESTAL_X))

    relief = (
        cq.Workplane("XY")
        .box(0.22, 0.12, 0.025, centered=(True, True, False))
        .translate((-(LEFT_PEDESTAL_X + RIGHT_PEDESTAL_X) / 2.0, 0.0, FOOT_THICK - 0.015))
    )
    return base.cut(relief)


def _build_spindle_shape() -> cq.Workplane:
    shaft = cq.Workplane("YZ").circle(SHAFT_RADIUS).extrude(SHAFT_LENGTH)
    hub = (
        cq.Workplane("YZ")
        .circle(HUB_RADIUS)
        .extrude(HUB_LENGTH)
        .translate((SHAFT_LENGTH - HUB_LENGTH, 0.0, 0.0))
    )
    faceplate = (
        cq.Workplane("YZ")
        .circle(FACEPLATE_RADIUS)
        .extrude(FACEPLATE_THICK)
        .translate((SHAFT_LENGTH, 0.0, 0.0))
    )

    spindle = shaft.union(hub).union(faceplate)
    spindle = (
        spindle.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .circle(0.04)
        .cutBlind(-0.005)
    )
    return spindle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_pedestal_spindle_module")

    model.material("cast_iron", rgba=(0.26, 0.27, 0.29, 1.0))
    model.material("spindle_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "dual_pedestal_base"),
        material="cast_iron",
        name="base_housing",
    )
    base.inertial = Inertial.from_geometry(
        Box((FOOT_LENGTH, FOOT_WIDTH, FOOT_THICK + PEDESTAL_HEIGHT)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, (FOOT_THICK + PEDESTAL_HEIGHT) / 2.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(_build_spindle_shape(), "dual_pedestal_spindle"),
        material="spindle_steel",
        name="spindle_body",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=FACEPLATE_RADIUS, length=SHAFT_LENGTH + FACEPLATE_THICK),
        mass=22.0,
        origin=Origin(
            xyz=((SHAFT_LENGTH + FACEPLATE_THICK) / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
    )

    model.articulation(
        "base_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(SHAFT_LEFT_END_X, 0.0, SPINDLE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=10.0),
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
    spindle = object_model.get_part("spindle")
    spindle_joint = object_model.get_articulation("base_to_spindle")

    ctx.allow_isolated_part(
        spindle,
        reason=(
            "The spindle runs in bearing clearances inside the two pedestal housings, "
            "so the rotating shaft is intentionally separated from the grounded base "
            "by a small radial gap in the rest pose."
        ),
    )

    ctx.check(
        "spindle uses one continuous revolute joint",
        spindle_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spindle_joint.articulation_type}",
    )
    ctx.check(
        "spindle rotates about the pedestal axis",
        tuple(round(v, 6) for v in spindle_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spindle_joint.axis}",
    )

    limits = spindle_joint.motion_limits
    ctx.check(
        "continuous spindle joint is unbounded",
        limits is not None and limits.lower is None and limits.upper is None,
        details=f"limits={limits}",
    )

    ctx.expect_origin_gap(
        spindle,
        base,
        axis="z",
        min_gap=SPINDLE_AXIS_Z - 0.001,
        max_gap=SPINDLE_AXIS_Z + 0.001,
        name="spindle axis sits at pedestal height",
    )
    ctx.expect_within(
        spindle,
        base,
        axes="y",
        margin=0.0,
        name="spindle stays centered laterally within the module footprint",
    )

    spindle_aabb = ctx.part_world_aabb(spindle)
    ctx.check(
        "shaft projects beyond the right pedestal to a faceplate",
        spindle_aabb is not None
        and spindle_aabb[1][0] > RIGHT_PEDESTAL_X + PEDESTAL_LENGTH / 2.0 + 0.10,
        details=f"spindle_aabb={spindle_aabb}",
    )
    ctx.check(
        "shaft reaches through the left support",
        spindle_aabb is not None
        and spindle_aabb[0][0] < LEFT_PEDESTAL_X - PEDESTAL_LENGTH / 2.0 + 0.01,
        details=f"spindle_aabb={spindle_aabb}",
    )

    rest_position = ctx.part_world_position(spindle)
    with ctx.pose({spindle_joint: pi / 2.0}):
        turned_position = ctx.part_world_position(spindle)
    ctx.check(
        "spindle rotates in place about a fixed axis",
        rest_position is not None
        and turned_position is not None
        and max(abs(a - b) for a, b in zip(rest_position, turned_position)) < 1e-9,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
