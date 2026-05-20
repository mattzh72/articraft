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


BODY_LENGTH = 0.180
BODY_WIDTH = 0.130
BODY_HEIGHT = 0.074

YOKE_BASE_LENGTH = 0.088
YOKE_BASE_WIDTH = 0.104
YOKE_BASE_HEIGHT = 0.016
TRUNNION_AXIS_X = 0.030
TRUNNION_AXIS_Z = 0.112
EAR_LENGTH = 0.040
EAR_THICKNESS = 0.015
EAR_HEIGHT = 0.060
INNER_STAGE_GAP = 0.068
RISER_LENGTH = 0.034
RISER_WIDTH = 0.018
RISER_HEIGHT = 0.016
REAR_BRIDGE_LENGTH = 0.060
REAR_BRIDGE_WIDTH = 0.100
REAR_BRIDGE_HEIGHT = 0.016

SHAFT_RADIUS = 0.0125
BORE_RADIUS = 0.0140
SHAFT_LENGTH = INNER_STAGE_GAP + 2.0 * EAR_THICKNESS + 0.006
COLLAR_RADIUS = 0.018
COLLAR_THICKNESS = 0.004
STAGE_WIDTH = 0.056

FACE_THICKNESS = 0.010
FACE_WIDTH = 0.066
FACE_HEIGHT = 0.066
FACE_CENTER_X = 0.069
REGISTER_RADIUS = 0.013
REGISTER_LENGTH = 0.006

PITCH_LOWER = 0.0
PITCH_UPPER = 0.85


def _body_housing_shape() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )

    top_lip = cq.Workplane("XY").box(
        BODY_LENGTH * 0.82,
        BODY_WIDTH * 0.88,
        0.006,
        centered=(True, True, False),
    ).translate((0.0, 0.0, BODY_HEIGHT - 0.006))

    return housing.union(top_lip)


def _support_yoke_shape() -> cq.Workplane:
    outer_y = INNER_STAGE_GAP / 2.0 + EAR_THICKNESS / 2.0
    bridge = cq.Workplane("XY").box(
        REAR_BRIDGE_LENGTH,
        REAR_BRIDGE_WIDTH,
        REAR_BRIDGE_HEIGHT,
        centered=(True, True, False),
    ).translate((TRUNNION_AXIS_X - 0.032, 0.0, BODY_HEIGHT))

    yoke = bridge
    ear_bottom = TRUNNION_AXIS_Z - 0.018

    for sign in (-1.0, 1.0):
        riser = cq.Workplane("XY").box(
            RISER_LENGTH,
            RISER_WIDTH,
            RISER_HEIGHT,
            centered=(True, True, False),
        ).translate((TRUNNION_AXIS_X - 0.010, sign * outer_y, BODY_HEIGHT + 0.006))
        yoke = yoke.union(riser)

        ear = (
            cq.Workplane("XY")
            .box(EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT, centered=(True, True, False))
            .translate((TRUNNION_AXIS_X, sign * outer_y, ear_bottom))
            .edges(">Z")
            .fillet(0.005)
        )
        yoke = yoke.union(ear)

        bore = (
            cq.Workplane("XY")
            .cylinder(EAR_THICKNESS + 0.010, BORE_RADIUS, centered=(True, True, True))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
            .translate((TRUNNION_AXIS_X, sign * outer_y, TRUNNION_AXIS_Z))
        )
        yoke = yoke.cut(bore)

    return yoke


def _stage_core_shape() -> cq.Workplane:
    profile_points = [
        (-0.014, -0.018),
        (0.010, -0.024),
        (0.040, -0.030),
        (0.056, -0.028),
        (0.064, -0.020),
        (0.064, 0.020),
        (0.056, 0.028),
        (0.040, 0.030),
        (0.010, 0.024),
        (-0.014, 0.018),
    ]
    carrier = (
        cq.Workplane("XZ")
        .polyline(profile_points)
        .close()
        .extrude(STAGE_WIDTH / 2.0, both=True)
        .edges("|Y")
        .fillet(0.0035)
    )

    shaft = (
        cq.Workplane("XY")
        .cylinder(SHAFT_LENGTH, SHAFT_RADIUS, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )

    return carrier.union(shaft)


def _trunnion_collar_shape() -> cq.Workplane:
    collar_center_y = INNER_STAGE_GAP / 2.0 + EAR_THICKNESS + COLLAR_THICKNESS / 2.0
    collars = None
    for sign in (-1.0, 1.0):
        collar = (
            cq.Workplane("XY")
            .cylinder(COLLAR_THICKNESS, COLLAR_RADIUS, centered=(True, True, True))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
            .translate((0.0, sign * collar_center_y, 0.0))
        )
        collars = collar if collars is None else collars.union(collar)

    return collars


def _output_face_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(FACE_THICKNESS, FACE_WIDTH, FACE_HEIGHT, centered=(True, True, True))
        .translate((FACE_CENTER_X, 0.0, 0.0))
        .edges("|X")
        .fillet(0.005)
    )


def _register_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(REGISTER_LENGTH, REGISTER_RADIUS, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((FACE_CENTER_X + FACE_THICKNESS / 2.0 + REGISTER_LENGTH / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_production_pitch_module")

    body_finish = model.material("body_finish", rgba=(0.22, 0.24, 0.27, 1.0))
    yoke_finish = model.material("yoke_finish", rgba=(0.58, 0.61, 0.64, 1.0))
    stage_finish = model.material("stage_finish", rgba=(0.78, 0.42, 0.12, 1.0))
    interface_finish = model.material("interface_finish", rgba=(0.79, 0.81, 0.84, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_housing_shape(), "body_housing"),
        material=body_finish,
        name="housing",
    )
    body.visual(
        mesh_from_cadquery(_support_yoke_shape(), "support_yoke"),
        material=yoke_finish,
        name="support_yoke",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, TRUNNION_AXIS_Z + 0.030)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, (TRUNNION_AXIS_Z + 0.030) / 2.0)),
    )

    stage = model.part("pitch_stage")
    stage.visual(
        mesh_from_cadquery(_stage_core_shape(), "pitch_stage_core"),
        material=stage_finish,
        name="stage_core",
    )
    stage.visual(
        mesh_from_cadquery(_trunnion_collar_shape(), "pitch_trunnion_collars"),
        material=interface_finish,
        name="trunnion_collars",
    )
    stage.visual(
        mesh_from_cadquery(_output_face_shape(), "pitch_output_face"),
        material=interface_finish,
        name="output_face",
    )
    stage.visual(
        mesh_from_cadquery(_register_shape(), "pitch_face_register"),
        material=interface_finish,
        name="face_register",
    )
    stage.inertial = Inertial.from_geometry(
        Box((0.090, SHAFT_LENGTH, 0.080)),
        mass=1.4,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_pitch_stage",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stage,
        origin=Origin(xyz=(TRUNNION_AXIS_X, 0.0, TRUNNION_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=32.0,
            velocity=2.0,
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    stage = object_model.get_part("pitch_stage")
    pitch_joint = object_model.get_articulation("body_to_pitch_stage")

    ctx.check(
        "pitch module parts exist",
        body is not None and stage is not None and pitch_joint is not None,
        details="Expected grounded body, pitch stage, and a single trunnion articulation.",
    )
    ctx.check(
        "pitch joint uses the supported horizontal trunnion axis",
        pitch_joint.axis == (0.0, -1.0, 0.0)
        and pitch_joint.motion_limits is not None
        and pitch_joint.motion_limits.lower == PITCH_LOWER
        and pitch_joint.motion_limits.upper == PITCH_UPPER,
        details=(
            f"axis={pitch_joint.axis}, "
            f"lower={pitch_joint.motion_limits.lower if pitch_joint.motion_limits else None}, "
            f"upper={pitch_joint.motion_limits.upper if pitch_joint.motion_limits else None}"
        ),
    )
    ctx.allow_overlap(
        body,
        stage,
        elem_a="support_yoke",
        elem_b="trunnion_collars",
        reason="The retained trunnion collars seat against the yoke thrust faces; the tiny mesh-level interference is intentional in this supported fit.",
    )

    ctx.expect_overlap(
        body,
        stage,
        axes="yz",
        elem_a="support_yoke",
        elem_b="stage_core",
        min_overlap=0.050,
        name="stage remains nested inside the trunnion support envelope",
    )
    ctx.expect_gap(
        stage,
        body,
        axis="x",
        positive_elem="output_face",
        negative_elem="housing",
        min_gap=0.004,
        name="output face projects ahead of the grounded housing",
    )

    def element_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            (lower[0] + upper[0]) / 2.0,
            (lower[1] + upper[1]) / 2.0,
            (lower[2] + upper[2]) / 2.0,
        )

    rest_center = element_center("pitch_stage", "output_face")
    with ctx.pose({pitch_joint: PITCH_UPPER}):
        raised_center = element_center("pitch_stage", "output_face")

    ctx.check(
        "positive pitch lifts the output face",
        rest_center is not None
        and raised_center is not None
        and raised_center[2] > rest_center[2] + 0.025
        and raised_center[0] < rest_center[0] - 0.010,
        details=f"rest_center={rest_center}, raised_center={raised_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
