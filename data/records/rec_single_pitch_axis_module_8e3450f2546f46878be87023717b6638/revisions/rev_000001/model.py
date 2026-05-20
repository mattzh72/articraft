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


BASE_WIDTH = 0.34
BASE_DEPTH = 0.22
BASE_THICKNESS = 0.012
BASE_EDGE_RADIUS = 0.014
GROUND_PAD_INSET = 0.002
GROUND_PAD_TOP_OFFSET = 0.0005

PLINTH_WIDTH = 0.146
PLINTH_DEPTH = 0.104
PLINTH_HEIGHT = 0.030

PIVOT_Z = 0.092
ARM_THICKNESS = 0.016
ARM_DEPTH = 0.060
ARM_STEM_HEIGHT = 0.052
ARM_LUG_RADIUS = 0.028
ARM_CENTER_X = 0.088

REAR_BRACE_DEPTH = 0.016
REAR_BRACE_HEIGHT = 0.018
REAR_BRACE_Y = -0.038

HEAD_WIDTH = 0.112
HEAD_DEPTH = 0.078
HEAD_HEIGHT = 0.092
HEAD_CENTER_Y = 0.010
HEAD_CORNER_RADIUS = 0.010

TRUNNION_RADIUS = 0.012
TRUNNION_LENGTH = 0.025
TRUNNION_OVERLAP = 0.001

FACEPLATE_WIDTH = 0.098
FACEPLATE_HEIGHT = 0.074
FACEPLATE_THICKNESS = 0.004
FACEPLATE_CENTER_Y = HEAD_CENTER_Y + HEAD_DEPTH / 2.0 + FACEPLATE_THICKNESS / 2.0


def _x_cylinder(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center_xyz)
    )


def _yoke_arm(x_center: float) -> cq.Workplane:
    stem = (
        cq.Workplane("XY")
        .box(ARM_THICKNESS, ARM_DEPTH, ARM_STEM_HEIGHT, centered=(True, True, False))
        .translate((x_center, 0.0, BASE_THICKNESS))
        .edges("|Z")
        .fillet(0.004)
    )
    lug = _x_cylinder(ARM_LUG_RADIUS, ARM_THICKNESS, (x_center, 0.0, PIVOT_Z))
    return stem.union(lug)


def _base_shell() -> cq.Workplane:
    ground_plate = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(BASE_EDGE_RADIUS)
    )
    plinth = (
        cq.Workplane("XY")
        .box(PLINTH_WIDTH, PLINTH_DEPTH, PLINTH_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_THICKNESS))
        .edges("|Z")
        .fillet(0.008)
    )
    rear_brace = (
        cq.Workplane("XY")
        .box(
            2.0 * (ARM_CENTER_X - ARM_THICKNESS / 2.0),
            REAR_BRACE_DEPTH,
            REAR_BRACE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, REAR_BRACE_Y, BASE_THICKNESS + PLINTH_HEIGHT - 0.004))
        .edges("|Z")
        .fillet(0.004)
    )
    return (
        ground_plate.union(plinth)
        .union(rear_brace)
        .union(_yoke_arm(+ARM_CENTER_X))
        .union(_yoke_arm(-ARM_CENTER_X))
    )


def _head_shell() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(HEAD_WIDTH, HEAD_DEPTH, HEAD_HEIGHT)
        .edges("|Z")
        .fillet(HEAD_CORNER_RADIUS)
        .translate((0.0, HEAD_CENTER_Y, 0.0))
    )
    left_trunnion = _x_cylinder(
        TRUNNION_RADIUS,
        TRUNNION_LENGTH,
        (
            HEAD_WIDTH / 2.0 + TRUNNION_LENGTH / 2.0 - TRUNNION_OVERLAP,
            0.0,
            0.0,
        ),
    )
    right_trunnion = _x_cylinder(
        TRUNNION_RADIUS,
        TRUNNION_LENGTH,
        (
            -HEAD_WIDTH / 2.0 - TRUNNION_LENGTH / 2.0 + TRUNNION_OVERLAP,
            0.0,
            0.0,
        ),
    )
    return housing.union(left_trunnion).union(right_trunnion)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_pitch_cradle")

    base_finish = model.material("base_finish", rgba=(0.24, 0.26, 0.29, 1.0))
    head_finish = model.material("head_finish", rgba=(0.66, 0.69, 0.73, 1.0))
    face_finish = model.material("face_finish", rgba=(0.87, 0.88, 0.90, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shell(), "base_shell"),
        material=base_finish,
        name="base_shell",
    )
    base.visual(
        Box(
            (
                BASE_WIDTH - 2.0 * GROUND_PAD_INSET,
                BASE_DEPTH - 2.0 * GROUND_PAD_INSET,
                BASE_THICKNESS - GROUND_PAD_TOP_OFFSET,
            )
        ),
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS - GROUND_PAD_TOP_OFFSET) / 2.0)),
        material=base_finish,
        name="ground_pad",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, 0.12)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell(), "head_shell"),
        material=head_finish,
        name="head_shell",
    )
    head.visual(
        Box((FACEPLATE_WIDTH, FACEPLATE_THICKNESS, FACEPLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, FACEPLATE_CENTER_Y, 0.0)),
        material=face_finish,
        name="faceplate",
    )
    head.inertial = Inertial.from_geometry(
        Box((HEAD_WIDTH + 2.0 * TRUNNION_LENGTH, HEAD_DEPTH, HEAD_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(0.0, HEAD_CENTER_Y, 0.0)),
    )

    model.articulation(
        "base_to_head_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    pitch = object_model.get_articulation("base_to_head_pitch")

    ctx.expect_origin_distance(
        head,
        base,
        axes="xy",
        max_dist=0.001,
        name="head stays centered over the base footprint",
    )
    ctx.expect_gap(
        head,
        base,
        axis="z",
        positive_elem="faceplate",
        negative_elem="ground_pad",
        min_gap=0.040,
        name="faceplate clears the grounded pad at rest",
    )

    rest_faceplate = _aabb_center(ctx.part_element_world_aabb(head, elem="faceplate"))

    upper_limit = 0.85
    lower_limit = -0.35
    if pitch.motion_limits is not None:
        if pitch.motion_limits.upper is not None:
            upper_limit = pitch.motion_limits.upper
        if pitch.motion_limits.lower is not None:
            lower_limit = pitch.motion_limits.lower

    with ctx.pose({pitch: upper_limit}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            positive_elem="faceplate",
            negative_elem="ground_pad",
            min_gap=0.070,
            name="faceplate remains clear at full upward pitch",
        )
        pitched_faceplate = _aabb_center(ctx.part_element_world_aabb(head, elem="faceplate"))

    ctx.check(
        "positive pitch raises the faceplate",
        rest_faceplate is not None
        and pitched_faceplate is not None
        and pitched_faceplate[2] > rest_faceplate[2] + 0.020,
        details=f"rest={rest_faceplate}, pitched={pitched_faceplate}",
    )

    with ctx.pose({pitch: lower_limit}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            positive_elem="faceplate",
            negative_elem="ground_pad",
            min_gap=0.020,
            name="faceplate remains above the base at full downward pitch",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
