from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BACKPLATE_W = 0.100
BACKPLATE_H = 0.160
BACKPLATE_T = 0.008

SUPPORT_W = 0.055
SUPPORT_D = 0.034
SUPPORT_H = 0.012
SUPPORT_Y = BACKPLATE_T * 0.5 + SUPPORT_D * 0.5
SUPPORT_Z = -0.041

LOWER_BEARING_R = 0.018
LOWER_BEARING_H = 0.018
LOWER_BEARING_Y = SUPPORT_Y
LOWER_BEARING_Z = SUPPORT_Z + SUPPORT_H * 0.5 + LOWER_BEARING_H * 0.5

JOINT_ORIGIN = (
    0.0,
    LOWER_BEARING_Y,
    LOWER_BEARING_Z + LOWER_BEARING_H * 0.5,
)

UPPER_COLLAR_R = 0.016
UPPER_COLLAR_H = 0.014

OUTPUT_FACE_W = 0.052
OUTPUT_FACE_T = 0.008
OUTPUT_FACE_H = 0.060
OUTPUT_FACE_Y = 0.025
OUTPUT_FACE_Z = 0.050


def _backplate_shape() -> cq.Workplane:
    hole_x = 0.032
    hole_z = 0.055
    return (
        cq.Workplane("XY")
        .box(BACKPLATE_W, BACKPLATE_T, BACKPLATE_H)
        .edges("|Y")
        .fillet(0.012)
        .faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-hole_x, hole_z),
                (hole_x, hole_z),
                (-hole_x, -hole_z),
                (hole_x, -hole_z),
            ]
        )
        .hole(0.007)
    )


def _support_block_shape() -> cq.Workplane:
    shelf = cq.Workplane("XY").box(SUPPORT_W, SUPPORT_D, SUPPORT_H).translate(
        (0.0, SUPPORT_Y, SUPPORT_Z)
    )
    rib = cq.Workplane("XY").box(0.024, 0.020, 0.028).translate((0.0, 0.013, -0.033))
    lower_bearing = (
        cq.Workplane("XY")
        .circle(LOWER_BEARING_R)
        .extrude(LOWER_BEARING_H)
        .translate(
            (
                0.0,
                LOWER_BEARING_Y,
                LOWER_BEARING_Z - LOWER_BEARING_H * 0.5,
            )
        )
    )
    return shelf.union(rib).union(lower_bearing)


def _carriage_body_shape() -> cq.Workplane:
    upper_collar = cq.Workplane("XY").circle(UPPER_COLLAR_R).extrude(UPPER_COLLAR_H)
    riser = cq.Workplane("XY").box(0.022, 0.024, 0.040).translate((0.0, 0.014, 0.031))
    return upper_collar.union(riser)


def _output_face_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(OUTPUT_FACE_W, OUTPUT_FACE_T, OUTPUT_FACE_H)
        .edges("|Y")
        .fillet(0.004)
        .faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.014, 0.014),
                (0.014, 0.014),
                (-0.014, -0.014),
                (0.014, -0.014),
            ]
        )
        .hole(0.005)
        .translate((0.0, OUTPUT_FACE_Y, OUTPUT_FACE_Z))
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((a + b) * 0.5 for a, b in zip(lo, hi))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backplate_pan_head")
    model.material("powder_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("machined_gray", rgba=(0.72, 0.74, 0.77, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_backplate_shape(), "backplate_shell"),
        material="powder_black",
        name="backplate_shell",
    )
    backplate.visual(
        mesh_from_cadquery(_support_block_shape(), "support_block"),
        material="powder_black",
        name="support_block",
    )

    pan_carrier = model.part("pan_carrier")
    pan_carrier.visual(
        mesh_from_cadquery(_carriage_body_shape(), "carriage_body"),
        material="machined_gray",
        name="carriage_body",
    )
    pan_carrier.visual(
        mesh_from_cadquery(_output_face_shape(), "output_face"),
        material="machined_gray",
        name="output_face",
    )

    model.articulation(
        "backplate_to_pan_carrier",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=pan_carrier,
        origin=Origin(xyz=JOINT_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-1.1,
            upper=1.1,
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

    backplate = object_model.get_part("backplate")
    pan_carrier = object_model.get_part("pan_carrier")
    yaw = object_model.get_articulation("backplate_to_pan_carrier")
    support_block = backplate.get_visual("support_block")
    backplate_shell = backplate.get_visual("backplate_shell")
    carriage_body = pan_carrier.get_visual("carriage_body")
    output_face = pan_carrier.get_visual("output_face")

    limits = yaw.motion_limits
    ctx.check(
        "yaw joint uses vertical axis with bounded travel",
        yaw.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == -1.1
        and limits.upper == 1.1,
        details=f"axis={yaw.axis}, limits={limits}",
    )

    with ctx.pose({yaw: 0.0}):
        ctx.expect_contact(
            pan_carrier,
            backplate,
            elem_a=carriage_body,
            elem_b=support_block,
            contact_tol=1e-4,
            name="carriage seats on the rotary support",
        )
        ctx.expect_gap(
            pan_carrier,
            backplate,
            axis="z",
            positive_elem=output_face,
            negative_elem=support_block,
            min_gap=0.010,
            max_gap=0.080,
            name="output face is carried above the support block",
        )

    with ctx.pose({yaw: yaw.motion_limits.upper}):
        ctx.expect_gap(
            pan_carrier,
            backplate,
            axis="y",
            positive_elem=output_face,
            negative_elem=backplate_shell,
            min_gap=0.002,
            name="output face stays proud of the backplate at max yaw",
        )

    with ctx.pose({yaw: 0.0}):
        rest_face = _aabb_center(ctx.part_element_world_aabb(pan_carrier, elem="output_face"))
    with ctx.pose({yaw: 1.0}):
        yawed_face = _aabb_center(ctx.part_element_world_aabb(pan_carrier, elem="output_face"))

    ctx.check(
        "output face swings laterally when yawed",
        rest_face is not None
        and yawed_face is not None
        and yawed_face[0] < rest_face[0] - 0.015
        and abs(yawed_face[2] - rest_face[2]) < 0.001,
        details=f"rest_face={rest_face}, yawed_face={yawed_face}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
