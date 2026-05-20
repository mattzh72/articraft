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


BASE_W = 0.170
BASE_D = 0.120
BASE_T = 0.016
BASE_HOLE_D = 0.009
BASE_HOLE_SPAN_X = 0.115
BASE_HOLE_SPAN_Y = 0.072

CHEEK_T = 0.016
CHEEK_D = 0.054
CHEEK_BODY_H = 0.060
CHEEK_LOBE_R = 0.022
CHEEK_HOLE_R = 0.0095
CHEEK_INNER_HALF_SPAN = 0.039
CHEEK_CENTER_X = CHEEK_INNER_HALF_SPAN + CHEEK_T / 2.0

TRUNNION_AXIS_Z = 0.072

FACE_W = 0.068
FACE_T = 0.018
FACE_H = 0.076
FACE_Z_MIN = -0.020
FACE_PAD_T = 0.007
FACE_PAD_H = 0.054
HUB_R = 0.012
COLLAR_T = CHEEK_INNER_HALF_SPAN - FACE_W / 2.0 - 0.0004
PIN_R = 0.0093
PIN_EXT = 0.013


def _make_base_yoke() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_W, BASE_D, BASE_T, centered=(True, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rarray(BASE_HOLE_SPAN_X, BASE_HOLE_SPAN_Y, 2, 2)
        .hole(BASE_HOLE_D)
    )

    cheek_box = (
        cq.Workplane("XY")
        .box(CHEEK_T, CHEEK_D, CHEEK_BODY_H, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_T))
    )
    cheek_lobe = (
        cq.Workplane("YZ")
        .circle(CHEEK_LOBE_R)
        .extrude(CHEEK_T)
        .translate((-CHEEK_T / 2.0, 0.0, TRUNNION_AXIS_Z))
    )
    trunnion_bore = (
        cq.Workplane("YZ")
        .circle(CHEEK_HOLE_R)
        .extrude(CHEEK_T + 0.002)
        .translate((-(CHEEK_T + 0.002) / 2.0, 0.0, TRUNNION_AXIS_Z))
    )

    cheek = cheek_box.union(cheek_lobe).cut(trunnion_bore)
    left_cheek = cheek.translate((-CHEEK_CENTER_X, 0.0, 0.0))
    right_cheek = cheek.translate((CHEEK_CENTER_X, 0.0, 0.0))

    return plate.union(left_cheek).union(right_cheek)


def _make_tilt_face() -> cq.Workplane:
    face_body = (
        cq.Workplane("XY")
        .box(FACE_W, FACE_T, FACE_H, centered=(True, True, False))
        .translate((0.0, 0.0, FACE_Z_MIN))
    )
    front_pad = (
        cq.Workplane("XY")
        .box(FACE_W * 0.82, FACE_PAD_T, FACE_PAD_H, centered=(True, False, False))
        .translate((0.0, FACE_T / 2.0, FACE_Z_MIN + 0.006))
    )
    trunnion_hub = (
        cq.Workplane("YZ")
        .circle(HUB_R)
        .extrude(FACE_W)
        .translate((-FACE_W / 2.0, 0.0, 0.0))
    )
    left_collar = (
        cq.Workplane("YZ")
        .circle(HUB_R)
        .extrude(COLLAR_T)
        .translate((-(FACE_W / 2.0 + COLLAR_T), 0.0, 0.0))
    )
    right_collar = (
        cq.Workplane("YZ")
        .circle(HUB_R)
        .extrude(COLLAR_T)
        .translate((FACE_W / 2.0, 0.0, 0.0))
    )
    left_pin = (
        cq.Workplane("YZ")
        .circle(PIN_R)
        .extrude(PIN_EXT)
        .translate((-(FACE_W / 2.0 + COLLAR_T + PIN_EXT), 0.0, 0.0))
    )
    right_pin = (
        cq.Workplane("YZ")
        .circle(PIN_R)
        .extrude(PIN_EXT)
        .translate((FACE_W / 2.0 + COLLAR_T, 0.0, 0.0))
    )

    return (
        face_body.union(front_pad)
        .union(trunnion_hub)
        .union(left_collar)
        .union(right_collar)
        .union(left_pin)
        .union(right_pin)
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def _aabb_min_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    return aabb[0][2]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_pitch_trunnion_head")

    base_finish = model.material("base_finish", rgba=(0.28, 0.30, 0.33, 1.0))
    face_finish = model.material("face_finish", rgba=(0.70, 0.72, 0.75, 1.0))

    base = model.part("base_yoke")
    base.visual(
        mesh_from_cadquery(
            _make_base_yoke(),
            "base_yoke",
            tolerance=0.00015,
            angular_tolerance=0.05,
        ),
        material=base_finish,
        name="yoke_shell",
    )

    face = model.part("tilt_face")
    face.visual(
        mesh_from_cadquery(
            _make_tilt_face(),
            "tilt_face",
            tolerance=0.00015,
            angular_tolerance=0.05,
        ),
        material=face_finish,
        name="face_shell",
    )

    model.articulation(
        "yoke_to_face",
        ArticulationType.REVOLUTE,
        parent=base,
        child=face,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.0,
            lower=-0.90,
            upper=0.90,
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

    base = object_model.get_part("base_yoke")
    face = object_model.get_part("tilt_face")
    tilt_joint = object_model.get_articulation("yoke_to_face")

    ctx.allow_isolated_part(
        face,
        reason=(
            "The moving face is carried by trunnion journals inside the cheek bores. "
            "A small modeled bearing clearance keeps the parts visually separate, "
            "so the floating QC sees a support gap instead of the intended mounted fit."
        ),
    )

    limits = tilt_joint.motion_limits
    ctx.check(
        "pitch joint has symmetric bidirectional travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"limits={limits}",
    )

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base plate is grounded",
        base_aabb is not None and abs(base_aabb[0][2]) <= 1e-6,
        details=f"base_aabb={base_aabb}",
    )

    ctx.expect_overlap(
        face,
        base,
        axes="x",
        min_overlap=0.080,
        name="face remains within the cheek support span",
    )

    rest_aabb = ctx.part_element_world_aabb(face, elem="face_shell")
    rest_center = _aabb_center(rest_aabb)
    rest_min_z = _aabb_min_z(rest_aabb)
    ctx.check(
        "face clears the base plate at rest",
        rest_min_z is not None and rest_min_z >= BASE_T + 0.030,
        details=f"rest_min_z={rest_min_z}, base_plate_top={BASE_T}",
    )

    with ctx.pose({tilt_joint: 0.60}):
        pos_aabb = ctx.part_element_world_aabb(face, elem="face_shell")
        pos_center = _aabb_center(pos_aabb)
        pos_min_z = _aabb_min_z(pos_aabb)
        ctx.check(
            "forward pitch still clears the base",
            pos_min_z is not None and pos_min_z >= BASE_T + 0.028,
            details=f"pos_min_z={pos_min_z}, base_plate_top={BASE_T}",
        )
    ctx.check(
        "positive pitch tips the face toward positive Y",
        rest_center is not None
        and pos_center is not None
        and pos_center[1] > rest_center[1] + 0.008,
        details=f"rest_center={rest_center}, pos_center={pos_center}",
    )

    with ctx.pose({tilt_joint: -0.60}):
        neg_aabb = ctx.part_element_world_aabb(face, elem="face_shell")
        neg_center = _aabb_center(neg_aabb)
        neg_min_z = _aabb_min_z(neg_aabb)
        ctx.check(
            "reverse pitch still clears the base",
            neg_min_z is not None and neg_min_z >= BASE_T + 0.028,
            details=f"neg_min_z={neg_min_z}, base_plate_top={BASE_T}",
        )
    ctx.check(
        "negative pitch tips the face toward negative Y",
        rest_center is not None
        and neg_center is not None
        and neg_center[1] < rest_center[1] - 0.010,
        details=f"rest_center={rest_center}, neg_center={neg_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
