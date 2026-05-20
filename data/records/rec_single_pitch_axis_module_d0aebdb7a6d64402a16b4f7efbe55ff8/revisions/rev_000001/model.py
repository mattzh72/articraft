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


FOOT_LENGTH = 0.240
FOOT_WIDTH = 0.180
FOOT_HEIGHT = 0.030

PEDESTAL_HEIGHT = 0.130
PIVOT_Z = 0.214

YOKE_WIDTH = 0.094
YOKE_BLOCK_LENGTH = 0.118
YOKE_BLOCK_HEIGHT = 0.022
YOKE_BLOCK_CENTER_Z = 0.169

TRUNNION_GAP = 0.066
EAR_THICKNESS = 0.014
EAR_CENTER_Y = TRUNNION_GAP / 2.0 + EAR_THICKNESS / 2.0


def _build_foot_pedestal() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT)
        .translate((0.0, 0.0, FOOT_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )

    collar = (
        cq.Workplane("XY")
        .box(0.112, 0.086, 0.014)
        .translate((0.0, 0.0, FOOT_HEIGHT + 0.007))
        .edges("|Z")
        .fillet(0.005)
    )

    pedestal = (
        cq.Workplane("XY")
        .rect(0.102, 0.076)
        .workplane(offset=PEDESTAL_HEIGHT)
        .rect(0.076, 0.058)
        .loft(combine=True)
        .translate((0.0, 0.0, FOOT_HEIGHT))
    )

    return foot.union(collar).union(pedestal)


def _build_head_yoke() -> cq.Workplane:
    head_block = (
        cq.Workplane("XY")
        .box(YOKE_BLOCK_LENGTH, YOKE_WIDTH, YOKE_BLOCK_HEIGHT)
        .translate((0.0, 0.0, YOKE_BLOCK_CENTER_Z))
        .edges("|Z")
        .fillet(0.006)
    )

    def make_ear(y_center: float) -> cq.Workplane:
        body = (
            cq.Workplane("XY")
            .box(0.038, EAR_THICKNESS, 0.060)
            .translate((0.0, y_center, 0.197))
        )
        cap = (
            cq.Workplane("XZ")
            .circle(0.018)
            .extrude(EAR_THICKNESS / 2.0, both=True)
            .translate((0.0, y_center, PIVOT_Z))
        )
        hole = (
            cq.Workplane("XZ")
            .circle(0.0094)
            .extrude((EAR_THICKNESS + 0.006) / 2.0, both=True)
            .translate((0.0, y_center, PIVOT_Z))
        )
        return body.union(cap).cut(hole)

    return head_block.union(make_ear(EAR_CENTER_Y)).union(make_ear(-EAR_CENTER_Y))


def _build_carrier_body() -> cq.Workplane:
    shaft = (
        cq.Workplane("XZ")
        .circle(0.0083)
        .extrude(0.044, both=True)
    )

    carrier = (
        cq.Workplane("XY")
        .box(0.034, 0.056, 0.034)
        .translate((0.020, 0.0, 0.006))
        .edges("|Z")
        .fillet(0.004)
    )

    rear_saddle = (
        cq.Workplane("XY")
        .box(0.018, 0.062, 0.024)
        .translate((0.008, 0.0, 0.004))
        .edges("|Z")
        .fillet(0.003)
    )

    return shaft.union(carrier).union(rear_saddle)


def _build_face_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.008, 0.078, 0.062)
        .translate((0.039, 0.0, 0.0))
        .edges("|X")
        .fillet(0.005)
    )

    rear_boss = (
        cq.Workplane("XY")
        .box(0.010, 0.042, 0.022)
        .translate((0.031, 0.0, 0.0))
        .edges("|X")
        .fillet(0.003)
    )

    hole_pattern = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (0.022, 0.018),
                (0.022, -0.018),
                (-0.022, 0.018),
                (-0.022, -0.018),
            ]
        )
        .circle(0.0026)
        .extrude(0.016)
        .translate((0.031, 0.0, 0.0))
    )

    return plate.union(rear_boss).cut(hole_pattern)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_pitch_stage")

    model.material("powder_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("cast_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    model.material("machined_aluminum", rgba=(0.77, 0.79, 0.81, 1.0))

    support = model.part("support_tower")
    support.visual(
        mesh_from_cadquery(_build_foot_pedestal(), "support_foot_pedestal"),
        material="powder_black",
        name="foot_pedestal",
    )
    support.visual(
        mesh_from_cadquery(_build_head_yoke(), "support_head_yoke"),
        material="cast_gray",
        name="head_yoke",
    )
    support.inertial = Inertial.from_geometry(
        Box((FOOT_LENGTH, FOOT_WIDTH, 0.220)),
        mass=9.2,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    tilt_face = model.part("tilt_face")
    tilt_face.visual(
        mesh_from_cadquery(_build_carrier_body(), "tilt_face_carrier"),
        material="cast_gray",
        name="carrier_body",
    )
    tilt_face.visual(
        mesh_from_cadquery(_build_face_plate(), "tilt_face_plate"),
        material="machined_aluminum",
        name="face_plate",
    )
    tilt_face.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.070)),
        mass=1.3,
        origin=Origin(xyz=(0.026, 0.0, 0.002)),
    )

    model.articulation(
        "support_to_tilt_face",
        ArticulationType.REVOLUTE,
        parent=support,
        child=tilt_face,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.35,
            upper=1.10,
            effort=18.0,
            velocity=1.5,
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

    support = object_model.get_part("support_tower")
    tilt_face = object_model.get_part("tilt_face")
    tilt_joint = object_model.get_articulation("support_to_tilt_face")

    ctx.allow_isolated_part(
        tilt_face,
        reason=(
            "The tilt face is carried on a coaxial trunnion with modeled bearing clearance "
            "inside the yoke holes, so the exact-contact floating-group heuristic should not "
            "treat the child as a loose unsupported assembly."
        ),
    )

    limits = tilt_joint.motion_limits
    ctx.check(
        "tilt joint uses a horizontal trunnion axis",
        tuple(round(v, 6) for v in tilt_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={tilt_joint.axis}",
    )
    ctx.check(
        "tilt joint spans down and up travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"limits={limits}",
    )

    head_aabb = ctx.part_element_world_aabb(support, elem="head_yoke")
    face_aabb = ctx.part_element_world_aabb(tilt_face, elem="face_plate")
    if head_aabb is not None and face_aabb is not None:
        head_size_y = head_aabb[1][1] - head_aabb[0][1]
        head_size_z = head_aabb[1][2] - head_aabb[0][2]
        face_size_y = face_aabb[1][1] - face_aabb[0][1]
        face_size_z = face_aabb[1][2] - face_aabb[0][2]
        ctx.check(
            "moving face is smaller than the support head below it",
            face_size_y < head_size_y and face_size_z < head_size_z,
            details=(
                f"face_size=(y={face_size_y:.4f}, z={face_size_z:.4f}), "
                f"head_size=(y={head_size_y:.4f}, z={head_size_z:.4f})"
            ),
        )

    rest_center = _aabb_center(face_aabb)
    with ctx.pose({tilt_joint: 0.80}):
        raised_aabb = ctx.part_element_world_aabb(tilt_face, elem="face_plate")
    raised_center = _aabb_center(raised_aabb)
    ctx.check(
        "positive tilt raises the moving face",
        rest_center is not None
        and raised_center is not None
        and raised_center[2] > rest_center[2] + 0.015
        and raised_center[0] < rest_center[0] - 0.008,
        details=f"rest_center={rest_center}, raised_center={raised_center}",
    )

    ctx.expect_overlap(
        tilt_face,
        support,
        axes="y",
        elem_a="face_plate",
        elem_b="head_yoke",
        min_overlap=0.060,
        name="moving face stays centered over the support head",
    )
    ctx.expect_within(
        tilt_face,
        support,
        axes="y",
        inner_elem="carrier_body",
        outer_elem="head_yoke",
        margin=0.004,
        name="carrier body stays between the yoke cheeks",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
