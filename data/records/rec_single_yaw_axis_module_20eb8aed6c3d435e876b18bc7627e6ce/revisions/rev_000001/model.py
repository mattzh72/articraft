from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_SIZE = 0.18
FOOT_THICKNESS = 0.028
PLINTH_SIZE_X = 0.072
PLINTH_SIZE_Y = 0.062
PLINTH_HEIGHT = 0.010
PEDESTAL_HEIGHT = 0.054
PEDESTAL_BASE_RADIUS = 0.025
PEDESTAL_TOP_RADIUS = 0.021
COLLAR_RADIUS = 0.028
COLLAR_HEIGHT = 0.010
JOINT_Z = FOOT_THICKNESS + PLINTH_HEIGHT + PEDESTAL_HEIGHT


def _base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(FOOT_SIZE, FOOT_SIZE, FOOT_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.016)
        .faces(">Z")
        .edges()
        .fillet(0.003)
    )

    plinth = (
        cq.Workplane("XY")
        .box(PLINTH_SIZE_X, PLINTH_SIZE_Y, PLINTH_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, 0.0, FOOT_THICKNESS))
    )

    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_BASE_RADIUS)
        .workplane(offset=PEDESTAL_HEIGHT)
        .circle(PEDESTAL_TOP_RADIUS)
        .loft(combine=True)
        .translate((0.0, 0.0, FOOT_THICKNESS + PLINTH_HEIGHT))
    )

    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_HEIGHT)
        .translate((0.0, 0.0, JOINT_Z))
    )

    return foot.union(plinth).union(pedestal).union(collar)


def _pan_head_shape() -> cq.Workplane:
    skirt = cq.Workplane("XY").circle(0.043).circle(COLLAR_RADIUS + 0.0015).extrude(0.018)

    thrust_plate = (
        cq.Workplane("XY")
        .circle(0.040)
        .extrude(0.002)
        .translate((0.0, 0.0, COLLAR_HEIGHT))
    )

    drum = (
        cq.Workplane("XY")
        .circle(0.046)
        .extrude(0.024)
        .faces(">Z")
        .edges()
        .fillet(0.004)
        .translate((0.0, 0.0, 0.012))
    )

    neck = (
        cq.Workplane("XY")
        .box(0.046, 0.046, 0.010, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, 0.0, 0.036))
    )

    plate = (
        cq.Workplane("XY")
        .box(0.096, 0.096, 0.006, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, 0.046))
    )

    return skirt.union(thrust_plate).union(drum).union(neck).union(plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_top_pan_module")

    model.material("powder_black", rgba=(0.15, 0.16, 0.17, 1.0))
    model.material("machined_gray", rgba=(0.67, 0.70, 0.73, 1.0))
    model.material("sensor_black", rgba=(0.09, 0.10, 0.11, 1.0))

    support = model.part("support_base")
    support.visual(
        mesh_from_cadquery(_base_shape(), "support_base"),
        material="powder_black",
        name="support_shell",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        mesh_from_cadquery(_pan_head_shape(), "pan_head"),
        material="machined_gray",
        name="head_shell",
    )
    pan_head.visual(
        Box((0.026, 0.030, 0.016)),
        origin=Origin(xyz=(0.049, 0.0, 0.028)),
        material="sensor_black",
        name="head_nose",
    )
    pan_head.visual(
        Box((0.078, 0.078, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.05275)),
        material="sensor_black",
        name="sensor_face",
    )

    model.articulation(
        "pedestal_pan",
        ArticulationType.REVOLUTE,
        parent=support,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=-2.7,
            upper=2.7,
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

    support = object_model.get_part("support_base")
    pan_head = object_model.get_part("pan_head")
    pan_joint = object_model.get_articulation("pedestal_pan")
    nose = pan_head.get_visual("head_nose")
    sensor_face = pan_head.get_visual("sensor_face")

    ctx.expect_origin_distance(
        pan_head,
        support,
        axes="xy",
        max_dist=1e-6,
        name="pan head stays concentric with the pedestal",
    )
    ctx.expect_origin_gap(
        pan_head,
        support,
        axis="z",
        min_gap=JOINT_Z - 1e-6,
        max_gap=JOINT_Z + 1e-6,
        name="pan axis sits at the top of the short pedestal",
    )
    ctx.expect_overlap(
        pan_head,
        support,
        axes="xy",
        min_overlap=0.055,
        name="rotary head is centered over the heavy foot",
    )
    ctx.expect_within(
        pan_head,
        pan_head,
        axes="xy",
        inner_elem=sensor_face,
        outer_elem="head_shell",
        margin=0.010,
        name="sensor face stays within the square sensor plate footprint",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_center = _aabb_center(ctx.part_element_world_aabb(pan_head, elem=nose))
    with ctx.pose({pan_joint: 1.0}):
        turned_center = _aabb_center(ctx.part_element_world_aabb(pan_head, elem=nose))

    ctx.check(
        "positive pan rotates the nose about +Z",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.035
        and abs(rest_center[1]) < 0.004
        and abs(turned_center[2] - rest_center[2]) < 1e-6
        and turned_center[1] > 0.025
        and turned_center[0] < rest_center[0] - 0.010,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
