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


FRAME_ARM_THICK = 0.016
FRAME_INNER_SPAN = 0.174
FRAME_ARM_CENTER_Y = FRAME_INNER_SPAN / 2.0 + FRAME_ARM_THICK / 2.0
FRAME_AXIS_X = 0.010
FRAME_AXIS_Z = 0.112

PLATE_THICK = 0.012
PLATE_WIDTH = 0.150
PLATE_HEIGHT = 0.120
PLATE_CENTER_X = 0.024
PLATE_CENTER_Z = 0.040
SHAFT_RADIUS = 0.009
SHAFT_LENGTH = 0.150
TRUNNION_RADIUS = 0.014
TRUNNION_LENGTH = 0.006
PIVOT_PAD_RADIUS = 0.018
PIVOT_PAD_LENGTH = 0.006


def _arm_profile_points() -> list[tuple[float, float]]:
    return [
        (-0.070, 0.000),
        (0.050, 0.000),
        (0.050, 0.014),
        (0.018, 0.014),
        (0.018, 0.084),
        (0.044, 0.091),
        (0.050, 0.112),
        (0.044, 0.133),
        (0.008, 0.145),
        (-0.020, 0.145),
        (-0.030, 0.116),
        (-0.030, 0.014),
        (-0.070, 0.014),
    ]


def _make_arm_shape() -> cq.Workplane:
    arm = (
        cq.Workplane("XZ")
        .polyline(_arm_profile_points())
        .close()
        .extrude(FRAME_ARM_THICK)
        .translate((0.0, FRAME_ARM_THICK / 2.0, 0.0))
    )

    lightening = (
        cq.Workplane("XZ")
        .rect(0.024, 0.048)
        .extrude(FRAME_ARM_THICK + 0.004)
        .translate((-0.002, FRAME_ARM_THICK / 2.0 + 0.002, 0.060))
    )

    return arm.cut(lightening)


def _make_faceplate_body() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(PLATE_THICK, PLATE_WIDTH, PLATE_HEIGHT)
        .edges("|X")
        .fillet(0.010)
        .translate((PLATE_CENTER_X, 0.0, PLATE_CENTER_Z))
    )
    panel = (
        panel.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(PLATE_WIDTH - 0.034, PLATE_HEIGHT - 0.028)
        .cutBlind(-0.003)
    )

    spine = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.006, -0.012),
                (0.010, -0.012),
                (0.018, 0.000),
                (0.018, 0.081),
                (0.010, 0.088),
                (-0.003, 0.020),
            ]
        )
        .close()
        .extrude(0.052)
        .translate((0.0, 0.026, 0.0))
    )

    hub_sleeve = (
        cq.Workplane("XZ")
        .circle(0.014)
        .extrude(0.060)
        .translate((0.0, 0.030, 0.0))
    )

    return panel.union(spine).union(hub_sleeve)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_supported_pitch_cradle")

    frame_finish = model.material("frame_finish", rgba=(0.24, 0.26, 0.29, 1.0))
    faceplate_finish = model.material("faceplate_finish", rgba=(0.84, 0.86, 0.88, 1.0))
    shaft_finish = model.material("shaft_finish", rgba=(0.70, 0.73, 0.76, 1.0))
    foot_finish = model.material("foot_finish", rgba=(0.11, 0.12, 0.13, 1.0))

    fork_frame = model.part("fork_frame")
    left_arm_shape = _make_arm_shape().translate((0.0, FRAME_ARM_CENTER_Y, 0.0))
    right_arm_shape = _make_arm_shape().translate((0.0, -FRAME_ARM_CENTER_Y, 0.0))

    fork_frame.visual(
        mesh_from_cadquery(left_arm_shape, "left_fork_arm"),
        material=frame_finish,
        name="left_arm",
    )
    fork_frame.visual(
        mesh_from_cadquery(right_arm_shape, "right_fork_arm"),
        material=frame_finish,
        name="right_arm",
    )
    fork_frame.visual(
        Box((0.028, FRAME_INNER_SPAN, 0.026)),
        origin=Origin(xyz=(-0.046, 0.0, 0.030)),
        material=frame_finish,
        name="lower_brace",
    )
    fork_frame.visual(
        Box((0.018, FRAME_INNER_SPAN + 0.032, 0.020)),
        origin=Origin(xyz=(-0.054, 0.0, 0.010)),
        material=foot_finish,
        name="rear_foot_bar",
    )
    fork_frame.visual(
        Cylinder(radius=PIVOT_PAD_RADIUS, length=PIVOT_PAD_LENGTH),
        origin=Origin(
            xyz=(FRAME_AXIS_X, FRAME_INNER_SPAN / 2.0 - PIVOT_PAD_LENGTH / 2.0, FRAME_AXIS_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=frame_finish,
        name="left_pivot_pad",
    )
    fork_frame.visual(
        Cylinder(radius=PIVOT_PAD_RADIUS, length=PIVOT_PAD_LENGTH),
        origin=Origin(
            xyz=(FRAME_AXIS_X, -FRAME_INNER_SPAN / 2.0 + PIVOT_PAD_LENGTH / 2.0, FRAME_AXIS_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=frame_finish,
        name="right_pivot_pad",
    )
    fork_frame.inertial = Inertial.from_geometry(
        Box((0.120, FRAME_INNER_SPAN + 0.030, 0.145)),
        mass=3.4,
        origin=Origin(xyz=(-0.010, 0.0, 0.060)),
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        mesh_from_cadquery(_make_faceplate_body(), "pitch_faceplate_body"),
        material=faceplate_finish,
        name="panel_body",
    )
    faceplate.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=shaft_finish,
        name="cross_shaft",
    )
    faceplate.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, FRAME_INNER_SPAN / 2.0 - PIVOT_PAD_LENGTH - TRUNNION_LENGTH / 2.0, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=shaft_finish,
        name="left_trunnion",
    )
    faceplate.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, -FRAME_INNER_SPAN / 2.0 + PIVOT_PAD_LENGTH + TRUNNION_LENGTH / 2.0, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=shaft_finish,
        name="right_trunnion",
    )
    faceplate.inertial = Inertial.from_geometry(
        Box((0.045, PLATE_WIDTH, PLATE_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(0.018, 0.0, 0.040)),
    )

    model.articulation(
        "cradle_pitch",
        ArticulationType.REVOLUTE,
        parent=fork_frame,
        child=faceplate,
        origin=Origin(xyz=(FRAME_AXIS_X, 0.0, FRAME_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.8,
            lower=-0.45,
            upper=0.60,
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

    fork_frame = object_model.get_part("fork_frame")
    faceplate = object_model.get_part("faceplate")
    cradle_pitch = object_model.get_articulation("cradle_pitch")

    ctx.check("fork frame part exists", fork_frame is not None)
    ctx.check("faceplate part exists", faceplate is not None)
    ctx.check(
        "pitch joint is revolute",
        cradle_pitch.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={cradle_pitch.articulation_type}",
    )

    with ctx.pose({cradle_pitch: 0.0}):
        ctx.expect_gap(
            fork_frame,
            faceplate,
            axis="y",
            positive_elem="left_arm",
            negative_elem="panel_body",
            min_gap=0.005,
            name="left arm clears the faceplate side",
        )
        ctx.expect_gap(
            faceplate,
            fork_frame,
            axis="y",
            positive_elem="panel_body",
            negative_elem="right_arm",
            min_gap=0.005,
            name="right arm clears the faceplate side",
        )
        ctx.expect_overlap(
            faceplate,
            fork_frame,
            axes="xz",
            elem_a="left_trunnion",
            elem_b="left_pivot_pad",
            min_overlap=0.020,
            name="left fork support lines up with the trunnion",
        )
        ctx.expect_overlap(
            faceplate,
            fork_frame,
            axes="xz",
            elem_a="right_trunnion",
            elem_b="right_pivot_pad",
            min_overlap=0.020,
            name="right fork support lines up with the trunnion",
        )
        ctx.expect_contact(
            faceplate,
            fork_frame,
            elem_a="left_trunnion",
            elem_b="left_pivot_pad",
            name="left trunnion seats on the fork support",
        )
        ctx.expect_contact(
            faceplate,
            fork_frame,
            elem_a="right_trunnion",
            elem_b="right_pivot_pad",
            name="right trunnion seats on the fork support",
        )

    rest_panel = ctx.part_element_world_aabb(faceplate, elem="panel_body")
    with ctx.pose({cradle_pitch: 0.55}):
        tilted_up_panel = ctx.part_element_world_aabb(faceplate, elem="panel_body")
    with ctx.pose({cradle_pitch: -0.35}):
        tilted_down_panel = ctx.part_element_world_aabb(faceplate, elem="panel_body")

    rest_center = _aabb_center(rest_panel)
    up_center = _aabb_center(tilted_up_panel)
    down_center = _aabb_center(tilted_down_panel)
    ctx.check(
        "positive pitch rocks the faceplate rearward",
        rest_center is not None
        and up_center is not None
        and down_center is not None
        and up_center[0] < rest_center[0] - 0.012
        and down_center[0] > rest_center[0] + 0.006,
        details=f"rest={rest_center}, up={up_center}, down={down_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
