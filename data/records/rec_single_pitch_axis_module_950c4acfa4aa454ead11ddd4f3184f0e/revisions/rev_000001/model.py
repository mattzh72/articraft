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


BACKPLATE_WIDTH = 0.100
BACKPLATE_HEIGHT = 0.140
BACKPLATE_THICKNESS = 0.008

SUPPORT_BLOCK_WIDTH = 0.074
SUPPORT_BLOCK_DEPTH = 0.024
SUPPORT_BLOCK_HEIGHT = 0.050
SUPPORT_BLOCK_CENTER_Y = 0.016

ARM_THICKNESS = 0.010
ARM_LENGTH = 0.060
ARM_HEIGHT = 0.072
ARM_CENTER_X = 0.041
ARM_CENTER_Y = 0.034
TRUNNION_AXIS_Y = 0.059

PANEL_WIDTH = 0.060
PANEL_HEIGHT = 0.062
PANEL_THICKNESS = 0.006
TRUNNION_RADIUS = 0.0055
TRUNNION_CLEARANCE_RADIUS = 0.0066
TRUNNION_LENGTH = 0.006
TRUNNION_COLLAR_RADIUS = 0.0078
TRUNNION_COLLAR_LENGTH = 0.003


def _stationary_body_shape() -> cq.Workplane:
    backplate = (
        cq.Workplane("XY")
        .box(BACKPLATE_WIDTH, BACKPLATE_THICKNESS, BACKPLATE_HEIGHT)
        .edges("|Y")
        .fillet(0.010)
    )

    mount_through = (
        cq.Workplane("XZ")
        .workplane(offset=-(BACKPLATE_THICKNESS / 2.0) - 0.001)
        .rarray(0.062, 0.090, 2, 2)
        .circle(0.0042)
        .extrude(BACKPLATE_THICKNESS + 0.002)
    )
    mount_relief = (
        cq.Workplane("XZ")
        .workplane(offset=(BACKPLATE_THICKNESS / 2.0) - 0.003)
        .rarray(0.062, 0.090, 2, 2)
        .circle(0.0074)
        .extrude(0.0035)
    )
    backplate = backplate.cut(mount_through).cut(mount_relief)

    support_block = cq.Workplane("XY").box(
        SUPPORT_BLOCK_WIDTH,
        SUPPORT_BLOCK_DEPTH,
        SUPPORT_BLOCK_HEIGHT,
    ).translate((0.0, SUPPORT_BLOCK_CENTER_Y, 0.0))

    return backplate.union(support_block)


def _arm_shape(*, mirrored: bool) -> cq.Workplane:
    arm = (
        cq.Workplane("XY")
        .box(ARM_THICKNESS, ARM_LENGTH, ARM_HEIGHT)
        .edges("|X")
        .fillet(0.004)
    )
    x_pos = -ARM_CENTER_X if mirrored else ARM_CENTER_X
    return arm.translate((x_pos, ARM_CENTER_Y, 0.0))


def _panel_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(PANEL_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)
        .edges("|Y")
        .fillet(0.0045)
    )

    raised_pad = cq.Workplane("XY").box(0.040, 0.003, 0.034).translate(
        (0.0, (PANEL_THICKNESS / 2.0) + 0.0015, 0.0)
    )
    slots = (
        cq.Workplane("XZ")
        .workplane(offset=-(PANEL_THICKNESS / 2.0) - 0.001)
        .rarray(0.030, 1.0, 2, 1)
        .slot2D(0.022, 0.007, angle=90)
        .extrude(PANEL_THICKNESS + 0.002)
    )
    return panel.union(raised_pad).cut(slots)


def _right_trunnion_shape() -> cq.Workplane:
    shaft = (
        cq.Workplane("YZ")
        .workplane(offset=PANEL_WIDTH / 2.0)
        .circle(TRUNNION_RADIUS)
        .extrude(TRUNNION_LENGTH)
    )
    collar = (
        cq.Workplane("YZ")
        .workplane(offset=PANEL_WIDTH / 2.0)
        .circle(TRUNNION_COLLAR_RADIUS)
        .extrude(TRUNNION_COLLAR_LENGTH)
    )
    return shaft.union(collar)


def _left_trunnion_shape() -> cq.Workplane:
    shaft = (
        cq.Workplane("YZ")
        .workplane(offset=-(PANEL_WIDTH / 2.0) - TRUNNION_LENGTH)
        .circle(TRUNNION_RADIUS)
        .extrude(TRUNNION_LENGTH)
    )
    collar = (
        cq.Workplane("YZ")
        .workplane(offset=-(PANEL_WIDTH / 2.0) - TRUNNION_COLLAR_LENGTH)
        .circle(TRUNNION_COLLAR_RADIUS)
        .extrude(TRUNNION_COLLAR_LENGTH)
    )
    return shaft.union(collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_tilt_head")

    dark_coat = model.material("dark_coat", rgba=(0.20, 0.22, 0.25, 1.0))
    satin_alloy = model.material("satin_alloy", rgba=(0.73, 0.75, 0.78, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_stationary_body_shape(), "support_body"),
        material=dark_coat,
        name="stationary_body",
    )
    support.visual(
        mesh_from_cadquery(_arm_shape(mirrored=True), "support_left_arm"),
        material=dark_coat,
        name="arm_left",
    )
    support.visual(
        mesh_from_cadquery(_arm_shape(mirrored=False), "support_right_arm"),
        material=dark_coat,
        name="arm_right",
    )
    support.inertial = Inertial.from_geometry(
        Box((BACKPLATE_WIDTH, 0.068, BACKPLATE_HEIGHT)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        mesh_from_cadquery(_panel_shape(), "tilt_panel"),
        material=satin_alloy,
        name="panel",
    )
    faceplate.visual(
        mesh_from_cadquery(_left_trunnion_shape(), "tilt_panel_left_trunnion"),
        material=satin_alloy,
        name="trunnion_left",
    )
    faceplate.visual(
        mesh_from_cadquery(_right_trunnion_shape(), "tilt_panel_right_trunnion"),
        material=satin_alloy,
        name="trunnion_right",
    )
    faceplate.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH + (2.0 * TRUNNION_LENGTH), 0.010, PANEL_HEIGHT)),
        mass=0.45,
        origin=Origin(),
    )

    model.articulation(
        "support_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=support,
        child=faceplate,
        origin=Origin(xyz=(0.0, TRUNNION_AXIS_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=-0.55,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("support")
    faceplate = object_model.get_part("faceplate")
    tilt = object_model.get_articulation("support_to_faceplate")

    support.get_visual("stationary_body")
    support.get_visual("arm_left")
    support.get_visual("arm_right")
    faceplate.get_visual("panel")
    faceplate.get_visual("trunnion_left")
    faceplate.get_visual("trunnion_right")

    limits = tilt.motion_limits
    ctx.check(
        "tilt joint uses a supported horizontal trunnion axis",
        abs(tilt.axis[0] + 1.0) < 1e-9
        and abs(tilt.axis[1]) < 1e-9
        and abs(tilt.axis[2]) < 1e-9
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"axis={tilt.axis}, limits={limits}",
    )

    with ctx.pose({tilt: 0.0}):
        ctx.expect_gap(
            support,
            faceplate,
            axis="x",
            positive_elem="arm_right",
            negative_elem="panel",
            min_gap=0.004,
            max_gap=0.008,
            name="panel clears the right arm cheek",
        )
        ctx.expect_gap(
            faceplate,
            support,
            axis="x",
            positive_elem="panel",
            negative_elem="arm_left",
            min_gap=0.004,
            max_gap=0.008,
            name="panel clears the left arm cheek",
        )
        ctx.expect_gap(
            faceplate,
            support,
            axis="y",
            positive_elem="panel",
            negative_elem="stationary_body",
            min_gap=0.020,
            name="panel stands forward of the grounded backplate support",
        )
        ctx.expect_contact(
            faceplate,
            support,
            elem_a="trunnion_left",
            elem_b="arm_left",
            name="left trunnion bears against the left arm face",
        )
        ctx.expect_contact(
            faceplate,
            support,
            elem_a="trunnion_right",
            elem_b="arm_right",
            name="right trunnion bears against the right arm face",
        )

    if limits is not None and limits.upper is not None:
        with ctx.pose({tilt: limits.upper}):
            ctx.expect_gap(
                faceplate,
                support,
                axis="y",
                positive_elem="panel",
                negative_elem="stationary_body",
                min_gap=0.004,
                name="panel clears the support block at max forward tilt",
            )

    if limits is not None and limits.lower is not None:
        with ctx.pose({tilt: limits.lower}):
            ctx.expect_gap(
                faceplate,
                support,
                axis="y",
                positive_elem="panel",
                negative_elem="stationary_body",
                min_gap=0.004,
                name="panel clears the support block at max backward tilt",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
