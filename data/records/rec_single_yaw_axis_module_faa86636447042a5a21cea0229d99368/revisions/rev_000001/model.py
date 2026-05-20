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


BACKPLATE_W = 0.110
BACKPLATE_H = 0.180
BACKPLATE_T = 0.010
BACKPLATE_CORNER_R = 0.012

ARM_OVERLAP = 0.001
SUPPORT_ARM_W = 0.046
SUPPORT_ARM_H = 0.032
SUPPORT_ARM_LEN = 0.068
REAR_BRIDGE_END_X = 0.045
STRUT_W = 0.018
STRUT_H = 0.012
STRUT_CENTER_Z = 0.015

AXIS_X = BACKPLATE_T + SUPPORT_ARM_LEN
STRUT_END_X = AXIS_X - 0.018
COLLAR_R = 0.021
COLLAR_H = 0.010
COLLAR_CENTER_Z = 0.012

SPINDLE_R = 0.015
HUB_H = 0.014

CHILD_ARM_W = 0.014
CHILD_ARM_H = 0.016
PANEL_W = 0.060
PANEL_H = 0.050
PANEL_T = 0.006
PANEL_CENTER_X = 0.028
PANEL_CORNER_R = 0.007


def _backplate_shape() -> cq.Workplane:
    plate = cq.Workplane("YZ").rect(BACKPLATE_W, BACKPLATE_H).extrude(BACKPLATE_T)
    return plate.edges("|X").fillet(BACKPLATE_CORNER_R)


def _cartridge_shape() -> cq.Workplane:
    bridge = (
        cq.Workplane("YZ")
        .rect(SUPPORT_ARM_W, SUPPORT_ARM_H)
        .extrude((AXIS_X - COLLAR_H) - (BACKPLATE_T - ARM_OVERLAP))
        .translate((BACKPLATE_T - ARM_OVERLAP, 0.0, 0.0))
    )
    nose_cartridge = (
        cq.Workplane("YZ")
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
        .translate((AXIS_X - COLLAR_H, 0.0, 0.0))
    )
    return bridge.union(nose_cartridge)


def _rotor_spindle_shape() -> cq.Workplane:
    return cq.Workplane("YZ").circle(SPINDLE_R).extrude(HUB_H)


def _faceplate_panel_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("YZ")
        .rect(PANEL_W, PANEL_H)
        .extrude(PANEL_T)
        .translate((PANEL_CENTER_X - PANEL_T / 2.0, 0.0, 0.0))
        .edges("|X")
        .fillet(PANEL_CORNER_R)
    )
    return panel


def _yaw_arm_shape() -> cq.Workplane:
    arm = (
        cq.Workplane("YZ")
        .rect(CHILD_ARM_W, CHILD_ARM_H)
        .extrude(PANEL_CENTER_X - PANEL_T / 2.0 + 0.001)
    )
    return arm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_yaw_head")

    model.material("powder_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("satin_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_backplate_shape(), "yaw_head_backplate"),
        material="powder_black",
        name="backplate",
    )
    support.visual(
        mesh_from_cadquery(_cartridge_shape(), "yaw_head_cartridge"),
        material="powder_black",
        name="cartridge_housing",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        mesh_from_cadquery(_rotor_spindle_shape(), "yaw_head_spindle"),
        material="satin_aluminum",
        name="rotor_spindle",
    )
    faceplate.visual(
        mesh_from_cadquery(_yaw_arm_shape(), "yaw_head_yaw_arm"),
        material="satin_aluminum",
        name="yaw_arm",
    )
    faceplate.visual(
        mesh_from_cadquery(_faceplate_panel_shape(), "yaw_head_faceplate"),
        material="satin_aluminum",
        name="faceplate_panel",
    )

    model.articulation(
        "support_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=support,
        child=faceplate,
        origin=Origin(xyz=(AXIS_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.35,
            upper=1.35,
            effort=6.0,
            velocity=2.0,
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
    support = object_model.get_part("support")
    faceplate = object_model.get_part("faceplate")
    yaw = object_model.get_articulation("support_to_faceplate")

    ctx.check(
        "yaw axis is vertical",
        tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "yaw limits open to both sides",
        yaw.motion_limits is not None
        and yaw.motion_limits.lower is not None
        and yaw.motion_limits.upper is not None
        and yaw.motion_limits.lower < 0.0
        and yaw.motion_limits.upper > 0.0,
        details=f"limits={yaw.motion_limits}",
    )
    ctx.expect_origin_distance(
        faceplate,
        support,
        axes="yz",
        max_dist=0.001,
        name="faceplate axis stays centered on the support centerline",
    )
    ctx.expect_gap(
        faceplate,
        support,
        axis="x",
        positive_elem="faceplate_panel",
        negative_elem="cartridge_housing",
        min_gap=0.018,
        max_gap=0.032,
        name="faceplate panel sits just ahead of the cartridge nose",
    )
    ctx.expect_contact(
        faceplate,
        support,
        elem_a="rotor_spindle",
        elem_b="cartridge_housing",
        contact_tol=0.0005,
        name="rotor core is clamped between the support collars",
    )

    rest_aabb = ctx.part_element_world_aabb(faceplate, elem="faceplate_panel")
    with ctx.pose({yaw: 0.9}):
        turned_aabb = ctx.part_element_world_aabb(faceplate, elem="faceplate_panel")
        ctx.expect_gap(
            faceplate,
            support,
            axis="x",
            positive_elem="faceplate_panel",
            negative_elem="backplate",
            min_gap=0.010,
            name="turned faceplate still stays forward of the wall plate",
        )

    rest_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
    turned_y = None if turned_aabb is None else (turned_aabb[0][1] + turned_aabb[1][1]) / 2.0
    ctx.check(
        "positive yaw sweeps the faceplate toward positive y",
        rest_y is not None and turned_y is not None and turned_y > rest_y + 0.010,
        details=f"rest_y={rest_y}, turned_y={turned_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
