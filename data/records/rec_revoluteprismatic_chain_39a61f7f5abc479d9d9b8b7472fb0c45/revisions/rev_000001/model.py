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


BASE_LENGTH = 0.24
BASE_WIDTH = 0.18
BASE_HEIGHT = 0.02
PEDESTAL_SIZE = 0.11
PEDESTAL_HEIGHT = 0.045
YOKE_LENGTH = 0.05
YOKE_THICKNESS = 0.014
YOKE_HEIGHT = 0.05
YOKE_GAP = 0.078
YOKE_CENTER_Z = 0.075
YOKE_CENTER_Y = YOKE_GAP / 2.0 + YOKE_THICKNESS / 2.0

LINK_FRONT_X = 0.225
SLIDER_TRAVEL = 0.07
SHOULDER_UPPER = 1.0


def _base_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )

    rear_housing = (
        cq.Workplane("XY")
        .box(0.07, 0.12, 0.035)
        .translate((-0.065, 0.0, 0.0375))
        .edges("|Z")
        .fillet(0.008)
    )
    center_pedestal = (
        cq.Workplane("XY")
        .box(0.05, 0.06, 0.055)
        .translate((0.0, 0.0, 0.0475))
        .edges("|Z")
        .fillet(0.006)
    )
    shoulder_pad = (
        cq.Workplane("XY")
        .box(0.065, 0.085, 0.008)
        .translate((0.006, 0.0, 0.071))
        .edges("|Z")
        .fillet(0.003)
    )

    return base_plate.union(rear_housing).union(center_pedestal).union(shoulder_pad)


def _link_shape() -> cq.Workplane:
    shoulder_hub = (
        cq.Workplane("XZ")
        .circle(0.022)
        .extrude(0.06)
        .translate((0.008, -0.03, 0.046))
    )
    shoulder_web = cq.Workplane("XY").box(0.018, 0.06, 0.024).translate((0.008, 0.0, 0.026))
    pivot_heel = cq.Workplane("XY").box(0.05, 0.07, 0.018).translate((0.012, 0.0, 0.009))
    arm_body = cq.Workplane("XY").box(0.17, 0.058, 0.042).translate((0.105, 0.0, 0.029))
    nose_sleeve = cq.Workplane("XY").box(0.04, 0.04, 0.028).translate((0.205, 0.0, 0.018))
    top_spine = cq.Workplane("XY").box(0.09, 0.032, 0.012).translate((0.09, 0.0, 0.048))
    guide_rail = cq.Workplane("XY").box(0.12, 0.028, 0.012).translate((0.15, 0.0, 0.006))

    shell = pivot_heel.union(arm_body).union(nose_sleeve).union(top_spine).union(guide_rail)
    shell = shell.edges("|Z").fillet(0.004)
    shell = shell.union(shoulder_web).union(shoulder_hub)

    side_pocket = cq.Workplane("XY").box(0.085, 0.01, 0.02).translate((0.102, 0.024, 0.028))

    return shell.cut(side_pocket).cut(side_pocket.mirror("XZ"))


def _slider_shape() -> cq.Workplane:
    inner_rail = cq.Workplane("XY").box(0.18, 0.02, 0.018).translate((-0.02, 0.0, -0.009))
    stop_cap = cq.Workplane("XY").box(0.006, 0.03, 0.018).translate((0.003, 0.0, -0.009))
    nose = (
        cq.Workplane("XY")
        .box(0.028, 0.022, 0.016)
        .translate((0.084, 0.0, -0.008))
        .edges(">X")
        .fillet(0.004)
    )

    return inner_rail.union(stop_cap).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_arm")

    model.material("base_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("arm_aluminum", rgba=(0.64, 0.67, 0.71, 1.0))
    model.material("slider_dark", rgba=(0.24, 0.26, 0.29, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "service_arm_base"), material="base_dark", name="base_shell")

    service_link = model.part("service_link")
    service_link.visual(
        mesh_from_cadquery(_link_shape(), "service_arm_link"),
        material="arm_aluminum",
        name="link_shell",
    )

    nose_slider = model.part("nose_slider")
    nose_slider.visual(
        mesh_from_cadquery(_slider_shape(), "service_arm_slider"),
        material="slider_dark",
        name="slider_body",
    )

    model.articulation(
        "base_to_link",
        ArticulationType.REVOLUTE,
        parent=base,
        child=service_link,
        origin=Origin(xyz=(0.0, 0.0, YOKE_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=SHOULDER_UPPER,
        ),
    )
    model.articulation(
        "link_to_slider",
        ArticulationType.PRISMATIC,
        parent=service_link,
        child=nose_slider,
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    service_link = object_model.get_part("service_link")
    nose_slider = object_model.get_part("nose_slider")
    shoulder = object_model.get_articulation("base_to_link")
    slide = object_model.get_articulation("link_to_slider")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        service_link,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="base pedestal supports the rigid link from below",
    )
    ctx.expect_gap(
        service_link,
        nose_slider,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="collapsed slider seats directly under the link guide",
    )
    ctx.expect_within(
        nose_slider,
        service_link,
        axes="y",
        margin=0.0,
        name="slider stays centered under the link guide",
    )
    ctx.expect_gap(
        service_link,
        nose_slider,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="slider rides directly beneath the link guide",
    )
    ctx.expect_overlap(
        nose_slider,
        service_link,
        axes="x",
        min_overlap=0.14,
        name="collapsed slider remains substantially inserted",
    )

    slider_rest = ctx.part_world_position(nose_slider)
    with ctx.pose({slide: SLIDER_TRAVEL}):
        ctx.expect_within(
            nose_slider,
            service_link,
            axes="y",
            margin=0.0,
            name="extended slider stays centered under the link guide",
        )
        ctx.expect_gap(
            service_link,
            nose_slider,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="extended slider remains guided below the link",
        )
        ctx.expect_overlap(
            nose_slider,
            service_link,
            axes="x",
            min_overlap=0.07,
            name="extended slider still retains insertion in the link",
        )
        slider_extended = ctx.part_world_position(nose_slider)

    ctx.check(
        "slider extends forward along the link axis",
        slider_rest is not None
        and slider_extended is not None
        and slider_extended[0] > slider_rest[0] + 0.05,
        details=f"rest={slider_rest}, extended={slider_extended}",
    )

    with ctx.pose({shoulder: SHOULDER_UPPER * 0.75}):
        raised_slider = ctx.part_world_position(nose_slider)

    ctx.check(
        "revolute shoulder lifts the arm upward",
        slider_rest is not None
        and raised_slider is not None
        and raised_slider[2] > slider_rest[2] + 0.08,
        details=f"rest={slider_rest}, raised={raised_slider}",
    )

    link_box = ctx.part_element_world_aabb(service_link, elem="link_shell")
    slider_box = ctx.part_element_world_aabb(nose_slider, elem="slider_body")
    slider_is_smaller = False
    if link_box is not None and slider_box is not None:
        link_y = link_box[1][1] - link_box[0][1]
        link_z = link_box[1][2] - link_box[0][2]
        slider_y = slider_box[1][1] - slider_box[0][1]
        slider_z = slider_box[1][2] - slider_box[0][2]
        slider_is_smaller = slider_y < link_y * 0.7 and slider_z < link_z * 0.7
    ctx.check(
        "tip slider stays clearly smaller than the rigid link",
        slider_is_smaller,
        details=f"link_box={link_box}, slider_box={slider_box}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
