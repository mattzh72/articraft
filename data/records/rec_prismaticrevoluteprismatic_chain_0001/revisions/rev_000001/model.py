from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_RAIL_TOP = 0.060
CENTER_PIVOT_Z = 0.113
TOOL_GUIDE_X = 0.150
BASE_SLIDER_TRAVEL = 0.110
ARM_SWEEP_UP = 1.050
TOOL_SLIDE_TRAVEL = 0.095


def _build_carriage_shape():
    lower_block = (
        cq.Workplane("XY")
        .box(0.080, 0.070, 0.029, centered=(True, True, False))
        .translate((0.0, 0.0, -0.0005))
    )
    column = (
        cq.Workplane("XY")
        .box(0.052, 0.070, 0.064, centered=(True, True, False))
        .translate((0.0, 0.0, 0.028))
    )
    rear_cover = (
        cq.Workplane("XY")
        .box(0.024, 0.058, 0.036, centered=(True, True, False))
        .translate((-0.028, 0.0, 0.046))
    )
    clevis_left = (
        cq.Workplane("XY")
        .box(0.026, 0.012, 0.046, centered=(True, True, False))
        .translate((0.0, 0.021, 0.090))
    )
    clevis_right = (
        cq.Workplane("XY")
        .box(0.026, 0.012, 0.046, centered=(True, True, False))
        .translate((0.0, -0.021, 0.090))
    )
    clevis_bridge = (
        cq.Workplane("XY")
        .box(0.036, 0.032, 0.012, centered=(True, True, False))
        .translate((0.0, 0.0, 0.082))
    )

    carriage = (
        lower_block.union(column)
        .union(rear_cover)
        .union(clevis_left)
        .union(clevis_right)
        .union(clevis_bridge)
    )

    clevis_relief = (
        cq.Workplane("XY")
        .box(0.020, 0.030, 0.056, centered=(True, True, False))
        .translate((0.0, 0.0, 0.080))
    )
    pivot_bore = (
        cq.Workplane("XY")
        .circle(0.009)
        .extrude(0.050)
        .translate((0.0, 0.0, -0.025))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, 0.0, CENTER_PIVOT_Z))
    )
    pivot_pin = (
        cq.Workplane("XY")
        .circle(0.0075)
        .extrude(0.034)
        .translate((0.0, 0.0, -0.017))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, 0.0, CENTER_PIVOT_Z))
    )

    return carriage.cut(clevis_relief).cut(pivot_bore).union(pivot_pin)


def _build_arm_shape():
    hub = (
        cq.Workplane("XY")
        .circle(0.022)
        .extrude(0.031)
        .translate((0.0, 0.0, -0.0155))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )
    beam = cq.Workplane("XY").box(0.220, 0.048, 0.040).translate((0.115, 0.0, 0.0))
    distal_housing = cq.Workplane("XY").box(0.055, 0.060, 0.058).translate((0.225, 0.0, 0.0))
    lower_rib = cq.Workplane("XY").box(0.120, 0.018, 0.026).translate((0.105, 0.0, -0.023))

    arm = hub.union(beam).union(distal_housing).union(lower_rib)

    tool_channel = cq.Workplane("XY").box(0.190, 0.030, 0.023).translate((0.168, 0.0, 0.0))
    pivot_bore = (
        cq.Workplane("XY")
        .circle(0.010)
        .extrude(0.040)
        .translate((0.0, 0.0, -0.020))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )

    return arm.cut(tool_channel).cut(pivot_bore)


def _build_tool_slide_shape():
    slide_body = cq.Workplane("XY").box(0.112, 0.031, 0.024).translate((0.054, 0.0, 0.0))
    front_plate = cq.Workplane("XY").box(0.014, 0.065, 0.045).translate((0.117, 0.0, 0.0))
    tool_head = cq.Workplane("XY").box(0.030, 0.042, 0.032).translate((0.139, 0.0, 0.0))
    spindle = cq.Workplane("XY").circle(0.010).extrude(0.048).translate((0.139, 0.0, -0.048))
    sensor_cap = cq.Workplane("XY").box(0.018, 0.020, 0.010).translate((0.085, 0.0, 0.017))

    return slide_body.union(front_plate).union(tool_head).union(spindle).union(sensor_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cartesian_fed_linkage", assets=ASSETS)

    model.material("machine_grey", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("rail_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("anodized_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("tool_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("safety_orange", rgba=(0.95, 0.46, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.420, 0.160, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="machine_grey",
    )
    base.visual(
        Box((0.340, 0.060, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material="machine_grey",
    )
    base.visual(
        Box((0.320, 0.038, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material="rail_steel",
    )
    base.visual(
        Box((0.300, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.048, 0.034)),
        material="tool_black",
    )
    base.visual(
        Box((0.300, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.048, 0.034)),
        material="tool_black",
    )
    base.visual(
        Box((0.024, 0.024, 0.008)),
        origin=Origin(xyz=(-0.150, 0.0, 0.056)),
        material="safety_orange",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.420, 0.160, 0.060)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "carriage.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="anodized_aluminum",
    )
    carriage.visual(
        Box((0.022, 0.060, 0.020)),
        origin=Origin(xyz=(-0.028, 0.0, 0.068)),
        material="safety_orange",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.080, 0.080, 0.140)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_build_arm_shape(), "arm.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="anodized_aluminum",
    )
    arm.visual(
        Box((0.150, 0.012, 0.010)),
        origin=Origin(xyz=(0.125, 0.0, 0.021)),
        material="tool_black",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.270, 0.060, 0.060)),
        mass=2.1,
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
    )

    tool_slide = model.part("tool_slide")
    tool_slide.visual(
        mesh_from_cadquery(_build_tool_slide_shape(), "tool_slide.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="tool_black",
    )
    tool_slide.visual(
        Box((0.018, 0.050, 0.012)),
        origin=Origin(xyz=(0.136, 0.0, 0.020)),
        material="safety_orange",
    )
    tool_slide.inertial = Inertial.from_geometry(
        Box((0.160, 0.070, 0.080)),
        mass=0.9,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_RAIL_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BASE_SLIDER_TRAVEL,
            upper=BASE_SLIDER_TRAVEL,
            effort=90.0,
            velocity=0.8,
        ),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, CENTER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=ARM_SWEEP_UP,
            effort=45.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "arm_to_tool_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=tool_slide,
        origin=Origin(xyz=(TOOL_GUIDE_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TOOL_SLIDE_TRAVEL,
            effort=35.0,
            velocity=0.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_joint_motion_axis(
        "base_to_carriage",
        "carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "arm_to_tool_slide",
        "tool_slide",
        world_axis="x",
        direction="positive",
        min_delta=0.02,
    )

    ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_gap(
        "carriage",
        "base",
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
    )
    ctx.expect_aabb_overlap("arm", "carriage", axes="yz", min_overlap=0.02)
    ctx.expect_aabb_contact("arm", "carriage")
    ctx.expect_aabb_overlap("tool_slide", "arm", axes="yz", min_overlap=0.025)
    ctx.expect_aabb_contact("tool_slide", "arm")
    ctx.expect_aabb_gap(
        "tool_slide",
        "base",
        axis="z",
        max_gap=0.08,
        max_penetration=0.0,
    )

    with ctx.pose(base_to_carriage=BASE_SLIDER_TRAVEL):
        ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.05)
        ctx.expect_aabb_gap(
            "carriage",
            "base",
            axis="z",
            max_gap=0.002,
            max_penetration=0.001,
        )

    with ctx.pose(base_to_carriage=-BASE_SLIDER_TRAVEL):
        ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.05)
        ctx.expect_aabb_gap(
            "carriage",
            "base",
            axis="z",
            max_gap=0.002,
            max_penetration=0.001,
        )

    with ctx.pose(carriage_to_arm=0.95):
        ctx.expect_aabb_overlap("arm", "carriage", axes="yz", min_overlap=0.02)
        ctx.expect_aabb_overlap("tool_slide", "arm", axes="yz", min_overlap=0.020)
        ctx.expect_aabb_gap(
            "tool_slide",
            "base",
            axis="z",
            max_gap=0.35,
            max_penetration=0.0,
        )

    with ctx.pose(arm_to_tool_slide=TOOL_SLIDE_TRAVEL):
        ctx.expect_aabb_overlap("tool_slide", "arm", axes="yz", min_overlap=0.020)
        ctx.expect_aabb_contact("tool_slide", "arm")
        ctx.expect_aabb_gap(
            "tool_slide",
            "base",
            axis="z",
            max_gap=0.08,
            max_penetration=0.0,
        )

    with ctx.pose(
        base_to_carriage=BASE_SLIDER_TRAVEL,
        carriage_to_arm=0.90,
        arm_to_tool_slide=TOOL_SLIDE_TRAVEL,
    ):
        ctx.expect_aabb_overlap("tool_slide", "arm", axes="yz", min_overlap=0.020)
        ctx.expect_aabb_gap(
            "tool_slide",
            "base",
            axis="z",
            max_gap=0.50,
            max_penetration=0.0,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
