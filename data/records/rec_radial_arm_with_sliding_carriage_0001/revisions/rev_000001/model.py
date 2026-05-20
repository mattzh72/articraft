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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

PIVOT_Z = 0.38
CARRIAGE_HOME_X = 0.20
CARRIAGE_TRAVEL = 0.24
CARRIAGE_FRAME_OFFSET_X = -0.01


def _make_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.36, 0.30, 0.045, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.01)
    )
    pedestal = (
        cq.Workplane("XY")
        .box(0.18, 0.16, 0.095, centered=(True, True, False))
        .translate((0.0, 0.0, 0.045))
        .edges("|Z")
        .fillet(0.008)
    )
    motor_housing = (
        cq.Workplane("XY")
        .box(
            0.14,
            0.12,
            0.11,
            centered=(True, True, False),
        )
        .translate((-0.07, 0.0, 0.045))
    )
    column = cq.Workplane("XY").circle(0.065).extrude(PIVOT_Z - 0.14).translate((0.0, 0.0, 0.14))
    rotary_head = (
        cq.Workplane("XY").circle(0.09).extrude(0.035).translate((0.0, 0.0, PIVOT_Z - 0.0175))
    )
    side_rib_a = (
        cq.Workplane("XY")
        .box(0.12, 0.022, 0.17)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -27.0)
        .translate((-0.01, 0.072, 0.205))
    )
    side_rib_b = (
        cq.Workplane("XY")
        .box(0.12, 0.022, 0.17)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -27.0)
        .translate((-0.01, -0.072, 0.205))
    )
    return (
        plate.union(pedestal)
        .union(motor_housing)
        .union(column)
        .union(rotary_head)
        .union(side_rib_a)
        .union(side_rib_b)
    )


def _make_arm_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.085).extrude(0.055).translate((0.0, 0.0, -0.0275))
    rear_block = cq.Workplane("XY").box(0.09, 0.11, 0.08).translate((-0.03, 0.0, 0.0))
    beam = cq.Workplane("XY").box(0.52, 0.09, 0.065).translate((0.29, 0.0, 0.0))
    top_cap = cq.Workplane("XY").box(0.38, 0.06, 0.028).translate((0.31, 0.0, 0.038))
    underside_rib = cq.Workplane("XY").box(0.30, 0.045, 0.04).translate((0.25, 0.0, -0.045))
    nose_block = cq.Workplane("XY").box(0.05, 0.12, 0.09).translate((0.55, 0.0, 0.0))
    bearing_cap = cq.Workplane("XY").box(0.10, 0.11, 0.03).translate((0.01, 0.0, 0.035))
    return (
        hub.union(rear_block)
        .union(beam)
        .union(top_cap)
        .union(underside_rib)
        .union(nose_block)
        .union(bearing_cap)
    )


def _make_carriage_shape() -> cq.Workplane:
    sleeve = (
        cq.Workplane("XY").box(0.10, 0.14, 0.12).cut(cq.Workplane("XY").box(0.16, 0.096, 0.074))
    )
    lower_head = cq.Workplane("XY").box(0.08, 0.10, 0.07).translate((0.0, 0.0, -0.085))
    front_face = cq.Workplane("XY").box(0.05, 0.09, 0.07).translate((0.04, 0.0, -0.01))
    top_cover = cq.Workplane("XY").box(0.07, 0.12, 0.03).translate((0.0, 0.0, 0.075))
    handwheel_a = cq.Workplane("XZ").circle(0.028).extrude(0.018).translate((-0.01, 0.061, -0.01))
    handwheel_b = cq.Workplane("XZ").circle(0.028).extrude(0.018).translate((-0.01, -0.079, -0.01))
    return (
        sleeve.union(lower_head)
        .union(front_face)
        .union(top_cover)
        .union(handwheel_a)
        .union(handwheel_b)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_arm_mechanism", assets=ASSETS)

    model.material("cast_iron", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("machine_gray", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("safety_orange", rgba=(0.88, 0.43, 0.14, 1.0))
    model.material("polished_steel", rgba=(0.78, 0.80, 0.83, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base.obj", assets=ASSETS),
        material="cast_iron",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.30, 0.40)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_make_arm_shape(), "arm.obj", assets=ASSETS),
        material="machine_gray",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.66, 0.14, 0.13)),
        mass=13.5,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "carriage.obj", assets=ASSETS),
        origin=Origin(xyz=(CARRIAGE_FRAME_OFFSET_X, 0.0, 0.0)),
        material="safety_orange",
    )
    carriage.visual(
        Cylinder(radius=0.02, length=0.08),
        origin=Origin(xyz=(CARRIAGE_FRAME_OFFSET_X, 0.0, -0.16)),
        material="polished_steel",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.12, 0.16, 0.21)),
        mass=6.5,
        origin=Origin(xyz=(CARRIAGE_FRAME_OFFSET_X, 0.0, -0.04)),
    )

    model.articulation(
        "arm_swing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.2,
            upper=2.2,
            effort=120.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_HOME_X - CARRIAGE_FRAME_OFFSET_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=60.0,
            velocity=0.25,
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

    ctx.expect_origin_distance("arm", "base", axes="xy", max_dist=0.001)
    ctx.expect_origin_gap("arm", "base", axis="z", min_gap=0.35)
    ctx.expect_aabb_contact("arm", "base")
    ctx.expect_aabb_overlap("arm", "base", axes="xy", min_overlap=0.10)

    ctx.expect_origin_gap("carriage", "base", axis="z", min_gap=0.35)
    ctx.expect_aabb_contact("carriage", "arm")
    ctx.expect_aabb_overlap("carriage", "arm", axes="yz", min_overlap=0.06)

    ctx.expect_joint_motion_axis(
        "arm_swing",
        "arm",
        world_axis="y",
        direction="positive",
        min_delta=0.07,
    )
    ctx.expect_joint_motion_axis(
        "carriage_slide",
        "carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.08,
    )

    with ctx.pose(carriage_slide=CARRIAGE_TRAVEL):
        ctx.expect_aabb_contact("carriage", "arm")
        ctx.expect_aabb_overlap("carriage", "arm", axes="yz", min_overlap=0.06)
        ctx.expect_origin_gap("carriage", "base", axis="x", min_gap=0.40)

    with ctx.pose({"arm_swing": 1.25, "carriage_slide": CARRIAGE_TRAVEL}):
        ctx.expect_aabb_contact("arm", "base")
        ctx.expect_aabb_overlap("arm", "base", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_contact("carriage", "arm")
        ctx.expect_aabb_overlap("carriage", "arm", axes="yz", min_overlap=0.06)
        ctx.expect_origin_gap("carriage", "base", axis="y", min_gap=0.35)

    with ctx.pose({"arm_swing": -1.25, "carriage_slide": 0.0}):
        ctx.expect_aabb_contact("arm", "base")
        ctx.expect_aabb_contact("carriage", "arm")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
