from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
from math import pi

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
)

ASSETS = AssetContext.from_script(__file__)


def _add_box(part, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material)


def _add_cylinder(part, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry_axis", assets=ASSETS)

    model.material("frame_aluminum", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("rail_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("carriage_body", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("slide_body", rgba=(0.86, 0.87, 0.88, 1.0))
    model.material("accent_blue", rgba=(0.15, 0.42, 0.78, 1.0))
    model.material("tool_steel", rgba=(0.42, 0.44, 0.48, 1.0))

    frame = model.part("frame")
    _add_box(frame, (0.20, 0.16, 0.04), (-0.27, 0.0, 0.02), "frame_aluminum")
    _add_box(frame, (0.20, 0.16, 0.04), (0.27, 0.0, 0.02), "frame_aluminum")
    _add_box(frame, (0.08, 0.12, 0.582), (-0.27, 0.0, 0.329), "frame_aluminum")
    _add_box(frame, (0.08, 0.12, 0.582), (0.27, 0.0, 0.329), "frame_aluminum")
    _add_box(frame, (0.62, 0.12, 0.08), (0.0, 0.0, 0.66), "frame_aluminum")
    _add_box(frame, (0.36, 0.08, 0.06), (0.0, 0.0, 0.07), "frame_aluminum")
    _add_box(frame, (0.46, 0.06, 0.05), (0.0, -0.02, 0.18), "frame_aluminum")
    _add_box(frame, (0.56, 0.012, 0.018), (0.0, 0.054, 0.682), "rail_steel")
    _add_box(frame, (0.56, 0.012, 0.018), (0.0, 0.054, 0.638), "rail_steel")
    _add_box(frame, (0.10, 0.04, 0.03), (-0.27, 0.0, 0.665), "accent_blue")
    _add_box(frame, (0.10, 0.04, 0.03), (0.27, 0.0, 0.665), "accent_blue")
    frame.inertial = Inertial.from_geometry(
        Box((0.74, 0.18, 0.70)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    carriage = model.part("carriage")
    _add_box(carriage, (0.17, 0.086, 0.14), (0.0, 0.0, 0.0), "carriage_body")
    _add_box(carriage, (0.16, 0.052, 0.44), (0.0, 0.018, -0.29), "slide_body")
    _add_box(carriage, (0.024, 0.014, 0.40), (-0.05, 0.032, -0.27), "rail_steel")
    _add_box(carriage, (0.024, 0.014, 0.40), (0.05, 0.032, -0.27), "rail_steel")
    _add_box(carriage, (0.032, 0.022, 0.42), (0.0, 0.03, -0.27), "carriage_body")
    _add_box(carriage, (0.09, 0.05, 0.06), (0.0, -0.005, 0.095), "accent_blue")
    _add_box(carriage, (0.11, 0.028, 0.02), (0.0, -0.028, 0.04), "tool_steel")
    _add_box(carriage, (0.11, 0.028, 0.02), (0.0, -0.028, -0.04), "tool_steel")
    carriage.inertial = Inertial.from_geometry(
        Box((0.18, 0.10, 0.57)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.01, -0.19)),
    )

    slide = model.part("slide")
    _add_box(slide, (0.13, 0.036, 0.26), (0.0, 0.0, -0.12), "slide_body")
    _add_box(slide, (0.10, 0.052, 0.18), (0.0, -0.02, -0.12), "carriage_body")
    _add_box(slide, (0.035, 0.022, 0.05), (-0.05, 0.0, -0.05), "tool_steel")
    _add_box(slide, (0.035, 0.022, 0.05), (0.05, 0.0, -0.05), "tool_steel")
    _add_box(slide, (0.035, 0.022, 0.05), (-0.05, 0.0, -0.17), "tool_steel")
    _add_box(slide, (0.035, 0.022, 0.05), (0.05, 0.0, -0.17), "tool_steel")
    _add_box(slide, (0.082, 0.054, 0.11), (0.0, 0.018, -0.305), "carriage_body")
    _add_box(slide, (0.10, 0.018, 0.028), (0.0, 0.004, -0.215), "accent_blue")
    _add_cylinder(
        slide,
        radius=0.018,
        length=0.045,
        xyz=(0.0, 0.045, -0.33),
        material="tool_steel",
        rpy=(pi / 2.0, 0.0, 0.0),
    )
    slide.inertial = Inertial.from_geometry(
        Box((0.14, 0.09, 0.37)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.018, -0.19)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.055, 0.66)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.13,
            upper=0.13,
            effort=450.0,
            velocity=0.8,
        ),
    )
    model.articulation(
        "carriage_to_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=slide,
        origin=Origin(xyz=(0.0, 0.032, -0.08)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.18,
            effort=220.0,
            velocity=0.5,
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
        max_pose_samples=96,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_joint_motion_axis(
        "frame_to_carriage",
        "carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "carriage_to_slide",
        "slide",
        world_axis="z",
        direction="negative",
        min_delta=0.05,
    )

    ctx.expect_origin_distance("carriage", "frame", axes="x", max_dist=0.02)
    ctx.expect_origin_distance("slide", "carriage", axes="x", max_dist=0.002)
    ctx.expect_aabb_contact("carriage", "frame")
    ctx.expect_aabb_contact("slide", "carriage")
    ctx.expect_aabb_overlap("carriage", "frame", axes="yz", min_overlap=0.06)
    ctx.expect_aabb_overlap("slide", "carriage", axes="xy", min_overlap=0.045)

    with ctx.pose(frame_to_carriage=-0.13):
        ctx.expect_aabb_contact("carriage", "frame")
        ctx.expect_aabb_overlap("carriage", "frame", axes="yz", min_overlap=0.06)
        ctx.expect_origin_distance("slide", "carriage", axes="x", max_dist=0.002)

    with ctx.pose(frame_to_carriage=0.13):
        ctx.expect_aabb_contact("carriage", "frame")
        ctx.expect_aabb_overlap("carriage", "frame", axes="yz", min_overlap=0.06)
        ctx.expect_origin_distance("slide", "carriage", axes="x", max_dist=0.002)

    with ctx.pose(carriage_to_slide=0.18):
        ctx.expect_aabb_contact("slide", "carriage")
        ctx.expect_aabb_overlap("slide", "carriage", axes="xy", min_overlap=0.045)

    with ctx.pose(frame_to_carriage=0.13, carriage_to_slide=0.18):
        ctx.expect_aabb_contact("carriage", "frame")
        ctx.expect_aabb_contact("slide", "carriage")
        ctx.expect_aabb_overlap("slide", "carriage", axes="xy", min_overlap=0.045)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
