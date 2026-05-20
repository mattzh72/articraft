from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FIRST_LINK_LENGTH = 0.300
SLIDE_ENTRY_X = 0.110
SLIDE_TRAVEL = 0.060


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_telescoping_arm")

    model.material("support_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("link_blue", rgba=(0.34, 0.45, 0.63, 1.0))
    model.material("link_silver", rgba=(0.68, 0.71, 0.74, 1.0))
    model.material("extension_steel", rgba=(0.80, 0.82, 0.85, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.180, 0.120, 0.030)),
        origin=Origin(xyz=(-0.080, 0.000, -0.185)),
        material="support_dark",
        name="base_plate",
    )
    rear_support.visual(
        Box((0.075, 0.095, 0.140)),
        origin=Origin(xyz=(-0.055, 0.000, -0.100)),
        material="support_dark",
        name="mast",
    )
    rear_support.visual(
        Box((0.055, 0.064, 0.055)),
        origin=Origin(xyz=(-0.060, 0.000, -0.015)),
        material="support_dark",
        name="rear_bridge",
    )
    rear_support.visual(
        Box((0.055, 0.016, 0.100)),
        origin=Origin(xyz=(0.012, 0.039, -0.005)),
        material="support_dark",
        name="left_cheek",
    )
    rear_support.visual(
        Box((0.055, 0.016, 0.100)),
        origin=Origin(xyz=(0.012, -0.039, -0.005)),
        material="support_dark",
        name="right_cheek",
    )
    rear_support.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.000, 0.039, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material="support_dark",
        name="left_boss",
    )
    rear_support.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.000, -0.039, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material="support_dark",
        name="right_boss",
    )
    rear_support.visual(
        Box((0.065, 0.016, 0.042)),
        origin=Origin(xyz=(-0.045, 0.039, -0.050)),
        material="support_dark",
        name="left_gusset",
    )
    rear_support.visual(
        Box((0.065, 0.016, 0.042)),
        origin=Origin(xyz=(-0.045, -0.039, -0.050)),
        material="support_dark",
        name="right_gusset",
    )

    first_link = model.part("first_link")
    first_link.visual(
        Cylinder(radius=0.022, length=0.062),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="link_blue",
        name="root_barrel",
    )
    first_link.visual(
        Box((0.040, 0.034, 0.040)),
        origin=Origin(xyz=(0.020, 0.000, 0.000)),
        material="link_blue",
        name="root_neck",
    )
    first_link.visual(
        Box((0.240, 0.036, 0.042)),
        origin=Origin(xyz=(0.145, 0.000, 0.000)),
        material="link_blue",
        name="main_beam",
    )
    first_link.visual(
        Box((0.038, 0.032, 0.038)),
        origin=Origin(xyz=(0.258, 0.000, 0.000)),
        material="link_blue",
        name="fork_bridge",
    )
    first_link.visual(
        Box((0.045, 0.012, 0.056)),
        origin=Origin(xyz=(0.292, 0.022, 0.000)),
        material="link_blue",
        name="left_fork_cheek",
    )
    first_link.visual(
        Box((0.045, 0.012, 0.056)),
        origin=Origin(xyz=(0.292, -0.022, 0.000)),
        material="link_blue",
        name="right_fork_cheek",
    )
    first_link.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.022, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material="link_blue",
        name="left_fork_boss",
    )
    first_link.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(FIRST_LINK_LENGTH, -0.022, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material="link_blue",
        name="right_fork_boss",
    )

    second_link = model.part("second_link")
    second_link.visual(
        Cylinder(radius=0.015, length=0.032),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="link_silver",
        name="root_barrel",
    )
    second_link.visual(
        Box((0.060, 0.020, 0.022)),
        origin=Origin(xyz=(0.030, 0.000, 0.000)),
        material="link_silver",
        name="root_neck",
    )
    second_link.visual(
        Box((0.082, 0.026, 0.030)),
        origin=Origin(xyz=(0.071, 0.000, 0.000)),
        material="link_silver",
        name="main_beam",
    )
    second_link.visual(
        Box((0.120, 0.008, 0.028)),
        origin=Origin(xyz=(0.170, 0.016, 0.000)),
        material="link_silver",
        name="left_rail",
    )
    second_link.visual(
        Box((0.120, 0.008, 0.028)),
        origin=Origin(xyz=(0.170, -0.016, 0.000)),
        material="link_silver",
        name="right_rail",
    )
    second_link.visual(
        Box((0.120, 0.024, 0.006)),
        origin=Origin(xyz=(0.170, 0.000, 0.017)),
        material="link_silver",
        name="top_rail",
    )
    second_link.visual(
        Box((0.120, 0.024, 0.006)),
        origin=Origin(xyz=(0.170, 0.000, -0.017)),
        material="link_silver",
        name="bottom_rail",
    )

    extension = model.part("output_extension")
    extension.visual(
        Box((0.160, 0.018, 0.020)),
        origin=Origin(xyz=(0.080, 0.000, 0.000)),
        material="extension_steel",
        name="rod",
    )
    extension.visual(
        Box((0.130, 0.003, 0.018)),
        origin=Origin(xyz=(0.065, 0.0105, 0.000)),
        material="extension_steel",
        name="left_guide",
    )
    extension.visual(
        Box((0.130, 0.003, 0.018)),
        origin=Origin(xyz=(0.065, -0.0105, 0.000)),
        material="extension_steel",
        name="right_guide",
    )
    extension.visual(
        Box((0.016, 0.030, 0.024)),
        origin=Origin(xyz=(0.152, 0.000, 0.000)),
        material="extension_steel",
        name="nose",
    )

    model.articulation(
        "rear_support_to_first_link",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=first_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=1.10, effort=35.0, velocity=1.2),
    )
    model.articulation(
        "first_link_to_second_link",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.20, effort=24.0, velocity=1.5),
    )
    model.articulation(
        "second_link_to_output_extension",
        ArticulationType.PRISMATIC,
        parent=second_link,
        child=extension,
        origin=Origin(xyz=(SLIDE_ENTRY_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_TRAVEL, effort=18.0, velocity=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    extension = object_model.get_part("output_extension")
    shoulder = object_model.get_articulation("rear_support_to_first_link")
    elbow = object_model.get_articulation("first_link_to_second_link")
    slide = object_model.get_articulation("second_link_to_output_extension")

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

    ctx.expect_origin_gap(
        first_link,
        rear_support,
        axis="x",
        min_gap=0.0,
        max_gap=0.0,
        name="first link pivots directly at the rear-support fork axis",
    )
    ctx.expect_origin_gap(
        second_link,
        rear_support,
        axis="x",
        min_gap=FIRST_LINK_LENGTH - 0.001,
        max_gap=FIRST_LINK_LENGTH + 0.001,
        name="second-link elbow sits at the first-link nose",
    )
    ctx.expect_origin_gap(
        extension,
        second_link,
        axis="x",
        min_gap=SLIDE_ENTRY_X - 0.001,
        max_gap=SLIDE_ENTRY_X + 0.001,
        name="output stage starts at the second-link slide entry",
    )
    ctx.expect_within(
        extension,
        second_link,
        axes="yz",
        margin=0.003,
        name="extension stays centered inside the sleeve at rest",
    )
    ctx.expect_overlap(
        extension,
        second_link,
        axes="x",
        min_overlap=0.100,
        name="rest pose keeps substantial extension insertion",
    )

    rest_elbow_pos = ctx.part_world_position(second_link)
    with ctx.pose({shoulder: 0.80}):
        raised_elbow_pos = ctx.part_world_position(second_link)

    ctx.check(
        "shoulder joint raises the downstream arm",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.12,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )

    rest_extension_pos = ctx.part_world_position(extension)
    with ctx.pose({elbow: 0.90}):
        elbow_lifted_extension_pos = ctx.part_world_position(extension)

    ctx.check(
        "elbow joint lifts the output stage",
        rest_extension_pos is not None
        and elbow_lifted_extension_pos is not None
        and elbow_lifted_extension_pos[2] > rest_extension_pos[2] + 0.08,
        details=f"rest={rest_extension_pos}, lifted={elbow_lifted_extension_pos}",
    )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            extension,
            second_link,
            axes="yz",
            margin=0.003,
            name="extended output stage stays centered in the sleeve",
        )
        ctx.expect_overlap(
            extension,
            second_link,
            axes="x",
            min_overlap=0.055,
            name="extended output stage still retains insertion",
        )
        extended_extension_pos = ctx.part_world_position(extension)

    ctx.check(
        "output extension translates forward along +X",
        rest_extension_pos is not None
        and extended_extension_pos is not None
        and extended_extension_pos[0] > rest_extension_pos[0] + 0.05,
        details=f"rest={rest_extension_pos}, extended={extended_extension_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
