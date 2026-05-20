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


SUPPORT_PLATE_X = 0.18
SUPPORT_PLATE_Y = 0.11
SUPPORT_PLATE_Z = 0.012
SUPPORT_DROP_Z = 0.052
JOINT_TAB_WIDTH = 0.018
FORK_WIDTH = 0.038
FORK_EAR_THICKNESS = 0.010
LINK_THICKNESS = 0.018
UPPER_LINK_LENGTH = 0.21
LOWER_LINK_LENGTH = 0.17
HEAD_NECK_LENGTH = 0.055


def make_support_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(SUPPORT_PLATE_X, SUPPORT_PLATE_Y, SUPPORT_PLATE_Z).translate(
        (-0.118, 0.0, 0.058)
    )
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.050, -0.030),
                (-0.050, 0.030),
                (0.050, -0.030),
                (0.050, 0.030),
            ]
        )
        .slot2D(0.022, 0.007, 0.0)
        .cutThruAll()
    )

    mount_block = (
        cq.Workplane("XY")
        .box(0.066, 0.060, 0.018)
        .translate((-0.072, 0.0, 0.043))
    )
    drop_web = (
        cq.Workplane("XY")
        .box(0.018, 0.036, SUPPORT_DROP_Z)
        .translate((-0.022, 0.0, SUPPORT_DROP_Z / 2.0))
    )
    front_bridge = (
        cq.Workplane("XY")
        .box(0.012, 0.034, 0.018)
        .translate((-0.006, 0.0, 0.010))
    )
    ear_offset = JOINT_TAB_WIDTH / 2.0 + FORK_EAR_THICKNESS / 2.0
    left_ear = (
        cq.Workplane("XY")
        .box(0.012, FORK_EAR_THICKNESS, 0.022)
        .translate((-0.006, ear_offset, 0.0))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.012, FORK_EAR_THICKNESS, 0.022)
        .translate((-0.006, -ear_offset, 0.0))
    )
    left_gusset = (
        cq.Workplane("XZ")
        .moveTo(-0.050, 0.008)
        .lineTo(-0.050, 0.044)
        .lineTo(0.000, 0.024)
        .lineTo(0.000, 0.008)
        .close()
        .extrude(0.010)
        .translate((0.0, 0.015, 0.0))
    )
    right_gusset = (
        cq.Workplane("XZ")
        .moveTo(-0.050, 0.008)
        .lineTo(-0.050, 0.044)
        .lineTo(0.000, 0.024)
        .lineTo(0.000, 0.008)
        .close()
        .extrude(0.010)
        .translate((0.0, -0.025, 0.0))
    )

    return (
        plate.union(mount_block)
        .union(drop_web)
        .union(front_bridge)
        .union(left_ear)
        .union(right_ear)
        .union(left_gusset)
        .union(right_gusset)
    )


def make_link_shape(length: float, name: str) -> cq.Workplane:
    beam_width = 0.026 if name == "upper" else 0.024
    root_lug = (
        cq.Workplane("XY")
        .box(0.030, JOINT_TAB_WIDTH, 0.022)
        .translate((0.015, 0.0, 0.0))
    )
    end_lug = (
        cq.Workplane("XY")
        .box(0.030, JOINT_TAB_WIDTH, 0.022)
        .translate((length - 0.015, 0.0, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(length - 0.090, beam_width, 0.014)
        .translate((length / 2.0, 0.0, 0.0))
    )
    front_shoulder = (
        cq.Workplane("XY")
        .box(0.034, 0.022, 0.018)
        .translate((0.045, 0.0, 0.0))
    )
    rear_shoulder = (
        cq.Workplane("XY")
        .box(0.034, 0.022, 0.018)
        .translate((length - 0.045, 0.0, 0.0))
    )
    lower_spine = (
        cq.Workplane("XY")
        .box(length - 0.060, 0.012, 0.006)
        .translate((length / 2.0, 0.0, -0.006))
    )
    return root_lug.union(front_shoulder).union(beam).union(rear_shoulder).union(end_lug).union(
        lower_spine
    )


def make_swivel_shape() -> cq.Workplane:
    base_block = (
        cq.Workplane("XY")
        .box(0.018, 0.028, 0.022)
        .translate((0.009, 0.0, 0.0))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.028, 0.022, 0.018)
        .translate((0.032, 0.0, 0.0))
    )
    yoke = (
        cq.Workplane("XY")
        .box(0.020, 0.026, 0.040)
        .translate((HEAD_NECK_LENGTH - 0.010, 0.0, 0.0))
    )
    front_pivot_block = (
        cq.Workplane("XY")
        .box(0.014, 0.026, 0.022)
        .translate((HEAD_NECK_LENGTH - 0.003, 0.0, 0.0))
    )
    lower_web = (
        cq.Workplane("XZ")
        .moveTo(0.018, -0.010)
        .lineTo(0.044, -0.010)
        .lineTo(0.052, 0.000)
        .lineTo(0.028, 0.006)
        .close()
        .extrude(0.010, both=True)
    )
    return base_block.union(neck).union(yoke).union(front_pivot_block).union(lower_web)


def make_head_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.008, 0.120, 0.120)
        .translate((0.028, 0.0, 0.0))
        .edges("|X")
        .fillet(0.010)
    )
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.0375, -0.0375),
                (-0.0375, 0.0375),
                (0.0375, -0.0375),
                (0.0375, 0.0375),
            ]
        )
        .hole(0.006)
    )
    center_pad = (
        cq.Workplane("XY")
        .box(0.018, 0.048, 0.048)
        .translate((0.016, 0.0, 0.0))
    )
    tilt_block = (
        cq.Workplane("XY")
        .box(0.014, JOINT_TAB_WIDTH, 0.022)
        .translate((0.007, 0.0, 0.0))
    )
    lower_rib = (
        cq.Workplane("XZ")
        .moveTo(0.010, -0.015)
        .lineTo(0.024, -0.050)
        .lineTo(0.024, -0.036)
        .lineTo(0.010, -0.010)
        .close()
        .extrude(0.016, both=True)
    )
    return plate.union(center_pad).union(tilt_block).union(lower_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_shelf_tv_wall_arm")

    model.material("powder_black", color=(0.15, 0.15, 0.16, 1.0))
    model.material("graphite", color=(0.23, 0.24, 0.26, 1.0))
    model.material("dark_steel", color=(0.45, 0.47, 0.50, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(make_support_shape(), "support_body"),
        material="powder_black",
        name="support_body",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(make_link_shape(UPPER_LINK_LENGTH, "upper"), "upper_link"),
        material="graphite",
        name="upper_link_body",
    )

    lower_link = model.part("lower_link")
    lower_link.visual(
        mesh_from_cadquery(make_link_shape(LOWER_LINK_LENGTH, "lower"), "lower_link"),
        material="graphite",
        name="lower_link_body",
    )

    swivel = model.part("swivel_head")
    swivel.visual(
        mesh_from_cadquery(make_swivel_shape(), "swivel_head"),
        material="powder_black",
        name="swivel_head_body",
    )

    head = model.part("monitor_head")
    head.visual(
        mesh_from_cadquery(make_head_shape(), "monitor_head"),
        material="dark_steel",
        name="head_plate",
    )

    model.articulation(
        "support_to_upper",
        ArticulationType.REVOLUTE,
        parent=support,
        child=upper_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=2.0,
            lower=-2.80,
            upper=2.80,
        ),
    )
    model.articulation(
        "upper_to_lower",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=lower_link,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=-2.60,
            upper=2.60,
        ),
    )
    model.articulation(
        "lower_to_swivel",
        ArticulationType.REVOLUTE,
        parent=lower_link,
        child=swivel,
        origin=Origin(xyz=(LOWER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-1.80,
            upper=1.80,
        ),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(HEAD_NECK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-0.80,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    upper_link = object_model.get_part("upper_link")
    lower_link = object_model.get_part("lower_link")
    swivel = object_model.get_part("swivel_head")
    head = object_model.get_part("monitor_head")

    shoulder = object_model.get_articulation("support_to_upper")
    elbow = object_model.get_articulation("upper_to_lower")
    swivel_joint = object_model.get_articulation("lower_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")

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

    ctx.check(
        "all_expected_parts_present",
        all(part is not None for part in (support, upper_link, lower_link, swivel, head)),
        "Expected support, upper_link, lower_link, swivel_head, and monitor_head parts.",
    )

    ctx.check(
        "arm_joint_axes",
        shoulder.axis == (0.0, 0.0, 1.0)
        and elbow.axis == (0.0, 0.0, 1.0)
        and swivel_joint.axis == (0.0, 0.0, 1.0)
        and tilt_joint.axis == (0.0, 1.0, 0.0),
        (
            f"Unexpected axes: shoulder={shoulder.axis}, elbow={elbow.axis}, "
            f"swivel={swivel_joint.axis}, tilt={tilt_joint.axis}"
        ),
    )

    ctx.expect_contact(
        support,
        upper_link,
        contact_tol=0.0008,
        name="support_contacts_upper_link",
    )
    ctx.expect_contact(
        upper_link,
        lower_link,
        contact_tol=0.0008,
        name="upper_link_contacts_lower_link",
    )
    ctx.expect_contact(
        lower_link,
        swivel,
        contact_tol=0.0008,
        name="lower_link_contacts_swivel",
    )
    ctx.expect_contact(
        swivel,
        head,
        contact_tol=0.0008,
        name="swivel_contacts_head",
    )

    ctx.expect_origin_distance(
        head,
        support,
        axes="xy",
        min_dist=0.40,
        name="extended_pose_reaches_out_from_support",
    )

    with ctx.pose(
        {
            shoulder: 2.75,
            elbow: -2.55,
            swivel_joint: -1.15,
            tilt_joint: 0.35,
        }
    ):
        ctx.expect_origin_distance(
            head,
            support,
            axes="xy",
            max_dist=0.16,
            name="folded_pose_brings_head_close_to_support",
        )
        ctx.expect_overlap(
            head,
            support,
            axes="y",
            min_overlap=0.05,
            name="folded_head_stays_under_support_width",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
