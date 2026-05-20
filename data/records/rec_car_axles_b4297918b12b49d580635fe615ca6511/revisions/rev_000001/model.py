from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="de_dion_rear_axle")

    axle_steel = model.material("axle_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.58, 0.60, 0.64, 1.0))
    hub_finish = model.material("hub_finish", rgba=(0.30, 0.31, 0.34, 1.0))

    beam_tube_radius = 0.042
    beam_tube_length = 0.49
    cross_trunnion_radius = 0.022
    cross_pitch_span = 0.120
    roll_trunnion_radius = 0.020
    roll_trunnion_span = 0.110
    beam_bearing_length = 0.024
    beam_bearing_center_x = 0.067
    tube_inner_end_x = beam_bearing_center_x + beam_bearing_length / 2.0
    spindle_mount_radius = beam_tube_radius
    spindle_joint_x = tube_inner_end_x + beam_tube_length
    spindle_flange_offset = 0.134

    coupling_block = model.part("coupling_block")
    coupling_block.visual(
        Box((0.10, 0.16, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=axle_steel,
        name="upper_bridge",
    )
    coupling_block.visual(
        Box((0.10, 0.16, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=axle_steel,
        name="lower_bridge",
    )
    coupling_block.visual(
        Box((0.10, 0.014, 0.204)),
        origin=Origin(xyz=(0.0, 0.067, 0.0)),
        material=axle_steel,
        name="front_lug",
    )
    coupling_block.visual(
        Box((0.10, 0.014, 0.204)),
        origin=Origin(xyz=(0.0, -0.067, 0.0)),
        material=axle_steel,
        name="rear_lug",
    )

    coupling_cross = model.part("coupling_cross")
    coupling_cross.visual(
        Box((0.040, 0.040, 0.040)),
        material=axle_steel,
        name="gimbal_core",
    )
    coupling_cross.visual(
        Cylinder(radius=cross_trunnion_radius, length=cross_pitch_span),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="pitch_trunnion",
    )
    coupling_cross.visual(
        Cylinder(radius=roll_trunnion_radius, length=roll_trunnion_span),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_steel,
        name="roll_trunnion",
    )

    axle_beam = model.part("axle_beam")
    axle_beam.visual(
        Box((beam_bearing_length, 0.050, 0.050)),
        origin=Origin(xyz=(-beam_bearing_center_x, 0.0, 0.0)),
        material=machined_steel,
        name="left_bearing",
    )
    axle_beam.visual(
        Box((beam_bearing_length, 0.050, 0.050)),
        origin=Origin(xyz=(beam_bearing_center_x, 0.0, 0.0)),
        material=machined_steel,
        name="right_bearing",
    )
    axle_beam.visual(
        Box((0.028, 0.020, 0.106)),
        origin=Origin(xyz=(-0.185, 0.0, -0.095)),
        material=axle_steel,
        name="left_drop_link",
    )
    axle_beam.visual(
        Box((0.028, 0.020, 0.106)),
        origin=Origin(xyz=(0.185, 0.0, -0.095)),
        material=axle_steel,
        name="right_drop_link",
    )
    axle_beam.visual(
        Box((0.398, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.148)),
        material=axle_steel,
        name="lower_tie_beam",
    )
    axle_beam.visual(
        Cylinder(radius=beam_tube_radius, length=beam_tube_length),
        origin=Origin(
            xyz=(-(tube_inner_end_x + beam_tube_length / 2.0), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=axle_steel,
        name="left_tube",
    )
    axle_beam.visual(
        Cylinder(radius=beam_tube_radius, length=beam_tube_length),
        origin=Origin(
            xyz=(tube_inner_end_x + beam_tube_length / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=axle_steel,
        name="right_tube",
    )

    left_spindle = model.part("left_spindle")
    left_spindle.visual(
        Cylinder(radius=spindle_mount_radius, length=0.028),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_steel,
        name="inner_mount",
    )
    left_spindle.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(-0.037, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="neck",
    )
    left_spindle.visual(
        Cylinder(radius=0.028, length=0.070),
        origin=Origin(xyz=(-0.081, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="stub_shaft",
    )
    left_spindle.visual(
        Cylinder(radius=0.047, length=0.018),
        origin=Origin(xyz=(-0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="flange",
    )

    right_spindle = model.part("right_spindle")
    right_spindle.visual(
        Cylinder(radius=spindle_mount_radius, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_steel,
        name="inner_mount",
    )
    right_spindle.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.037, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="neck",
    )
    right_spindle.visual(
        Cylinder(radius=0.028, length=0.070),
        origin=Origin(xyz=(0.081, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="stub_shaft",
    )
    right_spindle.visual(
        Cylinder(radius=0.047, length=0.018),
        origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="flange",
    )

    left_hub = model.part("left_hub")
    left_hub.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_finish,
        name="inboard_collar",
    )
    left_hub.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_finish,
        name="hub_drum",
    )
    left_hub.visual(
        Cylinder(radius=0.034, length=0.032),
        origin=Origin(xyz=(-0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="hub_nose",
    )

    right_hub = model.part("right_hub")
    right_hub.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_finish,
        name="inboard_collar",
    )
    right_hub.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_finish,
        name="hub_drum",
    )
    right_hub.visual(
        Cylinder(radius=0.034, length=0.032),
        origin=Origin(xyz=(0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="hub_nose",
    )

    model.articulation(
        "coupling_pitch",
        ArticulationType.REVOLUTE,
        parent=coupling_block,
        child=coupling_cross,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=2.0,
            lower=-0.42,
            upper=0.42,
        ),
    )
    model.articulation(
        "coupling_roll",
        ArticulationType.REVOLUTE,
        parent=coupling_cross,
        child=axle_beam,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=2.5,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "left_spindle_mount",
        ArticulationType.FIXED,
        parent=axle_beam,
        child=left_spindle,
        origin=Origin(xyz=(-spindle_joint_x, 0.0, 0.0)),
    )
    model.articulation(
        "right_spindle_mount",
        ArticulationType.FIXED,
        parent=axle_beam,
        child=right_spindle,
        origin=Origin(xyz=(spindle_joint_x, 0.0, 0.0)),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=left_spindle,
        child=left_hub,
        origin=Origin(xyz=(-spindle_flange_offset, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=80.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=right_spindle,
        child=right_hub,
        origin=Origin(xyz=(spindle_flange_offset, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    coupling_block = object_model.get_part("coupling_block")
    coupling_cross = object_model.get_part("coupling_cross")
    axle_beam = object_model.get_part("axle_beam")
    left_spindle = object_model.get_part("left_spindle")
    right_spindle = object_model.get_part("right_spindle")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")

    pitch_joint = object_model.get_articulation("coupling_pitch")
    roll_joint = object_model.get_articulation("coupling_roll")
    left_hub_joint = object_model.get_articulation("left_hub_spin")
    right_hub_joint = object_model.get_articulation("right_hub_spin")

    front_lug = coupling_block.get_visual("front_lug")
    rear_lug = coupling_block.get_visual("rear_lug")
    pitch_trunnion = coupling_cross.get_visual("pitch_trunnion")
    roll_trunnion = coupling_cross.get_visual("roll_trunnion")
    left_bearing = axle_beam.get_visual("left_bearing")
    right_bearing = axle_beam.get_visual("right_bearing")
    left_tube = axle_beam.get_visual("left_tube")
    right_tube = axle_beam.get_visual("right_tube")
    left_mount = left_spindle.get_visual("inner_mount")
    right_mount = right_spindle.get_visual("inner_mount")
    left_flange = left_spindle.get_visual("flange")
    right_flange = right_spindle.get_visual("flange")
    left_hub_collar = left_hub.get_visual("inboard_collar")
    right_hub_collar = right_hub.get_visual("inboard_collar")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(max_pose_samples=8)
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
        "pitch_joint_axis_and_limits",
        tuple(pitch_joint.axis) == (0.0, 1.0, 0.0)
        and pitch_joint.motion_limits is not None
        and pitch_joint.motion_limits.lower == -0.42
        and pitch_joint.motion_limits.upper == 0.42,
        details=f"unexpected pitch joint configuration: {pitch_joint.axis} {pitch_joint.motion_limits}",
    )
    ctx.check(
        "roll_joint_axis_and_limits",
        tuple(roll_joint.axis) == (1.0, 0.0, 0.0)
        and roll_joint.motion_limits is not None
        and roll_joint.motion_limits.lower == -0.35
        and roll_joint.motion_limits.upper == 0.35,
        details=f"unexpected roll joint configuration: {roll_joint.axis} {roll_joint.motion_limits}",
    )
    ctx.check(
        "hub_spin_joint_axes",
        tuple(left_hub_joint.axis) == (-1.0, 0.0, 0.0)
        and tuple(right_hub_joint.axis) == (1.0, 0.0, 0.0),
        details=f"unexpected hub spin axes: left={left_hub_joint.axis} right={right_hub_joint.axis}",
    )

    ctx.expect_contact(
        coupling_block,
        coupling_cross,
        elem_a=front_lug,
        elem_b=pitch_trunnion,
        contact_tol=1e-5,
        name="front_pitch_trunnion_contact",
    )
    ctx.expect_contact(
        coupling_block,
        coupling_cross,
        elem_a=rear_lug,
        elem_b=pitch_trunnion,
        contact_tol=1e-5,
        name="rear_pitch_trunnion_contact",
    )
    ctx.expect_contact(
        coupling_cross,
        axle_beam,
        elem_a=roll_trunnion,
        elem_b=left_bearing,
        contact_tol=1e-5,
        name="left_roll_trunnion_contact",
    )
    ctx.expect_contact(
        coupling_cross,
        axle_beam,
        elem_a=roll_trunnion,
        elem_b=right_bearing,
        contact_tol=1e-5,
        name="right_roll_trunnion_contact",
    )
    ctx.expect_contact(
        axle_beam,
        left_spindle,
        elem_a=left_tube,
        elem_b=left_mount,
        contact_tol=1e-5,
        name="left_spindle_mounted_to_beam",
    )
    ctx.expect_contact(
        axle_beam,
        right_spindle,
        elem_a=right_tube,
        elem_b=right_mount,
        contact_tol=1e-5,
        name="right_spindle_mounted_to_beam",
    )
    ctx.expect_contact(
        left_spindle,
        left_hub,
        elem_a=left_flange,
        elem_b=left_hub_collar,
        contact_tol=1e-5,
        name="left_hub_flange_contact",
    )
    ctx.expect_contact(
        right_spindle,
        right_hub,
        elem_a=right_flange,
        elem_b=right_hub_collar,
        contact_tol=1e-5,
        name="right_hub_flange_contact",
    )
    ctx.expect_overlap(
        left_spindle,
        left_hub,
        axes="yz",
        min_overlap=0.045,
        elem_a=left_flange,
        elem_b=left_hub_collar,
        name="left_hub_coaxial_overlap",
    )
    ctx.expect_overlap(
        right_spindle,
        right_hub,
        axes="yz",
        min_overlap=0.045,
        elem_a=right_flange,
        elem_b=right_hub_collar,
        name="right_hub_coaxial_overlap",
    )
    ctx.expect_origin_distance(
        left_spindle,
        right_spindle,
        axes="x",
        min_dist=1.12,
        max_dist=1.14,
        name="beam_stub_span",
    )
    ctx.expect_origin_distance(
        left_hub,
        right_hub,
        axes="x",
        min_dist=1.37,
        max_dist=1.42,
        name="rear_track_width",
    )
    ctx.expect_origin_gap(
        right_hub,
        left_hub,
        axis="x",
        min_gap=1.37,
        max_gap=1.42,
        name="hub_spacing_positive_x",
    )
    ctx.expect_origin_gap(
        left_spindle,
        left_hub,
        axis="x",
        min_gap=0.13,
        max_gap=0.14,
        name="left_hub_outboard_of_spindle",
    )
    ctx.expect_origin_gap(
        right_hub,
        right_spindle,
        axis="x",
        min_gap=0.13,
        max_gap=0.14,
        name="right_hub_outboard_of_spindle",
    )

    pitch_limits = pitch_joint.motion_limits
    if pitch_limits is not None and pitch_limits.lower is not None and pitch_limits.upper is not None:
        with ctx.pose({pitch_joint: pitch_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pitch_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="pitch_lower_no_floating")
            ctx.expect_contact(
                coupling_block,
                coupling_cross,
                elem_a=front_lug,
                elem_b=pitch_trunnion,
                contact_tol=1e-5,
                name="pitch_lower_front_contact",
            )
        with ctx.pose({pitch_joint: pitch_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pitch_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="pitch_upper_no_floating")
            ctx.expect_contact(
                coupling_block,
                coupling_cross,
                elem_a=rear_lug,
                elem_b=pitch_trunnion,
                contact_tol=1e-5,
                name="pitch_upper_rear_contact",
            )

    roll_limits = roll_joint.motion_limits
    if roll_limits is not None and roll_limits.lower is not None and roll_limits.upper is not None:
        with ctx.pose({roll_joint: roll_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="roll_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="roll_lower_no_floating")
            ctx.expect_contact(
                coupling_cross,
                axle_beam,
                elem_a=roll_trunnion,
                elem_b=left_bearing,
                contact_tol=1e-5,
                name="roll_lower_left_contact",
            )
        with ctx.pose({roll_joint: roll_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="roll_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="roll_upper_no_floating")
            ctx.expect_contact(
                coupling_cross,
                axle_beam,
                elem_a=roll_trunnion,
                elem_b=right_bearing,
                contact_tol=1e-5,
                name="roll_upper_right_contact",
            )

    with ctx.pose(
        {
            pitch_joint: 0.30,
            roll_joint: -0.22,
            left_hub_joint: math.pi / 2.0,
            right_hub_joint: -math.pi / 3.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")
        ctx.expect_contact(
            left_spindle,
            left_hub,
            elem_a=left_flange,
            elem_b=left_hub_collar,
            contact_tol=1e-5,
            name="left_hub_spun_contact",
        )
        ctx.expect_contact(
            right_spindle,
            right_hub,
            elem_a=right_flange,
            elem_b=right_hub_collar,
            contact_tol=1e-5,
            name="right_hub_spun_contact",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
