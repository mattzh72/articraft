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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_monitor_task_lamp")

    matte_black = model.material("matte_black", rgba=(0.13, 0.13, 0.14, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.94, 0.95, 0.97, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.018, 0.060, 0.112)),
        origin=Origin(xyz=(-0.032, 0.0, 0.056)),
        material=matte_black,
        name="rear_spine",
    )
    base.visual(
        Box((0.074, 0.060, 0.014)),
        origin=Origin(xyz=(-0.001, 0.0, 0.105)),
        material=matte_black,
        name="upper_jaw",
    )
    base.visual(
        Box((0.062, 0.048, 0.014)),
        origin=Origin(xyz=(-0.007, 0.0, 0.027)),
        material=matte_black,
        name="lower_jaw",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(-0.008, 0.0, 0.122)),
        material=graphite,
        name="swivel_boss",
    )
    base.visual(
        Cylinder(radius=0.0055, length=0.070),
        origin=Origin(xyz=(-0.003, 0.0, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="clamp_screw",
    )
    base.visual(
        Cylinder(radius=0.004, length=0.036),
        origin=Origin(xyz=(-0.038, 0.0, 0.060)),
        material=graphite,
        name="screw_handle",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.034, 0.0, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="pressure_pad",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.085, 0.070, 0.140)),
        mass=1.6,
        origin=Origin(xyz=(-0.004, 0.0, 0.070)),
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=graphite,
        name="swivel_collar",
    )
    post.visual(
        Cylinder(radius=0.013, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=aluminum,
        name="post_tube",
    )
    post.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, -0.014, 0.102), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="shoulder_cheek_left",
    )
    post.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.014, 0.102), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="shoulder_cheek_right",
    )
    post.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.120)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="shoulder_barrel",
    )
    upper_arm.visual(
        Box((0.266, 0.018, 0.010)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=aluminum,
        name="upper_arm_beam",
    )
    upper_arm.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.290, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elbow_cheek_left",
    )
    upper_arm.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.290, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elbow_cheek_right",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.310, 0.050, 0.050)),
        mass=0.8,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
    )

    forearm_sleeve = model.part("forearm_sleeve")
    forearm_sleeve.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elbow_barrel",
    )
    forearm_sleeve.visual(
        Box((0.032, 0.018, 0.016)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=aluminum,
        name="sleeve_neck",
    )
    forearm_sleeve.visual(
        Box((0.176, 0.028, 0.002)),
        origin=Origin(xyz=(0.120, 0.0, 0.009)),
        material=aluminum,
        name="sleeve_top",
    )
    forearm_sleeve.visual(
        Box((0.176, 0.028, 0.002)),
        origin=Origin(xyz=(0.120, 0.0, -0.009)),
        material=aluminum,
        name="sleeve_bottom",
    )
    forearm_sleeve.visual(
        Box((0.176, 0.004, 0.018)),
        origin=Origin(xyz=(0.120, -0.012, 0.0)),
        material=aluminum,
        name="sleeve_side_left",
    )
    forearm_sleeve.visual(
        Box((0.176, 0.004, 0.018)),
        origin=Origin(xyz=(0.120, 0.012, 0.0)),
        material=aluminum,
        name="sleeve_side_right",
    )
    forearm_sleeve.inertial = Inertial.from_geometry(
        Box((0.215, 0.040, 0.045)),
        mass=0.65,
        origin=Origin(xyz=(0.108, 0.0, 0.0)),
    )

    forearm_extension = model.part("forearm_extension")
    forearm_extension.visual(
        Box((0.192, 0.018, 0.016)),
        origin=Origin(xyz=(-0.022, 0.0, 0.0)),
        material=aluminum,
        name="inner_rail",
    )
    forearm_extension.visual(
        Box((0.012, 0.010, 0.024)),
        origin=Origin(xyz=(0.092, -0.014, 0.0)),
        material=graphite,
        name="head_cheek_left",
    )
    forearm_extension.visual(
        Box((0.012, 0.010, 0.024)),
        origin=Origin(xyz=(0.092, 0.014, 0.0)),
        material=graphite,
        name="head_cheek_right",
    )
    forearm_extension.visual(
        Box((0.020, 0.004, 0.010)),
        origin=Origin(xyz=(0.082, -0.011, 0.0)),
        material=graphite,
        name="left_yoke_arm",
    )
    forearm_extension.visual(
        Box((0.020, 0.004, 0.010)),
        origin=Origin(xyz=(0.082, 0.011, 0.0)),
        material=graphite,
        name="right_yoke_arm",
    )
    forearm_extension.inertial = Inertial.from_geometry(
        Box((0.220, 0.040, 0.040)),
        mass=0.45,
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="head_barrel",
    )
    lamp_head.visual(
        Box((0.101, 0.046, 0.014)),
        origin=Origin(xyz=(0.061, 0.0, 0.0)),
        material=matte_black,
        name="head_shell",
    )
    lamp_head.visual(
        Box((0.060, 0.036, 0.008)),
        origin=Origin(xyz=(0.040, 0.0, 0.011)),
        material=graphite,
        name="heat_sink",
    )
    lamp_head.visual(
        Box((0.094, 0.038, 0.003)),
        origin=Origin(xyz=(0.068, 0.0, -0.0085)),
        material=diffuser_white,
        name="beam_diffuser",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.120, 0.055, 0.035)),
        mass=0.32,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_post_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=post,
        origin=Origin(xyz=(-0.008, 0.0, 0.132)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-1.7,
            upper=1.7,
        ),
    )
    model.articulation(
        "post_to_upper_arm_shoulder",
        ArticulationType.REVOLUTE,
        parent=post,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=-0.55,
            upper=1.25,
        ),
    )
    model.articulation(
        "upper_arm_to_forearm_elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm_sleeve,
        origin=Origin(xyz=(0.290, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=-0.25,
            upper=1.55,
        ),
    )
    model.articulation(
        "forearm_sleeve_to_extension",
        ArticulationType.PRISMATIC,
        parent=forearm_sleeve,
        child=forearm_extension,
        origin=Origin(xyz=(0.198, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.12,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "extension_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=forearm_extension,
        child=lamp_head,
        origin=Origin(xyz=(0.092, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.2,
            lower=-0.95,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post = object_model.get_part("post")
    upper_arm = object_model.get_part("upper_arm")
    forearm_sleeve = object_model.get_part("forearm_sleeve")
    forearm_extension = object_model.get_part("forearm_extension")
    lamp_head = object_model.get_part("lamp_head")

    swivel = object_model.get_articulation("base_to_post_swivel")
    shoulder = object_model.get_articulation("post_to_upper_arm_shoulder")
    elbow = object_model.get_articulation("upper_arm_to_forearm_elbow")
    telescope = object_model.get_articulation("forearm_sleeve_to_extension")
    head_tilt = object_model.get_articulation("extension_to_head_tilt")

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
        "lamp has five primary articulations",
        len(object_model.articulations) == 5,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )

    ctx.expect_contact(post, base, name="post seats on swivel boss")
    ctx.expect_contact(upper_arm, post, name="upper arm is carried by the shoulder clevis")
    ctx.expect_contact(forearm_sleeve, upper_arm, name="forearm sleeve is carried by the elbow clevis")
    ctx.expect_contact(forearm_extension, forearm_sleeve, name="extension rail bears on sleeve guides at rest")
    ctx.expect_contact(lamp_head, forearm_extension, name="lamp head is mounted in the tip clevis")

    ctx.expect_within(
        forearm_extension,
        forearm_sleeve,
        axes="yz",
        inner_elem="inner_rail",
        margin=0.0,
        name="extension stays centered inside the forearm sleeve at rest",
    )
    ctx.expect_overlap(
        forearm_extension,
        forearm_sleeve,
        axes="x",
        elem_a="inner_rail",
        min_overlap=0.050,
        name="extension keeps meaningful insertion in the forearm sleeve at rest",
    )

    def _elem_center(part_obj, elem_name):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((a + b) * 0.5 for a, b in zip(lower, upper))

    swivel_rest = ctx.part_world_position(forearm_sleeve)
    with ctx.pose({swivel: 0.9}):
        swivel_turned = ctx.part_world_position(forearm_sleeve)
    ctx.check(
        "swivel turns the arm around the clamp post",
        swivel_rest is not None
        and swivel_turned is not None
        and abs(swivel_turned[1] - swivel_rest[1]) > 0.18,
        details=f"rest={swivel_rest}, turned={swivel_turned}",
    )

    shoulder_rest = ctx.part_world_position(forearm_sleeve)
    with ctx.pose({shoulder: 0.8}):
        shoulder_raised = ctx.part_world_position(forearm_sleeve)
    ctx.check(
        "shoulder lifts the forearm upward",
        shoulder_rest is not None
        and shoulder_raised is not None
        and shoulder_raised[2] > shoulder_rest[2] + 0.17,
        details=f"rest={shoulder_rest}, raised={shoulder_raised}",
    )

    elbow_rest = _elem_center(lamp_head, "beam_diffuser")
    with ctx.pose({elbow: 0.95}):
        elbow_raised = _elem_center(lamp_head, "beam_diffuser")
    ctx.check(
        "elbow lifts the lamp head",
        elbow_rest is not None
        and elbow_raised is not None
        and elbow_raised[2] > elbow_rest[2] + 0.12,
        details=f"rest={elbow_rest}, raised={elbow_raised}",
    )

    extension_rest = ctx.part_world_position(forearm_extension)
    with ctx.pose({telescope: telescope.motion_limits.upper or 0.0}):
        extension_extended = ctx.part_world_position(forearm_extension)
        ctx.expect_within(
            forearm_extension,
            forearm_sleeve,
            axes="yz",
            inner_elem="inner_rail",
            margin=0.0,
            name="extension stays centered inside the forearm sleeve when extended",
        )
        ctx.expect_overlap(
            forearm_extension,
            forearm_sleeve,
            axes="x",
            elem_a="inner_rail",
            min_overlap=0.040,
            name="extension keeps retained insertion when extended",
        )
    ctx.check(
        "telescoping stage extends forward",
        extension_rest is not None
        and extension_extended is not None
        and extension_extended[0] > extension_rest[0] + 0.06,
        details=f"rest={extension_rest}, extended={extension_extended}",
    )

    tilt_rest = _elem_center(lamp_head, "beam_diffuser")
    with ctx.pose({head_tilt: -0.6}):
        tilt_down = _elem_center(lamp_head, "beam_diffuser")
    ctx.check(
        "head tilt pitches the LED head downward",
        tilt_rest is not None
        and tilt_down is not None
        and tilt_down[2] < tilt_rest[2] - 0.03,
        details=f"rest={tilt_rest}, tilted={tilt_down}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
