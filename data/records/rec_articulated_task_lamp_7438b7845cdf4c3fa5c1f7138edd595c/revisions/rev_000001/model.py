from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_base_monitor_mount_task_lamp")

    matte_black = model.material("matte_black", rgba=(0.005, 0.006, 0.007, 1.0))
    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.10, 0.105, 0.11, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.55, 0.55, 0.52, 1.0))
    rubber = model.material("rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    led_diffuser = model.material("warm_led_diffuser", rgba=(1.0, 0.88, 0.58, 1.0))

    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        Box((0.040, 0.120, 0.200)),
        origin=Origin(xyz=(-0.015, 0.0, 0.100)),
        material=matte_black,
        name="rear_spine",
    )
    clamp_base.visual(
        Box((0.200, 0.120, 0.035)),
        origin=Origin(xyz=(0.080, 0.0, 0.200)),
        material=matte_black,
        name="top_jaw",
    )
    clamp_base.visual(
        Box((0.200, 0.120, 0.035)),
        origin=Origin(xyz=(0.080, 0.0, 0.020)),
        material=matte_black,
        name="lower_jaw",
    )
    clamp_base.visual(
        Box((0.090, 0.090, 0.012)),
        origin=Origin(xyz=(0.138, 0.0, 0.180)),
        material=rubber,
        name="upper_grip_pad",
    )
    clamp_base.visual(
        Cylinder(radius=0.009, length=0.132),
        origin=Origin(xyz=(0.145, 0.0, 0.084)),
        material=brushed_steel,
        name="clamp_screw",
    )
    clamp_base.visual(
        Cylinder(radius=0.041, length=0.012),
        origin=Origin(xyz=(0.145, 0.0, 0.153)),
        material=rubber,
        name="pressure_pad",
    )
    clamp_base.visual(
        Cylinder(radius=0.006, length=0.105),
        origin=Origin(xyz=(0.145, 0.0, 0.014), rpy=(pi / 2, 0.0, 0.0)),
        material=brushed_steel,
        name="tommy_bar",
    )
    clamp_base.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.020, 0.0, 0.2425)),
        material=dark_metal,
        name="swivel_boss",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.021, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=satin_black,
        name="post_shaft",
    )
    post.visual(
        Box((0.050, 0.132, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=satin_black,
        name="shoulder_yoke_bridge",
    )
    post.visual(
        Box((0.052, 0.014, 0.066)),
        origin=Origin(xyz=(0.0, 0.068, 0.176)),
        material=satin_black,
        name="shoulder_cheek_0",
    )
    post.visual(
        Box((0.052, 0.014, 0.066)),
        origin=Origin(xyz=(0.0, -0.068, 0.176)),
        material=satin_black,
        name="shoulder_cheek_1",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.021, length=0.122),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="shoulder_hub",
    )
    for index, y in enumerate((-0.044, 0.044)):
        upper_arm.visual(
            Cylinder(radius=0.008, length=0.340),
            origin=Origin(xyz=(0.190, y, 0.0), rpy=(0.0, pi / 2, 0.0)),
            material=satin_black,
            name=f"side_rail_{index}",
        )
    upper_arm.visual(
        Box((0.035, 0.020, 0.030)),
        origin=Origin(xyz=(0.350, 0.044, 0.0)),
        material=satin_black,
        name="elbow_fork_0",
    )
    upper_arm.visual(
        Box((0.035, 0.020, 0.030)),
        origin=Origin(xyz=(0.350, -0.044, 0.0)),
        material=satin_black,
        name="elbow_fork_1",
    )

    outer_profile = rounded_rect_profile(0.056, 0.080, 0.006, corner_segments=6)
    inner_profile = rounded_rect_profile(0.022, 0.035, 0.003, corner_segments=6)
    sleeve_mesh = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        0.320,
        cap=True,
        center=True,
    )
    sleeve_mesh.rotate_y(pi / 2).translate(0.178, 0.0, 0.0)

    forearm_sleeve = model.part("forearm_sleeve")
    forearm_sleeve.visual(
        Cylinder(radius=0.020, length=0.068),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="elbow_hub",
    )
    forearm_sleeve.visual(
        mesh_from_geometry(sleeve_mesh, "forearm_sleeve_shell"),
        material=satin_black,
        name="sleeve_shell",
    )
    forearm_sleeve.visual(
        Box((0.055, 0.070, 0.010)),
        origin=Origin(xyz=(0.326, 0.0, 0.033)),
        material=satin_black,
        name="sleeve_lip",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.350, 0.024, 0.014)),
        origin=Origin(xyz=(0.128, 0.0, 0.0)),
        material=dark_metal,
        name="slider_tube",
    )
    forearm.visual(
        Box((0.030, 0.070, 0.014)),
        origin=Origin(xyz=(0.288, 0.0, 0.0)),
        material=dark_metal,
        name="head_yoke_bridge",
    )
    forearm.visual(
        Box((0.040, 0.010, 0.052)),
        origin=Origin(xyz=(0.318, 0.038, 0.0)),
        material=dark_metal,
        name="head_cheek_0",
    )
    forearm.visual(
        Box((0.040, 0.010, 0.052)),
        origin=Origin(xyz=(0.318, -0.038, 0.0)),
        material=dark_metal,
        name="head_cheek_1",
    )

    head_shell_mesh = ExtrudeGeometry(
        rounded_rect_profile(0.140, 0.080, 0.018, corner_segments=10),
        0.035,
        cap=True,
        center=True,
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.016, length=0.066),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_barrel",
    )
    head.visual(
        Box((0.050, 0.028, 0.016)),
        origin=Origin(xyz=(0.025, 0.0, -0.012)),
        material=dark_metal,
        name="neck_link",
    )
    head.visual(
        Box((0.026, 0.026, 0.024)),
        origin=Origin(xyz=(0.047, 0.0, -0.028)),
        material=dark_metal,
        name="neck_drop",
    )
    head.visual(
        mesh_from_geometry(head_shell_mesh, "led_head_shell"),
        origin=Origin(xyz=(0.105, 0.0, -0.045)),
        material=satin_black,
        name="head_shell",
    )
    head.visual(
        Box((0.105, 0.052, 0.004)),
        origin=Origin(xyz=(0.105, 0.0, -0.0635)),
        material=led_diffuser,
        name="led_panel",
    )
    for index, y in enumerate((-0.026, 0.0, 0.026)):
        head.visual(
            Box((0.090, 0.004, 0.005)),
            origin=Origin(xyz=(0.105, y, -0.025)),
            material=matte_black,
            name=f"cooling_rib_{index}",
        )

    model.articulation(
        "clamp_to_post",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=post,
        origin=Origin(xyz=(0.020, 0.0, 0.2675)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.2, lower=-pi, upper=pi),
    )
    model.articulation(
        "post_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=post,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.75, upper=1.20),
    )
    model.articulation(
        "upper_arm_to_forearm_sleeve",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm_sleeve,
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "forearm_sleeve_to_forearm",
        ArticulationType.PRISMATIC,
        parent=forearm_sleeve,
        child=forearm,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.20, lower=0.0, upper=0.160),
    )
    model.articulation(
        "forearm_to_head",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=head,
        origin=Origin(xyz=(0.325, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    clamp_base = object_model.get_part("clamp_base")
    post = object_model.get_part("post")
    forearm_sleeve = object_model.get_part("forearm_sleeve")
    forearm = object_model.get_part("forearm")

    swivel = object_model.get_articulation("clamp_to_post")
    shoulder = object_model.get_articulation("post_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm_sleeve")
    telescope = object_model.get_articulation("forearm_sleeve_to_forearm")
    head_tilt = object_model.get_articulation("forearm_to_head")

    ctx.allow_overlap(
        forearm,
        forearm_sleeve,
        elem_a="slider_tube",
        elem_b="sleeve_shell",
        reason=(
            "The telescoping inner forearm is intentionally shown captured inside "
            "the outer sleeve proxy; insertion and centering are checked below."
        ),
    )

    ctx.check("base has vertical swivel", swivel.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("shoulder is revolute", shoulder.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("elbow is revolute", elbow.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("forearm telescopes", telescope.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("head tilts", head_tilt.articulation_type == ArticulationType.REVOLUTE)

    ctx.expect_contact(
        post,
        clamp_base,
        elem_a="post_shaft",
        elem_b="swivel_boss",
        contact_tol=0.001,
        name="post sits on clamp swivel boss",
    )
    ctx.expect_within(
        forearm,
        forearm_sleeve,
        axes="yz",
        inner_elem="slider_tube",
        outer_elem="sleeve_shell",
        margin=0.001,
        name="slider stays centered in sleeve",
    )
    ctx.expect_overlap(
        forearm,
        forearm_sleeve,
        axes="x",
        elem_a="slider_tube",
        elem_b="sleeve_shell",
        min_overlap=0.16,
        name="collapsed forearm remains inserted",
    )

    rest_forearm_pos = ctx.part_world_position(forearm)
    with ctx.pose({telescope: 0.160}):
        ctx.expect_within(
            forearm,
            forearm_sleeve,
            axes="yz",
            inner_elem="slider_tube",
            outer_elem="sleeve_shell",
            margin=0.001,
            name="extended slider stays centered",
        )
        ctx.expect_overlap(
            forearm,
            forearm_sleeve,
            axes="x",
            elem_a="slider_tube",
            elem_b="sleeve_shell",
            min_overlap=0.10,
            name="extended forearm retains insertion",
        )
        extended_forearm_pos = ctx.part_world_position(forearm)

    ctx.check(
        "forearm extends along lamp reach",
        rest_forearm_pos is not None
        and extended_forearm_pos is not None
        and extended_forearm_pos[0] > rest_forearm_pos[0] + 0.12,
        details=f"rest={rest_forearm_pos}, extended={extended_forearm_pos}",
    )

    return ctx.report()


object_model = build_object_model()
