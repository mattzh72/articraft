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
    model = ArticulatedObject(name="cantilever_arm")

    base_post = model.part("base_post")
    base_post.visual(Box((0.1, 0.1, 1.0)), origin=Origin(xyz=(0.0, 0.0, 0.5)), name="post_body")
    base_post.visual(Box((0.09, 0.08, 0.02)), origin=Origin(xyz=(0.085, 0.0, 0.85)), name="clevis_top")
    base_post.visual(Box((0.09, 0.08, 0.02)), origin=Origin(xyz=(0.085, 0.0, 0.75)), name="clevis_bottom")

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(Cylinder(radius=0.04, length=0.082), origin=Origin(xyz=(0.0, 0.0, 0.0)), name="hinge_cyl")
    shoulder_link.visual(Box((0.36, 0.06, 0.06)), origin=Origin(xyz=(0.18, 0.0, 0.0)), name="arm_body")
    shoulder_link.visual(Box((0.02, 0.08, 0.12)), origin=Origin(xyz=(0.35, 0.0, 0.0)), name="clevis_base")
    shoulder_link.visual(Box((0.09, 0.08, 0.02)), origin=Origin(xyz=(0.395, 0.0, 0.05)), name="clevis_top")
    shoulder_link.visual(Box((0.09, 0.08, 0.02)), origin=Origin(xyz=(0.395, 0.0, -0.05)), name="clevis_bottom")

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base_post,
        child=shoulder_link,
        origin=Origin(xyz=(0.1, 0.0, 0.8)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-1.57, upper=1.57),
    )

    forelink = model.part("forelink")
    forelink.visual(Cylinder(radius=0.04, length=0.082), origin=Origin(xyz=(0.0, 0.0, 0.0)), name="hinge_cyl")
    forelink.visual(Box((0.27, 0.06, 0.06)), origin=Origin(xyz=(0.135, 0.0, 0.0)), name="arm_body")
    forelink.visual(Box((0.02, 0.06, 0.10)), origin=Origin(xyz=(0.26, 0.0, 0.0)), name="clevis_base")
    forelink.visual(Box((0.07, 0.06, 0.02)), origin=Origin(xyz=(0.295, 0.0, 0.04)), name="clevis_top")
    forelink.visual(Box((0.07, 0.06, 0.02)), origin=Origin(xyz=(0.295, 0.0, -0.04)), name="clevis_bottom")

    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=forelink,
        origin=Origin(xyz=(0.4, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-2.0, upper=2.0),
    )

    wrist = model.part("wrist")
    wrist.visual(Cylinder(radius=0.03, length=0.062), origin=Origin(xyz=(0.0, 0.0, 0.0)), name="hinge_cyl")
    wrist.visual(Box((0.025, 0.04, 0.04)), origin=Origin(xyz=(0.0325, 0.0, 0.0)), name="stem")
    wrist.visual(Box((0.02, 0.1, 0.1)), origin=Origin(xyz=(0.05, 0.0, 0.0)), name="face_plate")

    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forelink,
        child=wrist,
        origin=Origin(xyz=(0.3, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-1.57, upper=1.57),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_post = object_model.get_part("base_post")
    shoulder_link = object_model.get_part("shoulder_link")
    forelink = object_model.get_part("forelink")
    wrist = object_model.get_part("wrist")

    ctx.allow_overlap(base_post, shoulder_link, reason="Hinge cylinder intentionally connects to clevis plates.")
    ctx.allow_overlap(shoulder_link, forelink, reason="Hinge cylinder intentionally connects to clevis plates.")
    ctx.allow_overlap(forelink, wrist, reason="Hinge cylinder intentionally connects to clevis plates.")

    # In rest pose, the arm extends along +X
    shoulder_pos = ctx.part_world_position(shoulder_link)
    forelink_pos = ctx.part_world_position(forelink)
    wrist_pos = ctx.part_world_position(wrist)

    ctx.check("shoulder_projects_side", shoulder_pos is not None and shoulder_pos[0] > 0.05)
    ctx.check("forelink_projects_side", forelink_pos is not None and forelink_pos[0] > 0.4)
    ctx.check("wrist_projects_side", wrist_pos is not None and wrist_pos[0] > 0.7)

    # Pose test: flex elbow
    with ctx.pose(elbow_joint=1.57):
        wrist_pos_flexed = ctx.part_world_position(wrist)
        ctx.check("elbow_flexes", wrist_pos_flexed is not None and wrist_pos_flexed[1] > 0.2)

    return ctx.report()

object_model = build_object_model()
