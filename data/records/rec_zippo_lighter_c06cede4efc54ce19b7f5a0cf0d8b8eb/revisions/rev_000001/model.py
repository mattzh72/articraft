from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq
import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zippo_lighter")

    # Materials
    metal_color = (0.7, 0.7, 0.75, 1.0)
    insert_color = (0.8, 0.8, 0.8, 1.0)
    wheel_color = (0.3, 0.3, 0.3, 1.0)
    brass_color = (0.8, 0.7, 0.3, 1.0)

    # 1. Lower Shell
    lower_shell = model.part("lower_shell")
    lower_shell_cq = (
        cq.Workplane("XY")
        .box(0.038, 0.013, 0.040, centered=(True, True, False))
        .faces(">Z")
        .shell(-0.001)
    )
    lower_shell.visual(
        mesh_from_cadquery(lower_shell_cq, "lower_shell_mesh"),
        name="lower_shell_geom",
        color=metal_color,
    )

    # 2. Upper Lid
    upper_lid = model.part("upper_lid")
    upper_lid_cq = (
        cq.Workplane("XY")
        .box(0.038, 0.013, 0.015, centered=(True, True, False))
        .faces("<Z")
        .shell(-0.001)
    )
    upper_lid.visual(
        mesh_from_cadquery(upper_lid_cq, "upper_lid_mesh"),
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
        name="upper_lid_geom",
        color=metal_color,
    )

    # Hinge is at the left edge of the lower shell
    model.articulation(
        "hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=upper_lid,
        origin=Origin(xyz=(-0.019, 0.0, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=0.0, upper=2.5),
    )

    # 3. Insert
    insert = model.part("insert")
    # Insert body
    insert.visual(
        Box((0.035, 0.010, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        name="insert_body",
        color=insert_color,
    )
    # Chimney
    insert.visual(
        Box((0.016, 0.008, 0.012)),
        origin=Origin(xyz=(0.008, 0.0, 0.044)),
        name="chimney",
        color=insert_color,
    )
    # Wheel mounts
    insert.visual(
        Box((0.006, 0.001, 0.008)),
        origin=Origin(xyz=(-0.004, 0.004, 0.042)),
        name="wheel_mount_left",
        color=insert_color,
    )
    insert.visual(
        Box((0.006, 0.001, 0.008)),
        origin=Origin(xyz=(-0.004, -0.004, 0.042)),
        name="wheel_mount_right",
        color=insert_color,
    )
    # Cam mounts
    insert.visual(
        Box((0.004, 0.001, 0.006)),
        origin=Origin(xyz=(-0.012, 0.0025, 0.041)),
        name="cam_mount_left",
        color=insert_color,
    )
    insert.visual(
        Box((0.004, 0.001, 0.006)),
        origin=Origin(xyz=(-0.012, -0.0025, 0.041)),
        name="cam_mount_right",
        color=insert_color,
    )

    model.articulation(
        "insert_fixed",
        ArticulationType.FIXED,
        parent=lower_shell,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
    )

    # 4. Spark Wheel
    spark_wheel = model.part("spark_wheel")
    spark_wheel.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        name="wheel_cylinder",
        color=wheel_color,
    )
    spark_wheel.visual(
        Cylinder(radius=0.001, length=0.009),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        name="wheel_axle",
        color=metal_color,
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=spark_wheel,
        origin=Origin(xyz=(-0.004, 0.0, 0.043)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    # 5. Cam Lever
    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        Box((0.003, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="cam_lever_geom",
        color=brass_color,
    )
    model.articulation(
        "cam_pivot",
        ArticulationType.REVOLUTE,
        parent=insert,
        child=cam_lever,
        origin=Origin(xyz=(-0.012, 0.0, 0.041)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=5.0, lower=-0.5, upper=0.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Allowances
    ctx.allow_overlap(
        "lower_shell",
        "insert",
        reason="The insert is seated inside the lower shell cavity.",
    )
    ctx.allow_overlap(
        "upper_lid",
        "lower_shell",
        reason="The lid and shell touch when closed.",
    )
    ctx.allow_overlap(
        "upper_lid",
        "insert",
        reason="The upper lid closes over the chimney and wheel of the insert.",
    )
    ctx.allow_overlap(
        "upper_lid",
        "cam_lever",
        reason="The cam lever touches the inside of the upper lid.",
    )
    ctx.allow_overlap(
        "upper_lid",
        "spark_wheel",
        reason="The spark wheel is enclosed by the upper lid.",
    )
    ctx.allow_overlap(
        "insert",
        "spark_wheel",
        reason="The spark wheel axle is seated in the wheel mounts.",
    )
    ctx.allow_overlap(
        "insert",
        "cam_lever",
        reason="The cam lever is pinned between the cam mounts.",
    )

    # Tests
    lower_shell = object_model.get_part("lower_shell")
    upper_lid = object_model.get_part("upper_lid")
    insert = object_model.get_part("insert")

    ctx.expect_within(
        insert,
        lower_shell,
        axes="xy",
        margin=0.001,
        name="insert fits within lower shell horizontally",
    )

    hinge = object_model.get_articulation("hinge")
    with ctx.pose({hinge: 2.0}):
        lid_aabb = ctx.part_world_aabb(upper_lid)
        shell_aabb = ctx.part_world_aabb(lower_shell)
        ctx.check(
            "lid opens to the left",
            lid_aabb is not None and shell_aabb is not None and lid_aabb[1][0] <= shell_aabb[0][0] + 0.001,
            details="The upper lid should swing left of the lower shell when open.",
        )

    return ctx.report()


object_model = build_object_model()