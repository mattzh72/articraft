from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="factory_wall_lamp", assets=ASSETS)

    bracket_steel = model.material("bracket_steel", rgba=(0.33, 0.35, 0.38, 1.0))
    arm_gray = model.material("arm_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.70, 0.71, 0.73, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))

    plate_thickness = 0.012
    plate_width = 0.14
    plate_height = 0.24
    plate_center = (plate_thickness / 2.0, 0.0, 0.18)

    pivot_x = 0.044
    pivot_z = 0.18

    arm_section = 0.034
    inner_length = 0.276
    elbow_x = 0.30
    outer_length = 0.22
    outer_end_x = 0.24

    backplate = model.part("backplate")
    backplate.visual(
        Box((plate_thickness, plate_width, plate_height)),
        origin=Origin(xyz=plate_center),
        material=bracket_steel,
        name="plate",
    )
    flange_size = (0.020, 0.024, 0.024)
    for idx, (fy, fz) in enumerate(
        (
            (-0.050, 0.090),
            (0.050, 0.090),
            (-0.050, 0.270),
            (0.050, 0.270),
        ),
        start=1,
    ):
        backplate.visual(
            Box(flange_size),
            origin=Origin(xyz=(0.016, fy, fz)),
            material=bracket_steel,
            name=f"flange_{idx}",
        )
        backplate.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.026, fy, fz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware_steel,
            name=f"bolt_{idx}",
        )

    backplate.visual(
        Box((0.032, 0.060, 0.090)),
        origin=Origin(xyz=(0.028, 0.0, pivot_z)),
        material=hardware_steel,
        name="pivot_mount",
    )
    backplate.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        material=hardware_steel,
        name="pivot_post",
    )
    backplate.inertial = Inertial.from_geometry(
        Box((0.060, plate_width, plate_height)),
        mass=2.6,
        origin=Origin(xyz=(0.024, 0.0, pivot_z)),
    )

    inner_boom = model.part("inner_boom")
    inner_boom.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(),
        material=hardware_steel,
        name="pivot_hub",
    )
    inner_boom.visual(
        Box((inner_length, arm_section, arm_section)),
        origin=Origin(xyz=(0.162, 0.0, 0.0)),
        material=arm_gray,
        name="inner_tube",
    )
    inner_boom.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(elbow_x, 0.0, 0.0)),
        material=hardware_steel,
        name="elbow_mount",
    )
    inner_boom.inertial = Inertial.from_geometry(
        Box((0.33, 0.05, 0.05)),
        mass=1.0,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
    )

    outer_boom = model.part("outer_boom")
    outer_boom.visual(
        Cylinder(radius=0.022, length=0.048),
        origin=Origin(),
        material=hardware_steel,
        name="outer_hub",
    )
    outer_boom.visual(
        Box((outer_length, arm_section, arm_section)),
        origin=Origin(xyz=(0.132, 0.0, 0.0)),
        material=arm_gray,
        name="outer_tube",
    )
    outer_boom.visual(
        Box((0.050, 0.008, 0.070)),
        origin=Origin(xyz=(outer_end_x, 0.021, -0.035)),
        material=arm_gray,
        name="socket_left",
    )
    outer_boom.visual(
        Box((0.050, 0.008, 0.070)),
        origin=Origin(xyz=(outer_end_x, -0.021, -0.035)),
        material=arm_gray,
        name="socket_right",
    )
    outer_boom.visual(
        Box((0.008, 0.034, 0.070)),
        origin=Origin(xyz=(0.219, 0.0, -0.035)),
        material=arm_gray,
        name="socket_inner",
    )
    outer_boom.visual(
        Box((0.008, 0.034, 0.070)),
        origin=Origin(xyz=(0.261, 0.0, -0.035)),
        material=arm_gray,
        name="socket_outer",
    )
    outer_boom.inertial = Inertial.from_geometry(
        Box((0.28, 0.06, 0.09)),
        mass=0.8,
        origin=Origin(xyz=(0.14, 0.0, -0.01)),
    )

    drop_rod = model.part("drop_rod")
    drop_rod.visual(
        Box((0.022, 0.022, 0.140)),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=arm_gray,
        name="drop_tube",
    )
    drop_rod.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.077)),
        material=hardware_steel,
        name="locking_ring",
    )
    drop_rod.visual(
        Cylinder(radius=0.018, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.140), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="tilt_barrel",
    )
    drop_rod.inertial = Inertial.from_geometry(
        Box((0.05, 0.05, 0.16)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    shade = model.part("shade")
    shade.visual(
        Box((0.018, 0.008, 0.024)),
        origin=Origin(xyz=(0.009, 0.021, 0.0)),
        material=hardware_steel,
        name="shade_hinge",
    )
    shade.visual(
        Cylinder(radius=0.005, length=0.046),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="shade_pivot_pin",
    )
    shade.visual(
        Box((0.018, 0.008, 0.024)),
        origin=Origin(xyz=(0.009, -0.021, 0.0)),
        material=hardware_steel,
        name="shade_hinge_mate",
    )
    shade.visual(
        Box((0.032, 0.008, 0.014)),
        origin=Origin(xyz=(0.025, 0.021, -0.016)),
        material=hardware_steel,
        name="yoke_link_left",
    )
    shade.visual(
        Box((0.032, 0.008, 0.014)),
        origin=Origin(xyz=(0.025, -0.021, -0.016)),
        material=hardware_steel,
        name="yoke_link_right",
    )
    shade.visual(
        Cylinder(radius=0.056, length=0.124),
        origin=Origin(
            xyz=(0.098, 0.0, -0.050),
            rpy=(0.0, math.pi / 2.0 + 0.25, 0.0),
        ),
        material=matte_black,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.16, 0.13, 0.13)),
        mass=0.9,
        origin=Origin(xyz=(0.045, 0.0, -0.030)),
    )

    model.articulation(
        "base_swing",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=inner_boom,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=inner_boom,
        child=outer_boom,
        origin=Origin(xyz=(elbow_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-1.70,
            upper=0.15,
        ),
    )
    model.articulation(
        "drop_height",
        ArticulationType.PRISMATIC,
        parent=outer_boom,
        child=drop_rod,
        origin=Origin(xyz=(outer_end_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.20,
            lower=-0.045,
            upper=0.0,
        ),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=drop_rod,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, -0.140)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.35,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    backplate = object_model.get_part("backplate")
    inner_boom = object_model.get_part("inner_boom")
    outer_boom = object_model.get_part("outer_boom")
    drop_rod = object_model.get_part("drop_rod")
    shade = object_model.get_part("shade")

    base_swing = object_model.get_articulation("base_swing")
    elbow_fold = object_model.get_articulation("elbow_fold")
    drop_height = object_model.get_articulation("drop_height")
    shade_tilt = object_model.get_articulation("shade_tilt")

    plate = backplate.get_visual("plate")
    flange_1 = backplate.get_visual("flange_1")
    flange_2 = backplate.get_visual("flange_2")
    flange_4 = backplate.get_visual("flange_4")
    pivot_post = backplate.get_visual("pivot_post")
    inner_hub = inner_boom.get_visual("pivot_hub")
    inner_tube = inner_boom.get_visual("inner_tube")
    elbow_mount = inner_boom.get_visual("elbow_mount")
    outer_hub = outer_boom.get_visual("outer_hub")
    outer_tube = outer_boom.get_visual("outer_tube")
    socket_inner = outer_boom.get_visual("socket_inner")
    drop_tube = drop_rod.get_visual("drop_tube")
    locking_ring = drop_rod.get_visual("locking_ring")
    tilt_barrel = drop_rod.get_visual("tilt_barrel")
    shade_hinge = shade.get_visual("shade_hinge")
    shade_shell = shade.get_visual("shade_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        inner_boom,
        backplate,
        reason="The boom's pivot sleeve nests around the wall bracket's vertical post.",
    )
    ctx.allow_overlap(
        outer_boom,
        inner_boom,
        reason="The mid-arm elbow uses a nested sleeve around the exposed pivot pin.",
    )
    ctx.allow_overlap(
        drop_rod,
        outer_boom,
        reason="The height-adjustable drop rod telescopes inside the square outer-arm sleeve.",
    )
    ctx.allow_overlap(
        shade,
        drop_rod,
        reason="The shade tilt hinge nests at the bottom barrel of the drop rod.",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(inner_boom, backplate, elem_a=inner_hub, elem_b=pivot_post)
    ctx.expect_overlap(outer_boom, inner_boom, axes="xy", min_overlap=0.001)
    ctx.expect_contact(drop_rod, outer_boom, elem_a=locking_ring, elem_b=socket_inner)
    ctx.expect_contact(shade, drop_rod, elem_a=shade_hinge, elem_b=tilt_barrel)
    ctx.expect_gap(
        backplate,
        backplate,
        axis="x",
        min_gap=0.013,
        positive_elem=pivot_post,
        negative_elem=plate,
        name="pivot_post_projects_forward_of_plate",
    )
    ctx.expect_gap(
        backplate,
        backplate,
        axis="z",
        min_gap=0.150,
        positive_elem=flange_4,
        negative_elem=flange_1,
        name="bolt_flanges_span_plate_height",
    )
    ctx.expect_gap(
        backplate,
        backplate,
        axis="y",
        min_gap=0.075,
        positive_elem=flange_2,
        negative_elem=flange_1,
        name="bolt_flanges_span_plate_width",
    )
    ctx.expect_gap(
        inner_boom,
        backplate,
        axis="x",
        min_gap=0.010,
        positive_elem=inner_tube,
        negative_elem=plate,
    )
    ctx.expect_within(drop_rod, outer_boom, axes="xy", inner_elem=drop_tube)
    ctx.expect_gap(
        outer_boom,
        shade,
        axis="z",
        min_gap=0.050,
        positive_elem=outer_tube,
        negative_elem=shade_shell,
    )
    ctx.expect_gap(
        shade,
        shade,
        axis="x",
        min_gap=0.005,
        positive_elem=shade_shell,
        negative_elem=shade_hinge,
        name="shade_shell_projects_ahead_of_tilt_hinge",
    )
    ctx.expect_origin_distance(shade, drop_rod, axes="xy", max_dist=0.030)

    with ctx.pose({base_swing: math.pi / 2.0}):
        ctx.expect_contact(inner_boom, backplate, elem_a=inner_hub, elem_b=pivot_post)
        ctx.expect_gap(
            inner_boom,
            backplate,
            axis="x",
            min_gap=0.010,
            positive_elem=inner_tube,
            negative_elem=plate,
        )

    with ctx.pose({elbow_fold: -1.50}):
        ctx.expect_overlap(outer_boom, inner_boom, axes="xy", min_overlap=0.001)
        ctx.expect_within(drop_rod, outer_boom, axes="xy", inner_elem=drop_tube)

    with ctx.pose({drop_height: -0.040, shade_tilt: 0.45}):
        ctx.expect_contact(shade, drop_rod, elem_a=shade_hinge, elem_b=tilt_barrel)
        ctx.expect_gap(
            outer_boom,
            shade,
            axis="z",
            min_gap=0.090,
            positive_elem=outer_tube,
            negative_elem=shade_shell,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
