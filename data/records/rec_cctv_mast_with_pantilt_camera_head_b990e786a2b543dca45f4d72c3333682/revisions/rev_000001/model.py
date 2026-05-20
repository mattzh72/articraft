from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GALVANIZED = Material("galvanized_steel", rgba=(0.62, 0.66, 0.66, 1.0))
DARK_METAL = Material("dark_powder_coat", rgba=(0.04, 0.045, 0.05, 1.0))
CAMERA_PAINT = Material("camera_light_grey", rgba=(0.82, 0.84, 0.80, 1.0))
LENS_GLASS = Material("smoked_lens", rgba=(0.02, 0.04, 0.07, 1.0))


def _tapered_pole(height: float, bottom_radius: float, top_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(bottom_radius)
        .workplane(offset=height)
        .circle(top_radius)
        .loft(combine=True)
    )


def _rounded_camera_body() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.28, 0.10, 0.09)
        .edges()
        .fillet(0.008)
        .translate((0.14, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="street_light_cctv_mount")

    pole = model.part("pole")
    pole.visual(
        Cylinder(radius=0.24, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=GALVANIZED,
        name="base_flange",
    )
    pole.visual(
        mesh_from_cadquery(_tapered_pole(3.94, 0.135, 0.085), "tapered_pole"),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=GALVANIZED,
        name="tapered_pole",
    )
    pole.visual(
        Cylinder(radius=0.112, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 3.975)),
        material=GALVANIZED,
        name="top_band",
    )
    pole.visual(
        Box((0.12, 0.10, 0.030)),
        origin=Origin(xyz=(0.145, 0.0, 4.125)),
        material=GALVANIZED,
        name="upper_clevis_plate",
    )
    pole.visual(
        Box((0.12, 0.10, 0.030)),
        origin=Origin(xyz=(0.145, 0.0, 3.935)),
        material=GALVANIZED,
        name="lower_clevis_plate",
    )
    for index, (x, y) in enumerate(
        ((0.175, 0.175), (-0.175, 0.175), (-0.175, -0.175), (0.175, -0.175))
    ):
        pole.visual(
            Cylinder(radius=0.018, length=0.045),
            origin=Origin(xyz=(x, y, 0.0775)),
            material=DARK_METAL,
            name=f"anchor_bolt_{index}",
        )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.046, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=GALVANIZED,
        name="swivel_barrel",
    )
    arm.visual(
        Cylinder(radius=0.034, length=0.82),
        origin=Origin(xyz=(0.41, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=GALVANIZED,
        name="arm_tube",
    )
    arm.visual(
        Box((0.46, 0.026, 0.028)),
        origin=Origin(xyz=(0.30, 0.0, -0.047), rpy=(0.0, -0.19, 0.0)),
        material=GALVANIZED,
        name="under_gusset",
    )
    arm.visual(
        Cylinder(radius=0.026, length=0.19),
        origin=Origin(xyz=(0.75, 0.0, -0.095)),
        material=GALVANIZED,
        name="drop_post",
    )
    arm.visual(
        Cylinder(radius=0.064, length=0.032),
        origin=Origin(xyz=(0.75, 0.0, -0.174)),
        material=GALVANIZED,
        name="pan_socket",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.052, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=DARK_METAL,
        name="pan_cap",
    )
    pan_head.visual(
        Cylinder(radius=0.026, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=DARK_METAL,
        name="pan_stem",
    )
    pan_head.visual(
        Box((0.125, 0.045, 0.026)),
        origin=Origin(xyz=(0.060, 0.0, -0.128)),
        material=DARK_METAL,
        name="yoke_neck",
    )
    pan_head.visual(
        Box((0.080, 0.200, 0.026)),
        origin=Origin(xyz=(0.100, 0.0, -0.142)),
        material=DARK_METAL,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.080, 0.020, 0.165)),
        origin=Origin(xyz=(0.100, 0.085, -0.225)),
        material=DARK_METAL,
        name="yoke_plate_0",
    )
    pan_head.visual(
        Box((0.080, 0.020, 0.165)),
        origin=Origin(xyz=(0.100, -0.085, -0.225)),
        material=DARK_METAL,
        name="yoke_plate_1",
    )
    pan_head.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.100, 0.101, -0.220), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=GALVANIZED,
        name="tilt_cap_0",
    )
    pan_head.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.100, -0.101, -0.220), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=GALVANIZED,
        name="tilt_cap_1",
    )

    camera = model.part("camera")
    camera.visual(
        mesh_from_cadquery(_rounded_camera_body(), "camera_housing"),
        material=CAMERA_PAINT,
        name="housing",
    )
    camera.visual(
        Box((0.330, 0.125, 0.014)),
        origin=Origin(xyz=(0.155, 0.0, 0.051)),
        material=CAMERA_PAINT,
        name="sunshade",
    )
    camera.visual(
        Box((0.020, 0.125, 0.040)),
        origin=Origin(xyz=(0.315, 0.0, 0.035)),
        material=CAMERA_PAINT,
        name="front_lip",
    )
    camera.visual(
        Cylinder(radius=0.037, length=0.028),
        origin=Origin(xyz=(0.292, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=DARK_METAL,
        name="lens_bezel",
    )
    camera.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.309, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=LENS_GLASS,
        name="lens_glass",
    )
    camera.visual(
        Cylinder(radius=0.018, length=0.025),
        origin=Origin(xyz=(0.0, 0.0625, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=CAMERA_PAINT,
        name="trunnion_0",
    )
    camera.visual(
        Cylinder(radius=0.018, length=0.025),
        origin=Origin(xyz=(0.0, -0.0625, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=CAMERA_PAINT,
        name="trunnion_1",
    )

    model.articulation(
        "pole_to_arm",
        ArticulationType.REVOLUTE,
        parent=pole,
        child=arm,
        origin=Origin(xyz=(0.180, 0.0, 4.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.7, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "arm_to_pan",
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=pan_head,
        origin=Origin(xyz=(0.750, 0.0, -0.190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.100, 0.0, -0.220)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.65, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pole = object_model.get_part("pole")
    arm = object_model.get_part("arm")
    pan_head = object_model.get_part("pan_head")
    camera = object_model.get_part("camera")
    arm_joint = object_model.get_articulation("pole_to_arm")
    pan_joint = object_model.get_articulation("arm_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_camera")

    ctx.check(
        "primary mechanisms are articulated",
        arm_joint.articulation_type == ArticulationType.REVOLUTE
        and pan_joint.articulation_type == ArticulationType.CONTINUOUS
        and tilt_joint.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"types={arm_joint.articulation_type}, "
            f"{pan_joint.articulation_type}, {tilt_joint.articulation_type}"
        ),
    )
    ctx.check(
        "arm and tilt have realistic stops",
        arm_joint.motion_limits is not None
        and arm_joint.motion_limits.lower < -1.0
        and arm_joint.motion_limits.upper > 1.0
        and tilt_joint.motion_limits is not None
        and tilt_joint.motion_limits.lower < -0.4
        and tilt_joint.motion_limits.upper > 0.4,
        details=f"arm={arm_joint.motion_limits}, tilt={tilt_joint.motion_limits}",
    )
    ctx.expect_overlap(
        arm,
        pole,
        axes="z",
        elem_a="swivel_barrel",
        elem_b="top_band",
        min_overlap=0.095,
        name="arm swivel barrel sits inside the top-band height",
    )
    ctx.expect_gap(
        arm,
        pole,
        axis="x",
        positive_elem="swivel_barrel",
        negative_elem="top_band",
        min_gap=0.004,
        name="swivel barrel clears the pole band radially",
    )
    ctx.expect_gap(
        arm,
        pan_head,
        axis="z",
        positive_elem="pan_socket",
        negative_elem="pan_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan bearing cap seats under the arm socket",
    )
    ctx.expect_within(
        camera,
        pan_head,
        axes="y",
        margin=0.0,
        outer_elem="yoke_bridge",
        name="camera body fits within the fork width",
    )

    rest_pan_position = ctx.part_world_position(pan_head)
    with ctx.pose({arm_joint: 0.75}):
        swung_pan_position = ctx.part_world_position(pan_head)
    ctx.check(
        "arm revolute joint swings the outboard bracket",
        rest_pan_position is not None
        and swung_pan_position is not None
        and swung_pan_position[1] > rest_pan_position[1] + 0.25,
        details=f"rest={rest_pan_position}, swung={swung_pan_position}",
    )

    rest_lens = ctx.part_element_world_aabb(camera, elem="lens_glass")
    with ctx.pose({pan_joint: 0.85}):
        panned_lens = ctx.part_element_world_aabb(camera, elem="lens_glass")
    ctx.check(
        "continuous pan bearing yaws the camera",
        rest_lens is not None
        and panned_lens is not None
        and ((panned_lens[0][1] + panned_lens[1][1]) / 2.0)
        > ((rest_lens[0][1] + rest_lens[1][1]) / 2.0) + 0.15,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    with ctx.pose({tilt_joint: 0.55}):
        tilted_lens = ctx.part_element_world_aabb(camera, elem="lens_glass")
    ctx.check(
        "positive tilt angles the camera nose downward",
        rest_lens is not None
        and tilted_lens is not None
        and ((tilted_lens[0][2] + tilted_lens[1][2]) / 2.0)
        < ((rest_lens[0][2] + rest_lens[1][2]) / 2.0) - 0.08,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}",
    )

    return ctx.report()


object_model = build_object_model()
