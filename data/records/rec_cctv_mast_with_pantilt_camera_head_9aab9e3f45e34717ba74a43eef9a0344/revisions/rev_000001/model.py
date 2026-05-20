from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freestanding_cctv_column")

    galvanized = Material("galvanized_steel", rgba=(0.55, 0.57, 0.56, 1.0))
    dark_steel = Material("dark_bolt_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    camera_white = Material("painted_camera_white", rgba=(0.88, 0.90, 0.88, 1.0))
    black_glass = Material("black_lens_glass", rgba=(0.01, 0.012, 0.014, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    column = model.part("column")
    column.visual(
        Box((0.52, 0.52, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=galvanized,
        name="base_plate",
    )
    column.visual(
        Cylinder(radius=0.055, length=2.55),
        origin=Origin(xyz=(0.0, 0.0, 1.32)),
        material=galvanized,
        name="round_pole",
    )
    column.visual(
        Cylinder(radius=0.080, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=galvanized,
        name="welded_collar",
    )

    # Four welded gusset plates tie the pole into the square flange.
    for suffix, x, y, size in (
        ("x", 0.13, 0.0, (0.28, 0.018, 0.24)),
        ("neg_x", -0.13, 0.0, (0.28, 0.018, 0.24)),
        ("y", 0.0, 0.13, (0.018, 0.28, 0.24)),
        ("neg_y", 0.0, -0.13, (0.018, 0.28, 0.24)),
    ):
        column.visual(
            Box(size),
            origin=Origin(xyz=(x, y, 0.165)),
            material=galvanized,
            name=f"base_gusset_{suffix}",
        )

    # Visible anchor bolts on the flanged base plate.
    for ix, x in enumerate((-0.19, 0.19)):
        for iy, y in enumerate((-0.19, 0.19)):
            column.visual(
                Cylinder(radius=0.026, length=0.018),
                origin=Origin(xyz=(x, y, 0.053)),
                material=dark_steel,
                name=f"anchor_bolt_{ix}_{iy}",
            )

    column.visual(
        Cylinder(radius=0.076, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 2.54)),
        material=galvanized,
        name="top_collar",
    )
    column.visual(
        Box((0.62, 0.080, 0.080)),
        origin=Origin(xyz=(0.34, 0.0, 2.545)),
        material=galvanized,
        name="side_arm",
    )
    column.visual(
        Box((0.42, 0.035, 0.035)),
        origin=Origin(xyz=(0.255, 0.0, 2.430), rpy=(0.0, -0.34, 0.0)),
        material=galvanized,
        name="diagonal_brace",
    )
    column.visual(
        Box((0.080, 0.100, 0.030)),
        origin=Origin(xyz=(0.620, 0.0, 2.490)),
        material=galvanized,
        name="pan_mount_pad",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=galvanized,
        name="swivel_cap",
    )
    pan_head.visual(
        Cylinder(radius=0.030, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, -0.0725)),
        material=galvanized,
        name="vertical_spindle",
    )
    pan_head.visual(
        Box((0.110, 0.168, 0.030)),
        origin=Origin(xyz=(0.010, 0.0, -0.125)),
        material=galvanized,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.110, 0.024, 0.120)),
        origin=Origin(xyz=(0.010, 0.070, -0.200)),
        material=galvanized,
        name="yoke_cheek_0",
    )
    pan_head.visual(
        Box((0.110, 0.024, 0.120)),
        origin=Origin(xyz=(0.010, -0.070, -0.200)),
        material=galvanized,
        name="yoke_cheek_1",
    )

    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.045, length=0.085),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="rear_trunnion_block",
    )
    camera.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.049, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="tilt_pin_0",
    )
    camera.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, -0.049, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="tilt_pin_1",
    )
    camera.visual(
        Cylinder(radius=0.045, length=0.300),
        origin=Origin(xyz=(0.190, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="bullet_body",
    )
    camera.visual(
        Cylinder(radius=0.047, length=0.024),
        origin=Origin(xyz=(0.336, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="front_bezel",
    )
    camera.visual(
        Cylinder(radius=0.034, length=0.006),
        origin=Origin(xyz=(0.350, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_glass,
        name="front_lens",
    )
    camera.visual(
        Box((0.340, 0.104, 0.018)),
        origin=Origin(xyz=(0.205, 0.0, 0.053)),
        material=camera_white,
        name="sunshade_top",
    )
    camera.visual(
        Box((0.340, 0.010, 0.050)),
        origin=Origin(xyz=(0.205, 0.052, 0.030)),
        material=camera_white,
        name="sunshade_side_0",
    )
    camera.visual(
        Box((0.340, 0.010, 0.050)),
        origin=Origin(xyz=(0.205, -0.052, 0.030)),
        material=camera_white,
        name="sunshade_side_1",
    )
    camera.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(-0.055, 0.0, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="rear_cable",
    )

    model.articulation(
        "arm_to_pan",
        ArticulationType.REVOLUTE,
        parent=column,
        child=pan_head,
        origin=Origin(xyz=(0.620, 0.0, 2.475)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=-2.70, upper=2.70),
    )
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, -0.200)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.35, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    pan_head = object_model.get_part("pan_head")
    camera = object_model.get_part("camera")
    pan_joint = object_model.get_articulation("arm_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_camera")

    def element_center(part, elem_name):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) / 2.0,
            (lo[1] + hi[1]) / 2.0,
            (lo[2] + hi[2]) / 2.0,
        )

    pan_limits = pan_joint.motion_limits
    tilt_limits = tilt_joint.motion_limits
    ctx.check(
        "pan joint is a vertical revolute scan axis",
        pan_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(pan_joint.axis) == (0.0, 0.0, 1.0)
        and pan_limits is not None
        and pan_limits.lower is not None
        and pan_limits.upper is not None
        and pan_limits.lower <= -2.5
        and pan_limits.upper >= 2.5,
        details=f"type={pan_joint.articulation_type}, axis={pan_joint.axis}, limits={pan_limits}",
    )
    ctx.check(
        "tilt joint is a horizontal revolute pitch axis",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(tilt_joint.axis) == (0.0, 1.0, 0.0)
        and tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
        and tilt_limits.lower < 0.0
        and tilt_limits.upper > 0.7,
        details=f"type={tilt_joint.articulation_type}, axis={tilt_joint.axis}, limits={tilt_limits}",
    )
    ctx.check(
        "flanged base has four anchor bolts",
        len([v for v in column.visuals if v.name and v.name.startswith("anchor_bolt_")]) == 4,
    )

    ctx.expect_contact(
        column,
        pan_head,
        elem_a="pan_mount_pad",
        elem_b="swivel_cap",
        contact_tol=0.003,
        name="pan swivel is seated under the side-arm pad",
    )
    ctx.expect_contact(
        pan_head,
        camera,
        elem_a="yoke_cheek_0",
        elem_b="tilt_pin_0",
        contact_tol=0.003,
        name="upper yoke cheek captures one tilt pin",
    )
    ctx.expect_contact(
        pan_head,
        camera,
        elem_a="yoke_cheek_1",
        elem_b="tilt_pin_1",
        contact_tol=0.003,
        name="lower yoke cheek captures the other tilt pin",
    )
    ctx.expect_within(
        camera,
        pan_head,
        axes="y",
        inner_elem="bullet_body",
        outer_elem="yoke_bridge",
        margin=0.0,
        name="bullet body fits between the yoke cheeks",
    )

    rest_lens = element_center(camera, "front_lens")
    with ctx.pose({pan_joint: 1.2}):
        panned_lens = element_center(camera, "front_lens")
    ctx.check(
        "positive pan swings the camera around the pole-side bracket",
        rest_lens is not None and panned_lens is not None and panned_lens[1] > rest_lens[1] + 0.20,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    rest_lens = element_center(camera, "front_lens")
    with ctx.pose({tilt_joint: 0.65}):
        tilted_lens = element_center(camera, "front_lens")
    ctx.check(
        "positive tilt aims the bullet camera downward",
        rest_lens is not None and tilted_lens is not None and tilted_lens[2] < rest_lens[2] - 0.12,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    return ctx.report()


object_model = build_object_model()
