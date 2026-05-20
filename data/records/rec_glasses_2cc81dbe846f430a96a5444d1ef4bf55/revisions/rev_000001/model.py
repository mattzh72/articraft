from __future__ import annotations

from math import radians

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
    model = ArticulatedObject(name="rectangular_eyeglasses")

    frame_mat = model.material("satin_black_acetate", rgba=(0.015, 0.014, 0.013, 1.0))
    hinge_mat = model.material("brushed_hinge_steel", rgba=(0.62, 0.60, 0.55, 1.0))
    lens_mat = model.material("faint_blue_lens", rgba=(0.66, 0.82, 0.95, 0.34))
    tip_mat = model.material("soft_black_tips", rgba=(0.025, 0.024, 0.023, 1.0))
    pad_mat = model.material("clear_nose_pads", rgba=(0.86, 0.90, 0.88, 0.48))

    lens_w = 0.052
    lens_h = 0.038
    rim = 0.0045
    depth = 0.006
    center_x = 0.0345
    outer_w = lens_w + 2.0 * rim
    outer_h = lens_h + 2.0 * rim
    side_offset = lens_w * 0.5 + rim * 0.5
    top_z = lens_h * 0.5 + rim * 0.5
    hinge_x = center_x + side_offset + 0.0065
    hinge_y = 0.011

    front = model.part("front_frame")

    for sign, prefix in ((-1.0, "left"), (1.0, "right")):
        cx = sign * center_x
        # Rectangular lens rims: separate top/bottom/side rails overlap at the
        # corners so the visual reads as a continuous molded front frame.
        front.visual(
            Box((outer_w, depth, rim)),
            origin=Origin(xyz=(cx, 0.0, top_z)),
            material=frame_mat,
            name=f"{prefix}_rim_top",
        )
        front.visual(
            Box((outer_w, depth, rim)),
            origin=Origin(xyz=(cx, 0.0, -top_z)),
            material=frame_mat,
            name=f"{prefix}_rim_bottom",
        )
        front.visual(
            Box((rim, depth, outer_h)),
            origin=Origin(xyz=(cx - side_offset, 0.0, 0.0)),
            material=frame_mat,
            name=f"{prefix}_rim_inner" if sign < 0 else f"{prefix}_rim_outer",
        )
        front.visual(
            Box((rim, depth, outer_h)),
            origin=Origin(xyz=(cx + side_offset, 0.0, 0.0)),
            material=frame_mat,
            name=f"{prefix}_rim_outer" if sign < 0 else f"{prefix}_rim_inner",
        )
        front.visual(
            Box((lens_w, 0.0012, lens_h)),
            origin=Origin(xyz=(cx, -0.0002, 0.0)),
            material=lens_mat,
            name=f"{prefix}_lens",
        )

    # A short top bridge and lower nose bridge tie the two rectangular rims
    # together while leaving a realistic nose opening.
    front.visual(
        Box((0.017, depth, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=frame_mat,
        name="top_bridge",
    )
    front.visual(
        Cylinder(radius=0.0022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.004), rpy=(0.0, radians(90.0), 0.0)),
        material=frame_mat,
        name="lower_bridge",
    )
    front.visual(
        Box((0.004, 0.003, 0.014)),
        origin=Origin(xyz=(-0.008, 0.004, -0.006), rpy=(0.0, 0.0, radians(-14.0))),
        material=hinge_mat,
        name="left_pad_arm",
    )
    front.visual(
        Box((0.004, 0.003, 0.014)),
        origin=Origin(xyz=(0.008, 0.004, -0.006), rpy=(0.0, 0.0, radians(14.0))),
        material=hinge_mat,
        name="right_pad_arm",
    )
    front.visual(
        Box((0.006, 0.002, 0.012)),
        origin=Origin(xyz=(-0.012, 0.006, -0.013), rpy=(0.0, 0.0, radians(-12.0))),
        material=pad_mat,
        name="left_nose_pad",
    )
    front.visual(
        Box((0.006, 0.002, 0.012)),
        origin=Origin(xyz=(0.012, 0.006, -0.013), rpy=(0.0, 0.0, radians(12.0))),
        material=pad_mat,
        name="right_nose_pad",
    )

    for sign, prefix in ((-1.0, "left"), (1.0, "right")):
        x = sign * hinge_x
        front.visual(
            Box((0.012, 0.008, 0.030)),
            origin=Origin(xyz=(x, 0.002, 0.0)),
            material=frame_mat,
            name=f"{prefix}_hinge_plate",
        )
        front.visual(
            Box((0.010, 0.0045, 0.006)),
            origin=Origin(xyz=(x, 0.0078, 0.011)),
            material=hinge_mat,
            name=f"{prefix}_hinge_top_lug",
        )
        front.visual(
            Box((0.010, 0.0045, 0.006)),
            origin=Origin(xyz=(x, 0.0078, -0.011)),
            material=hinge_mat,
            name=f"{prefix}_hinge_bottom_lug",
        )
        front.visual(
            Cylinder(radius=0.0036, length=0.008),
            origin=Origin(xyz=(x, hinge_y, 0.011)),
            material=hinge_mat,
            name=f"{prefix}_hinge_top",
        )
        front.visual(
            Cylinder(radius=0.0036, length=0.008),
            origin=Origin(xyz=(x, hinge_y, -0.011)),
            material=hinge_mat,
            name=f"{prefix}_hinge_bottom",
        )

    left_temple = model.part("left_temple")
    left_temple.visual(
        Cylinder(radius=0.0036, length=0.014),
        material=hinge_mat,
        name="hinge_knuckle",
    )
    left_temple.visual(
        Box((0.126, 0.003, 0.004)),
        origin=Origin(xyz=(0.063, -0.0025, 0.0)),
        material=frame_mat,
        name="arm_bar",
    )
    left_temple.visual(
        Box((0.032, 0.004, 0.005)),
        origin=Origin(xyz=(0.137, -0.0025, -0.004), rpy=(0.0, radians(10.0), 0.0)),
        material=tip_mat,
        name="ear_tip",
    )

    right_temple = model.part("right_temple")
    right_temple.visual(
        Cylinder(radius=0.0036, length=0.014),
        material=hinge_mat,
        name="hinge_knuckle",
    )
    right_temple.visual(
        Box((0.126, 0.003, 0.004)),
        origin=Origin(xyz=(-0.063, 0.0035, 0.0)),
        material=frame_mat,
        name="arm_bar",
    )
    right_temple.visual(
        Box((0.032, 0.004, 0.005)),
        origin=Origin(xyz=(-0.137, 0.0035, -0.004), rpy=(0.0, radians(-10.0), 0.0)),
        material=tip_mat,
        name="ear_tip",
    )

    open_angle = radians(100.0)
    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=left_temple,
        origin=Origin(xyz=(-hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=4.0, lower=0.0, upper=open_angle),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=right_temple,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=4.0, lower=0.0, upper=open_angle),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    left = object_model.get_part("left_temple")
    right = object_model.get_part("right_temple")
    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")

    for hinge in (left_hinge, right_hinge):
        limits = hinge.motion_limits
        ctx.check(
            f"{hinge.name} has 0 to 100 degree travel",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and radians(95.0) <= limits.upper <= radians(105.0),
            details=f"limits={limits}",
        )

    ctx.expect_contact(
        left,
        front,
        elem_a="hinge_knuckle",
        elem_b="left_hinge_top",
        contact_tol=1e-5,
        name="left temple knuckle is captured by hinge barrel",
    )
    ctx.expect_contact(
        right,
        front,
        elem_a="hinge_knuckle",
        elem_b="right_hinge_top",
        contact_tol=1e-5,
        name="right temple knuckle is captured by hinge barrel",
    )

    left_folded = ctx.part_element_world_aabb(left, elem="ear_tip")
    right_folded = ctx.part_element_world_aabb(right, elem="ear_tip")
    with ctx.pose({left_hinge: radians(100.0), right_hinge: radians(100.0)}):
        left_open = ctx.part_element_world_aabb(left, elem="ear_tip")
        right_open = ctx.part_element_world_aabb(right, elem="ear_tip")

    ctx.check(
        "left temple opens rearward",
        left_folded is not None
        and left_open is not None
        and left_open[1][1] > left_folded[1][1] + 0.08,
        details=f"folded={left_folded}, open={left_open}",
    )
    ctx.check(
        "right temple opens rearward",
        right_folded is not None
        and right_open is not None
        and right_open[1][1] > right_folded[1][1] + 0.08,
        details=f"folded={right_folded}, open={right_open}",
    )

    return ctx.report()


object_model = build_object_model()
