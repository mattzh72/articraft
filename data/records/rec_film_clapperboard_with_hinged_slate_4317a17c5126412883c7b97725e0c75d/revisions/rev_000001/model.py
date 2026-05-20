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
    model = ArticulatedObject(name="rehearsal_clapperboard")

    board_w = 0.24
    board_h = 0.18
    board_t = 0.008

    hinge_z = board_h / 2.0
    hinge_y = 0.002
    hinge_r = 0.004
    board_barrel_len = 0.018

    clap_w = 0.198
    clap_h = 0.032
    clap_t = 0.012
    clap_body_y = 0.0086
    clap_barrel_len = 0.09

    matte_black = model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    stripe_white = model.material("stripe_white", rgba=(0.96, 0.96, 0.94, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    clip_gray = model.material("clip_gray", rgba=(0.26, 0.28, 0.31, 1.0))

    slate = model.part("slate")
    slate.visual(
        Box((board_w, board_t, board_h)),
        material=matte_black,
        name="slate_panel",
    )

    frame_t = 0.001
    frame_proud_y = board_t / 2.0 - frame_t / 3.0
    slate.visual(
        Box((board_w - 0.014, frame_t, 0.008)),
        origin=Origin(xyz=(0.0, frame_proud_y, hinge_z - 0.013)),
        material=stripe_white,
        name="frame_top",
    )
    slate.visual(
        Box((board_w - 0.020, frame_t, 0.008)),
        origin=Origin(xyz=(0.0, frame_proud_y, -hinge_z + 0.013)),
        material=stripe_white,
        name="frame_bottom",
    )
    for side, x in enumerate((-board_w / 2.0 + 0.010, board_w / 2.0 - 0.010)):
        slate.visual(
            Box((0.008, frame_t, board_h - 0.026)),
            origin=Origin(xyz=(x, frame_proud_y, 0.0)),
            material=stripe_white,
            name=f"frame_side_{side}",
        )

    for line, z in enumerate((-0.030, -0.010, 0.010)):
        slate.visual(
            Box((0.145, frame_t, 0.004)),
            origin=Origin(xyz=(-0.008, frame_proud_y, z - 0.040)),
            material=stripe_white,
            name=f"write_line_{line}",
        )

    for side, x in enumerate((-0.110, 0.110)):
        slate.visual(
            Cylinder(radius=hinge_r, length=board_barrel_len),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, radians(90.0), 0.0)),
            material=hinge_metal,
            name=f"hinge_barrel_{side}",
        )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((clap_w, clap_t, clap_h)),
        origin=Origin(xyz=(0.0, clap_body_y, -clap_h / 2.0)),
        material=matte_black,
        name="clapstick_body",
    )
    clapstick.visual(
        Cylinder(radius=hinge_r, length=clap_barrel_len),
        origin=Origin(rpy=(0.0, radians(90.0), 0.0)),
        material=hinge_metal,
        name="clap_barrel",
    )
    clapstick.visual(
        Box((clap_barrel_len, 0.008, 0.005)),
        origin=Origin(xyz=(0.0, 0.0042, -0.0015)),
        material=hinge_metal,
        name="clap_leaf",
    )

    stripe_angle = radians(33.0)
    stripe_x = (-0.078, -0.040, -0.002, 0.036, 0.074)
    for idx, x in enumerate(stripe_x):
        clapstick.visual(
            Box((0.050, 0.0012, 0.008)),
            origin=Origin(
                xyz=(x, clap_body_y + clap_t / 2.0 - 0.0003, -clap_h / 2.0),
                rpy=(0.0, stripe_angle, 0.0),
            ),
            material=stripe_white,
            name=f"stripe_{idx}",
        )

    marker_clip = model.part("marker_clip")
    marker_clip.visual(
        Box((0.006, 0.028, 0.060)),
        material=clip_gray,
        name="mount_pad",
    )
    marker_clip.visual(
        Box((0.006, 0.028, 0.060)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=clip_gray,
        name="outer_spine",
    )
    for idx, z in enumerate((-0.026, 0.026)):
        marker_clip.visual(
            Box((0.012, 0.028, 0.008)),
            origin=Origin(xyz=(0.009, 0.0, z)),
            material=clip_gray,
            name=f"bridge_{idx}",
        )

    model.articulation(
        "slate_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=slate,
        child=clapstick,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=6.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "slate_to_marker_clip",
        ArticulationType.FIXED,
        parent=slate,
        child=marker_clip,
        origin=Origin(xyz=(board_w / 2.0 + 0.003, 0.0, -0.020)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    slate = object_model.get_part("slate")
    clapstick = object_model.get_part("clapstick")
    marker_clip = object_model.get_part("marker_clip")
    hinge = object_model.get_articulation("slate_to_clapstick")
    limits = hinge.motion_limits

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            clapstick,
            slate,
            axis="y",
            positive_elem="clapstick_body",
            negative_elem="slate_panel",
            min_gap=0.0004,
            max_gap=0.0012,
            name="closed clapstick sits just proud of the slate",
        )
        ctx.expect_overlap(
            clapstick,
            slate,
            axes="x",
            elem_a="clapstick_body",
            elem_b="slate_panel",
            min_overlap=0.19,
            name="clapstick spans the slate width",
        )

    ctx.expect_contact(
        marker_clip,
        slate,
        elem_a="mount_pad",
        elem_b="slate_panel",
        name="marker clip mount pad contacts the slate edge",
    )

    if limits is not None and limits.upper is not None:
        with ctx.pose({hinge: 0.0}):
            closed_aabb = ctx.part_element_world_aabb(clapstick, elem="clapstick_body")
        with ctx.pose({hinge: limits.upper}):
            open_aabb = ctx.part_element_world_aabb(clapstick, elem="clapstick_body")
        ctx.check(
            "clapstick opens forward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][1] > closed_aabb[1][1] + 0.020,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
