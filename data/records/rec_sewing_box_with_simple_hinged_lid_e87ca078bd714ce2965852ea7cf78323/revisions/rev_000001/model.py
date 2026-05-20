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
    model = ArticulatedObject(name="sewing_box")

    wood = model.material("warm_wood", rgba=(0.58, 0.42, 0.25, 1.0))
    brass = model.material("aged_brass", rgba=(0.71, 0.62, 0.34, 1.0))

    outer_w = 0.24
    outer_d = 0.14
    body_h = 0.15
    wall_t = 0.01
    bottom_t = 0.01

    rear_frame_d = 0.036
    rear_frame_h = 0.044
    rear_return_w = 0.026
    rear_return_d = 0.06
    rear_return_h = 0.034
    hinge_mount_d = 0.014
    hinge_mount_h = 0.016

    lid_t = 0.008
    lid_w = outer_w + 0.004
    lid_d = outer_d + 0.004
    lid_rear_clear = 0.007
    hinge_leaf_w = 0.096
    hinge_leaf_d = 0.018
    hinge_leaf_t = 0.006

    hinge_r = 0.005
    hinge_side_margin = 0.02
    body_knuckle_len = 0.056
    hinge_gap = 0.004
    lid_knuckle_len = (
        outer_w
        - 2.0 * hinge_side_margin
        - 2.0 * body_knuckle_len
        - 2.0 * hinge_gap
    )
    left_knuckle_x = -outer_w / 2.0 + hinge_side_margin + body_knuckle_len / 2.0
    right_knuckle_x = outer_w / 2.0 - hinge_side_margin - body_knuckle_len / 2.0

    hinge_y = outer_d / 2.0 + 0.001
    hinge_z = body_h + 0.002

    body = model.part("body")
    body.visual(
        Box((outer_w, outer_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=wood,
        name="bottom_panel",
    )
    body.visual(
        Box((wall_t, outer_d, body_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + wall_t / 2.0, 0.0, body_h / 2.0)),
        material=wood,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_d, body_h)),
        origin=Origin(xyz=(outer_w / 2.0 - wall_t / 2.0, 0.0, body_h / 2.0)),
        material=wood,
        name="right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -outer_d / 2.0 + wall_t / 2.0, body_h / 2.0)),
        material=wood,
        name="front_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, outer_d / 2.0 - wall_t / 2.0, body_h / 2.0)),
        material=wood,
        name="rear_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, rear_frame_d, rear_frame_h)),
        origin=Origin(
            xyz=(
                0.0,
                outer_d / 2.0 - rear_frame_d / 2.0 - 0.006,
                body_h - rear_frame_h / 2.0,
            )
        ),
        material=wood,
        name="rear_frame",
    )
    body.visual(
        Box((rear_return_w, rear_return_d, rear_return_h)),
        origin=Origin(
            xyz=(
                -outer_w / 2.0 + wall_t + rear_return_w / 2.0,
                outer_d / 2.0 - rear_return_d / 2.0 - 0.006,
                body_h - rear_return_h / 2.0,
            )
        ),
        material=wood,
        name="left_rear_return",
    )
    body.visual(
        Box((rear_return_w, rear_return_d, rear_return_h)),
        origin=Origin(
            xyz=(
                outer_w / 2.0 - wall_t - rear_return_w / 2.0,
                outer_d / 2.0 - rear_return_d / 2.0 - 0.006,
                body_h - rear_return_h / 2.0,
            )
        ),
        material=wood,
        name="right_rear_return",
    )
    hinge_pad_w = body_knuckle_len + 0.016
    body.visual(
        Box((hinge_pad_w, hinge_mount_d, hinge_mount_h)),
        origin=Origin(
            xyz=(
                left_knuckle_x,
                hinge_y - hinge_mount_d / 2.0,
                hinge_z - hinge_mount_h / 2.0,
            )
        ),
        material=wood,
        name="left_hinge_pad",
    )
    body.visual(
        Box((hinge_pad_w, hinge_mount_d, hinge_mount_h)),
        origin=Origin(
            xyz=(
                right_knuckle_x,
                hinge_y - hinge_mount_d / 2.0,
                hinge_z - hinge_mount_h / 2.0,
            )
        ),
        material=wood,
        name="right_hinge_pad",
    )
    body.visual(
        Cylinder(radius=hinge_r, length=body_knuckle_len),
        origin=Origin(
            xyz=(left_knuckle_x, hinge_y, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="left_hinge_knuckle",
    )
    body.visual(
        Cylinder(radius=hinge_r, length=body_knuckle_len),
        origin=Origin(
            xyz=(right_knuckle_x, hinge_y, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="right_hinge_knuckle",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(
            xyz=(
                0.0,
                -(lid_d / 2.0 + lid_rear_clear),
                lid_t / 2.0 - 0.002,
            )
        ),
        material=wood,
        name="lid_panel",
    )
    lid.visual(
        Box((hinge_leaf_w, hinge_leaf_d, hinge_leaf_t)),
        origin=Origin(
            xyz=(
                0.0,
                -hinge_leaf_d / 2.0,
                hinge_leaf_t / 2.0 - 0.002,
            )
        ),
        material=wood,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=lid_knuckle_len),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="center_hinge_knuckle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.3,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="rear_frame",
        max_gap=0.001,
        max_penetration=0.0,
        name="lid panel seats on the rear frame",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        min_overlap=0.12,
        name="lid panel covers the body opening",
    )

    closed_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")

    closed_center = None
    open_center = None
    if closed_panel is not None:
        closed_center = tuple(
            (closed_panel[0][i] + closed_panel[1][i]) / 2.0 for i in range(3)
        )
    if open_panel is not None:
        open_center = tuple(
            (open_panel[0][i] + open_panel[1][i]) / 2.0 for i in range(3)
        )

    ctx.check(
        "lid swings upward when opened",
        closed_center is not None
        and open_center is not None
        and open_center[2] > closed_center[2] + 0.045
        and open_center[1] > closed_center[1] + 0.02,
        details=f"closed_center={closed_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
