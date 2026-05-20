from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="wheelie_bin")

    body_green = model.material("body_green", rgba=(0.14, 0.33, 0.19, 1.0))
    lid_green = model.material("lid_green", rgba=(0.16, 0.37, 0.21, 1.0))
    axle_gray = model.material("axle_gray", rgba=(0.22, 0.23, 0.24, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.05, 0.05, 0.05, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.44, 0.46, 0.48, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.60, 0.74, 1.02)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )

    wall_height = 0.92
    rim_z = 0.965

    body.visual(
        Box((0.45, 0.55, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=body_green,
        name="floor_pan",
    )
    body.visual(
        Box((0.50, 0.040, wall_height)),
        origin=Origin(xyz=(0.0, 0.313, 0.49), rpy=(-0.11, 0.0, 0.0)),
        material=body_green,
        name="front_wall",
    )
    body.visual(
        Box((0.54, 0.040, wall_height)),
        origin=Origin(xyz=(0.0, -0.318, 0.49)),
        material=body_green,
        name="rear_wall",
    )
    body.visual(
        Box((0.040, 0.63, wall_height)),
        origin=Origin(xyz=(0.267, 0.0, 0.49), rpy=(0.0, 0.10, 0.0)),
        material=body_green,
        name="left_wall",
    )
    body.visual(
        Box((0.040, 0.63, wall_height)),
        origin=Origin(xyz=(-0.267, 0.0, 0.49), rpy=(0.0, -0.10, 0.0)),
        material=body_green,
        name="right_wall",
    )
    body.visual(
        Box((0.54, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.336, rim_z)),
        material=body_green,
        name="front_rim",
    )
    body.visual(
        Box((0.56, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.333, rim_z)),
        material=body_green,
        name="rear_rim",
    )
    body.visual(
        Box((0.045, 0.66, 0.040)),
        origin=Origin(xyz=(0.283, 0.0, rim_z)),
        material=body_green,
        name="left_rim",
    )
    body.visual(
        Box((0.045, 0.66, 0.040)),
        origin=Origin(xyz=(-0.283, 0.0, rim_z)),
        material=body_green,
        name="right_rim",
    )
    body.visual(
        Box((0.49, 0.065, 0.085)),
        origin=Origin(xyz=(0.0, 0.282, 0.0425)),
        material=body_green,
        name="front_toe_lip",
    )
    body.visual(
        Box((0.040, 0.065, 0.23)),
        origin=Origin(xyz=(0.215, -0.315, 0.175)),
        material=axle_gray,
        name="left_axle_cheek",
    )
    body.visual(
        Box((0.040, 0.065, 0.23)),
        origin=Origin(xyz=(-0.215, -0.315, 0.175)),
        material=axle_gray,
        name="right_axle_cheek",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.43),
        origin=Origin(xyz=(0.0, -0.335, 0.14), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="axle_tube",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.060),
        origin=Origin(xyz=(0.270, -0.335, 0.14), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="left_stub_axle",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.060),
        origin=Origin(xyz=(-0.270, -0.335, 0.14), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="right_stub_axle",
    )
    body.visual(
        Box((0.050, 0.095, 0.22)),
        origin=Origin(xyz=(0.215, -0.302, 0.205)),
        material=body_green,
        name="left_wheel_shroud",
    )
    body.visual(
        Box((0.050, 0.095, 0.22)),
        origin=Origin(xyz=(-0.215, -0.302, 0.205)),
        material=body_green,
        name="right_wheel_shroud",
    )
    body.visual(
        Box((0.47, 0.040, 0.040)),
        origin=Origin(xyz=(0.0, -0.368, 0.875)),
        material=lid_green,
        name="rear_handle_bar",
    )
    body.visual(
        Box((0.040, 0.055, 0.12)),
        origin=Origin(xyz=(0.215, -0.350, 0.920)),
        material=body_green,
        name="left_handle_post",
    )
    body.visual(
        Box((0.040, 0.055, 0.12)),
        origin=Origin(xyz=(-0.215, -0.350, 0.920)),
        material=body_green,
        name="right_handle_post",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(-0.185, -0.365, 0.985), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="left_hinge_lug",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(0.185, -0.365, 0.985), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="right_hinge_lug",
    )
    body.visual(
        Box((0.065, 0.045, 0.075)),
        origin=Origin(xyz=(-0.185, -0.345, 0.955)),
        material=body_green,
        name="left_hinge_support",
    )
    body.visual(
        Box((0.065, 0.045, 0.075)),
        origin=Origin(xyz=(0.185, -0.345, 0.955)),
        material=body_green,
        name="right_hinge_support",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.67, 0.77, 0.08)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.39, 0.0)),
    )
    lid.visual(
        Box((0.67, 0.730, 0.020)),
        origin=Origin(xyz=(0.0, 0.385, 0.015)),
        material=lid_green,
        name="lid_top",
    )
    lid.visual(
        Box((0.59, 0.028, 0.034)),
        origin=Origin(xyz=(0.0, 0.764, -0.012)),
        material=lid_green,
        name="front_lip",
    )
    lid.visual(
        Box((0.018, 0.733, 0.070)),
        origin=Origin(xyz=(0.342, 0.384, -0.010)),
        material=lid_green,
        name="left_skirt",
    )
    lid.visual(
        Box((0.018, 0.733, 0.070)),
        origin=Origin(xyz=(-0.342, 0.384, -0.010)),
        material=lid_green,
        name="right_skirt",
    )
    lid.visual(
        Box((0.63, 0.032, 0.022)),
        origin=Origin(xyz=(0.0, 0.090, 0.014)),
        material=lid_green,
        name="rear_beam",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.060),
        mass=1.6,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    left_wheel.visual(
        Cylinder(radius=0.135, length=0.060),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_black,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.103, length=0.052),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_gray,
        name="rim",
    )
    left_wheel.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="outer_cap",
    )
    left_wheel.visual(
        Cylinder(radius=0.042, length=0.008),
        origin=Origin(xyz=(-0.022, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="inner_cap",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.060),
        mass=1.6,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    right_wheel.visual(
        Cylinder(radius=0.135, length=0.060),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_black,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.103, length=0.052),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_gray,
        name="rim",
    )
    right_wheel.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(-0.024, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="outer_cap",
    )
    right_wheel.visual(
        Cylinder(radius=0.042, length=0.008),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="inner_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.365, 0.985)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.65),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(0.330, -0.335, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(-0.330, -0.335, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_top",
        negative_elem="rear_rim",
        max_gap=0.010,
        max_penetration=0.0,
        name="lid sits just above rear rim when closed",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_top",
        negative_elem="front_rim",
        max_gap=0.010,
        max_penetration=0.0,
        name="lid spans over front rim when closed",
    )

    left_pos = ctx.part_world_position(left_wheel)
    right_pos = ctx.part_world_position(right_wheel)
    ctx.check(
        "rear wheels straddle the body symmetrically",
        left_pos is not None
        and right_pos is not None
        and left_pos[0] > 0.30
        and right_pos[0] < -0.30
        and abs(left_pos[1] - right_pos[1]) < 1e-6
        and abs(left_pos[2] - right_pos[2]) < 1e-6,
        details=f"left={left_pos}, right={right_pos}",
    )
    ctx.check(
        "rear wheels sit behind the bin body",
        left_pos is not None and right_pos is not None and left_pos[1] < -0.30 and right_pos[1] < -0.30,
        details=f"left={left_pos}, right={right_pos}",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({lid_hinge: 1.50}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_lip")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_lip",
            negative_elem="rear_handle_bar",
            min_gap=0.10,
            name="opened lid front edge rises clear of rear handle",
        )
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[1][2] + 0.18,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
