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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _add_keeper(part, *, hardware) -> None:
    part.visual(
        Box((0.040, 0.004, 0.032)),
        origin=Origin(xyz=(0.0, 0.002, 0.016)),
        material=hardware,
        name="back_plate",
    )
    for sign, name in ((-1.0, "left_web"), (1.0, "right_web")):
        part.visual(
            Box((0.006, 0.028, 0.024)),
            origin=Origin(xyz=(sign * 0.017, 0.018, 0.018)),
            material=hardware,
            name=name,
        )
    part.visual(
        Box((0.040, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.026, 0.028)),
        material=hardware,
        name="lip",
    )


def _add_latch(part, *, hardware, grip) -> None:
    part.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="barrel",
    )
    part.visual(
        Box((0.032, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, -0.002, -0.018)),
        material=hardware,
        name="strap",
    )
    part.visual(
        Box((0.036, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.003, -0.032)),
        material=grip,
        name="pad",
    )
    part.visual(
        Box((0.032, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.002, -0.040)),
        material=hardware,
        name="hook",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_tackle_box")

    body_green = model.material("body_green", rgba=(0.28, 0.34, 0.24, 1.0))
    lid_green = model.material("lid_green", rgba=(0.30, 0.37, 0.26, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    keeper_steel = model.material("keeper_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body_width = 0.44
    body_depth = 0.25
    body_height = 0.152
    wall = 0.008
    floor = 0.008
    hinge_y = -(body_depth * 0.5) - 0.007
    hinge_z = body_height + 0.010

    lid_outer_depth = 0.265
    lid_top_depth = 0.247
    lid_skirt_depth = 0.249
    lid_width = body_width + 0.012
    lid_top = 0.008
    lid_skirt = 0.008
    lid_skirt_height = 0.052

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )
    body.visual(
        Box((body_width - 0.016, body_depth - 0.016, floor)),
        origin=Origin(xyz=(0.0, 0.0, floor * 0.5)),
        material=body_green,
        name="floor_pan",
    )
    body.visual(
        Box((wall, body_depth, body_height - 0.004)),
        origin=Origin(xyz=((body_width - wall) * 0.5, 0.0, 0.078)),
        material=body_green,
        name="right_wall",
    )
    body.visual(
        Box((wall, body_depth, body_height - 0.004)),
        origin=Origin(xyz=(-(body_width - wall) * 0.5, 0.0, 0.078)),
        material=body_green,
        name="left_wall",
    )
    body.visual(
        Box((body_width - 0.016, wall, body_height - 0.004)),
        origin=Origin(xyz=(0.0, (body_depth - wall) * 0.5, 0.078)),
        material=body_green,
        name="front_wall",
    )
    body.visual(
        Box((body_width - 0.016, wall, body_height - 0.004)),
        origin=Origin(xyz=(0.0, -(body_depth - wall) * 0.5, 0.078)),
        material=body_green,
        name="rear_wall",
    )
    body.visual(
        Box((body_width - 0.060, 0.028, 0.016)),
        origin=Origin(xyz=(0.0, 0.058, 0.008)),
        material=hardware_dark,
        name="front_rail",
    )
    body.visual(
        Box((body_width - 0.060, 0.028, 0.016)),
        origin=Origin(xyz=(0.0, -0.058, 0.008)),
        material=hardware_dark,
        name="rear_rail",
    )
    body.visual(
        Box((0.084, 0.022, 0.014)),
        origin=Origin(xyz=(-0.158, (body_depth - 0.022) * 0.5, 0.110)),
        material=hardware_dark,
        name="left_latch_pad",
    )
    body.visual(
        Box((0.084, 0.022, 0.014)),
        origin=Origin(xyz=(0.158, (body_depth - 0.022) * 0.5, 0.110)),
        material=hardware_dark,
        name="right_latch_pad",
    )
    body.visual(
        Box((0.110, 0.018, 0.020)),
        origin=Origin(xyz=(-0.165, hinge_y + 0.008, hinge_z - 0.020)),
        material=hardware_dark,
        name="left_hinge_block",
    )
    body.visual(
        Box((0.110, 0.018, 0.020)),
        origin=Origin(xyz=(0.165, hinge_y + 0.008, hinge_z - 0.020)),
        material=hardware_dark,
        name="right_hinge_block",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(-0.165, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_dark,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(0.165, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_dark,
        name="right_hinge_barrel",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_outer_depth, 0.060)),
        mass=1.3,
        origin=Origin(xyz=(0.0, lid_outer_depth * 0.5, -0.028)),
    )
    lid.visual(
        Box((lid_width, lid_top_depth, lid_top)),
        origin=Origin(xyz=(0.0, 0.1335, -0.004)),
        material=lid_green,
        name="top_panel",
    )
    lid.visual(
        Box((lid_skirt, lid_skirt_depth, lid_skirt_height)),
        origin=Origin(xyz=((lid_width - lid_skirt) * 0.5, 0.1325, -0.030)),
        material=lid_green,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_skirt, lid_skirt_depth, lid_skirt_height)),
        origin=Origin(xyz=(-(lid_width - lid_skirt) * 0.5, 0.1325, -0.030)),
        material=lid_green,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_width, lid_skirt, lid_skirt_height)),
        origin=Origin(xyz=(0.0, 0.261, -0.030)),
        material=lid_green,
        name="front_skirt",
    )
    lid.visual(
        Box((0.220, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.006, -0.004)),
        material=hardware_dark,
        name="center_hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.210),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_dark,
        name="center_hinge_barrel",
    )
    for sign, side in ((-1.0, "left"), (1.0, "right")):
        lid.visual(
            Box((0.056, 0.012, 0.008)),
            origin=Origin(xyz=(sign * 0.130, 0.267, 0.004)),
            material=hardware_dark,
            name=f"{side}_latch_cap",
        )
        lid.visual(
            Box((0.010, 0.012, 0.028)),
            origin=Origin(xyz=(sign * 0.150, 0.267, -0.014)),
            material=hardware_dark,
            name=f"{side}_outer_cheek",
        )
        lid.visual(
            Box((0.010, 0.012, 0.028)),
            origin=Origin(xyz=(sign * 0.110, 0.267, -0.014)),
            material=hardware_dark,
            name=f"{side}_inner_cheek",
        )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.220, 0.030, 0.065)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )
    handle.visual(
        Box((0.030, 0.018, 0.008)),
        origin=Origin(xyz=(-0.100, 0.0, 0.004)),
        material=hardware_dark,
        name="left_foot",
    )
    handle.visual(
        Box((0.030, 0.018, 0.008)),
        origin=Origin(xyz=(0.100, 0.0, 0.004)),
        material=hardware_dark,
        name="right_foot",
    )
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.100, 0.0, 0.008),
                    (-0.100, 0.0, 0.034),
                    (-0.060, 0.0, 0.056),
                    (0.060, 0.0, 0.056),
                    (0.100, 0.0, 0.034),
                    (0.100, 0.0, 0.008),
                ],
                radius=0.006,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
            "tackle_box_handle",
        ),
        material=handle_black,
        name="grip_loop",
    )

    left_keeper = model.part("left_keeper")
    left_keeper.inertial = Inertial.from_geometry(
        Box((0.040, 0.032, 0.032)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.016, 0.016)),
    )
    _add_keeper(left_keeper, hardware=keeper_steel)

    right_keeper = model.part("right_keeper")
    right_keeper.inertial = Inertial.from_geometry(
        Box((0.040, 0.032, 0.032)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.016, 0.016)),
    )
    _add_keeper(right_keeper, hardware=keeper_steel)

    left_latch = model.part("left_latch")
    left_latch.inertial = Inertial.from_geometry(
        Box((0.036, 0.022, 0.046)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.010, -0.020)),
    )
    _add_latch(left_latch, hardware=hardware_dark, grip=handle_black)

    right_latch = model.part("right_latch")
    right_latch.inertial = Inertial.from_geometry(
        Box((0.036, 0.022, 0.046)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.010, -0.020)),
    )
    _add_latch(right_latch, hardware=hardware_dark, grip=handle_black)

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.FIXED,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, 0.136, 0.0)),
    )
    model.articulation(
        "body_to_left_keeper",
        ArticulationType.FIXED,
        parent=body,
        child=left_keeper,
        origin=Origin(xyz=(-0.130, body_depth * 0.5, 0.080)),
    )
    model.articulation(
        "body_to_right_keeper",
        ArticulationType.FIXED,
        parent=body,
        child=right_keeper,
        origin=Origin(xyz=(0.130, body_depth * 0.5, 0.080)),
    )
    model.articulation(
        "lid_to_left_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=left_latch,
        origin=Origin(xyz=(-0.130, 0.273, -0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "lid_to_right_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=right_latch,
        origin=Origin(xyz=(0.130, 0.273, -0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    left_keeper = object_model.get_part("left_keeper")
    right_keeper = object_model.get_part("right_keeper")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    lid_hinge = object_model.get_articulation("body_to_lid")
    left_latch_joint = object_model.get_articulation("lid_to_left_latch")
    right_latch_joint = object_model.get_articulation("lid_to_right_latch")

    top_panel = lid.get_visual("top_panel")
    front_skirt = lid.get_visual("front_skirt")
    front_wall = body.get_visual("front_wall")
    left_lip = left_keeper.get_visual("lip")
    right_lip = right_keeper.get_visual("lip")
    left_hook = left_latch.get_visual("hook")
    right_hook = right_latch.get_visual("hook")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "lid hinge axis is across box width",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0),
        f"expected lid hinge axis (1, 0, 0), got {lid_hinge.axis}",
    )
    ctx.check(
        "latch axes are shared serviceable pivots",
        tuple(left_latch_joint.axis) == (1.0, 0.0, 0.0) and tuple(right_latch_joint.axis) == (1.0, 0.0, 0.0),
        f"left={left_latch_joint.axis}, right={right_latch_joint.axis}",
    )

    ctx.expect_contact(handle, lid, name="handle feet bear on lid shell")
    ctx.expect_contact(left_keeper, body, name="left keeper fastens to body")
    ctx.expect_contact(right_keeper, body, name="right keeper fastens to body")
    ctx.expect_contact(left_latch, lid, name="left latch rides on lid hardware")
    ctx.expect_contact(right_latch, lid, name="right latch rides on lid hardware")

    with ctx.pose({lid_hinge: 0.0, left_latch_joint: 0.0, right_latch_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.001,
            max_gap=0.006,
            positive_elem=top_panel,
            negative_elem=front_wall,
            name="closed lid panel sits just above shell rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            min_overlap=0.36,
            elem_a=top_panel,
            elem_b=front_wall,
            name="lid covers shell width",
        )
        ctx.expect_gap(
            left_keeper,
            left_latch,
            axis="y",
            min_gap=0.0005,
            max_gap=0.006,
            positive_elem=left_lip,
            negative_elem=left_hook,
            name="left latch closes just behind keeper lip",
        )
        ctx.expect_gap(
            right_keeper,
            right_latch,
            axis="y",
            min_gap=0.0005,
            max_gap=0.006,
            positive_elem=right_lip,
            negative_elem=right_hook,
            name="right latch closes just behind keeper lip",
        )

    with ctx.pose({left_latch_joint: 1.45, right_latch_joint: 1.45}):
        ctx.expect_gap(
            left_latch,
            left_keeper,
            axis="z",
            min_gap=0.010,
            positive_elem=left_hook,
            negative_elem=left_lip,
            name="left latch can swing clear for release",
        )
        ctx.expect_gap(
            right_latch,
            right_keeper,
            axis="z",
            min_gap=0.010,
            positive_elem=right_hook,
            negative_elem=right_lip,
            name="right latch can swing clear for release",
        )

    with ctx.pose({lid_hinge: 1.20, left_latch_joint: 1.45, right_latch_joint: 1.45}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.080,
            positive_elem=front_skirt,
            negative_elem=front_wall,
            name="lid opens for maintenance access",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
