from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _u_channel_mesh(
    *,
    outer_width: float,
    length: float,
    wall_thickness: float,
    wall_height: float,
    cap_thickness: float,
    name: str,
):
    cap = BoxGeometry((outer_width, length, cap_thickness))
    cap.translate(0.0, 0.0, wall_height - cap_thickness * 0.5)

    left_wall = BoxGeometry((wall_thickness, length, wall_height))
    left_wall.translate(-(outer_width - wall_thickness) * 0.5, 0.0, wall_height * 0.5)

    right_wall = BoxGeometry((wall_thickness, length, wall_height))
    right_wall.translate((outer_width - wall_thickness) * 0.5, 0.0, wall_height * 0.5)

    return mesh_from_geometry(cap.merge(left_wall).merge(right_wall), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floating_wall_mount_desk")

    powder_black = model.material("powder_black", rgba=(0.15, 0.16, 0.17, 1.0))
    warm_oak = model.material("warm_oak", rgba=(0.72, 0.57, 0.40, 1.0))
    soft_grey = model.material("soft_grey", rgba=(0.78, 0.79, 0.80, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.29, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.64, 1.0))

    hinge_axis_y = 0.042
    hinge_axis_z = 0.665

    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        Box((1.08, 0.018, 0.18)),
        origin=Origin(xyz=(0.0, 0.009, hinge_axis_z)),
        material=powder_black,
        name="backer_rail",
    )
    for side, x in (("left", -0.39), ("right", 0.39)):
        wall_mount.visual(
            Box((0.16, 0.024, 0.24)),
            origin=Origin(xyz=(x, 0.012, hinge_axis_z)),
            material=powder_black,
            name=f"{side}_wall_plate",
        )
    wall_mount.visual(
        Cylinder(radius=0.018, length=0.08),
        origin=Origin(
            xyz=(-0.43, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="left_hinge_ear",
    )
    wall_mount.visual(
        Cylinder(radius=0.018, length=0.08),
        origin=Origin(
            xyz=(0.43, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="right_hinge_ear",
    )
    wall_mount.inertial = Inertial.from_geometry(
        Box((1.08, 0.05, 0.24)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.015, hinge_axis_z)),
    )

    support_frame = model.part("support_frame")
    support_frame.visual(
        Cylinder(radius=0.018, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    support_frame.visual(
        Box((0.72, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.045, 0.045)),
        material=powder_black,
        name="rear_tie",
    )
    for side, x in (("left", -0.30), ("right", 0.30)):
        support_frame.visual(
            Box((0.06, 0.05, 0.05)),
            origin=Origin(xyz=(x, 0.043, 0.025)),
            material=powder_black,
            name=f"{side}_hinge_lug",
        )
        support_frame.visual(
            Box((0.06, 0.40, 0.03)),
            origin=Origin(xyz=(x, 0.24, 0.045)),
            material=powder_black,
            name=f"{side}_arm",
        )
        support_frame.visual(
            Box((0.024, 0.34, 0.02)),
            origin=Origin(xyz=(x, 0.21, 0.01)),
            material=powder_black,
            name=f"{side}_lower_rail",
        )
        support_frame.visual(
            Box((0.038, 0.04, 0.07)),
            origin=Origin(xyz=(x, 0.45, 0.025)),
            material=powder_black,
            name=f"{side}_front_upright",
        )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.80, 0.56, 0.08)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.26, 0.03)),
    )

    model.articulation(
        "wall_hinge",
        ArticulationType.REVOLUTE,
        parent=wall_mount,
        child=support_frame,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.6,
            lower=0.0,
            upper=1.15,
        ),
    )

    worktop = model.part("worktop")
    channel_mesh = _u_channel_mesh(
        outer_width=0.032,
        length=0.32,
        wall_thickness=0.006,
        wall_height=0.022,
        cap_thickness=0.006,
        name="desk_guide_channel",
    )
    worktop.visual(
        Box((1.22, 0.56, 0.032)),
        origin=Origin(xyz=(0.0, 0.28, 0.016)),
        material=warm_oak,
        name="top_panel",
    )
    worktop.visual(
        Box((0.92, 0.026, 0.05)),
        origin=Origin(xyz=(0.0, 0.525, -0.009)),
        material=graphite,
        name="front_stretcher",
    )
    for side, x in (("left", -0.22), ("right", 0.22)):
        worktop.visual(
            channel_mesh,
            origin=Origin(xyz=(x, 0.24, -0.022)),
            material=graphite,
            name=f"{side}_channel",
        )
    worktop.inertial = Inertial.from_geometry(
        Box((1.22, 0.56, 0.06)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.28, 0.010)),
    )

    model.articulation(
        "support_to_worktop",
        ArticulationType.FIXED,
        parent=support_frame,
        child=worktop,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.012, 0.24, 0.014)),
        origin=Origin(xyz=(-0.22, 0.12, -0.013)),
        material=graphite,
        name="left_runner",
    )
    drawer.visual(
        Box((0.012, 0.24, 0.014)),
        origin=Origin(xyz=(0.22, 0.12, -0.013)),
        material=graphite,
        name="right_runner",
    )
    drawer.visual(
        Box((0.010, 0.20, 0.016)),
        origin=Origin(xyz=(-0.22, 0.12, -0.028)),
        material=graphite,
        name="left_hanger",
    )
    drawer.visual(
        Box((0.010, 0.20, 0.016)),
        origin=Origin(xyz=(0.22, 0.12, -0.028)),
        material=graphite,
        name="right_hanger",
    )
    drawer.visual(
        Box((0.018, 0.20, 0.006)),
        origin=Origin(xyz=(-0.206, 0.12, -0.037)),
        material=graphite,
        name="left_bridge",
    )
    drawer.visual(
        Box((0.018, 0.20, 0.006)),
        origin=Origin(xyz=(0.206, 0.12, -0.037)),
        material=graphite,
        name="right_bridge",
    )
    drawer.visual(
        Box((0.376, 0.22, 0.008)),
        origin=Origin(xyz=(0.0, 0.12, -0.048)),
        material=soft_grey,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.012, 0.22, 0.028)),
        origin=Origin(xyz=(-0.191, 0.12, -0.048)),
        material=soft_grey,
        name="left_side",
    )
    drawer.visual(
        Box((0.012, 0.22, 0.028)),
        origin=Origin(xyz=(0.191, 0.12, -0.048)),
        material=soft_grey,
        name="right_side",
    )
    drawer.visual(
        Box((0.388, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, 0.016, -0.048)),
        material=soft_grey,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.50, 0.016, 0.032)),
        origin=Origin(xyz=(0.0, 0.236, -0.048)),
        material=graphite,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.16, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.248, -0.048)),
        material=steel,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.50, 0.26, 0.06)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.13, -0.040)),
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=worktop,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.08, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=0.14,
        ),
    )

    keyboard_shelf = model.part("keyboard_shelf")
    keyboard_shelf.visual(
        Box((0.82, 0.018, 0.24)),
        origin=Origin(xyz=(0.0, -0.009, -0.120)),
        material=warm_oak,
        name="shelf_panel",
    )
    keyboard_shelf.visual(
        Box((0.82, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, -0.006, -0.226)),
        material=graphite,
        name="front_lip",
    )
    keyboard_shelf.inertial = Inertial.from_geometry(
        Box((0.82, 0.03, 0.24)),
        mass=3.0,
        origin=Origin(xyz=(0.0, -0.012, -0.120)),
    )

    model.articulation(
        "keyboard_hinge",
        ArticulationType.REVOLUTE,
        parent=worktop,
        child=keyboard_shelf,
        origin=Origin(xyz=(0.0, 0.578, 0.013)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_mount = object_model.get_part("wall_mount")
    support_frame = object_model.get_part("support_frame")
    worktop = object_model.get_part("worktop")
    drawer = object_model.get_part("drawer")
    keyboard_shelf = object_model.get_part("keyboard_shelf")

    wall_hinge = object_model.get_articulation("wall_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    keyboard_hinge = object_model.get_articulation("keyboard_hinge")

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

    ctx.expect_contact(
        support_frame,
        wall_mount,
        name="support frame mounts on the wall hinge line",
    )
    ctx.expect_contact(
        worktop,
        support_frame,
        elem_a="top_panel",
        elem_b="left_arm",
        name="left arm supports the worktop",
    )
    ctx.expect_contact(
        worktop,
        support_frame,
        elem_a="top_panel",
        elem_b="right_arm",
        name="right arm supports the worktop",
    )
    ctx.expect_gap(
        worktop,
        drawer,
        axis="z",
        max_gap=0.04,
        max_penetration=0.0,
        positive_elem="top_panel",
        negative_elem="drawer_front",
        name="drawer front sits just below the worktop",
    )
    ctx.expect_within(
        drawer,
        worktop,
        axes="xz",
        inner_elem="left_runner",
        outer_elem="left_channel",
        margin=0.0015,
        name="left drawer runner stays inside the left guide channel",
    )
    ctx.expect_within(
        drawer,
        worktop,
        axes="xz",
        inner_elem="right_runner",
        outer_elem="right_channel",
        margin=0.0015,
        name="right drawer runner stays inside the right guide channel",
    )

    folded_top_rest = ctx.part_element_world_aabb(worktop, elem="top_panel")
    with ctx.pose({wall_hinge: wall_hinge.motion_limits.upper}):
        folded_top_open = ctx.part_element_world_aabb(worktop, elem="top_panel")
    ctx.check(
        "worktop folds upward on the wall hinges",
        folded_top_rest is not None
        and folded_top_open is not None
        and folded_top_open[1][2] > folded_top_rest[1][2] + 0.35
        and folded_top_open[1][1] < folded_top_rest[1][1] - 0.20,
        details=f"rest={folded_top_rest}, folded={folded_top_open}",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_within(
            drawer,
            worktop,
            axes="xz",
            inner_elem="left_runner",
            outer_elem="left_channel",
            margin=0.0015,
            name="left runner stays aligned at full extension",
        )
        ctx.expect_within(
            drawer,
            worktop,
            axes="xz",
            inner_elem="right_runner",
            outer_elem="right_channel",
            margin=0.0015,
            name="right runner stays aligned at full extension",
        )
        ctx.expect_overlap(
            drawer,
            worktop,
            axes="y",
            elem_a="left_runner",
            elem_b="left_channel",
            min_overlap=0.16,
            name="left runner retains insertion in the channel",
        )
        ctx.expect_overlap(
            drawer,
            worktop,
            axes="y",
            elem_a="right_runner",
            elem_b="right_channel",
            min_overlap=0.16,
            name="right runner retains insertion in the channel",
        )
    ctx.check(
        "drawer slides outward from under the desk",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > closed_drawer_pos[1] + 0.10,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    closed_shelf_aabb = ctx.part_world_aabb(keyboard_shelf)
    with ctx.pose({keyboard_hinge: keyboard_hinge.motion_limits.upper}):
        open_shelf_aabb = ctx.part_world_aabb(keyboard_shelf)
        ctx.expect_gap(
            keyboard_shelf,
            worktop,
            axis="y",
            min_gap=0.015,
            max_gap=0.03,
            positive_elem="shelf_panel",
            negative_elem="top_panel",
            name="opened keyboard shelf projects forward from the worktop edge",
        )
    ctx.check(
        "keyboard shelf folds down and forward",
        closed_shelf_aabb is not None
        and open_shelf_aabb is not None
        and open_shelf_aabb[1][1] > closed_shelf_aabb[1][1] + 0.20,
        details=f"closed={closed_shelf_aabb}, open={open_shelf_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
