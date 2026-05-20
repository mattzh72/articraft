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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_panel")

    frame_mat = model.material("frame_powdercoat", color=(0.80, 0.82, 0.84))
    panel_mat = model.material("panel_powdercoat", color=(0.92, 0.93, 0.91))
    latch_mat = model.material("latch_zinc", color=(0.56, 0.58, 0.60))

    frame_outer_w = 0.54
    frame_outer_h = 0.74
    opening_w = 0.45
    opening_h = 0.65
    frame_band = (frame_outer_w - opening_w) / 2.0
    frame_flange_t = 0.0025
    jamb_depth = 0.03
    jamb_wall = 0.018

    door_w = 0.47
    door_h = 0.67
    door_skin_t = 0.0012
    door_pan_depth = 0.022
    door_return_t = 0.0012
    return_inset = 0.014

    hinge_axis_x = 0.003
    hinge_axis_y = -door_w / 2.0
    latch_pivot_y = 0.418
    hinge_radius = 0.006

    def add_box(part, name, size, xyz, material):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    def add_cylinder(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    frame = model.part("frame")

    add_box(
        frame,
        "front_left",
        (frame_flange_t, frame_band, frame_outer_h),
        (0.0, -(opening_w / 2.0 + frame_band / 2.0), 0.0),
        frame_mat,
    )
    add_box(
        frame,
        "front_right",
        (frame_flange_t, frame_band, frame_outer_h),
        (0.0, opening_w / 2.0 + frame_band / 2.0, 0.0),
        frame_mat,
    )
    add_box(
        frame,
        "front_top",
        (frame_flange_t, opening_w, frame_band),
        (0.0, 0.0, opening_h / 2.0 + frame_band / 2.0),
        frame_mat,
    )
    add_box(
        frame,
        "front_bottom",
        (frame_flange_t, opening_w, frame_band),
        (0.0, 0.0, -(opening_h / 2.0 + frame_band / 2.0)),
        frame_mat,
    )

    jamb_x = -(jamb_depth / 2.0) + frame_flange_t / 2.0
    add_box(
        frame,
        "jamb_left",
        (jamb_depth, jamb_wall, opening_h),
        (jamb_x, -(opening_w / 2.0 + jamb_wall / 2.0), 0.0),
        frame_mat,
    )
    add_box(
        frame,
        "jamb_right",
        (jamb_depth, jamb_wall, opening_h),
        (jamb_x, opening_w / 2.0 + jamb_wall / 2.0, 0.0),
        frame_mat,
    )
    add_box(
        frame,
        "jamb_top",
        (jamb_depth, opening_w + 2.0 * jamb_wall, jamb_wall),
        (jamb_x, 0.0, opening_h / 2.0 + jamb_wall / 2.0),
        frame_mat,
    )
    add_box(
        frame,
        "jamb_bottom",
        (jamb_depth, opening_w + 2.0 * jamb_wall, jamb_wall),
        (jamb_x, 0.0, -(opening_h / 2.0 + jamb_wall / 2.0)),
        frame_mat,
    )

    add_box(
        frame,
        "keeper",
        (0.003, 0.016, 0.038),
        (0.0025, 0.242, 0.0),
        frame_mat,
    )
    add_box(
        frame,
        "stop_lip",
        (0.004, 0.018, 0.05),
        (-0.011, 0.216, 0.0),
        frame_mat,
    )

    add_box(
        frame,
        "hinge_leaf_top",
        (0.005, 0.014, 0.16),
        (0.0005, hinge_axis_y - 0.005, 0.25),
        frame_mat,
    )
    add_box(
        frame,
        "hinge_leaf_bottom",
        (0.005, 0.014, 0.16),
        (0.0005, hinge_axis_y - 0.005, -0.25),
        frame_mat,
    )
    add_cylinder(
        frame,
        "hinge_barrel_top",
        hinge_radius,
        0.16,
        (hinge_axis_x, hinge_axis_y, 0.25),
        frame_mat,
    )
    add_cylinder(
        frame,
        "hinge_barrel_bottom",
        hinge_radius,
        0.16,
        (hinge_axis_x, hinge_axis_y, -0.25),
        frame_mat,
    )

    panel = model.part("panel")

    add_box(
        panel,
        "skin",
        (door_skin_t, door_w, door_h),
        (door_skin_t / 2.0, door_w / 2.0, 0.0),
        panel_mat,
    )

    return_x = -(door_pan_depth / 2.0) + door_skin_t / 2.0
    add_box(
        panel,
        "return_left",
        (door_pan_depth, door_return_t, door_h - 0.002),
        (return_x, return_inset + door_return_t / 2.0, 0.0),
        panel_mat,
    )
    add_box(
        panel,
        "return_right",
        (door_pan_depth, door_return_t, door_h - 0.002),
        (return_x, door_w - return_inset - door_return_t / 2.0, 0.0),
        panel_mat,
    )
    add_box(
        panel,
        "return_top",
        (door_pan_depth, door_w - 2.0 * return_inset, door_return_t),
        (return_x, door_w / 2.0, door_h / 2.0 - return_inset - door_return_t / 2.0),
        panel_mat,
    )
    add_box(
        panel,
        "return_bottom",
        (door_pan_depth, door_w - 2.0 * return_inset, door_return_t),
        (return_x, door_w / 2.0, -(door_h / 2.0 - return_inset - door_return_t / 2.0)),
        panel_mat,
    )
    add_box(
        panel,
        "stop_pad",
        (0.004, 0.008, 0.05),
        (-0.018, 0.454, 0.0),
        panel_mat,
    )

    add_box(
        panel,
        "hinge_leaf",
        (0.005, 0.014, 0.18),
        (0.0005, 0.007, 0.0),
        panel_mat,
    )
    add_cylinder(
        panel,
        "hinge_barrel_mid",
        hinge_radius,
        0.18,
        (0.0, 0.0, 0.0),
        panel_mat,
    )

    latch = model.part("latch")
    add_cylinder(
        latch,
        "pivot_pad",
        0.014,
        0.003,
        (0.0015, 0.0, 0.0),
        latch_mat,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    add_cylinder(
        latch,
        "pivot_boss",
        0.006,
        0.006,
        (0.004, 0.0, 0.0),
        latch_mat,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    add_box(
        latch,
        "arm",
        (0.0022, 0.07, 0.018),
        (0.0041, 0.03, 0.0),
        latch_mat,
    )
    add_box(
        latch,
        "finger_tab",
        (0.004, 0.018, 0.03),
        (0.005, 0.061, 0.0),
        latch_mat,
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "panel_to_latch",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=latch,
        origin=Origin(xyz=(door_skin_t, latch_pivot_y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("frame_to_panel")
    latch_joint = object_model.get_articulation("panel_to_latch")

    stop_lip = frame.get_visual("stop_lip")
    keeper = frame.get_visual("keeper")
    skin = panel.get_visual("skin")
    stop_pad = panel.get_visual("stop_pad")
    pivot_pad = latch.get_visual("pivot_pad")
    arm = latch.get_visual("arm")

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

    with ctx.pose({door_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_contact(frame, panel, elem_a=stop_lip, elem_b=stop_pad)
        ctx.expect_contact(panel, latch, elem_a=skin, elem_b=pivot_pad)
        ctx.expect_overlap(
            latch,
            frame,
            axes="yz",
            elem_a=arm,
            elem_b=keeper,
            min_overlap=0.012,
            name="closed latch covers keeper",
        )
        ctx.expect_gap(
            latch,
            frame,
            axis="x",
            positive_elem=arm,
            negative_elem=keeper,
            min_gap=0.0005,
            max_gap=0.004,
            name="closed latch stands just above keeper",
        )

    with ctx.pose({door_hinge: 0.0, latch_joint: pi / 2.0}):
        ctx.expect_gap(
            frame,
            latch,
            axis="y",
            positive_elem=keeper,
            negative_elem=arm,
            min_gap=0.02,
            name="open latch clears keeper laterally",
        )

    with ctx.pose({door_hinge: 0.0}):
        closed_skin = ctx.part_element_world_aabb(panel, elem="skin")
    with ctx.pose({door_hinge: 1.2}):
        open_skin = ctx.part_element_world_aabb(panel, elem="skin")

    door_opens_outward = (
        closed_skin is not None
        and open_skin is not None
        and open_skin[1][0] > closed_skin[1][0] + 0.10
    )
    ctx.check(
        "door opens outward from hinge side",
        door_opens_outward,
        details=(
            f"closed_skin={closed_skin}, open_skin={open_skin}; "
            "expected open pose to move the free edge outward in +x."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
