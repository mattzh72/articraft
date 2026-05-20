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
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _diamond_hole(cx: float, cy: float, rx: float, ry: float) -> list[tuple[float, float]]:
    return [
        (cx - rx, cy),
        (cx, cy - ry),
        (cx + rx, cy),
        (cx, cy + ry),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="privacy_garden_gate")

    cedar = model.material("cedar", rgba=(0.55, 0.39, 0.23, 1.0))
    cedar_dark = model.material("cedar_dark", rgba=(0.42, 0.28, 0.16, 1.0))
    metal = model.material("powder_coat_black", rgba=(0.14, 0.14, 0.15, 1.0))
    concrete = model.material("footing_concrete", rgba=(0.50, 0.50, 0.50, 1.0))

    post_size = 0.12
    post_depth = 0.12
    post_embed = 0.25
    post_above_grade = 2.00
    post_height = post_embed + post_above_grade
    post_center_z = (post_above_grade - post_embed) * 0.5

    opening = 0.95
    post_spacing = opening + post_size
    footing_height = 0.10
    footing_length = post_spacing + post_size
    footing_depth = 0.16

    leaf_width = 0.91
    leaf_height = 1.78
    leaf_thickness = 0.045
    hinge_axis_x = post_size * 0.5 + 0.012
    leaf_base_z = 0.045

    stile_width = 0.09
    rail_height = 0.10
    lower_panel_height = 1.08
    lattice_height = 0.40
    frame_left = 0.018
    frame_right = leaf_width
    clear_width = frame_right - frame_left - 2.0 * stile_width
    clear_center_x = frame_left + stile_width + clear_width * 0.5

    lower_panel_center_z = rail_height + lower_panel_height * 0.5
    mid_rail_center_z = rail_height + lower_panel_height + rail_height * 0.5
    lattice_center_z = rail_height + lower_panel_height + rail_height + lattice_height * 0.5
    top_rail_center_z = leaf_height - rail_height * 0.5

    hinge_centers = (0.28, 1.50)
    barrel_radius = 0.010
    barrel_segment = 0.028
    barrel_pitch = 0.031

    support = model.part("support_frame")
    support.visual(
        Box((footing_length, footing_depth, footing_height)),
        origin=Origin(xyz=(post_spacing * 0.5, 0.0, -0.20)),
        material=concrete,
        name="footing",
    )
    support.visual(
        Box((post_size, post_depth, post_height)),
        origin=Origin(xyz=(0.0, 0.0, post_center_z)),
        material=cedar_dark,
        name="hinge_post",
    )
    support.visual(
        Box((post_size, post_depth, post_height)),
        origin=Origin(xyz=(post_spacing, 0.0, post_center_z)),
        material=cedar_dark,
        name="latch_post",
    )
    support.visual(
        Box((0.14, 0.14, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, post_above_grade + 0.015)),
        material=cedar_dark,
        name="hinge_post_cap",
    )
    support.visual(
        Box((0.14, 0.14, 0.03)),
        origin=Origin(xyz=(post_spacing, 0.0, post_above_grade + 0.015)),
        material=cedar_dark,
        name="latch_post_cap",
    )
    support.visual(
        Box((0.008, 0.030, 0.14)),
        origin=Origin(xyz=(post_spacing - post_size * 0.5 - 0.004, 0.026, leaf_base_z + 0.985)),
        material=metal,
        name="strike_block",
    )
    support.visual(
        Box((0.022, 0.010, 0.18)),
        origin=Origin(xyz=(0.051, 0.0, leaf_base_z + hinge_centers[0])),
        material=metal,
        name="lower_post_hinge_plate",
    )
    support.visual(
        Box((0.022, 0.010, 0.18)),
        origin=Origin(xyz=(0.051, 0.0, leaf_base_z + hinge_centers[1])),
        material=metal,
        name="upper_post_hinge_plate",
    )
    for hinge_name, hinge_z in (("lower", hinge_centers[0]), ("upper", hinge_centers[1])):
        support.visual(
            Cylinder(radius=barrel_radius, length=barrel_segment),
            origin=Origin(xyz=(hinge_axis_x, 0.0, leaf_base_z + hinge_z - barrel_pitch)),
            material=metal,
            name=f"{hinge_name}_post_knuckle_bottom",
        )
        support.visual(
            Cylinder(radius=barrel_radius, length=barrel_segment),
            origin=Origin(xyz=(hinge_axis_x, 0.0, leaf_base_z + hinge_z + barrel_pitch)),
            material=metal,
            name=f"{hinge_name}_post_knuckle_top",
        )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((stile_width, leaf_thickness, leaf_height)),
        origin=Origin(xyz=(frame_left + stile_width * 0.5, 0.0, leaf_height * 0.5)),
        material=cedar,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((stile_width, leaf_thickness, leaf_height)),
        origin=Origin(xyz=(frame_right - stile_width * 0.5, 0.0, leaf_height * 0.5)),
        material=cedar,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((clear_width, leaf_thickness, rail_height)),
        origin=Origin(xyz=(clear_center_x, 0.0, rail_height * 0.5)),
        material=cedar,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((clear_width, leaf_thickness, rail_height)),
        origin=Origin(xyz=(clear_center_x, 0.0, mid_rail_center_z)),
        material=cedar,
        name="mid_rail",
    )
    gate_leaf.visual(
        Box((clear_width, leaf_thickness, rail_height)),
        origin=Origin(xyz=(clear_center_x, 0.0, top_rail_center_z)),
        material=cedar,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((clear_width, 0.022, lower_panel_height)),
        origin=Origin(xyz=(clear_center_x, 0.0, lower_panel_center_z)),
        material=cedar_dark,
        name="lower_privacy_panel",
    )

    lattice_holes = []
    for hole_z in (-0.095, 0.095):
        for hole_x in (-0.215, 0.0, 0.215):
            lattice_holes.append(_diamond_hole(hole_x, hole_z, 0.070, 0.052))
    lattice_geom = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(clear_width, lattice_height),
            lattice_holes,
            height=0.014,
            center=True,
        ).rotate_x(pi * 0.5),
        "upper_lattice_panel",
    )
    gate_leaf.visual(
        lattice_geom,
        origin=Origin(xyz=(clear_center_x, 0.0, lattice_center_z)),
        material=cedar_dark,
        name="upper_lattice_panel",
    )
    gate_leaf.visual(
        Box((0.050, 0.010, 0.16)),
        origin=Origin(xyz=(0.035, 0.0, hinge_centers[0])),
        material=metal,
        name="lower_leaf_hinge_plate",
    )
    gate_leaf.visual(
        Box((0.050, 0.010, 0.16)),
        origin=Origin(xyz=(0.035, 0.0, hinge_centers[1])),
        material=metal,
        name="upper_leaf_hinge_plate",
    )
    for hinge_name, hinge_z in (("lower", hinge_centers[0]), ("upper", hinge_centers[1])):
        gate_leaf.visual(
            Cylinder(radius=barrel_radius, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=metal,
            name=f"{hinge_name}_leaf_knuckle",
        )

    rotary_latch = model.part("rotary_latch")
    rotary_latch.visual(
        Box((0.050, 0.008, 0.090)),
        origin=Origin(xyz=(-0.020, -0.0045, 0.0)),
        material=metal,
        name="latch_backplate",
    )
    rotary_latch.visual(
        Cylinder(radius=0.018, length=0.017),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=metal,
        name="latch_hub",
    )
    rotary_latch.visual(
        Box((0.036, 0.012, 0.026)),
        origin=Origin(xyz=(0.018, 0.003, 0.0)),
        material=metal,
        name="latch_cam",
    )
    rotary_latch.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(-0.012, 0.005, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=metal,
        name="latch_grip",
    )

    model.articulation(
        "support_to_gate",
        ArticulationType.REVOLUTE,
        parent=support,
        child=gate_leaf,
        origin=Origin(xyz=(hinge_axis_x, 0.0, leaf_base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "gate_to_latch",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=rotary_latch,
        origin=Origin(xyz=(0.890, leaf_thickness * 0.5 + 0.0085, 0.985)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-1.0, upper=1.0),
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

    support = object_model.get_part("support_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    rotary_latch = object_model.get_part("rotary_latch")

    gate_hinge = object_model.get_articulation("support_to_gate")
    latch_joint = object_model.get_articulation("gate_to_latch")

    hinge_post = support.get_visual("hinge_post")
    latch_post = support.get_visual("latch_post")
    strike_block = support.get_visual("strike_block")
    left_stile = gate_leaf.get_visual("left_stile")
    right_stile = gate_leaf.get_visual("right_stile")
    latch_backplate = rotary_latch.get_visual("latch_backplate")
    latch_cam = rotary_latch.get_visual("latch_cam")

    with ctx.pose({gate_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            gate_leaf,
            support,
            axis="x",
            positive_elem=left_stile,
            negative_elem=hinge_post,
            min_gap=0.020,
            max_gap=0.040,
            name="hinge side reveal stays narrow",
        )
        ctx.expect_gap(
            support,
            gate_leaf,
            axis="x",
            positive_elem=latch_post,
            negative_elem=right_stile,
            min_gap=0.010,
            max_gap=0.035,
            name="latch side reveal stays narrow",
        )
        ctx.expect_contact(
            rotary_latch,
            gate_leaf,
            elem_a=latch_backplate,
            elem_b=right_stile,
            contact_tol=0.001,
            name="latch backplate mounts to the free stile",
        )
        ctx.expect_gap(
            support,
            rotary_latch,
            axis="x",
            positive_elem=strike_block,
            negative_elem=latch_cam,
            min_gap=0.002,
            max_gap=0.010,
            name="closed latch cam stops just shy of the strike",
        )
        ctx.expect_overlap(
            rotary_latch,
            support,
            axes="yz",
            elem_a=latch_cam,
            elem_b=strike_block,
            min_overlap=0.010,
            name="latch cam lines up with the strike block",
        )

    closed_leaf_aabb = ctx.part_world_aabb(gate_leaf)
    with ctx.pose({gate_hinge: 1.10}):
        opened_leaf_aabb = ctx.part_world_aabb(gate_leaf)
    ctx.check(
        "gate swings open toward positive y",
        closed_leaf_aabb is not None
        and opened_leaf_aabb is not None
        and opened_leaf_aabb[1][1] > closed_leaf_aabb[1][1] + 0.50,
        details=f"closed={closed_leaf_aabb}, opened={opened_leaf_aabb}",
    )

    closed_cam_aabb = ctx.part_element_world_aabb(rotary_latch, elem=latch_cam)
    with ctx.pose({latch_joint: 0.85}):
        rotated_cam_aabb = ctx.part_element_world_aabb(rotary_latch, elem=latch_cam)
    ctx.check(
        "rotary latch lifts upward when opened",
        closed_cam_aabb is not None
        and rotated_cam_aabb is not None
        and rotated_cam_aabb[1][2] > closed_cam_aabb[1][2] + 0.015,
        details=f"closed={closed_cam_aabb}, opened={rotated_cam_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
