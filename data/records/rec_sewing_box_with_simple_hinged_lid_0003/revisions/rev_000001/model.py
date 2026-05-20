from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def _add_box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_z_cylinder(part, name, radius, length, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_x_cylinder(part, name, radius, length, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_sewing_box", assets=ASSETS)

    body_molded = model.material("body_molded", rgba=(0.27, 0.31, 0.20, 1.0))
    body_reinforced = model.material("body_reinforced", rgba=(0.20, 0.24, 0.15, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.34, 0.37, 0.40, 1.0))
    zinc = model.material("zinc_fastener", rgba=(0.71, 0.73, 0.76, 1.0))

    body_w = 0.320
    body_d = 0.200
    floor_t = 0.014
    wall_t = 0.012
    wall_h = 0.088
    rim_h = 0.016
    body_top = floor_t + wall_h + rim_h

    hinge_radius = 0.006
    hinge_axis_y = -(body_d / 2.0 + hinge_radius)
    hinge_axis_z = body_top + hinge_radius

    base = model.part("base")
    _add_box(base, "body_floor", (body_w, body_d, floor_t), (0.0, 0.0, floor_t / 2.0), body_molded)
    _add_box(
        base,
        "front_wall",
        (body_w, wall_t, wall_h),
        (0.0, body_d / 2.0 - wall_t / 2.0, floor_t + wall_h / 2.0),
        body_molded,
    )
    _add_box(
        base,
        "rear_wall",
        (body_w, wall_t, wall_h),
        (0.0, -(body_d / 2.0 - wall_t / 2.0), floor_t + wall_h / 2.0),
        body_molded,
    )
    _add_box(
        base,
        "left_wall",
        (wall_t, body_d - 2.0 * wall_t, wall_h),
        (-(body_w / 2.0 - wall_t / 2.0), 0.0, floor_t + wall_h / 2.0),
        body_molded,
    )
    _add_box(
        base,
        "right_wall",
        (wall_t, body_d - 2.0 * wall_t, wall_h),
        ((body_w / 2.0 - wall_t / 2.0), 0.0, floor_t + wall_h / 2.0),
        body_molded,
    )

    rim_z = floor_t + wall_h + rim_h / 2.0
    _add_box(base, "front_rim", (body_w, 0.018, rim_h), (0.0, 0.091, rim_z), body_reinforced)
    _add_box(base, "left_rim", (0.018, 0.164, rim_h), (-0.151, 0.0, rim_z), body_reinforced)
    _add_box(base, "right_rim", (0.018, 0.164, rim_h), (0.151, 0.0, rim_z), body_reinforced)
    _add_box(base, "rear_rim_left", (0.080, 0.018, rim_h), (-0.120, -0.091, rim_z), body_reinforced)
    _add_box(base, "rear_rim_right", (0.080, 0.018, rim_h), (0.120, -0.091, rim_z), body_reinforced)

    _add_box(base, "hinge_support_left", (0.052, 0.019, 0.026), (-0.108, -0.1085, 0.106), body_reinforced)
    _add_box(base, "hinge_support_center", (0.044, 0.017, 0.022), (0.0, -0.1075, 0.108), body_reinforced)
    _add_box(base, "hinge_support_right", (0.052, 0.019, 0.026), (0.108, -0.1085, 0.106), body_reinforced)
    _add_box(
        base,
        "hinge_gusset_left",
        (0.052, 0.010, 0.030),
        (-0.108, -0.102, 0.096),
        body_reinforced,
        rpy=(0.58, 0.0, 0.0),
    )
    _add_box(
        base,
        "hinge_gusset_right",
        (0.052, 0.010, 0.030),
        (0.108, -0.102, 0.096),
        body_reinforced,
        rpy=(0.58, 0.0, 0.0),
    )

    _add_x_cylinder(base, "base_barrel_left", hinge_radius, 0.055, (-0.108, hinge_axis_y, hinge_axis_z), painted_steel)
    _add_x_cylinder(base, "base_barrel_center", hinge_radius, 0.055, (0.0, hinge_axis_y, hinge_axis_z), painted_steel)
    _add_x_cylinder(base, "base_barrel_right", hinge_radius, 0.055, (0.108, hinge_axis_y, hinge_axis_z), painted_steel)

    for name, x, y in (
        ("corner_guard_front_left", -0.149, 0.089),
        ("corner_guard_front_right", 0.149, 0.089),
        ("corner_guard_rear_left", -0.149, -0.089),
        ("corner_guard_rear_right", 0.149, -0.089),
    ):
        _add_box(base, name, (0.022, 0.022, 0.072), (x, y, 0.050), body_reinforced)

    _add_box(base, "organizer_divider_center", (0.010, 0.150, 0.040), (0.0, 0.005, 0.034), body_reinforced)
    _add_box(base, "organizer_divider_left", (0.108, 0.010, 0.032), (-0.054, -0.022, 0.030), body_reinforced)
    _add_box(base, "organizer_divider_right", (0.108, 0.010, 0.032), (0.054, 0.040, 0.030), body_reinforced)

    _add_box(base, "left_skid", (0.100, 0.030, 0.010), (-0.080, 0.0, -0.005), body_reinforced)
    _add_box(base, "right_skid", (0.100, 0.030, 0.010), (0.080, 0.0, -0.005), body_reinforced)

    for name, x, y in (
        ("base_fastener_front_left", -0.149, 0.089),
        ("base_fastener_front_right", 0.149, 0.089),
        ("base_fastener_rear_left", -0.149, -0.089),
        ("base_fastener_rear_right", 0.149, -0.089),
    ):
        _add_z_cylinder(base, name, 0.004, 0.004, (x, y, 0.088), zinc)

    _add_z_cylinder(base, "support_fastener_left", 0.0045, 0.005, (-0.108, -0.1085, 0.1215), zinc)
    _add_z_cylinder(base, "support_fastener_right", 0.0045, 0.005, (0.108, -0.1085, 0.1215), zinc)

    lid = model.part("lid")
    lid_skin_w = 0.344
    lid_skin_d = 0.219
    lid_skin_t = 0.008
    lid_skin_center_y = lid_skin_d / 2.0 + 0.008
    lid_skin_center_z = body_top + lid_skin_t / 2.0 - hinge_axis_z
    _add_box(lid, "lid_skin", (lid_skin_w, lid_skin_d, lid_skin_t), (0.0, lid_skin_center_y, lid_skin_center_z), body_molded)

    _add_box(lid, "top_frame_front", (0.328, 0.020, 0.006), (0.0, 0.207, 0.005), body_reinforced)
    _add_box(lid, "top_frame_left", (0.018, 0.165, 0.006), (-0.161, 0.1145, 0.005), body_reinforced)
    _add_box(lid, "top_frame_right", (0.018, 0.165, 0.006), (0.161, 0.1145, 0.005), body_reinforced)
    _add_box(lid, "top_frame_rear_left", (0.080, 0.016, 0.006), (-0.108, 0.018, 0.005), body_reinforced)
    _add_box(lid, "top_frame_rear_right", (0.080, 0.016, 0.006), (0.108, 0.018, 0.005), body_reinforced)

    _add_box(lid, "lid_side_lip_left", (0.010, 0.184, 0.014), (-0.166, 0.108, -0.013), body_reinforced)
    _add_box(lid, "lid_side_lip_right", (0.010, 0.184, 0.014), (0.166, 0.108, -0.013), body_reinforced)
    _add_box(lid, "lid_front_lip", (0.312, 0.010, 0.014), (0.0, 0.212, -0.013), body_reinforced)
    _add_box(lid, "rear_hinge_strap", (0.132, 0.016, 0.012), (0.0, 0.012, -0.012), painted_steel)
    _add_box(lid, "lid_center_rib", (0.024, 0.120, 0.012), (0.0, 0.128, -0.012), body_reinforced)
    _add_x_cylinder(lid, "front_pull", 0.006, 0.094, (0.0, 0.226, -0.009), painted_steel)
    _add_box(lid, "front_pull_bracket_left", (0.014, 0.012, 0.018), (-0.042, 0.221, -0.010), painted_steel)
    _add_box(lid, "front_pull_bracket_right", (0.014, 0.012, 0.018), (0.042, 0.221, -0.010), painted_steel)
    _add_box(lid, "hinge_leaf_left", (0.026, 0.014, 0.014), (-0.054, 0.008, -0.007), painted_steel)
    _add_box(lid, "hinge_leaf_right", (0.026, 0.014, 0.014), (0.054, 0.008, -0.007), painted_steel)

    _add_x_cylinder(lid, "lid_barrel_left", hinge_radius, 0.045, (-0.054, 0.0, 0.0), painted_steel)
    _add_x_cylinder(lid, "lid_barrel_right", hinge_radius, 0.045, (0.054, 0.0, 0.0), painted_steel)

    for name, x, y in (
        ("lid_fastener_front_left", -0.120, 0.207),
        ("lid_fastener_front_right", 0.120, 0.207),
        ("lid_fastener_rear_left", -0.108, 0.018),
        ("lid_fastener_rear_right", 0.108, 0.018),
    ):
        _add_z_cylinder(lid, name, 0.004, 0.004, (x, y, 0.010), zinc)

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    def must_get_visual(part, visual_name):
        try:
            return part.get_visual(visual_name)
        except Exception:
            return None

    key_visuals = {
        "body_floor": must_get_visual(base, "body_floor"),
        "front_wall": must_get_visual(base, "front_wall"),
        "front_rim": must_get_visual(base, "front_rim"),
        "hinge_gusset_left": must_get_visual(base, "hinge_gusset_left"),
        "hinge_gusset_right": must_get_visual(base, "hinge_gusset_right"),
        "organizer_divider_center": must_get_visual(base, "organizer_divider_center"),
        "organizer_divider_left": must_get_visual(base, "organizer_divider_left"),
        "organizer_divider_right": must_get_visual(base, "organizer_divider_right"),
        "base_barrel_left": must_get_visual(base, "base_barrel_left"),
        "base_barrel_center": must_get_visual(base, "base_barrel_center"),
        "base_barrel_right": must_get_visual(base, "base_barrel_right"),
        "lid_skin": must_get_visual(lid, "lid_skin"),
        "lid_front_lip": must_get_visual(lid, "lid_front_lip"),
        "front_pull": must_get_visual(lid, "front_pull"),
        "lid_barrel_left": must_get_visual(lid, "lid_barrel_left"),
        "lid_barrel_right": must_get_visual(lid, "lid_barrel_right"),
    }

    for visual_name, visual in key_visuals.items():
        ctx.check(f"{visual_name}_present", visual is not None, details=f"Missing visual '{visual_name}'.")

    def aabb_dims(aabb):
        (min_pt, max_pt) = aabb
        return (
            max_pt[0] - min_pt[0],
            max_pt[1] - min_pt[1],
            max_pt[2] - min_pt[2],
        )

    def aabb_center(aabb):
        (min_pt, max_pt) = aabb
        return (
            0.5 * (min_pt[0] + max_pt[0]),
            0.5 * (min_pt[1] + max_pt[1]),
            0.5 * (min_pt[2] + max_pt[2]),
        )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.19, name="lid_covers_body_footprint")
    ctx.expect_contact(
        lid,
        base,
        elem_a=key_visuals["lid_skin"],
        elem_b=key_visuals["front_rim"],
        name="lid_skin_seats_on_top_rim",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem=key_visuals["lid_front_lip"],
        negative_elem=key_visuals["front_wall"],
        name="front_lip_clears_front_wall_without_floating",
    )

    ctx.expect_overlap(
        lid,
        base,
        axes="yz",
        min_overlap=0.010,
        elem_a=key_visuals["lid_barrel_left"],
        elem_b=key_visuals["base_barrel_left"],
        name="left_lid_barrel_shares_hinge_axis_with_left_base_barrel",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="yz",
        min_overlap=0.010,
        elem_a=key_visuals["lid_barrel_left"],
        elem_b=key_visuals["base_barrel_center"],
        name="left_lid_barrel_shares_hinge_axis_with_center_base_barrel",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="yz",
        min_overlap=0.010,
        elem_a=key_visuals["lid_barrel_right"],
        elem_b=key_visuals["base_barrel_center"],
        name="right_lid_barrel_shares_hinge_axis_with_center_base_barrel",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="yz",
        min_overlap=0.010,
        elem_a=key_visuals["lid_barrel_right"],
        elem_b=key_visuals["base_barrel_right"],
        name="right_lid_barrel_shares_hinge_axis_with_right_base_barrel",
    )

    ctx.expect_gap(
        lid,
        base,
        axis="x",
        min_gap=0.002,
        max_gap=0.008,
        positive_elem=key_visuals["lid_barrel_left"],
        negative_elem=key_visuals["base_barrel_left"],
        name="left_hinge_segment_gap_from_left_base_barrel",
    )
    ctx.expect_gap(
        base,
        lid,
        axis="x",
        min_gap=0.002,
        max_gap=0.008,
        positive_elem=key_visuals["base_barrel_center"],
        negative_elem=key_visuals["lid_barrel_left"],
        name="left_hinge_segment_gap_from_center_base_barrel",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="x",
        min_gap=0.002,
        max_gap=0.008,
        positive_elem=key_visuals["lid_barrel_right"],
        negative_elem=key_visuals["base_barrel_center"],
        name="right_hinge_segment_gap_from_center_base_barrel",
    )
    ctx.expect_gap(
        base,
        lid,
        axis="x",
        min_gap=0.002,
        max_gap=0.008,
        positive_elem=key_visuals["base_barrel_right"],
        negative_elem=key_visuals["lid_barrel_right"],
        name="right_hinge_segment_gap_from_right_base_barrel",
    )

    floor_aabb = ctx.part_element_world_aabb(base, elem=key_visuals["body_floor"])
    lid_skin_aabb = ctx.part_element_world_aabb(lid, elem=key_visuals["lid_skin"])
    if floor_aabb is not None and lid_skin_aabb is not None:
        floor_dims = aabb_dims(floor_aabb)
        lid_dims = aabb_dims(lid_skin_aabb)
        ctx.check(
            "compact_body_plan_size",
            0.30 <= floor_dims[0] <= 0.34 and 0.19 <= floor_dims[1] <= 0.21,
            details=f"Unexpected floor plan dimensions: {floor_dims}",
        )
        ctx.check(
            "serviceable_lid_overhang",
            0.018 <= (lid_dims[0] - floor_dims[0]) <= 0.030 and 0.016 <= (lid_dims[1] - floor_dims[1]) <= 0.025,
            details=f"Unexpected lid overhang: floor={floor_dims}, lid={lid_dims}",
        )

    organizer_center_aabb = ctx.part_element_world_aabb(base, elem=key_visuals["organizer_divider_center"])
    organizer_left_aabb = ctx.part_element_world_aabb(base, elem=key_visuals["organizer_divider_left"])
    organizer_right_aabb = ctx.part_element_world_aabb(base, elem=key_visuals["organizer_divider_right"])
    if all(aabb is not None for aabb in (organizer_center_aabb, organizer_left_aabb, organizer_right_aabb)):
        center_dims = aabb_dims(organizer_center_aabb)
        left_dims = aabb_dims(organizer_left_aabb)
        right_dims = aabb_dims(organizer_right_aabb)
        ctx.check(
            "sewing_organizer_dividers_are_present_and_low",
            center_dims[1] >= 0.14
            and 0.028 <= left_dims[2] <= 0.036
            and 0.028 <= right_dims[2] <= 0.036
            and organizer_center_aabb[1][2] < 0.060,
            details=(
                f"Unexpected organizer divider sizing: center={center_dims}, "
                f"left={left_dims}, right={right_dims}"
            ),
        )

    front_pull_aabb = ctx.part_element_world_aabb(lid, elem=key_visuals["front_pull"])
    if front_pull_aabb is not None:
        front_pull_dims = aabb_dims(front_pull_aabb)
        ctx.check(
            "front_pull_reads_as_bar_handle",
            front_pull_dims[0] >= 0.090 and front_pull_dims[1] <= 0.016 and front_pull_dims[2] <= 0.016,
            details=f"Front pull no longer reads as a slender bar: {front_pull_dims}",
        )

    left_base_barrel = ctx.part_element_world_aabb(base, elem=key_visuals["base_barrel_left"])
    center_base_barrel = ctx.part_element_world_aabb(base, elem=key_visuals["base_barrel_center"])
    right_base_barrel = ctx.part_element_world_aabb(base, elem=key_visuals["base_barrel_right"])
    left_lid_barrel = ctx.part_element_world_aabb(lid, elem=key_visuals["lid_barrel_left"])
    right_lid_barrel = ctx.part_element_world_aabb(lid, elem=key_visuals["lid_barrel_right"])
    if all(aabb is not None for aabb in (left_base_barrel, center_base_barrel, right_base_barrel, left_lid_barrel, right_lid_barrel)):
        ctx.check(
            "hinge_knuckles_interleave_in_x",
            left_base_barrel[1][0] < left_lid_barrel[0][0] < left_lid_barrel[1][0] < center_base_barrel[0][0]
            and center_base_barrel[1][0] < right_lid_barrel[0][0] < right_lid_barrel[1][0] < right_base_barrel[0][0],
            details="Hinge barrels do not interleave cleanly along the pivot axis.",
        )
        left_axis = aabb_center(left_lid_barrel)
        right_axis = aabb_center(right_lid_barrel)
        base_axis_left = aabb_center(left_base_barrel)
        base_axis_center = aabb_center(center_base_barrel)
        base_axis_right = aabb_center(right_base_barrel)
        ctx.check(
            "hinge_axis_centers_are_coaxial",
            abs(left_axis[1] - base_axis_left[1]) <= 0.001
            and abs(left_axis[2] - base_axis_center[2]) <= 0.001
            and abs(right_axis[1] - base_axis_right[1]) <= 0.001
            and abs(right_axis[2] - base_axis_center[2]) <= 0.001,
            details="Lid and base hinge barrels are not centered on a common hinge axis.",
        )

    closed_front_pull = ctx.part_element_world_aabb(lid, elem=key_visuals["front_pull"])
    with ctx.pose({lid_hinge: 1.15}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.120,
            positive_elem=key_visuals["front_pull"],
            negative_elem=key_visuals["front_wall"],
            name="front_pull_lifts_clear_when_open",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="yz",
            min_overlap=0.010,
            elem_a=key_visuals["lid_barrel_left"],
            elem_b=key_visuals["base_barrel_left"],
            name="left_hinge_axis_stays_registered_when_open",
        )
        open_front_pull = ctx.part_element_world_aabb(lid, elem=key_visuals["front_pull"])
        if closed_front_pull is not None and open_front_pull is not None:
            ctx.check(
                "lid_front_rotates_upward",
                open_front_pull[0][2] > closed_front_pull[1][2] + 0.120,
                details=f"Front pull did not rise enough: closed={closed_front_pull}, open={open_front_pull}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
