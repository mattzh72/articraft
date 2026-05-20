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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shell_ring(width: float, depth: float, corner_radius: float, wall: float, height: float):
    outer = rounded_rect_profile(width, depth, corner_radius, corner_segments=8)
    inner = rounded_rect_profile(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        max(corner_radius - wall, 0.0005),
        corner_segments=8,
    )
    return ExtrudeWithHolesGeometry(outer, [inner], height, center=True).translate(0.0, 0.0, height * 0.5)


def _solid_panel(width: float, depth: float, corner_radius: float, thickness: float):
    profile = rounded_rect_profile(width, depth, corner_radius, corner_segments=8)
    return ExtrudeGeometry(profile, thickness, center=True).translate(0.0, 0.0, thickness * 0.5)


def _tube_shell(radius_outer: float, radius_inner: float, length: float):
    outer = [(radius_outer, -length * 0.5), (radius_outer, length * 0.5)]
    inner = [(radius_inner, -length * 0.5), (radius_inner, length * 0.5)]
    return LatheGeometry.from_shell_profiles(outer, inner, segments=48).rotate_y(pi / 2.0)


def _yz_section(x: float, width_y: float, height_z: float, radius: float):
    return [
        (x, y, z)
        for y, z in rounded_rect_profile(width_y, height_z, radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_watch_winder_box")

    painted_metal = model.material("painted_metal", rgba=(0.23, 0.24, 0.27, 1.0))
    polymer = model.material("polymer", rgba=(0.08, 0.09, 0.10, 1.0))
    elastomer = model.material("elastomer", rgba=(0.18, 0.18, 0.19, 1.0))
    bearing_metal = model.material("bearing_metal", rgba=(0.69, 0.71, 0.74, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.55, 0.57, 0.60, 1.0))

    base_w = 0.178
    base_d = 0.154
    base_h = 0.078
    base_corner = 0.018
    base_wall = 0.0035
    floor_t = 0.005

    lid_clear = 0.0012
    lid_w = base_w + 2.0 * lid_clear
    lid_d = base_d + 2.0 * lid_clear
    lid_corner = base_corner + lid_clear
    lid_total_h = 0.060
    lid_top_t = 0.004
    lid_wall = 0.0032
    lid_lower_z = -0.018
    lid_axis_to_rear = 0.0035

    hinge_radius = 0.006
    hinge_y = -(base_d * 0.5) - 0.0045
    hinge_z = base_h + hinge_radius
    hinge_side_x = 0.052
    base_knuckle_len = 0.028
    lid_knuckle_len = 0.064

    cradle_axis_z = 0.046
    cradle_support_x = 0.060
    bearing_outer_r = 0.009
    bearing_inner_r = 0.0056
    bearing_len = 0.010

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((base_w, base_d, base_h)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, base_h * 0.5)),
    )

    base.visual(
        _mesh(
            "base_outer_shell",
            _shell_ring(base_w, base_d, base_corner, base_wall, base_h - floor_t).translate(0.0, 0.0, floor_t),
        ),
        material=painted_metal,
        name="outer_shell",
    )
    base.visual(
        _mesh("base_floor_panel", _solid_panel(base_w, base_d, base_corner, floor_t)),
        material=painted_metal,
        name="floor_panel",
    )
    base.visual(
        _mesh("base_inner_pad", _solid_panel(0.146, 0.122, 0.014, 0.0022)),
        origin=Origin(xyz=(0.0, 0.0, floor_t)),
        material=polymer,
        name="inner_pad",
    )
    base.visual(
        Box((0.112, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, -0.055, 0.010)),
        material=polymer,
        name="motor_pod",
    )
    base.visual(
        Box((0.014, 0.026, 0.052)),
        origin=Origin(xyz=(-0.075, 0.0, 0.026)),
        material=polymer,
        name="left_support_tower",
    )
    base.visual(
        Box((0.014, 0.026, 0.052)),
        origin=Origin(xyz=(0.075, 0.0, 0.026)),
        material=polymer,
        name="right_support_tower",
    )
    base.visual(
        Box((0.009, 0.020, 0.018)),
        origin=Origin(xyz=(-0.0695, 0.0, cradle_axis_z)),
        material=polymer,
        name="left_bearing_boss",
    )
    base.visual(
        Box((0.009, 0.020, 0.018)),
        origin=Origin(xyz=(0.0695, 0.0, cradle_axis_z)),
        material=polymer,
        name="right_bearing_boss",
    )
    base.visual(
        _mesh("left_bearing_collar", _tube_shell(bearing_outer_r, bearing_inner_r, bearing_len)),
        origin=Origin(xyz=(-cradle_support_x, 0.0, cradle_axis_z)),
        material=bearing_metal,
        name="left_bearing",
    )
    base.visual(
        _mesh("right_bearing_collar", _tube_shell(bearing_outer_r, bearing_inner_r, bearing_len)),
        origin=Origin(xyz=(cradle_support_x, 0.0, cradle_axis_z)),
        material=bearing_metal,
        name="right_bearing",
    )
    base.visual(
        Box((0.118, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.074, 0.078)),
        material=painted_metal,
        name="hinge_bridge",
    )
    base.visual(
        Box((0.020, 0.015, 0.012)),
        origin=Origin(xyz=(-hinge_side_x, -0.0745, 0.078)),
        material=painted_metal,
        name="left_hinge_bracket",
    )
    base.visual(
        Box((0.020, 0.015, 0.012)),
        origin=Origin(xyz=(hinge_side_x, -0.0745, 0.078)),
        material=painted_metal,
        name="right_hinge_bracket",
    )
    base.visual(
        Cylinder(radius=hinge_radius, length=base_knuckle_len),
        origin=Origin(xyz=(-hinge_side_x, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="left_hinge_barrel",
    )
    base.visual(
        Cylinder(radius=hinge_radius, length=base_knuckle_len),
        origin=Origin(xyz=(hinge_side_x, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="right_hinge_barrel",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_total_h)),
        mass=1.2,
        origin=Origin(xyz=(0.0, lid_axis_to_rear + lid_d * 0.5, lid_lower_z + lid_total_h * 0.5)),
    )

    lid_shell_y = lid_axis_to_rear + lid_d * 0.5
    lid_shell_z = lid_lower_z + (lid_total_h - lid_top_t) * 0.5
    lid.visual(
        _mesh("lid_shell", _shell_ring(lid_w, lid_d, lid_corner, lid_wall, lid_total_h - lid_top_t)),
        origin=Origin(xyz=(0.0, lid_shell_y, lid_shell_z)),
        material=painted_metal,
        name="lid_shell",
    )
    lid.visual(
        _mesh("lid_top_skin", _solid_panel(lid_w, lid_d, lid_corner, lid_top_t)),
        origin=Origin(xyz=(0.0, lid_shell_y, lid_lower_z + lid_total_h - lid_top_t)),
        material=painted_metal,
        name="top_skin",
    )
    lid.visual(
        _mesh("lid_inner_panel", _solid_panel(lid_w - 2.0 * lid_wall, lid_d - 2.0 * lid_wall, lid_corner - lid_wall, 0.0022)),
        origin=Origin(xyz=(0.0, lid_shell_y, 0.032)),
        material=polymer,
        name="inner_panel",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=lid_knuckle_len),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.012, 0.012, 0.010)),
        origin=Origin(xyz=(-0.022, 0.008, 0.009)),
        material=satin_metal,
        name="left_hinge_cheek",
    )
    lid.visual(
        Box((0.012, 0.012, 0.010)),
        origin=Origin(xyz=(0.022, 0.008, 0.009)),
        material=satin_metal,
        name="right_hinge_cheek",
    )

    cradle = model.part("cradle")
    cradle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.112),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    cushion_geom = section_loft(
        [
            _yz_section(-0.028, 0.044, 0.036, 0.010),
            _yz_section(-0.014, 0.054, 0.046, 0.013),
            _yz_section(0.0, 0.060, 0.050, 0.015),
            _yz_section(0.014, 0.054, 0.046, 0.013),
            _yz_section(0.028, 0.044, 0.036, 0.010),
        ]
    )

    cradle.visual(
        Cylinder(radius=0.0042, length=0.112),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="axle",
    )
    cradle.visual(
        Cylinder(radius=0.0049, length=0.010),
        origin=Origin(xyz=(-cradle_support_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="left_trunnion",
    )
    cradle.visual(
        Cylinder(radius=0.0049, length=0.010),
        origin=Origin(xyz=(cradle_support_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="right_trunnion",
    )
    cradle.visual(
        _mesh("cradle_hoop", _tube_shell(0.040, 0.033, 0.074)),
        material=polymer,
        name="rotor_hoop",
    )
    cradle.visual(
        Cylinder(radius=0.030, length=0.076),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=polymer,
        name="rotor_core",
    )
    cradle.visual(
        Cylinder(radius=0.043, length=0.006),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=polymer,
        name="left_flange",
    )
    cradle.visual(
        Cylinder(radius=0.043, length=0.006),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=polymer,
        name="right_flange",
    )
    cradle.visual(
        _mesh("cradle_cushion", cushion_geom),
        material=elastomer,
        name="cushion",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.6, lower=0.0, upper=1.95),
    )
    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, cradle_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    cradle_spin = object_model.get_articulation("cradle_spin")

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

    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.145, name="lid_covers_base_plan")
    ctx.expect_overlap(cradle, base, axes="xy", min_overlap=0.070, name="cradle_stays_within_base_plan")
    ctx.expect_overlap(
        cradle,
        base,
        axes="yz",
        elem_a="left_trunnion",
        elem_b="left_bearing",
        min_overlap=0.009,
        name="left_trunnion_aligned_to_bearing",
    )
    ctx.expect_overlap(
        cradle,
        base,
        axes="yz",
        elem_a="right_trunnion",
        elem_b="right_bearing",
        min_overlap=0.009,
        name="right_trunnion_aligned_to_bearing",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="yz",
        elem_a="hinge_barrel",
        elem_b="left_hinge_barrel",
        min_overlap=0.010,
        name="lid_hinge_axis_matches_left_knuckle",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="yz",
        elem_a="hinge_barrel",
        elem_b="right_hinge_barrel",
        min_overlap=0.010,
        name="lid_hinge_axis_matches_right_knuckle",
    )

    closed_top = ctx.part_element_world_aabb(lid, elem="top_skin")
    with ctx.pose({lid_hinge: 1.45}):
        open_top = ctx.part_element_world_aabb(lid, elem="top_skin")
        ctx.fail_if_parts_overlap_in_current_pose(name="open_lid_clearance")
    lid_opens = (
        closed_top is not None
        and open_top is not None
        and open_top[1][2] > closed_top[1][2] + 0.015
    )
    ctx.check(
        "lid_swings_upward",
        lid_opens,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )

    with ctx.pose({cradle_spin: pi * 0.5}):
        ctx.fail_if_parts_overlap_in_current_pose(name="cradle_clears_box_at_quarter_turn")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
