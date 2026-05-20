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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_vent_tower", assets=ASSETS)

    body_paint = model.material("body_paint", rgba=(0.34, 0.38, 0.40, 1.0))
    flap_paint = model.material("flap_paint", rgba=(0.30, 0.33, 0.35, 1.0))
    zinc = model.material("zinc", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    flange_w = 0.48
    flange_d = 0.34
    flange_t = 0.012
    curb_w = 0.36
    curb_d = 0.24
    curb_t = 0.018
    tower_w = 0.34
    tower_d = 0.22
    wall_t = 0.012
    lower_top_z = 0.152
    rear_top_z = 0.182
    tower_base_z = flange_t + curb_t
    lower_wall_h = lower_top_z - tower_base_z
    rear_raise_h = rear_top_z - lower_top_z

    front_face_y = -(tower_d / 2.0 - wall_t / 2.0)
    rear_face_y = tower_d / 2.0 - wall_t / 2.0
    side_face_x = tower_w / 2.0 - wall_t / 2.0

    opening_w = 0.24
    opening_bottom_z = 0.056
    opening_top_z = 0.138
    opening_h = opening_top_z - opening_bottom_z
    jamb_w = (tower_w - opening_w) / 2.0
    frame_span = opening_w + 2.0 * wall_t
    throat_d = 0.055

    hinge_axis_y = 0.082
    hinge_axis_z = 0.194
    hinge_radius = 0.010
    closed_pitch = math.radians(7.0)

    tower = model.part("vent_tower")
    tower.visual(
        Box((flange_w, flange_d, flange_t)),
        origin=Origin(xyz=(0.0, 0.0, flange_t / 2.0)),
        material=body_paint,
        name="roof_flange",
    )
    tower.visual(
        Box((curb_w, curb_d, curb_t)),
        origin=Origin(xyz=(0.0, 0.0, flange_t + curb_t / 2.0)),
        material=body_paint,
        name="mounting_curb",
    )

    for side, sign in (("left", -1.0), ("right", 1.0)):
        tower.visual(
            Box((wall_t, tower_d, lower_wall_h)),
            origin=Origin(
                xyz=(sign * side_face_x, 0.0, tower_base_z + lower_wall_h / 2.0)
            ),
            material=body_paint,
            name=f"{side}_lower_wall",
        )
        tower.visual(
            Box((wall_t, 0.05, rear_raise_h)),
            origin=Origin(
                xyz=(sign * side_face_x, 0.085, lower_top_z + rear_raise_h / 2.0)
            ),
            material=body_paint,
            name=f"{side}_rear_cheek",
        )
        tower.visual(
            Box((0.018, 0.055, 0.090)),
            origin=Origin(xyz=(sign * 0.175, -0.040, 0.075)),
            material=body_paint,
            name=f"{side}_front_rib",
        )
        tower.visual(
            Box((0.018, 0.055, 0.100)),
            origin=Origin(xyz=(sign * 0.175, 0.030, 0.080)),
            material=body_paint,
            name=f"{side}_mid_rib",
        )

    tower.visual(
        Box((tower_w - 2.0 * wall_t, wall_t, rear_top_z - tower_base_z)),
        origin=Origin(
            xyz=(0.0, rear_face_y, tower_base_z + (rear_top_z - tower_base_z) / 2.0)
        ),
        material=body_paint,
        name="rear_wall",
    )

    jamb_center_x = opening_w / 2.0 + jamb_w / 2.0
    tower.visual(
        Box((jamb_w, wall_t, opening_h)),
        origin=Origin(
            xyz=(-jamb_center_x, front_face_y, opening_bottom_z + opening_h / 2.0)
        ),
        material=body_paint,
        name="outlet_jamb_left",
    )
    tower.visual(
        Box((jamb_w, wall_t, opening_h)),
        origin=Origin(
            xyz=(jamb_center_x, front_face_y, opening_bottom_z + opening_h / 2.0)
        ),
        material=body_paint,
        name="outlet_jamb_right",
    )
    tower.visual(
        Box((frame_span, wall_t, opening_bottom_z - tower_base_z)),
        origin=Origin(
            xyz=(0.0, front_face_y, tower_base_z + (opening_bottom_z - tower_base_z) / 2.0)
        ),
        material=body_paint,
        name="outlet_sill",
    )
    tower.visual(
        Box((frame_span, wall_t, lower_top_z - opening_top_z)),
        origin=Origin(
            xyz=(0.0, front_face_y, opening_top_z + (lower_top_z - opening_top_z) / 2.0)
        ),
        material=body_paint,
        name="outlet_lintel",
    )

    tower.visual(
        Box((opening_w, throat_d, wall_t)),
        origin=Origin(xyz=(0.0, -0.078, opening_top_z + wall_t / 2.0)),
        material=rubber,
        name="opening_top_liner",
    )
    tower.visual(
        Box((opening_w, throat_d, wall_t)),
        origin=Origin(xyz=(0.0, -0.078, opening_bottom_z - wall_t / 2.0)),
        material=body_paint,
        name="opening_bottom_liner",
    )
    tower.visual(
        Box((wall_t, throat_d, opening_h)),
        origin=Origin(
            xyz=(-opening_w / 2.0 - wall_t / 2.0, -0.078, opening_bottom_z + opening_h / 2.0)
        ),
        material=body_paint,
        name="opening_left_liner",
    )
    tower.visual(
        Box((wall_t, throat_d, opening_h)),
        origin=Origin(
            xyz=(opening_w / 2.0 + wall_t / 2.0, -0.078, opening_bottom_z + opening_h / 2.0)
        ),
        material=body_paint,
        name="opening_right_liner",
    )

    tower.visual(
        Box((0.14, 0.022, 0.040)),
        origin=Origin(xyz=(0.0, 0.093, 0.168)),
        material=body_paint,
        name="hinge_bridge",
    )
    tower.visual(
        Box((0.030, 0.040, 0.060)),
        origin=Origin(xyz=(-0.065, 0.082, 0.152)),
        material=body_paint,
        name="left_hinge_gusset",
    )
    tower.visual(
        Box((0.030, 0.040, 0.060)),
        origin=Origin(xyz=(0.065, 0.082, 0.152)),
        material=body_paint,
        name="right_hinge_gusset",
    )
    tower.visual(
        Cylinder(radius=hinge_radius, length=0.16),
        origin=Origin(
            xyz=(0.0, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=zinc,
        name="hinge_center_barrel",
    )

    flange_bolts = [
        (-0.185, -0.125),
        (0.185, -0.125),
        (-0.185, 0.125),
        (0.185, 0.125),
        (-0.080, -0.145),
        (0.080, -0.145),
        (-0.080, 0.145),
        (0.080, 0.145),
    ]
    for idx, (x_pos, y_pos) in enumerate(flange_bolts, start=1):
        tower.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x_pos, y_pos, flange_t + 0.002)),
            material=zinc,
            name=f"flange_bolt_{idx}",
        )

    frame_bolts = [
        (-0.145, -0.112, 0.072),
        (-0.145, -0.112, 0.122),
        (0.145, -0.112, 0.072),
        (0.145, -0.112, 0.122),
        (-0.090, -0.112, 0.145),
        (0.090, -0.112, 0.145),
    ]
    for idx, (x_pos, y_pos, z_pos) in enumerate(frame_bolts, start=1):
        tower.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(
                xyz=(x_pos, y_pos, z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc,
            name=f"frame_bolt_{idx}",
        )

    bridge_bolts = [(-0.045, 0.104, 0.174), (0.045, 0.104, 0.174)]
    for idx, (x_pos, y_pos, z_pos) in enumerate(bridge_bolts, start=1):
        tower.visual(
            Cylinder(radius=0.005, length=0.004),
            origin=Origin(
                xyz=(x_pos, y_pos, z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc,
            name=f"bridge_bolt_{idx}",
        )

    tower.inertial = Inertial.from_geometry(
        Box((flange_w, flange_d, 0.205)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
    )

    flap = model.part("weather_flap")
    flap.visual(
        Box((0.38, 0.205, 0.010)),
        origin=Origin(xyz=(0.0, -0.121, -0.004), rpy=(closed_pitch, 0.0, 0.0)),
        material=flap_paint,
        name="cap_panel",
    )
    flap.visual(
        Box((0.012, 0.185, 0.032)),
        origin=Origin(xyz=(-0.184, -0.120, -0.019), rpy=(closed_pitch, 0.0, 0.0)),
        material=flap_paint,
        name="left_side_skirt",
    )
    flap.visual(
        Box((0.012, 0.185, 0.032)),
        origin=Origin(xyz=(0.184, -0.120, -0.019), rpy=(closed_pitch, 0.0, 0.0)),
        material=flap_paint,
        name="right_side_skirt",
    )
    flap.visual(
        Box((0.34, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, -0.223, -0.019), rpy=(closed_pitch, 0.0, 0.0)),
        material=flap_paint,
        name="front_drip_edge",
    )
    flap.visual(
        Box((0.022, 0.130, 0.020)),
        origin=Origin(xyz=(-0.095, -0.126, -0.015), rpy=(closed_pitch, 0.0, 0.0)),
        material=flap_paint,
        name="left_stiffener",
    )
    flap.visual(
        Box((0.022, 0.130, 0.020)),
        origin=Origin(xyz=(0.095, -0.126, -0.015), rpy=(closed_pitch, 0.0, 0.0)),
        material=flap_paint,
        name="right_stiffener",
    )
    flap.visual(
        Box((0.300, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, -0.056, -0.012), rpy=(closed_pitch, 0.0, 0.0)),
        material=flap_paint,
        name="hinge_mount_rail",
    )
    flap.visual(
        Box((0.055, 0.040, 0.006)),
        origin=Origin(xyz=(-0.120, -0.024, -0.006), rpy=(closed_pitch, 0.0, 0.0)),
        material=flap_paint,
        name="left_hinge_leaf",
    )
    flap.visual(
        Box((0.055, 0.040, 0.006)),
        origin=Origin(xyz=(0.120, -0.024, -0.006), rpy=(closed_pitch, 0.0, 0.0)),
        material=flap_paint,
        name="right_hinge_leaf",
    )
    flap.visual(
        Box((0.040, 0.014, 0.040)),
        origin=Origin(xyz=(-0.120, -0.006, -0.020)),
        material=flap_paint,
        name="left_hinge_strap",
    )
    flap.visual(
        Box((0.040, 0.014, 0.040)),
        origin=Origin(xyz=(0.120, -0.006, -0.020)),
        material=flap_paint,
        name="right_hinge_strap",
    )
    flap.visual(
        Cylinder(radius=hinge_radius, length=0.08),
        origin=Origin(
            xyz=(-0.120, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=zinc,
        name="left_hinge_barrel",
    )
    flap.visual(
        Cylinder(radius=hinge_radius, length=0.08),
        origin=Origin(
            xyz=(0.120, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=zinc,
        name="right_hinge_barrel",
    )

    flap_bolts = [
        (-0.120, -0.024, 0.003, 0.004),
        (0.120, -0.024, 0.003, 0.004),
        (-0.095, -0.126, 0.000, 0.006),
        (0.095, -0.126, 0.000, 0.006),
    ]
    for idx, (x_pos, y_pos, z_pos, length) in enumerate(flap_bolts, start=1):
        flap.visual(
            Cylinder(radius=0.005, length=length),
            origin=Origin(
                xyz=(x_pos, y_pos, z_pos),
                rpy=(closed_pitch, 0.0, 0.0),
            ),
            material=zinc,
            name=f"flap_bolt_{idx}",
        )

    flap.visual(
        Box((0.200, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, -0.110, -0.007), rpy=(closed_pitch, 0.0, 0.0)),
        material=rubber,
        name="seal_strip",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.39, 0.22, 0.06)),
        mass=6.0,
        origin=Origin(xyz=(0.0, -0.105, -0.020)),
    )

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("vent_tower")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("tower_to_flap")

    roof_flange = tower.get_visual("roof_flange")
    mounting_curb = tower.get_visual("mounting_curb")
    outlet_lintel = tower.get_visual("outlet_lintel")
    opening_top_liner = tower.get_visual("opening_top_liner")
    center_barrel = tower.get_visual("hinge_center_barrel")

    cap_panel = flap.get_visual("cap_panel")
    front_drip = flap.get_visual("front_drip_edge")
    left_barrel = flap.get_visual("left_hinge_barrel")
    right_barrel = flap.get_visual("right_hinge_barrel")
    seal_strip = flap.get_visual("seal_strip")

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

    ctx.fail_if_articulation_overlaps(max_pose_samples=16)

    ctx.expect_overlap(
        flap,
        tower,
        axes="x",
        elem_a=cap_panel,
        elem_b=outlet_lintel,
        min_overlap=0.24,
        name="cap_spans_outlet_width",
    )
    ctx.expect_gap(
        flap,
        tower,
        axis="z",
        positive_elem=front_drip,
        negative_elem=outlet_lintel,
        min_gap=0.004,
        max_gap=0.020,
        name="flap_sits_just_above_lintel",
    )
    ctx.expect_gap(
        flap,
        tower,
        axis="z",
        positive_elem=seal_strip,
        negative_elem=opening_top_liner,
        min_gap=0.020,
        max_gap=0.050,
        name="seal_strip_hangs_close_to_top_liner",
    )
    ctx.expect_overlap(
        flap,
        tower,
        axes="yz",
        elem_a=left_barrel,
        elem_b=center_barrel,
        min_overlap=0.018,
        name="left_knuckle_is_coaxial_with_center_barrel",
    )
    ctx.expect_overlap(
        flap,
        tower,
        axes="yz",
        elem_a=right_barrel,
        elem_b=center_barrel,
        min_overlap=0.018,
        name="right_knuckle_is_coaxial_with_center_barrel",
    )
    ctx.expect_contact(
        flap,
        tower,
        elem_a=left_barrel,
        elem_b=center_barrel,
        name="left_hinge_knuckle_bears_on_center_pivot",
    )
    ctx.expect_contact(
        flap,
        tower,
        elem_a=right_barrel,
        elem_b=center_barrel,
        name="right_hinge_knuckle_bears_on_center_pivot",
    )

    tower_aabb = ctx.part_world_aabb(tower)
    flap_aabb = ctx.part_world_aabb(flap)
    assert tower_aabb is not None
    assert flap_aabb is not None
    tower_width = tower_aabb[1][0] - tower_aabb[0][0]
    tower_height = tower_aabb[1][2] - tower_aabb[0][2]
    flap_depth = flap_aabb[1][1] - flap_aabb[0][1]
    ctx.check(
        "tower_has_rugged_utility_proportions",
        tower_width > 0.45 and tower_height > 0.19 and flap_depth > 0.19,
        details=(
            f"tower_width={tower_width:.3f}, tower_height={tower_height:.3f}, "
            f"flap_depth={flap_depth:.3f}"
        ),
    )

    flange_aabb = ctx.part_element_world_aabb(tower, elem="roof_flange")
    curb_aabb = ctx.part_element_world_aabb(tower, elem="mounting_curb")
    assert flange_aabb is not None
    assert curb_aabb is not None
    flange_width = flange_aabb[1][0] - flange_aabb[0][0]
    curb_width = curb_aabb[1][0] - curb_aabb[0][0]
    ctx.check(
        "roof_flange_projects_beyond_tower_shell",
        flange_width - curb_width > 0.08,
        details=f"flange_width={flange_width:.3f}, curb_width={curb_width:.3f}",
    )

    with ctx.pose({hinge: math.radians(60.0)}):
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            positive_elem=front_drip,
            negative_elem=outlet_lintel,
            min_gap=0.135,
            name="open_flap_lifts_clear_of_outlet",
        )
        ctx.expect_overlap(
            flap,
            tower,
            axes="yz",
            elem_a=left_barrel,
            elem_b=center_barrel,
            min_overlap=0.018,
            name="left_knuckle_stays_on_pivot_axis_when_open",
        )
        ctx.expect_overlap(
            flap,
            tower,
            axes="yz",
            elem_a=right_barrel,
            elem_b=center_barrel,
            min_overlap=0.018,
            name="right_knuckle_stays_on_pivot_axis_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
