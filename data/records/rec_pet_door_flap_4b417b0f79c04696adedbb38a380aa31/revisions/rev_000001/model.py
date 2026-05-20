from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

OPENING_WIDTH = 0.30
OPENING_HEIGHT = 0.38
TUNNEL_DEPTH = 0.13


def _clamped_radius(width: float, height: float, radius: float) -> float:
    return max(0.001, min(radius, width * 0.5 - 0.001, height * 0.5 - 0.001))


def _rounded_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    radius: float,
    name: str,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, _clamped_radius(width, height, radius)),
        thickness,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _frame_ring_mesh(
    *,
    outer_width: float,
    outer_height: float,
    inner_width: float,
    inner_height: float,
    depth: float,
    outer_radius: float,
    inner_radius: float,
    name: str,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(
            outer_width,
            outer_height,
            _clamped_radius(outer_width, outer_height, outer_radius),
        ),
        [
            rounded_rect_profile(
                inner_width,
                inner_height,
                _clamped_radius(inner_width, inner_height, inner_radius),
            )
        ],
        depth,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dog_door_with_security_panel")

    frame_plastic = model.material("frame_plastic", rgba=(0.26, 0.24, 0.22, 1.0))
    inner_trim = model.material("inner_trim", rgba=(0.18, 0.18, 0.18, 1.0))
    flap_smoke = model.material("flap_smoke", rgba=(0.44, 0.50, 0.54, 0.45))
    flap_frame = model.material("flap_frame", rgba=(0.10, 0.10, 0.10, 1.0))
    panel_abs = model.material("panel_abs", rgba=(0.86, 0.86, 0.82, 1.0))
    panel_handle = model.material("panel_handle", rgba=(0.20, 0.20, 0.20, 1.0))

    opening_width = OPENING_WIDTH
    opening_height = OPENING_HEIGHT
    tunnel_depth = TUNNEL_DEPTH
    tunnel_outer_width = 0.42
    tunnel_outer_height = 0.54
    front_bezel_width = 0.50
    front_bezel_height = 0.62
    rear_bezel_width = 0.46
    rear_bezel_height = 0.58
    bezel_thickness = 0.018

    flap_y = -0.018
    panel_y = 0.015

    frame = model.part("frame")

    side_wall_width = (tunnel_outer_width - opening_width) * 0.5
    tunnel_top_rail_height = 0.080
    header_clear_width = opening_width - 0.036
    tunnel_jamb_center_x = opening_width * 0.5 + side_wall_width * 0.5
    tunnel_header_center_z = opening_height * 0.5 + tunnel_top_rail_height * 0.5

    frame.visual(
        Box((side_wall_width, tunnel_depth, tunnel_outer_height)),
        origin=Origin(xyz=(-tunnel_jamb_center_x, 0.0, 0.0)),
        material=frame_plastic,
        name="left_jamb",
    )
    frame.visual(
        Box((side_wall_width, tunnel_depth, tunnel_outer_height)),
        origin=Origin(xyz=(tunnel_jamb_center_x, 0.0, 0.0)),
        material=frame_plastic,
        name="right_jamb",
    )
    frame.visual(
        Box((header_clear_width, tunnel_depth, tunnel_top_rail_height)),
        origin=Origin(xyz=(0.0, 0.0, tunnel_header_center_z)),
        material=frame_plastic,
        name="header",
    )

    front_inner_width = opening_width + 0.020
    front_inner_height = opening_height + 0.034
    front_side_trim_width = (front_bezel_width - front_inner_width) * 0.5
    front_top_trim_height = (front_bezel_height - front_inner_height) * 0.5
    front_trim_y = -tunnel_depth * 0.5 + bezel_thickness * 0.5
    front_trim_center_z = front_inner_height * 0.5 + front_top_trim_height * 0.5
    front_side_trim_center_x = front_inner_width * 0.5 + front_side_trim_width * 0.5

    frame.visual(
        Box((front_side_trim_width, bezel_thickness, front_bezel_height)),
        origin=Origin(xyz=(-front_side_trim_center_x, front_trim_y, 0.0)),
        material=frame_plastic,
        name="front_left_bezel",
    )
    frame.visual(
        Box((front_side_trim_width, bezel_thickness, front_bezel_height)),
        origin=Origin(xyz=(front_side_trim_center_x, front_trim_y, 0.0)),
        material=frame_plastic,
        name="front_right_bezel",
    )
    frame.visual(
        Box((front_inner_width, bezel_thickness, front_top_trim_height)),
        origin=Origin(xyz=(0.0, front_trim_y, front_trim_center_z)),
        material=frame_plastic,
        name="front_top_bezel",
    )
    frame.visual(
        Box((front_inner_width, bezel_thickness, front_top_trim_height)),
        origin=Origin(xyz=(0.0, front_trim_y, -front_trim_center_z)),
        material=frame_plastic,
        name="front_bottom_bezel",
    )

    rear_inner_width = opening_width + 0.006
    rear_inner_height = opening_height + 0.016
    rear_side_trim_width = (rear_bezel_width - rear_inner_width) * 0.5
    rear_top_trim_height = (rear_bezel_height - rear_inner_height) * 0.5
    rear_trim_y = tunnel_depth * 0.5 - bezel_thickness * 0.5
    rear_trim_center_z = rear_inner_height * 0.5 + rear_top_trim_height * 0.5
    rear_side_trim_center_x = rear_inner_width * 0.5 + rear_side_trim_width * 0.5

    frame.visual(
        Box((rear_side_trim_width, bezel_thickness, rear_bezel_height)),
        origin=Origin(xyz=(-rear_side_trim_center_x, rear_trim_y, 0.0)),
        material=frame_plastic,
        name="rear_left_bezel",
    )
    frame.visual(
        Box((rear_side_trim_width, bezel_thickness, rear_bezel_height)),
        origin=Origin(xyz=(rear_side_trim_center_x, rear_trim_y, 0.0)),
        material=frame_plastic,
        name="rear_right_bezel",
    )
    frame.visual(
        Box((rear_inner_width, bezel_thickness, rear_top_trim_height)),
        origin=Origin(xyz=(0.0, rear_trim_y, rear_trim_center_z)),
        material=frame_plastic,
        name="rear_top_bezel",
    )
    frame.visual(
        Box((rear_inner_width, bezel_thickness, rear_top_trim_height)),
        origin=Origin(xyz=(0.0, rear_trim_y, -rear_trim_center_z)),
        material=frame_plastic,
        name="rear_bottom_bezel",
    )

    tower_width = side_wall_width
    tower_top = 0.62
    tower_bottom = tunnel_outer_height * 0.5
    tower_height = tower_top - tower_bottom
    tower_center_z = (tower_top + tower_bottom) * 0.5
    tower_center_x = opening_width * 0.5 + tower_width * 0.5

    frame.visual(
        Box((tower_width, tunnel_depth, tower_height)),
        origin=Origin(xyz=(-tower_center_x, 0.0, tower_center_z)),
        material=frame_plastic,
        name="left_storage_tower",
    )
    frame.visual(
        Box((tower_width, tunnel_depth, tower_height)),
        origin=Origin(xyz=(tower_center_x, 0.0, tower_center_z)),
        material=frame_plastic,
        name="right_storage_tower",
    )
    frame.visual(
        Box((tunnel_outer_width, tunnel_depth, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, tower_top - 0.018)),
        material=frame_plastic,
        name="storage_cap",
    )

    channel_base_thickness = 0.004
    channel_lip_depth = 0.016
    channel_depth = 0.028
    lip_thickness = 0.004
    channel_height = tower_top - (-0.205)
    channel_center_z = (tower_top + (-0.205)) * 0.5
    channel_inner_face_x = opening_width * 0.5
    channel_base_center_x = channel_inner_face_x - channel_base_thickness * 0.5
    lip_center_offset_x = channel_lip_depth * 0.5
    front_lip_y = panel_y - (channel_depth * 0.5 - lip_thickness * 0.5)
    back_lip_y = panel_y + (channel_depth * 0.5 - lip_thickness * 0.5)

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            Box((channel_base_thickness, channel_depth, channel_height)),
            origin=Origin(
                xyz=(side_sign * channel_base_center_x, panel_y, channel_center_z)
            ),
            material=inner_trim,
            name=f"{side_name}_channel_base",
        )
        frame.visual(
            Box((channel_lip_depth, lip_thickness, channel_height)),
            origin=Origin(
                xyz=(side_sign * (channel_inner_face_x - lip_center_offset_x), front_lip_y, channel_center_z)
            ),
            material=inner_trim,
            name=f"{side_name}_front_lip",
        )
        frame.visual(
            Box((channel_lip_depth, lip_thickness, channel_height)),
            origin=Origin(
                xyz=(side_sign * (channel_inner_face_x - lip_center_offset_x), back_lip_y, channel_center_z)
            ),
            material=inner_trim,
            name=f"{side_name}_back_lip",
        )

    frame.visual(
        Box((opening_width, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, flap_y, -0.214)),
        material=inner_trim,
        name="flap_stop",
    )
    frame.visual(
        Box((opening_width, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, panel_y, -0.211)),
        material=inner_trim,
        name="panel_stop",
    )
    frame.inertial = Inertial.from_geometry(
        Box((front_bezel_width, tunnel_depth, tower_top + tunnel_outer_height * 0.5)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, (tower_top - tunnel_outer_height * 0.5) * 0.5)),
    )

    flap = model.part("flap")
    flap.visual(
        _rounded_panel_mesh(
            width=0.286,
            height=0.372,
            thickness=0.008,
            radius=0.014,
            name="dog_door_flap_panel",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.194)),
        material=flap_smoke,
        name="main_panel",
    )
    flap.visual(
        Box((0.256, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=flap_frame,
        name="top_rail",
    )
    flap.visual(
        Box((0.274, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.392)),
        material=flap_frame,
        name="bottom_bar",
    )
    flap.visual(
        Box((0.012, 0.014, 0.365)),
        origin=Origin(xyz=(-0.137, 0.0, -0.2035)),
        material=flap_frame,
        name="left_stile",
    )
    flap.visual(
        Box((0.012, 0.014, 0.365)),
        origin=Origin(xyz=(0.137, 0.0, -0.2035)),
        material=flap_frame,
        name="right_stile",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.290, 0.016, 0.402)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.201)),
    )

    security_panel = model.part("security_panel")
    security_panel.visual(
        _rounded_panel_mesh(
            width=0.272,
            height=0.370,
            thickness=0.006,
            radius=0.010,
            name="dog_door_security_panel",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=panel_abs,
        name="main_panel",
    )
    security_panel.visual(
        Box((0.010, 0.016, 0.406)),
        origin=Origin(xyz=(-0.141, 0.0, 0.203)),
        material=panel_abs,
        name="left_runner",
    )
    security_panel.visual(
        Box((0.010, 0.016, 0.406)),
        origin=Origin(xyz=(0.141, 0.0, 0.203)),
        material=panel_abs,
        name="right_runner",
    )
    security_panel.visual(
        Box((0.272, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=panel_abs,
        name="bottom_shoe",
    )
    security_panel.visual(
        Box((0.122, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.342)),
        material=panel_handle,
        name="pull_grip",
    )
    security_panel.inertial = Inertial.from_geometry(
        Box((0.292, 0.020, 0.410)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
    )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, flap_y, opening_height * 0.5)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "security_panel_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=security_panel,
        origin=Origin(xyz=(0.0, panel_y, -0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.25,
            lower=0.0,
            upper=0.405,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    security_panel = object_model.get_part("security_panel")
    flap_hinge = object_model.get_articulation("flap_hinge")
    security_panel_slide = object_model.get_articulation("security_panel_slide")

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

    ctx.expect_contact(flap, frame, elem_a="bottom_bar", elem_b="flap_stop")
    ctx.expect_contact(security_panel, frame, elem_a="bottom_shoe", elem_b="panel_stop")
    ctx.expect_contact(
        security_panel,
        frame,
        elem_a="left_runner",
        elem_b="left_channel_base",
    )
    ctx.expect_contact(
        security_panel,
        frame,
        elem_a="right_runner",
        elem_b="right_channel_base",
    )
    ctx.expect_gap(
        security_panel,
        flap,
        axis="y",
        min_gap=0.020,
        max_gap=0.035,
        positive_elem="main_panel",
        negative_elem="main_panel",
    )

    def _check_panel_capture() -> None:
        for side in ("left", "right"):
            ctx.expect_gap(
                security_panel,
                frame,
                axis="y",
                min_gap=0.001,
                max_gap=0.004,
                positive_elem=f"{side}_runner",
                negative_elem=f"{side}_front_lip",
                name=f"{side}_runner_clear_of_front_lip",
            )
            ctx.expect_gap(
                frame,
                security_panel,
                axis="y",
                min_gap=0.001,
                max_gap=0.004,
                positive_elem=f"{side}_back_lip",
                negative_elem=f"{side}_runner",
                name=f"{side}_runner_clear_of_back_lip",
            )
            ctx.expect_overlap(
                security_panel,
                frame,
                axes="xz",
                min_overlap=0.009,
                elem_a=f"{side}_runner",
                elem_b=f"{side}_front_lip",
                name=f"{side}_runner_covered_by_front_lip",
            )
            ctx.expect_overlap(
                security_panel,
                frame,
                axes="xz",
                min_overlap=0.009,
                elem_a=f"{side}_runner",
                elem_b=f"{side}_back_lip",
                name=f"{side}_runner_covered_by_back_lip",
            )

    _check_panel_capture()

    flap_rest_aabb = ctx.part_world_aabb(flap)
    panel_rest_aabb = ctx.part_world_aabb(security_panel)

    with ctx.pose({flap_hinge: 0.85}):
        flap_open_aabb = ctx.part_world_aabb(flap)
        flap_swung = (
            flap_rest_aabb is not None
            and flap_open_aabb is not None
            and flap_open_aabb[1][1] > flap_rest_aabb[1][1] + 0.18
        )
        ctx.check(
            "flap_swings_inward_about_top_axis",
            flap_swung,
            details=(
                f"rest_aabb={flap_rest_aabb}, open_aabb={flap_open_aabb}, "
                "expected flap to sweep rearward into the tunnel."
            ),
        )

    with ctx.pose({security_panel_slide: 0.405}):
        raised_panel_aabb = ctx.part_world_aabb(security_panel)
        panel_raised = (
            panel_rest_aabb is not None
            and raised_panel_aabb is not None
            and raised_panel_aabb[1][2] > panel_rest_aabb[1][2] + 0.38
            and raised_panel_aabb[0][2] > OPENING_HEIGHT * 0.5 - 0.005
        )
        ctx.check(
            "security_panel_slides_up_out_of_opening",
            panel_raised,
            details=(
                f"rest_aabb={panel_rest_aabb}, raised_aabb={raised_panel_aabb}, "
                "expected the security panel to lift above the flap opening."
            ),
        )
        _check_panel_capture()
        ctx.expect_contact(
            security_panel,
            frame,
            elem_a="left_runner",
            elem_b="left_channel_base",
            name="left_runner_stays_guided_when_raised",
        )
        ctx.expect_contact(
            security_panel,
            frame,
            elem_a="right_runner",
            elem_b="right_channel_base",
            name="right_runner_stays_guided_when_raised",
        )

    with ctx.pose({security_panel_slide: 0.405, flap_hinge: 0.85}):
        ctx.expect_gap(
            security_panel,
            flap,
            axis="z",
            min_gap=0.002,
            positive_elem="bottom_shoe",
            negative_elem="top_rail",
            name="raised_panel_clears_open_flap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
