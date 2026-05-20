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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    z_center: float,
    *,
    exponent: float = 2.5,
    segments: int = 32,
) -> list[tuple[float, float, float]]:
    profile = superellipse_profile(width, height, exponent=exponent, segments=segments)
    return [(x_pos, y_pos, z_center + z_pos) for y_pos, z_pos in profile]


def _outer_body_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0158, 0.000),
            (0.0158, 0.145),
            (0.0195, 0.145),
            (0.0195, 0.320),
            (0.0215, 0.320),
            (0.0215, 0.345),
        ],
        [
            (0.0148, 0.000),
            (0.0148, 0.145),
            (0.0152, 0.145),
            (0.0152, 0.320),
            (0.0150, 0.320),
            (0.0150, 0.345),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _lockout_ring_shell():
    return LatheGeometry.from_shell_profiles(
        [(0.0235, -0.009), (0.0235, 0.009)],
        [(0.0204, -0.009), (0.0204, 0.009)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def _mid_band_shell():
    return LatheGeometry.from_shell_profiles(
        [(0.0208, -0.011), (0.0208, 0.011)],
        [(0.0192, -0.011), (0.0192, 0.011)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def _saddle_shell():
    sections = [
        _yz_section(-0.115, 0.185, 0.034, 0.061, exponent=2.35),
        _yz_section(-0.075, 0.178, 0.032, 0.064, exponent=2.35),
        _yz_section(-0.020, 0.148, 0.028, 0.060, exponent=2.45),
        _yz_section(0.035, 0.108, 0.024, 0.055, exponent=2.55),
        _yz_section(0.085, 0.062, 0.018, 0.050, exponent=2.7),
        _yz_section(0.120, 0.028, 0.012, 0.047, exponent=2.8),
    ]
    return section_loft(sections)


def _rail_points(side: float) -> list[tuple[float, float, float]]:
    sweep = 1.0 if side > 0.0 else -1.0
    return [
        (-0.104, side * 0.68, 0.046),
        (-0.084, side * 0.86, 0.032),
        (-0.036, side * 1.00, 0.000),
        (0.036, side * 0.96, 0.000),
        (0.080, side * 0.78, 0.021),
        (0.108, side * 0.56, 0.046 + 0.002 * sweep),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_chamber_dropper_seatpost")

    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.16, 0.16, 0.17, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.24, 0.26, 1.0))
    soft_black = model.material("soft_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))

    outer_body = model.part("outer_body")
    outer_body.visual(
        mesh_from_geometry(_outer_body_shell(), "seatpost_outer_shell"),
        material=anodized_black,
        name="outer_shell",
    )
    outer_body.visual(
        mesh_from_geometry(_mid_band_shell(), "seatpost_mid_band"),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=satin_black,
        name="mid_band",
    )
    outer_body.visual(
        Box((0.007, 0.018, 0.014)),
        origin=Origin(xyz=(0.0205, 0.0, 0.205)),
        material=dark_gray,
        name="lockout_stop",
    )
    outer_body.visual(
        Cylinder(radius=0.0065, length=0.028),
        origin=Origin(
            xyz=(-0.024, 0.0, 0.112),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_gray,
        name="cable_port",
    )
    outer_body.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=Origin(
            xyz=(-0.037, 0.0, 0.112),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=soft_black,
        name="cable_socket",
    )
    outer_body.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.345)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.1725)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0144, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_gray,
        name="inner_shaft",
    )
    inner_post.visual(
        Cylinder(radius=0.0188, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=satin_black,
        name="wiper_collar",
    )
    inner_post.visual(
        Cylinder(radius=0.0168, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.249)),
        material=anodized_black,
        name="post_crown",
    )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.038, 0.038, 0.482)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    model.articulation(
        "outer_body_to_inner_post",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.25,
            lower=0.0,
            upper=0.150,
        ),
    )

    lockout_collar = model.part("lockout_collar")
    lockout_collar.visual(
        Box((0.006, 0.046, 0.018)),
        origin=Origin(xyz=(-0.0225, 0.0, 0.0)),
        material=satin_black,
        name="rear_band",
    )
    lockout_collar.visual(
        Box((0.0375, 0.006, 0.018)),
        origin=Origin(xyz=(-0.00075, 0.0225, 0.0)),
        material=satin_black,
        name="top_band",
    )
    lockout_collar.visual(
        Box((0.0375, 0.006, 0.018)),
        origin=Origin(xyz=(-0.00075, -0.0225, 0.0)),
        material=satin_black,
        name="bottom_band",
    )
    lockout_collar.visual(
        Box((0.006, 0.010, 0.018)),
        origin=Origin(xyz=(0.021, 0.0145, 0.0)),
        material=satin_black,
        name="upper_link",
    )
    lockout_collar.visual(
        Box((0.006, 0.010, 0.018)),
        origin=Origin(xyz=(0.021, -0.0145, 0.0)),
        material=satin_black,
        name="lower_link",
    )
    lockout_collar.visual(
        Box((0.012, 0.019, 0.014)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=dark_gray,
        name="cam_tab",
    )
    lockout_collar.visual(
        Box((0.028, 0.012, 0.006)),
        origin=Origin(xyz=(0.050, 0.0, 0.004)),
        material=dark_gray,
        name="lever_body",
    )
    lockout_collar.visual(
        Box((0.020, 0.017, 0.004)),
        origin=Origin(xyz=(0.068, 0.0, 0.004)),
        material=soft_black,
        name="thumb_paddle",
    )
    lockout_collar.inertial = Inertial.from_geometry(
        Box((0.082, 0.050, 0.024)),
        mass=0.10,
        origin=Origin(xyz=(0.018, 0.0, 0.002)),
    )

    model.articulation(
        "outer_body_to_lockout_collar",
        ArticulationType.REVOLUTE,
        parent=outer_body,
        child=lockout_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=0.55,
        ),
    )

    clamp_head = model.part("clamp_head")
    clamp_head.visual(
        Box((0.044, 0.034, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=anodized_black,
        name="mast_cap",
    )
    clamp_head.visual(
        Box((0.022, 0.076, 0.008)),
        origin=Origin(xyz=(-0.032, 0.0, 0.0065)),
        material=satin_black,
        name="rear_cradle",
    )
    clamp_head.visual(
        Box((0.022, 0.076, 0.008)),
        origin=Origin(xyz=(0.032, 0.0, 0.0065)),
        material=satin_black,
        name="front_cradle",
    )
    clamp_head.visual(
        Box((0.022, 0.062, 0.006)),
        origin=Origin(xyz=(-0.032, 0.0, 0.031)),
        material=dark_gray,
        name="rear_top_clamp",
    )
    clamp_head.visual(
        Box((0.022, 0.062, 0.006)),
        origin=Origin(xyz=(0.032, 0.0, 0.031)),
        material=dark_gray,
        name="front_top_clamp",
    )
    clamp_head.visual(
        Box((0.090, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.043, 0.018)),
        material=dark_gray,
        name="left_cheek",
    )
    clamp_head.visual(
        Box((0.090, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, -0.043, 0.018)),
        material=dark_gray,
        name="right_cheek",
    )
    for x_pos, bolt_name in ((-0.032, "rear_bolt"), (0.032, "front_bolt")):
        clamp_head.visual(
            Cylinder(radius=0.0038, length=0.028),
            origin=Origin(xyz=(x_pos, 0.0, 0.020)),
            material=steel,
            name=bolt_name,
        )
        clamp_head.visual(
            Cylinder(radius=0.0060, length=0.005),
            origin=Origin(xyz=(x_pos, 0.0, 0.0345)),
            material=steel,
            name=f"{bolt_name}_head",
        )
    clamp_head.inertial = Inertial.from_geometry(
        Box((0.096, 0.082, 0.040)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    model.articulation(
        "inner_post_to_clamp_head",
        ArticulationType.FIXED,
        parent=inner_post,
        child=clamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_geometry(_saddle_shell(), "comfort_saddle_shell"),
        material=soft_black,
        name="comfort_shell",
    )
    saddle.visual(
        Box((0.034, 0.064, 0.005)),
        origin=Origin(xyz=(0.103, 0.0, 0.049)),
        material=dark_gray,
        name="nose_patch",
    )
    for side, rail_name in ((0.033, "left_rail"), (-0.033, "right_rail")):
        saddle.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _rail_points(side),
                    radius=0.0045,
                    samples_per_segment=18,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"{rail_name}_mesh",
            ),
            material=steel,
            name=rail_name,
        )
    for x_pos, y_pos, name in (
        (-0.052, 0.032, "left_rear_strut"),
        (-0.052, -0.032, "right_rear_strut"),
        (0.058, 0.026, "left_front_strut"),
        (0.058, -0.026, "right_front_strut"),
    ):
        saddle.visual(
            Cylinder(radius=0.0052, length=0.036),
            origin=Origin(xyz=(x_pos, y_pos, 0.022)),
            material=dark_gray,
            name=name,
        )
    saddle.inertial = Inertial.from_geometry(
        Box((0.240, 0.190, 0.082)),
        mass=0.58,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    model.articulation(
        "clamp_head_to_saddle",
        ArticulationType.REVOLUTE,
        parent=clamp_head,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.18,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_body = object_model.get_part("outer_body")
    inner_post = object_model.get_part("inner_post")
    lockout_collar = object_model.get_part("lockout_collar")
    clamp_head = object_model.get_part("clamp_head")
    saddle = object_model.get_part("saddle")

    post_slide = object_model.get_articulation("outer_body_to_inner_post")
    lockout_joint = object_model.get_articulation("outer_body_to_lockout_collar")
    saddle_tilt = object_model.get_articulation("clamp_head_to_saddle")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

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
        inner_post,
        outer_body,
        elem_a="wiper_collar",
        elem_b="outer_shell",
        name="inner post is supported at the guide head",
    )
    ctx.expect_contact(
        lockout_collar,
        outer_body,
        elem_a="cam_tab",
        elem_b="lockout_stop",
        name="lockout collar is mounted against the outer body stop",
    )
    ctx.expect_contact(
        clamp_head,
        inner_post,
        elem_a="mast_cap",
        elem_b="post_crown",
        name="clamp head seats on the post crown",
    )
    ctx.expect_contact(
        saddle,
        clamp_head,
        name="saddle rails seat in the clamp head",
    )

    ctx.expect_within(
        inner_post,
        outer_body,
        axes="xy",
        inner_elem="inner_shaft",
        outer_elem="outer_shell",
        margin=0.006,
        name="inner post stays centered in the outer tube",
    )
    ctx.expect_overlap(
        inner_post,
        outer_body,
        axes="z",
        elem_a="inner_shaft",
        elem_b="outer_shell",
        min_overlap=0.205,
        name="collapsed post retains deep insertion",
    )

    rest_post_pos = ctx.part_world_position(inner_post)
    max_extension = post_slide.motion_limits.upper or 0.0
    with ctx.pose({post_slide: max_extension}):
        ctx.expect_overlap(
            inner_post,
            outer_body,
            axes="z",
            elem_a="inner_shaft",
            elem_b="outer_shell",
            min_overlap=0.055,
            name="extended post still retains insertion",
        )
        extended_post_pos = ctx.part_world_position(inner_post)
    ctx.check(
        "inner post extends upward",
        rest_post_pos is not None
        and extended_post_pos is not None
        and extended_post_pos[2] > rest_post_pos[2] + 0.12,
        details=f"rest={rest_post_pos}, extended={extended_post_pos}",
    )

    rest_paddle = aabb_center(ctx.part_element_world_aabb(lockout_collar, elem="thumb_paddle"))
    with ctx.pose({lockout_joint: lockout_joint.motion_limits.upper or 0.0}):
        open_paddle = aabb_center(ctx.part_element_world_aabb(lockout_collar, elem="thumb_paddle"))
    ctx.check(
        "lockout lever swings around the collar",
        rest_paddle is not None
        and open_paddle is not None
        and open_paddle[1] > rest_paddle[1] + 0.020,
        details=f"rest={rest_paddle}, open={open_paddle}",
    )

    rest_nose = aabb_center(ctx.part_element_world_aabb(saddle, elem="nose_patch"))
    with ctx.pose({saddle_tilt: saddle_tilt.motion_limits.upper or 0.0}):
        nose_up = aabb_center(ctx.part_element_world_aabb(saddle, elem="nose_patch"))
    with ctx.pose({saddle_tilt: saddle_tilt.motion_limits.lower or 0.0}):
        nose_down = aabb_center(ctx.part_element_world_aabb(saddle, elem="nose_patch"))
    ctx.check(
        "saddle tilt pivot changes nose height",
        rest_nose is not None
        and nose_up is not None
        and nose_down is not None
        and nose_up[2] > rest_nose[2] + 0.010
        and nose_down[2] < rest_nose[2] - 0.008,
        details=f"rest={rest_nose}, up={nose_up}, down={nose_down}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
