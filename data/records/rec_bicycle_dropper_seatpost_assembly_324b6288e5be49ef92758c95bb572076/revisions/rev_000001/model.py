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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    segments: int = 48,
    reverse: bool = False,
) -> list[tuple[float, float]]:
    points = [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]
    if reverse:
        points.reverse()
    return points


def _annulus_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    name: str,
    segments: int = 48,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [_circle_profile(inner_radius, segments=segments, reverse=True)],
            height,
            cap=True,
            center=True,
            closed=True,
        ),
        name,
    )


def _rounded_plate_mesh(
    *,
    width: float,
    depth: float,
    height: float,
    radius: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius),
            height,
            cap=True,
            center=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="air_sprung_dropper_seatpost")

    lower_length = 0.280
    lower_outer_radius = 0.0170
    lower_inner_radius = 0.0156
    inner_post_radius = 0.0145
    inner_post_length = 0.330
    travel = 0.170
    guide_length = 0.018
    guide_radial_thickness = 0.0020
    guide_span = 0.006

    lower_shell_mesh = _annulus_mesh(
        outer_radius=lower_outer_radius,
        inner_radius=lower_inner_radius,
        height=lower_length,
        name="lower_outer_sleeve_shell",
    )
    top_collar_mesh = _annulus_mesh(
        outer_radius=0.0190,
        inner_radius=lower_inner_radius,
        height=0.016,
        name="lower_outer_top_collar",
    )
    head_body_mesh = _rounded_plate_mesh(
        width=0.040,
        depth=0.022,
        height=0.012,
        radius=0.004,
        name="dropper_head_body_plate",
    )
    cap_bridge_mesh = _rounded_plate_mesh(
        width=0.042,
        depth=0.024,
        height=0.006,
        radius=0.0035,
        name="dropper_cap_bridge_plate",
    )
    anodized_black = model.material("anodized_black", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.19, 0.20, 1.0))
    seal_black = model.material("seal_black", rgba=(0.05, 0.05, 0.06, 1.0))
    hard_anodized = model.material("hard_anodized", rgba=(0.20, 0.21, 0.23, 1.0))
    hardware_silver = model.material("hardware_silver", rgba=(0.76, 0.78, 0.80, 1.0))
    valve_brass = model.material("valve_brass", rgba=(0.73, 0.62, 0.30, 1.0))

    lower_sleeve = model.part("lower_sleeve")
    outer_shell = lower_sleeve.visual(
        lower_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, lower_length * 0.5)),
        material=anodized_black,
        name="outer_shell",
    )
    lower_sleeve.visual(
        top_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, lower_length - 0.008)),
        material=satin_black,
        name="top_collar",
    )
    lower_sleeve.visual(
        Box((guide_radial_thickness, guide_span, guide_length)),
        origin=Origin(
            xyz=(
                inner_post_radius + (guide_radial_thickness * 0.5),
                0.0,
                lower_length - (guide_length * 0.5),
            )
        ),
        material=seal_black,
        name="right_guide_pad",
    )
    lower_sleeve.visual(
        Box((guide_radial_thickness, guide_span, guide_length)),
        origin=Origin(
            xyz=(
                -(inner_post_radius + (guide_radial_thickness * 0.5)),
                0.0,
                lower_length - (guide_length * 0.5),
            )
        ),
        material=seal_black,
        name="left_guide_pad",
    )
    lower_sleeve.visual(
        Box((guide_span, guide_radial_thickness, guide_length)),
        origin=Origin(
            xyz=(
                0.0,
                inner_post_radius + (guide_radial_thickness * 0.5),
                lower_length - (guide_length * 0.5),
            )
        ),
        material=seal_black,
        name="front_guide_pad",
    )
    lower_sleeve.visual(
        Box((guide_span, guide_radial_thickness, guide_length)),
        origin=Origin(
            xyz=(
                0.0,
                -(inner_post_radius + (guide_radial_thickness * 0.5)),
                lower_length - (guide_length * 0.5),
            )
        ),
        material=seal_black,
        name="rear_guide_pad",
    )
    lower_sleeve.inertial = Inertial.from_geometry(
        Cylinder(radius=lower_outer_radius, length=lower_length),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, lower_length * 0.5)),
    )

    valve_boss = model.part("valve_boss")
    valve_boss.visual(
        Cylinder(radius=0.0055, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_black,
        name="boss_base",
    )
    valve_boss.visual(
        Cylinder(radius=0.0022, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=valve_brass,
        name="valve_stem",
    )
    valve_boss.visual(
        Cylinder(radius=0.0030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=seal_black,
        name="valve_cap",
    )
    valve_boss.inertial = Inertial.from_geometry(
        Box((0.014, 0.014, 0.030)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=inner_post_radius, length=inner_post_length),
        origin=Origin(xyz=(0.0, 0.0, inner_post_length * 0.5)),
        material=hard_anodized,
        name="stanchion",
    )
    inner_post.inertial = Inertial.from_geometry(
        Cylinder(radius=inner_post_radius, length=inner_post_length),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, inner_post_length * 0.5)),
    )

    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        head_body_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_black,
        name="head_body",
    )
    clamp_base.visual(
        Box((0.012, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, 0.009, 0.014)),
        material=satin_black,
        name="front_tower",
    )
    clamp_base.visual(
        Box((0.012, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.009, 0.014)),
        material=satin_black,
        name="rear_tower",
    )
    clamp_base.visual(
        Box((0.004, 0.020, 0.014)),
        origin=Origin(xyz=(-0.020, 0.0, 0.021)),
        material=satin_black,
        name="left_cheek",
    )
    clamp_base.visual(
        Box((0.004, 0.020, 0.014)),
        origin=Origin(xyz=(0.020, 0.0, 0.021)),
        material=satin_black,
        name="right_cheek",
    )
    clamp_base.visual(
        Cylinder(radius=0.0045, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_silver,
        name="lower_cradle",
    )
    clamp_base.inertial = Inertial.from_geometry(
        Box((0.048, 0.026, 0.030)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    clamp_cap = model.part("clamp_cap")
    clamp_cap.visual(
        Cylinder(radius=0.0035, length=0.036),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_silver,
        name="pivot_barrel",
    )
    clamp_cap.visual(
        Cylinder(radius=0.0045, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_silver,
        name="upper_cradle",
    )
    clamp_cap.visual(
        cap_bridge_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=satin_black,
        name="cap_bridge",
    )
    clamp_cap.visual(
        Cylinder(radius=0.0022, length=0.010),
        origin=Origin(xyz=(0.0, 0.009, 0.012)),
        material=hardware_silver,
        name="front_bolt_shaft",
    )
    clamp_cap.visual(
        Cylinder(radius=0.0040, length=0.006),
        origin=Origin(xyz=(0.0, 0.009, 0.020)),
        material=hardware_silver,
        name="front_bolt_head",
    )
    clamp_cap.visual(
        Cylinder(radius=0.0022, length=0.010),
        origin=Origin(xyz=(0.0, -0.009, 0.012)),
        material=hardware_silver,
        name="rear_bolt_shaft",
    )
    clamp_cap.visual(
        Cylinder(radius=0.0040, length=0.006),
        origin=Origin(xyz=(0.0, -0.009, 0.020)),
        material=hardware_silver,
        name="rear_bolt_head",
    )
    clamp_cap.inertial = Inertial.from_geometry(
        Box((0.046, 0.028, 0.028)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    model.articulation(
        "lower_sleeve_to_valve_boss",
        ArticulationType.FIXED,
        parent=lower_sleeve,
        child=valve_boss,
        origin=place_on_surface(
            valve_boss,
            outer_shell,
            point_hint=(lower_outer_radius, 0.0, 0.185),
            child_axis="+z",
            clearance=0.0,
            prefer_collisions=False,
            child_prefer_collisions=False,
        ),
    )
    model.articulation(
        "post_travel",
        ArticulationType.PRISMATIC,
        parent=lower_sleeve,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=travel,
        ),
    )
    model.articulation(
        "inner_post_to_clamp_base",
        ArticulationType.FIXED,
        parent=inner_post,
        child=clamp_base,
        origin=Origin(xyz=(0.0, 0.0, inner_post_length)),
    )
    model.articulation(
        "clamp_pitch",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=clamp_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-0.24,
            upper=0.24,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_sleeve = object_model.get_part("lower_sleeve")
    valve_boss = object_model.get_part("valve_boss")
    inner_post = object_model.get_part("inner_post")
    clamp_base = object_model.get_part("clamp_base")
    clamp_cap = object_model.get_part("clamp_cap")

    post_travel = object_model.get_articulation("post_travel")
    clamp_pitch = object_model.get_articulation("clamp_pitch")

    outer_shell = lower_sleeve.get_visual("outer_shell")
    right_guide_pad = lower_sleeve.get_visual("right_guide_pad")
    left_guide_pad = lower_sleeve.get_visual("left_guide_pad")
    boss_base = valve_boss.get_visual("boss_base")
    stanchion = inner_post.get_visual("stanchion")
    head_body = clamp_base.get_visual("head_body")
    left_cheek = clamp_base.get_visual("left_cheek")
    right_cheek = clamp_base.get_visual("right_cheek")
    lower_cradle = clamp_base.get_visual("lower_cradle")
    pivot_barrel = clamp_cap.get_visual("pivot_barrel")
    upper_cradle = clamp_cap.get_visual("upper_cradle")
    front_bolt_head = clamp_cap.get_visual("front_bolt_head")
    rear_bolt_head = clamp_cap.get_visual("rear_bolt_head")

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
        valve_boss,
        lower_sleeve,
        elem_a=boss_base,
        elem_b=outer_shell,
        name="valve_boss_contacts_sleeve",
    )
    ctx.expect_origin_distance(
        lower_sleeve,
        inner_post,
        axes="xy",
        max_dist=0.0005,
        name="stanchion_concentric_with_sleeve",
    )
    ctx.expect_contact(
        lower_sleeve,
        inner_post,
        elem_a=right_guide_pad,
        elem_b=stanchion,
        name="right_guide_pad_contacts_stanchion",
    )
    ctx.expect_contact(
        lower_sleeve,
        inner_post,
        elem_a=left_guide_pad,
        elem_b=stanchion,
        name="left_guide_pad_contacts_stanchion",
    )
    ctx.expect_within(
        inner_post,
        lower_sleeve,
        axes="xy",
        margin=0.003,
        inner_elem=stanchion,
        outer_elem=outer_shell,
        name="stanchion_centered_in_sleeve",
    )
    ctx.expect_overlap(
        inner_post,
        lower_sleeve,
        axes="xy",
        min_overlap=0.028,
        elem_a=stanchion,
        elem_b=outer_shell,
        name="stanchion_overlaps_sleeve_footprint",
    )
    ctx.expect_contact(
        inner_post,
        clamp_base,
        elem_a=stanchion,
        elem_b=head_body,
        name="clamp_base_sits_on_post",
    )
    ctx.expect_contact(
        clamp_base,
        clamp_cap,
        elem_a=left_cheek,
        elem_b=pivot_barrel,
        name="left_pivot_cheek_contact",
    )
    ctx.expect_contact(
        clamp_base,
        clamp_cap,
        elem_a=right_cheek,
        elem_b=pivot_barrel,
        name="right_pivot_cheek_contact",
    )
    ctx.expect_gap(
        clamp_cap,
        clamp_base,
        axis="z",
        positive_elem=upper_cradle,
        negative_elem=lower_cradle,
        min_gap=0.003,
        name="rail_clamp_gap_at_rest",
    )

    sleeve_aabb = ctx.part_world_aabb(lower_sleeve)
    if sleeve_aabb is not None:
        sleeve_dx = sleeve_aabb[1][0] - sleeve_aabb[0][0]
        sleeve_dz = sleeve_aabb[1][2] - sleeve_aabb[0][2]
        ctx.check(
            "sleeve_realistic_scale",
            0.030 <= sleeve_dx <= 0.040 and 0.270 <= sleeve_dz <= 0.290,
            f"lower_sleeve extents were dx={sleeve_dx:.4f}, dz={sleeve_dz:.4f}",
        )
    else:
        ctx.fail("sleeve_realistic_scale", "lower_sleeve world AABB was unavailable")

    valve_origin = ctx.part_world_position(valve_boss)
    if valve_origin is not None:
        ctx.check(
            "valve_boss_side_position",
            valve_origin[0] > 0.015 and abs(valve_origin[1]) < 0.006 and 0.14 <= valve_origin[2] <= 0.23,
            f"valve_boss origin was {valve_origin}",
        )
    else:
        ctx.fail("valve_boss_side_position", "valve_boss world position was unavailable")

    def _center_of_aabb_z(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    travel_limits = post_travel.motion_limits
    assert travel_limits is not None
    assert travel_limits.upper is not None
    clamp_base_rest = ctx.part_world_position(clamp_base)
    clamp_base_high = None
    with ctx.pose({post_travel: travel_limits.upper}):
        clamp_base_high = ctx.part_world_position(clamp_base)
        ctx.fail_if_parts_overlap_in_current_pose(name="post_travel_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="post_travel_upper_no_floating")
        ctx.expect_origin_distance(
            lower_sleeve,
            inner_post,
            axes="xy",
            max_dist=0.0005,
            name="stanchion_concentric_with_sleeve_at_extension",
        )
        ctx.expect_contact(
            lower_sleeve,
            inner_post,
            elem_a=right_guide_pad,
            elem_b=stanchion,
            name="right_guide_pad_contacts_stanchion_at_extension",
        )
        ctx.expect_gap(
            clamp_cap,
            clamp_base,
            axis="z",
            positive_elem=upper_cradle,
            negative_elem=lower_cradle,
            min_gap=0.001,
            name="rail_clamp_gap_at_extension",
        )
    if clamp_base_rest is not None and clamp_base_high is not None:
        z_delta = clamp_base_high[2] - clamp_base_rest[2]
        ctx.check(
            "post_travel_moves_full_stroke",
            abs(z_delta - travel_limits.upper) <= 1e-5,
            f"expected {travel_limits.upper:.5f} m of travel but measured {z_delta:.5f} m",
        )
    else:
        ctx.fail("post_travel_moves_full_stroke", "clamp_base world positions were unavailable")

    pitch_limits = clamp_pitch.motion_limits
    assert pitch_limits is not None
    assert pitch_limits.lower is not None
    assert pitch_limits.upper is not None

    front_low = None
    rear_low = None
    with ctx.pose({clamp_pitch: pitch_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clamp_pitch_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="clamp_pitch_lower_no_floating")
        ctx.expect_contact(
            clamp_base,
            clamp_cap,
            elem_a=left_cheek,
            elem_b=pivot_barrel,
            name="left_pivot_contact_at_lower_pitch",
        )
        ctx.expect_contact(
            clamp_base,
            clamp_cap,
            elem_a=right_cheek,
            elem_b=pivot_barrel,
            name="right_pivot_contact_at_lower_pitch",
        )
        ctx.expect_gap(
            clamp_cap,
            clamp_base,
            axis="z",
            positive_elem=upper_cradle,
            negative_elem=lower_cradle,
            min_gap=0.001,
            name="rail_clamp_gap_at_lower_pitch",
        )
        front_low = _center_of_aabb_z(ctx.part_element_world_aabb(clamp_cap, elem=front_bolt_head))
        rear_low = _center_of_aabb_z(ctx.part_element_world_aabb(clamp_cap, elem=rear_bolt_head))

    front_high = None
    rear_high = None
    with ctx.pose({clamp_pitch: pitch_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clamp_pitch_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="clamp_pitch_upper_no_floating")
        ctx.expect_contact(
            clamp_base,
            clamp_cap,
            elem_a=left_cheek,
            elem_b=pivot_barrel,
            name="left_pivot_contact_at_upper_pitch",
        )
        ctx.expect_contact(
            clamp_base,
            clamp_cap,
            elem_a=right_cheek,
            elem_b=pivot_barrel,
            name="right_pivot_contact_at_upper_pitch",
        )
        ctx.expect_gap(
            clamp_cap,
            clamp_base,
            axis="z",
            positive_elem=upper_cradle,
            negative_elem=lower_cradle,
            min_gap=0.001,
            name="rail_clamp_gap_at_upper_pitch",
        )
        front_high = _center_of_aabb_z(ctx.part_element_world_aabb(clamp_cap, elem=front_bolt_head))
        rear_high = _center_of_aabb_z(ctx.part_element_world_aabb(clamp_cap, elem=rear_bolt_head))

    if None not in (front_low, rear_low, front_high, rear_high):
        ctx.check(
            "clamp_pitch_changes_bolt_heights",
            front_high > front_low + 0.003 and rear_high < rear_low - 0.003,
            (
                "front/rear bolt head centers did not tilt as expected: "
                f"front {front_low:.4f}->{front_high:.4f}, "
                f"rear {rear_low:.4f}->{rear_high:.4f}"
            ),
        )
    else:
        ctx.fail(
            "clamp_pitch_changes_bolt_heights",
            "one or more bolt-head world AABBs were unavailable",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
