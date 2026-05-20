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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _annulus_mesh(
    *,
    outer_size: float,
    inner_size: float,
    height: float,
    outer_radius: float,
    inner_radius: float,
    name: str,
):
    outer = rounded_rect_profile(outer_size, outer_size, outer_radius, corner_segments=8)
    inner = rounded_rect_profile(inner_size, inner_size, inner_radius, corner_segments=8)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            height=height,
            center=True,
        ),
        name,
    )


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _profile_loop(profile: list[tuple[float, float]], z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


def _cover_plate_mesh(name: str):
    base = _shift_profile(
        rounded_rect_profile(0.236, 0.228, 0.020, corner_segments=8),
        dy=0.102,
    )
    mid = _shift_profile(
        rounded_rect_profile(0.226, 0.218, 0.018, corner_segments=8),
        dy=0.102,
    )
    crown = _shift_profile(
        rounded_rect_profile(0.194, 0.184, 0.040, corner_segments=8),
        dy=0.102,
    )
    return mesh_from_geometry(
        section_loft(
            [
                _profile_loop(base, -0.007),
                _profile_loop(mid, -0.001),
                _profile_loop(crown, 0.010),
            ]
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_street_bollard")

    sleeve_grey = model.material("sleeve_grey", rgba=(0.34, 0.36, 0.38, 1.0))
    post_steel = model.material("post_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    cap_steel = model.material("cap_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    warning_red = model.material("warning_red", rgba=(0.70, 0.09, 0.06, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        _annulus_mesh(
            outer_size=0.220,
            inner_size=0.198,
            height=0.900,
            outer_radius=0.008,
            inner_radius=0.004,
            name="sleeve_shaft",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.450)),
        material=sleeve_grey,
        name="sleeve_shaft",
    )
    outer_sleeve.visual(
        _annulus_mesh(
            outer_size=0.300,
            inner_size=0.198,
            height=0.025,
            outer_radius=0.016,
            inner_radius=0.004,
            name="top_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=sleeve_grey,
        name="top_collar",
    )
    outer_sleeve.inertial = Inertial.from_geometry(
        Box((0.30, 0.30, 0.93)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, -0.4375)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        _annulus_mesh(
            outer_size=0.188,
            inner_size=0.152,
            height=1.100,
            outer_radius=0.006,
            inner_radius=0.003,
            name="post_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.550)),
        material=post_steel,
        name="post_shell",
    )
    inner_post.visual(
        _annulus_mesh(
            outer_size=0.190,
            inner_size=0.188,
            height=0.110,
            outer_radius=0.0065,
            inner_radius=0.006,
            name="reflective_band",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=warning_red,
        name="reflective_band",
    )
    inner_post.visual(
        Box((0.148, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, -0.082, 0.009)),
        material=post_steel,
        name="hinge_bridge",
    )
    inner_post.visual(
        Box((0.028, 0.018, 0.035)),
        origin=Origin(xyz=(-0.050, -0.085, 0.0175)),
        material=post_steel,
        name="lug_left",
    )
    inner_post.visual(
        Box((0.028, 0.018, 0.035)),
        origin=Origin(xyz=(0.050, -0.085, 0.0175)),
        material=post_steel,
        name="lug_right",
    )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.19, 0.19, 1.10)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, -0.550)),
    )

    dome_cap = model.part("dome_cap")
    dome_cap.visual(
        _cover_plate_mesh("cover_plate"),
        material=cap_steel,
        name="cover_plate",
    )
    dome_cap.visual(
        Box((0.138, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.010, -0.003)),
        material=cap_steel,
        name="hinge_strap",
    )
    dome_cap.visual(
        Cylinder(radius=0.007, length=0.112),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_dark,
        name="hinge_barrel",
    )
    dome_cap.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.03)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.120, 0.0)),
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.18,
            lower=0.0,
            upper=0.720,
        ),
    )
    model.articulation(
        "post_to_cap",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=dome_cap,
        origin=Origin(xyz=(0.0, -0.084, 0.042)),
        # The closed cover extends forward along +Y from the hinge barrel,
        # so +X raises the free edge upward for positive joint motion.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_sleeve = object_model.get_part("outer_sleeve")
    inner_post = object_model.get_part("inner_post")
    dome_cap = object_model.get_part("dome_cap")
    lift_joint = object_model.get_articulation("sleeve_to_post")
    cap_joint = object_model.get_articulation("post_to_cap")

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

    lift_upper = 0.0
    if lift_joint.motion_limits is not None and lift_joint.motion_limits.upper is not None:
        lift_upper = lift_joint.motion_limits.upper

    cap_upper = 0.0
    if cap_joint.motion_limits is not None and cap_joint.motion_limits.upper is not None:
        cap_upper = cap_joint.motion_limits.upper

    with ctx.pose({lift_joint: 0.0, cap_joint: 0.0}):
        ctx.expect_within(
            inner_post,
            outer_sleeve,
            axes="xy",
            inner_elem="post_shell",
            outer_elem="sleeve_shaft",
            margin=0.0,
            name="retracted post stays centered in the sleeve",
        )
        ctx.expect_overlap(
            inner_post,
            outer_sleeve,
            axes="z",
            elem_a="post_shell",
            elem_b="sleeve_shaft",
            min_overlap=0.35,
            name="retracted post remains deeply retained in the sleeve",
        )
        ctx.expect_contact(
            dome_cap,
            outer_sleeve,
            elem_a="cover_plate",
            elem_b="top_collar",
            name="closed cap rests on the top collar",
        )
        ctx.expect_overlap(
            dome_cap,
            outer_sleeve,
            axes="xy",
            elem_a="cover_plate",
            elem_b="sleeve_shaft",
            min_overlap=0.20,
            name="closed cap spans the sleeve opening",
        )
        ctx.expect_contact(
            dome_cap,
            inner_post,
            elem_a="hinge_barrel",
            elem_b="lug_left",
            name="left hinge lug supports the cap barrel",
        )
        ctx.expect_contact(
            dome_cap,
            inner_post,
            elem_a="hinge_barrel",
            elem_b="lug_right",
            name="right hinge lug supports the cap barrel",
        )

    rest_post_pos = ctx.part_world_position(inner_post)
    with ctx.pose({lift_joint: lift_upper, cap_joint: 0.0}):
        ctx.expect_within(
            inner_post,
            outer_sleeve,
            axes="xy",
            inner_elem="post_shell",
            outer_elem="sleeve_shaft",
            margin=0.0,
            name="extended post stays aligned with the sleeve",
        )
        ctx.expect_overlap(
            inner_post,
            outer_sleeve,
            axes="z",
            elem_a="post_shell",
            elem_b="sleeve_shaft",
            min_overlap=0.18,
            name="extended post still retains insertion in the sleeve",
        )
        extended_post_pos = ctx.part_world_position(inner_post)

    ctx.check(
        "post extends upward through the bollard stroke",
        rest_post_pos is not None
        and extended_post_pos is not None
        and extended_post_pos[2] > rest_post_pos[2] + 0.68,
        details=f"rest={rest_post_pos}, extended={extended_post_pos}",
    )

    with ctx.pose({lift_joint: lift_upper, cap_joint: 0.0}):
        closed_cover = ctx.part_element_world_aabb(dome_cap, elem="cover_plate")
    with ctx.pose({lift_joint: lift_upper, cap_joint: cap_upper}):
        opened_cover = ctx.part_element_world_aabb(dome_cap, elem="cover_plate")

    ctx.check(
        "cap swings upward on the hinge",
        closed_cover is not None
        and opened_cover is not None
        and opened_cover[1][2] > closed_cover[1][2] + 0.16,
        details=f"closed_cover={closed_cover}, opened_cover={opened_cover}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
