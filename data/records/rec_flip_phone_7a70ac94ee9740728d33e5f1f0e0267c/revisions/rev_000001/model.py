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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_WIDTH = 0.068
BODY_HEIGHT = 0.078
BODY_THICKNESS = 0.008
BODY_RADIUS = 0.009

GLASS_WIDTH = 0.060
GLASS_HEIGHT = 0.070
GLASS_THICKNESS = 0.0008
GLASS_RADIUS = 0.006

ACTIVE_WIDTH = 0.052
ACTIVE_HEIGHT = 0.060
ACTIVE_THICKNESS = 0.0002
ACTIVE_RADIUS = 0.004

HINGE_STRAP_WIDTH = 0.016
HINGE_STRAP_HEIGHT = 0.058
HINGE_STRAP_THICKNESS = 0.0016
HINGE_KNUCKLE_RADIUS = 0.0046
HINGE_KNUCKLE_LENGTH = 0.0016

LOWER_SHELL_CENTER = (-0.042, 0.0, BODY_THICKNESS * 0.5)
LOWER_HINGE_STRAP_CENTER = (-0.004, 0.0, 0.0092)
LOWER_HINGE_KNUCKLE_CENTER = (0.0, 0.0, 0.0092)

UPPER_SHELL_CENTER = (0.042, 0.0, 0.0044)
UPPER_HINGE_STRAP_CENTER = (0.007, 0.0, 0.0016)
UPPER_HINGE_KNUCKLE_CENTER = (0.0, 0.0, 0.0008)

HINGE_LIMIT_LOWER = 0.0
HINGE_LIMIT_UPPER = math.pi


def _rounded_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    radius: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, height, radius),
            thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        name,
    )


def _add_phone_half(
    part,
    *,
    shell_center: tuple[float, float, float],
    shell_mesh,
    glass_mesh,
    active_mesh,
    shell_material: str,
    glass_material: str,
    active_material: str,
    include_earpiece: bool,
) -> None:
    shell_top_z = shell_center[2] + BODY_THICKNESS * 0.5
    glass_center = (shell_center[0], shell_center[1], shell_top_z + GLASS_THICKNESS * 0.5)
    active_center = (
        shell_center[0],
        shell_center[1],
        shell_top_z + GLASS_THICKNESS - ACTIVE_THICKNESS * 0.5,
    )

    part.visual(
        shell_mesh,
        origin=Origin(xyz=shell_center),
        material=shell_material,
        name="shell",
    )
    part.visual(
        glass_mesh,
        origin=Origin(xyz=glass_center),
        material=glass_material,
        name="display_face",
    )
    part.visual(
        active_mesh,
        origin=Origin(xyz=active_center),
        material=active_material,
        name="active_display",
    )
    if include_earpiece:
        part.visual(
            Box((0.014, 0.0022, 0.0002)),
            origin=Origin(
                xyz=(
                    shell_center[0],
                    shell_center[1] + 0.026,
                    shell_top_z + GLASS_THICKNESS - 0.0001,
                )
            ),
            material=shell_material,
            name="earpiece",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_screen_flip_phone")

    shell_material = model.material("shell_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    hinge_material = model.material("hinge_black", rgba=(0.09, 0.10, 0.11, 1.0))
    glass_material = model.material("glass_black", rgba=(0.08, 0.10, 0.12, 1.0))
    display_material = model.material("display_lit", rgba=(0.16, 0.34, 0.42, 1.0))

    shell_mesh = _rounded_panel_mesh(
        width=BODY_WIDTH,
        height=BODY_HEIGHT,
        thickness=BODY_THICKNESS,
        radius=BODY_RADIUS,
        name="phone_shell",
    )
    glass_mesh = _rounded_panel_mesh(
        width=GLASS_WIDTH,
        height=GLASS_HEIGHT,
        thickness=GLASS_THICKNESS,
        radius=GLASS_RADIUS,
        name="phone_glass",
    )
    active_mesh = _rounded_panel_mesh(
        width=ACTIVE_WIDTH,
        height=ACTIVE_HEIGHT,
        thickness=ACTIVE_THICKNESS,
        radius=ACTIVE_RADIUS,
        name="phone_active_display",
    )

    lower_body = model.part("lower_body")
    _add_phone_half(
        lower_body,
        shell_center=LOWER_SHELL_CENTER,
        shell_mesh=shell_mesh,
        glass_mesh=glass_mesh,
        active_mesh=active_mesh,
        shell_material=shell_material,
        glass_material=glass_material,
        active_material=display_material,
        include_earpiece=False,
    )
    lower_body.visual(
        Box((HINGE_STRAP_WIDTH, HINGE_STRAP_HEIGHT, HINGE_STRAP_THICKNESS)),
        origin=Origin(xyz=LOWER_HINGE_STRAP_CENTER),
        material=hinge_material,
        name="hinge_strap",
    )
    lower_body.visual(
        Cylinder(radius=HINGE_KNUCKLE_RADIUS, length=HINGE_KNUCKLE_LENGTH),
        origin=Origin(xyz=LOWER_HINGE_KNUCKLE_CENTER),
        material=hinge_material,
        name="hinge_knuckle",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((0.084, BODY_HEIGHT, 0.012)),
        mass=0.14,
        origin=Origin(xyz=(-0.034, 0.0, 0.006)),
    )

    upper_body = model.part("upper_body")
    _add_phone_half(
        upper_body,
        shell_center=UPPER_SHELL_CENTER,
        shell_mesh=shell_mesh,
        glass_mesh=glass_mesh,
        active_mesh=active_mesh,
        shell_material=shell_material,
        glass_material=glass_material,
        active_material=display_material,
        include_earpiece=True,
    )
    upper_body.visual(
        Box((HINGE_STRAP_WIDTH, HINGE_STRAP_HEIGHT, HINGE_STRAP_THICKNESS)),
        origin=Origin(xyz=UPPER_HINGE_STRAP_CENTER),
        material=hinge_material,
        name="hinge_strap",
    )
    upper_body.visual(
        Cylinder(radius=HINGE_KNUCKLE_RADIUS, length=HINGE_KNUCKLE_LENGTH),
        origin=Origin(xyz=UPPER_HINGE_KNUCKLE_CENTER),
        material=hinge_material,
        name="hinge_knuckle",
    )
    upper_body.inertial = Inertial.from_geometry(
        Box((0.080, BODY_HEIGHT, 0.012)),
        mass=0.12,
        origin=Origin(xyz=(0.036, 0.0, 0.006)),
    )

    model.articulation(
        "lower_to_upper_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=HINGE_LIMIT_LOWER,
            upper=HINGE_LIMIT_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    upper_body = object_model.get_part("upper_body")
    hinge = object_model.get_articulation("lower_to_upper_hinge")
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

    ctx.check(
        "phone_parts_present",
        lower_body is not None and upper_body is not None and hinge is not None,
        "expected lower_body, upper_body, and lower_to_upper_hinge to exist",
    )
    ctx.check(
        "hinge_axis_is_screen_normal",
        tuple(float(value) for value in hinge.axis) == (0.0, 0.0, 1.0),
        f"expected hinge axis (0, 0, 1), got {hinge.axis}",
    )

    limits = hinge.motion_limits
    ctx.check(
        "hinge_limits_match_flip_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - HINGE_LIMIT_LOWER) <= 1e-6
        and abs(limits.upper - HINGE_LIMIT_UPPER) <= 1e-6,
        f"unexpected hinge limits: {limits}",
    )

    def _elem_z_stats(part, elem_name: str) -> tuple[float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        center_z = 0.5 * (min_corner[2] + max_corner[2])
        span_z = max_corner[2] - min_corner[2]
        return center_z, span_z

    pose_checks = (
        ("open", HINGE_LIMIT_LOWER),
        ("half", math.pi * 0.5),
        ("closed", HINGE_LIMIT_UPPER),
    )
    for label, angle in pose_checks:
        with ctx.pose({hinge: angle}):
            ctx.expect_contact(
                upper_body,
                lower_body,
                elem_a="hinge_knuckle",
                elem_b="hinge_knuckle",
                name=f"{label}_hinge_knuckles_touch",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_pose_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_pose_no_floating")
            ctx.expect_gap(
                upper_body,
                lower_body,
                axis="z",
                min_gap=0.0090,
                max_gap=0.0106,
                positive_elem="display_face",
                negative_elem="display_face",
                name=f"{label}_screen_face_gap",
            )

            lower_screen_stats = _elem_z_stats(lower_body, "display_face")
            upper_screen_stats = _elem_z_stats(upper_body, "display_face")
            screens_flat = (
                lower_screen_stats is not None
                and upper_screen_stats is not None
                and lower_screen_stats[1] <= 0.0012
                and upper_screen_stats[1] <= 0.0012
            )
            ctx.check(
                f"{label}_screens_stay_flat_and_parallel",
                screens_flat,
                (
                    "expected both display faces to remain parallel to the world XY plane; "
                    f"got lower={lower_screen_stats}, upper={upper_screen_stats}"
                ),
            )

    with ctx.pose({hinge: HINGE_LIMIT_LOWER}):
        ctx.expect_gap(
            upper_body,
            lower_body,
            axis="x",
            min_gap=0.012,
            max_gap=0.020,
            positive_elem="shell",
            negative_elem="shell",
            name="open_pose_side_by_side_shell_spacing",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
