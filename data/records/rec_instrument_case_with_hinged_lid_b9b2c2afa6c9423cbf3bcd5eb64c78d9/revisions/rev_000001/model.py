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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_plate_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    thickness: float,
    corner_radius: float,
    center_y: float = 0.0,
    z0: float = 0.0,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, depth, corner_radius, corner_segments=8),
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geom.translate(0.0, center_y, z0 + thickness * 0.5)
    return _mesh(name, geom)


def _rounded_ring_mesh(
    name: str,
    *,
    outer_width: float,
    outer_depth: float,
    wall: float,
    height: float,
    corner_radius: float,
    center_y: float = 0.0,
    z0: float = 0.0,
):
    inner_width = outer_width - 2.0 * wall
    inner_depth = outer_depth - 2.0 * wall
    inner_radius = max(0.002, corner_radius - wall)
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_depth, corner_radius, corner_segments=8),
        [rounded_rect_profile(inner_width, inner_depth, inner_radius, corner_segments=8)],
        height,
        cap=True,
        center=True,
        closed=True,
    )
    geom.translate(0.0, center_y, z0 + height * 0.5)
    return _mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="molded_equipment_case")

    shell_plastic = model.material("shell_plastic", rgba=(0.15, 0.16, 0.17, 1.0))
    shell_highlight = model.material("shell_highlight", rgba=(0.22, 0.23, 0.24, 1.0))
    hardware = model.material("hardware", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.25, 0.27, 0.29, 1.0))
    handle_rubber = model.material("handle_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    wall = 0.008
    lower_width = 0.54
    lower_depth = 0.36
    lower_height = 0.115
    lid_width = 0.56
    lid_depth = 0.38
    lid_height = 0.062
    hinge_radius = 0.007
    hinge_y = -0.184
    hinge_z = 0.110
    latch_x = 0.165
    latch_pivot_y = 0.192
    latch_pivot_z = 0.083

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        _rounded_ring_mesh(
            "lower_shell_ring",
            outer_width=lower_width,
            outer_depth=lower_depth,
            wall=wall,
            height=lower_height,
            corner_radius=0.034,
            center_y=0.0,
            z0=0.0,
        ),
        material=shell_plastic,
        name="lower_shell",
    )
    lower_shell.visual(
        _rounded_plate_mesh(
            "lower_floor_plate",
            width=lower_width,
            depth=lower_depth,
            thickness=wall,
            corner_radius=0.034,
            center_y=0.0,
            z0=0.0,
        ),
        material=shell_plastic,
        name="lower_floor",
    )
    lower_shell.visual(
        Box((0.42, 0.20, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=shell_highlight,
        name="bottom_rib",
    )
    lower_shell.visual(
        Box((0.46, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.171, 0.101)),
        material=shell_highlight,
        name="front_rim",
    )
    lower_shell.visual(
        Box((0.030, 0.012, 0.052)),
        origin=Origin(xyz=(-0.115, 0.176, 0.026)),
        material=dark_hardware,
        name="left_handle_mount",
    )
    lower_shell.visual(
        Box((0.030, 0.012, 0.052)),
        origin=Origin(xyz=(0.115, 0.176, 0.026)),
        material=dark_hardware,
        name="right_handle_mount",
    )
    for name, x_center, x_sign in (
        ("lower_hinge_left", -0.175, -1.0),
        ("lower_hinge_center", 0.0, 0.0),
        ("lower_hinge_right", 0.175, 1.0),
    ):
        length = 0.13 if x_sign else 0.08
        lower_shell.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(xyz=(x_center, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware,
            name=name,
        )

    for side_name, x_center in (("left", -latch_x), ("right", latch_x)):
        lower_shell.visual(
            Box((0.068, 0.016, 0.034)),
            origin=Origin(xyz=(x_center, 0.186, 0.060)),
            material=shell_highlight,
            name=f"{side_name}_latch_base",
        )
        lower_shell.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(x_center - 0.018, latch_pivot_y, latch_pivot_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_hardware,
            name=f"{side_name}_latch_boss_outboard",
        )
        lower_shell.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(x_center + 0.018, latch_pivot_y, latch_pivot_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_hardware,
            name=f"{side_name}_latch_boss_inboard",
        )

    lid = model.part("lid")
    lid.visual(
        _rounded_plate_mesh(
            "lid_top_plate",
            width=lid_width,
            depth=lid_depth,
            thickness=wall,
            corner_radius=0.036,
            center_y=0.197,
            z0=0.059,
        ),
        material=shell_plastic,
        name="lid_top",
    )
    lid.visual(
        Box((wall, lid_depth - 0.024, lid_height)),
        origin=Origin(xyz=(-(lid_width * 0.5) + (wall * 0.5), 0.200, 0.036)),
        material=shell_plastic,
        name="left_sidewall",
    )
    lid.visual(
        Box((wall, lid_depth - 0.024, lid_height)),
        origin=Origin(xyz=((lid_width * 0.5) - (wall * 0.5), 0.200, 0.036)),
        material=shell_plastic,
        name="right_sidewall",
    )
    lid.visual(
        Box((lid_width, wall, lid_height)),
        origin=Origin(xyz=(0.0, 0.382, 0.036)),
        material=shell_plastic,
        name="front_wall",
    )
    lid.visual(
        Box((lid_width, wall, lid_height - 0.003)),
        origin=Origin(xyz=(0.0, 0.019, 0.0295)),
        material=shell_plastic,
        name="rear_wall",
    )
    lid.visual(
        Box((0.20, 0.020, 0.016)),
        origin=Origin(xyz=(-0.14, 0.140, 0.061)),
        material=shell_highlight,
        name="left_inner_rib",
    )
    lid.visual(
        Box((0.20, 0.020, 0.016)),
        origin=Origin(xyz=(0.14, 0.140, 0.061)),
        material=shell_highlight,
        name="right_inner_rib",
    )
    lid.visual(
        Box((0.30, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.050, 0.060)),
        material=shell_highlight,
        name="center_inner_rib",
    )
    lid.visual(
        Box((0.50, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, 0.016)),
        material=hardware,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.07),
        origin=Origin(xyz=(-0.075, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="hinge_knuckle_left",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.07),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="hinge_knuckle_right",
    )
    lid.visual(
        Box((0.064, 0.004, 0.020)),
        origin=Origin(xyz=(-latch_x, 0.388, 0.022)),
        material=dark_hardware,
        name="left_strike",
    )
    lid.visual(
        Box((0.064, 0.004, 0.020)),
        origin=Origin(xyz=(latch_x, 0.388, 0.022)),
        material=dark_hardware,
        name="right_strike",
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.026, 0.010, 0.050)),
        origin=Origin(xyz=(-0.115, 0.000, 0.025)),
        material=dark_hardware,
        name="left_anchor",
    )
    handle.visual(
        Box((0.026, 0.010, 0.050)),
        origin=Origin(xyz=(0.115, 0.000, 0.025)),
        material=dark_hardware,
        name="right_anchor",
    )
    handle.visual(
        _mesh(
            "handle_grip_loop",
            tube_from_spline_points(
                [
                    (-0.115, 0.010, 0.048),
                    (-0.095, 0.030, 0.032),
                    (-0.055, 0.046, 0.014),
                    (0.055, 0.046, 0.014),
                    (0.095, 0.030, 0.032),
                    (0.115, 0.010, 0.048),
                ],
                radius=0.008,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=handle_rubber,
        name="grip_loop",
    )

    def _build_latch(name: str):
        latch = model.part(name)
        latch.visual(
            Cylinder(radius=0.006, length=0.028),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_hardware,
            name="barrel",
        )
        latch.visual(
            Box((0.046, 0.004, 0.050)),
            origin=Origin(xyz=(0.0, 0.008, 0.008)),
            material=hardware,
            name="body",
        )
        latch.visual(
            Box((0.046, 0.004, 0.014)),
            origin=Origin(xyz=(0.0, 0.012, 0.040)),
            material=dark_hardware,
            name="hook",
        )
        latch.visual(
            Box((0.046, 0.012, 0.014)),
            origin=Origin(xyz=(0.0, 0.016, 0.004)),
            material=hardware,
            name="finger_pull",
        )
        return latch

    left_latch = _build_latch("left_latch")
    right_latch = _build_latch("right_latch")

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.55),
    )
    model.articulation(
        "handle_mount",
        ArticulationType.FIXED,
        parent=lower_shell,
        child=handle,
        origin=Origin(xyz=(0.0, 0.187, 0.026)),
    )
    model.articulation(
        "left_draw_latch",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=left_latch,
        origin=Origin(xyz=(-latch_x, latch_pivot_y, latch_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.25, upper=0.0),
    )
    model.articulation(
        "right_draw_latch",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=right_latch,
        origin=Origin(xyz=(latch_x, latch_pivot_y, latch_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.25, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    def require_part(name: str):
        try:
            part = object_model.get_part(name)
        except Exception as exc:  # pragma: no cover - defensive authored test path
            ctx.fail(f"{name}_present", str(exc))
            return None
        ctx.check(f"{name}_present", True)
        return part

    def require_joint(name: str):
        try:
            articulation = object_model.get_articulation(name)
        except Exception as exc:  # pragma: no cover - defensive authored test path
            ctx.fail(f"{name}_present", str(exc))
            return None
        ctx.check(f"{name}_present", True)
        return articulation

    lower_shell = require_part("lower_shell")
    lid = require_part("lid")
    handle = require_part("handle")
    left_latch = require_part("left_latch")
    right_latch = require_part("right_latch")
    rear_hinge = require_joint("rear_hinge")
    left_draw_latch = require_joint("left_draw_latch")
    right_draw_latch = require_joint("right_draw_latch")

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

    if None in (
        lower_shell,
        lid,
        handle,
        left_latch,
        right_latch,
        rear_hinge,
        left_draw_latch,
        right_draw_latch,
    ):
        return ctx.report()

    rear_limits = rear_hinge.motion_limits
    left_limits = left_draw_latch.motion_limits
    right_limits = right_draw_latch.motion_limits

    ctx.check("rear_hinge_axis_x", rear_hinge.axis == (1.0, 0.0, 0.0), f"axis={rear_hinge.axis}")
    ctx.check(
        "rear_hinge_limits",
        rear_limits is not None and rear_limits.lower == 0.0 and rear_limits.upper == 1.55,
        f"limits={rear_limits}",
    )
    ctx.check("left_latch_axis_x", left_draw_latch.axis == (1.0, 0.0, 0.0), f"axis={left_draw_latch.axis}")
    ctx.check("right_latch_axis_x", right_draw_latch.axis == (1.0, 0.0, 0.0), f"axis={right_draw_latch.axis}")
    ctx.check(
        "left_latch_limits",
        left_limits is not None and left_limits.lower == -1.25 and left_limits.upper == 0.0,
        f"limits={left_limits}",
    )
    ctx.check(
        "right_latch_limits",
        right_limits is not None and right_limits.lower == -1.25 and right_limits.upper == 0.0,
        f"limits={right_limits}",
    )

    ctx.expect_contact(lid, lower_shell, elem_a="hinge_knuckle_left", elem_b="lower_hinge_left", name="lid_hinge_contact")
    ctx.expect_contact(
        handle,
        lower_shell,
        elem_a="left_anchor",
        elem_b="left_handle_mount",
        name="left_handle_anchor_mounted",
    )
    ctx.expect_contact(
        handle,
        lower_shell,
        elem_a="right_anchor",
        elem_b="right_handle_mount",
        name="right_handle_anchor_mounted",
    )
    ctx.expect_contact(
        left_latch,
        lower_shell,
        elem_a="barrel",
        elem_b="left_latch_boss_inboard",
        name="left_latch_pivot_supported",
    )
    ctx.expect_contact(
        right_latch,
        lower_shell,
        elem_a="barrel",
        elem_b="right_latch_boss_outboard",
        name="right_latch_pivot_supported",
    )
    ctx.expect_contact(left_latch, lid, elem_a="hook", elem_b="left_strike", name="left_latch_clamps_lid")
    ctx.expect_contact(right_latch, lid, elem_a="hook", elem_b="right_strike", name="right_latch_clamps_lid")
    ctx.expect_overlap(lid, lower_shell, axes="xy", min_overlap=0.30, name="lid_overlaps_case_plan")

    with ctx.pose({rear_hinge: 1.25}):
        strike_aabb = ctx.part_element_world_aabb(lid, elem="left_strike")
        strike_min_z = strike_aabb[0][2] if strike_aabb is not None else None
        ctx.check(
            "lid_swings_up_on_rear_hinge",
            strike_min_z is not None and strike_min_z > 0.34,
            f"left_strike_min_z={strike_min_z}",
        )

    with ctx.pose({left_draw_latch: -1.15, right_draw_latch: -1.15}):
        ctx.expect_gap(
            left_latch,
            lid,
            axis="y",
            positive_elem="hook",
            negative_elem="left_strike",
            min_gap=0.020,
            name="left_latch_releases_forward",
        )
        ctx.expect_gap(
            right_latch,
            lid,
            axis="y",
            positive_elem="hook",
            negative_elem="right_strike",
            min_gap=0.020,
            name="right_latch_releases_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
