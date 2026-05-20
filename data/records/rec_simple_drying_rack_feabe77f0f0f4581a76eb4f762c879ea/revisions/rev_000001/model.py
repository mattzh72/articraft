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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_body_frame(part, *, metal, polymer, elastomer) -> None:
    rail_half_span = 0.5475
    end_x = 0.57
    top_z = 0.795
    hinge_z = 0.826

    # Premium molded rail carriers define the seam strategy and tie the rack together.
    for x, name in ((-end_x, "front_carrier"), (end_x, "rear_carrier")):
        part.visual(
            Box((0.045, 0.31, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.805)),
            material=polymer,
            name=name,
        )
        part.visual(
            Box((0.052, 0.052, 0.032)),
            origin=Origin(xyz=(x, 0.160, 0.832)),
            material=polymer,
            name=f"{name}_left_hinge_seat",
        )
        part.visual(
            Box((0.052, 0.052, 0.032)),
            origin=Origin(xyz=(x, -0.160, 0.832)),
            material=polymer,
            name=f"{name}_right_hinge_seat",
        )

    # Main hanging deck.
    y_positions = (-0.102, -0.061, -0.020, 0.020, 0.061, 0.102)
    for index, y in enumerate(y_positions):
        part.visual(
            Cylinder(radius=0.0058, length=rail_half_span * 2.0),
            origin=Origin(xyz=(0.0, y, top_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"center_rail_{index}",
        )

    # Upper wing support rails / hinge lines.
    part.visual(
        Cylinder(radius=0.009, length=rail_half_span * 2.0),
        origin=Origin(xyz=(0.0, 0.160, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="left_hinge_rail",
    )
    part.visual(
        Cylinder(radius=0.009, length=rail_half_span * 2.0),
        origin=Origin(xyz=(0.0, -0.160, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="right_hinge_rail",
    )

    # Lower side rails stabilize the stance and add useful hanging area.
    part.visual(
        Cylinder(radius=0.0054, length=rail_half_span * 2.0),
        origin=Origin(xyz=(0.0, 0.118, 0.640), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="left_lower_rail",
    )
    part.visual(
        Cylinder(radius=0.0054, length=rail_half_span * 2.0),
        origin=Origin(xyz=(0.0, -0.118, 0.640), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="right_lower_rail",
    )

    # End support geometry.
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        for x_prefix, x in (("front", -end_x), ("rear", end_x)):
            foot = (x, side_sign * 0.240, 0.018)
            mid = (x, side_sign * 0.185, 0.340)
            top = (x, side_sign * 0.150, 0.790)
            _add_member(part, foot, mid, 0.0095, metal, name=f"{x_prefix}_{side_name}_lower_leg")
            _add_member(part, mid, top, 0.0088, metal, name=f"{x_prefix}_{side_name}_upper_leg")
            part.visual(
                Box((0.050, 0.032, 0.018)),
                origin=Origin(xyz=(x, side_sign * 0.240, 0.009)),
                material=elastomer,
                name=f"{x_prefix}_{side_name}_foot_pad",
            )
            rail_y = side_sign * 0.118
            rail_x = -rail_half_span if x_prefix == "front" else rail_half_span
            rail_point = (rail_x, rail_y, 0.640)
            brace_leg_point = (x, side_sign * 0.162, 0.640)
            _add_member(
                part,
                rail_point,
                brace_leg_point,
                0.0056,
                metal,
                name=f"{x_prefix}_{side_name}_lower_rail_brace",
            )

    _add_member(
        part,
        (-end_x, -0.185, 0.340),
        (-end_x, 0.185, 0.340),
        0.0068,
        metal,
        name="front_spread_bar",
    )
    _add_member(
        part,
        (end_x, -0.185, 0.340),
        (end_x, 0.185, 0.340),
        0.0068,
        metal,
        name="rear_spread_bar",
    )

    # Front and rear end caps across the full rack width.
    part.visual(
        Cylinder(radius=0.0072, length=0.320),
        origin=Origin(xyz=(-end_x, 0.0, 0.790), rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
        material=metal,
        name="front_top_cap",
    )
    part.visual(
        Cylinder(radius=0.0072, length=0.320),
        origin=Origin(xyz=(end_x, 0.0, 0.790), rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
        material=metal,
        name="rear_top_cap",
    )


def _build_hinge_link(part, *, metal, polymer, side_sign: float) -> None:
    del metal

    # Rebuilt as an honest offset strap rather than a solid concentric sleeve.
    # The top face seats against the underside of the fixed hinge rail without
    # occupying the same volume.
    part.visual(
        Box((0.980, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, side_sign * 0.019, -0.047)),
        material=polymer,
        name="mount_beam",
    )
    for x, name in ((-0.514, "front_mount_block"), (0.514, "rear_mount_block")):
        part.visual(
            Box((0.045, 0.022, 0.028)),
            origin=Origin(xyz=(math.copysign(0.570, x), side_sign * 0.006, -0.042)),
            material=polymer,
            name=name,
        )
        _add_member(
            part,
            (math.copysign(0.490, x), side_sign * 0.019, -0.042),
            (math.copysign(0.5475, x), side_sign * 0.006, -0.042),
            0.0042,
            polymer,
            name=f"{name}_bridge",
        )


def _build_wing(part, *, metal, polymer, side_sign: float) -> None:
    # Longitudinal rails.
    for name, y, z, radius, length in (
        ("inner_hanging_rail", side_sign * 0.072, 0.022, 0.0053, 1.040),
        ("mid_hanging_rail", side_sign * 0.128, 0.040, 0.0053, 1.040),
        ("outer_rail", side_sign * 0.190, 0.055, 0.0063, 1.050),
    ):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=name,
        )

    # Wing end brackets cleanly join the rails and give the premium molded look.
    for x, name in ((-0.535, "front_end_bracket"), (0.535, "rear_end_bracket")):
        part.visual(
            Box((0.080, 0.160, 0.084)),
            origin=Origin(xyz=(x - math.copysign(0.035, x), side_sign * 0.125, 0.042)),
            material=polymer,
            name=name,
        )
        part.visual(
            Box((0.024, 0.110, 0.034)),
            origin=Origin(xyz=(x - math.copysign(0.013, x), side_sign * 0.070, 0.018)),
            material=polymer,
            name=f"{name}_hinge_bridge",
        )

    # Central rib ties the hinge barrel into the hanging rails.
    part.visual(
        Box((0.920, 0.052, 0.064)),
        origin=Origin(xyz=(0.0, side_sign * 0.055, -0.006)),
        material=polymer,
        name="center_rib",
    )


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_drying_rack")

    painted_metal = model.material("painted_metal", rgba=(0.92, 0.92, 0.90, 1.0))
    soft_polymer = model.material("soft_polymer", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_elastomer = model.material("dark_elastomer", rgba=(0.16, 0.16, 0.17, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.76, 0.77, 0.78, 1.0))

    body_frame = model.part("body_frame")
    _build_body_frame(body_frame, metal=painted_metal, polymer=soft_polymer, elastomer=dark_elastomer)
    body_frame.inertial = Inertial.from_geometry(
        Box((1.18, 0.52, 0.86)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
    )

    left_hinge_link = model.part("left_hinge_link")
    _build_hinge_link(left_hinge_link, metal=satin_metal, polymer=soft_polymer, side_sign=1.0)
    left_hinge_link.inertial = Inertial.from_geometry(
        Box((1.04, 0.05, 0.06)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
    )

    right_hinge_link = model.part("right_hinge_link")
    _build_hinge_link(right_hinge_link, metal=satin_metal, polymer=soft_polymer, side_sign=-1.0)
    right_hinge_link.inertial = Inertial.from_geometry(
        Box((1.04, 0.05, 0.06)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
    )

    left_wing = model.part("left_wing")
    _build_wing(left_wing, metal=painted_metal, polymer=soft_polymer, side_sign=1.0)
    left_wing.inertial = Inertial.from_geometry(
        Box((1.12, 0.22, 0.08)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.095, 0.028)),
    )

    right_wing = model.part("right_wing")
    _build_wing(right_wing, metal=painted_metal, polymer=soft_polymer, side_sign=-1.0)
    right_wing.inertial = Inertial.from_geometry(
        Box((1.12, 0.22, 0.08)),
        mass=1.1,
        origin=Origin(xyz=(0.0, -0.095, 0.028)),
    )

    model.articulation(
        "body_to_left_hinge_link",
        ArticulationType.FIXED,
        parent=body_frame,
        child=left_hinge_link,
        origin=Origin(xyz=(0.0, 0.160, 0.844)),
    )
    model.articulation(
        "body_to_right_hinge_link",
        ArticulationType.FIXED,
        parent=body_frame,
        child=right_hinge_link,
        origin=Origin(xyz=(0.0, -0.160, 0.844)),
    )
    model.articulation(
        "left_wing_fold",
        ArticulationType.REVOLUTE,
        parent=left_hinge_link,
        child=left_wing,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "right_wing_fold",
        ArticulationType.REVOLUTE,
        parent=right_hinge_link,
        child=right_wing,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body_frame = object_model.get_part("body_frame")
    left_hinge_link = object_model.get_part("left_hinge_link")
    right_hinge_link = object_model.get_part("right_hinge_link")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    left_wing_fold = object_model.get_articulation("left_wing_fold")
    right_wing_fold = object_model.get_articulation("right_wing_fold")

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

    ctx.expect_contact(left_hinge_link, body_frame, name="left_hinge_link_mounts_to_body")
    ctx.expect_contact(right_hinge_link, body_frame, name="right_hinge_link_mounts_to_body")

    with ctx.pose({left_wing_fold: 0.0, right_wing_fold: 0.0}):
        ctx.expect_contact(left_wing, left_hinge_link, name="left_wing_supported_by_hinge_link")
        ctx.expect_contact(right_wing, right_hinge_link, name="right_wing_supported_by_hinge_link")
        ctx.expect_overlap(left_wing, left_hinge_link, axes="x", min_overlap=0.90, name="left_hinge_line_runs_along_rack")
        ctx.expect_overlap(right_wing, right_hinge_link, axes="x", min_overlap=0.90, name="right_hinge_line_runs_along_rack")

    with ctx.pose({left_wing_fold: 0.0}):
        open_aabb = ctx.part_element_world_aabb(left_wing, elem="outer_rail")
    with ctx.pose({left_wing_fold: left_wing_fold.motion_limits.upper}):
        folded_aabb = ctx.part_element_world_aabb(left_wing, elem="outer_rail")

    if open_aabb is None or folded_aabb is None:
        ctx.fail("left_wing_outer_rail_aabb_present", "Could not resolve left wing outer rail AABBs across poses.")
    else:
        open_center = _aabb_center(open_aabb)
        folded_center = _aabb_center(folded_aabb)
        ctx.check(
            "left_wing_folds_upward_to_stop",
            folded_center[2] > open_center[2] + 0.10 and folded_center[1] < open_center[1] - 0.15,
            (
                f"Expected left wing outer rail to rise and tuck inward. "
                f"Open center={open_center}, folded center={folded_center}."
            ),
        )

    with ctx.pose({right_wing_fold: 0.0}):
        open_aabb = ctx.part_element_world_aabb(right_wing, elem="outer_rail")
    with ctx.pose({right_wing_fold: right_wing_fold.motion_limits.upper}):
        folded_aabb = ctx.part_element_world_aabb(right_wing, elem="outer_rail")

    if open_aabb is None or folded_aabb is None:
        ctx.fail("right_wing_outer_rail_aabb_present", "Could not resolve right wing outer rail AABBs across poses.")
    else:
        open_center = _aabb_center(open_aabb)
        folded_center = _aabb_center(folded_aabb)
        ctx.check(
            "right_wing_folds_upward_to_stop",
            folded_center[2] > open_center[2] + 0.10 and folded_center[1] > open_center[1] + 0.15,
            (
                f"Expected right wing outer rail to rise and tuck inward. "
                f"Open center={open_center}, folded center={folded_center}."
            ),
        )

    ctx.warn_if_articulation_overlaps(max_pose_samples=12)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
