from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WALL_T = 0.020
SUPPORT_W = 0.300
SUPPORT_H = 0.320
FOOT_L = 0.140
FOOT_H = 0.020
BEAM_L = 0.340
BEAM_W = 0.190
BEAM_H = 0.040
BEAM_CENTER_Z = 0.120
GUSSET_T = 0.018
WEB_L = 0.180
WEB_W = 0.030
WEB_H = 0.085

LOWER_RAIL_L = 0.300
LOWER_RAIL_BASE_W = 0.080
LOWER_RAIL_BASE_H = 0.012
LOWER_RAIL_RIDGE_W = 0.000
LOWER_RAIL_RIDGE_H = 0.000
LOWER_RAIL_Y = 0.110

LOWER_HOME_X = 0.040
LOWER_TRAVEL = 0.120
LOWER_BODY_L = 0.180
LOWER_BODY_W = 0.190
LOWER_BODY_H = 0.060

UPPER_RAIL_BASE_X = 0.095
UPPER_RAIL_BASE_Y = 0.150
UPPER_RAIL_BASE_H = 0.010
UPPER_RAIL_RIDGE_X = 0.000
UPPER_RAIL_RIDGE_Y = 0.000
UPPER_RAIL_RIDGE_H = 0.000

UPPER_BODY_X = 0.110
UPPER_BODY_Y = 0.080
UPPER_BODY_H = 0.042
TOP_PLATE_X = 0.150
TOP_PLATE_Y = 0.100
TOP_PLATE_H = 0.012
UPPER_TRAVEL = 0.040


def _wp_box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _beam_top_z() -> float:
    return BEAM_CENTER_Z + (BEAM_H / 2.0)


def _lower_rail_top_z() -> float:
    return _beam_top_z() + LOWER_RAIL_BASE_H


def _upper_rail_top_z() -> float:
    return LOWER_BODY_H + UPPER_RAIL_BASE_H


def _make_side_gusset(y_center: float) -> cq.Workplane:
    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.000, FOOT_H),
                (0.000, SUPPORT_H * 0.76),
                (0.020, SUPPORT_H * 0.76),
                (0.125, _beam_top_z() - 0.004),
                (0.125, FOOT_H),
            ]
        )
        .close()
        .extrude(GUSSET_T)
    )
    return gusset.translate((0.0, y_center - (GUSSET_T / 2.0), 0.0))


def _make_support_frame() -> cq.Workplane:
    wall = _wp_box(0.220, WALL_T, SUPPORT_H, (0.110, -0.110, SUPPORT_H / 2.0))
    foot = _wp_box(0.180, 0.180, FOOT_H, (0.090, -0.065, FOOT_H / 2.0))
    rail_spine = _wp_box(0.220, 0.030, 0.022, (0.140, -0.030, _beam_top_z() - 0.011))
    front_rib = _wp_box(0.120, 0.014, 0.118, (0.095, -0.028, 0.079))
    rear_rib = _wp_box(0.120, 0.014, 0.118, (0.095, -0.072, 0.079))
    rail_root = _wp_box(
        0.026,
        LOWER_RAIL_Y + (LOWER_RAIL_BASE_W / 2.0),
        0.050,
        (0.017, (LOWER_RAIL_Y - (LOWER_RAIL_BASE_W / 2.0)) / 2.0, _beam_top_z() - 0.013),
    )
    return wall.union(foot).union(rail_spine).union(front_rib).union(rear_rib).union(rail_root)


def _make_lower_rail() -> cq.Workplane:
    return _wp_box(
        LOWER_RAIL_L,
        LOWER_RAIL_BASE_W,
        LOWER_RAIL_BASE_H,
        (
            0.180,
            LOWER_RAIL_Y,
            _beam_top_z() + (LOWER_RAIL_BASE_H / 2.0),
        ),
    )


def _make_lower_body() -> cq.Workplane:
    front_shoe = _wp_box(0.040, 0.070, 0.008, (0.034, 0.0, 0.004))
    rear_shoe = _wp_box(0.040, 0.070, 0.008, (LOWER_BODY_L - 0.034, 0.0, 0.004))
    rail_shoe = _wp_box(0.150, 0.060, 0.008, (LOWER_BODY_L / 2.0, LOWER_RAIL_Y, 0.004))
    main_body = _wp_box(LOWER_BODY_L, LOWER_BODY_W, 0.046, (LOWER_BODY_L / 2.0, 0.0, 0.031))
    bridge = _wp_box(0.110, 0.120, 0.014, (LOWER_BODY_L / 2.0, 0.0, 0.053))
    nose = _wp_box(0.028, 0.120, 0.010, (0.020, 0.0, 0.013))
    tail = _wp_box(0.028, 0.120, 0.010, (LOWER_BODY_L - 0.020, 0.0, 0.013))
    return (
        front_shoe
        .union(rear_shoe)
        .union(rail_shoe)
        .union(main_body)
        .union(bridge)
        .union(nose)
        .union(tail)
    )


def _make_upper_rail() -> cq.Workplane:
    rail_x = LOWER_BODY_L / 2.0
    base = _wp_box(
        0.052,
        0.140,
        UPPER_RAIL_BASE_H,
        (
            rail_x,
            0.0,
            LOWER_BODY_H + (UPPER_RAIL_BASE_H / 2.0),
        ),
    )
    return base.edges("|Y").chamfer(0.0015)


def _make_upper_body() -> cq.Workplane:
    guide = _wp_box(0.070, 0.030, 0.006, (0.0, 0.0, 0.003))
    base = _wp_box(UPPER_BODY_X, UPPER_BODY_Y, 0.036, (0.0, 0.0, 0.024))
    cap = _wp_box(0.084, 0.060, 0.010, (0.0, 0.0, 0.041))
    return guide.union(base).union(cap).edges("|Z").chamfer(0.0025)


def _make_top_plate() -> cq.Workplane:
    plate = _wp_box(
        TOP_PLATE_X,
        TOP_PLATE_Y,
        TOP_PLATE_H,
        (0.0, 0.0, UPPER_BODY_H + (TOP_PLATE_H / 2.0)),
    )
    return plate.edges("|Z").chamfer(0.003)


def _visual_center_axis(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis_index: int) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][axis_index] + aabb[1][axis_index]) / 2.0


def _max_limit_value(limits: MotionLimits | None) -> float | None:
    if limits is None:
        return None
    return limits.upper


def _min_limit_value(limits: MotionLimits | None) -> float | None:
    if limits is None:
        return None
    return limits.lower


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_xy_positioning_module")

    model.material("support_paint", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("rail_steel", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("carriage_gray", rgba=(0.58, 0.61, 0.65, 1.0))
    model.material("plate_silver", rgba=(0.82, 0.84, 0.87, 1.0))

    support = model.part("side_support")
    support.visual(
        mesh_from_cadquery(_make_support_frame(), "side_support_frame"),
        material="support_paint",
        name="support_frame",
    )
    support.visual(
        mesh_from_cadquery(_make_lower_rail(), "lower_slide_rail"),
        material="rail_steel",
        name="lower_rail",
    )
    support.inertial = Inertial.from_geometry(
        Box((BEAM_L, SUPPORT_W, SUPPORT_H)),
        mass=14.0,
        origin=Origin(xyz=(BEAM_L / 2.0, 0.0, SUPPORT_H / 2.0)),
    )

    lower = model.part("lower_carriage")
    lower.visual(
        mesh_from_cadquery(_make_lower_body(), "lower_carriage_body"),
        material="carriage_gray",
        name="lower_body",
    )
    lower.visual(
        mesh_from_cadquery(_make_upper_rail(), "upper_cross_rail"),
        material="rail_steel",
        name="upper_rail",
    )
    lower.inertial = Inertial.from_geometry(
        Box((LOWER_BODY_L, LOWER_BODY_W, _upper_rail_top_z())),
        mass=5.2,
        origin=Origin(xyz=(LOWER_BODY_L / 2.0, 0.0, _upper_rail_top_z() / 2.0)),
    )

    upper = model.part("upper_cross_slide")
    upper.visual(
        mesh_from_cadquery(_make_upper_body(), "upper_slide_body"),
        material="carriage_gray",
        name="upper_body",
    )
    upper.visual(
        mesh_from_cadquery(_make_top_plate(), "upper_top_plate"),
        material="plate_silver",
        name="top_plate",
    )
    upper.inertial = Inertial.from_geometry(
        Box((TOP_PLATE_X, TOP_PLATE_Y, UPPER_BODY_H + TOP_PLATE_H)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, (UPPER_BODY_H + TOP_PLATE_H) / 2.0)),
    )

    model.articulation(
        "support_to_lower_slide",
        ArticulationType.PRISMATIC,
        parent=support,
        child=lower,
        origin=Origin(xyz=(LOWER_HOME_X, 0.0, _lower_rail_top_z())),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LOWER_TRAVEL,
            effort=800.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "lower_to_upper_slide",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(LOWER_BODY_L / 2.0, 0.0, _upper_rail_top_z())),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-UPPER_TRAVEL,
            upper=UPPER_TRAVEL,
            effort=350.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("side_support")
    lower = object_model.get_part("lower_carriage")
    upper = object_model.get_part("upper_cross_slide")
    lower_slide = object_model.get_articulation("support_to_lower_slide")
    upper_slide = object_model.get_articulation("lower_to_upper_slide")
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

    ctx.expect_contact(lower, support, name="lower carriage is physically supported by the side rail")
    ctx.expect_contact(upper, lower, name="upper cross slide is physically supported by the lower carriage")

    ctx.check(
        "lower prismatic axis is horizontal x",
        tuple(round(v, 6) for v in lower_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={lower_slide.axis}",
    )
    ctx.check(
        "upper prismatic axis is horizontal y",
        tuple(round(v, 6) for v in upper_slide.axis) == (0.0, 1.0, 0.0),
        details=f"axis={upper_slide.axis}",
    )

    lower_home = ctx.part_world_position(lower)
    lower_max_q = _max_limit_value(lower_slide.motion_limits)
    if lower_home is None or lower_max_q is None:
        ctx.fail("lower slide travel can be measured", "missing lower carriage position or lower-slide limit")
    else:
        with ctx.pose({lower_slide: lower_max_q}):
            lower_out = ctx.part_world_position(lower)
        ctx.check(
            "lower carriage translates only along x",
            lower_out is not None
            and (lower_out[0] - lower_home[0]) > 0.10
            and abs(lower_out[1] - lower_home[1]) < 1e-6
            and abs(lower_out[2] - lower_home[2]) < 1e-6,
            details=f"home={lower_home}, out={lower_out}",
        )

    upper_min_q = _min_limit_value(upper_slide.motion_limits)
    upper_max_q = _max_limit_value(upper_slide.motion_limits)
    if upper_min_q is None or upper_max_q is None:
        ctx.fail("upper slide travel can be measured", "missing upper-slide limits")
    else:
        with ctx.pose({lower_slide: LOWER_TRAVEL / 2.0, upper_slide: upper_min_q}):
            upper_minus = ctx.part_world_position(upper)
        with ctx.pose({lower_slide: LOWER_TRAVEL / 2.0, upper_slide: upper_max_q}):
            upper_plus = ctx.part_world_position(upper)
        ctx.check(
            "upper carriage translates only along y",
            upper_minus is not None
            and upper_plus is not None
            and (upper_plus[1] - upper_minus[1]) > 0.075
            and abs(upper_plus[0] - upper_minus[0]) < 1e-6
            and abs(upper_plus[2] - upper_minus[2]) < 1e-6,
            details=f"minus={upper_minus}, plus={upper_plus}",
        )

        with ctx.pose({lower_slide: LOWER_TRAVEL / 2.0, upper_slide: upper_min_q}):
            ctx.expect_within(
                upper,
                lower,
                axes="x",
                inner_elem="top_plate",
                outer_elem="lower_body",
                name="top plate stays centered over the lower carriage in x at negative cross travel",
            )
            ctx.expect_within(
                upper,
                lower,
                axes="y",
                inner_elem="top_plate",
                outer_elem="lower_body",
                name="top plate stays over the lower carriage footprint at negative cross travel",
            )
            lower_body_aabb = ctx.part_element_world_aabb(lower, elem="lower_body")
            top_plate_aabb = ctx.part_element_world_aabb(upper, elem="top_plate")
            lower_center_x = _visual_center_axis(lower_body_aabb, 0)
            top_center_x = _visual_center_axis(top_plate_aabb, 0)
            ctx.check(
                "top plate x-center matches lower carriage at negative cross travel",
                lower_center_x is not None
                and top_center_x is not None
                and abs(top_center_x - lower_center_x) < 0.003,
                details=f"lower_center_x={lower_center_x}, top_center_x={top_center_x}",
            )

        with ctx.pose({lower_slide: LOWER_TRAVEL / 2.0, upper_slide: upper_max_q}):
            ctx.expect_within(
                upper,
                lower,
                axes="x",
                inner_elem="top_plate",
                outer_elem="lower_body",
                name="top plate stays centered over the lower carriage in x at positive cross travel",
            )
            ctx.expect_within(
                upper,
                lower,
                axes="y",
                inner_elem="top_plate",
                outer_elem="lower_body",
                name="top plate stays over the lower carriage footprint at positive cross travel",
            )
            lower_body_aabb = ctx.part_element_world_aabb(lower, elem="lower_body")
            top_plate_aabb = ctx.part_element_world_aabb(upper, elem="top_plate")
            lower_center_x = _visual_center_axis(lower_body_aabb, 0)
            top_center_x = _visual_center_axis(top_plate_aabb, 0)
            ctx.check(
                "top plate x-center matches lower carriage at positive cross travel",
                lower_center_x is not None
                and top_center_x is not None
                and abs(top_center_x - lower_center_x) < 0.003,
                details=f"lower_center_x={lower_center_x}, top_center_x={top_center_x}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
