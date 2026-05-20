from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.26
BASE_D = 0.20
BASE_T = 0.02

MAST_W = 0.14
MAST_D = 0.10
MAST_H = 1.18
MAST_WALL = 0.008

GUIDE_W = 0.022
GUIDE_T = 0.008
GUIDE_H = 0.90
GUIDE_START_Z = 0.16
GUIDE_X = 0.048

CARRIAGE_W = 0.17
CARRIAGE_D = 0.058
CARRIAGE_H = 0.16
CARRIAGE_REAR_POCKET_W = 0.104
CARRIAGE_REAR_POCKET_D = 0.03
CARRIAGE_REAR_POCKET_H = 0.12
CARRIAGE_Y = 0.101
LIFT_LOWER_Z = 0.28
LIFT_TRAVEL = 0.68

SHOE_W = 0.032
SHOE_D = 0.014
SHOE_H = 0.112
SHOE_Y = -0.036

YOKE_W = 0.13
YOKE_PLATE_T = 0.01
YOKE_PLATE_D = 0.038
YOKE_PLATE_H = 0.085
YOKE_OUTER_FACE_X = YOKE_W / 2.0
YOKE_PLATE_CENTER_X = YOKE_OUTER_FACE_X - YOKE_PLATE_T / 2.0

YOKE_BLOCK_D = 0.03
YOKE_BLOCK_H = 0.07
YOKE_BLOCK_Y = 0.044
YOKE_BLOCK_Z = 0.006

WRIST_Y = 0.155
WRIST_Z = 0.012

HINGE_CORE_R = 0.0125
HINGE_HOLE_R = 0.0132
HINGE_COLLAR_R = 0.016
HINGE_COLLAR_T = 0.008
BRACKET_BARREL_L = 0.036

def _box(
    sx: float,
    sy: float,
    sz: float,
    *,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    centered: tuple[bool, bool, bool] = (True, True, True),
) -> cq.Workplane:
    solid = cq.Workplane("XY").box(sx, sy, sz, centered=centered)
    if center != (0.0, 0.0, 0.0):
        solid = solid.translate(center)
    return solid


def _cyl_x(radius: float, length: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate(
        (center[0] - length / 2.0, center[1], center[2])
    )


def _union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _make_mast_body() -> cq.Workplane:
    base = _box(BASE_W, BASE_D, BASE_T, centered=(True, True, False))

    outer_column = _box(MAST_W, MAST_D, MAST_H, centered=(True, True, False))
    inner_column = _box(
        MAST_W - 2.0 * MAST_WALL,
        MAST_D - 2.0 * MAST_WALL,
        MAST_H - MAST_WALL,
        centered=(True, True, False),
    ).translate((0.0, 0.0, MAST_WALL))
    column = outer_column.cut(inner_column).translate((0.0, 0.0, BASE_T))

    top_cap = _box(
        MAST_W + 0.014,
        MAST_D + 0.014,
        0.012,
        centered=(True, True, False),
    ).translate((0.0, 0.0, BASE_T + MAST_H))

    upper_stop = _box(
        0.052,
        0.016,
        0.028,
        center=(0.0, MAST_D / 2.0 + 0.008, BASE_T + GUIDE_START_Z + GUIDE_H + 0.011),
    )

    return _union_all(base, column, top_cap, upper_stop)


def _make_mast_guides() -> cq.Workplane:
    z_center = BASE_T + GUIDE_START_Z + GUIDE_H / 2.0
    y_center = MAST_D / 2.0 + GUIDE_T / 2.0
    left = _box(GUIDE_W, GUIDE_T, GUIDE_H, center=(-GUIDE_X, y_center, z_center))
    right = _box(GUIDE_W, GUIDE_T, GUIDE_H, center=(GUIDE_X, y_center, z_center))
    return _union_all(left, right)


def _make_carriage_frame() -> cq.Workplane:
    back_plate = _box(0.156, 0.024, 0.156, center=(0.0, -0.022, 0.0))
    left_spine = _box(0.024, 0.052, 0.128, center=(-0.052, 0.004, 0.0))
    right_spine = _box(0.024, 0.052, 0.128, center=(0.052, 0.004, 0.0))
    top_web = _box(0.118, 0.018, 0.024, center=(0.0, 0.004, 0.056))
    bottom_web = _box(0.118, 0.018, 0.024, center=(0.0, 0.004, -0.056))
    front_stop = _box(0.076, 0.016, 0.024, center=(0.0, 0.056, -0.046))

    left_shoe = _box(SHOE_W, SHOE_D, SHOE_H, center=(-GUIDE_X, SHOE_Y, 0.0))
    right_shoe = _box(SHOE_W, SHOE_D, SHOE_H, center=(GUIDE_X, SHOE_Y, 0.0))

    left_plate = _box(
        YOKE_PLATE_T,
        0.022,
        0.074,
        center=(-0.058, WRIST_Y, WRIST_Z),
    ).cut(_cyl_x(HINGE_HOLE_R, 0.02, center=(-0.058, WRIST_Y, WRIST_Z)))
    right_plate = _box(
        YOKE_PLATE_T,
        0.022,
        0.074,
        center=(0.058, WRIST_Y, WRIST_Z),
    ).cut(_cyl_x(HINGE_HOLE_R, 0.02, center=(0.058, WRIST_Y, WRIST_Z)))

    left_upper_link = _box(0.018, 0.038, 0.022, center=(-0.048, 0.085, 0.044))
    right_upper_link = _box(0.018, 0.038, 0.022, center=(0.048, 0.085, 0.044))
    left_lower_link = _box(0.018, 0.038, 0.024, center=(-0.048, 0.085, -0.028))
    right_lower_link = _box(0.018, 0.038, 0.024, center=(0.048, 0.085, -0.028))

    return _union_all(
        back_plate,
        left_spine,
        right_spine,
        top_web,
        bottom_web,
        front_stop,
        left_shoe,
        right_shoe,
    )


def _make_carriage_clevis() -> cq.Workplane:
    left_tower = _box(0.024, 0.048, 0.112, center=(-0.05, 0.018, 0.0))
    right_tower = _box(0.024, 0.048, 0.112, center=(0.05, 0.018, 0.0))
    cross_tie = _box(0.088, 0.02, 0.024, center=(0.0, 0.052, -0.034))

    left_arm = _box(0.016, 0.118, 0.024, center=(-0.026, 0.084, -0.018))
    right_arm = _box(0.016, 0.118, 0.024, center=(0.026, 0.084, -0.018))
    left_upper_rib = _box(0.016, 0.072, 0.02, center=(-0.026, 0.092, 0.036))
    right_upper_rib = _box(0.016, 0.072, 0.02, center=(0.026, 0.092, 0.036))

    left_plate = _box(
        0.008,
        0.02,
        0.066,
        center=(-0.022, WRIST_Y, WRIST_Z),
    ).cut(_cyl_x(HINGE_HOLE_R, 0.016, center=(-0.022, WRIST_Y, WRIST_Z)))
    right_plate = _box(
        0.008,
        0.02,
        0.066,
        center=(0.022, WRIST_Y, WRIST_Z),
    ).cut(_cyl_x(HINGE_HOLE_R, 0.016, center=(0.022, WRIST_Y, WRIST_Z)))

    hinge_head_left = _cyl_x(0.006, 0.004, center=(-0.028, WRIST_Y, WRIST_Z))
    hinge_head_right = _cyl_x(0.006, 0.004, center=(0.028, WRIST_Y, WRIST_Z))
    upper_head_left = _cyl_x(0.005, 0.004, center=(-0.05, 0.03, 0.03))
    upper_head_right = _cyl_x(0.005, 0.004, center=(0.05, 0.03, 0.03))

    return _union_all(
        left_tower,
        right_tower,
        cross_tie,
        left_arm,
        right_arm,
        left_upper_rib,
        right_upper_rib,
        left_plate,
        right_plate,
        hinge_head_left,
        hinge_head_right,
        upper_head_left,
        upper_head_right,
    )


def _make_nose_bracket() -> cq.Workplane:
    arm_width = 0.03
    profile = [
        (0.01, -0.002),
        (0.024, -0.012),
        (0.084, -0.014),
        (0.132, -0.004),
        (0.132, 0.01),
        (0.11, 0.024),
        (0.02, 0.018),
        (0.01, 0.008),
    ]
    body = (
        cq.Workplane("YZ")
        .polyline(profile)
        .close()
        .extrude(arm_width)
        .translate((-arm_width / 2.0, 0.0, 0.0))
    )
    tongue = _box(0.018, 0.01, 0.022, center=(0.0, 0.008, 0.0))
    front_pad = _box(0.058, 0.014, 0.018, center=(0.0, 0.116, 0.001))
    underside_rib = _box(0.012, 0.03, 0.014, center=(0.0, 0.064, -0.013))
    return _union_all(body, tongue, front_pad, underside_rib)


def _make_nose_hinge() -> cq.Workplane:
    core = _cyl_x(
        HINGE_CORE_R,
        0.052,
        center=(0.0, 0.0, 0.0),
    )
    center_boss = _box(0.024, 0.014, 0.022, center=(0.0, 0.004, 0.0))
    left_washer = _cyl_x(HINGE_COLLAR_R, HINGE_COLLAR_T, center=(-0.03, 0.0, 0.0))
    right_washer = _cyl_x(HINGE_COLLAR_R, HINGE_COLLAR_T, center=(0.03, 0.0, 0.0))
    return _union_all(core, center_boss, left_washer, right_washer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_lift_with_wrist_nose")

    mast_steel = model.material("mast_steel", color=(0.29, 0.31, 0.34))
    guide_strip = model.material("guide_strip", color=(0.12, 0.12, 0.13))
    carriage_steel = model.material("carriage_steel", color=(0.43, 0.45, 0.48))
    hardware = model.material("hardware", color=(0.74, 0.76, 0.79))
    nose_steel = model.material("nose_steel", color=(0.22, 0.24, 0.27))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(_make_mast_body(), "mast_body"),
        material=mast_steel,
        name="body",
    )
    mast.visual(
        mesh_from_cadquery(_make_mast_guides(), "mast_guides"),
        material=guide_strip,
        name="guides",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_frame(), "carriage_body_v4"),
        material=carriage_steel,
        name="body",
    )
    carriage.visual(
        mesh_from_cadquery(_make_carriage_clevis(), "carriage_clevis_v4"),
        material=carriage_steel,
        name="clevis",
    )

    nose_bracket = model.part("nose_bracket")
    nose_bracket.visual(
        mesh_from_cadquery(_make_nose_bracket(), "nose_body_v4"),
        material=nose_steel,
        name="body",
    )
    nose_bracket.visual(
        mesh_from_cadquery(_make_nose_hinge(), "nose_hinge_v4"),
        material=hardware,
        name="hinge",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, CARRIAGE_Y, LIFT_LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=0.4,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_nose_bracket",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=nose_bracket,
        origin=Origin(xyz=(0.0, WRIST_Y, WRIST_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.4,
            lower=-0.75,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    nose_bracket = object_model.get_part("nose_bracket")
    lift = object_model.get_articulation("mast_to_carriage")
    wrist = object_model.get_articulation("carriage_to_nose_bracket")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        carriage,
        nose_bracket,
        elem_a="clevis",
        elem_b="hinge",
        reason="wrist shaft and thrust collars intentionally occupy the clevis bore as a shared hinge assembly",
    )

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
        "lift_is_vertical_prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC and tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"expected PRISMATIC on +Z, got {lift.articulation_type} with axis {lift.axis}",
    )
    ctx.check(
        "wrist_is_cross_shaft_revolute",
        wrist.articulation_type == ArticulationType.REVOLUTE and tuple(wrist.axis) == (1.0, 0.0, 0.0),
        details=f"expected REVOLUTE on +X, got {wrist.articulation_type} with axis {wrist.axis}",
    )

    with ctx.pose({lift: 0.0, wrist: 0.0}):
        ctx.expect_contact(
            carriage,
            mast,
            elem_a="body",
            elem_b="guides",
            name="lower_carriage_shoes_contact_guides",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="xz",
            elem_a="body",
            elem_b="guides",
            min_overlap=0.09,
            name="lower_carriage_remains_engaged_on_guides",
        )
        ctx.expect_contact(
            nose_bracket,
            carriage,
            elem_a="hinge",
            elem_b="clevis",
            name="wrist_barrel_is_supported_by_clevis",
        )
        ctx.expect_gap(
            nose_bracket,
            mast,
            axis="y",
            positive_elem="body",
            negative_elem="guides",
            min_gap=0.10,
            name="nose_bracket_clears_guides_in_rest_pose",
        )
        ctx.expect_within(
            nose_bracket,
            carriage,
            axes="x",
            inner_elem="body",
            margin=0.0,
            name="nose_bracket_stays_within_carriage_width",
        )

    with ctx.pose({lift: lift.motion_limits.upper, wrist: 0.0}):
        ctx.expect_contact(
            carriage,
            mast,
            elem_a="body",
            elem_b="guides",
            name="upper_carriage_shoes_contact_guides",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="xz",
            elem_a="body",
            elem_b="guides",
            min_overlap=0.09,
            name="upper_carriage_remains_engaged_on_guides",
        )

    with ctx.pose({lift: 0.0, wrist: wrist.motion_limits.lower}):
        ctx.expect_contact(
            nose_bracket,
            carriage,
            elem_a="hinge",
            elem_b="clevis",
            name="wrist_support_contact_at_lower_angle",
        )
        ctx.expect_gap(
            nose_bracket,
            mast,
            axis="y",
            positive_elem="body",
            negative_elem="guides",
            min_gap=0.095,
            name="nose_bracket_clears_guides_with_wrist_down",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_low_lift_wrist_down")

    with ctx.pose({lift: lift.motion_limits.upper, wrist: wrist.motion_limits.upper}):
        ctx.expect_contact(
            nose_bracket,
            carriage,
            elem_a="hinge",
            elem_b="clevis",
            name="wrist_support_contact_at_upper_angle",
        )
        ctx.expect_gap(
            nose_bracket,
            mast,
            axis="y",
            positive_elem="body",
            negative_elem="guides",
            min_gap=0.09,
            name="nose_bracket_clears_guides_with_wrist_up",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_high_lift_wrist_up")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
