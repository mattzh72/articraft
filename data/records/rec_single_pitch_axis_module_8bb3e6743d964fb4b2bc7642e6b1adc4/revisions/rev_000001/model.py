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


BASE_W = 0.240
BASE_D = 0.140
BASE_T = 0.018

TOWER_GAP = 0.170
TOWER_T = 0.020
TOWER_D = 0.082
TOWER_H = 0.162
TOWER_HEAD_R = 0.030
CAP_R = 0.028
CAP_T = 0.006
CAP_BOLT_R = 0.004
CAP_BOLT_OFFSET_Y = 0.022
CAP_BOLT_OFFSET_Z = 0.021
RIB_T = 0.008
BORE_R = 0.0125

PIVOT_Z = 0.154

CRADLE_OUTER_W = 0.130
CRADLE_CHEEK_T = 0.016
CRADLE_INNER_W = CRADLE_OUTER_W - 2.0 * CRADLE_CHEEK_T
CRADLE_D = 0.072
CRADLE_H = 0.092
CRADLE_Z_CENTER = -0.058
CRADLE_WINDOW_D = 0.042
CRADLE_WINDOW_H = 0.050
CRADLE_BOSS_R = 0.020
TRUNNION_R = BORE_R
TRUNNION_T = TOWER_GAP / 2.0 - CRADLE_OUTER_W / 2.0

SHAFT_R = 0.010
SHAFT_LEN = CRADLE_OUTER_W

TILT_LOWER = -0.78
TILT_UPPER = 0.78


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _make_base_plate() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_W, BASE_D, BASE_T).translate((0.0, 0.0, BASE_T / 2.0))
    base = base.edges("|Z").fillet(0.006)
    pocket = (
        cq.Workplane("XY")
        .box(0.112, 0.060, 0.006)
        .translate((0.0, 0.0, BASE_T - 0.003))
    )
    return base.cut(pocket)


def _make_right_tower() -> cq.Workplane:
    return _make_tower(1.0)


def _make_tower_yoke() -> cq.Workplane:
    return _make_tower(-1.0).union(_make_tower(1.0))


def _make_tower(side: float) -> cq.Workplane:
    inner_x = side * (TOWER_GAP / 2.0)
    tower_center_x = side * (TOWER_GAP / 2.0 + TOWER_T / 2.0)
    outer_face_x = side * (TOWER_GAP / 2.0 + TOWER_T)
    head_start_x = inner_x if side > 0.0 else inner_x - TOWER_T
    cap_start_x = outer_face_x if side > 0.0 else outer_face_x - CAP_T
    rib_center_x = side * 0.096

    pedestal = _box(
        (side * 0.102, 0.0, BASE_T + 0.018),
        (0.036, TOWER_D, 0.036),
    )
    upright = _box(
        (tower_center_x, 0.0, BASE_T + 0.036 + (TOWER_H - 0.036) / 2.0),
        (TOWER_T, 0.076, TOWER_H - 0.036),
    )
    head = (
        cq.Workplane("YZ", origin=(head_start_x, 0.0, 0.0))
        .center(0.0, PIVOT_Z)
        .circle(TOWER_HEAD_R)
        .extrude(TOWER_T)
    )
    cap = (
        cq.Workplane("YZ", origin=(cap_start_x, 0.0, 0.0))
        .center(0.0, PIVOT_Z)
        .circle(CAP_R)
        .extrude(CAP_T)
    )

    tower = pedestal.union(upright).union(head).union(cap)

    for bolt_y in (-CAP_BOLT_OFFSET_Y, CAP_BOLT_OFFSET_Y):
        for bolt_z in (-CAP_BOLT_OFFSET_Z, CAP_BOLT_OFFSET_Z):
            tower = tower.union(
                cq.Workplane("YZ", origin=(cap_start_x, 0.0, 0.0))
                .center(bolt_y, PIVOT_Z + bolt_z)
                .circle(CAP_BOLT_R)
                .extrude(CAP_T)
            )

    for y_center in (0.034, -0.034):
        rib = _box((rib_center_x, y_center, 0.086), (0.022, RIB_T, 0.100))
        tower = tower.union(rib)

    bore = (
        cq.Workplane("YZ", origin=(head_start_x, 0.0, 0.0))
        .center(0.0, PIVOT_Z)
        .circle(BORE_R)
        .extrude(TOWER_T)
    )
    return tower.cut(bore)


def _make_right_cheek() -> cq.Workplane:
    return _make_cheek(1.0)


def _make_cheek(side: float) -> cq.Workplane:
    cheek_center_x = side * (CRADLE_OUTER_W / 2.0 - CRADLE_CHEEK_T / 2.0)
    cheek = _box(
        (cheek_center_x, 0.0, CRADLE_Z_CENTER),
        (CRADLE_CHEEK_T, CRADLE_D, CRADLE_H),
    )
    cheek_window = _box(
        (cheek_center_x, 0.0, CRADLE_Z_CENTER - 0.010),
        (CRADLE_CHEEK_T + 0.002, CRADLE_WINDOW_D, CRADLE_WINDOW_H),
    )
    clamp_face = (
        cq.Workplane(
            "YZ",
            origin=(
                cheek_center_x - side * (CRADLE_CHEEK_T / 2.0),
                0.0,
                0.0,
            ),
        )
        .center(0.0, 0.0)
        .circle(CRADLE_BOSS_R)
        .extrude(CRADLE_CHEEK_T)
    )
    return cheek.cut(cheek_window).union(clamp_face)


def _make_cradle_frame() -> cq.Workplane:
    right_cheek = _make_cheek(1.0)
    left_cheek = _make_cheek(-1.0)

    top_front = _box((0.0, 0.030, -0.020), (CRADLE_INNER_W, 0.012, 0.012))
    top_rear = _box((0.0, -0.030, -0.020), (CRADLE_INNER_W, 0.012, 0.012))
    bottom_front = _box((0.0, 0.030, -0.098), (CRADLE_INNER_W, 0.012, 0.012))
    bottom_rear = _box((0.0, -0.030, -0.098), (CRADLE_INNER_W, 0.012, 0.012))
    shelf = _box((0.0, 0.0, -0.104), (CRADLE_INNER_W, 0.060, 0.006))
    back_face = _box((0.0, -0.032, -0.060), (CRADLE_INNER_W, 0.008, 0.056))
    front_lip = _box((0.0, 0.032, -0.076), (CRADLE_INNER_W, 0.008, 0.026))
    left_pad = _box((0.0, -0.015, -0.101), (CRADLE_INNER_W, 0.010, 0.010))
    right_pad = _box((0.0, 0.015, -0.101), (CRADLE_INNER_W, 0.010, 0.010))

    return (
        right_cheek.union(left_cheek)
        .union(top_front)
        .union(top_rear)
        .union(bottom_front)
        .union(bottom_rear)
        .union(shelf)
        .union(back_face)
        .union(front_lip)
        .union(left_pad)
        .union(right_pad)
    )


def _make_cross_shaft() -> cq.Workplane:
    shaft = (
        cq.Workplane("YZ", origin=(-SHAFT_LEN / 2.0, 0.0, 0.0))
        .center(0.0, 0.0)
        .circle(SHAFT_R)
        .extrude(SHAFT_LEN)
    )
    right_stub = (
        cq.Workplane("YZ", origin=(CRADLE_OUTER_W / 2.0, 0.0, 0.0))
        .center(0.0, 0.0)
        .circle(TRUNNION_R)
        .extrude(TRUNNION_T)
    )
    left_stub = (
        cq.Workplane("YZ", origin=(-CRADLE_OUTER_W / 2.0 - TRUNNION_T, 0.0, 0.0))
        .center(0.0, 0.0)
        .circle(TRUNNION_R)
        .extrude(TRUNNION_T)
    )
    return shaft.union(right_stub).union(left_stub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head")

    model.material("stand_dark", rgba=(0.15, 0.17, 0.20, 1.0))
    model.material("frame_gray", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("shaft_steel", rgba=(0.78, 0.80, 0.82, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_make_base_plate(), "tilt_head_base_plate"),
        material="stand_dark",
        name="base_plate",
    )
    stand.visual(
        mesh_from_cadquery(_make_tower(-1.0), "tilt_head_left_tower"),
        material="stand_dark",
        name="left_tower",
    )
    stand.visual(
        mesh_from_cadquery(_make_tower(1.0), "tilt_head_right_tower"),
        material="stand_dark",
        name="right_tower",
    )
    stand.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, PIVOT_Z + 0.040)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, (PIVOT_Z + 0.040) / 2.0)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(_make_cradle_frame(), "tilt_head_cradle_frame"),
        material="frame_gray",
        name="frame",
    )
    cradle.visual(
        mesh_from_cadquery(_make_cross_shaft(), "tilt_head_cross_shaft"),
        material="shaft_steel",
        name="cross_shaft",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((CRADLE_OUTER_W, CRADLE_D, CRADLE_H)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, CRADLE_Z_CENTER)),
    )

    model.articulation(
        "tilt_pitch",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.8,
            lower=TILT_LOWER,
            upper=TILT_UPPER,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    cradle = object_model.get_part("cradle")
    tilt = object_model.get_articulation("tilt_pitch")

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

    limits = tilt.motion_limits
    ctx.check(
        "tilt joint uses horizontal shaft axis",
        tuple(tilt.axis) == (1.0, 0.0, 0.0),
        f"axis={tilt.axis}",
    )
    ctx.check(
        "tilt limits bracket rest pose",
        limits is not None and limits.lower is not None and limits.upper is not None and limits.lower < 0.0 < limits.upper,
        f"limits={limits}",
    )
    ctx.expect_within(
        cradle,
        stand,
        axes="xy",
        inner_elem="cross_shaft",
        margin=0.003,
        name="cross shaft stays captured inside tower envelope",
    )
    ctx.expect_gap(
        cradle,
        stand,
        axis="z",
        negative_elem="base_plate",
        min_gap=0.028,
        name="cradle clears base in rest pose",
    )

    with ctx.pose({tilt: TILT_LOWER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no clipping at lower tilt stop")
        ctx.expect_gap(
            cradle,
            stand,
            axis="z",
            negative_elem="base_plate",
            min_gap=0.024,
            name="cradle clears base at lower tilt stop",
        )

    with ctx.pose({tilt: TILT_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no clipping at upper tilt stop")
        ctx.expect_gap(
            cradle,
            stand,
            axis="z",
            negative_elem="base_plate",
            min_gap=0.024,
            name="cradle clears base at upper tilt stop",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
