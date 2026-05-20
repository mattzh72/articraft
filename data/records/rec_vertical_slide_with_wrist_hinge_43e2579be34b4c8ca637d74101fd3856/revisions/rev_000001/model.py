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


PLATE_W = 0.14
PLATE_H = 0.42
PLATE_T = 0.012
MOUNT_HOLE_D = 0.012

GUIDE_W = 0.052
GUIDE_D = 0.018
GUIDE_H = 0.30

CARRIAGE_W = 0.09
CARRIAGE_D = 0.032
CARRIAGE_H = 0.072
CARRIAGE_CHANNEL_D = GUIDE_D
CARRIAGE_HOME_Z = -0.10
CARRIAGE_Y = (PLATE_T / 2.0) + GUIDE_D
CARRIAGE_TRAVEL = 0.18

HINGE_BARREL_R = 0.008
HINGE_BARREL_L = 0.024
CHEEK_W = 0.012
CHEEK_D = 0.016
CHEEK_H = 0.028
CHEEK_CENTER_X = (HINGE_BARREL_L / 2.0) + (CHEEK_W / 2.0)
CHEEK_CENTER_Y = 0.032
HINGE_CENTER_Y = CHEEK_CENTER_Y
HINGE_CENTER_Z = 0.0

TAB_W = 0.024
TAB_L = 0.060
TAB_T = 0.010
TAB_OPEN = 1.25


def _side_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_W, PLATE_T, PLATE_H)
    plate = (
        plate.faces(">Y")
        .workplane()
        .pushPoints(
            [
                (-0.040, -0.150),
                (0.040, -0.150),
                (-0.040, 0.150),
                (0.040, 0.150),
            ]
        )
        .hole(MOUNT_HOLE_D)
    )
    guide = cq.Workplane("XY").box(GUIDE_W, GUIDE_D, GUIDE_H).translate(
        (0.0, (PLATE_T / 2.0) + (GUIDE_D / 2.0), 0.0)
    )
    upper_cap = cq.Workplane("XY").box(0.082, 0.012, 0.020).translate((0.0, 0.015, 0.152))
    lower_cap = cq.Workplane("XY").box(0.082, 0.012, 0.020).translate((0.0, 0.015, -0.152))
    return plate.union(guide).union(upper_cap).union(lower_cap)


def _carriage_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(CARRIAGE_W, 0.024, CARRIAGE_H).translate((0.0, 0.012, 0.0))
    rear_runner_top = cq.Workplane("XY").box(0.040, 0.010, 0.020).translate((0.0, 0.005, 0.018))
    rear_runner_bottom = cq.Workplane("XY").box(0.040, 0.010, 0.020).translate((0.0, 0.005, -0.018))
    left_cheek = cq.Workplane("XY").box(CHEEK_W, CHEEK_D, CHEEK_H).translate(
        (CHEEK_CENTER_X, CHEEK_CENTER_Y, 0.0)
    )
    right_cheek = cq.Workplane("XY").box(CHEEK_W, CHEEK_D, CHEEK_H).translate(
        (-CHEEK_CENTER_X, CHEEK_CENTER_Y, 0.0)
    )
    left_web = cq.Workplane("XY").box(0.018, 0.012, CHEEK_H).translate((CHEEK_CENTER_X, 0.022, 0.0))
    right_web = cq.Workplane("XY").box(0.018, 0.012, CHEEK_H).translate((-CHEEK_CENTER_X, 0.022, 0.0))
    hinge_saddle = cq.Workplane("XY").box(0.028, 0.010, 0.018).translate((0.0, 0.022, 0.0))
    return (
        body.union(rear_runner_top)
        .union(rear_runner_bottom)
        .union(left_cheek)
        .union(right_cheek)
        .union(left_web)
        .union(right_web)
        .union(hinge_saddle)
    )


def _wrist_tab_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("YZ")
        .circle(HINGE_BARREL_R)
        .extrude(HINGE_BARREL_L)
        .translate((-(HINGE_BARREL_L / 2.0), 0.0, 0.0))
    )
    neck = cq.Workplane("XY").box(0.014, 0.018, TAB_T).translate((0.0, 0.013, 0.0))
    blade = cq.Workplane("XY").box(TAB_W, 0.044, TAB_T).translate((0.0, 0.043, 0.0))
    tip = cq.Workplane("YZ").circle(TAB_T / 2.0).extrude(TAB_W).translate((-(TAB_W / 2.0), 0.067, 0.0))
    return barrel.union(neck).union(blade).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_vertical_axis")

    model.material("plate_steel", rgba=(0.55, 0.58, 0.61, 1.0))
    model.material("carriage_polymer", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("tab_alloy", rgba=(0.46, 0.50, 0.57, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_side_plate_shape(), "side_plate"),
        material="plate_steel",
        name="plate_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        material="carriage_polymer",
        name="carriage_shell",
    )

    wrist_tab = model.part("wrist_tab")
    wrist_tab.visual(
        mesh_from_cadquery(_wrist_tab_shape(), "wrist_tab"),
        material="tab_alloy",
        name="tab_shell",
    )

    model.articulation(
        "side_plate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=carriage,
        origin=Origin(xyz=(0.0, CARRIAGE_Y, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=180.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "carriage_to_wrist_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist_tab,
        origin=Origin(xyz=(0.0, HINGE_CENTER_Y, HINGE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TAB_OPEN,
            effort=25.0,
            velocity=1.5,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    carriage = object_model.get_part("carriage")
    wrist_tab = object_model.get_part("wrist_tab")
    lift = object_model.get_articulation("side_plate_to_carriage")
    wrist = object_model.get_articulation("carriage_to_wrist_tab")

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
    ctx.allow_overlap(
        carriage,
        wrist_tab,
        reason="Simplified hinge representation uses a captured coaxial barrel at the carriage clevis, so the compact wrist-tab hinge intentionally shares the hinge envelope in the closed pose.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(carriage, side_plate, name="carriage_is_supported_by_side_plate")
    ctx.expect_contact(wrist_tab, carriage, name="wrist_tab_is_hinged_to_carriage")

    lift_lower = 0.0 if lift.motion_limits is None or lift.motion_limits.lower is None else lift.motion_limits.lower
    lift_upper = 0.0 if lift.motion_limits is None or lift.motion_limits.upper is None else lift.motion_limits.upper
    wrist_upper = 0.0 if wrist.motion_limits is None or wrist.motion_limits.upper is None else wrist.motion_limits.upper

    with ctx.pose({lift: lift_upper}):
        ctx.expect_contact(
            carriage,
            side_plate,
            name="carriage_stays_supported_at_top_of_stroke",
        )

    with ctx.pose({lift: lift_upper, wrist: wrist_upper}):
        ctx.expect_contact(
            wrist_tab,
            carriage,
            name="wrist_hinge_stays_connected_when_open",
        )

    with ctx.pose({lift: lift_lower}):
        low_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: lift_upper}):
        high_pos = ctx.part_world_position(carriage)

    carriage_moves_vertically = (
        low_pos is not None
        and high_pos is not None
        and abs((high_pos[2] - low_pos[2]) - (lift_upper - lift_lower)) <= 1e-4
        and abs(high_pos[0] - low_pos[0]) <= 1e-5
        and abs(high_pos[1] - low_pos[1]) <= 1e-5
    )
    ctx.check(
        "carriage_prismatic_motion_is_vertical",
        carriage_moves_vertically,
        details=f"low={low_pos}, high={high_pos}, expected_dz={lift_upper - lift_lower}",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

    with ctx.pose({wrist: 0.0}):
        closed_center = _aabb_center(ctx.part_element_world_aabb(wrist_tab, elem="tab_shell"))
    with ctx.pose({wrist: wrist_upper}):
        open_center = _aabb_center(ctx.part_element_world_aabb(wrist_tab, elem="tab_shell"))

    tab_pitches_up = (
        closed_center is not None
        and open_center is not None
        and open_center[2] > closed_center[2] + 0.010
        and abs(open_center[0] - closed_center[0]) <= 1e-5
    )
    ctx.check(
        "wrist_tab_opens_upward",
        tab_pitches_up,
        details=f"closed_center={closed_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
