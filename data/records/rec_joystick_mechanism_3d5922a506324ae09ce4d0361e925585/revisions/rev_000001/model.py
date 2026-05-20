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


HOUSING_W = 0.086
HOUSING_D = 0.086
BASE_H = 0.018
BEZEL_W = 0.066
BEZEL_D = 0.066
BEZEL_H = 0.010
EAR_X = 0.028
EAR_T = 0.012
EAR_D = 0.028
EAR_H = 0.030
GIMBAL_Z = 0.039
HOUSING_TRUNNION_CLEAR_R = 0.00335

OUTER_X = 0.034
OUTER_Y = 0.034
OUTER_Z = 0.018
OUTER_WINDOW_X = 0.022
OUTER_WINDOW_Y = 0.022
OUTER_SHAFT_R = 0.0030
OUTER_SHAFT_LEN = 0.017
OUTER_COLLAR_R = 0.0046
OUTER_COLLAR_T = 0.0020
INNER_TRUNNION_CLEAR_R = 0.00255

INNER_X = 0.019
INNER_Y = 0.016
INNER_Z = 0.014
INNER_WINDOW_X = 0.010
INNER_WINDOW_Y = 0.007
INNER_SHAFT_R = 0.0023
INNER_SHAFT_LEN = 0.009
INNER_COLLAR_R = 0.0036
INNER_COLLAR_T = 0.0018
HUB_R = 0.0055
HUB_H = 0.010
HUB_TOP_Z = HUB_H / 2.0


def _make_ear() -> cq.Workplane:
    ear = cq.Workplane("XY").box(EAR_T, EAR_D, EAR_H, centered=(True, True, False))
    ear = ear.edges("|Z").fillet(0.0015)
    ear_hole = (
        cq.Workplane("YZ")
        .circle(HOUSING_TRUNNION_CLEAR_R)
        .extrude(EAR_T * 3.0, both=True)
        .translate((0.0, 0.0, GIMBAL_Z - BASE_H))
    )
    return ear.cut(ear_hole)


def make_housing() -> cq.Workplane:
    lower = cq.Workplane("XY").box(HOUSING_W, HOUSING_D, BASE_H, centered=(True, True, False))
    lower = lower.edges("|Z").fillet(0.005)

    bezel = cq.Workplane("XY").box(BEZEL_W, BEZEL_D, BEZEL_H, centered=(True, True, False))
    bezel = bezel.edges("|Z").fillet(0.003)
    bezel = bezel.translate((0.0, 0.0, BASE_H))

    left_ear = _make_ear().translate((EAR_X, 0.0, BASE_H))
    right_ear = _make_ear().translate((-EAR_X, 0.0, BASE_H))

    pocket = (
        cq.Workplane("XY")
        .box(0.044, 0.036, 0.018, centered=(True, True, False))
        .translate((0.0, 0.0, 0.010))
    )
    mouth = (
        cq.Workplane("XY")
        .box(0.050, 0.042, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, 0.022))
    )

    housing = lower.union(bezel).union(left_ear).union(right_ear)
    return housing.cut(pocket).cut(mouth)


def make_outer_gimbal() -> cq.Workplane:
    body = cq.Workplane("XY").box(OUTER_X, OUTER_Y, OUTER_Z, centered=(True, True, True))
    window = cq.Workplane("XY").box(
        OUTER_WINDOW_X,
        OUTER_WINDOW_Y,
        OUTER_Z + 0.004,
        centered=(True, True, True),
    )
    ring = body.cut(window)
    ring = ring.edges("|Z").fillet(0.0012)

    inner_pin_clearance = (
        cq.Workplane("XZ")
        .circle(INNER_TRUNNION_CLEAR_R)
        .extrude(OUTER_Y + 0.010, both=True)
    )
    ring = ring.cut(inner_pin_clearance)

    right_shaft = (
        cq.Workplane("YZ")
        .workplane(offset=OUTER_X / 2.0)
        .circle(OUTER_SHAFT_R)
        .extrude(OUTER_SHAFT_LEN)
    )
    left_shaft = (
        cq.Workplane("YZ")
        .workplane(offset=-OUTER_X / 2.0)
        .circle(OUTER_SHAFT_R)
        .extrude(-OUTER_SHAFT_LEN)
    )
    right_collar = (
        cq.Workplane("YZ")
        .workplane(offset=OUTER_X / 2.0 + OUTER_SHAFT_LEN)
        .circle(OUTER_COLLAR_R)
        .extrude(OUTER_COLLAR_T)
    )
    left_collar = (
        cq.Workplane("YZ")
        .workplane(offset=-(OUTER_X / 2.0 + OUTER_SHAFT_LEN))
        .circle(OUTER_COLLAR_R)
        .extrude(-OUTER_COLLAR_T)
    )

    return ring.union(right_shaft).union(left_shaft).union(right_collar).union(left_collar)


def make_inner_gimbal() -> cq.Workplane:
    body = cq.Workplane("XY").box(INNER_X, INNER_Y, INNER_Z, centered=(True, True, True))
    window = cq.Workplane("XY").box(
        INNER_WINDOW_X,
        INNER_WINDOW_Y,
        INNER_Z + 0.004,
        centered=(True, True, True),
    )
    frame = body.cut(window)
    frame = frame.edges("|Z").fillet(0.0010)

    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_H).translate((0.0, 0.0, -HUB_H / 2.0))
    web_x = cq.Workplane("XY").box(0.014, 0.003, 0.008, centered=(True, True, True))
    web_y = cq.Workplane("XY").box(0.003, 0.013, 0.008, centered=(True, True, True))

    front_shaft = (
        cq.Workplane("XZ")
        .workplane(offset=INNER_Y / 2.0)
        .circle(INNER_SHAFT_R)
        .extrude(INNER_SHAFT_LEN)
    )
    rear_shaft = (
        cq.Workplane("XZ")
        .workplane(offset=-INNER_Y / 2.0)
        .circle(INNER_SHAFT_R)
        .extrude(-INNER_SHAFT_LEN)
    )
    front_collar = (
        cq.Workplane("XZ")
        .workplane(offset=INNER_Y / 2.0 + INNER_SHAFT_LEN)
        .circle(INNER_COLLAR_R)
        .extrude(INNER_COLLAR_T)
    )
    rear_collar = (
        cq.Workplane("XZ")
        .workplane(offset=-(INNER_Y / 2.0 + INNER_SHAFT_LEN))
        .circle(INNER_COLLAR_R)
        .extrude(-INNER_COLLAR_T)
    )

    return (
        frame.union(hub)
        .union(web_x)
        .union(web_y)
        .union(front_shaft)
        .union(rear_shaft)
        .union(front_collar)
        .union(rear_collar)
    )


def make_lever() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(0.0054).extrude(0.006)
    stem = cq.Workplane("XY").circle(0.0034).extrude(0.042).translate((0.0, 0.0, 0.006))
    knob = cq.Workplane("XY").circle(0.0072).extrude(0.011).translate((0.0, 0.0, 0.048))
    knob = knob.edges(">Z").fillet(0.003)
    return collar.union(stem).union(knob)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_joystick_module")

    housing_mat = model.material("housing_mat", rgba=(0.16, 0.17, 0.18, 1.0))
    yoke_mat = model.material("yoke_mat", rgba=(0.60, 0.62, 0.66, 1.0))
    lever_mat = model.material("lever_mat", rgba=(0.10, 0.10, 0.11, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(make_housing(), "housing_shell"),
        material=housing_mat,
        name="housing_shell",
    )

    outer_gimbal = model.part("outer_gimbal")
    outer_gimbal.visual(
        mesh_from_cadquery(make_outer_gimbal(), "outer_gimbal"),
        material=yoke_mat,
        name="outer_gimbal",
    )

    inner_gimbal = model.part("inner_gimbal")
    inner_gimbal.visual(
        mesh_from_cadquery(make_inner_gimbal(), "inner_gimbal"),
        material=yoke_mat,
        name="inner_gimbal",
    )

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(make_lever(), "lever"),
        material=lever_mat,
        name="lever",
    )

    model.articulation(
        "housing_to_outer",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=outer_gimbal,
        origin=Origin(xyz=(0.0, 0.0, GIMBAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-0.30, upper=0.30),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=outer_gimbal,
        child=inner_gimbal,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-0.30, upper=0.30),
    )
    model.articulation(
        "inner_to_lever",
        ArticulationType.FIXED,
        parent=inner_gimbal,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, INNER_Z / 2.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    outer_gimbal = object_model.get_part("outer_gimbal")
    inner_gimbal = object_model.get_part("inner_gimbal")
    lever = object_model.get_part("lever")
    housing_to_outer = object_model.get_articulation("housing_to_outer")
    outer_to_inner = object_model.get_articulation("outer_to_inner")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        housing,
        outer_gimbal,
        reason="outer trunnion journals are captured by the housing's side supports",
    )
    ctx.allow_overlap(
        outer_gimbal,
        inner_gimbal,
        reason="nested cardan journals sit inside the outer yoke at the orthogonal pivot",
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

    ctx.expect_contact(
        housing,
        outer_gimbal,
        contact_tol=8e-4,
        name="outer_gimbal_is_carried_by_housing",
    )
    ctx.expect_contact(
        outer_gimbal,
        inner_gimbal,
        contact_tol=8e-4,
        name="inner_gimbal_is_carried_by_outer_gimbal",
    )
    ctx.expect_contact(
        inner_gimbal,
        lever,
        contact_tol=8e-4,
        name="lever_root_collar_seats_on_inner_gimbal",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "housing_is_grounded",
        housing_aabb is not None and abs(housing_aabb[0][2]) <= 1e-6,
        f"housing min z={None if housing_aabb is None else housing_aabb[0][2]}",
    )
    ctx.check(
        "lever_projects_above_housing",
        housing_aabb is not None
        and lever_aabb is not None
        and lever_aabb[1][2] > housing_aabb[1][2] + 0.015,
        (
            "lever top z="
            f"{None if lever_aabb is None else lever_aabb[1][2]}, "
            "housing top z="
            f"{None if housing_aabb is None else housing_aabb[1][2]}"
        ),
    )

    outer_axis = housing_to_outer.axis
    inner_axis = outer_to_inner.axis
    axis_dot = sum(a * b for a, b in zip(outer_axis, inner_axis))
    ctx.check(
        "cardan_axes_are_orthogonal",
        abs(axis_dot) <= 1e-9,
        f"outer axis={outer_axis}, inner axis={inner_axis}, dot={axis_dot}",
    )

    lever_rest = ctx.part_world_position(lever)
    with ctx.pose({housing_to_outer: 0.22}):
        lever_outer_tilt = ctx.part_world_position(lever)
        ctx.check(
            "outer_axis_tilts_lever_in_y",
            lever_rest is not None
            and lever_outer_tilt is not None
            and lever_outer_tilt[1] < lever_rest[1] - 0.001
            and lever_outer_tilt[2] < lever_rest[2],
            f"rest={lever_rest}, tilted={lever_outer_tilt}",
        )
    with ctx.pose({outer_to_inner: 0.22}):
        lever_inner_tilt = ctx.part_world_position(lever)
        ctx.check(
            "inner_axis_tilts_lever_in_x",
            lever_rest is not None
            and lever_inner_tilt is not None
            and lever_inner_tilt[0] > lever_rest[0] + 0.001
            and abs(lever_inner_tilt[1] - lever_rest[1]) < 0.0015,
            f"rest={lever_rest}, tilted={lever_inner_tilt}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
