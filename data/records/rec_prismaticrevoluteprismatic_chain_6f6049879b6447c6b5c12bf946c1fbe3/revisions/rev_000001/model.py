from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from math import pi

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
    mesh_from_cadquery,
)


SUPPORT_LENGTH = 0.62
SUPPORT_WIDTH = 0.11
SUPPORT_BEAM_HEIGHT = 0.048
SUPPORT_CROWN_HEIGHT = 0.016
RAIL_LENGTH = 0.46
RAIL_WIDTH = 0.062
RAIL_HEIGHT = 0.028
RAIL_CENTER_Z = -(SUPPORT_BEAM_HEIGHT / 2.0) - (RAIL_HEIGHT / 2.0)

CARRIAGE_LENGTH = 0.12
CARRIAGE_OUTER_WIDTH = 0.17
CARRIAGE_BODY_LENGTH = 0.086
CARRIAGE_BODY_WIDTH = 0.052
CARRIAGE_BODY_HEIGHT = 0.044
CARRIAGE_BODY_CENTER_Z = -0.114
CARRIAGE_EAR_LENGTH = 0.034
CARRIAGE_EAR_WIDTH = 0.014
CARRIAGE_EAR_HEIGHT = 0.032
CARRIAGE_EAR_CENTER_X = 0.018
CARRIAGE_EAR_CENTER_Z = -0.148
RUNNER_Y = 0.038
RUNNER_Z = -0.039
RUNNER_HEIGHT = 0.026
DROP_Y = 0.028
DROP_CENTER_Z = -0.074
DROP_HEIGHT = 0.044
CLEVIS_BLOCK_CENTER_Z = -0.148
CLEVIS_SLOT_WIDTH = 0.024
CLEVIS_SLOT_HEIGHT = 0.028

HINGE_COLLAR_RADIUS = 0.014
HINGE_COLLAR_LENGTH = 0.004
HINGE_SHAFT_RADIUS = 0.008
HINGE_SHAFT_LENGTH = 0.024
HINGE_HUB_RADIUS = 0.009
HINGE_HUB_LENGTH = 0.024
HINGE_BORE_RADIUS = 0.009
ELBOW_WEB_WIDTH = 0.048
ELBOW_HOUSING_LENGTH = 0.072
ELBOW_HOUSING_WIDTH = 0.054
ELBOW_HOUSING_HEIGHT = 0.046
ELBOW_HOUSING_CENTER_X = 0.176
ELBOW_HOUSING_CENTER_Z = -0.07
NOSE_GUIDE_ORIGIN_X = 0.212
NOSE_GUIDE_LENGTH = 0.046
NOSE_GUIDE_WIDTH = 0.044
NOSE_GUIDE_HEIGHT = 0.03
NOSE_STROKE = 0.04


def _support_shape() -> cq.Workplane:
    beam = (
        cq.Workplane("XY")
        .box(SUPPORT_LENGTH, SUPPORT_WIDTH, SUPPORT_BEAM_HEIGHT)
        .edges("|X")
        .fillet(0.005)
    )
    crown = cq.Workplane("XY").box(
        SUPPORT_LENGTH * 0.72,
        SUPPORT_WIDTH * 0.56,
        SUPPORT_CROWN_HEIGHT,
    ).translate((0.0, 0.0, (SUPPORT_BEAM_HEIGHT / 2.0) + (SUPPORT_CROWN_HEIGHT / 2.0) - 0.001))
    rail = cq.Workplane("XY").box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT).translate(
        (0.0, 0.0, RAIL_CENTER_Z + 0.001)
    )
    stop_left = cq.Workplane("XY").box(0.016, 0.074, 0.034).translate(
        (-(RAIL_LENGTH / 2.0) + 0.008, 0.0, RAIL_CENTER_Z - 0.003)
    )
    stop_right = cq.Workplane("XY").box(0.016, 0.074, 0.034).translate(
        ((RAIL_LENGTH / 2.0) - 0.008, 0.0, RAIL_CENTER_Z - 0.003)
    )
    return beam.union(crown).union(rail).union(stop_left).union(stop_right)


def _carriage_shape() -> cq.Workplane:
    left_runner = cq.Workplane("XY").box(0.105, 0.014, RUNNER_HEIGHT).translate((0.0, RUNNER_Y, RUNNER_Z))
    right_runner = cq.Workplane("XY").box(0.105, 0.014, RUNNER_HEIGHT).translate((0.0, -RUNNER_Y, RUNNER_Z))
    left_drop = cq.Workplane("XY").box(0.092, 0.012, DROP_HEIGHT).translate((0.0, DROP_Y, DROP_CENTER_Z))
    right_drop = cq.Workplane("XY").box(0.092, 0.012, DROP_HEIGHT).translate((0.0, -DROP_Y, DROP_CENTER_Z))
    hanging_body = cq.Workplane("XY").box(
        CARRIAGE_BODY_LENGTH,
        CARRIAGE_BODY_WIDTH,
        CARRIAGE_BODY_HEIGHT,
    ).translate((0.0, 0.0, CARRIAGE_BODY_CENTER_Z))
    neck = cq.Workplane("XY").box(0.024, 0.036, 0.012).translate((0.006, 0.0, -0.138))
    clevis_block = cq.Workplane("XY").box(0.034, 0.048, 0.032).translate(
        (CARRIAGE_EAR_CENTER_X, 0.0, CLEVIS_BLOCK_CENTER_Z)
    )
    carriage = (
        left_runner.union(right_runner)
        .union(left_drop)
        .union(right_drop)
        .union(hanging_body)
        .union(neck)
        .union(clevis_block)
    )
    clevis_slot = cq.Workplane("XY").box(
        CARRIAGE_EAR_LENGTH + 0.004,
        CLEVIS_SLOT_WIDTH,
        CLEVIS_SLOT_HEIGHT,
    ).translate((CARRIAGE_EAR_CENTER_X, 0.0, CLEVIS_BLOCK_CENTER_Z))
    hinge_bore = (
        cq.Workplane("XZ")
        .circle(HINGE_BORE_RADIUS)
        .extrude(0.06, both=True)
        .translate((CARRIAGE_EAR_CENTER_X, 0.0, CARRIAGE_EAR_CENTER_Z))
    )
    return carriage.cut(clevis_slot).cut(hinge_bore)


def _elbow_frame_shape() -> cq.Workplane:
    outer_profile = [
        (0.034, 0.01),
        (0.082, 0.006),
        (0.126, -0.018),
        (0.176, -0.048),
        (0.204, -0.048),
        (0.204, -0.086),
        (0.156, -0.092),
        (0.106, -0.074),
        (0.058, -0.044),
        (0.034, -0.024),
    ]
    body = (
        cq.Workplane("XZ")
        .polyline(outer_profile)
        .close()
        .extrude(ELBOW_WEB_WIDTH / 2.0, both=True)
    )
    window_profile = [
        (0.074, -0.014),
        (0.132, -0.018),
        (0.146, -0.04),
        (0.11, -0.058),
        (0.082, -0.05),
        (0.062, -0.028),
    ]
    window = (
        cq.Workplane("XZ")
        .polyline(window_profile)
        .close()
        .extrude((ELBOW_WEB_WIDTH / 2.0) + 0.01, both=True)
    )
    body = body.cut(window)
    hub = cq.Workplane("XZ").circle(HINGE_HUB_RADIUS).extrude(HINGE_HUB_LENGTH / 2.0, both=True)
    hinge_neck = cq.Workplane("XY").box(0.05, 0.022, 0.02).translate((0.025, 0.0, 0.0))
    housing = cq.Workplane("XY").box(
        ELBOW_HOUSING_LENGTH,
        ELBOW_HOUSING_WIDTH,
        ELBOW_HOUSING_HEIGHT,
    ).translate((ELBOW_HOUSING_CENTER_X, 0.0, ELBOW_HOUSING_CENTER_Z))
    return (
        hub.union(hinge_neck)
        .union(body)
        .union(housing)
    )


def _nose_shape() -> cq.Workplane:
    guide_bar = cq.Workplane("XY").box(
        NOSE_GUIDE_LENGTH,
        NOSE_GUIDE_WIDTH,
        NOSE_GUIDE_HEIGHT,
    ).translate((NOSE_GUIDE_LENGTH / 2.0, 0.0, 0.0))
    front_cap = (
        cq.Workplane("XY")
        .box(0.038, 0.058, 0.044)
        .translate((NOSE_GUIDE_LENGTH + 0.019, 0.0, 0.0))
        .edges(">X")
        .fillet(0.01)
    )
    return guide_bar.union(front_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_slide_link_slide_unit")

    model.material("anodized_graphite", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("machined_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("cast_aluminum", rgba=(0.62, 0.66, 0.70, 1.0))
    model.material("tool_black", rgba=(0.12, 0.13, 0.14, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((SUPPORT_LENGTH, SUPPORT_WIDTH, SUPPORT_BEAM_HEIGHT)),
        material="anodized_graphite",
        name="support_beam",
    )
    top_support.visual(
        Box((SUPPORT_LENGTH * 0.72, SUPPORT_WIDTH * 0.56, SUPPORT_CROWN_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, (SUPPORT_BEAM_HEIGHT / 2.0) + (SUPPORT_CROWN_HEIGHT / 2.0) - 0.001)),
        material="anodized_graphite",
        name="support_crown",
    )
    top_support.visual(
        Box((RAIL_LENGTH, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.038, -0.035)),
        material="machined_steel",
        name="left_rail",
    )
    top_support.visual(
        Box((RAIL_LENGTH, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, -0.038, -0.035)),
        material="machined_steel",
        name="right_rail",
    )
    top_support.visual(
        Box((0.016, 0.074, 0.034)),
        origin=Origin(xyz=(-(RAIL_LENGTH / 2.0) + 0.008, 0.0, -0.041)),
        material="machined_steel",
        name="left_stop",
    )
    top_support.visual(
        Box((0.016, 0.074, 0.034)),
        origin=Origin(xyz=((RAIL_LENGTH / 2.0) - 0.008, 0.0, -0.041)),
        material="machined_steel",
        name="right_stop",
    )
    top_support.inertial = Inertial.from_geometry(
        Box((SUPPORT_LENGTH, SUPPORT_WIDTH, SUPPORT_BEAM_HEIGHT + RAIL_HEIGHT)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.105, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, 0.038, -0.011)),
        material="machined_steel",
        name="left_runner",
    )
    carriage.visual(
        Box((0.105, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, -0.038, -0.011)),
        material="machined_steel",
        name="right_runner",
    )
    carriage.visual(
        Box((0.092, 0.022, 0.094)),
        origin=Origin(xyz=(0.0, 0.028, -0.063)),
        material="machined_steel",
        name="left_drop",
    )
    carriage.visual(
        Box((0.092, 0.022, 0.094)),
        origin=Origin(xyz=(0.0, -0.028, -0.063)),
        material="machined_steel",
        name="right_drop",
    )
    carriage.visual(
        Box((CARRIAGE_BODY_LENGTH, CARRIAGE_BODY_WIDTH, CARRIAGE_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_BODY_CENTER_Z)),
        material="machined_steel",
        name="carriage_block",
    )
    carriage.visual(
        Box((0.024, 0.036, 0.012)),
        origin=Origin(xyz=(0.006, 0.0, -0.118)),
        material="machined_steel",
        name="carriage_neck",
    )
    carriage.visual(
        Box((0.034, 0.012, 0.032)),
        origin=Origin(xyz=(CARRIAGE_EAR_CENTER_X, 0.018, CARRIAGE_EAR_CENTER_Z)),
        material="machined_steel",
        name="left_clevis_cheek",
    )
    carriage.visual(
        Box((0.034, 0.012, 0.032)),
        origin=Origin(xyz=(CARRIAGE_EAR_CENTER_X, -0.018, CARRIAGE_EAR_CENTER_Z)),
        material="machined_steel",
        name="right_clevis_cheek",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_OUTER_WIDTH, 0.16)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
    )

    elbow_frame = model.part("elbow_frame")
    elbow_frame.visual(
        Cylinder(radius=HINGE_HUB_RADIUS, length=0.024),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="cast_aluminum",
        name="hinge_hub",
    )
    elbow_frame.visual(
        Box((0.05, 0.022, 0.02)),
        origin=Origin(xyz=(0.025, 0.0, -0.011)),
        material="cast_aluminum",
        name="hinge_neck",
    )
    elbow_frame.visual(
        Box((0.086, 0.024, 0.032)),
        origin=Origin(xyz=(0.083, 0.0, -0.028)),
        material="cast_aluminum",
        name="upper_spine",
    )
    elbow_frame.visual(
        Box((0.12, 0.022, 0.022)),
        origin=Origin(xyz=(0.128, 0.0, -0.053)),
        material="cast_aluminum",
        name="lower_brace",
    )
    elbow_frame.visual(
        Box((ELBOW_HOUSING_LENGTH, ELBOW_HOUSING_WIDTH, ELBOW_HOUSING_HEIGHT)),
        origin=Origin(xyz=(ELBOW_HOUSING_CENTER_X, 0.0, ELBOW_HOUSING_CENTER_Z)),
        material="cast_aluminum",
        name="nose_housing",
    )
    elbow_frame.inertial = Inertial.from_geometry(
        Box((0.25, ELBOW_HOUSING_WIDTH, 0.15)),
        mass=1.1,
        origin=Origin(xyz=(0.12, 0.0, -0.06)),
    )

    sliding_nose = model.part("sliding_nose")
    sliding_nose.visual(
        Box((NOSE_GUIDE_LENGTH, NOSE_GUIDE_WIDTH, NOSE_GUIDE_HEIGHT)),
        origin=Origin(xyz=(NOSE_GUIDE_LENGTH / 2.0, 0.0, 0.0)),
        material="tool_black",
        name="nose_guide",
    )
    sliding_nose.visual(
        Box((0.038, 0.058, 0.044)),
        origin=Origin(xyz=(NOSE_GUIDE_LENGTH + 0.019, 0.0, 0.0)),
        material="tool_black",
        name="nose_head",
    )
    sliding_nose.inertial = Inertial.from_geometry(
        Box((0.14, 0.062, 0.048)),
        mass=0.35,
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=top_support,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.16, upper=0.16, effort=220.0, velocity=0.28),
    )
    model.articulation(
        "carriage_to_elbow",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=elbow_frame,
        origin=Origin(xyz=(CARRIAGE_EAR_CENTER_X, 0.0, CARRIAGE_EAR_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=1.0, effort=45.0, velocity=1.2),
    )
    model.articulation(
        "elbow_to_nose",
        ArticulationType.PRISMATIC,
        parent=elbow_frame,
        child=sliding_nose,
        origin=Origin(xyz=(NOSE_GUIDE_ORIGIN_X, 0.0, ELBOW_HOUSING_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=NOSE_STROKE, effort=30.0, velocity=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    carriage = object_model.get_part("carriage")
    elbow_frame = object_model.get_part("elbow_frame")
    sliding_nose = object_model.get_part("sliding_nose")
    support_slide = object_model.get_articulation("support_to_carriage")
    elbow_hinge = object_model.get_articulation("carriage_to_elbow")
    nose_slide = object_model.get_articulation("elbow_to_nose")

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

    ctx.expect_origin_gap(
        top_support,
        carriage,
        axis="z",
        min_gap=0.03,
        name="carriage hangs beneath top support",
    )
    ctx.expect_contact(
        carriage,
        top_support,
        contact_tol=0.002,
        name="carriage saddle bears on rail",
    )
    ctx.expect_contact(
        carriage,
        elbow_frame,
        contact_tol=0.002,
        name="elbow barrel seats in carriage clevis",
    )
    ctx.expect_contact(
        sliding_nose,
        elbow_frame,
        contact_tol=0.002,
        name="nose stays guided inside distal slide housing",
    )

    with ctx.pose({elbow_hinge: 0.0, nose_slide: 0.0}):
        ctx.expect_origin_gap(
            sliding_nose,
            elbow_frame,
            axis="x",
            min_gap=0.12,
            name="nose is mounted at the elbow's distal end",
        )

    with ctx.pose({support_slide: support_slide.motion_limits.lower}):
        low_pos = ctx.part_world_position(carriage)
    with ctx.pose({support_slide: support_slide.motion_limits.upper}):
        high_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage prismatic motion runs along support length",
        low_pos is not None
        and high_pos is not None
        and (high_pos[0] - low_pos[0]) > 0.30,
        details=f"low={low_pos}, high={high_pos}",
    )

    with ctx.pose({support_slide: 0.0, elbow_hinge: 0.0, nose_slide: 0.0}):
        rest_nose_pos = ctx.part_world_position(sliding_nose)
    with ctx.pose({support_slide: 0.0, elbow_hinge: 0.9, nose_slide: 0.0}):
        raised_nose_pos = ctx.part_world_position(sliding_nose)
    ctx.check(
        "positive elbow rotation lifts the distal assembly",
        rest_nose_pos is not None
        and raised_nose_pos is not None
        and (raised_nose_pos[2] - rest_nose_pos[2]) > 0.11,
        details=f"rest={rest_nose_pos}, raised={raised_nose_pos}",
    )

    with ctx.pose({support_slide: 0.0, elbow_hinge: 0.45, nose_slide: 0.0}):
        elbow_pos_retracted = ctx.part_world_position(elbow_frame)
        nose_pos_retracted = ctx.part_world_position(sliding_nose)
    with ctx.pose({support_slide: 0.0, elbow_hinge: 0.45, nose_slide: NOSE_STROKE}):
        elbow_pos_extended = ctx.part_world_position(elbow_frame)
        nose_pos_extended = ctx.part_world_position(sliding_nose)

    def _planar_distance(a: tuple[float, float, float] | None, b: tuple[float, float, float] | None) -> float | None:
        if a is None or b is None:
            return None
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    retracted_dist = _planar_distance(nose_pos_retracted, elbow_pos_retracted)
    extended_dist = _planar_distance(nose_pos_extended, elbow_pos_extended)
    ctx.check(
        "distal prismatic stage extends the nose outward",
        retracted_dist is not None
        and extended_dist is not None
        and (extended_dist - retracted_dist) > 0.03,
        details=f"retracted={retracted_dist}, extended={extended_dist}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
