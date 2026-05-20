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
)


def _circle_profile(radius: float, *, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _aabb_center(bounds):
    (min_corner, max_corner) = bounds
    return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_wall_thermostat")

    plate_grey = model.material("plate_grey", rgba=(0.63, 0.66, 0.69, 1.0))
    housing_dark = model.material("housing_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    dial_black = model.material("dial_black", rgba=(0.12, 0.12, 0.13, 1.0))
    guard_yellow = model.material("guard_yellow", rgba=(0.80, 0.70, 0.12, 1.0))
    retainer_steel = model.material("retainer_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    signal_red = model.material("signal_red", rgba=(0.76, 0.16, 0.12, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.145, 0.205, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=plate_grey,
        name="mounting_plate",
    )
    wall_plate.visual(
        Box((0.110, 0.160, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=plate_grey,
        name="center_doubler",
    )
    wall_plate.visual(
        Box((0.016, 0.138, 0.010)),
        origin=Origin(xyz=(-0.047, 0.0, 0.013)),
        material=plate_grey,
        name="left_load_rail",
    )
    wall_plate.visual(
        Box((0.016, 0.138, 0.010)),
        origin=Origin(xyz=(0.047, 0.0, 0.013)),
        material=plate_grey,
        name="right_load_rail",
    )
    wall_plate.visual(
        Box((0.080, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.060, 0.012)),
        material=plate_grey,
        name="upper_cross_brace",
    )
    wall_plate.visual(
        Box((0.080, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -0.060, 0.012)),
        material=plate_grey,
        name="lower_cross_brace",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.050, 0.074),
            (0.050, 0.074),
            (-0.050, -0.074),
            (0.050, -0.074),
        )
    ):
        wall_plate.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x_pos, y_pos, 0.010)),
            material=plate_grey,
            name=f"mount_pad_{index}",
        )
        wall_plate.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(x_pos, y_pos, 0.0135)),
            material=fastener_steel,
            name=f"wall_fastener_{index}",
        )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.145, 0.205, 0.026)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.112, 0.154, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=housing_dark,
        name="main_body",
    )
    housing.visual(
        Box((0.014, 0.110, 0.028)),
        origin=Origin(xyz=(-0.040, 0.0, 0.014)),
        material=housing_dark,
        name="left_body_rib",
    )
    housing.visual(
        Box((0.014, 0.110, 0.028)),
        origin=Origin(xyz=(0.040, 0.0, 0.014)),
        material=housing_dark,
        name="right_body_rib",
    )
    housing.visual(
        Box((0.060, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.056, 0.019)),
        material=housing_dark,
        name="upper_joint_block",
    )
    housing.visual(
        Box((0.060, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.056, 0.019)),
        material=housing_dark,
        name="lower_joint_block",
    )
    housing.visual(
        Box((0.020, 0.046, 0.024)),
        origin=Origin(xyz=(-0.046, 0.0, 0.012)),
        material=housing_dark,
        name="left_guard_socket",
    )
    housing.visual(
        Box((0.020, 0.046, 0.024)),
        origin=Origin(xyz=(0.046, 0.0, 0.012)),
        material=housing_dark,
        name="right_guard_socket",
    )
    housing.visual(
        Cylinder(radius=0.046, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=housing_dark,
        name="front_collar",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=retainer_steel,
        name="shaft_boss",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.112, 0.154, 0.048)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )
    model.articulation(
        "wall_plate_to_housing",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    guard_frame = model.part("guard_frame")
    guard_frame.visual(
        Box((0.012, 0.154, 0.072)),
        origin=Origin(xyz=(-0.061, 0.0, 0.036)),
        material=guard_yellow,
        name="left_guard_rail",
    )
    guard_frame.visual(
        Box((0.012, 0.154, 0.072)),
        origin=Origin(xyz=(0.061, 0.0, 0.036)),
        material=guard_yellow,
        name="right_guard_rail",
    )
    guard_frame.visual(
        Box((0.110, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.072, 0.060)),
        material=guard_yellow,
        name="top_guard_bridge",
    )
    guard_frame.visual(
        Box((0.110, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.072, 0.058)),
        material=guard_yellow,
        name="bottom_guard_bridge",
    )
    guard_frame.visual(
        Box((0.020, 0.060, 0.020)),
        origin=Origin(xyz=(-0.066, 0.0, 0.010)),
        material=guard_yellow,
        name="left_rear_bracket",
    )
    guard_frame.visual(
        Box((0.020, 0.060, 0.020)),
        origin=Origin(xyz=(0.066, 0.0, 0.010)),
        material=guard_yellow,
        name="right_rear_bracket",
    )
    guard_frame.visual(
        Box((0.008, 0.018, 0.018)),
        origin=Origin(xyz=(-0.014, -0.060, 0.070)),
        material=guard_yellow,
        name="lockout_ear_left",
    )
    guard_frame.visual(
        Box((0.008, 0.018, 0.018)),
        origin=Origin(xyz=(0.014, -0.060, 0.070)),
        material=guard_yellow,
        name="lockout_ear_right",
    )
    guard_frame.inertial = Inertial.from_geometry(
        Box((0.134, 0.154, 0.072)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )
    model.articulation(
        "housing_to_guard_frame",
        ArticulationType.FIXED,
        parent=housing,
        child=guard_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    retainer_ring = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.037, segments=40),
            [_circle_profile(0.028, segments=32)],
            0.006,
            center=True,
            cap=True,
            closed=True,
        ),
        "thermostat_retainer_ring",
    )
    shaft_retainer = model.part("shaft_retainer")
    shaft_retainer.visual(
        retainer_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=retainer_steel,
        name="retainer_flange",
    )
    for index, angle in enumerate(
        (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)
    ):
        shaft_retainer.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(
                xyz=(0.024 * math.cos(angle), 0.024 * math.sin(angle), 0.006),
            ),
            material=fastener_steel,
            name=f"retainer_fastener_{index}",
        )
        shaft_retainer.visual(
            Cylinder(radius=0.0035, length=0.010),
            origin=Origin(
                xyz=(0.024 * math.cos(angle), 0.024 * math.sin(angle), 0.002),
            ),
            material=retainer_steel,
            name=f"retainer_spacer_{index}",
        )
    shaft_retainer.visual(
        Box((0.020, 0.006, 0.006)),
        origin=Origin(xyz=(-0.033, 0.024, 0.004)),
        material=retainer_steel,
        name="left_stop_rib",
    )
    shaft_retainer.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(-0.046, 0.029, 0.005)),
        material=retainer_steel,
        name="upper_left_stop",
    )
    shaft_retainer.visual(
        Box((0.020, 0.006, 0.006)),
        origin=Origin(xyz=(0.033, 0.024, 0.004)),
        material=retainer_steel,
        name="right_stop_rib",
    )
    shaft_retainer.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(0.046, 0.029, 0.005)),
        material=retainer_steel,
        name="upper_right_stop",
    )
    shaft_retainer.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.010)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    model.articulation(
        "housing_to_shaft_retainer",
        ArticulationType.FIXED,
        parent=housing,
        child=shaft_retainer,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.032, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dial_black,
        name="dial_core",
    )
    dial.visual(
        Cylinder(radius=0.044, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dial_black,
        name="dial_rim",
    )
    dial.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=retainer_steel,
        name="dial_shaft",
    )
    dial.visual(
        Cylinder(radius=0.031, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=retainer_steel,
        name="rear_thrust_disc",
    )
    dial.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=retainer_steel,
        name="front_retainer_cap",
    )
    dial.visual(
        Box((0.008, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.022, 0.020)),
        material=signal_red,
        name="pointer",
    )
    for index, angle in enumerate([index * (math.pi / 4.0) for index in range(8)]):
        dial.visual(
            Box((0.010, 0.006, 0.012)),
            origin=Origin(
                xyz=(0.039 * math.cos(angle), 0.039 * math.sin(angle), 0.013),
                rpy=(0.0, 0.0, angle),
            ),
            material=dial_black,
            name=f"grip_bar_{index}",
        )
    dial.visual(
        Box((0.008, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, -0.032, 0.009)),
        material=retainer_steel,
        name="travel_stop_arm",
    )
    dial.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, -0.044, 0.009)),
        material=retainer_steel,
        name="travel_stop_tip",
    )
    dial.visual(
        Box((0.008, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.032, 0.010)),
        material=retainer_steel,
        name="lockout_stem",
    )
    dial.visual(
        Box((0.012, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, -0.046, 0.015)),
        material=retainer_steel,
        name="lockout_tab",
    )
    dial.visual(
        Cylinder(radius=0.005, length=0.008),
        origin=Origin(xyz=(0.0, -0.055, 0.015)),
        material=retainer_steel,
        name="lockout_tab_tip",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.032),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )
    model.articulation(
        "housing_to_dial",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=-2.15,
            upper=2.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    housing = object_model.get_part("housing")
    guard_frame = object_model.get_part("guard_frame")
    shaft_retainer = object_model.get_part("shaft_retainer")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("housing_to_dial")

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

    ctx.expect_contact(housing, wall_plate, name="housing_bolted_to_wall_plate")
    ctx.expect_contact(guard_frame, housing, name="guard_frame_supported_by_housing")
    ctx.expect_contact(shaft_retainer, housing, name="retainer_supported_by_housing")
    ctx.expect_overlap(dial, shaft_retainer, axes="xy", min_overlap=0.060, name="dial_centered_on_retainer")
    ctx.expect_gap(
        dial,
        shaft_retainer,
        axis="z",
        min_gap=0.0045,
        max_gap=0.012,
        positive_elem="dial_rim",
        negative_elem="retainer_flange",
        name="dial_face_clear_of_retainer",
    )

    limits = dial_joint.motion_limits
    ctx.check(
        "dial_joint_is_center_axis_revolute",
        dial_joint.articulation_type == ArticulationType.REVOLUTE and tuple(dial_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}",
    )
    ctx.check(
        "dial_has_overtravel_motion_limits",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -2.0
        and limits.upper >= 2.0,
        details=(
            "expected heavy-duty dial rotation to be limited by substantial travel stops; "
            f"got limits={None if limits is None else (limits.lower, limits.upper)}"
        ),
    )

    pointer_rest = ctx.part_element_world_aabb(dial, elem="pointer")
    with ctx.pose({dial_joint: 1.2}):
        pointer_positive = ctx.part_element_world_aabb(dial, elem="pointer")
    with ctx.pose({dial_joint: -1.2}):
        pointer_negative = ctx.part_element_world_aabb(dial, elem="pointer")
    if pointer_rest is not None and pointer_positive is not None and pointer_negative is not None:
        rest_center = _aabb_center(pointer_rest)
        positive_center = _aabb_center(pointer_positive)
        negative_center = _aabb_center(pointer_negative)
        ctx.check(
            "dial_pointer_swings_around_center",
            positive_center[0] < rest_center[0] - 0.012
            and negative_center[0] > rest_center[0] + 0.012
            and positive_center[1] < rest_center[1]
            and negative_center[1] < rest_center[1],
            details=(
                f"rest={rest_center}, positive={positive_center}, negative={negative_center}"
            ),
        )
    else:
        ctx.fail("dial_pointer_swings_around_center", "pointer AABB could not be resolved across poses")

    left_ear = ctx.part_element_world_aabb(guard_frame, elem="lockout_ear_left")
    right_ear = ctx.part_element_world_aabb(guard_frame, elem="lockout_ear_right")
    lockout_tab = ctx.part_element_world_aabb(dial, elem="lockout_tab")
    if left_ear is not None and right_ear is not None and lockout_tab is not None:
        left_max_x = left_ear[1][0]
        right_min_x = right_ear[0][0]
        tab_center = _aabb_center(lockout_tab)
        left_center = _aabb_center(left_ear)
        right_center = _aabb_center(right_ear)
        ctx.check(
            "lockout_tab_sits_between_guard_ears",
            left_max_x < tab_center[0] < right_min_x
            and abs(tab_center[1] - left_center[1]) < 0.015
            and abs(tab_center[1] - right_center[1]) < 0.015,
            details=(
                f"left_max_x={left_max_x}, right_min_x={right_min_x}, tab_center={tab_center}, "
                f"left_center={left_center}, right_center={right_center}"
            ),
        )
    else:
        ctx.fail("lockout_tab_sits_between_guard_ears", "lockout feature AABBs could not be resolved")

    with ctx.pose({dial_joint: limits.upper if limits is not None and limits.upper is not None else 0.0}):
        ctx.expect_contact(
            dial,
            shaft_retainer,
            elem_a="travel_stop_tip",
            elem_b="upper_right_stop",
            contact_tol=0.003,
            name="upper_travel_stop_is_reachable",
        )
    with ctx.pose({dial_joint: limits.lower if limits is not None and limits.lower is not None else 0.0}):
        ctx.expect_contact(
            dial,
            shaft_retainer,
            elem_a="travel_stop_tip",
            elem_b="upper_left_stop",
            contact_tol=0.003,
            name="lower_travel_stop_is_reachable",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
