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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_WIDTH = 0.60
BODY_DEPTH = 0.44
BODY_HEIGHT = 0.85
SIDE_THICKNESS = 0.018
TOP_THICKNESS = 0.018
BOTTOM_THICKNESS = 0.030
BACK_THICKNESS = 0.015
FRONT_PANEL_THICKNESS = 0.018
OPENING_CENTER_X = -0.075
OPENING_CENTER_Z = 0.385
DOOR_OUTER_RADIUS = 0.168
DOOR_WINDOW_RADIUS = 0.118
DRUM_RADIUS = 0.154
DRUM_DEPTH = 0.318


def _circle_profile(
    radius: float,
    *,
    segments: int = 64,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + (radius * math.cos((2.0 * math.pi * index) / segments)),
            cy + (radius * math.sin((2.0 * math.pi * index) / segments)),
        )
        for index in range(segments)
    ]


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_width = width * 0.5
    half_height = height * 0.5
    return [
        (-half_width, -half_height),
        (half_width, -half_height),
        (half_width, half_height),
        (-half_width, half_height),
    ]


def _annulus_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 72,
):
    geometry = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        thickness,
        center=True,
    )
    return mesh_from_geometry(geometry, name)


def _front_fascia_mesh():
    geometry = ExtrudeWithHolesGeometry(
        _rect_profile(BODY_WIDTH - (2.0 * SIDE_THICKNESS), 0.73),
        [_circle_profile(0.150, center=(OPENING_CENTER_X, 0.020), segments=80)],
        FRONT_PANEL_THICKNESS,
        center=True,
    )
    return mesh_from_geometry(geometry, "washer_front_fascia_v2")


def _drum_shell_mesh():
    outer_profile = [
        (DRUM_RADIUS, -0.155),
        (DRUM_RADIUS, 0.110),
        (0.148, 0.136),
        (0.070, 0.160),
        (0.050, 0.163),
    ]
    inner_profile = [
        (0.136, -0.153),
        (0.136, 0.096),
        (0.130, 0.126),
        (0.050, 0.158),
    ]
    geometry = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geometry, "washer_drum_shell_v2")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_washing_machine")

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.96, 1.0))
    console_white = model.material("console_white", rgba=(0.92, 0.93, 0.95, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.15, 0.17, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.44, 0.52, 0.60, 0.38))
    chrome = model.material("chrome", rgba=(0.76, 0.79, 0.83, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.28, 0.29, 0.31, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((SIDE_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-(BODY_WIDTH * 0.5) + (SIDE_THICKNESS * 0.5), 0.0, BODY_HEIGHT * 0.5)),
        material=body_white,
        name="left_side",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=((BODY_WIDTH * 0.5) - (SIDE_THICKNESS * 0.5), 0.0, BODY_HEIGHT * 0.5)),
        material=body_white,
        name="right_side",
    )
    cabinet.visual(
        Box((BODY_WIDTH - (2.0 * SIDE_THICKNESS), BODY_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - (TOP_THICKNESS * 0.5))),
        material=body_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((BODY_WIDTH - (2.0 * SIDE_THICKNESS), BODY_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS * 0.5)),
        material=body_white,
        name="bottom_pan",
    )
    cabinet.visual(
        Box((BODY_WIDTH - (2.0 * SIDE_THICKNESS), BACK_THICKNESS, BODY_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_DEPTH * 0.5) - (BACK_THICKNESS * 0.5),
                BOTTOM_THICKNESS + ((BODY_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS) * 0.5),
            )
        ),
        material=body_white,
        name="rear_panel",
    )
    cabinet.visual(
        _front_fascia_mesh(),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH * 0.5) + (FRONT_PANEL_THICKNESS * 0.5), 0.365), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_white,
        name="front_fascia",
    )
    cabinet.visual(
        Box((BODY_WIDTH - (2.0 * SIDE_THICKNESS), 0.035, 0.120)),
        origin=Origin(xyz=(0.0, -0.2025, 0.790)),
        material=console_white,
        name="control_console",
    )
    cabinet.visual(
        Box((0.180, 0.006, 0.040)),
        origin=Origin(xyz=(-0.085, -0.223, 0.798)),
        material=dark_trim,
        name="display_window",
    )
    cabinet.visual(
        _annulus_mesh(
            "washer_boot_gasket_v2",
            outer_radius=0.164,
            inner_radius=0.122,
            thickness=0.020,
        ),
        origin=Origin(xyz=(OPENING_CENTER_X, -0.192, OPENING_CENTER_Z,), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="door_boot",
    )
    cabinet.visual(
        Box((0.070, 0.055, 0.070)),
        origin=Origin(xyz=(OPENING_CENTER_X, 0.1925, OPENING_CENTER_Z)),
        material=hinge_dark,
        name="bearing_support",
    )
    cabinet.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(OPENING_CENTER_X, 0.165, OPENING_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="bearing_cap",
    )
    cabinet.visual(
        Box((0.090, 0.020, 0.032)),
        origin=Origin(xyz=(OPENING_CENTER_X + DOOR_OUTER_RADIUS + 0.052, -0.210, OPENING_CENTER_Z + 0.095)),
        material=hinge_dark,
        name="upper_body_leaf",
    )
    cabinet.visual(
        Box((0.090, 0.020, 0.032)),
        origin=Origin(xyz=(OPENING_CENTER_X + DOOR_OUTER_RADIUS + 0.052, -0.210, OPENING_CENTER_Z - 0.095)),
        material=hinge_dark,
        name="lower_body_leaf",
    )
    cabinet.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(OPENING_CENTER_X + DOOR_OUTER_RADIUS, -0.228, OPENING_CENTER_Z + 0.095)),
        material=hinge_dark,
        name="upper_body_barrel",
    )
    cabinet.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(OPENING_CENTER_X + DOOR_OUTER_RADIUS, -0.228, OPENING_CENTER_Z - 0.095)),
        material=hinge_dark,
        name="lower_body_barrel",
    )
    cabinet.visual(
        Box((0.056, 0.022, 0.020)),
        origin=Origin(xyz=(OPENING_CENTER_X + DOOR_OUTER_RADIUS + 0.028, -0.219, OPENING_CENTER_Z + 0.095)),
        material=hinge_dark,
        name="upper_barrel_bridge",
    )
    cabinet.visual(
        Box((0.056, 0.022, 0.020)),
        origin=Origin(xyz=(OPENING_CENTER_X + DOOR_OUTER_RADIUS + 0.028, -0.219, OPENING_CENTER_Z - 0.095)),
        material=hinge_dark,
        name="lower_barrel_bridge",
    )

    drum = model.part("drum")
    drum.visual(
        _drum_shell_mesh(),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.105, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.136, length=0.012),
        origin=Origin(xyz=(0.0, 0.130, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_bulkhead",
    )
    drum.visual(
        Box((0.024, 0.240, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=steel,
        name="baffle_0",
    )
    drum.visual(
        Box((0.024, 0.240, 0.030)),
        origin=Origin(xyz=(0.100, 0.0, -0.058)),
        material=steel,
        name="baffle_1",
    )
    drum.visual(
        Box((0.024, 0.240, 0.030)),
        origin=Origin(xyz=(-0.100, 0.0, -0.058)),
        material=steel,
        name="baffle_2",
    )

    door = model.part("door")
    door.visual(
        _annulus_mesh(
            "washer_door_outer_ring_v2",
            outer_radius=DOOR_OUTER_RADIUS,
            inner_radius=0.136,
            thickness=0.035,
        ),
        origin=Origin(xyz=(-DOOR_OUTER_RADIUS, -0.0095, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_white,
        name="outer_ring",
    )
    door.visual(
        _annulus_mesh(
            "washer_door_chrome_bezel_v2",
            outer_radius=0.154,
            inner_radius=0.123,
            thickness=0.008,
        ),
        origin=Origin(xyz=(-DOOR_OUTER_RADIUS, -0.0155, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="chrome_bezel",
    )
    door.visual(
        Cylinder(radius=0.123, length=0.026),
        origin=Origin(xyz=(-DOOR_OUTER_RADIUS, -0.0115, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=smoked_glass,
        name="door_glass",
    )

    latch_release = model.part("latch_release")
    latch_release.visual(
        Cylinder(radius=0.005, length=0.030),
        material=hinge_dark,
        name="latch_pivot",
    )
    latch_release.visual(
        Box((0.022, 0.010, 0.034)),
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
        material=chrome,
        name="latch_handle",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_body",
    )
    selector_knob.visual(
        Box((0.008, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=dark_trim,
        name="pointer",
    )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(OPENING_CENTER_X, 0.015, OPENING_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=12.0),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(OPENING_CENTER_X + DOOR_OUTER_RADIUS, -0.228, OPENING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=-math.radians(100.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "door_to_latch_release",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch_release,
        origin=Origin(xyz=(-(2.0 * DOOR_OUTER_RADIUS) - 0.004, -0.011, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=-0.35,
            upper=0.0,
        ),
    )
    model.articulation(
        "cabinet_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(0.185, -0.231, 0.796)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    latch_release = object_model.get_part("latch_release")
    selector_knob = object_model.get_part("selector_knob")

    drum_spin = object_model.get_articulation("cabinet_to_drum")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    latch_hinge = object_model.get_articulation("door_to_latch_release")
    knob_spin = object_model.get_articulation("cabinet_to_selector_knob")

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
        door,
        latch_release,
        elem_a="outer_ring",
        elem_b="latch_pivot",
        reason="The latch pivot passes through a molded boss at the door rim.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(cabinet, drum, elem_a="bearing_cap", elem_b="rear_bulkhead")
    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        max_gap=0.0002,
        max_penetration=0.0,
        positive_elem="front_fascia",
        negative_elem="outer_ring",
        name="door_face_seats_on_front",
    )
    ctx.expect_contact(door, latch_release, elem_a="outer_ring", elem_b="latch_pivot")
    ctx.expect_contact(selector_knob, cabinet)

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            max_gap=0.004,
            max_penetration=0.0,
            positive_elem="front_fascia",
            negative_elem="outer_ring",
            name="door_closed_front_gap",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.22,
            elem_a="outer_ring",
            elem_b="front_fascia",
            name="door_centered_on_opening",
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_no_overlap")
            ctx.fail_if_isolated_parts(name="door_closed_no_floating")

    latch_limits = latch_hinge.motion_limits
    if latch_limits is not None and latch_limits.upper is not None:
        with ctx.pose({latch_hinge: latch_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="latch_rest_no_overlap")

    closed_ring_aabb = ctx.part_element_world_aabb(door, elem="outer_ring")
    if closed_ring_aabb is None:
        ctx.fail("door_outer_ring_present", "Could not resolve the door outer-ring AABB at rest.")
    else:
        with ctx.pose({door_hinge: -math.radians(100.0)}):
            open_ring_aabb = ctx.part_element_world_aabb(door, elem="outer_ring")
        if open_ring_aabb is None:
            ctx.fail("door_outer_ring_open_pose_present", "Could not resolve the door outer-ring AABB at open pose.")
        else:
            closed_center_y = 0.5 * (closed_ring_aabb[0][1] + closed_ring_aabb[1][1])
            open_center_y = 0.5 * (open_ring_aabb[0][1] + open_ring_aabb[1][1])
            ctx.check(
                "door_swings_clear_of_opening",
                open_center_y > (closed_center_y + 0.12),
                details=(
                    f"Closed center y={closed_center_y:.4f}, "
                    f"open center y={open_center_y:.4f}"
                ),
            )

    rest_baffle_aabb = ctx.part_element_world_aabb(drum, elem="baffle_0")
    if rest_baffle_aabb is None:
        ctx.fail("drum_baffle_present", "Could not resolve the drum baffle AABB.")
    else:
        with ctx.pose({drum_spin: math.pi / 2.0}):
            spun_baffle_aabb = ctx.part_element_world_aabb(drum, elem="baffle_0")
            ctx.fail_if_parts_overlap_in_current_pose(name="drum_quarter_turn_no_overlap")
        if spun_baffle_aabb is None:
            ctx.fail("drum_baffle_rotated_present", "Could not resolve the rotated drum baffle AABB.")
        else:
            rest_center_x = 0.5 * (rest_baffle_aabb[0][0] + rest_baffle_aabb[1][0])
            spun_center_x = 0.5 * (spun_baffle_aabb[0][0] + spun_baffle_aabb[1][0])
            ctx.check(
                "drum_rotates_about_axle",
                spun_center_x > (rest_center_x + 0.08),
                details=f"Rest center x={rest_center_x:.4f}, spun center x={spun_center_x:.4f}",
            )

    rest_latch_aabb = ctx.part_world_aabb(latch_release)
    if rest_latch_aabb is None:
        ctx.fail("latch_rest_present", "Could not resolve latch-release AABB at rest.")
    else:
        with ctx.pose({latch_hinge: -0.30}):
            open_latch_aabb = ctx.part_world_aabb(latch_release)
        if open_latch_aabb is None:
            ctx.fail("latch_open_present", "Could not resolve latch-release AABB at open pose.")
        else:
            ctx.check(
                "latch_pulls_forward",
                open_latch_aabb[1][1] > (rest_latch_aabb[1][1] + 0.003),
                details=(
                    f"Rest max y={rest_latch_aabb[1][1]:.4f}, "
                    f"open max y={open_latch_aabb[1][1]:.4f}"
                ),
            )

    rest_pointer_aabb = ctx.part_element_world_aabb(selector_knob, elem="pointer")
    if rest_pointer_aabb is None:
        ctx.fail("selector_pointer_present", "Could not resolve selector-knob pointer AABB at rest.")
    else:
        with ctx.pose({knob_spin: 1.2}):
            turned_pointer_aabb = ctx.part_element_world_aabb(selector_knob, elem="pointer")
            ctx.fail_if_parts_overlap_in_current_pose(name="selector_knob_turned_no_overlap")
        if turned_pointer_aabb is None:
            ctx.fail("selector_pointer_turned_present", "Could not resolve selector-knob pointer AABB at turned pose.")
        else:
            rest_center_x = 0.5 * (rest_pointer_aabb[0][0] + rest_pointer_aabb[1][0])
            turned_center_x = 0.5 * (turned_pointer_aabb[0][0] + turned_pointer_aabb[1][0])
            ctx.check(
                "selector_knob_rotates",
                turned_center_x > (rest_center_x + 0.015),
                details=f"Rest center x={rest_center_x:.4f}, turned center x={turned_center_x:.4f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
