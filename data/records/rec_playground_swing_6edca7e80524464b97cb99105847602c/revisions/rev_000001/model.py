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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


FRAME_WIDTH = 2.70
FRAME_HALF_WIDTH = FRAME_WIDTH / 2.0
FRAME_DEPTH = 1.70
FRAME_HALF_DEPTH = FRAME_DEPTH / 2.0
BEAM_Z = 2.45
BEAM_RADIUS = 0.06
LEG_RADIUS = 0.05
BRACE_RADIUS = 0.035
TOP_PIVOT_CLEARANCE = 0.035
SIDE_BRACE_Z = 0.78

HANGER_RADIUS = 0.016
PIN_RADIUS = 0.012
PIN_LENGTH = 0.075
CLEVIS_TAB_THICKNESS = 0.008
CLEVIS_TAB_HEIGHT = 0.05
CLEVIS_TAB_DEPTH = 0.055
TAB_X_OFFSET = (PIN_LENGTH + CLEVIS_TAB_THICKNESS) / 2.0
MOUNT_STRAP_DEPTH = 0.04

TIRE_MAJOR_RADIUS = 0.29
TIRE_TUBE_RADIUS = 0.11
TIRE_CENTER_WORLD = (0.0, 0.0, 0.70)

TOP_PIVOT_Z = BEAM_Z - BEAM_RADIUS - TOP_PIVOT_CLEARANCE
ATTACH_CENTER_Z = 0.895
SIDE_BRACE_HALF_SPAN = FRAME_HALF_DEPTH * ((BEAM_Z - SIDE_BRACE_Z) / BEAM_Z)

LEFT_ATTACH_WORLD = (-0.214, 0.180, ATTACH_CENTER_Z)
CENTER_ATTACH_WORLD = (0.0, -0.280, ATTACH_CENTER_Z)
RIGHT_ATTACH_WORLD = (0.214, 0.180, ATTACH_CENTER_Z)

LEFT_PIVOT_WORLD = (LEFT_ATTACH_WORLD[0], 0.0, TOP_PIVOT_Z)
CENTER_PIVOT_WORLD = (CENTER_ATTACH_WORLD[0], 0.0, TOP_PIVOT_Z)
RIGHT_PIVOT_WORLD = (RIGHT_ATTACH_WORLD[0], 0.0, TOP_PIVOT_Z)


def _sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0)


def _segment_length(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _segment_origin(a: tuple[float, float, float], b: tuple[float, float, float]) -> Origin:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return Origin(xyz=a)
    yaw = math.atan2(dy, dx)
    pitch = math.acos(max(-1.0, min(1.0, dz / length)))
    return Origin(xyz=_midpoint(a, b), rpy=(0.0, pitch, yaw))


def _x_axis_cylinder_origin(center: tuple[float, float, float]) -> Origin:
    return Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tire_style_playground_swing")

    frame_metal = model.material("frame_metal", rgba=(0.22, 0.29, 0.36, 1.0))
    hanger_metal = model.material("hanger_metal", rgba=(0.72, 0.75, 0.78, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.56, 0.60, 0.64, 1.0))

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=BEAM_RADIUS, length=FRAME_WIDTH),
        origin=_segment_origin(
            (-FRAME_HALF_WIDTH, 0.0, BEAM_Z),
            (FRAME_HALF_WIDTH, 0.0, BEAM_Z),
        ),
        material=frame_metal,
        name="top_beam",
    )

    left_top = (-FRAME_HALF_WIDTH, 0.0, BEAM_Z)
    right_top = (FRAME_HALF_WIDTH, 0.0, BEAM_Z)
    left_front_foot = (-FRAME_HALF_WIDTH, FRAME_HALF_DEPTH, 0.0)
    left_back_foot = (-FRAME_HALF_WIDTH, -FRAME_HALF_DEPTH, 0.0)
    right_front_foot = (FRAME_HALF_WIDTH, FRAME_HALF_DEPTH, 0.0)
    right_back_foot = (FRAME_HALF_WIDTH, -FRAME_HALF_DEPTH, 0.0)

    for name, start, end in (
        ("left_front_leg", left_top, left_front_foot),
        ("left_back_leg", left_top, left_back_foot),
        ("right_front_leg", right_top, right_front_foot),
        ("right_back_leg", right_top, right_back_foot),
    ):
        frame.visual(
            Cylinder(radius=LEG_RADIUS, length=_segment_length(start, end)),
            origin=_segment_origin(start, end),
            material=frame_metal,
            name=name,
        )

    for name, x_side in (("left_side_brace", -FRAME_HALF_WIDTH), ("right_side_brace", FRAME_HALF_WIDTH)):
        brace_start = (x_side, -SIDE_BRACE_HALF_SPAN, SIDE_BRACE_Z)
        brace_end = (x_side, SIDE_BRACE_HALF_SPAN, SIDE_BRACE_Z)
        frame.visual(
            Cylinder(radius=BRACE_RADIUS, length=_segment_length(brace_start, brace_end)),
            origin=_segment_origin(brace_start, brace_end),
            material=frame_metal,
            name=name,
        )

    for name, pivot in (
        ("left_mount", LEFT_PIVOT_WORLD),
        ("center_mount", CENTER_PIVOT_WORLD),
        ("right_mount", RIGHT_PIVOT_WORLD),
    ):
        beam_underside_z = BEAM_Z - BEAM_RADIUS
        strap_bottom_z = pivot[2] + (CLEVIS_TAB_HEIGHT / 2.0) - 0.003
        strap_top_z = beam_underside_z + 0.006
        strap_height = strap_top_z - strap_bottom_z
        frame.visual(
            Box((CLEVIS_TAB_THICKNESS, CLEVIS_TAB_DEPTH, CLEVIS_TAB_HEIGHT)),
            origin=Origin(xyz=(pivot[0] - TAB_X_OFFSET, pivot[1], pivot[2])),
            material=mount_metal,
            name=f"{name}_left_tab",
        )
        frame.visual(
            Box((CLEVIS_TAB_THICKNESS, CLEVIS_TAB_DEPTH, CLEVIS_TAB_HEIGHT)),
            origin=Origin(xyz=(pivot[0] + TAB_X_OFFSET, pivot[1], pivot[2])),
            material=mount_metal,
            name=f"{name}_right_tab",
        )
        frame.visual(
            Box((CLEVIS_TAB_THICKNESS, MOUNT_STRAP_DEPTH, strap_height)),
            origin=Origin(xyz=(pivot[0] - TAB_X_OFFSET, pivot[1], (strap_bottom_z + strap_top_z) / 2.0)),
            material=mount_metal,
            name=f"{name}_left_strap",
        )
        frame.visual(
            Box((CLEVIS_TAB_THICKNESS, MOUNT_STRAP_DEPTH, strap_height)),
            origin=Origin(xyz=(pivot[0] + TAB_X_OFFSET, pivot[1], (strap_bottom_z + strap_top_z) / 2.0)),
            material=mount_metal,
            name=f"{name}_right_strap",
        )

    def add_hanger(
        part_name: str,
        top_pivot: tuple[float, float, float],
        bottom_anchor: tuple[float, float, float],
    ):
        hanger = model.part(part_name)
        local_bottom = _sub(bottom_anchor, top_pivot)
        hanger.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=_x_axis_cylinder_origin((0.0, 0.0, 0.0)),
            material=mount_metal,
            name="top_pin",
        )
        hanger.visual(
            Cylinder(radius=HANGER_RADIUS, length=_segment_length((0.0, 0.0, 0.0), local_bottom)),
            origin=_segment_origin((0.0, 0.0, 0.0), local_bottom),
            material=hanger_metal,
            name="hanger_link",
        )
        hanger.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=_x_axis_cylinder_origin(local_bottom),
            material=mount_metal,
            name="bottom_pin",
        )
        return hanger

    left_hanger = add_hanger(
        "left_hanger",
        LEFT_PIVOT_WORLD,
        LEFT_ATTACH_WORLD,
    )
    center_hanger = add_hanger(
        "center_hanger",
        CENTER_PIVOT_WORLD,
        CENTER_ATTACH_WORLD,
    )
    right_hanger = add_hanger(
        "right_hanger",
        RIGHT_PIVOT_WORLD,
        RIGHT_ATTACH_WORLD,
    )

    swing_limits = MotionLimits(
        effort=120.0,
        velocity=2.5,
        lower=-0.55,
        upper=0.55,
    )
    for joint_name, child, pivot in (
        ("frame_to_left_hanger", left_hanger, LEFT_PIVOT_WORLD),
        ("frame_to_center_hanger", center_hanger, CENTER_PIVOT_WORLD),
        ("frame_to_right_hanger", right_hanger, RIGHT_PIVOT_WORLD),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=child,
            origin=Origin(xyz=pivot),
            axis=(1.0, 0.0, 0.0),
            motion_limits=swing_limits,
        )

    tire_seat = model.part("tire_seat")
    tire_origin_local = _sub(TIRE_CENTER_WORLD, CENTER_ATTACH_WORLD)
    tire_seat.visual(
        mesh_from_geometry(TorusGeometry(TIRE_MAJOR_RADIUS, TIRE_TUBE_RADIUS), "tire_outer_ring"),
        origin=Origin(xyz=tire_origin_local),
        material=tire_rubber,
        name="tire_ring",
    )

    bracket_strap_bottom_z = (TIRE_CENTER_WORLD[2] + 0.07) - CENTER_ATTACH_WORLD[2]
    bracket_strap_top_z = -(CLEVIS_TAB_HEIGHT / 2.0) + 0.003
    bracket_strap_height = bracket_strap_top_z - bracket_strap_bottom_z

    for name, attach_world in (
        ("rear_bracket", CENTER_ATTACH_WORLD),
        ("left_bracket", LEFT_ATTACH_WORLD),
        ("right_bracket", RIGHT_ATTACH_WORLD),
    ):
        center_local = _sub(attach_world, CENTER_ATTACH_WORLD)
        tire_seat.visual(
            Box((CLEVIS_TAB_THICKNESS, CLEVIS_TAB_DEPTH, CLEVIS_TAB_HEIGHT)),
            origin=Origin(xyz=(center_local[0] - TAB_X_OFFSET, center_local[1], center_local[2])),
            material=mount_metal,
            name=f"{name}_left_tab",
        )
        tire_seat.visual(
            Box((CLEVIS_TAB_THICKNESS, CLEVIS_TAB_DEPTH, CLEVIS_TAB_HEIGHT)),
            origin=Origin(xyz=(center_local[0] + TAB_X_OFFSET, center_local[1], center_local[2])),
            material=mount_metal,
            name=f"{name}_right_tab",
        )
        tire_seat.visual(
            Box((CLEVIS_TAB_THICKNESS, MOUNT_STRAP_DEPTH, bracket_strap_height)),
            origin=Origin(
                xyz=(
                    center_local[0] - TAB_X_OFFSET,
                    center_local[1],
                    (bracket_strap_bottom_z + bracket_strap_top_z) / 2.0,
                )
            ),
            material=mount_metal,
            name=f"{name}_left_strap",
        )
        tire_seat.visual(
            Box((CLEVIS_TAB_THICKNESS, MOUNT_STRAP_DEPTH, bracket_strap_height)),
            origin=Origin(
                xyz=(
                    center_local[0] + TAB_X_OFFSET,
                    center_local[1],
                    (bracket_strap_bottom_z + bracket_strap_top_z) / 2.0,
                )
            ),
            material=mount_metal,
            name=f"{name}_right_strap",
        )

    model.articulation(
        "center_hanger_to_tire",
        ArticulationType.FIXED,
        parent=center_hanger,
        child=tire_seat,
        origin=Origin(xyz=_sub(CENTER_ATTACH_WORLD, CENTER_PIVOT_WORLD)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_hanger = object_model.get_part("left_hanger")
    center_hanger = object_model.get_part("center_hanger")
    right_hanger = object_model.get_part("right_hanger")
    tire_seat = object_model.get_part("tire_seat")

    left_joint = object_model.get_articulation("frame_to_left_hanger")
    center_joint = object_model.get_articulation("frame_to_center_hanger")
    right_joint = object_model.get_articulation("frame_to_right_hanger")
    tire_joint = object_model.get_articulation("center_hanger_to_tire")

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

    ctx.check("frame_present", frame is not None, "missing frame part")
    ctx.check("left_hanger_present", left_hanger is not None, "missing left hanger")
    ctx.check("center_hanger_present", center_hanger is not None, "missing center hanger")
    ctx.check("right_hanger_present", right_hanger is not None, "missing right hanger")
    ctx.check("tire_seat_present", tire_seat is not None, "missing tire seat")
    ctx.check(
        "top_hangers_revolute_about_beam_axis",
        left_joint.axis == (1.0, 0.0, 0.0)
        and center_joint.axis == (1.0, 0.0, 0.0)
        and right_joint.axis == (1.0, 0.0, 0.0),
        f"unexpected axes: left={left_joint.axis}, center={center_joint.axis}, right={right_joint.axis}",
    )
    ctx.check(
        "tire_mount_is_fixed",
        tire_joint.joint_type == ArticulationType.FIXED,
        f"unexpected tire joint type: {tire_joint.joint_type}",
    )

    for name, joint in (
        ("left_hanger_limits", left_joint),
        ("center_hanger_limits", center_joint),
        ("right_hanger_limits", right_joint),
    ):
        limits = joint.motion_limits
        ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and math.isclose(limits.lower, -0.55, abs_tol=1e-9)
            and math.isclose(limits.upper, 0.55, abs_tol=1e-9)
        )
        ctx.check(name, ok, f"unexpected limits for {joint.name}: {limits}")

    ctx.expect_contact(frame, left_hanger, elem_a="left_mount_left_tab", elem_b="top_pin", name="left_hanger_top_mount_contact")
    ctx.expect_contact(frame, center_hanger, elem_a="center_mount_left_tab", elem_b="top_pin", name="center_hanger_top_mount_contact")
    ctx.expect_contact(frame, right_hanger, elem_a="right_mount_left_tab", elem_b="top_pin", name="right_hanger_top_mount_contact")

    ctx.expect_contact(center_hanger, tire_seat, elem_a="bottom_pin", elem_b="rear_bracket_left_tab", name="center_hanger_tire_contact")
    ctx.expect_contact(left_hanger, tire_seat, elem_a="bottom_pin", elem_b="left_bracket_left_tab", name="left_hanger_tire_contact")
    ctx.expect_contact(right_hanger, tire_seat, elem_a="bottom_pin", elem_b="right_bracket_left_tab", name="right_hanger_tire_contact")

    ctx.expect_gap(
        frame,
        tire_seat,
        axis="z",
        min_gap=1.35,
        max_gap=1.70,
        positive_elem="top_beam",
        name="beam_above_tire_clearance",
    )
    ctx.expect_overlap(
        frame,
        tire_seat,
        axes="x",
        min_overlap=0.70,
        elem_a="top_beam",
        elem_b="tire_ring",
        name="tire_centered_under_beam_span",
    )

    synchronized_pose = {
        left_joint: 0.38,
        center_joint: 0.38,
        right_joint: 0.38,
    }
    reverse_pose = {
        left_joint: -0.38,
        center_joint: -0.38,
        right_joint: -0.38,
    }

    with ctx.pose(synchronized_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="forward_swing_no_overlap")
        ctx.fail_if_isolated_parts(name="forward_swing_no_floating")
        ctx.expect_contact(left_hanger, tire_seat, elem_a="bottom_pin", elem_b="left_bracket_left_tab", name="forward_left_contact")
        ctx.expect_contact(center_hanger, tire_seat, elem_a="bottom_pin", elem_b="rear_bracket_left_tab", name="forward_center_contact")
        ctx.expect_contact(right_hanger, tire_seat, elem_a="bottom_pin", elem_b="right_bracket_left_tab", name="forward_right_contact")

    with ctx.pose(reverse_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="reverse_swing_no_overlap")
        ctx.fail_if_isolated_parts(name="reverse_swing_no_floating")
        ctx.expect_contact(left_hanger, tire_seat, elem_a="bottom_pin", elem_b="left_bracket_left_tab", name="reverse_left_contact")
        ctx.expect_contact(center_hanger, tire_seat, elem_a="bottom_pin", elem_b="rear_bracket_left_tab", name="reverse_center_contact")
        ctx.expect_contact(right_hanger, tire_seat, elem_a="bottom_pin", elem_b="right_bracket_left_tab", name="reverse_right_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
