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


WALL_PLATE_THICKNESS = 0.012
WALL_PLATE_WIDTH = 0.170
WALL_PLATE_HEIGHT = 0.420
WALL_PIVOT_X = 0.030

FIRST_ARM_LENGTH = 0.470
SECOND_ARM_LENGTH = 0.300
HEAD_FRAME_OFFSET_X = 0.074


def _vertical_collar(radius: float, height: float, *, x: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, 0.0, z0 - height / 2.0))


def _y_axis_collar(
    radius: float,
    length: float,
    *,
    x: float,
    y0: float,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((x, y0 - length / 2.0, z))


def _beam_box(
    length: float,
    width: float,
    height: float,
    *,
    x: float,
    z: float,
    fillet: float,
    y: float = 0.0,
) -> cq.Workplane:
    beam = cq.Workplane("XY").box(length, width, height).translate((x, y, z))
    return beam.edges("|X").fillet(fillet)


def _box_span(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(x1 - x0, y1 - y0, z1 - z0)
        .translate(((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0))
    )


def _pivot_tongue(
    *,
    x_center: float,
    length: float,
    width: float,
    height: float,
    drum_radius: float,
    z: float = 0.0,
) -> cq.Workplane:
    tongue = _beam_box(length, width, height, x=x_center, y=0.0, z=z, fillet=0.003)
    drum = cq.Workplane("XY").circle(drum_radius).extrude(height).translate((0.0, 0.0, z - height / 2.0))
    return tongue.union(drum)


def _pivot_fork(
    *,
    x_center: float,
    length: float,
    gap_half: float,
    tab_width: float,
    height: float,
    z: float = 0.0,
) -> cq.Workplane:
    tab_center = gap_half + tab_width / 2.0
    left_tab = _beam_box(length, tab_width, height, x=x_center, y=-tab_center, z=z, fillet=0.003)
    right_tab = _beam_box(length, tab_width, height, x=x_center, y=tab_center, z=z, fillet=0.003)
    bridge = _beam_box(
        length * 0.64,
        2.0 * (gap_half + tab_width),
        height * 0.55,
        x=x_center - length * 0.20,
        y=0.0,
        z=z - height * 0.18,
        fillet=0.0025,
    )
    return left_tab.union(right_tab).union(bridge)


def _make_wall_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(WALL_PLATE_THICKNESS, WALL_PLATE_WIDTH, WALL_PLATE_HEIGHT)
    plate = plate.edges("|X").fillet(0.004)

    slot_points = [
        (-0.048, 0.135),
        (0.048, 0.135),
        (-0.048, -0.135),
        (0.048, -0.135),
    ]
    slot_cuts = (
        cq.Workplane("YZ")
        .pushPoints(slot_points)
        .slot2D(0.050, 0.012, angle=90)
        .extrude(WALL_PLATE_THICKNESS + 0.020, both=True)
    )
    plate = plate.cut(slot_cuts)

    upper_tower = _box_span(0.006, 0.024, -0.032, 0.032, 0.006, 0.118)
    lower_tower = _box_span(0.006, 0.022, -0.028, 0.028, -0.072, -0.012)
    center_spine = _box_span(0.006, 0.018, -0.020, 0.020, -0.118, 0.118)
    rear_rib_top = _box_span(0.000, 0.006, -0.060, 0.060, 0.102, 0.130)
    rear_rib_bottom = _box_span(0.000, 0.006, -0.060, 0.060, -0.130, -0.102)
    pivot_pad = _box_span(0.020, WALL_PIVOT_X, -0.011, 0.011, -0.012, 0.012)
    left_ear = _box_span(0.018, WALL_PIVOT_X, -0.021, -0.011, -0.026, 0.026)
    right_ear = _box_span(0.018, WALL_PIVOT_X, 0.011, 0.021, -0.026, 0.026)

    return plate.union(upper_tower).union(lower_tower).union(center_spine).union(rear_rib_top).union(rear_rib_bottom).union(pivot_pad).union(left_ear).union(right_ear)


def _make_arm_shape(length: float, *, beam_width: float, beam_height: float, drum_radius: float) -> cq.Workplane:
    del drum_radius
    base_tongue = _box_span(0.000, 0.028, -0.011, 0.011, -0.022, 0.022)
    base_shoulder = _box_span(0.028, 0.090, -beam_width * 0.60, beam_width * 0.60, -beam_height * 0.70, beam_height * 0.70)
    main_beam = _box_span(0.090, length - 0.074, -beam_width / 2.0, beam_width / 2.0, -beam_height / 2.0, beam_height / 2.0)
    top_cap = _box_span(0.135, length - 0.108, -beam_width * 0.28, beam_width * 0.28, beam_height * 0.15, beam_height * 0.45)
    tip_shoulder = _box_span(length - 0.074, length - 0.028, -beam_width * 0.55, beam_width * 0.55, -beam_height * 0.70, beam_height * 0.70)
    pivot_face = _box_span(length - 0.028, length, -0.011, 0.011, -0.012, 0.012)
    left_tip_ear = _box_span(length - 0.028, length, -0.021, -0.011, -0.024, 0.024)
    right_tip_ear = _box_span(length - 0.028, length, 0.011, 0.021, -0.024, 0.024)

    return base_tongue.union(base_shoulder).union(main_beam).union(tip_shoulder).union(top_cap).union(pivot_face).union(left_tip_ear).union(right_tip_ear)


def _make_head_frame_shape() -> cq.Workplane:
    rear_tongue = _box_span(0.000, 0.028, -0.011, 0.011, -0.022, 0.022)
    neck = _box_span(0.028, 0.058, -0.018, 0.018, -0.016, 0.016)
    left_cheek = _box_span(HEAD_FRAME_OFFSET_X - 0.004, HEAD_FRAME_OFFSET_X + 0.004, -0.104, -0.092, -0.056, 0.056)
    right_cheek = _box_span(HEAD_FRAME_OFFSET_X - 0.004, HEAD_FRAME_OFFSET_X + 0.004, 0.092, 0.104, -0.056, 0.056)
    left_side = _box_span(0.056, 0.068, -0.104, -0.092, -0.094, 0.094)
    right_side = _box_span(0.056, 0.068, 0.092, 0.104, -0.094, 0.094)
    top_bar = _box_span(0.056, 0.068, -0.104, 0.104, 0.088, 0.100)
    bottom_bar = _box_span(0.056, 0.068, -0.104, 0.104, -0.100, -0.088)
    left_link = _box_span(0.040, 0.058, -0.090, -0.074, -0.026, 0.026)
    right_link = _box_span(0.040, 0.058, 0.074, 0.090, -0.026, 0.026)

    return rear_tongue.union(neck).union(left_cheek).union(right_cheek).union(left_side).union(right_side).union(top_bar).union(bottom_bar).union(left_link).union(right_link)


def _make_tilt_cradle_shape() -> cq.Workplane:
    plate_x = 0.128
    plate = cq.Workplane("XY").box(0.008, 0.156, 0.136).translate((plate_x, 0.0, 0.0))
    plate = plate.edges("|X").fillet(0.004)

    slot_points = [
        (-0.046, 0.042),
        (0.046, 0.042),
        (-0.046, -0.042),
        (0.046, -0.042),
    ]
    slot_cuts = (
        cq.Workplane("YZ")
        .workplane(offset=plate_x)
        .pushPoints(slot_points)
        .slot2D(0.036, 0.010, angle=90)
        .extrude(0.020, both=True)
    )
    plate = plate.cut(slot_cuts)

    left_trunnion = _box_span(0.004, 0.016, -0.092, -0.080, -0.012, 0.012)
    right_trunnion = _box_span(0.004, 0.016, 0.080, 0.092, -0.012, 0.012)
    left_link = _box_span(0.016, 0.106, -0.075, -0.061, -0.037, 0.037)
    right_link = _box_span(0.016, 0.106, 0.061, 0.075, -0.037, 0.037)
    top_lip = _box_span(plate_x, plate_x + 0.012, -0.067, 0.067, 0.064, 0.074)
    bottom_lip = _box_span(plate_x, plate_x + 0.012, -0.067, 0.067, -0.074, -0.064)

    return plate.union(left_trunnion).union(right_trunnion).union(left_link).union(right_link).union(top_lip).union(bottom_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_tv_wall_mount")

    powder_coat = model.material("powder_coat_black", rgba=(0.12, 0.13, 0.15, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.60, 0.62, 0.66, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_make_wall_plate_shape(), "wall_plate"),
        material=powder_coat,
        name="wall_plate_shell",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.060, WALL_PLATE_WIDTH, WALL_PLATE_HEIGHT)),
        mass=4.8,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    first_arm = model.part("first_arm")
    first_arm.visual(
        mesh_from_cadquery(_make_arm_shape(FIRST_ARM_LENGTH, beam_width=0.056, beam_height=0.024, drum_radius=0.026), "first_arm"),
        material=powder_coat,
        name="first_arm_shell",
    )
    first_arm.inertial = Inertial.from_geometry(
        Box((FIRST_ARM_LENGTH, 0.070, 0.052)),
        mass=2.3,
        origin=Origin(xyz=(FIRST_ARM_LENGTH / 2.0, 0.0, 0.008)),
    )

    second_arm = model.part("second_arm")
    second_arm.visual(
        mesh_from_cadquery(
            _make_arm_shape(SECOND_ARM_LENGTH, beam_width=0.050, beam_height=0.022, drum_radius=0.024),
            "second_arm",
        ),
        material=powder_coat,
        name="second_arm_shell",
    )
    second_arm.inertial = Inertial.from_geometry(
        Box((SECOND_ARM_LENGTH, 0.062, 0.048)),
        mass=1.6,
        origin=Origin(xyz=(SECOND_ARM_LENGTH / 2.0, 0.0, 0.008)),
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        mesh_from_cadquery(_make_head_frame_shape(), "head_frame"),
        material=powder_coat,
        name="head_frame_shell",
    )
    head_frame.inertial = Inertial.from_geometry(
        Box((0.120, 0.230, 0.220)),
        mass=1.4,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
    )

    tilt_cradle = model.part("tilt_cradle")
    tilt_cradle.visual(
        mesh_from_cadquery(_make_tilt_cradle_shape(), "tilt_cradle"),
        material=satin_steel,
        name="tilt_cradle_shell",
    )
    tilt_cradle.inertial = Inertial.from_geometry(
        Box((0.050, 0.170, 0.140)),
        mass=0.9,
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
    )

    model.articulation(
        "wall_to_first_arm",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=first_arm,
        origin=Origin(xyz=(WALL_PIVOT_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "first_to_second_arm",
        ArticulationType.REVOLUTE,
        parent=first_arm,
        child=second_arm,
        origin=Origin(xyz=(FIRST_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.7, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "second_to_head",
        ArticulationType.REVOLUTE,
        parent=second_arm,
        child=head_frame,
        origin=Origin(xyz=(SECOND_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.8, lower=-2.2, upper=2.2),
    )
    model.articulation(
        "head_to_tilt_cradle",
        ArticulationType.REVOLUTE,
        parent=head_frame,
        child=tilt_cradle,
        origin=Origin(xyz=(HEAD_FRAME_OFFSET_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-0.35, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    first_arm = object_model.get_part("first_arm")
    second_arm = object_model.get_part("second_arm")
    head_frame = object_model.get_part("head_frame")
    tilt_cradle = object_model.get_part("tilt_cradle")

    wall_to_first = object_model.get_articulation("wall_to_first_arm")
    first_to_second = object_model.get_articulation("first_to_second_arm")
    second_to_head = object_model.get_articulation("second_to_head")
    head_to_tilt = object_model.get_articulation("head_to_tilt_cradle")

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

    ctx.check(
        "required_parts_present",
        all(
            object_model.get_part(name) is not None
            for name in ("wall_plate", "first_arm", "second_arm", "head_frame", "tilt_cradle")
        ),
    )
    ctx.check(
        "required_articulations_present",
        all(
            object_model.get_articulation(name) is not None
            for name in (
                "wall_to_first_arm",
                "first_to_second_arm",
                "second_to_head",
                "head_to_tilt_cradle",
            )
        ),
    )
    ctx.check(
        "joint_axes_match_prompt",
        wall_to_first.axis == (0.0, 0.0, 1.0)
        and first_to_second.axis == (0.0, 0.0, 1.0)
        and second_to_head.axis == (0.0, 0.0, 1.0)
        and head_to_tilt.axis == (0.0, 1.0, 0.0),
        details=(
            f"axes were {wall_to_first.axis}, {first_to_second.axis}, "
            f"{second_to_head.axis}, {head_to_tilt.axis}"
        ),
    )

    ctx.expect_contact(first_arm, wall_plate, contact_tol=0.0015, name="wall_pivot_is_bearing")
    ctx.expect_contact(second_arm, first_arm, contact_tol=0.0015, name="elbow_pivot_is_bearing")
    ctx.expect_contact(head_frame, second_arm, contact_tol=0.0015, name="head_swivel_is_bearing")
    ctx.expect_contact(tilt_cradle, head_frame, contact_tol=0.0015, name="tilt_trunnions_touch_frame")

    ctx.expect_origin_gap(first_arm, wall_plate, axis="x", min_gap=0.020, max_gap=0.040, name="wall_standoff_depth")
    ctx.expect_origin_gap(second_arm, first_arm, axis="x", min_gap=0.440, max_gap=0.500, name="first_arm_reach")
    ctx.expect_origin_gap(head_frame, second_arm, axis="x", min_gap=0.270, max_gap=0.330, name="second_arm_reach")

    ctx.expect_overlap(head_frame, tilt_cradle, axes="yz", min_overlap=0.120, name="cradle_stays_inside_head_frame")

    with ctx.pose(
        wall_to_first_arm=0.55,
        first_to_second_arm=0.25,
        second_to_head=-0.15,
        head_to_tilt_cradle=0.12,
    ):
        head_pos = ctx.part_world_position(head_frame)
        second_pos = ctx.part_world_position(second_arm)
        ctx.check(
            "reach_pose_moves_head_off_wall",
            head_pos is not None
            and second_pos is not None
            and head_pos[0] > 0.55
            and abs(head_pos[1]) > 0.30
            and second_pos[0] > 0.35,
            details=f"head_pos={head_pos}, second_pos={second_pos}",
        )
        ctx.expect_contact(first_arm, wall_plate, contact_tol=0.0015, name="wall_pivot_contact_in_reach_pose")
        ctx.expect_contact(second_arm, first_arm, contact_tol=0.0015, name="elbow_contact_in_reach_pose")
        ctx.expect_contact(head_frame, second_arm, contact_tol=0.0015, name="head_swivel_contact_in_reach_pose")
        ctx.expect_contact(tilt_cradle, head_frame, contact_tol=0.0015, name="tilt_contact_in_reach_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
