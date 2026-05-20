from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FRAME_W = 0.86
FRAME_H = 0.62
FRAME_FRONT_D = 0.028
FRAME_BAR = 0.045
MULLION_W = 0.035
DRUM_OUTER_R = 0.225
DRUM_INNER_R = 0.205
DRUM_D = 0.18
HINGE_BARREL_R = 0.006
HINGE_Y = FRAME_FRONT_D + HINGE_BARREL_R
HINGE_Z = FRAME_H / 2.0 - FRAME_BAR
PANEL_T = 0.018
OPENING_W = (FRAME_W - (2.0 * FRAME_BAR) - MULLION_W) / 2.0
PANEL_W = OPENING_W - 0.006
PANEL_H = FRAME_H - (2.0 * FRAME_BAR) - 0.008
LEFT_PANEL_X = -((MULLION_W / 2.0) + (OPENING_W / 2.0))
RIGHT_PANEL_X = -LEFT_PANEL_X
DRUM_CENTER_Y = -0.090
CONTROL_BOX_CENTER = (0.505, 0.040, -0.135)
CONTROL_BOX_SIZE = (0.11, 0.08, 0.13)
KNOB_ORIGIN = (
    CONTROL_BOX_CENTER[0],
    CONTROL_BOX_CENTER[1] + (CONTROL_BOX_SIZE[1] / 2.0),
    CONTROL_BOX_CENTER[2],
)


def _merge_meshes(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _box_mesh(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> MeshGeometry:
    geom = BoxGeometry(size)
    roll, pitch, yaw = rpy
    if abs(roll) > 1e-9:
        geom.rotate_x(roll)
    if abs(pitch) > 1e-9:
        geom.rotate_y(pitch)
    if abs(yaw) > 1e-9:
        geom.rotate_z(yaw)
    geom.translate(*center)
    return geom


def _cylinder_mesh(
    radius: float,
    length: float,
    center: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    *,
    radial_segments: int = 28,
) -> MeshGeometry:
    geom = CylinderGeometry(radius=radius, height=length, radial_segments=radial_segments)
    roll, pitch, yaw = rpy
    if abs(roll) > 1e-9:
        geom.rotate_x(roll)
    if abs(pitch) > 1e-9:
        geom.rotate_y(pitch)
    if abs(yaw) > 1e-9:
        geom.rotate_z(yaw)
    geom.translate(*center)
    return geom


def _build_front_frame_mesh() -> MeshGeometry:
    front_y = FRAME_FRONT_D / 2.0
    return _merge_meshes(
        _box_mesh(
            (FRAME_W, FRAME_FRONT_D, FRAME_BAR),
            (0.0, front_y, (FRAME_H / 2.0) - (FRAME_BAR / 2.0)),
        ),
        _box_mesh(
            (FRAME_W, FRAME_FRONT_D, FRAME_BAR),
            (0.0, front_y, -(FRAME_H / 2.0) + (FRAME_BAR / 2.0)),
        ),
        _box_mesh(
            (FRAME_BAR, FRAME_FRONT_D, FRAME_H - (2.0 * FRAME_BAR)),
            (-(FRAME_W / 2.0) + (FRAME_BAR / 2.0), front_y, 0.0),
        ),
        _box_mesh(
            (FRAME_BAR, FRAME_FRONT_D, FRAME_H - (2.0 * FRAME_BAR)),
            ((FRAME_W / 2.0) - (FRAME_BAR / 2.0), front_y, 0.0),
        ),
        _box_mesh((MULLION_W, FRAME_FRONT_D, FRAME_H - (2.0 * FRAME_BAR)), (0.0, front_y, 0.0)),
    )


def _build_support_brackets_mesh() -> MeshGeometry:
    side_span = ((FRAME_W / 2.0) - FRAME_BAR - DRUM_OUTER_R) + 0.018
    vertical_span = ((FRAME_H / 2.0) - FRAME_BAR - DRUM_OUTER_R) + 0.018
    side_center_x = DRUM_OUTER_R + (side_span / 2.0) - 0.009
    vertical_center_z = DRUM_OUTER_R + (vertical_span / 2.0) - 0.009
    return _merge_meshes(
        _box_mesh((side_span, 0.016, 0.050), (-side_center_x, 0.002, 0.0)),
        _box_mesh((side_span, 0.016, 0.050), (side_center_x, 0.002, 0.0)),
        _box_mesh((0.050, 0.016, vertical_span), (0.0, 0.002, vertical_center_z)),
        _box_mesh((0.050, 0.016, vertical_span), (0.0, 0.002, -vertical_center_z)),
    )


def _build_drum_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.214, -(DRUM_D / 2.0) - 0.010),
            (DRUM_OUTER_R, -(DRUM_D / 2.0)),
            (DRUM_OUTER_R, (DRUM_D / 2.0) - 0.010),
            (0.214, DRUM_D / 2.0),
        ],
        [
            (0.198, -(DRUM_D / 2.0) - 0.010),
            (DRUM_INNER_R, -(DRUM_D / 2.0)),
            (DRUM_INNER_R, (DRUM_D / 2.0) - 0.010),
            (0.198, DRUM_D / 2.0),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    shell.rotate_x(-math.pi / 2.0)
    shell.translate(0.0, DRUM_CENTER_Y, 0.0)
    return shell


def _build_motor_mount_mesh() -> MeshGeometry:
    motor_radius = 0.070
    motor_length = 0.034
    motor_y = DRUM_CENTER_Y - 0.072
    bearing_radius = 0.022
    bearing_length = 0.010
    bearing_y = motor_y + (motor_length / 2.0)
    arm_len = (DRUM_INNER_R - motor_radius) + 0.016
    arm_center = motor_radius + (arm_len / 2.0) - 0.008
    return _merge_meshes(
        _cylinder_mesh(
            motor_radius,
            motor_length,
            (0.0, motor_y, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
            radial_segments=36,
        ),
        _cylinder_mesh(
            bearing_radius,
            bearing_length,
            (0.0, bearing_y, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
            radial_segments=28,
        ),
        _box_mesh((arm_len, 0.012, 0.020), (arm_center, motor_y, 0.0)),
        _box_mesh((arm_len, 0.012, 0.020), (-arm_center, motor_y, 0.0)),
        _box_mesh((0.020, 0.012, arm_len), (0.0, motor_y, arm_center)),
        _box_mesh((0.020, 0.012, arm_len), (0.0, motor_y, -arm_center)),
    )


def _build_panel_outline_mesh() -> MeshGeometry:
    stile_w = 0.028
    rail_h = 0.024
    panel_y = (PANEL_T / 2.0) - HINGE_BARREL_R
    side_height = PANEL_H - rail_h
    side_z = -((PANEL_H + rail_h) / 2.0)
    barrel_len = 0.100
    barrel_xs = (-0.116, 0.0, 0.116)
    geometry = _merge_meshes(
        _box_mesh((PANEL_W, PANEL_T, rail_h), (0.0, panel_y, -(rail_h / 2.0))),
        _box_mesh((PANEL_W, PANEL_T, rail_h), (0.0, panel_y, -PANEL_H + (rail_h / 2.0))),
        _box_mesh((stile_w, PANEL_T, side_height), (-(PANEL_W / 2.0) + (stile_w / 2.0), panel_y, side_z)),
        _box_mesh(((stile_w), PANEL_T, side_height), ((PANEL_W / 2.0) - (stile_w / 2.0), panel_y, side_z)),
    )
    for x_pos in barrel_xs:
        geometry.merge(
            _cylinder_mesh(
                HINGE_BARREL_R,
                barrel_len,
                (x_pos, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
                radial_segments=24,
            )
        )
    return geometry


def _build_panel_slats_mesh() -> MeshGeometry:
    stile_w = 0.028
    slat_len = (PANEL_W - (2.0 * stile_w)) + 0.018
    slat_y = (PANEL_T / 2.0) - HINGE_BARREL_R
    slat_depth = 0.014
    slat_height = 0.016
    tilt = 0.34
    positions = (-0.100, -0.185, -0.270, -0.355, -0.440)
    merged = MeshGeometry()
    for z_pos in positions:
        merged.merge(
            _box_mesh(
                (slat_len, slat_depth, slat_height),
                (0.0, slat_y, z_pos),
                rpy=(tilt, 0.0, 0.0),
            )
        )
    return merged


def _build_fan_assembly_mesh() -> MeshGeometry:
    blade_length = 0.170
    blade_width = 0.052
    blade_thickness = 0.005
    blade_center_radius = 0.102
    blade_pitch = 0.34
    merged = _merge_meshes(
        _cylinder_mesh(0.048, 0.050, (0.0, 0.0, 0.0), radial_segments=36),
        _cylinder_mesh(0.030, 0.030, (0.0, 0.0, 0.028), radial_segments=32),
        _cylinder_mesh(0.012, 0.040, (0.0, 0.0, -0.030), radial_segments=24),
    )
    for angle in (0.0, math.tau / 4.0, math.pi, 3.0 * math.tau / 4.0):
        blade = BoxGeometry((blade_length, blade_width, blade_thickness))
        blade.rotate_x(blade_pitch)
        blade.translate(blade_center_radius, 0.0, 0.0)
        blade.rotate_z(angle)
        merged.merge(blade)
    return merged


def _build_knob_mesh() -> MeshGeometry:
    return _merge_meshes(
        _cylinder_mesh(0.028, 0.022, (0.0, 0.0, 0.011), radial_segments=28),
        _cylinder_mesh(0.020, 0.016, (0.0, 0.0, 0.030), radial_segments=28),
        _box_mesh((0.005, 0.012, 0.014), (0.0, 0.018, 0.030)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="attic_gable_box_fan")

    frame_paint = model.material("frame_paint", rgba=(0.86, 0.84, 0.78, 1.0))
    galvanized = model.material("galvanized", rgba=(0.66, 0.69, 0.72, 1.0))
    dark_motor = model.material("dark_motor", rgba=(0.24, 0.25, 0.28, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.74, 0.77, 0.79, 1.0))
    box_gray = model.material("box_gray", rgba=(0.57, 0.59, 0.60, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("vent_frame")
    frame.visual(
        mesh_from_geometry(_build_front_frame_mesh(), "gable_front_frame"),
        material=frame_paint,
        name="front_frame",
    )
    frame.visual(
        mesh_from_geometry(_build_support_brackets_mesh(), "gable_support_brackets"),
        material=galvanized,
        name="support_brackets",
    )
    frame.visual(
        mesh_from_geometry(_build_drum_shell_mesh(), "gable_drum_shell"),
        material=galvanized,
        name="drum_shell",
    )
    frame.visual(
        mesh_from_geometry(_build_motor_mount_mesh(), "gable_motor_mount"),
        material=dark_motor,
        name="motor_mount",
    )
    frame.visual(
        Box(CONTROL_BOX_SIZE),
        origin=Origin(xyz=CONTROL_BOX_CENTER),
        material=box_gray,
        name="control_box",
    )
    frame.visual(
        Box((0.040, 0.050, 0.040)),
        origin=Origin(xyz=(0.445, 0.028, CONTROL_BOX_CENTER[2])),
        material=box_gray,
        name="control_bracket",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.02, 0.28, 0.72)),
        mass=18.0,
        origin=Origin(xyz=(0.04, -0.01, 0.0)),
    )

    left_flap = model.part("left_louver_flap")
    left_flap.visual(
        mesh_from_geometry(_build_panel_outline_mesh(), "left_louver_outline"),
        material=frame_paint,
        name="panel_outline",
    )
    left_flap.visual(
        mesh_from_geometry(_build_panel_slats_mesh(), "left_louver_slats"),
        material=frame_paint,
        name="panel_slats",
    )
    left_flap.inertial = Inertial.from_geometry(
        Box((PANEL_W, PANEL_T, PANEL_H)),
        mass=1.1,
        origin=Origin(xyz=(0.0, (PANEL_T / 2.0) - HINGE_BARREL_R, -(PANEL_H / 2.0))),
    )

    right_flap = model.part("right_louver_flap")
    right_flap.visual(
        mesh_from_geometry(_build_panel_outline_mesh(), "right_louver_outline"),
        material=frame_paint,
        name="panel_outline",
    )
    right_flap.visual(
        mesh_from_geometry(_build_panel_slats_mesh(), "right_louver_slats"),
        material=frame_paint,
        name="panel_slats",
    )
    right_flap.inertial = Inertial.from_geometry(
        Box((PANEL_W, PANEL_T, PANEL_H)),
        mass=1.1,
        origin=Origin(xyz=(0.0, (PANEL_T / 2.0) - HINGE_BARREL_R, -(PANEL_H / 2.0))),
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        mesh_from_geometry(_build_fan_assembly_mesh(), "gable_fan_blades"),
        material=blade_metal,
        name="blade_sweep",
    )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.190, length=0.060),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(_build_knob_mesh(), "gable_speed_knob"),
        material=knob_black,
        name="knob_body",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.050),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    model.articulation(
        "frame_to_left_louver",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_flap,
        origin=Origin(xyz=(LEFT_PANEL_X, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "frame_to_right_louver",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_flap,
        origin=Origin(xyz=(RIGHT_PANEL_X, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "frame_to_blade_assembly",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, DRUM_CENTER_Y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=20.0),
    )
    model.articulation(
        "frame_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=speed_knob,
        origin=Origin(xyz=KNOB_ORIGIN, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("vent_frame")
    left_flap = object_model.get_part("left_louver_flap")
    right_flap = object_model.get_part("right_louver_flap")
    blade_assembly = object_model.get_part("blade_assembly")
    speed_knob = object_model.get_part("speed_knob")

    left_hinge = object_model.get_articulation("frame_to_left_louver")
    right_hinge = object_model.get_articulation("frame_to_right_louver")
    fan_spin = object_model.get_articulation("frame_to_blade_assembly")
    knob_spin = object_model.get_articulation("frame_to_speed_knob")

    ctx.check(
        "primary parts are authored",
        all(part is not None for part in (frame, left_flap, right_flap, blade_assembly, speed_knob)),
        details="Expected vent frame, both louver flaps, blade assembly, and speed knob.",
    )
    ctx.expect_gap(
        left_flap,
        frame,
        axis="y",
        positive_elem="panel_outline",
        negative_elem="front_frame",
        max_gap=0.003,
        max_penetration=0.0,
        name="left flap seats just proud of the vent frame",
    )
    ctx.expect_gap(
        right_flap,
        frame,
        axis="y",
        positive_elem="panel_outline",
        negative_elem="front_frame",
        max_gap=0.003,
        max_penetration=0.0,
        name="right flap seats just proud of the vent frame",
    )
    ctx.expect_within(
        blade_assembly,
        frame,
        axes="xz",
        inner_elem="blade_sweep",
        outer_elem="drum_shell",
        margin=0.018,
        name="fan blades stay within the drum diameter",
    )
    ctx.expect_overlap(
        blade_assembly,
        frame,
        axes="y",
        elem_a="blade_sweep",
        elem_b="drum_shell",
        min_overlap=0.040,
        name="fan assembly remains inside the drum depth",
    )

    left_closed = ctx.part_element_world_aabb(left_flap, elem="panel_outline")
    right_closed = ctx.part_element_world_aabb(right_flap, elem="panel_outline")
    with ctx.pose({left_hinge: 0.95, right_hinge: 0.95}):
        left_open = ctx.part_element_world_aabb(left_flap, elem="panel_outline")
        right_open = ctx.part_element_world_aabb(right_flap, elem="panel_outline")

    ctx.check(
        "left louver opens outward",
        left_closed is not None
        and left_open is not None
        and left_open[1][1] > left_closed[1][1] + 0.10,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right louver opens outward",
        right_closed is not None
        and right_open is not None
        and right_open[1][1] > right_closed[1][1] + 0.10,
        details=f"closed={right_closed}, open={right_open}",
    )

    ctx.check(
        "fan uses continuous spin articulation",
        fan_spin.articulation_type == ArticulationType.CONTINUOUS
        and fan_spin.motion_limits is not None
        and fan_spin.motion_limits.lower is None
        and fan_spin.motion_limits.upper is None
        and tuple(fan_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={fan_spin.articulation_type}, axis={fan_spin.axis}, limits={fan_spin.motion_limits}",
    )
    ctx.check(
        "speed knob uses continuous rotary articulation",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.motion_limits is not None
        and knob_spin.motion_limits.lower is None
        and knob_spin.motion_limits.upper is None
        and tuple(knob_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={knob_spin.articulation_type}, axis={knob_spin.axis}, limits={knob_spin.motion_limits}",
    )

    knob_pos = ctx.part_world_position(speed_knob)
    ctx.check(
        "speed knob is mounted on the right-side control box",
        knob_pos is not None and knob_pos[0] > 0.45 and knob_pos[1] > 0.07 and knob_pos[2] < -0.05,
        details=f"knob position={knob_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
