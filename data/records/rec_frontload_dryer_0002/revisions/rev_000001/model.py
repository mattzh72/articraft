from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_front_radius: float,
    inner_back_radius: float,
    depth: float,
    segments: int = 72,
):
    ring = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -depth / 2.0),
            (outer_radius, depth / 2.0),
        ],
        [
            (inner_front_radius, -depth / 2.0),
            (inner_back_radius, depth / 2.0),
        ],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    ring.rotate_x(-math.pi / 2.0)
    return _save_mesh(name, ring)


def _drum_shell_mesh(name: str):
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.244, -0.235),
            (0.244, 0.175),
            (0.184, 0.224),
            (0.086, 0.240),
        ],
        [
            (0.215, -0.215),
            (0.215, 0.165),
            (0.162, 0.204),
            (0.042, 0.220),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )
    shell.rotate_x(-math.pi / 2.0)
    return _save_mesh(name, shell)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_dryer", assets=ASSETS)

    appliance_white = model.material("appliance_white", rgba=(0.95, 0.95, 0.94, 1.0))
    charcoal = model.material("charcoal_trim", rgba=(0.16, 0.17, 0.19, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    steel = model.material("stainless_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    glass = model.material("dryer_glass", rgba=(0.72, 0.82, 0.90, 0.32))
    control_gray = model.material("control_gray", rgba=(0.86, 0.87, 0.89, 1.0))
    black = model.material("black_plastic", rgba=(0.09, 0.09, 0.10, 1.0))

    width = 0.685
    depth = 0.740
    height = 0.950
    shell_t = 0.028
    front_face_y = depth / 2.0
    rear_face_y = -depth / 2.0
    drum_center_z = 0.455
    drum_center_y = -0.010
    opening_outer_diameter = 0.510
    opening_radius = opening_outer_diameter / 2.0
    door_radius = 0.248

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((width, depth, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=appliance_white,
        name="base_plinth",
    )
    cabinet.visual(
        Box((shell_t, depth, height - 0.040)),
        origin=Origin(
            xyz=(-width / 2.0 + shell_t / 2.0, 0.0, 0.040 + (height - 0.040) / 2.0)
        ),
        material=appliance_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((shell_t, depth, height - 0.040)),
        origin=Origin(
            xyz=(width / 2.0 - shell_t / 2.0, 0.0, 0.040 + (height - 0.040) / 2.0)
        ),
        material=appliance_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((width - 2.0 * shell_t, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, height - shell_t / 2.0)),
        material=appliance_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((width - 2.0 * shell_t, shell_t, height - 0.040)),
        origin=Origin(
            xyz=(0.0, rear_face_y + shell_t / 2.0, 0.040 + (height - 0.040) / 2.0)
        ),
        material=appliance_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((0.600, shell_t, 0.170)),
        origin=Origin(xyz=(0.0, front_face_y - shell_t / 2.0, 0.105)),
        material=appliance_white,
        name="lower_fascia",
    )
    cabinet.visual(
        Box((width, shell_t, 0.170)),
        origin=Origin(xyz=(0.0, front_face_y - shell_t / 2.0, 0.805)),
        material=appliance_white,
        name="front_header",
    )
    cabinet.visual(
        Box((0.060, shell_t, 0.475)),
        origin=Origin(xyz=(-0.2845, front_face_y - shell_t / 2.0, drum_center_z)),
        material=appliance_white,
        name="left_stile",
    )
    cabinet.visual(
        Box((0.060, shell_t, 0.475)),
        origin=Origin(xyz=(0.2845, front_face_y - shell_t / 2.0, drum_center_z)),
        material=appliance_white,
        name="right_stile",
    )
    cabinet.visual(
        Box((0.605, 0.050, 0.114)),
        origin=Origin(xyz=(0.0, front_face_y - 0.024, 0.842)),
        material=control_gray,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.090, 0.010, 0.012)),
        origin=Origin(xyz=(-0.245, front_face_y - 0.003, 0.310)),
        material=black,
        name="door_gasket_lower",
    )
    cabinet.visual(
        _ring_mesh(
            "dryer_bezel_ring.obj",
            outer_radius=0.255,
            inner_front_radius=0.218,
            inner_back_radius=0.225,
            depth=0.028,
        ),
        origin=Origin(xyz=(0.0, front_face_y - 0.014, drum_center_z)),
        material=charcoal,
        name="door_bezel",
    )
    cabinet.visual(
        Cylinder(radius=0.068, length=0.020),
        origin=Origin(
            xyz=(0.0, -0.333, drum_center_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_gray,
        name="rear_bearing_plate",
    )
    cabinet.visual(
        Cylinder(radius=0.020, length=0.070),
        origin=Origin(
            xyz=(0.0, -0.296, drum_center_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_gray,
        name="rear_spindle",
    )
    cabinet.visual(
        Box((0.060, 0.030, 0.120)),
        origin=Origin(xyz=(0.0, -0.342, drum_center_z)),
        material=dark_gray,
        name="rear_bearing_support",
    )
    cabinet.visual(
        Cylinder(radius=0.008, length=0.270),
        origin=Origin(xyz=(-0.314, front_face_y, drum_center_z)),
        material=dark_gray,
        name="hinge_pin",
    )
    cabinet.visual(
        Box((0.024, 0.024, 0.080)),
        origin=Origin(xyz=(-0.302, front_face_y - 0.006, drum_center_z + 0.100)),
        material=dark_gray,
        name="upper_hinge_mount",
    )
    cabinet.visual(
        Box((0.024, 0.024, 0.080)),
        origin=Origin(xyz=(-0.302, front_face_y - 0.006, drum_center_z - 0.100)),
        material=dark_gray,
        name="lower_hinge_mount",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(
            xyz=(0.192, front_face_y - 0.017, 0.830),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_gray,
        name="knob_bushing",
    )
    for foot_x in (-0.250, 0.250):
        cabinet.visual(
            Box((0.065, 0.080, 0.020)),
            origin=Origin(xyz=(foot_x, -0.250, 0.010)),
            material=dark_gray,
            name=f"foot_{'left' if foot_x < 0.0 else 'right'}",
        )
    cabinet.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    drum = model.part("drum")
    drum.visual(
        _drum_shell_mesh("dryer_drum_shell.obj"),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.026, length=0.160),
        origin=Origin(xyz=(0.0, -0.180, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="rear_stub",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.310, 0.016, 0.020)),
            origin=Origin(
                xyz=(0.155 * math.cos(angle), -0.190, 0.155 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=dark_gray,
            name=f"rear_spider_arm_{index}",
        )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.050, 0.300, 0.022)),
            origin=Origin(
                xyz=(0.202 * math.cos(angle), 0.0, 0.202 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=steel,
            name=f"baffle_{index}",
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.244, length=0.480),
        mass=9.0,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        _ring_mesh(
            "dryer_door_outer_ring.obj",
            outer_radius=door_radius,
            inner_front_radius=0.180,
            inner_back_radius=0.192,
            depth=0.055,
        ),
        origin=Origin(xyz=(0.314, 0.028, 0.0)),
        material=charcoal,
        name="outer_ring",
    )
    door.visual(
        _ring_mesh(
            "dryer_door_inner_trim.obj",
            outer_radius=0.194,
            inner_front_radius=0.167,
            inner_back_radius=0.171,
            depth=0.018,
        ),
        origin=Origin(xyz=(0.314, 0.022, 0.0)),
        material=appliance_white,
        name="inner_trim",
    )
    door.visual(
        Cylinder(radius=0.172, length=0.008),
        origin=Origin(xyz=(0.314, 0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="window_glass",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(0.0, 0.000, 0.118)),
        material=dark_gray,
        name="upper_knuckle",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(0.0, 0.000, -0.118)),
        material=dark_gray,
        name="lower_knuckle",
    )
    door.visual(
        Box((0.190, 0.016, 0.034)),
        origin=Origin(xyz=(0.098, -0.002, 0.118)),
        material=dark_gray,
        name="upper_hinge_arm",
    )
    door.visual(
        Box((0.190, 0.016, 0.034)),
        origin=Origin(xyz=(0.098, -0.002, -0.118)),
        material=dark_gray,
        name="lower_hinge_arm",
    )
    door.visual(
        Box((0.060, 0.036, 0.108)),
        origin=Origin(xyz=(0.532, 0.030, 0.0)),
        material=dark_gray,
        name="latch_grip",
    )
    door.inertial = Inertial.from_geometry(
        Cylinder(radius=door_radius, length=0.055),
        mass=2.4,
        origin=Origin(xyz=(door_radius, 0.0275, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.0, 0.0275, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_body",
    )
    selector_knob.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="knob_spindle",
    )
    selector_knob.visual(
        Box((0.006, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.031, 0.019)),
        material=control_gray,
        name="knob_indicator",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.018),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0275, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "drum_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, drum_center_y, drum_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.314, front_face_y, drum_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=0.0, upper=1.95),
    )
    model.articulation(
        "selector_turn",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(0.192, front_face_y - 0.017, 0.830)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=-2.4, upper=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    selector_knob = object_model.get_part("selector_knob")

    drum_spin = object_model.get_articulation("drum_spin")
    door_hinge = object_model.get_articulation("door_hinge")
    selector_turn = object_model.get_articulation("selector_turn")

    door_bezel = cabinet.get_visual("door_bezel")
    left_stile = cabinet.get_visual("left_stile")
    left_wall = cabinet.get_visual("left_wall")
    control_panel = cabinet.get_visual("control_panel")
    door_gasket_lower = cabinet.get_visual("door_gasket_lower")
    hinge_pin = cabinet.get_visual("hinge_pin")
    rear_spindle = cabinet.get_visual("rear_spindle")
    upper_hinge_mount = cabinet.get_visual("upper_hinge_mount")
    lower_hinge_mount = cabinet.get_visual("lower_hinge_mount")
    knob_bushing = cabinet.get_visual("knob_bushing")

    drum_shell = drum.get_visual("drum_shell")
    rear_stub = drum.get_visual("rear_stub")

    outer_ring = door.get_visual("outer_ring")
    window_glass = door.get_visual("window_glass")
    upper_knuckle = door.get_visual("upper_knuckle")
    lower_knuckle = door.get_visual("lower_knuckle")
    upper_hinge_arm = door.get_visual("upper_hinge_arm")
    lower_hinge_arm = door.get_visual("lower_hinge_arm")
    latch_grip = door.get_visual("latch_grip")

    knob_body = selector_knob.get_visual("knob_body")
    knob_spindle = selector_knob.get_visual("knob_spindle")

    ctx.allow_overlap(
        drum,
        cabinet,
        elem_a=rear_stub,
        elem_b=rear_spindle,
        reason="rear drum stub intentionally telescopes onto the fixed rear spindle support",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=upper_knuckle,
        elem_b=hinge_pin,
        reason="upper door hinge knuckle intentionally rotates around the fixed hinge pin",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=lower_knuckle,
        elem_b=hinge_pin,
        reason="lower door hinge knuckle intentionally rotates around the fixed hinge pin",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=outer_ring,
        elem_b=door_gasket_lower,
        reason="the soft lower door gasket is represented as a rigid strip and intentionally compresses against the door rim",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=upper_knuckle,
        elem_b=left_stile,
        reason="the upper hinge knuckle sits inside a recessed pocket in the left front stile",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=lower_knuckle,
        elem_b=left_stile,
        reason="the lower hinge knuckle sits inside a recessed pocket in the left front stile",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=upper_knuckle,
        elem_b=left_wall,
        reason="the upper hinge knuckle passes through the left wall hinge recess",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=lower_knuckle,
        elem_b=left_wall,
        reason="the lower hinge knuckle passes through the left wall hinge recess",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=upper_knuckle,
        elem_b=upper_hinge_mount,
        reason="the upper hinge knuckle nests inside the upper hinge mount housing",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=lower_knuckle,
        elem_b=lower_hinge_mount,
        reason="the lower hinge knuckle nests inside the lower hinge mount housing",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=upper_hinge_arm,
        elem_b=upper_hinge_mount,
        reason="the upper hinge arm is simplified as a solid leaf inside the upper hinge mount",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=lower_hinge_arm,
        elem_b=lower_hinge_mount,
        reason="the lower hinge arm is simplified as a solid leaf inside the lower hinge mount",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=upper_hinge_arm,
        elem_b=hinge_pin,
        reason="the upper hinge arm wraps the shared hinge pin in a simplified solid representation",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=lower_hinge_arm,
        elem_b=hinge_pin,
        reason="the lower hinge arm wraps the shared hinge pin in a simplified solid representation",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=upper_hinge_arm,
        elem_b=door_bezel,
        reason="the upper hinge arm tucks behind the bezel in the closed-door pose",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=lower_hinge_arm,
        elem_b=door_bezel,
        reason="the lower hinge arm tucks behind the bezel in the closed-door pose",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=upper_hinge_arm,
        elem_b=left_stile,
        reason="the upper hinge leaf is recessed into the left stile when the door is closed",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=lower_hinge_arm,
        elem_b=left_stile,
        reason="the lower hinge leaf is recessed into the left stile when the door is closed",
    )
    ctx.allow_overlap(
        selector_knob,
        cabinet,
        elem_a=knob_spindle,
        elem_b=knob_bushing,
        reason="the selector knob spindle intentionally nests inside the control-panel bushing",
    )
    ctx.allow_overlap(
        selector_knob,
        cabinet,
        elem_a=knob_spindle,
        elem_b=control_panel,
        reason="the selector knob spindle passes through the control panel opening, which is approximated by an uncut panel solid",
    )
    ctx.allow_overlap(
        selector_knob,
        cabinet,
        elem_a=knob_spindle,
        elem_b=cabinet.get_visual("front_header"),
        reason="the selector knob spindle passes through the front header opening, which is approximated by an uncut front panel solid",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.check(
        "drum_spin_axis_front_to_back",
        tuple(drum_spin.axis) == (0.0, 1.0, 0.0),
        f"expected drum axis (0, 1, 0), got {drum_spin.axis}",
    )
    ctx.check(
        "door_hinge_axis_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        f"expected door axis (0, 0, 1), got {door_hinge.axis}",
    )
    door_limits = door_hinge.motion_limits
    ctx.check(
        "door_hinge_limit_reasonable",
        door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and 1.80 <= door_limits.upper <= 2.05,
        f"unexpected door limits: {door_limits}",
    )
    ctx.check(
        "selector_turn_axis_panel_normal",
        tuple(selector_turn.axis) == (0.0, 1.0, 0.0),
        f"expected selector knob axis (0, 1, 0), got {selector_turn.axis}",
    )

    ctx.expect_origin_gap(
        drum,
        cabinet,
        axis="z",
        min_gap=0.430,
        max_gap=0.480,
        name="drum_center_height",
    )
    ctx.expect_within(
        drum,
        cabinet,
        axes="xz",
        inner_elem=drum_shell,
        name="drum_centered_within_cabinet",
    )
    ctx.expect_gap(
        cabinet,
        drum,
        axis="y",
        min_gap=0.090,
        max_gap=0.135,
        positive_elem=door_bezel,
        negative_elem=drum_shell,
        name="drum_sits_behind_front_bezel",
    )
    ctx.expect_overlap(
        drum,
        door,
        axes="xz",
        min_overlap=0.300,
        elem_a=drum_shell,
        elem_b=window_glass,
        name="door_window_faces_drum_opening",
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=outer_ring,
        negative_elem=door_bezel,
        name="closed_door_seats_on_front_bezel",
    )
    ctx.expect_gap(
        selector_knob,
        cabinet,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=knob_body,
        negative_elem=control_panel,
        name="selector_knob_seats_on_control_panel",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_no_unexpected_overlaps")
        ctx.fail_if_isolated_parts(name="door_closed_no_floating")

    with ctx.pose({door_hinge: 1.95}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_unexpected_overlaps")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.expect_gap(
            cabinet,
            door,
            axis="x",
            min_gap=0.075,
            positive_elem=left_stile,
            negative_elem=latch_grip,
            name="opened_door_swings_left_clear_of_opening",
        )

    with ctx.pose({drum_spin: 2.4}):
        ctx.fail_if_parts_overlap_in_current_pose(name="drum_rotated_no_unexpected_overlaps")
        ctx.fail_if_isolated_parts(name="drum_rotated_no_floating")
        ctx.expect_within(
            drum,
            cabinet,
            axes="xz",
            inner_elem=drum_shell,
            name="drum_remains_centered_when_rotated",
        )

    with ctx.pose({selector_turn: 1.8}):
        ctx.fail_if_parts_overlap_in_current_pose(name="selector_turned_no_unexpected_overlaps")
        ctx.fail_if_isolated_parts(name="selector_turned_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
