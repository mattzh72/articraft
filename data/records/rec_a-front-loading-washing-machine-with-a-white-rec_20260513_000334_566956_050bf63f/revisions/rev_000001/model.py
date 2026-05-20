from __future__ import annotations

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

CABINET_WIDTH = 0.595
CABINET_DEPTH = 0.620
CABINET_HEIGHT = 0.850
WALL_THICKNESS = 0.018

DRUM_CENTER_Z = 0.390
DRUM_AXLE_Y = -0.270
DOOR_RING_RADIUS = 0.205
DOOR_GLASS_RADIUS = 0.150

FRONT_OUTER_Y = CABINET_DEPTH / 2.0
FRONT_PANEL_CENTER_Y = FRONT_OUTER_Y - WALL_THICKNESS / 2.0

DOOR_HINGE_X = -0.268
DOOR_HINGE_Y = FRONT_OUTER_Y + 0.018
DOOR_RING_OFFSET_X = -DOOR_HINGE_X

DRAWER_SLOT_HEIGHT = 0.090
DRAWER_CENTER_X = -0.135
DRAWER_CENTER_Z = 0.755
DRAWER_TRAVEL = 0.115

KNOB_CENTER_X = 0.185
KNOB_CENTER_Y = FRONT_OUTER_Y + 0.011
KNOB_CENTER_Z = 0.755

START_BUTTON_CENTER_X = 0.080
START_BUTTON_CENTER_Y = FRONT_OUTER_Y + 0.006
START_BUTTON_CENTER_Z = 0.755
START_BUTTON_TRAVEL = 0.004


def _shell_mesh(
    filename: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    geometry = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washing_machine", assets=ASSETS)

    cabinet_white = model.material("cabinet_white", rgba=(0.96, 0.96, 0.95, 1.0))
    chrome_gray = model.material("chrome_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.13, 0.14, 0.15, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.65, 0.82, 0.92, 0.35))
    panel_black = model.material("panel_black", rgba=(0.10, 0.11, 0.12, 1.0))
    drawer_gray = model.material("drawer_gray", rgba=(0.86, 0.87, 0.88, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.20, 0.55, 0.85, 1.0))

    front_bezel_mesh = _shell_mesh(
        "front_bezel.obj",
        outer_profile=[(0.215, -0.004), (0.215, 0.004)],
        inner_profile=[(0.190, -0.004), (0.190, 0.004)],
    )
    drum_shell_mesh = _shell_mesh(
        "drum_shell.obj",
        outer_profile=[
            (0.198, -0.170),
            (0.213, -0.155),
            (0.213, 0.150),
            (0.203, 0.170),
        ],
        inner_profile=[
            (0.182, -0.162),
            (0.196, -0.148),
            (0.196, 0.153),
            (0.186, 0.166),
        ],
    )
    door_ring_mesh = _shell_mesh(
        "door_ring.obj",
        outer_profile=[(DOOR_RING_RADIUS, -0.014), (DOOR_RING_RADIUS, 0.014)],
        inner_profile=[(DOOR_GLASS_RADIUS, -0.014), (DOOR_GLASS_RADIUS, 0.014)],
    )
    door_seal_mesh = _shell_mesh(
        "door_seal.obj",
        outer_profile=[(0.178, -0.008), (0.170, 0.0), (0.166, 0.008)],
        inner_profile=[(0.152, -0.008), (0.152, 0.008)],
    )
    hinge_sleeve_mesh = _shell_mesh(
        "hinge_sleeve.obj",
        outer_profile=[(0.014, -0.034), (0.014, 0.034)],
        inner_profile=[(0.0105, -0.034), (0.0105, 0.034)],
    )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(-CABINET_WIDTH / 2.0 + WALL_THICKNESS / 2.0, 0.0, CABINET_HEIGHT / 2.0)
        ),
        material=cabinet_white,
        name="left_side",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(CABINET_WIDTH / 2.0 - WALL_THICKNESS / 2.0, 0.0, CABINET_HEIGHT / 2.0)
        ),
        material=cabinet_white,
        name="right_side",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS, CABINET_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, WALL_THICKNESS / 2.0)),
        material=cabinet_white,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS, CABINET_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - WALL_THICKNESS / 2.0)),
        material=cabinet_white,
        name="top_panel",
    )
    cabinet.visual(
        Box(
            (
                CABINET_WIDTH - 2.0 * WALL_THICKNESS,
                WALL_THICKNESS,
                CABINET_HEIGHT - 2.0 * WALL_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(0.0, -CABINET_DEPTH / 2.0 + WALL_THICKNESS / 2.0, CABINET_HEIGHT / 2.0)
        ),
        material=cabinet_white,
        name="rear_panel",
    )

    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, 0.200)),
        origin=Origin(xyz=(0.0, FRONT_PANEL_CENTER_Y, 0.100)),
        material=cabinet_white,
        name="front_lower_band",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, 0.130)),
        origin=Origin(xyz=(0.0, FRONT_PANEL_CENTER_Y, 0.645)),
        material=cabinet_white,
        name="front_mid_band",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, 0.050)),
        origin=Origin(xyz=(0.0, FRONT_PANEL_CENTER_Y, 0.825)),
        material=cabinet_white,
        name="front_top_band",
    )
    cabinet.visual(
        Box((0.092, WALL_THICKNESS, 0.380)),
        origin=Origin(xyz=(-0.234, FRONT_PANEL_CENTER_Y, DRUM_CENTER_Z)),
        material=cabinet_white,
        name="front_left_jamb",
    )
    cabinet.visual(
        Box((0.092, WALL_THICKNESS, 0.380)),
        origin=Origin(xyz=(0.234, FRONT_PANEL_CENTER_Y, DRUM_CENTER_Z)),
        material=cabinet_white,
        name="front_right_jamb",
    )

    cabinet.visual(
        Box((0.040, WALL_THICKNESS, DRAWER_SLOT_HEIGHT)),
        origin=Origin(xyz=(-0.260, FRONT_PANEL_CENTER_Y, DRAWER_CENTER_Z)),
        material=cabinet_white,
        name="drawer_slot_left_frame",
    )
    cabinet.visual(
        Box((0.314, WALL_THICKNESS, DRAWER_SLOT_HEIGHT)),
        origin=Origin(xyz=(0.122, FRONT_PANEL_CENTER_Y, DRAWER_CENTER_Z)),
        material=cabinet_white,
        name="drawer_slot_right_frame",
    )

    cabinet.visual(
        front_bezel_mesh,
        origin=Origin(
            xyz=(0.0, FRONT_OUTER_Y - 0.005, DRUM_CENTER_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=cabinet_white,
        name="front_bezel",
    )

    cabinet.visual(
        Box((0.046, 0.020, 0.030)),
        origin=Origin(xyz=(-0.281, FRONT_OUTER_Y + 0.006, DRUM_CENTER_Z)),
        material=chrome_gray,
        name="hinge_pin_bridge",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.162),
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DRUM_CENTER_Z)),
        material=chrome_gray,
        name="hinge_pin",
    )
    cabinet.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(
            xyz=(0.0, DRUM_AXLE_Y - 0.005, DRUM_CENTER_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome_gray,
        name="rear_bearing",
    )

    cabinet.visual(
        Box((0.008, 0.180, 0.050)),
        origin=Origin(xyz=(-0.232, 0.215, DRAWER_CENTER_Z)),
        material=drawer_gray,
        name="left_drawer_rail",
    )
    cabinet.visual(
        Box((0.008, 0.180, 0.050)),
        origin=Origin(xyz=(-0.038, 0.215, DRAWER_CENTER_Z)),
        material=drawer_gray,
        name="right_drawer_rail",
    )

    cabinet.visual(
        Box((0.555, 0.030, 0.080)),
        origin=Origin(xyz=(0.0, FRONT_OUTER_Y - 0.010, 0.040)),
        material=panel_black,
        name="toe_kick",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=44.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT / 2.0)),
    )

    drum = model.part("drum")
    drum.visual(
        drum_shell_mesh,
        origin=Origin(xyz=(0.0, 0.175, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="shell",
    )
    drum.visual(
        Cylinder(radius=0.034, length=0.100),
        origin=Origin(xyz=(0.0, 0.052, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome_gray,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.184, length=0.012),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="rear_spider_disc",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.210, length=0.345),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.175, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    drawer = model.part("detergent_drawer")
    drawer.visual(
        Box((0.190, 0.140, 0.060)),
        origin=Origin(),
        material=drawer_gray,
        name="drawer_body",
    )
    drawer.visual(
        Box((0.198, 0.015, 0.080)),
        origin=Origin(xyz=(0.0, 0.077, 0.0)),
        material=cabinet_white,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.112, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.086, 0.0)),
        material=chrome_gray,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.198, 0.155, 0.080)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.005, 0.0)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.034, length=0.022),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome_gray,
        name="knob_body",
    )
    selector_knob.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=panel_black,
        name="knob_cap",
    )
    selector_knob.visual(
        Box((0.004, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.015, 0.024)),
        material=accent_blue,
        name="knob_pointer",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.024),
        mass=0.20,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent_blue,
        name="button_cap",
    )
    start_button.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome_gray,
        name="button_collar",
    )
    start_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.014),
        mass=0.04,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        door_ring_mesh,
        origin=Origin(xyz=(DOOR_RING_OFFSET_X, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome_gray,
        name="outer_ring",
    )
    door.visual(
        Cylinder(radius=DOOR_GLASS_RADIUS, length=0.012),
        origin=Origin(xyz=(DOOR_RING_OFFSET_X, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass_blue,
        name="glass_window",
    )
    door.visual(
        door_seal_mesh,
        origin=Origin(
            xyz=(DOOR_RING_OFFSET_X, -0.022, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)
        ),
        material=rubber_black,
        name="seal_ring",
    )
    door.visual(
        hinge_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=chrome_gray,
        name="upper_hinge_sleeve",
    )
    door.visual(
        hinge_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=chrome_gray,
        name="lower_hinge_sleeve",
    )
    door.visual(
        Box((0.124, 0.012, 0.028)),
        origin=Origin(xyz=(0.074, 0.0, 0.055)),
        material=chrome_gray,
        name="upper_hinge_arm",
    )
    door.visual(
        Box((0.124, 0.012, 0.028)),
        origin=Origin(xyz=(0.074, 0.0, -0.055)),
        material=chrome_gray,
        name="lower_hinge_arm",
    )
    door.visual(
        Box((0.036, 0.022, 0.092)),
        origin=Origin(xyz=(DOOR_RING_OFFSET_X + 0.158, 0.020, 0.0)),
        material=chrome_gray,
        name="pull_handle",
    )
    door.inertial = Inertial.from_geometry(
        Cylinder(radius=DOOR_RING_RADIUS, length=0.044),
        mass=4.2,
        origin=Origin(xyz=(DOOR_RING_OFFSET_X, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, DRUM_AXLE_Y, DRUM_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=16.0),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DRUM_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=0.0,
            upper=2.0,
        ),
    )
    model.articulation(
        "cabinet_to_detergent_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(DRAWER_CENTER_X, FRONT_OUTER_Y - 0.072, DRAWER_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "cabinet_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(KNOB_CENTER_X, KNOB_CENTER_Y, KNOB_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=4.0,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "cabinet_to_start_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=start_button,
        origin=Origin(
            xyz=(START_BUTTON_CENTER_X, START_BUTTON_CENTER_Y, START_BUTTON_CENTER_Z)
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.10,
            lower=0.0,
            upper=START_BUTTON_TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drawer = object_model.get_part("detergent_drawer")
    selector_knob = object_model.get_part("selector_knob")
    start_button = object_model.get_part("start_button")

    drum_axle = object_model.get_articulation("cabinet_to_drum")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    drawer_slide = object_model.get_articulation("cabinet_to_detergent_drawer")
    knob_joint = object_model.get_articulation("cabinet_to_selector_knob")
    button_joint = object_model.get_articulation("cabinet_to_start_button")

    front_bezel = cabinet.get_visual("front_bezel")
    rear_bearing = cabinet.get_visual("rear_bearing")
    hinge_pin = cabinet.get_visual("hinge_pin")
    drawer_slot_left_frame = cabinet.get_visual("drawer_slot_left_frame")
    drawer_slot_right_frame = cabinet.get_visual("drawer_slot_right_frame")

    drum_shell = drum.get_visual("shell")
    drum_hub = drum.get_visual("rear_hub")

    door_ring = door.get_visual("outer_ring")
    door_glass = door.get_visual("glass_window")
    door_seal = door.get_visual("seal_ring")
    upper_hinge_sleeve = door.get_visual("upper_hinge_sleeve")
    lower_hinge_sleeve = door.get_visual("lower_hinge_sleeve")

    drawer_body = drawer.get_visual("drawer_body")
    knob_body = selector_knob.get_visual("knob_body")
    button_cap = start_button.get_visual("button_cap")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_isolated_part(
        door,
        reason="Door hangs from a modeled pin-and-sleeve hinge with running clearance.",
    )
    ctx.allow_isolated_part(
        drawer,
        reason="Drawer rides on hidden slide clearances; support is the articulated guide path.",
    )
    ctx.fail_if_isolated_parts(max_pose_samples=8)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.check(
        "door_hinge_axis_is_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected door hinge axis (0, 0, 1), got {door_hinge.axis}",
    )
    ctx.check(
        "drum_axis_is_front_to_back",
        tuple(drum_axle.axis) == (0.0, 1.0, 0.0),
        details=f"expected drum axis (0, 1, 0), got {drum_axle.axis}",
    )
    ctx.check(
        "drawer_slides_forward",
        tuple(drawer_slide.axis) == (0.0, 1.0, 0.0),
        details=f"expected drawer axis (0, 1, 0), got {drawer_slide.axis}",
    )
    ctx.check(
        "knob_rotates_about_depth_axis",
        tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected knob axis (0, 1, 0), got {knob_joint.axis}",
    )
    ctx.check(
        "drum_is_continuous",
        drum_axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected continuous drum articulation, got {drum_axle.articulation_type}",
    )
    ctx.check(
        "button_pushes_inward",
        tuple(button_joint.axis) == (0.0, -1.0, 0.0),
        details=f"expected button axis (0, -1, 0), got {button_joint.axis}",
    )

    ctx.expect_overlap(
        door, cabinet, axes="xz", min_overlap=0.34, elem_a=door_ring, elem_b=front_bezel
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        min_gap=-0.014,
        max_gap=-0.004,
        positive_elem=door_seal,
        negative_elem=front_bezel,
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        min_gap=0.003,
        max_gap=0.014,
        positive_elem=door_ring,
        negative_elem=front_bezel,
    )
    ctx.expect_overlap(
        door, cabinet, axes="z", min_overlap=0.06, elem_a=upper_hinge_sleeve, elem_b=hinge_pin
    )
    ctx.expect_overlap(
        door, cabinet, axes="z", min_overlap=0.06, elem_a=lower_hinge_sleeve, elem_b=hinge_pin
    )

    ctx.expect_contact(drum, cabinet, elem_a=drum_hub, elem_b=rear_bearing)
    ctx.expect_overlap(
        drum, door, axes="xz", min_overlap=0.28, elem_a=drum_shell, elem_b=door_glass
    )
    ctx.expect_within(drum, cabinet, axes="xz", margin=0.0)

    ctx.expect_within(drawer, cabinet, axes="xz", margin=0.0)
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="z",
        min_overlap=0.05,
        elem_a=drawer_body,
        elem_b=drawer_slot_left_frame,
    )
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="z",
        min_overlap=0.05,
        elem_a=drawer_body,
        elem_b=drawer_slot_right_frame,
    )

    ctx.expect_contact(selector_knob, cabinet, elem_a=knob_body, elem_b=drawer_slot_right_frame)
    ctx.expect_contact(start_button, cabinet, elem_a=button_cap, elem_b=drawer_slot_right_frame)

    door_closed_aabb = ctx.part_world_aabb(door)
    drawer_closed_pos = ctx.part_world_position(drawer)
    button_closed_pos = ctx.part_world_position(start_button)
    assert door_closed_aabb is not None
    assert drawer_closed_pos is not None
    assert button_closed_pos is not None

    with ctx.pose({door_hinge: 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.07,
            positive_elem=door_ring,
            negative_elem=front_bezel,
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="z",
            min_overlap=0.06,
            elem_a=upper_hinge_sleeve,
            elem_b=hinge_pin,
        )
        door_open_aabb = ctx.part_world_aabb(door)
        assert door_open_aabb is not None
        ctx.check(
            "door_swings_clear_of_front",
            door_open_aabb[1][1] > door_closed_aabb[1][1] + 0.18,
            details=(
                f"expected open door max y > {door_closed_aabb[1][1] + 0.18:.3f}, "
                f"got {door_open_aabb[1][1]:.3f}"
            ),
        )

    with ctx.pose({drum_axle: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="drum_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="drum_rotated_no_floating")
        ctx.expect_contact(drum, cabinet, elem_a=drum_hub, elem_b=rear_bearing)
        ctx.expect_within(drum, cabinet, axes="xz", margin=0.0)

    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="drawer_open_no_overlap")
        ctx.fail_if_isolated_parts(name="drawer_open_no_floating")
        ctx.expect_within(drawer, cabinet, axes="xz", margin=0.0)
        drawer_open_pos = ctx.part_world_position(drawer)
        assert drawer_open_pos is not None
        ctx.check(
            "drawer_travel_distance",
            drawer_open_pos[1] > drawer_closed_pos[1] + 0.10,
            details=(
                f"expected drawer to move forward by >0.10 m, "
                f"got {drawer_open_pos[1] - drawer_closed_pos[1]:.3f} m"
            ),
        )

    with ctx.pose({button_joint: START_BUTTON_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="button_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="button_pressed_no_floating")
        button_pressed_pos = ctx.part_world_position(start_button)
        assert button_pressed_pos is not None
        ctx.check(
            "button_travels_inward",
            button_pressed_pos[1] < button_closed_pos[1] - 0.001,
            details=(
                f"expected button to move inward (-y), got delta "
                f"{button_pressed_pos[1] - button_closed_pos[1]:.4f} m"
            ),
        )

    knob_limits = knob_joint.motion_limits
    assert knob_limits is not None
    assert knob_limits.lower is not None
    assert knob_limits.upper is not None
    for label, value in (("lower", knob_limits.lower), ("upper", knob_limits.upper)):
        with ctx.pose({knob_joint: value}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"knob_{label}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"knob_{label}_no_floating")
            ctx.expect_contact(
                selector_knob, cabinet, elem_a=knob_body, elem_b=drawer_slot_right_frame
            )

    return ctx.report()


object_model = build_object_model()
