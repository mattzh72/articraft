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

CABINET_WIDTH = 0.60
CABINET_DEPTH = 0.64
CABINET_HEIGHT = 0.85
SIDE_PANEL_THICKNESS = 0.018
FRONT_PANEL_THICKNESS = 0.018
DRUM_CENTER_Z = 0.39
DRUM_AXLE_Y = -0.29
DOOR_RING_RADIUS = 0.205
DOOR_GLASS_RADIUS = 0.148
DRAWER_SLOT_WIDTH = 0.208
DRAWER_SLOT_HEIGHT = 0.09
DRAWER_CENTER_X = -0.135
DRAWER_CENTER_Z = 0.755
DRAWER_TRAVEL = 0.11
FRONT_PANEL_CENTER_Y = CABINET_DEPTH / 2.0 - FRONT_PANEL_THICKNESS / 2.0
FRONT_OUTER_Y = CABINET_DEPTH / 2.0
DOOR_HINGE_X = -0.270
DOOR_HINGE_Y = FRONT_OUTER_Y + 0.018
KNOB_CENTER_X = 0.185
KNOB_CENTER_Y = FRONT_OUTER_Y + 0.011
KNOB_CENTER_Z = 0.755


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
    model = ArticulatedObject(name="front_load_washer", assets=ASSETS)

    cabinet_white = model.material("cabinet_white", rgba=(0.97, 0.97, 0.96, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.77, 0.79, 0.81, 1.0))
    drum_gray = model.material("drum_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    rubber_gray = model.material("rubber_gray", rgba=(0.16, 0.17, 0.18, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.70, 0.84, 0.92, 0.35))
    control_black = model.material("control_black", rgba=(0.11, 0.12, 0.13, 1.0))
    drawer_gray = model.material("drawer_gray", rgba=(0.86, 0.87, 0.88, 1.0))

    bezel_ring_mesh = _shell_mesh(
        "washer_front_bezel.obj",
        outer_profile=[(0.206, -0.004), (0.206, 0.004)],
        inner_profile=[(0.188, -0.004), (0.188, 0.004)],
    )
    drum_shell_mesh = _shell_mesh(
        "washer_drum_shell.obj",
        outer_profile=[(0.200, -0.170), (0.215, -0.155), (0.215, 0.150), (0.205, 0.170)],
        inner_profile=[(0.184, -0.162), (0.198, -0.148), (0.198, 0.153), (0.188, 0.166)],
    )
    door_ring_mesh = _shell_mesh(
        "washer_door_ring.obj",
        outer_profile=[(DOOR_RING_RADIUS, -0.014), (DOOR_RING_RADIUS, 0.014)],
        inner_profile=[(DOOR_GLASS_RADIUS, -0.014), (DOOR_GLASS_RADIUS, 0.014)],
    )
    door_seal_mesh = _shell_mesh(
        "washer_door_seal.obj",
        outer_profile=[(0.176, -0.007), (0.170, 0.0), (0.165, 0.007)],
        inner_profile=[(0.150, -0.007), (0.150, 0.007)],
    )
    hinge_sleeve_mesh = _shell_mesh(
        "washer_hinge_sleeve.obj",
        outer_profile=[(0.014, -0.035), (0.014, 0.035)],
        inner_profile=[(0.0105, -0.035), (0.0105, 0.035)],
    )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((SIDE_PANEL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(-CABINET_WIDTH / 2.0 + SIDE_PANEL_THICKNESS / 2.0, 0.0, CABINET_HEIGHT / 2.0)
        ),
        material=cabinet_white,
        name="left_side",
    )
    cabinet.visual(
        Box((SIDE_PANEL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(CABINET_WIDTH / 2.0 - SIDE_PANEL_THICKNESS / 2.0, 0.0, CABINET_HEIGHT / 2.0)
        ),
        material=cabinet_white,
        name="right_side",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_PANEL_THICKNESS, CABINET_DEPTH, SIDE_PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, SIDE_PANEL_THICKNESS / 2.0)),
        material=cabinet_white,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_PANEL_THICKNESS, CABINET_DEPTH, SIDE_PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - SIDE_PANEL_THICKNESS / 2.0)),
        material=cabinet_white,
        name="top_panel",
    )
    cabinet.visual(
        Box(
            (
                CABINET_WIDTH - 2.0 * SIDE_PANEL_THICKNESS,
                SIDE_PANEL_THICKNESS,
                CABINET_HEIGHT - 2.0 * SIDE_PANEL_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(0.0, -CABINET_DEPTH / 2.0 + SIDE_PANEL_THICKNESS / 2.0, CABINET_HEIGHT / 2.0)
        ),
        material=cabinet_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_PANEL_THICKNESS, FRONT_PANEL_THICKNESS, 0.20)),
        origin=Origin(xyz=(0.0, FRONT_PANEL_CENTER_Y, 0.10)),
        material=cabinet_white,
        name="front_lower_band",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_PANEL_THICKNESS, FRONT_PANEL_THICKNESS, 0.13)),
        origin=Origin(xyz=(0.0, FRONT_PANEL_CENTER_Y, 0.645)),
        material=cabinet_white,
        name="front_mid_band",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_PANEL_THICKNESS, FRONT_PANEL_THICKNESS, 0.05)),
        origin=Origin(xyz=(0.0, FRONT_PANEL_CENTER_Y, 0.825)),
        material=cabinet_white,
        name="front_top_band",
    )
    cabinet.visual(
        Box((0.092, FRONT_PANEL_THICKNESS, 0.38)),
        origin=Origin(xyz=(-0.236, FRONT_PANEL_CENTER_Y, DRUM_CENTER_Z)),
        material=cabinet_white,
        name="front_left_jamb",
    )
    cabinet.visual(
        Box((0.092, FRONT_PANEL_THICKNESS, 0.38)),
        origin=Origin(xyz=(0.236, FRONT_PANEL_CENTER_Y, DRUM_CENTER_Z)),
        material=cabinet_white,
        name="front_right_jamb",
    )
    cabinet.visual(
        Box((0.043, FRONT_PANEL_THICKNESS, DRAWER_SLOT_HEIGHT)),
        origin=Origin(xyz=(-0.2605, FRONT_PANEL_CENTER_Y, DRAWER_CENTER_Z)),
        material=cabinet_white,
        name="drawer_slot_left_frame",
    )
    cabinet.visual(
        Box((0.313, FRONT_PANEL_THICKNESS, DRAWER_SLOT_HEIGHT)),
        origin=Origin(xyz=(0.1255, FRONT_PANEL_CENTER_Y, DRAWER_CENTER_Z)),
        material=cabinet_white,
        name="drawer_slot_right_frame",
    )
    cabinet.visual(
        bezel_ring_mesh,
        origin=Origin(xyz=(0.0, 0.315, DRUM_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cabinet_white,
        name="front_bezel",
    )
    cabinet.visual(
        Box((0.110, 0.010, 0.042)),
        origin=Origin(xyz=(0.020, FRONT_OUTER_Y + 0.005, DRAWER_CENTER_Z)),
        material=control_black,
        name="display_panel",
    )
    cabinet.visual(
        Box((0.043, 0.010, 0.024)),
        origin=Origin(xyz=(-0.2805, 0.32375, DRUM_CENTER_Z)),
        material=trim_gray,
        name="hinge_pin_bridge",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.162),
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DRUM_CENTER_Z)),
        material=trim_gray,
        name="hinge_pin",
    )
    cabinet.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.0, -0.294, DRUM_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="rear_bearing",
    )
    cabinet.visual(
        Box((0.008, 0.190, 0.050)),
        origin=Origin(xyz=(-0.235, 0.220, DRAWER_CENTER_Z)),
        material=drawer_gray,
        name="left_drawer_rail",
    )
    cabinet.visual(
        Box((0.008, 0.190, 0.050)),
        origin=Origin(xyz=(-0.035, 0.220, DRAWER_CENTER_Z)),
        material=drawer_gray,
        name="right_drawer_rail",
    )
    cabinet.visual(
        Box((0.560, 0.030, 0.080)),
        origin=Origin(xyz=(0.0, FRONT_OUTER_Y - 0.010, 0.040)),
        material=control_black,
        name="toe_kick",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT / 2.0)),
    )

    drum = model.part("drum")
    drum.visual(
        drum_shell_mesh,
        origin=Origin(xyz=(0.0, 0.175, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=drum_gray,
        name="shell",
    )
    drum.visual(
        Cylinder(radius=0.034, length=0.100),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.186, length=0.012),
        origin=Origin(xyz=(0.0, 0.017, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=drum_gray,
        name="rear_spider_disc",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.215, length=0.350),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.175, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    drawer = model.part("detergent_drawer")
    drawer.visual(
        Box((0.1925, 0.140, 0.060)),
        origin=Origin(),
        material=drawer_gray,
        name="drawer_body",
    )
    drawer.visual(
        Box((0.200, 0.015, 0.080)),
        origin=Origin(xyz=(0.0, 0.0775, 0.0)),
        material=cabinet_white,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.110, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.086, 0.0)),
        material=trim_gray,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.200, 0.155, 0.080)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.005, 0.0)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="knob_body",
    )
    selector_knob.visual(
        Box((0.004, 0.024, 0.015)),
        origin=Origin(xyz=(0.0, 0.012, 0.026)),
        material=control_black,
        name="knob_pointer",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.022),
        mass=0.25,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        door_ring_mesh,
        origin=Origin(xyz=(0.270, 0.000, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="outer_ring",
    )
    door.visual(
        Cylinder(radius=DOOR_GLASS_RADIUS, length=0.012),
        origin=Origin(xyz=(0.270, 0.000, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass_blue,
        name="glass_window",
    )
    door.visual(
        door_seal_mesh,
        origin=Origin(xyz=(0.270, -0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber_gray,
        name="seal_ring",
    )
    door.visual(
        hinge_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=trim_gray,
        name="upper_hinge_sleeve",
    )
    door.visual(
        hinge_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=trim_gray,
        name="lower_hinge_sleeve",
    )
    door.visual(
        Box((0.124, 0.012, 0.028)),
        origin=Origin(xyz=(0.073, 0.0, 0.055)),
        material=trim_gray,
        name="upper_hinge_arm",
    )
    door.visual(
        Box((0.124, 0.012, 0.028)),
        origin=Origin(xyz=(0.073, 0.0, -0.055)),
        material=trim_gray,
        name="lower_hinge_arm",
    )
    door.visual(
        Box((0.036, 0.020, 0.082)),
        origin=Origin(xyz=(0.425, 0.018, 0.0)),
        material=trim_gray,
        name="pull_handle",
    )
    door.inertial = Inertial.from_geometry(
        Cylinder(radius=DOOR_RING_RADIUS, length=0.040),
        mass=4.0,
        origin=Origin(xyz=(0.270, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
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
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(DRAWER_CENTER_X, 0.245, DRAWER_CENTER_Z)),
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
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drawer = object_model.get_part("detergent_drawer")
    selector_knob = object_model.get_part("selector_knob")

    drum_axle = object_model.get_articulation("cabinet_to_drum")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    drawer_slide = object_model.get_articulation("cabinet_to_drawer")
    knob_joint = object_model.get_articulation("cabinet_to_selector_knob")

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
    drawer_face = drawer.get_visual("drawer_face")
    knob_body = selector_knob.get_visual("knob_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_isolated_part(
        door,
        reason="Door hangs from a modeled pin-and-sleeve hinge with intentional running clearance.",
    )
    ctx.allow_isolated_part(
        drawer,
        reason="Drawer rides on hidden slide clearances; support is represented by the articulated guide path.",
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

    ctx.expect_overlap(door, cabinet, axes="xz", min_overlap=0.34, elem_a=door_ring, elem_b=front_bezel)
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        min_gap=-0.012,
        max_gap=-0.006,
        positive_elem=door_seal,
        negative_elem=front_bezel,
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        min_gap=0.003,
        max_gap=0.012,
        positive_elem=door_ring,
        negative_elem=front_bezel,
    )
    ctx.expect_overlap(door, cabinet, axes="z", min_overlap=0.06, elem_a=upper_hinge_sleeve, elem_b=hinge_pin)
    ctx.expect_overlap(door, cabinet, axes="z", min_overlap=0.06, elem_a=lower_hinge_sleeve, elem_b=hinge_pin)

    ctx.expect_contact(drum, cabinet, elem_a=drum_hub, elem_b=rear_bearing)
    ctx.expect_overlap(drum, door, axes="xz", min_overlap=0.28, elem_a=drum_shell, elem_b=door_glass)
    ctx.expect_within(drum, cabinet, axes="xz", margin=0.0)

    ctx.expect_within(drawer, cabinet, axes="xz", margin=0.0)
    ctx.expect_overlap(drawer, cabinet, axes="z", min_overlap=0.05, elem_a=drawer_body, elem_b=drawer_slot_left_frame)
    ctx.expect_overlap(drawer, cabinet, axes="z", min_overlap=0.05, elem_a=drawer_body, elem_b=drawer_slot_right_frame)

    ctx.expect_contact(selector_knob, cabinet, elem_a=knob_body, elem_b="drawer_slot_right_frame")

    door_closed_aabb = ctx.part_world_aabb(door)
    drawer_closed_pos = ctx.part_world_position(drawer)
    assert door_closed_aabb is not None
    assert drawer_closed_pos is not None

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
            details=f"expected open door max y > {door_closed_aabb[1][1] + 0.18:.3f}, got {door_open_aabb[1][1]:.3f}",
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
            details=f"expected drawer to move forward by >0.10 m, got {drawer_open_pos[1] - drawer_closed_pos[1]:.3f} m",
        )

    knob_limits = knob_joint.motion_limits
    assert knob_limits is not None
    assert knob_limits.lower is not None
    assert knob_limits.upper is not None
    for label, value in (("lower", knob_limits.lower), ("upper", knob_limits.upper)):
        with ctx.pose({knob_joint: value}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"knob_{label}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"knob_{label}_no_floating")
            ctx.expect_contact(selector_knob, cabinet, elem_a=knob_body, elem_b="drawer_slot_right_frame")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
