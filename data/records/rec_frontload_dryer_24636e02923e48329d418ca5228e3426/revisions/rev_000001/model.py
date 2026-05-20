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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CYLINDER_X = (0.0, math.pi / 2.0, 0.0)
CYLINDER_Y = (math.pi / 2.0, 0.0, 0.0)


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh(
    *,
    name: str,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 64,
):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        thickness,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _tube_shell_mesh(
    *,
    name: str,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 32,
):
    half = length * 0.5
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ventless_condensation_dryer")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.96, 1.0))
    warm_white = model.material("warm_white", rgba=(0.92, 0.92, 0.93, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.18, 0.20, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    chrome = model.material("chrome", rgba=(0.83, 0.84, 0.87, 1.0))
    glass = model.material("glass", rgba=(0.60, 0.72, 0.80, 0.25))
    filter_grey = model.material("filter_grey", rgba=(0.56, 0.59, 0.61, 1.0))

    width = 0.60
    depth = 0.48
    height = 0.85
    side_thickness = 0.018
    top_thickness = 0.018
    bottom_thickness = 0.018
    rear_thickness = 0.012
    front_thickness = 0.018
    front_y = depth * 0.5

    door_center_z = 0.48
    door_outer_radius = 0.205
    door_window_radius = 0.132
    bezel_outer_radius = 0.192
    bezel_inner_radius = 0.156
    door_hinge_x = -0.195
    door_hinge_y = 0.255

    drawer_origin_y = 0.227
    drawer_origin_z = 0.104
    drawer_travel = 0.12

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    cabinet.visual(
        Box((side_thickness, depth, height)),
        origin=Origin(xyz=(-width * 0.5 + side_thickness * 0.5, 0.0, height * 0.5)),
        material=body_white,
        name="left_side",
    )
    cabinet.visual(
        Box((side_thickness, depth, height)),
        origin=Origin(xyz=(width * 0.5 - side_thickness * 0.5, 0.0, height * 0.5)),
        material=body_white,
        name="right_side",
    )
    cabinet.visual(
        Box((width, depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, height - top_thickness * 0.5)),
        material=body_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((width, depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness * 0.5)),
        material=warm_white,
        name="bottom_pan",
    )
    cabinet.visual(
        Box((width - 2.0 * side_thickness, rear_thickness, height - top_thickness - bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -depth * 0.5 + rear_thickness * 0.5,
                bottom_thickness + (height - top_thickness - bottom_thickness) * 0.5,
            )
        ),
        material=warm_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((width - 2.0 * side_thickness, front_thickness, 0.140)),
        origin=Origin(xyz=(0.0, front_y - front_thickness * 0.5, 0.779)),
        material=body_white,
        name="top_fascia",
    )
    cabinet.visual(
        Box((0.095, front_thickness, 0.430)),
        origin=Origin(xyz=(-0.2525, front_y - front_thickness * 0.5, 0.465)),
        material=body_white,
        name="left_door_stile",
    )
    cabinet.visual(
        Box((0.095, front_thickness, 0.430)),
        origin=Origin(xyz=(0.2525, front_y - front_thickness * 0.5, 0.465)),
        material=body_white,
        name="right_door_stile",
    )
    cabinet.visual(
        Box((width - 2.0 * side_thickness, front_thickness, 0.096)),
        origin=Origin(xyz=(0.0, front_y - front_thickness * 0.5, 0.202)),
        material=body_white,
        name="mid_sill",
    )
    cabinet.visual(
        Box((0.050, front_thickness, 0.100)),
        origin=Origin(xyz=(-0.275, front_y - front_thickness * 0.5, 0.104)),
        material=body_white,
        name="left_drawer_post",
    )
    cabinet.visual(
        Box((0.050, front_thickness, 0.100)),
        origin=Origin(xyz=(0.275, front_y - front_thickness * 0.5, 0.104)),
        material=body_white,
        name="right_drawer_post",
    )
    cabinet.visual(
        Box((width, front_thickness, 0.054)),
        origin=Origin(xyz=(0.0, front_y - front_thickness * 0.5, 0.027)),
        material=warm_white,
        name="plinth",
    )
    cabinet.visual(
        _annulus_mesh(
            name="dryer_front_bezel",
            outer_radius=bezel_outer_radius,
            inner_radius=bezel_inner_radius,
            thickness=0.012,
        ),
        origin=Origin(xyz=(0.0, 0.230, door_center_z)),
        material=warm_white,
        name="front_bezel",
    )
    cabinet.visual(
        _annulus_mesh(
            name="dryer_gasket_ring_v2",
            outer_radius=0.160,
            inner_radius=0.145,
            thickness=0.012,
        ),
        origin=Origin(xyz=(0.0, 0.221, door_center_z)),
        material=rubber,
        name="door_gasket",
    )
    cabinet.visual(
        Box((0.014, 0.012, 0.300)),
        origin=Origin(xyz=(-0.198, 0.228, door_center_z)),
        material=warm_white,
        name="left_bezel_bridge",
    )
    cabinet.visual(
        Box((0.014, 0.012, 0.300)),
        origin=Origin(xyz=(0.198, 0.228, door_center_z)),
        material=warm_white,
        name="right_bezel_bridge",
    )
    cabinet.visual(
        Box((0.210, 0.006, 0.060)),
        origin=Origin(xyz=(0.085, front_y + 0.003, 0.782)),
        material=glass,
        name="display_panel",
    )
    cabinet.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(-0.125, front_y + 0.006, 0.782), rpy=CYLINDER_Y),
        material=steel,
        name="program_knob_trim",
    )

    cabinet.visual(
        Box((0.034, 0.280, 0.032)),
        origin=Origin(xyz=(-0.196, 0.105, 0.034)),
        material=filter_grey,
        name="left_drawer_shelf",
    )
    cabinet.visual(
        Box((0.034, 0.280, 0.032)),
        origin=Origin(xyz=(0.196, 0.105, 0.034)),
        material=filter_grey,
        name="right_drawer_shelf",
    )

    cabinet.visual(
        Cylinder(radius=0.025, length=0.100),
        origin=Origin(xyz=(-0.125, 0.020, 0.190), rpy=CYLINDER_X),
        material=graphite,
        name="left_drum_roller",
    )
    cabinet.visual(
        Cylinder(radius=0.025, length=0.100),
        origin=Origin(xyz=(0.125, 0.020, 0.190), rpy=CYLINDER_X),
        material=graphite,
        name="right_drum_roller",
    )
    cabinet.visual(
        Box((0.050, 0.050, 0.147)),
        origin=Origin(xyz=(-0.125, 0.020, 0.0915)),
        material=warm_white,
        name="left_roller_support",
    )
    cabinet.visual(
        Box((0.050, 0.050, 0.147)),
        origin=Origin(xyz=(0.125, 0.020, 0.0915)),
        material=warm_white,
        name="right_roller_support",
    )

    cabinet.visual(
        Cylinder(radius=0.022, length=0.046),
        origin=Origin(xyz=(0.0, -0.205, 0.460), rpy=CYLINDER_Y),
        material=steel,
        name="rear_bearing_collar",
    )
    cabinet.visual(
        Box((0.080, 0.036, 0.080)),
        origin=Origin(xyz=(0.0, -0.216, 0.460)),
        material=warm_white,
        name="rear_bearing_housing",
    )

    cabinet.visual(
        Cylinder(radius=0.006, length=0.220),
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z)),
        material=steel,
        name="hinge_pin",
    )
    cabinet.visual(
        Box((0.024, 0.014, 0.058)),
        origin=Origin(xyz=(door_hinge_x - 0.012, door_hinge_y - 0.010, door_center_z + 0.078)),
        material=warm_white,
        name="upper_hinge_tab",
    )
    cabinet.visual(
        Box((0.024, 0.014, 0.058)),
        origin=Origin(xyz=(door_hinge_x - 0.012, door_hinge_y - 0.010, door_center_z - 0.078)),
        material=warm_white,
        name="lower_hinge_tab",
    )

    for foot_name, foot_x, foot_y in (
        ("front_left_foot", -0.240, 0.180),
        ("front_right_foot", 0.240, 0.180),
        ("rear_left_foot", -0.240, -0.180),
        ("rear_right_foot", 0.240, -0.180),
    ):
        cabinet.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(foot_x, foot_y, 0.006)),
            material=charcoal,
            name=foot_name,
        )

    drum = model.part("drum")
    drum_shell = LatheGeometry.from_shell_profiles(
        [
            (0.235, -0.165),
            (0.242, -0.150),
            (0.245, -0.120),
            (0.245, 0.120),
            (0.242, 0.150),
            (0.235, 0.165),
        ],
        [
            (0.225, -0.165),
            (0.232, -0.145),
            (0.235, -0.120),
            (0.235, 0.120),
            (0.232, 0.145),
            (0.225, 0.165),
        ],
        segments=72,
    )
    drum.visual(
        mesh_from_geometry(drum_shell.rotate_x(math.pi / 2.0), "dryer_drum_shell"),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.225, length=0.004),
        origin=Origin(xyz=(0.0, -0.163, 0.0), rpy=CYLINDER_Y),
        material=steel,
        name="rear_web",
    )
    drum.visual(
        Cylinder(radius=0.030, length=0.036),
        origin=Origin(xyz=(0.0, -0.179, 0.0), rpy=CYLINDER_Y),
        material=graphite,
        name="axle_spindle",
    )
    for paddle_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        drum.visual(
            Box((0.028, 0.260, 0.050)),
            origin=Origin(
                xyz=(0.212 * math.sin(angle), 0.0, 0.212 * math.cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=graphite,
            name=f"paddle_{paddle_index}",
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.245, length=0.340),
        mass=11.5,
        origin=Origin(rpy=CYLINDER_Y),
    )

    door = model.part("door")
    door.visual(
        _annulus_mesh(
            name="dryer_door_outer_ring_v2",
            outer_radius=door_outer_radius,
            inner_radius=0.146,
            thickness=0.044,
        ),
        origin=Origin(xyz=(0.215, 0.030, 0.0)),
        material=warm_white,
        name="outer_ring",
    )
    door.visual(
        _annulus_mesh(
            name="dryer_door_inner_ring_v2",
            outer_radius=0.164,
            inner_radius=0.132,
            thickness=0.018,
        ),
        origin=Origin(xyz=(0.215, 0.012, 0.0)),
        material=graphite,
        name="inner_ring",
    )
    door.visual(
        Cylinder(radius=0.134, length=0.012),
        origin=Origin(xyz=(0.215, 0.012, 0.0), rpy=CYLINDER_Y),
        material=glass,
        name="glass_window",
    )
    door.visual(
        Box((0.050, 0.016, 0.200)),
        origin=Origin(xyz=(0.030, 0.010, 0.0)),
        material=warm_white,
        name="hinge_arm",
    )
    door.visual(
        _tube_shell_mesh(
            name="dryer_upper_hinge_sleeve_v1",
            outer_radius=0.009,
            inner_radius=0.006,
            length=0.056,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=steel,
        name="upper_hinge_knuckle",
    )
    door.visual(
        _tube_shell_mesh(
            name="dryer_lower_hinge_sleeve_v1",
            outer_radius=0.009,
            inner_radius=0.006,
            length=0.056,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=steel,
        name="lower_hinge_knuckle",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.156),
        origin=Origin(xyz=(0.366, 0.056, 0.0)),
        material=chrome,
        name="handle_bar",
    )
    door.visual(
        Cylinder(radius=0.0045, length=0.028),
        origin=Origin(xyz=(0.366, 0.042, 0.050), rpy=CYLINDER_Y),
        material=chrome,
        name="handle_upper_standoff",
    )
    door.visual(
        Cylinder(radius=0.0045, length=0.028),
        origin=Origin(xyz=(0.366, 0.042, -0.050), rpy=CYLINDER_Y),
        material=chrome,
        name="handle_lower_standoff",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.420, 0.070, 0.260)),
        mass=3.1,
        origin=Origin(xyz=(0.215, 0.030, 0.0)),
    )

    drawer = model.part("lint_filter_drawer")
    drawer.visual(
        Box((0.496, 0.026, 0.096)),
        material=body_white,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.456, 0.168, 0.006)),
        origin=Origin(xyz=(0.0, -0.091, -0.047)),
        material=filter_grey,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.006, 0.168, 0.046)),
        origin=Origin(xyz=(-0.225, -0.091, -0.023)),
        material=filter_grey,
        name="left_wall",
    )
    drawer.visual(
        Box((0.006, 0.168, 0.046)),
        origin=Origin(xyz=(0.225, -0.091, -0.023)),
        material=filter_grey,
        name="right_wall",
    )
    drawer.visual(
        Box((0.456, 0.006, 0.046)),
        origin=Origin(xyz=(0.0, -0.175, -0.023)),
        material=filter_grey,
        name="rear_wall",
    )
    drawer.visual(
        Box((0.420, 0.130, 0.004)),
        origin=Origin(xyz=(0.0, -0.091, -0.045)),
        material=charcoal,
        name="filter_screen",
    )
    drawer.visual(
        Cylinder(radius=0.007, length=0.180),
        origin=Origin(xyz=(0.0, 0.018, 0.012), rpy=CYLINDER_X),
        material=chrome,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.032, 0.160, 0.008)),
        origin=Origin(xyz=(-0.196, -0.100, -0.050)),
        material=graphite,
        name="left_runner",
    )
    drawer.visual(
        Box((0.032, 0.160, 0.008)),
        origin=Origin(xyz=(0.196, -0.100, -0.050)),
        material=graphite,
        name="right_runner",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.500, 0.200, 0.100)),
        mass=1.0,
        origin=Origin(xyz=(0.0, -0.070, -0.010)),
    )

    model.articulation(
        "drum_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, 0.015, 0.460)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, drawer_origin_y, drawer_origin_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.20,
            lower=0.0,
            upper=drawer_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drawer = object_model.get_part("lint_filter_drawer")
    hinge_pin = cabinet.get_visual("hinge_pin")
    upper_hinge_tab = cabinet.get_visual("upper_hinge_tab")
    lower_hinge_tab = cabinet.get_visual("lower_hinge_tab")
    upper_hinge_knuckle = door.get_visual("upper_hinge_knuckle")
    lower_hinge_knuckle = door.get_visual("lower_hinge_knuckle")

    drum_spin = object_model.get_articulation("drum_spin")
    door_hinge = object_model.get_articulation("door_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a=hinge_pin,
        elem_b=upper_hinge_knuckle,
        reason="Door hinge pin runs inside the upper hinge knuckle.",
    )
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a=hinge_pin,
        elem_b=lower_hinge_knuckle,
        reason="Door hinge pin runs inside the lower hinge knuckle.",
    )
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a=upper_hinge_tab,
        elem_b=upper_hinge_knuckle,
        reason="Upper hinge tab captures the upper hinge knuckle around the pin axis.",
    )
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a=lower_hinge_tab,
        elem_b=lower_hinge_knuckle,
        reason="Lower hinge tab captures the lower hinge knuckle around the pin axis.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32, name="moving_clearance_sweep")

    ctx.expect_contact(cabinet, drum, name="drum_supported_at_rest")
    ctx.expect_contact(cabinet, door, name="door_hinge_contact_at_rest")
    ctx.expect_contact(cabinet, drawer, name="drawer_guided_at_rest")
    ctx.expect_within(drum, cabinet, axes="xz", margin=0.0, name="drum_within_body")
    ctx.expect_overlap(door, cabinet, axes="xz", min_overlap=0.35, name="door_covers_porthole")
    ctx.expect_overlap(drawer, cabinet, axes="xz", min_overlap=0.09, name="drawer_aligned_with_base")

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    drawer_rest_pos = ctx.part_world_position(drawer)
    handle_rest_aabb = ctx.part_element_world_aabb(door, elem="handle_bar")
    assert cabinet_aabb is not None
    assert drawer_rest_pos is not None
    assert handle_rest_aabb is not None

    with ctx.pose({drum_spin: math.pi / 3.0}):
        ctx.expect_contact(cabinet, drum, name="drum_supported_when_turned")
        ctx.fail_if_parts_overlap_in_current_pose(name="drum_turned_no_overlap")

    door_limits = door_hinge.motion_limits
    assert door_limits is not None and door_limits.lower is not None and door_limits.upper is not None
    with ctx.pose({door_hinge: door_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="door_closed_no_floating")
    with ctx.pose({door_hinge: door_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.expect_contact(cabinet, door, name="door_hinge_contact_open")
        handle_open_aabb = ctx.part_element_world_aabb(door, elem="handle_bar")
        assert handle_open_aabb is not None
        assert handle_open_aabb[1][1] > handle_rest_aabb[1][1] + 0.20
        assert handle_open_aabb[0][0] < handle_rest_aabb[0][0] - 0.20

    drawer_limits = drawer_slide.motion_limits
    assert drawer_limits is not None and drawer_limits.lower is not None and drawer_limits.upper is not None
    with ctx.pose({drawer_slide: drawer_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="drawer_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="drawer_closed_no_floating")
    with ctx.pose({drawer_slide: drawer_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="drawer_open_no_overlap")
        ctx.fail_if_isolated_parts(name="drawer_open_no_floating")
        ctx.expect_contact(cabinet, drawer, name="drawer_guided_open")
        drawer_open_pos = ctx.part_world_position(drawer)
        drawer_open_aabb = ctx.part_world_aabb(drawer)
        assert drawer_open_pos is not None
        assert drawer_open_aabb is not None
        assert drawer_open_pos[1] > drawer_rest_pos[1] + 0.10
        assert drawer_open_aabb[1][1] > cabinet_aabb[1][1] + 0.08

    with ctx.pose(
        {
            door_hinge: door_limits.upper,
            drawer_slide: drawer_limits.upper,
            drum_spin: math.pi / 2.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="service_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="service_pose_no_floating")
        ctx.expect_contact(cabinet, drum, name="service_pose_drum_supported")
        ctx.expect_contact(cabinet, door, name="service_pose_door_supported")
        ctx.expect_contact(cabinet, drawer, name="service_pose_drawer_supported")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
