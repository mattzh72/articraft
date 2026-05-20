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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washer")

    body_white = model.material("body_white", rgba=(0.95, 0.96, 0.97, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.15, 0.17, 0.19, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.62, 0.76, 0.86, 0.30))
    gasket_black = model.material("gasket_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))

    body_width = 0.60
    body_depth = 0.63
    body_height = 0.85
    panel_t = 0.018

    drum_radius = 0.225
    drum_length = 0.30
    opening_radius = 0.205
    opening_center_z = 0.42

    door_center_x = 0.0
    door_axis_x = -0.248
    door_axis_y = body_depth / 2.0 + 0.006
    door_axis_z = opening_center_z

    drawer_width = 0.158
    drawer_height = 0.074
    drawer_center_x = -0.168
    drawer_center_z = 0.725

    def circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
        return [
            (
                radius * math.cos((2.0 * math.pi * index) / segments),
                radius * math.sin((2.0 * math.pi * index) / segments),
            )
            for index in range(segments)
        ]

    def rect_profile(width: float, height: float) -> list[tuple[float, float]]:
        half_w = width * 0.5
        half_h = height * 0.5
        return [
            (-half_w, -half_h),
            (half_w, -half_h),
            (half_w, half_h),
            (-half_w, half_h),
        ]

    def offset_profile(
        profile: list[tuple[float, float]], dx: float, dy: float
    ) -> list[tuple[float, float]]:
        return [(x + dx, y + dy) for x, y in profile]

    def make_front_frame_mesh():
        front_width = body_width - 2.0 * panel_t + 0.004
        front_height = body_height - 2.0 * panel_t + 0.004
        outer = rect_profile(front_width, front_height)
        door_hole = offset_profile(
            circle_profile(opening_radius, segments=72),
            door_center_x,
            opening_center_z - body_height * 0.5,
        )
        drawer_hole = offset_profile(
            rounded_rect_profile(drawer_width, drawer_height, 0.010, corner_segments=8),
            drawer_center_x,
            drawer_center_z - body_height * 0.5,
        )
        geom = ExtrudeWithHolesGeometry(
            outer,
            [door_hole, drawer_hole],
            panel_t,
            cap=True,
            center=True,
            closed=True,
        )
        geom.rotate_x(math.pi / 2.0)
        return geom

    def make_boot_ring_mesh():
        geom = LatheGeometry.from_shell_profiles(
            [
                (0.176, -0.014),
                (0.196, -0.014),
                (0.218, 0.002),
                (0.228, 0.018),
            ],
            [
                (0.150, -0.010),
                (0.175, -0.010),
                (0.192, 0.002),
                (0.198, 0.014),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        )
        geom.rotate_x(math.pi / 2.0)
        return geom

    def make_door_ring_mesh():
        geom = LatheGeometry.from_shell_profiles(
            [
                (0.172, -0.020),
                (0.214, -0.020),
                (0.248, -0.006),
                (0.258, 0.016),
                (0.236, 0.030),
            ],
            [
                (0.158, -0.011),
                (0.184, -0.011),
                (0.204, 0.003),
                (0.198, 0.017),
                (0.176, 0.021),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        )
        geom.rotate_x(math.pi / 2.0)
        return geom

    def make_inner_trim_mesh():
        geom = LatheGeometry.from_shell_profiles(
            [
                (0.155, -0.008),
                (0.180, -0.008),
                (0.194, 0.002),
                (0.188, 0.013),
            ],
            [
                (0.135, -0.005),
                (0.156, -0.005),
                (0.171, 0.002),
                (0.166, 0.010),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        )
        geom.rotate_x(math.pi / 2.0)
        return geom

    body = model.part("body")
    front_frame_mesh = mesh_from_geometry(make_front_frame_mesh(), "washer_front_frame")
    boot_ring_mesh = mesh_from_geometry(make_boot_ring_mesh(), "washer_boot_ring")

    body.visual(
        Box((panel_t, body_depth, body_height)),
        origin=Origin(xyz=(-body_width / 2.0 + panel_t / 2.0, 0.0, body_height / 2.0)),
        material=body_white,
        name="left_panel",
    )
    body.visual(
        Box((panel_t, body_depth, body_height)),
        origin=Origin(xyz=(body_width / 2.0 - panel_t / 2.0, 0.0, body_height / 2.0)),
        material=body_white,
        name="right_panel",
    )
    body.visual(
        Box((body_width - 2.0 * panel_t + 0.004, body_depth, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, panel_t / 2.0)),
        material=body_white,
        name="bottom_panel",
    )
    body.visual(
        Box((body_width - 2.0 * panel_t + 0.004, body_depth, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, body_height - panel_t / 2.0)),
        material=body_white,
        name="top_panel",
    )
    body.visual(
        Box((body_width - 2.0 * panel_t + 0.004, panel_t, body_height - 2.0 * panel_t + 0.004)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0 + panel_t / 2.0, body_height / 2.0)),
        material=body_white,
        name="back_panel",
    )
    body.visual(
        front_frame_mesh,
        origin=Origin(xyz=(0.0, body_depth / 2.0 - panel_t / 2.0, body_height / 2.0)),
        material=body_white,
        name="front_frame",
    )
    body.visual(
        boot_ring_mesh,
        origin=Origin(xyz=(0.0, body_depth / 2.0 - 0.022, opening_center_z)),
        material=gasket_black,
        name="door_boot",
    )
    body.visual(
        Box((drawer_width + 0.030, 0.185, 0.005)),
        origin=Origin(xyz=(drawer_center_x, body_depth / 2.0 - 0.092, drawer_center_z - 0.0345)),
        material=trim_dark,
        name="drawer_housing_floor",
    )
    body.visual(
        Box((drawer_width + 0.030, 0.185, 0.005)),
        origin=Origin(xyz=(drawer_center_x, body_depth / 2.0 - 0.092, drawer_center_z + 0.0345)),
        material=trim_dark,
        name="drawer_housing_ceiling",
    )
    body.visual(
        Box((0.005, 0.185, drawer_height + 0.006)),
        origin=Origin(xyz=(drawer_center_x - drawer_width / 2.0 - 0.0125, body_depth / 2.0 - 0.092, drawer_center_z)),
        material=trim_dark,
        name="drawer_housing_left_wall",
    )
    body.visual(
        Box((0.005, 0.185, drawer_height + 0.006)),
        origin=Origin(xyz=(drawer_center_x + drawer_width / 2.0 + 0.0125, body_depth / 2.0 - 0.092, drawer_center_z)),
        material=trim_dark,
        name="drawer_housing_right_wall",
    )
    body.visual(
        Box((0.030, 0.034, 0.070)),
        origin=Origin(xyz=(door_axis_x - 0.029, body_depth / 2.0 - 0.006, opening_center_z + 0.150)),
        material=body_white,
        name="upper_hinge_mount",
    )
    body.visual(
        Box((0.030, 0.034, 0.070)),
        origin=Origin(xyz=(door_axis_x - 0.029, body_depth / 2.0 - 0.006, opening_center_z - 0.150)),
        material=body_white,
        name="lower_hinge_mount",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.062),
        origin=Origin(xyz=(door_axis_x, body_depth / 2.0 + 0.002, opening_center_z + 0.150)),
        material=trim_dark,
        name="upper_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.062),
        origin=Origin(xyz=(door_axis_x, body_depth / 2.0 + 0.002, opening_center_z - 0.150)),
        material=trim_dark,
        name="lower_hinge_barrel",
    )
    body.visual(
        Box((0.240, 0.022, 0.080)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0 + panel_t + 0.011, opening_center_z)),
        material=trim_dark,
        name="rear_bearing_bridge",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.036),
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0 + panel_t + 0.018, opening_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_dark,
        name="rear_bearing_housing",
    )
    for index, x in enumerate((-0.215, 0.215)):
        body.visual(
            Box((0.060, 0.055, 0.024)),
            origin=Origin(xyz=(x, -body_depth / 2.0 + 0.060, 0.012)),
            material=trim_dark,
            name=f"rear_foot_{index}",
        )
    for index, x in enumerate((-0.215, 0.215), start=2):
        body.visual(
            Box((0.060, 0.055, 0.024)),
            origin=Origin(xyz=(x, body_depth / 2.0 - 0.060, 0.012)),
            material=trim_dark,
            name=f"front_foot_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=drum_radius, length=0.294),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=drum_radius + 0.010, length=0.030),
        origin=Origin(xyz=(0.0, 0.144, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drum_front_rim",
    )
    drum.visual(
        Cylinder(radius=0.090, length=0.032),
        origin=Origin(xyz=(0.0, -0.144, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drum_rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.014, length=0.110),
        origin=Origin(xyz=(0.0, -0.206, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drum_axle",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.030, 0.220, 0.018)),
            origin=Origin(
                xyz=(0.185 * math.cos(angle), 0.0, 0.185 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=steel,
            name=f"drum_paddle_{index}",
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_radius + 0.010, length=drum_length + 0.070),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door_ring_mesh = mesh_from_geometry(make_door_ring_mesh(), "washer_door_ring")
    inner_trim_mesh = mesh_from_geometry(make_inner_trim_mesh(), "washer_door_inner_trim")
    door.visual(
        door_ring_mesh,
        origin=Origin(xyz=(0.248, 0.024, 0.0)),
        material=body_white,
        name="door_ring",
    )
    door.visual(
        inner_trim_mesh,
        origin=Origin(xyz=(0.248, 0.016, 0.0)),
        material=trim_dark,
        name="door_inner_trim",
    )
    door.visual(
        Cylinder(radius=0.162, length=0.010),
        origin=Origin(
            xyz=(0.248, 0.026, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Box((0.206, 0.018, 0.020)),
        origin=Origin(xyz=(0.116, 0.006, 0.150)),
        material=body_white,
        name="upper_hinge_arm",
    )
    door.visual(
        Box((0.206, 0.018, 0.020)),
        origin=Origin(xyz=(0.116, 0.006, -0.150)),
        material=body_white,
        name="lower_hinge_arm",
    )
    door.visual(
        Box((0.034, 0.036, 0.112)),
        origin=Origin(xyz=(0.456, 0.046, 0.0)),
        material=body_white,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.520, 0.090, 0.520)),
        mass=4.2,
        origin=Origin(xyz=(0.260, 0.020, 0.0)),
    )

    drawer = model.part("soap_drawer")
    drawer.visual(
        Box((drawer_width + 0.010, 0.040, drawer_height + 0.006)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=body_white,
        name="drawer_front",
    )
    drawer.visual(
        Box((drawer_width - 0.012, 0.165, 0.005)),
        origin=Origin(xyz=(0.0, -0.0825, -0.027)),
        material=body_white,
        name="drawer_tray_floor",
    )
    drawer.visual(
        Box((0.005, 0.165, drawer_height - 0.018)),
        origin=Origin(xyz=(-drawer_width / 2.0 + 0.006, -0.0825, -0.001)),
        material=body_white,
        name="drawer_tray_left",
    )
    drawer.visual(
        Box((0.005, 0.165, drawer_height - 0.018)),
        origin=Origin(xyz=(drawer_width / 2.0 - 0.006, -0.0825, -0.001)),
        material=body_white,
        name="drawer_tray_right",
    )
    drawer.visual(
        Box((drawer_width - 0.020, 0.005, drawer_height - 0.018)),
        origin=Origin(xyz=(0.0, -0.170, -0.001)),
        material=body_white,
        name="drawer_tray_back",
    )
    drawer.visual(
        Box((0.005, 0.135, drawer_height - 0.026)),
        origin=Origin(xyz=(0.000, -0.100, -0.005)),
        material=body_white,
        name="drawer_divider",
    )
    drawer.visual(
        Box((0.074, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=trim_dark,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((drawer_width + 0.010, 0.190, drawer_height + 0.006)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -0.075, 0.0)),
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, opening_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=18.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_axis_x, door_axis_y, door_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "body_to_soap_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(drawer_center_x, body_depth / 2.0, drawer_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.18,
            lower=0.0,
            upper=0.120,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drawer = object_model.get_part("soap_drawer")
    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    drawer_joint = object_model.get_articulation("body_to_soap_drawer")

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
        "drum articulation is continuous",
        drum_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={drum_joint.articulation_type}",
    )
    ctx.check(
        "drum spins around the front-back axis",
        tuple(drum_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={drum_joint.axis}",
    )
    ctx.check(
        "door hinge opens about a vertical axis",
        tuple(door_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "soap drawer slides outward from the front",
        tuple(drawer_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={drawer_joint.axis}",
    )

    with ctx.pose({door_joint: 0.0, drawer_joint: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_ring",
            negative_elem="front_frame",
            max_gap=0.025,
            max_penetration=0.0,
            name="door sits just proud of the cabinet front",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="door_ring",
            elem_b="front_frame",
            min_overlap=0.38,
            name="door ring spans the porthole opening",
        )
        ctx.expect_overlap(
            drum,
            door,
            axes="xz",
            elem_a="drum_shell",
            elem_b="door_glass",
            min_overlap=0.28,
            name="drum is centered behind the viewing window",
        )
        ctx.expect_gap(
            drawer,
            body,
            axis="y",
            positive_elem="drawer_front",
            negative_elem="front_frame",
            max_gap=0.004,
            max_penetration=0.0,
            name="soap drawer front closes flush with the fascia",
        )

    with ctx.pose({door_joint: 0.0}):
        closed_handle = ctx.part_element_world_aabb(door, elem="door_handle")
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        open_handle = ctx.part_element_world_aabb(door, elem="door_handle")
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_handle",
            negative_elem="front_frame",
            min_gap=0.12,
            name="opened door swings clear of the front panel",
        )
    ctx.check(
        "door handle swings outward when opened",
        closed_handle is not None
        and open_handle is not None
        and open_handle[0][1] > closed_handle[0][1] + 0.12,
        details=f"closed={closed_handle}, open={open_handle}",
    )

    with ctx.pose({drawer_joint: 0.0}):
        drawer_closed_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: drawer_joint.motion_limits.upper}):
        drawer_open_pos = ctx.part_world_position(drawer)
        ctx.expect_gap(
            drawer,
            body,
            axis="y",
            positive_elem="drawer_front",
            negative_elem="front_frame",
            min_gap=0.10,
            name="opened soap drawer extends forward",
        )
    ctx.check(
        "soap drawer translates forward at full extension",
        drawer_closed_pos is not None
        and drawer_open_pos is not None
        and drawer_open_pos[1] > drawer_closed_pos[1] + 0.10,
        details=f"closed={drawer_closed_pos}, open={drawer_open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
