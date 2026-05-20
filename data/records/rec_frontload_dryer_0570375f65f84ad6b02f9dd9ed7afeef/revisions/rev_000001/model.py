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
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _translate_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _orient_front_mesh(geometry):
    geometry.rotate_y(math.pi * 0.5)
    geometry.rotate_x(math.pi * 0.5)
    return geometry


def _front_annulus_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(_orient_front_mesh(geom), name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heat_pump_tumble_dryer")

    width = 0.598
    depth = 0.642
    height = 0.850
    shell_t = 0.018
    front_t = 0.018
    plinth_open_w = 0.448
    plinth_open_h = 0.084
    plinth_center_z = 0.055
    door_center_z = 0.455
    door_opening_r = 0.228
    door_outer_r = 0.248
    drum_radius = 0.262
    drum_length = 0.440
    drum_wall = 0.010
    drum_center_x = 0.030

    cabinet_white = model.material("cabinet_white", rgba=(0.96, 0.97, 0.98, 1.0))
    warm_white = model.material("warm_white", rgba=(0.93, 0.94, 0.95, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.18, 0.20, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.23, 0.25, 0.28, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.69, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.58, 0.72, 0.82, 0.28))
    filter_dark = model.material("filter_dark", rgba=(0.24, 0.28, 0.31, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.12, 0.13, 0.15, 1.0))

    front_panel_outer = _rect_profile(width - 2.0 * shell_t, height - 2.0 * shell_t)
    drawer_hole = _translate_profile(
        rounded_rect_profile(plinth_open_w, plinth_open_h, 0.010),
        0.0,
        plinth_center_z - height * 0.5,
    )
    front_panel_geom = ExtrudeWithHolesGeometry(
        front_panel_outer,
        [
            _translate_profile(
                _circle_profile(door_opening_r),
                0.0,
                door_center_z - height * 0.5,
            ),
            drawer_hole,
        ],
        front_t,
        cap=True,
        center=True,
        closed=True,
    )
    front_panel_mesh = mesh_from_geometry(
        _orient_front_mesh(front_panel_geom),
        "dryer_front_panel",
    )
    aperture_ring_mesh = _front_annulus_mesh(
        "dryer_aperture_ring",
        outer_radius=door_opening_r + 0.017,
        inner_radius=door_opening_r - 0.006,
        thickness=0.020,
    )
    door_outer_frame_mesh = _front_annulus_mesh(
        "dryer_door_outer_frame",
        outer_radius=door_outer_r,
        inner_radius=0.182,
        thickness=0.036,
    )
    door_inner_retainer_mesh = _front_annulus_mesh(
        "dryer_door_inner_retainer",
        outer_radius=0.188,
        inner_radius=0.166,
        thickness=0.010,
    )
    chrome_bezel_mesh = _front_annulus_mesh(
        "dryer_door_chrome_bezel",
        outer_radius=0.224,
        inner_radius=0.184,
        thickness=0.010,
    )
    drum_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (drum_radius, -drum_length * 0.5),
                (drum_radius, drum_length * 0.5),
            ],
            [
                (drum_radius - drum_wall, -drum_length * 0.5),
                (drum_radius - drum_wall, drum_length * 0.5),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "dryer_drum_shell",
    )

    body = model.part("body")
    body.visual(
        Box((depth, shell_t, height)),
        origin=Origin(xyz=(0.0, -(width * 0.5 - shell_t * 0.5), height * 0.5)),
        material=cabinet_white,
        name="left_side",
    )
    body.visual(
        Box((depth, shell_t, height)),
        origin=Origin(xyz=(0.0, width * 0.5 - shell_t * 0.5, height * 0.5)),
        material=cabinet_white,
        name="right_side",
    )
    body.visual(
        Box((depth, width - 2.0 * shell_t, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_t * 0.5)),
        material=cabinet_white,
        name="bottom_panel",
    )
    body.visual(
        Box((depth, width - 2.0 * shell_t, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, height - shell_t * 0.5)),
        material=cabinet_white,
        name="top_panel",
    )
    body.visual(
        Box((shell_t, width - 2.0 * shell_t, height - shell_t)),
        origin=Origin(
            xyz=(-depth * 0.5 + shell_t * 0.5, 0.0, (height - shell_t) * 0.5)
        ),
        material=cabinet_white,
        name="back_panel",
    )
    body.visual(
        front_panel_mesh,
        origin=Origin(xyz=(depth * 0.5 - front_t * 0.5, 0.0, height * 0.5)),
        material=cabinet_white,
        name="front_panel",
    )
    body.visual(
        aperture_ring_mesh,
        origin=Origin(xyz=(depth * 0.5 - 0.022, 0.0, door_center_z)),
        material=warm_white,
        name="aperture_ring",
    )
    body.visual(
        Cylinder(radius=door_opening_r - 0.004, length=0.046),
        origin=Origin(
            xyz=(depth * 0.5 - 0.050, 0.0, door_center_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=gasket_dark,
        name="drum_shroud",
    )
    body.visual(
        Box((0.260, 0.150, 0.014)),
        origin=Origin(xyz=(depth * 0.5 - 0.008, 0.110, height - 0.064)),
        material=dark_trim,
        name="control_glass",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(
            xyz=(depth * 0.5 - 0.003, -0.185, height - 0.064),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=knob_dark,
        name="program_knob",
    )
    for hinge_name, z_value in (
        ("upper_hinge", door_center_z + 0.130),
        ("lower_hinge", door_center_z - 0.130),
    ):
        body.visual(
            Box((0.030, 0.020, 0.048)),
            origin=Origin(
                xyz=(depth * 0.5 - 0.020, -door_outer_r - 0.003, z_value)
            ),
            material=cabinet_white,
            name=f"{hinge_name}_bracket",
        )
        body.visual(
            Cylinder(radius=0.011, length=0.048),
            origin=Origin(
                xyz=(depth * 0.5 - 0.011, -door_outer_r, z_value)
            ),
            material=steel,
            name=f"{hinge_name}_barrel",
        )
    body.visual(
        Cylinder(radius=0.034, length=0.030),
        origin=Origin(
            xyz=(-depth * 0.5 + 0.023, 0.0, door_center_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="rear_bearing_hub",
    )
    body.inertial = Inertial.from_geometry(
        Box((depth, width, height)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    drum = model.part("drum")
    drum.visual(
        drum_shell_mesh,
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=drum_radius - drum_wall, length=0.012),
        origin=Origin(
            xyz=(-drum_length * 0.5 + 0.006, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="rear_bulkhead",
    )
    drum.visual(
        Cylinder(radius=0.020, length=0.102),
        origin=Origin(
            xyz=(-0.265, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="rear_axle_stub",
    )
    lifter_radius = drum_radius - drum_wall - 0.010
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((drum_length - 0.085, 0.036, 0.020)),
            origin=Origin(
                xyz=(
                    0.0,
                    lifter_radius * math.sin(angle),
                    lifter_radius * math.cos(angle),
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            material=steel,
            name=f"lifter_{index}",
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_radius, length=drum_length),
        mass=8.5,
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
    )

    door = model.part("door")
    door.visual(
        door_outer_frame_mesh,
        origin=Origin(xyz=(0.030, door_outer_r, 0.0)),
        material=cabinet_white,
        name="outer_frame",
    )
    door.visual(
        door_inner_retainer_mesh,
        origin=Origin(xyz=(0.020, door_outer_r, 0.0)),
        material=warm_white,
        name="inner_retainer",
    )
    door.visual(
        chrome_bezel_mesh,
        origin=Origin(xyz=(0.036, door_outer_r, 0.0)),
        material=chrome,
        name="chrome_bezel",
    )
    door.visual(
        Cylinder(radius=0.176, length=0.006),
        origin=Origin(
            xyz=(0.031, door_outer_r, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=tinted_glass,
        name="glass_window",
    )
    door.visual(
        Box((0.022, 0.070, 0.024)),
        origin=Origin(xyz=(0.040, door_outer_r + 0.170, 0.015)),
        material=warm_white,
        name="handle_grip",
    )
    for hinge_name, z_value in (
        ("upper_hinge", 0.130),
        ("lower_hinge", -0.130),
    ):
        door.visual(
            Box((0.018, 0.046, 0.028)),
            origin=Origin(xyz=(0.015, 0.028, z_value)),
            material=warm_white,
            name=f"{hinge_name}_strap",
        )
        door.visual(
            Cylinder(radius=0.011, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, z_value)),
            material=chrome,
            name=f"{hinge_name}_sleeve",
        )
    door.inertial = Inertial.from_geometry(
        Box((0.090, 0.500, 0.500)),
        mass=3.4,
        origin=Origin(xyz=(0.0, door_outer_r, 0.0)),
    )

    filter_drawer = model.part("filter_drawer")
    filter_drawer.visual(
        Box((0.018, 0.436, 0.076)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=cabinet_white,
        name="drawer_front",
    )
    filter_drawer.visual(
        Box((0.010, 0.280, 0.030)),
        origin=Origin(xyz=(0.014, 0.0, -0.004)),
        material=filter_dark,
        name="drawer_grille",
    )
    filter_drawer.visual(
        Box((0.010, 0.112, 0.014)),
        origin=Origin(xyz=(0.015, 0.0, 0.022)),
        material=dark_trim,
        name="drawer_handle_recess",
    )
    filter_drawer.visual(
        Box((0.258, 0.006, 0.060)),
        origin=Origin(xyz=(-0.129, -(0.410 * 0.5 - 0.003), -0.003)),
        material=warm_white,
        name="left_tray_wall",
    )
    filter_drawer.visual(
        Box((0.258, 0.006, 0.060)),
        origin=Origin(xyz=(-0.129, 0.410 * 0.5 - 0.003, -0.003)),
        material=warm_white,
        name="right_tray_wall",
    )
    filter_drawer.visual(
        Box((0.258, 0.398, 0.006)),
        origin=Origin(xyz=(-0.129, 0.0, -0.031)),
        material=warm_white,
        name="tray_floor",
    )
    filter_drawer.visual(
        Box((0.258, 0.398, 0.006)),
        origin=Origin(xyz=(-0.129, 0.0, 0.024)),
        material=warm_white,
        name="tray_top_lip",
    )
    filter_drawer.visual(
        Box((0.008, 0.398, 0.060)),
        origin=Origin(xyz=(-0.255, 0.0, -0.003)),
        material=filter_dark,
        name="filter_panel",
    )
    filter_drawer.inertial = Inertial.from_geometry(
        Box((0.276, 0.436, 0.080)),
        mass=0.9,
        origin=Origin(xyz=(-0.120, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(drum_center_x, 0.0, door_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=5.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(
            xyz=(depth * 0.5 + 0.011, -door_outer_r, door_center_z)
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_filter_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_drawer,
        origin=Origin(xyz=(depth * 0.5, 0.0, plinth_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.18,
            lower=0.0,
            upper=0.180,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    filter_drawer = object_model.get_part("filter_drawer")
    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    drawer_joint = object_model.get_articulation("body_to_filter_drawer")

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

    ctx.expect_within(
        drum,
        body,
        axes="yz",
        margin=0.018,
        name="drum stays within cabinet width and height envelope",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="upper_hinge_sleeve",
        elem_b="upper_hinge_barrel",
        contact_tol=0.004,
        name="upper barrel hinge remains mounted",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="lower_hinge_sleeve",
        elem_b="lower_hinge_barrel",
        contact_tol=0.004,
        name="lower barrel hinge remains mounted",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            min_gap=0.0,
            max_gap=0.045,
            positive_elem="outer_frame",
            negative_elem="front_panel",
            name="closed door sits just proud of cabinet front",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            min_overlap=0.25,
            name="closed door covers the porthole opening",
        )
        ctx.expect_gap(
            filter_drawer,
            body,
            axis="x",
            min_gap=0.0,
            max_gap=0.025,
            positive_elem="drawer_front",
            negative_elem="front_panel",
            name="closed filter drawer sits flush with plinth face",
        )

    rest_drawer_pos = ctx.part_world_position(filter_drawer)
    with ctx.pose({drawer_joint: drawer_joint.motion_limits.upper}):
        extended_drawer_pos = ctx.part_world_position(filter_drawer)
        ctx.expect_overlap(
            filter_drawer,
            body,
            axes="x",
            min_overlap=0.050,
            name="extended filter drawer retains insertion in cabinet base",
        )
    ctx.check(
        "filter drawer extends outward along the plinth",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.12,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    rest_door_aabb = ctx.part_element_world_aabb(door, elem="outer_frame")
    with ctx.pose({door_joint: 1.10}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="outer_frame")
    rest_door_center = _aabb_center(rest_door_aabb)
    open_door_center = _aabb_center(open_door_aabb)
    ctx.check(
        "door swings outward on its left hinges",
        rest_door_center is not None
        and open_door_center is not None
        and open_door_center[0] > rest_door_center[0] + 0.10
        and open_door_center[1] < rest_door_center[1] - 0.05,
        details=f"closed={rest_door_center}, open={open_door_center}",
    )

    rest_lifter_aabb = ctx.part_element_world_aabb(drum, elem="lifter_0")
    with ctx.pose({drum_joint: 1.0}):
        spun_lifter_aabb = ctx.part_element_world_aabb(drum, elem="lifter_0")
    rest_lifter_center = _aabb_center(rest_lifter_aabb)
    spun_lifter_center = _aabb_center(spun_lifter_aabb)
    ctx.check(
        "drum articulation rotates the internal lifters about the central axle",
        rest_lifter_center is not None
        and spun_lifter_center is not None
        and (
            abs(spun_lifter_center[1] - rest_lifter_center[1]) > 0.05
            or abs(spun_lifter_center[2] - rest_lifter_center[2]) > 0.05
        ),
        details=f"rest={rest_lifter_center}, spun={spun_lifter_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
