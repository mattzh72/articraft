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


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 64,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_h = height / 2.0
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_wifi_dryer")

    white_enamel = model.material("white_enamel", rgba=(0.95, 0.96, 0.97, 1.0))
    warm_white = model.material("warm_white", rgba=(0.92, 0.93, 0.94, 1.0))
    console_black = model.material("console_black", rgba=(0.10, 0.11, 0.12, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.18, 0.22, 0.26, 0.70))
    stainless = model.material("stainless", rgba=(0.72, 0.75, 0.79, 1.0))
    drum_metal = model.material("drum_metal", rgba=(0.73, 0.76, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.27, 0.29, 0.31, 1.0))
    led_blue = model.material("led_blue", rgba=(0.25, 0.60, 0.95, 1.0))

    cabinet_width = 0.68
    cabinet_depth = 0.74
    cabinet_height = 0.93
    side_thickness = 0.018
    rear_panel_thickness = 0.012
    front_panel_thickness = 0.024
    door_center_z = 0.43
    door_hinge_x = 0.275
    door_hinge_y = 0.377
    top_cover_hinge_y = -0.348
    top_cover_hinge_z = 0.914

    front_fascia_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(cabinet_width - 2.0 * side_thickness, 0.80),
            [_circle_profile(0.236, segments=80)],
            front_panel_thickness,
            center=True,
        ),
        "dryer_front_fascia",
    )
    door_outer_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.275, segments=96),
            [_circle_profile(0.198, segments=96)],
            0.055,
            center=True,
        ),
        "dryer_door_outer_ring",
    )
    door_inner_trim_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.214, segments=96),
            [_circle_profile(0.178, segments=96)],
            0.024,
            center=True,
        ),
        "dryer_door_inner_trim",
    )
    drum_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.045, 0.000),
                (0.090, 0.018),
                (0.248, 0.050),
                (0.268, 0.110),
                (0.272, 0.520),
                (0.278, 0.565),
                (0.266, 0.595),
            ],
            [
                (0.000, 0.000),
                (0.045, 0.010),
                (0.228, 0.050),
                (0.252, 0.110),
                (0.252, 0.525),
                (0.232, 0.575),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "dryer_drum_shell",
    )
    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((side_thickness, cabinet_depth, 0.914)),
        origin=Origin(xyz=(-(cabinet_width / 2.0) + (side_thickness / 2.0), 0.0, 0.457)),
        material=white_enamel,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((side_thickness, cabinet_depth, 0.914)),
        origin=Origin(xyz=((cabinet_width / 2.0) - (side_thickness / 2.0), 0.0, 0.457)),
        material=white_enamel,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * side_thickness, cabinet_depth - 0.036, side_thickness)),
        origin=Origin(xyz=(0.0, -0.008, side_thickness / 2.0)),
        material=white_enamel,
        name="bottom_pan",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * side_thickness, rear_panel_thickness, 0.914)),
        origin=Origin(
            xyz=(0.0, -(cabinet_depth / 2.0) + (rear_panel_thickness / 2.0), 0.457)
        ),
        material=warm_white,
        name="rear_panel",
    )
    cabinet.visual(
        front_fascia_mesh,
        origin=Origin(xyz=(0.0, 0.358, door_center_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=white_enamel,
        name="front_fascia",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * side_thickness, 0.110, 0.078)),
        origin=Origin(xyz=(0.0, 0.295, 0.866), rpy=(-0.18, 0.0, 0.0)),
        material=white_enamel,
        name="control_console_body",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * side_thickness, 0.042, 0.030)),
        origin=Origin(xyz=(0.0, 0.268, 0.867)),
        material=warm_white,
        name="front_top_beam",
    )
    cabinet.visual(
        Box((0.44, 0.040, 0.032)),
        origin=Origin(xyz=(0.0, -0.348, 0.896)),
        material=warm_white,
        name="rear_top_beam",
    )
    cabinet.visual(
        Box((0.190, 0.010, 0.060)),
        origin=Origin(xyz=(0.090, 0.340, 0.873), rpy=(-0.18, 0.0, 0.0)),
        material=console_black,
        name="display_panel",
    )
    cabinet.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(-0.132, 0.335, 0.868), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="selector_dial",
    )
    cabinet.visual(
        Box((0.024, 0.006, 0.010)),
        origin=Origin(xyz=(0.232, 0.336, 0.872), rpy=(-0.18, 0.0, 0.0)),
        material=led_blue,
        name="wifi_status_light",
    )
    cabinet.visual(
        Cylinder(radius=0.052, length=0.026),
        origin=Origin(xyz=(0.0, -0.349, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="rear_bearing_housing",
    )
    cabinet.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, -0.336, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="rear_bearing_cap",
    )
    cabinet.visual(
        Cylinder(radius=0.009, length=0.070),
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z + 0.175)),
        material=stainless,
        name="upper_hinge_barrel",
    )
    cabinet.visual(
        Cylinder(radius=0.009, length=0.070),
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z - 0.175)),
        material=stainless,
        name="lower_hinge_barrel",
    )
    cabinet.visual(
        Box((0.028, 0.022, 0.090)),
        origin=Origin(xyz=(door_hinge_x + 0.018, 0.365, door_center_z + 0.175)),
        material=white_enamel,
        name="upper_hinge_mount",
    )
    cabinet.visual(
        Box((0.028, 0.022, 0.090)),
        origin=Origin(xyz=(door_hinge_x + 0.018, 0.365, door_center_z - 0.175)),
        material=white_enamel,
        name="lower_hinge_mount",
    )
    cabinet.visual(
        Box((0.028, 0.022, 0.028)),
        origin=Origin(xyz=(-0.326, -0.352, top_cover_hinge_z - 0.010)),
        material=warm_white,
        name="left_cover_hinge_support",
    )
    cabinet.visual(
        Box((0.028, 0.022, 0.028)),
        origin=Origin(xyz=(0.326, -0.352, top_cover_hinge_z - 0.010)),
        material=warm_white,
        name="right_cover_hinge_support",
    )
    cabinet.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(
            xyz=(-0.310, top_cover_hinge_y, top_cover_hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="left_cover_hinge_pin",
    )
    cabinet.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(
            xyz=(0.310, top_cover_hinge_y, top_cover_hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="right_cover_hinge_pin",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=46.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height / 2.0)),
    )

    drum = model.part("drum")
    drum.visual(
        drum_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=drum_metal,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.066, length=0.045),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="axle_stub",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.272, length=0.595),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.292, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        door_outer_ring_mesh,
        origin=Origin(xyz=(-0.275, 0.028, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=white_enamel,
        name="door_outer_ring",
    )
    door.visual(
        door_inner_trim_mesh,
        origin=Origin(xyz=(-0.275, 0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="door_inner_trim",
    )
    door.visual(
        Cylinder(radius=0.186, length=0.010),
        origin=Origin(xyz=(-0.275, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tinted_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.050, 0.020, 0.160)),
        origin=Origin(xyz=(-0.472, 0.012, 0.0)),
        material=warm_white,
        name="handle_recess_bezel",
    )
    door.visual(
        Box((0.020, 0.014, 0.120)),
        origin=Origin(xyz=(-0.472, 0.002, 0.0)),
        material=console_black,
        name="handle_bar",
    )
    door.visual(
        Box((0.018, 0.028, 0.070)),
        origin=Origin(xyz=(-0.018, 0.009, 0.175)),
        material=white_enamel,
        name="upper_hinge_leaf",
    )
    door.visual(
        Box((0.018, 0.028, 0.070)),
        origin=Origin(xyz=(-0.018, 0.009, -0.175)),
        material=white_enamel,
        name="lower_hinge_leaf",
    )
    door.visual(
        Box((0.200, 0.018, 0.050)),
        origin=Origin(xyz=(-0.100, 0.021, 0.175)),
        material=warm_white,
        name="upper_hinge_bridge",
    )
    door.visual(
        Box((0.200, 0.018, 0.050)),
        origin=Origin(xyz=(-0.100, 0.021, -0.175)),
        material=warm_white,
        name="lower_hinge_bridge",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.56, 0.08, 0.56)),
        mass=4.8,
        origin=Origin(xyz=(-0.275, 0.028, 0.0)),
    )

    top_cover = model.part("top_cover")
    top_cover.visual(
        Box((cabinet_width - 2.0 * side_thickness, 0.540, 0.012)),
        origin=Origin(xyz=(0.0, 0.278, 0.014)),
        material=white_enamel,
        name="cover_panel",
    )
    top_cover.visual(
        Box((cabinet_width - 2.0 * side_thickness, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, 0.548, 0.006)),
        material=white_enamel,
        name="front_cover_lip",
    )
    top_cover.visual(
        Box((0.012, 0.540, 0.022)),
        origin=Origin(xyz=(-0.315, 0.278, 0.005)),
        material=warm_white,
        name="left_cover_hem",
    )
    top_cover.visual(
        Box((0.012, 0.540, 0.022)),
        origin=Origin(xyz=(0.315, 0.278, 0.005)),
        material=warm_white,
        name="right_cover_hem",
    )
    top_cover.visual(
        Box((0.028, 0.020, 0.020)),
        origin=Origin(xyz=(-0.282, 0.000, 0.000)),
        material=stainless,
        name="left_cover_hinge_tab",
    )
    top_cover.visual(
        Box((0.028, 0.020, 0.020)),
        origin=Origin(xyz=(0.282, 0.000, 0.000)),
        material=stainless,
        name="right_cover_hinge_tab",
    )
    top_cover.inertial = Inertial.from_geometry(
        Box((cabinet_width - 2.0 * side_thickness, 0.540, 0.040)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.278, 0.014)),
    )

    model.articulation(
        "drum_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, -0.310, door_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=25.0),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.5, lower=0.0, upper=1.92),
    )
    model.articulation(
        "top_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=top_cover,
        origin=Origin(xyz=(0.0, top_cover_hinge_y, top_cover_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    top_cover = object_model.get_part("top_cover")
    drum_spin = object_model.get_articulation("drum_spin")
    door_hinge = object_model.get_articulation("door_hinge")
    top_cover_hinge = object_model.get_articulation("top_cover_hinge")

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

    drum_limits = drum_spin.motion_limits
    ctx.check(
        "drum uses a continuous front-back spin axis",
        drum_spin.articulation_type == ArticulationType.CONTINUOUS
        and drum_limits is not None
        and drum_limits.lower is None
        and drum_limits.upper is None
        and abs(drum_spin.axis[0]) < 1e-9
        and abs(drum_spin.axis[1] - 1.0) < 1e-9
        and abs(drum_spin.axis[2]) < 1e-9,
        details=f"type={drum_spin.articulation_type}, axis={drum_spin.axis}, limits={drum_limits}",
    )
    ctx.check(
        "door is right-hinged with realistic opening range",
        door_hinge.motion_limits is not None
        and abs(door_hinge.origin.xyz[0] - 0.275) < 0.01
        and abs(door_hinge.axis[2] + 1.0) < 1e-9
        and door_hinge.motion_limits.upper is not None
        and 1.6 <= door_hinge.motion_limits.upper <= 2.1,
        details=f"origin={door_hinge.origin.xyz}, axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )
    ctx.check(
        "top cover hinges from the rear edge",
        top_cover_hinge.motion_limits is not None
        and top_cover_hinge.motion_limits.upper is not None
        and abs(top_cover_hinge.origin.xyz[1] + 0.348) < 0.01
        and abs(top_cover_hinge.axis[0] - 1.0) < 1e-9,
        details=f"origin={top_cover_hinge.origin.xyz}, axis={top_cover_hinge.axis}",
    )

    ctx.expect_contact(
        drum,
        cabinet,
        elem_a="axle_stub",
        elem_b="rear_bearing_cap",
        contact_tol=0.002,
        name="drum axle seats on the rear bearing cap",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a="upper_hinge_leaf",
        elem_b="upper_hinge_barrel",
        contact_tol=0.002,
        name="upper barrel hinge mounts the door",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a="lower_hinge_leaf",
        elem_b="lower_hinge_barrel",
        contact_tol=0.002,
        name="lower barrel hinge mounts the door",
    )
    ctx.expect_overlap(
        cabinet,
        top_cover,
        axes="yz",
        elem_a="left_cover_hinge_pin",
        elem_b="left_cover_hinge_tab",
        min_overlap=0.007,
        name="left rear hinge pin aligns with the service-cover hinge tab",
    )
    ctx.expect_gap(
        top_cover,
        cabinet,
        axis="x",
        positive_elem="left_cover_hinge_tab",
        negative_elem="left_cover_hinge_pin",
        max_gap=0.001,
        max_penetration=0.0,
        name="left rear hinge pin meets the service-cover hinge tab",
    )
    ctx.expect_overlap(
        cabinet,
        top_cover,
        axes="yz",
        elem_a="right_cover_hinge_pin",
        elem_b="right_cover_hinge_tab",
        min_overlap=0.007,
        name="right rear hinge pin aligns with the service-cover hinge tab",
    )
    ctx.expect_gap(
        cabinet,
        top_cover,
        axis="x",
        positive_elem="right_cover_hinge_pin",
        negative_elem="right_cover_hinge_tab",
        max_gap=0.001,
        max_penetration=0.0,
        name="right rear hinge pin meets the service-cover hinge tab",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem="door_outer_ring",
            negative_elem="front_fascia",
            max_gap=0.015,
            max_penetration=0.0,
            name="closed door sits just proud of the front fascia",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            elem_a="door_outer_ring",
            elem_b="front_fascia",
            min_overlap=0.30,
            name="door covers the porthole opening footprint",
        )

    closed_door_box = ctx.part_element_world_aabb(door, elem="door_outer_ring")
    with ctx.pose({door_hinge: 1.70}):
        opened_door_box = ctx.part_element_world_aabb(door, elem="door_outer_ring")
    ctx.check(
        "door swings outward from the right edge",
        closed_door_box is not None
        and opened_door_box is not None
        and opened_door_box[1][1] > closed_door_box[1][1] + 0.12,
        details=f"closed={closed_door_box}, opened={opened_door_box}",
    )

    closed_cover_box = ctx.part_world_aabb(top_cover)
    with ctx.pose({top_cover_hinge: 1.10}):
        opened_cover_box = ctx.part_world_aabb(top_cover)
    ctx.check(
        "service cover lifts upward on the rear hinge line",
        closed_cover_box is not None
        and opened_cover_box is not None
        and opened_cover_box[1][2] > closed_cover_box[1][2] + 0.16,
        details=f"closed={closed_cover_box}, opened={opened_cover_box}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
