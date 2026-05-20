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


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_condensation_dryer")

    cabinet_white = model.material("cabinet_white", rgba=(0.94, 0.95, 0.96, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    glass_smoke = model.material("glass_smoke", rgba=(0.30, 0.38, 0.44, 0.38))
    drum_metal = model.material("drum_metal", rgba=(0.72, 0.75, 0.78, 1.0))
    seal_grey = model.material("seal_grey", rgba=(0.55, 0.56, 0.58, 1.0))
    filter_grey = model.material("filter_grey", rgba=(0.34, 0.37, 0.40, 1.0))
    screen_dark = model.material("screen_dark", rgba=(0.18, 0.20, 0.22, 1.0))

    cabinet_width = 0.595
    cabinet_depth = 0.540
    cabinet_height = 0.850
    wall_thickness = 0.018
    front_frame_thickness = 0.024
    front_frame_y = (cabinet_depth * 0.5) - (front_frame_thickness * 0.5)
    door_center_z = 0.410
    opening_radius = 0.205
    opening_collar_radius = 0.224
    door_outer_radius = 0.247
    door_glass_radius = 0.180

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((cabinet_width, wall_thickness, cabinet_height)),
        origin=Origin(xyz=(0.0, -(cabinet_depth * 0.5) + (wall_thickness * 0.5), cabinet_height * 0.5)),
        material=cabinet_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((wall_thickness, cabinet_depth - wall_thickness, cabinet_height)),
        origin=Origin(xyz=(-(cabinet_width * 0.5) + (wall_thickness * 0.5), 0.0, cabinet_height * 0.5)),
        material=cabinet_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall_thickness, cabinet_depth - wall_thickness, cabinet_height)),
        origin=Origin(xyz=((cabinet_width * 0.5) - (wall_thickness * 0.5), 0.0, cabinet_height * 0.5)),
        material=cabinet_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth - wall_thickness, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, wall_thickness * 0.5)),
        material=cabinet_white,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth - wall_thickness, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - (wall_thickness * 0.5))),
        material=cabinet_white,
        name="top_panel",
    )

    front_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(cabinet_width, cabinet_height, 0.024, corner_segments=8),
            [_circle_profile(opening_radius, segments=72)],
            height=front_frame_thickness,
            center=True,
        ),
        "dryer_front_frame",
    )
    cabinet.visual(
        front_frame_mesh,
        origin=Origin(xyz=(0.0, front_frame_y, cabinet_height * 0.5), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cabinet_white,
        name="body_front_frame",
    )

    opening_collar_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (opening_collar_radius, -0.035),
                (opening_collar_radius, 0.035),
            ],
            [
                (opening_radius, -0.030),
                (opening_radius, 0.030),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "dryer_opening_collar",
    )
    cabinet.visual(
        opening_collar_mesh,
        origin=Origin(xyz=(0.0, 0.220, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seal_grey,
        name="opening_collar",
    )

    cabinet.visual(
        Box((0.430, 0.010, 0.078)),
        origin=Origin(xyz=(0.0, 0.269, 0.775)),
        material=trim_dark,
        name="control_band",
    )
    cabinet.visual(
        Box((0.318, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, 0.273, 0.775)),
        material=glass_smoke,
        name="display_window",
    )

    filter_z = 0.290
    cabinet.visual(
        Box((0.012, 0.110, 0.028)),
        origin=Origin(xyz=(-0.146, 0.190, filter_z)),
        material=seal_grey,
        name="filter_left_rail",
    )
    cabinet.visual(
        Box((0.012, 0.110, 0.028)),
        origin=Origin(xyz=(0.146, 0.190, filter_z)),
        material=seal_grey,
        name="filter_right_rail",
    )
    cabinet.visual(
        Box((0.320, 0.110, 0.010)),
        origin=Origin(xyz=(0.0, 0.191, 0.271)),
        material=seal_grey,
        name="filter_shelf",
    )
    cabinet.visual(
        Box((0.010, 0.038, 0.060)),
        origin=Origin(xyz=(-0.256, 0.287, door_center_z + 0.145)),
        material=seal_grey,
        name="hinge_mount_top",
    )
    cabinet.visual(
        Box((0.010, 0.038, 0.060)),
        origin=Origin(xyz=(-0.256, 0.287, door_center_z - 0.145)),
        material=seal_grey,
        name="hinge_mount_bottom",
    )

    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    door = model.part("door")
    door_ring_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (door_outer_radius, -0.004),
                (door_outer_radius, 0.042),
            ],
            [
                (door_glass_radius, 0.002),
                (door_glass_radius, 0.036),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "dryer_door_ring",
    )
    door.visual(
        door_ring_mesh,
        origin=Origin(xyz=(0.238, 0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=door_glass_radius + 0.010, length=0.012),
        origin=Origin(xyz=(0.238, 0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass_smoke,
        name="door_glass",
    )
    door.visual(
        Box((0.022, 0.038, 0.405)),
        origin=Origin(xyz=(0.008, 0.022, 0.0)),
        material=trim_dark,
        name="hinge_stile",
    )
    door.visual(
        Box((0.028, 0.020, 0.165)),
        origin=Origin(xyz=(0.456, 0.022, 0.0)),
        material=seal_grey,
        name="door_handle",
    )
    door.visual(
        Cylinder(radius=0.013, length=0.054),
        origin=Origin(xyz=(0.0, 0.026, 0.145)),
        material=trim_dark,
        name="hinge_barrel_top",
    )
    door.visual(
        Cylinder(radius=0.013, length=0.054),
        origin=Origin(xyz=(0.0, 0.026, -0.145)),
        material=trim_dark,
        name="hinge_barrel_bottom",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.505, 0.060, 0.520)),
        mass=4.4,
        origin=Origin(xyz=(0.238, 0.024, 0.0)),
    )

    lint_filter = model.part("lint_filter")
    lint_filter.visual(
        Box((0.280, 0.095, 0.035)),
        origin=Origin(xyz=(0.0, -0.0475, 0.0)),
        material=filter_grey,
        name="filter_cassette",
    )
    lint_filter.visual(
        Box((0.245, 0.082, 0.006)),
        origin=Origin(xyz=(0.0, -0.045, 0.008)),
        material=screen_dark,
        name="filter_screen",
    )
    lint_filter.visual(
        Box((0.282, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.007, 0.003)),
        material=trim_dark,
        name="filter_pull",
    )
    lint_filter.inertial = Inertial.from_geometry(
        Box((0.300, 0.110, 0.040)),
        mass=0.45,
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
    )

    drum = model.part("drum")
    drum_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.212, -0.190),
                (0.212, 0.190),
            ],
            [
                (0.200, -0.178),
                (0.200, 0.178),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "dryer_drum_shell",
    )
    drum.visual(
        drum_shell_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_metal,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.0, -0.180, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_metal,
        name="drum_rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.202, length=0.008),
        origin=Origin(xyz=(0.0, -0.186, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_metal,
        name="drum_rear_wall",
    )
    for index, angle in enumerate((0.0, (2.0 * math.pi) / 3.0, (4.0 * math.pi) / 3.0)):
        drum.visual(
            Box((0.170, 0.012, 0.010)),
            origin=Origin(
                xyz=(0.120 * math.cos(angle), -0.182, 0.120 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=drum_metal,
            name=f"drum_rear_spoke_{index}",
        )
    for index, angle in enumerate((0.25, 2.35, 4.45)):
        drum.visual(
            Box((0.034, 0.280, 0.020)),
            origin=Origin(
                xyz=(0.196 * math.cos(angle), 0.0, 0.196 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=drum_metal,
            name=f"drum_paddle_{index}",
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.212, length=0.380),
        mass=8.5,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.238, 0.289, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.6,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "cabinet_to_lint_filter",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lint_filter,
        origin=Origin(xyz=(0.0, 0.238, filter_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, -0.065, door_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=8.0,
        ),
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

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    lint_filter = object_model.get_part("lint_filter")
    drum = object_model.get_part("drum")

    door_hinge = object_model.get_articulation("cabinet_to_door")
    filter_slide = object_model.get_articulation("cabinet_to_lint_filter")
    drum_spin = object_model.get_articulation("cabinet_to_drum")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    ctx.check(
        "door articulation is vertical revolute",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_hinge.axis) == (0.0, 0.0, 1.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper >= 1.5,
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )
    ctx.check(
        "lint filter articulation is outward prismatic",
        filter_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(filter_slide.axis) == (0.0, 1.0, 0.0)
        and filter_slide.motion_limits is not None
        and filter_slide.motion_limits.upper is not None
        and filter_slide.motion_limits.upper >= 0.075,
        details=f"type={filter_slide.articulation_type}, axis={filter_slide.axis}, limits={filter_slide.motion_limits}",
    )
    ctx.check(
        "drum articulation is continuous about depth axis",
        drum_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(drum_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={drum_spin.articulation_type}, axis={drum_spin.axis}",
    )

    with ctx.pose({door_hinge: 0.0, filter_slide: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem="door_ring",
            negative_elem="body_front_frame",
            min_gap=0.0,
            max_gap=0.002,
            name="door seats against the cabinet front",
        )
        ctx.expect_gap(
            door,
            drum,
            axis="y",
            positive_elem="door_glass",
            negative_elem="drum_shell",
            min_gap=0.080,
            name="door glass clears the drum front",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            elem_a="door_ring",
            elem_b="body_front_frame",
            min_overlap=0.350,
            name="door covers the front opening",
        )

    with ctx.pose({door_hinge: 0.0}):
        closed_handle = _aabb_center(ctx.part_element_world_aabb(door, elem="door_handle"))
    with ctx.pose({door_hinge: 1.25}):
        open_handle = _aabb_center(ctx.part_element_world_aabb(door, elem="door_handle"))
    ctx.check(
        "door swings outward when opened",
        closed_handle is not None
        and open_handle is not None
        and open_handle[1] > closed_handle[1] + 0.150
        and open_handle[0] < closed_handle[0] - 0.020,
        details=f"closed={closed_handle}, open={open_handle}",
    )

    with ctx.pose({filter_slide: 0.0}):
        filter_rest = ctx.part_world_position(lint_filter)
    with ctx.pose({door_hinge: 1.25, filter_slide: 0.080}):
        filter_extended = ctx.part_world_position(lint_filter)
        ctx.expect_gap(
            lint_filter,
            cabinet,
            axis="y",
            positive_elem="filter_pull",
            negative_elem="body_front_frame",
            min_gap=0.015,
            name="extended filter pull clears the front fascia",
        )
    ctx.check(
        "lint filter slides outward from the opening rim",
        filter_rest is not None and filter_extended is not None and filter_extended[1] > filter_rest[1] + 0.075,
        details=f"rest={filter_rest}, extended={filter_extended}",
    )

    with ctx.pose({drum_spin: 0.0}):
        paddle_rest = _aabb_center(ctx.part_element_world_aabb(drum, elem="drum_paddle_0"))
    with ctx.pose({drum_spin: math.pi / 2.0}):
        paddle_spun = _aabb_center(ctx.part_element_world_aabb(drum, elem="drum_paddle_0"))
    ctx.check(
        "drum rotation moves an internal paddle around the axis",
        paddle_rest is not None
        and paddle_spun is not None
        and abs(paddle_spun[0] - paddle_rest[0]) > 0.100
        and abs(paddle_spun[2] - paddle_rest[2]) > 0.100,
        details=f"rest={paddle_rest}, spun={paddle_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
