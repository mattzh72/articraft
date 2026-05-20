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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


HOUSING_WIDTH = 0.56
HOUSING_HEIGHT = 0.62
HOUSING_DEPTH = 0.24
HOUSING_CENTER_Z = HOUSING_HEIGHT * 0.5
FRONT_PANEL_THICKNESS = 0.018
FRONT_PANEL_CENTER_Y = 0.111
OPENING_RADIUS = 0.173
OPENING_CENTER_Z = 0.285
DRUM_CENTER_Y = -0.012
DOOR_HINGE_X = -0.232
DOOR_HINGE_Y = 0.133
DOOR_PANEL_CENTER_X = abs(DOOR_HINGE_X)


def _circle_profile(
    radius: float,
    *,
    segments: int = 48,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _clipped_circle_profile(
    radius: float,
    clip_x: float,
    *,
    segments: int = 72,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    clip_x = max(cx - radius + 1e-4, min(clip_x, cx + radius - 1e-4))
    angle = math.acos((clip_x - cx) / radius)
    clip_y = radius * math.sin(angle)
    points = [(clip_x, cy + clip_y), (clip_x, cy - clip_y)]
    for index in range(1, segments):
        theta = -angle + (2.0 * angle * index) / segments
        points.append((cx + radius * math.cos(theta), cy + radius * math.sin(theta)))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_compact_washer")

    white_enamel = model.material("white_enamel", rgba=(0.95, 0.96, 0.97, 1.0))
    bracket_metal = model.material("bracket_metal", rgba=(0.23, 0.25, 0.28, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.76, 0.79, 0.82, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.70, 0.82, 0.90, 0.35))
    gasket_rubber = model.material("gasket_rubber", rgba=(0.09, 0.10, 0.11, 1.0))
    paddle_gray = model.material("paddle_gray", rgba=(0.82, 0.84, 0.86, 1.0))

    front_fascia_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(HOUSING_WIDTH, HOUSING_HEIGHT, radius=0.045, corner_segments=8),
            [
                _circle_profile(
                    OPENING_RADIUS,
                    segments=72,
                    center=(0.0, OPENING_CENTER_Z - HOUSING_CENTER_Z),
                )
            ],
            FRONT_PANEL_THICKNESS,
            center=True,
        ),
        "washer_front_fascia",
    )
    door_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _clipped_circle_profile(0.188, -0.150, segments=96),
            [_circle_profile(0.142, segments=72)],
            0.045,
            center=True,
        ),
        "washer_door_ring",
    )
    drum_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.145, -0.080),
                (0.152, -0.068),
                (0.152, 0.068),
                (0.145, 0.080),
            ],
            [
                (0.128, -0.074),
                (0.135, -0.060),
                (0.135, 0.070),
                (0.128, 0.074),
            ],
            segments=64,
        ),
        "washer_drum_shell",
    )
    drum_bulkhead_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.130, segments=72),
            [_circle_profile(0.033, segments=48)],
            0.006,
            center=True,
        ),
        "washer_drum_bulkhead",
    )
    drum_rim_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.145, tube=0.007, radial_segments=16, tubular_segments=56),
        "washer_drum_rim",
    )
    gasket_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.134, tube=0.010, radial_segments=16, tubular_segments=56),
        "washer_gasket_ring",
    )

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.34, 0.012, 0.46)),
        origin=Origin(xyz=(0.0, -0.176, 0.34)),
        material=bracket_metal,
        name="wall_plate",
    )
    wall_bracket.visual(
        Box((0.03, 0.06, 0.40)),
        origin=Origin(xyz=(-0.115, -0.150, 0.30)),
        material=bracket_metal,
        name="left_support_arm",
    )
    wall_bracket.visual(
        Box((0.03, 0.06, 0.40)),
        origin=Origin(xyz=(0.115, -0.150, 0.30)),
        material=bracket_metal,
        name="right_support_arm",
    )
    wall_bracket.visual(
        Box((0.24, 0.06, 0.028)),
        origin=Origin(xyz=(0.0, -0.150, 0.50)),
        material=bracket_metal,
        name="top_mount_rail",
    )
    wall_bracket.visual(
        Box((0.28, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, -0.150, 0.09)),
        material=bracket_metal,
        name="bottom_mount_rail",
    )
    wall_bracket.visual(
        Box((0.12, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, -0.150, 0.31)),
        material=bracket_metal,
        name="center_saddle",
    )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.34, 0.06, 0.50)),
        mass=5.5,
        origin=Origin(xyz=(0.0, -0.150, 0.30)),
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.52, 0.006, 0.58)),
        origin=Origin(xyz=(0.0, -0.117, HOUSING_CENTER_Z)),
        material=white_enamel,
        name="back_panel",
    )
    housing.visual(
        Box((0.02, 0.234, 0.58)),
        origin=Origin(xyz=(-0.27, -0.003, HOUSING_CENTER_Z)),
        material=white_enamel,
        name="left_side",
    )
    housing.visual(
        Box((0.02, 0.234, 0.58)),
        origin=Origin(xyz=(0.27, -0.003, HOUSING_CENTER_Z)),
        material=white_enamel,
        name="right_side",
    )
    housing.visual(
        Box((0.52, 0.234, 0.02)),
        origin=Origin(xyz=(0.0, -0.003, 0.01)),
        material=white_enamel,
        name="bottom_panel",
    )
    housing.visual(
        Box((0.52, 0.234, 0.02)),
        origin=Origin(xyz=(0.0, -0.003, 0.61)),
        material=white_enamel,
        name="top_panel",
    )
    housing.visual(
        front_fascia_mesh,
        origin=Origin(xyz=(0.0, FRONT_PANEL_CENTER_Y, HOUSING_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white_enamel,
        name="front_fascia",
    )
    housing.visual(
        Box((0.20, 0.010, 0.050)),
        origin=Origin(xyz=(0.085, 0.116, 0.53)),
        material=dark_trim,
        name="control_strip",
    )
    housing.visual(
        Cylinder(radius=0.033, length=0.020),
        origin=Origin(xyz=(0.0, -0.110, OPENING_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="axle_bearing",
    )
    housing.visual(
        Box((0.022, 0.010, 0.050)),
        origin=Origin(xyz=(0.185, 0.115, OPENING_CENTER_Z)),
        material=dark_trim,
        name="latch_receiver",
    )
    housing.visual(
        Box((0.028, 0.020, 0.272)),
        origin=Origin(xyz=(-0.246, 0.114, OPENING_CENTER_Z)),
        material=dark_trim,
        name="hinge_mount_column",
    )
    for name, local_z in (
        ("upper_hinge_barrel_top", 0.126),
        ("upper_hinge_barrel_bottom", 0.094),
        ("lower_hinge_barrel_top", -0.094),
        ("lower_hinge_barrel_bottom", -0.126),
    ):
        housing.visual(
            Cylinder(radius=0.009, length=0.014),
            origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, OPENING_CENTER_Z + local_z)),
            material=dark_trim,
            name=name,
        )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_CENTER_Z)),
    )

    drum = model.part("drum")
    drum.visual(
        drum_shell_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        drum_rim_mesh,
        origin=Origin(xyz=(0.0, 0.071, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="front_rim",
    )
    drum.visual(
        Cylinder(radius=0.135, length=0.006),
        origin=Origin(xyz=(0.0, -0.077, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="rear_web",
    )
    drum.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, -0.090, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="rear_hub",
    )
    paddle_radius = 0.126
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.035, 0.110, 0.018)),
            origin=Origin(
                xyz=(
                    paddle_radius * math.sin(angle),
                    0.0,
                    paddle_radius * math.cos(angle),
                ),
                rpy=(0.0, angle, 0.0),
            ),
            material=paddle_gray,
            name=f"paddle_{index}",
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=0.16),
        mass=6.0,
    )

    door = model.part("door")
    door.visual(
        door_ring_mesh,
        origin=Origin(xyz=(DOOR_PANEL_CENTER_X, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white_enamel,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.147, length=0.008),
        origin=Origin(xyz=(DOOR_PANEL_CENTER_X, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass_tint,
        name="porthole_glass",
    )
    door.visual(
        gasket_ring_mesh,
        origin=Origin(xyz=(DOOR_PANEL_CENTER_X, -0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_rubber,
        name="gasket_ring",
    )
    door.visual(
        Box((0.052, 0.020, 0.300)),
        origin=Origin(xyz=(0.056, 0.008, 0.0)),
        material=white_enamel,
        name="hinge_rail",
    )
    door.visual(
        Box((0.026, 0.010, 0.028)),
        origin=Origin(xyz=(0.030, 0.018, 0.110)),
        material=dark_trim,
        name="upper_hinge_arm",
    )
    door.visual(
        Box((0.026, 0.010, 0.028)),
        origin=Origin(xyz=(0.030, 0.018, -0.110)),
        material=dark_trim,
        name="lower_hinge_arm",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.0, 0.002, 0.110)),
        material=dark_trim,
        name="upper_door_knuckle",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.0, 0.002, -0.110)),
        material=dark_trim,
        name="lower_door_knuckle",
    )
    door.visual(
        Box((0.024, 0.010, 0.016)),
        origin=Origin(xyz=(0.012, 0.014, 0.110)),
        material=dark_trim,
        name="upper_knuckle_bridge",
    )
    door.visual(
        Box((0.024, 0.010, 0.016)),
        origin=Origin(xyz=(0.012, 0.014, -0.110)),
        material=dark_trim,
        name="lower_knuckle_bridge",
    )
    door.visual(
        Box((0.052, 0.020, 0.080)),
        origin=Origin(xyz=(0.375, 0.010, 0.0)),
        material=white_enamel,
        name="latch_bridge",
    )
    door.visual(
        Box((0.018, 0.010, 0.042)),
        origin=Origin(xyz=(0.417, -0.008, 0.0)),
        material=dark_trim,
        name="door_latch_pad",
    )
    door.visual(
        Box((0.030, 0.018, 0.052)),
        origin=Origin(xyz=(0.381, 0.024, 0.0)),
        material=white_enamel,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.05),
        mass=2.2,
        origin=Origin(xyz=(DOOR_PANEL_CENTER_X, 0.013, 0.0)),
    )

    model.articulation(
        "bracket_to_housing",
        ArticulationType.FIXED,
        parent=wall_bracket,
        child=housing,
        origin=Origin(),
    )
    model.articulation(
        "drum_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=drum,
        origin=Origin(xyz=(0.0, DRUM_CENTER_Y, OPENING_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=20.0,
        ),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, OPENING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(135.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    housing = object_model.get_part("housing")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drum_spin = object_model.get_articulation("drum_spin")
    door_hinge = object_model.get_articulation("door_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        drum,
        housing,
        elem_a="rear_hub",
        elem_b="axle_bearing",
        reason="The drum spindle is intentionally seated inside the housing bearing cartridge.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "drum_spin_axis_is_y",
        tuple(drum_spin.axis) == (0.0, 1.0, 0.0),
        details=f"Expected drum axis (0, 1, 0), got {drum_spin.axis!r}",
    )
    ctx.check(
        "door_hinge_axis_is_z",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"Expected door hinge axis (0, 0, 1), got {door_hinge.axis!r}",
    )
    ctx.check(
        "door_hinge_has_real_opening_range",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > math.radians(120.0),
        details="Door hinge should open from closed to a wide service angle.",
    )

    ctx.expect_contact(housing, wall_bracket)
    ctx.expect_contact(drum, housing, elem_a="rear_hub", elem_b="axle_bearing")
    ctx.expect_contact(door, housing, elem_a="door_latch_pad", elem_b="latch_receiver")
    ctx.expect_gap(
        door,
        housing,
        axis="y",
        positive_elem="door_ring",
        negative_elem="front_fascia",
        min_gap=0.001,
        max_gap=0.004,
    )
    ctx.expect_overlap(door, housing, axes="xz", elem_a="door_ring", elem_b="front_fascia", min_overlap=0.30)
    ctx.expect_within(drum, housing, axes="xz", margin=0.03)

    door_rest_aabb = ctx.part_world_aabb(door)
    assert door_rest_aabb is not None
    with ctx.pose({door_hinge: math.radians(75.0)}):
        door_open_aabb = ctx.part_world_aabb(door)
        assert door_open_aabb is not None
        assert door_open_aabb[1][1] > door_rest_aabb[1][1] + 0.20
        ctx.expect_gap(
            door,
            housing,
            axis="y",
            positive_elem="door_latch_pad",
            negative_elem="latch_receiver",
            min_gap=0.25,
        )

    paddle_rest_aabb = ctx.part_element_world_aabb(drum, elem="paddle_0")
    assert paddle_rest_aabb is not None
    rest_center = _aabb_center(paddle_rest_aabb)
    with ctx.pose({drum_spin: math.pi / 2.0}):
        paddle_turn_aabb = ctx.part_element_world_aabb(drum, elem="paddle_0")
        assert paddle_turn_aabb is not None
        turn_center = _aabb_center(paddle_turn_aabb)
        assert turn_center[0] > rest_center[0] + 0.09
        assert turn_center[2] < rest_center[2] - 0.09

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
