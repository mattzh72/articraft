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
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_HALF_WIDTH = 0.125
WHEEL_RADIUS = 0.13
WHEEL_WIDTH = 0.065
WHEEL_CENTER_Y = 0.185
AXLE_X = -0.045
AXLE_Z = WHEEL_RADIUS
TUBE_RADIUS = 0.016


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _side_rail_points(side_y: float) -> list[tuple[float, float, float]]:
    return [
        (-0.015, side_y, 0.145),
        (-0.028, side_y, 0.420),
        (-0.042, side_y, 0.770),
        (-0.055, side_y, 0.995),
        (0.000, side_y, 1.120),
        (0.075, side_y, 1.085),
        (0.080, side_y, 0.920),
        (0.022, side_y, 0.805),
    ]


def _wheel_shell_mesh():
    half_width = WHEEL_WIDTH * 0.5
    profile = [
        (WHEEL_RADIUS * 0.42, -half_width * 0.96),
        (WHEEL_RADIUS * 0.70, -half_width),
        (WHEEL_RADIUS * 0.90, -half_width * 0.78),
        (WHEEL_RADIUS * 0.98, -half_width * 0.34),
        (WHEEL_RADIUS, -half_width * 0.10),
        (WHEEL_RADIUS, half_width * 0.10),
        (WHEEL_RADIUS * 0.98, half_width * 0.34),
        (WHEEL_RADIUS * 0.90, half_width * 0.78),
        (WHEEL_RADIUS * 0.70, half_width),
        (WHEEL_RADIUS * 0.42, half_width * 0.96),
        (WHEEL_RADIUS * 0.34, half_width * 0.30),
        (WHEEL_RADIUS * 0.29, 0.0),
        (WHEEL_RADIUS * 0.34, -half_width * 0.30),
        (WHEEL_RADIUS * 0.42, -half_width * 0.96),
    ]
    return _save_mesh(
        "hand_truck_wheel_shell.obj",
        LatheGeometry(profile, segments=64).rotate_x(math.pi / 2.0),
    )


def _add_wheel_visuals(part, *, side_sign: float, rubber, wheel_steel, dark_steel, shell_mesh) -> None:
    part.visual(shell_mesh, material=rubber, name="tire_shell")

    hub_depth = 0.018
    cap_depth = 0.010
    hub_radius = 0.055
    outer_hub_radius = 0.047
    cap_radius = 0.024
    spin_rpy = (math.pi / 2.0, 0.0, 0.0)

    inner_local_y = -side_sign * (WHEEL_WIDTH * 0.5 - hub_depth * 0.5)
    outer_local_y = side_sign * (WHEEL_WIDTH * 0.5 - hub_depth * 0.5)
    cap_local_y = side_sign * (WHEEL_WIDTH * 0.5 - cap_depth * 0.5)

    part.visual(
        Cylinder(radius=hub_radius, length=hub_depth),
        origin=Origin(xyz=(0.0, inner_local_y, 0.0), rpy=spin_rpy),
        material=dark_steel,
        name="inner_hub",
    )
    part.visual(
        Cylinder(radius=outer_hub_radius, length=hub_depth),
        origin=Origin(xyz=(0.0, outer_local_y, 0.0), rpy=spin_rpy),
        material=wheel_steel,
        name="outer_hub",
    )
    part.visual(
        Cylinder(radius=cap_radius, length=cap_depth),
        origin=Origin(xyz=(0.0, cap_local_y, 0.0), rpy=spin_rpy),
        material=wheel_steel,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_hand_truck", assets=ASSETS)

    frame_red = model.material("frame_red", rgba=(0.71, 0.11, 0.09, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.74, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.08, 1.0))

    wheel_shell_mesh = _wheel_shell_mesh()

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.30, 0.42, 1.16)),
        mass=13.0,
        origin=Origin(xyz=(0.00, 0.0, 0.58)),
    )

    frame.visual(
        Box((0.18, 0.26, 0.012)),
        origin=Origin(xyz=(0.090, 0.0, 0.006)),
        material=frame_red,
        name="toe_plate",
    )
    frame.visual(
        Box((0.014, 0.26, 0.045)),
        origin=Origin(xyz=(0.173, 0.0, 0.0225)),
        material=frame_red,
        name="toe_lip",
    )
    frame.visual(
        Box((0.030, 0.22, 0.052)),
        origin=Origin(xyz=(0.002, 0.0, 0.026)),
        material=dark_steel,
        name="heel_block",
    )

    frame.visual(
        _save_mesh(
            "hand_truck_left_rail.obj",
            tube_from_spline_points(
                _side_rail_points(-FRAME_HALF_WIDTH),
                radius=TUBE_RADIUS,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=frame_red,
        name="left_rail",
    )
    frame.visual(
        _save_mesh(
            "hand_truck_right_rail.obj",
            tube_from_spline_points(
                _side_rail_points(FRAME_HALF_WIDTH),
                radius=TUBE_RADIUS,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=frame_red,
        name="right_rail",
    )

    _add_member(
        frame,
        (-0.022, -FRAME_HALF_WIDTH, 0.250),
        (-0.022, FRAME_HALF_WIDTH, 0.250),
        radius=0.012,
        material=frame_red,
        name="lower_brace",
    )
    _add_member(
        frame,
        (-0.036, -FRAME_HALF_WIDTH, 0.600),
        (-0.036, FRAME_HALF_WIDTH, 0.600),
        radius=0.012,
        material=frame_red,
        name="middle_brace",
    )
    _add_member(
        frame,
        (0.022, -FRAME_HALF_WIDTH, 0.805),
        (0.022, FRAME_HALF_WIDTH, 0.805),
        radius=0.012,
        material=frame_red,
        name="upper_brace",
    )
    _add_member(
        frame,
        (0.010, -FRAME_HALF_WIDTH, 1.112),
        (0.070, -FRAME_HALF_WIDTH, 1.084),
        radius=0.019,
        material=grip_black,
        name="left_grip",
    )
    _add_member(
        frame,
        (0.010, FRAME_HALF_WIDTH, 1.112),
        (0.070, FRAME_HALF_WIDTH, 1.084),
        radius=0.019,
        material=grip_black,
        name="right_grip",
    )

    _add_member(
        frame,
        (0.020, -0.095, 0.012),
        (-0.016, -FRAME_HALF_WIDTH, 0.145),
        radius=0.010,
        material=dark_steel,
        name="left_toe_gusset",
    )
    _add_member(
        frame,
        (0.020, 0.095, 0.012),
        (-0.016, FRAME_HALF_WIDTH, 0.145),
        radius=0.010,
        material=dark_steel,
        name="right_toe_gusset",
    )
    _add_member(
        frame,
        (-0.016, -FRAME_HALF_WIDTH, 0.145),
        (AXLE_X, -0.132, AXLE_Z),
        radius=0.010,
        material=dark_steel,
        name="left_axle_brace",
    )
    _add_member(
        frame,
        (-0.016, FRAME_HALF_WIDTH, 0.145),
        (AXLE_X, 0.132, AXLE_Z),
        radius=0.010,
        material=dark_steel,
        name="right_axle_brace",
    )
    _add_member(
        frame,
        (AXLE_X, -0.138, AXLE_Z),
        (AXLE_X, 0.138, AXLE_Z),
        radius=0.010,
        material=dark_steel,
        name="axle",
    )
    _add_member(
        frame,
        (AXLE_X, -0.1525, AXLE_Z),
        (AXLE_X, -0.138, AXLE_Z),
        radius=0.009,
        material=wheel_steel,
        name="left_stub_axle",
    )
    _add_member(
        frame,
        (AXLE_X, 0.138, AXLE_Z),
        (AXLE_X, 0.1525, AXLE_Z),
        radius=0.009,
        material=wheel_steel,
        name="right_stub_axle",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=2.6,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        left_wheel,
        side_sign=-1.0,
        rubber=rubber,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
        shell_mesh=wheel_shell_mesh,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=2.6,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        right_wheel,
        side_sign=1.0,
        rubber=rubber,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
        shell_mesh=wheel_shell_mesh,
    )

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(AXLE_X, -WHEEL_CENTER_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(AXLE_X, WHEEL_CENTER_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

    toe_plate = frame.get_visual("toe_plate")
    axle = frame.get_visual("axle")
    left_stub_axle = frame.get_visual("left_stub_axle")
    right_stub_axle = frame.get_visual("right_stub_axle")
    upper_brace = frame.get_visual("upper_brace")
    left_inner_hub = left_wheel.get_visual("inner_hub")
    right_inner_hub = right_wheel.get_visual("inner_hub")
    left_tire = left_wheel.get_visual("tire_shell")
    right_tire = right_wheel.get_visual("tire_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(frame, left_wheel, elem_a=left_stub_axle, elem_b=left_inner_hub)
    ctx.expect_contact(frame, right_wheel, elem_a=right_stub_axle, elem_b=right_inner_hub)
    ctx.expect_overlap(frame, left_wheel, axes="xz", elem_a=axle, elem_b=left_tire, min_overlap=0.015)
    ctx.expect_overlap(frame, right_wheel, axes="xz", elem_a=axle, elem_b=right_tire, min_overlap=0.015)
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="y", min_dist=0.36, max_dist=0.38)
    ctx.expect_overlap(frame, frame, axes="x", elem_a=toe_plate, elem_b=upper_brace, min_overlap=0.01)

    ctx.check(
        "left_wheel_joint_is_continuous",
        getattr(left_spin, "articulation_type", None) == ArticulationType.CONTINUOUS,
        "Left wheel should spin continuously on the axle.",
    )
    ctx.check(
        "right_wheel_joint_is_continuous",
        getattr(right_spin, "articulation_type", None) == ArticulationType.CONTINUOUS,
        "Right wheel should spin continuously on the axle.",
    )
    ctx.check(
        "left_wheel_joint_axis",
        tuple(getattr(left_spin, "axis", ())) == (0.0, 1.0, 0.0),
        "Left wheel spin axis should follow the axle across the truck width.",
    )
    ctx.check(
        "right_wheel_joint_axis",
        tuple(getattr(right_spin, "axis", ())) == (0.0, 1.0, 0.0),
        "Right wheel spin axis should follow the axle across the truck width.",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    toe_aabb = ctx.part_element_world_aabb(frame, elem=toe_plate)
    left_wheel_aabb = ctx.part_world_aabb(left_wheel)
    assert frame_aabb is not None
    assert toe_aabb is not None
    assert left_wheel_aabb is not None

    frame_size = tuple(frame_aabb[1][i] - frame_aabb[0][i] for i in range(3))
    toe_size = tuple(toe_aabb[1][i] - toe_aabb[0][i] for i in range(3))
    left_wheel_size = tuple(left_wheel_aabb[1][i] - left_wheel_aabb[0][i] for i in range(3))

    ctx.check(
        "frame_reads_tall_and_narrow",
        1.08 <= frame_size[2] <= 1.18 and 0.20 <= frame_size[1] <= 0.33,
        f"Frame extents were {frame_size}.",
    )
    ctx.check(
        "toe_plate_reads_flat_and_forward",
        0.17 <= toe_size[0] <= 0.19 and 0.24 <= toe_size[1] <= 0.27 and toe_size[2] <= 0.05,
        f"Toe plate extents were {toe_size}.",
    )
    ctx.check(
        "wheel_reads_large",
        0.24 <= left_wheel_size[0] <= 0.28 and 0.06 <= left_wheel_size[1] <= 0.08 and 0.24 <= left_wheel_size[2] <= 0.28,
        f"Wheel extents were {left_wheel_size}.",
    )

    left_rest = ctx.part_world_position(left_wheel)
    right_rest = ctx.part_world_position(right_wheel)
    assert left_rest is not None
    assert right_rest is not None
    with ctx.pose({left_spin: math.pi / 2.0, right_spin: -math.pi / 3.0}):
        left_turned = ctx.part_world_position(left_wheel)
        right_turned = ctx.part_world_position(right_wheel)
        assert left_turned is not None
        assert right_turned is not None
        ctx.check(
            "left_wheel_spin_keeps_center",
            max(abs(a - b) for a, b in zip(left_rest, left_turned)) <= 1e-9,
            f"Left wheel center moved from {left_rest} to {left_turned}.",
        )
        ctx.check(
            "right_wheel_spin_keeps_center",
            max(abs(a - b) for a, b in zip(right_rest, right_turned)) <= 1e-9,
            f"Right wheel center moved from {right_rest} to {right_turned}.",
        )
        ctx.expect_contact(frame, left_wheel, elem_a=left_stub_axle, elem_b=left_inner_hub)
        ctx.expect_contact(frame, right_wheel, elem_a=right_stub_axle, elem_b=right_inner_hub)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
