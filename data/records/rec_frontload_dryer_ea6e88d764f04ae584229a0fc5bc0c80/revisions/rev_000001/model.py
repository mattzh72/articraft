from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_h = height / 2.0
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 56,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos(2.0 * pi * i / segments),
            cy + radius * sin(2.0 * pi * i / segments),
        )
        for i in range(segments)
    ]


def _lathe_shell_mesh(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    name: str,
    *,
    segments: int = 72,
):
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    geom.rotate_x(pi / 2.0)
    return mesh_from_geometry(geom, name)


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laundry_center_dryer_section")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.97, 1.0))
    trim_white = model.material("trim_white", rgba=(0.90, 0.90, 0.92, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.68, 1.0))
    drum_gray = model.material("drum_gray", rgba=(0.33, 0.36, 0.40, 1.0))
    glass = model.material("glass", rgba=(0.58, 0.72, 0.82, 0.35))

    body_w = 0.68
    body_d = 0.72
    body_h = 1.02
    wall = 0.015
    front_y = body_d / 2.0

    drum_center_z = 0.63
    door_opening_r = 0.205
    door_outer_r = 0.255
    door_depth = 0.055
    flap_w = 0.24
    flap_h = 0.10
    flap_t = 0.018
    flap_bottom_z = 0.11

    body = model.part("body")
    body.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(-(body_w - wall) / 2.0, 0.0, body_h / 2.0)),
        material=body_white,
        name="left_wall",
    )
    body.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=((body_w - wall) / 2.0, 0.0, body_h / 2.0)),
        material=body_white,
        name="right_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=body_white,
        name="bottom_panel",
    )
    body.visual(
        Box((body_w - 2.0 * wall, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_h - wall / 2.0)),
        material=body_white,
        name="top_panel",
    )
    body.visual(
        Box((body_w - 2.0 * wall, wall, body_h - 2.0 * wall)),
        origin=Origin(xyz=(0.0, -front_y + wall / 2.0, body_h / 2.0)),
        material=body_white,
        name="rear_panel",
    )

    front_panel = ExtrudeWithHolesGeometry(
        _rect_profile(body_w - 2.0 * wall, body_h - 2.0 * wall),
        [
            _circle_profile(
                door_opening_r,
                center=(0.0, drum_center_z - body_h / 2.0),
                segments=72,
            )
        ],
        wall,
        center=True,
        cap=True,
        closed=True,
    )
    front_panel.rotate_x(pi / 2.0)
    body.visual(
        mesh_from_geometry(front_panel, "dryer_front_panel"),
        origin=Origin(xyz=(0.0, front_y - wall / 2.0, body_h / 2.0)),
        material=body_white,
        name="front_panel",
    )
    body.visual(
        _lathe_shell_mesh(
            [
                (0.220, -0.012),
                (0.220, 0.012),
            ],
            [
                (0.200, -0.012),
                (0.200, 0.012),
            ],
            "dryer_front_collar",
        ),
        origin=Origin(xyz=(0.0, front_y - wall - 0.012, drum_center_z)),
        material=trim_white,
        name="front_collar",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(
            xyz=(0.0, -front_y + wall, drum_center_z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="rear_bearing",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    drum = model.part("drum")
    drum.visual(
        _lathe_shell_mesh(
            [
                (0.040, -0.285),
                (0.225, -0.265),
                (0.225, 0.255),
                (0.205, 0.285),
            ],
            [
                (0.000, -0.285),
                (0.190, -0.245),
                (0.190, 0.250),
                (0.165, 0.275),
            ],
            "dryer_drum_shell",
        ),
        material=drum_gray,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.192, length=0.024),
        origin=Origin(xyz=(0.0, -0.255, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=drum_gray,
        name="rear_bulkhead",
    )
    drum.visual(
        Cylinder(radius=0.016, length=0.090),
        origin=Origin(xyz=(0.0, -0.285, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drum_axle",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.225, length=0.570),
        mass=8.5,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        _lathe_shell_mesh(
            [
                (0.225, -door_depth / 2.0),
                (0.255, -0.020),
                (0.255, 0.022),
                (0.235, door_depth / 2.0),
            ],
            [
                (0.162, -door_depth / 2.0),
                (0.180, -0.010),
                (0.180, 0.018),
                (0.158, door_depth / 2.0),
            ],
            "dryer_door_ring",
        ),
        origin=Origin(xyz=(door_outer_r, 0.0, 0.0)),
        material=trim_white,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.176, length=0.018),
        origin=Origin(xyz=(door_outer_r, 0.008, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Box((0.020, 0.040, 0.120)),
        origin=Origin(xyz=(0.455, 0.030, 0.0)),
        material=steel,
        name="door_handle",
    )
    for idx, hinge_z in enumerate((0.170, -0.170), start=1):
        door.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=steel,
            name=f"hinge_barrel_{idx}",
        )
        door.visual(
            Box((0.140, 0.018, 0.050)),
            origin=Origin(xyz=(0.060, 0.0, hinge_z)),
            material=steel,
            name=f"hinge_leaf_{idx}",
        )
    door.inertial = Inertial.from_geometry(
        Box((0.520, door_depth, 0.520)),
        mass=5.2,
        origin=Origin(xyz=(door_outer_r, 0.0, 0.0)),
    )

    lint_flap = model.part("lint_flap")
    lint_flap.visual(
        Box((flap_w, flap_t, flap_h)),
        origin=Origin(xyz=(0.0, 0.0, flap_h / 2.0)),
        material=trim_white,
        name="flap_panel",
    )
    lint_flap.visual(
        Cylinder(radius=0.006, length=flap_w * 0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="flap_hinge",
    )
    lint_flap.visual(
        Box((0.100, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.012, flap_h * 0.78)),
        material=steel,
        name="flap_pull",
    )
    lint_flap.inertial = Inertial.from_geometry(
        Box((flap_w, flap_t, flap_h)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, flap_h / 2.0)),
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, drum_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=9.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-door_outer_r, front_y + door_depth / 2.0, drum_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.85,
        ),
    )
    model.articulation(
        "body_to_lint_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lint_flap,
        origin=Origin(xyz=(0.0, front_y + flap_t / 2.0, flap_bottom_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    lint_flap = object_model.get_part("lint_flap")
    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    flap_joint = object_model.get_articulation("body_to_lint_flap")

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
        "all dryer section parts exist",
        all(part is not None for part in (body, drum, door, lint_flap)),
    )
    ctx.check(
        "drum uses a continuous front-back axle",
        drum_joint.articulation_type == ArticulationType.CONTINUOUS
        and drum_joint.motion_limits is not None
        and drum_joint.motion_limits.lower is None
        and drum_joint.motion_limits.upper is None
        and abs(drum_joint.axis[1]) > 0.99,
        details=(
            f"type={drum_joint.articulation_type}, axis={drum_joint.axis}, "
            f"limits={drum_joint.motion_limits}"
        ),
    )
    ctx.check(
        "door and flap hinges sit on the expected front edges",
        door_joint.origin.xyz[0] < -0.20
        and door_joint.origin.xyz[1] > 0.36
        and abs(door_joint.axis[2]) > 0.99
        and flap_joint.origin.xyz[2] < 0.20
        and flap_joint.origin.xyz[1] > 0.36
        and abs(flap_joint.axis[0]) > 0.99,
        details=(
            f"door_origin={door_joint.origin.xyz}, door_axis={door_joint.axis}, "
            f"flap_origin={flap_joint.origin.xyz}, flap_axis={flap_joint.axis}"
        ),
    )
    ctx.expect_contact(
        drum,
        body,
        elem_a="drum_axle",
        elem_b="rear_bearing",
        name="drum axle is carried by the rear bearing",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_contact(
            door,
            body,
            elem_a="door_ring",
            elem_b="front_panel",
            name="closed door seats against the front panel",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="door_ring",
            elem_b="front_panel",
            min_overlap=0.40,
            name="door covers the main porthole opening",
        )

    with ctx.pose({flap_joint: 0.0}):
        ctx.expect_contact(
            lint_flap,
            body,
            elem_a="flap_panel",
            elem_b="front_panel",
            name="closed lint flap sits flush on the lower front face",
        )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    with ctx.pose({door_joint: 1.20}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    closed_door_center = _aabb_center(closed_door_aabb)
    open_door_center = _aabb_center(open_door_aabb)
    ctx.check(
        "door swings outward from the left edge",
        closed_door_center is not None
        and open_door_center is not None
        and open_door_center[1] > closed_door_center[1] + 0.15
        and open_door_center[0] < closed_door_center[0] - 0.10,
        details=f"closed={closed_door_center}, open={open_door_center}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(lint_flap, elem="flap_panel")
    with ctx.pose({flap_joint: 1.05}):
        open_flap_aabb = ctx.part_element_world_aabb(lint_flap, elem="flap_panel")
    closed_flap_center = _aabb_center(closed_flap_aabb)
    open_flap_center = _aabb_center(open_flap_aabb)
    ctx.check(
        "lint flap folds down and outward",
        closed_flap_center is not None
        and open_flap_center is not None
        and open_flap_center[1] > closed_flap_center[1] + 0.03
        and open_flap_center[2] < closed_flap_center[2] - 0.01,
        details=f"closed={closed_flap_center}, open={open_flap_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
