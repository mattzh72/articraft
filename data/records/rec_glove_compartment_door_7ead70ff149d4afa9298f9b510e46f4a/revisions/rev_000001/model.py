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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    center_x: float,
    center_y: float,
    radius: float,
    *,
    segments: int = 28,
) -> list[tuple[float, float]]:
    return [
        (
            center_x + radius * math.cos(index * math.tau / segments),
            center_y + radius * math.sin(index * math.tau / segments),
        )
        for index in range(segments)
    ]


def _rounded_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
    holes: list[list[tuple[float, float]]] | None = None,
    name: str,
):
    profile = rounded_rect_profile(width, height, corner_radius)
    if holes:
        geom = ExtrudeWithHolesGeometry(profile, holes, thickness)
    else:
        geom = ExtrudeGeometry(profile, thickness)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _hinge_clip_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    gap_angle: float,
    name: str,
):
    start = math.pi + gap_angle * 0.5
    end = math.pi + (math.tau - gap_angle * 0.5)
    outer_points: list[tuple[float, float]] = []
    inner_points: list[tuple[float, float]] = []
    segments = 26
    for index in range(segments + 1):
        t = index / segments
        angle = start + (end - start) * t
        outer_points.append((outer_radius * math.cos(angle), outer_radius * math.sin(angle)))
        inner_points.append((inner_radius * math.cos(angle), inner_radius * math.sin(angle)))
    profile = outer_points + list(reversed(inner_points))
    geom = ExtrudeGeometry(profile, height)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lockable_glove_compartment")

    dashboard_shell = model.material("dashboard_shell", rgba=(0.20, 0.21, 0.23, 1.0))
    dashboard_interior = model.material("dashboard_interior", rgba=(0.08, 0.09, 0.10, 1.0))
    door_trim = model.material("door_trim", rgba=(0.24, 0.25, 0.27, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_lock = model.material("dark_lock", rgba=(0.18, 0.18, 0.19, 1.0))

    opening_w = 0.268
    opening_h = 0.142
    surround_w = 0.378
    surround_h = 0.222
    bezel_t = 0.012
    wall_t = 0.004
    bin_depth = 0.182

    door_w = 0.275
    door_h = 0.148
    door_t = 0.010
    door_center_y = -0.005
    hinge_offset = 0.010
    hinge_axis_x = -(door_w * 0.5 + hinge_offset)
    hinge_y = 0.022
    upper_hinge_z = 0.045
    lower_hinge_z = -0.045
    hinge_pin_r = 0.0035
    hinge_pin_len = 0.028

    lock_x_local = 0.245
    lock_hole_r = 0.0087

    housing = model.part("dashboard_bin")

    fascia_mesh = _rounded_panel_mesh(
        width=surround_w,
        height=surround_h,
        thickness=bezel_t,
        corner_radius=0.020,
        holes=[rounded_rect_profile(opening_w, opening_h, 0.008)],
        name="glovebox_fascia",
    )
    housing.visual(
        fascia_mesh,
        origin=Origin(xyz=(0.0, bezel_t * 0.5, 0.0)),
        material=dashboard_shell,
        name="fascia",
    )
    housing.visual(
        Box((wall_t, bin_depth, opening_h)),
        origin=Origin(xyz=(-opening_w * 0.5 - wall_t * 0.5, -bin_depth * 0.5, 0.0)),
        material=dashboard_interior,
        name="left_wall",
    )
    housing.visual(
        Box((wall_t, bin_depth, opening_h)),
        origin=Origin(xyz=(opening_w * 0.5 + wall_t * 0.5, -bin_depth * 0.5, 0.0)),
        material=dashboard_interior,
        name="right_wall",
    )
    housing.visual(
        Box((opening_w, bin_depth, wall_t)),
        origin=Origin(xyz=(0.0, -bin_depth * 0.5, opening_h * 0.5 + wall_t * 0.5)),
        material=dashboard_interior,
        name="top_wall",
    )
    housing.visual(
        Box((opening_w, bin_depth, wall_t)),
        origin=Origin(xyz=(0.0, -bin_depth * 0.5, -opening_h * 0.5 - wall_t * 0.5)),
        material=dashboard_interior,
        name="bottom_wall",
    )
    housing.visual(
        Box((opening_w, wall_t, opening_h + 2.0 * wall_t)),
        origin=Origin(xyz=(0.0, -bin_depth + wall_t * 0.5, 0.0)),
        material=dashboard_interior,
        name="back_wall",
    )
    housing.visual(
        Box((0.004, 0.028, 0.120)),
        origin=Origin(xyz=(hinge_axis_x + 0.0115, 0.014, 0.0)),
        material=dashboard_shell,
        name="hinge_backstrap",
    )
    housing.visual(
        Box((0.013, 0.008, 0.004)),
        origin=Origin(xyz=(hinge_axis_x + 0.0055, 0.024, upper_hinge_z + 0.016)),
        material=dashboard_shell,
        name="upper_hinge_bracket",
    )
    housing.visual(
        Box((0.013, 0.008, 0.004)),
        origin=Origin(xyz=(hinge_axis_x + 0.0055, 0.024, lower_hinge_z - 0.016)),
        material=dashboard_shell,
        name="lower_hinge_bracket",
    )
    housing.visual(
        Cylinder(radius=hinge_pin_r, length=hinge_pin_len),
        origin=Origin(xyz=(hinge_axis_x, hinge_y, upper_hinge_z)),
        material=satin_metal,
        name="upper_hinge_pin",
    )
    housing.visual(
        Cylinder(radius=hinge_pin_r, length=hinge_pin_len),
        origin=Origin(xyz=(hinge_axis_x, hinge_y, lower_hinge_z)),
        material=satin_metal,
        name="lower_hinge_pin",
    )
    housing.visual(
        Box((0.006, 0.020, 0.040)),
        origin=Origin(xyz=(opening_w * 0.5, -0.010, 0.0)),
        material=satin_metal,
        name="strike_plate",
    )
    housing.inertial = Inertial.from_geometry(
        Box((surround_w, bin_depth + bezel_t, surround_h)),
        mass=3.8,
        origin=Origin(xyz=(0.0, -0.080, 0.0)),
    )

    door = model.part("door")
    door_outer_mesh = _rounded_panel_mesh(
        width=door_w,
        height=door_h,
        thickness=door_t,
        corner_radius=0.010,
        holes=[
            _circle_profile(
                lock_x_local - (hinge_offset + door_w * 0.5),
                0.0,
                lock_hole_r,
                segments=30,
            )
        ],
        name="glovebox_door_outer",
    )
    clip_mesh = _hinge_clip_mesh(
        outer_radius=0.0086,
        inner_radius=0.0052,
        height=0.024,
        gap_angle=math.radians(82.0),
        name="glovebox_hinge_clip",
    )

    door.visual(
        door_outer_mesh,
        origin=Origin(xyz=(hinge_offset + door_w * 0.5, door_center_y, 0.0)),
        material=door_trim,
        name="outer_skin",
    )
    door.visual(
        clip_mesh,
        origin=Origin(xyz=(0.0, 0.0, upper_hinge_z)),
        material=door_trim,
        name="upper_hinge_clip",
    )
    door.visual(
        clip_mesh,
        origin=Origin(xyz=(0.0, 0.0, lower_hinge_z)),
        material=door_trim,
        name="lower_hinge_clip",
    )
    door.visual(
        Box((0.014, 0.006, 0.030)),
        origin=Origin(xyz=(0.007, -0.004, upper_hinge_z)),
        material=door_trim,
        name="upper_clip_connector",
    )
    door.visual(
        Box((0.014, 0.006, 0.030)),
        origin=Origin(xyz=(0.007, -0.004, lower_hinge_z)),
        material=door_trim,
        name="lower_clip_connector",
    )
    door.visual(
        Box((0.208, 0.012, 0.014)),
        origin=Origin(xyz=(0.129, -0.015, 0.056)),
        material=door_trim,
        name="top_inner_rib",
    )
    door.visual(
        Box((0.208, 0.012, 0.014)),
        origin=Origin(xyz=(0.129, -0.015, -0.056)),
        material=door_trim,
        name="bottom_inner_rib",
    )
    door.visual(
        Box((0.014, 0.012, 0.104)),
        origin=Origin(xyz=(0.262, -0.015, 0.0)),
        material=door_trim,
        name="latch_edge_rib",
    )
    door.visual(
        Box((0.190, 0.004, 0.098)),
        origin=Origin(xyz=(0.132, -0.021, 0.0)),
        material=door_trim,
        name="inner_liner",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, 0.028, door_h)),
        mass=1.1,
        origin=Origin(xyz=(hinge_offset + door_w * 0.5, -0.010, 0.0)),
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    lock = model.part("lock_cylinder")
    lock.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="escutcheon",
    )
    lock.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="barrel_body",
    )
    lock.visual(
        Cylinder(radius=0.0105, length=0.004),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_lock,
        name="rear_nut",
    )
    lock.visual(
        Box((0.004, 0.0012, 0.014)),
        origin=Origin(xyz=(0.0, 0.0046, 0.0)),
        material=dark_lock,
        name="key_slot",
    )
    lock.visual(
        Box((0.030, 0.004, 0.012)),
        origin=Origin(xyz=(0.015, -0.021, 0.0)),
        material=dark_lock,
        name="cam_arm",
    )
    lock.visual(
        Box((0.008, 0.004, 0.018)),
        origin=Origin(xyz=(0.030, -0.021, 0.003)),
        material=dark_lock,
        name="cam_tip",
    )
    lock.inertial = Inertial.from_geometry(
        Box((0.040, 0.030, 0.020)),
        mass=0.18,
        origin=Origin(xyz=(0.014, -0.010, 0.0)),
    )

    model.articulation(
        "door_to_lock",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lock,
        origin=Origin(xyz=(lock_x_local, door_center_y + door_t * 0.5, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    housing = object_model.get_part("dashboard_bin")
    door = object_model.get_part("door")
    lock = object_model.get_part("lock_cylinder")
    door_hinge = object_model.get_articulation("housing_to_door")
    lock_joint = object_model.get_articulation("door_to_lock")

    ctx.check(
        "door_hinge_axis_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical hinge axis, got {door_hinge.axis}",
    )
    ctx.check(
        "lock_axis_normal_to_door",
        tuple(lock_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected lock cylinder axis along local y, got {lock_joint.axis}",
    )

    ctx.expect_contact(door, housing, elem_a="outer_skin", elem_b="fascia")
    ctx.expect_contact(lock, door)
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        elem_a="outer_skin",
        elem_b="fascia",
        min_overlap=0.14,
    )

    closed_cam = ctx.part_element_world_aabb(lock, elem="cam_arm")
    closed_door = ctx.part_element_world_aabb(door, elem="outer_skin")
    if closed_cam is not None and closed_door is not None:
        closed_cam_max_x = closed_cam[1][0]
        closed_door_max_x = closed_door[1][0]
        ctx.check(
            "cam_stays_near_latching_edge",
            closed_cam_max_x > closed_door_max_x - 0.055,
            details=(
                f"cam should sit near the free edge: cam_max_x={closed_cam_max_x:.4f}, "
                f"door_max_x={closed_door_max_x:.4f}"
            ),
        )

    with ctx.pose({door_hinge: math.radians(72.0)}):
        ctx.expect_overlap(
            door,
            housing,
            axes="yz",
            elem_a="upper_hinge_clip",
            elem_b="upper_hinge_pin",
            min_overlap=0.006,
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="yz",
            elem_a="lower_hinge_clip",
            elem_b="lower_hinge_pin",
            min_overlap=0.006,
        )
        open_door = ctx.part_element_world_aabb(door, elem="outer_skin")
        if closed_door is not None and open_door is not None:
            closed_center_y = 0.5 * (closed_door[0][1] + closed_door[1][1])
            open_center_y = 0.5 * (open_door[0][1] + open_door[1][1])
            ctx.check(
                "door_swings_outward",
                open_center_y > closed_center_y + 0.10,
                details=(
                    f"door should swing out from the bin: "
                    f"closed_center_y={closed_center_y:.4f}, open_center_y={open_center_y:.4f}"
                ),
            )

    rest_cam = ctx.part_element_world_aabb(lock, elem="cam_arm")
    with ctx.pose({lock_joint: math.radians(90.0)}):
        turned_cam = ctx.part_element_world_aabb(lock, elem="cam_arm")
        ctx.expect_contact(lock, door)
        if rest_cam is not None and turned_cam is not None:
            rest_center = tuple(0.5 * (rest_cam[0][i] + rest_cam[1][i]) for i in range(3))
            turned_center = tuple(0.5 * (turned_cam[0][i] + turned_cam[1][i]) for i in range(3))
            ctx.check(
                "cam_rotates_with_lock",
                abs(turned_center[0] - rest_center[0]) > 0.010
                and abs(turned_center[2] - rest_center[2]) > 0.010,
                details=(
                    f"cam center should move in x and z when the lock turns: "
                    f"rest={rest_center}, turned={turned_center}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
