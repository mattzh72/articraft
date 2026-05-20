from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sin, cos, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 24,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos((tau * i) / segments),
            cy + radius * sin((tau * i) / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_vehicle_glove_compartment")

    dash_mat = model.material("dash_plastic", rgba=(0.16, 0.18, 0.20, 1.0))
    door_mat = model.material("door_plastic", rgba=(0.20, 0.22, 0.24, 1.0))
    bin_mat = model.material("bin_liner", rgba=(0.08, 0.09, 0.10, 1.0))
    hinge_mat = model.material("hinge_metal", rgba=(0.52, 0.55, 0.58, 1.0))
    latch_mat = model.material("latch_metal", rgba=(0.66, 0.67, 0.69, 1.0))
    dark_mat = model.material("dark_insert", rgba=(0.05, 0.05, 0.06, 1.0))

    outer_w = 0.400
    outer_h = 0.190
    opening_w = 0.320
    opening_h = 0.130
    frame_t = 0.016
    wall_t = 0.012
    cavity_depth = 0.230

    side_border = (outer_w - opening_w) / 2.0
    top_border = (outer_h - opening_h) / 2.0

    hinge_axis_x = -0.183
    hinge_axis_y = 0.014

    door_panel_w = 0.340
    door_panel_h = 0.136
    door_t = 0.012
    door_shell_x = 0.010
    door_shell_center_x = door_shell_x + door_panel_w / 2.0

    lock_axis_x = 0.305
    lock_hole_r = 0.0095
    hinge_knuckle_r = 0.007
    root_knuckle_len = 0.040
    middle_knuckle_len = 0.044

    dash_bin = model.part("dash_bin")

    dash_bin.visual(
        Box((outer_w, frame_t, top_border)),
        origin=Origin(xyz=(0.0, 0.0, opening_h / 2.0 + top_border / 2.0)),
        material=dash_mat,
        name="frame_top",
    )
    dash_bin.visual(
        Box((outer_w, frame_t, top_border)),
        origin=Origin(xyz=(0.0, 0.0, -(opening_h / 2.0 + top_border / 2.0))),
        material=dash_mat,
        name="frame_bottom",
    )
    dash_bin.visual(
        Box((side_border, frame_t, outer_h)),
        origin=Origin(xyz=(-(opening_w / 2.0 + side_border / 2.0), 0.0, 0.0)),
        material=dash_mat,
        name="frame_left",
    )
    dash_bin.visual(
        Box((side_border, frame_t, outer_h)),
        origin=Origin(xyz=(opening_w / 2.0 + side_border / 2.0, 0.0, 0.0)),
        material=dash_mat,
        name="frame_right",
    )

    wall_center_y = -(frame_t / 2.0 + cavity_depth / 2.0)
    dash_bin.visual(
        Box((wall_t, cavity_depth, opening_h)),
        origin=Origin(xyz=(-(opening_w / 2.0 - wall_t / 2.0), wall_center_y, 0.0)),
        material=bin_mat,
        name="bin_left_wall",
    )
    dash_bin.visual(
        Box((wall_t, cavity_depth, opening_h)),
        origin=Origin(xyz=(opening_w / 2.0 - wall_t / 2.0, wall_center_y, 0.0)),
        material=bin_mat,
        name="bin_right_wall",
    )
    dash_bin.visual(
        Box((opening_w, cavity_depth, wall_t)),
        origin=Origin(xyz=(0.0, wall_center_y, opening_h / 2.0 + wall_t / 2.0)),
        material=bin_mat,
        name="bin_top_wall",
    )
    dash_bin.visual(
        Box((opening_w, cavity_depth, wall_t)),
        origin=Origin(xyz=(0.0, wall_center_y, -(opening_h / 2.0 + wall_t / 2.0))),
        material=bin_mat,
        name="bin_bottom_wall",
    )
    dash_bin.visual(
        Box((opening_w, wall_t, opening_h)),
        origin=Origin(
            xyz=(
                0.0,
                -(frame_t / 2.0 + cavity_depth + wall_t / 2.0),
                0.0,
            )
        ),
        material=bin_mat,
        name="bin_back",
    )

    dash_bin.visual(
        Box((0.018, frame_t, door_panel_h)),
        origin=Origin(xyz=(hinge_axis_x, 0.0, 0.0)),
        material=dash_mat,
        name="hinge_mount",
    )
    dash_bin.visual(
        Cylinder(radius=hinge_knuckle_r, length=root_knuckle_len),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.047)),
        material=hinge_mat,
        name="upper_hinge_knuckle",
    )
    dash_bin.visual(
        Cylinder(radius=hinge_knuckle_r, length=root_knuckle_len),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, -0.047)),
        material=hinge_mat,
        name="lower_hinge_knuckle",
    )
    dash_bin.visual(
        Box((0.008, 0.020, 0.034)),
        origin=Origin(xyz=(opening_w / 2.0 - 0.004, -0.018, 0.0)),
        material=hinge_mat,
        name="latch_strike",
    )

    door = model.part("door")
    door_profile = rounded_rect_profile(
        door_panel_w,
        door_panel_h,
        radius=0.008,
        corner_segments=8,
    )
    lock_profile_center_x = lock_axis_x - door_shell_center_x
    door_hole = _circle_profile(lock_hole_r, center=(lock_profile_center_x, 0.0), segments=28)
    door_shell = ExtrudeWithHolesGeometry(
        door_profile,
        [door_hole],
        door_t,
        cap=True,
        center=True,
        closed=True,
    )
    door.visual(
        mesh_from_geometry(door_shell, "glove_box_door_shell"),
        origin=Origin(xyz=(door_shell_center_x, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=door_mat,
        name="outer_panel",
    )
    door.visual(
        Box((0.028, 0.004, 0.100)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=hinge_mat,
        name="hinge_leaf",
    )
    door.visual(
        Cylinder(radius=hinge_knuckle_r, length=middle_knuckle_len),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_mat,
        name="middle_hinge_knuckle",
    )

    lock = model.part("lock_cylinder")
    lock.visual(
        Cylinder(radius=0.008, length=door_t + 0.010),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=latch_mat,
        name="lock_barrel",
    )
    lock.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, door_t / 2.0 + 0.002, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=latch_mat,
        name="outer_escutcheon",
    )
    lock.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, -(door_t / 2.0 + 0.003), 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=latch_mat,
        name="inner_hub",
    )
    lock.visual(
        Box((0.010, 0.0015, 0.002)),
        origin=Origin(xyz=(0.0, door_t / 2.0 + 0.0035, 0.0)),
        material=dark_mat,
        name="key_slot",
    )
    lock.visual(
        Box((0.006, 0.004, 0.050)),
        origin=Origin(xyz=(0.009, -(door_t / 2.0 + 0.005), 0.0)),
        material=latch_mat,
        name="cam_arm",
    )

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=dash_bin,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=1.50,
        ),
    )
    model.articulation(
        "key_lock",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lock,
        origin=Origin(xyz=(lock_axis_x, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-1.20,
            upper=1.20,
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

    dash_bin = object_model.get_part("dash_bin")
    door = object_model.get_part("door")
    lock = object_model.get_part("lock_cylinder")
    left_hinge = object_model.get_articulation("left_hinge")
    key_lock = object_model.get_articulation("key_lock")

    ctx.expect_contact(
        door,
        dash_bin,
        elem_a="outer_panel",
        elem_b="frame_top",
        contact_tol=0.0005,
        name="door skin meets the dash frame when closed",
    )
    ctx.expect_overlap(
        door,
        dash_bin,
        axes="xz",
        elem_a="outer_panel",
        min_overlap=0.13,
        name="door spans the glove compartment opening footprint",
    )
    ctx.expect_contact(
        lock,
        door,
        elem_a="outer_escutcheon",
        elem_b="outer_panel",
        contact_tol=0.0005,
        name="lock escutcheon seats against the door skin",
    )

    closed_panel = ctx.part_element_world_aabb(door, elem="outer_panel")
    with ctx.pose({left_hinge: 1.20}):
        opened_panel = ctx.part_element_world_aabb(door, elem="outer_panel")
    door_swings_out = (
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[1][1] > closed_panel[1][1] + 0.12
    )
    ctx.check(
        "left-hinged door swings outward on a vertical axis",
        door_swings_out,
        details=f"closed_panel={closed_panel}, opened_panel={opened_panel}",
    )

    rest_cam = ctx.part_element_world_aabb(lock, elem="cam_arm")
    with ctx.pose({key_lock: 1.00}):
        turned_cam = ctx.part_element_world_aabb(lock, elem="cam_arm")

    def _span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    cam_rotates = False
    if rest_cam is not None and turned_cam is not None:
        rest_span = _span(rest_cam)
        turned_span = _span(turned_cam)
        cam_rotates = turned_span[0] > rest_span[0] + 0.010 and turned_span[2] < rest_span[2] - 0.010
    else:
        rest_span = None
        turned_span = None
    ctx.check(
        "internal cam rotates with the key cylinder near the latch edge",
        cam_rotates,
        details=f"rest_cam={rest_cam}, turned_cam={turned_cam}, rest_span={rest_span}, turned_span={turned_span}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
