from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    superellipse_profile,
    tube_from_spline_points,
)


CASE_LENGTH = 0.58
CASE_DEPTH = 0.24
LOWER_HEIGHT = 0.097
LID_HEIGHT = 0.040
SHELL_WALL = 0.010
CORNER_RADIUS = 0.032
BASE_PANEL = 0.004
HINGE_AXIS_Z = 0.096


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_panel_mesh(width: float, depth: float, thickness: float, name: str):
    return _mesh(name, ExtrudeGeometry(rounded_rect_profile(width, depth, CORNER_RADIUS), thickness))


def _rounded_ring_mesh(
    width: float,
    depth: float,
    wall: float,
    height: float,
    name: str,
    *,
    radius: float | None = None,
):
    outer_radius = CORNER_RADIUS if radius is None else radius
    inner_radius = max(outer_radius - wall * 0.8, 0.004)
    outer = rounded_rect_profile(width, depth, outer_radius)
    inner = rounded_rect_profile(width - 2.0 * wall, depth - 2.0 * wall, inner_radius)
    return _mesh(name, ExtrudeWithHolesGeometry(outer, [inner], height))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trumpet_case")

    shell_black = model.material("shell_black", rgba=(0.12, 0.13, 0.14, 1.0))
    trim_aluminum = model.material("trim_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    plush = model.material("plush", rgba=(0.39, 0.06, 0.09, 1.0))
    handle_leather = model.material("handle_leather", rgba=(0.25, 0.15, 0.08, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.inertial = Inertial.from_geometry(
        Box((CASE_LENGTH, CASE_DEPTH, LOWER_HEIGHT)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, LOWER_HEIGHT * 0.5)),
    )

    lower_shell.visual(
        _rounded_panel_mesh(CASE_LENGTH, CASE_DEPTH, BASE_PANEL, "lower_floor_panel"),
        origin=Origin(xyz=(0.0, 0.0, BASE_PANEL * 0.5)),
        material=shell_black,
        name="outer_floor",
    )
    lower_wall_height = LOWER_HEIGHT - BASE_PANEL
    lower_shell.visual(
        Box((CASE_LENGTH - 0.064, SHELL_WALL, lower_wall_height)),
        origin=Origin(xyz=(0.0, CASE_DEPTH * 0.5 - SHELL_WALL * 0.5, BASE_PANEL + lower_wall_height * 0.5)),
        material=shell_black,
        name="front_wall",
    )
    lower_shell.visual(
        Box((SHELL_WALL, CASE_DEPTH - 0.040, lower_wall_height)),
        origin=Origin(
            xyz=(-CASE_LENGTH * 0.5 + SHELL_WALL * 0.5, 0.010, BASE_PANEL + lower_wall_height * 0.5)
        ),
        material=shell_black,
        name="left_wall",
    )
    lower_shell.visual(
        Box((SHELL_WALL, CASE_DEPTH - 0.040, lower_wall_height)),
        origin=Origin(
            xyz=(CASE_LENGTH * 0.5 - SHELL_WALL * 0.5, 0.010, BASE_PANEL + lower_wall_height * 0.5)
        ),
        material=shell_black,
        name="right_wall",
    )
    lower_shell.visual(
        Cylinder(radius=0.030, length=lower_wall_height),
        origin=Origin(xyz=(-CASE_LENGTH * 0.5 + 0.034, CASE_DEPTH * 0.5 - 0.034, BASE_PANEL + lower_wall_height * 0.5)),
        material=shell_black,
        name="front_left_corner",
    )
    lower_shell.visual(
        Cylinder(radius=0.030, length=lower_wall_height),
        origin=Origin(xyz=(CASE_LENGTH * 0.5 - 0.034, CASE_DEPTH * 0.5 - 0.034, BASE_PANEL + lower_wall_height * 0.5)),
        material=shell_black,
        name="front_right_corner",
    )
    lower_shell.visual(
        Box((0.088, SHELL_WALL, lower_wall_height - 0.020)),
        origin=Origin(
            xyz=(-0.190, -CASE_DEPTH * 0.5 + SHELL_WALL * 0.5, BASE_PANEL + (lower_wall_height - 0.020) * 0.5)
        ),
        material=shell_black,
        name="rear_left_shoulder",
    )
    lower_shell.visual(
        Box((0.088, SHELL_WALL, lower_wall_height - 0.020)),
        origin=Origin(
            xyz=(0.190, -CASE_DEPTH * 0.5 + SHELL_WALL * 0.5, BASE_PANEL + (lower_wall_height - 0.020) * 0.5)
        ),
        material=shell_black,
        name="rear_right_shoulder",
    )
    lower_shell.visual(
        Box((CASE_LENGTH - 0.074, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, CASE_DEPTH * 0.5 - 0.005, LOWER_HEIGHT - 0.009)),
        material=trim_aluminum,
        name="front_valance",
    )
    lower_shell.visual(
        Box((0.006, CASE_DEPTH - 0.050, 0.018)),
        origin=Origin(xyz=(-CASE_LENGTH * 0.5 + 0.004, 0.012, LOWER_HEIGHT - 0.009)),
        material=trim_aluminum,
        name="left_valance",
    )
    lower_shell.visual(
        Box((0.006, CASE_DEPTH - 0.050, 0.018)),
        origin=Origin(xyz=(CASE_LENGTH * 0.5 - 0.004, 0.012, LOWER_HEIGHT - 0.009)),
        material=trim_aluminum,
        name="right_valance",
    )
    lower_shell.visual(
        _rounded_panel_mesh(CASE_LENGTH - 0.028, CASE_DEPTH - 0.028, 0.010, "lower_plush_floor"),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=plush,
        name="cavity_floor",
    )

    bell_outer = superellipse_profile(0.160, 0.160, exponent=2.0, segments=40)
    bell_inner = superellipse_profile(0.112, 0.112, exponent=2.0, segments=36)
    lower_shell.visual(
        _mesh("bell_cradle_ring", ExtrudeWithHolesGeometry(bell_outer, [bell_inner], 0.030)),
        origin=Origin(xyz=(0.185, 0.000, 0.026)),
        material=plush,
        name="bell_cradle",
    )
    lower_shell.visual(
        Box((0.090, 0.070, 0.032)),
        origin=Origin(xyz=(0.035, -0.020, 0.026)),
        material=plush,
        name="valve_block",
    )
    lower_shell.visual(
        Box((0.205, 0.040, 0.028)),
        origin=Origin(xyz=(-0.110, 0.055, 0.024), rpy=(0.0, 0.0, 0.48)),
        material=plush,
        name="leadpipe_rail",
    )
    lower_shell.visual(
        Box((0.145, 0.035, 0.024)),
        origin=Origin(xyz=(-0.125, -0.052, 0.022), rpy=(0.0, 0.0, -0.22)),
        material=plush,
        name="tuning_slide_rail",
    )
    lower_shell.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(-0.225, 0.075, 0.020), rpy=(0.0, pi / 2.0, 0.0)),
        material=plush,
        name="mouthpiece_pocket",
    )

    lower_shell.visual(
        Box((CASE_LENGTH - 0.040, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -CASE_DEPTH * 0.5 + 0.006, HINGE_AXIS_Z - 0.016)),
        material=trim_aluminum,
        name="rear_hinge_plate",
    )
    for name, x_pos, width in (
        ("handle_mount_left", -0.050, 0.034),
        ("handle_mount_right", 0.050, 0.034),
        ("latch_mount_left", -0.135, 0.028),
        ("latch_mount_right", 0.135, 0.028),
    ):
        lower_shell.visual(
            Box((width, 0.010, 0.020)),
            origin=Origin(xyz=(x_pos, CASE_DEPTH * 0.5 - 0.005, 0.040 if "handle" in name else 0.076)),
            material=hardware_steel,
            name=name,
        )
    for foot_name, x_pos, y_pos in (
        ("foot_fl", -0.18, 0.07),
        ("foot_fr", 0.18, 0.07),
        ("foot_rl", -0.18, -0.07),
        ("foot_rr", 0.18, -0.07),
    ):
        lower_shell.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(x_pos, y_pos, -0.004)),
            material=rubber,
            name=foot_name,
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((CASE_LENGTH, CASE_DEPTH, LID_HEIGHT + 0.020)),
        mass=1.3,
        origin=Origin(xyz=(0.0, CASE_DEPTH * 0.5, 0.020)),
    )
    lid.visual(
        _rounded_panel_mesh(CASE_LENGTH, CASE_DEPTH, 0.004, "lid_top_panel"),
        origin=Origin(xyz=(0.0, CASE_DEPTH * 0.5, 0.036)),
        material=shell_black,
        name="top_panel",
    )
    lid.visual(
        Cylinder(radius=0.010, length=CASE_LENGTH - 0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="hinge_barrel",
    )
    lid.visual(
        Box((CASE_LENGTH - 0.100, 0.014, 0.036)),
        origin=Origin(xyz=(0.0, 0.007, 0.018)),
        material=shell_black,
        name="rear_rail",
    )
    lid.visual(
        Box((CASE_LENGTH - 0.064, SHELL_WALL, LID_HEIGHT)),
        origin=Origin(xyz=(0.0, CASE_DEPTH - SHELL_WALL * 0.5, LID_HEIGHT * 0.5)),
        material=shell_black,
        name="front_wall",
    )
    lid.visual(
        Box((SHELL_WALL, CASE_DEPTH, LID_HEIGHT)),
        origin=Origin(xyz=(-CASE_LENGTH * 0.5 + SHELL_WALL * 0.5, CASE_DEPTH * 0.5, LID_HEIGHT * 0.5)),
        material=shell_black,
        name="left_wall",
    )
    lid.visual(
        Box((SHELL_WALL, CASE_DEPTH, LID_HEIGHT)),
        origin=Origin(xyz=(CASE_LENGTH * 0.5 - SHELL_WALL * 0.5, CASE_DEPTH * 0.5, LID_HEIGHT * 0.5)),
        material=shell_black,
        name="right_wall",
    )
    lid.visual(
        Cylinder(radius=0.030, length=LID_HEIGHT),
        origin=Origin(xyz=(-CASE_LENGTH * 0.5 + 0.034, CASE_DEPTH - 0.034, LID_HEIGHT * 0.5)),
        material=shell_black,
        name="front_left_corner",
    )
    lid.visual(
        Cylinder(radius=0.030, length=LID_HEIGHT),
        origin=Origin(xyz=(CASE_LENGTH * 0.5 - 0.034, CASE_DEPTH - 0.034, LID_HEIGHT * 0.5)),
        material=shell_black,
        name="front_right_corner",
    )
    lid.visual(
        Box((CASE_LENGTH - 0.074, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, CASE_DEPTH - 0.003, 0.030)),
        material=trim_aluminum,
        name="front_valance",
    )
    lid.visual(
        Box((0.006, CASE_DEPTH - 0.020, 0.014)),
        origin=Origin(xyz=(-CASE_LENGTH * 0.5 + 0.004, CASE_DEPTH * 0.5, 0.030)),
        material=trim_aluminum,
        name="left_valance",
    )
    lid.visual(
        Box((0.006, CASE_DEPTH - 0.020, 0.014)),
        origin=Origin(xyz=(CASE_LENGTH * 0.5 - 0.004, CASE_DEPTH * 0.5, 0.030)),
        material=trim_aluminum,
        name="right_valance",
    )
    lid.visual(
        Box((CASE_LENGTH - 0.090, CASE_DEPTH - 0.070, 0.018)),
        origin=Origin(xyz=(0.0, CASE_DEPTH * 0.53, 0.027)),
        material=plush,
        name="lid_pad",
    )
    for name, x_pos in (("left_catch", -0.135), ("right_catch", 0.135)):
        lid.visual(
            Box((0.018, 0.010, 0.012)),
            origin=Origin(xyz=(x_pos, CASE_DEPTH - 0.002, 0.020)),
            material=hardware_steel,
            name=name,
        )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.140, 0.055, 0.050)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.024, 0.016)),
    )
    handle.visual(
        Box((0.024, 0.012, 0.028)),
        origin=Origin(xyz=(-0.050, 0.006, 0.000)),
        material=hardware_steel,
        name="left_bracket",
    )
    handle.visual(
        Box((0.024, 0.012, 0.028)),
        origin=Origin(xyz=(0.050, 0.006, 0.000)),
        material=hardware_steel,
        name="right_bracket",
    )
    handle.visual(
        _mesh(
            "carry_handle_grip",
            tube_from_spline_points(
                [
                    (-0.058, 0.012, 0.014),
                    (-0.040, 0.028, 0.026),
                    (0.000, 0.038, 0.032),
                    (0.040, 0.028, 0.026),
                    (0.058, 0.012, 0.014),
                ],
                radius=0.010,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=handle_leather,
        name="grip",
    )

    def add_latch(name: str) -> None:
        latch = model.part(name)
        latch.inertial = Inertial.from_geometry(
            Box((0.022, 0.028, 0.064)),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.002, 0.028)),
        )
        latch.visual(
            Cylinder(radius=0.007, length=0.022),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware_steel,
            name="pivot_barrel",
        )
        latch.visual(
            Box((0.018, 0.012, 0.018)),
            origin=Origin(xyz=(0.0, 0.002, 0.012)),
            material=hardware_steel,
            name="toggle_body",
        )
        latch.visual(
            Box((0.016, 0.010, 0.046)),
            origin=Origin(xyz=(0.0, 0.001, 0.029)),
            material=hardware_steel,
            name="lever",
        )
        latch.visual(
            Box((0.016, 0.012, 0.008)),
            origin=Origin(xyz=(0.0, 0.006, 0.047)),
            material=hardware_steel,
            name="hook_tip",
        )

    add_latch("left_latch")
    add_latch("right_latch")

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, -CASE_DEPTH * 0.5, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "handle_mount",
        ArticulationType.FIXED,
        parent=lower_shell,
        child=handle,
        origin=Origin(xyz=(0.0, CASE_DEPTH * 0.5, 0.040)),
    )
    model.articulation(
        "left_latch_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child="left_latch",
        origin=Origin(xyz=(-0.135, CASE_DEPTH * 0.5 + 0.007, 0.076)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=-1.45, upper=0.05),
    )
    model.articulation(
        "right_latch_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child="right_latch",
        origin=Origin(xyz=(0.135, CASE_DEPTH * 0.5 + 0.007, 0.076)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=-1.45, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    rear_hinge = object_model.get_articulation("rear_hinge")
    left_latch_hinge = object_model.get_articulation("left_latch_hinge")
    right_latch_hinge = object_model.get_articulation("right_latch_hinge")

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

    ctx.check("rear_hinge_axis_x", rear_hinge.axis == (1.0, 0.0, 0.0), details=str(rear_hinge.axis))
    ctx.check(
        "left_latch_axis_x",
        left_latch_hinge.axis == (1.0, 0.0, 0.0),
        details=str(left_latch_hinge.axis),
    )
    ctx.check(
        "right_latch_axis_x",
        right_latch_hinge.axis == (1.0, 0.0, 0.0),
        details=str(right_latch_hinge.axis),
    )

    ctx.expect_contact(handle, lower_shell, elem_a="left_bracket", elem_b="handle_mount_left")
    ctx.expect_contact(handle, lower_shell, elem_a="right_bracket", elem_b="handle_mount_right")
    ctx.expect_contact(left_latch, lower_shell, elem_a="pivot_barrel", elem_b="latch_mount_left")
    ctx.expect_contact(right_latch, lower_shell, elem_a="pivot_barrel", elem_b="latch_mount_right")
    ctx.expect_contact(lid, lower_shell, elem_a="hinge_barrel", elem_b="rear_hinge_plate")
    ctx.expect_overlap(lid, lower_shell, axes="xy", min_overlap=0.20)

    left_rest = ctx.part_element_world_aabb(left_latch, elem="hook_tip")
    right_rest = ctx.part_element_world_aabb(right_latch, elem="hook_tip")
    front_trim_rest = ctx.part_element_world_aabb(lid, elem="front_valance")
    lid_rest_pos = ctx.part_world_position(lid)
    assert left_rest is not None
    assert right_rest is not None
    assert front_trim_rest is not None
    assert lid_rest_pos is not None

    with ctx.pose({rear_hinge: 1.30}):
        front_trim_open = ctx.part_element_world_aabb(lid, elem="front_valance")
        lid_open_pos = ctx.part_world_position(lid)
        assert front_trim_open is not None
        assert lid_open_pos is not None
        ctx.check(
            "lid_front_rises_when_open",
            front_trim_open[1][2] > front_trim_rest[1][2] + 0.18,
            details=f"rest={front_trim_rest} open={front_trim_open}",
        )
        ctx.check(
            "lid_hinge_line_stays_put",
            abs(lid_open_pos[0] - lid_rest_pos[0]) < 1e-6
            and abs(lid_open_pos[1] - lid_rest_pos[1]) < 1e-6
            and abs(lid_open_pos[2] - lid_rest_pos[2]) < 1e-6,
            details=f"rest={lid_rest_pos} open={lid_open_pos}",
        )

    with ctx.pose({left_latch_hinge: -1.10, right_latch_hinge: -1.10}):
        left_open = ctx.part_element_world_aabb(left_latch, elem="hook_tip")
        right_open = ctx.part_element_world_aabb(right_latch, elem="hook_tip")
        assert left_open is not None
        assert right_open is not None
        ctx.check(
            "left_latch_swings_down",
            left_open[1][1] > left_rest[1][1] + 0.025 and left_open[1][2] < left_rest[1][2] - 0.012,
            details=f"rest={left_rest} open={left_open}",
        )
        ctx.check(
            "right_latch_swings_down",
            right_open[1][1] > right_rest[1][1] + 0.025 and right_open[1][2] < right_rest[1][2] - 0.012,
            details=f"rest={right_rest} open={right_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
