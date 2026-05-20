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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_front_fascia_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    y_center: float,
    z_center: float,
    door_opening_width: float,
    door_opening_height: float,
    door_opening_center_x: float,
    door_opening_center_z: float,
    knob_centers: list[tuple[float, float]],
    knob_hole_diameter: float,
):
    outer = rounded_rect_profile(width, height, radius=0.016, corner_segments=8)
    holes: list[list[tuple[float, float]]] = [
        _translate_profile(
            rounded_rect_profile(
                door_opening_width,
                door_opening_height,
                radius=0.010,
                corner_segments=8,
            ),
            door_opening_center_x,
            door_opening_center_z - z_center,
        )
    ]
    for knob_x, knob_z in knob_centers:
        holes.append(
            _translate_profile(
                superellipse_profile(
                    knob_hole_diameter,
                    knob_hole_diameter,
                    exponent=2.0,
                    segments=28,
                ),
                knob_x,
                knob_z - z_center,
            )
        )

    fascia = ExtrudeWithHolesGeometry(
        outer,
        holes,
        height=thickness,
        center=True,
        cap=True,
        closed=True,
    )
    fascia.rotate_x(math.pi / 2.0)
    fascia.translate(0.0, y_center, z_center)
    return mesh_from_geometry(fascia, "toaster_oven_front_fascia")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_toaster_oven")

    enamel = model.material("enamel", rgba=(0.18, 0.19, 0.20, 1.0))
    trim = model.material("trim", rgba=(0.42, 0.43, 0.45, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.23, 0.28, 0.32))
    knob_dark = model.material("knob_dark", rgba=(0.11, 0.11, 0.12, 1.0))
    metal = model.material("metal", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    body_width = 0.43
    body_depth = 0.33
    body_height = 0.27
    foot_height = 0.012
    shell_height = body_height - foot_height
    wall = 0.008
    fascia_thickness = 0.006
    door_center_x = -0.065
    door_opening_width = 0.286
    door_opening_height = 0.156
    door_opening_center_z = 0.133
    door_width = 0.312
    door_height = 0.195
    door_depth = 0.022
    door_hinge_z = 0.038
    hinge_y = body_depth * 0.5 + 0.003
    fascia_center_y = body_depth * 0.5 - fascia_thickness * 0.5
    shell_depth = body_depth - fascia_thickness
    knob_positions = [(0.148, 0.220), (0.148, 0.156), (0.148, 0.092)]
    knob_hole_diameter = 0.013

    body = model.part("body")
    body.visual(
        _build_front_fascia_mesh(
            width=body_width,
            height=shell_height,
            thickness=fascia_thickness,
            y_center=fascia_center_y,
            z_center=foot_height + shell_height * 0.5,
            door_opening_width=door_opening_width,
            door_opening_height=door_opening_height,
            door_opening_center_x=door_center_x,
            door_opening_center_z=door_opening_center_z,
            knob_centers=knob_positions,
            knob_hole_diameter=knob_hole_diameter,
        ),
        material=enamel,
        name="front_fascia",
    )
    body.visual(
        Box((wall, shell_depth, shell_height)),
        origin=Origin(
            xyz=(-body_width * 0.5 + wall * 0.5, -fascia_thickness * 0.5, foot_height + shell_height * 0.5)
        ),
        material=enamel,
        name="left_wall",
    )
    body.visual(
        Box((wall, shell_depth, shell_height)),
        origin=Origin(
            xyz=(body_width * 0.5 - wall * 0.5, -fascia_thickness * 0.5, foot_height + shell_height * 0.5)
        ),
        material=enamel,
        name="right_wall",
    )
    body.visual(
        Box((body_width, shell_depth, wall)),
        origin=Origin(xyz=(0.0, -fascia_thickness * 0.5, foot_height + wall * 0.5)),
        material=enamel,
        name="bottom_shell",
    )
    body.visual(
        Box((body_width, shell_depth, wall)),
        origin=Origin(xyz=(0.0, -fascia_thickness * 0.5, body_height - wall * 0.5)),
        material=enamel,
        name="top_shell",
    )
    body.visual(
        Box((body_width, wall, shell_height)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 + wall * 0.5, foot_height + shell_height * 0.5)),
        material=enamel,
        name="back_shell",
    )
    body.visual(
        Box((door_width + 0.040, 0.010, 0.016)),
        origin=Origin(xyz=(door_center_x, body_depth * 0.5 - 0.016, door_hinge_z + 0.004)),
        material=trim,
        name="lower_hinge_beam",
    )
    body.visual(
        Box((0.022, 0.008, 0.188)),
        origin=Origin(xyz=(0.192, body_depth * 0.5 - 0.010, 0.154)),
        material=trim,
        name="control_side_trim",
    )
    for name, hinge_x in (
        ("body_hinge_left", door_center_x - 0.115),
        ("body_hinge_right", door_center_x + 0.115),
    ):
        body.visual(
            Box((0.042, 0.010, 0.016)),
            origin=Origin(
                xyz=(hinge_x, body_depth * 0.5 - 0.011, door_hinge_z),
            ),
            material=metal,
            name=name,
        )
    for name, foot_x, foot_y in (
        ("foot_front_left", -0.145, 0.105),
        ("foot_front_right", 0.145, 0.105),
        ("foot_back_left", -0.145, -0.120),
        ("foot_back_right", 0.145, -0.120),
    ):
        body.visual(
            Box((0.040, 0.032, foot_height)),
            origin=Origin(xyz=(foot_x, foot_y, foot_height * 0.5)),
            material=rubber,
            name=name,
        )

    door = model.part("door")
    frame_width = 0.022
    frame_top = 0.022
    frame_bottom = 0.026
    door_center_y = door_depth * 0.5
    bottom_frame_depth = 0.010
    bottom_frame_center_y = 0.017
    glass_width = door_width - 2.0 * frame_width + 0.008
    glass_height = door_height - frame_top - frame_bottom + 0.008
    glass_center_z = frame_bottom + (door_height - frame_bottom - frame_top) * 0.5
    door.visual(
        Box((frame_width, door_depth, door_height)),
        origin=Origin(xyz=(-door_width * 0.5 + frame_width * 0.5, door_center_y, door_height * 0.5)),
        material=enamel,
        name="left_frame",
    )
    door.visual(
        Box((frame_width, door_depth, door_height)),
        origin=Origin(xyz=(door_width * 0.5 - frame_width * 0.5, door_center_y, door_height * 0.5)),
        material=enamel,
        name="right_frame",
    )
    door.visual(
        Box((door_width, door_depth, frame_top)),
        origin=Origin(xyz=(0.0, door_center_y, door_height - frame_top * 0.5)),
        material=enamel,
        name="top_frame",
    )
    door.visual(
        Box((door_width, bottom_frame_depth, frame_bottom)),
        origin=Origin(xyz=(0.0, bottom_frame_center_y, frame_bottom * 0.5)),
        material=enamel,
        name="bottom_frame",
    )
    door.visual(
        Box((glass_width, 0.006, glass_height)),
        origin=Origin(xyz=(0.0, 0.004, glass_center_z)),
        material=glass,
        name="glass_panel",
    )
    door.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(-0.096, 0.019, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="handle_standoff_left",
    )
    door.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.096, 0.019, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="handle_standoff_right",
    )
    door.visual(
        Box((0.026, 0.008, 0.044)),
        origin=Origin(xyz=(-0.096, 0.011, 0.145)),
        material=metal,
        name="handle_mount_left",
    )
    door.visual(
        Box((0.026, 0.008, 0.044)),
        origin=Origin(xyz=(0.096, 0.011, 0.145)),
        material=metal,
        name="handle_mount_right",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.208),
        origin=Origin(xyz=(0.0, 0.036, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="handle_grip",
    )
    for name, leaf_x in (
        ("hinge_leaf_left", -0.065),
        ("hinge_leaf_right", 0.065),
    ):
        door.visual(
            Box((0.050, 0.010, 0.012)),
            origin=Origin(xyz=(leaf_x, 0.012, 0.006)),
            material=metal,
            name=name,
        )
    door.visual(
        Cylinder(radius=0.007, length=0.056),
        origin=Origin(xyz=(-0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_knuckle_left",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.056),
        origin=Origin(xyz=(0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_knuckle_right",
    )

    knob_specs = [
        ("temp_knob", knob_positions[0], 0.017),
        ("function_knob", knob_positions[1], 0.017),
        ("timer_knob", knob_positions[2], 0.018),
    ]
    for part_name, (knob_x, knob_z), knob_radius in knob_specs:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=0.0045, length=0.018),
            origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.0032, length=0.010),
            origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="rear_shaft",
        )
        knob.visual(
            Cylinder(radius=0.0115, length=0.003),
            origin=Origin(xyz=(0.0, 0.0045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="shaft_collar",
        )
        knob.visual(
            Cylinder(radius=knob_radius, length=0.022),
            origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_dark,
            name="knob_barrel",
        )
        knob.visual(
            Cylinder(radius=knob_radius * 0.88, length=0.006),
            origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=trim,
            name="knob_face",
        )
        knob.visual(
            Box((0.004, 0.003, 0.010)),
            origin=Origin(xyz=(0.0, 0.034, knob_radius * 0.70)),
            material=metal,
            name="pointer",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, fascia_center_y, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.35,
                velocity=5.0,
                lower=-2.6,
                upper=2.6,
            ),
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_center_x, hinge_y, door_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.4,
            lower=0.0,
            upper=1.52,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    temp_knob = object_model.get_part("temp_knob")
    function_knob = object_model.get_part("function_knob")
    timer_knob = object_model.get_part("timer_knob")
    door_hinge = object_model.get_articulation("body_to_door")
    temp_joint = object_model.get_articulation("body_to_temp_knob")
    function_joint = object_model.get_articulation("body_to_function_knob")
    timer_joint = object_model.get_articulation("body_to_timer_knob")

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

    ctx.expect_gap(
        door,
        body,
        axis="y",
        min_gap=0.003,
        max_gap=0.012,
        positive_elem="glass_panel",
        negative_elem="front_fascia",
        name="closed_door_glass_stands_proud_of_fascia",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.14,
        elem_a="glass_panel",
        elem_b="front_fascia",
        name="door_covers_opening_zone",
    )

    for knob in (temp_knob, function_knob, timer_knob):
        ctx.expect_contact(
            knob,
            body,
            elem_a="shaft_collar",
            elem_b="front_fascia",
            name=f"{knob.name}_collar_seats_on_panel",
        )
        ctx.expect_gap(
            knob,
            body,
            axis="y",
            min_gap=0.020,
            max_gap=0.060,
            positive_elem="knob_face",
            negative_elem="front_fascia",
            name=f"{knob.name}_face_clears_panel",
        )

    closed_top = ctx.part_element_world_aabb(door, elem="top_frame")
    with ctx.pose({door_hinge: 1.45}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_overlap_free")
        opened_top = ctx.part_element_world_aabb(door, elem="top_frame")
    door_motion_ok = False
    if closed_top is not None and opened_top is not None:
        closed_y_max = closed_top[1][1]
        closed_z_max = closed_top[1][2]
        opened_y_max = opened_top[1][1]
        opened_z_max = opened_top[1][2]
        door_motion_ok = opened_y_max > closed_y_max + 0.12 and opened_z_max < closed_z_max - 0.12
    ctx.check(
        "door_folds_forward_and_down",
        door_motion_ok,
        details="Door top frame did not move clearly forward and downward between closed and open poses.",
    )

    pointer_closed = ctx.part_element_world_aabb(temp_knob, elem="pointer")
    with ctx.pose({temp_joint: 1.2}):
        pointer_rotated = ctx.part_element_world_aabb(temp_knob, elem="pointer")
    knob_motion_ok = False
    if pointer_closed is not None and pointer_rotated is not None:
        closed_pointer_x = (pointer_closed[0][0] + pointer_closed[1][0]) * 0.5
        rotated_pointer_x = (pointer_rotated[0][0] + pointer_rotated[1][0]) * 0.5
        knob_motion_ok = rotated_pointer_x > closed_pointer_x + 0.008
    ctx.check(
        "temp_knob_pointer_rotates_about_shaft",
        knob_motion_ok,
        details="Temp knob pointer did not sweep laterally when the joint rotated.",
    )

    joints_ok = (
        tuple(door_hinge.axis) == (-1.0, 0.0, 0.0)
        and tuple(temp_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(function_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(timer_joint.axis) == (0.0, 1.0, 0.0)
    )
    ctx.check(
        "primary_axes_match_mechanisms",
        joints_ok,
        details="Door hinge or knob shaft axes do not match the intended manufacturable mechanisms.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
