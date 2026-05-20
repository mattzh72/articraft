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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    body_paint = model.material("body_paint", rgba=(0.20, 0.21, 0.23, 1.0))
    band_paint = model.material("band_paint", rgba=(0.13, 0.14, 0.16, 1.0))
    door_trim = model.material("door_trim", rgba=(0.11, 0.12, 0.13, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.24, 0.28, 0.30))
    steel = model.material("steel", rgba=(0.70, 0.71, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body_w = 0.52
    body_d = 0.38
    body_h = 0.31
    wall_t = 0.012
    front_frame_d = 0.016
    band_h = 0.032
    panel_w = 0.126
    divider_w = 0.014
    top_header_h = 0.050
    foot_w = 0.040
    foot_d = 0.028
    foot_h = 0.010

    x_min = -body_w * 0.5
    x_max = body_w * 0.5
    y_front = -body_d * 0.5
    y_back = body_d * 0.5

    opening_x_min = x_min + wall_t
    opening_x_max = x_max - wall_t - panel_w - divider_w
    opening_w = opening_x_max - opening_x_min
    opening_center_x = 0.5 * (opening_x_min + opening_x_max)
    opening_z_min = band_h
    opening_z_max = body_h - top_header_h
    opening_h = opening_z_max - opening_z_min

    shell = model.part("main_shell")
    shell.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - wall_t * 0.5)),
        material=body_paint,
        name="top_shell",
    )
    shell.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, wall_t * 0.5 + foot_h)),
        material=body_paint,
        name="floor_shell",
    )
    shell.visual(
        Box((wall_t, body_d, body_h - foot_h)),
        origin=Origin(
            xyz=(x_min + wall_t * 0.5, 0.0, foot_h + (body_h - foot_h) * 0.5),
        ),
        material=body_paint,
        name="left_wall",
    )
    shell.visual(
        Box((wall_t, body_d, body_h - foot_h)),
        origin=Origin(
            xyz=(x_max - wall_t * 0.5, 0.0, foot_h + (body_h - foot_h) * 0.5),
        ),
        material=body_paint,
        name="right_wall",
    )
    shell.visual(
        Box((body_w, wall_t, body_h - foot_h)),
        origin=Origin(
            xyz=(0.0, y_back - wall_t * 0.5, foot_h + (body_h - foot_h) * 0.5),
        ),
        material=body_paint,
        name="back_wall",
    )
    shell.visual(
        Box((panel_w, front_frame_d, body_h - foot_h)),
        origin=Origin(
            xyz=(
                x_max - wall_t - panel_w * 0.5,
                y_front + front_frame_d * 0.5,
                foot_h + (body_h - foot_h) * 0.5,
            ),
        ),
        material=body_paint,
        name="control_panel",
    )
    shell.visual(
        Box((divider_w, front_frame_d, opening_h)),
        origin=Origin(
            xyz=(
                opening_x_max + divider_w * 0.5,
                y_front + front_frame_d * 0.5,
                opening_z_min + opening_h * 0.5,
            ),
        ),
        material=body_paint,
        name="panel_divider",
    )
    shell.visual(
        Box((opening_w, front_frame_d, top_header_h)),
        origin=Origin(
            xyz=(
                opening_center_x,
                y_front + front_frame_d * 0.5,
                opening_z_max + top_header_h * 0.5,
            ),
        ),
        material=body_paint,
        name="top_header",
    )
    knob_x = x_max - wall_t - panel_w * 0.5
    knob_z_positions = (0.235, 0.165, 0.095)
    for knob_index, knob_z in enumerate(knob_z_positions):
        shell.visual(
            Cylinder(radius=0.0075, length=0.010),
            origin=Origin(
                xyz=(knob_x, y_front - 0.005, knob_z),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=steel,
            name=f"knob_shaft_{knob_index}",
        )
        shell.visual(
            Cylinder(radius=0.011, length=0.003),
            origin=Origin(
                xyz=(knob_x, y_front - 0.0015, knob_z),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=steel,
            name=f"knob_bezel_{knob_index}",
        )

    foot_x = body_w * 0.5 - 0.058
    foot_y = body_d * 0.5 - 0.055
    for side_x, side_name_x in ((-foot_x, "left"), (foot_x, "right")):
        for side_y, side_name_y in ((-foot_y, "front"), (foot_y, "rear")):
            shell.visual(
                Box((foot_w, foot_d, foot_h)),
                origin=Origin(xyz=(side_x, side_y, foot_h * 0.5)),
                material=rubber,
                name=f"{side_name_y}_{side_name_x}_foot",
            )

    shell.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    band = model.part("hinge_band")
    band.visual(
        Box((opening_w + 0.010, front_frame_d, band_h)),
        origin=Origin(
            xyz=(0.0, 0.0, band_h * 0.5),
        ),
        material=band_paint,
        name="reinforcement_strip",
    )
    band.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(
            xyz=(-opening_w * 0.5 + 0.030, -0.001, band_h - 0.009),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=steel,
        name="left_hinge_barrel",
    )
    band.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(
            xyz=(opening_w * 0.5 - 0.030, -0.001, band_h - 0.009),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=steel,
        name="right_hinge_barrel",
    )
    band.inertial = Inertial.from_geometry(
        Box((opening_w + 0.010, front_frame_d, band_h)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, band_h * 0.5)),
    )

    model.articulation(
        "shell_to_hinge_band",
        ArticulationType.FIXED,
        parent=shell,
        child=band,
        origin=Origin(xyz=(opening_center_x, y_front - front_frame_d * 0.5, foot_h)),
    )

    door = model.part("door")
    door_w = opening_w + 0.020
    door_h = opening_h + 0.012
    frame_t = 0.028
    door_depth = 0.018
    glass_w = door_w - 0.090
    glass_h = door_h - 0.082

    door.visual(
        Box((door_w, door_depth, frame_t)),
        origin=Origin(xyz=(0.0, 0.0, frame_t * 0.5)),
        material=door_trim,
        name="bottom_rail",
    )
    door.visual(
        Box((door_w, door_depth, frame_t)),
        origin=Origin(xyz=(0.0, 0.0, door_h - frame_t * 0.5)),
        material=door_trim,
        name="top_rail",
    )
    stile_h = door_h - 2.0 * frame_t
    stile_x = door_w * 0.5 - frame_t * 0.5
    door.visual(
        Box((frame_t, door_depth, stile_h)),
        origin=Origin(xyz=(-stile_x, 0.0, frame_t + stile_h * 0.5)),
        material=door_trim,
        name="left_stile",
    )
    door.visual(
        Box((frame_t, door_depth, stile_h)),
        origin=Origin(xyz=(stile_x, 0.0, frame_t + stile_h * 0.5)),
        material=door_trim,
        name="right_stile",
    )
    door.visual(
        Box((glass_w, 0.006, glass_h)),
        origin=Origin(
            xyz=(0.0, 0.002, frame_t + glass_h * 0.5 - 0.001),
        ),
        material=glass,
        name="glass_panel",
    )
    handle_z = door_h - 0.056
    handle_half_span = 0.080
    post_size = 0.012
    door.visual(
        Box((post_size, 0.026, 0.020)),
        origin=Origin(xyz=(-handle_half_span, -0.006, handle_z)),
        material=steel,
        name="left_handle_post",
    )
    door.visual(
        Box((post_size, 0.026, 0.020)),
        origin=Origin(xyz=(handle_half_span, -0.006, handle_z)),
        material=steel,
        name="right_handle_post",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.19),
        origin=Origin(
            xyz=(0.0, -0.018, handle_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_depth, door_h)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, door_h * 0.5)),
    )

    model.articulation(
        "hinge_band_to_door",
        ArticulationType.REVOLUTE,
        parent=band,
        child=door,
        origin=Origin(xyz=(0.0, -0.012, band_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )

    for knob_index, knob_z in enumerate(knob_z_positions):
        knob = model.part(f"knob_{knob_index}")
        knob.visual(
            Cylinder(radius=0.021, length=0.018),
            origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=door_trim,
            name="outer_body",
        )
        knob.visual(
            Cylinder(radius=0.0165, length=0.024),
            origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=band_paint,
            name="inner_body",
        )
        knob.visual(
            Box((0.0035, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, -0.020, 0.0135)),
            material=steel,
            name="pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.046, 0.028, 0.046)),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.010, 0.0)),
        )
        model.articulation(
            f"shell_to_knob_{knob_index}",
            ArticulationType.CONTINUOUS,
            parent=shell,
            child=knob,
            origin=Origin(xyz=(knob_x, y_front - 0.010, knob_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("main_shell")
    band = object_model.get_part("hinge_band")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("hinge_band_to_door")
    top_knob = object_model.get_part("knob_0")
    top_knob_joint = object_model.get_articulation("shell_to_knob_0")

    ctx.expect_contact(
        band,
        shell,
        elem_a="reinforcement_strip",
        elem_b="floor_shell",
        name="hinge band mounts to shell floor line",
    )
    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            band,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="reinforcement_strip",
            min_gap=0.0,
            max_gap=0.002,
            name="closed door seats on hinge line",
        )
        ctx.expect_overlap(
            door,
            shell,
            axes="xz",
            min_overlap=0.18,
            name="closed door covers oven opening footprint",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.2}):
        opened_aabb = ctx.part_world_aabb(door)
    closed_pos = None
    opened_pos = None
    if closed_aabb is not None:
        closed_pos = tuple(
            0.5 * (closed_aabb[0][axis] + closed_aabb[1][axis]) for axis in range(3)
        )
    if opened_aabb is not None:
        opened_pos = tuple(
            0.5 * (opened_aabb[0][axis] + opened_aabb[1][axis]) for axis in range(3)
        )
    ctx.check(
        "door opens downward and outward",
        closed_pos is not None
        and opened_pos is not None
        and opened_pos[1] < closed_pos[1] - 0.05
        and opened_pos[2] < closed_pos[2] - 0.02,
        details=f"closed={closed_pos}, opened={opened_pos}",
    )

    for knob_index in range(3):
        knob = object_model.get_part(f"knob_{knob_index}")
        ctx.expect_gap(
            shell,
            knob,
            axis="y",
            positive_elem="control_panel",
            min_gap=0.0,
            max_gap=0.012,
            name=f"knob {knob_index} sits just in front of control panel",
        )

    closed_pointer_aabb = ctx.part_element_world_aabb(top_knob, elem="pointer")
    with ctx.pose({top_knob_joint: math.pi * 0.5}):
        rotated_pointer_aabb = ctx.part_element_world_aabb(top_knob, elem="pointer")
    closed_pointer_center = None
    rotated_pointer_center = None
    if closed_pointer_aabb is not None:
        closed_pointer_center = tuple(
            0.5 * (closed_pointer_aabb[0][axis] + closed_pointer_aabb[1][axis])
            for axis in range(3)
        )
    if rotated_pointer_aabb is not None:
        rotated_pointer_center = tuple(
            0.5 * (rotated_pointer_aabb[0][axis] + rotated_pointer_aabb[1][axis])
            for axis in range(3)
        )
    ctx.check(
        "top knob pointer rotates around its shaft",
        closed_pointer_center is not None
        and rotated_pointer_center is not None
        and abs(rotated_pointer_center[0] - closed_pointer_center[0]) > 0.008
        and abs(rotated_pointer_center[2] - closed_pointer_center[2]) > 0.008,
        details=f"closed={closed_pointer_center}, rotated={rotated_pointer_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
