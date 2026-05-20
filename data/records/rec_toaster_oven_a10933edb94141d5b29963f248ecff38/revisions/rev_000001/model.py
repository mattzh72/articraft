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

    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.78, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    black = model.material("black", rgba=(0.08, 0.09, 0.10, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.16, 0.22, 0.25, 0.40))
    warm_glass = model.material("warm_glass", rgba=(0.18, 0.24, 0.27, 0.30))
    knob_mark = model.material("knob_mark", rgba=(0.88, 0.88, 0.83, 1.0))

    outer_width = 0.470
    outer_depth = 0.335
    outer_height = 0.285
    shell_thickness = 0.012
    face_depth = 0.018
    control_panel_width = 0.116
    divider_width = 0.012
    header_height = 0.048
    lower_sill_height = 0.026
    door_gap = 0.0

    door_width = outer_width - control_panel_width - divider_width - (2.0 * shell_thickness)
    door_height = 0.220
    door_thickness = 0.018
    door_center_x = (-outer_width * 0.5) + shell_thickness + (door_width * 0.5)
    divider_center_x = door_center_x + (door_width * 0.5) + (divider_width * 0.5)
    control_center_x = (outer_width * 0.5) - (control_panel_width * 0.5)
    front_face_y = outer_depth * 0.5
    door_hinge_z = lower_sill_height

    body = model.part("body")
    body.visual(
        Box((outer_width, outer_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, shell_thickness * 0.5)),
        material=stainless,
        name="floor_shell",
    )
    body.visual(
        Box((outer_width, outer_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, outer_height - (shell_thickness * 0.5))),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((shell_thickness, outer_depth, outer_height)),
        origin=Origin(
            xyz=(-(outer_width * 0.5) + (shell_thickness * 0.5), 0.0, outer_height * 0.5)
        ),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((shell_thickness, outer_depth, outer_height)),
        origin=Origin(
            xyz=((outer_width * 0.5) - (shell_thickness * 0.5), 0.0, outer_height * 0.5)
        ),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((outer_width, shell_thickness, outer_height)),
        origin=Origin(
            xyz=(0.0, -(outer_depth * 0.5) + (shell_thickness * 0.5), outer_height * 0.5)
        ),
        material=charcoal,
        name="back_wall",
    )
    body.visual(
        Box((door_width + shell_thickness, face_depth, header_height)),
        origin=Origin(
            xyz=(
                door_center_x - (shell_thickness * 0.5),
                front_face_y - (face_depth * 0.5),
                outer_height - (header_height * 0.5),
            )
        ),
        material=charcoal,
        name="door_header",
    )
    body.visual(
        Box((door_width, 0.030, lower_sill_height)),
        origin=Origin(
            xyz=(
                door_center_x,
                front_face_y - 0.015,
                lower_sill_height * 0.5,
            )
        ),
        material=charcoal,
        name="door_sill",
    )
    body.visual(
        Box((divider_width, face_depth, outer_height)),
        origin=Origin(
            xyz=(divider_center_x, front_face_y - (face_depth * 0.5), outer_height * 0.5)
        ),
        material=charcoal,
        name="panel_divider",
    )
    body.visual(
        Box((control_panel_width, face_depth, outer_height)),
        origin=Origin(
            xyz=(control_center_x, front_face_y - (face_depth * 0.5), outer_height * 0.5)
        ),
        material=charcoal,
        name="control_panel",
    )

    knob_x = control_center_x
    knob_z_positions = (0.206, 0.147, 0.088)
    for index, knob_z in enumerate(knob_z_positions):
        body.visual(
            Cylinder(radius=0.022, length=0.004),
            origin=Origin(
                xyz=(knob_x, front_face_y - 0.002, knob_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=black,
            name=f"knob_bezel_{index}",
        )

    body.visual(
        Box((0.052, 0.003, 0.016)),
        origin=Origin(
            xyz=(control_center_x, front_face_y - 0.0005, outer_height - 0.046),
        ),
        material=knob_mark,
        name="status_window",
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_width, outer_depth, outer_height)),
        mass=7.6,
        origin=Origin(xyz=(0.0, 0.0, outer_height * 0.5)),
    )

    door = model.part("door")
    side_frame = 0.028
    top_frame = 0.028
    bottom_frame = 0.040
    glass_width = door_width - (2.0 * side_frame)
    glass_height = door_height - top_frame - bottom_frame

    door.visual(
        Box((side_frame, door_thickness, door_height)),
        origin=Origin(
            xyz=(-(door_width * 0.5) + (side_frame * 0.5), door_thickness * 0.5, door_height * 0.5)
        ),
        material=stainless,
        name="left_stile",
    )
    door.visual(
        Box((side_frame, door_thickness, door_height)),
        origin=Origin(
            xyz=((door_width * 0.5) - (side_frame * 0.5), door_thickness * 0.5, door_height * 0.5)
        ),
        material=stainless,
        name="right_stile",
    )
    door.visual(
        Box((door_width, door_thickness, top_frame)),
        origin=Origin(xyz=(0.0, door_thickness * 0.5, door_height - (top_frame * 0.5))),
        material=stainless,
        name="top_rail",
    )
    door.visual(
        Box((door_width, door_thickness, bottom_frame)),
        origin=Origin(xyz=(0.0, door_thickness * 0.5, bottom_frame * 0.5)),
        material=stainless,
        name="bottom_rail",
    )
    door.visual(
        Box((glass_width, 0.004, glass_height)),
        origin=Origin(
            xyz=(
                0.0,
                0.006,
                bottom_frame + (glass_height * 0.5),
            )
        ),
        material=warm_glass,
        name="door_glass",
    )
    handle_z = door_height - 0.060
    for index, handle_x in enumerate((-0.078, 0.078)):
        door.visual(
            Box((0.014, 0.026, 0.018)),
            origin=Origin(xyz=(handle_x, door_thickness + 0.004, handle_z)),
            material=charcoal,
            name=f"handle_post_{index}",
        )
    door.visual(
        Box((0.204, door_thickness, 0.024)),
        origin=Origin(xyz=(0.0, door_thickness * 0.5, handle_z)),
        material=charcoal,
        name="handle_mount",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.188),
        origin=Origin(
            xyz=(0.0, door_thickness + 0.018, handle_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=charcoal,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.045, door_height)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0225, door_height * 0.5)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_center_x, front_face_y + door_gap, door_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=0.0,
            upper=1.70,
        ),
    )

    knob_radius = 0.018
    knob_depth = 0.022
    shaft_radius = 0.005
    shaft_depth = 0.010
    indicator_width = 0.004
    indicator_height = 0.010
    indicator_depth = 0.003

    for index, knob_z in enumerate(knob_z_positions):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=shaft_radius, length=shaft_depth),
            origin=Origin(
                xyz=(0.0, shaft_depth * 0.5, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=black,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=knob_radius, length=knob_depth),
            origin=Origin(
                xyz=(0.0, shaft_depth + (knob_depth * 0.5), 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=black,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(
                xyz=(0.0, shaft_depth + knob_depth + 0.003, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=charcoal,
            name="knob_cap",
        )
        knob.visual(
            Box((indicator_width, indicator_depth, indicator_height)),
            origin=Origin(
                xyz=(0.0, shaft_depth + knob_depth + 0.004, knob_radius - 0.003),
            ),
            material=knob_mark,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.022, length=0.040),
            mass=0.08,
            origin=Origin(
                xyz=(0.0, 0.020, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, front_face_y, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.6,
                velocity=6.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            max_penetration=1e-4,
            max_gap=0.001,
            negative_elem="door_sill",
            name="door closes flush to the front sill plane",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.200,
            name="door remains centered over the oven opening",
        )
        for index in range(3):
            knob = object_model.get_part(f"knob_{index}")
            ctx.expect_contact(
                knob,
                body,
                contact_tol=5e-4,
                name=f"knob {index + 1} remains mounted to the control panel",
            )

    closed_top = ctx.part_element_world_aabb(door, elem="top_rail")
    with ctx.pose({door_hinge: math.radians(90.0)}):
        open_top = ctx.part_element_world_aabb(door, elem="top_rail")

    if closed_top is not None and open_top is not None:
        closed_top_center_y = (closed_top[0][1] + closed_top[1][1]) * 0.5
        open_top_center_y = (open_top[0][1] + open_top[1][1]) * 0.5
        closed_top_center_z = (closed_top[0][2] + closed_top[1][2]) * 0.5
        open_top_center_z = (open_top[0][2] + open_top[1][2]) * 0.5
        ctx.check(
            "door top swings forward and downward when opened",
            open_top_center_y > closed_top_center_y + 0.10
            and open_top_center_z < closed_top_center_z - 0.08,
            details=(
                f"closed_center=({closed_top_center_y:.4f}, {closed_top_center_z:.4f}), "
                f"open_center=({open_top_center_y:.4f}, {open_top_center_z:.4f})"
            ),
        )
    else:
        ctx.fail("door top swings forward and downward when opened", "top rail AABB unavailable")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
