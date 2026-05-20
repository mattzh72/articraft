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
    metal_dark = model.material("metal_dark", rgba=(0.33, 0.35, 0.37, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass_smoke = model.material("glass_smoke", rgba=(0.18, 0.24, 0.28, 0.45))
    foot_black = model.material("foot_black", rgba=(0.06, 0.06, 0.07, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_mark = model.material("knob_mark", rgba=(0.78, 0.80, 0.82, 1.0))

    body_width = 0.54
    body_depth = 0.39
    body_height = 0.31
    shell_t = 0.012
    base_t = 0.014
    front_band_depth = 0.022

    door_width = 0.405
    door_height = 0.228
    door_thickness = 0.020
    door_center_x = -0.028
    door_bottom_z = 0.034
    opening_left = door_center_x - door_width * 0.5
    opening_right = door_center_x + door_width * 0.5

    control_panel_width = 0.078
    control_panel_center_x = 0.220
    control_panel_left = control_panel_center_x - control_panel_width * 0.5

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t * 0.5)),
        material=metal_dark,
        name="base_pan",
    )
    body.visual(
        Box((body_width, body_depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, body_height - shell_t * 0.5)),
        material=body_paint,
        name="top_shell",
    )
    body.visual(
        Box((shell_t, body_depth, body_height - base_t)),
        origin=Origin(
            xyz=(-body_width * 0.5 + shell_t * 0.5, 0.0, base_t + (body_height - base_t) * 0.5)
        ),
        material=body_paint,
        name="left_shell",
    )
    body.visual(
        Box((shell_t, body_depth, body_height - base_t)),
        origin=Origin(
            xyz=(body_width * 0.5 - shell_t * 0.5, 0.0, base_t + (body_height - base_t) * 0.5)
        ),
        material=body_paint,
        name="right_shell",
    )
    body.visual(
        Box((body_width - 2.0 * shell_t, shell_t, body_height - base_t)),
        origin=Origin(
            xyz=(0.0, body_depth * 0.5 - shell_t * 0.5, base_t + (body_height - base_t) * 0.5)
        ),
        material=body_paint,
        name="back_shell",
    )

    left_fascia_width = opening_left - (-body_width * 0.5 + shell_t)
    divider_width = control_panel_left - opening_right

    body.visual(
        Box((left_fascia_width, front_band_depth, door_height + 0.022)),
        origin=Origin(
            xyz=(
                -body_width * 0.5 + shell_t + left_fascia_width * 0.5,
                -body_depth * 0.5 + front_band_depth * 0.5,
                door_bottom_z + door_height * 0.5,
            )
        ),
        material=body_paint,
        name="left_fascia",
    )
    body.visual(
        Box((door_width + 0.016, front_band_depth, door_bottom_z + 0.006)),
        origin=Origin(
            xyz=(
                door_center_x,
                -body_depth * 0.5 + front_band_depth * 0.5,
                (door_bottom_z + 0.006) * 0.5,
            )
        ),
        material=metal_dark,
        name="lower_sill",
    )
    body.visual(
        Box((door_width + 0.016, front_band_depth, body_height - (door_bottom_z + door_height) + 0.002)),
        origin=Origin(
            xyz=(
                door_center_x,
                -body_depth * 0.5 + front_band_depth * 0.5,
                door_bottom_z + door_height + (body_height - (door_bottom_z + door_height) + 0.002) * 0.5,
            )
        ),
        material=body_paint,
        name="upper_header",
    )
    body.visual(
        Box((divider_width, front_band_depth, door_height + 0.038)),
        origin=Origin(
            xyz=(
                opening_right + divider_width * 0.5,
                -body_depth * 0.5 + front_band_depth * 0.5,
                door_bottom_z + door_height * 0.5,
            )
        ),
        material=body_paint,
        name="right_divider",
    )
    body.visual(
        Box((control_panel_width, front_band_depth, 0.242)),
        origin=Origin(
            xyz=(
                control_panel_center_x,
                -body_depth * 0.5 + front_band_depth * 0.5,
                0.153,
            )
        ),
        material=metal_dark,
        name="control_panel",
    )
    body.visual(
        Box((control_panel_width - 0.018, 0.008, 0.050)),
        origin=Origin(
            xyz=(
                control_panel_center_x,
                -body_depth * 0.5 - 0.001,
                0.246,
            )
        ),
        material=trim_black,
        name="display_window",
    )
    body.visual(
        Box((door_width - 0.030, body_depth - 0.080, 0.004)),
        origin=Origin(xyz=(door_center_x, 0.008, 0.016)),
        material=trim_black,
        name="cavity_floor",
    )

    foot_offset_x = body_width * 0.5 - 0.060
    foot_offset_y = body_depth * 0.5 - 0.060
    for foot_index, (foot_x, foot_y) in enumerate(
        (
            (-foot_offset_x, -foot_offset_y),
            (foot_offset_x, -foot_offset_y),
            (-foot_offset_x, foot_offset_y),
            (foot_offset_x, foot_offset_y),
        )
    ):
        body.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(foot_x, foot_y, 0.005)),
            material=foot_black,
            name=f"foot_{foot_index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    door = model.part("door")
    frame_width = 0.035
    door.visual(
        Box((door_width, door_thickness, 0.038)),
        origin=Origin(xyz=(0.0, -door_thickness * 0.5, 0.019)),
        material=metal_dark,
        name="bottom_rail",
    )
    door.visual(
        Box((door_width, door_thickness, 0.032)),
        origin=Origin(xyz=(0.0, -door_thickness * 0.5, door_height - 0.016)),
        material=body_paint,
        name="top_rail",
    )
    door.visual(
        Box((frame_width, door_thickness, door_height)),
        origin=Origin(xyz=(-door_width * 0.5 + frame_width * 0.5, -door_thickness * 0.5, door_height * 0.5)),
        material=body_paint,
        name="left_stile",
    )
    door.visual(
        Box((frame_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width * 0.5 - frame_width * 0.5, -door_thickness * 0.5, door_height * 0.5)),
        material=body_paint,
        name="right_stile",
    )
    door.visual(
        Box((door_width - 2.0 * frame_width + 0.006, 0.006, door_height - 0.090)),
        origin=Origin(
            xyz=(
                0.0,
                -0.006,
                door_height * 0.5 + 0.008,
            )
        ),
        material=glass_smoke,
        name="window_glass",
    )
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        door.visual(
            Box((0.024, 0.034, 0.018)),
            origin=Origin(xyz=(x_sign * 0.103, -0.017, 0.158)),
            material=metal_dark,
            name=f"{side}_handle_post",
        )
    door.visual(
        Box((0.240, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.037, 0.158)),
        material=trim_black,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -door_thickness * 0.5, door_height * 0.5)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_center_x, -body_depth * 0.5, door_bottom_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )

    knob_z_positions = (0.214, 0.158, 0.102)
    for knob_index, knob_z in enumerate(knob_z_positions):
        knob = model.part(f"knob_{knob_index}")
        knob.visual(
            Cylinder(radius=0.0045, length=0.014),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=metal_dark,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.024, length=0.026),
            origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_black,
            name="skirt",
        )
        knob.visual(
            Cylinder(radius=0.017, length=0.010),
            origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_black,
            name="cap",
        )
        knob.visual(
            Box((0.005, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, -0.036, 0.016)),
            material=knob_mark,
            name="pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.024, length=0.036),
            mass=0.06,
            origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        )
        model.articulation(
            f"body_to_knob_{knob_index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(control_panel_center_x, -body_depth * 0.5, knob_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=3.5,
                lower=-2.4,
                upper=2.4,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")
    knob_0 = object_model.get_part("knob_0")
    knob_joints = [object_model.get_articulation(f"body_to_knob_{index}") for index in range(3)]

    ctx.expect_gap(
        body,
        door,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lower_sill",
        negative_elem="bottom_rail",
        name="closed door sits just ahead of the front frame",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.20,
        name="door covers the oven opening footprint",
    )

    for knob_index in range(3):
        ctx.expect_contact(
            f"knob_{knob_index}",
            body,
            elem_a="shaft",
            elem_b="control_panel",
            contact_tol=1e-5,
            name=f"knob {knob_index + 1} is shaft-mounted to the control panel",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door opens outward and downward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.10
        and open_aabb[1][2] < closed_aabb[1][2] - 0.06,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    def _aabb_center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    closed_pointer_aabb = ctx.part_element_world_aabb(knob_0, elem="pointer")
    with ctx.pose({knob_joints[0]: 1.1}):
        turned_pointer_aabb = ctx.part_element_world_aabb(knob_0, elem="pointer")

    ctx.check(
        "knob pointer sweeps when rotated",
        closed_pointer_aabb is not None
        and turned_pointer_aabb is not None
        and abs(_aabb_center(turned_pointer_aabb)[0] - _aabb_center(closed_pointer_aabb)[0]) > 0.01,
        details=f"closed={closed_pointer_aabb}, turned={turned_pointer_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
