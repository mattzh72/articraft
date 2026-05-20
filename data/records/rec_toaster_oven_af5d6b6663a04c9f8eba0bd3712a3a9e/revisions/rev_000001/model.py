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

    body_metal = model.material("body_metal", rgba=(0.19, 0.20, 0.22, 1.0))
    trim_metal = model.material("trim_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    cavity_dark = model.material("cavity_dark", rgba=(0.15, 0.14, 0.13, 1.0))

    body_width = 0.50
    body_depth = 0.37
    body_height = 0.31
    shell_thickness = 0.008
    floor_thickness = 0.014
    roof_thickness = 0.012
    back_thickness = 0.010
    panel_width = 0.110
    divider_thickness = 0.008
    front_frame_depth = 0.012
    chamber_width = body_width - panel_width
    door_opening_width = 0.364
    door_opening_height = 0.178
    door_hinge_z = 0.052
    chamber_center_x = -(panel_width * 0.5)
    front_surface_y = -body_depth * 0.5

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=body_metal,
        name="floor_shell",
    )
    body.visual(
        Box((body_width, body_depth, roof_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height - roof_thickness * 0.5)),
        material=body_metal,
        name="roof_shell",
    )
    body.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(
            xyz=(-body_width * 0.5 + shell_thickness * 0.5, 0.0, body_height * 0.5)
        ),
        material=body_metal,
        name="left_wall",
    )
    body.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(
            xyz=(body_width * 0.5 - shell_thickness * 0.5, 0.0, body_height * 0.5)
        ),
        material=body_metal,
        name="right_wall",
    )
    body.visual(
        Box((body_width, back_thickness, body_height)),
        origin=Origin(
            xyz=(0.0, body_depth * 0.5 - back_thickness * 0.5, body_height * 0.5)
        ),
        material=body_metal,
        name="back_wall",
    )
    body.visual(
        Box((divider_thickness, body_depth - 0.026, body_height - 0.020)),
        origin=Origin(
            xyz=(
                body_width * 0.5 - panel_width - divider_thickness * 0.5,
                0.0,
                (body_height - 0.020) * 0.5 + 0.010,
            )
        ),
        material=panel_dark,
        name="panel_divider",
    )
    body.visual(
        Box((body_width - panel_width + 0.002, front_frame_depth, 0.054)),
        origin=Origin(
            xyz=(
                chamber_center_x,
                -body_depth * 0.5 + front_frame_depth * 0.5,
                body_height - 0.027,
            )
        ),
        material=trim_metal,
        name="door_header",
    )
    body.visual(
        Box((0.026, front_frame_depth, door_opening_height + 0.046)),
        origin=Origin(
            xyz=(
                -body_width * 0.5 + 0.019,
                -body_depth * 0.5 + front_frame_depth * 0.5,
                door_hinge_z + door_opening_height * 0.5 + 0.010,
            )
        ),
        material=trim_metal,
        name="left_jamb",
    )
    body.visual(
        Box((body_width - panel_width + 0.010, front_frame_depth, 0.030)),
        origin=Origin(
            xyz=(
                chamber_center_x,
                -body_depth * 0.5 + front_frame_depth * 0.5,
                0.027,
            )
        ),
        material=trim_metal,
        name="bottom_sill",
    )
    for hinge_x, suffix in (
        (chamber_center_x - 0.135, "left"),
        (chamber_center_x + 0.135, "right"),
    ):
        body.visual(
            Box((0.036, 0.010, 0.020)),
            origin=Origin(xyz=(hinge_x, front_surface_y + 0.005, 0.052)),
            material=trim_metal,
            name=f"hinge_mount_{suffix}",
        )
    body.visual(
        Box((panel_width + 0.004, front_frame_depth, body_height - 0.014)),
        origin=Origin(
            xyz=(
                body_width * 0.5 - panel_width * 0.5 - 0.002,
                -body_depth * 0.5 + front_frame_depth * 0.5,
                (body_height - 0.014) * 0.5 + 0.007,
            )
        ),
        material=panel_dark,
        name="control_panel_face",
    )
    body.visual(
        Box((panel_width - 0.018, 0.022, 0.050)),
        origin=Origin(
            xyz=(
                body_width * 0.5 - panel_width * 0.5 - 0.002,
                -body_depth * 0.5 + 0.022 * 0.5 + 0.002,
                body_height - 0.044,
            )
        ),
        material=cavity_dark,
        name="display_block",
    )
    for index, knob_z in enumerate((0.226, 0.160, 0.094)):
        body.visual(
            Cylinder(radius=0.023, length=0.004),
            origin=Origin(
                xyz=(
                    body_width * 0.5 - panel_width * 0.5 - 0.002,
                    -body_depth * 0.5 + 0.004,
                    knob_z,
                ),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=trim_metal,
            name=f"knob_bezel_{index}",
        )
    for foot_x in (-0.165, 0.165):
        for foot_y in (-0.125, 0.125):
            body.visual(
                Box((0.038, 0.038, 0.016)),
                origin=Origin(xyz=(foot_x, foot_y, 0.004)),
                material=panel_dark,
                name=f"foot_{'l' if foot_x < 0 else 'r'}_{'f' if foot_y < 0 else 'b'}",
            )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    door = model.part("door")
    door_width = 0.372
    door_height = 0.176
    door_thickness = 0.022
    door_frame_width = 0.026
    handle_z = 0.118
    handle_y = -0.043
    glass_width = door_width - door_frame_width * 2.0 - 0.014
    glass_height = door_height - door_frame_width - 0.044

    door.visual(
        Box((door_width, door_thickness, 0.028)),
        origin=Origin(xyz=(0.0, -door_thickness * 0.5, 0.014)),
        material=trim_metal,
        name="bottom_rail",
    )
    door.visual(
        Box((door_width, door_thickness, door_frame_width)),
        origin=Origin(
            xyz=(0.0, -door_thickness * 0.5, door_height - door_frame_width * 0.5)
        ),
        material=trim_metal,
        name="top_rail",
    )
    door.visual(
        Box((door_frame_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(door_width * 0.5 - door_frame_width * 0.5, -door_thickness * 0.5, door_height * 0.5)
        ),
        material=trim_metal,
        name="right_stile",
    )
    door.visual(
        Box((door_frame_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(-door_width * 0.5 + door_frame_width * 0.5, -door_thickness * 0.5, door_height * 0.5)
        ),
        material=trim_metal,
        name="left_stile",
    )
    door.visual(
        Box((glass_width, 0.006, glass_height)),
        origin=Origin(
            xyz=(0.0, -0.009, door_frame_width + glass_height * 0.5 - 0.004)
        ),
        material=model.material("door_glass", rgba=(0.18, 0.29, 0.36, 0.35)),
        name="glass",
    )
    door.visual(
        Box((door_width * 0.88, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, -0.006, 0.012)),
        material=body_metal,
        name="inner_lip",
    )
    for handle_x, suffix in ((-0.108, "left"), (0.108, "right")):
        door.visual(
            Cylinder(radius=0.005, length=0.036),
            origin=Origin(
                xyz=(handle_x, -0.024, handle_z),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=trim_metal,
            name=f"handle_post_{suffix}",
        )
    door.visual(
        Cylinder(radius=0.007, length=door_width * 0.66),
        origin=Origin(
            xyz=(0.0, handle_y, handle_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_metal,
        name="handle_bar",
    )
    for barrel_x, suffix in ((-0.135, "left"), (0.135, "right")):
        door.visual(
            Cylinder(radius=0.005, length=0.034),
            origin=Origin(
                xyz=(barrel_x, -0.003, 0.005),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=body_metal,
            name=f"hinge_barrel_{suffix}",
        )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness + 0.034, door_height)),
        mass=1.1,
        origin=Origin(xyz=(0.0, -0.020, door_height * 0.5)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(chamber_center_x, front_surface_y - 0.002, door_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )

    knob_x = body_width * 0.5 - panel_width * 0.5 - 0.002
    knob_y = front_surface_y
    for index, knob_z in enumerate((0.226, 0.160, 0.094)):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.004, length=0.012),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=trim_metal,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.019, length=0.026),
            origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=panel_dark,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.015, length=0.008),
            origin=Origin(xyz=(0.0, -0.042, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=trim_metal,
            name="knob_cap",
        )
        knob.visual(
            Box((0.003, 0.006, 0.014)),
            origin=Origin(xyz=(0.0, -0.046, 0.012)),
            material=trim_metal,
            name="pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.038, 0.050, 0.038)),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.025, 0.0)),
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, knob_y, knob_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=5.0,
                lower=-math.radians(135.0),
                upper=math.radians(135.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")
    knob_0 = object_model.get_part("knob_0")
    knob_joint = object_model.get_articulation("body_to_knob_0")

    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=0.0,
        max_gap=0.006,
        name="door sits just ahead of the oven face when closed",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.14,
        name="closed door covers the oven opening footprint",
    )
    ctx.expect_gap(
        body,
        knob_0,
        axis="y",
        min_gap=0.0,
        max_gap=0.006,
        name="knob stands just proud of the control panel",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    closed_pointer_aabb = ctx.part_element_world_aabb(knob_0, elem="pointer")
    with ctx.pose({door_hinge: math.radians(88.0), knob_joint: math.radians(90.0)}):
        open_door_aabb = ctx.part_world_aabb(door)
        rotated_pointer_aabb = ctx.part_element_world_aabb(knob_0, elem="pointer")

    door_opened_forward = (
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.10
    )
    ctx.check(
        "door swings downward and outward",
        door_opened_forward,
        details=f"closed_aabb={closed_door_aabb}, open_aabb={open_door_aabb}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    pointer_rest = _aabb_center(closed_pointer_aabb)
    pointer_rotated = _aabb_center(rotated_pointer_aabb)
    knob_rotates = (
        pointer_rest is not None
        and pointer_rotated is not None
        and abs(pointer_rotated[1] - pointer_rest[1]) < 0.004
        and abs(pointer_rotated[0] - pointer_rest[0]) > 0.008
        and abs(pointer_rotated[2] - pointer_rest[2]) > 0.008
    )
    ctx.check(
        "knob pointer rotates around its shaft",
        knob_rotates,
        details=f"rest={pointer_rest}, rotated={pointer_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
