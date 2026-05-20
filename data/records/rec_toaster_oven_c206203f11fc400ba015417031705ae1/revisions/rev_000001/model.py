from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

    stainless = model.material("stainless", rgba=(0.78, 0.79, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.15, 0.16, 0.17, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.16, 0.20, 0.23, 0.45))
    cavity_dark = model.material("cavity_dark", rgba=(0.23, 0.24, 0.25, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))
    indicator_silver = model.material("indicator_silver", rgba=(0.83, 0.84, 0.86, 1.0))

    body_width = 0.49
    body_depth = 0.38
    body_height = 0.285
    shell_t = 0.012
    fascia_t = 0.018

    door_width = 0.344
    door_height = 0.198
    door_bottom = 0.042
    door_center_x = -0.064

    control_panel_width = 0.106
    control_panel_x = 0.181
    knob_x = 0.181
    knob_zs = (0.212, 0.152, 0.092)

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=7.8,
        origin=Origin(xyz=(0.0, body_depth * 0.5, body_height * 0.5)),
    )

    # Outer shell: simple thin-walled appliance housing with a front opening.
    body.visual(
        Box((body_width, body_depth, shell_t)),
        origin=Origin(xyz=(0.0, body_depth * 0.5, shell_t * 0.5)),
        material=stainless,
        name="bottom_shell",
    )
    body.visual(
        Box((body_width, body_depth, shell_t)),
        origin=Origin(xyz=(0.0, body_depth * 0.5, body_height - shell_t * 0.5)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((shell_t, body_depth, body_height - 2.0 * shell_t)),
        origin=Origin(
            xyz=(-body_width * 0.5 + shell_t * 0.5, body_depth * 0.5, body_height * 0.5)
        ),
        material=stainless,
        name="left_shell",
    )
    body.visual(
        Box((shell_t, body_depth, body_height - 2.0 * shell_t)),
        origin=Origin(
            xyz=(body_width * 0.5 - shell_t * 0.5, body_depth * 0.5, body_height * 0.5)
        ),
        material=stainless,
        name="right_shell",
    )
    body.visual(
        Box((body_width - 2.0 * shell_t, shell_t, body_height - 2.0 * shell_t)),
        origin=Origin(
            xyz=(0.0, body_depth - shell_t * 0.5, body_height * 0.5)
        ),
        material=stainless,
        name="rear_shell",
    )

    # Front frame around the oven cavity.
    body.visual(
        Box((0.372, fascia_t, 0.040)),
        origin=Origin(xyz=(door_center_x, fascia_t * 0.5, body_height - 0.020)),
        material=stainless,
        name="door_header",
    )
    body.visual(
        Box((0.372, fascia_t, 0.044)),
        origin=Origin(xyz=(door_center_x, fascia_t * 0.5, 0.022)),
        material=stainless,
        name="door_sill",
    )
    body.visual(
        Box((0.024, fascia_t, 0.204)),
        origin=Origin(xyz=(-0.232, fascia_t * 0.5, 0.143)),
        material=stainless,
        name="left_jamb",
    )
    body.visual(
        Box((0.018, fascia_t, 0.204)),
        origin=Origin(xyz=(0.116, fascia_t * 0.5, 0.143)),
        material=stainless,
        name="right_jamb",
    )
    body.visual(
        Box((control_panel_width, fascia_t, 0.248)),
        origin=Origin(xyz=(control_panel_x, fascia_t * 0.5, 0.156)),
        material=stainless,
        name="control_panel",
    )
    body.visual(
        Box((control_panel_width, 0.008, 0.032)),
        origin=Origin(xyz=(control_panel_x, 0.008, 0.260)),
        material=dark_trim,
        name="top_trim_band",
    )

    # Darker cavity liner so the opening reads as a real oven chamber.
    body.visual(
        Box((0.326, 0.308, 0.022)),
        origin=Origin(xyz=(door_center_x, 0.154, 0.023)),
        material=cavity_dark,
        name="cavity_floor",
    )
    body.visual(
        Box((0.326, 0.010, 0.214)),
        origin=Origin(xyz=(door_center_x, 0.305, 0.136)),
        material=cavity_dark,
        name="cavity_back",
    )
    body.visual(
        Box((0.010, 0.308, 0.212)),
        origin=Origin(xyz=(-0.223, 0.154, 0.136)),
        material=cavity_dark,
        name="cavity_left_wall",
    )
    body.visual(
        Box((0.010, 0.308, 0.212)),
        origin=Origin(xyz=(0.095, 0.154, 0.136)),
        material=cavity_dark,
        name="cavity_right_wall",
    )
    body.visual(
        Box((0.326, 0.308, 0.012)),
        origin=Origin(xyz=(door_center_x, 0.154, 0.239)),
        material=cavity_dark,
        name="cavity_ceiling",
    )

    # Short shaft collars integrated into the control fascia to ground the knobs.
    for index, knob_z in enumerate(knob_zs):
        body.visual(
            Cylinder(radius=0.0105, length=0.010),
            origin=Origin(xyz=(knob_x, 0.005, knob_z), rpy=(1.57079632679, 0.0, 0.0)),
            material=dark_trim,
            name=f"shaft_collar_{index}",
        )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.080, door_height)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.030, door_height * 0.5)),
    )
    door.visual(
        Box((door_width, 0.016, 0.030)),
        origin=Origin(xyz=(0.0, -0.009, 0.015)),
        material=dark_trim,
        name="bottom_rail",
    )
    door.visual(
        Box((door_width, 0.016, 0.030)),
        origin=Origin(xyz=(0.0, -0.009, door_height - 0.015)),
        material=dark_trim,
        name="top_rail",
    )
    door.visual(
        Box((0.030, 0.016, 0.138)),
        origin=Origin(xyz=(-0.157, -0.009, door_height * 0.5)),
        material=dark_trim,
        name="left_rail",
    )
    door.visual(
        Box((0.030, 0.016, 0.138)),
        origin=Origin(xyz=(0.157, -0.009, door_height * 0.5)),
        material=dark_trim,
        name="right_rail",
    )
    door.visual(
        Box((0.286, 0.006, 0.138)),
        origin=Origin(xyz=(0.0, -0.004, door_height * 0.5)),
        material=dark_glass,
        name="glass_panel",
    )
    door.visual(
        Box((0.018, 0.045, 0.022)),
        origin=Origin(xyz=(-0.092, -0.038, 0.164)),
        material=dark_trim,
        name="left_handle_post",
    )
    door.visual(
        Box((0.018, 0.045, 0.022)),
        origin=Origin(xyz=(0.092, -0.038, 0.164)),
        material=dark_trim,
        name="right_handle_post",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.224),
        origin=Origin(xyz=(0.0, -0.061, 0.164), rpy=(0.0, 1.57079632679, 0.0)),
        material=stainless,
        name="handle_bar",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_center_x, 0.001, door_bottom)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.65,
        ),
    )

    for index, knob_z in enumerate(knob_zs):
        knob = model.part(f"knob_{index}")
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.024, length=0.048),
            mass=0.06,
            origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        )
        knob.visual(
            Cylinder(radius=0.010, length=0.022),
            origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
            material=dark_trim,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.021, length=0.006),
            origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
            material=dark_trim,
            name="skirt",
        )
        knob.visual(
            Cylinder(radius=0.018, length=0.026),
            origin=Origin(xyz=(0.0, -0.032, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.0025, 0.012, 0.020)),
            origin=Origin(xyz=(0.0, -0.044, 0.015)),
            material=indicator_silver,
            name="indicator",
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, 0.000, knob_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=5.0,
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

    knob_joints = [
        object_model.get_articulation("body_to_knob_0"),
        object_model.get_articulation("body_to_knob_1"),
        object_model.get_articulation("body_to_knob_2"),
    ]

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="door_sill",
            negative_elem="bottom_rail",
            max_gap=0.003,
            max_penetration=0.0,
            name="door sits just in front of the oven opening",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="x",
            elem_a="door_header",
            elem_b="top_rail",
            min_overlap=0.30,
            name="door spans the opening width",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens downward and outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.05
        and open_aabb[1][2] < closed_aabb[1][2] - 0.06,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for index, joint in enumerate(knob_joints):
        knob = object_model.get_part(f"knob_{index}")
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            positive_elem="control_panel",
            negative_elem="shaft",
            max_gap=0.0015,
            max_penetration=0.0,
            name=f"knob {index} seats on its shaft axis",
        )
        ctx.check(
            f"knob {index} uses a rotary control range",
            joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower < 0.0
            and joint.motion_limits.upper > 0.0,
            details=f"limits={joint.motion_limits}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
