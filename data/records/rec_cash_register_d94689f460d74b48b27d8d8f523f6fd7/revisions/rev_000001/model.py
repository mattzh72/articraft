from __future__ import annotations

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


PANEL_ANGLE = math.radians(-18.0)
PANEL_SURFACE_ORIGIN = (0.0, -0.018, 0.199)


def _panel_point(local_x: float, local_y: float, local_z: float = 0.0) -> tuple[float, float, float]:
    cos_a = math.cos(PANEL_ANGLE)
    sin_a = math.sin(PANEL_ANGLE)
    return (
        PANEL_SURFACE_ORIGIN[0] + local_x,
        PANEL_SURFACE_ORIGIN[1] + local_y * cos_a - local_z * sin_a,
        PANEL_SURFACE_ORIGIN[2] + local_y * sin_a + local_z * cos_a,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electronic_cash_register")

    enclosure = model.material("enclosure", rgba=(0.69, 0.67, 0.61, 1.0))
    enclosure_dark = model.material("enclosure_dark", rgba=(0.56, 0.54, 0.49, 1.0))
    drawer_metal = model.material("drawer_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    drawer_trim = model.material("drawer_trim", rgba=(0.24, 0.25, 0.28, 1.0))
    department_amber = model.material("department_amber", rgba=(0.79, 0.62, 0.30, 1.0))
    department_olive = model.material("department_olive", rgba=(0.58, 0.62, 0.38, 1.0))
    department_cream = model.material("department_cream", rgba=(0.85, 0.80, 0.63, 1.0))
    number_key = model.material("number_key", rgba=(0.84, 0.84, 0.82, 1.0))
    key_frame = model.material("key_frame", rgba=(0.22, 0.22, 0.22, 1.0))
    display_bezel = model.material("display_bezel", rgba=(0.19, 0.20, 0.22, 1.0))
    display_glass = model.material("display_glass", rgba=(0.18, 0.36, 0.25, 0.55))

    body = model.part("body")
    body.visual(
        Box((0.445, 0.450, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=enclosure_dark,
        name="base_floor",
    )
    body.visual(
        Box((0.024, 0.450, 0.125)),
        origin=Origin(xyz=(-0.2105, 0.0, 0.0625)),
        material=enclosure,
        name="left_wall",
    )
    body.visual(
        Box((0.024, 0.450, 0.125)),
        origin=Origin(xyz=(0.2105, 0.0, 0.0625)),
        material=enclosure,
        name="right_wall",
    )
    body.visual(
        Box((0.445, 0.450, 0.021)),
        origin=Origin(xyz=(0.0, 0.0, 0.1145)),
        material=enclosure,
        name="lower_roof",
    )
    body.visual(
        Box((0.397, 0.024, 0.092)),
        origin=Origin(xyz=(0.0, -0.213, 0.058)),
        material=enclosure,
        name="rear_bulkhead",
    )
    body.visual(
        Box((0.395, 0.210, 0.080)),
        origin=Origin(xyz=_panel_point(0.0, 0.0, -0.052), rpy=(PANEL_ANGLE, 0.0, 0.0)),
        material=enclosure,
        name="keypad_housing",
    )
    body.visual(
        Box((0.360, 0.110, 0.145)),
        origin=Origin(xyz=(0.0, -0.176, 0.1975)),
        material=enclosure,
        name="rear_tower",
    )
    body.visual(
        Box((0.050, 0.165, 0.105)),
        origin=Origin(xyz=(-0.172, -0.060, 0.160)),
        material=enclosure,
        name="left_shoulder",
    )
    body.visual(
        Box((0.050, 0.165, 0.105)),
        origin=Origin(xyz=(0.172, -0.060, 0.160)),
        material=enclosure,
        name="right_shoulder",
    )
    body.visual(
        Box((0.365, 0.230, 0.004)),
        origin=Origin(xyz=_panel_point(0.0, 0.0, -0.014), rpy=(PANEL_ANGLE, 0.0, 0.0)),
        material=key_frame,
        name="keypad_deck",
    )
    body.visual(
        Box((0.172, 0.128, 0.006)),
        origin=Origin(xyz=_panel_point(-0.072, -0.005, -0.004), rpy=(PANEL_ANGLE, 0.0, 0.0)),
        material=enclosure_dark,
        name="department_field",
    )
    body.visual(
        Box((0.104, 0.140, 0.006)),
        origin=Origin(xyz=_panel_point(0.070, -0.012, -0.004), rpy=(PANEL_ANGLE, 0.0, 0.0)),
        material=enclosure_dark,
        name="number_field",
    )
    body.visual(
        Box((0.040, 0.150, 0.006)),
        origin=Origin(xyz=_panel_point(0.002, -0.010, -0.004), rpy=(PANEL_ANGLE, 0.0, 0.0)),
        material=enclosure_dark,
        name="field_bridge",
    )
    body.visual(
        Box((0.445, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.219, 0.126)),
        material=enclosure_dark,
        name="drawer_header",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.445, 0.450, 0.320)),
        mass=8.5,
        origin=Origin(xyz=(0.0, -0.025, 0.125)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.388, 0.340, 0.006)),
        origin=Origin(xyz=(0.0, -0.170, 0.003)),
        material=drawer_metal,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.006, 0.340, 0.065)),
        origin=Origin(xyz=(-0.191, -0.170, 0.0325)),
        material=drawer_metal,
        name="tray_wall_0",
    )
    drawer.visual(
        Box((0.006, 0.340, 0.065)),
        origin=Origin(xyz=(0.191, -0.170, 0.0325)),
        material=drawer_metal,
        name="tray_wall_1",
    )
    drawer.visual(
        Box((0.388, 0.006, 0.065)),
        origin=Origin(xyz=(0.0, -0.337, 0.0325)),
        material=drawer_metal,
        name="tray_back",
    )
    drawer.visual(
        Box((0.118, 0.004, 0.054)),
        origin=Origin(xyz=(-0.062, -0.160, 0.033)),
        material=drawer_metal,
        name="divider_0",
    )
    drawer.visual(
        Box((0.118, 0.004, 0.054)),
        origin=Origin(xyz=(0.062, -0.160, 0.033)),
        material=drawer_metal,
        name="divider_1",
    )
    drawer.visual(
        Box((0.004, 0.086, 0.036)),
        origin=Origin(xyz=(0.0, -0.255, 0.024)),
        material=drawer_metal,
        name="coin_divider",
    )
    drawer.visual(
        Box((0.410, 0.024, 0.096)),
        origin=Origin(xyz=(0.0, 0.012, 0.048)),
        material=drawer_trim,
        name="front_panel",
    )
    drawer.visual(
        Box((0.180, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.028, 0.058)),
        material=drawer_trim,
        name="front_lip",
    )
    drawer.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.025, 0.032), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drawer_metal,
        name="lock",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.410, 0.364, 0.100)),
        mass=2.2,
        origin=Origin(xyz=(0.0, -0.150, 0.050)),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.225, 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.180),
    )

    display_post = model.part("display_post")
    display_post.visual(
        Box((0.082, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=enclosure_dark,
        name="post_base",
    )
    display_post.visual(
        Box((0.030, 0.018, 0.128)),
        origin=Origin(xyz=(0.0, -0.010, 0.078)),
        material=display_bezel,
        name="post_stem",
    )
    display_post.visual(
        Box((0.070, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.016, 0.140)),
        material=display_bezel,
        name="post_crossbar",
    )
    display_post.visual(
        Box((0.012, 0.032, 0.026)),
        origin=Origin(xyz=(-0.029, -0.026, 0.139)),
        material=display_bezel,
        name="yoke_0",
    )
    display_post.visual(
        Box((0.012, 0.032, 0.026)),
        origin=Origin(xyz=(0.029, -0.026, 0.139)),
        material=display_bezel,
        name="yoke_1",
    )
    display_post.inertial = Inertial.from_geometry(
        Box((0.082, 0.060, 0.160)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.010, 0.080)),
    )
    model.articulation(
        "body_to_display_post",
        ArticulationType.FIXED,
        parent=body,
        child=display_post,
        origin=Origin(xyz=(0.0, -0.168, 0.270)),
    )

    display_head = model.part("display_head")
    display_head.visual(
        Cylinder(radius=0.006, length=0.074),
        origin=Origin(xyz=(0.0, -0.006, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=display_bezel,
        name="hinge_barrel",
    )
    display_head.visual(
        Box((0.152, 0.024, 0.076)),
        origin=Origin(xyz=(0.0, -0.014, 0.052)),
        material=display_bezel,
        name="housing",
    )
    display_head.visual(
        Box((0.130, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, -0.025, 0.054)),
        material=display_glass,
        name="display_screen",
    )
    display_head.inertial = Inertial.from_geometry(
        Box((0.152, 0.030, 0.086)),
        mass=0.30,
        origin=Origin(xyz=(0.0, -0.015, 0.045)),
    )
    model.articulation(
        "display_post_to_display_head",
        ArticulationType.REVOLUTE,
        parent=display_post,
        child=display_head,
        origin=Origin(xyz=(0.0, -0.026, 0.139)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=-0.50, upper=0.35),
    )

    def add_key(
        *,
        part_name: str,
        joint_name: str,
        local_x: float,
        local_y: float,
        size: tuple[float, float, float],
        travel: float,
        cap_material,
        effort: float,
        velocity: float,
    ) -> None:
        body.visual(
            Box((size[0] + 0.004, size[1] + 0.004, 0.002)),
            origin=Origin(
                xyz=_panel_point(local_x, local_y, -0.001),
                rpy=(PANEL_ANGLE, 0.0, 0.0),
            ),
            material=key_frame,
            name=f"{part_name}_seat",
        )
        key_part = model.part(part_name)
        key_part.visual(
            Box(size),
            origin=Origin(xyz=(0.0, 0.0, size[2] / 2.0)),
            material=cap_material,
            name="cap",
        )
        key_part.inertial = Inertial.from_geometry(
            Box(size),
            mass=0.035 if size[0] > 0.028 else 0.020,
            origin=Origin(xyz=(0.0, 0.0, size[2] / 2.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=key_part,
            origin=Origin(xyz=_panel_point(local_x, local_y), rpy=(PANEL_ANGLE, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=effort,
                velocity=velocity,
                lower=-travel,
                upper=0.0,
            ),
        )

    department_rows = (-0.052, -0.010, 0.032)
    department_cols = (-0.126, -0.088, -0.050, -0.012)
    department_materials = (department_amber, department_cream, department_olive)
    department_index = 0
    for row_index, local_y in enumerate(department_rows):
        for local_x in department_cols:
            add_key(
                part_name=f"dept_key_{department_index}",
                joint_name=f"body_to_dept_key_{department_index}",
                local_x=local_x,
                local_y=local_y,
                size=(0.032, 0.032, 0.014),
                travel=0.0022,
                cap_material=department_materials[row_index],
                effort=8.0,
                velocity=0.05,
            )
            department_index += 1

    number_rows = (-0.060, -0.032, -0.004, 0.024)
    number_cols = (0.040, 0.068, 0.096)
    number_index = 0
    for local_y in number_rows:
        for local_x in number_cols:
            add_key(
                part_name=f"num_key_{number_index}",
                joint_name=f"body_to_num_key_{number_index}",
                local_x=local_x,
                local_y=local_y,
                size=(0.021, 0.021, 0.010),
                travel=0.0016,
                cap_material=number_key,
                effort=5.0,
                velocity=0.05,
            )
            number_index += 1

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    display_pitch = object_model.get_articulation("display_post_to_display_head")
    display_head = object_model.get_part("display_head")
    dept_key = object_model.get_part("dept_key_0")
    dept_joint = object_model.get_articulation("body_to_dept_key_0")
    num_key = object_model.get_part("num_key_0")
    num_joint = object_model.get_articulation("body_to_num_key_0")

    for index in range(12):
        ctx.allow_overlap(
            f"dept_key_{index}",
            body,
            elem_a="cap",
            elem_b=f"dept_key_{index}_seat",
            reason="Each department key is intentionally simplified as a plunger entering a shallow seat proxy instead of a fully holed bezel.",
        )
        ctx.allow_overlap(
            f"num_key_{index}",
            body,
            elem_a="cap",
            elem_b=f"num_key_{index}_seat",
            reason="Each number key is intentionally simplified as a plunger entering a shallow seat proxy instead of a fully holed bezel.",
        )
    for yoke_name in ("yoke_0", "yoke_1"):
        ctx.allow_overlap(
            display_head,
            "display_post",
            elem_a="hinge_barrel",
            elem_b=yoke_name,
            reason="The display hinge barrel is intentionally captured inside the fixed yoke arms.",
        )

    ctx.expect_within(
        drawer,
        body,
        axes="xz",
        margin=0.002,
        name="drawer stays centered in the register opening",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: drawer_joint.motion_limits.upper}):
        ctx.expect_within(
            drawer,
            body,
            axes="xz",
            margin=0.002,
            name="extended drawer stays aligned to the shell",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            min_overlap=0.150,
            name="drawer remains retained when open",
        )
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "drawer extends forward",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[1] > drawer_rest[1] + 0.12,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    pitch_lower = display_pitch.motion_limits.lower
    pitch_upper = display_pitch.motion_limits.upper
    lower_screen = None
    upper_screen = None
    if pitch_lower is not None and pitch_upper is not None:
        with ctx.pose({display_pitch: pitch_lower}):
            lower_screen = ctx.part_element_world_aabb(display_head, elem="display_screen")
        with ctx.pose({display_pitch: pitch_upper}):
            upper_screen = ctx.part_element_world_aabb(display_head, elem="display_screen")
    ctx.check(
        "customer display head pitches through a visible arc",
        lower_screen is not None
        and upper_screen is not None
        and (
            ((upper_screen[0][2] + upper_screen[1][2]) * 0.5)
            > ((lower_screen[0][2] + lower_screen[1][2]) * 0.5) + 0.01
            or abs(((upper_screen[0][1] + upper_screen[1][1]) * 0.5) - ((lower_screen[0][1] + lower_screen[1][1]) * 0.5)) > 0.01
        ),
        details=f"lower={lower_screen}, upper={upper_screen}",
    )

    dept_rest = ctx.part_world_position(dept_key)
    with ctx.pose({dept_joint: dept_joint.motion_limits.lower}):
        dept_pressed = ctx.part_world_position(dept_key)
    ctx.check(
        "department key depresses into the sloped deck",
        dept_rest is not None
        and dept_pressed is not None
        and dept_pressed[2] < dept_rest[2] - 0.001,
        details=f"rest={dept_rest}, pressed={dept_pressed}",
    )

    num_rest = ctx.part_world_position(num_key)
    with ctx.pose({num_joint: num_joint.motion_limits.lower}):
        num_pressed = ctx.part_world_position(num_key)
    ctx.check(
        "number key depresses into the sloped deck",
        num_rest is not None
        and num_pressed is not None
        and num_pressed[2] < num_rest[2] - 0.0007,
        details=f"rest={num_rest}, pressed={num_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
