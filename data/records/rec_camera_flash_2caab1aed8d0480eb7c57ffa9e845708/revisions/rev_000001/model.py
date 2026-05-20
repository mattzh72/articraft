from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.065
BODY_D = 0.050
BODY_H = 0.110
BODY_Z = 0.070
BODY_TOP_Z = BODY_Z + BODY_H / 2.0
PANEL_SURFACE_Y = -0.0244


def _rounded_box(size: tuple[float, float, float], center: tuple[float, float, float], radius: float) -> cq.Workplane:
    solid = cq.Workplane("XY").box(*size).translate(center)
    if radius > 0:
        solid = solid.edges("|Z").fillet(radius)
    return solid


def _body_shell_mesh():
    shell = _rounded_box((BODY_W, BODY_D, BODY_H), (0.0, 0.0, BODY_Z), 0.004)
    # Shallow rear control-panel recess: rear face is at -0.025 m and the
    # pocket floor is at -0.022 m, leaving a visible raised rim around the panel.
    panel_cutter = (
        cq.Workplane("XY")
        .box(0.052, 0.008, 0.064)
        .translate((0.0, -0.026, 0.079))
    )
    shell = shell.cut(panel_cutter)
    return mesh_from_cadquery(shell, "body_shell", tolerance=0.0007, angular_tolerance=0.12)


def _head_shell_mesh():
    shell = _rounded_box((0.080, 0.054, 0.036), (0.0, 0.027, 0.008), 0.003)
    # A subtle stepped brow above the flash window makes the head read less like
    # a plain rectangular block while remaining one continuous molded housing.
    brow = cq.Workplane("XY").box(0.074, 0.008, 0.006).translate((0.0, 0.004, 0.025))
    shell = shell.union(brow)
    return mesh_from_cadquery(shell, "flash_head_shell", tolerance=0.0007, angular_tolerance=0.12)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hot_shoe_camera_flash")

    body_plastic = model.material("slightly textured black plastic", rgba=(0.015, 0.016, 0.017, 1.0))
    matte_panel = model.material("matte recessed black", rgba=(0.002, 0.002, 0.003, 1.0))
    button_rubber = model.material("dark rubber button caps", rgba=(0.040, 0.042, 0.045, 1.0))
    lcd_glass = model.material("dim blue lcd glass", rgba=(0.070, 0.130, 0.170, 1.0))
    diffuser = model.material("warm translucent diffuser", rgba=(1.0, 0.92, 0.70, 0.62))
    shoe_metal = model.material("satin blackened metal", rgba=(0.18, 0.17, 0.15, 1.0))
    seam_black = model.material("black seam shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=body_plastic, name="body_shell")

    # Rectangular hot-shoe mounting foot and its narrow riser, connected to the
    # molded body bottom at camera-accessory scale.
    body.visual(
        Box((0.040, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=shoe_metal,
        name="mounting_foot",
    )
    body.visual(
        Box((0.029, 0.019, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=body_plastic,
        name="foot_riser",
    )
    body.visual(
        Box((0.052, 0.004, 0.002)),
        origin=Origin(xyz=(0.0, -0.014, 0.0065)),
        material=shoe_metal,
        name="rear_shoe_lip",
    )
    body.visual(
        Box((0.052, 0.004, 0.002)),
        origin=Origin(xyz=(0.0, 0.014, 0.0065)),
        material=shoe_metal,
        name="front_shoe_lip",
    )

    # Inset rear control panel, small LCD, and side battery-door seam details.
    body.visual(
        Box((0.049, 0.0024, 0.060)),
        origin=Origin(xyz=(0.0, -0.0232, 0.079)),
        material=matte_panel,
        name="control_panel",
    )
    body.visual(
        Box((0.026, 0.0008, 0.016)),
        origin=Origin(xyz=(0.0, -0.0248, 0.087)),
        material=lcd_glass,
        name="rear_screen",
    )
    for idx, (y, z, sy, sz) in enumerate(
        (
            (-0.018, 0.070, 0.0010, 0.052),
            (0.018, 0.070, 0.0010, 0.052),
            (0.0, 0.096, 0.036, 0.0010),
            (0.0, 0.044, 0.036, 0.0010),
        )
    ):
        body.visual(
            Box((0.0008, sy, sz)),
            origin=Origin(xyz=(BODY_W / 2.0 + 0.0004, y, z)),
            material=seam_black,
            name=f"battery_seam_{idx}",
        )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=body_plastic,
        name="swivel_collar",
    )
    neck.visual(
        Box((0.020, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=body_plastic,
        name="neck_post",
    )
    neck.visual(
        Box((0.096, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, -0.002, 0.012)),
        material=body_plastic,
        name="yoke_bridge",
    )
    for x in (-0.0455, 0.0455):
        neck.visual(
            Box((0.006, 0.020, 0.040)),
            origin=Origin(xyz=(x, 0.004, 0.028)),
            material=body_plastic,
            name="yoke_ear",
        )

    head = model.part("head")
    head.visual(_head_shell_mesh(), material=body_plastic, name="head_shell")
    head.visual(
        Box((0.068, 0.0012, 0.025)),
        origin=Origin(xyz=(0.0, 0.0546, 0.008)),
        material=diffuser,
        name="flash_window",
    )
    head.visual(
        Box((0.074, 0.0010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0545, 0.027)),
        material=seam_black,
        name="head_front_seam",
    )
    for x in (-0.0405, 0.0405):
        head.visual(
            Box((0.004, 0.014, 0.014)),
            origin=Origin(xyz=(x, 0.008, 0.004)),
            material=body_plastic,
            name="side_pivot_boss",
        )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.5, lower=-0.25, upper=1.55),
    )

    button_specs = [
        ("button_0", 0.000, 0.105, (0.016, 0.0028, 0.006)),
        ("button_1", 0.000, 0.067, (0.016, 0.0028, 0.006)),
        ("button_2", -0.020, 0.087, (0.007, 0.0028, 0.012)),
        ("button_3", 0.020, 0.087, (0.007, 0.0028, 0.012)),
        ("button_4", -0.014, 0.055, (0.012, 0.0028, 0.006)),
        ("button_5", 0.014, 0.055, (0.012, 0.0028, 0.006)),
    ]
    for name, x, z, size in button_specs:
        button = model.part(name)
        button.visual(
            Box(size),
            origin=Origin(xyz=(0.0, -size[1] / 2.0, 0.0)),
            material=button_rubber,
            name="button_cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, PANEL_SURFACE_Y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=0.04, lower=0.0, upper=0.0015),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")

    ctx.expect_gap(
        neck,
        body,
        axis="z",
        positive_elem="swivel_collar",
        negative_elem="body_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel collar is seated on body top",
    )

    button_names = [f"button_{i}" for i in range(6)]
    button_joints = [object_model.get_articulation(f"body_to_{name}") for name in button_names]
    ctx.check(
        "all visible rear buttons are prismatic parts",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in button_joints),
        details=[j.name for j in button_joints],
    )
    for name in button_names:
        button = object_model.get_part(name)
        ctx.expect_gap(
            body,
            button,
            axis="y",
            positive_elem="control_panel",
            negative_elem="button_cap",
            max_gap=0.0006,
            max_penetration=0.0,
            name=f"{name} sits raised on rear panel",
        )

    def _aabb_center_y(aabb):
        return (aabb[0][1] + aabb[1][1]) / 2.0

    def _aabb_center_x(aabb):
        return (aabb[0][0] + aabb[1][0]) / 2.0

    rest_button_aabb = ctx.part_world_aabb(object_model.get_part("button_0"))
    with ctx.pose({button_joints[0]: 0.0015}):
        pressed_button_aabb = ctx.part_world_aabb(object_model.get_part("button_0"))
    ctx.check(
        "rear push button travels inward",
        rest_button_aabb is not None
        and pressed_button_aabb is not None
        and _aabb_center_y(pressed_button_aabb) > _aabb_center_y(rest_button_aabb) + 0.001,
        details=f"rest={rest_button_aabb}, pressed={pressed_button_aabb}",
    )

    rest_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({tilt: 1.0}):
        tilted_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "flash head tilts upward on horizontal hinge",
        rest_head_aabb is not None
        and tilted_head_aabb is not None
        and tilted_head_aabb[1][2] > rest_head_aabb[1][2] + 0.015,
        details=f"rest={rest_head_aabb}, tilted={tilted_head_aabb}",
    )

    with ctx.pose({swivel: 0.65}):
        swiveled_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "neck swivel rotates head about vertical axis",
        rest_head_aabb is not None
        and swiveled_head_aabb is not None
        and abs(_aabb_center_x(swiveled_head_aabb)) > 0.010,
        details=f"rest={rest_head_aabb}, swiveled={swiveled_head_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
