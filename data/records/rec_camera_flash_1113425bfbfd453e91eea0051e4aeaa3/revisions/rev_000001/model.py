from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_xy_section(width_x: float, width_y: float, z: float, radius: float):
    return [(x, y, z) for x, y in rounded_rect_profile(width_x, width_y, radius, corner_segments=8)]


def _rounded_profile_section(width_y: float, height_z: float, x: float, radius: float):
    return [(y, z, x) for y, z in rounded_rect_profile(width_y, height_z, radius, corner_segments=8)]


def _body_shell_mesh():
    return LoftGeometry(
        [
            _rounded_xy_section(0.043, 0.070, 0.012, 0.010),
            _rounded_xy_section(0.048, 0.074, 0.066, 0.012),
            _rounded_xy_section(0.042, 0.066, 0.120, 0.010),
        ],
        cap=True,
        closed=True,
    )


def _head_shell_mesh():
    raw = LoftGeometry(
        [
            _rounded_profile_section(0.052, 0.040, -0.014, 0.008),
            _rounded_profile_section(0.060, 0.046, 0.032, 0.010),
            _rounded_profile_section(0.056, 0.042, 0.083, 0.008),
        ],
        cap=True,
        closed=True,
    )
    mapped = MeshGeometry()
    for y, z, x in raw.vertices:
        mapped.add_vertex(x, y, z)
    for a, b, c in raw.faces:
        mapped.add_face(a, b, c)
    return mapped


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_camera_flash")

    satin_black = model.material("satin_black", rgba=(0.025, 0.027, 0.030, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.058, 0.064, 1.0))
    rubber = model.material("button_rubber", rgba=(0.090, 0.092, 0.098, 1.0))
    panel_black = model.material("panel_black", rgba=(0.010, 0.012, 0.014, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.10, 0.18, 0.20, 0.75))
    diffuser = model.material("diffuser", rgba=(0.92, 0.90, 0.78, 0.65))
    metal = model.material("brushed_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    label_grey = model.material("label_grey", rgba=(0.42, 0.43, 0.45, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_shell_mesh(), "flash_body_shell"),
        material=satin_black,
        name="body_shell",
    )
    body.visual(
        Box((0.060, 0.088, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=dark_plastic,
        name="mount_foot_plate",
    )
    body.visual(
        Box((0.068, 0.012, 0.005)),
        origin=Origin(xyz=(0.000, 0.036, 0.0025)),
        material=metal,
        name="mount_rail_0",
    )
    body.visual(
        Box((0.068, 0.012, 0.005)),
        origin=Origin(xyz=(0.000, -0.036, 0.0025)),
        material=metal,
        name="mount_rail_1",
    )
    body.visual(
        Box((0.036, 0.036, 0.0025)),
        origin=Origin(xyz=(0.000, 0.000, -0.00125)),
        material=metal,
        name="hotshoe_contact",
    )
    body.visual(
        Box((0.0030, 0.056, 0.094)),
        origin=Origin(xyz=(-0.0238, 0.000, 0.075)),
        material=panel_black,
        name="rear_panel",
    )
    body.visual(
        Box((0.0010, 0.029, 0.020)),
        origin=Origin(xyz=(-0.0274, 0.000, 0.090)),
        material=screen_glass,
        name="screen",
    )
    body.visual(
        Box((0.0018, 0.034, 0.025)),
        origin=Origin(xyz=(-0.0262, 0.000, 0.090)),
        material=label_grey,
        name="screen_bezel",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.123)),
        material=dark_plastic,
        name="swivel_socket",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=dark_plastic,
        name="swivel_disc",
    )
    yoke.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
        material=dark_plastic,
        name="swivel_neck",
    )
    yoke.visual(
        Box((0.038, 0.090, 0.014)),
        origin=Origin(xyz=(0.024, 0.000, 0.026)),
        material=dark_plastic,
        name="yoke_bridge",
    )
    for side_index, side_y in enumerate((-0.040, 0.040)):
        yoke.visual(
            Box((0.020, 0.008, 0.075)),
            origin=Origin(xyz=(0.034, side_y, 0.055)),
            material=dark_plastic,
            name=f"yoke_arm_{side_index}",
        )
        yoke.visual(
            Cylinder(radius=0.011, length=0.005),
            origin=Origin(
                xyz=(0.036, side_y + (0.006 if side_y > 0 else -0.006), 0.055),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal,
            name=f"axis_boss_{side_index}",
        )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(_head_shell_mesh(), "flash_head_shell"),
        material=satin_black,
        name="head_shell",
    )
    head.visual(
        Box((0.004, 0.052, 0.036)),
        origin=Origin(xyz=(0.0845, 0.000, 0.000)),
        material=diffuser,
        name="front_diffuser",
    )
    head.visual(
        Box((0.002, 0.050, 0.032)),
        origin=Origin(xyz=(0.0875, 0.000, 0.000)),
        material=diffuser,
        name="fresnel_face",
    )
    head.visual(
        Cylinder(radius=0.0055, length=0.072),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="tilt_axle",
    )
    for side_index, side_y in enumerate((-0.0340, 0.0340)):
        head.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(0.000, side_y, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"trunnion_cap_{side_index}",
        )

    model.articulation(
        "body_to_yoke",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yoke,
        origin=Origin(xyz=(0.000, 0.000, 0.126)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.4, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.036, 0.000, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.25, upper=1.35),
    )

    button_positions = [
        (0.000, 0.116, 0.006, 0.014, 0.008),
        (0.000, 0.064, 0.006, 0.014, 0.008),
        (-0.023, 0.090, 0.006, 0.010, 0.012),
        (0.023, 0.090, 0.006, 0.010, 0.012),
        (-0.018, 0.048, 0.006, 0.013, 0.008),
        (0.018, 0.048, 0.006, 0.013, 0.008),
        (-0.016, 0.030, 0.006, 0.012, 0.007),
        (0.016, 0.030, 0.006, 0.012, 0.007),
    ]
    for index, (local_y, local_z, _thick_x, size_y, size_z) in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.0030, size_y, size_z)),
            origin=Origin(xyz=(-0.0015, 0.000, 0.000)),
            material=rubber,
            name="button_cap",
        )
        button.visual(
            Box((0.0010, size_y * 0.55, size_z * 0.35)),
            origin=Origin(xyz=(-0.0035, 0.000, 0.000)),
            material=label_grey,
            name="button_mark",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(-0.0253, local_y, local_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=0.06, lower=0.0, upper=0.0025),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("body_to_yoke")
    pitch = object_model.get_articulation("yoke_to_head")

    ctx.expect_gap(
        yoke,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="swivel_disc",
        negative_elem="swivel_socket",
        name="swivel disc is seated on the body socket",
    )
    ctx.expect_within(
        head,
        yoke,
        axes="y",
        margin=0.001,
        name="flash head sits between the yoke arms",
    )
    ctx.expect_overlap(
        head,
        yoke,
        axes="xz",
        min_overlap=0.010,
        name="pitch axis crosses both head and yoke",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({swivel: 0.65}):
        turned_head_pos = ctx.part_world_position(head)
    ctx.check(
        "head swivels around the vertical body axis",
        rest_head_pos is not None
        and turned_head_pos is not None
        and turned_head_pos[1] > rest_head_pos[1] + 0.010,
        details=f"rest={rest_head_pos}, turned={turned_head_pos}",
    )

    rest_face = ctx.part_element_world_aabb(head, elem="front_diffuser")
    with ctx.pose({pitch: 1.0}):
        raised_face = ctx.part_element_world_aabb(head, elem="front_diffuser")
    ctx.check(
        "head pitch raises the forward diffuser",
        rest_face is not None
        and raised_face is not None
        and raised_face[0][2] > rest_face[0][2] + 0.025,
        details=f"rest={rest_face}, raised={raised_face}",
    )

    button_joints = [object_model.get_articulation(f"body_to_button_{i}") for i in range(8)]
    ctx.check(
        "rear button bank has eight independent prismatic controls",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in button_joints),
        details="expected all rear buttons to be prismatic push controls",
    )
    for index, joint in enumerate(button_joints):
        button = object_model.get_part(f"button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.0025}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[0] > rest_pos[0] + 0.0020,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
