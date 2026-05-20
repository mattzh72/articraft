from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _canopy_shell_geometry() -> MeshGeometry:
    """Closed trapezoidal chimney-hood canopy, authored in meters."""

    bottom = [
        (-0.46, -0.29, 0.04),
        (0.46, -0.29, 0.04),
        (0.46, 0.19, 0.04),
        (-0.46, 0.19, 0.04),
    ]
    top = [
        (-0.17, -0.08, 0.33),
        (0.17, -0.08, 0.33),
        (0.17, 0.12, 0.33),
        (-0.17, 0.12, 0.33),
    ]

    geom = MeshGeometry()
    for vertex in bottom + top:
        geom.add_vertex(*vertex)

    faces = [
        # bottom and top caps
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        # sloped front, rear, and side panels
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ]
    for face in faces:
        geom.add_face(*face)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.64, 0.66, 0.64, 1.0))
    shadow = model.material("dark_filter", rgba=(0.04, 0.045, 0.05, 1.0))
    seam = model.material("soft_shadow_seam", rgba=(0.18, 0.19, 0.19, 1.0))
    button_face = model.material("satin_black_button", rgba=(0.015, 0.016, 0.018, 1.0))
    button_edge = model.material("button_chamfer_highlight", rgba=(0.16, 0.17, 0.17, 1.0))

    hood = model.part("hood")
    hood.visual(
        mesh_from_geometry(_canopy_shell_geometry(), "canopy_shell"),
        material=stainless,
        name="canopy_shell",
    )
    hood.visual(
        Box((0.92, 0.045, 0.12)),
        origin=Origin(xyz=(0.0, -0.3125, 0.09)),
        material=stainless,
        name="front_strip",
    )
    hood.visual(
        Box((0.92, 0.035, 0.018)),
        origin=Origin(xyz=(0.0, -0.323, 0.032)),
        material=stainless,
        name="straight_lip",
    )
    hood.visual(
        Box((0.64, 0.004, 0.088)),
        origin=Origin(xyz=(0.0, -0.337, 0.105)),
        material=shadow,
        name="control_inset",
    )
    hood.visual(
        Box((0.62, 0.32, 0.010)),
        origin=Origin(xyz=(0.0, -0.04, 0.035)),
        material=shadow,
        name="grease_filter",
    )
    for index, x in enumerate((-0.24, -0.16, -0.08, 0.0, 0.08, 0.16, 0.24)):
        hood.visual(
            Box((0.010, 0.30, 0.004)),
            origin=Origin(xyz=(x, -0.04, 0.041)),
            material=seam,
            name=f"filter_slat_{index}",
        )
    hood.visual(
        Box((0.30, 0.18, 0.80)),
        origin=Origin(xyz=(0.0, 0.02, 0.72)),
        material=stainless,
        name="chimney_cover",
    )
    hood.visual(
        Box((0.004, 0.003, 0.72)),
        origin=Origin(xyz=(0.0, -0.0715, 0.74)),
        material=seam,
        name="chimney_center_seam",
    )
    hood.visual(
        Box((0.29, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.0715, 0.46)),
        material=seam,
        name="chimney_step_seam",
    )

    front_face_y = -0.339
    x_positions = (-0.24, -0.12, 0.0, 0.12, 0.24)
    arc_half_width = 0.24
    base_z = 0.094
    arc_rise = 0.020
    button_travel = 0.008
    for index, x in enumerate(x_positions):
        z = base_z + arc_rise * (1.0 - (x / arc_half_width) ** 2)
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.017, length=0.014),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=button_face,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.014, length=0.0015),
            origin=Origin(xyz=(0.0, -0.01475, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=button_edge,
            name="button_face",
        )
        model.articulation(
            f"hood_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=hood,
            child=button,
            origin=Origin(xyz=(x, front_face_y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                lower=0.0,
                upper=button_travel,
                effort=4.0,
                velocity=0.08,
            ),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hood = object_model.get_part("hood")
    buttons = [object_model.get_part(f"button_{index}") for index in range(5)]
    joints = [object_model.get_articulation(f"hood_to_button_{index}") for index in range(5)]

    ctx.check(
        "only five controls articulate",
        len(object_model.articulations) == 5
        and all(joint.articulation_type == ArticulationType.PRISMATIC for joint in joints),
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )

    for index, (button, joint) in enumerate(zip(buttons, joints)):
        ctx.allow_overlap(
            hood,
            button,
            elem_a="control_inset",
            elem_b="button_cap",
            reason="A real push-button plunger travels inward through a hole in the front control strip.",
        )
        ctx.expect_gap(
            hood,
            button,
            axis="y",
            positive_elem="control_inset",
            negative_elem="button_cap",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"button_{index} rests flush on strip",
        )
        ctx.check(
            f"button_{index} plunges inward",
            joint.axis == (0.0, 1.0, 0.0)
            and joint.motion_limits is not None
            and joint.motion_limits.upper == 0.008,
            details=f"axis={joint.axis}, limits={joint.motion_limits}",
        )

    z_positions = [ctx.part_world_position(button)[2] for button in buttons]
    ctx.check(
        "button centers form gentle arc",
        z_positions[2] > z_positions[1] > z_positions[0]
        and z_positions[2] > z_positions[3] > z_positions[4]
        and abs(z_positions[0] - z_positions[4]) < 1e-6
        and abs(z_positions[1] - z_positions[3]) < 1e-6,
        details=f"button z positions={z_positions}",
    )

    middle_button = buttons[2]
    middle_joint = joints[2]
    rest_position = ctx.part_world_position(middle_button)
    with ctx.pose({middle_joint: 0.008}):
        pressed_position = ctx.part_world_position(middle_button)
        ctx.expect_gap(
            hood,
            middle_button,
            axis="y",
            positive_elem="control_inset",
            negative_elem="button_cap",
            max_penetration=0.010,
            name="pressed button enters control strip",
        )
    ctx.check(
        "pressed button moves toward hood",
        rest_position is not None
        and pressed_position is not None
        and pressed_position[1] > rest_position[1] + 0.007,
        details=f"rest={rest_position}, pressed={pressed_position}",
    )

    return ctx.report()


object_model = build_object_model()
