from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _section_loop(
    width: float,
    depth: float,
    z: float,
    *,
    radius: float,
    x: float = 0.0,
    y: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + px, y + py, z) for px, py in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_kitchen_faucet")

    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.14, 1.0))
    button_finish = model.material("button_finish", rgba=(0.92, 0.94, 0.95, 1.0))

    body = model.part("body")
    body_shell = mesh_from_geometry(
        section_loft(
            [
                _section_loop(0.092, 0.070, 0.006, radius=0.018),
                _section_loop(0.086, 0.066, 0.036, radius=0.017, x=-0.002),
                _section_loop(0.070, 0.058, 0.074, radius=0.014, x=-0.005),
                _section_loop(0.056, 0.050, 0.094, radius=0.012, x=-0.004),
            ]
        ),
        "faucet_body_shell",
    )
    body.visual(
        Cylinder(radius=0.033, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=chrome,
        name="deck_ring",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=chrome,
        name="mount_collar",
    )
    body.visual(body_shell, material=chrome, name="body_shell")
    body.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=chrome,
        name="swivel_base",
    )
    body.visual(
        Box((0.026, 0.020, 0.020)),
        origin=Origin(xyz=(0.004, 0.032, 0.062)),
        material=chrome,
        name="lever_mount",
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=chrome,
        name="swivel_collar",
    )
    gooseneck = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.012),
                (0.0, 0.0, 0.090),
                (0.028, 0.0, 0.190),
                (0.118, 0.0, 0.302),
                (0.212, 0.0, 0.296),
                (0.235, 0.0, 0.212),
                (0.235, 0.0, 0.176),
            ],
            radius=0.0165,
            samples_per_segment=18,
            radial_segments=20,
        ),
        "faucet_gooseneck",
    )
    spout.visual(gooseneck, material=chrome, name="gooseneck")
    spout.visual(
        Cylinder(radius=0.0195, length=0.030),
        origin=Origin(xyz=(0.235, 0.0, 0.186)),
        material=chrome,
        name="socket_collar",
    )
    socket_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.019, 0.0), (0.0185, -0.020), (0.021, -0.058)],
            [(0.0134, -0.001), (0.0134, -0.020), (0.0169, -0.056)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "faucet_socket_shell",
    )
    spout.visual(
        socket_shell,
        origin=Origin(xyz=(0.235, 0.0, 0.176)),
        material=chrome,
        name="socket_shell",
    )

    side_lever = model.part("side_lever")
    side_lever.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_hub",
    )
    side_lever.visual(
        Box((0.008, 0.052, 0.005)),
        origin=Origin(xyz=(0.0, 0.028, 0.010), rpy=(0.34, 0.0, 0.0)),
        material=chrome,
        name="lever_arm",
    )
    side_lever.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(0.0, 0.054, 0.021), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_tip",
    )

    spray_head = model.part("spray_head")
    spray_head.visual(
        Cylinder(radius=0.0122, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=dark_trim,
        name="neck",
    )
    spray_head.visual(
        Cylinder(radius=0.0158, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.076)),
        material=chrome,
        name="head_collar",
    )
    spray_head.visual(
        Cylinder(radius=0.0150, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.112)),
        material=chrome,
        name="head_shell",
    )
    spray_head.visual(
        Cylinder(radius=0.0130, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.141)),
        material=dark_trim,
        name="nozzle_face",
    )
    spray_head.visual(
        Box((0.003, 0.020, 0.024)),
        origin=Origin(xyz=(0.0135, 0.0, -0.110)),
        material=chrome,
        name="button_pad",
    )

    button = model.part("button")
    button.visual(
        Box((0.003, 0.015, 0.008)),
        origin=Origin(xyz=(0.0015, 0.0, 0.0)),
        material=button_finish,
        name="button_cap",
    )

    model.articulation(
        "body_to_spout",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0),
    )
    model.articulation(
        "body_to_side_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_lever,
        origin=Origin(xyz=(0.004, 0.050, 0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-0.25,
            upper=0.75,
        ),
    )
    model.articulation(
        "spout_to_spray_head",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray_head,
        origin=Origin(xyz=(0.235, 0.0, 0.176)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=0.0,
            upper=0.040,
        ),
    )
    model.articulation(
        "spray_head_to_button",
        ArticulationType.PRISMATIC,
        parent=spray_head,
        child=button,
        origin=Origin(xyz=(0.015, 0.0, -0.110)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.03,
            lower=0.0,
            upper=0.0006,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    spout = object_model.get_part("spout")
    spray_head = object_model.get_part("spray_head")
    side_lever = object_model.get_part("side_lever")
    button = object_model.get_part("button")

    spout_joint = object_model.get_articulation("body_to_spout")
    lever_joint = object_model.get_articulation("body_to_side_lever")
    spray_head_joint = object_model.get_articulation("spout_to_spray_head")
    button_joint = object_model.get_articulation("spray_head_to_button")

    ctx.expect_gap(
        spout,
        spray_head,
        axis="z",
        positive_elem="socket_shell",
        negative_elem="head_collar",
        min_gap=0.0,
        max_gap=0.006,
        name="spray head docks just below the socket",
    )
    ctx.expect_overlap(
        spray_head,
        spout,
        axes="xy",
        elem_a="neck",
        elem_b="socket_shell",
        min_overlap=0.023,
        name="spray head neck stays centered in the socket footprint",
    )

    rest_head_pos = ctx.part_world_position(spray_head)
    with ctx.pose({spray_head_joint: 0.040}):
        extended_head_pos = ctx.part_world_position(spray_head)
        ctx.expect_overlap(
            spray_head,
            spout,
            axes="xy",
            elem_a="neck",
            elem_b="socket_shell",
            min_overlap=0.023,
            name="extended spray head stays aligned with the socket",
        )
    ctx.check(
        "spray head extends downward",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[2] < rest_head_pos[2] - 0.03,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    rest_button_pos = ctx.part_world_position(button)
    with ctx.pose({button_joint: 0.0006}):
        pressed_button_pos = ctx.part_world_position(button)
    ctx.check(
        "spray selector button depresses inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[0] < rest_button_pos[0] - 0.0004,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    rest_socket_pos = ctx.part_world_position(spray_head)
    with ctx.pose({spout_joint: math.pi / 2.0}):
        turned_socket_pos = ctx.part_world_position(spray_head)
    ctx.check(
        "spout swivels around the base axis",
        rest_socket_pos is not None
        and turned_socket_pos is not None
        and abs(turned_socket_pos[1]) > 0.18
        and abs(turned_socket_pos[0]) < abs(rest_socket_pos[0]) * 0.35
        and abs(turned_socket_pos[2] - rest_socket_pos[2]) < 0.01,
        details=f"rest={rest_socket_pos}, turned={turned_socket_pos}",
    )

    rest_lever_aabb = ctx.part_world_aabb(side_lever)
    with ctx.pose({lever_joint: 0.75}):
        raised_lever_aabb = ctx.part_world_aabb(side_lever)
    ctx.check(
        "side lever lifts upward",
        rest_lever_aabb is not None
        and raised_lever_aabb is not None
        and raised_lever_aabb[1][2] > rest_lever_aabb[1][2] + 0.02,
        details=f"rest={rest_lever_aabb}, raised={raised_lever_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
