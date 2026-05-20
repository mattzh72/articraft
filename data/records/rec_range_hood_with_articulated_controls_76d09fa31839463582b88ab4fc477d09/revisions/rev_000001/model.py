from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOOD_WIDTH = 0.90
RAIL_DEPTH = 0.095
RAIL_HEIGHT = 0.046
SPINE_WIDTH = 0.34
SPINE_DEPTH = 0.405
SPINE_HEIGHT = 0.100
CHIMNEY_WIDTH = 0.32
CHIMNEY_DEPTH = 0.280
CHIMNEY_HEIGHT = 0.720
MOUNT_WIDTH = 0.84
MOUNT_DEPTH = 0.030
MOUNT_HEIGHT = 0.018
GLASS_WIDTH = 0.84
GLASS_TRAVEL_OPEN = math.radians(68.0)
BUTTON_RADIUS = 0.0105
BUTTON_TRAVEL = 0.0010
BUTTON_Y = 0.455
BUTTON_XS = (-0.090, -0.030, 0.030, 0.090)


def _glass_panel_shape():
    profile_points = [
        (0.000, -0.004),
        (0.018, -0.010),
        (0.042, -0.028),
        (0.073, -0.066),
        (0.104, -0.118),
        (0.129, -0.178),
        (0.145, -0.235),
        (0.138, -0.228),
        (0.121, -0.173),
        (0.097, -0.121),
        (0.067, -0.072),
        (0.036, -0.032),
        (0.010, -0.010),
    ]
    return (
        cq.Workplane("YZ")
        .polyline(profile_points)
        .close()
        .extrude(GLASS_WIDTH * 0.5, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_glass_range_hood")

    stainless = model.material("stainless", rgba=(0.79, 0.80, 0.79, 1.0))
    brushed_dark = model.material("brushed_dark", rgba=(0.32, 0.33, 0.34, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.74, 0.82, 0.88, 0.34))
    button_finish = model.material("button_finish", rgba=(0.12, 0.12, 0.13, 1.0))
    button_ring = model.material("button_ring", rgba=(0.55, 0.56, 0.57, 1.0))

    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((HOOD_WIDTH, RAIL_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.4525, RAIL_HEIGHT * 0.5)),
        material=stainless,
        name="lower_panel",
    )
    hood_body.visual(
        Box((0.64, 0.210, 0.010)),
        origin=Origin(xyz=(0.0, 0.300, 0.005)),
        material=brushed_dark,
        name="underside_tray",
    )
    hood_body.visual(
        Box((SPINE_WIDTH, SPINE_DEPTH, SPINE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                SPINE_DEPTH * 0.5,
                RAIL_HEIGHT + SPINE_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="spine_shell",
    )
    hood_body.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CHIMNEY_DEPTH * 0.5,
                RAIL_HEIGHT + SPINE_HEIGHT + CHIMNEY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="chimney_shell",
    )
    hood_body.visual(
        Box((MOUNT_WIDTH, MOUNT_DEPTH, MOUNT_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.500 - (MOUNT_DEPTH * 0.5),
                RAIL_HEIGHT - 0.017,
            )
        ),
        material=stainless,
        name="glass_mount",
    )

    glass_shield = model.part("glass_shield")
    glass_shield.visual(
        mesh_from_cadquery(_glass_panel_shape(), "glass_shield_panel"),
        material=smoked_glass,
        name="glass_panel",
    )
    glass_shield.visual(
        Box((GLASS_WIDTH, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, 0.010, -0.010)),
        material=stainless,
        name="glass_rail",
    )
    glass_shield.visual(
        Cylinder(radius=0.006, length=GLASS_WIDTH * 0.97),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=stainless,
        name="glass_barrel",
    )

    model.articulation(
        "body_to_glass",
        ArticulationType.REVOLUTE,
        parent=hood_body,
        child=glass_shield,
        origin=Origin(
            xyz=(
                0.0,
                0.500,
                -0.006,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=GLASS_TRAVEL_OPEN,
        ),
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=BUTTON_RADIUS + 0.0025, length=0.0030),
            origin=Origin(xyz=(0.0, 0.0, -0.0015)),
            material=button_ring,
            name="button_bezel",
        )
        button.visual(
            Cylinder(radius=BUTTON_RADIUS, length=0.0055),
            origin=Origin(xyz=(0.0, 0.0, -0.00575)),
            material=button_finish,
            name="button_cap",
        )

        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(button_x, BUTTON_Y, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.03,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hood_body = object_model.get_part("hood_body")
    glass_shield = object_model.get_part("glass_shield")
    glass_joint = object_model.get_articulation("body_to_glass")

    ctx.expect_overlap(
        glass_shield,
        hood_body,
        axes="x",
        elem_a="glass_panel",
        elem_b="glass_mount",
        min_overlap=0.72,
        name="glass shield spans the mounting width",
    )
    ctx.expect_gap(
        hood_body,
        glass_shield,
        axis="z",
        positive_elem="glass_mount",
        negative_elem="glass_panel",
        min_gap=0.001,
        max_gap=0.030,
        name="glass shield hangs just below the top mounting strip",
    )

    closed_glass_aabb = ctx.part_element_world_aabb(glass_shield, elem="glass_panel")
    with ctx.pose({glass_joint: GLASS_TRAVEL_OPEN}):
        open_glass_aabb = ctx.part_element_world_aabb(glass_shield, elem="glass_panel")

    glass_lifts = (
        closed_glass_aabb is not None
        and open_glass_aabb is not None
        and open_glass_aabb[1][2] > closed_glass_aabb[1][2] + 0.12
        and open_glass_aabb[0][2] > closed_glass_aabb[0][2] + 0.05
    )
    ctx.check(
        "glass shield lifts upward on its hinge",
        glass_lifts,
        details=f"closed={closed_glass_aabb!r}, open={open_glass_aabb!r}",
    )

    button_top_z: list[float | None] = []
    button_parts = []
    button_joints = []
    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"body_to_button_{index}")
        button_parts.append(button)
        button_joints.append(joint)

        ctx.expect_gap(
            hood_body,
            button,
            axis="z",
            positive_elem="lower_panel",
            negative_elem="button_cap",
            min_gap=0.0020,
            max_gap=0.0040,
            name=f"button {index} rests just below the lower panel",
        )
        ctx.expect_overlap(
            button,
            hood_body,
            axes="xy",
            elem_a="button_cap",
            elem_b="lower_panel",
            min_overlap=0.018,
            name=f"button {index} stays under the control strip footprint",
        )

        button_aabb = ctx.part_element_world_aabb(button, elem="button_cap")
        button_top_z.append(None if button_aabb is None else float(button_aabb[1][2]))

    for index, joint in enumerate(button_joints):
        with ctx.pose({joint: BUTTON_TRAVEL}):
            moved_aabb = ctx.part_element_world_aabb(button_parts[index], elem="button_cap")
            neighbor_index = (index + 1) % 4
            neighbor_aabb = ctx.part_element_world_aabb(button_parts[neighbor_index], elem="button_cap")

        moved_top = None if moved_aabb is None else float(moved_aabb[1][2])
        neighbor_top = None if neighbor_aabb is None else float(neighbor_aabb[1][2])

        ctx.check(
            f"button {index} presses upward",
            button_top_z[index] is not None
            and moved_top is not None
            and moved_top > button_top_z[index] + 0.0006,
            details=f"rest={button_top_z[index]!r}, moved={moved_top!r}",
        )
        ctx.check(
            f"button {index} moves independently",
            button_top_z[neighbor_index] is not None
            and neighbor_top is not None
            and abs(neighbor_top - button_top_z[neighbor_index]) < 1e-6,
            details=(
                f"neighbor_rest={button_top_z[neighbor_index]!r}, "
                f"neighbor_during_press={neighbor_top!r}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
