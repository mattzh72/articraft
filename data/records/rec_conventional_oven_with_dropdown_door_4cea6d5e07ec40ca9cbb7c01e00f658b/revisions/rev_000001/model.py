from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_tunnel_oven")

    stainless = model.material("brushed_stainless", rgba=(0.63, 0.66, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    hot_shadow = model.material("dark_hot_chamber", rgba=(0.03, 0.025, 0.020, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.08, 1.0))
    red = model.material("red_indicator", rgba=(0.80, 0.05, 0.03, 1.0))

    frame = model.part("oven_frame")

    # Overall industrial proportions: a long insulated tunnel chamber raised on legs.
    length = 3.20
    depth = 0.90
    chamber_bottom = 0.72
    chamber_height = 0.92
    chamber_top = chamber_bottom + chamber_height
    wall = 0.06
    front_y = -depth / 2.0 + wall / 2.0
    rear_y = depth / 2.0 - wall / 2.0

    # Hollow rectangular tunnel housing, built as connected sheet-metal panels.
    frame.visual(
        Box((length, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, chamber_top - wall / 2.0)),
        material=stainless,
        name="top_panel",
    )
    frame.visual(
        Box((length, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, chamber_bottom + wall / 2.0)),
        material=stainless,
        name="bottom_panel",
    )
    frame.visual(
        Box((length, wall, chamber_height)),
        origin=Origin(xyz=(0.0, rear_y, chamber_bottom + chamber_height / 2.0)),
        material=stainless,
        name="rear_wall",
    )
    frame.visual(
        Box((length, wall, 0.16)),
        origin=Origin(xyz=(0.0, front_y, chamber_top - wall - 0.08)),
        material=stainless,
        name="front_lintel",
    )
    frame.visual(
        Box((length, wall, 0.13)),
        origin=Origin(xyz=(0.0, front_y, chamber_bottom + wall + 0.065)),
        material=stainless,
        name="front_sill",
    )
    for x, name in [(-0.66, "front_jamb_0"), (0.66, "front_jamb_1")]:
        frame.visual(
            Box((0.08, wall, 0.58)),
            origin=Origin(xyz=(x, front_y, 1.145)),
            material=stainless,
            name=name,
        )

    # Dark recessed chamber visible when the sliding door is moved aside.
    frame.visual(
        Box((1.22, 0.018, 0.52)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.008, 1.155)),
        material=hot_shadow,
        name="front_opening_shadow",
    )

    # Four legs and cross bracing, all touching the chamber bottom for a supported assembly.
    leg_size = 0.08
    for x in (-1.38, 1.38):
        for y in (-0.34, 0.34):
            frame.visual(
                Box((leg_size, leg_size, chamber_bottom)),
                origin=Origin(xyz=(x, y, chamber_bottom / 2.0)),
                material=dark_steel,
                name=f"leg_{x:+.1f}_{y:+.1f}".replace("+", "p").replace("-", "m").replace(".", "_"),
            )
    for y, name in [(-0.34, "front_lower_rail"), (0.34, "rear_lower_rail")]:
        frame.visual(
            Box((2.84, 0.055, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.28)),
            material=dark_steel,
            name=name,
        )
    for x, name in [(-1.38, "entry_lower_rail"), (1.38, "exit_lower_rail")]:
        frame.visual(
            Box((0.055, 0.72, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.28)),
            material=dark_steel,
            name=name,
        )

    # Front horizontal sliding-door guide rails and stand-off brackets.
    rail_y = -0.57
    rail_center_x = 0.33
    rail_length = 2.45
    frame.visual(
        Box((rail_length, 0.035, 0.035)),
        origin=Origin(xyz=(rail_center_x, rail_y, 1.535)),
        material=dark_steel,
        name="upper_guide_rail",
    )
    frame.visual(
        Box((rail_length, 0.035, 0.035)),
        origin=Origin(xyz=(rail_center_x, rail_y, 0.805)),
        material=dark_steel,
        name="lower_guide_rail",
    )
    for z, name in [(1.535, "upper_guide_rail"), (0.805, "lower_guide_rail")]:
        for x in (-0.80, 0.05, 0.88, 1.48):
            frame.visual(
                Box((0.055, 0.1025, 0.070)),
                origin=Origin(xyz=(x, -0.50125, z)),
                material=dark_steel,
                name=f"{name}_bracket_{x:+.1f}".replace("+", "p").replace("-", "m").replace(".", "_"),
            )

    # Conveyor belt and side rails inside the heated tunnel.
    frame.visual(
        Box((2.88, 0.575, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.9195)),
        material=black_rubber,
        name="top_belt_run",
    )
    frame.visual(
        Box((2.78, 0.575, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.785)),
        material=black_rubber,
        name="return_belt_run",
    )
    for y, name in [(-0.31, "front_conveyor_rail"), (0.31, "rear_conveyor_rail")]:
        frame.visual(
            Box((2.95, 0.045, 0.140)),
            origin=Origin(xyz=(0.0, y, 0.855)),
            material=dark_steel,
            name=name,
        )
    for x in (-1.40, -0.95, 0.95, 1.40):
        for y in (-0.31, 0.31):
            frame.visual(
                Box((0.050, 0.050, 0.130)),
                origin=Origin(xyz=(x, y, 0.845)),
                material=dark_steel,
                name=f"conveyor_post_{x:+.1f}_{y:+.1f}".replace("+", "p").replace("-", "m").replace(".", "_"),
            )
    for x, name in [(-1.46, "entry_belt_nose"), (1.46, "exit_belt_nose")]:
        frame.visual(
            Cylinder(radius=0.058, length=0.56),
            origin=Origin(xyz=(x, 0.0, 0.855), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=name,
        )

    # A few indicator lamps are fixed to the front lintel; they read as controls
    # without inventing an extra control-panel mechanism for this prompt.
    for x, mat, name in [(-1.24, red, "red_status_lamp"), (-1.12, safety_yellow, "yellow_status_lamp")]:
        frame.visual(
            Cylinder(radius=0.035, length=0.025),
            origin=Origin(xyz=(x, -0.4625, 1.505), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mat,
            name=name,
        )

    # Wide front sliding door: child frame is at the closed panel center.
    door = model.part("front_door")
    door.visual(
        Box((1.18, 0.045, 0.62)),
        origin=Origin(),
        material=stainless,
        name="insulated_panel",
    )
    door.visual(
        Box((1.24, 0.026, 0.055)),
        origin=Origin(xyz=(0.0, -0.032, 0.320)),
        material=dark_steel,
        name="top_trim",
    )
    door.visual(
        Box((1.24, 0.026, 0.055)),
        origin=Origin(xyz=(0.0, -0.032, -0.305)),
        material=dark_steel,
        name="bottom_trim",
    )
    for x, name in [(-0.62, "side_trim_0"), (0.62, "side_trim_1")]:
        door.visual(
            Box((0.055, 0.026, 0.64)),
            origin=Origin(xyz=(x, -0.032, 0.0)),
            material=dark_steel,
            name=name,
        )
    for x, name in [(-0.36, "hanger_0"), (0.36, "hanger_1")]:
        door.visual(
            Box((0.075, 0.030, 0.16)),
            origin=Origin(xyz=(x, 0.0, 0.390)),
            material=dark_steel,
            name=name,
        )
        door.visual(
            Cylinder(radius=0.034, length=0.040),
            origin=Origin(xyz=(x, -0.035, 0.4315), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"hanger_wheel_{name[-1]}",
        )
    # Fixed pull handle mounted on stand-offs, connected to the door panel.
    door.visual(
        Cylinder(radius=0.022, length=0.36),
        origin=Origin(xyz=(-0.47, -0.090, 0.0)),
        material=dark_steel,
        name="pull_handle",
    )
    for z, name in [(-0.14, "handle_standoff_0"), (0.14, "handle_standoff_1")]:
        door.visual(
            Box((0.055, 0.085, 0.045)),
            origin=Origin(xyz=(-0.47, -0.056, z)),
            material=dark_steel,
            name=name,
        )

    model.articulation(
        "frame_to_door",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door,
        origin=Origin(xyz=(0.0, -0.535, 1.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.74, effort=500.0, velocity=0.35),
    )

    # Continuously rotating conveyor rollers inside the tunnel.
    roller_xs = [-1.25, -0.65, 0.0, 0.65, 1.25]
    for index, x in enumerate(roller_xs):
        roller = model.part(f"roller_{index}")
        roller.visual(
            Cylinder(radius=0.052, length=0.56),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name="roller_shell",
        )
        roller.visual(
            Cylinder(radius=0.016, length=0.66),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="axle_stub",
        )
        model.articulation(
            f"frame_to_roller_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=roller,
            origin=Origin(xyz=(x, 0.0, 0.855)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("oven_frame")
    door = object_model.get_part("front_door")
    door_joint = object_model.get_articulation("frame_to_door")

    ctx.expect_gap(
        frame,
        door,
        axis="y",
        positive_elem="front_lintel",
        negative_elem="insulated_panel",
        min_gap=0.030,
        name="door rides proud of front face",
    )
    ctx.expect_overlap(
        door,
        frame,
        axes="z",
        elem_a="insulated_panel",
        elem_b="front_opening_shadow",
        min_overlap=0.45,
        name="closed door covers tall front opening",
    )
    ctx.expect_overlap(
        door,
        frame,
        axes="x",
        elem_a="insulated_panel",
        elem_b="front_opening_shadow",
        min_overlap=1.00,
        name="closed door spans front opening",
    )

    closed_position = ctx.part_world_position(door)
    with ctx.pose({door_joint: 0.74}):
        open_position = ctx.part_world_position(door)
        ctx.expect_overlap(
            door,
            frame,
            axes="x",
            elem_a="insulated_panel",
            elem_b="upper_guide_rail",
            min_overlap=0.70,
            name="open door remains carried by rail",
        )

    ctx.check(
        "door slides sideways on horizontal rails",
        closed_position is not None
        and open_position is not None
        and open_position[0] > closed_position[0] + 0.70
        and abs(open_position[2] - closed_position[2]) < 0.001,
        details=f"closed={closed_position}, open={open_position}",
    )

    for index in range(5):
        roller = object_model.get_part(f"roller_{index}")
        joint = object_model.get_articulation(f"frame_to_roller_{index}")
        for rail_name in ("front_conveyor_rail", "rear_conveyor_rail"):
            ctx.allow_overlap(
                frame,
                roller,
                elem_a=rail_name,
                elem_b="axle_stub",
                reason="The roller axle is intentionally captured in the conveyor side-rail bearing block.",
            )
            ctx.expect_overlap(
                frame,
                roller,
                axes="y",
                elem_a=rail_name,
                elem_b="axle_stub",
                min_overlap=0.015,
                name=f"roller_{index} axle is retained in {rail_name}",
            )
        ctx.expect_overlap(
            roller,
            frame,
            axes="y",
            elem_a="roller_shell",
            elem_b="top_belt_run",
            min_overlap=0.48,
            name=f"roller_{index} spans conveyor belt width",
        )
        before = ctx.part_world_position(roller)
        with ctx.pose({joint: math.pi}):
            after = ctx.part_world_position(roller)
        ctx.check(
            f"roller_{index} is continuous at fixed axle",
            before is not None and after is not None and all(abs(a - b) < 1e-6 for a, b in zip(before, after)),
            details=f"before={before}, after={after}",
        )

    return ctx.report()


object_model = build_object_model()
