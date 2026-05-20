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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WIDTH = 0.72
DEPTH = 0.62
FRONT_Y = -DEPTH / 2.0
SHELL_SIDE_X = WIDTH / 2.0


def _arched_cabinet_shell():
    """Full-height slot-machine cabinet with a rounded top-box silhouette."""
    base_z = 0.05
    shoulder_z = 1.39
    top_z = 1.78
    half_w = WIDTH / 2.0

    return (
        cq.Workplane("XZ")
        .moveTo(-half_w, base_z)
        .lineTo(-half_w, shoulder_z)
        .threePointArc((0.0, top_z), (half_w, shoulder_z))
        .lineTo(half_w, base_z)
        .close()
        .extrude(DEPTH)
        .translate((0.0, DEPTH / 2.0, 0.0))
        .edges("|Y")
        .fillet(0.018)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_reel_slot_machine")

    cabinet_red = model.material("cabinet_red", rgba=(0.56, 0.03, 0.035, 1.0))
    black = model.material("black_enamel", rgba=(0.01, 0.01, 0.012, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_chrome = model.material("dark_chrome", rgba=(0.18, 0.18, 0.18, 1.0))
    warm_light = model.material("warm_marquee", rgba=(1.0, 0.74, 0.18, 1.0))
    cream = model.material("reel_ivory", rgba=(0.95, 0.91, 0.78, 1.0))
    glass = model.material("smoked_glass", rgba=(0.34, 0.62, 0.82, 0.38))
    service_blue = model.material("service_door_blue", rgba=(0.08, 0.18, 0.27, 1.0))
    red_ball = model.material("red_handle_ball", rgba=(0.86, 0.02, 0.015, 1.0))
    cherry = model.material("cherry_red", rgba=(0.85, 0.02, 0.06, 1.0))
    lemon = model.material("lemon_yellow", rgba=(0.95, 0.82, 0.05, 1.0))
    green = model.material("lucky_green", rgba=(0.05, 0.62, 0.16, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_arched_cabinet_shell(), "arched_cabinet_shell"),
        material=cabinet_red,
        name="arched_shell",
    )
    cabinet.visual(
        Box((0.78, 0.68, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=black,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.54, 0.024, 0.20)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.011, 1.48)),
        material=warm_light,
        name="top_marquee",
    )
    cabinet.visual(
        Box((0.62, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.009, 1.31)),
        material=chrome,
        name="upper_trim",
    )
    cabinet.visual(
        Box((0.64, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.009, 0.835)),
        material=chrome,
        name="lower_trim",
    )
    cabinet.visual(
        Box((0.026, 0.020, 0.58)),
        origin=Origin(xyz=(-0.316, FRONT_Y - 0.010, 0.50)),
        material=chrome,
        name="door_hinge_jamb",
    )
    cabinet.visual(
        Cylinder(radius=0.006, length=0.56),
        origin=Origin(xyz=(-0.285, FRONT_Y - 0.019, 0.49)),
        material=dark_chrome,
        name="door_hinge_pin",
    )
    for i, (z, length) in enumerate(((0.235, 0.030), (0.402, 0.040), (0.577, 0.040), (0.745, 0.030))):
        cabinet.visual(
            Cylinder(radius=0.018, length=length),
            origin=Origin(xyz=(-0.285, FRONT_Y - 0.019, z)),
            material=chrome,
            name=f"cabinet_hinge_knuckle_{i}",
        )
    cabinet.visual(
        Cylinder(radius=0.095, length=0.026),
        origin=Origin(
            xyz=(SHELL_SIDE_X + 0.011, -0.04, 0.88),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="handle_side_boss",
    )

    reel_window = model.part("reel_window")
    reel_window.visual(
        Box((0.60, 0.040, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=black,
        name="window_top_frame",
    )
    reel_window.visual(
        Box((0.60, 0.040, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        material=black,
        name="window_bottom_frame",
    )
    reel_window.visual(
        Box((0.050, 0.040, 0.36)),
        origin=Origin(xyz=(-0.275, 0.0, 0.0)),
        material=black,
        name="window_side_frame_0",
    )
    reel_window.visual(
        Box((0.050, 0.040, 0.36)),
        origin=Origin(xyz=(0.275, 0.0, 0.0)),
        material=black,
        name="window_side_frame_1",
    )
    reel_window.visual(
        Box((0.51, 0.010, 0.235)),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=cream,
        name="reel_backing",
    )
    reel_window.visual(
        Box((0.515, 0.006, 0.240)),
        origin=Origin(xyz=(0.0, -0.023, 0.0)),
        material=glass,
        name="window_glass",
    )
    for i, x in enumerate((-0.16, 0.0, 0.16)):
        reel_window.visual(
            Box((0.130, 0.008, 0.205)),
            origin=Origin(xyz=(x, -0.029, 0.0)),
            material=cream,
            name=f"reel_strip_{i}",
        )
    for i, x in enumerate((-0.08, 0.08)):
        reel_window.visual(
            Box((0.014, 0.014, 0.225)),
            origin=Origin(xyz=(x, -0.032, 0.0)),
            material=black,
            name=f"reel_divider_{i}",
        )
    symbol_specs: tuple[tuple[float, float, Material, str], ...] = (
        (-0.16, 0.055, cherry, "symbol_cherry"),
        (0.0, 0.000, lemon, "symbol_lemon"),
        (0.16, -0.055, green, "symbol_seven"),
        (-0.16, -0.065, lemon, "symbol_bell"),
        (0.0, 0.065, cherry, "symbol_bar"),
        (0.16, 0.055, lemon, "symbol_star"),
    )
    for x, z, mat, name in symbol_specs:
        reel_window.visual(
            Box((0.070, 0.010, 0.040)),
            origin=Origin(xyz=(x, -0.037, z)),
            material=mat,
            name=name,
        )
    model.articulation(
        "cabinet_to_reel_window",
        ArticulationType.FIXED,
        parent=cabinet,
        child=reel_window,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.020, 1.06)),
    )

    door = model.part("service_door")
    door_width = 0.57
    panel_start = 0.025
    panel_width = door_width - panel_start
    door_height = 0.54
    door.visual(
        Box((panel_width, 0.030, door_height)),
        origin=Origin(xyz=(panel_start + panel_width / 2.0, 0.0, door_height / 2.0)),
        material=service_blue,
        name="door_panel",
    )
    door.visual(
        Box((panel_width - 0.065, 0.012, 0.030)),
        origin=Origin(xyz=(panel_start + panel_width / 2.0, -0.021, door_height - 0.040)),
        material=chrome,
        name="door_top_rail",
    )
    door.visual(
        Box((panel_width - 0.065, 0.012, 0.030)),
        origin=Origin(xyz=(panel_start + panel_width / 2.0, -0.021, 0.040)),
        material=chrome,
        name="door_bottom_rail",
    )
    door.visual(
        Box((0.030, 0.012, door_height - 0.085)),
        origin=Origin(xyz=(panel_start + 0.055, -0.021, door_height / 2.0)),
        material=chrome,
        name="door_side_rail_0",
    )
    door.visual(
        Box((0.030, 0.012, door_height - 0.085)),
        origin=Origin(xyz=(panel_start + panel_width - 0.055, -0.021, door_height / 2.0)),
        material=chrome,
        name="door_side_rail_1",
    )
    for i, z in enumerate((0.145, 0.185, 0.225)):
        door.visual(
            Box((0.30, 0.010, 0.012)),
            origin=Origin(xyz=(panel_start + panel_width / 2.0, -0.020, z)),
            material=black,
            name=f"door_louver_{i}",
        )
    door.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(
            xyz=(panel_start + panel_width - 0.125, -0.024, 0.310),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="lock_core",
    )
    door.visual(
        Box((0.006, 0.004, 0.034)),
        origin=Origin(xyz=(panel_start + panel_width - 0.125, -0.033, 0.310)),
        material=black,
        name="key_slot",
    )
    for i, z in enumerate((0.095, 0.270, 0.445)):
        door.visual(
            Box((0.035, 0.012, 0.090)),
            origin=Origin(xyz=(0.032, 0.0, z)),
            material=chrome,
            name=f"door_hinge_leaf_{i}",
        )
        door.visual(
            Cylinder(radius=0.018, length=0.125),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=chrome,
            name=f"door_hinge_barrel_{i}",
        )
    door_hinge = model.articulation(
        "cabinet_to_service_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.285, FRONT_Y - 0.019, 0.22)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.65, effort=12.0, velocity=1.2),
    )

    handle = model.part("pull_handle")
    handle.visual(
        Cylinder(radius=0.055, length=0.080),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.54),
        origin=Origin(xyz=(0.040, 0.0, 0.270)),
        material=chrome,
        name="handle_stem",
    )
    handle.visual(
        Sphere(radius=0.075),
        origin=Origin(xyz=(0.040, 0.0, 0.575)),
        material=red_ball,
        name="handle_ball",
    )
    handle_pivot = model.articulation(
        "cabinet_to_pull_handle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=handle,
        origin=Origin(xyz=(SHELL_SIDE_X + 0.064, -0.04, 0.88)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=18.0, velocity=1.6),
    )

    # Keep local names alive for static analyzers without changing the model.
    _ = (door_hinge, handle_pivot, dark_chrome)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    reel_window = object_model.get_part("reel_window")
    door = object_model.get_part("service_door")
    handle = object_model.get_part("pull_handle")
    door_hinge = object_model.get_articulation("cabinet_to_service_door")
    handle_pivot = object_model.get_articulation("cabinet_to_pull_handle")

    for i in range(3):
        barrel = f"door_hinge_barrel_{i}"
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a="door_hinge_pin",
            elem_b=barrel,
            reason="The fixed cabinet hinge pin is intentionally captured inside the service-door hinge barrel.",
        )
        ctx.expect_within(
            cabinet,
            door,
            axes="xy",
            inner_elem="door_hinge_pin",
            outer_elem=barrel,
            margin=0.001,
            name=f"hinge pin stays centered in barrel {i}",
        )
        ctx.expect_overlap(
            cabinet,
            door,
            axes="z",
            elem_a="door_hinge_pin",
            elem_b=barrel,
            min_overlap=0.10,
            name=f"hinge pin passes through barrel {i}",
        )

    ctx.expect_gap(
        cabinet,
        reel_window,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="arched_shell",
        name="reel window sits proud of cabinet without merging",
    )
    ctx.expect_overlap(
        reel_window,
        cabinet,
        axes="xz",
        min_overlap=0.30,
        name="central reel window is mounted on the cabinet front",
    )
    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        min_gap=0.002,
        max_gap=0.010,
        positive_elem="arched_shell",
        negative_elem="door_panel",
        name="service door remains visibly separate from shell",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.50,
        elem_a="door_panel",
        elem_b="arched_shell",
        name="service door covers the lower service bay",
    )

    def coord(vec, index: int) -> float:
        if hasattr(vec, "__getitem__"):
            return float(vec[index])
        return float((vec.x, vec.y, vec.z)[index])

    closed_door = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.10}):
        open_door = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "service door swings outward on vertical hinge",
        closed_door is not None
        and open_door is not None
        and coord(open_door[0], 1) < coord(closed_door[0], 1) - 0.20,
        details=f"closed={closed_door}, open={open_door}",
    )

    rest_ball = ctx.part_element_world_aabb(handle, elem="handle_ball")
    with ctx.pose({handle_pivot: 1.10}):
        pulled_ball = ctx.part_element_world_aabb(handle, elem="handle_ball")
    ctx.check(
        "pull handle pivots down and forward",
        rest_ball is not None
        and pulled_ball is not None
        and coord(pulled_ball[1], 2) < coord(rest_ball[1], 2) - 0.18
        and coord(pulled_ball[0], 1) < coord(rest_ball[0], 1) - 0.20,
        details=f"rest={rest_ball}, pulled={pulled_ball}",
    )

    return ctx.report()


object_model = build_object_model()
