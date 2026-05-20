from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _helix_points(
    *,
    x: float,
    y0: float,
    y1: float,
    z: float,
    radius: float,
    turns: float,
    samples: int,
) -> list[tuple[float, float, float]]:
    """A vending-machine auger coil running from the rear motor plate to the front."""
    pts: list[tuple[float, float, float]] = [(x, y1 + 0.030, z), (x + radius, y1, z)]
    for i in range(samples + 1):
        t = i / samples
        theta = 2.0 * math.pi * turns * t
        y = y1 + (y0 - y1) * t
        pts.append((x + radius * math.cos(theta), y, z + radius * math.sin(theta)))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_candy_vender")

    paint = model.material("red_enamel", rgba=(0.70, 0.03, 0.03, 1.0))
    dark_paint = model.material("dark_recess", rgba=(0.025, 0.025, 0.030, 1.0))
    black = model.material("black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    metal = model.material("brushed_metal", rgba=(0.63, 0.63, 0.58, 1.0))
    chrome = model.material("bright_chrome", rgba=(0.88, 0.88, 0.82, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.65, 0.86, 1.0, 0.34))
    smoked_glass = model.material("smoked_chute_plastic", rgba=(0.08, 0.10, 0.12, 0.52))
    yellow = model.material("yellow_candy", rgba=(1.0, 0.82, 0.04, 1.0))
    blue = model.material("blue_candy", rgba=(0.05, 0.20, 0.85, 1.0))
    green = model.material("green_candy", rgba=(0.03, 0.55, 0.17, 1.0))
    orange = model.material("orange_candy", rgba=(1.0, 0.33, 0.04, 1.0))
    white = model.material("white_label", rgba=(0.94, 0.92, 0.84, 1.0))

    W = 0.66
    D = 0.46
    H = 1.65
    t = 0.035
    front_y = -D / 2.0
    back_y = D / 2.0

    cabinet = model.part("cabinet")

    # Rectangular floor-standing cabinet: side walls, rear wall, top cap and base.
    cabinet.visual(Box((t, D, H)), origin=Origin(xyz=(-W / 2 + t / 2, 0.0, H / 2)), material=paint, name="side_wall_0")
    cabinet.visual(Box((t, D, H)), origin=Origin(xyz=(W / 2 - t / 2, 0.0, H / 2)), material=paint, name="side_wall_1")
    cabinet.visual(Box((W, t, H)), origin=Origin(xyz=(0.0, back_y - t / 2, H / 2)), material=paint, name="rear_wall")
    cabinet.visual(Box((W, D, 0.075)), origin=Origin(xyz=(0.0, 0.0, 0.0375)), material=paint, name="base_plinth")
    cabinet.visual(Box((W, D, 0.055)), origin=Origin(xyz=(0.0, 0.0, H - 0.0275)), material=paint, name="top_cap")

    # Front fascia around the glazed product door and the retrieval chute opening.
    cabinet.visual(Box((W, 0.045, 0.105)), origin=Origin(xyz=(0.0, front_y + 0.0225, 1.592)), material=paint, name="front_header")
    cabinet.visual(Box((0.12, 0.045, 0.36)), origin=Origin(xyz=(-0.255, front_y + 0.0225, 0.245)), material=paint, name="lower_stile_0")
    cabinet.visual(Box((0.12, 0.045, 0.36)), origin=Origin(xyz=(0.255, front_y + 0.0225, 0.245)), material=paint, name="lower_stile_1")
    cabinet.visual(Box((W, 0.045, 0.065)), origin=Origin(xyz=(0.0, front_y + 0.0225, 0.407)), material=paint, name="lower_top_rail")
    cabinet.visual(Box((W, 0.045, 0.065)), origin=Origin(xyz=(0.0, front_y + 0.0225, 0.082)), material=paint, name="lower_bottom_rail")
    cabinet.visual(Box((0.21, 0.046, 0.19)), origin=Origin(xyz=(0.225, front_y + 0.023, 0.250)), material=paint, name="selector_panel")

    # Recessed pickup chute with a sloped tray behind the moving flap.
    cabinet.visual(Box((0.38, 0.18, 0.17)), origin=Origin(xyz=(0.0, front_y + 0.095, 0.225)), material=dark_paint, name="chute_shadow_box")
    cabinet.visual(
        Box((0.36, 0.22, 0.012)),
        origin=Origin(xyz=(0.0, front_y + 0.080, 0.188), rpy=(0.22, 0.0, 0.0)),
        material=metal,
        name="sloped_chute_tray",
    )
    cabinet.visual(Box((0.42, 0.020, 0.020)), origin=Origin(xyz=(0.0, front_y - 0.006, 0.305)), material=chrome, name="chute_top_lip")
    cabinet.visual(Box((0.42, 0.020, 0.020)), origin=Origin(xyz=(0.0, front_y - 0.006, 0.145)), material=chrome, name="chute_bottom_lip")
    cabinet.visual(Box((0.024, 0.050, 0.060)), origin=Origin(xyz=(-0.195, front_y - 0.015, 0.300)), material=chrome, name="flap_hinge_lug_0")
    cabinet.visual(Box((0.024, 0.050, 0.060)), origin=Origin(xyz=(0.195, front_y - 0.015, 0.300)), material=chrome, name="flap_hinge_lug_1")

    # Product shelves, candy bars and auger spirals visible through the glass.
    shelf_levels = (0.615, 0.915, 1.215)
    lane_xs = (-0.18, 0.0, 0.18)
    candy_mats = (yellow, blue, green, orange)
    for si, shelf_z in enumerate(shelf_levels):
        cabinet.visual(Box((W - 0.040, 0.335, 0.018)), origin=Origin(xyz=(0.0, -0.025, shelf_z)), material=metal, name=f"shelf_tray_{si}")
        cabinet.visual(Box((W - 2 * t, 0.018, 0.055)), origin=Origin(xyz=(0.0, front_y + 0.070, shelf_z + 0.036)), material=chrome, name=f"shelf_front_lip_{si}")
        for li, lane_x in enumerate(lane_xs):
            motor_y = back_y - t - 0.015
            axis_z = shelf_z + 0.080
            cabinet.visual(
                Box((0.060, 0.035, 0.060)),
                origin=Origin(xyz=(lane_x, motor_y, axis_z)),
                material=black,
                name=f"auger_motor_{si}_{li}",
            )
            coil = tube_from_spline_points(
                _helix_points(
                    x=lane_x,
                    y0=front_y + 0.100,
                    y1=motor_y - 0.020,
                    z=axis_z,
                    radius=0.034,
                    turns=3.6,
                    samples=80,
                ),
                radius=0.0042,
                samples_per_segment=1,
                radial_segments=12,
                cap_ends=True,
            )
            cabinet.visual(mesh_from_geometry(coil, f"spiral_coil_{si}_{li}"), material=chrome, name=f"spiral_coil_{si}_{li}")
            for pi, y in enumerate((-0.118, -0.050, 0.018)):
                mat = candy_mats[(si + li + pi) % len(candy_mats)]
                cabinet.visual(
                    Box((0.105, 0.042, 0.052)),
                    origin=Origin(xyz=(lane_x, y, shelf_z + 0.035), rpy=(0.0, 0.0, 0.06 * ((pi % 2) * 2 - 1))),
                    material=mat,
                    name=f"candy_pack_{si}_{li}_{pi}",
                )
                cabinet.visual(
                    Box((0.065, 0.044, 0.010)),
                    origin=Origin(xyz=(lane_x, y - 0.001, shelf_z + 0.064), rpy=(0.0, 0.0, 0.06 * ((pi % 2) * 2 - 1))),
                    material=white,
                    name=f"candy_label_{si}_{li}_{pi}",
                )

    # Visible side hinge knuckles fixed to the cabinet side.
    hinge_x = -W / 2 + 0.015
    door_y = front_y - 0.025
    for i, (zc, knuckle_len) in enumerate(((0.590, 0.130), (0.985, 0.280), (1.400, 0.170))):
        cabinet.visual(Cylinder(radius=0.013, length=knuckle_len), origin=Origin(xyz=(hinge_x, door_y, zc)), material=chrome, name=f"cabinet_hinge_knuckle_{i}")
        cabinet.visual(Box((0.040, 0.030, min(0.120, knuckle_len))), origin=Origin(xyz=(hinge_x + 0.010, front_y - 0.008, zc)), material=chrome, name=f"cabinet_hinge_leaf_{i}")

    # Rotary selector support collar on the front panel.
    knob_x = 0.275
    knob_z = 0.312
    knob_mount_y = front_y - 0.018
    cabinet.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(knob_x, knob_mount_y + 0.009, knob_z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=chrome,
        name="knob_collar",
    )
    cabinet.visual(Box((0.095, 0.008, 0.018)), origin=Origin(xyz=(knob_x, front_y - 0.002, knob_z + 0.083)), material=white, name="selector_index")

    # Glazed hinged front door.  The child frame is the hinge line; the panel extends along +X.
    door = model.part("front_door")
    door_h = 1.115
    door_w = W - 2 * t
    door.visual(Box((0.045, 0.024, door_h)), origin=Origin(xyz=(0.060, 0.0, 0.0)), material=paint, name="hinge_stile")
    door.visual(Box((0.045, 0.024, door_h)), origin=Origin(xyz=(door_w - 0.032, 0.0, 0.0)), material=paint, name="pull_stile")
    door.visual(Box((door_w, 0.024, 0.045)), origin=Origin(xyz=(door_w / 2, 0.0, door_h / 2 - 0.0225)), material=paint, name="top_rail")
    door.visual(Box((door_w, 0.024, 0.045)), origin=Origin(xyz=(door_w / 2, 0.0, -door_h / 2 + 0.0225)), material=paint, name="bottom_rail")
    door.visual(Box((door_w - 0.105, 0.006, door_h - 0.135)), origin=Origin(xyz=(door_w / 2, -0.005, 0.015)), material=glass, name="glass_pane")
    door.visual(Box((0.020, 0.035, 0.180)), origin=Origin(xyz=(door_w - 0.050, -0.020, 0.0)), material=chrome, name="vertical_handle")
    for i, zc in enumerate((-0.235, 0.235)):
        door.visual(Cylinder(radius=0.012, length=0.190), origin=Origin(xyz=(0.0, 0.0, zc)), material=chrome, name=f"door_hinge_knuckle_{i}")
        door.visual(Box((0.055, 0.016, 0.070)), origin=Origin(xyz=(0.020, 0.007, zc)), material=chrome, name=f"door_hinge_leaf_{i}")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, door_y, 0.985)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.3, lower=0.0, upper=1.75),
    )

    # Pickup opening flap, hinged horizontally along its top edge.
    flap = model.part("chute_flap")
    flap.visual(Box((0.340, 0.016, 0.145)), origin=Origin(xyz=(0.0, 0.0, -0.073)), material=smoked_glass, name="flap_panel")
    flap.visual(
        Cylinder(radius=0.010, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=chrome,
        name="flap_hinge_tube",
    )
    flap.visual(Box((0.100, 0.012, 0.016)), origin=Origin(xyz=(0.0, -0.014, -0.128)), material=chrome, name="flap_pull_lip")
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(0.0, front_y - 0.032, 0.300)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=0.95),
    )

    # Continuous front product selector knob.
    knob = model.part("selector_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.083,
            0.040,
            body_style="faceted",
            top_diameter=0.066,
            edge_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0012, width=0.0025),
            indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "selector_knob",
    )
    knob.visual(knob_mesh, origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)), material=black, name="knob_cap")
    model.articulation(
        "selector_rotation",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(knob_x, knob_mount_y, knob_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("front_door")
    flap = object_model.get_part("chute_flap")
    knob = object_model.get_part("selector_knob")
    door_hinge = object_model.get_articulation("door_hinge")
    flap_hinge = object_model.get_articulation("flap_hinge")
    selector = object_model.get_articulation("selector_rotation")

    ctx.check("has three primary articulations", len(object_model.articulations) == 3)
    ctx.check("door hinge is revolute", door_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("chute flap hinge is revolute", flap_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("selector knob is continuous", selector.articulation_type == ArticulationType.CONTINUOUS)

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        min_gap=0.004,
        max_gap=0.035,
        positive_elem="front_header",
        negative_elem="top_rail",
        name="glazed door sits just proud of the cabinet face",
    )
    ctx.expect_overlap(door, cabinet, axes="xz", min_overlap=0.45, elem_a="glass_pane", elem_b="rear_wall", name="glass door covers the product bay")
    ctx.expect_gap(
        cabinet,
        knob,
        axis="y",
        min_gap=0.0,
        max_gap=0.018,
        positive_elem="knob_collar",
        negative_elem="knob_cap",
        name="selector knob seats on its front collar",
    )

    def axis_value(vec, idx: int) -> float:
        return float((vec.x, vec.y, vec.z)[idx]) if hasattr(vec, "x") else float(vec[idx])

    closed_pull = ctx.part_element_world_aabb(door, elem="pull_stile")
    closed_flap = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({door_hinge: 1.20, flap_hinge: 0.70, selector: math.pi * 1.5}):
        open_pull = ctx.part_element_world_aabb(door, elem="pull_stile")
        open_flap = ctx.part_element_world_aabb(flap, elem="flap_panel")
        ctx.check(
            "door opens outward from side hinge",
            closed_pull is not None
            and open_pull is not None
            and axis_value(open_pull[0], 1) < axis_value(closed_pull[0], 1) - 0.20,
            details=f"closed={closed_pull}, open={open_pull}",
        )
        ctx.check(
            "chute flap swings out at pickup opening",
            closed_flap is not None
            and open_flap is not None
            and axis_value(open_flap[0], 1) < axis_value(closed_flap[0], 1) - 0.030,
            details=f"closed={closed_flap}, open={open_flap}",
        )

    return ctx.report()


object_model = build_object_model()
