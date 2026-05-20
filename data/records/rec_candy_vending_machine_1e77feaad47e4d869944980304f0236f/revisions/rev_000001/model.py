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


def _helix_points(
    *,
    x: float,
    z: float,
    y_front: float,
    y_back: float,
    radius: float,
    turns: float,
    phase: float = 0.0,
    samples: int = 96,
) -> list[tuple[float, float, float]]:
    """Depth-running vending coil with a short rear tang into the drive wall."""
    pts: list[tuple[float, float, float]] = [(x + radius, y_back + 0.018, z)]
    for i in range(samples + 1):
        t = i / samples
        a = phase + turns * 2.0 * math.pi * t
        y = y_back + (y_front - y_back) * t
        pts.append((x + radius * math.cos(a), y, z + radius * math.sin(a)))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_tower_candy_vender")

    model.material("charcoal_powdercoat", rgba=(0.03, 0.035, 0.04, 1.0))
    model.material("black_trim", rgba=(0.005, 0.005, 0.006, 1.0))
    model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    model.material("dark_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    model.material("glass_blue", rgba=(0.55, 0.80, 0.95, 0.34))
    model.material("smoked_plastic", rgba=(0.10, 0.13, 0.15, 0.55))
    model.material("control_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("selector_orange", rgba=(1.0, 0.48, 0.08, 1.0))
    model.material("candy_red", rgba=(0.95, 0.05, 0.05, 1.0))
    model.material("candy_yellow", rgba=(1.0, 0.82, 0.04, 1.0))
    model.material("candy_blue", rgba=(0.04, 0.25, 0.95, 1.0))
    model.material("candy_green", rgba=(0.04, 0.70, 0.18, 1.0))
    model.material("white_label", rgba=(0.95, 0.94, 0.86, 1.0))

    cabinet = model.part("cabinet")

    # Tall narrow enclosure: a dark steel tower with an open glass vending bay.
    cabinet.visual(
        Box((0.035, 0.36, 1.60)),
        origin=Origin(xyz=(-0.1925, 0.0, 0.80)),
        material="charcoal_powdercoat",
        name="left_side",
    )
    cabinet.visual(
        Box((0.035, 0.36, 1.60)),
        origin=Origin(xyz=(0.1925, 0.0, 0.80)),
        material="charcoal_powdercoat",
        name="right_side",
    )
    cabinet.visual(
        Box((0.42, 0.025, 1.60)),
        origin=Origin(xyz=(0.0, 0.1675, 0.80)),
        material="charcoal_powdercoat",
        name="rear_wall",
    )
    cabinet.visual(
        Box((0.42, 0.36, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 1.5775)),
        material="charcoal_powdercoat",
        name="top_cap",
    )
    cabinet.visual(
        Box((0.42, 0.36, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="charcoal_powdercoat",
        name="bottom_plinth",
    )
    cabinet.visual(
        Box((0.42, 0.36, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material="charcoal_powdercoat",
        name="pickup_floor",
    )

    # Glass front and its surrounding front frame.
    cabinet.visual(
        Box((0.27, 0.006, 0.86)),
        origin=Origin(xyz=(-0.060, -0.183, 1.060)),
        material="glass_blue",
        name="glass_front",
    )
    cabinet.visual(
        Box((0.030, 0.035, 0.96)),
        origin=Origin(xyz=(0.075, -0.181, 1.065)),
        material="black_trim",
        name="glass_mullion",
    )
    cabinet.visual(
        Box((0.31, 0.035, 0.030)),
        origin=Origin(xyz=(-0.060, -0.181, 1.510)),
        material="black_trim",
        name="glass_top_rail",
    )
    cabinet.visual(
        Box((0.31, 0.035, 0.030)),
        origin=Origin(xyz=(-0.060, -0.181, 0.610)),
        material="black_trim",
        name="glass_bottom_rail",
    )

    # Mid-height control panel with fixed payment details; the selector dial is
    # a separate continuously rotating child part.
    cabinet.visual(
        Box((0.120, 0.026, 0.360)),
        origin=Origin(xyz=(0.140, -0.195, 0.850)),
        material="control_graphite",
        name="control_panel",
    )
    cabinet.visual(
        Box((0.074, 0.004, 0.020)),
        origin=Origin(xyz=(0.140, -0.210, 0.990)),
        material="black_trim",
        name="coin_slot",
    )
    cabinet.visual(
        Box((0.072, 0.004, 0.060)),
        origin=Origin(xyz=(0.140, -0.210, 0.730)),
        material="black_trim",
        name="card_reader",
    )
    cabinet.visual(
        Box((0.050, 0.004, 0.022)),
        origin=Origin(xyz=(0.140, -0.2095, 0.680)),
        material="white_label",
        name="price_label",
    )

    # Pickup opening frame and hinge pin for the rotating retrieval flap.
    cabinet.visual(
        Box((0.300, 0.030, 0.024)),
        origin=Origin(xyz=(-0.040, -0.185, 0.456)),
        material="black_trim",
        name="pickup_top_rail",
    )
    cabinet.visual(
        Box((0.300, 0.030, 0.024)),
        origin=Origin(xyz=(-0.040, -0.185, 0.286)),
        material="black_trim",
        name="pickup_bottom_rail",
    )
    cabinet.visual(
        Box((0.024, 0.030, 0.150)),
        origin=Origin(xyz=(-0.202, -0.185, 0.375)),
        material="black_trim",
        name="pickup_side_0",
    )
    cabinet.visual(
        Box((0.024, 0.030, 0.150)),
        origin=Origin(xyz=(0.122, -0.185, 0.375)),
        material="black_trim",
        name="pickup_side_1",
    )
    cabinet.visual(
        Cylinder(radius=0.006, length=0.310),
        origin=Origin(xyz=(-0.040, -0.192, 0.435), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="pickup_hinge_pin",
    )

    # Straight drawer runners in the base bay.
    cabinet.visual(
        Box((0.030, 0.240, 0.018)),
        origin=Origin(xyz=(-0.1775, -0.070, 0.070)),
        material="brushed_steel",
        name="cash_runner_0",
    )
    cabinet.visual(
        Box((0.030, 0.240, 0.018)),
        origin=Origin(xyz=(0.1775, -0.070, 0.070)),
        material="brushed_steel",
        name="cash_runner_1",
    )
    cabinet.visual(
        Box((0.035, 0.035, 0.160)),
        origin=Origin(xyz=(-0.180, -0.188, 0.120)),
        material="black_trim",
        name="cash_frame_0",
    )
    cabinet.visual(
        Box((0.035, 0.035, 0.160)),
        origin=Origin(xyz=(0.180, -0.188, 0.120)),
        material="black_trim",
        name="cash_frame_1",
    )
    cabinet.visual(
        Box((0.340, 0.035, 0.024)),
        origin=Origin(xyz=(0.0, -0.188, 0.190)),
        material="black_trim",
        name="cash_frame_top",
    )

    # Shelves, candy packets, and the stacked dispensing spirals visible behind glass.
    shelf_zs = [0.690, 0.905, 1.120, 1.335]
    candy_mats = ["candy_red", "candy_yellow", "candy_blue", "candy_green"]
    for row, coil_z in enumerate(shelf_zs):
        shelf_z = coil_z - 0.064
        cabinet.visual(
            Box((0.260, 0.270, 0.012)),
            origin=Origin(xyz=(-0.060, 0.000, shelf_z)),
            material="brushed_steel",
            name=f"shelf_{row}",
        )
        coil_geom = tube_from_spline_points(
            _helix_points(
                x=-0.060,
                z=coil_z,
                y_front=-0.132,
                y_back=0.142,
                radius=0.043,
                turns=3.4,
                phase=row * math.pi / 4.0,
                samples=110,
            ),
            radius=0.0042,
            samples_per_segment=2,
            radial_segments=12,
            cap_ends=True,
        )
        cabinet.visual(
            mesh_from_geometry(coil_geom, f"candy_spiral_{row}"),
            material="brushed_steel",
            name=f"candy_spiral_{row}",
        )
        for col, x in enumerate([-0.128, -0.062, 0.004]):
            cabinet.visual(
                Box((0.050, 0.030, 0.036)),
                origin=Origin(xyz=(x, -0.030 + 0.045 * (col % 2), shelf_z + 0.023)),
                material=candy_mats[(row + col) % len(candy_mats)],
                name=f"candy_pack_{row}_{col}",
            )

    # Cash drawer translating out of the base on the two runners.
    cash_drawer = model.part("cash_drawer")
    cash_drawer.visual(
        Box((0.300, 0.025, 0.110)),
        origin=Origin(xyz=(0.0, -0.0125, 0.0)),
        material="brushed_steel",
        name="drawer_face",
    )
    cash_drawer.visual(
        Box((0.260, 0.200, 0.070)),
        origin=Origin(xyz=(0.0, 0.090, -0.010)),
        material="charcoal_powdercoat",
        name="drawer_tray",
    )
    cash_drawer.visual(
        Box((0.018, 0.340, 0.014)),
        origin=Origin(xyz=(-0.1535, 0.150, -0.045)),
        material="brushed_steel",
        name="drawer_runner_0",
    )
    cash_drawer.visual(
        Box((0.018, 0.340, 0.014)),
        origin=Origin(xyz=(0.1535, 0.150, -0.045)),
        material="brushed_steel",
        name="drawer_runner_1",
    )
    cash_drawer.visual(
        Box((0.130, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, -0.034, 0.005)),
        material="dark_rubber",
        name="drawer_pull",
    )
    cash_joint = model.articulation(
        "cabinet_to_cash_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=cash_drawer,
        origin=Origin(xyz=(0.0, -0.192, 0.115)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.180),
    )

    # Retrieval flap, hinged horizontally at the pickup opening.
    flap = model.part("retrieval_flap")
    flap.visual(
        Box((0.240, 0.010, 0.124)),
        origin=Origin(xyz=(0.0, 0.0, -0.073)),
        material="smoked_plastic",
        name="flap_panel",
    )
    flap.visual(
        Box((0.220, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material="brushed_steel",
        name="flap_leaf",
    )
    flap.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(-0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="flap_knuckle_0",
    )
    flap.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="flap_knuckle_1",
    )
    flap_joint = model.articulation(
        "cabinet_to_retrieval_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(-0.040, -0.192, 0.435)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=1.10),
    )

    # Continuously rotating selector dial on the control panel.
    selector_dial = model.part("selector_dial")
    knob_geom = KnobGeometry(
        0.070,
        0.030,
        body_style="faceted",
        top_diameter=0.058,
        base_diameter=0.074,
        edge_radius=0.001,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0014, width=0.002),
        indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        center=False,
    )
    selector_dial.visual(
        mesh_from_geometry(knob_geom, "selector_dial_body"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="selector_orange",
        name="dial_body",
    )
    dial_joint = model.articulation(
        "cabinet_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_dial,
        origin=Origin(xyz=(0.140, -0.208, 0.875)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    # Store salient limits for tests without hard-coding repeated constants.
    model.meta["cash_drawer_upper"] = cash_joint.motion_limits.upper
    model.meta["flap_upper"] = flap_joint.motion_limits.upper
    model.meta["dial_joint"] = dial_joint.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drawer = object_model.get_part("cash_drawer")
    flap = object_model.get_part("retrieval_flap")
    dial = object_model.get_part("selector_dial")
    drawer_joint = object_model.get_articulation("cabinet_to_cash_drawer")
    flap_joint = object_model.get_articulation("cabinet_to_retrieval_flap")
    dial_joint = object_model.get_articulation("cabinet_to_selector_dial")

    # The flap knuckles intentionally wrap the fixed hinge pin, just like a
    # captured barrel hinge. Scope the allowance to the pin/knuckle elements.
    for knuckle in ("flap_knuckle_0", "flap_knuckle_1"):
        ctx.allow_overlap(
            cabinet,
            flap,
            elem_a="pickup_hinge_pin",
            elem_b=knuckle,
            reason="The retrieval-flap knuckle intentionally captures the fixed horizontal hinge pin.",
        )
        ctx.expect_overlap(
            cabinet,
            flap,
            axes="xyz",
            elem_a="pickup_hinge_pin",
            elem_b=knuckle,
            min_overlap=0.010,
            name=f"{knuckle} wraps hinge pin",
        )

    ctx.expect_gap(
        drawer,
        cabinet,
        axis="x",
        positive_elem="drawer_runner_0",
        negative_elem="cash_runner_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="left drawer runner rides cabinet rail",
    )
    ctx.expect_gap(
        cabinet,
        drawer,
        axis="x",
        positive_elem="cash_runner_1",
        negative_elem="drawer_runner_1",
        max_gap=0.001,
        max_penetration=0.00001,
        name="right drawer runner rides cabinet rail",
    )
    ctx.expect_gap(
        cabinet,
        dial,
        axis="y",
        positive_elem="control_panel",
        negative_elem="dial_body",
        max_gap=0.002,
        max_penetration=0.001,
        name="selector dial seats on panel face",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: object_model.meta["cash_drawer_upper"]}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="drawer_runner_0",
            elem_b="cash_runner_0",
            min_overlap=0.050,
            name="extended drawer retains runner insertion",
        )
    ctx.check(
        "cash drawer translates outward on straight runners",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.150,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_joint: object_model.meta["flap_upper"]}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    if closed_flap_aabb is not None and open_flap_aabb is not None:
        closed_y = (closed_flap_aabb[0][1] + closed_flap_aabb[1][1]) * 0.5
        open_y = (open_flap_aabb[0][1] + open_flap_aabb[1][1]) * 0.5
        closed_z = (closed_flap_aabb[0][2] + closed_flap_aabb[1][2]) * 0.5
        open_z = (open_flap_aabb[0][2] + open_flap_aabb[1][2]) * 0.5
    else:
        closed_y = open_y = closed_z = open_z = None
    ctx.check(
        "retrieval flap swings inward and upward",
        closed_y is not None
        and open_y is not None
        and closed_z is not None
        and open_z is not None
        and open_y > closed_y + 0.040
        and open_z > closed_z + 0.020,
        details=f"closed_y={closed_y}, open_y={open_y}, closed_z={closed_z}, open_z={open_z}",
    )

    rest_dial_pos = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi * 1.5}):
        turned_dial_pos = ctx.part_world_position(dial)
    ctx.check(
        "selector dial is continuous and rotates about its shaft",
        getattr(dial_joint, "articulation_type", None) == ArticulationType.CONTINUOUS
        and rest_dial_pos is not None
        and turned_dial_pos is not None
        and abs(rest_dial_pos[0] - turned_dial_pos[0]) < 1e-6
        and abs(rest_dial_pos[1] - turned_dial_pos[1]) < 1e-6
        and abs(rest_dial_pos[2] - turned_dial_pos[2]) < 1e-6,
        details=f"type={getattr(dial_joint, 'articulation_type', None)}, rest={rest_dial_pos}, turned={turned_dial_pos}",
    )

    return ctx.report()


object_model = build_object_model()
