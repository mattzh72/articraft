from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_convection_toaster_oven")

    brushed_steel = model.material("brushed_steel", rgba=(0.55, 0.56, 0.55, 1.0))
    dark_trim = model.material("black_enamel_trim", rgba=(0.015, 0.014, 0.012, 1.0))
    warm_interior = model.material("warm_reflective_interior", rgba=(0.72, 0.68, 0.58, 1.0))
    glass = model.material("smoky_heat_glass", rgba=(0.12, 0.20, 0.24, 0.42))
    chrome = model.material("chrome_wire", rgba=(0.82, 0.84, 0.84, 1.0))

    body_w = 0.50
    body_d = 0.38
    body_h = 0.30
    wall = 0.025
    front_y = -body_d / 2.0
    rear_y = body_d / 2.0

    body = model.part("body")

    # Hollow, open-front rectangular oven body.  The front bezel and the five
    # shell panels overlap slightly as one welded stamped-metal cabinet.
    body.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_h - wall / 2.0)),
        material=brushed_steel,
        name="top_shell",
    )
    body.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=brushed_steel,
        name="bottom_shell",
    )
    body.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + wall / 2.0, 0.0, body_h / 2.0)),
        material=brushed_steel,
        name="side_shell_0",
    )
    body.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - wall / 2.0, 0.0, body_h / 2.0)),
        material=brushed_steel,
        name="side_shell_1",
    )
    body.visual(
        Box((body_w, wall, body_h)),
        origin=Origin(xyz=(0.0, rear_y - wall / 2.0, body_h / 2.0)),
        material=warm_interior,
        name="rear_wall",
    )
    body.visual(
        Cylinder(radius=0.060, length=0.006),
        origin=Origin(xyz=(0.0, rear_y - wall - 0.001, 0.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="convection_fan_recess",
    )
    body.visual(
        Cylinder(radius=0.0018, length=0.112),
        origin=Origin(xyz=(0.0, rear_y - wall - 0.004, 0.165), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="fan_grille_cross_x",
    )
    body.visual(
        Cylinder(radius=0.0018, length=0.112),
        origin=Origin(xyz=(0.0, rear_y - wall - 0.004, 0.165)),
        material=chrome,
        name="fan_grille_cross_z",
    )

    bezel_y = front_y - 0.012
    body.visual(
        Box((body_w, 0.026, 0.046)),
        origin=Origin(xyz=(0.0, bezel_y, body_h - 0.023)),
        material=dark_trim,
        name="front_top_bezel",
    )
    body.visual(
        Box((body_w, 0.026, 0.046)),
        origin=Origin(xyz=(0.0, bezel_y, 0.023)),
        material=dark_trim,
        name="front_bottom_bezel",
    )
    body.visual(
        Box((0.035, 0.026, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + 0.0175, bezel_y, body_h / 2.0)),
        material=dark_trim,
        name="front_stile_0",
    )
    body.visual(
        Box((0.035, 0.026, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - 0.0175, bezel_y, body_h / 2.0)),
        material=dark_trim,
        name="front_stile_1",
    )

    # Lighter baking-chamber liner visible through the glass door.
    body.visual(
        Box((body_w - 2.0 * wall, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.150, 0.275)),
        material=warm_interior,
        name="inner_top_lip",
    )
    body.visual(
        Box((body_w - 2.0 * wall, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.150, 0.045)),
        material=warm_interior,
        name="inner_bottom_lip",
    )

    # Dark side ventilation slots on the outer shell.
    for side_idx, x_sign in enumerate((-1.0, 1.0)):
        x = x_sign * (body_w / 2.0 + 0.0008)
        for slot_idx, z in enumerate((0.105, 0.130, 0.155, 0.180, 0.205)):
            body.visual(
                Box((0.003, 0.115, 0.007)),
                origin=Origin(xyz=(x, 0.035, z)),
                material=dark_trim,
                name=f"vent_{side_idx}_{slot_idx}",
            )

    # Two fixed side guide rails inside the baking chamber.  Each has bracket
    # tabs that visibly carry the rail back into the side wall.
    rail_z = 0.149
    for rail_idx, x in enumerate((-0.188, 0.188)):
        sign = -1.0 if x < 0.0 else 1.0
        body.visual(
            Box((0.012, 0.270, 0.008)),
            origin=Origin(xyz=(x, -0.015, rail_z)),
            material=chrome,
            name=f"side_guide_{rail_idx}",
        )
        for tab_idx, y in enumerate((-0.112, 0.092)):
            body.visual(
                Box((0.037, 0.018, 0.010)),
                origin=Origin(xyz=(x + sign * 0.0185, y, rail_z)),
                material=chrome,
                name=f"guide_tab_{rail_idx}_{tab_idx}",
            )

    # Fixed knuckles for a full-width bottom hinge, interleaved with the door
    # knuckle.  They are connected to the lower front bezel by a narrow leaf.
    hinge_y = front_y - 0.045
    hinge_z = 0.045
    for idx, x in enumerate((-0.140, 0.140)):
        body.visual(
            Box((0.112, 0.028, 0.012)),
            origin=Origin(xyz=(x, hinge_y + 0.022, hinge_z)),
            material=dark_trim,
            name=f"hinge_leaf_{idx}",
        )
        body.visual(
            Cylinder(radius=0.0095, length=0.110),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_trim,
            name=f"body_hinge_barrel_{idx}",
        )
    body.visual(
        Cylinder(radius=0.0035, length=0.430),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_pin",
    )

    door = model.part("door")
    door_w = 0.430
    door_h = 0.245
    door_y = -0.014
    bar = 0.026
    door.visual(
        Box((door_w, 0.020, 0.028)),
        origin=Origin(xyz=(0.0, door_y, 0.040)),
        material=dark_trim,
        name="bottom_rail",
    )
    door.visual(
        Box((door_w, 0.020, 0.028)),
        origin=Origin(xyz=(0.0, door_y, door_h - 0.014)),
        material=dark_trim,
        name="top_rail",
    )
    for stile_idx, x in enumerate((-door_w / 2.0 + bar / 2.0, door_w / 2.0 - bar / 2.0)):
        door.visual(
            Box((bar, 0.020, door_h - 0.028)),
            origin=Origin(xyz=(x, door_y, door_h / 2.0)),
            material=dark_trim,
            name=f"door_stile_{stile_idx}",
        )
    door.visual(
        Box((0.385, 0.006, 0.180)),
        origin=Origin(xyz=(0.0, door_y - 0.001, 0.128)),
        material=glass,
        name="glass_panel",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.150, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, door_y, 0.020)),
        material=dark_trim,
        name="door_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.305),
        origin=Origin(xyz=(0.0, -0.055, 0.193), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="front_handle",
    )
    for post_idx, x in enumerate((-0.130, 0.130)):
        door.visual(
            Box((0.018, 0.042, 0.018)),
            origin=Origin(xyz=(x, -0.035, 0.193)),
            material=brushed_steel,
            name=f"handle_post_{post_idx}",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.72),
    )

    # Two small upper-fascia control dials; they are separate revolute controls
    # rather than printed-on circles.
    for dial_idx, x in enumerate((0.125, 0.180)):
        dial = model.part(f"dial_{dial_idx}")
        dial.visual(
            Cylinder(radius=0.017, length=0.018),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_trim,
            name="dial_cap",
        )
        dial.visual(
            Box((0.004, 0.003, 0.014)),
            origin=Origin(xyz=(0.0, -0.016, 0.008)),
            material=chrome,
            name="dial_pointer",
        )
        model.articulation(
            f"body_to_dial_{dial_idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=dial,
            origin=Origin(xyz=(x, front_y - 0.028, body_h - 0.028)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=-2.35, upper=2.35),
        )

    rack = model.part("wire_rack")
    rod_radius = 0.003
    rack_w = 0.360
    rack_d = 0.250

    def rack_rod(name: str, *, xyz: tuple[float, float, float], length: float, along: str) -> None:
        rpy = (0.0, math.pi / 2.0, 0.0) if along == "x" else (math.pi / 2.0, 0.0, 0.0)
        rack.visual(
            Cylinder(radius=rod_radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=chrome,
            name=name,
        )

    rack_rod("side_rod_0", xyz=(-rack_w / 2.0, 0.0, 0.0), length=rack_d, along="y")
    rack_rod("side_rod_1", xyz=(rack_w / 2.0, 0.0, 0.0), length=rack_d, along="y")
    rack_rod("front_rod", xyz=(0.0, -rack_d / 2.0, 0.0), length=rack_w + 0.006, along="x")
    rack_rod("rear_rod", xyz=(0.0, rack_d / 2.0, 0.0), length=rack_w + 0.006, along="x")
    for idx, y in enumerate((-0.085, -0.0425, 0.0, 0.0425, 0.085)):
        rack_rod(f"cross_wire_{idx}", xyz=(0.0, y, 0.004), length=rack_w + 0.010, along="x")
    for idx, x in enumerate((-0.060, 0.060)):
        rack_rod(f"length_wire_{idx}", xyz=(x, 0.0, 0.004), length=rack_d - 0.010, along="y")

    model.articulation(
        "body_to_wire_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(0.0, -0.015, 0.156)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.160),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    rack = object_model.get_part("wire_rack")
    door_joint = object_model.get_articulation("body_to_door")
    rack_slide = object_model.get_articulation("body_to_wire_rack")

    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_pin",
        elem_b="door_hinge_barrel",
        reason="The continuous hinge pin is intentionally captured inside the solid-proxy door barrel.",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="x",
        elem_a="hinge_pin",
        elem_b="door_hinge_barrel",
        min_overlap=0.10,
        name="door barrel is captured on the hinge pin",
    )

    for idx in (0, 1):
        ctx.expect_gap(
            rack,
            body,
            axis="z",
            positive_elem=f"side_rod_{idx}",
            negative_elem=f"side_guide_{idx}",
            max_gap=0.001,
            max_penetration=0.001,
            name=f"rack side rod {idx} rides on its guide rail",
        )
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            elem_a=f"side_rod_{idx}",
            elem_b=f"side_guide_{idx}",
            min_overlap=0.22,
            name=f"rack side rod {idx} is retained in the guide rail at rest",
        )

    rest_rack_pos = ctx.part_world_position(rack)
    with ctx.pose({rack_slide: 0.160}):
        extended_rack_pos = ctx.part_world_position(rack)
        for idx in (0, 1):
            ctx.expect_overlap(
                rack,
                body,
                axes="y",
                elem_a=f"side_rod_{idx}",
                elem_b=f"side_guide_{idx}",
                min_overlap=0.085,
                name=f"extended rack side rod {idx} remains in the guide rail",
            )
    ctx.check(
        "wire rack slides forward on prismatic rails",
        rest_rack_pos is not None
        and extended_rack_pos is not None
        and extended_rack_pos[1] < rest_rack_pos[1] - 0.14,
        details=f"rest={rest_rack_pos}, extended={extended_rack_pos}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    closed_glass = aabb_center(ctx.part_element_world_aabb(door, elem="glass_panel"))
    with ctx.pose({door_joint: 1.72}):
        dropped_glass = aabb_center(ctx.part_element_world_aabb(door, elem="glass_panel"))
    ctx.check(
        "glass door drops forward and downward about the bottom hinge",
        closed_glass is not None
        and dropped_glass is not None
        and dropped_glass[1] < closed_glass[1] - 0.08
        and dropped_glass[2] < closed_glass[2] - 0.10,
        details=f"closed={closed_glass}, dropped={dropped_glass}",
    )

    return ctx.report()


object_model = build_object_model()
