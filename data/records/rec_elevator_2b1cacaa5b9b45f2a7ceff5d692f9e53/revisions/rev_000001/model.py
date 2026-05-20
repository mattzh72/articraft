from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="piston_residential_elevator")

    galvanized = Material("galvanized_steel", rgba=(0.55, 0.58, 0.58, 1.0))
    dark_steel = Material("dark_guide_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    concrete = Material("brushed_concrete", rgba=(0.55, 0.53, 0.50, 1.0))
    car_panel = Material("warm_car_panel", rgba=(0.82, 0.76, 0.66, 1.0))
    glass = Material("smoked_glass", rgba=(0.45, 0.66, 0.78, 0.38))
    door_paint = Material("cream_lobby_door", rgba=(0.80, 0.78, 0.70, 1.0))
    brass = Material("brushed_brass", rgba=(0.78, 0.61, 0.30, 1.0))
    chrome = Material("polished_chrome", rgba=(0.86, 0.90, 0.92, 1.0))
    hydraulic_blue = Material("hydraulic_blue", rgba=(0.05, 0.16, 0.45, 1.0))

    shaft = model.part("shaft_frame")

    shaft_width = 1.70
    shaft_depth = 1.35
    shaft_height = 6.70
    post = 0.080
    landing_zs = (0.0, 2.2, 4.4)
    door_height = 2.02
    door_width = 1.12
    door_y = -0.82
    hinge_x = -0.72

    # Pit and base slab: the slab ties the shaft frame together and hides the
    # in-ground part of the hydraulic piston.
    for name, origin, size in (
        ("base_front_slab", (0.0, -0.265, -0.06), (2.20, 1.42, 0.12)),
        ("base_rear_slab", (0.0, 0.845, -0.06), (2.20, 0.26, 0.12)),
        ("base_side_slab_0", (-0.63, 0.58, -0.06), (0.94, 0.30, 0.12)),
        ("base_side_slab_1", (0.63, 0.58, -0.06), (0.94, 0.30, 0.12)),
    ):
        shaft.visual(
            Box(size),
            origin=Origin(xyz=origin),
            material=concrete,
            name=name,
        )
    for name, origin, size in (
        ("pit_front_wall", (0.0, 0.41, -2.265), (0.36, 0.05, 4.65)),
        ("pit_rear_wall", (0.0, 0.75, -2.265), (0.36, 0.05, 4.65)),
        ("pit_side_wall_0", (-0.17, 0.58, -2.265), (0.05, 0.34, 4.65)),
        ("pit_side_wall_1", (0.17, 0.58, -2.265), (0.05, 0.34, 4.65)),
    ):
        shaft.visual(
            Box(size),
            origin=Origin(xyz=origin),
            material=concrete,
            name=name,
        )
    shaft.visual(
        Cylinder(radius=0.075, length=4.72),
        origin=Origin(xyz=(0.0, 0.58, -2.29)),
        material=hydraulic_blue,
        name="hydraulic_barrel",
    )
    for x in (-0.095, 0.095):
        shaft.visual(
            Box((0.050, 0.18, 0.045)),
            origin=Origin(xyz=(x, 0.58, 0.025)),
            material=galvanized,
            name=f"barrel_clamp_{x:+.1f}",
        )

    # Four open corner posts and perimeter beams make the open shaft frame.
    for x in (-shaft_width / 2, shaft_width / 2):
        for y in (-shaft_depth / 2, shaft_depth / 2):
            shaft.visual(
                Box((post, post, shaft_height)),
                origin=Origin(xyz=(x, y, shaft_height / 2)),
                material=galvanized,
                name=f"corner_post_{x:+.1f}_{y:+.1f}",
            )

    for z in (0.06, shaft_height - 0.04):
        shaft.visual(
            Box((shaft_width + post, post, post)),
            origin=Origin(xyz=(0.0, -shaft_depth / 2, z)),
            material=galvanized,
            name=f"front_crossbeam_{z:.1f}",
        )
        shaft.visual(
            Box((shaft_width + post, post, post)),
            origin=Origin(xyz=(0.0, shaft_depth / 2, z)),
            material=galvanized,
            name=f"rear_crossbeam_{z:.1f}",
        )
        shaft.visual(
            Box((post, shaft_depth + post, post)),
            origin=Origin(xyz=(-shaft_width / 2, 0.0, z)),
            material=galvanized,
            name=f"side_crossbeam_neg_{z:.1f}",
        )
        shaft.visual(
            Box((post, shaft_depth + post, post)),
            origin=Origin(xyz=(shaft_width / 2, 0.0, z)),
            material=galvanized,
            name=f"side_crossbeam_pos_{z:.1f}",
        )

    # Landing portals with sills, headers and hinge-side jambs.
    for idx, z in enumerate(landing_zs):
        shaft.visual(
            Box((1.42, 0.10, 0.08)),
            origin=Origin(xyz=(0.0, -0.72, z + 0.02)),
            material=concrete,
            name=f"landing_sill_{idx}",
        )
        shaft.visual(
            Box((1.42, 0.10, 0.08)),
            origin=Origin(xyz=(0.0, -0.72, z + door_height + 0.10)),
            material=galvanized,
            name=f"landing_header_{idx}",
        )
        for x in (-0.66, 0.66):
            shaft.visual(
                Box((0.08, 0.10, door_height + 0.18)),
                origin=Origin(xyz=(x, -0.71, z + (door_height + 0.18) / 2)),
                material=galvanized,
                name=f"landing_jamb_{idx}_{x:+.1f}",
            )

        # Fixed hinge leaves and fixed knuckles interleave with each door's
        # moving knuckles around the same vertical axis.
        for k, zc in enumerate((z + 0.64, z + 1.38)):
            shaft.visual(
                Cylinder(radius=0.026, length=0.28),
                origin=Origin(xyz=(hinge_x, door_y, zc)),
                material=dark_steel,
                name=f"fixed_knuckle_{idx}_{k}",
            )
            shaft.visual(
                Box((0.075, 0.038, 0.30)),
                origin=Origin(xyz=(hinge_x + 0.0275, door_y + 0.045, zc)),
                material=dark_steel,
                name=f"fixed_hinge_leaf_{idx}_{k}",
            )

    # Two dark guide rails mounted just outside the cab sides.
    for x in (-0.84, 0.84):
        shaft.visual(
            Box((0.060, 0.10, shaft_height - 0.08)),
            origin=Origin(xyz=(x, 0.04, (shaft_height - 0.08) / 2 + 0.04)),
            material=galvanized,
            name=f"rail_support_post_{x:+.1f}",
        )
    for x in (-0.75, 0.75):
        shaft.visual(
            Box((0.035, 0.060, shaft_height - 0.20)),
            origin=Origin(xyz=(x, 0.0, (shaft_height - 0.20) / 2 + 0.08)),
            material=dark_steel,
            name=f"guide_rail_{x:+.1f}",
        )
        shaft.visual(
            Box((0.22, 0.09, 0.08)),
            origin=Origin(xyz=((1 if x > 0 else -1) * 0.81, 0.02, 0.09)),
            material=galvanized,
            name=f"lower_rail_tie_{x:+.1f}",
        )
        shaft.visual(
            Box((0.22, 0.09, 0.08)),
            origin=Origin(xyz=((1 if x > 0 else -1) * 0.81, 0.02, shaft_height - 0.09)),
            material=galvanized,
            name=f"upper_rail_tie_{x:+.1f}",
        )
        for z in (0.75, 2.6, 4.45, 6.10):
            shaft.visual(
                Box((0.22, 0.10, 0.045)),
                origin=Origin(xyz=((1 if x > 0 else -1) * 0.81, 0.04, z)),
                material=galvanized,
                name=f"rail_bracket_{x:+.1f}_{z:.1f}",
            )

    car = model.part("car")
    car.visual(
        Box((1.16, 0.96, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=car_panel,
        name="car_floor",
    )
    car.visual(
        Box((1.16, 0.96, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.16)),
        material=car_panel,
        name="car_roof",
    )
    car.visual(
        Box((0.06, 0.96, 2.10)),
        origin=Origin(xyz=(-0.58, 0.0, 1.10)),
        material=car_panel,
        name="side_wall_0",
    )
    car.visual(
        Box((0.06, 0.96, 2.10)),
        origin=Origin(xyz=(0.58, 0.0, 1.10)),
        material=car_panel,
        name="side_wall_1",
    )
    car.visual(
        Box((1.16, 0.06, 2.10)),
        origin=Origin(xyz=(0.0, 0.48, 1.10)),
        material=car_panel,
        name="back_wall",
    )
    # Front cab frame leaves a clear opening aligned with the lobby doors.
    for x in (-0.52, 0.52):
        car.visual(
            Box((0.10, 0.06, 2.02)),
            origin=Origin(xyz=(x, -0.48, 1.07)),
            material=car_panel,
            name=f"front_stile_{x:+.1f}",
        )
    car.visual(
        Box((1.16, 0.06, 0.12)),
        origin=Origin(xyz=(0.0, -0.48, 2.07)),
        material=car_panel,
        name="front_header",
    )
    car.visual(
        Box((1.00, 0.025, 1.28)),
        origin=Origin(xyz=(0.0, 0.515, 1.18)),
        material=glass,
        name="back_glass",
    )
    for x in (-0.615, 0.615):
        car.visual(
            Box((0.025, 0.62, 1.20)),
            origin=Origin(xyz=(x, 0.02, 1.18)),
            material=glass,
            name=f"side_glass_{x:+.1f}",
        )
    for x in (-0.64, 0.64):
        for z in (0.38, 1.82):
            car.visual(
                Box((0.075, 0.18, 0.12)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=dark_steel,
                name=f"guide_shoe_{x:+.1f}_{z:.1f}",
            )

    piston = model.part("piston_ram")
    piston.visual(
        Cylinder(radius=0.045, length=4.65),
        origin=Origin(xyz=(0.0, 0.58, -2.205)),
        material=chrome,
        name="chrome_ram",
    )
    piston.visual(
        Box((0.30, 0.12, 0.055)),
        origin=Origin(xyz=(0.0, 0.52, 0.1025)),
        material=dark_steel,
        name="car_lift_pad",
    )

    car_lift = model.articulation(
        "car_lift",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.65, lower=0.0, upper=4.4),
    )
    model.articulation(
        "ram_lift",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=piston,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.65, lower=0.0, upper=4.4),
        mimic=Mimic(joint=car_lift.name, multiplier=1.0, offset=0.0),
    )

    for idx, z in enumerate(landing_zs):
        door = model.part(f"door_{idx}")
        door.visual(
            Box((door_width, 0.048, door_height)),
            origin=Origin(xyz=(0.08 + door_width / 2, 0.0, door_height / 2)),
            material=door_paint,
            name="door_panel",
        )
        door.visual(
            Box((0.070, 0.052, door_height)),
            origin=Origin(xyz=(0.065, 0.0, door_height / 2)),
            material=door_paint,
            name="hinge_stile",
        )
        door.visual(
            Box((0.055, 0.030, 0.18)),
            origin=Origin(xyz=(0.88, -0.035, 1.02)),
            material=brass,
            name="pull_handle",
        )
        door.visual(
            Box((0.45, 0.014, 0.58)),
            origin=Origin(xyz=(0.60, -0.031, 1.32)),
            material=glass,
            name="narrow_window",
        )
        for k, zc in enumerate((0.28, 0.98, 1.68)):
            door.visual(
                Box((0.070, 0.052, 0.28)),
                origin=Origin(xyz=(0.055, 0.0, zc)),
                material=door_paint,
                name=f"knuckle_web_{k}",
            )
            door.visual(
                Cylinder(radius=0.024, length=0.34),
                origin=Origin(xyz=(0.0, 0.0, zc)),
                material=dark_steel,
                name=f"moving_knuckle_{k}",
            )

        model.articulation(
            f"door_{idx}_hinge",
            ArticulationType.REVOLUTE,
            parent=shaft,
            child=door,
            origin=Origin(xyz=(hinge_x, door_y, z + 0.05)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=math.radians(100.0)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft_frame")
    car = object_model.get_part("car")
    piston = object_model.get_part("piston_ram")
    car_lift = object_model.get_articulation("car_lift")
    lower_door = object_model.get_part("door_0")
    lower_hinge = object_model.get_articulation("door_0_hinge")

    ctx.allow_overlap(
        shaft,
        piston,
        elem_a="hydraulic_barrel",
        elem_b="chrome_ram",
        reason="The chrome hydraulic ram is intentionally nested inside the in-ground cylinder barrel.",
    )
    ctx.expect_within(
        piston,
        shaft,
        axes="xy",
        inner_elem="chrome_ram",
        outer_elem="hydraulic_barrel",
        margin=0.002,
        name="hydraulic ram centered in barrel",
    )
    ctx.expect_overlap(
        piston,
        shaft,
        axes="z",
        elem_a="chrome_ram",
        elem_b="hydraulic_barrel",
        min_overlap=0.12,
        name="hydraulic ram retained in barrel",
    )

    rest_pos = ctx.part_world_position(car)
    with ctx.pose({car_lift: 4.4}):
        raised_pos = ctx.part_world_position(car)
        ctx.expect_overlap(
            piston,
            shaft,
            axes="z",
            elem_a="chrome_ram",
            elem_b="hydraulic_barrel",
            min_overlap=0.12,
            name="raised ram still retained in barrel",
        )
    ctx.check(
        "car rises along guide rails",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 4.3,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    ctx.expect_gap(
        car,
        lower_door,
        axis="y",
        min_gap=0.18,
        name="closed lobby door stands outside the cab",
    )

    closed_aabb = ctx.part_world_aabb(lower_door)
    with ctx.pose({lower_hinge: math.radians(85.0)}):
        opened_aabb = ctx.part_world_aabb(lower_door)
    ctx.check(
        "lobby door swings outward on vertical hinge",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.35,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
