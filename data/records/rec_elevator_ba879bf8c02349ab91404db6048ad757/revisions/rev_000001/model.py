from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="small_goods_dumbwaiter")

    painted_steel = Material("painted_steel", rgba=(0.70, 0.74, 0.76, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.12, 0.14, 0.15, 1.0))
    polished_rail = Material("polished_rail", rgba=(0.82, 0.84, 0.82, 1.0))
    car_metal = Material("car_metal", rgba=(0.55, 0.58, 0.60, 1.0))
    car_floor = Material("car_floor", rgba=(0.30, 0.31, 0.31, 1.0))
    wood = Material("varnished_wood", rgba=(0.55, 0.30, 0.13, 1.0))
    dark_wood = Material("dark_end_grain", rgba=(0.34, 0.17, 0.07, 1.0))
    brass = Material("brass", rgba=(0.88, 0.63, 0.22, 1.0))

    shaft = model.part("shaft")

    # A compact two-stop service shaft: bolted base/top plates, four posts,
    # front hatch frames, and smooth round guide rails.
    shaft.visual(Box((0.96, 0.82, 0.06)), origin=Origin(xyz=(0.0, 0.0, 0.03)), material=dark_steel, name="base_plate")
    shaft.visual(Box((0.96, 0.82, 0.06)), origin=Origin(xyz=(0.0, 0.0, 2.57)), material=dark_steel, name="top_plate")

    for x in (-0.45, 0.45):
        for y in (-0.36, 0.36):
            shaft.visual(
                Box((0.06, 0.06, 2.54)),
                origin=Origin(xyz=(x, y, 1.30)),
                material=painted_steel,
                name=f"corner_post_{x}_{y}",
            )

    # Rear and side bracing make the shaft read as one rigid vertical cage.
    shaft.visual(Box((0.84, 0.045, 2.20)), origin=Origin(xyz=(0.0, 0.39, 1.30)), material=painted_steel, name="rear_backbone")
    for x in (-0.45, 0.45):
        shaft.visual(Box((0.045, 0.66, 0.055)), origin=Origin(xyz=(x, 0.0, 0.34)), material=painted_steel, name=f"lower_side_tie_{x}")
        shaft.visual(Box((0.045, 0.66, 0.055)), origin=Origin(xyz=(x, 0.0, 1.30)), material=painted_steel, name=f"middle_side_tie_{x}")
        shaft.visual(Box((0.045, 0.66, 0.055)), origin=Origin(xyz=(x, 0.0, 2.26)), material=painted_steel, name=f"upper_side_tie_{x}")

    # Two front hatch openings, one per floor level.
    for zc, label in ((0.765, "lower"), (1.815, "upper")):
        top_z = zc + 0.285
        bottom_z = zc - 0.285
        shaft.visual(Box((0.76, 0.06, 0.07)), origin=Origin(xyz=(0.0, -0.36, top_z + 0.035)), material=painted_steel, name=f"{label}_header")
        shaft.visual(Box((0.76, 0.06, 0.07)), origin=Origin(xyz=(0.0, -0.36, bottom_z - 0.035)), material=painted_steel, name=f"{label}_sill")
        for x in (-0.40, 0.40):
            shaft.visual(Box((0.055, 0.06, 0.64)), origin=Origin(xyz=(x, -0.36, zc)), material=painted_steel, name=f"{label}_jamb_{x}")
        # Fixed hinge knuckle mounts sit just outside the movable central barrel.
        for x in (-0.265, 0.265):
            shaft.visual(Box((0.050, 0.085, 0.018)), origin=Origin(xyz=(x, -0.405, top_z + 0.028)), material=brass, name=f"{label}_hinge_mount_{x}")

    for x, suffix in ((-0.34, "0"), (0.34, "1")):
        shaft.visual(
            Cylinder(radius=0.018, length=2.42),
            origin=Origin(xyz=(x, 0.0, 1.31)),
            material=polished_rail,
            name=f"guide_rail_{suffix}",
        )
        for z, level in ((0.32, "low"), (1.30, "mid"), (2.28, "high")):
            shaft.visual(
                Box((0.052, 0.375, 0.035)),
                origin=Origin(xyz=(x, 0.1875, z)),
                material=dark_steel,
                name=f"rail_clamp_{suffix}_{level}",
            )

    car = model.part("car")
    # The movable car is an open-front square goods box with metal side walls,
    # a dark load floor, a roof sheet, and guide shoes bearing on both rails.
    car.visual(Box((0.54, 0.50, 0.040)), origin=Origin(xyz=(0.0, 0.0, -0.260)), material=car_floor, name="car_floor")
    car.visual(Box((0.54, 0.50, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.255)), material=car_metal, name="car_roof")
    car.visual(Box((0.035, 0.50, 0.52)), origin=Origin(xyz=(-0.270, 0.0, 0.0)), material=car_metal, name="car_side_0")
    car.visual(Box((0.035, 0.50, 0.52)), origin=Origin(xyz=(0.270, 0.0, 0.0)), material=car_metal, name="car_side_1")
    car.visual(Box((0.54, 0.035, 0.52)), origin=Origin(xyz=(0.0, 0.250, 0.0)), material=car_metal, name="car_back")
    car.visual(Box((0.54, 0.030, 0.060)), origin=Origin(xyz=(0.0, -0.250, -0.210)), material=car_metal, name="front_lip")
    for x, suffix in ((-0.300, "0"), (0.300, "1")):
        for z, pos in ((-0.150, "low"), (0.150, "high")):
            car.visual(
                Box((0.044, 0.110, 0.110)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=dark_steel,
                name=f"guide_shoe_{suffix}_{pos}",
            )

    model.articulation(
        "car_slide",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.765)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=0.0, upper=1.05),
    )

    for name, zc in (("lower_hatch", 0.765), ("upper_hatch", 1.815)):
        top_z = zc + 0.285
        door = model.part(name)
        # Child frame is the horizontal hinge axis at the top edge; the panel
        # hangs downward in the closed pose and swings outward/upward for +q.
        door.visual(Cylinder(radius=0.018, length=0.43), origin=Origin(rpy=(0.0, pi / 2.0, 0.0)), material=brass, name="hinge_barrel")
        door.visual(Box((0.56, 0.036, 0.55)), origin=Origin(xyz=(0.0, 0.0, -0.293)), material=wood, name="door_panel")
        # Proud boards give the wooden hatch a planked service-door look.
        for x in (-0.185, 0.0, 0.185):
            door.visual(Box((0.010, 0.006, 0.50)), origin=Origin(xyz=(x, -0.021, -0.293)), material=dark_wood, name=f"vertical_batten_{x}")
        door.visual(Box((0.50, 0.006, 0.012)), origin=Origin(xyz=(0.0, -0.021, -0.065)), material=dark_wood, name="top_rail")
        door.visual(Box((0.50, 0.006, 0.012)), origin=Origin(xyz=(0.0, -0.021, -0.520)), material=dark_wood, name="bottom_rail")
        door.visual(Box((0.220, 0.022, 0.028)), origin=Origin(xyz=(0.0, -0.066, -0.320)), material=brass, name="pull_handle")
        for x in (-0.085, 0.085):
            door.visual(Box((0.026, 0.050, 0.026)), origin=Origin(xyz=(x, -0.041, -0.320)), material=brass, name=f"handle_standoff_{x}")

        model.articulation(
            f"shaft_to_{name}",
            ArticulationType.REVOLUTE,
            parent=shaft,
            child=door,
            origin=Origin(xyz=(0.0, -0.405, top_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    lower_hatch = object_model.get_part("lower_hatch")
    upper_hatch = object_model.get_part("upper_hatch")
    car_slide = object_model.get_articulation("car_slide")
    lower_hinge = object_model.get_articulation("shaft_to_lower_hatch")
    upper_hinge = object_model.get_articulation("shaft_to_upper_hatch")

    ctx.expect_within(
        car,
        shaft,
        axes="xy",
        margin=0.0,
        name="car stays inside the shaft footprint",
    )
    ctx.expect_contact(
        car,
        shaft,
        elem_a="guide_shoe_1_low",
        elem_b="guide_rail_1",
        contact_tol=0.002,
        name="car rides on the right smooth guide rail",
    )
    ctx.expect_contact(
        car,
        shaft,
        elem_a="guide_shoe_0_low",
        elem_b="guide_rail_0",
        contact_tol=0.002,
        name="car rides on the left smooth guide rail",
    )

    rest_car_pos = ctx.part_world_position(car)
    with ctx.pose({car_slide: 1.05}):
        ctx.expect_within(
            car,
            shaft,
            axes="xy",
            margin=0.0,
            name="raised car stays between the rails",
        )
        ctx.expect_contact(
            car,
            shaft,
            elem_a="guide_shoe_1_high",
            elem_b="guide_rail_1",
            contact_tol=0.002,
            name="raised car remains guided by the rail",
        )
        raised_car_pos = ctx.part_world_position(car)

    ctx.check(
        "car slide raises the goods box between floors",
        rest_car_pos is not None and raised_car_pos is not None and raised_car_pos[2] > rest_car_pos[2] + 1.0,
        details=f"rest={rest_car_pos}, raised={raised_car_pos}",
    )

    for hatch, hinge, label in ((lower_hatch, lower_hinge, "lower"), (upper_hatch, upper_hinge, "upper")):
        closed_aabb = ctx.part_element_world_aabb(hatch, elem="door_panel")
        with ctx.pose({hinge: 1.20}):
            open_aabb = ctx.part_element_world_aabb(hatch, elem="door_panel")
        ctx.check(
            f"{label} hatch swings outward from the top hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][1] < closed_aabb[0][1] - 0.20
            and open_aabb[0][2] > closed_aabb[0][2] + 0.10,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
