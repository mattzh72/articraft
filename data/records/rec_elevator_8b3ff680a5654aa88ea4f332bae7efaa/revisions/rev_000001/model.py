from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dumbwaiter")

    shaft_paint = model.material("shaft_paint", rgba=(0.74, 0.75, 0.78, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    car_paint = model.material("car_paint", rgba=(0.63, 0.66, 0.70, 1.0))
    interior_panel = model.material("interior_panel", rgba=(0.88, 0.88, 0.86, 1.0))
    wood = model.material("wood", rgba=(0.54, 0.37, 0.22, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.20, 0.20, 0.21, 1.0))

    shaft_outer = 0.82
    shaft_half = shaft_outer * 0.5
    wall_t = 0.045
    base_t = 0.06
    top_t = 0.05
    wall_h = 2.34
    top_z = base_t + wall_h

    opening_w = 0.56
    jamb_w = (shaft_outer - opening_w) * 0.5
    lower_open_bottom = 0.26
    lower_open_top = 0.84
    upper_open_bottom = 1.40
    upper_open_top = 1.98

    car_w = 0.52
    car_d = 0.52
    car_h = 0.56
    car_wall_t = 0.015
    car_floor_t = 0.018
    rear_stile_depth = 0.05
    lower_stop_z = lower_open_bottom
    upper_stop_z = upper_open_bottom
    travel = upper_stop_z - lower_stop_z

    shaft = model.part("shaft")
    shaft.visual(
        Box((shaft_outer, shaft_outer, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t * 0.5)),
        material=shaft_paint,
        name="base_slab",
    )
    shaft.visual(
        Box((wall_t, shaft_outer, wall_h)),
        origin=Origin(xyz=(-shaft_half + wall_t * 0.5, 0.0, base_t + wall_h * 0.5)),
        material=shaft_paint,
        name="left_wall",
    )
    shaft.visual(
        Box((wall_t, shaft_outer, wall_h)),
        origin=Origin(xyz=(shaft_half - wall_t * 0.5, 0.0, base_t + wall_h * 0.5)),
        material=shaft_paint,
        name="right_wall",
    )
    shaft.visual(
        Box((shaft_outer, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -shaft_half + wall_t * 0.5, base_t + wall_h * 0.5)),
        material=shaft_paint,
        name="back_wall",
    )
    shaft.visual(
        Box((shaft_outer, shaft_outer, top_t)),
        origin=Origin(xyz=(0.0, 0.0, top_z + top_t * 0.5)),
        material=shaft_paint,
        name="top_cap",
    )
    shaft.visual(
        Box((shaft_outer, wall_t, lower_open_bottom - base_t)),
        origin=Origin(
            xyz=(0.0, shaft_half - wall_t * 0.5, base_t + (lower_open_bottom - base_t) * 0.5)
        ),
        material=shaft_paint,
        name="lower_face_panel",
    )
    shaft.visual(
        Box((shaft_outer, wall_t, upper_open_bottom - lower_open_top)),
        origin=Origin(
            xyz=(
                0.0,
                shaft_half - wall_t * 0.5,
                lower_open_top + (upper_open_bottom - lower_open_top) * 0.5,
            )
        ),
        material=shaft_paint,
        name="mid_spandrel",
    )
    shaft.visual(
        Box((shaft_outer, wall_t, top_z - upper_open_top)),
        origin=Origin(
            xyz=(0.0, shaft_half - wall_t * 0.5, upper_open_top + (top_z - upper_open_top) * 0.5)
        ),
        material=shaft_paint,
        name="upper_face_panel",
    )
    for side_name, x_center in (
        ("left", -opening_w * 0.5 - jamb_w * 0.5),
        ("right", opening_w * 0.5 + jamb_w * 0.5),
    ):
        shaft.visual(
            Box((jamb_w, wall_t, lower_open_top - lower_open_bottom)),
            origin=Origin(
                xyz=(
                    x_center,
                    shaft_half - wall_t * 0.5,
                    lower_open_bottom + (lower_open_top - lower_open_bottom) * 0.5,
                )
            ),
            material=shaft_paint,
            name=f"lower_{side_name}_jamb",
        )
        shaft.visual(
            Box((jamb_w, wall_t, upper_open_top - upper_open_bottom)),
            origin=Origin(
                xyz=(
                    x_center,
                    shaft_half - wall_t * 0.5,
                    upper_open_bottom + (upper_open_top - upper_open_bottom) * 0.5,
                )
            ),
            material=shaft_paint,
            name=f"upper_{side_name}_jamb",
        )
    shaft.visual(
        Box((opening_w + 0.08, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, shaft_half - wall_t - 0.015, lower_open_bottom + 0.015)),
        material=dark_trim,
        name="lower_sill",
    )
    shaft.visual(
        Box((opening_w + 0.08, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, shaft_half - wall_t - 0.015, upper_open_bottom + 0.015)),
        material=dark_trim,
        name="upper_sill",
    )
    for index, x_pos in enumerate((-0.25, 0.25)):
        shaft.visual(
            Cylinder(radius=0.012, length=wall_h),
            origin=Origin(xyz=(x_pos, -0.329, base_t + wall_h * 0.5)),
            material=rail_steel,
            name=f"guide_rail_{index}",
        )
    shaft.inertial = Inertial.from_geometry(
        Box((shaft_outer, shaft_outer, top_z + top_t)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, (top_z + top_t) * 0.5)),
    )

    car = model.part("car")
    car.visual(
        Box((car_w, car_d, car_floor_t)),
        origin=Origin(xyz=(0.0, 0.0, car_floor_t * 0.5)),
        material=car_paint,
        name="car_floor",
    )
    car.visual(
        Box((car_wall_t, car_d, car_h)),
        origin=Origin(xyz=(-car_w * 0.5 + car_wall_t * 0.5, 0.0, car_h * 0.5)),
        material=car_paint,
        name="left_side",
    )
    car.visual(
        Box((car_wall_t, car_d, car_h)),
        origin=Origin(xyz=(car_w * 0.5 - car_wall_t * 0.5, 0.0, car_h * 0.5)),
        material=car_paint,
        name="right_side",
    )
    car.visual(
        Box((car_w, car_wall_t, car_h)),
        origin=Origin(xyz=(0.0, -car_d * 0.5 + car_wall_t * 0.5, car_h * 0.5)),
        material=car_paint,
        name="back_wall",
    )
    car.visual(
        Box((car_w, car_d, car_floor_t)),
        origin=Origin(xyz=(0.0, 0.0, car_h - car_floor_t * 0.5)),
        material=interior_panel,
        name="car_roof",
    )
    car.visual(
        Box((car_w, car_wall_t, 0.08)),
        origin=Origin(xyz=(0.0, car_d * 0.5 - car_wall_t * 0.5, 0.04)),
        material=car_paint,
        name="front_sill",
    )
    car.visual(
        Box((car_w, car_wall_t, 0.08)),
        origin=Origin(xyz=(0.0, car_d * 0.5 - car_wall_t * 0.5, car_h - 0.04)),
        material=car_paint,
        name="front_header",
    )
    car.visual(
        Box((0.036, rear_stile_depth + 0.01, car_h)),
        origin=Origin(xyz=(-0.232, -0.287, car_h * 0.5)),
        material=dark_trim,
        name="left_guide_stile",
    )
    car.visual(
        Box((0.036, rear_stile_depth + 0.01, car_h)),
        origin=Origin(xyz=(0.232, -0.287, car_h * 0.5)),
        material=dark_trim,
        name="right_guide_stile",
    )
    car.visual(
        Box((0.50, 0.04, 0.06)),
        origin=Origin(xyz=(0.0, -0.287, car_h - 0.055)),
        material=dark_trim,
        name="guide_crosshead",
    )
    car.inertial = Inertial.from_geometry(
        Box((car_w, car_d + 0.05, car_h)),
        mass=9.0,
        origin=Origin(xyz=(0.0, -0.015, car_h * 0.5)),
    )

    lower_hatch = model.part("lower_hatch")
    lower_hatch.visual(
        Box((0.54, 0.03, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, -0.28)),
        material=wood,
        name="lower_panel",
    )
    lower_hatch.visual(
        Cylinder(radius=0.009, length=0.50),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_trim,
        name="lower_hinge_barrel",
    )
    lower_hatch.visual(
        Box((0.12, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.024, -0.46)),
        material=dark_trim,
        name="lower_pull",
    )
    lower_hatch.inertial = Inertial.from_geometry(
        Box((0.54, 0.05, 0.56)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.015, -0.28)),
    )

    upper_hatch = model.part("upper_hatch")
    upper_hatch.visual(
        Box((0.54, 0.03, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, -0.28)),
        material=wood,
        name="upper_panel",
    )
    upper_hatch.visual(
        Cylinder(radius=0.009, length=0.50),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_trim,
        name="upper_hinge_barrel",
    )
    upper_hatch.visual(
        Box((0.12, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.024, -0.46)),
        material=dark_trim,
        name="upper_pull",
    )
    upper_hatch.inertial = Inertial.from_geometry(
        Box((0.54, 0.05, 0.56)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.015, -0.28)),
    )

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, lower_stop_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.35,
            lower=0.0,
            upper=travel,
        ),
    )
    model.articulation(
        "shaft_to_lower_hatch",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=lower_hatch,
        origin=Origin(xyz=(0.0, shaft_half + 0.015, lower_open_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "shaft_to_upper_hatch",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=upper_hatch,
        origin=Origin(xyz=(0.0, shaft_half + 0.015, upper_open_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
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

    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    lower_hatch = object_model.get_part("lower_hatch")
    upper_hatch = object_model.get_part("upper_hatch")
    car_lift = object_model.get_articulation("shaft_to_car")
    lower_hinge = object_model.get_articulation("shaft_to_lower_hatch")
    upper_hinge = object_model.get_articulation("shaft_to_upper_hatch")
    car_upper = car_lift.motion_limits.upper if car_lift.motion_limits is not None else 1.14
    hatch_open = 1.20

    ctx.expect_within(
        car,
        shaft,
        axes="xy",
        margin=0.0,
        name="car stays within shaft plan at lower stop",
    )
    ctx.expect_gap(
        lower_hatch,
        shaft,
        axis="y",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="lower_panel",
        name="lower hatch closes flush to shaft face",
    )
    ctx.expect_gap(
        upper_hatch,
        shaft,
        axis="y",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="upper_panel",
        name="upper hatch closes flush to shaft face",
    )

    lower_car_aabb = ctx.part_world_aabb(car)
    lower_panel_aabb = ctx.part_element_world_aabb(lower_hatch, elem="lower_panel")
    upper_panel_aabb = ctx.part_element_world_aabb(upper_hatch, elem="upper_panel")

    with ctx.pose({car_lift: car_upper}):
        ctx.expect_within(
            car,
            shaft,
            axes="xy",
            margin=0.0,
            name="car stays within shaft plan at upper stop",
        )
        upper_car_aabb = ctx.part_world_aabb(car)

    with ctx.pose({lower_hinge: hatch_open}):
        lower_panel_open_aabb = ctx.part_element_world_aabb(lower_hatch, elem="lower_panel")

    with ctx.pose({upper_hinge: hatch_open}):
        upper_panel_open_aabb = ctx.part_element_world_aabb(upper_hatch, elem="upper_panel")

    ctx.check(
        "car reaches both floor landings",
        lower_car_aabb is not None
        and upper_car_aabb is not None
        and 0.24 <= lower_car_aabb[0][2] <= 0.28
        and 1.38 <= upper_car_aabb[0][2] <= 1.42,
        details=f"lower={lower_car_aabb}, upper={upper_car_aabb}",
    )
    ctx.check(
        "lower hatch opens upward from its top edge",
        lower_panel_aabb is not None
        and lower_panel_open_aabb is not None
        and lower_panel_open_aabb[0][2] > lower_panel_aabb[0][2] + 0.18
        and lower_panel_open_aabb[1][1] > lower_panel_aabb[1][1] + 0.18,
        details=f"closed={lower_panel_aabb}, open={lower_panel_open_aabb}",
    )
    ctx.check(
        "upper hatch opens upward from its top edge",
        upper_panel_aabb is not None
        and upper_panel_open_aabb is not None
        and upper_panel_open_aabb[0][2] > upper_panel_aabb[0][2] + 0.18
        and upper_panel_open_aabb[1][1] > upper_panel_aabb[1][1] + 0.18,
        details=f"closed={upper_panel_aabb}, open={upper_panel_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
