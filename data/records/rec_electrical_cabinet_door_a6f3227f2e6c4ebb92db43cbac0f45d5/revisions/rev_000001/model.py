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
    model = ArticulatedObject(name="flush_wall_consumer_unit")

    housing_plastic = model.material("housing_plastic", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_plastic = model.material("trim_plastic", rgba=(0.86, 0.87, 0.89, 1.0))
    inner_plastic = model.material("inner_plastic", rgba=(0.77, 0.79, 0.81, 1.0))
    smoked_window = model.material("smoked_window", rgba=(0.18, 0.22, 0.25, 0.42))
    latch_plastic = model.material("latch_plastic", rgba=(0.95, 0.95, 0.96, 1.0))

    outer_width = 0.400
    outer_height = 0.560
    bezel_depth = 0.012
    aperture_width = 0.364
    aperture_height = 0.524

    shell_depth = 0.096
    shell_outer_width = 0.344
    shell_outer_height = 0.504
    shell_thickness = 0.0045

    door_width = 0.358
    door_height = 0.518
    door_thickness = 0.018
    hinge_axis_y = 0.008
    hinge_axis_z = door_height * 0.5

    housing = model.part("housing")

    bezel_side_width = (outer_width - aperture_width) * 0.5
    bezel_top_height = (outer_height - aperture_height) * 0.5
    housing.visual(
        Box((outer_width, bezel_depth, bezel_top_height)),
        origin=Origin(xyz=(0.0, 0.0, outer_height * 0.5 - bezel_top_height * 0.5)),
        material=trim_plastic,
        name="top_bezel",
    )
    housing.visual(
        Box((outer_width, bezel_depth, bezel_top_height)),
        origin=Origin(xyz=(0.0, 0.0, -outer_height * 0.5 + bezel_top_height * 0.5)),
        material=trim_plastic,
        name="bottom_bezel",
    )
    housing.visual(
        Box((bezel_side_width, bezel_depth, outer_height)),
        origin=Origin(xyz=(-outer_width * 0.5 + bezel_side_width * 0.5, 0.0, 0.0)),
        material=trim_plastic,
        name="left_bezel",
    )
    housing.visual(
        Box((bezel_side_width, bezel_depth, outer_height)),
        origin=Origin(xyz=(outer_width * 0.5 - bezel_side_width * 0.5, 0.0, 0.0)),
        material=trim_plastic,
        name="right_bezel",
    )

    housing.visual(
        Box((shell_thickness, shell_depth, shell_outer_height)),
        origin=Origin(
            xyz=(-shell_outer_width * 0.5 + shell_thickness * 0.5, -shell_depth * 0.5, 0.0)
        ),
        material=housing_plastic,
        name="left_wall",
    )
    housing.visual(
        Box((shell_thickness, shell_depth, shell_outer_height)),
        origin=Origin(
            xyz=(shell_outer_width * 0.5 - shell_thickness * 0.5, -shell_depth * 0.5, 0.0)
        ),
        material=housing_plastic,
        name="right_wall",
    )
    housing.visual(
        Box((shell_outer_width, shell_depth, shell_thickness)),
        origin=Origin(
            xyz=(0.0, -shell_depth * 0.5, shell_outer_height * 0.5 - shell_thickness * 0.5)
        ),
        material=housing_plastic,
        name="top_wall",
    )
    housing.visual(
        Box((shell_outer_width, shell_depth, shell_thickness)),
        origin=Origin(
            xyz=(0.0, -shell_depth * 0.5, -shell_outer_height * 0.5 + shell_thickness * 0.5)
        ),
        material=housing_plastic,
        name="bottom_wall",
    )
    housing.visual(
        Box((shell_outer_width, shell_thickness, shell_outer_height)),
        origin=Origin(xyz=(0.0, -shell_depth + shell_thickness * 0.5, 0.0)),
        material=inner_plastic,
        name="back_pan",
    )
    housing.visual(
        Box((0.012, 0.018, aperture_height)),
        origin=Origin(xyz=(-0.177, -0.003, 0.0)),
        material=trim_plastic,
        name="left_jamb",
    )
    housing.visual(
        Box((0.012, 0.018, aperture_height)),
        origin=Origin(xyz=(0.177, -0.003, 0.0)),
        material=trim_plastic,
        name="right_jamb",
    )
    housing.visual(
        Box((aperture_width, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.003, 0.259)),
        material=trim_plastic,
        name="top_jamb",
    )
    housing.visual(
        Box((aperture_width, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.003, -0.259)),
        material=trim_plastic,
        name="bottom_jamb",
    )
    housing.visual(
        Box((0.028, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.001, -0.266)),
        material=inner_plastic,
        name="latch_keeper",
    )

    hinge_barrel_radius = 0.008
    hinge_barrel_length = 0.056
    hinge_x = 0.142
    for side_name, x_pos in (("left", -hinge_x), ("right", hinge_x)):
        housing.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(
                xyz=(x_pos, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=housing_plastic,
            name=f"{side_name}_hinge_knuckle",
        )

    housing.inertial = Inertial.from_geometry(
        Box((outer_width, shell_depth + bezel_depth, outer_height)),
        mass=4.8,
        origin=Origin(xyz=(0.0, -0.044, 0.0)),
    )

    door = model.part("door")
    top_rail_height = 0.032
    bottom_rail_height = 0.044
    side_rail_width = 0.024
    window_height = door_height - top_rail_height - bottom_rail_height
    window_width = door_width - 2.0 * side_rail_width
    side_rail_height = window_height

    door.visual(
        Box((0.224, door_thickness, top_rail_height)),
        origin=Origin(xyz=(0.0, door_thickness * 0.5, -top_rail_height * 0.5)),
        material=housing_plastic,
        name="door_top_rail",
    )
    door.visual(
        Box((0.043, door_thickness, 0.018)),
        origin=Origin(xyz=(-0.1335, door_thickness * 0.5, -0.023)),
        material=housing_plastic,
        name="door_top_left_shoulder",
    )
    door.visual(
        Box((0.043, door_thickness, 0.018)),
        origin=Origin(xyz=(0.1335, door_thickness * 0.5, -0.023)),
        material=housing_plastic,
        name="door_top_right_shoulder",
    )
    door.visual(
        Box((door_width, door_thickness, bottom_rail_height)),
        origin=Origin(
            xyz=(0.0, door_thickness * 0.5, -door_height + bottom_rail_height * 0.5)
        ),
        material=housing_plastic,
        name="door_bottom_rail",
    )
    door.visual(
        Box((side_rail_width, door_thickness, side_rail_height)),
        origin=Origin(
            xyz=(
                -door_width * 0.5 + side_rail_width * 0.5,
                door_thickness * 0.5,
                -top_rail_height - side_rail_height * 0.5,
            )
        ),
        material=housing_plastic,
        name="door_left_rail",
    )
    door.visual(
        Box((side_rail_width, door_thickness, side_rail_height)),
        origin=Origin(
            xyz=(
                door_width * 0.5 - side_rail_width * 0.5,
                door_thickness * 0.5,
                -top_rail_height - side_rail_height * 0.5,
            )
        ),
        material=housing_plastic,
        name="door_right_rail",
    )
    door.visual(
        Box((window_width, 0.004, window_height)),
        origin=Origin(
            xyz=(
                0.0,
                0.006,
                -top_rail_height - window_height * 0.5,
            )
        ),
        material=smoked_window,
        name="door_window",
    )
    door.visual(
        Cylinder(radius=0.0065, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=housing_plastic,
        name="door_hinge_knuckle",
    )
    for ear_name, x_pos in (("left", -0.010), ("right", 0.010)):
        door.visual(
            Box((0.006, 0.014, 0.016)),
            origin=Origin(xyz=(x_pos, 0.021, -door_height + 0.020)),
            material=housing_plastic,
            name=f"latch_pivot_{ear_name}_ear",
        )

    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=1.25,
        origin=Origin(xyz=(0.0, door_thickness * 0.5, -door_height * 0.5)),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=latch_plastic,
        name="latch_pivot_barrel",
    )
    latch.visual(
        Box((0.020, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.005, -0.012)),
        material=latch_plastic,
        name="latch_body",
    )
    latch.visual(
        Box((0.014, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.003, -0.025)),
        material=latch_plastic,
        name="latch_hook",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.020, 0.010, 0.034)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.004, -0.015)),
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.6,
            lower=0.0,
            upper=2.15,
        ),
    )
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.0, 0.024, -door_height + 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.05,
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

    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("housing_to_door")
    latch_pivot = object_model.get_articulation("door_to_latch")

    with ctx.pose({door_hinge: 0.0, latch_pivot: 0.0}):
        ctx.expect_gap(
            door,
            housing,
            axis="y",
            positive_elem="door_bottom_rail",
            negative_elem="bottom_bezel",
            min_gap=0.001,
            max_gap=0.006,
            name="door sits just proud of the bottom bezel when closed",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="xz",
            elem_a="door_window",
            elem_b="back_pan",
            min_overlap=0.240,
            name="door covers the breaker cavity in the closed pose",
        )
        ctx.expect_overlap(
            latch,
            housing,
            axes="xz",
            elem_a="latch_hook",
            elem_b="latch_keeper",
            min_overlap=0.008,
            name="closed latch lines up with the housing keeper",
        )

        closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_bottom_rail")
        closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_hook")

    with ctx.pose({door_hinge: 1.25, latch_pivot: 0.0}):
        opened_door_aabb = ctx.part_element_world_aabb(door, elem="door_bottom_rail")

    ctx.check(
        "top-hinged door swings outward",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][1] > closed_door_aabb[1][1] + 0.18,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    with ctx.pose({door_hinge: 0.0, latch_pivot: 0.85}):
        released_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_hook")

    ctx.check(
        "bottom-center latch flips forward on its pivot",
        closed_latch_aabb is not None
        and released_latch_aabb is not None
        and released_latch_aabb[1][1] > closed_latch_aabb[1][1] + 0.012,
        details=f"closed={closed_latch_aabb}, released={released_latch_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
