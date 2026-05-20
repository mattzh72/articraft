from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    model = ArticulatedObject(name="duct_mounted_air_purifier")

    housing_color = model.material("housing_color", rgba=(0.84, 0.86, 0.88, 1.0))
    trim_color = model.material("trim_color", rgba=(0.66, 0.69, 0.72, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.22, 0.24, 0.25, 1.0))
    filter_media_color = model.material("filter_media_color", rgba=(0.56, 0.74, 0.77, 0.95))

    housing_width = 1.20
    housing_depth = 0.56
    housing_height = 0.46
    wall = 0.03

    opening_bottom_z = 0.055
    opening_top_z = 0.400
    opening_height = opening_top_z - opening_bottom_z
    opening_center_z = (opening_bottom_z + opening_top_z) * 0.5

    housing = model.part("housing")
    housing.visual(
        Box((housing_width, housing_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=housing_color,
        name="bottom_shell",
    )
    housing.visual(
        Box((housing_width, housing_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, housing_height - wall * 0.5)),
        material=housing_color,
        name="top_shell",
    )
    housing.visual(
        Box((wall, housing_depth, housing_height)),
        origin=Origin(xyz=(housing_width * 0.5 - wall * 0.5, 0.0, housing_height * 0.5)),
        material=housing_color,
        name="right_wall",
    )
    housing.visual(
        Box((wall, housing_depth, housing_height)),
        origin=Origin(xyz=(-housing_width * 0.5 + wall * 0.5, 0.0, housing_height * 0.5)),
        material=housing_color,
        name="left_wall",
    )
    housing.visual(
        Box((housing_width, wall, housing_height)),
        origin=Origin(xyz=(0.0, -housing_depth * 0.5 + wall * 0.5, housing_height * 0.5)),
        material=housing_color,
        name="front_shell",
    )

    housing.visual(
        Box((1.10, 0.05, housing_height - opening_top_z)),
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - 0.025,
                opening_top_z + (housing_height - opening_top_z) * 0.5,
            )
        ),
        material=trim_color,
        name="rear_top_rail",
    )
    housing.visual(
        Box((1.10, 0.05, opening_bottom_z)),
        origin=Origin(
            xyz=(0.0, housing_depth * 0.5 - 0.025, opening_bottom_z * 0.5)
        ),
        material=trim_color,
        name="rear_bottom_sill",
    )
    jamb_width = 0.055
    jamb_center_x = housing_width * 0.5 - wall - jamb_width * 0.5
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        housing.visual(
            Box((jamb_width, 0.05, opening_height)),
            origin=Origin(
                xyz=(x_sign * jamb_center_x, housing_depth * 0.5 - 0.025, opening_center_z)
            ),
            material=trim_color,
            name=f"rear_{side}_jamb",
        )

    housing.visual(
        Box((0.14, 0.24, 0.012)),
        origin=Origin(xyz=(-0.53, 0.04, opening_center_z - 0.0235)),
        material=trim_color,
        name="left_filter_shelf",
    )
    housing.visual(
        Box((0.14, 0.24, 0.012)),
        origin=Origin(xyz=(0.53, 0.04, opening_center_z - 0.0235)),
        material=trim_color,
        name="right_filter_shelf",
    )
    housing.visual(
        Box((0.96, 0.48, 0.012)),
        origin=Origin(xyz=(0.0, -0.02, 0.424)),
        material=trim_color,
        name="upper_filter_guide",
    )

    collar_radius = 0.12
    collar_length = 0.104
    flange_length = 0.018
    collar_x = housing_width * 0.5 + 0.047
    flange_x = housing_width * 0.5 + flange_length * 0.5 - 0.003
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        housing.visual(
            Cylinder(radius=0.137, length=flange_length),
            origin=Origin(
                xyz=(x_sign * flange_x, 0.0, housing_height * 0.5),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=steel,
            name=f"{side}_duct_flange",
        )
        housing.visual(
            Cylinder(radius=collar_radius, length=collar_length),
            origin=Origin(
                xyz=(x_sign * collar_x, 0.0, housing_height * 0.5),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=steel,
            name=f"{side}_duct_collar",
        )

    housing.visual(
        Box((0.18, 0.10, 0.08)),
        origin=Origin(xyz=(0.34, -0.12, housing_height + 0.04)),
        material=trim_color,
        name="control_box",
    )
    housing.visual(
        Box((0.14, 0.006, 0.035)),
        origin=Origin(
            xyz=(0.34, -housing_depth * 0.5 - 0.003, housing_height * 0.5 + 0.08)
        ),
        material=dark_hardware,
        name="status_window",
    )

    hinge_axis_y = housing_depth * 0.5
    hinge_axis_z = opening_top_z
    hinge_x_positions = (-0.34, 0.34)
    for index, hinge_x in enumerate(hinge_x_positions):
        housing.visual(
            Box((0.12, 0.04, 0.022)),
            origin=Origin(xyz=(hinge_x, hinge_axis_y + 0.010, hinge_axis_z + 0.023)),
            material=trim_color,
            name=f"hinge_mount_{index}",
        )
        housing.visual(
            Box((0.014, 0.036, 0.032)),
            origin=Origin(xyz=(hinge_x - 0.049, hinge_axis_y + 0.038, hinge_axis_z + 0.001)),
            material=dark_hardware,
            name=f"hinge_ear_{index}_a",
        )
        housing.visual(
            Box((0.014, 0.036, 0.032)),
            origin=Origin(xyz=(hinge_x + 0.049, hinge_axis_y + 0.038, hinge_axis_z + 0.001)),
            material=dark_hardware,
            name=f"hinge_ear_{index}_b",
        )

    housing.inertial = Inertial.from_geometry(
        Box((1.32, 0.62, 0.54)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, housing_height * 0.5)),
    )

    rear_door = model.part("rear_door")
    rear_door.visual(
        Box((1.08, 0.024, opening_height)),
        origin=Origin(xyz=(0.0, 0.012, -opening_height * 0.5)),
        material=trim_color,
        name="door_panel",
    )
    rear_door.visual(
        Box((0.16, 0.028, 0.040)),
        origin=Origin(xyz=(0.0, 0.030, -opening_height + 0.060)),
        material=dark_hardware,
        name="service_pull",
    )
    for side, x_pos in (("left", -0.34), ("right", 0.34)):
        rear_door.visual(
            Box((0.10, 0.024, 0.014)),
            origin=Origin(xyz=(x_pos, 0.012, -0.007)),
            material=dark_hardware,
            name=f"{side}_hinge_leaf",
        )
        rear_door.visual(
            Cylinder(radius=0.011, length=0.078),
            origin=Origin(
                xyz=(x_pos, 0.012, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=dark_hardware,
            name=f"{side}_hinge_knuckle",
        )

    rear_door.inertial = Inertial.from_geometry(
        Box((1.08, 0.06, opening_height)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.018, -opening_height * 0.5)),
    )

    filter_frame = model.part("filter_frame")
    filter_frame.visual(
        Box((0.035, 0.320, 0.035)),
        origin=Origin(xyz=(-0.46, -0.140, 0.0)),
        material=dark_hardware,
        name="left_runner",
    )
    filter_frame.visual(
        Box((0.035, 0.320, 0.035)),
        origin=Origin(xyz=(0.46, -0.140, 0.0)),
        material=dark_hardware,
        name="right_runner",
    )
    filter_frame.visual(
        Box((0.930, 0.060, 0.030)),
        origin=Origin(xyz=(0.0, -0.020, 0.155)),
        material=dark_hardware,
        name="top_frame",
    )
    filter_frame.visual(
        Box((0.930, 0.060, 0.030)),
        origin=Origin(xyz=(0.0, -0.020, -0.155)),
        material=dark_hardware,
        name="bottom_frame",
    )
    filter_frame.visual(
        Box((0.035, 0.060, 0.310)),
        origin=Origin(xyz=(-0.4475, -0.020, 0.0)),
        material=dark_hardware,
        name="left_frame_side",
    )
    filter_frame.visual(
        Box((0.035, 0.060, 0.310)),
        origin=Origin(xyz=(0.4475, -0.020, 0.0)),
        material=dark_hardware,
        name="right_frame_side",
    )
    filter_frame.visual(
        Box((0.895, 0.040, 0.270)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=filter_media_color,
        name="filter_media",
    )
    filter_frame.visual(
        Box((0.14, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_color,
        name="filter_pull_tab",
    )

    filter_frame.inertial = Inertial.from_geometry(
        Box((0.96, 0.34, 0.34)),
        mass=2.0,
        origin=Origin(xyz=(0.0, -0.115, 0.0)),
    )

    model.articulation(
        "rear_access_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=rear_door,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "filter_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_frame,
        origin=Origin(xyz=(0.0, 0.230, opening_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.25,
            lower=0.0,
            upper=0.20,
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
    rear_door = object_model.get_part("rear_door")
    filter_frame = object_model.get_part("filter_frame")
    rear_access_hinge = object_model.get_articulation("rear_access_hinge")
    filter_slide = object_model.get_articulation("filter_slide")

    door_panel = rear_door.get_visual("door_panel")
    filter_media = filter_frame.get_visual("filter_media")

    ctx.expect_overlap(
        rear_door,
        housing,
        axes="xz",
        min_overlap=0.30,
        elem_a=door_panel,
        name="rear door covers the service opening",
    )
    ctx.expect_gap(
        rear_door,
        filter_frame,
        axis="y",
        min_gap=0.015,
        positive_elem=door_panel,
        name="closed door clears the parked filter frame",
    )
    ctx.expect_overlap(
        filter_frame,
        housing,
        axes="y",
        min_overlap=0.22,
        name="filter frame remains inserted in the housing at rest",
    )

    door_closed_aabb = ctx.part_element_world_aabb(rear_door, elem=door_panel)
    filter_rest_position = ctx.part_world_position(filter_frame)
    filter_slide_upper = 0.20
    if (
        filter_slide.motion_limits is not None
        and filter_slide.motion_limits.upper is not None
    ):
        filter_slide_upper = filter_slide.motion_limits.upper

    with ctx.pose({rear_access_hinge: 1.20, filter_slide: filter_slide_upper}):
        ctx.expect_overlap(
            filter_frame,
            housing,
            axes="y",
            min_overlap=0.14,
            name="filter frame retains guide engagement when extended",
        )
        door_open_aabb = ctx.part_element_world_aabb(rear_door, elem=door_panel)
        filter_extended_position = ctx.part_world_position(filter_frame)

    ctx.check(
        "rear access door swings outward from the top edge",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[1][1] > door_closed_aabb[1][1] + 0.18,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )
    ctx.check(
        "filter frame slides rearward out of the housing",
        filter_rest_position is not None
        and filter_extended_position is not None
        and filter_extended_position[1] > filter_rest_position[1] + 0.15,
        details=f"rest={filter_rest_position}, extended={filter_extended_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
