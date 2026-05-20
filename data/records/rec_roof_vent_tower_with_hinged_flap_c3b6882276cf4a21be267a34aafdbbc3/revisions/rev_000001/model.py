from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = model.material("galvanized_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    flashing = model.material("roof_flashing", rgba=(0.46, 0.48, 0.50, 1.0))
    throat_shadow = model.material("throat_shadow", rgba=(0.19, 0.21, 0.23, 1.0))
    flap_finish = model.material("flap_finish", rgba=(0.56, 0.58, 0.60, 1.0))

    flange_width = 0.46
    flange_depth = 0.38
    flange_thickness = 0.012

    tower_width = 0.28
    tower_depth = 0.22
    tower_height = 0.76
    wall_thickness = 0.014
    tower_base_z = 0.008
    front_face_y = tower_depth * 0.5

    outlet_width = 0.23
    outlet_height = 0.19
    outlet_bottom_z = 0.49
    outlet_top_z = outlet_bottom_z + outlet_height
    frame_depth = 0.085
    frame_front_y = front_face_y + frame_depth
    frame_member = 0.022

    housing = model.part("housing")
    housing.visual(
        Box((flange_width, flange_depth, flange_thickness)),
        origin=Origin(xyz=(0.0, 0.0, flange_thickness * 0.5)),
        material=flashing,
        name="roof_flange",
    )
    housing.visual(
        Box((tower_width, tower_depth * 0.78, 0.040)),
        origin=Origin(xyz=(0.0, -0.010, 0.026)),
        material=flashing,
        name="base_curb",
    )
    housing.visual(
        Box((wall_thickness, tower_depth, tower_height)),
        origin=Origin(
            xyz=((tower_width - wall_thickness) * 0.5, 0.0, tower_base_z + tower_height * 0.5)
        ),
        material=galvanized,
        name="right_wall",
    )
    housing.visual(
        Box((wall_thickness, tower_depth, tower_height)),
        origin=Origin(
            xyz=(-(tower_width - wall_thickness) * 0.5, 0.0, tower_base_z + tower_height * 0.5)
        ),
        material=galvanized,
        name="left_wall",
    )
    housing.visual(
        Box((tower_width - 2.0 * wall_thickness + 0.006, wall_thickness, tower_height)),
        origin=Origin(
            xyz=(0.0, -(tower_depth - wall_thickness) * 0.5, tower_base_z + tower_height * 0.5)
        ),
        material=galvanized,
        name="rear_wall",
    )
    housing.visual(
        Box((tower_width - 2.0 * wall_thickness + 0.006, wall_thickness, outlet_bottom_z - tower_base_z + 0.006)),
        origin=Origin(
            xyz=(
                0.0,
                (tower_depth - wall_thickness) * 0.5,
                (tower_base_z + outlet_bottom_z) * 0.5,
            )
        ),
        material=galvanized,
        name="front_plenum",
    )
    housing.visual(
        Box((tower_width + 0.020, tower_depth + 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.004, tower_base_z + tower_height - 0.004)),
        material=galvanized,
        name="roof_cap",
    )

    frame_outer_width = outlet_width + 2.0 * frame_member
    frame_outer_height = outlet_height + 2.0 * frame_member
    side_frame_x = outlet_width * 0.5 + frame_member * 0.5

    housing.visual(
        Box((frame_member, frame_depth, frame_outer_height)),
        origin=Origin(
            xyz=(-side_frame_x, front_face_y + frame_depth * 0.5, outlet_bottom_z + frame_outer_height * 0.5)
        ),
        material=galvanized,
        name="frame_left",
    )
    housing.visual(
        Box((frame_member, frame_depth, frame_outer_height)),
        origin=Origin(
            xyz=(side_frame_x, front_face_y + frame_depth * 0.5, outlet_bottom_z + frame_outer_height * 0.5)
        ),
        material=galvanized,
        name="frame_right",
    )
    housing.visual(
        Box((frame_outer_width, frame_depth, frame_member)),
        origin=Origin(
            xyz=(0.0, front_face_y + frame_depth * 0.5, outlet_top_z + frame_member * 0.5)
        ),
        material=galvanized,
        name="frame_header",
    )
    housing.visual(
        Box((frame_outer_width, frame_depth, frame_member)),
        origin=Origin(
            xyz=(0.0, front_face_y + frame_depth * 0.5, outlet_bottom_z - frame_member * 0.5)
        ),
        material=galvanized,
        name="frame_sill",
    )

    throat_depth = frame_depth - 0.009
    throat_y = front_face_y - 0.003 + throat_depth * 0.5
    throat_height = outlet_height + 0.010
    throat_side_x = outlet_width * 0.5 + 0.006
    housing.visual(
        Box((0.012, throat_depth, throat_height)),
        origin=Origin(xyz=(-throat_side_x, throat_y, outlet_bottom_z + throat_height * 0.5)),
        material=throat_shadow,
        name="throat_left",
    )
    housing.visual(
        Box((0.012, throat_depth, throat_height)),
        origin=Origin(xyz=(throat_side_x, throat_y, outlet_bottom_z + throat_height * 0.5)),
        material=throat_shadow,
        name="throat_right",
    )
    housing.visual(
        Box((outlet_width, throat_depth, 0.012)),
        origin=Origin(xyz=(0.0, throat_y, outlet_top_z + 0.006)),
        material=throat_shadow,
        name="throat_header",
    )
    housing.visual(
        Box((outlet_width, throat_depth, 0.012)),
        origin=Origin(xyz=(0.0, throat_y, outlet_bottom_z - 0.006)),
        material=throat_shadow,
        name="throat_sill",
    )
    housing.inertial = Inertial.from_geometry(
        Box((flange_width, flange_depth, tower_base_z + tower_height)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, (tower_base_z + tower_height) * 0.5)),
    )

    flap_width = outlet_width + 0.014
    flap_height = outlet_height + 0.020
    flap_thickness = 0.008

    weather_flap = model.part("weather_flap")
    weather_flap.visual(
        Box((flap_width, flap_thickness, flap_height)),
        origin=Origin(xyz=(0.0, flap_thickness * 0.5, -flap_height * 0.5)),
        material=flap_finish,
        name="flap_panel",
    )
    weather_flap.visual(
        Box((flap_width, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.009, -0.008)),
        material=flap_finish,
        name="flap_top_hem",
    )
    weather_flap.inertial = Inertial.from_geometry(
        Box((flap_width, 0.018, flap_height)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.009, -flap_height * 0.5)),
    )

    model.articulation(
        "housing_to_weather_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=weather_flap,
        origin=Origin(xyz=(0.0, frame_front_y, outlet_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.18,
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
    weather_flap = object_model.get_part("weather_flap")
    flap_joint = object_model.get_articulation("housing_to_weather_flap")

    ctx.expect_gap(
        weather_flap,
        housing,
        axis="y",
        positive_elem="flap_panel",
        max_gap=0.01,
        max_penetration=0.0,
        name="closed flap sits just proud of the outlet frame",
    )
    ctx.expect_overlap(
        weather_flap,
        housing,
        axes="x",
        elem_a="flap_panel",
        elem_b="frame_sill",
        min_overlap=0.22,
        name="flap spans the outlet width",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(weather_flap, elem="flap_panel")
    with ctx.pose({flap_joint: 1.0}):
        opened_panel_aabb = ctx.part_element_world_aabb(weather_flap, elem="flap_panel")

    closed_center = _aabb_center(closed_panel_aabb)
    opened_center = _aabb_center(opened_panel_aabb)
    ctx.check(
        "flap opens outward and upward",
        closed_center is not None
        and opened_center is not None
        and opened_center[1] > closed_center[1] + 0.06
        and opened_center[2] > closed_center[2] + 0.03,
        details=f"closed_center={closed_center}, opened_center={opened_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
