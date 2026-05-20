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
    model = ArticulatedObject(name="rooftop_vent_tower")

    housing_paint = model.material("housing_paint", rgba=(0.58, 0.60, 0.62, 1.0))
    flashing_metal = model.material("flashing_metal", rgba=(0.46, 0.47, 0.49, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.30, 0.31, 0.33, 1.0))

    housing = model.part("housing")

    curb_width = 1.02
    curb_depth = 0.66
    curb_height = 0.10

    shell_width = 0.88
    shell_depth = 0.50
    shell_height = 0.60
    wall_thickness = 0.03

    front_face_x = shell_depth * 0.5
    rear_face_x = -shell_depth * 0.5
    wall_center_z = curb_height + shell_height * 0.5

    housing.visual(
        Box((curb_depth, curb_width, curb_height)),
        origin=Origin(xyz=(0.0, 0.0, curb_height * 0.5)),
        material=flashing_metal,
        name="roof_curb",
    )
    housing.visual(
        Box((shell_depth, shell_width, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, curb_height + wall_thickness * 0.5)),
        material=trim_dark,
        name="plenum_floor",
    )
    housing.visual(
        Box((shell_depth, wall_thickness, shell_height)),
        origin=Origin(
            xyz=(0.0, shell_width * 0.5 - wall_thickness * 0.5, wall_center_z),
        ),
        material=housing_paint,
        name="left_wall",
    )
    housing.visual(
        Box((shell_depth, wall_thickness, shell_height)),
        origin=Origin(
            xyz=(0.0, -shell_width * 0.5 + wall_thickness * 0.5, wall_center_z),
        ),
        material=housing_paint,
        name="right_wall",
    )
    housing.visual(
        Box((wall_thickness, shell_width, shell_height)),
        origin=Origin(xyz=(rear_face_x + wall_thickness * 0.5, 0.0, wall_center_z)),
        material=housing_paint,
        name="rear_wall",
    )
    housing.visual(
        Box((shell_depth + 0.04, shell_width + 0.04, 0.04)),
        origin=Origin(
            xyz=(0.0, 0.0, curb_height + shell_height + 0.04),
            rpy=(0.0, -0.12, 0.0),
        ),
        material=flashing_metal,
        name="top_cap",
    )
    housing.visual(
        Box((0.08, shell_width + 0.02, 0.05)),
        origin=Origin(xyz=(front_face_x - 0.01, 0.0, curb_height + shell_height + 0.01)),
        material=flashing_metal,
        name="cap_front_lip",
    )
    housing.visual(
        Box((0.05, shell_width - 0.06, 0.18)),
        origin=Origin(xyz=(front_face_x - 0.025, 0.0, curb_height + 0.13)),
        material=housing_paint,
        name="front_apron",
    )

    opening_width = 0.72
    opening_height = 0.30
    opening_bottom_z = curb_height + 0.21
    opening_center_z = opening_bottom_z + opening_height * 0.5
    jamb_width = 0.065
    frame_depth = 0.05
    frame_center_x = front_face_x - frame_depth * 0.5
    head_height = 0.065
    sill_height = 0.045

    jamb_center_y = opening_width * 0.5 + jamb_width * 0.5
    housing.visual(
        Box((frame_depth, jamb_width, opening_height)),
        origin=Origin(xyz=(frame_center_x, jamb_center_y, opening_center_z)),
        material=trim_dark,
        name="left_jamb",
    )
    housing.visual(
        Box((frame_depth, jamb_width, opening_height)),
        origin=Origin(xyz=(frame_center_x, -jamb_center_y, opening_center_z)),
        material=trim_dark,
        name="right_jamb",
    )
    housing.visual(
        Box((frame_depth, opening_width + 2.0 * jamb_width, head_height)),
        origin=Origin(
            xyz=(
                frame_center_x,
                0.0,
                opening_bottom_z + opening_height + head_height * 0.5,
            )
        ),
        material=trim_dark,
        name="outlet_head",
    )
    housing.visual(
        Box((0.03, opening_width + 0.18, 0.03)),
        origin=Origin(
            xyz=(
                front_face_x - 0.015,
                0.0,
                opening_bottom_z + opening_height + 0.015,
            )
        ),
        material=flashing_metal,
        name="hinge_mount_rail",
    )
    housing.visual(
        Box((frame_depth, opening_width + 2.0 * jamb_width, sill_height)),
        origin=Origin(
            xyz=(frame_center_x, 0.0, opening_bottom_z - sill_height * 0.5),
        ),
        material=trim_dark,
        name="outlet_sill",
    )

    housing.inertial = Inertial.from_geometry(
        Box((curb_depth, curb_width, curb_height + shell_height + 0.12)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )

    flap = model.part("weather_flap")

    flap_width = 0.70
    flap_height = 0.32
    flap_thickness = 0.012
    barrel_radius = 0.009
    barrel_length = 0.74

    flap.visual(
        Cylinder(radius=barrel_radius, length=barrel_length),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.022, flap_width, 0.024)),
        origin=Origin(xyz=(0.009, 0.0, -0.011)),
        material=trim_dark,
        name="top_stiffener",
    )
    flap.visual(
        Box((flap_thickness, flap_width, flap_height)),
        origin=Origin(
            xyz=(
                0.018,
                0.0,
                -0.5 * flap_height - 0.016,
            )
        ),
        material=housing_paint,
        name="panel",
    )
    flap.visual(
        Box((0.016, flap_width - 0.05, 0.020)),
        origin=Origin(
            xyz=(
                0.022,
                0.0,
                -flap_height - 0.008,
            )
        ),
        material=trim_dark,
        name="bottom_stiffener",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.05, flap_width, flap_height + 0.05)),
        mass=7.5,
        origin=Origin(xyz=(0.02, 0.0, -0.17)),
    )

    hinge_axis_x = front_face_x + 0.009
    hinge_axis_z = opening_bottom_z + opening_height + 0.012
    model.articulation(
        "housing_to_weather_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
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
    flap = object_model.get_part("weather_flap")
    flap_joint = object_model.get_articulation("housing_to_weather_flap")

    with ctx.pose({flap_joint: 0.0}):
        ctx.expect_contact(
            flap,
            housing,
            elem_a="hinge_barrel",
            elem_b="hinge_mount_rail",
            contact_tol=1e-4,
            name="hinge barrel stays mounted to the housing rail",
        )
        ctx.expect_gap(
            flap,
            housing,
            axis="x",
            positive_elem="panel",
            negative_elem="front_apron",
            min_gap=0.015,
            max_gap=0.03,
            name="closed flap sits just proud of the housing face",
        )
        ctx.expect_overlap(
            flap,
            housing,
            axes="y",
            elem_a="panel",
            elem_b="outlet_head",
            min_overlap=0.68,
            name="flap spans most of the outlet width",
        )

        closed_panel = ctx.part_element_world_aabb(flap, elem="panel")

    with ctx.pose({flap_joint: 1.05}):
        open_panel = ctx.part_element_world_aabb(flap, elem="panel")

    opens_outward = (
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][0] > closed_panel[1][0] + 0.10
        and open_panel[0][2] > closed_panel[0][2] + 0.12
    )
    ctx.check(
        "weather flap opens upward and outward",
        opens_outward,
        details=f"closed_panel={closed_panel}, open_panel={open_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
