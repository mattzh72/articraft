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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = model.material("galvanized_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    shadow = model.material("interior_shadow", rgba=(0.18, 0.19, 0.20, 1.0))

    plate_w = 0.58
    plate_d = 0.44
    plate_t = 0.006

    tower_w = 0.30
    tower_d = 0.22
    tower_h = 0.42
    wall_t = 0.012

    outlet_outer_w = 0.270
    outlet_outer_h = 0.186
    outlet_depth = 0.048
    outlet_side_border = 0.023
    outlet_top_border = 0.020

    outlet_inner_w = outlet_outer_w - 2.0 * outlet_side_border
    outlet_inner_h = outlet_outer_h - 2.0 * outlet_top_border

    tower_front_y = -tower_d / 2.0
    outlet_center_y = tower_front_y - outlet_depth / 2.0
    outlet_center_z = 0.318
    outlet_top_z = outlet_center_z + outlet_outer_h / 2.0
    outlet_front_y = outlet_center_y - outlet_depth / 2.0

    housing = model.part("housing")
    housing.visual(
        Box((plate_w, plate_d, plate_t)),
        origin=Origin(xyz=(0.0, 0.0, plate_t / 2.0)),
        material=galvanized,
        name="flashing_plate",
    )
    housing.visual(
        Box((wall_t, tower_d, tower_h)),
        origin=Origin(
            xyz=(
                -tower_w / 2.0 + wall_t / 2.0,
                0.0,
                plate_t + tower_h / 2.0,
            )
        ),
        material=galvanized,
        name="left_wall",
    )
    housing.visual(
        Box((wall_t, tower_d, tower_h)),
        origin=Origin(
            xyz=(
                tower_w / 2.0 - wall_t / 2.0,
                0.0,
                plate_t + tower_h / 2.0,
            )
        ),
        material=galvanized,
        name="right_wall",
    )
    housing.visual(
        Box((tower_w - 2.0 * wall_t, wall_t, tower_h)),
        origin=Origin(
            xyz=(
                0.0,
                tower_d / 2.0 - wall_t / 2.0,
                plate_t + tower_h / 2.0,
            )
        ),
        material=galvanized,
        name="back_wall",
    )
    housing.visual(
        Box((tower_w - 2.0 * wall_t, wall_t, 0.219)),
        origin=Origin(
            xyz=(
                0.0,
                tower_front_y + wall_t / 2.0,
                plate_t + 0.219 / 2.0,
            )
        ),
        material=galvanized,
        name="lower_front_wall",
    )
    housing.visual(
        Box((tower_w + 0.024, tower_d + 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, plate_t + tower_h + 0.005)),
        material=galvanized,
        name="top_cap",
    )
    housing.visual(
        Box((outlet_side_border, outlet_depth, outlet_outer_h)),
        origin=Origin(
            xyz=(
                -outlet_outer_w / 2.0 + outlet_side_border / 2.0,
                outlet_center_y,
                outlet_center_z,
            )
        ),
        material=galvanized,
        name="outlet_left_jamb",
    )
    housing.visual(
        Box((outlet_side_border, outlet_depth, outlet_outer_h)),
        origin=Origin(
            xyz=(
                outlet_outer_w / 2.0 - outlet_side_border / 2.0,
                outlet_center_y,
                outlet_center_z,
            )
        ),
        material=galvanized,
        name="outlet_right_jamb",
    )
    housing.visual(
        Box((outlet_outer_w, outlet_depth, outlet_top_border)),
        origin=Origin(
            xyz=(
                0.0,
                outlet_center_y,
                outlet_center_z + outlet_outer_h / 2.0 - outlet_top_border / 2.0,
            )
        ),
        material=galvanized,
        name="outlet_lintel",
    )
    housing.visual(
        Box((outlet_outer_w, outlet_depth, outlet_top_border)),
        origin=Origin(
            xyz=(
                0.0,
                outlet_center_y,
                outlet_center_z - outlet_outer_h / 2.0 + outlet_top_border / 2.0,
            )
        ),
        material=galvanized,
        name="outlet_sill",
    )
    housing.visual(
        Box((outlet_inner_w, wall_t, outlet_inner_h)),
        origin=Origin(
            xyz=(
                0.0,
                tower_front_y + wall_t / 2.0,
                outlet_center_z,
            )
        ),
        material=shadow,
        name="outlet_shadow",
    )
    housing.inertial = Inertial.from_geometry(
        Box((plate_w, plate_d, plate_t + tower_h + 0.010)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (plate_t + tower_h + 0.010) / 2.0)),
    )

    flap = model.part("weather_flap")
    flap_width = outlet_inner_w - 0.010
    flap_height = outlet_inner_h - 0.004
    flap.visual(
        Box((flap_width, 0.008, flap_height)),
        origin=Origin(xyz=(0.0, -0.004, -flap_height / 2.0)),
        material=galvanized,
        name="panel",
    )
    flap.visual(
        Box((flap_width, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, -0.006, -0.008)),
        material=galvanized,
        name="top_hem",
    )
    flap.inertial = Inertial.from_geometry(
        Box((flap_width, 0.012, flap_height)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -0.006, -flap_height / 2.0)),
    )

    model.articulation(
        "housing_to_weather_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(0.0, outlet_front_y, outlet_top_z - outlet_top_border)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("housing_to_weather_flap")

    ctx.expect_origin_distance(
        flap,
        housing,
        axes="x",
        min_dist=0.0,
        max_dist=0.0001,
        name="flap hinge stays centered on the housing",
    )

    panel_aabb = ctx.part_element_world_aabb(flap, elem="panel")
    left_jamb_aabb = ctx.part_element_world_aabb(housing, elem="outlet_left_jamb")
    right_jamb_aabb = ctx.part_element_world_aabb(housing, elem="outlet_right_jamb")
    lintel_aabb = ctx.part_element_world_aabb(housing, elem="outlet_lintel")

    aabbs_ok = (
        panel_aabb is not None
        and left_jamb_aabb is not None
        and right_jamb_aabb is not None
        and lintel_aabb is not None
    )
    ctx.check(
        "named outlet elements resolve",
        aabbs_ok,
        details=(
            f"panel={panel_aabb}, left={left_jamb_aabb}, "
            f"right={right_jamb_aabb}, lintel={lintel_aabb}"
        ),
    )

    if aabbs_ok:
        left_clearance = panel_aabb[0][0] - left_jamb_aabb[1][0]
        right_clearance = right_jamb_aabb[0][0] - panel_aabb[1][0]
        frame_front_y = min(
            left_jamb_aabb[0][1],
            right_jamb_aabb[0][1],
            lintel_aabb[0][1],
        )
        panel_inner_face_y = panel_aabb[1][1]
        ctx.check(
            "flap panel is centered within the outlet frame",
            left_clearance >= 0.003
            and right_clearance >= 0.003
            and abs(left_clearance - right_clearance) <= 0.002,
            details=(
                f"left_clearance={left_clearance:.4f}, "
                f"right_clearance={right_clearance:.4f}"
            ),
        )
        ctx.check(
            "closed flap sits flush to the framed outlet face",
            abs(panel_inner_face_y - frame_front_y) <= 0.001,
            details=(
                f"panel_inner_face_y={panel_inner_face_y:.4f}, "
                f"frame_front_y={frame_front_y:.4f}"
            ),
        )

    closed_panel_aabb = panel_aabb
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        opened_panel_aabb = ctx.part_element_world_aabb(flap, elem="panel")
        ctx.check(
            "weather flap swings outward",
            closed_panel_aabb is not None
            and opened_panel_aabb is not None
            and opened_panel_aabb[0][1] < closed_panel_aabb[0][1] - 0.050,
            details=f"closed={closed_panel_aabb}, opened={opened_panel_aabb}",
        )
        ctx.check(
            "weather flap lifts from the outlet when opened",
            closed_panel_aabb is not None
            and opened_panel_aabb is not None
            and opened_panel_aabb[0][2] > closed_panel_aabb[0][2] + 0.040,
            details=f"closed={closed_panel_aabb}, opened={opened_panel_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
