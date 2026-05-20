from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("walnut", rgba=(0.24, 0.15, 0.09, 1.0))
    espresso = model.material("espresso", rgba=(0.10, 0.07, 0.05, 1.0))
    suede = model.material("suede", rgba=(0.24, 0.24, 0.22, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.84, 1.0))
    cream = model.material("cream", rgba=(0.78, 0.72, 0.62, 1.0))

    body_width = 0.168
    body_depth = 0.138
    body_height = 0.186
    wall = 0.012
    floor = 0.016
    rear_frame_extra = 0.020
    side_wall_height = body_height - 0.014
    spindle_y = body_depth * 0.5 - wall - 0.007
    spindle_z = 0.112

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height + rear_frame_extra)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, (body_height + rear_frame_extra) * 0.5)),
    )

    body.visual(
        Box((body_width, body_depth, floor)),
        origin=Origin(xyz=(0.0, 0.0, floor * 0.5)),
        material=walnut,
        name="body_floor",
    )
    body.visual(
        Box((wall, body_depth, side_wall_height)),
        origin=Origin(
            xyz=(body_width * 0.5 - wall * 0.5, 0.0, side_wall_height * 0.5),
        ),
        material=walnut,
        name="right_wall",
    )
    body.visual(
        Box((wall, body_depth, side_wall_height)),
        origin=Origin(
            xyz=(-body_width * 0.5 + wall * 0.5, 0.0, side_wall_height * 0.5),
        ),
        material=walnut,
        name="left_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall, wall, body_height * 0.90)),
        origin=Origin(
            xyz=(0.0, -body_depth * 0.5 + wall * 0.5, body_height * 0.45),
        ),
        material=walnut,
        name="front_frame",
    )
    body.visual(
        Box((body_width - 2.0 * wall, wall, body_height - 0.016)),
        origin=Origin(
            xyz=(0.0, body_depth * 0.5 - wall * 0.5, (body_height - 0.016) * 0.5),
        ),
        material=walnut,
        name="rear_wall",
    )
    body.visual(
        Box((body_width - 2.4 * wall, body_depth - 2.3 * wall, 0.010)),
        origin=Origin(xyz=(0.0, -0.002, floor + 0.004)),
        material=suede,
        name="interior_base_pad",
    )
    body.visual(
        Box((body_width - 2.0 * wall, 0.020, 0.018)),
        origin=Origin(
            xyz=(0.0, body_depth * 0.5 - wall - 0.010, body_height + rear_frame_extra - 0.009),
        ),
        material=espresso,
        name="hinge_frame_beam",
    )
    body.visual(
        Box((wall, wall, 0.036)),
        origin=Origin(
            xyz=(body_width * 0.5 - wall * 0.5, body_depth * 0.5 - wall * 0.5, 0.188),
        ),
        material=espresso,
        name="right_hinge_upright",
    )
    body.visual(
        Box((wall, wall, 0.036)),
        origin=Origin(
            xyz=(-body_width * 0.5 + wall * 0.5, body_depth * 0.5 - wall * 0.5, 0.188),
        ),
        material=espresso,
        name="left_hinge_upright",
    )
    body.visual(
        Box((0.012, 0.022, 0.060)),
        origin=Origin(
            xyz=(-0.022, spindle_y - 0.004, spindle_z),
        ),
        material=suede,
        name="left_spindle_support_cheek",
    )
    body.visual(
        Box((0.012, 0.022, 0.060)),
        origin=Origin(
            xyz=(0.022, spindle_y - 0.004, spindle_z),
        ),
        material=suede,
        name="right_spindle_support_cheek",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(
            xyz=(0.0, spindle_y + 0.005, spindle_z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=chrome,
        name="spindle_collar",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.024),
        origin=Origin(
            xyz=(-0.060, body_depth * 0.5 - 0.001, body_height + 0.001),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=chrome,
        name="left_hinge_knuckle",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.024),
        origin=Origin(
            xyz=(0.060, body_depth * 0.5 - 0.001, body_height + 0.001),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=chrome,
        name="right_hinge_knuckle",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((body_width + 0.014, body_depth + 0.012, 0.046)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -body_depth * 0.5 - 0.004, -0.018)),
    )
    lid.visual(
        Box((body_width + 0.014, 0.149, 0.008)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 - 0.016, -0.004)),
        material=walnut,
        name="lid_top",
    )
    lid.visual(
        Box((body_width + 0.014, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, -body_depth - 0.005, -0.015)),
        material=walnut,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((0.014, body_depth + 0.012, 0.030)),
        origin=Origin(
            xyz=(body_width * 0.5 + 0.007, -body_depth * 0.5 - 0.006, -0.015),
        ),
        material=walnut,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((0.014, body_depth + 0.012, 0.030)),
        origin=Origin(
            xyz=(-body_width * 0.5 - 0.007, -body_depth * 0.5 - 0.006, -0.015),
        ),
        material=walnut,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((0.096, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.004, -0.006)),
        material=espresso,
        name="lid_hinge_frame",
    )
    lid.visual(
        Box((body_width - 0.018, body_depth - 0.026, 0.006)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 - 0.010, -0.011)),
        material=suede,
        name="lid_inner_liner",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="lid_center_knuckle",
    )

    cradle = model.part("cradle")
    cradle.inertial = Inertial.from_geometry(
        Box((0.078, 0.090, 0.090)),
        mass=0.38,
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
    )
    cradle.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=chrome,
        name="spindle_stub",
    )
    cradle.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=espresso,
        name="cradle_hub",
    )
    cradle.visual(
        Box((0.058, 0.016, 0.070)),
        origin=Origin(xyz=(0.0, -0.037, 0.0)),
        material=suede,
        name="pillow_backplate",
    )
    cradle.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(
                rounded_rect_profile(0.074, 0.088, 0.018, corner_segments=8),
                0.050,
                cap=True,
                closed=True,
            ),
            "cradle_pillow",
        ),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=cream,
        name="cradle_pillow",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, body_depth * 0.5 - 0.001, body_height + 0.001)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, spindle_y, spindle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_joint = object_model.get_articulation("body_to_lid")
    cradle_joint = object_model.get_articulation("body_to_cradle")
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_gap(
        body,
        cradle,
        axis="y",
        positive_elem="spindle_collar",
        negative_elem="spindle_stub",
        min_gap=0.0,
        max_gap=0.001,
        name="cradle spindle seats just in front of collar",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.05}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.05,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_cradle_aabb = ctx.part_world_aabb(cradle)
    with ctx.pose({cradle_joint: pi * 0.5}):
        turned_cradle_aabb = ctx.part_world_aabb(cradle)
    rest_dx = None if rest_cradle_aabb is None else rest_cradle_aabb[1][0] - rest_cradle_aabb[0][0]
    rest_dz = None if rest_cradle_aabb is None else rest_cradle_aabb[1][2] - rest_cradle_aabb[0][2]
    turned_dx = None if turned_cradle_aabb is None else turned_cradle_aabb[1][0] - turned_cradle_aabb[0][0]
    turned_dz = None if turned_cradle_aabb is None else turned_cradle_aabb[1][2] - turned_cradle_aabb[0][2]
    ctx.check(
        "cradle spins about the spindle axis",
        rest_dx is not None
        and rest_dz is not None
        and turned_dx is not None
        and turned_dz is not None
        and turned_dx > rest_dx + 0.008
        and turned_dz < rest_dz - 0.008,
        details=(
            f"rest_dims={(rest_dx, rest_dz)}, "
            f"turned_dims={(turned_dx, turned_dz)}, "
            f"rest={rest_cradle_aabb}, turned={turned_cradle_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
