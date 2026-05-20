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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_countertop_toaster_oven")

    brushed_steel = model.material("brushed_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.10, 0.12, 0.15, 0.38))
    chrome = model.material("chrome", rgba=(0.83, 0.84, 0.87, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.46, 0.47, 0.49, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))

    outer_w = 0.43
    outer_d = 0.33
    outer_h = 0.26
    wall_t = 0.015
    bottom_t = 0.018
    top_t = 0.020
    front_t = 0.008

    opening_center_x = -0.03625
    door_w = 0.324
    door_h = 0.187
    door_t = 0.022
    stile_w = 0.028
    top_rail_h = 0.032
    bottom_rail_h = 0.022

    tray_w = 0.292
    tray_pan_d = 0.246
    tray_wall_t = 0.002
    tray_side_h = 0.012
    tray_front_d = 0.012
    tray_front_h = 0.018

    body = model.part("housing")
    body.visual(
        Box((outer_w, outer_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=brushed_steel,
        name="body_floor",
    )
    body.visual(
        Box((wall_t, outer_d, outer_h - bottom_t - top_t)),
        origin=Origin(
            xyz=(
                -outer_w / 2.0 + wall_t / 2.0,
                0.0,
                bottom_t + (outer_h - bottom_t - top_t) / 2.0,
            )
        ),
        material=brushed_steel,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_d, outer_h - bottom_t - top_t)),
        origin=Origin(
            xyz=(
                outer_w / 2.0 - wall_t / 2.0,
                0.0,
                bottom_t + (outer_h - bottom_t - top_t) / 2.0,
            )
        ),
        material=brushed_steel,
        name="right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, outer_h - bottom_t - top_t)),
        origin=Origin(
            xyz=(
                0.0,
                -outer_d / 2.0 + wall_t / 2.0,
                bottom_t + (outer_h - bottom_t - top_t) / 2.0,
            )
        ),
        material=brushed_steel,
        name="back_wall",
    )
    body.visual(
        Box((outer_w, outer_d, top_t)),
        origin=Origin(xyz=(0.0, 0.0, outer_h - top_t / 2.0)),
        material=brushed_steel,
        name="top_shell",
    )
    body.visual(
        Box((0.020, front_t, 0.190)),
        origin=Origin(xyz=(-0.205, outer_d / 2.0 - front_t / 2.0, 0.113)),
        material=brushed_steel,
        name="front_left_trim",
    )
    body.visual(
        Box((0.090, front_t, 0.220)),
        origin=Origin(xyz=(0.1675, outer_d / 2.0 - front_t / 2.0, 0.128)),
        material=brushed_steel,
        name="control_panel",
    )
    body.visual(
        Box((0.335, front_t, 0.055)),
        origin=Origin(xyz=(opening_center_x, outer_d / 2.0 - front_t / 2.0, 0.2325)),
        material=brushed_steel,
        name="front_header",
    )
    body.visual(
        Box((0.335, front_t, 0.014)),
        origin=Origin(xyz=(opening_center_x, outer_d / 2.0 - front_t / 2.0, 0.025)),
        material=brushed_steel,
        name="front_sill",
    )
    for name, knob_z in (
        ("knob_top", 0.196),
        ("knob_middle", 0.149),
        ("knob_bottom", 0.102),
    ):
        body.visual(
            Cylinder(radius=0.016, length=0.028),
            origin=Origin(
                xyz=(0.1675, 0.173, knob_z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=knob_black,
            name=name,
        )
    body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, outer_h)),
        mass=8.2,
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2.0)),
    )

    door = model.part("door")
    door.visual(
        Box((stile_w, door_t, door_h)),
        origin=Origin(
            xyz=(-door_w / 2.0 + stile_w / 2.0, 0.0, door_h / 2.0)
        ),
        material=brushed_steel,
        name="door_left_stile",
    )
    door.visual(
        Box((stile_w, door_t, door_h)),
        origin=Origin(
            xyz=(door_w / 2.0 - stile_w / 2.0, 0.0, door_h / 2.0)
        ),
        material=brushed_steel,
        name="door_right_stile",
    )
    door.visual(
        Box((door_w, door_t, top_rail_h)),
        origin=Origin(xyz=(0.0, 0.0, door_h - top_rail_h / 2.0)),
        material=brushed_steel,
        name="door_top_rail",
    )
    door.visual(
        Box((door_w, door_t, bottom_rail_h)),
        origin=Origin(xyz=(0.0, 0.0, bottom_rail_h / 2.0)),
        material=brushed_steel,
        name="door_bottom_rail",
    )
    door.visual(
        Box((door_w - 2.0 * (stile_w - 0.002), 0.006, 0.143)),
        origin=Origin(xyz=(0.0, -0.004, 0.0915)),
        material=dark_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.030, 0.014, 0.022)),
        origin=Origin(xyz=(-0.092, 0.002, 0.130)),
        material=chrome,
        name="handle_base_left",
    )
    door.visual(
        Box((0.030, 0.014, 0.022)),
        origin=Origin(xyz=(0.092, 0.002, 0.130)),
        material=chrome,
        name="handle_base_right",
    )
    door.visual(
        Box((0.014, 0.030, 0.014)),
        origin=Origin(xyz=(-0.092, 0.018, 0.130)),
        material=chrome,
        name="handle_post_left",
    )
    door.visual(
        Box((0.014, 0.030, 0.014)),
        origin=Origin(xyz=(0.092, 0.018, 0.130)),
        material=chrome,
        name="handle_post_right",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.250),
        origin=Origin(
            xyz=(0.0, 0.030, 0.130),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=chrome,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, 0.050, door_h)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.015, door_h / 2.0)),
    )

    tray = model.part("crumb_tray")
    tray.visual(
        Box((tray_w, tray_pan_d, tray_wall_t)),
        origin=Origin(xyz=(0.0, -(tray_pan_d / 2.0 - tray_front_d / 2.0), tray_wall_t / 2.0)),
        material=tray_metal,
        name="tray_bottom",
    )
    tray.visual(
        Box((tray_wall_t, tray_pan_d, tray_side_h)),
        origin=Origin(
            xyz=(
                -tray_w / 2.0 + tray_wall_t / 2.0,
                -(tray_pan_d / 2.0 - tray_front_d / 2.0),
                tray_side_h / 2.0,
            )
        ),
        material=tray_metal,
        name="tray_left_wall",
    )
    tray.visual(
        Box((tray_wall_t, tray_pan_d, tray_side_h)),
        origin=Origin(
            xyz=(
                tray_w / 2.0 - tray_wall_t / 2.0,
                -(tray_pan_d / 2.0 - tray_front_d / 2.0),
                tray_side_h / 2.0,
            )
        ),
        material=tray_metal,
        name="tray_right_wall",
    )
    tray.visual(
        Box((tray_w, tray_wall_t, tray_side_h)),
        origin=Origin(
            xyz=(0.0, -tray_pan_d + tray_front_d / 2.0 + tray_wall_t / 2.0, tray_side_h / 2.0)
        ),
        material=tray_metal,
        name="tray_back_wall",
    )
    tray.visual(
        Box((tray_w, tray_front_d, tray_front_h)),
        origin=Origin(xyz=(0.0, 0.0, tray_front_h / 2.0)),
        material=tray_metal,
        name="tray_front_lip",
    )
    tray.inertial = Inertial.from_geometry(
        Box((tray_w, tray_pan_d + tray_front_d, tray_front_h)),
        mass=0.35,
        origin=Origin(
            xyz=(0.0, -(tray_pan_d / 2.0 - tray_front_d / 4.0), tray_front_h / 2.0)
        ),
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(
            xyz=(opening_center_x, outer_d / 2.0 + door_t / 2.0, bottom_t)
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "housing_to_crumb_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(opening_center_x, 0.144, bottom_t)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.22,
            lower=0.0,
            upper=0.11,
        ),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    crumb_tray = object_model.get_part("crumb_tray")
    door_hinge = object_model.get_articulation("housing_to_door")
    tray_slide = object_model.get_articulation("housing_to_crumb_tray")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        door,
        housing,
        axis="y",
        positive_elem="door_top_rail",
        negative_elem="front_header",
        max_gap=0.002,
        max_penetration=0.0005,
        name="door top rail seats against the oven header",
    )
    ctx.expect_gap(
        door,
        housing,
        axis="y",
        positive_elem="door_bottom_rail",
        negative_elem="front_sill",
        max_gap=0.002,
        max_penetration=0.0005,
        name="door bottom rail seats against the oven sill",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        min_overlap=0.17,
        name="door covers the cooking opening in the closed pose",
    )
    ctx.expect_contact(
        crumb_tray,
        housing,
        elem_a="tray_bottom",
        elem_b="body_floor",
        contact_tol=0.0005,
        name="crumb tray rides on the oven floor",
    )
    ctx.expect_within(
        crumb_tray,
        housing,
        axes="x",
        inner_elem="tray_bottom",
        outer_elem="body_floor",
        margin=0.001,
        name="crumb tray stays centered between the oven side walls",
    )

    closed_top_center = _aabb_center(
        ctx.part_element_world_aabb(door, elem="door_top_rail")
    )
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        open_top_center = _aabb_center(
            ctx.part_element_world_aabb(door, elem="door_top_rail")
        )
    ctx.check(
        "door drops forward on its bottom hinge",
        closed_top_center is not None
        and open_top_center is not None
        and open_top_center[1] > closed_top_center[1] + 0.10
        and open_top_center[2] < closed_top_center[2] - 0.08,
        details=f"closed_top_center={closed_top_center}, open_top_center={open_top_center}",
    )

    closed_tray_origin = ctx.part_world_position(crumb_tray)
    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        open_tray_origin = ctx.part_world_position(crumb_tray)
        ctx.expect_overlap(
            crumb_tray,
            housing,
            axes="y",
            elem_a="tray_bottom",
            elem_b="body_floor",
            min_overlap=0.12,
            name="extended crumb tray keeps retained insertion inside the oven",
        )
    ctx.check(
        "crumb tray slides forward from the floor of the opening",
        closed_tray_origin is not None
        and open_tray_origin is not None
        and open_tray_origin[1] > closed_tray_origin[1] + 0.08,
        details=f"closed_tray_origin={closed_tray_origin}, open_tray_origin={open_tray_origin}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
