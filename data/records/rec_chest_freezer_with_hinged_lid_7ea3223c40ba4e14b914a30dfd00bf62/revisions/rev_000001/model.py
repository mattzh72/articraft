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
    model = ArticulatedObject(name="outdoor_patio_chest_freezer")

    body_finish = model.material("body_finish", rgba=(0.31, 0.35, 0.38, 1.0))
    liner_white = model.material("liner_white", rgba=(0.93, 0.95, 0.97, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.35, 0.39, 0.42, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.09, 0.10, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.79, 0.82, 1.0))
    latch_black = model.material("latch_black", rgba=(0.11, 0.12, 0.13, 1.0))
    foot_black = model.material("foot_black", rgba=(0.09, 0.09, 0.10, 1.0))

    width = 1.28
    depth = 0.72
    height = 0.88
    base_clearance = 0.025
    outer_skin_t = 0.018
    rim_t = 0.030
    inner_floor_t = 0.006
    inner_wall_t = 0.006
    inner_width = 1.10
    inner_depth = 0.54
    inner_floor_top = 0.128
    mate = 0.001

    lid_width = 1.32
    lid_depth = 0.75
    hinge_radius = 0.012
    hinge_axis_y = -(depth * 0.5) - hinge_radius
    hinge_axis_z = height + hinge_radius

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    outer_wall_height = height - rim_t + mate - base_clearance
    outer_wall_center_z = base_clearance + (outer_wall_height * 0.5)
    inner_wall_height = height - rim_t + mate - inner_floor_top
    inner_wall_center_z = inner_floor_top + (inner_wall_height * 0.5)

    body.visual(
        Box((width, depth, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, base_clearance + 0.010)),
        material=body_finish,
        name="outer_base",
    )
    body.visual(
        Box((width, outer_skin_t, outer_wall_height)),
        origin=Origin(xyz=(0.0, (depth * 0.5) - (outer_skin_t * 0.5), outer_wall_center_z)),
        material=body_finish,
        name="outer_front",
    )
    body.visual(
        Box((width, outer_skin_t, outer_wall_height)),
        origin=Origin(xyz=(0.0, -(depth * 0.5) + (outer_skin_t * 0.5), outer_wall_center_z)),
        material=body_finish,
        name="outer_rear",
    )
    body.visual(
        Box((outer_skin_t, depth - (2.0 * outer_skin_t), outer_wall_height)),
        origin=Origin(xyz=((width * 0.5) - (outer_skin_t * 0.5), 0.0, outer_wall_center_z)),
        material=body_finish,
        name="outer_right",
    )
    body.visual(
        Box((outer_skin_t, depth - (2.0 * outer_skin_t), outer_wall_height)),
        origin=Origin(xyz=(-(width * 0.5) + (outer_skin_t * 0.5), 0.0, outer_wall_center_z)),
        material=body_finish,
        name="outer_left",
    )

    body.visual(
        Box((inner_width, inner_depth, inner_floor_t)),
        origin=Origin(xyz=(0.0, 0.0, inner_floor_top - (inner_floor_t * 0.5))),
        material=liner_white,
        name="inner_floor",
    )
    body.visual(
        Box((inner_width, inner_wall_t, inner_wall_height)),
        origin=Origin(
            xyz=(0.0, (inner_depth * 0.5) - (inner_wall_t * 0.5), inner_wall_center_z)
        ),
        material=liner_white,
        name="inner_front",
    )
    body.visual(
        Box((inner_width, inner_wall_t, inner_wall_height)),
        origin=Origin(
            xyz=(0.0, -(inner_depth * 0.5) + (inner_wall_t * 0.5), inner_wall_center_z)
        ),
        material=liner_white,
        name="inner_rear",
    )
    body.visual(
        Box((inner_wall_t, inner_depth - (2.0 * inner_wall_t), inner_wall_height)),
        origin=Origin(
            xyz=((inner_width * 0.5) - (inner_wall_t * 0.5), 0.0, inner_wall_center_z)
        ),
        material=liner_white,
        name="inner_right",
    )
    body.visual(
        Box((inner_wall_t, inner_depth - (2.0 * inner_wall_t), inner_wall_height)),
        origin=Origin(
            xyz=(-(inner_width * 0.5) + (inner_wall_t * 0.5), 0.0, inner_wall_center_z)
        ),
        material=liner_white,
        name="inner_left",
    )

    rim_band_y = (depth - inner_depth) * 0.5
    rim_band_x = (width - inner_width) * 0.5
    body.visual(
        Box((width, rim_band_y, rim_t)),
        origin=Origin(
            xyz=(0.0, (inner_depth * 0.5) + (rim_band_y * 0.5), height - (rim_t * 0.5))
        ),
        material=body_finish,
        name="rim_front",
    )
    body.visual(
        Box((width, rim_band_y, rim_t)),
        origin=Origin(
            xyz=(0.0, -(inner_depth * 0.5) - (rim_band_y * 0.5), height - (rim_t * 0.5))
        ),
        material=body_finish,
        name="rim_rear",
    )
    body.visual(
        Box((rim_band_x, inner_depth, rim_t)),
        origin=Origin(
            xyz=((inner_width * 0.5) + (rim_band_x * 0.5), 0.0, height - (rim_t * 0.5))
        ),
        material=body_finish,
        name="rim_right",
    )
    body.visual(
        Box((rim_band_x, inner_depth, rim_t)),
        origin=Origin(
            xyz=(-(inner_width * 0.5) - (rim_band_x * 0.5), 0.0, height - (rim_t * 0.5))
        ),
        material=body_finish,
        name="rim_left",
    )

    body.visual(
        Box((0.33, 0.18, 0.13)),
        origin=Origin(
            xyz=(
                (inner_width * 0.5) - 0.165 - 0.035,
                -(inner_depth * 0.5) + 0.090,
                inner_floor_top + 0.065,
            )
        ),
        material=liner_white,
        name="compressor_hump",
    )

    for foot_name, sx, sy in (
        ("front_right_foot", 1.0, 1.0),
        ("front_left_foot", -1.0, 1.0),
        ("rear_right_foot", 1.0, -1.0),
        ("rear_left_foot", -1.0, -1.0),
    ):
        body.visual(
            Box((0.070, 0.050, base_clearance + mate)),
            origin=Origin(
                xyz=(
                    sx * ((width * 0.5) - 0.105),
                    sy * ((depth * 0.5) - 0.080),
                    (base_clearance + mate) * 0.5,
                )
            ),
            material=foot_black,
            name=foot_name,
        )

    for pad_name, sign_x in (("right_latch_pad", 1.0), ("left_latch_pad", -1.0)):
        body.visual(
            Box((0.007, 0.050, 0.060)),
            origin=Origin(xyz=(sign_x * ((width * 0.5) + 0.0035), 0.120, 0.718)),
            material=body_finish,
            name=pad_name,
        )

    hinge_x_positions = (-0.435, 0.435)
    hinge_barrel_len = 0.100

    for prefix, hinge_x in zip(("left", "right"), hinge_x_positions):
        body.visual(
            Box((0.110, 0.012, 0.060)),
            origin=Origin(xyz=(hinge_x, -(depth * 0.5) - 0.006, height - 0.020)),
            material=stainless,
            name=f"{prefix}_hinge_mount",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=hinge_barrel_len),
            origin=Origin(
                xyz=(hinge_x, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=stainless,
            name=f"{prefix}_hinge_barrel",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, 0.094)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.370, 0.022)),
    )

    lid.visual(
        Box((lid_width, lid_depth, 0.034)),
        origin=Origin(xyz=(0.0, 0.367, 0.040)),
        material=lid_finish,
        name="top_shell",
    )
    lid.visual(
        Box((1.20, 0.650, 0.017)),
        origin=Origin(xyz=(0.0, 0.366, 0.0145)),
        material=lid_finish,
        name="insulation_core",
    )
    lid.visual(
        Box((1.20, 0.620, 0.006)),
        origin=Origin(xyz=(0.0, 0.366, 0.003)),
        material=liner_white,
        name="underside_liner",
    )
    lid.visual(
        Box((0.700, 0.024, 0.052)),
        origin=Origin(xyz=(0.0, 0.004, 0.020)),
        material=lid_finish,
        name="rear_skirt_center",
    )
    lid.visual(
        Box((0.140, 0.024, 0.052)),
        origin=Origin(xyz=(0.590, 0.004, 0.020)),
        material=lid_finish,
        name="rear_skirt_right_end",
    )
    lid.visual(
        Box((0.140, 0.024, 0.052)),
        origin=Origin(xyz=(-0.590, 0.004, 0.020)),
        material=lid_finish,
        name="rear_skirt_left_end",
    )
    lid.visual(
        Box((lid_width, 0.032, 0.058)),
        origin=Origin(xyz=(0.0, 0.733, 0.017)),
        material=lid_finish,
        name="front_skirt",
    )
    lid.visual(
        Box((0.032, 0.697, 0.058)),
        origin=Origin(xyz=((lid_width * 0.5) - 0.016, 0.369, 0.017)),
        material=lid_finish,
        name="right_skirt",
    )
    lid.visual(
        Box((0.032, 0.697, 0.058)),
        origin=Origin(xyz=(-(lid_width * 0.5) + 0.016, 0.369, 0.017)),
        material=lid_finish,
        name="left_skirt",
    )

    lid.visual(
        Box((1.26, 0.052, 0.006)),
        origin=Origin(xyz=(0.0, 0.687, -0.009)),
        material=gasket_black,
        name="gasket_front",
    )
    lid.visual(
        Box((1.26, 0.052, 0.006)),
        origin=Origin(xyz=(0.0, 0.057, -0.009)),
        material=gasket_black,
        name="gasket_rear",
    )
    lid.visual(
        Box((0.052, 0.582, 0.006)),
        origin=Origin(xyz=(0.595, 0.372, -0.009)),
        material=gasket_black,
        name="gasket_right",
    )
    lid.visual(
        Box((0.052, 0.582, 0.006)),
        origin=Origin(xyz=(-0.595, 0.372, -0.009)),
        material=gasket_black,
        name="gasket_left",
    )

    for prefix, hinge_x in zip(("left", "right"), hinge_x_positions):
        lid.visual(
            Box((0.110, 0.022, 0.010)),
            origin=Origin(xyz=(hinge_x, 0.020, 0.018)),
            material=stainless,
            name=f"{prefix}_hinge_strap",
        )

    lid.visual(
        Box((0.012, 0.032, 0.016)),
        origin=Origin(xyz=((lid_width * 0.5) - 0.010, 0.492, -0.004)),
        material=stainless,
        name="right_strike",
    )
    lid.visual(
        Box((0.012, 0.032, 0.016)),
        origin=Origin(xyz=(-(lid_width * 0.5) + 0.010, 0.492, -0.004)),
        material=stainless,
        name="left_strike",
    )

    right_latch = model.part("right_latch")
    right_latch.inertial = Inertial.from_geometry(
        Box((0.022, 0.040, 0.180)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )
    right_latch.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=latch_black,
        name="pivot",
    )
    right_latch.visual(
        Box((0.010, 0.026, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=latch_black,
        name="housing",
    )
    right_latch.visual(
        Box((0.010, 0.018, 0.122)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=latch_black,
        name="arm",
    )
    right_latch.visual(
        Box((0.010, 0.038, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        material=latch_black,
        name="hook",
    )
    right_latch.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.0, -0.010, 0.096), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=stainless,
        name="lock_loop",
    )

    left_latch = model.part("left_latch")
    left_latch.inertial = Inertial.from_geometry(
        Box((0.022, 0.040, 0.180)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )
    left_latch.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=latch_black,
        name="pivot",
    )
    left_latch.visual(
        Box((0.010, 0.026, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=latch_black,
        name="housing",
    )
    left_latch.visual(
        Box((0.010, 0.018, 0.122)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=latch_black,
        name="arm",
    )
    left_latch.visual(
        Box((0.010, 0.038, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        material=latch_black,
        name="hook",
    )
    left_latch.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.0, -0.010, 0.096), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=stainless,
        name="lock_loop",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "body_to_right_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_latch,
        origin=Origin(xyz=((width * 0.5) + 0.014, 0.120, 0.718)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-1.25,
            upper=0.10,
        ),
    )
    model.articulation(
        "body_to_left_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_latch,
        origin=Origin(xyz=(-(width * 0.5) - 0.014, 0.120, 0.718)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-1.25,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body_top_aabb = None
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    lid_hinge = object_model.get_articulation("body_to_lid")
    left_latch_hinge = object_model.get_articulation("body_to_left_latch")
    right_latch_hinge = object_model.get_articulation("body_to_right_latch")

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

    ctx.expect_contact(right_latch, body, elem_a="pivot", elem_b="right_latch_pad")
    ctx.expect_contact(left_latch, body, elem_a="pivot", elem_b="left_latch_pad")
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="gasket_front",
        negative_elem="rim_front",
        max_gap=0.0025,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        lid,
        right_latch,
        axis="z",
        positive_elem="right_strike",
        negative_elem="hook",
        max_gap=0.0025,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        lid,
        left_latch,
        axis="z",
        positive_elem="left_strike",
        negative_elem="hook",
        max_gap=0.0025,
        max_penetration=0.0,
    )

    right_hook_rest = ctx.part_element_world_aabb(right_latch, elem="hook")
    left_hook_rest = ctx.part_element_world_aabb(left_latch, elem="hook")
    body_top_aabb = ctx.part_world_aabb(body)
    if right_hook_rest is None or left_hook_rest is None or body_top_aabb is None:
        ctx.fail(
            "pose_measurements_available",
            "Could not resolve latch-hook or body world AABBs for articulation checks.",
        )
        return ctx.report()

    with ctx.pose({lid_hinge: math.radians(72.0)}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="gasket_front",
            negative_elem="rim_front",
            min_gap=0.22,
        )
        open_front = ctx.part_element_world_aabb(lid, elem="gasket_front")
        ctx.check(
            "lid_open_front_lifts_clear",
            open_front is not None and open_front[0][2] > body_top_aabb[1][2] + 0.16,
            details=(
                "Expected the front gasket edge to lift well above the body top when the lid is open."
            ),
        )

    with ctx.pose({right_latch_hinge: -1.10, left_latch_hinge: -1.10}):
        right_hook_open = ctx.part_element_world_aabb(right_latch, elem="hook")
        left_hook_open = ctx.part_element_world_aabb(left_latch, elem="hook")
        ctx.check(
            "right_latch_swings_down_and_forward",
            right_hook_open is not None
            and right_hook_open[1][2] < right_hook_rest[1][2] - 0.05
            and right_hook_open[0][1] > right_hook_rest[0][1] + 0.08,
            details=(
                "Expected the right latch hook to rotate downward and forward in its open pose."
            ),
        )
        ctx.check(
            "left_latch_swings_down_and_forward",
            left_hook_open is not None
            and left_hook_open[1][2] < left_hook_rest[1][2] - 0.05
            and left_hook_open[0][1] > left_hook_rest[0][1] + 0.08,
            details=(
                "Expected the left latch hook to rotate downward and forward in its open pose."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
