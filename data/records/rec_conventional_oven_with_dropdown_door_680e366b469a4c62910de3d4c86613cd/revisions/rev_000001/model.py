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
    model = ArticulatedObject(name="steam_combi_oven")

    body_w = 0.64
    body_d = 0.58
    body_h = 0.74
    wall = 0.018

    door_w = 0.552
    door_h = 0.556
    door_t = 0.026

    drawer_opening_y = 0.20
    drawer_opening_z = 0.11
    drawer_center_y = 0.15
    drawer_center_z = 0.61

    pod_w = 0.58
    pod_d = 0.14
    pod_h = 0.05

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.15, 0.16, 0.17, 1.0))
    glass = model.material("glass", rgba=(0.12, 0.16, 0.19, 0.30))
    black_glass = model.material("black_glass", rgba=(0.06, 0.08, 0.09, 0.65))
    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.90, 1.0))
    water = model.material("water", rgba=(0.48, 0.73, 0.89, 0.30))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")

    # Main housing shell.
    body.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=stainless,
        name="floor",
    )
    body.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_h - wall * 0.5)),
        material=stainless,
        name="roof",
    )
    body.visual(
        Box((body_w, wall, body_h)),
        origin=Origin(xyz=(0.0, -body_d * 0.5 + wall * 0.5, body_h * 0.5)),
        material=stainless,
        name="back_wall",
    )
    body.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(body_w * 0.5 - wall * 0.5, 0.0, body_h * 0.5)),
        material=stainless,
        name="right_wall",
    )

    opening_y_min = drawer_center_y - drawer_opening_y * 0.5
    opening_y_max = drawer_center_y + drawer_opening_y * 0.5
    opening_z_min = drawer_center_z - drawer_opening_z * 0.5
    opening_z_max = drawer_center_z + drawer_opening_z * 0.5

    rear_strip_d = opening_y_min - (-body_d * 0.5)
    front_strip_d = body_d * 0.5 - opening_y_max
    top_strip_h = body_h - opening_z_max
    bottom_strip_h = opening_z_min

    body.visual(
        Box((wall, rear_strip_d, body_h)),
        origin=Origin(
            xyz=(
                -body_w * 0.5 + wall * 0.5,
                -body_d * 0.5 + rear_strip_d * 0.5,
                body_h * 0.5,
            )
        ),
        material=stainless,
        name="left_wall_rear",
    )
    body.visual(
        Box((wall, front_strip_d, body_h)),
        origin=Origin(
            xyz=(
                -body_w * 0.5 + wall * 0.5,
                opening_y_max + front_strip_d * 0.5,
                body_h * 0.5,
            )
        ),
        material=stainless,
        name="left_wall_front",
    )
    body.visual(
        Box((wall, drawer_opening_y, bottom_strip_h)),
        origin=Origin(
            xyz=(
                -body_w * 0.5 + wall * 0.5,
                drawer_center_y,
                bottom_strip_h * 0.5,
            )
        ),
        material=stainless,
        name="left_wall_bottom",
    )
    body.visual(
        Box((wall, drawer_opening_y, top_strip_h)),
        origin=Origin(
            xyz=(
                -body_w * 0.5 + wall * 0.5,
                drawer_center_y,
                opening_z_max + top_strip_h * 0.5,
            )
        ),
        material=stainless,
        name="left_wall_top",
    )

    # Front oven opening frame.
    opening_w = 0.54
    opening_h = 0.54
    frame_side_w = (body_w - opening_w) * 0.5
    frame_header_h = body_h - wall - opening_h - 0.06
    frame_sill_h = 0.06
    opening_z0 = frame_sill_h

    body.visual(
        Box((frame_side_w, wall, body_h - wall)),
        origin=Origin(
            xyz=(
                -body_w * 0.5 + frame_side_w * 0.5,
                body_d * 0.5 - wall * 0.5,
                (body_h - wall) * 0.5,
            )
        ),
        material=stainless,
        name="front_left_jamb",
    )
    body.visual(
        Box((frame_side_w, wall, body_h - wall)),
        origin=Origin(
            xyz=(
                body_w * 0.5 - frame_side_w * 0.5,
                body_d * 0.5 - wall * 0.5,
                (body_h - wall) * 0.5,
            )
        ),
        material=stainless,
        name="front_right_jamb",
    )
    body.visual(
        Box((opening_w, wall, frame_sill_h)),
        origin=Origin(
            xyz=(
                0.0,
                body_d * 0.5 - wall * 0.5,
                frame_sill_h * 0.5,
            )
        ),
        material=stainless,
        name="front_sill",
    )
    body.visual(
        Box((opening_w, wall, frame_header_h)),
        origin=Origin(
            xyz=(
                0.0,
                body_d * 0.5 - wall * 0.5,
                opening_z0 + opening_h + frame_header_h * 0.5,
            )
        ),
        material=stainless,
        name="front_header",
    )

    # Subtle side drawer guide cavity so the drawer reads supported.
    guide_length = 0.15
    guide_rail_y = 0.012
    lower_rail_z = opening_z_min - 0.013
    upper_rail_z = opening_z_max + 0.004
    lower_rail_y = drawer_center_y - (drawer_opening_y * 0.5 - guide_rail_y * 0.5)
    upper_rail_y = drawer_center_y + (drawer_opening_y * 0.5 - guide_rail_y * 0.5)
    body.visual(
        Box((guide_length, guide_rail_y, 0.006)),
        origin=Origin(
            xyz=(
                -body_w * 0.5 + guide_length * 0.5,
                lower_rail_y,
                lower_rail_z,
            )
        ),
        material=dark_trim,
        name="drawer_guide_bottom",
    )
    body.visual(
        Box((guide_length, guide_rail_y, 0.006)),
        origin=Origin(
            xyz=(
                -body_w * 0.5 + guide_length * 0.5,
                upper_rail_y,
                lower_rail_z,
            )
        ),
        material=dark_trim,
        name="drawer_guide_bottom_outer",
    )
    body.visual(
        Box((guide_length, guide_rail_y, 0.006)),
        origin=Origin(
            xyz=(
                -body_w * 0.5 + guide_length * 0.5,
                lower_rail_y,
                upper_rail_z,
            )
        ),
        material=dark_trim,
        name="drawer_guide_top",
    )
    body.visual(
        Box((guide_length, guide_rail_y, 0.006)),
        origin=Origin(
            xyz=(
                -body_w * 0.5 + guide_length * 0.5,
                upper_rail_y,
                upper_rail_z,
            )
        ),
        material=dark_trim,
        name="drawer_guide_top_outer",
    )
    # Top control pod and fascia.
    pod_center_y = body_d * 0.5 - pod_d * 0.5
    body.visual(
        Box((pod_w, pod_d, pod_h)),
        origin=Origin(
            xyz=(
                0.0,
                pod_center_y,
                body_h + pod_h * 0.5,
            )
        ),
        material=stainless,
        name="control_pod",
    )
    body.visual(
        Box((pod_w, wall, 0.048)),
        origin=Origin(
            xyz=(
                0.0,
                body_d * 0.5 - wall * 0.5,
                body_h + 0.024,
            )
        ),
        material=stainless,
        name="control_face",
    )
    body.visual(
        Box((0.44, 0.006, 0.030)),
        origin=Origin(
            xyz=(
                0.0,
                body_d * 0.5 + 0.001,
                body_h + 0.025,
            )
        ),
        material=black_glass,
        name="display_strip",
    )

    # Lower feet.
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((0.055, 0.050, 0.012)),
                origin=Origin(
                    xyz=(
                        sx * (body_w * 0.5 - 0.055),
                        sy * (body_d * 0.5 - 0.055),
                        0.006,
                    )
                ),
                material=rubber,
                name=f"foot_{'l' if sx < 0 else 'r'}_{'rear' if sy < 0 else 'front'}",
            )

    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h + pod_h)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, (body_h + pod_h) * 0.5)),
    )

    door = model.part("door")
    stile_w = 0.050
    top_rail_h = 0.090
    bottom_rail_h = 0.070

    door.visual(
        Box((stile_w, door_t, door_h)),
        origin=Origin(
            xyz=(-door_w * 0.5 + stile_w * 0.5, door_t * 0.5, door_h * 0.5),
        ),
        material=stainless,
        name="left_stile",
    )
    door.visual(
        Box((stile_w, door_t, door_h)),
        origin=Origin(
            xyz=(door_w * 0.5 - stile_w * 0.5, door_t * 0.5, door_h * 0.5),
        ),
        material=stainless,
        name="right_stile",
    )
    door.visual(
        Box((door_w - 2.0 * stile_w, door_t, bottom_rail_h)),
        origin=Origin(
            xyz=(0.0, door_t * 0.5, bottom_rail_h * 0.5),
        ),
        material=stainless,
        name="bottom_rail",
    )
    door.visual(
        Box((door_w - 2.0 * stile_w, door_t, top_rail_h)),
        origin=Origin(
            xyz=(0.0, door_t * 0.5, door_h - top_rail_h * 0.5),
        ),
        material=stainless,
        name="top_rail",
    )
    door.visual(
        Box((door_w - 0.092, 0.010, door_h - 0.156)),
        origin=Origin(
            xyz=(0.0, 0.013, bottom_rail_h + (door_h - 0.156) * 0.5),
        ),
        material=glass,
        name="glass_panel",
    )
    handle_z = door_h - 0.072
    handle_y = 0.038
    door.visual(
        Cylinder(radius=0.011, length=0.42),
        origin=Origin(
            xyz=(0.0, handle_y, handle_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=chrome,
        name="handle_bar",
    )
    for side, handle_x in (("left", -0.15), ("right", 0.15)):
        door.visual(
            Cylinder(radius=0.008, length=0.052),
            origin=Origin(
                xyz=(handle_x, 0.018, handle_z),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=chrome,
            name=f"handle_post_{side}",
        )

    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=8.0,
        origin=Origin(xyz=(0.0, door_t * 0.5, door_h * 0.5)),
    )

    drawer = model.part("water_reservoir_drawer")
    drawer_length = 0.122
    drawer_width = 0.172
    drawer_height = 0.072
    fascia_t = 0.010
    fascia_h = 0.118
    floor_t = 0.004
    wall_t = 0.004

    drawer.visual(
        Box((fascia_t, drawer_opening_y + 0.020, fascia_h)),
        origin=Origin(xyz=(-fascia_t * 0.5, 0.0, fascia_h * 0.5)),
        material=stainless,
        name="drawer_fascia",
    )
    drawer.visual(
        Box((0.020, 0.110, 0.012)),
        origin=Origin(xyz=(-0.016, 0.0, fascia_h - 0.016)),
        material=chrome,
        name="drawer_pull",
    )
    drawer.visual(
        Box((drawer_length, drawer_width, 0.006)),
        origin=Origin(xyz=(drawer_length * 0.5, 0.0, 0.012)),
        material=dark_trim,
        name="tray_floor",
    )
    drawer.visual(
        Box((drawer_length, wall_t, drawer_height)),
        origin=Origin(
            xyz=(
                drawer_length * 0.5,
                drawer_width * 0.5 - wall_t * 0.5,
                0.012 + drawer_height * 0.5,
            )
        ),
        material=dark_trim,
        name="tray_right_wall",
    )
    drawer.visual(
        Box((drawer_length, wall_t, drawer_height)),
        origin=Origin(
            xyz=(
                drawer_length * 0.5,
                -drawer_width * 0.5 + wall_t * 0.5,
                0.012 + drawer_height * 0.5,
            )
        ),
        material=dark_trim,
        name="tray_left_wall",
    )
    drawer.visual(
        Box((wall_t, drawer_width, drawer_height)),
        origin=Origin(
            xyz=(
                drawer_length - wall_t * 0.5,
                0.0,
                0.012 + drawer_height * 0.5,
            )
        ),
        material=dark_trim,
        name="tray_rear_wall",
    )
    drawer.visual(
        Box((0.086, 0.150, 0.056)),
        origin=Origin(xyz=(0.052, 0.0, 0.043)),
        material=water,
        name="water_tank",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.152, drawer_opening_y + 0.020, fascia_h)),
        mass=1.6,
        origin=Origin(xyz=(0.060, 0.0, fascia_h * 0.5)),
    )

    cover = model.part("control_panel_cover")
    cover.visual(
        Box((0.56, 0.136, 0.006)),
        origin=Origin(xyz=(0.0, 0.068, 0.003)),
        material=stainless,
        name="cover_plate",
    )
    cover.visual(
        Box((0.56, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.131, 0.010)),
        material=stainless,
        name="cover_front_lip",
    )
    cover.visual(
        Box((0.38, 0.070, 0.002)),
        origin=Origin(xyz=(0.0, 0.075, 0.0065)),
        material=black_glass,
        name="cover_window",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.56, 0.14, 0.020)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.070, 0.010)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, body_d * 0.5, 0.060)),
        # Closed door extends upward from the hinge line. -X makes positive q
        # drop the free edge downward while swinging it outward from the front.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )

    model.articulation(
        "body_to_water_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(-body_w * 0.5, drawer_center_y, opening_z_min)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=0.0,
            upper=0.095,
        ),
    )

    model.articulation(
        "body_to_control_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, body_d * 0.5 - pod_d, body_h + pod_h)),
        # Closed cover extends toward +Y from the rear hinge line; +X lifts the
        # front edge upward for service access.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    drawer = object_model.get_part("water_reservoir_drawer")
    cover = object_model.get_part("control_panel_cover")

    door_hinge = object_model.get_articulation("body_to_door")
    drawer_slide = object_model.get_articulation("body_to_water_drawer")
    cover_hinge = object_model.get_articulation("body_to_control_cover")

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

    ctx.check(
        "door hinge axis opens downward",
        tuple(door_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "drawer slides out of left side",
        tuple(drawer_slide.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={drawer_slide.axis}",
    )
    ctx.check(
        "control cover hinge lifts from rear edge",
        tuple(cover_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={cover_hinge.axis}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.48,
            name="door covers the front cavity footprint",
        )

    drawer_rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="yz",
            min_overlap=0.10,
            name="drawer aligns with the side service bay opening",
        )

    door_closed_aabb = ctx.part_world_aabb(door)
    door_open_aabb = None
    door_upper = (
        door_hinge.motion_limits.upper
        if door_hinge.motion_limits is not None and door_hinge.motion_limits.upper is not None
        else 0.0
    )
    with ctx.pose({door_hinge: door_upper}):
        door_open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door swings outward and downward",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[1][1] > door_closed_aabb[1][1] + 0.10
        and door_open_aabb[1][2] < door_closed_aabb[1][2] - 0.45,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )

    drawer_open_pos = None
    drawer_upper = (
        drawer_slide.motion_limits.upper
        if drawer_slide.motion_limits is not None and drawer_slide.motion_limits.upper is not None
        else 0.0
    )
    with ctx.pose({drawer_slide: drawer_upper}):
        drawer_open_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends outward from the left side",
        drawer_rest_pos is not None
        and drawer_open_pos is not None
        and drawer_open_pos[0] < drawer_rest_pos[0] - 0.07,
        details=f"rest={drawer_rest_pos}, open={drawer_open_pos}",
    )

    cover_closed_aabb = ctx.part_world_aabb(cover)
    cover_open_aabb = None
    cover_upper = (
        cover_hinge.motion_limits.upper
        if cover_hinge.motion_limits is not None and cover_hinge.motion_limits.upper is not None
        else 0.0
    )
    with ctx.pose({cover_hinge: cover_upper}):
        cover_open_aabb = ctx.part_world_aabb(cover)

    ctx.check(
        "control cover lifts for service access",
        cover_closed_aabb is not None
        and cover_open_aabb is not None
        and cover_open_aabb[1][2] > cover_closed_aabb[1][2] + 0.06,
        details=f"closed={cover_closed_aabb}, open={cover_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
