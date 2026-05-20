from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    model = ArticulatedObject(name="desktop_cube_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.94, 0.95, 0.96, 1.0))
    grille_gray = model.material("grille_gray", rgba=(0.74, 0.77, 0.79, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    filter_frame = model.material("filter_frame", rgba=(0.88, 0.90, 0.91, 1.0))
    filter_media = model.material("filter_media", rgba=(0.67, 0.75, 0.69, 1.0))

    body_w = 0.24
    body_d = 0.24
    body_h = 0.276
    wall = 0.012
    half_w = body_w / 2.0
    half_d = body_d / 2.0

    front_bottom_h = 0.052
    front_top_h = 0.040
    front_side_w = 0.022
    front_open_h = body_h - (2.0 * wall) - front_bottom_h - front_top_h

    side_frame_w = 0.020
    side_bottom_h = 0.022
    side_top_h = 0.028
    door_open_w = body_d - (2.0 * side_frame_w)
    door_open_h = body_h - (2.0 * wall) - side_bottom_h - side_top_h
    door_t = 0.010
    door_w = door_open_w - 0.004
    door_h = door_open_h - 0.004
    door_bottom_z = wall + side_bottom_h + 0.002

    filter_depth = 0.155
    filter_w = 0.180
    filter_h = 0.172
    filter_face_x = half_w - wall - 0.004
    filter_center_z = door_bottom_z + (door_h / 2.0)
    slide_upper = 0.082

    housing = model.part("housing")

    housing.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=shell_white,
        name="bottom_shell",
    )
    housing.visual(
        Box((wall, body_d, body_h - (2.0 * wall))),
        origin=Origin(xyz=(-half_w + (wall / 2.0), 0.0, body_h / 2.0)),
        material=shell_white,
        name="left_shell",
    )
    housing.visual(
        Box((body_w - (2.0 * wall), wall, body_h - (2.0 * wall))),
        origin=Origin(xyz=(0.0, -half_d + (wall / 2.0), body_h / 2.0)),
        material=shell_white,
        name="back_shell",
    )

    housing.visual(
        Box((body_w - (2.0 * wall), wall, front_bottom_h)),
        origin=Origin(
            xyz=(0.0, half_d - (wall / 2.0), wall + (front_bottom_h / 2.0))
        ),
        material=shell_white,
        name="front_bottom_bar",
    )
    housing.visual(
        Box((body_w - (2.0 * wall), wall, front_top_h)),
        origin=Origin(
            xyz=(
                0.0,
                half_d - (wall / 2.0),
                body_h - wall - (front_top_h / 2.0),
            )
        ),
        material=shell_white,
        name="front_top_bar",
    )
    housing.visual(
        Box((front_side_w, wall, front_open_h)),
        origin=Origin(
            xyz=(
                -((body_w - (2.0 * wall)) / 2.0) + (front_side_w / 2.0),
                half_d - (wall / 2.0),
                wall + front_bottom_h + (front_open_h / 2.0),
            )
        ),
        material=shell_white,
        name="front_left_bar",
    )
    housing.visual(
        Box((front_side_w, wall, front_open_h)),
        origin=Origin(
            xyz=(
                ((body_w - (2.0 * wall)) / 2.0) - (front_side_w / 2.0),
                half_d - (wall / 2.0),
                wall + front_bottom_h + (front_open_h / 2.0),
            )
        ),
        material=shell_white,
        name="front_right_bar",
    )

    grille_x_positions = [-0.066, -0.040, -0.014, 0.014, 0.040, 0.066]
    for index, x_pos in enumerate(grille_x_positions, start=1):
        housing.visual(
            Box((0.010, wall * 0.55, front_open_h)),
            origin=Origin(
                xyz=(
                    x_pos,
                    half_d - (wall * 0.275),
                    wall + front_bottom_h + (front_open_h / 2.0),
                )
            ),
            material=grille_gray,
            name=f"front_grille_slat_{index}",
        )

    housing.visual(
        Box((body_w - (2.0 * wall), wall, wall)),
        origin=Origin(xyz=(0.0, half_d - (wall / 2.0), body_h - (wall / 2.0))),
        material=shell_white,
        name="top_front_ring",
    )
    housing.visual(
        Box((body_w - (2.0 * wall), wall, wall)),
        origin=Origin(xyz=(0.0, -half_d + (wall / 2.0), body_h - (wall / 2.0))),
        material=shell_white,
        name="top_back_ring",
    )
    housing.visual(
        Box((wall, body_d - (2.0 * wall), wall)),
        origin=Origin(xyz=(-half_w + (wall / 2.0), 0.0, body_h - (wall / 2.0))),
        material=shell_white,
        name="top_left_ring",
    )
    housing.visual(
        Box((wall, body_d - (2.0 * wall), wall)),
        origin=Origin(xyz=(half_w - (wall / 2.0), 0.0, body_h - (wall / 2.0))),
        material=shell_white,
        name="top_right_ring",
    )

    top_slat_x_positions = [-0.070, -0.035, 0.0, 0.035, 0.070]
    for index, x_pos in enumerate(top_slat_x_positions, start=1):
        housing.visual(
            Box((0.018, body_d - (2.0 * wall), 0.006)),
            origin=Origin(xyz=(x_pos, 0.0, body_h - (wall / 2.0))),
            material=grille_gray,
            name=f"top_exhaust_slat_{index}",
        )

    housing.visual(
        Box((wall, side_frame_w, body_h - (2.0 * wall))),
        origin=Origin(
            xyz=(half_w - (wall / 2.0), half_d - (side_frame_w / 2.0), body_h / 2.0)
        ),
        material=shell_white,
        name="side_front_frame",
    )
    housing.visual(
        Box((wall, side_frame_w, body_h - (2.0 * wall))),
        origin=Origin(
            xyz=(half_w - (wall / 2.0), -half_d + (side_frame_w / 2.0), body_h / 2.0)
        ),
        material=shell_white,
        name="side_back_frame",
    )
    housing.visual(
        Box((wall, door_open_w, side_bottom_h)),
        origin=Origin(
            xyz=(half_w - (wall / 2.0), 0.0, wall + (side_bottom_h / 2.0))
        ),
        material=shell_white,
        name="side_bottom_frame",
    )
    housing.visual(
        Box((wall, door_open_w, side_top_h)),
        origin=Origin(
            xyz=(half_w - (wall / 2.0), 0.0, body_h - wall - (side_top_h / 2.0))
        ),
        material=shell_white,
        name="side_top_frame",
    )

    rail_size = (body_w - (2.0 * wall), door_open_w, 0.008)
    housing.visual(
        Box(rail_size),
        origin=Origin(xyz=(0.0, 0.0, filter_center_z - (filter_h / 2.0) - 0.004)),
        material=charcoal,
        name="lower_guide_rail",
    )
    housing.visual(
        Box(rail_size),
        origin=Origin(xyz=(0.0, 0.0, filter_center_z + (filter_h / 2.0) + 0.004)),
        material=charcoal,
        name="upper_guide_rail",
    )

    housing.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    door = model.part("filter_door")
    door.visual(
        Box((door_t, door_w, door_h)),
        origin=Origin(xyz=(0.0, door_w / 2.0, door_h / 2.0)),
        material=shell_white,
        name="door_panel",
    )
    door.visual(
        Box((door_t, 0.002, door_h)),
        origin=Origin(xyz=(0.0, -0.001, door_h / 2.0)),
        material=shell_white,
        name="door_hinge_stile",
    )
    door.visual(
        Box((0.016, 0.042, 0.010)),
        origin=Origin(xyz=(0.013, door_w - 0.041, door_h / 2.0)),
        material=charcoal,
        name="door_pull",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_t, door_w, door_h)),
        mass=0.38,
        origin=Origin(xyz=(0.0, door_w / 2.0, door_h / 2.0)),
    )

    model.articulation(
        "housing_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(
            xyz=(half_w - (door_t / 2.0), -(door_w / 2.0), door_bottom_z)
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    filter_block = model.part("hepa_filter")
    filter_block.visual(
        Box((filter_depth, filter_w, filter_h)),
        origin=Origin(xyz=(-filter_depth / 2.0, 0.0, 0.0)),
        material=filter_frame,
        name="filter_cartridge",
    )
    filter_block.visual(
        Box((filter_depth - 0.014, filter_w - 0.018, filter_h - 0.016)),
        origin=Origin(xyz=((-(filter_depth - 0.014) / 2.0) - 0.007, 0.0, 0.0)),
        material=filter_media,
        name="filter_media_block",
    )
    pleat_y_positions = [-0.060, -0.040, -0.020, 0.0, 0.020, 0.040, 0.060]
    for index, y_pos in enumerate(pleat_y_positions, start=1):
        filter_block.visual(
            Box((0.006, 0.010, filter_h - 0.022)),
            origin=Origin(xyz=(-0.012, y_pos, 0.0)),
            material=charcoal,
            name=f"filter_pleat_{index}",
        )
    filter_block.visual(
        Box((0.003, 0.040, 0.032)),
        origin=Origin(xyz=(0.0015, 0.0, 0.0)),
        material=charcoal,
        name="filter_pull_tab",
    )
    filter_block.inertial = Inertial.from_geometry(
        Box((filter_depth, filter_w, filter_h)),
        mass=0.62,
        origin=Origin(xyz=(-filter_depth / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_hepa_filter",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_block,
        origin=Origin(xyz=(filter_face_x, 0.0, filter_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.12,
            lower=0.0,
            upper=slide_upper,
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
    door = object_model.get_part("filter_door")
    filter_block = object_model.get_part("hepa_filter")
    door_hinge = object_model.get_articulation("housing_to_filter_door")
    filter_slide = object_model.get_articulation("housing_to_hepa_filter")
    door_open_angle = min(
        math.radians(95.0),
        door_hinge.motion_limits.upper if door_hinge.motion_limits is not None else math.radians(95.0),
    )
    filter_slide_upper = (
        filter_slide.motion_limits.upper if filter_slide.motion_limits is not None else 0.082
    )

    ctx.check("housing exists", housing is not None, details="housing part missing")
    ctx.check("filter door exists", door is not None, details="filter_door part missing")
    ctx.check(
        "hepa filter exists",
        filter_block is not None,
        details="hepa_filter part missing",
    )

    with ctx.pose({door_hinge: 0.0, filter_slide: 0.0}):
        ctx.expect_contact(
            door,
            housing,
            contact_tol=0.0025,
            name="closed door seats inside the side opening",
        )
        ctx.expect_gap(
            door,
            filter_block,
            axis="x",
            min_gap=0.002,
            max_gap=0.008,
            positive_elem="door_panel",
            name="closed door leaves a narrow service gap ahead of the filter",
        )
        ctx.expect_within(
            filter_block,
            housing,
            axes="yz",
            margin=0.008,
            inner_elem="filter_cartridge",
            name="resting filter stays centered within the housing opening",
        )

        closed_door_aabb = ctx.part_world_aabb(door)
        rest_filter_pos = ctx.part_world_position(filter_block)

    with ctx.pose({door_hinge: door_open_angle}):
        opened_door_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door swings outward from the side wall",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][0] > closed_door_aabb[1][0] + 0.06,
        details=f"closed_aabb={closed_door_aabb}, opened_aabb={opened_door_aabb}",
    )

    with ctx.pose({door_hinge: door_open_angle, filter_slide: filter_slide_upper}):
        ctx.expect_within(
            filter_block,
            housing,
            axes="yz",
            margin=0.008,
            inner_elem="filter_cartridge",
            name="extended filter stays on the guide rails laterally",
        )
        ctx.expect_overlap(
            filter_block,
            housing,
            axes="x",
            elem_a="filter_cartridge",
            min_overlap=0.07,
            name="extended filter retains insertion on its guide rails",
        )
        ctx.expect_gap(
            filter_block,
            door,
            axis="y",
            min_gap=0.001,
            name="opened door clears the extended filter cartridge",
        )
        extended_filter_pos = ctx.part_world_position(filter_block)

    ctx.check(
        "filter slides outward through the service opening",
        rest_filter_pos is not None
        and extended_filter_pos is not None
        and extended_filter_pos[0] > rest_filter_pos[0] + 0.07,
        details=f"rest={rest_filter_pos}, extended={extended_filter_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
