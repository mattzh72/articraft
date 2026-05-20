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
    model = ArticulatedObject(name="medical_room_air_purifier")

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.96, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.70, 0.73, 0.77, 1.0))
    base_gray = model.material("base_gray", rgba=(0.56, 0.59, 0.63, 1.0))
    grille_gray = model.material("grille_gray", rgba=(0.31, 0.35, 0.39, 1.0))
    filter_blue = model.material("filter_blue", rgba=(0.45, 0.63, 0.78, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.20, 0.22, 0.24, 1.0))

    housing_w = 0.62
    housing_d = 0.26
    housing_h = 0.48
    wall = 0.015
    housing_bottom = 0.58
    housing_top = housing_bottom + housing_h

    lid_w = 0.56
    lid_d = 0.224
    lid_t = 0.018

    opening_y = 0.186
    opening_z = 0.30
    opening_margin_y = (housing_d - opening_y) * 0.5
    opening_margin_z = (housing_h - opening_z) * 0.5
    filter_center_z = housing_bottom + opening_margin_z + opening_z * 0.5

    body = model.part("body")
    body.visual(
        Box((0.54, 0.34, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=base_gray,
        name="stand_base",
    )
    body.visual(
        Box((0.12, 0.18, housing_bottom - 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.036 + (housing_bottom - 0.036) * 0.5)),
        material=trim_gray,
        name="stand_column",
    )
    body.visual(
        Box((housing_w - 2.0 * wall, housing_d - 2.0 * wall, wall)),
        origin=Origin(xyz=(0.0, 0.0, housing_bottom + wall * 0.5)),
        material=body_white,
        name="housing_floor",
    )
    body.visual(
        Box((housing_w, wall, housing_h)),
        origin=Origin(
            xyz=(0.0, housing_d * 0.5 - wall * 0.5, housing_bottom + housing_h * 0.5)
        ),
        material=body_white,
        name="front_shell",
    )
    body.visual(
        Box((housing_w, wall, housing_h)),
        origin=Origin(
            xyz=(0.0, -housing_d * 0.5 + wall * 0.5, housing_bottom + housing_h * 0.5)
        ),
        material=body_white,
        name="rear_shell",
    )
    body.visual(
        Box((wall, housing_d - 2.0 * wall, housing_h)),
        origin=Origin(xyz=(-housing_w * 0.5 + wall * 0.5, 0.0, housing_bottom + housing_h * 0.5)),
        material=body_white,
        name="left_shell",
    )
    body.visual(
        Box((wall, opening_margin_y, opening_z)),
        origin=Origin(
            xyz=(
                housing_w * 0.5 - wall * 0.5,
                opening_y * 0.5 + opening_margin_y * 0.5,
                filter_center_z,
            )
        ),
        material=body_white,
        name="right_front_frame",
    )
    body.visual(
        Box((wall, opening_margin_y, opening_z)),
        origin=Origin(
            xyz=(
                housing_w * 0.5 - wall * 0.5,
                -opening_y * 0.5 - opening_margin_y * 0.5,
                filter_center_z,
            )
        ),
        material=body_white,
        name="right_rear_frame",
    )
    body.visual(
        Box((wall, opening_y, opening_margin_z)),
        origin=Origin(
            xyz=(
                housing_w * 0.5 - wall * 0.5,
                0.0,
                housing_bottom + opening_margin_z * 0.5,
            )
        ),
        material=body_white,
        name="right_bottom_frame",
    )
    body.visual(
        Box((wall, opening_y, opening_margin_z)),
        origin=Origin(
            xyz=(
                housing_w * 0.5 - wall * 0.5,
                0.0,
                housing_bottom + opening_margin_z + opening_z + opening_margin_z * 0.5,
            )
        ),
        material=body_white,
        name="right_top_frame",
    )

    top_open_w = 0.54
    top_open_d = 0.20
    side_rim = (housing_w - top_open_w) * 0.5
    front_back_rim = (housing_d - top_open_d) * 0.5
    body.visual(
        Box((housing_w, front_back_rim, wall)),
        origin=Origin(xyz=(0.0, housing_d * 0.5 - front_back_rim * 0.5, housing_top - wall * 0.5)),
        material=body_white,
        name="top_front_rim",
    )
    body.visual(
        Box((housing_w, front_back_rim, wall)),
        origin=Origin(xyz=(0.0, -housing_d * 0.5 + front_back_rim * 0.5, housing_top - wall * 0.5)),
        material=body_white,
        name="top_rear_rim",
    )
    body.visual(
        Box((side_rim, top_open_d, wall)),
        origin=Origin(xyz=(-housing_w * 0.5 + side_rim * 0.5, 0.0, housing_top - wall * 0.5)),
        material=body_white,
        name="top_left_rim",
    )
    body.visual(
        Box((side_rim, top_open_d, wall)),
        origin=Origin(xyz=(housing_w * 0.5 - side_rim * 0.5, 0.0, housing_top - wall * 0.5)),
        material=body_white,
        name="top_right_rim",
    )

    body.visual(
        Box((0.22, 0.016, 0.26)),
        origin=Origin(xyz=(0.185, 0.096, filter_center_z)),
        material=trim_gray,
        name="front_guide_rail",
    )
    body.visual(
        Box((0.22, 0.016, 0.26)),
        origin=Origin(xyz=(0.185, -0.096, filter_center_z)),
        material=trim_gray,
        name="rear_guide_rail",
    )
    body.visual(
        Box((0.50, 0.004, 0.30)),
        origin=Origin(xyz=(0.0, housing_d * 0.5 - wall - 0.002, housing_bottom + 0.20)),
        material=grille_gray,
        name="intake_screen",
    )
    for slat_index in range(6):
        body.visual(
            Box((0.49, 0.006, 0.012)),
            origin=Origin(
                xyz=(
                    0.0,
                    housing_d * 0.5 - 0.002,
                    housing_bottom + 0.12 + 0.045 * slat_index,
                )
            ),
            material=grille_gray,
            name=f"intake_slat_{slat_index}",
        )
    body.visual(
        Box((0.11, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, housing_d * 0.5 - 0.002, housing_top - 0.07)),
        material=trim_gray,
        name="status_bezel",
    )
    body.inertial = Inertial.from_geometry(
        Box((housing_w, 0.34, housing_top)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, housing_top * 0.5)),
    )

    lid = model.part("hepa_lid")
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(xyz=(0.0, lid_d * 0.5, lid_t * 0.5)),
        material=body_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.14, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, lid_d - 0.009, lid_t * 0.5 + 0.002)),
        material=handle_dark,
        name="lid_pull",
    )
    lid.visual(
        Box((0.50, 0.17, 0.010)),
        origin=Origin(xyz=(0.0, 0.112, -0.005)),
        material=trim_gray,
        name="lid_inner_flange",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_t)),
        mass=2.1,
        origin=Origin(xyz=(0.0, lid_d * 0.5, lid_t * 0.5)),
    )

    pre_filter = model.part("pre_filter")
    pre_filter.visual(
        Box((0.018, 0.18, 0.30)),
        material=trim_gray,
        name="filter_frame",
    )
    pre_filter.visual(
        Box((0.004, 0.184, 0.304)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material=body_white,
        name="filter_door",
    )
    pre_filter.visual(
        Box((0.010, 0.158, 0.272)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=filter_blue,
        name="filter_media",
    )
    pre_filter.visual(
        Box((0.20, 0.010, 0.24)),
        origin=Origin(xyz=(-0.10, 0.082, 0.0)),
        material=trim_gray,
        name="front_runner",
    )
    pre_filter.visual(
        Box((0.20, 0.010, 0.24)),
        origin=Origin(xyz=(-0.10, -0.082, 0.0)),
        material=trim_gray,
        name="rear_runner",
    )
    pre_filter.visual(
        Box((0.012, 0.060, 0.016)),
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material=handle_dark,
        name="filter_handle",
    )
    pre_filter.inertial = Inertial.from_geometry(
        Box((0.20, 0.18, 0.30)),
        mass=1.0,
        origin=Origin(xyz=(-0.05, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_hepa_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -lid_d * 0.5, housing_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.22,
        ),
    )
    model.articulation(
        "body_to_pre_filter",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pre_filter,
        origin=Origin(xyz=(housing_w * 0.5 - 0.009, 0.0, filter_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.16,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("hepa_lid")
    pre_filter = object_model.get_part("pre_filter")
    lid_hinge = object_model.get_articulation("body_to_hepa_lid")
    filter_slide = object_model.get_articulation("body_to_pre_filter")

    lid_shell = lid.get_visual("lid_shell")
    filter_frame = pre_filter.get_visual("filter_frame")
    filter_door = pre_filter.get_visual("filter_door")
    front_runner = pre_filter.get_visual("front_runner")
    rear_runner = pre_filter.get_visual("rear_runner")
    front_rail = body.get_visual("front_guide_rail")
    rear_rail = body.get_visual("rear_guide_rail")

    with ctx.pose({lid_hinge: 0.0, filter_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=lid_shell,
            max_gap=0.002,
            max_penetration=0.0,
            name="HEPA lid sits closed on the housing rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=lid_shell,
            min_overlap=0.20,
            name="HEPA lid covers the purifier top opening",
        )
        ctx.expect_overlap(
            pre_filter,
            body,
            axes="yz",
            elem_a=filter_frame,
            min_overlap=0.18,
            name="Pre-filter occupies the side service bay",
        )
        ctx.expect_overlap(
            pre_filter,
            body,
            axes="x",
            elem_a=front_runner,
            elem_b=front_rail,
            min_overlap=0.18,
            name="Front runner is deeply engaged with the front guide rail at rest",
        )
        ctx.expect_overlap(
            pre_filter,
            body,
            axes="x",
            elem_a=rear_runner,
            elem_b=rear_rail,
            min_overlap=0.18,
            name="Rear runner is deeply engaged with the rear guide rail at rest",
        )
        closed_pull_aabb = ctx.part_element_world_aabb(lid, elem="lid_pull")
        closed_filter_door_aabb = ctx.part_element_world_aabb(pre_filter, elem="filter_door")
        body_aabb = ctx.part_world_aabb(body)
        rest_filter_pos = ctx.part_world_position(pre_filter)

    ctx.check(
        "Pre-filter door sits flush with the side opening",
        closed_filter_door_aabb is not None
        and body_aabb is not None
        and abs(closed_filter_door_aabb[1][0] - body_aabb[1][0]) <= 0.002
        and closed_filter_door_aabb[0][0] < body_aabb[1][0],
        details=f"filter_door={closed_filter_door_aabb}, body={body_aabb}",
    )

    with ctx.pose({lid_hinge: 1.0}):
        open_pull_aabb = ctx.part_element_world_aabb(lid, elem="lid_pull")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_pull",
            min_gap=0.12,
            name="Opened lid lifts the front pull well above the housing",
        )

    ctx.check(
        "Top lid opens upward from the rear hinge",
        closed_pull_aabb is not None
        and open_pull_aabb is not None
        and open_pull_aabb[0][2] > closed_pull_aabb[0][2] + 0.16,
        details=f"closed_pull={closed_pull_aabb}, open_pull={open_pull_aabb}",
    )

    with ctx.pose({filter_slide: 0.16}):
        extended_filter_pos = ctx.part_world_position(pre_filter)
        ctx.expect_gap(
            pre_filter,
            body,
            axis="x",
            positive_elem=filter_frame,
            min_gap=0.13,
            name="Pre-filter can slide clearly out of the side of the housing",
        )
        ctx.expect_overlap(
            pre_filter,
            body,
            axes="x",
            elem_a=front_runner,
            elem_b=front_rail,
            min_overlap=0.03,
            name="Front runner retains insertion at full extension",
        )
        ctx.expect_overlap(
            pre_filter,
            body,
            axes="x",
            elem_a=rear_runner,
            elem_b=rear_rail,
            min_overlap=0.03,
            name="Rear runner retains insertion at full extension",
        )

    ctx.check(
        "Pre-filter slides outward along the right side",
        rest_filter_pos is not None
        and extended_filter_pos is not None
        and extended_filter_pos[0] > rest_filter_pos[0] + 0.14,
        details=f"rest={rest_filter_pos}, extended={extended_filter_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
