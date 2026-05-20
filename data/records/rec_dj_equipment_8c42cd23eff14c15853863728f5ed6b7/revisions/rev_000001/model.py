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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translate_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_rack_ear_mesh(*, width: float, height: float, thickness: float):
    corner_radius = min(width * 0.18, height * 0.05, 0.003)
    slot_radius = 0.0016
    slot_width = 0.0100
    slot_height = 0.0068
    hole_offset_z = height * 0.5 - 0.0115

    outer = rounded_rect_profile(width, height, corner_radius, corner_segments=8)
    slot = rounded_rect_profile(slot_width, slot_height, slot_radius, corner_segments=8)
    holes = [
        _translate_profile(slot, 0.0, hole_offset_z),
        _translate_profile(slot, 0.0, -hole_offset_z),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, thickness, center=True),
        "rack_ear_flange",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_patchbay")

    powder_black = model.material("powder_black", rgba=(0.11, 0.12, 0.13, 1.0))
    cover_black = model.material("cover_black", rgba=(0.08, 0.09, 0.10, 1.0))
    steel_grey = model.material("steel_grey", rgba=(0.54, 0.56, 0.59, 1.0))
    tray_grey = model.material("tray_grey", rgba=(0.23, 0.24, 0.26, 1.0))
    label_grey = model.material("label_grey", rgba=(0.35, 0.37, 0.40, 1.0))
    jack_black = model.material("jack_black", rgba=(0.03, 0.03, 0.04, 1.0))
    mat_pad = model.material("mat_pad", rgba=(0.18, 0.19, 0.20, 1.0))

    total_width = 0.4826
    body_width = 0.4380
    ear_width = (total_width - body_width) * 0.5
    height = 0.04445
    depth = 0.125

    shell_t = 0.0018
    side_t = 0.0020
    frame_t = 0.0025
    back_t = 0.0016

    front_bar_w = 0.0120
    opening_width = body_width - 2.0 * front_bar_w
    bottom_sill_h = 0.0025
    drawer_slot_h = 0.0085
    cover_bottom_z = bottom_sill_h + drawer_slot_h
    top_bar_h = 0.0045
    cover_height = height - cover_bottom_z - top_bar_h

    drawer_front_y = 0.0010
    drawer_width = 0.404
    drawer_depth = 0.094
    drawer_floor_z0 = 0.0042
    drawer_floor_t = 0.0014
    drawer_wall_t = 0.0016
    drawer_wall_h = 0.0060
    drawer_lip_t = 0.0020
    drawer_lip_h = 0.0078

    chassis = model.part("chassis")
    ear_mesh = _build_rack_ear_mesh(width=ear_width, height=height, thickness=frame_t)

    chassis.visual(
        ear_mesh,
        origin=Origin(
            xyz=(-body_width * 0.5 - ear_width * 0.5, frame_t * 0.5, height * 0.5),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=powder_black,
        name="left_ear",
    )
    chassis.visual(
        ear_mesh,
        origin=Origin(
            xyz=(body_width * 0.5 + ear_width * 0.5, frame_t * 0.5, height * 0.5),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=powder_black,
        name="right_ear",
    )
    chassis.visual(
        Box((body_width, depth, shell_t)),
        origin=Origin(xyz=(0.0, depth * 0.5, shell_t * 0.5)),
        material=powder_black,
        name="bottom_pan",
    )
    chassis.visual(
        Box((body_width, depth, shell_t)),
        origin=Origin(xyz=(0.0, depth * 0.5, height - shell_t * 0.5)),
        material=powder_black,
        name="top_pan",
    )
    chassis.visual(
        Box((side_t, depth, height - 2.0 * shell_t)),
        origin=Origin(xyz=(-body_width * 0.5 + side_t * 0.5, depth * 0.5, height * 0.5)),
        material=powder_black,
        name="left_wall",
    )
    chassis.visual(
        Box((side_t, depth, height - 2.0 * shell_t)),
        origin=Origin(xyz=(body_width * 0.5 - side_t * 0.5, depth * 0.5, height * 0.5)),
        material=powder_black,
        name="right_wall",
    )
    chassis.visual(
        Box((body_width - 2.0 * side_t, back_t, height - 2.0 * shell_t)),
        origin=Origin(xyz=(0.0, depth - back_t * 0.5, height * 0.5)),
        material=powder_black,
        name="rear_panel",
    )
    chassis.visual(
        Box((front_bar_w, frame_t, height)),
        origin=Origin(
            xyz=(-opening_width * 0.5 - front_bar_w * 0.5, frame_t * 0.5, height * 0.5)
        ),
        material=powder_black,
        name="left_front_post",
    )
    chassis.visual(
        Box((front_bar_w, frame_t, height)),
        origin=Origin(
            xyz=(opening_width * 0.5 + front_bar_w * 0.5, frame_t * 0.5, height * 0.5)
        ),
        material=powder_black,
        name="right_front_post",
    )
    chassis.visual(
        Box((opening_width, frame_t, top_bar_h)),
        origin=Origin(xyz=(0.0, frame_t * 0.5, height - top_bar_h * 0.5)),
        material=powder_black,
        name="top_header",
    )
    chassis.visual(
        Box((opening_width, frame_t, bottom_sill_h)),
        origin=Origin(xyz=(0.0, frame_t * 0.5, bottom_sill_h * 0.5)),
        material=powder_black,
        name="bottom_sill",
    )

    patch_plate_h = cover_height - 0.0060
    patch_plate_y = 0.010
    patch_plate = chassis.visual(
        Box((opening_width, 0.0016, patch_plate_h)),
        origin=Origin(
            xyz=(
                0.0,
                patch_plate_y,
                cover_bottom_z + 0.0030 + patch_plate_h * 0.5,
            )
        ),
        material=tray_grey,
        name="patch_plate",
    )
    _ = patch_plate
    bracket_x = opening_width * 0.5 - 0.003
    bracket_y_len = patch_plate_y - frame_t + 0.0020
    bracket_y = frame_t + bracket_y_len * 0.5 - 0.0002
    for sign, bracket_name in ((-1.0, "left_patch_bracket"), (1.0, "right_patch_bracket")):
        chassis.visual(
            Box((0.006, bracket_y_len, patch_plate_h)),
            origin=Origin(
                xyz=(
                    sign * bracket_x,
                    bracket_y,
                    cover_bottom_z + 0.0030 + patch_plate_h * 0.5,
                )
            ),
            material=tray_grey,
            name=bracket_name,
        )

    jack_rows = (cover_bottom_z + 0.0085, cover_bottom_z + 0.0195)
    jack_xs = (-0.165, -0.099, -0.033, 0.033, 0.099, 0.165)
    jack_y = patch_plate_y - 0.0023
    for row_index, jack_z in enumerate(jack_rows):
        for col_index, jack_x in enumerate(jack_xs):
            chassis.visual(
                Cylinder(radius=0.0040, length=0.0030),
                origin=Origin(
                    xyz=(jack_x, jack_y, jack_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=jack_black,
                name=f"jack_{row_index}_{col_index}",
            )

    rail_width = 0.020
    rail_height = drawer_floor_z0 - shell_t
    rail_length = drawer_depth - 0.002
    rail_center_y = drawer_front_y + 0.002 + rail_length * 0.5
    rail_x = drawer_width * 0.5 - rail_width * 0.5 - 0.034
    for sign, name in ((-1.0, "left_drawer_rail"), (1.0, "right_drawer_rail")):
        chassis.visual(
            Box((rail_width, rail_length, rail_height)),
            origin=Origin(
                xyz=(sign * rail_x, rail_center_y, shell_t + rail_height * 0.5)
            ),
            material=tray_grey,
            name=name,
        )

    chassis.inertial = Inertial.from_geometry(
        Box((total_width, depth, height)),
        mass=4.8,
        origin=Origin(xyz=(0.0, depth * 0.5, height * 0.5)),
    )

    front_cover = model.part("front_cover")
    cover_width = opening_width + 0.008
    cover_t = 0.0024
    front_cover.visual(
        Box((cover_width, cover_t, cover_height)),
        origin=Origin(xyz=(0.0, -cover_t * 0.5, cover_height * 0.5)),
        material=cover_black,
        name="cover_panel",
    )
    front_cover.visual(
        Box((cover_width * 0.54, 0.0009, 0.0042)),
        origin=Origin(
            xyz=(0.0, -cover_t - 0.00045, cover_height - 0.0058)
        ),
        material=label_grey,
        name="cover_label_strip",
    )
    for sign, latch_name in ((-1.0, "left_latch"), (1.0, "right_latch")):
        front_cover.visual(
            Cylinder(radius=0.0032, length=0.0040),
            origin=Origin(
                xyz=(sign * cover_width * 0.32, -cover_t - 0.0020, cover_height - 0.0068),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel_grey,
            name=latch_name,
        )
    front_cover.inertial = Inertial.from_geometry(
        Box((cover_width, 0.0065, cover_height)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.0032, cover_height * 0.5)),
    )

    media_drawer = model.part("media_drawer")
    drawer_floor_len = drawer_depth - drawer_lip_t
    drawer_center_y = drawer_lip_t + drawer_floor_len * 0.5
    media_drawer.visual(
        Box((drawer_width, drawer_floor_len, drawer_floor_t)),
        origin=Origin(xyz=(0.0, drawer_center_y, drawer_floor_t * 0.5)),
        material=tray_grey,
        name="drawer_floor",
    )
    media_drawer.visual(
        Box((drawer_wall_t, drawer_floor_len, drawer_wall_h)),
        origin=Origin(
            xyz=(
                -drawer_width * 0.5 + drawer_wall_t * 0.5,
                drawer_center_y,
                drawer_wall_h * 0.5,
            )
        ),
        material=tray_grey,
        name="left_drawer_wall",
    )
    media_drawer.visual(
        Box((drawer_wall_t, drawer_floor_len, drawer_wall_h)),
        origin=Origin(
            xyz=(
                drawer_width * 0.5 - drawer_wall_t * 0.5,
                drawer_center_y,
                drawer_wall_h * 0.5,
            )
        ),
        material=tray_grey,
        name="right_drawer_wall",
    )
    media_drawer.visual(
        Box((drawer_width - 2.0 * drawer_wall_t, drawer_wall_t, drawer_wall_h)),
        origin=Origin(
            xyz=(0.0, drawer_depth - drawer_wall_t * 0.5, drawer_wall_h * 0.5)
        ),
        material=tray_grey,
        name="rear_drawer_wall",
    )
    lip_z = drawer_lip_h * 0.5 - (drawer_floor_z0 - bottom_sill_h)
    media_drawer.visual(
        Box((drawer_width, drawer_lip_t, drawer_lip_h)),
        origin=Origin(xyz=(0.0, drawer_lip_t * 0.5, lip_z)),
        material=cover_black,
        name="drawer_front",
    )
    media_drawer.visual(
        Cylinder(radius=0.0018, length=0.080),
        origin=Origin(
            xyz=(0.0, drawer_lip_t + 0.0018, lip_z + 0.0027),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_grey,
        name="drawer_pull",
    )
    media_drawer.visual(
        Box((0.150, 0.070, 0.0008)),
        origin=Origin(
            xyz=(
                0.0,
                drawer_lip_t + 0.040,
                drawer_floor_t + 0.0004,
            )
        ),
        material=mat_pad,
        name="media_pad",
    )
    media_drawer.inertial = Inertial.from_geometry(
        Box((drawer_width, drawer_depth, drawer_lip_h)),
        mass=0.5,
        origin=Origin(xyz=(0.0, drawer_depth * 0.5, drawer_lip_h * 0.5 - 0.0015)),
    )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_cover,
        origin=Origin(xyz=(0.0, 0.0, cover_bottom_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=media_drawer,
        origin=Origin(xyz=(0.0, drawer_front_y, drawer_floor_z0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.35,
            lower=0.0,
            upper=0.085,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    front_cover = object_model.get_part("front_cover")
    media_drawer = object_model.get_part("media_drawer")
    cover_hinge = object_model.get_articulation("cover_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "cover_hinge_axis_is_widthwise",
        tuple(cover_hinge.axis) == (1.0, 0.0, 0.0),
        f"expected hinge axis (1, 0, 0), got {cover_hinge.axis}",
    )
    ctx.check(
        "drawer_slide_axis_is_forward",
        tuple(drawer_slide.axis) == (0.0, -1.0, 0.0),
        f"expected drawer axis (0, -1, 0), got {drawer_slide.axis}",
    )
    ctx.expect_contact(front_cover, chassis, name="cover_contacts_chassis_when_closed")
    ctx.expect_gap(
        chassis,
        front_cover,
        axis="y",
        min_gap=-1e-6,
        max_gap=1e-6,
        name="cover_sits_flush_on_front_face",
    )
    ctx.expect_contact(media_drawer, chassis, name="drawer_supported_by_internal_rails")
    ctx.expect_within(
        media_drawer,
        chassis,
        axes="xz",
        margin=0.0,
        name="drawer_stays_within_housing_profile",
    )
    ctx.expect_gap(
        front_cover,
        media_drawer,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="drawer_slot_clears_cover_bottom_edge",
    )

    cover_closed = ctx.part_world_aabb(front_cover)
    drawer_closed = ctx.part_world_aabb(media_drawer)
    assert cover_closed is not None
    assert drawer_closed is not None

    with ctx.pose({cover_hinge: math.radians(105.0)}):
        cover_open = ctx.part_world_aabb(front_cover)
        assert cover_open is not None
        ctx.check(
            "cover_folds_outward",
            cover_open[0][1] < cover_closed[0][1] - 0.018,
            f"expected cover to swing outward; closed={cover_closed}, open={cover_open}",
        )
        ctx.check(
            "cover_folds_down",
            cover_open[1][2] < cover_closed[1][2] - 0.012,
            f"expected cover top to drop when opened; closed={cover_closed}, open={cover_open}",
        )
        ctx.expect_gap(
            chassis,
            front_cover,
            axis="y",
            max_penetration=0.001,
            name="open_cover_hinge_region_stays_near_front_plane",
        )

    with ctx.pose({drawer_slide: 0.080}):
        drawer_open = ctx.part_world_aabb(media_drawer)
        assert drawer_open is not None
        ctx.check(
            "drawer_extends_forward",
            drawer_open[0][1] < drawer_closed[0][1] - 0.070,
            f"expected drawer to slide forward; closed={drawer_closed}, open={drawer_open}",
        )
        ctx.expect_contact(
            media_drawer,
            chassis,
            name="drawer_remains_supported_when_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
