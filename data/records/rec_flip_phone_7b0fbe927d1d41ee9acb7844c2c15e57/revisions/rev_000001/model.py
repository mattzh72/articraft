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
    model = ArticulatedObject(name="wide_flip_communicator")

    body_plastic = model.material("body_plastic", rgba=(0.20, 0.21, 0.24, 1.0))
    body_dark = model.material("body_dark", rgba=(0.11, 0.12, 0.14, 1.0))
    key_dark = model.material("key_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    glass = model.material("glass", rgba=(0.15, 0.29, 0.35, 0.55))
    trim = model.material("trim", rgba=(0.55, 0.57, 0.60, 1.0))

    body_w = 0.168
    body_d = 0.118
    body_h = 0.024
    wall_t = 0.003
    lid_t = 0.010
    lid_inner_z = 0.0012
    hinge_radius = 0.0032
    hinge_axis_y = body_d * 0.5 - 0.006
    hinge_axis_z = body_h + hinge_radius - 0.001

    lower_body = model.part("lower_body")
    lower_body.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, wall_t * 0.5)),
        material=body_dark,
        name="bottom_shell",
    )
    lower_body.visual(
        Box((body_w, wall_t, body_h - wall_t)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - wall_t * 0.5, body_h * 0.5)),
        material=body_plastic,
        name="rear_wall",
    )
    lower_body.visual(
        Box((body_w, wall_t, body_h - wall_t)),
        origin=Origin(xyz=(0.0, -body_d * 0.5 + wall_t * 0.5, body_h * 0.5)),
        material=body_plastic,
        name="front_wall",
    )
    lower_body.visual(
        Box((wall_t, body_d - 2.0 * wall_t, body_h - wall_t)),
        origin=Origin(xyz=(-body_w * 0.5 + wall_t * 0.5, 0.0, body_h * 0.5)),
        material=body_plastic,
        name="left_wall",
    )
    lower_body.visual(
        Box((wall_t, body_d - 2.0 * wall_t, body_h - wall_t)),
        origin=Origin(xyz=(body_w * 0.5 - wall_t * 0.5, 0.0, body_h * 0.5)),
        material=body_plastic,
        name="right_wall",
    )
    lower_body.visual(
        Box((body_w - 2.0 * wall_t, body_d - 2.0 * wall_t, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - wall_t * 0.5)),
        material=body_plastic,
        name="top_deck",
    )
    lower_body.visual(
        Box((body_w - 0.018, body_d - 0.030, 0.0015)),
        origin=Origin(xyz=(0.0, -0.003, body_h - 0.0010)),
        material=key_dark,
        name="keyboard_sheet",
    )
    lower_body.visual(
        Box((0.082, 0.009, 0.004)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - 0.0065, body_h + 0.0005)),
        material=body_plastic,
        name="hinge_bridge",
    )
    lower_body.visual(
        Cylinder(radius=hinge_radius, length=0.030),
        origin=Origin(
            xyz=(-0.051, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim,
        name="left_hinge_post",
    )
    lower_body.visual(
        Cylinder(radius=hinge_radius, length=0.030),
        origin=Origin(
            xyz=(0.051, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim,
        name="right_hinge_post",
    )
    lower_body.visual(
        Box((0.092, wall_t, 0.018)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - wall_t * 0.5, 0.011)),
        material=body_dark,
        name="battery_track_band",
    )
    lower_body.visual(
        Box((0.006, 0.0020, 0.016)),
        origin=Origin(xyz=(-0.043, body_d * 0.5 - 0.0010, 0.011)),
        material=body_plastic,
        name="battery_left_rail",
    )
    lower_body.visual(
        Box((0.006, 0.0020, 0.016)),
        origin=Origin(xyz=(0.043, body_d * 0.5 - 0.0010, 0.011)),
        material=body_plastic,
        name="battery_right_rail",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h + 0.008)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    key_gap = 0.0012
    key_base_height = 0.0016
    key_cap_height = 0.0012
    key_depth = 0.0105
    key_base_center_z = body_h + 0.00005
    key_cap_center_z = body_h + 0.0011

    def add_key_row(
        row_name: str,
        y_center: float,
        unit_pitch: float,
        widths: tuple[float, ...],
        x_offset: float = 0.0,
        depth_scale: float = 1.0,
    ) -> None:
        total_width = sum(widths) * unit_pitch + key_gap * (len(widths) - 1)
        cursor_x = -total_width * 0.5 + x_offset
        for key_index, width_units in enumerate(widths):
            key_width = width_units * unit_pitch
            lower_body.visual(
                Box((key_width, key_depth * depth_scale, key_base_height)),
                origin=Origin(
                    xyz=(cursor_x + key_width * 0.5, y_center, key_base_center_z)
                ),
                material=key_dark,
                name=f"{row_name}_{key_index}_base",
            )
            lower_body.visual(
                Box((max(key_width - 0.0016, 0.0048), key_depth * depth_scale * 0.82, key_cap_height)),
                origin=Origin(
                    xyz=(cursor_x + key_width * 0.5, y_center, key_cap_center_z)
                ),
                material=trim,
                name=f"{row_name}_{key_index}_cap",
            )
            cursor_x += key_width + key_gap

    add_key_row(
        "key_row_qwerty",
        0.017,
        0.0098,
        (1.15, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.45),
        x_offset=-0.004,
        depth_scale=0.96,
    )
    add_key_row(
        "key_row_asdf",
        0.003,
        0.0106,
        (1.30, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.55),
        x_offset=0.001,
    )
    add_key_row(
        "key_row_zxcv",
        -0.011,
        0.0115,
        (1.70, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.70),
        x_offset=0.006,
    )
    add_key_row(
        "key_row_space",
        -0.026,
        0.0122,
        (1.25, 4.80, 1.35, 1.55, 1.55),
        x_offset=0.003,
        depth_scale=0.92,
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.0028, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    lid.visual(
        Box((body_w, wall_t, lid_t - 0.002)),
        origin=Origin(xyz=(0.0, -body_d + wall_t * 0.5, lid_t * 0.5 - 0.001)),
        material=body_plastic,
        name="front_lip",
    )
    lid.visual(
        Box((wall_t, body_d - 0.006, lid_t - 0.002)),
        origin=Origin(
            xyz=(-body_w * 0.5 + wall_t * 0.5, -body_d * 0.5, lid_t * 0.5 - 0.001)
        ),
        material=body_plastic,
        name="left_lip",
    )
    lid.visual(
        Box((wall_t, body_d - 0.006, lid_t - 0.002)),
        origin=Origin(
            xyz=(body_w * 0.5 - wall_t * 0.5, -body_d * 0.5, lid_t * 0.5 - 0.001)
        ),
        material=body_plastic,
        name="right_lip",
    )
    lid.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, -body_d * 0.5, lid_t - wall_t * 0.5)),
        material=body_plastic,
        name="outer_skin",
    )
    lid.visual(
        Box((body_w, wall_t, lid_t - 0.002)),
        origin=Origin(xyz=(0.0, -wall_t * 0.5, lid_t * 0.5 - 0.001)),
        material=body_plastic,
        name="rear_lip",
    )
    lid.visual(
        Box((0.158, 0.007, 0.003)),
        origin=Origin(xyz=(0.0, -0.0115, 0.0028)),
        material=body_plastic,
        name="display_bezel_top",
    )
    lid.visual(
        Box((0.158, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, -0.106, 0.0028)),
        material=body_plastic,
        name="display_bezel_bottom",
    )
    lid.visual(
        Box((0.010, 0.087, 0.003)),
        origin=Origin(xyz=(-0.078, -0.059, 0.0028)),
        material=body_plastic,
        name="display_bezel_left",
    )
    lid.visual(
        Box((0.010, 0.087, 0.003)),
        origin=Origin(xyz=(0.078, -0.059, 0.0028)),
        material=body_plastic,
        name="display_bezel_right",
    )
    lid.visual(
        Box((body_w - 2.0 * wall_t, body_d - 2.0 * wall_t, 0.002)),
        origin=Origin(xyz=(0.0, -body_d * 0.5, lid_inner_z + 0.001)),
        material=body_dark,
        name="display_backing",
    )
    lid.visual(
        Box((0.146, 0.094, 0.0016)),
        origin=Origin(xyz=(0.0, -body_d * 0.5, lid_inner_z + 0.002)),
        material=glass,
        name="inner_display",
    )
    lid.visual(
        Box((0.052, 0.022, 0.0012)),
        origin=Origin(xyz=(0.0, -0.040, lid_t + 0.0006)),
        material=glass,
        name="outer_display",
    )
    lid.inertial = Inertial.from_geometry(
        Box((body_w, body_d, lid_t)),
        mass=0.20,
        origin=Origin(xyz=(0.0, -body_d * 0.5, lid_t * 0.5)),
    )

    model.articulation(
        "lower_body_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.5,
            velocity=2.5,
            lower=0.0,
            upper=2.2,
        ),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.080, 0.0028, 0.0155)),
        origin=Origin(xyz=(0.0, 0.0014, -0.00775)),
        material=body_plastic,
        name="door_panel",
    )
    battery_door.visual(
        Box((0.010, 0.0012, 0.0045)),
        origin=Origin(xyz=(-0.033, -0.0006, -0.003)),
        material=body_dark,
        name="left_slider_tab",
    )
    battery_door.visual(
        Box((0.010, 0.0012, 0.0045)),
        origin=Origin(xyz=(0.033, -0.0006, -0.003)),
        material=body_dark,
        name="right_slider_tab",
    )
    battery_door.visual(
        Box((0.024, 0.0010, 0.0035)),
        origin=Origin(xyz=(0.0, 0.0001, -0.0125)),
        material=trim,
        name="finger_latch",
    )
    battery_door.inertial = Inertial.from_geometry(
        Box((0.080, 0.0028, 0.0155)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0014, -0.00775)),
    )
    model.articulation(
        "lower_body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=lower_body,
        child=battery_door,
        origin=Origin(xyz=(0.0, body_d * 0.5 + 0.0004, 0.0182)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.08,
            lower=0.0,
            upper=0.007,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    lid = object_model.get_part("lid")
    battery_door = object_model.get_part("battery_door")
    lid_hinge = object_model.get_articulation("lower_body_to_lid")
    door_slide = object_model.get_articulation("lower_body_to_battery_door")
    lid.get_visual("inner_display")
    lid.get_visual("outer_display")

    ctx.expect_overlap(
        lid,
        lower_body,
        axes="xy",
        min_overlap=0.10,
        name="closed lid covers the communicator body footprint",
    )
    ctx.expect_gap(
        battery_door,
        lower_body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="battery_track_band",
        min_gap=0.0001,
        max_gap=0.0020,
        name="battery door sits just proud of the rear wall",
    )
    ctx.expect_within(
        battery_door,
        lower_body,
        axes="x",
        inner_elem="door_panel",
        outer_elem="battery_track_band",
        margin=0.006,
        name="battery door stays centered on the lower body rear",
    )

    lid_closed_aabb = ctx.part_world_aabb(lid)
    door_closed_pos = ctx.part_world_position(battery_door)
    with ctx.pose({lid_hinge: 1.4, door_slide: 0.007}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        door_open_pos = ctx.part_world_position(battery_door)
        ctx.expect_overlap(
            battery_door,
            lower_body,
            axes="z",
            elem_a="door_panel",
            elem_b="battery_track_band",
            min_overlap=0.006,
            name="battery door remains captured by the rear track when slid open",
        )

    closed_lid_top = lid_closed_aabb[1][2] if lid_closed_aabb is not None else None
    open_lid_top = lid_open_aabb[1][2] if lid_open_aabb is not None else None
    ctx.check(
        "lid opens upward on its hinge",
        closed_lid_top is not None
        and open_lid_top is not None
        and open_lid_top > closed_lid_top + 0.07,
        details=f"closed_top={closed_lid_top}, open_top={open_lid_top}",
    )
    ctx.check(
        "battery door slides downward",
        door_closed_pos is not None
        and door_open_pos is not None
        and door_open_pos[2] < door_closed_pos[2] - 0.005,
        details=f"closed_pos={door_closed_pos}, open_pos={door_open_pos}",
    )

    inner_display_aabb = ctx.part_element_world_aabb(lid, elem="inner_display")
    outer_display_aabb = ctx.part_element_world_aabb(lid, elem="outer_display")
    inner_area = None
    outer_area = None
    if inner_display_aabb is not None:
        inner_area = (inner_display_aabb[1][0] - inner_display_aabb[0][0]) * (
            inner_display_aabb[1][1] - inner_display_aabb[0][1]
        )
    if outer_display_aabb is not None:
        outer_area = (outer_display_aabb[1][0] - outer_display_aabb[0][0]) * (
            outer_display_aabb[1][1] - outer_display_aabb[0][1]
        )
    ctx.check(
        "inner display is substantially larger than the outer display window",
        inner_area is not None and outer_area is not None and inner_area > outer_area * 8.0,
        details=f"inner_area={inner_area}, outer_area={outer_area}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
