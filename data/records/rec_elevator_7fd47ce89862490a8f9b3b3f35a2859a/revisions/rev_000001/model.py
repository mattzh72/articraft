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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="piston_residential_elevator")

    frame_mtl = model.material("frame_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    rail_mtl = model.material("rail_steel", rgba=(0.42, 0.45, 0.48, 1.0))
    car_mtl = model.material("car_shell", rgba=(0.78, 0.80, 0.82, 1.0))
    door_mtl = model.material("door_paint", rgba=(0.92, 0.93, 0.90, 1.0))
    chrome_mtl = model.material("chrome", rgba=(0.86, 0.88, 0.90, 1.0))

    outer_w = 1.28
    outer_d = 1.48
    post = 0.08
    pit_depth = 0.55
    base_h = 0.10
    travel = 2.90
    door_w = 0.94
    door_h = 2.03
    door_t = 0.04
    knuckle_r = 0.012
    knuckle_len = 0.18
    car_w = 0.92
    car_d = 1.02
    floor_t = 0.05
    wall_t = 0.03
    car_clear_h = 2.02
    roof_t = 0.03
    top_frame_bottom = travel + door_h + 0.24
    top_frame_h = 0.08
    total_top = top_frame_bottom + top_frame_h
    post_h = total_top + pit_depth

    front_y = -(outer_d / 2.0 - post / 2.0)
    rear_y = outer_d / 2.0 - post / 2.0
    left_x = -(outer_w / 2.0 - post / 2.0)
    right_x = outer_w / 2.0 - post / 2.0
    rail_w = 0.05
    rail_d = 0.04
    rail_h = 5.00
    rail_z = rail_h / 2.0
    left_rail_x = -(outer_w / 2.0 - post - rail_w / 2.0)
    right_rail_x = outer_w / 2.0 - post - rail_w / 2.0
    rail_y = rear_y - post / 2.0 - rail_d / 2.0
    jamb_x = door_w / 2.0 + 0.04
    jamb_w = 0.08
    door_origin_x = -(door_w / 2.0 + knuckle_r)
    door_y = -(outer_d / 2.0 + door_t / 2.0)

    shaft = model.part("shaft_frame")

    def add_box(part, name, size, xyz, material):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    def add_cyl(part, name, radius, length, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    # Pit/base ring.
    add_box(shaft, "elem_base_front", (outer_w, post, base_h), (0.0, front_y, -pit_depth + base_h / 2.0), frame_mtl)
    add_box(shaft, "elem_base_rear", (outer_w, post, base_h), (0.0, rear_y, -pit_depth + base_h / 2.0), frame_mtl)
    add_box(
        shaft,
        "elem_base_left",
        (post, outer_d - 2.0 * post, base_h),
        (left_x, 0.0, -pit_depth + base_h / 2.0),
        frame_mtl,
    )
    add_box(
        shaft,
        "elem_base_right",
        (post, outer_d - 2.0 * post, base_h),
        (right_x, 0.0, -pit_depth + base_h / 2.0),
        frame_mtl,
    )

    # Corner posts.
    for px, py, name in (
        (left_x, front_y, "elem_post_front_left"),
        (right_x, front_y, "elem_post_front_right"),
        (left_x, rear_y, "elem_post_rear_left"),
        (right_x, rear_y, "elem_post_rear_right"),
    ):
        add_box(
            shaft,
            name,
            (post, post, post_h),
            (px, py, -pit_depth + post_h / 2.0),
            frame_mtl,
        )

    # Intermediate landing tie ring at the upper landing.
    mid_z = travel + top_frame_h / 2.0
    add_box(shaft, "elem_mid_rear", (outer_w, post, top_frame_h), (0.0, rear_y, mid_z), frame_mtl)
    add_box(
        shaft,
        "elem_mid_left",
        (post, outer_d - 2.0 * post, top_frame_h),
        (left_x, 0.0, mid_z),
        frame_mtl,
    )
    add_box(
        shaft,
        "elem_mid_right",
        (post, outer_d - 2.0 * post, top_frame_h),
        (right_x, 0.0, mid_z),
        frame_mtl,
    )

    # Top ring beam.
    top_z = top_frame_bottom + top_frame_h / 2.0
    add_box(shaft, "elem_top_front", (outer_w, post, top_frame_h), (0.0, front_y, top_z), frame_mtl)
    add_box(shaft, "elem_top_rear", (outer_w, post, top_frame_h), (0.0, rear_y, top_z), frame_mtl)
    add_box(
        shaft,
        "elem_top_left",
        (post, outer_d - 2.0 * post, top_frame_h),
        (left_x, 0.0, top_z),
        frame_mtl,
    )
    add_box(
        shaft,
        "elem_top_right",
        (post, outer_d - 2.0 * post, top_frame_h),
        (right_x, 0.0, top_z),
        frame_mtl,
    )

    # Landing door frames built into the front of the shaft.
    for prefix, sill_z in (("lower", 0.0), ("upper", travel)):
        add_box(
            shaft,
            f"elem_{prefix}_sill",
            (outer_w - 2.0 * post, post, floor_t),
            (0.0, front_y, sill_z - floor_t / 2.0),
            frame_mtl,
        )
        add_box(
            shaft,
            f"elem_{prefix}_header",
            (outer_w - 2.0 * post, post, top_frame_h),
            (0.0, front_y, sill_z + door_h + top_frame_h / 2.0),
            frame_mtl,
        )
        add_box(
            shaft,
            f"elem_{prefix}_jamb_left",
            (jamb_w, post, door_h),
            (-jamb_x, front_y, sill_z + door_h / 2.0),
            frame_mtl,
        )
        add_box(
            shaft,
            f"elem_{prefix}_jamb_right",
            (jamb_w, post, door_h),
            (jamb_x, front_y, sill_z + door_h / 2.0),
            frame_mtl,
        )

    # Guide rails tied directly to the rear posts.
    add_box(shaft, "elem_left_rail", (rail_w, rail_d, rail_h), (left_rail_x, rail_y, rail_z), rail_mtl)
    add_box(shaft, "elem_right_rail", (rail_w, rail_d, rail_h), (right_rail_x, rail_y, rail_z), rail_mtl)

    # Hollow hydraulic jack sleeve below the car, connected back to the base frame.
    sleeve_outer = 0.24
    sleeve_wall = 0.02
    sleeve_h = 0.44
    sleeve_center_z = -0.32
    sleeve_half = sleeve_outer / 2.0
    sleeve_wall_c = sleeve_half - sleeve_wall / 2.0
    collar_span = sleeve_outer - 2.0 * sleeve_wall
    add_box(
        shaft,
        "elem_ram_sleeve_left",
        (sleeve_wall, sleeve_outer, sleeve_h),
        (-sleeve_wall_c, 0.0, sleeve_center_z),
        frame_mtl,
    )
    add_box(
        shaft,
        "elem_ram_sleeve_right",
        (sleeve_wall, sleeve_outer, sleeve_h),
        (sleeve_wall_c, 0.0, sleeve_center_z),
        frame_mtl,
    )
    add_box(
        shaft,
        "elem_ram_sleeve_front",
        (collar_span, sleeve_wall, sleeve_h),
        (0.0, -sleeve_wall_c, sleeve_center_z),
        frame_mtl,
    )
    add_box(
        shaft,
        "elem_ram_sleeve_rear",
        (collar_span, sleeve_wall, sleeve_h),
        (0.0, sleeve_wall_c, sleeve_center_z),
        frame_mtl,
    )
    for prefix, collar_z in (("bottom", sleeve_center_z - sleeve_h / 2.0 + sleeve_wall / 2.0), ("top", sleeve_center_z + sleeve_h / 2.0 - sleeve_wall / 2.0)):
        add_box(
            shaft,
            f"elem_ram_{prefix}_collar_x",
            (sleeve_outer, sleeve_wall, sleeve_wall),
            (0.0, -sleeve_wall_c, collar_z),
            frame_mtl,
        )
        add_box(
            shaft,
            f"elem_ram_{prefix}_collar_x_rear",
            (sleeve_outer, sleeve_wall, sleeve_wall),
            (0.0, sleeve_wall_c, collar_z),
            frame_mtl,
        )
        add_box(
            shaft,
            f"elem_ram_{prefix}_collar_y_left",
            (sleeve_wall, collar_span, sleeve_wall),
            (-sleeve_wall_c, 0.0, collar_z),
            frame_mtl,
        )
        add_box(
            shaft,
            f"elem_ram_{prefix}_collar_y_right",
            (sleeve_wall, collar_span, sleeve_wall),
            (sleeve_wall_c, 0.0, collar_z),
            frame_mtl,
        )
    add_box(shaft, "elem_ram_mount_left", (0.04, 0.56, 0.06), (-0.09, 0.39, -0.52), frame_mtl)
    add_box(shaft, "elem_ram_mount_right", (0.04, 0.56, 0.06), (0.09, 0.39, -0.52), frame_mtl)

    car = model.part("car")
    add_box(car, "elem_car_floor", (car_w, car_d, floor_t), (0.0, 0.0, -floor_t / 2.0), car_mtl)
    add_box(
        car,
        "elem_car_rear_wall",
        (car_w, wall_t, car_clear_h),
        (0.0, car_d / 2.0 - wall_t / 2.0, car_clear_h / 2.0),
        car_mtl,
    )
    add_box(
        car,
        "elem_car_left_wall",
        (wall_t, car_d, car_clear_h),
        (-(car_w / 2.0 - wall_t / 2.0), 0.0, car_clear_h / 2.0),
        car_mtl,
    )
    add_box(
        car,
        "elem_car_right_wall",
        (wall_t, car_d, car_clear_h),
        (car_w / 2.0 - wall_t / 2.0, 0.0, car_clear_h / 2.0),
        car_mtl,
    )
    add_box(
        car,
        "elem_car_roof",
        (car_w, car_d, roof_t),
        (0.0, 0.0, car_clear_h + roof_t / 2.0),
        car_mtl,
    )

    shoe_size = (0.06, 0.11, 0.07)
    shoe_y = 0.565
    for side, sx in (("left", -(car_w / 2.0 + 0.02)), ("right", car_w / 2.0 + 0.02)):
        add_box(car, f"elem_car_{side}_shoe_lower", shoe_size, (sx, shoe_y, 0.35), rail_mtl)
        add_box(car, f"elem_car_{side}_shoe_upper", shoe_size, (sx, shoe_y, 1.72), rail_mtl)

    add_box(car, "elem_ram_head", (0.16, 0.16, 0.02), (0.0, 0.0, -0.06), frame_mtl)
    add_cyl(car, "elem_piston_ram", 0.045, 3.20, (0.0, 0.0, -1.65), chrome_mtl)

    lower_door = model.part("lower_lobby_door")
    upper_door = model.part("upper_lobby_door")

    def build_door(part, prefix):
        add_box(
            part,
            f"elem_{prefix}_door_panel",
            (door_w, door_t, door_h),
            (knuckle_r + door_w / 2.0, 0.0, door_h / 2.0),
            door_mtl,
        )
        add_box(
            part,
            f"elem_{prefix}_door_pull",
            (0.03, 0.05, 0.24),
            (knuckle_r + door_w - 0.07, -0.005, 1.02),
            chrome_mtl,
        )
        for idx, kz in enumerate((0.24, door_h / 2.0, door_h - 0.24), start=1):
            add_cyl(
                part,
                f"elem_{prefix}_knuckle_{idx}",
                knuckle_r,
                knuckle_len,
                (0.0, 0.0, kz),
                chrome_mtl,
            )

    build_door(lower_door, "lower")
    build_door(upper_door, "upper")

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3500.0,
            velocity=0.30,
            lower=0.0,
            upper=travel,
        ),
    )
    model.articulation(
        "shaft_to_lower_lobby_door",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=lower_door,
        origin=Origin(xyz=(door_origin_x, door_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "shaft_to_upper_lobby_door",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=upper_door,
        origin=Origin(xyz=(door_origin_x, door_y, travel)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=1.35,
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

    shaft = object_model.get_part("shaft_frame")
    car = object_model.get_part("car")
    lower_door = object_model.get_part("lower_lobby_door")
    upper_door = object_model.get_part("upper_lobby_door")

    car_joint = object_model.get_articulation("shaft_to_car")
    lower_door_joint = object_model.get_articulation("shaft_to_lower_lobby_door")
    upper_door_joint = object_model.get_articulation("shaft_to_upper_lobby_door")

    ctx.check(
        "car guide joint is vertical prismatic",
        car_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(car_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={car_joint.articulation_type}, axis={car_joint.axis}",
    )
    ctx.check(
        "landing doors hinge on vertical knuckles",
        lower_door_joint.articulation_type == ArticulationType.REVOLUTE
        and upper_door_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lower_door_joint.axis) == (0.0, 0.0, -1.0)
        and tuple(upper_door_joint.axis) == (0.0, 0.0, -1.0),
        details=(
            f"lower={lower_door_joint.articulation_type, lower_door_joint.axis}, "
            f"upper={upper_door_joint.articulation_type, upper_door_joint.axis}"
        ),
    )

    ctx.expect_contact(
        car,
        shaft,
        elem_a="elem_car_left_shoe_lower",
        elem_b="elem_left_rail",
        contact_tol=0.001,
        name="lower left guide shoe rides on the left rail",
    )
    ctx.expect_contact(
        car,
        shaft,
        elem_a="elem_car_right_shoe_lower",
        elem_b="elem_right_rail",
        contact_tol=0.001,
        name="lower right guide shoe rides on the right rail",
    )

    rest_car_pos = ctx.part_world_position(car)
    lower_door_closed = ctx.part_element_world_aabb(lower_door, elem="elem_lower_door_panel")
    upper_door_closed = ctx.part_element_world_aabb(upper_door, elem="elem_upper_door_panel")

    with ctx.pose({car_joint: car_joint.motion_limits.upper}):
        ctx.expect_contact(
            car,
            shaft,
            elem_a="elem_car_left_shoe_upper",
            elem_b="elem_left_rail",
            contact_tol=0.001,
            name="upper left guide shoe remains on the rail at the top landing",
        )
        ctx.expect_contact(
            car,
            shaft,
            elem_a="elem_car_right_shoe_upper",
            elem_b="elem_right_rail",
            contact_tol=0.001,
            name="upper right guide shoe remains on the rail at the top landing",
        )
        upper_car_pos = ctx.part_world_position(car)

    ctx.check(
        "car rises to the upper landing",
        rest_car_pos is not None
        and upper_car_pos is not None
        and abs(rest_car_pos[0]) < 1e-6
        and abs(rest_car_pos[1]) < 1e-6
        and abs(upper_car_pos[2] - car_joint.motion_limits.upper) < 1e-6,
        details=f"rest={rest_car_pos}, upper={upper_car_pos}",
    )

    with ctx.pose({lower_door_joint: math.radians(75.0)}):
        lower_door_open = ctx.part_element_world_aabb(lower_door, elem="elem_lower_door_panel")
    ctx.check(
        "lower lobby door opens outward from the shaft",
        lower_door_closed is not None
        and lower_door_open is not None
        and lower_door_open[0][1] < lower_door_closed[0][1] - 0.15,
        details=f"closed={lower_door_closed}, open={lower_door_open}",
    )

    with ctx.pose({upper_door_joint: math.radians(75.0)}):
        upper_door_open = ctx.part_element_world_aabb(upper_door, elem="elem_upper_door_panel")
    ctx.check(
        "upper lobby door opens outward from the shaft",
        upper_door_closed is not None
        and upper_door_open is not None
        and upper_door_open[0][1] < upper_door_closed[0][1] - 0.15,
        details=f"closed={upper_door_closed}, open={upper_door_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
