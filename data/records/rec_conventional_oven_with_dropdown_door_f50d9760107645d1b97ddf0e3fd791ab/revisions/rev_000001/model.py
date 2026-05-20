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
    model = ArticulatedObject(name="double_oven_with_storage_drawer")

    stainless = model.material("stainless", rgba=(0.72, 0.73, 0.75, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    glass = model.material("glass", rgba=(0.08, 0.12, 0.16, 0.35))
    cavity_dark = model.material("cavity_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    housing_width = 0.76
    housing_depth = 0.65
    housing_height = 1.50
    front_y = housing_depth / 2.0

    side_t = 0.03
    top_t = 0.03
    bottom_t = 0.03
    back_t = 0.02
    liner_t = 0.018

    drawer_bottom_z = 0.04
    drawer_height = 0.18
    drawer_top_band_h = 0.06

    lower_door_bottom_z = 0.28
    door_height = 0.50
    middle_band_h = 0.08
    upper_door_bottom_z = 0.86
    top_band_h = 0.11

    door_width = 0.68
    door_thickness = 0.045
    drawer_front_width = 0.70
    drawer_front_thickness = 0.02

    cavity_inner_width = 0.64
    cavity_depth = 0.60
    cavity_front_y = front_y - 0.03
    cavity_center_y = cavity_front_y - cavity_depth / 2.0
    cavity_back_center_y = cavity_front_y - cavity_depth + liner_t / 2.0

    def add_bar_handle(
        part,
        *,
        width: float,
        front_face_y: float,
        z: float,
        bar_length: float,
        material,
        prefix: str,
    ) -> None:
        bar_radius = 0.0085
        post_radius = 0.006
        post_length = 0.024
        bar_center_y = front_face_y + 0.022
        post_center_y = front_face_y + post_length / 2.0
        post_x = bar_length * 0.33

        part.visual(
            Cylinder(radius=bar_radius, length=bar_length),
            origin=Origin(xyz=(0.0, bar_center_y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=material,
            name=f"{prefix}_bar",
        )
        part.visual(
            Cylinder(radius=post_radius, length=post_length),
            origin=Origin(xyz=(-post_x, post_center_y, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_post_left",
        )
        part.visual(
            Cylinder(radius=post_radius, length=post_length),
            origin=Origin(xyz=(post_x, post_center_y, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_post_right",
        )

    def add_drop_down_door(part, *, prefix: str) -> None:
        frame_side = 0.055
        frame_top = 0.08
        frame_bottom = 0.07
        center_glass_h = door_height - frame_top - frame_bottom + 0.006
        center_glass_w = door_width - 2.0 * (frame_side - 0.004)

        part.visual(
            Box((door_width, door_thickness, frame_bottom)),
            origin=Origin(xyz=(0.0, 0.0, frame_bottom / 2.0)),
            material=charcoal,
            name="bottom_frame",
        )
        part.visual(
            Box((door_width, door_thickness, frame_top)),
            origin=Origin(xyz=(0.0, 0.0, door_height - frame_top / 2.0)),
            material=charcoal,
            name="top_frame",
        )
        side_height = door_height - frame_top - frame_bottom + 0.002
        side_center_z = frame_bottom + side_height / 2.0
        part.visual(
            Box((frame_side, door_thickness, side_height)),
            origin=Origin(xyz=(-door_width / 2.0 + frame_side / 2.0, 0.0, side_center_z)),
            material=charcoal,
            name="left_frame",
        )
        part.visual(
            Box((frame_side, door_thickness, side_height)),
            origin=Origin(xyz=(door_width / 2.0 - frame_side / 2.0, 0.0, side_center_z)),
            material=charcoal,
            name="right_frame",
        )
        part.visual(
            Box((center_glass_w, 0.012, center_glass_h)),
            origin=Origin(xyz=(0.0, -0.004, frame_bottom + (door_height - frame_top - frame_bottom) / 2.0)),
            material=glass,
            name="glass_pane",
        )
        add_bar_handle(
            part,
            width=door_width,
            front_face_y=door_thickness / 2.0,
            z=door_height - 0.06,
            bar_length=door_width * 0.74,
            material=stainless,
            prefix=prefix,
        )

    housing = model.part("housing")
    housing.visual(
        Box((side_t, housing_depth, housing_height)),
        origin=Origin(xyz=(-housing_width / 2.0 + side_t / 2.0, 0.0, housing_height / 2.0)),
        material=stainless,
        name="left_side_panel",
    )
    housing.visual(
        Box((side_t, housing_depth, housing_height)),
        origin=Origin(xyz=(housing_width / 2.0 - side_t / 2.0, 0.0, housing_height / 2.0)),
        material=stainless,
        name="right_side_panel",
    )
    housing.visual(
        Box((housing_width, housing_depth, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=stainless,
        name="bottom_panel",
    )
    housing.visual(
        Box((housing_width, housing_depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, housing_height - top_t / 2.0)),
        material=stainless,
        name="top_panel",
    )
    housing.visual(
        Box((housing_width - 2.0 * side_t, back_t, housing_height - top_t - bottom_t)),
        origin=Origin(
            xyz=(
                0.0,
                -housing_depth / 2.0 + back_t / 2.0,
                bottom_t + (housing_height - top_t - bottom_t) / 2.0,
            )
        ),
        material=stainless,
        name="back_panel",
    )
    housing.visual(
        Box((housing_width - 2.0 * side_t, housing_depth, drawer_top_band_h)),
        origin=Origin(xyz=(0.0, 0.0, drawer_bottom_z + drawer_height + drawer_top_band_h / 2.0)),
        material=stainless,
        name="drawer_band",
    )
    housing.visual(
        Box((housing_width - 2.0 * side_t, housing_depth, middle_band_h)),
        origin=Origin(xyz=(0.0, 0.0, lower_door_bottom_z + door_height + middle_band_h / 2.0)),
        material=stainless,
        name="middle_band",
    )
    housing.visual(
        Box((housing_width - 2.0 * side_t, housing_depth, top_band_h)),
        origin=Origin(xyz=(0.0, 0.0, upper_door_bottom_z + door_height + top_band_h / 2.0)),
        material=stainless,
        name="top_band",
    )
    housing.visual(
        Box((0.60, 0.008, 0.042)),
        origin=Origin(xyz=(0.0, front_y - 0.004, lower_door_bottom_z + door_height + middle_band_h / 2.0)),
        material=charcoal,
        name="middle_control_face",
    )
    housing.visual(
        Box((0.60, 0.008, 0.05)),
        origin=Origin(xyz=(0.0, front_y - 0.004, upper_door_bottom_z + door_height + top_band_h / 2.0)),
        material=charcoal,
        name="top_control_face",
    )

    for prefix, cavity_bottom_z in (
        ("lower", lower_door_bottom_z),
        ("upper", upper_door_bottom_z),
    ):
        cavity_center_z = cavity_bottom_z + door_height / 2.0
        housing.visual(
            Box((liner_t, cavity_depth, door_height)),
            origin=Origin(
                xyz=(-cavity_inner_width / 2.0 - liner_t / 2.0, cavity_center_y, cavity_center_z)
            ),
            material=cavity_dark,
            name=f"{prefix}_cavity_left",
        )
        housing.visual(
            Box((liner_t, cavity_depth, door_height)),
            origin=Origin(
                xyz=(cavity_inner_width / 2.0 + liner_t / 2.0, cavity_center_y, cavity_center_z)
            ),
            material=cavity_dark,
            name=f"{prefix}_cavity_right",
        )
        housing.visual(
            Box((cavity_inner_width, cavity_depth, liner_t)),
            origin=Origin(xyz=(0.0, cavity_center_y, cavity_bottom_z + liner_t / 2.0)),
            material=cavity_dark,
            name=f"{prefix}_cavity_floor",
        )
        housing.visual(
            Box((cavity_inner_width, cavity_depth, liner_t)),
            origin=Origin(xyz=(0.0, cavity_center_y, cavity_bottom_z + door_height - liner_t / 2.0)),
            material=cavity_dark,
            name=f"{prefix}_cavity_ceiling",
        )
        housing.visual(
            Box((cavity_inner_width, liner_t, door_height - 2.0 * liner_t)),
            origin=Origin(xyz=(0.0, cavity_back_center_y, cavity_center_z)),
            material=cavity_dark,
            name=f"{prefix}_cavity_back",
        )

    housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, housing_height)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, housing_height / 2.0)),
    )

    upper_door = model.part("upper_door")
    add_drop_down_door(upper_door, prefix="upper_handle")
    upper_door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, door_height / 2.0)),
    )

    lower_door = model.part("lower_door")
    add_drop_down_door(lower_door, prefix="lower_handle")
    lower_door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, door_height / 2.0)),
    )

    drawer = model.part("storage_drawer")
    drawer_shell_depth = 0.48
    drawer_shell_width = 0.66
    drawer_side_height = 0.12
    drawer_wall_t = 0.015
    drawer_bottom_t = 0.015
    drawer.visual(
        Box((drawer_front_width, drawer_front_thickness, drawer_height)),
        origin=Origin(xyz=(0.0, 0.0, drawer_height / 2.0)),
        material=stainless,
        name="front_panel",
    )
    drawer.visual(
        Box((drawer_wall_t, drawer_shell_depth, drawer_side_height)),
        origin=Origin(
            xyz=(
                -drawer_shell_width / 2.0 + drawer_wall_t / 2.0,
                -(drawer_front_thickness / 2.0 + drawer_shell_depth / 2.0),
                drawer_side_height / 2.0,
            )
        ),
        material=charcoal,
        name="left_wall",
    )
    drawer.visual(
        Box((drawer_wall_t, drawer_shell_depth, drawer_side_height)),
        origin=Origin(
            xyz=(
                drawer_shell_width / 2.0 - drawer_wall_t / 2.0,
                -(drawer_front_thickness / 2.0 + drawer_shell_depth / 2.0),
                drawer_side_height / 2.0,
            )
        ),
        material=charcoal,
        name="right_wall",
    )
    drawer.visual(
        Box((drawer_shell_width - 2.0 * drawer_wall_t, drawer_shell_depth, drawer_bottom_t)),
        origin=Origin(
            xyz=(
                0.0,
                -(drawer_front_thickness / 2.0 + drawer_shell_depth / 2.0),
                drawer_bottom_t / 2.0,
            )
        ),
        material=charcoal,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((drawer_shell_width, drawer_wall_t, drawer_side_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(drawer_front_thickness / 2.0 + drawer_shell_depth - drawer_wall_t / 2.0),
                drawer_side_height / 2.0,
            )
        ),
        material=charcoal,
        name="back_wall",
    )
    add_bar_handle(
        drawer,
        width=drawer_front_width,
        front_face_y=drawer_front_thickness / 2.0,
        z=drawer_height - 0.055,
        bar_length=drawer_front_width * 0.72,
        material=stainless,
        prefix="drawer_handle",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((drawer_front_width, drawer_shell_depth + drawer_front_thickness, drawer_height)),
        mass=8.0,
        origin=Origin(xyz=(0.0, -drawer_shell_depth / 2.0, drawer_height / 2.0)),
    )

    model.articulation(
        "housing_to_upper_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=upper_door,
        origin=Origin(xyz=(0.0, front_y + door_thickness / 2.0, upper_door_bottom_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "housing_to_lower_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lower_door,
        origin=Origin(xyz=(0.0, front_y + door_thickness / 2.0, lower_door_bottom_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "housing_to_storage_drawer",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(xyz=(0.0, front_y + drawer_front_thickness / 2.0, drawer_bottom_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.3, lower=0.0, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    upper_door = object_model.get_part("upper_door")
    lower_door = object_model.get_part("lower_door")
    drawer = object_model.get_part("storage_drawer")
    upper_hinge = object_model.get_articulation("housing_to_upper_door")
    lower_hinge = object_model.get_articulation("housing_to_lower_door")
    drawer_slide = object_model.get_articulation("housing_to_storage_drawer")

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
        "upper door hinge axis points along negative x",
        tuple(upper_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={upper_hinge.axis}",
    )
    ctx.check(
        "lower door hinge axis points along negative x",
        tuple(lower_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={lower_hinge.axis}",
    )
    ctx.check(
        "drawer slide axis points outward along positive y",
        tuple(drawer_slide.axis) == (0.0, 1.0, 0.0),
        details=f"axis={drawer_slide.axis}",
    )

    ctx.expect_gap(
        upper_door,
        housing,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper door closes flush with the housing face",
    )
    ctx.expect_gap(
        lower_door,
        housing,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower door closes flush with the housing face",
    )
    ctx.expect_gap(
        drawer,
        housing,
        axis="y",
        positive_elem="front_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="drawer front closes flush with the housing face",
    )
    ctx.expect_overlap(
        upper_door,
        housing,
        axes="xz",
        min_overlap=0.18,
        name="upper door covers its opening footprint",
    )
    ctx.expect_overlap(
        lower_door,
        housing,
        axes="xz",
        min_overlap=0.18,
        name="lower door covers its opening footprint",
    )
    ctx.expect_within(
        drawer,
        housing,
        axes="x",
        inner_elem="drawer_bottom",
        margin=0.0,
        name="drawer box stays centered between the housing sides at rest",
    )

    upper_closed_aabb = ctx.part_world_aabb(upper_door)
    with ctx.pose({upper_hinge: 1.15}):
        upper_open_aabb = ctx.part_world_aabb(upper_door)
    ctx.check(
        "upper door swings forward and lowers its top edge when opened",
        upper_closed_aabb is not None
        and upper_open_aabb is not None
        and upper_open_aabb[1][1] > upper_closed_aabb[1][1] + 0.18
        and upper_open_aabb[1][2] < upper_closed_aabb[1][2] - 0.22,
        details=f"closed={upper_closed_aabb}, open={upper_open_aabb}",
    )

    lower_closed_aabb = ctx.part_world_aabb(lower_door)
    with ctx.pose({lower_hinge: 1.15}):
        lower_open_aabb = ctx.part_world_aabb(lower_door)
    ctx.check(
        "lower door swings forward and lowers its top edge when opened",
        lower_closed_aabb is not None
        and lower_open_aabb is not None
        and lower_open_aabb[1][1] > lower_closed_aabb[1][1] + 0.18
        and lower_open_aabb[1][2] < lower_closed_aabb[1][2] - 0.22,
        details=f"closed={lower_closed_aabb}, open={lower_open_aabb}",
    )

    drawer_closed_aabb = ctx.part_world_aabb(drawer)
    drawer_upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None
    with ctx.pose({drawer_slide: 0.28 if drawer_upper is None else drawer_upper}):
        drawer_open_aabb = ctx.part_world_aabb(drawer)
        ctx.expect_within(
            drawer,
            housing,
            axes="x",
            inner_elem="drawer_bottom",
            margin=0.0,
            name="drawer box stays centered between the housing sides when extended",
        )
        ctx.expect_overlap(
            drawer,
            housing,
            axes="y",
            elem_a="drawer_bottom",
            min_overlap=0.16,
            name="drawer retains insertion when extended",
        )
    ctx.check(
        "drawer slides outward from the housing",
        drawer_closed_aabb is not None
        and drawer_open_aabb is not None
        and drawer_open_aabb[1][1] > drawer_closed_aabb[1][1] + 0.24,
        details=f"closed={drawer_closed_aabb}, open={drawer_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
