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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modular_din_rail_enclosure")

    housing_plastic = model.material("housing_plastic", rgba=(0.84, 0.85, 0.87, 1.0))
    cover_clear = model.material("cover_clear", rgba=(0.63, 0.74, 0.83, 0.38))
    tray_metal = model.material("tray_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.67, 0.69, 0.72, 1.0))
    latch_plastic = model.material("latch_plastic", rgba=(0.25, 0.27, 0.30, 1.0))

    width = 0.220
    depth = 0.120
    height = 0.270
    wall_t = 0.004
    top_t = 0.010
    bottom_t = 0.006

    door_width = 0.214
    door_height = 0.184
    door_thickness = 0.004
    door_axis_z = 0.258
    door_knuckle_radius = 0.005
    door_knuckle_len = 0.030
    door_knuckle_x = 0.071

    tray_width = 0.202
    tray_depth = 0.095
    tray_thickness = 0.0025
    tray_closed_y = 0.012
    tray_travel = 0.050
    runner_width = 0.006
    runner_height = 0.022
    runner_depth = 0.104
    runner_top_z = bottom_t + runner_height
    tray_center_z = runner_top_z + tray_thickness / 2.0

    housing = model.part("housing")
    housing.visual(
        Box((width, wall_t, height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall_t / 2.0, height / 2.0)),
        material=housing_plastic,
        name="back_shell",
    )
    housing.visual(
        Box((wall_t, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall_t / 2.0, 0.0, height / 2.0)),
        material=housing_plastic,
        name="left_wall",
    )
    housing.visual(
        Box((wall_t, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall_t / 2.0, 0.0, height / 2.0)),
        material=housing_plastic,
        name="right_wall",
    )
    housing.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, height - top_t / 2.0)),
        material=housing_plastic,
        name="top_cap",
    )
    housing.visual(
        Box((width, depth, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=housing_plastic,
        name="bottom_base",
    )
    housing.visual(
        Box((width - 2.0 * wall_t, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.003, 0.066)),
        material=housing_plastic,
        name="front_crossbar",
    )
    housing.visual(
        Box((runner_width, runner_depth, runner_height)),
        origin=Origin(
            xyz=(
                -width / 2.0 + wall_t + runner_width / 2.0,
                0.006,
                bottom_t + runner_height / 2.0,
            )
        ),
        material=housing_plastic,
        name="left_runner",
    )
    housing.visual(
        Box((runner_width, runner_depth, runner_height)),
        origin=Origin(
            xyz=(
                width / 2.0 - wall_t - runner_width / 2.0,
                0.006,
                bottom_t + runner_height / 2.0,
            )
        ),
        material=housing_plastic,
        name="right_runner",
    )
    housing.visual(
        Cylinder(radius=door_knuckle_radius, length=0.034),
        origin=Origin(
            xyz=(-door_knuckle_x, depth / 2.0 - door_knuckle_radius, door_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=housing_plastic,
        name="left_hinge_mount",
    )
    housing.visual(
        Cylinder(radius=door_knuckle_radius, length=0.034),
        origin=Origin(
            xyz=(door_knuckle_x, depth / 2.0 - door_knuckle_radius, door_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=housing_plastic,
        name="right_hinge_mount",
    )
    housing.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    door = model.part("cover_door")
    door_outer = rounded_rect_profile(door_width, door_height, 0.010, corner_segments=10)
    door_inner = rounded_rect_profile(door_width - 0.032, door_height - 0.036, 0.006, corner_segments=8)
    door_frame_geom = ExtrudeWithHolesGeometry(
        door_outer,
        [door_inner],
        door_thickness,
        cap=True,
        center=True,
    )
    door_frame_geom.rotate_x(math.pi / 2.0)
    door_frame_mesh = mesh_from_geometry(door_frame_geom, "cover_door_frame")
    door.visual(
        door_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, -door_height / 2.0)),
        material=cover_clear,
        name="door_frame",
    )
    door.visual(
        Box((door_width - 0.026, door_thickness * 0.45, door_height - 0.030)),
        origin=Origin(xyz=(0.0, -0.0007, -door_height / 2.0)),
        material=cover_clear,
        name="window_pane",
    )
    door.visual(
        Cylinder(radius=door_knuckle_radius, length=door_knuckle_len),
        origin=Origin(
            xyz=(-door_knuckle_x, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cover_clear,
        name="left_hinge_knuckle",
    )
    door.visual(
        Cylinder(radius=door_knuckle_radius, length=door_knuckle_len),
        origin=Origin(
            xyz=(door_knuckle_x, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cover_clear,
        name="right_hinge_knuckle",
    )
    door.visual(
        Box((0.040, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, door_thickness / 2.0 + 0.003, -door_height + 0.018)),
        material=latch_plastic,
        name="pull_tab",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.014, door_height)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -door_height / 2.0)),
    )

    mounting_plate = model.part("mounting_plate")
    mounting_plate.visual(
        Box((tray_width, tray_depth, tray_thickness)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tray_metal,
        name="tray_base",
    )
    flange_height = 0.012
    flange_thickness = 0.002
    mounting_plate.visual(
        Box((flange_thickness, tray_depth, flange_height)),
        origin=Origin(
            xyz=(-tray_width / 2.0 + flange_thickness / 2.0, 0.0, -(tray_thickness + flange_height) / 2.0)
        ),
        material=tray_metal,
        name="left_flange",
    )
    mounting_plate.visual(
        Box((flange_thickness, tray_depth, flange_height)),
        origin=Origin(
            xyz=(tray_width / 2.0 - flange_thickness / 2.0, 0.0, -(tray_thickness + flange_height) / 2.0)
        ),
        material=tray_metal,
        name="right_flange",
    )
    mounting_plate.visual(
        Box((0.048, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, tray_depth / 2.0 + 0.002, -0.002)),
        material=tray_metal,
        name="front_pull",
    )
    rail_length = 0.170
    mounting_plate.visual(
        Box((rail_length, 0.035, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, tray_thickness / 2.0 + 0.00075)),
        material=rail_metal,
        name="rail_base_flange",
    )
    mounting_plate.visual(
        Box((rail_length, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, tray_thickness / 2.0 + 0.0015 + 0.003)),
        material=rail_metal,
        name="rail_web",
    )
    mounting_plate.visual(
        Box((rail_length, 0.024, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, tray_thickness / 2.0 + 0.0015 + 0.006 + 0.00075)),
        material=rail_metal,
        name="rail_top_hat",
    )
    mounting_plate.inertial = Inertial.from_geometry(
        Box((tray_width, tray_depth + 0.010, 0.026)),
        mass=0.68,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "housing_to_cover_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, depth / 2.0 + door_thickness / 2.0, door_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )

    model.articulation(
        "housing_to_mounting_plate",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=mounting_plate,
        origin=Origin(xyz=(0.0, tray_closed_y, tray_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=tray_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("cover_door")
    mounting_plate = object_model.get_part("mounting_plate")
    door_hinge = object_model.get_articulation("housing_to_cover_door")
    tray_slide = object_model.get_articulation("housing_to_mounting_plate")

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
        positive_elem="door_frame",
        max_gap=0.001,
        max_penetration=0.0,
        name="cover door closes onto the housing front plane",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="x",
        elem_a="door_frame",
        min_overlap=0.200,
        name="cover door spans the enclosure opening width",
    )

    ctx.expect_contact(
        mounting_plate,
        housing,
        elem_a="tray_base",
        elem_b="left_runner",
        name="tray base is supported by the left runner",
    )
    ctx.expect_contact(
        mounting_plate,
        housing,
        elem_a="tray_base",
        elem_b="right_runner",
        name="tray base is supported by the right runner",
    )
    ctx.expect_within(
        mounting_plate,
        housing,
        axes="x",
        inner_elem="tray_base",
        margin=0.0,
        name="tray stays centered between the enclosure walls",
    )
    ctx.expect_overlap(
        mounting_plate,
        housing,
        axes="y",
        elem_a="tray_base",
        min_overlap=0.090,
        name="closed tray remains deeply inserted in the housing",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    with ctx.pose({door_hinge: math.radians(105.0)}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
        ctx.check(
            "cover door opens upward about the top edge",
            closed_door_aabb is not None
            and open_door_aabb is not None
            and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.050
            and open_door_aabb[0][2] > closed_door_aabb[0][2] + 0.060,
            details=f"closed={closed_door_aabb}, open={open_door_aabb}",
        )

    closed_tray_aabb = ctx.part_element_world_aabb(mounting_plate, elem="tray_base")
    with ctx.pose({tray_slide: 0.050}):
        extended_tray_aabb = ctx.part_element_world_aabb(mounting_plate, elem="tray_base")
        ctx.expect_within(
            mounting_plate,
            housing,
            axes="x",
            inner_elem="tray_base",
            margin=0.0,
            name="extended tray stays laterally guided by the housing",
        )
        ctx.expect_overlap(
            mounting_plate,
            housing,
            axes="y",
            elem_a="tray_base",
            min_overlap=0.040,
            name="extended tray retains insertion in the housing",
        )
        ctx.check(
            "mounting plate slides forward for service access",
            closed_tray_aabb is not None
            and extended_tray_aabb is not None
            and extended_tray_aabb[1][1] > closed_tray_aabb[1][1] + 0.045,
            details=f"closed={closed_tray_aabb}, extended={extended_tray_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
