from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dashboard_glove_compartment", assets=ASSETS)

    housing_plastic = model.material("housing_plastic", rgba=(0.18, 0.19, 0.20, 1.0))
    door_plastic = model.material("door_plastic", rgba=(0.26, 0.27, 0.29, 1.0))
    bin_plastic = model.material("bin_plastic", rgba=(0.09, 0.10, 0.11, 1.0))
    handle_trim = model.material("handle_trim", rgba=(0.62, 0.64, 0.66, 1.0))

    opening_width = 0.340
    opening_height = 0.136
    fascia_width = 0.460
    fascia_height = 0.220
    fascia_depth = 0.028

    side_band = (fascia_width - opening_width) * 0.5
    top_band = (fascia_height - opening_height) * 0.5

    bin_depth = 0.172
    bin_width = opening_width
    bin_height = opening_height
    bin_wall = 0.004

    door_width = 0.352
    door_height = 0.148
    door_thickness = 0.016
    hinge_radius = 0.006
    door_hinge_x = -0.006
    door_panel_center_x = -door_thickness * 0.5 - door_hinge_x

    dashboard_housing = model.part("dashboard_housing")
    bezel_outer = rounded_rect_profile(fascia_height, fascia_width, radius=0.026, corner_segments=8)
    bezel_opening = rounded_rect_profile(opening_height, opening_width, radius=0.010, corner_segments=6)
    bezel_mesh = _save_mesh(
        "dashboard_fascia_bezel.obj",
        ExtrudeWithHolesGeometry(
            bezel_outer,
            [bezel_opening],
            height=fascia_depth,
            center=True,
        ).rotate_y(math.pi * 0.5),
    )
    dashboard_housing.visual(
        bezel_mesh,
        origin=Origin(xyz=(fascia_depth * 0.5, 0.0, 0.0)),
        material=housing_plastic,
        name="fascia_bezel",
    )
    dashboard_housing.visual(
        Box((fascia_depth, side_band, fascia_height)),
        origin=Origin(xyz=(fascia_depth * 0.5, -(opening_width * 0.5 + side_band * 0.5), 0.0)),
        material=housing_plastic,
        name="fascia_left",
    )
    dashboard_housing.visual(
        Box((fascia_depth, side_band, fascia_height)),
        origin=Origin(xyz=(fascia_depth * 0.5, opening_width * 0.5 + side_band * 0.5, 0.0)),
        material=housing_plastic,
        name="fascia_right",
    )
    dashboard_housing.visual(
        Box((fascia_depth, opening_width, top_band)),
        origin=Origin(xyz=(fascia_depth * 0.5, 0.0, opening_height * 0.5 + top_band * 0.5)),
        material=housing_plastic,
        name="fascia_top",
    )
    dashboard_housing.visual(
        Box((fascia_depth, opening_width, top_band)),
        origin=Origin(xyz=(fascia_depth * 0.5, 0.0, -(opening_height * 0.5 + top_band * 0.5))),
        material=housing_plastic,
        name="fascia_bottom",
    )
    dashboard_housing.inertial = Inertial.from_geometry(
        Box((fascia_depth, fascia_width, fascia_height)),
        mass=1.8,
        origin=Origin(xyz=(fascia_depth * 0.5, 0.0, 0.0)),
    )

    storage_bin = model.part("storage_bin")
    storage_bin.visual(
        Box((bin_depth, bin_wall, bin_height)),
        origin=Origin(xyz=(bin_depth * 0.5, -(bin_width * 0.5 - bin_wall * 0.5), 0.0)),
        material=bin_plastic,
        name="bin_left_wall",
    )
    storage_bin.visual(
        Box((bin_depth, bin_wall, bin_height)),
        origin=Origin(xyz=(bin_depth * 0.5, bin_width * 0.5 - bin_wall * 0.5, 0.0)),
        material=bin_plastic,
        name="bin_right_wall",
    )
    storage_bin.visual(
        Box((bin_depth, bin_width, bin_wall)),
        origin=Origin(xyz=(bin_depth * 0.5, 0.0, bin_height * 0.5 - bin_wall * 0.5)),
        material=bin_plastic,
        name="bin_top_wall",
    )
    storage_bin.visual(
        Box((bin_depth, bin_width, bin_wall)),
        origin=Origin(xyz=(bin_depth * 0.5, 0.0, -(bin_height * 0.5 - bin_wall * 0.5))),
        material=bin_plastic,
        name="bin_bottom_wall",
    )
    storage_bin.visual(
        Box((bin_wall, bin_width, bin_height)),
        origin=Origin(xyz=(bin_depth - bin_wall * 0.5, 0.0, 0.0)),
        material=bin_plastic,
        name="bin_back_wall",
    )
    storage_bin.inertial = Inertial.from_geometry(
        Box((bin_depth, bin_width, bin_height)),
        mass=0.9,
        origin=Origin(xyz=(bin_depth * 0.5, 0.0, 0.0)),
    )

    glove_door = model.part("glove_door")
    door_shell_mesh = _save_mesh(
        "glove_door_shell.obj",
        ExtrudeGeometry(
            rounded_rect_profile(door_height, door_width, radius=0.016, corner_segments=8),
            door_thickness,
            center=True,
        ).rotate_y(math.pi * 0.5),
    )
    glove_door.visual(
        door_shell_mesh,
        origin=Origin(xyz=(door_panel_center_x, 0.0, door_height * 0.5)),
        material=door_plastic,
        name="door_panel",
    )
    inner_liner_mesh = _save_mesh(
        "glove_door_inner_liner.obj",
        ExtrudeGeometry(
            rounded_rect_profile(door_height * 0.68, door_width * 0.76, radius=0.012, corner_segments=6),
            0.004,
            center=True,
        ).rotate_y(math.pi * 0.5),
    )
    glove_door.visual(
        inner_liner_mesh,
        origin=Origin(xyz=(door_panel_center_x - 0.006, 0.0, door_height * 0.50)),
        material=bin_plastic,
        name="door_inner_liner",
    )
    glove_door.visual(
        Cylinder(radius=hinge_radius, length=door_width * 0.78),
        origin=Origin(xyz=(-door_thickness * 0.55, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=door_plastic,
        name="hinge_barrel",
    )
    handle_z = door_height * 0.72
    glove_door.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(-0.013, -0.042, handle_z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=handle_trim,
        name="handle_left_post",
    )
    glove_door.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(-0.013, 0.042, handle_z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=handle_trim,
        name="handle_right_post",
    )
    glove_door.visual(
        Cylinder(radius=0.0065, length=0.104),
        origin=Origin(xyz=(-0.019, 0.0, handle_z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=handle_trim,
        name="handle_grip",
    )
    glove_door.inertial = Inertial.from_geometry(
        Box((door_thickness, door_width, door_height)),
        mass=0.8,
        origin=Origin(xyz=(door_panel_center_x, 0.0, door_height * 0.5)),
    )

    model.articulation(
        "bin_mount",
        ArticulationType.FIXED,
        parent=dashboard_housing,
        child=storage_bin,
        origin=Origin(xyz=(fascia_depth, 0.0, 0.0)),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=dashboard_housing,
        child=glove_door,
        origin=Origin(xyz=(door_hinge_x, 0.0, -opening_height * 0.5)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=math.pi * 0.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    dashboard_housing = object_model.get_part("dashboard_housing")
    storage_bin = object_model.get_part("storage_bin")
    glove_door = object_model.get_part("glove_door")
    door_hinge = object_model.get_articulation("door_hinge")

    fascia_left = dashboard_housing.get_visual("fascia_left")
    fascia_right = dashboard_housing.get_visual("fascia_right")
    fascia_top = dashboard_housing.get_visual("fascia_top")
    fascia_bottom = dashboard_housing.get_visual("fascia_bottom")
    bin_left_wall = storage_bin.get_visual("bin_left_wall")
    bin_right_wall = storage_bin.get_visual("bin_right_wall")
    bin_top_wall = storage_bin.get_visual("bin_top_wall")
    bin_bottom_wall = storage_bin.get_visual("bin_bottom_wall")
    door_panel = glove_door.get_visual("door_panel")
    handle_grip = glove_door.get_visual("handle_grip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    housing_aabb = ctx.part_world_aabb(dashboard_housing)
    bin_aabb = ctx.part_world_aabb(storage_bin)
    door_aabb = ctx.part_world_aabb(glove_door)
    door_panel_aabb = ctx.part_element_world_aabb(glove_door, elem=door_panel)
    handle_aabb = ctx.part_element_world_aabb(glove_door, elem=handle_grip)
    assert housing_aabb is not None
    assert bin_aabb is not None
    assert door_aabb is not None
    assert door_panel_aabb is not None
    assert handle_aabb is not None

    door_width = door_aabb[1][1] - door_aabb[0][1]
    door_height = door_aabb[1][2] - door_aabb[0][2]
    bin_depth = bin_aabb[1][0] - bin_aabb[0][0]
    handle_center_y = 0.5 * (handle_aabb[0][1] + handle_aabb[1][1])
    door_center_y = 0.5 * (door_panel_aabb[0][1] + door_panel_aabb[1][1])
    handle_center_z = 0.5 * (handle_aabb[0][2] + handle_aabb[1][2])
    door_mid_z = 0.5 * (door_panel_aabb[0][2] + door_panel_aabb[1][2])

    ctx.check(
        "door_hinge_axis_is_lower_horizontal",
        tuple(round(value, 3) for value in door_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"Expected hinge axis (0, -1, 0), got {door_hinge.axis}.",
    )
    limits = door_hinge.motion_limits
    ctx.check(
        "door_hinge_limits_are_90_degrees",
        limits is not None
        and abs(limits.lower - 0.0) < 1e-9
        and abs(limits.upper - (math.pi * 0.5)) < 1e-6,
        details=f"Expected 0 to pi/2 motion range, got {limits}.",
    )

    ctx.expect_contact(
        storage_bin,
        dashboard_housing,
        elem_a=bin_left_wall,
        elem_b=fascia_left,
        name="bin_left_wall_contacts_housing",
    )
    ctx.expect_contact(
        storage_bin,
        dashboard_housing,
        elem_a=bin_right_wall,
        elem_b=fascia_right,
        name="bin_right_wall_contacts_housing",
    )
    ctx.expect_contact(
        storage_bin,
        dashboard_housing,
        elem_a=bin_top_wall,
        elem_b=fascia_top,
        name="bin_top_wall_contacts_housing",
    )
    ctx.expect_contact(
        storage_bin,
        dashboard_housing,
        elem_a=bin_bottom_wall,
        elem_b=fascia_bottom,
        name="bin_bottom_wall_contacts_housing",
    )
    ctx.expect_within(
        storage_bin,
        dashboard_housing,
        axes="yz",
        margin=0.002,
        name="bin_stays_inside_dashboard_footprint",
    )

    ctx.expect_contact(
        glove_door,
        dashboard_housing,
        elem_a=door_panel,
        elem_b=fascia_left,
        name="door_seats_on_left_frame",
    )
    ctx.expect_contact(
        glove_door,
        dashboard_housing,
        elem_a=door_panel,
        elem_b=fascia_right,
        name="door_seats_on_right_frame",
    )
    ctx.expect_contact(
        glove_door,
        dashboard_housing,
        elem_a=door_panel,
        elem_b=fascia_top,
        name="door_seats_on_top_frame",
    )
    ctx.expect_gap(
        dashboard_housing,
        glove_door,
        axis="x",
        positive_elem=fascia_left,
        negative_elem=door_panel,
        max_gap=0.001,
        max_penetration=0.0,
        name="door_sits_flush_with_dashboard_face",
    )
    ctx.expect_overlap(
        glove_door,
        dashboard_housing,
        axes="yz",
        min_overlap=0.140,
        elem_a=door_panel,
        name="door_covers_opening_area",
    )
    ctx.check(
        "handle_is_centered_on_door_width",
        abs(handle_center_y - door_center_y) <= 0.002,
        details=(
            f"Handle should be laterally centered on the door: "
            f"handle_y={handle_center_y:.3f}, door_y={door_center_y:.3f}."
        ),
    )
    ctx.check(
        "handle_sits_in_upper_half_of_door",
        handle_center_z > door_mid_z + 0.020,
        details=(
            f"Handle should sit above the vertical midpoint of the door: "
            f"handle_z={handle_center_z:.3f}, door_mid_z={door_mid_z:.3f}."
        ),
    )
    ctx.check(
        "door_is_wider_than_tall",
        door_width > door_height * 2.0,
        details=f"Door proportions read incorrectly: width={door_width:.3f}, height={door_height:.3f}.",
    )
    ctx.check(
        "bin_depth_reads_shallow",
        0.14 <= bin_depth <= 0.19,
        details=f"Expected shallow storage depth near 0.17 m, got {bin_depth:.3f}.",
    )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=12,
        ignore_adjacent=True,
        ignore_fixed=False,
    )

    with ctx.pose({door_hinge: math.pi * 0.5}):
        open_door_aabb = ctx.part_world_aabb(glove_door)
        assert open_door_aabb is not None
        ctx.check(
            "door_swings_downward",
            open_door_aabb[1][2] < door_aabb[1][2] - 0.08,
            details=(
                f"Door top should drop significantly when opened: "
                f"closed_top={door_aabb[1][2]:.3f}, open_top={open_door_aabb[1][2]:.3f}."
            ),
        )
        ctx.check(
            "door_swings_outward",
            open_door_aabb[0][0] < door_aabb[0][0] - 0.10,
            details=(
                f"Door should move outward from dashboard: "
                f"closed_min_x={door_aabb[0][0]:.3f}, open_min_x={open_door_aabb[0][0]:.3f}."
            ),
        )
        ctx.expect_gap(
            storage_bin,
            glove_door,
            axis="x",
            min_gap=0.020,
            name="open_door_clears_storage_bin",
        )
        ctx.expect_overlap(
            glove_door,
            dashboard_housing,
            axes="y",
            min_overlap=0.250,
            elem_a=door_panel,
            name="door_remains_laterally_aligned_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
