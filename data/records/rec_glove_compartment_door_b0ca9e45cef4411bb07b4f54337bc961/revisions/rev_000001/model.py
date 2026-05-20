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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
    name: str,
):
    geometry = ExtrudeGeometry(
        rounded_rect_profile(width, height, corner_radius),
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geometry.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geometry, name)


def _rounded_frame_mesh(
    *,
    outer_width: float,
    outer_height: float,
    outer_radius: float,
    opening_width: float,
    opening_height: float,
    opening_radius: float,
    thickness: float,
    name: str,
):
    geometry = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_height, outer_radius),
        [rounded_rect_profile(opening_width, opening_height, opening_radius)],
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geometry.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_vehicle_glove_compartment")

    dashboard_plastic = model.material(
        "dashboard_plastic",
        rgba=(0.20, 0.21, 0.22, 1.0),
    )
    liner_plastic = model.material(
        "liner_plastic",
        rgba=(0.12, 0.13, 0.14, 1.0),
    )
    door_plastic = model.material(
        "door_plastic",
        rgba=(0.16, 0.17, 0.18, 1.0),
    )
    hinge_metal = model.material(
        "hinge_metal",
        rgba=(0.70, 0.72, 0.74, 1.0),
    )
    latch_black = model.material(
        "latch_black",
        rgba=(0.08, 0.08, 0.09, 1.0),
    )
    striker_metal = model.material(
        "striker_metal",
        rgba=(0.62, 0.64, 0.66, 1.0),
    )

    opening_width = 0.340
    opening_height = 0.160
    bin_depth = 0.220
    liner_wall = 0.006
    bezel_width = 0.430
    bezel_height = 0.240
    bezel_thickness = 0.014
    door_width = 0.336
    door_height = 0.156
    door_thickness = 0.012
    door_panel_offset_x = 0.008
    door_center_y = bezel_thickness * 0.5 + door_thickness * 0.5
    hinge_axis_x = -opening_width * 0.5 + 0.002

    dashboard_bin = model.part("dashboard_bin")
    dashboard_bin.visual(
        _rounded_frame_mesh(
            outer_width=bezel_width,
            outer_height=bezel_height,
            outer_radius=0.020,
            opening_width=opening_width,
            opening_height=opening_height,
            opening_radius=0.012,
            thickness=bezel_thickness,
            name="glovebox_bezel",
        ),
        material=dashboard_plastic,
        name="dashboard_bezel",
    )
    dashboard_bin.visual(
        Box((opening_width - 2.0 * liner_wall, liner_wall, opening_height - 2.0 * liner_wall)),
        origin=Origin(xyz=(0.0, -bin_depth + liner_wall * 0.5, 0.0)),
        material=liner_plastic,
        name="back_wall",
    )
    dashboard_bin.visual(
        Box((liner_wall, bin_depth, opening_height)),
        origin=Origin(xyz=(-opening_width * 0.5 + liner_wall * 0.5, -bin_depth * 0.5, 0.0)),
        material=liner_plastic,
        name="left_wall",
    )
    dashboard_bin.visual(
        Box((liner_wall, bin_depth, opening_height)),
        origin=Origin(xyz=(opening_width * 0.5 - liner_wall * 0.5, -bin_depth * 0.5, 0.0)),
        material=liner_plastic,
        name="right_wall",
    )
    dashboard_bin.visual(
        Box((opening_width - 2.0 * liner_wall, bin_depth, liner_wall)),
        origin=Origin(xyz=(0.0, -bin_depth * 0.5, opening_height * 0.5 - liner_wall * 0.5)),
        material=liner_plastic,
        name="top_wall",
    )
    dashboard_bin.visual(
        Box((opening_width - 2.0 * liner_wall, bin_depth, liner_wall)),
        origin=Origin(xyz=(0.0, -bin_depth * 0.5, -opening_height * 0.5 + liner_wall * 0.5)),
        material=liner_plastic,
        name="bottom_wall",
    )
    dashboard_bin.visual(
        Box((0.016, 0.010, 0.042)),
        origin=Origin(xyz=(hinge_axis_x - 0.012, bezel_thickness * 0.5 + 0.001, 0.058)),
        material=hinge_metal,
        name="body_hinge_leaf_upper",
    )
    dashboard_bin.visual(
        Box((0.016, 0.010, 0.042)),
        origin=Origin(xyz=(hinge_axis_x - 0.012, bezel_thickness * 0.5 + 0.001, -0.020)),
        material=hinge_metal,
        name="body_hinge_leaf_lower",
    )
    dashboard_bin.visual(
        Cylinder(radius=0.0055, length=0.036),
        origin=Origin(xyz=(hinge_axis_x, door_center_y, 0.060)),
        material=hinge_metal,
        name="body_hinge_knuckle_upper",
    )
    dashboard_bin.visual(
        Cylinder(radius=0.0055, length=0.036),
        origin=Origin(xyz=(hinge_axis_x, door_center_y, -0.020)),
        material=hinge_metal,
        name="body_hinge_knuckle_lower",
    )
    dashboard_bin.visual(
        Box((0.008, 0.016, 0.040)),
        origin=Origin(xyz=(opening_width * 0.5 - 0.010, -0.004, 0.0)),
        material=striker_metal,
        name="striker_plate",
    )
    dashboard_bin.inertial = Inertial.from_geometry(
        Box((bezel_width, bin_depth, bezel_height)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -bin_depth * 0.5, 0.0)),
    )

    door = model.part("door")
    door.visual(
        _rounded_panel_mesh(
            width=door_width,
            height=door_height,
            thickness=door_thickness,
            corner_radius=0.011,
            name="glovebox_door_panel",
        ),
        origin=Origin(xyz=(door_panel_offset_x + door_width * 0.5, 0.0, 0.0)),
        material=door_plastic,
        name="door_panel",
    )
    door.visual(
        Box((0.016, 0.006, 0.116)),
        origin=Origin(xyz=(door_panel_offset_x + 0.030, -0.009, 0.0)),
        material=liner_plastic,
        name="inner_stile_hinge_side",
    )
    door.visual(
        Box((0.016, 0.006, 0.116)),
        origin=Origin(xyz=(door_panel_offset_x + door_width - 0.020, -0.009, 0.0)),
        material=liner_plastic,
        name="inner_stile_latch_side",
    )
    door.visual(
        Box((door_width - 0.050, 0.006, 0.016)),
        origin=Origin(xyz=(door_panel_offset_x + door_width * 0.5, -0.009, 0.052)),
        material=liner_plastic,
        name="inner_rail_upper",
    )
    door.visual(
        Box((door_width - 0.050, 0.006, 0.016)),
        origin=Origin(xyz=(door_panel_offset_x + door_width * 0.5, -0.009, -0.052)),
        material=liner_plastic,
        name="inner_rail_lower",
    )
    door.visual(
        Box((0.016, 0.010, 0.048)),
        origin=Origin(xyz=(0.010, 0.003, 0.020)),
        material=hinge_metal,
        name="door_hinge_leaf_upper",
    )
    door.visual(
        Box((0.016, 0.010, 0.044)),
        origin=Origin(xyz=(0.010, 0.003, -0.058)),
        material=hinge_metal,
        name="door_hinge_leaf_lower",
    )
    door.visual(
        Cylinder(radius=0.0055, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=hinge_metal,
        name="door_hinge_knuckle_upper",
    )
    door.visual(
        Cylinder(radius=0.0055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=hinge_metal,
        name="door_hinge_knuckle_lower",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.028, door_height)),
        mass=0.72,
        origin=Origin(xyz=(door_panel_offset_x + door_width * 0.5, 0.0, 0.0)),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_black,
        name="latch_boss",
    )
    latch.visual(
        Box((0.034, 0.006, 0.010)),
        origin=Origin(xyz=(0.010, 0.007, 0.0)),
        material=latch_black,
        name="thumb_turn",
    )
    latch.visual(
        Box((0.010, 0.008, 0.016)),
        origin=Origin(xyz=(-0.009, 0.006, 0.0)),
        material=latch_black,
        name="thumb_tab",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.042, 0.014, 0.020)),
        mass=0.08,
        origin=Origin(xyz=(0.006, 0.006, 0.0)),
    )

    model.articulation(
        "bin_to_door",
        ArticulationType.REVOLUTE,
        parent=dashboard_bin,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, door_center_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(door_panel_offset_x + door_width - 0.030, door_thickness * 0.5, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-0.80,
            upper=0.80,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dashboard_bin = object_model.get_part("dashboard_bin")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("bin_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "parts_present",
        all(part is not None for part in (dashboard_bin, door, latch)),
        "Dashboard bin, door, and latch must all exist.",
    )
    ctx.check(
        "door_hinge_axis_is_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical hinge axis, got {door_hinge.axis!r}.",
    )
    ctx.check(
        "latch_axis_is_front_to_back",
        tuple(latch_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected front-to-back latch axis, got {latch_joint.axis!r}.",
    )
    ctx.expect_gap(
        door,
        dashboard_bin,
        axis="y",
        positive_elem="door_panel",
        negative_elem="dashboard_bezel",
        max_gap=0.001,
        max_penetration=0.0,
        name="door_sits_flush_to_dashboard_bezel",
    )
    ctx.expect_overlap(
        door,
        dashboard_bin,
        axes="xz",
        elem_a="door_panel",
        elem_b="dashboard_bezel",
        min_overlap=0.150,
        name="door_covers_glovebox_opening",
    )
    ctx.expect_contact(
        latch,
        door,
        elem_a="latch_boss",
        elem_b="door_panel",
        name="latch_mounts_to_door_face",
    )

    door_rest_aabb = ctx.part_world_aabb(door)
    latch_rest_aabb = ctx.part_world_aabb(latch)
    assert door_rest_aabb is not None
    assert latch_rest_aabb is not None

    with ctx.pose({door_hinge: 1.15}):
        door_open_aabb = ctx.part_world_aabb(door)
        assert door_open_aabb is not None
        ctx.check(
            "door_swings_outward",
            door_open_aabb[1][1] > door_rest_aabb[1][1] + 0.12,
            "Door does not move outward from the dashboard when opened.",
        )

    with ctx.pose({latch_joint: 0.70}):
        latch_turn_aabb = ctx.part_world_aabb(latch)
        assert latch_turn_aabb is not None
        rest_height = latch_rest_aabb[1][2] - latch_rest_aabb[0][2]
        turned_height = latch_turn_aabb[1][2] - latch_turn_aabb[0][2]
        ctx.check(
            "latch_rotates_about_front_to_back_axis",
            turned_height > rest_height + 0.010,
            "Latch thumb-turn does not change orientation when rotated.",
        )
        ctx.expect_contact(
            latch,
            door,
            elem_a="latch_boss",
            elem_b="door_panel",
            name="latch_remains_seated_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
