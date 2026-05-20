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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_glove_compartment")

    outer_width = 0.44
    outer_height = 0.28
    opening_width = 0.34
    opening_height = 0.18
    bezel_thickness = 0.028
    bin_depth = 0.185
    wall_thickness = 0.006
    rear_width = 0.292
    rear_height = 0.136

    door_width = 0.332
    door_height = 0.172
    door_thickness = 0.018
    hinge_axis_x = -opening_width * 0.5
    hinge_axis_y = 0.006

    def rounded_panel_mesh(
        width: float,
        height: float,
        radius: float,
        thickness: float,
        name: str,
    ):
        geometry = ExtrudeGeometry(
            rounded_rect_profile(width, height, radius, corner_segments=10),
            thickness,
            cap=True,
            center=True,
            closed=True,
        )
        geometry.rotate_x(math.pi / 2.0)
        return mesh_from_geometry(geometry, name)

    def rounded_ring_mesh(
        outer_w: float,
        outer_h: float,
        inner_w: float,
        inner_h: float,
        thickness: float,
        name: str,
    ):
        geometry = ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_w, outer_h, 0.028, corner_segments=10),
            [rounded_rect_profile(inner_w, inner_h, 0.016, corner_segments=10)],
            thickness,
            cap=True,
            center=True,
            closed=True,
        )
        geometry.rotate_x(math.pi / 2.0)
        return mesh_from_geometry(geometry, name)

    model.material("dash_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("bin_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("door_graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    model.material("door_insert", rgba=(0.25, 0.26, 0.28, 1.0))
    model.material("hardware_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    model.material("latch_black", rgba=(0.11, 0.11, 0.12, 1.0))

    housing = model.part("dashboard_bin")
    housing.visual(
        rounded_ring_mesh(
            outer_width,
            outer_height,
            opening_width,
            opening_height,
            bezel_thickness,
            "dashboard_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material="dash_charcoal",
        name="bezel",
    )

    wall_run = math.sqrt((bin_depth - bezel_thickness) ** 2 + ((opening_width - rear_width) * 0.5) ** 2)
    wall_pitch = math.atan2((opening_height - rear_height) * 0.5, bin_depth - bezel_thickness)
    wall_yaw = math.atan2((opening_width - rear_width) * 0.5, bin_depth - bezel_thickness)

    housing.visual(
        Box((wall_thickness, wall_run, 0.154)),
        origin=Origin(xyz=(-0.161, -0.104, 0.0), rpy=(0.0, 0.0, wall_yaw)),
        material="bin_black",
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, wall_run, 0.154)),
        origin=Origin(xyz=(0.161, -0.104, 0.0), rpy=(0.0, 0.0, -wall_yaw)),
        material="bin_black",
        name="right_wall",
    )
    housing.visual(
        Box((0.316, wall_run, wall_thickness)),
        origin=Origin(xyz=(0.0, -0.104, 0.082), rpy=(wall_pitch, 0.0, 0.0)),
        material="bin_black",
        name="top_wall",
    )
    housing.visual(
        Box((0.316, wall_run, wall_thickness)),
        origin=Origin(xyz=(0.0, -0.104, -0.082), rpy=(-wall_pitch, 0.0, 0.0)),
        material="bin_black",
        name="bottom_wall",
    )
    housing.visual(
        Box((0.304, 0.008, 0.148)),
        origin=Origin(xyz=(0.0, -0.192, 0.0)),
        material="bin_black",
        name="back_wall",
    )
    housing.visual(
        rounded_ring_mesh(
            opening_width,
            opening_height,
            0.290,
            0.130,
            0.016,
            "front_jamb_ring",
        ),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material="bin_black",
        name="front_jamb_ring",
    )
    housing.visual(
        rounded_ring_mesh(
            0.304,
            0.148,
            0.244,
            0.096,
            0.014,
            "rear_stiffener_ring",
        ),
        origin=Origin(xyz=(0.0, -0.184, 0.0)),
        material="bin_black",
        name="rear_stiffener_ring",
    )
    housing.visual(
        Box((0.012, 0.016, 0.050)),
        origin=Origin(xyz=(0.175, -0.007, 0.0)),
        material="hardware_steel",
        name="latch_keeper",
    )

    frame_leaf_size = (0.028, 0.004, 0.036)
    housing.visual(
        Box(frame_leaf_size),
        origin=Origin(xyz=(hinge_axis_x - 0.014, 0.009, 0.058)),
        material="hardware_steel",
        name="upper_frame_leaf",
    )
    housing.visual(
        Box(frame_leaf_size),
        origin=Origin(xyz=(hinge_axis_x - 0.014, 0.009, -0.058)),
        material="hardware_steel",
        name="lower_frame_leaf",
    )
    housing.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.069)),
        material="hardware_steel",
        name="upper_frame_knuckle",
    )
    housing.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, -0.069)),
        material="hardware_steel",
        name="lower_frame_knuckle",
    )
    housing.inertial = Inertial.from_geometry(
        Box((outer_width, bin_depth + 0.020, outer_height)),
        mass=4.8,
        origin=Origin(xyz=(0.0, -0.090, 0.0)),
    )

    door = model.part("door_panel")
    door.visual(
        rounded_panel_mesh(door_width, door_height, 0.012, door_thickness, "glovebox_door"),
        origin=Origin(xyz=(0.170, 0.003, 0.0)),
        material="door_graphite",
        name="door_shell",
    )
    door.visual(
        rounded_panel_mesh(0.236, 0.094, 0.010, 0.004, "glovebox_door_insert"),
        origin=Origin(xyz=(0.188, 0.010, 0.0)),
        material="door_insert",
        name="center_insert",
    )
    door.visual(
        Box((0.272, 0.004, 0.124)),
        origin=Origin(xyz=(0.188, -0.008, 0.0)),
        material="bin_black",
        name="inner_liner",
    )

    door_leaf_size = (0.018, 0.004, 0.034)
    door.visual(
        Box(door_leaf_size),
        origin=Origin(xyz=(0.009, 0.009, 0.046)),
        material="hardware_steel",
        name="upper_door_leaf",
    )
    door.visual(
        Box(door_leaf_size),
        origin=Origin(xyz=(0.009, 0.009, -0.046)),
        material="hardware_steel",
        name="lower_door_leaf",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.023),
        origin=Origin(xyz=(0.0, hinge_axis_y, 0.0465)),
        material="hardware_steel",
        name="upper_door_knuckle",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.023),
        origin=Origin(xyz=(0.0, hinge_axis_y, -0.0465)),
        material="hardware_steel",
        name="lower_door_knuckle",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=1.2,
        origin=Origin(xyz=(0.170, 0.003, 0.0)),
    )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    latch = model.part("quarter_turn_latch")
    latch.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hardware_steel",
        name="escutcheon",
    )
    latch.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hardware_steel",
        name="hub",
    )
    latch.visual(
        Box((0.112, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.013, 0.0)),
        material="latch_black",
        name="turn_bar",
    )
    latch.visual(
        Box((0.030, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        material="latch_black",
        name="grip_block",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.112, 0.024, 0.040)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
    )

    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.170, 0.012, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=3.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("dashboard_bin")
    door = object_model.get_part("door_panel")
    latch = object_model.get_part("quarter_turn_latch")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_turn = object_model.get_articulation("latch_turn")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "door hinge axis is vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {door_hinge.axis}",
    )
    ctx.check(
        "door opens to realistic angle",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper >= math.radians(100.0),
        details=f"door upper limit was {None if door_hinge.motion_limits is None else door_hinge.motion_limits.upper}",
    )
    ctx.check(
        "latch rotates on door-normal axis",
        tuple(latch_turn.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {latch_turn.axis}",
    )
    ctx.check(
        "latch is quarter turn",
        latch_turn.motion_limits is not None
        and latch_turn.motion_limits.upper is not None
        and abs(latch_turn.motion_limits.upper - (math.pi / 2.0)) < 1e-6,
        details=f"latch upper limit was {None if latch_turn.motion_limits is None else latch_turn.motion_limits.upper}",
    )

    ctx.expect_contact(
        door,
        housing,
        elem_a="upper_door_knuckle",
        elem_b="upper_frame_knuckle",
        name="upper hinge knuckles contact",
    )
    ctx.expect_contact(
        door,
        housing,
        elem_a="lower_door_knuckle",
        elem_b="lower_frame_knuckle",
        name="lower hinge knuckles contact",
    )
    ctx.expect_gap(
        latch,
        door,
        axis="y",
        positive_elem="escutcheon",
        negative_elem="door_shell",
        max_gap=0.0005,
        max_penetration=0.0,
        name="latch seats on door face",
    )
    ctx.expect_within(
        latch,
        door,
        axes="xz",
        margin=0.0,
        name="latch stays within door face footprint",
    )

    door_rest_aabb = ctx.part_world_aabb(door)
    rest_bar_aabb = ctx.part_element_world_aabb(latch, elem="turn_bar")
    assert door_rest_aabb is not None
    assert rest_bar_aabb is not None

    with ctx.pose({door_hinge: math.radians(85.0)}):
        door_open_aabb = ctx.part_world_aabb(door)
        assert door_open_aabb is not None
        ctx.expect_contact(
            door,
            housing,
            elem_a="upper_door_knuckle",
            elem_b="upper_frame_knuckle",
            name="upper hinge stays engaged when open",
        )
        ctx.expect_contact(
            door,
            housing,
            elem_a="lower_door_knuckle",
            elem_b="lower_frame_knuckle",
            name="lower hinge stays engaged when open",
        )
        ctx.check(
            "door swings outward from dashboard",
            door_open_aabb[1][1] > door_rest_aabb[1][1] + 0.12,
            details=f"rest max y {door_rest_aabb[1][1]:.4f}, open max y {door_open_aabb[1][1]:.4f}",
        )

    with ctx.pose({latch_turn: math.pi / 2.0}):
        turned_bar_aabb = ctx.part_element_world_aabb(latch, elem="turn_bar")
        assert turned_bar_aabb is not None
        rest_dx = rest_bar_aabb[1][0] - rest_bar_aabb[0][0]
        rest_dz = rest_bar_aabb[1][2] - rest_bar_aabb[0][2]
        turned_dx = turned_bar_aabb[1][0] - turned_bar_aabb[0][0]
        turned_dz = turned_bar_aabb[1][2] - turned_bar_aabb[0][2]
        ctx.check(
            "latch handle visibly quarter turns",
            rest_dx > rest_dz and turned_dz > turned_dx,
            details=(
                f"rest extents (dx={rest_dx:.4f}, dz={rest_dz:.4f}), "
                f"turned extents (dx={turned_dx:.4f}, dz={turned_dz:.4f})"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
