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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _add_sleeve_channel(
    part,
    *,
    center_x: float,
    bottom_z: float,
    height: float,
    width: float,
    depth: float,
    wall: float,
    material,
    prefix: str,
) -> None:
    z_center = bottom_z + 0.5 * height
    side_offset = 0.5 * width - 0.5 * wall
    rear_y = -0.5 * depth + 0.5 * wall
    retainer_y = 0.5 * depth - 0.002

    part.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(center_x - side_offset, 0.0, z_center)),
        material=material,
        name=f"{prefix}_wall_outer",
    )
    part.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(center_x + side_offset, 0.0, z_center)),
        material=material,
        name=f"{prefix}_wall_inner",
    )
    part.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(center_x, rear_y, z_center)),
        material=material,
        name=f"{prefix}_back",
    )
    part.visual(
        Box((0.028, 0.004, height)),
        origin=Origin(xyz=(center_x, retainer_y, z_center)),
        material=material,
        name=f"{prefix}_retainer",
    )


def _add_wheel_visuals(
    part,
    *,
    mesh_name: str,
    wheel_radius: float,
    wheel_width: float,
    rim_material,
    hub_material,
    tire_material,
) -> None:
    half_width = 0.5 * wheel_width
    tire_profile = [
        (wheel_radius * 0.56, -half_width * 0.98),
        (wheel_radius * 0.80, -half_width),
        (wheel_radius * 0.95, -half_width * 0.74),
        (wheel_radius, -half_width * 0.28),
        (wheel_radius, half_width * 0.28),
        (wheel_radius * 0.95, half_width * 0.74),
        (wheel_radius * 0.80, half_width),
        (wheel_radius * 0.56, half_width * 0.98),
        (wheel_radius * 0.46, half_width * 0.46),
        (wheel_radius * 0.42, 0.0),
        (wheel_radius * 0.46, -half_width * 0.46),
        (wheel_radius * 0.56, -half_width * 0.98),
    ]
    tire_geom = LatheGeometry(tire_profile, segments=64).rotate_y(math.pi / 2.0)
    part.visual(
        mesh_from_geometry(tire_geom, mesh_name),
        material=tire_material,
        name="tire_shell",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.69, length=wheel_width * 0.80),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rim_material,
        name="rim_shell",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.27, length=wheel_width),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="hub_core",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.13, length=wheel_width * 0.94),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rim_material,
        name="bearing_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="collapsible_hand_truck")

    aluminum = model.material("aluminum", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    grip_black = model.material("grip_black", rgba=(0.12, 0.12, 0.12, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.42, 0.10, 1.06)),
        mass=6.8,
        origin=Origin(xyz=(0.0, -0.01, 0.53)),
    )

    sleeve_bottom_z = 0.18
    sleeve_height = 0.84
    sleeve_width = 0.032
    sleeve_depth = 0.022
    sleeve_wall = 0.005
    upright_x = 0.16

    _add_sleeve_channel(
        frame,
        center_x=-upright_x,
        bottom_z=sleeve_bottom_z,
        height=sleeve_height,
        width=sleeve_width,
        depth=sleeve_depth,
        wall=sleeve_wall,
        material=aluminum,
        prefix="left_sleeve",
    )
    _add_sleeve_channel(
        frame,
        center_x=upright_x,
        bottom_z=sleeve_bottom_z,
        height=sleeve_height,
        width=sleeve_width,
        depth=sleeve_depth,
        wall=sleeve_wall,
        material=aluminum,
        prefix="right_sleeve",
    )

    frame.visual(
        Box((0.32, 0.022, 0.03)),
        origin=Origin(xyz=(0.0, -0.002, 0.18)),
        material=aluminum,
        name="base_crossbar",
    )
    frame.visual(
        Box((0.29, 0.018, 0.03)),
        origin=Origin(xyz=(0.0, -0.003, 0.54)),
        material=aluminum,
        name="mid_crossbar",
    )
    frame.visual(
        Box((0.29, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, -0.003, 0.91)),
        material=aluminum,
        name="upper_crossbar",
    )
    frame.visual(
        Box((0.23, 0.012, 0.56)),
        origin=Origin(xyz=(0.0, -0.008, 0.49)),
        material=dark_aluminum,
        name="back_spine",
    )
    frame.visual(
        Box((0.07, 0.14, 0.18)),
        origin=Origin(xyz=(-0.125, -0.07, 0.09)),
        material=aluminum,
        name="left_axle_support",
    )
    frame.visual(
        Box((0.07, 0.14, 0.18)),
        origin=Origin(xyz=(0.125, -0.07, 0.09)),
        material=aluminum,
        name="right_axle_support",
    )
    frame.visual(
        Box((0.07, 0.03, 0.03)),
        origin=Origin(xyz=(-0.11, 0.015, 0.03)),
        material=aluminum,
        name="left_hinge_strap",
    )
    frame.visual(
        Box((0.07, 0.03, 0.03)),
        origin=Origin(xyz=(0.11, 0.015, 0.03)),
        material=aluminum,
        name="right_hinge_strap",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.07),
        origin=Origin(xyz=(-0.11, 0.03, 0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="left_hinge_barrel",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.07),
        origin=Origin(xyz=(0.11, 0.03, 0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="right_hinge_barrel",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.335),
        origin=Origin(xyz=(0.0, -0.13, 0.11), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="shared_axle",
    )

    telescoping_handle = model.part("telescoping_handle")
    telescoping_handle.inertial = Inertial.from_geometry(
        Box((0.36, 0.04, 0.90)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.006, 0.45)),
    )
    telescoping_handle.visual(
        Box((0.020, 0.012, 0.76)),
        origin=Origin(xyz=(-upright_x, 0.002, 0.38)),
        material=aluminum,
        name="left_slider",
    )
    telescoping_handle.visual(
        Box((0.020, 0.012, 0.76)),
        origin=Origin(xyz=(upright_x, 0.002, 0.38)),
        material=aluminum,
        name="right_slider",
    )
    telescoping_handle.visual(
        Cylinder(radius=0.015, length=0.32),
        origin=Origin(xyz=(0.0, 0.008, 0.775), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="grip_bar",
    )
    telescoping_handle.visual(
        Cylinder(radius=0.020, length=0.10),
        origin=Origin(xyz=(-0.11, 0.008, 0.775), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip_sleeve",
    )
    telescoping_handle.visual(
        Cylinder(radius=0.020, length=0.10),
        origin=Origin(xyz=(0.11, 0.008, 0.775), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip_sleeve",
    )

    toe_plate = model.part("toe_plate")
    toe_plate.inertial = Inertial.from_geometry(
        Box((0.34, 0.21, 0.05)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.105, 0.025)),
    )
    toe_outer = rounded_rect_profile(0.34, 0.205, 0.012, corner_segments=8)
    toe_slot = rounded_rect_profile(0.060, 0.110, 0.010, corner_segments=6)
    toe_holes = [
        _shift_profile(toe_slot, dx=-0.10, dy=0.0),
        _shift_profile(toe_slot, dx=0.0, dy=0.0),
        _shift_profile(toe_slot, dx=0.10, dy=0.0),
    ]
    toe_panel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(toe_outer, toe_holes, height=0.008, center=True),
        "toe_plate_panel",
    )
    toe_plate.visual(
        toe_panel_mesh,
        origin=Origin(xyz=(0.0, 0.1115, 0.004)),
        material=aluminum,
        name="toe_panel",
    )
    toe_plate.visual(
        Cylinder(radius=0.009, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="hinge_barrel",
    )
    toe_plate.visual(
        Box((0.34, 0.008, 0.03)),
        origin=Origin(xyz=(0.0, 0.218, 0.015)),
        material=aluminum,
        name="toe_lip",
    )
    toe_plate.visual(
        Box((0.022, 0.17, 0.028)),
        origin=Origin(xyz=(-0.159, 0.09, 0.018)),
        material=dark_aluminum,
        name="left_gusset",
    )
    toe_plate.visual(
        Box((0.022, 0.17, 0.028)),
        origin=Origin(xyz=(0.159, 0.09, 0.018)),
        material=dark_aluminum,
        name="right_gusset",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.055),
        mass=1.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        left_wheel,
        mesh_name="left_hand_truck_wheel_tire",
        wheel_radius=0.11,
        wheel_width=0.055,
        rim_material=aluminum,
        hub_material=dark_aluminum,
        tire_material=rubber,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.055),
        mass=1.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        right_wheel,
        mesh_name="right_hand_truck_wheel_tire",
        wheel_radius=0.11,
        wheel_width=0.055,
        rim_material=aluminum,
        hub_material=dark_aluminum,
        tire_material=rubber,
    )

    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=telescoping_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.5,
            lower=-0.10,
            upper=0.16,
        ),
    )
    model.articulation(
        "toe_plate_fold",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=toe_plate,
        origin=Origin(xyz=(0.0, 0.03, 0.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(-0.195, -0.13, 0.11)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(0.195, -0.13, 0.11)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    telescoping_handle = object_model.get_part("telescoping_handle")
    toe_plate = object_model.get_part("toe_plate")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    handle_slide = object_model.get_articulation("handle_slide")
    toe_plate_fold = object_model.get_articulation("toe_plate_fold")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        telescoping_handle,
        frame,
        elem_a="left_slider",
        elem_b="left_sleeve_retainer",
        name="left slider stays guided by sleeve retainer",
    )
    ctx.expect_contact(
        telescoping_handle,
        frame,
        elem_a="right_slider",
        elem_b="right_sleeve_retainer",
        name="right slider stays guided by sleeve retainer",
    )
    ctx.expect_contact(
        toe_plate,
        frame,
        elem_a="hinge_barrel",
        elem_b="left_hinge_barrel",
        name="toe plate hinge contacts frame barrels",
    )
    ctx.expect_contact(
        left_wheel,
        frame,
        elem_a="hub_core",
        elem_b="shared_axle",
        name="left wheel seats on shared axle",
    )
    ctx.expect_contact(
        right_wheel,
        frame,
        elem_a="hub_core",
        elem_b="shared_axle",
        name="right wheel seats on shared axle",
    )

    def check_slider_capture(prefix: str) -> bool:
        slider = ctx.part_element_world_aabb(telescoping_handle, elem=f"{prefix}_slider")
        wall_a = ctx.part_element_world_aabb(frame, elem=f"{prefix}_sleeve_wall_outer")
        wall_b = ctx.part_element_world_aabb(frame, elem=f"{prefix}_sleeve_wall_inner")
        back = ctx.part_element_world_aabb(frame, elem=f"{prefix}_sleeve_back")
        retainer = ctx.part_element_world_aabb(frame, elem=f"{prefix}_sleeve_retainer")
        assert slider is not None and wall_a is not None and wall_b is not None
        assert back is not None and retainer is not None

        x_min = min(wall_a[0][0], wall_b[0][0])
        x_max = max(wall_a[1][0], wall_b[1][0])
        y_min = back[0][1]
        y_max = retainer[1][1]
        z_overlap = min(slider[1][2], back[1][2]) - max(slider[0][2], back[0][2])

        return (
            slider[0][0] >= x_min - 1e-6
            and slider[1][0] <= x_max + 1e-6
            and slider[0][1] >= y_min - 1e-6
            and slider[1][1] <= y_max + 1e-6
            and z_overlap >= 0.50
        )

    lower = handle_slide.motion_limits.lower
    upper = handle_slide.motion_limits.upper
    assert lower is not None and upper is not None

    with ctx.pose({handle_slide: lower}):
        low_pos = ctx.part_world_position(telescoping_handle)
        assert low_pos is not None
        ctx.check(
            "collapsed handle stays captured in left sleeve",
            check_slider_capture("left"),
            details="Left telescoping rail left the sleeve envelope in the collapsed pose.",
        )
        ctx.check(
            "collapsed handle stays captured in right sleeve",
            check_slider_capture("right"),
            details="Right telescoping rail left the sleeve envelope in the collapsed pose.",
        )

    with ctx.pose({handle_slide: upper}):
        high_pos = ctx.part_world_position(telescoping_handle)
        assert high_pos is not None
        ctx.check(
            "extended handle stays captured in left sleeve",
            check_slider_capture("left"),
            details="Left telescoping rail lost sleeve engagement in the extended pose.",
        )
        ctx.check(
            "extended handle stays captured in right sleeve",
            check_slider_capture("right"),
            details="Right telescoping rail lost sleeve engagement in the extended pose.",
        )
        ctx.expect_contact(
            telescoping_handle,
            frame,
            elem_a="left_slider",
            elem_b="left_sleeve_retainer",
            name="left slider remains clipped when extended",
        )
        ctx.expect_contact(
            telescoping_handle,
            frame,
            elem_a="right_slider",
            elem_b="right_sleeve_retainer",
            name="right slider remains clipped when extended",
        )

    travel = upper - lower
    assert low_pos is not None and high_pos is not None
    ctx.check(
        "handle telescopes vertically",
        abs((high_pos[2] - low_pos[2]) - travel) < 1e-6
        and abs(high_pos[0] - low_pos[0]) < 1e-6
        and abs(high_pos[1] - low_pos[1]) < 1e-6,
        details="The prismatic handle travel is not a pure vertical slide.",
    )

    with ctx.pose({toe_plate_fold: 0.0}):
        deployed = ctx.part_world_aabb(toe_plate)
        assert deployed is not None
        ctx.check(
            "toe plate deploys forward",
            deployed[1][1] > 0.25 and deployed[0][2] >= 0.02,
            details="The toe plate does not project forward low to the ground when deployed.",
        )

    with ctx.pose({toe_plate_fold: math.pi / 2.0}):
        stowed = ctx.part_world_aabb(toe_plate)
        assert stowed is not None
        ctx.check(
            "toe plate folds up for storage",
            stowed[1][2] > 0.25 and stowed[1][1] <= 0.06,
            details="The toe plate does not rotate up against the frame for storage.",
        )
        ctx.expect_contact(
            toe_plate,
            frame,
            elem_a="hinge_barrel",
            elem_b="right_hinge_barrel",
            name="stowed toe plate remains pinned on hinge barrels",
        )

    left_origin = left_wheel_spin.origin.xyz
    right_origin = right_wheel_spin.origin.xyz
    ctx.check(
        "wheels share one axle line",
        tuple(left_wheel_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(right_wheel_spin.axis) == (1.0, 0.0, 0.0)
        and abs(left_origin[1] - right_origin[1]) < 1e-9
        and abs(left_origin[2] - right_origin[2]) < 1e-9,
        details="The wheel spin joints are not aligned on a common axle axis.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
