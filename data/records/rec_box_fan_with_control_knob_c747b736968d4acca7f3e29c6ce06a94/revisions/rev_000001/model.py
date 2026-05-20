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
    FanRotorGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


HOUSING_SIZE = 0.34
HOUSING_DEPTH = 0.14
HOUSING_CENTER_Z = HOUSING_SIZE * 0.5
INNER_APERTURE = 0.282
FRAME_THICKNESS = (HOUSING_SIZE - INNER_APERTURE) * 0.5
FRONT_Y = HOUSING_DEPTH * 0.5
BACK_Y = -HOUSING_DEPTH * 0.5
RIGHT_X = HOUSING_SIZE * 0.5
TOP_Z = HOUSING_SIZE
BLADE_RADIUS = 0.108
GRILLE_WIDTH = 0.300
GRILLE_HEIGHT = 0.300
GRILLE_DEPTH = 0.008
GRILLE_FRAME = 0.014
GRILLE_BAR_RADIUS = 0.003


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_box_fan")

    plastic = model.material("plastic", rgba=(0.86, 0.88, 0.90, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    blade_tint = model.material("blade_tint", rgba=(0.72, 0.78, 0.82, 0.92))
    knob_black = model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.42, 0.45, 0.48, 1.0))

    housing = model.part("housing")
    grille_hinge_y = FRONT_Y + 0.010
    hinge_pin_radius = 0.005
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_SIZE, HOUSING_DEPTH, HOUSING_SIZE)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_CENTER_Z)),
    )

    side_frame_x = INNER_APERTURE * 0.5 + FRAME_THICKNESS * 0.5
    top_frame_z = HOUSING_SIZE - FRAME_THICKNESS * 0.5
    bottom_frame_z = FRAME_THICKNESS * 0.5
    for x_pos, name in ((-side_frame_x, "housing_frame_left"), (side_frame_x, "housing_frame_right")):
        housing.visual(
            Box((FRAME_THICKNESS, HOUSING_DEPTH, HOUSING_SIZE)),
            origin=Origin(xyz=(x_pos, 0.0, HOUSING_CENTER_Z)),
            material=plastic,
            name=name,
        )
    for z_pos, name in ((bottom_frame_z, "housing_frame_bottom"), (top_frame_z, "housing_frame_top")):
        housing.visual(
            Box((INNER_APERTURE, HOUSING_DEPTH, FRAME_THICKNESS)),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=plastic,
            name=name,
        )

    for x_pos in (-0.145, 0.145):
        housing.visual(
            Box((0.060, 0.056, 0.018)),
            origin=Origin(xyz=(x_pos, 0.022, 0.009)),
            material=rubber,
            name=f"foot_{'left' if x_pos < 0.0 else 'right'}",
        )

    for x_pos in (-0.146, 0.146):
        housing.visual(
            Box((0.048, 0.028, 0.028)),
            origin=Origin(xyz=(x_pos, FRONT_Y - 0.010, TOP_Z - 0.010)),
            material=plastic,
            name=f"handle_boss_{'left' if x_pos < 0.0 else 'right'}",
        )

    housing.visual(
        Box((0.034, 0.086, 0.126)),
        origin=Origin(xyz=(RIGHT_X - 0.016, 0.000, HOUSING_CENTER_Z + 0.008)),
        material=plastic,
        name="control_pad",
    )
    for z_pos, name in (
        (HOUSING_CENTER_Z + 0.105, "grille_hinge_mount_upper"),
        (HOUSING_CENTER_Z - 0.105, "grille_hinge_mount_lower"),
    ):
        housing.visual(
            Box((0.016, 0.012, 0.052)),
            origin=Origin(xyz=(RIGHT_X - 0.019, FRONT_Y - 0.005, z_pos)),
            material=plastic,
            name=name,
        )

    rear_bar_y = BACK_Y + 0.010
    for x_pos in (-0.090, -0.045, 0.0, 0.045, 0.090):
        housing.visual(
            Box((0.008, 0.016, INNER_APERTURE - 0.012)),
            origin=Origin(xyz=(x_pos, rear_bar_y, HOUSING_CENTER_Z)),
            material=plastic,
            name=f"rear_vertical_bar_{int((x_pos + 0.09) * 1000):03d}",
        )
    for z_pos in (
        HOUSING_CENTER_Z - 0.090,
        HOUSING_CENTER_Z - 0.045,
        HOUSING_CENTER_Z,
        HOUSING_CENTER_Z + 0.045,
        HOUSING_CENTER_Z + 0.090,
    ):
        housing.visual(
            Box((INNER_APERTURE - 0.012, 0.016, 0.008)),
            origin=Origin(xyz=(0.0, rear_bar_y, z_pos)),
            material=plastic,
            name=f"rear_horizontal_bar_{int((z_pos - (HOUSING_CENTER_Z - 0.09)) * 1000):03d}",
        )
    for x_pos, name in ((-0.135, "rear_frame_left"), (0.135, "rear_frame_right")):
        housing.visual(
            Box((0.012, 0.016, INNER_APERTURE)),
            origin=Origin(xyz=(x_pos, rear_bar_y, HOUSING_CENTER_Z)),
            material=plastic,
            name=name,
        )
    for z_pos, name in (
        (HOUSING_CENTER_Z - 0.135, "rear_frame_bottom"),
        (HOUSING_CENTER_Z + 0.135, "rear_frame_top"),
    ):
        housing.visual(
            Box((INNER_APERTURE, 0.016, 0.012)),
            origin=Origin(xyz=(0.0, rear_bar_y, z_pos)),
            material=plastic,
            name=name,
        )

    housing.visual(
        Cylinder(radius=0.044, length=0.060),
        origin=Origin(xyz=(0.0, -0.034, HOUSING_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=motor_gray,
        name="motor_housing",
    )
    housing.visual(
        Box((INNER_APERTURE - 0.034, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.044, HOUSING_CENTER_Z)),
        material=motor_gray,
        name="motor_strut_horizontal",
    )
    housing.visual(
        Box((0.018, 0.020, INNER_APERTURE - 0.034)),
        origin=Origin(xyz=(0.0, -0.044, HOUSING_CENTER_Z)),
        material=motor_gray,
        name="motor_strut_vertical",
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=BLADE_RADIUS, length=0.036),
        mass=0.22,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    rotor = FanRotorGeometry(
        BLADE_RADIUS,
        0.032,
        5,
        thickness=0.018,
        blade_pitch_deg=28.0,
        blade_sweep_deg=18.0,
    ).rotate_x(pi / 2.0)
    blade_assembly.visual(_mesh("rotor", rotor), material=blade_tint, name="rotor")
    blade_assembly.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=motor_gray,
        name="hub",
    )
    blade_assembly.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="spinner",
    )
    blade_assembly.visual(
        Cylinder(radius=0.008, length=0.029),
        origin=Origin(xyz=(0.0, -0.0255, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=motor_gray,
        name="shaft",
    )

    front_grille = model.part("front_grille")
    front_grille.inertial = Inertial.from_geometry(
        Box((GRILLE_WIDTH, 0.012, GRILLE_HEIGHT)),
        mass=0.34,
        origin=Origin(xyz=(-GRILLE_WIDTH * 0.5, 0.0, 0.0)),
    )
    inner_grille_width = GRILLE_WIDTH - 2.0 * GRILLE_FRAME
    inner_grille_height = GRILLE_HEIGHT - 2.0 * GRILLE_FRAME
    left_frame_x = -GRILLE_WIDTH + GRILLE_FRAME * 0.5
    right_frame_x = -GRILLE_FRAME * 0.5
    frame_center_x = -GRILLE_WIDTH * 0.5
    top_frame_z = GRILLE_HEIGHT * 0.5 - GRILLE_FRAME * 0.5
    bottom_frame_z = -GRILLE_HEIGHT * 0.5 + GRILLE_FRAME * 0.5

    front_grille.visual(
        Box((GRILLE_FRAME, GRILLE_DEPTH, GRILLE_HEIGHT)),
        origin=Origin(xyz=(left_frame_x, 0.0, 0.0)),
        material=plastic,
        name="grille_left_frame",
    )
    front_grille.visual(
        Box((GRILLE_FRAME, GRILLE_DEPTH, GRILLE_HEIGHT)),
        origin=Origin(xyz=(right_frame_x, 0.0, 0.0)),
        material=plastic,
        name="grille_right_frame",
    )
    front_grille.visual(
        Box((inner_grille_width, GRILLE_DEPTH, GRILLE_FRAME)),
        origin=Origin(xyz=(frame_center_x, 0.0, top_frame_z)),
        material=plastic,
        name="grille_top_frame",
    )
    front_grille.visual(
        Box((inner_grille_width, GRILLE_DEPTH, GRILLE_FRAME)),
        origin=Origin(xyz=(frame_center_x, 0.0, bottom_frame_z)),
        material=plastic,
        name="grille_bottom_frame",
    )

    vertical_bar_count = 6
    for index in range(vertical_bar_count):
        x_pos = -GRILLE_WIDTH + GRILLE_FRAME + (index + 1) * inner_grille_width / (vertical_bar_count + 1)
        front_grille.visual(
            Cylinder(radius=GRILLE_BAR_RADIUS, length=inner_grille_height),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=plastic,
            name=f"grille_vertical_bar_{index + 1}",
        )

    horizontal_bar_count = 6
    for index in range(horizontal_bar_count):
        z_pos = -inner_grille_height * 0.5 + (index + 1) * inner_grille_height / (horizontal_bar_count + 1)
        front_grille.visual(
            Cylinder(radius=GRILLE_BAR_RADIUS, length=inner_grille_width),
            origin=Origin(xyz=(frame_center_x, 0.0, z_pos), rpy=(0.0, pi / 2.0, 0.0)),
            material=plastic,
            name=f"grille_horizontal_bar_{index + 1}",
        )

    front_grille.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(frame_center_x, 0.001, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="grille_center_badge",
    )
    hinge_barrel = LatheGeometry.from_shell_profiles(
        [(0.009, -0.020), (0.009, 0.020)],
        [(hinge_pin_radius, -0.020), (hinge_pin_radius, 0.020)],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )
    front_grille.visual(
        _mesh("front_grille_upper_hinge_barrel", hinge_barrel),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=plastic,
        name="upper_hinge_barrel",
    )
    front_grille.visual(
        _mesh("front_grille_lower_hinge_barrel", hinge_barrel),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=plastic,
        name="lower_hinge_barrel",
    )
    front_grille.visual(
        Box((0.012, 0.012, 0.050)),
        origin=Origin(xyz=(-0.296, -0.002, 0.0)),
        material=plastic,
        name="latch_tab",
    )

    carry_handle = model.part("carry_handle")
    carry_handle.inertial = Inertial.from_geometry(
        Box((0.280, 0.090, 0.032)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.046, 0.010)),
    )
    handle_bar = tube_from_spline_points(
        [
            (-0.132, 0.000, 0.000),
            (-0.116, -0.016, 0.004),
            (-0.084, -0.044, 0.008),
            (-0.032, -0.066, 0.011),
            (0.032, -0.066, 0.011),
            (0.084, -0.044, 0.008),
            (0.116, -0.016, 0.004),
            (0.132, 0.000, 0.000),
        ],
        radius=0.007,
        samples_per_segment=16,
        radial_segments=18,
    )
    carry_handle.visual(_mesh("carry_handle_bar", handle_bar), material=plastic, name="handle_bar")
    carry_handle.visual(
        Cylinder(radius=0.0105, length=0.092),
        origin=Origin(xyz=(0.0, -0.062, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="grip_sleeve",
    )

    control_knob = model.part("control_knob")
    control_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.024),
        mass=0.05,
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    control_knob.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
        name="knob_shaft",
    )
    control_knob.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    control_knob.visual(
        Box((0.004, 0.004, 0.012)),
        origin=Origin(xyz=(0.027, 0.0, 0.012)),
        material=plastic,
        name="knob_indicator",
    )

    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.036, HOUSING_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=30.0),
    )
    model.articulation(
        "front_grille_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_grille,
        origin=Origin(xyz=(0.150, grille_hinge_y, HOUSING_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=carry_handle,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.010, TOP_Z + 0.008)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.2, lower=0.0, upper=1.15),
    )
    model.articulation(
        "control_knob_turn",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=control_knob,
        origin=Origin(xyz=(RIGHT_X + 0.001, 0.0, HOUSING_CENTER_Z + 0.032)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.1, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    blade_assembly = object_model.get_part("blade_assembly")
    front_grille = object_model.get_part("front_grille")
    carry_handle = object_model.get_part("carry_handle")
    control_knob = object_model.get_part("control_knob")

    blade_spin = object_model.get_articulation("blade_spin")
    front_grille_hinge = object_model.get_articulation("front_grille_hinge")
    handle_hinge = object_model.get_articulation("handle_hinge")
    control_knob_turn = object_model.get_articulation("control_knob_turn")

    housing_shell_aabb = ctx.part_element_world_aabb(housing, elem="housing_shell")
    housing_frame_aabbs = [
        ctx.part_element_world_aabb(housing, elem="housing_frame_left"),
        ctx.part_element_world_aabb(housing, elem="housing_frame_right"),
        ctx.part_element_world_aabb(housing, elem="housing_frame_bottom"),
        ctx.part_element_world_aabb(housing, elem="housing_frame_top"),
    ]
    ctx.check(
        "housing keeps a true pass-through frame opening",
        housing_shell_aabb is None and all(aabb is not None for aabb in housing_frame_aabbs),
        details=(
            f"housing_shell={housing_shell_aabb}, "
            f"housing_frame_aabbs={housing_frame_aabbs}"
        ),
    )
    grille_panel_aabb = ctx.part_element_world_aabb(front_grille, elem="grille_panel")
    grille_structure_aabbs = [
        ctx.part_element_world_aabb(front_grille, elem="grille_left_frame"),
        ctx.part_element_world_aabb(front_grille, elem="grille_right_frame"),
        ctx.part_element_world_aabb(front_grille, elem="grille_top_frame"),
        ctx.part_element_world_aabb(front_grille, elem="grille_bottom_frame"),
        ctx.part_element_world_aabb(front_grille, elem="grille_vertical_bar_3"),
        ctx.part_element_world_aabb(front_grille, elem="grille_horizontal_bar_3"),
        ctx.part_element_world_aabb(front_grille, elem="grille_center_badge"),
    ]
    ctx.check(
        "front grille uses explicit framed-bar construction",
        grille_panel_aabb is None and all(aabb is not None for aabb in grille_structure_aabbs),
        details=(
            f"grille_panel={grille_panel_aabb}, "
            f"grille_structure_aabbs={grille_structure_aabbs}"
        ),
    )
    ctx.expect_gap(
        front_grille,
        housing,
        axis="y",
        positive_elem="grille_center_badge",
        max_gap=0.014,
        max_penetration=0.0,
        name="front grille sits just ahead of the housing",
    )
    ctx.expect_overlap(
        front_grille,
        housing,
        axes="xz",
        min_overlap=0.27,
        name="front grille covers the square front opening",
    )
    ctx.expect_gap(
        front_grille,
        blade_assembly,
        axis="y",
        positive_elem="grille_center_badge",
        negative_elem="rotor",
        min_gap=0.020,
        name="front grille clears the rotor at rest",
    )
    ctx.expect_within(
        blade_assembly,
        housing,
        axes="xz",
        inner_elem="rotor",
        margin=0.020,
        name="rotor stays within the housing aperture",
    )
    ctx.expect_gap(
        control_knob,
        housing,
        axis="x",
        positive_elem="knob_body",
        min_gap=0.0,
        max_gap=0.010,
        name="control knob mounts on the right face",
    )
    ctx.expect_gap(
        carry_handle,
        housing,
        axis="z",
        positive_elem="grip_sleeve",
        min_gap=0.002,
        max_gap=0.040,
        name="folded handle rests just above the top shell",
    )

    rest_grip_aabb = ctx.part_element_world_aabb(carry_handle, elem="grip_sleeve")
    with ctx.pose({handle_hinge: 1.05, blade_spin: pi / 3.0, control_knob_turn: pi / 2.0}):
        ctx.expect_gap(
            carry_handle,
            housing,
            axis="z",
            positive_elem="grip_sleeve",
            min_gap=0.050,
            name="raised handle lifts into a carry position",
        )
        raised_grip_aabb = ctx.part_element_world_aabb(carry_handle, elem="grip_sleeve")
    ctx.check(
        "handle rotates upward from the folded position",
        rest_grip_aabb is not None
        and raised_grip_aabb is not None
        and raised_grip_aabb[0][2] > rest_grip_aabb[0][2] + 0.040,
        details=f"rest={rest_grip_aabb}, raised={raised_grip_aabb}",
    )

    rest_latch_aabb = ctx.part_element_world_aabb(front_grille, elem="latch_tab")
    with ctx.pose({front_grille_hinge: 1.10}):
        ctx.expect_gap(
            front_grille,
            housing,
            axis="y",
            positive_elem="latch_tab",
            min_gap=0.040,
            name="front grille swings forward to open",
        )
        open_latch_aabb = ctx.part_element_world_aabb(front_grille, elem="latch_tab")
    ctx.check(
        "front grille opens outward on its side hinge",
        rest_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[0][1] > rest_latch_aabb[0][1] + 0.040,
        details=f"rest={rest_latch_aabb}, open={open_latch_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
