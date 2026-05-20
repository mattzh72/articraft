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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _frame_mesh(
    *,
    width: float,
    height: float,
    corner_radius: float,
    opening_width: float,
    opening_height: float,
    opening_radius: float,
    depth: float,
    name: str,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(width, height, corner_radius),
        [rounded_rect_profile(opening_width, opening_height, opening_radius)],
        depth,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _pad_mesh(*, width: float, height: float, radius: float, depth: float, name: str):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius),
        depth,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _tube_shell_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
):
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dorm_room_microwave")

    housing_white = model.material("housing_white", rgba=(0.93, 0.93, 0.90, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.11, 0.12, 0.14, 1.0))
    cavity_enamel = model.material("cavity_enamel", rgba=(0.79, 0.81, 0.83, 1.0))
    smoky_glass = model.material("smoky_glass", rgba=(0.12, 0.16, 0.18, 0.35))
    button_gray = model.material("button_gray", rgba=(0.82, 0.84, 0.85, 1.0))
    dial_black = model.material("dial_black", rgba=(0.13, 0.14, 0.15, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.62, 0.65, 0.68, 1.0))
    glass_plate = model.material("glass_plate", rgba=(0.82, 0.89, 0.95, 0.50))

    housing_width = 0.50
    housing_depth = 0.40
    housing_height = 0.32
    shell_thickness = 0.018

    front_band_depth = 0.016
    front_band_y = -0.192
    front_face_y = front_band_y - front_band_depth / 2.0

    door_zone_x = -0.058
    front_zone_z = 0.163
    door_frame_outer_w = 0.362
    door_frame_outer_h = 0.286
    door_frame_open_w = 0.294
    door_frame_open_h = 0.214

    control_x = 0.184
    control_frame_outer_w = 0.110
    control_frame_outer_h = 0.286
    control_frame_open_w = 0.090
    control_frame_open_h = 0.244

    cavity_x = door_zone_x
    cavity_y = -0.002
    cavity_z = 0.170
    cavity_outer_w = 0.304
    cavity_outer_h = 0.206
    cavity_outer_d = 0.338
    cavity_wall = 0.004

    hinge_x = -0.236
    hinge_y = -0.211
    hinge_z = front_zone_z
    hinge_sleeve_inner = 0.0068
    hinge_sleeve_outer = 0.010
    hinge_knuckle_length = 0.062
    hinge_center_knuckle_length = 0.104
    hinge_knuckle_offset = 0.087

    door_width = 0.344
    door_height = 0.282
    door_thickness = 0.022
    door_panel_width = 0.324
    door_frame_x = 0.182
    window_width = 0.252
    window_height = 0.168

    button_mesh = _pad_mesh(
        width=0.066,
        height=0.022,
        radius=0.005,
        depth=0.002,
        name="membrane_button",
    )
    button_ring_mesh = _frame_mesh(
        width=0.074,
        height=0.028,
        corner_radius=0.006,
        opening_width=0.058,
        opening_height=0.014,
        opening_radius=0.003,
        depth=0.016,
        name="membrane_button_ring",
    )
    door_frame_mesh = _frame_mesh(
        width=door_panel_width,
        height=door_height,
        corner_radius=0.018,
        opening_width=window_width,
        opening_height=window_height,
        opening_radius=0.012,
        depth=door_thickness,
        name="microwave_door_frame",
    )
    door_inner_frame_mesh = _frame_mesh(
        width=0.292,
        height=0.238,
        corner_radius=0.014,
        opening_width=0.230,
        opening_height=0.148,
        opening_radius=0.010,
        depth=0.010,
        name="microwave_door_inner_frame",
    )
    housing_front_frame_mesh = _frame_mesh(
        width=door_frame_outer_w,
        height=door_frame_outer_h,
        corner_radius=0.014,
        opening_width=door_frame_open_w,
        opening_height=door_frame_open_h,
        opening_radius=0.010,
        depth=front_band_depth,
        name="microwave_front_frame",
    )
    control_frame_mesh = _frame_mesh(
        width=control_frame_outer_w,
        height=control_frame_outer_h,
        corner_radius=0.010,
        opening_width=control_frame_open_w,
        opening_height=control_frame_open_h,
        opening_radius=0.008,
        depth=front_band_depth,
        name="microwave_control_frame",
    )
    cavity_flange_mesh = _frame_mesh(
        width=0.304,
        height=0.224,
        corner_radius=0.008,
        opening_width=0.286,
        opening_height=0.206,
        opening_radius=0.006,
        depth=0.014,
        name="microwave_cavity_flange",
    )
    dial_bezel_mesh = _tube_shell_mesh(
        outer_radius=0.028,
        inner_radius=0.019,
        length=0.016,
        name="dial_bezel_ring",
    )
    door_sleeve_mesh = _tube_shell_mesh(
        outer_radius=hinge_sleeve_outer,
        inner_radius=hinge_sleeve_inner,
        length=0.052,
        name="door_hinge_sleeve",
    )

    housing = model.part("housing")
    housing.visual(
        Box((housing_width, housing_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, shell_thickness / 2.0)),
        material=housing_white,
        name="bottom_shell",
    )
    housing.visual(
        Box((housing_width, housing_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, housing_height - shell_thickness / 2.0)),
        material=housing_white,
        name="top_shell",
    )
    housing.visual(
        Box((shell_thickness, housing_depth, housing_height - shell_thickness * 2.0)),
        origin=Origin(
            xyz=(-housing_width / 2.0 + shell_thickness / 2.0, 0.0, housing_height / 2.0)
        ),
        material=housing_white,
        name="left_shell",
    )
    housing.visual(
        Box((shell_thickness, housing_depth, housing_height - shell_thickness * 2.0)),
        origin=Origin(
            xyz=(housing_width / 2.0 - shell_thickness / 2.0, 0.0, housing_height / 2.0)
        ),
        material=housing_white,
        name="right_shell",
    )
    housing.visual(
        Box((housing_width - shell_thickness * 2.0, shell_thickness, housing_height - shell_thickness * 2.0)),
        origin=Origin(
            xyz=(0.0, housing_depth / 2.0 - shell_thickness / 2.0, housing_height / 2.0)
        ),
        material=housing_white,
        name="back_shell",
    )
    housing.visual(
        Box((housing_width, front_band_depth, 0.020)),
        origin=Origin(xyz=(0.0, front_band_y, 0.030)),
        material=housing_white,
        name="front_bottom_rail",
    )
    housing.visual(
        Box((housing_width, front_band_depth, 0.018)),
        origin=Origin(xyz=(0.0, front_band_y, 0.311)),
        material=housing_white,
        name="front_top_rail",
    )
    housing.visual(
        Box((0.014, front_band_depth, 0.262)),
        origin=Origin(xyz=(0.125, front_band_y, front_zone_z)),
        material=housing_white,
        name="front_divider",
    )
    housing.visual(
        housing_front_frame_mesh,
        origin=Origin(xyz=(door_zone_x, front_band_y, front_zone_z)),
        material=housing_white,
        name="front_frame",
    )
    housing.visual(
        control_frame_mesh,
        origin=Origin(xyz=(control_x, front_band_y, front_zone_z)),
        material=housing_white,
        name="control_frame",
    )
    housing.visual(
        Box((0.092, 0.010, 0.246)),
        origin=Origin(xyz=(control_x, -0.183, front_zone_z)),
        material=trim_dark,
        name="control_backplate",
    )

    button_zs = (0.246, 0.213, 0.180, 0.147, 0.114)
    for index, z_pos in enumerate(button_zs, start=1):
        housing.visual(
            button_ring_mesh,
            origin=Origin(xyz=(control_x, front_band_y, z_pos)),
            material=trim_dark,
            name=f"button_ring_{index}",
        )

    housing.visual(
        dial_bezel_mesh,
        origin=Origin(xyz=(control_x, front_band_y, 0.056), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="dial_bezel",
    )

    housing.visual(
        _tube_shell_mesh(
            outer_radius=hinge_sleeve_outer,
            inner_radius=hinge_sleeve_inner,
            length=hinge_knuckle_length,
            name="housing_hinge_knuckle_top",
        ),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z + hinge_knuckle_offset)),
        material=metal_gray,
        name="hinge_knuckle_top",
    )
    housing.visual(
        _tube_shell_mesh(
            outer_radius=hinge_sleeve_outer,
            inner_radius=hinge_sleeve_inner,
            length=hinge_knuckle_length,
            name="housing_hinge_knuckle_bottom",
        ),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z - hinge_knuckle_offset)),
        material=metal_gray,
        name="hinge_knuckle_bottom",
    )
    housing.visual(
        Box((0.010, 0.020, 0.044)),
        origin=Origin(xyz=(hinge_x - 0.011, hinge_y + 0.009, hinge_z + hinge_knuckle_offset)),
        material=housing_white,
        name="hinge_tab_top",
    )
    housing.visual(
        Box((0.010, 0.020, 0.044)),
        origin=Origin(xyz=(hinge_x - 0.011, hinge_y + 0.009, hinge_z - hinge_knuckle_offset)),
        material=housing_white,
        name="hinge_tab_bottom",
    )
    housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, housing_height)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, housing_height / 2.0)),
    )

    cavity = model.part("cavity")
    cavity.visual(
        cavity_flange_mesh,
        origin=Origin(xyz=(0.0, -0.175, 0.0)),
        material=cavity_enamel,
        name="front_flange",
    )
    cavity.visual(
        Box((cavity_outer_w, cavity_outer_d, cavity_wall)),
        origin=Origin(xyz=(0.0, 0.0, -0.101)),
        material=cavity_enamel,
        name="floor",
    )
    cavity.visual(
        Box((cavity_outer_w, cavity_outer_d, cavity_wall)),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=cavity_enamel,
        name="ceiling",
    )
    cavity.visual(
        Box((cavity_wall, cavity_outer_d, cavity_outer_h)),
        origin=Origin(xyz=(-0.150, 0.0, 0.0)),
        material=cavity_enamel,
        name="left_wall",
    )
    cavity.visual(
        Box((cavity_wall, cavity_outer_d, cavity_outer_h)),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=cavity_enamel,
        name="right_wall",
    )
    cavity.visual(
        Box((cavity_outer_w, cavity_wall, cavity_outer_h)),
        origin=Origin(xyz=(0.0, 0.166, 0.0)),
        material=cavity_enamel,
        name="back_wall",
    )
    cavity.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, -0.001, -0.093)),
        material=metal_gray,
        name="spindle",
    )
    cavity.inertial = Inertial.from_geometry(
        Box((cavity_outer_w, cavity_outer_d, cavity_outer_h)),
        mass=1.2,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_cavity",
        ArticulationType.FIXED,
        parent=housing,
        child=cavity,
        origin=Origin(xyz=(cavity_x, cavity_y, cavity_z)),
    )

    door = model.part("door")
    door.visual(
        door_frame_mesh,
        origin=Origin(xyz=(door_frame_x, 0.0, 0.0)),
        material=trim_dark,
        name="outer_frame",
    )
    door.visual(
        Box((window_width * 0.96, 0.004, window_height * 0.94)),
        origin=Origin(xyz=(door_frame_x, 0.0, 0.0)),
        material=smoky_glass,
        name="window_glass",
    )
    door.visual(
        door_inner_frame_mesh,
        origin=Origin(xyz=(door_frame_x, 0.006, 0.0)),
        material=housing_white,
        name="inner_frame",
    )
    door.visual(
        Box((0.020, 0.020, 0.094)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_stile",
    )
    door.visual(
        Box((0.024, 0.012, 0.196)),
        origin=Origin(xyz=(door_width - 0.020, -0.004, 0.0)),
        material=trim_dark,
        name="latch_edge",
    )
    door.visual(
        _tube_shell_mesh(
            outer_radius=hinge_sleeve_outer,
            inner_radius=hinge_sleeve_inner,
            length=hinge_center_knuckle_length,
            name="door_hinge_knuckle_center",
        ),
        origin=Origin(),
        material=metal_gray,
        name="hinge_sleeve_mid",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=1.45,
        origin=Origin(xyz=(door_frame_x, 0.0, 0.0)),
    )
    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.118, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=glass_plate,
        name="plate",
    )
    turntable.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass_plate,
        name="hub",
    )
    turntable.visual(
        Box((0.018, 0.010, 0.003)),
        origin=Origin(xyz=(0.075, 0.0, 0.0065)),
        material=glass_plate,
        name="plate_marker",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.118, length=0.008),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )
    model.articulation(
        "cavity_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=cavity,
        child=turntable,
        origin=Origin(xyz=(0.0, -0.001, -0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    for index, z_pos in enumerate(button_zs, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            button_mesh,
            origin=Origin(),
            material=button_gray,
            name="pad",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.066, 0.002, 0.022)),
            mass=0.01,
            origin=Origin(),
        )
        model.articulation(
            f"housing_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(control_x, front_face_y - 0.001, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.03,
                lower=0.0,
                upper=0.0012,
            ),
        )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="knob",
    )
    dial.visual(
        Box((0.010, 0.002, 0.004)),
        origin=Origin(xyz=(0.016, -0.010, 0.0)),
        material=button_gray,
        name="pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=0.018),
        mass=0.04,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_dial",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(control_x, -0.209, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=4.0,
            lower=-math.radians(140.0),
            upper=math.radians(140.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    cavity = object_model.get_part("cavity")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    dial = object_model.get_part("dial")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 6)]

    door_joint = object_model.get_articulation("housing_to_door")
    turntable_joint = object_model.get_articulation("cavity_to_turntable")
    dial_joint = object_model.get_articulation("housing_to_dial")
    button_joints = [
        object_model.get_articulation(f"housing_to_button_{index}") for index in range(1, 6)
    ]

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
        "door_hinge_axis_is_vertical",
        tuple(door_joint.axis) == (0.0, 0.0, -1.0),
        details=f"door hinge axis was {door_joint.axis!r}",
    )
    ctx.check(
        "turntable_axis_is_vertical",
        tuple(turntable_joint.axis) == (0.0, 0.0, 1.0),
        details=f"turntable axis was {turntable_joint.axis!r}",
    )
    ctx.check(
        "dial_axis_faces_forward",
        tuple(dial_joint.axis) == (0.0, 1.0, 0.0),
        details=f"dial axis was {dial_joint.axis!r}",
    )
    for index, joint in enumerate(button_joints, start=1):
        ctx.check(
            f"button_{index}_axis_translates_inward",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"button {index} axis was {joint.axis!r}",
        )

    ctx.expect_contact(cavity, housing, elem_a="front_flange", elem_b="front_frame")
    ctx.expect_within(
        door,
        housing,
        axes="xy",
        inner_elem="hinge_sleeve_mid",
        outer_elem="hinge_knuckle_top",
        margin=0.001,
        name="hinge_barrels_share_axis_top",
    )
    ctx.expect_within(
        door,
        housing,
        axes="xy",
        inner_elem="hinge_sleeve_mid",
        outer_elem="hinge_knuckle_bottom",
        margin=0.001,
        name="hinge_barrels_share_axis_bottom",
    )
    ctx.expect_gap(
        housing,
        door,
        axis="z",
        positive_elem="hinge_knuckle_top",
        negative_elem="hinge_sleeve_mid",
        min_gap=0.002,
        max_gap=0.006,
        name="upper_hinge_barrel_gap",
    )
    ctx.expect_gap(
        door,
        housing,
        axis="z",
        positive_elem="hinge_sleeve_mid",
        negative_elem="hinge_knuckle_bottom",
        min_gap=0.002,
        max_gap=0.006,
        name="lower_hinge_barrel_gap",
    )
    ctx.expect_contact(turntable, cavity, elem_a="hub", elem_b="spindle")
    ctx.expect_contact(dial, housing, elem_a="knob", elem_b="dial_bezel")
    ctx.expect_within(turntable, cavity, axes="xy", margin=0.03, name="turntable_within_cavity")

    for index, button in enumerate(buttons, start=1):
        ctx.expect_contact(button, housing, name=f"button_{index}_seats_on_ring")

    closed_door_aabb = ctx.part_world_aabb(door)
    assert closed_door_aabb is not None
    with ctx.pose({door_joint: math.radians(85.0)}):
        ctx.expect_within(
            door,
            housing,
            axes="xy",
            inner_elem="hinge_sleeve_mid",
            outer_elem="hinge_knuckle_top",
            margin=0.001,
            name="door_stays_supported_when_open",
        )
        ctx.expect_gap(
            housing,
            door,
            axis="z",
            positive_elem="hinge_knuckle_top",
            negative_elem="hinge_sleeve_mid",
            min_gap=0.002,
            max_gap=0.006,
            name="open_upper_hinge_barrel_gap",
        )
        open_door_aabb = ctx.part_world_aabb(door)
        assert open_door_aabb is not None
        ctx.check(
            "door_swings_sideways",
            open_door_aabb[1][0] < closed_door_aabb[1][0] - 0.12
            and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10,
            details=(
                f"closed aabb={closed_door_aabb!r}, "
                f"open aabb={open_door_aabb!r}"
            ),
        )

    marker_rest = ctx.part_element_world_aabb(turntable, elem="plate_marker")
    assert marker_rest is not None
    marker_rest_center = (
        (marker_rest[0][0] + marker_rest[1][0]) / 2.0,
        (marker_rest[0][1] + marker_rest[1][1]) / 2.0,
        (marker_rest[0][2] + marker_rest[1][2]) / 2.0,
    )
    with ctx.pose({turntable_joint: 1.2}):
        marker_spun = ctx.part_element_world_aabb(turntable, elem="plate_marker")
        assert marker_spun is not None
        marker_spun_center = (
            (marker_spun[0][0] + marker_spun[1][0]) / 2.0,
            (marker_spun[0][1] + marker_spun[1][1]) / 2.0,
            (marker_spun[0][2] + marker_spun[1][2]) / 2.0,
        )
        ctx.check(
            "turntable_marker_orbits_spindle",
            abs(marker_spun_center[0] - marker_rest_center[0]) > 0.02
            and abs(marker_spun_center[1] - marker_rest_center[1]) > 0.02,
            details=f"rest={marker_rest_center!r}, spun={marker_spun_center!r}",
        )
        ctx.expect_contact(turntable, cavity, elem_a="hub", elem_b="spindle")

    pointer_rest = ctx.part_element_world_aabb(dial, elem="pointer")
    assert pointer_rest is not None
    pointer_rest_center = (
        (pointer_rest[0][0] + pointer_rest[1][0]) / 2.0,
        (pointer_rest[0][1] + pointer_rest[1][1]) / 2.0,
        (pointer_rest[0][2] + pointer_rest[1][2]) / 2.0,
    )
    with ctx.pose({dial_joint: math.radians(80.0)}):
        pointer_turned = ctx.part_element_world_aabb(dial, elem="pointer")
        assert pointer_turned is not None
        pointer_turned_center = (
            (pointer_turned[0][0] + pointer_turned[1][0]) / 2.0,
            (pointer_turned[0][1] + pointer_turned[1][1]) / 2.0,
            (pointer_turned[0][2] + pointer_turned[1][2]) / 2.0,
        )
        ctx.check(
            "dial_pointer_rotates_about_knob_axis",
            math.hypot(
                pointer_turned_center[0] - pointer_rest_center[0],
                pointer_turned_center[2] - pointer_rest_center[2],
            )
            > 0.01
            and abs(pointer_turned_center[1] - pointer_rest_center[1]) < 1e-6,
            details=f"rest={pointer_rest_center!r}, turned={pointer_turned_center!r}",
        )
        ctx.expect_contact(dial, housing, elem_a="knob", elem_b="dial_bezel")

    for index, (button, joint) in enumerate(zip(buttons, button_joints), start=1):
        rest_pos = ctx.part_world_position(button)
        assert rest_pos is not None
        with ctx.pose({joint: 0.0011}):
            pressed_pos = ctx.part_world_position(button)
            assert pressed_pos is not None
            ctx.check(
                f"button_{index}_presses_inward",
                pressed_pos[1] > rest_pos[1] + 0.0009
                and abs(pressed_pos[0] - rest_pos[0]) < 1e-6
                and abs(pressed_pos[2] - rest_pos[2]) < 1e-6,
                details=f"rest={rest_pos!r}, pressed={pressed_pos!r}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
