from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    FanRotorGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_housing_shell_geometry(
    *,
    width: float,
    depth: float,
    height: float,
    wall: float,
    face_thickness: float,
    corner_radius: float,
    opening_radius: float,
    fan_center_x: float,
    fan_center_z: float,
):
    outer = ExtrudeGeometry.centered(
        rounded_rect_profile(width, height, corner_radius, corner_segments=8),
        depth,
    )
    outer.rotate((1.0, 0.0, 0.0), math.pi / 2.0).translate(0.0, 0.0, height * 0.5)

    inner_corner_radius = max(min(corner_radius - wall * 0.5, corner_radius), 0.004)
    inner = ExtrudeGeometry.centered(
        rounded_rect_profile(
            width - 2.0 * wall,
            height - 2.0 * wall,
            inner_corner_radius,
            corner_segments=8,
        ),
        depth - 2.0 * face_thickness,
    )
    inner.rotate((1.0, 0.0, 0.0), math.pi / 2.0).translate(0.0, 0.0, height * 0.5)

    shell = boolean_difference(outer, inner)
    for side_sign in (-1.0, 1.0):
        cutter = CylinderGeometry(opening_radius, depth + 0.040, radial_segments=48)
        cutter.rotate((1.0, 0.0, 0.0), math.pi / 2.0).translate(
            side_sign * fan_center_x,
            0.0,
            fan_center_z,
        )
        shell = boolean_difference(shell, cutter)
    return shell


def _build_front_grille_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
    fan_center_x: float,
    fan_center_y: float,
    opening_radius: float,
    slot_width: float,
    slot_pitch: float,
    aperture_margin: float,
    name: str,
):
    panel = ExtrudeGeometry.centered(
        rounded_rect_profile(width, height, corner_radius, corner_segments=8),
        thickness,
    )
    cut_radius = max(opening_radius - aperture_margin, slot_width * 1.5)
    usable_half_span = cut_radius * 0.78
    slot_count = max(int((2.0 * usable_half_span) / slot_pitch) + 1, 4)
    for side_sign in (-1.0, 1.0):
        center_x = side_sign * fan_center_x
        if slot_count == 1:
            offsets = [0.0]
        else:
            offsets = [
                -usable_half_span + (2.0 * usable_half_span * index) / (slot_count - 1)
                for index in range(slot_count)
            ]
        for offset in offsets:
            half_height = math.sqrt(max(cut_radius * cut_radius - offset * offset, 0.0))
            slot_height = max(2.0 * (half_height - slot_width * 0.7), slot_width * 2.2)
            cutter = ExtrudeGeometry.centered(
                rounded_rect_profile(slot_width, slot_height, slot_width * 0.48, corner_segments=6),
                thickness + 0.006,
            )
            cutter.translate(center_x + offset, fan_center_y, 0.0)
            panel = boolean_difference(panel, cutter)
    return _mesh(name, panel)


def _add_vertical_grille_bars(
    part,
    *,
    prefix: str,
    fan_center_x: float,
    fan_center_z: float,
    opening_radius: float,
    bar_depth: float,
    bar_width: float,
    count: int,
    material,
    local_y: float,
    z_shift: float = 0.0,
) -> None:
    usable_half_span = opening_radius * 0.82
    if count == 1:
        offsets = [0.0]
    else:
        offsets = [
            -usable_half_span + (2.0 * usable_half_span * index) / (count - 1)
            for index in range(count)
        ]
    for index, offset in enumerate(offsets):
        part.visual(
            Box((bar_width, bar_depth, opening_radius * 2.0 + 0.018)),
            origin=Origin(xyz=(fan_center_x + offset, local_y, fan_center_z + z_shift)),
            material=material,
            name=f"{prefix}_{index:02d}",
        )


def _add_horizontal_grille_bars(
    part,
    *,
    prefix: str,
    fan_center_x: float,
    fan_center_z: float,
    opening_radius: float,
    bar_depth: float,
    bar_height: float,
    count: int,
    material,
    local_y: float,
) -> None:
    usable_half_span = opening_radius * 0.74
    if count == 1:
        offsets = [0.0]
    else:
        offsets = [
            -usable_half_span + (2.0 * usable_half_span * index) / (count - 1)
            for index in range(count)
        ]
    for index, offset in enumerate(offsets):
        span = 2.0 * math.sqrt(max(opening_radius * opening_radius - offset * offset, 0.0))
        part.visual(
            Box((max(span - 0.014, 0.030), bar_depth, bar_height)),
            origin=Origin(xyz=(fan_center_x, local_y, fan_center_z + offset)),
            material=material,
            name=f"{prefix}_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_window_box_fan")

    width = 0.68
    depth = 0.18
    height = 0.38
    wall = 0.024
    bezel_thickness = 0.014
    door_thickness = 0.012
    top_band_height = 0.075
    opening_radius = 0.112
    center_divider = 0.026
    fan_center_x = opening_radius + center_divider * 0.5
    fan_center_z = 0.172
    rear_bezel_y = depth * 0.5 - bezel_thickness * 0.5
    door_bottom_z = 0.020
    door_top_z = height - top_band_height
    door_height = door_top_z - door_bottom_z
    door_y = -depth * 0.5 - door_thickness * 0.5
    rotor_axis_y = -0.010
    motor_center_y = 0.046
    motor_support_center_y = (rear_bezel_y + motor_center_y) * 0.5
    motor_support_depth = rear_bezel_y - motor_center_y + 0.020

    housing_white = model.material("housing_white", rgba=(0.90, 0.92, 0.94, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.74, 0.77, 0.80, 1.0))
    grille_gray = model.material("grille_gray", rgba=(0.63, 0.67, 0.71, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    blade_blue = model.material("blade_blue", rgba=(0.58, 0.78, 0.92, 0.88))

    housing_shell_mesh = _mesh(
        "box_fan_housing_shell.obj",
        _build_housing_shell_geometry(
            width=width,
            depth=depth,
            height=height,
            wall=wall,
            face_thickness=bezel_thickness,
            corner_radius=0.020,
            opening_radius=opening_radius,
            fan_center_x=fan_center_x,
            fan_center_z=fan_center_z,
        ),
    )
    rotor_mesh = _mesh(
        "box_fan_rotor.obj",
        FanRotorGeometry(
            opening_radius * 0.90,
            0.026,
            5,
            thickness=0.018,
            blade_pitch_deg=26.0,
            blade_sweep_deg=16.0,
            center=True,
        ),
    )

    housing = model.part("housing")
    housing.visual(
        housing_shell_mesh,
        origin=Origin(),
        material=housing_white,
        name="housing_shell",
    )
    housing.visual(
        Box((center_divider, depth - 2.0 * bezel_thickness, height - 2.0 * wall)),
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
        material=housing_white,
        name="center_divider_shell",
    )
    housing.visual(
        Box((0.182, 0.024, 0.060)),
        origin=Origin(xyz=(0.0, -depth * 0.5 + 0.015, height - top_band_height * 0.5)),
        material=trim_gray,
        name="control_panel",
    )
    housing.visual(
        Box((0.120, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -depth * 0.5 + 0.024, height - top_band_height + 0.020)),
        material=grille_gray,
        name="speed_scale",
    )
    housing.visual(
        Box((0.090, depth * 0.62, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, height - top_band_height - 0.004)),
        material=trim_gray,
        name="upper_cross_shelf",
    )
    housing.visual(
        Box((0.060, 0.028, 0.012)),
        origin=Origin(xyz=(-width * 0.5 + 0.070, 0.0, 0.006)),
        material=trim_gray,
        name="left_foot",
    )
    housing.visual(
        Box((0.060, 0.028, 0.012)),
        origin=Origin(xyz=(width * 0.5 - 0.070, 0.0, 0.006)),
        material=trim_gray,
        name="right_foot",
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        x_center = side_sign * fan_center_x
        housing.visual(
            Cylinder(radius=0.034, length=0.058),
            origin=Origin(
                xyz=(x_center, motor_center_y, fan_center_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=motor_gray,
            name=f"{side_name}_motor_can",
        )
        housing.visual(
            Cylinder(radius=0.006, length=0.016),
            origin=Origin(
                xyz=(x_center, 0.017, fan_center_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_plastic,
            name=f"{side_name}_motor_shaft",
        )
        housing.visual(
            Box((opening_radius * 1.76, motor_support_depth, 0.012)),
            origin=Origin(xyz=(x_center, motor_support_center_y, fan_center_z)),
            material=motor_gray,
            name=f"{side_name}_motor_brace_horizontal",
        )
        housing.visual(
            Box((0.012, motor_support_depth, opening_radius * 1.76)),
            origin=Origin(xyz=(x_center, motor_support_center_y, fan_center_z)),
            material=motor_gray,
            name=f"{side_name}_motor_brace_vertical",
        )
        _add_vertical_grille_bars(
            housing,
            prefix=f"{side_name}_rear_bar",
            fan_center_x=x_center,
            fan_center_z=fan_center_z,
            opening_radius=opening_radius,
            bar_depth=0.006,
            bar_width=0.006,
            count=7,
            material=grille_gray,
            local_y=rear_bezel_y,
        )
        _add_horizontal_grille_bars(
            housing,
            prefix=f"{side_name}_rear_crossbar",
            fan_center_x=x_center,
            fan_center_z=fan_center_z,
            opening_radius=opening_radius,
            bar_depth=0.006,
            bar_height=0.006,
            count=5,
            material=grille_gray,
            local_y=rear_bezel_y,
        )

    housing.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    front_grille = model.part("front_grille")
    door_width = width - 0.018
    fan_local_z = fan_center_z - door_top_z
    grille_panel_mesh = _build_front_grille_panel_mesh(
        width=door_width,
        height=door_height,
        thickness=door_thickness,
        corner_radius=0.016,
        fan_center_x=fan_center_x,
        fan_center_y=fan_local_z + door_height * 0.5,
        opening_radius=opening_radius,
        slot_width=0.016,
        slot_pitch=0.028,
        aperture_margin=0.014,
        name="box_fan_front_grille_panel.obj",
    )

    front_grille.visual(
        grille_panel_mesh,
        origin=Origin(
            xyz=(0.0, 0.0, -door_height * 0.5),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_gray,
        name="grille_panel",
    )
    front_grille.visual(
        Box((0.100, door_thickness, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -door_height + 0.030)),
        material=trim_gray,
        name="grille_pull",
    )
    front_grille.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, -door_height * 0.5)),
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        rotor = model.part(f"{side_name}_rotor")
        rotor.visual(
            rotor_mesh,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=blade_blue,
            name="fan_rotor",
        )
        rotor.visual(
            Cylinder(radius=0.013, length=0.038),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_plastic,
            name="hub_cap",
        )
        rotor.inertial = Inertial.from_geometry(
            Cylinder(radius=opening_radius * 0.90, length=0.024),
            mass=0.18,
        )
        model.articulation(
            f"{side_name}_motor_spin",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=rotor,
            origin=Origin(xyz=(side_sign * fan_center_x, rotor_axis_y, fan_center_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=32.0),
        )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="knob_body",
    )
    speed_knob.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="knob_cap",
    )
    speed_knob.visual(
        Box((0.004, 0.008, 0.020)),
        origin=Origin(xyz=(0.017, -0.010, 0.0)),
        material=trim_gray,
        name="knob_pointer",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.020),
        mass=0.08,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "front_grille_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_grille,
        origin=Origin(xyz=(0.0, door_y, door_top_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=1.8,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "speed_knob_turn",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(0.0, -depth * 0.5 - 0.010, height - top_band_height * 0.5)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    housing = object_model.get_part("housing")
    front_grille = object_model.get_part("front_grille")
    speed_knob = object_model.get_part("speed_knob")
    left_rotor = object_model.get_part("left_rotor")
    right_rotor = object_model.get_part("right_rotor")

    grille_hinge = object_model.get_articulation("front_grille_hinge")

    ctx.expect_overlap(
        front_grille,
        housing,
        axes="xz",
        min_overlap=0.28,
        name="front grille covers the twin fan opening area",
    )
    ctx.expect_gap(
        housing,
        front_grille,
        axis="y",
        max_gap=0.020,
        max_penetration=0.0001,
        name="closed grille sits just in front of the housing face",
    )
    ctx.expect_origin_distance(
        left_rotor,
        right_rotor,
        axes="x",
        min_dist=0.20,
        max_dist=0.30,
        name="two rotor stages are clearly separated across the housing width",
    )

    closed_grille_aabb = ctx.part_world_aabb(front_grille)
    knob_pos = ctx.part_world_position(speed_knob)
    with ctx.pose({grille_hinge: 1.05}):
        open_grille_aabb = ctx.part_world_aabb(front_grille)

    ctx.check(
        "grille lifts upward when opened",
        closed_grille_aabb is not None
        and open_grille_aabb is not None
        and open_grille_aabb[0][2] > closed_grille_aabb[0][2] + 0.10
        and open_grille_aabb[0][1] < closed_grille_aabb[0][1] - 0.08,
        details=f"closed_aabb={closed_grille_aabb}, open_aabb={open_grille_aabb}",
    )
    ctx.check(
        "speed knob stays on the exposed top center panel",
        knob_pos is not None
        and abs(knob_pos[0]) < 0.01
        and knob_pos[2] > 0.29,
        details=f"knob_pos={knob_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
