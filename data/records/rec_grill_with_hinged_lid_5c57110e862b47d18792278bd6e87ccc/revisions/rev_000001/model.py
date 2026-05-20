from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


BARREL_LENGTH = 0.92
BARREL_RADIUS = 0.23
SHELL_THICKNESS = 0.0035
FRONT_SEAM_ANGLE = 1.05
REAR_HINGE_ANGLE = -1.20
HINGE_CLEARANCE_ANGLE = 0.10


def _add_quad(geom, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _rotate_x(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (x, y * ca - z * sa, y * sa + z * ca)


def _translate(
    base: tuple[float, float, float],
    offset: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (base[0] + offset[0], base[1] + offset[1], base[2] + offset[2])


def _point_on_cylinder(
    *,
    center_y: float,
    center_z: float,
    radius: float,
    angle: float,
    x: float = 0.0,
) -> tuple[float, float, float]:
    return (x, center_y + radius * math.sin(angle), center_z + radius * math.cos(angle))


def _build_partial_barrel_shell(
    *,
    length: float,
    outer_radius: float,
    thickness: float,
    start_angle: float,
    end_angle: float,
    center_y: float = 0.0,
    center_z: float = 0.0,
    arc_segments: int = 40,
):
    geom = BoxGeometry((0.0001, 0.0001, 0.0001))
    geom.vertices = []
    geom.faces = []

    x0 = -length * 0.5
    x1 = length * 0.5
    inner_radius = outer_radius - thickness
    angles = [
        start_angle + (end_angle - start_angle) * (index / arc_segments)
        for index in range(arc_segments + 1)
    ]

    outer_left: list[int] = []
    outer_right: list[int] = []
    inner_left: list[int] = []
    inner_right: list[int] = []

    for angle in angles:
        outer_left.append(
            geom.add_vertex(
                x0,
                center_y + outer_radius * math.sin(angle),
                center_z + outer_radius * math.cos(angle),
            )
        )
        outer_right.append(
            geom.add_vertex(
                x1,
                center_y + outer_radius * math.sin(angle),
                center_z + outer_radius * math.cos(angle),
            )
        )
        inner_left.append(
            geom.add_vertex(
                x0,
                center_y + inner_radius * math.sin(angle),
                center_z + inner_radius * math.cos(angle),
            )
        )
        inner_right.append(
            geom.add_vertex(
                x1,
                center_y + inner_radius * math.sin(angle),
                center_z + inner_radius * math.cos(angle),
            )
        )

    for index in range(arc_segments):
        _add_quad(
            geom,
            outer_left[index],
            outer_left[index + 1],
            outer_right[index + 1],
            outer_right[index],
        )
        _add_quad(
            geom,
            inner_left[index],
            inner_right[index],
            inner_right[index + 1],
            inner_left[index + 1],
        )
        _add_quad(
            geom,
            outer_left[index],
            inner_left[index],
            inner_left[index + 1],
            outer_left[index + 1],
        )
        _add_quad(
            geom,
            outer_right[index],
            outer_right[index + 1],
            inner_right[index + 1],
            inner_right[index],
        )

    _add_quad(
        geom,
        outer_left[0],
        outer_right[0],
        inner_right[0],
        inner_left[0],
    )
    _add_quad(
        geom,
        outer_left[-1],
        inner_left[-1],
        inner_right[-1],
        outer_right[-1],
    )
    return geom


def _build_wheel_mesh():
    wheel_radius = 0.17
    wheel = TorusGeometry(
        radius=0.125,
        tube=0.022,
        radial_segments=20,
        tubular_segments=44,
    ).rotate_y(math.pi / 2.0)
    wheel.merge(CylinderGeometry(radius=0.042, height=0.052, radial_segments=28).rotate_y(math.pi / 2.0))

    spoke_length = 0.112
    spoke_size = (0.016, spoke_length, 0.018)
    spoke_center = 0.090
    for index in range(6):
        angle = index * (math.pi / 3.0)
        spoke = BoxGeometry(spoke_size)
        spoke.rotate_x(angle)
        spoke.translate(0.0, spoke_center * math.cos(angle), spoke_center * math.sin(angle))
        wheel.merge(spoke)

    wheel.merge(
        CylinderGeometry(radius=wheel_radius, height=0.010, radial_segments=40)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, -0.021)
    )
    wheel.merge(
        CylinderGeometry(radius=wheel_radius, height=0.010, radial_segments=40)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, 0.021)
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wagon_style_charcoal_grill")

    enamel_black = model.material("enamel_black", rgba=(0.12, 0.12, 0.12, 1.0))
    frame_charcoal = model.material("frame_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.32, 0.33, 0.35, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.09, 0.09, 0.09, 1.0))
    handle_wood = model.material("handle_wood", rgba=(0.48, 0.30, 0.16, 1.0))

    hinge_y = BARREL_RADIUS * math.sin(REAR_HINGE_ANGLE)
    hinge_z = BARREL_RADIUS * math.cos(REAR_HINGE_ANGLE)

    lower_assembly = model.part("lower_assembly")
    lower_shell_mesh = mesh_from_geometry(
        _build_partial_barrel_shell(
            length=BARREL_LENGTH,
            outer_radius=BARREL_RADIUS,
            thickness=SHELL_THICKNESS,
            start_angle=FRONT_SEAM_ANGLE,
            end_angle=REAR_HINGE_ANGLE + 2.0 * math.pi - HINGE_CLEARANCE_ANGLE,
        ),
        "grill_lower_shell",
    )
    lower_assembly.visual(lower_shell_mesh, material=enamel_black, name="lower_shell")

    lower_assembly.visual(
        Box((0.034, 0.070, 0.066)),
        origin=Origin(xyz=(-0.32, 0.0, -0.258)),
        material=steel_dark,
        name="left_mount_block",
    )
    lower_assembly.visual(
        Box((0.034, 0.070, 0.066)),
        origin=Origin(xyz=(0.32, 0.0, -0.258)),
        material=steel_dark,
        name="right_mount_block",
    )
    for x_pos, prefix in ((-0.32, "left"), (0.32, "right")):
        lower_assembly.visual(
            Box((0.03, 0.34, 0.04)),
            origin=Origin(xyz=(x_pos, 0.0, -0.31)),
            material=frame_charcoal,
            name=f"{prefix}_top_beam",
        )
        lower_assembly.visual(
            Box((0.03, 0.04, 0.37)),
            origin=Origin(xyz=(x_pos, 0.15, -0.515)),
            material=frame_charcoal,
            name=f"{prefix}_front_leg",
        )
        lower_assembly.visual(
            Box((0.03, 0.04, 0.37)),
            origin=Origin(xyz=(x_pos, -0.15, -0.515)),
            material=frame_charcoal,
            name=f"{prefix}_rear_leg",
        )
        axle_center_x = -0.356 if x_pos < 0.0 else 0.356
        lower_assembly.visual(
            Box((0.106, 0.04, 0.04)),
            origin=Origin(xyz=(axle_center_x, 0.0, -0.50)),
            material=frame_charcoal,
            name=f"{prefix}_axle_support",
        )
        lower_assembly.visual(
            Cylinder(radius=0.020, length=0.084),
            origin=Origin(
                xyz=(-0.445, 0.0, -0.50) if x_pos < 0.0 else (0.445, 0.0, -0.50),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel_dark,
            name=f"{prefix}_hub_boss",
        )
        lower_assembly.visual(
            Box((0.060, 0.040, 0.084)),
            origin=Origin(xyz=(-0.345, 0.0, -0.561) if x_pos < 0.0 else (0.345, 0.0, -0.561)),
            material=frame_charcoal,
            name=f"{prefix}_axle_drop",
        )

    lower_assembly.visual(
        Box((0.64, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, 0.17, -0.345)),
        material=frame_charcoal,
        name="front_upper_rail",
    )
    lower_assembly.visual(
        Box((0.64, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, -0.17, -0.345)),
        material=frame_charcoal,
        name="rear_upper_rail",
    )
    lower_assembly.visual(
        Box((0.64, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, 0.12, -0.63)),
        material=frame_charcoal,
        name="front_lower_rail",
    )
    lower_assembly.visual(
        Box((0.64, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, -0.12, -0.63)),
        material=frame_charcoal,
        name="rear_lower_rail",
    )
    lower_assembly.visual(
        Box((0.70, 0.26, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.608)),
        material=steel_dark,
        name="lower_shelf",
    )

    lower_assembly.visual(
        Cylinder(radius=0.014, length=0.12),
        origin=Origin(xyz=(-0.30, hinge_y, hinge_z, ), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="left_hinge_knuckle",
    )
    lower_assembly.visual(
        Cylinder(radius=0.014, length=0.12),
        origin=Origin(xyz=(0.30, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="right_hinge_knuckle",
    )
    lower_assembly.visual(
        Box((0.12, 0.020, 0.024)),
        origin=Origin(xyz=(-0.30, hinge_y + 0.001, 0.104)),
        material=steel_dark,
        name="left_hinge_bracket",
    )
    lower_assembly.visual(
        Box((0.12, 0.020, 0.024)),
        origin=Origin(xyz=(0.30, hinge_y + 0.001, 0.104)),
        material=steel_dark,
        name="right_hinge_bracket",
    )
    lower_assembly.visual(
        Box((0.050, 0.020, 0.070)),
        origin=Origin(xyz=(-0.30, hinge_y + 0.002, 0.057)),
        material=steel_dark,
        name="left_hinge_strap",
    )
    lower_assembly.visual(
        Box((0.050, 0.020, 0.070)),
        origin=Origin(xyz=(0.30, hinge_y + 0.002, 0.057)),
        material=steel_dark,
        name="right_hinge_strap",
    )

    lower_assembly.inertial = Inertial.from_geometry(
        Box((1.06, 0.54, 0.94)),
        mass=20.0,
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
    )

    lid = model.part("lid")
    lid_center_y = -hinge_y
    lid_center_z = -hinge_z
    lid_shell_mesh = mesh_from_geometry(
        _build_partial_barrel_shell(
            length=BARREL_LENGTH,
            outer_radius=BARREL_RADIUS,
            thickness=SHELL_THICKNESS,
            start_angle=REAR_HINGE_ANGLE + HINGE_CLEARANCE_ANGLE,
            end_angle=FRONT_SEAM_ANGLE,
            center_y=lid_center_y,
            center_z=lid_center_z,
        ),
        "grill_lid_shell",
    )
    lid.visual(lid_shell_mesh, material=enamel_black, name="lid_shell")
    lid.visual(
        Cylinder(radius=0.010, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="center_hinge_knuckle",
    )
    lid.visual(
        Box((0.22, 0.022, 0.070)),
        origin=Origin(xyz=(0.0, 0.009, 0.035), rpy=(0.0, 0.0, 0.0)),
        material=steel_dark,
        name="center_hinge_strap",
    )

    handle_angle = 0.42
    handle_roll = -handle_angle
    handle_mount = _point_on_cylinder(
        center_y=lid_center_y,
        center_z=lid_center_z,
        radius=BARREL_RADIUS - 0.0015,
        angle=handle_angle,
    )
    handle_center = _translate(handle_mount, _rotate_x((0.0, 0.0, 0.052), handle_roll))
    for x_pos, prefix in ((-0.12, "left"), (0.12, "right")):
        base_center = _translate(handle_mount, _rotate_x((x_pos, 0.0, 0.012), handle_roll))
        post_center = _translate(handle_mount, _rotate_x((x_pos, 0.0, 0.030), handle_roll))
        lid.visual(
            Box((0.028, 0.016, 0.024)),
            origin=Origin(xyz=base_center, rpy=(handle_roll, 0.0, 0.0)),
            material=steel_dark,
            name=f"{prefix}_handle_base",
        )
        lid.visual(
            Box((0.022, 0.012, 0.034)),
            origin=Origin(xyz=post_center, rpy=(handle_roll, 0.0, 0.0)),
            material=steel_dark,
            name=f"{prefix}_handle_post",
        )
    lid.visual(
        Box((0.29, 0.030, 0.022)),
        origin=Origin(xyz=handle_center, rpy=(handle_roll, 0.0, 0.0)),
        material=handle_wood,
        name="handle_grip",
    )

    panel_roll = -FRONT_SEAM_ANGLE
    panel_height = 0.055
    panel_thickness = 0.003
    panel_center = _translate(
        _point_on_cylinder(
            center_y=lid_center_y,
            center_z=lid_center_z,
            radius=BARREL_RADIUS,
            angle=FRONT_SEAM_ANGLE,
        ),
        _rotate_x((0.0, panel_height * 0.5, panel_thickness * 0.5), panel_roll),
    )

    def panel_xyz(local_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
        return _translate(panel_center, _rotate_x(local_xyz, panel_roll))

    lid.visual(
        Box((0.17, 0.010, panel_thickness)),
        origin=Origin(xyz=panel_xyz((0.0, -0.0225, 0.0)), rpy=(panel_roll, 0.0, 0.0)),
        material=steel_dark,
        name="vent_top_rail",
    )
    lid.visual(
        Box((0.17, 0.008, panel_thickness)),
        origin=Origin(xyz=panel_xyz((0.0, 0.0, 0.0)), rpy=(panel_roll, 0.0, 0.0)),
        material=steel_dark,
        name="vent_center_rail",
    )
    lid.visual(
        Box((0.17, 0.010, panel_thickness)),
        origin=Origin(xyz=panel_xyz((0.0, 0.0225, 0.0)), rpy=(panel_roll, 0.0, 0.0)),
        material=steel_dark,
        name="vent_bottom_rail",
    )
    lid.visual(
        Box((0.012, panel_height, panel_thickness)),
        origin=Origin(xyz=panel_xyz((-0.079, 0.0, 0.0)), rpy=(panel_roll, 0.0, 0.0)),
        material=steel_dark,
        name="vent_left_rail",
    )
    lid.visual(
        Box((0.012, panel_height, panel_thickness)),
        origin=Origin(xyz=panel_xyz((0.079, 0.0, 0.0)), rpy=(panel_roll, 0.0, 0.0)),
        material=steel_dark,
        name="vent_right_rail",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.96, 0.40, 0.24)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.17, 0.06)),
    )

    vent_slider = model.part("vent_slider")
    vent_slider.visual(
        Box((0.145, 0.044, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=steel_dark,
        name="vent_plate",
    )
    vent_slider.visual(
        Box((0.024, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.0125, 0.0035)),
        material=steel_dark,
        name="upper_tab",
    )
    vent_slider.visual(
        Box((0.030, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, -0.0125, 0.0055)),
        material=steel_dark,
        name="upper_backer",
    )
    vent_slider.visual(
        Box((0.024, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0125, 0.0035)),
        material=steel_dark,
        name="lower_tab",
    )
    vent_slider.visual(
        Box((0.030, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.0125, 0.0055)),
        material=steel_dark,
        name="lower_backer",
    )
    vent_slider.inertial = Inertial.from_geometry(
        Box((0.15, 0.05, 0.010)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
    )

    wheel_mesh = mesh_from_geometry(_build_wheel_mesh(), "wagon_wheel")
    left_wheel = model.part("left_wheel")
    left_wheel.visual(wheel_mesh, material=wheel_black, name="wheel")
    left_wheel.inertial = Inertial.from_geometry(Box((0.05, 0.34, 0.34)), mass=1.8)

    right_wheel = model.part("right_wheel")
    right_wheel.visual(wheel_mesh, material=wheel_black, name="wheel")
    right_wheel.inertial = Inertial.from_geometry(Box((0.05, 0.34, 0.34)), mass=1.8)

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_assembly,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.1,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "vent_slider_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=vent_slider,
        origin=Origin(xyz=panel_center, rpy=(panel_roll, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.18,
            lower=-0.045,
            upper=0.045,
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_assembly,
        child=left_wheel,
        origin=Origin(xyz=(-0.513, 0.0, -0.50)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_assembly,
        child=right_wheel,
        origin=Origin(xyz=(0.513, 0.0, -0.50)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_assembly = object_model.get_part("lower_assembly")
    lid = object_model.get_part("lid")
    vent_slider = object_model.get_part("vent_slider")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    lid_hinge = object_model.get_articulation("lid_hinge")
    vent_slider_slide = object_model.get_articulation("vent_slider_slide")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("lid_hinge_axis", lid_hinge.axis == (1.0, 0.0, 0.0), "Lid hinge must run along barrel length.")
    ctx.check(
        "vent_slider_axis",
        vent_slider_slide.axis == (1.0, 0.0, 0.0),
        "Vent slider must translate laterally across the front slots.",
    )
    ctx.check(
        "wheel_axes",
        left_wheel_spin.axis == (1.0, 0.0, 0.0) and right_wheel_spin.axis == (1.0, 0.0, 0.0),
        "Both wagon wheels should spin about their axle supports.",
    )

    ctx.expect_contact(lid, lower_assembly, name="lid_contacts_lower_assembly_closed")
    ctx.expect_contact(vent_slider, lid, name="vent_slider_clipped_to_lid")
    ctx.expect_contact(left_wheel, lower_assembly, name="left_wheel_contact_support")
    ctx.expect_contact(right_wheel, lower_assembly, name="right_wheel_contact_support")

    rest_handle = ctx.part_element_world_aabb(lid, elem="handle_grip")
    assert rest_handle is not None
    with ctx.pose({lid_hinge: 1.15}):
        open_handle = ctx.part_element_world_aabb(lid, elem="handle_grip")
        assert open_handle is not None
        ctx.check(
            "lid_handle_rises_when_open",
            open_handle[1][2] > rest_handle[1][2] + 0.18,
            f"Open lid handle top={open_handle[1][2]:.4f} should rise well above closed top={rest_handle[1][2]:.4f}.",
        )
        ctx.expect_contact(vent_slider, lid, name="vent_stays_clipped_when_lid_open")

    with ctx.pose({vent_slider_slide: -0.04}):
        ctx.expect_contact(vent_slider, lid, name="vent_contact_at_left_travel")

    with ctx.pose({vent_slider_slide: 0.04}):
        ctx.expect_contact(vent_slider, lid, name="vent_contact_at_right_travel")

    with ctx.pose({left_wheel_spin: 1.4, right_wheel_spin: -0.9}):
        ctx.expect_contact(left_wheel, lower_assembly, name="left_wheel_stays_on_support")
        ctx.expect_contact(right_wheel, lower_assembly, name="right_wheel_stays_on_support")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
