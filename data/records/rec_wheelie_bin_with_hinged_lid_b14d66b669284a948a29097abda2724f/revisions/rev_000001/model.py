from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan, cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _cylinder_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, pi / 2.0, 0.0)
    if axis == "y":
        return (pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_bolt(
    part,
    *,
    pos: tuple[float, float, float],
    axis: str,
    radius: float,
    length: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=pos, rpy=_cylinder_rpy(axis)),
        material=material,
        name=name,
    )


def _add_bolted_plate(
    part,
    *,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    normal_axis: str,
    plate_material,
    bolt_material,
    plate_name: str | None = None,
) -> None:
    if normal_axis == "y":
        width, thickness, height = size
        part.visual(
            Box((width, thickness, height)),
            origin=Origin(xyz=center),
            material=plate_material,
            name=plate_name,
        )
        x_margin = width * 0.32
        z_margin = height * 0.32
        for sx in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                _add_bolt(
                    part,
                    pos=(center[0] + sx * x_margin, center[1], center[2] + sz * z_margin),
                    axis="y",
                    radius=0.0055,
                    length=max(thickness + 0.010, 0.016),
                    material=bolt_material,
                )
        return

    thickness, width, height = size
    part.visual(
        Box((thickness, width, height)),
        origin=Origin(xyz=center),
        material=plate_material,
        name=plate_name,
    )
    y_margin = width * 0.28
    z_margin = height * 0.32
    for sy in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            _add_bolt(
                part,
                pos=(center[0], center[1] + sy * y_margin, center[2] + sz * z_margin),
                axis="x",
                radius=0.0055,
                length=max(thickness + 0.010, 0.016),
                material=bolt_material,
            )


def _wheel_tire_mesh(name: str, *, radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.52, -half_width * 0.98),
        (radius * 0.74, -half_width),
        (radius * 0.91, -half_width * 0.76),
        (radius * 0.99, -half_width * 0.34),
        (radius, 0.0),
        (radius * 0.99, half_width * 0.34),
        (radius * 0.91, half_width * 0.76),
        (radius * 0.74, half_width),
        (radius * 0.52, half_width * 0.98),
        (radius * 0.40, half_width * 0.28),
        (radius * 0.36, 0.0),
        (radius * 0.40, -half_width * 0.28),
        (radius * 0.52, -half_width * 0.98),
    ]
    return _save_mesh(name, LatheGeometry(profile, segments=56).rotate_y(pi / 2.0))


def _build_wheel(
    part,
    *,
    tire_mesh,
    side_sign: float,
    steel,
    dark_steel,
    rubber,
) -> None:
    spin_rpy = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(Cylinder(radius=0.108, length=0.032), origin=spin_rpy, material=steel, name="rim_barrel")
    part.visual(
        Cylinder(radius=0.088, length=0.012),
        origin=Origin(xyz=(side_sign * 0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
    )
    part.visual(
        Cylinder(radius=0.050, length=0.014),
        origin=Origin(xyz=(-side_sign * 0.0165, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="inner_hub",
    )
    part.visual(
        Cylinder(radius=0.046, length=0.012),
        origin=Origin(xyz=(side_sign * 0.017, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="outer_hub",
    )
    part.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=Origin(xyz=(side_sign * 0.024, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
    )
    bolt_x = side_sign * 0.028
    for bolt_index in range(6):
        angle = 2.0 * pi * bolt_index / 6.0
        part.visual(
            Cylinder(radius=0.0055, length=0.008),
            origin=Origin(
                xyz=(bolt_x, cos(angle) * 0.026, sin(angle) * 0.026),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_wheelie_bin")

    body_green = model.material("body_green", rgba=(0.19, 0.29, 0.18, 1.0))
    lid_green = model.material("lid_green", rgba=(0.22, 0.33, 0.20, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    galvanized = model.material("galvanized", rgba=(0.67, 0.69, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    adapter_gray = model.material("adapter_gray", rgba=(0.53, 0.55, 0.57, 1.0))

    lower_width = 0.44
    upper_width = 0.58
    lower_depth = 0.56
    upper_depth = 0.73
    base_thickness = 0.024
    wall_height = 0.87
    wall_thickness = 0.016
    rim_height = 0.020
    wheel_radius = 0.145
    wheel_width = 0.047

    side_tilt = atan((upper_width - lower_width) / (2.0 * wall_height))
    front_tilt = atan((upper_depth - lower_depth) / (2.0 * wall_height))
    wall_center_z = base_thickness + wall_height * 0.5
    side_wall_x = lower_width * 0.5 + sin(side_tilt) * wall_height * 0.5
    front_wall_y = lower_depth * 0.5 + sin(front_tilt) * wall_height * 0.5
    rim_z = base_thickness + wall_height
    hinge_y = -0.368
    hinge_z = 0.922
    axle_y = -0.248
    axle_z = wheel_radius
    wheel_x = 0.315

    tire_mesh = _wheel_tire_mesh("wheelie_bin_tire", radius=wheel_radius, width=wheel_width)

    body = model.part("bin_body")
    body.inertial = Inertial.from_geometry(
        Box((0.66, 0.82, 0.96)),
        mass=17.5,
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
    )

    body.visual(
        Box((lower_width + 0.02, lower_depth + 0.03, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness * 0.5)),
        material=body_green,
        name="base_pan",
    )
    body.visual(
        Box((upper_width - 0.02, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, front_wall_y, wall_center_z), rpy=(-front_tilt, 0.0, 0.0)),
        material=body_green,
        name="front_wall",
    )
    body.visual(
        Box((upper_width - 0.05, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, -front_wall_y, wall_center_z), rpy=(front_tilt, 0.0, 0.0)),
        material=body_green,
        name="rear_wall",
    )
    body.visual(
        Box((wall_thickness, upper_depth - 0.03, wall_height)),
        origin=Origin(xyz=(side_wall_x, 0.0, wall_center_z), rpy=(0.0, side_tilt, 0.0)),
        material=body_green,
        name="right_wall",
    )
    body.visual(
        Box((wall_thickness, upper_depth - 0.03, wall_height)),
        origin=Origin(xyz=(-side_wall_x, 0.0, wall_center_z), rpy=(0.0, -side_tilt, 0.0)),
        material=body_green,
        name="left_wall",
    )

    body.visual(
        Box((0.54, 0.030, rim_height)),
        origin=Origin(xyz=(0.0, 0.337, rim_z)),
        material=body_green,
        name="front_rim",
    )
    body.visual(
        Box((0.44, 0.036, rim_height)),
        origin=Origin(xyz=(0.0, -0.333, rim_z)),
        material=body_green,
        name="rear_rim",
    )
    body.visual(
        Box((0.030, 0.685, rim_height)),
        origin=Origin(xyz=(0.274, 0.0, rim_z)),
        material=body_green,
        name="right_rim",
    )
    body.visual(
        Box((0.030, 0.685, rim_height)),
        origin=Origin(xyz=(-0.274, 0.0, rim_z)),
        material=body_green,
        name="left_rim",
    )

    body.visual(
        Box((0.52, 0.086, 0.075)),
        origin=Origin(xyz=(0.0, -0.364, 0.872)),
        material=dark_steel,
        name="rear_backbone",
    )
    body.visual(
        Box((0.044, 0.014, 0.118)),
        origin=Origin(xyz=(-0.233, -0.394, 0.902)),
        material=adapter_gray,
        name="left_hinge_cheek",
    )
    body.visual(
        Box((0.044, 0.014, 0.118)),
        origin=Origin(xyz=(0.233, -0.394, 0.902)),
        material=adapter_gray,
        name="right_hinge_cheek",
    )
    body.visual(
        Box((0.40, 0.022, 0.038)),
        origin=Origin(xyz=(0.0, -0.394, 0.928)),
        material=galvanized,
        name="hinge_adapter_bar",
    )

    body.visual(
        Cylinder(radius=0.018, length=0.54),
        origin=Origin(xyz=(0.0, axle_y, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_tube",
    )
    body.visual(
        Box((0.060, 0.080, 0.160)),
        origin=Origin(xyz=(-0.240, -0.286, 0.185)),
        material=dark_steel,
        name="left_axle_gusset",
    )
    body.visual(
        Box((0.060, 0.080, 0.160)),
        origin=Origin(xyz=(0.240, -0.286, 0.185)),
        material=dark_steel,
        name="right_axle_gusset",
    )
    body.visual(
        Box((0.052, 0.060, 0.140)),
        origin=Origin(xyz=(-0.244, -0.234, 0.092)),
        material=dark_steel,
        name="left_lower_brace",
    )
    body.visual(
        Box((0.052, 0.060, 0.140)),
        origin=Origin(xyz=(0.244, -0.234, 0.092)),
        material=dark_steel,
        name="right_lower_brace",
    )

    _add_bolted_plate(
        body,
        center=(-0.276, axle_y, 0.190),
        size=(0.030, 0.082, 0.206),
        normal_axis="x",
        plate_material=adapter_gray,
        bolt_material=galvanized,
        plate_name="left_adapter_plate",
    )
    _add_bolted_plate(
        body,
        center=(0.276, axle_y, 0.190),
        size=(0.030, 0.082, 0.206),
        normal_axis="x",
        plate_material=adapter_gray,
        bolt_material=galvanized,
        plate_name="right_adapter_plate",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.007),
        origin=Origin(xyz=(-0.2880, axle_y, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="left_bearing_face",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.007),
        origin=Origin(xyz=(0.2880, axle_y, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="right_bearing_face",
    )

    _add_bolted_plate(
        body,
        center=(0.0, 0.320, 0.305),
        size=(0.210, 0.010, 0.155),
        normal_axis="y",
        plate_material=adapter_gray,
        bolt_material=galvanized,
        plate_name="front_service_hatch",
    )
    _add_bolted_plate(
        body,
        center=(0.0, -0.318, 0.285),
        size=(0.170, 0.010, 0.135),
        normal_axis="y",
        plate_material=adapter_gray,
        bolt_material=galvanized,
        plate_name="rear_service_hatch",
    )
    body.visual(
        Box((0.090, 0.040, 0.420)),
        origin=Origin(xyz=(0.0, 0.300, 0.470)),
        material=body_green,
        name="front_center_rib",
    )
    body.visual(
        Box((0.045, 0.080, 0.700)),
        origin=Origin(xyz=(-0.233, 0.020, 0.508)),
        material=body_green,
    )
    body.visual(
        Box((0.045, 0.080, 0.700)),
        origin=Origin(xyz=(0.233, 0.020, 0.508)),
        material=body_green,
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.62, 0.76, 0.10)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.36, 0.02)),
    )
    lid.visual(
        Box((0.604, 0.734, 0.007)),
        origin=Origin(xyz=(0.0, 0.367, 0.022)),
        material=lid_green,
        name="lid_skin",
    )
    lid.visual(
        Box((0.010, 0.704, 0.047)),
        origin=Origin(xyz=(-0.297, 0.354, 0.001)),
        material=lid_green,
        name="left_skirt",
    )
    lid.visual(
        Box((0.010, 0.704, 0.047)),
        origin=Origin(xyz=(0.297, 0.354, 0.001)),
        material=lid_green,
        name="right_skirt",
    )
    lid.visual(
        Box((0.562, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.724, 0.015)),
        material=lid_green,
        name="front_lip",
    )
    lid.visual(
        Box((0.492, 0.030, 0.019)),
        origin=Origin(xyz=(0.0, 0.707, -0.0085)),
        material=adapter_gray,
        name="front_seal",
    )
    lid.visual(
        Box((0.060, 0.150, 0.008)),
        origin=Origin(xyz=(-0.195, 0.102, 0.015)),
        material=galvanized,
        name="left_hinge_strap",
    )
    lid.visual(
        Box((0.060, 0.150, 0.008)),
        origin=Origin(xyz=(0.195, 0.102, 0.015)),
        material=galvanized,
        name="right_hinge_strap",
    )
    lid.visual(
        Box((0.500, 0.038, 0.026)),
        origin=Origin(xyz=(0.0, 0.026, 0.006)),
        material=galvanized,
        name="rear_hinge_bridge",
    )
    lid.visual(
        Box((0.500, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, 0.360, 0.006)),
        material=lid_green,
        name="center_rib",
    )
    lid.visual(
        Box((0.020, 0.420, 0.024)),
        origin=Origin(xyz=(-0.160, 0.380, 0.008)),
        material=lid_green,
        name="left_lid_stiffener",
    )
    lid.visual(
        Box((0.020, 0.420, 0.024)),
        origin=Origin(xyz=(0.160, 0.380, 0.008)),
        material=lid_green,
        name="right_lid_stiffener",
    )
    lid.visual(
        Box((0.280, 0.024, 0.028)),
        origin=Origin(xyz=(0.0, 0.610, 0.006)),
        material=lid_green,
        name="pull_rib",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.9,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_wheel(
        left_wheel,
        tire_mesh=tire_mesh,
        side_sign=-1.0,
        steel=galvanized,
        dark_steel=dark_steel,
        rubber=wheel_rubber,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.9,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_wheel(
        right_wheel,
        tire_mesh=tire_mesh,
        side_sign=1.0,
        steel=galvanized,
        dark_steel=dark_steel,
        rubber=wheel_rubber,
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-wheel_x, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(wheel_x, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bin_body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    lid_hinge = object_model.get_articulation("body_to_lid")
    left_spin = object_model.get_articulation("body_to_left_wheel")
    right_spin = object_model.get_articulation("body_to_right_wheel")

    front_rim = body.get_visual("front_rim")
    left_bearing_face = body.get_visual("left_bearing_face")
    right_bearing_face = body.get_visual("right_bearing_face")
    lid_skin = lid.get_visual("lid_skin")
    front_seal = lid.get_visual("front_seal")
    left_inner_hub = left_wheel.get_visual("inner_hub")
    right_inner_hub = right_wheel.get_visual("inner_hub")

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
        "primary_axes_are_correct",
        lid_hinge.axis == (1.0, 0.0, 0.0)
        and left_spin.axis == (1.0, 0.0, 0.0)
        and right_spin.axis == (1.0, 0.0, 0.0),
        "Lid hinge and both wheel joints should articulate around the bin width axis.",
    )
    ctx.expect_contact(
        left_wheel,
        body,
        elem_a=left_inner_hub,
        elem_b=left_bearing_face,
        name="left_wheel_supported_by_adapter",
    )
    ctx.expect_contact(
        right_wheel,
        body,
        elem_a=right_inner_hub,
        elem_b=right_bearing_face,
        name="right_wheel_supported_by_adapter",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=front_seal,
            negative_elem=front_rim,
            max_gap=0.010,
            max_penetration=0.001,
            name="lid_closes_down_to_front_rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=lid_skin,
            min_overlap=0.38,
            name="lid_covers_shell_opening",
        )

    with ctx.pose({lid_hinge: 1.15}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=front_seal,
            negative_elem=front_rim,
            min_gap=0.18,
            name="lid_front_edge_lifts_clear_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
