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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _permute_extrusion_axes(geometry):
    remapped = geometry.copy()
    remapped.vertices[:] = [(z, x, y) for x, y, z in remapped.vertices]
    return remapped


def _build_lid_shell_mesh(name: str, *, width: float, depth: float, height: float):
    outer_curve = sample_catmull_rom_spline_2d(
        [
            (-depth + 0.035, 0.030),
            (-depth * 0.77, 0.098),
            (-depth * 0.46, height),
            (-depth * 0.14, height * 0.90),
            (0.014, 0.028),
        ],
        samples_per_segment=10,
    )
    inner_curve = sample_catmull_rom_spline_2d(
        [
            (-depth + 0.060, 0.022),
            (-depth * 0.72, 0.074),
            (-depth * 0.45, height - 0.018),
            (-depth * 0.17, height * 0.73),
            (0.000, 0.008),
        ],
        samples_per_segment=10,
    )
    profile = [
        (-depth + 0.035, -0.012),
        (-depth + 0.035, 0.030),
        *outer_curve[1:],
        (0.014, -0.018),
        (0.000, -0.018),
        *list(reversed(inner_curve)),
        (-depth + 0.060, 0.000),
    ]
    shell = ExtrudeGeometry.centered(profile, width, cap=True, closed=True)
    return _save_mesh(name, _permute_extrusion_axes(shell))


def _build_handle_mesh(name: str, *, span: float):
    handle = wire_from_points(
        [
            (-span * 0.5, 0.0, 0.0),
            (-span * 0.5, -0.030, 0.024),
            (span * 0.5, -0.030, 0.024),
            (span * 0.5, 0.0, 0.0),
        ],
        radius=0.0055,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.020,
        corner_segments=10,
    )
    return _save_mesh(name, handle)


def _add_wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    tire_radius: float,
    tire_width: float,
    wheel_black,
    wheel_steel,
):
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.54, -half_width * 0.98),
        (tire_radius * 0.76, -half_width),
        (tire_radius * 0.95, -half_width * 0.68),
        (tire_radius, -half_width * 0.20),
        (tire_radius, half_width * 0.20),
        (tire_radius * 0.95, half_width * 0.68),
        (tire_radius * 0.76, half_width),
        (tire_radius * 0.54, half_width * 0.98),
        (tire_radius * 0.44, half_width * 0.38),
        (tire_radius * 0.40, 0.0),
        (tire_radius * 0.44, -half_width * 0.38),
        (tire_radius * 0.54, -half_width * 0.98),
    ]
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire",
        LatheGeometry(tire_profile, segments=56).rotate_y(math.pi / 2.0),
    )
    part.visual(tire_mesh, material=wheel_black, name="tire")
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius * 0.56, length=tire_width),
        origin=spin_origin,
        material=wheel_steel,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.35, length=tire_width * 0.62),
        origin=spin_origin,
        material=wheel_black,
        name="hub_cap",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.09, length=tire_width * 0.70),
        origin=spin_origin,
        material=wheel_steel,
        name="axle_bore_trim",
    )


def _add_lid_part(
    model: ArticulatedObject,
    *,
    name: str,
    lid_finish,
    handle_steel,
    outer_sign: float,
    width: float,
    depth: float,
    height: float,
):
    lid = model.part(name)
    lid.visual(
        _build_lid_shell_mesh(f"{name}_shell", width=width - 0.010, depth=depth, height=height),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=lid_finish,
        name="shell",
    )
    cap_size = (0.012, depth * 0.82, height * 0.82)
    cap_y = -depth * 0.43
    cap_z = cap_size[2] * 0.5
    x_offset = width * 0.5 - cap_size[0] * 0.5
    lid.visual(
        Box(cap_size),
        origin=Origin(xyz=(outer_sign * x_offset, cap_y, cap_z)),
        material=lid_finish,
        name="outer_end_cap",
    )
    lid.visual(
        Box(cap_size),
        origin=Origin(xyz=(-outer_sign * x_offset, cap_y, cap_z)),
        material=lid_finish,
        name="inner_end_cap",
    )
    lid.visual(
        _build_handle_mesh(f"{name}_handle", span=width * 0.62),
        origin=Origin(xyz=(0.0, -depth + 0.060, 0.054)),
        material=handle_steel,
        name="handle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=5.0,
        origin=Origin(xyz=(0.0, -depth * 0.44, height * 0.44)),
    )
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_zone_gas_grill")

    cart_charcoal = model.material("cart_charcoal", rgba=(0.13, 0.13, 0.14, 1.0))
    lid_black = model.material("lid_black", rgba=(0.18, 0.18, 0.19, 1.0))
    cookbox_black = model.material("cookbox_black", rgba=(0.11, 0.11, 0.12, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.05, 0.05, 0.05, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.35, 0.37, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    control_silver = model.material("control_silver", rgba=(0.80, 0.81, 0.83, 1.0))

    cart = model.part("cart")
    cart.visual(
        Box((0.92, 0.42, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
        material=cart_charcoal,
        name="lower_shelf",
    )
    for x_sign in (-1.0, 1.0):
        for y_pos, leg_name in ((-0.20, "front"), (0.20, "rear")):
            cart.visual(
                Box((0.04, 0.04, 0.62)),
                origin=Origin(xyz=(x_sign * 0.48, y_pos, 0.31)),
                material=cart_charcoal,
                name=f"{leg_name}_{'right' if x_sign > 0 else 'left'}_leg",
            )
    cart.visual(
        Box((1.00, 0.03, 0.14)),
        origin=Origin(xyz=(0.0, -0.205, 0.49)),
        material=cart_charcoal,
        name="front_apron",
    )
    cart.visual(
        Box((0.90, 0.01, 0.33)),
        origin=Origin(xyz=(0.0, -0.195, 0.255)),
        material=cart_charcoal,
        name="front_lower_panel",
    )
    cart.visual(
        Box((0.012, 0.40, 0.40)),
        origin=Origin(xyz=(-0.454, 0.0, 0.30)),
        material=cart_charcoal,
        name="left_side_panel",
    )
    cart.visual(
        Box((0.012, 0.40, 0.40)),
        origin=Origin(xyz=(0.454, 0.0, 0.30)),
        material=cart_charcoal,
        name="right_side_panel",
    )
    cart.visual(
        Box((1.04, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, -0.20, 0.595)),
        material=cart_charcoal,
        name="front_upper_rail",
    )
    cart.visual(
        Box((1.04, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.20, 0.595)),
        material=cart_charcoal,
        name="rear_upper_rail",
    )
    cart.visual(
        Box((0.04, 0.42, 0.05)),
        origin=Origin(xyz=(-0.52, 0.0, 0.595)),
        material=cart_charcoal,
        name="left_upper_rail",
    )
    cart.visual(
        Box((0.04, 0.42, 0.05)),
        origin=Origin(xyz=(0.52, 0.0, 0.595)),
        material=cart_charcoal,
        name="right_upper_rail",
    )
    cart.visual(
        Box((1.12, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.23, 0.18)),
        material=dark_steel,
        name="axle_cross_brace",
    )
    cart.visual(
        Box((0.016, 0.08, 0.14)),
        origin=Origin(xyz=(-0.552, 0.23, 0.145)),
        material=dark_steel,
        name="left_wheel_bracket",
    )
    cart.visual(
        Box((0.016, 0.08, 0.14)),
        origin=Origin(xyz=(0.552, 0.23, 0.145)),
        material=dark_steel,
        name="right_wheel_bracket",
    )
    cart.inertial = Inertial.from_geometry(
        Box((1.16, 0.50, 0.62)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
    )

    cookbox = model.part("cookbox")
    cookbox.visual(
        Box((1.08, 0.54, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cookbox_black,
        name="bottom_pan",
    )
    cookbox.visual(
        Box((1.00, 0.34, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=cookbox_black,
        name="lower_belly",
    )
    cookbox.visual(
        Box((1.08, 0.012, 0.158)),
        origin=Origin(xyz=(0.0, -0.264, 0.079)),
        material=cookbox_black,
        name="front_wall",
    )
    cookbox.visual(
        Box((1.08, 0.012, 0.162)),
        origin=Origin(xyz=(0.0, 0.264, 0.081)),
        material=cookbox_black,
        name="rear_wall",
    )
    cookbox.visual(
        Box((0.012, 0.54, 0.162)),
        origin=Origin(xyz=(-0.534, 0.0, 0.081)),
        material=cookbox_black,
        name="left_wall",
    )
    cookbox.visual(
        Box((0.012, 0.54, 0.162)),
        origin=Origin(xyz=(0.534, 0.0, 0.081)),
        material=cookbox_black,
        name="right_wall",
    )
    cookbox.visual(
        Box((0.018, 0.52, 0.156)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=cookbox_black,
        name="center_divider",
    )
    cookbox.inertial = Inertial.from_geometry(
        Box((1.10, 0.56, 0.22)),
        mass=17.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((1.06, 0.06, 0.10)),
        material=control_silver,
        name="panel_body",
    )
    control_panel.visual(
        Box((1.08, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.036, -0.041)),
        material=dark_steel,
        name="panel_face_trim",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((1.08, 0.06, 0.10)),
        mass=4.0,
    )

    lid_width = 0.524
    lid_depth = 0.29
    lid_height = 0.176
    left_lid = _add_lid_part(
        model,
        name="left_lid",
        lid_finish=lid_black,
        handle_steel=steel,
        outer_sign=-1.0,
        width=lid_width,
        depth=lid_depth,
        height=lid_height,
    )
    right_lid = _add_lid_part(
        model,
        name="right_lid",
        lid_finish=lid_black,
        handle_steel=steel,
        outer_sign=1.0,
        width=lid_width,
        depth=lid_depth,
        height=lid_height,
    )

    for side_name, x_pos in (("left", -0.585), ("right", 0.585)):
        wheel = model.part(f"{side_name}_wheel")
        _add_wheel_visuals(
            wheel,
            mesh_prefix=f"{side_name}_grill_wheel",
            tire_radius=0.145,
            tire_width=0.050,
            wheel_black=wheel_black,
            wheel_steel=dark_steel,
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.145, length=0.050),
            mass=2.3,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        model.articulation(
            f"cart_to_{side_name}_wheel",
            ArticulationType.CONTINUOUS,
            parent=cart,
            child=wheel,
            origin=Origin(xyz=(x_pos, 0.23, 0.145)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    knob_x_positions = (-0.33, -0.11, 0.11, 0.33)
    for index, knob_x in enumerate(knob_x_positions, start=1):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.030, length=0.004),
            origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="mount_flange",
        )
        knob.visual(
            Cylinder(radius=0.028, length=0.028),
            origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.020, length=0.016),
            origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name="front_cap",
        )
        knob.visual(
            Box((0.004, 0.010, 0.012)),
            origin=Origin(xyz=(0.0, -0.034, 0.017)),
            material=steel,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.030, length=0.040),
            mass=0.10,
            origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"control_panel_to_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=control_panel,
            child=knob,
            origin=Origin(xyz=(knob_x, -0.030, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=4.0,
                lower=0.0,
                upper=math.radians(300.0),
            ),
        )

    model.articulation(
        "cart_to_cookbox",
        ArticulationType.FIXED,
        parent=cart,
        child=cookbox,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
    )
    model.articulation(
        "cart_to_control_panel",
        ArticulationType.FIXED,
        parent=cart,
        child=control_panel,
        origin=Origin(xyz=(0.0, -0.25, 0.57)),
    )
    model.articulation(
        "cookbox_to_left_lid",
        ArticulationType.REVOLUTE,
        parent=cookbox,
        child=left_lid,
        origin=Origin(xyz=(-0.271, 0.272, 0.162)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "cookbox_to_right_lid",
        ArticulationType.REVOLUTE,
        parent=cookbox,
        child=right_lid,
        origin=Origin(xyz=(0.271, 0.272, 0.162)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cart = object_model.get_part("cart")
    cookbox = object_model.get_part("cookbox")
    control_panel = object_model.get_part("control_panel")
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    knobs = [object_model.get_part(f"knob_{index}") for index in range(1, 5)]

    left_hinge = object_model.get_articulation("cookbox_to_left_lid")
    right_hinge = object_model.get_articulation("cookbox_to_right_lid")
    left_wheel_spin = object_model.get_articulation("cart_to_left_wheel")
    right_wheel_spin = object_model.get_articulation("cart_to_right_wheel")
    knob_joints = [object_model.get_articulation(f"control_panel_to_knob_{index}") for index in range(1, 5)]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(cookbox, cart, name="cookbox sits on cart frame")
    ctx.expect_contact(control_panel, cart, name="control panel mounts to cart")

    ctx.expect_contact(left_lid, cookbox, elem_a="outer_end_cap", name="left lid outer edge sits on cookbox")
    ctx.expect_contact(left_lid, cookbox, elem_a="inner_end_cap", name="left lid inner edge sits on divider")
    ctx.expect_contact(right_lid, cookbox, elem_a="outer_end_cap", name="right lid outer edge sits on cookbox")
    ctx.expect_contact(right_lid, cookbox, elem_a="inner_end_cap", name="right lid inner edge sits on divider")
    ctx.expect_overlap(left_lid, cookbox, axes="xy", min_overlap=0.20, name="left lid covers left cooking zone")
    ctx.expect_overlap(right_lid, cookbox, axes="xy", min_overlap=0.20, name="right lid covers right cooking zone")

    ctx.expect_contact(left_wheel, cart, elem_a="hub_shell", name="left wheel bears on bracket")
    ctx.expect_contact(right_wheel, cart, elem_a="hub_shell", name="right wheel bears on bracket")

    for index, knob in enumerate(knobs, start=1):
        ctx.expect_contact(knob, control_panel, elem_a="mount_flange", name=f"knob {index} mounts on control panel")
        ctx.expect_within(knob, control_panel, axes="xz", margin=0.03, name=f"knob {index} stays within panel layout")

    def _axis_is_x(axis) -> bool:
        return abs(abs(axis[0]) - 1.0) < 1e-9 and abs(axis[1]) < 1e-9 and abs(axis[2]) < 1e-9

    def _axis_is_y(axis) -> bool:
        return abs(abs(axis[1]) - 1.0) < 1e-9 and abs(axis[0]) < 1e-9 and abs(axis[2]) < 1e-9

    ctx.check(
        "lid hinges use parallel rear x-axes",
        _axis_is_x(left_hinge.axis) and _axis_is_x(right_hinge.axis),
        details=f"left={left_hinge.axis}, right={right_hinge.axis}",
    )
    ctx.check(
        "wheel axles spin about x-axis",
        _axis_is_x(left_wheel_spin.axis) and _axis_is_x(right_wheel_spin.axis),
        details=f"left={left_wheel_spin.axis}, right={right_wheel_spin.axis}",
    )
    ctx.check(
        "wheel articulations are continuous",
        left_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"left={left_wheel_spin.articulation_type}, right={right_wheel_spin.articulation_type}",
    )
    ctx.check(
        "knob articulations rotate on panel-normal axes",
        all(_axis_is_y(joint.axis) for joint in knob_joints),
        details=", ".join(str(joint.axis) for joint in knob_joints),
    )
    ctx.check(
        "lid opening range is grill-like",
        left_hinge.motion_limits is not None
        and right_hinge.motion_limits is not None
        and left_hinge.motion_limits.lower == 0.0
        and right_hinge.motion_limits.lower == 0.0
        and left_hinge.motion_limits.upper is not None
        and right_hinge.motion_limits.upper is not None
        and left_hinge.motion_limits.upper > 1.2
        and right_hinge.motion_limits.upper > 1.2,
        details=f"left={left_hinge.motion_limits}, right={right_hinge.motion_limits}",
    )

    left_closed_aabb = ctx.part_world_aabb(left_lid)
    if left_closed_aabb is None:
        ctx.fail("left lid has measurable geometry", "left_lid produced no world-space AABB")
    else:
        with ctx.pose({left_hinge: math.radians(72.0)}):
            left_open_aabb = ctx.part_world_aabb(left_lid)
            if left_open_aabb is None:
                ctx.fail("left lid opens into measurable pose", "open left_lid pose produced no AABB")
            else:
                ctx.check(
                    "left lid lifts upward",
                    left_open_aabb[1][2] > left_closed_aabb[1][2] + 0.10,
                    details=f"closed_max_z={left_closed_aabb[1][2]:.4f}, open_max_z={left_open_aabb[1][2]:.4f}",
                )
            ctx.expect_contact(
                right_lid,
                cookbox,
                elem_a="outer_end_cap",
                name="right lid stays seated while left lid opens",
            )

    right_closed_aabb = ctx.part_world_aabb(right_lid)
    if right_closed_aabb is None:
        ctx.fail("right lid has measurable geometry", "right_lid produced no world-space AABB")
    else:
        with ctx.pose({right_hinge: math.radians(72.0)}):
            right_open_aabb = ctx.part_world_aabb(right_lid)
            if right_open_aabb is None:
                ctx.fail("right lid opens into measurable pose", "open right_lid pose produced no AABB")
            else:
                ctx.check(
                    "right lid lifts upward",
                    right_open_aabb[1][2] > right_closed_aabb[1][2] + 0.10,
                    details=f"closed_max_z={right_closed_aabb[1][2]:.4f}, open_max_z={right_open_aabb[1][2]:.4f}",
                )
            ctx.expect_contact(
                left_lid,
                cookbox,
                elem_a="outer_end_cap",
                name="left lid stays seated while right lid opens",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
