from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_antenna_router")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    soft_black = model.material("soft_black", rgba=(0.035, 0.037, 0.040, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.09, 0.095, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    port_black = model.material("port_black", rgba=(0.002, 0.002, 0.003, 1.0))
    metal = model.material("brushed_metal", rgba=(0.62, 0.64, 0.62, 1.0))
    label_blue = model.material("wan_blue", rgba=(0.0, 0.18, 0.85, 1.0))
    led_green = model.material("led_green", rgba=(0.0, 0.95, 0.23, 1.0))
    led_amber = model.material("led_amber", rgba=(1.0, 0.55, 0.05, 1.0))
    led_blue = model.material("led_blue", rgba=(0.05, 0.35, 1.0, 1.0))
    white_print = model.material("white_print", rgba=(0.86, 0.88, 0.86, 1.0))

    body_width = 0.300
    body_depth = 0.180
    body_height = 0.048
    body_bottom = 0.006
    body_top = body_bottom + body_height

    shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(body_width, body_depth, 0.026, corner_segments=10),
            body_height,
            cap=True,
            center=True,
        ),
        "rounded_router_shell",
    )

    vent_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.190, 0.055),
            0.003,
            slot_size=(0.030, 0.0045),
            pitch=(0.041, 0.012),
            frame=0.010,
            corner_radius=0.006,
            stagger=True,
        ),
        "top_vent_slots",
    )

    antenna_blade_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.018, 0.007, 0.0032, corner_segments=5),
            0.145,
            cap=True,
            center=True,
        ),
        "flat_antenna_blade",
    )

    housing = model.part("housing")
    housing.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, body_bottom + body_height / 2.0)),
        material=matte_black,
        name="main_shell",
    )
    housing.visual(
        Box((0.238, 0.132, 0.003)),
        origin=Origin(xyz=(0.0, -0.004, body_top + 0.0010)),
        material=soft_black,
        name="top_inset",
    )
    housing.visual(
        vent_mesh,
        origin=Origin(xyz=(0.0, -0.014, body_top + 0.0028)),
        material=dark_grey,
        name="top_vents",
    )
    housing.visual(
        Box((0.055, 0.010, 0.0012)),
        origin=Origin(xyz=(0.0, -0.063, body_top + 0.0027)),
        material=white_print,
        name="brand_mark",
    )

    # Four low rubber feet raise the rounded plastic case off the desk.
    for index, (x, y) in enumerate(
        ((-0.115, -0.064), (0.115, -0.064), (-0.115, 0.064), (0.115, 0.064))
    ):
        housing.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, 0.003)),
            material=rubber,
            name=f"foot_{index}",
        )

    # Front status light pipe: power, internet, Wi-Fi bands, and activity.
    led_xs = (-0.070, -0.045, -0.020, 0.005, 0.030, 0.055, 0.080)
    led_mats = (led_green, led_green, led_blue, led_blue, led_green, led_amber, led_green)
    for index, (x, mat) in enumerate(zip(led_xs, led_mats)):
        housing.visual(
            Box((0.010, 0.0035, 0.0055)),
            origin=Origin(xyz=(x, -body_depth / 2.0 - 0.0014, 0.031)),
            material=mat,
            name=f"front_led_{index}",
        )

    # Rear I/O field with Ethernet jacks, USB, DC barrel jack, and cable labels.
    housing.visual(
        Box((0.214, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, body_depth / 2.0 + 0.0022, 0.030)),
        material=dark_grey,
        name="rear_panel",
    )
    port_centers = (-0.075, -0.044, -0.013, 0.018)
    for index, x in enumerate(port_centers):
        housing.visual(
            Box((0.024, 0.005, 0.016)),
            origin=Origin(xyz=(x, body_depth / 2.0 + 0.0060, 0.030)),
            material=port_black,
            name=f"lan_port_{index}",
        )
        housing.visual(
            Box((0.019, 0.0016, 0.003)),
            origin=Origin(xyz=(x, body_depth / 2.0 + 0.0090, 0.0385)),
            material=metal,
            name=f"lan_contact_{index}",
        )
    housing.visual(
        Box((0.026, 0.005, 0.017)),
        origin=Origin(xyz=(0.054, body_depth / 2.0 + 0.0060, 0.030)),
        material=label_blue,
        name="wan_port",
    )
    housing.visual(
        Box((0.022, 0.005, 0.013)),
        origin=Origin(xyz=(0.087, body_depth / 2.0 + 0.0060, 0.030)),
        material=metal,
        name="usb_port",
    )
    housing.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(
            xyz=(0.119, body_depth / 2.0 + 0.0020, 0.030),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material=port_black,
        name="power_jack",
    )

    # Rear hinge sockets.  Each antenna barrel rests on a short stem and is
    # flanked by two yoke cheeks, giving the antennas a credible pivot mount.
    antenna_xs = (-0.115, -0.038, 0.038, 0.115)
    antenna_y = body_depth / 2.0 + 0.008
    socket_stem_height = 0.010
    barrel_radius = 0.009
    joint_z = body_top + socket_stem_height + barrel_radius
    for index, x in enumerate(antenna_xs):
        housing.visual(
            Cylinder(radius=0.0085, length=socket_stem_height),
            origin=Origin(xyz=(x, antenna_y, body_top + socket_stem_height / 2.0)),
            material=dark_grey,
            name=f"socket_stem_{index}",
        )
        housing.visual(
            Box((0.042, 0.018, 0.006)),
            origin=Origin(xyz=(x, antenna_y, joint_z - 0.012)),
            material=dark_grey,
            name=f"socket_bridge_{index}",
        )
        for side, dx in (("a", -0.018), ("b", 0.018)):
            housing.visual(
                Box((0.006, 0.020, 0.020)),
                origin=Origin(xyz=(x + dx, antenna_y, joint_z)),
                material=dark_grey,
                name=f"socket_yoke_{index}_{side}",
            )

    # The two visible push buttons are separate prismatic parts so the WPS/reset
    # controls can depress into the housing like real router tact switches.
    for name, x, y, radius in (
        ("wps_button", -0.120, -0.024, 0.009),
        ("reset_button", 0.120, -0.024, 0.006),
    ):
        button = model.part(name)
        button.visual(
            Cylinder(radius=radius, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=soft_black,
            name="button_cap",
        )
        button.visual(
            Box((radius * 1.4, 0.0012, 0.001)),
            origin=Origin(xyz=(0.0, -radius * 0.20, 0.0058)),
            material=white_print,
            name="button_glyph",
        )
        model.articulation(
            f"housing_to_{name}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(x, y, body_top)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=0.004),
        )

    for index, x in enumerate(antenna_xs):
        antenna = model.part(f"antenna_{index}")
        antenna.visual(
            Cylinder(radius=barrel_radius, length=0.026),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=soft_black,
            name="barrel",
        )
        antenna.visual(
            Cylinder(radius=0.0058, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=soft_black,
            name="neck",
        )
        antenna.visual(
            antenna_blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.096)),
            material=soft_black,
            name="blade",
        )
        antenna.visual(
            Box((0.014, 0.006, 0.018)),
            origin=Origin(xyz=(0.0, -0.0008, 0.035)),
            material=matte_black,
            name="blade_boot",
        )
        model.articulation(
            f"housing_to_antenna_{index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=antenna,
            origin=Origin(xyz=(x, antenna_y, joint_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.6,
                velocity=1.2,
                lower=-1.10,
                upper=1.10,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")

    antenna_parts = [object_model.get_part(f"antenna_{i}") for i in range(4)]
    antenna_joints = [object_model.get_articulation(f"housing_to_antenna_{i}") for i in range(4)]
    ctx.check(
        "router_has_four_antennas",
        all(part is not None for part in antenna_parts)
        and all(joint is not None for joint in antenna_joints),
        details="Expected four separate articulated antenna parts and hinge joints.",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) / 2.0,
            (lo[1] + hi[1]) / 2.0,
            (lo[2] + hi[2]) / 2.0,
        )

    for index, (antenna, joint) in enumerate(zip(antenna_parts, antenna_joints)):
        ctx.expect_contact(
            antenna,
            housing,
            elem_a="barrel",
            elem_b=f"socket_stem_{index}",
            contact_tol=0.0015,
            name=f"antenna_{index}_barrel_sits_on_socket",
        )
        ctx.expect_overlap(
            antenna,
            housing,
            axes="xy",
            elem_a="barrel",
            elem_b=f"socket_stem_{index}",
            min_overlap=0.004,
            name=f"antenna_{index}_barrel_centered_on_socket",
        )
        rest_center = _aabb_center(ctx.part_element_world_aabb(antenna, elem="blade"))
        with ctx.pose({joint: 0.85}):
            tilted_center = _aabb_center(ctx.part_element_world_aabb(antenna, elem="blade"))
        ctx.check(
            f"antenna_{index}_hinge_tilts_forward",
            rest_center is not None
            and tilted_center is not None
            and tilted_center[1] < rest_center[1] - 0.045
            and tilted_center[2] < rest_center[2] - 0.020,
            details=f"rest={rest_center}, tilted={tilted_center}",
        )

    for name in ("wps_button", "reset_button"):
        button = object_model.get_part(name)
        joint = object_model.get_articulation(f"housing_to_{name}")
        ctx.expect_contact(
            button,
            housing,
            elem_a="button_cap",
            elem_b="main_shell",
            contact_tol=0.0015,
            name=f"{name}_rests_on_top_shell",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.004}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{name}_presses_down",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
