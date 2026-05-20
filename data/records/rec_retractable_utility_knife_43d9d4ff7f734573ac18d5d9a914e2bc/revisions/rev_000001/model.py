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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_utility_knife")

    shell_dark = model.material("shell_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    slider_orange = model.material("slider_orange", rgba=(0.88, 0.44, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.79, 0.81, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.32, 0.33, 0.36, 1.0))

    def add_box(part, name, size, center, material) -> None:
        part.visual(
            Box(size),
            origin=Origin(xyz=center),
            material=material,
            name=name,
        )

    def add_y_cylinder(part, name, radius, length, center, material) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    handle_shell = model.part("handle_shell")
    add_box(handle_shell, "bottom_spine", (0.158, 0.006, 0.022), (0.079, 0.003, 0.0), shell_dark)
    add_box(handle_shell, "left_wall", (0.166, 0.028, 0.0025), (0.083, 0.020, -0.00975), shell_dark)
    add_box(handle_shell, "right_front_wall", (0.032, 0.028, 0.0025), (0.016, 0.020, 0.00975), shell_dark)
    add_box(handle_shell, "right_rear_wall", (0.093, 0.028, 0.0025), (0.1285, 0.020, 0.00975), shell_dark)
    add_box(handle_shell, "hatch_sill", (0.050, 0.005, 0.0025), (0.057, 0.0085, 0.00975), shell_dark)
    add_box(handle_shell, "hatch_lintel", (0.050, 0.008, 0.0025), (0.057, 0.032, 0.00975), shell_dark)
    add_box(handle_shell, "rear_bridge", (0.016, 0.026, 0.017), (0.008, 0.019, 0.0), shell_dark)
    add_box(handle_shell, "slot_rear_bridge", (0.014, 0.004, 0.022), (0.046, 0.032, 0.0), shell_dark)
    add_box(handle_shell, "top_slot_left_rail", (0.092, 0.004, 0.0065), (0.095, 0.032, -0.00775), shell_dark)
    add_box(handle_shell, "top_slot_right_rail", (0.092, 0.004, 0.0065), (0.095, 0.032, 0.00775), shell_dark)
    add_box(handle_shell, "nose_floor", (0.022, 0.004, 0.022), (0.168, 0.002, 0.0), shell_dark)
    add_box(handle_shell, "nose_left_cheek", (0.030, 0.016, 0.0045), (0.168, 0.014, -0.00875), shell_dark)
    add_box(handle_shell, "nose_right_cheek", (0.030, 0.016, 0.0045), (0.168, 0.014, 0.00875), shell_dark)
    add_box(handle_shell, "nose_bridge", (0.024, 0.008, 0.022), (0.158, 0.030, 0.0), shell_dark)
    add_y_cylinder(handle_shell, "hinge_lower_ear", 0.0022, 0.005, (0.082, 0.0115, 0.0123), dark_metal)
    add_y_cylinder(handle_shell, "hinge_upper_ear", 0.0022, 0.005, (0.082, 0.0265, 0.0123), dark_metal)
    add_box(handle_shell, "hatch_tray", (0.046, 0.003, 0.008), (0.057, 0.0075, 0.0045), dark_metal)
    add_box(handle_shell, "spare_blades", (0.035, 0.014, 0.0016), (0.056, 0.016, 0.0068), steel)
    add_box(handle_shell, "left_grip_pad", (0.086, 0.012, 0.0014), (0.084, 0.018, -0.0117), rubber_black)
    add_box(handle_shell, "right_front_grip_pad", (0.022, 0.012, 0.0014), (0.017, 0.018, 0.0117), rubber_black)
    add_box(handle_shell, "right_rear_grip_pad", (0.048, 0.012, 0.0014), (0.131, 0.018, 0.0117), rubber_black)
    handle_shell.inertial = Inertial.from_geometry(
        Box((0.185, 0.036, 0.022)),
        mass=0.45,
        origin=Origin(xyz=(0.0925, 0.018, 0.0)),
    )

    blade_carrier = model.part("blade_carrier")
    add_box(blade_carrier, "carrier_base", (0.100, 0.004, 0.008), (0.050, 0.008, 0.0), dark_metal)
    add_box(blade_carrier, "carrier_clamp", (0.022, 0.010, 0.009), (0.102, 0.011, 0.0), dark_metal)
    add_box(blade_carrier, "thumb_stem", (0.010, 0.026, 0.0035), (0.048, 0.021, 0.0), dark_metal)
    add_box(blade_carrier, "thumb_slider", (0.018, 0.006, 0.008), (0.048, 0.035, 0.0), slider_orange)
    add_box(blade_carrier, "thumb_ridge_front", (0.002, 0.0015, 0.0075), (0.043, 0.03825, 0.0), slider_orange)
    add_box(blade_carrier, "thumb_ridge_mid", (0.002, 0.0015, 0.0075), (0.048, 0.03825, 0.0), slider_orange)
    add_box(blade_carrier, "thumb_ridge_rear", (0.002, 0.0015, 0.0075), (0.053, 0.03825, 0.0), slider_orange)
    add_box(blade_carrier, "utility_blade", (0.032, 0.018, 0.0008), (0.126, 0.014, 0.0), steel)
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.140, 0.040, 0.012)),
        mass=0.08,
        origin=Origin(xyz=(0.076, 0.018, 0.0)),
    )

    side_hatch = model.part("side_hatch")
    add_box(side_hatch, "hatch_panel", (0.042, 0.016, 0.0018), (-0.026, 0.0, -0.0022), shell_dark)
    add_box(side_hatch, "hatch_hinge_leaf", (0.006, 0.010, 0.0036), (-0.0035, 0.0, -0.0022), dark_metal)
    add_y_cylinder(side_hatch, "hatch_knuckle", 0.0022, 0.010, (0.0, 0.0, 0.0), dark_metal)
    side_hatch.inertial = Inertial.from_geometry(
        Box((0.045, 0.020, 0.006)),
        mass=0.03,
        origin=Origin(xyz=(-0.024, 0.0, -0.0022)),
    )

    model.articulation(
        "handle_to_blade_carrier",
        ArticulationType.PRISMATIC,
        parent=handle_shell,
        child=blade_carrier,
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.028),
    )

    model.articulation(
        "handle_to_side_hatch",
        ArticulationType.REVOLUTE,
        parent=handle_shell,
        child=side_hatch,
        origin=Origin(xyz=(0.082, 0.019, 0.0123)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle_shell = object_model.get_part("handle_shell")
    blade_carrier = object_model.get_part("blade_carrier")
    side_hatch = object_model.get_part("side_hatch")

    carrier_joint = object_model.get_articulation("handle_to_blade_carrier")
    hatch_joint = object_model.get_articulation("handle_to_side_hatch")

    carrier_base = blade_carrier.get_visual("carrier_base")
    thumb_slider = blade_carrier.get_visual("thumb_slider")
    utility_blade = blade_carrier.get_visual("utility_blade")
    hatch_panel = side_hatch.get_visual("hatch_panel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(blade_carrier, handle_shell, name="blade carrier remains supported in guide")
    ctx.expect_contact(side_hatch, handle_shell, name="side hatch remains clipped into handle shell")
    ctx.expect_within(
        blade_carrier,
        handle_shell,
        axes="yz",
        inner_elem=carrier_base,
        name="carrier base stays within straight guide cross-section",
    )
    ctx.expect_within(
        side_hatch,
        handle_shell,
        axes="xy",
        inner_elem=hatch_panel,
        name="closed hatch panel stays within side opening footprint",
    )

    carrier_rest = ctx.part_world_position(blade_carrier)
    slider_rest = ctx.part_element_world_aabb(blade_carrier, elem="thumb_slider")
    blade_rest = ctx.part_element_world_aabb(blade_carrier, elem="utility_blade")
    hatch_closed = ctx.part_element_world_aabb(side_hatch, elem="hatch_panel")
    handle_aabb = ctx.part_world_aabb(handle_shell)
    if (
        carrier_rest is None
        or slider_rest is None
        or blade_rest is None
        or hatch_closed is None
        or handle_aabb is None
    ):
        ctx.fail("world-space measurements available", "Expected current-pose measurements for key visuals.")
        return ctx.report()

    ctx.check(
        "spare-blade hatch sits behind slider",
        hatch_closed[1][0] < slider_rest[0][0] - 0.003,
        details=(
            f"hatch max x={hatch_closed[1][0]:.4f}, "
            f"slider min x={slider_rest[0][0]:.4f}"
        ),
    )

    with ctx.pose({carrier_joint: 0.028}):
        carrier_open = ctx.part_world_position(blade_carrier)
        slider_open = ctx.part_element_world_aabb(blade_carrier, elem="thumb_slider")
        blade_open = ctx.part_element_world_aabb(blade_carrier, elem="utility_blade")
        ctx.expect_contact(blade_carrier, handle_shell, name="extended carrier stays in contact with guide bed")
        ctx.expect_within(
            blade_carrier,
            handle_shell,
            axes="yz",
            inner_elem=carrier_base,
            name="extended carrier stays aligned in handle channel",
        )
        if carrier_open is None or slider_open is None or blade_open is None:
            ctx.fail("carrier open-pose measurements available", "Expected blade carrier measurements in extended pose.")
        else:
            carrier_delta = carrier_open[0] - carrier_rest[0]
            slider_delta = slider_open[0][0] - slider_rest[0][0]
            ctx.check(
                "blade carrier slides only along handle axis",
                carrier_delta > 0.020
                and abs(carrier_open[1] - carrier_rest[1]) < 1e-6
                and abs(carrier_open[2] - carrier_rest[2]) < 1e-6,
                details=(
                    f"carrier rest={carrier_rest}, open={carrier_open}, "
                    f"delta={carrier_delta:.4f}"
                ),
            )
            ctx.check(
                "top slider translates with blade carrier",
                abs(slider_delta - carrier_delta) < 1e-6,
                details=(
                    f"carrier delta={carrier_delta:.6f}, "
                    f"slider delta={slider_delta:.6f}"
                ),
            )
            ctx.check(
                "blade projects forward when extended",
                blade_open[1][0] > handle_aabb[1][0] + 0.020
                and blade_rest[1][0] <= handle_aabb[1][0] + 0.002,
                details=(
                    f"blade rest max x={blade_rest[1][0]:.4f}, "
                    f"blade open max x={blade_open[1][0]:.4f}, "
                    f"handle max x={handle_aabb[1][0]:.4f}"
                ),
            )

    with ctx.pose({hatch_joint: 1.1}):
        hatch_open = ctx.part_element_world_aabb(side_hatch, elem="hatch_panel")
        ctx.expect_contact(side_hatch, handle_shell, name="opened hatch remains on hinge knuckle")
        if hatch_open is None:
            ctx.fail("hatch open-pose measurements available", "Expected side hatch panel measurements in open pose.")
        else:
            ctx.check(
                "side hatch swings outboard to open",
                hatch_open[1][2] > hatch_closed[1][2] + 0.015,
                details=(
                    f"closed max z={hatch_closed[1][2]:.4f}, "
                    f"open max z={hatch_open[1][2]:.4f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
