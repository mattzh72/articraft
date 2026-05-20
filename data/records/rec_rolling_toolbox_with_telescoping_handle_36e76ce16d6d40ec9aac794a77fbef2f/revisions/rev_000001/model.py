from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="molded_rolling_toolbox")

    yellow = model.material("molded_yellow", rgba=(0.96, 0.66, 0.08, 1.0))
    dark_yellow = model.material("corner_yellow", rgba=(0.84, 0.50, 0.03, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    dark = model.material("shadow_black", rgba=(0.03, 0.035, 0.04, 1.0))
    grey = model.material("molded_grey", rgba=(0.34, 0.35, 0.35, 1.0))
    metal = model.material("axle_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    clear = model.material("clear_smoke_blue", rgba=(0.50, 0.82, 1.0, 0.42))

    cover_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.250, 0.238, 0.020, corner_segments=8),
            0.012,
            center=True,
        ),
        "rounded_clear_organizer_cover",
    )
    tire_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.058, -0.032),
                (0.076, -0.032),
                (0.085, -0.022),
                (0.085, 0.022),
                (0.076, 0.032),
                (0.058, 0.032),
            ],
            closed=True,
        ),
        "hollow_utility_tire",
    )

    lower = model.part("lower_shell")
    lower.visual(Box((0.500, 0.710, 0.060)), origin=Origin(xyz=(0.020, 0.000, 0.120)), material=yellow, name="broad_base")
    lower.visual(Box((0.450, 0.660, 0.035)), origin=Origin(xyz=(0.020, 0.000, 0.160)), material=yellow, name="deep_floor")
    lower.visual(Box((0.430, 0.045, 0.235)), origin=Origin(xyz=(0.020, 0.330, 0.275)), material=yellow, name="side_wall_0")
    lower.visual(Box((0.430, 0.045, 0.235)), origin=Origin(xyz=(0.020, -0.330, 0.275)), material=yellow, name="side_wall_1")
    lower.visual(Box((0.045, 0.660, 0.235)), origin=Origin(xyz=(-0.215, 0.000, 0.275)), material=yellow, name="rear_wall")
    lower.visual(Box((0.045, 0.660, 0.075)), origin=Origin(xyz=(0.255, 0.000, 0.185)), material=yellow, name="front_lower_rail")
    lower.visual(Box((0.045, 0.660, 0.075)), origin=Origin(xyz=(0.255, 0.000, 0.355)), material=yellow, name="front_upper_rail")
    lower.visual(Box((0.045, 0.075, 0.180)), origin=Origin(xyz=(0.255, 0.292, 0.275)), material=yellow, name="front_jamb_0")
    lower.visual(Box((0.045, 0.075, 0.180)), origin=Origin(xyz=(0.255, -0.292, 0.275)), material=yellow, name="front_jamb_1")
    lower.visual(Box((0.012, 0.530, 0.012)), origin=Origin(xyz=(0.282, 0.000, 0.321)), material=dark, name="drawer_bay_top")
    lower.visual(Box((0.012, 0.530, 0.012)), origin=Origin(xyz=(0.282, 0.000, 0.219)), material=dark, name="drawer_bay_bottom")
    lower.visual(Box((0.012, 0.012, 0.100)), origin=Origin(xyz=(0.282, 0.258, 0.270)), material=dark, name="drawer_bay_side_0")
    lower.visual(Box((0.012, 0.012, 0.100)), origin=Origin(xyz=(0.282, -0.258, 0.270)), material=dark, name="drawer_bay_side_1")
    lower.visual(Box((0.320, 0.014, 0.014)), origin=Origin(xyz=(0.045, 0.238, 0.183)), material=grey, name="guide_runner_0")
    lower.visual(Box((0.320, 0.014, 0.014)), origin=Origin(xyz=(0.045, -0.238, 0.183)), material=grey, name="guide_runner_1")
    lower.visual(Cylinder(0.036, 0.245), origin=Origin(xyz=(0.225, 0.318, 0.272)), material=dark_yellow, name="corner_post_0")
    lower.visual(Cylinder(0.036, 0.245), origin=Origin(xyz=(0.225, -0.318, 0.272)), material=dark_yellow, name="corner_post_1")
    lower.visual(Cylinder(0.034, 0.245), origin=Origin(xyz=(-0.210, 0.318, 0.272)), material=dark_yellow, name="corner_post_2")
    lower.visual(Cylinder(0.034, 0.245), origin=Origin(xyz=(-0.210, -0.318, 0.272)), material=dark_yellow, name="corner_post_3")
    lower.visual(Box((0.390, 0.026, 0.026)), origin=Origin(xyz=(0.030, 0.343, 0.365)), material=black, name="upper_side_latch_0")
    lower.visual(Box((0.390, 0.026, 0.026)), origin=Origin(xyz=(0.030, -0.343, 0.365)), material=black, name="upper_side_latch_1")
    lower.visual(Box((0.050, 0.040, 0.340)), origin=Origin(xyz=(-0.262, 0.185, 0.370)), material=grey, name="handle_sleeve_0")
    lower.visual(Box((0.050, 0.040, 0.340)), origin=Origin(xyz=(-0.262, -0.185, 0.370)), material=grey, name="handle_sleeve_1")
    lower.visual(Cylinder(0.012, 0.850), origin=Origin(xyz=(-0.190, 0.000, 0.090), rpy=(pi / 2, 0.0, 0.0)), material=metal, name="wheel_axle")
    lower.visual(Box((0.170, 0.040, 0.030)), origin=Origin(xyz=(-0.190, 0.365, 0.207)), material=yellow, name="wheel_fender_0")
    lower.visual(Box((0.170, 0.040, 0.030)), origin=Origin(xyz=(-0.190, -0.365, 0.207)), material=yellow, name="wheel_fender_1")

    lid = model.part("lid_shell")
    lid.visual(Box((0.470, 0.680, 0.050)), origin=Origin(xyz=(0.235, 0.000, 0.035)), material=yellow, name="lid_top")
    lid.visual(Box((0.035, 0.670, 0.070)), origin=Origin(xyz=(0.472, 0.000, 0.005)), material=yellow, name="front_lip")
    lid.visual(Box((0.455, 0.035, 0.070)), origin=Origin(xyz=(0.235, 0.342, 0.005)), material=yellow, name="lid_side_0")
    lid.visual(Box((0.455, 0.035, 0.070)), origin=Origin(xyz=(0.235, -0.342, 0.005)), material=yellow, name="lid_side_1")
    lid.visual(Cylinder(0.014, 0.620), origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(pi / 2, 0.0, 0.0)), material=black, name="main_hinge_barrel")
    lid.visual(Box((0.265, 0.248, 0.006)), origin=Origin(xyz=(0.255, 0.158, 0.061)), material=dark, name="cover_recess_0")
    lid.visual(Box((0.265, 0.248, 0.006)), origin=Origin(xyz=(0.255, -0.158, 0.061)), material=dark, name="cover_recess_1")
    lid.visual(Box((0.310, 0.018, 0.020)), origin=Origin(xyz=(0.255, 0.000, 0.070)), material=yellow, name="center_divider")
    lid.visual(Box((0.022, 0.600, 0.018)), origin=Origin(xyz=(0.112, 0.000, 0.069)), material=yellow, name="rear_organizer_rib")
    lid.visual(Box((0.022, 0.600, 0.018)), origin=Origin(xyz=(0.395, 0.000, 0.069)), material=yellow, name="front_organizer_rib")
    lid.visual(Box((0.038, 0.200, 0.040)), origin=Origin(xyz=(0.455, 0.000, 0.020)), material=black, name="front_latch")

    model.articulation(
        "lower_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lid,
        origin=Origin(xyz=(-0.225, 0.000, 0.425)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    drawer = model.part("front_drawer")
    drawer.visual(Box((0.340, 0.480, 0.060)), origin=Origin(xyz=(-0.020, 0.000, 0.000)), material=grey, name="drawer_body")
    drawer.visual(Box((0.035, 0.500, 0.085)), origin=Origin(xyz=(0.1625, 0.000, 0.000)), material=yellow, name="drawer_face")
    drawer.visual(Cylinder(0.018, 0.220), origin=Origin(xyz=(0.183, 0.000, 0.003), rpy=(pi / 2, 0.0, 0.0)), material=dark, name="finger_pull")
    drawer.visual(Box((0.310, 0.012, 0.053)), origin=Origin(xyz=(-0.032, 0.238, -0.0535)), material=dark, name="drawer_runner_0")
    drawer.visual(Box((0.310, 0.012, 0.053)), origin=Origin(xyz=(-0.032, -0.238, -0.0535)), material=dark, name="drawer_runner_1")
    model.articulation(
        "lower_to_drawer",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=drawer,
        origin=Origin(xyz=(0.080, 0.000, 0.270)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.180),
    )

    handle = model.part("trolley_handle")
    handle.visual(Cylinder(0.012, 0.620), origin=Origin(xyz=(-0.262, 0.185, 0.390)), material=metal, name="rail_0")
    handle.visual(Cylinder(0.012, 0.620), origin=Origin(xyz=(-0.262, -0.185, 0.390)), material=metal, name="rail_1")
    handle.visual(Cylinder(0.019, 0.410), origin=Origin(xyz=(-0.262, 0.000, 0.705), rpy=(pi / 2, 0.0, 0.0)), material=black, name="hand_grip")
    handle.visual(Box((0.038, 0.050, 0.040)), origin=Origin(xyz=(-0.262, 0.185, 0.690)), material=black, name="grip_socket_0")
    handle.visual(Box((0.038, 0.050, 0.040)), origin=Origin(xyz=(-0.262, -0.185, 0.690)), material=black, name="grip_socket_1")
    model.articulation(
        "lower_to_handle",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=handle,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.45, lower=0.0, upper=0.280),
    )

    for idx, y in enumerate((0.405, -0.405)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(tire_mesh, origin=Origin(rpy=(0.0, pi / 2, 0.0)), material=black, name="tire")
        wheel.visual(Cylinder(0.058, 0.070), origin=Origin(rpy=(0.0, pi / 2, 0.0)), material=grey, name="rim")
        wheel.visual(Cylinder(0.030, 0.078), origin=Origin(rpy=(0.0, pi / 2, 0.0)), material=metal, name="hub")
        for tread_idx in range(12):
            tread_angle = tread_idx * 2.0 * pi / 12.0
            wheel.visual(
                Box((0.070, 0.012, 0.018)),
                origin=Origin(
                    xyz=(0.000, 0.085 * sin(tread_angle), 0.085 * cos(tread_angle)),
                    rpy=(tread_angle, 0.0, 0.0),
                ),
                material=black,
                name=f"tread_{tread_idx}",
            )
        for spoke_idx in range(5):
            angle = spoke_idx * 2.0 * pi / 5.0
            wheel.visual(
                Box((0.020, 0.010, 0.034)),
                origin=Origin(
                    xyz=(0.030, 0.045 * sin(angle), 0.045 * cos(angle)),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=metal,
                name=f"spoke_{spoke_idx}",
            )
        model.articulation(
            f"axle_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=lower,
            child=wheel,
            origin=Origin(xyz=(-0.190, y, 0.090), rpy=(0.0, 0.0, pi / 2)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    for idx, y in enumerate((0.158, -0.158)):
        cover = model.part(f"organizer_cover_{idx}")
        cover.visual(cover_mesh, origin=Origin(xyz=(0.125, 0.000, 0.006)), material=clear, name="cover_panel")
        cover.visual(Cylinder(0.010, 0.210), origin=Origin(xyz=(0.000, 0.000, 0.006), rpy=(pi / 2, 0.0, 0.0)), material=clear, name="hinge_knuckle")
        model.articulation(
            f"lid_to_cover_{idx}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=cover,
            origin=Origin(xyz=(0.130, y, 0.068)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.75),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid_shell")
    drawer = object_model.get_part("front_drawer")
    handle = object_model.get_part("trolley_handle")
    cover_0 = object_model.get_part("organizer_cover_0")
    cover_1 = object_model.get_part("organizer_cover_1")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    handle_joint = object_model.get_articulation("lower_to_handle")
    drawer_joint = object_model.get_articulation("lower_to_drawer")
    lid_joint = object_model.get_articulation("lower_to_lid")
    cover_joint_0 = object_model.get_articulation("lid_to_cover_0")
    cover_joint_1 = object_model.get_articulation("lid_to_cover_1")
    wheel_joint_0 = object_model.get_articulation("axle_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("axle_to_wheel_1")

    ctx.allow_overlap(
        lower,
        handle,
        elem_a="handle_sleeve_0",
        elem_b="rail_0",
        reason="The telescoping rail is intentionally represented captured inside a molded rear sleeve.",
    )
    ctx.allow_overlap(
        lower,
        handle,
        elem_a="handle_sleeve_1",
        elem_b="rail_1",
        reason="The telescoping rail is intentionally represented captured inside a molded rear sleeve.",
    )
    for idx, wheel in enumerate((wheel_0, wheel_1)):
        ctx.allow_overlap(
            lower,
            wheel,
            elem_a="wheel_axle",
            elem_b="hub",
            reason="The steel axle is intentionally captured through the simplified solid wheel hub proxy.",
        )
        ctx.allow_overlap(
            lower,
            wheel,
            elem_a="wheel_axle",
            elem_b="rim",
            reason="The solid rim proxy represents the hidden bored wheel center around the same captured axle.",
        )
        ctx.expect_within(
            lower,
            wheel,
            axes="xz",
            inner_elem="wheel_axle",
            outer_elem="hub",
            margin=0.002,
            name=f"wheel {idx} axle centered inside hub",
        )
        ctx.expect_overlap(
            wheel,
            lower,
            axes="y",
            elem_a="hub",
            elem_b="wheel_axle",
            min_overlap=0.040,
            name=f"wheel {idx} hub has retained axle engagement",
        )
        ctx.expect_within(
            lower,
            wheel,
            axes="xz",
            inner_elem="wheel_axle",
            outer_elem="rim",
            margin=0.002,
            name=f"wheel {idx} axle passes through rim center",
        )

    for idx in (0, 1):
        ctx.expect_within(
            handle,
            lower,
            axes="xy",
            inner_elem=f"rail_{idx}",
            outer_elem=f"handle_sleeve_{idx}",
            margin=0.001,
            name=f"handle rail {idx} centered in sleeve",
        )
        ctx.expect_overlap(
            handle,
            lower,
            axes="z",
            elem_a=f"rail_{idx}",
            elem_b=f"handle_sleeve_{idx}",
            min_overlap=0.200,
            name=f"collapsed handle rail {idx} retained in sleeve",
        )

    with ctx.pose({handle_joint: 0.280}):
        for idx in (0, 1):
            ctx.expect_within(
                handle,
                lower,
                axes="xy",
                inner_elem=f"rail_{idx}",
                outer_elem=f"handle_sleeve_{idx}",
                margin=0.001,
                name=f"extended handle rail {idx} stays in sleeve",
            )
            ctx.expect_overlap(
                handle,
                lower,
                axes="z",
                elem_a=f"rail_{idx}",
                elem_b=f"handle_sleeve_{idx}",
                min_overlap=0.050,
                name=f"extended handle rail {idx} keeps retained insertion",
            )

    ctx.expect_within(
        drawer,
        lower,
        axes="yz",
        inner_elem="drawer_face",
        margin=0.004,
        name="drawer face aligned within lower shell bay",
    )
    ctx.expect_overlap(
        drawer,
        lower,
        axes="x",
        elem_a="drawer_body",
        elem_b="guide_runner_0",
        min_overlap=0.100,
        name="closed drawer remains on straight runner",
    )
    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.180}):
        ctx.expect_within(
            drawer,
            lower,
            axes="yz",
            inner_elem="drawer_face",
            margin=0.004,
            name="extended drawer remains aligned in bay",
        )
        ctx.expect_overlap(
            drawer,
            lower,
            axes="x",
            elem_a="drawer_body",
            elem_b="guide_runner_0",
            min_overlap=0.080,
            name="extended drawer retains guide engagement",
        )
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides outward toward the front",
        drawer_rest is not None and drawer_extended is not None and drawer_extended[0] > drawer_rest[0] + 0.15,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    for idx, cover in enumerate((cover_0, cover_1)):
        ctx.expect_gap(
            cover,
            lid,
            axis="z",
            min_gap=0.0,
            max_gap=0.010,
            positive_elem="cover_panel",
            negative_elem="lid_top",
            name=f"organizer cover {idx} sits just above lid recess",
        )
        ctx.expect_overlap(
            cover,
            lid,
            axes="xy",
            elem_a="cover_panel",
            elem_b=f"cover_recess_{idx}",
            min_overlap=0.180,
            name=f"organizer cover {idx} spans its clear compartment",
        )

    cover_rest = ctx.part_world_aabb(cover_0)
    with ctx.pose({cover_joint_0: 1.250, cover_joint_1: 1.250}):
        cover_open = ctx.part_world_aabb(cover_0)
    ctx.check(
        "organizer covers hinge upward from rear edge",
        cover_rest is not None and cover_open is not None and cover_open[1][2] > cover_rest[1][2] + 0.12,
        details=f"rest={cover_rest}, open={cover_open}",
    )

    lid_rest = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.0}):
        lid_open = ctx.part_world_aabb(lid)
    ctx.check(
        "separate lid shell opens upward on rear hinge",
        lid_rest is not None and lid_open is not None and lid_open[1][2] > lid_rest[1][2] + 0.18,
        details=f"rest={lid_rest}, open={lid_open}",
    )

    ctx.check(
        "transport wheels use continuous spin joints",
        wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS,
    )
    ctx.expect_contact(wheel_0, lower, elem_a="hub", elem_b="wheel_axle", contact_tol=0.020, name="wheel 0 mounted on axle")
    ctx.expect_contact(wheel_1, lower, elem_a="hub", elem_b="wheel_axle", contact_tol=0.020, name="wheel 1 mounted on axle")

    return ctx.report()


object_model = build_object_model()
