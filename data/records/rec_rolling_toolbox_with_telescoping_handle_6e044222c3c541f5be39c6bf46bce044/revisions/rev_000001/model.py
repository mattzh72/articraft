from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stackable_rolling_toolbox_base")

    body_mat = model.material("graphite_rugged_polymer", rgba=(0.10, 0.12, 0.13, 1.0))
    lid_mat = model.material("black_lid_polymer", rgba=(0.03, 0.035, 0.04, 1.0))
    rib_mat = model.material("charcoal_molded_ribs", rgba=(0.055, 0.065, 0.07, 1.0))
    drawer_mat = model.material("dark_gray_drawer", rgba=(0.15, 0.17, 0.18, 1.0))
    latch_mat = model.material("yellow_latch_plastic", rgba=(0.95, 0.63, 0.08, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.55, 0.56, 0.55, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.015, 0.015, 0.013, 1.0))
    wheel_mat = model.material("gray_wheel_hub", rgba=(0.33, 0.34, 0.34, 1.0))

    lower_shell = model.part("lower_shell")

    # Large open lower bin: separate wall members leave a real hollow interior
    # rather than a solid block, with thick molded rim and drawer aperture.
    lower_shell.visual(Box((0.64, 0.40, 0.050)), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=body_mat, name="bottom_pan")
    lower_shell.visual(Box((0.045, 0.40, 0.370)), origin=Origin(xyz=(-0.2975, 0.0, 0.235)), material=body_mat, name="side_wall_0")
    lower_shell.visual(Box((0.045, 0.40, 0.370)), origin=Origin(xyz=(0.2975, 0.0, 0.235)), material=body_mat, name="side_wall_1")
    lower_shell.visual(Box((0.64, 0.045, 0.370)), origin=Origin(xyz=(0.0, 0.1775, 0.235)), material=body_mat, name="rear_wall")
    lower_shell.visual(Box((0.64, 0.045, 0.080)), origin=Origin(xyz=(0.0, -0.1775, 0.090)), material=body_mat, name="front_lower_rail")
    lower_shell.visual(Box((0.64, 0.045, 0.170)), origin=Origin(xyz=(0.0, -0.1775, 0.335)), material=body_mat, name="front_upper_rail")
    lower_shell.visual(Box((0.070, 0.045, 0.120)), origin=Origin(xyz=(-0.285, -0.1775, 0.190)), material=body_mat, name="front_post_0")
    lower_shell.visual(Box((0.070, 0.045, 0.120)), origin=Origin(xyz=(0.285, -0.1775, 0.190)), material=body_mat, name="front_post_1")
    lower_shell.visual(Box((0.60, 0.035, 0.025)), origin=Origin(xyz=(0.0, -0.1825, 0.4075)), material=rib_mat, name="front_top_lip")
    lower_shell.visual(Box((0.60, 0.035, 0.025)), origin=Origin(xyz=(0.0, 0.1825, 0.4075)), material=rib_mat, name="rear_top_lip")
    lower_shell.visual(Box((0.035, 0.36, 0.025)), origin=Origin(xyz=(-0.3025, 0.0, 0.4075)), material=rib_mat, name="side_top_lip_0")
    lower_shell.visual(Box((0.035, 0.36, 0.025)), origin=Origin(xyz=(0.3025, 0.0, 0.4075)), material=rib_mat, name="side_top_lip_1")

    # Straight drawer runners and molded front ribs, all tied into the shell.
    lower_shell.visual(Box((0.032, 0.290, 0.020)), origin=Origin(xyz=(-0.260, -0.055, 0.158)), material=rib_mat, name="drawer_runner_0")
    lower_shell.visual(Box((0.032, 0.290, 0.020)), origin=Origin(xyz=(0.260, -0.055, 0.158)), material=rib_mat, name="drawer_runner_1")
    for x in (-0.18, 0.0, 0.18):
        lower_shell.visual(Box((0.030, 0.018, 0.130)), origin=Origin(xyz=(x, -0.207, 0.330)), material=rib_mat, name=f"front_rib_{x:+.2f}")
    lower_shell.visual(Box((0.50, 0.010, 0.020)), origin=Origin(xyz=(0.0, -0.201, 0.248)), material=rib_mat, name="drawer_shadow_gap")

    # Rear telescoping rail sockets, mounted to the back wall.
    lower_shell.visual(Box((0.048, 0.035, 0.410)), origin=Origin(xyz=(-0.225, 0.237, 0.255)), material=rib_mat, name="rear_rail_0")
    lower_shell.visual(Box((0.070, 0.030, 0.040)), origin=Origin(xyz=(-0.225, 0.2395, 0.475)), material=rib_mat, name="rail_collar_0")
    lower_shell.visual(Box((0.048, 0.035, 0.410)), origin=Origin(xyz=(0.225, 0.237, 0.255)), material=rib_mat, name="rear_rail_1")
    lower_shell.visual(Box((0.070, 0.030, 0.040)), origin=Origin(xyz=(0.225, 0.2395, 0.475)), material=rib_mat, name="rail_collar_1")

    # Exposed rear hinge knuckles: parent barrels interleave with lid barrels.
    for i, x in enumerate((-0.245, 0.0, 0.245)):
        lower_shell.visual(
            Cylinder(radius=0.018, length=0.110),
            origin=Origin(xyz=(x, 0.236, 0.440), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel_mat,
            name=f"hinge_barrel_{i}",
        )
        lower_shell.visual(Box((0.110, 0.018, 0.012)), origin=Origin(xyz=(x, 0.209, 0.413)), material=rib_mat, name=f"hinge_foot_{i}")
        lower_shell.visual(Box((0.110, 0.020, 0.024)), origin=Origin(xyz=(x, 0.226, 0.410)), material=rib_mat, name=f"hinge_riser_{i}")

    # Wheel axle and side pivot bosses are real support features on the shell.
    lower_shell.visual(
        Cylinder(radius=0.008, length=0.740),
        origin=Origin(xyz=(0.0, 0.165, 0.080), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_mat,
        name="rear_axle",
    )
    lower_shell.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(-0.328, -0.055, 0.355), rpy=(0.0, pi / 2.0, 0.0)),
        material=rib_mat,
        name="latch_boss_0",
    )
    lower_shell.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.328, -0.055, 0.355), rpy=(0.0, pi / 2.0, 0.0)),
        material=rib_mat,
        name="latch_boss_1",
    )

    lid = model.part("lid")
    lid.visual(Box((0.66, 0.410, 0.050)), origin=Origin(xyz=(0.0, -0.226, 0.005)), material=lid_mat, name="lid_slab")
    lid.visual(Box((0.58, 0.030, 0.020)), origin=Origin(xyz=(0.0, -0.406, 0.040)), material=rib_mat, name="front_lid_rib")
    lid.visual(Box((0.58, 0.030, 0.020)), origin=Origin(xyz=(0.0, -0.091, 0.040)), material=rib_mat, name="rear_lid_rib")
    lid.visual(Box((0.035, 0.300, 0.020)), origin=Origin(xyz=(-0.285, -0.246, 0.040)), material=rib_mat, name="side_lid_rib_0")
    lid.visual(Box((0.035, 0.300, 0.020)), origin=Origin(xyz=(0.285, -0.246, 0.040)), material=rib_mat, name="side_lid_rib_1")
    lid.visual(Box((0.23, 0.070, 0.012)), origin=Origin(xyz=(0.0, -0.266, 0.036)), material=body_mat, name="stacking_recess")
    for i, x in enumerate((-0.125, 0.125)):
        lid.visual(Box((0.105, 0.022, 0.012)), origin=Origin(xyz=(x, -0.011, -0.014)), material=lid_mat, name=f"hinge_leaf_{i}")
    for i, x in enumerate((-0.125, 0.125)):
        lid.visual(
            Cylinder(radius=0.018, length=0.105),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel_mat,
            name=f"lid_barrel_{i}",
        )
    for i, x in enumerate((-0.334, 0.334)):
        lid.visual(Box((0.018, 0.075, 0.040)), origin=Origin(xyz=(x, -0.291, -0.010)), material=rib_mat, name=f"latch_catch_{i}")

    drawer = model.part("drawer")
    drawer.visual(Box((0.540, 0.025, 0.120)), origin=Origin(xyz=(0.0, -0.0125, 0.0)), material=drawer_mat, name="drawer_front")
    drawer.visual(Box((0.455, 0.245, 0.018)), origin=Origin(xyz=(0.0, 0.115, -0.028)), material=drawer_mat, name="drawer_floor")
    drawer.visual(Box((0.018, 0.245, 0.085)), origin=Origin(xyz=(-0.235, 0.115, 0.015)), material=drawer_mat, name="drawer_side_0")
    drawer.visual(Box((0.018, 0.245, 0.085)), origin=Origin(xyz=(0.235, 0.115, 0.015)), material=drawer_mat, name="drawer_side_1")
    drawer.visual(Box((0.455, 0.018, 0.075)), origin=Origin(xyz=(0.0, 0.230, 0.010)), material=drawer_mat, name="drawer_back")
    drawer.visual(Box((0.190, 0.004, 0.036)), origin=Origin(xyz=(0.0, -0.027, 0.005)), material=lid_mat, name="finger_pull")

    handle = model.part("telescoping_handle")
    handle.visual(Cylinder(radius=0.014, length=0.620), origin=Origin(xyz=(-0.225, 0.0485, -0.110)), material=steel_mat, name="handle_post_0")
    handle.visual(Box((0.038, 0.024, 0.045)), origin=Origin(xyz=(-0.225, 0.0485, -0.405)), material=rib_mat, name="slide_block_0")
    handle.visual(Cylinder(radius=0.014, length=0.620), origin=Origin(xyz=(0.225, 0.0485, -0.110)), material=steel_mat, name="handle_post_1")
    handle.visual(Box((0.038, 0.024, 0.045)), origin=Origin(xyz=(0.225, 0.0485, -0.405)), material=rib_mat, name="slide_block_1")
    handle.visual(
        Cylinder(radius=0.020, length=0.520),
        origin=Origin(xyz=(0.0, 0.0485, 0.210), rpy=(0.0, pi / 2.0, 0.0)),
        material=rib_mat,
        name="grip_bar",
    )

    for suffix, side in (("0", -1.0), ("1", 1.0)):
        latch = model.part(f"side_latch_{suffix}")
        latch.visual(Box((0.010, 0.078, 0.130)), origin=Origin(xyz=(side * 0.014, 0.0, 0.057)), material=latch_mat, name="latch_plate")
        latch.visual(
            Cylinder(radius=0.020, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=latch_mat,
            name="pivot_disc",
        )
        latch.visual(Box((0.012, 0.060, 0.016)), origin=Origin(xyz=(side * 0.018, 0.0, 0.117)), material=latch_mat, name="top_hook")

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.052,
            0.044,
            rim=WheelRim(inner_radius=0.034, flange_height=0.004, flange_thickness=0.003, bead_seat_depth=0.002),
            hub=WheelHub(
                radius=0.020,
                width=0.034,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.026, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0025, window_radius=0.007),
            bore=WheelBore(style="round", diameter=0.022),
        ),
        "toolbox_wheel_hub",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.052,
            inner_radius=0.053,
            tread=TireTread(style="block", depth=0.005, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.005, radius=0.003),
        ),
        "toolbox_utility_tire",
    )
    for suffix in ("0", "1"):
        wheel = model.part(f"wheel_{suffix}")
        wheel.visual(wheel_mesh, material=wheel_mat, name="wheel_hub")
        wheel.visual(tire_mesh, material=rubber_mat, name="tire")

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, 0.236, 0.440)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=lower_shell,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.222, 0.176)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.210),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=lower_shell,
        child=handle,
        origin=Origin(xyz=(0.0, 0.220, 0.455)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.30, lower=0.0, upper=0.300),
    )
    model.articulation(
        "body_to_latch_0",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child="side_latch_0",
        origin=Origin(xyz=(-0.347, -0.055, 0.355)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.25, upper=1.10),
    )
    model.articulation(
        "body_to_latch_1",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child="side_latch_1",
        origin=Origin(xyz=(0.347, -0.055, 0.355)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.25, upper=1.10),
    )
    model.articulation(
        "body_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=lower_shell,
        child="wheel_0",
        origin=Origin(xyz=(-0.346, 0.165, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "body_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=lower_shell,
        child="wheel_1",
        origin=Origin(xyz=(0.346, 0.165, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("drawer")
    handle = object_model.get_part("telescoping_handle")
    latch_1 = object_model.get_part("side_latch_1")
    wheel_0 = object_model.get_part("wheel_0")

    lid_joint = object_model.get_articulation("body_to_lid")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    handle_joint = object_model.get_articulation("body_to_handle")
    latch_joint = object_model.get_articulation("body_to_latch_1")
    wheel_joint = object_model.get_articulation("body_to_wheel_0")

    expected_types = {
        "body_to_lid": ArticulationType.REVOLUTE,
        "body_to_drawer": ArticulationType.PRISMATIC,
        "body_to_handle": ArticulationType.PRISMATIC,
        "body_to_latch_1": ArticulationType.REVOLUTE,
        "body_to_wheel_0": ArticulationType.CONTINUOUS,
    }
    for joint_name, expected in expected_types.items():
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} mechanism type",
            joint.articulation_type == expected,
            details=f"expected {expected}, got {joint.articulation_type}",
        )

    with ctx.pose({lid_joint: 0.0, drawer_joint: 0.0, handle_joint: 0.0, latch_joint: 0.0, wheel_joint: 0.0}):
        ctx.expect_gap(
            lid,
            lower_shell,
            axis="z",
            positive_elem="lid_slab",
            negative_elem="front_top_lip",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid seats on molded front rim",
        )
        ctx.expect_gap(
            drawer,
            lower_shell,
            axis="x",
            positive_elem="drawer_side_0",
            negative_elem="drawer_runner_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="drawer side rides against left runner",
        )
        ctx.expect_overlap(
            drawer,
            lower_shell,
            axes="y",
            elem_a="drawer_side_0",
            elem_b="drawer_runner_0",
            min_overlap=0.10,
            name="drawer remains engaged along straight runner",
        )
        ctx.expect_within(
            drawer,
            lower_shell,
            axes="xz",
            margin=0.002,
            name="closed drawer stays aligned in lower shell",
        )
        ctx.expect_gap(
            handle,
            lower_shell,
            axis="y",
            positive_elem="handle_post_0",
            negative_elem="rear_rail_0",
            max_gap=0.002,
            max_penetration=0.0,
            name="telescoping post rides rear rail",
        )
        ctx.expect_gap(
            latch_1,
            lower_shell,
            axis="x",
            positive_elem="pivot_disc",
            negative_elem="latch_boss_1",
            max_gap=0.001,
            max_penetration=0.000001,
            name="side latch pivots on molded boss",
        )
        ctx.expect_overlap(
            wheel_0,
            lower_shell,
            axes="x",
            elem_a="wheel_hub",
            elem_b="rear_axle",
            min_overlap=0.020,
            name="wheel hub is centered on axle line",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_slab")
    with ctx.pose({lid_joint: 1.20}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_slab")
    ctx.check(
        "lid opens upward about rear hinge",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.210}):
        ctx.expect_within(
            drawer,
            lower_shell,
            axes="xz",
            margin=0.002,
            name="extended drawer stays level and centered",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides straight out the front",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < closed_drawer_pos[1] - 0.18,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    collapsed_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_joint: 0.300}):
        raised_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "handle extends upward on twin rear rails",
        collapsed_handle_pos is not None
        and raised_handle_pos is not None
        and raised_handle_pos[2] > collapsed_handle_pos[2] + 0.25,
        details=f"collapsed={collapsed_handle_pos}, raised={raised_handle_pos}",
    )

    return ctx.report()


object_model = build_object_model()
