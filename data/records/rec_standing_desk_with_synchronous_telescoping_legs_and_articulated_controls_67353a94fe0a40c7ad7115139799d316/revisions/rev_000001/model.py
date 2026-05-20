from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_standing_desk")

    warm_wood = Material("warm_laminate_top", color=(0.72, 0.52, 0.32, 1.0))
    black_metal = Material("satin_black_steel", color=(0.015, 0.016, 0.018, 1.0))
    dark_plastic = Material("dark_control_plastic", color=(0.035, 0.038, 0.043, 1.0))
    button_mat = Material("soft_black_buttons", color=(0.006, 0.007, 0.008, 1.0))
    white_mark = Material("white_button_legends", color=(0.92, 0.92, 0.86, 1.0))
    power_mat = Material("muted_power_button", color=(0.18, 0.025, 0.025, 1.0))
    rubber = Material("black_rubber_feet", color=(0.0, 0.0, 0.0, 1.0))
    for mat in (warm_wood, black_metal, dark_plastic, button_mat, white_mark, power_mat, rubber):
        model.materials.append(mat)

    column_xs = (-0.86, 0.0, 0.86)
    outer_bottom_z = 0.04
    outer_height = 0.62
    outer_top_z = outer_bottom_z + outer_height
    travel = 0.47
    retained_insertion = 0.12
    inner_min_z = -(travel + retained_insertion)
    inner_max_z = 0.04
    inner_len = inner_max_z - inner_min_z
    desktop_offset_z = 0.08

    # One continuous floor/base frame carries all three fixed outer sleeves so the
    # three columns visibly support the same bench-length workstation.
    base = model.part("base_frame")
    base.visual(Box((2.18, 0.045, 0.035)), origin=Origin(xyz=(0.0, -0.33, 0.042)), material=black_metal, name="front_floor_rail")
    base.visual(Box((2.18, 0.045, 0.035)), origin=Origin(xyz=(0.0, 0.33, 0.042)), material=black_metal, name="rear_floor_rail")

    for i, x in enumerate(column_xs):
        base.visual(Box((0.25, 0.76, 0.040)), origin=Origin(xyz=(x, 0.0, 0.020)), material=black_metal, name=f"foot_bar_{i}")
        base.visual(Cylinder(radius=0.026, length=0.010), origin=Origin(xyz=(x, -0.34, 0.005)), material=rubber, name=f"front_glide_{i}")
        base.visual(Cylinder(radius=0.026, length=0.010), origin=Origin(xyz=(x, 0.34, 0.005)), material=rubber, name=f"rear_glide_{i}")

        # Rectangular hollow outer column, built from four steel wall members.
        zc = outer_bottom_z + outer_height / 2.0
        wall = 0.014
        outer_x = 0.130
        outer_y = 0.100
        base.visual(Box((outer_x, wall, outer_height)), origin=Origin(xyz=(x, -(outer_y / 2.0 - wall / 2.0), zc)), material=black_metal, name=f"outer_front_wall_{i}")
        base.visual(Box((outer_x, wall, outer_height)), origin=Origin(xyz=(x, outer_y / 2.0 - wall / 2.0, zc)), material=black_metal, name=f"outer_rear_wall_{i}")
        base.visual(Box((wall, outer_y, outer_height)), origin=Origin(xyz=(x - (outer_x / 2.0 - wall / 2.0), 0.0, zc)), material=black_metal, name=f"outer_side_wall_{i}_0")
        base.visual(Box((wall, outer_y, outer_height)), origin=Origin(xyz=(x + (outer_x / 2.0 - wall / 2.0), 0.0, zc)), material=black_metal, name=f"outer_side_wall_{i}_1")

        # A top collar makes the sleeve opening read as a real telescoping tube.
        base.visual(Box((outer_x + 0.018, 0.018, 0.030)), origin=Origin(xyz=(x, -0.055, outer_top_z - 0.005)), material=black_metal, name=f"collar_front_{i}")
        base.visual(Box((outer_x + 0.018, 0.018, 0.030)), origin=Origin(xyz=(x, 0.055, outer_top_z - 0.005)), material=black_metal, name=f"collar_rear_{i}")
        base.visual(Box((0.018, outer_y + 0.018, 0.030)), origin=Origin(xyz=(x - 0.074, 0.0, outer_top_z - 0.005)), material=black_metal, name=f"collar_side_{i}_0")
        base.visual(Box((0.018, outer_y + 0.018, 0.030)), origin=Origin(xyz=(x + 0.074, 0.0, outer_top_z - 0.005)), material=black_metal, name=f"collar_side_{i}_1")

    inner_parts = []
    for i, x in enumerate(column_xs):
        inner = model.part(f"inner_stage_{i}")
        inner.visual(
            Box((0.102, 0.072, inner_len)),
            origin=Origin(xyz=(0.0, 0.0, (inner_min_z + inner_max_z) / 2.0)),
            material=black_metal,
            name="inner_tube",
        )
        inner.visual(
            Box((0.120, 0.095, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, inner_max_z + 0.009)),
            material=black_metal,
            name="top_bearing_plate",
        )
        inner_parts.append(inner)

    lift_limits = MotionLimits(effort=900.0, velocity=0.06, lower=0.0, upper=travel)
    center_lift = model.articulation(
        "lift_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_parts[1],
        origin=Origin(xyz=(column_xs[1], 0.0, outer_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "lift_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_parts[0],
        origin=Origin(xyz=(column_xs[0], 0.0, outer_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint="lift_1", multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "lift_2",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_parts[2],
        origin=Origin(xyz=(column_xs[2], 0.0, outer_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint="lift_1", multiplier=1.0, offset=0.0),
    )

    desktop = model.part("desktop_frame")
    desktop.visual(Box((2.40, 0.90, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.0225)), material=warm_wood, name="long_desktop")
    desktop.visual(Box((2.16, 0.052, 0.050)), origin=Origin(xyz=(0.0, -0.365, -0.025)), material=black_metal, name="front_apron")
    desktop.visual(Box((2.16, 0.052, 0.050)), origin=Origin(xyz=(0.0, 0.365, -0.025)), material=black_metal, name="rear_apron")
    desktop.visual(Box((0.050, 0.74, 0.045)), origin=Origin(xyz=(-1.02, 0.0, -0.0275)), material=black_metal, name="end_crossbar_0")
    desktop.visual(Box((0.050, 0.74, 0.045)), origin=Origin(xyz=(1.02, 0.0, -0.0275)), material=black_metal, name="end_crossbar_1")
    for i, x in enumerate(column_xs):
        desktop.visual(Box((0.25, 0.18, 0.040)), origin=Origin(xyz=(x, 0.0, -0.002)), material=black_metal, name=f"column_socket_{i}")
        desktop.visual(Box((0.11, 0.11, 0.040)), origin=Origin(xyz=(x, 0.0, -0.002)), material=black_metal, name=f"socket_drop_{i}")
    model.articulation(
        "inner_to_desktop",
        ArticulationType.FIXED,
        parent=inner_parts[1],
        child=desktop,
        origin=Origin(xyz=(0.0, 0.0, desktop_offset_z)),
    )

    # A real under-edge control pod, with an open front cavity for independently
    # sliding push buttons and a hinged paddle hanging below.
    pod = model.part("control_pod")
    pod.visual(Box((0.360, 0.090, 0.006)), origin=Origin(xyz=(0.0, 0.0, 0.027)), material=dark_plastic, name="pod_top_wall")
    pod.visual(Box((0.360, 0.090, 0.006)), origin=Origin(xyz=(0.0, 0.0, -0.027)), material=dark_plastic, name="pod_bottom_wall")
    pod.visual(Box((0.360, 0.006, 0.055)), origin=Origin(xyz=(0.0, 0.042, 0.0)), material=dark_plastic, name="pod_back_wall")
    pod.visual(Box((0.006, 0.090, 0.055)), origin=Origin(xyz=(-0.177, 0.0, 0.0)), material=dark_plastic, name="pod_side_0")
    pod.visual(Box((0.006, 0.090, 0.055)), origin=Origin(xyz=(0.177, 0.0, 0.0)), material=dark_plastic, name="pod_side_1")
    pod.visual(Box((0.300, 0.050, 0.045)), origin=Origin(xyz=(0.0, -0.005, 0.0525)), material=dark_plastic, name="edge_mount_tongue")
    pod.visual(Box((0.354, 0.084, 0.004)), origin=Origin(xyz=(0.0, -0.003, 0.003)), material=dark_plastic, name="button_slide_shelf")
    pod.visual(Box((0.014, 0.024, 0.030)), origin=Origin(xyz=(-0.068, -0.052, -0.043)), material=dark_plastic, name="paddle_bracket_0")
    pod.visual(Box((0.014, 0.024, 0.030)), origin=Origin(xyz=(0.068, -0.052, -0.043)), material=dark_plastic, name="paddle_bracket_1")
    model.articulation(
        "desktop_to_pod",
        ArticulationType.FIXED,
        parent=desktop,
        child=pod,
        origin=Origin(xyz=(0.0, -0.435, -0.075)),
    )

    button_specs = (
        ("preset_0", -0.112, 0.008, Box((0.034, 0.012, 0.018)), button_mat),
        ("preset_1", -0.066, 0.008, Box((0.034, 0.012, 0.018)), button_mat),
        ("preset_2", -0.020, 0.008, Box((0.034, 0.012, 0.018)), button_mat),
    )
    for label_index, (name, x, z, geom, mat) in enumerate(button_specs, start=1):
        button = model.part(name)
        button.visual(geom, origin=Origin(xyz=(0.0, -0.006, 0.0)), material=mat, name="button_cap")
        button.visual(Box((0.012, 0.076, 0.006)), origin=Origin(xyz=(0.0, 0.038, 0.0)), material=button_mat, name="button_stem")
        # Small raised bars stand in for preset legends without needing text geometry.
        button.visual(Box((0.006 * label_index, 0.0020, 0.003)), origin=Origin(xyz=(0.0, -0.0120, 0.004)), material=white_mark, name="preset_mark")
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=pod,
            child=button,
            origin=Origin(xyz=(x, -0.045, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=0.006),
        )

    power = model.part("power_button")
    power.visual(
        Cylinder(radius=0.0105, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=power_mat,
        name="power_cap",
    )
    power.visual(Box((0.010, 0.076, 0.006)), origin=Origin(xyz=(0.0, 0.038, 0.0)), material=button_mat, name="power_stem")
    power.visual(Box((0.012, 0.0020, 0.003)), origin=Origin(xyz=(0.0, -0.0120, 0.004)), material=white_mark, name="power_mark")
    model.articulation(
        "power_press",
        ArticulationType.PRISMATIC,
        parent=pod,
        child=power,
        origin=Origin(xyz=(0.072, -0.045, 0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=0.005),
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.008, length=0.122),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=black_metal,
        name="paddle_hinge_barrel",
    )
    paddle.visual(Box((0.108, 0.012, 0.060)), origin=Origin(xyz=(0.0, -0.005, -0.038)), material=dark_plastic, name="paddle_plate")
    model.articulation(
        "paddle_hinge",
        ArticulationType.REVOLUTE,
        parent=pod,
        child=paddle,
        origin=Origin(xyz=(0.0, -0.052, -0.043)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.0, lower=-0.35, upper=0.35),
    )

    # Keep a direct reference to the driving lift joint alive for static checkers.
    center_lift.meta["drives"] = ("lift_0", "lift_2")
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desktop = object_model.get_part("desktop_frame")
    pod = object_model.get_part("control_pod")
    lift = object_model.get_articulation("lift_1")

    for i in range(3):
        inner = object_model.get_part(f"inner_stage_{i}")
        ctx.expect_contact(
            inner,
            desktop,
            elem_a="top_bearing_plate",
            elem_b=f"socket_drop_{i}",
            contact_tol=0.001,
            name=f"inner stage {i} bears on common desktop frame",
        )
        inner_pos = ctx.part_world_position(inner)
        ctx.check(
            f"inner stage {i} centered in its column",
            inner_pos is not None and abs(inner_pos[0] - (-0.86, 0.0, 0.86)[i]) < 0.002 and abs(inner_pos[1]) < 0.002,
            details=f"inner_pos={inner_pos}",
        )
        ctx.expect_overlap(
            inner,
            "base_frame",
            axes="z",
            elem_a="inner_tube",
            elem_b=f"outer_front_wall_{i}",
            min_overlap=0.40,
            name=f"inner stage {i} retained in sleeve at seated height",
        )

    rest_pos = ctx.part_world_position(desktop)
    with ctx.pose({lift: 0.47}):
        raised_pos = ctx.part_world_position(desktop)
        for i in range(3):
            ctx.expect_contact(
                f"inner_stage_{i}",
                desktop,
                elem_a="top_bearing_plate",
                elem_b=f"socket_drop_{i}",
                contact_tol=0.001,
                name=f"raised inner stage {i} still supports desktop frame",
            )
            ctx.expect_overlap(
                f"inner_stage_{i}",
                "base_frame",
                axes="z",
                elem_a="inner_tube",
                elem_b=f"outer_front_wall_{i}",
                min_overlap=0.10,
                name=f"raised inner stage {i} remains inserted in sleeve",
            )
    ctx.check(
        "desktop raises on synchronized lift",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.45,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    ctx.expect_contact(pod, desktop, elem_a="edge_mount_tongue", elem_b="long_desktop", contact_tol=0.002, name="control pod mounts to front desktop underside")

    for name, travel in (("preset_0", 0.006), ("preset_1", 0.006), ("preset_2", 0.006), ("power_button", 0.005)):
        joint_name = "power_press" if name == "power_button" else f"{name}_press"
        joint = object_model.get_articulation(joint_name)
        part = object_model.get_part(name)
        at_rest = ctx.part_world_position(part)
        with ctx.pose({joint: travel}):
            depressed = ctx.part_world_position(part)
        ctx.check(
            f"{name} depresses independently into pod",
            at_rest is not None and depressed is not None and depressed[1] > at_rest[1] + travel * 0.8,
            details=f"rest={at_rest}, depressed={depressed}",
        )

    paddle = object_model.get_part("paddle")
    paddle_joint = object_model.get_articulation("paddle_hinge")
    neutral = ctx.part_world_aabb(paddle)
    with ctx.pose({paddle_joint: 0.30}):
        tilted = ctx.part_world_aabb(paddle)
    ctx.check(
        "front paddle pivots under pod",
        neutral is not None and tilted is not None and tilted[1][1] > neutral[1][1] + 0.010,
        details=f"neutral={neutral}, tilted={tilted}",
    )

    return ctx.report()


object_model = build_object_model()
