from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_espresso_machine")

    dark_panel = model.material("dark_panel", rgba=(0.16, 0.17, 0.19, 1.0))
    stainless = model.material("stainless", rgba=(0.80, 0.81, 0.82, 1.0))
    black_grip = model.material("black_grip", rgba=(0.10, 0.10, 0.11, 1.0))
    black_trim = model.material("black_trim", rgba=(0.08, 0.08, 0.09, 1.0))

    def add_box(
        part,
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def add_cylinder(
        part,
        name: str,
        radius: float,
        length: float,
        xyz: tuple[float, float, float],
        material,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    body = model.part("body")
    add_box(body, "lower_base", (0.18, 0.60, 0.10), (-0.11, 0.0, 0.05), dark_panel)
    add_box(body, "side_rail_0", (0.22, 0.03, 0.10), (0.09, -0.285, 0.05), dark_panel)
    add_box(body, "side_rail_1", (0.22, 0.03, 0.10), (0.09, 0.285, 0.05), dark_panel)
    add_box(body, "foot_0", (0.05, 0.05, 0.02), (-0.15, -0.22, 0.01), black_trim)
    add_box(body, "foot_1", (0.05, 0.05, 0.02), (-0.15, 0.22, 0.01), black_trim)
    add_box(body, "foot_2", (0.05, 0.05, 0.02), (0.09, -0.315, 0.01), black_trim)
    add_box(body, "foot_3", (0.05, 0.05, 0.02), (0.09, 0.315, 0.01), black_trim)
    add_box(body, "housing", (0.30, 0.60, 0.26), (-0.03, 0.0, 0.23), dark_panel)
    add_box(
        body,
        "fascia",
        (0.14, 0.42, 0.18),
        (0.06, 0.0, 0.21),
        stainless,
        rpy=(0.0, -0.30, 0.0),
    )
    add_box(body, "top_front_strip", (0.08, 0.60, 0.02), (0.10, 0.0, 0.37), stainless)
    add_box(body, "cup_rail_post_0", (0.02, 0.02, 0.055), (0.13, -0.22, 0.3975), stainless)
    add_box(body, "cup_rail_post_1", (0.02, 0.02, 0.055), (0.13, 0.00, 0.3975), stainless)
    add_box(body, "cup_rail_post_2", (0.02, 0.02, 0.055), (0.13, 0.22, 0.3975), stainless)
    add_box(body, "cup_rail_bar", (0.02, 0.50, 0.02), (0.13, 0.0, 0.435), stainless)
    add_box(body, "group_mount", (0.04, 0.16, 0.07), (0.155, 0.0, 0.185), stainless)
    add_cylinder(
        body,
        "group_head",
        radius=0.045,
        length=0.06,
        xyz=(0.205, 0.0, 0.18),
        material=stainless,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    add_box(body, "steam_support", (0.024, 0.035, 0.03), (0.126, 0.25, 0.236), stainless)
    add_box(body, "water_support", (0.024, 0.035, 0.03), (0.126, -0.25, 0.236), stainless)

    service_cover = model.part("service_cover")
    add_box(service_cover, "cover_panel", (0.20, 0.54, 0.018), (0.10, 0.0, 0.009), stainless)
    add_box(service_cover, "cover_handle", (0.04, 0.16, 0.012), (0.17, 0.0, 0.020), black_trim)

    portafilter = model.part("portafilter")
    add_cylinder(portafilter, "collar", 0.040, 0.018, (0.0, 0.0, -0.009), stainless)
    add_cylinder(portafilter, "basket", 0.033, 0.040, (0.0, 0.0, -0.038), stainless)
    add_box(portafilter, "spout", (0.028, 0.018, 0.020), (-0.004, 0.0, -0.068), stainless)
    add_box(portafilter, "neck", (0.05, 0.034, 0.022), (0.026, 0.0, -0.030), stainless)
    add_cylinder(
        portafilter,
        "handle",
        0.015,
        0.18,
        (0.115, 0.0, -0.032),
        black_grip,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    add_cylinder(
        portafilter,
        "grip_cap",
        0.018,
        0.045,
        (0.192, 0.0, -0.032),
        black_trim,
        rpy=(0.0, pi / 2.0, 0.0),
    )

    steam_wand = model.part("steam_wand")
    add_box(steam_wand, "wand_pivot", (0.024, 0.03, 0.024), (0.0, 0.0, 0.0), stainless)
    steam_wand.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.014, 0.0, -0.014)),
        material=stainless,
        name="wand_elbow",
    )
    add_cylinder(steam_wand, "tube", 0.0055, 0.19, (0.020, 0.0, -0.109), stainless)
    add_box(steam_wand, "wand_tip", (0.016, 0.012, 0.014), (0.028, 0.0, -0.199), stainless)

    water_wand = model.part("water_wand")
    add_box(water_wand, "wand_pivot", (0.024, 0.03, 0.024), (0.0, 0.0, 0.0), stainless)
    water_wand.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.012, 0.0, -0.012)),
        material=stainless,
        name="wand_elbow",
    )
    add_cylinder(water_wand, "tube", 0.0045, 0.17, (0.016, 0.0, -0.097), stainless)
    add_box(water_wand, "wand_tip", (0.014, 0.010, 0.012), (0.024, 0.0, -0.181), stainless)

    drip_tray = model.part("drip_tray")
    add_box(drip_tray, "tray_floor", (0.18, 0.54, 0.006), (0.0, 0.0, 0.003), stainless)
    add_box(drip_tray, "tray_wall_0", (0.18, 0.01, 0.05), (0.0, -0.265, 0.025), stainless)
    add_box(drip_tray, "tray_wall_1", (0.18, 0.01, 0.05), (0.0, 0.265, 0.025), stainless)
    add_box(drip_tray, "tray_wall_rear", (0.01, 0.54, 0.05), (-0.085, 0.0, 0.025), stainless)
    add_box(drip_tray, "tray_face", (0.01, 0.54, 0.020), (0.085, 0.0, 0.010), stainless)
    add_box(drip_tray, "tray_grate", (0.16, 0.50, 0.004), (0.0, 0.0, 0.028), dark_panel)
    add_box(drip_tray, "tray_pull", (0.018, 0.50, 0.012), (0.094, 0.0, 0.012), black_trim)

    model.articulation(
        "body_to_service_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_cover,
        origin=Origin(xyz=(-0.15, 0.0, 0.360)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.205, 0.0, 0.135)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-1.0, upper=0.35),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.15, 0.25, 0.236)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.1),
    )
    model.articulation(
        "body_to_water_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=water_wand,
        origin=Origin(xyz=(0.15, -0.25, 0.236)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.0),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.30, lower=0.0, upper=0.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    service_cover = object_model.get_part("service_cover")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    water_wand = object_model.get_part("water_wand")
    drip_tray = object_model.get_part("drip_tray")

    cover_joint = object_model.get_articulation("body_to_service_cover")
    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    steam_joint = object_model.get_articulation("body_to_steam_wand")
    water_joint = object_model.get_articulation("body_to_water_wand")
    tray_joint = object_model.get_articulation("body_to_drip_tray")

    def elem_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        service_cover,
        body,
        axis="z",
        positive_elem="cover_panel",
        negative_elem="housing",
        min_gap=0.0,
        max_gap=0.005,
        name="service cover rests just above the housing",
    )
    ctx.expect_overlap(
        service_cover,
        body,
        axes="xy",
        elem_a="cover_panel",
        elem_b="housing",
        min_overlap=0.18,
        name="service cover spans the top opening",
    )
    ctx.expect_overlap(
        portafilter,
        body,
        axes="xy",
        elem_a="collar",
        elem_b="group_head",
        min_overlap=0.05,
        name="portafilter stays centered under the group head",
    )
    ctx.expect_contact(
        portafilter,
        body,
        elem_a="collar",
        elem_b="group_head",
        contact_tol=0.0015,
        name="portafilter collar locks into the group head",
    )
    ctx.expect_overlap(
        drip_tray,
        body,
        axes="x",
        elem_a="tray_floor",
        elem_b="side_rail_0",
        min_overlap=0.17,
        name="closed drip tray remains captured in the base rails",
    )
    ctx.expect_contact(
        steam_wand,
        body,
        elem_a="wand_pivot",
        elem_b="steam_support",
        name="steam wand is mounted to its support block",
    )
    ctx.expect_contact(
        water_wand,
        body,
        elem_a="wand_pivot",
        elem_b="water_support",
        name="hot water wand is mounted to its support block",
    )

    closed_cover = ctx.part_element_world_aabb(service_cover, elem="cover_panel")
    with ctx.pose({cover_joint: 1.20}):
        opened_cover = ctx.part_element_world_aabb(service_cover, elem="cover_panel")
    closed_cover_center = elem_center(closed_cover)
    opened_cover_center = elem_center(opened_cover)
    ctx.check(
        "service cover opens upward",
        closed_cover_center is not None
        and opened_cover_center is not None
        and opened_cover_center[2] > closed_cover_center[2] + 0.08
        and opened_cover_center[0] < closed_cover_center[0] - 0.05,
        details=f"closed={closed_cover_center}, opened={opened_cover_center}",
    )

    with ctx.pose({portafilter_joint: -0.90}):
        pf_low = elem_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    with ctx.pose({portafilter_joint: 0.25}):
        pf_high = elem_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    ctx.check(
        "portafilter handle swings around the brew axis",
        pf_low is not None and pf_high is not None and abs(pf_high[1] - pf_low[1]) > 0.10,
        details=f"low={pf_low}, high={pf_high}",
    )

    tray_rest = ctx.part_world_position(drip_tray)
    with ctx.pose({tray_joint: 0.14}):
        tray_extended = ctx.part_world_position(drip_tray)
        ctx.expect_overlap(
            drip_tray,
            body,
            axes="x",
            elem_a="tray_floor",
            elem_b="side_rail_0",
            min_overlap=0.045,
            name="extended drip tray still retains insertion in the base rails",
        )
    ctx.check(
        "drip tray slides forward",
        tray_rest is not None and tray_extended is not None and tray_extended[0] > tray_rest[0] + 0.10,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    steam_rest = elem_center(ctx.part_element_world_aabb(steam_wand, elem="wand_tip"))
    with ctx.pose({steam_joint: 0.90}):
        steam_swung = elem_center(ctx.part_element_world_aabb(steam_wand, elem="wand_tip"))
    ctx.check(
        "steam wand swings forward from its pivot",
        steam_rest is not None and steam_swung is not None and steam_swung[0] > steam_rest[0] + 0.08,
        details=f"rest={steam_rest}, swung={steam_swung}",
    )

    water_rest = elem_center(ctx.part_element_world_aabb(water_wand, elem="wand_tip"))
    with ctx.pose({water_joint: 0.85}):
        water_swung = elem_center(ctx.part_element_world_aabb(water_wand, elem="wand_tip"))
    ctx.check(
        "hot water wand swings forward from its pivot",
        water_rest is not None and water_swung is not None and water_swung[0] > water_rest[0] + 0.06,
        details=f"rest={water_rest}, swung={water_swung}",
    )

    return ctx.report()


object_model = build_object_model()
