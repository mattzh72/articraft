from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metrology_bridge_axis")

    cast_gray = model.material("painted_cast_gray", rgba=(0.42, 0.45, 0.46, 1.0))
    dark_gray = model.material("dark_anodized_gray", rgba=(0.10, 0.11, 0.12, 1.0))
    rail_steel = model.material("ground_rail_steel", rgba=(0.76, 0.78, 0.76, 1.0))
    cap_black = model.material("black_bearing_caps", rgba=(0.02, 0.025, 0.03, 1.0))
    bolt_steel = model.material("brushed_bolt_heads", rgba=(0.62, 0.63, 0.60, 1.0))
    sensor_blue = model.material("blue_sensor_tabs", rgba=(0.05, 0.20, 0.80, 1.0))
    warning_yellow = model.material("yellow_limit_markers", rgba=(0.95, 0.72, 0.08, 1.0))

    # Root: a fixed precision deck with low raised side frames and two exposed
    # ground rails.  The dimensions are roughly a compact benchtop CMM axis.
    fixed_deck = model.part("fixed_deck")
    fixed_deck.visual(
        Box((1.80, 1.55, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_gray,
        name="deck_plate",
    )
    fixed_deck.visual(
        Box((1.68, 0.055, 0.014)),
        origin=Origin(xyz=(0.0, -0.705, 0.091)),
        material=dark_gray,
        name="rear_cover_strip",
    )
    fixed_deck.visual(
        Box((1.68, 0.055, 0.014)),
        origin=Origin(xyz=(0.0, 0.705, 0.091)),
        material=dark_gray,
        name="front_cover_strip",
    )

    for x, suffix in [(-0.62, "0"), (0.62, "1")]:
        fixed_deck.visual(
            Box((0.18, 1.40, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.120)),
            material=cast_gray,
            name=f"side_frame_{suffix}",
        )
        fixed_deck.visual(
            Box((0.058, 1.24, 0.042)),
            origin=Origin(xyz=(x, 0.0, 0.181)),
            material=rail_steel,
            name=f"linear_rail_{suffix}",
        )
        fixed_deck.visual(
            Box((0.090, 0.055, 0.082)),
            origin=Origin(xyz=(x, -0.666, 0.201)),
            material=cap_black,
            name=f"rear_stop_{suffix}",
        )
        fixed_deck.visual(
            Box((0.090, 0.055, 0.082)),
            origin=Origin(xyz=(x, 0.666, 0.201)),
            material=cap_black,
            name=f"front_stop_{suffix}",
        )
        fixed_deck.visual(
            Box((0.030, 0.135, 0.035)),
            origin=Origin(xyz=(x + (0.105 if x < 0 else -0.105), -0.515, 0.174)),
            material=sensor_blue,
            name=f"home_sensor_tab_{suffix}",
        )
        fixed_deck.visual(
            Box((0.030, 0.135, 0.035)),
            origin=Origin(xyz=(x + (0.105 if x < 0 else -0.105), 0.515, 0.174)),
            material=warning_yellow,
            name=f"limit_marker_{suffix}",
        )
        for y in (-0.45, -0.15, 0.15, 0.45):
            fixed_deck.visual(
                Cylinder(radius=0.014, length=0.010),
                origin=Origin(xyz=(x + (-0.070 if x < 0 else 0.070), y, 0.165)),
                material=bolt_steel,
                name=f"rail_bolt_{suffix}_{int((y + 0.60) * 100):02d}",
            )

    bridge = model.part("bridge")
    for x, suffix in [(-0.62, "0"), (0.62, "1")]:
        bridge.visual(
            Box((0.205, 0.220, 0.066)),
            origin=Origin(xyz=(x, 0.0, 0.235)),
            material=dark_gray,
            name=f"bearing_block_{suffix}",
        )
        bridge.visual(
            Box((0.075, 0.240, 0.052)),
            origin=Origin(xyz=(x - (0.090 if x < 0 else -0.090), 0.0, 0.247)),
            material=cap_black,
            name=f"outer_bearing_cap_{suffix}",
        )
        bridge.visual(
            Box((0.060, 0.200, 0.040)),
            origin=Origin(xyz=(x + (0.090 if x < 0 else -0.090), 0.0, 0.242)),
            material=cap_black,
            name=f"inner_bearing_cap_{suffix}",
        )
        bridge.visual(
            Box((0.145, 0.180, 0.535)),
            origin=Origin(xyz=(x, 0.0, 0.530)),
            material=cast_gray,
            name=f"upright_{suffix}",
        )
        bridge.visual(
            Box((0.185, 0.055, 0.040)),
            origin=Origin(xyz=(x, -0.137, 0.286)),
            material=cap_black,
            name=f"front_wiper_{suffix}",
        )
        for y in (-0.065, 0.065):
            bridge.visual(
                Cylinder(radius=0.013, length=0.011),
                origin=Origin(xyz=(x, y, 0.279)),
                material=bolt_steel,
                name=f"bearing_bolt_{suffix}_{0 if y < 0 else 1}",
            )

    bridge.visual(
        Box((1.42, 0.190, 0.200)),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=cast_gray,
        name="crossbeam_body",
    )
    bridge.visual(
        Box((1.18, 0.034, 0.050)),
        origin=Origin(xyz=(0.0, -0.112, 0.805)),
        material=rail_steel,
        name="front_rail_top",
    )
    bridge.visual(
        Box((1.18, 0.034, 0.050)),
        origin=Origin(xyz=(0.0, -0.112, 0.665)),
        material=rail_steel,
        name="front_rail_bottom",
    )
    bridge.visual(
        Box((1.28, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, 0.103, 0.735)),
        material=dark_gray,
        name="rear_beam_cover",
    )
    bridge.visual(
        Box((0.050, 0.090, 0.180)),
        origin=Origin(xyz=(-0.645, -0.112, 0.735)),
        material=cap_black,
        name="beam_end_cap_0",
    )
    bridge.visual(
        Box((0.050, 0.090, 0.180)),
        origin=Origin(xyz=(0.645, -0.112, 0.735)),
        material=cap_black,
        name="beam_end_cap_1",
    )
    for x in (-0.42, -0.14, 0.14, 0.42):
        bridge.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(x, -0.134, 0.805)),
            material=bolt_steel,
            name=f"beam_rail_bolt_top_{int((x + 0.50) * 100):02d}",
        )
        bridge.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(x, -0.134, 0.665)),
            material=bolt_steel,
            name=f"beam_rail_bolt_bottom_{int((x + 0.50) * 100):02d}",
        )

    truck = model.part("truck")
    truck.visual(
        Box((0.245, 0.120, 0.205)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_gray,
        name="saddle",
    )
    truck.visual(
        Box((0.210, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, -0.070, 0.070)),
        material=cap_black,
        name="upper_bearing_cap",
    )
    truck.visual(
        Box((0.210, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, -0.070, -0.070)),
        material=cap_black,
        name="lower_bearing_cap",
    )
    truck.visual(
        Box((0.118, 0.090, 0.185)),
        origin=Origin(xyz=(0.0, -0.012, -0.195)),
        material=cast_gray,
        name="center_tool_plate",
    )
    truck.visual(
        Box((0.070, 0.072, 0.055)),
        origin=Origin(xyz=(0.0, -0.012, -0.315)),
        material=cap_black,
        name="probe_mount_boss",
    )
    truck.visual(
        Box((0.155, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.069, 0.0)),
        material=sensor_blue,
        name="reader_head_tab",
    )
    for x in (-0.070, 0.070):
        for z in (-0.070, 0.070):
            truck.visual(
                Cylinder(radius=0.009, length=0.010),
                origin=Origin(xyz=(x, -0.065, z)),
                material=bolt_steel,
                name=f"saddle_bolt_{0 if x < 0 else 1}_{0 if z < 0 else 1}",
            )

    bridge_slide = model.articulation(
        "deck_to_bridge",
        ArticulationType.PRISMATIC,
        parent=fixed_deck,
        child=bridge,
        origin=Origin(xyz=(0.0, -0.420, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=0.840),
    )
    bridge_slide.meta["qc_samples"] = [0.0, 0.42, 0.84]

    truck_slide = model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(0.0, -0.189, 0.735)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.45, lower=-0.420, upper=0.420),
    )
    truck_slide.meta["qc_samples"] = [-0.42, 0.0, 0.42]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_deck = object_model.get_part("fixed_deck")
    bridge = object_model.get_part("bridge")
    truck = object_model.get_part("truck")
    bridge_slide = object_model.get_articulation("deck_to_bridge")
    truck_slide = object_model.get_articulation("bridge_to_truck")

    # The bridge bearing blocks ride just above the deck rails and stay between
    # the end stops at both ends of travel.
    for suffix in ("0", "1"):
        ctx.expect_gap(
            bridge,
            fixed_deck,
            axis="z",
            positive_elem=f"bearing_block_{suffix}",
            negative_elem=f"linear_rail_{suffix}",
            min_gap=0.0,
            max_gap=0.001,
            name=f"bridge bearing {suffix} clears rail vertically",
        )
        ctx.expect_overlap(
            bridge,
            fixed_deck,
            axes="xy",
            elem_a=f"bearing_block_{suffix}",
            elem_b=f"linear_rail_{suffix}",
            min_overlap=0.050,
            name=f"bridge bearing {suffix} follows deck rail",
        )
        ctx.expect_gap(
            bridge,
            fixed_deck,
            axis="y",
            positive_elem=f"bearing_block_{suffix}",
            negative_elem=f"rear_stop_{suffix}",
            min_gap=0.080,
            name=f"bridge bearing {suffix} clear of rear stop",
        )

    bridge_rest = ctx.part_world_position(bridge)
    with ctx.pose({bridge_slide: 0.840}):
        bridge_far = ctx.part_world_position(bridge)
        for suffix in ("0", "1"):
            ctx.expect_gap(
                fixed_deck,
                bridge,
                axis="y",
                positive_elem=f"front_stop_{suffix}",
                negative_elem=f"bearing_block_{suffix}",
                min_gap=0.080,
                name=f"bridge bearing {suffix} clear of front stop at travel",
            )
            ctx.expect_gap(
                bridge,
                fixed_deck,
                axis="z",
                positive_elem=f"bearing_block_{suffix}",
                negative_elem=f"linear_rail_{suffix}",
                min_gap=0.0,
                max_gap=0.001,
                name=f"bridge bearing {suffix} rail clearance at travel",
            )
    ctx.check(
        "bridge slide moves along deck rails",
        bridge_rest is not None and bridge_far is not None and bridge_far[1] > bridge_rest[1] + 0.80,
        details=f"rest={bridge_rest}, far={bridge_far}",
    )

    # The compact center truck rides on the front crossbeam guide hardware with
    # a small visible running clearance, and remains on the beam at both limits.
    for q, label in [(-0.420, "negative"), (0.0, "center"), (0.420, "positive")]:
        with ctx.pose({truck_slide: q}):
            ctx.expect_gap(
                bridge,
                truck,
                axis="y",
                positive_elem="front_rail_top",
                negative_elem="saddle",
                max_gap=0.002,
                max_penetration=1e-6,
                name=f"truck top rail running clearance {label}",
            )
            ctx.expect_gap(
                bridge,
                truck,
                axis="y",
                positive_elem="front_rail_bottom",
                negative_elem="saddle",
                max_gap=0.002,
                max_penetration=1e-6,
                name=f"truck lower rail running clearance {label}",
            )
            ctx.expect_overlap(
                truck,
                bridge,
                axes="xz",
                elem_a="saddle",
                elem_b="front_rail_top",
                min_overlap=0.045,
                name=f"truck remains engaged with top beam rail {label}",
            )

    truck_center = ctx.part_world_position(truck)
    with ctx.pose({truck_slide: 0.420}):
        truck_positive = ctx.part_world_position(truck)
    ctx.check(
        "truck slide moves across crossbeam",
        truck_center is not None
        and truck_positive is not None
        and truck_positive[0] > truck_center[0] + 0.40,
        details=f"center={truck_center}, positive={truck_positive}",
    )

    return ctx.report()


object_model = build_object_model()
