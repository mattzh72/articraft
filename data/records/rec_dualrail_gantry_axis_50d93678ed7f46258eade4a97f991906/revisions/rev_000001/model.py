from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_gantry_axis")

    aluminum = Material("clear_anodized_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_aluminum = Material("dark_hard_anodized_aluminum", rgba=(0.12, 0.13, 0.14, 1.0))
    ground_steel = Material("ground_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    blue_tool = Material("blue_tool_mount", rgba=(0.05, 0.19, 0.42, 1.0))
    warning_red = Material("red_stop_bumpers", rgba=(0.85, 0.08, 0.05, 1.0))

    # Fixed tabletop base.  The thin plate ties the two side rail supports
    # together so the moving gantry has a credible closed load path.
    base = model.part("base_frame")
    base.visual(
        Box((1.35, 0.75, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_aluminum,
        name="tabletop_plate",
    )
    for y in (-0.29, 0.29):
        base.visual(
            Box((1.24, 0.070, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.0575)),
            material=aluminum,
            name=f"rail_support_{'n' if y < 0 else 'p'}",
        )
        base.visual(
            Box((1.18, 0.040, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.091)),
            material=aluminum,
            name=f"machined_pad_{'n' if y < 0 else 'p'}",
        )
        base.visual(
            Box((1.12, 0.024, 0.024)),
            origin=Origin(xyz=(0.0, y, 0.109)),
            material=ground_steel,
            name="linear_rail_n" if y < 0 else "linear_rail_p",
        )
        for x in (-0.58, 0.58):
            base.visual(
                Box((0.040, 0.074, 0.070)),
                origin=Origin(xyz=(x, y, 0.132)),
                material=warning_red,
                name=(
                    "rail_stop_n_n"
                    if y < 0 and x < 0
                    else "rail_stop_n_p"
                    if y < 0
                    else "rail_stop_p_n"
                    if x < 0
                    else "rail_stop_p_p"
                ),
            )
        for x in (-0.42, -0.21, 0.0, 0.21, 0.42):
            base.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(x, y, 0.094)),
                material=dark_aluminum,
                name=f"rail_screw_{'n' if y < 0 else 'p'}_{x:+.2f}",
            )

    # First moving stage: a stiff bridge beam, much larger than the tool sled,
    # riding on four U-shaped guide block assemblies over the side rails.
    bridge = model.part("bridge")
    bridge.visual(
        Box((0.190, 0.740, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=aluminum,
        name="bridge_beam",
    )
    bridge.visual(
        Box((0.210, 0.700, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.303)),
        material=dark_aluminum,
        name="lower_flange",
    )
    for y in (-0.29, 0.29):
        bridge.visual(
            Box((0.170, 0.040, 0.210)),
            origin=Origin(xyz=(0.0, y, 0.248)),
            material=aluminum,
            name=f"end_upright_{'n' if y < 0 else 'p'}",
        )
        bridge.visual(
            Box((0.150, 0.022, 0.150)),
            origin=Origin(xyz=(-0.080, y * 0.86, 0.282)),
            material=dark_aluminum,
            name=f"stiffener_front_{'n' if y < 0 else 'p'}",
        )
        bridge.visual(
            Box((0.150, 0.022, 0.150)),
            origin=Origin(xyz=(0.080, y * 0.86, 0.282)),
            material=dark_aluminum,
            name=f"stiffener_rear_{'n' if y < 0 else 'p'}",
        )
        for x in (-0.055, 0.055):
            bridge.visual(
                Box((0.120, 0.076, 0.027)),
                origin=Origin(xyz=(x, y, 0.1345)),
                material=dark_aluminum,
                name=(
                    "guide_cap_n_n"
                    if y < 0 and x < 0
                    else "guide_cap_n_p"
                    if y < 0
                    else "guide_cap_p_n"
                    if x < 0
                    else "guide_cap_p_p"
                ),
            )
            for side in (-1.0, 1.0):
                bridge.visual(
                    Box((0.108, 0.012, 0.046)),
                    origin=Origin(xyz=(x, y + side * 0.044, 0.127)),
                    material=dark_aluminum,
                    name=(
                        f"guide_cheek_{'n' if y < 0 else 'p'}_"
                        f"{'n' if x < 0 else 'p'}_{'i' if side < 0 else 'o'}"
                    ),
                )
            for wx in (-0.065, 0.065):
                bridge.visual(
                    Box((0.010, 0.082, 0.030)),
                    origin=Origin(xyz=(x + wx, y, 0.149)),
                    material=black_rubber,
                    name=(
                        f"rail_wiper_{'n' if y < 0 else 'p'}_"
                        f"{'n' if x < 0 else 'p'}_{'n' if wx < 0 else 'p'}"
                    ),
                )

    # Sled axis rails and practical travel stops are carried by the bridge.
    for x in (-0.060, 0.060):
        bridge.visual(
            Box((0.020, 0.560, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.284)),
            material=ground_steel,
            name="sled_rail_n" if x < 0 else "sled_rail_p",
        )
    for y in (-0.315, 0.315):
        bridge.visual(
            Box((0.230, 0.028, 0.070)),
            origin=Origin(xyz=(0.0, y, 0.258)),
            material=warning_red,
            name=f"sled_stop_{'n' if y < 0 else 'p'}",
        )

    # Second moving stage: compact hanging tool sled.  Its scale contrasts
    # clearly with the bridge, making the two prismatic motions easy to read.
    sled = model.part("tool_sled")
    sled.visual(
        Box((0.200, 0.165, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.232)),
        material=blue_tool,
        name="carriage_plate",
    )
    for x in (-0.060, 0.060):
        sled.visual(
            Box((0.052, 0.120, 0.034)),
            origin=Origin(xyz=(x, 0.0, 0.258)),
            material=dark_aluminum,
            name="sled_block_n" if x < 0 else "sled_block_p",
        )
        for y in (-0.066, 0.066):
            sled.visual(
                Box((0.058, 0.008, 0.038)),
                origin=Origin(xyz=(x, y, 0.253)),
                material=black_rubber,
                name=f"sled_wiper_{'n' if x < 0 else 'p'}_{'n' if y < 0 else 'p'}",
            )
    sled.visual(
        Box((0.170, 0.032, 0.210)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=blue_tool,
        name="tool_plate",
    )
    sled.visual(
        Box((0.100, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=dark_aluminum,
        name="tool_clamp",
    )
    sled.visual(
        Cylinder(radius=0.026, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=ground_steel,
        name="spindle_body",
    )
    sled.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=ground_steel,
        name="tool_tip",
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "bridge_to_sled",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=sled,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=-0.21, upper=0.21),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    bridge = object_model.get_part("bridge")
    sled = object_model.get_part("tool_sled")
    x_axis = object_model.get_articulation("base_to_bridge")
    y_axis = object_model.get_articulation("bridge_to_sled")

    ctx.expect_overlap(
        bridge,
        base,
        axes="y",
        elem_a="guide_cap_n_n",
        elem_b="linear_rail_n",
        min_overlap=0.020,
        name="bridge guide blocks align over side rails",
    )
    ctx.expect_contact(
        bridge,
        base,
        elem_a="guide_cap_n_n",
        elem_b="linear_rail_n",
        contact_tol=0.001,
        name="bridge guide blocks ride on rail tops",
    )
    ctx.expect_contact(
        sled,
        bridge,
        elem_a="sled_block_n",
        elem_b="sled_rail_n",
        contact_tol=0.001,
        name="hanging sled blocks ride just below beam rails",
    )

    def _elem_minmax(part, elem, axis_index):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return aabb[0][axis_index], aabb[1][axis_index]

    for q in (-0.42, 0.42):
        with ctx.pose({x_axis: q}):
            rear = _elem_minmax(bridge, "guide_cap_n_n", 0)
            front = _elem_minmax(bridge, "guide_cap_n_p", 0)
            neg_stop = _elem_minmax(base, "rail_stop_n_n", 0)
            pos_stop = _elem_minmax(base, "rail_stop_n_p", 0)
            ok = (
                rear is not None
                and front is not None
                and neg_stop is not None
                and pos_stop is not None
                and rear[0] > neg_stop[1] + 0.020
                and front[1] < pos_stop[0] - 0.020
            )
            ctx.check(
                f"bridge travel {q:+.2f} m stays inside rail stops",
                ok,
                details=f"rear={rear}, front={front}, neg_stop={neg_stop}, pos_stop={pos_stop}",
            )

    for q in (-0.21, 0.21):
        with ctx.pose({y_axis: q}):
            block = _elem_minmax(sled, "sled_block_n", 1)
            neg_stop = _elem_minmax(bridge, "sled_stop_n", 1)
            pos_stop = _elem_minmax(bridge, "sled_stop_p", 1)
            ok = (
                block is not None
                and neg_stop is not None
                and pos_stop is not None
                and block[0] > neg_stop[1] + 0.010
                and block[1] < pos_stop[0] - 0.010
            )
            ctx.check(
                f"sled travel {q:+.2f} m stays between beam stops",
                ok,
                details=f"block={block}, neg_stop={neg_stop}, pos_stop={pos_stop}",
            )

    return ctx.report()


object_model = build_object_model()
