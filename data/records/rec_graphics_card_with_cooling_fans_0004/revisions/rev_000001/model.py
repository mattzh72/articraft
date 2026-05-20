from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BODY_LENGTH = 0.324
BODY_HEIGHT = 0.128
PCB_LENGTH = 0.314
PCB_HEIGHT = 0.114
HEATSINK_LENGTH = 0.312
HEATSINK_HEIGHT = 0.118
TOP_PLATE_LENGTH = 0.318
TOP_PLATE_WIDTH = 0.112
SHROUD_BASE_WIDTH = 0.126
FAN_CENTERS_X = (-0.104, 0.0, 0.104)
ROTOR_RADIUS = 0.039
ROTOR_Z = 0.024


def _add_fan_blades(part, material) -> None:
    blade_count = 9
    for index in range(blade_count):
        angle = (2.0 * math.pi * index) / blade_count
        part.visual(
            Box((0.040, 0.009, 0.002)),
            origin=Origin(
                xyz=(0.022 * math.cos(angle), 0.022 * math.sin(angle), 0.0),
                rpy=(0.0, 0.0, angle + 0.55),
            ),
            material=material,
            name="blades" if index == 0 else f"blade_{index}",
        )


def _add_shroud_opening_braces(
    part,
    *,
    fan_center_x: float,
    left_anchor_x: float,
    right_anchor_x: float,
    prefix: str,
    material,
) -> None:
    brace_specs = (
        ((fan_center_x - 0.029, 0.034), (left_anchor_x, 0.046), f"{prefix}_top_left"),
        ((fan_center_x + 0.029, 0.034), (right_anchor_x, 0.046), f"{prefix}_top_right"),
        ((fan_center_x - 0.029, -0.034), (left_anchor_x, -0.046), f"{prefix}_bottom_left"),
        ((fan_center_x + 0.029, -0.034), (right_anchor_x, -0.046), f"{prefix}_bottom_right"),
    )
    for (x0, y0), (x1, y1), name in brace_specs:
        dx = x1 - x0
        dy = y1 - y0
        length = math.hypot(dx, dy)
        part.visual(
            Box((length, 0.006, 0.004)),
            origin=Origin(
                xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, 0.031),
                rpy=(0.0, 0.0, math.atan2(dy, dx)),
            ),
            material=material,
            name=name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fan_graphics_card", assets=ASSETS)

    pcb_green = model.material("pcb_green", rgba=(0.08, 0.27, 0.18, 1.0))
    heatsink_black = model.material("heatsink_black", rgba=(0.15, 0.16, 0.17, 1.0))
    shroud_black = model.material("shroud_black", rgba=(0.12, 0.12, 0.13, 1.0))
    backplate_gunmetal = model.material("backplate_gunmetal", rgba=(0.32, 0.34, 0.37, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.81, 0.68, 0.28, 1.0))
    port_black = model.material("port_black", rgba=(0.07, 0.07, 0.08, 1.0))
    fan_black = model.material("fan_black", rgba=(0.09, 0.09, 0.10, 1.0))

    card_core = model.part("card_core")
    pcb = card_core.visual(
        Box((PCB_LENGTH, PCB_HEIGHT, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pcb_green,
        name="pcb",
    )
    heatsink = card_core.visual(
        Box((HEATSINK_LENGTH, HEATSINK_HEIGHT, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=heatsink_black,
        name="heatsink",
    )
    card_core.visual(
        Box((0.090, 0.008, 0.001)),
        origin=Origin(xyz=(-0.032, -0.061, 0.0005)),
        material=connector_gold,
        name="pcie_fingers",
    )
    for index, y_pos in enumerate((0.028, 0.0, -0.028), start=1):
        card_core.visual(
            Box((0.006, 0.016, 0.008)),
            origin=Origin(xyz=(-0.153, y_pos, 0.012)),
            material=port_black,
            name=f"port_block_{index}",
        )
    for index, fan_center_x in enumerate(FAN_CENTERS_X, start=1):
        card_core.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(fan_center_x, 0.0, 0.019)),
            material=heatsink_black,
            name=f"fan{index}_mount",
        )
    card_core.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_HEIGHT, 0.026)),
        mass=1.65,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    shroud = model.part("shroud")
    top_plate = shroud.visual(
        Box((TOP_PLATE_LENGTH, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.046, 0.031)),
        material=shroud_black,
        name="top_plate",
    )
    shroud.visual(
        Box((TOP_PLATE_LENGTH, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, -0.046, 0.031)),
        material=shroud_black,
        name="rear_top_rail",
    )
    shroud.visual(
        Box((0.012, 0.080, 0.004)),
        origin=Origin(xyz=(-0.052, 0.0, 0.031)),
        material=shroud_black,
        name="left_bridge",
    )
    shroud.visual(
        Box((0.012, 0.080, 0.004)),
        origin=Origin(xyz=(0.052, 0.0, 0.031)),
        material=shroud_black,
        name="right_bridge",
    )
    shroud.visual(
        Box((0.012, 0.080, 0.004)),
        origin=Origin(xyz=(-0.150, 0.0, 0.031)),
        material=shroud_black,
        name="left_end_frame",
    )
    shroud.visual(
        Box((0.012, 0.080, 0.004)),
        origin=Origin(xyz=(0.150, 0.0, 0.031)),
        material=shroud_black,
        name="right_end_frame",
    )
    shroud.visual(
        Box((HEATSINK_LENGTH, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.050, 0.027), rpy=(math.radians(-24.0), 0.0, 0.0)),
        material=shroud_black,
        name="left_bevel",
    )
    shroud.visual(
        Box((HEATSINK_LENGTH, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.050, 0.027), rpy=(math.radians(24.0), 0.0, 0.0)),
        material=shroud_black,
        name="right_bevel",
    )
    left_skirt = shroud.visual(
        Box((HEATSINK_LENGTH, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.055, 0.024)),
        material=shroud_black,
        name="left_skirt",
    )
    right_skirt = shroud.visual(
        Box((HEATSINK_LENGTH, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, -0.055, 0.024)),
        material=shroud_black,
        name="right_skirt",
    )
    shroud.visual(
        Box((0.024, 0.022, 0.010)),
        origin=Origin(xyz=(-0.129, 0.043, 0.024)),
        material=shroud_black,
        name="io_end_cap",
    )
    shroud.visual(
        Box((0.036, 0.018, 0.010)),
        origin=Origin(xyz=(0.118, 0.044, 0.024)),
        material=shroud_black,
        name="tail_cap",
    )
    _add_shroud_opening_braces(
        shroud,
        fan_center_x=-0.104,
        left_anchor_x=-0.150,
        right_anchor_x=-0.052,
        prefix="fan_left_opening",
        material=shroud_black,
    )
    _add_shroud_opening_braces(
        shroud,
        fan_center_x=0.0,
        left_anchor_x=-0.052,
        right_anchor_x=0.052,
        prefix="fan_center_opening",
        material=shroud_black,
    )
    _add_shroud_opening_braces(
        shroud,
        fan_center_x=0.104,
        left_anchor_x=0.052,
        right_anchor_x=0.150,
        prefix="fan_right_opening",
        material=shroud_black,
    )
    shroud.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, SHROUD_BASE_WIDTH, 0.014)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    backplate = model.part("backplate")
    back_plate = backplate.visual(
        Box((BODY_LENGTH, BODY_HEIGHT, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=backplate_gunmetal,
        name="plate",
    )
    backplate.visual(
        Box((0.236, 0.016, 0.0012)),
        origin=Origin(xyz=(0.018, 0.0, -0.0036)),
        material=backplate_gunmetal,
        name="center_rib",
    )
    backplate.visual(
        Box((0.070, 0.050, 0.0012)),
        origin=Origin(xyz=(0.102, -0.028, -0.0036)),
        material=backplate_gunmetal,
        name="rear_pad",
    )
    backplate.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_HEIGHT, 0.0045)),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, -0.0022)),
    )

    io_bracket = model.part("io_bracket")
    bracket_plate = io_bracket.visual(
        Box((0.002, 0.118, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=bracket_steel,
        name="bracket_plate",
    )
    mount_tab = io_bracket.visual(
        Box((0.008, 0.034, 0.010)),
        origin=Origin(xyz=(0.003, 0.0, -0.004)),
        material=bracket_steel,
        name="mount_tab",
    )
    io_bracket.visual(
        Box((0.005, 0.012, 0.010)),
        origin=Origin(xyz=(0.0015, 0.050, 0.018)),
        material=bracket_steel,
        name="screw_ear",
    )
    io_bracket.visual(
        Box((0.005, 0.012, 0.008)),
        origin=Origin(xyz=(0.0015, -0.050, -0.014)),
        material=bracket_steel,
        name="retention_lip",
    )
    io_bracket.visual(
        Box((0.006, 0.090, 0.006)),
        origin=Origin(xyz=(0.003, 0.0, 0.007)),
        material=bracket_steel,
        name="inner_flange",
    )
    io_bracket.inertial = Inertial.from_geometry(
        Box((0.010, 0.118, 0.048)),
        mass=0.12,
        origin=Origin(xyz=(0.0015, 0.0, 0.001)),
    )

    fan_parts = []
    for label in ("left", "center", "right"):
        fan = model.part(f"fan_{label}")
        _add_fan_blades(fan, fan_black)
        fan.visual(
            Cylinder(radius=0.013, length=0.006),
            material=fan_black,
            name="hub",
        )
        fan.visual(
            Cylinder(radius=0.016, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=fan_black,
            name="hub_cap",
        )
        fan.inertial = Inertial.from_geometry(
            Cylinder(radius=ROTOR_RADIUS, length=0.008),
            mass=0.08,
        )
        fan_parts.append(fan)

    model.articulation(
        "core_to_shroud",
        ArticulationType.FIXED,
        parent=card_core,
        child=shroud,
        origin=Origin(),
    )
    model.articulation(
        "core_to_backplate",
        ArticulationType.FIXED,
        parent=card_core,
        child=backplate,
        origin=Origin(),
    )
    model.articulation(
        "core_to_io_bracket",
        ArticulationType.FIXED,
        parent=card_core,
        child=io_bracket,
        origin=Origin(xyz=(-0.163, 0.0, 0.018)),
    )

    for fan_center_x, fan_part, joint_name in zip(
        FAN_CENTERS_X,
        fan_parts,
        ("fan_left_spin", "fan_center_spin", "fan_right_spin"),
    ):
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=card_core,
            child=fan_part,
            origin=Origin(xyz=(fan_center_x, 0.0, ROTOR_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=45.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    card_core = object_model.get_part("card_core")
    shroud = object_model.get_part("shroud")
    backplate = object_model.get_part("backplate")
    io_bracket = object_model.get_part("io_bracket")
    fan_left = object_model.get_part("fan_left")
    fan_center = object_model.get_part("fan_center")
    fan_right = object_model.get_part("fan_right")

    core_to_shroud = object_model.get_articulation("core_to_shroud")
    fan_left_spin = object_model.get_articulation("fan_left_spin")
    fan_center_spin = object_model.get_articulation("fan_center_spin")
    fan_right_spin = object_model.get_articulation("fan_right_spin")

    pcb = card_core.get_visual("pcb")
    heatsink = card_core.get_visual("heatsink")
    fan1_mount = card_core.get_visual("fan1_mount")
    fan2_mount = card_core.get_visual("fan2_mount")
    fan3_mount = card_core.get_visual("fan3_mount")
    shroud_top = shroud.get_visual("top_plate")
    left_skirt = shroud.get_visual("left_skirt")
    right_skirt = shroud.get_visual("right_skirt")
    back_plate = backplate.get_visual("plate")
    bracket_plate = io_bracket.get_visual("bracket_plate")
    mount_tab = io_bracket.get_visual("mount_tab")
    fan_left_hub = fan_left.get_visual("hub")
    fan_center_hub = fan_center.get_visual("hub")
    fan_right_hub = fan_right.get_visual("hub")
    fan_left_blades = fan_left.get_visual("blades")
    fan_center_blades = fan_center.get_visual("blades")
    fan_right_blades = fan_right.get_visual("blades")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(max_pose_samples=4)
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

    ctx.expect_gap(
        shroud,
        card_core,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_skirt,
        negative_elem=heatsink,
        name="left_shroud_skirt_seated",
    )
    ctx.expect_gap(
        shroud,
        card_core,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_skirt,
        negative_elem=heatsink,
        name="right_shroud_skirt_seated",
    )
    ctx.expect_within(
        card_core,
        backplate,
        axes="xy",
        inner_elem=pcb,
        outer_elem=back_plate,
        margin=0.006,
        name="backplate_covers_pcb",
    )
    ctx.expect_gap(
        card_core,
        backplate,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=pcb,
        negative_elem=back_plate,
        name="backplate_flush_to_pcb",
    )
    ctx.expect_gap(
        card_core,
        io_bracket,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=heatsink,
        negative_elem=mount_tab,
        name="io_bracket_mount_tab_attached",
    )
    ctx.expect_overlap(
        io_bracket,
        card_core,
        axes="yz",
        min_overlap=0.020,
        elem_a=bracket_plate,
        name="io_bracket_spans_card_stack",
    )

    fan_checks = (
        ("left", fan_left, fan_left_spin, fan1_mount, fan_left_hub, fan_left_blades),
        ("center", fan_center, fan_center_spin, fan2_mount, fan_center_hub, fan_center_blades),
        ("right", fan_right, fan_right_spin, fan3_mount, fan_right_hub, fan_right_blades),
    )
    for label, fan, joint, mount, hub, blades in fan_checks:
        ctx.check(
            f"{label}_fan_joint_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"Expected CONTINUOUS articulation, got {joint.articulation_type}.",
        )
        axis = tuple(round(value, 6) for value in (joint.axis or ()))
        ctx.check(
            f"{label}_fan_joint_axis_is_axial",
            axis == (0.0, 0.0, 1.0),
            details=f"Expected axial Z spin, got {axis}.",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{label}_fan_joint_limits_are_continuous_style",
            limits is not None
            and getattr(limits, "effort", 0.0) > 0.0
            and getattr(limits, "velocity", 0.0) > 0.0
            and getattr(limits, "lower", None) is None
            and getattr(limits, "upper", None) is None,
            details="Continuous fan joints should have positive effort/velocity and no lower/upper limits.",
        )
        ctx.expect_gap(
            fan,
            card_core,
            axis="z",
            max_gap=0.0008,
            max_penetration=0.0,
            positive_elem=hub,
            negative_elem=mount,
            name=f"{label}_fan_hub_seated_on_mount",
        )
        ctx.expect_gap(
            shroud,
            fan,
            axis="z",
            min_gap=0.0015,
            max_gap=0.008,
            positive_elem=shroud_top,
            negative_elem=blades,
            name=f"{label}_fan_blade_clearance_under_shroud",
        )
        ctx.expect_overlap(
            fan,
            shroud,
            axes="xy",
            min_overlap=0.075,
            name=f"{label}_fan_stays_under_opening_footprint",
        )

    ctx.expect_origin_distance(
        fan_left,
        fan_center,
        axes="x",
        min_dist=0.103,
        max_dist=0.105,
        name="left_center_fan_spacing",
    )
    ctx.expect_origin_distance(
        fan_center,
        fan_right,
        axes="x",
        min_dist=0.103,
        max_dist=0.105,
        name="center_right_fan_spacing",
    )
    ctx.expect_origin_distance(
        fan_left,
        fan_center,
        axes="y",
        max_dist=0.001,
        name="left_center_fans_share_heightline",
    )
    ctx.expect_origin_distance(
        fan_center,
        fan_right,
        axes="y",
        max_dist=0.001,
        name="center_right_fans_share_heightline",
    )

    for joint, fan, mount, blades, angle, label in (
        (fan_left_spin, fan_left, fan1_mount, fan_left_blades, 0.9, "left"),
        (fan_center_spin, fan_center, fan2_mount, fan_center_blades, 1.7, "center"),
        (fan_right_spin, fan_right, fan3_mount, fan_right_blades, 2.6, "right"),
    ):
        with ctx.pose({joint: angle}):
            ctx.expect_gap(
                fan,
                card_core,
                axis="z",
                max_gap=0.0008,
                max_penetration=0.0,
                positive_elem=fan.get_visual("hub"),
                negative_elem=mount,
                name=f"{label}_fan_hub_stays_seated_when_rotated",
            )
            ctx.expect_gap(
                shroud,
                fan,
                axis="z",
                min_gap=0.0015,
                max_gap=0.008,
                positive_elem=shroud_top,
                negative_elem=blades,
                name=f"{label}_fan_blade_clearance_when_rotated",
            )

    overall_bounds = [
        ctx.part_world_aabb(part)
        for part in (backplate, shroud, io_bracket)
    ]
    if any(bounds is None for bounds in overall_bounds):
        ctx.fail("overall_gpu_bounds_available", "Expected bracket, shroud, and backplate AABBs to be measurable.")
    else:
        min_x = min(bounds[0][0] for bounds in overall_bounds if bounds is not None)
        min_y = min(bounds[0][1] for bounds in overall_bounds if bounds is not None)
        min_z = min(bounds[0][2] for bounds in overall_bounds if bounds is not None)
        max_x = max(bounds[1][0] for bounds in overall_bounds if bounds is not None)
        max_y = max(bounds[1][1] for bounds in overall_bounds if bounds is not None)
        max_z = max(bounds[1][2] for bounds in overall_bounds if bounds is not None)
        ctx.check(
            "overall_gpu_proportions",
            0.320 <= (max_x - min_x) <= 0.334
            and 0.120 <= (max_y - min_y) <= 0.136
            and 0.034 <= (max_z - min_z) <= 0.050,
            details=(
                f"Expected a large triple-fan card envelope; "
                f"got {(max_x - min_x):.4f} x {(max_y - min_y):.4f} x {(max_z - min_z):.4f}."
            ),
        )

    ctx.check(
        "shroud_is_rigidly_attached_to_core",
        core_to_shroud.articulation_type == ArticulationType.FIXED,
        details=f"Expected fixed shroud mount, got {core_to_shroud.articulation_type}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
