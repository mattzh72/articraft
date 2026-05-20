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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _add_bolt(
    part,
    *,
    name: str,
    center: tuple[float, float, float],
    shank_radius: float,
    shank_length: float,
    head_radius: float,
    head_length: float,
    material,
) -> None:
    x, y, z = center
    part.visual(
        Cylinder(radius=shank_radius, length=shank_length),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=f"{name}_shank",
    )
    part.visual(
        Cylinder(radius=head_radius, length=head_length),
        origin=Origin(xyz=(x, y, z + shank_length * 0.5 + head_length * 0.5)),
        material=material,
        name=f"{name}_head",
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((low + high) * 0.5 for low, high in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_sun_visor")

    aged_steel = model.material("aged_steel", rgba=(0.53, 0.54, 0.56, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.20, 0.21, 0.23, 1.0))
    fastener_black = model.material("fastener_black", rgba=(0.11, 0.11, 0.12, 1.0))
    visor_vinyl = model.material("visor_vinyl", rgba=(0.41, 0.38, 0.31, 1.0))
    hatch_paint = model.material("hatch_paint", rgba=(0.46, 0.44, 0.39, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    roof_mount = model.part("roof_mount")
    roof_mount.visual(
        Box((0.148, 0.078, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=aged_steel,
        name="roof_plate",
    )
    roof_mount.visual(
        Box((0.118, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, -0.012, -0.006)),
        material=dark_oxide,
        name="retrofit_adapter",
    )
    roof_mount.visual(
        Box((0.090, 0.024, 0.016)),
        origin=Origin(xyz=(0.0, -0.018, -0.010)),
        material=dark_oxide,
        name="adapter_block",
    )
    roof_mount.visual(
        Box((0.050, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.002, -0.017)),
        material=dark_oxide,
        name="hinge_saddle",
    )
    roof_mount.visual(
        Box((0.006, 0.024, 0.022)),
        origin=Origin(xyz=(-0.019, 0.009, -0.018)),
        material=dark_oxide,
        name="left_pivot_brace",
    )
    roof_mount.visual(
        Box((0.006, 0.024, 0.022)),
        origin=Origin(xyz=(0.019, 0.009, -0.018)),
        material=dark_oxide,
        name="right_pivot_brace",
    )
    roof_mount.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(-0.015, 0.020, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_steel,
        name="left_primary_ear",
    )
    roof_mount.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.015, 0.020, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_steel,
        name="right_primary_ear",
    )
    roof_mount.visual(
        Box((0.010, 0.046, 0.026)),
        origin=Origin(xyz=(-0.028, -0.004, -0.008)),
        material=dark_oxide,
        name="left_side_reinforcement",
    )
    roof_mount.visual(
        Box((0.010, 0.046, 0.026)),
        origin=Origin(xyz=(0.028, -0.004, -0.008)),
        material=dark_oxide,
        name="right_side_reinforcement",
    )

    for index, (bolt_x, bolt_y) in enumerate(
        ((-0.048, -0.022), (0.048, -0.022), (-0.048, 0.022), (0.048, 0.022))
    ):
        _add_bolt(
            roof_mount,
            name=f"roof_bolt_{index}",
            center=(bolt_x, bolt_y, 0.001),
            shank_radius=0.003,
            shank_length=0.018,
            head_radius=0.006,
            head_length=0.003,
            material=fastener_black,
        )

    roof_mount.inertial = Inertial.from_geometry(
        Box((0.148, 0.078, 0.050)),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_steel,
        name="primary_barrel",
    )
    hinge_arm.visual(
        Box((0.018, 0.026, 0.022)),
        origin=Origin(xyz=(0.0, 0.013, -0.006)),
        material=dark_oxide,
        name="arm_anchor_block",
    )
    hinge_arm.visual(
        Box((0.030, 0.088, 0.018)),
        origin=Origin(xyz=(0.0, 0.070, -0.009)),
        material=dark_oxide,
        name="main_arm",
    )
    hinge_arm.visual(
        Box((0.008, 0.084, 0.024)),
        origin=Origin(xyz=(-0.012, 0.071, -0.008)),
        material=aged_steel,
        name="left_arm_flange",
    )
    hinge_arm.visual(
        Box((0.008, 0.084, 0.024)),
        origin=Origin(xyz=(0.012, 0.071, -0.008)),
        material=aged_steel,
        name="right_arm_flange",
    )
    hinge_arm.visual(
        Box((0.030, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.093, -0.008)),
        material=dark_oxide,
        name="fork_base",
    )
    hinge_arm.visual(
        Box((0.006, 0.040, 0.020)),
        origin=Origin(xyz=(-0.018, 0.123, -0.008)),
        material=aged_steel,
        name="left_fork_rail",
    )
    hinge_arm.visual(
        Box((0.006, 0.040, 0.020)),
        origin=Origin(xyz=(0.018, 0.123, -0.008)),
        material=aged_steel,
        name="right_fork_rail",
    )
    hinge_arm.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.108, -0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_steel,
        name="front_clevis_ear",
    )
    hinge_arm.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.138, -0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_steel,
        name="rear_clevis_ear",
    )
    hinge_arm.inertial = Inertial.from_geometry(
        Box((0.034, 0.150, 0.036)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.074, -0.008)),
    )

    visor_panel = model.part("visor_panel")
    visor_core_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.340, 0.180, 0.020), 0.018, center=True),
        "retro_visor_core",
    )
    visor_panel.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_steel,
        name="pivot_knuckle",
    )
    visor_panel.visual(
        Box((0.020, 0.016, 0.022)),
        origin=Origin(xyz=(0.030, 0.000, -0.015)),
        material=dark_oxide,
        name="pivot_cheek",
    )
    visor_panel.visual(
        Box((0.020, 0.014, 0.010)),
        origin=Origin(xyz=(0.020, -0.008, -0.014)),
        material=dark_oxide,
        name="knuckle_support_web",
    )
    visor_panel.visual(
        Box((0.010, 0.014, 0.018)),
        origin=Origin(xyz=(0.006, 0.003, -0.011)),
        material=dark_oxide,
        name="knuckle_bridge",
    )
    visor_panel.visual(
        visor_core_mesh,
        origin=Origin(xyz=(0.175, 0.092, -0.020)),
        material=visor_vinyl,
        name="visor_core",
    )
    visor_panel.visual(
        Box((0.290, 0.020, 0.006)),
        origin=Origin(xyz=(0.168, 0.014, -0.008)),
        material=aged_steel,
        name="top_spine_reinforcement",
    )
    visor_panel.visual(
        Box((0.290, 0.020, 0.008)),
        origin=Origin(xyz=(0.168, 0.126, -0.029)),
        material=seal_rubber,
        name="bottom_binding_strip",
    )
    visor_panel.visual(
        Box((0.110, 0.028, 0.006)),
        origin=Origin(xyz=(0.074, 0.012, -0.012)),
        material=aged_steel,
        name="pivot_adapter_plate",
    )

    for index, (bolt_x, bolt_y) in enumerate(
        ((0.024, 0.008), (0.024, 0.028), (0.076, 0.008), (0.076, 0.028))
    ):
        _add_bolt(
            visor_panel,
            name=f"adapter_bolt_{index}",
            center=(bolt_x, bolt_y, -0.013),
            shank_radius=0.0025,
            shank_length=0.010,
            head_radius=0.005,
            head_length=0.0025,
            material=fastener_black,
        )

    visor_panel.visual(
        Box((0.095, 0.060, 0.003)),
        origin=Origin(xyz=(0.110, 0.088, -0.0305)),
        material=hatch_paint,
        name="service_hatch_left",
    )
    visor_panel.visual(
        Box((0.095, 0.060, 0.003)),
        origin=Origin(xyz=(0.245, 0.088, -0.0305)),
        material=hatch_paint,
        name="service_hatch_right",
    )

    hatch_fasteners = (
        ("left", 0.110, ((-0.034, -0.020), (-0.034, 0.020), (0.034, -0.020), (0.034, 0.020))),
        ("right", 0.245, ((-0.034, -0.020), (-0.034, 0.020), (0.034, -0.020), (0.034, 0.020))),
    )
    for side_name, hatch_x, offsets in hatch_fasteners:
        for index, (dx, dy) in enumerate(offsets):
            visor_panel.visual(
                Cylinder(radius=0.004, length=0.002),
                origin=Origin(xyz=(hatch_x + dx, 0.088 + dy, -0.033)),
                material=fastener_black,
                name=f"{side_name}_hatch_fastener_{index}",
            )

    visor_panel.inertial = Inertial.from_geometry(
        Box((0.350, 0.182, 0.050)),
        mass=0.88,
        origin=Origin(xyz=(0.175, 0.092, -0.020)),
    )

    model.articulation(
        "primary_hinge",
        ArticulationType.REVOLUTE,
        parent=roof_mount,
        child=hinge_arm,
        origin=Origin(xyz=(0.0, 0.020, -0.018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "secondary_swing",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.0, 0.123, -0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-1.1,
            upper=1.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_mount = object_model.get_part("roof_mount")
    hinge_arm = object_model.get_part("hinge_arm")
    visor_panel = object_model.get_part("visor_panel")
    primary_hinge = object_model.get_articulation("primary_hinge")
    secondary_swing = object_model.get_articulation("secondary_swing")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
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

    ctx.expect_contact(
        roof_mount,
        hinge_arm,
        elem_a="left_primary_ear",
        elem_b="primary_barrel",
        name="primary hinge barrel seats in roof ear",
    )
    ctx.expect_contact(
        hinge_arm,
        visor_panel,
        elem_a="front_clevis_ear",
        elem_b="pivot_knuckle",
        name="secondary pivot knuckle seats in clevis",
    )
    ctx.expect_gap(
        roof_mount,
        visor_panel,
        axis="z",
        positive_elem="roof_plate",
        negative_elem="visor_core",
        min_gap=0.025,
        max_gap=0.055,
        name="visor stows below roof plate with realistic clearance",
    )

    visor_core_aabb = ctx.part_element_world_aabb(visor_panel, elem="visor_core")
    left_hatch_aabb = ctx.part_element_world_aabb(visor_panel, elem="service_hatch_left")
    right_hatch_aabb = ctx.part_element_world_aabb(visor_panel, elem="service_hatch_right")
    if visor_core_aabb is not None and left_hatch_aabb is not None:
        left_ok = (
            left_hatch_aabb[0][0] >= visor_core_aabb[0][0] - 1e-6
            and left_hatch_aabb[1][0] <= visor_core_aabb[1][0] + 1e-6
            and left_hatch_aabb[0][1] >= visor_core_aabb[0][1] - 1e-6
            and left_hatch_aabb[1][1] <= visor_core_aabb[1][1] + 1e-6
        )
        ctx.check(
            "left service hatch sits within visor footprint",
            left_ok,
            details=f"left hatch aabb={left_hatch_aabb}, visor core aabb={visor_core_aabb}",
        )
    else:
        ctx.fail("left service hatch sits within visor footprint", "missing visor core or left hatch AABB")

    if visor_core_aabb is not None and right_hatch_aabb is not None:
        right_ok = (
            right_hatch_aabb[0][0] >= visor_core_aabb[0][0] - 1e-6
            and right_hatch_aabb[1][0] <= visor_core_aabb[1][0] + 1e-6
            and right_hatch_aabb[0][1] >= visor_core_aabb[0][1] - 1e-6
            and right_hatch_aabb[1][1] <= visor_core_aabb[1][1] + 1e-6
        )
        ctx.check(
            "right service hatch sits within visor footprint",
            right_ok,
            details=f"right hatch aabb={right_hatch_aabb}, visor core aabb={visor_core_aabb}",
        )
    else:
        ctx.fail("right service hatch sits within visor footprint", "missing visor core or right hatch AABB")

    with ctx.pose({primary_hinge: 0.0, secondary_swing: 0.0}):
        closed_core = ctx.part_element_world_aabb(visor_panel, elem="visor_core")
    with ctx.pose({primary_hinge: 1.15, secondary_swing: 0.0}):
        opened_core = ctx.part_element_world_aabb(visor_panel, elem="visor_core")
    if closed_core is not None and opened_core is not None:
        closed_center = _aabb_center(closed_core)
        opened_center = _aabb_center(opened_core)
        ctx.check(
            "primary hinge drops visor into use position",
            opened_center[2] < closed_center[2] - 0.10,
            details=f"closed_center={closed_center}, opened_center={opened_center}",
        )
    else:
        ctx.fail("primary hinge drops visor into use position", "missing visor core AABB in closed or opened pose")

    with ctx.pose({primary_hinge: 1.15, secondary_swing: 0.0}):
        centered_core = ctx.part_element_world_aabb(visor_panel, elem="visor_core")
    with ctx.pose({primary_hinge: 1.15, secondary_swing: 0.90}):
        swung_core = ctx.part_element_world_aabb(visor_panel, elem="visor_core")
    if centered_core is not None and swung_core is not None:
        centered = _aabb_center(centered_core)
        swung = _aabb_center(swung_core)
        ctx.check(
            "secondary swing visibly reorients visor panel",
            abs(swung[0] - centered[0]) > 0.05,
            details=f"centered={centered}, swung={swung}",
        )
    else:
        ctx.fail("secondary swing visibly reorients visor panel", "missing visor core AABB in swing comparison poses")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
