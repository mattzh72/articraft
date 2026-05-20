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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _tube_mesh(name: str, points: list[tuple[float, float, float]], radius: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _add_bolt_ring_z(
    part,
    *,
    name_prefix: str,
    ring_radius: float,
    bolt_radius: float,
    bolt_length: float,
    z: float,
    count: int,
    material,
) -> None:
    for index in range(count):
        angle = (2.0 * math.pi * index) / count
        part.visual(
            Cylinder(radius=bolt_radius, length=bolt_length),
            origin=Origin(
                xyz=(
                    ring_radius * math.cos(angle),
                    ring_radius * math.sin(angle),
                    z,
                )
            ),
            material=material,
            name=f"{name_prefix}_{index:02d}",
        )


def _add_bolt_group_y(
    part,
    *,
    name_prefix: str,
    x_values: tuple[float, ...],
    z_values: tuple[float, ...],
    y: float,
    bolt_radius: float,
    bolt_length: float,
    material,
) -> None:
    for xi, x in enumerate(x_values):
        for zi, z in enumerate(z_values):
            part.visual(
                Cylinder(radius=bolt_radius, length=bolt_length),
                origin=Origin(
                    xyz=(x, y, z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=material,
                name=f"{name_prefix}_{xi}_{zi}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_searchlight_tower")

    heritage_cream = model.material("heritage_cream", rgba=(0.77, 0.74, 0.66, 1.0))
    machinery_green = model.material("machinery_green", rgba=(0.35, 0.40, 0.35, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.17, 0.18, 0.20, 1.0))
    service_gray = model.material("service_gray", rgba=(0.50, 0.51, 0.53, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.65, 0.67, 0.69, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.76, 0.87, 0.93, 0.45))

    mast_brace_mesh = _tube_mesh(
        "mast_brace",
        [
            (0.44, 0.0, 1.16),
            (0.40, 0.0, 1.52),
            (0.28, 0.0, 2.14),
        ],
        radius=0.05,
    )
    yoke_knee_brace_mesh = _tube_mesh(
        "yoke_knee_brace",
        [
            (0.02, 0.0, 0.56),
            (0.18, 0.0, 0.92),
            (0.28, 0.0, 1.62),
        ],
        radius=0.045,
    )

    tower_base = model.part("tower_base")
    tower_base.visual(
        Box((1.80, 1.80, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=dark_metal,
        name="foundation_plinth",
    )
    tower_base.visual(
        Box((1.20, 1.20, 0.92)),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=machinery_green,
        name="service_pedestal",
    )
    tower_base.visual(
        Box((1.30, 1.30, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        material=service_gray,
        name="pedestal_cap",
    )
    tower_base.visual(
        Cylinder(radius=0.32, length=2.65),
        origin=Origin(xyz=(0.0, 0.0, 2.525)),
        material=heritage_cream,
        name="lower_mast",
    )
    tower_base.visual(
        Cylinder(radius=0.41, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 3.85)),
        material=service_gray,
        name="mast_transition_flange",
    )
    tower_base.visual(
        Cylinder(radius=0.25, length=1.28),
        origin=Origin(xyz=(0.0, 0.0, 4.47)),
        material=heritage_cream,
        name="upper_mast",
    )
    tower_base.visual(
        Cylinder(radius=0.44, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 5.01)),
        material=dark_metal,
        name="top_bearing_skirt",
    )
    tower_base.visual(
        Cylinder(radius=0.54, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 5.15)),
        material=service_gray,
        name="top_bearing_flange",
    )
    tower_base.visual(
        Box((0.05, 0.46, 0.88)),
        origin=Origin(xyz=(0.305, 0.0, 2.25)),
        material=service_gray,
        name="front_service_hatch",
    )
    tower_base.visual(
        Box((0.05, 0.44, 0.62)),
        origin=Origin(xyz=(-0.58, 0.0, 0.74)),
        material=service_gray,
        name="rear_service_hatch",
    )
    for index in range(4):
        tower_base.visual(
            mast_brace_mesh,
            origin=Origin(rpy=(0.0, 0.0, index * (math.pi / 2.0))),
            material=service_gray,
            name=f"mast_brace_{index:02d}",
        )
    _add_bolt_ring_z(
        tower_base,
        name_prefix="mast_flange_bolt",
        ring_radius=0.33,
        bolt_radius=0.024,
        bolt_length=0.045,
        z=3.92,
        count=10,
        material=bolt_steel,
    )
    tower_base.inertial = Inertial.from_geometry(
        Box((1.80, 1.80, 5.22)),
        mass=6200.0,
        origin=Origin(xyz=(0.0, 0.0, 2.61)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.46, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=dark_metal,
        name="pan_bearing_drum",
    )
    pan_yoke.visual(
        Cylinder(radius=0.62, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=service_gray,
        name="pan_adapter_plate",
    )
    pan_yoke.visual(
        Box((1.40, 1.10, 0.18)),
        origin=Origin(xyz=(0.02, 0.0, 0.47)),
        material=machinery_green,
        name="rotating_deck",
    )
    pan_yoke.visual(
        Box((0.54, 0.68, 0.96)),
        origin=Origin(xyz=(-0.18, 0.0, 1.04)),
        material=heritage_cream,
        name="center_pedestal",
    )
    pan_yoke.visual(
        Box((0.04, 0.40, 0.58)),
        origin=Origin(xyz=(-0.43, 0.0, 1.04)),
        material=service_gray,
        name="pan_service_hatch",
    )
    pan_yoke.visual(
        Box((0.18, 0.32, 0.56)),
        origin=Origin(xyz=(-0.02, 0.0, 1.24)),
        material=dark_metal,
        name="rear_spine_block",
    )
    pan_yoke.visual(
        Cylinder(radius=0.07, length=1.22),
        origin=Origin(xyz=(-0.03, 0.0, 1.48), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="yoke_cross_tie",
    )
    for side, y in (("left", 0.69), ("right", -0.69)):
        pan_yoke.visual(
            Box((0.22, 0.54, 0.92)),
            origin=Origin(xyz=(0.09, 0.46 if side == "left" else -0.46, 1.06)),
            material=service_gray,
            name=f"{side}_side_web",
        )
        pan_yoke.visual(
            Box((0.20, 0.20, 1.36)),
            origin=Origin(xyz=(0.18, y, 1.24)),
            material=heritage_cream,
            name=f"{side}_yoke_arm",
        )
        pan_yoke.visual(
            Box((0.50, 0.20, 0.24)),
            origin=Origin(xyz=(0.42, y, 1.74)),
            material=heritage_cream,
            name=f"{side}_arm_cap",
        )
        pan_yoke.visual(
            Box((0.36, 0.28, 0.34)),
            origin=Origin(xyz=(0.63, y, 1.84)),
            material=service_gray,
            name=f"{side}_bearing_block",
        )
        pan_yoke.visual(
            Cylinder(radius=0.18, length=0.08),
            origin=Origin(
                xyz=(0.63, 0.87 if side == "left" else -0.87, 1.84),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"{side}_outer_bearing_cap",
        )
        pan_yoke.visual(
            yoke_knee_brace_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=dark_metal,
            name=f"{side}_knee_brace",
        )
    _add_bolt_group_y(
        pan_yoke,
        name_prefix="left_bearing_bolt",
        x_values=(0.55, 0.71),
        z_values=(1.74, 1.94),
        y=0.842,
        bolt_radius=0.016,
        bolt_length=0.03,
        material=bolt_steel,
    )
    _add_bolt_group_y(
        pan_yoke,
        name_prefix="right_bearing_bolt",
        x_values=(0.55, 0.71),
        z_values=(1.74, 1.94),
        y=-0.842,
        bolt_radius=0.016,
        bolt_length=0.03,
        material=bolt_steel,
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((1.40, 1.82, 2.04)),
        mass=1800.0,
        origin=Origin(xyz=(0.02, 0.0, 1.02)),
    )

    spotlight_head = model.part("spotlight_head")
    spotlight_head.visual(
        Cylinder(radius=0.08, length=1.10),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_shaft",
    )
    spotlight_head.visual(
        Cylinder(radius=0.14, length=0.16),
        origin=Origin(xyz=(0.02, 0.43, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=service_gray,
        name="left_trunnion_collar",
    )
    spotlight_head.visual(
        Cylinder(radius=0.14, length=0.16),
        origin=Origin(xyz=(0.02, -0.43, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=service_gray,
        name="right_trunnion_collar",
    )
    spotlight_head.visual(
        Box((0.56, 0.70, 0.50)),
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
        material=machinery_green,
        name="trunnion_cradle",
    )
    spotlight_head.visual(
        Box((0.16, 0.12, 0.34)),
        origin=Origin(xyz=(0.02, 0.42, 0.0)),
        material=service_gray,
        name="left_trunnion_adapter",
    )
    spotlight_head.visual(
        Box((0.16, 0.12, 0.34)),
        origin=Origin(xyz=(0.02, -0.42, 0.0)),
        material=service_gray,
        name="right_trunnion_adapter",
    )
    spotlight_head.visual(
        Cylinder(radius=0.28, length=0.24),
        origin=Origin(xyz=(-0.06, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_green,
        name="rear_neck",
    )
    spotlight_head.visual(
        Cylinder(radius=0.36, length=0.26),
        origin=Origin(xyz=(0.13, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=heritage_cream,
        name="intermediate_barrel",
    )
    spotlight_head.visual(
        Cylinder(radius=0.46, length=0.92),
        origin=Origin(xyz=(0.67, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=heritage_cream,
        name="main_barrel",
    )
    spotlight_head.visual(
        Cylinder(radius=0.54, length=0.15),
        origin=Origin(xyz=(1.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=service_gray,
        name="front_bezel",
    )
    spotlight_head.visual(
        Cylinder(radius=0.46, length=0.035),
        origin=Origin(xyz=(1.2975, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_glass",
    )
    spotlight_head.visual(
        Box((0.34, 0.48, 0.30)),
        origin=Origin(xyz=(-0.19, 0.0, -0.27)),
        material=machinery_green,
        name="rear_power_box",
    )
    spotlight_head.visual(
        Box((0.035, 0.34, 0.22)),
        origin=Origin(xyz=(-0.34, 0.0, -0.27)),
        material=service_gray,
        name="rear_access_hatch",
    )
    for index, x in enumerate((-0.12, -0.08, -0.04)):
        spotlight_head.visual(
            Cylinder(radius=0.30, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"cooling_band_{index:02d}",
        )
    _add_bolt_group_y(
        spotlight_head,
        name_prefix="left_adapter_bolt",
        x_values=(-0.02, 0.06),
        z_values=(-0.10, 0.10),
        y=0.466,
        bolt_radius=0.014,
        bolt_length=0.028,
        material=bolt_steel,
    )
    _add_bolt_group_y(
        spotlight_head,
        name_prefix="right_adapter_bolt",
        x_values=(-0.02, 0.06),
        z_values=(-0.10, 0.10),
        y=-0.466,
        bolt_radius=0.014,
        bolt_length=0.028,
        material=bolt_steel,
    )
    spotlight_head.inertial = Inertial.from_geometry(
        Box((1.68, 1.10, 1.10)),
        mass=760.0,
        origin=Origin(xyz=(0.47, 0.0, -0.03)),
    )

    model.articulation(
        "searchlight_pan",
        ArticulationType.CONTINUOUS,
        parent=tower_base,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 5.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.7),
    )
    model.articulation(
        "searchlight_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=spotlight_head,
        origin=Origin(xyz=(0.63, 0.0, 1.84)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=0.9,
            lower=-0.20,
            upper=1.05,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_pt, max_pt = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_pt, max_pt))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_base = object_model.get_part("tower_base")
    pan_yoke = object_model.get_part("pan_yoke")
    spotlight_head = object_model.get_part("spotlight_head")
    pan_joint = object_model.get_articulation("searchlight_pan")
    tilt_joint = object_model.get_articulation("searchlight_tilt")

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
        pan_yoke,
        tower_base,
        elem_a="pan_bearing_drum",
        elem_b="top_bearing_flange",
        name="pan_bearing_seated_on_tower",
    )
    ctx.expect_overlap(
        pan_yoke,
        tower_base,
        axes="xy",
        min_overlap=0.90,
        elem_a="pan_bearing_drum",
        elem_b="top_bearing_flange",
        name="pan_bearing_centered_on_tower",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_yoke,
        elem_a="trunnion_shaft",
        elem_b="left_bearing_block",
        name="left_trunnion_supported",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_yoke,
        elem_a="trunnion_shaft",
        elem_b="right_bearing_block",
        name="right_trunnion_supported",
    )
    ctx.expect_gap(
        pan_yoke,
        spotlight_head,
        axis="y",
        positive_elem="left_bearing_block",
        negative_elem="main_barrel",
        min_gap=0.07,
        name="barrel_clears_left_bearing_block",
    )
    ctx.expect_gap(
        spotlight_head,
        pan_yoke,
        axis="y",
        positive_elem="main_barrel",
        negative_elem="right_bearing_block",
        min_gap=0.07,
        name="barrel_clears_right_bearing_block",
    )
    ctx.expect_gap(
        spotlight_head,
        pan_yoke,
        axis="z",
        positive_elem="trunnion_shaft",
        negative_elem="rotating_deck",
        min_gap=1.15,
        name="tilt_axis_carried_above_deck",
    )

    closed_lens = _aabb_center(ctx.part_element_world_aabb(spotlight_head, elem="lens_glass"))
    with ctx.pose({pan_joint: 0.9}):
        panned_lens = _aabb_center(ctx.part_element_world_aabb(spotlight_head, elem="lens_glass"))
    pan_ok = (
        closed_lens is not None
        and panned_lens is not None
        and panned_lens[1] > 1.0
        and panned_lens[0] < closed_lens[0] - 0.45
    )
    ctx.check(
        "pan_swings_head_around_mast",
        pan_ok,
        details=f"closed_lens={closed_lens}, panned_lens={panned_lens}",
    )

    with ctx.pose({tilt_joint: 0.75}):
        raised_lens = _aabb_center(ctx.part_element_world_aabb(spotlight_head, elem="lens_glass"))
        ctx.expect_gap(
            spotlight_head,
            pan_yoke,
            axis="z",
            positive_elem="rear_power_box",
            negative_elem="rotating_deck",
            min_gap=0.20,
            name="rear_power_box_clears_deck_when_raised",
        )
    tilt_ok = (
        closed_lens is not None
        and raised_lens is not None
        and raised_lens[2] > closed_lens[2] + 0.55
        and raised_lens[0] < closed_lens[0] - 0.30
    )
    ctx.check(
        "tilt_raises_light_output",
        tilt_ok,
        details=f"closed_lens={closed_lens}, raised_lens={raised_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
