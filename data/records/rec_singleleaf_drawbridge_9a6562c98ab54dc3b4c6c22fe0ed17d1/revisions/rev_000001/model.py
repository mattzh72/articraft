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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_y_cylinder(part, *, radius: float, length: float, xyz, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _add_bolted_trunnion_adapter(part, *, side_sign: float, plate_material, bolt_material) -> None:
    y_center = side_sign * 4.105
    part.visual(
        Box((0.90, 0.25, 1.12)),
        origin=Origin(xyz=(0.30, y_center, -0.02)),
        material=plate_material,
        name="left_trunnion_adapter" if side_sign > 0.0 else "right_trunnion_adapter",
    )
    for x_pos in (0.02, 0.46):
        for z_pos in (-0.34, 0.30):
            _add_y_cylinder(
                part,
                radius=0.050,
                length=0.12,
                xyz=(x_pos, side_sign * 4.25, z_pos),
                material=bolt_material,
            )


def _add_hatch_handle(part, *, xyz, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=0.026, length=0.20),
        origin=Origin(xyz=xyz, rpy=(0.0, 0.0, math.pi / 2.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_singleleaf_drawbridge")

    concrete = model.material("concrete", rgba=(0.68, 0.68, 0.66, 1.0))
    bridge_green = model.material("bridge_green", rgba=(0.30, 0.39, 0.34, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.36, 0.34, 0.32, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    asphalt = model.material("asphalt", rgba=(0.10, 0.10, 0.11, 1.0))
    hatch_gray = model.material("hatch_gray", rgba=(0.55, 0.57, 0.58, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.74, 0.18, 1.0))

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((8.0, 12.0, 8.0)),
        mass=28000.0,
        origin=Origin(xyz=(0.0, 0.0, 4.0)),
    )

    support_frame.visual(
        Box((8.0, 2.6, 1.4)),
        origin=Origin(xyz=(0.0, 4.70, 0.70)),
        material=concrete,
        name="left_foundation",
    )
    support_frame.visual(
        Box((8.0, 2.6, 1.4)),
        origin=Origin(xyz=(0.0, -4.70, 0.70)),
        material=concrete,
        name="right_foundation",
    )
    support_frame.visual(
        Box((2.2, 7.0, 1.4)),
        origin=Origin(xyz=(-2.90, 0.0, 0.70)),
        material=concrete,
        name="rear_cross_foundation",
    )
    support_frame.visual(
        Box((2.0, 8.8, 1.0)),
        origin=Origin(xyz=(1.40, 0.0, 0.50)),
        material=concrete,
        name="front_cross_sill",
    )

    support_frame.visual(
        Box((5.6, 1.35, 2.8)),
        origin=Origin(xyz=(-1.80, 5.17, 2.80)),
        material=weathered_steel,
        name="left_service_block",
    )
    support_frame.visual(
        Box((5.6, 1.35, 2.8)),
        origin=Origin(xyz=(-1.80, -5.17, 2.80)),
        material=weathered_steel,
        name="right_service_block",
    )
    support_frame.visual(
        Box((2.6, 1.45, 0.36)),
        origin=Origin(xyz=(-1.20, 5.17, 4.38)),
        material=bridge_green,
        name="left_maintenance_deck",
    )
    support_frame.visual(
        Box((2.6, 1.45, 0.36)),
        origin=Origin(xyz=(-1.20, -5.17, 4.38)),
        material=bridge_green,
        name="right_maintenance_deck",
    )
    support_frame.visual(
        Box((1.00, 6.80, 0.23)),
        origin=Origin(xyz=(1.15, 0.0, 4.675)),
        material=weathered_steel,
        name="roadway_seat",
    )
    for y_pos, name in ((-2.10, "seat_stanchion_right"), (0.0, "seat_stanchion_center"), (2.10, "seat_stanchion_left")):
        support_frame.visual(
            Box((0.50, 0.70, 3.56)),
            origin=Origin(xyz=(0.90, y_pos, 2.78)),
            material=weathered_steel,
            name=name,
        )

    support_frame.visual(
        Box((1.6, 1.2, 5.9)),
        origin=Origin(xyz=(0.15, 5.00, 4.35)),
        material=bridge_green,
        name="left_tower",
    )
    support_frame.visual(
        Box((1.6, 1.2, 5.9)),
        origin=Origin(xyz=(0.15, -5.00, 4.35)),
        material=bridge_green,
        name="right_tower",
    )
    support_frame.visual(
        Box((1.6, 10.0, 0.55)),
        origin=Origin(xyz=(0.05, 0.0, 7.575)),
        material=bridge_green,
        name="portal_beam",
    )
    support_frame.visual(
        Box((0.92, 0.42, 1.72)),
        origin=Origin(xyz=(0.05, 4.61, 4.35)),
        material=weathered_steel,
        name="left_bearing_block",
    )
    support_frame.visual(
        Box((0.92, 0.42, 1.72)),
        origin=Origin(xyz=(0.05, -4.61, 4.35)),
        material=weathered_steel,
        name="right_bearing_block",
    )
    support_frame.visual(
        Box((0.78, 0.18, 1.34)),
        origin=Origin(xyz=(0.18, 4.46, 4.35)),
        material=dark_steel,
        name="left_bearing_face",
    )
    support_frame.visual(
        Box((0.78, 0.18, 1.34)),
        origin=Origin(xyz=(0.18, -4.46, 4.35)),
        material=dark_steel,
        name="right_bearing_face",
    )
    for side_sign in (-1.0, 1.0):
        support_frame.visual(
            Box((1.45, 0.22, 2.05)),
            origin=Origin(xyz=(-0.55, side_sign * 4.62, 5.70), rpy=(0.0, 0.72, 0.0)),
            material=weathered_steel,
        )
        support_frame.visual(
            Box((1.15, 0.22, 1.65)),
            origin=Origin(xyz=(-0.88, side_sign * 4.62, 3.10), rpy=(0.0, -0.62, 0.0)),
            material=weathered_steel,
        )

    support_frame.visual(
        Box((0.08, 0.88, 0.84)),
        origin=Origin(xyz=(-4.56, 5.17, 2.80)),
        material=hatch_gray,
        name="left_service_hatch",
    )
    support_frame.visual(
        Box((0.08, 0.88, 0.84)),
        origin=Origin(xyz=(-4.56, -5.17, 2.80)),
        material=hatch_gray,
        name="right_service_hatch",
    )
    _add_hatch_handle(support_frame, xyz=(-4.50, 5.17, 2.80), material=safety_yellow)
    _add_hatch_handle(support_frame, xyz=(-4.50, -5.17, 2.80), material=safety_yellow)
    support_frame.visual(
        Box((0.85, 0.70, 0.06)),
        origin=Origin(xyz=(-1.20, 5.17, 4.59)),
        material=hatch_gray,
        name="left_top_hatch",
    )
    support_frame.visual(
        Box((0.85, 0.70, 0.06)),
        origin=Origin(xyz=(-1.20, -5.17, 4.59)),
        material=hatch_gray,
        name="right_top_hatch",
    )
    _add_y_cylinder(
        support_frame,
        radius=0.025,
        length=0.22,
        xyz=(-1.20, 5.17, 4.63),
        material=safety_yellow,
    )
    _add_y_cylinder(
        support_frame,
        radius=0.025,
        length=0.22,
        xyz=(-1.20, -5.17, 4.63),
        material=safety_yellow,
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((22.0, 9.2, 3.0)),
        mass=16500.0,
        origin=Origin(xyz=(7.0, 0.0, -0.05)),
    )

    _add_y_cylinder(
        bridge_leaf,
        radius=0.30,
        length=8.38,
        xyz=(0.0, 0.0, 0.0),
        material=dark_steel,
        name="trunnion_shaft",
    )
    _add_y_cylinder(
        bridge_leaf,
        radius=0.64,
        length=0.18,
        xyz=(0.0, 4.28, 0.0),
        material=weathered_steel,
        name="left_trunnion_disc",
    )
    _add_y_cylinder(
        bridge_leaf,
        radius=0.64,
        length=0.18,
        xyz=(0.0, -4.28, 0.0),
        material=weathered_steel,
        name="right_trunnion_disc",
    )

    _add_bolted_trunnion_adapter(
        bridge_leaf,
        side_sign=1.0,
        plate_material=weathered_steel,
        bolt_material=dark_steel,
    )
    _add_bolted_trunnion_adapter(
        bridge_leaf,
        side_sign=-1.0,
        plate_material=weathered_steel,
        bolt_material=dark_steel,
    )

    support_profile_braces = [
        ("leaf_tail_brace_left", [(-3.15, 2.50, -1.08), (-1.70, 3.00, -0.76), (-0.10, 3.78, -0.12)]),
        ("leaf_tail_brace_right", _mirror_y([(-3.15, 2.50, -1.08), (-1.70, 3.00, -0.76), (-0.10, 3.78, -0.12)])),
        ("underbrace_left_inner", [(0.55, 3.90, -0.62), (2.65, 2.10, -0.40), (4.80, 0.0, 0.15)]),
        ("underbrace_right_inner", _mirror_y([(0.55, 3.90, -0.62), (2.65, 2.10, -0.40), (4.80, 0.0, 0.15)])),
        ("underbrace_left_outer", [(4.80, 0.0, 0.15), (7.60, 2.10, -0.18), (10.60, 3.90, -0.34)]),
        ("underbrace_right_outer", _mirror_y([(4.80, 0.0, 0.15), (7.60, 2.10, -0.18), (10.60, 3.90, -0.34)])),
    ]
    for name, points in support_profile_braces:
        bridge_leaf.visual(
            _mesh(name, tube_from_spline_points(points, radius=0.085, samples_per_segment=14, radial_segments=18)),
            material=weathered_steel,
            name=name,
        )

    bridge_leaf.visual(
        Box((3.6, 5.8, 2.35)),
        origin=Origin(xyz=(-1.95, 0.0, -0.83)),
        material=weathered_steel,
        name="counterweight_box",
    )
    bridge_leaf.visual(
        Box((3.2, 5.4, 0.20)),
        origin=Origin(xyz=(-1.55, 0.0, 0.445)),
        material=bridge_green,
        name="tail_service_deck",
    )
    bridge_leaf.visual(
        Box((0.95, 0.70, 0.07)),
        origin=Origin(xyz=(-1.55, 0.0, 0.58)),
        material=hatch_gray,
        name="tail_hatch",
    )
    _add_y_cylinder(
        bridge_leaf,
        radius=0.022,
        length=0.20,
        xyz=(-1.55, 0.0, 0.62),
        material=safety_yellow,
    )

    for side_sign in (-1.0, 1.0):
        bridge_leaf.visual(
            Box((5.2, 0.56, 1.12)),
            origin=Origin(xyz=(2.60, side_sign * 3.95, -0.12)),
            material=bridge_green,
            name="left_root_girder" if side_sign > 0.0 else "right_root_girder",
        )
        bridge_leaf.visual(
            Box((6.6, 0.48, 1.20)),
            origin=Origin(xyz=(8.00, side_sign * 3.95, -0.06)),
            material=bridge_green,
            name="left_mid_girder" if side_sign > 0.0 else "right_mid_girder",
        )
        bridge_leaf.visual(
            Box((7.2, 0.42, 0.86)),
            origin=Origin(xyz=(14.60, side_sign * 3.95, 0.03)),
            material=bridge_green,
            name="left_tip_girder" if side_sign > 0.0 else "right_tip_girder",
        )
        bridge_leaf.visual(
            Box((1.35, 0.16, 1.60)),
            origin=Origin(xyz=(-0.10, side_sign * 3.80, -0.58), rpy=(0.0, -0.82, 0.0)),
            material=weathered_steel,
        )
        bridge_leaf.visual(
            Box((1.10, 0.16, 1.22)),
            origin=Origin(xyz=(0.60, side_sign * 3.80, 0.18), rpy=(0.0, 0.74, 0.0)),
            material=weathered_steel,
        )
        bridge_leaf.visual(
            Box((16.8, 0.22, 0.18)),
            origin=Origin(xyz=(10.20, side_sign * 3.72, 0.77)),
            material=weathered_steel,
        )

    bridge_leaf.visual(
        Box((18.6, 8.10, 0.24)),
        origin=Origin(xyz=(9.30, 0.0, 0.56)),
        material=bridge_green,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((16.4, 7.20, 0.09)),
        origin=Origin(xyz=(10.20, 0.0, 0.725)),
        material=asphalt,
        name="wear_surface",
    )
    for index, x_pos in enumerate((1.95, 4.80, 8.20, 11.60, 14.80, 17.60)):
        bridge_leaf.visual(
            Box((0.35, 7.70, 0.56)),
            origin=Origin(xyz=(x_pos, 0.0, 0.16)),
            material=weathered_steel,
            name=f"crossbeam_{index}",
        )

    bridge_leaf.visual(
        Box((0.60, 8.00, 0.40)),
        origin=Origin(xyz=(18.85, 0.0, 0.40)),
        material=weathered_steel,
        name="tip_nose",
    )

    model.articulation(
        "support_to_leaf",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 4.35)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220000.0,
            velocity=0.22,
            lower=0.0,
            upper=1.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("support_to_leaf")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    limits = hinge.motion_limits
    axis_ok = hinge.axis == (0.0, -1.0, 0.0)
    limits_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - 0.0) < 1e-6
        and abs(limits.upper - 1.22) < 1e-6
    )
    ctx.check(
        "singleleaf_hinge_configured",
        axis_ok and limits_ok,
        details=f"axis={hinge.axis}, limits={None if limits is None else (limits.lower, limits.upper)}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            bridge_leaf,
            support_frame,
            elem_a="left_trunnion_disc",
            elem_b="left_bearing_face",
            name="left_trunnion_supported",
        )
        ctx.expect_contact(
            bridge_leaf,
            support_frame,
            elem_a="right_trunnion_disc",
            elem_b="right_bearing_face",
            name="right_trunnion_supported",
        )
        ctx.expect_gap(
            bridge_leaf,
            support_frame,
            axis="z",
            positive_elem="deck_plate",
            negative_elem="roadway_seat",
            max_gap=0.002,
            max_penetration=0.0,
            name="leaf_seats_on_support_frame",
        )
        ctx.expect_overlap(
            bridge_leaf,
            support_frame,
            axes="xy",
            elem_a="deck_plate",
            elem_b="roadway_seat",
            min_overlap=1.0,
            name="leaf_and_roadway_seat_align",
        )

    with ctx.pose({hinge: 1.05}):
        ctx.expect_gap(
            bridge_leaf,
            support_frame,
            axis="z",
            positive_elem="tip_nose",
            negative_elem="portal_beam",
            min_gap=4.0,
            name="leaf_raises_clear_of_portal",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
