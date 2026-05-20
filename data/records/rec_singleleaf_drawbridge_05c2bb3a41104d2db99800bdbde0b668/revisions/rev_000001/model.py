from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi

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
    model = ArticulatedObject(name="industrial_singleleaf_drawbridge")

    structural_steel = model.material("structural_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.24, 0.26, 0.28, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.92, 0.74, 0.10, 1.0))
    concrete = model.material("concrete", rgba=(0.50, 0.51, 0.52, 1.0))
    hardware = model.material("hardware", rgba=(0.10, 0.10, 0.11, 1.0))

    def add_box(
        part,
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        *,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
        material=structural_steel,
    ) -> None:
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def add_cylinder(
        part,
        name: str,
        radius: float,
        length: float,
        xyz: tuple[float, float, float],
        *,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
        material=hardware,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def add_y_bolt(
        part,
        name: str,
        xyz: tuple[float, float, float],
        *,
        radius: float = 0.026,
        length: float = 0.08,
    ) -> None:
        add_cylinder(part, name, radius, length, xyz, rpy=(-pi / 2.0, 0.0, 0.0), material=hardware)

    def add_z_bolt(
        part,
        name: str,
        xyz: tuple[float, float, float],
        *,
        radius: float = 0.024,
        length: float = 0.06,
    ) -> None:
        add_cylinder(part, name, radius, length, xyz, material=hardware)

    support = model.part("support_frame")
    leaf = model.part("bridge_leaf")

    brace_angle = atan2(0.50, 0.70)

    # Fixed support frame: concrete machinery plinth, steel pedestals, guarded side bearings.
    add_box(support, "base_slab", (2.00, 4.80, 0.45), (-0.25, 0.0, 0.225), material=concrete)
    add_box(support, "hinge_sill", (0.56, 2.50, 0.22), (-0.86, 0.0, 0.56), material=structural_steel)
    add_box(support, "rear_crossbeam", (0.70, 3.95, 0.25), (-0.55, 0.0, 0.775), material=structural_steel)

    add_box(support, "left_pedestal", (0.75, 0.48, 0.90), (0.00, 2.18, 0.45), material=structural_steel)
    add_box(support, "right_pedestal", (0.75, 0.48, 0.90), (0.00, -2.18, 0.45), material=structural_steel)
    add_box(support, "left_bearing_saddle", (0.36, 0.32, 0.16), (0.00, 2.06, 0.82), material=structural_steel)
    add_box(support, "right_bearing_saddle", (0.36, 0.32, 0.16), (0.00, -2.06, 0.82), material=structural_steel)

    for side_name, side_y in (("left", 2.06), ("right", -2.06)):
        for idx, x_pos in enumerate((-0.24, 0.24), start=1):
            add_box(
                support,
                f"{side_name}_cap_riser_{idx}",
                (0.08, 0.08, 0.30),
                (x_pos, side_y, 1.05),
                material=structural_steel,
            )
        add_box(
            support,
            f"{side_name}_bearing_cap",
            (0.60, 0.32, 0.08),
            (0.00, side_y, 1.24),
            material=structural_steel,
        )

    add_box(
        support,
        "left_diagonal_brace",
        (0.90, 0.16, 0.16),
        (-0.60, 1.70, 0.70),
        rpy=(0.0, brace_angle, 0.0),
        material=structural_steel,
    )
    add_box(
        support,
        "right_diagonal_brace",
        (0.90, 0.16, 0.16),
        (-0.60, -1.70, 0.70),
        rpy=(0.0, brace_angle, 0.0),
        material=structural_steel,
    )

    add_box(support, "left_seat_block", (0.44, 0.32, 0.12), (0.32, 1.28, 0.39), material=structural_steel)
    add_box(support, "right_seat_block", (0.44, 0.32, 0.12), (0.32, -1.28, 0.39), material=structural_steel)

    for side_name, side_y in (("left", 2.06), ("right", -2.06)):
        add_box(
            support,
            f"{side_name}_overtravel_riser",
            (0.10, 0.22, 0.26),
            (-0.26, side_y, 1.03),
            material=safety_yellow,
        )
        add_box(
            support,
            f"{side_name}_overtravel_stop",
            (0.20, 0.24, 0.14),
            (-0.26, side_y, 1.23),
            material=safety_yellow,
        )

    for side_name, side_y in (("left", 2.46), ("right", -2.46)):
        add_box(
            support,
            f"{side_name}_pivot_guard_post_rear",
            (0.08, 0.08, 0.78),
            (-0.24, side_y, 0.84),
            material=safety_yellow,
        )
        add_box(
            support,
            f"{side_name}_pivot_guard_post_front",
            (0.08, 0.08, 0.78),
            (0.20, side_y, 0.84),
            material=safety_yellow,
        )
        add_box(
            support,
            f"{side_name}_pivot_guard_top",
            (0.52, 0.08, 0.08),
            (-0.02, side_y, 1.19),
            material=safety_yellow,
        )
        add_box(
            support,
            f"{side_name}_pivot_guard_midrail",
            (0.52, 0.05, 0.05),
            (-0.02, side_y, 0.84),
            material=safety_yellow,
        )
        add_box(
            support,
            f"{side_name}_lockout_storage",
            (0.18, 0.18, 0.56),
            (0.10, 2.54 if side_name == "left" else -2.54, 0.73),
            material=safety_yellow,
        )
        add_box(
            support,
            f"{side_name}_lockout_bar",
            (0.08, 0.08, 0.72),
            (0.10, 2.54 if side_name == "left" else -2.54, 1.15),
            material=safety_yellow,
        )
        add_box(
            support,
            f"{side_name}_lockout_retainer",
            (0.12, 0.05, 0.12),
            (0.10, 2.50 if side_name == "left" else -2.50, 1.28),
            material=hardware,
        )

    for side_name, side_y in (("left", 2.06), ("right", -2.06)):
        for bolt_idx, (x_pos, y_pos) in enumerate(((-0.18, side_y - 0.08), (-0.18, side_y + 0.08), (0.18, side_y - 0.08), (0.18, side_y + 0.08)), start=1):
            add_z_bolt(
                support,
                f"{side_name}_cap_bolt_{bolt_idx}",
                (x_pos, y_pos, 1.27),
            )

    # Rotating bridge leaf: box-girder leaf, trunnion cheeks, shoes, guards, and stop strikers.
    add_box(leaf, "root_box", (0.80, 1.70, 0.52), (0.40, 0.0, -0.21), material=structural_steel)
    add_box(leaf, "deck_plate", (4.80, 3.20, 0.10), (2.40, 0.0, -0.05), material=deck_steel)
    add_box(leaf, "left_side_girder", (4.20, 0.22, 0.56), (2.60, 1.62, -0.28), material=structural_steel)
    add_box(leaf, "right_side_girder", (4.20, 0.22, 0.56), (2.60, -1.62, -0.28), material=structural_steel)
    add_box(leaf, "center_stiffener", (4.10, 0.16, 0.22), (2.20, 0.0, -0.33), material=structural_steel)
    add_box(leaf, "left_inner_stiffener", (4.10, 0.14, 0.22), (2.20, 0.72, -0.33), material=structural_steel)
    add_box(leaf, "right_inner_stiffener", (4.10, 0.14, 0.22), (2.20, -0.72, -0.33), material=structural_steel)
    add_box(leaf, "nose_beam", (0.22, 3.70, 0.32), (4.69, 0.0, -0.17), material=structural_steel)

    for idx, x_pos in enumerate((0.95, 1.95, 2.95, 3.95), start=1):
        add_box(
            leaf,
            f"diaphragm_{idx}",
            (0.18, 3.18, 0.46),
            (x_pos, 0.0, -0.28),
            material=structural_steel,
        )

    add_box(leaf, "left_trunnion_cheek", (0.58, 0.08, 0.64), (0.14, 1.76, -0.17), material=structural_steel)
    add_box(leaf, "right_trunnion_cheek", (0.58, 0.08, 0.64), (0.14, -1.76, -0.17), material=structural_steel)
    add_cylinder(
        leaf,
        "left_trunnion",
        0.15,
        0.56,
        (0.00, 2.06, 0.0),
        rpy=(-pi / 2.0, 0.0, 0.0),
        material=hardware,
    )
    add_cylinder(
        leaf,
        "right_trunnion",
        0.15,
        0.56,
        (0.00, -2.06, 0.0),
        rpy=(-pi / 2.0, 0.0, 0.0),
        material=hardware,
    )
    add_box(
        leaf,
        "left_trunnion_gusset",
        (0.72, 0.08, 0.18),
        (0.48, 1.70, -0.31),
        rpy=(0.0, -0.62, 0.0),
        material=structural_steel,
    )
    add_box(
        leaf,
        "right_trunnion_gusset",
        (0.72, 0.08, 0.18),
        (0.48, -1.70, -0.31),
        rpy=(0.0, -0.62, 0.0),
        material=structural_steel,
    )

    add_box(leaf, "left_rest_shoe", (0.44, 0.44, 0.08), (0.32, 1.40, -0.56), material=structural_steel)
    add_box(leaf, "right_rest_shoe", (0.44, 0.44, 0.08), (0.32, -1.40, -0.56), material=structural_steel)
    add_box(leaf, "left_lock_capture", (0.18, 0.10, 0.22), (0.42, 1.83, 0.10), material=safety_yellow)
    add_box(leaf, "right_lock_capture", (0.18, 0.10, 0.22), (0.42, -1.83, 0.10), material=safety_yellow)
    add_box(leaf, "left_stop_striker", (0.12, 0.12, 0.22), (0.08, 1.79, 0.14), material=safety_yellow)
    add_box(leaf, "right_stop_striker", (0.12, 0.12, 0.22), (0.08, -1.79, 0.14), material=safety_yellow)

    rail_post_x = (0.35, 1.55, 2.75, 3.95)
    for side_name, side_y in (("left", 1.76), ("right", -1.76)):
        for idx, x_pos in enumerate(rail_post_x, start=1):
            add_box(
                leaf,
                f"{side_name}_rail_post_{idx}",
                (0.06, 0.06, 1.05),
                (x_pos, side_y, 0.47),
                material=safety_yellow,
            )
        add_box(
            leaf,
            f"{side_name}_top_rail",
            (4.34, 0.06, 0.06),
            (2.17, side_y, 0.96),
            material=safety_yellow,
        )
        add_box(
            leaf,
            f"{side_name}_mid_rail",
            (4.34, 0.05, 0.05),
            (2.17, side_y, 0.58),
            material=safety_yellow,
        )
        add_box(
            leaf,
            f"{side_name}_toe_board",
            (4.54, 0.03, 0.16),
            (2.27, 1.74 if side_name == "left" else -1.74, 0.08),
            material=safety_yellow,
        )

    for side_name, side_y, bolt_y in (("left", 1.76, 1.79), ("right", -1.76, -1.79)):
        for bolt_idx, (x_pos, z_pos) in enumerate(((0.00, -0.08), (0.00, -0.48), (0.26, -0.08), (0.26, -0.48)), start=1):
            add_y_bolt(
                leaf,
                f"{side_name}_cheek_bolt_{bolt_idx}",
                (x_pos, bolt_y, z_pos),
            )

    model.articulation(
        "support_to_leaf",
        ArticulationType.REVOLUTE,
        parent=support,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.45, lower=0.0, upper=1.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("support_to_leaf")

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

    limits = hinge.motion_limits
    ctx.check(
        "leaf_hinge_uses_side_trunnion_axis",
        hinge.axis == (0.0, -1.0, 0.0),
        f"expected hinge axis (0, -1, 0), got {hinge.axis}",
    )
    ctx.check(
        "leaf_hinge_limits_match_raise_motion",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - 0.0) < 1e-9
        and 1.10 <= limits.upper <= 1.30,
        f"unexpected motion limits: {limits}",
    )

    ctx.expect_contact(
        leaf,
        support,
        elem_a="left_rest_shoe",
        elem_b="left_seat_block",
        name="left_rest_shoe_seated",
    )
    ctx.expect_contact(
        leaf,
        support,
        elem_a="right_rest_shoe",
        elem_b="right_seat_block",
        name="right_rest_shoe_seated",
    )
    ctx.expect_gap(
        leaf,
        support,
        axis="z",
        positive_elem="left_trunnion",
        negative_elem="left_bearing_saddle",
        max_gap=0.001,
        max_penetration=0.0,
        name="left_trunnion_bearing_support",
    )
    ctx.expect_gap(
        leaf,
        support,
        axis="z",
        positive_elem="right_trunnion",
        negative_elem="right_bearing_saddle",
        max_gap=0.001,
        max_penetration=0.0,
        name="right_trunnion_bearing_support",
    )
    ctx.expect_overlap(
        leaf,
        support,
        axes="xy",
        elem_a="left_trunnion",
        elem_b="left_bearing_saddle",
        min_overlap=0.18,
        name="left_trunnion_over_bearing_footprint",
    )
    ctx.expect_overlap(
        leaf,
        support,
        axes="xy",
        elem_a="right_trunnion",
        elem_b="right_bearing_saddle",
        min_overlap=0.18,
        name="right_trunnion_over_bearing_footprint",
    )

    leaf_aabb = ctx.part_world_aabb(leaf)
    if leaf_aabb is None:
        ctx.fail("bridge_leaf_has_geometry", "bridge leaf AABB is unavailable")
    else:
        leaf_dims = tuple(leaf_aabb[1][i] - leaf_aabb[0][i] for i in range(3))
        ctx.check(
            "bridge_leaf_has_heavy_duty_span",
            leaf_dims[0] > 4.7 and leaf_dims[1] > 3.6 and leaf_dims[2] > 1.4,
            f"bridge leaf dims too small for the requested heavy-duty silhouette: {leaf_dims}",
        )

    def aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    closed_nose = ctx.part_element_world_aabb(leaf, elem="nose_beam")
    with ctx.pose({hinge: 1.15}):
        open_nose = ctx.part_element_world_aabb(leaf, elem="nose_beam")

    if closed_nose is None or open_nose is None:
        ctx.fail("nose_beam_pose_tracking", "nose beam AABB unavailable for articulation check")
    else:
        closed_center = aabb_center(closed_nose)
        open_center = aabb_center(open_nose)
        ctx.check(
            "leaf_nose_rises_when_opened",
            open_center[2] > closed_center[2] + 3.0,
            f"nose beam did not rise enough: closed={closed_center}, open={open_center}",
        )
        ctx.check(
            "leaf_nose_swings_back_toward_support_when_opened",
            open_center[0] < closed_center[0] - 2.3,
            f"nose beam did not rotate around the trunnion axis as expected: closed={closed_center}, open={open_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
