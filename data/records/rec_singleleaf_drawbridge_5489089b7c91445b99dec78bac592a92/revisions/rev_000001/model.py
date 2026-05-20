from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    section_loft,
)


def _leaf_section(
    x: float,
    width: float,
    *,
    deck_z: float,
    shoulder_z: float,
    lower_z: float,
    bottom_z: float,
) -> list[tuple[float, float, float]]:
    half = width * 0.5
    return [
        (x, -half, -0.02),
        (x, -half + 0.10, bottom_z),
        (x, -half + 0.26, lower_z),
        (x, -half + 0.40, shoulder_z),
        (x, -half + 0.62, deck_z),
        (x, half - 0.62, deck_z),
        (x, half - 0.40, shoulder_z),
        (x, half - 0.26, lower_z),
        (x, half - 0.10, bottom_z),
        (x, half, -0.02),
    ]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_singleleaf_drawbridge")

    concrete = model.material("concrete", rgba=(0.56, 0.58, 0.60, 1.0))
    coated_steel = model.material("coated_steel", rgba=(0.22, 0.34, 0.34, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.67, 0.70, 0.72, 1.0))
    stainless_steel = model.material("stainless_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    epdm = model.material("epdm", rgba=(0.09, 0.09, 0.09, 1.0))

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((4.4, 6.2, 3.8)),
        mass=58000.0,
        origin=Origin(xyz=(-0.7, 0.0, 1.9)),
    )
    support_frame.visual(
        Box((4.2, 6.2, 0.9)),
        origin=Origin(xyz=(-1.05, 0.0, 0.45)),
        material=concrete,
        name="foundation",
    )
    support_frame.visual(
        Box((2.5, 3.2, 0.18)),
        origin=Origin(xyz=(-1.30, 0.0, 2.42)),
        material=concrete,
        name="road_apron",
    )
    support_frame.visual(
        Box((0.90, 4.7, 0.60)),
        origin=Origin(xyz=(-0.70, 0.0, 2.15)),
        material=coated_steel,
        name="anchor_rib",
    )
    support_frame.visual(
        Box((1.10, 4.7, 0.40)),
        origin=Origin(xyz=(-0.20, 0.0, 1.80)),
        material=coated_steel,
        name="sill_beam",
    )
    support_frame.visual(
        Box((1.10, 0.84, 2.60)),
        origin=Origin(xyz=(-0.28, 2.66, 2.10)),
        material=coated_steel,
        name="left_tower",
    )
    support_frame.visual(
        Box((1.10, 0.84, 2.60)),
        origin=Origin(xyz=(-0.28, -2.66, 2.10)),
        material=coated_steel,
        name="right_tower",
    )
    support_frame.visual(
        Box((1.05, 5.40, 0.62)),
        origin=Origin(xyz=(-0.95, 0.0, 3.10)),
        material=coated_steel,
        name="portal_beam",
    )
    support_frame.visual(
        Box((0.40, 3.90, 0.06)),
        origin=Origin(xyz=(0.30, 0.0, 2.00)),
        material=stainless_steel,
        name="sill_cap",
    )
    support_frame.visual(
        Box((0.18, 3.50, 0.04)),
        origin=Origin(xyz=(0.32, 0.0, 2.03)),
        material=epdm,
        name="seal_strip",
    )
    for side_name, side_y in (("left", 2.14), ("right", -2.14)):
        support_frame.visual(
            Box((0.74, 0.20, 0.42)),
            origin=Origin(xyz=(0.18, side_y, 1.84)),
            material=stainless_steel,
            name=f"{side_name}_bearing_pedestal",
        )
        support_frame.visual(
            Box((0.86, 0.26, 0.30)),
            origin=Origin(xyz=(-0.02, side_y, 2.85)),
            material=stainless_steel,
            name=f"{side_name}_bearing_hood",
        )
        support_frame.visual(
            Box((0.24, 0.34, 1.04)),
            origin=Origin(xyz=(-0.60, side_y, 2.24)),
            material=stainless_steel,
            name=f"{side_name}_bearing_rear_web",
        )
        support_frame.visual(
            Box((0.98, 0.38, 0.08)),
            origin=Origin(xyz=(0.02, side_y, 3.06)),
            material=stainless_steel,
            name=f"{side_name}_bearing_drip_cap",
        )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((8.4, 4.1, 0.8)),
        mass=22000.0,
        origin=Origin(xyz=(4.2, 0.0, -0.02)),
    )

    leaf_shell = section_loft(
        [
            _leaf_section(0.00, 4.04, deck_z=0.16, shoulder_z=0.10, lower_z=-0.14, bottom_z=-0.34),
            _leaf_section(2.60, 3.98, deck_z=0.19, shoulder_z=0.12, lower_z=-0.13, bottom_z=-0.34),
            _leaf_section(5.90, 3.86, deck_z=0.16, shoulder_z=0.09, lower_z=-0.08, bottom_z=-0.26),
            _leaf_section(8.20, 3.70, deck_z=0.10, shoulder_z=0.04, lower_z=-0.02, bottom_z=-0.12),
        ]
    )
    bridge_leaf.visual(
        _save_mesh("bridge_leaf_shell", leaf_shell),
        material=coated_steel,
        name="leaf_shell",
    )
    bridge_leaf.visual(
        Box((7.20, 0.18, 0.14)),
        origin=Origin(xyz=(4.20, 1.64, 0.18)),
        material=galvanized_steel,
        name="left_curb",
    )
    bridge_leaf.visual(
        Box((7.20, 0.18, 0.14)),
        origin=Origin(xyz=(4.20, -1.64, 0.18)),
        material=galvanized_steel,
        name="right_curb",
    )
    bridge_leaf.visual(
        Box((0.60, 4.08, 0.08)),
        origin=Origin(xyz=(0.18, 0.0, 0.18)),
        material=galvanized_steel,
        name="rear_weather_hood",
    )
    bridge_leaf.visual(
        Box((0.72, 0.24, 0.64)),
        origin=Origin(xyz=(0.18, 1.86, -0.02)),
        material=coated_steel,
        name="left_cheek_plate",
    )
    bridge_leaf.visual(
        Box((0.72, 0.24, 0.64)),
        origin=Origin(xyz=(0.18, -1.86, -0.02)),
        material=coated_steel,
        name="right_cheek_plate",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.23, length=0.32),
        origin=Origin(xyz=(0.0, 2.04, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless_steel,
        name="left_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.23, length=0.32),
        origin=Origin(xyz=(0.0, -2.04, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless_steel,
        name="right_trunnion",
    )
    bridge_leaf.visual(
        Box((0.18, 0.48, 0.12)),
        origin=Origin(xyz=(0.32, 1.15, -0.29)),
        material=stainless_steel,
        name="left_landing_shoe",
    )
    bridge_leaf.visual(
        Box((0.18, 0.48, 0.12)),
        origin=Origin(xyz=(0.32, -1.15, -0.29)),
        material=stainless_steel,
        name="right_landing_shoe",
    )
    bridge_leaf.visual(
        Box((0.28, 0.40, 0.24)),
        origin=Origin(xyz=(0.30, 1.15, -0.15)),
        material=coated_steel,
        name="left_shoe_bracket",
    )
    bridge_leaf.visual(
        Box((0.28, 0.40, 0.24)),
        origin=Origin(xyz=(0.30, -1.15, -0.15)),
        material=coated_steel,
        name="right_shoe_bracket",
    )
    bridge_leaf.visual(
        Box((0.18, 0.48, 0.22)),
        origin=Origin(xyz=(0.24, 1.54, -0.10)),
        material=coated_steel,
        name="left_shoe_gusset",
    )
    bridge_leaf.visual(
        Box((0.18, 0.48, 0.22)),
        origin=Origin(xyz=(0.24, -1.54, -0.10)),
        material=coated_steel,
        name="right_shoe_gusset",
    )
    bridge_leaf.visual(
        Box((0.12, 3.70, 0.42)),
        origin=Origin(xyz=(8.08, 0.0, -0.01)),
        material=galvanized_steel,
        name="front_fascia",
    )
    bridge_leaf.visual(
        Box((0.36, 3.86, 0.05)),
        origin=Origin(xyz=(8.06, 0.0, 0.16)),
        material=galvanized_steel,
        name="nose_overhang",
    )
    bridge_leaf.visual(
        Box((0.18, 3.28, 0.04)),
        origin=Origin(xyz=(8.16, 0.0, -0.22)),
        material=galvanized_steel,
        name="drip_edge",
    )

    model.articulation(
        "support_to_leaf",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 2.40)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=280000.0, velocity=0.35, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("support_to_leaf")

    seal_strip = frame.get_visual("seal_strip")
    road_apron = frame.get_visual("road_apron")
    left_tower = frame.get_visual("left_tower")
    left_bearing_hood = frame.get_visual("left_bearing_hood")

    left_landing_shoe = leaf.get_visual("left_landing_shoe")
    right_landing_shoe = leaf.get_visual("right_landing_shoe")
    left_cheek_plate = leaf.get_visual("left_cheek_plate")
    right_cheek_plate = leaf.get_visual("right_cheek_plate")
    left_shoe_bracket = leaf.get_visual("left_shoe_bracket")
    right_shoe_bracket = leaf.get_visual("right_shoe_bracket")
    left_shoe_gusset = leaf.get_visual("left_shoe_gusset")
    right_shoe_gusset = leaf.get_visual("right_shoe_gusset")
    left_trunnion = leaf.get_visual("left_trunnion")
    nose_overhang = leaf.get_visual("nose_overhang")

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

    ctx.check(
        "leaf_hinge_orientation",
        hinge.axis == (0.0, -1.0, 0.0),
        details=f"expected opening axis (0, -1, 0), got {hinge.axis}",
    )
    ctx.check(
        "leaf_motion_limits",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper >= 1.2,
        details="drawbridge leaf should open upward from the closed pose through a substantial angle",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            leaf,
            frame,
            elem_a=left_landing_shoe,
            elem_b=seal_strip,
            name="left_landing_shoe_seats_on_weather_seal",
        )
        ctx.expect_contact(
            leaf,
            frame,
            elem_a=right_landing_shoe,
            elem_b=seal_strip,
            name="right_landing_shoe_seats_on_weather_seal",
        )
        ctx.expect_contact(
            leaf,
            leaf,
            elem_a=left_shoe_gusset,
            elem_b=left_shoe_bracket,
            name="left_shoe_gusset_supports_bracket",
        )
        ctx.expect_contact(
            leaf,
            leaf,
            elem_a=left_shoe_gusset,
            elem_b=left_cheek_plate,
            name="left_shoe_gusset_ties_into_side_cheek",
        )
        ctx.expect_contact(
            leaf,
            leaf,
            elem_a=right_shoe_gusset,
            elem_b=right_shoe_bracket,
            name="right_shoe_gusset_supports_bracket",
        )
        ctx.expect_contact(
            leaf,
            leaf,
            elem_a=right_shoe_gusset,
            elem_b=right_cheek_plate,
            name="right_shoe_gusset_ties_into_side_cheek",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="y",
            positive_elem=left_tower,
            negative_elem=left_trunnion,
            min_gap=0.02,
            max_gap=0.12,
            name="left_trunnion_clears_side_frame",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="z",
            positive_elem=left_bearing_hood,
            negative_elem=left_trunnion,
            min_gap=0.03,
            max_gap=0.14,
            name="left_trunnion_stays_under_protective_hood",
        )

    with ctx.pose({hinge: 1.0}):
        ctx.expect_gap(
            leaf,
            frame,
            axis="z",
            positive_elem=nose_overhang,
            negative_elem=road_apron,
            min_gap=3.0,
            name="open_leaf_nose_rises_clear_of_fixed_apron",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
