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
    section_loft,
)


def _thin_mesh_from_xz_outline(
    outline: list[tuple[float, float]],
    *,
    thickness: float,
    name: str,
):
    half_thickness = thickness * 0.5
    sections = [
        [(x, -half_thickness, z) for x, z in outline],
        [(x, half_thickness, z) for x, z in outline],
    ]
    return mesh_from_geometry(section_loft(sections), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_slider_utility_knife")

    body_yellow = model.material("body_yellow", rgba=(0.90, 0.78, 0.18, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.17, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.83, 0.85, 0.88, 1.0))

    blade_mesh = _thin_mesh_from_xz_outline(
        [
            (-0.020, -0.0045),
            (0.012, -0.0045),
            (0.032, 0.0),
            (0.018, 0.0045),
            (-0.020, 0.0045),
        ],
        thickness=0.0008,
        name="knife_blade",
    )
    spare_blade_mesh = _thin_mesh_from_xz_outline(
        [
            (-0.010, -0.0030),
            (0.006, -0.0030),
            (0.013, 0.0),
            (0.008, 0.0030),
            (-0.010, 0.0030),
        ],
        thickness=0.0007,
        name="spare_blade",
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.166, 0.0025, 0.018)),
        origin=Origin(xyz=(0.0, -0.01475, 0.0)),
        material=body_yellow,
        name="left_shell",
    )
    handle.visual(
        Box((0.078, 0.0025, 0.018)),
        origin=Origin(xyz=(-0.044, 0.01475, 0.0)),
        material=body_yellow,
        name="right_rear_shell",
    )
    handle.visual(
        Box((0.023, 0.0025, 0.018)),
        origin=Origin(xyz=(0.0715, 0.01475, 0.0)),
        material=body_yellow,
        name="right_front_shell",
    )
    handle.visual(
        Box((0.068, 0.0025, 0.0045)),
        origin=Origin(xyz=(0.029, 0.01475, 0.00675)),
        material=body_yellow,
        name="right_slot_upper",
    )
    handle.visual(
        Box((0.068, 0.0025, 0.0055)),
        origin=Origin(xyz=(0.029, 0.01475, -0.00625)),
        material=body_yellow,
        name="right_slot_lower",
    )
    handle.visual(
        Box((0.135, 0.027, 0.003)),
        origin=Origin(xyz=(-0.003, 0.0, 0.0080)),
        material=charcoal,
        name="top_spine",
    )
    handle.visual(
        Box((0.152, 0.027, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.0080)),
        material=charcoal,
        name="bottom_spine",
    )
    handle.visual(
        Box((0.026, 0.027, 0.0025)),
        origin=Origin(xyz=(0.074, 0.0, 0.00625)),
        material=steel,
        name="nose_top_guide",
    )
    handle.visual(
        Box((0.026, 0.027, 0.0025)),
        origin=Origin(xyz=(0.074, 0.0, -0.00475)),
        material=steel,
        name="nose_bottom_guide",
    )
    handle.visual(
        Box((0.050, 0.004, 0.002)),
        origin=Origin(xyz=(0.018, 0.0, 0.0105)),
        material=steel,
        name="top_track",
    )
    handle.visual(
        Box((0.060, 0.0015, 0.008)),
        origin=Origin(xyz=(-0.005, -0.01675, 0.0)),
        material=rubber_black,
        name="left_grip_pad",
    )
    handle.visual(
        Box((0.050, 0.0015, 0.008)),
        origin=Origin(xyz=(-0.020, 0.01675, -0.0020)),
        material=rubber_black,
        name="right_grip_pad",
    )
    handle.visual(
        Box((0.034, 0.027, 0.005)),
        origin=Origin(xyz=(0.072, 0.0, 0.0112), rpy=(0.0, -0.34, 0.0)),
        material=body_yellow,
        name="nose_top_bevel",
    )
    handle.visual(
        Box((0.026, 0.027, 0.0045)),
        origin=Origin(xyz=(0.074, 0.0, -0.0094), rpy=(0.0, 0.22, 0.0)),
        material=body_yellow,
        name="nose_bottom_bevel",
    )
    handle.visual(
        Box((0.006, 0.003, 0.016)),
        origin=Origin(xyz=(-0.080, 0.0175, 0.0)),
        material=steel,
        name="hinge_bracket",
    )
    handle.visual(
        Cylinder(radius=0.0025, length=0.004),
        origin=Origin(xyz=(-0.083, 0.0185, -0.006)),
        material=steel,
        name="cap_hinge_lower",
    )
    handle.visual(
        Cylinder(radius=0.0025, length=0.004),
        origin=Origin(xyz=(-0.083, 0.0185, 0.006)),
        material=steel,
        name="cap_hinge_upper",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.176, 0.038, 0.022)),
        mass=0.22,
    )

    carrier = model.part("blade_carrier")
    carrier.visual(
        Box((0.050, 0.023, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.0015)),
        material=steel,
        name="carrier_block",
    )
    carrier.visual(
        Box((0.030, 0.012, 0.0035)),
        origin=Origin(xyz=(0.013, 0.0, 0.0040)),
        material=charcoal,
        name="blade_clamp",
    )
    carrier.visual(
        Box((0.010, 0.018, 0.004)),
        origin=Origin(xyz=(-0.018, 0.0, 0.0030)),
        material=charcoal,
        name="slider_boss",
    )
    carrier.visual(
        blade_mesh,
        origin=Origin(xyz=(0.034, 0.0, 0.0005)),
        material=blade_steel,
        name="blade",
    )
    carrier.inertial = Inertial.from_geometry(
        Box((0.062, 0.024, 0.014)),
        mass=0.04,
    )

    slider = model.part("thumb_slider")
    slider.visual(
        Box((0.010, 0.0045, 0.004)),
        origin=Origin(xyz=(0.0, 0.01375, 0.0000)),
        material=charcoal,
        name="stem",
    )
    slider.visual(
        Box((0.022, 0.0050, 0.006)),
        origin=Origin(xyz=(0.0, 0.0185, 0.0005)),
        material=rubber_black,
        name="thumb_pad",
    )
    slider.visual(
        Box((0.012, 0.0015, 0.0015)),
        origin=Origin(xyz=(0.0, 0.02175, 0.00425)),
        material=rubber_black,
        name="thumb_ridge",
    )
    slider.inertial = Inertial.from_geometry(
        Box((0.024, 0.010, 0.008)),
        mass=0.01,
    )

    end_cap = model.part("end_cap")
    end_cap.visual(
        Cylinder(radius=0.0025, length=0.008),
        origin=Origin(),
        material=steel,
        name="hinge_barrel",
    )
    end_cap.visual(
        Box((0.004, 0.004, 0.015)),
        origin=Origin(xyz=(-0.002, -0.0015, 0.0)),
        material=charcoal,
        name="hinge_leaf",
    )
    end_cap.visual(
        Box((0.020, 0.032, 0.0018)),
        origin=Origin(xyz=(-0.010, -0.016, 0.0071)),
        material=charcoal,
        name="cap_top",
    )
    end_cap.visual(
        Box((0.020, 0.032, 0.0018)),
        origin=Origin(xyz=(-0.010, -0.016, -0.0071)),
        material=charcoal,
        name="cap_bottom",
    )
    end_cap.visual(
        Box((0.020, 0.002, 0.015)),
        origin=Origin(xyz=(-0.010, -0.031, 0.0)),
        material=charcoal,
        name="cap_left_wall",
    )
    end_cap.visual(
        Box((0.020, 0.0025, 0.015)),
        origin=Origin(xyz=(-0.010, -0.00125, 0.0)),
        material=charcoal,
        name="cap_right_wall",
    )
    end_cap.visual(
        Box((0.002, 0.032, 0.015)),
        origin=Origin(xyz=(-0.019, -0.016, 0.0)),
        material=body_yellow,
        name="cap_rear_wall",
    )
    end_cap.visual(
        spare_blade_mesh,
        origin=Origin(xyz=(-0.010, -0.016, -0.00585)),
        material=blade_steel,
        name="spare_blade",
    )
    end_cap.inertial = Inertial.from_geometry(
        Box((0.024, 0.034, 0.018)),
        mass=0.02,
    )

    model.articulation(
        "handle_to_blade_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carrier,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.12,
            lower=0.0,
            upper=0.030,
        ),
    )
    model.articulation(
        "carrier_to_thumb_slider",
        ArticulationType.FIXED,
        parent=carrier,
        child=slider,
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
    )
    model.articulation(
        "handle_to_end_cap",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=end_cap,
        origin=Origin(xyz=(-0.083, 0.0185, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=-1.25,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    carrier = object_model.get_part("blade_carrier")
    slider = object_model.get_part("thumb_slider")
    end_cap = object_model.get_part("end_cap")

    carrier_slide = object_model.get_articulation("handle_to_blade_carrier")
    end_cap_hinge = object_model.get_articulation("handle_to_end_cap")

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

    ctx.expect_contact(carrier, handle, elem_a="carrier_block", elem_b="bottom_spine")
    ctx.expect_within(carrier, handle, axes="yz", inner_elem="carrier_block", margin=0.001)
    ctx.expect_contact(slider, carrier, elem_a="stem", elem_b="carrier_block")
    ctx.expect_gap(
        slider,
        handle,
        axis="y",
        positive_elem="thumb_pad",
        negative_elem="right_slot_upper",
        min_gap=0.0,
        max_gap=0.0005,
    )
    ctx.expect_contact(end_cap, handle, elem_a="hinge_barrel", elem_b="cap_hinge_lower")

    slot_side_aabb = ctx.part_element_world_aabb(handle, elem="right_slot_upper")
    thumb_aabb = ctx.part_element_world_aabb(slider, elem="thumb_pad")
    thumb_outside = (
        slot_side_aabb is not None
        and thumb_aabb is not None
        and thumb_aabb[1][1] > slot_side_aabb[1][1] + 0.004
    )
    ctx.check(
        "thumb_slider_protrudes_from_side_slot",
        thumb_outside,
        details=(
            "Thumb pad should stand proud of the right side shell."
            if not thumb_outside
            else ""
        ),
    )

    carrier_rest = ctx.part_world_position(carrier)
    slider_rest = ctx.part_world_position(slider)
    with ctx.pose({carrier_slide: 0.028}):
        carrier_extended = ctx.part_world_position(carrier)
        slider_extended = ctx.part_world_position(slider)
        travel_ok = (
            carrier_rest is not None
            and slider_rest is not None
            and carrier_extended is not None
            and slider_extended is not None
            and carrier_extended[0] > carrier_rest[0] + 0.025
            and abs(
                (slider_extended[0] - slider_rest[0])
                - (carrier_extended[0] - carrier_rest[0])
            )
            < 1e-6
        )
        ctx.check(
            "carrier_and_thumb_slider_translate_together",
            travel_ok,
            details="Slider should follow the blade carrier along the handle axis.",
        )
        ctx.expect_contact(carrier, handle, elem_a="carrier_block", elem_b="bottom_spine")
        ctx.expect_within(carrier, handle, axes="yz", inner_elem="carrier_block", margin=0.001)

    cap_closed = ctx.part_element_world_aabb(end_cap, elem="cap_rear_wall")
    with ctx.pose({end_cap_hinge: -1.05}):
        cap_open = ctx.part_element_world_aabb(end_cap, elem="cap_rear_wall")
        cap_swings = (
            cap_closed is not None
            and cap_open is not None
            and cap_open[1][1] > cap_closed[1][1] + 0.012
        )
        ctx.check(
            "rear_end_cap_swings_open",
            cap_swings,
            details="Rear spare-blade end cap should swing outward on its hinge.",
        )
        ctx.expect_contact(end_cap, handle, elem_a="hinge_barrel", elem_b="cap_hinge_lower")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
