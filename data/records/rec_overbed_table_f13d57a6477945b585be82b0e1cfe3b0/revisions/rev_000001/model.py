from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_care_overbed_table")

    model.material("powder_coated_steel", rgba=(0.82, 0.84, 0.82, 1.0))
    model.material("dark_steel", rgba=(0.18, 0.19, 0.19, 1.0))
    model.material("brushed_metal", rgba=(0.60, 0.62, 0.60, 1.0))
    model.material("maple_laminate", rgba=(0.86, 0.72, 0.50, 1.0))
    model.material("dark_edge_band", rgba=(0.20, 0.14, 0.09, 1.0))
    model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    model.material("caster_gray", rgba=(0.46, 0.47, 0.46, 1.0))
    model.material("red_lock", rgba=(0.72, 0.05, 0.035, 1.0))

    # Object frame: +X reaches toward the patient/bed side, +Z is up.
    base = model.part("base")

    # Low offset base frame: rectangular steel tubing with the upright mounted
    # near one side so the tabletop can cantilever over a bed or recliner.
    rail_z = 0.132
    base.visual(
        Box((1.00, 0.045, 0.045)),
        origin=Origin(xyz=(0.00, 0.25, rail_z)),
        material="powder_coated_steel",
        name="side_rail_0",
    )
    base.visual(
        Box((1.00, 0.045, 0.045)),
        origin=Origin(xyz=(0.00, -0.25, rail_z)),
        material="powder_coated_steel",
        name="side_rail_1",
    )
    base.visual(
        Box((0.045, 0.55, 0.045)),
        origin=Origin(xyz=(-0.46, 0.00, rail_z)),
        material="powder_coated_steel",
        name="post_side_crossbar",
    )
    base.visual(
        Box((0.045, 0.55, 0.045)),
        origin=Origin(xyz=(0.46, 0.00, rail_z)),
        material="powder_coated_steel",
        name="patient_side_crossbar",
    )
    base.visual(
        Box((0.62, 0.035, 0.035)),
        origin=Origin(xyz=(-0.15, 0.00, 0.102)),
        material="powder_coated_steel",
        name="center_spine",
    )
    base.visual(
        Box((0.17, 0.14, 0.014)),
        origin=Origin(xyz=(-0.38, 0.00, 0.117)),
        material="brushed_metal",
        name="post_foot_plate",
    )

    # Hollow lower sleeve, built as connected square-tube walls rather than a
    # solid block so the sliding inner post has real clearance.
    sleeve_x = -0.38
    sleeve_top = 0.63
    sleeve_len = 0.52
    sleeve_center_z = sleeve_top - sleeve_len / 2.0
    sleeve_outer = 0.064
    sleeve_wall = 0.006
    sleeve_wall_offset = sleeve_outer / 2.0 - sleeve_wall / 2.0
    for sx in (-1.0, 1.0):
        base.visual(
            Box((sleeve_wall, sleeve_outer, sleeve_len)),
            origin=Origin(
                xyz=(sleeve_x + sx * sleeve_wall_offset, 0.00, sleeve_center_z)
            ),
            material="brushed_metal",
            name=f"outer_sleeve_x_{0 if sx < 0.0 else 1}",
        )
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_len)),
        origin=Origin(xyz=(sleeve_x, -sleeve_wall_offset, sleeve_center_z)),
        material="brushed_metal",
        name="outer_sleeve_y_0",
    )
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_len)),
        origin=Origin(xyz=(sleeve_x, sleeve_wall_offset, sleeve_center_z)),
        material="brushed_metal",
        name="outer_sleeve_y_1",
    )
    base.visual(
        Box((0.012, 0.125, 0.17)),
        origin=Origin(xyz=(-0.43, 0.00, 0.195)),
        material="powder_coated_steel",
        name="upright_gusset",
    )

    height_collar = model.part("height_collar")
    collar_outer = 0.130
    collar_inner = 0.064
    collar_wall = (collar_outer - collar_inner) / 2.0
    collar_half = collar_outer / 2.0 - collar_wall / 2.0
    collar_height = 0.070
    height_collar.visual(
        Box((collar_wall, collar_outer, collar_height)),
        origin=Origin(xyz=(-collar_half, 0.0, 0.0)),
        material="dark_steel",
        name="collar_side_0",
    )
    height_collar.visual(
        Box((collar_wall, collar_outer, collar_height)),
        origin=Origin(xyz=(collar_half, 0.0, 0.0)),
        material="dark_steel",
        name="collar_side_1",
    )
    height_collar.visual(
        Box((collar_outer, collar_wall, collar_height)),
        origin=Origin(xyz=(0.0, -collar_half, 0.0)),
        material="dark_steel",
        name="collar_face_0",
    )
    height_collar.visual(
        Box((collar_outer, collar_wall, collar_height)),
        origin=Origin(xyz=(0.0, collar_half, 0.0)),
        material="dark_steel",
        name="collar_face_1",
    )
    height_collar.visual(
        Cylinder(radius=0.011, length=0.052),
        origin=Origin(xyz=(0.0, -0.091, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="threaded_hub",
    )
    model.articulation(
        "base_to_height_collar",
        ArticulationType.FIXED,
        parent=base,
        child=height_collar,
        origin=Origin(xyz=(sleeve_x, 0.0, sleeve_top)),
    )

    top_carriage = model.part("top_carriage")
    top_carriage.visual(
        Box((0.040, 0.040, 0.520)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material="brushed_metal",
        name="inner_mast",
    )
    # Low-friction guide shoes touch the inside of the hollow lower sleeve so
    # the prismatic mast is visibly supported instead of floating in clearance.
    top_carriage.visual(
        Box((0.006, 0.032, 0.090)),
        origin=Origin(xyz=(0.023, 0.0, -0.240)),
        material="black_rubber",
        name="guide_shoe_x_0",
    )
    top_carriage.visual(
        Box((0.006, 0.032, 0.090)),
        origin=Origin(xyz=(-0.023, 0.0, -0.240)),
        material="black_rubber",
        name="guide_shoe_x_1",
    )
    top_carriage.visual(
        Box((0.032, 0.006, 0.090)),
        origin=Origin(xyz=(0.0, 0.023, -0.240)),
        material="black_rubber",
        name="guide_shoe_y_0",
    )
    top_carriage.visual(
        Box((0.032, 0.006, 0.090)),
        origin=Origin(xyz=(0.0, -0.023, -0.240)),
        material="black_rubber",
        name="guide_shoe_y_1",
    )
    top_carriage.visual(
        Box((0.85, 0.055, 0.035)),
        origin=Origin(xyz=(0.40, 0.0, 0.1675)),
        material="brushed_metal",
        name="cantilever_arm",
    )
    top_carriage.visual(
        Box((0.96, 0.42, 0.030)),
        origin=Origin(xyz=(0.50, 0.0, 0.2025)),
        material="maple_laminate",
        name="laminate_top",
    )
    top_carriage.visual(
        Box((0.96, 0.010, 0.035)),
        origin=Origin(xyz=(0.50, 0.215, 0.2025)),
        material="dark_edge_band",
        name="edge_band_0",
    )
    top_carriage.visual(
        Box((0.96, 0.010, 0.035)),
        origin=Origin(xyz=(0.50, -0.215, 0.2025)),
        material="dark_edge_band",
        name="edge_band_1",
    )
    top_carriage.visual(
        Box((0.010, 0.44, 0.035)),
        origin=Origin(xyz=(0.015, 0.0, 0.2025)),
        material="dark_edge_band",
        name="edge_band_post",
    )
    top_carriage.visual(
        Box((0.010, 0.44, 0.035)),
        origin=Origin(xyz=(0.985, 0.0, 0.2025)),
        material="dark_edge_band",
        name="edge_band_patient",
    )
    height_slide = model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=top_carriage,
        origin=Origin(xyz=(sleeve_x, 0.0, sleeve_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.24),
    )

    collar_knob = model.part("collar_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.066,
            0.032,
            body_style="lobed",
            base_diameter=0.046,
            top_diameter=0.060,
            crown_radius=0.0015,
            bore=KnobBore(style="round", diameter=0.010),
            center=False,
        ),
        "height_collar_lobed_knob",
    )
    collar_knob.visual(
        knob_mesh,
        origin=Origin(),
        material="dark_steel",
        name="lobed_knob",
    )
    model.articulation(
        "collar_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=height_collar,
        child=collar_knob,
        origin=Origin(xyz=(0.0, -0.117, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    caster_positions = [
        (0, 0.46, 0.25),
        (1, 0.46, -0.25),
        (2, -0.46, 0.25),
        (3, -0.46, -0.25),
    ]
    wheel_radius = 0.038
    wheel_width = 0.026
    axle_z = 0.040
    for index, x, y in caster_positions:
        fork = model.part(f"fork_{index}")
        fork.visual(
            Box((0.090, 0.060, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.048)),
            material="caster_gray",
            name="top_plate",
        )
        fork.visual(
            Box((0.024, 0.024, 0.0175)),
            origin=Origin(xyz=(0.0, 0.0, 0.06075)),
            material="caster_gray",
            name="swivel_stem",
        )
        fork.visual(
            Box((0.018, 0.006, 0.085)),
            origin=Origin(xyz=(0.0, 0.0245, 0.010)),
            material="caster_gray",
            name="fork_cheek_0",
        )
        fork.visual(
            Box((0.018, 0.006, 0.085)),
            origin=Origin(xyz=(0.0, -0.0245, 0.010)),
            material="caster_gray",
            name="fork_cheek_1",
        )
        model.articulation(
            f"base_to_fork_{index}",
            ArticulationType.FIXED,
            parent=base,
            child=fork,
            origin=Origin(xyz=(x, y, axle_z)),
        )

        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=wheel_radius, length=wheel_width),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="black_rubber",
            name="rubber_tire",
        )
        wheel.visual(
            Cylinder(radius=0.020, length=0.043),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="caster_gray",
            name="wheel_hub",
        )
        model.articulation(
            f"wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=12.0),
        )

        lock_tab = model.part(f"lock_tab_{index}")
        lock_tab.visual(
            Cylinder(radius=0.006, length=0.043),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="dark_steel",
            name="pivot_pin",
        )
        lock_tab.visual(
            Box((0.055, 0.022, 0.008)),
            origin=Origin(xyz=(-0.030, 0.0, 0.0)),
            material="red_lock",
            name="brake_pedal",
        )
        model.articulation(
            f"lock_tab_pivot_{index}",
            ArticulationType.REVOLUTE,
            parent=fork,
            child=lock_tab,
            origin=Origin(xyz=(-0.030, 0.0, 0.058)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=0.75),
        )

    model.meta["height_slide_name"] = height_slide.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    top = object_model.get_part("top_carriage")
    height_slide = object_model.get_articulation("height_slide")
    knob_spin = object_model.get_articulation("collar_knob_spin")

    ctx.check(
        "height post uses a prismatic slide",
        height_slide.articulation_type == ArticulationType.PRISMATIC
        and height_slide.motion_limits is not None
        and height_slide.motion_limits.upper is not None
        and height_slide.motion_limits.upper >= 0.20,
        details=f"type={height_slide.articulation_type}, limits={height_slide.motion_limits}",
    )
    rest_pos = ctx.part_world_position(top)
    with ctx.pose({height_slide: 0.24}):
        high_pos = ctx.part_world_position(top)
    ctx.check(
        "tabletop carriage rises for height adjustment",
        rest_pos is not None
        and high_pos is not None
        and high_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, high={high_pos}",
    )
    top_aabb = ctx.part_element_world_aabb(top, elem="laminate_top")
    patient_crossbar_aabb = ctx.part_element_world_aabb(
        base, elem="patient_side_crossbar"
    )
    ctx.check(
        "laminate top overhangs patient side of base",
        top_aabb is not None
        and patient_crossbar_aabb is not None
        and top_aabb[1][0] > patient_crossbar_aabb[1][0] + 0.08,
        details=f"top_aabb={top_aabb}, patient_crossbar_aabb={patient_crossbar_aabb}",
    )
    ctx.expect_overlap(
        top,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve_y_0",
        min_overlap=0.07,
        name="lower mast remains captured in sleeve at rest",
    )
    with ctx.pose({height_slide: 0.24}):
        ctx.expect_overlap(
            top,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve_y_0",
            min_overlap=0.03,
            name="lower mast remains captured at full height",
        )

    ctx.check(
        "collar knob is a continuous rotary control",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type}",
    )
    for index in range(4):
        wheel_spin = object_model.get_articulation(f"wheel_spin_{index}")
        lock_pivot = object_model.get_articulation(f"lock_tab_pivot_{index}")
        ctx.check(
            f"caster wheel {index} spins on its axle",
            wheel_spin.articulation_type == ArticulationType.CONTINUOUS
            and tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
            details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
        )
        ctx.check(
            f"caster lock tab {index} has a local pivot",
            lock_pivot.articulation_type == ArticulationType.REVOLUTE
            and lock_pivot.motion_limits is not None
            and lock_pivot.motion_limits.upper is not None
            and 0.5 <= lock_pivot.motion_limits.upper <= 1.0,
            details=f"type={lock_pivot.articulation_type}, limits={lock_pivot.motion_limits}",
        )

    return ctx.report()


object_model = build_object_model()
