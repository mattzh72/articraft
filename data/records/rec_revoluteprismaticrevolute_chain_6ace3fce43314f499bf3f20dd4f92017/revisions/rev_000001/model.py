from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolute_prismatic_revolute_chain")

    painted = Material("painted_base", rgba=(0.08, 0.10, 0.12, 1.0))
    dark = Material("blackened_steel", rgba=(0.015, 0.017, 0.018, 1.0))
    rail = Material("brushed_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    slide = Material("satin_aluminum", rgba=(0.78, 0.76, 0.70, 1.0))
    brass = Material("oiled_bronze_bushings", rgba=(0.86, 0.58, 0.22, 1.0))
    warning = Material("yellow_end_fork", rgba=(0.95, 0.68, 0.10, 1.0))

    base = model.part("ground_support")
    base.visual(
        Box((0.36, 0.20, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=painted,
        name="base_plate",
    )
    for y, name in ((0.062, "cheek_0"), (-0.062, "cheek_1")):
        base.visual(
            Box((0.070, 0.025, 0.184)),
            origin=Origin(xyz=(0.0, y, 0.122)),
            material=painted,
            name=name,
        )
        base.visual(
            Cylinder(radius=0.019, length=0.010),
            origin=Origin(xyz=(0.0, y, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"cheek_bushing_{name[-1]}",
        )
    base.visual(
        Cylinder(radius=0.012, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.170), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="root_pin",
    )
    base.visual(
        Box((0.050, 0.135, 0.020)),
        origin=Origin(xyz=(-0.030, 0.0, 0.042)),
        material=painted,
        name="rear_tie",
    )

    guide = model.part("swinging_guide")
    guide.visual(
        Cylinder(radius=0.045, length=0.078),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rail,
        name="root_hub",
    )
    guide.visual(
        Box((0.280, 0.014, 0.040)),
        origin=Origin(xyz=(0.200, 0.034, 0.0)),
        material=rail,
        name="guide_rail_0",
    )
    guide.visual(
        Box((0.280, 0.014, 0.040)),
        origin=Origin(xyz=(0.200, -0.034, 0.0)),
        material=rail,
        name="guide_rail_1",
    )
    guide.visual(
        Box((0.100, 0.014, 0.040)),
        origin=Origin(xyz=(0.070, 0.034, 0.0)),
        material=rail,
        name="hub_strap_0",
    )
    guide.visual(
        Box((0.100, 0.014, 0.040)),
        origin=Origin(xyz=(0.070, -0.034, 0.0)),
        material=rail,
        name="hub_strap_1",
    )
    guide.visual(
        Box((0.022, 0.095, 0.014)),
        origin=Origin(xyz=(0.085, 0.0, 0.024)),
        material=rail,
        name="proximal_bridge",
    )
    guide.visual(
        Box((0.022, 0.095, 0.014)),
        origin=Origin(xyz=(0.325, 0.0, 0.024)),
        material=rail,
        name="distal_bridge",
    )

    slider = model.part("sliding_link")
    slider.visual(
        Box((0.320, 0.026, 0.022)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=slide,
        name="slide_bar",
    )
    slider.visual(
        Box((0.210, 0.014, 0.022)),
        origin=Origin(xyz=(0.110, 0.020, 0.0)),
        material=brass,
        name="slide_shoe_0",
    )
    slider.visual(
        Box((0.210, 0.014, 0.022)),
        origin=Origin(xyz=(0.110, -0.020, 0.0)),
        material=brass,
        name="slide_shoe_1",
    )
    slider.visual(
        Box((0.035, 0.085, 0.022)),
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
        material=slide,
        name="clevis_yoke",
    )
    for y, name in ((0.040, "clevis_ear_0"), (-0.040, "clevis_ear_1")):
        slider.visual(
            Box((0.052, 0.012, 0.040)),
            origin=Origin(xyz=(0.297, y, 0.0)),
            material=slide,
            name=name,
        )
        slider.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(xyz=(0.297, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"ear_bushing_{name[-1]}",
        )
    slider.visual(
        Cylinder(radius=0.010, length=0.095),
        origin=Origin(xyz=(0.297, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="distal_pin",
    )

    fork = model.part("distal_fork")
    fork.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warning,
        name="tab_boss",
    )
    for y, name in ((0.025, "fork_tine_0"), (-0.025, "fork_tine_1")):
        fork.visual(
            Box((0.114, 0.014, 0.018)),
            origin=Origin(xyz=(0.075, y, 0.0)),
            material=warning,
            name=name,
        )
    fork.visual(
        Box((0.024, 0.064, 0.018)),
        origin=Origin(xyz=(0.138, 0.0, 0.0)),
        material=warning,
        name="fork_tip",
    )

    model.articulation(
        "root_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=guide,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-0.25, upper=0.95),
        motion_properties=MotionProperties(damping=0.2, friction=0.03),
    )
    model.articulation(
        "middle_slide",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=slider,
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=0.0, upper=0.18),
        motion_properties=MotionProperties(damping=0.4, friction=0.08),
    )
    model.articulation(
        "distal_pivot",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=fork,
        origin=Origin(xyz=(0.297, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.9, upper=0.9),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("ground_support")
    guide = object_model.get_part("swinging_guide")
    slider = object_model.get_part("sliding_link")
    fork = object_model.get_part("distal_fork")
    root_hinge = object_model.get_articulation("root_hinge")
    middle_slide = object_model.get_articulation("middle_slide")
    distal_pivot = object_model.get_articulation("distal_pivot")

    ctx.check(
        "chain is revolute prismatic revolute",
        root_hinge.articulation_type == ArticulationType.REVOLUTE
        and middle_slide.articulation_type == ArticulationType.PRISMATIC
        and distal_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"types: {root_hinge.articulation_type}, "
            f"{middle_slide.articulation_type}, {distal_pivot.articulation_type}"
        ),
    )

    ctx.allow_overlap(
        base,
        guide,
        elem_a="root_pin",
        elem_b="root_hub",
        reason="The grounded hinge pin intentionally passes through the rotating hub bushing.",
    )
    ctx.expect_within(
        base,
        guide,
        axes="xz",
        inner_elem="root_pin",
        outer_elem="root_hub",
        name="root hinge pin lies inside hub bore envelope",
    )
    ctx.expect_overlap(
        base,
        guide,
        axes="y",
        elem_a="root_pin",
        elem_b="root_hub",
        min_overlap=0.070,
        name="root hinge pin spans the hub",
    )

    ctx.allow_overlap(
        slider,
        fork,
        elem_a="distal_pin",
        elem_b="tab_boss",
        reason="The distal pin intentionally passes through the fork tab boss.",
    )
    ctx.expect_within(
        slider,
        fork,
        axes="xz",
        inner_elem="distal_pin",
        outer_elem="tab_boss",
        name="distal pin lies inside tab boss envelope",
    )
    ctx.expect_overlap(
        slider,
        fork,
        axes="y",
        elem_a="distal_pin",
        elem_b="tab_boss",
        min_overlap=0.035,
        name="distal pin spans the tab boss",
    )

    ctx.expect_overlap(
        slider,
        guide,
        axes="x",
        elem_a="slide_bar",
        elem_b="guide_rail_0",
        min_overlap=0.25,
        name="collapsed slide remains carried by rail",
    )
    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({middle_slide: 0.18}):
        ctx.expect_overlap(
            slider,
            guide,
            axes="x",
            elem_a="slide_bar",
            elem_b="guide_rail_0",
            min_overlap=0.10,
            name="extended slide retains rail engagement",
        )
        extended_slider_pos = ctx.part_world_position(slider)
    ctx.check(
        "middle prismatic joint extends outward",
        rest_slider_pos is not None
        and extended_slider_pos is not None
        and extended_slider_pos[0] > rest_slider_pos[0] + 0.16,
        details=f"rest={rest_slider_pos}, extended={extended_slider_pos}",
    )

    rest_root_pos = ctx.part_world_position(slider)
    with ctx.pose({root_hinge: 0.55}):
        raised_root_pos = ctx.part_world_position(slider)
    ctx.check(
        "root revolute joint lifts the chain",
        rest_root_pos is not None
        and raised_root_pos is not None
        and raised_root_pos[2] > rest_root_pos[2] + 0.05,
        details=f"rest={rest_root_pos}, raised={raised_root_pos}",
    )

    def _aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        low, high = aabb
        return 0.5 * (low[2] + high[2])

    rest_tip_z = _aabb_center_z(ctx.part_element_world_aabb(fork, elem="fork_tip"))
    with ctx.pose({distal_pivot: 0.55}):
        raised_tip_z = _aabb_center_z(ctx.part_element_world_aabb(fork, elem="fork_tip"))
    ctx.check(
        "distal revolute joint swings the fork",
        rest_tip_z is not None and raised_tip_z is not None and raised_tip_z > rest_tip_z + 0.05,
        details=f"rest_tip_z={rest_tip_z}, raised_tip_z={raised_tip_z}",
    )

    return ctx.report()


object_model = build_object_model()
