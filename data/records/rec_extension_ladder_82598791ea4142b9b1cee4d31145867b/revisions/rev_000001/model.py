from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


RAIL_X = 0.245
BASE_RAIL_BOTTOM = 0.205
BASE_RAIL_TOP = 3.55
FLY_OFFSET_Y = 0.14
FLY_JOINT_Z = 0.45
FOOT_HINGE_Z = 0.105


def _add_rung(part, name: str, z: float, *, y: float, length: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _d_ring_mesh(name: str, *, x: float, y: float, z: float):
    """A black bent-steel D guide loop in the horizontal plane around one fly rail."""
    points = [
        (x - 0.080, y - 0.070, z),
        (x + 0.080, y - 0.070, z),
        (x + 0.092, y - 0.020, z),
        (x + 0.074, y + 0.048, z),
        (x + 0.000, y + 0.080, z),
        (x - 0.074, y + 0.048, z),
        (x - 0.092, y - 0.020, z),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=0.009,
            samples_per_segment=12,
            closed_spline=True,
            radial_segments=16,
            cap_ends=False,
            up_hint=(0.0, 0.0, 1.0),
        ),
        name,
    )


def _add_guide_ring(
    part,
    *,
    ring_name: str,
    plate_name: str,
    bolt_prefix: str,
    x: float,
    z: float,
    steel,
) -> None:
    # A bolted back plate touches the FRP stile; the D loop protrudes around the
    # offset fly stile with visible clearance.
    part.visual(
        Box((0.140, 0.028, 0.105)),
        origin=Origin(xyz=(x, 0.054, z)),
        material=steel,
        name=plate_name,
    )
    part.visual(
        _d_ring_mesh(ring_name, x=x, y=FLY_OFFSET_Y, z=z),
        material=steel,
        name=ring_name,
    )
    side_sign = -1.0 if x < 0.0 else 1.0
    part.visual(
        Box((0.053, 0.040, 0.100)),
        origin=Origin(xyz=(x + side_sign * 0.0575, FLY_OFFSET_Y, z)),
        material=steel,
        name=f"{ring_name}_wear_pad",
    )
    for bolt_x in (x - 0.040, x + 0.040):
        part.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(xyz=(bolt_x, 0.073, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"{bolt_prefix}_{bolt_x:+.2f}",
        )


def _add_base_foot_yoke(
    part,
    *,
    outer_name: str,
    inner_name: str,
    hanger_name: str,
    x: float,
    steel,
) -> None:
    # Two cheek plates and a top hanger form the fixed half of the swivel foot hinge.
    part.visual(
        Box((0.090, 0.018, 0.090)),
        origin=Origin(xyz=(x, 0.045, 0.125)),
        material=steel,
        name=outer_name,
    )
    part.visual(
        Box((0.090, 0.018, 0.090)),
        origin=Origin(xyz=(x, -0.045, 0.125)),
        material=steel,
        name=inner_name,
    )
    part.visual(
        Box((0.090, 0.090, 0.040)),
        origin=Origin(xyz=(x, 0.0, 0.187)),
        material=steel,
        name=hanger_name,
    )


def _build_foot(part, *, rubber, steel) -> None:
    part.visual(
        Box((0.160, 0.200, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=rubber,
        name="rubber_pad",
    )
    part.visual(
        Box((0.052, 0.072, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=steel,
        name="pivot_tongue",
    )
    part.visual(
        Box((0.142, 0.180, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=steel,
        name="pad_backing",
    )
    part.visual(
        Box((0.150, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.055, -0.055)),
        material=steel,
        name="front_grip_bar",
    )
    part.visual(
        Box((0.150, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, -0.055, -0.055)),
        material=steel,
        name="rear_grip_bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fibreglass_extension_ladder")

    fiberglass = model.material("safety_yellow_fibreglass", rgba=(0.96, 0.70, 0.16, 1.0))
    fly_fiberglass = model.material("fly_yellow_fibreglass", rgba=(1.0, 0.79, 0.20, 1.0))
    aluminium = model.material("brushed_aluminium", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    warning_red = model.material("red_warning_label", rgba=(0.78, 0.10, 0.07, 1.0))
    label_white = model.material("white_label", rgba=(0.92, 0.90, 0.84, 1.0))

    base = model.part("base_section")
    base_length = BASE_RAIL_TOP - BASE_RAIL_BOTTOM
    base_center_z = (BASE_RAIL_TOP + BASE_RAIL_BOTTOM) * 0.5
    base_stile_names = ("base_stile_0", "base_stile_1")
    base_top_cap_names = ("base_top_cap_0", "base_top_cap_1")
    base_bottom_cap_names = ("base_bottom_cap_0", "base_bottom_cap_1")
    for index, x in enumerate((-RAIL_X, RAIL_X)):
        base.visual(
            Box((0.075, 0.080, base_length)),
            origin=Origin(xyz=(x, 0.0, base_center_z)),
            material=fiberglass,
            name=base_stile_names[index],
        )
        base.visual(
            Box((0.062, 0.086, 0.055)),
            origin=Origin(xyz=(x, 0.0, BASE_RAIL_TOP + 0.0275)),
            material=dark_steel,
            name=base_top_cap_names[index],
        )
        base.visual(
            Box((0.062, 0.086, 0.055)),
            origin=Origin(xyz=(x, 0.0, BASE_RAIL_BOTTOM - 0.0275)),
            material=dark_steel,
            name=base_bottom_cap_names[index],
        )

    for rung_index, z in enumerate([0.46, 0.76, 1.06, 1.36, 1.66, 1.96, 2.26, 2.56, 2.86, 3.16, 3.46]):
        _add_rung(
            base,
            f"base_rung_{rung_index}",
            z,
            y=-0.020,
            length=0.455,
            radius=0.024,
            material=aluminium,
        )

    # Surface labels, proud of the FRP stile faces.
    base.visual(
        Box((0.050, 0.006, 0.260)),
        origin=Origin(xyz=(-RAIL_X, -0.043, 1.54)),
        material=warning_red,
        name="base_warning_label",
    )
    base.visual(
        Box((0.046, 0.007, 0.090)),
        origin=Origin(xyz=(-RAIL_X, -0.047, 1.61)),
        material=label_white,
        name="base_label_patch",
    )

    # Four D-ring guide brackets fixed to the base rails and wrapped around the fly rails.
    for ring_name, plate_name, bolt_prefix, x, z in (
        ("guide_ring_0", "guide_plate_0", "guide_bolt_0", -RAIL_X, 2.32),
        ("guide_ring_1", "guide_plate_1", "guide_bolt_1", RAIL_X, 2.32),
        ("guide_ring_2", "guide_plate_2", "guide_bolt_2", -RAIL_X, 2.98),
        ("guide_ring_3", "guide_plate_3", "guide_bolt_3", RAIL_X, 2.98),
    ):
        _add_guide_ring(
            base,
            ring_name=ring_name,
            plate_name=plate_name,
            bolt_prefix=bolt_prefix,
            x=x,
            z=z,
            steel=dark_steel,
        )

    _add_base_foot_yoke(
        base,
        outer_name="foot_0_yoke_outer",
        inner_name="foot_0_yoke_inner",
        hanger_name="foot_0_hanger",
        x=-RAIL_X,
        steel=dark_steel,
    )
    _add_base_foot_yoke(
        base,
        outer_name="foot_1_yoke_outer",
        inner_name="foot_1_yoke_inner",
        hanger_name="foot_1_hanger",
        x=RAIL_X,
        steel=dark_steel,
    )

    fly = model.part("fly_section")
    fly_length = 3.40
    fly_bottom = -0.10
    fly_center_z = fly_bottom + fly_length * 0.5
    fly_stile_names = ("fly_stile_0", "fly_stile_1")
    fly_top_cap_names = ("fly_top_cap_0", "fly_top_cap_1")
    fly_bottom_cap_names = ("fly_bottom_cap_0", "fly_bottom_cap_1")
    for index, x in enumerate((-RAIL_X, RAIL_X)):
        fly.visual(
            Box((0.062, 0.055, fly_length)),
            origin=Origin(xyz=(x, 0.0, fly_center_z)),
            material=fly_fiberglass,
            name=fly_stile_names[index],
        )
        fly.visual(
            Box((0.052, 0.061, 0.052)),
            origin=Origin(xyz=(x, 0.0, fly_bottom + fly_length + 0.026)),
            material=dark_steel,
            name=fly_top_cap_names[index],
        )
        fly.visual(
            Box((0.052, 0.061, 0.052)),
            origin=Origin(xyz=(x, 0.0, fly_bottom - 0.026)),
            material=dark_steel,
            name=fly_bottom_cap_names[index],
        )

    for rung_index, z in enumerate([0.22, 0.52, 0.82, 1.12, 1.42, 1.72, 2.02, 2.32, 2.62, 2.92, 3.22]):
        _add_rung(
            fly,
            f"fly_rung_{rung_index}",
            z,
            y=0.018,
            length=0.442,
            radius=0.022,
            material=aluminium,
        )

    fly.visual(
        Box((0.050, 0.006, 0.220)),
        origin=Origin(xyz=(RAIL_X, 0.034, 1.10)),
        material=warning_red,
        name="fly_warning_label",
    )
    fly.visual(
        Box((0.046, 0.007, 0.080)),
        origin=Origin(xyz=(RAIL_X, 0.038, 1.15)),
        material=label_white,
        name="fly_label_patch",
    )

    foot_0 = model.part("foot_0")
    _build_foot(foot_0, rubber=rubber, steel=dark_steel)
    foot_1 = model.part("foot_1")
    _build_foot(foot_1, rubber=rubber, steel=dark_steel)

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, FLY_OFFSET_Y, FLY_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.35, lower=0.0, upper=0.90),
    )
    model.articulation(
        "base_to_foot_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=foot_0,
        origin=Origin(xyz=(-RAIL_X, 0.0, FOOT_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "base_to_foot_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=foot_1,
        origin=Origin(xyz=(RAIL_X, 0.0, FOOT_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    foot_0 = object_model.get_part("foot_0")
    foot_1 = object_model.get_part("foot_1")
    slide = object_model.get_articulation("base_to_fly")
    foot_joint = object_model.get_articulation("base_to_foot_0")

    # The fly stiles must stay captured in the fixed D guide brackets at both
    # collapsed and extended positions.
    ctx.expect_within(
        fly,
        base,
        axes="xy",
        inner_elem="fly_stile_0",
        outer_elem="guide_ring_0",
        margin=0.0,
        name="fly stile is centered in lower guide",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        elem_a="fly_stile_0",
        elem_b="guide_ring_2",
        min_overlap=0.008,
        name="fly stile passes through upper guide",
    )

    rest_pos = ctx.part_world_position(fly)
    with ctx.pose({slide: 0.90}):
        ctx.expect_within(
            fly,
            base,
            axes="xy",
            inner_elem="fly_stile_0",
            outer_elem="guide_ring_0",
            margin=0.0,
            name="extended fly stays centered in lower guide",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            elem_a="fly_stile_0",
            elem_b="guide_ring_2",
            min_overlap=0.008,
            name="extended fly remains captured by upper guide",
        )
        extended_pos = ctx.part_world_position(fly)
    ctx.check(
        "prismatic fly section extends upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.80,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    # The swivel foot tongues sit between the base-rail yoke plates without a
    # visual gap, and the revolute joint visibly tips the rubber pad.
    ctx.expect_gap(
        base,
        foot_0,
        axis="y",
        positive_elem="foot_0_yoke_outer",
        negative_elem="pivot_tongue",
        max_gap=0.001,
        max_penetration=0.0,
        name="foot 0 tongue bears on outer yoke plate",
    )
    ctx.expect_gap(
        base,
        foot_1,
        axis="y",
        positive_elem="foot_1_yoke_outer",
        negative_elem="pivot_tongue",
        max_gap=0.001,
        max_penetration=0.0,
        name="foot 1 tongue bears on outer yoke plate",
    )
    level_aabb = ctx.part_element_world_aabb(foot_0, elem="rubber_pad")
    with ctx.pose({foot_joint: 0.35}):
        tilted_aabb = ctx.part_element_world_aabb(foot_0, elem="rubber_pad")
    ctx.check(
        "rubber foot swivels on hinge",
        level_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][1] > level_aabb[1][1] + 0.020,
        details=f"level={level_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
