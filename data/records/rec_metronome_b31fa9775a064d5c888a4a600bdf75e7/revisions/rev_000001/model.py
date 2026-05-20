from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_metronome")

    wood = model.material("dark_walnut", rgba=(0.28, 0.13, 0.055, 1.0))
    endgrain = model.material("endgrain_walnut", rgba=(0.20, 0.085, 0.035, 1.0))
    brass = model.material("brushed_brass", rgba=(0.86, 0.64, 0.25, 1.0))
    black = model.material("black_enamel", rgba=(0.02, 0.018, 0.015, 1.0))
    steel = model.material("polished_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    ivory = model.material("aged_ivory", rgba=(0.90, 0.82, 0.64, 1.0))

    housing = model.part("housing")

    # A flat plinth with a box-style upright case.  The case is built from
    # connected wall/rail members so the central front and top stay visibly open.
    housing.visual(
        Box((0.25, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=wood,
        name="base_plinth",
    )
    housing.visual(
        Box((0.17, 0.012, 0.34)),
        origin=Origin(xyz=(0.0, 0.049, 0.195)),
        material=endgrain,
        name="rear_panel",
    )
    for x, name in ((-0.079, "side_wall_0"), (0.079, "side_wall_1")):
        housing.visual(
            Box((0.012, 0.110, 0.34)),
            origin=Origin(xyz=(x, 0.0, 0.195)),
            material=wood,
            name=name,
        )
    housing.visual(
        Box((0.17, 0.012, 0.04)),
        origin=Origin(xyz=(0.0, -0.061, 0.045)),
        material=wood,
        name="front_bottom_rail",
    )
    housing.visual(
        Box((0.17, 0.012, 0.035)),
        origin=Origin(xyz=(0.0, -0.061, 0.3475)),
        material=wood,
        name="front_top_rail",
    )
    for x, name in ((-0.0775, "front_jamb_0"), (0.0775, "front_jamb_1")):
        housing.visual(
            Box((0.015, 0.014, 0.300)),
            origin=Origin(xyz=(x, -0.062, 0.195)),
            material=wood,
            name=name,
        )

    # Brass tempo scale and raised tick marks behind the pendulum.
    housing.visual(
        Box((0.016, 0.002, 0.270)),
        origin=Origin(xyz=(0.034, -0.067, 0.195)),
        material=brass,
        name="tempo_scale",
    )
    for i, z in enumerate((0.095, 0.125, 0.155, 0.185, 0.215, 0.245, 0.275, 0.305)):
        housing.visual(
            Box((0.010 if i % 2 else 0.014, 0.0015, 0.0025)),
            origin=Origin(xyz=(0.034, -0.0685, z)),
            material=black,
            name=f"scale_tick_{i}",
        )

    # A small bearing plate indicates the upper internal pendulum shaft.
    housing.visual(
        Box((0.040, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, -0.068, 0.315)),
        material=brass,
        name="pivot_plate",
    )

    # Exposed piano-hinge style knuckles along the rear top edge.
    housing.visual(
        Box((0.17, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.059, 0.358)),
        material=brass,
        name="rear_hinge_leaf",
    )
    for x, name in ((-0.057, "rear_hinge_knuckle_0"), (0.057, "rear_hinge_knuckle_1")):
        housing.visual(
            Cylinder(radius=0.006, length=0.045),
            origin=Origin(xyz=(x, 0.061, 0.371), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=name,
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.18, 0.110, 0.012)),
        # The lid frame is the hinge axis; the slab extends forward from it.
        origin=Origin(xyz=(0.0, -0.061, 0.0)),
        material=wood,
        name="lid_panel",
    )
    lid.visual(
        Box((0.145, 0.085, 0.0025)),
        origin=Origin(xyz=(0.0, -0.061, 0.0069)),
        material=endgrain,
        name="lid_inlay",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lid_hinge_knuckle",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0025, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, -0.1225)),
        material=steel,
        name="rod",
    )
    pendulum.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_hub",
    )
    pendulum.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.250)),
        material=brass,
        name="lower_bob",
    )

    weight = model.part("sliding_weight")
    weight_sleeve = LatheGeometry.from_shell_profiles(
        [(0.016, -0.021), (0.016, 0.021)],
        [(0.0048, -0.021), (0.0048, 0.021)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    weight.visual(
        mesh_from_geometry(weight_sleeve, "weight_sleeve"),
        material=brass,
        name="weight_sleeve",
    )
    weight.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(0.0, -0.023, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="thumb_screw",
    )
    weight.visual(
        Sphere(radius=0.005),
        origin=Origin(xyz=(0.0, -0.032, 0.0)),
        material=black,
        name="screw_knob",
    )
    weight.visual(
        Box((0.003, 0.004, 0.006)),
        # Hidden pad at the end of the thumb screw: it touches the rod so the
        # sliding sleeve reads as clamped/retained rather than floating on air.
        origin=Origin(xyz=(0.0, -0.0045, 0.0)),
        material=black,
        name="clamp_pad",
    )

    key = model.part("winding_key")
    key.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=brass,
        name="key_stem",
    )
    key.visual(
        Box((0.052, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=brass,
        name="key_bar",
    )
    for x, name in ((-0.029, "key_wing_0"), (0.029, "key_wing_1")):
        key.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, 0.0, -0.025)),
            material=brass,
            name=name,
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lid,
        origin=Origin(xyz=(0.0, 0.061, 0.371)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.85),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )
    model.articulation(
        "pendulum_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, -0.083, 0.315)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0, lower=-0.28, upper=0.28),
        motion_properties=MotionProperties(damping=0.01, friction=0.002),
    )
    model.articulation(
        "weight_slide",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.15, lower=-0.055, upper=0.060),
        motion_properties=MotionProperties(damping=0.10, friction=0.20),
    )
    model.articulation(
        "key_turn",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=key,
        origin=Origin(xyz=(0.0, -0.055, -0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=5.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    lid = object_model.get_part("lid")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("sliding_weight")
    key = object_model.get_part("winding_key")

    lid_hinge = object_model.get_articulation("lid_hinge")
    pendulum_pivot = object_model.get_articulation("pendulum_pivot")
    weight_slide = object_model.get_articulation("weight_slide")
    key_turn = object_model.get_articulation("key_turn")

    ctx.check(
        "four requested user mechanisms are articulated",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and pendulum_pivot.articulation_type == ArticulationType.REVOLUTE
        and weight_slide.articulation_type == ArticulationType.PRISMATIC
        and key_turn.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"types: lid={lid_hinge.articulation_type}, pendulum={pendulum_pivot.articulation_type}, "
            f"weight={weight_slide.articulation_type}, key={key_turn.articulation_type}"
        ),
    )

    ctx.expect_gap(
        lid,
        housing,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_top_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid sits on the top rail",
    )
    ctx.expect_overlap(
        lid,
        housing,
        axes="x",
        elem_a="lid_panel",
        elem_b="front_top_rail",
        min_overlap=0.15,
        name="closed lid spans the housing width",
    )
    ctx.expect_contact(
        key,
        lid,
        elem_a="key_stem",
        elem_b="lid_panel",
        contact_tol=0.001,
        name="winding key stem is seated into the lid face",
    )
    ctx.expect_within(
        key,
        lid,
        axes="xy",
        inner_elem="key_bar",
        outer_elem="lid_panel",
        margin=0.0,
        name="winding key is concealed within the lid footprint",
    )

    def aabb_center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.85}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid opens rearward and upward about the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[0][1] > 0.055
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.045,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_rod_aabb = ctx.part_element_world_aabb(pendulum, elem="rod")
    with ctx.pose({pendulum_pivot: 0.28}):
        swung_rod_aabb = ctx.part_element_world_aabb(pendulum, elem="rod")
    rest_rod_x = aabb_center(rest_rod_aabb, 0)
    swung_rod_x = aabb_center(swung_rod_aabb, 0)
    ctx.check(
        "pendulum rod swings side-to-side from upper shaft",
        rest_rod_x is not None and swung_rod_x is not None and abs(swung_rod_x - rest_rod_x) > 0.025,
        details=f"rest_x={rest_rod_x}, swung_x={swung_rod_x}",
    )

    ctx.expect_overlap(
        weight,
        pendulum,
        axes="z",
        elem_a="weight_sleeve",
        elem_b="rod",
        min_overlap=0.035,
        name="sliding weight stays on the pendulum rod",
    )
    ctx.expect_contact(
        weight,
        pendulum,
        elem_a="clamp_pad",
        elem_b="rod",
        contact_tol=0.001,
        name="sliding weight clamp bears on the rod",
    )
    rest_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({weight_slide: 0.060}):
        high_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({weight_slide: -0.055}):
        low_weight_pos = ctx.part_world_position(weight)
    ctx.check(
        "cylindrical weight slides along the pendulum rod",
        rest_weight_pos is not None
        and high_weight_pos is not None
        and low_weight_pos is not None
        and high_weight_pos[2] > rest_weight_pos[2] + 0.050
        and low_weight_pos[2] < rest_weight_pos[2] - 0.045,
        details=f"low={low_weight_pos}, rest={rest_weight_pos}, high={high_weight_pos}",
    )

    return ctx.report()


object_model = build_object_model()
