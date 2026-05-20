from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)
import cadquery as cq


def _rounded_box_mesh(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(width, depth, radius, corner_segments=10), height),
        name,
    )


def _hopper_shell_mesh(name: str):
    # Thin transparent reservoir with a visible lip, rounded shoulder, and open-looking
    # neck.  The wall thickness is exaggerated slightly so it reads at vendor scale.
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.110, 0.040),
                (0.123, 0.085),
                (0.125, 0.260),
                (0.118, 0.405),
                (0.110, 0.440),
            ],
            [
                (0.094, 0.056),
                (0.104, 0.095),
                (0.106, 0.255),
                (0.101, 0.398),
                (0.095, 0.424),
            ],
            segments=64,
            start_cap="round",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def _candy_fill_mesh(name: str):
    # A single lumpy lathed volume avoids loose floating candy balls while still
    # showing a colored fill through the clear reservoir.
    return mesh_from_geometry(
        LatheGeometry(
            [
                (0.000, 0.050),
                (0.092, 0.050),
                (0.108, 0.075),
                (0.106, 0.155),
                (0.098, 0.245),
                (0.072, 0.285),
                (0.000, 0.292),
            ],
            segments=48,
        ),
        name,
    )


def _selector_knob_mesh(name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.086,
            0.046,
            body_style="faceted",
            base_diameter=0.090,
            top_diameter=0.070,
            edge_radius=0.002,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0022, width=0.003),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0, depth=0.001),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_hopper_pedestal_candy_vendor")

    red_enamel = model.material("red_enamel", rgba=(0.68, 0.035, 0.025, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.07, 0.075, 0.08, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.014, 0.012, 1.0))
    clear_acrylic = model.material("clear_acrylic", rgba=(0.80, 0.94, 1.00, 0.34))
    amber_candy = model.material("amber_candy", rgba=(1.00, 0.47, 0.10, 0.86))
    green_candy = model.material("green_candy", rgba=(0.18, 0.78, 0.30, 0.86))
    label_white = model.material("label_white", rgba=(0.96, 0.92, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.215, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=chrome,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.680),
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        material=chrome,
        name="pedestal_pole",
    )
    base.visual(
        _rounded_box_mesh(0.700, 0.360, 0.240, 0.040, "shared_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        material=red_enamel,
        name="shared_body",
    )
    base.visual(
        _rounded_box_mesh(0.720, 0.380, 0.040, 0.040, "top_deck"),
        origin=Origin(xyz=(0.0, 0.0, 0.960)),
        material=chrome,
        name="top_deck",
    )

    for index, x_pos in enumerate((-0.180, 0.180)):
        base.visual(
            Box((0.130, 0.055, 0.060)),
            origin=Origin(xyz=(x_pos, -0.190, 0.900)),
            material=chrome,
            name=f"outlet_{index}",
        )
        base.visual(
            Cylinder(radius=0.018, length=0.060),
            origin=Origin(xyz=(x_pos, -0.230, 0.897), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"spout_{index}",
        )
        base.visual(
            Cylinder(radius=0.046, length=0.026),
            origin=Origin(xyz=(x_pos, -0.192, 0.835), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"bearing_{index}",
        )
        base.visual(
            Box((0.112, 0.006, 0.030)),
            origin=Origin(xyz=(x_pos, -0.214, 0.776)),
            material=label_white,
            name=f"price_label_{index}",
        )

    base.visual(
        Box((0.280, 0.026, 0.155)),
        origin=Origin(xyz=(0.0, -0.197, 0.772)),
        material=black_plastic,
        name="pickup_recess",
    )
    base.visual(
        Box((0.250, 0.044, 0.026)),
        origin=Origin(xyz=(0.0, -0.201, 0.868)),
        material=chrome,
        name="pickup_top_trim",
    )
    for x_pos, name in ((-0.120, "pickup_side_0"), (0.120, "pickup_side_1")):
        base.visual(
            Box((0.014, 0.044, 0.154)),
            origin=Origin(xyz=(x_pos, -0.201, 0.781)),
            material=chrome,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.010, length=0.320),
        origin=Origin(xyz=(0.0, -0.224, 0.696), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="door_hinge_pin",
    )

    hoppers = []
    for index, x_pos in enumerate((-0.180, 0.180)):
        hopper = model.part(f"hopper_{index}")
        hopper.visual(
            Cylinder(radius=0.132, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, 0.020)),
            material=chrome,
            name="base_ring",
        )
        hopper.visual(
            _hopper_shell_mesh(f"reservoir_shell_{index}"),
            material=clear_acrylic,
            name="reservoir_shell",
        )
        hopper.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.113, tube=0.008, radial_segments=12, tubular_segments=64),
                f"top_rim_{index}",
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.440)),
            material=chrome,
            name="top_rim",
        )
        hopper.visual(
            _candy_fill_mesh(f"candy_fill_{index}"),
            material=amber_candy if index == 0 else green_candy,
            name="candy_fill",
        )
        hopper.visual(
            Box((0.118, 0.016, 0.018)),
            origin=Origin(xyz=(0.0, 0.145, 0.429)),
            material=chrome,
            name="rear_hinge_leaf",
        )
        hopper.visual(
            Box((0.072, 0.040, 0.010)),
            origin=Origin(xyz=(0.0, 0.128, 0.429)),
            material=chrome,
            name="rear_hinge_bridge",
        )
        model.articulation(
            f"base_to_hopper_{index}",
            ArticulationType.FIXED,
            parent=base,
            child=hopper,
            origin=Origin(xyz=(x_pos, 0.0, 0.980)),
        )
        hoppers.append(hopper)

        lid = model.part(f"lid_{index}")
        lid.visual(
            Cylinder(radius=0.138, length=0.020),
            origin=Origin(xyz=(0.0, -0.138, 0.018)),
            material=chrome,
            name="cap",
        )
        lid.visual(
            Cylinder(radius=0.007, length=0.124),
            origin=Origin(xyz=(0.0, 0.0, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name="hinge_barrel",
        )
        lid.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.0, -0.138, 0.034)),
            material=dark_metal,
            name="lift_button",
        )
        model.articulation(
            f"hopper_{index}_to_lid",
            ArticulationType.REVOLUTE,
            parent=hopper,
            child=lid,
            origin=Origin(xyz=(0.0, 0.138, 0.440)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=1.5, lower=0.0, upper=1.25),
        )

        knob = model.part(f"knob_{index}")
        knob.visual(
            _selector_knob_mesh(f"selector_knob_{index}"),
            origin=Origin(xyz=(0.0, -0.036, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="grip",
        )
        knob.visual(
            Cylinder(radius=0.012, length=0.052),
            origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="shaft",
        )
        model.articulation(
            f"base_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=knob,
            origin=Origin(xyz=(x_pos, -0.205, 0.835)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=6.0),
        )

    door = model.part("cup_door")
    door.visual(
        Box((0.235, 0.014, 0.142)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=clear_acrylic,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="lower_barrel",
    )
    door.visual(
        Box((0.052, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.008, 0.127)),
        material=dark_metal,
        name="finger_pull",
    )
    model.articulation(
        "base_to_cup_door",
        ArticulationType.REVOLUTE,
        parent=base,
        child=door,
        origin=Origin(xyz=(0.0, -0.224, 0.696)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    door = object_model.get_part("cup_door")
    door_joint = object_model.get_articulation("base_to_cup_door")

    ctx.check(
        "two independent clear hoppers",
        object_model.get_part("hopper_0") is not None and object_model.get_part("hopper_1") is not None,
        details="Expected hopper_0 and hopper_1 as separate supported reservoirs.",
    )

    ctx.allow_overlap(
        base,
        door,
        elem_a="door_hinge_pin",
        elem_b="lower_barrel",
        reason="The pickup cup door barrel is intentionally coaxial around the fixed lower hinge pin.",
    )
    ctx.expect_within(
        door,
        base,
        axes="xz",
        inner_elem="lower_barrel",
        outer_elem="door_hinge_pin",
        margin=0.002,
        name="cup door barrel is captured on the hinge pin",
    )
    ctx.expect_overlap(
        door,
        base,
        axes="x",
        elem_a="lower_barrel",
        elem_b="door_hinge_pin",
        min_overlap=0.22,
        name="cup door hinge barrels share a long pin",
    )

    for index in (0, 1):
        hopper = object_model.get_part(f"hopper_{index}")
        lid = object_model.get_part(f"lid_{index}")
        knob = object_model.get_part(f"knob_{index}")
        lid_joint = object_model.get_articulation(f"hopper_{index}_to_lid")
        knob_joint = object_model.get_articulation(f"base_to_knob_{index}")

        ctx.check(
            f"selector knob {index} rotates continuously",
            getattr(knob_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
            details=f"{knob_joint.name} should be a continuous horizontal rotary selector.",
        )
        ctx.check(
            f"refill lid {index} is rear hinged",
            getattr(lid_joint, "articulation_type", None) == ArticulationType.REVOLUTE
            and tuple(lid_joint.axis) == (-1.0, 0.0, 0.0),
            details=f"{lid_joint.name} should rotate about a rear horizontal hinge axis.",
        )

        ctx.allow_overlap(
            base,
            knob,
            elem_a=f"bearing_{index}",
            elem_b="shaft",
            reason="The selector shaft is intentionally captured inside the front bearing collar.",
        )
        ctx.expect_within(
            knob,
            base,
            axes="xz",
            inner_elem="shaft",
            outer_elem=f"bearing_{index}",
            margin=0.002,
            name=f"selector shaft {index} is centered in its bearing",
        )
        ctx.expect_overlap(
            knob,
            base,
            axes="y",
            elem_a="shaft",
            elem_b=f"bearing_{index}",
            min_overlap=0.018,
            name=f"selector shaft {index} remains inserted in bearing",
        )
        ctx.expect_gap(
            hopper,
            base,
            axis="z",
            positive_elem="base_ring",
            negative_elem="top_deck",
            min_gap=0.0,
            max_gap=0.002,
            name=f"hopper {index} base ring sits on shared top deck",
        )
        ctx.expect_gap(
            lid,
            hopper,
            axis="z",
            positive_elem="cap",
            negative_elem="top_rim",
            min_gap=0.0,
            max_gap=0.002,
            name=f"lid {index} closes onto the hopper rim",
        )

        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="cap")
        with ctx.pose({lid_joint: 1.0}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="cap")
        ctx.check(
            f"lid {index} opens upward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.035,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    ctx.expect_gap(
        object_model.get_part("hopper_1"),
        object_model.get_part("hopper_0"),
        axis="x",
        min_gap=0.070,
        name="hoppers stay separated instead of merging into one bowl",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 0.85}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "pickup cup door swings outward on lower hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.045,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
