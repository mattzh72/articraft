from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TAU = 2.0 * math.pi


def _cylinder_between_z(radius: float, z_min: float, z_max: float) -> cq.Workplane:
    """CadQuery cylinder spanning an explicit world/local z interval."""
    return cq.Workplane("XY").circle(radius).extrude(z_max - z_min).translate((0.0, 0.0, z_min))


def _tower_body() -> cq.Workplane:
    """Single grounded base, center post, and fixed spacer stack."""
    base_h = 0.045
    post_r = 0.038
    post_top = 0.645

    body = _cylinder_between_z(0.19, 0.0, base_h)
    body = body.union(_cylinder_between_z(post_r, base_h - 0.006, post_top))

    # Thin fixed separator sleeves visibly keep the independent rotary collars
    # stacked on the same center post without touching each other.
    for z_center in (0.105, 0.215, 0.325, 0.435, 0.545):
        sleeve_h = 0.030
        body = body.union(_cylinder_between_z(0.052, z_center - sleeve_h / 2.0, z_center + sleeve_h / 2.0))

    body = body.union(_cylinder_between_z(0.060, post_top - 0.020, post_top + 0.012))
    return body


def _carrier_body(height: float = 0.060) -> cq.Workplane:
    """Annular bearing carrier with a radial fixture arm and through holes."""
    outer_r = 0.096
    inner_r = 0.064
    arm_len = 0.285
    arm_w = 0.064
    arm_overlap = 0.014
    pad_r = 0.048
    pad_x = outer_r + arm_len - arm_overlap

    ring = cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(height).translate((0.0, 0.0, -height / 2.0))
    arm = cq.Workplane("XY").box(arm_len, arm_w, height).translate(
        (outer_r + arm_len / 2.0 - arm_overlap, 0.0, 0.0)
    )
    pad = cq.Workplane("XY").cylinder(height, pad_r).translate((pad_x, 0.0, 0.0))

    carrier = ring.union(arm).union(pad)

    # A large tooling hole in the outer pad and a smaller indexing hole on the
    # arm make the carrier read as a real workholding/rotary fixture plate.
    tooling_hole = cq.Workplane("XY").cylinder(height + 0.020, 0.018).translate((pad_x, 0.0, 0.0))
    index_hole = cq.Workplane("XY").cylinder(height + 0.020, 0.010).translate((outer_r + 0.090, 0.0, 0.0))
    carrier = carrier.cut(tooling_hole).cut(index_hole)
    return carrier


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_mounted_coaxial_rotary_fixture")

    anodized = Material("dark_anodized_aluminum", rgba=(0.12, 0.13, 0.14, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    bolt_black = Material("black_oxide_fasteners", rgba=(0.025, 0.025, 0.025, 1.0))
    carrier_materials = (
        Material("carrier_blue", rgba=(0.10, 0.25, 0.74, 1.0)),
        Material("carrier_gold", rgba=(0.82, 0.55, 0.13, 1.0)),
        Material("carrier_green", rgba=(0.12, 0.55, 0.32, 1.0)),
        Material("carrier_red", rgba=(0.70, 0.18, 0.16, 1.0)),
    )

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(_tower_body(), "tower_body", tolerance=0.0008, angular_tolerance=0.08),
        material=satin_steel,
        name="base_post_stack",
    )
    for index, (x, y) in enumerate(((0.135, 0.135), (-0.135, 0.135), (-0.135, -0.135), (0.135, -0.135))):
        tower.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(xyz=(x, y, 0.045)),
            material=bolt_black,
            name=f"anchor_bolt_{index}",
        )

    carrier_z = (0.160, 0.270, 0.380, 0.490)
    carrier_h = 0.060
    washer_h = 0.018
    for index, z in enumerate(carrier_z):
        # Fixed thrust washers touch the underside of each rotating carrier and
        # overlap the central post, creating a continuous support path to ground.
        carrier_bottom = z - carrier_h / 2.0
        tower.visual(
            Cylinder(radius=0.090, length=washer_h),
            origin=Origin(xyz=(0.0, 0.0, carrier_bottom - washer_h / 2.0)),
            material=anodized,
            name=f"thrust_washer_{index}",
        )

    zero_offsets = (0.0, math.radians(38.0), math.radians(82.0), math.radians(126.0))
    carrier_mesh = _carrier_body()
    for index, (z, offset) in enumerate(zip(carrier_z, zero_offsets)):
        carrier = model.part(f"carrier_{index}")
        carrier.visual(
            mesh_from_cadquery(
                carrier_mesh,
                f"carrier_{index}_body",
                tolerance=0.0008,
                angular_tolerance=0.08,
            ),
            origin=Origin(rpy=(0.0, 0.0, offset)),
            material=carrier_materials[index],
            name="annular_fixture_arm",
        )
        # The parent tower carries each collar independently on the same
        # vertical centerline; no carrier is chained through another carrier.
        model.articulation(
            f"tower_to_carrier_{index}",
            ArticulationType.REVOLUTE,
            parent=tower,
            child=carrier,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=9.0, velocity=4.0, lower=-math.pi, upper=math.pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    carriers = [object_model.get_part(f"carrier_{index}") for index in range(4)]
    joints = [object_model.get_articulation(f"tower_to_carrier_{index}") for index in range(4)]

    ctx.check(
        "all carriers are coaxial revolutes",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.parent == "tower"
            and joint.axis == (0.0, 0.0, 1.0)
            and abs(joint.origin.xyz[0]) < 1e-9
            and abs(joint.origin.xyz[1]) < 1e-9
            for joint in joints
        ),
        details=[(joint.name, joint.articulation_type, joint.parent, joint.axis, joint.origin.xyz) for joint in joints],
    )

    heights = [joint.origin.xyz[2] for joint in joints]
    ctx.check(
        "carrier joints are stacked upward",
        all(heights[index + 1] - heights[index] > 0.095 for index in range(len(heights) - 1)),
        details=f"joint_heights={heights}",
    )

    for index, carrier in enumerate(carriers):
        ctx.expect_overlap(
            carrier,
            tower,
            axes="z",
            min_overlap=0.035,
            elem_a="annular_fixture_arm",
            elem_b="base_post_stack",
            name=f"carrier_{index} surrounds post height",
        )

    for lower, upper in zip(carriers, carriers[1:]):
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.035,
            max_gap=0.060,
            name=f"{upper.name} clears {lower.name}",
        )

    with ctx.pose({joints[0]: math.pi / 2.0, joints[2]: -math.pi / 2.0}):
        moved_heights = [ctx.part_world_position(carrier)[2] for carrier in carriers]
        ctx.check(
            "independent rotations keep carriers on centerline",
            all(abs(ctx.part_world_position(carrier)[0]) < 1e-9 and abs(ctx.part_world_position(carrier)[1]) < 1e-9 for carrier in carriers)
            and all(abs(moved_heights[i] - heights[i]) < 1e-9 for i in range(len(heights))),
            details=f"moved_heights={moved_heights}",
        )

    return ctx.report()


object_model = build_object_model()
