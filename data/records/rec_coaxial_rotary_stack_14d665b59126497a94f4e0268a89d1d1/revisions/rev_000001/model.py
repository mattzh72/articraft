from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 96, *, clockwise: bool = False):
    angles = [2.0 * pi * i / segments for i in range(segments)]
    if clockwise:
        angles = list(reversed(angles))
    return [(radius * cos(a), radius * sin(a)) for a in angles]


def _annular_mesh(name: str, outer_radius: float, inner_radius: float, height: float):
    """Centered annular puck with a real through-bore for the fixed shaft."""
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius),
            [_circle_profile(inner_radius)],
            height,
            cap=True,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_rotary_stack")

    brushed_steel = Material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    turntable_blue = Material("satin_blue_turntable", rgba=(0.12, 0.30, 0.70, 1.0))
    ring_orange = Material("anodized_orange_ring", rgba=(0.95, 0.42, 0.10, 1.0))
    flange_green = Material("green_top_flange", rgba=(0.10, 0.58, 0.36, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    white_index = Material("white_index_marks", rgba=(0.92, 0.90, 0.82, 1.0))

    shaft = model.part("center_shaft")
    shaft.visual(
        Cylinder(radius=0.24, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="ground_plinth",
    )
    shaft.visual(
        Cylinder(radius=0.030, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=brushed_steel,
        name="fixed_shaft",
    )
    # Stationary collars fill the axial gaps and make each rotating member read
    # as being carried by the same grounded post rather than floating in space.
    shaft.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=brushed_steel,
        name="lower_bearing_washer",
    )
    shaft.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=brushed_steel,
        name="middle_spacer",
    )
    shaft.visual(
        Cylinder(radius=0.050, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.194)),
        material=brushed_steel,
        name="top_spacer",
    )
    shaft.visual(
        Cylinder(radius=0.064, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.294)),
        material=brushed_steel,
        name="retaining_nut",
    )

    base = model.part("base_turntable")
    base.visual(
        _annular_mesh("base_turntable_plate", 0.255, 0.046, 0.045),
        material=turntable_blue,
        name="turntable_plate",
    )
    base.visual(
        _annular_mesh("base_turntable_outer_rim", 0.278, 0.238, 0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=turntable_blue,
        name="raised_outer_rim",
    )
    base.visual(
        _annular_mesh("base_turntable_bearing_boss", 0.078, 0.046, 0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=turntable_blue,
        name="central_bearing_boss",
    )
    base.visual(
        Box((0.080, 0.035, 0.018)),
        origin=Origin(xyz=(0.285, 0.0, 0.031)),
        material=white_index,
        name="radial_index_tab",
    )

    middle = model.part("middle_ring")
    middle.visual(
        _annular_mesh("middle_outer_ring", 0.205, 0.142, 0.050),
        material=ring_orange,
        name="outer_ring",
    )
    middle.visual(
        _annular_mesh("middle_hub", 0.080, 0.046, 0.054),
        material=ring_orange,
        name="shaft_hub",
    )
    for i, yaw in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        middle.visual(
            Box((0.078, 0.020, 0.030)),
            origin=Origin(xyz=(0.111, 0.0, 0.0), rpy=(0.0, 0.0, yaw)),
            material=ring_orange,
            name=f"spoke_{i}",
        )
    middle.visual(
        Box((0.045, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.206, 0.012)),
        material=white_index,
        name="outer_index_block",
    )

    top = model.part("top_flange")
    top.visual(
        _annular_mesh("top_flange_disk", 0.148, 0.046, 0.055),
        material=flange_green,
        name="flange_disk",
    )
    top.visual(
        _annular_mesh("top_flange_neck", 0.075, 0.046, 0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=flange_green,
        name="raised_neck",
    )
    top.visual(
        Box((0.070, 0.028, 0.018)),
        origin=Origin(xyz=(-0.160, 0.0, 0.033)),
        material=white_index,
        name="flange_pointer",
    )
    for i, yaw in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        top.visual(
            Cylinder(radius=0.012, length=0.011),
            origin=Origin(
                xyz=(0.103 * cos(yaw), 0.103 * sin(yaw), 0.032),
            ),
            material=black_rubber,
            name=f"bolt_head_{i}",
        )

    # All rotating stages are independently journaled on the same vertical shaft
    # centerline; the different Z origins put their joint frames at the centers
    # of their respective stacked members.
    model.articulation(
        "shaft_to_turntable",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=base,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-pi, upper=pi),
    )
    model.articulation(
        "shaft_to_middle_ring",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "shaft_to_top_flange",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=-pi, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("center_shaft")
    base = object_model.get_part("base_turntable")
    middle = object_model.get_part("middle_ring")
    top = object_model.get_part("top_flange")
    base_joint = object_model.get_articulation("shaft_to_turntable")
    middle_joint = object_model.get_articulation("shaft_to_middle_ring")
    top_joint = object_model.get_articulation("shaft_to_top_flange")

    # The annular stages are visible mesh rings with real through-bores, but the
    # mesh collision proxy is conservative at the center.  Scope overlap
    # allowances to the fixed post passing through the named ring/boss elements.
    for stage, elem_names in (
        (base, ("turntable_plate", "raised_outer_rim", "central_bearing_boss")),
        (middle, ("outer_ring", "shaft_hub")),
        (top, ("flange_disk", "raised_neck")),
    ):
        for elem_name in elem_names:
            if stage.name < shaft.name:
                ctx.allow_overlap(
                    stage,
                    shaft,
                    elem_a=elem_name,
                    elem_b="fixed_shaft",
                    reason=(
                        "The fixed center shaft intentionally passes through the "
                        "clearanced bore of this annular rotating stage."
                    ),
                )
            else:
                ctx.allow_overlap(
                    shaft,
                    stage,
                    elem_a="fixed_shaft",
                    elem_b=elem_name,
                    reason=(
                        "The fixed center shaft intentionally passes through the "
                        "clearanced bore of this annular rotating stage."
                    ),
                )
            ctx.expect_overlap(
                stage,
                shaft,
                axes="z",
                min_overlap=0.020,
                elem_a=elem_name,
                elem_b="fixed_shaft",
                name=f"{stage.name} {elem_name} is journaled on the fixed shaft",
            )

    joints = (base_joint, middle_joint, top_joint)
    ctx.check(
        "three independent rotary stages",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "shared vertical rotation axis",
        all(j.axis == (0.0, 0.0, 1.0) for j in joints)
        and all(abs(j.origin.xyz[0]) < 1e-9 and abs(j.origin.xyz[1]) < 1e-9 for j in joints),
        details=f"axes={[j.axis for j in joints]}, origins={[j.origin.xyz for j in joints]}",
    )

    for stage in (base, middle, top):
        ctx.expect_origin_distance(
            stage,
            shaft,
            axes="xy",
            max_dist=0.001,
            name=f"{stage.name} stays coaxial with shaft",
        )

    ctx.expect_gap(
        middle,
        base,
        axis="z",
        min_gap=0.006,
        max_gap=0.040,
        name="middle ring is visibly stacked above base turntable",
    )
    ctx.expect_gap(
        top,
        middle,
        axis="z",
        min_gap=0.006,
        max_gap=0.045,
        name="top flange is visibly stacked above middle ring",
    )

    rest_positions = {
        stage.name: ctx.part_world_position(stage) for stage in (base, middle, top)
    }
    with ctx.pose({base_joint: 0.8, middle_joint: -1.1, top_joint: 1.4}):
        posed_positions = {
            stage.name: ctx.part_world_position(stage) for stage in (base, middle, top)
        }
    ctx.check(
        "rotary poses keep all stages on the common centerline",
        rest_positions == posed_positions,
        details=f"rest={rest_positions}, posed={posed_positions}",
    )

    return ctx.report()


object_model = build_object_model()
