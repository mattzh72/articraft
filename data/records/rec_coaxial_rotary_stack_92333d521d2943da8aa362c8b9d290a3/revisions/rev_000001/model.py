from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _lathed_stage(
    outer_profile: list[tuple[float, float]],
    *,
    bore_radius: float = 0.043,
    segments: int = 96,
):
    """Build one hollow, stepped rotary drum centered on the local Z axis."""
    z_min = min(z for _, z in outer_profile)
    z_max = max(z for _, z in outer_profile)
    inner_profile = [(bore_radius, z_min), (bore_radius, z_max)]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _bolt_circle(
    part,
    *,
    count: int,
    radius: float,
    z: float,
    bolt_radius: float,
    height: float,
    material,
    prefix: str,
    phase: float = 0.0,
) -> None:
    for index in range(count):
        angle = phase + index * math.tau / count
        part.visual(
            Cylinder(radius=bolt_radius, length=height),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
            ),
            material=material,
            name=f"{prefix}_{index}",
        )


def _index_ticks(
    part,
    *,
    count: int,
    radius: float,
    z: float,
    material,
    prefix: str,
    major_every: int = 3,
) -> None:
    for index in range(count):
        angle = index * math.tau / count
        radial_len = 0.046 if index % major_every == 0 else 0.030
        part.visual(
            Box((radial_len, 0.006, 0.006)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
                rpy=(0.0, 0.0, angle),
            ),
            material=material,
            name=f"{prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drum_rotary_indexing_head")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.30, 0.32, 1.0))
    parkerized = model.material("parkerized", rgba=(0.18, 0.19, 0.21, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.62, 0.28, 1.0))
    red_mark = model.material("red_index_mark", rgba=(0.80, 0.04, 0.02, 1.0))

    core = model.part("core")
    core.visual(
        Cylinder(radius=0.035, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=dark_steel,
        name="vertical_shaft",
    )
    core.visual(
        Cylinder(radius=0.140, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=parkerized,
        name="lower_bearing",
    )
    core.visual(
        Cylinder(radius=0.135, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.1435)),
        material=parkerized,
        name="lower_spacer",
    )
    core.visual(
        Cylinder(radius=0.112, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.264)),
        material=parkerized,
        name="upper_spacer",
    )
    core.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.438)),
        material=parkerized,
        name="top_retainer",
    )

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            _lathed_stage(
                [
                    (0.360, -0.060),
                    (0.360, -0.035),
                    (0.310, -0.035),
                    (0.310, 0.030),
                    (0.250, 0.030),
                    (0.250, 0.055),
                    (0.180, 0.055),
                    (0.180, 0.060),
                ]
            ),
            "base_stepped_drum",
        ),
        material=gunmetal,
        name="base_drum",
    )
    _index_ticks(base, count=24, radius=0.292, z=0.033, material=brass, prefix="base_tick")
    _bolt_circle(
        base,
        count=8,
        radius=0.210,
        z=0.060,
        bolt_radius=0.011,
        height=0.010,
        material=dark_steel,
        prefix="base_bolt",
        phase=math.pi / 8.0,
    )
    base.visual(
        Box((0.060, 0.020, 0.014)),
        origin=Origin(xyz=(0.352, 0.0, -0.045)),
        material=red_mark,
        name="base_index_lug",
    )

    collar = model.part("collar")
    collar.visual(
        mesh_from_geometry(
            _lathed_stage(
                [
                    (0.240, -0.055),
                    (0.240, -0.038),
                    (0.205, -0.038),
                    (0.205, 0.035),
                    (0.225, 0.035),
                    (0.225, 0.055),
                ]
            ),
            "collar_stepped_ring",
        ),
        material=satin_steel,
        name="collar_ring",
    )
    _bolt_circle(
        collar,
        count=6,
        radius=0.160,
        z=0.059,
        bolt_radius=0.009,
        height=0.008,
        material=dark_steel,
        prefix="collar_bolt",
        phase=math.pi / 6.0,
    )
    collar.visual(
        Box((0.042, 0.014, 0.012)),
        origin=Origin(xyz=(0.214, 0.0, 0.010)),
        material=red_mark,
        name="collar_index_lug",
    )

    nose = model.part("nose")
    nose.visual(
        mesh_from_geometry(
            _lathed_stage(
                [
                    (0.145, -0.072),
                    (0.145, -0.052),
                    (0.115, -0.052),
                    (0.115, -0.012),
                    (0.096, 0.030),
                    (0.075, 0.060),
                    (0.075, 0.076),
                ]
            ),
            "nose_tapered_tooling",
        ),
        material=satin_steel,
        name="tooling_nose",
    )
    nose.visual(
        Box((0.033, 0.012, 0.054)),
        origin=Origin(xyz=(0.077, 0.0, 0.012)),
        material=red_mark,
        name="nose_drive_key",
    )
    nose.visual(
        mesh_from_geometry(
            _lathed_stage([(0.052, -0.005), (0.052, 0.005)]),
            "tool_socket_lip",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=dark_steel,
        name="tool_socket_lip",
    )

    one_turn = math.tau
    model.articulation(
        "core_to_base",
        ArticulationType.REVOLUTE,
        parent=core,
        child=base,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.2, lower=-one_turn / 2.0, upper=one_turn / 2.0),
    )
    model.articulation(
        "core_to_collar",
        ArticulationType.REVOLUTE,
        parent=core,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.207)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.5, lower=-one_turn / 2.0, upper=one_turn / 2.0),
    )
    model.articulation(
        "core_to_nose",
        ArticulationType.REVOLUTE,
        parent=core,
        child=nose,
        origin=Origin(xyz=(0.0, 0.0, 0.342)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.0, lower=-one_turn / 2.0, upper=one_turn / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    core = object_model.get_part("core")
    base = object_model.get_part("base")
    collar = object_model.get_part("collar")
    nose = object_model.get_part("nose")
    base_joint = object_model.get_articulation("core_to_base")
    collar_joint = object_model.get_articulation("core_to_collar")
    nose_joint = object_model.get_articulation("core_to_nose")

    vertical_revolutes = [base_joint, collar_joint, nose_joint]
    ctx.check(
        "three independent vertical revolute stages",
        all(j.articulation_type == ArticulationType.REVOLUTE and j.axis == (0.0, 0.0, 1.0) for j in vertical_revolutes),
        details=f"joints={[j.name for j in vertical_revolutes]}",
    )
    ctx.expect_origin_distance(base, core, axes="xy", max_dist=0.001, name="base centered on core")
    ctx.expect_origin_distance(collar, core, axes="xy", max_dist=0.001, name="collar centered on core")
    ctx.expect_origin_distance(nose, core, axes="xy", max_dist=0.001, name="nose centered on core")
    ctx.expect_gap(collar, base, axis="z", min_gap=0.008, max_gap=0.030, name="visible spacer gap above base")
    ctx.expect_gap(nose, collar, axis="z", min_gap=0.006, max_gap=0.030, positive_elem="tooling_nose", negative_elem="collar_ring", name="visible spacer gap above collar")
    ctx.expect_overlap(core, base, axes="z", elem_a="vertical_shaft", elem_b="base_drum", min_overlap=0.10, name="shaft passes through base stage")
    ctx.expect_overlap(core, collar, axes="z", elem_a="vertical_shaft", elem_b="collar_ring", min_overlap=0.09, name="shaft passes through collar stage")
    ctx.expect_overlap(core, nose, axes="z", elem_a="vertical_shaft", elem_b="tooling_nose", min_overlap=0.12, name="shaft passes through nose stage")

    rest_base_pos = ctx.part_world_position(base)
    with ctx.pose({base_joint: math.radians(45), collar_joint: math.radians(-30), nose_joint: math.radians(60)}):
        posed_base_pos = ctx.part_world_position(base)
        ctx.expect_gap(collar, base, axis="z", min_gap=0.008, max_gap=0.030, name="base and collar remain vertically separated when indexed")
        ctx.expect_gap(nose, collar, axis="z", min_gap=0.006, max_gap=0.030, positive_elem="tooling_nose", negative_elem="collar_ring", name="collar and nose remain vertically separated when indexed")
    ctx.check(
        "base rotates in place around shared shaft",
        rest_base_pos is not None and posed_base_pos is not None and abs(posed_base_pos[2] - rest_base_pos[2]) < 1e-6,
        details=f"rest={rest_base_pos}, posed={posed_base_pos}",
    )

    return ctx.report()


object_model = build_object_model()
