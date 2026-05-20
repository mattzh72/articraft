from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _merge(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 64,
) -> MeshGeometry:
    """Thin-walled vertical sleeve/ring with an actual open bore."""
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _tripod_leg_mesh() -> MeshGeometry:
    tube = wire_from_points(
        [(0.018, 0.0, 0.0), (0.22, 0.0, -0.028), (0.62, 0.0, -0.095)],
        radius=0.014,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.10,
    )
    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_pole_lamp")

    satin_black = _mat(model, "satin_black", (0.025, 0.027, 0.030, 1.0))
    dark_graphite = _mat(model, "dark_graphite", (0.11, 0.115, 0.12, 1.0))
    brushed_steel = _mat(model, "brushed_steel", (0.60, 0.62, 0.64, 1.0))
    warm_lens = _mat(model, "warm_lens", (1.0, 0.86, 0.55, 0.92))
    rubber = _mat(model, "matte_rubber", (0.015, 0.014, 0.013, 1.0))
    knob_plastic = _mat(model, "knob_plastic", (0.035, 0.035, 0.038, 1.0))

    sleeve = model.part("sleeve")

    sleeve_shell = _merge(
        [
            _ring_band(outer_radius=0.034, inner_radius=0.0235, z_min=0.10, z_max=1.015),
            _ring_band(outer_radius=0.054, inner_radius=0.0180, z_min=0.970, z_max=1.080),
            _ring_band(outer_radius=0.045, inner_radius=0.0240, z_min=0.932, z_max=0.958),
            CylinderGeometry(radius=0.086, height=0.075, radial_segments=64).translate(
                0.0, 0.0, 0.055
            ),
            CylinderGeometry(radius=0.070, height=0.020, radial_segments=64).translate(
                0.0, 0.0, 0.102
            ),
        ]
    )
    sleeve.visual(
        mesh_from_geometry(sleeve_shell, "sleeve_shell"),
        material=satin_black,
        name="sleeve_shell",
    )

    # Three paired hinge ears around the lower hub for the adjustable tripod legs.
    for index in range(3):
        yaw = index * math.tau / 3.0
        for side, y_offset in enumerate((-0.031, 0.031)):
            sleeve.visual(
                Box((0.080, 0.012, 0.050)),
                origin=Origin(
                    xyz=(0.104 * math.cos(yaw) - y_offset * math.sin(yaw),
                         0.104 * math.sin(yaw) + y_offset * math.cos(yaw),
                         0.120),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=dark_graphite,
                name=f"hinge_ear_{index}_{side}",
            )

    leg_mesh = mesh_from_geometry(_tripod_leg_mesh(), "tripod_leg_tube")
    for index in range(3):
        leg = model.part(f"leg_{index}")
        leg.visual(
            leg_mesh,
            material=dark_graphite,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.018, length=0.050),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name="hinge_barrel",
        )
        leg.visual(
            Cylinder(radius=0.072, length=0.026),
            origin=Origin(xyz=(0.635, 0.0, -0.108)),
            material=rubber,
            name="foot_pad",
        )

        yaw = index * math.tau / 3.0
        model.articulation(
            f"leg_{index}_spread",
            ArticulationType.REVOLUTE,
            parent=sleeve,
            child=leg,
            origin=Origin(
                xyz=(0.104 * math.cos(yaw), 0.104 * math.sin(yaw), 0.120),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.42, upper=0.36),
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.018, length=1.380),
        # The mast extends below its sliding frame so it stays captured in the sleeve.
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=brushed_steel,
        name="inner_pole",
    )
    mast.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.914)),
        material=dark_graphite,
        name="top_plug",
    )
    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 1.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.22, lower=0.0, upper=0.35),
    )

    thumbscrew = model.part("thumbscrew")
    thumbscrew.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="threaded_stem",
    )
    thumbscrew.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.055,
                0.020,
                body_style="lobed",
                base_diameter=0.036,
                top_diameter=0.050,
                crown_radius=0.0015,
            ),
            "thumbscrew_knob",
        ),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_plastic,
        name="lobed_knob",
    )
    model.articulation(
        "thumbscrew_turn",
        ArticulationType.CONTINUOUS,
        parent=sleeve,
        child=thumbscrew,
        origin=Origin(xyz=(0.054, 0.0, 1.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )

    arm = model.part("arm")
    arm_collar = _merge(
        [
            _ring_band(outer_radius=0.048, inner_radius=0.0180, z_min=-0.035, z_max=0.035),
            CylinderGeometry(radius=0.016, height=0.060, radial_segments=24)
            .rotate((1.0, 0.0, 0.0), math.pi / 2.0)
            .translate(0.000, -0.063, 0.000),
        ]
    )
    arm.visual(
        mesh_from_geometry(arm_collar, "arm_collar"),
        material=satin_black,
        name="pivot_collar",
    )
    arm.visual(
        Box((0.065, 0.040, 0.040)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=satin_black,
        name="collar_socket",
    )
    arm.visual(
        Box((0.050, 0.038, 0.045)),
        origin=Origin(xyz=(0.012, -0.054, 0.0)),
        material=dark_graphite,
        name="clamp_ears",
    )
    arm.visual(
        Cylinder(radius=0.018, length=0.645),
        origin=Origin(xyz=(0.365, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="extension_tube",
    )
    arm.visual(
        Box((0.034, 0.284, 0.050)),
        origin=Origin(xyz=(0.670, 0.0, 0.0)),
        material=satin_black,
        name="yoke_backplate",
    )
    arm.visual(
        Box((0.085, 0.014, 0.270)),
        origin=Origin(xyz=(0.705, 0.132, 0.015)),
        material=satin_black,
        name="yoke_cheek_0",
    )
    arm.visual(
        Box((0.085, 0.014, 0.270)),
        origin=Origin(xyz=(0.705, -0.132, 0.015)),
        material=satin_black,
        name="yoke_cheek_1",
    )
    arm.visual(
        Box((0.075, 0.280, 0.026)),
        origin=Origin(xyz=(0.700, 0.0, 0.145)),
        material=satin_black,
        name="yoke_bridge",
    )
    arm.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.705, 0.143, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pivot_cap_0",
    )
    arm.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.705, -0.143, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pivot_cap_1",
    )
    model.articulation(
        "arm_swing",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.590)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-1.55, upper=1.55),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.108, length=0.086),
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="head_body",
    )
    head.visual(
        Cylinder(radius=0.118, length=0.018),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="rear_bezel",
    )
    head.visual(
        Cylinder(radius=0.094, length=0.006),
        origin=Origin(xyz=(0.103, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_lens,
        name="warm_lens",
    )
    head.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.0, 0.116, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="trunnion_0",
    )
    head.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.0, -0.116, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="trunnion_1",
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.705, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.3, lower=-0.55, upper=0.78),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    sleeve = object_model.get_part("sleeve")
    mast = object_model.get_part("mast")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")
    leg_0 = object_model.get_part("leg_0")
    mast_slide = object_model.get_articulation("mast_slide")
    arm_swing = object_model.get_articulation("arm_swing")
    head_tilt = object_model.get_articulation("head_tilt")
    leg_spread = object_model.get_articulation("leg_0_spread")

    ctx.allow_overlap(
        sleeve,
        mast,
        elem_a="sleeve_shell",
        elem_b="inner_pole",
        reason=(
            "The inner mast is intentionally represented as a tight sliding fit "
            "through the sleeve collar bushing."
        ),
    )
    ctx.allow_overlap(
        arm,
        mast,
        elem_a="pivot_collar",
        elem_b="inner_pole",
        reason=(
            "The arm collar clamp intentionally wraps and grips the pole at the pivot."
        ),
    )

    ctx.expect_within(
        mast,
        sleeve,
        axes="xy",
        inner_elem="inner_pole",
        outer_elem="sleeve_shell",
        margin=0.003,
        name="inner pole stays centered in hollow sleeve",
    )
    ctx.expect_overlap(
        mast,
        sleeve,
        axes="z",
        elem_a="inner_pole",
        elem_b="sleeve_shell",
        min_overlap=0.12,
        name="collapsed mast remains captured by sleeve",
    )
    ctx.expect_within(
        mast,
        arm,
        axes="xy",
        inner_elem="inner_pole",
        outer_elem="pivot_collar",
        margin=0.002,
        name="arm collar concentrically wraps pole",
    )
    ctx.expect_overlap(
        arm,
        mast,
        axes="z",
        elem_a="pivot_collar",
        elem_b="inner_pole",
        min_overlap=0.050,
        name="arm collar grips a real length of pole",
    )
    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.35}):
        ctx.expect_within(
            mast,
            sleeve,
            axes="xy",
            inner_elem="inner_pole",
            outer_elem="sleeve_shell",
            margin=0.003,
            name="extended pole remains centered in sleeve",
        )
        ctx.expect_overlap(
            mast,
            sleeve,
            axes="z",
            elem_a="inner_pole",
            elem_b="sleeve_shell",
            min_overlap=0.05,
            name="extended mast keeps retained insertion",
        )
        raised_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast slides upward",
        rest_mast_pos is not None
        and raised_mast_pos is not None
        and raised_mast_pos[2] > rest_mast_pos[2] + 0.30,
        details=f"rest={rest_mast_pos}, raised={raised_mast_pos}",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({arm_swing: 0.90}):
        swung_head_pos = ctx.part_world_position(head)
    ctx.check(
        "arm collar swings head around pole",
        rest_head_pos is not None
        and swung_head_pos is not None
        and abs(swung_head_pos[1] - rest_head_pos[1]) > 0.35,
        details=f"rest={rest_head_pos}, swung={swung_head_pos}",
    )

    def _elem_center_z(part, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return 0.5 * (lower[2] + upper[2])

    rest_head_z = _elem_center_z(head, "head_body")
    with ctx.pose({head_tilt: 0.65}):
        tilted_head_z = _elem_center_z(head, "head_body")
    ctx.check(
        "head yoke tilt aims housing downward",
        rest_head_z is not None and tilted_head_z is not None and tilted_head_z < rest_head_z - 0.025,
        details=f"rest_z={rest_head_z}, tilted_z={tilted_head_z}",
    )

    rest_foot = ctx.part_element_world_aabb(leg_0, elem="foot_pad")
    with ctx.pose({leg_spread: -0.30}):
        spread_foot = ctx.part_element_world_aabb(leg_0, elem="foot_pad")
    if rest_foot is not None and spread_foot is not None:
        rest_x = 0.5 * (rest_foot[0][0] + rest_foot[1][0])
        spread_x = 0.5 * (spread_foot[0][0] + spread_foot[1][0])
        ctx.check(
            "tripod leg spread changes footprint",
            abs(spread_x - rest_x) > 0.04,
            details=f"rest_x={rest_x}, spread_x={spread_x}",
        )
    else:
        ctx.fail("tripod leg spread changes footprint", "could not measure foot pad")

    return ctx.report()


object_model = build_object_model()
