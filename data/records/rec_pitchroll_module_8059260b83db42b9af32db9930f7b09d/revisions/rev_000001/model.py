from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitch_roll_mechanical_study")

    dark_steel = model.material("dark_phosphate_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    bead_blast = model.material("bead_blast_aluminum", rgba=(0.55, 0.57, 0.56, 1.0))
    bearing = model.material("ground_bearing_steel", rgba=(0.78, 0.79, 0.76, 1.0))
    black = model.material("black_oxide_hardware", rgba=(0.015, 0.014, 0.013, 1.0))
    brass = model.material("etched_brass_marks", rgba=(0.90, 0.67, 0.25, 1.0))
    paint = model.material("satin_safety_orange", rgba=(0.95, 0.28, 0.06, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.68, 0.30, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.182)),
        material=dark_steel,
        name="ground_plate",
    )
    base.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.56, 0.22, 0.26),
                span_width=0.42,
                trunnion_diameter=0.066,
                trunnion_center_z=0.170,
                base_thickness=0.030,
                corner_radius=0.010,
                center=False,
            ),
            "base_roll_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.170)),
        material=bead_blast,
        name="roll_yoke",
    )
    base.visual(
        Box((0.50, 0.050, 0.016)),
        origin=Origin(xyz=(0.0, -0.162, -0.164)),
        material=bead_blast,
        name="roll_scale_bar",
    )
    for i, x in enumerate((-0.20, -0.12, -0.04, 0.04, 0.12, 0.20)):
        base.visual(
            Box((0.004, 0.006, 0.030 if i in (0, 5) else 0.020)),
            origin=Origin(xyz=(x, -0.180, -0.142 if i in (0, 5) else -0.147)),
            material=brass,
            name=f"roll_tick_{i}",
        )
    for x in (-0.305, 0.305):
        base.visual(
            Box((0.050, 0.065, 0.012)),
            origin=Origin(xyz=(x, 0.0, -0.182)),
            material=black,
            name=f"foot_clamp_{0 if x < 0 else 1}",
        )
        for y in (-0.022, 0.022):
            base.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(x, y, -0.173)),
                material=black,
                name=f"foot_screw_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )

    roll_frame = model.part("roll_frame")
    roll_frame.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.30, 0.13, 0.22),
                span_width=0.205,
                trunnion_diameter=0.052,
                trunnion_center_z=0.110,
                base_thickness=0.024,
                corner_radius=0.008,
                center=True,
            ),
            "roll_pitch_yoke",
        ),
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=bead_blast,
        name="pitch_yoke",
    )
    roll_frame.visual(
        Box((0.405, 0.060, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=bead_blast,
        name="lower_spine",
    )
    for x in (-0.142, 0.142):
        roll_frame.visual(
            Box((0.050, 0.048, 0.152)),
            origin=Origin(xyz=(x, 0.0, -0.026)),
            material=bead_blast,
            name=f"roll_web_{0 if x < 0 else 1}",
        )
        roll_frame.visual(
            Cylinder(radius=0.024, length=0.162),
            origin=Origin(xyz=(x * 1.72, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=bearing,
            name=f"roll_trunnion_{0 if x < 0 else 1}",
        )
        roll_frame.visual(
            Cylinder(radius=0.039, length=0.026),
            origin=Origin(xyz=(x * 1.32, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name=f"roll_clamp_collar_{0 if x < 0 else 1}",
        )
        roll_frame.visual(
            Cylinder(radius=0.028, length=0.012),
            origin=Origin(xyz=(x * 2.30, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name=f"roll_end_cap_{0 if x < 0 else 1}",
        )
        roll_frame.visual(
            Cylinder(radius=0.036, length=0.012),
            origin=Origin(xyz=(-0.212 if x < 0 else 0.212, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=bearing,
            name=f"roll_thrust_washer_{0 if x < 0 else 1}",
        )
    roll_frame.visual(
        Box((0.010, 0.007, 0.050)),
        origin=Origin(xyz=(0.0, -0.172, -0.052)),
        material=paint,
        name="roll_index_pointer",
    )
    roll_frame.visual(
        Box((0.010, 0.152, 0.012)),
        origin=Origin(xyz=(0.0, -0.100, -0.077)),
        material=black,
        name="roll_pointer_bridge",
    )
    for y in (-0.112, 0.112):
        roll_frame.visual(
            Box((0.070, 0.014, 0.008)),
            origin=Origin(xyz=(0.0, y, 0.106)),
            material=black,
            name=f"pitch_access_screw_bar_{0 if y < 0 else 1}",
        )

    pitch_stage = model.part("pitch_stage")
    pitch_stage.visual(
        Cylinder(radius=0.015, length=0.365),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bearing,
        name="pitch_trunnion",
    )
    pitch_stage.visual(
        Box((0.120, 0.075, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=bead_blast,
        name="tilt_boss",
    )
    pitch_stage.visual(
        Box((0.160, 0.105, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_steel,
        name="mounting_plate",
    )
    pitch_stage.visual(
        Box((0.018, 0.105, 0.060)),
        origin=Origin(xyz=(-0.062, 0.0, 0.040)),
        material=bead_blast,
        name="side_rib_0",
    )
    pitch_stage.visual(
        Box((0.018, 0.105, 0.060)),
        origin=Origin(xyz=(0.062, 0.0, 0.040)),
        material=bead_blast,
        name="side_rib_1",
    )
    for y in (-0.086, 0.086):
        pitch_stage.visual(
            Cylinder(radius=0.034, length=0.020),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"pitch_clamp_collar_{0 if y < 0 else 1}",
        )
        pitch_stage.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(xyz=(0.0, y * 2.12, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"pitch_end_cap_{0 if y < 0 else 1}",
        )
        pitch_stage.visual(
            Cylinder(radius=0.031, length=0.008),
            origin=Origin(xyz=(0.0, -0.104 if y < 0 else 0.104, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bearing,
            name=f"pitch_thrust_washer_{0 if y < 0 else 1}",
        )
    for ix, x in enumerate((-0.052, 0.052)):
        for iy, y in enumerate((-0.032, 0.032)):
            pitch_stage.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(x, y, 0.072)),
                material=black,
                name=f"plate_bolt_{ix}_{iy}",
            )
    pitch_stage.visual(
        Box((0.004, 0.050, 0.004)),
        origin=Origin(xyz=(0.081, 0.0, 0.070)),
        material=brass,
        name="pitch_zero_mark",
    )

    for side, x in (("0", -0.293), ("1", 0.293)):
        cover = model.part(f"roll_cover_{side}")
        cover.visual(
            mesh_from_geometry(TorusGeometry(0.042, 0.013, radial_segments=48, tubular_segments=12), f"roll_cover_ring_{side}"),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name="cover_ring",
        )
        screw_x = -0.010 if x < 0 else 0.010
        for iz, z in enumerate((-0.035, 0.035)):
            for iy, y in enumerate((-0.035, 0.035)):
                cover.visual(
                    Cylinder(radius=0.0045, length=0.006),
                    origin=Origin(xyz=(screw_x, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                    material=bearing,
                    name=f"cover_screw_{iy}_{iz}",
                )
        model.articulation(
            f"base_to_roll_cover_{side}",
            ArticulationType.FIXED,
            parent=base,
            child=cover,
            origin=Origin(xyz=(x, 0.0, 0.0)),
        )

    for side, y in (("0", -0.160), ("1", 0.160)):
        cover = model.part(f"pitch_cover_{side}")
        cover.visual(
            mesh_from_geometry(TorusGeometry(0.040, 0.010, radial_segments=48, tubular_segments=10), f"pitch_cover_ring_{side}"),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=black,
            name="cover_ring",
        )
        screw_y = -0.012 if y < 0 else 0.012
        for ix, x in enumerate((-0.032, 0.032)):
            for iz, z in enumerate((-0.030, 0.030)):
                cover.visual(
                    Cylinder(radius=0.0038, length=0.005),
                    origin=Origin(xyz=(x, screw_y, z), rpy=(pi / 2.0, 0.0, 0.0)),
                    material=bearing,
                    name=f"cover_screw_{ix}_{iz}",
                )
        cover.visual(
            Box((0.004, 0.004, 0.030)),
            origin=Origin(xyz=(0.0, screw_y, 0.044)),
            material=paint,
            name="pitch_index_line",
        )
        model.articulation(
            f"roll_frame_to_pitch_cover_{side}",
            ArticulationType.FIXED,
            parent=roll_frame,
            child=cover,
            origin=Origin(xyz=(0.0, y, 0.0)),
        )

    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=roll_frame,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=roll_frame,
        child=pitch_stage,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.85, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    roll_frame = object_model.get_part("roll_frame")
    pitch_stage = object_model.get_part("pitch_stage")
    roll_axis = object_model.get_articulation("roll_axis")
    pitch_axis = object_model.get_articulation("pitch_axis")

    ctx.check(
        "orthogonal nested revolute axes",
        tuple(roll_axis.axis) == (1.0, 0.0, 0.0) and tuple(pitch_axis.axis) == (0.0, 1.0, 0.0),
        details=f"roll_axis={roll_axis.axis}, pitch_axis={pitch_axis.axis}",
    )
    for side in ("0", "1"):
        ctx.allow_overlap(
            base,
            roll_frame,
            elem_a="roll_yoke",
            elem_b=f"roll_thrust_washer_{side}",
            reason="The roll thrust washer is intentionally seated with a slight captured fit against the yoke bearing face.",
        )
        ctx.expect_overlap(
            roll_frame,
            base,
            axes="x",
            elem_a=f"roll_thrust_washer_{side}",
            elem_b="roll_yoke",
            min_overlap=0.002,
            name=f"roll washer {side} is retained at bearing face",
        )
        ctx.allow_overlap(
            roll_frame,
            pitch_stage,
            elem_a="pitch_yoke",
            elem_b=f"pitch_thrust_washer_{side}",
            reason="The pitch thrust washer is intentionally seated with a slight captured fit against the roll-frame cheek bearing face.",
        )
        ctx.expect_overlap(
            pitch_stage,
            roll_frame,
            axes="y",
            elem_a=f"pitch_thrust_washer_{side}",
            elem_b="pitch_yoke",
            min_overlap=0.002,
            name=f"pitch washer {side} is retained at bearing face",
        )
    ctx.expect_within(
        roll_frame,
        base,
        axes="x",
        inner_elem="pitch_yoke",
        outer_elem="roll_yoke",
        margin=0.012,
        name="roll cage sits between base trunnion cheeks",
    )
    ctx.expect_overlap(
        roll_frame,
        base,
        axes="x",
        elem_a="roll_trunnion_1",
        elem_b="roll_yoke",
        min_overlap=0.035,
        name="roll trunnion penetrates bearing span",
    )
    ctx.expect_within(
        pitch_stage,
        roll_frame,
        axes="y",
        inner_elem="mounting_plate",
        outer_elem="pitch_yoke",
        margin=0.010,
        name="pitch stage is carried inside roll frame cheeks",
    )
    rest_pos = ctx.part_world_position(pitch_stage)
    with ctx.pose({roll_axis: 0.35, pitch_axis: -0.40}):
        posed_pos = ctx.part_world_position(pitch_stage)
        ctx.expect_overlap(
            pitch_stage,
            roll_frame,
            axes="y",
            elem_a="pitch_trunnion",
            elem_b="pitch_yoke",
            min_overlap=0.080,
            name="pitch trunnions remain captured in rotated pose",
        )
    ctx.check(
        "nested pitch stage follows roll frame pose",
        rest_pos is not None and posed_pos is not None and abs(posed_pos[2] - rest_pos[2]) < 0.020,
        details=f"rest={rest_pos}, posed={posed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
