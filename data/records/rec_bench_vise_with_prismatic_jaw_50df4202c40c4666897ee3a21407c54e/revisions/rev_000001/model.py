from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _helical_thread_geometry(
    *,
    x_start: float,
    x_end: float,
    helix_radius: float,
    wire_radius: float,
    pitch: float,
    points_per_turn: int = 12,
):
    length = x_end - x_start
    turns = max(1.0, length / pitch)
    samples = max(16, int(turns * points_per_turn))
    pts: list[tuple[float, float, float]] = []
    for idx in range(samples + 1):
        t = idx / samples
        angle = 2.0 * math.pi * turns * t
        x = x_start + length * t
        y = helix_radius * math.cos(angle)
        z = helix_radius * math.sin(angle)
        pts.append((x, y, z))
    return tube_from_spline_points(
        pts,
        radius=wire_radius,
        samples_per_segment=2,
        radial_segments=10,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tail_vise")

    maple = model.material("maple", rgba=(0.76, 0.63, 0.43, 1.0))
    oiled_maple = model.material("oiled_maple", rgba=(0.62, 0.46, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.48, 0.49, 0.50, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.25, 1.0))

    top_surface_z = 0.84
    top_thickness = 0.09
    top_center_z = top_surface_z - top_thickness * 0.5

    bench_base = model.part("bench_base")
    bench_base.visual(
        Box((0.72, 0.33, top_thickness)),
        origin=Origin(xyz=(-0.36, 0.125, top_center_z)),
        material=maple,
        name="rear_top_body",
    )
    bench_base.visual(
        Box((0.72, 0.06, 0.03)),
        origin=Origin(xyz=(-0.36, -0.05, top_surface_z - 0.015)),
        material=maple,
        name="rear_top",
    )
    bench_base.visual(
        Box((0.40, 0.20, top_thickness)),
        origin=Origin(xyz=(-0.52, -0.18, top_center_z)),
        material=maple,
        name="front_top_strip",
    )
    bench_base.visual(
        Box((0.40, 0.05, 0.20)),
        origin=Origin(xyz=(-0.52, -0.275, 0.65)),
        material=oiled_maple,
        name="front_apron",
    )
    bench_base.visual(
        Box((0.72, 0.05, 0.12)),
        origin=Origin(xyz=(-0.36, 0.265, 0.69)),
        material=oiled_maple,
        name="rear_skirt",
    )
    bench_base.visual(
        Box((0.05, 0.37, 0.28)),
        origin=Origin(xyz=(0.025, 0.105, 0.655)),
        material=oiled_maple,
        name="rear_end_cap",
    )
    bench_base.visual(
        Box((0.08, 0.05, 0.18)),
        origin=Origin(xyz=(-0.28, -0.232, 0.66)),
        material=oiled_maple,
        name="rail_anchor_front",
    )
    bench_base.visual(
        Box((0.08, 0.05, 0.18)),
        origin=Origin(xyz=(-0.28, -0.128, 0.66)),
        material=oiled_maple,
        name="rail_anchor_rear",
    )
    bench_base.visual(
        Box((0.31, 0.035, 0.045)),
        origin=Origin(xyz=(-0.155, -0.245, 0.6225)),
        material=oiled_maple,
        name="rail_front",
    )
    bench_base.visual(
        Box((0.31, 0.035, 0.045)),
        origin=Origin(xyz=(-0.155, -0.115, 0.6225)),
        material=oiled_maple,
        name="rail_rear",
    )
    bench_base.visual(
        Box((0.03, 0.10, 0.03)),
        origin=Origin(xyz=(-0.015, -0.18, 0.705)),
        material=oiled_maple,
        name="screw_support_top",
    )
    bench_base.visual(
        Box((0.03, 0.10, 0.03)),
        origin=Origin(xyz=(-0.015, -0.18, 0.575)),
        material=oiled_maple,
        name="screw_support_bottom",
    )
    bench_base.visual(
        Box((0.03, 0.02, 0.10)),
        origin=Origin(xyz=(-0.015, -0.22, 0.64)),
        material=oiled_maple,
        name="screw_support_front_cheek",
    )
    bench_base.visual(
        Box((0.03, 0.02, 0.10)),
        origin=Origin(xyz=(-0.015, -0.14, 0.64)),
        material=oiled_maple,
        name="screw_support_rear_cheek",
    )
    bench_base.inertial = Inertial.from_geometry(
        Box((0.77, 0.58, 0.33)),
        mass=42.0,
        origin=Origin(xyz=(-0.335, 0.0, 0.675)),
    )

    sliding_bed = model.part("sliding_bed")
    sliding_bed.visual(
        Box((0.32, 0.20, 0.085)),
        origin=Origin(xyz=(-0.16, -0.18, top_surface_z - 0.0425)),
        material=maple,
        name="bed_top",
    )
    sliding_bed.visual(
        Box((0.32, 0.026, 0.165)),
        origin=Origin(xyz=(-0.16, -0.277, 0.6725)),
        material=oiled_maple,
        name="bed_runner_front",
    )
    sliding_bed.visual(
        Box((0.32, 0.026, 0.165)),
        origin=Origin(xyz=(-0.16, -0.083, 0.6725)),
        material=oiled_maple,
        name="bed_runner_rear",
    )
    sliding_bed.inertial = Inertial.from_geometry(
        Box((0.32, 0.20, 0.25)),
        mass=9.5,
        origin=Origin(xyz=(-0.16, -0.18, 0.715)),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.05, 0.20, 0.055)),
        origin=Origin(xyz=(0.025, -0.18, 0.7275)),
        material=oiled_maple,
        name="jaw_top",
    )
    moving_jaw.visual(
        Box((0.05, 0.20, 0.04)),
        origin=Origin(xyz=(0.025, -0.18, 0.58)),
        material=oiled_maple,
        name="jaw_bottom",
    )
    moving_jaw.visual(
        Box((0.05, 0.032, 0.11)),
        origin=Origin(xyz=(0.025, -0.264, 0.6475)),
        material=oiled_maple,
        name="jaw_front_stile",
    )
    moving_jaw.visual(
        Box((0.05, 0.032, 0.11)),
        origin=Origin(xyz=(0.025, -0.096, 0.6475)),
        material=oiled_maple,
        name="jaw_rear_stile",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.05, 0.20, 0.19)),
        mass=4.8,
        origin=Origin(xyz=(0.025, -0.18, 0.65)),
    )

    lead_screw = model.part("lead_screw")
    screw_thread = mesh_from_geometry(
        _helical_thread_geometry(
            x_start=-0.36,
            x_end=0.20,
            helix_radius=0.0142,
            wire_radius=0.0018,
            pitch=0.03,
        ),
        "tail_vise_screw_thread",
    )
    lead_screw.visual(
        Cylinder(radius=0.013, length=0.62),
        origin=Origin(xyz=(-0.08, -0.18, 0.64), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="screw_core",
    )
    lead_screw.visual(
        screw_thread,
        origin=Origin(xyz=(0.0, -0.18, 0.64)),
        material=steel,
        name="screw_thread",
    )
    lead_screw.visual(
        Cylinder(radius=0.021, length=0.05),
        origin=Origin(xyz=(0.235, -0.18, 0.64), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="handle_hub",
    )
    lead_screw.visual(
        Cylinder(radius=0.03, length=0.018),
        origin=Origin(xyz=(-0.015, -0.18, 0.64), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="bearing_collar",
    )
    lead_screw.visual(
        Cylinder(radius=0.009, length=0.24),
        origin=Origin(xyz=(0.256, -0.18, 0.64), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_bar",
    )
    lead_screw.visual(
        Cylinder(radius=0.017, length=0.055),
        origin=Origin(xyz=(0.256, -0.295, 0.64), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oiled_maple,
        name="handle_knob_front",
    )
    lead_screw.visual(
        Cylinder(radius=0.017, length=0.055),
        origin=Origin(xyz=(0.256, -0.065, 0.64), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oiled_maple,
        name="handle_knob_rear",
    )
    lead_screw.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.62),
        mass=3.2,
        origin=Origin(xyz=(-0.08, -0.18, 0.64), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    bed_slide = model.articulation(
        "bed_slide",
        ArticulationType.PRISMATIC,
        parent=bench_base,
        child=sliding_bed,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.10,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "bed_to_jaw",
        ArticulationType.FIXED,
        parent=sliding_bed,
        child=moving_jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    lead_screw_spin = model.articulation(
        "lead_screw_spin",
        ArticulationType.CONTINUOUS,
        parent=bench_base,
        child=lead_screw,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=8.0),
    )

    model.meta["key_articulations"] = {
        "bed_slide": bed_slide.name,
        "lead_screw_spin": lead_screw_spin.name,
    }
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

    bench_base = object_model.get_part("bench_base")
    sliding_bed = object_model.get_part("sliding_bed")
    moving_jaw = object_model.get_part("moving_jaw")
    lead_screw = object_model.get_part("lead_screw")
    bed_slide = object_model.get_articulation("bed_slide")
    lead_screw_spin = object_model.get_articulation("lead_screw_spin")

    rear_top = bench_base.get_visual("rear_top")
    front_top_strip = bench_base.get_visual("front_top_strip")
    rail_front = bench_base.get_visual("rail_front")
    rail_rear = bench_base.get_visual("rail_rear")
    bed_top = sliding_bed.get_visual("bed_top")
    bed_runner_front = sliding_bed.get_visual("bed_runner_front")
    bed_runner_rear = sliding_bed.get_visual("bed_runner_rear")
    jaw_top = moving_jaw.get_visual("jaw_top")
    handle_bar = lead_screw.get_visual("handle_bar")

    ctx.check(
        "bed slide joint axis is along bench length",
        tuple(round(v, 6) for v in bed_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={bed_slide.axis}",
    )
    ctx.check(
        "lead screw handle uses continuous rotation",
        lead_screw_spin.motion_limits is not None
        and lead_screw_spin.motion_limits.lower is None
        and lead_screw_spin.motion_limits.upper is None,
        details=f"limits={lead_screw_spin.motion_limits}",
    )
    ctx.expect_contact(
        sliding_bed,
        moving_jaw,
        elem_a=bed_top,
        elem_b=jaw_top,
        name="moving jaw is mounted directly to the sliding bed",
    )

    with ctx.pose({bed_slide: 0.0}):
        front_top_aabb = ctx.part_element_world_aabb(bench_base, elem=front_top_strip)
        bed_top_aabb = ctx.part_element_world_aabb(sliding_bed, elem=bed_top)
        flush_ok = (
            front_top_aabb is not None
            and bed_top_aabb is not None
            and abs(front_top_aabb[1][2] - bed_top_aabb[1][2]) <= 0.001
        )
        ctx.check(
            "sliding bed top stays flush with the bench top at rest",
            flush_ok,
            details=f"front_top={front_top_aabb}, bed_top={bed_top_aabb}",
        )

        rear_top_aabb = ctx.part_element_world_aabb(bench_base, elem=rear_top)
        bed_top_aabb = ctx.part_element_world_aabb(sliding_bed, elem=bed_top)
        pocket_align_ok = (
            rear_top_aabb is not None
            and bed_top_aabb is not None
            and abs(rear_top_aabb[0][1] - bed_top_aabb[1][1]) <= 0.0015
        )
        ctx.check(
            "sliding bed meets the fixed rear benchtop edge cleanly",
            pocket_align_ok,
            details=f"rear_top={rear_top_aabb}, bed_top={bed_top_aabb}",
        )

    rest_position = ctx.part_world_position(sliding_bed)
    with ctx.pose({bed_slide: 0.18}):
        ctx.expect_overlap(
            sliding_bed,
            bench_base,
            axes="x",
            elem_a=bed_runner_front,
            elem_b=rail_front,
            min_overlap=0.12,
            name="front guide runner retains engagement on its rail at full extension",
        )
        ctx.expect_overlap(
            sliding_bed,
            bench_base,
            axes="x",
            elem_a=bed_runner_rear,
            elem_b=rail_rear,
            min_overlap=0.12,
            name="rear guide runner retains engagement on its rail at full extension",
        )
        extended_position = ctx.part_world_position(sliding_bed)
    ctx.check(
        "sliding bed extends outward from the bench end",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.16,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    with ctx.pose({lead_screw_spin: 0.0}):
        rest_bar_aabb = ctx.part_element_world_aabb(lead_screw, elem=handle_bar)
        rest_screw_pos = ctx.part_world_position(lead_screw)
    with ctx.pose({lead_screw_spin: math.pi / 2.0}):
        quarter_bar_aabb = ctx.part_element_world_aabb(lead_screw, elem=handle_bar)
        quarter_screw_pos = ctx.part_world_position(lead_screw)

    if rest_bar_aabb is None or quarter_bar_aabb is None:
        ctx.fail(
            "lead screw handle bar remains measurable through rotation",
            details=f"rest={rest_bar_aabb}, quarter={quarter_bar_aabb}",
        )
    else:
        rest_y_span = rest_bar_aabb[1][1] - rest_bar_aabb[0][1]
        rest_z_span = rest_bar_aabb[1][2] - rest_bar_aabb[0][2]
        quarter_y_span = quarter_bar_aabb[1][1] - quarter_bar_aabb[0][1]
        quarter_z_span = quarter_bar_aabb[1][2] - quarter_bar_aabb[0][2]
        ctx.check(
            "lead screw handle bar rotates around the screw axis",
            rest_y_span > rest_z_span * 2.0 and quarter_z_span > quarter_y_span * 2.0,
            details=(
                f"rest_y_span={rest_y_span}, rest_z_span={rest_z_span}, "
                f"quarter_y_span={quarter_y_span}, quarter_z_span={quarter_z_span}"
            ),
        )

    ctx.check(
        "lead screw stays axially mounted while the handle rotates",
        rest_screw_pos is not None
        and quarter_screw_pos is not None
        and max(abs(a - b) for a, b in zip(rest_screw_pos, quarter_screw_pos)) <= 1e-6,
        details=f"rest={rest_screw_pos}, quarter={quarter_screw_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
