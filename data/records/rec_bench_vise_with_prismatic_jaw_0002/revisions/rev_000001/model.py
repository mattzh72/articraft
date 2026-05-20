from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

SCREW_Z = 0.64
GUIDE_Z = 0.24


def _x_axis_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _y_axis_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_leg_vise", assets=ASSETS)

    bench_oak = model.material("bench_oak", rgba=(0.63, 0.48, 0.31, 1.0))
    jaw_maple = model.material("jaw_maple", rgba=(0.73, 0.60, 0.40, 1.0))
    screw_walnut = model.material("screw_walnut", rgba=(0.42, 0.26, 0.14, 1.0))
    iron = model.material("iron", rgba=(0.33, 0.35, 0.38, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.20, 0.21, 0.23, 1.0))

    bench = model.part("bench")

    bench.visual(
        Box((0.40, 0.66, 0.08)),
        origin=Origin(xyz=(0.20, 0.0, 0.86)),
        material=bench_oak,
        name="benchtop",
    )
    bench.visual(
        Box((0.10, 0.225, 0.32)),
        origin=Origin(xyz=(0.05, -0.1775, 0.66)),
        material=bench_oak,
        name="apron_left",
    )
    bench.visual(
        Box((0.10, 0.225, 0.32)),
        origin=Origin(xyz=(0.05, 0.1775, 0.66)),
        material=bench_oak,
        name="apron_right",
    )
    bench.visual(
        Box((0.10, 0.13, 0.14)),
        origin=Origin(xyz=(0.05, 0.0, 0.75)),
        material=bench_oak,
        name="apron_top",
    )
    bench.visual(
        Box((0.10, 0.13, 0.10)),
        origin=Origin(xyz=(0.05, 0.0, 0.55)),
        material=bench_oak,
        name="apron_bottom",
    )
    bench.visual(
        Box((0.10, 0.055, 0.34)),
        origin=Origin(xyz=(0.05, -0.0525, 0.33)),
        material=bench_oak,
        name="leg_cheek_left",
    )
    bench.visual(
        Box((0.10, 0.055, 0.34)),
        origin=Origin(xyz=(0.05, 0.0525, 0.33)),
        material=bench_oak,
        name="leg_cheek_right",
    )

    bench.visual(
        Box((0.16, 0.055, 0.12)),
        origin=Origin(xyz=(0.11, -0.0625, SCREW_Z)),
        material=bench_oak,
        name="nut_housing_left",
    )
    bench.visual(
        Box((0.16, 0.055, 0.12)),
        origin=Origin(xyz=(0.11, 0.0625, SCREW_Z)),
        material=bench_oak,
        name="nut_housing_right",
    )
    bench.visual(
        Box((0.16, 0.07, 0.025)),
        origin=Origin(xyz=(0.11, 0.0, SCREW_Z + 0.0475)),
        material=bench_oak,
        name="nut_housing_top",
    )
    bench.visual(
        Box((0.16, 0.07, 0.025)),
        origin=Origin(xyz=(0.11, 0.0, SCREW_Z - 0.0475)),
        material=bench_oak,
        name="nut_housing_bottom",
    )
    bench.visual(
        Box((0.08, 0.11, 0.15)),
        origin=Origin(xyz=(0.04, 0.0, SCREW_Z)),
        material=bench_oak,
        name="front_nut_cheek",
    )
    bench.visual(
        Cylinder(radius=0.032, length=0.12),
        origin=_x_axis_origin(0.06, 0.0, SCREW_Z),
        material=dark_iron,
        name="captured_nut",
    )

    bench.visual(
        Box((0.10, 0.055, 0.13)),
        origin=Origin(xyz=(0.05, -0.0525, GUIDE_Z)),
        material=bench_oak,
        name="guide_block_left",
    )
    bench.visual(
        Box((0.10, 0.055, 0.13)),
        origin=Origin(xyz=(0.05, 0.0525, GUIDE_Z)),
        material=bench_oak,
        name="guide_block_right",
    )
    bench.visual(
        Box((0.10, 0.05, 0.04)),
        origin=Origin(xyz=(0.05, 0.0, GUIDE_Z + 0.045)),
        material=bench_oak,
        name="guide_block_top",
    )
    bench.visual(
        Box((0.10, 0.05, 0.04)),
        origin=Origin(xyz=(0.05, 0.0, GUIDE_Z - 0.045)),
        material=bench_oak,
        name="guide_block_bottom",
    )
    bench.visual(
        Cylinder(radius=0.017, length=0.08),
        origin=_x_axis_origin(0.04, 0.0, GUIDE_Z),
        material=iron,
        name="guide_bushing",
    )
    bench.visual(
        Box((0.03, 0.05, 0.06)),
        origin=Origin(xyz=(0.015, 0.0, GUIDE_Z)),
        material=bench_oak,
        name="front_guide_cheek",
    )
    bench.inertial = Inertial.from_geometry(
        Box((0.40, 0.66, 0.82)),
        mass=28.0,
        origin=Origin(xyz=(0.20, 0.0, 0.41)),
    )

    jaw = model.part("jaw")
    jaw.visual(
        Box((0.055, 0.24, 0.74)),
        origin=Origin(xyz=(-0.0315, 0.0, -0.19)),
        material=jaw_maple,
        name="jaw_face",
    )
    jaw.visual(
        Cylinder(radius=0.046, length=0.024),
        origin=_x_axis_origin(-0.016, 0.0, 0.0),
        material=screw_walnut,
        name="screw_boss",
    )
    jaw.visual(
        Cylinder(radius=0.028, length=0.33),
        origin=_x_axis_origin(0.03, 0.0, 0.0),
        material=screw_walnut,
        name="screw_core",
    )
    for index in range(16):
        jaw.visual(
            Cylinder(radius=0.032, length=0.006),
            origin=_x_axis_origin(-0.105 + 0.015 * index, 0.0, 0.0),
            material=screw_walnut,
            name=f"thread_ring_{index}",
        )
    jaw.visual(
        Cylinder(radius=0.052, length=0.045),
        origin=_x_axis_origin(-0.155, 0.0, 0.0),
        material=screw_walnut,
        name="handle_hub",
    )
    jaw.visual(
        Cylinder(radius=0.011, length=0.31),
        origin=_y_axis_origin(-0.155, 0.0, 0.0),
        material=screw_walnut,
        name="handle_bar",
    )
    jaw.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(-0.155, -0.155, 0.0)),
        material=screw_walnut,
        name="handle_knob_left",
    )
    jaw.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(-0.155, 0.155, 0.0)),
        material=screw_walnut,
        name="handle_knob_right",
    )
    jaw.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=_x_axis_origin(-0.016, 0.0, GUIDE_Z - SCREW_Z),
        material=iron,
        name="guide_collar",
    )
    jaw.visual(
        Cylinder(radius=0.012, length=0.32),
        origin=_x_axis_origin(0.05, 0.0, GUIDE_Z - SCREW_Z),
        material=iron,
        name="guide_rod",
    )
    jaw.visual(
        Cylinder(radius=0.0065, length=0.18),
        origin=_y_axis_origin(0.185, 0.0, GUIDE_Z - SCREW_Z),
        material=dark_iron,
        name="guide_pin",
    )
    jaw.inertial = Inertial.from_geometry(
        Box((0.40, 0.32, 0.78)),
        mass=7.0,
        origin=Origin(xyz=(0.00, 0.0, -0.19)),
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=bench,
        child=jaw,
        origin=Origin(xyz=(0.0, 0.0, SCREW_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.18, lower=0.0, upper=0.12),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bench = object_model.get_part("bench")
    jaw = object_model.get_part("jaw")
    jaw_slide = object_model.get_articulation("jaw_slide")

    jaw_face = jaw.get_visual("jaw_face")
    screw_core = jaw.get_visual("screw_core")
    guide_rod = jaw.get_visual("guide_rod")
    guide_pin = jaw.get_visual("guide_pin")
    handle_bar = jaw.get_visual("handle_bar")
    handle_hub = jaw.get_visual("handle_hub")
    captured_nut = bench.get_visual("captured_nut")
    guide_bushing = bench.get_visual("guide_bushing")
    apron_top = bench.get_visual("apron_top")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        jaw,
        bench,
        reason="the wooden screw and iron parallel guide intentionally nest through the captured nut and guide block during travel",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        jaw,
        bench,
        axes="yz",
        min_overlap=0.13,
        elem_a=jaw_face,
        elem_b=apron_top,
        name="jaw_covers_apron_height",
    )
    ctx.expect_gap(
        bench,
        jaw,
        axis="x",
        max_gap=0.008,
        max_penetration=0.0,
        negative_elem=jaw_face,
        name="jaw_closes_near_bench_face",
    )
    ctx.expect_overlap(
        jaw,
        bench,
        axes="yz",
        min_overlap=0.045,
        elem_a=screw_core,
        elem_b=captured_nut,
        name="screw_runs_through_captured_nut",
    )
    ctx.expect_overlap(
        jaw,
        bench,
        axes="yz",
        min_overlap=0.020,
        elem_a=guide_rod,
        elem_b=guide_bushing,
        name="parallel_guide_runs_through_fixed_block",
    )
    ctx.expect_contact(
        jaw,
        jaw,
        elem_a=guide_pin,
        elem_b=guide_rod,
        name="guide_pin_passes_through_parallel_rod",
    )
    ctx.expect_overlap(
        jaw,
        jaw,
        axes="yz",
        min_overlap=0.020,
        elem_a=handle_bar,
        elem_b=handle_hub,
        name="turned_handle_crosses_screw_hub",
    )

    with ctx.pose({jaw_slide: 0.09}):
        ctx.expect_gap(
            bench,
            jaw,
            axis="x",
            min_gap=0.08,
            max_gap=0.12,
            negative_elem=jaw_face,
            name="jaw_opens_forward",
        )
        ctx.expect_overlap(
            jaw,
            bench,
            axes="yz",
            min_overlap=0.13,
            elem_a=jaw_face,
            elem_b=apron_top,
            name="jaw_stays_under_apron_when_open",
        )
        ctx.expect_overlap(
            jaw,
            bench,
            axes="yz",
            min_overlap=0.030,
            elem_a=screw_core,
            elem_b=captured_nut,
            name="screw_stays_engaged_when_open",
        )
        ctx.expect_overlap(
            jaw,
            bench,
            axes="yz",
            min_overlap=0.020,
            elem_a=guide_rod,
            elem_b=guide_bushing,
            name="parallel_guide_stays_registered_when_open",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
