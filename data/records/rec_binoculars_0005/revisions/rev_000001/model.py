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
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="opera_glass", assets=ASSETS)

    brass = model.material("brass", rgba=(0.77, 0.66, 0.33, 1.0))
    black_enamel = model.material("black_enamel", rgba=(0.14, 0.11, 0.09, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.22, 0.24, 1.0))

    barrel_y = 0.029
    barrel_z = -0.018
    barrel_length = 0.072
    barrel_radius = 0.0145

    def add_half(part_name: str, side_sign: float, include_focus_wheel: bool) -> None:
        part = model.part(part_name)

        part.visual(
            Cylinder(radius=barrel_radius, length=barrel_length),
            origin=Origin(
                xyz=(0.018, side_sign * barrel_y, barrel_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black_enamel,
            name=f"{part_name}_barrel",
        )
        part.visual(
            Cylinder(radius=0.012, length=0.018),
            origin=Origin(
                xyz=(-0.022, side_sign * barrel_y, barrel_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_metal,
            name=f"{part_name}_eyecup",
        )
        part.visual(
            Cylinder(radius=0.0185, length=0.018),
            origin=Origin(
                xyz=(0.050, side_sign * barrel_y, barrel_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"{part_name}_objective_bell",
        )
        part.visual(
            Cylinder(radius=0.0195, length=0.004),
            origin=Origin(
                xyz=(0.061, side_sign * barrel_y, barrel_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"{part_name}_front_ring",
        )
        part.visual(
            Box((0.018, 0.018, 0.008)),
            origin=Origin(xyz=(-0.002, side_sign * 0.010, -0.006)),
            material=brass,
            name=f"{part_name}_bridge_arm",
        )
        part.visual(
            Box((0.010, 0.010, 0.006)),
            origin=Origin(xyz=(-0.001, side_sign * 0.004, 0.000)),
            material=brass,
            name=f"{part_name}_hinge_cheek",
        )

        pivot_name = "left_pivot_lower" if side_sign < 0.0 else "right_pivot_middle"
        pivot_z = -0.001 if side_sign < 0.0 else 0.003
        part.visual(
            Cylinder(radius=0.0065, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, pivot_z)),
            material=brass,
            name=pivot_name,
        )

        if include_focus_wheel:
            part.visual(
                Cylinder(radius=0.0035, length=0.008),
                origin=Origin(xyz=(0.0, 0.0, 0.003)),
                material=dark_metal,
                name="focus_spindle",
            )
            part.visual(
                Cylinder(radius=0.009, length=0.008),
                origin=Origin(xyz=(0.0, 0.0, 0.009)),
                material=brass,
                name="focus_wheel_core",
            )
            for tooth_index in range(10):
                angle = tooth_index * (2.0 * math.pi / 10.0)
                part.visual(
                    Box((0.005, 0.003, 0.008)),
                    origin=Origin(
                        xyz=(0.0085 * math.cos(angle), 0.0085 * math.sin(angle), 0.009),
                        rpy=(0.0, 0.0, angle),
                    ),
                    material=brass,
                    name=f"focus_tooth_{tooth_index}",
                )

        part.inertial = Inertial.from_geometry(
            Box((0.096, 0.046, 0.034)),
            mass=0.24 if include_focus_wheel else 0.22,
            origin=Origin(xyz=(0.018, side_sign * 0.018, -0.012)),
        )

    add_half("left_body", side_sign=-1.0, include_focus_wheel=True)
    add_half("right_body", side_sign=1.0, include_focus_wheel=False)

    left_body = model.get_part("left_body")
    right_body = model.get_part("right_body")

    model.articulation(
        "central_pivot",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=right_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-0.30,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    central_pivot = object_model.get_articulation("central_pivot")

    left_barrel = left_body.get_visual("left_body_barrel")
    right_barrel = right_body.get_visual("right_body_barrel")
    left_pivot_lower = left_body.get_visual("left_pivot_lower")
    right_pivot_middle = right_body.get_visual("right_pivot_middle")
    focus_wheel_core = left_body.get_visual("focus_wheel_core")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    ctx.expect_overlap(
        left_body,
        right_body,
        axes="xz",
        min_overlap=0.002,
        elem_a=left_barrel,
        elem_b=right_barrel,
        name="paired_barrels_run_parallel",
    )
    ctx.expect_gap(
        right_body,
        left_body,
        axis="y",
        min_gap=0.022,
        max_gap=0.034,
        positive_elem=right_barrel,
        negative_elem=left_barrel,
        name="barrels_have_slim_binocular_spacing",
    )
    ctx.expect_gap(
        right_body,
        left_body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=right_pivot_middle,
        negative_elem=left_pivot_lower,
        name="hinge_leafs_seat_together",
    )
    ctx.expect_contact(
        left_body,
        right_body,
        elem_a=focus_wheel_core,
        elem_b=right_pivot_middle,
        name="focus_wheel_caps_the_hinge_stack",
    )
    ctx.expect_within(
        right_body,
        left_body,
        axes="xy",
        inner_elem=right_pivot_middle,
        outer_elem=focus_wheel_core,
        name="focus_wheel_crowns_the_pivot",
    )

    with ctx.pose({central_pivot: 0.35}):
        ctx.expect_gap(
            right_body,
            left_body,
            axis="y",
            min_gap=0.012,
            max_gap=0.045,
            positive_elem=right_barrel,
            negative_elem=left_barrel,
            name="folded_pose_keeps_barrels_clear",
        )
        ctx.expect_gap(
            right_body,
            left_body,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=right_pivot_middle,
            negative_elem=left_pivot_lower,
            name="hinge_seat_holds_in_folded_pose",
        )
        ctx.expect_contact(
            left_body,
            right_body,
            elem_a=focus_wheel_core,
            elem_b=right_pivot_middle,
            name="focus_wheel_stays_seated_in_folded_pose",
        )
        ctx.expect_within(
            right_body,
            left_body,
            axes="xy",
            inner_elem=right_pivot_middle,
            outer_elem=focus_wheel_core,
            name="focus_wheel_still_crowns_pivot_in_folded_pose",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
