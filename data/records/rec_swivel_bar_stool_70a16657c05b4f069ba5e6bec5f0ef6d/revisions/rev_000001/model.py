from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    powder_black = model.material("powder_black", rgba=(0.16, 0.16, 0.17, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.22, 0.14, 0.10, 1.0))

    foot_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.17, tube=0.011, radial_segments=18, tubular_segments=72),
        "foot_ring",
    )
    seat_cushion_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.09, 0.0),
                (0.15, 0.006),
                (0.17, 0.016),
                (0.176, 0.032),
                (0.168, 0.050),
                (0.10, 0.061),
                (0.0, 0.055),
            ],
            segments=64,
        ),
        "seat_cushion",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.22, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=powder_black,
        name="floor_disc",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.64),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=brushed_steel,
        name="pedestal_post",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=powder_black,
        name="foot_ring_collar",
    )
    base.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=brushed_steel,
        name="foot_ring",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.13),
        origin=Origin(xyz=(0.11, 0.0, 0.30), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="foot_ring_arm_pos_x",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.13),
        origin=Origin(xyz=(-0.11, 0.0, 0.30), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="foot_ring_arm_neg_x",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.13),
        origin=Origin(xyz=(0.0, 0.11, 0.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
        name="foot_ring_arm_pos_y",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.13),
        origin=Origin(xyz=(0.0, -0.11, 0.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
        name="foot_ring_arm_neg_y",
    )
    base.visual(
        Cylinder(radius=0.05, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        material=powder_black,
        name="bearing_head",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.72),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.042, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=powder_black,
        name="swivel_hub",
    )
    seat.visual(
        Cylinder(radius=0.085, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=powder_black,
        name="seat_pan",
    )
    seat.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.175, length=0.095),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.71)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=8.0),
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

    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="swivel_hub",
        negative_elem="bearing_head",
        max_gap=0.001,
        max_penetration=0.0005,
        name="seat hub sits on bearing head",
    )
    ctx.expect_overlap(
        seat,
        base,
        axes="xy",
        elem_a="swivel_hub",
        elem_b="bearing_head",
        min_overlap=0.07,
        name="seat hub stays centered over pedestal head",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_gap(
            seat,
            base,
            axis="z",
            positive_elem="swivel_hub",
            negative_elem="bearing_head",
            max_gap=0.001,
            max_penetration=0.0005,
            name="seat hub remains seated while rotated",
        )
        ctx.expect_overlap(
            seat,
            base,
            axes="xy",
            elem_a="swivel_hub",
            elem_b="bearing_head",
            min_overlap=0.07,
            name="seat swivel remains coaxial when rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
