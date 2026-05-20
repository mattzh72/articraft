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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_side_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    black_steel = model.material("black_steel", rgba=(0.15, 0.16, 0.17, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.16, 0.10, 0.08, 1.0))

    base_profile = [
        (0.0, 0.000),
        (0.080, 0.000),
        (0.170, 0.006),
        (0.215, 0.016),
        (0.228, 0.031),
        (0.205, 0.041),
        (0.110, 0.046),
        (0.000, 0.046),
    ]
    floor_base_mesh = _mesh("floor_base", LatheGeometry(base_profile, segments=72))
    seat_cushion_mesh = _mesh(
        "seat_cushion",
        superellipse_side_loft(
            [
                (-0.175, 0.000, 0.032, 0.210),
                (-0.070, 0.000, 0.049, 0.300),
                (0.055, 0.000, 0.051, 0.315),
                (0.165, 0.000, 0.034, 0.225),
            ],
            exponents=2.4,
            segments=56,
        ),
    )
    foot_ring_mesh = _mesh(
        "foot_ring",
        TorusGeometry(
            radius=0.170,
            tube=0.011,
            radial_segments=18,
            tubular_segments=56,
        ),
    )

    base = model.part("base")
    base.visual(floor_base_mesh, material=black_steel, name="floor_base")
    base.visual(
        Cylinder(radius=0.078, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=satin_steel,
        name="bearing_housing",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=black_steel,
        name="upper_trim_cap",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.230, length=0.100),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    seat_assembly = model.part("seat_assembly")
    seat_assembly.visual(
        Cylinder(radius=0.056, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin_steel,
        name="swivel_drum",
    )
    seat_assembly.visual(
        Cylinder(radius=0.028, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        material=satin_steel,
        name="seat_post",
    )
    seat_assembly.visual(
        Cylinder(radius=0.041, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=black_steel,
        name="post_shroud",
    )
    seat_assembly.visual(
        Cylinder(radius=0.047, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=satin_steel,
        name="foot_ring_collar",
    )
    seat_assembly.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=satin_steel,
        name="foot_ring",
    )
    for suffix, xyz, rpy in (
        ("x_pos", (0.102, 0.0, 0.225), (0.0, math.pi / 2.0, 0.0)),
        ("x_neg", (-0.102, 0.0, 0.225), (0.0, math.pi / 2.0, 0.0)),
        ("y_pos", (0.0, 0.102, 0.225), (math.pi / 2.0, 0.0, 0.0)),
        ("y_neg", (0.0, -0.102, 0.225), (math.pi / 2.0, 0.0, 0.0)),
    ):
        seat_assembly.visual(
            Cylinder(radius=0.008, length=0.138),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=satin_steel,
            name=f"foot_ring_spoke_{suffix}",
        )
    seat_assembly.visual(
        Cylinder(radius=0.095, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.565)),
        material=black_steel,
        name="underseat_hub",
    )
    seat_assembly.visual(
        Box((0.225, 0.175, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.607)),
        material=black_steel,
        name="seat_pan_plate",
    )
    seat_assembly.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.613)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat_assembly.inertial = Inertial.from_geometry(
        Box((0.360, 0.360, 0.665)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.333)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
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
    seat_assembly = object_model.get_part("seat_assembly")
    swivel = object_model.get_articulation("seat_swivel")

    limits = swivel.motion_limits
    ctx.check(
        "seat uses a continuous vertical swivel joint",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and swivel.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None
        and swivel.origin.xyz[2] <= 0.10,
        details=(
            f"type={swivel.articulation_type}, axis={swivel.axis}, "
            f"origin={swivel.origin.xyz}, lower={None if limits is None else limits.lower}, "
            f"upper={None if limits is None else limits.upper}"
        ),
    )

    ctx.expect_gap(
        seat_assembly,
        base,
        axis="z",
        positive_elem="seat_cushion",
        negative_elem="floor_base",
        min_gap=0.62,
        name="seat sits at bar stool height above the base",
    )
    ctx.expect_gap(
        seat_assembly,
        base,
        axis="z",
        positive_elem="foot_ring",
        negative_elem="floor_base",
        min_gap=0.22,
        name="foot ring clears the base",
    )
    ctx.expect_overlap(
        seat_assembly,
        base,
        axes="xy",
        elem_a="seat_cushion",
        elem_b="floor_base",
        min_overlap=0.18,
        name="seat remains centered over the pedestal base",
    )

    rest_pos = ctx.part_world_position(seat_assembly)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(seat_assembly)
        ctx.expect_gap(
            seat_assembly,
            base,
            axis="z",
            positive_elem="seat_cushion",
            negative_elem="floor_base",
            min_gap=0.62,
            name="seat height is preserved while swiveling",
        )

    ctx.check(
        "swivel keeps the seat post centered on the pedestal axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) <= 1e-6
        and abs(rest_pos[1] - turned_pos[1]) <= 1e-6
        and abs(rest_pos[2] - turned_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
