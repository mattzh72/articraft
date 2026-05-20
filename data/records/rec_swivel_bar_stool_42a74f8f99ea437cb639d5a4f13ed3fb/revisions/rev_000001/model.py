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
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.27, 1.0))
    vinyl_black = model.material("vinyl_black", rgba=(0.11, 0.11, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    bell_cover_mesh = _save_mesh(
        "bar_stool_bell_cover",
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.060, 0.000),
                (0.115, 0.010),
                (0.162, 0.030),
                (0.188, 0.060),
                (0.188, 0.072),
                (0.045, 0.072),
                (0.040, 0.086),
                (0.0, 0.086),
            ],
            segments=56,
        ),
    )
    foot_ring_mesh = _save_mesh(
        "bar_stool_foot_ring",
        TorusGeometry(radius=0.175, tube=0.011, radial_segments=18, tubular_segments=64),
    )
    cushion_mesh = _save_mesh(
        "bar_stool_seat_cushion",
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.065, 0.002),
                (0.120, 0.006),
                (0.150, 0.014),
                (0.166, 0.028),
                (0.160, 0.045),
                (0.118, 0.057),
                (0.060, 0.061),
                (0.0, 0.060),
            ],
            segments=56,
        ),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=0.225, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brushed_steel,
        name="base_disc",
    )
    pedestal_base.visual(
        Cylinder(radius=0.205, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="base_glide",
    )
    pedestal_base.visual(
        bell_cover_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=brushed_steel,
        name="bell_cover",
    )
    pedestal_base.visual(
        Cylinder(radius=0.034, length=0.575),
        origin=Origin(xyz=(0.0, 0.0, 0.3675)),
        material=brushed_steel,
        name="pedestal_post",
    )
    pedestal_base.visual(
        Cylinder(radius=0.046, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=dark_metal,
        name="foot_ring_hub",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        pedestal_base.visual(
            Cylinder(radius=0.008, length=0.142),
            origin=Origin(
                xyz=(0.106 * c, 0.106 * s, 0.300),
                rpy=(math.pi / 2.0, 0.0, angle + math.pi / 2.0),
            ),
            material=dark_metal,
            name=f"foot_ring_spoke_{index}",
        )
    pedestal_base.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=dark_metal,
        name="foot_ring",
    )
    pedestal_base.visual(
        Cylinder(radius=0.055, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        material=dark_metal,
        name="bearing_housing",
    )
    pedestal_base.visual(
        Cylinder(radius=0.025, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.665)),
        material=brushed_steel,
        name="bearing_cap",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 0.70)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    seat_assembly = model.part("seat_assembly")
    seat_assembly.visual(
        Cylinder(radius=0.053, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_metal,
        name="swivel_plate",
    )
    seat_assembly.visual(
        Cylinder(radius=0.042, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=dark_metal,
        name="swivel_hub",
    )
    seat_assembly.visual(
        Box((0.290, 0.032, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=dark_metal,
        name="support_bar_x",
    )
    seat_assembly.visual(
        Box((0.032, 0.290, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=dark_metal,
        name="support_bar_y",
    )
    seat_assembly.visual(
        Cylinder(radius=0.178, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=dark_metal,
        name="seat_pan",
    )
    seat_assembly.visual(
        cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=vinyl_black,
        name="seat_cushion",
    )
    seat_assembly.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 0.16)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=seat_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
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

    pedestal_base = object_model.get_part("pedestal_base")
    seat_assembly = object_model.get_part("seat_assembly")
    seat_swivel = object_model.get_articulation("seat_swivel")

    with ctx.pose({seat_swivel: 0.0}):
        ctx.expect_gap(
            seat_assembly,
            pedestal_base,
            axis="z",
            positive_elem="swivel_plate",
            negative_elem="bearing_housing",
            max_gap=0.001,
            max_penetration=0.0,
            name="swivel plate rests cleanly on the bearing housing",
        )
        ctx.expect_overlap(
            seat_assembly,
            pedestal_base,
            axes="xy",
            elem_a="swivel_hub",
            elem_b="bearing_housing",
            min_overlap=0.08,
            name="seat swivel stays centered over the pedestal axis",
        )
        ctx.expect_origin_gap(
            seat_assembly,
            pedestal_base,
            axis="z",
            min_gap=0.66,
            max_gap=0.69,
            name="seat swivel stage sits high above the base",
        )

    rest_pos = ctx.part_world_position(seat_assembly)
    with ctx.pose({seat_swivel: math.pi / 2.0}):
        rotated_pos = ctx.part_world_position(seat_assembly)

    ctx.check(
        "seat rotates in place about the pedestal axis",
        rest_pos is not None
        and rotated_pos is not None
        and abs(rest_pos[0] - rotated_pos[0]) < 1e-6
        and abs(rest_pos[1] - rotated_pos[1]) < 1e-6
        and abs(rest_pos[2] - rotated_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
