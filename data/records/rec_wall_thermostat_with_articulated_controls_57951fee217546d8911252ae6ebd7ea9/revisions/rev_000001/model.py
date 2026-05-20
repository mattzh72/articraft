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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    plate_white = model.material("plate_white", rgba=(0.93, 0.93, 0.91, 1.0))
    body_graphite = model.material("body_graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    ring_steel = model.material("ring_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    face_glass = model.material("face_glass", rgba=(0.10, 0.12, 0.14, 1.0))

    housing = model.part("housing")

    wall_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.120, 0.120, 0.012, corner_segments=10),
            0.004,
        ),
        "thermostat_wall_plate",
    )
    housing.visual(
        wall_plate_mesh,
        material=plate_white,
        name="wall_plate",
    )
    housing.visual(
        Cylinder(radius=0.043, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=body_graphite,
        name="rear_can",
    )
    housing.visual(
        Cylinder(radius=0.0235, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=body_graphite,
        name="inner_support",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=face_glass,
        name="display_face",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.120, 0.120, 0.040)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    dial = model.part("dial")
    dial_ring_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0265, -0.0070),
                (0.0375, -0.0070),
                (0.0396, -0.0052),
                (0.0410, -0.0012),
                (0.0410, 0.0038),
                (0.0388, 0.0066),
                (0.0290, 0.0066),
                (0.0272, 0.0044),
                (0.0265, 0.0012),
            ],
            segments=80,
        ),
        "thermostat_dial_ring",
    )
    dial.visual(
        dial_ring_mesh,
        material=ring_steel,
        name="dial_bezel",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.014),
        mass=0.12,
    )

    model.articulation(
        "dial_rotation",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
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
    housing = object_model.get_part("housing")
    dial = object_model.get_part("dial")
    spin = object_model.get_articulation("dial_rotation")

    ctx.expect_origin_distance(
        dial,
        housing,
        axes="xy",
        max_dist=1e-6,
        name="dial shares the thermostat centerline",
    )
    ctx.expect_within(
        dial,
        housing,
        axes="xy",
        inner_elem="dial_bezel",
        outer_elem="rear_can",
        margin=0.0025,
        name="dial ring stays closely wrapped by the fixed body",
    )
    ctx.expect_gap(
        dial,
        housing,
        axis="z",
        positive_elem="dial_bezel",
        negative_elem="rear_can",
        min_gap=0.0,
        max_gap=0.0025,
        name="dial ring seats tightly against the rear body shoulder",
    )

    rest_pos = ctx.part_world_position(dial)
    with ctx.pose({spin: math.pi * 0.75}):
        turned_pos = ctx.part_world_position(dial)
        ctx.expect_origin_distance(
            dial,
            housing,
            axes="xy",
            max_dist=1e-6,
            name="dial remains centered while rotated",
        )
    ctx.check(
        "dial rotates in place",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(a - b) <= 1e-6 for a, b in zip(rest_pos, turned_pos)),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
