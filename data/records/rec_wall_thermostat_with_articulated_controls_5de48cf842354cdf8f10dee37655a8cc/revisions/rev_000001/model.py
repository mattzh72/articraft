from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    model = ArticulatedObject(name="wall_thermostat_large_dial")

    plate_white = model.material("plate_white", rgba=(0.95, 0.95, 0.96, 1.0))
    body_white = model.material("body_white", rgba=(0.86, 0.87, 0.89, 1.0))
    ring_metal = model.material("ring_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    face_dark = model.material("face_dark", rgba=(0.16, 0.18, 0.20, 1.0))

    plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.148, 0.148, 0.022, corner_segments=10),
            0.006,
            cap=True,
            closed=True,
        ),
        "thermostat_wall_plate",
    )

    dial_ring_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.058, 0.010),
                (0.062, 0.0115),
                (0.064, 0.0165),
                (0.063, 0.0215),
                (0.059, 0.024),
            ],
            inner_profile=[
                (0.0475, 0.011),
                (0.0495, 0.0155),
                (0.0495, 0.0205),
                (0.0475, 0.023),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
        ),
        "thermostat_dial_ring",
    )
    dial_support_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.059, 0.006),
                (0.060, 0.011),
            ],
            inner_profile=[
                (0.052, 0.006),
                (0.053, 0.011),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
        ),
        "thermostat_dial_support",
    )

    base = model.part("base")
    base.visual(
        plate_mesh,
        material=plate_white,
        name="wall_plate",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=body_white,
        name="thermostat_body",
    )
    base.visual(
        Cylinder(radius=0.037, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=face_dark,
        name="center_face",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.074, length=0.024),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    dial = model.part("dial")
    dial.visual(
        dial_ring_mesh,
        material=ring_metal,
        name="dial_ring",
    )
    dial.visual(
        dial_support_mesh,
        material=ring_metal,
        name="rear_bearing_skirt",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.064, length=0.014),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=8.0,
        ),
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
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("base_to_dial")

    ctx.expect_origin_distance(
        dial,
        base,
        axes="xy",
        max_dist=0.0005,
        name="dial is centered on the thermostat body",
    )
    ctx.expect_gap(
        dial,
        base,
        axis="z",
        positive_elem="dial_ring",
        negative_elem="wall_plate",
        min_gap=0.003,
        max_gap=0.020,
        name="dial ring stands proud of the wall plate",
    )
    ctx.expect_overlap(
        dial,
        base,
        axes="xy",
        elem_a="dial_ring",
        elem_b="thermostat_body",
        min_overlap=0.090,
        name="dial ring remains concentric over the shallow body",
    )

    rest_pos = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(dial)
        ctx.expect_origin_distance(
            dial,
            base,
            axes="xy",
            max_dist=0.0005,
            name="dial stays centered while turned",
        )

    ctx.check(
        "dial rotates about a fixed center axis",
        rest_pos is not None
        and turned_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, turned_pos)) <= 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
