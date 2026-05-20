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

    wall_white = model.material("wall_white", rgba=(0.94, 0.94, 0.93, 1.0))
    shell_white = model.material("shell_white", rgba=(0.96, 0.95, 0.93, 1.0))
    dial_graphite = model.material("dial_graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    metallic_edge = model.material("metallic_edge", rgba=(0.70, 0.72, 0.75, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.08, 0.10, 0.12, 0.85))
    marker_dark = model.material("marker_dark", rgba=(0.18, 0.18, 0.18, 1.0))

    thermostat_body = model.part("thermostat_body")

    wall_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.148, 0.148, 0.018, corner_segments=8),
            0.004,
            cap=True,
            closed=True,
        ),
        "thermostat_wall_plate",
    )
    thermostat_body.visual(
        wall_plate_mesh,
        material=wall_white,
        name="wall_plate",
    )
    thermostat_body.visual(
        Cylinder(radius=0.056, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=shell_white,
        name="rear_pedestal",
    )
    thermostat_body.visual(
        Cylinder(radius=0.045, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=shell_white,
        name="bearing_shoulder",
    )
    thermostat_body.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=shell_white,
        name="center_core",
    )
    thermostat_body.visual(
        Box((0.024, 0.007, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0248)),
        material=glass_dark,
        name="display_lens",
    )
    thermostat_body.visual(
        Box((0.010, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.044, 0.0135)),
        material=marker_dark,
        name="top_index_mark",
    )
    thermostat_body.inertial = Inertial.from_geometry(
        Box((0.148, 0.148, 0.026)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    dial = model.part("dial")

    dial_collar_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.060, 0.000),
                (0.060, 0.010),
                (0.058, 0.016),
            ],
            inner_profile=[
                (0.036, 0.000),
                (0.037, 0.010),
                (0.039, 0.016),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "dial_collar",
    )
    dial_bezel_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.063, 0.000),
                (0.062, 0.004),
                (0.059, 0.008),
            ],
            inner_profile=[
                (0.036, 0.000),
                (0.036, 0.004),
                (0.038, 0.008),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "dial_bezel",
    )
    dial.visual(
        dial_collar_mesh,
        material=dial_graphite,
        name="dial_collar",
    )
    dial.visual(
        dial_bezel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=metallic_edge,
        name="dial_bezel",
    )

    grip_radius = 0.0615
    for rib_index in range(18):
        angle = (2.0 * math.pi * rib_index) / 18.0
        dial.visual(
            Box((0.0035, 0.006, 0.010)),
            origin=Origin(
                xyz=(
                    grip_radius * math.cos(angle),
                    grip_radius * math.sin(angle),
                    0.014,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=dial_graphite,
            name=f"grip_rib_{rib_index:02d}",
        )

    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.063, length=0.022),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=thermostat_body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
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

    thermostat_body = object_model.get_part("thermostat_body")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("body_to_dial")

    limits = dial_joint.motion_limits
    ctx.check(
        "dial uses continuous center-axis rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(dial_joint.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={dial_joint.articulation_type}, axis={dial_joint.axis}, "
            f"limits={limits}"
        ),
    )
    ctx.expect_origin_distance(
        dial,
        thermostat_body,
        axes="xy",
        max_dist=0.001,
        name="dial stays centered on thermostat body",
    )
    ctx.expect_gap(
        dial,
        thermostat_body,
        axis="z",
        positive_elem="dial_collar",
        negative_elem="bearing_shoulder",
        max_gap=0.001,
        max_penetration=0.0,
        name="dial collar seats flush on shoulder",
    )
    ctx.expect_gap(
        dial,
        thermostat_body,
        axis="z",
        positive_elem="dial_bezel",
        negative_elem="center_core",
        min_gap=0.0015,
        max_gap=0.012,
        name="front bezel stands proud of fixed center core",
    )
    ctx.expect_overlap(
        dial,
        thermostat_body,
        axes="xy",
        elem_a="dial_bezel",
        elem_b="center_core",
        min_overlap=0.050,
        name="dial ring reads concentric with central core",
    )

    with ctx.pose({dial_joint: 1.4}):
        ctx.expect_origin_distance(
            dial,
            thermostat_body,
            axes="xy",
            max_dist=0.001,
            name="dial remains centered while rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
