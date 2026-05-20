from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

    warm_white = model.material("warm_white", rgba=(0.95, 0.95, 0.93, 1.0))
    matte_white = model.material("matte_white", rgba=(0.90, 0.90, 0.88, 1.0))
    champagne = model.material("champagne", rgba=(0.82, 0.79, 0.72, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.18, 0.22, 0.24, 0.80))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.74, 1.0))

    body = model.part("thermostat_body")

    plate_profile = rounded_rect_profile(0.132, 0.132, 0.014, corner_segments=10)
    wall_plate = mesh_from_geometry(
        ExtrudeGeometry.from_z0(plate_profile, 0.004),
        "thermostat_wall_plate",
    )
    body.visual(wall_plate, material=warm_white, name="wall_plate")
    body.visual(
        Cylinder(radius=0.0363, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=matte_white,
        name="body_core",
    )
    body.visual(
        Cylinder(radius=0.0285, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.02175)),
        material=charcoal,
        name="inner_face",
    )
    body.visual(
        Cylinder(radius=0.0245, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_glass,
        name="display_lens",
    )
    body.visual(
        Box((0.008, 0.0014, 0.0016)),
        origin=Origin(xyz=(0.0, 0.020, 0.0288)),
        material=satin_steel,
        name="index_mark",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.132, 0.132, 0.030)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    dial = model.part("dial_ring")

    dial_outer_profile = [
        (0.0435, -0.0030),
        (0.0435, 0.0075),
        (0.0424, 0.0105),
        (0.0408, 0.0120),
    ]
    dial_inner_profile = [
        (0.0363, -0.0030),
        (0.0363, 0.0015),
        (0.0330, 0.0060),
        (0.0292, 0.0120),
    ]
    dial_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            dial_outer_profile,
            dial_inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "thermostat_dial_ring",
    )
    dial.visual(dial_shell, material=champagne, name="dial_shell")
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0435, length=0.015),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
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
    body = object_model.get_part("thermostat_body")
    dial = object_model.get_part("dial_ring")
    joint = object_model.get_articulation("body_to_dial")

    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        min_overlap=0.075,
        name="dial stays centered over the thermostat face",
    )

    rest_pos = ctx.part_world_position(dial)
    with ctx.pose({joint: 2.4}):
        turned_pos = ctx.part_world_position(dial)

    ctx.check(
        "dial rotates in place",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(a - b) <= 1e-6 for a, b in zip(rest_pos, turned_pos)),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )
    ctx.check(
        "dial articulation is continuous",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and joint.motion_limits is not None
        and joint.motion_limits.lower is None
        and joint.motion_limits.upper is None,
        details=(
            f"type={joint.articulation_type}, "
            f"limits={joint.motion_limits}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
