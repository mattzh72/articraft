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
    model = ArticulatedObject(name="wall_thermostat_with_large_dial")

    plate_white = model.material("plate_white", rgba=(0.95, 0.95, 0.93, 1.0))
    shell_white = model.material("shell_white", rgba=(0.92, 0.92, 0.90, 1.0))
    dial_silver = model.material("dial_silver", rgba=(0.77, 0.79, 0.81, 1.0))
    face_dark = model.material("face_dark", rgba=(0.20, 0.23, 0.25, 1.0))
    marker_dark = model.material("marker_dark", rgba=(0.18, 0.18, 0.18, 1.0))

    plate_profile = rounded_rect_profile(0.140, 0.140, 0.014, corner_segments=10)
    plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(plate_profile, 0.0052, cap=True, closed=True),
        "thermostat_wall_plate",
    )

    dial_ring_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0480, -0.0070),
                (0.0500, -0.0045),
                (0.0500, 0.0048),
                (0.0480, 0.0070),
            ],
            [
                (0.0400, -0.0070),
                (0.0403, -0.0040),
                (0.0403, 0.0040),
                (0.0400, 0.0070),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "thermostat_dial_ring",
    )

    body = model.part("body")
    body.visual(plate_mesh, material=plate_white, name="wall_plate")
    body.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0150)),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.048, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.0200)),
        material=shell_white,
        name="dial_seat",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0240)),
        material=shell_white,
        name="face_mount",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0300)),
        material=face_dark,
        name="center_face",
    )
    body.visual(
        Box((0.004, 0.016, 0.0020)),
        origin=Origin(xyz=(0.0, 0.020, 0.0320)),
        material=marker_dark,
        name="screen_mark",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.140, 0.140, 0.040)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    dial = model.part("dial")
    dial.visual(dial_ring_mesh, material=dial_silver, name="dial_ring")
    dial.visual(
        Box((0.010, 0.0032, 0.0024)),
        origin=Origin(xyz=(0.0, 0.0435, 0.0066)),
        material=marker_dark,
        name="dial_marker",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.014),
        mass=0.08,
        origin=Origin(),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.0280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
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

    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.expect_origin_distance(
        dial,
        body,
        axes="xy",
        max_dist=1e-6,
        name="dial axis stays centered on thermostat body",
    )
    ctx.expect_origin_gap(
        dial,
        body,
        axis="z",
        min_gap=0.026,
        max_gap=0.030,
        name="dial sits proud of the wall plate",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        elem_a="dial_ring",
        elem_b="body_shell",
        min_overlap=0.070,
        name="dial ring remains centered over the shallow body",
    )

    rest_pos = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: 1.8}):
        ctx.expect_origin_distance(
            dial,
            body,
            axes="xy",
            max_dist=1e-6,
            name="dial stays centered while rotated",
        )
        turned_pos = ctx.part_world_position(dial)
    ctx.check(
        "continuous dial rotates in place",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
