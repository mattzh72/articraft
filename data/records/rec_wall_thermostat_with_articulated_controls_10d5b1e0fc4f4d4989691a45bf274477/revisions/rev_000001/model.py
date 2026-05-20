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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat_dial")

    wall_plate_white = model.material("wall_plate_white", rgba=(0.94, 0.94, 0.93, 1.0))
    body_white = model.material("body_white", rgba=(0.96, 0.96, 0.95, 1.0))
    dial_satin = model.material("dial_satin", rgba=(0.72, 0.74, 0.76, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.14, 0.16, 0.18, 0.70))

    plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.128, 0.128, 0.014, corner_segments=8),
            0.005,
            cap=True,
            closed=True,
        ),
        "thermostat_wall_plate",
    )
    dial_ring_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0415, 0.0000),
                (0.0435, 0.0015),
                (0.0435, 0.0048),
                (0.0418, 0.0060),
            ],
            [
                (0.0290, 0.0006),
                (0.0310, 0.0017),
                (0.0310, 0.0046),
                (0.0292, 0.0054),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "thermostat_dial_ring",
    )
    outer_bezel_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.0408, tube=0.0027, radial_segments=18, tubular_segments=56),
        "thermostat_outer_bezel",
    )

    housing = model.part("housing")
    housing.visual(plate_mesh, material=wall_plate_white, name="wall_plate")
    housing.visual(
        Cylinder(radius=0.047, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=body_white,
        name="rear_body",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=body_white,
        name="center_face",
    )
    housing.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=graphite,
        name="bearing_collar",
    )
    housing.visual(
        Cylinder(radius=0.0165, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.02425)),
        material=smoked_glass,
        name="display_lens",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.128, 0.128, 0.026)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    dial = model.part("dial")
    dial.visual(
        dial_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0020)),
        material=dial_satin,
        name="dial_ring",
    )
    dial.visual(
        outer_bezel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=dial_satin,
        name="outer_bezel",
    )
    dial.visual(
        Box((0.004, 0.012, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0365, 0.0084)),
        material=graphite,
        name="dial_tick",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.044, length=0.009),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    model.articulation(
        "housing_to_dial",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("housing_to_dial")

    ctx.expect_within(
        dial,
        housing,
        axes="xy",
        inner_elem="dial_ring",
        outer_elem="rear_body",
        margin=0.004,
        name="dial ring stays within the body footprint",
    )
    ctx.expect_contact(
        dial,
        housing,
        elem_a="dial_ring",
        elem_b="rear_body",
        contact_tol=1e-6,
        name="dial ring seats cleanly on the body shoulder",
    )

    rest_pos = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: 1.4}):
        turned_pos = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates without translating",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) <= 1e-6
        and abs(rest_pos[1] - turned_pos[1]) <= 1e-6
        and abs(rest_pos[2] - turned_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )
    ctx.check(
        "axis hardware stays tucked into the body depth",
        0.005 < dial_joint.origin.xyz[2] < 0.020,
        details=f"joint origin z={dial_joint.origin.xyz[2]}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
