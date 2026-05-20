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


def _build_wall_plate_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.138, 0.138, 0.014, corner_segments=10),
            0.005,
            cap=True,
            closed=True,
        ),
        "thermostat_wall_plate",
    )


def _build_dial_ring_mesh():
    outer_profile = [
        (0.036, 0.000),
        (0.039, 0.002),
        (0.042, 0.006),
        (0.041, 0.011),
        (0.0375, 0.013),
    ]
    inner_profile = [
        (0.029, 0.000),
        (0.0305, 0.002),
        (0.0325, 0.006),
        (0.0315, 0.011),
        (0.0295, 0.013),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "thermostat_dial_ring",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    plate_white = model.material("plate_white", rgba=(0.96, 0.96, 0.95, 1.0))
    body_satin = model.material("body_satin", rgba=(0.86, 0.87, 0.88, 1.0))
    bezel_warm = model.material("bezel_warm", rgba=(0.79, 0.78, 0.76, 1.0))
    hub_glass = model.material("hub_glass", rgba=(0.10, 0.11, 0.12, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        _build_wall_plate_mesh(),
        material=plate_white,
        name="plate_shell",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.138, 0.138, 0.005)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
    )

    rear_body = model.part("rear_body")
    rear_body.visual(
        Cylinder(radius=0.043, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=body_satin,
        name="rear_drum",
    )
    rear_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.043, length=0.013),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
    )
    model.articulation(
        "plate_to_rear_body",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=rear_body,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    mid_body = model.part("mid_body")
    mid_body.visual(
        Cylinder(radius=0.026, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=body_satin,
        name="mid_section",
    )
    mid_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.005),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
    )
    model.articulation(
        "rear_body_to_mid_body",
        ArticulationType.FIXED,
        parent=rear_body,
        child=mid_body,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    center_hub = model.part("center_hub")
    center_hub.visual(
        Cylinder(radius=0.0205, length=0.0085),
        origin=Origin(xyz=(0.0, 0.0, 0.00425)),
        material=hub_glass,
        name="hub_face",
    )
    center_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0205, length=0.0085),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.00425)),
    )
    model.articulation(
        "mid_body_to_center_hub",
        ArticulationType.FIXED,
        parent=mid_body,
        child=center_hub,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    dial_ring = model.part("dial_ring")
    dial_ring.visual(
        _build_dial_ring_mesh(),
        material=bezel_warm,
        name="dial_outer_ring",
    )
    dial_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.042, length=0.013),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
    )
    model.articulation(
        "rear_body_to_dial_ring",
        ArticulationType.CONTINUOUS,
        parent=rear_body,
        child=dial_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    rear_body = object_model.get_part("rear_body")
    mid_body = object_model.get_part("mid_body")
    center_hub = object_model.get_part("center_hub")
    dial_ring = object_model.get_part("dial_ring")
    dial_joint = object_model.get_articulation("rear_body_to_dial_ring")

    ctx.expect_gap(
        rear_body,
        wall_plate,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        name="rear body mounts flush to wall plate",
    )
    ctx.expect_within(
        mid_body,
        rear_body,
        axes="xy",
        margin=0.0,
        name="mid section stays within rear body footprint",
    )
    ctx.expect_within(
        center_hub,
        mid_body,
        axes="xy",
        margin=0.0,
        name="center hub stays within mid section footprint",
    )
    ctx.expect_gap(
        dial_ring,
        rear_body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        name="dial ring seats on the front body face",
    )
    ctx.expect_overlap(
        dial_ring,
        rear_body,
        axes="xy",
        min_overlap=0.072,
        name="dial ring shares the rear body center footprint",
    )

    rest_pos = ctx.part_world_position(dial_ring)
    with ctx.pose({dial_joint: 1.7}):
        spun_pos = ctx.part_world_position(dial_ring)
        ctx.expect_origin_distance(
            dial_ring,
            rear_body,
            axes="xy",
            max_dist=1e-6,
            name="dial remains centered while spinning",
        )

    ctx.check(
        "dial rotates about its own center axis",
        rest_pos is not None
        and spun_pos is not None
        and max(abs(spun_pos[index] - rest_pos[index]) for index in range(3)) <= 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
