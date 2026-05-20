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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _x_cylinder_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _revolved_shell(name: str, outer_profile, inner_profile):
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_syringe")

    barrel_clear = model.material("barrel_clear", rgba=(0.80, 0.86, 0.92, 0.34))
    hardware_dark = model.material("hardware_dark", rgba=(0.23, 0.25, 0.27, 1.0))
    hardware_mid = model.material("hardware_mid", rgba=(0.46, 0.49, 0.53, 1.0))
    seal_black = model.material("seal_black", rgba=(0.06, 0.06, 0.07, 1.0))
    scale_ink = model.material("scale_ink", rgba=(0.12, 0.12, 0.12, 1.0))
    grip_polymer = model.material("grip_polymer", rgba=(0.32, 0.17, 0.08, 1.0))

    body = model.part("body")
    plunger = model.part("plunger")

    body.visual(
        _revolved_shell(
            "rear_guide_sleeve",
            outer_profile=[(0.0120, 0.0160), (0.0120, 0.0260)],
            inner_profile=[(0.0046, 0.0160), (0.0046, 0.0260)],
        ),
        material=hardware_mid,
        name="rear_guide_sleeve",
    )

    body.visual(
        _revolved_shell(
            "forward_stop_ring",
            outer_profile=[(0.0180, 0.1380), (0.0180, 0.1480)],
            inner_profile=[(0.0046, 0.1380), (0.0046, 0.1480)],
        ),
        material=hardware_mid,
        name="forward_stop_ring",
    )

    body.visual(
        _revolved_shell(
            "barrel_rear_collar",
            outer_profile=[(0.0240, 0.1120), (0.0240, 0.1280)],
            inner_profile=[(0.0184, 0.1120), (0.0184, 0.1280)],
        ),
        material=hardware_dark,
        name="barrel_rear_collar",
    )

    body.visual(
        _revolved_shell(
            "barrel_shell",
            outer_profile=[(0.0192, 0.1200), (0.0192, 0.2240)],
            inner_profile=[(0.0162, 0.1200), (0.0162, 0.2240)],
        ),
        material=barrel_clear,
        name="barrel_shell",
    )

    body.visual(
        _revolved_shell(
            "front_nose",
            outer_profile=[
                (0.0192, 0.2240),
                (0.0188, 0.2290),
                (0.0105, 0.2440),
                (0.0065, 0.2520),
                (0.0042, 0.2700),
            ],
            inner_profile=[
                (0.0158, 0.2240),
                (0.0148, 0.2290),
                (0.0060, 0.2440),
                (0.0026, 0.2520),
                (0.0016, 0.2700),
            ],
        ),
        material=hardware_mid,
        name="front_nose",
    )

    rail_length = 0.126
    rail_center_x = 0.083
    for side, y in (("left", 0.022), ("right", -0.022)):
        body.visual(
            Box((rail_length, 0.010, 0.012)),
            origin=Origin(xyz=(rail_center_x, y, -0.014)),
            material=hardware_dark,
            name=f"{side}_service_rail",
        )
        body.visual(
            Box((0.014, 0.020, 0.018)),
            origin=Origin(xyz=(0.022, y, -0.009)),
            material=hardware_mid,
            name=f"{side}_rear_bridge",
        )
        body.visual(
            Box((0.024, 0.014, 0.028)),
            origin=Origin(xyz=(0.120, 0.026 if side == "left" else -0.026, 0.0)),
            material=hardware_mid,
            name=f"{side}_front_clamp",
        )

    for side, y in (("left", 0.041), ("right", -0.041)):
        body.visual(
            Box((0.018, 0.040, 0.006)),
            origin=Origin(xyz=(0.118, y, -0.012)),
            material=grip_polymer,
            name=f"{side}_finger_wing",
        )

    for idx, x_center in enumerate((0.145, 0.157, 0.169, 0.181, 0.193, 0.205), start=1):
        body.visual(
            _revolved_shell(
                f"graduation_band_{idx}",
                outer_profile=[(0.0198, x_center - 0.0005), (0.0198, x_center + 0.0005)],
                inner_profile=[(0.0191, x_center - 0.0005), (0.0191, x_center + 0.0005)],
            ),
            material=scale_ink,
            name=f"graduation_band_{idx}",
        )

    plunger.visual(
        Box((0.012, 0.060, 0.008)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=grip_polymer,
        name="thumb_bar",
    )
    plunger.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=_x_cylinder_origin(0.003),
        material=hardware_dark,
        name="thumb_boss",
    )
    plunger.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=_x_cylinder_origin(0.011),
        material=hardware_dark,
        name="rear_stop_collar",
    )
    plunger.visual(
        Cylinder(radius=0.0036, length=0.118),
        origin=_x_cylinder_origin(0.069),
        material=hardware_mid,
        name="rod",
    )
    plunger.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=_x_cylinder_origin(0.055),
        material=hardware_dark,
        name="forward_stop_collar",
    )
    plunger.visual(
        Cylinder(radius=0.0125, length=0.006),
        origin=_x_cylinder_origin(0.123),
        material=hardware_mid,
        name="seal_retainer",
    )
    plunger.visual(
        Cylinder(radius=0.0150, length=0.012),
        origin=_x_cylinder_origin(0.132),
        material=seal_black,
        name="seal",
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.35,
            lower=0.0,
            upper=0.078,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    stroke = object_model.get_articulation("body_to_plunger")

    rear_guide = body.get_visual("rear_guide_sleeve")
    forward_stop_ring = body.get_visual("forward_stop_ring")
    front_nose = body.get_visual("front_nose")
    rear_stop_collar = plunger.get_visual("rear_stop_collar")
    forward_stop_collar = plunger.get_visual("forward_stop_collar")
    rod = plunger.get_visual("rod")
    seal = plunger.get_visual("seal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    with ctx.pose({stroke: 0.0}):
        ctx.expect_contact(
            plunger,
            body,
            elem_a=rear_stop_collar,
            elem_b=rear_guide,
            name="rear stop collar seats on guide at rest",
        )
        ctx.expect_within(
            plunger,
            body,
            axes="yz",
            inner_elem=rod,
            outer_elem=rear_guide,
            margin=0.0005,
            name="rod stays coaxial within rear guide at rest",
        )

    with ctx.pose({stroke: 0.078}):
        ctx.expect_contact(
            plunger,
            body,
            elem_a=forward_stop_collar,
            elem_b=forward_stop_ring,
            name="forward stop collar hits service stop ring at full stroke",
        )
        ctx.expect_within(
            plunger,
            body,
            axes="yz",
            inner_elem=rod,
            outer_elem=rear_guide,
            margin=0.0005,
            name="rod stays coaxial within rear guide at full stroke",
        )
        ctx.expect_gap(
            body,
            plunger,
            axis="x",
            positive_elem=front_nose,
            negative_elem=seal,
            min_gap=0.006,
            max_gap=0.010,
            name="seal remains just behind front nose at full stroke",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
