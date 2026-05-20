from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wall_thermostat", assets=ASSETS)

    warm_plate = model.material("warm_plate", rgba=(0.88, 0.89, 0.87, 1.0))
    matte_body = model.material("matte_body", rgba=(0.80, 0.82, 0.80, 1.0))
    satin_ring = model.material("satin_ring", rgba=(0.63, 0.66, 0.70, 1.0))
    dial_graphite = model.material("dial_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.16, 0.20, 0.23, 0.85))
    soft_shadow = model.material("soft_shadow", rgba=(0.20, 0.21, 0.22, 1.0))
    fastener_metal = model.material("fastener_metal", rgba=(0.60, 0.62, 0.65, 1.0))

    def save_mesh(geometry, filename: str):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))

    def ring_shell(
        *,
        outer_profile: list[tuple[float, float]],
        inner_profile: list[tuple[float, float]],
        filename: str,
    ):
        return save_mesh(
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
            filename,
        )

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        save_mesh(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.142, 0.142, 0.018),
                0.006,
                cap=True,
                closed=True,
            ),
            "thermostat_wall_plate.obj",
        ),
        material=warm_plate,
        name="mount_plate",
    )
    wall_plate.visual(
        ring_shell(
            outer_profile=[(0.051, 0.0), (0.051, 0.0008)],
            inner_profile=[(0.0465, 0.0), (0.0465, 0.0008)],
            filename="thermostat_shadow_gasket.obj",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=soft_shadow,
        name="shadow_gasket",
    )
    wall_plate.visual(
        Cylinder(radius=0.0034, length=0.0008),
        origin=Origin(xyz=(0.056, 0.0, 0.0064)),
        material=fastener_metal,
        name="upper_fastener_cap",
    )
    wall_plate.visual(
        Cylinder(radius=0.0034, length=0.0008),
        origin=Origin(xyz=(-0.056, 0.0, 0.0064)),
        material=fastener_metal,
        name="lower_fastener_cap",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.142, 0.142, 0.006)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    body_core = model.part("body_core")
    body_core.visual(
        save_mesh(
            LatheGeometry(
                [
                    (0.0, 0.0),
                    (0.039, 0.0),
                    (0.044, 0.0015),
                    (0.0475, 0.0055),
                    (0.0475, 0.0115),
                    (0.045, 0.0145),
                    (0.041, 0.0160),
                    (0.0, 0.0160),
                ],
                segments=80,
            ),
            "thermostat_body_core.obj",
        ),
        material=matte_body,
        name="body_shell",
    )
    body_core.visual(
        Cylinder(radius=0.0495, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0006)),
        material=soft_shadow,
        name="rear_reveal_band",
    )
    body_core.inertial = Inertial.from_geometry(
        Cylinder(radius=0.048, length=0.016),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    model.articulation(
        "plate_to_body_core",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=body_core,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    face_ring = model.part("face_ring")
    face_ring.visual(
        ring_shell(
            outer_profile=[(0.0460, 0.0), (0.0467, 0.0012), (0.0460, 0.0040)],
            inner_profile=[(0.0365, 0.0), (0.0360, 0.0014), (0.0360, 0.0040)],
            filename="thermostat_face_ring.obj",
        ),
        material=satin_ring,
        name="dial_face_ring",
    )
    face_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.004),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    model.articulation(
        "body_core_to_face_ring",
        ArticulationType.FIXED,
        parent=body_core,
        child=face_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    axle_core = model.part("axle_core")
    axle_core.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=smoked_glass,
        name="center_disc",
    )
    axle_core.visual(
        Cylinder(radius=0.0042, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_ring,
        name="axis_pin",
    )
    axle_core.visual(
        Cylinder(radius=0.0095, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=satin_ring,
        name="support_sleeve",
    )
    axle_core.visual(
        Cylinder(radius=0.0105, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.0092)),
        material=smoked_glass,
        name="center_lens",
    )
    axle_core.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.010),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    model.articulation(
        "body_core_to_axle_core",
        ArticulationType.FIXED,
        parent=body_core,
        child=axle_core,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    dial_knob = model.part("dial_knob")
    dial_knob.visual(
        ring_shell(
            outer_profile=[
                (0.0310, 0.0),
                (0.0335, 0.0020),
                (0.0340, 0.0070),
                (0.0330, 0.0120),
                (0.0300, 0.0140),
            ],
            inner_profile=[
                (0.0170, 0.0),
                (0.0170, 0.0060),
                (0.0165, 0.0120),
                (0.0160, 0.0140),
            ],
            filename="thermostat_dial_knob.obj",
        ),
        material=dial_graphite,
        name="dial_shell",
    )
    dial_knob.visual(
        Cylinder(radius=0.002, length=0.010),
        origin=Origin(xyz=(0.0, 0.023, 0.013), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_ring,
        name="indicator_ridge",
    )
    dial_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.014),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    model.articulation(
        "body_core_to_dial_knob",
        ArticulationType.CONTINUOUS,
        parent=body_core,
        child=dial_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    wall_plate = object_model.get_part("wall_plate")
    body_core = object_model.get_part("body_core")
    face_ring = object_model.get_part("face_ring")
    axle_core = object_model.get_part("axle_core")
    dial_knob = object_model.get_part("dial_knob")
    dial_joint = object_model.get_articulation("body_core_to_dial_knob")
    mount_plate = wall_plate.get_visual("mount_plate")
    body_shell = body_core.get_visual("body_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(max_pose_samples=10)
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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=12,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.expect_gap(
        body_core,
        wall_plate,
        axis="z",
        max_gap=0.0001,
        max_penetration=0.0,
        positive_elem=body_shell,
        negative_elem=mount_plate,
    )
    ctx.expect_gap(face_ring, body_core, axis="z", max_gap=0.0001, max_penetration=0.0)
    ctx.expect_gap(axle_core, body_core, axis="z", max_gap=0.0001, max_penetration=0.0)
    ctx.expect_gap(dial_knob, body_core, axis="z", max_gap=0.0001, max_penetration=0.0)

    ctx.expect_contact(body_core, wall_plate)
    ctx.expect_contact(face_ring, body_core)
    ctx.expect_contact(axle_core, body_core)
    ctx.expect_contact(dial_knob, body_core)

    ctx.expect_overlap(body_core, wall_plate, axes="xy", min_overlap=0.090)
    ctx.expect_within(face_ring, body_core, axes="xy", margin=0.001)
    ctx.expect_within(dial_knob, body_core, axes="xy", margin=0.001)
    ctx.expect_origin_distance(dial_knob, axle_core, axes="xy", max_dist=0.0001)

    def aabb_center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    dial_rest = ctx.part_world_position(dial_knob)
    axle_rest = ctx.part_world_position(axle_core)
    ridge_rest_aabb = ctx.part_element_world_aabb(dial_knob, elem="indicator_ridge")
    assert dial_rest is not None
    assert axle_rest is not None
    assert ridge_rest_aabb is not None
    ridge_rest = aabb_center(ridge_rest_aabb)
    ctx.check(
        "dial_and_axle_coaxial_rest",
        abs(dial_rest[0] - axle_rest[0]) <= 1e-6 and abs(dial_rest[1] - axle_rest[1]) <= 1e-6,
        details=f"dial={dial_rest}, axle={axle_rest}",
    )
    ctx.check(
        "dial_indicator_rest_pose",
        abs(ridge_rest[0]) < 0.004 and ridge_rest[1] > 0.018,
        details=f"indicator center at rest={ridge_rest}",
    )

    with ctx.pose({dial_joint: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="dial_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(max_pose_samples=1, name="dial_quarter_turn_no_floating")
        ctx.expect_contact(dial_knob, body_core, name="dial_quarter_turn_contact")
        dial_turned = ctx.part_world_position(dial_knob)
        ridge_turned_aabb = ctx.part_element_world_aabb(dial_knob, elem="indicator_ridge")
        assert dial_turned is not None
        assert ridge_turned_aabb is not None
        ridge_turned = aabb_center(ridge_turned_aabb)
        ctx.check(
            "dial_center_stable_in_rotation",
            all(abs(a - b) <= 1e-6 for a, b in zip(dial_rest, dial_turned)),
            details=f"rest={dial_rest}, turned={dial_turned}",
        )
        ctx.check(
            "dial_indicator_rotates_about_z",
            abs(ridge_turned[1]) < 0.004
            and abs(ridge_turned[0]) > 0.018
            and abs(ridge_turned[2] - ridge_rest[2]) < 0.001,
            details=f"rest={ridge_rest}, turned={ridge_turned}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
