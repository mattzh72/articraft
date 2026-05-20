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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_studio_spotlight")

    powder_coat = model.material("powder_coat", rgba=(0.19, 0.20, 0.22, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.28, 0.30, 0.32, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))
    stainless = model.material("stainless", rgba=(0.70, 0.72, 0.75, 1.0))
    glass = model.material("glass", rgba=(0.52, 0.63, 0.70, 0.45))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    body_shell_mesh = _mesh(
        "spotlight_body_shell",
        CylinderGeometry(radius=0.086, height=0.205, radial_segments=40, closed=False),
    )
    body_liner_mesh = _mesh(
        "spotlight_body_liner",
        CylinderGeometry(radius=0.076, height=0.190, radial_segments=40, closed=False),
    )
    visor_shell_mesh = _mesh(
        "spotlight_visor_shell",
        CylinderGeometry(radius=0.097, height=0.058, radial_segments=40, closed=False),
    )
    glare_tube_mesh = _mesh(
        "spotlight_glare_tube",
        CylinderGeometry(radius=0.070, height=0.070, radial_segments=32, closed=False),
    )

    stand = model.part("stand_yoke")
    stand.visual(
        Cylinder(radius=0.22, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=powder_coat,
        name="base_plate",
    )
    stand.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=housing_gray,
        name="base_hub",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=powder_coat,
        name="mast",
    )
    stand.visual(
        Box((0.090, 0.220, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.428)),
        material=housing_gray,
        name="yoke_block",
    )
    stand.visual(
        Box((0.050, 0.014, 0.300)),
        origin=Origin(xyz=(0.0, 0.116, 0.558)),
        material=powder_coat,
        name="left_yoke_arm",
    )
    stand.visual(
        Box((0.050, 0.014, 0.300)),
        origin=Origin(xyz=(0.0, -0.116, 0.558)),
        material=powder_coat,
        name="right_yoke_arm",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.127, 0.578), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="left_lock_rosette",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, -0.127, 0.578), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="right_lock_rosette",
    )
    stand.visual(
        Cylinder(radius=0.021, length=0.030),
        origin=Origin(xyz=(0.0, 0.146, 0.578), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="left_lock_knob",
    )
    stand.visual(
        Cylinder(radius=0.021, length=0.030),
        origin=Origin(xyz=(0.0, -0.146, 0.578), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="right_lock_knob",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.155, 0.0, 0.015)),
        material=rubber,
        name="front_foot",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(-0.120, 0.115, 0.015)),
        material=rubber,
        name="rear_left_foot",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(-0.120, -0.115, 0.015)),
        material=rubber,
        name="rear_right_foot",
    )
    stand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.64),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
    )

    head = model.part("spotlight_head")
    head.visual(
        body_shell_mesh,
        origin=Origin(xyz=(-0.005, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_gray,
        name="body_shell",
    )
    head.visual(
        body_liner_mesh,
        origin=Origin(xyz=(-0.007, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gasket_black,
        name="body_liner",
    )
    head.visual(
        Cylinder(radius=0.092, length=0.016),
        origin=Origin(xyz=(-0.099, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="rear_service_ring",
    )
    head.visual(
        Box((0.100, 0.120, 0.082)),
        origin=Origin(xyz=(-0.085, 0.0, 0.105)),
        material=housing_gray,
        name="driver_box",
    )
    head.visual(
        Box((0.112, 0.132, 0.014)),
        origin=Origin(xyz=(-0.078, 0.0, 0.153)),
        material=powder_coat,
        name="driver_box_lid",
    )
    head.visual(
        Box((0.016, 0.050, 0.044)),
        origin=Origin(xyz=(-0.143, 0.0, 0.090)),
        material=housing_gray,
        name="gland_base",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.034),
        origin=Origin(xyz=(-0.168, 0.0, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gasket_black,
        name="cable_gland",
    )
    head.visual(
        Cylinder(radius=0.096, length=0.024),
        origin=Origin(xyz=(0.089, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="front_bezel",
    )
    head.visual(
        visor_shell_mesh,
        origin=Origin(xyz=(0.124, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="visor_shell",
    )
    head.visual(
        Box((0.050, 0.150, 0.012)),
        origin=Origin(xyz=(0.110, 0.0, 0.114)),
        material=powder_coat,
        name="visor_rain_hood",
    )
    head.visual(
        glare_tube_mesh,
        origin=Origin(xyz=(0.094, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gasket_black,
        name="glare_tube",
    )
    head.visual(
        Cylinder(radius=0.082, length=0.004),
        origin=Origin(xyz=(0.108, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.0, 0.097, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.0, -0.097, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="right_trunnion",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.090, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="left_trunnion_seal",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, -0.090, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="right_trunnion_seal",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.320, 0.220, 0.260)),
        mass=7.0,
        origin=Origin(xyz=(0.010, 0.0, 0.035)),
    )

    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.578)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=-0.40,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand_yoke")
    head = object_model.get_part("spotlight_head")
    tilt = object_model.get_articulation("yoke_tilt")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(
        head,
        stand,
        elem_a="left_trunnion",
        elem_b="left_yoke_arm",
        name="left trunnion seated in left yoke arm",
    )
    ctx.expect_contact(
        head,
        stand,
        elem_a="right_trunnion",
        elem_b="right_yoke_arm",
        name="right trunnion seated in right yoke arm",
    )

    upper = 0.95
    if tilt.motion_limits is not None and tilt.motion_limits.upper is not None:
        upper = tilt.motion_limits.upper

    with ctx.pose({tilt: 0.0}):
        level_lens = ctx.part_element_world_aabb(head, elem="front_lens")

    with ctx.pose({tilt: upper}):
        aimed_lens = ctx.part_element_world_aabb(head, elem="front_lens")
        aimed_rear = ctx.part_element_world_aabb(head, elem="rear_service_ring")

    if level_lens is not None and aimed_lens is not None:
        level_z = 0.5 * (level_lens[0][2] + level_lens[1][2])
        aimed_z = 0.5 * (aimed_lens[0][2] + aimed_lens[1][2])
        ctx.check(
            "positive tilt raises spotlight nose",
            aimed_z > level_z + 0.050,
            details=f"front lens z moved from {level_z:.3f} m to {aimed_z:.3f} m",
        )

    if aimed_lens is not None and aimed_rear is not None:
        aimed_lens_z = 0.5 * (aimed_lens[0][2] + aimed_lens[1][2])
        aimed_rear_z = 0.5 * (aimed_rear[0][2] + aimed_rear[1][2])
        ctx.check(
            "upper tilt aims beam above rear housing",
            aimed_lens_z > aimed_rear_z + 0.060,
            details=(
                f"front lens z={aimed_lens_z:.3f} m, "
                f"rear housing z={aimed_rear_z:.3f} m"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
